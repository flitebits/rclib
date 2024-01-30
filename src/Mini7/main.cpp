// Copyright 2023 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "Boot.h"
#include "Dbg.h"
#include "IntTypes.h"
#include "RtcTime.h"

#include "leds/Clr.h"
#include "leds/FPMath.h"
#include "leds/LedSpan.h"
#include "leds/LedSpanOps.h"
#include "leds/Pixel.h"
#include "leds/Rgb.h"

#include "Adc.h"
#include "Pins.h"
#include "Pwm.h"
#include "SBus.h"
#include "Serial.h"
#include "SportSensor.h"
#include "WS2812.h"

using led::HSV;
using led::Logify;
using led::RGB;
using led::RGBW;
using led::bscale8;
using led::scale8;
using led::sin8;
using led::VariableSaw;

#if defined(__AVR_ATmega4809__)
#define RGB_LED_PIN (PIN_C2)
#endif

#define VOLT_PIN (PIN_E0)  // ANALOG pin 8, PE0

#define LIGHT_LEVEL_CH    (4)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH     (5)  // Sets Lights Mode (solid, spin, glow)
#define LIGHT_SUBMODE_CH  (6)  // Sets mode sub mode (color range, etc)
#define LIGHT_BRIGHT_CH   (7)  // Adjust leading edge light brightness.
#define LIGHT_THROTTLE_CH (8)  // Adjusts lights speed

#define NUM_PWM (1)
#define WING_CNT (9) // Each wing quarter (6 led, 3 led)

#define RGB_CNT (4 * WING_CNT)
#define RGBW_CNT (0)

u8_t led_data[RGBW_CNT * 4 + RGB_CNT * 3];

enum Modes {
            MODE_S0 = 0,
            MODE_S1 = 1,
            MODE_S2 = 2,
            MODE_S_MSK = 3,
            MODE_SOLID = 0 << 2,
            MODE_COLOR = 1 << 2,
            MODE_PULSE = 2 << 2,
            MODE_MSK   = 3 << 2,
};

struct CtrlState {
  enum  ChangeBits {
                    CHG_NONE = 0,
                    CHG_MODE = 1 << 0,
                    CHG_LVL  = 1 << 1,
                    CHG_BRT  = 1 << 2,
                    CHG_THR  = 1 << 3,
  };
  u8_t mode;
  u8_t level;
  i16_t brt;
  i16_t thr;

  // brt & thr are 0 -> 2047 nominal.
  CtrlState() : mode(0), level(0), brt(0), thr(0) { }
  CtrlState(u8_t mode, u8_t level, i16_t brt, i16_t thr)
    : mode(mode), level(level), brt(brt), thr(thr) { }
  u8_t Mode() const { return mode >> 2; }
  u8_t Submode() const { return mode & 0x03; }
  u8_t Set(SBus* sbus) {
    u8_t mode = ((SBus::ThreePosSwitch(sbus->GetChannel(LIGHT_MODE_CH)) << 2) |
                 SBus::ThreePosSwitch(sbus->GetChannel(LIGHT_SUBMODE_CH)));
    CtrlState new_state(mode,
                        SBus::ThreePosSwitch(sbus->GetChannel(LIGHT_LEVEL_CH)),
                        sbus->GetChannel(LIGHT_BRIGHT_CH),
                        sbus->GetChannel(LIGHT_THROTTLE_CH)
                        );
    return UpdateFromState(new_state);
  }

  u8_t UpdateFromState(const CtrlState& other) {
    u8_t change = CHG_NONE;
    change |= (other.mode != mode) ? CHG_MODE : 0;
    change |= (other.level != level) ? CHG_LVL : 0;
    change |= abs(other.brt - brt) > 10 ? CHG_BRT : 0;
    change |= abs(other.thr - thr) > 10 ? CHG_THR : 0;
    if (change == CHG_NONE) return change;

    *this = other;
    DBG_MD(APP, ("State: L:%d M:%02x B:%d T:%d\n", level, mode, brt, thr));
    return change;
  }

  void Log(const char* str) {
    DBG_MD(APP, ("CtrlState L:%d B:%4d M:%2d T:%4d (%s)\n", level, brt, mode, thr, str));
  }
};

class SportADCCtrl {
public:
  SportADCCtrl(SportSensor& sport, PinIdEnum volt_pin)
    : sport_(sport), volt_aidx_(GetAnalogIdx(volt_pin)){
    i8_t sport_current_idx;
    sport_.AddFcs40Sensors(&sport_current_idx, &sport_volt_idx_);
    sport.SetSensor(sport_current_idx, 0);  // Not sensing current.
    Adc::ConfigurePin(volt_aidx_);
    adc_.Enable();
    adc_.StartRead(volt_aidx_);
  }

  void Start() {
    adc_.StartRead(volt_aidx_);
  }

  void Run() {
    sport_.Run();
  }

  void UpdateVolts() {
    // analog value is 10bits (0 -> 0V, 1024 -> 5.2V)
    const int volts = adc_.ContinueRead();
    // 33/133 voltage divider -> 1024 = 21V
    // Sport value is 1/100ths of a volt
    // logically 21 * 100 * analog / 1024,
    // but we pull out common factor of 4 and shift by 8.
    const int v100 = ((525L * volts) + (1<<7)) >> 8;
    DBG_HI(APP, ("ADC: V:%d V100:%d\n", volts, v100));
    sport_.SetSensor(sport_volt_idx_, v100);
  }

protected:
  SportSensor& sport_;
  const i8_t volt_aidx_;  // Analog pin index...
  i8_t sport_volt_idx_;
  Adc adc_;
};

class Lights {
public:
  Lights(PinId led_pin, Pwm* pwm)
    : led_pin_(led_pin), pwm_(pwm) {
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      pwm_val_[i] = 0;
      pwm_->Enable(i, pwm_val_[i]);
    }
    void* ptr = led_data;
    ptr = rwngb.SetSpan(ptr, WING_CNT, /*reverse=*/false); // core to tip
    ptr = rwngf.SetSpan(ptr, WING_CNT, /*reverse=*/true);
    ptr = lwngb.SetSpan(ptr, WING_CNT, /*reverse=*/false);
    ptr = lwngf.SetSpan(ptr, WING_CNT, /*reverse=*/true);
  }
  void Update(u16_t now_ms, const CtrlState* state) {
    if (!UpdateBright(state->level, state->brt)) {
      memset(led_data, 0, sizeof(led_data));
      pwm_val_[0] = 0;
    } else {
      u16_t spd = (state->thr + (1 <<3)) >> 5;
      if (spd < 16) spd = 16;
      pulse_saw_.SetSpeed(spd);

      switch (state->Mode()) {
      case 0: UpdateSolidMode(state->Submode(), now_ms); break;
      case 1: UpdateWhiteMode(state->Submode(), now_ms); break;
      case 2: UpdateRollingMode(state->Submode(), now_ms); break;
      }
    }
    Push();
  }

  void UpdateSolidMode(u8_t submode, u16_t now_ms) {
    RGB rnav, lnav;
    pwm_val_[0] = spon_brt_;
    if (submode == 0) {
      rnav = RGB(0, spon_brt_, 0);
      lnav = RGB(spon_brt_, 0, 0);
    } else {
      u8_t glow = sin8(pulse_saw_.Get(now_ms));
      u8_t lo = spon_brt_ >> 3;
      u8_t hi = spon_brt_;
      u8_t brt = bscale8(hi - lo, glow) + lo;
      rnav = RGB(0, brt, 0);
      lnav = RGB(brt, 0, 0);
    }
    rwngb.Fill(rnav);
    rwngf.Fill(rnav);
    lwngb.Fill(lnav);
    lwngf.Fill(lnav);
  }

  void UpdateWhiteMode(u8_t submode, u16_t now_ms) {
    const RGB rnav = RGB(0, spon_brt_, 0);
    const RGB lnav = RGB(spon_brt_, 0, 0);
    rwngb.Fill(rnav, 6, 3);
    rwngf.Fill(rnav, 6, 3);
    lwngb.Fill(lnav, 6, 3);
    lwngf.Fill(lnav, 6, 3);

    u8_t brt = brt_;
    if (submode != 0) {
      u8_t glow = sin8(pulse_saw_.Get(now_ms));
      u8_t hi = brt_;
      u8_t lo = brt_ >> 3;
      brt = bscale8(hi - lo, glow) + lo;
    }

    pwm_val_[0] = brt;
    const RGB wht(brt);
    rwngb.Fill(brt, 0, 6);
    rwngf.Fill(brt, 0, 6);
    lwngb.Fill(brt, 0, 6);
    lwngf.Fill(brt, 0, 6);
  }

  void UpdateRollingMode(u8_t submode, u16_t now_ms) {
    pwm_val_[0] = brt_;
    u8_t inner = pulse_saw_.Get(now_ms);
    const u8_t step = 256 / 10;
    u8_t lo = spon_brt_ >> 3;
    u8_t hi = spon_brt_;
    for (i8_t i = 8; i >=0; --i) {
      u8_t b = bscale8(hi - lo, sin8(inner)) + lo;
      inner += step;
      const RGB rnav = RGB(0, b, 0);
      const RGB lnav = RGB(b, 0, 0);
      rwngb.Set(i, rnav);
      rwngf.Set(i, rnav);
      lwngb.Set(i, lnav);
      lwngf.Set(i, lnav);
    }
    if (submode == 0) {
      const RGB rnav = RGB(0, spon_brt_, 0);
      const RGB lnav = RGB(spon_brt_, 0, 0);
      rwngb.Fill(rnav, 6, 3);
      rwngf.Fill(rnav, 6, 3);
      lwngb.Fill(lnav, 6, 3);
      lwngf.Fill(lnav, 6, 3);
    }
  }

  void PushLeds() {
    SendWS2812(led_pin_, led_data, sizeof(led_data), 0xFF);
  }
  void PushPwm() {
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      pwm_->Set(i, pwm_val_[i]);
    }
  }
  void Push() {
    PushLeds();
    PushPwm();
  }

  // lvl is overall brightness mode (off, med, hi)
  // sbrt is 11 bit brightness slider affects 'add-ins'
  // not critical lights light wing tip/sponsons.
  bool UpdateBright(u8_t lvl, u16_t sbrt) {
    brt_ = 0;
    if (sbrt > 32) {
      sbrt = sbrt >> 3;
      brt_ = (sbrt > 255) ? 255 : sbrt;
    }

    bool result = true;
    switch (lvl) {
    default:
    case 0: brt_ = spon_brt_ = 0; result = false; break;
    case 1: spon_brt_ = 0x40; brt_ = brt_ >> 2; break;
    case 2: spon_brt_ = 0xFF; break;
    }

    DBG_LO(APP, ("Update Brt: %d Spn: %d lvl: %d\n", brt_, spon_brt_, lvl));
    return result;
  }
protected:
  const PinId led_pin_;
  Pwm* const pwm_;

  u8_t brt_, spon_brt_;
  u8_t pwm_val_[NUM_PWM];
  VariableSaw pulse_saw_;
  LedSpan<RGB> rwngb;
  LedSpan<RGB> rwngf;
  LedSpan<RGB> lwngb;
  LedSpan<RGB> lwngf;
};


int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);

  Pwm pwm(PORT_D, 400);

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  DBG_LEVEL_MD(SBUS);
  SBus sbus(&Serial::usart1, /*invert=*/false);

  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();
  blink_pin.toggle();

  SportSensor sport(&Serial::usart3, /*invert=*/true, /*use_alt_pins=*/false);
  SportADCCtrl volt_ctrl(sport, VOLT_PIN);

  PinId rgb_led_pin(RGB_LED_PIN);
  rgb_led_pin.SetOutput();

  sei();
  DBG_MD(APP, ("Mini-7: Run\n"));
  volt_ctrl.Start();

  memset(led_data, 0, sizeof(led_data));
  SendWS2812(rgb_led_pin, led_data, sizeof(led_data), 0xFF);
  Lights lights(rgb_led_pin, &pwm);

  CtrlState state;
  u8_t blink_phase = 0;

  u8_t update_0 = 0;
  u8_t update_4 = 0;
  u8_t update_5 = 0;
  u8_t update_8 = 0;
  u8_t update_16s = 0xFF;


  while (1) {
    u16_t now = FastTimeMs();
    // Updates sbus state, called frequently to ensure serial buffer doesn't overflow.
    if (sbus.Run()) {
      state.Set(&sbus);  // Could call lights.Run if state changed...
      // state.Log("Sbus");
    }
    // Update telemetry if we were polled (call very frequently since there
    // is a limited time window to respond when polled).  This uses the last
    // value set for the polled device.
    volt_ctrl.Run();

    if (update_0 == now) continue;  // 1ms
    update_0 = now;

    const u8_t now_4 = now >> 4;   // 1/64 sec
    if (now_4 == update_4) continue;
    update_4 = now_4;
    volt_ctrl.UpdateVolts();

    const u8_t now_5 = now >> 5;   // 1/32 sec
    if (now_5 == update_5) continue;
    update_5 = now_5;
    lights.Update(now, &state);

    const u8_t now_8 = now >> 8;
    if (now_8 == update_8) continue;  // 1/4 sec
    update_8 = now_8;
    u8_t pat = 0x33;
    blink_pin.set(!(pat & (1 << blink_phase)));  // pin high is off.
    blink_phase = (blink_phase + 1) & 0x07;

    u16_t now_s = FastSecs();
    u8_t now_16s = now_s >> 4;
    if (now_16s == update_16s) continue;  // 16 sec
    update_16s = now_16s;
  }
}
