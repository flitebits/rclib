// Copyright 2021, 2022 Thomas DeWeese
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
#include "leds/Pixel.h"
#include "leds/Rgb.h"

#include "Pins.h"
#include "Pwm.h"
#include "SBus.h"
#include "Serial.h"
#include "WS2812.h"

using led::HSV;
using led::Logify;
using led::RGB;
using led::RGBW;
using led::bscale8;
using led::sin8;

#if defined(__AVR_ATmega4808__)
#define LED_PIN (PIN_F0)
#endif

#if defined(__AVR_ATmega4809__)
#define LED_PIN (PIN_F0)
#endif

#define LIGHT_BRIGHT_CH (10)  // Adjust leading edge light brightness.
#define LIGHT_LEVEL_CH (11)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH (12)  // Sets Lights Mode (solid, spin, glow)
#define LIGHT_SUBMODE_CH (13)  // Sets mode sub mode (color range, etc)
#define LIGHT_THROTTLE_CH (14)  // Adjusts lights speed

#define NUM_PWM (6)
#define FUSE_CNT (15)
#define RUDR_CNT (10)
#define ELEV_CNT (15)

#define RGBW_CNT (FUSE_CNT + RUDR_CNT + 2 * ELEV_CNT)

u8_t led_data[RGBW_CNT * 4];


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
  CtrlState(u8_t m, u8_t l, i16_t b, i16_t t)
    : mode(m), level(l), brt(b), thr(t) { }

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

};

class SinFilled {
public:
  SinFilled(const RGBW& color, u8_t scale, u8_t offset = 0) :
    scale_(scale), color_(color), offset_(offset) { }
  void SetOffset(u8_t offset) { offset_ = offset; }
  u8_t GetOffset() { return offset_; }

  void operator() (u8_t frac, RGBW* pix) const {
    frac = (u16_t(frac) * scale_) >> 6;
    *pix = color_;
    Fade(pix, Logify(sin8(offset_ + frac)));
  }
private:
  const u8_t scale_;
  const RGBW color_;
  u8_t offset_;
};

class Lights {
public:
  Lights(PinId led_pin, Pwm* pwm) :
    now_(0), prev_update_(0),
    led_pin_(led_pin), pwm_(pwm),
    mode_(0), brt_(0), spon_brt_(0),
    state_change_(CtrlState::CHG_NONE) {
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      pwm_val_[i] = 0;
      pwm_->Set(i, pwm_val_[i]);
      pwm_->Enable(i);
    }

    void* ptr = led_data;
    ptr = fuse_.SetSpan(ptr, FUSE_CNT, false);
    ptr = rudr_.SetSpan(ptr, RUDR_CNT, false);
    ptr = elvr_.SetSpan(ptr, ELEV_CNT, true);
    ptr = elvl_.SetSpan(ptr, ELEV_CNT, false);
  }

  u8_t GetBlinkPattern() {
    // const u8_t pat_host = 0xCC; // Fast blink
    // const u8_t pat_client = 0x0C; // Slow/Fast
    const u8_t pat_slow = 0x0F; // Slow blink
    const u8_t pat_off = 0x00; // All off
    return (brt_ == 0) ? pat_off : pat_slow;
  }

  u8_t Mode() { return mode_ >> 2; }
  u8_t Submode() { return mode_ & 0x03; }
  void PushLeds() {
    SendWS2812(led_pin_, led_data, sizeof(led_data), 0xFF);
  }
  void PushPwm() {
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      pwm_->Set(i, pwm_val_[4 - i]); // PWM are back (idx =0) to front (idx=4)
    }
  }
  void Push() {
    PushLeds();
    PushPwm();
  }

  // Returns true if the state_ was changed.
  bool SBusUpdate(SBus* sbus) {
    state_change_ |= state_.Set(sbus);
    return (state_change_ != CtrlState::CHG_NONE);
  }

  // lvl is overall brightness mode (off, med, hi)
  // sbrt is 11 bit brightness slider affects 'add-ins'
  // not critical lights light wing tip/sponsons.
  void UpdateBright(u8_t lvl, u16_t sbrt) {
    brt_ = 0;
    if (sbrt > 32) {
      sbrt = sbrt >> 3;
      brt_ = (sbrt > 255) ? 255 : sbrt;
    }

    switch (lvl) {
    case 0: brt_ = spon_brt_ = 0; break;
    case 1: spon_brt_ = 0x40; brt_ = brt_ >> 3; break;
    case 2: spon_brt_ = 0xFF; break;
    }
    DBG_HI(APP, ("Update Brt: %d Spn: %d\n", brt_, spon_brt_));
  }

  void ApplyUpdatedState() {
    // bool mode_change = (state_change_ & CHG_MODE) != CHG_NONE;
    state_change_ = CtrlState::CHG_NONE;
    mode_ = state_.mode;
    UpdateBright(state_.level, state_.brt);
    SinFilled red(led::wclr::red, 1 << 6);
    SinFilled grn(led::wclr::green, 1 << 6);
    SinFilled wht(led::wclr::white, 1 << 6);
    SinFilled blu(led::wclr::blue, 1 << 6);
    fuse_.FillOp(blu);
    rudr_.FillOp(wht);
    elvr_.FillOp(grn);
    elvl_.FillOp(red);
    /*
    bool is_host = (host_ == HOST);
    solid_state_.UpdateState(is_host, brt_, spon_brt_);
    color_mode_state_.UpdateState(is_host, Submode(), state_.thr, spon_brt_);
    pulse_mode_state_.UpdateState(is_host, Submode(), state_.thr);
    */
  }

  void Update(u16_t now) {
    DBG_HI(APP, ("Lights::Update now: %u\n", now));
    fuse_.Rotate();
    rudr_.Rotate();
    elvr_.Rotate();
    elvl_.Rotate();
    // We want cycle pos to complete a cycle every second (8bits)
    // 1s = 1024'ms' = 10bits, so shift 2 bits for that, and 5
    // bits for spd for a total of 7.
    // u8_t cycle_pos = ((now * spd_) + (1 << 6)) >> 7;
    // cycle_pos += shared_state_->cycle_pos_offset();
    /*
    switch (Mode()) {
    case 0: solid_state_.Update(now); UpdateSolidMode(); break;
    case 1:
      color_mode_state_.Update(now);
      UpdateColorMode();
      break;
    case 2:
      pulse_mode_state_.Update(now);
      UpdatePulseMode();
      break;
    }
    */
    Push();
  }

  // Called roughly every 32ms (32 fps)
  void Run(SBus* sbus) {
    DBG_HI(APP, ("Update\n"));
    now_ = FastTimeMs();
    u8_t now_8 = now_ >> 8;  // 1/4 seconds
    // Update state if forced,  sbus settings change or 1/4 sec has elapsed.
    if (now_8 != prev_update_ || SBusUpdate(sbus) ) {
      prev_update_ = now_8;
      ApplyUpdatedState();
    }
    Update(now_);
  }

private:
  u16_t now_;
  u8_t prev_update_;
  PinId led_pin_;
  Pwm* const pwm_;
  u8_t pwm_val_[NUM_PWM];
  u8_t mode_, brt_, spon_brt_;
  CtrlState state_;
  u8_t state_change_;

  LedSpan<RGBW> fuse_;
  LedSpan<RGBW> rudr_;
  LedSpan<RGBW> elvr_;
  LedSpan<RGBW> elvl_;
};

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  Pwm pwm(PORT_D, 400);
  for (u8_t i = 0; i < 5; ++i) {
    pwm.Enable(i);
    pwm.Set(i, 0);
  }

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  DBG_LEVEL_HI(SBUS);
  SBus sbus(&Serial::usart2, /*invert=*/true);

  PinId led_pin(LED_PIN);
  led_pin.SetOutput();
  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();

  Lights lights(LED_PIN, &pwm);

  sei();
  DBG_MD(APP, ("Hello World: Test\n"));

  memset(led_data, 0, sizeof(led_data));
  SendWS2812(LED_PIN, led_data, sizeof(led_data), 0xFF);

  u8_t update_5 = 0;
  DBG_MD(APP, ("Entering Run Loop\n"));
  u8_t update_8 = 0;
  while (1) {
    u16_t now = FastTimeMs();
    const u8_t now_8 = now >> 8;

    sbus.Run();

    const u8_t now_5 = now >> 5;
    if (now_5 == update_5) continue;
    update_5 = now_5;
    lights.Run(&sbus);  // Won't do anything if not known host

    if (now_8 == update_8) continue;
    update_8 = now_8;
    u8_t phase = (now_8 & 0x07);
    u8_t pat = lights.GetBlinkPattern();
    blink_pin.set(pat & (1 << phase));
  }
}
