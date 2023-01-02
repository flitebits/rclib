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
#include "leds/LedSpanOps.h"
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
using led::VariableSaw;

#if defined(__AVR_ATmega4808__)
#define LED_PIN (PIN_F0)
#endif

#if defined(__AVR_ATmega4809__)
#define LED_PIN (PIN_C2)
#endif

#define LIGHT_BRIGHT_CH (10)  // Adjust leading edge light brightness.
#define LIGHT_LEVEL_CH (11)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH (12)  // Sets Lights Mode (solid, spin, glow)
#define LIGHT_SUBMODE_CH (13)  // Sets mode sub mode (color range, etc)
#define LIGHT_THROTTLE_CH (14)  // Adjusts lights speed

#define NUM_PWM (6)
#define WING_CNT (21)  // Each wing half has 21 RGBW
#define TIP_CNT (8)   // Each wing tip has 8 RGBW
#define FUSE_CNT (15)
#define RUDR_CNT (10)
#define ELEV_CNT (15)


#define RGBW_CNT (FUSE_CNT + RUDR_CNT + 2 * ELEV_CNT + \
                  4 * (WING_CNT + TIP_CNT))

u8_t led_data[RGBW_CNT * 4];

enum PwmCh {
            PWM_WING = 0,
            PWM_LTIP = 1,
            PWM_FUSE = 2,
            PWM_TAIL = 3,
            PWM_RTIP = 4,
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

class LogSinOp {
public:
  LogSinOp() : scale_(1<<6), offset_(0) { }
  LogSinOp(RGBW color, u8_t scale = (1<<6), u8_t offset = 0)
    :color_(color), scale_(scale), offset_(offset) { }
  void Set(const RGBW& color, u8_t scale, u8_t offset) {
    color_ = color;
    scale_ = scale;
    offset_ = offset;
  }
  void SetColor(const RGBW& color) { color_ = color; }
  RGBW GetColor() const { return color_; }
  u8_t GetOffset() const { return offset_; }
  void SetOffset(u8_t offset) { offset_ = offset; }
  // Scale is 2.6 FP number
  u8_t GetScale() const { return scale_; }
  void SetScale(u8_t scale) { scale_ = scale; }

  void operator() (u8_t frac, RGBW* pix) const {
    frac = (u16_t(frac) * scale_) >> 6;
    *pix = color_;
    Fade(pix, Logify(sin8(frac + offset_ )));
  }

  RGBW color_;
  u8_t scale_;
  u8_t offset_;
};

class SolidModeState {
public:
  SolidModeState()
    : brt_(0), scale_(0), wave_phase_(0),
      blink_sec_(0), blink_(false) { }

  bool blink() const { return blink_; }

  void UpdateState(i16_t thr, u8_t brt) {
    brt_ = brt;
    // Throttle is 11 bits, spd has 5 fractional bits, so this makes the
    // speed multiplier go from 0 -> 4x with throttle.
    u8_t spd = (thr + (1 <<3)) >> 5;
    if (spd < 0x07) spd = 0x07;
    wave_saw_.SetSpeed(u16_t(spd));
  }
  void Update(u16_t now_ms) {
    wave_phase_ = 255 - wave_saw_.Get(now_ms);
    u8_t now_s = now_ms >> 10;
    u8_t now_5 = (now_ms >> 5) & 0x1F;
    if (u8_t(now_s - blink_sec_) >= 2) {
      blink_sec_ = now_s;
      blink_ = true;
    } else if (now_5 >= 1) {
      blink_ = false;
    }
  }
  const LogSinOp& Op() const { return op_; }

  // Scale is 2.6 FP number
  void SetOp(const RGBW& color) {
    op_.Set(color, 1<<6, wave_phase_);
  }

protected:
  u8_t brt_;
  u8_t scale_;
  u8_t wave_phase_;
  u8_t blink_sec_;
  bool blink_;
  RGBW color_;
  VariableSaw wave_saw_;
  LogSinOp op_;
};

class ColorModeState {
public:
  ColorModeState()
    : brt_(0), pulse_phase_(0) { }

  void UpdateState(u8_t submode, i16_t thr, u8_t brt) {
    submode_ = submode;
    brt_ = brt;
    // Throttle is 11 bits, spd has 5 fractional bits, so this makes the
    // speed multiplier go from 0 -> 4x with throttle.
    u8_t spd = (thr + (1 <<3)) >> 5;
    if (spd < 0x07) spd = 0x07;
    color_saw_.SetSpeed(spd >> 1);
    pulse_saw_.SetSpeed(u16_t(spd) << 1);
  }

  void Update(u16_t now) {
    u8_t color_hue = color_saw_.Get(now);
    if (submode_ != 2) {
      // Limit hue to 0 -> 16 (red -> orange)
      // so limit to 16 and subtract 4 (it's unsigned so the negative
      // values wrap to high values which is fine).
      if (color_hue >= 128) {
        // If it's greater than 128 the reflect it back towards zero.
        color_hue = 255 - color_hue;
      }
      // Now limited to 0->128, so divide by 8 so it's 0-16.
      color_hue = (color_hue >> 3);
    }
    op_.SetColor(HsvToRgb(HSV(color_hue, 0xFF, brt_)));
    pulse_phase_ = 255 - pulse_saw_.Get(now);
  }
  const RGB& GetPix() { return color_; }

  const LogSinOp& Op() const { return op_; }
  // Scale is 2.6 FP number
  void SetOp(u8_t scale, u8_t offset) {
    op_.SetScale(scale);
    op_.SetOffset(pulse_phase_ + offset);
  }

protected:
  struct WireData {
    u8_t color_off;
    u8_t pulse_off;
  };
  WireData wire_;
  u8_t submode_;
  u8_t brt_;
  VariableSaw color_saw_;
  VariableSaw pulse_saw_;
  u8_t pulse_phase_;
  RGB color_;
  LogSinOp op_;
};

class PulseModeState {
public:
  PulseModeState() { }
  void UpdateState() {
  }
  void Update(u16_t now_ms) {
  }

protected:
  u8_t brt_;
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
    state_.mode = 6;
    state_.level = 1;
    // state_.brt = 0x080;

    void* ptr = led_data;
    ptr = ll_tip_ .SetSpan(ptr, TIP_CNT, false);
    ptr = ll_wing_.SetSpan(ptr, WING_CNT, true);
    ptr = lr_wing_.SetSpan(ptr, WING_CNT, false);
    ptr = lr_tip_ .SetSpan(ptr, TIP_CNT, false);
    ptr = ur_tip_ .SetSpan(ptr, TIP_CNT, false);
    ptr = ur_wing_.SetSpan(ptr, WING_CNT, true);
    ptr = ul_wing_.SetSpan(ptr, WING_CNT, false);
    ptr = ul_tip_ .SetSpan(ptr, TIP_CNT, false);

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
      pwm_->Set(i, pwm_val_[i]);
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

    DBG_HI(APP, ("Update Brt: %d Spn: %d lvl: %d\n", brt_, spon_brt_, lvl));
  }

  void ApplyUpdatedState() {
    // bool mode_change = (state_change_ & CHG_MODE) != CHG_NONE;
    state_change_ = CtrlState::CHG_NONE;
    mode_ = state_.mode;
    UpdateBright(state_.level, state_.brt);
    solid_mode_state_.UpdateState(state_.thr, brt_);
    color_mode_state_.UpdateState(Submode(), state_.thr, spon_brt_);
    pulse_mode_state_.UpdateState();
  }

  void Update(u16_t now) {
    // DBG_HI(APP, ("Lights::Update now: %u\n", now));
    switch (Mode()) {
    case 0:
      solid_mode_state_.Update(now);
      UpdateSolidMode();
      break;
    case 1:
      color_mode_state_.Update(now);
      UpdateColorMode();
      break;
    case 2:
      pulse_mode_state_.Update(now);
      UpdatePulseMode();
      break;
    }
    Push();
  }

  void UpdateSolidMode() {
    u8_t submode = Submode();
    const bool blink = (submode != 1) && solid_mode_state_.blink();
    RGBW rnav = blink ? RGBW(spon_brt_) : RGBW(0, spon_brt_, 0);
    RGBW lnav = blink ? RGBW(spon_brt_) : RGBW(spon_brt_, 0, 0);
    lr_tip_.Fill(rnav);
    ll_tip_.Fill(lnav);
    ur_tip_.Fill(rnav);
    ul_tip_.Fill(lnav);
    elvr_.Fill(rnav);
    elvl_.Fill(lnav);
    rudr_.Fill(RGBW(spon_brt_));

    RGBW wht = RGBW(brt_);
    if (submode != 2) {
      elvr_.Fill(wht, 0, 7);
      elvl_.Fill(wht, 0, 7);
      rudr_.Fill(wht, 0, 7);
    }

    if (submode == 0) {
      ll_wing_.Fill(wht);
      ul_wing_.Fill(wht);
      lr_wing_.Fill(wht);
      ur_wing_.Fill(wht);
    } else if (submode == 1) {
      ll_wing_.Fill(wht, 0, 12);
      ul_wing_.Fill(wht, 0, 12);
      ll_wing_.Fill(lnav, 12, 9);
      ul_wing_.Fill(lnav, 12, 9);
      lr_wing_.Fill(wht, 0, 12);
      ur_wing_.Fill(wht, 0, 12);
      lr_wing_.Fill(rnav, 12, 9);
      ur_wing_.Fill(rnav, 12, 9);
    } else {  // submode 2
      solid_mode_state_.SetOp(rnav);
      lr_wing_.FillOp(solid_mode_state_.Op());
      ur_wing_.FillOp(solid_mode_state_.Op());
      solid_mode_state_.SetOp(lnav);
      ll_wing_.FillOp(solid_mode_state_.Op());
      ul_wing_.FillOp(solid_mode_state_.Op());
    }

    pwm_val_[PWM_LTIP] = pwm_val_[PWM_RTIP] = 0; // spon_brt_;
    pwm_val_[PWM_FUSE] = pwm_val_[PWM_WING] = brt_;
    pwm_val_[PWM_TAIL] = (brt_ < 1 || brt_ > 3) ? brt_ >> 2 : 1;
  }

  void UpdateColorMode() {
    RGBW rnav = RGBW(0, spon_brt_, 0);
    lr_tip_.Fill(rnav);
    ur_tip_.Fill(rnav);
    elvr_.Fill(rnav);

    RGBW lnav = RGBW(spon_brt_, 0, 0);
    ll_tip_.Fill(lnav);
    ul_tip_.Fill(lnav);
    elvl_.Fill(lnav);

    color_mode_state_.SetOp(1 << 6, 0);
    lr_wing_.FillOp(color_mode_state_.Op());
    ur_wing_.FillOp(color_mode_state_.Op());
    ll_wing_.FillOp(color_mode_state_.Op());
    ul_wing_.FillOp(color_mode_state_.Op());

    color_mode_state_.SetOp(1 << 6, 0);
    fuse_.FillOp(color_mode_state_.Op());
    color_mode_state_.SetOp(1 << 6, 128);
    rudr_.FillOp(color_mode_state_.Op());

    pwm_val_[PWM_LTIP] = pwm_val_[PWM_RTIP] = 0; //  spon_brt_;
    pwm_val_[PWM_FUSE] = pwm_val_[PWM_WING] =  brt_;
    pwm_val_[PWM_TAIL] = (brt_ < 1 || brt_ > 3) ? brt_ >> 2 : 1;
  }

  void UpdatePulseMode() {
    memset(led_data, 0, sizeof(led_data));

    pwm_val_[PWM_LTIP] = pwm_val_[PWM_RTIP] =  spon_brt_;
    pwm_val_[PWM_FUSE] = pwm_val_[PWM_WING] =  brt_;
    pwm_val_[PWM_TAIL] = (brt_ < 1 || brt_ > 3) ? brt_ >> 2 : 1;
  }

  // Called roughly every 32ms (32 fps)
  void Run(SBus* sbus) {
    // DBG_HI(APP, ("Update\n"));
    now_ = FastTimeMs();
    u8_t now_8 = now_ >> 8;  // 1/4 seconds
    // Update state if forced,  sbus settings change or 1/4 sec has elapsed.
    if (SBusUpdate(sbus) || now_8 != prev_update_) {
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
  SolidModeState solid_mode_state_;
  ColorModeState color_mode_state_;
  PulseModeState pulse_mode_state_;
  u8_t state_change_;

  LedSpan<RGBW> ll_tip_;
  LedSpan<RGBW> ll_wing_;
  LedSpan<RGBW> lr_wing_;
  LedSpan<RGBW> lr_tip_;
  LedSpan<RGBW> ur_tip_;
  LedSpan<RGBW> ur_wing_;
  LedSpan<RGBW> ul_wing_;
  LedSpan<RGBW> ul_tip_;
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

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_HI(APP);
  DBG_LEVEL_MD(SBUS);
  SBus sbus(&Serial::usart1, /*invert=*/true);

  PinId led_pin(LED_PIN);
  led_pin.SetOutput();
  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();

  Pwm pwm(PORT_D, 400);
  Lights lights(LED_PIN, &pwm);

  sei();
  DBG_MD(APP, ("BristolSpadport: Test\n"));

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
