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
using led::VariableSaw;
using led::bscale8;
using led::sin8;

#if defined(__AVR_ATmega4808__)
#define LED_PIN (PIN_F0)
#endif

#if defined(__AVR_ATmega4809__)
#define LED_PIN (PIN_C2)
#endif

#define LIGHT_BRIGHT_CH (11)  // Adjust leading edge light brightness.
#define LIGHT_LEVEL_CH (12)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH (13)  // Sets Lights Mode (solid, spin, glow)
#define LIGHT_SUBMODE_CH (14)  // Sets mode sub mode (color range, etc)
#define LIGHT_THROTTLE_CH (15)  // Adjusts lights speed

#define NUM_PWM (3)
#define WING_CNT (33)
#define FUSE_CNT (24)

#define RGBW_CNT (WING_CNT + 2 * FUSE_CNT)

u8_t led_data[RGBW_CNT * 4];

u8_t min(u8_t a, u8_t b) {
  return (a < b) ? a : b;
}
u8_t max(u8_t a, u8_t b) {
  return (a > b) ? a : b;
}

u8_t sbus2u8(i16_t val) {
  if (val <= 32) return 0;
  val = val >> 3;
  return (val > 255) ? 255 : val;
}

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

struct SolidState {
  u8_t brt;
  u8_t slide;
  u8_t sub_mode;
  bool flash;

  SolidState() : brt(0), slide(0), flash(false) { }

  void UpdateState(i16_t sbrt, u8_t brt) {
    this->brt = brt;
    this->slide = sbus2u8(sbrt);
  }

  void Update(u16_t now) {
    // Flash for the last 64ms of every second (1s = 1024 or 10bits
    // 64ms = (1 << 6) so
    flash = ((u8_t(now >> 6) & 0x1F) == 0x1F);
  }
};

class ColorModeState {
public:

  ColorModeState()
    : submode_(0), brt_(0), pulse_phase_(0), scale_(0), offset_(0) { }

  void UpdateState(u8_t submode, i16_t thr, u8_t brt) {
    submode_ = submode;
    brt_ = brt;
    // Throttle is 11 bits, spd has 5 fractional bits, so this makes the
    // speed multiplier go from 0 -> 4x with throttle.
    u8_t spd = (thr + (1 <<3)) >> 4;
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
    color_ = HsvToRgbw(HSV(color_hue, 0xFF, brt_));
    pulse_phase_ = 255 - pulse_saw_.Get(now);
    flash = ((u8_t(now >> 6) & 0x1F) == 0x1F);
  }
  const RGBW& GetPix() { return color_; }

  void SetOpOffset(u8_t offset) {
    offset_ = offset;
  }
  // Scale is 2.6 FP number
  void SetOpScale(u8_t scale) {
    scale_ = scale;
  }
  void operator() (u8_t frac, RGBW* pix) const {
    frac = (u16_t(frac) * scale_) >> 6;
    *pix = color_;
    u8_t wave = sin8(offset_ + pulse_phase_ + frac);
    Fade(pix, Logify((wave >> 1) + 0x80 ));
  }

  bool flash;

protected:
  u8_t submode_;
  u8_t brt_;
  VariableSaw color_saw_;
  VariableSaw pulse_saw_;
  u8_t pulse_phase_;
  u8_t scale_;
  u8_t offset_;
  RGBW color_;
};

static const u8_t spd_map[] = {0x20, 0x50, 0x70, 0xA0, 0xFF};
u8_t GetSpd(i16_t thr) {
  // Throttle is 11 bits, spd has 5 fractional bits, so this makes the
  // speed multiplier go from 0 -> 4x with throttle.
  u8_t thr8 = thr >> 3;
  u8_t spd_idx = thr8 >> 6;
  u8_t spd_frac = thr8 & 0x3F;
  return spd_map[spd_idx] +
    ((u16_t(spd_map[spd_idx + 1] - spd_map[spd_idx]) * spd_frac) >> 6);
}

class PulseModeState {
public:
  PulseModeState() {}

  void UpdateState(u8_t submode, i16_t thr) {
    submode_ = submode;
    u8_t spd = GetSpd(thr);
    glow_saw_.SetSpeed(spd);
    move_saw_.SetSpeed(spd);
  }

  void Update(u16_t now) {
    // u8_t glow = glow_saw_.Get(now);
  }

  u8_t submode_;
  VariableSaw glow_saw_;
  VariableSaw move_saw_;
};

u8_t GetBlinkPattern() {
  // const u8_t pat_slow = 0x0F; // Slow blink
  // const u8_t pat_med = 0xCC; // Med blink
  const u8_t pat_fast = 0xAA; // Fast blink
  // const u8_t pat_fast_med = 0xAC; // 2 Fast, one Med
  // const u8_t pat_short_slow = 0x01; // slow short
  return pat_fast;
}


class Lights {
public:
  Lights(PinId led_pin, Pwm* pwm) :
    now_(0), prev_update_(0),
    led_pin_(led_pin), pwm_(pwm),
    mode_(0), brt_(0), lvl_(0),
    state_change_(CtrlState::CHG_NONE) {
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      pwm_val_[i] = 0;
      pwm_->Set(i, pwm_val_[i]);
      pwm_->Enable(i);
    }
    memset(led_data, 0, sizeof(led_data));
    PushLeds();

    void* ptr = led_data;
    ptr = wing_.SetSpan(ptr, WING_CNT, false);
    ptr = rfus_.SetSpan(ptr, FUSE_CNT, true);
    ptr = lfus_.SetSpan(ptr, FUSE_CNT, false);

    ptr = rwing_.SetSpan(wing_.ptr(), WING_CNT / 2, true);
    ptr = cwing_.SetSpan(ptr, 1, false);
    ptr = lwing_.SetSpan(ptr, WING_CNT / 2, false);

    ptr = rwngt_.SetSpan(wing_.ptr(), 5, true);
    ptr = cwngt_.SetSpan(ptr, WING_CNT - 10, false);
    ptr = lwngt_.SetSpan(ptr, 5, false);

    ptr = rbfus_.SetSpan(rfus_.ptr(), FUSE_CNT - 8, true);
    ptr = rffus_.SetSpan(ptr, 8, true);
    ptr = lffus_.SetSpan(ptr, 8, true);
    ptr = lbfus_.SetSpan(ptr, FUSE_CNT - 8, true);

    color_state_.SetOpScale(1 << 6);
    color_state_.SetOpOffset(0);
  }

  u8_t Mode() { return mode_ >> 2; }
  u8_t Submode() { return mode_ & 0x03; }
  void PushLeds() {
    SendWS2812(led_pin_, led_data, sizeof(led_data), lvl_);
  }
  void PushPwm() {
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      if (pwm_val_[i] != pwm_->Get(i)) {
        pwm_->Set(i, pwm_val_[i]);
      }
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
    case 0: lvl_ = 0x00; brt_ = 0;         break;
    case 1: lvl_ = 0x3F; brt_ = brt_ >> 2; break;
    case 2: lvl_ = 0xFF; brt_ = brt_;      break;
    }
    DBG_HI(APP, ("Update Lvl: %d Brt: %d\n", lvl_, brt_));
  }

  void ApplyUpdatedState() {
    // bool mode_change = (state_change_ & CHG_MODE) != CHG_NONE;
    state_change_ = CtrlState::CHG_NONE;
    mode_ = state_.mode;
    UpdateBright(state_.level, state_.brt);
    solid_state_.UpdateState(state_.brt, brt_);
    color_state_.UpdateState(Submode(), state_.thr, brt_);
    pulse_state_.UpdateState(Submode(), state_.thr);
  }

  void Update(u16_t now) {
    DBG_HI(APP, ("Lights::Update now: %u\n", now));
    // We want cycle pos to complete a cycle every second (8bits)
    // 1s = 1024'ms' = 10bits, so shift 2 bits for that, and 5
    // bits for spd for a total of 7.
    // u8_t cycle_pos = ((now * spd_) + (1 << 6)) >> 7;
    // cycle_pos += shared_state_->cycle_pos_offset();
    switch (Mode()) {
    case 0:
      solid_state_.Update(now);
      UpdateSolidMode(); break;
    case 1:
      color_state_.Update(now);
      UpdateColorMode();
      break;
    case 2:
      pulse_state_.Update(now);
      UpdatePulseMode();
      break;
    }
    Push();
  }

  // Solid mode is wing mostly white, fuse white or orientation colors.
  void UpdateSolidMode() {
    u8_t smode = Submode();
    // Submodes are default = 0, simple = 1, flashy = 2
    RGBW wht = RGBW(smode != 1 ? brt_ : lvl_);
    RGBW fwht = RGBW(lvl_);
    RGBW lclr = (smode != 1 && solid_state_.flash) ? fwht : RGBW(lvl_, 0, 0);
    RGBW lblend = lclr;
    Blend(&lblend, fwht, solid_state_.slide);
    RGBW rclr = (smode != 1 && solid_state_.flash) ? fwht : RGBW(0, lvl_, 0);
    RGBW rblend = rclr;
    Blend(&rblend, fwht, solid_state_.slide);
    RGBW center = led::wclr::black;
    Blend(&center, fwht, solid_state_.slide);

    switch (smode) {
    case 0:
      lwing_.Fill(lblend);
      cwing_.Fill(center);
      rwing_.Fill(rblend);
      lfus_.Fill(lblend);
      rfus_.Fill(rblend);
      break;
    case 1:
      lwing_.Fill(lblend);
      lwngt_.Fill(lclr);
      rwing_.Fill(rblend);
      rwngt_.Fill(rclr);
      cwing_.Fill(center);
      lbfus_.Fill(lclr);
      lffus_.Fill(lblend);
      rbfus_.Fill(rclr);
      rffus_.Fill(rblend);
      break;
    case 2:
      cwngt_.Fill(wht);
      lffus_.Fill(wht);
      rffus_.Fill(wht);
      lwngt_.Fill(lclr);
      rwngt_.Fill(rclr);
      lbfus_.Fill(lclr);
      rbfus_.Fill(rclr);
      break;
    }


    u8_t val = min(max(brt_, lvl_ >> 1), lvl_);
    if (smode == 2 || solid_state_.flash) {
      val = lvl_;
    }
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      pwm_val_[i] = val;
    }
  }

  // Solid mode is wing mostly white, fuse white or orientation colors.
  void UpdateColorMode() {
    // Submodes are default = 0, simple = 1, flashy = 2
    u8_t smode = Submode();

    lwing_.FillOp(color_state_);
    cwing_.Fill(lwing_.At(0));
    rwing_.FillOp(color_state_);
    lfus_.FillOp(color_state_);
    rfus_.FillOp(color_state_);

    if (smode != 2) {
      RGBW fwht = RGBW(lvl_);
      RGBW lclr = (smode == 0 && color_state_.flash)
        ? fwht : RGBW(lvl_, 0, 0);
      lwing_.At(lwing_.len() - 1) = lclr;
      lwing_.At(lwing_.len() - 2) = lclr;
      RGBW rclr = (smode == 0 && color_state_.flash)
        ? fwht : RGBW(0, lvl_, 0);
      rwing_.At(lwing_.len() - 1) = rclr;
      rwing_.At(lwing_.len() - 2) = rclr;
    }

    for (u8_t i = 0; i < NUM_PWM; ++i) {
      pwm_val_[i] = lvl_;
    }
  }

  // Solid mode is wing mostly white, fuse white or orientation colors.
  void UpdatePulseMode() {
    u8_t smode = Submode();
    // Submodes are default = 0, simple = 1, flashy = 2
    RGBW wht = RGBW(brt_);
    cwngt_.Fill(wht);
    lffus_.Fill(wht);
    rffus_.Fill(wht);

    RGBW lclr = (smode != 1 && solid_state_.flash) ? wht : RGBW(lvl_, 0, 0);
    RGBW rclr = (smode != 1 && solid_state_.flash) ? wht : RGBW(0, lvl_, 0);
    lwngt_.Fill(lclr);
    lbfus_.Fill(lclr);
    rwngt_.Fill(rclr);
    rbfus_.Fill(rclr);

    u8_t val = lvl_;
    if (smode != 2 || solid_state_.flash) {
      val = min(max(brt_, lvl_ >> 1), lvl_);
    }
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      pwm_val_[i] = val;
    }
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
  u8_t mode_, brt_, lvl_;
  CtrlState state_;
  u8_t state_change_;

  SolidState solid_state_;
  ColorModeState color_state_;
  PulseModeState pulse_state_;

  LedSpan<RGBW> wing_;  // full wing
  LedSpan<RGBW> lwing_; // ~1/2 the wing
  LedSpan<RGBW> cwing_; // left over (1 led)
  LedSpan<RGBW> rwing_; // ~1/2 the wing

  LedSpan<RGBW> lwngt_; // Just tip of wing (5)
  LedSpan<RGBW> cwngt_; // Bulk of wing
  LedSpan<RGBW> rwngt_; // Just tip of wing (5)

  LedSpan<RGBW> lfus_;  // full left side
  LedSpan<RGBW> lffus_; // left front fuse
  LedSpan<RGBW> lbfus_; // left back fuse
  LedSpan<RGBW> rfus_;  // full right side
  LedSpan<RGBW> rffus_; // right front fuse
  LedSpan<RGBW> rbfus_; // right back fuse


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
  SBus sbus(&Serial::usart1, /*invert=*/true);

  PinId led_pin(LED_PIN);
  led_pin.SetOutput();
  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();

  Lights lights(LED_PIN, &pwm);

  sei();
  DBG_MD(APP, ("Hello World: Test\n"));

  memset(led_data, 0, sizeof(led_data));
  SendWS2812(LED_PIN, led_data, sizeof(led_data), 0xFF);

  const u8_t pat = GetBlinkPattern();

  u8_t update_5 = 0;
  DBG_MD(APP, ("Entering Run Loop\n"));
  u8_t update_8 = 0;
  while (1) {
    u16_t now = FastTimeMs();

    sbus.Run();

    const u8_t now_5 = now >> 5;
    if (now_5 == update_5) continue;
    update_5 = now_5;
    lights.Run(&sbus);

    const u8_t now_8 = now >> 8;
    if (now_8 == update_8) continue;
    update_8 = now_8;
    u8_t phase = (now_8 & 0x07);
    blink_pin.set(pat & (1 << phase));
  }
}
