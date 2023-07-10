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

#include "Button.h"
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
using led::scale8;
using led::sin8;
using led::VariableSaw;

#if defined(__AVR_ATmega4809__)
#define LED_PIN (PIN_F0)
#endif

#define LIGHT_LEVEL_CH    (4)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH     (5)  // Sets Lights Mode (solid, spin, glow)
#define LIGHT_SUBMODE_CH  (6)  // Sets mode sub mode (color range, etc)
#define LIGHT_BRIGHT_CH   (7)  // Adjust leading edge light brightness.
#define LIGHT_THROTTLE_CH (8)  // Adjusts lights speed

#define NUM_PWM (6)

#define WNGB_CNT (21) // Each wing half
#define WNGT_CNT ( 6) // Wing Tip
#define WNGF_CNT (21) // Each wing half
#define FUSE_CNT (4) // Back Fuse

#define RGBW_CNT (2 * (WNGB_CNT + WNGF_CNT) + FUSE_CNT)
#define RGB_CNT  (2 * WNGT_CNT)

u8_t led_data[RGBW_CNT * 4 + RGB_CNT * 3];

u8_t max(u8_t a, u8_t b) { return a > b ? a : b; }

enum PwmCh {
            PWM_RBDY = 1,
            PWM_LBDY = 2,
            PWM_TAIL = 3,
            PWM_RTIP = 4,
            PWM_LTIP = 5,
};

enum Modes {
            MODE_S0 = 0,
            MODE_S1 = 1,
            MODE_S2 = 2,
            MODE_SOLID = 0 << 2,
            MODE_COLOR = 1 << 2,
            MODE_PULSE = 2 << 2
};

Modes IncMode(int m) {
  if ((m & 0x03) != 2) { m++; } // next submode
  else { // wrap, inc mode
    m = (m & 0x0C);
    if (m == 0x08) m = 0; // wrap mode
    else m += 0x04;       // inc mode
  }
  return Modes(m);
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
  CtrlState(u8_t mode, u8_t level, i16_t brt, i16_t thr)
    : mode(mode), level(level), brt(brt), thr(thr) { }

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

class ColorModeState : public LogSinOp{
public:
  ColorModeState()
    : submode_(0), brt_(0), pulse_phase_(0) { }

  void UpdateState(u8_t submode, i16_t thr, u8_t brt) {
    submode_ = submode;
    brt_ = brt;
    // Throttle is 11 bits, spd has 5 fractional bits, so this makes the
    // speed multiplier go from 0 -> 4x with throttle.
    u8_t spd = (thr + (1 <<3)) >> 5;
    if (spd < 0x07) spd = 0x07;
    color_saw_.SetSpeed(spd >> 1);
    pulse_saw_.SetSpeed(u16_t(spd) << 1);
    SetScale(1 << 6);
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
    SetColor(HsvToRgb(HSV(color_hue, 0xFF, brt_)));
    pulse_phase_ = 255 - pulse_saw_.Get(now);
  }

  void operator() (u8_t frac, RGBW* pix) const {
    frac = (u16_t(frac) * scale_) >> 6;
    *pix = color_;
    u16_t v = (sin8(frac + pulse_phase_ + offset_ ) +
               sin8(frac + pulse_phase_ + offset_ + 64));
    if (v > 255) v = 255;
    Fade(pix, Logify(v));
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
  LogSinOp op_;
};

class PulseModeState {
public:
  PulseModeState() { }
  void UpdateState(u8_t submode, i16_t thr, u8_t brt) {
    submode_ = submode;
    brt_ = brt;
    // Throttle is 11 bits, spd has 5 fractional bits, so this makes the
    // speed multiplier go from 0 -> 4x with throttle.
    u8_t spd = (thr + (1 <<3)) >> 5;
    if (spd < 0x07) spd = 0x07;
    pulse_saw_.SetSpeed(u16_t(spd) << 1);
  }
  void Update(u16_t now_ms) {
    glow_ = sin8(pulse_saw_.Get(now_ms));
  }

  const u8_t Glow() { return glow_; }

protected:
  u8_t submode_;
  u8_t brt_;
  u8_t glow_;
  VariableSaw pulse_saw_;
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
      pwm_->Enable(i, pwm_val_[i]);
    }

    void* ptr = led_data;
    ptr = lwngb_.SetSpan(ptr, WNGB_CNT, /*reverse=*/false); // core to tip
    ptr = lwngt_.SetSpan(ptr, WNGT_CNT, /*reverse=*/true);  // front to back
    ptr = lwngf_.SetSpan(ptr, WNGF_CNT, /*reverse=*/true);
    ptr = rwngf_.SetSpan(ptr, WNGF_CNT, /*reverse=*/false);
    ptr = rwngt_.SetSpan(ptr, WNGT_CNT, /*reverse=*/false);
    ptr = rwngb_.SetSpan(ptr, WNGB_CNT, /*reverse=*/true);
    ptr = fuse_.SetSpan(ptr, FUSE_CNT, /*reverse=*/false);
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
  bool StateUpdate(const CtrlState* new_state) {
    state_change_ |= state_.UpdateFromState(*new_state);
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
    pulse_mode_state_.UpdateState(Submode(), state_.thr, spon_brt_);
  }

  void Update(u16_t now) {
    // DBG_HI(APP, ("Lights::Update now: %u\n", now));
    pwm_val_[4] = spon_brt_;
    pwm_val_[PWM_LTIP] = pwm_val_[PWM_RTIP] = pwm_val_[PWM_TAIL] = spon_brt_;
    switch (Mode()) {
    case 0:
      pwm_val_[PWM_LBDY] = pwm_val_[PWM_RBDY] = brt_;
      solid_mode_state_.Update(now);
      UpdateSolidMode();
      break;
    case 1:
      pwm_val_[PWM_LBDY] = pwm_val_[PWM_RBDY] = brt_;
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
    RGB rnav = blink ? RGB(spon_brt_) : RGB(0, spon_brt_, 0);
    RGB lnav = blink ? RGB(spon_brt_) : RGB(spon_brt_, 0, 0);

    RGBW wht = RGBW(brt_);
    RGB rgb_w = RGB(brt_, brt_, brt_ >> 1);
    switch(submode) {
    case 0:
      lwngb_.Fill(wht);
      lwngt_.Fill(rgb_w);
      lwngf_.Fill(wht);
      rwngf_.Fill(wht);
      rwngt_.Fill(rgb_w);
      rwngb_.Fill(wht);
      break;
    case 1:
      lwngb_.Fill(wht, 0, 11);
      lwngb_.Fill(lnav, 11);
      lwngt_.Fill(lnav);
      lwngf_.Fill(wht, 0, 11);
      lwngf_.Fill(lnav, 11);

      rwngf_.Fill(wht, 0, 11);
      rwngf_.Fill(rnav, 11);
      rwngt_.Fill(rnav);
      rwngb_.Fill(wht, 0, 11);
      rwngb_.Fill(rnav, 11);
      break;
    case 2:
      solid_mode_state_.SetOp(lnav);
      lwngb_.FillOp(solid_mode_state_.Op());
      lwngt_.Fill(lnav);
      lwngf_.FillOp(solid_mode_state_.Op());
      solid_mode_state_.SetOp(rnav);
      rwngb_.FillOp(solid_mode_state_.Op());
      rwngt_.Fill(rnav);
      rwngf_.FillOp(solid_mode_state_.Op());
    }
  }

  void UpdateColorMode() {
    lwngt_.Fill(RGB(spon_brt_, 0, 0));
    rwngt_.Fill(RGB(0, spon_brt_, 0));

    if (Submode() == 1) {
      lwngb_.Fill(color_mode_state_.GetColor());
      lwngf_.Fill(color_mode_state_.GetColor());
      rwngb_.Fill(color_mode_state_.GetColor());
      rwngf_.Fill(color_mode_state_.GetColor());
    } else {
      lwngb_.FillOp(color_mode_state_);
      lwngf_.FillOp(color_mode_state_);
      rwngb_.FillOp(color_mode_state_);
      rwngf_.FillOp(color_mode_state_);
    }
  }

  void UpdatePulseMode() {
    u8_t submode = Submode();
    lwngt_.Fill(RGB(spon_brt_, 0, 0));
    rwngt_.Fill(RGB(0, spon_brt_, 0));

    const u8_t g = pulse_mode_state_.Glow();
    const u8_t g_brt = max(4, scale8(Logify(g), brt_));
    RGBW glow(g_brt);
    switch (submode) {
    case 1: {
      pwm_val_[PWM_LBDY] = pwm_val_[PWM_RBDY] = g_brt;
      RGBW rnav(0, spon_brt_, 0);
      RGBW lnav(spon_brt_, 0, 0);
      lwngb_.Fill(glow, 0, 15);
      lwngb_.Fill(lnav, 15);
      lwngf_.Fill(glow, 0, 14);
      lwngf_.Fill(lnav, 14);

      rwngf_.Fill(glow, 0, 15);
      rwngf_.Fill(rnav, 15);
      rwngb_.Fill(glow, 0, 14);
      rwngb_.Fill(rnav, 14);
      break;
    }
    case 2: {
      pwm_val_[PWM_LBDY] = pwm_val_[PWM_RBDY] = g_brt;
      lwngb_.Fill(glow);
      lwngf_.Fill(glow);
      rwngf_.Fill(glow);
      rwngb_.Fill(glow);
      break;
    }
    default: {
      pwm_val_[PWM_LBDY] = pwm_val_[PWM_RBDY] = 0;
      u8_t step = 128 / WNGF_CNT;
      u8_t v = 64;  // 90 deg (sin should go from 255 -> 1)
      for (int i = 0; i < WNGF_CNT; ++i, v += step) {
        u8_t s = scale8(sin8(v), g);
        RGBW lpix(max(g_brt, spon_brt_));
        RGBW rpix(max(g_brt, spon_brt_));
        Blend(&lpix, RGBW(spon_brt_, 0, 0), 255 - s);
        Blend(&rpix, RGBW(0, spon_brt_, 0), 255 - s);
        lwngf_.Set(i, lpix);
        rwngf_.Set(i, rpix);
        if (i < WNGB_CNT) {
          lwngb_.Set(i, lpix);
          rwngb_.Set(i, rpix);
        }
      }
      break;
    }
    }
  }

  // Called roughly every 32ms (32 fps)
  void Run(const CtrlState* state) {
    // DBG_HI(APP, ("Update\n"));
    now_ = FastTimeMs();
    u8_t now_8 = now_ >> 8;  // 1/4 seconds
    // Update state if forced,  sbus settings change or 1/4 sec has elapsed.
    if (StateUpdate(state) || now_8 != prev_update_) {
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

  LedSpan<RGBW> lwngb_;
  LedSpan<RGB>  lwngt_;
  LedSpan<RGBW> lwngf_;
  LedSpan<RGBW> rwngf_;
  LedSpan<RGB>  rwngt_;
  LedSpan<RGBW> rwngb_;
  LedSpan<RGBW> fuse_;
};

void ButtonCheck(Button& bright, Button& mode, Button& speed, CtrlState& state) {
  if (bright.Check() && bright.Down()) {
    if (state.brt == 2047) { // bump to next brightness level
      if (state.level == 2) { state.level = 0; state.brt = 0; }// turn off
      else { state.level++; state.brt = 1024; }
    } else if (state.brt == 1024) { state.brt = 2047; }
    else { state.level = 1; state.brt = 1024; } // was off go to first level
    state.Log("Bright");
  }
  if (mode.Check() && mode.Down()) {
    state.mode = IncMode(state.mode);
    state.Log("Mode");
  }
  if (speed.Check() && speed.Down()) {
    state.thr += 255;
    if (state.thr >= 2048) state.thr = 0;
    state.Log("Speed");
  }
}

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_HI(APP);
  DBG_LEVEL_MD(SBUS);
  SBus sbus(&Serial::usart2, /*invert=*/false);

  PinId led_pin(LED_PIN);
  led_pin.SetOutput();
  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();

  Button bright_button(PIN_C4);
  Button mode_button(PIN_C5);
  Button speed_button(PIN_C6);

  Pwm pwm(PORT_D, 400);
  Lights lights(LED_PIN, &pwm);

  sei();
  DBG_MD(APP, ("FT_Sportster: Run\n"));

  memset(led_data, 0, sizeof(led_data));
  SendWS2812(LED_PIN, led_data, sizeof(led_data), 0xFF);

  CtrlState state(MODE_SOLID | MODE_S1, 1, 1000, 1000);

  u8_t update_0 = 0;
  u8_t update_5 = 0;
  u8_t update_8 = 0;
  u8_t update_5s = 0;
  while (1) {
    u16_t now = FastMs();
    if (sbus.Run()) {
      state.Set(&sbus);  // Could call lights.Run if state changed...
      state.Log("Sbus");
    }
    if (update_0 == now) continue;
    update_0 = now;
    // ButtonCheck(bright_button, mode_button, speed_button, state);

    const u8_t now_5 = now >> 5;
    if (now_5 == update_5) continue;
    update_5 = now_5;
    lights.Run(&state);

    const u8_t now_8 = now >> 8;
    if (now_8 == update_8) continue;
    update_8 = now_8;
    u8_t phase = (now_8 & 0x07);
    u8_t pat = lights.GetBlinkPattern();
    blink_pin.set(pat & (1 << phase));
    u16_t now_s = FastSecs();
    u8_t now_5s = now_s >> 5;
    if (now_5s == update_5s) continue;
    // Every 32s...
  }
}
