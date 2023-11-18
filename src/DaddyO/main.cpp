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
#include "Pca9685.h"
#include "Pwm.h"
#include "SBus.h"
#include "Serial.h"
#include "Twi.h"
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
#define RGB_LED_PIN (PIN_F0)
#endif

#define LIGHT_LEVEL_CH    (5)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH     (6)  // Sets Lights Mode (solid, spin, glow)
#define LIGHT_SUBMODE_CH  (7)  // Sets mode sub mode (color range, etc)
#define LIGHT_BRIGHT_CH   (8)  // Adjust leading edge light brightness.
#define LIGHT_THROTTLE_CH (9)  // Adjusts lights speed

// PWM LEDS: LEDS 0-2 body, 3 cockpit, 4-7 tail, 89 = wing
enum PwmCh {
            PWM_BODY_F = 0,
            PWM_BODY_M = 1,
            PWM_BODY_L = 2,
            PWM_COCKPIT = 3,
            PWM_RUDDER = 4,
            PWM_ELE_W = 5,
            PWM_ELE_B = 6,
            PWM_ELE_N = 7,
            PWM_WING_L = 8,
            PWM_WING_R = 9,
            PWM_CNT = 10,
};


#define WNGB_CNT (23) // Each wing half (7x 3led, 1x 2led)
#define WNGF_CNT (28) // Each wing half ( 5x 4led, 2x 3led, 1x 2led)

#define RGBW_CNT (2 * (WNGB_CNT + WNGF_CNT))
#define RGB_CNT  (0)

u8_t led_data[RGBW_CNT * 4 + RGB_CNT * 3];

u8_t max(u8_t a, u8_t b) { return a > b ? a : b; }

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

class FadeLogSinOp {
public:
  FadeLogSinOp() : scale_(1<<6), offset_(0) { }
  FadeLogSinOp(u8_t scale, u8_t offset = 0)
    :scale_(scale), offset_(offset) { }
  void Set(u8_t scale, u8_t offset) {
    scale_ = scale;
    offset_ = offset;
  }
  u8_t GetOffset() const { return offset_; }
  void SetOffset(u8_t offset) { offset_ = offset; }
  // Scale is 2.6 FP number
  u8_t GetScale() const { return scale_; }
  void SetScale(u8_t scale) { scale_ = scale; }

  void operator() (u8_t frac, RGBW* pix) const {
    frac = (u16_t(frac) * scale_) >> 6;
    Fade(pix, Logify(sin8(frac + offset_ )));
  }

  u8_t scale_;
  u8_t offset_;
};

class ClrLogSinOp : public FadeLogSinOp {
public:
  ClrLogSinOp() { }
  ClrLogSinOp(RGBW color, u8_t scale = (1<<6), u8_t offset = 0)
    : FadeLogSinOp(scale, offset), color_(color) { }
  void Set(const RGBW& color, u8_t scale, u8_t offset) {
    color_ = color;
    this->FadeLogSinOp::Set(scale, offset);
  }
  void SetColor(const RGBW& color) { color_ = color; }
  RGBW GetColor() const { return color_; }
  void operator() (u8_t frac, RGBW* pix) const {
    *pix = color_;
    this->FadeLogSinOp::operator()(frac, pix);
  }

  RGBW color_;
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
  const FadeLogSinOp& Op() const { return op_; }

  // Scale is 2.6 FP number
  void SetOp() {
    op_.Set(1<<6, wave_phase_);
  }

protected:
  u8_t brt_;
  u8_t scale_;
  u8_t wave_phase_;
  u8_t blink_sec_;
  bool blink_;
  RGBW color_;
  VariableSaw wave_saw_;
  FadeLogSinOp op_;
};

class ColorModeState : public ClrLogSinOp{
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
  Lights(PinId rgb_led_pin, Pca9685* pwm) :
    now_(0), prev_update_(0),
    led_pin_(rgb_led_pin), pwm_(pwm),
    mode_(0), brt_(0), spon_brt_(0),
    state_change_(CtrlState::CHG_NONE) {

    void* ptr = led_data;
    ptr = rwngb_.SetSpan(ptr, WNGB_CNT, /*reverse=*/false); // core to tip
    ptr = rwngf_.SetSpan(ptr, WNGF_CNT, /*reverse=*/true);
    ptr = lwngf_.SetSpan(ptr, WNGF_CNT, /*reverse=*/false);
    ptr = lwngb_.SetSpan(ptr, WNGB_CNT, /*reverse=*/true);
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
  void Push() {
    PushLeds();
    pwm_->Write();
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
    case 1: spon_brt_ = 0x40; brt_ = brt_ >> 2; break;
    case 2: spon_brt_ = 0xFF; break;
    }

    DBG_HI(APP, ("Update Brt: %d Spn: %d lvl: %d\n", brt_, spon_brt_, lvl));
  }

  void SetFlagMode(u8_t brt) {
    RGBW wht(brt);
    RGBW blu(0,0,brt);
    RGBW red(brt,0,0);
    for (u8_t i = 0; i < 20;) {
      rwngf_.At(i) = lwngf_.At(i)= wht; ++i;
      rwngf_.At(i) = lwngf_.At(i)= wht; ++i;
      rwngf_.At(i) = lwngf_.At(i)= blu; ++i;
      rwngf_.At(i) = lwngf_.At(i)= blu; ++i;
    }
    rwngf_.Fill(wht, 20, 3);
    lwngf_.Fill(wht, 20, 3);
    for (u8_t i = 0; i < 18; i += 6) {
      rwngb_.Fill(red, i, 3);
      lwngb_.Fill(red, i, 3);
      rwngb_.Fill(wht, i + 3, 3);
      lwngb_.Fill(wht, i + 3, 3);
    }
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
  void PwmSetBody(u8_t apparent, u8_t nav_brt) {
    u16_t val = pwm_->Apparent2Pwm(apparent);
    pwm_->led(PWM_BODY_F) = pwm_->led(PWM_BODY_M) = pwm_->led(PWM_BODY_L) = val;
    pwm_->led(PWM_COCKPIT) = pwm_->led(PWM_RUDDER) = pwm_->led(PWM_ELE_B) = val;
    if (nav_brt == 0) {
      pwm_->led(PWM_ELE_N) = 0;
      pwm_->led(PWM_ELE_W) = val;
    } else {
      u16_t nval = pwm_->Apparent2Pwm(nav_brt);
      pwm_->led(PWM_ELE_N) = nval;
      pwm_->led(PWM_ELE_W) = 0;
    }
  }

  void PwmSetWing(u8_t apparent) {
    u16_t val = pwm_->Apparent2Pwm(apparent);
    pwm_->led(PWM_WING_L) = pwm_->led(PWM_WING_R) = val;
  }

  void Update(u16_t now) {
    // DBG_HI(APP, ("Lights::Update now: %u\n", now));
    // pwm_val_[4] = spon_brt_;
    // pwm_val_[PWM_LTIP] = pwm_val_[PWM_RTIP] = pwm_val_[PWM_TAIL] = spon_brt_;
    switch (Mode()) {
    case 0:  // MODE_SOLID
      // pwm_val_[PWM_LBDY] = pwm_val_[PWM_RBDY] = brt_;
      solid_mode_state_.Update(now);
      UpdateSolidMode();
      break;
    case 1:  // MODE_COLOR
      // pwm_val_[PWM_LBDY] = pwm_val_[PWM_RBDY] = brt_;
      color_mode_state_.Update(now);
      UpdateColorMode();
      break;
    case 2:  // MODE_PULSE
      pulse_mode_state_.Update(now);
      UpdatePulseMode();
      break;
    }
    Push();
  }

  void UpdateSolidMode() {
    u8_t submode = Submode();
    RGBW rnav(0, spon_brt_, 0);
    RGBW lnav(spon_brt_, 0, 0);
    PwmSetWing(spon_brt_);


    RGBW wht = RGBW(brt_);

    switch(submode) {
    case 0:
      PwmSetBody(brt_, spon_brt_);
      SetFlagMode(brt_);
      lwngb_.Fill(lnav, 18);
      lwngf_.Fill(lnav, 23);
      rwngb_.Fill(rnav, 18);
      rwngf_.Fill(rnav, 23);
      break;

    case 1:
      PwmSetBody(brt_, 0);
      SetFlagMode(brt_);
      lwngb_.Fill(wht, 18);
      lwngf_.Fill(wht, 23);
      rwngb_.Fill(wht, 18);
      rwngf_.Fill(wht, 23);
      break;
    case 2:
      PwmSetBody(brt_, spon_brt_);
      SetFlagMode(255);
      solid_mode_state_.SetOp();
      lwngb_.FillOp(solid_mode_state_.Op());
      lwngf_.FillOp(solid_mode_state_.Op());
      rwngb_.FillOp(solid_mode_state_.Op());
      rwngf_.FillOp(solid_mode_state_.Op());
    }
  }

  void UpdateColorMode() {
    PwmSetWing(spon_brt_);

    RGBW rnav(0, spon_brt_, 0);
    rwngb_.Fill(rnav, 18);
    rwngf_.Fill(rnav, 23);

    RGBW lnav(spon_brt_, 0, 0);
    lwngb_.Fill(lnav, 18);
    lwngf_.Fill(lnav, 23);

    RGBW wht = RGBW(brt_);
    switch(Submode()) {
    case 0:
      PwmSetBody(brt_, spon_brt_);
      lwngb_.Fill(wht, 0, 18);
      lwngf_.Fill(wht, 0, 23);
      rwngb_.Fill(wht, 0, 18);
      rwngf_.Fill(wht, 0, 23);
      break;
    case 1: {
      PwmSetBody(brt_, spon_brt_);
      RGBW clr = color_mode_state_.GetColor();
      lwngb_.Fill(clr, 0, 18);
      lwngf_.Fill(clr, 0, 23);
      rwngb_.Fill(clr, 0, 18);
      rwngf_.Fill(clr, 0, 23);
    }
      break;
    case 2:
      PwmSetBody(brt_, 0);
      lwngb_.FillOp(color_mode_state_, 0, 18);
      lwngf_.FillOp(color_mode_state_, 0, 23);
      rwngb_.FillOp(color_mode_state_, 0, 18);
      rwngf_.FillOp(color_mode_state_, 0, 23);
      break;
    }
  }

  void UpdatePulseMode() {
    u8_t submode = Submode();

    const u8_t g = pulse_mode_state_.Glow();
    const u8_t g_brt = max(4, scale8(Logify(g), brt_));
    RGBW glow(g_brt);
    switch (submode) {
    case 1: {
      // pwm_val_[PWM_LBDY] = pwm_val_[PWM_RBDY] = g_brt;
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
      // pwm_val_[PWM_LBDY] = pwm_val_[PWM_RBDY] = g_brt;
      lwngb_.Fill(glow);
      lwngf_.Fill(glow);
      rwngf_.Fill(glow);
      rwngb_.Fill(glow);
      break;
    }
    default: {
      // pwm_val_[PWM_LBDY] = pwm_val_[PWM_RBDY] = 0;
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
  Pca9685* pwm_;
  u8_t mode_, brt_, spon_brt_;
  CtrlState state_;
  SolidModeState solid_mode_state_;
  ColorModeState color_mode_state_;
  PulseModeState pulse_mode_state_;
  u8_t state_change_;

  LedSpan<RGBW> lwngb_;
  LedSpan<RGBW> lwngf_;
  LedSpan<RGBW> rwngf_;
  LedSpan<RGBW> rwngb_;
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
  SBus sbus(&Serial::usart1, /*invert=*/true);

  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();
  blink_pin.toggle();

  PinId rgb_led_pin(RGB_LED_PIN);
  rgb_led_pin.SetOutput();

  Twi::twi.Setup(Twi::PINS_DEF, Twi::I2C_1M);
  Pca9685 pwm16(0x80, 16); // Uses Twi

  sei();
  DBG_MD(APP, ("DaddyO: Run\n"));

  memset(led_data, 0, sizeof(led_data));
  SendWS2812(rgb_led_pin, led_data, sizeof(led_data), 0xFF);

  pwm16.Init(/*totem=*/true);
  Lights lights(rgb_led_pin, &pwm16);

  CtrlState state;
  u8_t blink_phase = 0;

  u8_t update_0 = 0;
  u8_t update_5 = 0;
  u8_t update_8 = 0;
  // u8_t update_16s = 0xFF;

  while (1) {
    u16_t now = FastMs();
    if (sbus.Run()) {
      state.Set(&sbus);  // Could call lights.Run if state changed...
      state.Log("Sbus");
    }
    if (update_0 == now) continue;  // 1ms
    update_0 = now;

    const u8_t now_5 = now >> 5;   // 1/32 sec
    if (now_5 == update_5) continue;
    update_5 = now_5;
    lights.Run(&state);

    const u8_t now_8 = now >> 8;
    if (now_8 == update_8) continue;  // 1/4 sec
    update_8 = now_8;
    u8_t pat = lights.GetBlinkPattern();
    blink_pin.set(!(pat & (1 << blink_phase)));  // pin high is off.
    blink_phase = (blink_phase + 1) & 0x07;

#if 0
    u16_t now_s = FastSecs();
    u8_t now_16s = now_s >> 4;
    if (now_16s == update_16s) continue;  // 16 sec
    update_16s = now_16s;
    switch (now_16s & 0x03) {
    case 0:
      state = CtrlState(MODE_SOLID | MODE_S0, 1, 1000, 1000);
      break;;
    case 1:
      state = CtrlState(MODE_SOLID | MODE_S1, 1, 1000, 1000);
      break;;
    case 2:
      state = CtrlState(MODE_SOLID | MODE_S2, 1, 1000, 1000);
      break;;
    case 3:
      state = CtrlState(MODE_SOLID | MODE_S0, 0, 1000, 1000);
      break;;
    }
#endif
  }
}
