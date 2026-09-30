// Copyright 2021, 2022, 2026 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <ctype.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "Boot.h"
#include "DbgCmds.h"
#include "VarCmds.h"
#include "IntTypes.h"
#include "RtcTime.h"

#include "leds/Clr.h"
#include "leds/FPMath.h"
#include "leds/LedSpan.h"
#include "leds/Pixel.h"
#include "leds/Rgb.h"

#include "Pca9685.h"
#include "Pins.h"
#include "Pwm.h"
#include "SBus.h"
#include "Serial.h"
#include "Twi.h"
#include "WS2812.h"

using led::bscale8;
using led::HSV;
using led::Logify;
using led::RGBW;
using led::RGB;
using led::sin8;
using led::VariableSaw;
using dbg::DbgCmds;
using dbg::CmdHandler;

#if defined(__AVR_ATmega4809__)
#define LED_PIN (PIN_F0)
#endif

#define LIGHT_BRIGHT_CH (10)  // Adjust leading edge light brightness.
#define LIGHT_LEVEL_CH (11)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH (12)  // Sets Lights Mode (solid, spin, glow)
#define LIGHT_SUBMODE_CH (13)  // Sets mode sub mode (color range, etc)
#define LIGHT_THROTTLE_CH (14)  // Adjusts lights speed

#define LED_TOPL_CNT (20)
#define LED_TOPH_CNT (16)
#define LED_MOTOR_CNT (38)

#define RGBW_CNT (0)
#define RGB_CNT  (LED_TOPL_CNT + LED_TOPH_CNT + 2 * LED_MOTOR_CNT)

u8_t led_data[RGBW_CNT * 4 + RGB_CNT * 3];

enum PwmChannels{
  PWM_WING_LR = 0,  // Left/right wing tips
  PWM_RUDDER  = 1,  // Rudder front lighs
  PWM_ELEV_LR = 2,  // Left/right elev tips
  PWM_BOOM_B  = 3,  // Tail boom side, wing back lights
  PWM_FUSE    = 6,  // Fusilage side lights
  PWM_TAIL_C  = 7   // tail center disk lights
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
    const u8_t mode =
      ((ThreePosSwitch(sbus->GetChannel(LIGHT_MODE_CH)) << 2) |
       ThreePosSwitch(sbus->GetChannel(LIGHT_SUBMODE_CH)));
    CtrlState new_state(
      mode,
      ThreePosSwitch(sbus->GetChannel(LIGHT_LEVEL_CH)),
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
    DBG_MD(APP, ("State: L:%d M:%02x B:%d T:%d\n",
		 level, mode, brt, thr));
    return change;
  }
};

const u8_t color_spin_spd_map[] = {0x20, 0x50, 0x70, 0xA0, 0xFF};
class ColorSpin {
public:
  void Set(u8_t submode, u8_t brt, i16_t thr) {
    submode_ = submode;
    brt_ = brt;
    // Throttle is 11 bits
    const u8_t thr8 = thr >> 3;
    const u8_t spd_idx = thr8 >> 6;
    const u8_t spd_frac = thr8 & 0x3F;
    // Map speed through spd_map
    u8_t c_spd = color_spin_spd_map[spd_idx] +
      ((u16_t(color_spin_spd_map[spd_idx + 1] - color_spin_spd_map[spd_idx]) *
	spd_frac) >> 6);
    color_saw_.SetSpeed(c_spd);
    
    u8_t p_spd = (thr + (1 <<3)) >> 4;
    if (p_spd < 0x07) p_spd = 0x07;
    pulse_saw_.SetSpeed(p_spd);
    DBG_MD(APP, ("ColorSpin: M: %d B: %d c_spd: %d p_spd: %d\n",
		 submode_, brt_, c_spd, p_spd));
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
    u8_t pulse_phase_ = 255 - pulse_saw_.Get(now);

    color_ = HsvToRgb(HSV(color_hue, 0xFF, brt_));
    Fade(&color_, Logify(sin8(pulse_phase_)));
  }

  RGB& color() { return color_; }
  
protected:
  u8_t submode_;
  u8_t brt_;
  VariableSaw color_saw_;
  VariableSaw pulse_saw_;
  RGB color_;
};

class SinSpin {
public:
  SinSpin() : f_scale_(1 << 8) {}

  void set_color(RGB rgb) {
    color_ = rgb;
  }
  RGB& mutable_color() {
    return color_;
  }
  const RGB& color() const {
    return color_;
  }
  void shift_phase(u8_t shift) {
    phase_ += shift;
  }
		  
  void UpdateState(i16_t thr, u8_t brt) {
    brt_ = brt;
    // Throttle is 11 bits, spd has 5 fractional bits, so this makes the
    // speed multiplier go from 0 -> 4x with throttle.
    u8_t spd = (thr + (1 <<3)) >> 4;
    if (spd < 0x07) spd = 0x07;
    phase_saw_.SetSpeed(u16_t(spd) << 1);
    u8_t frac = 128;
    frac = (frac * f_scale_ + (1 << 7)) >> 8;
    DBG_HI(APP, ("SinSpin: B: %d S: %d fs: %d F: %d\n",
		 brt, spd, f_scale_, frac));
  }

  void Update(u16_t now) {
    u32_t s_now = (u32_t(now) * f_scale_) >> 8;
    phase_ = 255 - phase_saw_.Get(s_now);
  }

  // 8.8 fixed point
  void SetFracScale(u16_t f_scale) {
    f_scale_ = f_scale;
  }
  
  void operator() (u8_t frac, RGB* pix) const {
    frac = (frac * f_scale_ + (1 << 7)) >> 8;
    *pix = color_;
    Fade(pix, bscale8(Logify(sin8(phase_ + frac)), brt_));
  }

  RGB color_;
  VariableSaw phase_saw_;
  u8_t phase_;
  u8_t brt_;
  u16_t f_scale_;
};


class Lights {
public:
  Lights(PinId led_pin, Pca9685* pwm) :
    led_pin_(led_pin), pwm_(pwm), mode_(0), brt_(0), wtip_brt_(0) {
    spinner_.SetFracScale(u16_t(3) << 8);
    void* ptr = led_data;
    ptr = led_topl_.SetSpan(ptr, LED_TOPL_CNT, false);
    ptr = led_toph_.SetSpan(ptr, LED_TOPH_CNT, false);
    ptr = led_engl_.SetSpan(ptr, LED_MOTOR_CNT, true);
    ptr = led_engr_.SetSpan(ptr, LED_MOTOR_CNT, false);
  }

  const CtrlState& State() const { return state_; }
  void ApplyState(const CtrlState& new_state) {
    state_.UpdateFromState(new_state);
    ApplyUpdatedState();
  }

  u8_t Mode() { return mode_ >> 2; }
  u8_t Submode() { return mode_ & 0x03; }

  void Push() {
    SendWS2812(led_pin_, led_data, sizeof(led_data), 0xFF);
    pwm_->Write();
  }

  // lvl is overall brightness mode (off, med, hi)
  // sbrt is 11 bit brightness slider affects 'add-ins'
  // not critical lights light wing tip/sponsons.
  void UpdateBright(u8_t lvl, u16_t sbrt) {
    brt_ = 0;
    if (sbrt > 32) {
      sbrt = sbrt >> 3;  // 11 to 8 bits
      brt_ = (sbrt > 255) ? 255 : sbrt;  // clip at 255
    }

    switch (lvl) {
    case 0: brt_ = wtip_brt_ = 0; break;
    case 1: wtip_brt_ = 0x40; brt_ = brt_ >> 3; break;
    case 2: wtip_brt_ = 0xFF; break;
    }
    DBG_HI(APP, ("Update Brt: %d Spn: %d\n", brt_, wtip_brt_));
  }

  void SetFuse(u16_t fuse, u16_t ctail) {
    pwm_->SetLed(fuse, PWM_FUSE);
    pwm_->SetLed(fuse, PWM_BOOM_B);

    pwm_->SetLed(ctail, PWM_TAIL_C);
  }

  void SetNav(u16_t val) {
    pwm_->SetLed(val, PWM_ELEV_LR);
    pwm_->SetLed(val, PWM_WING_LR);
  }

  // Solid mode is body on, wing edges white, Wingtip R/G.
  void UpdateSolidMode() {
    const u16_t val = Pca9685::Apparent2Pwm(brt_);
    SetFuse(/*fuse=*/val, /*ctail=*/val);
    pwm_->SetLed(val, PWM_RUDDER);
    
    RGB wht_b(brt_);
    switch (Submode()) {
    case 0:
      led_topl_.Fill(wht_b); // white
      led_toph_.Fill(wht_b); // white
      led_engl_.Fill(wht_b);
      led_engr_.Fill(wht_b);
      break;
      break;
    case 1:
      spinner_.set_color(wht_b);
      led_engl_.FillOp(spinner_);
      led_engr_.FillOp(spinner_);
      spinner_.set_color(led::wclr::blue);
      led_topl_.FillOp(spinner_);
      spinner_.shift_phase(127);
      led_toph_.FillOp(spinner_);
      break;
    case 2:
      led_topl_.Fill(led::wclr::blue);
      led_toph_.Fill(led::wclr::blue);
      led_engl_.Fill(RGB(wtip_brt_, 0, 0));  // red
      led_engr_.Fill(RGB(0, wtip_brt_, 0));  // grn
      break;
    }
  }

  // Color mode
  void UpdateColorMode() {
    const u16_t wtip_val = Pca9685::Apparent2Pwm(wtip_brt_);
    
    RGB wht_b(brt_); // white
    led_topl_.Fill(wht_b);
    led_toph_.Fill(wht_b);
    switch (Submode()) {
    case 0:
    case 2:
      spinner_.set_color(wht_b);
      led_engl_.FillOp(spinner_);
      led_engr_.FillOp(spinner_);
      break;
    case 1:
      spinner_.set_color(RGB(wtip_brt_, 0, 0)); // red
      led_engl_.FillOp(spinner_);
      spinner_.set_color(RGB(0, wtip_brt_, 0)); // green
      led_engr_.FillOp(spinner_);
      break;
    }
    const u16_t val = Pca9685::Apparent2Pwm(brt_);
    const u16_t f_val = Submode() == 2 ? val : wtip_val;
    SetFuse(/*fuse=*/f_val, /*ctail=*/val);
    pwm_->SetLed(f_val, PWM_RUDDER);
  }

  // Pulse mode
  void UpdatePulseMode() {
    const u16_t wtip_val = Pca9685::Apparent2Pwm(wtip_brt_);
    const u16_t val = Pca9685::Apparent2Pwm(brt_);

    RGB wht_b(brt_); // white
    RGB pulse = clr_spin_.color();
    switch (Submode()) {
    case 0:
      led_topl_.Fill(wht_b);
      led_toph_.Fill(wht_b);
      led_engl_.Fill(pulse);
      led_engr_.Fill(pulse);
      SetFuse(/*fuse=*/wtip_val, /*ctail=*/val);
      pwm_->SetLed(wtip_val, PWM_RUDDER);
    break;
    case 1:
      led_topl_.Fill(pulse);
      led_toph_.Fill(pulse);
      led_engl_.Fill(pulse);
      led_engr_.Fill(pulse);
      SetFuse(/*fuse=*/val, /*ctail=*/val);
      pwm_->SetLed(val, PWM_RUDDER);
    break;
    case 2:
      led_topl_.Fill(pulse);
      led_toph_.Fill(pulse);
      led_engl_.Fill(pulse);
      led_engr_.Fill(pulse);
      SetFuse(/*fuse=*/val, /*ctail=*/val);
      pwm_->SetLed(val, PWM_RUDDER);
      break;
    }
  }

  // Returns true if the state_ was changed.
  bool SBusUpdate(SBus* sbus) {
    state_change_ |= state_.Set(sbus);
    return (state_change_ != CtrlState::CHG_NONE);
  }

  void ApplyUpdatedState() {
    mode_ = state_.mode;
    state_change_ = 0;
    UpdateBright(state_.level, state_.brt);
    spinner_.UpdateState(state_.thr, 255);
    clr_spin_.Set(Submode(), state_.brt, state_.thr);
  }

  void Update(u16_t now) {
    DBG_HI(APP, ("Lights::Update now: %u\n", now));
    ApplyUpdatedState();
    spinner_.Update(now);
    clr_spin_.Update(now);

    const u16_t wtip_val = Pca9685::Apparent2Pwm(wtip_brt_);
    SetNav(wtip_val);
    
    switch (Mode()) {
    case 0:
      UpdateSolidMode();
      break;
    case 1:
      UpdateColorMode();
      break;
    case 2:
      UpdatePulseMode();
      break;
    }
    Push();
  }

  PinId led_pin_;
  Pca9685* const pwm_;
  u8_t state_change_;
  u8_t mode_, brt_, wtip_brt_;
  SinSpin spinner_;
  ColorSpin clr_spin_;
  VariableSaw color_saw_;
  CtrlState state_;
  LedSpan<RGB>  led_topl_, led_toph_, led_engl_, led_engr_;
};

class PwmCmdHandler : public CmdHandler {
public:
  explicit PwmCmdHandler(Pca9685& pwm) :
    CmdHandler("pwm"), pwm_(&pwm) {}

  virtual void HandleLine(const char* args) {
    int led, val;
    int cnt = sscanf(args, "%d %d", &led, &val);
    if (cnt != 2) {
      DBG_LO(APP, ("Unable to scan 2 ints, found %d ints\n", cnt));
      return;
    }
    pwm_->SetLed(val, led);
    pwm_->Write();
    for (int i = 0; i < 16; ++i) {
      DBG_LO(APP, ("PWM[%d]: %04X\n", i, pwm_->led(i)));
    }
  }
  Pca9685* pwm_;
};

class LightsCmd : public CmdHandler {
public:
  LightsCmd(Lights* lights)
    : CmdHandler("lights"), lights_(lights) { }

  virtual void HandleLine(const char* args) {
    DBG_MD(APP, ("Lights HandleLine: %s\n", args));
    CtrlState state = lights_->State();
    int iter = 0;
    while (*args) {
      while (isspace(*args)) ++args;
      if (!args[0] || !args[1]) break;
      int len = 0;
      int val;
      int cnt = sscanf(args + 2, "%d%n", &val, &len);
      if (cnt != 1) break;
      ++iter;
      switch (*args) {
      case 'm': case 'M':
        state.mode = val;
        break;
      case 'l': case 'L':
        state.level = val;
        break;
      case 'b': case 'B':
        state.brt = val;
        break;
      case 't': case 'T':
        state.thr = val;
        break;
      default:
        break;
      }
      args += 2 + len;
    }
    if (iter != 0) {
      lights_->ApplyState(state);
    }
  }

private:
  Lights* lights_;
};

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  DBG_LEVEL_HI(SBUS);
  DBG_LEVEL_MD(TWI);

  SBus sbus(&Serial::usart2, /*invert=*/true);

  PinId led_pin(LED_PIN);
  led_pin.SetOutput();
  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();

  sei();
  DBG_MD(APP, ("FT SeaDuck: Startup\n"));

  memset(led_data, 0, sizeof(led_data));
  SendWS2812(LED_PIN, led_data, sizeof(led_data), 0xFF);

  Twi::twi.Setup(Twi::PINS_DEF, Twi::I2C_400K);
  Pca9685 pwm(0x80, 16);
  pwm.Init(/*totem=*/true);
  pwm.SetLeds(0, 0, 16);
  pwm.Write();
  Lights lights(LED_PIN, &pwm);

  DbgCmds cmds(&Serial::usart0);
  VARCMDS_INIT(cmds);
  PwmCmdHandler pwm_cmd(pwm);
  cmds.RegisterHandler(&pwm_cmd);
  LightsCmd lights_cmd(&lights);
  cmds.RegisterHandler(&lights_cmd);

  DBG_MD(APP, ("FT SeaDuck: Running\n"));
  u8_t update_3 = 0;
  u8_t update_5 = 0;
  u8_t update_8 = 0;
  while (1) {
    const u16_t now = FastTimeMs();
    if (sbus.Run()) {
      // sbus.Dump();
      lights.SBusUpdate(&sbus);
    }

    const u8_t now_3 = now >> 3;
    if (now_3 == update_3) continue;
    update_3 = now_3;
    cmds.Run();

    const u8_t now_5 = now >> 5;
    if (now_5 == update_5) continue;
    update_5 = now_5;
    lights.Update(now);

    const u8_t now_8 = now >> 8;
    if (now_8 == update_8) continue;
    update_8 = now_8;
    blink_pin.toggle();
  }
}
