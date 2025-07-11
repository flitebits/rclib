// Copyright 2021, 2022 Thomas DeWeese
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
using led::Logify;
using led::HSV;
using led::RGBW;
using led::RGB;
using led::sin8;
using led::VariableSaw;
using dbg::DbgCmds;
using dbg::CmdHandler;

#define LIGHT_BRIGHT_CH (10)  // Adjust leading edge light brightness.
#define LIGHT_LEVEL_CH (11)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH (12)  // Sets Lights Mode (solid, spin, glow)
#define LIGHT_THROTTLE_CH (13)  // Adjusts lights speed


enum PwmChannels{
      PWM_TAIL = 0,
      PWM_ELEV = 1,
      PWM_RFUSE = 2,
      PWM_LFUSE = 3,
      PWM_WING = 4,
      PWM_NAV = 5,
      PWM_RFSTRIP = 12,
      PWM_RBSTRIP = 13,
      PWM_LFSTRIP = 14,
      PWM_LBSTRIP = 15,
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
    u8_t mode = (SBus::ThreePosSwitch(sbus->GetChannel(LIGHT_MODE_CH)) << 2);
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

class Lights {
public:
  Lights(Pca9685* pwm) :
    pwm_(pwm), mode_(0), brt_(0), spon_brt_(0) { }

  const CtrlState& State() const { return state_; }
  void ApplyState(const CtrlState& new_state) {
    state_.UpdateFromState(new_state);
    ApplyUpdatedState();
  }

  u8_t Mode() { return mode_ >> 2; }

  void Push() {
    pwm_->Write();
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
    DBG_LO(APP, ("Update Brt: %d Spn: %d\n", brt_, spon_brt_));
  }

  void SetFuse(u16_t strip, u16_t fuse) {
    pwm_->SetLed(fuse, PWM_RFUSE);
    pwm_->SetLed(fuse, PWM_LFUSE);

    pwm_->SetLed(strip, PWM_RBSTRIP);
    pwm_->SetLed(strip, PWM_RFSTRIP);
    pwm_->SetLed(strip, PWM_LBSTRIP);
    pwm_->SetLed(strip, PWM_LFSTRIP);
  }

  void SetNav(u16_t val) {
    pwm_->SetLed(val, PWM_TAIL);
    pwm_->SetLed(val, PWM_NAV);
  }

  // Solid mode is body on, wing edges white, sponsons R/G.
  void UpdateSolidMode() {
    const u16_t spon_val = Pca9685::Apparent2Pwm(spon_brt_);
    SetNav(spon_val);
    pwm_->SetLed(spon_val, PWM_WING);

    const u16_t val = Pca9685::Apparent2Pwm(brt_);
    SetFuse(val, val);
    pwm_->SetLed(val, PWM_ELEV);
  }

  // Color mode
  void UpdateColorMode() {
    const u16_t spon_val = Pca9685::Apparent2Pwm(spon_brt_);
    SetNav(spon_val);

    const u16_t val = Pca9685::Apparent2Pwm(brt_);
    SetFuse(val, 0);
    pwm_->SetLed(val, PWM_WING);
    pwm_->SetLed(val, PWM_ELEV);
  }

  // Pulse mode
  void UpdatePulseMode() {
    const u16_t spon_val = Pca9685::Apparent2Pwm(spon_brt_);
    SetNav(spon_val);
    pwm_->SetLed(spon_val, PWM_WING);

    const u16_t val = Pca9685::Apparent2Pwm(brt_);
    SetFuse(val, 0);
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
  }

  void Update(u16_t now) {
    DBG_HI(APP, ("Lights::Update now: %u\n", now));
    ApplyUpdatedState();


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

  Pca9685* const pwm_;
  u8_t state_change_;
  u8_t mode_, brt_, spon_brt_;
  CtrlState state_;
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
  Twi::twi.Setup(Twi::PINS_DEF, Twi::I2C_1M);

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  DBG_LEVEL_HI(SBUS);
  SBus sbus(&Serial::usart2, /*invert=*/true);

  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();

  sei();
  DBG_MD(APP, ("Eclipson Model B XL: Startup\n"));

  Pca9685 pwm(0x80, 16);
  pwm.Init(/*totem=*/true);
  pwm.SetLeds(0, 0, 16);
  pwm.Write();
  Lights lights(&pwm);

  DbgCmds cmds(&Serial::usart0);
  VARCMDS_INIT(cmds);
  PwmCmdHandler pwm_cmd(pwm);
  cmds.RegisterHandler(&pwm_cmd);
  LightsCmd lights_cmd(&lights);
  cmds.RegisterHandler(&lights_cmd);

  DBG_MD(APP, ("Eclipson Model B XL: Running\n"));
  u8_t update_3 = 0;
  u8_t update_5 = 0;
  u8_t update_8 = 0;
  while (1) {
    const u16_t now = FastTimeMs();
    if (sbus.Run()) {
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
