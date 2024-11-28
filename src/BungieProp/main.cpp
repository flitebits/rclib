// Copyright 2021, 2022, 2024 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <util/atomic.h>

#include "Boot.h"
#include "Dbg.h"
#include "DbgCmds.h"
#include "IntTypes.h"
#include "RtcTime.h"
#include "VarCmds.h"

#include "leds/FPMath.h"

#include "Pins.h"
#include "Pwm.h"
#include "SBus.h"
#include "Serial.h"
#include "WS2812.h"

using dbg::CmdHandler;
using dbg::DbgCmds;
using led::VariableSaw;
using led::bscale8;
using led::Logify;
using led::sin8;

#define LIGHT_LEVEL_CH    (4)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH     (5)  // Sets Lights Mode (solid, spin, glow)
#define LIGHT_BRIGHT_CH   (6)  // Adjust leading edge light brightness.
#define LIGHT_THROTTLE_CH (7)  // Adjusts lights speed

#define NUM_PWM (4)

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

class PulseModeState {
public:
  PulseModeState() {}

  void UpdateState(i16_t thr) {
    u8_t spd = GetSpd(thr);
    glow_saw_.SetSpeed(spd);
  }

  void Update(u16_t now) {
    lvl_ = Logify(sin8(glow_saw_.Get(now)));
  }
  const u8_t& GetLvl() { return lvl_; }

  u8_t submode_;
  u8_t lvl_;
  VariableSaw glow_saw_;
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
  Lights(Pwm* pwm) :
    now_(0), prev_update_(0),
    pwm_(pwm),
    mode_(0), brt_(0), lvl_(0),
    state_change_(CtrlState::CHG_NONE) {
    state_.level = 1;
    state_.mode = 0;
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      pwm_val_[i] = 0;
      pwm_->Set(i, pwm_val_[i]);
      pwm_->Enable(i);
    }
  }

  CtrlState* State() { return &state_; }

  u8_t Mode() { return mode_ >> 2; }
  void PushPwm() {
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      if (pwm_val_[i] != pwm_->Get(i)) {
        pwm_->Set(i, pwm_val_[i]);
      }
    }
  }
  void Push() {
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
    DBG_MD(APP, ("Update Lvl: %d Brt: %d\n", lvl_, brt_));
  }

  void ApplyUpdatedState() {
    // bool mode_change = (state_change_ & CHG_MODE) != CHG_NONE;
    DBG_MD(APP, ("State: L:%d M:%02x B:%d T:%d\n",
                 state_.level, state_.mode, state_.brt, state_.thr));
    state_change_ = CtrlState::CHG_NONE;
    mode_ = state_.mode;
    UpdateBright(state_.level, state_.brt);
    pulse_state_.UpdateState(state_.thr);
  }

  void Update(u16_t now) {
    DBG_HI(APP, ("Lights::Update now: %u\n", now));
    pulse_state_.Update(now);
    const u8_t pulse = pulse_state_.GetLvl();
    const u8_t pbrt = bscale8((lvl_ - brt_), pulse) + brt_;
    switch (Mode()) {
    case 0:
      pwm_val_[0] = brt_;
      pwm_val_[1] = lvl_;
      pwm_val_[2] = lvl_;
      pwm_val_[3] = lvl_;
      break;
    case 1: {
      pwm_val_[0] = pulse;
      pwm_val_[1] = lvl_;
      pwm_val_[2] = lvl_;
      pwm_val_[3] = lvl_;
    } break;
    case 2:
      pwm_val_[0] = pbrt;
      pwm_val_[1] = pbrt;
      pwm_val_[2] = pbrt;
      pwm_val_[3] = pbrt;
      break;
    }
    Push();
  }

  // Called roughly every 32ms (32 fps)
  void Run(SBus* sbus) {
    DBG_HI(APP, ("Update\n"));
    now_ = FastTimeMs();
    u8_t now_8 = now_ >> 8;  // 1/4 seconds
    // Update state if forced,  sbus settings change or 1/4 sec has elapsed.
    bool update = now_8 != prev_update_;
    if (update) {
      prev_update_ = now_8;
    }
    if (!update && sbus && SBusUpdate(sbus)) {
      update = true;
    }
    if (update) {
      ApplyUpdatedState();
    }
    Update(now_);
  }

private:
  u16_t now_;
  u8_t prev_update_;
  Pwm* const pwm_;
  u8_t pwm_val_[NUM_PWM];
  u8_t mode_, brt_, lvl_;
  CtrlState state_;
  u8_t state_change_;

  PulseModeState pulse_state_;
};

class LightsCmd : public CmdHandler {
public:
  LightsCmd(Lights* lights)
    : CmdHandler("lights"), lights_(lights) { }

  virtual void HandleLine(const char* args) {
    CtrlState* state = lights_->State();
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
        state->mode = val;
        break;
      case 'l': case 'L':
        state->level = val;
        break;
      case 'b': case 'B':
        state->brt = val;
        break;
      case 't': case 'T':
        state->thr = val;
        break;
      default:
        break;
      }
      args += 2 + len;
    }
    if (iter != 0) {
      lights_->ApplyUpdatedState();
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
  Pwm pwm(PORT_D, 400);
  for (u8_t i = 0; i < 4; ++i) {
    pwm.Enable(i);
    pwm.Set(i, 0);
  }

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  DBG_LEVEL_HI(SBUS);
  SBus sbus(&Serial::usart2, /*invert=*/true);

  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();

  Lights lights(&pwm);

  sei();
  DBG_MD(APP, ("Hello World: Test\n"));

  DbgCmds cmds(&Serial::usart0);
  VARCMDS_INIT(cmds);
  LightsCmd lights_cmd(&lights);
  cmds.RegisterHandler(&lights_cmd);

  const u8_t pat = GetBlinkPattern();

  u8_t update_5 = 0;
  DBG_MD(APP, ("Entering Run Loop\n"));
  u8_t update_8 = 0;
  bool sbus_update = false;
  while (true) {
    u16_t now = FastTimeMs();

    cmds.Run();
    if (sbus.Run()) {
      sbus_update = true;
    }

    const u8_t now_5 = now >> 5;
    if (now_5 == update_5) continue;
    update_5 = now_5;
    lights.Run(sbus_update ? &sbus : NULL);
    sbus_update = false;

    const u8_t now_8 = now >> 8;
    if (now_8 == update_8) continue;
    update_8 = now_8;
    u8_t phase = (now_8 & 0x07);
    blink_pin.set(pat & (1 << phase));
  }
}
