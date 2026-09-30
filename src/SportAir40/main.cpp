// Copyright 2026 Thomas DeWeese
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

#include "Crsf.h"
#include "Pins.h"
#include "Pwm.h"
#include "Serial.h"
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

#define LED_PIN (PIN_C2)


#define LIGHT_BRIGHT_CH (10)  // Adjust leading edge light brightness.
#define LIGHT_LEVEL_CH (11)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH (12) 
#define LIGHT_SUBMODE_CH (13)  
#define LIGHT_THROTTLE_CH (14)  // Adjusts lights speed

#define LED_FUSE_SIDE_CNT (78)
#define LED_FUSE_CNT (2 * LED_FUSE_SIDE_CNT)
#define NUM_WING_PANELS (6)
#define LED_WING_PANEL_V_CNT (8)
#define LED_WING_PANEL_H_CNT (4)
#define LED_WING_PANEL_CNT (LED_WING_PANEL_H_CNT +      \
                            2 * LED_WING_PANEL_V_CNT)
#define LED_WING_TIP_CNT (2 + 12)
#define LED_WING_CNT ((NUM_WING_PANELS * LED_WING_PANEL_CNT) + (3 * LED_WING_TIP_CNT) + 2)

#define RGB_CNT (LED_FUSE_CNT)
#define RGBW_CNT (2 * LED_WING_CNT)
u8_t led_data[3 * RGB_CNT + 4 * RGBW_CNT];

struct State {
  enum Channels {
    SLIDER   = 10,  // Light brightness slider
    SWITCH   = 11,  // 3 pos off/low/high
    MODE     = 12,  // Sets Lights Mode (solid, spin, glow)
    SUBMODE  = 13,  // Sets mode sub mode (best, simple, max)
    THROTTLE = 14,  // Adjust 'energy' (speed etc).
  };
  State(u8_t mode, u8_t level, i16_t brt, i16_t thr) :
    mode(mode), level(level), brt(brt), thr(thr) { }
  State() : mode(0), level(0), brt(0), thr(0) { }
  State(RcChannels& chan) :
    mode((ThreePosSwitch(chan.GetChannel(MODE)) << 2) |
	 ThreePosSwitch(chan.GetChannel(SUBMODE))),
    level(ThreePosSwitch(chan.GetChannel(SWITCH))),
    brt(chan.GetChannel(SLIDER)),
    thr(chan.GetChannel(THROTTLE)) { }

  void Update(RcChannels& chan) {
    *this = State(chan);
  }

  void Dump() {
    DBG_MD(APP, ("State: L:%d M:%01x B:%d T:%d\n",
		 level, mode, brt, thr));
  }

  u8_t mode;
  u8_t level;
  i16_t brt;   // brt & thr are 0 -> 2047 nominal (11 bits)
  i16_t thr;
};

struct WingPanel {
  WingPanel() { }
  void* Init(void* ptr) {
    ptr = i_strip.SetSpan(ptr, LED_WING_PANEL_V_CNT, true);
    ptr = h_strip.SetSpan(ptr, LED_WING_PANEL_H_CNT, false);
    ptr = o_strip.SetSpan(ptr, LED_WING_PANEL_V_CNT, true);
    return ptr;
  }

  void Set(const RGBW& clr) {
    i_strip.Fill(clr);
    h_strip.Fill(clr);
    o_strip.Fill(clr);
  }

  // Set wing panel horiz index (0->5)
  void SetH(int i, const RGBW& clr) {
    switch(i) {
    case 0:
      i_strip.Fill(clr);
      break;
    case 1: case 2: case 3: case 4:
      h_strip.Set(i - 1, clr);
      break;
    case 5: default:
      o_strip.Fill(clr);
      break;
    }
  }
  // Set wing panel vertical index (0->8)
  void SetV(int i, const RGBW& clr) {
    switch(i) {
    case 0:
      h_strip.Fill(clr);
      break;
    default:
      i_strip.Set(i - 1, clr);
      o_strip.Set(i - 1, clr);
      break;
    }
  }

  LedSpan<RGBW> i_strip, h_strip, o_strip;
  void* end;
};

struct WingTip {
  WingTip() {}
  void*Init(void* ptr, bool left) {
    ptr = to_strip.SetSpan(ptr, LED_WING_TIP_CNT, true);
    ptr = ti_strip.SetSpan(ptr, LED_WING_TIP_CNT, false);
	// Left bottom wing tip is miswired so two leds are the same.
    ptr = bi_strip.SetSpan(ptr, LED_WING_TIP_CNT + (left ? 1 : 2), true);
    return ptr;
  }

  void Set(const RGBW& clr) {
    to_strip.Fill(clr);
    ti_strip.Fill(clr);
    bi_strip.Fill(clr);
  }

  void Set(int i, const RGBW& clr) {
    switch(i) {
    case 0:
      to_strip.Fill(clr);
      break;
    case 1:
      ti_strip.Fill(clr);
      break;
    case 2:
      bi_strip.Fill(clr);
      break;
    }
  }
  
  LedSpan<RGBW> to_strip, ti_strip, bi_strip;
};

class Lights {
public:
  Lights() : state_(1 << 2 | 0, 1, 256, 0),
	     pwm_(PORT_D, 400), color_saw(0x20) {
    pwm_.Enable(1, 0x00);
    pwm_.Enable(2, 0x00);
    void* ptr = led_data;
    ptr = r_fuse.SetSpan(ptr, LED_FUSE_SIDE_CNT, true);
    ptr = l_fuse.SetSpan(ptr, LED_FUSE_SIDE_CNT, false);

    for (int p = 0; p < NUM_WING_PANELS; ++p) {
      ptr = l_panels[p].Init(ptr);
    }
    ptr = l_tip.Init(ptr, /*left=*/true);

    for (int p = 0; p < NUM_WING_PANELS; ++p) {
      ptr = r_panels[p].Init(ptr);
    }
    ptr = r_tip.Init(ptr, /*left=*/false);
    UpdateBright(state_.level, state_.brt);
  }

  const State& state() const { return state_; }
  u8_t Mode() { return mode_ >> 2; }
  u8_t Submode() { return mode_ & 0x03; }

  void UpdateRc(RcChannels& chan) {
    state_.Update(chan);
  }
  void UpdateState(const State& state) {
    state_ = state;
  }

  void ApplyState() {
    mode_ = state_.mode;
    UpdateBright(state_.level, state_.brt);
    // spinner_.UpdateState(state_.thr, 255);
    // clr_spin_.Set(Submode(), state_.brt, state_.thr);
    DBG_HI(APP, ("Update Brt: %d Spn: %d\n", brt_, wtip_brt_));
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
    l_clr_ = led::wclr::red;
    Fade(&l_clr_, wtip_brt_);
    r_clr_ = led::wclr::green;
    Fade(&r_clr_, wtip_brt_);
  }

  void Update(u16_t now) {
    DBG_HI(APP, ("Lights::Update now: %u\n", now));
    ApplyState();
    switch (Mode()) {
    case 0:
      UpdateSolidMode(now);
      break;
    case 1:
      UpdateColorMode(now);
      break;
    case 2:
      // UpdatePulseMode();
      break;
    }
    Push();
  }

  void UpdateSolidMode(u16_t now) {
    u8_t smode = Submode();
    l_tip.Set(l_clr_);
    r_tip.Set(r_clr_);
    land_brt_ = brt_;
    RGBW wwht(0, brt_);
    RGB wht(wwht);
    switch (smode) {
    case 0:
      l_fuse.Fill(l_clr_);
      r_fuse.Fill(r_clr_);
      for (int p = 0; p < NUM_WING_PANELS; ++p) {
	WingPanel& l_panel = l_panels[p];
	WingPanel& r_panel = r_panels[p];
	l_panel.Set(l_clr_);
	r_panel.Set(r_clr_);
      }
      break;
    case 1:
    case 2:
      l_fuse.Fill(wht);
      r_fuse.Fill(wht);
      for (int p = 0; p < NUM_WING_PANELS; ++p) {
	WingPanel& l_panel = l_panels[p];
	WingPanel& r_panel = r_panels[p];
	l_panel.Set(wwht);
	r_panel.Set(wwht);
      }
      break;
    }
  }
  void UpdateColorMode(u16_t now) {
    l_tip.Set(l_clr_);
    r_tip.Set(r_clr_);
    land_brt_ = brt_;

    const u8_t hue_step = (0xFF0 / 36);
    i16_t color_phase = (0xFF - color_saw.Get(now)) << 4;

    RGBW rgbw = HsvToRgb(HSV(
      RND_SHIFT(color_phase, 4), 0xFF, wtip_brt_));
    l_fuse.Fill(rgbw);
    r_fuse.Fill(rgbw);

    for (int p = 0; p < NUM_WING_PANELS; ++p) {
      WingPanel& l_panel = l_panels[p];
      WingPanel& r_panel = r_panels[p];
      for (int l = 0; l < 6; ++l) {
        u8_t hue = RND_SHIFT(color_phase, 4);
	RGBW rgbw = HsvToRgbw(HSV(hue, 0xFF, wtip_brt_));
        l_panel.SetH(l, rgbw );
        r_panel.SetH(l, rgbw);
        color_phase += hue_step;
      }
    }
  }

  void Push() {
    pwm_.Set(1, land_brt_);
    pwm_.Set(2, land_brt_);

    // 20ms with 156 RGB and 290 RGBW leds.
    SendWS2812(LED_PIN, led_data, sizeof(led_data), 0xFF);
  }
  
  State state_;
  u8_t mode_, brt_, wtip_brt_, land_brt_;
  Pwm pwm_;
  VariableSaw color_saw;
  LedSpan<RGB> l_fuse, r_fuse;
  RGBW l_clr_, r_clr_;
  WingPanel l_panels[6];
  WingTip l_tip;
  WingPanel r_panels[6];
  WingTip r_tip;
};

class LightCmd : public CmdHandler {
public:
  LightCmd(Lights* lights)
    : CmdHandler("l"), lights_(lights) { }

  virtual void HandleLine(const char* args) {
    DBG_MD(APP, ("Lights HandleLine: %s\n", args));
    State state = lights_->state();
    int iter = 0;
    while (*args) {
      while (isspace(*args)) ++args;
      if (!args[0] || !args[1]) break;
      int len = 0;
      int val;
      int cnt = sscanf(args + 2, "%d%n", &val, &len);
      DBG_MD(APP, ("Lights: %c v: %d cnt: %d\n", *args, val, cnt));
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
    DBG_MD(APP, ("Lights iter: %d\n", iter));
    if (iter != 0) {
      state.Dump();
      lights_->UpdateState(state);
      lights_->ApplyState();
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
  DBG_LEVEL_LO(SBUS);

  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();
  PinId led_pin(LED_PIN);
  led_pin.SetOutput();

  Crsf crsf(&Serial::usart1);
  CrsfRcHandler rc_handler;
  crsf.AddHandler(&rc_handler);

  sei();
  DBG_MD(APP, ("OSMW Sport Air 40: Startup!!!\n"));

  memset(led_data, 0, sizeof(led_data));
  SendWS2812(LED_PIN, led_data, sizeof(led_data), 0xFF);
  Lights lights;

  DbgCmds cmds(&Serial::usart0);
  VARCMDS_INIT(cmds);
  LightCmd lights_cmd(&lights);
  cmds.RegisterHandler(&lights_cmd);

  DBG_MD(APP, ("OSMW Sport Air 40: Running!!!\n"));

  u8_t update_3 = 0;
  u8_t update_5 = 0;
  u8_t update_8 = 0;
  while (1) {
    const u16_t now = FastTimeMs();
//     if (crsf.Run()) {
//       if (crsf.RunHandlers()) {
//       }
//     }
    const u8_t now_3 = now >> 3;
    if (now_3 == update_3) continue;
    update_3 = now_3;  // every 8ms
    cmds.Run();

    const u8_t now_5 = now >> 5;
    if (now_5 == update_5) continue;
    update_5 = now_5;  // every 32ms
    lights.Update(now);
    
    const u8_t now_8 = now >> 8;  // 1/4 sec
    if (now_8 == update_8) continue;
    update_8 = now_8;
    // rc_handler.Dump();
    blink_pin.toggle();
  }
}

