// Copyright 2024 Thomas DeWeese
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
#include "Dbg.h"
#include "DbgCmds.h"
#include "IntTypes.h"
#include "RtcTime.h"
#include "Util.h"
#include "VarCmds.h"

#include "leds/Clr.h"
#include "leds/FPMath.h"
#include "leds/LedSpan.h"
#include "leds/LedSpanOps.h"
#include "leds/Pixel.h"
#include "leds/Rgb.h"

#include "Button.h"
#include "Pins.h"
#include "Rand.h"
#include "Serial.h"
#include "WS2812.h"

#include "led_map.h"

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

#define OCHIN_RGBW_CNT (7)
#define OCHEEK_RGBW_CNT (15)
#define OCHEEK_RGB_CNT (6)
#define LHORN_RGBW_CNT (6)
#define LEYE_RGBW_CNT (3)
#define REYE_RGBW_CNT (3)
#define MOUTH_RGBW_CNT (11)
#define BCHEEK_RGBW_CNT (37)
#define RHORN_RGBW_CNT (5)

#define FORE_RGBW_CNT (35)
#define FORE_RGB_CNT (27)
#define RCHEEK_RGBW_CNT (1)

#define RGBW_CNT (OCHIN_RGBW_CNT + OCHEEK_RGBW_CNT + LHORN_RGBW_CNT + LEYE_RGBW_CNT + REYE_RGBW_CNT + \
                  MOUTH_RGBW_CNT + BCHEEK_RGBW_CNT + RHORN_RGBW_CNT + FORE_RGBW_CNT + RCHEEK_RGBW_CNT)
#define RGB_CNT (OCHEEK_RGB_CNT + +FORE_RGB_CNT)

u8_t led_data[RGBW_CNT * 4 + RGB_CNT * 3];

static const RGB fire_grad_rgb[2][5]  = {/* Blue */
                                         { RGB(0x00, 0x00, 0x00),
                                           RGB(0x00, 0x00, 0x40),
                                           RGB(0x00, 0x00, 0x80),
                                           RGB(0x00, 0x44, 0xFF),
                                           RGB(0x40, 0x80, 0xFF) },
                                         /* Orange */
                                         { RGB(0x00, 0x00, 0x00),
                                           RGB(0x40, 0x11, 0x00),
                                           RGB(0x80, 0x22, 0x00),
                                           RGB(0xFF, 0x44, 0x00),
                                           RGB(0xFF, 0xC4, 0x80) }};

static const RGBW fire_grad_rgbw[2][5] = {/* Blue */
                                          { RGBW(0x00, 0x00, 0x00),
                                            RGBW(0x00, 0x00, 0x40),
                                            RGBW(0x00, 0x00, 0x80),
                                            RGBW(0x00, 0x44, 0xFF),
                                            RGBW(0x00, 0x44, 0xFF, 0x40)
                                          }, /* Orange */
                                          { RGBW(0x00),
                                            RGBW(0x40, 0x11, 0x00),
                                            RGBW(0x80, 0x22, 0x00),
                                            RGBW(0xFF, 0x44, 0x00),
                                            RGBW(0xFF, 0x44, 0x00, 0x80) }};

inline u8_t subClip(u8_t a, u8_t b) {
  u8_t r = a - b;
  return (r > a) ? 0 : r;
}

inline u8_t addClip(u8_t a, u8_t b) {
  u8_t r = a + b;
  return (r < a) ? 0xFF : r;
}

inline u8_t max(u8_t a, u8_t b) {
  return a > b ? a : b;
}
inline u8_t min(u8_t a, u8_t b) {
  return a < b ? a : b;
}

inline u16_t min16(u16_t a, u16_t b) {
  return a < b ? a : b;
}
inline u16_t max16(u16_t a, u16_t b) {
  return a > b ? a : b;
}


void UpdateHeatMap(u8_t cooling, u8_t sparking, int len, u8_t* heat) {
  // Step 1.  Cool down every cell a little
  const u8_t kCooling = ((cooling * 5) / len) + 2;
  for(int i = 0; i < len; i++) {
    heat[i] = subClip(heat[i],  random8(kCooling));
  }

  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for(int k = len - 1; k >= 2; --k) {
    heat[k] = (2 * heat[k] + heat[k - 1] + heat[k - 2] + 3) >> 2;
  }

  // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  while ( random8(255) < sparking ) {
    int y = random8(3);
    heat[y] = addClip(heat[y], random8(255 - 160) + 160);
    sparking = sparking >> 1;
  }
  if (heat[0] < 75) heat[0] = 75;
  if (heat[1] < 50) heat[1] = 50;
}

template <class led_t> void
MapHeatToLed(u8_t len, u8_t* heat, LedSpan<led_t>& leds, const led_t gradient[5]) {
  if (len == leds.len()) {
    for (int i = 0; i < leds.len(); ++i) {
      led_t& v = leds.At(i);
      v = Lookup5(gradient, heat[i]);
    }
  } else {
    u16_t heat_step = ((len - 1) << 8) / (leds.len() - 1);
    u16_t heat_pos = 0;
    for (int i = 0; i < leds.len(); ++i) {
      const u8_t heat_idx = heat_pos >> 8;
      const u8_t heat_frac = heat_pos & 0xFF;
      const u8_t t0 = heat[heat_idx];
      const u8_t t1 = heat[heat_idx + 1];
      const u8_t t = t0 + ((((i16_t)t1 - t0) * heat_frac) >> 8);
      led_t& v = leds.At(i);
      v = Lookup5(gradient, t);
      heat_pos += heat_step;
    }
  }
}

const RGBW tag_clr_rgbw[TAG_COUNT] =
  {
   RGBW(0xFF, 0x44, 0x00), // TAG_OCHIN
    RGBW(0xFF, 0x44, 0x00), // TAG_OCHEEK
    RGBW(0xFF), // TAG_HORN
    RGBW(0xFF), // TAG_EYE
    RGBW(0xFF), // TAG_MOUTH
    RGBW(0xFF, 0x44, 0x00), // TAG_FORE
    RGBW(0, 0, 0xFF), // TAG_BCHEEK
  };

const RGB tag_clr_rgb[TAG_COUNT] =
  {
    RGB(0xFF, 0x44, 0x00), // TAG_OCHIN
    RGB(0xFF, 0x44, 0x00), // TAG_OCHEEK
    RGB(0xFF, 0xAA, 0x77), // TAG_HORN
    RGB(0xFF, 0xAA, 0x77), // TAG_EYE
    RGB(0xFF, 0xAA, 0x77), // TAG_MOUTH
    RGB(0xFF, 0x44, 0x00), // TAG_FORE
    RGB(0, 0, 0xFF), // TAG_BCHEEK
 };

class State {
public:
  enum Modes {
              MODE_FLAT = 0,
              MODE_FLAT_EYE,
              MODE_PULSE,
              MODE_PULSE_EYE,
              MODE_WAVE,
              MODE_FIRE,
              MODE_COUNT
  };

  State(u8_t mode, u8_t bright, u8_t speed)
    : mode_(mode), bright_(bright), speed_(speed) { }

  u8_t GetMode() const { return mode_; }
  void SetMode(u8_t mode) {
    mode_ = mode;
    if (mode_ >= MODE_COUNT) mode_ = MODE_FLAT;
  }
  void IncMode() { SetMode(++mode_); }

  u8_t GetBright() const { return bright_; }
  void SetBright(u8_t bright) { bright_ = bright; }
  void IncBright() {
    if (bright_ < 16)       { bright_ =  16; }
    else if (bright_ < 64)  { bright_ =  64; }
    else if (bright_ < 255) { bright_ = 255; }
    else {                    bright_ =   0; }
  }

  u8_t GetSpeed() { return speed_; }
  void SetSpeed(u8_t speed) {
    speed_ = speed;
    if (speed_ > 64) speed_ = 64;
  }
  void IncSpeed() {
    SetSpeed(speed_ == 0 ? 4 : speed_ << 1);
  }

protected:
  u8_t mode_;
  u8_t bright_;
  u8_t speed_;
};

class StateCmd : public dbg::CmdHandler {
public:
  StateCmd(State* state)
    : CmdHandler("state"), state_(state) { }

  virtual void HandleLine(const char* args) {
    while (*args) {
      while (isspace(*args)) ++args;
      if (!args[0] || !args[1]) break;
      int len = 0;
      int val;
      int cnt = sscanf(args + 2, "%d%n", &val, &len);
      if (cnt != 1) break;
      switch (*args) {
      case 'm': case 'M':
        state_->SetMode(val);
        break;
      case 'b': case 'B':
        state_->SetBright(val);
        break;
      case 's': case 'S':
        state_->SetSpeed(val);
        break;
      default:
        break;
      }
      args += 2 + len;
    }
  }

private:
  State* state_;
};


class Lights {
public:
  Lights(PinId led_pin) :
    led_pin_(led_pin),
    state_(/*mode_=*/State::MODE_FLAT, /*bright_=*/32, /*speed_=*/12),
    skip_cnt_(0), red_eye_(false), eye_pulse_start_ms_(0) {
    void* ptr = led_data;

    ptr = ochin_rgbw.SetSpan(ptr, OCHIN_RGBW_CNT, /*reverse=*/false);
    ptr = ocheek_rgbw.SetSpan(ptr, OCHEEK_RGBW_CNT, /*reverse=*/false);
    ptr = ocheek_rgb.SetSpan(ptr, OCHEEK_RGB_CNT, /*reverse=*/false);
    ptr = lhorn_rgbw.SetSpan(ptr, LHORN_RGBW_CNT, /*reverse=*/false);
    ptr = leye_rgbw.SetSpan(ptr, LEYE_RGBW_CNT, /*reverse=*/false);
    ptr = reye_rgbw.SetSpan(ptr, REYE_RGBW_CNT, /*reverse=*/false);
    ptr = mouth_rgbw.SetSpan(ptr, MOUTH_RGBW_CNT, /*reverse=*/false);
    ptr = bcheek_rgbw.SetSpan(ptr, BCHEEK_RGBW_CNT, /*reverse=*/false);
    ptr = rhorn_rgbw.SetSpan(ptr, RHORN_RGBW_CNT, /*reverse=*/false);
    ptr = fore_rgbw.SetSpan(ptr, FORE_RGBW_CNT, /*reverse=*/false);
    ptr = fore_rgb.SetSpan(ptr, FORE_RGB_CNT, /*reverse=*/false);
    ptr = rcheek_rgbw.SetSpan(ptr, RCHEEK_RGBW_CNT, /*reverse=*/false);
    pulse_saw_.SetSpeed(state_.GetSpeed());
  }

  void Update(u16_t now_ms) {
    Update(now_ms, state_.GetMode(), state_.GetSpeed());
  }

  State& GetState() { return state_; }

  void PushLeds(u8_t bright) {
    SendWS2812(led_pin_, led_data, sizeof(led_data), bright);
  }

private:
  void Update(u16_t now_ms, u8_t mode, u8_t speed) {
    pulse_saw_.SetSpeed(speed);
    u8_t bright = state_.GetBright();
    switch (mode) {
    case State::MODE_PULSE:
    case State::MODE_PULSE_EYE:
    case State::MODE_FLAT:
    case State::MODE_FLAT_EYE:
      UpdatePulse(now_ms, mode, speed, bright);
      bright = 0xFF;
      break;
    case State::MODE_WAVE:
      eye_pulse_start_ms_ = 0;
      UpdateWave(now_ms);
      break;
    case State::MODE_FIRE:
      eye_pulse_start_ms_ = 0;
      UpdateFire(speed);
      break;
    }
    PushLeds(bright);
  }

  void UpdateFire(u8_t speed) {
    const u16_t fire = 4 * speed; // 0-255
    const int cooling = 80 - ((50 * fire) >> 8);  // 85 - 35
    const int sparking = 50 + ((150 * fire) >> 8); // 50 - 200
    u16_t sum = 0;
    for (int i = 0; i < 4; ++i) {
      UpdateHeatMap(cooling, sparking, ARRAY_SIZE(heat[i]), heat[i]);
      for (u8_t j = 0; j < ARRAY_SIZE(heat[i]); ++j) {
        sum += heat[i][j];
      }
    }
    const u8_t avg_t = sum / (4 * ARRAY_SIZE(heat[0]));

    u8_t* ptr = led_data;
    for (unsigned i = 0; i < ARRAY_SIZE(ranges); ++i) {
      const led_range& item = ranges[i];
      // X range 18 -> 208 - delta is 190 so ~64 between groups
      // Y range 13 -> 236 -> 223,  64 *(16 / 170) = ~6
      u8_t col = (item.xloc >= 100) ? ((item.xloc >= 140) ? 3 : 2) :
        ((item.xloc >= 60) ? 1 : 0);

      i16_t y_val = ((236 - item.yloc) * 9); // flip y-axis.
      u8_t y_idx = y_val >> 7; // 0 -> 15
      u8_t y_frac = y_val & ((1 << 7) - 1);
      const u8_t t0 = heat[col][y_idx];
      const u8_t t1 = heat[col][y_idx + 1];
      const u8_t t = t0 + ((((i16_t)t1 - t0) * y_frac) >> 7);

      if (item.is_rgbw) {
        RGBW* pix = reinterpret_cast<RGBW*>(ptr);
        switch (item.tag) {
        case TAG_HORN:
        case TAG_EYE:
        case TAG_MOUTH:
          *pix = RGBW(avg_t);
          break;
        case TAG_OCHIN:
        case TAG_OCHEEK:
        case TAG_FORE:
          *pix = Lookup5(fire_grad_rgbw[1], t);
          break;
        case TAG_BCHEEK:
          *pix = Lookup5(fire_grad_rgbw[0], t);
          break;
        case TAG_COUNT: break;
        }
        ptr += 4;
      } else {
        RGB* pix = reinterpret_cast<RGB*>(ptr);
        switch (item.tag) {
        case TAG_HORN:
        case TAG_EYE:
        case TAG_MOUTH:
          // Not actually used.
          break;
        case TAG_OCHIN:
        case TAG_OCHEEK:
        case TAG_FORE:
          *pix = Lookup5(fire_grad_rgb[1], t);
          break;
        case TAG_BCHEEK:
          // Not actually used..
          break;
        case TAG_COUNT: break;
        }
        ptr += 3;
      }
    }
  }

  RGBW GetEyeColor(u16_t now_ms, u8_t mode, u8_t speed, u8_t pulse_val) {
    if (mode != State::MODE_PULSE_EYE &&
        mode != State::MODE_FLAT_EYE ) {
      eye_pulse_start_ms_ = 0;
      return tag_clr_rgbw[TAG_EYE];
    }
    if (eye_pulse_start_ms_ == 0) {  // First entry...
      eye_pulse_start_ms_ = now_ms;
      eye_saw_.SetState(speed, /*offset=*/0);
      red_eye_ = true;
    }
    u16_t pulse_delta = now_ms - eye_pulse_start_ms_;
    if (!red_eye_) {
      if (mode == State::MODE_FLAT_EYE && pulse_delta < 100) {
        eye_saw_.SetState(24, /*offset=*/0);
        red_eye_ = true;
      }
      else if (mode == State::MODE_PULSE_EYE && pulse_val < 6) {
        DBG_MD(APP, ("Pulse Eye start: %d pulse_val: %d\n", now_ms, pulse_val));
        eye_pulse_start_ms_ = min16(u16_t(65505U), max16(1, now_ms));
        pulse_delta = 0;
        eye_saw_.SetState(32, /*offset=*/0);
        red_eye_ = true;
      }
    }
    if (red_eye_) {
      u8_t frac = eye_saw_.Get(pulse_delta) + 32;
      if (frac < 128) {
        RGBW eye(0xFF, 0, 0, 0);
        u8_t val = 255 - ((sin8(frac) - 128) << 1);
        Blend(&eye, tag_clr_rgbw[TAG_EYE], val);
        return eye;
      }

      // Random value is up to 16s w/ 2bits frac accuracy so we shift by 8
      // to get to 'millisecond' accuracy.
      if (mode == State::MODE_FLAT_EYE) {
        eye_pulse_start_ms_ = u16_t(now_ms + 2048 + (u16_t(random8(64)) << 8));
        eye_pulse_start_ms_ = min16(u16_t(65505U),
                                    max16(1, eye_pulse_start_ms_));
        DBG_MD(APP, ("Flat Eye Wait: %d\n", eye_pulse_start_ms_ - now_ms));
      }
      red_eye_ = false;
    }

    return tag_clr_rgbw[TAG_EYE];
  }

  void UpdatePulse(u16_t now_ms, u8_t mode, u8_t speed, u8_t bright) {
    const u8_t pulse_val = sin8(pulse_saw_.Get(now_ms));
    if (mode == State::MODE_PULSE || mode == State::MODE_PULSE_EYE) {
      bright = bscale8(pulse_val, bright);
    }

    RGB wht_rgb(0xFF, 0xAA, 0x88); Fade(&wht_rgb, bright);
    RGBW wht_rgbw(0xFF); Fade(&wht_rgbw, bright);

    RGB org_rgb(0xFF, 0x44, 0); Fade(&org_rgb, bright);
    RGBW org_rgbw(0xFF, 0x44, 0); Fade(&org_rgbw, bright);

    RGB blu_rgb(0, 0, 0xFF); Fade(&blu_rgb, bright);
    RGBW blu_rgbw(0, 0, 0xFF); Fade(&blu_rgbw, bright);

    RGBW eye = GetEyeColor(now_ms, mode, speed, pulse_val);
    Fade(&eye, state_.GetBright());  // eye's aren't faded by pulse.

    ochin_rgbw.Fill(org_rgbw);
    ocheek_rgbw.Fill(org_rgbw);
    ocheek_rgb.Fill(org_rgb);
    lhorn_rgbw.Fill(wht_rgbw);
    leye_rgbw.Fill(eye);
    reye_rgbw.Fill(eye);
    mouth_rgbw.Fill(wht_rgbw);
    bcheek_rgbw.Fill(blu_rgbw);
    rhorn_rgbw.Fill(wht_rgbw);
    fore_rgbw.Fill(org_rgbw);
    fore_rgb.Fill(org_rgb);
    rcheek_rgbw.Fill(org_rgbw);
  }

  void UpdateWave(u16_t now_ms) {
    u8_t phase = ~pulse_saw_.Get(now_ms);
    void* ptr = led_data;
    const RGBW* clr_rgbw = tag_clr_rgbw;
    const RGB* clr_rgb = tag_clr_rgb;
    for (u8_t i = 0; i < ARRAY_SIZE(ranges); ++i) {
      const led_range& item = ranges[i];
      const u8_t item_phase = phase + (item.dist << 1);
      const u8_t glow = max(4, sin8(item_phase));
      if (item.is_rgbw) {
        LedSpan<RGBW> span(ptr, 1, false);
        RGBW clr = clr_rgbw[item.tag];
        Fade(&clr, glow);
        span.Fill(clr);
        ptr = span.next_ptr();
      } else {
        LedSpan<RGB> span(ptr, 1, false);
        RGB clr = clr_rgb[item.tag];
        Fade(&clr, glow);
        span.Fill(clr);
        ptr = span.next_ptr();
      }
    }
  }

protected:
  const PinId led_pin_;
  State state_;

  u8_t skip_cnt_;
  bool red_eye_;
  u16_t eye_pulse_start_ms_;
  LedSpan<RGBW> ochin_rgbw;
  LedSpan<RGBW> ocheek_rgbw;
  LedSpan<RGB> ocheek_rgb;
  LedSpan<RGBW> lhorn_rgbw;
  LedSpan<RGBW> leye_rgbw;
  LedSpan<RGBW> reye_rgbw;
  LedSpan<RGBW> mouth_rgbw;
  LedSpan<RGBW> bcheek_rgbw;
  LedSpan<RGBW> rhorn_rgbw;
  LedSpan<RGBW> fore_rgbw;
  LedSpan<RGB> fore_rgb;
  LedSpan<RGBW> rcheek_rgbw;

  VariableSaw pulse_saw_;
  VariableSaw eye_saw_;
  u8_t heat[4][17];
};

#if 0
void CalcPrintDists(int center_x, int center_y) {
  // int center_x = 106;
  // int center_y = 101;
  for (unsigned i = 0; i < ARRAY_SIZE(ranges); ++i) {
    const led_range& range = ranges[i];
    long dx = range.xloc - center_x;
    long dy = range.yloc - center_y;
    long dst_sq = dx * dx + dy * dy;
    int dist = (sqrt(dst_sq) + 0.5);
    DBG_MD(APP, ("Dist[%d]: %d\n", i, dist));
  }
}
#endif


int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  Button btn_brt(PIN_A5);
  Button btn_mode(PIN_A6);

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  DBG_LEVEL_MD(SBUS);
  dbg::DbgCmds cmds(&Serial::usart0);
  VARCMDS_INIT(cmds);

  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();
  blink_pin.set(1);

  PinId rgb_led_pin(RGB_LED_PIN);
  rgb_led_pin.SetOutput();

  sei();
  DBG_MD(APP, ("BisonPlaque: Run\n"));

  memset(led_data, 0, sizeof(led_data));
  SendWS2812(rgb_led_pin, led_data, sizeof(led_data), 0xFF);
  Lights lights(rgb_led_pin);

  StateCmd state_cmd(&lights.GetState());
  cmds.RegisterHandler(&state_cmd);

  // u8_t blink_phase = 0;

  u8_t update_0 = 0;
  u8_t update_3 = 0;
  u8_t update_5 = 0;
  u8_t update_8 = 0;
  u8_t update_1s = 0xFF;
  u8_t update_16s = 0xFF;

  while (1) {
    u16_t now = FastTimeMs();

    if (update_0 == now) continue;  // 1ms
    update_0 = now;
    cmds.Run();

    const u8_t now_3 = now >> 3;   // 1/128 sec
    if (now_3 == update_3) continue;
    update_3 = now_3;
    State& state = lights.GetState();
    if (btn_mode.Check() && btn_mode.Down()) {
      state.IncMode();
    }
    if (btn_brt.Check() && btn_brt.Down()) {
      state.IncBright();
    }

    const u8_t now_5 = now >> 5;   // 1/32 sec
    if (now_5 == update_5) continue;
    update_5 = now_5;
    lights.Update(now);

    const u8_t now_8 = now >> 8;
    if (now_8 == update_8) continue;  // 1/4 sec
    update_8 = now_8;
    // u8_t pat = 0x33;
    // blink_pin.set(!(pat & (1 << blink_phase)));  // pin high is off.
    // blink_phase = (blink_phase + 1) & 0x07;

    u16_t now_s = FastSecs();
    if (now_s == update_1s) continue;
    update_1s = now_s;

    u8_t now_16s = now_s >> 4;
    if (now_16s == update_16s) continue;  // 16 sec
    update_16s = now_16s;
  }
}
