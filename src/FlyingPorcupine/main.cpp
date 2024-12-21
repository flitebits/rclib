// Copyright 2024 Thomas DeWeese
//state
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
#include <ctype.h>
#include <math.h>
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

#define swap_u8(a, b) do { \
    u8_t h = (a); \
    (a) = (b); \
    (b) = h; \
} while(false)

#define ENGINE_RGBW_CNT (12)
#define ENGINE_RGB_CNT (18)
#define NOZEL_RGB_CNT (4)
#define FLAME_RGBW_CNT (6)
#define FLAME_RGB_CNT (6)
#define BODY0_RGBW_CNT (11)
#define BODY1_RGBW_CNT (13)
#define BODY2_RGBW_CNT (13)
#define BODY3_RGBW_CNT (11)
#define BODY4_RGBW_CNT (11)
#define BODY_RGBW_CNT (11 + 13 + 13 + 11 + 11)
#define EYE_RGB_CNT (1)

#define RGBW_CNT (ENGINE_RGBW_CNT + FLAME_RGBW_CNT + BODY_RGBW_CNT)
#define RGB_CNT (ENGINE_RGB_CNT + NOZEL_RGB_CNT + FLAME_RGB_CNT + EYE_RGB_CNT)

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

static const RGB kRgbWht(0xFF, 0xCC, 0x88);

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
    RGBW(0xFF), // TAG_ENGINE
    RGBW(0x00, 0x44, 0xFF), // TAG_NOZEL
    RGBW(0xFF, 0, 0, 0xFF), // TAG_FLAME
    RGBW(0xFF), // TAG_BODY
    RGBW(0xFF, 0, 0, 0), // TAG_EYE
  };

const RGB tag_clr_rgb[TAG_COUNT] =
  {
   kRgbWht,  // TAG_ENGINE
   RGB(0x44, 0xFF, 0x00), // TAG_NOZEL GRB
   RGB(0xFF, 0x88, 0x33), // TAG_FLAME
   kRgbWht, // TAG_BODY
   RGB(0xFF, 0, 0), // TAG_EYE
 };

class State {
public:
  enum Modes {
              MODE_FLAT = 0,
              MODE_FLYING,
              MODE_WAVE,
              MODE_FIRE,
              MODE_PULSE,
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


  void Print() {
    DBG_MD(APP, ("State M: %d B: %d S: %d\n", mode_, bright_, speed_));
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
    state_(/*mode_=*/State::MODE_FLAT, /*bright_=*/32, /*speed_=*/32),
    skip_cnt_(0), red_eye_(false), eye_pulse_start_ms_(0) {
    void* ptr = led_data;
    memset(zips_, 0, sizeof(zips_));
    ptr = engine_rgbw_.SetSpan(ptr, ENGINE_RGBW_CNT, /*reverse=*/false);
    ptr = engine_rgb_.SetSpan(ptr, ENGINE_RGB_CNT, /*reverse=*/false);
    ptr = nozel_rgb_.SetSpan(ptr, NOZEL_RGB_CNT, /*reverse=*/false);
    ptr = fire_rgbw_.SetSpan(ptr, FLAME_RGBW_CNT, /*reverse=*/true);
    ptr = fire_rgb_.SetSpan(ptr, FLAME_RGB_CNT, /*reverse=*/false);
    ptr = body_rgbw_.SetSpan(ptr, BODY_RGBW_CNT, /*reverse=*/false);
    ptr = eye_rgb_.SetSpan(ptr, EYE_RGB_CNT, /*reverse=*/false);
  }

  void Update(u16_t now_ms) {
    Update(now_ms, state_.GetMode(), state_.GetSpeed());
  }

  State& GetState() { return state_; }

  void PushLeds(u8_t bright) {
    // RGB rgb(0xFF, 0, 0);
    // SendWS2812(led_pin_, &rgb, sizeof(rgb), 0xff);
    SendWS2812(led_pin_, led_data, sizeof(led_data), bright);
  }

private:
  void Update(u16_t now_ms, u8_t mode, u8_t speed) {
    pulse_saw_.SetSpeed(speed);
    zip_saw_.SetSpeed(2 * speed);
    u8_t bright = state_.GetBright();
    switch (mode) {
    case State::MODE_FLAT:
      UpdateFlat(now_ms, mode, speed, bright);
      break;
    case State::MODE_FLYING:
      if (prev_mode_ != State::MODE_FLYING) {
        InitFlying(now_ms);
      }
      UpdateFlying(now_ms, speed);
      break;
    case State::MODE_PULSE:
      UpdatePulse(now_ms, mode, speed, bright);
      bright = 0xFF;
      break;
    case State::MODE_WAVE:
      eye_pulse_start_ms_ = 0;
      UpdateWave(now_ms, speed);
      break;
    case State::MODE_FIRE:
      eye_pulse_start_ms_ = 0;
      UpdateFire(now_ms, speed);
      break;
    }
    PushLeds(bright);
    prev_mode_ = mode;
  }

  void InitFlying(u16_t now_ms) {
    for (unsigned i =0; i < ARRAY_SIZE(zips_); ++i) {
      zips_[i].start_time = u16_t(now_ms + (u16_t(random8(128)) << 3));
    }
  }

  void UpdateFlame(u8_t speed) {
    const u16_t fire = 3 * speed; // 0-255
    const int cooling = 80 - ((50 * fire) >> 8);  // 85 - 35
    const int sparking = 50 + ((150 * fire) >> 8); // 50 - 200
    UpdateHeatMap(cooling, sparking, ARRAY_SIZE(flame_heat), flame_heat);
    // const u8_t min_flame_x = 11;
    const u8_t max_flame_x = 93;
    u8_t* ptr = reinterpret_cast<u8_t*>(fire_rgbw_.ptr());
    for (u8_t i = 0; i < TAG_FLAME_LEN; ++i) {
      const LedRange& item = ranges[TAG_FLAME_START + i];
      u8_t xloc = (max_flame_x - item.xloc);  // 4.2 frac bits
      u8_t x_idx = xloc >> 2;
      u8_t x_frac = xloc & 0x3;
      const u8_t t0 = flame_heat[x_idx];
      const u8_t t1 = flame_heat[x_idx + 1];
      const u8_t t = t0 + ((((i16_t)t1 - t0) * x_frac) >> 2);
      if (item.is_rgbw) {
        RGBW* pix = reinterpret_cast<RGBW*>(ptr);
        *pix = Lookup5(fire_grad_rgbw[1], t);
        ptr += 4;
      } else {
        RGB* pix = reinterpret_cast<RGB*>(ptr);
        *pix = Lookup5(fire_grad_rgb[1], t);
        ptr += 3;
      }
    }
  }

  void UpdateNozel() {
    u16_t sum = 0;
    for (u8_t i = 0; i < 4; ++i) {
      sum += flame_heat[i];
    }
    const u8_t avg_t = (sum + 3) >> 2;
    RGB noz = Lookup5(fire_grad_rgb[1], avg_t);
    swap_u8(noz.grn, noz.red); // grb pixels
    nozel_rgb_.Fill(noz);
  }

  void UpdateEngineWave(u16_t now_ms) {
    u8_t saw_base = pulse_saw_.Get(now_ms);
    const u8_t min_engine_x = 118;
    // const u8_t max_engine_x = 176;
    u8_t* ptr = reinterpret_cast<u8_t*>(engine_rgbw_.ptr());
    for (u8_t i = 0; i < TAG_ENGINE_LEN; ++i) {
      const LedRange& item = ranges[TAG_ENGINE_START + i];
      u8_t xloc = (item.xloc - min_engine_x); // 0 -> 58
      xloc = (xloc << 2) + saw_base;
      u8_t b = (255 - sin8(255 - xloc));
      b = ((b * u16_t(255 - 64)) >> 8) + 64;
      if (item.is_rgbw) {
        RGBW* pix = reinterpret_cast<RGBW*>(ptr);
        RGBW clr(0xFF);
        Fade(&clr, b);
        *pix = clr;
        ptr += 4;
      } else {
        RGB* pix = reinterpret_cast<RGB*>(ptr);
        RGB clr = kRgbWht;
        Fade(&clr, b);
        *pix = clr;
        ptr += 3;
      }
    }
  }

  void UpdateBodyZips(u16_t now_ms) {
    u8_t zip_base = zip_saw_.Get(now_ms);
    body_rgbw_.Fill(RGBW(0x20));
    for (u8_t i = 0; i < ARRAY_SIZE(zips_); ++i) {
      ZipInfo& zip = zips_[i];
      if (!zip.active && zip.start_time < now_ms) {
        zip.active = true;
        zip.phase_offset = zip_base;
        zip.prev_phase = 0;
      }

      if (zip.active) {
        u8_t zip_phase_base = zip_base - zip.phase_offset;
        if (zip_phase_base < zip.prev_phase) {  // complete
          zip.active = false;
          // delay up to 2 seconds
          zip.start_time = u16_t(now_ms + (u16_t(random8(128)) << 4));
          zip.prev_phase = 0;
          continue;
        }
        zip.prev_phase = zip_phase_base;
        const u8_t body_max_x = 211;
        // const u8_t body_min_x = 44;
        RGBW* pix = &body_rgbw_.At(body_rows[i].idx);
        const LedRange *led = ranges + (TAG_BODY_START + body_rows[i].idx);
        for (u8_t j = 0; j < body_rows[i].len; ++j) {
          if (led[j].xloc > body_max_x) continue;  // leave the head alone
          u8_t loc = (body_max_x - led[j].xloc);
          if (loc < zip_phase_base) continue;
          if (loc > int(zip_phase_base) + 64) continue;
          u8_t adden = sin8((loc - zip_phase_base) << 1);
          pix[j].wht += (adden - 128);
        }
      }
    }
    bool zips_quiet = true;
    for (u8_t i = 0; i < ARRAY_SIZE(zips_); ++i) {
      ZipInfo& zip = zips_[i];
      if (zip.active || (zip.start_time - now_ms < 196)) {
        zips_quiet = false;
      }
    }
    RGB* pix = &eye_rgb_.At(0);
    if (zips_quiet) {
      // If all zips are quiet make the eye red and fire all of the zips in the
      // next quarter second, just because...
      *pix = RGB(0x80, 0, 0);
      u8_t sm_idx = 0;
      u8_t sm_off = 255;
      for (u8_t i = 0; i < ARRAY_SIZE(zips_); ++i) {
        const u8_t off = random8(255);
        if (off < sm_off) {
          sm_off = off;
          sm_idx = i;
        }
        zips_[i].start_time = now_ms + off;
      }
      // Make the closest time now, so that we don't find quiet zips again.
      zips_[sm_idx].start_time = now_ms;
    } else {
      RGB wht = kRgbWht;
      Fade(&wht, 0x80);
      Blend(pix, wht, 0x08);
    }
  }

  void UpdateFlying(u16_t now_ms, u8_t speed) {
    UpdateFlame(speed);
    UpdateNozel();

    UpdateEngineWave(now_ms);
    UpdateBodyZips(now_ms);
  }

  void UpdateFire(u16_t now_ms, u8_t speed) {
    UpdateFlame(speed);
    UpdateNozel();
    UpdateEngineWave(now_ms);

    const u16_t fire = 3 * speed; // 0-255
    const int cooling = 80 - ((50 * fire) >> 8);  // 85 - 35
    const int sparking = 50 + ((150 * fire) >> 8); // 50 - 200
    for (u8_t i = 0; i < ARRAY_SIZE(heat); ++i) {
      UpdateHeatMap(cooling, sparking, ARRAY_SIZE(heat[i]), heat[i]);
    }
    const u8_t body_max_x = 243;
    // const u8_t body_min_x = 44;

    u8_t* ptr = reinterpret_cast<u8_t*>(body_rgbw_.ptr());
    int row = 0;
    for (unsigned i = 0; i < TAG_BODY_LEN; ++i) {
      const LedRange& item = ranges[TAG_BODY_START + i];
      if (i >= body_rows[row + 1].idx) {
        ++row;
      }

      // Xragne 44 -> 243 = 199; (20 * 256) / 199 = ~26
      i16_t x_val = (int(26) * (body_max_x - item.xloc));
      u8_t x_idx = x_val >> 8; // 0 -> 20ish
      u8_t x_frac = x_val & ((1 << 8) - 1);
      if (x_idx > 20) {
        DBG_MD(APP, ("I: %d x: %d x_val: %d x_idx: %d x_frac: %d",
                     i, body_max_x - item.xloc, x_val, x_idx, x_frac));
      }
      const u8_t t0 = heat[row][x_idx];
      const u8_t t1 = heat[row][x_idx + 1];
      const u8_t t = t0 + ((((i16_t)t1 - t0) * x_frac) >> 8);

      if (item.is_rgbw) {
        RGBW* pix = reinterpret_cast<RGBW*>(ptr);
        *pix = RGBW(t);
        ptr += 4;
      } else {
        RGB* pix = reinterpret_cast<RGB*>(ptr);
        RGB clr = kRgbWht;
        Fade(&clr, t);
        *pix = clr;
        ptr += 3;
      }
    }
  }

  RGBW GetEyeColor(u16_t now_ms, u8_t mode, u8_t speed, u8_t pulse_val) {
    eye_pulse_start_ms_ = 0;
    return tag_clr_rgbw[TAG_EYE];

    if (eye_pulse_start_ms_ == 0) {  // First entry...
      eye_pulse_start_ms_ = now_ms;
      eye_saw_.SetState(speed, /*offset=*/0);
      red_eye_ = true;
    }
    u16_t pulse_delta = now_ms - eye_pulse_start_ms_;
    if (red_eye_) {
      u8_t frac = eye_saw_.Get(pulse_delta) + 32;
      if (frac < 128) {
        RGBW eye(0xFF, 0, 0, 0);
        u8_t val = 255 - ((sin8(frac) - 128) << 1);
        Blend(&eye, tag_clr_rgbw[TAG_EYE], val);
        return eye;
      }
      red_eye_ = false;
    }

    return tag_clr_rgbw[TAG_EYE];
  }

  void UpdateFlat(u16_t now_ms, u8_t mode, u8_t speed, u8_t bright) {
    RGBW wht_rgbw(0xFF);

    RGB org_rgb(0xFF, 0x44, 0);
    RGBW org_rgbw(0xFF, 0x44, 0);

    RGB blu_rgb(0, 0, 0xFF);
    RGBW blu_rgbw(0, 0, 0xFF);

    RGBW eye = GetEyeColor(now_ms, mode, speed, bright);
    Fade(&eye, bright);

    engine_rgbw_.Fill(wht_rgbw);
    engine_rgb_.Fill(wht_rgbw);
    RGB nrgb(org_rgb.grn, org_rgb.red, org_rgb.blu);
    nozel_rgb_.Fill(nrgb);  // GRB
    fire_rgbw_.Fill(org_rgbw);
    fire_rgb_.Fill(org_rgb);
    body_rgbw_.Fill(wht_rgbw);
    eye_rgb_.Fill(RGB(eye));
  }


  void UpdatePulse(u16_t now_ms, u8_t mode, u8_t speed, u8_t bright) {
    const u8_t pulse_val = sin8(pulse_saw_.Get(now_ms));
    const u8_t pulse_brt = bscale8(pulse_val, bright);

    RGBW eye = GetEyeColor(now_ms, mode, speed, pulse_val);
    Fade(&eye, bright);  // eye's aren't faded by pulse.

    RGB wht_rgb = kRgbWht; Fade(&wht_rgb, pulse_brt);
    RGBW wht_rgbw(0xFF); Fade(&wht_rgbw, pulse_brt);

    RGB org_rgb(0xFF, 0x44, 0); Fade(&org_rgb, pulse_brt);
    RGBW org_rgbw(0xFF, 0x44, 0); Fade(&org_rgbw, pulse_brt);

    RGB blu_rgb(0, 0, 0xFF); Fade(&blu_rgb, pulse_brt);
    RGBW blu_rgbw(0, 0, 0xFF); Fade(&blu_rgbw, pulse_brt);


    engine_rgbw_.Fill(wht_rgbw);
    engine_rgb_.Fill(wht_rgbw);
    RGB nrgb(org_rgb.grn, org_rgb.red, org_rgb.blu);
    nozel_rgb_.Fill(nrgb);  // GRB
    fire_rgbw_.Fill(org_rgbw);
    fire_rgb_.Fill(org_rgb);
    body_rgbw_.Fill(wht_rgbw);
    eye_rgb_.Fill(RGB(eye));
  }

  void SetWavePix(u8_t phase, u8_t r_idx, void* pix_ptr) {
    const LedRange& item = ranges[r_idx];
    int xloc = (item.xloc - 230) >> 1;
    int yloc = (item.yloc - 79) >> 1;
    int dist_sq = (xloc * xloc + yloc * yloc);
    int dist = sqrt(dist_sq);
    const u8_t item_phase = phase + (dist << 1);
    const u8_t glow = max(16, sin8(item_phase));
    if (item.is_rgbw) {
      RGBW* pix = reinterpret_cast<RGBW*>(pix_ptr);
      *pix = tag_clr_rgbw[item.tag];
      Fade(pix, glow);
    } else {
      RGB* pix = reinterpret_cast<RGB*>(pix_ptr);
      *pix = tag_clr_rgb[item.tag];
      Fade(pix, glow);
    }
  }

  void UpdateWave(u16_t now_ms, u8_t speed) {
    u8_t phase = ~pulse_saw_.Get(now_ms);
    for (u8_t i = 0; i < body_rgbw_.len(); ++i) {
      SetWavePix(phase, TAG_BODY_START + i, &body_rgbw_.At(i));
    }
    for (u8_t i = 0; i < engine_rgbw_.len(); ++i) {
      SetWavePix(phase, TAG_ENGINE_START + i, &engine_rgbw_.At(i));
    }
    for (u8_t i = 0; i < engine_rgb_.len(); ++i) {
      SetWavePix(phase, TAG_ENGINE_START + engine_rgbw_.len() + i,
                 &engine_rgb_.At(i));
    }
    SetWavePix(phase, TAG_EYE_START, &eye_rgb_.At(0));
    UpdateFlame(speed);
    UpdateNozel();

  }

protected:
  const PinId led_pin_;
  State state_;

  u8_t skip_cnt_;
  bool red_eye_;
  u16_t eye_pulse_start_ms_;
  LedSpan<RGBW> engine_rgbw_;
  LedSpan<RGB> engine_rgb_;
  LedSpan<RGB> nozel_rgb_;
  LedSpan<RGBW> fire_rgbw_;
  LedSpan<RGB>  fire_rgb_;
  LedSpan<RGBW> body_rgbw_;
  LedSpan<RGB> eye_rgb_;

  VariableSaw pulse_saw_;
  VariableSaw eye_saw_;
  VariableSaw zip_saw_;
  u8_t flame_heat[21];  // Just for the exhaust
  u8_t heat[5][21];  // for body flame mode

  struct ZipInfo {
    bool active;
    u8_t phase_offset;
    u8_t prev_phase;
    u16_t start_time;
  };
  ZipInfo zips_[5];
  u8_t prev_mode_;
};

#if 0
void CalcPrintDists(int center_x, int center_y) {
  // int center_x = 106;
  // int center_y = 101;
  for (unsigned i = 0; i < ARRAY_SIZE(ranges); ++i) {
    const LedRange& range = ranges[i];
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
  DBG_MD(APP, ("FlyingPorcupine: Run\n"));

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
    state.Print();

    u8_t now_16s = now_s >> 4;
    if (now_16s == update_16s) continue;  // 16 sec
    update_16s = now_16s;
  }
}
