// Copyright 2024 Thomas DeWeese
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
#include "Util.h"

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

#define OCHIN_RGBW_CNT (7)
#define OCHEEK_RGBW_CNT (15)
#define OCHEEK_RGB_CNT (5)
#define LHORN_RGBW_CNT (6)
#define LEYE_RGBW_CNT (3)
#define REYE_RGBW_CNT (3)
#define MOUTH_RGBW_CNT (11)
#define BCHEEK_RGBW_CNT (37)
#define RHORN_RGBW_CNT (5)

#define FORE_RGBW_CNT(35)
#define FORE_RGB_CNT(27)

#define RGBW_CNT (OCHIN_RGBW_CNT + OCHEEK_RGBW_CNT + LHORN_RGBW_CNT + LEYE_RGBW_CNT + REYE_RGBW_CNT + \
                  MOUTH_RGBW_CNT + BCHEEK_RGBW_CNT + RHORN_RGBW_CNT + FORE_RGBW_CNT)
#define RGB_CNT (OCHEEK_RGB_CNT + +FORE_RGB_CNT)

u8_t led_data[RGBW_CNT * 4 + RGB_CNT * 3];

static const RGB fire_grad_rgb[3][5]  = {/* Green */
                                         { RGB(0x00, 0x00, 0x00),
                                           RGB(0x00, 0x40, 0x00),
                                           RGB(0x00, 0x80, 0x20),
                                           RGB(0x00, 0xFF, 0x80),
                                           RGB(0x80, 0xFF, 0xFF) },
                                         /* Blue */
                                         { RGB(0x00, 0x00, 0x00),
                                           RGB(0x00, 0x00, 0x40),
                                           RGB(0x00, 0x00, 0x80),
                                           RGB(0x00, 0x44, 0xFF),
                                           RGB(0x40, 0x80, 0xFF) },
                                         /* Red */
                                         { RGB(0x00, 0x00, 0x00),
                                           RGB(0x40, 0x00, 0x00),
                                           RGB(0x80, 0x20, 0x00),
                                           RGB(0xFF, 0x80, 0x00),
                                           RGB(0xFF, 0xFF, 0x80) }};

static const RGBW fire_grad_rgbw[3][5] = {/* Green */
                                          { RGBW(0x00, 0x00, 0x00),
                                            RGBW(0x00, 0x40, 0x00),
                                            RGBW(0x00, 0x80, 0x00),
                                            RGBW(0x00, 0xFF, 0x44),
                                            RGBW(0x00, 0xFF, 0x44, 0x80) },
                                          /* Blue */
                                          { RGBW(0x00, 0x00, 0x00),
                                             RGBW(0x00, 0x00, 0x40),
                                             RGBW(0x00, 0x00, 0x80),
                                             RGBW(0x00, 0x44, 0xFF),
                                             RGBW(0x00, 0x44, 0xFF, 0x40)
                                          }, /* Red */
                                          { RGBW(0x00),
                                            RGBW(0x40, 0x00, 0x00),
                                            RGBW(0x80, 0x20, 0x00),
                                            RGBW(0xFF, 0x80, 0x00),
                                            RGBW(0x80, 0x80, 0x00, 0x80) }};
inline u8_t subClip(u8_t a, u8_t b) {
  u8_t r = a - b;
  return (r > a) ? 0 : r;
}

inline u8_t addClip(u8_t a, u8_t b) {
  u8_t r = a + b;
  return (r < a) ? 0xFF : r;
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

/*
template <class led_t> void
MapHeatToLed(u8_t bright, u8_t len, u8_t* heat, LedSpan<led_t>& leds, const led_t gradient[5]) {
  if (len == leds.len()) {
    for (int i = 0; i < leds.len(); ++i) {
      led_t& v = leds.At(i);
      v = Lookup5(gradient, heat[i]);
      Fade(&v, bright);
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
      Fade(&v, bright);
      heat_pos += heat_step;
    }
  }
}

enum part_tags {
      TAG_HEAD = 0,
      TAG_CNTR,
      TAG_TAIL,
      TAG_LIWG,
      TAG_RIWG,
      TAG_LOWG,
      TAG_ROWG,
      TAG_COUNT
};

struct led_range {
  u8_t dist, len;
  bool is_rgbw;
  part_tags tag;
};

const led_range ranges[] = {
                            {5<<5,  2, true,  TAG_HEAD},  // head crest
                            {4<<5,  4, true,  TAG_HEAD},  // head nose
                            {3<<5,  1, true,  TAG_HEAD},  // head neck
                            {3<<5,  6, false, TAG_HEAD},  // head neck
                            {4<<5,  6, false, TAG_HEAD},  // head nose
                            {5<<5,  3, false, TAG_HEAD},  // head crest
                            {2<<5,  2, true,  TAG_CNTR},  // rec bot
                            {1<<5,  1, true,  TAG_CNTR},  // rec center
                            {0<<5,  1, true,  TAG_CNTR},  // rec center
                            {1<<5,  1, true,  TAG_CNTR},  // rec center
                            {2<<5,  2, true,  TAG_CNTR},  // rec top
                            {2<<5,  3, false, TAG_CNTR},  // rec left
                            {1<<5,  6, false, TAG_CNTR},  // rec bot mid
                            {2<<5,  3, false, TAG_CNTR},  // rec right
                            {1<<5,  6, false, TAG_CNTR},  // rec top mid
                            {3<<5,  1, true,  TAG_TAIL},  // tail
                            {4<<5,  1, true,  TAG_TAIL},  // tail
                            {5<<5,  1, true,  TAG_TAIL},  // tail
                            {6<<5,  1, true,  TAG_TAIL},  // tail
                            {7<<5,  1, true,  TAG_TAIL},  // tail
                            {0xFF,  1, true,  TAG_TAIL},  // tail
                            {5<<5,  2, false, TAG_TAIL},  // tail left mid
                            {4<<5,  2, false, TAG_TAIL},  // tail left mid
                            {3<<5,  2, false, TAG_TAIL},  // tail rt top
                            {4<<5,  7, true,  TAG_LIWG},  // Left Wing out
                            {2<<5,  5, true,  TAG_LIWG},  // Left Wing in
                            {3<<5, 12, false, TAG_LIWG},  // Left Wing mid
                            {4<<5,  7, true,  TAG_RIWG},  // Right Wing out
                            {2<<5,  5, true,  TAG_RIWG},  // Right Wing in
                            {3<<5, 12, false, TAG_RIWG},  // Right Wing mid
                            {5<<5,  8, true,  TAG_LOWG},  // Left Wing in
                            {7<<5, 12, true,  TAG_LOWG},  // Left Wing out
                            {6<<5, 21, false, TAG_LOWG}, // Left Wing mid
                            {5<<5,  8, true,  TAG_ROWG},  // Right Wing in
                            {7<<5, 12, true,  TAG_ROWG},  // Right Wing out
                            {6<<5, 21, false, TAG_ROWG}, // Right Wing mid
};

const RGBW tag_clr_rgbw[12][TAG_COUNT] =
  {
   // Pallete 0
   { // Green Tint
     RGBW(0xFF),              // TAG_HEAD
     RGBW(0x00, 0xFF, 0x00),  // TAG_CNTR
     RGBW(0xFF),              // RAG_TAIL
     RGBW(0x00, 0xFF, 0x80),  // TAG_LIWG
     RGBW(0x00, 0xFF, 0x80),  // TAG_RIWG
     RGBW(0x00, 0xFF, 0xFF),  // TAG_LOWG
     RGBW(0x00, 0xFF, 0xFF),  // TAG_ROWG
    },
   { // Red heart
    RGBW(0xFF),              // TAG_HEAD
    RGBW(0xFF, 0x00, 0x00),  // TAG_CNTR
    RGBW(0xFF),              // RAG_TAIL
    RGBW(0xFF),              // TAG_LIWG
    RGBW(0xFF),              // TAG_RIWG
    RGBW(0xFF),              // TAG_LOWG
    RGBW(0xFF),              // TAG_ROWG
   },
   { // Green heart
    RGBW(0xFF),              // TAG_HEAD
    RGBW(0x00, 0xFF, 0x00),  // TAG_CNTR
    RGBW(0xFF),              // RAG_TAIL
    RGBW(0xFF),              // TAG_LIWG
    RGBW(0xFF),              // TAG_RIWG
    RGBW(0xFF),              // TAG_LOWG
    RGBW(0xFF),              // TAG_ROWG
   },
   { // All green (not used)
    RGBW(0x00, 0xFF, 0x00),  // TAG_HEAD
    RGBW(0x00, 0xFF, 0x00),  // TAG_CNTR
    RGBW(0x00, 0xFF, 0x00),  // RAG_TAIL
    RGBW(0x00, 0xFF, 0x00),  // TAG_LIWG
    RGBW(0x00, 0xFF, 0x00),  // TAG_RIWG
    RGBW(0x00, 0xFF, 0x00),  // TAG_LOWG
    RGBW(0x00, 0xFF, 0x00),  // TAG_ROWG
   },
   // Pallete 1
   {  // Blue tint
    RGBW(0xFF),              // TAG_HEAD
    RGBW(0x00, 0x00, 0xFF),  // TAG_CNTR
    RGBW(0xFF),              // RAG_TAIL
    RGBW(0x00, 0x80, 0xFF),  // TAG_LIWG
    RGBW(0x00, 0x80, 0xFF),  // TAG_RIWG
    RGBW(0x00, 0xFF, 0xFF),  // TAG_LOWG
    RGBW(0x00, 0xFF, 0xFF),  // TAG_ROWG
   },
   { // Red heart
    RGBW(0xFF),              // TAG_HEAD
    RGBW(0xFF, 0x00, 0x00),  // TAG_CNTR
    RGBW(0xFF),              // RAG_TAIL
    RGBW(0xFF),              // TAG_LIWG
    RGBW(0xFF),              // TAG_RIWG
    RGBW(0xFF),              // TAG_LOWG
    RGBW(0xFF),              // TAG_ROWG
   },
   { // Blue heart
    RGBW(0xFF),              // TAG_HEAD
    RGBW(0x00, 0x00, 0xFF),  // TAG_CNTR
    RGBW(0xFF),              // RAG_TAIL
    RGBW(0xFF),              // TAG_LIWG
    RGBW(0xFF),              // TAG_RIWG
    RGBW(0xFF),              // TAG_LOWG
    RGBW(0xFF),              // TAG_ROWG
   },
   { // All Blue (not used)
    RGBW(0x00, 0x00, 0xFF),  // TAG_HEAD
    RGBW(0x00, 0x00, 0xFF),  // TAG_CNTR
    RGBW(0x00, 0x00, 0xFF),  // RAG_TAIL
    RGBW(0x00, 0x00, 0xFF),  // TAG_LIWG
    RGBW(0x00, 0x00, 0xFF),  // TAG_RIWG
    RGBW(0x00, 0x00, 0xFF),  // TAG_LOWG
    RGBW(0x00, 0x00, 0xFF),  // TAG_ROWG
   },
   // Pallete 2
   {  // Red tint
    RGBW(0xFF),              // TAG_HEAD
    RGBW(0xFF, 0x00, 0x00),  // TAG_CNTR
    RGBW(0xFF),              // RAG_TAIL
    RGBW(0xFF, 0x40, 0x00),  // TAG_LIWG
    RGBW(0xFF, 0x40, 0x00),  // TAG_RIWG
    RGBW(0xFF, 0x80, 0x00),  // TAG_LOWG
    RGBW(0xFF, 0x80, 0x00),  // TAG_ROWG
   },
   { // Red heart
    RGBW(0xFF),              // TAG_HEAD
    RGBW(0xFF, 0x00, 0x00),  // TAG_CNTR
    RGBW(0xFF),              // RAG_TAIL
    RGBW(0xFF),              // TAG_LIWG
    RGBW(0xFF),              // TAG_RIWG
    RGBW(0xFF),              // TAG_LOWG
    RGBW(0xFF),              // TAG_ROWG
   },
   { // All white
    RGBW(0xFF),              // TAG_HEAD
    RGBW(0xFF),              // TAG_CNTR
    RGBW(0xFF),              // RAG_TAIL
    RGBW(0xFF),              // TAG_LIWG
    RGBW(0xFF),              // TAG_RIWG
    RGBW(0xFF),              // TAG_LOWG
    RGBW(0xFF),              // TAG_ROWG
   },
   { // All Red (not used)
    RGBW(0xFF, 0x00, 0x00),  // TAG_HEAD
    RGBW(0xFF, 0x00, 0x00),  // TAG_CNTR
    RGBW(0xFF, 0x00, 0x00),  // RAG_TAIL
    RGBW(0xFF, 0x00, 0x00),  // TAG_LIWG
    RGBW(0xFF, 0x00, 0x00),  // TAG_RIWG
    RGBW(0xFF, 0x00, 0x00),  // TAG_LOWG
    RGBW(0xFF, 0x00, 0x00),  // TAG_ROWG
   }
  };
const RGB tag_clr_rgb[12][TAG_COUNT] =
  {
   // Pallete 0
   { // Green Tint
    RGB(0xFF, 0xCC, 0xAA), // TAG_HEAD
    RGB(0x00, 0xFF, 0x00), // TAG_CNTR
    RGB(0xFF, 0xCC, 0xAA), // RAG_TAIL
    RGB(0x00, 0xFF, 0x80), // TAG_LIWG
    RGB(0x00, 0xFF, 0x80), // TAG_RIWG
    RGB(0x00, 0xFF, 0xFF), // TAG_LOWG
    RGB(0x00, 0xFF, 0xFF), // TAG_ROWG
    },
   { // Red heart
    RGB(0xFF, 0xCC, 0xAA), // TAG_HEAD
    RGB(0xFF, 0x00, 0x00), // TAG_CNTR
    RGB(0xFF, 0xCC, 0xAA), // RAG_TAIL
    RGB(0xFF, 0xCC, 0xAA), // TAG_LIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_RIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_LOWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_ROWG
   },
   { // Green heart
    RGB(0xFF, 0xCC, 0xAA), // TAG_HEAD
    RGB(0x00, 0xFF, 0x00), // TAG_CNTR
    RGB(0xFF, 0xCC, 0xAA), // RAG_TAIL
    RGB(0xFF, 0xCC, 0xAA), // TAG_LIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_RIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_LOWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_ROWG
   },
   { // All green (not used)
    RGB(0x00, 0xFF, 0x00), // TAG_HEAD
    RGB(0x00, 0xFF, 0x00), // TAG_CNTR
    RGB(0x00, 0xFF, 0x00), // RAG_TAIL
    RGB(0x00, 0xFF, 0x00), // TAG_LIWG
    RGB(0x00, 0xFF, 0x00), // TAG_RIWG
    RGB(0x00, 0xFF, 0x00), // TAG_LOWG
    RGB(0x00, 0xFF, 0x00), // TAG_ROWG
   },
   // Pallete 1
   {  // Blue tint
    RGB(0xFF, 0xCC, 0xAA), // TAG_HEAD
    RGB(0x00, 0x00, 0xFF), // TAG_CNTR
    RGB(0xFF, 0xCC, 0xAA), // RAG_TAIL
    RGB(0x80, 0x00, 0xFF), // TAG_LIWG
    RGB(0x80, 0x00, 0xFF), // TAG_RIWG
    RGB(0x00, 0x00, 0xFF), // TAG_LOWG
    RGB(0x00, 0x00, 0xFF), // TAG_ROWG
    },
   { // Red heart
    RGB(0xFF, 0xCC, 0xAA), // TAG_HEAD
    RGB(0xFF, 0x00, 0x00), // TAG_CNTR
    RGB(0xFF, 0xCC, 0xAA), // RAG_TAIL
    RGB(0xFF, 0xCC, 0xAA), // TAG_LIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_RIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_LOWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_ROWG
   },
   { // Blue heart
    RGB(0xFF, 0xCC, 0xAA), // TAG_HEAD
    RGB(0x00, 0x00, 0xFF), // TAG_CNTR
    RGB(0xFF, 0xCC, 0xAA), // RAG_TAIL
    RGB(0xFF, 0xCC, 0xAA), // TAG_LIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_RIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_LOWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_ROWG
   },
   { // All Blue (not used)
    RGB(0x00, 0x00, 0xFF), // TAG_HEAD
    RGB(0x00, 0x00, 0xFF), // TAG_CNTR
    RGB(0x00, 0x00, 0xFF), // RAG_TAIL
    RGB(0x00, 0x00, 0xFF), // TAG_LIWG
    RGB(0x00, 0x00, 0xFF), // TAG_RIWG
    RGB(0x00, 0x00, 0xFF), // TAG_LOWG
    RGB(0x00, 0x00, 0xFF), // TAG_ROWG
   },
   // Pallete 2
   {  // Red tint
    RGB(0xFF, 0xCC, 0xAA), // TAG_HEAD
    RGB(0xFF, 0x00, 0x00), // TAG_CNTR
    RGB(0xFF, 0xCC, 0xAA), // RAG_TAIL
    RGB(0xFF, 0x80, 0x00), // TAG_LIWG
    RGB(0xFF, 0x80, 0x00), // TAG_RIWG
    RGB(0xFF, 0x00, 0x00), // TAG_LOWG
    RGB(0xFF, 0x00, 0x00), // TAG_ROWG
    },
   { // Red heart
    RGB(0xFF, 0xCC, 0xAA), // TAG_HEAD
    RGB(0xFF, 0x00, 0x00), // TAG_CNTR
    RGB(0xFF, 0xCC, 0xAA), // RAG_TAIL
    RGB(0xFF, 0xCC, 0xAA), // TAG_LIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_RIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_LOWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_ROWG
   },
   { // All White
    RGB(0xFF, 0xCC, 0xAA), // TAG_HEAD
    RGB(0xFF, 0xCC, 0xAA), // TAG_CNTR
    RGB(0xFF, 0xCC, 0xAA), // RAG_TAIL
    RGB(0xFF, 0xCC, 0xAA), // TAG_LIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_RIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_LOWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_ROWG
   },
   { // All Red (not used)
    RGB(0xFF, 0x00, 0x00), // TAG_HEAD
    RGB(0xFF, 0x00, 0x00), // TAG_CNTR
    RGB(0xFF, 0x00, 0x00), // RAG_TAIL
    RGB(0xFF, 0x00, 0x00), // TAG_LIWG
    RGB(0xFF, 0x00, 0x00), // TAG_RIWG
    RGB(0xFF, 0x00, 0x00), // TAG_LOWG
    RGB(0xFF, 0x00, 0x00), // TAG_ROWG
   },
 };
*/

class State {
public:
  State(u8_t mode, u8_t bright, u8_t speed, u8_t pallete)
    : mode_(mode), bright_(bright), speed_(speed), pallete_(pallete) { }

  u8_t GetMode() { return mode_; }
  void SetMode(u8_t mode) {
    mode_ = mode;
    if (mode_ > 6) mode_ = 0;
  }
  void IncMode() {
    ++mode_;
    if (mode_ > 6) mode_ = 0;
  }

  u8_t GetBright() { return bright_; }
  void SetBright(u8_t bright) {
    bright_ = bright;
  }
  void IncBright() {
    if (bright_ < 8) {
      bright_ = 16;
    } else  if (bright_ == 255) {
      bright_ = 0;
    } else if (bright_ >= 64) {
      bright_ = 255;
    } else {
      bright_ = bright_ << 2;
    }
  }

  u8_t GetSpeed() { return speed_; }
  void SetSpeed(u8_t speed) {
    speed_ = speed;
    if (speed > 64) speed = 64;
  }
  void IncSpeed() {
    if (speed_ == 0) {
      speed_ = 8;
    } else {
      speed_ = speed_ << 1;
      if (speed_ > 64) speed_ = 0;
    }
  }

  u8_t GetPallete() { return pallete_; }
  void SetPallete(u8_t pallete) {
    pallete_ = pallete;
    if (pallete_ > 2) pallete_ = 2;
  }
protected:
  u8_t mode_;
  u8_t bright_;
  u8_t speed_;
  u8_t pallete_;
};

class Lights {
public:
  Lights(PinId led_pin) :
    led_pin_(led_pin),
    btn_state_(/*mode_=*/1, /*bright_=*/255, /*speed_=*/16, /*pallete_=*/2),
    host_state_(/*mode_=*/5, /*bright_=*/64, /*speed_=*/16, /*pallete_=*/1) {
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
    pulse_saw_.SetSpeed(16);
  }

  void Update(u16_t now_ms) {
    State& state = (btn_state_.GetMode() == 6) ? host_state_ : btn_state_;
    Update(now_ms, state.GetMode(), btn_state_.GetBright(), state.GetSpeed(), state.GetPallete());
  }
  State& GetBtnState() { return btn_state_; }
  State& GetHostState() { return host_state_; }

private:
  void Update(u16_t now_ms, u8_t mode, u8_t bright, u8_t speed, u8_t pallete) {
    pulse_saw_.SetSpeed(speed);
    if (bright == 0 || mode == 0) {
      memset(led_data, 0, sizeof(led_data));
    } else {
      switch (mode) {
      case 1:
        UpdatePulse(now_ms, bright);
        break;
      case 2:
      case 3:
      case 4:
        // UpdateWave(now_ms, bright, (pallete << 2) + mode - 2);
        break;
      case 5:
        //UpdateFire(bright, speed, pallete);
        break;
      }
    }
    PushLeds();
  }

  // void UpdateFire(u8_t bright, u8_t speed, u8_t pallete) {
  //   const u16_t fire = 4 * speed; // 0-255
  //   const int cooling = 85 - ((50 * fire) >> 8);  // 85 - 35
  //   const int sparking = 50 + ((150 * fire) >> 8); // 50 - 200
  //   pallete = ((pallete < 0) ? 0 : ((pallete > 2) ? 2 : pallete));
  //   const RGB*  grad_rgb  = fire_grad_rgb [pallete];
  //   const RGBW* grad_rgbw = fire_grad_rgbw[pallete];
  //   UpdateHeatMap(cooling, sparking, ARRAY_SIZE(heat_lo), heat_lo);
  //   UpdateHeatMap(cooling, sparking, ARRAY_SIZE(heat_ro), heat_ro);
  //   UpdateHeatMap(cooling, sparking, ARRAY_SIZE(heat_li), heat_li);
  //   UpdateHeatMap(cooling, sparking, ARRAY_SIZE(heat_ri), heat_ri);
  //   LedSpan<RGBW> in;
  //   LedSpan<RGB>  mid;
  //   LedSpan<RGBW> out;
  //   in.SetSpan(low_rgbw.ptr(), 12, /*reverse=*/false);
  //   mid.SetSpan(low_rgb.ptr(), OW_RGB_CNT, /*reverse=*/false);
  //   out.SetSpan(in.next_ptr(), 8, /*reverse=*/true);
  //   MapHeatToLed(bright, ARRAY_SIZE(heat_lo), heat_lo, in,  grad_rgbw);
  //   MapHeatToLed(bright, ARRAY_SIZE(heat_lo), heat_lo, mid, grad_rgb);
  //   MapHeatToLed(bright, ARRAY_SIZE(heat_lo), heat_lo, out, grad_rgbw);
  //
  //   in.SetSpan(row_rgbw.ptr(), 12, /*reverse=*/false);
  //   mid.SetSpan(row_rgb.ptr(), OW_RGB_CNT, /*reverse=*/false);
  //   out.SetSpan(in.next_ptr(), 8, /*reverse=*/true);
  //   MapHeatToLed(bright, ARRAY_SIZE(heat_ro), heat_ro, in,  grad_rgbw);
  //   MapHeatToLed(bright, ARRAY_SIZE(heat_ro), heat_ro, mid, grad_rgb);
  //   MapHeatToLed(bright, ARRAY_SIZE(heat_ro), heat_ro, out, grad_rgbw);
  //
  //   out.SetSpan(liw_rgbw.ptr(), 7, /*reverse=*/false);
  //   mid.SetSpan(liw_rgb.ptr(), IW_RGB_CNT, /*reverse=*/false);
  //   in.SetSpan(out.next_ptr(), 5, /*reverse=*/true);
  //   MapHeatToLed(bright, ARRAY_SIZE(heat_li), heat_li, in,  grad_rgbw);
  //   MapHeatToLed(bright, ARRAY_SIZE(heat_li), heat_li, mid, grad_rgb);
  //   MapHeatToLed(bright, ARRAY_SIZE(heat_li), heat_li, out, grad_rgbw);
  //
  //   out.SetSpan(riw_rgbw.ptr(), 7, /*reverse=*/false);
  //   mid.SetSpan(riw_rgb.ptr(), IW_RGB_CNT, /*reverse=*/false);
  //   in.SetSpan(out.next_ptr(), 5, /*reverse=*/true);
  //   MapHeatToLed(bright, ARRAY_SIZE(heat_ri), heat_ri, in,  grad_rgbw);
  //   MapHeatToLed(bright, ARRAY_SIZE(heat_ri), heat_ri, mid, grad_rgb);
  //   MapHeatToLed(bright, ARRAY_SIZE(heat_ri), heat_ri, out, grad_rgbw);
  //
  //   u16_t sum = 0;
  //   for (u8_t i = 0; i < ARRAY_SIZE(heat_lo); ++i) {
  //     sum += heat_lo[i];
  //     sum += heat_ro[i];
  //   }
  //   for (u8_t i = 0; i < ARRAY_SIZE(heat_li); ++i) {
  //     sum += heat_li[i];
  //     sum += heat_ri[i];
  //   }
  //   const u8_t t = sum / (4 * (ARRAY_SIZE(heat_li) + ARRAY_SIZE(heat_lo)));
  //
  //   RGBW c_rgbw = Lookup5(grad_rgbw, t);
  //   Fade(&c_rgbw, bright);
  //   rec_rgbw.Fill(c_rgbw);
  //   RGB c_rgb = Lookup5(grad_rgb, t);
  //   Fade(&c_rgb, bright);
  //   rec_rgb.Fill(c_rgb);
  //
  //   RGB wht(bright);
  //   RGBW wwht(bright);
  //   head_rgbw.Fill(wwht);
  //   head_rgb.Fill(wht);
  //   tail_rgbw.Fill(wwht);
  //   tail_rgb.Fill(wht);
  // }

  void UpdatePulse(u16_t now_ms, u8_t bright) {
    u8_t glow = 64; // sin8(pulse_saw_.Get(now_ms));
    // glow = bscale8(glow, bright);
    RGB clr(0xFF, 0xCC, 0xAA);
    Fade(&clr, glow);
    RGBW color(glow);
    ochin_rgbw.Fill(color);
    ocheek_rgbw.Fill(color);
    ocheek_rgb.Fill(clr);
    lhorn_rgbw.Fill(color);
    leye_rgbw.Fill(color);
    reye_rgbw.Fill(color);
    mouth_rgbw.Fill(color);
    bcheek_rgbw.Fill(color);
    rhorn_rgbw.Fill(color);
    fore_rgbw.Fill(color);
    fore_rgb.Fill(clr);
  }

  // void UpdateWave(u16_t now_ms, u8_t bright, u8_t pallete) {
  //   u8_t phase = ~pulse_saw_.Get(now_ms);
  //   void* ptr = led_data;
  //   const RGBW* clr_rgbw = tag_clr_rgbw[pallete];
  //   const RGB* clr_rgb = tag_clr_rgb[pallete];
  //   for (u8_t i = 0; i < ARRAY_SIZE(ranges); ++i) {
  //     const led_range& item = ranges[i];
  //     const u8_t item_phase = phase + item.dist;
  //     const u8_t glow = bscale8(sin8(item_phase) >> 2, bright);
  //     if (item.is_rgbw) {
  //       LedSpan<RGBW> span(ptr, item.len, false);
  //       RGBW clr = clr_rgbw[item.tag];
  //       Fade(&clr, glow);
  //       span.Fill(clr);
  //       ptr = span.next_ptr();
  //     } else {
  //       LedSpan<RGB> span(ptr, item.len, false);
  //       RGB clr = clr_rgb[item.tag];
  //       Fade(&clr, glow);
  //       span.Fill(clr);
  //       ptr = span.next_ptr();
  //     }
  //   }
  // }

  void PushLeds() {
    SendWS2812(led_pin_, led_data, sizeof(led_data), 0xFF);
  }

protected:
  const PinId led_pin_;
  State btn_state_;
  State host_state_;

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

  VariableSaw pulse_saw_;

  u8_t heat_lo[12];
  u8_t heat_ro[12];
  u8_t heat_li[7];
  u8_t heat_ri[7];
};

void ProcessCmd(State& host_state, u8_t* line) {
  const u8_t kBase = 'A';
  const u8_t kVersion =  kBase + 0;
  u8_t *ptr = line;
  if (*ptr != kVersion) {
    DBG_MD(APP, ("Wrong Version: %c expecting: %c\n", *ptr, kVersion));
    return;
  }
  u8_t mode = *++ptr - kBase;
  if (mode > 5) mode = 5;
  u8_t pallete = *++ptr - kBase;
  if (pallete > 1) pallete = 1;
  u8_t speed = *++ptr - kBase;
  if (speed > 64) speed = 64;

  host_state.SetMode(mode);
  host_state.SetPallete(pallete);
  host_state.SetSpeed(speed);
  DBG_MD(APP, ("State: M:%d P:%d S:%d\n", mode, pallete, speed));
}

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  Button btn_mode(PIN_C4);
  Button btn_brt(PIN_C5);
  Button btn_spd(PIN_C6);

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  DBG_LEVEL_MD(SBUS);

  Serial& bt_serial = Serial::usart3;
  bt_serial.Setup(115200, 8, 0, 1);

  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();
  blink_pin.toggle();

  PinId rgb_led_pin(RGB_LED_PIN);
  rgb_led_pin.SetOutput();

  sei();
  DBG_MD(APP, ("LogoPlaque: Run\n"));

  memset(led_data, 0, sizeof(led_data));
  SendWS2812(rgb_led_pin, led_data, sizeof(led_data), 0xFF);
  Lights lights(rgb_led_pin);

  // CtrlState state;
  u8_t blink_phase = 0;

  u8_t update_0 = 0;
  u8_t update_3 = 0;
  u8_t update_5 = 0;
  u8_t update_8 = 0;
  u8_t update_16s = 0xFF;

  bool bad_data = false;
  u8_t data_pos = 0;
  u8_t read_data[64];

  while (1) {
    u16_t now = FastTimeMs();

    if (update_0 == now) continue;  // 1ms
    update_0 = now;
    if (bt_serial.Avail()) {
      u8_t rd_err;
      u8_t ch = bt_serial.Read(&rd_err);
      if (rd_err) {
        bad_data = true;
      } else if (ch == '\r') {
        read_data[data_pos] = 0;
        if (!bad_data) {
          ProcessCmd(lights.GetHostState(), read_data);
        }
        data_pos = 0;
        bad_data = false;
      } else {
        read_data[data_pos] = ch;
        data_pos = (data_pos + 1) & 0x3F;
      }
    }

    const u8_t now_3 = now >> 3;   // 1/128 sec
    if (now_3 == update_3) continue;
    update_3 = now_3;
    State& btn_state = lights.GetBtnState();
    if (btn_mode.Check() && btn_mode.Down()) {
      btn_state.IncMode();
    }
    if (btn_brt.Check() && btn_brt.Down()) {
      btn_state.IncBright();
    }
    if (btn_spd.Check() && btn_spd.Down()) {
      btn_state.IncSpeed();
    }

    const u8_t now_5 = now >> 5;   // 1/32 sec
    if (now_5 == update_5) continue;
    update_5 = now_5;
    lights.Update(now);

    const u8_t now_8 = now >> 8;
    if (now_8 == update_8) continue;  // 1/4 sec
    update_8 = now_8;
    u8_t pat = 0x33;
    blink_pin.set(!(pat & (1 << blink_phase)));  // pin high is off.
    blink_phase = (blink_phase + 1) & 0x07;

    u16_t now_s = FastSecs();
    u8_t now_16s = now_s >> 4;
    if (now_16s == update_16s) continue;  // 16 sec
    update_16s = now_16s;
  }
}
