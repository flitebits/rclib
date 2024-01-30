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

#define HEAD_RGBW_CNT (2 + 4 + 1)  // Head leds
#define HEAD_RGB_CNT (3 + 3 + 6 + 3)  // Head leds
#define REC_RGBW_CNT (2 + 3 + 2)
#define REC_RGB_CNT (6 + 3 + 6 + 3)
#define TAIL_RGB_CNT (3 + 3)
#define TAIL_RGBW_CNT (6)
#define IW_RGBW_CNT (7 + 5)
#define IW_RGB_CNT (4 * 3)
#define OW_RGBW_CNT (8 + 12)
#define OW_RGB_CNT (7 * 3)

#define RGBW_CNT (HEAD_RGBW_CNT + TAIL_RGBW_CNT + REC_RGBW_CNT + 2 * (IW_RGBW_CNT + OW_RGBW_CNT))
#define RGB_CNT (HEAD_RGB_CNT + TAIL_RGB_CNT + REC_RGB_CNT + 2 * (IW_RGB_CNT + OW_RGB_CNT))

u8_t led_data[RGBW_CNT * 4 + RGB_CNT * 3];

static const RGB fire_grad_rgb[5]   = { RGB(0x00, 0x00, 0x00),
                                        RGB(0x40, 0x00, 0x00),
                                        RGB(0x80, 0x20, 0x00),
                                        RGB(0xFF, 0x80, 0x00),
                                        RGB(0xFF, 0xFF, 0x80) };
static const RGBW fire_grad_rgbw[5] = { RGBW(0x00),
                                        RGBW(0x40, 0x00, 0x00),
                                        RGBW(0x80, 0x20, 0x00),
                                        RGBW(0xFF, 0x80, 0x00),
                                        RGBW(0x80, 0x80, 0x00, 0x80) };

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

const RGBW tag_clr_rgbw[][TAG_COUNT] =
  {{ RGBW(0xFF),           // TAG_HEAD
     RGBW(0xFF, 0, 0),     // TAG_CNTR
     RGBW(0xFF),           // RAG_TAIL
     RGBW(0x80, 0, 0xFF),  // TAG_LIWG
     RGBW(0x80, 0, 0xFF),  // TAG_RIWG
     RGBW(0x00, 0, 0xFF),  // TAG_LOWG
     RGBW(0x00, 0, 0xFF),  // TAG_ROWG
    },
   {
    RGBW(0xFF),        // TAG_HEAD
    RGBW(0xFF, 0, 0),  // TAG_CNTR
    RGBW(0xFF),        // RAG_TAIL
    RGBW(0xFF),        // TAG_LIWG
    RGBW(0xFF),        // TAG_RIWG
    RGBW(0xFF),        // TAG_LOWG
    RGBW(0xFF),        // TAG_ROWG
   },
   {
    RGBW(0xFF),        // TAG_HEAD
    RGBW(0xFF),        // TAG_CNTR
    RGBW(0xFF),        // RAG_TAIL
    RGBW(0xFF),        // TAG_LIWG
    RGBW(0xFF),        // TAG_RIWG
    RGBW(0xFF),        // TAG_LOWG
    RGBW(0xFF),        // TAG_ROWG
   }};
const RGB tag_clr_rgb[][TAG_COUNT] =
  {{
    RGB(0xFF),             // TAG_HEAD
    RGB(0xFF, 0x00, 0x00), // TAG_CNTR
    RGB(0xFF),             // RAG_TAIL
    RGB(0x80, 0x00, 0xFF), // TAG_LIWG
    RGB(0x80, 0x00, 0xFF), // TAG_RIWG
    RGB(0x00, 0x00, 0xFF), // TAG_LOWG
    RGB(0x00, 0x00, 0xFF), // TAG_ROWG
    },
   {
    RGB(0xFF, 0xCC, 0xAA), // TAG_HEAD
    RGB(0xFF, 0, 0),       // TAG_CNTR
    RGB(0xFF, 0xCC, 0xAA), // RAG_TAIL
    RGB(0xFF, 0xCC, 0xAA), // TAG_LIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_RIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_LOWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_ROWG
   },
   {
    RGB(0xFF, 0xCC, 0xAA), // TAG_HEAD
    RGB(0xFF, 0xCC, 0xAA), // TAG_CNTR
    RGB(0xFF, 0xCC, 0xAA), // RAG_TAIL
    RGB(0xFF, 0xCC, 0xAA), // TAG_LIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_RIWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_LOWG
    RGB(0xFF, 0xCC, 0xAA), // TAG_ROWG
   }};

class Lights {
public:
  Lights(PinId led_pin) : led_pin_(led_pin), mode_(2), bright_(255), speed_(16) {
    void* ptr = led_data;
    ptr = head_rgbw.SetSpan(ptr, HEAD_RGBW_CNT, /*reverse=*/false);
    ptr = head_rgb.SetSpan(ptr, HEAD_RGB_CNT, /*reverse=*/false);
    ptr = rec_rgbw.SetSpan(ptr, REC_RGBW_CNT, /*reverse=*/false);
    ptr = rec_rgb.SetSpan(ptr, REC_RGB_CNT, /*reverse=*/false);
    ptr = tail_rgbw.SetSpan(ptr, TAIL_RGBW_CNT, /*reverse=*/true);
    ptr = tail_rgb.SetSpan(ptr, TAIL_RGB_CNT, /*reverse=*/false);
    ptr = liw_rgbw.SetSpan(ptr, IW_RGBW_CNT, /*reverse=*/false);
    ptr = liw_rgb.SetSpan(ptr, IW_RGB_CNT, /*reverse=*/false);
    ptr = riw_rgbw.SetSpan(ptr, IW_RGBW_CNT, /*reverse=*/false);
    ptr = riw_rgb.SetSpan(ptr, IW_RGB_CNT, /*reverse=*/false);
    ptr = low_rgbw.SetSpan(ptr, OW_RGBW_CNT, /*reverse=*/false);
    ptr = low_rgb.SetSpan(ptr, OW_RGB_CNT, /*reverse=*/false);
    ptr = row_rgbw.SetSpan(ptr, OW_RGBW_CNT, /*reverse=*/false);
    ptr = row_rgb.SetSpan(ptr, OW_RGB_CNT, /*reverse=*/false);
    pulse_saw_.SetSpeed(16);
  }

  void IncMode() {
    ++mode_;
    if (mode_ > 4) mode_ = 0;
  }
  void IncSpeed() {
    switch (speed_) {
    default:
    case  64: speed_ =   0; break;
    case   0: speed_ =   8; break;
    case   8: speed_ =  16; break;
    case  16: speed_ =  32; break;
    case  32: speed_ =  64; break;
    }
    pulse_saw_.SetSpeed(speed_);
  }
  void IncBright() {
    switch (bright_) {
    default:
    case 255: bright_ =   0; break;
    case   0: bright_ =  64; break;
    case  64: bright_ = 128; break;
    case 128: bright_ = 255; break;
    }
  }

  void Update(u16_t now_ms) {
    if (bright_ == 0) {
      memset(led_data, 0, sizeof(led_data));
    } else {
      switch (mode_) {
      case 0:
        UpdatePulse(now_ms);
        break;
      case 1:
        UpdateWave(now_ms, 0);
        break;
      case 2:
        UpdateWave(now_ms, 1);
        break;
      case 3:
        UpdateWave(now_ms, 2);
        break;
      case 4:
        UpdateFire(speed_);
        break;
      }
    }
    PushLeds();
  }

  void UpdateFire(u8_t speed) {
    const u16_t fire = 4 * speed; // 0-255
    const int cooling = 85 - ((50 * fire) >> 8);  // 85 - 35
    const int sparking = 50 + ((150 * fire) >> 8); // 50 - 200
    UpdateHeatMap(cooling, sparking, ARRAY_SIZE(heat_lo), heat_lo);
    UpdateHeatMap(cooling, sparking, ARRAY_SIZE(heat_ro), heat_ro);
    UpdateHeatMap(cooling, sparking, ARRAY_SIZE(heat_li), heat_li);
    UpdateHeatMap(cooling, sparking, ARRAY_SIZE(heat_ri), heat_ri);
    LedSpan<RGBW> in;
    LedSpan<RGB>  mid;
    LedSpan<RGBW> out;
    in.SetSpan(low_rgbw.ptr(), 12, /*reverse=*/false);
    mid.SetSpan(low_rgb.ptr(), OW_RGB_CNT, /*reverse=*/false);
    out.SetSpan(in.next_ptr(), 8, /*reverse=*/true);
    MapHeatToLed(bright_, ARRAY_SIZE(heat_lo), heat_lo, in, fire_grad_rgbw);
    MapHeatToLed(bright_, ARRAY_SIZE(heat_lo), heat_lo, mid, fire_grad_rgb);
    MapHeatToLed(bright_, ARRAY_SIZE(heat_lo), heat_lo, out, fire_grad_rgbw);

    in.SetSpan(row_rgbw.ptr(), 12, /*reverse=*/false);
    mid.SetSpan(row_rgb.ptr(), OW_RGB_CNT, /*reverse=*/false);
    out.SetSpan(in.next_ptr(), 8, /*reverse=*/true);
    MapHeatToLed(bright_, ARRAY_SIZE(heat_ro), heat_ro, in, fire_grad_rgbw);
    MapHeatToLed(bright_, ARRAY_SIZE(heat_ro), heat_ro, mid, fire_grad_rgb);
    MapHeatToLed(bright_, ARRAY_SIZE(heat_ro), heat_ro, out, fire_grad_rgbw);

    out.SetSpan(liw_rgbw.ptr(), 7, /*reverse=*/false);
    mid.SetSpan(liw_rgb.ptr(), IW_RGB_CNT, /*reverse=*/false);
    in.SetSpan(out.next_ptr(), 5, /*reverse=*/true);
    MapHeatToLed(bright_, ARRAY_SIZE(heat_li), heat_li, in, fire_grad_rgbw);
    MapHeatToLed(bright_, ARRAY_SIZE(heat_li), heat_li, mid, fire_grad_rgb);
    MapHeatToLed(bright_, ARRAY_SIZE(heat_li), heat_li, out, fire_grad_rgbw);

    out.SetSpan(riw_rgbw.ptr(), 7, /*reverse=*/false);
    mid.SetSpan(riw_rgb.ptr(), IW_RGB_CNT, /*reverse=*/false);
    in.SetSpan(out.next_ptr(), 5, /*reverse=*/true);
    MapHeatToLed(bright_, ARRAY_SIZE(heat_ri), heat_ri, in, fire_grad_rgbw);
    MapHeatToLed(bright_, ARRAY_SIZE(heat_ri), heat_ri, mid, fire_grad_rgb);
    MapHeatToLed(bright_, ARRAY_SIZE(heat_ri), heat_ri, out, fire_grad_rgbw);

    u16_t sum = 0;
    for (u8_t i = 0; i < ARRAY_SIZE(heat_lo); ++i) {
      sum += heat_lo[i];
      sum += heat_ro[i];
    }
    for (u8_t i = 0; i < ARRAY_SIZE(heat_li); ++i) {
      sum += heat_li[i];
      sum += heat_ri[i];
    }
    const u8_t t = sum / (2 * ARRAY_SIZE(heat_li) + ARRAY_SIZE(heat_lo));
    RGBW c_rgbw = Lookup5(fire_grad_rgbw, t);
    Fade(&c_rgbw, bright_);
    rec_rgbw.Fill(c_rgbw);
    RGB c_rgb = Lookup5(fire_grad_rgb, t);
    Fade(&c_rgb, bright_);
    rec_rgb.Fill(c_rgb);

    RGB wht(bright_);
    RGBW wwht(bright_);
    head_rgbw.Fill(wwht);
    head_rgb.Fill(wht);
    tail_rgbw.Fill(wwht);
    tail_rgb.Fill(wht);
  }

  void UpdatePulse(u16_t now_ms) {
    u8_t glow = sin8(pulse_saw_.Get(now_ms)) >> 2;
    glow = bscale8(glow, bright_);
    RGB wht(glow);
    RGBW wwht(glow);
    head_rgbw.Fill(wwht);
    head_rgb.Fill(wht);
    tail_rgbw.Fill(wwht);
    tail_rgb.Fill(wht);
    rec_rgbw.Fill(wwht);
    rec_rgb.Fill(wht);
    liw_rgbw.Fill(wwht);
    liw_rgb.Fill(wht);
    riw_rgbw.Fill(wwht);
    riw_rgb.Fill(wht);
    low_rgbw.Fill(wwht);
    low_rgb.Fill(wht);
    row_rgbw.Fill(wwht);
    row_rgb.Fill(wht);
  }

  void UpdateWave(u16_t now_ms, u8_t pallete) {
    u8_t phase = ~pulse_saw_.Get(now_ms);
    void* ptr = led_data;
    const RGBW* clr_rgbw = tag_clr_rgbw[pallete];
    const RGB* clr_rgb = tag_clr_rgb[pallete];
    for (u8_t i = 0; i < ARRAY_SIZE(ranges); ++i) {
      const led_range& item = ranges[i];
      const u8_t item_phase = phase + item.dist;
      const u8_t glow = bscale8(sin8(item_phase) >> 2, bright_);
      if (item.is_rgbw) {
        LedSpan<RGBW> span(ptr, item.len, false);
        RGBW clr = clr_rgbw[item.tag];
        Fade(&clr, glow);
        span.Fill(clr);
        ptr = span.next_ptr();
      } else {
        LedSpan<RGB> span(ptr, item.len, false);
        RGB clr = clr_rgb[item.tag];
        Fade(&clr, glow);
        span.Fill(clr);
        ptr = span.next_ptr();
      }
    }
  }

  void PushLeds() {
    SendWS2812(led_pin_, led_data, sizeof(led_data), 0xFF);
  }

protected:
  const PinId led_pin_;
  u8_t mode_, bright_, speed_;

  LedSpan<RGBW> head_rgbw;
  LedSpan<RGB>  head_rgb;
  LedSpan<RGB>  tail_rgb;
  LedSpan<RGBW> tail_rgbw;
  LedSpan<RGBW> rec_rgbw;
  LedSpan<RGB>  rec_rgb;
  LedSpan<RGBW> liw_rgbw;
  LedSpan<RGB>  liw_rgb;
  LedSpan<RGBW> riw_rgbw;
  LedSpan<RGB>  riw_rgb;
  LedSpan<RGBW> low_rgbw;
  LedSpan<RGB>  low_rgb;
  LedSpan<RGBW> row_rgbw;
  LedSpan<RGB>  row_rgb;
  VariableSaw pulse_saw_;

  u8_t heat_lo[12];
  u8_t heat_ro[12];
  u8_t heat_li[7];
  u8_t heat_ri[7];
};

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

  while (1) {
    u16_t now = FastTimeMs();

    if (update_0 == now) continue;  // 1ms
    update_0 = now;

    const u8_t now_3 = now >> 3;   // 1/128 sec
    if (now_3 == update_3) continue;
    update_3 = now_3;
    if (btn_mode.Check() && btn_mode.Down()) {
      lights.IncMode();
    }
    if (btn_brt.Check() && btn_brt.Down()) {
      lights.IncBright();
    }
    if (btn_spd.Check() && btn_spd.Down()) {
      lights.IncSpeed();
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
