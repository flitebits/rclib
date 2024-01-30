// Copyright 2020 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _LED_RGB_
#define _LED_RGB_

#include "../IntTypes.h"

namespace led {
  inline u8_t addsat(u8_t v0, u8_t v1) { // Saturating Add
    if (v0 + v1 >= 256) return 255;
    return v0 + v1;
  }
  inline u8_t addsat(u8_t v0, u8_t v1, u8_t v2) { // Saturating Add
    if (v0 + v1 + v2 >= 256) return 255;
    return v0 + v1 + v2;
  }

// These allow you to set the color component order to match your LEDs
// If you set this it is best it in the Project C++ Symbols '-D' section
// e.g RGB_LED_ORDER=123
// The value is the 'index' of the R, G & B colors, one based.
#ifndef RGB_LED_ORDER
#define RGB_LED_ORDER 213
#endif

#ifndef RGBW_LED_ORDER
#define RGBW_LED_ORDER 2134
#endif

struct RGB;
struct RGBW;

// These are configured for WS2812 leds.  For APA 102 it fetches
// them in the correct order for them.
struct RGB {
  RGB() { red = grn = blu = 0; }
  RGB(u8_t v) { red = grn = blu = v; }
  RGB(u8_t r, u8_t g, u8_t b) { red = r; grn = g; blu = b; }
  inline RGB(const RGBW& rgbw);

  // We double count green since it's much more important than R/B for
  // perceived brightness and that makes the rescale a shift (since it's
  // divided by 4 not 3).
  u8_t average() { return (red + (grn << 1) + blu) >> 2; }

  RGB& operator+=(const RGB& rhs) {
    red = addsat(red, rhs.red);
    grn = addsat(grn, rhs.grn);
    blu = addsat(blu, rhs.blu);
    return *this;
  }
#if   RGB_LED_ORDER == 123
  u8_t red, grn, blu;
#elif RGB_LED_ORDER == 132
  u8_t red, blu, grn;
#elif RGB_LED_ORDER == 213
  u8_t grn, red, blu;
#elif RGB_LED_ORDER == 231
  u8_t grn, blu, red;
#elif RGB_LED_ORDER == 312
  u8_t blu, red, grn;
#elif RGB_LED_ORDER == 321
  u8_t blu, grn, red;
#endif
};

struct RGBW {
  RGBW() { red = grn = blu = wht = 0; }
  RGBW(u8_t w) { red = grn = blu = 0; wht = w; }
  RGBW(u8_t rgb, u8_t w) { red = grn = blu = rgb; wht = w; }
  RGBW(u8_t r, u8_t g, u8_t b) { red = r; grn = g; blu = b; wht = 0; }
  RGBW(u8_t r, u8_t g, u8_t b, u8_t w) { red = r; grn = g; blu = b; wht = w; }
  RGBW(const RGB& rgb) { red = rgb.red; grn = rgb.grn; blu = rgb.blu; wht = 0; }
  RGBW(const RGB& rgb, u8_t w) { red = rgb.red; grn = rgb.grn; blu = rgb.blu; wht = w; }

  u8_t average() {
    return (red + (grn << 1) + blu + (wht << 2) + (1 << 2)) >> 3;
  }

  RGBW& operator+=(const RGBW& rhs) {
    red = addsat(red, rhs.red);
    grn = addsat(grn, rhs.grn);
    blu = addsat(blu, rhs.blu);
    wht = addsat(wht, rhs.wht);
    return *this;
  }

#if   RGBW_LED_ORDER == 1234
  u8_t red, grn, blu, wht;
#elif RGBW_LED_ORDER == 1324
  u8_t red, blu, grn, wht;
#elif RGBW_LED_ORDER == 2134
  u8_t grn, red, blu, wht;
#elif RGBW_LED_ORDER == 2314
  u8_t grn, blu, red, wht;
#elif RGBW_LED_ORDER == 3124
  u8_t blu, red, grn, wht;
#elif RGBW_LED_ORDER == 3214
  u8_t blu, grn, red, wht;
#elif RGBW_LED_ORDER == 1243
  u8_t red, grn, wht, blu;
#elif RGBW_LED_ORDER == 1342
  u8_t red, blu, wht, grn;
#elif RGBW_LED_ORDER == 2143
  u8_t grn, red, wht, blu;
#elif RGBW_LED_ORDER == 2341
  u8_t grn, blu, wht, red;
#elif RGBW_LED_ORDER == 3142
  u8_t blu, red, wht, grn;
#elif RGBW_LED_ORDER == 3241
  u8_t blu, grn, wht, red;
#elif RGBW_LED_ORDER == 1423
  u8_t red, wht, grn, blu;
#elif RGBW_LED_ORDER == 1432
  u8_t red, wht, blu, grn;
#elif RGBW_LED_ORDER == 2413
  u8_t grn, wht, red, blu;
#elif RGBW_LED_ORDER == 2431
  u8_t grn, wht, blu, red;
#elif RGBW_LED_ORDER == 3412
  u8_t blu, wht, red, grn;
#elif RGBW_LED_ORDER == 3421
  u8_t blu, wht, grn, red;
#elif RGBW_LED_ORDER == 4123
  u8_t wht, red, grn, blu;
#elif RGBW_LED_ORDER == 4132
  u8_t wht, red, blu, grn;
#elif RGBW_LED_ORDER == 4213
  u8_t wht, grn, red, blu;
#elif RGBW_LED_ORDER == 4231
  u8_t wht, grn, blu, red;
#elif RGBW_LED_ORDER == 4312
  u8_t wht, blu, red, grn;
#elif RGBW_LED_ORDER == 4321
  u8_t wht, blu, grn, red;
#endif
};

  inline RGB operator+(RGB lhs, const RGB& rhs) {
    lhs.red = addsat(lhs.red, rhs.red);
    lhs.grn = addsat(lhs.grn, rhs.grn);
    lhs.blu = addsat(lhs.blu, rhs.blu);
    return lhs;
  }
  inline RGBW operator+(RGB lhs, const RGBW& rhs) {
    return RGBW(addsat(lhs.red, rhs.red),
                addsat(lhs.grn, rhs.grn),
                addsat(lhs.blu, rhs.blu),
                rhs.wht);
  }
  inline RGBW operator+(RGBW lhs, const RGB& rhs) {
    lhs.red = addsat(lhs.red, rhs.red);
    lhs.grn = addsat(lhs.grn, rhs.grn);
    lhs.blu = addsat(lhs.blu, rhs.blu);
    return lhs;
  }
  inline RGBW operator+(RGBW lhs, const RGBW& rhs) {
    lhs.red = addsat(lhs.red, rhs.red);
    lhs.grn = addsat(lhs.grn, rhs.grn);
    lhs.blu = addsat(lhs.blu, rhs.blu);
    lhs.wht = addsat(lhs.wht, rhs.wht);
    return lhs;
  }

  RGB::RGB(const RGBW& rgbw) {
    red = rgbw.red + rgbw.wht;
    grn = rgbw.grn + rgbw.wht;
    blu = rgbw.blu + rgbw.wht;
  }
}  // namespace led

#endif  // _LED_RGB_
