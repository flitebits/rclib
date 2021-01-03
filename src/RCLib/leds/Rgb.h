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

struct RGB;
struct RGBW;
  
  // These are configured for WS2812 leds.  For APA 102 it fetches
  // them in the correct order for them.
struct RGB {
  RGB() : grn(0), red(0), blu(0) {}
  RGB(u8_t v) : grn(v), red(v), blu(v) {}
  RGB(u8_t r, u8_t g, u8_t b) : grn(g), red(r), blu(b) {}

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
  u8_t grn;
  u8_t red;
  u8_t blu;
};

struct RGBW {
  RGBW() : grn(0), red(0), blu(0), wht(0) {}
  RGBW(u8_t w) : grn(0), red(0), blu(0), wht(w) {}
  RGBW(u8_t rgb, u8_t w) : grn(rgb), red(rgb), blu(rgb), wht(w) {}
  RGBW(u8_t r, u8_t g, u8_t b) : grn(g), red(r), blu(b), wht(0) {}
  RGBW(u8_t r, u8_t g, u8_t b, u8_t w) : grn(g), red(r), blu(b), wht(w) {}
  RGBW(const RGB& rgb) : grn(rgb.grn), red(rgb.red), blu(rgb.blu), wht(0) {}
  RGBW(const RGB& rgb, u8_t w) : grn(rgb.grn), red(rgb.red), blu(rgb.blu), wht(w) {}

  u8_t average() {
    return (((red + (grn << 1) + blu) >> 2) + wht) >> 1;
  }
    
  RGBW& operator+=(const RGBW& rhs) {
    red = addsat(red, rhs.red);
    grn = addsat(grn, rhs.grn);
    blu = addsat(blu, rhs.blu);
    wht = addsat(wht, rhs.wht);
    return *this;
  }

  u8_t grn;
  u8_t red;
  u8_t blu;
  u8_t wht;  // white
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
}  // namespace led

#endif  // _LED_RGB_
