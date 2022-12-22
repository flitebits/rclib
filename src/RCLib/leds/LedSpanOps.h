// Copyright 2022 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
#include "../IntTypes.h"
#include "leds/Pixel.h"
#include "leds/Rgb.h"

namespace led {

class FnFilled {
public:
  typedef u8_t (*fn_t)(u8_t);
 FnFilled(fn_t fn, const RGBW& color, u8_t scale, u8_t offset = 0) :
  fn_(fn), scale_(scale), color_(color), offset_(offset) { }
  void SetOffset(u8_t offset) { offset_ = offset; }
  u8_t GetOffset() { return offset_; }

  void operator() (u8_t frac, RGBW* pix) const {
    frac = (u16_t(frac) * scale_) >> 6;
    *pix = color_;
    Fade(pix, (*fn_)(offset_ + frac));
  }
private:
  fn_t fn_;
  const u8_t scale_;
  const RGBW color_;
  u8_t offset_;
};

u8_t log_sin8(u8_t v) { return  Logify(sin8(v)); }
FnFilled MakeSinFilled(const RGBW& color, u8_t scale, u8_t offset = 0) {
  return FnFilled(&log_sin8, color, scale, offset);
}

}  // namespace led
