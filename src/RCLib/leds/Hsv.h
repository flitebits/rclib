// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _LED_HSV_
#define _LED_HSV_

#include "../IntTypes.h"

namespace led {
struct HSV {
  HSV() : hue(0), sat(0), val(0) {}
  HSV(u8_t v) : hue(0), sat(0), val(v) {}
  HSV(u8_t h, u8_t v) : hue(h), sat(0xFF), val(v) {}
  HSV(u8_t h, u8_t s, u8_t v) : hue(h), sat(s), val(v) {}
  u8_t hue;
  u8_t sat;
  u8_t val;
};
}  // namespace led

#endif  // _LED_HSV_
