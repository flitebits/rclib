// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _LED_PIXEL_
#define _LED_PIXEL_

#include "../IntTypes.h"
#include "Rgb.h"
#include "Hsv.h"
#include "Clr.h"

namespace led {
// amount = 0, pix set to black, amount = 0xFF no change
void Fade(RGB* pix, u8_t amount);
void Fade(RGBW* pix, u8_t amount);
void Fade(RGB* pix, int num_pix, u8_t amount);
void Fade(RGBW* pix, int num_pix, u8_t amount);
// amount = 0, no change, amount = 0xff sets pix to white.
void Lighten(RGB* pix, u8_t amount);
void Lighten(RGBW* pix, u8_t amount);
// amount = 0 dst unchanged, amount = 0xff dst = other.
void Blend(RGB* dst, const RGB& other, u8_t amount);
void Blend(RGBW* dst, const RGBW& other, u8_t amount);
// Sets all 'num_pix' entries in 'pixs' to val
void Fill(RGB* pixs, int num_pix, const RGB& val);
void Fill(RGBW* pixs, int num_pix, const RGBW& val);

// Lookup gradient value from a 4 entry gradient.
// zero is first element 255 is last element.
RGB Lookup5(const RGB* gradient, u8_t offset);
RGB Lookup9(const RGB* gradient, u8_t offset);

RGB HsvToRgb(const HSV& hsv);
RGBW HsvToRgbw(const HSV& hsv);
  
}  // namespace led
  
#endif  // _LED_PIXEL_
