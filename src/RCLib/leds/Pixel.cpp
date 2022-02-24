// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include "Pixel.h"
#include "FPMath.h"

namespace led {

namespace {
  RGB hues[7] = {
    RGB(0xFF, 0x00, 0x00),  // Red
    RGB(0xFF, 0xFF, 0x00),  // Yellow
    RGB(0x00, 0xFF, 0x00),  // Green
    RGB(0x00, 0xFF, 0xFF),  // Cyan
    RGB(0x00, 0x00, 0xFF),  // Blue
    RGB(0xFF, 0x00, 0xFF),  // Purple
    RGB(0xFF, 0x00, 0x00),  // Red again
  };
}  // namespace

void Fade(RGB* pix, u8_t amount) {
  if (amount == 0xFF) return;
  amount++;
  pix->red = scale8(amount, pix->red);
  pix->grn = scale8(amount, pix->grn);
  pix->blu = scale8(amount, pix->blu);
}
void Fade(RGBW* pix, u8_t amount) {
  if (amount == 0xFF) return;
  amount++;
  pix->red = scale8(amount, pix->red);
  pix->grn = scale8(amount, pix->grn);
  pix->blu = scale8(amount, pix->blu);
  pix->wht = scale8(amount, pix->wht);
}
void Fade(RGB* pix, int num_pix, u8_t amount) {
  if (amount == 0xFF) return;
  amount++;
  while (num_pix > 0) {
    pix->red = scale8(amount, pix->red);
    pix->grn = scale8(amount, pix->grn);
    pix->blu = scale8(amount, pix->blu);
    ++pix;
    --num_pix;
  }
}
void Fade(RGBW* pix, int num_pix, u8_t amount) {
  if (amount == 0xFF) return;
  amount++;
  while (num_pix > 0) {
    pix->red = scale8(amount, pix->red);
    pix->grn = scale8(amount, pix->grn);
    pix->blu = scale8(amount, pix->blu);
    pix->wht = scale8(amount, pix->wht);
    ++pix;
    --num_pix;
  }
}

void Lighten(RGB* pix, u8_t amount) {
  Blend(pix, clr::white, amount);
}
void Lighten(RGBW* pix, u8_t amount) {
  Blend(pix, wclr::white, amount);
}

void Blend(RGB* dst, const RGB& other, u8_t amount) {
  long scale = amount + 1;
  dst->red += (((i16_t)other.red - dst->red) * scale) >> 8;
  dst->grn += (((i16_t)other.grn - dst->grn) * scale) >> 8;
  dst->blu += (((i16_t)other.blu - dst->blu) * scale) >> 8;

}
void Blend(RGBW* dst, const RGBW& other, u8_t amount) {
  long scale = amount + 1;
  dst->red += (((i16_t)other.red - dst->red) * scale) >> 8;
  dst->grn += (((i16_t)other.grn - dst->grn) * scale) >> 8;
  dst->blu += (((i16_t)other.blu - dst->blu) * scale) >> 8;
  dst->wht += (((i16_t)other.wht - dst->wht) * scale) >> 8;
}

void Fill(RGB* pixs, int num_pix, const RGB& val) {
  while (num_pix--) {
    *(pixs++) = val;
  }
}
void Fill(RGBW* pixs, int num_pix, const RGBW& val){
  while (num_pix--) {
    *(pixs++) = val;
  }
}  

RGB Lookup5(const RGB* gradient, u8_t offset) {
  const int idx = offset >> 6;
  const int frac = ((offset & 0x3F) << 2);
  const RGB& lo = gradient[idx];
  const RGB& hi = gradient[idx + 1];
  RGB result;
  result.red = lo.red + (((hi.red - i16_t(lo.red)) * frac) >> 8);
  result.grn = lo.grn + (((hi.grn - i16_t(lo.grn)) * frac) >> 8);
  result.blu = lo.blu + (((hi.blu - i16_t(lo.blu)) * frac) >> 8);
  return result;
}

RGB Lookup9(const RGB* gradient, u8_t offeset) {
  const int idx = offeset >> 5;
  const RGB& lo = gradient[idx];
  const RGB& hi = gradient[idx + 1];
  const int frac = ((offeset & 0x1F) << 3) + 1;
  RGB result;
  result.red = lo.red + (((hi.red - i16_t(lo.red)) * frac) >> 8);
  result.grn = lo.grn + (((hi.grn - i16_t(lo.grn)) * frac) >> 8);
  result.blu = lo.blu + (((hi.blu - i16_t(lo.blu)) * frac) >> 8);
  return result;
}

RGBW Lookup5(const RGBW* gradient, u8_t offset) {
  const int idx = offset >> 6;
  const int frac = ((offset & 0x3F) << 2);
  const RGBW& lo = gradient[idx];
  const RGBW& hi = gradient[idx + 1];
  RGBW result;
  result.red = lo.red + (((hi.red - i16_t(lo.red)) * frac) >> 8);
  result.grn = lo.grn + (((hi.grn - i16_t(lo.grn)) * frac) >> 8);
  result.blu = lo.blu + (((hi.blu - i16_t(lo.blu)) * frac) >> 8);
  result.wht = lo.wht + (((hi.wht - i16_t(lo.wht)) * frac) >> 8);
  return result;
}

RGBW Lookup9(const RGBW* gradient, u8_t offeset) {
  const int idx = offeset >> 5;
  const RGBW& lo = gradient[idx];
  const RGBW& hi = gradient[idx + 1];
  const int frac = ((offeset & 0x1F) << 3) + 1;
  RGBW result;
  result.red = lo.red + (((hi.red - i16_t(lo.red)) * frac) >> 8);
  result.grn = lo.grn + (((hi.grn - i16_t(lo.grn)) * frac) >> 8);
  result.blu = lo.blu + (((hi.blu - i16_t(lo.blu)) * frac) >> 8);
  result.wht = lo.wht + (((hi.wht - i16_t(lo.wht)) * frac) >> 8);
  return result;
}
  
RGB HsvToRgb(const HSV& hsv) {
  int hue_fp = hsv.hue * 6;  // 8bit FP 0->6

  int hue_idx = hue_fp >> 8;
  const RGB& lo = hues[hue_idx];
  const RGB& hi = hues[hue_idx + 1];

  int frac = (hue_fp & 0xFF);
  RGB result;
  result.red = lo.red + (((hi.red - i16_t(lo.red)) * frac) >> 8);
  result.grn = lo.grn + (((hi.grn - i16_t(lo.grn)) * frac) >> 8);
  result.blu = lo.blu + (((hi.blu - i16_t(lo.blu)) * frac) >> 8);

  volatile u8_t scale = (hsv.sat * (int(hsv.val) + 1)) >> 8;
  const u8_t grey = hsv.val - scale;
  result.red = scale8(result.red, scale) + grey;
  result.grn = scale8(result.grn, scale) + grey;
  result.blu = scale8(result.blu, scale) + grey;
  return result;
}

RGBW HsvToRgbw(const HSV& hsv) {
  int hue_fp = hsv.hue * 6;  // 8bit FP 0->6

  int hue_idx = hue_fp >> 8;
  const RGB& lo = hues[hue_idx];
  const RGB& hi = hues[hue_idx + 1];

  int frac = (hue_fp & 0xFF);
  RGBW result;
  result.red = lo.red + (((hi.red - i16_t(lo.red)) * frac) >> 8);
  result.grn = lo.grn + (((hi.grn - i16_t(lo.grn)) * frac) >> 8);
  result.blu = lo.blu + (((hi.blu - i16_t(lo.blu)) * frac) >> 8);

  volatile u8_t scale = (hsv.sat * (int(hsv.val) + 1)) >> 8;
  u8_t grey = hsv.val - scale;
  result.red = scale8(result.red, scale) + grey;
  result.grn = scale8(result.grn, scale) + grey;
  result.blu = scale8(result.blu, scale) + grey;
  result.wht = 0x00;
  return result;
}

void Max(RGB* pix, const RGB& other) {
  if (pix->red < other.red) pix->red = other.red;
  if (pix->grn < other.grn) pix->grn = other.grn;
  if (pix->blu < other.blu) pix->blu = other.blu;
}
void Max(RGBW* pix, const RGBW& other){
  if (pix->red < other.red) pix->red = other.red;
  if (pix->grn < other.grn) pix->grn = other.grn;
  if (pix->blu < other.blu) pix->blu = other.blu;
  if (pix->wht < other.wht) pix->wht = other.wht;
}
  
}  // namespace led
