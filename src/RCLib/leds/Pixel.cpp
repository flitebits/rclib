#include "pixel.h"
namespace led {

namespace {
  RGB hues[7] = {
    RGB(0xFF, 0x00, 0x00),
    RGB(0xFF, 0xFF, 0x00),
    RGB(0x00, 0xFF, 0x00),
    RGB(0x00, 0xFF, 0xFF),
    RGB(0x00, 0x00, 0xFF),
    RGB(0xFF, 0x00, 0xFF),
    RGB(0xFF, 0x00, 0x00),
  };
}  // namespace

void Fade(RGB* pix, u8_t amount) {
  if (amount == 0xFF) return;
  amount++;
  pix->red = (amount * (int)pix->red) >> 8;
  pix->grn = (amount * (int)pix->grn) >> 8;
  pix->blu = (amount * (int)pix->blu) >> 8;
}
void Fade(RGBW* pix, u8_t amount) {
  if (amount == 0xFF) return;
  amount++;
  pix->red = (amount * (int)pix->red) >> 8;
  pix->grn = (amount * (int)pix->grn) >> 8;
  pix->blu = (amount * (int)pix->blu) >> 8;
  pix->wht = (amount * (int)pix->wht) >> 8;
}

void Lighten(RGB* pix, u8_t amount) {
  Blend(pix, clr::white, amount);
}
void Lighten(RGBW* pix, u8_t amount) {
  Blend(pix, wclr::white, amount);
}

void Blend(RGB* dst, const RGB& other, u8_t amount) {
  long scale = amount + 1;
  dst->red += (((int)other.red - dst->red) * scale) >> 8;
  dst->grn += (((int)other.grn - dst->grn) * scale) >> 8;
  dst->blu += (((int)other.blu - dst->blu) * scale) >> 8;

}
void Blend(RGBW* dst, const RGBW& other, u8_t amount) {
  long scale = amount + 1;
  dst->red += (((int)other.red - dst->red) * scale) >> 8;
  dst->grn += (((int)other.grn - dst->grn) * scale) >> 8;
  dst->blu += (((int)other.blu - dst->blu) * scale) >> 8;
  dst->wht += (((int)other.wht - dst->wht) * scale) >> 8;
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
  result.red = lo.red + (((hi.red - int(lo.red)) * frac) >> 8);
  result.grn = lo.grn + (((hi.grn - int(lo.grn)) * frac) >> 8);
  result.blu = lo.blu + (((hi.blu - int(lo.blu)) * frac) >> 8);
  return result;
}

RGB Lookup9(const RGB* gradient, u8_t offeset) {
  const int idx = offeset >> 5;
  const RGB& lo = gradient[idx];
  const RGB& hi = gradient[idx + 1];
  const int frac = ((offeset & 0x1F) << 3) + 1;
  RGB result;
  result.red = lo.red + (((hi.red - int(lo.red)) * frac) >> 8);
  result.grn = lo.grn + (((hi.grn - int(lo.grn)) * frac) >> 8);
  result.blu = lo.blu + (((hi.blu - int(lo.blu)) * frac) >> 8);
  return result;
}

RGB HsvToRgb(const HSV& hsv) {
  int hue_fp = hsv.hue * 6;  // 8bit FP 0->6

  int hue_idx = hue_fp >> 8;
  const RGB& lo = hues[hue_idx];
  const RGB& hi = hues[hue_idx + 1];

  int frac = (hue_fp & 0xFF);
  RGB result;
  result.red = lo.red + (((hi.red - int(lo.red)) * frac) >> 8);
  result.grn = lo.grn + (((hi.grn - int(lo.grn)) * frac) >> 8);
  result.blu = lo.blu + (((hi.blu - int(lo.blu)) * frac) >> 8);

  volatile u8_t scale = (hsv.sat * (int(hsv.val) + 1)) >> 8;
  const u8_t grey = hsv.val - scale;
  result.red = ((result.red * scale) >> 8) + grey;
  result.grn = ((result.grn * scale) >> 8) + grey;
  result.blu = ((result.blu * scale) >> 8) + grey;
  return result;
}

RGBW HsvToRgbw(const HSV& hsv) {
  int hue_fp = hsv.hue * 6;  // 8bit FP 0->6

  int hue_idx = hue_fp >> 8;
  const RGB& lo = hues[hue_idx];
  const RGB& hi = hues[hue_idx + 1];

  int frac = (hue_fp & 0xFF);
  RGBW result;
  result.red = lo.red + (((hi.red - int(lo.red)) * frac) >> 8);
  result.grn = lo.grn + (((hi.grn - int(lo.grn)) * frac) >> 8);
  result.blu = lo.blu + (((hi.blu - int(lo.blu)) * frac) >> 8);

  volatile u8_t scale = (hsv.sat * (int(hsv.val) + 1)) >> 8;
  u8_t grey = hsv.val - scale;
  result.red = ((result.red * scale) >> 8) + grey;
  result.grn = ((result.grn * scale) >> 8) + grey;
  result.blu = ((result.blu * scale) >> 8) + grey;
  result.wht = 0x00;
  return result;
}
  
}  // namespace led
