#ifndef _LED_RGB_
#define _LED_RGB_

#include "../IntTypes.h"

namespace led {
  // These are configured for WS2812 leds.  For APA 102 it fetches them in the correct order for them.
struct RGB {
  RGB() : grn(0), red(0), blu(0) {}
  RGB(u8_t v) : grn(v), red(v), blu(v) {}
  RGB(u8_t r, u8_t g, u8_t b) : grn(g), red(r), blu(b) {}
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
  u8_t grn;
  u8_t red;
  u8_t blu;
  u8_t wht;  // white
};

}  // namespace led

#endif  // _LED_RGB_
