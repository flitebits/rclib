// Copyright 2020,2021 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include "FPMath.h"

#include "../RtcTime.h"

namespace led {
namespace {
  struct PLin8{
    u8_t base;
    u8_t slope;
  };
  // base is 127 * sin(x), for x = 0, 22.5, 45, 67.5 degrees.
  // Slope is just the delta between base values (makes interpolation faster).
  const PLin8 sin8_data[] = { {0, 49}, {49, 41}, {90, 27}, {117, 10}, {127, 0}};

  struct PLin16{
    u16_t base; // base
    u8_t slope_hi; // slope hi 8bits
    u8_t slope_lo; // slope lo 8bits
  };

  const PLin16 sin16_data[] =
    {{0, 24, 249}, {6393, 24, 2}, {12539, 23, 132}, {18559, 18, 3},
     {23170, 15, 235}, {27245, 13, 18}, {30591, 6, 191}, {32318, 1, 193},
     {32767, 0, 0}};
}  // anonymous namespace
  
u8_t sin8(u8_t theta) {
  u8_t qt = theta & 0x3F;  // theta withing it's quarter
  if (theta & 0x40) {  // odd quarters flip horizontally.
    qt = (1 << 6) - qt;
  }
  const PLin8& v = sin8_data[qt >> 4];
  // use the low 4 bits as 
  u8_t result = v.base + ((v.slope * (qt & 0x0F)) >> 4);
  if (theta & 0x80) {
    return 128 - result;
  }
  return 128 + result;
}

i16_t sin16(u16_t theta) {
  // Starts with 14 bits
  u16_t qt = theta & 0x3FFF;  // theta within it's quarter
  qt = qt >> 3;  // Now has 11 bits
  if (theta & 0x4000) {  // odd quarters flip horizontally.
    qt = (1 << 11) - qt;
  }
  const PLin16& v = sin16_data[qt >> 8];  // use high 3 bits to lookup segment
  u8_t frac = qt & 0xFF;
  // use the low 8 bits as interpolent, we do the multiply 'by hand'
  // so we can skip the not important parts.
  i16_t result = v.base + v.slope_hi * frac + scale8(v.slope_lo, frac);
  if (theta & 0x8000) {
    return -result;
  }
  return result;
}

i16_t sin816(u8_t theta) {
  u16_t qt = theta & 0x3F;  // theta within it's quarter
  if (theta & 0x40) {  // odd quarters flip horizontally.
    qt = (1 << 6) - qt;
  }
  const PLin16& v = sin16_data[qt >> 3];
  u8_t frac = (qt & 0x07) << 5;
  // use the low 3 bits as interpolent, we do the multiply 'by hand'
  // so we can skip the not important parts.
  i16_t result = v.base + v.slope_hi * frac + ((v.slope_lo * frac) >> 8);
  if (theta & 0x80) {
    return -result;
  }
  return result;
}

u16_t saw16(u16_t bpm88) {
  // We want the result to go from 0 -> 65536 bpm88 times every 60sec.
  // We are cheating so we are actually using 65536 ticks from FastTimeMS
  // as our 'minute' (really 64 sec since ticks are 1024th of sec).
  // This means we can just multiply bmp88 with FastTimeMS and shift 8 bits out.
  return (FastTimeMs() * bpm88) >> 8;
}
u16_t saw8(u8_t bpm8) {
  // Same as above but just integral bpm.
  u16_t ms = FastTimeMs();  // we only care about low 16 bits.
  return ms * bpm8;
}
 
u8_t bpm8(u8_t bpm8) {
  const u16_t frac = saw8(bpm8);
  return sin8(frac >> 8);
}

u8_t bpm8Ranged(u8_t bpm8, u8_t lo, u8_t hi) {
  const u16_t frac = saw8(bpm8);
  return scale8(sin8(frac >> 8), hi - lo) + lo;
}

u8_t bpm888Ranged(u16_t bpm88, u8_t lo, u8_t hi) {
  const u16_t frac = saw16(bpm88);
  return scale8(sin8(frac >> 8), u8_t(hi - lo)) + lo;
}

  i16_t bpm16(u16_t bpm88) {
  const u16_t frac = saw16(bpm88);
  return sin16(frac);
}
i16_t bpm16Ranged(u16_t bpm88, i16_t lo, i16_t hi) {
  const u16_t frac = saw16(bpm88);
  return (((sin16(frac) + 32767) * u32_t(hi - lo)) >> 16) + lo;
}

}
