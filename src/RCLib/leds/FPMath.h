// Copyright 2020,2021 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _LED_FPMATH_
#define _LED_FPMATH_

#include "../IntTypes.h"
namespace led {
  
inline u8_t scale8(u8_t v0, u8_t v1) {
  u8_t out;
  __asm__ __volatile__ (		     \
			"mul %[v0], %[v1]\n\t" \
			"mov %[out], r1\n\t"   \
			"clr __zero_reg__  \n\t" \
			: [out] "=r" (out)
			: [v0] "r" (v0),
			  [v1] "r" (v1));
  return out;
}

inline u16_t scale16(u16_t v0, u16_t v1) {
  u16_t out;
  __asm__ __volatile__ (		       \
			/* multiply high bytes and make output word */ \
			"mul %B1, %B2    \n\t" \
			"movw %[out], r0 \n\t" \
			/* multiply low bytes only keep high byte for */ \
			/* accumulation purposes to catch carries */ \
			"mul %A1, %A2    \n\t" \
			"mov r24, r1     \n\t" \
			/* multiply hi-lo bytes, add low byte to */ \
			/* accumulator byte and high byte to low byte of */ \
			/* result, with carry, the add any carry from that */ \
			/* to the high result byte */ \
			"mul %B1, %A2    \n\t" \
			"clr r25         \n\t" \
			"add r24, r0     \n\t" \
			"adc %A0, r1     \n\t" \
			"adc %B0, r25    \n\t" \
			/* repeat with lo-hi result */
			"mul %A1, %B2    \n\t" \
			"add r24, r0     \n\t" \
			"adc %A0, r1     \n\t" \
			"adc %B0, r25    \n\t" \
			/* clear zero reg that we used */ \
			"clr __zero_reg__  \n\t" \
			: [out] "=r" (out)
			: "a" (v0),
			  "a" (v1)
                        : "r24", "r25");
  return out;
}

// Returns approximation of sin.  The input angle treates 256 as 360
// deg/2*pi.  The result is +/-127 with a bias of 128.
 u8_t sin8(u8_t angle);
// Returns approximation of sin.  The input angle treates 256 as 360
// deg/2*pi, the result is in the range of +/-32767.
 i16_t sin816(u8_t angle);
// Returns approximation of sin.  The input angle treates 65536 as 360
// deg/2*pi, the result is in the range of +/-32767.
 i16_t sin16(u16_t angle);


 // Returns a value that goes from 0->65535 at the frequency specified
 // by bmp88 (saw wave).  bmp88 is a 8 bit fixed point number of beats
 // per minute so for 120bmp, pass in 120 * 256 (or a bit more
 // idomatically 120 << 8).  This cheats a bit in that it is actually
 // beats/64sec (Avoids a bunch of expensive multiplies, if you care
 // you can correct for this in your bpm by multiplying by 64 / 60 -
 // try to do that once not in loops).
 u16_t saw16(u16_t bpm88);
 // Same as saw16 but only integral BPMs supported.
 u16_t saw8(u8_t bpm);

 // Returns an 8bit sin wave that has a frequence of bpm8. Uses FastTimeNow
 // to get current time, and 1 min = 64sec.
 u8_t bpm8(u8_t bpm8);
 // Returns a sin wave going from lo->hi that has a frequence of
 // bpm8.  Uses FastTimeNow to get current time, and 1 min = 64sec.
 u8_t bpm8Ranged(u8_t bpm8, u8_t lo, u8_t hi);
 // Returns a sin wave going from lo->hi that has a frequence of
 // bpm88.  Uses FastTimeNow to get current time, and 1 min = 64sec.
 u8_t bpm888Ranged(u16_t bpm88, u8_t lo, u8_t hi);

 // Returns a 16bit sin wave with a frequency of bmp88. Uses
 // FastTimeNow to get current time, and 1 min = 64sec.
 i16_t bpm16(u16_t bpm88);
 // Returns a sin wave going from lo->hi with a frequency of
 // bmp88. Uses FastTimeNow to get current time, and 1 min = 64sec.
 i16_t bpm16Ranged(u16_t bpm88, i16_t lo, i16_t hi);
 
}  // namespace led
#endif  // _LED_FPMATH_
