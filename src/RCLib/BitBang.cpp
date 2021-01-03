// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include "BitBang.h"

#include <stddef.h>
#include <util/atomic.h>

#include "leds/FPMath.h"

// If you use ASM then all the timeing critical stuff is done with asm
// and thus build settings can't affect results. I have rough
// equivilents in straight C/C++ wich you can use if you want to
// debug/understand stuff.
#define USE_ASM 1

#define nop() __asm__ __volatile__ ( "nop\n" )

namespace {
#define DSHOT600_ASM(port, data_ptr, mask)			\
  __asm__ __volatile__ (                                                \
  ".L%=_top: \n\t" \
  /* Set all bits high (OUTSET) */ \
  "std Z+5, %[msk]\n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  /* Clear zeros (OUTCLR) for dshot channels 625uS */ \
  "ld __tmp_reg__, X+ \n\t" \
  "std Z+6, __tmp_reg__ \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "std Z+6, %[msk] \n\t" \
  "subi %[idx],lo8(1) \n\t" \
  "breq .L%=_end\n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "rjmp .L%=_top \n\t" \
  ".L%=_end: \n\t" \
                        : /* no outputs */                              \
                        : "z"  (port),                         \
                          "x"  (data_ptr),                     \
                          [msk]   "r"  (mask),                 \
			  [idx]    "a" (idx));

#define DSHOT600WS2812_ASM(port, data_ptr, all_mask, led_mask, led1, led2)			\
  __asm__ __volatile__ (                                                \
  ".L%=_top: \n\t" \
  /* Set all bits high (OUTSET) */ \
  "std Z+5, %[all_msk]\n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  /* If high bit of led is set skip clearing led pin 300uS */ \
  "sbrs %[led], 7 \n\t" \
  "std Z+6, %[led_msk] \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  /* Clear zeros (OUTCLR) for dshot channels 625uS */ \
  "ld __tmp_reg__, X+ \n\t" \
  "std Z+6, __tmp_reg__ \n\t" \
  /* Clear led bit unconditionally (for ones) ~700us */ \
  "std Z+6, %[led_msk] \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  /* If idx == 9 then switch to led2 byte, else right shift led 1 bit */ \
  "cpi %[idx],lo8(9) \n\t" \
  "breq .L%=_next_byte \n\t" \
  "lsl %[led] \n\t" \
  "rjmp .L%=_clear_all \n\t" \
  ".L%=_next_byte: \n\t" \
  "mov %[led], %[led2] \n\t" \
  "nop \n\t" \
  ".L%=_clear_all: \n\t" \
  "nop \n\t" \
  "std Z+6, %[all_msk]\n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "nop \n\t" \
  "subi %[idx],lo8(1) \n\t" \
  "brne .L%=_top\n\t" \
                        : /* no outputs */                              \
                        : "z"  (port),                         \
                          "x"  (data_ptr),                     \
                          [all_msk]   "r"  (all_mask),                 \
                          [led_msk]   "r"  (led_mask),                 \
			  [idx]    "a" (idx),			\
                          [led]    "r"  (led1),                         \
                          [led2]   "r"  (led2));

#define WS2812Full_ASM(port, led_mask, idx, b1)			\
  __asm__ __volatile__ (                                                \
			".L%=_top: \n\t"				\
			/* Set all bits high (OUTSET) */		\
			"std Z+5, %[led_mask]\n\t"			\
			"nop \n\t"					\
			"nop \n\t"					\
			"nop \n\t"					\
			"nop \n\t"					\
			/* If high bit of led is set skip clearing */	\
			/* led pin 300uS */				\
			"sbrs %[b1], 7 \n\t"				\
			"std Z+6, %[led_mask] \n\t"			\
			/* Shift bits up to get next bit */		\
			"lsl %[b1] \n\t"				\
			"nop \n\t"					\
			"nop \n\t"					\
			"nop \n\t"					\
			"nop \n\t"					\
			"nop \n\t"					\
			/* Clear led bit unconditionally @900us */ 	\
			"std Z+6, %[led_mask] \n\t"			\
                        /* Subtract one from idx and break out if zero */ \
			"subi %[idx],lo8(1) \n\t"			\
			"breq .L%=_end \n\t"				\
			"nop \n\t"					\
			"nop \n\t"					\
			"nop \n\t"					\
			"nop \n\t"					\
			"nop \n\t"					\
			"nop \n\t"					\
			"rjmp .L%=_top \n\t"				\
			".L%=_end: \n\t"				\
			/* Clear led bit unconditionally @900us */	\
			"std Z+6, %[led_mask] \n\t"			\
			: /* no outputs */				\
                        : "z"  (port),					\
			  [b1] "r" (b1),				\
                          [led_mask] "r"  (led_mask),			\
			  [idx]      "a" (idx));
}

BitBang::BitBang()
  : port_(NULL),
    ws2812_in_progress_(false),
    dshot_data_(NULL) { }

BitBang::BitBang(PinGroupId pin_group, u8_t pins)
  : port_(NULL),
    ws2812_in_progress_(false),
    dshot_data_(NULL) {
  SetPort(pin_group, pins);
}

void BitBang::AddPin(u8_t pin) {
  port_->OUTCLR = 1 << pin;  // Set bit low to start
  port_->DIRSET = 1 << pin;  // Set bit as output
}

void BitBang::SetPort(PinGroupId pin_group, u8_t pins) {
  port_ = GetPortStruct(pin_group);
  for (int i=0; i < 8; ++i) {
    if (pins & (1 << i)) PinId(pin_group, i).SetOutput();
  }
}

void BitBang::SendWS2812(u8_t pin, void* ptr, u16_t len, u8_t scale) {
  ws2812_in_progress_ = true;
  if (scale == 255) {
    SendWS2812Full(pin, ptr, len);
  } else {
    SendWS2812Scale(pin, ptr, len, scale);
  }
  ws2812_in_progress_ = false;
}

// DShot zero is .625uS, and one is 1.25uS and bit width is 1.66667uS.
void BitBang::SendDShot600(u8_t* dshot_data, u8_t dshot_mask) {
  if (ws2812_in_progress_) {
    this->dshot_mask_ = dshot_mask;
    this->dshot_data_ = dshot_data;
    return;
  }
  u8_t idx = 16;
  PORT_t* port = port_;
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
#if USE_ASM
    DSHOT600_ASM(port, dshot_data, dshot_mask);
#else
    while (true) {
      port->OUTSET = dshot_mask;  // Set all bits high
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      port->OUTCLR = *(dshot_data++);
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      if (--idx == 0) {
	port->OUTCLR = dshot_mask;
	break;
      }
      port->OUTCLR = dshot_mask;
      nop();
      nop();
      nop();
      nop();
      nop();
    }
#endif
  }
}

// We want ~.3 - .35uS high for zeros, and ~.65-.7uS high for ones with a
// total time of ~1.2uS start to start.
void BitBang::SendWS2812Full(u8_t pin, void* ptr, u16_t len) {
  PORT_t* port = port_;
  const u8_t led_mask = 1 << pin;
  u8_t *p = (u8_t *)ptr;
  u8_t b1 = *(p++);
  while(len) {
    if (dshot_mask_) {
      u8_t l1 = b1;
      u8_t l2 = *(p++);
      b1 = *(p++);
      // Sends DShot data along with 2 bytes of led data.
      DShot600WS2812(l1, l2, led_mask);
      if (len <= 2) return;
      len -= 3;
      continue;
    }
#if USE_ASM
    u8_t idx = 8;
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      WS2812Full_ASM(port, led_mask, idx, b1);
    }
    b1 = *(p++);
#else
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      for (u8_t i = 0; i < 8; ++i) {
	port->OUTSET = led_mask;  // Set led pin high
	nop();
	nop();
	nop();
	if ((b1 & 0x1) == 0) port->OUTCLR = led_mask;
	nop();
	nop();
	nop();
	nop();
	port->OUTCLR = led_mask;
	if (i == 7) {
	  b1 = *(p++);
	} else {
	  b1 = b1 >> 1;
	}
	nop();
	nop();
	nop();
      }
    }
#endif  // USE_ASM
    --len;
  }
}

// We want ~.3 - .35uS high for zeros, and ~.65-.7uS high for ones with a
// total time of ~1.2uS start to start.
void BitBang::SendWS2812Scale(u8_t pin, void* ptr, u16_t len, u8_t scale) {
  PORT_t* port = port_;
  const u8_t led_mask = 1 << pin;
  const u8_t scale_1 = scale + 1;
  u8_t *p = (u8_t *)ptr;
  u8_t b1 = led::scale8(*(p++), scale_1);
  while(len) {
    if (dshot_mask_) {
      u8_t l1 = b1;
      u8_t l2 = led::scale8(*(p++), scale_1);
      b1 = led::scale8(*(p++), scale_1);
      // Sends DShot data along with 2 bytes of led data.
      DShot600WS2812(l1, l2, led_mask);
      if (len <= 2) return;
      len -= 3;
      continue;
    }
#if USE_ASM
    u8_t idx = 8;
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      WS2812Full_ASM(port, led_mask, idx, b1);
    }
    b1 = led::scale8(*(p++), scale_1);
#else
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      for (u8_t i = 0; i < 8; ++i) {
	port->OUTSET = led_mask;  // Set led pin high
	nop();
	nop();
	nop();
	if ((b1 & 0x1) == 0) port->OUTCLR = led_mask;
	nop();
	nop();
	nop();
	nop();
	port->OUTCLR = led_mask;
	if (i == 7) {
	  b1 = led::scale8(*(p++), scale_1);
	} else {
	  b1 = b1 >> 1;
	}
	nop();
	nop();
	nop();
      }
    }
#endif
    --len;
  }
}

// This sends dshot data along with 2 bytes of ws2812 data, the pins
// must all be on the same port.  WS2812 0/1 pulse widths are roughly
// half that of DSHOT600, but total pulse duration is roughly equal.
// So we set all pins high at the start of each bit, then clear WS2812
// led bit for zeros at ~300us, then clear DSHOT zero bits at 625us,
// then clear led bit at 900us and finally all bits at 1.25us, start
// next loop at 1.6667us.
void BitBang::DShot600WS2812(u8_t led1, u8_t led2, u8_t led_mask) {
  u8_t idx = 16;
  PORT_t* port = port_;
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    const u8_t all_mask = dshot_mask_ | led_mask;
    u8_t *dshot = dshot_data_;
#if USE_ASM
    DSHOT600WS2812_ASM(port, dshot, all_mask, led_mask, led1, led2);
#else
    while (idx) {
      port->OUTSET = all_mask;  // Set all pins high (LED + DSHOT)
      nop();
      nop();
      nop();
      nop();
      if ((led1 & 0x1) == 0) {
	port->OUTCLR = led_mask;  // LED pin low for zero bit
      }
      nop();
      nop();
      nop();
      port->OUTCLR = *(dshot++);  // DShot pins low for zeros
      nop();
      nop();
      nop();
      nop();
      port->OUTCLR = led_mask;  // LED pin low for ones (already low for zero)
      if (idx == 9) {  // Prep for next led byte/bit
	led1 = led2;
      } else {
	led1 = led1 >> 1;
      }
      nop();
      port->OUTCLR = all_mask;  // All pins low
      nop();
      nop();
      nop();
      --idx;
    }
#endif // USE_ASM
    // Clear dshot mask so we know it's been handled.
    dshot_mask_ = 0;
  }
}

