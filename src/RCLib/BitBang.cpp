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

#define nop() __asm__ __volatile__ ( "nop\n" )

namespace {
}
BitBang::BitBang()
  : port_(NULL),
    ws2812_in_progress(false),
    dshot_data(NULL) { }

BitBang::BitBang(PinGroupId pin_group, u8_t pins)
  : port_(NULL),
    ws2812_in_progress(false),
    dshot_data(NULL) {
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

void BitBang::SendWS2812(PinId pin, void* ptr, u16_t len, u8_t scale) {
  ws2812_in_progress = true;
  SendWS2812Full(pin, ptr, len);
  ws2812_in_progress = false;
}

// DShot zero is .625uS, and one is 1.25uS and bit width is 1.66667uS.
void BitBang::SendDShot600(u8_t* dshot_data, u8_t dshot_mask) {
  if (ws2812_in_progress) {
    this->dshot_mask = dshot_mask;
    this->dshot_data = dshot_data;
    return;
  }
  register8_t* out_set = &port_->OUTSET;
  register8_t* out_clr = &port_->OUTCLR;
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    u8_t i = 0;
    while (true) {
      *out_set = dshot_mask;  // Set all bits high
      nop();
      nop();
      nop();
      nop();
      nop();
      *out_clr = *(dshot_data++);
      nop();
      nop();
      nop();
      nop();
      if (i == 15) {
	*out_clr = dshot_mask;
	break;
      }
      *out_clr = dshot_mask;
      i++;
      nop();
    }
  }
}

// We want ~.3 - .35uS high for zeros, and ~.65-.7uS high for ones with a
// total time of ~1.2uS start to start.
void BitBang::SendWS2812Full(PinId pin, void* ptr, u16_t len) {
  register8_t* out_set = &port_->OUTSET;
  register8_t* out_clr = &port_->OUTCLR;
  const u8_t led_mask = 1 << pin.pin();
  u8_t bit_mask = 0x1;
  u8_t *p = (u8_t *)ptr;
  u8_t b1 = *(p++);
  while(len) {
    if (dshot_data != NULL) {
      u8_t l1 = b1;
      u8_t l2 = 0;
      --len;
      if (len) {
	l2 = *(p++);
	--len;
      }
      // Sends DShot data along with 2 bytes of led data.
      DShot600WS2812(dshot_data, dshot_mask, l1, l2, led_mask);
      if (len) {
	b1 = *(p++);
	--len;
      }
      continue;
    }
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      for (u8_t i = 0; i < 8; ++i) {
	*out_set = led_mask;  // Set led pin high
	nop();
	nop();
	nop();
	if ((b1 & bit_mask) == 0) *out_clr = led_mask;
	nop();
	nop();
	nop();
	nop();
	*out_clr = led_mask;
	if (i == 7) {
	  b1 = *(p++);
	  bit_mask = 0x1;
	  --len;
	} else {
	  bit_mask = bit_mask << 1;
	  nop();
	}
	nop();
	nop();
      }
    }
  }
}

// This sends dshot data along with ws2812 data, the pins must all be
// on the same port.  WS2812 0/1 pulse widths are roughly half that of
// DSHOT600, but total pulse duration is roughly equal.
// So we set all pins high at the start of each bit, then clear WS2812 bit
// for zeros at ~300us, then clear DSHOT zero bits at 625us,
// then clear all bits at 1.25us, start next loop at 1.6667us.
void BitBang::DShot600WS2812(
  u8_t *dshot, u8_t dshot_mask, u8_t led1, u8_t led2, u8_t led_mask) {
  register8_t* out_set = &port_->OUTSET;
  register8_t* out_clr = &port_->OUTCLR;
  u8_t bit_mask = 0x1;
  u8_t pin_mask = dshot_mask | led_mask;
  u8_t led = led1;
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    for (u8_t i = 0; i < 16; ++i) {
      *out_set = pin_mask;  // Set all bits high
      nop();
      nop();
      nop();
      if ((led & bit_mask) == 0) *out_clr = led_mask;
      nop();
      nop();
      nop();
      *out_clr = *(dshot++);
      nop();
      nop();
      nop();
      nop();
      *out_clr = led_mask;
      if (i == 7) {
	led = led2;
	bit_mask = 0x1;
      } else {
	bit_mask = bit_mask << 1;
	nop();
      }
      *out_clr = pin_mask;
      nop();
      nop();
    }
  }
}

