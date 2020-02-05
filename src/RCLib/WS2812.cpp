// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include "WS2812.h"
#include <util/atomic.h>

#define nop() __asm__ __volatile__ ( "nop\n" )

// The exact number of nops here has been calibrated with an oscilloscope
#define SEND_0(port, mask, skip) do {           \
    port->OUTSET = mask;                        \
    port->OUTCLR = mask;                        \
    if (skip < 4) nop();                        \
    if (skip < 3) nop();                        \
    if (skip < 2) nop();                        \
    if (skip < 1) nop();                        \
  } while(false)

#define SEND_1(port, mask, skip) do {           \
    port->OUTSET = mask;                        \
    nop();                                      \
    nop();                                      \
    port->OUTCLR = mask;                        \
    if (skip < 2) nop();                        \
    if (skip < 1) nop();                        \
  } while(false)


namespace {
  void SendWS2812Scale(PinId pin, void* ptr, u16_t len, u8_t scale) __attribute__((optimize("O1")));
  void SendWS2812Full(PinId pin, void* ptr, u16_t len) __attribute__((optimize("O1")));
  
  void SendWS2812Full(PinId pin, void* ptr, u16_t len) {
    PORT_t *const port = pin.port_ptr();
    const u8_t mask = 1 << pin.pin();
    u8_t *p = (u8_t *)ptr;
    u8_t b1 = *(p++);
    while(len--) {
      ATOMIC_BLOCK(ATOMIC_FORCEON) {
        const u8_t b = b1;
        if (b & 0x80) SEND_1(port, mask, 2);
        else SEND_0(port, mask, 2);
        b1 = *(p++);
        if (b & 0x40) SEND_1(port, mask, 0);
        else SEND_0(port, mask, 0);
        if (b & 0x20) SEND_1(port, mask, 0);
        else SEND_0(port, mask, 0); 
        if (b & 0x10) SEND_1(port, mask, 0);
        else SEND_0(port, mask, 0);
        if (b & 0x08) SEND_1(port, mask, 0);
        else SEND_0(port, mask, 0);
        if (b & 0x04) SEND_1(port, mask, 0);
        else SEND_0(port, mask, 0);
        if (b & 0x02) SEND_1(port, mask, 0);
        else SEND_0(port, mask, 0);
        if (b & 0x01) SEND_1(port, mask, 5);
        else SEND_0(port, mask, 5);
      }
    }
  }

  void SendWS2812Scale(PinId pin, void* ptr, u16_t len, u8_t scale) {
    PORT_t *const port = pin.port_ptr();
    const u8_t mask = 1 << pin.pin();
    const u8_t scale_1 = scale + 1;
    u8_t *p = (u8_t *)ptr;
    int b1 = *(p++) * scale_1;
    while(len--) {
      ATOMIC_BLOCK(ATOMIC_FORCEON) {
        const u8_t b = b1 >> 8;
        if (b & 0x80) SEND_1(port, mask, 0);
        else SEND_0(port, mask, 0);
        if (b & 0x40) SEND_1(port, mask, 2);
        else SEND_0(port, mask, 2);

        const u8_t nb = *(p++);

        if (b & 0x20) SEND_1(port, mask, 0);
        else SEND_0(port, mask, 0); 
        if (b & 0x10) SEND_1(port, mask, 4);
        else SEND_0(port, mask, 4);

        b1 = nb * scale_1;

        if (b & 0x08) SEND_1(port, mask, 0);
        else SEND_0(port, mask, 0);
        if (b & 0x04) SEND_1(port, mask, 0);
        else SEND_0(port, mask, 0);
        if (b & 0x02) SEND_1(port, mask, 0);
        else SEND_0(port, mask, 0);
        if (b & 0x01) SEND_1(port, mask, 5);
        else SEND_0(port, mask, 5);
      }
    }
  }
}  // anonymous namespace

void SendWS2812(PinId pin, void* ptr, u16_t len, u8_t scale) {
  if (scale == 255) SendWS2812Full(pin, ptr, len);
  else              SendWS2812Scale(pin, ptr, len, scale);
}
