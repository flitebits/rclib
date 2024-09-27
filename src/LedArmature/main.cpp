// Copyright 2020 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "Boot.h"
#include "Dbg.h"
#include "IntTypes.h"
#include "RtcTime.h"
#include "Util.h"

#include "KeyScan.h"
#include "leds/Effects.h"
#include "leds/FPMath.h"
#include "leds/Hsv.h"
#include "leds/Pixel.h"
#include "leds/Rgb.h"
#include "Pins.h"
#include "Serial.h"
#include "WS2812.h"
#include "DShotASM.h"

#define BLINK_PIN (PIN_F2)

#if defined(__AVR_ATmega4808__)
#define LED_PIN (PIN_F0)
#endif

#if defined(__AVR_ATmega4809__)
#define LED_PIN (PIN_C2)
#endif

using led::RGB;

const u16_t nLeds = 200;
RGB leds[nLeds];
using led::Fill;

const int start_idx = 194;

u8_t phase_map[8] = {2, 4, 8, 16, 32, 64, 128, 255};

void UpdateLeds(u8_t phase){
  RGB pix(0);
  Fill(leds, nLeds, pix);
  leds[194] = RGB(0xB0, 0x50, 0);
  leds[195] = RGB(0xFF, 0, 0);
  leds[196] = RGB(0xFF, 0, 0);
  leds[197] = RGB(0x40, 0x20, 0);
  leds[198] = RGB(0x40, 0x20, 0);
  leds[199] = RGB(0x40, 0x20, 0);
}

void SetWhite(bool boost){
  // Boost means everything on at full power.
  RGB pix(0xFF);
  Fill(leds, nLeds, pix);
}

class State {
};

struct TimeValues {
  u16_t cycle_ms;
  RGB values[6];
};

int loop_idx = 0;
int loop_buffer = 0;
TimeValues loops[2][20];
TimeValues* loop = loops[loop_buffer];

void FinishTimeValues() {
    loop_idx = 0;
    loop = loops[loop_buffer];
    loop_buffer = loop_buffer ? 0 : 1;
}
u8_t addClip(u8_t v0, u8_t v1) {
  u16_t s = v0;
  s += v1;
  if (s > 255) return 255;
  return s;
}

void ProcessCmd(State& state, char* line) {
  if (*line == 0) {
    FinishTimeValues();
    return;
  }
  TimeValues* tv = &loops[loop_buffer][loop_idx];
  int len = sscanf(line, "%d", &tv->cycle_ms);
  if (len == EOF) {
    FinishTimeValues();
  }
  line += len;
  for (int idx = 0; idx < 6; ++idx) {
    while (*line == ' ') ++line;
    if (*line == 0) break;
    u8_t v[6];
    int i = 0;
    while (true) {
      u8_t ch = *line;
      if (ch >= '0' && ch <= '9') {
        v[i] = ch - '0';
      } else if (ch >= 'A' && ch <= 'F') {
        v[i] = 10 + (ch - 'A');
      } else if (ch >= 'a' && ch <= 'f') {
        v[i] = 10 + (ch - 'a');
      } else {
        switch (i) {
        case 0: tv->values[idx] = RGB(); break; // Off?
        case 1:
          tv->values[idx] = RGB(v[0]); break;
        case 2:
          tv->values[idx] = RGB(v[0], v[1], 0); break;  // weird...
        case 3:
          tv->values[idx] = RGB(v[0], v[1], v[2]); break;
        case 4:
          tv->values[idx] = RGB(addClip(v[3], v[0]),
                               addClip(v[3], v[1]),
                               addClip(v[3], v[2])); break; // RGBW?
        case 5:
          tv->values[idx] = RGB((v[0] << 4) | v[1],
                               (v[2] << 4) | v[3],
                               (v[4] << 4)); break; // Weird...
        case 6:
          tv->values[idx] = RGB((v[0] << 4) | v[1],
                               (v[2] << 4) | v[3],
                               (v[4] << 4) | v[5]); break;
        }
        break;
      }
      ++line;
      ++i;
    }
  }
}

int main(void) {
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);

  DBG_MD(APP, ("Hello World\n"));

  Serial& serial = Serial::usart3;
  serial.Setup(115200, 8, 0, 1);

  PinId blink_pin(BLINK_PIN);
  blink_pin.SetOutput();

  PinId led_pin(LED_PIN);
  led_pin.SetOutput();

  // Yes I know with 2x2 you could avoid key-scan and just wire each
  // seperately, but I am using a purchased key-scan board.
  PinIdEnum row_pins[2] = {PIN_C7, PIN_C6};
  PinIdEnum col_pins[2] = {PIN_C5, PIN_C4};
  KeyScan<2,2> keys(row_pins, col_pins);

  u8_t phase = 0;
  UpdateLeds(phase);

  sei();
  DBG_MD(APP, ("Hello World: Test\n"));

  u8_t update_0 = 0;
  u8_t update_3 = 0;
  u8_t update_4 = 0;  // ~64 updates/sec
  u8_t update_6 = 0;  // ~16 updates/sec
  u8_t update_8 = 0;  // ~4 updates/sec

  State state;
  bool bad_data = false;
  u8_t data_pos = 0;
  char read_data[64];
  while (1) {
    const i16_t now = FastMs();

    // 1024 updates/sec
    const u8_t now_0 = now;
    if (now_0 == update_0) continue;

    if (serial.Avail()) {
      u8_t rd_err;
      u8_t ch = serial.Read(&rd_err);
      if (rd_err) {
        bad_data = true;
      } else if (ch == '\r' || ch == '\n') {
        read_data[data_pos] = 0;
        if (!bad_data) {
          ProcessCmd(state, read_data);
        }
        data_pos = 0;
        bad_data = false;
      } else {
        read_data[data_pos] = ch;
        data_pos = (data_pos + 1) & 0x3F;
      }
    }
    // keys.Scan();

    // 128 updates/sec
    const u8_t now_3 = now >> 3;
    if (now_3 == update_3) continue;
    update_3 = now_3;

    // 64 updates/sec
    const u8_t now_4 = now >> 4;
    if (now_4 == update_4) continue;
    update_4 = now_4;

    // 16 updates/sec
    u8_t now_6 = now >> 6;
    if (now_6 == update_6) continue;
    update_6 = now_6;

    ++phase;
    UpdateLeds(phase);
    DBG_LO(APP, ("Call Ptr: %p Len: %d\n", leds, sizeof(leds)));
    SendWS2812(led_pin, leds, sizeof(leds), 0xFE);

    // 4 updates/sec
    u8_t now_8 = now >> 8;
    if (now_8 == update_8) continue;  // ~4fps
    update_8 = now_8;
    blink_pin.toggle();
  }
}
