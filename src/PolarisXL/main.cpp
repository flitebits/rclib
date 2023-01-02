// Copyright 2023 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "Boot.h"
#include "Dbg.h"
#include "IntTypes.h"
#include "RtcTime.h"

#include "leds/Clr.h"
#include "leds/FPMath.h"
#include "leds/LedSpan.h"
#include "leds/Pixel.h"
#include "leds/Rgb.h"

#include "Pins.h"
#include "Rand.h"
#include "SBus.h"
#include "Serial.h"
#include "Spi.h"

using led::RGB;
using led::scale8;

#define LIGHT_ON_CH (8)
#define LIGHT_MODE_CH (9)
#define LIGHT_THR_CH (10)

const int NAV_OUT_LEDS = 10;
const int NAV_IN_LEDS = 5;
const int WING_LEDS = 17;
const int WING_LED_OFF = (NAV_IN_LEDS + NAV_OUT_LEDS);
const int WING_MID_OFF = (WING_LED_OFF + WING_LEDS);
const int NAV_LOFF = 0;
const int NAV_ROFF = (WING_LED_OFF + 2 * WING_LEDS);
const int TAIL_LEDS = 12;
const int TAIL_LED_OFF = (2 * (NAV_IN_LEDS + NAV_OUT_LEDS + WING_LEDS));
const int NUM_LEDS = TAIL_LED_OFF + 2 * TAIL_LEDS;
RGB leds[NUM_LEDS + 1];
u8_t led_map[NUM_LEDS];

static const RGB eng_grad[5] = { RGB(0x00, 0x00, 0x00),
                                 RGB(0x40, 0x00, 0x00),
                                 RGB(0x80, 0x20, 0x00),
                                 RGB(0xFF, 0x80, 0x00),
                                 RGB(0xFF, 0xFF, 0x80) };

inline u8_t subClip(u8_t a, u8_t b) {
  u8_t r = a - b;
  return (r > a) ? 0 : r;
}

inline u8_t addClip(u8_t a, u8_t b) {
  u8_t r = a + b;
  return (r < a) ? 0xFF : r;
}

class Lights {
public:
  Lights(RGB* leds) : iter_(0) {
    void* ptr = leds;
    ptr = lnav_ .SetSpan(ptr, NAV_IN_LEDS + NAV_OUT_LEDS, false);
    ptr = lwing_.SetSpan(ptr, WING_LEDS, true);
    ptr = rwing_.SetSpan(ptr, WING_LEDS, false);
    ptr = rnav_ .SetSpan(ptr, NAV_IN_LEDS + NAV_OUT_LEDS, false);
    ptr = tail_ .SetSpan(ptr, 2* TAIL_LEDS, false);
  }

  bool Update(const SBus& sbus) {
    static u8_t prev_brightness = 0;
    u8_t brightness;
    u8_t lvl = SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_ON_CH));
    switch (lvl) {
    default: brightness = 0x00; break;
    case  1: brightness = 0x40; break;
    case  2: brightness = 0xFF; break;
    }
    if (brightness == 0) {
      if (brightness == prev_brightness) return false;
      memset(led_map, 0, NUM_LEDS);
      led::Fill(leds, NUM_LEDS, led::clr::black);
      prev_brightness = brightness;
      return true;
    }

    if (brightness != prev_brightness) {
      u8_t b = brightness;
      lnav_.Fill(RGB(b, 0, 0));
      rnav_.Fill(RGB(0, b, 0));
      tail_.Fill(RGB(b, b, b));
    }

    const u8_t mode = SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_MODE_CH));
    static u8_t prev_mode = -1;
    if (mode != prev_mode) {
      memset(led_map, 0, NUM_LEDS);
    }
    const u16_t throttle = sbus.GetChannel(LIGHT_THR_CH);
    switch (mode) {
    case 0: {
      if (brightness == prev_brightness &&
          mode == prev_mode) {
        return false; // no update
      }
      RGB wht(brightness);
      lwing_.Fill(wht);
      rwing_.Fill(wht);
      break;
    }
    case 1: PulseMode(brightness, throttle); break;
    case 2: FireMode(brightness, throttle); break;
    }
    prev_mode = mode;
    prev_brightness = brightness;
    return true;
  }

protected:
  void PulseMode(u8_t brightness, u16_t throttle) {
    const i8_t PULSE_SZ = 2;
    const i32_t PULSE_SCALE = 16;  // 4 bit value
    const i32_t SPEED_SCALE = 16;  // 4 bit value
    const i16_t PULSE_MIN = 32;

    static i16_t pulse_val = 0;  // 12 bit 0-1
    static i16_t pulse_loc = 0;  // Has 8 bits frac
    for (int i=0; i < WING_LEDS + 2 * PULSE_SZ; i++) {
      led_map[i] = scale8(led_map[i], 0xF0);
    }
    // pulse_val is logically 12 bits
    pulse_val += PULSE_MIN + ((throttle * PULSE_SCALE + (1<<7)) >> 8);
    if (pulse_val >= (1<<12)) {
      pulse_val = (1<<12);
      pulse_loc += PULSE_MIN + ((throttle * SPEED_SCALE + (1 << 7)) >> 8);
    }
    int pulse_idx = ((pulse_loc + (1 << 7)) >> 8);
    if (pulse_idx > WING_LEDS) {
      pulse_idx = 0;
      pulse_loc = 0;
      pulse_val = 0;
    }
    pulse_idx += PULSE_SZ;

    for (int i=0; i <= PULSE_SZ; i++) {
      int val = led_map[pulse_idx + i] + (((0xFFL - 0x11 * i) * pulse_val + (1<<11)) >> 12);
      val = val > 0xFF ? 0xFF : val;
      led_map[pulse_idx + i] = val;
      led_map[pulse_idx - i] = val;
    }
    for (int i=0; i<WING_LEDS; i++) {
      int val = scale8(led_map[PULSE_SZ + i], brightness);
      RGB pix(val, val, val);
      lwing_.Set(i, pix);
      rwing_.Set(i, pix);
    }
  }

  void Fire2012WithPalette(u8_t cooling, u8_t sparking, bool reverse, int numLeds,
                           unsigned char brightness, u8_t* heat, LedSpan<RGB>* leds)
  {
    // Step 1.  Cool down every cell a little
    for( int i = 0; i < numLeds; i++) {
      heat[i] = subClip(heat[i],  random8(((cooling * 10) / numLeds) + 2));
    }

    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for( int k= numLeds - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] ) / 2;
    }

    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if( random8(255) < sparking ) {
      int y = random8(7);
      heat[y] = addClip(heat[y], random8(255 - 160) + 160);
    }

    // Step 4.  Map from heat cells to LED colors
    for( int j = 0; j < numLeds; j++) {
      // Scale the heat value from 0-255 down to 0-240
      // for best results with color palettes.
      u8_t colorindex = scale8( heat[j], 240);
      RGB& v = leds->At(j);
      v = Lookup5(eng_grad, colorindex);
      Fade(&v, brightness);
    }
  }

  void FireMode(u8_t brightness, u16_t throttle) {
#ifdef DBG_LEDS
    DbgSerial.print("Fire: ");
    DbgSerial.print(brightness);
    DbgSerial.println();
#endif
    if ((++iter_ & 0x1) == 1) return;

    const u16_t fire = (throttle + (1<<3)) >> 4;
    const int cooling = 85 - ((50 * fire) >> 8);  // 100 - 20
    const int sparking = 50 + ((150 * fire) >> 8); // 50 - 200
    Fire2012WithPalette(cooling, sparking, true, WING_LEDS, brightness,
                        led_map, &lwing_);
    Fire2012WithPalette(cooling, sparking, false, WING_LEDS, brightness,
                        led_map + WING_LEDS, &rwing_);
  }

  u8_t iter_;
  LedSpan<RGB>  lnav_;
  LedSpan<RGB>  lwing_;
  LedSpan<RGB>  rwing_;
  LedSpan<RGB>  rnav_;
  LedSpan<RGB>  tail_;
};

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);

  // Blnky Led on board
  PinId blinky(PIN_F2);
  blinky.SetOutput();

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_HI(APP);
  DBG_LEVEL_MD(SBUS);
  SBus sbus(&Serial::usart2, /*invert=*/true);
  led::Fill(leds, NUM_LEDS, led::clr::black);
  Spi::spi.SetupAPA102(Spi::PINS_PA47, 2000000);

  Lights lights(leds);
  sei();
  DBG_MD(APP, ("Hello World\n"));

  Spi::spi.UpdateLeds(leds, NUM_LEDS, 0x00);

  u8_t update_4 = 0;  // ~1/64 sec
  u8_t update_8 = 0;  // ~1/4 sec
  while (true) {
    sbus.Run();

    const unsigned long now = FastTimeMs();
    const u8_t now_4 = now >> 4;
    if (now_4 == update_4) continue;
    update_4 = now_4;
    if (lights.Update(sbus)) {
      Spi::spi.UpdateLeds(leds, NUM_LEDS, 0xFF);
    }

    const u8_t now_8 = now >> 8; // 1/4 sec (1024 / 256)
    if (now_8 != update_8) {
      update_8 = now_8;
      blinky.toggle();
    }
  }
}
