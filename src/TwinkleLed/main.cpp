// Copyright 2020 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <avr/io.h>
#include <avr/interrupt.h>

// Basic bootstraping/framework code
#include "Boot.h"
#include "Dbg.h"
#include "IntTypes.h"
#include "RtcTime.h"

// Subsystem includes
#include "Adc.h"
#include "Pwm.h"
#include "Serial.h"
#include "WS2812.h"

// LED helper functions.
#include "leds/Clr.h"
#include "leds/Pixel.h"
#include "leds/Rgb.h"
#include "leds/FPMath.h"

using led::RGB;
using led::Logify;
using led::Lookup9;
using led::bscale8;

const bool HEARTBEAT = false;  // set true to get heartbeat on board led.

#define LED_PIN (PIN_C2)
const int NUM_LEDS = 10;

u32_t seed;
void randomSeed(u16_t new_seed) {
  seed = new_seed;
}
u8_t random8(u8_t range) {
  seed = seed * 134775813L + 1;
  return (u16_t(((seed >> 16) & 0xFF) + 1) * range) >> 8;
}
u16_t random(u16_t range) {
  seed = seed * 134775813L + 1;
  return (u32_t(((seed >> 8) & 0xFFFF) + 1) * range) >> 16;
}

// clr_grad goes from pure red through yellow, to white.
// It's kind of like color tempurature.
const RGB clr_grad[9] = {
    RGB(0xFF, 0x00, 0x00), RGB(0xFF, 0x18, 0x00), RGB(0xFF, 0x38, 0x00),
    RGB(0xFF, 0x60, 0x00), RGB(0xFF, 0x80, 0x00), RGB(0xFF, 0x80, 0x10),
    RGB(0xFF, 0x80, 0x20), RGB(0xFF, 0x80, 0x40), RGB(0xFF, 0x90, 0x80)};

struct Clr {
  u8_t clr;
  u8_t brt; // How bright the value is.
  // clr goes from pure red through yellow at 128, to white.
  // It's kind of like color tempurature.
  led::RGB ToRgb() {
    led::RGB rgb = Lookup9(clr_grad, clr);
    u8_t b = Logify(brt);
    rgb.red = b;
    rgb.grn = bscale8(rgb.grn, b);
    rgb.blu = bscale8(rgb.blu, b);
    return rgb;
  }
};

struct Pix {
  Pix() : adden(0), val(0) {
    tgt.clr = random8(255);
    tgt.brt = random8(255);
    Shift();
  }
  void Shift() {
    curr = tgt;
    val = 0;
    adden = random8(64) + 8;
    tgt.clr = random8(255);
    // Alternate bright and dark.
    tgt.brt = random8(128) + ((curr.brt > 127) ? 16 : 128);
  }

  void Inc() {
    val += adden;
    if (val > (0xFF << 6)) Shift();
  }

  led::RGB ToRgb() {
    u8_t v = ((val + (1<<5)) >> 6);
    Clr clr;
    clr.brt = curr.brt + (((tgt.brt - i16_t(curr.brt)) * v) >> 8);
    clr.clr = curr.clr + (((tgt.clr - i16_t(curr.clr)) * v) >> 8);
    return clr.ToRgb();
  }
  u8_t ToPwm() {
    u8_t v = ((val + (1<<5)) >> 6);
    u8_t brt = curr.brt + (((tgt.brt - i16_t(curr.brt)) * v) >> 8);
    return Logify(brt);
  }
  protected:
    u8_t adden; // 2.6 fp
    i16_t val;  // 8.6 fp
    Clr tgt;
    Clr curr;
};

void update_leds(Pix* pixs, led::RGB* leds, int len) {
  for (int i = 0; i < len; ++i) {
    pixs[i].Inc();
    leds[i] = pixs[i].ToRgb();
  }
}

const int NUM_STRAND = 6;
Pix strand[NUM_STRAND];
u8_t map[NUM_STRAND] = {0, 1, 2, 3, 4, 5};
void update_strands(Pwm* pwm) {
  for (int i = 0; i < NUM_STRAND; ++i) {
    strand[i].Inc();
    pwm->Set(map[i], strand[i].ToPwm());
  }
}

int main(void) {
  led::RGB leds[NUM_LEDS + 1]; // Define the array of leds
  Pix pixs[NUM_LEDS];

  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  DBG_INIT(Serial::usart0, 115200);

  Adc adc;
  adc.ConfigurePin(6);    // Read a "mostly random" seed from Analog pin 6 (PD6)
  randomSeed(adc.Read(6));

  // Configure LED pin as output (low, not inverted, not pullup)
  PinId(LED_PIN).SetOutput();
  // Blnky Led on board
  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput(1);  // High is led off.

  // Pwm module use the 3 TCA timers as split 8bit PWM control providing
  // up to 6 channels of PWM.
  Pwm pwm(PORT_D, 1000);  // Select what port to use and freq of PWM
  for (int i = 0; i < 6; ++i) {
    pwm.Enable(i);  // Enable PWM on pins D0-D5
    pwm.Set(i, 0);
  }

  // Enable interrupts (end of init, real program begin).
  sei();

  u8_t update_4 = 255;  // ~1/64 sec  (~16ms each)
  u8_t update_8 = 255;  // ~1/4 sec  (~250ms each )
  while (true) {
    u32_t now = FastTimeMs();
    u8_t now_4 = now >> 4;
    if (now_4 == update_4) continue;
    update_4 = now_4;  // ~60fps
    update_strands(&pwm);
    update_leds(pixs, leds, NUM_LEDS);
    SendWS2812(PinId(LED_PIN), leds, 3 * NUM_LEDS, 0xFF);

    u8_t now_8 = now >> 8;
    if (now_8 == update_8) continue;
    update_8 = now_8;  // ~4fps
    if (HEARTBEAT) {
      blink_pin.toggle();
    }
  }
}
