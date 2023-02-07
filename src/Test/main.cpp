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
#include "Pwm.h"
#include "Pca9685.h"
#include "Serial.h"
#include "Twi.h"
#include "WS2812.h"
#include "DShotASM.h"

#define BLINK_PIN (PIN_F2)

#if defined(__AVR_ATmega4808__)
#define LED_PIN (PIN_F0)
#endif

#if defined(__AVR_ATmega4809__)
#define LED_PIN (PIN_C2)
#endif

#define LED_RGB (0)
#define LED_RGBW (1)

#define LED_TYPE (LED_RGBW)

#if (LED_TYPE == LED_RGBW)
using led::RGBW;
typedef led::RGBW led_t;
#else
using led::RGB;
typedef led::RGB led_t;
#endif

const u8_t nLeds = 200;
led_t leds[nLeds];
using led::Fill;

u8_t phase_map[8] = {2, 4, 8, 16, 32, 64, 128, 255};
void UpdatePwm(Pwm& pwm, u8_t state, u8_t phase) {
  for (int i = 0; i < pwm.NumPwm(); ++i) {
    pwm.Set(i, (state & (0x1 << i)) ? 64 : 0);
  }
}

void UpdateLeds(u8_t phase, bool solid){
  for (int c = 0; c < (nLeds / 8); c++) {
    int idx = c * 8;
    for (u8_t i = 0; i < 8; ++i) {
      u8_t p = solid ? 255 : phase_map[(i + phase) % 0x7];
#if (LED_TYPE == LED_RGBW)
      leds[idx + i] = led::HsvToRgbw(led::HSV(c * 50, 255, p));
#else
      leds[idx + i] = led::HsvToRgb(led::HSV(c * 50, 255, p));
#endif
    }
  }
}

void SetWhite(bool boost){
  // Boost means everything on at full power.
#if (LED_TYPE == LED_RGBW)
  RGBW pix = boost ? RGBW(0xFF, 0xFF) : RGBW(0xFF);
#else
  RGB pix(0xFF);
#endif
  Fill(leds, nLeds, pix);
}

static u8_t key_map[] = {0, 1, 3, 2 };

void SetKeys(const KeyScan<2,2>& keys, bool is_white) {
  for (u8_t i = 0; i < keys.NumKeys(); ++i) {
    if (keys.IsDown(i)) {
      if (is_white) {
        leds[key_map[i]] = led_t(0xFF, 0, 0);
      } else {
        leds[key_map[i]] = led_t(0xFF);
      }
    } else {
      leds[key_map[i]] = led_t(0);
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

  PinId blink_pin(BLINK_PIN);
  blink_pin.SetOutput();

  PinId led_pin(LED_PIN);
  led_pin.SetOutput();

  Twi::twi.Setup(Twi::PINS_DEF, Twi::I2C_1M);

  // Yes I know with 2x2 you could avoid key-scan and just wire each
  // seperately, but I am using a purchased key-scan board.
  PinIdEnum row_pins[2] = {PIN_C7, PIN_C6};
  PinIdEnum col_pins[2] = {PIN_C5, PIN_C4};
  KeyScan<2,2> keys(row_pins, col_pins);

  // Pwm module use the 3 TCA timers as split 8bit PWM control providing
  // up to 6 channels of PWM.
  Pwm pwm(PORT_D, 1000);  // Select what port to use and freq of PWM
  for (int i = 0; i < pwm.NumPwm(); ++i) {
    pwm.Enable(i);  // Enable PWM on pins D0-D5
    pwm.Set(i, 0);
  }

  u8_t phase = 0;
  UpdateLeds(false, phase);

  Pca9685 pwm16(0x80, 16);

  sei();
  DBG_MD(APP, ("Hello World: Test\n"));

  pwm16.Init(/*totem=*/true);

  u8_t update_0 = 0;
  u8_t update_3 = 0;
  u8_t update_4 = 0;  // ~64 updates/sec
  u8_t update_6 = 0;  // ~16 updates/sec
  u8_t update_8 = 0;  // ~4 updates/sec

  u8_t pwm_state = 0;
  i16_t cnt = 4096;
  i8_t add = -16;
  while (1) {
    const i16_t now = FastMs();

    // 1024 updates/sec
    const u8_t now_0 = now;
    if (now_0 == update_0) continue;
    keys.Scan();

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

    UpdatePwm(pwm, pwm_state, phase);
    // DBG_MD(APP, ("Keys: 0x%08lx\n", keys.State()));
    pwm16.SetLed(cnt, 1);
    pwm16.Write();
    /*
    u8_t led0_seq[5] = {0x0A, 0x00, 0x00, 0x00, 0x00};
    led0_seq[3] = cnt;
    Twi::twi.MasterSendBytes(0x80, led0_seq, sizeof(led0_seq));
    */
    cnt += add;
    if (cnt < -add || cnt > (4096 - add)) add = -add;

    ++phase;
    u8_t test_mode = (FastSecs() >> 2) & 0x3; // change mode every 2 sec
    switch (test_mode) {
    case 0:
      // Animated (test of signal integrity).
      UpdateLeds(phase, false);
      SetKeys(keys, false);
      break;
    case 1:
      // all 8 colors at the same level (semi realistic color draw)
      UpdateLeds(phase, true);
      SetKeys(keys, false);
      break;
    case 2:
      SetWhite(false);  // Just White at 100%
      SetKeys(keys, true);
      break;
    case 3:
      SetWhite(true);   // Everything at 100% (max power draw).
      SetKeys(keys, true);
      break;
    }
    // SetKeys(keys, true);
    SendWS2812(led_pin, leds, sizeof(leds), 0xFF);

    // 4 updates/sec
    u8_t now_8 = now >> 8;
    if (now_8 == update_8) continue;  // ~4fps
    update_8 = now_8;
    blink_pin.toggle();
  }
}
