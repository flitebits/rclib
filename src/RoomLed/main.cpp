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

#include "leds/Rgb.h"
#include "leds/Pixel.h"
#include "Spi.h"
#include "Pins.h"
#include "Serial.h"
#include "WS2812.h"

#define LED_PIN (2)

const u8_t nLeds = 200;
led::RGBW leds[nLeds];

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);

  Fill(leds, nLeds, led::wclr::black);
  Fill(leds +   0, 49, led::wclr::blue);
  Fill(leds +  49, 51, led::wclr::red);
  Fill(leds + 100, 50, led::wclr::green);
  Fill(leds + 150, 50, led::wclr::yellow);
  
  PinId led_pin(PIN_A4);
  led_pin.SetOutput();
  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();

  Spi::spi.SetupSK6812(Spi::PINS_PA47);
  
  sei();
  DBG_MD(APP, ("Hello World: Test\n"));

  u8_t update_4 = 0;
  u8_t bright = 0;
  i8_t adden = 1;
  while (1) {
    const unsigned long now = FastTimeMs();
    const u8_t now_4 = now >> 4;
    if (now_4 != update_4) {
      update_4 = now_4;
      // Spi::spi.UpdateLeds(leds, nLeds, bright);
      SendWS2812(led_pin, leds, 4*nLeds, bright);
      bright += adden;
      if (bright == 255) adden = -1;
      else if (bright == 0) adden = 1;
    }
  }
}
