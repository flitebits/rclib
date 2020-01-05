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
#include "DShotISR.h"
#include "IntTypes.h"
#include "RtcTime.h"

#include "leds/Rgb.h"
#include "leds/Pixel.h"
#include "Spi.h"
#include "Pins.h"
#include "Serial.h"
#include "WS2812.h"

#define LED_PIN (2)
#define DSHOT_PIN (2)

const u8_t nLeds = 200;
led::RGBW leds[nLeds];

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/2, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  DBG_LEVEL_HI(DSHOT);
  Fill(leds, nLeds, led::wclr::black);
  Fill(leds +   0, 49, led::wclr::blue);
  Fill(leds +  49, 51, led::wclr::red);
  Fill(leds + 100, 50, led::wclr::green);
  Fill(leds + 150, 50, led::wclr::yellow);
  
  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();

  DShotISR dshot(&Serial::usart1, PORT_F);
  Spi::spi.SetupSK6812(Spi::PINS_PA47);
  
  sei();
  DBG_MD(APP, ("Hello World: Test\n"));

  dshot.SetChannel(DSHOT_PIN, 0, false);
  // dshot.Start();
  
  const u16_t min_thr = 48;
  const u16_t max_thr = 48;
  i8_t thr_add = 1;
  u16_t thr = min_thr;
  int nDshotSent = 0;
  int reset_cnt = 0;
  u8_t update_3 = 0;
  u8_t update_4 = 0;  // ~64 updates/sec
  //u8_t update_8 = 0;  // ~4 updates/sec
  u8_t bright = 0;
  i8_t adden = 1;
  while (1) {
    bool new_telemetry;
    if (dshot.Run(&new_telemetry)) {
      nDshotSent++;
      if (reset_cnt < 1000) {
        reset_cnt++;
        continue;
      }
      if (reset_cnt == 1000) {
        reset_cnt++;
        dshot.SetChannel(DSHOT_PIN, thr, false);
      }
    }
    if (new_telemetry) {
      dshot.GetTelemetry()->Print();
    }
          
    const unsigned long now = FastTimeMs();
    const u8_t now_3 = now >> 3;
    if (now_3 != update_3) {
      update_3 = now_3;
      thr += thr_add;
      if (thr > max_thr) { thr = max_thr; thr_add = -thr_add; }
      else if (thr < min_thr) { thr = min_thr; thr_add = -thr_add; }
      bool request_telemetry = false;
      if (now_3 == 0xFF) {
        DBG_MD(APP, ("\n Requesting Telemetry[%lu-%u]: %d",
		     now, nDshotSent, thr));
        request_telemetry = true;
      }
      dshot.SetChannel(DSHOT_PIN, thr, request_telemetry);
    }
    /*
    u8_t now_8 = now >> 8;
    if (now_8 != update_8) {  // ~4fps
      update_8 = now_8;
      PORTF.OUTTGL = 1 << 2;  // Toggle PF2
    }
    */
    const u8_t now_4 = now >> 4;
    if (now_4 != update_4) {
      update_4 = now_4;
      Spi::spi.UpdateLeds(leds, nLeds, bright);
      bright += adden;
      if (bright == 255) adden = -1;
      else if (bright == 0) adden = 1;
      // SendWS2812(PIN_F2, leds, 4*nLeds, 0xFF);
    }
  }
}
