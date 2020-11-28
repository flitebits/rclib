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

#include "DShotISR.h"
#include "leds/Rgb.h"
#include "leds/Pixel.h"
#include "Pins.h"
#include "Serial.h"
#include "Spi.h"
#include "WS2812.h"
#include "DShotASM.h"

#define LED_PIN (6)
#define DSHOT_PIN (5)

const u8_t nLeds = 200;
led::RGBW leds[nLeds];

#define DSHOT_ENABLE 1

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

  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();

  BitBang bitbang(PORT_C, 0x60);  // pins C5 DShot, C6 LED.
#if DSHOT_ENABLE
  DBG_LEVEL_HI(DSHOT);
  DShotISR dshot(&Serial::usart1, &bitbang);
#endif
  sei();
  DBG_MD(APP, ("Hello World: Test\n"));
  // dshot_pdiv1(&bitbang);

#if DSHOT_ENABLE
  dshot.SetChannel(DSHOT_PIN, 0, false);
  dshot.Start();
  int nDshotSent = 0;
  const u16_t min_thr = 75;
  const u16_t max_thr = 255;
  i8_t thr_add = 1;
  u16_t thr = min_thr;
  int reset_cnt = 0;
#endif
  
  u8_t update_3 = 0;
  u8_t update_4 = 0;  // ~64 updates/sec
  u8_t update_6 = 0;  // ~16 updates/sec
  u8_t update_8 = 0;  // ~4 updates/sec
  u8_t bright = 0;
  while (1) {
#if DSHOT_ENABLE
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
#endif	     
    const unsigned long now = FastTimeMs();

#if DSHOT_ENABLE
    // 128 updates/sec
    const u8_t now_3 = now >> 3;
    if (now_3 == update_3) continue;
    update_3 = now_3;

    thr += thr_add;
    if (thr > max_thr) { thr = max_thr; thr_add = -thr_add; }
    else if (thr < min_thr) { thr = min_thr; thr_add = -thr_add; }
    bool request_telemetry = false;
    if (now_3 == 0xFF) {
      DBG_MD(APP, ("\n Requesting Telemetry[%lu-%u]: %d\n",
		   now, nDshotSent, thr));
      request_telemetry = true;
    }
    dshot.SetChannel(DSHOT_PIN, thr, request_telemetry);
#endif
    
    // 64 updates/sec
    const u8_t now_4 = now >> 4;
    if (now_4 == update_4) continue;
    update_4 = now_4;
    bitbang.SendWS2812(PinId(PIN_C6), leds, sizeof(leds), bright);

    // 16 updates/sec
    u8_t now_6 = now >> 6;
    if (now_6 == update_6) continue;
    update_6 = now_6;
    led::RGBW hold = leds[0];
    memmove(leds, leds + 1, sizeof(leds) - sizeof(leds[0]));
    leds[nLeds - 1] = hold;

    // 4 updates/sec
    u8_t now_8 = now >> 8;
    if (now_8 == update_8) continue;  // ~4fps
    update_8 = now_8;
    PORTF.OUTTGL = 1 << 2;  // Toggle PF2
  }
}
