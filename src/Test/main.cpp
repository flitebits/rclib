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
#include "leds/Effects.h"
#include "leds/Rgb.h"
#include "leds/Pixel.h"
#include "leds/FPMath.h"
#include "Pins.h"
#include "Serial.h"
#include "Spi.h"
#include "WS2812.h"
#include "DShotASM.h"

#define LED_PIN (2)
#define DSHOT_PIN (4)

const u8_t nLeds = 200;
led::RGB leds[nLeds];

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  DBG_LEVEL_MD(DSHOT);
  Fill(leds, nLeds, led::clr::black);
  Fill(leds +   0, 50, led::clr::blue);
  Fill(leds +  50, 50, led::clr::red);
  Fill(leds + 100, 50, led::clr::green);
  Fill(leds + 150, 50, led::clr::yellow);

  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();

  BitBang bitbang(PORT_C, (1 << LED_PIN) | (1 << DSHOT_PIN));

  DShotISR dshot(&Serial::usart1, &bitbang);
  led::Pacifica pacifica;
  
  sei();
  DBG_MD(APP, ("Hello World: Test\n"));
  dshot.SetChannel(DSHOT_PIN, 0, false);
  dshot.Start();
  int nDshotSent = 0;

  const u16_t min_thr = 75;
  const u16_t max_thr = 255;
  i8_t thr_add = 1;
  u16_t thr = min_thr;
  int reset_cnt = 0;

  u8_t update_3 = 0;
  u8_t update_4 = 0;  // ~64 updates/sec
  u8_t update_6 = 0;  // ~16 updates/sec
  u8_t update_8 = 0;  // ~4 updates/sec
  
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

    const i16_t now = FastMs();

    // 128 updates/sec
    const u8_t now_3 = now >> 3;
    if (now_3 == update_3) continue;
    update_3 = now_3;

    thr += thr_add;
    if (thr > max_thr) { thr = max_thr; thr_add = -thr_add; }
    else if (thr < min_thr) { thr = min_thr; thr_add = -thr_add; }
    
    // 64 updates/sec
    const u8_t now_4 = now >> 4;
    if (now_4 == update_4) continue;
    update_4 = now_4;
    dshot.SetChannel(DSHOT_PIN, thr, true);
    pacifica.Run(leds, nLeds);
    // Send LED pixels out via bitbang.
    bitbang.SendWS2812(LED_PIN, leds, sizeof(leds), 0xFF);
    

    // 16 updates/sec
    u8_t now_6 = now >> 6;
    if (now_6 == update_6) continue;
    update_6 = now_6;

    // 4 updates/sec
    u8_t now_8 = now >> 8;
    if (now_8 == update_8) continue;  // ~4fps
    update_8 = now_8;
    PORTF.OUTTGL = 1 << 2;  // Toggle PF2
  }
}
