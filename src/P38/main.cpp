// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Boot.h"
#include "Dbg.h"
#include "IntTypes.h"
#include "RtcTime.h"

#include "Adc.h"
#include "Serial.h"
#include "SportSensor.h"

#define VOLT_APIN (14)
#define AMP_APIN (15)

#define LED_PIN (2)

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/4, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);

  DBG_INIT(Serial::usart0, 115200);
  Adc adc;
  Adc::ConfigurePin(VOLT_APIN);
  Adc::ConfigurePin(AMP_APIN);

  PORTF.OUT  = 0;  // Set all Low
  PORTF.DIR  |= (1 << LED_PIN);  // Set LED blink pin as output.


  SportSensor sport(&Serial::usart1, /*invert=*/true, /*use_alt_pins=*/false);
  i8_t sport_current_idx, sport_volt_idx;
  sport.AddFcs40Sensors(&sport_current_idx, &sport_volt_idx);

  // Enable interrupts program execution really begins.
  sei();
  DBG_MD(APP, ("Hello World\n"));
  bool read_volt = true;
  adc.StartRead(VOLT_APIN);

  // int volt_avg = 0;
  // int amp_avg = 0;
  
  u8_t update_4 = 0;  // ~1/64 sec
  u8_t update_8 = 0;  // ~1/4 sec
  while (true) {
    const unsigned long now = FastTimeMs();
    const u8_t now_8 = now >> 8; // 1/4 sec (1024 / 256)
    if (now_8 != update_8) {
      update_8 = now_8;
      PORTF.OUTTGL = (1 << LED_PIN);  // Toggle led pin, simple heartbeat.
    }
    // Update telemetry if we were polled (call very frequently since there
    // is a limited time window to respond when polled).  This uses the last
    // value set for the polled device.
    sport.Run();

    const u8_t now_4 = now >> 4;  // roughly 60fps.
    if (update_4 != now_4) {
      update_4 = now_4;
      // OK this is overly clever but it avoids waiting on A2D
      // conversion basically it alternates reading amps/volts and
      // starts the read of one immediately after finishing the read
      // of the other.  The frequency here is probly serious overkill.
      // since it's only actually sent to controller every couple
      // 100ms.
      if (read_volt) {
        // 10bits (0 -> 0V, 1024 -> 5V)
        // 4:1 voltage divider -> 1024 = 20V
        int volts = adc.FinishRead();
        adc.StartRead(AMP_APIN);
        volts = ((20L * 100 * volts) + (1<<9)) >> 10;
        sport.SetSensor(sport_volt_idx, volts);
      } else {
        // 0.5V = 0 Amps, 4.5V = 100 Amps
        // 1024 = 5V -> 102.4 = .5V
        // 4V = 4 * (1024 / 5) = 819 -> 100 A
        // scale = 100* 10 * (1 << 16) / 819 = 
        const int ampsV = adc.FinishRead();
        adc.StartRead(VOLT_APIN);
        int amps10 =  ((80019 * (ampsV - 121)) + (1 << 15)) >> 16;
		if (amps10 < 0) amps10 = 0;
        sport.SetSensor(sport_current_idx, amps10);
      }
      read_volt = !read_volt;
    }
  }
}

