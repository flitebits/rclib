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

#include "DShot.h"
#include "Serial.h"
#include "SportSensor.h"

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  DBG_LEVEL_MD(DSHOT);

  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();

  Serial* kSerial = &Serial::usart1;
  KissTelemetry::ConfigureSerial(kSerial, false, false);
  
  SportSensor sport(&Serial::usart2, /*invert=*/true);
  i8_t sport_current_idx, sport_volt_idx;
  sport.AddFcs40Sensors(&sport_current_idx, &sport_volt_idx);
  
  sei();
  DBG_MD(APP, ("Hello World: WingTelem\n"));
  
  KissTelemetry telemetry;
  u8_t telem_byte = 0;
  u8_t update_3 = 0;
  u8_t update_4 = 0;  // ~64 Hz
  u8_t update_9 = 0;  // ~2 Hz
  /* Replace with your application code */
  while (1)  {
    sport.Run();
    
    while (kSerial->Avail()) {
      u8_t err;
      u8_t data = kSerial->Read(&err);
      if (err) {
	DBG_MD(DSHOT, ("DSHOT Err: %d\n", err));
	telem_byte = 0;
	continue;
      }
      telemetry.SetBuffer(telem_byte++, data);
      if (telem_byte >= kKissTelementryLen) {
	if (telemetry.Parse(telem_byte - kKissTelementryLen)) {
	  telem_byte = 0;
	  // telemetry.Print();
	  sport.SetSensor(sport_volt_idx, telemetry.volts);
	  sport.SetSensor(sport_current_idx, telemetry.amps / 10);
	}
      }
    }
    const unsigned long now = FastTimeMs();
    const u8_t now_3 = now >> 3;
    if (now_3 == update_3) continue;
    update_3 = now_3;

    const u8_t now_4 = now >> 4;
    if (now_4 == update_4) continue;
    update_4 = now_4;

    const u8_t now_9 = now >> 9;
    if (now_9 == update_9) continue;  // ~2 Hz
    update_9 = now_9;
    PORTF.OUTTGL = 1 << 2;  // Toggle PF2
    // telemetry.Print();
    // DBG_HI(APP, ("Bytes: %d\n", telem_byte));
  }
}
