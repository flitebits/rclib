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
#include "Spi.h"
#include "Serial.h"
#include "SBus.h"
#include "SportSensor.h"

// LED helper functions.
#include "leds/Rgb.h"
#include "leds/Clr.h"

#define TEST_RGBW
#ifdef TEST_RGBW
typedef led::RGBW pix_t;
pix_t rgb_data[] = {
  led::wclr::blue,
  led::wclr::blue,
  led::wclr::red,
  led::wclr::blue,
  led::wclr::blue,
};
#else  // !TEST_RGBW
typedef led::RGB pix_t;
pix_t rgb_data[] = {
  led::clr::blue,
  led::clr::blue,
  led::clr::red,
  led::clr::blue,
  led::clr::blue,
};
#endif  // TEST_RGBW

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/2, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_HI(ADC);

  // This reads usart 3 to get RC channel information (up to 16 channels).
  SBus sbus(&Serial::usart1);

  // This reads/writes usart 1 to respond to telemetry polling from
  // receiver.
  SportSensor sport(&Serial::usart1, /*use_alt_pins=*/false);
  i8_t sport_current_idx, sport_volt_idx;
  sport.AddFcs40Sensors(&sport_current_idx, &sport_volt_idx);

  // Setup and configure the Analog to Digial converter.  It will to
  // do 10bit A2D. You can convert between analog pins and port pins
  // using Pins.h.
  Adc adc;
  u8_t ain_pin = 8;
  Adc::ConfigurePin(ain_pin);
  int adc_val = adc.Read(ain_pin);

  // Pwm module use the 3 TCA timers as split 8bit PWM control providing
  // up to 6 channels of PWM.
  Pwm pwm(PORT_D, 1000);  // Select what port to use and freq of PWM
  for (int pwm_pin = 0; pwm_pin < 6; ++pwm_pin) {
    pwm.Enable(pwm_pin);  // Enable PWM for this pin.
  }

  // Used to control addressable LED strips. It can talk to either
  // APA102 (which has data + clk lines) or WS2812/SK6812 leds (just
  // data line).  SK6812 leds generally have more varients (different
  // sizes as well as RGBW).
#ifdef TEST_RGBW  
  Spi::spi.SetupSK6812(Spi::PINS_PA47);
#else
  Spi::spi.SetupAPA102(Spi::PINS_PA47, 2000000);
#endif

  PORTF.OUT = 0;  // set all PORTF pins to zero.
  PORTF.DIR = 0xFF;  // Pins PF2 & PF6 set as output pints.

  PORTF.PIN0CTRL = 0;  // Not inverted, not pullup enabled, no interrupts.
  PORTF.PIN1CTRL = 0;
  PORTF.PIN2CTRL = 0;
  PORTF.PIN3CTRL = 0;
  PORTF.PIN4CTRL = 0;
  PORTF.PIN5CTRL = 0;
  PORTF.PIN6CTRL = 0;

  // Enable interrupts (end of init, real program begin).
  sei();
  
  u8_t update_10 = 0;  // ~1 sec
  u8_t update_9 = 0;  // ~1/2 sec
  u8_t update_8 = 0;  // ~1/4 sec
  u8_t update_4 = 0;  // ~1/64 sec
  while (true) {
    u32_t now = FastTimeMs();
    u8_t now_4 = now >> 4;
    if (now_4 != update_4) {  // ~60fps
      u8_t level = now_4;
      for (int i = 0; i < 6; ++i) {
	pwm.Set(i, level);
	level = level + (256 / 6);
      }
    }

    u8_t now_9 = now >> 9;
    if (now_9 != update_9) {  // Simple heart beat 
      update_9 = now_9;
      PORTF.OUTTGL = 0x7F; // Toggle PF0-6
    }

    sport.Run();
    if (sbus.Run()) {
      // Update the PWM from the just update rc channel info.
      pwm.Set(0,  sbus.GetChannel(1) >> 3);
    }

    u8_t now_8 = now >> 8;
    if (now_8 != update_8) {
      update_8 = now_8;
      Spi::spi.WaitForIdle();
      pix_t first = rgb_data[0];
      for (int i = 0; i < 4; ++i) {
        rgb_data[i] = rgb_data[i + 1];
      }
      rgb_data[4] = first;
      Spi::spi.UpdateLeds(rgb_data, 5, 0x10);
    }
    
    u8_t now_10 = now >> 10;
    if (now_10 != update_10) {
      update_10 = now_10;

      adc.StartRead(ain_pin);
      sport.SetSensor(sport_current_idx,
		      (((now >> 5) * 10) >> 5) & 0xFF);
      sport.SetSensor(sport_volt_idx, (now >> 3) & 0xFFF);

      adc_val = adc.FinishRead();
      DBG_LO(ADC, ("Analog: %d\n", adc_val));
    }
  }
}
