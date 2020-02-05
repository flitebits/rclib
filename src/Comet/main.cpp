// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "Boot.h"
#include "Dbg.h"
#include "IntTypes.h"
#include "RtcTime.h"

#include "Pwm.h"
#include "Rand.h"
#include "Serial.h"
#include "SBus.h"
#include "Util.h"
#include "Pins.h"
#include "WS2812.h"

// LED helper functions.
#include "leds/Rgb.h"
#include "leds/Pixel.h"
#include "leds/Clr.h"

#define WING_CNT (16)
#define TIP_CNT (7)
#define LED_CNT (2 * (WING_CNT + TIP_CNT))
#define LWING_IDX (0)
#define LTIP_IDX (WING_CNT)
#define RWING_IDX (WING_CNT + TIP_CNT)
#define RTIP_IDX (2 * WING_CNT + TIP_CNT)

#define LWING_PWM (0)
#define RWING_PWM (1)

#define LIGHT_LEVEL_CH (4)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH (5)  // Sets Lights Mode
#define LIGHT_BRIGHT_CH (6)  // Adjust leading edge light brightness.
#define LIGHT_THROTTLE_CH (7)

#define LED_PIN (PIN_A4)

const u8_t log_map[9] = {0, 1, 2, 5, 11, 24, 52, 115, 255};

u8_t logify(u8_t val) {
  u8_t lo = val & 0x1F;
  u8_t hi = val >> 5;
  int ret = ((u16_t(log_map[hi]) << 5) +
             (i16_t(log_map[hi + 1]) - log_map[hi]) * lo);
  return (ret + (1<<4)) >> 5;
}

led::RGBW heat [5] = {
                     led::RGBW(0x00, 0x00, 0x00),
                     led::RGBW(0x80, 0x00, 0x00),
                     led::RGBW(0xFF, 0x80, 0x00),
                     led::RGBW(0xFF, 0xFF, 0x00),
                     led::RGBW(0xFF, 0xFF, 0xFF, 0xFF),
};
enum {
      LEFT_SIDE = 1 << 0,
      RIGHT_SIDE = 1 << 1,
      BOTH_SIDES = (LEFT_SIDE | RIGHT_SIDE),
};

class Lights {
public:
  Lights(Pwm* pwm) :
    curr_level_(255),
    curr_scale_(0),
    curr_mode_(255),
    pwm_scale_(1<<4),
    tip_flash_idx_(0),
    pwm_(pwm) {
    pwm_->Set(LWING_PWM, 0);
    pwm_->Set(RWING_PWM, 0);
  }
  void SetLeftNav(const led::RGBW& color) {
    led::Fill(&leds[LTIP_IDX] , TIP_CNT, color);
  }
  void SetRightNav(const led::RGBW& color) {
    led::Fill(&leds[RTIP_IDX], TIP_CNT, color);
  }
  void SetWing(const led::RGBW& color, u8_t sides = BOTH_SIDES) {
    if (sides & LEFT_SIDE) {
      led::Fill(&leds[LWING_IDX], WING_CNT, color);
    }
    if (sides & RIGHT_SIDE) {
      led::Fill(&leds[RWING_IDX], WING_CNT, color);
    }
  }

  void OffMode() {
    led::Fill(leds, LED_CNT, led::wclr::black);
  }

  void UpdateMode(u8_t new_level, u8_t new_mode, int brt, int thr) {
    randomSeedMix(RTC.CNT);
    if (new_level == 0){
      if (curr_level_ == 0) return;
      curr_level_ = new_level;
      curr_mode_ = 255;
      pwm_->Set(LWING_PWM, 0);
      pwm_->Set(RWING_PWM, 0);
      curr_scale_ = 0;
      led::Fill(leds, LED_CNT, led::wclr::black);
      PushLeds();
      return;
    }

    if (brt < 32) brt = 0;
    else {
      brt = (brt >> 3);  // convert 11 bits to 8 bits.
    }
    if (brt > 255) brt = 255;
    if (new_level == 1) brt = brt >> 2;
    brt = (brt * (int)pwm_scale_ + (1<<3)) >> 4;
    if (brt > 255) brt = 255;
    pwm_->Set(LWING_PWM, brt);
    pwm_->Set(RWING_PWM, brt);

    if (curr_mode_ == new_mode && curr_level_ == new_level) return;
    curr_scale_ = (new_level == 1) ? 0x40 : 0xFF;
    switch (new_mode) {
    case 0:
      SetLeftNav(led::wclr::red);
      SetRightNav(led::wclr::green);
      SetWing(led::wclr::white, BOTH_SIDES);
      break;
    case 1:
      SetLeftNav(led::wclr::red);
      SetRightNav(led::wclr::green);
      if (curr_mode_ != new_mode) {
        for (int i=0; i > LED_CNT; ++i) {
          work[i] = 0;
        }
        SetWing(led::wclr::black);
      }      
      break;
    case 2:
      SetLeftNav(led::wclr::red);
      SetRightNav(led::wclr::green);
      if (curr_mode_ != new_mode) {
        SetWing(led::wclr::black);
      }      
      break;
    }
    curr_level_ = new_level;
    curr_mode_ = new_mode;
  }

  void Update(int thr) {
    switch (curr_mode_) {
    case 0: break;
    case 1: PulseMode(thr); break;
    case 2: Trippy(thr); break;
    }
    PushLeds();
  }
  
  void PushLeds() {
    SendWS2812(PinId(LED_PIN), leds, LED_CNT * 4, curr_scale_);
  }


  void PulseMode(int thr) {
    const u32_t SPEED_SCALE = 32;  // 4 bit value
    const u32_t PULSE_SCALE = 32;  // 4 bit value
    const i16_t PULSE_MIN   = 32; 
  
    // "Cool" Wing leds
    for (int i=0; i < WING_CNT + 4; i++) {
      work[i] = (work[i] * int(0xF0)) >> 8;
    }
    // Cool pwm boost...
    pwm_scale_ = pwm_scale_ + (((0x10 - pwm_scale_) * 0xF0) >> 8);

    // pulse_val is logically 12 bits
    static int pulse_val = 0;  // 12 bit (0 -> 1)
    static int pulse_loc = 0;  // Has 8 frac bits (0 -> WING_CNT)
    pulse_val += PULSE_MIN + ((thr * PULSE_SCALE) >> 8);
    if (pulse_val < 0xFFF) {
      int val = work[2] + (pulse_val >> 4);
      work[2] = (val > 0xFF) ? 0xFF : val;  // center of pulse
      val = work[1] + (pulse_val >> 6);
      work[1] = (val > 0xFF) ? 0xFF : val;
      val = work[3] + (pulse_val >> 6);
      work[3] = (val > 0xFF) ? 0xFF : val;
      val = work[0] + (pulse_val >> 8);
      work[0] = (val > 0xFF) ? 0xFF : val;
      val = work[4] + (pulse_val >> 8);
      work[4] = (val > 0xFF) ? 0xFF : val;
    } else {
      pulse_val = 0xFFF;
      int prev_pulse_idx = ((pulse_loc + (1 << 7)) >> 8);
      pulse_loc += PULSE_MIN + ((thr * SPEED_SCALE + (1 << 7)) >> 8);
      int pulse_idx = ((pulse_loc + (1 << 7)) >> 8);
      for (int i = prev_pulse_idx; i <= pulse_idx + 2; ++i) {
	work[i] = 0xFF;
      }
      int val = work[pulse_idx + 3] + (0xFFF >> 6);
      work[pulse_idx + 3] = (val > 0xFF) ? 0xFF : val;
      val = work[pulse_idx + 4] + (0xFFF >> 8);
      work[pulse_idx + 4] = (val > 0xFF) ? 0xFF : val;
      if (pulse_idx > WING_CNT - 1) {
	pulse_loc = 0;
	pulse_val = 0;
	work[WING_CNT] = 0xFF; // Flash wing tips white...
	pwm_scale_ = 0x20; // busrt pwm lights.
      }
    }

    led::RGBW ltip = led::wclr::red;
    ltip.wht = work[WING_CNT];
    SetLeftNav(ltip);
    led::RGBW rtip = led::wclr::green;
    rtip.wht = work[WING_CNT];
    SetRightNav(rtip);
    for (int i=0; i<WING_CNT; i++) {
      led::RGBW clr = led::Lookup5(heat, work[i]);
      leds[LWING_IDX + i] = clr;
      leds[RWING_IDX + i] = clr;
    }
  }

  void Trippy(int thr) {
    const u32_t SPEED_SCALE = 16;  // 4 bit value
    const i16_t SPEED_MIN   = 40; 
    const u32_t HUE_SCALE   =  4;  // 5 bit value
    const u32_t HUE_MIN     =  2;  // 5 bit value
  
    static int pulse_hue = 0;
    pulse_hue += HUE_MIN + ((thr * HUE_SCALE + (1 << 7)) >> 8);
    const led::RGB pix = led::HsvToRgb(led::HSV(pulse_hue>>6, 0xFF, 0xFF));

    static int pulse_loc = 0;  // Has 8 bits frac (0 -> WING_CNT)
    u8_t prev_pulse_idx = ((pulse_loc + (1 << 7)) >> 8);
    pulse_loc += SPEED_MIN + ((thr * SPEED_SCALE + (1 << 7)) >> 8);
    u8_t pulse_idx = ((pulse_loc + (1 << 7)) >> 8);
    if (pulse_idx >= WING_CNT) {  // wrap the pulse back to start.
      led::Fill(leds + LWING_IDX + prev_pulse_idx,
		WING_CNT - prev_pulse_idx, pix);
      led::Fill(leds + RWING_IDX + prev_pulse_idx,
		WING_CNT - prev_pulse_idx, pix);
      prev_pulse_idx = 0;
      pulse_loc -= (WING_CNT << 8);
      pulse_idx -= WING_CNT;
    }
    led::Fill(leds + LWING_IDX + prev_pulse_idx,
	      pulse_idx - prev_pulse_idx + 1, pix);
    led::Fill(leds + RWING_IDX + prev_pulse_idx,
	      pulse_idx - prev_pulse_idx + 1, pix);
  }

protected:
  u8_t curr_level_;  // Brightness level 0 -> off, 1 -> 12%, 2 -> 100%
  u8_t curr_scale_;  // Scale value derived from curr_level_
  u8_t curr_mode_;   // Modes, 0 -> static, 1 -> pulse, 2 -> ???
  u8_t pwm_scale_;   // 4.4 FP
  u8_t tip_flash_idx_;
  Pwm* pwm_;
  led::RGBW leds[LED_CNT];
  u8_t work[LED_CNT];
};

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/4, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_HI(SBUS);
  DBG_LEVEL_HI(APP);
  DBG_MD(APP, ("Hello World\n"));

  SBus sbus(&Serial::usart1, /*invert=*/true);

  // Configure LED pin as output (low, not inverted, not pullup)
  PinId(LED_PIN).SetOutput(0);
  // Blnky Led on board
  PinId(PIN_F2).SetOutput(0);
  
  Pwm pwm(PORT_D, 400);
  pwm.Set(LWING_PWM, 0);
  pwm.Enable(LWING_PWM);
  pwm.Set(RWING_PWM, 0);
  pwm.Enable(RWING_PWM);

  Lights lights(&pwm);
  lights.UpdateMode(0, 0, 0, 0);
  // Enable interrupts program execution really begins.
  sei();
  
  u8_t update_4 = 0;  // ~1/64 sec
  u8_t update_8 = 0;  // ~1/4 sec
  u8_t update_9 = 0;  // ~1 sec
  while (true) {
    const i16_t now = FastMs();
    
    if (sbus.Run()) {
      u8_t level = SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_LEVEL_CH));
      u8_t mode = SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_MODE_CH));
      int brt = sbus.GetChannel(LIGHT_BRIGHT_CH);
      int thr = sbus.GetChannel(LIGHT_THROTTLE_CH);
      // Run returns true of a new frame of channel data was recieved
      lights.UpdateMode(level, mode, brt, thr);
    }

    const u8_t now_8 = now >> 8; // 1/4 sec (1024 / 256)
    if (now_8 != update_8) {
      update_8 = now_8;
      PORTF.OUTTGL = (1 << 2);  // Pin PF2 toggle, simple heartbeat.
    }

    const u8_t now_4 = now >> 4;  // roughly 60fps.
    if (update_4 != now_4) {
      update_4 = now_4;
      lights.Update(sbus.GetChannel(LIGHT_THROTTLE_CH));
    }

    u8_t now_9 = now >> 9;  // ~.5/sec, print debug info..
    if (now_9 != update_9) {
      update_9 = now_9;
      sbus.Dump();
    }      
  }
}
