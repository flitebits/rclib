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

#define WING_CNT (24)
#define TIP_CNT (8)
#define LWING_CNT (WING_CNT)
#define LTIP_CNT (TIP_CNT)
#define RWING_CNT (WING_CNT)
#define RTIP_CNT (TIP_CNT)
#define TAIL_CNT (8)

#define LED_SIZE (3 * (2 * TIP_CNT + TAIL_CNT) + 4 * (2 * WING_CNT))
#define RGB_LED_CNT ((LED_SIZE + 2)/ 3)
#define RGBW_LED_CNT ((LED_SIZE + 3)/ 4)

#define NUM_PWM (6)
#define LLAND_PWM (0)
#define RLAND_PWM (1)
#define LWFT_PWM (2)
#define RWFT_PWM (3)
#define LWBK_PWM (4)
#define RWBK_PWM (5)

#define LIGHT_LEVEL_CH (5)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH (6)  // Sets Lights Mode
#define LIGHT_BRIGHT_CH (7)  // Adjust leading edge light brightness.
#define LIGHT_THROTTLE_CH (8)

#define LED_PIN (PIN_C2)

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
    trip_loc_(0),
    trip_hue_(0),
    pwm_(pwm) {
    OffMode();
  }
  void SetLeftNav(const led::RGB& color) {
    led::Fill(ltip, LTIP_CNT, color); 
  }
  void SetRightNav(const led::RGB& color) {
    led::Fill(rtip, LTIP_CNT, color); 
  }
  void SetWing(const led::RGBW& color, u8_t sides = BOTH_SIDES) {
    if (sides & LEFT_SIDE) {
      led::Fill(lwing, LWING_CNT, color);
    }
    if (sides & RIGHT_SIDE) {
      led::Fill(rwing, RWING_CNT, color);
    }
  }
  void SetTail(const led::RGB& color) {
    led::Fill(tail, TAIL_CNT, color);
  }

  void OffMode() {
    led::Fill(lwing, RGBW_LED_CNT, led::wclr::black);
    for (int i = 0; i < 6; ++i) {
      pwm_->Set(i, 0);
    }
  }

  void UpdateMode(u8_t new_level, u8_t new_mode, int brt, int thr) {
    randomSeedMix(RTC.CNT);
    if (new_level == 0){
      if (curr_level_ == 0) return;
      curr_level_ = new_level;
      curr_mode_ = 255;
      curr_scale_ = 0;
      OffMode();
      PushLeds();
      return;
    }

    brt = (brt < 32) ? 0 : (brt >> 3); // convert 11 bits to 8 bits.
    if (brt > 255) brt = 255;
    if (new_level == 1) brt = brt >> 2;
    pwm_->Set(LLAND_PWM, brt);
    pwm_->Set(RLAND_PWM, brt);

    curr_scale_ = (new_level == 1) ? 0x40 : 0xFF;
    switch (new_mode) {
    case 0: brt = curr_scale_; break;
    case 1: {
      int cs = curr_scale_ >> 2;
      if (brt < cs) brt = cs;
      brt = (brt * (int)pwm_scale_ + (1<<3)) >> 4;
      if (brt > 255) brt = 255;
      break;
    }
    case 2: break;
    }
    pwm_->Set(LWFT_PWM, brt);
    pwm_->Set(RWFT_PWM, brt);
    pwm_->Set(LWBK_PWM, brt);
    pwm_->Set(RWBK_PWM, brt);

    if (curr_mode_ == new_mode && curr_level_ == new_level) return;
    switch (new_mode) {
    case 0:
      SetLeftNav(led::clr::red);
      SetRightNav(led::clr::green);
      SetWing(led::wclr::white, BOTH_SIDES);
      SetTail(led::clr::white);
      brt = curr_scale_;
      break;
    case 1: {
      SetLeftNav(led::clr::red);
      SetRightNav(led::clr::green);
      SetTail(led::clr::white);
      if (curr_mode_ != new_mode) {
        SetWing(led::wclr::black);
	memset(work, 0, sizeof(work));
      }      
      int cs = curr_scale_ >> 2;
      if (brt < cs) brt = cs;
      brt = (brt * (int)pwm_scale_ + (1<<3)) >> 4;
      if (brt > 255) brt = 255;
      break;
    }
    case 2:
      SetLeftNav(led::clr::red);
      SetRightNav(led::clr::green);
      SetTail(led::clr::white);
      if (curr_mode_ != new_mode) {
        SetWing(led::wclr::black);
	trip_loc_ = 0;
	trip_hue_ = 0;
      }
      break;
    }
    pwm_->Set(LWFT_PWM, brt);
    pwm_->Set(RWFT_PWM, brt);
    pwm_->Set(LWBK_PWM, brt);
    pwm_->Set(RWBK_PWM, brt);
    curr_level_ = new_level;
    curr_mode_ = new_mode;
  }

  void Update(int brt, int thr) {
    switch (curr_mode_) {
    default:
    case 0: break;
    case 1: PulseMode(brt, thr); break;
    case 2: Trippy(thr); break;
    }
    PushLeds();
  }
  
  void PushLeds() {
    SendWS2812(PinId(LED_PIN), lwing, LED_SIZE, curr_scale_);
  }


  void PulseMode(int brt, int thr) {
    const u32_t SPEED_SCALE = 32;  // 4 bit value
    const u32_t PULSE_SCALE = 32;  // 4 bit value
    const i16_t PULSE_MIN   = 32; 
  
    // "Cool" Wing leds
    for (u8_t i=0; i < sizeof(work); i++) {
      work[i] = (work[i] * int(0xF0)) >> 8;
    }
    // Cool pwm boost...
    pwm_scale_ = pwm_scale_ + (((0x10 - pwm_scale_) * 0x10) >> 8);

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
    led::RGB tip_pulse(work[WING_CNT], work[WING_CNT], work[WING_CNT]);
    led::RGB ltip = led::clr::red;
    Max(&ltip, tip_pulse);
    SetLeftNav(ltip);
    led::RGB rtip = led::clr::green;
    Max(&rtip, tip_pulse);
    SetRightNav(rtip);
    for (int i=0; i<WING_CNT; i++) {
      led::RGBW clr = led::Lookup5(heat, work[i]);
      lwing[i] = clr;
      rwing[i] = clr;
    }

    int cs = curr_scale_ >> 2;
    brt = (brt < 32) ? 0 : (brt >> 3); // convert 11 bits to 8 bits.
    if (curr_level_ == 1) brt = brt >> 2;
    if (brt < cs) brt = cs;
    brt = (brt * (int)pwm_scale_ + (1<<3)) >> 4;
    if (brt > 255) brt = 255;
    pwm_->Set(LWFT_PWM, brt);
    pwm_->Set(RWFT_PWM, brt);
    pwm_->Set(LWBK_PWM, brt);
    pwm_->Set(RWBK_PWM, brt);
  }

  void Trippy(int thr) {
    const u32_t SPEED_SCALE = 16;  // 4 bit value
    const i16_t SPEED_MIN   = 40; 
    const u32_t HUE_SCALE   =  4;  // 5 bit value
    const u32_t HUE_MIN     =  2;  // 5 bit value
  
    trip_hue_ += HUE_MIN + ((thr * HUE_SCALE + (1 << 7)) >> 8);
    const led::RGB pix = led::HsvToRgb(led::HSV(trip_hue_ >> 6, 0xFF, 0xFF));

    u8_t prev_trip_idx = trip_loc_ >> 8;
    trip_loc_ += SPEED_MIN + ((thr * SPEED_SCALE + (1 << 7)) >> 8);
    u8_t trip_idx = trip_loc_ >> 8;
    if (trip_idx >= WING_CNT) {  // wrap the pulse back to start.
      led::Fill(lwing + prev_trip_idx,
		WING_CNT - prev_trip_idx, pix);
      led::Fill(rwing + prev_trip_idx,
		WING_CNT - prev_trip_idx, pix);
      trip_loc_ -= (WING_CNT << 8);
      prev_trip_idx = 0;
      trip_idx = trip_loc_ >> 8;
    }
    led::Fill(lwing + prev_trip_idx,
	      trip_idx - prev_trip_idx + 1, pix);
    led::Fill(rwing + prev_trip_idx,
	      trip_idx - prev_trip_idx + 1, pix);
  }

protected:
  u8_t curr_level_;  // Brightness level 0 -> off, 1 -> 12%, 2 -> 100%
  u8_t curr_scale_;  // Scale value derived from curr_level_
  u8_t curr_mode_;   // Modes, 0 -> static, 1 -> pulse, 2 -> ???
  u8_t pwm_scale_;   // 4.4 FP
  u8_t tip_flash_idx_;
  u16_t trip_loc_;
  u16_t trip_hue_;
  Pwm* pwm_;
  led::RGBW lwing[LWING_CNT];
  led::RGB ltip[LTIP_CNT];
  led::RGBW rwing[RWING_CNT];
  led::RGB rtip[RTIP_CNT];
  led::RGB tail[TAIL_CNT];
  led::RGB padding;
  u8_t work[WING_CNT + 1];
};

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
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
  for (int i = 0; i < NUM_PWM; ++i) {
    pwm.Set(i, 0);
    pwm.Enable(i);
  }

  Lights lights(&pwm);
  lights.UpdateMode(0, 0, 0, 0);

  // Enable interrupts program execution really begins.
  sei();
  
  u8_t update_4 = 0;  // ~1/64 sec
  u8_t update_8 = 0;  // ~1/4 sec
  u8_t update_9 = 0;  // ~1/2 sec
  while (true) {
    const u16_t now = FastMs();
    
    if (sbus.Run()) {
      u8_t level = SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_LEVEL_CH));
      u8_t mode = SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_MODE_CH));
      int brt = sbus.GetChannel(LIGHT_BRIGHT_CH);
      int thr = sbus.GetChannel(LIGHT_THROTTLE_CH);
      // Run returns true of a new frame of channel data was recieved
      lights.UpdateMode(level, mode, brt, thr);
    }

    const u8_t now_4 = now >> 4;  // roughly 60fps.
    if (update_4 == now_4) continue;
    update_4 = now_4;

    lights.Update(sbus.GetChannel(LIGHT_BRIGHT_CH),
		  sbus.GetChannel(LIGHT_THROTTLE_CH));

    const u8_t now_8 = now >> 8; // 1/4 sec (1024 / 256)
    if (now_8 == update_8) continue;
    update_8 = now_8;
    PORTF.OUTTGL = (1 << 2);  // Pin PF2 toggle, simple heartbeat.

    u8_t now_9 = now >> 9;  // ~.5/sec, print debug info..
    if (now_9 == update_9) continue;
    update_9 = now_9;
    sbus.Dump();
  }
}
