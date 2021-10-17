// Copyright 2021 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "Boot.h"
#include "Dbg.h"
#include "IntTypes.h"
#include "RtcTime.h"

#include "leds/Clr.h"
#include "leds/FPMath.h"
#include "leds/Pixel.h"
#include "leds/Rgb.h"

#include "Pins.h"
#include "Pwm.h"
#include "SBus.h"
#include "Serial.h"
#include "WS2812.h"

using led::bscale8;
using led::RGBW;

#define CTRL (true)

#define WING_CNT (28)
#define LOST_CNT ( 4)
#define SPON_CNT (20)
#define ALL_CNT (LOST_CNT + 2 * (WING_CNT + SPON_CNT))
#define LED_PIN (PIN_C2)

#define LIGHT_LEVEL_CH (12)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH (13)  // Sets Lights Mode (solid, spin, glow)
#define LIGHT_BRIGHT_CH (14)  // Adjust leading edge light brightness.
#define LIGHT_THROTTLE_CH (15)  // Adjusts lights speed

// The PWM cannels are top outside, top inside, bottom inside, bottom
// outside on both hulls so they are wired so the same sequence of
// animation will be mirrored.
//           Port hull       Starbord hull (ctrl)
//             0  1           1  0
//             3  2           2  3
#define NUM_PWM (4)  // Only 4 are used

class Lights {
public:
  Lights(Pwm* pwm) :
    pwm_(pwm), mode_(0xFF), brt_(0), spon_brt_(0), spd_(0),
    led_move_freq_(31), led_move_time_(0) {
    UpdateMode(/*now=*/0, /*mode=*/0, /*lvl=*/0, /*sbrt=*/0, /*thr=*/0);
    for (int i = 0; i < NUM_PWM; ++i) {
      pwm->Enable(i);
    }
  }

  void PushLeds() {
    SendWS2812(PinId(LED_PIN), rwing_, ALL_CNT, 0xFF);
  }
  void PushPwm() {
    for (int i = 0; i < NUM_PWM; ++i) {
      pwm_->Set(i, bscale8(pwm_val_[i], brt_));
    }
  }
  void Push() {
    PushLeds();
    PushPwm();
  }

  // cycle_pos is an 8bit fraction of where it should be in the cycle
  void CalcGlowPwm(u8_t cycle_pos) {
    if (cycle_pos & (1<<7)) {
      // For the second half of the cycle count down.
      cycle_pos = (1 << 8) - (cycle_pos + 1);
    }
    memset(pwm_val_, cycle_pos << 1, sizeof(pwm_val_));
  }


  // cycle_pos is an 8bit fraction of where it should be in the cycle
  // brt is a multiplier all fraction (0xFF means full brightness)
  // This accumulates the part of the curve shaped like /-\_ across the
  // four lights.
  //         __ __
  // |    | /  |  \ |    |
  // |    |/   |   \|    |
  // |   /|    |    |\   |
  // |__/ |    |    | \__|
  void CalcRotPwm(u8_t cycle_pos) {
    // We are treating cycle_pos as a 2.6 fixed point where the
    // top two bits indicate which led to 'start from'.
    u8_t idx = (cycle_pos >> 6) & 0x3;
    u8_t f = cycle_pos & ((1 << 6) - 1);  // fraction into idx
    u8_t r = ((1 << 6) - 1) - f;                // remainder of idx

    // for first slope up, it's 45% so half a box of size remainder on
    // each side. remainder has 6bits fraction so the result has 12 bits
    // fraction and we want to keep 8, but we shift 5 out to include the
    // divide by 2.
    pwm_val_[idx] = ((r * (u16_t)r) + (1 << 4)) >> 5;
    idx = (idx + 1) & 0x3;
    // The remainder of the up slope has left side 'remainder' high and right
    // side full height and is 'frac' wide. So it's area is:
    // (remainder + 1) * frac / 2.
    pwm_val_[idx] = (((r + (1 << 6)) * f) + (1 << 4)) >> 5;
    // This bin also will have a flat section remainder wide by 1.0
    // So shift r up 2 bits.
    pwm_val_[idx] += (r << 2) | (r >> 4);
    idx = (idx + 1) & 0x3;

    // This bin starts with a flat section frac wide by 1.0
    pwm_val_[idx] = (f << 2) | (f >> 4);
    // Then a downslope that is 1.0 high on the left and frac high on the right
    // by remainder wide.
    pwm_val_[idx] += (((f + (1 << 6)) * r) + (1 << 4)) >> 5;
    idx = (idx + 1) & 0x3;

    // Final bin just has a right triangle frac high by frac wide.
    pwm_val_[idx] = ((f * (u16_t)f) + (1 << 4)) >> 5;
  }

  void SetSpon() {
    led::Fill(lspon_, WING_CNT, RGBW(spon_brt_, 0, 0));
    led::Fill(rspon_, WING_CNT, RGBW(0, spon_brt_, 0));
  }

  // Solid mode is body on, wing edges white, sponsons R/G.
  void SolidMode() {
    led::Fill(lwing_, WING_CNT, RGBW(brt_));
    led::Fill(lspon_, WING_CNT, RGBW(spon_brt_, 0, 0));
    led::Fill(rspon_, WING_CNT, RGBW(0, spon_brt_, 0));
    led::Fill(rwing_, WING_CNT, RGBW(brt_));
    for (int i = 0; i < NUM_PWM; ++i) {
      pwm_->Set(i, brt_);
    }
  }
  
  void ShiftWingLed(u8_t now) {
    while ((u8_t)(now - led_move_time_) >= led_move_freq_) {
      led_move_time_ += led_move_freq_;
      led::RGBW* rptr = rwing_ + WING_CNT - 1;
      led::RGBW* lptr = lwing_;
      for (int i = 0; i < WING_CNT - 1; ++i) {
	rptr[0] = rptr[-1]; --rptr;
	lptr[0] = lptr[1]; ++lptr;
      }
      *rptr = led::RGBW(pwm_val_[3]);
      *lptr = led::RGBW(pwm_val_[3]);
    }
  }

  void UpdateBright(u8_t lvl, u16_t sbrt) {
    brt_ = 0;
    if (sbrt > 32) {
      sbrt = sbrt >> 3;
      brt_ = (sbrt > 255) ? 255 : sbrt;
    }

    switch (lvl) {
    case 0: brt_ = spon_brt_ = 0; break;
    case 1: spon_brt_ = 0x40; brt_ = brt_ >> 3; break;
    case 2: spon_brt_ = 0xFF; break;
    }
  }

  void UpdateMode(u16_t now, u8_t mode, u8_t lvl, u16_t sbrt, u16_t thr) {
    UpdateBright(lvl, sbrt);
    if (lvl == 0) {
      SolidMode(); // Off
      Push();
      return;
    }

    SetSpon();
    // Throttle is 11 bits, spd has 5 fractional bits, so this makes the
    // speed multiplier go from 0 -> 4x with throttle.
    spd_ = (thr + (1 <<3)) >> 4;
    led_move_freq_ = (31 << 5)/spd_;

    if (mode_ != mode || mode == 0) {
      mode_ = mode;
      // We want cycle pos to complete a cycle every second (8bits)
      // 1s = 1024'ms' = 10bits, so shift 2 bits for that, and 5
      // bits for spd for a total of 7.
      u8_t cycle_pos = (((u16_t)now * spd_) + (1 << 6)) >> 7;

      // Perhaps have knob control colorfulness
      switch (mode) {
	// Solid colors
      case 0: SolidMode(); return;
      case 1:
	// Body lights rotate and send pulses down wings.
	led::Fill(rwing_, WING_CNT, led::clr::black);
	led::Fill(lwing_, WING_CNT, led::clr::black);
	CalcRotPwm(cycle_pos);
	break;
      case 2:
	// Body lights glow and send pulses down wings
	led::Fill(rwing_, WING_CNT, led::clr::black);
	led::Fill(lwing_, WING_CNT, led::clr::black);
	CalcGlowPwm(cycle_pos);
	break;
      }
    }
    Push();
  }

  void Update(u16_t now) {
    if (mode_ == 0) return;

    // We want cycle pos to complete a cycle every second (8bits)
    // 1s = 1024'ms' = 10bits, so shift 2 bits for that, and 5
    // bits for spd for a total of 7.
    u8_t cycle_pos = ((now * spd_) + (1 << 6)) >> 7;
    switch (mode_) {
    case 1:
      CalcRotPwm(cycle_pos);
      break;
    case 2:
      CalcGlowPwm(cycle_pos);
      break;
    }
    SetSpon();
    ShiftWingLed(now);
    PushLeds();
  }

  Pwm* const pwm_;
  u8_t mode_;
  u8_t brt_;
  u8_t spon_brt_;
  u8_t spd_;
  u8_t led_move_freq_;
  u8_t led_move_time_;
  u8_t pwm_val_[4];
  led::RGBW rwing_[WING_CNT];
  led::RGBW rspon_[SPON_CNT];
  led::RGBW lspon_[SPON_CNT];
  led::RGBW lost_[LOST_CNT];
  led::RGBW lwing_[WING_CNT];
};
  
u32_t GetNow(u32_t local_ms, Serial* com) {
#if CTRL
  static u8_t update_5 = 0;

  const u32_t now = local_ms;
  const u8_t now_5 = now >> 5;  // Update time every 32ms
  if (now_5 != update_5) {
    update_5 = now_5;
    const u8_t *ptr = (const u8_t*)&now;
    com->WriteByte(ptr[0]);
    com->WriteByte(ptr[1]);
    com->WriteByte(ptr[2]);
    com->WriteByte(ptr[3]);
  }
#else
  static u16_t last_read = 0;
  static u8_t err = false;
  static u32_t ctrl_ms = 0;
  static u32_t base_ms = 0;
  static u8_t word[4];
  static u8_t word_idx = 0;
  while (com->Avail()) {
    ReadInfo info;
    com->Read(&info);
    if (info.err) {
      err = true;
    } else {
      if ((info.time - last_read) > (1 << 9)) {
	// This is the first read in a new round
	word_idx = 0;
      }
      word[word_idx++] = info.data;
    }
    last_read = info.time;
    if (word_idx == 4) {
      if (!err) {
	ctrl_ms = *(u32_t *)&word[0];
	base_ms = local_ms;
      }
      word_idx = 0;
      err = false;
    }
  }
  const u32_t now = ctrl_ms + (local_ms - base_ms);
#endif
 return now;
}

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  Pwm pwm(PORT_D, 400);
  Lights lights(&pwm);

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);

  SBus sbus(&Serial::usart1, /*invert=*/true);
  Serial* com = &Serial::usart3;
  com->Setup(115200, 8, Serial::PARITY_EVEN, 1, /*invert=*/false,
	     /*use_alt_pins=*/false,
	     CTRL ? Serial::MODE_TX : Serial::MODE_RX,
	     /*use_pullup=*/false, /*buffered=*/true);

  PinId led_pin(LED_PIN);
  led_pin.SetOutput();
  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();
 
  sei();
  DBG_MD(APP, ("Hello World: Test\n"));
  lights.PushLeds();

  lights.UpdateMode(0, /*mode=*/1, /*level=*/1, /*brt=*/2000, /*thr=*/1000);

  u8_t update_9 = 0;
  while (1) {
    const u32_t local_ms = FastTimeMs();
    const u32_t now = GetNow(local_ms, com);
    if (sbus.Run()) {
      // Run returns true of a new frame of channel data was recieved
      u8_t level = SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_LEVEL_CH));
      u8_t mode = SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_MODE_CH));
      int brt = sbus.GetChannel(LIGHT_BRIGHT_CH);
      int thr = sbus.GetChannel(LIGHT_THROTTLE_CH);
      level = 1;
      mode = 1;
      brt = 2000;
      thr = 1000;
      lights.UpdateMode(now, mode, level, brt, thr);
    }

    lights.Update(now);

    const u8_t now_9 = now >> 9;
    if (now_9 == update_9) continue;
    update_9 = now_9;
    blink_pin.toggle();
  }
}
