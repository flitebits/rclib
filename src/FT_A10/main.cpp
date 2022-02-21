// Copyright 2021 Thomas DeWeesef
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
#include "Rand.h"
#include "SBus.h"
#include "Serial.h"
#include "WS2812.h"

using led::bscale8;
using led::RGBW;
using led::RGB;
using led::saw8;

#define CTRL (true)

#define ENGF_CNT (19)
#define ENGB_CNT (12)

#define TEMP_CNT (ENGF_CNT)

#define ENG_ALL_CNT (2 * (ENGF_CNT + ENGB_CNT))
#define TAIL_F_CNT (10)
#define TAIL_T_CNT (6)
#define TAIL_ALL_CNT (2 * (TAIL_F_CNT + TAIL_T_CNT))

#define LED_PIN (PIN_C0)

#define LIGHT_LEVEL_CH (11)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH (12)  // Sets Lights Mode (solid, spin, glow)
#define LIGHT_BRIGHT_CH (13)  // Adjust leading edge light brightness.
#define LIGHT_THROTTLE_CH (14)  // Adjusts lights speed

#define NUM_PWM (6)
#define LTIP_CH (0)
#define RTIP_CH (1)
#define LND_CH (2)
#define BACK_CH (3)

static RGBW eng_grad[5] = { RGBW(0x40, 0x00, 0x00, 0x00),
			    RGBW(0xFF, 0x20, 0x00, 0x00),
			    RGBW(0xFF, 0x80, 0x00, 0x00),
			    RGBW(0xFF, 0xFF, 0x80, 0x20),
			    RGBW(0x00, 0x00, 0xFF, 0x40) };

u8_t log_map[17] = {1, 2, 4, 8, 16, 32, 64, 128, 255};
u8_t logify(u8_t val) {
  u16_t lo = val & 0x1F;
  u8_t hi = val >> 5;
  int ret = ((u16_t(log_map[hi]) << 5) +
	     (log_map[hi + 1] - log_map[hi]) * lo);
  return (ret + (1<<4)) >> 5;
}

u8_t clip8(u16_t v) {
  if (v > 255) return 255;
  return v;
}

class Lights {
public:
  Lights(Pwm* pwm) :
    pwm_(pwm), mode_(0xFF),
    brt_(0), solid_brt_(0), spd_(0),
    lb_eng_ptr_(&eng_[0]),
    lf_eng_ptr_(&eng_[ENGB_CNT]),
    rb_eng_ptr_(&eng_[ENGB_CNT + ENGF_CNT]),
    rf_eng_ptr_(&eng_[ENGB_CNT + ENGF_CNT + ENGB_CNT]),
    ltail_ptr_(&tail_[0]),
    rtail_ptr_(&tail_[TAIL_F_CNT + TAIL_T_CNT])
  {
    UpdateMode(/*now=*/0, /*mode=*/0, /*lvl=*/0, /*sbrt=*/0, /*thr=*/0);
    for (int i = 0; i < NUM_PWM; ++i) {
      pwm_val_[i] = 0;
      pwm->Set(i, 0);
      pwm->Enable(i);
    }
  }

  void PushLeds() {
    SendWS2812(LED_PIN, eng_, sizeof(eng_) + sizeof(tail_), 0xFF);
  }
  void PushPwm() {
    for (int i = 0; i < NUM_PWM; ++i) {
      pwm_->Set(i, pwm_val_[i]);
    }
  }
  void Push() {
    PushLeds();
    PushPwm();
  }

  // Solid mode is wing and tail tips on at solid_brt_.
  // Engine, tail front all white at brt_. Wing back, landing pwm also brt_.
  void SolidMode() {
    led::Fill(eng_, ENG_ALL_CNT, RGBW(brt_));
  }

  void UpdateLedFromTemp(u16_t *tf, u16_t *tb, RGBW* lf, RGBW* lb) {
    // Now dump temps into led array...
    for (u8_t i = 0; i < (ENGB_CNT / 2); ++i) {
      u8_t b = 3 * i;
      lf[b + 0] = Lookup5(eng_grad, clip8(tf[b + 0] >> 6));
      lf[b + 1] = Lookup5(eng_grad, clip8(tf[b + 1] >> 6));
      lf[b + 2] = Lookup5(eng_grad, clip8(tf[b + 2] >> 6));

      // do a 3:2 downsample for the back ring.
      const u8_t b0_t = clip8((tb[b + 0] * 3 + tb[b + 1]) >> 8);
      lb[2 * i + 0] = Lookup5(eng_grad, b0_t);
      const u8_t b1_t = clip8((tb[b + 2] * 3 + tb[b + 1]) >> 8);
      lb[2 * i + 1] = Lookup5(eng_grad, b1_t);
    }
    lf[ENGF_CNT - 1] = Lookup5(eng_grad, clip8(tf[ENGF_CNT - 1] >> 6));
    for (u8_t i = 0; i < ENGB_CNT; ++i) {
      led::Fade(lb + i, solid_brt_);
    }
    for (u8_t i = 0; i < ENGF_CNT; ++i) {
      led::Fade(lf + i, solid_brt_);
    }
  }
  
  // In throttle mode the engine color ramps with throttle.
  // It tracks a 'heat' value for each front and back led, the trottle sets
  // the base temp and then random 'hot spots' are added occationally.
  //
  // The code uses the same number of temp values for both
  // front and back even though there are fewer leds in the back.
  // Before updating the back leds it will resize down.
  //
  // Tempurature is stored as a 8.6 fixed point number so we can
  // accumulate small numbers and not always round down.
  //
  // The Throttle set the target temp of the back ring and the front
  // ring is 1/2 that temp.
  //
  // Throttle comes in as 11 bit value.
  void ThrottleMode(u16_t throttle, u16_t *tf, u16_t *tb) {
    // Map 100% throttle to 100% of max.
    // 100% brightness would be throttle * 8 or shifted by 3,
    // we want three quarters of that so multiply by 3 and
    // divide by 4 (or shifted by 2). So we multiply by 3 and
    // shift by 1.
    u16_t base_temp = throttle << 3;

    // Now blend that with all back leds, we blend 7:1
    const u16_t bbt_adden = base_temp >> 4;
    for (int i = 0; i < TEMP_CNT; ++i) {
      tb[i] = tb[i] - (tb[i] >> 4) + bbt_adden;
    }

    // Do the same with the front but cut temp by 4.
    const u16_t fbt_adden = base_temp >> 6;
    for (int i = 0; i < TEMP_CNT; ++i) {
      tf[i] = tf[i] - (tf[i] >> 4) + fbt_adden;
    }

    // Now we want to blur temps a bit, the kernel looks like this:
    //  2 | 27 | 2  
    //  0 |  1 | 0
    // It sums to 32 (or 5 bits).
    // copy the first value to the 'past end' place so we get the
    // wrap around value for the last entry (also avoids needing special case).
    tf[TEMP_CNT] = tf[0];
    tb[TEMP_CNT] = tb[0];
    u16_t prevf = tf[TEMP_CNT - 1];
    u16_t prevb = tb[TEMP_CNT - 1];
    for (int i = 0; i < TEMP_CNT; ++i) {
      u16_t vf = tf[i];
      vf = vf - (vf >> 3);  // This is 26 * vf / 32

      u16_t vb = tb[i];
      vb = vb - (vb >> 3);

      u16_t f_out = vf + ((prevf + tf[i + 1]) >> 5) + (vb >> 4);
      u16_t b_out = vb + ((prevb + tb[i + 1]) >> 5) + (vf >> 4);
      prevf = tf[i];
      prevb = tb[i];
      tf[i] = f_out;
      tb[i] = b_out;
    }

    // Now possibly add some hot spots to the back ring.
    u8_t n = 4;
    for (u8_t i =0; i < n; ++i) {
      // This technically could go one to far but tb is oversized so
      // that is ok. If you don't do this then the last entry doesn't
      // get selected very much (should really make random8 more uniform).
      u8_t idx = random8(TEMP_CNT);
      u16_t v = tb[idx] + (base_temp >> 1);
      tb[idx] = (v > ((1 << 14) - 1)) ? ((1 << 14) - 1) : v;

      idx = random8(TEMP_CNT);
      v = tf[idx] + (base_temp >> 2);
      tf[idx] = (v > ((1 << 14) - 1)) ? ((1 << 14) - 1) : v;
    }

  }

  void PulseMode(u16_t now, u16_t throttle, u16_t *tf, u16_t *tb) {
    u8_t bpm8 = logify((throttle >> 4) + 127);
    u16_t prev_saw_at_bpm8 = prev_time_ * bpm8;
    u16_t curr_saw_at_bpm8 = now * bpm8;
    u16_t curr_saw = prev_saw_ + (curr_saw_at_bpm8 - prev_saw_at_bpm8);
    u16_t sawb = curr_saw >> 2;
    u32_t sawf = (sawb * (u32_t)sawb) >> 14;
    for (int i = 0; i < TEMP_CNT; ++i) {
      tf[i] = sawf;
      tb[i] = sawb;
    }
    prev_saw_ = curr_saw;
  }
  
  void UpdateBright(u8_t lvl, u16_t sbrt) {
    brt_ = 0;
    if (sbrt > 16) {
      sbrt = sbrt >> 3;
      brt_ = (sbrt > 255) ? 255 : sbrt;
    }

    switch (lvl) {
    case 0: brt_ = solid_brt_ = 0; break;
    case 1: solid_brt_ = 0x40; brt_ = brt_ >> 3; break;
    case 2: solid_brt_ = 0xFF; break;
    }
    DBG_HI(APP, ("Update Brt: %d Spn: %d\n", brt_, solid_brt_));

    led::Fill(ltail_ptr_, TAIL_F_CNT, RGB(brt_));
    led::Fill(rtail_ptr_, TAIL_F_CNT, RGB(brt_));
    led::Fill(ltail_ptr_ + TAIL_F_CNT, TAIL_T_CNT, RGB(solid_brt_, 0, 0));
    led::Fill(rtail_ptr_ + TAIL_F_CNT, TAIL_T_CNT, RGB(0, solid_brt_, 0));
    pwm_val_[LTIP_CH] = solid_brt_;
    pwm_val_[RTIP_CH] = solid_brt_;
    pwm_val_[LND_CH] = brt_;
    pwm_val_[BACK_CH] = brt_;
  }

  void UpdateMode(u16_t now, u8_t mode, u8_t lvl, u16_t sbrt, u16_t thr) {
    UpdateBright(lvl, sbrt);
    thr_ = thr;
    if (lvl == 0 || mode == 0) {
      SolidMode();
      Push();
      prev_time_ = now;
      return;
    }

    // Throttle is 11 bits, spd has 5 fractional bits, so this makes the
    // speed multiplier go from 0 -> 4x with throttle.
    spd_ = (thr + (1 <<3)) >> 4;
    if (spd_ < 0x07) spd_ = 0x07;
    if (mode_ != mode) {
      mode_ = mode;
      Update(now);
    }
  }

  void Update(u16_t now) {
    switch (mode_) {
    case 0:
      if ((((u8_t)now) & 0xFF) != 0) return;
      SolidMode();
      break;
    case 1:
      ThrottleMode(thr_, lf_eng_temp_, lb_eng_temp_);
      UpdateLedFromTemp(lf_eng_temp_, lb_eng_temp_, lf_eng_ptr_, lb_eng_ptr_);
      ThrottleMode(thr_, rf_eng_temp_, rb_eng_temp_);
      UpdateLedFromTemp(rf_eng_temp_, rb_eng_temp_, rf_eng_ptr_, rb_eng_ptr_);
      break;
    case 2:
      PulseMode(now, thr_, lf_eng_temp_, lb_eng_temp_);
      UpdateLedFromTemp(lf_eng_temp_, lb_eng_temp_, lf_eng_ptr_, lb_eng_ptr_);
      PulseMode(now, thr_, rf_eng_temp_, rb_eng_temp_);
      UpdateLedFromTemp(rf_eng_temp_, rb_eng_temp_, rf_eng_ptr_, rb_eng_ptr_);
      break;
    }
    Push();
    prev_time_ = now;
  }

  Pwm* const pwm_;
  u8_t mode_;
  u8_t brt_;
  u8_t solid_brt_;
  u16_t thr_;
  u8_t spd_;
  u8_t pwm_val_[6];
  u16_t prev_saw_;
  u16_t prev_time_;
  
  u16_t lf_eng_temp_[TEMP_CNT + 1];
  u16_t lb_eng_temp_[TEMP_CNT + 1];
  u16_t rf_eng_temp_[TEMP_CNT + 1];
  u16_t rb_eng_temp_[TEMP_CNT + 1];
  
  led::RGBW eng_[ENG_ALL_CNT];
  led::RGB  tail_[TAIL_ALL_CNT];
  led::RGBW* lb_eng_ptr_;
  led::RGBW* lf_eng_ptr_;
  led::RGBW* rb_eng_ptr_;
  led::RGBW* rf_eng_ptr_;
  led::RGB*  ltail_ptr_;
  led::RGB*  rtail_ptr_;
};

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  Pwm pwm(PORT_D, 400);
  Lights lights(&pwm);

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_HI(APP);
  DBG_LEVEL_MD(SBUS);
  SBus sbus(&Serial::usart1, /*invert=*/true);

  PinId led_pin(LED_PIN);
  led_pin.SetOutput();
  PinId blink_pin(PIN_A2);
  blink_pin.SetOutput();
 
  sei();
  DBG_MD(APP, ("Hello World: Test\n"));
  lights.PushLeds();

  lights.UpdateMode(0, /*mode=*/0, /*level=*/0, /*brt=*/0, /*thr=*/0);

  u8_t update_9 = 0;
  u8_t update_6 = 0;
  while (1) {
    const u16_t now = FastTimeMs();
    if (sbus.Run()) {
	    // Run returns true of a new frame of channel data was received
	    u8_t level = SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_LEVEL_CH));
	    u8_t mode = SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_MODE_CH));
	    int brt = sbus.GetChannel(LIGHT_BRIGHT_CH);
	    int thr = sbus.GetChannel(LIGHT_THROTTLE_CH);
	    lights.UpdateMode(now, mode, level, brt, thr);
	    DBG_HI(APP, ("SBus: L:%d M:%d B:%d T:%d\n", level, mode, brt, thr));
    }

    const u8_t now_6 = now >> 6;
    if (now_6 == update_6) continue;
    update_6 = now_6;
    lights.Update(now);

    const u8_t now_9 = now >> 9;
    if (now_9 == update_9) continue;
    update_9 = now_9;
    blink_pin.toggle();
    // sbus.Dump();
  }
}
