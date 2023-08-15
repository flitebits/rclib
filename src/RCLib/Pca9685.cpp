// Copyright 2023 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "Pca9685.h"

#include "Twi.h"

namespace {
  // This is a power series of (e ^ (log(255) / 16)) with the power going from 0
  // to 16.  This is chosen because the apparent brightness is like loudness a log
  // of the "number of photons" which is what the PWM adjusts.
const u8_t apparent2pwm[18] = {
  0, 1, 2, 3, 5, 7, 10, 13, 17, 23, 33, 45, 64, 90, 128, 180, 255, 255 };
} // anonymous namespace

// This E(log(255)/15) raised to the power 1->15, this gives an exponential curve

u16_t Pca9685::Apparent2Pwm(u8_t apparent) {
  if (apparent == 255) return 0x0FFF;
  if (apparent < 64) {
    return apparent;
  }
  u8_t idx = apparent >> 4;
  u8_t f = (apparent & 0x0F);
  u8_t v0 = apparent2pwm[idx];
  u8_t v1 = apparent2pwm[idx + 1];
  return (v0 << 4) + (u16_t(v1 - v0) * f);
}

void Pca9685::Init(bool totem) {
  memset(leds_, 0, sizeof(leds_));
  u8_t idx = 0;
  cmd_buf_[idx++] = 0x00; // Register: zero (MODE1)
  // MODE1: No ext clk, set register auto inc, disable subcall & all call adr
  cmd_buf_[idx++] = totem ? 0x20 : 0x00;
  // MODE2: OUTDRV=1, led Totem pole, OCH on Stop, not inverted, respect OE.
  cmd_buf_[idx++] = 0x04;
  Twi::twi.MasterSendBytes(i2c_addr_, cmd_buf_, idx);
}

void Pca9685::Write() {
  Twi::twi.WaitForIdle();
  u8_t idx = 0;
  cmd_buf_[idx++] = 6 + (start_led_idx_ * 4);  // First Led register...
  for (int i = start_led_idx_; i < num_led_; ++i) {
    u16_t val = leds_[i];
    if (val == 0) {
      cmd_buf_[idx++] = 0x00;  //  ON: zero
      cmd_buf_[idx++] = 0x00;
      cmd_buf_[idx++] = 0x00;  // OFF: full off
      cmd_buf_[idx++] = 0x10;
    } else if ((val & 0xF000) != 0) {
      cmd_buf_[idx++] = 0x00;  //  ON: Full On
      cmd_buf_[idx++] = 0x10;
      cmd_buf_[idx++] = 0x00;  // OFF: zero
      cmd_buf_[idx++] = 0x00;
    } else {
      cmd_buf_[idx++] = 0x00;  //  ON: zero
      cmd_buf_[idx++] = 0x00;
      cmd_buf_[idx++] = val & 0x0FF;
      cmd_buf_[idx++] = (val >> 8) & 0x0F;
    }
  }
  Twi::twi.MasterSendBytes(i2c_addr_, cmd_buf_, idx);
}

void Pca9685::SetLed(u16_t val, u8_t idx) {
  leds_[idx & 0x0F] = (val > 4095) ? 4096 : val;
}

void Pca9685::SetLeds(u16_t val, u8_t idx, u8_t len) {
  val = (val > 4095) ? 4096 : val;
  const u8_t end = idx + len;
  while (idx < end) {
    leds_[idx & 0x0F] = val;
    ++idx;
  }
}
