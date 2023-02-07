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

} // anonymous namespace

void Pca9685::Init(bool totem) {
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
