// Copyright 2023 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "IntTypes.h"

class Pca9685 {
 public:
 Pca9685(u8_t i2c_addr, u8_t num_led) :
  i2c_addr_(i2c_addr), start_led_idx_(0), num_led_(num_led) { }
 Pca9685(u8_t i2c_addr, u8_t start_led_idx, u8_t num_led) :
  i2c_addr_(i2c_addr), start_led_idx_(start_led_idx), num_led_(num_led) { }

  // When called will configure the PCA9685 control registers for later updates.
  // If totem is true then
  void Init(bool totem);
  void Write();

  // Set duty cycle of led, 0 -> off, 4096 full on.
  void SetLed(u16_t val, u8_t idx);
  void SetLeds(u16_t val, u8_t idx, u8_t len);

  // If true then the start time of every led will be the end time of the prior led.
  // Check if this works or if you get flashing depending on cycle state when updating...
  void LedSequential(bool sequential);

 private:
  const u8_t i2c_addr_;
  u8_t start_led_idx_;
  u8_t num_led_;
  u16_t leds_[16];
  u8_t cmd_buf_[1 + 4 * 16];
};
