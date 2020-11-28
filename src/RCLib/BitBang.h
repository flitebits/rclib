// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _BITBANG_H_
#define _BITBANG_H_

#include "Pins.h"

class BitBang {
 public:
  BitBang();
  BitBang(PinGroupId pin_group, u8_t pins);

  void AddPin(u8_t pin);
  void SetPort(PinGroupId pin_group, u8_t pins = 0x00);
  
  void SendWS2812(PinId pin, void* ptr, u16_t len, u8_t scale);
  void SendDShot600(u8_t* dshot_data, u8_t dshot_mask);

  // protected:
  void SendWS2812Full(PinId pin, void* ptr, u16_t len);
  void DShot600WS2812(
    u8_t *dshot, u8_t dshot_mask, u8_t led1, u8_t led2, u8_t led_mask);

 private:
  PORT_t* port_;
  bool ws2812_in_progress;
  u8_t dshot_mask;
  u8_t *dshot_data;
};

#endif  // _BITBANG_H_
