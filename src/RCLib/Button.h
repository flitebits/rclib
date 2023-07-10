// Copyright 2023 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "IntTypes.h"
#include "Pins.h"

#ifndef _BUTTON_H_
#define _BUTTON_H_

class Button {
 public:
 Button(PinIdEnum pin) :
  pin_(pin), state_(0) {
    pin_.SetInput(/*inverted=*/false, /*pullup=*/true);
  }

  bool Check() {
    state_ = (state_ << 1) | (pin_.in() ? 0 : 1);
    // To debounce we require the same reading 4x.
    // If the 5th oldest reading didn't agree then we just changed state.
    if ((state_ & 0xF) == 0xF && ((state_ & 0x10) == 0)) return true;
    if ((state_ & 0xF) == 0x0 && ((state_ & 0x10) != 0)) return true;
    return false;
  }
  bool Down() const {
    return (state_ & 0xF) == 0xF; // has it been down the last 4 samples.
  }

 private:
  PinId pin_;
  u8_t state_;
};

#endif  // _BUTTON_H_
