// Copyright 2020 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _PWM_
#define _PWM_

#include "IntTypes.h"
#include "Pins.h"

class Pwm {
public:
  Pwm(PinGroupId port, int freqHz);
  // Index of pwm to enable, 0-5.  This also configures the pin for output.
  // Does not affect the pins associated PINCTRL register.
  void Enable(u8_t idx);
  void Enable(u8_t idx, u8_t val);
  void Disable(u8_t idx);
  // Returns the value that the pwm index is set to.
  u8_t Get(u8_t idx);
  // Set PWM pin's duty cycle to 'val';
  // 0 is off all the time 255 is on all the time.
  void Set(u8_t idx, u8_t val);
  // Set multiple PWM pin's duty cycles.
  // 0 is off all the time 255 is on all the time.
  void Set(u8_t start_idx, u8_t n_val, u8_t* val);
  u8_t NumPwm(){ return 6; }

  PORT_t* port_ptr() { return port_ptr_; }

private:
  PORT_t *port_ptr_;
};

#endif  // _PWM_
