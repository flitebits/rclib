// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include "Pins.h"
#include "stddef.h"

namespace {
  PinId analog_pins[16] = {
    PinId(AIN_0), PinId(AIN_1), PinId(AIN_2), PinId(AIN_3),
    PinId(AIN_4), PinId(AIN_5), PinId(AIN_6), PinId(AIN_7),
    PinId(AIN_8), PinId(AIN_9), PinId(AIN_10), PinId(AIN_11),
    PinId(AIN_12), PinId(AIN_13), PinId(AIN_14), PinId(AIN_15)
  };
}  // anonymous namespace

PinId GetAnalogPin(i8_t idx) {
  if (idx < 0 || idx > 15) return PinId(PIN_UNDEFINED);
  return analog_pins[idx];
}

i8_t GetAnalogIdx(PinId pin_id) {
  switch (pin_id.port()) {
    case PORT_A: case PORT_B: case PORT_C: return -1;
    case PORT_D: return pin_id.pin();
    case PORT_E: return (pin_id.pin() < 4) ? 8 + pin_id.pin() : -1;
    case PORT_F: return (pin_id.pin() > 1 && pin_id.pin() < 6) ?
        10 + pin_id.pin() : -1;
  }
  return -1;
}

struct PORT_struct* GetPortStruct(PinGroupId group_id) {
  switch (group_id) {
    case PORT_A: return &PORTA;
    case PORT_B: return &PORTB;
    case PORT_C: return &PORTC;
    case PORT_D: return &PORTD;
    case PORT_E: return &PORTE;
    case PORT_F: return &PORTF;
  }
  return NULL;
}
