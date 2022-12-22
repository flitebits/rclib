// Copyright 2020 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _Pins_4809_
#define _Pins_4809_

#include "IntTypes.h"
#include "avr/io.h"

enum PinGroupId {
  PORT_A = 0,
  PORT_B,
  PORT_C,
  PORT_D,
  PORT_E,
  PORT_F,
};

enum PinIdEnum {
  PIN_A0 = (PORT_A << 4) | 0,
  PIN_A1 = (PORT_A << 4) | 1,
  PIN_A2 = (PORT_A << 4) | 2,
  PIN_A3 = (PORT_A << 4) | 3,
  PIN_A4 = (PORT_A << 4) | 4,
  PIN_A5 = (PORT_A << 4) | 5,
  PIN_A6 = (PORT_A << 4) | 6,
  PIN_A7 = (PORT_A << 4) | 7,

  PIN_B0 = (PORT_B << 4) | 0,
  PIN_B1 = (PORT_B << 4) | 1,
  PIN_B2 = (PORT_B << 4) | 2,
  PIN_B3 = (PORT_B << 4) | 3,
  PIN_B4 = (PORT_B << 4) | 4,
  PIN_B5 = (PORT_B << 4) | 5,

  PIN_C0 = (PORT_C << 4) | 0,
  PIN_C1 = (PORT_C << 4) | 1,
  PIN_C2 = (PORT_C << 4) | 2,
  PIN_C3 = (PORT_C << 4) | 3,
  PIN_C4 = (PORT_C << 4) | 4,
  PIN_C5 = (PORT_C << 4) | 5,
  PIN_C6 = (PORT_C << 4) | 6,
  PIN_C7 = (PORT_C << 4) | 7,

  PIN_D0 = (PORT_D << 4) | 0,
  PIN_D1 = (PORT_D << 4) | 1,
  PIN_D2 = (PORT_D << 4) | 2,
  PIN_D3 = (PORT_D << 4) | 3,
  PIN_D4 = (PORT_D << 4) | 4,
  PIN_D5 = (PORT_D << 4) | 5,
  PIN_D6 = (PORT_D << 4) | 6,
  PIN_D7 = (PORT_D << 4) | 7,

  PIN_E0 = (PORT_E << 4) | 0,
  PIN_E1 = (PORT_E << 4) | 1,
  PIN_E2 = (PORT_E << 4) | 2,
  PIN_E3 = (PORT_E << 4) | 3,

  PIN_F0 = (PORT_F << 4) | 0,
  PIN_F1 = (PORT_F << 4) | 1,
  PIN_F2 = (PORT_F << 4) | 2,
  PIN_F3 = (PORT_F << 4) | 3,
  PIN_F4 = (PORT_F << 4) | 4,
  PIN_F5 = (PORT_F << 4) | 5,
  PIN_F6 = (PORT_F << 4) | 6,

  AIN_0 = PIN_D0,
  AIN_1 = PIN_D1,
  AIN_2 = PIN_D2,
  AIN_3 = PIN_D3,
  AIN_4 = PIN_D4,
  AIN_5 = PIN_D5,
  AIN_6 = PIN_D6,
  AIN_7 = PIN_D7,
  AIN_8 = PIN_E0,
  AIN_9 = PIN_E1,
  AIN_10 = PIN_E2,
  AIN_11 = PIN_E3,
  AIN_12 = PIN_F2,
  AIN_13 = PIN_F3,
  AIN_14 = PIN_F4,
  AIN_15 = PIN_F5,

  PIN_UPDI = (PORT_F << 4) | 7,
  PIN_UNDEFINED = 255,
};

class PinId;

PinId GetAnalogPin(i8_t idx);
i8_t GetAnalogIdx(PinId pin_id);
PORT_t* GetPortStruct(PinGroupId);

class PinId {
public:
  PinId() : val_(PIN_UNDEFINED) { }
  PinId(PinIdEnum pinVal) : val_(pinVal) { }
  PinId(PinGroupId pin_group, u8_t pin) :
    val_(PinIdEnum((pin_group << 4) | pin)) { }
  void SetPin(PinIdEnum pinVal) { val_ = pinVal; }

  void SetOutput(bool state = false, bool inverted = false,
                 bool pullup = false) {
    PORT_t* port = port_ptr();
    u8_t pidx = pin();
    *pin_ctrl() = ((inverted ? PORT_INVEN_bm : 0) |
                   (pullup ? PORT_PULLUPEN_bm : 0));
    if (state) port->OUTSET = 1 << pidx;
    else       port->OUTCLR = 1 << pidx;
    port->DIRSET = 1 << pidx;
  }
  /*
     PORT_ISC_INTDISABLE_gc, // Interrupt disabled but input buffer enabled
     PORT_ISC_BOTHEDGES_gc = (0x01<<0), //Sense Both Edges
     PORT_ISC_RISING_gc = (0x02<<0),  // Sense Rising Edge
     PORT_ISC_FALLING_gc = (0x03<<0),  // Sense Falling Edge
     PORT_ISC_INPUT_DISABLE_gc = (0x04<<0),  // Digital Input Buffer disabled  */
  void SetInput(bool inverted = false, bool pullup = false,
                PORT_ISC_t interrupt_mode = PORT_ISC_INTDISABLE_gc) {
    *pin_ctrl() = ((inverted ? PORT_INVEN_bm : 0) |
                   (pullup ? PORT_PULLUPEN_bm : 0) |
                   interrupt_mode);
    port_ptr()->DIRCLR = 1 << pin();
  }
  void toggle() { port_ptr()->OUTTGL = (1 << pin()); }
  void set(bool state) {
    if (state) port_ptr()->OUTSET = (1 << pin());
    else       port_ptr()->OUTCLR = (1 << pin());
  }
  bool get() { return port_ptr()->OUT & (1 << pin()); }

  bool in() { return port_ptr()->IN & (1 << pin()); }

  PinGroupId port() { return PinGroupId(val_ >> 4); }
  u8_t pin() { return val_ & 0xF; }
  PORT_t* port_ptr() { return GetPortStruct(port()); }
  register8_t* pin_ctrl() { return &((&port_ptr()->PIN0CTRL)[pin()]); }
  PinIdEnum val_;
};

#endif
