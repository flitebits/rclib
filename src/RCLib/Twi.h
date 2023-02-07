// Copyright 2023 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "IntTypes.h"

#ifndef _TWI_H_
#define _TWI_H_

class Twi {
  public:
  // Only one Twi
  static Twi twi;

  enum TwiPinOpt {
        PINS_DEF,  // default PA32, (slave PC32 in dual)
        PINS_ALT1, //         PA32, (slave PF32 in dual)
        PINS_ALT2, //         PC32, (slave PF32 in dual)
        PINS_NONE, //         Not connected.
  };
  enum TwiStdClk {
        I2C_100K, // Standard mode I2C
        I2C_400K, // Fast mode I2C
        I2C_1M,   // Fast mode+ I2C
  };

  void Setup(TwiPinOpt pins, TwiStdClk std_clk);
  void Setup(TwiPinOpt pins, u16_t tgt_kHz);

  void MasterSendBytes(u8_t addr, const u8_t* data, int len);
  void WaitForIdle();

  bool Error(){ return error_; }

  // Do not call, exposed only to allow the ISR to call.
  void step();

protected:
  Twi();

  bool error_;
  volatile u8_t state_;
  u8_t len_;
  u8_t idx_;
  const u8_t* data_ptr_;
};

#endif /* SPI_H_ */
