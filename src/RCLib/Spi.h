// Copyright 2020 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "IntTypes.h"
#include "leds/Rgb.h"

#ifndef SPI_H_
#define SPI_H_

class Spi {
  public:
  // Only one Spi
  static Spi spi;

  enum SpiPinOpt {
        PINS_PA47,  // default
        PINS_PC03,
        PINS_PE03,
        PINS_NONE,
  };
  void Setup(SpiPinOpt pins, u32_t target_clk);
  // Will try and get SPI Clock 'near' target_clk by adjsting SPI
  // clock scaling, will not adjust chip level pdiv (you must set that
  // keeping in mind clocks needed).
  void SetupAPA102(SpiPinOpt pins, u32_t target_clk);
  // Global PDIV can not be greater than 4, or this can not get SPI
  // clock high enough to time the pulses correctly.
  void SetupSK6812(SpiPinOpt pins);
  void WaitForIdle();
  void SendBytes(const u8_t* data, int len);
  void SendByte(u8_t data);
  // This is an async method using interrupts to send pix out on the
  // SPI bus.  That means that if you modify pix immediately after calling
  // this you might display 'weird' results.  If this is an issue then
  // you should call WaitForIdle() before modifying the pix array.
  void UpdateLeds(const led::RGB* pix, int nLed, u8_t level = 0xFF);

  // Only supported for RGBW SK6812 varient leds
  void UpdateLeds(const led::RGBW* pix, int nLed, u8_t level = 0xFF);

  // Do not call, exposed only to allow the ISR to call.
  void step();

protected:
  Spi();
  void StartTransfer();
  // Start and update for sk6812, len should be byte len not number of leds.
  void Sk6812UpdateSetup(const u8_t *data, int len, u8_t level);
  // Populate long_data_ with bits from val
  void Sk6812StartByte(u8_t val);

  volatile char state_;
  const u8_t* data_ptr_;
  int len_;
  int idx_;
  u8_t led_mode_;
  u8_t pix_init_mode_;
  u8_t one_byte_data_;
  u8_t long_data_[8];
};

#endif /* SPI_H_ */
