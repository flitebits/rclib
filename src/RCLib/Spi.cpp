// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include "Spi.h"

#include <stddef.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "Dbg.h"
#include "util.h"

Spi Spi::spi;

namespace {
  enum {
	SPI_LED_MODE_NONE,
	SPI_LED_MODE_APA102,
	SPI_LED_MODE_SK6812,
  };
  
  enum {
	SPI_STATE_ONE_BYTE,
	SPI_STATE_BYTES,
	SPI_STATE_STOP,
	SPI_STATE_IDLE,

	SPI_STATE_APA102_ZEROS,
	SPI_STATE_APA102_PIXINIT,
	SPI_STATE_APA102_PIXDAT0,
	SPI_STATE_APA102_PIXDAT1,
	SPI_STATE_APA102_PIXDAT2,
	SPI_STATE_APA102_FLUSH,

	SPI_STATE_SK6812_PIXINIT,
	SPI_STATE_SK6812_PIXINIT_SCALE,
	SPI_STATE_SK6812_PIXINIT_ZERO,
	SPI_STATE_SK6812_PIX1,
	SPI_STATE_SK6812_PIX2,
	SPI_STATE_SK6812_PIX3,
	SPI_STATE_SK6812_PIX4,
	SPI_STATE_SK6812_PIX5,
	SPI_STATE_SK6812_PIX6,
	SPI_STATE_SK6812_PIX7,
	SPI_STATE_SK6812_FLUSH,
  };

  // This calculates the preScale and clock_2x values to get to a
  // clock speed near target_clk.
  void SetClock(u32_t target_clk, u8_t ctrla_base) {
    u32_t base_clk = GetMainClock();
    u8_t pdiv = GetPerClockScale();

    u32_t clk = (base_clk / pdiv) >> 1;
    u8_t bits = 1;
    // Calculate the lowest scaled clock still >= target.
    while ((clk >> 1) >= target_clk) {
      clk = clk >> 1;
      bits++;
    }
    // Then Check if going one more step (under target) is closer.
    if (target_clk - (clk >> 1) < clk - target_clk) 
      bits++;
    if (bits > 7) bits = 7;  // Max prescale is 128
    
    u8_t clk_2x = 0;
    u8_t preScale = 0;
    switch (bits) {
    case 1: clk_2x = 1;  // fallthrough
    case 2: preScale = 0; break;
    case 3: clk_2x = 1;  // fallthrough
    case 4: preScale = 1; break;
    case 5: clk_2x = 1;  // fallthrough
    case 6: preScale = 2; break;
    case 7: preScale = 3; break;
    }
    SPI0.CTRLA = (ctrla_base | // SPI Master mode
		  (clk_2x << SPI_CLK2X_bp) |  //  Use 2x CLK
		  (preScale << SPI_PRESC_gp));  // prescale clock
  }
}  // namespace

ISR(SPI0_INT_vect) {
  Spi::spi.step();
}

void Spi::step() {
  switch (state_) {
  case SPI_STATE_STOP:
  case SPI_STATE_IDLE:
    SPI0.INTCTRL = 0;  // Disable interrupts
    SPI0.CTRLA &= ~(1 << SPI_ENABLE_bp);  // Disable SPI 
    state_ = SPI_STATE_IDLE;
    DBG_MD(SPI, ("Update Done\n"));
    return;

    // Sends one byte and stops...
  case SPI_STATE_ONE_BYTE:
    SPI0.DATA = one_byte_data_;
    SPI0.INTCTRL = (1 << SPI_TXCIE_bp);  // switch to xfer complete.
    state_ = SPI_STATE_STOP;
    break;
    // Sends len bytes and stops...
  case SPI_STATE_BYTES:
    SPI0.DATA = *(data_ptr_++);
    if (++idx_ == len_) {
      SPI0.INTCTRL = (1 << SPI_TXCIE_bp);  // switch to xfer complete.
      state_ = SPI_STATE_STOP;
    }
    break;
    
    // For APA102 this is interrupt driven. It uses one_byte_data as
    // the first 8bits for each pixel (0xE | 5bit level). Then it sends
    // the next three bytes from data_ptr (which should point at pixels).
    // Rinse and repeat.  It also sends 4 bytes of zeros at the start and
    // then len_/2 1 bits at the end (to signal reset).
  case SPI_STATE_APA102_ZEROS:
    SPI0.DATA = 0;
    if (++idx_ == 4) {
      idx_ = 0;
      state_ = (idx_ == len_) ?
	SPI_STATE_APA102_FLUSH : SPI_STATE_APA102_PIXINIT;
    }
    break;
  case SPI_STATE_APA102_PIXINIT:
    SPI0.DATA = one_byte_data_;
    state_ = SPI_STATE_APA102_PIXDAT0;
    break;
  case SPI_STATE_APA102_PIXDAT0:
    SPI0.DATA = ((led::RGB*)data_ptr_)->blu;
    state_ = SPI_STATE_APA102_PIXDAT1;
    break;
  case SPI_STATE_APA102_PIXDAT1:
    SPI0.DATA = ((led::RGB*)data_ptr_)->grn;
    state_ = SPI_STATE_APA102_PIXDAT2;
    break;
  case SPI_STATE_APA102_PIXDAT2:
    SPI0.DATA = ((led::RGB*)data_ptr_)->red;
    data_ptr_ += sizeof(led::RGB);
    if (++idx_ < len_) {
      state_ = SPI_STATE_APA102_PIXINIT;
    } else {
      idx_ = 0;
      state_ = SPI_STATE_APA102_FLUSH;
    }
    break;
  case SPI_STATE_APA102_FLUSH:
    SPI0.DATA = 0xFF;
    idx_ += 16;  // Each set of 8 ones flushes 16 leds.
    if (idx_ >= len_) {
      SPI0.INTCTRL = (1 << SPI_TXCIE_bp);  // switch to xfer complete.
      state_ = SPI_STATE_STOP;
    }
    break;

    // SK6812 is interrupt driven.  Each bit is one clock of high or
    // low.  we target 400ns for each sample and expand each data bit
    // to 4 output bits 2 hi, 2 low for a 1 bit, and 1 hi, 3 low for a
    // 0 bit. Note that this is slightly outside the spec for timing,
    // since the 1 bit should be 600ns but this produces 800ns. This
    // appears to work fine anyway and extending the 1 pulse doesn't
    // seem overly problematic since it still wouldn't be confused
    // with a zero.
    //
    // The pix init preps 4 output bytes from 1 source byte at
    // a time.  At the end it sends ~80us (~25 bytes) of all zeros.
    // RGB vs RGBW is simply handled by sending num_pix * 4 bytes
    // (properly ordered) vs num_pix * 3 bytes.
  case SPI_STATE_SK6812_PIXINIT:
    Sk6812StartByte(*(data_ptr_++));
    state_ = SPI_STATE_SK6812_PIX1;
    break;
  case SPI_STATE_SK6812_PIXINIT_ZERO:
    SPI0.DATA     = long_data_[0];
    state_ = SPI_STATE_SK6812_PIX1;
    break;
  case SPI_STATE_SK6812_PIXINIT_SCALE:
    Sk6812StartByte((*(data_ptr_++) * u16_t(one_byte_data_)) >> 8);
    state_ = SPI_STATE_SK6812_PIX2;
    break;
  case SPI_STATE_SK6812_PIX1:
    SPI0.DATA = long_data_[1];
    state_ = SPI_STATE_SK6812_PIX2;
    break;
  case SPI_STATE_SK6812_PIX2:
    SPI0.DATA = long_data_[2];
    state_ = SPI_STATE_SK6812_PIX3;
    break;
  case SPI_STATE_SK6812_PIX3:
    SPI0.DATA = long_data_[3];
    if (++idx_ < len_) {
      state_ = pix_init_mode_;
    } else { // sent all data
      idx_ = 0;
      state_ = SPI_STATE_SK6812_FLUSH;
    }
    break;
  case SPI_STATE_SK6812_FLUSH:
    SPI0.DATA = 0x00;
    if (++idx_ > 50) {
      SPI0.INTCTRL = (1 << SPI_TXCIE_bp);  // switch to xfer complete.
      state_ = SPI_STATE_STOP;
    }
    break;
  }
}

Spi::Spi()
  : state_(SPI_STATE_IDLE),
    data_ptr_(NULL),
    len_(0),
    idx_(0),
    led_mode_(SPI_LED_MODE_NONE),
    pix_init_mode_(SPI_STATE_IDLE),
    one_byte_data_(0) { }

void Spi::Setup(SpiPinOpt pins, u32_t target_clk) {
  led_mode_ = SPI_LED_MODE_NONE;
  
  // Pins PA4,PA6,PA7 output (MOSI,SCLK,SS), PA5 input (MISO)
  switch (pins) {
  case PINS_PA47:
    PORTA.DIR  = (PORTA.DIR & 0b00001111) | 0b11010000;  // setup all pins
    PORTMUX.TWISPIROUTEA = (PORTMUX_SPI0_DEFAULT_gc);
    break;
  case PINS_PC03:
    PORTC.DIR  = (PORTC.DIR & 0b11110000) | 0b00001101;
    PORTMUX.TWISPIROUTEA = (PORTMUX_SPI0_ALT1_gc);
    break;
  case PINS_PE03:
    PORTE.DIR  = (PORTE.DIR & 0b11110000) | 0b00001101;
    PORTMUX.TWISPIROUTEA = (PORTMUX_SPI0_ALT2_gc);
    break;
  }

  SetClock(target_clk, ((0 << SPI_DORD_bp) |    // MSB
			(1 << SPI_MASTER_bp))); // SPI Master mode

  SPI0.CTRLB = ((1 << SPI_BUFEN_bp) | // Buffer Mode: enable
                (1 << SPI_BUFWR_bp) | // Buffer Mode Wait disable
                (1 << SPI_SSD_bp) |   // Disable Slave Select line as SPI Master
                (SPI_MODE_3_gc));
  // Disable all interrupts, we will enable DRE (data ready enable)
  // interrupt once we have data to send, then disable it when all the
  // data is sent, also we don't enable SPI here we do that when we
  // have data to send.
  SPI0.INTCTRL = 0;
}

void Spi::SetupAPA102(SpiPinOpt pins, u32_t target_clk) {
  led_mode_ = SPI_LED_MODE_APA102;
  switch (pins) {
  case PINS_PA47:
    PORTA.DIRSET = 0b01010000;  // Only need MOSI & SCLK for APA102
    PORTMUX.TWISPIROUTEA = (PORTMUX_SPI0_DEFAULT_gc);
    break;
  case PINS_PC03:
    PORTC.DIRSET = 0b00000101;  // Only need MOSI & SCLK for APA102
    PORTMUX.TWISPIROUTEA = (PORTMUX_SPI0_ALT1_gc);
    break;
  case PINS_PE03:
    PORTE.DIRSET = 0b00000101;  // Only need MOSI & SCLK for APA102
    PORTMUX.TWISPIROUTEA = (PORTMUX_SPI0_ALT2_gc);
    break;
  }
  
  SetClock(target_clk, ((0 << SPI_DORD_bp) |    // MSB
			(1 << SPI_MASTER_bp))); // SPI Master mode

  SPI0.CTRLB = ((1 << SPI_BUFEN_bp) | // Buffer Mode: enable
                (1 << SPI_BUFWR_bp) | // Buffer Mode Wait disable
                (1 << SPI_SSD_bp) |   // Disable Slave Select line as SPI Master
                (SPI_MODE_3_gc));
  // Disable all interrupts, we will enable DRE (data ready enable)
  // interrupt once we have data to send, then disable it when all the
  // data is sent, also we don't enable SPI here we do that when we
  // have data to send.
  SPI0.INTCTRL = 0;
}

void Spi::SetupSK6812(SpiPinOpt pins) {
  led_mode_ = SPI_LED_MODE_SK6812;
  switch (pins) {
  case PINS_PA47:
    PORTA.DIRSET = 0b00010000;  // Only need MOSI for SK6812
    PORTMUX.TWISPIROUTEA = (PORTMUX_SPI0_DEFAULT_gc);
    break;
  case PINS_PC03:
    PORTC.DIRSET = 0b00000001;  // Only need MOSI for SK6812
    PORTMUX.TWISPIROUTEA = (PORTMUX_SPI0_ALT1_gc);
    break;
  case PINS_PE03:
    PORTE.DIRSET = 0b00000001;  // Only need MOSI for SK6812
    PORTMUX.TWISPIROUTEA = (PORTMUX_SPI0_ALT2_gc);
    break;
  }

  // We will get the clock to ~2.5Mhz which is 400ns/sample
  SetClock(2500000, ((0 << SPI_DORD_bp) |    // MSB
		     (1 << SPI_MASTER_bp))); // SPI Master mode
  
  SPI0.CTRLB = ((1 << SPI_BUFEN_bp) | // Buffer Mode: enable
                (1 << SPI_BUFWR_bp) | // Buffer Mode Wait disable
                (1 << SPI_SSD_bp) |   // Disable Slave Select line as SPI Master
                (SPI_MODE_3_gc));
  // Disable all interrupts, we will enable DRE (data ready enable)
  // interrupt once we have data to send, then disable it when all the
  // data is sent, also we don't enable SPI here we do that when we
  // have data to send.
  SPI0.INTCTRL = 0;
}

void Spi::WaitForIdle() {
  // State goes to IDLE when SPI transfer completes.
  while(state_ != SPI_STATE_IDLE);
}

void Spi::StartTransfer() {
  // Turn on SPI and enable DRE interrupt, this should immediately trigger
  // the SPI interrupt which will start shifting out data.
  SPI0.CTRLA |= (1 << SPI_ENABLE_bp);  // Enable SPI 
  SPI0.INTCTRL = (1 << SPI_DREIF_bp);  // Enable Data ready interrupt
}

void Spi::SendBytes(const u8_t* data, int len) {
  WaitForIdle();
  state_ = SPI_STATE_BYTES;
  idx_ = 0;
  len_ = len;
  data_ptr_ = data;
  StartTransfer();
}

void Spi::SendByte(u8_t data) {
  WaitForIdle();
  state_ = SPI_STATE_ONE_BYTE;
  one_byte_data_ = data;
  StartTransfer();
}

void Spi::UpdateLeds(const led::RGB* pix, int nLed, u8_t level) {
  WaitForIdle();
  switch (led_mode_) {
  case SPI_LED_MODE_NONE: return;  // Do nothing
  case SPI_LED_MODE_APA102:  
    one_byte_data_ = 0xE0 | (level >> 3);
    data_ptr_ = (u8_t *)pix;
    idx_ = 0;
    len_ = nLed;
    state_ = SPI_STATE_APA102_ZEROS;
    break;
  case SPI_LED_MODE_SK6812:  
    Sk6812UpdateSetup((u8_t *)pix, 3 * nLed, level);
    break;
  }
  StartTransfer();
}

void Spi::UpdateLeds(const led::RGBW* pix, int nLed, u8_t level) {
  WaitForIdle();
  if (led_mode_ != SPI_LED_MODE_SK6812) {
    return;  // Do nothing (unsupported)
  }
  Sk6812UpdateSetup((u8_t *)pix, 4 * nLed, level);
  StartTransfer();
}

void Spi::Sk6812UpdateSetup(const u8_t* data, int len, u8_t level) {
  DBG_MD(SPI, ("Update Start\n"));
  data_ptr_ = data;
  idx_ = 0;
  len_ = len;
  if (level == 0) {
    pix_init_mode_ = SPI_STATE_SK6812_PIXINIT_ZERO;
    for (int i=0; i < 8; ++i)
      long_data_[i] = 0b10001000;
  } else if (level < 0xFF) {
    one_byte_data_ = level + 1;
    pix_init_mode_ = SPI_STATE_SK6812_PIXINIT_SCALE;
  } else {
    pix_init_mode_ = SPI_STATE_SK6812_PIXINIT;  // no scaling
  }
  state_ = pix_init_mode_;
}

/*  // For using 1byte for 1 bit output, with a 5MHz clock.
void Spi::Sk6812StartByte(u8_t val) {
  SPI0.DATA = 0b11000000 | ((val & 0x80) >> 2);
  long_data_[1] = 0b11000000 | ((val & 0x40) >> 1);
  long_data_[2] = 0b11000000 |  (val & 0x20);
  long_data_[3] = 0b11000000 | ((val & 0x10) << 1);
  long_data_[4] = 0b11000000 | ((val & 0x08) << 2);
  long_data_[5] = 0b11000000 | ((val & 0x04) << 3);
  long_data_[6] = 0b11000000 | ((val & 0x02) << 4);
  long_data_[7] = 0b11000000 | ((val & 0x01) << 5);
}

void Spi::Sk6812StartByte(u8_t val) {
  SPI0.DATA = 0b11000011 | ((val & 0x80) >> 2);
  long_data_[1] = 0b00001100 | ((val & 0x40) << 1) | ((val & 0x20) >> 4);
  long_data_[2] = 0b00110000 | ((val & 0x10) >> 1);
  while (!(SPI0.INTFLAGS & (1 << SPI_DREIF_bp)));
  SPI0.DATA = long_data_[1];

  long_data_[3] = 0b11000011 | ((val & 0x08) << 2);
  long_data_[4] = 0b00001100 | ((val & 0x04) << 5) | (val & 0x02);
  long_data_[5] = 0b00110000 | ((val & 0x01) << 3);
  while (!(SPI0.INTFLAGS & (1 << SPI_DREIF_bp)));
  SPI0.DATA = long_data_[2];
}
*/

void Spi::Sk6812StartByte(u8_t val) {
  SPI0.DATA     = 0b10001000 | ((val & 0x80) >> 1) | ((val & 0x40) >> 4);
  long_data_[1] = 0b10001000 | ((val & 0x20) << 1) | ((val & 0x10) >> 2);
  long_data_[2] = 0b10001000 | ((val & 0x08) << 3) | ((val & 0x04));
  long_data_[3] = 0b10001000 | ((val & 0x02) << 5) | ((val & 0x01) << 2);
  while (!(SPI0.INTFLAGS & (1 << SPI_DREIF_bp)));
  SPI0.DATA = long_data_[1];
}
