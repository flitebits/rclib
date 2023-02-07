// Copyright 2023 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "Twi.h"

#include <stddef.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "Dbg.h"
#include "Pins.h"
#include "util.h"

Twi Twi::twi;

namespace {
  u8_t MainClkMhz() {
    switch (FUSE.OSCCFG & FUSE_FREQSEL_gm){
    case 0: case 3: break;  // reserved values, how are we running?
    case 1: return 16;
    case 2: return 20;
    }
    return 0;
  }

  u8_t FScl(u16_t tgt_kHz) {
    u16_t main_clk = MainClkMhz() * 1000;
    u8_t pscl = GetPerClockScale();

    u16_t p_clk = main_clk / pscl;
    return (p_clk / tgt_kHz - 10);
  }

  void SetBaud(u16_t tgt_kHz) {
    u8_t baud = FScl(tgt_kHz);
    TWI0.MBAUD = baud;
  }

  u16_t StdClkTokHz(Twi::TwiStdClk std_clk) {
    switch (std_clk)  {
    case Twi::I2C_100K: return 100;
    case Twi::I2C_400K: return 400;
    case Twi::I2C_1M: return 1000;
    default: break;
    }
    return 0;
  }
}  // anonymous namespace

enum {
      TWI_STATE_IDLE,
      TWI_STATE_BYTES,
      TWI_STATE_STOP,
};

ISR(TWI0_TWIM_vect) {
  Twi::twi.step();
}

Twi::Twi()
  : error_(false), state_(TWI_STATE_IDLE), len_(0), idx_(0), data_ptr_(NULL) {}

void Twi::step() {
  switch(state_) {
  case TWI_STATE_IDLE: TWI0.MSTATUS = TWI_WIF_bm; break;

  case TWI_STATE_BYTES: {
    if ((TWI0.MSTATUS & (TWI_RXACK_bm | TWI_BUSERR_bm | TWI_ARBLOST_bm)) != 0) {
      error_ = true;
      state_ = TWI_STATE_IDLE;
      DBG_MD(APP, ("TWI Bytes, error MSTATUS: %d\n", TWI0.MSTATUS));
      break;
    }
    u8_t idx = idx_;
    if (++idx_ == len_) {
      state_ = TWI_STATE_STOP;
    }
    TWI0.MDATA = data_ptr_[idx];
  }
    break;

  case TWI_STATE_STOP:
    state_ = TWI_STATE_IDLE;
    TWI0.MCTRLB = TWI_MCMD_STOP_gc;
    break;
  }
}

void Twi::Setup(Twi::TwiPinOpt pins, Twi::TwiStdClk std_clk) {
  Setup(pins, StdClkTokHz(std_clk));
}
void Twi::Setup(Twi::TwiPinOpt pins, u16_t tgt_kHz) {
  u8_t port_mux;
  switch (pins) {
  case PINS_DEF:  port_mux = PORTMUX_TWI0_DEFAULT_gc; break;
  case PINS_ALT1: port_mux = PORTMUX_TWI0_ALT1_gc; break;
  case PINS_ALT2: port_mux = PORTMUX_TWI0_ALT2_gc; break;
  default:
  case PINS_NONE: port_mux = PORTMUX_TWI0_NONE_gc; break;
  }
  PORTMUX.TWISPIROUTEA = (PORTMUX.TWISPIROUTEA & PORTMUX_SPI0_gm) | port_mux;
  SetBaud(tgt_kHz);

  TWI0.CTRLA = (TWI_SDASETUP_4CYC_gc | TWI_SDAHOLD_OFF_gc
                /*| (((tgt_kHz < 999)?1:0) << TWI_FMPEN_bp)*/);
  TWI0.DUALCTRL = 0;

  TWI0.MCTRLB = TWI_FLUSH_bm;

  TWI0.MCTRLA = TWI_WIEN_bm | TWI_TIMEOUT_DISABLED_gc | TWI_ENABLE_bm;
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
  TWI0.MCTRLB = TWI_FLUSH_bm;
}

void Twi::WaitForIdle() {
  while(state_ != TWI_STATE_IDLE);
}

void Twi::MasterSendBytes(u8_t addr, const u8_t* data, int len) {
  WaitForIdle();
  if ((TWI0.MSTATUS & TWI_BUSSTATE_gm) == 0) {
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
  }
  error_ = false;
  data_ptr_ = data;
  len_ = len;
  idx_ = 0;
  state_ = TWI_STATE_BYTES;
  TWI0.MADDR = (addr & 0xFE); // clear low bit 0 = W, 1 = R
}
