// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include "DShotISR.h"
#include "DShotASM.h"

#include "Util.h"
#include "stdlib.h"
#include <util/atomic.h>

namespace {
  register8_t* out_set = NULL;
  register8_t* out_clr = NULL;
  u8_t* data_ptr = NULL;
  u8_t end_byte;
  u8_t mask_val = 0;
  bool sent_dshot = false;
}


#define TCBn TCB0
#define TCBn_vect (TCB0_INT_vect)

ISR(TCBn_vect) {
  TCBn.INTFLAGS = 1;
  u8_t data_val = 0;
  DSHOT600_ASM(out_set, out_clr, data_ptr, end_byte, data_val, mask_val);
  sent_dshot = true;
  // We are going to clear the bits in the telemetry bit so fix the
  // CRC 1/2 byte first. Then set the telemetry bits to zero for all
  // channels.
  data_ptr[15] = ~(data_ptr[15] ^ data_ptr[11]);
  data_ptr[11] = mask_val;  // Clear telemetry for all chanels.
}

DShotISR::DShotISR(Serial* serial, PinGroupId pins)
  : DShot(serial, pins) {
  long per_clk_sec = GetPerClock();
  u16_t per_clk_ms = per_clk_sec / 1000;
  TCBn.CTRLA = 0;  // CLK_PER = 1, not enabled.
  // We want it to run the interrupt every ~1ms.
  TCBn.CCMP = per_clk_ms; // TOP = 1ms
}

void DShotISR::Start() {
  out_set = &port_->OUTSET;
  out_clr = &port_->OUTCLR;
  data_ptr = dshot_data_;
  end_byte = ((u16_t)dshot_data_) + 16;
  mask_val = dshot_mask_;
  sent_dshot = false;
  
  TCBn.INTCTRL = 1;  // Enable interrupt
  TCBn.CTRLA = 1;  // Enable the counter
}

bool DShotISR::Run(bool* new_telemetry) {
  bool result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { 
    result = sent_dshot;
    sent_dshot = false;
  }
  bool got_telemetry = CheckSerial();
  if (new_telemetry) {
    *new_telemetry = got_telemetry;
  }
  return result;
}

