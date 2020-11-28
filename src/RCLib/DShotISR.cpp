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
  BitBang* gBitBang = NULL;
  u8_t* gDShotData = NULL;
  u8_t gDShotMask = 0;
  bool gSentDShot = false;
}


#define TCBn TCB0
#define TCBn_vect (TCB0_INT_vect)

ISR(TCBn_vect) {
  TCBn.INTFLAGS = 1;
  gBitBang->SendDShot600(gDShotData, gDShotMask);
  gSentDShot = true;
  // We are going to clear the bits in the telemetry bit so fix the
  // CRC 1/2 byte first. Then set the telemetry bits to zero for all
  // channels.
  gDShotData[15] = ~(gDShotData[15] ^ gDShotData[11]);
  gDShotData[11] = gDShotMask;  // Clear telemetry for all chanels.
}

DShotISR::DShotISR(Serial* serial, BitBang* bitbang)
  : DShot(serial, bitbang) {
  long per_clk_sec = GetPerClock();
  u16_t per_clk_ms = per_clk_sec / 1000;
  TCBn.CTRLA = 0;  // CLK_PER = 1, not enabled.
  // We want it to run the interrupt every ~1ms.
  TCBn.CCMP = per_clk_ms; // TOP = 1ms
}

void DShotISR::Start() {
  gBitBang = bitbang_;
  gDShotData = dshot_data_;
  gDShotMask = dshot_mask_;
  gSentDShot = false;
  
  TCBn.INTCTRL = 1;  // Enable interrupt
  TCBn.CTRLA = 1;  // Enable the counter
}

bool DShotISR::Run(bool* new_telemetry) {
  bool result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { 
    result = gSentDShot;
    gSentDShot = false;
  }
  bool got_telemetry = CheckSerial();
  if (new_telemetry) {
    *new_telemetry = got_telemetry;
  }
  return result;
}

