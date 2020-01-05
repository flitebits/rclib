// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include "Util.h"
#include <avr/io.h>

namespace {
long ReadMainClockFromReg() {
  switch (CLKCTRL.MCLKCTRLA & CLKCTRL_CLKSEL_gm) {
    case 0:
      switch (FUSE.OSCCFG & FUSE_FREQSEL_gm){
        case 0: case 3: break;  // reserved values, how are we running?
        case 1: return 16000000;
        case 2: return 20000000;
      }
    break;
    case 1: return 32000; 
    case 2: return 32768;
    case 3: break;
  }
  return 0;
}

u8_t ReadPerClockScaleFromReg() {
  if ((CLKCTRL.MCLKCTRLB & CLKCTRL_PEN_bm) == 0) return 1;
  switch (CLKCTRL.MCLKCTRLB & CLKCTRL_PDIV_gm) {
    case CLKCTRL_PDIV_2X_gc: return 2;
    case CLKCTRL_PDIV_4X_gc: return 4;
    case CLKCTRL_PDIV_8X_gc: return 8;
    case CLKCTRL_PDIV_16X_gc: return 16;
    case CLKCTRL_PDIV_32X_gc: return 32;
    case CLKCTRL_PDIV_64X_gc: return 64;
    case CLKCTRL_PDIV_6X_gc: return 6;
    case CLKCTRL_PDIV_10X_gc: return 10;
    case CLKCTRL_PDIV_12X_gc: return 12;
    case CLKCTRL_PDIV_24X_gc: return 24;
    case CLKCTRL_PDIV_48X_gc: return 48;
  }
  return 1;
}

long ReadPerClockFromReg() {
  long main_clk = GetMainClock();
  u8_t pclk_scale = GetPerClockScale();
  return main_clk / pclk_scale;
}

}  // namespace

long GetMainClock() {
  static long main_clk = 0;
  if (main_clk == 0) main_clk = ReadMainClockFromReg();
  return main_clk;
}

u8_t GetPerClockScale() {
  static u8_t per_clk_scale = 0;
  if (per_clk_scale == 0) per_clk_scale = ReadPerClockScaleFromReg();
  return per_clk_scale;
}

long GetPerClock() {
  static long per_clk = 0;
  if (per_clk == 0) per_clk = ReadPerClockFromReg();
  return per_clk;
}


i8_t GetMainClockErr(bool at5V)
{
  switch (FUSE.OSCCFG & FUSE_FREQSEL_gm){
    case 0: case 3: break;  // reserved values, how are we running?
    case 1: return (at5V ? SIGROW.OSC16ERR5V : SIGROW.OSC16ERR3V);
    case 2: return (at5V ? SIGROW.OSC20ERR5V : SIGROW.OSC20ERR3V);
  } 
  return 0;
}

void memset(void* ptr, int len, u8_t val){
  u8_t *p = (u8_t *)ptr;
  while (len--) {
    *(p++) = val;
  }
}
