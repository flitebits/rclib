// Copyright 2026 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "RcChannels.h"

#include "Dbg.h"

u8_t ThreePosSwitch(i16_t val, i16_t threshold) {
  if (val <= threshold) return 0;
  if (val >= (2047 - threshold)) return 2;
  return 1;
}

void RcChannels::Dump() const {
  for (int i=0; i<16; ++i) {
    DBG_LO(SBUS, (" 0x%x", GetChannel(i)));
  }
  DBG_LO(SBUS, ("%c\n", FailSafe() ? 'F' : '-'));
}

void RcChannels::ChannelsUnpack(u8_t* data, u8_t num_ch) {
  if (num_ch > 16) num_ch = 16;
  u16_t *ch_ptr = channels_;
  u8_t* ptr = data;
  u32_t acc = 0;
  u8_t bits = 0;
  for (int i = 0; i < num_ch; ++i) {
    while (bits < 11) {
      acc |= (long(*(ptr++)) << bits);
      bits += 8;
    }
    *(ch_ptr++) = acc & (0x07FF);
    acc = acc >> 11;
    bits -= 11;
  }
}

void RcChannels::RescaleChannels(u16_t min, u16_t max) {
  const long scale = (((1L << 11) - 1) << 16) / (max - min);
  for (u8_t i = 0; i < 16; ++i) {
    channels_[i] = ((channels_[i] - min) * scale + (1 << 15)) >> 16;
  }
}
