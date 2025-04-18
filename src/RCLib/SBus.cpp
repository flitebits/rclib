// Copyright 2020 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "SBus.h"

#include "Dbg.h"

namespace {
  const int sbus_max_val = 2047;
  const int sbus_surr = 172;
  const long sbus_scale = (2047L << 16) / 1640;
  int sbus_err = 0;
  void memset(u8_t* ptr, u8_t val, int len) {
    while (len--) (*ptr++) = val;
  }
}

SBus::SBus(Serial* serial, bool invert) :
  serial_(serial),
  idx_(0), failSafe_(true), bytes_read_(0), frames_(0) {
  serial_->Setup(100000, 8, Serial::PARITY_EVEN, 2, invert,
                 /*use_alt_pins=*/false, Serial::MODE_RX);
  serial_->SetBuffered(true);
  DBG_LO(SBUS, ("SBus: Init\n"));
}

void SBus::Dump() const {
  DBG_LO(SBUS, ("SBus: %d/%d-%d", GetDataFrames(), GetBytesRead(), sbus_err));
  for (int i=0; i<16; ++i) {
    DBG_LO(SBUS, (" 0x%x", GetChannel(i)));
  }
  DBG_LO(SBUS, ("%c\n", FailSafe() ? 'F' : '-'));
}

u8_t SBus::ThreePosSwitch(i16_t val, i16_t threshold) {
  if (val <= threshold) return 0;
  if (val >= (sbus_max_val - threshold)) return 2;
  return 1;
}

// Stitches 11 bits out of the data array at the given offeset.
int SBus::GetBits(int start_byte, int start_bit) {
  start_byte += (start_bit >> 3);
  const u8_t b0 = data_[(start_byte + 0) & 0x1F];
  const u8_t b1 = data_[(start_byte + 1) & 0x1F];
  switch (start_bit & 0x07) {
    default:
    case 0: return ( b0       | (u16_t(b1 & ((1 << 3) - 1)) << 8));
    case 1: return ((b0 >> 1) | (u16_t(b1 & ((1 << 4) - 1)) << 7));
    case 2: return ((b0 >> 2) | (u16_t(b1 & ((1 << 5) - 1)) << 6));
    case 3: return ((b0 >> 3) | (u16_t(b1 & ((1 << 6) - 1)) << 5));
    case 4: return ((b0 >> 4) | (u16_t(b1 & ((1 << 7) - 1)) << 4));
    case 5: return ((b0 >> 5) | (u16_t(b1)                  << 3));
    case 6: {
      const u8_t b2 = data_[(start_byte + 2) & 0x1F];
      return (b0 >> 6) | (b1 << 2) | ((b2 & 0x01) << 10);
    }
    case 7: {
      const u8_t b2 = data_[(start_byte + 2) & 0x1F];
      return (b0 >> 7) | (b1 << 1) | ((b2 & 0x03) << 9);
    }
  }
}

bool SBus::Run() {
  while(serial_->Avail()) {
    ReadInfo info;
    serial_->Read(&info);
    bytes_read_++;
    if (info.err) {  // error reading serial data, so don't trust anything.
      DBG_MD(SBUS, ("SBus serial read Error: %d\n", info.err));
      memset(data_, 0xFF, sizeof(data_));
      sbus_err++;
      continue;
    }
    idx_ = (idx_ + 1) & 0x1F;
    data_[idx_] = info.data;
    u8_t read_time = (info.time >> 5);  // convert to ms.
    time_[idx_] = read_time;

    // Start byte should be 0x0F and end byte is 0x00
    if (info.data != 0x00) continue;
    u8_t start_idx = (idx_ - 24) & 0x1F;
    if (data_[start_idx] != 0x0F) continue;

    // All the bytes should arrive within 4ms if it takes longer
    // it is probably split across multiple messages.
    u8_t elapsed = read_time - time_[start_idx];
    if (elapsed > 4) continue;

    failSafe_ = (data_[(start_idx + 23) & (0x1F)] & (0x3 << 2));
    int start_bit = 0;
    for (int i=0; i < 16; i++) {
      int val = GetBits(start_idx + 1, start_bit);
      if (val <= sbus_surr) val = 0;
      else val -= sbus_surr;
      val = (val * sbus_scale) >> 16;
      if (val > sbus_max_val) val = sbus_max_val;
      rcVal_[i] = val;
      start_bit += 11;
    }
    memset(data_, 0xFF, sizeof(data_));
    ++frames_;
    return true;
  }
  return false;
}
