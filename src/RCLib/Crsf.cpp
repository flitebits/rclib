// Copyright 2026 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "Crsf.h"

#include "Dbg.h"

namespace {
  // Total range is +/- 800 (0x320) nominal
  // Bias is 992 (0x3D0) which provides some room
#define TICKS_TO_US(x)  ((x - 0x3D0) * 5 / 8 + 1500)
#define US_TO_TICKS(x)  ((x - 1500) * 8 / 5 + 0x3D0)

  // Max packet len = 64
  // 8 bits per byte on wire (with 1 stop)
  static const long kDefaultBaud = 420000;
  static const u8_t kMaxElapsed = 41;  // (64*8*32768) / 420000) = 39.95
  static const u8_t kFrameLenIdx = 1;
  static const u8_t kTypeIdx = 2;
  
  // Do a 16 value (4bit) crc8 table and use it twice per-byte.
  // Reduces memory usage 16x with slight runtime increase.
  static const u8_t crc8_table[16] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D
  };
  u8_t crsf_crc8(const u8_t *data, u8_t len, u8_t crc=0x00) {
    for (size_t i = 0; i < len; i++) {
      crc = crc ^ data[i];
      crc = (crc << 4) ^ crc8_table[crc >> 4];
      crc = (crc << 4) ^ crc8_table[crc >> 4];
    }
    return crc;
  }
}  // anonymous namespace

// 0x08 Battery Sensor
struct CrsfBatt {
    i16_t voltage;        // Voltage (LSB = 10 µV)
    i16_t current;        // Current (LSB = 10 µA)
    u32_t capacity_remain;  // Capacity used (mAh) 24bits, remaining 8 bits (%)
  // uint24_t capacity_used; // Capacity used (mAh)
  // uint8_t remaining;      // Battery remaining (percent)
};

// 0x0E Voltages (or "Voltage Group")
// source_id - 0->127 cell voltages of a single battery (up to 29s)
// source_id - 127->255 General voltages (ESC in, BEC out, etc).
struct CrsfVoltage {
  u8_t source_id;
  u16_t values[];  // up to 29 voltages in mV (3.85V = 3850)
};
  
Crsf::Crsf(Serial* serial)
  : serial_(serial),
    frames_(0),
    idx_(0),
    start_time_(0),
    last_time_(0),
    serial_err_(0),
    crsf_err_(0),
    handlers_(NULL) {
  serial_->Setup(kDefaultBaud, 8, Serial::PARITY_NONE, 1, false,
                 /*use_alt_pins=*/false, Serial::MODE_RX);
  serial_->SetBuffered(true);
}

void Crsf::AddHandler(CrsfFrameHandler* handler) {
  if (handlers_) handler->AddHandler(handlers_);
  handlers_ = handler;
}

bool Crsf::Run() {
  while(serial_->Avail()) {
    ReadInfo info;
    serial_->Read(&info);
    if (info.err) {  // error reading serial data, so don't trust anything.
      DBG_MD(SBUS, ("Crsf serial read Error: %d\n", info.err));
      serial_err_++;
      idx_ = 0;
      continue;
    }
    const u8_t curr_time = info.time;
    // If time since last byte is > ~.2ms
    // or if total time is longer than a full frame
    // should take, then assume this starts a new frame.
    if (curr_time - last_time_ > 5 ||
        curr_time - start_time_ > kMaxElapsed) {
      idx_ = 0;
    }
    if (idx_ == 0) {
      start_time_ = curr_time;
    }
    last_time_ = curr_time;
    data_[idx_++] = info.data;
    if (idx_ < 3) continue;
    const u8_t len = data_[1] + 2;
    if (len > 64) {
      DBG_MD(SBUS, ("Crsf Illegal len sync: 0x%02X type:0x%02X len:%d\n",
		    data_[0], data_[2], len));
      crsf_err_++;
      idx_ = 0;
      continue;
    }
    if (idx_ < len) continue;

    // calcuate crc of type + payload
    const u8_t wire_crc = data_[len - 1];
    const u8_t calc_crc = crsf_crc8(data_ + 2, len - 3);
    if (wire_crc != calc_crc) {
      DBG_MD(SBUS, ("Crsf CRC mismatch len sync: 0x%02X type:0x%02X len:%d wire:0x%02X calc:0x%02X\n",
		    data_[0], data_[2], len, wire_crc, calc_crc));
      crsf_err_++;
      idx_ = 0;
      continue;
    }
    DBG_LO(SBUS, ("Crsf Frame sync: 0x%02X type: 0x%02X len: %d\n",
		  data_[0], data_[2], len));
    frames_++;
    idx_ = 0;
    return true;
  }
  return false;
}

bool Crsf::RunHandlers() {
  for (CrsfFrameHandler* handler = handlers_;
       handler != NULL; handler = handler->NextHandler()) {
    if (handler->HandleFrame(data_[2], data_[1], data_ + 3))
      return true;
  }
  return false;
}

void Crsf::Dump() const {
  DBG_LO(SBUS, ("Crsf frames:%d err: %d\n", frames_, crsf_err_));
}

bool CrsfRcHandler::HandleFrame(u8_t type, u8_t len, u8_t* payload) {
  if (type != type_) return false;
  if (len < kExpectedLen) return false;

  channels_.SetFailSafe(false);
  channels_.ChannelsUnpack(payload, 16);

  return true;
}

void CrsfRcHandler::Dump() const {
  DBG_LO(APP, ("RcHandler:"));
  for (int i=0; i<16; ++i) {
    DBG_LO(APP, (" 0x%03X", GetChannel(i)));
  }
  DBG_LO(APP, ("\n"));
}
