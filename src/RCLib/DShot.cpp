// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include "DShot.h"
#include "DShotASM.h"
#include "Pins.h"
#include "Util.h"

#include <util/atomic.h>
BitBang gBitBang;

DShot::DShot(Serial* serial, PinGroupId pin_group)
  : dshot_mask_(0),
    serial_(serial),
    telemetry_status_(1),
    telemetry_time_(0),
    telemetry_bad_(false),
    telemetry_err_cnt_(0),
    telemetry_fail_cnt_(0)
{
  gBitBang.SetPort(pin_group);
  bitbang_ = &gBitBang;
  memset(dshot_data_, 0, sizeof(dshot_data_));
  last_cnt_5_ = RTC.CNT;
  KissTelemetry::ConfigureSerial(serial, false, false);
}

DShot::DShot(Serial* serial, BitBang* bitbang)
  : dshot_mask_(0),
    serial_(serial),
    bitbang_(bitbang),
    telemetry_status_(1),
    telemetry_time_(0),
    telemetry_bad_(false),
    telemetry_err_cnt_(0),
    telemetry_fail_cnt_(0)
{
  memset(dshot_data_, 0, sizeof(dshot_data_));
  last_cnt_5_ = RTC.CNT;
  KissTelemetry::ConfigureSerial(serial, false, false);
}

bool DShot::CheckSerial() {
  if (!serial_->Avail()) return false;
    
  if (telemetry_status_ == 1) {
    telemetry_time_ = RTC.CNT;
    telemetry_status_ = 0;
    telemetry_bad_ = false;
  } else if (telemetry_status_ == 2) {
    while (serial_->Avail()) {
      u8_t err;
      serial_->Read(&err);
    }
    return false;
  }
  do {
    u8_t err;
    u8_t data = serial_->Read(&err);
    if (err) {
      DBG_MD(DSHOT, ("DSHOT: Err\n"));
      ++telemetry_err_cnt_;
    }
    if (telemetry_bad_) {
      telemetry_.ResetBuffer();
      continue;
    }

    u8_t now_rtc = RTC.CNT;
    if (now_rtc - telemetry_time_ > 100) {
      // Sending 10 bytes at 115200 should take ~1ms, so ignore after 3ms
      // until we re-request telemetry.
      telemetry_bad_ = true;
      continue;
    }
    telemetry_.AddByte(data);

    KissTelemetry::ParseResult result = telemetry_.Parse();
    if (result == KissTelemetry::kSuccess) {
      telemetry_status_ = 2;
      return true;
    }
    if (result == KissTelemetry::kNotReady) { continue; }
    DBG_LO(DSHOT, ("DSHOT: Failed parse\n"));
  } while (serial_->Avail());

  DBG_HI(DSHOT, ("Buffer:"));
  const u8_t* buf = telemetry_.GetBuffer();
  for(int i=0; i < 16; ++ i) {
    DBG_HI(DSHOT, ("%02X", buf[i]));
  }
  DBG_HI(DSHOT, ("\n"));
  return false;
}

void DShot::ClearTelemetryReq() {
  // We are going to clear the bits in the telemetry bit so fix the
  // CRC 1/2 byte first. Then set the telemetry bits to zero for all
  // channels.
  dshot_data_[15] = ~(dshot_data_[15] ^ dshot_data_[11]);
  dshot_data_[11] = 0xFF;
}

bool DShot::Run(bool* new_telemetry) {
  u16_t rtc_cnt = RTC.CNT;
  u8_t rtc_cnt_5 = rtc_cnt >> 5;
  bool sent_dshot = false;;
  if (last_cnt_5_ != rtc_cnt_5) {
    last_cnt_5_ = rtc_cnt_5;
    SendDShot600();
    sent_dshot = true;
    ClearTelemetryReq();
  }
  bool got_telemetry = CheckSerial();
  if (new_telemetry) {
    *new_telemetry = got_telemetry;
  }
  return sent_dshot;
}

void DShot::ClearChannel(u8_t bit) {
  const u8_t bit_mask = ~(1 << bit);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { 
    dshot_mask_ &= bit_mask;
    for (u8_t i = 0; i < 16; ++i) {
      dshot_data_[i] &= bit_mask;
    }
  }
}

void DShot::SetChannel(u8_t bit, u16_t value, bool telemetry) {
  if (bit >= 8) return;  // Only 0-7 possible

  const u8_t bit_set = 1 << bit;
  const u8_t bit_clr = ~bit_set;
  // You should always wait until you get telemtry back from one channel
  // before requesting telemetry from another channel.
  DBG_LO_IF(DSHOT, telemetry && (!telemetry_bad_) && (telemetry_status_ == 0),
            ("DSHOT: Telemetry requested while one pending\n"));

  u16_t encoded = (value & ((1 << 11) - 1)) << 5;
  if (telemetry) {
    encoded |= 0x10;
  }
  u8_t crc = encoded;
  crc ^= (encoded >>  8);
  crc ^= (crc >> 4);
  encoded |= (crc & 0xF);
  
  u16_t mask = 0x8000;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { 
    if (telemetry) {
      telemetry_status_ = 1;
    }
    if ((dshot_mask_ & bit_set) == 0) {
      dshot_mask_ |= bit_set;
      bitbang_->AddPin(bit);
    }
  
    for (u8_t i = 0; i < 16; ++i) {
      if (encoded & mask) {  // Reversed, we want to 'clear' just zero bits.
        dshot_data_[i] &= bit_clr;
       } else {
        dshot_data_[i] |= bit_set;
      }
      mask = mask >> 1;
    }
  }
}

void DShot::SendDShot600() {
  bitbang_->SendDShot600(dshot_data_, dshot_mask_);
}
