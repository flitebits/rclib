// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include "SportSensor.h"

#include <stddef.h>

#include "Dbg.h"
#include "RtcTime.h"
#include "Util.h"

enum {
  FRAME_POLL_START = 0x7E,
  FRAME_SENSOR_START = 0x10,
  FRAME_SENSOR_NULL = 0x00,
  ESCAPE_BYTE =  0x7D
};

SportSensor::SportSensor(Serial* serial, bool invert, bool alt_pins,
			 bool use_pullup)
  : serial_(serial),
    prev_read_(0),
    crc_(0),
    poll_cnt_(1),
    num_entries_(0) {
  serial_->Setup(57600, 8, Serial::PARITY_NONE, /*stop_bits=*/1, invert,
		 alt_pins, Serial::MODE_TX_RX_1WIRE,
		 use_pullup = use_pullup);
  serial_->Disable(Serial::MODE_TX);
  serial_->Enable(Serial::MODE_RX);
}

i8_t SportSensor::AddSensor(u8_t sensor_id, u16_t data_id,
			    u16_t poll_time_ms) {
  u8_t poll_100ms = (poll_time_ms + (1 << 6)) >> 7;
  for (int i = 0; i < num_entries_; ++i) {
    Entry& entry = entries_[i];
    if (entry.sensor_id == sensor_id && entry.data_id == data_id) {
      entry.poll_interval_100ms = poll_100ms;
      return i;
    }
  }
  if (num_entries_ >= ARRAY_SIZE(entries_)) return -1;
  i8_t idx = num_entries_++;
  Entry& entry = entries_[idx];
  entry.sensor_id = sensor_id;
  entry.data_id = data_id;
  entry.poll_interval_100ms = poll_100ms;
  entry.most_recent_poll_cnt = 0;
  entry.most_recent_update_time = 0;
  return idx;
}

void SportSensor::AddFcs40Sensors(i8_t* current_idx, i8_t* volt_idx) {
  i8_t idx = AddSensor(SENSOR_FCS40, DATA_CURRENT_10x, 500);
  if (current_idx != NULL) *current_idx = idx;
  idx = AddSensor(SENSOR_FCS40, DATA_VOLTAGE_100x, 500);
  if (volt_idx != NULL) *volt_idx = idx;
}

void SportSensor::SetSensor(u8_t sensor_idx, u32_t value) {
  entries_[sensor_idx].value = value;
}

void SportSensor::SetSensor(u8_t sensor_id, u16_t data_id, u32_t value) {
  for (int i = 0; i < num_entries_; ++i) {
    Entry& entry = entries_[i];
    if (entry.sensor_id == sensor_id && entry.data_id == data_id) {
      entry.value = value;
      return;
    }
  }
}

void SportSensor::Run() {
  while(serial_->Avail()) ReadAndProcess();
}

void SportSensor::ReadAndProcess() {
  u8_t err = 0;
  u8_t data = serial_->Read(&err);
  if (err) {
    prev_read_ = 0;
    return;
  }
  if (prev_read_ != FRAME_POLL_START) {
     prev_read_ = data;
     return;
  }
  
  u8_t idx = num_entries_;
  u32_t now = FastTimeMs();
  u8_t now7 = now >> 7;  // down to 8ths of a second (125ms)
  u8_t poll_max_delta = 0;

  for (int i = 0; i < num_entries_; ++i) {
    Entry& entry = entries_[i];
    if (entry.sensor_id != data) continue;
    u8_t poll_delta = poll_cnt_ - entry.most_recent_poll_cnt;
    if (poll_delta > poll_max_delta) {
      poll_max_delta = poll_delta;
      idx = i;
    }
  } 
  if (idx >= num_entries_) return;  // We don't have that sensor

  Entry& entry = entries_[idx];
  entry.most_recent_poll_cnt = poll_cnt_++;
  u8_t elapsed_time = now7 - entry.most_recent_update_time;
  if (elapsed_time < entry.poll_interval_100ms) {
    SendNoUpdate(entry.data_id);
  } else {
    entry.most_recent_update_time = now7;
    SendEntry(idx);
  }

  DBG_MD(SPORT, ("%sUpdate Idx: %d SensorId: %d DataId %d val: %ld\n",
		 (elapsed_time < entry.poll_interval_100ms) ? "No " : "",
		 idx, entry.sensor_id, entry.data_id, entry.value));
}

void SportSensor::SendEntry(u8_t idx) {
  Entry& entry = entries_[idx];
  crc_ = 0;
  serial_->Disable(Serial::MODE_RX);
  serial_->Enable(Serial::MODE_TX);
  SendByte(FRAME_SENSOR_START);
  SendByte((entry.data_id      ) & 0xFF);
  SendByte((entry.data_id >>  8) & 0xFF);
  SendByte((entry.value          ) & 0xFF);
  SendByte((entry.value     >>  8) & 0xFF);
  SendByte((entry.value     >> 16) & 0xFF);
  SendByte((entry.value     >> 24) & 0xFF);
  SendByte(0xFF - crc_);
  serial_->FlushTx();
  serial_->Disable(Serial::MODE_TX);
  serial_->Enable(Serial::MODE_RX);
}

void SportSensor::SendNoUpdate(u8_t idx) {
  Entry& entry = entries_[idx];
  crc_ = 0;
  serial_->Disable(Serial::MODE_RX);
  serial_->Enable(Serial::MODE_TX);
  SendByte(FRAME_SENSOR_NULL);
  SendByte((entry.data_id      ) & 0xFF);
  SendByte((entry.data_id >>  8) & 0xFF);
  SendByte(0);
  SendByte(0);
  SendByte(0);
  SendByte(0);
  SendByte(0xFF - crc_);
  serial_->FlushTx();
  serial_->Disable(Serial::MODE_TX);
  serial_->Enable(Serial::MODE_RX);
}

void SportSensor::SendByte(u8_t val) {
  switch (val) {
  case FRAME_POLL_START:
  case ESCAPE_BYTE:
    serial_->WriteByte(ESCAPE_BYTE);
    serial_->WriteByte(val ^ 0x20);
    break;
  default:
    serial_->WriteByte(val);
    break;
  }
  i16_t crc_val = crc_;
  crc_val += val;
  crc_ = crc_val + (crc_val >> 8);
}
