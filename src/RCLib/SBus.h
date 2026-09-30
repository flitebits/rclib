// Copyright 2020 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _SBUS_
#define _SBUS_

#include "IntTypes.h"
#include "RcChannels.h"
#include "Serial.h"

class SBus {
public:
  SBus(Serial* serial, bool invert=true);  // Normal SBus is inverted serial

  // Read any data available on serial port, returns true if a frame
  // was completed and the channel values are updated.
  bool Run();

  RcChannels& GetChannels() { return channels_; }
  
  bool FailSafe() const { return channels_.FailSafe(); }
  // The value for one channel.
  u16_t GetChannel(int ch) const {
    return channels_.GetChannel(ch);
  }

  int GetBytesRead() const { return bytes_read_; }
  int GetDataFrames() const { return frames_; }

  void Dump() const;

protected:
  // Get 11 bist from the 'data' array starting at start_byte + start_bit
  int GetBits(int start_byte, int start_bit);

private:
  Serial* const serial_;
  u8_t idx_;  // Current index to update in data and time.
  u8_t data_[32];  // Last 32 samples read from serial port
  u8_t time_[32];  // The time the last 32 bytes were read for frame detection

  u16_t bytes_read_;  // Total number of bytes read (for debugging)
  u16_t frames_;  // Total number of frames decoded (for debugging)

  RcChannels channels_;
};

#endif  // _SBUS_
