// Copyright 2026 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _RC_CHANNELS_
#define _RC_CHANNELS_

#include <stddef.h>

#include "IntTypes.h"

// Convert RC channel value to three position switch values (0-2)
// threshold and 2047 - threshold is where values change.
u8_t ThreePosSwitch(i16_t val, i16_t threshold = 667);
  
class RcChannels {
 public:
 RcChannels() : failSafe_(true) { }

  bool FailSafe() const { return failSafe_; }
  // The value for one channel.
  u16_t GetChannel(int ch) const { return channels_[ch];}
  
  void SetFailSafe(bool val) { failSafe_ = val; }
  u16_t* GetChannelArray() { return channels_; }

  void Dump() const;

  // data should be 11bit per channel data bit packed, this will unpack num_ch
  // into channels_.
  void ChannelsUnpack(u8_t* data, u8_t num_ch);
  // Rescales data in channels_ so min goes to zero and max goes to 2047.
  void RescaleChannels(u16_t min, u16_t max);

 protected:
  u16_t channels_[16];
  bool failSafe_;
};

#endif  // _RC_CHANNELS_
