// Copyright 2026 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _CRSF_
#define _CRSF_

#include <stddef.h>

#include "IntTypes.h"
#include "RcChannels.h"
#include "RtcTime.h"
#include "Serial.h"

class CrsfFrameHandler;

class Crsf {
 public:
  Crsf(Serial* serial);

  // Read any data available on serial port, returns true if a frame
  // was completed and the channel values are updated.
  bool Run();
  // Runs registered handlers until one of then returns true then
  // returns true.  If no handler returns true this returns false.
  bool RunHandlers();

  // Registers handler to be called from RunHandlers.  Passed
  // handler must outlive this object.
  void AddHandler(CrsfFrameHandler* handler);

  // Print debug info
  void Dump() const;

 protected:
  Serial* serial_;
  u16_t frames_;    // Total number of frames decoded (for debugging)
  u8_t idx_;        // Number of bytes in frame so far
  u8_t start_time_; // Start time of packet
  u8_t last_time_;  // Last byte time
  u8_t serial_err_;   // number of corrupt serial packets
  u8_t crsf_err_;   // number of corrupt crsf packets
  CrsfFrameHandler* handlers_;
  u8_t data_[64];
};

class CrsfFrameHandler {
 public:
 CrsfFrameHandler(u8_t type) :
  type_(type), next_handler_(NULL) { }
  CrsfFrameHandler* NextHandler() { return next_handler_; }
  void AddHandler(CrsfFrameHandler* handler) { 
    next_handler_ = handler;
  }

  // Returns true if this handler handled the payload.
  virtual bool HandleFrame(u8_t type, u8_t len, u8_t* payload) = 0;

 protected:
  u8_t type_;
  CrsfFrameHandler* next_handler_;
};

class CrsfRcHandler : public CrsfFrameHandler {
 public:
  static const u8_t kType = 0x16;
  static const u8_t kExpectedLen = 22;
  CrsfRcHandler() : CrsfFrameHandler(kType) { }

  u16_t GetChannel(u8_t ch) const { return channels_.GetChannel(ch); }
  const RcChannels& GetChannels() const { return channels_; }
  
  virtual bool HandleFrame(u8_t type, u8_t len, u8_t* payload);
  void Dump() const;

 protected:  
  RcChannels channels_;
};

#endif  // _CRSF_
