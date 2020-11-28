// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef __KISSTELEMETRY_H__
#define __KISSTELEMETRY_H__

#include "IntTypes.h"
#include "Serial.h"

#define kKissTelementryLen (10)
struct KissTelemetry {
  static void ConfigureSerial(Serial* serial, bool alt_pin, bool one_wire);
  enum ParseResult { kSuccess = 0,
		     kNotReady = 1,
		     kCrcErr = 2 };
		    
  KissTelemetry();

  i8_t temperature;
  u16_t volts;   // Volt * 100
  u16_t amps;  // Amps * 100
  u16_t mAh; // milliamp Hours consumed
  u16_t erpm;  // ERpm / 100
  u8_t crc;
  u8_t *GetBuffer() { return buffer; }

  u8_t idx;
  u8_t buffer[16];
  void AddByte(u8_t val);
  void ResetBuffer();
  ParseResult Parse();
  void Print();
};

#endif //__KISSTELEMETRY_H__
