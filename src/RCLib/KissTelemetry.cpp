// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include "KissTelemetry.h"

#include "Dbg.h"

KissTelemetry::KissTelemetry()
  : idx(0) { }

void KissTelemetry::AddByte(u8_t val) {
  buffer[idx & 0xF] = val;
  idx++;
  if (idx & 0x80) idx = 16 + (idx & 0xF);
}

void KissTelemetry::ResetBuffer() {
  idx = 0;
}

KissTelemetry::ParseResult KissTelemetry::Parse() {
  // One transmission will have 10 8-bit bytes sent with 115200 baud and 3.6V.
  // Byte 0: Temperature (Deg C)
  // Byte 1: Voltage high byte (Volt * 100)
  // Byte 2: Voltage low byte
  // Byte 3: Current high byte (Amp * 100)
  // Byte 4: Current low byte
  // Byte 5: Consumption high byte (mAh)
  // Byte 6: Consumption low byte
  // Byte 7: Rpm high byte (ERpm / 100)
  // Byte 8: Rpm low byte
  // Byte 9: 8-bit CRC
  if (idx < 10) return kNotReady;
  u8_t start_byte = idx - 10;
  u8_t rec_crc = 0;
  DBG_HI(DSHOT,("Start: %d Bytes: ", start_byte));
  for (int i = 0; i < 9; ++i) {
    u8_t b = buffer[(start_byte + i) & 0xF];
    DBG_HI(DSHOT,("%02x ", b));
    rec_crc ^= b;
    for (int j = 0; j < 8; ++j) {
      rec_crc = ((rec_crc & 0x80) ? 0x07 ^ (rec_crc << 1) : (rec_crc << 1));
    }
  }
  DBG_HI(DSHOT,("\n"));

  u8_t buf_crc = buffer[(start_byte + 9) & 0xF];
  if (rec_crc != buf_crc) {
    DBG_MD(DSHOT, ("CRC mismatch, rec: %02X found: %02X\n", rec_crc, buf_crc));
    return kCrcErr;
  }
  temperature = buffer[start_byte];
  volts = ((buffer[(start_byte + 1) & 0xF] << 8) |
	   buffer[(start_byte + 2) & 0xF]);
  amps  = ((buffer[(start_byte + 3) & 0xF] << 8) |
	   buffer[(start_byte + 4) & 0xF]);
  mAh   = ((buffer[(start_byte + 5) & 0xF] << 8) |
	   buffer[(start_byte + 6) & 0xF]);
  erpm  = ((buffer[(start_byte + 7) & 0xF] << 8) |
	   buffer[(start_byte + 8) & 0xF]);
  crc = buffer[(start_byte + 9) & 0xF];
  return kSuccess;
}

void KissTelemetry::Print() {
  DBG_LO(DSHOT, ("Telemetry T:%d V:%d.%02d A:%d.%02d mAh:%d eRMP:%d00\n",
               temperature,
               volts/100, volts%100,
               amps/100, amps%100,
               mAh, erpm));
}

void KissTelemetry::ConfigureSerial(Serial* serial,
				    bool alt_pin,
				    bool one_wire) {
  serial->Setup(115200, 8, Serial::PARITY_NONE, /*stop_bits=*/1,
		/*invert=*/false, alt_pin,
		one_wire ? Serial::MODE_1WIRE : Serial::MODE_RX,
		/*use_pullup=*/true, /*buffered=*/true);
  if (one_wire) serial->Disable(Serial::MODE_TX);
  serial->Enable(Serial::MODE_RX);
}
