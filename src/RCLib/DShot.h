#ifndef _DSHOT_
#define _DSHOT_

#include "Dbg.h"
#include "IntTypes.h"
#include "Pins.h"
#include "Serial.h"

struct KissTelemetry {
  i8_t temperature;
  u16_t volts;   // Volt * 100
  u16_t amps;  // Amps * 100
  u16_t mAh; // milliamp Hours consumed
  u16_t erpm;  // ERpm / 100
  u8_t crc;
  u8_t buffer[10];
  u8_t *GetBuffer() { return buffer; }

  bool Parse();
  void Print();
};

class DShot {
 public:
  DShot(Serial* serial, PinGroupId pins);

  // Returns true if it send a DShot blast.
  // Sets new_telemetry to true if it detected new telemetry available.
  bool Run(bool* new_telemetry);
  
  void SetChannel(u8_t pin, u16_t value, bool request_telemetry);
  void ClearChannel(u8_t pin);
  KissTelemetry* GetTelemetry() { return &telemetry_; }

 protected:
  void SendDShot600() __attribute__((optimize("O1")));;
  bool CheckSerial();
  void ClearTelemetryReq();
  
  u8_t dshot_mask_;
  u8_t dshot_data_[16];
  u8_t last_cnt_5_;
  Serial* serial_;
  PORT_t* port_;

  KissTelemetry telemetry_;
  u8_t telemetry_request_mask_;
  u8_t telemetry_awaiting_mask_;
  u8_t telemetry_byte_;
  u8_t telemetry_time_;
  bool telemetry_bad_;
  int telemetry_bad_cnt_;
  int telemetry_fail_cnt_;
};

#endif  // _DSHOT_
