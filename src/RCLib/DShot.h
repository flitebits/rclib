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

// This implements DShot600 protocol including requesting telemetry.
// Note that this is incompatible with long WS2812/SK6812 Led strings.
// It can take several milliseconds to send a long LED string (1ms per
// 25-30 leds) and DShot requires updates every 1-2ms.
//
// There is an ISR version of this DShot code but that blocks the
// sending of the Led data long enough that the led strip restarts at
// the first LED (the spec sheet says you must be low for 50+us, which
// wouldn't be a problem, but in fact they reset after more like 5us).
// For shorter strings (perhaps up to ~50) leds you can simply call
// DShot between updates of the LEDs (don't use the ISR version).
class DShot {
 public:
  DShot(Serial* serial, PinGroupId pins);
  virtual void Start() { }
  
  // Returns true if it send a DShot blast.
  // Sets new_telemetry to true if it detected new telemetry available.
  virtual bool Run(bool* new_telemetry);

  void SetChannel(u8_t pin, u16_t value, bool request_telemetry);
  void ClearChannel(u8_t pin);
  KissTelemetry* GetTelemetry() { return &telemetry_; }

 protected:
  void SendDShot600();
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
