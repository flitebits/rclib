#ifndef _SBUS_
#define _SBUS_

#include "IntTypes.h"
#include "RtcTime.h"
#include "Serial.h"

class SBus {
public:
  SBus(Serial* serial, bool invert=true);  // Normal SBus is inverted serial

  // Read any data available on serial port, returns true if a frame
  // was completed and the channel values are updated.
  bool Run();
  
  bool FailSafe() { return failSafe_; }
  // The value for one channel.
  u16_t GetChannel(int ch) { return rcVal_[ch];}

  // Convert RC channel value to three position switch values (0-2)
  // threshold is where values change.
  static int ThreePosSwitch(short val, short threshold = 667);

  int GetBytesRead() { return bytes_read_; }
  int GetDataFrames() { return frames_; }

  void Dump();
  
protected:
  // Get 11 bist from the 'data' array starting at start_byte + start_bit
  int GetBits(int start_byte, int start_bit);
  
private:
  Serial* const serial_;
  u8_t idx_;  // Current index to update in data and time.
  u8_t data_[32];  // Last 32 samples read from serial port
  u8_t time_[32];  // The time the last 32 bytes were read for frame detection

  bool failSafe_;  // True if receiver is in failsafe mode
  u16_t rcVal_[16];  // The most recent values for each channel
  u16_t bytes_read_;  // Total number of bytes read (for debugging)
  u16_t frames_;  // Total number of frames decoded (for debugging)
};

#endif  // _SBUS_
