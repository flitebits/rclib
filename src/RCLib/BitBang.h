// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _BITBANG_H_
#define _BITBANG_H_

#include "Pins.h"

// This class handles bit banging out both WS2812/SK6812 data as well
// as DSHOT data.  This is needed because DShot data must be sent
// once approximately every millisecond, but sending more than ~30 leds
// takes at least 1ms:
// 34 leds * 3 bytes/led * 8 bits/byte * 1.2us/bit = 979us = .978ms
//
// You can trigger DShot via ISR, which normally would simply interrupt
// sending the led data, but it also takes longer than the reset time for
// leds (9-10us) the send the dshot packet:
// 16 bits * 1.6667 us/bit = 26.667us
//
// Thus this class is notified when to send the DShot packet via the
// DShot ISR. This class then bit bangs both led data and the dshot
// packet for two bytes and then goes back to just sending LED data.
// We use DShot600's slightly longer bit time of 1.6667us for both for
// those two bytes.
class BitBang {
 public:
  BitBang();  // for later setup via 'SetPort'
  // Configure the pin group (port) to use and what pins will be used.
  // It will configure those pins appropriately for output.
  BitBang(PinGroupId pin_group, u8_t pins);

  // Configure the pin group (port) and pins.
  void SetPort(PinGroupId pin_group, u8_t pins = 0x00);
  // Add a new pin for output.
  void AddPin(u8_t pin);

  // Send len bytes from ptr out on 'pin' using WS2812/SK6812 timing.
  // Scale is a global scale value applied to all bytes.  Zero = black
  // 255 = unchanged.
  void SendWS2812(u8_t pin, void* ptr, u16_t len, u8_t scale);
  // Send Dshot data out on the configured ping group.  dshot_data
  // must be 16 bytes, each byte is a bit in the dshot packet (one for
  // each pin).  These bytes have thier sense flipped (because they
  // signal what bits to set low early for zeros).  If bits not in
  // dshot mask are set then those bits will be manipulated.
  void SendDShot600(u8_t* dshot_data, u8_t dshot_mask);

protected:
  void SendWS2812Full(u8_t pin, void* ptr, u16_t len);
  void SendWS2812Scale(u8_t pin, void* ptr, u16_t len, u8_t scale);
  void DShot600WS2812(u8_t led1, u8_t led2, u8_t led_mask);

 private:
  PORT_t* port_;
  bool ws2812_in_progress_;
  u8_t dshot_mask_;
  u8_t* dshot_data_;
};

#endif  // _BITBANG_H_
