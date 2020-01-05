// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _ADC_
#define _ADC_

#include "Pins.h"

class Adc {
 public:
  enum VRefSrc {
	  VREF_055,  // 0.55V
	  VREF_11,   // 1.1V
	  VREF_15,   // 1.5V
	  VREF_25,   // 2.5V
	  VREF_43,   // 4.3V
	  VREF_VREFA, // VREFA pin
	  VREF_VDD,  // Chip input Voltage
  };
	  
  // Configures Adc for just under 1Mhz sample, 10bit, Vrefa external
  // ref, and enables Adc unit.
  Adc(VRefSrc src = VREF_VDD);
  void Disable();  // Disable ADC (save power)
  void Enable();  // Enable ADC

  // Do Analog to Digital conversion for given pin.
  int Read(PinId pin) { return Read(GetAnalogIdx(pin)); }
  int Read(i8_t ain_idx) {
    StartRead(ain_idx);
    return FinishRead();
  }
  // Start an Analog to Digital conversion for a pin.
  void StartRead(PinId pin) { return StartRead(GetAnalogIdx(pin)); }
  void StartRead(i8_t ain_idx);
  // Returns value from most recent StartRead call
  int FinishRead();
  // Returns value from most recent StartRead call, and starts a new
  // conversion on the same pin.
  int ConinueRead();

  // Configures the given pin for analog reading (sets it to input only
  // and sets the assocaited PINCTRL register to all zeros).
  static void ConfigurePin(PinId pin);
  static void ConfigurePin(i8_t ain_idx) {
    ConfigurePin(GetAnalogPin(ain_idx)); }
};

#endif  // _ADC_
