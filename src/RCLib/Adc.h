#ifndef _ADC_
#define _ADC_

#include "Pins.h"

class Adc {
 public:
  // Configures Adc for just under 1Mhz sample, 10bit, Vrefa external
  // ref, and enables Adc unit.
  Adc();
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
