#ifndef _PWM_
#define _PWM_

#include "IntTypes.h"
#include "Pins.h"

class Pwm {
public:
  Pwm(PinGroupId port, int freqHz);
  // Index of pwm to enable, 0-5.  This also configures the pin for output.
  // Does not affect the pins associated PINCTRL register.
  void Enable(u8_t idx);
  void Disable(u8_t idx);
  // Set PWM pin's duty cycle to 'val';
  // 0 is off all the time 255 is on all the time.
  void Set(u8_t idx, u8_t val);

  PORT_t* port_ptr() { return port_ptr_; }

private:
  PORT_t *port_ptr_;
};

#endif  // _PWM_
