*RCLib*

This is a basic example of bringing up the ATMega 4808/4809 and
enabling the most common hardware. I wrote this so I could use it to
control various features in RC aircraft (telemetry and lights mostly).

I also have some small board designs for the 4808/4809 that I may
include as well.

It has support for:
  * SBus protocol (channel information from radio)
  * S.Port Protocol (only for volts/amps reporting)
  * DSHOT 600 (bit bang) to set motor speed and get telemetry from ESC
  * Analog to Digital conversion (to read battery volts/amps)
  * PWM for controlling 'dumb' leds
  * APA102 (via SPI) & WS2812 (via bit bang)
  * Serial used by several protocols above as well as for debugging
  * Some simple RGB functions for LED control
  * Very minimal debugging log/print support
  * RTC clock

Licensed under the Apache 2.0 license (see the file LICENSE-2.0 for
details).  Please let me know if you decide to use it.

Flite Bits
flitebits@gmail.com
