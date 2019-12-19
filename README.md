**ATMega4809 Basic

This is a basic example of bringing up the ATMega 4809 and enabling
the most common hardware. I wrote this so I could use it to control
various features in RC aircraft (telemetry and lights mostly).

It has support for:
  * SBus protocol (get channel information)
  * S.Port Protocol (only for volts/amps reporting)
  * DSHOT 600 (bit bang) to set motor speed and get telemetry from ESC.
  * Analog to Digital conversion (to read battery volts/amps).
  * PWM for controlling 'dumb' leds.
  * APA102 (via SPI) & WS2812 (via bit bang)
  * Serial used by several protocols above as well as for debugging.
  * Some simple RGB functions for LED control
  * RTC clock.

Flite Bits
flitebits@gmail.com
