void setup() {
  // You can also use a Serial port to control SK6812 leds, but the
  // implementation with SPI is more efficient since it's entirely
  // interrupt driven (something similar could be done with serials
  // but it would probably complicate the base serial interface a
  // bit).
  // 
  // Note that in serial async mode you can only get to 1/8 of Pclock,
  // which means that Pclock has to be the full CPU clock (20Mhz) to
  // get close enough (So pdiv disabled in clkctrl).  You can get
  // closer to Pclock in sync mode (in this mode the serial act
  // basically like ISP with a few features disabled).
  Serial::usart2.Setup(3333333, 6, Serial::PARITY_NONE, 1,
		       /*invert=*/true, /*use_alt_pins=*/true,
		       Serial::MODE_TX, /*use_2x_mode=*/true);
}

SendLeds() {
  static u8_t led_offset = 0;
  // This uses a similar method to SPI to control SK6812 leds but
  // is quite a bit more complex.  In particular we use the serial
  // start/end bits as part of the signal so we have to invert it
  // (serial start is low and serial end is high normally, but
  // SK6812 protocol for a bit starts with high and ends with
  // low). I'd rather use the SPI than a Serial port, so this is
  // more of proof of concept.
  static const u8_t clrs[] =
    {
     0xFF, 0x00, 0x00, 0x00, 
     0x00, 0xFF, 0x00, 0x00,
     0x00, 0x00, 0xFF, 0x00,
     0x00, 0x00, 0x00, 0xFF,
     0x80, 0x80, 0x80, 0x80,
    };
  const u8_t* clr_ptr = clrs + (led_offset << 2);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 4; ++j) {
      u8_t val[4];
      u8_t clr = ~(*clr_ptr++);
      val[3] = 0b100110 | ((clr & 0b10) >> 1) | ((clr & 0b01) << 4);
      clr = clr >> 2;
      val[2] = 0b100110 | ((clr & 0b10) >> 1) | ((clr & 0b01) << 4);
      clr = clr >> 2;
      val[1] = 0b100110 | ((clr & 0b10) >> 1) | ((clr & 0b01) << 4);
      clr = clr >> 2;
      val[0] = 0b100110 | ((clr & 0b10) >> 1) | ((clr & 0b01) << 4);
      Serial::usart2.WriteByte(val[0]);
      Serial::usart2.WriteByte(val[1]);
      Serial::usart2.WriteByte(val[2]);
      Serial::usart2.WriteByte(val[3]);
    }
    if (clr_ptr >= clrs + 20) clr_ptr = clrs;
  }
  ++led_offset;
  if (led_offset >= 5) led_offset = 0;
}

