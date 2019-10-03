#include <avr/io.h>
#include <avr/interrupt.h>

#include "Boot.h"
#include "Dbg.h"
#include "IntTypes.h"
#include "RtcTime.h"

#include "Serial.h"
#include "Spi.h"

#include "leds/Clr.h"
#include "leds/Hsv.h"
#include "leds/Pixel.h"
#include "leds/Rgb.h"

#define NLED_R3 (24)
#define NLED_R2 (16)
#define NLED_R1 (12)
#define NLED_R0 (8)
#define NLED_ALL (NLED_R3 + NLED_R2 + NLED_R1 + NLED_R0)
typedef led::RGBW pix_t;
pix_t rgb_data[NLED_ALL];

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/2, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_HI(LED);

  led::Fill(rgb_data, NLED_ALL, led::wclr::black);
  Spi::spi.SetupSK6812(Spi::PINS_PA47);

  PORTF.OUT  = 0;  // set all PORTF pins to zero.
  PORTF.DIR  |= ((1 << 6) | (1 << 2));  // Pins PF2 & PF6 set as output pints.
  
  // Enable interrupts (end of init, real program begin).
  sei();
  Spi::spi.UpdateLeds(rgb_data, NLED_ALL, 0x08);
  
  u8_t hue_pos = 0;
  u8_t update_9 = 0;  // ~1/2 sec
  u8_t update_4 = 0;  // ~1/64 sec
  while (true) {
    unsigned long now = FastTimeMs();
    u8_t now_9 = now >> 9;
    if (now_9 != update_9) {  // Simple heart beat 
      update_9 = now_9;
      PORTF.OUTTGL = (1 << 6);  // Pin PF6 toggle
    }

    u8_t now_4 = now >> 4;
    if (now_4 != update_4) {
      update_4 = now_4;
      Spi::spi.WaitForIdle();
      hue_pos++;
      led::HSV hsv(hue_pos, 0xFF, 0xFF);
      led::RGBW rgbw = HsvToRgbw(hsv);
      led::Fill(rgb_data, NLED_ALL, rgbw);
      Spi::spi.UpdateLeds(rgb_data, NLED_ALL, 0x08);
    }
  }  
}

