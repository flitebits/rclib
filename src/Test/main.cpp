#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "Boot.h"
#include "Dbg.h"
#include "DShot.h"
#include "IntTypes.h"
#include "RtcTime.h"

#include "leds/Rgb.h"
#include "leds/Pixel.h"
#include "Pins.h"
#include "Serial.h"
#include "WS2812.h"

#define LED_PIN (2)
#define DSHOT_PIN (4)

const u8_t nLeds = 7;
led::RGBW leds[nLeds];

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/2, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  DBG_LEVEL_HI(DSHOT);

  Fill(leds, nLeds, led::wclr::red);
  
  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();
  DShot dshot(&Serial::usart1, PORT_A);

  sei();
  DBG_MD(APP, ("Hello World: Test\n"));

  dshot.SetChannel(DSHOT_PIN, 0, false);

  const u16_t min_thr = 48;
  const u16_t max_thr = 48;
  i8_t thr_add = 1;
  u16_t thr = min_thr;
  int nDshotSent = 0;
  int reset_cnt = 0;
  u8_t update_3 = 0;
  while (1) {
    bool new_telemetry;
    if (dshot.Run(&new_telemetry)) {
      nDshotSent++;
      if (reset_cnt < 1000) {
	reset_cnt++;
	continue;
      }
      if (reset_cnt == 1000) {
	reset_cnt++;
	dshot.SetChannel(DSHOT_PIN, thr, false);
      }
    }
    if (new_telemetry) {
      dshot.GetTelemetry()->Print();
    }
          
    const unsigned long now = FastTimeMs();
    const u8_t now_3 = now >> 3;
    if (now_3 != update_3) {
      update_3 = now_3;
      thr += thr_add;
      if (thr > max_thr) { thr = max_thr; thr_add = -thr_add; }
      else if (thr < min_thr) { thr = min_thr; thr_add = -thr_add; }
      bool request_telemetry = false;
      if (now_3 == 0xFF) {
	DBG_MD(APP, ("\n Requesting Telemetry[%u]: %d", nDshotSent, thr));
	request_telemetry = true;
      }
      dshot.SetChannel(DSHOT_PIN, thr, request_telemetry);
    }
  }
}
