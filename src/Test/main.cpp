#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "Boot.h"
#include "Dbg.h"
#include "IntTypes.h"
#include "RtcTime.h"

#include "Serial.h"

#define LED_PIN (6)

u8_t dshot_mask = 0;
u8_t dshot_data[16];

void ClearDshotState(u8_t bit) {
  const u8_t bit_mask = ~(1 << bit);
  dshot_mask &= bit_mask;
  for (u8_t i = 0; i < 16; ++i) {
    dshot_data[i] &= bit_mask;
  }
}

void EncodeDshotState(u8_t bit, u16_t throttle, bool telemetry) {
  u16_t encoded = throttle << 5;
  if (telemetry) encoded |= 0x10;
  u8_t crc = (((encoded >> 12) & 0x0F) ^
	      ((encoded >>  8) & 0x0F) ^
	      ((encoded >>  4) & 0x0F));
  encoded |= crc;

  const u8_t bit_set = 1 << bit;
  const u8_t bit_mask = ~bit_set;
  dshot_mask |= bit_set;
  u16_t mask = 0x8000;
  for (u8_t i = 0; i < 16; ++i) {
    dshot_data[i] = (dshot_data[i] & bit_mask) |
      ((encoded & mask) ? 0 : bit_set);
    mask = mask >> 1;
  }
}

void SendDShot300() {
  u8_t mask = dshot_mask;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (u8_t bit_cnt = 0; bit_cnt < 16; ++bit_cnt) {
      PORTC.OUTSET = mask;  // 25 cycles 
      PORTC.OUTSET = mask;
      PORTC.OUTSET = mask;
      PORTC.OUTSET = mask;
      u8_t bits = dshot_data[bit_cnt];
    
      PORTC.OUTCLR = bits; // 25 cycles
      PORTC.OUTCLR = bits;
      PORTC.OUTCLR = bits;
      PORTC.OUTCLR = bits;
      PORTC.OUTCLR = bits;
      PORTC.OUTCLR = bits;
    
      PORTC.OUTCLR = mask;  // 17 cycles
    }
  }
}

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/4, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  
  Serial* serial = &Serial::usart3;
  serial->Setup(115200, 8, Serial::PARITY_NONE, /*stop_bits=*/1,
		/*invert=*/false, /*alt_pins=*/false,
		Serial::MODE_RX, /*use_pullup=*/false);

  PORTF.OUT  = 0;  // Set all Low
  PORTF.DIR  |= (1 << LED_PIN);  // Set LED blink pin as output.
  
  PORTC.OUT  = 0;  // Set all Low
  PORTC.DIR  |= 0xFF;  // Set high 4 pins as output.

  sei();
  DBG_MD(APP, ("Hello World: Test\n"));
  
  EncodeDshotState(1, 0, 0);
  //EncodeDshotState(1, 22, true);

  u8_t update_8 = 0;  // ~1/4 sec
  u8_t update_3 = 0;  // ~8 millisec
  while (1) {
    const unsigned long now = FastTimeMs();
    const u8_t now_8 = now >> 8; // 1/4 sec (1024 / 256)
    if (now_8 != update_8) {
      update_8 = now_8;
      PORTF.OUTTGL = (1 << LED_PIN);  // Toggle led pin, simple heartbeat.
      DBG_MD(APP, ("Blink\n"));
      DBG_MD(APP, ("\n Sending DSHOT:"));
      SendDShot300();
    }

    if (now > (6 << 10)) {
      EncodeDshotState(1, 0, true);
    } else if (now > (5 << 10)) {
		  EncodeDshotState(1, 50, false);
	}

    const u8_t now_3 = now >> 3;
    if (now_3 != update_3) {
      update_3 = now_3;
    }
    if (serial->Avail()) {
      bool err;
      u8_t data = serial->Read(&err);
      if (err) {
	DBG_MD(APP, ("---- "));
      } else {
	DBG_MD(APP, ("0x%02X ", data));
      }
    }
  }
}

