#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "Boot.h"
#include "Dbg.h"
#include "IntTypes.h"
#include "RtcTime.h"

#include "leds/Rgb.h"
#include "leds/Pixel.h"
#include "Pins.h"
#include "Serial.h"
#include "WS2812.h"

#define LED_PIN (2)
#define DSHOT_PIN (4)

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
  u16_t encoded = (throttle & ((1 << 11) - 1)) << 5;
  if (telemetry) encoded |= 0x10;
  u8_t crc = ((encoded >> 12) ^
              (encoded >>  8) ^
              (encoded >>  4)) & 0x0F;
  encoded |= crc;

  const u8_t bit_set = 1 << bit;
  const u8_t bit_mask = ~bit_set;
  dshot_mask |= bit_set;
  u16_t mask = 0x8000;
  for (u8_t i = 0; i < 16; ++i) {
    if (encoded & mask) {  // Logic reversed because we want to 'clear' just zeros early.
      dshot_data[i] = dshot_data[i] & bit_mask;
    } else {
      dshot_data[i] = dshot_data[i] | bit_set;
    }
    mask = mask >> 1;
  }
}

#define DSHOT_PORT (PORTA)
#define nop() __asm__ __volatile__ ( "nop\n" )

void SendDShot600() {
  const u8_t mask = dshot_mask;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (u8_t bit_cnt = 0; bit_cnt < 16; ++bit_cnt) {
      DSHOT_PORT.OUTSET = mask;
      nop();
      nop();
      nop();

      u8_t bits = dshot_data[bit_cnt];
    
      DSHOT_PORT.OUTCLR = bits;
      nop();
      nop();
      nop();
      nop();
      nop();
    
      DSHOT_PORT.OUTCLR = mask;
      nop();
    }
  }
}

void SendDShot300() {
  const u8_t mask = dshot_mask;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (u8_t bit_cnt = 0; bit_cnt < 16; ++bit_cnt) {
      DSHOT_PORT.OUTSET = mask;
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();

      u8_t bits = dshot_data[bit_cnt];
    
      DSHOT_PORT.OUTCLR = bits;
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
      nop();
    
      DSHOT_PORT.OUTCLR = mask;
      nop();
      nop();
      nop();
      nop();
      nop();
    }
  }
}

const u8_t nLeds = 7;
led::RGBW leds[nLeds];

struct KissTelemetry {
  i8_t temperature;
  u16_t volts;   // Volt * 100
  u16_t amps;  // Amps * 100
  u16_t mAh; // milliamp Hours consumed
  u16_t erpm;  // ERpm / 100
  u8_t crc;
  u8_t buffer[10];
  u8_t *GetBuffer() { return buffer; }

  bool Parse() {
    // One transmission will have 10 8-bit bytes sent with 115200 baud and 3.6V.
    // Byte 0: Temperature (Deg C)
    // Byte 1: Voltage high byte (Volt * 100)
    // Byte 2: Voltage low byte
    // Byte 3: Current high byte (Amp * 100)
    // Byte 4: Current low byte
    // Byte 5: Consumption high byte (mAh)
    // Byte 6: Consumption low byte
    // Byte 7: Rpm high byte (ERpm / 100)
    // Byte 8: Rpm low byte
    // Byte 9: 8-bit CRC
    u8_t rec_crc = 0;
    for (int i = 0; i < 9; ++i) {
      rec_crc ^= buffer[i];
      for (int j = 0; j < 8; ++j) {
	rec_crc = ((rec_crc & 0x80) ? 0x07 ^ (rec_crc << 1) : (rec_crc << 1)) ;
      }
    }
    if (rec_crc != buffer[9]) return false;
    temperature = buffer[0];
    volts = (buffer[1] << 8) | buffer[2];
    amps  = (buffer[3] << 8) | buffer[4];
    mAh   = (buffer[5] << 8) | buffer[6];
    erpm  = (buffer[7] << 8) | buffer[8];
    crc = buffer[9];
    return true;
  }
  void Print() {
    DBG_MD(APP, ("Telemetry T:%d V:%d.%02d A:%d.%02d mAh:%d eRMP:%d00\n",
                 temperature,
                 volts/100, volts%100,
                 amps/100, amps%100,
                 mAh, erpm));
  }
};

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/2, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);

  Fill(leds, nLeds, led::wclr::red);
  
  Serial* serial = &Serial::usart1;
  serial->Setup(115200, 8, Serial::PARITY_NONE, /*stop_bits=*/1,
                /*invert=*/false, /*alt_pins=*/false,
                Serial::MODE_RX, /*use_pullup=*/false, /*buffered=*/true);
 
  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();
  
  DSHOT_PORT.OUTCLR = 1 << DSHOT_PIN;  // Set PA4 Low
  DSHOT_PORT.DIR |= 1 << DSHOT_PIN;  // Set PA4 output
  DSHOT_PORT.PIN4CTRL = 0; //  PORT_PULLUPEN_bp;

  sei();
  DBG_MD(APP, ("Hello World: Test\n"));
  
  EncodeDshotState(DSHOT_PIN, 0, /*telemetry=*/false);

  u16_t nDshotSent = 0;
  const u16_t min_thr = 48;
  const u16_t max_thr = 48;
  i8_t thr_add = 1;
  u16_t thr = min_thr;
  int reset_cnt =0;
  u8_t rtc_5 =0;
  u8_t update_3 = 0;
  bool clear_telemetry = false;
  bool telemetry_bad = true;
  KissTelemetry telemetry;
  u8_t telemetry_time = 0;
  u8_t telemetry_byte = 255;
  long bytes_read = 0;
  while (1) {
    u16_t rtc = RTC.CNT;
    if (serial->Avail()) {
      if (telemetry_byte == 255) {
        telemetry_time = rtc;
        telemetry_byte = 0;
        telemetry_bad = false;
      }
      do {
        u8_t err;
        u8_t data = serial->Read(&err);
	bytes_read++;
        if (err) {
          telemetry_bad = true;
          DBG_MD(APP, ("Bad: %X\n", err));
        }
        if (telemetry_bad || telemetry_byte == 10) {
          continue;
        }
        u8_t now_rtc = rtc;
        if (now_rtc - telemetry_time > 48 ) {
          // Sending 10 bytes at 115200 should take ~1ms, so ignore after 1.5ms
          // Or if we already read 10 bytes ignore until we re-request
          // telemetry.
          DBG_MD(APP, ("Timeout\n"));
          continue;
        }
        telemetry.GetBuffer()[telemetry_byte++] = data;
        if (telemetry_byte == 10) {
          bool result = telemetry.Parse();
          DBG_MD(APP, ("Got: %s\n", result ? "Good" : "Bad"));
	  if (result) {
	    telemetry.Print();
	  } else {
	    u8_t* buf = telemetry.GetBuffer();
	    DBG_MD(APP,
		   ("Buffer: %02X %02X%02X %02X%02X %02X%02X %02X%02X %02X\n",
		    buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], 
		    buf[6], buf[7], buf[8], buf[9]));
	  }
        }
      } while (serial->Avail());
    }
          
    u8_t rtc_now_5 = rtc >> 5;
    if (rtc_now_5 == rtc_5) continue;
    SendDShot600();
    rtc_5 = rtc_now_5;
    if (clear_telemetry) {
      EncodeDshotState(DSHOT_PIN, thr, false);
      clear_telemetry = false;
    }
    nDshotSent++;
    if (reset_cnt < 1000) {
      reset_cnt++;
      continue;
    }
    if (reset_cnt == 1000) {
      reset_cnt++;
      EncodeDshotState(DSHOT_PIN, thr, false);
    }
        
    const unsigned long now = FastTimeMs();
    const u8_t now_3 = now >> 3;
    if (now_3 != update_3) {
      update_3 = now_3;
      thr += thr_add;
      if (thr > max_thr) { thr = max_thr; thr_add = -thr_add; }
      else if (thr < min_thr) { thr = min_thr; thr_add = -thr_add; }
      if (now_3 == 0xFF) {
	DBG_MD(APP, ("\n Requesting Telemetry[%0x.%0x - %u]: %d - %ld",
		     rtc>>5, rtc&0x1F, nDshotSent, thr, bytes_read));
	clear_telemetry = true;
	telemetry_byte = 255;
      }
      EncodeDshotState(DSHOT_PIN, thr, clear_telemetry);
    }
  }
}
