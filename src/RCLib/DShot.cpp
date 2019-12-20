#include "DShot.h"
#include "Pins.h"
#include "Util.h"

#include <util/atomic.h>

bool KissTelemetry::Parse() {
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

void KissTelemetry::Print() {
  DBG_LO(DSHOT, ("Telemetry T:%d V:%d.%02d A:%d.%02d mAh:%d eRMP:%d00\n",
	       temperature,
	       volts/100, volts%100,
	       amps/100, amps%100,
	       mAh, erpm));
}

DShot::DShot(Serial* serial, PinGroupId pins)
  : dshot_mask_(0),
    serial_(serial),
    port_(GetPortStruct(pins)),
    telemetry_byte_(10),
    telemetry_time_(0),
    telemetry_bad_(false),
    telemetry_bad_cnt_(0),
    telemetry_fail_cnt_(0)
{
  memset(dshot_data_, 0, sizeof(dshot_data_));
  last_cnt_5_ = RTC.CNT;
  serial_->Setup(115200, 8, Serial::PARITY_NONE, /*stop_bits=*/1,
		 /*invert=*/false, /*alt_pins=*/false,
		 Serial::MODE_RX, /*use_pullup=*/false, /*buffered=*/true);
}

bool DShot::CheckSerial() {
  if (!serial_->Avail()) return false;
  if (telemetry_byte_ == 255) {
    telemetry_time_ = RTC.CNT;
    telemetry_byte_ = 0;
    telemetry_bad_ = false;
  }
  do {
    u8_t err;
    u8_t data = serial_->Read(&err);
    if (err) {
      DBG_MD(DSHOT, ("DSHOT: Err\n"));
      telemetry_bad_ = true;
      ++telemetry_bad_cnt_;
    }
    if (telemetry_bad_ || telemetry_byte_ == 10) {
      continue;
    }

    u8_t now_rtc = RTC.CNT;
    if (now_rtc - telemetry_time_ > 48 ) {
      // Sending 10 bytes at 115200 should take ~1ms, so ignore after 1.5ms
      // Or if we already read 10 bytes ignore until we re-request
      // telemetry.
      telemetry_byte_ = 10;
      continue;
    }
    telemetry_.GetBuffer()[telemetry_byte_++] = data;
    if (telemetry_byte_ != 10) continue;

    bool result = telemetry_.Parse();
    DBG_LO_IF(DSHOT, !result, ("DSHOT: Failed parse\n"));
    if (result) return true;

    telemetry_bad_ = true;
    ++telemetry_bad_cnt_;
    const u8_t* buf = telemetry_.GetBuffer();
    DBG_HI(DSHOT,
	   ("Buffer: %02X %02X%02X %02X%02X %02X%02X %02X%02X %02X\n",
	    buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], 
	    buf[6], buf[7], buf[8], buf[9]));
    return false;
  } while (serial_->Avail());
  return false;
}

void DShot::ClearTelemetryReq() {
  // We are going to clear the bits in the telemetry bit so fix the
  // CRC 1/2 byte first. Then set the telemetry bits to zero for all
  // channels.
  dshot_data_[15] = ~(~dshot_data_[15] ^ ~dshot_data_[11]);
  dshot_data_[11] = 0xFF;
}

bool DShot::Run(bool* new_telemetry) {
  u16_t rtc_cnt = RTC.CNT;
  u8_t rtc_cnt_5 = rtc_cnt >> 5;
  bool sent_dshot = false;;
  if (last_cnt_5_ != rtc_cnt_5) {
    last_cnt_5_ = rtc_cnt_5;
    SendDShot600();
    sent_dshot = true;
    ClearTelemetryReq();
  }
  bool got_telemetry = CheckSerial();
  if (new_telemetry) {
    *new_telemetry = got_telemetry;
  }
  return sent_dshot;
}

void DShot::ClearChannel(u8_t bit) {
  const u8_t bit_mask = ~(1 << bit);
  dshot_mask_ &= bit_mask;
  for (u8_t i = 0; i < 16; ++i) {
    dshot_data_[i] &= bit_mask;
  }
}

void DShot::SetChannel(u8_t bit, u16_t value, bool telemetry) {
  if (bit > 8) return;
  const u8_t bit_set = 1 << bit;
  const u8_t bit_clr = ~bit_set;
  if ((dshot_mask_ & bit_set) == 0) {
    port_->OUTCLR = bit_set;  // Set bit low to start
    port_->DIR |= bit_set;  // Set bit as output
    dshot_mask_ |= bit_set;
  }
  
  u16_t encoded = (value & ((1 << 11) - 1)) << 5;
  if (telemetry) {
    encoded |= 0x10;
    // You should always wait until you get telemtry back from one channel
    // before requesting telemetry from another channel.
    DBG_LO_IF(DSHOT, (telemetry_byte_ != 10),
	      ("DSHOT: Telementry requested while one pending: %d",
	       telemetry_byte_));
    telemetry_byte_ = 255;
  }
  u8_t crc = ((encoded >> 12) ^
              (encoded >>  8) ^
              (encoded >>  4)) & 0x0F;
  encoded |= crc;

  u16_t mask = 0x8000;
  for (u8_t i = 0; i < 16; ++i) {
    if (encoded & mask) {  // Reversed, we want to 'clear' just zero bits.
      dshot_data_[i] = dshot_data_[i] & bit_clr;
    } else {
      dshot_data_[i] = dshot_data_[i] | bit_set;
    }
    mask = mask >> 1;
  }
}

#define nop() __asm__ __volatile__ ( "nop\n" )

void DShot::SendDShot600() {
  const u8_t mask = dshot_mask_;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (u8_t bit_cnt = 0; bit_cnt < 16; ++bit_cnt) {
      port_->OUTSET = mask;
      nop();
      nop();
      nop();

      u8_t bits = dshot_data_[bit_cnt];
    
      port_->OUTCLR = bits;
      nop();
      nop();
      nop();
      nop();
      nop();
    
      port_->OUTCLR = mask;
      nop();
    }
  }
}

