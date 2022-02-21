// Copyright 2021 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "Boot.h"
#include "Dbg.h"
#include "IntTypes.h"
#include "RtcTime.h"

#include "leds/Clr.h"
#include "leds/FPMath.h"
#include "leds/Pixel.h"
#include "leds/Rgb.h"

#include "Pins.h"
#include "Pwm.h"
#include "SBus.h"
#include "Serial.h"
#include "WS2812.h"

using led::bscale8;
using led::HSV;
using led::RGBW;
using led::RGB;
using led::sin8;

#if defined(__AVR_ATmega4808__)
#define LED_PIN (PIN_F0)
#endif

#if defined(__AVR_ATmega4809__)
#define LED_PIN (PIN_C2)
#endif

#define LIGHT_BRIGHT_CH (10)  // Adjust leading edge light brightness.
#define LIGHT_LEVEL_CH (11)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH (12)  // Sets Lights Mode (solid, spin, glow)
#define LIGHT_SUBMODE_CH (13)  // Sets mode sub mode (color range, etc)
#define LIGHT_THROTTLE_CH (14)  // Adjusts lights speed

#define IS_HOST (true)

#define WING_CNT (36)
#define SPON_CNT (16)
#define BACK_CNT (17)
#define MAIN_CNT (30)
#define TAIL_FRT (8)
#define TAIL_SDE (8)
#define TAIL_BCK (9)
#define TAIL_CNT (TAIL_FRT + TAIL_SDE + TAIL_SDE + TAIL_BCK)
#define RGBW_CNT (WING_CNT + SPON_CNT + TAIL_CNT)
#define RGB_CNT  (BACK_CNT + 2 * MAIN_CNT)

u8_t led_data[RGBW_CNT * 4 + RGB_CNT * 3];

enum CmdCode {
	      CMD_TYPE_MSK   = 0xF0,
	      CMD_CLIENT     = 0x11,
	      CMD_UPDATE     = 0x12,
	      CMD_STATE      = 0x20,
	      CMD_CTRL_STATE = CMD_STATE | 0x00,
	      CMD_CLR_STATE  = CMD_STATE | 0x01,
	      CMD_PLS_STATE  = CMD_STATE | 0x02,
};

bool IsStateCmd(u8_t cmd) {
  return (cmd & CMD_TYPE_MSK) == CMD_STATE;
}

static RGB eng_grad[5] = { RGB(0x00, 0x00, 0x00),
			   RGB(0x40, 0x00, 0x00),
			   RGB(0x80, 0x20, 0x00),
			   RGB(0xFF, 0x80, 0x00),
			   RGB(0xFF, 0xFF, 0x80) };

const u8_t log_map[9] = {0, 1, 2, 5, 11, 24, 52, 115, 255};

u8_t logify(u8_t val) {
  u8_t lo = val & 0x1F;
  u8_t hi = val >> 5;
  int ret = ((u16_t(log_map[hi]) << 5) +
             (i16_t(log_map[hi + 1]) - log_map[hi]) * lo);
  return (ret + (1<<4)) >> 5;
}

i16_t abs(i16_t a) { return a < 0 ? -a : a; }

template <class P>
class LedSpanT {
public:
  LedSpanT() : ptr_(NULL), len_(0), reverse_(false) { }
  void* SetSpan(void* leds, u8_t len, bool reverse = false) {
    ptr_ = reinterpret_cast<P*>(leds);
    len_ = len;
    reverse_ = reverse;
    return ptr_ + len_;
  }
  u8_t len() const { return len_; }
  void* next_ptr() const { return ptr_ + len_; }

  P& At(u8_t idx) {
    if (idx < 0) idx = 0;
    else if ( idx > len_ - 1) idx = len_ - 1;

    if (reverse_) {
      return ptr_[len_ - 1 - idx];
    }
    return ptr_[idx];
  }
  void Set(u8_t idx, const P& pix) {
	At(idx) = pix;
  }
  void Fill(const P& pix) {
    led::Fill(ptr_, len_, pix);
  }

  template <class O>
  void FillOp(const O& op) {
    u16_t step = (u16_t(255) << 8) / len_;
    u16_t frac = 0;
    for (u8_t i = 0; i < len_; ++i, frac += step) {
      P& pix = At(i);
      op(u8_t(frac >> 8), &pix);
    }
  }

  // Forward means N -> N+1, otherwise N+1 -> N (accounting for reverse_).
  void Rotate(bool forward = true) {
    if (forward == reverse_) {
      P p0 = ptr_[0];
      for (int i = 0; i < len_ - 1; ++i) {
	ptr_[i] = ptr_[i + 1];
      }
      ptr_[len_ - 1] = p0;
    } else {
      P pn = ptr_[len_ - 1];
      for (int i = len_ - 1; i > 0; --i) {
	ptr_[i] = ptr_[i - 1];
      }
      ptr_[0] = pn;
    }
  }

protected:
  P* ptr_;
  u8_t len_;
  bool reverse_;
};

// This supports changing the saw pattern speed while running without
// introducing big jumps.  It basicaly aligns the phase of the new saw
// with the previous saw.
// Speed is the counts/ms as a 8.8 Fixed Point Number.
// If you set it to 1 then it completes a cycle in 1 minute
class VariableSaw {
public:
  VariableSaw(u16_t speed = 0) : off_(0), spd_(speed), prev_time_(0) { }

  // Speed is the counts/ms as a 8.8 Fixed Point Number.
  // If you set it to 1 then it completes a cycle in 1 minute
  void SetSpeed(u16_t speed) {
    u8_t prev = Get(prev_time_);
    spd_ = speed;
    u8_t curr = Get(prev_time_);
    // Add the difference to off so that at prev_time_ the old speed and
    // new speed are aligned.
    off_ += prev - curr;
  }
  
  u8_t Get(u16_t now) {
    prev_time_ = now;
    return u8_t(((now * spd_) + (1 << 7)) >> 8) + off_;
  }

  u8_t GetOff() { return off_; }
  void SetState(u16_t speed, u8_t off) {
    spd_ = speed;
    off_ = off;
  }


private:  
  u8_t off_;
  u16_t spd_;
  u16_t prev_time_;
};

// This class uses a packet structure that starts with a key byte that
// signals the start of a packet by having it's high bit set, no other bytes
// will have their high bits set.  The structure of the key byte is:
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |  1  |          packet high bits               |
// bit 7 is always 1
// bits 0-6 are the high bits from the packet bytes (which are cleared).
// This means that you can resync by dropping bytes until you get a byte
// with it's high bit set and then you know the rest of the bytes are a
// packet.  A packet is always 8 bytes (including the signal byte).
class SyncableChannel {
public:
  SyncableChannel(Serial* com) : com_(com), len_(0) { }
  Serial* Channel() { return com_; }
  
  void WriteByte(u8_t b, bool flush = false) {
    packet_[len_++] = b;
    if (flush || len_ == 7) {
      SendPacket();
    }
  }
  void WriteBytes(const u8_t* ptr, int sz, bool flush = false) {
    while (sz) {
      u8_t space = 7 - len_;
      if (space > sz) space = sz;
      for (int i = 0; i < space; ++i) {
	packet_[len_++] = *(ptr++);
      }
      sz -= space;
      if (len_ == 7) {
	SendPacket();
      }
    }
    if (flush) {
      SendPacket();
    }
  }

  bool DoRead() {
    while (com_->Avail()) {
      ReadInfo info;
      com_->Read(&info);
      if (info.err) {
	len_ = 255;
	continue;
      }
      if (info.data & 0x80) {
	len_ = 0;
      }
      if (len_ == 255) continue;
      packet_[len_++] = info.data;
      if (len_ == 8) return true;
    }
    return false;
  }
  u8_t* GetPacket() {
    if (len_ != 8) return NULL;
    len_ = 255;
    u8_t key_packet = packet_[0];
    u8_t* ptr = &packet_[1];
    for (int i = 0; i < 7; ++i, ++ptr) {
      if (key_packet & (1 << i)) {
	*ptr |= 0x80;
      }
    }
    return packet_ + 1;
  }
  
protected:
  void SendPacket() {
    if (len_ == 0) return;
    memset(packet_ + len_, 0, 7 - len_);
    len_ = 7;
    u8_t key_byte = 0x80;
    // migrate high bits to key_byte, clear them from packet bytes.
    for (int i = 0; i < len_; ++i) {
      if (packet_[i] & 0x80) key_byte |= 1 << i;
      packet_[i] = packet_[i] & 0x7F;
    }
    com_->WriteByte(key_byte);
    for (int i = 0; i < len_; ++i) {
      com_->WriteByte(packet_[i]);
    }
    len_ = 0;
  }

  Serial* com_;
  u8_t len_;
  u8_t packet_[8];
};

// The PWM channels go back to front with 5 zones.
#define NUM_PWM (5)

struct CtrlState {
  enum  ChangeBits {
		    CHG_NONE = 0,
		    CHG_MODE = 1 << 0,
		    CHG_LVL  = 1 << 1,
		    CHG_BRT  = 1 << 2,
		    CHG_THR  = 1 << 1,
  };
  u8_t mode;
  u8_t level;
  i16_t brt;
  i16_t thr;

  // brt & thr are 0 -> 2047 nominal.
  CtrlState() : mode(0), level(0), brt(0), thr(0) { }
  CtrlState(  u8_t m, u8_t l, i16_t b, i16_t t)
    : mode(m), level(l), brt(b), thr(t) { }

  u8_t Set(SBus* sbus) {
    u8_t mode = ((SBus::ThreePosSwitch(sbus->GetChannel(LIGHT_MODE_CH)) << 2) |
		 SBus::ThreePosSwitch(sbus->GetChannel(LIGHT_SUBMODE_CH)));
    CtrlState new_state(mode,
			SBus::ThreePosSwitch(sbus->GetChannel(LIGHT_LEVEL_CH)),
			sbus->GetChannel(LIGHT_BRIGHT_CH),
			sbus->GetChannel(LIGHT_THROTTLE_CH)
			);
    return UpdateFromState(new_state);
  }
    
  u8_t UpdateFromState(const CtrlState& other) {
    u8_t change = 0;
    change |= (other.mode != mode) ? CHG_MODE : 0;
    change |= (other.level != level) ? CHG_LVL : 0;
    change |= abs(other.brt - brt) > 10 ? CHG_BRT : 0;
    change |= abs(other.thr - thr) > 10 ? CHG_THR : 0;
    if (change == CHG_NONE) return change;

    *this = other;
    DBG_MD(APP, ("State: L:%d M:%02x B:%d T:%d\n", level, mode, brt, thr));
    return change;
  }

  void Send(SyncableChannel* ch, u8_t max_bytes) {
    if (sizeof(*this) > max_bytes) return;
    u8_t* ptr = reinterpret_cast<u8_t*>(this);
    DBG_OFF(APP, ("Send: %d\n", sizeof(*this)));
    ch->WriteBytes(ptr, sizeof(*this), true);
  }

  bool Receive(u8_t* packet, u8_t max_bytes) {
    if (max_bytes < sizeof(*this)) return false;
    
    memcpy(this, packet, sizeof(*this));
    DBG_HI(APP, ("Rcv: L:%d M:%02x B:%d T:%d\n", level, mode, brt, thr));
    return true;
  }
};

struct SolidState {
  u8_t brt;
  u8_t spon_brt;
  bool flash;
  
  SolidState() : brt(0), spon_brt(0), flash(false) { }

  void UpdateState(bool is_host, u8_t brt, u8_t spon_brt) {
    this->brt = brt;
    this->spon_brt = spon_brt;    
  }
  void Update(u16_t now) {
    // Flash for the last 64ms of every second (1s = 1024 or 10bits
    // 64ms = (1 << 6) so 
    flash = ((u8_t(now >> 6) & 0x1F) == 0x1F);
  }
};

class ColorModeState {
public:
  void SetOpOffset(u8_t offset) {
    offset_ = offset;
  }

  void UpdateState(bool is_host, i16_t thr, u8_t brt) {
    brt_ = brt;
    // Throttle is 11 bits, spd has 5 fractional bits, so this makes the
    // speed multiplier go from 0 -> 4x with throttle.
    u8_t spd = (thr + (1 <<3)) >> 4;
    if (spd < 0x07) spd = 0x07;
    if (is_host) {
      color_saw_.SetSpeed(spd >> 1);
      pulse_saw_.SetSpeed(u16_t(spd) << 1);
    } else {
      color_saw_.SetState(spd >> 1, wire_.color_off);
      pulse_saw_.SetState(u16_t(spd) << 1, wire_.pulse_off);
    }
  }
  void Update(u16_t now) {
    u8_t color_hue = color_saw_.Get(now);
    color_ = HsvToRgb(HSV(color_hue, 0xFF, brt_));
    pulse_phase_ = 255 - pulse_saw_.Get(now);
  }
  const RGB& GetPix() { return color_; }

  void operator() (u8_t frac, RGB* pix) const {
    *pix = color_;
    Fade(pix, logify(sin8(offset_ + pulse_phase_ + frac)));
  }
  void operator() (u8_t frac, RGBW* pix) const {
    *pix = color_;
    Fade(pix, logify(sin8(offset_ + pulse_phase_ + frac)));
  }

  void Send(SyncableChannel* ch, u8_t max_bytes) {
    if (sizeof(WireData) > max_bytes) return;
    wire_.color_off = color_saw_.GetOff();
    wire_.pulse_off = pulse_saw_.GetOff();
    ch->WriteBytes(reinterpret_cast<u8_t*>(&wire_), sizeof(wire_), true);
  }
  bool Receive(u8_t* packet, u8_t max_bytes) {
    if (max_bytes < sizeof(wire_)) return false;
    memcpy(&wire_, packet, sizeof(wire_));
    DBG_HI(APP, ("Rcv: clr:%d pulse:%d\n", wire_.color_off, wire_.pulse_off));
    return true;
  }

protected:
  struct WireData {
    u8_t color_off;
    u8_t pulse_off;
  };
  WireData wire_;
  u8_t brt_;
  VariableSaw color_saw_;
  VariableSaw pulse_saw_;
  u8_t pulse_phase_;
  u8_t offset_;
  RGB color_;
};

#define MAX_WAVE (3)
// 0x00, 0x40, 0x80, 0xC0, 0xFF 
static const u8_t spd_map[] = {0x15, 0x50, 0x70, 0xB0, 0xFF};

class PulseModeState {
public:
  PulseModeState()
    : prev_pulse_pos_(0) {
    memset(temp_, 0, sizeof(temp_));
  }
  bool flash() { return flash_cnt_ > 0; };
  u8_t temp(u8_t idx) { return temp_[idx]; }
  void UpdateState(bool is_host, i16_t thr) {
    // Throttle is 11 bits, spd has 5 fractional bits, so this makes the
    // speed multiplier go from 0 -> 4x with throttle.
    u8_t thr8 = thr >> 3;
    u8_t spd_idx = thr8 >> 6;
    u8_t spd_frac = thr8 & 0x3F;
    u8_t spd = spd_map[spd_idx] +
      ((u16_t(spd_map[spd_idx + 1] - spd_map[spd_idx]) * spd_frac) >> 6);
    if (is_host) {
      glow_saw_.SetSpeed(spd);
      pulse_saw_.SetSpeed(spd);
    } else {
      glow_saw_.SetState(spd, wire_.glow_off);
      pulse_saw_.SetState(spd, wire_.pulse_off);
      for (u8_t i = 0; i < MAX_WAVE; ++i) {
	wave_off_[i] = wire_.wave_off[i];
      }
    }
  }

  void Update(u16_t now) {
    if (flash_cnt_ > 0) --flash_cnt_;
    u8_t pulse_pos = pulse_saw_.Get(now);
    u8_t glow = glow_saw_.Get(now);
    if (glow < prev_glow_) { // Fired, create wave
      for (u8_t i = 0; i < MAX_WAVE; ++i) {
	if (wave_off_[i] == 0) {
	  u8_t pulse_val = pulse_pos;
	  if (pulse_val == 0) {
	    wave_off_[i] = 1;
	  } else {
	    wave_off_[i] = 0 - pulse_val;
	  }
	  break;
	}
      }
    }
    prev_glow_ = glow;
    u8_t k[2] = { temp_[0], temp_[0] };
    // Cool and blend everything a bit.
    for (u8_t i = 0; i < sizeof(temp_); ++i) {
      u8_t t = temp_[i];
      temp_[i] = (u16_t(t) + k[1]  + k[0]) >> 2;
      k[0] = k[1];
      k[1] = t;
    }
    for (u8_t i = 0; i < 3; ++i) { // Add glow to the front
      if (temp_[i] < glow) temp_[i] = glow;
      glow = (glow * 15) >> 4;
    }

    for (u8_t i = 0; i < MAX_WAVE; ++i) {
      if (wave_off_[i] == 0) continue;
      u8_t prev_wave_pos = prev_pulse_pos_ + wave_off_[i];
      if (prev_pulse_pos_ < 0) prev_pulse_pos_ = 0;
      u8_t wave_pos = pulse_pos + wave_off_[i];
      if (wave_pos > (MAIN_CNT + BACK_CNT - 3)) {
	wave_pos = (MAIN_CNT + BACK_CNT - 3);
	wave_off_[i] = 0;
	flash_cnt_ = 4;
      }
      if (wave_pos < 0) wave_pos = 0;
      for (int j = prev_wave_pos; j <= wave_pos; ++j) {
	temp_[j] = 0xFF;
      }
      temp_[wave_pos + 1] = 0xC8;
      temp_[wave_pos + 2] = 0x80;
    }
    prev_pulse_pos_ = pulse_pos;
  }

  void Send(SyncableChannel* ch, u8_t max_bytes) {
    if (sizeof(WireData) > max_bytes) return;
    wire_.glow_off = glow_saw_.GetOff();
    wire_.pulse_off = pulse_saw_.GetOff();
    for (u8_t i = 0; i < MAX_WAVE; ++i) {
      wire_.wave_off[i] = wave_off_[i];
    }
    ch->WriteBytes(reinterpret_cast<u8_t*>(&wire_), sizeof(wire_), true);
  }
  bool Receive(u8_t* packet, u8_t max_bytes) {
    if (max_bytes < sizeof(wire_)) return false;
    memcpy(&wire_, packet, sizeof(wire_));
    DBG_HI(APP, ("Rcv: glow:%d pulse:%d\n", wire_.glow_off, wire_.pulse_off));
    return true;
  }

protected:
  
  struct WireData {
    u8_t glow_off;
    u8_t pulse_off;
    u8_t wave_off[MAX_WAVE];
  };
  WireData wire_;
  VariableSaw glow_saw_;
  VariableSaw pulse_saw_;
  u8_t prev_pulse_pos_;
  u8_t prev_glow_;
  u8_t wave_off_[MAX_WAVE];
  u8_t temp_[MAIN_CNT + BACK_CNT];
  u8_t flash_cnt_;
};

class Lights {
public:
  enum {
	UNSET = 0,
	HOST = 1,
	CLIENT = 2,
  };
  Lights(PinId led_pin, Pwm* pwm) :
    host_(UNSET), led_pin_(led_pin), pwm_(pwm),
    mode_(0), brt_(0), spon_brt_(0) {
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      pwm->Enable(i);
      pwm_val_[i] = 0;
    }
  }
  u8_t Mode() { return mode_ >> 2; }
  u8_t Submode() { return mode_ & 0x03; }
  
  bool IsHostSet() { return host_ !=  UNSET; }
  void SetIsHost(bool is_host) {
    host_ = is_host ? HOST : CLIENT;
    // Order of leds in the string are slightly different for host/client
    // sides of plane.  In particular fuselage font/back are swapped.
    void* ptr = led_data;
    ptr = wing_.SetSpan(ptr, WING_CNT, false);
    ptr = spon_.SetSpan(ptr, SPON_CNT, false);
    if (host_ == HOST) {
      ptr = back_.SetSpan(ptr, BACK_CNT, true);
      ptr = f_osd_.SetSpan(ptr, MAIN_CNT, true);
      ptr = f_isd_.SetSpan(ptr, MAIN_CNT, false);
      ptr = tail_frt_.SetSpan(ptr, TAIL_FRT, false);
      ptr = tail_isd_.SetSpan(ptr, TAIL_SDE, false);
      ptr = tail_osd_.SetSpan(ptr, TAIL_SDE, false);
      ptr = tail_bck_.SetSpan(ptr, TAIL_BCK, false);
    } else {
      ptr = f_isd_.SetSpan(ptr, MAIN_CNT, true);
      ptr = f_osd_.SetSpan(ptr, MAIN_CNT, false);
      ptr = back_.SetSpan(ptr, BACK_CNT, false);
      ptr = tail_frt_.SetSpan(ptr, TAIL_FRT, false);
      ptr = tail_osd_.SetSpan(ptr, TAIL_SDE, false);
      ptr = tail_isd_.SetSpan(ptr, TAIL_SDE, false);
      ptr = tail_bck_.SetSpan(ptr, TAIL_BCK, false);
    }
  }
  void PushLeds() {
    SendWS2812(led_pin_, led_data, sizeof(led_data), 0xFF);
  }
  void PushPwm() {
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      pwm_->Set(i, pwm_val_[4 - i]); // PWM are back (idx =0) to front (idx=4)
    }
  }
  void Push() {
    PushLeds();
    PushPwm();
  }

  RGB GetNavColor() {
    switch (host_) {
    default: break;
    case HOST: return RGB(0, spon_brt_, 0); // Green right/starboard side
    case CLIENT: return RGB(spon_brt_, 0, 0); // Red left/port side
    }
    return RGB(0, 0, 0);
  }
  
  // Solid mode is body on, wing edges white, sponsons R/G.
  void UpdateSolidMode() {
    // Submodes are default = 0, simple = 1, flashy = 2
    u8_t smode = Submode();
    if (smode != 1 && solid_state_.flash) {
      spon_.Fill(RGBW(spon_brt_));
      tail_osd_.Fill(RGBW(spon_brt_));
      f_osd_.Fill(RGB(spon_brt_));
    } else {
      RGB nav = GetNavColor();
      spon_.Fill(nav);
      f_osd_.Fill(nav);
      tail_osd_.Fill(nav);
    }

    wing_. Fill(RGBW(brt_));
    if (smode == 2) {
      f_isd_. Fill(RGB(0, 0, brt_));
      back_. Fill(RGB(brt_, 0, brt_));
    } else {
      f_isd_. Fill(RGB(brt_));
      back_. Fill(GetNavColor());
    }
    RGBW white(spon_brt_);
    tail_frt_.Fill(white);
    tail_isd_.Fill(white);
    tail_bck_.Fill(white);

    u8_t val = brt_;
    switch (smode) {
    case 0: val = brt_; break;
    case 1: val = 0; break;
    case 2: val = solid_state_.flash ? spon_brt_ : brt_; break;
    }
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      pwm_val_[i] = val;
    }
  }

  // Color mode
  void UpdateColorMode() {
    RGB nav = GetNavColor();
    spon_.Fill(nav);
    color_mode_state_.SetOpOffset(150);
    wing_. FillOp(color_mode_state_);
    color_mode_state_.SetOpOffset(0);
    f_isd_. FillOp(color_mode_state_);
    f_osd_.FillOp(color_mode_state_);
    back_. Fill(nav);
    tail_osd_.Fill(nav);
    RGB pix = color_mode_state_.GetPix();
    tail_frt_.Fill(pix);
    tail_bck_.Fill(pix);
    tail_isd_.Fill(pix);
    
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      pwm_val_[i] = brt_;
    }
  }
  
  // Pulse mode
  void UpdatePulseMode() {
    RGB nav = GetNavColor();
    spon_.Fill(nav);
    bool flash = pulse_mode_state_.flash();
    u8_t brt = flash ? spon_brt_ : brt_;
    for (u8_t i = 0; i < NUM_PWM; ++i) {
      pwm_val_[i] = brt;
    }
    RGBW bw(brt);
    wing_.Fill(bw);
    tail_frt_.Fill(bw);
    tail_bck_.Fill(bw);
    tail_isd_.Fill(bw);
    tail_osd_.Fill(bw);

    u8_t pwm_idx = 0;
    u16_t pwm_sum = 0;
    i8_t pwm_cnt = 0;
    for (u8_t i = 0; i < MAIN_CNT; ++i) {
      u8_t t = pulse_mode_state_.temp(i);
      RGB pix = Lookup5(eng_grad, t);
      Fade(&pix, spon_brt_);
      f_isd_.At(i) = pix;
      f_osd_.At(i) = pix;
      if (!flash) {
	pwm_sum += u16_t(13) * t;
	if (++pwm_cnt == 10) {
	  pwm_sum  = pwm_sum >> 7;
	  u8_t v = (pwm_sum > 255) ? 255 : pwm_sum;
	  pwm_val_[pwm_idx++] = bscale8(v, spon_brt_);
	  pwm_sum = 0;
	  pwm_cnt = 0;
	}
      }
    }
    pwm_sum = 0;
    pwm_cnt = 0;
    for (u8_t i = 0; i < BACK_CNT; ++i) {
      u8_t t = pulse_mode_state_.temp(MAIN_CNT + i);
      RGB pix = Lookup5(eng_grad, t);
      Fade(&pix, spon_brt_);
      back_.At(i) = pix;
      if (!flash) {
	pwm_sum += t;
	if (++pwm_cnt == 8) {
	  u8_t v = pwm_sum >> 3;
	  pwm_val_[pwm_idx++] = bscale8(v, spon_brt_);
	  pwm_sum = 0;
	  pwm_cnt = 1;
	}
      }
    }
    DBG_LO(APP, ("PulseU: flash: %c pwm_idx: %d "
		 "pwm[%02x, %02x, %02x, %02x, %02x]\n",
		 (flash ? 'T' : 'F'), pwm_idx, pwm_val_[0], pwm_val_[1],
		 pwm_val_[2], pwm_val_[3], pwm_val_[4]));
  }

  // lvl is overall brightness mode (off, med, hi)
  // sbrt is 11 bit brightness slider affects 'add-ins'
  // not critical lights light wing tip/sponsons.
  void UpdateBright(u8_t lvl, u16_t sbrt) {
    brt_ = 0;
    if (sbrt > 32) {
      sbrt = sbrt >> 3;
      brt_ = (sbrt > 255) ? 255 : sbrt;
    }

    switch (lvl) {
    case 0: brt_ = spon_brt_ = 0; break;
    case 1: spon_brt_ = 0x40; brt_ = brt_ >> 3; break;
    case 2: spon_brt_ = 0xFF; break;
    }
    DBG_HI(APP, ("Update Brt: %d Spn: %d\n", brt_, spon_brt_));
  }

  // Returns true if the state_ was changed.
  bool SBusUpdate(SBus* sbus) {
    state_change_ |= state_.Set(sbus);
    return (state_change_ != CtrlState::CHG_NONE);
  }

  void SendState(SyncableChannel* ch) {
    ch->WriteByte(CMD_CTRL_STATE);
    state_.Send(ch, 6); 

    ch->WriteByte(CMD_CLR_STATE);
    color_mode_state_.Send(ch, 6);

    ch->WriteByte(CMD_PLS_STATE);
    pulse_mode_state_.Send(ch, 6);
  }

  void Receive(u8_t* packet) {
    switch (packet[0]) {
    case CMD_CTRL_STATE: {
      CtrlState new_state;
      new_state.Receive(packet + 1, 6);
      state_change_ = state_.UpdateFromState(new_state);
    } break;
    case CMD_CLR_STATE:
      color_mode_state_.Receive(packet + 1, 6);
      break;
    case CMD_PLS_STATE:
      pulse_mode_state_.Receive(packet + 1, 6);
      break;
    }
  }

  void ApplyUpdatedState() {
    // bool mode_change = (state_change_ & CHG_MODE) != CHG_NONE;
    mode_ = state_.mode;
    state_change_ = 0;
    UpdateBright(state_.level, state_.brt);
    bool is_host = (host_ == HOST);
    solid_state_.UpdateState(is_host, brt_, spon_brt_);
    color_mode_state_.UpdateState(is_host, state_.thr, spon_brt_);
    pulse_mode_state_.UpdateState(is_host, state_.thr);
  }

  void Update(u16_t now) {
    DBG_HI(APP, ("Lights::Update now: %u\n", now));
    // We want cycle pos to complete a cycle every second (8bits)
    // 1s = 1024'ms' = 10bits, so shift 2 bits for that, and 5
    // bits for spd for a total of 7.
    // u8_t cycle_pos = ((now * spd_) + (1 << 6)) >> 7;
    // cycle_pos += shared_state_->cycle_pos_offset();
    switch (Mode()) {
    case 0: solid_state_.Update(now); UpdateSolidMode(); break;
    case 1:
      color_mode_state_.Update(now);
      UpdateColorMode();
      break;
    case 2:
      pulse_mode_state_.Update(now);
      UpdatePulseMode();
      break;
    }
    Push();
  }

  void Rotate() {
    wing_. Rotate();
    spon_. Rotate();
    back_. Rotate();
    f_isd_. Rotate();
    f_osd_.Rotate();
    tail_frt_. Rotate();
    tail_isd_. Rotate();
    tail_osd_. Rotate();
    tail_bck_. Rotate();
  }
  
  u8_t host_;
  PinId led_pin_;
  Pwm* const pwm_;
  u8_t state_change_;
  u8_t pwm_val_[5];
  u8_t mode_, brt_, spon_brt_;
  CtrlState state_;
  SolidState solid_state_;
  ColorModeState color_mode_state_;
  PulseModeState pulse_mode_state_;
  LedSpanT<RGBW> wing_;
  LedSpanT<RGBW> spon_;
  LedSpanT<RGB>  back_;
  LedSpanT<RGB>  f_isd_;
  LedSpanT<RGB>  f_osd_;
  LedSpanT<RGBW> tail_frt_;
  LedSpanT<RGBW> tail_isd_;
  LedSpanT<RGBW> tail_osd_;
  LedSpanT<RGBW> tail_bck_;
};

class StateManager {
public:
  StateManager(Serial* com, Lights* lights) :
    ch_(com),
    lights_(lights),
    now_(0), prev_update_(0), update_cnt_(0),
    run_(false),
    host_(false),
    state_pkts_(0)
  {
    ch_.Channel()->Setup
      (115200, 8, Serial::PARITY_EVEN, 1, /*invert=*/false,
       /*use_alt_pins=*/false, Serial::MODE_RX,
       /*use_pullup=*/true, /*buffered=*/true);
  }

  bool IsRun() const { return run_; }
  bool IsHost() const { return host_; }
  bool IsClient() const { return !host_; }

  u32_t GetNow() const { return now_; }

  u8_t GetBlinkPattern() {
    const u8_t pat_host = 0xCC; // Fast blink
    const u8_t pat_client = 0x0C; // Slow/Fast
    const u8_t pat_search = 0x0F; // Slow blink
    if (!run_) return pat_search;
    return host_ ? pat_host : pat_client;
  }

  void ClaimClient() {
    DBG_LO(APP, ("Claim Client!\n"));
    run_ = true;
    host_ = false;
    lights_->SetIsHost(false);
  }

  void ClaimHost() {
    DBG_LO(APP, ("ClaimHost!\n"));
    ch_.Channel()->Setup
      (115200, 8, Serial::PARITY_EVEN, 1, /*invert=*/false,
       /*use_alt_pins=*/false, Serial::MODE_TX,
       /*use_pullup=*/true, /*buffered=*/true);
    run_ = true;
    host_ = true;
    lights_->SetIsHost(true);
  }

  void DoUpdate() {
    update_cnt_++;
    if (!run_) return;
    lights_->Update(now_);
  }

  // Called roughly every 32ms (32 fps)
  void HostUpdate(SBus* sbus, bool force_state_update = false) {
    if (!run_ || !host_) return; // Not known host so don't trigger update.

    DBG_HI(APP, ("Host Update\n"));
    now_ = FastTimeMs();
    u8_t now_8 = now_ >> 8;  // 1/4 seconds
    // Update state if forced,  sbus settings change or 1/4 sec has elapsed.
    if (force_state_update || now_8 != prev_update_ ||
	lights_->SBusUpdate(sbus) ) {
      prev_update_ = now_8;
      lights_->ApplyUpdatedState();
      lights_->SendState(&ch_);
    }
    ch_.WriteByte(CMD_UPDATE);
    ch_.WriteByte(update_cnt_);
    ch_.WriteBytes(reinterpret_cast<const u8_t*>(&now_),
		   sizeof(now_), /*flush=*/true);
    DoUpdate();
  }

  void ProcessPacket(u8_t* packet) {
    if (packet == NULL) return;
    DBG_HI(APP, ("Packet: %02x spkts: %d\n", packet[0], state_pkts_));
    switch (packet[0]) {
    case CMD_UPDATE:
      {
	memcpy(&now_, packet + 2, sizeof(now_));
	const u8_t host_cnt = packet[1];
	if (update_cnt_ - host_cnt > 4) {
	  // If this delta is too large then something unexpected happened
	  // and running extra update 'to catch up' is probably not going to
	  // help and could help, so just do one.
	  update_cnt_ = host_cnt - 1;
	}
	
	if (state_pkts_ == 3) {  // Process state update now.
	  if (!run_) {
	    ClaimClient();
	    update_cnt_ = host_cnt - 1;
	  }
	  lights_->Receive(ctrl_packet_);
	  lights_->Receive(clr_packet_);
	  lights_->Receive(pls_packet_);
	  lights_->ApplyUpdatedState();
	}
	while (update_cnt_ != host_cnt) {
	  DoUpdate();
	}
      }
      break;
      
    case CMD_CTRL_STATE:
      memcpy(ctrl_packet_, packet, 7);
      state_pkts_ = 1;
      break;
    case CMD_CLR_STATE:
      if (state_pkts_ == 1) {
	memcpy(clr_packet_, packet, 7);
	++state_pkts_;
      } else {
	state_pkts_ = 0;
      }
      break;
    case CMD_PLS_STATE:
      if (state_pkts_ == 2) {
	memcpy(pls_packet_, packet, 7);
	++state_pkts_;
      } else {
	state_pkts_ = 0;
      }	  
      break;
    }
  }
  
  void CheckChannel() {
    if (run_ && host_) return;  // Not reading from host.
    while (ch_.DoRead()) {
      u8_t* packet = ch_.GetPacket();
      if (packet == NULL) continue;
      ProcessPacket(packet);
    }
  }

  void CheckSBus(SBus* sbus) {
    if (run_ && !host_) return;  // Not reading from SBus
    if (!sbus->Run()) return;  // No data from SBus
    
    if (!run_) {
      ClaimHost(); // We got SBus packet, thus we are host
      HostUpdate(sbus, /*force_state_update=*/true);
    }
  }
  
  void Run(SBus* sbus) {
    // Check channel, will do nothing if known host.
    CheckChannel();
    // Check SBus, will do nothing if known client.
    CheckSBus(sbus);
  }

protected:
  SyncableChannel ch_;
  Lights* lights_;
  u16_t now_;
  u8_t prev_update_;
  u8_t update_cnt_;
  bool run_;
  bool host_;
  u8_t state_pkts_;
  u8_t ctrl_packet_[7];
  u8_t clr_packet_[7];
  u8_t pls_packet_[7];
};


int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  Pwm pwm(PORT_D, 400);
  for (u8_t i = 0; i < 5; ++i) {
    pwm.Enable(i);
    pwm.Set(i, 0);
  }

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  DBG_LEVEL_HI(SBUS);
  SBus sbus(&Serial::usart3, /*invert=*/true);

  PinId led_pin(LED_PIN);
  led_pin.SetOutput();
  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();
 
  Lights lights(LED_PIN, &pwm);
  StateManager state_mgr(&Serial::usart1, &lights);
  
  sei();
  DBG_MD(APP, ("Hello World: Test\n"));

  memset(led_data, 0, sizeof(led_data));
  SendWS2812(LED_PIN, led_data, sizeof(led_data), 0xFF);

  u8_t update_5 = 0;
  DBG_MD(APP, ("Entering Run Loop\n"));
  u8_t update_8 = 0;
  while (1) {
    u16_t now = FastTimeMs();
    const u8_t now_8 = now >> 8;

    state_mgr.Run(&sbus);

    const u8_t now_5 = now >> 5;
    if (now_5 == update_5) continue;
    update_5 = now_5;
    state_mgr.HostUpdate(&sbus);  // Won't do anything if not known host
	
    if (now_8 == update_8) continue;
    update_8 = now_8;
    u8_t phase = (now_8 & 0x07);
    u8_t pat = state_mgr.GetBlinkPattern();
    blink_pin.set(pat & (1 << phase));
    // if (state_mgr.IsHost()) sbus.Dump();
  }
}
