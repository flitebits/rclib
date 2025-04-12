// Copyright 2021, 2022 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <ctype.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "Boot.h"
#include "DbgCmds.h"
#include "VarCmds.h"
#include "IntTypes.h"
#include "RtcTime.h"

#include "leds/Clr.h"
#include "leds/FPMath.h"
#include "leds/LedSpan.h"
#include "leds/Pixel.h"
#include "leds/Rgb.h"

#include "Pca9685.h"
#include "Pins.h"
#include "Pwm.h"
#include "SBus.h"
#include "Serial.h"
#include "Twi.h"
#include "WS2812.h"

using led::bscale8;
using led::Logify;
using led::HSV;
using led::RGBW;
using led::RGB;
using led::sin8;
using led::VariableSaw;
using dbg::DbgCmds;
using dbg::CmdHandler;

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

#define FUSE_CNT (34)
#define IWING_CNT (12)
#define OWING_CNT (38) // Starboard (39)
#define SPON_CNT (17)
#define STAB_CNT (7)
#define NCEL_CNT (7)
#define TAIL_CNT (STAB_CNT + 2 * NCEL_CNT)

#define NUM_PWM (15)

#define RGBW_CNT (IWING_CNT + OWING_CNT + 1 + SPON_CNT + TAIL_CNT)
#define RGB_CNT  (2* FUSE_CNT)

u8_t led_data[RGBW_CNT * 4 + RGB_CNT * 3];

const u8_t lfuse_left_idx[6]  = {10, 11, 0, 1, 2, 3};
const u8_t lfuse_right_idx[6] = {12, 13, 4, 5, 6, 7};
const u8_t lwing_idx[3]       = {8, 15, 9};

const u8_t rfuse_left_idx[6]  = {15, 14, 4, 5, 6, 7};
const u8_t rfuse_right_idx[6] = {13, 12, 0, 1, 2, 3};
const u8_t rwing_idx[3]       = {8, 8, 8};

const u8_t* fuse_left_idx;
const u8_t* fuse_right_idx;
const u8_t* wing_idx;

void SetPwmIdxs(bool left_hull) {
  if (left_hull) {
    fuse_left_idx  = lfuse_left_idx;
    fuse_right_idx = lfuse_right_idx;
    wing_idx       = lwing_idx;
  } else {
    fuse_left_idx  = rfuse_left_idx;
    fuse_right_idx = rfuse_right_idx;
    wing_idx       = rwing_idx;
  }
}

enum CmdCode {
              CMD_TYPE_MSK   = 0xF0,
              CMD_CLIENT     = 0x11,
              CMD_UPDATE     = 0x12,
              CMD_STATE      = 0x20,
              CMD_CTRL_STATE = CMD_STATE | 0x00,
              CMD_SLD_STATE  = CMD_STATE | 0x01,
              CMD_CLR_STATE  = CMD_STATE | 0x02,
              CMD_PLS_STATE  = CMD_STATE | 0x03,
};

bool IsStateCmd(u8_t cmd) {
  return (cmd & CMD_TYPE_MSK) == CMD_STATE;
}

static RGB eng_grad[5] = { RGB(0x00, 0x00, 0x00),
                           RGB(0x40, 0x00, 0x00),
                           RGB(0x80, 0x20, 0x00),
                           RGB(0xFF, 0x80, 0x00),
                           RGB(0xFF, 0xFF, 0x80) };

// This class uses a packet structure that starts with a key byte that
// signals the start of a packet by having it's high bit set, no other bytes
// will have their high bits set.  The structure of the key byte is:
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |  1  |    high bit from each byte in packet    |
// bit 7 is always 1
// bits 0-6 are the high bits from the packet bytes (which are cleared).
// Rest of the bytes in the packet are
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |  0  |          packet high bits               |

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
  CtrlState(u8_t m, u8_t l, i16_t b, i16_t t)
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
    u8_t change = CHG_NONE;
    change |= (other.mode != mode) ? CHG_MODE : 0;
    change |= (other.level != level) ? CHG_LVL : 0;
    change |= abs(other.brt - brt) > 10 ? CHG_BRT : 0;
    change |= abs(other.thr - thr) > 10 ? CHG_THR : 0;
    if (change == CHG_NONE) return change;

    *this = other;
    PrintState("Update");
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

  void PrintState(const char* prefix) {
    DBG_MD(APP, ("State %s: L:%d M:%02x B:%d T:%d\n",
                 prefix, level, mode, brt, thr));
  }
};

struct SolidState {
  SolidState() : flash_(false) { }

  void UpdateState(bool is_host, i16_t thr, u8_t brt, u8_t spon_brt) {
    u8_t spd = (thr + (1 <<3)) >> 4;
    if (spd < 0x07) spd = 0x07;
    u8_t base_val = spon_brt >> 4;
    u8_t clr_val = spon_brt - base_val;
    if (is_host) {
      base_color_ = RGB(base_val, 0, 0);
      color_ = RGB(clr_val, 0, 0);
      pulse_saw_.SetSpeed(u16_t(spd) << 1);
    } else {
      base_color_ = RGB(0, base_val, 0);
      color_ = RGB(0, clr_val, 0);
      pulse_saw_.SetState(u16_t(spd) << 1, wire_.pulse_off);
    }
  }

  void Update(u16_t now) {
    // Flash for the last 64ms of every second (1s = 1024 or 10bits
    // 64ms = (1 << 6) so
    flash_ = ((u8_t(now >> 6) & 0x1F) == 0x1F);
    pulse_phase_ = 255 - pulse_saw_.Get(now);
  }

  const RGB& GetPix() { return color_; }
  bool flash() { return flash_; }

  void SetOpOffset(u8_t offset) {
    offset_ = offset;
  }
  // Scale is 2.6 FP number
  void SetOpScale(u8_t scale) {
    scale_ = scale;
  }

  void operator() (u8_t frac, RGB* pix) const {
    frac = (u16_t(frac) * scale_) >> 6;
    *pix = color_;
    Fade(pix, Logify(sin8(offset_ + pulse_phase_ + frac)));
    *pix += base_color_;
  }
  void operator() (u8_t frac, RGBW* pix) const {
    frac = (u16_t(frac) * scale_) >> 6;
    *pix = color_;
    Fade(pix, Logify(sin8(offset_ + pulse_phase_ + frac)));
    *pix += base_color_;
  }

  void Send(SyncableChannel* ch, u8_t max_bytes) {
    if (sizeof(WireData) > max_bytes) return;
    wire_.pulse_off = pulse_saw_.GetOff();
    ch->WriteBytes(reinterpret_cast<u8_t*>(&wire_), sizeof(wire_), true);
  }
  bool Receive(u8_t* packet, u8_t max_bytes) {
    if (max_bytes < sizeof(wire_)) return false;
    memcpy(&wire_, packet, sizeof(wire_));
    DBG_HI(APP, ("Rcv Solid: pulse:%d\n", wire_.pulse_off));
    return true;
  }

protected:
  struct WireData {
    u8_t pulse_off;
  };
  WireData wire_;
  RGB base_color_;
  RGB color_;
  bool flash_;
  VariableSaw pulse_saw_;
  u8_t pulse_phase_;
  u8_t offset_;
  u8_t scale_;
};

class ColorModeState {
public:
  ColorModeState()
    : brt_(0), pulse_phase_(0), scale_(0), offset_(0) { }

  void UpdateState(bool is_host, u8_t submode, i16_t thr, u8_t brt) {
    submode_ = submode;
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
    if (submode_ != 2) {
      // Limit hue to 0 -> 16 (red -> orange)
      // so limit to 16 and subtract 4 (it's unsigned so the negative
      // values wrap to high values which is fine).
      if (color_hue >= 128) {
        // If it's greater than 128 the reflect it back towards zero.
        color_hue = 255 - color_hue;
      }
      // Now limited to 0->128, so divide by 8 so it's 0-16.
      color_hue = (color_hue >> 3);
    }
    color_ = HsvToRgb(HSV(color_hue, 0xFF, brt_));
    pulse_phase_ = 255 - pulse_saw_.Get(now);
  }
  const RGB& GetPix() { return color_; }

  void SetOpOffset(u8_t offset) {
    offset_ = offset;
  }
  // Scale is 2.6 FP number
  void SetOpScale(u8_t scale) {
    scale_ = scale;
  }
  void operator() (u8_t frac, RGB* pix) const {
    frac = (u16_t(frac) * scale_) >> 6;
    *pix = color_;
    Fade(pix, Logify(sin8(offset_ + pulse_phase_ + frac)));
  }
  void operator() (u8_t frac, RGBW* pix) const {
    frac = (u16_t(frac) * scale_) >> 6;
    *pix = color_;
    Fade(pix, Logify(sin8(offset_ + pulse_phase_ + frac)));
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
    DBG_HI(APP, ("Rcv Color: clr:%d pulse:%d\n", wire_.color_off, wire_.pulse_off));
    return true;
  }

protected:
  struct WireData {
    u8_t color_off;
    u8_t pulse_off;
  };
  WireData wire_;
  u8_t submode_;
  u8_t brt_;
  VariableSaw color_saw_;
  VariableSaw pulse_saw_;
  u8_t pulse_phase_;
  u8_t scale_;
  u8_t offset_;
  RGB color_;
};

#define MAX_WAVE (3)
// 0x00, 0x40, 0x80, 0xC0, 0xFF
static const u8_t spd_map[] = {0x20, 0x50, 0x70, 0xA0, 0xFF};

class PulseModeState {
public:
  PulseModeState()
    : prev_pulse_pos_(0), prev_glow_(0), glow_off_(0) {
    memset(temp_, 0, sizeof(temp_));
  }
  bool flash() { return flash_cnt_ > 0; };
  u8_t temp(u8_t idx) { return temp_[idx]; }
  void UpdateState(bool is_host, u8_t submode, i16_t thr) {
    // Throttle is 11 bits, spd has 5 fractional bits, so this makes the
    // speed multiplier go from 0 -> 4x with throttle.
    u8_t thr8 = thr >> 3;
    u8_t spd_idx = thr8 >> 6;
    u8_t spd_frac = thr8 & 0x3F;
    u8_t spd = spd_map[spd_idx] +
      ((u16_t(spd_map[spd_idx + 1] - spd_map[spd_idx]) * spd_frac) >> 6);
    glow_off_ = 0;
    if (is_host) {
      glow_saw_.SetSpeed(spd);
      pulse_saw_.SetSpeed(spd >> 1);
    } else {
      glow_saw_.SetState(spd, wire_.glow_off);
      pulse_saw_.SetState(spd >> 1, wire_.pulse_off);
      if (submode != 1) {
        glow_off_ = 128;
      }
    }
  }

  void Update(u16_t now) {
    if (flash_cnt_ > 0) {
       --flash_cnt_;
       int v = temp_[FUSE_CNT] + 0xC0;
       temp_[FUSE_CNT] = v > 255 ? 255 : v;
    }
    u8_t pulse_pos = pulse_saw_.Get(now);
    u8_t glow = glow_saw_.Get(now) + glow_off_;
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
    u8_t k[2] = { 0, u8_t(glow >> 1) };
    // Cool and blend everything a bit.
    for (u8_t i = 0; i < sizeof(temp_); ++i) {
      u8_t t = temp_[i];
      temp_[i] = (u16_t(t) + k[1]  + k[0]) >> 2;
      k[0] = k[1];
      k[1] = t;
    }
    for (u8_t i = 0; i < 3; ++i) { // Add glow to the front
      if (temp_[i] < glow) temp_[i] = glow;
      glow = (glow * 7) >> 3;
    }

    for (u8_t i = 0; i < MAX_WAVE; ++i) {
      if (wave_off_[i] == 0) continue;
      u8_t prev_wave_pos = prev_pulse_pos_ + wave_off_[i];
      if (prev_pulse_pos_ < 0) prev_pulse_pos_ = 0;
      u8_t wave_pos = pulse_pos + wave_off_[i];
      if (wave_pos > (FUSE_CNT - 3)) {
        wave_pos = (FUSE_CNT - 3);
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
  };
  WireData wire_;
  VariableSaw glow_saw_;
  VariableSaw pulse_saw_;
  u8_t prev_pulse_pos_;
  u8_t prev_glow_;
  u8_t glow_off_;
  u8_t wave_off_[MAX_WAVE];
  u8_t temp_[FUSE_CNT + 1];
  u8_t tail_temp_;
  u8_t flash_cnt_;
};

class Lights {
public:
  enum {
        UNSET = 0,
        HOST = 1,
        CLIENT = 2,
  };
  Lights(PinId led_pin, Pca9685* pwm) :
    host_(UNSET), led_pin_(led_pin), pwm_(pwm),
    mode_(0), brt_(0), spon_brt_(0) {
  }

  const CtrlState& State() const { return state_; }
  void ApplyState(const CtrlState& new_state) {
    state_.UpdateFromState(new_state);
    ApplyUpdatedState();
  }

  u8_t Mode() { return mode_ >> 2; }
  u8_t Submode() { return mode_ & 0x03; }

  bool IsHostSet() { return host_ !=  UNSET; }
  void SetIsHost(bool is_host) {
    // Host is always the left hull (it has the receiver).
    host_ = is_host ? HOST : CLIENT;
    SetPwmIdxs(/*left_hull=*/is_host);
    // Order of leds in the string are slightly different for host/client
    // sides of plane.  In particular fuselage font/back are swapped.
    void* ptr = led_data;
    // Host is the left(port) hull, led spans are largely the same, but the left
    // outer wing has 1 less led (grr).
    u8_t owing_cnt = OWING_CNT + (is_host ? 0 : 1);
    ptr = iwing_.SetSpan(ptr, IWING_CNT, false);
    if (is_host) {
      ptr = ifuse_.SetSpan(ptr, FUSE_CNT, true);
      ptr = ofuse_.SetSpan(ptr, FUSE_CNT, false);
    } else {
      ptr = ofuse_.SetSpan(ptr, FUSE_CNT, true);
      ptr = ifuse_.SetSpan(ptr, FUSE_CNT, false);
    }
    ptr = owing_.SetSpan(ptr, owing_cnt, false);
    ptr = spon_.SetSpan(ptr, SPON_CNT, false);
    ptr = stab_.SetSpan(ptr, STAB_CNT, false);
    if (is_host) {
      ptr = inac_.SetSpan(ptr, NCEL_CNT, true);
      ptr = onac_.SetSpan(ptr, NCEL_CNT, false);
    } else {
      ptr = onac_.SetSpan(ptr, NCEL_CNT, true);
      ptr = inac_.SetSpan(ptr, NCEL_CNT, false);
    }
  }

  void PushLeds() {
    SendWS2812(led_pin_, led_data, sizeof(led_data), 0xFF);
  }

  void PushPwm() {
    pwm_->Write();
  }
  void Push() {
    PushLeds();
    PushPwm();
  }

  RGB GetNavColor(u8_t brt) {
    switch (host_) {
    default: break;
    case HOST: return RGB(brt, 0, 0); // Red left/port side
    case CLIENT: return RGB(0, brt, 0); // Green right/starboard side
    }
    return RGB(0, 0, 0);
  }

  // Solid mode is body on, wing edges white, sponsons R/G.
  void UpdateSolidMode() {
    // Submodes are 0 - selective lights are nav
    // 1 - all lights are static nav
    // 2 - all lights are rolling nav
    u8_t smode = Submode();
    RGB nav = GetNavColor(spon_brt_);
    spon_.Fill(nav);
    if (smode == 0) {
      stab_.Fill(nav);
      ofuse_.Fill(nav);
      ifuse_.Fill(nav);
      onac_.Fill(nav);

      RGBW swht(spon_brt_);
      inac_.Fill(swht);
      RGBW bwht(brt_);
      owing_. Fill(bwht);
      iwing_. Fill(bwht);
    } else if (smode == 1) {
      RGB nav = GetNavColor(spon_brt_);
      stab_.Fill(nav);
      ofuse_.Fill(nav);
      onac_.Fill(nav);
      owing_. Fill(nav);
      iwing_. Fill(nav);
      inac_.Fill(nav);
      ifuse_.Fill(nav);
    } else {
      RGBW bwht(brt_);
      RGBW swht(spon_brt_);
      solid_state_.SetOpOffset(0);
      solid_state_.SetOpScale(3 << 5);
      ofuse_.FillOp(solid_state_);
      ifuse_.FillOp(solid_state_);
      solid_state_.SetOpOffset(128);
      owing_.FillOp(solid_state_);
      solid_state_.SetOpScale(1 << 5);
      iwing_.FillOp(solid_state_);
      solid_state_.SetOpOffset(160);
      solid_state_.SetOpScale(1 << 4);
      stab_.FillOp(solid_state_);
      solid_state_.SetOpOffset(180);
      inac_.FillOp(solid_state_);
      onac_.FillOp(solid_state_);
    }

    u8_t val = brt_;
    u16_t pwm_val = Pca9685::Apparent2Pwm(val);
    for (u8_t i = 0; i < 6; ++i) {
      pwm_->SetLed(pwm_val, fuse_right_idx[i]);
      pwm_->SetLed(pwm_val, fuse_left_idx[i]);
    }
    for (u8_t i = 0; i < 3; ++i) {
      pwm_->SetLed(pwm_val, wing_idx[i]);
    }
  }

  // Color mode
  void UpdateColorMode() {
    u8_t smode = Submode();
    RGB nav = GetNavColor(spon_brt_);
    spon_.Fill(nav);

    color_mode_state_.SetOpScale(1 << 6);
    color_mode_state_.SetOpOffset(0);
    ifuse_. FillOp(color_mode_state_);
    ofuse_.FillOp(color_mode_state_);

    if (smode == 1) {
      iwing_. Fill(RGBW(brt_));
      owing_. Fill(RGBW(brt_));
      stab_.Fill(nav);
      inac_.Fill(nav);
      onac_.Fill(nav);
    } else {
      color_mode_state_.SetOpOffset(100);
      color_mode_state_.SetOpScale(1 << 6);
      owing_. FillOp(color_mode_state_);
      color_mode_state_.SetOpScale(1 << 4);  // .25
      iwing_. FillOp(color_mode_state_);

      color_mode_state_.SetOpOffset(10);
      stab_.FillOp(color_mode_state_);
      RGBW last_pix = stab_.At(stab_.len() - 1);
      inac_. Fill(last_pix);
      onac_.Fill(last_pix);
    }

    u8_t val = brt_;
    u16_t pwm_val = Pca9685::Apparent2Pwm(val);
    for (u8_t i = 0; i < 6; ++i) {
      pwm_->SetLed(pwm_val, fuse_right_idx[i]);
      pwm_->SetLed(pwm_val, fuse_left_idx[i]);
    }
    for (u8_t i = 0; i < 3; ++i) {
      pwm_->SetLed(pwm_val, wing_idx[i]);
    }
  }

  // Pulse mode
  void UpdatePulseMode() {
    /*
    u8_t smode = Submode();
    RGB nav = GetNavColor(spon_brt_);
    spon_.Fill(nav);
    bool flash = pulse_mode_state_.flash();
    u8_t brt = flash ? spon_brt_ : brt_;

    u8_t tail_temp = pulse_mode_state_.temp(MAIN_CNT + BACK_CNT);
    RGBW bw = Lookup5(eng_grad, tail_temp);
    Fade(&bw, spon_brt_);
    tail_frt_.Fill(bw);
    tail_bck_.Fill(bw);
    tail_isd_.Fill(bw);
    tail_osd_.Fill(bw);
    if (smode == 2) {
    bw.wht = brt;
    } else {
      bw = RGBW(brt_);  // default and simple
    }
    wing_.Fill(bw);

    u8_t pwm_idx = 0;
    u16_t pwm_sum = 0;
    i8_t pwm_cnt = 0;
    for (u8_t i = 0; i < MAIN_CNT; ++i) {
      u8_t t = pulse_mode_state_.temp(i);
      RGB pix = Lookup5(eng_grad, t);
      Fade(&pix, spon_brt_);
      f_isd_.At(i) = pix;
      f_osd_.At(i) = pix;
      pwm_sum += u16_t(10) * t;
      if (++pwm_cnt == 10) {
        pwm_sum  = pwm_sum >> 7;
        u8_t v = (pwm_sum > 255) ? 255 : pwm_sum;
        pwm_val_[pwm_idx++] = bscale8(Logify(v), spon_brt_);
        pwm_sum = 0;
        pwm_cnt = 0;
      }
    }
    pwm_sum = 0;
    pwm_cnt = 0;
    for (u8_t i = 0; i < BACK_CNT; ++i) {
      u8_t t = pulse_mode_state_.temp(FUSE_CNT + i);
      RGB pix = Lookup5(eng_grad, t);
      Fade(&pix, spon_brt_);
      back_.At(i) = pix;
      pwm_sum += t;
      if (++pwm_cnt == 8) {
        u8_t v = pwm_sum >> 3;
        pwm_val_[pwm_idx++] = bscale8(Logify(v), spon_brt_);
        pwm_sum = 0;
        pwm_cnt = 1;
      }
    }

    // In default mode just let pwm from above show (no flash).
    // In simple mode set it to slider brightness
    // In fancy mode flash it at spon_brt
    if (Submode() == 1 || (Submode() == 2 && flash)) {
      if (Submode() == 1) brt = brt_;  // dont' flash in
      for (u8_t i = 0; i < NUM_PWM; ++i) {
        pwm_val_[i] = brt;
      }
    }
    DBG_HI(APP, ("PulseU: flash: %c pwm_idx: %d "
                 "pwm[%02x, %02x, %02x, %02x, %02x]\n",
                 (flash ? 'T' : 'F'), pwm_idx, pwm_val_[0], pwm_val_[1],
                 pwm_val_[2], pwm_val_[3], pwm_val_[4]));
    */
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
    if (sbus->GetDataFrames() == 0) return false;

    state_change_ |= state_.Set(sbus);
    return (state_change_ != CtrlState::CHG_NONE);
  }

  void SendState(SyncableChannel* ch) {
    ch->WriteByte(CMD_CTRL_STATE);
    state_.Send(ch, 6);

    ch->WriteByte(CMD_SLD_STATE);
    solid_state_.Send(ch, 6);

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
    case CMD_SLD_STATE:
      solid_state_.Receive(packet + 1, 6);
      break;
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
    solid_state_.UpdateState(is_host, state_.thr, brt_, spon_brt_);
    color_mode_state_.UpdateState(is_host, Submode(), state_.thr, spon_brt_);
    pulse_mode_state_.UpdateState(is_host, Submode(), state_.thr);
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
    iwing_. Rotate();
    owing_. Rotate();
    spon_. Rotate();
    stab_. Rotate();
    inac_. Rotate();
    onac_.Rotate();
    ofuse_. Rotate();
    ifuse_. Rotate();
  }

  u8_t host_;
  PinId led_pin_;
  Pca9685* const pwm_;
  u8_t state_change_;
  u8_t pwm_val_[5];
  u8_t mode_, brt_, spon_brt_;
  CtrlState state_;
  SolidState solid_state_;
  ColorModeState color_mode_state_;
  PulseModeState pulse_mode_state_;
  LedSpan<RGBW> iwing_, owing_, spon_, stab_, inac_, onac_;
  LedSpan<RGB>  ofuse_, ifuse_;
};

class StateManager {
public:
  StateManager(Serial* com, Lights* lights) :
    ch_(com),
    lights_(lights),
    now_(0), prev_update_(0), update_cnt_(0),
    host_(false),
    state_pkts_(0)
  {
    ch_.Channel()->Setup
      (115200, 8, Serial::PARITY_EVEN, 1, /*invert=*/false,
       /*use_alt_pins=*/false, Serial::MODE_RX,
       /*use_pullup=*/true, /*buffered=*/true);
  }

  bool IsHost() const { return host_; }
  bool IsClient() const { return !host_; }

  u32_t GetNow() const { return now_; }

  u8_t GetBlinkPattern() {
    const u8_t pat_host = 0xCC; // Fast blink
    const u8_t pat_client = 0x0C; // Slow/Fast
    // const u8_t pat_search = 0x0F; // Slow blink
    return host_ ? pat_host : pat_client;
  }

  void ClaimClient() {
    DBG_LO(APP, ("Claim Client!\n"));
    host_ = false;
    lights_->SetIsHost(false);
  }

  void ClaimHost() {
    DBG_LO(APP, ("ClaimHost!\n"));
    ch_.Channel()->Setup
      (115200, 8, Serial::PARITY_EVEN, 1, /*invert=*/false,
       /*use_alt_pins=*/false, Serial::MODE_TX,
       /*use_pullup=*/true, /*buffered=*/true);
    host_ = true;
    lights_->SetIsHost(true);
  }

  void DoUpdate() {
    update_cnt_++;
    lights_->Update(now_);
  }

  // Called roughly every 32ms (32 fps)
  void HostUpdate(SBus* sbus, bool force_state_update = false) {
    if (IsClient()) return; // Not known host so don't trigger update.

    DBG_HI(APP, ("Host Update\n"));
    now_ = FastTimeMs();
    u8_t now_8 = now_ >> 8;  // 1/4 seconds
    // Update state if forced,  sbus settings change or 1/4 sec has elapsed.
    if (force_state_update || now_8 != prev_update_ ||
        lights_->SBusUpdate(sbus) ) {
      DBG_MD(APP, ("Sending State..\n"));
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
    DBG_MD(APP, ("Packet: %02x spkts: %d\n", packet[0], state_pkts_));
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
    if (IsHost()) return;  // Not reading from host because this is the host...

    while (ch_.DoRead()) {
      u8_t* packet = ch_.GetPacket();
      if (packet == NULL) continue;
      ProcessPacket(packet);
    }
  }

  void CheckSBus(SBus* sbus) {
    if (IsClient()) return;  // Client doesn't read from SBus
    sbus->Run();
  }

  void Run(SBus* sbus) {
    if (host_) {
      CheckSBus(sbus);
    } else {
      CheckChannel();
    }
  }

protected:
  SyncableChannel ch_;
  Lights* lights_;
  u16_t now_;
  u8_t prev_update_;
  u8_t update_cnt_;
  bool host_;
  u8_t state_pkts_;
  u8_t ctrl_packet_[7];
  u8_t clr_packet_[7];
  u8_t pls_packet_[7];
};

class PwmCmdHandler : public CmdHandler {
public:
  explicit PwmCmdHandler(Pca9685& pwm) :
    CmdHandler("pwm"), pwm_(&pwm) {}

  virtual void HandleLine(const char* args) {
    int led, val;
    int cnt = sscanf(args, "%d %d", &led, &val);
    if (cnt != 2) {
      DBG_LO(APP, ("Unable to scan 2 ints, found %d ints\n", cnt));
      return;
    }
    pwm_->SetLed(val, led);
    pwm_->Write();
    for (int i = 0; i < 16; ++i) {
      DBG_LO(APP, ("PWM[%d]: %04X\n", i, pwm_->led(i)));
    }
  }
  Pca9685* pwm_;
};

class LightsCmd : public CmdHandler {
public:
  LightsCmd(Lights* lights)
    : CmdHandler("lights"), lights_(lights) { }

  virtual void HandleLine(const char* args) {
    CtrlState state = lights_->State();
    int iter = 0;
    while (*args) {
      while (isspace(*args)) ++args;
      if (!args[0] || !args[1]) break;
      int len = 0;
      int val;
      int cnt = sscanf(args + 2, "%d%n", &val, &len);
      if (cnt != 1) break;
      ++iter;
      switch (*args) {
      case 'm': case 'M':
        state.mode = val;
        break;
      case 'l': case 'L':
        state.level = val;
        break;
      case 'b': case 'B':
        state.brt = val;
        break;
      case 't': case 'T':
        state.thr = val;
        break;
      default:
        break;
      }
      args += 2 + len;
    }
    if (iter != 0) {
      lights_->ApplyState(state);
    }
  }

private:
  Lights* lights_;
};

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/1, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  Twi::twi.Setup(Twi::PINS_DEF, Twi::I2C_1M);

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  DBG_LEVEL_HI(SBUS);
  SBus sbus(&Serial::usart1, /*invert=*/true);

  // The host pin is shorted to ground on the host, so setting
  // it for input with the pullup resitor enabled means it will
  // read high on the client and low on the host.
  PinId host_pin(PIN_C7);
  host_pin.SetInput(/*inverted=*/false, /*pullup=*/true);
  PinId led_pin(LED_PIN);
  led_pin.SetOutput();
  PinId blink_pin(PIN_F2);
  blink_pin.SetOutput();


  sei();
  DBG_MD(APP, ("Hello World: Test\n"));

  memset(led_data, 0, sizeof(led_data));
  SendWS2812(LED_PIN, led_data, sizeof(led_data), 0xFF);

  Pca9685 pwm(0x80, 16);
  pwm.Init(/*totem=*/true);
  pwm.SetLeds(0, 0, 16);
  pwm.Write();
  Lights lights(LED_PIN, &pwm);
  StateManager state_mgr(&Serial::usart3, &lights);

  DbgCmds cmds(&Serial::usart0);
  VARCMDS_INIT(cmds);
  PwmCmdHandler pwm_cmd(pwm);
  cmds.RegisterHandler(&pwm_cmd);
  LightsCmd lights_cmd(&lights);
  cmds.RegisterHandler(&lights_cmd);

  if (host_pin.in()) {
    state_mgr.ClaimClient();
  } else {
    state_mgr.ClaimHost();
  }
  host_pin.SetOutput();  // Disable pin, we don't need it anymore.

  DBG_MD(APP, ("Entering Run Loop\n"));
  u8_t update_3 = 0;
  u8_t update_5 = 0;
  u8_t update_8 = 0;
  while (1) {
    u16_t now = FastTimeMs();
    state_mgr.Run(&sbus);

    const u8_t now_3 = now >> 3;
    if (now_3 == update_3) continue;
    update_3 = now_3;
    cmds.Run();

    const u8_t now_5 = now >> 5;
    if (now_5 == update_5) continue;
    update_5 = now_5;
    state_mgr.HostUpdate(&sbus);  // Won't do anything if not known host

    const u8_t now_8 = now >> 8;
    if (now_8 == update_8) continue;
    update_8 = now_8;
    u8_t phase = (now_8 & 0x07);
    u8_t pat = state_mgr.GetBlinkPattern();
    blink_pin.set(pat & (1 << phase));
    // Serial::usart0.DumpReadBuffer();
    // if (state_mgr.IsHost()) sbus.Dump();
  }
}
