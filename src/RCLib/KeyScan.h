// Copyright 2022 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "IntTypes.h"
#include "Pins.h"

#ifndef KEY_SCAN_H_
#define KEY_SCAN_H_

template <int rows, int cols>
class KeyScan {
 public:
  // When scanning row_pins are set as input with pull-up enabled (meaning they
  // read high normally).  The col_pins are set as output defaulting to high.
  // Then one col_pin at a time is set low and each row is scanned, if the key
  // is pressed the row will be pulled low by the low col, otherwise it will
  // stay high.  If your key matrix has diods then you need to make sure the
  // diode allows current to flow from row (high) to col (low), otherwise you
  // may need to swap rows and cols.
  //
  // NB: If you have the choice have there be more rows and cols.  It is much
  // slower to scan columns than rows since every time it changes the column it
  // needs to set the pin low and give it time to settle, where as row pins can
  // all just be read.
  KeyScan(PinIdEnum row_pins[rows], PinIdEnum col_pins[cols])
    : prev_state_(0),
      curr_state_(0) {
    for (u8_t r=0; r < rows; ++r) {
      row_pins_[r].SetPin(row_pins[r]);
      // Set pullup true so they are high unless pulled down through
      // the key by the current row pin being set low.
      row_pins_[r].SetInput(/*inverted=*/false, /*pullup=*/true);
    }
    for (u8_t c = 0; c < cols; ++c) {
      col_pins_[c].SetPin(col_pins[c]);
      col_pins_[c].SetOutput(true);
    }
  }

  // Helper method to map row/col to 'index' space which is what 'state' is
  // populated with.
  static u8_t ToIdx(u8_t row, u8_t col) { return row * cols + col; }
  static u8_t IdxToRow(u8_t idx) { return idx / cols; }
  static u8_t IdxToCol(u8_t idx) { return idx % cols; }

  // Number of possible keys
  u8_t NumKeys() const { return rows * cols; }

  // Returns true if row/col or idx key is down.
  bool IsDown(u8_t row, u8_t col) const { return IsDown(ToIdx(row, col)); }
  bool IsDown(u8_t idx) const { return curr_state_ & (1 << idx); }

  // Returns the state of the first 32 keys
  u32_t State() const{ return curr_state_; }

  // Returns a bitset of keys that changed value as a result of the most recent
  // call to Scan.
  u32_t Changed() const {
    return prev_state_ ^ curr_state_;
  }


  void Scan() {
    u32_t new_state = 0;
    for (u8_t c = 0; c < cols; ++c) {
      col_pins_[c].set(false);
      // Wait at least one clock tick for output/input to settle, this
      // may wait up to ~2 clock ticks to make sure that at least one full tick has
      // passed (otherwise RTC.CNT might change on the next tick).
      for (volatile u8_t tick = 0; tick < 32; ++tick);
      for (u8_t r = 0, idx = c; r < rows; ++r, idx += cols) {
        u8_t state = states_[idx >> 1];
        bool is_down = false;
        if (idx & 0x1) {
          state = (((state & 0xE0) >> 1)) | (state & 0x0F);
          if (!row_pins_[r].in()) state |= (1 << 7);
          is_down = IsDownState(state >> 4);
        } else {
          state = (state & 0xF0) | (((state & 0x0E) >> 1));
          if (!row_pins_[r].in()) state |= (1 << 3);
          is_down = IsDownState(state & 0xF);
        }
        if (is_down) new_state |= 1 << idx;
        states_[idx >> 1] = state;
      }
      col_pins_[c].set(true);
    }
    prev_state_ = curr_state_;
    curr_state_ = new_state;
  }

 protected:
  // Based on last 4 scans is the key stably up or down?
  static bool IsDownState(u8_t state) {
    if (state >= 0x0C) return true; // 011??
    if (state == 0x0A) return true; // 01011
    return false;
  }

  u8_t State(int idx) {
    u8_t state = states_[idx >> 1];
    if (idx & 0x1) {
      return state >> 4;
    }
    return state & 0x0F;
  }

  PinId row_pins_[rows];
  PinId col_pins_[cols];
  // Used to track key state over the last 4 scans used to debounce keys.
  u8_t states_[(rows * cols + 1) / 2];
  u32_t prev_state_;
  u32_t curr_state_;
};

#endif  /* KEY_SCAN_H_ */
