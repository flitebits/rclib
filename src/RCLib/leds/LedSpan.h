// Copyright 2022 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <stdlib.h>
#include "IntTypes.h"

// Provides a convenient way to reference a run of pixels out of a larger string
// of leds.  This is nice for situations where you have logically discreate
// groups of leds that are all part of one longer serial chain of leds.
template <class P>
class LedSpan {
public:
  // 'leds' should point to the first byte of the first pixel.
  // 'len' is the length of the led run in pixels (not bytes).
  // If reverse is true then all indexing is reversed (At(0) returns
  // leds[len-1], etc).
  LedSpan(void* leds = NULL, u8_t len = 0, bool reverse = false) :
    ptr_(reinterpret_cast<P*>(leds)), len_(len), reverse_(reverse) { }

  // Can be used to initialize a Span after construction.
  // This also has the nice feature of returning a pointer to the
  // pixel immediately after this span.
  // This makes code like this possible:
  // void *ptr = led_array;
  // ptr = left_span.SetSpan(ptr, n);
  // ptr = right_span.SetSpan(ptr, m);
  // ...
  void* SetSpan(void* leds, u8_t len, bool reverse = false) {
    ptr_ = reinterpret_cast<P*>(leds);
    len_ = len;
    reverse_ = reverse;
    return ptr_ + len_;
  }

  // Returns the length of this span.
  u8_t len() const { return len_; }
  // Returns a ptr to the start of span (makes overlaped spans easier).
  void* ptr() const { return ptr_; }
  // Returns a ptr to the start of the pixel after this span (see SetSpan
  // for more info).
  void* next_ptr() const { return ptr_ + len_; }

  // Return a reference to the requested pixel (taking reverse into account).
  P& At(u8_t idx) {
    if (idx < 0) idx = 0;
    else if ( idx > len_ - 1) idx = len_ - 1;

    if (reverse_) {
      return ptr_[len_ - 1 - idx];
    }
    return ptr_[idx];
  }
  // Return a reference to the requested pixel (taking reverse into account).
  const P& At(u8_t idx) const {
    if (idx < 0) idx = 0;
    else if ( idx > len_ - 1) idx = len_ - 1;

    if (reverse_) {
      return ptr_[len_ - 1 - idx];
    }
    return ptr_[idx];
  }
  // Sets the specified pixel to the provided color.
  void Set(u8_t idx, const P& pix) {
        At(idx) = pix;
  }
  // Fills all the pixels in the span to to specified color.
  void Fill(const P& pix) {
    P* ptr = ptr_;
    P* end = ptr_ + len_;
    while (ptr != end) *(ptr++) = pix;
  }

  // Fills all the pixels in the span to to specified color.
  void Fill(const P& pix, u8_t idx, u8_t len) {
    P* ptr = reverse_ ? &At(idx + len) : &At(idx);
    P* end = ptr + len;
    while (ptr != end) *(ptr++) = pix;
  }

  // Calls 'op' with 8bit fraction (0 -> index 0, 255 -> index len-1)
  // and the associated pixel for every pixel in the span.
  template <class O>
  void FillOp(const O& op) {
    u16_t step = (u16_t(255) << 8) / len_;
    // Use step/2 so it points at the midpoint of the span the pixel covers.
    u16_t frac = step >> 1;
    for (u8_t i = 0; i < len_; ++i, frac += step) {
      P& pix = At(i);
      op(u8_t(frac >> 8), &pix);
    }
  }

  // Moves all pixels one place (wraps last one back to start).
  // forward true means N -> N+1, false means N+1 -> N (accounts for reverse_).
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
