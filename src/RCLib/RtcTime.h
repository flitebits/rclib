// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _RTC_TIME_
#define _RTC_TIME_

#include "IntTypes.h"

// Must be called to Setup the Real time clock.
void SetupRtcClock(bool use_internal_32K);

// Returns milliseconds since boot as unsigned long (good for ~50
// days).  This is kind of expensive. If you don't care too much use
// FastTimeMs, or even lighter weight FastMs or FastSecs.
u32_t TimeMs();

// Returns FastSecs + FastMs as a long (avoiding ms wrap issues)
// Value wraps at ~18hrs, since seconds is only 16bits.
u32_t FastTimeMs();

// Returns Seconds since boot (unsigned 16 bit int so wraps ~18hrs).
// Note that useing FastSecs and FastMs/FastFrac is frought with
// perile since at some point the fractional seconds will wrap and
// seconds will not have wrapped. Just use FastMsLong if you need this.
u16_t FastSecs();

// Returns 1024ths of a second, only counts to one second (then
// wraps).  Useful for quick common update checks:
// u8_t update_5 = 0;  // can use u8_t since FastMs only had 10bits to start.
// ...
// u8_t now_5 = FastMs() >> 5;  // will change every ~32ms
// if (now_5 != update_5) {
//    update_5 = now_5;
//    ... Do update
i16_t FastMs();

#endif  // _RTC_TIME_
