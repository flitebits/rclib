// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef UTIL_H_
#define UTIL_H_

#include "IntTypes.h"

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

#define write_ccp(ptr, val) \
  __asm__ __volatile__ ( \
  "out	%[ccp_reg], %[ioreg_cen_mask]	\n\t" \
  "sts   %[ccp_ptr], %[ccp_val]		\n\t" \
  : /* no outputs */ \
  : [ccp_reg]			"I"  (_SFR_IO_ADDR(CCP)),     \
    [ioreg_cen_mask]		"r"  ((uint8_t)CCP_IOREG_gc), \
    [ccp_ptr]			"n"  (ptr), \
    [ccp_val]			"r"  ((uint8_t)val)	\
    );

long GetMainClock();
u8_t GetPerClockScale();
long GetPerClock();

i8_t GetMainClockErr(bool at5V = true);

void memset(void* ptr, int len, u8_t val);
void memmove(void* dst, void* src, u16_t len);
#endif /* UTIL_H_ */
