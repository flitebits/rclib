// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _RAND_
#define _RAND_

#include "IntTypes.h"

void randomSeed(u16_t new_seed);
u8_t random8(u8_t range);
u16_t random(u16_t range);

#endif  // _RAND_
