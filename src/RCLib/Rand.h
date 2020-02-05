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

// Sets the seed to new_seed, you will always get the same sequence of
// results from the same seed.
void randomSeed(u16_t new_seed);
// Mix whatever the current seeds value is with new_seed.  This can be used
// to periodically inject some new randomness into the sequence.
void randomSeedMix(u16_t new_seed);
// returns a random number between zero and 'range' (255 max).
u8_t random8(u8_t range);
// returns a random number between zero and 'range' (165535 max).
u16_t random(u16_t range);

#endif  // _RAND_
