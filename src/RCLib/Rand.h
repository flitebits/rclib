#ifndef _RAND_
#define _RAND_

#include "IntTypes.h"

void randomSeed(u16_t new_seed);
u8_t random8(u8_t range);
u16_t random(u16_t range);

#endif  // _RAND_
