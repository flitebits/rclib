// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _WS2812_H_
#define _WS2812_H_

#include "IntTypes.h"
#include "Pins.h"

void SendWS2812(PinId pin, void* ptr, u16_t len, u8_t scale);

#endif  // _WS2812_H_
