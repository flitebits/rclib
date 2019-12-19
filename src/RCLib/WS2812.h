#ifndef _WS2812_H_
#define _WS2812_H_

#include "IntTypes.h"
#include "Pins.h"

void SendWS2812(PinId pin, void* ptr, u16_t len, u8_t scale);

#endif  // _WS2812_H_
