// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _LED_EFFECTS_
#define _LED_EFFECTS_

#include "../IntTypes.h"
#include "Rgb.h"

namespace led {
  class Pacifica {
    // Simulates waves in the ocean, borrowed from WLED, but reworked
    // substantially.
  public:
    Pacifica() {
      layers[0].pallete = palette_1;
      layers[0].color_offset = 0;
      layers[1].pallete = palette_2;
      layers[1].color_offset = 0;
      layers[2].pallete = palette_3;
      layers[2].color_offset = 0;
      layers[3].pallete = palette_3;
      layers[3].color_offset = 0;
    }
    void Run(RGB* leds, u16_t nLed);

  protected:
    struct Layer {
      void Run(RGB* leds, u16_t nLed);
      // Angle of the first pixel in strip.
      u16_t offset;
      // small 8.8 fp value (<=16 so really 4.8), scaled by sin of pixel angle.
      // 
      u16_t wave_scale;
      u16_t color_offset;
      u8_t bright;
      const led::RGB* pallete;
    };

    Layer layers[4];
    static const RGB palette_1[9];
    static const RGB palette_2[9];
    static const RGB palette_3[9];
  };
}  // namespace led

#endif  // _LED_EFFECTS_
