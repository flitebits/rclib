// Copyright 2020,2021 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include "Effects.h"

#include "../RtcTime.h"
#include "FPMath.h"
#include "Pixel.h"
#include "Rgb.h"

namespace led {

const RGB Pacifica::palette_1[9] = 
  { RGB(0x00, 0x05, 0x07), RGB(0x00, 0x03, 0x0B), RGB(0x00, 0x02, 0x10),
    RGB(0x00, 0x01, 0x14), RGB(0x00, 0x00, 0x19), RGB(0x00, 0x00, 0x1C),
    RGB(0x00, 0x00, 0x31), RGB(0x00, 0x00, 0x46), RGB(0x28, 0xAA, 0x50) };
const RGB Pacifica::palette_2[9] = 
  { RGB(0x00, 0x05, 0x07), RGB(0x00, 0x04, 0x09), RGB(0x00, 0x03, 0x0B),
    RGB(0x00, 0x02, 0x10), RGB(0x00, 0x01, 0x14), RGB(0x00, 0x00, 0x19),
    RGB(0x00, 0x00, 0x26), RGB(0x00, 0x00, 0x46), RGB(0x19, 0xBE, 0x5F) };
const RGB Pacifica::palette_3[9] = 
  { RGB(0x00, 0x02, 0x08), RGB(0x00, 0x05, 0x14), RGB(0x00, 0x08, 0x20),
    RGB(0x00, 0x0C, 0x33), RGB(0x00, 0x10, 0x40), RGB(0x00, 0x18, 0x60),
    RGB(0x00, 0x20, 0x80), RGB(0x10, 0x40, 0xBF), RGB(0x20, 0x60, 0xFF) };

// Add this layer of waves into the led array
void Pacifica::Layer::Run(RGB* leds, u16_t nLed)
{
  // 
  const u16_t wave_scale_half = (wave_scale >> 1) + 20;
  u16_t color_index = color_offset;

  for (u16_t i = 0; i < nLed; ++i) {
    // increment just under 1 deg/pixel
    u16_t wave_angle = offset + (250 * i);
    // Color step is a small .8fp result
    u16_t color_step = scale16(wave_scale_half, sin16(wave_angle) + 32767) +
      wave_scale_half;
    // This is the original code:
    //   u16_t color_index = color_offset + cs * i;
    //
    // This seems flawed, since cs our color 'step' already varies by
    // i this makes the same change in cs result in changes in color
    // index that grow substantially as i increases.  This will cause
    // the waves to be closer together as you move along the strip
    // eventually becoming essentially noise.  Perhaps this is
    // intentional to simulate waves approaching a beach.  If so it
    // would only really work well for a specifc # of leds, and I
    // think this more-uniform version is probably better. If you
    // wanted the original effect I would suggest adding a scaling
    // factor to 'wave_angle' that is related to i / nLed so it could
    // scale with strip length.
    color_index += color_step;
    u8_t c_idx = (sin16(color_index) + 32767) >> 8;
    *leds = Lookup9(pallete, c_idx);
    Fade(leds++, bright);
  }
}

void Pacifica::Run(RGB* led, u16_t nLed)
{
  u8_t deltams = 32;

  // Increment the four "color index start" counters, one for each
  // wave layer.  Each is incremented at a different speed, and the
  // speeds vary over time.
  i16_t speedfactor1 = bpm16Ranged(3, 179, 269);
  i16_t speedfactor2 = bpm16Ranged(4, 179, 269);
  u16_t deltams1 = (deltams * speedfactor1) >> 8;
  u16_t deltams2 = (deltams * speedfactor2) >> 8;
  u16_t deltams21 = (deltams1 + deltams2) >> 1;

  layers[0].color_offset += (deltams1 * bpm888Ranged(1011, 10, 13));
  layers[0].wave_scale   = bpm16Ranged(3, 11 << 8, 14 << 8);
  // layers[0].bright       = bpm8Ranged(10, 70, 130);
  layers[0].bright       = bpm8Ranged(10, 140, 255);
  layers[0].offset       = 0 - bpm16(301);

  layers[1].color_offset -= (deltams21 * bpm888Ranged(777, 8, 11));
  layers[1].wave_scale   = bpm16Ranged(4, 6 << 8, 9 << 8);
  layers[1].bright       = bpm8Ranged(17, 80, 160);
  layers[1].offset       = bpm16(401);

  layers[2].color_offset -= (deltams1 * bpm888Ranged(501, 5, 7));
  layers[2].wave_scale   = 6 * 256;
  layers[2].bright       = bpm8Ranged(9, 20, 80);
  layers[2].offset       = 0 - bpm16(503);

  layers[3].color_offset -= (deltams2 * bpm888Ranged(257, 4, 6));
  layers[3].wave_scale   = 5 * 256;
  layers[3].bright       = bpm8Ranged(8, 20, 60);
  layers[3].offset       = bpm16(601);
  
  for (int i = 0; i < 4; ++i){
    layers[i].Run(led, nLed);
  }

  u8_t basethreshold = bpm8Ranged( 9, 55, 65);
  u8_t wave = bpm8( 7 );

  for (u16_t i = 0; i < nLed; ++i) {
    // Add extra 'white' to areas where the four layers of light have
    // lined up brightly
    u8_t threshold = basethreshold + scale8(20, sin8( wave) + 1) ;
    wave += 7;
    RGB& p = led[i];
    p += RGB(1, 4, 8);
    u8_t l = p.average();
    if (l > threshold) {
      u8_t overage = l - threshold;
      u8_t overage2 = led::addsat(overage, overage);
      p += RGB(overage, overage2, led::addsat(overage2, overage2));
    }

    //deepen the blues and greens
    p.blu  = scale8(p.blu,  145); 
    p.grn = scale8(p.grn, 200); 
  }
}

}  // namespace led
