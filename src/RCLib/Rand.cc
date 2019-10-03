#include "Rand.h"

namespace {
  u32_t seed;
}  // namespace

void randomSeed(u16_t new_seed) {
  seed = new_seed;
}
u8_t random8(u8_t range) {
  seed = seed * 134775813L + 1;
  return (u16_t(((seed >> 16) & 0xFF) + 1) * range) >> 8;
}
u16_t random(u16_t range) {
  seed = seed * 134775813L + 1;
  return (u32_t(((seed >> 8) & 0xFFFF) + 1) * range) >> 16;
}

