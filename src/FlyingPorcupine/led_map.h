// Copyright 2024 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _LED_MAP_
#define _LED_MAP_

enum part_tags {
                TAG_ENGINE,
                TAG_NOZEL,
                TAG_FLAME,
                TAG_BODY,
                TAG_EYE,
                TAG_COUNT,
};

struct LedRange {
  u8_t xloc, yloc;
  bool is_rgbw : 1;
  part_tags tag : 4;
};

#define TAG_ENGINE_START (0)
#define TAG_ENGINE_LEN (30)
#define TAG_NOZEL_START (TAG_ENGINE_START + TAG_ENGINE_LEN)
#define TAG_NOZEL_LEN (4)
#define TAG_FLAME_START (TAG_NOZEL_START + TAG_NOZEL_LEN)
#define TAG_FLAME_LEN (12)
#define TAG_BODY_START (TAG_FLAME_START + TAG_FLAME_LEN)
#define TAG_BODY_LEN (59)
#define TAG_EYE_START (TAG_BODY_START + TAG_BODY_LEN)
#define TAG_EYE_LEN (1)

const LedRange ranges[] =
  {
   121,  32, true, TAG_ENGINE,
   138,  34, true, TAG_ENGINE,
   154,  36, true, TAG_ENGINE,
   170,  37, true, TAG_ENGINE,
   171,  24, true, TAG_ENGINE,
   155,  23, true, TAG_ENGINE,
   138,  23, true, TAG_ENGINE,
   121,  22, true, TAG_ENGINE,
   121,  10, true, TAG_ENGINE,
   138,   9, true, TAG_ENGINE,
   155,   8, true, TAG_ENGINE,
   171,   7, true, TAG_ENGINE,

   174,  17, false, TAG_ENGINE,
   167,  17, false, TAG_ENGINE,
   160,  17, false, TAG_ENGINE,
   153,  17, false, TAG_ENGINE,
   146,  17, false, TAG_ENGINE,
   139,  17, false, TAG_ENGINE,
   132,  16, false, TAG_ENGINE,
   125,  16, false, TAG_ENGINE,
   118,  16, false, TAG_ENGINE,

   120,  28, false, TAG_ENGINE,
   127,  28, false, TAG_ENGINE,
   134,  29, false, TAG_ENGINE,
   141,  29, false, TAG_ENGINE,
   148,  29, false, TAG_ENGINE,
   155,  29, false, TAG_ENGINE,
   161,  29, false, TAG_ENGINE,
   168,  29, false, TAG_ENGINE,
   176,  30, false, TAG_ENGINE,

   104,  18, false, TAG_NOZEL,
   105,  21, false, TAG_NOZEL,
   105,  24, false, TAG_NOZEL,
   105,  28, false, TAG_NOZEL,

    11,  20, true, TAG_FLAME,
    28,  20, true, TAG_FLAME,
    45,  20, true, TAG_FLAME,
    62,  19, true, TAG_FLAME,
    78,  19, true, TAG_FLAME,
    93,  24, true, TAG_FLAME,

    86,  26, false, TAG_FLAME,
    79,  26, false, TAG_FLAME,
    72,  26, false, TAG_FLAME,
    65,  25, false, TAG_FLAME,
    58,  25, false, TAG_FLAME,
    51,  25, false, TAG_FLAME,

    68,  54, true, TAG_BODY,
    85,  53, true, TAG_BODY,
   101,  53, true, TAG_BODY,
   118,  52, true, TAG_BODY,
   134,  52, true, TAG_BODY,
   151,  53, true, TAG_BODY,
   167,  55, true, TAG_BODY,
   186,  63, true, TAG_BODY,
   208,  63, true, TAG_BODY,
   224,  67, true, TAG_BODY,
   243,  75, true, TAG_BODY,  // 11
   238,  85, true, TAG_BODY,  // 2nd row
   222,  83, true, TAG_BODY,
   211,  75, true, TAG_BODY,
   194,  74, true, TAG_BODY,
   178,  72, true, TAG_BODY,
   161,  70, true, TAG_BODY,
   145,  68, true, TAG_BODY,
   129,  67, true, TAG_BODY,
   112,  66, true, TAG_BODY,
    96,  65, true, TAG_BODY,
    80,  66, true, TAG_BODY,
    63,  67, true, TAG_BODY,
    47,  68, true, TAG_BODY,  // 13
    44,  81, true, TAG_BODY,  // 3rd row
    61,  81, true, TAG_BODY,
    77,  81, true, TAG_BODY,
    93,  81, true, TAG_BODY,
   109,  81, true, TAG_BODY,
   125,  82, true, TAG_BODY,
   142,  82, true, TAG_BODY,
   158,  84, true, TAG_BODY,
   175,  85, true, TAG_BODY,
   191,  86, true, TAG_BODY,
   207,  88, true, TAG_BODY,
   224,  90, true, TAG_BODY,
   240,  92, true, TAG_BODY,  // 13
   220, 100, true, TAG_BODY,  // 4th row
   204, 100, true, TAG_BODY,
   187, 100, true, TAG_BODY,
   170,  99, true, TAG_BODY,
   154,  98, true, TAG_BODY,
   138,  97, true, TAG_BODY,
   121,  96, true, TAG_BODY,
   105,  95, true, TAG_BODY,
    89,  95, true, TAG_BODY,
    73,  95, true, TAG_BODY,
    57,  95, true, TAG_BODY,  // 11
    79, 106, true, TAG_BODY,  // 5th row
    95, 107, true, TAG_BODY,
   111, 108, true, TAG_BODY,
   127, 109, true, TAG_BODY,
   143, 110, true, TAG_BODY,
   158, 112, true, TAG_BODY,
   148, 124, true, TAG_BODY,
   162, 121, true, TAG_BODY,
   175, 111, true, TAG_BODY,
   193, 108, true, TAG_BODY,
   209, 107, true, TAG_BODY,  // 11

   230, 79, true, TAG_EYE,
  };

struct BodyRowInfo {
  u8_t idx;
  u8_t len;
};

#define BODY_ROW0_IDX (0)
#define BODY_ROW0_LEN (11)
#define BODY_ROW1_IDX (BODY_ROW0_IDX + BODY_ROW0_LEN)
#define BODY_ROW1_LEN (13)
#define BODY_ROW2_IDX (BODY_ROW1_IDX + BODY_ROW1_LEN)
#define BODY_ROW2_LEN (13)
#define BODY_ROW3_IDX (BODY_ROW2_IDX + BODY_ROW2_LEN)
#define BODY_ROW3_LEN (11)
#define BODY_ROW4_IDX (BODY_ROW3_IDX + BODY_ROW3_LEN)
#define BODY_ROW4_LEN (11)

BodyRowInfo body_rows[5] = {
  { BODY_ROW0_IDX, BODY_ROW0_LEN },
  { BODY_ROW1_IDX, BODY_ROW1_LEN },
  { BODY_ROW2_IDX, BODY_ROW2_LEN },
  { BODY_ROW3_IDX, BODY_ROW3_LEN },
  { BODY_ROW4_IDX, BODY_ROW4_LEN },
};


#endif  // _LED_MAP_
