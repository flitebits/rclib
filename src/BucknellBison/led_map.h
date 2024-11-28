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
                TAG_OCHIN,
                TAG_OCHEEK,
                TAG_HORN,
                TAG_EYE,
                TAG_MOUTH,
                TAG_FORE,
                TAG_BCHEEK,
                TAG_COUNT
};

struct led_range {
  u8_t xloc, yloc, dist;
  bool is_rgbw : 1;
  part_tags tag : 4;
};

const led_range ranges[] =
  {
    77, 200, 103, true, TAG_OCHIN,
    73, 183,  88, true, TAG_OCHIN,
    96, 215, 114, true, TAG_OCHIN,
    90, 200, 100, true, TAG_OCHIN,
    83, 185,  87, true, TAG_OCHIN,
    91, 176,  76, true, TAG_OCHIN,
    94, 187,  87, true, TAG_OCHIN,
    24,  97,  82, true, TAG_OCHEEK,
    32,  99,  74, true, TAG_OCHEEK,
    18, 105,  88, true, TAG_OCHEEK,
    34, 115,  73, true, TAG_OCHEEK,
    39, 129,  73, true, TAG_OCHEEK,
    43, 143,  76, true, TAG_OCHEEK,
    51, 148,  72, true, TAG_OCHEEK,
    47, 133,  67, true, TAG_OCHEEK,
    43, 119,  66, true, TAG_OCHEEK,
    57, 141,  63, true, TAG_OCHEEK,
    63, 150,  65, true, TAG_OCHEEK,
    68, 145,  58, true, TAG_OCHEEK,
    70, 130,  46, true, TAG_OCHEEK,
    67, 116,  42, true, TAG_OCHEEK,
    57, 112,  50, true, TAG_OCHEEK,
    76, 127,  40, false, TAG_OCHEEK,
    75, 133,  45, false, TAG_OCHEEK,
    74, 140,  50, false, TAG_OCHEEK,
    83, 144,  49, false, TAG_OCHEEK,
    89, 146,  48, false, TAG_OCHEEK,
    95, 148,  48, false, TAG_OCHEEK,
    33,  46,  91, true, TAG_HORN, // left
    24,  57,  93, true, TAG_HORN,
    32,  65,  82, true, TAG_HORN,
    39,  78,  71, true, TAG_HORN,
    30,  82,  78, true, TAG_HORN,
    25,  70,  87, true, TAG_HORN,
    64,  94,  43, true, TAG_EYE,  // left
    69, 100,  37, true, TAG_EYE,
    79, 103,  27, true, TAG_EYE,
   126, 106,  21, true, TAG_EYE,  // right
   136, 104,  30, true, TAG_EYE,
   142,  98,  36, true, TAG_EYE,
   121, 162,  63, true, TAG_MOUTH,
   106, 163,  62, true, TAG_MOUTH,
    91, 163,  64, true, TAG_MOUTH,
    83, 158,  61, true, TAG_MOUTH,
    98, 158,  58, true, TAG_MOUTH,
   113, 158,  57, true, TAG_MOUTH,
   119, 146,  47, true, TAG_MOUTH,
   108, 138,  37, true, TAG_MOUTH,
    92, 138,  40, true, TAG_MOUTH,
    97, 133,  33, true, TAG_MOUTH,
   112, 133,  33, true, TAG_MOUTH,
   157, 105,  51, true, TAG_BCHEEK,
   160,  89,  55, true, TAG_BCHEEK,
   169, 102,  63, true, TAG_BCHEEK,
   177, 104,  71, true, TAG_BCHEEK,
   188,  93,  82, true, TAG_BCHEEK,
   208, 105, 102, true, TAG_BCHEEK,
   193, 109,  87, true, TAG_BCHEEK,
   178, 112,  73, true, TAG_BCHEEK,
   163, 116,  59, true, TAG_BCHEEK,
   148, 122,  47, true, TAG_BCHEEK,
   136, 131,  42, true, TAG_BCHEEK,
   134, 143,  50, true, TAG_BCHEEK,
   147, 134,  53, true, TAG_BCHEEK,
   159, 125,  58, true, TAG_BCHEEK,
   174, 126,  72, true, TAG_BCHEEK,
   171, 140,  76, true, TAG_BCHEEK,
   165, 156,  81, true, TAG_BCHEEK,
   155, 167,  82, true, TAG_BCHEEK,
   145, 178,  86, true, TAG_BCHEEK,
   135, 189,  93, true, TAG_BCHEEK,
   123, 200, 100, true, TAG_BCHEEK,
   111, 210, 109, true, TAG_BCHEEK,
   116, 220, 119, true, TAG_BCHEEK,
   120, 236, 136, true, TAG_BCHEEK,
   108, 192,  91, true, TAG_BCHEEK,
   120, 182,  82, true, TAG_BCHEEK,
   109, 182,  81, true, TAG_BCHEEK,
   107, 201, 100, true, TAG_BCHEEK,
   119, 191,  91, true, TAG_BCHEEK,
   130, 180,  83, true, TAG_BCHEEK,
   141, 170,  77, true, TAG_BCHEEK,
   152, 160,  75, true, TAG_BCHEEK,
   158, 149,  71, true, TAG_BCHEEK,
   143, 151,  62, true, TAG_BCHEEK,
   143, 160,  70, true, TAG_BCHEEK,
   149, 142,  59, true, TAG_BCHEEK,
   161, 134,  64, true, TAG_BCHEEK,
   185,  55,  91, true, TAG_HORN,  // right
   190,  69,  90, true, TAG_HORN,
   183,  68,  84, true, TAG_HORN,
   174,  80,  71, true, TAG_HORN,
   181,  85,  77, true, TAG_HORN,
   103,  99,   4, true, TAG_FORE,  // nose
   105, 114,  13, true, TAG_FORE,
    98, 113,  14, true, TAG_FORE,
   111, 114,  14, true, TAG_FORE,
   110,  99,   4, true, TAG_FORE,
    92,  89,  18, true, TAG_FORE,
   107,  88,  13, true, TAG_FORE, // bridge
   122,  88,  21, true, TAG_FORE,
   130,  81,  31, true, TAG_FORE,
   114,  81,  22, true, TAG_FORE,
    99,  82,  20, true, TAG_FORE,
    84,  82,  29, true, TAG_FORE,
    50,  68,  65, true, TAG_FORE,
    72,  66,  49, true, TAG_FORE,
    87,  65,  41, true, TAG_FORE,
   102,  63,  38, true, TAG_FORE,
   117,  63,  40, true, TAG_FORE,
   132,  62,  47, true, TAG_FORE,
   147,  62,  57, true, TAG_FORE,
   165,  66,  69, true, TAG_FORE,
   159,  43,  79, true, TAG_FORE,
   144,  43,  69, true, TAG_FORE,
   129,  43,  62, true, TAG_FORE,
   114,  43,  59, true, TAG_FORE,
    99,  43,  58, true, TAG_FORE,
    84,  42,  63, true, TAG_FORE,
    69,  41,  70, true, TAG_FORE,
    54,  40,  80, true, TAG_FORE,
    65,  22,  89, true, TAG_FORE,
    80,  21,  84, true, TAG_FORE,
    95,  21,  81, true, TAG_FORE,
   110,  21,  80, true, TAG_FORE,
   125,  22,  81, true, TAG_FORE,
   140,  24,  84, true, TAG_FORE,
   154,  26,  89, true, TAG_FORE,
    82,  13,  91, false, TAG_FORE, // 1
    97,  13,  88, false, TAG_FORE, // 1
   112,  14,  87, false, TAG_FORE, // 1
   127,  14,  89, false, TAG_FORE, // 1
   142,  15,  93, false, TAG_FORE, // 1
   162,  36,  86, false, TAG_FORE, // 2
   147,  34,  79, false, TAG_FORE, // 2
   132,  33,  73, false, TAG_FORE, // 2
   117,  32,  70, false, TAG_FORE, // 2
   102,  31,  70, false, TAG_FORE, // 2
    87,  31,  73, false, TAG_FORE, // 2
    72,  30,  79, false, TAG_FORE, // 2
    57,  29,  87, false, TAG_FORE, // 2
    47,  56,  74, false, TAG_FORE, // 3
    62,  56,  63, false, TAG_FORE, // 3
    70,  55,  58, false, TAG_FORE, // 3
    93,  55,  48, false, TAG_FORE, // 3
   107,  55,  46, false, TAG_FORE, // 3
   122,  55,  49, false, TAG_FORE, // 3
   137,  54,  56, false, TAG_FORE, // 3
   152,  54,  66, false, TAG_FORE, // 3
   167,  54,  77, false, TAG_FORE, // 3
   140,  71,  45, false, TAG_FORE, // 4
   125,  72,  35, false, TAG_FORE, // 4
   110,  73,  28, false, TAG_FORE, // 4
    95,  74,  29, false, TAG_FORE, // 4
    80,  74,  37, false, TAG_FORE, // 4
   131, 120,  31, true, TAG_OCHEEK,
  };

#endif  // _LED_MAP_
