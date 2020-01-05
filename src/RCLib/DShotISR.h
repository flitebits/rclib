// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _DSHOTISR_
#define _DSHOTISR_

#include "DShot.h"

class DShotISR : public DShot {
 public:
  DShotISR(Serial* serial, PinGroupId pins);

  virtual void Start();
  virtual bool Run(bool* new_telemetry);
};

#endif  // _DSHOTISR_
