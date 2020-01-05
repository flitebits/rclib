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
