#ifndef _DGB_
#define _DBG_

#ifndef NDEBUG
#include <stdio.h>
#include "Serial.h"
#include "IntTypes.h"

namespace dbg {
  enum Topic { ADC, SBUS, SPORT, LED, NUM_TOPIC };
  enum Level { OFF, LO, MD, HI, };

  class Dbg {
  public:
    // Must be called before any other methods are called.
    void Setup(Serial* port);

    // Set verbosity level for a topic (default is zero).
    void SetLevel(Topic topic, Level lvl) {
      while (topic >= 16);  // only supports up to 16 topics
      state_ = (state_ & ~(0x3 << (2 * topic))) | (lvl << (2 * topic));
    }
    bool Check(Topic topic, Level lvl) {
      return lvl <= ((state_ >> (2 * topic)) & 0x3);
    }
    void PutChar(char c) { if (port_) port_->WriteByte(c); }

    static Dbg dbg;
  protected:
    Dbg();
    u32_t state_;  // good for up to 16 topics
    Serial* port_;
  };
}  // namespace dbg
 
#define DBG_LEVEL_OFF(topic) ::dbg::Dbg::dbg.SetLevel(::dbg::topic, ::dbg::OFF)
#define DBG_LEVEL_LO(topic) ::dbg::Dbg::dbg.SetLevel(::dbg::topic, ::dbg::LO)
#define DBG_LEVEL_MD(topic) ::dbg::Dbg::dbg.SetLevel(::dbg::topic, ::dbg::MD)
#define DBG_LEVEL_HI(topic) ::dbg::Dbg::dbg.SetLevel(::dbg::topic, ::dbg::HI)
#define DBG_INIT(uart, speed) do { \
  uart.Setup(speed, 8, Serial::PARITY_NONE, 1); \
  ::dbg::Dbg::dbg.Setup(&uart);			\
} while(false)
#define DBG_LO(topic, args) \
  do { if (::dbg::Dbg::dbg.Check(dbg::topic, dbg::LO)) {printf args;} \
  } while(false)
#define DBG_MD(topic, args) \
  do { if (::dbg::Dbg::dbg.Check(dbg::topic, dbg::MD)) {printf args;} \
  } while(false)
#define DBG_HI(topic, args) \
  do { if (::dbg::Dbg::dbg.Check(dbg::topic, dbg::HI)) {printf args;} \
  } while(false)
#else
  // NDEBUG - Release build make everything disappear!
#define DBG_INIT(uart, speed) do { } while(false)
#define DBG_LO(topic, args) do { } while(false)
#define DBG_MD(topic, args) do { } while(false)
#define DBG_HI(topic, args) do { } while(false)
#define DBG_LEVEL_OFF(topic) do { } while(false)
#define DBG_LEVEL_LO(topic) do { } while(false)
#define DBG_LEVEL_MD(topic) do { } while(false)
#define DBG_LEVEL_HI(topic) do { } while(false)
#endif  // NDEBUG

#endif  // _DBG_
