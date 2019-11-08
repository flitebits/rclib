#include "RtcTime.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "IntTypes.h"

namespace {
  volatile u32_t timer_millis = 0;
  volatile u16_t timer_secs = 0;
  ISR(RTC_CNT_vect) {
    timer_millis += 1000;
    timer_secs++;
    RTC.INTFLAGS = RTC_OVF_bm;
  }
}  // anonymous namespace

void SetupRtcClock(bool use_internal_32K) {
  RTC.CLKSEL =
    use_internal_32K ? RTC_CLKSEL_INT32K_gc : RTC_CLKSEL_TOSC32K_gc;
  RTC.PER = 32767;  // wrap counter 1/sec
  RTC.INTCTRL = (1 << RTC_OVF_bp);  // Enabable OVERFLOW interrupt
  RTC.CTRLA = ((1 << RTC_RUNSTDBY_bp)  |
	       (RTC_PRESCALER_DIV1_gc) |
	       (1 << RTC_RTCEN_bp));
}

i16_t FastMs() {
  u16_t cnt = RTC.CNT;
  return cnt >> 5;
}

u16_t FastSecs() {
  return timer_secs;
}

u32_t FastTimeMs() {
  volatile u32_t cnt;
  volatile u16_t sec;
  volatile u8_t flgs;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    cnt = RTC.CNT;
    sec = timer_secs;
    flgs = RTC.INTFLAGS;
  }
  // Interrupt fired and cnt overflowed while we were blocking
  // interrupts.
  if ((flgs & RTC_OVF_bm) && (cnt == 0)) {
    ++sec;
  }
  return (sec << 10) + ((cnt + (1<<4)) >> 5);
}

u32_t TimeMs() {
  volatile u32_t cnt;
  volatile u32_t millis;
  volatile u8_t flgs;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    cnt = RTC.CNT;
    millis = timer_millis;
    flgs = RTC.INTFLAGS;
  }
  if ((flgs & RTC_OVF_bm) && (cnt == 0)) {
    millis += 1000;
  }
  return millis + ((cnt * 1000UL + (1 << 14)) >> 15); 
}
