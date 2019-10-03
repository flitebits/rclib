#include "Pwm.h"
#include "util.h"
#include "avr/io.h"

namespace {
u8_t GetIdxEnBit(u8_t idx) {
  switch (idx) {
  case 0: return TCA_SPLIT_LCMP0EN_bp; break;
  case 1: return TCA_SPLIT_LCMP1EN_bp; break;
  case 2: return TCA_SPLIT_LCMP2EN_bp; break;
  case 3: return TCA_SPLIT_HCMP0EN_bp; break;
  case 4: return TCA_SPLIT_HCMP1EN_bp; break;
  case 5: return TCA_SPLIT_HCMP2EN_bp; break;
  }
  return 0;
}
}  // anonymous namespace

Pwm::Pwm(PinGroupId port, int freqHz) {
  TCA0.SPLIT.CTRLB = 0; // Disable all
  TCA0.SPLIT.CTRLD = (TCA_SPLIT_SPLITM_bm);
  TCA0.SPLIT.LPER = 0xFF;
  TCA0.SPLIT.HPER = 0xFF;
  PORTMUX.TCAROUTEA = port;
  port_ptr_ = GetPortStruct(port);
  // Extract base clock frequencies to calculate baud rate setting.
  long base_clk = GetMainClock();
  long pdiv = GetPerClockScale();
  int div = (base_clk >> 8) / (pdiv * freqHz);
  u8_t clk_scale = TCA_SPLIT_CLKSEL_DIV1_gc;
  if (div < 2) {
    clk_scale = TCA_SPLIT_CLKSEL_DIV1_gc;
  } else if (div < 4) {
    clk_scale = TCA_SPLIT_CLKSEL_DIV2_gc;
  } else if (div < 8) {
    clk_scale = TCA_SPLIT_CLKSEL_DIV4_gc;
  } else if (div < 16) {
    clk_scale = TCA_SPLIT_CLKSEL_DIV8_gc;
  } else if (div < 64) {
    clk_scale = TCA_SPLIT_CLKSEL_DIV16_gc;
  } else if (div < 256) {
    clk_scale = TCA_SPLIT_CLKSEL_DIV64_gc;
  } else if (div < 1024) {
    clk_scale = TCA_SPLIT_CLKSEL_DIV256_gc;
  } else {
    clk_scale = TCA_SPLIT_CLKSEL_DIV1024_gc;
  }
  TCA0.SPLIT.CTRLA = (clk_scale | (1 << TCA_SPLIT_ENABLE_bp));
}

void Pwm::Enable(u8_t idx) {
  u8_t bit = GetIdxEnBit(idx);
  port_ptr_->DIRSET = (1 << idx);
  TCA0.SPLIT.CTRLB = (TCA0.SPLIT.CTRLB | (1 << bit));
}

void Pwm::Disable(u8_t idx){
  u8_t bit = GetIdxEnBit(idx);
  TCA0.SPLIT.CTRLB = (TCA0.SPLIT.CTRLB & ~(1 << bit));
  port_ptr_->DIRCLR = (1 << idx);
}

void Pwm::Set(u8_t idx, u8_t val){
  switch (idx) {
  case 0: TCA0.SPLIT.LCMP0 = val; break;
  case 1: TCA0.SPLIT.LCMP1 = val; break;
  case 2: TCA0.SPLIT.LCMP2 = val; break;
  case 3: TCA0.SPLIT.HCMP0 = val; break;
  case 4: TCA0.SPLIT.HCMP1 = val; break;
  case 5: TCA0.SPLIT.HCMP2 = val; break;
  }
}
