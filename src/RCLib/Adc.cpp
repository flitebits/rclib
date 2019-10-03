#include "Adc.h"

#include <avr/io.h>
#include "util.h"

namespace {
  const long max_pclk = 1000000; // Recommened max sample rate for ADC.
}

Adc::Adc(){
  i8_t p_scale = 0;
  long pclk = GetPerClock() >> 1;  // min div is 2x
  while (pclk > max_pclk) {
    p_scale++;
    pclk = pclk >> 1;
  }
  
  ADC0.CTRLA = 0;  // 10Bit, not-enabled yet.
  ADC0.CTRLB = ADC_SAMPNUM_ACC1_gc;  // No multi-sample
  ADC0.CTRLC = ((1 << ADC_SAMPCAP_bp)  |  // Reduce Sampling capacitance
                (ADC_REFSEL_VREFA_gc) |  // Use VDD for reference
		p_scale);
  ADC0.CTRLD = 0;  // No added Sample delays.                
  ADC0.CTRLE = 0;  // No windowed mode.
  // No added sample cycles (needed for high impedence signals).
  ADC0.SAMPCTRL = 0;  
  ADC0.CTRLA = ADC0.CTRLA | (1 << ADC_ENABLE_bp);  // Enable ADC.
}

void Adc::Disable() {
  ADC0.CTRLA = ADC0.CTRLA & ~(1 << ADC_ENABLE_bp);
}
void Adc::Enable() {
  ADC0.CTRLA = ADC0.CTRLA | (1 << ADC_ENABLE_bp);
}

void Adc::StartRead(i8_t ain_idx) {
  ADC0.MUXPOS = ain_idx;
  while ((ADC0.COMMAND & (1 << ADC_STCONV_bp)) != 0);
  ADC0.COMMAND = 1 << ADC_STCONV_bp;  // Start ADC conversion
}
int Adc::FinishRead() {
  while ((ADC0.INTFLAGS & (1 << ADC_RESRDY_bp)) == 0);  // Wait for Ready
  return ADC0.RES;
}

int Adc::ConinueRead() {
  int result = FinishRead();
  ADC0.COMMAND = 1 << ADC_STCONV_bp;  // Start ADC conversion
  return result;
}

void Adc::ConfigurePin(PinId pin) {
  struct PORT_struct* port = pin.port_ptr();
  port->DIRCLR = 1 << pin.pin();  // Make the pin input only
  *((&port->PIN0CTRL) + pin.pin()) = 0; // Disable everything.
}
