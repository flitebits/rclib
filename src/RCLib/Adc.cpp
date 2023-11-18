// Copyright 2020 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "Adc.h"

#include <avr/io.h>
#include "util.h"

namespace {
  const long max_pclk = 1000000; // Recomened max sample rate for ADC.
}

Adc::Adc(VRefSrc src){
  i8_t p_scale = 0;
  long pclk = GetPerClock() >> 1;  // min div is 2x
  while (pclk > max_pclk) {
    p_scale++;
    pclk = pclk >> 1;
  }

  u8_t ref_sel = ADC_REFSEL_INTREF_gc;
  switch (src) {
  case VREF_VDD: ref_sel = ADC_REFSEL_VDDREF_gc; break;
  case VREF_VREFA: ref_sel = ADC_REFSEL_VREFA_gc; break;
  case VREF_055:
    VREF.CTRLA = (VREF.CTRLA & ~VREF_ADC0REFSEL_gm) | VREF_ADC0REFSEL_0V55_gc;
    break;
  case VREF_11:
    VREF.CTRLA = (VREF.CTRLA & ~VREF_ADC0REFSEL_gm) | VREF_ADC0REFSEL_1V1_gc;
    break;
  case VREF_15:
    VREF.CTRLA = (VREF.CTRLA & ~VREF_ADC0REFSEL_gm) | VREF_ADC0REFSEL_1V5_gc;
    break;
  case VREF_25:
    VREF.CTRLA = (VREF.CTRLA & ~VREF_ADC0REFSEL_gm) | VREF_ADC0REFSEL_2V5_gc;
    break;
  case VREF_43:
    VREF.CTRLA = (VREF.CTRLA & ~VREF_ADC0REFSEL_gm) | VREF_ADC0REFSEL_4V34_gc;
    break;
  }

  ADC0.CTRLA = 0;  // 10Bit, not-enabled yet.
  ADC0.CTRLB = ADC_SAMPNUM_ACC1_gc;  // No multi-sample
  ADC0.CTRLC = ((1 << ADC_SAMPCAP_bp)  |  // Reduce Sampling capacitance
                ref_sel |  // select Voltage reference
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

int Adc::ContinueRead() {
  int result = FinishRead();
  ADC0.COMMAND = 1 << ADC_STCONV_bp;  // Start ADC conversion
  return result;
}

void Adc::ConfigurePin(PinId pin) {
  struct PORT_struct* port = pin.port_ptr();
  port->DIRCLR = 1 << pin.pin();  // Make the pin input only
  *((&port->PIN0CTRL) + pin.pin()) = 0; // Disable everything.
}
