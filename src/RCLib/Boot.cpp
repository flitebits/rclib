// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include "Boot.h"

#include <avr/io.h>
#include "Util.h"

namespace {
  void SetPinsLowPower(PORT_t *port) {
    port->PIN0CTRL |= 1 << PORT_PULLUPEN_bp;
    port->PIN1CTRL |= 1 << PORT_PULLUPEN_bp;
    port->PIN2CTRL |= 1 << PORT_PULLUPEN_bp;
    port->PIN3CTRL |= 1 << PORT_PULLUPEN_bp;
    port->PIN4CTRL |= 1 << PORT_PULLUPEN_bp;
    port->PIN5CTRL |= 1 << PORT_PULLUPEN_bp;
    port->PIN6CTRL |= 1 << PORT_PULLUPEN_bp;
    port->PIN7CTRL |= 1 << PORT_PULLUPEN_bp;
  }

  void SetChipPinsLowPower() {
    SetPinsLowPower(&PORTA);
    SetPinsLowPower(&PORTB);
    SetPinsLowPower(&PORTC);
    SetPinsLowPower(&PORTD);
    SetPinsLowPower(&PORTE);
    SetPinsLowPower(&PORTF);
  }

  void ConfigureClocks(u8_t target_pdiv, bool use_internal_32Kclk) {
    // Decide if we are using lower power/accuracy internal 32K clock
    // or an external 32K crystal on TOSC1 and TOSC2.
    if (use_internal_32Kclk) {
      write_ccp(&CLKCTRL.OSC32KCTRLA, 0 << CLKCTRL_RUNSTDBY_bp);
	  write_ccp(&CLKCTRL.XOSC32KCTRLA, 0);
    } else { // enable the external clock.
      write_ccp(&CLKCTRL.XOSC32KCTRLA,
	       CLKCTRL_CSUT_1K_gc |         // 1k cycles for start
	       (1 << CLKCTRL_ENABLE_bp) |   // enable the clock
	       (0 << CLKCTRL_RUNSTDBY_bp) | // don't run in standby
	       (0 << CLKCTRL_SEL_bp));      // using external crystal
    }

    // Don't run 20Mhz clock in Standby.
    write_ccp(&CLKCTRL.OSC20MCTRLA, 0 << CLKCTRL_RUNSTDBY_bp);

    if (target_pdiv == 1) {
      // Disable prescaler if target pdiv is 1.
      write_ccp(&CLKCTRL.MCLKCTRLB, 0 << CLKCTRL_PEN_bp);
    } else {
      u8_t pdiv;
      if      (target_pdiv <  4) pdiv = 0x0;
      else if (target_pdiv <  6) pdiv = 0x1;
      else if (target_pdiv <  8) pdiv = 0x8;
      else if (target_pdiv < 10) pdiv = 0x2;
      else if (target_pdiv < 12) pdiv = 0x9;
      else if (target_pdiv < 16) pdiv = 0xA;
      else if (target_pdiv < 24) pdiv = 0x3;
      else if (target_pdiv < 32) pdiv = 0xB;
      else if (target_pdiv < 48) pdiv = 0x4;
      else if (target_pdiv < 64) pdiv = 0xC;
      else  pdiv = 0x5;
      write_ccp(&CLKCTRL.MCLKCTRLB,
	       ((pdiv << CLKCTRL_PDIV_gp) |  // set pdiv value
		(1 << CLKCTRL_PEN_bp)));     // enable peripheral prescale.
    }
    write_ccp(&CLKCTRL.MCLKCTRLA,
	     (CLKCTRL_CLKSEL_OSC20M_gc |  /* 20Mhz oscillator for main clk*/
	      (0 << CLKCTRL_CLKOUT_bp))); /* don't output clock on a pin */
    // Don't lock the clock (although I don't plan to change it).
    write_ccp(&CLKCTRL.MCLKLOCK, 0 << CLKCTRL_LOCKEN_bp);
  }

  void ConfigureInterrupts() {
    write_ccp(&CPUINT.CTRLA,
	     ((0 << CPUINT_CVT_bp) |     // Don't use compact vector table
	      (0 << CPUINT_IVSEL_bp) |   // Interrupts at start of app flash
	      (0 << CPUINT_LVL0RR_bp))); // Use fixed prioriest, not round robin

    CPUINT.LVL0PRI = 0x0 << CPUINT_LVL0PRI_gp; // Use default priorities
    CPUINT.LVL1VEC = 0x0 << CPUINT_LVL1VEC_gp; // No high priority interrupt
  }
}  // namespace

void Boot(u8_t target_pdiv, bool use_internal_32Kclk) {
  // Turning on pullup enable for pins saves power.
  SetChipPinsLowPower();
  ConfigureClocks(target_pdiv, use_internal_32Kclk);
  ConfigureInterrupts();
}
