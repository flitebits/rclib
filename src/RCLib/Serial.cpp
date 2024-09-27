// Copyright 2020 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "Serial.h"

#include <stddef.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "Dbg.h"
#include "RtcTime.h"
#include "util.h"

namespace {
  const char kNumCh[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8',
                         '9', 'A', 'B', 'C', 'D', 'E', 'F'};
}

Serial Serial::usart0(0);
ISR(USART0_RXC_vect) {
  Serial::usart0.ReadInterrupt();
}

Serial Serial::usart1(1);
ISR(USART1_RXC_vect) {
  Serial::usart1.ReadInterrupt();
}

Serial Serial::usart2(2);
ISR(USART2_RXC_vect) {
  Serial::usart2.ReadInterrupt();
}

#ifdef __AVR_ATmega4809__
Serial Serial::usart3(3);
ISR(USART3_RXC_vect) {
  Serial::usart3.ReadInterrupt();
}
#endif

Serial::Serial(u8_t idx)
  : idx_(idx)
#ifdef SERIAL_TRACK_ERRORS
  ,overflow_(0), frame_err_(0), parity_err_(0)
#endif
{
  switch (idx_) {
    case 0: usart_ = &USART0; port_ = &PORTA; break;
    case 1: usart_ = &USART1; port_ = &PORTC; break;
    case 2: usart_ = &USART2; port_ = &PORTF; break;
    case 3: usart_ = &USART3; port_ = &PORTB; break;
  }
}
void Serial::Setup(long baud, u8_t data_bits, u8_t parity, u8_t stop_bits,
                   bool invert, bool use_alt_pins, u8_t mode, bool use_pullup,
                   bool buffered, bool use_2x_mode) {
  buffered_ = buffered;
  PORTMUX.USARTROUTEA = (PORTMUX.USARTROUTEA & ~(0b11 << (2 * idx_))) |
    ((use_alt_pins ? 0b01 : 0) << (2 * idx_));
  bool tx_enable = mode & MODE_TX;
  bool rx_enable = mode & MODE_RX;
  bool one_wire = mode & MODE_1WIRE;

  if (tx_enable || one_wire) {
    port_->OUTSET = 1 << (use_alt_pins ? 4 : 0);  // Tx0 high
    port_->DIRSET = 1 << (use_alt_pins ? 4 : 0);  // Tx0 output pin
    const u8_t pin_val = (((invert     ? 1 : 0) << PORT_INVEN_bp) |  // Inverted Serial
                          ((use_pullup ? 1 : 0) << PORT_PULLUPEN_bp));
    if (use_alt_pins) port_->PIN4CTRL = pin_val;
    else              port_->PIN0CTRL = pin_val;
  }
  if (rx_enable && !one_wire) {  // In one wire mode no RX pin.
    port_->DIRCLR = 1 << (use_alt_pins ? 5 : 1);  // Rx0 input pin
    const u8_t pin_val = (((invert     ? 1 : 0) << PORT_INVEN_bp) |  // Inverted Serial
                          ((use_pullup ? 1 : 0) << PORT_PULLUPEN_bp));
    if (use_alt_pins) port_->PIN5CTRL = pin_val;
    else              port_->PIN1CTRL = pin_val;
  }

  // Extract base clock frequencies to calculate baud rate setting.
  long base_clk = GetMainClock();
  u8_t pdiv = GetPerClockScale();
  // normal calculation is 64 * per_clck / <samples> where samples is
  // 8 or 16.  Here we shift up base_clock by the remainder of 64/8 =
  // 8 -> 3 bits, or 64 / 16 = 4 -> 2 bits;
  base_clk = base_clk << (use_2x_mode ? 3 : 2);
  i32_t clk_divisor = pdiv * baud;
  // Now divide base_clk by pdiv * baud rate to get base baud clock rate
  i32_t baud_clk = (base_clk + (clk_divisor >> 1)) / clk_divisor;
  // Now scale the base baud clk by the 5V error in the main oscillator.
  usart_->BAUD = baud_clk;

  u8_t ctrlc_val = USART_CMODE_ASYNCHRONOUS_gc;
  switch (parity) {
  case PARITY_EVEN: ctrlc_val |= USART_PMODE_EVEN_gc; break;
  case PARITY_ODD: ctrlc_val |= USART_PMODE_ODD_gc; break;
  default:
  case PARITY_NONE: ctrlc_val |= USART_PMODE_DISABLED_gc; break;
  }
  switch (stop_bits) {
  case 2: ctrlc_val |= USART_SBMODE_2BIT_gc; break;
  default:
  case 1: ctrlc_val |= USART_SBMODE_1BIT_gc; break;
  }
  switch (data_bits) {
  case 5: ctrlc_val |= USART_CHSIZE_5BIT_gc; break;
  case 6: ctrlc_val |= USART_CHSIZE_6BIT_gc; break;
  case 7: ctrlc_val |= USART_CHSIZE_7BIT_gc; break;
  default:
  case 8: ctrlc_val |= USART_CHSIZE_8BIT_gc; break;
  }

  // Loop back mode enable (read/write to same pin)
  usart_->CTRLA = ((one_wire ? USART_LBME_bm : 0) |
                   // Buffered mode, interrupt reads into circular buffer
                   (buffered ? USART_RXCIF_bm : 0));
  usart_->CTRLB =
    (((rx_enable ? 1 : 0) << USART_RXEN_bp) |  // En/Disable Receive pin
     ((tx_enable ? 1 : 0) << USART_TXEN_bp) |  // En/Disable Transmit
     (0 << USART_SFDEN_bp) |  // No Wake on Start Frame)
     ((one_wire ? 1 : 0) << USART_ODME_bp) |  // Open Drain for 1 wire
     // Not 2x speed
     (use_2x_mode ? USART_RXMODE_CLK2X_gc : USART_RXMODE_NORMAL_gc));
  usart_->CTRLC = ctrlc_val;
}

void Serial::Disable() {
  PORTMUX.USARTROUTEA = ((PORTMUX.USARTROUTEA & ~(0b11 << (2 * idx_))) |
                         (0b11 << (2 * idx_)));
}

void Serial::Enable(u8_t mode) {
  if (mode & MODE_RX) {
    usart_->CTRLB |= (1 << USART_RXEN_bp);
  }
  if (mode & MODE_TX) {
    usart_->CTRLB |=  (1 << USART_TXEN_bp);
  }
}
void Serial::Disable(u8_t mode) {
  if (mode & MODE_RX) {
    usart_->CTRLB &= ~USART_RXEN_bm;
  }
  if (mode & MODE_TX) {
    usart_->CTRLB &= ~USART_TXEN_bm;
  }
}

void Serial::SetBuffered(bool buffered) {
  if (buffered_ == buffered) return;
  buffered_ = buffered;
  widx_ = ridx_ = 0;
  usart_->CTRLA = buffered ?
    (usart_->CTRLA | USART_RXCIF_bm) :
    (usart_->CTRLA & ~USART_RXCIF_bm);
}

void Serial::ReadInterrupt() {
  ReadInfo* info = &infos_[widx_];
  info->data = ReadInternal(&info->err);
  info->time = RTC.CNT;

  u8_t next_widx = (widx_ + 1) & ((1 << BUFFER_SZ_BITS) - 1);
  if (next_widx == ridx_) {
    info->err |= OVERFLOW_ERR_BM;
#ifdef SERIAL_TRACK_ERRORS
    ++overflow_;
#endif
  } else {
    widx_ = next_widx;
  }
}

void Serial::FlushTx() {
  // wait for send buffer to be empty.
  while((usart_->STATUS & USART_DREIE_bm) == 0);
  // Wait for Transmit to complete.
  while ((usart_->STATUS & USART_TXCIF_bm) == 0);
}

bool Serial::Avail() {
  if (!buffered_) {
    return ((usart_->STATUS & USART_RXCIF_bm) != 0);
  }
  bool result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          result = ridx_ != widx_;
  }
  return result;
}

u8_t Serial::Read() {
  if (!buffered_) {
    return ReadInternal(NULL);
  }
  u8_t result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    result = infos_[ridx_].data;
    if (ridx_ != widx_) {
      ridx_ = (ridx_ + 1) & ((1 << BUFFER_SZ_BITS) - 1);
    }
  }
  return result;
}

u8_t Serial::Read(u8_t* err_ptr) {
  if (!buffered_) {
    return ReadInternal(err_ptr);
  }

  ReadInfo info;
  Read(&info);
  if (err_ptr) *err_ptr = info.err;
  return info.data;
}

void Serial::Read(ReadInfo* info_ptr) {
  if (!buffered_) {
    info_ptr->data = ReadInternal(&info_ptr->err);
    info_ptr->time = RTC.CNT;
    return;
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    *info_ptr = infos_[ridx_];
    if (ridx_ != widx_) {
      ridx_ = (ridx_ + 1) & ((1 << BUFFER_SZ_BITS) - 1);
    }
  }
}

u8_t Serial::ReadInternal(u8_t* err_ptr) {
  u8_t rx_hi = usart_->RXDATAH;
  u8_t rx_lo = usart_->RXDATAL;
#ifdef SERIAL_TRACK_ERRORS
  if ((rx_hi & USART_BUFOVF_bm) != 0) {
    ++overflow_;
  }
  if ((rx_hi & USART_FERR_bm) != 0) {
    ++frame_err_;
  }
  if ((rx_hi & USART_PERR_bm) != 0) {
    ++parity_err_;
  }
#endif
  if (err_ptr != NULL) {
    *err_ptr = rx_hi & (USART_BUFOVF_bm | USART_FERR_bm | USART_PERR_bm);
  }
  return rx_lo;
}

void Serial::DumpReadBuffer() {
  DBG_LO(SBUS, ("Widx: %d Ridx: %d\n", widx_, ridx_));
  for (int i = 0; i < (1 << BUFFER_SZ_BITS); ++i) {
    DBG_LO(SBUS, ("%cEnt[%d]: D:%02X E:%02X T:%04X\n",
                  (i == widx_) ? ((i == ridx_) ? '*' : 'w') :
                  ((i == ridx_) ? 'r' : ' '), i, infos_[i].data, infos_[i].err,
                  infos_[i].time));
  }
}

void Serial::WriteByte(const char ch) {
  while((usart_->STATUS & USART_DREIE_bm) == 0);  // wait for send buffer to be empty.
  usart_->STATUS = USART_TXCIF_bm;  // Clear transmit complete flag.
  usart_->TXDATAL = ch;
}

void Serial::Write(const char* str) {
  while (*str != 0) {
    WriteByte(*(str++));
  }
}

void Serial::WriteBase(int num, u8_t base) {
   if (num == 0) {
     WriteByte('0');
     return;
   }
   u16_t unum = num;
   if (num < 0) {
     WriteByte('-');
     unum = (~unum) + u16_t(1);
   }
   if (base > 16) base = 16;
   else if (base < 2) base = 2;
   char buf[5];
   int idx = 0;
   while(unum != 0) {
     buf[idx++] = kNumCh[unum % base];
     unum = unum / base;
   }
   while (idx) {
     WriteByte(buf[--idx]);
   }
}
