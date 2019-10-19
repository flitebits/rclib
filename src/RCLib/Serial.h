#ifndef SERIAL_H_
#define SERIAL_H_

#include "IntTypes.h"
#include "avr/io.h"

class Serial {
  public:
  enum {
    PARITY_NONE = 0,
    PARITY_ODD = 1,
    PARITY_EVEN = 2,

    MODE_TX    = 1 << 0,
    MODE_RX    = 1 << 1,
    MODE_1WIRE = 1 << 3,
    MODE_TX_RX = MODE_TX | MODE_RX,
    MODE_TX_RX_2WIRE = MODE_TX | MODE_RX,
    MODE_TX_RX_1WIRE = MODE_TX | MODE_RX | MODE_1WIRE,
    MODE_RX_TX = MODE_TX | MODE_RX,
    MODE_RX_TX_2WIRE = MODE_TX | MODE_RX,
    MODE_RX_TX_1WIRE = MODE_TX | MODE_RX | MODE_1WIRE,
  };
  // Disconnects
  void Disable();
  void Enable(u8_t mode);
  void Disable(u8_t mode);
  void FlushTx();  // returns once all bytes have been written.
  
  void Setup(long baud, u8_t data_bits, u8_t parity, u8_t stop_bits,
	     bool invert=false, bool use_alt_pins=false,
	     u8_t mode=MODE_TX_RX, bool use_pullup = false, bool use_2x_mode = false);
  // Returns true if at least one byte is avaialable to read.
  bool Avail();
  // Reads one byte, sets err to true if there was any issue (parity
  // or framing) reading the returned byte.
  u8_t Read(bool* err);
  // Write one byte.
  void WriteByte(const char val);
  // Write null terminated string.
  void Write(const char* str);
  // Write a null terminated string, followed by a newline.
  void WriteLn(const char* str) { Write(str); WriteByte('\n'); }
  // Write an integer with the given base.
  void WriteBase(int num, u8_t base);
  // Write an integer base 10
  void Write(int num) { WriteBase(num, 10); }
  // Write an integer base 10, follwed by a newline
  void WriteLn(int num) { WriteBase(num, 10); WriteByte('\n'); }
  // Write an integer base 16 (hex)
  void WriteHex(int num) { WriteBase(num, 16); }
  // Write an integer base 16 (hex), follwed by a newline
  void WriteHexLn(int num) { WriteBase(num, 16); WriteByte('\n'); }

  // The 4 usarts on a 4809
  static Serial usart0;
  static Serial usart1;
  static Serial usart2;
  static Serial usart3;

protected:
  // protected used to construct the four usarts.
  Serial(u8_t usart_idx);

  u8_t idx_;
  USART_t* usart_;
  PORT_t* port_;
  int overflow_;
  int frame_err_;
  int parity_err_;
};



#endif /* SERIAL_H_ */
