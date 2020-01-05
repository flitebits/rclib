#ifndef SERIAL_H_
#define SERIAL_H_

#include "IntTypes.h"
#include "avr/io.h"

#ifndef NDEBUG
// If this is defined the serial classes will increment a counter for
// every error type encountered when reading (overflow, frame error
// and parity error).  This can be really ueseful for debugging but is
// generally not useful for the final build (each read can tell you if
// it had an error which is generally more useful for code) and adds
// time to the interrupt handler.
#define SERIAL_TRACK_ERRORS
#endif

struct ReadInfo {
  u8_t data;
  u8_t err;    // Uses *_ERR_BM bitmasks
  u16_t time;  // Time in 1/32768 of a second.
};

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

    BUFFER_SZ_BITS = 5,
    OVERFLOW_ERR_BM = USART_BUFOVF_bm,
    FRAME_ERR_BM = USART_FERR_bm,
    PARITY_ERR_BM = USART_PERR_bm,
  };
  
  // Disconnects
  void Disable();
  void Enable(u8_t mode);
  void Disable(u8_t mode);
  void FlushTx();  // returns once all bytes have been written.
  
  void Setup(long baud, u8_t data_bits, u8_t parity, u8_t stop_bits,
             bool invert=false, bool use_alt_pins=false,
             u8_t mode=MODE_TX_RX, bool use_pullup = false,
             bool buffered = false, bool use_2x_mode = false);
  void SetBuffered(bool buffered);
  // Returns true if at least one byte is avaialable to read.
  bool Avail();
  // Reads one byte, sets err to true if there was any issue (parity
  // or framing) reading the returned byte.
  u8_t Read();
  u8_t Read(u8_t* err);
  void Read(ReadInfo* info);
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
#ifdef __AVR_ATmega4809__
  static Serial usart3;
#endif

  void ReadInterrupt();  // Called from Interrupt, don't call directly.

protected:
  // protected used to construct the four usarts.
  Serial(u8_t usart_idx);
  u8_t ReadInternal(u8_t* err);  // Reads a byte from register
  
  u8_t idx_;
  USART_t* usart_;
  PORT_t* port_;
#ifdef SERIAL_TRACK_ERRORS
  u8_t overflow_;
  u8_t frame_err_;
  u8_t parity_err_;
#endif
  bool buffered_;
  u8_t widx_;
  u8_t ridx_;
  ReadInfo infos_[1 << BUFFER_SZ_BITS];
};

#endif /* SERIAL_H_ */
