#include "Dbg.h"
#include <stdio.h>

namespace dbg {
namespace {
  int dbg_putchar(char c, FILE* stream) {
    Dbg::dbg.PutChar(c);
    return 0;
  }
  FILE dbg_fp;
}  // anonymous namespace

Dbg Dbg::dbg;

Dbg::Dbg() : port_(NULL) { }

void Dbg::Setup(Serial* port) {
  port_ = port;
  fdev_setup_stream(&dbg_fp, dbg_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &dbg_fp;
}

}  // namespace dbg
