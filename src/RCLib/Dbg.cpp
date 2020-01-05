// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#include "Dbg.h"
#include <stdio.h>
#ifndef NDEBUG
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
#endif  // NDEBUG
