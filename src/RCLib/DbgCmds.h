// Copyright 2024 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _DBG_CMDS_
#define _DBG_CMDS_

#include <string.h>
#include "Dbg.h"
#include "IntTypes.h"
#include "Serial.h"

namespace dbg {

// Handler for command 'cmd'.
class CmdHandler {
public:
  CmdHandler(const char* cmd)
    : cmd_len_(strlen(cmd)), cmd_str_(cmd) { }

  // If line starts with cmd_str_ then call HandleLine
  // to implement actual command handling.
  bool CheckLine(const char* line) {
    if (strncmp(cmd_str_, line, cmd_len_)) return false;
    HandleLine(line + cmd_len_);
    return true;
  }

  protected:
  // The rest of the command line after 'cmd_str_'
  // can be parsed to decide on exact action taken.
  virtual void HandleLine(const char* args) = 0;
  const u8_t cmd_len_;
  const char* cmd_str_;
};

// Typically defined in main and called moderately frequently in your processing
// loop to process commands given on the debug serial port. Typically uses the
// same serial port as Dbg printing, but not reqired.
//
//  DBG_INIT(Serial::usart0, 115200);
//  DbgCmds cmds(&Serial::usart0);
//  // Register/define handlers...
// ...
//  while (true) {
//    cmds.Run();
//

class DbgCmds {
public:
  static const u8_t kMaxHandlers = 16;

  explicit DbgCmds(Serial* dbg_serial)
    : dbg_serial_(dbg_serial), num_handlers_(0),
      bad_data_(false), data_pos_(0) {
    dbg_serial_->SetBuffered(true);
  }

  void RegisterHandler(CmdHandler* handler) {
    if (num_handlers_ == kMaxHandlers) {
      DBG_LO(DBG, ("Too many DbgCmd handlers registered only supports %d.",
                   kMaxHandlers));
      return;
    }
    handlers_[num_handlers_++] = handler;
  }

  void Run() {
    if (!dbg_serial_->Avail()) return;

    u8_t rd_err;
    char ch = dbg_serial_->Read(&rd_err);
    if (rd_err) {
      bad_data_ = true;
      return;
    }
    if (ch != '\r' && ch != '\n') {  // end of line
      read_data_[data_pos_] = ch;
      if (++data_pos_ > sizeof(read_data_)) {
        data_pos_ = 0;
        bad_data_ = true;
      }
      return;
    }
    if (data_pos_ == 0) return;
    if (!bad_data_) {
      read_data_[data_pos_] = 0;
      ProcessCmd(read_data_);
    }
    data_pos_ = 0;
    bad_data_ = false;
  }

  void ProcessCmd(char* line) {
    DBG_LO(DBG, ("Line: '%s'\n", line));
    bool matched = false;
    for (u8_t i = 0; i < num_handlers_; ++i) {
      if (handlers_[i]->CheckLine(line)) {
        matched = true;
        break;
      }
    }
    if (!matched) {
      DBG_LO(DBG, ("No handler matched line"));
    }
  }

private:
  Serial* dbg_serial_;
  u8_t num_handlers_;
  CmdHandler* handlers_[kMaxHandlers];

  u8_t bad_data_;
  u8_t data_pos_;
  char read_data_[64];
};

}  // namespace dbg

#endif  // _DBG_CMDS_
