// Copyright 2024 Thomas DeWeese
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _VAR_CMDS_
#define _VAR_CMDS_

#include <stdio.h>
#include <string.h>

#include "Dbg.h"
#include "DbgCmds.h"
#include "IntTypes.h"

namespace dbg {

// VarRegistry is used to track global scope variables that you want to be able to manipulate via the debug serial port.
// Generally you instantiate and register with DbgCnds once early:
//  DBG_INIT(Serial::usart0, 115200);
//  DbgCmds cmds(&Serial::usart0);
//  VARCMDS_INIT(&cmds);


class VarRegistry {
public:
  static VarRegistry singleton;
  VarRegistry() : cnt_(0) {}

  void Register(const char* id, char* ptr, const char* scan,
                const char* print) {
    ptr_types p;
    p.s = ptr;
    Entry e = {id, p, scan, print, TYPE_STR};
    Add(&e);
  }
  void Register(const char* id, int* ptr, const char* scan, const char* print) {
    ptr_types p;
    p.d = ptr;
    Entry e = {id, p, scan, print, TYPE_INT};
    Add(&e);
  }
  void Register(const char* id, bool* ptr, const char* scan,
                const char* print) {
    ptr_types p;
    p.b = ptr;
    Entry e = {id, p, scan, print, TYPE_BOOL};
    Add(&e);
  }
  void RegisterChar(const char* id, u8_t* ptr, const char* scan,
                const char* print) {
    ptr_types p;
    p.c = ptr;
    Entry e = {id, p, scan, print, TYPE_CHAR};
    Add(&e);
  }

  virtual void HandleLine(const char* args) {
    char id[16];
    int len;
    int cnt = sscanf(args, "%15s %n", id, &len);
    if (cnt != 1) {
      DBG_LO(APP, ("Unable to scan id from str: '%s'\n", args));
      return;
    }
    args += len;
    for (int i = 0; i < cnt; ++i) {
      if (strcmp(id, entries_[i].id) == 0) {
        cnt = sscanf(args, entries_[i].scan, entries_[i].ptr.v);
        if (cnt != 1) {
          DBG_LO(APP, ("Unable to scan content for id '%s': '%s'\n",
                       id, args));
        }
        return;
      }
    }
    DBG_LO(APP, ("Unable to find var id: '%s'\n", id));
  }

  void PrintVar(const char* id) const {
    for (int i = 0; i < cnt_; ++i) {
      if (strcmp(id, entries_[i].id) == 0) {
        PrintEntry(entries_+ i);
        break;
      }
    }
  }
  void PrintEntries() const {
    for (int i = 0; i < cnt_; ++i) {
      PrintEntry(entries_ + i);
    }
  }

 protected:
  struct Entry;
  void PrintEntry(const VarRegistry::Entry* e) const {
    DBG_LO(APP, ("%s: ", e->id));
    switch (e->type) {
    case TYPE_STR: DBG_LO(APP, (e->print, e->ptr.s)); break;
    case TYPE_INT: DBG_LO(APP, (e->print, *e->ptr.d)); break;
    case TYPE_CHAR: DBG_LO(APP, (e->print, *e->ptr.c)); break;
    case TYPE_BOOL: DBG_LO(APP, (e->print, *e->ptr.b ? "True" : "False"));
      break;
    }
    DBG_LO(APP, ("\n"));
  }

  union ptr_types {
    void *v;
    char *s;
    int *d;
    bool* b;
    u8_t* c;
  };
  enum type_type {
        TYPE_STR, TYPE_INT, TYPE_BOOL, TYPE_CHAR
  };
  struct Entry {
    const char* id;
    ptr_types ptr;
    const char* scan;
    const char* print;
    type_type type;
  };

  void Add(VarRegistry::Entry* item) {
    if (cnt_ >= 32) {
      DBG_LO(APP, ("Too many vars registered: %s", item->id));
      return;
    }
    entries_[cnt_++] = *item;
  }

  int cnt_;
  Entry entries_[32];
};

 // defines 'sv' that sets the value of a var from 'args'.
class SetVarCmd : public CmdHandler {
public:
  SetVarCmd() : CmdHandler("sv") { }
  virtual void HandleLine(const char* args) {
    VarRegistry::singleton.HandleLine(args);
  }
  static SetVarCmd singleton;
};

// defines 'dv' that dumps the values of all know vars
class DumpVarCmd : public CmdHandler {
public:
  DumpVarCmd() : CmdHandler("dv") { }
  virtual void HandleLine(const char* args) {
    if (*args == 0) {
      VarRegistry::singleton.PrintEntries();
    } else {
      char id[16];
      if (sscanf(args, "%s", id) == 1) {
        VarRegistry::singleton.PrintVar(id);
      }
    }
  }
  static DumpVarCmd singleton;
};

#define VARCMDS_INIT(dbg_cmd) do {                        \
    dbg_cmd.RegisterHandler(&dbg::SetVarCmd::singleton);  \
    dbg_cmd.RegisterHandler(&dbg::DumpVarCmd::singleton); \
  } while (false)

#define DEFVAR_INT(name, init) int name = (init); do { \
    dbg::VarRegistry::singleton.Register(#name, &name, "%d", "%d");     \
} while(false)
#define REFVAR_INT(name, init) \
  dbg::VarRegistry::singleton.Register(#name, &name, "%d", "%d");       \

#define DEFVAR_STR(name, sz, init) char name[sz] = (init); do { \
    dbg::VarRegistry::singleton.Register(#name, name, "%s", "%s");      \
} while(false)
#define REFVAR_STR(name) \
  dbg::VarRegistry::singleton.Register(#name, name, "%s", "%s");

#define DEFVAR_BOOL(name, init) bool name = (init); do {       \
    dbg::VarRegistry::singleton.Register(#name, &name, "%s", "%s");     \
} while(false)
#define REFVAR_BOOL(name) \
  dbg::VarRegistry::singleton.Register(#name, &name, "%s", "%s");

#define DEFVAR_CHAR(name, init) u8_t name = (init); do {       \
  VarRegistry::singleton.Register(#name, &name, "%c", "%c"); \
} while(false)
#define REFVAR_CHAR(name) \
  VarRegistry::singleton.Register(#name, &name, "%c", "%c");

#define DEFVAR_U8T(name, init) u8_t name = (init); do {     \
  VarRegistry::singleton.Register(#name, &name, "%d", "%d"); \
} while(false)
#define REFVAR_U8T(name) \
  VarRegistry::singleton.Register(#name, &name, "%d", "%d");

}  // namespace dbg

#endif  // _VAR_CMDS_
