// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#ifdef CFG_backtrace

// append 14-bit value to ring buffer
void backtrace_val (int line, unsigned short val);

// append 16-bit address to ring buffer
void backtrace_addr (int line, unsigned short addr);

// append program counter to ring buffer
void backtrace_pc (int line);

// save trace to EEPROM
void backtrace_save (void);

// get pointer and length of saved trace from EEPROM, return if trace is new/unread
bool backtrace_get (unsigned char** buf, unsigned int* len);

// invalidate saved trace in EEPROM
void backtrace_invalidate (void);

#define BACKTRACE()   backtrace_pc(__LINE__)
#define TRACE_VAL(v)  backtrace_val(__LINE__, (v))
#define TRACE_ADDR(a) backtrace_addr(__LINE__, (a))

#else

#define BACKTRACE()
#define TRACE_VAL(v)
#define TRACE_ADDR(a)

#endif
