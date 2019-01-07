// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

//! \file
#ifndef _oslmic_h_
#define _oslmic_h_

// Dependencies required for the LoRa MAC in C to run.
// These settings can be adapted to the underlying system.
// You should not, however, change the lmic.[hc]



//================================================================================
//================================================================================
// Target platform as C library
#include <stdbool.h>
#include <stdint.h>
typedef uint8_t		bit_t;
typedef uint8_t		u1_t;
typedef int8_t		s1_t;
typedef uint16_t	u2_t;
typedef int16_t		s2_t;
typedef uint32_t	u4_t;
typedef int32_t		s4_t;
typedef uint64_t	u8_t;
typedef int64_t		s8_t;
typedef unsigned int	uint;
typedef const char*	str_t;

#include <string.h>
#include "hal.h"
#if !defined(CFG_simul)
#include "debug.h"
#endif
#define EV(a,b,c) /**/
#define DO_DEVDB(field1,field2) /**/
#if !defined(CFG_noassert)
#if defined(CFG_simul) && defined(CFG_DEBUG)
#include <stdio.h>
#define ASSERT(cond) do { \
    if(!(cond)) { fprintf(stderr, "ASSERTION FAILED: %s at %s:%d\n", \
			  #cond, __FILE__, __LINE__); hal_failed(); } } while (0)
#elif defined(CFG_DEBUG)
#define ASSERT(cond) do { if(!(cond)) { debug_printf("%s:%d: assertion failed\r\n", __FILE__, __LINE__); hal_failed(); } } while (0)
#else
#define ASSERT(cond) do { if(!(cond)) hal_failed(); } while (0)
#endif
#else
#define ASSERT(cond) do { } while (0)
#endif

#define os_minmax(vmin,v,vmax) ((vmin)>(v)?(vmin):(vmax)<(v)?(vmax):(v))
#define os_clearMem(a,b)   memset(a,0,b)
#define os_copyMem(a,b,c)  memcpy(a,b,c)
#define os_moveMem(a,b,c)  memmove(a,b,c)

typedef     struct osjob_t osjob_t;
typedef      struct band_t band_t;
typedef   struct chnldef_t chnldef_t;
typedef   struct rxsched_t rxsched_t;
typedef   struct bcninfo_t bcninfo_t;
typedef        const u1_t* xref2cu1_t;
typedef              u1_t* xref2u1_t;
#define TYPEDEF_xref2rps_t     typedef         rps_t* xref2rps_t
#define TYPEDEF_xref2rxsched_t typedef     rxsched_t* xref2rxsched_t
#define TYPEDEF_xref2chnldef_t typedef     chnldef_t* xref2chnldef_t
#define TYPEDEF_xref2band_t    typedef        band_t* xref2band_t
#define TYPEDEF_xref2osjob_t   typedef       osjob_t* xref2osjob_t

#define SIZEOFEXPR(x) sizeof(x)

#define ON_LMIC_EVENT(ev)  onLmicEvent(ev)
#define DECL_ON_LMIC_EVENT void onLmicEvent(ev_t e)

#define ON_BUDHA_EVENT(ev)  onBudhaEvent(ev)
#define DECL_ON_BUDHA_EVENT void onBudhaEvent(ev_t e)

#if defined(CFG_bootloader) && defined(CFG_bootloader_aes)
extern uint32_t (*AESFUNC) (uint8_t mode, uint8_t* buf, uint16_t len, uint32_t* key, uint32_t* aux);
#endif
extern u4_t AESAUX[];
extern u4_t AESKEY[];
#define AESkey ((u1_t*)AESKEY)
#define AESaux ((u1_t*)AESAUX)
#define FUNC_ADDR(func) (&(func))

u1_t radio_rand1 (void);
#define os_getRndU1() radio_rand1()

#if defined(CFG_simul)
#define DEFINE_LMIC
#define DECLARE_LMIC extern struct lmic_t* plmic
#define LMIC (*(plmic))
#else
#define DEFINE_LMIC  struct lmic_t LMIC
#define DECLARE_LMIC extern struct lmic_t LMIC
#endif

#if defined(CFG_budha)
#if defined(CFG_simul)
#define DEFINE_BUDHA
#define DECLARE_BUDHA extern struct budha_t* pbudha
#define BUDHA (*(pbudha))
#else
#define DEFINE_BUDHA  struct lmic_t BUDHA
#define DECLARE_BUDHA extern struct budha_t BUDHA
#endif
#endif //defined(CFG_budha)

#if defined(CFG_simul)
#define DEFINE_APP(t)
#define DECLARE_APP(t) extern void* pappdata
#define APP(t) (*(struct t*)pappdata)
#else
#define DEFINE_APP(t)  struct t APPDATA
#define DECLARE_APP(t) extern struct t APPDATA
#define APP(t) (APPDATA)
#endif

#define LOGCHECK(lvl,block) do {		\
	if( lvl <= log_lvl ) {			\
	    block;				\
	} 					\
    } while(0)
#if defined(CFG_simul)
extern int log_lvl;
void LOGIT(int lvl, char* fmt, ...);
#else
#define LOGIT(lvl, fmt, ...) debug_printf(fmt, ## __VA_ARGS__)
#endif

void radio_init (void);
void radio_reset (void);
void radio_writeBuf (u1_t addr, xref2u1_t buf, u1_t len);
void radio_readBuf (u1_t addr, xref2u1_t buf, u1_t len);
void radio_irq_handler (u1_t dio);
void os_init (void* bootarg);
void os_runstep (void);
void os_runloop (void);

//================================================================================


#ifndef RX_RAMPUP
#ifndef CFG_rxrampup
#define RX_RAMPUP  (us2osticks(1200))
#else
#define RX_RAMPUP  (us2osticksCeil(CFG_rxrampup))
#endif
#endif
#ifndef TX_RAMPUP
#ifndef CFG_txrampup
#define TX_RAMPUP  (us2osticks(2000))
#else
#define TX_RAMPUP  (us2osticksCeil(CFG_txrampup))
#endif
#endif

#ifndef OSTICKS_PER_SEC
#define OSTICKS_PER_SEC 32768
#elif OSTICKS_PER_SEC < 10000 || OSTICKS_PER_SEC > 64516
#error Illegal OSTICKS_PER_SEC - must be in range [10000:64516]. One tick must be 15.5us .. 100us long.
#endif

typedef s4_t  ostime_t;
typedef s8_t  osxtime_t;

#if !HAS_ostick_conv
#define us2osticks(us)   ((ostime_t)( ((s8_t)(us) * OSTICKS_PER_SEC) / 1000000))
#define ms2osticks(ms)   ((ostime_t)( ((s8_t)(ms) * OSTICKS_PER_SEC)    / 1000))
#define sec2osticks(sec) ((ostime_t)( (s8_t)(sec) * OSTICKS_PER_SEC))
#define osticks2sec(os)  ((s4_t)(((os)               ) / OSTICKS_PER_SEC))
#define osticks2ms(os)   ((s4_t)(((os)*(s8_t)1000    ) / OSTICKS_PER_SEC))
#define osticks2us(os)   ((s4_t)(((os)*(s8_t)1000000 ) / OSTICKS_PER_SEC))
// Special versions
#define us2osticksCeil(us)  ((ostime_t)( ((s8_t)(us) * OSTICKS_PER_SEC + 999999) / 1000000))
#define us2osticksRound(us) ((ostime_t)( ((s8_t)(us) * OSTICKS_PER_SEC + 500000) / 1000000))
#define ms2osticksCeil(ms)  ((ostime_t)( ((s8_t)(ms) * OSTICKS_PER_SEC + 999) / 1000))
#define ms2osticksRound(ms) ((ostime_t)( ((s8_t)(ms) * OSTICKS_PER_SEC + 500) / 1000))
// Extended versions
#define us2osxticks(us)   ((osxtime_t)( ((s8_t)(us) * OSTICKS_PER_SEC) / 1000000))
#define ms2osxticks(ms)   ((osxtime_t)( ((s8_t)(ms) * OSTICKS_PER_SEC)    / 1000))
#define sec2osxticks(sec) ((osxtime_t)( (s8_t)(sec) * OSTICKS_PER_SEC))
#endif


struct osjob_t;  // fwd decl.
typedef void (*osjobcb_t) (struct osjob_t*);
struct osjob_t {
    struct osjob_t* next;
    ostime_t deadline;
    osjobcb_t  func;
    unsigned int flags;
#if defined(CFG_simul)
    void* ctx;
    int pqidx;
#endif
};
TYPEDEF_xref2osjob_t;


#ifndef HAS_os_calls

#if defined(CFG_lorawan11)
#ifndef os_getNwkKey
void os_getNwkKey (xref2u1_t buf);
#endif
#ifndef os_getAppKey
void os_getAppKey (xref2u1_t buf);
#endif
#ifndef os_getJoinEui
void os_getJoinEui (xref2u1_t buf);
#endif
#else
#ifndef os_getDevKey
void os_getDevKey (xref2u1_t buf);
#endif
#ifndef os_getArtEui
void os_getArtEui (xref2u1_t buf);
#endif
#define os_getNwkKey(buf) os_getDevKey(buf)
#define os_getJoinEui(buf) os_getArtEui(buf)
#endif
#ifndef os_getDevEui
void os_getDevEui (xref2u1_t buf);
#endif
#ifndef os_setCallback
void os_setCallback (xref2osjob_t job, osjobcb_t cb);
#endif
#ifndef os_setTimedCallback
void os_setTimedCallback (xref2osjob_t job, ostime_t time, osjobcb_t cb);
#endif
#ifndef os_setApproxTimedCallback
void os_setApproxTimedCallback (xref2osjob_t job, ostime_t time, osjobcb_t cb);
#endif
#ifndef os_clearCallback
int os_clearCallback (xref2osjob_t job);
#endif
#ifndef os_getTime
ostime_t os_getTime (void);
#endif
#ifndef os_getXTime
osxtime_t os_getXTime (void);
#endif
#ifndef os_time2XTime
osxtime_t os_time2XTime (ostime_t t, osxtime_t context);
#endif
#ifndef os_getTimeSecs
uint os_getTimeSecs (void);
#endif
#ifndef os_radio
void os_radio (u1_t mode);
#endif
#ifndef os_cca
bit_t os_cca (u2_t rps, u4_t freq);
#endif
#ifndef os_getBattLevel
u1_t os_getBattLevel (void);
#endif
#ifndef os_getRegion
u4_t os_getRegion (void);
#endif

#ifndef os_rlsbf4
//! Read 32-bit quantity from given pointer in little endian byte order.
u4_t os_rlsbf4 (xref2cu1_t buf);
#endif
#ifndef os_wlsbf4
//! Write 32-bit quntity into buffer in little endian byte order.
void os_wlsbf4 (xref2u1_t buf, u4_t value);
#endif
#ifndef os_rmsbf4
//! Read 32-bit quantity from given pointer in big endian byte order.
u4_t os_rmsbf4 (xref2cu1_t buf);
#endif
#ifndef os_wmsbf4
//! Write 32-bit quntity into buffer in big endian byte order.
void os_wmsbf4 (xref2u1_t buf, u4_t value);
#endif
#ifndef os_rlsbf2
//! Read 16-bit quantity from given pointer in little endian byte order.
u2_t os_rlsbf2 (xref2cu1_t buf);
#endif
#ifndef os_wlsbf2
//! Write 16-bit quntity into buffer in little endian byte order.
void os_wlsbf2 (xref2u1_t buf, u2_t value);
#endif
#ifndef os_wlsbf3
//! Write 24-bit quntity into buffer in little endian byte order.
void os_wlsbf3 (xref2u1_t buf, u4_t value);
#endif

//! Get random number (default impl for u2_t).
#ifndef os_getRndU2
#define os_getRndU2() ((u2_t)((os_getRndU1()<<8)|os_getRndU1()))
#endif
#ifndef os_crc16
u2_t os_crc16 (xref2u1_t d, uint len);
#endif

#if defined(CFG_budha)
// HAL support required by BUDHA:
u1_t* os_getWSleepParams(void);
void  os_setWSleepParams(u1_t* params);
int   os_checkSignature(const u1_t* buf, u1_t len);
u4_t  os_getFWCRC(void);
int   os_usedWakeNonce(u2_t nonce);
void  os_commitWakeNonce(u2_t nonce);
enum { BOOT_WSLEEP=0, BOOT_OS };
void  os_setBootMode(u1_t mode);
void  os_boot(void);
s2_t  os_fwChunk(u1_t* p, u1_t len);
#endif // defined(CFG_budha)

#endif // !HAS_os_calls

// ======================================================================
// AES support
// !!Keep in sync with lorabase.hpp!!
// !!Keep in sync with bootloader/aes.c!!

#ifndef AES_ENC  // if AES_ENC is defined as macro all other values must be too
#define AES_ENC       0x00
#define AES_DEC       0x01
#define AES_MIC       0x02
#define AES_CTR       0x04
#define AES_MICNOAUX  0x08
#endif
#ifndef AESkey  // if AESkey is defined as macro all other values must be too
extern xref2u1_t AESkey;
extern xref2u1_t AESaux;
#endif
#ifndef os_aes
u4_t os_aes (u1_t mode, xref2u1_t buf, u2_t len);
#endif



#endif // _oslmic_h_
