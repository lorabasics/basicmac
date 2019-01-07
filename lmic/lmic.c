// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

//! \file
#include "lmic.h"
#include "backtrace.h"

#if !defined(MINRX_SYMS)
#define MINRX_SYMS 5
#endif // !defined(MINRX_SYMS)
#define PAMBL_SYMS_BCN BCN_PREAMBLE_LEN
#define PAMBL_SYMS     STD_PREAMBLE_LEN
#define PAMBL_FSK  5
#define PRERX_FSK  1
#define RXLEN_FSK  (1+5+2)
#define BCN_100PPM_ms 13   // 13ms = 128sec*100ppm which is roughly +/-100ppm

#define BCN_INTV_osticks       sec2osticks(BCN_INTV_sec)
#define TXRX_GUARD_osticks     ms2osticks(TXRX_GUARD_ms)
#define JOIN_GUARD_osticks     ms2osticks(JOIN_GUARD_ms)
#define DELAY_DNW1_osticks     sec2osticks(DELAY_DNW1)
#define DELAY_DNW2_osticks     sec2osticks(DELAY_DNW2)
#define DELAY_JACC1_osticks    sec2osticks(DELAY_JACC1)
#define DELAY_JACC2_osticks    sec2osticks(DELAY_JACC2)
#define DELAY_EXTDNW2_osticks  sec2osticks(DELAY_EXTDNW2)
#define BCN_RESERVE_osticks    ms2osticks(BCN_RESERVE_ms)
#define BCN_GUARD_osticks      ms2osticks(BCN_GUARD_ms)
#define BCN_WINDOW_osticks     ms2osticks(BCN_WINDOW_ms)
#define AIRTIME_BCN_osticks    us2osticksRound(AIRTIME_BCN)
#if defined(CFG_eu868)
#define DNW2_SAFETY_ZONE       ms2osticks(3000)
#endif
#if defined(CFG_us915)
#define DNW2_SAFETY_ZONE       ms2osticks(750)
#endif

// Special APIs - for development or testing
#if defined(CFG_extapi)
#define isTESTMODE() (LMIC.opmode & OP_TESTMODE)
#else
#define isTESTMODE() 0
#endif

DEFINE_LMIC;
DECL_ON_LMIC_EVENT;


// Fwd decls.
static void engineUpdate(void);
static void startScan (void);


// ================================================================================
// BEG OS - default implementations for certain OS suport functions

#if !defined(HAS_os_calls)

#if !defined(os_rlsbf2)
u2_t os_rlsbf2 (xref2cu1_t buf) {
    return (u2_t)(buf[0] | (buf[1]<<8));
}
#endif

#if !defined(os_rlsbf4)
u4_t os_rlsbf4 (xref2cu1_t buf) {
    return (u4_t)(buf[0] | (buf[1]<<8) | ((u4_t)buf[2]<<16) | ((u4_t)buf[3]<<24));
}
#endif


#if !defined(os_rmsbf4)
u4_t os_rmsbf4 (xref2cu1_t buf) {
    return (u4_t)(buf[3] | (buf[2]<<8) | ((u4_t)buf[1]<<16) | ((u4_t)buf[0]<<24));
}
#endif


#if !defined(os_wlsbf2)
void os_wlsbf2 (xref2u1_t buf, u2_t v) {
    buf[0] = v;
    buf[1] = v>>8;
}
#endif

#if !defined(os_wlsbf3)
void os_wlsbf3 (xref2u1_t buf, u4_t v) {
    buf[0] = v;
    buf[1] = v>>8;
    buf[2] = v>>16;
}
#endif

#if !defined(os_wlsbf4)
void os_wlsbf4 (xref2u1_t buf, u4_t v) {
    buf[0] = v;
    buf[1] = v>>8;
    buf[2] = v>>16;
    buf[3] = v>>24;
}
#endif

#if !defined(os_wmsbf4)
void os_wmsbf4 (xref2u1_t buf, u4_t v) {
    buf[3] = v;
    buf[2] = v>>8;
    buf[1] = v>>16;
    buf[0] = v>>24;
}
#endif

#if !defined(os_crc16)
// New CRC-16 CCITT(XMODEM) checksum for beacons:
u2_t os_crc16 (xref2u1_t data, uint len) {
    u2_t remainder = 0;
    u2_t polynomial = 0x1021;
    for( uint i = 0; i < len; i++ ) {
        remainder ^= data[i] << 8;
        for( u1_t bit = 8; bit > 0; bit--) {
            if( (remainder & 0x8000) )
                remainder = (remainder << 1) ^ polynomial;
            else
                remainder <<= 1;
        }
    }
    return remainder;
}
#endif

#endif // !HAS_os_calls

// END OS - default implementations for certain OS suport functions
// ================================================================================

// ================================================================================
// BEG AES

static void micB0 (u4_t devaddr, u4_t seqno, int dndir, int len) {
    os_clearMem(AESaux,16);
    AESaux[0]  = 0x49;
    AESaux[5]  = dndir?1:0;
    AESaux[15] = len;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
}


static int aes_verifyMic (xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t pdu, int len) {
    micB0(devaddr, seqno, dndir, len);
    os_copyMem(AESkey,key,16);
    return os_aes(AES_MIC, pdu, len) == os_rmsbf4(pdu+len);
}


static void aes_appendMic (xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t pdu, int len) {
    micB0(devaddr, seqno, dndir, len);
    os_copyMem(AESkey,key,16);
    // MSB because of internal structure of AES
    os_wmsbf4(pdu+len, os_aes(AES_MIC, pdu, len));
}


static void aes_appendMic0 (xref2u1_t pdu, int len) {
    os_getNwkKey(AESkey);
    os_wmsbf4(pdu+len, os_aes(AES_MIC|AES_MICNOAUX, pdu, len));  // MSB because of internal structure of AES
}


// Verify the Join-Accept MIC
// In in case of OptNeg, this will destroy the MIC in the incoming buffer
static int aes_verifyMic0 (xref2u1_t pdu, int len) {
    u4_t mic1 = os_rmsbf4(pdu+len);
#if defined(CFG_lorawan11)
    u1_t optneg = pdu[OFF_JA_DLSET] & JA_DLS_OPTNEG;
    if( optneg ) {
	os_moveMem(pdu+OFF_JA_JOINNONCE+2, pdu+OFF_JA_JOINNONCE, len-OFF_JA_JOINNONCE);
	os_wlsbf2(pdu+OFF_JA_JOINNONCE, LMIC.devNonce-1);
	len+=2;
    }
#endif
    os_getNwkKey(AESkey);
    u4_t mic2 = os_aes(AES_MIC|AES_MICNOAUX, pdu, len);
#if defined(CFG_lorawan11)
    if( optneg ) {
	os_moveMem(pdu+OFF_JA_JOINNONCE, pdu+OFF_JA_JOINNONCE+2, len-OFF_JA_JOINNONCE-2);
    }
#endif
    return mic1 == mic2;
}


static void aes_encrypt (xref2u1_t pdu, int len) {
    os_getNwkKey(AESkey);
    os_aes(AES_ENC, pdu, len);
}


static void aes_cipher (xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t payload, int len) {
    if( len <= 0 )
        return;
    if (!dndir && (LMIC.opmode & OP_NOCRYPT)) {
	return;
    }
    os_clearMem(AESaux, 16);
    AESaux[0] = AESaux[15] = 1; // mode=cipher / dir=down / block counter=1
    AESaux[5] = dndir?1:0;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
    os_copyMem(AESkey,key,16);
    os_aes(AES_CTR, payload, len);
}


static void aes_sessKeys (u2_t devnonce, xref2cu1_t joinnonce, xref2u1_t nwkkey,
#if defined(CFG_lorawan11)
	xref2u1_t nwkkeydn,
#endif
	xref2u1_t appkey) {
    os_clearMem(nwkkey, 16);
    nwkkey[0] = 0x01;
    os_copyMem(nwkkey+1, joinnonce, LEN_JOINNONCE+LEN_NETID);
    os_wlsbf2(nwkkey+1+LEN_JOINNONCE+LEN_NETID, devnonce);
    os_copyMem(appkey, nwkkey, 16);
    appkey[0] = 0x02;
#if defined(CFG_lorawan11)
    os_copyMem(nwkkeydn, nwkkey, 16);
    nwkkeydn[0] = 0x03;
#endif

    os_getNwkKey(AESkey);
    os_aes(AES_ENC, nwkkey, 16);
#if defined(CFG_lorawan11)
    if (LMIC.opts) {
	os_getNwkKey(AESkey);
	os_aes(AES_ENC, nwkkeydn, 16);
	os_getAppKey(AESkey);
    } else {
	os_copyMem(nwkkeydn, nwkkey, 16);
	os_getNwkKey(AESkey);
    }
#else
    os_getNwkKey(AESkey);
#endif
    os_aes(AES_ENC, appkey, 16);
}

// END AES
// ================================================================================


// ================================================================================
// BEG LORA

// Default scheme to map power index to dBm
#define powadj2dBm(p) (-2*(p))
// If region needs special version: #undef powadj2dBm   #define powadj2dBm..


#if defined(CFG_as923) // ========================================

#define maxFrameLen(dr) ((dr)<=DR_SF9 ? maxFrameLens[(dr)] : 0xFF)
const u1_t maxFrameLens [] = { 64,64,64,123 };

const s1_t effectiveDn1DrOff[8] = { 0,1,2,3,4,5,-1,-2 };
const u1_t _DR2RPS_CRC[] = {
    ILLEGAL_RPS,
    MAKERPS(SF12, BW125, CR_4_5, 0, 0),
    MAKERPS(SF11, BW125, CR_4_5, 0, 0),
    MAKERPS(SF10, BW125, CR_4_5, 0, 0),
    MAKERPS(SF9,  BW125, CR_4_5, 0, 0),
    MAKERPS(SF8,  BW125, CR_4_5, 0, 0),
    MAKERPS(SF7,  BW125, CR_4_5, 0, 0),
    MAKERPS(SF7,  BW250, CR_4_5, 0, 0),
    MAKERPS(FSK,  BW125, CR_4_5, 0, 0),
    ILLEGAL_RPS,
    ILLEGAL_RPS,
    ILLEGAL_RPS,
    ILLEGAL_RPS,
    ILLEGAL_RPS,
    ILLEGAL_RPS,
    ILLEGAL_RPS,
    ILLEGAL_RPS
};

#elif defined(CFG_eu868) // ========================================

#define maxFrameLen(dr) ((dr)<=DR_SF9 ? maxFrameLens[(dr)] : 0xFF)
const u1_t maxFrameLens [] = { 64,64,64,123 };

const s1_t effectiveDn1DrOff[8] = { 0,1,2,3,4,5,0,0 };
const u1_t _DR2RPS_CRC[] = {
    ILLEGAL_RPS,
    MAKERPS(SF12, BW125, CR_4_5, 0, 0),
    MAKERPS(SF11, BW125, CR_4_5, 0, 0),
    MAKERPS(SF10, BW125, CR_4_5, 0, 0),
    MAKERPS(SF9,  BW125, CR_4_5, 0, 0),
    MAKERPS(SF8,  BW125, CR_4_5, 0, 0),
    MAKERPS(SF7,  BW125, CR_4_5, 0, 0),
    MAKERPS(SF7,  BW250, CR_4_5, 0, 0),
    MAKERPS(FSK,  BW125, CR_4_5, 0, 0),
    ILLEGAL_RPS,
    ILLEGAL_RPS,
    ILLEGAL_RPS,
    ILLEGAL_RPS,
    ILLEGAL_RPS,
    ILLEGAL_RPS,
    ILLEGAL_RPS,
    ILLEGAL_RPS
};

#elif defined(CFG_au915) // ========================================

#define maxFrameLen(dr) ((dr)<=DR_SF11CR ? maxFrameLens[(dr)] : 0xFF)
const u1_t maxFrameLens [] = { 24,66,142,255,255,255,255,255,  66,142 };

const s1_t effectiveDn1DrOff[8] = { 0,1,2,3,4,5,0,0 };
const u1_t _DR2RPS_CRC[] = {
    ILLEGAL_RPS,
    MAKERPS(SF12, BW125, CR_4_5, 0, 0),
    MAKERPS(SF11, BW125, CR_4_5, 0, 0),
    MAKERPS(SF10, BW125, CR_4_5, 0, 0),
    MAKERPS(SF9 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF7 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW500, CR_4_5, 0, 0),
    ILLEGAL_RPS ,
    MAKERPS(SF12, BW500, CR_4_5, 0, 0),
    MAKERPS(SF11, BW500, CR_4_5, 0, 0),
    MAKERPS(SF10, BW500, CR_4_5, 0, 0),
    MAKERPS(SF9 , BW500, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW500, CR_4_5, 0, 0),
    MAKERPS(SF7 , BW500, CR_4_5, 0, 0),
    ILLEGAL_RPS ,
    ILLEGAL_RPS
};

#elif defined(CFG_cn470) // ========================================

#define maxFrameLen(dr) ((dr)<=DR_SF7 ? maxFrameLens[(dr)] : 0xFF)
const u1_t maxFrameLens [] = { 59,59,59,123,230,230 };

const s1_t effectiveDn1DrOff[8] = { 0,1,2,3,4,5,0,0 };
const u1_t _DR2RPS_CRC[] = {
    ILLEGAL_RPS,
    MAKERPS(SF12, BW125, CR_4_5, 0, 0),
    MAKERPS(SF11, BW125, CR_4_5, 0, 0),
    MAKERPS(SF10, BW125, CR_4_5, 0, 0),
    MAKERPS(SF9 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF7 , BW125, CR_4_5, 0, 0),
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    ILLEGAL_RPS
};

#elif defined(CFG_us915) // ========================================

#define maxFrameLen(dr) ((dr)<=DR_SF11CR ? maxFrameLens[(dr)] : 0xFF)
const u1_t maxFrameLens [] = { 24,66,142,255,255,255,255,255,  66,142 };

const s1_t effectiveDn1DrOff[8] = { 0,1,2,3,0,0,0,0 };
const u1_t _DR2RPS_CRC[] = {
    ILLEGAL_RPS,
    MAKERPS(SF10, BW125, CR_4_5, 0, 0),
    MAKERPS(SF9 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF7 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW500, CR_4_5, 0, 0),
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    MAKERPS(SF12, BW500, CR_4_5, 0, 0),
    MAKERPS(SF11, BW500, CR_4_5, 0, 0),
    MAKERPS(SF10, BW500, CR_4_5, 0, 0),
    MAKERPS(SF9 , BW500, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW500, CR_4_5, 0, 0),
    MAKERPS(SF7 , BW500, CR_4_5, 0, 0),
    ILLEGAL_RPS ,
    ILLEGAL_RPS
};

#endif // ================================================

static const u1_t SENSITIVITY[7][3] = {
    // ------------bw----------
    // 125kHz    250kHz    500kHz
    { 141-109,  141-109, 141-109 },  // FSK
    { 141-127,  141-124, 141-121 },  // SF7
    { 141-129,  141-126, 141-123 },  // SF8
    { 141-132,  141-129, 141-126 },  // SF9
    { 141-135,  141-132, 141-129 },  // SF10
    { 141-138,  141-135, 141-132 },  // SF11
    { 141-141,  141-138, 141-135 }   // SF12
};

int getSensitivity (rps_t rps) {
    return -141 + SENSITIVITY[getSf(rps)][getBw(rps)];
}

ostime_t calcAirTime (rps_t rps, u1_t plen) {
    u1_t bw = getBw(rps);  // 0,1,2 = 125,250,500kHz
    u1_t sf = getSf(rps);  // 0=FSK, 1..6 = SF7..12
    if( sf == FSK ) {
        return (plen+/*preamble*/5+/*syncword*/3+/*len*/1+/*crc*/2) * /*bits/byte*/8
            * (s4_t)OSTICKS_PER_SEC / /*kbit/s*/50000;
    }
    u1_t sfx = 4*(sf+(7-SF7));
    // XXX:??? condition for datarate optimize missing: sf==SF12 && bw==BW250 - but this is not used in any channel plan
    // XXX:??? -> see https://github.com/Lora-net/LoRaMac-node/blob/develop/src/radio/sx1276/sx1276.c
    u1_t q = sfx - (sf>=SF11 && bw==BW125 ? 8 : 0);
    int tmp = 8*plen - sfx + 28 + (getNocrc(rps)?0:16) - (getIh(rps)?20:0);
    if( tmp > 0 ) {
        tmp = (tmp + q - 1) / q;
        tmp *= getCr(rps)+5;
        tmp += 8;
    } else {
        tmp = 8;
    }
    tmp = (tmp<<2) + /*preamble*/49 /* 4 * (8 + 4.25) */;
    // bw = 125000 = 15625 * 2^3
    //      250000 = 15625 * 2^4
    //      500000 = 15625 * 2^5
    // sf = 7..12
    //
    // osticks =  tmp * OSTICKS_PER_SEC * 1<<sf / bw
    //
    // 3 => counter reduced divisor 125000/8 => 15625
    // 2 => counter 2 shift on tmp
    sfx = sf+(7-SF7) - (3+2) - bw;
    int div = 15625;
    if( sfx > 4 ) {
        // prevent 32bit signed int overflow in last step
        div >>= sfx-4;
        sfx = 4;
    }
    // Need 32bit arithmetic for this last step
    return (((ostime_t)tmp << sfx) * OSTICKS_PER_SEC + div/2) / div;
}

extern inline rps_t updr2dndr(dr_t dr, s1_t off);
extern inline rps_t updr2rps (dr_t dr);
extern inline rps_t dndr2rps (dr_t dr);
extern inline int isFasterDR (dr_t dr1, dr_t dr2);
extern inline int isSlowerDR (dr_t dr1, dr_t dr2);
extern inline dr_t  incDR    (dr_t dr);
extern inline dr_t  decDR    (dr_t dr);
extern inline dr_t  assertDR (dr_t dr);
extern inline dr_t  validDR  (dr_t dr);
extern inline dr_t  lowerDR  (dr_t dr, u1_t n);

extern inline sf_t  getSf    (rps_t params);
extern inline rps_t setSf    (rps_t params, sf_t sf);
extern inline bw_t  getBw    (rps_t params);
extern inline rps_t setBw    (rps_t params, bw_t cr);
extern inline cr_t  getCr    (rps_t params);
extern inline rps_t setCr    (rps_t params, cr_t cr);
extern inline int   getNocrc (rps_t params);
extern inline rps_t setNocrc (rps_t params, int nocrc);
extern inline int   getIh    (rps_t params);
extern inline rps_t setIh    (rps_t params, int ih);
extern inline rps_t makeRps  (sf_t sf, bw_t bw, cr_t cr, int ih, int nocrc);
extern inline int   sameSfBw (rps_t r1, rps_t r2);

// END LORA
// ================================================================================


#if defined(CFG_lorawan11)
static void incPollcnt (void) {
    u1_t c = LMIC.pollcnt;
    if( c < 0xFF )
	LMIC.pollcnt = c+1;
}

static void decPollcnt (void) {
    u1_t c = LMIC.pollcnt;
    if( c > 0 )
	LMIC.pollcnt = c-1;
}
#endif

// Table below defines the size of one symbol as
//   symtime = 256us * 2^T(sf,bw)
// 256us is called one symunit.
//                 SF:
//      BW:      |__7___8___9__10__11__12
//      125kHz   |  2   3   4   5   6   7
//      250kHz   |  1   2   3   4   5   6
//      500kHz   |  0   1   2   3   4   5
//
//  EU863___________       US915__________________
//    7 DR_SF12              5 DR_SF10   DR_SF12CR
//    6 DR_SF11		     4 DR_SF9    DR_SF11CR
//    5 DR_SF10		     3 DR_SF8    DR_SF10CR
//    4 DR_SF9		     2 DR_SF7    DR_SF9CR
//    3 DR_SF8		     1 DR_SF8C   DR_SF8CR
//    2 DR_SF7		     0 ------    DR_SF7CR
//    1 DR_SF7B
//    - FSK (not used)
//
// Calculate times for N half symbol per DR
// SYM_Q is chose as per:
//   - SYM_Q = z * q  where q = OSTICKS_PER_SEC / 1e6 -- q needed to convert from us to osticks
//   - z = 2^k and k is the biggest value so that SYM_Q < 2^32 (fits into u4_t)
//   - while considering OSTICKS_PER_SEC < 2^16
static const u4_t SYM_Q = (u4_t)( ((u8_t)1<<(16+20)) * OSTICKS_PER_SEC / (u8_t)1000000);
#if defined(CFG_us915)
#define DR0_SYM_EXP 5
#else // EU868, AU915, AS923
#define DR0_SYM_EXP 7
#endif

static ostime_t dr2hsym (dr_t dr, s1_t num) {
    int k = (16+20-7 +   // z=2^(((16+20))) / half sym = 128us = 2^(((7)))
	     - DR0_SYM_EXP + ((dr) & 7));
    // Note: 22 <= k <= 29
    s4_t rem = ((SYM_Q & ((1<<k)-1)) >> 8) * num;
    return (SYM_Q >> k) * num + (rem >> (k-8));
}

static ostime_t calcRxWindow (u1_t secs, dr_t dr) {
    ostime_t rxoff, err;

    // assume max wobble for missed bcn periods
    err = (ostime_t)LMIC.maxDriftDiff * LMIC.missedBcns;
    if( secs==0 ) {
        // aka 128 secs (next becaon)
	rxoff  = dr2hsym(dr, PAMBL_SYMS_BCN);
        rxoff += LMIC.drift;
        err   += LMIC.lastDriftDiff;
    } else {
        // scheduled RX window within secs into current beacon period
	rxoff  = dr2hsym(dr, PAMBL_SYMS);
        rxoff += (LMIC.drift * (ostime_t)secs) >> BCN_INTV_exp;
        err   += (LMIC.lastDriftDiff * (ostime_t)secs) >> BCN_INTV_exp;
    }
    // std RX window, enlarged by drift wobble
    ostime_t hsym = dr2hsym(dr,1);   // 1 symbol in ticks
    u4_t rxsyms = MINRX_SYMS + (err+hsym-1) / hsym;  // ceil syms
    // rxoff is the center of the beacon preamble adjusted by drift
    // rxsyms is the width of the rx window
    // limit for dr2hsym/rxsym: s1_t
    return rxoff - dr2hsym(dr, rxsyms>127 ? 127 : rxsyms);
}


// Setup beacon RX parameters assuming we need a tolerance of 'ms' (aka +/-ms)
static void calcBcnRxWindowFromMillis (u1_t ms, bit_t ini) {
    if( ini ) {
        LMIC.drift = 0;
        LMIC.maxDriftDiff = 0;
        LMIC.missedBcns = 0;
        LMIC.bcninfo.flags |= BCN_NODRIFT|BCN_NODDIFF;
    }
    ostime_t hsym = dr2hsym(DR_BCN,1);
    ostime_t cpre = dr2hsym(DR_BCN,PAMBL_SYMS_BCN);      // offset: center preamble
    int wsyms = (ms2osticksCeil(ms) + hsym - 1) / hsym;  // len RX span (2*ms) in syms (ceil)
    if( wsyms < MINRX_SYMS ) wsyms = MINRX_SYMS;         // no smaller than min
    ostime_t whspan = dr2hsym(DR_BCN,wsyms);             // half RX span in osticks
    LMIC.bcnRxsyms = wsyms;
    LMIC.bcnRxtime = LMIC.bcninfo.txtime + BCN_INTV_osticks + cpre - whspan;
}

static void iniRxdErr () {
    // Avg(rxdErrs) == 0
    // Min(rxdErrs) == -RXDERR_INI/2
    // Min(rxdErrs) == +RXDERR_INI/2
    LMIC.rxdErrs[0] = 0;
    for( u1_t u=RXDERR_NUM&0; u<RXDERR_NUM; u++ )
	LMIC.rxdErrs[u] = (us2osticksCeil(RXDERR_INI) << (RXDERR_SHIFT-1)) * (u&1?-1:1);
    LMIC.rxdErrIdx = 0;
}

static void addRxdErr (u1_t rxdelay) {
    s4_t err = (((LMIC.rxtime0 - LMIC.txend) - sec2osticks(rxdelay)) << RXDERR_SHIFT) / rxdelay;
    if( (u4_t)((err>>20)+1) > 1 )  // overflow?
	return;
    LMIC.rxdErrs[LMIC.rxdErrIdx] = err;
    LMIC.rxdErrIdx = (LMIC.rxdErrIdx + 1) % RXDERR_NUM;
}

static s4_t evalRxdErr (u4_t* span) {
    s4_t min = 0x7FFFFFFF, min2=0x7FFFFFFF;
    s4_t max = 0x80000000, max2=0x80000000;
    s4_t sum = 0;
    for( u1_t u=0; u<RXDERR_NUM; u++ ) {
	s4_t v = LMIC.rxdErrs[u];
	/**/ if( v <= min ) { min2=min; min=v; }
	else if( v <= min2) { min2=v; }
	/**/ if( v >= max ) { max2=max; max=v; }
	else if( v >= max2) { max2=v; }
	sum += v;
    }
    //*span = max-min;
    //return (sum + (RXDERR_NUM/2)) / RXDERR_NUM;
    *span = max2-min2;
    return (sum - max - min + ((RXDERR_NUM-2)/2)) / (RXDERR_NUM-2);
}

static void adjustByRxdErr (u1_t rxdelay, u1_t dr) {
#ifdef CFG_testpin
    // XXX: for now only in HW regr tests - not yet ready for prime time
    u4_t span;
    s4_t skew = evalRxdErr(&span);
    LMIC.rxtime += (skew * rxdelay + (1<<(RXDERR_SHIFT-1))) >> RXDERR_SHIFT;
    ostime_t hsym = dr2hsym(dr,1);
    span /= dr2hsym(dr,1); // additional half symbols
    LMIC.rxsyms += (span + 1) >> 1;
    LMIC.rxtime -= span*hsym;
#endif // CFG_testpin
}


// Setup scheduled RX window (ping/multicast slot)
static void rxschedInit (xref2rxsched_t rxsched) {
    os_clearMem(AESkey,16);
    os_clearMem(LMIC.frame+8,8);
    os_wlsbf4(LMIC.frame, LMIC.bcninfo.time);
    os_wlsbf4(LMIC.frame+4, LMIC.devaddr);
    os_aes(AES_ENC,LMIC.frame,16);
    // Relates to the standard in the following way:
    //   pingNb = 2^(7-intvExp)
    //   pingOffset = Rand % pingPeriod
    //   pingPeriod = 2^12 / pingNb = 2^12 / 2^(7-intvExp) = 2^5/2^-intvExp = 32<<intvExp
    ASSERT((LMIC.opmode & OP_PINGABLE) && rxsched->intvExp <= 7);
    u1_t intvExp = rxsched->intvExp;
    ostime_t off = os_rlsbf2(LMIC.frame) & ((32<<intvExp)-1); // random offset (slot units)
    rxsched->rxbase = (LMIC.bcninfo.txtime +
                       BCN_RESERVE_osticks +
                       ms2osticks(BCN_SLOT_SPAN_ms * off)); // random offset osticks
    rxsched->slot   = 0;
    rxsched->rxtime = rxsched->rxbase + calcRxWindow(/*secs BCN_RESERVE*/2+(1<<intvExp),rxsched->dr);
    rxsched->rxsyms = LMIC.rxsyms;
}


static bit_t rxschedNext (xref2rxsched_t rxsched, ostime_t cando) {
  again:
    if( rxsched->rxtime - cando >= 0 )
        return 1;
    u1_t slot;
    if( (slot=rxsched->slot) >= 128 )
        return 0;
    u1_t intv = 1<<(rxsched->intvExp & 0x7);
    if( (rxsched->slot = (slot += (intv))) >= 128 )
        return 0;
    rxsched->rxtime = rxsched->rxbase
        + ((BCN_WINDOW_osticks * (ostime_t)slot) >> BCN_INTV_exp)
        + calcRxWindow(/*secs BCN_RESERVE*/2+slot+intv,rxsched->dr);
    rxsched->rxsyms = LMIC.rxsyms;
    goto again;
}


static ostime_t rndDelay (u1_t secSpan) {
    u2_t r = os_getRndU2();
    ostime_t delay = r;
    if( delay > OSTICKS_PER_SEC )
        delay = r % (u2_t)OSTICKS_PER_SEC;
    if( secSpan > 0 )
        delay += ((u1_t)r % secSpan) * OSTICKS_PER_SEC;
    return delay;
}


static void txDelay (ostime_t reftime, u1_t secSpan) {
    reftime += rndDelay(secSpan);
    if( LMIC.globalDutyRate == 0  ||  (reftime - LMIC.globalDutyAvail) > 0 ) {
        LMIC.globalDutyAvail = reftime;
        LMIC.opmode |= OP_RNDTX;
    }
}


static void setDrJoin (u1_t reason, u1_t dr) {
    EV(drChange, INFO, (e_.reason    = reason,
                        e_.deveui    = MAIN::CDEV->getEui(),
                        e_.dr        = dr|DR_PAGE,
                        e_.txpow     = LMIC.txpow,
                        e_.prevdr    = LMIC.datarate|DR_PAGE,
                        e_.prevtxpow = LMIC.txpow));
    LMIC.datarate = dr;
    DO_DEVDB(LMIC.datarate,datarate);
}


static void setDrTxpow (u1_t reason, u1_t dr, s1_t powadj) {
    EV(drChange, INFO, (e_.reason    = reason,
                        e_.deveui    = MAIN::CDEV->getEui(),
                        e_.dr        = dr|DR_PAGE,
                        e_.txpow     = powadj,
                        e_.prevdr    = LMIC.datarate|DR_PAGE,
                        e_.prevtxpow = LMIC.txPowAdj));

    if( powadj != KEEP_TXPOWADJ )
        LMIC.txPowAdj = powadj;
    if( LMIC.datarate != dr ) {
        LMIC.datarate = dr;
        DO_DEVDB(LMIC.datarate,datarate);
        LMIC.opmode |= OP_NEXTCHNL;
    }
}


void LMIC_stopPingable (void) {
    LMIC.opmode &= ~(OP_PINGABLE|OP_PINGINI);
}


u1_t LMIC_setPingable (u1_t intvExp) {
    ASSERT(intvExp <= 7);
    // Change setting
    if( LMIC.ping.intvExp == intvExp ) {
	LMIC.opmode |= OP_PINGABLE;
	return 0;  // no change
    }
    // Change of interval requires to disable class B until we got this ACKed
    LMIC.ping.intvExp = 0x80 | intvExp;
    LMIC.opmode &= ~OP_PINGABLE;
    if( (LMIC.opmode & OP_TRACK) != 0 )
	return 1;   // already tracking a beacon - communicating change to NWKS
    // Start tracking a beacon
    LMIC_enableTracking(3);
    return 2;
}

// ================================================================================
//
// BEG: EU868 related stuff
//

#if defined(CFG_as923)

enum { NUM_JOIN_CHANNELS=2 };

static void initDefaultChannels (bit_t join) {
    os_clearMem(&LMIC.channelFreq,   sizeof(LMIC.channelFreq));
    os_clearMem(&LMIC.channelDnFreq, sizeof(LMIC.channelDnFreq));
    os_clearMem(&LMIC.channelDrMap,  sizeof(LMIC.channelDrMap));
    os_clearMem(&LMIC.bands, sizeof(LMIC.bands));
    LMIC.channelMap = 0;
    LMIC.perChnlDC = 0;    // no per channel DC

    LMIC_setupBand(BAND_MILLI, 14, 1000); // 0.1%
    LMIC_setupBand(BAND_CENTI, 14, 100);  // 1%
    LMIC_setupBand(BAND_DECI,  27, 10);   // 10%

    if(join) { // XXX for now join with 0.1% duty cycle, need to implement 1h/10h/24h back-off...!
	LMIC_setupChannel(0, AS923_F1, 0, BAND_MILLI);
	LMIC_setupChannel(1, AS923_F2, 0, BAND_MILLI);
    } else {
	LMIC_setupChannel(0, AS923_F1, 0, -1);
	LMIC_setupChannel(1, AS923_F2, 0, -1);
    }
}

#elif defined(CFG_il915)

enum { NUM_JOIN_CHANNELS=3 };

static void initDefaultChannels (bit_t join) {
    os_clearMem(&LMIC.channelFreq,   sizeof(LMIC.channelFreq));
    os_clearMem(&LMIC.channelDnFreq, sizeof(LMIC.channelDnFreq));
    os_clearMem(&LMIC.channelDrMap,  sizeof(LMIC.channelDrMap));
    os_clearMem(&LMIC.bands, sizeof(LMIC.bands));
    LMIC.channelMap = 0;
    LMIC.perChnlDC = 0;    // no per channel DC

    LMIC_setupBand(BAND_MILLI, 14, 1000); // 0.1%
    LMIC_setupBand(BAND_CENTI, 14, 100);  // 1%
    LMIC_setupBand(BAND_DECI,  14, 10);   // 10%

    if(join) { // XXX for now join with 0.1% duty cycle, need to implement 1h/10h/24h back-off...!
	LMIC_setupChannel(0, IL915_F1, 0, BAND_MILLI);
	LMIC_setupChannel(1, IL915_F2, 0, BAND_MILLI);
	LMIC_setupChannel(2, IL915_F3, 0, BAND_MILLI);
    } else {
	LMIC_setupChannel(0, IL915_F1, 0, -1);
	LMIC_setupChannel(1, IL915_F2, 0, -1);
	LMIC_setupChannel(2, IL915_F3, 0, -1);
    }
}

#elif defined(CFG_kr920)

// XXX - need to implement LBT instead of DC

enum { NUM_JOIN_CHANNELS=3 };

static void initDefaultChannels (bit_t join) {
    os_clearMem(&LMIC.channelFreq,   sizeof(LMIC.channelFreq));
    os_clearMem(&LMIC.channelDnFreq, sizeof(LMIC.channelDnFreq));
    os_clearMem(&LMIC.channelDrMap,  sizeof(LMIC.channelDrMap));
    os_clearMem(&LMIC.bands, sizeof(LMIC.bands));
    LMIC.channelMap = 0;
    LMIC.perChnlDC = 0;    // no per channel DC

    LMIC_setupBand(BAND_MILLI, 14, 1000); // 0.1%
    LMIC_setupBand(BAND_CENTI, 14, 100);  // 1%
    LMIC_setupBand(BAND_DECI,  14, 10);   // 10%

    if(join) { // XXX for now join with 0.1% duty cycle, need to implement 1h/10h/24h back-off...!
	LMIC_setupChannel(0, KR920_F1, 0, BAND_MILLI);
	LMIC_setupChannel(1, KR920_F2, 0, BAND_MILLI);
	LMIC_setupChannel(2, KR920_F3, 0, BAND_MILLI);
    } else {
	LMIC_setupChannel(0, KR920_F1, 0, -1);
	LMIC_setupChannel(1, KR920_F2, 0, -1);
	LMIC_setupChannel(2, KR920_F3, 0, -1);
    }
}

#elif defined(CFG_eu868)

enum { NUM_JOIN_CHANNELS=3 };

static void initDefaultChannels (bit_t join) {
    os_clearMem(&LMIC.channelFreq,   sizeof(LMIC.channelFreq));
    os_clearMem(&LMIC.channelDnFreq, sizeof(LMIC.channelDnFreq));
    os_clearMem(&LMIC.channelDrMap,  sizeof(LMIC.channelDrMap));
    os_clearMem(&LMIC.bands, sizeof(LMIC.bands));
    LMIC.channelMap = 0;
    LMIC.perChnlDC = 36;    // per channel DC for PSA: 100s/1h max cumulative on time

    LMIC_setupBand(BAND_MILLI, 14, 1000); // 0.1%
    LMIC_setupBand(BAND_CENTI, 14, 100);  // 1%
    LMIC_setupBand(BAND_DECI,  27, 10);   // 10%

    if(join) { // XXX for now join with 0.1% duty cycle, need to implement 1h/10h/24h back-off...!
	LMIC_setupChannel(0, EU868_F1, 0, BAND_MILLI);
	LMIC_setupChannel(1, EU868_F2, 0, BAND_MILLI);
	LMIC_setupChannel(2, EU868_F3, 0, BAND_MILLI);
    } else {
	LMIC_setupChannel(0, EU868_F1, 0, -1); // default 1%
	LMIC_setupChannel(1, EU868_F2, 0, -1);
	LMIC_setupChannel(2, EU868_F3, 0, -1);
    }
}

#endif


#if defined(CFG_eu868)

bit_t LMIC_setupBand (u1_t bandidx, s1_t txpow, u2_t txcap) {
    if( bandidx > BAND_AUX ) return 0;
    //band_t* b = &LMIC.bands[bandidx];
    xref2band_t b = &LMIC.bands[bandidx];
    b->txpow = txpow;
    b->txcap = txcap;
    b->avail = os_getTime();
    return 1;
}

bit_t LMIC_setupChannel (u1_t chidx, u4_t freq, u2_t drmap, s1_t band) {
    if( chidx >= MAX_CHNLS )
        return 0;
    if( freq == 0 ) {
	LMIC_disableChannel(chidx);
	return 1;
    }
    if( band == -1 ) {
#if defined(CFG_as923)
	freq |= BAND_DECI;
#elif defined(CFG_il915)
	freq |= BAND_CENTI;
#elif defined(CFG_kr920)
	freq |= BAND_CENTI; // XXX - use LBT!
#else
        if( freq >= 869400000 && freq <= 869650000 )
            freq |= BAND_DECI;   // 10% 27dBm
        else if( (freq >= 868000000 && freq <= 868600000) ||
                 (freq >= 869700000 && freq <= 870000000)  )
            freq |= BAND_CENTI;  // 1% 14dBm
        else
            freq |= BAND_MILLI;  // 0.1% 14dBm
#endif
    } else {
        if( band > BAND_AUX ) return 0;
        freq = (freq&~3) | band;
    }
    LMIC.channelFreq  [chidx] = freq;
    LMIC.channelDnFreq[chidx] = 0;   // reset DN freq if channel is setup/modified
    LMIC.channelDrMap [chidx] = drmap==0 ? DR_RANGE_MAP(DR_SF12,DR_SF7) : drmap;
    LMIC.channelMap |= 1<<chidx;     // enabled right away
    return 1;
}

void LMIC_disableChannel (u1_t channel) {
    LMIC.channelFreq  [channel] = 0;
    LMIC.channelDnFreq[channel] = 0;
    LMIC.channelDrMap [channel] = 0;
    LMIC.channelMap &= ~(1<<channel);
    if( LMIC.channelMap == 0 )
	LMIC.channelMap = (1<<MIN_CHNLS)-1;  // safety net
}

static s4_t convFreq (xref2u1_t ptr) {
    u4_t freq = (os_rlsbf4(ptr-1) >> 8) * 100;
    if( freq != 0 && (freq < FREQ_MIN || freq > FREQ_MAX) )
        freq = -1;
    return freq;
}

static u1_t applyChannelMap (u1_t chpage, u2_t chmap, u2_t* dest) {
    if( chpage == MCMD_LADR_CHP_ALLON ) {
	chmap = 0;
	for( u1_t ci=0; ci<MAX_CHNLS; ci++ ) {
	    if( LMIC.channelFreq[ci] == 0 ) {
		chmap |= (1<<ci);
	    }
	}
    } else if( chpage != 0 ) {
	return 0;  // illegal input
    }
    *dest = chmap;
    return 1;
}

static u1_t checkChannelMap (u2_t* map) {
    if( *map == 0) {
	return 0; // no channel is enabled
    }
    for( u1_t ci=0; ci<MAX_CHNLS; ci++ ) {
        if( (*map & (1<<ci)) != 0 && LMIC.channelFreq[ci] == 0 ) {
	    return 0; // channel is not defined
	}
    }
    return 1;
}

static void syncDatarate () {
    u2_t endrs = 0;  // enabled data rates
    for( u1_t ci=0; ci<MAX_CHNLS; ci++ ) {
        if( LMIC.channelMap & (1<<ci) )
	    endrs |= LMIC.channelDrMap[ci];
    }
    ASSERT(endrs != 0);
    if( ((1<<LMIC.datarate) & endrs) == 0 ) {
	// Find closest DR
	u1_t drbit = 1<<LMIC.datarate;
	u1_t dr, n = 1;
	while(1) {
	    if( (drbit>>n) & endrs ) {
		dr = LMIC.datarate - n;
		break;
	    }
	    if( (drbit<<n) & endrs ) {
		dr = LMIC.datarate + n;
		break;
	    }
	    n += 1;
	}
	setDrTxpow(DRCHG_SET, dr, KEEP_TXPOWADJ);
    }
}

static void updateTx (ostime_t txbeg) {
    u4_t freq = LMIC.channelFreq[LMIC.txChnl];
    // Update global/band specific duty cycle stats
    ostime_t airtime = calcAirTime(LMIC.rps, LMIC.dataLen);
    // Update channel/global duty cycle stats
    xref2band_t band = &LMIC.bands[freq & 0x3];
    LMIC.freq  = freq & ~(u4_t)3;
    LMIC.txpow = band->txpow;
    band->avail = os_time2XTime(txbeg + airtime * band->txcap, os_getXTime());
    LMIC.channelAvail[LMIC.txChnl] = os_time2XTime(txbeg, os_getXTime()) + (osxtime_t)airtime * LMIC.perChnlDC;
    if( LMIC.globalDutyRate != 0 )
        LMIC.globalDutyAvail = txbeg + (airtime<<LMIC.globalDutyRate);
}

static u1_t selectRandomChnl (u2_t map, u1_t nbits) {
    u1_t k;
 again:
    // Note: we have a small negligible bias of 2^16 % nbits (nbits <= 16 => bias < 0.025%)
    k = os_getRndU2() % nbits;
    for( u1_t chnl=0; chnl<16; chnl++ ) {
	if( (map & (1<<chnl)) == 0 )
	    continue;
	if( k==0 ) {
	    if( LMIC.refChnl == chnl && nbits > 1 )
		goto again; // don't use same channel twice
	    LMIC.refChnl = chnl;
	    return chnl;
	}
	k--;
    }
    ASSERT(0);
    return 0;
}

static ostime_t nextTx (ostime_t now) {
    u2_t drbit = 1<<(LMIC.datarate&0xF);
    osxtime_t xnow = os_getXTime();
    osxtime_t avail, txavail  = 0x7FFFffffFFFFffff;
    u1_t cccnt = 0;  // count # of bits
    u2_t ccmap = 0;  // PSA check channels
    u2_t dcmap = 0;  // duty cycle free
 again:
    for( u1_t chnl=0; chnl<MAX_CHNLS; chnl++ ) {
	u2_t chnlbit = 1<<chnl;
	if( (LMIC.channelMap & chnlbit) == 0 ||      // channel disabled
	    (LMIC.channelDrMap[chnl] & drbit) == 0 ) // or not enabled for current datarate
	    continue;
	u1_t ok=0;
	if( LMIC.perChnlDC ) {
	    avail = LMIC.channelAvail[chnl];
	    ok = (xnow >= avail);
	    if( avail < xnow )   // avoid clear channel polling
		avail = xnow + OSTICKS_PER_SEC;
	    if( txavail >= avail )
		txavail = avail;
	}
	avail = LMIC.bands[LMIC.channelFreq[chnl] & 3].avail;
	if( txavail >= avail )
	    txavail = avail;
	if( LMIC.noDC || xnow >= avail ) {
	    dcmap |= chnlbit;
	    ok = 1;
	}
	if( ok ) {
	    ccmap |= chnlbit;
	    cccnt++;
	}
    }
    if( !LMIC.noDC && txavail == 0x7FFFffffFFFFffff ) {
	// No suitable channel found - Maybe there's no channel which includes current datarate
	syncDatarate();
	u2_t drbit2 = 1<<(LMIC.datarate&0xF);
	ASSERT(drbit != drbit2);
	drbit = drbit2;
	goto again;
    }

    if( cccnt ) {
	rps_t rps = updr2rps(LMIC.datarate&0xF);
	while( cccnt ) {
	    u1_t probechnl = selectRandomChnl(ccmap, cccnt);
	    if( (dcmap & (1<<probechnl)) != 0 ) {
		LMIC.opmode &= ~OP_NEXTCHNL;  // channel decision is stable
	    done:
		LMIC.txChnl = probechnl;
		return os_getTime();
	    }
	    // Check per channel duty cycle - EU868 polite spectrum access
	    // os_cca() - clear channel assessment (ETSI EN 300 220-1 V3.1.1 (2017-02)
	    //   - threshold depends on output power and antenna gain  (5.21.2.2)
	    //   - assessment lasts 160us (5.21.3.1)
	    //   - max TX time <= 1s
	    //   - Toff after TX 100ms
	    //   - max TX on 100s / 1h
	    // Note: each call to os_cca takes 160us
	    // Worst case: 16 x 160us => nextTx might take up to 2.56ms to complete!
	    if( os_cca(rps, LMIC.channelFreq[probechnl] & ~3) ) {
		goto done;    // channel decision is not stable
	    }
	    ccmap &= ~(1<<probechnl);   // probed but not free
	    cccnt--;
	}
    }
    // Earliest duty cycle expiry of a band
    // or earliest time a channel might be tested again under PSA.
    return (ostime_t)txavail;
}


static void setBcnRxParams (void) {
    LMIC.dataLen = 0;
    LMIC.freq = LMIC.bcnFreq ? LMIC.bcnFreq : FREQ_BCN;
    LMIC.rps  = setIh(setNocrc(dndr2rps((dr_t)DR_BCN),1),LEN_BCN);
}

static void initJoinLoop (void) {
#if CFG_TxContinuousMode
    LMIC.txChnl = 0;
#else
    LMIC.txChnl = os_getRndU1() % NUM_JOIN_CHANNELS;
#endif
    LMIC.txPowAdj = 0;
    LMIC.nbTrans = 0;
    setDrJoin(DRCHG_SET, DR_SF7);
    initDefaultChannels(1);
    ASSERT((LMIC.opmode & OP_NEXTCHNL)==0);
    LMIC.txend = LMIC.bands[BAND_MILLI].avail + rndDelay(8); // random delay before first join req
}


static ostime_t nextJoinState (void) {
    u1_t failed = 0;

    // use next channel
    if( ++LMIC.txChnl == NUM_JOIN_CHANNELS ) {
        LMIC.txChnl = 0;
    }
    // lower DR every 2nd try
    if( (++LMIC.txCnt & 1) == 0 ) {
        if( LMIC.datarate == DR_SF12 ) {
            failed = 1; // we have tried all DR - signal EV_JOIN_FAILED
	}
        else {
            setDrJoin(DRCHG_NOJACC, decDR((dr_t)LMIC.datarate));
	}
    }
    // Clear NEXTCHNL because join state engine controls channel hopping
    LMIC.opmode &= ~OP_NEXTCHNL;
    // Move txend to randomize synchronized concurrent joins.
    // Duty cycle is based on txend.
    ostime_t time = os_getTime();
    if( time - LMIC.bands[BAND_MILLI].avail < 0 )
        time = LMIC.bands[BAND_MILLI].avail;
    LMIC.txend = time +
        (isTESTMODE()
         // Avoid collision with JOIN ACCEPT @ SF12 being sent by GW (but we missed it)
         ? DNW2_SAFETY_ZONE
         // Otherwise: randomize join (street lamp case):
         // SF12:255, SF11:127, .., SF7:8secs
         : DNW2_SAFETY_ZONE+rndDelay(255>>LMIC.datarate));
    // 1 - triggers EV_JOIN_FAILED event
    return failed;
}

//
// END: EU868 related stuff
//
// ================================================================================
#elif defined(CFG_us915)
// ================================================================================
//
// BEG: US915 related stuff
//

// return: 1 - some channels were disabled, 0 - all channels were already enabled
static u1_t enableAllChannels (void) {
    u1_t rv = 0;
    for (u1_t i = 0; i < (MAX_CHNLS >> 4); i++) {
	if (LMIC.channelMap[i] != 0xFFFF) {
	    LMIC.channelMap[i] = 0xFFFF;
	    rv = 1;
	}
    }
    if (MAX_CHNLS & 0xF ) {
	if (LMIC.channelMap[MAX_CHNLS >> 4] != (1<<(MAX_CHNLS & 0xF)) - 1) {
	    LMIC.channelMap[MAX_CHNLS >> 4] = (1<<(MAX_CHNLS & 0xF)) - 1;
	    rv = 1;
	}
    }
    return rv;
}


static void initDefaultChannels (void) {
    enableAllChannels();
}

static s4_t convFreq (xref2u1_t ptr) {
    u4_t freq = (os_rlsbf4(ptr-1) >> 8) * 100;
    if( freq != 0 && (freq < FREQ_MIN || freq > FREQ_MAX) )
        freq = -1;
    return freq;
}

bit_t LMIC_setupChannel (u1_t chidx, u4_t freq, u2_t drmap, s1_t band) {
    // US likes have a fixed channel plan - no changes allowed
    return 0;   // fail
}

void LMIC_disableChannel (u1_t channel) {
    if( channel < MAX_CHNLS )
        LMIC.channelMap[channel>>4] &= ~(1<<(channel&0xF));
    // safety net - all channels disabled -> turn all on
    u2_t anyon = 0;
    for( u1_t i=0; i<sizeof(LMIC.channelMap)/sizeof(u2_t); i++ )
        anyon |= LMIC.channelMap[i];
    if( !anyon )
	initDefaultChannels();
}

static u1_t checkChannelMap (u2_t* map) {
    u2_t anyon = 0;
    for( u1_t i=0; i<sizeof(LMIC.channelMap)/sizeof(u2_t); i++ ) {
        anyon |= map[i];
    }
    return !!anyon;
}

#if defined(CFG_cn470)
static u1_t applyChannelMap (u1_t chpage, u2_t chmap, u2_t* dest) {
    if( chpage == MCMD_LADR_CHP_ALLON ) {
        for( u1_t u=0; u<(MAX_CHNLS+15)/16; u++ )
            dest[u] = 0xFFFF;
    } else {
	chpage >>= MCMD_LADR_CHPAGE_SHIFT;
        if( chpage >= (MAX_CHNLS+15)/16 )
            return 0;
        dest[chpage] = chmap;
    }
    return 1;
}

static void syncDatarate () {
}

#else // CFG_us915
static u1_t applyChannelMap (u1_t chpage, u2_t chmap, u2_t* dest) {
    if( chpage == MCMD_LADR_CHP_125ON || chpage == MCMD_LADR_CHP_125OFF ) {
        u2_t en125 = chpage == MCMD_LADR_CHP_125ON ? 0xFFFF : 0x0000;
        for( u1_t u=0; u<4; u++ )
            dest[u] = en125;
        dest[64/16] = chmap;
    } else if( chpage == MCMD_LADR_CHP_BLK8 )  {
	dest[64/16] = chmap & 0xFF;
	for( u1_t u=0; u<4; u++ ) {
	    dest[u] = ((chmap & 1) ? 0x00ff : 0) | ((chmap & 2) ? 0xff00 : 0);
	    chmap >>= 2;
	}
    } else {
	chpage >>= MCMD_LADR_CHPAGE_SHIFT;
        if( chpage >= (MAX_CHNLS+15)/16 )
            return 0;
	if( (chpage == 64/16) && (chmap & ~0xFF) ) {
	    return 0;
	}
        dest[chpage] = chmap;
    }
    return 1;
}

static void syncDatarate () {
    if( LMIC.channelMap[64/16] == 0 && LMIC.datarate == DR_SF8C ) {
	setDrTxpow(DRCHG_SET, DR_SF7, KEEP_TXPOWADJ);
    }
    else if( (LMIC.channelMap[0] | LMIC.channelMap[1] | LMIC.channelMap[2] | LMIC.channelMap[3]) == 0 &&
	LMIC.datarate < DR_SF8C ) {
	setDrTxpow(DRCHG_SET, DR_SF8C, KEEP_TXPOWADJ);
    }
}

#endif // CFG_us915

#if defined(CFG_cn470)
static void updateTx (ostime_t txbeg) {
    ostime_t airtime = calcAirTime(LMIC.rps, LMIC.dataLen);
    u1_t chnl = LMIC.txChnl;
    LMIC.freq = CN470_125kHz_UPFBASE + (chnl % 48) * CN470_125kHz_UPFSTEP;
    if( LMIC.globalDutyRate != 0 ) {
        airtime = calcAirTime(LMIC.rps, LMIC.dataLen);
        LMIC.globalDutyAvail = txbeg + (airtime<<LMIC.globalDutyRate);
    }
}
#else
static int activeChannelCount (void) {
    int i, cc = 0;
    for (i = 0; i < 4; i++) {
	cc += __builtin_popcount(LMIC.channelMap[i]);
    }
    return cc;
}

static void updateTx (ostime_t txbeg) {
    ostime_t airtime;
    u1_t chnl = LMIC.txChnl;
    if( chnl < 64 ) {
        LMIC.freq = US915_125kHz_UPFBASE + chnl*US915_125kHz_UPFSTEP;

	int chcnt = activeChannelCount();

	LMIC.txpow =
#if defined(CFG_high_txpow)
	    (chcnt >= 50) ? 30 :
#endif
	    21;

	// update channel dwell time availability
	airtime = calcAirTime(LMIC.rps, LMIC.dataLen);
	// add 6.25% margin to airtime
	airtime += (airtime >> 4);
	LMIC.chDwellAvail[chnl] = os_time2XTime(txbeg
		+ ((chcnt >= 50) ? sec2osticks(20) : chcnt * ms2osticks(400))
		- (ms2osticks(400) - airtime),
		os_getXTime());
        return;
    }
    LMIC.txpow = 26;
    ASSERT( chnl < MAX_CHNLS );
    LMIC.freq = US915_500kHz_UPFBASE + (chnl-64)*US915_500kHz_UPFSTEP;

    // Update global duty cycle stats XXX this is skipped for 125kHz channels right now!
    if( LMIC.globalDutyRate != 0 ) {
        airtime = calcAirTime(LMIC.rps, LMIC.dataLen);
        LMIC.globalDutyAvail = txbeg + (airtime<<LMIC.globalDutyRate);
    }
}
#endif

/*
 * >>> import numpy as np
 * >>> np.random.permutation(64)
 */
#if defined(CFG_cn470)
static const unsigned char HOPLIST[MAX_CHNLS] = {
    9, 62, 83, 69, 60, 58, 75, 28, 88, 74, 45, 59, 71, 12, 22, 36, 66,
    70, 43, 93, 92, 80, 55, 29, 30, 76, 38, 94,  7, 82, 42, 68, 16, 13,
    25, 18,  5, 17, 48, 91, 51, 79, 10, 44,  2,  1, 77, 81, 11, 64, 50,
    47,  8, 61, 24, 54, 84, 89, 15, 85,  4, 78, 20, 57, 33,  6, 41,  3,
    27, 53, 35, 95, 39, 34, 49, 23, 37, 90, 72, 31, 21, 65, 67, 26, 86,
    19,  0, 52, 40, 46, 14, 73, 87, 63, 32, 56
};

static ostime_t nextTx (ostime_t now) {
    for( u1_t i=0; i<MAX_CHNLS; i++ ) {
	LMIC.refChnl = (LMIC.refChnl+1) % MAX_CHNLS;
	u1_t chnl = HOPLIST[LMIC.refChnl];
	if( (LMIC.channelMap[(chnl >> 4)] & (1<<(chnl & 0xF))) != 0 ) {
	    LMIC.txChnl = chnl;
	    break;
	}
    }
    LMIC.opmode &= ~OP_NEXTCHNL;  // channel decision is stable
    osxtime_t xnow = os_time2XTime(now, os_getXTime());
    osxtime_t avail = LMIC.chDwellAvail[LMIC.txChnl];
    return (ostime_t) ((xnow >= avail) ? xnow : avail);
}

#else

static const unsigned char HOPLIST[64] = {
    20, 57, 41, 53, 59, 60, 61, 39, 27, 52, 30, 29,  6, 21, 43, 54,  8, 3, 49, 38, 22, 47, 46, 32, 10,  5, 42, 33, 13,  7, 45, 40,
    35, 63, 16, 26, 23, 58, 14, 50, 51, 24,  4,  0, 19, 36, 31, 37, 56, 2, 25, 55,  1,  9, 48, 15, 18, 17, 62, 34, 44, 11, 12, 28
};

static ostime_t nextTx (ostime_t now) {
    if( LMIC.datarate >= DR_SF8C ) { // 500kHz
        u1_t map = LMIC.channelMap[64/16]&0xFF;
        for( u1_t i=0; i<8; i++ ) {
            if( (map & (1<<(++LMIC.refChnl & 7))) != 0 ) {
                LMIC.txChnl = 64 + (LMIC.refChnl & 7);
		break;
            }
        }
	return now;
    }
    // 125kHz
    for( u1_t i=0; i<64; i++ ) {
	u1_t chnl = HOPLIST[++LMIC.refChnl & 0x3F];
	if( (LMIC.channelMap[(chnl >> 4)] & (1<<(chnl & 0xF))) != 0 ) {
	    LMIC.txChnl = chnl;
	    break;
	}
    }
    LMIC.opmode &= ~OP_NEXTCHNL;  // channel decision is stable
    osxtime_t xnow = os_time2XTime(now, os_getXTime());
    osxtime_t avail = LMIC.chDwellAvail[LMIC.txChnl];
    return (ostime_t) ((xnow >= avail) ? xnow : avail);
}
#endif

static void setBcnRxParams (void) {
    LMIC.dataLen = 0;
    LMIC.rps  = setIh(setNocrc(dndr2rps((dr_t)DR_BCN),1),LEN_BCN);
    LMIC.freq = (LMIC.bcnFreq ? LMIC.bcnFreq : BCN_FREQ_BASE + LMIC.bcnChnl * BCN_FREQ_STEP);
}

static void initJoinLoop (void) {
    LMIC.refChnl = os_getRndU1();
    LMIC.txChnl = 0;
    LMIC.txPowAdj = 0;
    ASSERT((LMIC.opmode & OP_NEXTCHNL)==0);
    LMIC.txend = os_getTime();
    setDrJoin(DRCHG_SET, DR_SF7);
}

static ostime_t nextJoinState (void) {
    u1_t failed = 0;
#if defined(CFG_cn470)
    LMIC.txChnl = ((LMIC.txChnl & ~0x07) + 8 + (os_getRndU1() & 0x07)) % MAX_CHNLS;
    s1_t dr = DR_SF7 - (++LMIC.txCnt / 8);
    if( dr < DR_UPSLOWEST ) {
	dr = DR_UPSLOWEST;
	failed = 1; // All DR exhausted - signal failed
    }
    setDrJoin(DRCHG_SET, dr);
#else
    // Try the following:
    //   SF7/8/9/10  on a random channel 0..63
    //   SF8C        on a random channel 64..71
    //
    if( LMIC.datarate != DR_SF8C ) {
        LMIC.txChnl = 64 + (LMIC.txChnl / 8);
        setDrJoin(DRCHG_SET, DR_SF8C);
    } else {
	LMIC.txChnl = (((LMIC.txChnl - 63) & 0x07) * 8) | (os_getRndU1() & 0x07);
	s1_t dr = DR_SF7 - (++LMIC.txCnt / 8);
        if( dr < DR_UPSLOWEST ) {
            dr = DR_UPSLOWEST;
            failed = 1; // All DR exhausted - signal failed
        }
        setDrJoin(DRCHG_SET, dr);
    }
#endif
    LMIC.opmode &= ~OP_NEXTCHNL;
    LMIC.txend = os_getTime() +
        (isTESTMODE()
         // Avoid collision with JOIN ACCEPT being sent by GW (but we missed it - GW is still busy)
         ? DNW2_SAFETY_ZONE
         // Otherwise: randomize join (street lamp case):
         // SF10:16, SF9=8,..SF8C:1secs
         : rndDelay(16>>LMIC.datarate));
    // 1 - triggers EV_JOIN_FAILED event
    return failed;
}

//
// END: US915 related stuff
//
// ================================================================================
#else
#error Unsupported frequency band!
#endif


static void runEngineUpdate (xref2osjob_t osjob) {
    engineUpdate();
}


static void reportEvent (ev_t ev) {
    EV(devCond, INFO, (e_.reason = EV::devCond_t::LMIC_EV,
                       e_.eui    = MAIN::CDEV->getEui(),
                       e_.info   = ev));
    TRACE_VAL(ev);
    ON_LMIC_EVENT(ev);
    engineUpdate();
}


static void runReset (xref2osjob_t osjob) {
    // Disable session
    LMIC_reset();
    LMIC_startJoining();
    reportEvent(EV_RESET);
}

static void stateJustJoined (void) {
    LMIC.opmode     &= ~(OP_JOINING|OP_TRACK|OP_REJOIN|OP_TXRXPEND|OP_PINGINI) | OP_NEXTCHNL;
    LMIC.opmode     |= OP_NEXTCHNL;
    LMIC.txCnt       = 0;
    LMIC.seqnoDn     = LMIC.seqnoUp = 0;
#if defined(CFG_lorawan11)
    LMIC.seqnoADn    = 0;
#endif
    LMIC.rejoinCnt   = 0;
    LMIC.foptsUpLen  = 0;
    LMIC.dnConf      = LMIC.devsAns = 0;
    LMIC.dnfqAns     = LMIC.dnfqAnsPend = LMIC.dnfqAcks = 0;
    LMIC.moreData    = LMIC.dn2Ans = LMIC.dn1DlyAns = LMIC.dutyCapAns = 0;
    LMIC.adrAckLimit = ADR_ACK_LIMIT;
    LMIC.adrAckDelay = ADR_ACK_DELAY;
    LMIC.adrAckReq   = LINK_CHECK_INIT;
    LMIC.dn2Freq     = FREQ_DNW2;
    LMIC.gwmargin    = 0;
    LMIC.gwcnt       = 0;
    LMIC.bcnfAns     = 0;
    LMIC.bcnChnl     = 0;
    LMIC.bcnFreq     = 0;
    LMIC.ping.freq   = FREQ_PING;
    LMIC.ping.dr     = DR_PING;
#if defined(CFG_lorawan11)
    if( (LMIC.opts &= OPT_LORAWAN11) ) {
	LMIC.opts |= OPT_OPTNEG;
    }
#endif
}


// ================================================================================
// Decoding frames


// Decode beacon  - do not overwrite bcninfo unless we have a match!
static int decodeBeacon (void) {
    xref2u1_t d = LMIC.frame;
    if( LMIC.dataLen != LEN_BCN || os_rlsbf2(&d[OFF_BCN_CRC1]) != os_crc16(d,OFF_BCN_CRC1) )
        return 0;   // first (common) part fails CRC check

    LMIC.bcninfo.flags &= ~(BCN_PARTIAL|BCN_FULL);
    // Match - update bcninfo structure
    LMIC.bcninfo.snr    = LMIC.snr;
    LMIC.bcninfo.rssi   = LMIC.rssi;
    LMIC.bcninfo.txtime = LMIC.rxtime - AIRTIME_BCN_osticks;
    LMIC.bcninfo.time   = os_rlsbf4(&d[OFF_BCN_TIME]);
    LMIC.bcninfo.flags |= BCN_PARTIAL;

    // Check 2nd set
    if( os_rlsbf2(&d[OFF_BCN_CRC2]) != os_crc16(d,OFF_BCN_CRC2) )
        return 1;
    // Second set of fields is ok
    LMIC.bcninfo.lat    = (s4_t)os_rlsbf4(&d[OFF_BCN_LAT-1]) >> 8; // read as signed 24-bit
    LMIC.bcninfo.lon    = (s4_t)os_rlsbf4(&d[OFF_BCN_LON-1]) >> 8; // ditto
    LMIC.bcninfo.info   = d[OFF_BCN_INFO];
    LMIC.bcninfo.flags |= BCN_FULL;
    return 2;
}


static bit_t decodeFrame (void) {
    xref2u1_t d = LMIC.frame;
    u1_t hdr    = d[0];
    u1_t ftype  = hdr & HDR_FTYPE;
    int  dlen   = LMIC.dataLen;
    if( dlen < OFF_DAT_OPTS+4 ||
        (hdr & HDR_MAJOR) != HDR_MAJOR_V1 ||
        (ftype != HDR_FTYPE_DADN  &&  ftype != HDR_FTYPE_DCDN) ) {
        // Basic sanity checks failed
        EV(specCond, WARN, (e_.reason = EV::specCond_t::UNEXPECTED_FRAME,
                            e_.eui    = MAIN::CDEV->getEui(),
                            e_.info   = dlen < 4 ? 0 : os_rlsbf4(&d[dlen-4]),
                            e_.info2  = hdr + (dlen<<8)));
      norx:
        LMIC.dataLen = 0;
        return 0;
    }
    // Validate exact frame length
    // Note: device address was already read+evaluated in order to arrive here.
    int  fct   = d[OFF_DAT_FCT];
    u4_t addr  = os_rlsbf4(&d[OFF_DAT_ADDR]);
    u4_t seqno = os_rlsbf2(&d[OFF_DAT_SEQNO]);
    int  olen  = fct & FCT_OPTLEN;
    int  ackup = (fct & FCT_ACK) != 0 ? 1 : 0;   // ACK last up frame
    int  poff  = OFF_DAT_OPTS+olen;
    int  pend  = dlen-4;  // MIC

    if( addr != LMIC.devaddr ) {
        EV(specCond, WARN, (e_.reason = EV::specCond_t::ALIEN_ADDRESS,
                            e_.eui    = MAIN::CDEV->getEui(),
                            e_.info   = addr,
                            e_.info2  = LMIC.devaddr));
        goto norx;
    }
    if( poff > pend ) {
        EV(specCond, ERR, (e_.reason = EV::specCond_t::CORRUPTED_FRAME,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = 0x1000000 + (poff-pend) + (fct<<8) + (dlen<<16)));
        goto norx;
    }

    int port = -1;
    int replayConf = 0;

    if( pend > poff )
        port = d[poff++];

    if( port == 0 && olen > 0 )
        goto norx;
    
    u4_t* pseqnoDn;
#if defined(CFG_lorawan11)
    pseqnoDn = (port > 0 && (LMIC.opts & OPT_LORAWAN11))
	? &LMIC.seqnoADn : &LMIC.seqnoDn;
#else
    pseqnoDn = &LMIC.seqnoDn;
#endif
    seqno = *pseqnoDn + (u2_t)(seqno - *pseqnoDn);

    if( !aes_verifyMic(
#if defined(CFG_lorawan11)
		LMIC.nwkKeyDn,
#else
		LMIC.nwkKey,
#endif
		LMIC.devaddr, seqno, /*dn*/1, d, pend) ) {
        EV(spe3Cond, ERR, (e_.reason = EV::spe3Cond_t::CORRUPTED_MIC,
                           e_.eui1   = MAIN::CDEV->getEui(),
                           e_.info1  = Base::lsbf4(&d[pend]),
                           e_.info2  = seqno,
                           e_.info3  = LMIC.devaddr));
        goto norx;
    }
    if( seqno < *pseqnoDn ) {
        if( (s4_t)seqno > (s4_t)*pseqnoDn ) {
            EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_ROLL_OVER,
                                e_.eui    = MAIN::CDEV->getEui(),
                                e_.info   = *pseqnoDn,
                                e_.info2  = seqno));
            goto norx;
        }
        if( seqno != *pseqnoDn-1 || !LMIC.dnConf || ftype != HDR_FTYPE_DCDN ) {
            EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_OBSOLETE,
                                e_.eui    = MAIN::CDEV->getEui(),
                                e_.info   = *pseqnoDn,
                                e_.info2  = seqno));
            goto norx;
        }
        // Replay of previous sequence number allowed only if
        // previous frame and repeated both requested confirmation
        replayConf = 1;
    }
    else {
        if( seqno > *pseqnoDn ) {
            EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_SKIP,
                                e_.eui    = MAIN::CDEV->getEui(),
                                e_.info   = *pseqnoDn,
                                e_.info2  = seqno));
        }
        *pseqnoDn = seqno+1;  // next number to be expected
        DO_DEVDB(*pseqnoDn,seqnoDn); // XXX
    }
    // DN frame requested confirmation - provide ACK once with next UP frame
    LMIC.dnConf = (ftype == HDR_FTYPE_DCDN ? FCT_ACK : 0);

    if( LMIC.dnConf || (fct & FCT_MORE) )
        LMIC.opmode |= OP_POLL;

    // We heard from network
    LMIC.rejoinCnt = 0;
    if( LMIC.adrAckReq != LINK_CHECK_OFF )
        LMIC.adrAckReq = LINK_CHECK_INIT;

    // Process OPTS
    xref2u1_t opts = &d[OFF_DAT_OPTS];
    int oidx = 0;

    if( !replayConf ) {
        // Handle payload only if not a replay
        // Decrypt payload - if any
        if( port >= 0 && pend-poff > 0 )
            aes_cipher(port <= 0 ?
#if defined(CFG_lorawan11)
		       LMIC.nwkKeyDn
#else
		       LMIC.nwkKey
#endif
		       : LMIC.appKey, LMIC.devaddr, seqno, /*dn*/1, d+poff, pend-poff);

        EV(dfinfo, DEBUG, (e_.deveui  = MAIN::CDEV->getEui(),
                           e_.devaddr = LMIC.devaddr,
                           e_.seqno   = seqno,
                           e_.flags   = (port < 0 ? EV::dfinfo_t::NOPORT : 0) | EV::dfinfo_t::DN,
                           e_.mic     = Base::lsbf4(&d[pend]),
                           e_.hdr     = d[LORA::OFF_DAT_HDR],
                           e_.fct     = d[LORA::OFF_DAT_FCT],
                           e_.port    = port,
                           e_.plen    = dlen,
                           e_.opts.length = olen,
                           memcpy(&e_.opts[0], opts, olen)));
    } else {
        EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_REPLAY,
                            e_.eui    = MAIN::CDEV->getEui(),
                            e_.info   = Base::lsbf4(&d[pend]),
                            e_.info2  = seqno));
    }

    if( port == 0 && pend-poff > 0 ) {
	// We process both FOpts and FRMPayload - FPort=0 inbetween is ignored
	olen = pend-OFF_DAT_OPTS;
    }

    if( (LMIC.dn2Ans || LMIC.dn1DlyAns || LMIC.dnfqAns || LMIC.dnfqAnsPend )
	&& !(LMIC.txrxFlags & TXRX_PING) && !((LMIC.clmode & CLASS_C) && (LMIC.txrxFlags & TXRX_DNW2)) ) {
	// Ack of RXParamSetup is very delicate since it might lead to loosing shared state between NWKS/device.
	// The server proves it has seen the RXParamSetupAns from the device by responding in a RX window anchored
	// to the frame that carried the RXParamSetupAns! This means RX from ping slots **and** any class C using RX2
	// is not prove of reception! Thus, keep sending the RXParamSetupAns.
	// Ditto: RXTimingSetup
	LMIC.dn2Ans = LMIC.dn1DlyAns = LMIC.dnfqAns = LMIC.dnfqAnsPend = LMIC.dnfqAcks = 0;
    }
    while( oidx < olen ) {
        switch( opts[oidx] ) {
	case 0:  // FPort=0 if we had MAC commands in payload
	    oidx += 1;
	    continue;

        case MCMD_LCHK_ANS: {
            LMIC.gwmargin = opts[oidx+1];
            LMIC.gwcnt = opts[oidx+2];
            oidx += 3;
            continue;
        }
        case MCMD_LADR_REQ: {
            u1_t p1, chpage, nbtrans, cnt = 0;
            u2_t chmap;
	    u2_t dmap[sizeof(LMIC.channelMap)/sizeof(u2_t)] = { 0 };
	    u1_t ans = 0x80 |     // Include an answer into next frame up
		MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK | MCMD_LADR_ANS_DRACK;
	    do {
		p1      = opts[oidx+1];            		// txpow + DR
		chmap   = os_rlsbf2(&opts[oidx+2]);		// list of enabled channels
		chpage  = opts[oidx+4] & MCMD_LADR_CHPAGE_MASK;	// channel page
		nbtrans = opts[oidx+4] & MCMD_LADR_REPEAT_MASK;	// up repeat count
		oidx += 5;
		cnt  += 1;
		if( !applyChannelMap(chpage, chmap, dmap) ) {
		    ans &= ~MCMD_LADR_ANS_CHACK;
		}
	    } while (oidx < olen && opts[oidx] == MCMD_LADR_REQ);

            if( (ans & MCMD_LADR_ANS_CHACK) && !checkChannelMap(dmap) ) {
                ans &= ~MCMD_LADR_ANS_CHACK;
	    }
	    if( nbtrans == 0 ) {
		nbtrans = LMIC.nbTrans;  // keep unchanged
	    }
            dr_t dr = (dr_t)(((p1 & MCMD_LADR_DR_MASK) >> MCMD_LADR_DR_SHIFT));
	    s1_t powadj = (s1_t)((p1 & MCMD_LADR_POW_MASK) >> MCMD_LADR_POW_SHIFT);
#if 0
	    debug_printf("ADR: p1=%02x,dr=%d,powadj=%d,chmap=%04x,chpage=%d,nbtrans=%d\r\n",
		    p1, dr, powadj, chmap, chpage, nbtrans);
#endif
	    if( dr == 15 ) {
		dr = LMIC.datarate; // Do not change DR
	    } else if( !validDR(dr) ) {
                ans &= ~MCMD_LADR_ANS_DRACK;
                EV(specCond, ERR, (e_.reason = EV::specCond_t::BAD_MAC_CMD,
                                   e_.eui    = MAIN::CDEV->getEui(),
                                   e_.info   = Base::lsbf4(&d[pend]),
                                   e_.info2  = Base::msbf4(&opts[oidx-4])));
            }
            if( (ans & 0x7F) == (MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK | MCMD_LADR_ANS_DRACK) ) {
                // Nothing went wrong - use settings
#if defined(CFG_eu868)
		LMIC.channelMap = *dmap;
#else
		os_copyMem(LMIC.channelMap, dmap, sizeof(dmap));
#endif
                LMIC.nbTrans = nbtrans;
                setDrTxpow(DRCHG_NWKCMD, dr, powadj==15 ? KEEP_TXPOWADJ : powadj2dBm(powadj));
		reportEvent(EV_DATARATE);
            }
	    while( cnt-- > 0 ) {
		LMIC.foptsUp[LMIC.foptsUpLen++] = MCMD_LADR_ANS;
		LMIC.foptsUp[LMIC.foptsUpLen++] = ans & ~MCMD_LADR_ANS_RFU;
	    }
	    syncDatarate();              // fix DR if no more channel allowing this datarate
	    LMIC.opmode |= OP_NEXTCHNL;  // DR might have changed, channel might no longer be available
            continue;
        }
        case MCMD_DEVS_REQ: {
	    LMIC.margin = (LMIC.snr >> 2) & 0x3f;
            LMIC.devsAns = 1;
            oidx += 1;
	    LMIC.opmode |= OP_POLL;
            continue;
        }
        case MCMD_DN2P_SET: {
            u1_t dr = opts[oidx+1] & 0xF;
            u1_t off = (opts[oidx+1] >> 4) & 7;
            s4_t freq = convFreq(&opts[oidx+2]);
            oidx += 5;
            u1_t ans = 0;
            if( validDR(dr) )
                ans |= MCMD_DN2P_ANS_DRACK;
            if( freq > 0 )
                ans |= MCMD_DN2P_ANS_CHACK;
	    if( off==0 || effectiveDn1DrOff[off] != 0 )
                ans |= MCMD_DN2P_ANS_OFFACK;
            if( ans == (MCMD_DN2P_ANS_OFFACK|MCMD_DN2P_ANS_DRACK|MCMD_DN2P_ANS_CHACK) ) {
                LMIC.dn1DrOff = off;
                LMIC.dn2Dr = dr;
                LMIC.dn2Freq = freq;
                DO_DEVDB(LMIC.dn2Dr,dn2Dr);
                DO_DEVDB(LMIC.dn2Freq,dn2Freq);
            }
	    u1_t i = LMIC.foptsUpLen;
	    LMIC.foptsUpLen = i+2;
	    LMIC.foptsUp[i+0] = MCMD_DN2P_ANS;
	    LMIC.foptsUp[i+1] = ans;
	    LMIC.dn2Ans = MCMD_DN2P_ANS_REPLY | ans;   // answer pending
	    LMIC.opmode |= OP_POLL;
            continue;
        }
        case MCMD_DCAP_REQ: {
            u1_t cap = opts[oidx+1];
            oidx += 2;
            // A value cap=0xFF means device is OFF unless enabled again manually.
            if( cap==0xFF )
                LMIC.opmode |= OP_SHUTDOWN;  // stop any sending
            LMIC.globalDutyRate  = cap & 0xF;
            LMIC.globalDutyAvail = os_getTime();
            DO_DEVDB(cap,dutyCap);
            LMIC.dutyCapAns = 1;
            continue;
        }
        case MCMD_SNCH_REQ: {
            u1_t ans = MCMD_SNCH_ANS_PEND;
#if CFG_eu868 // Only available in EU like regions
            u1_t chidx = opts[oidx+1];
            s4_t freq  = convFreq(&opts[oidx+2]);
	    if( chidx < MIN_CHNLS ) {
		// don't allow modification of default channels
	    } else if( freq == 0 && chidx < MAX_CHNLS ) {
		LMIC_disableChannel(chidx);
		ans = MCMD_SNCH_ANS_PEND|MCMD_SNCH_ANS_DRACK|MCMD_SNCH_ANS_FQACK;
	    } else {
		u1_t mindr = opts[oidx+5] & 0xF;
		u1_t maxdr = opts[oidx+5] >> 4;

		if( validDR(mindr) && validDR(maxdr) && mindr <= maxdr &&
		    (chidx >= MIN_CHNLS || (mindr == 0 && maxdr >= DR_MAX125)) )
		    ans |= MCMD_SNCH_ANS_DRACK;
		if( chidx <= MAX_CHNLS && freq >= 0 )
		    ans |= MCMD_SNCH_ANS_FQACK;
		if( ans == (MCMD_SNCH_ANS_PEND|MCMD_SNCH_ANS_DRACK|MCMD_SNCH_ANS_FQACK) )
		    LMIC_setupChannel(chidx, freq, DR_RANGE_MAP(mindr,maxdr), -1);
	    }
#endif
            oidx += 6;
	    LMIC.opmode |= OP_POLL;
	    u1_t i = LMIC.foptsUpLen;
	    LMIC.foptsUpLen = i+2;
	    LMIC.foptsUp[i+0] = MCMD_SNCH_ANS;
	    LMIC.foptsUp[i+1] = ans & ~MCMD_SNCH_ANS_RFU;
            continue;
        }
        case MCMD_DNFQ_REQ: {
#if CFG_eu868 // Only available in EU like regions
            u1_t ans = MCMD_DNFQ_ANS_PEND;
	    u1_t chidx = opts[oidx+1];
	    s4_t freq  = convFreq(&opts[oidx+2]);
	    if( chidx <= MAX_CHNLS && LMIC.channelFreq[chidx] != 0 )
		ans |= MCMD_DNFQ_ANS_CHACK;
	    if( freq >= 0 )   // we allow 0 here to reset to default behavior (aka downlink same as uplink)
		ans |= MCMD_DNFQ_ANS_FQACK;
	    if( ans == (MCMD_DNFQ_ANS_PEND|MCMD_DNFQ_ANS_CHACK|MCMD_DNFQ_ANS_FQACK) )
		LMIC.channelDnFreq[chidx] = freq;
	    if( LMIC.dnfqAns + LMIC.dnfqAnsPend < 16 )
		LMIC.dnfqAcks |= ans << (2*(LMIC.dnfqAns + LMIC.dnfqAnsPend));
	    LMIC.dnfqAns += 1;
	    u1_t i = LMIC.foptsUpLen;
	    LMIC.foptsUpLen = i+2;
	    LMIC.foptsUp[i+0] = MCMD_DNFQ_ANS;
	    LMIC.foptsUp[i+1] = ans & ~MCMD_DNFQ_ANS_RFU;
#endif
            oidx += 5;
	    LMIC.opmode |= OP_POLL;
            continue;
	}
	case MCMD_RXTM_REQ: {
	    LMIC.dn1Dly = opts[oidx+1] & 0xF;
	    if( LMIC.dn1Dly == 0 )
		LMIC.dn1Dly = 1;
	    LMIC.dn1DlyAns = 0x80;
	    LMIC.opmode |= OP_POLL;
	    oidx += 2;
	    continue;
	}
        case MCMD_PITV_ANS: {
	    if( (LMIC.ping.intvExp & 0x80) ) {
		LMIC.ping.intvExp &= 0x7F;   // clear pending bit
		LMIC.opmode |= OP_PINGABLE;
	    } // else: ignore if we weren't waiting for it
	    oidx += 1;
	    continue;
	}
        case MCMD_PNGC_REQ: {
            s4_t freq = convFreq(&opts[oidx+1]);
	    u1_t dr = opts[oidx+4] & 0xF;
            oidx += 5;
	    u1_t ans = 0;
	    if( validDR(dr) )
		ans |= MCMD_PNGC_ANS_DRACK;
	    if( freq >= 0)
		ans |= MCMD_PNGC_ANS_FQACK;
	    if( ans == (MCMD_PNGC_ANS_FQACK|MCMD_PNGC_ANS_DRACK) ) {
		LMIC.ping.freq = freq==0 ? FREQ_PING : freq;
		LMIC.ping.dr = dr;
	    }
	    u1_t i = LMIC.foptsUpLen;
	    LMIC.foptsUpLen = i+2;
	    LMIC.foptsUp[i+0] = MCMD_PNGC_ANS;
	    LMIC.foptsUp[i+1] = ans;
            continue;
        }
        case MCMD_TIME_ANS: {
	    u4_t secs = os_rlsbf4(&opts[oidx+1]);
	    u1_t frac = opts[oidx+5];
	    osxtime_t ref = os_time2XTime(LMIC.txend, os_getXTime());
	    LMIC.gpsEpochOff = secs * OSTICKS_PER_SEC + (((frac * OSTICKS_PER_SEC) >> 8) + 128) - ref;
	    LMIC.askForTime = 0;   // stop asking for time
	    oidx += 6;
	    // Currently, we only ask for time when we want to track a beacon
	    // If there are other reasons for getting MCMD_TIME_ANS we have to discern them here
	    // Set up tracking of next beacon based on the obtained time:
	    // Accuracy error: 1/512 sec = ~2ms - spec promises +/-100ms
	    LMIC.bcninfo.txtime = LMIC.txend - (secs & 0x7F) * OSTICKS_PER_SEC - (((frac * OSTICKS_PER_SEC) + 128) >> 8);
	    LMIC.bcninfo.flags = 0;  // no previous beacon as reference (BCN_PARTIAL|BCN_FULL cleared)
	    calcBcnRxWindowFromMillis(100,1);
	    LMIC.bcnChnl = (1+(secs >> 7)) % NCHNL_BCN;
	    LMIC.opmode = (LMIC.opmode & ~OP_SCAN) | OP_TRACK;
	    continue;
	}
        case MCMD_BCNI_ANS: {
            // Ignore if tracking already enabled
            if( (LMIC.opmode & OP_TRACK) == 0 ) {
                LMIC.bcnChnl = opts[oidx+3];
                // Disable tracking
                LMIC.opmode |= OP_TRACK;
                // Cleared later in txComplete handling - triggers EV_BEACON_FOUND
                ASSERT(LMIC.askForTime!=0);
                // Setup RX parameters
                LMIC.bcninfo.txtime = (LMIC.rxtime
                                       + ms2osticks(os_rlsbf2(&opts[oidx+1]) * MCMD_BCNI_TUNIT)
                                       + ms2osticksCeil(MCMD_BCNI_TUNIT/2)
                                       - BCN_INTV_osticks);
                LMIC.bcninfo.flags = 0;  // txtime above cannot be used as reference (BCN_PARTIAL|BCN_FULL cleared)
                calcBcnRxWindowFromMillis(MCMD_BCNI_TUNIT,1);  // error of +/-N ms

                EV(lostFrame, INFO, (e_.reason  = EV::lostFrame_t::MCMD_BCNI_ANS,
                                     e_.eui     = MAIN::CDEV->getEui(),
                                     e_.lostmic = Base::lsbf4(&d[pend]),
                                     e_.info    = (LMIC.missedBcns |
                                                   (osticks2us(LMIC.bcninfo.txtime + BCN_INTV_osticks
                                                               - LMIC.bcnRxtime) << 8)),
                                     e_.time    = MAIN::CDEV->ostime2ustime(LMIC.bcninfo.txtime + BCN_INTV_osticks)));
            }
            oidx += 4;
            continue;
        }
	case MCMD_BCNF_REQ: {
            s4_t freq = convFreq(&opts[oidx+1]);
	    u1_t ans = MCMD_BCNF_ANS_PEND;
	    if( freq >= 0 )
		ans |= MCMD_BCNF_ANS_FQACK;
	    LMIC.bcnfAns = ans;
	    if( ans == (MCMD_BCNF_ANS_PEND | MCMD_BCNF_ANS_FQACK) )
		LMIC.bcnFreq = freq;
	    oidx += 4;
	    continue;
	}
        case MCMD_ADRP_REQ: {
	    LMIC_setLinkCheck(1 << (opts[oidx+1] >> 4), 1 << (opts[oidx+1] & 0xf));
	    LMIC.foptsUp[LMIC.foptsUpLen++] = MCMD_ADRP_ANS;
	    oidx += 2;
	    continue;
	}
#if defined(CFG_lorawan11)
        case MCMD_RKEY_CNF: {
	    // Ignore if we did not ask for options negotiation
	    if( LMIC.opts & OPT_OPTNEG ) {
		LMIC.opts = OPT_LORAWAN11;
	    }
            oidx += 2 + (opts[oidx+1] >> 4);
	    continue;
	}
	case MCMD_DEVMD_CONF: {
	    if( (LMIC.clmode & PEND_CLASS_C) && opts[oidx+1] == ((LMIC.clmode & CLASS_C) ? 0 : 2) ) {
		decPollcnt();
		LMIC.clmode &= ~PEND_CLASS_C;
		LMIC.clmode ^= CLASS_C;
	    } // else: unexpected confirm or unexpected class -- ignore
	    oidx += 2;
	    continue;
	}
#endif
        }
        EV(specCond, ERR, (e_.reason = EV::specCond_t::BAD_MAC_CMD,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = Base::lsbf4(&d[pend]),
                           e_.info2  = Base::msbf4(&opts[oidx])));
        break;
    }
    if( oidx != olen ) {
        EV(specCond, ERR, (e_.reason = EV::specCond_t::CORRUPTED_FRAME,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = 0x1000000 + (oidx) + (olen<<8)));
    }

#if defined(CFG_lorawan11)
    if( LMIC.opts & OPT_OPTNEG ) {
	// Don't keep asking for options negotiation if the LNS does not want to answer us
	LMIC.opts &= ~OPT_OPTNEG;
    }
#endif

    if( // NWK acks but we don't have a frame pending
        (ackup && LMIC.pendTxConf == 0) ||
        // We sent up confirmed and we got a response in DNW1/DNW2
        // BUT it did not carry an ACK - this should never happen
        // Do not resend and assume frame was not ACKed.
        (!ackup && LMIC.pendTxConf != 0) ) {
        EV(specCond, ERR, (e_.reason = EV::specCond_t::SPURIOUS_ACK,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = seqno,
                           e_.info2  = ackup));
    }

    if( LMIC.pendTxConf != 0 ) { // we requested an ACK
        LMIC.txrxFlags |= ackup ? TXRX_ACK : TXRX_NACK;
	LMIC.pendTxConf = 0;
    }

    if( port < 0 ) {
        LMIC.txrxFlags |= TXRX_NOPORT;
        LMIC.dataBeg = poff;
        LMIC.dataLen = 0;
    } else {
        LMIC.txrxFlags |= TXRX_PORT;
        LMIC.dataBeg = poff;
        LMIC.dataLen = pend-poff;
    }
    return 1;
}


static int decodeMultiCastFrame (void) {
    xref2u1_t d = LMIC.frame;
    u1_t hdr    = d[0];
    u1_t ftype  = hdr & HDR_FTYPE;
    int  dlen   = LMIC.dataLen;
    if( dlen < OFF_DAT_OPTS+4 ||
        (hdr & HDR_MAJOR) != HDR_MAJOR_V1 ||
	ftype != HDR_FTYPE_DADN ) {
        // Basic sanity checks failed
        EV(specCond, WARN, (e_.reason = EV::specCond_t::UNEXPECTED_FRAME,
                            e_.eui    = MAIN::CDEV->getEui(),
                            e_.info   = dlen < 4 ? 0 : os_rlsbf4(&d[dlen-4]),
                            e_.info2  = hdr + (dlen<<8)));
      norx:
        LMIC.dataLen = 0;
        return 0;
    }
    // Validate exact frame length
    int  fct   = d[OFF_DAT_FCT];
    u4_t addr  = os_rlsbf4(&d[OFF_DAT_ADDR]);
    u4_t seqno = os_rlsbf2(&d[OFF_DAT_SEQNO]);
    int  poff  = OFF_DAT_OPTS;
    int  pend  = dlen-4;  // MIC

    // check for multicast session with this address
    session_t* s;
    for(s = LMIC.sessions; s<LMIC.sessions+MAX_MULTICAST_SESSIONS && s->grpaddr!=addr; s++);
    if( s == LMIC.sessions+MAX_MULTICAST_SESSIONS ) {
        EV(specCond, WARN, (e_.reason = EV::specCond_t::ALIEN_ADDRESS,
                            e_.eui    = MAIN::CDEV->getEui(),
                            e_.info   = addr,
                            e_.info2  = LMIC.devaddr));
        goto norx;
    }
    // check for short frame
    if( poff > pend ) {
        EV(specCond, ERR, (e_.reason = EV::specCond_t::CORRUPTED_FRAME,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = 0x1000000 + (poff-pend) + (fct<<8) + (dlen<<16)));
        goto norx;
    }
    // check for port
    int port = -1;
    if( pend > poff ) {
        port = d[poff++];
    }
    if( port == 0 ) {
        EV(specCond, ERR, (e_.reason = EV::specCond_t::BAD_PORT,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = 0));
        goto norx;
    }

    // check for bad flags or options (only FPending allowed, no options)
    if( fct & ~FCT_MORE ) {
        EV(specCond, ERR, (e_.reason = EV::specCond_t::CORRUPTED_FRAME,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = 0x1000000 + fct));
        goto norx;
    }

    seqno = s->seqnoADn + (u2_t)(seqno - s->seqnoADn);

    // verify MIC
    if( !aes_verifyMic(s->nwkKeyDn, s->grpaddr, seqno, /*dn*/1, d, pend) ) {
        EV(spe3Cond, ERR, (e_.reason = EV::spe3Cond_t::CORRUPTED_MIC,
                           e_.eui1   = MAIN::CDEV->getEui(),
                           e_.info1  = Base::lsbf4(&d[pend]),
                           e_.info2  = seqno,
                           e_.info3  = s->grpaddr));
        goto norx;
    }
    // check down frame counter
    if( seqno < s->seqnoADn ) {
	EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_OBSOLETE,
			    e_.eui    = MAIN::CDEV->getEui(),
			    e_.info   = s->seqnoADn,
			    e_.info2  = seqno));
	goto norx;
    }
    if( seqno > s->seqnoADn ) {
	EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_SKIP,
			    e_.eui    = MAIN::CDEV->getEui(),
			    e_.info   = s->seqnoADn,
			    e_.info2  = seqno));
    }
    s->seqnoADn = seqno+1;  // next number to be expected

    // We heard from network
    LMIC.rejoinCnt = 0;
    if( LMIC.adrAckReq != LINK_CHECK_OFF )
        LMIC.adrAckReq = LINK_CHECK_INIT;

    // Decrypt payload - if any
    if( pend-poff > 0 ) {
	aes_cipher(s->appKey, s->grpaddr, seqno, /*dn*/1, d+poff, pend-poff);

        EV(dfinfo, DEBUG, (e_.deveui  = MAIN::CDEV->getEui(),
                           e_.devaddr = LMIC.devaddr,
                           e_.seqno   = seqno,
                           e_.flags   = (port < 0 ? EV::dfinfo_t::NOPORT : 0) | EV::dfinfo_t::DN,
                           e_.mic     = Base::lsbf4(&d[pend]),
                           e_.hdr     = d[LORA::OFF_DAT_HDR],
                           e_.fct     = d[LORA::OFF_DAT_FCT],
                           e_.port    = port,
                           e_.plen    = dlen,
                           e_.opts.length = olen,
                           memcpy(&e_.opts[0], opts, olen)));
    }

    if( port < 0 ) {
        LMIC.txrxFlags |= TXRX_NOPORT;
        LMIC.dataBeg = poff;
        LMIC.dataLen = 0;
    } else {
        LMIC.txrxFlags |= TXRX_PORT;
        LMIC.dataBeg = poff;
        LMIC.dataLen = pend-poff;
    }
    return 1;
}


// ================================================================================
// TX/RX transaction support


static void setupRx2 (void) {
    LMIC.txrxFlags = TXRX_DNW2;
    LMIC.rps = dndr2rps(LMIC.dn2Dr);
    LMIC.freq = LMIC.dn2Freq;
    LMIC.dataLen = 0;
    os_radio(RADIO_RX);
}


static void schedRx2 (u1_t delay, osjobcb_t func) {
    // Add 1.5 symbols we need 5 out of 8. Try to sync 1.5 symbols into the preamble.
    LMIC.rxtime = LMIC.txend + delay*sec2osticks(1) + dr2hsym(LMIC.dn2Dr, PAMBL_SYMS-MINRX_SYMS);
    adjustByRxdErr(delay, LMIC.dn2Dr);
    os_setTimedCallback(&LMIC.osjob, LMIC.rxtime - RX_RAMPUP, func);
}

static void setupRx1 (osjobcb_t func) {
    LMIC.txrxFlags = TXRX_DNW1;
    // Turn LMIC.rps from TX over to RX
    LMIC.rps = setNocrc(LMIC.rps,1);
    LMIC.dataLen = 0;
    LMIC.osjob.func = func;
    os_radio(RADIO_RX);
}


// Called by HAL once TX complete and delivers exact end of TX time stamp in LMIC.rxtime
static void txDone (u1_t delay, osjobcb_t func) {
    if( (LMIC.opmode & (OP_TRACK|OP_PINGABLE|OP_PINGINI)) == (OP_TRACK|OP_PINGABLE) ) {
        rxschedInit(&LMIC.ping);    // note: reuses LMIC.frame buffer!
        LMIC.opmode |= OP_PINGINI;
    }
#if defined(CFG_cn470)
    // Calculate RX frequency / rps (US like only) before we increment txChnl
    LMIC.freq = CN470_125kHz_DNFBASE + (LMIC.txChnl % 48) * CN470_125kHz_DNFSTEP;
#elif defined(CFG_us915)
    // Calculate RX frequency / rps (US like only) before we increment txChnl
    LMIC.freq = US915_500kHz_DNFBASE + (LMIC.txChnl & 0x7) * US915_500kHz_DNFSTEP;
#endif
#if defined(CFG_eu868)
    // Check DN link freq for EU like regions
    u4_t dnfreq = LMIC.channelDnFreq[LMIC.txChnl];
    if( dnfreq ) // different from uplink freq
	LMIC.freq = dnfreq;
#endif
    LMIC.dndr = updr2dndr(LMIC.dndr, LMIC.dn1DrOff);
    LMIC.rps  = dndr2rps(LMIC.dndr);
#if defined(CFG_eu868) && !defined(CFG_kr920)
    if( /* TX datarate */LMIC.rxsyms == DR_FSK ) {
        LMIC.rxtime = LMIC.txend + delay*sec2osticks(1) - PRERX_FSK*us2osticksRound(160);
        LMIC.rxsyms = RXLEN_FSK;
    }
    else
#endif
    {
        LMIC.rxtime = LMIC.txend + delay*sec2osticks(1) + dr2hsym(LMIC.dndr, PAMBL_SYMS-MINRX_SYMS);
        LMIC.rxsyms = MINRX_SYMS;
	adjustByRxdErr(delay, LMIC.dndr);
    }
    os_setTimedCallback(&LMIC.osjob, LMIC.rxtime - RX_RAMPUP, func);
}


// ======================================== Join frames


static void onJoinFailed (xref2osjob_t osjob) {
    // Notify app - must call LMIC_reset() to stop joining
    // otherwise join procedure continues.
    reportEvent(EV_JOIN_FAILED);
}


static bit_t processJoinAccept (void) {
    ASSERT(LMIC.txrxFlags != TXRX_DNW1 || LMIC.dataLen != 0);
    ASSERT((LMIC.opmode & OP_TXRXPEND)!=0);

    if( LMIC.dataLen == 0 ) {
      nojoinframe:
        if( (LMIC.opmode & OP_JOINING) == 0 ) {
            ASSERT((LMIC.opmode & OP_REJOIN) != 0);
            // REJOIN attempt for roaming
            LMIC.opmode &= ~(OP_REJOIN|OP_TXRXPEND);
            if( LMIC.rejoinCnt < 10 )
                LMIC.rejoinCnt++;
            reportEvent(EV_REJOIN_FAILED);
            return 1;
        }
        LMIC.opmode &= ~OP_TXRXPEND;
        ostime_t delay = nextJoinState();
        EV(devCond, DEBUG, (e_.reason = EV::devCond_t::NO_JACC,
                            e_.eui    = MAIN::CDEV->getEui(),
                            e_.info   = LMIC.datarate|DR_PAGE,
                            e_.info2  = osticks2ms(delay)));
        // Build next JOIN REQUEST with next engineUpdate call
        // Optionally, report join failed.
        // Both after a random/chosen amount of ticks.
        os_setApproxTimedCallback(&LMIC.osjob, os_getTime()+delay,
                            (delay&1) != 0
                            ? FUNC_ADDR(onJoinFailed)      // one JOIN iteration done and failed
                            : FUNC_ADDR(runEngineUpdate)); // next step to be delayed
        return 1;
    }
    u1_t hdr  = LMIC.frame[0];
    u1_t dlen = LMIC.dataLen;
    //u4_t mic  = os_rlsbf4(&LMIC.frame[dlen-4]); // save before modified by encrypt!
    if( (dlen != LEN_JA && dlen != LEN_JAEXT)
        || (hdr & (HDR_FTYPE|HDR_MAJOR)) != (HDR_FTYPE_JACC|HDR_MAJOR_V1) ) {
        EV(specCond, ERR, (e_.reason = EV::specCond_t::UNEXPECTED_FRAME,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = dlen < 4 ? 0 : mic,
                           e_.info2  = hdr + (dlen<<8)));
      badframe:
        if( (LMIC.txrxFlags & TXRX_DNW1) != 0 )
            return 0;
        goto nojoinframe;
    }
    aes_encrypt(LMIC.frame+1, dlen-1);
    if( !aes_verifyMic0(LMIC.frame, dlen-4) ) {
        EV(specCond, ERR, (e_.reason = EV::specCond_t::JOIN_BAD_MIC,
                           e_.info   = mic));
        goto badframe;
    }

    u4_t addr = os_rlsbf4(LMIC.frame+OFF_JA_DEVADDR);
    LMIC.devaddr  = addr;
    LMIC.netid    = os_rlsbf4(&LMIC.frame[OFF_JA_NETID]) & 0xFFFFFF;
    LMIC.dn1Dly   = os_minmax(1, LMIC.frame[OFF_JA_RXDLY] & 0xF, 15);
    LMIC.dn1DrOff = effectiveDn1DrOff[(LMIC.frame[OFF_JA_DLSET] & JA_DLS_RX1DROFF) >> 4];  // illegal values map to 0
    LMIC.dn2Dr    = LMIC.frame[OFF_JA_DLSET] & JA_DLS_RX2DR;
#if defined(CFG_lorawan11)
    LMIC.opts     = (LMIC.frame[OFF_JA_DLSET] & JA_DLS_OPTNEG) ? OPT_LORAWAN11 : 0;
#endif

#if defined(CFG_eu868)
    initDefaultChannels(0);
#endif
    if( dlen > LEN_JA ) {
#if defined(CFG_eu868)
	dlen = OFF_CFLIST;
	for( u1_t chidx=3; chidx<8; chidx++, dlen+=3 ) {
	    s4_t freq = convFreq(&LMIC.frame[dlen]);
	    if( freq > 0 )
		LMIC_setupChannel(chidx, freq, 0, -1);
	}
#elif defined(CFG_us915)
	if (LMIC.frame[OFF_CFLIST + 15] != 1) { // must be CFList type 1
	    goto badframe;
	}
	dlen = OFF_CFLIST;
	for (u1_t i = 0; i < 4; i++, dlen += 2) {
	    LMIC.channelMap[i] = os_rlsbf2(&LMIC.frame[dlen]);
	}
	LMIC.channelMap[4] = LMIC.frame[dlen];
#else
	goto badframe;
#endif
    }

    // already incremented when JOIN REQ got sent off
    aes_sessKeys(LMIC.devNonce-1, &LMIC.frame[OFF_JA_JOINNONCE], LMIC.nwkKey,
#if defined(CFG_lorawan11)
	    LMIC.nwkKeyDn,
#endif
	    LMIC.appKey);
    DO_DEVDB(LMIC.netid,   netid);
    DO_DEVDB(LMIC.devaddr, devaddr);
    DO_DEVDB(LMIC.nwkKey,  nwkkey);
    DO_DEVDB(LMIC.appKey,  artkey);

    EV(joininfo, INFO, (e_.arteui  = MAIN::CDEV->getArtEui(),
                        e_.deveui  = MAIN::CDEV->getEui(),
                        e_.devaddr = LMIC.devaddr,
                        e_.oldaddr = oldaddr,
                        e_.nonce   = LMIC.devNonce-1,
                        e_.mic     = mic,
                        e_.reason  = ((LMIC.opmode & OP_REJOIN) != 0
                                      ? EV::joininfo_t::REJOIN_ACCEPT
                                      : EV::joininfo_t::ACCEPT)));

    ASSERT((LMIC.opmode & (OP_JOINING|OP_REJOIN))!=0);
    if( (LMIC.opmode & OP_REJOIN) != 0 ) {
        // Lower DR every try below current UP DR
        LMIC.datarate = lowerDR(LMIC.datarate, LMIC.rejoinCnt);
    }
    addRxdErr(DELAY_JACC1 + (LMIC.txrxFlags & TXRX_DNW2 ? DELAY_EXTDNW2 : 0));
    stateJustJoined();
    reportEvent(EV_JOINED);
    return 1;
}


static void processRx2Jacc (xref2osjob_t osjob) {
    if( LMIC.dataLen == 0 )
        LMIC.txrxFlags = 0;  // nothing in 1st/2nd DN slot
    processJoinAccept();
}


static void setupRx2Jacc (xref2osjob_t osjob) {
    LMIC.osjob.func = FUNC_ADDR(processRx2Jacc);
    setupRx2();
}


static void processRx1Jacc (xref2osjob_t osjob) {
    if( LMIC.dataLen == 0 || !processJoinAccept() )
        schedRx2(DELAY_JACC2, FUNC_ADDR(setupRx2Jacc));
}


static void setupRx1Jacc (xref2osjob_t osjob) {
    setupRx1(FUNC_ADDR(processRx1Jacc));
}


static void jreqDone (xref2osjob_t osjob) {
    txDone(DELAY_JACC1, FUNC_ADDR(setupRx1Jacc));
    reportEvent(EV_TXDONE);
}

// ======================================== Data frames

// Fwd decl.
static bit_t processDnData(void);

static void processRx2DnDataDelay (xref2osjob_t osjob) {
    processDnData();
}

static void processRx2DnData (xref2osjob_t osjob) {
    if( LMIC.dataLen == 0 ) {
        LMIC.txrxFlags = 0;  // nothing in 1st/2nd DN slot
        // Delay callback processing to avoid up TX while gateway is txing our missed frame!
        // Since DNW2 uses SF12 by default we wait 3 secs.
        os_setTimedCallback(&LMIC.osjob,
                            (os_getTime() + DNW2_SAFETY_ZONE + rndDelay(2)),
                            FUNC_ADDR(processRx2DnDataDelay));
        return;
    }
    processDnData();
}


static void setupRx2DnData (xref2osjob_t osjob) {
    LMIC.osjob.func = FUNC_ADDR(processRx2DnData);
    setupRx2();
}


static void processRx1DnData (xref2osjob_t osjob) {
    if( LMIC.dataLen == 0 || !processDnData() ) {
        schedRx2(LMIC.dn1Dly+DELAY_EXTDNW2, FUNC_ADDR(setupRx2DnData));
    }
}

static void processRx2ClassC (xref2osjob_t osjob) {
    if( LMIC.dataLen != 0 ) {
        LMIC.txrxFlags = TXRX_DNW2;
        if( decodeFrame() ) {
            reportEvent(EV_RXCOMPLETE);
            return;
        }
    }
    engineUpdate();
}

static void setupRx2ClassC () {
    LMIC.osjob.func = FUNC_ADDR(processRx2ClassC);
    LMIC.txrxFlags = TXRX_DNW2;
    LMIC.rps = dndr2rps(LMIC.dn2Dr);
    LMIC.freq = LMIC.dn2Freq;
    LMIC.dataLen = 0;
    os_radio(RADIO_RXON);
}

static void processRx1ClassC (xref2osjob_t osjob) {
    if( !processDnData() ) {
	setupRx2ClassC();
    }
}

static void setupRx1DnData (xref2osjob_t osjob) {
    setupRx1(FUNC_ADDR(processRx1DnData));
}

static void setupRx1ClassC (xref2osjob_t osjob) {
    setupRx1(FUNC_ADDR(processRx1ClassC));
}


static void updataDone (xref2osjob_t osjob) {
    reportEvent(EV_TXDONE);
    if( LMIC.pendTxNoRx ) {
	// transaction done
        LMIC.opmode &= ~(OP_TXDATA|OP_TXRXPEND);
        LMIC.txrxFlags = TXRX_NOPORT;
        LMIC.dataBeg = 0;
        LMIC.dataLen = 0;
	LMIC.pendTxNoRx = 0;
	reportEvent(EV_TXCOMPLETE);
    } else {
	// schedule down window1 reception
	txDone(LMIC.dn1Dly,
	       (LMIC.clmode & CLASS_C) == 0
	       ? FUNC_ADDR(setupRx1DnData)
	       : FUNC_ADDR(setupRx1ClassC));
    }
}

// ========================================


static void buildDataFrame (void) {
    bit_t txdata = ((LMIC.opmode & (OP_TXDATA|OP_POLL)) != OP_POLL);
    u1_t dlen = txdata ? LMIC.pendTxLen : 0;

    // Piggyback MAC options
    // Prioritize by importance
    int  end = OFF_DAT_OPTS;
#if defined(CFG_lorawan11)
    if( LMIC.opts & OPT_OPTNEG ) {
        LMIC.frame[end+0] = MCMD_RKEY_IND;
        LMIC.frame[end+1] = MCMD_RKEY_VERSION_1_1;
	end += 2;
    }
    if( (LMIC.clmode & PEND_CLASS_C) ) {
        LMIC.frame[end+0] = MCMD_DEVMD_IND;
        LMIC.frame[end+1] = (LMIC.clmode & CLASS_C) ? 0 : 2;
	end += 2;
    }
#endif
    if( LMIC.foptsUpLen ) {
	u1_t n = LMIC.foptsUpLen;
	LMIC.foptsUpLen = 0;
	if( end <= OFF_DAT_OPTS+15-n ) {
	    os_copyMem(&LMIC.frame[end], LMIC.foptsUp, n);
	    end += n;
	}
    }
    if( LMIC.ping.intvExp & 0x80 ) {
        // Announce ping interval - LNS hasn't acked it yet
        LMIC.frame[end] = MCMD_PITV_REQ;
        LMIC.frame[end+1] = LMIC.ping.intvExp & 0x7;
        end += 2;
    }
    if( LMIC.dutyCapAns ) {
	if( end <= OFF_DAT_OPTS + 15 - 1 )
	    LMIC.frame[end] = MCMD_DCAP_ANS;
        LMIC.dutyCapAns = 0;
        end += 1;
    }
    if( LMIC.dn2Ans ) {
	// Note: this is cleared with reception of a frame in a class A RX1/RX2 window
	if( (LMIC.dn2Ans & MCMD_DN2P_ANS_PEND) && end <= OFF_DAT_OPTS + 15 - 2 ) {
	    LMIC.frame[end+0] = MCMD_DN2P_ANS;
	    LMIC.frame[end+1] = LMIC.dn2Ans & ~MCMD_DN2P_ANS_RFU;
	    end += 2;
	} else {
	    LMIC.dn2Ans ^= MCMD_DN2P_ANS_REPLY|MCMD_DN2P_ANS_PEND;
	}
    }
    if( LMIC.dn1DlyAns && end <= OFF_DAT_OPTS + 15 - 1 ) {
	// Note: this is cleared with reception of a frame in a class A RX1/RX2 window
        LMIC.frame[end+0] = MCMD_RXTM_ANS;
        end += 1;
    }
    if( LMIC.dnfqAns || LMIC.dnfqAnsPend ) {
	// Note: this is cleared with reception of a frame in a class A RX1/RX2 window
	for( u1_t i=0; i < LMIC.dnfqAnsPend && end <= OFF_DAT_OPTS + 15 - 2; i++ ) {
	    LMIC.frame[end+0] = MCMD_DNFQ_ANS;
	    LMIC.frame[end+1] = (LMIC.dnfqAcks >> (2*i)) & 3;
	    end += 2;
	}
	LMIC.dnfqAnsPend += LMIC.dnfqAns;
	LMIC.dnfqAns = 0;
    }
    if( LMIC.devsAns ) {  // answer to device status
	if( end <= OFF_DAT_OPTS + 15 - 3 ) {
	    LMIC.frame[end+0] = MCMD_DEVS_ANS;
	    LMIC.frame[end+1] = os_getBattLevel();
	    LMIC.frame[end+2] = LMIC.margin;
	}
	LMIC.devsAns = 0;
        end += 3;
    }
    if( LMIC.askForTime > 0 ) {
        LMIC.frame[end] = MCMD_TIME_REQ;
        end += 1;
    }
    if( LMIC.bcnfAns ) {
        LMIC.frame[end+0] = MCMD_BCNF_ANS;
        LMIC.frame[end+1] = LMIC.bcnfAns;
	LMIC.bcnfAns = 0;
        end += 2;
    }
    if( LMIC.gwmargin == 255 ) {
        LMIC.frame[end] = MCMD_LCHK_REQ;
        end += 1;
    }
    ASSERT(end-OFF_DAT_OPTS <= 15);

    // XXX: if( end - OFF_DAT_OPTS > 15 ) ... send as MAC frame with port=0
    u1_t flen = end + (txdata ? 5+dlen : 4);
    if( flen > MAX_LEN_FRAME ) {
        // Options and payload too big - delay payload
        txdata = 0;
        flen = end+4;
    }
    LMIC.frame[OFF_DAT_HDR] = HDR_FTYPE_DAUP | HDR_MAJOR_V1;
    LMIC.frame[OFF_DAT_FCT] = (LMIC.dnConf | (LMIC.pendTxNoRx ? 0 : LMIC.adrEnabled
			       | ((LMIC.adrAckReq >= 0 && (LMIC.opmode & OP_LINKDEAD) == 0) ? FCT_ADRARQ : 0))
			       | ((LMIC.opmode & (OP_TRACK|OP_PINGABLE)) == (OP_TRACK|OP_PINGABLE) ? FCT_CLASSB : 0)
			       | (end-OFF_DAT_OPTS));
    os_wlsbf4(LMIC.frame+OFF_DAT_ADDR,  LMIC.devaddr);

    if( LMIC.txCnt == 0 || (LMIC.opmode & (OP_TXDATA|OP_POLL)) == OP_POLL ) {
	LMIC.txCnt = 0;
        LMIC.seqnoUp += 1;
        DO_DEVDB(LMIC.seqnoUp,seqnoUp);
    }
    os_wlsbf2(LMIC.frame+OFF_DAT_SEQNO, LMIC.seqnoUp-1);

    // Clear pending DN confirmation
    LMIC.dnConf = 0;

    if( txdata ) {
        if( LMIC.pendTxConf ) {
            // Confirmed only makes sense if we have a payload (or at least a port)
            LMIC.frame[OFF_DAT_HDR] = HDR_FTYPE_DCUP | HDR_MAJOR_V1;
        }
        LMIC.frame[end] = LMIC.pendTxPort;
        os_copyMem(LMIC.frame+end+1, LMIC.pendTxData, dlen);
        aes_cipher(LMIC.pendTxPort==0 ? LMIC.nwkKey : LMIC.appKey,
                   LMIC.devaddr, LMIC.seqnoUp-1,
                   /*up*/0, LMIC.frame+end+1, dlen);
    }
    aes_appendMic(LMIC.nwkKey, LMIC.devaddr, LMIC.seqnoUp-1, /*up*/0, LMIC.frame, flen-4);

    EV(dfinfo, DEBUG, (e_.deveui  = MAIN::CDEV->getEui(),
                       e_.devaddr = LMIC.devaddr,
                       e_.seqno   = LMIC.seqnoUp-1,
                       e_.flags   = (LMIC.pendTxPort < 0 ? EV::dfinfo_t::NOPORT : EV::dfinfo_t::NOP),
                       e_.mic     = Base::lsbf4(&LMIC.frame[flen-4]),
                       e_.hdr     = LMIC.frame[LORA::OFF_DAT_HDR],
                       e_.fct     = LMIC.frame[LORA::OFF_DAT_FCT],
                       e_.port    = LMIC.pendTxPort,
                       e_.plen    = txdata ? dlen : 0,
                       e_.opts.length = end-LORA::OFF_DAT_OPTS,
                       memcpy(&e_.opts[0], LMIC.frame+LORA::OFF_DAT_OPTS, end-LORA::OFF_DAT_OPTS)));
    LMIC.dataLen = flen;
}


// Callback from HAL during scan mode or when job timer expires.
static void onBcnScanRx (xref2osjob_t job) {
    // stop radio and its job
    os_radio(RADIO_RST);
    if( decodeBeacon() ) {
	// Found our 1st beacon
	// We don't have a previous beacon to calc some drift - assume some max drift
	calcBcnRxWindowFromMillis(BCN_100PPM_ms,1);
	LMIC.opmode = (LMIC.opmode & ~OP_SCAN) | OP_TRACK;
	reportEvent(EV_BEACON_FOUND);    // can be disabled in callback
	return;
    }
    if( (os_getTime() - LMIC.bcninfo.txtime) >= 0 ) {
        LMIC.opmode &= ~(OP_SCAN | OP_TRACK);
        reportEvent(EV_SCAN_TIMEOUT);
        return;
    }
    engineUpdate();
}


// Enable receiver to listen to incoming beacons
// This mode ends with events: EV_SCAN_TIMEOUT/EV_SCAN_BEACON
// Implicitely cancels any pending TX/RX transaction.
// Also cancels an onpoing joining procedure.
static void startScan (void) {
    if( (LMIC.opmode & (OP_TRACK|OP_SHUTDOWN)) != 0 )
        return;  // already tracking a beacon or shutting down
    // Set scan timeout - one sec longer than beacon period
    LMIC.bcninfo.txtime = os_getTime() + sec2osticks(BCN_INTV_sec+1);
    LMIC.opmode |= OP_SCAN;
    LMIC.askForTime = 0;
}


u1_t LMIC_enableTracking (u1_t tryAskForTime) {
    if( (LMIC.opmode & (OP_SCAN|OP_SHUTDOWN)) != 0 )
        return 0;  // already scanning or shutdown in progress - ignore
    if( (LMIC.opmode & OP_TRACK) != 0 )
	return 2;  // already tracking a beacon - we're done
    // If BCN info requested from NWK then app has to take care
    // of sending data up so that MCMD_BCNI_REQ can be attached.
    LMIC.missedBcns = 0;
    LMIC.askForTime = tryAskForTime;
    if( tryAskForTime == 0 ) {
        startScan();
	reportEvent(EV_START_SCAN);
	return 1;
    }
    LMIC.opmode |= OP_POLL;
    engineUpdate();
    return 1;  // enabled
}


void LMIC_disableTracking (void) {
    LMIC.opmode &= ~(OP_SCAN|OP_TRACK);
    LMIC.askForTime = 0;
    engineUpdate();
}


// called by radio when data arrived or by timeout job
static void scan_done (xref2osjob_t job) {
    if( LMIC.dataLen ) { // frame received (scan_done scheduled by irqjob after rxdone irq, radio stopped)
        if( decodeMultiCastFrame() ) {
	    // good frame
	    LMIC.opmode &= ~OP_NOENGINE;
	    reportEvent(EV_SCAN_FOUND);
	} else {
	    // bad frame
	    BACKTRACE();
	    LMIC.dataLen = 0;
	    // continue scanning until timeout
	    os_setTimedCallback(&LMIC.osjob, LMIC.bcninfo.txtime, scan_done); // job/func also used for radio callback!
	    os_radio(RADIO_RXON);
	}
    } else { // timeout
	// stop radio and its job
	os_radio(RADIO_RST);
        LMIC.opmode &= ~OP_NOENGINE;
        reportEvent(EV_SCAN_TIMEOUT);
    }
}


// start scanning for multi-cast data frame until timeout (LMIC.freq and LMIC.dndr must be set)
// will generate EV_SCAN_FOUND or EV_SCAN_TIMEOUT events
int LMIC_scan (ostime_t timeout) {
    if( (LMIC.opmode & (OP_SCAN|OP_TRACK|OP_SHUTDOWN)) != 0 ) {
        return 0;  // already in progress or failed to enable
    }

    // cancel onging TX/RX transaction
    LMIC.txCnt = LMIC.dnConf = 0;
    LMIC.opmode = (LMIC.opmode | OP_NOENGINE) & ~(OP_TXRXPEND);
    LMIC.dataLen = 0;

    // start scanning until timeout
    LMIC.rps = dndr2rps(LMIC.dndr);
    LMIC.bcninfo.txtime = os_getTime() + timeout; // save timeout
    os_setTimedCallback(&LMIC.osjob, LMIC.bcninfo.txtime, scan_done); // job/func also used for radio callback!
    os_radio(RADIO_RXON);

    return 1;  // enabled
}


// called by radio when data arrived or on symbol timeout
static void track_done (xref2osjob_t job) {
    LMIC.opmode &= ~OP_NOENGINE;
    if( LMIC.dataLen && decodeMultiCastFrame() ) {
	reportEvent(EV_BEACON_TRACKED);
    } else {
	LMIC.dataLen = 0;
	reportEvent(EV_BEACON_MISSED);
    }
}


static void track_start (xref2osjob_t job) {
    LMIC.rxsyms = MINRX_SYMS;
    LMIC.osjob.func = track_done;
    os_radio(RADIO_RX);
}


// try to receive single multi-cast data frame at specified time (LMIC.freq and LMIC.dndr must be set)
// will generate EV_BEACON_TRACKED or EV_BEACON_MISSED events
int LMIC_track (ostime_t when) {
    if( (LMIC.opmode & (OP_SCAN|OP_TRACK|OP_SHUTDOWN)) != 0 ) {
        return 0;  // already in progress or failed to enable
    }

    // cancel onging TX/RX transaction
    LMIC.txCnt = LMIC.dnConf = 0;
    LMIC.opmode = (LMIC.opmode | OP_NOENGINE) & ~(OP_TXRXPEND);
    LMIC.dataLen = 0;

    // schedule single rx at given time considering ramp-up time
    LMIC.rps = dndr2rps(LMIC.dndr);
    LMIC.rxtime = when + dr2hsym(LMIC.dndr, PAMBL_SYMS-MINRX_SYMS);
    os_setTimedCallback(&LMIC.osjob, LMIC.rxtime - RX_RAMPUP, track_start);
    return 1;
}


// ================================================================================
//
// Join stuff
//
// ================================================================================

static void buildJoinRequest (u1_t ftype) {
    // Do not use pendTxData since we might have a pending
    // user level frame in there. Use RX holding area instead.
    xref2u1_t d = LMIC.frame;
    d[OFF_JR_HDR] = ftype;
    os_getJoinEui(d + OFF_JR_JOINEUI);
    os_getDevEui(d + OFF_JR_DEVEUI);
    os_wlsbf2(d + OFF_JR_DEVNONCE, LMIC.devNonce);
    aes_appendMic0(d, OFF_JR_MIC);

    EV(joininfo,INFO,(e_.deveui  = MAIN::CDEV->getEui(),
                      e_.arteui  = MAIN::CDEV->getArtEui(),
                      e_.nonce   = LMIC.devNonce,
                      e_.oldaddr = LMIC.devaddr,
                      e_.mic     = Base::lsbf4(&d[LORA::OFF_JR_MIC]),
                      e_.reason  = ((LMIC.opmode & OP_REJOIN) != 0
                                    ? EV::joininfo_t::REJOIN_REQUEST
                                    : EV::joininfo_t::REQUEST)));
    LMIC.dataLen = LEN_JR;
    LMIC.devNonce++;
    DO_DEVDB(LMIC.devNonce,devNonce);
}

static void startJoining (xref2osjob_t osjob) {
    reportEvent(EV_JOINING);
}

// Start join procedure if not already joined.
bit_t LMIC_startJoining (void) {
    if( LMIC.devaddr == 0 ) {
        // There should be no TX/RX going on
        ASSERT((LMIC.opmode & (OP_POLL|OP_TXRXPEND)) == 0);
        // Lift any previous duty limitation
        LMIC.globalDutyRate = 0;
        // Cancel scanning
        LMIC.opmode &= ~(OP_SCAN|OP_REJOIN|OP_LINKDEAD|OP_NEXTCHNL);
        // Setup state
        LMIC.rejoinCnt = LMIC.txCnt = LMIC.pendTxConf = 0;
        initJoinLoop();
        LMIC.opmode |= OP_JOINING;
	LMIC.clmode = 0;
	LMIC.pollcnt = 0;
        // reportEvent will call engineUpdate which then starts sending JOIN REQUESTS
        os_setCallback(&LMIC.osjob, FUNC_ADDR(startJoining));
        return 1;
    }
    return 0; // already joined
}


// ================================================================================
//
//
//
// ================================================================================

static void processPingRx (xref2osjob_t osjob) {
    if( LMIC.dataLen != 0 ) {
        LMIC.txrxFlags = TXRX_PING;
        if( decodeFrame() ) {
            reportEvent(EV_RXCOMPLETE);
            return;
        }
    }
    // Pick next ping slot
    engineUpdate();
}


static bit_t processDnData (void) {
    ASSERT((LMIC.opmode & OP_TXRXPEND)!=0);

    if( LMIC.dataLen == 0 ) {
      norx:
	// Nothing received - implies no port
	LMIC.txrxFlags = TXRX_NOPORT;
	if( (LMIC.opmode & OP_TXDATA) ) {
	    LMIC.txCnt += 1;
	    if( LMIC.txCnt < LMIC.nbTrans ) {
		// Schedule another retransmission
		txDelay(LMIC.rxtime, RETRY_PERIOD_secs);
		LMIC.opmode |= OP_NEXTCHNL;
	    } else {
		LMIC.opmode &= ~OP_TXDATA;
            }
	    if( LMIC.pendTxConf != 0 )  // we requested an ACK
		LMIC.txrxFlags |= TXRX_NACK;
        }
        if( LMIC.adrAckReq != LINK_CHECK_OFF )
            LMIC.adrAckReq += 1;
	LMIC.dataBeg = LMIC.dataLen = 0;
      txcomplete:
	LMIC.opmode &= ~OP_TXRXPEND;
        if( (LMIC.txrxFlags & (TXRX_DNW1|TXRX_DNW2|TXRX_PING)) != 0  &&  (LMIC.opmode & OP_LINKDEAD) != 0 ) {
            LMIC.opmode &= ~OP_LINKDEAD;
            reportEvent(EV_LINK_ALIVE);
        }
        reportEvent(EV_TXCOMPLETE);
        // If we haven't heard from NWK in a while although we asked for a sign
        // assume link is dead - notify application and keep going
        if( LMIC.adrAckReq >= (s4_t) LINK_CHECK_DEAD ) {
            // We haven't heard from NWK for some time although we
            // asked for a response for some time - assume we're disconnected. Lower DR one notch.
            EV(devCond, ERR, (e_.reason = EV::devCond_t::LINK_DEAD,
                              e_.eui    = MAIN::CDEV->getEui(),
                              e_.info   = LMIC.adrAckReq));
	    if (LMIC.txPowAdj) {
		setDrTxpow(DRCHG_NOADRACK, LMIC.datarate, 0);
	    } else if (decDR((dr_t) LMIC.datarate) != LMIC.datarate) {
		setDrTxpow(DRCHG_NOADRACK, decDR((dr_t) LMIC.datarate), KEEP_TXPOWADJ);
#if defined(CFG_us915)
	    } else if (enableAllChannels()) {
		// some channels were disabled
#endif
	    } else {
		// nothing we can do anymore
		LMIC.opmode |= /* XXX OP_REJOIN| */ OP_LINKDEAD;
	    }
            LMIC.adrAckReq = 0;
            reportEvent((LMIC.opmode & OP_LINKDEAD) ? EV_LINK_DEAD : EV_ADR_BACKOFF);
        }
        // If this falls to zero the NWK did not answer our MCMD_TIME_REQ commands - try full scan
        if( LMIC.askForTime > 0 ) {
	    if( --LMIC.askForTime == 0 ) {
                startScan();   // NWK did not answer - try scan
		reportEvent(EV_START_SCAN);
	    }
	    else {
		LMIC.opmode |= OP_POLL;
	    }
        }
        return 1;
    }
    if( !decodeFrame() ) {
        if( (LMIC.txrxFlags & TXRX_DNW1) != 0 )
            return 0;
        goto norx;
    }
    addRxdErr(LMIC.dn1Dly + (LMIC.txrxFlags & TXRX_DNW2 ? DELAY_EXTDNW2 : 0));
    if( (LMIC.opmode & OP_TXDATA) )
	LMIC.txCnt += 1;
    LMIC.opmode &= ~OP_TXDATA;
    LMIC.pendTxConf = 0;
    goto txcomplete;
}


static void processBeacon (xref2osjob_t osjob) {
    ostime_t lasttx = LMIC.bcninfo.txtime;   // save previous - decodeBeacon overwrites
    u1_t flags = LMIC.bcninfo.flags;
    ev_t ev;

    if( decodeBeacon() >= 1 ) {
        ev = EV_BEACON_TRACKED;
        if( (flags & (BCN_PARTIAL|BCN_FULL)) == 0 ) {
            // We don't have a previous beacon to calc some drift - assume some max value
            calcBcnRxWindowFromMillis(BCN_100PPM_ms,0);
            goto rev;
        }
        // We have a previous BEACON to calculate some drift
        s2_t drift = (LMIC.bcninfo.txtime - lasttx) - BCN_INTV_osticks;
        if( LMIC.missedBcns > 0 ) {
            drift = LMIC.drift + (drift - LMIC.drift) / (LMIC.missedBcns+1);
        }
        if( (LMIC.bcninfo.flags & BCN_NODRIFT) == 0 ) {
            s2_t diff = LMIC.drift - drift;
            if( diff < 0 ) diff = -diff;
            LMIC.lastDriftDiff = diff;
            if( LMIC.maxDriftDiff < diff )
                LMIC.maxDriftDiff = diff;
            LMIC.bcninfo.flags &= ~BCN_NODDIFF;
        }
        LMIC.drift = drift;
        LMIC.missedBcns = LMIC.rejoinCnt = 0;
        LMIC.bcninfo.flags &= ~BCN_NODRIFT;
        EV(devCond,INFO,(e_.reason = EV::devCond_t::CLOCK_DRIFT,
                         e_.eui    = MAIN::CDEV->getEui(),
                         e_.info   = drift,
                         e_.info2  = /*occasion BEACON*/0));
        ASSERT((LMIC.bcninfo.flags & (BCN_PARTIAL|BCN_FULL)) != 0);
    } else {
        ev = EV_BEACON_MISSED;
        LMIC.bcninfo.txtime += BCN_INTV_osticks + LMIC.drift;
        LMIC.bcninfo.time   += BCN_INTV_sec;
        LMIC.missedBcns++;
        // Delay any possible TX after surmised beacon - it's there although we missed it
        txDelay(LMIC.bcninfo.txtime + BCN_RESERVE_osticks, 4);
        if( LMIC.missedBcns > MAX_MISSED_BCNS )
            LMIC.opmode |= OP_REJOIN;  // try if we can roam to another network
        if( LMIC.bcnRxsyms > MAX_RXSYMS ) {
            LMIC.opmode &= ~(OP_TRACK|OP_PINGABLE|OP_PINGINI|OP_REJOIN);
            reportEvent(EV_LOST_TSYNC);
            return;
        }
    }
    LMIC.bcnRxtime = LMIC.bcninfo.txtime + BCN_INTV_osticks - calcRxWindow(0,DR_BCN);
    LMIC.bcnRxsyms = LMIC.rxsyms;
  rev:
    LMIC.bcnChnl = (LMIC.bcnChnl+1) % NCHNL_BCN;
    if( (LMIC.opmode & OP_PINGINI) != 0 )
        rxschedInit(&LMIC.ping);  // note: reuses LMIC.frame buffer!
    reportEvent(ev);
}


static void startRxBcn (xref2osjob_t osjob) {
    LMIC.osjob.func = FUNC_ADDR(processBeacon);
    os_radio(RADIO_RX);
}


static void startRxPing (xref2osjob_t osjob) {
    LMIC.osjob.func = FUNC_ADDR(processPingRx);
    os_radio(RADIO_RX);
}


// Decide what to do next for the MAC layer of a device
static void engineUpdate (void) {
    // Check for ongoing state: scan or TX/RX transaction
    if( (LMIC.opmode & (OP_NOENGINE|OP_TXRXPEND|OP_SHUTDOWN)) != 0 ) {
        return;
    }

#ifdef CFG_autojoin
    if( LMIC.devaddr == 0 && (LMIC.opmode & OP_JOINING) == 0 ) {
        LMIC_startJoining();
        return;
    }
#endif // CFG_autojoin

    ostime_t now    = os_getTime();
    ostime_t rxtime = 0;
    ostime_t txbeg  = 0;

    if( (LMIC.opmode & OP_SCAN) != 0 ) {
	// Looking for a beacon - LMIC.bcninfo.txtime is timeout for scan
	// Cancel onging TX/RX transaction
	LMIC.dataLen = 0;
	LMIC.txCnt = LMIC.dnConf = LMIC.bcninfo.flags = 0;
	LMIC.opmode &= ~OP_TXRXPEND;
	LMIC.bcnChnl = 0;
	setBcnRxParams();
	os_setTimedCallback(&LMIC.osjob, LMIC.bcninfo.txtime, FUNC_ADDR(onBcnScanRx));
	os_radio(RADIO_RXON);
	return;
    }
    if( (LMIC.opmode & OP_TRACK) != 0 ) {
        // We are tracking a beacon
        rxtime = LMIC.bcnRxtime - RX_RAMPUP;
#if CFG_simul
	// Simulation is sometimes late - don't die here but keep going.
	// Results in a missed beacon. On the HW this spells a more serious problem.
	if( (ostime_t)(rxtime-now) < 0 ) {
	    fprintf(stderr, "ERROR: engineUpdate/OP_TRACK: delta=%d now=0x%X rxtime=0x%X LMIC.bcnRxtime=0x%X RX_RAMPUP=%d\n",
		    (ostime_t)(rxtime-now),now,rxtime,LMIC.bcnRxtime,RX_RAMPUP);
	}
#else
        ASSERT( (ostime_t)(rxtime-now) >= 0 );
#endif
    }
    if( LMIC.pollcnt )
	LMIC.opmode |= OP_POLL;

    if( (LMIC.opmode & (OP_JOINING|OP_REJOIN|OP_TXDATA|OP_POLL)) != 0 ) {
        // Need to TX some data...
        // Assuming txChnl points to channel which first becomes available again.
        bit_t jacc = ((LMIC.opmode & (OP_JOINING|OP_REJOIN)) != 0 ? 1 : 0);
        // Find next suitable channel and return availability time
        if( (LMIC.opmode & OP_NEXTCHNL) != 0 ) {
            txbeg = LMIC.txend = nextTx(now);
        } else {
            txbeg = LMIC.txend;
        }
        // Delayed TX or waiting for duty cycle?
        if( (LMIC.globalDutyRate != 0 || (LMIC.opmode & OP_RNDTX) != 0)  &&  (txbeg - LMIC.globalDutyAvail) < 0 )
            txbeg = LMIC.globalDutyAvail;
        // If we're tracking a beacon...
        // then make sure TX-RX transaction is complete before beacon
        if( (LMIC.opmode & OP_TRACK) != 0 &&
            txbeg + (jacc ? JOIN_GUARD_osticks : TXRX_GUARD_osticks) - rxtime > 0 ) {
            // Not enough time to complete TX-RX before beacon - postpone after beacon.
            // In order to avoid clustering of postponed TX right after beacon randomize start!
            txDelay(rxtime + BCN_RESERVE_osticks, 16);
            txbeg = 0;
            goto checkrx;
        }
        // Earliest possible time vs overhead to setup radio
        if( txbeg - (now + TX_RAMPUP) <= 0 ) {
            // We could send right now!
	    txbeg = now;
            dr_t txdr = (dr_t)LMIC.datarate;
            if( jacc ) {
                u1_t ftype;
                if( (LMIC.opmode & OP_REJOIN) != 0 ) {
                    txdr = lowerDR(txdr, LMIC.rejoinCnt);
                    ftype = HDR_FTYPE_REJOIN;
                } else {
                    ftype = HDR_FTYPE_JREQ;
                }
                buildJoinRequest(ftype);
                LMIC.osjob.func = FUNC_ADDR(jreqDone);
            } else {
		// XXX - TODO - also handle LMIC.seqnoADn rollover
                if( LMIC.seqnoDn >= 0xFFFFFF80 ) {
                    // Imminent roll over - proactively reset MAC
                    EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_ROLL_OVER,
                                        e_.eui    = MAIN::CDEV->getEui(),
                                        e_.info   = LMIC.seqnoDn,
                                        e_.info2  = 0));
                    // Device has to react! NWK will not roll over and just stop sending.
                    // Thus, we have N frames to detect a possible lock up.
                  reset:
                    os_setCallback(&LMIC.osjob, FUNC_ADDR(runReset));
                    return;
                }
                if( (LMIC.txCnt==0 && LMIC.seqnoUp == 0xFFFFFFFF) ) {
                    // Roll over of up seq counter
                    EV(specCond, ERR, (e_.reason = EV::specCond_t::UPSEQNO_ROLL_OVER,
                                       e_.eui    = MAIN::CDEV->getEui(),
                                       e_.info2  = LMIC.seqnoUp));
                    // Do not run RESET event callback from here!
                    // App code might do some stuff after send unaware of RESET.
                    goto reset;
                }
                buildDataFrame();
                LMIC.osjob.func = FUNC_ADDR(updataDone);
            }
            LMIC.rps    = setCr(updr2rps(txdr), (cr_t)LMIC.errcr);
            LMIC.dndr   = txdr;  // carry TX datarate (can be != LMIC.datarate) over to txDone/setupRx1
            LMIC.opmode = (LMIC.opmode & ~(OP_POLL|OP_RNDTX)) | OP_TXRXPEND | OP_NEXTCHNL;
            updateTx(txbeg);
            reportEvent(EV_TXSTART);
            os_radio(RADIO_TX);
            return;
        }
        // Cannot yet TX
        if( (LMIC.opmode & OP_TRACK) == 0 )
            goto txdelay; // We don't track the beacon - nothing else to do - so wait for the time to TX
        // Consider RX tasks
        if( txbeg == 0 ) // zero indicates no TX pending
            txbeg += 1;  // TX delayed by one tick (insignificant amount of time)
    } else {
        // No TX pending - no scheduled RX
        if( (LMIC.opmode & OP_TRACK) == 0 ) {
	    if( (LMIC.clmode & CLASS_C) ) {
		setupRx2ClassC();
	    }
            return;
	}
    }

    // Are we pingable?
  checkrx:
    if( (LMIC.opmode & OP_PINGINI) != 0 ) {
        // One more RX slot in this beacon period?
        if( rxschedNext(&LMIC.ping, now+RX_RAMPUP) ) {
            if( txbeg != 0  &&  (txbeg - LMIC.ping.rxtime) < 0 )
                goto txdelay;
            LMIC.rxsyms  = LMIC.ping.rxsyms;
            LMIC.rxtime  = LMIC.ping.rxtime;
            LMIC.freq    = LMIC.ping.freq;          // XXX:US like => calc based on beacon time!
            LMIC.rps     = dndr2rps(LMIC.ping.dr);
            LMIC.dataLen = 0;
            ASSERT(LMIC.rxtime - now+RX_RAMPUP >= 0 );
            os_setTimedCallback(&LMIC.osjob, LMIC.rxtime - RX_RAMPUP, FUNC_ADDR(startRxPing));
            return;
        }
        // no - just wait for the beacon
    }

    if( txbeg != 0  &&  (txbeg - rxtime) < 0 )
        goto txdelay;

    setBcnRxParams();
    LMIC.rxsyms = LMIC.bcnRxsyms;
    LMIC.rxtime = LMIC.bcnRxtime;
    if( now - rxtime >= 0 ) {
        LMIC.osjob.func = FUNC_ADDR(processBeacon);
        os_radio(RADIO_RX);
        return;
    }
    os_setTimedCallback(&LMIC.osjob, rxtime, FUNC_ADDR(startRxBcn));
    return;

  txdelay:
    EV(devCond, INFO, (e_.reason = EV::devCond_t::TX_DELAY,
                       e_.eui    = MAIN::CDEV->getEui(),
                       e_.info   = osticks2ms(txbeg-now),
                       e_.info2  = LMIC.seqnoUp-1));
    if( (LMIC.clmode & CLASS_C) ) {
	setupRx2ClassC();
    }
    os_setTimedCallback(&LMIC.osjob, txbeg-TX_RAMPUP, FUNC_ADDR(runEngineUpdate));
}


void LMIC_setAdrMode (bit_t enabled) {
    LMIC.adrEnabled = enabled ? FCT_ADREN : 0;
}


//  Should we have/need an ext. API like this?
void LMIC_setDrTxpow (dr_t dr, s1_t txpowadj) {
    setDrTxpow(DRCHG_SET, dr, txpowadj);
}


void LMIC_shutdown (void) {
    os_clearCallback(&LMIC.osjob);
    os_radio(RADIO_RST);
    LMIC.opmode |= OP_SHUTDOWN;
}


void LMIC_reset (void) {
    EV(devCond, INFO, (e_.reason = EV::devCond_t::LMIC_EV,
                       e_.eui    = MAIN::CDEV->getEui(),
                       e_.info   = EV_RESET));
    os_radio(RADIO_RST);
    os_clearCallback(&LMIC.osjob);

    os_clearMem((xref2u1_t)&LMIC,SIZEOFEXPR(LMIC));
    LMIC.devaddr      =  0;
    LMIC.devNonce     =  os_getRndU2();
    LMIC.opmode       =  OP_NONE;
    LMIC.clmode       =  0;
    LMIC.pollcnt      =  0;
    LMIC.errcr        =  CR_4_5;
    LMIC.adrEnabled   =  FCT_ADREN;
    LMIC.datarate     =  DR_MAX125;
    LMIC.txPowAdj     =  0;
    LMIC.nbTrans      =  0;
    LMIC.dn1DrOff     =  0;
    LMIC.dn1Dly       =  1;
    LMIC.dn2Dr        =  DR_DNW2;   // we need this for 2nd DN window of join accept
    LMIC.dn2Freq      =  FREQ_DNW2; // ditto
    LMIC.ping.freq    =  FREQ_PING; // defaults for ping
    LMIC.ping.dr      =  DR_PING;   // ditto
    LMIC.ping.intvExp =  8;         // no ping interval ever sent up
    // Ini all time keeping in case current time at reset is NOT 0.
    // This is true for:
    //   - intermittent LMIC_reset() - no HW reset
    //   - initial clock value not 0 (e.g. simulation)
    osxtime_t xnow = os_getXTime();
    LMIC.globalDutyAvail = (ostime_t)xnow;
    LMIC.refChnl = os_getRndU1();
#if defined(CFG_us915)
    initDefaultChannels();
    for( u1_t i=0; i<MAX_CHNLS; i++ )
	LMIC.chDwellAvail[i] = xnow;
#endif
#if defined(CFG_eu868)
    LMIC.noDC = 0;
    for( u1_t i=0; i<MAX_BANDS; i++ )
	LMIC.bands[i].avail = (osxtime_t)xnow;
    for( u1_t i=0; i<MAX_CHNLS; i++ )
	LMIC.channelAvail[i] = (osxtime_t)xnow + os_getRndU1() % (2*MAX_CHNLS);
#endif
    iniRxdErr();
    DO_DEVDB(LMIC.devaddr,      devaddr);
    DO_DEVDB(LMIC.devNonce,     devNonce);
    DO_DEVDB(LMIC.dn2Dr,        dn2Dr);
    DO_DEVDB(LMIC.dn2Freq,      dn2Freq);
    DO_DEVDB(LMIC.ping.freq,    pingFreq);
    DO_DEVDB(LMIC.ping.dr,      pingDr);
    DO_DEVDB(LMIC.ping.intvExp, pingIntvExp);
}


void LMIC_init (void) {
    LMIC.opmode = OP_SHUTDOWN;
}


void LMIC_clrTxData (void) {
    LMIC.opmode &= ~(OP_TXDATA|OP_TXRXPEND|OP_POLL);
    LMIC.pendTxLen = 0;
    if( (LMIC.opmode & (OP_JOINING|OP_SCAN)) != 0 ) // do not interfere with JOINING/SCANNING
        return;
    os_clearCallback(&LMIC.osjob);
    os_radio(RADIO_RST);
    engineUpdate();
}


void LMIC_setTxData (void) {
    ASSERT((LMIC.opmode & OP_JOINING) == 0);
    LMIC.opmode |= OP_TXDATA;
    LMIC.txCnt = 0;             // reset nbTrans counter
    engineUpdate();
}


//
int LMIC_setTxData2 (u1_t port, xref2u1_t data, u1_t dlen, u1_t confirmed) {
    if( dlen > SIZEOFEXPR(LMIC.pendTxData) )
        return -2;
    if( data != (xref2u1_t)0 )
        os_copyMem(LMIC.pendTxData, data, dlen);
    LMIC.pendTxConf = confirmed;
    LMIC.pendTxPort = port;
    LMIC.pendTxLen  = dlen;
    LMIC_setTxData();
    return 0;
}


// Send a payload-less message to signal device is alive
void LMIC_sendAlive (void) {
    LMIC.opmode |= OP_POLL;
    LMIC.pendTxConf = 0;
    LMIC.pendTxLen = 0;
    LMIC.txCnt = 0;             // reset nbTrans counter
    engineUpdate();
}


// Check if other networks are around.
void LMIC_tryRejoin (void) {
    LMIC.opmode |= OP_REJOIN;
    engineUpdate();
}


// Check if other networks are around.
void LMIC_setClassC (u1_t enabled) {
    if( (LMIC.clmode & CLASS_C) == (enabled?CLASS_C:0) )
	return;  // already in that mode
    if( (LMIC.clmode & PEND_CLASS_C) )
	return; // change is already pending
    if( enabled )
	LMIC_stopPingable();  // stop class B
#if defined(CFG_lorawan11)
    if( enabled != UNILATERAL_CLASS_C ) {
	LMIC.clmode |= PEND_CLASS_C;
	LMIC.opmode |= OP_POLL;
	incPollcnt();
	engineUpdate();
	return;
    }
#endif
    // LoRaWAN 1.0.2 - switch mode unilaterally
    // Device is provisioned as class C
    LMIC.clmode = CLASS_C;
    engineUpdate();
}

//! \brief Setup given session keys
//! and put the MAC in a state as if
//! a join request/accept would have negotiated just these keys.
//! It is crucial that the combinations `devaddr/nwkkey` and `devaddr/artkey`
//! are unique within the network identified by `netid`.
//! NOTE: on Harvard architectures when session keys are in flash:
//!  Caller has to fill in LMIC.{nwk,art}Key  before and pass {nwk,art}Key are NULL
//! \param netid a 24 bit number describing the network id this device is using
//! \param devaddr the 32 bit session address of the device. It is strongly recommended
//!    to ensure that different devices use different numbers with high probability.
//! \param nwkKey  the 16 byte network session key used for message integrity.
//!     If NULL the caller has copied the key into `LMIC.nwkKey` before.
#if defined(CFG_lorawan11)
//! \param nwkKeyDn  the 16 byte network session key used for down-link message integrity.
//!     If NULL the caller has copied the key into `LMIC.nwkKeyDn` before.
#endif
//! \param appKey  the 16 byte application router session key used for message confidentiality.
//!     If NULL the caller has copied the key into `LMIC.appKey` before.
void LMIC_setSession (u4_t netid, devaddr_t devaddr, xref2cu1_t nwkKey,
#if defined(CFG_lorawan11)
	xref2cu1_t nwkKeyDn,
#endif
	xref2cu1_t appKey) {
    LMIC.netid = netid;
    LMIC.devaddr = devaddr;
    if( nwkKey != (xref2u1_t)0 )
        os_copyMem(LMIC.nwkKey, nwkKey, 16);
#if defined(CFG_lorawan11)
    if( nwkKeyDn != (xref2u1_t)0 )
        os_copyMem(LMIC.nwkKeyDn, nwkKeyDn, 16);
#endif
    if( appKey != (xref2u1_t)0 )
        os_copyMem(LMIC.appKey, appKey, 16);

#if defined(CFG_eu868)
    initDefaultChannels(0);
#endif

    stateJustJoined();
    LMIC.dn2Dr = DR_DNW2;
    LMIC.dn1Dly = 1;
    LMIC.dn1DrOff = 0;

    DO_DEVDB(LMIC.netid,   netid);
    DO_DEVDB(LMIC.devaddr, devaddr);
    DO_DEVDB(LMIC.nwkKey,  nwkkey);
#if defined(CFG_lorawan11)
    DO_DEVDB(LMIC.nwkKeyDn,nwkkeydn);
#endif
    DO_DEVDB(LMIC.appKey,  artkey);
    DO_DEVDB(LMIC.seqnoUp, seqnoUp);
    DO_DEVDB(LMIC.seqnoDn, seqnoDn);
#if defined(CFG_lorawan11)
    DO_DEVDB(LMIC.seqnoADn,seqnoADn);
#endif
}

void LMIC_setMultiCastSession (devaddr_t grpaddr, xref2cu1_t nwkKeyDn, xref2cu1_t appKey, u4_t seqnoADn) {
    session_t* s;
    for(s = LMIC.sessions; s<LMIC.sessions+MAX_MULTICAST_SESSIONS && s->grpaddr!=0 && s->grpaddr!=grpaddr; s++);
    ASSERT(s < LMIC.sessions+MAX_MULTICAST_SESSIONS);

    s->grpaddr  = grpaddr;
    s->seqnoADn = seqnoADn;

    if( nwkKeyDn != (xref2u1_t)0 ) {
        os_copyMem(s->nwkKeyDn, nwkKeyDn, 16);
    }

    if( appKey != (xref2u1_t)0 ) {
        os_copyMem(s->appKey, appKey, 16);
    }
}

// Enable/disable link check validation.
// LMIC sets the ADRACKREQ bit in UP frames if there were no DN frames
// for a while. It expects the network to provide a DN message to prove
// connectivity with a span of UP frames. If this no such prove is coming
// then the datarate is lowered and a LINK_DEAD event is generated.
// This mode can be disabled and no connectivity prove (ADRACKREQ) is requested
// nor is the datarate changed.
// This must be called only if a session is established (e.g. after EV_JOINED)
void LMIC_setLinkCheckMode (bit_t enabled) {
    LMIC.adrAckReq = enabled ? LINK_CHECK_INIT : LINK_CHECK_OFF;
}

void LMIC_setLinkCheck (u4_t limit, u4_t delay) {
    LMIC.adrAckLimit = limit;
    LMIC.adrAckDelay = delay;
    LMIC.adrAckReq = LINK_CHECK_INIT;
}

void LMIC_askForLinkCheck (void) {
    LMIC.gwmargin = 255;
    LMIC.gwcnt = 0;
}


#if defined(CFG_simul)
#include "addr2func.h"
#include "arr2len.h"
#endif

#if defined(CFG_extapi)

// Enable fast join (for testing only)
// Removes duty cycle limitations.
// Call directly after LMIC_startJoining() returns non-zero.
void LMIC_enableFastJoin (void) {
#if defined(CFG_eu868)
    LMIC.bands[BAND_MILLI].txcap = 1;  // 100%, no DC
#endif // defined(CFG_eu868)
    LMIC.opmode |= OP_TESTMODE;
}

// Remove duty cycle limitations
void LMIC_disableDC (void) {
#if defined(CFG_eu868)
    LMIC.noDC = 1;
#endif
}

/// Used for regression testing
ostime_t LMIC_dr2hsym (dr_t dr, s1_t num) {
    return dr2hsym(dr,num);
}

ostime_t LMIC_nextTx (ostime_t now) {
    ostime_t t = nextTx(now);
    updateTx(t);
    return t;
}


ostime_t LMIC_calcAirTime (rps_t rps, u1_t plen) {
    return calcAirTime(rps, plen);
}

// Return scaled clock skew (RXDERR_SHIFT) and variation span in osticks.
void LMIC_getRxdErrInfo (s4_t* skew, u4_t* span) {
    *skew = evalRxdErr(span);
}

#endif
