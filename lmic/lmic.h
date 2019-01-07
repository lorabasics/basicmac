// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

//! @file
//! @brief LMIC API

#ifndef _lmic_h_
#define _lmic_h_

#include "oslmic.h"
#include "lorabase.h"

// LMIC version
#define LMIC_VERSION_MAJOR 2
#define LMIC_VERSION_MINOR 0
#define LMIC_VERSION_BUILD 1468577746

//! Only For Antenna Tuning Tests !
//#define CFG_TxContinuousMode 1

enum { TXCONF_ATTEMPTS    =   8 };   //!< Transmit attempts for confirmed frames
enum { MAX_MISSED_BCNS    =  20 };   // threshold for triggering rejoin requests
enum { MAX_RXSYMS         = 100 };   // stop tracking beacon beyond this

#define RXDERR_NUM 5
#define RXDERR_SHIFT 4
#define RXDERR_SCALE (1<<RXERR_SHIFT)
#ifndef RXDERR_INI
#define RXDERR_INI 50  // ppm
#endif

#define LINK_CHECK_OFF  (0x80000000)
#define LINK_CHECK_INIT (-LMIC.adrAckLimit)
#define LINK_CHECK_DEAD (LMIC.adrAckDelay)

enum { TIME_RESYNC        = 6*128 }; // secs
enum { TXRX_GUARD_ms      =  6000 };  // msecs - don't start TX-RX transaction before beacon
enum { JOIN_GUARD_ms      =  9000 };  // msecs - don't start Join Req/Acc transaction before beacon
enum { TXRX_BCNEXT_secs   =     2 };  // secs - earliest start after beacon time
enum { RETRY_PERIOD_secs  =     3 };  // secs - random period for retrying a confirmed send

#if defined(CFG_eu868) // EU868 spectrum ====================================================

enum { MAX_BANDS    =  4 };

//! \internal
struct band_t {
    u2_t      txcap;    // duty cycle limitation: 1/txcap
    s1_t      txpow;    // maximum TX power
    osxtime_t avail;    // channel is blocked until this time
};
TYPEDEF_xref2band_t; //!< \internal

#endif // ==========================================================================

// Keep in sync with evdefs.hpp::drChange
enum { DRCHG_SET, DRCHG_NOJACC, DRCHG_NOACK, DRCHG_NOADRACK, DRCHG_NWKCMD };
enum { KEEP_TXPOWADJ = -128 };


//! \internal
struct rxsched_t {
    u1_t     dr;
    u1_t     intvExp;   // bits: 7:pend, 3:illegal intv, 2-0:intv
    u1_t     slot;      // runs from 0 to 128
    u1_t     rxsyms;
    ostime_t rxbase;
    ostime_t rxtime;    // start of next spot
    u4_t     freq;
};
TYPEDEF_xref2rxsched_t;  //!< \internal


//! Parsing and tracking states of beacons.
enum { BCN_NONE    = 0x00,   //!< No beacon received
       BCN_PARTIAL = 0x01,   //!< Only first (common) part could be decoded (info,lat,lon invalid/previous)
       BCN_FULL    = 0x02,   //!< Full beacon decoded
       BCN_NODRIFT = 0x04,   //!< No drift value measured yet
       BCN_NODDIFF = 0x08 }; //!< No differential drift measured yet
//! Information about the last and previous beacons.
struct bcninfo_t {
    ostime_t txtime;  //!< Time when the beacon was sent
    s1_t     rssi;    //!< Adjusted RSSI value of last received beacon
    s1_t     snr;     //!< Scaled SNR value of last received beacon
    u1_t     flags;   //!< Last beacon reception and tracking states. See BCN_* values.
    u4_t     time;    //!< GPS time in seconds of last beacon (received or surrogate)
    //
    u1_t     info;    //!< Info field of last beacon (valid only if BCN_FULL set)
    s4_t     lat;     //!< Lat field of last beacon (valid only if BCN_FULL set)
    s4_t     lon;     //!< Lon field of last beacon (valid only if BCN_FULL set)
};

// purpose of receive window - lmic_t.rxState
enum { RADIO_RST=0, RADIO_TX=1, RADIO_RX=2, RADIO_RXON=3, RADIO_TXCW };
// Netid values /  lmic_t.netid
enum { NETID_NONE=(int)~0U, NETID_MASK=(int)0xFFFFFF };
// MAC operation modes (lmic_t.opmode).
enum { OP_NONE     = 0x0000,
       OP_SCAN     = 0x0001, // radio scan to find a beacon
       OP_TRACK    = 0x0002, // track my networks beacon (netid)
       OP_JOINING  = 0x0004, // device joining in progress (blocks other activities)
       OP_TXDATA   = 0x0008, // TX user data (buffered in pendTxData)
       OP_POLL     = 0x0010, // send empty UP frame to ACK confirmed DN/fetch more DN data
       OP_REJOIN   = 0x0020, // occasionally send JOIN REQUEST
       OP_SHUTDOWN = 0x0040, // prevent MAC from doing anything
       OP_TXRXPEND = 0x0080, // TX/RX transaction pending
       OP_RNDTX    = 0x0100, // prevent TX lining up after a beacon
       OP_PINGINI  = 0x0200, // pingable is initialized and scheduling active
       OP_PINGABLE = 0x0400, // we're pingable - aka class B
       OP_NEXTCHNL = 0x0800, // find a new channel
       OP_LINKDEAD = 0x1000, // link was reported as dead
       OP_TESTMODE = 0x2000, // developer test mode
       OP_NOENGINE = 0x4000, // bypass engine update
       OP_NOCRYPT  = 0x8000, // do not encrypt uplinks
};
// TX-RX transaction flags - report back to user
enum { TXRX_ACK    = 0x80,   // confirmed UP frame was acked
       TXRX_NACK   = 0x40,   // confirmed UP frame was not acked
       TXRX_NOPORT = 0x20,   // set if a frame with a port was RXed, clr if no frame/no port
       TXRX_PORT   = 0x10,   // set if a frame with a port was RXed, LMIC.frame[LMIC.dataBeg-1] => port
       TXRX_DNW1   = 0x01,   // received in 1st DN slot
       TXRX_DNW2   = 0x02,   // received in 2dn DN slot
       TXRX_PING   = 0x04,   // received in a scheduled RX slot
};
// Event types for event callback
enum _ev_t { EV_SCAN_TIMEOUT=1, EV_BEACON_FOUND,
             EV_BEACON_MISSED, EV_BEACON_TRACKED, EV_JOINING,
             EV_JOINED, EV_RFU1, EV_JOIN_FAILED, EV_REJOIN_FAILED,
             EV_TXCOMPLETE, EV_LOST_TSYNC, EV_RESET, EV_RXCOMPLETE,
             EV_LINK_DEAD, EV_LINK_ALIVE, EV_SCAN_FOUND, EV_TXSTART,
	     EV_TXDONE, EV_DATARATE, EV_START_SCAN, EV_ADR_BACKOFF };
typedef enum _ev_t ev_t;


// Internal use values in lmic_t.opts, uses the unused upper nibble
// of option bitmap 1 (0xf0).
enum {
    OPT_LORAWAN11 = 0x80,  // Running LoRaWAN 1.1
    OPT_OPTNEG    = 0x40,  // Send ResetInd mac command
};

// data stored in clmode
enum {
    CLASS_C      = 0x01,  // 0=class A, 1=class C
    PEND_CLASS_C = 0x02,  // 1=MCMD_DEVMD_IND sent (with !CLASS_C, waiting for MCMD_DEVMD_CONF)
};
// parameters for LMIC_setClassC(..)
enum {
    DISABLE_CLASS_C = 0,  // disable class C - aka enable class A
    ENABLE_CLASS_C  = 1,  // enable class C - stop class A
    // do not notify network and do not wait for confirmation
    // (device provisioned as class C)
    UNILATERAL_CLASS_C = 2,
};

typedef struct {
    devaddr_t   grpaddr;      // multicast group address
    u1_t        nwkKeyDn[16]; // network session key for down-link
    u1_t        appKey[16];   // application session key
    u4_t        seqnoADn;     // down stream seqno (AFCntDown)
} session_t;

#define MAX_MULTICAST_SESSIONS 1

struct lmic_t {
    // Radio settings TX/RX (also accessed by HAL)
    ostime_t    txend;
    ostime_t    rxtime;  // timestamp when frame was fully received
    ostime_t    rxtime0; // timestamp when preamble of frame was received (computed)
    u4_t        freq;
    s1_t        rssi;
    s1_t        snr;
    rps_t       rps;
    u1_t        rxsyms;
    u1_t        dndr;
    s1_t        txpow;     // dBm -- needs to be combined with txPowAdj

    osjob_t     osjob;

    // Channel scheduling
#if defined(CFG_eu868)
    band_t      bands[MAX_BANDS];
    u4_t        channelFreq[MAX_CHNLS];
    u4_t        channelDnFreq[MAX_CHNLS];
    osxtime_t   channelAvail[MAX_CHNLS];       // time device can send again on this channel (PSA)
    u2_t        channelDrMap[MAX_CHNLS];
    u2_t        channelMap;
    u2_t        perChnlDC;
    u1_t        noDC;
#elif defined(CFG_us915)
    u2_t        channelMap[(MAX_CHNLS+15)/16]; // enabled bits
    osxtime_t   chDwellAvail[MAX_CHNLS];       // time device can send again on channel
#endif
    u1_t        refChnl;         // channel randomizer - search relative to this indicator
    u1_t        txChnl;          // channel for next TX
    u1_t        globalDutyRate;  // max rate: 1/2^k
    ostime_t    globalDutyAvail; // time device can send again

    u4_t        netid;        // current network id (~0 - none)
    u2_t        opmode;
    u1_t        clmode;       // current/pending class A/B/C
    u1_t        pollcnt;      // >0 waiting for an answer from network
    u1_t        nbTrans;      // ADR controlled frame repetition
    s1_t        txPowAdj;     // adjustment for txpow (ADR controlled)
    u1_t        datarate;     // current data rate
    u1_t        errcr;        // error coding rate (used for TX only)
    u1_t        rejoinCnt;    // adjustment for rejoin datarate
    s2_t        drift;        // last measured drift
    s2_t        lastDriftDiff;
    s2_t        maxDriftDiff;
    osxtime_t   gpsEpochOff;  // gpstime = gpsEpochOff+getXTime(), 0=undefined
    s4_t        rxdErrs[RXDERR_NUM];
    u1_t        rxdErrIdx;
    
    u1_t        pendTxPort;
    u1_t        pendTxConf;   // confirmed data
    u1_t        pendTxLen;    // +0x80 = confirmed
    u1_t        pendTxData[MAX_LEN_PAYLOAD];
    u1_t        pendTxNoRx;   // don't listen for down data after tx

    u2_t        devNonce;     // last generated nonce
    u1_t        nwkKey[16];   // network session key (LoRaWAN1.1: up-link only)
#if defined(CFG_lorawan11)
    u1_t        nwkKeyDn[16]; // network session key for down-link
#endif
    u1_t        appKey[16];   // application session key
    devaddr_t   devaddr;
    u4_t        seqnoDn;      // device level down stream seqno
#if defined(CFG_lorawan11)
    u4_t        seqnoADn;     // device level down stream seqno (AFCntDown)
#endif
    u4_t        seqnoUp;

    u1_t        dnConf;       // dn frame confirm pending: LORA::FCT_ACK or 0
    s4_t        adrAckReq;    // counter until we reset data rate (0x80000000=off)
    u4_t	adrAckLimit;  // ADR_ACK_LIMIT
    u4_t	adrAckDelay;  // ADR_ACK_DELAY

    u1_t        margin;       // bits 7/6:RFU, 0-5: SNR of last DevStatusReq frame, reported by DevStatusAns to network
    u1_t        gwmargin;     // last reported by network via LinkCheckAns
    u1_t        gwcnt;        //  - ditto -
    u1_t        foptsUpLen;
    u1_t        foptsUp[64];  // pending FOpts in up direction - cleared after next send
    bit_t       devsAns;      // device status answer pending
    u1_t        adrEnabled;
    u1_t        moreData;     // NWK has more data pending
    bit_t       dutyCapAns;   // have to ACK duty cycle settings
    //XXX:old: u1_t        snchAns;      // answer set new channel
    u1_t        dn1Dly;       // delay in secs to DNW1
    s1_t        dn1DrOff;     // DR offset to DNW1 (can be negative in some regions!)
    // 2nd RX window (after up stream)
    u1_t        dn2Dr;
    u4_t        dn2Freq;
    u1_t        dn2Ans;       // 0=no answer pend, 0x80+ACKs
    u1_t        dn1DlyAns;    // 0=no answer pend, 0x80 send MCMD_RXTM_ANS
    u1_t        dnfqAns;      // # of DNFQ in this down frame
    u1_t        dnfqAnsPend;  // pending ACK bits (2 each)
    u4_t        dnfqAcks;     // ack bit pending

    // multicast sessions
    session_t  sessions[MAX_MULTICAST_SESSIONS];

#if defined(CFG_lorawan11)
    u1_t	opts;         // negotiated protocol options
#endif

    // Class B state
    u1_t        missedBcns;   // unable to track last N beacons
    s1_t        askForTime;   // how often to ask for time
    //XXX:old: u1_t        pingSetAns;   // answer set cmd and ACK bits
    rxsched_t   ping;         // pingable setup

    // Public part of MAC state
    u1_t        txCnt;
    u1_t        txrxFlags;  // transaction flags (TX-RX combo)
    u1_t        dataBeg;    // 0 or start of data (dataBeg-1 is port)
    u1_t        dataLen;    // 0 no data or zero length data, >0 byte count of data
    u1_t        frame[MAX_LEN_FRAME];

    u1_t        bcnfAns;      // mcmd beacon freq: bit7:pending, bit0:ACK/NACK
    u1_t        bcnChnl;
    u4_t        bcnFreq;      // 0=default, !=0: specific BCN freq/no hopping
    u1_t        bcnRxsyms;    //
    ostime_t    bcnRxtime;
    bcninfo_t   bcninfo;      // Last received beacon info

    u1_t        noRXIQinversion;

#ifdef CFG_testpin
    // Signal specific event via a GPIO pin.
    // Test pin is routed to PPS pin of SX1301 to record time.
    // Current events:
    //  1=txdone, 2=rxend, 3=rxstart
    u1_t        testpinMode;
#endif
};
//! \var struct lmic_t LMIC
//! The state of LMIC MAC layer is encapsulated in this variable.
DECLARE_LMIC; //!< \internal

//! Construct a bit map of allowed datarates from drlo to drhi (both included).
#define DR_RANGE_MAP(drlo,drhi) (((u2_t)0xFFFF<<(drlo)) & ((u2_t)0xFFFF>>(15-(drhi))))
#if defined(CFG_eu868)
enum { BAND_MILLI=0, BAND_CENTI=1, BAND_DECI=2, BAND_AUX=3 };
bit_t LMIC_setupBand (u1_t bandidx, s1_t txpow, u2_t txcap);
#endif
bit_t LMIC_setupChannel (u1_t channel, u4_t freq, u2_t drmap, s1_t band);
void  LMIC_disableChannel (u1_t channel);

void  LMIC_setDrTxpow   (dr_t dr, s1_t txpow);  // set default/start DR/txpow
void  LMIC_setAdrMode   (bit_t enabled);        // set ADR mode (if mobile turn off)
bit_t LMIC_startJoining (void);

void  LMIC_shutdown     (void);
void  LMIC_init         (void);
void  LMIC_reset        (void);
void  LMIC_clrTxData    (void);
void  LMIC_setTxData    (void);
int   LMIC_setTxData2   (u1_t port, xref2u1_t data, u1_t dlen, u1_t confirmed);
void  LMIC_sendAlive    (void);

u1_t  LMIC_enableTracking  (u1_t tryBcnInfo);
void  LMIC_disableTracking (void);

void  LMIC_setClassC     (u1_t enabled);
void  LMIC_stopPingable  (void);
u1_t  LMIC_setPingable   (u1_t intvExp);
void  LMIC_tryRejoin     (void);

int LMIC_scan (ostime_t timeout);
int LMIC_track (ostime_t when);
void LMIC_setMultiCastSession (devaddr_t grpaddr, xref2cu1_t nwkKeyDn, xref2cu1_t appKey, u4_t seqnoAdn);

void LMIC_setSession (u4_t netid, devaddr_t devaddr, xref2cu1_t nwkKey,
#if defined(CFG_lorawan11)
	xref2cu1_t nwkKeyDn,
#endif
	xref2cu1_t appKey);
void LMIC_setLinkCheckMode (bit_t enabled);
void LMIC_setLinkCheck (u4_t limit, u4_t delay);
void LMIC_askForLinkCheck (void);


// Simulation only APIs
#if defined(CFG_simul)
const char* LMIC_addr2func (void* addr);
int LMIC_arr2len (const char* name);
#endif

// Special APIs - for development or testing
// !!!See implementation for caveats!!!
#if defined(CFG_extapi)
void     LMIC_enableFastJoin (void);
void     LMIC_disableDC (void);
ostime_t LMIC_calcAirTime (rps_t rps, u1_t plen);
ostime_t LMIC_dr2hsym (dr_t dr, s1_t num);
ostime_t LMIC_nextTx (ostime_t now);
void     LMIC_getRxdErrInfo (s4_t* skew, u4_t* span);
#endif


#endif // _lmic_h_
