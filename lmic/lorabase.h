// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#ifndef _lorabase_h_
#define _lorabase_h_

// ================================================================================
// BEG: Keep in sync with lorabase.hpp
//

enum _cr_t { CR_4_5=0, CR_4_6, CR_4_7, CR_4_8 };
enum _sf_t { FSK=0, SF7, SF8, SF9, SF10, SF11, SF12, SFrfu };
enum _bw_t { BW125=0, BW250, BW500, BWrfu };
typedef u1_t cr_t;
typedef u1_t sf_t;
typedef u1_t bw_t;
typedef u1_t dr_t;
// Radio parameter set (encodes SF/BW/CR/IH/NOCRC)
typedef u2_t rps_t;
TYPEDEF_xref2rps_t;

enum { ILLEGAL_RPS = 0xFF };
enum { DR_PAGE_EU868 = 0x00 };
enum { DR_PAGE_US915 = 0x10 };
enum { DR_PAGE_AU915 = 0x20 };
enum { DR_PAGE_AS923 = 0x30 };
enum { DR_PAGE_CN470 = 0x40 };
enum { DR_PAGE_IL915 = 0x50 };
enum { DR_PAGE_KR920 = 0x60 };

// Global maximum frame length
enum { BCN_PREAMBLE_LEN  = 10 };  // length in symbols - actual time depends on DR
enum { STD_PREAMBLE_LEN  =  8 };  //  -ditto-
enum { MAX_LEN_FRAME     =255 };  // in bytes
enum { LEN_DEVNONCE      =  2 };  //   -ditto-
enum { LEN_JOINNONCE     =  3 };  //   -ditto-
enum { LEN_NETID         =  3 };  //   -ditto-
enum { DELAY_JACC1       =  5 };  // in secs
enum { DELAY_DNW1        =  1 };  // in secs down window #1
enum { DELAY_EXTDNW2     =  1 };  // in secs
enum { DELAY_JACC2       =  DELAY_JACC1+(int)DELAY_EXTDNW2 }; // in secs
enum { DELAY_DNW2        =  DELAY_DNW1 +(int)DELAY_EXTDNW2 }; // in secs down window #1
enum { BCN_INTV_exp      = 7 };
enum { BCN_INTV_sec      = 1<<BCN_INTV_exp };
enum { BCN_INTV_ms       = BCN_INTV_sec*1000L };
enum { BCN_INTV_us       = BCN_INTV_ms*1000L };
enum { BCN_RESERVE_ms    = 2120 };   // space reserved for beacon and NWK management
enum { BCN_GUARD_ms      = 3000 };   // end of beacon period to prevent interference with beacon
enum { BCN_SLOT_SPAN_ms  =   30 };   // 2^12 reception slots a this span
enum { BCN_WINDOW_ms     = BCN_INTV_ms-(int)BCN_GUARD_ms-(int)BCN_RESERVE_ms };
enum { BCN_RESERVE_us    = 2120000 };
enum { BCN_GUARD_us      = 3000000 };
enum { BCN_SLOT_SPAN_us  =   30000 };

// Default ADR back-off parameters
// Actually region-specific, but identical for all known regions.
enum {
    ADR_ACK_LIMIT        = 64,
    ADR_ACK_DELAY        = 32,
};

#if defined(CFG_eu868) // ==============================================

enum _dr_eu868_t { DR_SF12=0, DR_SF11, DR_SF10, DR_SF9, DR_SF8, DR_SF7, DR_SF7B, DR_FSK, DR_NONE };
enum { DR_MAX125 = DR_SF7 };  // fastest 125kHz
enum { DR_UPSLOWEST = DR_SF12, DR_UPFASTEST = DR_FSK };
enum { DR_DNSLOWEST = DR_SF12, DR_DNFASTEST = DR_FSK };
enum { DR_PAGE = DR_PAGE_EU868 };

// Default frequency plan for EU 868MHz ISM band
// Bands:
//  g1 :   1%  14dBm  
//  g2 : 0.1%  14dBm  
//  g3 :  10%  27dBm  
//                 freq             band     datarates
enum { EU868_F1 = 868100000,      // g1   SF7-12 
       EU868_F2 = 868300000,      // g1   SF7-12 FSK SF7/250         
       EU868_F3 = 868500000,      // g1   SF7-12         
       EU868_F4 = 868850000,      // g2   SF7-12         
       EU868_F5 = 869050000,      // g2   SF7-12         
       EU868_F6 = 869525000,      // g3   SF7-12         
       EU868_J4 = 864100000,      // g2   SF7-12  used during join
       EU868_J5 = 864300000,      // g2   SF7-12   ditto
       EU868_J6 = 864500000,      // g2   SF7-12   ditto
};
enum { FREQ_MIN = 863000000,
       FREQ_MAX = 870000000 };

enum { MIN_CHNLS         = 3 };
enum { MAX_CHNLS         = 16 };
enum { MAX_EIRP          = 16 };
enum { FREQ_PING         = EU868_F6 };  // default ping freq
enum { DR_PING           = DR_SF12 };   // default ping DR (same as DNW2)
enum { CHNL_DNW2         = 5 };
enum { FREQ_DNW2         = EU868_F6 };
enum { DR_DNW2           = DR_SF12 };
enum { NCHNL_BCN         = 1 };
enum { FREQ_BCN          = EU868_F6 };
enum { DR_BCN            = DR_SF9 };
enum { AIRTIME_BCN       = 152576 };  // micros / airtime(sf=9,bw=125,plen=17,crc=0,ih=1,npreamble=10)

enum {
    // Beacon layout format EU SF9/125kHz
    OFF_BCN_TIME     = 2,
    OFF_BCN_INFO     = 8,
    LEN_BCN          = 17
};

inline rps_t updr2dndr(dr_t dr, s1_t off) { return (dr_t)os_minmax(DR_DNSLOWEST, (s1_t)dr - off, DR_DNFASTEST); }

#elif defined(CFG_as923) // ==============================================

#define CFG_eu868 1

enum _dr_eu868_t { DR_SF12=0, DR_SF11, DR_SF10, DR_SF9, DR_SF8, DR_SF7, DR_SF7B, DR_FSK, DR_NONE };
enum { DR_MAX125 = DR_SF7 };  // fastest 125kHz
enum { DR_UPSLOWEST = DR_SF12, DR_UPFASTEST = DR_FSK };
enum { DR_DNSLOWEST = DR_SF12, DR_DNFASTEST = DR_FSK };
enum { DR_PAGE = DR_PAGE_AS923 };

// Default frequency plan for AS923 ISM band
//                 freq             band     datarates
enum { AS923_F1 = 923200000,
       AS923_F2 = 923400000,
};
enum { FREQ_MIN = 920000000,
       FREQ_MAX = 928000000 };

enum { MIN_CHNLS         = 2 };
enum { MAX_CHNLS         = 16 };
enum { MAX_EIRP          = 16 };
enum { FREQ_PING         = AS923_F2 };     // default ping freq
enum { FREQ_DNW2         = AS923_F1 };
enum { DR_PING           = DR_SF10 };      // default ping DR (same as DNW2)
enum { DR_DNW2           = DR_SF10 };
enum { NCHNL_BCN         = 1 };
enum { FREQ_BCN          = AS923_F2 };
enum { DR_BCN            = DR_SF9 };
enum { AIRTIME_BCN       = 152576 };       // micros / airtime(sf=9,bw=125,plen=17,crc=0,ih=1,npreamble=10)

enum {
    // Beacon layout format AS SF9/125kHz
    OFF_BCN_TIME     = 2,
    OFF_BCN_INFO     = 8,
    LEN_BCN          = 17
};

inline rps_t updr2dndr(dr_t dr, s1_t off) { return (dr_t)os_minmax(DR_DNSLOWEST, (s1_t)dr - off, DR_DNFASTEST); }

#elif defined(CFG_il915) // ==============================================

#define CFG_eu868 1

enum _dr_eu868_t { DR_SF12=0, DR_SF11, DR_SF10, DR_SF9, DR_SF8, DR_SF7, DR_SF7B, DR_FSK, DR_NONE };
enum { DR_MAX125 = DR_SF7 };  // fastest 125kHz
enum { DR_UPSLOWEST = DR_SF12, DR_UPFASTEST = DR_FSK };
enum { DR_DNSLOWEST = DR_SF12, DR_DNFASTEST = DR_FSK };
enum { DR_PAGE = DR_PAGE_IL915 };

// Default frequency plan for IL915 ISM band
//                 freq             band     datarates
enum { IL915_F1 = 915700000,
       IL915_F2 = 915900000,
       IL915_F3 = 916100000,
       IL915_DN = 916300000,
};
enum { FREQ_MIN = 915500000,
       FREQ_MAX = 916500000 };

enum { MIN_CHNLS         = 3 };
enum { MAX_CHNLS         = 16 };
enum { MAX_EIRP          = 14 };
enum { FREQ_PING         = IL915_DN };     // default ping freq
enum { FREQ_DNW2         = IL915_DN };
enum { DR_PING           = DR_SF12 };      // default ping DR (same as DNW2)
enum { DR_DNW2           = DR_SF12 };
enum { NCHNL_BCN         = 1 };
enum { FREQ_BCN          = IL915_DN };
enum { DR_BCN            = DR_SF9 };
enum { AIRTIME_BCN       = 152576 };       // micros / airtime(sf=9,bw=125,plen=17,crc=0,ih=1,npreamble=10)

enum {
    // Beacon layout format AS SF9/125kHz
    OFF_BCN_TIME     = 2,
    OFF_BCN_INFO     = 8,
    LEN_BCN          = 17
};

inline rps_t updr2dndr(dr_t dr, s1_t off) { return (dr_t)os_minmax(DR_DNSLOWEST, (s1_t)dr - off, DR_DNFASTEST); }

#elif defined(CFG_kr920) // ==============================================

#define CFG_eu868 1

enum _dr_eu868_t { DR_SF12=0, DR_SF11, DR_SF10, DR_SF9, DR_SF8, DR_SF7, DR_NONE };
enum { DR_MAX125 = DR_SF7 };  // fastest 125kHz
enum { DR_UPSLOWEST = DR_SF12, DR_UPFASTEST = DR_SF7 };
enum { DR_DNSLOWEST = DR_SF12, DR_DNFASTEST = DR_SF7 };
enum { DR_PAGE = DR_PAGE_KR920 };

// Default frequency plan for KR920 ISM band
//                 freq             band     datarates
enum { KR920_F1 = 922100000,
       KR920_F2 = 922300000,
       KR920_F3 = 922500000,
       KR920_DN = 921900000,
       KR920_BC = 923100000,
};
enum { FREQ_MIN = 920900000,
       FREQ_MAX = 923300000 };

enum { MIN_CHNLS         = 3 };
enum { MAX_CHNLS         = 16 };
enum { MAX_EIRP          = 14 };
enum { FREQ_PING         = KR920_BC };     // default ping freq
enum { FREQ_DNW2         = KR920_DN };
enum { DR_PING           = DR_SF12 };      // default ping DR (same as DNW2)
enum { DR_DNW2           = DR_SF12 };
enum { NCHNL_BCN         = 1 };
enum { FREQ_BCN          = KR920_BC };
enum { DR_BCN            = DR_SF9 };
enum { AIRTIME_BCN       = 152576 };       // micros / airtime(sf=9,bw=125,plen=17,crc=0,ih=1,npreamble=10)

enum {
    // Beacon layout format AS SF9/125kHz
    OFF_BCN_TIME     = 2,
    OFF_BCN_INFO     = 8,
    LEN_BCN          = 17
};

inline rps_t updr2dndr(dr_t dr, s1_t off) { return (dr_t)os_minmax(DR_DNSLOWEST, (s1_t)dr - off, DR_DNFASTEST); }

#elif defined(CFG_us915) // =========================================

enum _dr_us915_t { DR_SF10=0, DR_SF9, DR_SF8, DR_SF7, DR_SF8C, DR_NONE,
                   // Devices behind a router:
                   DR_SF12CR=8, DR_SF11CR, DR_SF10CR, DR_SF9CR, DR_SF8CR, DR_SF7CR };
enum { DR_MAX125 = DR_SF7 };  // fastest 125kHz
enum { DR_UPSLOWEST = DR_SF10,   DR_UPFASTEST = DR_SF8C };
enum { DR_DNSLOWEST = DR_SF12CR, DR_DNFASTEST = DR_SF7CR };
enum { DR_PAGE = DR_PAGE_US915 };

// Default frequency plan for US 915MHz
enum { US915_125kHz_UPFBASE = 902300000,
       US915_125kHz_UPFSTEP =    200000,
       US915_500kHz_UPFBASE = 903000000,
       US915_500kHz_UPFSTEP =   1600000,
       US915_500kHz_DNFBASE = 923300000,
       US915_500kHz_DNFSTEP =    600000
};
enum { BCN_FREQ_BASE = 923300000,
       BCN_FREQ_STEP =    600000
};
enum { FREQ_MIN = 902000000,
       FREQ_MAX = 928000000
};

enum { MIN_CHNLS         = 72 };
enum { MAX_CHNLS         = 72 };
enum { MAX_EIRP          = 30 };
enum { FREQ_PING         = 0 }; // default ping freq - 0 => default rotating
enum { DR_PING           = DR_SF12CR };       // default ping DR (same as DNW2)
enum { FREQ_DNW2         = US915_500kHz_DNFBASE + 0*US915_500kHz_DNFSTEP };
enum { DR_DNW2           = DR_SF12CR };
enum { NCHNL_BCN         = 8 }; // default scheme hops over 8 channels
enum { FREQ_BCN          = 0 };
enum { DR_BCN            = DR_SF12CR };
enum { AIRTIME_BCN       = 305152 };  // micros / airtime(sf=12,bw=500,plen=23,crc=0,ih=1,npreamble=10)

enum {
    // Beacon layout format US SF12/500kHz
    OFF_BCN_TIME     = 5,
    OFF_BCN_INFO     = 11,
    LEN_BCN          = 23
};

inline rps_t updr2dndr(dr_t dr, s1_t off) { return (dr_t)os_minmax(DR_DNSLOWEST, (s1_t)dr + (DR_SF10CR - DR_SF10) - off, DR_DNFASTEST); }

#elif defined(CFG_au915) // =========================================

#define CFG_us915 1   // make the code behave like US

enum _dr_us915_t { DR_SF12=0, DR_SF11, DR_SF10, DR_SF9, DR_SF8, DR_SF7, DR_SF8C, DR_NONE,
                   // Devices behind a router:
                   DR_SF12CR=8, DR_SF11CR, DR_SF10CR, DR_SF9CR, DR_SF8CR, DR_SF7CR };
enum { DR_MAX125 = DR_SF7 };  // fastest 125kHz
enum { DR_UPSLOWEST = DR_SF12,   DR_UPFASTEST = DR_SF8C };
enum { DR_DNSLOWEST = DR_SF12CR, DR_DNFASTEST = DR_SF7CR };
enum { DR_PAGE = DR_PAGE_AU915 };

// Default frequency plan for US 915MHz
enum { US915_125kHz_UPFBASE = 915200000,
       US915_125kHz_UPFSTEP =    200000,
       US915_500kHz_UPFBASE = 915900000,
       US915_500kHz_UPFSTEP =   1600000,
       US915_500kHz_DNFBASE = 923300000,
       US915_500kHz_DNFSTEP =    600000
};
enum { BCN_FREQ_BASE = 923300000,
       BCN_FREQ_STEP =    600000
};
enum { FREQ_MIN = 915000000,
       FREQ_MAX = 928000000
};
enum { MIN_CHNLS         = 72 };
enum { MAX_CHNLS         = 72 };
enum { MAX_EIRP          = 30 };
enum { FREQ_PING         = 0 }; // default ping freq - 0 => default rotating
enum { DR_PING           = DR_SF10CR };       // default ping DR (same as DNW2)
enum { FREQ_DNW2         = US915_500kHz_DNFBASE + 0*US915_500kHz_DNFSTEP };
enum { DR_DNW2           = DR_SF12CR };
enum { NCHNL_BCN         = 8 }; // default scheme hops over 8 channels
enum { FREQ_BCN          = 0 };
enum { DR_BCN            = DR_SF10CR };
enum { AIRTIME_BCN       = 76288 };  // micros / airtime(sf=10,bw=500,plen=19,crc=0,ih=1,npreamble=10)

enum {
    // Beacon layout format AU SF10/500kHz
    OFF_BCN_TIME     = 3,
    OFF_BCN_INFO     = 9,
    LEN_BCN          = 19
};

inline rps_t updr2dndr(dr_t dr, s1_t off) { return (dr_t)os_minmax(DR_DNSLOWEST, (s1_t)dr + (DR_SF12CR - DR_SF12) - off, DR_DNFASTEST); }

#elif defined(CFG_cn470) // =========================================

#define CFG_us915 1   // make the code behave like US

enum _dr_us915_t { DR_SF12=0, DR_SF11, DR_SF10, DR_SF9, DR_SF8, DR_SF7, DR_NONE };
enum { DR_MAX125 = DR_SF7 };  // fastest 125kHz
enum { DR_UPSLOWEST = DR_SF12, DR_UPFASTEST = DR_SF7 };
enum { DR_DNSLOWEST = DR_SF12, DR_DNFASTEST = DR_SF7 };
enum { DR_PAGE = DR_PAGE_CN470 };

// Default frequency plan for US 915MHz
enum { CN470_125kHz_UPFBASE = 470200000,
       CN470_125kHz_UPFSTEP =    200000,
       CN470_125kHz_DNFBASE = 500300000,
       CN470_125kHz_DNFSTEP =    200000
};
enum { BCN_FREQ_BASE = 508300000,
       BCN_FREQ_STEP =    200000
};
enum { FREQ_MIN = 470000000,
       FREQ_MAX = 510000000
};
enum { MIN_CHNLS         = 96 };
enum { MAX_CHNLS         = 96 };
enum { MAX_EIRP          = 12 };
enum { FREQ_PING         =  0 }; // default ping freq - 0 => default rotating
enum { DR_PING           = DR_SF12 };       // default ping DR (same as DNW2)
enum { FREQ_DNW2         = 505300000 };
enum { DR_DNW2           = DR_SF12 };
enum { NCHNL_BCN         = 8 }; // default scheme hops over 8 channels
enum { FREQ_BCN          = 0 };
enum { DR_BCN            = DR_SF10 };
enum { AIRTIME_BCN       = 305152 };  // micros / airtime(sf=10,bw=125,plen=19,crc=0,ih=1,npreamble=10)

enum {
    // Beacon layout format CN SF10/125kHz
    OFF_BCN_TIME     = 3,
    OFF_BCN_INFO     = 9,
    LEN_BCN          = 19
};

inline rps_t updr2dndr(dr_t dr, s1_t off) { return (dr_t)os_minmax(DR_DNSLOWEST, (s1_t)dr - off, DR_DNFASTEST); }

#endif // ===================================================

enum {
    // Convenience offsets derived from region specific beacon layout
    // RFU fields are not mapped out - they are not checked
    OFF_BCN_CRC1 = OFF_BCN_INFO-2,
    OFF_BCN_LAT  = OFF_BCN_INFO+1,
    OFF_BCN_LON  = OFF_BCN_INFO+4,
    OFF_BCN_CRC2 = LEN_BCN-2,
};


enum {
    // Join Request frame format
    OFF_JR_HDR      = 0,
    OFF_JR_JOINEUI  = 1,
    OFF_JR_DEVEUI   = 9,
    OFF_JR_DEVNONCE = 17,
    OFF_JR_MIC      = 19,
    LEN_JR          = 23
};
enum {
    // Join Accept frame format
    OFF_JA_HDR      = 0,
    OFF_JA_JOINNONCE= 1,
    OFF_JA_NETID    = 4,
    OFF_JA_DEVADDR  = 7,
    OFF_JA_RFU      = 11,
    OFF_JA_DLSET    = 11,
    OFF_JA_RXDLY    = 12,
    OFF_CFLIST      = 13,
    LEN_JA          = 17,
    LEN_JAEXT       = 17+16
};
enum {
    // Bitfields in DLsettings octet (OFF_JA_DLSET)
    JA_DLS_OPTNEG   = 0x80,
    JA_DLS_RX1DROFF = 0x70,
    JA_DLS_RX2DR    = 0x0F,
};
enum {
    // Data frame format
    OFF_DAT_HDR      = 0,
    OFF_DAT_ADDR     = 1,
    OFF_DAT_FCT      = 5,
    OFF_DAT_SEQNO    = 6,
    OFF_DAT_OPTS     = 8,
};
enum { MAX_LEN_PAYLOAD = MAX_LEN_FRAME-(int)OFF_DAT_OPTS-4 };
enum {
    // Bitfields in frame format octet
    HDR_FTYPE   = 0xE0,
    HDR_RFU     = 0x1C,
    HDR_MAJOR   = 0x03
};
enum { HDR_FTYPE_DNFLAG = 0x20 };  // flags DN frame except for HDR_FTYPE_PROP
enum {
    // Values of frame type bit field
    HDR_FTYPE_JREQ   = 0x00,
    HDR_FTYPE_JACC   = 0x20,
    HDR_FTYPE_DAUP   = 0x40,  // data (unconfirmed) up
    HDR_FTYPE_DADN   = 0x60,  // data (unconfirmed) dn
    HDR_FTYPE_DCUP   = 0x80,  // data confirmed up
    HDR_FTYPE_DCDN   = 0xA0,  // data confirmed dn
    HDR_FTYPE_REJOIN = 0xC0,  // rejoin for roaming
    HDR_FTYPE_PROP   = 0xE0
};
enum {
    HDR_MAJOR_V1 = 0x00,
};
enum {
    // Bitfields in frame control octet
    FCT_ADREN  = 0x80,
    FCT_ADRARQ = 0x40,
    FCT_ACK    = 0x20,
    FCT_MORE   = 0x10,   // also in UP direction: Class B indicator
    FCT_OPTLEN = 0x0F,
};
enum {
    // In UP direction: signals class B enabled
    FCT_CLASSB = FCT_MORE
};
enum {
    NWKID_MASK = (int)0xFE000000,
    NWKID_BITS = 7
};

// MAC uplink commands   downwlink too
enum {
    // Class A               1=1.1, - 1.0.2
    MCMD_LCHK_REQ = 0x02, // - link check request : -
    MCMD_LADR_ANS = 0x03, // - link ADR answer    : u1:7-3:RFU, 3/2/1: pow/DR/Ch ACK
    MCMD_DCAP_ANS = 0x04, // - duty cycle answer  : -
    MCMD_DN2P_ANS = 0x05, // - 2nd DN slot status : u1:7-2:RFU  2/1/0:rx1off,rx2dr/channel ack
    MCMD_DEVS_ANS = 0x06, // - device status ans  : u1:battery 0,1-254,255=?, u1:7-6:RFU,5-0:margin(-32..31)
    MCMD_SNCH_ANS = 0x07, // - ack new channel    : u1: 7-2=RFU, 1/0:DR/freq ACK
    MCMD_RXTM_ANS = 0x08, // - ack RX delay       : -
    MCMD_DNFQ_ANS = 0x0A, // - ack DN freq        : u1: 7-2=RFU, 1/0:Freq/Chnl ACK
    MCMD_RKEY_IND = 0x0B, // - rekey indication   : u1: version, u1: nonce, u1: opt1, [n opts...]
    MCMD_ADRP_ANS = 0x0C, // - adr params         : -
    MCMD_TIME_REQ = 0x0D, // - time request       : -
    // Class B
    MCMD_PITV_REQ = 0x10, // - ping interval      : u1: 7-3=RFU, 2-0:interval
    MCMD_PNGC_ANS = 0x11, // - ack ping freq/dr   : u1: 7-2:RFU, 1:drAck, 0:freqAck
    MCMD_BCNI_REQ = 0x12, // - next beacon start  : -                                     -- DEPRECATED
    MCMD_BCNF_ANS = 0x13, // - ack beacon freq    : u1: 0:ack
    // Class C
    MCMD_DEVMD_CONF=0x20, // 1 device mode confirm: u1: 0=A, 2=C
};

// MAC downlink commands
enum {
    // Class A
    MCMD_LCHK_ANS = 0x02, // - link check answer  : u1:margin 0-254,255=unknown margin / u1:gwcnt
    MCMD_LADR_REQ = 0x03, // - link ADR request   : u1:DR/TXPow, u2:chmask, u1:chpage/repeat
    MCMD_DCAP_REQ = 0x04, // - duty cycle cap     : u1:255 dead [7-4]:RFU, [3-0]:cap 2^-k
    MCMD_DN2P_SET = 0x05, // - 2nd DN window param: u1:7:RFU, 6-4:rx1droff, 3-0:rx2dr, u3:freq
    MCMD_DEVS_REQ = 0x06, // - device status req  : -
    MCMD_SNCH_REQ = 0x07, // - set new channel    : u1:chidx, u3:freq, u1:DRrange
    MCMD_RXTM_REQ = 0x08, // - change RX delay    : u1:0-3:delay, 4-7:RFU
    MCMD_DNFQ_REQ = 0x0A, // - dn link freq       : u1:chnl, u3:freq
    MCMD_RKEY_CNF = 0x0B, // - reset confirmation : u1: opt1, [n opts...]
    MCMD_ADRP_REQ = 0x0C, // - adr params         : u1: 7-4: limit_exp, 3-0: delay_exp
    MCMD_TIME_ANS = 0x0D, // - time answer        : u4:epoch_secs, u1:fracs
    // Class B		     - 
    MCMD_PITV_ANS = 0x10, // - ping interval ack  : -
    MCMD_PNGC_REQ = 0x11, // - set ping freq/dr   : u3: freq, u1:7-4:RFU/3-0:datarate
    MCMD_BCNI_ANS = 0x12, // - next beacon start  : u2: delay(in TUNIT millis), u1:channel -- DEPRECATED
    MCMD_BCNF_REQ = 0x13, // - set beacon freq    : u3:freq
    // Class C
    MCMD_DEVMD_IND= 0x20, // 1 device mode        : u1: 0=A, 2=C
};

enum {
    MCMD_BCNI_TUNIT = 30  // time unit of delay value in millis
};
enum {
    MCMD_LADR_ANS_RFU    = 0xF8, // RFU bits
    MCMD_LADR_ANS_POWACK = 0x04, // 0=not supported power level
    MCMD_LADR_ANS_DRACK  = 0x02, // 0=unknown data rate
    MCMD_LADR_ANS_CHACK  = 0x01, // 0=unknown channel enabled
};
enum {
    MCMD_DN2P_ANS_PEND   = 0x80, // ACK is pending
    MCMD_DN2P_ANS_REPLY  = 0x40, // ACK has been added to foptsup (to keep ORDER!)
    MCMD_DN2P_ANS_RFU    = 0xF8, // RFU bits
    MCMD_DN2P_ANS_OFFACK = 0x04, // 0=unknown RX1 data rate offset
    MCMD_DN2P_ANS_DRACK  = 0x02, // 0=unknown RX2 data rate
    MCMD_DN2P_ANS_CHACK  = 0x01, // 0=unknown channel enabled
};
enum {
    MCMD_SNCH_ANS_PEND   = 0x80, // ACK is pending
    MCMD_SNCH_ANS_RFU    = 0xFC, // RFU bits
    MCMD_SNCH_ANS_DRACK  = 0x02, // 0=unknown data rate
    MCMD_SNCH_ANS_FQACK  = 0x01, // 0=rejected channel frequency
};
enum {
    MCMD_DNFQ_ANS_PEND   = 0x80, // ACK is pending
    MCMD_DNFQ_ANS_RFU    = 0xFC, // RFU bits
    MCMD_DNFQ_ANS_CHACK  = 0x02, // 0=unknown channel
    MCMD_DNFQ_ANS_FQACK  = 0x01, // 0=rejected channel frequency
};
enum {
    MCMD_PNGC_ANS_PEND  = 0x80,  // ACK is pending
    MCMD_PNGC_ANS_RFU   = 0xFC,
    MCMD_PNGC_ANS_DRACK = 0x02,
    MCMD_PNGC_ANS_FQACK = 0x01
};
enum {
    MCMD_BCNF_ANS_PEND  = 0x80,
    MCMD_BCNF_ANS_RFU   = 0xFE,
    MCMD_BCNF_ANS_FQACK = 0x01
};

enum {
    MCMD_DEVS_EXT_POWER   = 0x00, // external power supply
    MCMD_DEVS_BATT_MIN    = 0x01, // min battery value
    MCMD_DEVS_BATT_MAX    = 0xFE, // max battery value
    MCMD_DEVS_BATT_NOINFO = 0xFF, // unknown battery level
};

// Bit fields byte#3 of MCMD_LADR_REQ payload
enum {
    MCMD_LADR_CHP_ALLON   = 0x60,  // enable all channels
};
enum {
    MCMD_LADR_CHP_BLK8    = 0x50,  // special channel page enable, control blocks of 8+1
    MCMD_LADR_CHP_125ON   = 0x60,  // special channel page enable, bits applied to 64..71
    MCMD_LADR_CHP_125OFF  = 0x70,  //  ditto
    MCMD_LADR_N3RFU_MASK  = 0x80,
    MCMD_LADR_CHPAGE_MASK = 0x70,
    MCMD_LADR_REPEAT_MASK = 0x0F,
    MCMD_LADR_CHPAGE_SHIFT= 4,
    MCMD_LADR_REPEAT_1    = 0x01,
    MCMD_LADR_CHPAGE_1    = 0x10
};
// Bit fields byte#0 of MCMD_LADR_REQ payload
enum {
    MCMD_LADR_DR_MASK    = 0xF0,
    MCMD_LADR_POW_MASK   = 0x0F,
    MCMD_LADR_DR_SHIFT   = 4,
    MCMD_LADR_POW_SHIFT  = 0,
};

enum {
    MCMD_RKEY_VERSION_1_1    = 0x01,  // LoRaWAN 1.1
};

// Device address
typedef u4_t devaddr_t;

// RX quality (device)
enum { RSSI_OFF=64, SNR_SCALEUP=4 };

inline sf_t  getSf   (rps_t params)            { return   (sf_t)(params &  0x7); }
inline rps_t setSf   (rps_t params, sf_t sf)   { return (rps_t)((params & ~0x7) | sf); }
inline bw_t  getBw   (rps_t params)            { return  (bw_t)((params >> 3) & 0x3); }
inline rps_t setBw   (rps_t params, bw_t cr)   { return (rps_t)((params & ~0x18) | (cr<<3)); }
inline cr_t  getCr   (rps_t params)            { return  (cr_t)((params >> 5) & 0x3); }
inline rps_t setCr   (rps_t params, cr_t cr)   { return (rps_t)((params & ~0x60) | (cr<<5)); }
inline int   getNocrc(rps_t params)            { return        ((params >> 7) & 0x1); }
inline rps_t setNocrc(rps_t params, int nocrc) { return (rps_t)((params & ~0x80) | (nocrc<<7)); }
inline int   getIh   (rps_t params)            { return        ((params >> 8) & 0xFF); }
inline rps_t setIh   (rps_t params, int ih)    { return (rps_t)((params & ~0xFF00) | (ih<<8)); }
inline rps_t makeRps (sf_t sf, bw_t bw, cr_t cr, int ih, int nocrc) {
    return sf | (bw<<3) | (cr<<5) | (nocrc?(1<<7):0) | ((ih&0xFF)<<8);
}
#define MAKERPS(sf,bw,cr,ih,nocrc) ((rps_t)((sf) | ((bw)<<3) | ((cr)<<5) | ((nocrc)?(1<<7):0) | ((ih&0xFF)<<8)))
// Two frames with params r1/r2 would interfere on air: same SFx + BWx 
inline int sameSfBw(rps_t r1, rps_t r2) { return ((r1^r2)&0x1F) == 0; }

extern const u1_t _DR2RPS_CRC[];
inline rps_t updr2rps (dr_t dr) { return (rps_t)_DR2RPS_CRC[dr+1]; }
inline rps_t dndr2rps (dr_t dr) { return setNocrc(updr2rps(dr),1); }
inline int isFasterDR (dr_t dr1, dr_t dr2) { return dr1 > dr2; }
inline int isSlowerDR (dr_t dr1, dr_t dr2) { return dr1 < dr2; }
inline dr_t  incDR    (dr_t dr) { return _DR2RPS_CRC[dr+2]==ILLEGAL_RPS ? dr : (dr_t)(dr+1); } // increase data rate
inline dr_t  decDR    (dr_t dr) { return _DR2RPS_CRC[dr  ]==ILLEGAL_RPS ? dr : (dr_t)(dr-1); } // decrease data rate
inline dr_t  assertDR (dr_t dr) { return _DR2RPS_CRC[dr+1]==ILLEGAL_RPS ? DR_UPFASTEST : dr; } // force into a valid DR
inline bit_t validDR  (dr_t dr) { return _DR2RPS_CRC[dr+1]!=ILLEGAL_RPS; } // in range
inline dr_t  lowerDR  (dr_t dr, u1_t n) { while(n--){dr=decDR(dr);} return dr; } // decrease data rate by n steps

//
// BEG: Keep in sync with lorabase.hpp
// ================================================================================


// Convert between dBm values and power codes (MCMD_LADR_XdBm)
s1_t pow2dBm (u1_t mcmd_ladr_p1);
// Calculate airtime
ostime_t calcAirTime (rps_t rps, u1_t plen);
// Sensitivity at given SF/BW
int getSensitivity (rps_t rps);


#endif // _lorabase_h_
