// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#include "lmic.h"

#include "lwmux/lwmux.h"

#define TESTMODE_PORT        224

#define TESTMODE_MAXCNT      192
#define TESTMODE_TIMEOUT    (30*60) // (sec)
#define TESTMODE_INTERVAL    5      // (sec)

// commands
#define TESTCMD_STOP         0x00
#define TESTCMD_START        0x01
#define TESTCMD_CONFIRMED    0x02
#define TESTCMD_UNCONFIRMED  0x03
#define TESTCMD_ECHO         0x04
#define TESTCMD_LINKCHECK    0x05
#define TESTCMD_JOIN         0x06
#define TESTCMD_CW           0x07
#define TESTCMD_RFU          0x08 // 0x08-0x7F
#define TESTCMD_VENDOR       0x80 // 0x80-0xFF

static struct {
    uint8_t active;    // test mode active
    uint8_t join;      // trigger join request
    uint8_t confirmed; // request confirmation for uplinks
    uint16_t dncnt;    // downlink counter (to be echoed)
    uint16_t upcnt;    // number of up messages w/o downlink (max 192)
    ostime_t dntime;   // time of last downlink (max 30 minutes)
    osjob_t timer;     // cw timeout or uplink timer
    lwm_job lwmjob;    // uplink job
} testmode;

static void starttestmode (void) {
    debug_printf("TEST MODE START\r\n");
    testmode.active = 1;
    // disable duty cycle
    LMIC_enableFastJoin();
    LMIC_disableDC();
    // disable application messages
    lwm_setpriority(LWM_PRIO_MAX);
}

static void stoptestmode (void) {
    debug_printf("TEST MODE END\r\n");
    testmode.active = 0;
    // stop uplinks
    os_clearCallback(&testmode.timer);
    // reset duty cycle limitations (XXX)
#if defined(CFG_eu868)
    LMIC.noDC = 0;
    LMIC.bands[BAND_MILLI].txcap = 1000;  // 0.1%
#endif // defined(CFG_eu868)
    LMIC.opmode &= ~OP_TESTMODE;
    // enable application traffic
    lwm_setpriority(LWM_PRIO_MIN);
}

static bool txfunc (lwm_txinfo* txi) {
    // decrement sequence counter for exact frame repetition if last confirmed up frame has not been ack'ed
    if (testmode.confirmed && LMIC.txrxFlags & TXRX_NACK) {
	LMIC.seqnoUp -= 1;
    }
    txi->port = TESTMODE_PORT;
    txi->confirmed = testmode.confirmed;
    if (LMIC.frame[LMIC.dataBeg] == TESTCMD_ECHO) {
        txi->data[0] = TESTCMD_ECHO;
        for( int i = 1; i < LMIC.dataLen; i++ ) {
            txi->data[i] = LMIC.frame[LMIC.dataBeg + i] + 1;
        }
	txi->dlen = LMIC.dataLen;
    } else {
	txi->data[0] = testmode.dncnt >> 8; // fill in downlink_counter
	txi->data[1] = testmode.dncnt;      // (2 bytes, big endian)
	txi->dlen = 2;
    }
    LMIC_setAdrMode(1);
    debug_printf("TESTMODE UPLINK (%sconfirmed, len=%d): %h\r\n", (testmode.confirmed) ? "" : "un", txi->dlen, txi->data, txi->dlen);
    return true;
}

static void uplink (osjob_t* job) {
    if (testmode.join) {
	testmode.join = 0;
	stoptestmode(); // (activation command will be resent after join)
        // XXX - use explicit rejoin
	lwm_setmode(LWM_MODE_SHUTDOWN);
	lwm_setmode(LWM_MODE_NORMAL);
    } else {
	lwm_request_send(&testmode.lwmjob, LWM_PRIO_MAX, txfunc);
    }
}

static void stopcw (osjob_t* job) {
    // stop continuous wave
    os_radio(RADIO_RST);
    // continue frame reporting...
    uplink(job);
}

// referenced by tabs / rm_event()
void testmode_handleEvent (ev_t ev) {
    switch (ev) {
	case EV_TXCOMPLETE: {
	    if (((LMIC.txrxFlags & TXRX_PORT) && LMIC.frame[LMIC.dataBeg - 1] == TESTMODE_PORT && LMIC.dataLen > 0) ||
		((LMIC.txrxFlags & (TXRX_DNW1|TXRX_DNW2)) && (LMIC.txrxFlags & TXRX_PORT) == 0 && testmode.active)) {
		unsigned char *buf = LMIC.frame + LMIC.dataBeg;

		debug_printf("TESTMODE DOWNLINK (%s%slen=%d): % h\r\n",
			     (LMIC.txrxFlags & TXRX_ACK) ? "ACK, " : "",
			     (LMIC.txrxFlags & TXRX_NACK) ? "NACK, " : "",
			     LMIC.dataLen, buf, LMIC.dataLen);

		if (testmode.active) {
		    // save timestamp of last server command
		    testmode.dntime = os_getTime();

		    // reset uplink counter
		    testmode.upcnt = 0;

		    // update downlink counter
		    testmode.dncnt += 1;

		    // dispatch test commands
		    switch (buf[0]) {

			case TESTCMD_STOP: // deactivate test mode
			    if (LMIC.dataLen == 1) {
				stoptestmode();
			    }
			    break;

			case TESTCMD_CONFIRMED: // activate confirmations
			    if (LMIC.dataLen == 1) {
				testmode.confirmed = 1;
			    }
			    break;

			case TESTCMD_UNCONFIRMED: // deactivate confirmations
			    if (LMIC.dataLen == 1) {
				testmode.confirmed = 0;
			    }
			    break;

			case TESTCMD_LINKCHECK: // XXX undocumented?!?
			    if (LMIC.dataLen == 1) {
				// XXX
			    }
			    break;

			case TESTCMD_JOIN: // trigger join request
			    if (LMIC.dataLen == 1) {
				testmode.join = 1; // defer join since rm is considered still busy
			    }
			    break;

			case TESTCMD_ECHO: // modify and echo frame
			    LMIC.pendTxData[0] = buf[0];
			    for (int i = 1; i < LMIC.dataLen; i++) {
				LMIC.pendTxData[i] = buf[i] + 1;
			    }
			    LMIC.pendTxLen = LMIC.dataLen;
			    break;

			case TESTCMD_CW: // continous wave
			    // set timeout and parameters
			    os_setApproxTimedCallback(&testmode.timer, os_getTime() + sec2osticks((buf[1] << 8) | buf[2]), stopcw); // duration [s]
			    LMIC.freq = ((buf[3] << 16) | (buf[4] << 8) | buf[5]) * 100; // [Hz]
			    LMIC.txpow = buf[6]; // dBm
			    // start continuous wave
			    os_radio(RADIO_TXCW);
			    return; // no uplink now
		    }
		} else { // test mode not active
		    if (LMIC.dataLen == 4 && os_rlsbf4(buf) == 0x01010101) { // activate test mode
			// reset downlink counter
			testmode.dncnt = 0;
			// begin test mode
			starttestmode();
		    }
		}
	    } else { // no downlink data or not a test command
		if (testmode.active &&
		    (++testmode.upcnt > TESTMODE_MAXCNT ||
		     (os_getTime() - testmode.dntime) > sec2osticks(TESTMODE_TIMEOUT))) {
		    // test mode timed out
		    debug_printf("TEST MODE TIMEOUT\r\n");
		    stoptestmode();
		}
	    }

	    if (testmode.active) {
		// schedule next uplink (or join request)
		os_setApproxTimedCallback(&testmode.timer, os_getTime() + sec2osticks(TESTMODE_INTERVAL), uplink);
	    }
	}

	default: // ignore other events
	    break;
    }
}
