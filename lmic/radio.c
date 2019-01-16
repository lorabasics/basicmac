// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

//#define DEBUG_TX
//#define DEBUG_RX

#include "board.h"
#include "lmic.h"
#include "backtrace.h"

// ----------------------------------------
// RADIO STATE
static struct {
    ostime_t irqtime;
    osjob_t irqjob;
} state;

static void radio_irq_func (osjob_t* j) {
    // call radio-specific irq processing function
    radio_irq_process(state.irqtime);

    // put radio back to sleep
    radio_sleep();

    // run os job (use preset func ptr)
    // (an eventually scheduled timeout job will be replaced)
    os_setCallback(&LMIC.osjob, LMIC.osjob.func);

#ifdef CFG_testpin
    if (LMIC.testpinMode & 0x80) {
	// clear GPIO signal
	CFG_PIN_DEFAULT(CFG_testpin);
	// reset signaling request
	LMIC.testpinMode = 0;
    }
    if( LMIC.testpinMode == 2 ) {
	// enable for next IRQ which should be RX
	LMIC.testpinMode = 1;
    }
#endif
}

#define IRQ_GUARD_TICKS sec2osticks(15)

// guard timeout in case interrupt is not asserted by radio
static void radio_irq_guard (osjob_t* j) {
    // radio is in weird state -- let's reboot
    ASSERT(0);
}

// called by hal exti IRQ handler (radio is in STANDBY mode now)
void radio_irq_handler (u1_t diomask) {
#ifdef CFG_testpin
    // signal (txdone) irq on GPIO line
    if (LMIC.testpinMode == 1) {
	CFG_PIN_VAL(CFG_testpin, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE, 1);
	LMIC.testpinMode |= 0x80;
    }
#endif
    // save current time
    state.irqtime = os_getTime();

    // check number of interrupt sources
    ASSERT( __builtin_popcount(diomask) == 1 );

    BACKTRACE();

    // disable antenna switch
    hal_pin_rxtx(-1);

    // disable IRQs in HAL
    hal_irqmask_set(0);

    // schedule irq job
    os_setCallback(&state.irqjob, radio_irq_func);
}

// stop radio, disarm interrupts, cancel jobs
static void radio_stop (void) {
    hal_disableIRQs();
    // put radio to sleep
    radio_sleep();
    // disable antenna switch
    hal_pin_rxtx(-1);
    // disable IRQs in HAL
    hal_irqmask_set(0);
    // cancel radio job
    os_clearCallback(&state.irqjob);
    hal_enableIRQs();
}

void os_radio (u1_t mode) {
    switch (mode) {
	case RADIO_RST:
	    radio_stop();
	    break;

	case RADIO_TX:
	    radio_stop();
	    // set timeout for tx operation
	    os_setTimedCallback(&state.irqjob, os_getTime() + IRQ_GUARD_TICKS, radio_irq_guard);
#ifdef DEBUG_TX
	    debug_printf("TX[freq=%.1F,sf=%d,bw=%s,len=%d%s]: %h\r\n",
			 LMIC.freq, 6,
			 getSf(LMIC.rps) + 6, ("125\0" "250\0" "500\0" "rfu") + (4 * getBw(LMIC.rps)),
			 LMIC.dataLen,
			 (LMIC.pendTxPort != 0 && (LMIC.frame[OFF_DAT_FCT] & FCT_ADRARQ)) ? ",ADRARQ" : "",
			 LMIC.frame, LMIC.dataLen);
#endif
	    // transmit frame now
	    radio_starttx(0);
	    break;

	case RADIO_RX:
	    radio_stop();
	    // set timeout for rx operation
	    os_setTimedCallback(&state.irqjob, LMIC.rxtime + IRQ_GUARD_TICKS, radio_irq_guard);
	    // receive frame now (exactly at rxtime)
	    radio_startrx(0);
#ifdef CFG_testpin
	    // signal (rxstart) irq on GPIO line
	    if (LMIC.testpinMode == 3) {
		CFG_PIN_VAL(CFG_testpin, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE, 1);
		LMIC.testpinMode |= 0x80;
	    }
#endif
	    break;

	case RADIO_RXON:
	    radio_stop();
	    // start scanning for frame now
	    radio_startrx(1);
	    break;

	case RADIO_TXCW:
	    radio_stop();
	    // transmit CW continuous -- end with os_radio(RADIO_RST)
	    radio_starttx(1);
	    break;
    }
}
