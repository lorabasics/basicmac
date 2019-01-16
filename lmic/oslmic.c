// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#include "lmic.h"

// RUNTIME STATE
static struct {
    osjob_t* scheduledjobs;
    unsigned int exact;
    u1_t randbuf[16];
} OS;

// job flags
enum {
    FLAG_APPROX = (1 << 0),
};

void os_init (void* bootarg) {
    memset(&OS, 0x00, sizeof(OS));
    hal_init(bootarg);
    // init prng
    memcpy(OS.randbuf, __TIME__, 8);
    os_getDevEui(OS.randbuf + 8);
    OS.randbuf[0] = 16;
#ifndef CFG_noradio
    radio_init();
#endif
    LMIC_init();
}

// return next random byte derived from seed buffer
// (buf[0] holds index of next byte to be returned 1-16)
u1_t os_getRndU1 (void) {
    u1_t i = OS.randbuf[0];
    ASSERT(i != 0);
    if (i == 16) {
        os_aes(AES_ENC, OS.randbuf, 16); // encrypt seed with any key
        i = 0;
    }
    u1_t v = OS.randbuf[i++];
    OS.randbuf[0] = i;
    return v;
}

bit_t os_cca (u2_t rps, u4_t freq) { //XXX:this belongs into os_radio module
    return 0;  // never grant access
}

u1_t os_getBattLevel (void) {
    return hal_getBattLevel();
}

ostime_t os_getTime () {
    return hal_ticks();
}

osxtime_t os_getXTime () {
    return hal_xticks();
}

osxtime_t os_time2XTime (ostime_t t, osxtime_t context) {
    return context + ((t - (ostime_t) context));
}

// unlink job from queue, return 1 if removed
static int unlinkjob (osjob_t** pnext, osjob_t* job) {
    for( ; *pnext; pnext = &((*pnext)->next)) {
        if(*pnext == job) { // unlink
            *pnext = job->next;
	    if ((job->flags & FLAG_APPROX) == 0) {
		OS.exact -= 1;
	    }
            return 1;
        }
    }
    return 0;
}

// clear scheduled job, return 1 if job was removed
int os_clearCallback (osjob_t* job) {
    hal_disableIRQs();
    int r = unlinkjob(&OS.scheduledjobs, job);
    hal_enableIRQs();
    return r;
}

// schedule timed job
static void setTimedCallback (osjob_t* job, ostime_t time, osjobcb_t cb, unsigned int flags) {
    osjob_t** pnext;
    hal_disableIRQs();
    // remove if job was already queued
    unlinkjob(&OS.scheduledjobs, job);
    // fill-in job
    job->deadline = time;
    job->func = cb;
    job->next = NULL;
    job->flags = flags;
    if ((flags & FLAG_APPROX) == 0) {
	OS.exact += 1;
    }
    // insert into schedule
    for(pnext=&OS.scheduledjobs; *pnext; pnext=&((*pnext)->next)) {
        if((*pnext)->deadline - time > 0) { // (cmp diff, not abs!)
            // enqueue before next element and stop
            job->next = *pnext;
            break;
        }
    }
    *pnext = job;
    hal_enableIRQs();
}

// schedule immediately runnable job
void os_setCallback (osjob_t* job, osjobcb_t cb) {
    setTimedCallback(job, os_getTime(), cb, 0);
}

void os_setTimedCallback (osjob_t* job, ostime_t time, osjobcb_t cb) {
    setTimedCallback(job, time, cb, 0);
}

void os_setApproxTimedCallback (xref2osjob_t job, ostime_t time, osjobcb_t cb) {
    setTimedCallback(job, time, cb, FLAG_APPROX);
}

// execute 1 job from timer or run queue, or sleep if nothing is pending
void os_runstep (void) {
    osjob_t* j = NULL;
    hal_disableIRQs();
    // check for runnable jobs
    if (OS.scheduledjobs) {
	if (hal_sleep(OS.exact ? HAL_SLEEP_EXACT : HAL_SLEEP_APPROX, OS.scheduledjobs->deadline) == 0) {
	    j = OS.scheduledjobs;
	    OS.scheduledjobs = j->next;
	    if ((j->flags & FLAG_APPROX) == 0) {
		OS.exact -= 1;
	    }
	}
    } else { // nothing pending
	hal_sleep(HAL_SLEEP_FOREVER, 0);
    }
    hal_enableIRQs();
    if (j) { // run job callback
	hal_watchcount(30); // max 60 sec
	j->func(j);
	hal_watchcount(0);
    }
}

// execute jobs from timer and from run queue
void os_runloop (void) {
    while (1) {
	os_runstep();
    }
}
