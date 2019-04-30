//  __ __   ___ __ __    _________________________________
// (_ |_ |V| | |_ /  |_| (C) 2018-2018 Semtech Corporation
// __)|__| | | |__\__| |               All rights reserved

#include "lmic.h"
#include "peripherals.h"
#include "boottab.h"

#if defined(SVC_eefs)
#include "eefs/eefs.h"
#endif

#if defined(SVC_frag)
#include "fuota/frag.h"
#endif

enum {
    SVC_DEBUG_STR = BOOT_SVC_FWBASE,
    SVC_TICKS,
    SVC_SLEEP,
    SVC_TX,
    SVC_RX_START,
    SVC_RX_DO,
    SVC_RX_DONE,
    SVC_RNG_SEED,
    SVC_VTOR,
    SVC_IRQ,
    SVC_PIO,
    SVC_UART,
};

static struct {
    osjob_t rjob; // radio job
    int rand;
    boot_boottab* boottab;
    unsigned int irqlevel;
    osxtime_t xnow_cached;
} sim;

static uint32_t irqvector[]; // fwd decl

static uint64_t svc64 (uint32_t id, uint32_t p1, uint32_t p2, uint32_t p3) {
    return ((uint64_t (*) (uint32_t, uint32_t, uint32_t, uint32_t))
            sim.boottab->svc)(id, p1, p2, p3);
}

static uint32_t svc32 (uint32_t id, uint32_t p1, uint32_t p2, uint32_t p3) {
    return ((uint32_t (*) (uint32_t, uint32_t, uint32_t, uint32_t))
            sim.boottab->svc)(id, p1, p2, p3);
}

static void svc (uint32_t id, uint32_t p1, uint32_t p2, uint32_t p3) {
    ((void (*) (uint32_t, uint32_t, uint32_t, uint32_t))
     sim.boottab->svc)(id, p1, p2, p3);
}

void hal_init (void* bootarg) {
    sim.boottab = bootarg;
    sim.xnow_cached = -1;
    sim.rand = svc32(SVC_RNG_SEED, 0, 0, 0);

    svc(SVC_VTOR, (uint32_t) irqvector, 0, 0);

    pd_init();

#if defined(SVC_eefs)
    eefs_init((void*) APPDATA_BASE, APPDATA_SZ);
#endif

#if defined(SVC_frag)
    {
        extern volatile boot_fwhdr fwhdr;
        frag_init((void*) (((uintptr_t) &fwhdr + fwhdr.size
                        + (FLASH_PAGE_SZ - 1)) & ~(FLASH_PAGE_SZ - 1)),
                (void*) FLASH_END);
    }
#endif

#if CFG_DEBUG != 0
    debug_str("\r\n============== DEBUG STARTED ==============\r\n");
#endif
}

void hal_watchcount (int cnt) {
}

#if 0
void hal_pin_rxtx (s1_t val) {
}

void hal_pin_rst (u1_t val) {
}

void hal_irqmask_set (int mask) {
}

void hal_spi_select (int on) {
}

u1_t hal_spi (u1_t outval) {
    return 0;
}
#endif

void hal_disableIRQs (void) {
    if( sim.irqlevel++ == 0 ) {
        asm volatile ("cpsid i" : : : "memory");
    }
}

void hal_enableIRQs (void) {
    ASSERT(sim.irqlevel);
    if( --sim.irqlevel == 0 ) {
        asm volatile ("cpsie i" : : : "memory");
        svc(SVC_IRQ, 0, 0, 0);
    }
}

u1_t hal_sleep (u1_t type, u4_t targettime) {
    sim.xnow_cached = -1;
    return svc32(SVC_SLEEP, type, targettime, 0);
}

u4_t hal_ticks (void) {
    return hal_xticks();
}

u8_t hal_xticks (void) {
    if( sim.xnow_cached < 0 ) {
        sim.xnow_cached = svc64(SVC_TICKS, 0, 0, 0);
    }
    return sim.xnow_cached;
}

s2_t hal_subticks (void) {
    return 0;
}

void hal_waitUntil (u4_t time) {
    while (1) {
        u4_t now = hal_ticks();
        if (((s4_t) (time - now)) <= 0) {
            return;
        }
        sim.xnow_cached = -1;
        svc(SVC_SLEEP, HAL_SLEEP_EXACT, time, 0);
    }
}

u1_t hal_getBattLevel (void) {
    return 0;
}

void hal_setBattLevel (u1_t level) {
}

__attribute__((noreturn, naked))
void hal_failed (void) {
    // get return address
    uint32_t addr;
    __asm__("mov %[addr], lr" : [addr]"=r" (addr) : : );
    // in thumb mode the linked address is the address of the calling instruction plus 4 bytes
    addr -= 4;

#ifdef CFG_backtrace
    // log address of assertion
    backtrace_addr(__LINE__, addr);
    // save trace to EEPROM
    backtrace_save();
#endif

    // call panic function
    sim.boottab->panic(0, addr);
    // not reached
}

void radio_init (void) {

}

typedef struct {
    int32_t xbeg;
    int32_t xend;
    uint32_t freq;
    uint32_t rps;
    int32_t pow; // txpow or rssi
    int32_t snr;
    uint32_t dlen;
    unsigned char data[256];
} sim_rxtx;

static ostime_t syms2ticks (rps_t rps, int n) {
    if( getSf(rps) == FSK ) {
	// rough estimate of FSK @ 50kBit/s
        int extra = 5+3+1+2;                     // preamble, syncword, len, crc
	double us = ((n+extra) * 8e6 / 50e3);    // bits * (us/sec) / Bit/s
	return (ostime_t)(us * OSTICKS_PER_SEC / 1e6);
    }
    double Rs = (double) ((1<<getBw(rps))*125000) / (1<<(getSf(rps)+(7-SF7)));
    double Ts = 1 / Rs;

    return (ostime_t) (n * Ts * OSTICKS_PER_SEC);
}

static void txdone (osjob_t* osjob) {
#if 0
    LOGIT(2, "txdone: pctx xbeg=%ldus/0x%X xend=%ldus/0x%X now=%ld/0x%X\n",
	  ticks2time(pctx->txframe.xbeg), pctx->txframe.xbeg,
	  ticks2time(pctx->txframe.xend), pctx->txframe.xend, enow, now);
    pctx->txfunc(pctx, pctx->txframe.frame, pctx->txframe.flen, pctx->txframe.freq, pctx->txframe.rps,
		 ticks2time(pctx->txframe.xbeg), ticks2time(pctx->txframe.xend));
#endif
    os_setTimedCallback(&LMIC.osjob, LMIC.txend + us2osticks(43), LMIC.osjob.func);
}

static void tx (void) {
    sim_rxtx tx;

    ostime_t now = hal_ticks();

    tx.xbeg = now;
    tx.xend = now + calcAirTime(LMIC.rps, LMIC.dataLen);
    tx.freq = LMIC.freq;
    tx.rps = LMIC.rps;
    tx.pow = LMIC.txpow + LMIC.txPowAdj;
    tx.dlen = LMIC.dataLen;
    memcpy(tx.data, LMIC.frame, LMIC.dataLen);

    svc(SVC_TX, (uint32_t) &tx, 0, 0);

    LMIC.txend = tx.xend;
    os_setTimedCallback(&sim.rjob, LMIC.txend, txdone);
}

static void rxdone (osjob_t* job) {
    sim_rxtx rx;
    if (svc32(SVC_RX_DONE, (uint32_t) &rx, 0, 0) == 0) {
        // nothing received
        LMIC.dataLen = 0;
    } else {
        LMIC.rxtime0 = rx.xbeg;
        LMIC.rxtime = rx.xend;
        LMIC.snr = rx.snr;
        LMIC.rssi = rx.pow;
        LMIC.dataLen = rx.dlen;
        memcpy(LMIC.frame, rx.data, LMIC.dataLen);
    }
    os_setCallback(&LMIC.osjob, LMIC.osjob.func);
}

static void rxdo (osjob_t* job) {
    if (svc32(SVC_RX_DO, 0, 0, 0) == 0) {
        // no preamble in sight
        rxdone(job);
    } else {
        // there's a chance we might get something
        os_setTimedCallback(&sim.rjob, LMIC.txend, rxdone);
    }
}

static void rx (void) {
    hal_waitUntil(LMIC.rxtime); // busy wait until exact rx time
#if 0
    debug_printf("RX[freq=%.3F,sf=%d,bw=%s]\r\n",
            LMIC.freq, 6, getSf(LMIC.rps) + 6, ("125\0" "250\0" "500\0" "rfu") + (4 * getBw(LMIC.rps)));
#endif
    ostime_t timeout = syms2ticks(LMIC.rps, LMIC.rxsyms);
    svc(SVC_RX_START, LMIC.freq, LMIC.rps, timeout);
    os_setTimedCallback(&sim.rjob, LMIC.rxtime + timeout, rxdo);
}

void os_radio (u1_t mode) {
    switch (mode) {
	case RADIO_RST:
            os_clearCallback(&sim.rjob);
            break;
	case RADIO_TX:
            tx();
            break;

	case RADIO_RX:
            rx();
            break;

        case RADIO_CCA:
            LMIC.rssi = -127;
            break;

	case RADIO_RXON:
            // not yet implemented

        default:
            hal_failed();
    }
}

u1_t radio_rand1 (void) {
    sim.rand = sim.rand * 214013 + 2531011;
    return sim.rand >> 16;
}


#ifdef CFG_powerstats

void hal_stats_get (hal_statistics* stats) {
}
void hal_stats_consume (hal_statistics* stats) {
}

#endif


#ifdef CFG_DEBUG

void hal_debug_str (const char* str) {
    svc(SVC_DEBUG_STR, (uint32_t) str, 0, 0);
}

void hal_debug_led (int val) {
}

#endif


void hal_fwinfo (hal_fwi* fwi) {
    fwi->blversion = sim.boottab->version;

    extern volatile boot_fwhdr fwhdr;
    fwi->version = 0; // XXX no longer in fwhdr
    fwi->crc = fwhdr.crc;
    fwi->flashsz = 128*1024;
}

u4_t hal_unique (void) {
    return 0xdeadbeef;
}


// ------------------------------------------------
// EEPROM

void eeprom_write (void* dest, unsigned int val) {
    ASSERT(((uintptr_t) dest & 3) == 0
            && (uintptr_t) dest >= EEPROM_BASE
            && (uintptr_t) dest < EEPROM_END);
    *((uint32_t*) dest) = val;
}

void eeprom_copy (void* dest, const void* src, int len) {
    ASSERT(((uintptr_t) src & 3) == 0 && (len & 3) == 0);
    uint32_t* p = dest;
    const uint32_t* s = src;
    len >>= 2;
    while( len-- > 0 ) {
        eeprom_write(p++, *s++);
    }
}


// ------------------------------------------------
// CRC engine (32bit aligned words only)

unsigned int crc32 (void* ptr, int nwords) {
    return sim.boottab->crc32(ptr, nwords);
}


// ------------------------------------------------
// SHA-256 engine

void sha256 (uint32_t* hash, const uint8_t* msg, uint32_t len) {
    sim.boottab->sha256(hash, msg, len);
}


// ------------------------------------------------
// UART

enum {
    UART_READREG = 0,
    UART_WRITE,
    UART_CONFIG,
    UART_ENABLE,

    UART_REG_SR = 0,
    UART_REG_RX,

    UART_EN_RX = 0,
    UART_EN_TX,

    UART_SR_RXE = (1 << 0),
    UART_SR_TXE = (1 << 1),
    UART_SR_RXR = (1 << 2),
    UART_SR_TXR = (1 << 3),
    UART_SR_OVR = (1 << 4),
};

static struct {
    usart_rx_func rx;
    usart_tx_func tx;
} uart;

void usart_init (void) {
}

void usart_cfg (unsigned int br) {
    svc(SVC_UART, UART_CONFIG, br, 0);
}

void usart_recv (usart_rx_func rx, void* arg) {
    uart.rx = rx;
    svc(SVC_UART, UART_ENABLE, UART_EN_RX, 1);
}

void usart_send (usart_tx_func tx, void* arg) {
    uart.tx = tx;
    svc(SVC_UART, UART_ENABLE, UART_EN_TX, 1);
}

void usart_abort_recv (void) {
    hal_disableIRQs();
    unsigned int sr = svc32(SVC_UART, UART_READREG, UART_REG_SR, 0);
    if( sr & UART_SR_RXE ) {
        svc(SVC_UART, UART_ENABLE, UART_EN_RX, 0);
        uart.rx(USART_ERROR, NULL);
    }
    hal_enableIRQs();
}

static void usart_handler (void) {
    unsigned int sr = svc32(SVC_UART, UART_READREG, UART_REG_SR, 0);
    if( (sr & UART_SR_RXR) ) {
        int ch = svc32(SVC_UART, UART_READREG, UART_REG_RX, 0);
        if( sr & UART_SR_OVR ) {
            ch = USART_ERROR;
        }
        if( sr & UART_SR_RXE ) {
            if( uart.rx(ch, NULL) != USART_CONTINUE ) {
                svc(SVC_UART, UART_ENABLE, UART_EN_RX, 0);
            }
        }
    }
    if( (sr & UART_SR_TXR) ) {
        int ch = uart.tx(USART_CONTINUE, NULL);
        if( ch < 0 ) {
            svc(SVC_UART, UART_ENABLE, UART_EN_TX, 0);
            uart.tx(USART_DONE, NULL);
        } else {
            svc(SVC_UART, UART_WRITE, ch, 0);
        }
    }
}

// ------------------------------------------------
// Programmable I/O

static struct _pio {   // high  low  inp  ana
    uint32_t cfg1;     //    1    1    0    0
    uint32_t cfg2;     //    1    0    1    0
    uint32_t pup;
    uint32_t pdn;
    uint32_t irq_rising;
    uint32_t irq_falling;
    uint32_t irq_enable;
} pio;

static void pio_handler (void) {
#ifdef CFG_EXTI_IRQ_HANDLER
    // invoke user-defined interrupt handler
    extern void CFG_EXTI_IRQ_HANDLER (void);
    CFG_EXTI_IRQ_HANDLER();
#endif
}

enum {
    PIO_CONFIG,
    PIO_GET,
    PIO_IRQ_GET,
    PIO_IRQ_CLEAR,
};

static void pio_config (void) {
    struct _pio npio;
    npio = pio;
    npio.irq_rising &= npio.irq_enable;
    npio.irq_falling &= npio.irq_enable;
    svc(SVC_PIO, PIO_CONFIG, (uint32_t) &npio, 0);
}

void pio_set (unsigned int pin, int value) {
    ASSERT(pin < 32);
    uint32_t mask = (1 << pin);

    // unconditionally clear all bits
    pio.cfg1 &= ~mask;
    pio.cfg2 &= ~mask;
    pio.pup  &= ~mask;
    pio.pdn  &= ~mask;

    // now set the necessary bits
    if( value >= 0 ) {
        pio.cfg1 |= mask;
        if( value ) {
            pio.cfg2 |= mask;
        }
    } else {
        if( (value & 0x4) ) {
            pio.cfg2 |= mask;
            if( (value & 0x1) == 0 ) {
                pio.pup |= mask;
            }
            if( (value & 0x2) == 0 ) {
                pio.pdn |= mask;
            }
        }
    }
    pio_config();
}

int pio_get (unsigned int pin) {
    ASSERT(pin < 32);
    uint32_t v = svc32(SVC_PIO, PIO_GET, 0, 0);
    return v & (1 << pin);
}

static void bit_set_clear (uint32_t* dst, uint32_t mask, bool set) {
    if (set) {
        *dst |= mask;
    } else {
        *dst &= ~mask;
    }
}

void pio_irq_config (unsigned int pin, bool rising, bool falling) {
    uint32_t mask = (1 << pin);
    bit_set_clear(&pio.irq_rising, mask, rising);
    bit_set_clear(&pio.irq_falling, mask, falling);
    pio_config();
}

void pio_irq_enable (unsigned int gpio, bool enable) {
    bit_set_clear(&pio.irq_enable, (1 << gpio), enable);
    pio_config();
}

uint32_t pio_irq_get (void) {
    return svc32(SVC_PIO, PIO_IRQ_GET, 0, 0);
}

void pio_irq_clear (uint32_t mask) {
    svc(SVC_PIO, PIO_IRQ_CLEAR, mask, 0);
}

static uint32_t irqvector[16] = {
    (uint32_t) pio_handler,
    (uint32_t) usart_handler,
};

void hal_reboot (void) {
    // TODO: implement
    hal_failed();
}

typedef struct {
    uint32_t    dnonce;      // dev nonce
} pdata;

u4_t hal_dnonce_next (void) {
    pdata* p = (pdata*) STACKDATA_BASE;
    return p->dnonce++;
}

void hal_dnonce_clear (void) {
    pdata* p = (pdata*) STACKDATA_BASE;
    p->dnonce = 0;
}
