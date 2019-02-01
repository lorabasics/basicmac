// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#include "lmic.h"
#include "hw.h"
#include "peripherals.h"
#include "backtrace.h"
#include "bootloader.h"
#include "boottab.h"

#if defined(CFG_perso)
#include "perso.h"
#endif

#if defined(SVC_eefs)
#include "eefs/eefs.h"
#endif

// HAL state
static struct {
    s4_t irqlevel;
    u4_t ticks;
    s2_t veto_deep;
    s2_t veto_medium;
    int watchcount;
#ifdef CFG_powerstats
    struct {
	uint64_t run;		// running
	uint64_t sleep[3];	// normal, medium, sleep
	uint64_t rx, tx;	// radio RX/TX
    } stats;
#endif
    u1_t battlevel;
    boot_boottab* boottab;
} HAL;


// -----------------------------------------------------------------------------
// Panic

// don't change these values, so we know what they are in the field...
enum {
    PANIC_HAL_FAILED	= 0,
    PANIC_LSE_NOSTART	= 1,
    PANIC_CAL_FAILED	= 2,
    PANIC_STS_FAILED	= 3,
};

#ifdef CFG_panic911
__attribute__((noinline))	// ensure function has a frame, since called from naked
static void call911 (uint32_t reason, uint32_t addr) {
    struct {
	uint32_t magic;
	unsigned char deveui[8];
	uint32_t reason;
	uint32_t addr;
    } info;
    info.magic = 0x504c4548; // HELP
    memcpy(info.deveui, BOOT_DEVINFO->deveui, 8);
    info.reason = reason;
    info.addr = addr;
    memcpy(LMIC.frame, &info, LMIC.dataLen = sizeof(info));

#if defined(CFG_eu868)
    LMIC.freq = 868300000;
    LMIC.rps = updr2rps(DR_SF12);
#elif defined(CFG_us915)
    LMIC.freq = 903700000;
    LMIC.rps = updr2rps(DR_SF10);
#else
#error "Unsupported region"
#endif
    LMIC.txpow = 14;
    os_radio(RADIO_TX);
}
#endif

__attribute__((noreturn))
static void panic (uint32_t reason, uint32_t addr) {
    // disable interrupts
    __disable_irq();

    // set watchdog on older bootloaders to ensure device restarts
    if (HAL.boottab->version < 6) {
	IWDG->KR = 0xcccc;	// start
	IWDG->KR = 0x5555;	// enable write
	IWDG->PR = 7;		// prescaler: 256
	IWDG->RLR = (1 << 12) - 1;
	while (IWDG->SR);	// wait until IWDG is ready
	IWDG->KR = 0xaaaa;	// refresh
    }

#ifdef CFG_panic911
    // yelp for help
    call911(reason, addr);
#endif

    // call bootloader's panic function
    HAL.boottab->panic(reason, addr);
    // not reached
}

__attribute__((noreturn, naked))
void hal_failed () {
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
    panic(PANIC_HAL_FAILED, addr);
    // not reached
}


// -----------------------------------------------------------------------------
// timer rollover based watchdog

void hal_watchcount (int cnt) {
    HAL.watchcount = cnt;
}

// -----------------------------------------------------------------------------
// I/O

static void hal_io_init () {
#ifdef CFG_rxtx_always_on
#if defined(GPIO_RX) && ((GPIO_RX & BRD_GPIO_EXT_PULLUP) == 0)
    SET_PIN(GPIO_RX, 1);
    CFG_PIN(GPIO_RX, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
#endif
#if defined(GPIO_TX) && ((GPIO_TX & BRD_GPIO_EXT_PULLDN) == 0)
    SET_PIN(GPIO_TX, 0);
    CFG_PIN(GPIO_TX, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
#endif
#endif

#ifndef CFG_noradio
#ifdef GPIO_DIO0
    IRQ_PIN(GPIO_DIO0, GPIO_IRQ_RISING);
#endif
#ifdef GPIO_DIO1
    IRQ_PIN(GPIO_DIO1, GPIO_IRQ_RISING);
#endif
#ifdef GPIO_DIO2
    IRQ_PIN(GPIO_DIO2, GPIO_IRQ_RISING);
#endif
#endif
}

// val == 1  => tx 1, rx 0 ; val == 0 => tx 0, rx 1 ;  val == -1 => off
void hal_pin_rxtx (s1_t val) {
#ifdef CFG_powerstats
    static ostime_t t1;
    static uint64_t* pt;
    ostime_t now = hal_ticks();
    if (pt) {
	*pt += (now - t1);
    }
#endif
    ASSERT(val == 1 || val == 0 || val == -1);
    if (val < 0) {
#ifdef CFG_powerstats
	pt = NULL;
#endif
#ifdef GPIO_TXRX_EN
	SET_PIN(GPIO_TXRX_EN, 0);
	CFG_PIN_DEFAULT(GPIO_TXRX_EN);
#endif
#ifndef CFG_rxtx_always_on
#ifdef GPIO_RX
	CFG_PIN_DEFAULT(GPIO_RX);
#endif
#ifdef GPIO_TX
	CFG_PIN_DEFAULT(GPIO_TX);
#endif
#endif
    } else {
#ifdef CFG_powerstats
	t1 = now;
	pt = (val == 0) ? &HAL.stats.rx : &HAL.stats.tx;
#endif
#ifdef GPIO_TXRX_EN
	SET_PIN(GPIO_TXRX_EN, 1);
	CFG_PIN(GPIO_TXRX_EN, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
#endif
#ifdef GPIO_RX
	SET_PIN(GPIO_RX, val ^ 1);
#endif
#ifdef GPIO_TX
	SET_PIN(GPIO_TX, val);
#endif
#ifndef CFG_rxtx_always_on
#ifdef GPIO_RX
	CFG_PIN(GPIO_RX, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
#endif
#ifdef GPIO_TX
	CFG_PIN(GPIO_TX, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
#endif
#endif
    }
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if(val == 0 || val == 1) { // drive pin
        SET_PIN(GPIO_RST, val);
	CFG_PIN(GPIO_RST, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
    } else { // keep pin floating
	CFG_PIN(GPIO_RST, GPIOCFG_MODE_ANA | GPIOCFG_OSPEED_400kHz | GPIOCFG_OTYPE_OPEN | GPIOCFG_PUPD_NONE);
    }
}

void hal_pin_busy_wait (void) {
#ifdef GPIO_BUSY
    CFG_PIN(GPIO_BUSY, GPIOCFG_MODE_INP | GPIOCFG_OSPEED_40MHz);

    while (GET_PIN(GPIO_BUSY) != 0);

    CFG_PIN_DEFAULT(GPIO_BUSY);
#endif
}

extern void radio_irq_handler(u1_t diomask);

// generic EXTI IRQ handler for all channels
static void EXTI_IRQHandler () {
    u1_t diomask = 0;
#ifdef GPIO_DIO0
    // DIO 0
    if ((EXTI->PR & (1 << BRD_PIN(GPIO_DIO0))) != 0) { // pending
        EXTI->PR = (1 << BRD_PIN(GPIO_DIO0)); // clear irq
	diomask |= (1 << 0);
    }
#endif
#ifdef GPIO_DIO1
    // DIO 1
    if ((EXTI->PR & (1 << BRD_PIN(GPIO_DIO1))) != 0) { // pending
        EXTI->PR = (1 << BRD_PIN(GPIO_DIO1)); // clear irq
	diomask |= (1 << 1);
    }
#endif
#ifdef GPIO_DIO2
    // DIO 2
    if ((EXTI->PR & (1 << BRD_PIN(GPIO_DIO2))) != 0) { // pending
        EXTI->PR = (1 << BRD_PIN(GPIO_DIO2)); // clear irq
	diomask |= (1 << 2);
    }
#endif

    if(diomask) {
        // invoke radio handler (on IRQ)
        radio_irq_handler(diomask);
    }

#ifdef CFG_EXTI_IRQ_HANDLER
    // invoke user-defined interrupt handler
    {
        extern void CFG_EXTI_IRQ_HANDLER(void);
        CFG_EXTI_IRQ_HANDLER();
    }
#endif // CFG_EXTI_IRQ_HANDLER
}

static void dio_config (int mask, int pin, int gpio) {
    if (mask & pin) {
	CFG_PIN(gpio, GPIOCFG_MODE_INP | GPIOCFG_OSPEED_40MHz);
	IRQ_PIN_SET(gpio, 1);
    } else {
	IRQ_PIN_SET(gpio, 0);
	CFG_PIN_DEFAULT(gpio);
        EXTI->PR = (1<<BRD_PIN(gpio)); // clear irq
    }
}

void hal_irqmask_set (int mask) {
    static int prevmask = 0;

#ifdef GPIO_DIO0
    dio_config(mask, HAL_IRQMASK_DIO0, GPIO_DIO0);
#endif
#ifdef GPIO_DIO1
    dio_config(mask, HAL_IRQMASK_DIO1, GPIO_DIO1);
#endif
#ifdef GPIO_DIO2
    dio_config(mask, HAL_IRQMASK_DIO2, GPIO_DIO2);
#endif

    mask = (mask != 0);
    if (prevmask != mask) {
	if (mask) {
	    hal_disableSleep(HAL_SLEEP_MEDIUM);
	} else {
	    hal_enableSleep(HAL_SLEEP_MEDIUM);
	}
	prevmask = mask;
    }
}


// -----------------------------------------------------------------------------
// SPI

#if BRD_RADIO_SPI == 1
#define SPIx			SPI1
#define SPIx_enable()		do { RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; } while (0)
#define SPIx_disable()		do { RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN; } while (0)
#define RCC_APB2ENR_SPIxEN	RCC_APB2ENR_SPI1EN
#elif BRD_RADIO_SPI == 2
#define SPIx			SPI2
#define SPIx_enable()		do { RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; } while (0)
#define SPIx_disable()		do { RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN; } while (0)
#else
#error "Unsupported value for BRD_RADIO_SPI"
#endif


static void hal_spi_init () {
    // enable clock for SPI interface 1
    SPIx_enable();

    // configure and activate the SPI (master, internal slave select, software slave mgmt)
    // (use default mode: 8-bit, 2-wire, no crc, MSBF, CPOL0 [clock is low when idle], CPHA0, PCLK32/4=8MHz)
    SPIx->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_SPE | SPI_CR1_BR_0;

    // configure I/O lines and disable clock
    hal_spi_select(0);
}

void hal_spi_select (int on) {
    if (on) {
	// enable clock for SPI interface 1
	SPIx_enable();
	// configure pins for alternate function SPIx (SCK, MISO, MOSI)
	CFG_PIN_AF(GPIO_SCK, GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);
	CFG_PIN_AF(GPIO_MISO, GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);
	CFG_PIN_AF(GPIO_MOSI, GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);
	// drive chip select low
        SET_PIN(GPIO_NSS, 0);
	CFG_PIN(GPIO_NSS, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
    } else {
	// stop driving chip select, activate pull-up
	CFG_PIN(GPIO_NSS, GPIOCFG_MODE_INP | GPIOCFG_PUPD_PUP);
	// put SCK, MISO, MOSI back to analog input (HiZ) mode
#if defined(BRD_sck_mosi_pulldown)
	CFG_PIN(GPIO_SCK, GPIOCFG_MODE_INP | GPIOCFG_PUPD_PDN);
	CFG_PIN(GPIO_MOSI, GPIOCFG_MODE_INP | GPIOCFG_PUPD_PDN);
#elif defined(BRD_sck_mosi_drivelow)
        SET_PIN(GPIO_SCK, 0);
	CFG_PIN(GPIO_SCK, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
        SET_PIN(GPIO_MOSI, 0);
	CFG_PIN(GPIO_MOSI, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
#else
	CFG_PIN_DEFAULT(GPIO_SCK);
	CFG_PIN_DEFAULT(GPIO_MOSI);
#endif
	CFG_PIN_DEFAULT(GPIO_MISO);
	// disable clock for SPI interface
	SPIx_disable();
    }
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    SPIx->DR = out;
    while( (SPIx->SR & SPI_SR_RXNE ) == 0);
    return SPIx->DR; // in
}


// -----------------------------------------------------------------------------
// TIME

#if defined(STM32L0)
#define TIMx			TIM22
#define RCC_APB2ENR_TIMxEN	RCC_APB2ENR_TIM22EN
#define RCC_APB2RSTR_TIMxRST	RCC_APB2RSTR_TIM22RST
#define TIMx_IRQn		TIM22_IRQn
#elif defined(STM32L1)
#define TIMx			TIM9
#define RCC_APB2ENR_TIMxEN	RCC_APB2ENR_TIM9EN
#define RCC_APB2LPENR_TIMxLPEN	RCC_APB2LPENR_TIM9LPEN
#define RCC_APB2RSTR_TIMxRST	RCC_APB2RSTR_TIM9RST
#define TIMx_IRQn		TIM9_IRQn
#endif

// Busy wait on condition with timeout (about 10s)
#define SAFE_while(code, expr) do { \
    volatile uint32_t __timeout = (1 << 25); \
    while (expr) { \
	if (__timeout-- == 0) { \
	    uint32_t __addr; \
	    __asm__("mov %[__addr], pc" : [__addr]"=r" (__addr) : : ); \
	    panic(code, __addr); \
	} \
    } \
} while (0)

#if defined(STM32L0)
static uint32_t cal_measure (void) {
    // clear all flags
    TIM21->SR = 0;
    // enable channel
    TIM21->CCER |= TIM_CCER_CC1E;
    uint32_t accu = 0;
    uint16_t t0 = 0;
    for (int i = 0; i < 8; i++) {
	// wait for capture
	SAFE_while(PANIC_CAL_FAILED, (TIM21->SR & TIM_SR_CC1IF) == 0);
	// read stamp
	uint32_t t1 = TIM21->CCR1;
	// accumulate
	if (i != 0) {
	    accu += (int16_t) (t1 - t0);
	}
	// save last stamp
	t0 = t1;
    }
    // disable channel
    TIM21->CCER &= ~TIM_CCER_CC1E;
    // return accumulator
    return accu;
}

static uint32_t cal_lsi (void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;	// enable peripheral clock
    TIM21->PSC = 0;				// 32 MHz
    TIM21->CCMR1 = (3 << 2) | 1;		// ch1: psc=8, input capture IC1->TI1

    // measure LSE
    TIM21->OR = (4 << 2);			// TI1 -> LSE
    TIM21->CR1 = TIM_CR1_CEN;			// enable counter
    uint32_t lse = cal_measure();
    TIM21->CR1 = 0;				// disable counter

    // measure LSI
    TIM21->OR = (5 << 2);			// TI1 -> LSI
    TIM21->CR1 = TIM_CR1_CEN;			// enable counter
    uint32_t lsi = cal_measure();
    TIM21->CR1 = 0;				// disable counter

    RCC->APB2ENR &= ~RCC_APB2ENR_TIM21EN;	// disable peripheral clock

    return (lse << 15) / lsi;			// f(lsi) = 32.768kHz * lse / lsi
}

#ifdef CFG_subticks
static void subticks_start (void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;	// enable peripheral clock
    TIM21->PSC = 0;				// 32 MHz
    TIM21->CCMR1 = 1;				// input capture IC1->TI1
    TIM21->SMCR = (4 << 4) | 4;			// reset on TI1 edge
    TIM21->OR = (4 << 2);			// TI1 -> LSE
    TIM21->CR1 = TIM_CR1_CEN;			// enable counter
}

static void subticks_stop (void) {
    TIM21->CR1 = 0;				// disable counter
    RCC->APB2ENR &= ~RCC_APB2ENR_TIM21EN;	// disable peripheral clock
}

s2_t hal_subticks (void) {
    uint32_t cnt = TIM21->CNT;
    uint32_t max = TIM21->CCR1;
    if (max < 878 || max > 1075) { // 32MHz/32.768kHz ±10%
	return -1;
    }
    return (cnt * 1024) / max;
}
#endif
#endif

static void hal_time_init () {
#ifndef CFG_clock_HSE
    PWR->CR |= PWR_CR_DBP; // disable write protect

    // Reset RTC
    RCC->CSR |= RCC_CSR_RTCRST;
    RCC->CSR &= ~RCC_CSR_RTCRST;

#ifdef BRD_clock_lsedrv
    RCC->CSR |= BRD_clock_lsedrv;
#endif
    RCC->CSR |= RCC_CSR_LSEON; // switch on low-speed oscillator @32.768kHz
    SAFE_while(PANIC_LSE_NOSTART, (RCC->CSR & RCC_CSR_LSERDY) == 0); // wait for it...
#endif

    RCC->APB2ENR   |= RCC_APB2ENR_TIMxEN;     // enable clock to TIMx peripheral
#if defined(STM32L1)
    RCC->APB2LPENR |= RCC_APB2LPENR_TIMxLPEN; // enable clock to TIMx peripheral also in low power mode
#endif
    RCC->APB2RSTR  |= RCC_APB2RSTR_TIMxRST;   // reset TIMx interface
    RCC->APB2RSTR  &= ~RCC_APB2RSTR_TIMxRST;  // reset TIMx interface

#if CFG_clock_HSE
    TIMx->PSC  = (640 - 1); // HSE_CLOCK_HWTIMER_PSC-1);  XXX: define HSE_CLOCK_HWTIMER_PSC somewhere
#else
    TIMx->SMCR = TIM_SMCR_ECE; // external clock enable (source clock mode 2) with no prescaler and no filter
#if defined(STM32L0)
    TIMx->OR |= 3; // XXX - ETR remap
#endif
#endif

    NVIC_EnableIRQ(TIMx_IRQn);

    // enable update (overflow) interrupt
    TIMx->DIER |= TIM_DIER_UIE;

    // Enable timer counting
    TIMx->CR1 = TIM_CR1_CEN;
#ifdef CFG_subticks
    subticks_start();
#endif

    // Prepare RTC
    // configure EXTI line 20 (RTC wakeup)
    EXTI->RTSR |= (1 << 20); // rising edge
    EXTI->PR = (1 << 20);
    EXTI->IMR |= (1 << 20);
    // configure NVIC
    NVIC_EnableIRQ(RTC_IRQn);
}

#if 0
static inline u4_t digit (u4_t* val, u4_t base) {
    u4_t d = 0;
    while (*val >= base) {
	d += 1;
	*val -= base;
    }
    return d;
}

static u4_t sec2bcd (u4_t sec) {
    u4_t v;
    v  = (digit(&sec, 10*60*60) << 20);	// 10 hours
    v |= (digit(&sec,    60*60) << 16);	// hours
    v |= (digit(&sec,    10*60) << 12);	// 10 mins
    v |= (digit(&sec,       60) <<  8);	// mins
    v |= (digit(&sec,       10) <<  4);	// 10 secs
    v |= sec;				// secs
    return v;
}
#endif

static u4_t bcd2sec (u4_t bcd) {
    u4_t v;
    v  =        ((((bcd >>  4) & 0x7) * 10) + (bcd       & 0xf));
    v +=   60 * ((((bcd >> 12) & 0x7) * 10) + (bcd >>  8 & 0xf));
    v += 3600 * ((((bcd >> 20) & 0x3) * 10) + (bcd >> 16 & 0xf));
    return v;
}

// TODO - some of the settings could be done in init(), restore write protection?
// switch to RTC, set wakeup (in seconds)
static void hal_time_rtc_enter (int wakeup) {
#ifdef CFG_subticks
    // disable subtick counting
    subticks_stop();
#endif
    // disable RTC domain write protection
    PWR->CR |= PWR_CR_DBP;
    // enable LSI
    RCC->CSR |= RCC_CSR_LSION;
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0);
    // get LSI freq (measured using LSE as reference)
    uint32_t f_lsi = cal_lsi();
    ASSERT(f_lsi >= 26000 && f_lsi <= 56000);
    // set LSI as RTC clock
    RCC->CSR |= RCC_CSR_RTCSEL_1; // XXX - assumes that RTCSEL bits were 0
    // enable RTC
    RCC->CSR |= RCC_CSR_RTCEN;
    // disable RTC registers write protection
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    // switch to initialization mode
    RTC->ISR = RTC_ISR_INIT;
    while ((RTC->ISR & RTC_ISR_INITF) == 0);
    // set prescaler for LSI
    RTC->PRER = (127 << 16) | (((f_lsi + 64) >> 7) - 1);	// (1+127) * (1+(round(f_lsi/128)-1))
    // set time to 0
    RTC->TR = 0;
    // switch out of init mode
    RTC->ISR &= ~RTC_ISR_INIT;
    // wait until RTC is running
    while (RTC->ISR & RTC_ISR_INITF);
    // disable timer counting
    TIMx->CR1 &= ~TIM_CR1_CEN;
    // handle pending overflow
    if (TIMx->SR & TIM_SR_UIF) {
	HAL.ticks++;
    }
    // clear timer interrupt flags
    TIMx->SR = 0;
    // save current ticks
    u8_t now = hal_xticks();
    // disable LSE
    RCC->CSR &= ~RCC_CSR_LSEON;
    // backup current time to backup register 0+1
    RTC->BKP0R = now;
    RTC->BKP1R = now >> 32;
    // disable wake-up
    RTC->CR &= ~RTC_CR_WUTE;
    // wait for wake-up timer to become writable
    while ((RTC->ISR & RTC_ISR_WUTWF) == 0);
    // set wake-up timer
    RTC->WUTR = wakeup - 1;
    // set ck_spre (1Hz) as wake-up clock -- rest of bits: reset value
    RTC->CR = 4;
    // enable wake-up
    RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;
}

// switch back from RTC to TIMx-based time keeping
static void hal_time_rtc_exit (void) {
    // disable RTC domain write protection
    PWR->CR |= PWR_CR_DBP;
    // enable LSE
    RCC->CSR |= RCC_CSR_LSEON;
    SAFE_while(PANIC_LSE_NOSTART, (RCC->CSR & RCC_CSR_LSERDY) == 0);
    // get ticks at enter time from backup register 0+1
    u8_t now = (((u8_t)RTC->BKP1R) << 32) | RTC->BKP0R;
    // synchronize and read calendar
    RTC->ISR &= ~RTC_ISR_RSF;
    while ((RTC->ISR & RTC_ISR_RSF) == 0);
    uint32_t tr = RTC->TR;
    uint32_t ssr = RTC->SSR;
    (void) RTC->DR; // read DR to unlock shadow registers
    // add elapsed seconds
    now += (OSTICKS_PER_SEC * bcd2sec(tr));
    // add elapsed sub-second
    uint32_t pds = RTC->PRER & RTC_PRER_PREDIV_S;
    now += ((OSTICKS_PER_SEC * (pds - ssr)) / (pds + 1));
    // restore ticks, counter, compare register
    HAL.ticks = now >> 16;
    TIMx->CNT = now;
    TIMx->CCR2 = 0xffff;
    // enable counting
    TIMx->CR1 |= TIM_CR1_CEN;
#ifdef CFG_subticks
    subticks_start();
#endif
    // disable RTC, LSI
    RCC->CSR &= ~(RCC_CSR_RTCEN | RCC_CSR_LSION);
}

// Only call if interrupts are disabled!!
#define hal_ticks_unsafe() ((u4_t) hal_xticks_unsafe())
static u8_t hal_xticks_unsafe (void) {
    u8_t xt = HAL.ticks;
    u2_t cnt = TIMx->CNT;
    if( (TIMx->SR & TIM_SR_UIF) ) {
        // Overflow before we read CNT?
        // Include overflow in evaluation but
        // leave update of state to ISR once interrupts enabled again
        cnt = TIMx->CNT;
        xt++;
    }
    return (xt<<16)|cnt;
}

u8_t hal_xticks () {
    hal_disableIRQs();
    u8_t xt = hal_xticks_unsafe();
    hal_enableIRQs();
    return xt;
}

u4_t hal_ticks () {
    return hal_xticks();
}

// return delta ticks from now to specified ticktime (0 for past)
static u4_t deltaticks (u4_t time) {
    s4_t d = time - hal_ticks();
    if (d <= 0) {
	return 0;    // in the past
    } else {
	return d;
    }
}

void hal_waitUntil (u4_t time) {
    while( deltaticks(time) != 0 ); // busy wait until timestamp is reached
}

__attribute__((noinline,used))	// ensure function has a frame, since called from naked
static void _TIMx_IRQHandler (uint32_t addr) {
    if (TIMx->SR & TIM_SR_UIF) { // overflow
	TIMx->SR = ~TIM_SR_UIF; // clear IRQ
	HAL.ticks++;
	if(HAL.watchcount) {
	    if(--HAL.watchcount == 0) {
		// log return address
		TRACE_ADDR(addr);
		// start over...
		ASSERT(0);
	    }
	}
    }
    if (TIMx->SR & TIM_SR_CC2IF) { // expired
	TIMx->SR = ~TIM_SR_CC2IF; // clear IRQ
        // do nothing, only wake up cpu
    }
}

__attribute__ ((naked))
void TIMx_IRQHandler () {
    // get return address (avoid stack push prolog of C)
    __asm__("    movs r0, #4\n"
	    "    mov  r1, lr\n"
	    "    tst  r0, r1\n"
	    "    bne  1f\n"
	    "    mrs  r0, msp\n"
	    "    b    2f\n"
	    "1:  mrs  r0, psp\n"
	    "2:  ldr  r0, [r0, #0x18]\n"
	    "    b    _TIMx_IRQHandler");
}

void RTC_IRQHandler () {
    // clear pending bit
    EXTI->PR = (1 << 20);
    // do nothing, only wake up cpu
}

// -----------------------------------------------------------------------------
// IRQ

void hal_disableIRQs () {
    __disable_irq();
    HAL.irqlevel++;
}

void hal_enableIRQs () {
    if(--HAL.irqlevel == 0) {
        __enable_irq();
    }
}

// -----------------------------------------------------------------------------

static void clock_init() {
    // System is clocked by MSI @2.1MHz at this point,
    // We want to go to PLL(HSI16) @32MHz

    // 1a. HSI: Enable
    RCC->CR |= RCC_CR_HSION;
    // 1b. HSI: Wait for it
    while ((RCC->CR & RCC_CR_HSIRDY) == 0);

#if defined(STM32L1)
    // 2a. Flash: Enable 64bit access
    FLASH->ACR |= FLASH_ACR_ACC64;
#endif
    // 2b. Flash: Enable prefetch buffer
    FLASH->ACR |= FLASH_ACR_PRFTEN;
    // 2c. Flash: Use 1 Wait state
    FLASH->ACR |= FLASH_ACR_LATENCY;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) == 0);

    // 3a. Power: Enable clock
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    // 3b. Power: Select Vrange 1 (min. 1.71V!)
    PWR->CR = PWR_CR_VOS_0;
    // 3c. Power: Wait for regulator
    while ((PWR->CSR & PWR_CSR_VOSF) != 0);

    // 4a. PLL: Set source (HSI16), multiplier (4), divider (2)
    RCC->CFGR |= (RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL4 | RCC_CFGR_PLLDIV2);
    // 4b. PLL: Enable
    RCC->CR |= RCC_CR_PLLON;
    // 4c. PLL: Wait for it
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);

    // 5a. System clock: Set source (PLL)
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    // 5b. System clock: Wait for it
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // 6a. Configure MSI to use 130kHz (for sleep) --> must be at least 2xfLSE (32.768kHz)
    RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_MSIRANGE) | RCC_ICSCR_MSIRANGE_1;
    // 6b. Turn off MSI
    RCC->CR &= ~RCC_CR_MSION;
}

enum {
    SLEEP_NONE = -1,
    SLEEP_NORMAL = 0,
    SLEEP_MEDIUM,
    SLEEP_DEEP,
};

static void clock_sleep (int kind) {
    // prepare for sleep mode

    if (kind > SLEEP_NORMAL) {
	// startup MSI @130kHz
	RCC->CR |= RCC_CR_MSION;
	while ((RCC->CR & RCC_CR_MSIRDY) == 0);
	// switch clock source to MSI
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_MSI;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI);
	// disable PLL and HSI
	RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSION);
	// Power: Select Vrange 2, enable low-power sleep
	PWR->CR = PWR_CR_VOS_1 | PWR_CR_LPSDSR;
	while ((PWR->CSR & PWR_CSR_VOSF) != 0);
    }

    if (kind == SLEEP_DEEP) {
	// set sleep mode to deep
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	// enable ultra-low power sleep (no Vref, thus no BOR, PVD, temp sensor)
	PWR->CR |= PWR_CR_ULP;
    } else {
	// set sleep mode to regular
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	// disable ultra-low power sleep
	PWR->CR &= ~PWR_CR_ULP;
    }

    // disable power manager peripheral
    RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;
}

static void clock_run (int kind) {
    // enable power manager peripheral
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    if (kind > SLEEP_NORMAL) {
	// Switch to fast clock for run
	// select Vrange 1
	PWR->CR = PWR_CR_VOS_0;
	while ((PWR->CSR & PWR_CSR_VOSF) != 0);
	// startup HSI
	RCC->CR |= RCC_CR_HSION;
	while ((RCC->CR & RCC_CR_HSIRDY) == 0);
	// enable PLL
	RCC->CR |= RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);
	// switch clock source to PLL
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	// disable MSI
	RCC->CR &= ~RCC_CR_MSION;
	if (kind == SLEEP_DEEP) {
	    hal_time_rtc_exit();
	}
    }
}

#define SLEEP_THRESHOLD		5
#define MEDIUM_SLEEP_THRESHOLD	us2osticks(3000)
#define MEDIUM_SLEEP_ADJUST	us2osticks(2500)
#define DEEP_SLEEP_THRESHOLD	sec2osticks(3)

// decide sleep strategy and setup timer
static int which_sleep (u1_t type, u4_t targettime) {
    // clean up timer interrupts
    TIMx->DIER &= ~TIM_DIER_CC2IE;  // disable IE
    TIMx->CCER &= ~TIM_CCER_CC2E;   // disable capture/compare unit 2
    TIMx->SR = ~TIM_SR_CC2IF;      // clear pending capture/compare interrupt
    // calculate sleep time
    u4_t dt;
    if (type == HAL_SLEEP_FOREVER) {
	// sleep for approximately 12 hours
	type = HAL_SLEEP_APPROX;
	targettime = hal_ticks_unsafe() + sec2osticks(12*60*60);
	dt = ~0;
    } else {
	dt = deltaticks(targettime);
	if (dt < SLEEP_THRESHOLD) {
	    return SLEEP_NONE;
	}
    }
    // consider deep sleep
    if (type == HAL_SLEEP_APPROX && HAL.veto_deep == 0 && dt > DEEP_SLEEP_THRESHOLD) {
	hal_time_rtc_enter(dt / OSTICKS_PER_SEC);
	return SLEEP_DEEP;
    }
    int kind;
    if (HAL.veto_medium == 0 && dt > MEDIUM_SLEEP_THRESHOLD) {
	// medium sleep
	dt -= MEDIUM_SLEEP_ADJUST;
	kind = SLEEP_MEDIUM;
    } else {
	// regular sleep
	kind = SLEEP_NORMAL;
    }
    dt += TIMx->CNT;
    if (dt < 0xffff) {
	TIMx->CCR2 = dt;   // set comparator
	TIMx->DIER |= TIM_DIER_CC2IE;  // enable IE
	TIMx->CCER |= TIM_CCER_CC2E;   // enable capture/compare unit 2
    }
    return kind;
}

u1_t hal_sleep (u1_t type, u4_t targettime) {
    int kind = which_sleep(type, targettime);

    if (kind < SLEEP_NORMAL) {
	return 0;
    }

#ifdef CFG_powerstats
    static ostime_t wakeup;
    ostime_t t1 = hal_ticks_unsafe();
    ASSERT((t1 - wakeup) >= 0);
    HAL.stats.run += (t1 - wakeup);
#endif

    clock_sleep(kind);

    // suspend execution until IRQ, regardless of the CPSR I-bit
    // Note: interrupts are disabled when hal_sleep() is called
    __WFI();

    clock_run(kind);

#ifdef CFG_powerstats
    ostime_t t2 = hal_ticks_unsafe();
    ASSERT((t2 - t1) >= 0);
    HAL.stats.sleep[kind] += (t2 - t1);
    wakeup = t2;
#endif

    return 1;
}

#ifdef CFG_powerstats
void hal_stats_get (hal_statistics* stats) {
    stats->run_ms	= osticks2ms(HAL.stats.run);
    stats->sleep1_ms	= osticks2ms(HAL.stats.sleep[SLEEP_NORMAL]);
    stats->sleep2_ms	= osticks2ms(HAL.stats.sleep[SLEEP_MEDIUM]);
    stats->sleep3_s	= osticks2sec(HAL.stats.sleep[SLEEP_DEEP]);
    stats->rx_ms	= osticks2ms(HAL.stats.rx);
    stats->tx_ms	= osticks2ms(HAL.stats.tx);
}

void hal_stats_consume (hal_statistics* stats) {
    HAL.stats.run			-= ms2osxticks(stats->run_ms);
    HAL.stats.sleep[SLEEP_NORMAL]	-= ms2osxticks(stats->sleep1_ms);
    HAL.stats.sleep[SLEEP_MEDIUM]	-= ms2osxticks(stats->sleep2_ms);
    HAL.stats.sleep[SLEEP_DEEP]		-= sec2osxticks(stats->sleep3_s);
    HAL.stats.rx			-= ms2osxticks(stats->rx_ms);
    HAL.stats.tx			-= ms2osxticks(stats->tx_ms);
}
#endif

void hal_disableSleep (u1_t kind) {
    hal_disableIRQs();
    HAL.veto_deep += 1;
    if (kind != HAL_SLEEP_DEEP) {
	HAL.veto_medium += 1;
    }
    hal_enableIRQs();
}

void hal_enableSleep (u1_t kind) {
    hal_disableIRQs();
    if (kind != HAL_SLEEP_DEEP) {
	HAL.veto_medium -= 1;
    }
    HAL.veto_deep -= 1;
    hal_enableIRQs();
}

// -----------------------------------------------------------------------------

#ifdef CFG_DEBUG
static void debug_init (void); // fwd decl
#endif

void hal_init (void* bootarg) {
    memset(&HAL, 0x00, sizeof(HAL));
    HAL.boottab = bootarg;
    HAL.battlevel = MCMD_DEVS_BATT_NOINFO;

    extern void pd_init (void);
    pd_init();

    hal_disableIRQs();

    clock_init();

    // sleep mode settings
    FLASH->ACR |= FLASH_ACR_SLEEP_PD;	// enable flash power-off during sleep

#if 1
    // disable single-wire debug (SWD) when running
    CFG_PIN_DEFAULT(BRD_GPIO(PORT_A, 13));
    CFG_PIN_DEFAULT(BRD_GPIO(PORT_A, 14));
#endif

#if defined(CFG_wailmer_board) || defined(CFG_wailord_board) || defined(CFG_coxproto_board) || defined(CFG_itrack_board)
    // ensure GNSS is turned off (there is no external pull-down on load switch control line)
    CFG_PIN(GPIO_GNSS_EN, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_400kHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
    SET_PIN(GPIO_GNSS_EN, 0);
#endif

#if defined(CFG_bootloader) && defined(CFG_bootloader_aes)
    // ensure we have bootloader that supports AES
    ASSERT(HAL.boottab->version >= 3);
#endif

    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();

#if defined(GPIO_TCXO_PWR)
    // XXX
    CFG_PIN(GPIO_TCXO_PWR, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_400kHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
    SET_PIN(GPIO_TCXO_PWR, 1);
    hal_waitUntil(os_getTime() + ms2osticks(10));
#endif

    hal_enableIRQs();

    leds_init();
#ifdef BRD_USART
    usart_init();
#endif
#ifdef BRD_VIBE_TIM
    vibe_init();
#endif

#if defined(CFG_perso)
    perso_enter();
#endif

#ifdef CFG_DEBUG
    debug_init();
#endif

#if defined(SVC_eefs)
    eefs_init((void*) APPDATA_BASE, APPDATA_SZ);
#endif

}

const irqdef HAL_irqdefs[] = {
#if defined(STM32L0)
    { EXTI0_1_IRQn, EXTI_IRQHandler },
    { EXTI2_3_IRQn, EXTI_IRQHandler },
    { EXTI4_15_IRQn, EXTI_IRQHandler },
#elif defined(STM32L1)
    { EXTI0_IRQn, EXTI_IRQHandler },
    { EXTI1_IRQn, EXTI_IRQHandler },
    { EXTI2_IRQn, EXTI_IRQHandler },
    { EXTI3_IRQn, EXTI_IRQHandler },
    { EXTI4_IRQn, EXTI_IRQHandler },
    { EXTI9_5_IRQn, EXTI_IRQHandler },
    { EXTI15_10_IRQn, EXTI_IRQHandler },
#endif

    { TIMx_IRQn, TIMx_IRQHandler },
    { RTC_IRQn, RTC_IRQHandler },

#if defined(BRD_I2C)
#if BRD_I2C == 1
    { I2C1_IRQn, i2c_irq },
#endif
#endif

#if defined(BRD_USART)
#if BRD_USART == 1
    { USART1_IRQn, usart_irq },
#elif BRD_USART == BRD_LPUART(1)
    { LPUART1_IRQn, usart_irq },
#endif
#endif

#if defined(BRD_PWM_TIM)
#if BRD_PWM_TIM == 3
    { TIM3_IRQn, pwm_irq },
#endif
#endif

#if defined(BRD_PIR_TIM)
#if BRD_PIR_TIM == 3
    { TIM3_IRQn, pir_tim_irq },
#endif
#endif

#if defined(BRD_LED_TIM)
#if BRD_LED_TIM == 2
    { TIM2_IRQn, leds_pwm_irq },
#endif
#endif

#if defined(BRD_IR_TIM)
    { TIM2_IRQn, ir_tim_irq },
#endif

    { ~0, NULL } // end of list
};

unsigned int crc32 (void* ptr, int nwords) {
    return HAL.boottab->crc32(ptr, nwords);
}

u1_t hal_getBattLevel (void) {
    return HAL.battlevel;
}

void hal_setBattLevel (u1_t level) {
    HAL.battlevel = level;
}


#ifdef CFG_DEBUG

#if BRD_DBG_UART == 1
#define DBG_USART USART1
#define DBG_USART_enable()		do { RCC->APB2ENR |= RCC_APB2ENR_USART1EN; } while (0)
#define DBG_USART_disable()		do { RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN; } while (0)
#elif BRD_DBG_UART == 2
#define DBG_USART USART2
#define DBG_USART_enable()		do { RCC->APB1ENR |= RCC_APB1ENR_USART2EN; } while (0)
#define DBG_USART_disable()		do { RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN; } while (0)
#elif BRD_DBG_UART == 4
#define DBG_USART USART4
#define DBG_USART_enable()		do { RCC->APB1ENR |= RCC_APB1ENR_USART4EN; } while (0)
#define DBG_USART_disable()		do { RCC->APB1ENR &= ~RCC_APB1ENR_USART4EN; } while (0)
#endif
#if defined(STM32L0)
#define DBG_USART_WRITE(c)              do { DBG_USART->TDR = (c); } while (0)
#define DBG_USART_BUSY()                ((DBG_USART->ISR & USART_ISR_TXE) == 0)
#define DBG_USART_TXING()               ((DBG_USART->ISR & USART_ISR_TC) == 0)
#elif defined(STM32L1)
#define DBG_USART_WRITE(c)              do { DBG_USART->DR = (c); } while (0)
#define DBG_USART_BUSY()                ((DBG_USART->SR & USART_SR_TXE) == 0)
#define DBG_USART_TXING()               ((DBG_USART->SR & USART_SR_TC) == 0)
#endif

static void debug_init (void) {
    // configure USART (115200/8N1, tx-only)
    DBG_USART_enable();
    DBG_USART->BRR = 278; // 115200 (APB1 clock @32MHz)
    DBG_USART->CR1 = USART_CR1_UE | USART_CR1_TE; // usart + transmitter enable
    DBG_USART_disable();
#if CFG_DEBUG != 0
    debug_str("\r\n============== DEBUG STARTED ==============\r\n");
#endif
}

void hal_debug_str (const char* str) {
    DBG_USART_enable();
    CFG_PIN_AF(GPIO_DBG_TX, GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
    char c;
    while( (c = *str++) ) {
        while( DBG_USART_BUSY() );
        DBG_USART_WRITE(c);
    }
    while( DBG_USART_TXING() );
    CFG_PIN(GPIO_DBG_TX, GPIOCFG_MODE_INP | GPIOCFG_OSPEED_400kHz | GPIOCFG_OTYPE_OPEN | GPIOCFG_PUPD_PUP);
    DBG_USART_disable();
}

void hal_debug_led (int val) {
#if defined(GPIO_DBG_LED)
    leds_set(GPIO_DBG_LED, val);
#endif
}

#endif


void hal_fwinfo (hal_fwi* fwi) {
    fwi->blversion = HAL.boottab->version;

    extern volatile hal_fwhdr fwhdr;
    fwi->version = fwhdr.version;
    fwi->crc = fwhdr.boot.crc;
    fwi->flashsz = FLASH_SIZE;
}

u4_t hal_unique (void) {
    uint32_t unique[3] = {
        *((uint32_t*) UNIQUE_ID0),
        *((uint32_t*) UNIQUE_ID1),
        *((uint32_t*) UNIQUE_ID2)
    };
    return crc32(&unique, 3);
}

void hal_reboot (void) {
    NVIC_SystemReset();
    // not reached
    hal_failed();
}

// persistent storage of stack data
typedef struct {
    uint32_t    dnonce[4];      // dev nonce history
    uint32_t    jnonce[4];      // join nonce history
} pdata;

// Note: The next nonce is stored, and the writes are spread over 4 fields. At
// 100k write cycles, this will allow 400k join requests. The DevNonce counter
// is only 16 bits, so it will roll-over much earlier, but the counter can be
// restarted at 0 if/when the Join EUI changes.

u4_t hal_dnonce_next (void) {
    pdata* p = (pdata*) STACKDATA_BASE;
    int x = 0;
    while( x < 3 && p->dnonce[x] < p->dnonce[x + 1] ) {
        x += 1;
    }
    u4_t dn = p->dnonce[x];
    eeprom_write(p->dnonce + ((x + 1) & 3), dn + 1);
    return dn;
}

void hal_dnonce_clear (void) {
    pdata* p = (pdata*) STACKDATA_BASE;
    for( int i = 0; i < 4; i++) {
        eeprom_write(p->dnonce + i, 0);
    }
}
