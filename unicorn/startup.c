//  __ __   ___ __ __    _________________________________
// (_ |_ |V| | |_ /  |_| (C) 2017-2018 Semtech Corporation
// __)|__| | | |__\__| |               All rights reserved

#include "lmic.h"

#include "boottab.h"

void _start (boot_boottab* boottab) {
    // symbols provided by linker script
    extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

    // initialize data
    uint32_t* src = &_sidata;
    uint32_t* dst = &_sdata;
    while( dst < &_edata ) {
	*dst++ = *src++;
    }

    // initialize bss
    dst = &_sbss;
    while( dst < &_ebss ) {
	*dst++ = 0;
    }

    // call main function
    extern void main (boot_boottab* boottab);
    main(boottab);
}

// Firmware header
__attribute__((section(".fwhdr")))
const volatile hal_fwhdr fwhdr = {
    // CRC and size will be patched by external tool
    .boot.crc           = 0,
    .boot.size          = BOOT_MAGIC_SIZE,
    .boot.entrypoint    = (uint32_t) _start,

    .version            = 0,
};
