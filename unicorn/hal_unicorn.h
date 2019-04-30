//  __ __   ___ __ __    _________________________________
// (_ |_ |V| | |_ /  |_| (C) 2018-2018 Semtech Corporation
// __)|__| | | |__\__| |               All rights reserved

#ifndef _hal_unicorn_h_
#define _hal_unicorn_h_

#include "hw.h"
#include "boottab.h"

uint32_t pio_irq_get (void);
void pio_irq_clear (uint32_t mask);
void pio_irq_enable (unsigned int gpio, bool enable);
void pio_irq_config (unsigned int pin, bool rising, bool falling);

// Personalization data (persodata.c)
void pd_init (void);
bool pd_verify (void);

#if defined(SVC_frag)
// Glue for Fragmentation service

#define fuota_flash_pagesz FLASH_PAGE_SZ
#define fuota_flash_bitdefault 0

#define fuota_flash_write(dst,src,nwords,erase) \
    flash_write((uint32_t*) (dst), (uint32_t*) (src), nwords, erase)

#define fuota_flash_read(dst,src,nwords) \
    memcpy(dst, src, (nwords) << 2)

#define fuota_flash_rd_u4(addr) \
    (*((uint32_t*) (addr)))

#define fuota_flash_rd_ptr(addr) \
    (*((void**) (addr)))

#endif

// Firmware header -- do not modify (append only)
typedef struct {
    boot_fwhdr boot;

    uint32_t version;
} hal_fwhdr;


#endif
