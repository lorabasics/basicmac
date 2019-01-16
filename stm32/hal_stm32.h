// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#ifndef _hal_stm32_h_
#define _hal_stm32_h_

#include "hw.h"

// NVIC interrupt definition
typedef struct {
    uint32_t	num;		// NVIC interrupt number
    void*	handler;	// Pointer to handler function
} irqdef;

extern const irqdef HAL_irqdefs[];

void usart_init (void);
void usart_irq (void);

void flash_write (uint32_t* dst, uint32_t* src, uint32_t nwords, bool erase);

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



#endif
