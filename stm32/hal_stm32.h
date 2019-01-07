// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#ifndef _hal_stm32_h_
#define _hal_stm32_h_

// NVIC interrupt definition
typedef struct {
    uint32_t	num;		// NVIC interrupt number
    void*	handler;	// Pointer to handler function
} irqdef;

extern const irqdef HAL_irqdefs[];

#endif
