// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#include "peripherals.h"

#if defined(STM32L072xx)

void trng_next (uint32_t* dest, int count) {
    RCC->AHBENR |= RCC_AHBENR_RNGEN;
    RNG->CR |= RNG_CR_RNGEN;
    while( count-- > 0 ) {
        while( (RNG->SR & RNG_SR_DRDY) == 0 );
        *dest++ = RNG->DR;
    }
    RNG->CR &= ~RNG_CR_RNGEN;
    RCC->AHBENR &= ~RCC_AHBENR_RNGEN;
}

#endif
