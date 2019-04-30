# Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
#
# This file is subject to the terms and conditions defined in file 'LICENSE',
# which is part of this source code package.

# ------------------------------------------------
# Family: Simulation

ifneq (,$(filter unicorn,$(FAMILIES)))
    MCU		:= unicorn
    LD_SCRIPTS	+= $(BL)/src/arm/unicorn/ld/mem.ld
    CFLAGS	+= -Dunicorn
    BL_BRD	:= simul-unicorn
endif
