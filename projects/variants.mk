# Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
#
# This file is subject to the terms and conditions defined in file 'LICENSE',
# which is part of this source code package.

VARIANTS ?= eu868

ifeq ($(VARIANT),)
    VTARGET	:= variants
    VARIANT	:= $(firstword $(DEFAULT_VARIANT) $(VARIANTS))
else
    VTARGET	:= all
endif

BUILDDIR	= $(BUILDDIR_PFX)$(VARIANT)
VTARGETS	:= $(addprefix variant-,$(VARIANTS))

EMPTY		:=
SPACE		:= $(EMPTY) $(EMPTY)

VPARTS		:= $(subst -, ,$(VARIANT))
V_REGION	:= $(lastword $(VPARTS))
V_NONREGION	:= $(addprefix _,$(subst $(SPACE),_,$(filter-out $(V_REGION),$(VPARTS))))


ifneq (,$(filter eu868,$(VPARTS)))
LMICCFG	+= eu868
HWIDext := 868
endif

ifneq (,$(filter us915,$(VPARTS)))
LMICCFG	+= us915
HWIDext := 915
endif

ifneq (,$(filter as923,$(VPARTS)))
LMICCFG	+= as923
HWIDext := 915
endif

ifneq (,$(filter il915,$(VPARTS)))
LMICCFG	+= il915
HWIDext := 915
endif

ifneq (,$(filter kr920,$(VPARTS)))
LMICCFG	+= kr920
HWIDext := 915
endif

ifneq (,$(filter au915,$(VPARTS)))
LMICCFG	+= au915
HWIDext := 915
endif

ifneq (,$(filter uftonly,$(VPARTS)))
DEFS   += -DTABS_UFT_ONLY
UNDEFS += -DTABS_UFT_SLOW -DTABS_UFT_FAST
endif

HWID.$(VARIANT)	?= $(HWID.$(HWIDext))
