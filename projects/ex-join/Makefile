TARGET  := b_l072z_lrwan1
#TARGET  := nucleo_l053r8-sx1276mbed
#TARGET  := nucleo_l053r8-sx1261mbed
#TARGET  := nucleo_l053r8-sx1262mbed
#TARGET  := unicorn

VARIANTS := eu868 us915

CFLAGS += -Os
CFLAGS += -g
CFLAGS += -Wall -Wno-main

SVCS	+= app

DEFS	+= -DDEBUG_RX
DEFS	+= -DDEBUG_TX

LMICCFG	+= eeprom_keys
LMICCFG	+= DEBUG
LMICCFG	+= extapi

include ../projects.gmk
