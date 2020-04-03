PROJECT = blink
BUILD_DIR = bin

CFILES = blink.c

DEVICE?=stm32l476vg
OPENCM3_DIR=libopencm3

OOCD_INTERFACE=stlink
OOCD_TARGET=stm32l4x

include $(OPENCM3_DIR)/mk/genlink-config.mk
include rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk

gdb: blink.elf
	gdb-multiarch $<

tags:
	ctags -R .
