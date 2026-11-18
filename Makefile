# SPDX-License-Identifier: GPL-2.0 OR MIT
# Copyright (C) 2015-2023 Ichiro Kawazome

#
# For in kernel tree variables
# 
obj-$(CONFIG_UIOMEM) += uiomem.o
ifdef UIOMEM_CONFIG
ccflags-y            += -DUIOMEM_CONFIG=$(UIOMEM_CONFIG)
endif
ifdef UIOMEM_DEBUG
ccflags-y            += -DUIOMEM_DEBUG=$(UIOMEM_DEBUG)
endif
ifdef UIOMEM_IOCTL
ccflags-y            += -DUIOMEM_IOCTL=$(UIOMEM_IOCTL)
endif
ifdef UIOMEM_STATIC_DEVICE
ccflags-y            += -DUIOMEM_STATIC_DEVICE=$(UIOMEM_STATIC_DEVICE)
endif

#
# For out of kernel tree variables
#
CONFIG_UIOMEM                ?= m
CONFIG_UIOMEM_DEBUG          ?= y
CONFIG_UIOMEM_IOCTL          ?= y
CONFIG_UIOMEM_STATIC_DEVICE  ?= 4

CONFIG_OPTIONS := CONFIG_UIOMEM=$(CONFIG_UIOMEM)
CONFIG_OPTIONS += UIOMEM_CONFIG=1
ifeq ($(CONFIG_UIOMEM_DEBUG), y)
CONFIG_OPTIONS += UIOMEM_DEBUG=1
else
CONFIG_OPTIONS += UIOMEM_DEBUG=0
endif
ifeq ($(CONFIG_UIOMEM_IOCTL), y)
CONFIG_OPTIONS += UIOMEM_IOCTL=3
else
CONFIG_OPTIONS += UIOMEM_IOCTL=0
endif
ifneq ($(filter 0 1 2 3 4 5 6 7 8,$(CONFIG_UIOMEM_STATIC_DEVICE)),)
CONFIG_OPTIONS += UIOMEM_STATIC_DEVICE=$(CONFIG_UIOMEM_STATIC_DEVICE)
else
CONFIG_OPTIONS += UIOMEM_STATIC_DEVICE=0
endif

HOST_ARCH ?= $(shell uname -m | sed $(SUBARCH_SCRIPT))
ARCH      ?= $(shell uname -m | sed $(SUBARCH_SCRIPT))

SUBARCH_SCRIPT := -e s/i.86/x86/ -e s/x86_64/x86/ \
		  -e s/sun4u/sparc64/ \
		  -e s/arm.*/arm/ -e s/sa110/arm/ \
		  -e s/s390x/s390/ \
		  -e s/ppc.*/powerpc/ -e s/mips.*/mips/ \
		  -e s/sh[234].*/sh/ -e s/aarch64.*/arm64/ \
		  -e s/riscv.*/riscv/ -e s/loongarch.*/loongarch/

ifeq ($(ARCH), arm)
 ifneq ($(HOST_ARCH), arm)
   CROSS_COMPILE ?= arm-linux-gnueabihf-
 endif
endif
ifeq ($(ARCH), arm64)
 ifneq ($(HOST_ARCH), arm64)
   CROSS_COMPILE ?= aarch64-linux-gnu-
 endif
endif

ifdef KERNEL_SRC
  KERNEL_SRC_DIR := $(KERNEL_SRC)
else
  KERNEL_SRC_DIR ?= /lib/modules/$(shell uname -r)/build
endif

#
# For out of kernel tree rules
#
all:
	$(MAKE) -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) $(CONFIG_OPTIONS) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) $(CONFIG_OPTIONS) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean

