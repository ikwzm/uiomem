# SPDX-License-Identifier: GPL-2.0 OR MIT
# Copyright (C) 2015-2023 Ichiro Kawazome

#
# For in kernel tree variables
# 
obj-$(CONFIG_UIOMEM) += uiomem.o

#
# For out of kernel tree variables
#
CONFIG_UIOMEM  ?= m

CONFIG_OPTIONS := CONFIG_UIOMEM=$(CONFIG_UIOMEM)

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

