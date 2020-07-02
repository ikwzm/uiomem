uiomem(User space mappable I/O Memory)
==================================================================================

**This project is under development.**

See the develop branch for this project for details.

https://github.com/ikwzm/uiomem/tree/develop

# Overview

## Introduction of uiomem

uiomem is a Linux device driver for accessing a memory area outside the Linux
Kernel management from user space.

uiomem has following features.

  * uiomem can enable CPU cache, so it can access memory at high speed.
  * uiomem can manually invalidiate and flush the CPU cache.
  * uiomem can be freely attached and detached from Linux Kernel.

It is possible to access memory from the user space by opneing the device
file(e.g. /dev/uiomem0) and mapping to the user memory space, or using
the read()/write() functions.

The start address and size of the allocated memory area can be specified when 
the device driver is loaded (e.g. when loaded via the `insmod` command).
Some platforms allow to specify them in the device tree.


## Supported platforms

* OS : Linux Kernel Version 4.19, 5.4 (the author tested on 5.4).
* CPU: ARMv7 Cortex-A9 (Xilinx ZYNQ / Altera CycloneV SoC)
* CPU: ARM64 Cortex-A53 (Xilinx ZYNQ UltraScale+ MPSoC)

# Usage

## Compile

The following `Makefile` is included in the repository.

```Makefile:Makefile
HOST_ARCH   ?= $(shell uname -m | sed -e s/arm.*/arm/ -e s/aarch64.*/arm64/)
ARCH        ?= $(shell uname -m | sed -e s/arm.*/arm/ -e s/aarch64.*/arm64/)
KERNEL_SRC  ?= /lib/modules/$(shell uname -r)/build

ifeq ($(ARCH), arm)
 ifneq ($(HOST_ARCH), arm)
   CROSS_COMPILE  ?= arm-linux-gnueabihf-
 endif
endif
ifeq ($(ARCH), arm64)
 ifneq ($(HOST_ARCH), arm64)
   CROSS_COMPILE  ?= aarch64-linux-gnu-
 endif
endif

uiomem-obj           := uiomem.o
obj-$(CONFIG_UIOMEM) += $(uiomem-obj)

ifndef UIOMEM_MAKE_TARGET
  KERNEL_VERSION_LT_5 ?= $(shell awk '/^VERSION/{print int($$3) < 5}' $(KERNEL_SRC)/Makefile)
  ifeq ($(KERNEL_VERSION_LT_5), 1)
    UIOMEM_MAKE_TARGET ?= modules
  else
    UIOMEM_MAKE_TARGET ?= uiomem.ko
  endif
endif

all:
	make -C $(KERNEL_SRC) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) obj-m=$(uiomem-obj) $(UIOMEM_MAKE_TARGET)

clean:
	make -C $(KERNEL_SRC) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean

```

## Install

Load the uiomem kernel driver using `insmod`.
The start address and size of the allocated memory area should be provided as an argument as follows.
The device driver is created, and allocates memory area with the specified address and size.
The memory area that can be specified must be aligned with the page size.
The maximum number of memory area that can be allocated using `insmod` is 8 (uiomem0/1/2/3/4/5/6/7).

```console
shell$ sudo insmod uiomem.ko uiomem0_addr=0x0400000000 uiomem0_size=0x00040000
[13494.188047] uiomem uiomem0: driver version = 0.0.1
[13494.192932] uiomem uiomem0: major number   = 510
[13494.197572] uiomem uiomem0: minor number   = 0
[13494.202015] uiomem uiomem0: range address  = 0x0000000400000000
[13494.207933] uiomem uiomem0: range size     = 262144
[13494.212896] uiomem uiomem.0: driver installed.
shell$ ls -la /dev/udmabuf0
crw------- 1 root root 510, 0 Jun 30 10:19 /dev/uiomem0
```

The module can be uninstalled by the `rmmod` command.

```console
shell$ sudo rmmod uiomem
[13742.388123] uiomem uiomem.0: driver removed.
```

## Configuration via the device tree file

In addition to the allocation via the `insmod` command and its arguments,
memory area can be allocated by specifying the reg property in the device tree file.
When a device tree file contains an entry like the following, uiomem will allocate
memory area and create device drivers when loaded by `insmod`.

```devicetree:devicetree.dts
		#address-cells = <2>;
		#size-cells    = <2>;
		uiomem_plmem {
			compatible = "ikwzm,uiomem";
			device-name = "uiomem0";
			minor-number = <0>;
			reg = <0x04 0x00000000 0x0 0x00040000>;
		};

```

```console
shell$ sudo insmod uiomem.ko
[14208.028545] uiomem uiomem0: driver version = 0.0.1
[14208.033377] uiomem uiomem0: major number   = 509
[14208.038008] uiomem uiomem0: minor number   = 0
[14208.042448] uiomem uiomem0: range address  = 0x0000000400000000
[14208.048369] uiomem uiomem0: range size     = 262144
[14208.053244] uiomem 400000000.uiomem_plbram: driver installed.
shell$ ls -la /dev/uiomem0
crw------- 1 root root 509, 0 Jun 30 10:31 /dev/uiomem0
```

The following properties can be set in the device tree.

  *  `compatible`
  *  `reg`
  *  `shareable`
  *  `minor-number`
  *  `device-name`
  *  `sync-offset`
  *  `sync-size`
  *  `sync-direction`

### `compatible`

The `compatible` property is used to set the corresponding device driver when loading
uiomem. The `compatible` property is mandatory. Be sure to specify `compatible`
property as "ikwzm,uiomem".

### `reg`

The `reg` property specifies the physical address and size.
The `reg` property is used when uiomem allocates a buffer outside the management of Linux Kernel.
The memory area that can be specified must be aligned with the page size.

```devicetree:devicetree.dts
		#address-cells = <2>;
		#size-cells = <2>;
		uiomem@0xFFFC0000 {
			compatible = "ikwzm,uiomem";
			reg = <0x0 0xFFFC0000 0x0 0x00040000>;
		};
```

### `shareable`

The `shareable` property is specified when multiple uiomem shares the memory space specified by `reg` property.

```devicetree:devicetree.dts
		#address-cells = <2>;
		#size-cells = <2>;
		uiomem0 {
			compatible = "ikwzm,uiomem";
			reg = <0x0 0xFFFC0000 0x0 0x00040000>;
			shareable;
		};
		uiomem1 {
			compatible = "ikwzm,uiomem";
			reg = <0x0 0xFFFC0000 0x0 0x00040000>;
			shareable;
		};
```

### `minor-number`

The `minor-number` property is used to set the minor number.
The valid minor number range is 0 to 255. A minor number provided as `insmod`
argument will has higher precedence, and when definition in the device tree has
colliding number, creation of the device defined in the device tree will fail.

The `minor-number` property is optional. When the `minor-number` property is not
specified, uiomem automatically assigns an appropriate one.

```devicetree:devicetree.dts
		uiomem0 {
			compatible = "ikwzm,uiomem";
			minor-number = <0>;
			reg = <0x0 0xFFFC0000 0x0 0x00040000>;
		};

```

### `device-name`

The `device-name` property is used to set the name of device.

The `device-name` property is optional. The device name is determined as follow:

  1. If `device-name` property is specified, the value of `device-name` property is used.
  2. If `device-name` property is not present, and if `minor-number` property is
     specified, `sprintf("uiomem%d", minor-number)` is used.

### `sync-offset`

The `sync-offset` property is used to set the start of the buffer range when manually
controlling the cache of uiomem. 

The `sync-offset` property is optional.
When the `sync-offset` property is not specified, `sync-offset` is set to <0>.

### `sync-size`

The `sync-size` property is used to set the size of the buffer range when manually
controlling the cache of uiomem.

The `sync-size` property is optional.
When the `sync-size` property is not specified, `sync-size` is set to ths size specified by the `reg` property.

### `sync-direction`

The `sync-direction` property is used to set the direction of DMA when manually
controlling the cache of uiomem

  * `sync-direction`=<0>: Read and Write
  * `sync-direction`=<1>: Write Only
  * `sync-direction`=<2>: Read Only

The `sync-direction` property is optional.
When the `sync-direction` property is not specified, `sync-direction` is set to <0>.

```devicetree:devicetree.dts
		uiomem0 {
			compatible = "ikwzm,uiomem";
			reg = <0x0 0xFFFC0000 0x0 0x00040000>;
			sync-offset = <0x00010000>;
			sync-size = <0x000F0000>;
			sync-direction = <2>;
		};

```

## Device file

When uiomem is loaded into the kernel, the following device files are created.
`<device-name>` is a placeholder for the device name described in the previous section.

  * `/dev/<device-name>`
  * `/sys/class/uiomem/<device-name>/phys_addr`
  * `/sys/class/uiomem/<device-name>/size`
  * `/sys/class/uiomem/<device-name>/sync_offset`
  * `/sys/class/uiomem/<device-name>/sync_size`
  * `/sys/class/uiomem/<device-name>/sync_direction`
  * `/sys/class/uiomem/<device-name>/sync_owner`
  * `/sys/class/uiomem/<device-name>/sync_for_cpu`
  * `/sys/class/uiomem/<device-name>/sync_for_device`

### `/dev/<device-name>`

`/dev/<device-name>` is used when `mmap()`-ed to the user space or accessed via `read()`/`write()`.

```C:uiomem_test.c
    if ((fd  = open("/dev/uiomem0", O_RDWR)) != -1) {
        buf = mmap(NULL, buf_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
        /* Do some read/write access to buf */
        close(fd);
    }

```

The device file can be directly read/written by specifying the device as the target of `dd` in the shell.

```console
shell$  dd if=/dev/urandom of=/dev/uiomem0 bs=4096 count=64
64+0 records in
64+0 records out
262144 bytes (262 kB, 256 KiB) copied, 0.00341051 s, 76.9 MB/s
```

```console
shell$ dd if=/dev/uiomem0 of=random.bin bs=4096
64+0 records in
64+0 records out
262144 bytes (262 kB, 256 KiB) copied, 0.00192588 s, 136 MB/s
```

### `phys_addr`

The physical address of a memory area can be retrieved by reading `/sys/class/uiomem/<device-name>/phys_addr`.

```C:uiomem_test.c
    unsigned char  attr[1024];
    unsigned long  phys_addr;
    if ((fd  = open("/sys/class/uiomem/uiomem0/phys_addr", O_RDONLY)) != -1) {
        read(fd, attr, 1024);
        sscanf(attr, "%x", &phys_addr);
        close(fd);
    }

```

### `size`

The size of a memory area can be retrieved by reading `/sys/class/uiomem/<device-name>/size`.

```C:uiomem_test.c
    unsigned char  attr[1024];
    unsigned int   buf_size;
    if ((fd  = open("/sys/class/uiomem/uiomem0/size", O_RDONLY)) != -1) {
        read(fd, attr, 1024);
        sscanf(attr, "%d", &buf_size);
        close(fd);
    }

```

### `sync_offset`

The device file `/sys/class/uiomem/<device-name>/sync_offset` is used to specify
the start address of a memory block of which cache is manually managed.

```C:uiomem_test.c
    unsigned char  attr[1024];
    unsigned long  sync_offset = 0x00000000;
    if ((fd  = open("/sys/class/uiomem/uiomem0/sync_offset", O_WRONLY)) != -1) {
        sprintf(attr, "%d", sync_offset); /* or sprintf(attr, "0x%x", sync_offset); */
        write(fd, attr, strlen(attr));
        close(fd);
    }
```

### `sync_size`

The device file `/sys/class/uiomem/<device-name>/sync_size` is used to specify
the size of a memory block of which cache is manually managed.

```C:uiomem_test.c
    unsigned char  attr[1024];
    unsigned long  sync_size = 1024;
    if ((fd  = open("/sys/class/uiomem/uiomem0/sync_size", O_WRONLY)) != -1) {
        sprintf(attr, "%d", sync_size); /* or sprintf(attr, "0x%x", sync_size); */
        write(fd, attr, strlen(attr));
        close(fd);
    }
```

### `sync_direction`

The device file `/sys/class/uiomem/<device-name>/sync_direction` is used to set the
direction(Read/Write) of memory area of which cache is manually managed.

  - 0: sets Read and Write
  - 1: sets Write Only
  - 2: sets Read Only

```C:uiomem_test.c
    unsigned char  attr[1024];
    unsigned long  sync_direction = 1;
    if ((fd  = open("/sys/class/uiomem/uiomem0/sync_direction", O_WRONLY)) != -1) {
        sprintf(attr, "%d", sync_direction);
        write(fd, attr, strlen(attr));
        close(fd);
    }
```

### `sync_owner`

The device file `/sys/class/uiomem/<device-name>/sync_owner` reports the owner of
the memory block in the manual cache management mode.
If this value is 1, the buffer is owned by the device.
If this value is 0, the buffer is owned by the cpu.

```C:uiomem_test.c
    unsigned char  attr[1024];
    int sync_owner;
    if ((fd  = open("/sys/class/uiomem/uiomem0/sync_owner", O_RDONLY)) != -1) {
        read(fd, attr, 1024);
        sscanf(attr, "%x", &sync_owner);
        close(fd);
    }

```

### `sync_for_cpu`

In the manual cache management mode, CPU can be the owner of the buffer by writing
non-zero to the device file `/sys/class/uiomem/<device-name>/sync_for_cpu`.
This device file is write only.

If '1' is written to device file, if `sync_direction` is 2(=Read Only) or 0(=Read and Write),
the write to the device file invalidates a cache specified by `sync_offset` and `sync_size`.

```C:uiomem_test.c
    unsigned char  attr[1024];
    unsigned long  sync_for_cpu = 1;
    if ((fd  = open("/sys/class/uiomem/uiomem0/sync_for_cpu", O_WRONLY)) != -1) {
        sprintf(attr, "%d", sync_for_cpu);
        write(fd, attr, strlen(attr));
        close(fd);
    }
```

The value written to this device file can include sync_offset, sync_size, and sync_direction. 

```C:uiomem_test.c
    unsigned char  attr[1024];
    unsigned long  sync_offset    = 0;
    unsigned long  sync_size      = 0x10000;
    unsigned int   sync_direction = 1;
    unsigned long  sync_for_cpu   = 1;
    if ((fd  = open("/sys/class/uiomem/uiomem0/sync_for_cpu", O_WRONLY)) != -1) {
        sprintf(attr, "0x%08X%08X", (sync_offset & 0xFFFFFFFF), (sync_size & 0xFFFFFFF0) | (sync_direction << 2) | sync_for_cpu);
        write(fd, attr, strlen(attr));
        close(fd);
    }
```

The sync_offset/sync_size/sync_direction specified by ```sync_for_cpu``` is temporary and does not affect the ```sync_offset``` or ```sync_size``` or ```sync_direction``` device files.

### `sync_for_device`

In the manual cache management mode, DEVICE can be the owner of the buffer by
writing non-zero to the device file `/sys/class/uiomem/<device-name>/sync_for_device`.
This device file is write only.

If '1' is written to device file, if `sync_direction` is 1(=Write Only) or 0(=Read and Write),
the write to the device file flushes a cache specified by `sync_offset` and `sync_size` (i.e. the
cached data, if any, will be updated with data on DDR memory).

```C:uiomem_test.c
    unsigned char  attr[1024];
    unsigned long  sync_for_device = 1;
    if ((fd  = open("/sys/class/uiomem/uiomem0/sync_for_device", O_WRONLY)) != -1) {
        sprintf(attr, "%d", sync_for_device);
        write(fd, attr, strlen(attr));
        close(fd);
    }
```

The value written to this device file can include sync_offset, sync_size, and sync_direction. 

```C:uiomem_test.c
    unsigned char  attr[1024];
    unsigned long  sync_offset     = 0;
    unsigned long  sync_size       = 0x10000;
    unsigned int   sync_direction  = 1;
    unsigned long  sync_for_device = 1;
    if ((fd  = open("/sys/class/uiomem/uiomem0/sync_for_device", O_WRONLY)) != -1) {
        sprintf(attr, "0x%08X%08X", (sync_offset & 0xFFFFFFFF), (sync_size & 0xFFFFFFF0) | (sync_direction << 2) | sync_for_device);
        write(fd, attr, strlen(attr));
        close(fd);
    }
```

The sync_offset/sync_size/sync_direction specified by ```sync_for_device``` is temporary and does not affect the ```sync_offset``` or ```sync_size``` or ```sync_direction``` device files.



# Example using uiomem

  * https://github.com/ikwzm/PLBRAM-Ultra96
  * https://github.com/ikwzm/PLBRAM-ZYBO-Z7

