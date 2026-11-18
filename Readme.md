uiomem(User space mappable I/O Memory)
==================================================================================

**This project is under development.**

See the develop branch for this project for details.

https://github.com/ikwzm/uiomem/tree/develop

Currently 1.4.0-beta.1 is tentatively released.

https://github.com/ikwzm/uiomem/tree/v1.4.0-beta.1.

# Overview

## Introduction to uiomem

uiomem is a Linux device driver that allows user space applications to access I/O memory regions that are not managed by the Linux kernel.

uiomem provides the following features:

  * uiomem can map I/O memory regions as cacheable, allowing the CPU to access them at high speed.
  * uiomem supports manual cache invalidation and write-back for mapped I/O memory regions to maintain cache coherency when the regions are shared with devices.
  * uiomem can be dynamically loaded into and unloaded from the Linux kernel.

User space applications can access I/O memory regions by opening a device file (e.g., `/dev/uiomem0`) and mapping the I/O memory into their address space using `mmap()`, or by using the `read()` and `write()` system calls.

The start address and size of the I/O memory region can be specified through module parameters when the driver is loaded (e.g., using the `insmod` command). On some platforms, these parameters can also be specified in the device tree.

## Supported platforms

* OS : Linux Kernel Version 4.19, 5.4, 6.1, 6.6, 6.12 (the author tested on 5.4 and 6.1 and 6.6 and 6.12).
* CPU: ARMv7 Cortex-A9 (Xilinx ZYNQ / Altera CycloneV SoC)
* CPU: ARM64 Cortex-A53 (Xilinx ZYNQ UltraScale+ MPSoC)
* CPU: RISC-V (Microchip PolarFire SoC)

# Usage

## Compile

This repository contains a [Makefie](./Makefile).
Makefile has the following Parameters:

| Parameter Name                       | Description                  | Default Value                        |
|--------------------------------------|-----------------------------------|--------------------------------------|
| ARCH                                 | Architecture Name                 | `$(shell uname -m \| sed -e s/arm.*/arm/ -e s/aarch64.*/arm64/ -e s/riscv.*/riscv/ )` |
| KERNEL_SRC                           | Kernel Source Directory           | `/lib/modules/$(shell uname -r)/build` |
| CONFIG_UIOMEM_STATIC_DEVICE          | Number of static devices (0 to 8) | 8 |
| CONFIG_UIOMEM_DEBUG                  | Enable debug report               | y |
| CONFIG_UIOMEM_IOCTL                  | Enable ioctl                      | y |

### Cross Compile

If you have a cross-compilation environment for target system, you can compile with:

```console
shell$ make ARCH=arm KERNEL_SRC=/home/fpga/src/linux-5.10.120-zynqmp-fpga-generic all
```
The ARCH variable specifies the architecture name.    
The KERNEL_SRC variable specifies the Linux Kernel source code path.    

### Self Compile

If your target system is capable of self-compiling the Linux Kernel module, you can compile it with:

```console
shell$ make all
```
You need the kernel source code in ```/lib/modules/$(shell uname -r)/build``` to compile.

### Build in Linux Source Tree

It can also be compiled into the Linux Kernel Source Tree.

#### Make directory in Linux Kernel Source Tree.

```console
shell$ mkdir <linux-source-tree>/drivers/staging/uiomem
```

#### Copy files to Linux Kernel Source Tree.

```console
shell$ cp Kconfig Makefile uiomem.c <linux-source-tree>/drivers/staging/uiomem
```

#### Add uiomem to Kconfig

```console
shell$ diff <linux-source-tree>/drivers/staging/Kconfig
  :
+source "drivers/staging/uiomem/Kconfig"
+
```

#### Add uiomem to Makefile

```Makefile
shell$ diff <linux-source-tree>/drivers/staging/Makefile
  :
+obj-$(CONFIG_UIOMEM) += uiomem/
```

#### Set CONFIG_UIOMEM

For make menuconfig, set the following:

```console
Device Drivers --->
  Staging drivers --->
    <M> uiomem(uiomem(User space mappable I/O memory)) --->
```

If you write it directly in defconfig:

```console
shell$ diff <linux-source-tree>/arch/arm64/configs/xilinx_zynqmp_defconfig
   :
+CONFIG_UIOMEM=m
```

## Install

Load the uiomem kernel driver using insmod.
Specify the start address and size of the I/O memory region as module parameters, as shown below.
The driver creates a device for the specified I/O memory region.
Both the start address and size of the I/O memory region must be aligned to the system page size.

The number of static devices that can be created when loading the module using insmod (uiomem0, uiomem1, ...) can be configured at build time using CONFIG_UIOMEM_STATIC_DEVICE.
The configurable range is 0 to 8, and the default value is 8.

```console
shell$ sudo insmod uiomem.ko uiomem0_addr=0x0400000000 uiomem0_size=0x00040000
[  562.657246] uiomem uiomem0: driver version = 1.4.0-beta.1
[  562.657264] uiomem uiomem0: ioctl version  = 1
[  562.657264] uiomem uiomem0: major number   = 238
[  562.657270] uiomem uiomem0: minor number   = 0
[  562.657275] uiomem uiomem0: range address  = 0x0000000400000000
[  562.657282] uiomem uiomem0: range size     = 262144
[  562.657282] uiomem uiomem0: cached         = 1
[  562.657282] uiomem uiomem0: coherent       = 0
[  562.657282] uiomem uiomem0: sync_operation = ARM64 Native
[  562.657282] uiomem uiomem0: shareable      = 0
[  562.657287] uiomem uiomem.0: driver installed.
shell$ ls -la /dev/uiomem0
crw------- 1 root root 238, 0 Oct 21 17:36 /dev/uiomem0
```

The module can be uninstalled by the `rmmod` command.

```console
shell$ sudo rmmod uiomem
[  657.672544] uiomem uiomem.0: driver removed.
```

## Configuration via the module parameters

The uiomem kernel module has the following module parameters:

| Parameter Name     | Type  | Default | Description                                |
|:-------------------|:------|---------|:-------------------------------------------|
| uiomem_addr[0-7]   | ulong |    0    | Start address of uiomem[0-7]               |
| uiomem_size[0-7]   | ulong |    0    | Size of uiomem[0-7]                        |
| uiomem_option[0-7] | ulong |    2    | Memory attributes of uiomem[0-7]           |
| info_enable        | int   |    1    | nable installation/uninstallation messages |

The `uiomem_option[0-7]` parameter is a bit field with the following flags:

  * `coherent` (bit 2): Indicates whether cache coherency is maintained by hardware.
    * 1: Hardware maintains cache coherency. No explicit cache management by the driver is required.
    * 0: Hardware does not maintain cache coherency. Explicit cache management by the driver is required.
  * `cache` (bit 1): Controls whether CPU caching is enabled when accessing I/O memory.
    * 1: CPU caching is enabled.
    * 0: CPU caching is disabled.
  * `shareable` (bit 0): Controls the shareability attribute of the memory region.

The `cache` and `coherent` flags are used together to select one of the following three cache modes:

| Cache Mode        | cache | coherent | Description            |
|:------------------|:-----:|:--------:|:-----------------------|
| cache-off         |   0   |    1     | CPU caching is disabled.<br>No explicit cache management is required. |
| cache-noncoherent |   1   |    0     | CPU caching is enabled.<br>The driver must explicitly manage cache coherency. |
| cache-coherent    |   1   |    1     | CPU caching is enabled.<br>Hardware maintains cache coherency, so the driver does not need to explicitly manage cache coherency. |


## Configuration via the device tree

In addition to specifying I/O memory regions through module parameters when loading the driver with `insmod`, I/O memory regions can also be specified using the `reg` property in the device tree.

When the device tree contains an entry like the following, uiomem creates a device for the specified I/O memory region when the driver is loaded using `insmod`.

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
[  773.889476] uiomem uiomem0: driver version = 1.4.0-beta.1
[  773.889496] uiomem uiomem0: ioctl version  = 1
[  773.889496] uiomem uiomem0: major number   = 237
[  773.889501] uiomem uiomem0: minor number   = 0
[  773.889506] uiomem uiomem0: range address  = 0x0000000400000000
[  773.889512] uiomem uiomem0: range size     = 262144
[  773.889512] uiomem uiomem0: cached         = 1
[  773.889512] uiomem uiomem0: coherent       = 0
[  773.889518] uiomem uiomem0: sync_operation = ARM64 Native
[  773.889512] uiomem uiomem0: shareable      = 0
[  773.889518] uiomem 400000000.uiomem_plbram: driver installed.
shell$ ls -la /dev/uiomem0
crw------- 1 root root 237, 0 Oct 21 17:40 /dev/uiomem0
```

The following properties can be set in the device tree.

  *  `compatible`
  *  `reg`
  *  `memory-region`
  *  `cache-off`
  *  `cache-noncoherent`
  *  `cache-coherent`
  *  `shareable`
  *  `minor-number`
  *  `device-name`
  *  `sync-offset`
  *  `sync-size`
  *  `sync-direction`

### `compatible`

The `compatible` property is used to identify the device driver that handles the device.    
This property is mandatory and must be set to `"ikwzm,uiomem"`.

### `reg`

The `reg` property specifies the physical address and size of an I/O memory region.    
It is used to specify an I/O memory region that is not managed by the Linux kernel.    
Both the start address and size of the I/O memory region must be aligned to the system page size.    
Either the `reg` property or the `memory-region` property is required.    

```devicetree:devicetree.dts
		#address-cells = <2>;
		#size-cells = <2>;
		uiomem@0xFFFC0000 {
			compatible = "ikwzm,uiomem";
			reg = <0x0 0xFFFC0000 0x0 0x00040000>;
		};
```

### `memory-region`

The `memory-region` property specifies a reserved memory region.   
The referenced reserved memory region must have the `no-map` property.    
Either the `reg` property or the `memory-region` property is required.   

```devicetree:devicetree.dts
		#address-cells = <2>;
		#size-cells = <2>;
		reserved_memory {
			ranges;
			image_buf0: image_buf@0 {
				no-map;
				reg = <0x0 0x70000000 0x0 0x10000000>;
				label = "image_buf0";
			};
		};
		uiomem@image_buf0 {
			compatible = "ikwzm,uiomem";
			memory-region = <&image_buf0>;
		};
```

### `cache-off`

The `cache-off` property specifies that CPU caching is disabled for the I/O memory region.
Since CPU caching is disabled, cache coherency management is not required.

```devicetree:devicetree.dts
		uiomem0 {
			compatible = "ikwzm,uiomem";
			reg = <0x0 0xFFFC0000 0x0 0x00040000>;
			cache-off;
		};

```

### `cache-noncoherent`

The `cache-noncoherent` property specifies that CPU caching is enabled for the I/O memory region, but cache coherency is not maintained by hardware.
Therefore, the driver must explicitly manage cache coherency.

```devicetree:devicetree.dts
		uiomem0 {
			compatible = "ikwzm,uiomem";
			reg = <0x0 0xFFFC0000 0x0 0x00040000>;
			cache-noncoherent;
		};

```

### `cache-coherent`

The `cache-coherent` property specifies that CPU caching is enabled for the I/O memory region and cache coherency is maintained by hardware.
Therefore, the driver does not need to explicitly manage cache coherency.

```devicetree:devicetree.dts
		uiomem0 {
			compatible = "ikwzm,uiomem";
			reg = <0x0 0xFFFC0000 0x0 0x00040000>;
			cache-coherent;
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

The `minor-number` property specifies the minor number of the device.

The valid minor number range is 0 to 255.
A minor number specified through an `insmod` argument takes precedence over the value specified in the device tree.
If the specified minor number conflicts with an existing device, the device defined in the device tree will not be created.

The `minor-number` property is optional.
If it is omitted, uiomem automatically assigns an available minor number.

```devicetree:devicetree.dts
		uiomem0 {
			compatible = "ikwzm,uiomem";
			minor-number = <0>;
			reg = <0x0 0xFFFC0000 0x0 0x00040000>;
		};

```

### `device-name`

The `device-name` property specifies the device name.

The `device-name` property is optional. If it is omitted, the device name is determined as follows:

  1. If `device-name` is specified, its value is used as the device name.
  2. If `device-name` is not specified but `minor-number` is specified, the device name is generated using `sprintf("uiomem%d", minor-number)`.

### `sync-offset`

The `sync-offset` property specifies the starting offset of the memory area for which the cache is manually managed.

The `sync-offset` property is optional.
If it is omitted, the starting offset defaults to `0`.

### `sync-size`

The `sync-size` property specifies the size of the memory area for which the cache is manually managed.

The `sync-size` property is optional.
If it is omitted, the size defaults to the size specified by the `reg` property.

### `sync-direction`

The `sync-direction` property specifies the direction of CPU access to the memory area:

  * `0`: Read and Write
  * `1`: Write Only
  * `2`: Read Only

The `sync-direction` property is optional.
If it is omitted, the direction defaults to `0` (Read and Write).

```devicetree:devicetree.dts
		uiomem0 {
			compatible = "ikwzm,uiomem";
			reg = <0x0 0xFFFC0000 0x0 0x00040000>;
			sync-offset = <0x00010000>;
			sync-size = <0x000F0000>;
			sync-direction = <2>;
		};
```

## Device Files

When uiomem is loaded, the following device files are created.
`<device-name>` is a placeholder for the device name described in the previous section.

  * `/dev/<device-name>`
  * `/sys/class/uiomem/<device-name>/phys_addr`
  * `/sys/class/uiomem/<device-name>/size`
  * `/sys/class/uiomem/<device-name>/cached`
  * `/sys/class/uiomem/<device-name>/coherent`
  * `/sys/class/uiomem/<device-name>/shareable`
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

The physical address of an I/O memory region can be retrieved by reading `/sys/class/uiomem/<device-name>/phys_addr`.

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

The size of an I/O memory region can be retrieved by reading `/sys/class/uiomem/<device-name>/size`.

```C:uiomem_test.c
    unsigned char  attr[1024];
    unsigned int   buf_size;
    if ((fd  = open("/sys/class/uiomem/uiomem0/size", O_RDONLY)) != -1) {
        read(fd, attr, 1024);
        sscanf(attr, "%d", &buf_size);
        close(fd);
    }
```

### `cached`

The sysfs attribute `/sys/class/uiomem/<device-name>/cached` indicates whether the I/O memory region is cacheable.

  * `1`: The I/O memory region is cacheable.
  * `0`: The I/O memory region is non-cacheable.

```C:uiomem_test.c
    unsigned char  attr[1024];
    unsigned int   cached;
    if ((fd  = open("/sys/class/uiomem/uiomem0/cached", O_RDONLY)) != -1) {
        read(fd, attr, 1024);
        sscanf(attr, "%d", &cached);
        close(fd);
    }
```

### `coherent`

The sysfs attribute `/sys/class/uiomem/<device-name>/coherent` indicates whether cache coherency is maintained by hardware for the I/O memory region.

  * `1`: The I/O memory region is coherent. Explicit cache synchronization is not required.
  * `0`: The I/O memory region is non-coherent. Explicit cache synchronization is required.

```C:uiomem_test.c
    unsigned char  attr[1024];
    unsigned int   coherent;
    if ((fd  = open("/sys/class/uiomem/uiomem0/coherent", O_RDONLY)) != -1) {
        read(fd, attr, 1024);
        sscanf(attr, "%d", &coherent);
        close(fd);
    }
```

### `shareable`

Whether the memory range of uiomem is shareable can be determined by reading `/sys/class/uiomem/<device-name>/shareable`.

```C:uiomem_test.c
    unsigned char  attr[1024];
    unsigned int   shareable;
    if ((fd  = open("/sys/class/uiomem/uiomem0/shareable", O_RDONLY)) != -1) {
        read(fd, attr, 1024);
        sscanf(attr, "%d", &shareable);
        close(fd);
    }
```

### `sync_offset`

The sysfs attribute `/sys/class/uiomem/<device-name>/sync_offset` specifies the starting offset, relative to the beginning of the I/O memory region, for which the cache is manually managed.

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

The sysfs attribute `/sys/class/uiomem/<device-name>/sync_size` specifies the size of the I/O memory region for which the cache is manually managed.

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

The sysfs attribute `/sys/class/uiomem/<device-name>/sync_direction` specifies the direction of CPU access to the I/O memory region for which the cache is manually managed.

  * `0`: Read and Write
  * `1`: Write Only
  * `2`: Read Only

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

The sysfs attribute `/sys/class/uiomem/<device-name>/sync_owner` reports the current owner of the I/O memory region in manual cache management mode.

  * `1`: The I/O memory region is owned by the device.
  * `0`: The I/O memory region is owned by the CPU.

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

In manual cache management mode, writing a non-zero value to the sysfs attribute `/sys/class/uiomem/<device-name>/sync_for_cpu` transfers ownership of the I/O memory region to the CPU.

This sysfs attribute is write-only.

When `1` is written to this attribute, the cache for the I/O memory region specified by `sync_offset` and `sync_size` is invalidated if `sync_direction` is `2` (Read Only) or `0` (Read and Write).

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

In manual cache management mode, writing a non-zero value to the sysfs attribute `/sys/class/uiomem/<device-name>/sync_for_device` transfers ownership of the I/O memory region to the device.

This sysfs attribute is write-only.

When `1` is written to this attribute, the cache for the I/O memory region specified by `sync_offset` and `sync_size` is written back to memory if `sync_direction` is `1` (Write Only) or `0` (Read and Write).

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

## ioctl

Starting with uiomem v1.3.0-beta.1, devices can be controlled by issuing ioctl to the device file.
The ioctl can do the following

 * `UIOMEM_IOCTL_GET_DRV_INFO`
 * `UIOMEM_IOCTL_GET_SIZE`
 * `UIOMEM_IOCTL_GET_PHYS_ADDR`
 * `UIOMEM_IOCTL_GET_SYNC_OWNER`
 * `UIOMEM_IOCTL_GET_DEV_INFO`
 * `UIOMEM_IOCTL_GET_SYNC`
 * `UIOMEM_IOCTL_SET_SYNC`
 * `UIOMEM_IOCTL_SET_SYNC_FOR_CPU`
 * `UIOMEM_IOCTL_SET_SYNC_FOR_DEVICE`

### uiomem-ioctl.h

The following header file is required to use ioctl.

```C:uiomem-ioctl.h
#ifndef  UIOMEM_IOCTL_H
#define  UIOMEM_IOCTL_H
#include <linux/ioctl.h>

#define DEFINE_UIOMEM_IOCTL_FLAGS(name,type,lo,hi)                     \
static const  int      UIOMEM_IOCTL_FLAGS_ ## name ## _SHIFT = (lo);   \
static const  uint64_t UIOMEM_IOCTL_FLAGS_ ## name ## _MASK  =         \
    (((hi)-(lo)+1) >= 64) ? ~((uint64_t)0) :                           \
                            (((uint64_t)1 << ((hi)-(lo)+1)) - 1);      \
static inline void SET_UIOMEM_IOCTL_FLAGS_ ## name(type *p, int value) \
{                                                                      \
    const int      shift = UIOMEM_IOCTL_FLAGS_ ## name ## _SHIFT;      \
    const uint64_t mask  = UIOMEM_IOCTL_FLAGS_ ## name ## _MASK;       \
    p->flags &= ~(mask << shift);                                      \
    p->flags |= ((value & mask) << shift);                             \
}                                                                      \
static inline int  GET_UIOMEM_IOCTL_FLAGS_ ## name(type *p)            \
{                                                                      \
    const int      shift = UIOMEM_IOCTL_FLAGS_ ## name ## _SHIFT;      \
    const uint64_t mask  = UIOMEM_IOCTL_FLAGS_ ## name ## _MASK;       \
    return (int)((p->flags >> shift) & mask);                          \
}

typedef struct {
    uint64_t flags;
    char     version[16];
    char     sync_operation[16];
} uiomem_ioctl_drv_info;

DEFINE_UIOMEM_IOCTL_FLAGS(IOCTL_VERSION    , uiomem_ioctl_drv_info ,  0,  7)
DEFINE_UIOMEM_IOCTL_FLAGS(STATIC_DEVICE_NUM, uiomem_ioctl_drv_info ,  8, 15)

typedef struct {
    uint64_t flags;
    uint64_t size;
    uint64_t addr;
} uiomem_ioctl_dev_info;

DEFINE_UIOMEM_IOCTL_FLAGS(SHAREABLE    , uiomem_ioctl_dev_info ,  0,  0)
DEFINE_UIOMEM_IOCTL_FLAGS(CACHED       , uiomem_ioctl_dev_info ,  1,  1)
DEFINE_UIOMEM_IOCTL_FLAGS(COHERENT     , uiomem_ioctl_dev_info ,  2,  2)

typedef struct {
    uint64_t flags;
    uint64_t size;
    uint64_t offset;
} uiomem_ioctl_sync_args;

DEFINE_UIOMEM_IOCTL_FLAGS(SYNC_CMD     , uiomem_ioctl_sync_args,  0,  1)
DEFINE_UIOMEM_IOCTL_FLAGS(SYNC_DIR     , uiomem_ioctl_sync_args,  2,  3)
DEFINE_UIOMEM_IOCTL_FLAGS(SYNC_FAST    , uiomem_ioctl_sync_args,  4,  4)
DEFINE_UIOMEM_IOCTL_FLAGS(SYNC_MODE    , uiomem_ioctl_sync_args,  8, 15)
DEFINE_UIOMEM_IOCTL_FLAGS(SYNC_OWNER   , uiomem_ioctl_sync_args, 16, 16)

enum {
    UIOMEM_IOCTL_FLAGS_SYNC_CMD_FOR_CPU    = 1,
    UIOMEM_IOCTL_FLAGS_SYNC_CMD_FOR_DEVICE = 3
};

#define UIOMEM_IOCTL_MAGIC               'U'
#define UIOMEM_IOCTL_GET_DRV_INFO        _IOR (UIOMEM_IOCTL_MAGIC, 1, uiomem_ioctl_drv_info)
#define UIOMEM_IOCTL_GET_SIZE            _IOR (UIOMEM_IOCTL_MAGIC, 2, uint64_t)
#define UIOMEM_IOCTL_GET_PHYS_ADDR       _IOR (UIOMEM_IOCTL_MAGIC, 3, uint64_t)
#define UIOMEM_IOCTL_GET_SYNC_OWNER      _IOR (UIOMEM_IOCTL_MAGIC, 4, uint32_t)
#define UIOMEM_IOCTL_SET_SYNC_FOR_CPU    _IOW (UIOMEM_IOCTL_MAGIC, 5, uint64_t)
#define UIOMEM_IOCTL_SET_SYNC_FOR_DEVICE _IOW (UIOMEM_IOCTL_MAGIC, 6, uint64_t)
#define UIOMEM_IOCTL_GET_DEV_INFO        _IOR (UIOMEM_IOCTL_MAGIC, 7, uiomem_ioctl_dev_info)
#define UIOMEM_IOCTL_GET_SYNC            _IOR (UIOMEM_IOCTL_MAGIC, 8, uiomem_ioctl_sync_args)
#define UIOMEM_IOCTL_SET_SYNC            _IOW (UIOMEM_IOCTL_MAGIC, 9, uiomem_ioctl_sync_args)

#endif /* #ifndef UIOMEM_IOCTL_H */
```

### Example of required header files

```C:uiomem-ioctl-test.c
#include <inttypes.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "uiomem-ioctl.h"
```

### `UIOMEM_IOCTL_GET_DRV_INFO`

This ioctl retrieves driver information.
The returned information includes the driver's capabilities and version information.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        uiomem_ioctl_drv_info drv_info = {0};
        int status = ioctl(fd, UIOMEM_IOCTL_GET_DRV_INFO, &drv_info);
	if (status == 0) {
            int static_device_num = GET_UIOMEM_IOCTL_FLAGS_STATIC_DEVICE_NUM(&drv_info);
            int ioctl_version     = GET_UIOMEM_IOCTL_FLAGS_IOCTL_VERSION(&drv_info);
            char* drv_version     = strdup(&drv_info.version[0]);
	}
        close(fd);
    }
```

### `UIOMEM_IOCTL_GET_SIZE`

This ioctl retrieves the size of an I/O memory region.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        uint64_t size;
        status = ioctl(fd, UIOMEM_IOCTL_GET_SIZE, &size);
        close(fd);
    }
```

### `UIOMEM_IOCTL_GET_PHYS_ADDR`

This ioctl retrieves the physical address of an I/O memory region.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        uint64_t phys_addr;
        status = ioctl(fd, UIOMEM_IOCTL_GET_PHYS_ADDR, &phys_addr);
        close(fd);
    }
```

### `UIOMEM_IOCTL_GET_SYNC_OWNER`

This ioctl retrieves the current owner of the I/O memory region in manual cache management mode.

  * `1`: The I/O memory region is owned by the device.
  * `0`: The I/O memory region is owned by the CPU.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        int sync_owner;
        status = ioctl(fd, UIOMEM_IOCTL_GET_SYNC_OWNER, &sync_owner);
        close(fd);
    }
```

### `UIOMEM_IOCTL_SET_SYNC_FOR_CPU`

This ioctl requests cache synchronization for CPU access.

The ioctl takes a 64-bit value as an argument.
When the value is `1`, the driver invalidates the cache range for the I/O memory region specified by `sync_offset` and `sync_size` if `sync_direction` is `2` (Read Only) or `0` (Read and Write).

The mutex protects the synchronization state and cache synchronization operation against concurrent access by code paths that use the same mutex.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        uint64_t sync_for_cpu = 1;
        status = ioctl(fd, UIOMEM_IOCTL_SET_SYNC_FOR_CPU, &sync_for_cpu);
        close(fd);
    }
```

The 64-bit value passed to this ioctl can also encode `sync_offset`, `sync_size`, and `sync_direction`.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        unsigned long sync_offset     = 0;
        unsigned long sync_size       = 0x10000;
        unsigned int  sync_direction  = 0;
        uint64_t      sync_for_cpu    = ((uint64_t)(sync_offset    & 0xFFFFFFFF) << 32) |
                                        ((uint64_t)(sync_size      & 0xFFFFFFF0) <<  0) |
                                        ((uint64_t)(sync_direction & 0x00000003) <<  2) |
                                        0x00000001;
        status = ioctl(fd, UIOMEM_IOCTL_SET_SYNC_FOR_CPU, &sync_for_cpu);
        close(fd);
    }
```

The `sync_offset`, `sync_size`, and `sync_direction` values encoded in `sync_for_cpu` apply only to the current synchronization request. They do not modify the synchronization parameters configured for the device.

### `UIOMEM_IOCTL_SET_SYNC_FOR_DEVICE`

This ioctl requests cache synchronization for device access.

The ioctl takes a 64-bit value as an argument.
When the value is `1`, the driver writes back the cache range for the I/O memory region specified by `sync_offset` and `sync_size` if `sync_direction` is `1` (Write Only) or `0` (Read and Write).

The mutex protects the synchronization state and cache synchronization operation against concurrent access by code paths that use the same mutex.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        uint64_t sync_for_device = 1;
        status = ioctl(fd, UIOMEM_IOCTL_SET_SYNC_FOR_DEVICE, &sync_for_device);
        close(fd);
    }
```

The 64-bit value passed to this ioctl can also encode `sync_offset`, `sync_size`, and `sync_direction`.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        unsigned long sync_offset     = 0;
        unsigned long sync_size       = 0x10000;
        unsigned int  sync_direction  = 0;
        uint64_t      sync_for_device = ((uint64_t)(sync_offset    & 0xFFFFFFFF) << 32) |
                                        ((uint64_t)(sync_size      & 0xFFFFFFF0) <<  0) |
                                        ((uint64_t)(sync_direction & 0x00000003) <<  2) |
                                        0x00000001;
        status = ioctl(fd, UIOMEM_IOCTL_SET_SYNC_FOR_DEVICE, &sync_for_device);
        close(fd);
    }
```

The `sync_offset`, `sync_size`, and `sync_direction` values encoded in `sync_for_device` apply only to the current synchronization request. They do not modify the synchronization parameters configured for the device.

### `UIOMEM_IOCTL_SET_SYNC_FOR_DEVICE`

This ioctl requests cache synchronization for device access.

The ioctl writes a 64-bit value to `sync_for_device`.
When the value is `1`, the driver writes back the cache range specified by `sync_offset` and `sync_size` to memory if `sync_direction` is `1` (Write Only) or `0` (Read and Write).

The mutex protects the synchronization state and operation against concurrent access through code paths that use the same mutex.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        uint64_t sync_for_device = 1;
        status = ioctl(fd, UIOMEM_IOCTL_SET_SYNC_FOR_DEVICE, &sync_for_device);
        close(fd);
    }
```

The 64-bit value passed to this ioctl can also encode `sync_offset`, `sync_size`, and `sync_direction`.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        unsigned long sync_offset     = 0;
        unsigned long sync_size       = 0x10000;
        unsigned int  sync_direction  = 0;
        uint64_t      sync_for_device = ((uint64_t)(sync_offset    & 0xFFFFFFFF) << 32) |
                                        ((uint64_t)(sync_size      & 0xFFFFFFF0) <<  0) |
                                        ((uint64_t)(sync_direction & 0x00000003) <<  2) |
                                        0x00000001;
        status = ioctl(fd, UIOMEM_IOCTL_SET_SYNC_FOR_DEVICE, &sync_for_device);
        close(fd);
    }
```

The sync_offset/sync_size/sync_direction specified by ```sync_for_device``` is temporary and does not affect the ```sync_offset``` or ```sync_size``` or ```sync_direction``` device files.

### `UIOMEM_IOCTL_GET_DEV_INFO`

This ioctl retrieves device information.

The returned information includes the physical address and size of the I/O memory region, as well as its memory attributes.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        uiomem_ioctl_dev_info dev_info = {0};
        status = ioctl(fd, UIOMEM_IOCTL_GET_DEV_INFO, &dev_info);
	if (status == 0) {
            int      shareable = Get_UIOMEM_IOCTL_FLAGS_SHAREABLE(&dev_info);
            int      cached    = GET_UIOMEM_IOCTL_FLAGS_CACHED(&dev_info);
            int      coherent  = GET_UIOMEM_IOCTL_FLAGS_COHRENT(&dev_info);
            uint64_t phys_addr = dev_info.addr;
            uint64_t size      = dev_info.size;
	}
        close(fd);
    }
```

### `UIOMEM_IOCTL_GET_SYNC`

This ioctl retrieves the synchronization parameters, including `sync_offset`, `sync_size`, `sync_direction`, `sync_owner`, and `sync_mode`.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        uiomem_ioctl_sync_args sync_args = {0};
        status = ioctl(fd, UIOMEM_IOCTL_GET_SYNC, &sync_args);
	if (status == 0) {
            uint64_t sync_offset    = sync_args.offset;
            uint64_t sync_size      = sync_args.size;
            int      sync_direction = GET_UIOMEM_IOCTL_FLAGS_SYNC_DIR(&sync_args);
            int      sync_owner     = GET_UIOMEM_IOCTL_FLAGS_SYNC_OWNER(&sync_args);
            int      sync_mode      = GET_UIOMEM_IOCTL_FLAGS_SYNC_MODE(&sync_args);
	}
        close(fd);
    }
```

### `UIOMEM_IOCTL_SET_SYNC`

This ioctl sets `sync_offset`, `sync_size`, `sync_direction`, and `sync_mode`.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        uiomem_ioctl_sync_args sync_args = {0};
        uint64_t sync_offset    = 0;
        uint64_t sync_size      = 0x10000;
        int      sync_direction = 0;
        sync_args.offset = sync_offset;
        sync_args.size   = sync_size;
        SET_UIOMEM_IOCTL_FLAGS_SYNC_DIR(&sync_args, sync_direction);
        status = ioctl(fd, UIOMEM_IOCTL_SET_SYNC, &sync_args);
        close(fd);
    }
```

A synchronization command can also be specified in the `flags` field of `sync_args` to trigger synchronization for the CPU or the device.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        uiomem_ioctl_sync_args sync_args = {0};
        uint64_t sync_offset    = 0;
        uint64_t sync_size      = 0x10000;
        int      sync_direction = 0;
        sync_args.offset = sync_offset;
        sync_args.size   = sync_size;
        SET_UIOMEM_IOCTL_FLAGS_SYNC_DIR(&sync_args, sync_direction);
        SET_UIOMEM_IOCTL_FLAGS_SYNC_CMD(&sync_args, UIOMEM_IOCTL_FLAGS_SYNC_CMD_FOR_CPU);
        status = ioctl(fd, UIOMEM_IOCTL_SET_SYNC, &sync_args);
        close(fd);
    }
```

#### Normal Synchronization

When `SYNC_FAST` is not set, `UIOMEM_IOCTL_SET_SYNC` uses the normal synchronization path.

- The driver acquires a mutex before updating the synchronization parameters and performing the synchronization operation.
- The `sync_direction` value is updated only when the specified value is `0`, `1`, or `2`. Other values leave the current setting unchanged.
- If the mode bits in the `flags` field of `uiomem_ioctl_sync_args` are zero, `sync_mode` is not updated.
- The `sync_offset` value is updated using the `offset` field of `uiomem_ioctl_sync_args`.
- If the `size` field of `uiomem_ioctl_sync_args` is zero, `sync_size` is not updated.
- Depending on the synchronization command, the driver performs synchronization for the CPU or the device using the current synchronization parameters.

The requested memory area is validated before the synchronization operation. If acquiring the mutex is interrupted, the ioctl returns `-ERESTARTSYS`.

#### Fast Synchronization (`SYNC_FAST`)

Starting with IOCTL version 3, the `SYNC_FAST` flag can be used with `UIOMEM_IOCTL_SET_SYNC` to perform cache synchronization with reduced overhead.

When `SYNC_FAST` is set, the driver:

- Skips mutex locking.
- Does not update the synchronization parameters (`sync_offset`, `sync_size`, `sync_direction`, and `sync_mode`) or the synchronization state (`sync_owner`) maintained by the driver.
- Uses the `offset`, `size`, and `SYNC_DIR` fields supplied in the ioctl arguments for the synchronization operation.

The driver validates the requested memory area before performing synchronization.

**Important considerations:**

- Since mutex locking and synchronization state updates are skipped, the caller is responsible for coordinating CPU and device access and ensuring that synchronization operations do not conflict with other operations on the same memory area.
- The synchronization parameters supplied with each request must be valid for the target memory area.
- When `SYNC_FAST` is not set, the driver uses the normal synchronization path, including mutex locking and synchronization parameter updates.

Use `SYNC_FAST` when the application manages synchronization explicitly and needs to reduce ioctl synchronization overhead.

```C:uiomem-ioctl-test.c
    if ((fd = open("/dev/uiomem0", O_RDWR)) != -1) {
        uiomem_ioctl_sync_args sync_args = {0};
        uint64_t sync_offset    = 0;
        uint64_t sync_size      = 0x10000;
        int      sync_direction = 0;
        sync_args.offset = sync_offset;
        sync_args.size   = sync_size;
        SET_UIOMEM_IOCTL_FLAGS_SYNC_DIR(&sync_args, sync_direction);
        SET_UIOMEM_IOCTL_FLAGS_SYNC_FAST(&sync_args, 1);
        SET_UIOMEM_IOCTL_FLAGS_SYNC_CMD(&sync_args, UIOMEM_IOCTL_FLAGS_SYNC_CMD_FOR_CPU);
        status = ioctl(fd, UIOMEM_IOCTL_SET_SYNC, &sync_args);
        close(fd);
    }
```

# Example using uiomem

  * https://github.com/ikwzm/PLBRAM-Ultra96
  * https://github.com/ikwzm/PLBRAM-ZYBO-Z7

