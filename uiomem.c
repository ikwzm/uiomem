/*********************************************************************************
 *
 *       Copyright (C) 2015-2026 Ichiro Kawazome
 *       All rights reserved.
 * 
 *       Redistribution and use in source and binary forms, with or without
 *       modification, are permitted provided that the following conditions
 *       are met:
 * 
 *         1. Redistributions of source code must retain the above copyright
 *            notice, this list of conditions and the following disclaimer.
 * 
 *         2. Redistributions in binary form must reproduce the above copyright
 *            notice, this list of conditions and the following disclaimer in
 *            the documentation and/or other materials provided with the
 *            distribution.
 * 
 *       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *       A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 *       OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *       SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *       LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *       DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *       THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *       (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *       OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ********************************************************************************/
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysctl.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/scatterlist.h>
#include <linux/pagemap.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/version.h>
#include <asm/page.h>
#include <asm/byteorder.h>

/**
 * DOC: Uiomem Constants 
 */

MODULE_DESCRIPTION("User space mappable io-memory device driver");
MODULE_AUTHOR("ikwzm");
MODULE_LICENSE("Dual BSD/GPL");

#define DRIVER_VERSION     "1.1.0-beta.1"
#define DRIVER_NAME        "uiomem"
#define DEVICE_NAME_FORMAT "uiomem%d"
#define DEVICE_MAX_NUM      256
#define UIOMEM_DEBUG        1
#define IOCTL_VERSION       1

#if     (UIOMEM_DEBUG == 1)
#define UIOMEM_DEBUG_CHECK(this,debug) (this->debug)
#else
#define UIOMEM_DEBUG_CHECK(this,debug) (0)
#endif

#ifndef U64_MAX
#define U64_MAX ((u64)~0ULL)
#endif

/**
 * DOC: Uiomem Static Variables
 *
 * * uiomem_sys_class - uiomem system class
 * * init_enable      - uiomem install/uninstall infomation enable
 */

/**
 * uiomem_sys_class - uiomem system class
 */
static struct class*  uiomem_sys_class = NULL;

/**
 * info_enable module parameter
 */
static int        info_enable = 1;
module_param(     info_enable , int, S_IRUGO);
MODULE_PARM_DESC( info_enable , "uiomem install/uninstall infomation enable");

/**
 * DOC: Uiomem Object Structure
 *
 * This section defines the structure of uiomem device.
 *
 */

/**
 * struct uiomem_object - uiomem object structure.
 */
struct uiomem_object {
    struct device*       sys_dev;
    struct cdev          cdev;
    dev_t                device_number;
    struct mutex         sem;
    bool                 is_open;
    size_t               size;
    void*                virt_addr;
    phys_addr_t          phys_addr;
    int                  sync_mode;
    u64                  sync_offset;
    size_t               sync_size;
    int                  sync_direction;
    bool                 sync_owner;
    u64                  sync_for_cpu;
    u64                  sync_for_device;
    struct resource*     mem_region;
    bool                 cached;
    bool                 coherent;
    bool                 shareable;
};

/**
 * enum uiomem_direction - uiomem read/write direction
 */
enum uiomem_direction {
    UIOMEM_READ_WRITE = 0,
    UIOMEM_WRITE_ONLY = 1,
    UIOMEM_READ_ONLY  = 2,
    UIOMEM_NONE       = 3,
};
#define DIR_MAX UIOMEM_READ_ONLY

/**
 * sync_mode(synchronous mode) value
 */
#define SYNC_MODE_INVALID       (0x00)
#define SYNC_MODE_NONCACHED     (0x01)
#define SYNC_MODE_WRITECOMBINE  (0x02)
#define SYNC_MODE_DMACOHERENT   (0x03)
#define SYNC_MODE_MASK          (0x03)
#define SYNC_MODE_MIN           (0x01)
#define SYNC_MODE_MAX           (0x03)
#define SYNC_ALWAYS             (0x04)

/**
 * DOC: Data Cache Clean/Invalid Operations using PMEM API
 *
 * This section defines arch_sync_for_cpu() and arch_sync_for_dev().
 *
 * * arch_sync_for_cpu() - _uiomem_sync_for_cpu() using PMEM API
 * * arch_sync_for_dev() - _uiomem_sync_for_dev() using PMEM API
 */
#if (defined(CONFIG_ARCH_HAS_PMEM_API))
#ifndef UIOMEM_CACHE_SYNC_OPERATION
#define UIOMEM_CACHE_SYNC_OPERATION "PMEM API"
#include <linux/libnvdimm.h>
static void arch_sync_for_cpu(void* virt_start, phys_addr_t phys_start, size_t size, enum uiomem_direction direction)
{
    if (direction != UIOMEM_WRITE_ONLY)
        arch_invalidate_pmem(virt_start, size);
}
static void arch_sync_for_dev(void* virt_start, phys_addr_t phys_start, size_t size, enum uiomem_direction direction)
{
    if (direction == UIOMEM_READ_ONLY)
        arch_invalidate_pmem(virt_start, size);
    else
        arch_wb_cache_pmem(virt_start, size);
}
#endif /* #ifndef UIOMEM_CACHE_SYNC_OPERATION */
#endif /* #if (defined(CONFIG_ARCH_HAS_PMEM_API)) */

/**
 * DOC: Data Cache Clean/Invalid Operations for arm64 architecture.
 *
 * This section defines arch_sync_for_cpu() and arch_sync_for_dev().
 *
 * * arm64_read_dcache_line_size()     - read data cache line size of arm64.
 * * arm64_inval_dcache_area()         - invalid data cache.
 * * arm64_clean_dcache_area()         - clean(flush and invalidiate) data cache.
 * * arch_sync_for_cpu()               - _uiomem_sync_for_cpu() for arm64
 * * arch_sync_for_dev()               - _uiomem_sync_for_dev() for arm64
 */
#if (defined(CONFIG_ARM64))
#ifndef UIOMEM_CACHE_SYNC_OPERATION
#define UIOMEM_CACHE_SYNC_OPERATION "ARM64 Native"
static inline u64  arm64_read_dcache_line_size(void)
{
    u64       ctr;
    u64       dcache_line_size;
    const u64 bytes_per_word = 4;
    asm volatile ("mrs %0, ctr_el0" : "=r"(ctr) : : );
    asm volatile ("nop" : : : );
    dcache_line_size = (ctr >> 16) & 0xF;
    return (bytes_per_word << dcache_line_size);
}
static inline void arm64_inval_dcache_area(void* start, size_t size)
{
    u64   vaddr           = (u64)start;
    u64   __end           = (u64)start + size;
    u64   cache_line_size = arm64_read_dcache_line_size();
    u64   cache_line_mask = cache_line_size - 1;
    if ((__end & cache_line_mask) != 0) {
        __end &= ~cache_line_mask;
        asm volatile ("dc civac, %0" :  : "r"(__end) : );
    }
    if ((vaddr & cache_line_mask) != 0) {
        vaddr &= ~cache_line_mask;
        asm volatile ("dc civac, %0" :  : "r"(vaddr) : );
    }
    while (vaddr < __end) {
        asm volatile ("dc ivac, %0"  :  : "r"(vaddr) : );
        vaddr += cache_line_size;
    }
    asm volatile ("dsb	sy"  :  :  : );
}
static inline void arm64_clean_dcache_area(void* start, size_t size)
{
    u64   vaddr           = (u64)start;
    u64   __end           = (u64)start + size;
    u64   cache_line_size = arm64_read_dcache_line_size();
    u64   cache_line_mask = cache_line_size - 1;
    vaddr &= ~cache_line_mask;
    while (vaddr < __end) {
        asm volatile ("dc cvac, %0"  :  : "r"(vaddr) : );
        vaddr += cache_line_size;
    }
    asm volatile ("dsb	sy"  :  :  : );
}
static void arch_sync_for_cpu(void* virt_start, phys_addr_t phys_start, size_t size, enum uiomem_direction direction)
{
    if (direction != UIOMEM_WRITE_ONLY)
        arm64_inval_dcache_area(virt_start, size);
}
static void arch_sync_for_dev(void* virt_start, phys_addr_t phys_start, size_t size, enum uiomem_direction direction)
{
    if (direction == UIOMEM_READ_ONLY)
        arm64_inval_dcache_area(virt_start, size);
    else
        arm64_clean_dcache_area(virt_start, size);
}
#endif /* #ifndef UIOMEM_CACHE_SYNC_OPERATION */
#endif /* #if (defined(CONFIG_ARM64)) */

/**
 * DOC: Data Cache Clean/Invalid Operations for armv7 architecture.
 *
 * This section defines arch_sync_for_cpu() and arch_sync_for_dev().
 *
 * * armv7_read_dcache_line_size()     - read data cache line size of armv7.
 * * armv7_inval_dcache_area()         - invalid data cache.
 * * armv7_clean_dcache_area()         - clean(flush and invalidiate) data cache.
 * * arch_sync_for_cpu()               - _uiomem_sync_for_cpu() for armv7
 * * arch_sync_for_dev()               - _uiomem_sync_for_dev() for armv7
 */
#if (defined(CONFIG_ARM) && defined(CONFIG_CPU_V7))
#ifndef UIOMEM_CACHE_SYNC_OPERATION
#define UIOMEM_CACHE_SYNC_OPERATION "ARMV7 Native"
static inline u32  armv7_read_dcache_line_size(void)
{
    u32       ctr;
    u32       dcache_line_size;
    const u32 bytes_per_word = 4;
    asm volatile ("mrc	p15, 0, %0, c0, c0, 1": "=r"(ctr) : : );
    dcache_line_size = (ctr >> 16) & 0xF;
    return (bytes_per_word << dcache_line_size);
}
static inline void armv7_inval_dcache_area(void* start, size_t size)
{
    u32   vaddr           = (u32)start;
    u32   __end           = (u32)start + size;
    u32   cache_line_size = armv7_read_dcache_line_size();
    u32   cache_line_mask = cache_line_size - 1;
    if ((__end & cache_line_mask) != 0) {
        __end &= ~cache_line_mask;
        asm volatile ("mcr	p15, 0, %0, c7, c14, 1" :  : "r"(__end) : );
    }
    if ((vaddr & cache_line_mask) != 0) {
        vaddr &= ~cache_line_mask;
        asm volatile ("mcr	p15, 0, %0, c7, c14, 1" :  : "r"(vaddr) : );
    }
    while (vaddr < __end) {
        asm volatile ("mcr	p15, 0, %0, c7, c6,  1" :  : "r"(vaddr) : );
        vaddr += cache_line_size;
    }
    asm volatile ("dsb	st"  :  :  : );
}
static inline void armv7_clean_dcache_area(void* start, size_t size)
{
    u32   vaddr           = (u32)start;
    u32   __end           = (u32)start + size;
    u32   cache_line_size = armv7_read_dcache_line_size();
    u32   cache_line_mask = cache_line_size - 1;
    vaddr &= ~cache_line_mask;
    while (vaddr < __end) {
        asm volatile ("mcr	p15, 0, %0, c7, c10, 1"  :  : "r"(vaddr) : );
        vaddr += cache_line_size;
    }
    asm volatile ("dsb	st"  :  :  : );
}
static void arch_sync_for_cpu(void* virt_start, phys_addr_t phys_start, size_t size, enum uiomem_direction direction)
{
    if (direction != UIOMEM_WRITE_ONLY) {
        armv7_inval_dcache_area(virt_start, size);
        outer_inv_range(phys_start, phys_start + size);
    }
}
static void arch_sync_for_dev(void* virt_start, phys_addr_t phys_start, size_t size, enum uiomem_direction direction)
{
    if (direction == UIOMEM_READ_ONLY) {
        armv7_inval_dcache_area(virt_start, size);
        outer_inv_range(phys_start, phys_start + size);
    } else {
        armv7_clean_dcache_area(virt_start, size);
        outer_clean_range(phys_start, phys_start + size);
    }
}
#endif /* #ifndef UIOMEM_CACHE_SYNC_OPERATION */
#endif /* #if (defined(CONFIG_ARM) && defined(CONFIG_CPU_V7)) */

/**
 * DOC: Data Cache Clean/Invalid for architecuture independent.
 *
 * This section defines the following functions.
 *
 * * _uiomem_sync_for_cpu() - synchronous for cpu. 
 * * _uiomem_sync_for_dev() - synchronous for device.
 */
/**
 * _uiomem_sync_for_cpu() - call arch_sync_for_cpu().
 * @this:       Pointer to the uiomem object structure.
 * @virt_addr:  Virtua address.
 * @phys_addr:  Physical address.
 * @size:       Sync size.
 * @direction:  Sync direction.
 * Return:      Success(=0) or error status(<0).
 */
static inline void _uiomem_sync_for_cpu(
  struct uiomem_object*  this      ,
  void*                  virt_addr ,
  phys_addr_t            phys_addr ,
  size_t                 size      ,
  enum uiomem_direction  direction
) {
#ifdef UIOMEM_CACHE_SYNC_OPERATION
    arch_sync_for_cpu(virt_addr, phys_addr, size, direction);
#endif
}

/**
 * _uiomem_sync_for_dev() - call arch_sync_for_dev().
 * @this:       Pointer to the uiomem object structure.
 * @virt_addr:  Virtua address.
 * @phys_addr:  Physical address.
 * @size:       Sync size.
 * @direction:  Sync direction.
 * Return:      Success(=0) or error status(<0).
 */
static inline void _uiomem_sync_for_dev(
  struct uiomem_object*  this      ,
  void*                  virt_addr ,
  phys_addr_t            phys_addr ,
  size_t                 size      ,
  enum uiomem_direction  direction
) {
#ifdef UIOMEM_CACHE_SYNC_OPERATION
    arch_sync_for_dev(virt_addr, phys_addr, size, direction);
#endif
}

/**
 * DOC: Uiomem System Class Device File Description
 *
 * This section define the device file created in system class when uiomem is 
 * loaded into the kernel.
 *
 * The device file created in system class is as follows.
 *
 * * /sys/class/uiomem/<device-name>/driver_version
 * * /sys/class/uiomem/<device-name>/phys_addr
 * * /sys/class/uiomem/<device-name>/size
 * * /sys/class/uiomem/<device-name>/cached
 * * /sys/class/uiomem/<device-name>/coherent
 * * /sys/class/uiomem/<device-name>/shareable
 * * /sys/class/uiomem/<device-name>/sync_mode
 * * /sys/class/uiomem/<device-name>/sync_offset
 * * /sys/class/uiomem/<device-name>/sync_size
 * * /sys/class/uiomem/<device-name>/sync_direction
 * * /sys/class/uiomem/<device-name>/sync_owner
 * * /sys/class/uiomem/<device-name>/sync_for_cpu
 * * /sys/class/uiomem/<device-name>/sync_for_device
 * * /sys/class/uiomem/<device-name>/ioctl_version
 * * 
 */

#define  SYNC_COMMAND_DIR_MASK        (0x000000000000000C)
#define  SYNC_COMMAND_DIR_SHIFT       (2)
#define  SYNC_COMMAND_SIZE_MASK       (0x00000000FFFFFFF0)
#define  SYNC_COMMAND_SIZE_SHIFT      (0)
#define  SYNC_COMMAND_OFFSET_MASK     (0xFFFFFFFF00000000)
#define  SYNC_COMMAND_OFFSET_SHIFT    (32)
#define  SYNC_COMMAND_ARGMENT_MASK    (0xFFFFFFFFFFFFFFFE)
/**
 * uiomem_sync_command_argments() - get argment for _uiomem_sync_for_cpu() or _uiomem_sync_for_dev()
 *                                  
 * @this:       Pointer to the uiomem object structure.
 * @command:    sync command (this->sync_for_cpu or this->sync_for_device)
 * @phys_addr:  Pointer to the phys_addr for dma_sync_single_for_...()
 * @size:       Pointer to the size for dma_sync_single_for_...()
 * @direction:  Pointer to the direction for dma_sync_single_for_...()
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_sync_command_argments(
    struct uiomem_object *     this      ,
    u64                        command   ,
    void*                     *virt_addr ,
    phys_addr_t               *phys_addr ,
    size_t                    *size      ,
    enum uiomem_direction     *direction
) {
    u64    sync_offset   ;
    size_t sync_size     ;
    int    sync_direction;
    if ((command & SYNC_COMMAND_ARGMENT_MASK) != 0) {
        sync_offset    = (u64   )((command & SYNC_COMMAND_OFFSET_MASK) >> SYNC_COMMAND_OFFSET_SHIFT);
        sync_size      = (size_t)((command & SYNC_COMMAND_SIZE_MASK  ) >> SYNC_COMMAND_SIZE_SHIFT  );
        sync_direction = (int   )((command & SYNC_COMMAND_DIR_MASK   ) >> SYNC_COMMAND_DIR_SHIFT   );
    } else {
        sync_offset    = this->sync_offset;
        sync_size      = this->sync_size;
        sync_direction = this->sync_direction;
    }
    if (sync_offset + sync_size > this->size)
        return -EINVAL;
    switch(sync_direction) {
        case 1 : *direction = UIOMEM_WRITE_ONLY; break;
        case 2 : *direction = UIOMEM_READ_ONLY ; break;
        default: *direction = UIOMEM_READ_WRITE; break;
    }
    *virt_addr = this->virt_addr + sync_offset;
    *phys_addr = this->phys_addr + sync_offset;
    *size      = sync_size;
    return 0;
} 

/**
 * uiomem_sync_for_cpu() - call _uiomem_sync_for_cpu() when (sync_for_cpu != 0)
 * @this:       Pointer to the uiomem object structure.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_sync_for_cpu(struct uiomem_object* this)
{
    int status = 0;

    if (this->sync_for_cpu) {
        u64                     command = this->sync_for_cpu;
        void*                   virt_addr;
        phys_addr_t             phys_addr;
        size_t                  size;
        enum uiomem_direction   direction;
        status = uiomem_sync_command_argments(this, command, &virt_addr, &phys_addr, &size, &direction);
        if (status == 0) {
            if (this->coherent == false) {
                _uiomem_sync_for_cpu(this, virt_addr, phys_addr, size, direction);
            }
            this->sync_for_cpu = 0;
            this->sync_owner   = 0;
        }
    }
    return status;
}

/**
 * uiomem_sync_for_device() - call _uiomem_sync_for_dev() when (sync_for_device != 0)
 * @this:       Pointer to the uiomem object structure.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_sync_for_device(struct uiomem_object* this)
{
    int status = 0;

    if (this->sync_for_device) {
        u64                     command = this->sync_for_device;
        void*                   virt_addr;
        phys_addr_t             phys_addr;
        size_t                  size;
        enum uiomem_direction   direction;
        status = uiomem_sync_command_argments(this, command, &virt_addr, &phys_addr, &size, &direction);
        if (status == 0) {
            if (this->coherent == false) {
                _uiomem_sync_for_dev(this, virt_addr, phys_addr, size, direction);
            }
            this->sync_for_device = 0;
            this->sync_owner      = 1;
        }
    }
    return status;
}

#define DEF_ATTR_SHOW(__attr_name, __format, __value) \
static ssize_t uiomem_show_ ## __attr_name(struct device *dev, struct device_attribute *attr, char *buf) \
{                                                            \
    ssize_t status;                                          \
    struct uiomem_object* this = dev_get_drvdata(dev);       \
    if (mutex_lock_interruptible(&this->sem) != 0)           \
        return -ERESTARTSYS;                                 \
    status = sprintf(buf, __format, (__value));              \
    mutex_unlock(&this->sem);                                \
    return status;                                           \
}

static inline int NO_ACTION(struct uiomem_object* this){return 0;}

#define DEF_ATTR_SET(__attr_name, __min, __max, __pre_action, __post_action) \
static ssize_t uiomem_set_ ## __attr_name(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) \
{ \
    ssize_t       status; \
    u64           value;  \
    struct uiomem_object* this = dev_get_drvdata(dev);                       \
    if (0 != mutex_lock_interruptible(&this->sem)){return -ERESTARTSYS;}     \
    if (0 != (status = kstrtoull(buf, 0, &value))){            goto failed;} \
    if ((value < __min) || (__max < value)) {status = -EINVAL; goto failed;} \
    if (0 != (status = __pre_action(this)))       {            goto failed;} \
    this->__attr_name = value;                                               \
    if (0 != (status = __post_action(this)))      {            goto failed;} \
    status = size;                                                           \
  failed:                                                                    \
    mutex_unlock(&this->sem);                                                \
    return status;                                                           \
}

DEF_ATTR_SHOW(driver_version , "%s\n"    , DRIVER_VERSION                                 );
DEF_ATTR_SHOW(size           , "%zu\n"   , this->size                                     );
DEF_ATTR_SHOW(phys_addr      , "%pad\n"  , &this->phys_addr                               );
DEF_ATTR_SHOW(cached         , "%d\n"    , this->cached                                   );
DEF_ATTR_SHOW(coherent       , "%d\n"    , this->coherent                                 );
DEF_ATTR_SHOW(shareable      , "%d\n"    , this->shareable                                );
DEF_ATTR_SHOW(sync_mode      , "%d\n"    , this->sync_mode                                );
DEF_ATTR_SET( sync_mode                  , 0, 7,        NO_ACTION, NO_ACTION              );
DEF_ATTR_SHOW(sync_offset    , "0x%llx\n", this->sync_offset                              );
DEF_ATTR_SET( sync_offset                , 0, U64_MAX,  NO_ACTION, NO_ACTION              );
DEF_ATTR_SHOW(sync_size      , "%zu\n"   , this->sync_size                                );
DEF_ATTR_SET( sync_size                  , 0, SIZE_MAX, NO_ACTION, NO_ACTION              );
DEF_ATTR_SHOW(sync_direction , "%d\n"    , this->sync_direction                           );
DEF_ATTR_SET( sync_direction             , 0, DIR_MAX,  NO_ACTION, NO_ACTION              );
DEF_ATTR_SHOW(sync_owner     , "%d\n"    , this->sync_owner                               );
DEF_ATTR_SHOW(sync_for_cpu   , "%llu\n"  , this->sync_for_cpu                             );
DEF_ATTR_SET( sync_for_cpu               , 0, U64_MAX,  NO_ACTION, uiomem_sync_for_cpu    );
DEF_ATTR_SHOW(sync_for_device, "%llu\n"  , this->sync_for_device                          );
DEF_ATTR_SET( sync_for_device            , 0, U64_MAX,  NO_ACTION, uiomem_sync_for_device );
#ifdef UIOMEM_CACHE_SYNC_OPERATION
DEF_ATTR_SHOW(sync_operation , "%s\n"    , UIOMEM_CACHE_SYNC_OPERATION                    );
#else
DEF_ATTR_SHOW(sync_operation , "%s\n"    , "NONE"                                         );
#endif
#if (IOCTL_VERSION > 0)
DEF_ATTR_SHOW(ioctl_version  , "%d\n"    , (int)(IOCTL_VERSION)                           );
#endif

static struct device_attribute uiomem_device_attrs[] = {
  __ATTR(driver_version , 0444, uiomem_show_driver_version  , NULL                        ),
  __ATTR(size           , 0444, uiomem_show_size            , NULL                        ),
  __ATTR(phys_addr      , 0444, uiomem_show_phys_addr       , NULL                        ),
  __ATTR(cached         , 0444, uiomem_show_cached          , NULL                        ),
  __ATTR(coherent       , 0444, uiomem_show_coherent        , NULL                        ),
  __ATTR(shareable      , 0444, uiomem_show_shareable       , NULL                        ),
  __ATTR(sync_operation , 0444, uiomem_show_sync_operation  , NULL                        ),
  __ATTR(sync_mode      , 0664, uiomem_show_sync_mode       , uiomem_set_sync_mode        ),
  __ATTR(sync_offset    , 0664, uiomem_show_sync_offset     , uiomem_set_sync_offset      ),
  __ATTR(sync_size      , 0664, uiomem_show_sync_size       , uiomem_set_sync_size        ),
  __ATTR(sync_direction , 0664, uiomem_show_sync_direction  , uiomem_set_sync_direction   ),
  __ATTR(sync_owner     , 0444, uiomem_show_sync_owner      , NULL                        ),
  __ATTR(sync_for_cpu   , 0664, uiomem_show_sync_for_cpu    , uiomem_set_sync_for_cpu     ),
  __ATTR(sync_for_device, 0664, uiomem_show_sync_for_device , uiomem_set_sync_for_device  ),
#if (IOCTL_VERSION > 0)
  __ATTR(ioctl_version  , 0444, uiomem_show_ioctl_version   , NULL                        ),
#endif
  __ATTR_NULL,
};

#define uiomem_device_attrs_size (sizeof(uiomem_device_attrs)/sizeof(uiomem_device_attrs[0]))

static struct attribute* uiomem_attrs[uiomem_device_attrs_size] = {
  NULL
};
static struct attribute_group uiomem_attr_group = {
  .attrs = uiomem_attrs
};
static const struct attribute_group* uiomem_attr_groups[] = {
  &uiomem_attr_group,
  NULL
};

static inline void uiomem_sys_class_set_attributes(void)
{
    int i;
    for (i = 0 ; i < uiomem_device_attrs_size-1 ; i++) {
        uiomem_attrs[i] = &(uiomem_device_attrs[i].attr);
    }
    uiomem_attrs[i] = NULL;
    uiomem_sys_class->dev_groups = uiomem_attr_groups;
}

/**
 * DOC: Uiomem Device File Operations
 *
 * This section defines the operation of the uiomem device file.
 *
 * * uiomem_device_file_open()    - uiomem device file open operation.
 * * uiomem_device_file_release() - uiomem device file release operation.
 * * uiomem_device_file_mmap()    - uiomem device file memory map operation.
 * * uiomem_device_file_read()    - uiomem device file read operation.
 * * uiomem_device_file_write()   - uiomem device file write operation.
 * * uiomem_device_file_llseek()  - uiomem device file llseek operation.
 * * uiomem_device_file_ops       - uiomem device file operation table.
 */

/**
 * uiomem_device_file_open() - uiomem device file open operation.
 * @inode:      Pointer to the inode structure of this device.
 * @file:       to the file structure.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_device_file_open(struct inode *inode, struct file *file)
{
    struct uiomem_object* this;
    int status = 0;

    this = container_of(inode->i_cdev, struct uiomem_object, cdev);
    file->private_data = this;
    this->is_open = 1;

    return status;
}

/**
 * uiomem_device_file_release() - uiomem device file release operation.
 * @inode:      Pointer to the inode structure of this device.
 * @file:       Pointer to the file structure.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_device_file_release(struct inode *inode, struct file *file)
{
    struct uiomem_object* this = file->private_data;

    this->is_open = 0;

    return 0;
}

/**
 * _PGPROT_NONCACHED    : vm_page_prot value when ((sync_mode & SYNC_MODE_MASK) == SYNC_MODE_NONCACHED   )
 * _PGPROT_WRITECOMBINE : vm_page_prot value when ((sync_mode & SYNC_MODE_MASK) == SYNC_MODE_WRITECOMBINE)
 * _PGPROT_DMACOHERENT  : vm_page_prot value when ((sync_mode & SYNC_MODE_MASK) == SYNC_MODE_DMACOHERENT )
 */
#if     defined(CONFIG_ARM)
#define _PGPROT_NONCACHED(vm_page_prot)    pgprot_noncached(vm_page_prot)
#define _PGPROT_WRITECOMBINE(vm_page_prot) pgprot_writecombine(vm_page_prot)
#define _PGPROT_DMACOHERENT(vm_page_prot)  pgprot_dmacoherent(vm_page_prot)
#elif   defined(CONFIG_ARM64)
#define _PGPROT_NONCACHED(vm_page_prot)    pgprot_noncached(vm_page_prot)
#define _PGPROT_WRITECOMBINE(vm_page_prot) pgprot_writecombine(vm_page_prot)
#define _PGPROT_DMACOHERENT(vm_page_prot)  pgprot_writecombine(vm_page_prot)
#else
#define _PGPROT_NONCACHED(vm_page_prot)    pgprot_noncached(vm_page_prot)
#define _PGPROT_WRITECOMBINE(vm_page_prot) pgprot_writecombine(vm_page_prot)
#define _PGPROT_DMACOHERENT(vm_page_prot)  pgprot_writecombine(vm_page_prot)
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0))
static inline void vm_flags_set(struct vm_area_struct* vma, vm_flags_t flags)
{
    vma->vm_flags |=  (flags);
}
#endif

/**
 * uiomem_device_file_mmap() - uiomem device file memory map operation.
 * @file:       Pointer to the file structure.
 * @vma:        Pointer to the vm area structure.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_device_file_mmap(struct file *file, struct vm_area_struct* vma)
{
    struct uiomem_object* this = file->private_data;
    unsigned long         page_frame_num;
    unsigned long         map_area_size;

    if (vma->vm_pgoff + vma_pages(vma) > (this->size >> PAGE_SHIFT))
        return -ENXIO;

    if (((file->f_flags   & O_SYNC     ) != 0    ) ||
        ((this->sync_mode & SYNC_ALWAYS) != 0    ) ||
        ((this->cached                 ) == false)) {
        switch (this->sync_mode & SYNC_MODE_MASK) {
            case SYNC_MODE_NONCACHED :
                vma->vm_page_prot = _PGPROT_NONCACHED(vma->vm_page_prot);
                break;
            case SYNC_MODE_WRITECOMBINE :
                vma->vm_page_prot = _PGPROT_WRITECOMBINE(vma->vm_page_prot);
                break;
            case SYNC_MODE_DMACOHERENT :
                vma->vm_page_prot = _PGPROT_DMACOHERENT(vma->vm_page_prot);
                break;
            default :
                break;
        }
    }
    vm_flags_set(vma, (VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP));
    vma->vm_private_data = this;

    page_frame_num = (this->phys_addr >> PAGE_SHIFT) + vma->vm_pgoff;
    map_area_size  = vma_pages(vma) << PAGE_SHIFT;
    return remap_pfn_range(vma,
                           vma->vm_start,
                           page_frame_num,
                           map_area_size,
                           vma->vm_page_prot);
}

/**
 * uiomem_device_file_read() - uiomem device file read operation.
 * @file:       Pointer to the file structure.
 * @buff:       Pointer to the user buffer.
 * @count:      The number of bytes to be read.
 * @ppos:       Pointer to the offset value.
 * Return:      Transferd size.
 */
static ssize_t uiomem_device_file_read(struct file* file, char __user* buff, size_t count, loff_t* ppos)
{
    struct uiomem_object*  this      = file->private_data;
    int                    result    = 0;
    size_t                 xfer_size;
    size_t                 remain_size;
    phys_addr_t            phys_addr;
    void*                  virt_addr;

    if (mutex_lock_interruptible(&this->sem))
        return -ERESTARTSYS;

    if (*ppos >= this->size) {
        result = 0;
        goto return_unlock;
    }

    phys_addr = this->phys_addr + *ppos;
    virt_addr = this->virt_addr + *ppos;
    xfer_size = (*ppos + count >= this->size) ? this->size - *ppos : count;

    if (this->coherent == false) {
        _uiomem_sync_for_cpu(this, virt_addr, phys_addr, xfer_size, UIOMEM_READ_ONLY);
    }

    if ((remain_size = copy_to_user(buff, virt_addr, xfer_size)) != 0) {
        result = 0;
        goto return_unlock;
    }

    if (this->coherent == false) {
        _uiomem_sync_for_dev(this, virt_addr, phys_addr, xfer_size, UIOMEM_READ_ONLY);
    }

    *ppos += xfer_size;
    result = xfer_size;
 return_unlock:
    mutex_unlock(&this->sem);
    return result;
}

/**
 * uiomem_device_file_write() - uiomem device file write operation.
 * @file:       Pointer to the file structure.
 * @buff:       Pointer to the user buffer.
 * @count:      The number of bytes to be written.
 * @ppos:       Pointer to the offset value
 * Return:      Transferd size.
 */
static ssize_t uiomem_device_file_write(struct file* file, const char __user* buff, size_t count, loff_t* ppos)
{
    struct uiomem_object*  this      = file->private_data;
    int                    result    = 0;
    size_t                 xfer_size;
    size_t                 remain_size;
    phys_addr_t            phys_addr;
    void*                  virt_addr;

    if (mutex_lock_interruptible(&this->sem))
        return -ERESTARTSYS;

    if (*ppos >= this->size) {
        result = 0;
        goto return_unlock;
    }

    phys_addr = this->phys_addr + *ppos;
    virt_addr = this->virt_addr + *ppos;
    xfer_size = (*ppos + count >= this->size) ? this->size - *ppos : count;

    if (this->coherent == false) {
        _uiomem_sync_for_cpu(this, virt_addr, phys_addr, xfer_size, UIOMEM_WRITE_ONLY);
    }

    if ((remain_size = copy_from_user(virt_addr, buff, xfer_size)) != 0) {
        result = 0;
        goto return_unlock;
    }

    if (this->coherent == false) {
        _uiomem_sync_for_dev(this, virt_addr, phys_addr, xfer_size, UIOMEM_WRITE_ONLY);
    }

    *ppos += xfer_size;
    result = xfer_size;
 return_unlock:
    mutex_unlock(&this->sem);
    return result;
}

/**
 * uiomem_device_file_llseek() - uiomem device file llseek operation.
 * @file:       Pointer to the file structure.
 * @offset:     File offset to seek.
 * @whence:     Type of seek.
 * Return:      The new position.
 */
static loff_t uiomem_device_file_llseek(struct file* file, loff_t offset, int whence)
{
    struct uiomem_object*  this = file->private_data;
    loff_t                 new_pos;

    switch (whence) {
        case 0 : /* SEEK_SET */
            new_pos = offset;
            break;
        case 1 : /* SEEK_CUR */
            new_pos = file->f_pos + offset;
            break;
        case 2 : /* SEEK_END */
            new_pos = this->size  + offset;
            break;
        default:
            return -EINVAL;
    }
    if (new_pos < 0         ){return -EINVAL;}
    if (new_pos > this->size){return -EINVAL;}
    file->f_pos = new_pos;
    return new_pos;
}

/**
 * uiomem-ioctl.h - uiomem ioctl header file
 *
 * This source code(uiomem.c) has built-in header file(uiomem-ioctl.h) 
 * so that it can be built with only one source code.
 * To generate a header file (uiomem-ioctl.h) from this source code (uiomem.c), 
 * do the following
 * 
 * sed -n '/^\/\*\*\*\*\*\*\*\*\*\*\**$/,/\**\*\*\*\*\*\*\*\*\*\*\/$/p' uiomem.c >  uiomem-ioctl.h
 * sed -n '/^#ifndef.*UIOMEM_IOCTL_H/,/^#endif.*UIOMEM_IOCTL_H/p'       uiomem.c >> uiomem-ioctl.h
 * 
 */
#if (IOCTL_VERSION > 0)
#ifndef  UIOMEM_IOCTL_H
#define  UIOMEM_IOCTL_H
#include <linux/ioctl.h>

#define DEFINE_UIOMEM_IOCTL_FLAGS(name,type,lo,hi)                     \
static const  int      UIOMEM_IOCTL_FLAGS_ ## name ## _SHIFT = (lo);   \
static const  uint64_t UIOMEM_IOCTL_FLAGS_ ## name ## _MASK  = (((uint64_t)1UL << ((hi)-(lo)+1))-1); \
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

DEFINE_UIOMEM_IOCTL_FLAGS(IOCTL_VERSION, uiomem_ioctl_drv_info ,  0,  7)

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
#endif /* #if (IOCTL_VERSION > 0) */

/**
 * uiomem_device_file_ioctl() - uiomem device file ioctl operation.
 * @file:       Pointer to the file structure.
 * @cmd:        The ioctl command to be executed.
 * @arg:        Pointer to user space data associated with the ioctl command.
 * Return:      Success(=0) or error status(<0).
 */
#if (IOCTL_VERSION > 0)
static long uiomem_device_file_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    struct uiomem_object*  this   = file->private_data;
    void __user*           argp   = (void __user*)arg;
    int                    result = 0;

    switch(cmd) {
        case UIOMEM_IOCTL_GET_DRV_INFO: {
            uiomem_ioctl_drv_info drv_info = {0};
            SET_UIOMEM_IOCTL_FLAGS_IOCTL_VERSION(&drv_info, IOCTL_VERSION);
            if (strscpy(&drv_info.version[0], DRIVER_VERSION, sizeof(drv_info.version)) < 0) {
                result = -EFAULT;
                break;
            }
#ifdef UIOMEM_CACHE_SYNC_OPERATION
            if (strscpy(&drv_info.sync_operation[0], UIOMEM_CACHE_SYNC_OPERATION, sizeof(drv_info.sync_operation)) < 0) {
                result = -EFAULT;
                break;
            }
#endif
            if (copy_to_user(argp, &drv_info, sizeof(drv_info)) != 0)
                result = -EFAULT;
            else 
                result = 0;
            break;
        }
        case UIOMEM_IOCTL_GET_SIZE: {
            uint64_t size = (uint64_t)this->size;
            if (copy_to_user(argp, &size, sizeof(size)) != 0)
                result = -EFAULT;
            else 
                result = 0;
            break;
        }
        case UIOMEM_IOCTL_GET_PHYS_ADDR: {
            uint64_t phys_addr = (uint64_t)this->phys_addr;
            if (copy_to_user(argp, &phys_addr, sizeof(phys_addr)) != 0)
                result = -EFAULT;
            else 
                result = 0;
            break;
        }
        case UIOMEM_IOCTL_GET_SYNC_OWNER: {
            uint32_t sync_owner = (uint32_t)this->sync_owner;
            if (copy_to_user(argp, &sync_owner, sizeof(sync_owner)) != 0)
                result = -EFAULT;
            else 
                result = 0;
            break;
        }
        case UIOMEM_IOCTL_GET_DEV_INFO: {
            uiomem_ioctl_dev_info dev_info = {0};
            SET_UIOMEM_IOCTL_FLAGS_SHAREABLE(&dev_info, this->shareable);
            SET_UIOMEM_IOCTL_FLAGS_CACHED   (&dev_info, this->cached   );
            SET_UIOMEM_IOCTL_FLAGS_COHERENT (&dev_info, this->coherent );
            dev_info.size = (uint64_t)(this->size);
            dev_info.addr = (uint64_t)(this->phys_addr);
            if (copy_to_user(argp, &dev_info, sizeof(dev_info)) != 0)
                result = -EFAULT;
            else 
                result = 0;
            break;
        }
        case UIOMEM_IOCTL_GET_SYNC: {
            uiomem_ioctl_sync_args sync_args = {0};
            SET_UIOMEM_IOCTL_FLAGS_SYNC_DIR  (&sync_args, this->sync_direction);
            SET_UIOMEM_IOCTL_FLAGS_SYNC_MODE (&sync_args, this->sync_mode);
            SET_UIOMEM_IOCTL_FLAGS_SYNC_OWNER(&sync_args, this->sync_owner);
            sync_args.size   = (uint64_t)this->sync_size;
            sync_args.offset = (uint64_t)this->sync_offset;
            if (copy_to_user(argp, &sync_args, sizeof(sync_args)) != 0)
                result = -EFAULT;
            else 
                result = 0;
            break;
        }
        case UIOMEM_IOCTL_SET_SYNC: {
            uiomem_ioctl_sync_args sync_args;
            if (copy_from_user(&sync_args, argp, sizeof(sync_args)) != 0)
                result = -EFAULT;
            else {
                int    sync_command   = GET_UIOMEM_IOCTL_FLAGS_SYNC_CMD (&sync_args);
                int    sync_direction = GET_UIOMEM_IOCTL_FLAGS_SYNC_DIR (&sync_args);
                int    sync_mode      = GET_UIOMEM_IOCTL_FLAGS_SYNC_MODE(&sync_args);
                u64    sync_offset    = (u64)(sync_args.offset);
                size_t sync_size      = (size_t)(sync_args.size);
                switch(sync_direction) {
                    case 0   : this->sync_direction = 0; break;
                    case 1   : this->sync_direction = 1; break;
                    case 2   : this->sync_direction = 2; break;
                    default  : /* none */                break;
                }
                if (sync_mode   >  0) {this->sync_mode   = sync_mode  ;}
                if (sync_offset >= 0) {this->sync_offset = sync_offset;}
                if (sync_size   >  0) {this->sync_size   = sync_size  ;}
                switch(sync_command) {
                    case UIOMEM_IOCTL_FLAGS_SYNC_CMD_FOR_CPU:
                        this->sync_for_cpu = 1;
                        result = uiomem_sync_for_cpu(this);
                        break;
                    case UIOMEM_IOCTL_FLAGS_SYNC_CMD_FOR_DEVICE:
                        this->sync_for_device = 1;
                        result = uiomem_sync_for_device(this);
                        break;
                    default  :
                        result = 0;
                        break;
                }
            }
            break;
        }
        case UIOMEM_IOCTL_SET_SYNC_FOR_CPU: {
            u64 sync_args;
            if (copy_from_user(&sync_args, argp, sizeof(sync_args)) != 0)
                result = -EFAULT;
            else {
                this->sync_for_cpu = sync_args;
                result = uiomem_sync_for_cpu(this);
            }
            break;
        }
        case UIOMEM_IOCTL_SET_SYNC_FOR_DEVICE: {
            u64 sync_args;
            if (copy_from_user(&sync_args, argp, sizeof(sync_args)) != 0)
                result = -EFAULT;
            else {
                this->sync_for_device = sync_args;
                result = uiomem_sync_for_device(this);
            }
            break;
        }
        default:
            result = -ENOTTY;
    }
    return (long)result;
}

#if defined(CONFIG_COMPAT) && (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 4))
static long compat_ptr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	if (!file->f_op->unlocked_ioctl)
		return -ENOIOCTLCMD;

	return file->f_op->unlocked_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

#endif /* #if (IOCTL_VERSION > 0) */

/**
 * uiomem device file operation table.
 */
static const struct file_operations uiomem_device_file_ops = {
    .owner          = THIS_MODULE,
    .open           = uiomem_device_file_open,
    .release        = uiomem_device_file_release,
    .mmap           = uiomem_device_file_mmap,
    .read           = uiomem_device_file_read,
    .write          = uiomem_device_file_write,
    .llseek         = uiomem_device_file_llseek,
#if (IOCTL_VERSION > 0)
    .unlocked_ioctl = uiomem_device_file_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = compat_ptr_ioctl,
#endif
#endif
};

/**
 * DOC: Uiomem Object Operations
 *
 * This section defines the operation of uiomem object.
 *
 * * uiomem_device_ida         - Uiomem Object Device Minor Number allocator variable.
 * * uiomem_device_number      - Uiomem Object Device Major Number.
 * * uiomem_object_create()    - Create uiomem object.
 * * uiomem_object_setup()     - Setup the uiomem object.
 * * uiomem_object_info()      - Print infomation the uiomem object.
 * * uiomem_object_destroy()   - Destroy the uiomem object.
 * * uiomem_device_remove()    - Remove uiomem object from device driver.
 */
static DEFINE_IDA(uiomem_device_ida);
static dev_t      uiomem_device_number = 0;

/**
 * uiomem_object_create() -  Create uiomem object.
 * @name:       device name   or NULL.
 * @parent:     parent device or NULL.
 * @minor:      minor_number  or -1 or -2.
 * Return:      Pointer to the uiomem object or NULL.
 */
static struct uiomem_object* uiomem_object_create(const char* name, struct device* parent, int minor)
{
    struct uiomem_object* this     = NULL;
    unsigned int               done     = 0;
    const unsigned int         DONE_ALLOC_MINOR   = (1 << 0);
    const unsigned int         DONE_CHRDEV_ADD    = (1 << 1);
    const unsigned int         DONE_DEVICE_CREATE = (1 << 3);
    /*
     * allocate device minor number
     */
    {
        if ((0 <= minor) && (minor < DEVICE_MAX_NUM)) {
            if (ida_simple_get(&uiomem_device_ida, minor, minor+1, GFP_KERNEL) < 0) {
                printk(KERN_ERR "couldn't allocate minor number(=%d).\n", minor);
                goto failed;
            }
        } else if(minor < 0) {
            if ((minor = ida_simple_get(&uiomem_device_ida, 0, DEVICE_MAX_NUM, GFP_KERNEL)) < 0) {
                printk(KERN_ERR "couldn't allocate new minor number. return=%d.\n", minor);
                goto failed;
            }
        } else {
                printk(KERN_ERR "invalid minor number(=%d), valid range is 0 to %d\n", minor, DEVICE_MAX_NUM-1);
                goto failed;
        }
        done |= DONE_ALLOC_MINOR;
    }
    /*
     * create (uiomem_object*) this.
     */
    {
        this = kzalloc(sizeof(*this), GFP_KERNEL);
        if (IS_ERR_OR_NULL(this)) {
            int retval = PTR_ERR(this);
            this = NULL;
            printk(KERN_ERR "kzalloc() failed. return=%d\n", retval);
            goto failed;
        }
    }
    /*
     * set device_number
     */
    {
        this->device_number = MKDEV(MAJOR(uiomem_device_number), minor);
    }
    /*
     * register /sys/class/uiomem/<name>
     */
    {
        if (name == NULL) {
            this->sys_dev = device_create(uiomem_sys_class,
                                          parent,
                                          this->device_number,
                                          (void *)this,
                                          DEVICE_NAME_FORMAT, MINOR(this->device_number));
        } else {
            this->sys_dev = device_create(uiomem_sys_class,
                                          parent,
                                          this->device_number,
                                          (void *)this,
                                         "%s", name);
        }
        if (IS_ERR_OR_NULL(this->sys_dev)) {
            int retval = PTR_ERR(this->sys_dev);
            this->sys_dev = NULL;
            printk(KERN_ERR "device_create() failed. return=%d\n", retval);
            goto failed;
        }
        done |= DONE_DEVICE_CREATE;
    }
    /*
     * add chrdev.
     */
    {
        int retval;
        cdev_init(&this->cdev, &uiomem_device_file_ops);
        this->cdev.owner = THIS_MODULE;
        if ((retval = cdev_add(&this->cdev, this->device_number, 1)) != 0) {
            printk(KERN_ERR "cdev_add() failed. return=%d\n", retval);
            goto failed;
        }
        done |= DONE_CHRDEV_ADD;
    }
    /*
     * initialize other variables.
     */
    {
        this->size            = 0;
        this->sync_mode       = SYNC_MODE_NONCACHED;
        this->sync_offset     = 0;
        this->sync_size       = 0;
        this->sync_direction  = 0;
        this->sync_owner      = 0;
        this->sync_for_cpu    = 0;
        this->sync_for_device = 0;
        this->mem_region      = NULL;
    }
    mutex_init(&this->sem);

    return this;

 failed:
    if (done & DONE_CHRDEV_ADD   ) { cdev_del(&this->cdev); }
    if (done & DONE_DEVICE_CREATE) { device_destroy(uiomem_sys_class, this->device_number);}
    if (done & DONE_ALLOC_MINOR  ) { ida_simple_remove(&uiomem_device_ida, minor);}
    if (this != NULL)              { kfree(this); }
    return NULL;
}

/**
 * uiomem_object_setup() - Setup the uiomem object.
 * @this:       Pointer to the uiomem object.
 * @phys_addr:  Physical address.
 * @size:       size.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_object_setup(struct uiomem_object* this, phys_addr_t phys_addr, size_t size)
{
    if (!this)
        return -ENODEV;
    /*
     * setup phys_addr, size
     */
    this->phys_addr = phys_addr;
    this->size      = size;
    /*
     * setup virtual address
     */
    this->virt_addr = memremap(this->phys_addr,
                               this->size,
                               (this->cached == true)?  MEMREMAP_WB  :  MEMREMAP_WC );
    if (IS_ERR_OR_NULL(this->virt_addr)) {
        int retval = PTR_ERR(this->virt_addr);
        dev_err(this->sys_dev, "memremap(addr=%pad,size=%zu,%s) failed. return(%d)\n",
                               &this->phys_addr,
                               this->size,
                               (this->cached == true)? "MEMREMAP_WB" : "MEMREMAP_WC",
                               retval);
        this->virt_addr = NULL;
        return (retval == 0) ? -ENOMEM : retval;
    }
    return 0;
}

/**
 * uiomem_object_info() - Print infomation the uiomem object structure.
 * @this:       Pointer to the uiomem object structure.
 */
static void uiomem_object_info(struct uiomem_object* this)
{
    dev_info(this->sys_dev, "driver version = %s\n"  , DRIVER_VERSION);
#if (IOCTL_VERSION > 0)
    dev_info(this->sys_dev, "ioctl version  = %d\n"  , IOCTL_VERSION);
#endif
    dev_info(this->sys_dev, "major number   = %d\n"  , MAJOR(this->device_number));
    dev_info(this->sys_dev, "minor number   = %d\n"  , MINOR(this->device_number));
    dev_info(this->sys_dev, "range address  = %pad\n", &this->phys_addr);
    dev_info(this->sys_dev, "range size     = %zu\n" , this->size);
    dev_info(this->sys_dev, "cached         = %d\n"  , this->cached);
    dev_info(this->sys_dev, "coherent       = %d\n"  , this->coherent);
#ifdef UIOMEM_CACHE_SYNC_OPERATION
    dev_info(this->sys_dev, "sync_operation = %s\n"  , UIOMEM_CACHE_SYNC_OPERATION);
#else
    dev_info(this->sys_dev, "sync_operation = %s\n"  , "NONE");
#endif
    dev_info(this->sys_dev, "shareable      = %d\n"  , this->shareable);
}

/**
 * uiomem_object_destroy() -  Destroy the uiomem object.
 * @this:       Pointer to the uiomem object.
 * Return:      Success(=0) or error status(<0).
 *
 * Unregister the device after releasing the resources.
 */
static int uiomem_object_destroy(struct uiomem_object* this)
{
    if (!this)
        return -ENODEV;

    if (this->virt_addr != NULL) {
        memunmap(this->virt_addr);
        this->virt_addr = NULL;
    }
    if (this->mem_region != NULL) {
        release_mem_region(this->mem_region->start, resource_size(this->mem_region));
    }
    cdev_del(&this->cdev);
    device_destroy(uiomem_sys_class, this->device_number);
    ida_simple_remove(&uiomem_device_ida, MINOR(this->device_number));
    kfree(this);
    return 0;
}

/**
 * uiomem_device_remove()   - Remove uiomem object from device driver.
 * @dev:        handle to the device structure.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_device_remove(struct device *dev)
{
    struct uiomem_object* this   = dev_get_drvdata(dev);
    int                   retval = 0;

    if (this != NULL) {
        retval = uiomem_object_destroy(this);
        dev_set_drvdata(dev, NULL);
    } else {
        retval = -ENODEV;
    }
    return retval;
}

/**
 * DOC: Uiomem Device List section.
 *
 * This section defines the uiomem platform device list.
 *
 * * struct uiomem_device_entry          - uiomem device entry structure.
 * * uiomem_device_list                  - list of uiomem device entry structure.
 * * uiomem_device_list_sem              - semaphore of uiomem device entry list.
 * * uiomem_device_list_create_entry()   - Create uiomem device entry and add to list.
 * * uiomem_device_list_delete_entry()   - Delete uiomem device entry from list.
 * * uiomem_device_list_remove_entry()   - Remove uiomem device entry from list with remove function
 * * uiomem_device_list_cleanup()        - Remove all uiomem device entry from list.
 * * uiomem_device_list_search()         - Search uiomem device entry from list by name or number.
 * * uiomem_get_device_name_property()   - Get "device-name"  property from uiomem device entry.
 * * uiomem_get_minor_number_property()  - Get "minor-number" property from uiomem device entry.
 * * uiomem_get_addr_property()          - Get "addr"         property from uiomem device entry.
 * * uiomem_get_size_property()          - Get "size"         property from uiomem device entry.
 * * uiomem_get_option_property()        - Get "option"       property from uiomem device entry.
 */
#include <linux/property.h>

/**
 * struct uiomem_device_entry - uiomem platform device structure.
 */
struct uiomem_device_entry {
    struct device*       dev;
    struct device*       parent;
    void                 (*prep_remove)(struct device* dev);
    void                 (*post_remove)(struct device* dev);
    struct list_head     list;
};

/**
 * uiomem_device_list        - list of uiomem device entry structure.
 * uiomem_device_list_sem    - semaphore of uiomem platform device list.
 */
static struct list_head uiomem_device_list;
static struct mutex     uiomem_device_list_sem;

/**
 * uiomem_get_device_name_property()  - Get "device-name"  property from uiomem device.
 * @dev:        handle to the device structure.
 * @name:       address of device name.
 * Return:      Success(=0) or error status(<0).
 */
static inline int uiomem_get_device_name_property(struct device *dev, const char** name)
{
    return device_property_read_string(dev, "device-name", name);
}

/**
 * uiomem_get_minor_number_property() - Get "minor-number" property from uiomem device.
 * @dev:        handle to the device structure.
 * @value:      address of minor number value.
 * Return:      Success(=0) or error status(<0).
 */
static inline int uiomem_get_minor_number_property(struct device *dev, u32* value)
{
    return device_property_read_u32(dev, "minor-number", value);
}

/**
 * * uiomem_get_addr_property()          - Get "addr" property from uiomem device entry.
 * @dev:        handle to the device structure.
 * @value:      address of iomem size value.
 * Return:      Success(=0) or error status(<0).
 */
static inline int uiomem_get_addr_property(struct device *dev, u64* value)
{
    return device_property_read_u64(dev, "addr", value);
}

/**
 * * uiomem_get_size_property()          - Get "size" property from uiomem device entry.
 * @dev:        handle to the device structure.
 * @value:      address of iomem size value.
 * Return:      Success(=0) or error status(<0).
 */
static inline int uiomem_get_size_property(struct device *dev, u64* value)
{
    return device_property_read_u64(dev, "size", value);
}

/**
 * uiomem_get_option_property() - Get "option" property from uiomem device.
 * @dev:        handle to the device structure.
 * @value:      address of option value.
 * Return:      Success(=0) or error status(<0).
 */
static inline int uiomem_get_option_property(struct device *dev, u64* value)
{
    return device_property_read_u64(dev, "option", value);
}
/**
 * uiomem_get_option_shareable()   - Get shareable property from option[0:0].
 * uiomem_get_option_cached()      - Get cached    property from option[1:1].
 * uiomem_get_option_coherent()    - Get coherent  property from option[2:2].
 * @option:     option.
 */
#define DEFINE_UIOMEM_OPTION(name,type,lo,hi)             \
static inline type uiomem_get_option_ ## name(u64 option) \
{                                                         \
    const u64 mask = ((1UL << ((hi)-(lo)+1))-1);          \
    return (type)((option >> (lo)) & mask);               \
}
DEFINE_UIOMEM_OPTION(shareable, bool, 0, 0)
DEFINE_UIOMEM_OPTION(cached   , bool, 1, 1)
DEFINE_UIOMEM_OPTION(coherent , bool, 2, 2)

#define UIOMEM_STATIC_DEVICE_OPTION_DEFAULT (2) /* coherent=false, cache=true, shareable=false */
#define UIOMEM_STATIC_DEVICE_OPTION_DESC " coherent=option[2],cache=option[1],shareable=option[0]"

/**
 * uiomem_device_list_search()    - Search uiomem device entry from list by name or number.
 * @dev:        handle to the device structure or NULL.
 * @name:       device name or NULL.
 * @id:         device id or negative integer.
 * Return:      Pointer to the found udmabuf device entry or NULL.
 */
static struct uiomem_device_entry* uiomem_device_list_search(struct device *dev, const char* name, int id)
{
    struct uiomem_device_entry* entry;
    struct uiomem_device_entry* found_entry = NULL;
    mutex_lock(&uiomem_device_list_sem);
    list_for_each_entry(entry, &uiomem_device_list, list) {
        bool found_by_dev  = true;
        bool found_by_name = true;
        bool found_by_id   = true;
        if (dev != NULL) {
            found_by_dev = false;
            if (dev == entry->dev)
                found_by_dev = true;
        }
        if (name != NULL) {
            const char* device_name;
            found_by_name = false;
            if (uiomem_get_device_name_property(entry->dev, &device_name) == 0) 
                if (strcmp(name, device_name) == 0)
                    found_by_name = true;
        }
        if (id >= 0) {
            u32 minor_number;
            found_by_id = false;
            if (uiomem_get_minor_number_property(entry->dev, &minor_number) == 0) 
                if (id == minor_number)
                    found_by_id = true;
        }
        if ((found_by_dev == true) && (found_by_name == true) && (found_by_id == true))
            found_entry = entry;
    }
    mutex_unlock(&uiomem_device_list_sem);
    return found_entry;
}

/**
 * uiomem_device_list_create_entry() - Create uiomem device entry and add to list.
 * @dev:        handle to the device structure.
 * @parent:     handle to the parent device structure
 *              If the entry is successfully created, it is get_device(parent)
 * @name:       device name or NULL.
 * @id:         device id or negative integer.
 * @addr:       iomem addresss.
 * @size:       iomem size.
 * @option      option.
 * @prep_remove prepare function when remove entry from udmabuf device list or NULL.
 * @post_remove post function when remove entry from udmabuf device list or NULL.
 * Return:      pointer to the udmabuf device entry or NULL.
 */
static struct uiomem_device_entry* uiomem_device_list_create_entry(struct device *dev, struct device *parent, const char* name, int id, u64 addr, u64 size, u64 option, void (*prep_remove)(struct device*), void (*post_remove)(struct device*))
{                              
    struct uiomem_device_entry* exist_entry;
    struct uiomem_device_entry* entry  = NULL;
    int                         retval = 0;
    
    exist_entry = uiomem_device_list_search(NULL, name, id);
    if (!IS_ERR_OR_NULL(exist_entry)) {
        pr_err(DRIVER_NAME ": device name(%s) or id(%d) is already exists\n", (name)?name:"NULL", id);
        retval = -EINVAL;
        goto failed;
    }

    entry = kzalloc(sizeof(*entry), GFP_KERNEL);
    if (IS_ERR_OR_NULL(entry)) {
        retval = PTR_ERR(entry);
        entry  = NULL;
        pr_err(DRIVER_NAME ": kzalloc() failed. return=%d\n", retval);
        goto failed;
    }

    {
        struct property_entry   props_list[] = {
            PROPERTY_ENTRY_STRING("device-name" , name  ),
            PROPERTY_ENTRY_U64(   "addr"        , addr  ),
            PROPERTY_ENTRY_U64(   "size"        , size  ),
            PROPERTY_ENTRY_U32(   "minor-number", id    ),
            PROPERTY_ENTRY_U64(   "option"      , option),
            {},
        };
        struct property_entry* props = (name != NULL) ? &props_list[0] : &props_list[1];
#if     (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
        {
            retval = device_create_managed_software_node(dev, props, NULL);
            if (retval != 0) {
                pr_err(DRIVER_NAME ": device_create_managed_software_node failed. return=%d\n", retval);
                goto failed;
            }
        }
#elif   (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
        {
            retval = device_add_properties(dev, props);
            if (retval != 0) {
                pr_err(DRIVER_NAME ": device_add_properties failed. return=%d\n", retval);
                goto failed;
            }
        }
#else
        {
            const struct property_set pset = {
                .properties = props,
            };
            retval = device_add_property_set(dev, &pset);
            if (retval != 0) {
                pr_err(DRIVER_NAME ": device_add_propertiy_set failed. return=%d\n", retval);
                goto failed;
            }
        }
#endif
    }
    
    entry->dev = dev;
    entry->parent = parent;
    entry->prep_remove = prep_remove;
    entry->post_remove = post_remove;
    
    mutex_lock(&uiomem_device_list_sem);
    list_add_tail(&entry->list, &uiomem_device_list);
    mutex_unlock(&uiomem_device_list_sem);

    return entry;

 failed:
    if (entry != NULL) {
        kfree(entry);
    }
    return ERR_PTR(retval);
}
    
/**
 * uiomem_device_list_delete_entry() - Delete uiomem device entry from list.
 * @entry:      Pointer to the udmabuf device entry.
 */
static void uiomem_device_list_delete_entry(struct uiomem_device_entry* entry)
{
    mutex_lock(&uiomem_device_list_sem);
    list_del(&entry->list);
    mutex_unlock(&uiomem_device_list_sem);
    kfree(entry);
}

/**
 * uiomem_device_list_remove_entry() - Remove uiomem device entry from list with remove functions.
 * @entry:      Pointer to the udmabuf device entry.
 */
static void uiomem_device_list_remove_entry(struct uiomem_device_entry* entry)
{
    struct device* dev    = entry->dev;
    struct device* parent = entry->parent;
    void (*prep_remove)(struct device* dev) = entry->prep_remove;
    void (*post_remove)(struct device* dev) = entry->post_remove;
    if (prep_remove)
        prep_remove(dev);
    uiomem_device_list_delete_entry(entry);
    if (post_remove)
        post_remove(dev);
    if (parent)
        put_device(parent);
}

/**
 * uiomem_device_list_cleanup() - Remove all uiomem device entry from list.
 */
static void uiomem_device_list_cleanup(void)
{
    struct uiomem_device_entry* entry;
    while(!list_empty(&uiomem_device_list)) {
        entry = list_first_entry(&uiomem_device_list, typeof(*(entry)), list);
        uiomem_device_list_remove_entry(entry);
    }
}

/**
 * DOC: Uiomem Platform Device section.
 *
 * This section defines the udmabuf platform device.
 *
 * * uiomem_platform_device_create() - Create uiomem platform device and add to device list.
 * * uiomem_platform_device_del()    - Delete uiomem platform device before remove from device list.
 * * uiomem_platform_device_put()    - Put uiomem platform device after remove from device list.
 * * uiomem_platform_device_probe()  - Probe  call for the platform device driver.
 * * uiomem_platform_device_remove() - Remove call for the platform device driver.
 */

/**
 * uiomem_platform_device_del() - Delete uiomem platform device before remove from device list.
 * @dev:        handle to the device structure.
 */
static void uiomem_platform_device_del(struct device* dev)
{
    /*
     * platform_device_del() calls udmabuf_platform_driver_remove()
     */
    platform_device_del(to_platform_device(dev));
}

/**
 * uiomem_platform_device_put() - Put uiomem platform device after remove from device list.
 * @dev:        handle to the device structure.
 */
static void uiomem_platform_device_put(struct device* dev)
{
    platform_device_put(to_platform_device(dev));
}

/**
 * uiomem_platform_device_create() - Create uiomem platform device and add to device list.
 * @name:       device name or NULL.
 * @id:         device id.
 * @addr:       iomem address.
 * @size:       iomem size.
 * @option      option.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_platform_device_create(const char* name, int id, u64 addr, u64 size, u64 option)
{
    struct platform_device*     pdev   = NULL;
    struct uiomem_device_entry* entry  = NULL;
    int                         retval = 0;

    if (addr == 0)
        return -EINVAL;

    if (size == 0)
        return -EINVAL;

    pdev = platform_device_alloc(DRIVER_NAME, id);
    if (IS_ERR_OR_NULL(pdev)) {
        retval = PTR_ERR(pdev);
        pdev   = NULL;
        pr_err(DRIVER_NAME ": platform_device_alloc(%s,%d) failed. return=%d\n", DRIVER_NAME, id, retval);
        goto failed;
    }

    entry = uiomem_device_list_create_entry(&pdev->dev,
                                            NULL,
                                            name,
                                            id,
                                            addr,
                                            size,
                                            option,
                                            uiomem_platform_device_del,
                                            uiomem_platform_device_put);
    if (IS_ERR_OR_NULL(entry)) {
        retval = PTR_ERR(entry);
        entry  = NULL;
        pr_err(DRIVER_NAME ": device create entry failed. return=%d\n", retval);
        goto failed;
    }

    {
        struct resource resource_list[] = {DEFINE_RES_MEM(addr,size)};
        retval = platform_device_add_resources(pdev, resource_list, ARRAY_SIZE(resource_list));
        if (retval != 0) {
            pr_err(DRIVER_NAME ": platform_device_add_resources failed. return=%d\n", retval);
            goto failed;
        }
    }

    /*
     * platform_device_add() calls uiomem_platform_driver_probe()
     */
    retval = platform_device_add(pdev);
    if (retval != 0) {
        pr_err(DRIVER_NAME ": platform_device_add failed. return=%d\n", retval);
        goto failed;
    }

    if (dev_get_drvdata(&pdev->dev) == NULL) {
        pr_err(DRIVER_NAME ": object of %s is none.", dev_name(&pdev->dev));
        platform_device_del(pdev);
        retval = -ENODEV;
        goto failed;
    }
    
    return 0;

 failed:
    if (entry != NULL) {
        uiomem_device_list_delete_entry(entry);
    }
    if (pdev  != NULL) {
        platform_device_put(pdev);
    }
    return retval;
}

/**
 * of_property_read_ulong() -  Find and read a unsigned long intger from a property.
 * @node:       device node which the property value is to be read.
 * @propname:   name of property to be searched.
 * @out_value:  pointer to return value, modified only if return value is 0.
 * Return:      Success(=0) or error status(<0).
 */
static int of_property_read_ulong(const struct device_node* node, const char* propname, u64* out_value)
{
    u32    u32_value;
    u64    u64_value;
    int    retval;

    if ((retval = of_property_read_u64(node, propname, &u64_value)) == 0) {
        *out_value = u64_value;
        return 0;
    }
      
    if ((retval = of_property_read_u32(node, propname, &u32_value)) == 0) {
        *out_value = (u64)u32_value;
        return 0;
    }
      
    return retval;
}

/**
 * uiomem_platform_device_remove() - Remove call for the uiomem platform device.
 * @dev:        handle to the device structure.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_platform_device_remove(struct device *dev)
{
    return uiomem_device_remove(dev);
}

/**
 * uiomem_platform_device_probe() -  Probe call for the device driver.
 * @dev:        handle to the device structure.
 * @res:        handle to the resource structure or NULL.
 * Return:      Success(=0) or error status(<0).
 *
 * It does all the memory allocation and registration for the device.
 */
static int uiomem_platform_device_probe(struct device *dev, struct resource* res)
{
    int                    retval       = 0;
    u32                    u32_value    = 0;
    u64                    u64_value    = 0;
    int                    minor_number = -1;
    phys_addr_t            mem_addr;
    size_t                 mem_size;
    struct uiomem_object*  obj          = NULL;
    const char*            device_name  = NULL;

    /*
     * check handle to the resource structure.
     */
    if (res == NULL) {
        struct device_node* np = of_parse_phandle(dev->of_node, "memory-region", 0);
        if (np == NULL) {
            dev_err(dev, "can not found resource or memory region\n");
            retval = -EINVAL;
            goto failed;
        } else {
            struct reserved_mem* rmem = of_reserved_mem_lookup(np);
            of_node_put(np);
            if (rmem == NULL) {
                dev_err(dev, "failed to acquire memory region\n");
                retval = -EINVAL;
                goto failed;
            } else {
                struct resource resource_list[] = {DEFINE_RES_MEM(rmem->base,rmem->size)};
                if (info_enable) {
                    dev_info(dev, "assigned reserved memory node %s\n", rmem->name);
                }
                res = &resource_list[0];
            }
        }
    }

    if (resource_size(res) == 0) {
        dev_err(dev, "invalid resource size(=%zu).\n", (size_t)resource_size(res));
        retval = -EINVAL;
        goto failed;
    }
    if ((resource_size(res) & ~PAGE_MASK) != 0) {
        dev_err(dev, "invalid resource size(=%zu), size must be page alignemnt(=%zu).\n",
                (size_t)resource_size(res), (size_t)PAGE_SIZE);
        retval = -EINVAL;
        goto failed;
    }
    if ((res->start & ~PAGE_MASK) != 0) {
        dev_err(dev, "invalid resource addr(=%pad), addr must be page alignemnt(=%zu).\n",
                &res->start, (size_t)PAGE_SIZE);
        retval = -EINVAL;
        goto failed;
    }
    if (pfn_valid(PFN_DOWN(res->start)) || pfn_valid(PFN_DOWN(res->end)))
    {
        dev_err(dev, "invalid resource addr(=%pad) size(=%zu), this region is used by the kernel.\n",
                &res->start, (size_t)resource_size(res));
        retval = -EINVAL;
        goto failed;
    }
    /*
     * minor-number property
     */
    if        (uiomem_get_minor_number_property(dev, &u32_value) == 0) {
        minor_number = u32_value;
    } else if (of_property_read_u32(dev->of_node, "minor-number", &u32_value) == 0) {
        minor_number = u32_value;
    } else {
        minor_number = -1;
    }
    /*
     * device-name property
     */
    if (uiomem_get_device_name_property(dev, &device_name) != 0)
        device_name = of_get_property(dev->of_node, "device-name", NULL);
    if (IS_ERR_OR_NULL(device_name)) {
        if (minor_number < 0)
            device_name = dev_name(dev);
        else
            device_name = NULL;
    }
    /*
     * uiomem_object_create()
     */
    obj = uiomem_object_create(device_name, dev, minor_number);
    if (IS_ERR_OR_NULL(obj)) {
        retval = PTR_ERR(obj);
        dev_err(dev, "object create failed. return=%d.\n", retval);
        obj = NULL;
        retval = (retval == 0) ? -EINVAL : retval;
        goto failed;
    }
    dev_set_drvdata(dev, obj);
    /*
     * shareable property
     */
    if (of_property_read_bool(dev->of_node, "shareable")) {
        obj->shareable = true;
    } else if (uiomem_get_option_property(dev, &u64_value) == 0) {
        obj->shareable = uiomem_get_option_shareable(u64_value);
    } else {
        obj->shareable = false;
    }
    /*
     * cache property
     */
    if        (of_property_read_bool(dev->of_node, "cache-off"        )) {
        obj->cached   = false;
        obj->coherent = true;
    } else if (of_property_read_bool(dev->of_node, "cache-noncoherent")) {
        obj->cached   = true;
        obj->coherent = false;
    } else if (of_property_read_bool(dev->of_node, "cache-coherent"   )) {
        obj->cached   = true;
        obj->coherent = true;
    } else if (uiomem_get_option_property(dev, &u64_value) == 0) {
        obj->cached   = uiomem_get_option_cached(u64_value);
        obj->coherent = (obj->cached == false) ? true :
                        uiomem_get_option_coherent(u64_value);
    } else {
        obj->cached   = true;
        obj->coherent = false;
    }
#ifndef UIOMEM_CACHE_SYNC_OPERATION
    if (obj->coherent == false) {
        dev_warn(dev, "coherent=false, but cache synchronization not supported, forcing cache off.\n");
        obj->cached   = false;
        obj->coherent = true;
    }
#endif
    /*
     * set mem_region and mem_addr and mem_size
     */
    if (obj->shareable == true) {
        obj->mem_region = NULL;
        mem_addr        = res->start;
        mem_size        = resource_size(res);
    } else {
        obj->mem_region = request_mem_region(res->start, resource_size(res), dev_name(dev));
        if (obj->mem_region == NULL) {
            dev_err(dev, "request_mem_region failed.\n");
            retval = -EBUSY;
            goto failed;
        }
        mem_addr        = obj->mem_region->start;
        mem_size        = resource_size(obj->mem_region);
    }
    /*
     * sync-mode property
     */
    if (of_property_read_u32(dev->of_node, "sync-mode", &u32_value) == 0) {
        if ((u32_value < SYNC_MODE_MIN) || (u32_value > SYNC_MODE_MAX)) {
            dev_err(dev, "invalid sync-mode property value=%d\n", u32_value);
            goto failed;
        }
        obj->sync_mode &= ~SYNC_MODE_MASK;
        obj->sync_mode |= (int)u32_value;
    }
    /*
     * sync-always property
     */
    if (of_property_read_bool(dev->of_node, "sync-always")) {
        obj->sync_mode |= SYNC_ALWAYS;
    }
    /*
     * sync-direction property
     */
    if (of_property_read_u32(dev->of_node, "sync-direction", &u32_value) == 0) {
        if (u32_value > DIR_MAX) {
            dev_err(dev, "invalid sync-direction property value=%d\n", u32_value);
            goto failed;
        }
        obj->sync_direction = (int)u32_value;
    }
    /*
     * sync-offset property
     */
    if (of_property_read_ulong(dev->of_node, "sync-offset", &u64_value) == 0) {
        if (u64_value >= mem_size) {
            dev_err(dev, "invalid sync-offset property value=%llu\n", u64_value);
            goto failed;
        }
        obj->sync_offset = (int)u64_value;
    }
    /*
     * sync-size property
     */
    if (of_property_read_ulong(dev->of_node, "sync-size", &u64_value) == 0) {
        if (obj->sync_offset + u64_value > mem_size) {
            dev_err(dev, "invalid sync-size property value=%llu\n", u64_value);
            goto failed;
        }
        obj->sync_size = (size_t)u64_value;
    } else {
        obj->sync_size = (size_t)(mem_size - obj->sync_offset);
    }
    /*
     * uiomem_object_setup()
     */
    retval = uiomem_object_setup(obj, mem_addr, mem_size);
    if (retval) {
        dev_err(dev, "driver setup failed. return=%d\n", retval);
        goto failed;
    }

    if (info_enable) {
        uiomem_object_info(obj);
    }

    return 0;

failed:
    if (obj != NULL)
        (void)uiomem_platform_device_remove(dev);

    return retval;
}

/**
 * DOC: Uiomem Static Devices.
 *
 * This section defines the uiomem device to be created with arguments when loaded
 * into ther kernel with insmod.
 *
 */
#define DEFINE_UIOMEM_STATIC_DEVICE_PARAM(__num)                                    \
    static ulong     uiomem ## __num ## _addr = 0;                                  \
    module_param(    uiomem ## __num ## _addr, ulong, S_IRUGO);                     \
    MODULE_PARM_DESC(uiomem ## __num ## _addr, DRIVER_NAME #__num " start address");\
    static ulong     uiomem ## __num ## _size = 0;                                  \
    module_param(    uiomem ## __num ## _size, ulong, S_IRUGO);                     \
    MODULE_PARM_DESC(uiomem ## __num ## _size, DRIVER_NAME #__num " range size");   \
    static ulong     uiomem ## __num ## _option = UIOMEM_STATIC_DEVICE_OPTION_DEFAULT;\
    module_param(    uiomem ## __num ## _option, ulong, S_IRUGO);                   \
    MODULE_PARM_DESC(uiomem ## __num ## _option, DRIVER_NAME #__num                 \
                                                 UIOMEM_STATIC_DEVICE_OPTION_DESC );

#define CALL_UIOMEM_STATIC_DEVICE_CREATE(__num)                         \
    if (uiomem ## __num ## _size != 0) {                                \
        ida_simple_remove(&uiomem_device_ida, __num);                   \
        uiomem_platform_device_create(NULL, __num, uiomem ## __num ## _addr, uiomem ## __num ## _size, uiomem ## __num ## _option); \
    }

#define CALL_UIOMEM_STATIC_DEVICE_RESERVE_MINOR_NUMBER(__num)           \
    if (uiomem ## __num ## _size != 0) {                                \
        ida_simple_get(&uiomem_device_ida, __num, __num+1, GFP_KERNEL); \
    }

DEFINE_UIOMEM_STATIC_DEVICE_PARAM(0);
DEFINE_UIOMEM_STATIC_DEVICE_PARAM(1);
DEFINE_UIOMEM_STATIC_DEVICE_PARAM(2);
DEFINE_UIOMEM_STATIC_DEVICE_PARAM(3);
DEFINE_UIOMEM_STATIC_DEVICE_PARAM(4);
DEFINE_UIOMEM_STATIC_DEVICE_PARAM(5);
DEFINE_UIOMEM_STATIC_DEVICE_PARAM(6);
DEFINE_UIOMEM_STATIC_DEVICE_PARAM(7);

/**
 * uiomem_static_device_reserve_minor_number_all() - Reserve uiomem static device's minor-number.
 */
static void uiomem_static_device_reserve_minor_number_all(void)
{
    CALL_UIOMEM_STATIC_DEVICE_RESERVE_MINOR_NUMBER(0);
    CALL_UIOMEM_STATIC_DEVICE_RESERVE_MINOR_NUMBER(1);
    CALL_UIOMEM_STATIC_DEVICE_RESERVE_MINOR_NUMBER(2);
    CALL_UIOMEM_STATIC_DEVICE_RESERVE_MINOR_NUMBER(3);
    CALL_UIOMEM_STATIC_DEVICE_RESERVE_MINOR_NUMBER(4);
    CALL_UIOMEM_STATIC_DEVICE_RESERVE_MINOR_NUMBER(5);
    CALL_UIOMEM_STATIC_DEVICE_RESERVE_MINOR_NUMBER(6);
    CALL_UIOMEM_STATIC_DEVICE_RESERVE_MINOR_NUMBER(7);
}

/**
 * uiomem_static_device_create_all() - Create uiomem static devices.
 */
static void uiomem_static_device_create_all(void)
{
    CALL_UIOMEM_STATIC_DEVICE_CREATE(0);
    CALL_UIOMEM_STATIC_DEVICE_CREATE(1);
    CALL_UIOMEM_STATIC_DEVICE_CREATE(2);
    CALL_UIOMEM_STATIC_DEVICE_CREATE(3);
    CALL_UIOMEM_STATIC_DEVICE_CREATE(4);
    CALL_UIOMEM_STATIC_DEVICE_CREATE(5);
    CALL_UIOMEM_STATIC_DEVICE_CREATE(6);
    CALL_UIOMEM_STATIC_DEVICE_CREATE(7);
}

/**
 * DOC: Uiomem Platform Driver
 *
 * This section defines the uiomem platform driver.
 *
 * * uiomem_platform_driver_probe()   - Probe call for platform_device_add().
 * * uiomem_platform_driver_remove()  - Remove call for platform_device_del().
 * * uiomem_of_match                  - Open Firmware Device Identifier Matching Table.
 * * uiomem_platform_driver           - Platform Driver Structure.
 */

/**
 * uiomem_platform_driver_probe() -  Probe call for the device.
 * @pdev:       Handle to the platform device structure.
 * Return:      Success(=0) or error status(<0).
 *
 * It does all the memory allocation and registration for the device.
 */
static int uiomem_platform_driver_probe(struct platform_device *pdev)
{
    int retval = 0;
    struct resource* res = NULL;

    dev_dbg(&pdev->dev, "driver probe start.\n");

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    retval = uiomem_platform_device_probe(&pdev->dev, res);
    
    if (info_enable && (retval == 0)) {
        dev_info(&pdev->dev, "driver installed.\n");
    }
    return retval;
}
/**
 * _uiomem_platform_driver_remove() -  Remove call for the platform device driver.
 * @pdev:       Handle to the platform device structure.
 * Return:      Success(=0) or error status(<0).
 *
 * Unregister the device after releasing the resources.
 */
static int _uiomem_platform_driver_remove(struct platform_device *pdev)
{
    int retval = 0;

    dev_dbg(&pdev->dev, "driver remove start.\n");

    retval = uiomem_platform_device_remove(&pdev->dev);

    if (info_enable) {
        dev_info(&pdev->dev, "driver removed.\n");
    }
    return retval;
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
/**
 * uiomem_platform_driver_remove() -  Remove call for the platform device driver.
 * @pdev:       Handle to the platform device structure.
 * Return:      Success(=0) or error status(<0).
 *
 * Unregister the device after releasing the resources.
 */
static int uiomem_platform_driver_remove(struct platform_device *pdev)
{
    return _uiomem_platform_driver_remove(pdev);
}
#else
/**
 * uiomem_platform_driver_remove() -  Remove call for the platform device driver.
 * @pdev:       Handle to the platform device structure.
 * Return:      void
 *
 * Unregister the device after releasing the resources.
 */
static void uiomem_platform_driver_remove(struct platform_device *pdev)
{
    _uiomem_platform_driver_remove(pdev);
}
#endif

/**
 * Open Firmware Device Identifier Matching Table
 */
static struct of_device_id uiomem_of_match[] = {
    { .compatible = "ikwzm,uiomem", },
    { /* end of table */}
};
MODULE_DEVICE_TABLE(of, uiomem_of_match);

/**
 * Platform Driver Structure
 */
static struct platform_driver uiomem_platform_driver = {
    .probe  = uiomem_platform_driver_probe,
    .remove = uiomem_platform_driver_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name  = DRIVER_NAME,
        .of_match_table = uiomem_of_match,
    },
};

/**
 * DOC: Uiomem Kernel Module Operations
 *
 * * uiomem_cleanup()
 * * uiomem_init()
 * * uiomem_exit()
 */

static bool uiomem_platform_driver_registerd = false;

/**
 * uiomem_cleanup()
 */
static void uiomem_cleanup(void)
{
    uiomem_device_list_cleanup();
    if (uiomem_platform_driver_registerd){platform_driver_unregister(&uiomem_platform_driver);}
    if (uiomem_sys_class     != NULL    ){class_destroy(uiomem_sys_class);}
    if (uiomem_device_number != 0       ){unregister_chrdev_region(uiomem_device_number, 0);}
    ida_destroy(&uiomem_device_ida);
}

/**
 * uiomem_init()
 */
static int __init uiomem_init(void)
{
    int retval = 0;

    ida_init(&uiomem_device_ida);
    INIT_LIST_HEAD(&uiomem_device_list);
    mutex_init(&uiomem_device_list_sem);

    retval = alloc_chrdev_region(&uiomem_device_number, 0, 0, DRIVER_NAME);
    if (retval != 0) {
        printk(KERN_ERR "%s: couldn't allocate device major number. return=%d\n", DRIVER_NAME, retval);
        uiomem_device_number = 0;
        goto failed;
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    uiomem_sys_class = class_create(THIS_MODULE, DRIVER_NAME);
#else
    uiomem_sys_class = class_create(DRIVER_NAME);
#endif
    if (IS_ERR_OR_NULL(uiomem_sys_class)) {
        retval = PTR_ERR(uiomem_sys_class);
        uiomem_sys_class = NULL;
        printk(KERN_ERR "%s: couldn't create sys class. return=%d\n", DRIVER_NAME, retval);
        retval = (retval == 0) ? -ENOMEM : retval;
        goto failed;
    }

    uiomem_sys_class_set_attributes();

    uiomem_static_device_reserve_minor_number_all();

    retval = platform_driver_register(&uiomem_platform_driver);
    if (retval) {
        printk(KERN_ERR "%s: couldn't register platform driver. return=%d\n", DRIVER_NAME, retval);
        uiomem_platform_driver_registerd = false;
        goto failed;
    } else {
        uiomem_platform_driver_registerd = true;
    }

    uiomem_static_device_create_all();

    return 0;

 failed:
    uiomem_cleanup();
    return retval;
}

/**
 * uiomem_exit()
 */
static void __exit uiomem_exit(void)
{
    uiomem_cleanup();
}

module_init(uiomem_init);
module_exit(uiomem_exit);
