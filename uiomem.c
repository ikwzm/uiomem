/*********************************************************************************
 *
 *       Copyright (C) 2015-2020 Ichiro Kawazome
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

#define DRIVER_VERSION     "0.0.2"
#define DRIVER_NAME        "uiomem"
#define DEVICE_NAME_FORMAT "uiomem%d"
#define DEVICE_MAX_NUM      256
#define UIOMEM_DEBUG        1

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
 * DOC: Uiomem Device Data Structure
 *
 * This section defines the structure of uiomem device.
 *
 */

/**
 * struct uiomem_device_data - uiomem device data structure.
 */
struct uiomem_device_data {
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


#if (defined(CONFIG_ARM64))
/**
 * DOC: Data Cache Clean/Invalid for arm64 architecture.
 *
 * This section defines mem_sync_sinfle_for_cpu() and mem_sync_single_for_device().
 *
 * * arm64_read_dcache_line_size()     - read data cache line size of arm64.
 * * arm64_inval_dcache_area()         - invalid data cache.
 * * arm64_clean_dcache_area()         - clean(flush and invalidiate) data cache.
 * * arch_sync_for_cpu()               - _uiomem_sync_for_cpu() for arm64
 * * arch_sync_for_dev()               - _uiomem_sync_for_dev() for arm64
 */
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
#endif

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
 * @this:       Pointer to the uiomem device data structure.
 * @virt_addr:  Virtua address.
 * @phys_addr:  Physical address.
 * @size:       Sync size.
 * @direction:  Sync direction.
 * Return:      Success(=0) or error status(<0).
 */
static inline void _uiomem_sync_for_cpu(
  struct uiomem_device_data*  this      ,
  void*                       virt_addr ,
  phys_addr_t                 phys_addr ,
  size_t                      size      ,
  enum uiomem_direction       direction
) {
    arch_sync_for_cpu(virt_addr, phys_addr, size, direction);
}

/**
 * _uiomem_sync_for_dev() - call arch_sync_for_dev().
 * @this:       Pointer to the uiomem device data structure.
 * @virt_addr:  Virtua address.
 * @phys_addr:  Physical address.
 * @size:       Sync size.
 * @direction:  Sync direction.
 * Return:      Success(=0) or error status(<0).
 */
static inline void _uiomem_sync_for_dev(
  struct uiomem_device_data*  this      ,
  void*                       virt_addr ,
  phys_addr_t                 phys_addr ,
  size_t                      size      ,
  enum uiomem_direction       direction
) {
    arch_sync_for_dev(virt_addr, phys_addr, size, direction);
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
 * * /sys/class/uiomem/<device-name>/sync_mode
 * * /sys/class/uiomem/<device-name>/sync_offset
 * * /sys/class/uiomem/<device-name>/sync_size
 * * /sys/class/uiomem/<device-name>/sync_direction
 * * /sys/class/uiomem/<device-name>/sync_owner
 * * /sys/class/uiomem/<device-name>/sync_for_cpu
 * * /sys/class/uiomem/<device-name>/sync_for_device
 * * /sys/class/uiomem/<device-name>/dma_coherent
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
 * @this:       Pointer to the uiomem device data structure.
 * @command:    sync command (this->sync_for_cpu or this->sync_for_device)
 * @phys_addr:  Pointer to the phys_addr for dma_sync_single_for_...()
 * @size:       Pointer to the size for dma_sync_single_for_...()
 * @direction:  Pointer to the direction for dma_sync_single_for_...()
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_sync_command_argments(
    struct uiomem_device_data *this      ,
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
 * @this:       Pointer to the uiomem device data structure.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_sync_for_cpu(struct uiomem_device_data* this)
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
            _uiomem_sync_for_cpu(this, virt_addr, phys_addr, size, direction);
            this->sync_for_cpu = 0;
            this->sync_owner   = 0;
        }
    }
    return status;
}

/**
 * uiomem_sync_for_device() - call _uiomem_sync_for_dev() when (sync_for_device != 0)
 * @this:       Pointer to the uiomem device data structure.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_sync_for_device(struct uiomem_device_data* this)
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
            _uiomem_sync_for_dev(this, virt_addr, phys_addr, size, direction);
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
    struct uiomem_device_data* this = dev_get_drvdata(dev);  \
    if (mutex_lock_interruptible(&this->sem) != 0)           \
        return -ERESTARTSYS;                                 \
    status = sprintf(buf, __format, (__value));              \
    mutex_unlock(&this->sem);                                \
    return status;                                           \
}

static inline int NO_ACTION(struct uiomem_device_data* this){return 0;}

#define DEF_ATTR_SET(__attr_name, __min, __max, __pre_action, __post_action) \
static ssize_t uiomem_set_ ## __attr_name(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) \
{ \
    ssize_t       status; \
    u64           value;  \
    struct uiomem_device_data* this = dev_get_drvdata(dev);                  \
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

static struct device_attribute uiomem_device_attrs[] = {
  __ATTR(driver_version , 0444, uiomem_show_driver_version  , NULL                        ),
  __ATTR(size           , 0444, uiomem_show_size            , NULL                        ),
  __ATTR(phys_addr      , 0444, uiomem_show_phys_addr       , NULL                        ),
  __ATTR(sync_mode      , 0664, uiomem_show_sync_mode       , uiomem_set_sync_mode        ),
  __ATTR(sync_offset    , 0664, uiomem_show_sync_offset     , uiomem_set_sync_offset      ),
  __ATTR(sync_size      , 0664, uiomem_show_sync_size       , uiomem_set_sync_size        ),
  __ATTR(sync_direction , 0664, uiomem_show_sync_direction  , uiomem_set_sync_direction   ),
  __ATTR(sync_owner     , 0444, uiomem_show_sync_owner      , NULL                        ),
  __ATTR(sync_for_cpu   , 0664, uiomem_show_sync_for_cpu    , uiomem_set_sync_for_cpu     ),
  __ATTR(sync_for_device, 0664, uiomem_show_sync_for_device , uiomem_set_sync_for_device  ),
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
    struct uiomem_device_data* this;
    int status = 0;

    this = container_of(inode->i_cdev, struct uiomem_device_data, cdev);
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
    struct uiomem_device_data* this = file->private_data;

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

/**
 * uiomem_device_file_mmap() - uiomem device file memory map operation.
 * @file:       Pointer to the file structure.
 * @vma:        Pointer to the vm area structure.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_device_file_mmap(struct file *file, struct vm_area_struct* vma)
{
    struct uiomem_device_data* this = file->private_data;
    unsigned long              page_frame_num;
    unsigned long              map_area_size;

    if (vma->vm_pgoff + vma_pages(vma) > (this->size >> PAGE_SHIFT))
        return -ENXIO;

    if ((file->f_flags & O_SYNC) | (this->sync_mode & SYNC_ALWAYS)) {
        switch (this->sync_mode & SYNC_MODE_MASK) {
            case SYNC_MODE_NONCACHED :
                vma->vm_flags    |= VM_IO;
                vma->vm_page_prot = _PGPROT_NONCACHED(vma->vm_page_prot);
                break;
            case SYNC_MODE_WRITECOMBINE :
                vma->vm_flags    |= VM_IO;
                vma->vm_page_prot = _PGPROT_WRITECOMBINE(vma->vm_page_prot);
                break;
            case SYNC_MODE_DMACOHERENT :
                vma->vm_flags    |= VM_IO;
                vma->vm_page_prot = _PGPROT_DMACOHERENT(vma->vm_page_prot);
                break;
            default :
                break;
        }
    }
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
    struct uiomem_device_data* this      = file->private_data;
    int                        result    = 0;
    size_t                     xfer_size;
    size_t                     remain_size;
    phys_addr_t                phys_addr;
    void*                      virt_addr;

    if (mutex_lock_interruptible(&this->sem))
        return -ERESTARTSYS;

    if (*ppos >= this->size) {
        result = 0;
        goto return_unlock;
    }

    phys_addr = this->phys_addr + *ppos;
    virt_addr = this->virt_addr + *ppos;
    xfer_size = (*ppos + count >= this->size) ? this->size - *ppos : count;

    if ((file->f_flags & O_SYNC) | (this->sync_mode & SYNC_ALWAYS))
        _uiomem_sync_for_cpu(this, virt_addr, phys_addr, xfer_size, UIOMEM_READ_ONLY);

    if ((remain_size = copy_to_user(buff, virt_addr, xfer_size)) != 0) {
        result = 0;
        goto return_unlock;
    }

    if ((file->f_flags & O_SYNC) | (this->sync_mode & SYNC_ALWAYS))
        _uiomem_sync_for_dev(this, virt_addr, phys_addr, xfer_size, UIOMEM_READ_ONLY);

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
    struct uiomem_device_data* this      = file->private_data;
    int                        result    = 0;
    size_t                     xfer_size;
    size_t                     remain_size;
    phys_addr_t                phys_addr;
    void*                      virt_addr;

    if (mutex_lock_interruptible(&this->sem))
        return -ERESTARTSYS;

    if (*ppos >= this->size) {
        result = 0;
        goto return_unlock;
    }

    phys_addr = this->phys_addr + *ppos;
    virt_addr = this->virt_addr + *ppos;
    xfer_size = (*ppos + count >= this->size) ? this->size - *ppos : count;

    if ((file->f_flags & O_SYNC) | (this->sync_mode & SYNC_ALWAYS))
        _uiomem_sync_for_cpu(this, virt_addr, phys_addr, xfer_size, UIOMEM_WRITE_ONLY);

    if ((remain_size = copy_from_user(virt_addr, buff, xfer_size)) != 0) {
        result = 0;
        goto return_unlock;
    }

    if ((file->f_flags & O_SYNC) | (this->sync_mode & SYNC_ALWAYS))
        _uiomem_sync_for_dev(this, virt_addr, phys_addr, xfer_size, UIOMEM_WRITE_ONLY);

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
    struct uiomem_device_data* this = file->private_data;
    loff_t                      new_pos;

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
 * uiomem device file operation table.
 */
static const struct file_operations uiomem_device_file_ops = {
    .owner   = THIS_MODULE,
    .open    = uiomem_device_file_open,
    .release = uiomem_device_file_release,
    .mmap    = uiomem_device_file_mmap,
    .read    = uiomem_device_file_read,
    .write   = uiomem_device_file_write,
    .llseek  = uiomem_device_file_llseek,
};

/**
 * DOC: Uiomem Device Data Operations
 *
 * This section defines the operation of uiomem device data.
 *
 * * uiomem_device_ida         - Uiomem Device Minor Number allocator variable.
 * * uiomem_device_number      - Uiomem Device Major Number.
 * * uiomem_device_create()    - Create uiomem device data.
 * * uiomem_device_setup()     - Setup the uiomem device data.
 * * uiomem_device_info()      - Print infomation the uiomem device data.
 * * uiomem_device_destroy()   - Destroy the uiomem device data.
 * * uiomem_device_probe()     - Probe call for the device driver.
 * * uiomem_device_remove()    - Remove uiomem device data from device driver.
 */

static DEFINE_IDA(uiomem_device_ida);
static dev_t      uiomem_device_number = 0;

/**
 * uiomem_device_create() -  Create uiomem device data.
 * @name:       device name   or NULL.
 * @parent:     parent device or NULL.
 * @minor:      minor_number  or -1 or -2.
 * Return:      Pointer to the uiomem device data or NULL.
 */
static struct uiomem_device_data* uiomem_device_create(const char* name, struct device* parent, int minor)
{
    struct uiomem_device_data* this     = NULL;
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
     * create (uiomem_device_data*) this.
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
 * uiomem_device_setup() - Setup the uiomem device data.
 * @this:       Pointer to the uiomem device data.
 * @res:        handle to the resource structure.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_device_setup(struct uiomem_device_data* this, struct resource* res)
{
    if (!this)
        return -ENODEV;
    /*
     * setup mem_region, phys_addr, size
     */
    this->mem_region = res;
    this->phys_addr  = res->start;
    this->size       = (size_t)resource_size(res);
    /*
     * setup virtual address
     */
    this->virt_addr = (void*)ioremap_cache(this->phys_addr, this->size);
    if (IS_ERR_OR_NULL(this->virt_addr)) {
        int retval = PTR_ERR(this->virt_addr);
        dev_err(this->sys_dev, "ioremap_cache(addr=%pad,size=%zu) failed. return(%d)\n", &this->phys_addr, this->size, retval);
        this->virt_addr = NULL;
        return (retval == 0) ? -ENOMEM : retval;
    }
    return 0;
}

/**
 * uiomem_device_info() - Print infomation the uiomem device data structure.
 * @this:       Pointer to the uiomem device data structure.
 */
static void uiomem_device_info(struct uiomem_device_data* this)
{
    dev_info(this->sys_dev, "driver version = %s\n"  , DRIVER_VERSION);
    dev_info(this->sys_dev, "major number   = %d\n"  , MAJOR(this->device_number));
    dev_info(this->sys_dev, "minor number   = %d\n"  , MINOR(this->device_number));
    dev_info(this->sys_dev, "range address  = %pad\n", &this->phys_addr);
    dev_info(this->sys_dev, "range size     = %zu\n" , this->size);
}

/**
 * uiomem_device_destroy() -  Destroy the uiomem device data.
 * @this:       Pointer to the uiomem device data.
 * Return:      Success(=0) or error status(<0).
 *
 * Unregister the device after releasing the resources.
 */
static int uiomem_device_destroy(struct uiomem_device_data* this)
{
    if (!this)
        return -ENODEV;

    if (this->virt_addr != NULL) {
        iounmap((void __iomem *)this->virt_addr);
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
 * uiomem_device_remove()   - Remove uiomem device data from device driver.
 * @dev:        handle to the device structure.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_device_remove(struct device *dev)
{
    struct uiomem_device_data* this   = dev_get_drvdata(dev);
    int                        retval = 0;

    if (this != NULL) {
        retval = uiomem_device_destroy(this);
        dev_set_drvdata(dev, NULL);
    } else {
        retval = -ENODEV;
    }
    return retval;
}

/**
 * uiomem_device_probe() -  Probe call for the device driver.
 * @dev:        handle to the device structure.
 * @res:        handle to the resource structure.
 * Return:      Success(=0) or error status(<0).
 *
 * It does all the memory allocation and registration for the device.
 */
static int uiomem_device_probe(struct device *dev, struct resource* res)
{
    int                         retval       = 0;
    unsigned int                u32_value    = 0;
    int                         minor_number = -1;
    struct resource*            mem_resource;
    struct uiomem_device_data*  device_data  = NULL;
    const char*                 device_name  = NULL;

    /*
     * check handle to the resource structure.
     */
    if (IS_ERR_OR_NULL(res)) {
        dev_err(dev, "can not found resource\n");
        retval = -EINVAL;
        goto failed;
    }
    if (resource_size(res) == 0) {
        dev_err(dev, "invalid resource size(=%zu).\n", (size_t)resource_size(res));
        retval = -EINVAL;
        goto failed;
    }
    if ((resource_size(res) & ~PAGE_MASK) != 0) {
        dev_err(dev, "invalid resource size(=%zu), size must be page alignemnt(=%zu).\n", (size_t)resource_size(res), PAGE_SIZE);
        retval = -EINVAL;
        goto failed;
    }
    if ((res->start & ~PAGE_MASK) != 0) {
        dev_err(dev, "invalid resource addr(=%pad), addr must be page alignemnt(=%zu).\n", &res->start, PAGE_SIZE);
        retval = -EINVAL;
        goto failed;
    }
    if (pfn_valid(PFN_DOWN(res->start)) || pfn_valid(PFN_DOWN(res->end)))
    {
        dev_err(dev, "invalid resource addr(=%pad) size(=%zu), this region is used by the kernel.\n", &res->start, (size_t)resource_size(res));
        retval = -EINVAL;
        goto failed;
    }
    /*
     * minor-number property
     */
    if        (device_property_read_u32(dev, "minor-number", &u32_value) == 0) {
        minor_number = u32_value;
    } else if (of_property_read_u32(dev->of_node, "minor-number", &u32_value) == 0) {
        minor_number = u32_value;
    } else {
        minor_number = -1;
    }
    /*
     * device-name property
     */
    if (device_property_read_string(dev, "device-name", &device_name) != 0)
        device_name = of_get_property(dev->of_node, "device-name", NULL);

    if (IS_ERR_OR_NULL(device_name)) {
        if (minor_number < 0)
            device_name = dev_name(dev);
        else
            device_name = NULL;
    }
    /*
     * uiomem_device_create()
     */
    device_data = uiomem_device_create(device_name, dev, minor_number);
    if (IS_ERR_OR_NULL(device_data)) {
        retval = PTR_ERR(device_data);
        dev_err(dev, "driver create failed. return=%d.\n", retval);
        device_data = NULL;
        retval = (retval == 0) ? -EINVAL : retval;
        goto failed;
    }
    dev_set_drvdata(dev, device_data);
    /*
     * set mem_resource
     */
    if (of_property_read_bool(dev->of_node, "shareable")) {
        mem_resource = res;
        device_data->mem_region = NULL;
    } else {
        mem_resource = request_mem_region(res->start, resource_size(res), dev_name(dev));
        if (mem_resource == NULL) {
            dev_err(dev, "request_mem_region failed.\n");
            retval = -EBUSY;
            goto failed;
        }
    }
    /*
     * sync-mode property
     */
    if (of_property_read_u32(dev->of_node, "sync-mode", &u32_value) == 0) {
        if ((u32_value < SYNC_MODE_MIN) || (u32_value > SYNC_MODE_MAX)) {
            dev_err(dev, "invalid sync-mode property value=%d\n", u32_value);
            goto failed;
        }
        device_data->sync_mode &= ~SYNC_MODE_MASK;
        device_data->sync_mode |= (int)u32_value;
    }
    /*
     * sync-always property
     */
    if (of_property_read_bool(dev->of_node, "sync-always")) {
        device_data->sync_mode |= SYNC_ALWAYS;
    }
    /*
     * sync-direction property
     */
    if (of_property_read_u32(dev->of_node, "sync-direction", &u32_value) == 0) {
        if (u32_value > DIR_MAX) {
            dev_err(dev, "invalid sync-direction property value=%d\n", u32_value);
            goto failed;
        }
        device_data->sync_direction = (int)u32_value;
    }
    /*
     * sync-offset property
     */
    if (of_property_read_u32(dev->of_node, "sync-offset", &u32_value) == 0) {
        if (u32_value >= resource_size(mem_resource)) {
            dev_err(dev, "invalid sync-offset property value=%d\n", u32_value);
            goto failed;
        }
        device_data->sync_offset = (int)u32_value;
    }
    /*
     * sync-size property
     */
    if (of_property_read_u32(dev->of_node, "sync-size", &u32_value) == 0) {
        if (device_data->sync_offset + u32_value > resource_size(mem_resource)) {
            dev_err(dev, "invalid sync-size property value=%d\n", u32_value);
            goto failed;
        }
        device_data->sync_size = (size_t)u32_value;
    } else {
        device_data->sync_size = (size_t)(resource_size(mem_resource) - device_data->sync_offset);
    }
    /*
     * uiomem_device_setup()
     */
    retval = uiomem_device_setup(device_data, mem_resource);
    if (retval) {
        dev_err(dev, "driver setup failed. return=%d\n", retval);
        goto failed;
    }

    if (info_enable) {
        uiomem_device_info(device_data);
    }

    return 0;

failed:
    if (device_data != NULL)
        (void)uiomem_device_remove(dev);

    return retval;
}

/**
 * DOC: Uiomem Platform Device.
 *
 * This section defines the uiomem platform device list.
 *
 * * struct uiomem_platform_device       - uiomem platform device structure.
 * * uiomem_platform_device_list         - list of uiomem platform device structure.
 * * uiomem_platform_device_sem          - semaphore of uiomem platform device list.
 * * uiomem_platform_device_create()     - Create uiomem platform device and add to list.
 * * uiomem_platform_device_remove()     - Remove uiomem platform device and delete from list.
 * * uiomem_platform_device_remove_all() - Remove all uiomem platform devices and clear list.
 */
#include <linux/property.h>

/**
 * struct uiomem_platform_device - uiomem platform device structure.
 */
struct uiomem_platform_device {
    struct device*       dev;
    struct list_head     list;
};

/**
 * uiomem_static_device_list     - list of uiomem platform device structure.
 * uiomem_platform_device_sem    - semaphore of uiomem platform device list.
 */
static struct list_head uiomem_platform_device_list;
static struct mutex     uiomem_platform_device_sem;

/**
 * uiomem_platform_device_create() - Create uiomem platform device and add to list.
 * @name:       device name or NULL.
 * @id:         device id.
 * @size:       buffer size.
 * Return:      Success(=0) or error status(<0).
 */
static int uiomem_platform_device_create(const char* name, int id, ulong addr, ulong size)
{
    struct platform_device*         pdev       = NULL;
    struct uiomem_platform_device*  plat       = NULL;
    int                             retval     = 0;
    bool                            list_added = false;

    if (size == 0)
        return -EINVAL;

    pdev = platform_device_alloc(DRIVER_NAME, id);
    if (IS_ERR_OR_NULL(pdev)) {
        retval = PTR_ERR(pdev);
        pdev   = NULL;
        printk(KERN_ERR "platform_device_alloc(%s,%d) failed. return=%d\n", DRIVER_NAME, id, retval);
        goto failed;
    }

    plat = kzalloc(sizeof(*plat), GFP_KERNEL);
    if (IS_ERR_OR_NULL(plat)) {
        retval = PTR_ERR(plat);
        plat   = NULL;
        dev_err(&pdev->dev, "kzalloc() failed. return=%d\n", retval);
        goto failed;
    }

    {
        struct resource resource_list[] = {
            {
                .start = (phys_addr_t)(addr),
                .end   = (phys_addr_t)(addr + size-1),
                .flags = IORESOURCE_MEM,
            },
        };
        retval = platform_device_add_resources(pdev, resource_list, ARRAY_SIZE(resource_list));
        if (retval != 0) {
            dev_err(&pdev->dev, "platform_device_add_resources failed. return=%d\n", retval);
            goto failed;
        }
    }

    {
        struct property_entry   props_list[] = {
            PROPERTY_ENTRY_STRING("device-name" , name),
            PROPERTY_ENTRY_U32(   "minor-number", id  ),
            {},
        };
        struct property_entry* props = (name != NULL) ? &props_list[0] : &props_list[1];
        retval = device_add_properties(&pdev->dev, props);
        if (retval != 0) {
            dev_err(&pdev->dev, "device_add_properties failed. return=%d\n", retval);
            goto failed;
        }
    }

    plat->dev  = &pdev->dev;
    mutex_lock(&uiomem_platform_device_sem);
    list_add_tail(&plat->list, &uiomem_platform_device_list);
    list_added = true;
    mutex_unlock(&uiomem_platform_device_sem);
    
    retval = platform_device_add(pdev);
    if (retval != 0) {
        dev_err(&pdev->dev, "platform_device_add failed. return=%d\n", retval);
        goto failed;
    }

    return 0;

 failed:
    if (list_added == true) {
        mutex_lock(&uiomem_platform_device_sem);
        list_del(&plat->list);
        mutex_unlock(&uiomem_platform_device_sem);
    }
    if (pdev != NULL) {
        platform_device_put(pdev);
    }
    if (plat != NULL) {
        kfree(plat);
    }
    return retval;
}

/**
 * uiomem_platform_device_remove() - Remove uiomem platform device and delete from list.
 * @plat:       uiomem_platform_device*
 */
static void uiomem_platform_device_remove(struct uiomem_platform_device* plat)
{
    struct device*           dev  = plat->dev;
    struct platform_device*  pdev = to_platform_device(dev);
    platform_device_del(pdev);
    platform_device_put(pdev);
    mutex_lock(&uiomem_platform_device_sem);
    list_del(&plat->list);
    mutex_unlock(&uiomem_platform_device_sem);
    kfree(plat);
}

/**
 * uiomem_platform_device_remove_all() - Remove all uiomem platform devices and clear list.
 */
static void uiomem_platform_device_remove_all(void)
{
    while(!list_empty(&uiomem_platform_device_list)) {
        struct uiomem_platform_device* plat = list_first_entry(&uiomem_platform_device_list, typeof(*(plat)), list);
        uiomem_platform_device_remove(plat);
    }
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
    MODULE_PARM_DESC(uiomem ## __num ## _size, DRIVER_NAME #__num " range size");

#define CALL_UIOMEM_STATIC_DEVICE_CREATE(__num)                         \
    if (uiomem ## __num ## _size != 0) {                                \
        ida_simple_remove(&uiomem_device_ida, __num);                   \
        uiomem_platform_device_create(NULL, __num, uiomem ## __num ## _addr, uiomem ## __num ## _size); \
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
 * * uiomem_platform_driver_probe()   - Probe  call for the platform device driver.
 * * uiomem_platform_driver_remove()  - Remove call for the platform device driver.
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
    
    retval = uiomem_device_probe(&pdev->dev, res);
    
    if (info_enable && (retval == 0)) {
        dev_info(&pdev->dev, "driver installed.\n");
    }
    return retval;
}
/**
 * uiomem_platform_driver_remove() -  Remove call for the device.
 * @pdev:       Handle to the platform device structure.
 * Return:      Success(=0) or error status(<0).
 *
 * Unregister the device after releasing the resources.
 */
static int uiomem_platform_driver_remove(struct platform_device *pdev)
{
    int retval = 0;

    dev_dbg(&pdev->dev, "driver remove start.\n");

    retval = uiomem_device_remove(&pdev->dev);

    if (info_enable) {
        dev_info(&pdev->dev, "driver removed.\n");
    }
    return retval;
}

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
    uiomem_platform_device_remove_all();
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
    INIT_LIST_HEAD(&uiomem_platform_device_list);
    mutex_init(&uiomem_platform_device_sem);

    retval = alloc_chrdev_region(&uiomem_device_number, 0, 0, DRIVER_NAME);
    if (retval != 0) {
        printk(KERN_ERR "%s: couldn't allocate device major number. return=%d\n", DRIVER_NAME, retval);
        uiomem_device_number = 0;
        goto failed;
    }

    uiomem_sys_class = class_create(THIS_MODULE, DRIVER_NAME);
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
