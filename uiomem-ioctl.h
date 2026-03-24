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
