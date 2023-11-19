/*
 * Copyright (c) 2023 averne <averne381@gmail.com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef AVUTIL_TX1_H
#define AVUTIL_TX1_H

#include <stdint.h>
#include <stdbool.h>

#include "nvhost_ioctl.h"
#include "nvmap_ioctl.h"

typedef struct AVTX1Channel {
#ifndef __SWITCH__
    int fd;
    int module_id;
#else
    NvChannel channel;
#endif

    uint32_t syncpt;

#ifdef __SWITCH__
    MmuRequest mmu_request;
#endif
    uint32_t clock;
} AVTX1Channel;

typedef struct AVTX1Map {
#ifndef __SWITCH__
    uint32_t handle;
    uint32_t size;
    void *cpu_addr;
#else
    NvMap map;
    uint32_t iova;
    uint32_t owner;
#endif
    bool is_linear;
} AVTX1Map;

typedef struct ACTX1Cmdbuf {
    AVTX1Map *map;

    uint32_t mem_offset, mem_size;

    uint32_t *cur_word;

    struct nvhost_cmdbuf       *cmdbufs;
#ifndef __SWITCH__
    struct nvhost_cmdbuf_ext   *cmdbuf_exts;
    uint32_t                   *class_ids;
#endif
    uint32_t num_cmdbufs;

#ifndef __SWITCH__
    struct nvhost_reloc        *relocs;
    struct nvhost_reloc_type   *reloc_types;
    struct nvhost_reloc_shift  *reloc_shifts;
    uint32_t num_relocs;
#endif

    struct nvhost_syncpt_incr  *syncpt_incrs;
#ifndef __SWITCH__
    uint32_t                   *fences;
#endif
    uint32_t num_syncpt_incrs;

#ifndef __SWITCH__
    struct nvhost_waitchk      *waitchks;
    uint32_t num_waitchks;
#endif
} AVTX1Cmdbuf;

int ff_tx1_channel_open(AVTX1Channel *channel, const char *dev);
int ff_tx1_channel_close(AVTX1Channel *channel);
int ff_tx1_channel_get_clock_rate(AVTX1Channel *channel, uint32_t moduleid, uint32_t *clock_rate);
int ff_tx1_channel_set_clock_rate(AVTX1Channel *channel, uint32_t moduleid, uint32_t clock_rate);
int ff_tx1_channel_submit(AVTX1Channel *channel, AVTX1Cmdbuf *cmdbuf, uint32_t *fence);
int ff_tx1_channel_set_submit_timeout(AVTX1Channel *channel, uint32_t timeout_ms);

int ff_tx1_syncpt_wait(AVTX1Channel *channel, uint32_t threshold, int32_t timeout);

int ff_tx1_map_allocate(AVTX1Map *map, uint32_t size, uint32_t align, uint32_t flags);
int ff_tx1_map_free(AVTX1Map *map);
int ff_tx1_map_from_va(AVTX1Map *map, void *mem, uint32_t size, uint32_t align, uint32_t flags);
int ff_tx1_map_close(AVTX1Map *map);
int ff_tx1_map_map(AVTX1Map *map);
int ff_tx1_map_unmap(AVTX1Map *map);
int ff_tx1_map_realloc(AVTX1Map *map, uint32_t size, uint32_t align, uint32_t flags);

static inline int ff_tx1_map_create(AVTX1Map *map, uint32_t size, uint32_t align, uint32_t flags) {
    int err;

    err = ff_tx1_map_allocate(map, size, align, flags);
    if (err < 0)
        return err;

    return ff_tx1_map_map(map);
}

static inline int ff_tx1_map_destroy(AVTX1Map *map) {
    int err;

    err = ff_tx1_map_unmap(map);
    if (err < 0)
        return err;

    return ff_tx1_map_free(map);
}

int ff_tx1_cmdbuf_init(AVTX1Cmdbuf *cmdbuf);
int ff_tx1_cmdbuf_deinit(AVTX1Cmdbuf *cmdbuf);
int ff_tx1_cmdbuf_add_memory(AVTX1Cmdbuf *cmdbuf, AVTX1Map *map, uint32_t offset, uint32_t size);
int ff_tx1_cmdbuf_clear(AVTX1Cmdbuf *cmdbuf);
int ff_tx1_cmdbuf_begin(AVTX1Cmdbuf *cmdbuf, uint32_t class_id);
int ff_tx1_cmdbuf_end(AVTX1Cmdbuf *cmdbuf);
int ff_tx1_cmdbuf_push_word(AVTX1Cmdbuf *cmdbuf, uint32_t word);
int ff_tx1_cmdbuf_push_value(AVTX1Cmdbuf *cmdbuf, uint32_t offset, uint32_t word);
int ff_tx1_cmdbuf_push_reloc(AVTX1Cmdbuf *cmdbuf, uint32_t offset, AVTX1Map *target, uint32_t target_offset,
                             int reloc_type, int shift);
int ff_tx1_cmdbuf_push_wait(AVTX1Cmdbuf *cmdbuf, uint32_t syncpt, uint32_t fence);
int ff_tx1_cmdbuf_add_syncpt_incr(AVTX1Cmdbuf *cmdbuf, uint32_t syncpt, uint32_t
                                  num_incrs, uint32_t fence);
int ff_tx1_cmdbuf_add_waitchk(AVTX1Cmdbuf *cmdbuf, uint32_t syncpt, uint32_t fence);

static inline uint32_t ff_tx1_map_get_handle(AVTX1Map *map) {
#ifndef __SWITCH__
    return map->handle;
#else
    return map->map.handle;
#endif
}

static inline void *ff_tx1_map_get_addr(AVTX1Map *map) {
#ifndef __SWITCH__
    return map->cpu_addr;
#else
    return map->map.cpu_addr;
#endif
}

static inline uint32_t ff_tx1_map_get_size(AVTX1Map *map) {
#ifndef __SWITCH__
    return map->size;
#else
    return map->map.size;
#endif
}

/* Addresses are shifted by 8 bits in the command buffer, requiring an alignment to 256 */
#define FF_TX1_MAP_ALIGN (1 << 8)

#define FF_TX1_VALUE(offset, field, value)                                                          \
    ((value &                                                                                       \
    ((uint32_t)((UINT64_C(1) << ((1?offset ## _ ## field) - (0?offset ## _ ## field) + 1)) - 1)))   \
    << (0?offset ## _ ## field))

#define FF_TX1_ENUM(offset, field, value)                                                           \
    ((offset ## _ ## field ## _ ## value &                                                          \
    ((uint32_t)((UINT64_C(1) << ((1?offset ## _ ## field) - (0?offset ## _ ## field) + 1)) - 1)))   \
    << (0?offset ## _ ## field))

#define FF_TX1_PUSH_VALUE(cmdbuf, offset, value) ({                                     \
    int _err = ff_tx1_cmdbuf_push_value(cmdbuf, (offset) / sizeof(uint32_t), value);    \
    if (_err < 0)                                                                       \
        return _err;                                                                    \
})

#define FF_TX1_PUSH_RELOC(cmdbuf, offset, target, target_offset, type) ({               \
    int _err = ff_tx1_cmdbuf_push_reloc(cmdbuf, (offset) / sizeof(uint32_t),            \
                                        target, target_offset, type, 8);                \
    if (_err < 0)                                                                       \
        return _err;                                                                    \
})

#endif /* AVUTIL_TX1_H */
