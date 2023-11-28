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

#ifndef AVUTIL_NVTEGRA_H
#define AVUTIL_NVTEGRA_H

#include <stdint.h>
#include <stdbool.h>

#include "nvhost_ioctl.h"
#include "nvmap_ioctl.h"

typedef struct AVNVTegraChannel {
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
} AVNVTegraChannel;

typedef struct AVNVTegraMap {
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
} AVNVTegraMap;

typedef struct AVNVTegraCmdbuf {
    AVNVTegraMap *map;

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
} AVNVTegraCmdbuf;

int ff_nvtegra_channel_open(AVNVTegraChannel *channel, const char *dev);
int ff_nvtegra_channel_close(AVNVTegraChannel *channel);
int ff_nvtegra_channel_get_clock_rate(AVNVTegraChannel *channel, uint32_t moduleid, uint32_t *clock_rate);
int ff_nvtegra_channel_set_clock_rate(AVNVTegraChannel *channel, uint32_t moduleid, uint32_t clock_rate);
int ff_nvtegra_channel_submit(AVNVTegraChannel *channel, AVNVTegraCmdbuf *cmdbuf, uint32_t *fence);
int ff_nvtegra_channel_set_submit_timeout(AVNVTegraChannel *channel, uint32_t timeout_ms);

int ff_nvtegra_syncpt_wait(AVNVTegraChannel *channel, uint32_t threshold, int32_t timeout);

int ff_nvtegra_map_allocate(AVNVTegraMap *map, uint32_t size, uint32_t align, uint32_t flags);
int ff_nvtegra_map_free(AVNVTegraMap *map);
int ff_nvtegra_map_from_va(AVNVTegraMap *map, void *mem, uint32_t size, uint32_t align, uint32_t flags);
int ff_nvtegra_map_close(AVNVTegraMap *map);
int ff_nvtegra_map_map(AVNVTegraMap *map);
int ff_nvtegra_map_unmap(AVNVTegraMap *map);
int ff_nvtegra_map_realloc(AVNVTegraMap *map, uint32_t size, uint32_t align, uint32_t flags);

static inline int ff_nvtegra_map_create(AVNVTegraMap *map, uint32_t size, uint32_t align, uint32_t flags) {
    int err;

    err = ff_nvtegra_map_allocate(map, size, align, flags);
    if (err < 0)
        return err;

    return ff_nvtegra_map_map(map);
}

static inline int ff_nvtegra_map_destroy(AVNVTegraMap *map) {
    int err;

    err = ff_nvtegra_map_unmap(map);
    if (err < 0)
        return err;

    return ff_nvtegra_map_free(map);
}

int ff_nvtegra_cmdbuf_init(AVNVTegraCmdbuf *cmdbuf);
int ff_nvtegra_cmdbuf_deinit(AVNVTegraCmdbuf *cmdbuf);
int ff_nvtegra_cmdbuf_add_memory(AVNVTegraCmdbuf *cmdbuf, AVNVTegraMap *map, uint32_t offset, uint32_t size);
int ff_nvtegra_cmdbuf_clear(AVNVTegraCmdbuf *cmdbuf);
int ff_nvtegra_cmdbuf_begin(AVNVTegraCmdbuf *cmdbuf, uint32_t class_id);
int ff_nvtegra_cmdbuf_end(AVNVTegraCmdbuf *cmdbuf);
int ff_nvtegra_cmdbuf_push_word(AVNVTegraCmdbuf *cmdbuf, uint32_t word);
int ff_nvtegra_cmdbuf_push_value(AVNVTegraCmdbuf *cmdbuf, uint32_t offset, uint32_t word);
int ff_nvtegra_cmdbuf_push_reloc(AVNVTegraCmdbuf *cmdbuf, uint32_t offset, AVNVTegraMap *target, uint32_t target_offset,
                                 int reloc_type, int shift);
int ff_nvtegra_cmdbuf_push_wait(AVNVTegraCmdbuf *cmdbuf, uint32_t syncpt, uint32_t fence);
int ff_nvtegra_cmdbuf_add_syncpt_incr(AVNVTegraCmdbuf *cmdbuf, uint32_t syncpt, uint32_t
                                      num_incrs, uint32_t fence);
int ff_nvtegra_cmdbuf_add_waitchk(AVNVTegraCmdbuf *cmdbuf, uint32_t syncpt, uint32_t fence);

static inline uint32_t ff_nvtegra_map_get_handle(AVNVTegraMap *map) {
#ifndef __SWITCH__
    return map->handle;
#else
    return map->map.handle;
#endif
}

static inline void *ff_nvtegra_map_get_addr(AVNVTegraMap *map) {
#ifndef __SWITCH__
    return map->cpu_addr;
#else
    return map->map.cpu_addr;
#endif
}

static inline uint32_t ff_nvtegra_map_get_size(AVNVTegraMap *map) {
#ifndef __SWITCH__
    return map->size;
#else
    return map->map.size;
#endif
}

/* Addresses are shifted by 8 bits in the command buffer, requiring an alignment to 256 */
#define FF_NVTEGRA_MAP_ALIGN (1 << 8)

#define FF_NVTEGRA_VALUE(offset, field, value)                                                    \
    ((value &                                                                                     \
    ((uint32_t)((UINT64_C(1) << ((1?offset ## _ ## field) - (0?offset ## _ ## field) + 1)) - 1))) \
    << (0?offset ## _ ## field))

#define FF_NVTEGRA_ENUM(offset, field, value)                                                     \
    ((offset ## _ ## field ## _ ## value &                                                        \
    ((uint32_t)((UINT64_C(1) << ((1?offset ## _ ## field) - (0?offset ## _ ## field) + 1)) - 1))) \
    << (0?offset ## _ ## field))

#define FF_NVTEGRA_PUSH_VALUE(cmdbuf, offset, value) ({                                  \
    int _err = ff_nvtegra_cmdbuf_push_value(cmdbuf, (offset) / sizeof(uint32_t), value); \
    if (_err < 0)                                                                        \
        return _err;                                                                     \
})

#define FF_NVTEGRA_PUSH_RELOC(cmdbuf, offset, target, target_offset, type) ({    \
    int _err = ff_nvtegra_cmdbuf_push_reloc(cmdbuf, (offset) / sizeof(uint32_t), \
                                        target, target_offset, type, 8);         \
    if (_err < 0)                                                                \
        return _err;                                                             \
})

#endif /* AVUTIL_NVTEGRA_H */
