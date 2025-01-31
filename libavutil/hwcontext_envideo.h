/*
 * Copyright (c) 2025 averne <averne381@gmail.com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef AVUTIL_HWCONTEXT_ENVIDEO_H
#define AVUTIL_HWCONTEXT_ENVIDEO_H

#include <envideo/envideo.h>

#include "hwcontext.h"

typedef struct AVEnvideoDeviceContext {
    /**
     * Pointer to the root device object
     */
    EnvideoDevice *device;
} AVEnvideoDeviceContext;

typedef struct AVEnvideoFrame {
    /**
     * Pointer to the object representing the backing memory-mapped buffer
     */
    EnvideoMap *map;

    /**
     * The fence associated with the last operation on the frame
     */
    EnvideoFence fence;

    /**
     * Whether the frame data layout is pitch linear (row-based)
     * or tiled (hardware-specific)
     */
    bool is_pitch;
} AVEnvideoFrame;

/**
 * Helper to retrieve a map object from the corresponding frame
 */
static inline EnvideoMap *av_envideo_frame_get_fbuf_map(const AVFrame *frame) {
    return ((AVEnvideoFrame *)frame->buf[0]->data)->map;
}

typedef struct AVEnvideoJobPool {
    /**
     * Pool object for job allocation
     */
    AVBufferPool *pool;

    /**
     * Hardware device associated with this job pool
     */
    EnvideoDevice *device;

    /**
     * Hardware channel the jobs will be submitted to
     */
    EnvideoChannel *channel;

    /**
     * Total size of the input memory-mapped buffer
     */
    size_t input_map_size;

    /**
     * Alignment of the input memory-mapped buffer
     */
    size_t input_map_align;

    /**
     * Flags for creation of the input memory-mapped buffer
     */
    EnvideoMapFlags input_map_flags;

    /**
     * Whether a new job object was just allocated
     */
    bool new_job;

    /**
     * Offset of the command data within the input map
     */
    off_t cmdbuf_off;

    /**
     * Maximum memory usable by the command buffer
     */
    size_t max_cmdbuf_size;
} AVEnvideoJobPool;

typedef struct AVEnvideoJob {
    /**
     * Memory-mapped buffer for command buffers, metadata structures, ...
     */
    EnvideoMap *input_map;

    /**
     * Object for command recording
     */
    EnvideoCmdbuf *cmdbuf;
} AVEnvideoJob;

/**
 * Job allocation and submission routines
 */
int av_envideo_job_pool_init(AVEnvideoJobPool *pool, EnvideoDevice *device, EnvideoChannel *channel,
                             size_t input_map_size, size_t input_map_align, EnvideoMapFlags input_map_flags,
                             off_t cmdbuf_off, size_t max_cmdbuf_size);
int av_envideo_job_pool_uninit(AVEnvideoJobPool *pool);
AVBufferRef *av_envideo_job_pool_get(AVEnvideoJobPool *pool, bool *new_buffer);
int av_envideo_job_realloc(AVEnvideoJobPool *pool, AVEnvideoJob *job, size_t size, size_t align);

#endif /* AVUTIL_HWCONTEXT_ENVIDEO_H */
