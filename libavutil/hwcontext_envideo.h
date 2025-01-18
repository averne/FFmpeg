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
} AVEnvideoFrame;

/**
 * Helper to retrieve a map object from the corresponding frame
 */
static inline EnvideoMap *av_envideo_frame_get_fbuf_map(const AVFrame *frame) {
    return ((AVEnvideoFrame *)frame->buf[0]->data)->map;
}

#endif /* AVUTIL_HWCONTEXT_ENVIDEO_H */
