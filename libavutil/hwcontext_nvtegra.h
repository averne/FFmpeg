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

#ifndef AVUTIL_HWCONTEXT_NVTEGRA_H
#define AVUTIL_HWCONTEXT_NVTEGRA_H

#include <stdint.h>

#include "hwcontext.h"
#include "frame.h"
#include "pixfmt.h"

#include "nvtegra.h"

/**
 * @file
 * API-specific header for AV_HWDEVICE_TYPE_NVTEGRA.
 *
 * For user-allocated pools, AVHWFramesContext.pool must return AVBufferRefs
 * with the data pointer set to an AVNVTegraMap.
 */

typedef struct AVNVTegraDeviceContext {
    /*
     * Hardware multimedia engines
     */
    AVNVTegraChannel nvdec_channel, nvjpg_channel, vic_channel;
    bool has_nvdec, has_nvjpg;
} AVNVTegraDeviceContext;

static inline AVNVTegraMap *ff_nvtegra_frame_get_fbuf_map(const AVFrame *frame) {
    return (AVNVTegraMap *)frame->buf[0]->data;
}

/*
 * Helper to retrieve a map object from the corresponding frame
 */
int ff_nvtegra_map_vic_pic_fmt(enum AVPixelFormat fmt);

/*
 * Dynamic frequency scaling routines
 */
int ff_nvtegra_dfs_init(AVHWDeviceContext *ctx, AVNVTegraChannel *channel, int width, int height, double framerate_hz);
int ff_nvtegra_dfs_update(AVHWDeviceContext *ctx, AVNVTegraChannel *channel, int bitstream_len, int decode_cycles);
int ff_nvtegra_dfs_uninit(AVHWDeviceContext *ctx, AVNVTegraChannel *channel);

#endif /* AVUTIL_HWCONTEXT_NVTEGRA_H */
