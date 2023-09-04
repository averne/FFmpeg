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

#ifndef AVUTIL_HWCONTEXT_TX1_H
#define AVUTIL_HWCONTEXT_TX1_H

#include <stdint.h>

#include "hwcontext.h"
#include "frame.h"
#include "pixfmt.h"

#include "tx1.h"

typedef struct AVTX1DeviceContext {
    AVTX1Channel nvdec_channel, nvjpg_channel, vic_channel;

    AVTX1Map vic_map;
    uint32_t vic_setup_off, vic_cmdbuf_off, vic_filter_off;
    uint32_t vic_max_cmdbuf_size;

    AVTX1Cmdbuf vic_cmdbuf;

    uint32_t dfs_lowcorner;

    double dfs_decode_cycles_ema;
    double dfs_decode_ema_damping;

    int *dfs_bitrate_samples;
    int dfs_cur_sample, dfs_num_samples;
    int64_t dfs_sampling_start_ts;
} AVTX1DeviceContext;

static inline AVTX1Map *ff_tx1_frame_get_fbuf_map(const AVFrame *frame) {
    return (AVTX1Map *)frame->buf[0]->data;
}

int ff_tx1_map_vic_pic_fmt(enum AVPixelFormat fmt);

int ff_tx1_dfs_init(AVHWDeviceContext *ctx, AVTX1Channel *channel, int width, int height, double framerate_hz);
int ff_tx1_dfs_update(AVHWDeviceContext *ctx, AVTX1Channel *channel, int bitstream_len, int decode_cycles);
int ff_tx1_dfs_uninit(AVHWDeviceContext *ctx, AVTX1Channel *channel);

#endif /* AVUTIL_HWCONTEXT_TX1_H */
