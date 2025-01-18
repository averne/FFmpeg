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

#ifndef AVCODEC_ENVIDEO_DECODE_H
#define AVCODEC_ENVIDEO_DECODE_H

#include <stdbool.h>

#include <envideo/envideo.h>

#include <envideo/classes/nvmisc.h>
#include <envideo/classes/clc9b0.h>
#include <envideo/classes/cle7d0.h>
#include <envideo/classes/nvdec_drv.h>
#include <envideo/classes/nvjpg_drv.h>

#include "avcodec.h"
#include "libavutil/hwcontext_envideo.h"

typedef struct FFEnvideoDecodeContext FFEnvideoDecodeContext;

typedef struct FFEnvideoOperation {
    AVBufferRef *input_map_ref;
    EnvideoFence fence;
} FFEnvideoOperation;

typedef struct FFEnvideoDecodeFrame {
    FFEnvideoDecodeContext *ctx;
    FFEnvideoOperation operation;
    bool in_flight;
} FFEnvideoDecodeFrame;

typedef struct FFEnvideoDecodeContext {
    uint64_t frame_idx;

    AVBufferRef *hw_device_ref;
    AVBufferPool *decoder_pool;
    FFEnvideoOperation *operations;
    size_t num_operations;

    bool is_nvjpg;
    EnvideoChannel *channel;
    EnvideoCmdbuf  *cmdbuf;

    uint32_t pic_setup_off, status_off, cmdbuf_off,
             bitstream_off, slice_offsets_off;
    uint32_t input_map_size;
    uint32_t max_cmdbuf_size, max_bitstream_size, max_num_slices;

    uint32_t num_slices;
    uint32_t bitstream_len;

    bool new_input_buffer;
} FFEnvideoDecodeContext;

#define FF_ENVIDEO_PUSH_VALUE(cmdbuf, off, val) ({                      \
    int _err_ = envideo_cmdbuf_push_value(cmdbuf, off, val);            \
    if (_err_ < 0)                                                      \
        return _err_;                                                   \
})

#define FF_ENVIDEO_PUSH_RELOC_TYPE(cmdbuf, off, map, map_off, type) ({  \
    int _err_ = envideo_cmdbuf_push_reloc(cmdbuf, off, map, map_off,    \
                                          type, 8);                     \
    if (_err_ < 0)                                                      \
        return _err_;                                                   \
})

#define FF_ENVIDEO_PUSH_RELOC(cmdbuf, off, map, map_off)       FF_ENVIDEO_PUSH_RELOC_TYPE(cmdbuf, off, map, map_off, EnvideoRelocType_Default)
#define FF_ENVIDEO_PUSH_RELOC_PITCH(cmdbuf, off, map, map_off) FF_ENVIDEO_PUSH_RELOC_TYPE(cmdbuf, off, map, map_off, EnvideoRelocType_Pitch)
#define FF_ENVIDEO_PUSH_RELOC_TILED(cmdbuf, off, map, map_off) FF_ENVIDEO_PUSH_RELOC_TYPE(cmdbuf, off, map, map_off, EnvideoRelocType_Tiled)

static inline size_t ff_envideo_decode_pick_bitstream_buffer_size(AVCodecContext *avctx) {
    /**
     * Official software uses a static map of a predetermined size, usually around 0x600000 (6MiB).
     * Our implementation supports dynamically resizing the input map, so be less conservative.
     */
    if ((avctx->coded_width >= 3840) || (avctx->coded_height >= 2160))  /* 4k */
        return 0x100000;                                                /* 1MiB */
    if ((avctx->coded_width >= 1920) || (avctx->coded_height >= 1080))  /* 1080p */
        return 0x40000;                                                 /* 256KiB */
    else
        return 0x10000;                                                 /* 64KiB */
}

static inline AVFrame *ff_envideo_safe_get_ref(AVFrame *ref, AVFrame *fallback) {
    return (ref && ref->private_ref) ? ref : fallback;
}

int ff_envideo_decode_init(AVCodecContext *avctx, FFEnvideoDecodeContext *ctx);
int ff_envideo_decode_uninit(AVCodecContext *avctx, FFEnvideoDecodeContext *ctx);
int ff_envideo_start_frame(AVCodecContext *avctx, AVFrame *frame, FFEnvideoDecodeContext *ctx);
int ff_envideo_decode_slice(AVCodecContext *avctx, AVFrame *frame,
                            const uint8_t *buf, uint32_t buf_size, bool add_startcode);
int ff_envideo_end_frame(AVCodecContext *avctx, AVFrame *frame, FFEnvideoDecodeContext *ctx,
                         const uint8_t *end_sequence, int end_sequence_size);

int ff_envideo_wait_decode(void *logctx, AVFrame *frame);

int ff_envideo_frame_params(AVCodecContext *avctx, AVBufferRef *hw_frames_ctx);

#endif /* AVCODEC_ENVIDEO_DECODE_H */
