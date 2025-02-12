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

#ifndef AVFILTER_ENVIDEO_VIC_H
#define AVFILTER_ENVIDEO_VIC_H

#include "avfilter.h"

#include "libavutil/mem.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_envideo.h"

#include <envideo/classes/nvmisc.h>
#include <envideo/classes/clb0b6.h>
#include <envideo/classes/vic_drv.h>

typedef struct FFEnvideoOperation {
    AVBufferRef *job_ref;
    EnvideoFence fence;
} FFEnvideoOperation;

typedef struct FFEnvideoVppContext {
    const AVClass *class;

    AVBufferRef *hw_device_ref;
    EnvideoChannel *channel;
    AVEnvideoJobPool pool;
    FFEnvideoOperation *operations;
    size_t num_operations;

    off_t vic_setup_off, vic_cmdbuf_off;
    size_t vic_map_size;
    size_t max_cmdbuf_size;

    enum AVPixelFormat input_format, output_format;
    int output_width;
    int output_height;
} FFEnvideoVppContext;

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

int ff_envideo_vpp_ctx_init(AVFilterContext *avctx);
void ff_envideo_vpp_ctx_uninit(AVFilterContext *avctx);

int ff_envideo_vpp_config_input(AVFilterLink *link);
int ff_envideo_vpp_config_output(AVFilterLink *link);

int ff_envideo_vpp_init_config(FFEnvideoVppContext *ctx, VicConfigStruct *config, AVFrame *output,
                               AVFrame **input, int num_input_frames);

#endif /* AVFILTER_ENVIDEO_VIC_H */
