/*
 * Copyright (c) 2024 averne <averne381@gmail.com>
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

#ifndef AVFILTER_NVTEGRA_VIC_H
#define AVFILTER_NVTEGRA_VIC_H

#include "avfilter.h"

#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_nvtegra.h"
#include "libavutil/clb0b6.h"
#include "libavutil/vic_drv.h"

typedef struct FFNVTegraVppContext {
    const AVClass *class;

    AVNVTegraJobPool pool;
    off_t vic_setup_off, vic_cmdbuf_off;
    size_t vic_map_size;
    size_t max_cmdbuf_size;

    enum AVPixelFormat input_format, output_format;
    int output_width;
    int output_height;
} FFNVTegraVppContext;

int ff_nvtegra_vpp_ctx_init(AVFilterContext *avctx);
void ff_nvtegra_vpp_ctx_uninit(AVFilterContext *avctx);

int ff_nvtegra_vpp_config_input(AVFilterLink *link);
int ff_nvtegra_vpp_config_output(AVFilterLink *link);

int ff_nvtegra_vpp_init_config(FFNVTegraVppContext *ctx, VicConfigStruct *config, AVFrame *output,
                               AVFrame **input, int num_input_frames);

#endif /* AVFILTER_NVTEGRA_VIC_H */
