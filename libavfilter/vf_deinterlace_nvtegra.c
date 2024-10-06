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

#include "config_components.h"

#include <stdbool.h>

#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/nvtegra_host1x.h"

#include "avfilter.h"
#include "video.h"

#include "nvtegra_vpp.h"

/* Deinterlacing min/max/default values */
#define DEINTERLACE_MODE_MIN     NVB0B6_DXVAHD_DEINTERLACE_MODE_PRIVATE_WEAVE
#define DEINTERLACE_MODE_MAX     NVB0B6_DXVAHD_DEINTERLACE_MODE_PRIVATE_BOB
#define DEINTERLACE_MODE_DEFAULT NVB0B6_DXVAHD_DEINTERLACE_MODE_PRIVATE_BOB

typedef struct NVTegraDeinterlaceContext {
    FFNVTegraVppContext core;

    int deint_mode;
} NVTegraDeinterlaceContext;

static inline int nvtegra_map_frame_format(AVFrame *f) {
    if (!(f->flags & AV_FRAME_FLAG_INTERLACED))
        return NVB0B6_DXVAHD_FRAME_FORMAT_PROGRESSIVE;
    else if (f->flags & AV_FRAME_FLAG_TOP_FIELD_FIRST)
        return NVB0B6_DXVAHD_FRAME_FORMAT_INTERLACED_TOP_FIELD_FIRST;
    else
        return NVB0B6_DXVAHD_FRAME_FORMAT_INTERLACED_BOTTOM_FIELD_FIRST;
}

static int nvtegra_deinterlace_prepare_config(NVTegraDeinterlaceContext *ctx, VicConfigStruct *config, AVFrame *in) {
    VicSlotStruct *slot = &config->slotStruct[0];

    slot->slotConfig.FrameFormat     = nvtegra_map_frame_format(in);
    slot->slotConfig.DeinterlaceMode = ctx->deint_mode;

    return 0;
}

static int nvtegra_deinterlace_prepare_cmdbuf(NVTegraDeinterlaceContext *ctx, AVNVTegraJobPool *pool,
                                              AVNVTegraJob *job, const AVFrame *in, const AVFrame *out)
{
    AVNVTegraCmdbuf *cmdbuf = &job->cmdbuf;

    const AVPixFmtDescriptor *input_desc, *output_desc;
    AVNVTegraMap *input_map, *output_map;
    int reloc_type, i, err;

    input_desc  = av_pix_fmt_desc_get(ctx->core.input_format);
    output_desc = av_pix_fmt_desc_get(ctx->core.output_format);

    input_map  = av_nvtegra_frame_get_fbuf_map(in);
    output_map = av_nvtegra_frame_get_fbuf_map(out);

    err = av_nvtegra_cmdbuf_begin(cmdbuf, HOST1X_CLASS_VIC);
    if (err < 0)
        return err;

    AV_NVTEGRA_PUSH_VALUE(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS,
                          AV_NVTEGRA_VALUE(NVB0B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS, CONFIG_STRUCT_SIZE, sizeof(VicConfigStruct) >> 4) |
                          AV_NVTEGRA_VALUE(NVB0B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS, GPTIMER_ON,         1)                            |
                          AV_NVTEGRA_VALUE(NVB0B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS, FALCON_CONTROL,     1));
    AV_NVTEGRA_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_CONFIG_STRUCT_OFFSET,
                          &job->input_map,  ctx->core.vic_setup_off, NVHOST_RELOC_TYPE_DEFAULT);

    reloc_type = !input_map->is_linear  ? NVHOST_RELOC_TYPE_BLOCK_LINEAR : NVHOST_RELOC_TYPE_PITCH_LINEAR;
    for (i = 0; i < input_desc->nb_components; ++i) {
        AV_NVTEGRA_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_SURFACE0_LUMA_OFFSET(0)    + i * sizeof(uint32_t),
                              input_map,  in->data[i]  - in->data[0],  reloc_type);
    }

    reloc_type = !output_map->is_linear ? NVHOST_RELOC_TYPE_BLOCK_LINEAR : NVHOST_RELOC_TYPE_PITCH_LINEAR;
    for (i = 0; i < output_desc->nb_components; ++i) {
        AV_NVTEGRA_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_OUTPUT_SURFACE_LUMA_OFFSET + i * sizeof(uint32_t),
                              output_map, out->data[i] - out->data[0], reloc_type);
    }

    AV_NVTEGRA_PUSH_VALUE(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_EXECUTE,
                          AV_NVTEGRA_ENUM(NVB0B6_VIDEO_COMPOSITOR_EXECUTE, AWAKEN, ENABLE));

    err = av_nvtegra_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    err = av_nvtegra_cmdbuf_begin(cmdbuf, HOST1X_CLASS_VIC);
    if (err < 0)
        return err;

    err = av_nvtegra_cmdbuf_add_syncpt_incr(cmdbuf, pool->channel->syncpt, 0);
    if (err < 0)
        return err;

    err = av_nvtegra_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    return 0;
}

static int nvtegra_deinterlace_filter_frame(AVFilterLink *link, AVFrame *in) {
    AVFilterContext         *avctx = link->dst;
    NVTegraDeinterlaceContext *ctx = avctx->priv;
    AVFilterLink          *outlink = avctx->outputs[0];

    AVBufferRef *job_ref;
    AVNVTegraJob *job;
    AVFrame *out = NULL;
    VicConfigStruct *config;
    int err;

    job_ref = av_nvtegra_job_pool_get(&ctx->core.pool);
    if (!job_ref) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    job    = (AVNVTegraJob *)job_ref->data;
    config = (VicConfigStruct *)((uint8_t *)av_nvtegra_map_get_addr(&job->input_map) + ctx->core.vic_setup_off);

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    err = av_frame_copy_props(out, in);
    if (err < 0)
        goto fail;

    err = ff_nvtegra_vpp_init_config(&ctx->core, config, out, &in, 1);
    if (err < 0)
        goto fail;

    err = nvtegra_deinterlace_prepare_config(ctx, config, in);
    if (err < 0)
        goto fail;

    err = av_nvtegra_cmdbuf_clear(&job->cmdbuf);
    if (err < 0)
        return err;

    err = nvtegra_deinterlace_prepare_cmdbuf(ctx, &ctx->core.pool, job, in, out);
    if (err < 0)
        goto fail;

    err = av_nvtegra_job_submit(&ctx->core.pool, job);
    if (err < 0)
        goto fail;

    err = av_nvtegra_job_wait(&ctx->core.pool, job, -1);
    if (err < 0)
        goto fail;

    av_buffer_unref(&job_ref);

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);

fail:
    av_buffer_unref(&job_ref);
    av_frame_free(&in);
    av_frame_free(&out);
    return err;
}

#define SOFFSET(x) offsetof(NVTegraDeinterlaceContext, x)
#define FLAGS (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_RUNTIME_PARAM | AV_OPT_FLAG_FILTERING_PARAM)

static const AVOption nvtegra_deinterlace_options[] = {
    { "mode", "deinterlace algorithm", SOFFSET(deint_mode), AV_OPT_TYPE_INT,
      { .i64 = DEINTERLACE_MODE_DEFAULT }, DEINTERLACE_MODE_MIN, DEINTERLACE_MODE_MAX, FLAGS, "mode" },
    { "weave", "use the weave algorithm", 0, AV_OPT_TYPE_CONST,
      { .i64 = NVB0B6_DXVAHD_DEINTERLACE_MODE_PRIVATE_WEAVE }, 0, 0, FLAGS, "mode" },
    { "bob",   "use the bob algorithm",   0, AV_OPT_TYPE_CONST,
      { .i64 = NVB0B6_DXVAHD_DEINTERLACE_MODE_PRIVATE_BOB },   0, 0, FLAGS, "mode" },

    { NULL },
};

AVFILTER_DEFINE_CLASS(nvtegra_deinterlace);

static const AVFilterPad nvtegra_deinterlace_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = &nvtegra_deinterlace_filter_frame,
        .config_props = &ff_nvtegra_vpp_config_input,
    },
};

static const AVFilterPad nvtegra_deinterlace_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = &ff_nvtegra_vpp_config_output,
    },
};

const AVFilter ff_vf_deinterlace_nvtegra = {
    .name            = "deinterlace_nvtegra",
    .description     = NULL_IF_CONFIG_SMALL("NVTegra accelerated deinterlacing"),
    .priv_size       = sizeof(NVTegraDeinterlaceContext),
    .init            = &ff_nvtegra_vpp_ctx_init,
    .uninit          = &ff_nvtegra_vpp_ctx_uninit,
    .process_command = &ff_filter_process_command,
    FILTER_INPUTS(nvtegra_deinterlace_inputs),
    FILTER_OUTPUTS(nvtegra_deinterlace_outputs),
    FILTER_SINGLE_PIXFMT(AV_PIX_FMT_NVTEGRA),
    .priv_class      = &nvtegra_deinterlace_class,
    .flags_internal  = FF_FILTER_FLAG_HWFRAME_AWARE,
};
