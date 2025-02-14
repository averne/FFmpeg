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

#include "config_components.h"

#include <stdbool.h>

#include "libavutil/internal.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"

#include "avfilter.h"
#include "video.h"

#include "envideo_vpp.h"

/* Deinterlacing min/max/default values */
#define DEINTERLACE_MODE_MIN     NVB0B6_DXVAHD_DEINTERLACE_MODE_PRIVATE_WEAVE
#define DEINTERLACE_MODE_MAX     NVB0B6_DXVAHD_DEINTERLACE_MODE_PRIVATE_BOB
#define DEINTERLACE_MODE_DEFAULT NVB0B6_DXVAHD_DEINTERLACE_MODE_PRIVATE_BOB

typedef struct EnvideoDeinterlaceContext {
    FFEnvideoVppContext core;

    int deint_mode;
} EnvideoDeinterlaceContext;

static inline int envideo_map_frame_format(AVFrame *f) {
    if (!(f->flags & AV_FRAME_FLAG_INTERLACED))
        return NVB0B6_DXVAHD_FRAME_FORMAT_PROGRESSIVE;
    else if (f->flags & AV_FRAME_FLAG_TOP_FIELD_FIRST)
        return NVB0B6_DXVAHD_FRAME_FORMAT_INTERLACED_TOP_FIELD_FIRST;
    else
        return NVB0B6_DXVAHD_FRAME_FORMAT_INTERLACED_BOTTOM_FIELD_FIRST;
}

static int envideo_deinterlace_prepare_config(EnvideoDeinterlaceContext *ctx, VicConfigStruct *config, AVFrame *in) {
    VicSlotStruct *slot = &config->slotStruct[0];

    slot->slotConfig.FrameFormat     = envideo_map_frame_format(in);
    slot->slotConfig.DeinterlaceMode = ctx->deint_mode;

    return 0;
}

static int envideo_deinterlace_prepare_cmdbuf(EnvideoDeinterlaceContext *ctx, AVEnvideoJob *job,
                                              const AVFrame *in, const AVFrame *out)
{
    EnvideoCmdbuf *cmdbuf = job->cmdbuf;

    const AVPixFmtDescriptor *input_desc, *output_desc;
    AVEnvideoFrame *input_frame, *output_frame;
    EnvideoMap *input_map, *output_map;
    int reloc_type, i, err;

    input_desc  = av_pix_fmt_desc_get(ctx->core.input_format);
    output_desc = av_pix_fmt_desc_get(ctx->core.output_format);

    input_frame = (AVEnvideoFrame *)in->buf[0]->data, output_frame = (AVEnvideoFrame *)out->buf[0]->data;
    input_map = input_frame->map, output_map = output_frame->map;

    err = envideo_cmdbuf_begin(cmdbuf, EnvideoEngine_Host);
    if (err < 0)
        return err;

    err = envideo_cmdbuf_wait_fence(cmdbuf, input_frame->fence);
    if (err < 0)
        return err;

    err = envideo_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    err = envideo_cmdbuf_begin(cmdbuf, EnvideoEngine_Vic);
    if (err < 0)
        return err;

    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS,
                          DRF_NUM(B0B6_VIDEO_COMPOSITOR, _SET_CONTROL_PARAMS, _CONFIG_STRUCT_SIZE, sizeof(VicConfigStruct) >> 4) |
                          DRF_NUM(B0B6_VIDEO_COMPOSITOR, _SET_CONTROL_PARAMS, _GPTIMER_ON,     1)                                |
                          DRF_NUM(B0B6_VIDEO_COMPOSITOR, _SET_CONTROL_PARAMS, _FALCON_CONTROL, 1));
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_CONFIG_STRUCT_OFFSET, job->input_map, ctx->core.vic_setup_off);

    reloc_type = !input_frame->is_pitch  ? EnvideoRelocType_Tiled : EnvideoRelocType_Pitch;
    for (i = 0; i < input_desc->nb_components; ++i) {
        FF_ENVIDEO_PUSH_RELOC_TYPE(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_SURFACE0_LUMA_OFFSET(0)    + i * sizeof(uint32_t),
                                   input_map,  in->data[i]  - in->data[0],  reloc_type);
    }

    reloc_type = !output_frame->is_pitch ? EnvideoRelocType_Tiled : EnvideoRelocType_Pitch;
    for (i = 0; i < output_desc->nb_components; ++i) {
        FF_ENVIDEO_PUSH_RELOC_TYPE(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_OUTPUT_SURFACE_LUMA_OFFSET + i * sizeof(uint32_t),
                                   output_map, out->data[i] - out->data[0], reloc_type);
    }

    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_EXECUTE,
                          DRF_DEF(B0B6_VIDEO_COMPOSITOR, _EXECUTE, _AWAKEN, _ENABLE));

    err = envideo_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    return 0;
}

static int envideo_deinterlace_filter_frame(AVFilterLink *link, AVFrame *in) {
    AVFilterContext             *avctx = link->dst;
    EnvideoDeinterlaceContext     *ctx = avctx->priv;
    FFEnvideoVppContext          *core = &ctx->core;
    AVHWDeviceContext   *hw_device_ctx = (AVHWDeviceContext *)core->hw_device_ref->data;
    AVEnvideoDeviceContext *device_ctx = hw_device_ctx->hwctx;
    AVFilterLink              *outlink = avctx->outputs[0];

    AVFrame *out;
    AVEnvideoFrame *evframe;
    EnvideoMap *map;
    FFEnvideoOperation *op;
    AVEnvideoJob *job;
    VicConfigStruct *config;
    bool new_buffer, is_done;
    int i, err;

    evframe = (AVEnvideoFrame *)in->buf[0]->data;
    err = envideo_map_pin(evframe->map, core->channel);
    if (err < 0)
        return err;

    for (i = 0; i < core->num_operations; ++i) {
        op = &core->operations[i];
        if (!op->job_ref)
            continue;

        err = envideo_fence_poll(device_ctx->device, op->fence, &is_done);
        if (err < 0 || !is_done)
            continue;

        av_buffer_unref(&op->job_ref);
    }

    for (i = 0; i < core->num_operations; ++i) {
        if (!core->operations[i].job_ref)
            break;
    }

    if (i == core->num_operations) {
        core->operations = av_realloc_array(core->operations, core->num_operations + 1, sizeof(FFEnvideoOperation));
        if (!core->operations)
            return AVERROR(ENOMEM);

        core->operations[core->num_operations++] = (FFEnvideoOperation){0};
    }

    op = &core->operations[i];

    op->job_ref = av_envideo_job_pool_get(&core->pool, &new_buffer);
    if (!op->job_ref) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    job = (AVEnvideoJob *)op->job_ref->data;
    map = job->input_map;

    config = (VicConfigStruct *)((uint8_t *)envideo_map_get_cpu_addr(job->input_map) + ctx->core.vic_setup_off);

    if (new_buffer) {
        err = envideo_map_pin(map, core->channel);
        if (err < 0)
            goto fail;
    }

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    err = av_frame_copy_props(out, in);
    if (err < 0)
        goto fail;

    evframe = (AVEnvideoFrame *)out->buf[0]->data;
    err = envideo_map_pin(evframe->map, core->channel);
    if (err < 0)
        return err;

    err = ff_envideo_vpp_init_config(&ctx->core, config, out, &in, 1);
    if (err < 0)
        goto fail;

    err = envideo_deinterlace_prepare_config(ctx, config, in);
    if (err < 0)
        goto fail;

    err = envideo_cmdbuf_clear(job->cmdbuf);
    if (err < 0)
        return err;

    err = envideo_deinterlace_prepare_cmdbuf(ctx, job, in, out);
    if (err < 0)
        goto fail;

    err = envideo_channel_submit(core->channel, job->cmdbuf, &op->fence);
    if (err < 0)
        goto fail;

    evframe->fence = op->fence;

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);

fail:
    av_buffer_unref(&op->job_ref);
    av_frame_free(&in);
    av_frame_free(&out);
    return err;
}

#define SOFFSET(x) offsetof(EnvideoDeinterlaceContext, x)
#define FLAGS (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_RUNTIME_PARAM | AV_OPT_FLAG_FILTERING_PARAM)

static const AVOption envideo_deinterlace_options[] = {
    { "mode", "deinterlace algorithm", SOFFSET(deint_mode), AV_OPT_TYPE_INT,
      { .i64 = DEINTERLACE_MODE_DEFAULT }, DEINTERLACE_MODE_MIN, DEINTERLACE_MODE_MAX, FLAGS, "mode" },
    { "weave", "use the weave algorithm", 0, AV_OPT_TYPE_CONST,
      { .i64 = NVB0B6_DXVAHD_DEINTERLACE_MODE_PRIVATE_WEAVE }, 0, 0, FLAGS, "mode" },
    { "bob",   "use the bob algorithm",   0, AV_OPT_TYPE_CONST,
      { .i64 = NVB0B6_DXVAHD_DEINTERLACE_MODE_PRIVATE_BOB },   0, 0, FLAGS, "mode" },

    { NULL },
};

AVFILTER_DEFINE_CLASS(envideo_deinterlace);

static const AVFilterPad envideo_deinterlace_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = &envideo_deinterlace_filter_frame,
        .config_props = &ff_envideo_vpp_config_input,
    },
};

static const AVFilterPad envideo_deinterlace_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = &ff_envideo_vpp_config_output,
    },
};

const AVFilter ff_vf_deinterlace_envideo = {
    .name            = "deinterlace_envideo",
    .description     = NULL_IF_CONFIG_SMALL("Envideo accelerated deinterlacing"),
    .priv_size       = sizeof(EnvideoDeinterlaceContext),
    .init            = &ff_envideo_vpp_ctx_init,
    .uninit          = &ff_envideo_vpp_ctx_uninit,
    .process_command = &ff_filter_process_command,
    FILTER_INPUTS(envideo_deinterlace_inputs),
    FILTER_OUTPUTS(envideo_deinterlace_outputs),
    FILTER_SINGLE_PIXFMT(AV_PIX_FMT_ENVIDEO),
    .priv_class      = &envideo_deinterlace_class,
    .flags_internal  = FF_FILTER_FLAG_HWFRAME_AWARE,
};
