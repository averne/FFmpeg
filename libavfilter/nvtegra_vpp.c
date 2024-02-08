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

#include "libavutil/pixdesc.h"

#include "nvtegra_vpp.h"

int ff_nvtegra_vpp_ctx_init(AVFilterContext *avctx) {
    FFNVTegraVppContext *ctx = avctx->priv;

    av_log(avctx, AV_LOG_INFO, "%s\n", __PRETTY_FUNCTION__);

    if (!ctx->vic_map_size) {
        ctx->vic_setup_off  = 0;
        ctx->vic_cmdbuf_off = FFALIGN(ctx->vic_setup_off  + sizeof(VicConfigStruct),
                                      AV_NVTEGRA_MAP_ALIGN);
        ctx->vic_map_size   = FFALIGN(ctx->vic_cmdbuf_off + AV_NVTEGRA_MAP_ALIGN,
                                      0x1000);

        ctx->max_cmdbuf_size = ctx->vic_map_size - ctx->vic_cmdbuf_off;
    }

    return 0;
}

void ff_nvtegra_vpp_ctx_uninit(AVFilterContext *avctx) {
    FFNVTegraVppContext *ctx = avctx->priv;

    av_nvtegra_job_pool_uninit(&ctx->pool);

    av_buffer_unref(&ctx->device_ref);
}

int ff_nvtegra_vpp_config_input(AVFilterLink *inlink) {
    AVFilterContext   *avctx = inlink->dst;
    FFNVTegraVppContext *ctx = avctx->priv;

    AVHWFramesContext *input_frames_ctx;
    AVNVTegraDeviceContext *device_ctx;
    const AVPixFmtDescriptor *desc;
    int i, err;

    av_log(avctx, AV_LOG_INFO, "%s\n", __PRETTY_FUNCTION__);

    if (!inlink->hw_frames_ctx) {
        av_log(avctx, AV_LOG_ERROR, "No hardware frame context provided on input\n");
        return AVERROR(EINVAL);
    }

    input_frames_ctx = (AVHWFramesContext *)inlink->hw_frames_ctx->data;
    if (input_frames_ctx->format != AV_PIX_FMT_NVTEGRA)
        return AVERROR(EINVAL);

    desc = av_pix_fmt_desc_get(input_frames_ctx->format);
    for (i = 0; i < desc->nb_components; ++i) {
        if (desc->comp[i].depth > 8) {
            av_log(avctx, AV_LOG_ERROR, "Color depth %d > 8 for component %d of format %s unsupported\n",
                   desc->comp[i].depth, i, desc->name);
            return AVERROR(EINVAL);
        }
    }

    ctx->input_format  = input_frames_ctx->sw_format;
    ctx->output_format = input_frames_ctx->sw_format;
    ctx->output_width  = inlink->w;
    ctx->output_height = inlink->h;

    if (!input_frames_ctx->device_ref) {
        av_log(avctx, AV_LOG_ERROR, "No hardware context provided on input\n");
        return AVERROR(EINVAL);
    }

    ctx->device_ref = av_buffer_ref(input_frames_ctx->device_ref);
    if (!ctx->device_ref)
        return AVERROR(ENOMEM);

    device_ctx = ((AVHWDeviceContext *)ctx->device_ref->data)->hwctx;

    err = av_nvtegra_job_pool_init(&ctx->pool, &device_ctx->vic_channel,
                                   ctx->vic_map_size, ctx->vic_cmdbuf_off,
                                   ctx->max_cmdbuf_size);
    if (err < 0)
        goto fail;

    return 0;

fail:
    av_buffer_unref(&ctx->device_ref);
    return err;
}

int ff_nvtegra_vpp_config_output(AVFilterLink *outlink) {
    AVFilterLink     *inlink = outlink->src->inputs[0];
    AVFilterContext   *avctx = outlink->src;
    FFNVTegraVppContext *ctx = avctx->priv;

    AVHWFramesContext *input_hwframes_ctx, *output_hwframes_ctx;
    int err;

    av_log(avctx, AV_LOG_INFO, "%s\n", __PRETTY_FUNCTION__);

    av_buffer_unref(&outlink->hw_frames_ctx);

    input_hwframes_ctx = (AVHWFramesContext *)inlink->hw_frames_ctx->data;

    outlink->w = ctx->output_width;
    outlink->h = ctx->output_height;

    /* Attempt to reuse the input hardware frame context */
    if ((input_hwframes_ctx->sw_format == ctx->output_format) &&
            (input_hwframes_ctx->width == ctx->output_width) &&
            (input_hwframes_ctx->width == ctx->output_width))
    {
        av_log(avctx, AV_LOG_INFO, "Reusing hardware frame context\n");

        outlink->hw_frames_ctx = av_buffer_ref(inlink->hw_frames_ctx);
        if (!outlink->hw_frames_ctx)
            return AVERROR(ENOMEM);
    } else {
        av_log(avctx, AV_LOG_INFO, "Allocating new hardware frame context\n");

        outlink->hw_frames_ctx = av_hwframe_ctx_alloc(ctx->device_ref);
        if (!outlink->hw_frames_ctx)
            return AVERROR(ENOMEM);

        output_hwframes_ctx = (AVHWFramesContext *)outlink->hw_frames_ctx->data;

        output_hwframes_ctx->format    = AV_PIX_FMT_NVTEGRA;
        output_hwframes_ctx->sw_format = ctx->output_format;
        output_hwframes_ctx->width     = outlink->w;
        output_hwframes_ctx->height    = outlink->h;

        err = av_hwframe_ctx_init(outlink->hw_frames_ctx);
        if (err < 0) {
            av_buffer_unref(&outlink->hw_frames_ctx);
            return err;
        }
    }

    return 0;
}

int ff_nvtegra_vpp_init_config(FFNVTegraVppContext *ctx, VicConfigStruct *config, AVFrame *output,
                               AVFrame **input, int num_input_frames)
{
    const AVPixFmtDescriptor *input_desc, *output_desc;
    AVNVTegraMap *input_map, *output_map;

    int i;

    output_desc = av_pix_fmt_desc_get(ctx->output_format);
    output_map  = av_nvtegra_frame_get_fbuf_map(output);

    *config = (VicConfigStruct){
        .pipeConfig = {
            .DownsampleHoriz            = 1 << 2, /* U9.2 */
            .DownsampleVert             = 1 << 2, /* U9.2 */
        },
        .outputConfig = {
            .AlphaFillMode              = NVB0B6_DXVAHD_ALPHA_FILL_MODE_OPAQUE,
            .BackgroundAlpha            = 0,
            .BackgroundR                = 0,
            .BackgroundG                = 0,
            .BackgroundB                = 0,
            .TargetRectLeft             = 0,
            .TargetRectRight            = output->width  - 1,
            .TargetRectTop              = 0,
            .TargetRectBottom           = output->height - 1,
        },
        .outputSurfaceConfig = {
            .OutPixelFormat             = av_nvtegra_pixfmt_to_vic(ctx->output_format),
            .OutSurfaceWidth            = output->width  - 1,
            .OutSurfaceHeight           = output->height - 1,
            .OutBlkKind                 = !output_map->is_linear ? NVB0B6_BLK_KIND_GENERIC_16Bx2 : NVB0B6_BLK_KIND_PITCH,
            .OutBlkHeight               = !output_map->is_linear ? 1 : 0, /* GOB height 2 */
            .OutLumaWidth               = (output->linesize[0] / output_desc->comp[0].step) - 1,
            .OutLumaHeight              = FFALIGN(output->height, !output_map->is_linear ? 32 : 2) - 1,
            .OutChromaWidth             = (output_desc->flags & AV_PIX_FMT_FLAG_RGB) ?
                                          -1 : (output->linesize[1] / output_desc->comp[1].step) - 1,
            .OutChromaHeight            = (output_desc->flags & AV_PIX_FMT_FLAG_RGB) ? -1 :
                                          (FFALIGN(output->height, !output_map->is_linear ? 32 : 2) >> output_desc->log2_chroma_h) - 1,
        },
    };

    for (i = 0; i < num_input_frames; ++i) {
        input_desc = av_pix_fmt_desc_get(ctx->output_format);
        input_map  = av_nvtegra_frame_get_fbuf_map(input[i]);

        config->slotStruct[i] = (VicSlotStruct){
            .slotConfig = {
                .SlotEnable         = 1,
                .CurrentFieldEnable = 1,
                .SoftClampLow       = 0,
                .SoftClampHigh      = 1023,
                .PlanarAlpha        = 1023,
                .ConstantAlpha      = 1,
                .SourceRectLeft     = 0,
                .SourceRectRight    = (input[i]->width  - 1) << 16, /* U14.16 (for subpixel positioning) */
                .SourceRectTop      = 0,
                .SourceRectBottom   = (input[i]->height - 1) << 16,
                .DestRectLeft       = 0,
                .DestRectRight      = input[i]->width  - 1,
                .DestRectTop        = 0,
                .DestRectBottom     = input[i]->height - 1,
            },
            .slotSurfaceConfig = {
                .SlotPixelFormat    = av_nvtegra_pixfmt_to_vic(ctx->input_format),
                .SlotChromaLocHoriz = ((input_desc->flags & AV_PIX_FMT_FLAG_RGB)         ||
                                       input[i]->chroma_location == AVCHROMA_LOC_TOPLEFT ||
                                       input[i]->chroma_location == AVCHROMA_LOC_LEFT    ||
                                       input[i]->chroma_location == AVCHROMA_LOC_BOTTOMLEFT) ? 0 : 1,
                .SlotChromaLocVert  = ((input_desc->flags & AV_PIX_FMT_FLAG_RGB)         ||
                                       input[i]->chroma_location == AVCHROMA_LOC_TOPLEFT ||
                                       input[i]->chroma_location == AVCHROMA_LOC_TOP) ? 0 :
                                      (input[i]->chroma_location == AVCHROMA_LOC_LEFT    ||
                                       input[i]->chroma_location == AVCHROMA_LOC_CENTER) ? 1 : 2,
                .SlotBlkKind        = !input_map->is_linear ? NVB0B6_BLK_KIND_GENERIC_16Bx2 : NVB0B6_BLK_KIND_PITCH,
                .SlotBlkHeight      = !input_map->is_linear ? 1 : 0, /* GOB height 2 */
                .SlotCacheWidth     = !input_map->is_linear ? 1 : 3, /* 32Bx8 for block, 128Bx2 for pitch */
                .SlotSurfaceWidth   = input[i]->width  - 1,
                .SlotSurfaceHeight  = input[i]->height - 1,
                .SlotLumaWidth      = (input[i]->linesize[0] / input_desc->comp[0].step) - 1,
                .SlotLumaHeight     = FFALIGN(input[i]->height, !input_map->is_linear ? 32 : 2) - 1,
                .SlotChromaWidth    = (input_desc->flags & AV_PIX_FMT_FLAG_RGB) ?
                                      -1 : (input[i]->linesize[1] / input_desc->comp[1].step) - 1,
                .SlotChromaHeight   = (input_desc->flags & AV_PIX_FMT_FLAG_RGB) ? -1 :
                                      (FFALIGN(input[i]->height, !input_map->is_linear ? 32 : 2) >> input_desc->log2_chroma_h) - 1,
            },
        };
    }

    return 0;
}
