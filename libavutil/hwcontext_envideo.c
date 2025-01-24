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

#include <envideo/envideo.h>

#include "imgutils.h"
#include "mem.h"
#include "pixdesc.h"
#include "hwcontext.h"
#include "hwcontext_internal.h"

#include "hwcontext_envideo.h"

typedef struct EnvideoDevicePriv {
    /* The public AVEnvideoDeviceContext */
    AVEnvideoDeviceContext p;

    EnvideoChannel *copy_channel;
    EnvideoMap     *copy_cmdbuf_map;
    EnvideoCmdbuf  *copy_cmdbuf;
} EnvideoDevicePriv;

static const enum AVPixelFormat supported_sw_formats[] = {
    AV_PIX_FMT_GRAY8,
    AV_PIX_FMT_NV12,
    AV_PIX_FMT_P010,
    AV_PIX_FMT_YUV420P,
};

static void envideo_dev_uninit(AVHWDeviceContext *ctx) {
    EnvideoDevicePriv       *priv = ctx->hwctx;
    AVEnvideoDeviceContext *hwctx = &priv->p;

    av_log(ctx, AV_LOG_DEBUG, "Deinitializing envideo device\n");

    envideo_cmdbuf_destroy(priv->copy_cmdbuf);
    envideo_map_destroy(priv->copy_cmdbuf_map);
    envideo_channel_destroy(priv->copy_channel);

    envideo_device_destroy(hwctx->device);
}

static int envideo_dev_init(AVHWDeviceContext *ctx) {
    EnvideoDevicePriv       *priv = ctx->hwctx;
    AVEnvideoDeviceContext *hwctx = &priv->p;

    int err;

    av_log(ctx, AV_LOG_DEBUG, "Initializing envideo device\n");

    err = envideo_channel_create(hwctx->device, &priv->copy_channel, EnvideoEngine_Copy);
    if (err)
        goto fail;

    err = envideo_map_create(hwctx->device, &priv->copy_cmdbuf_map, 0x1000, ENVIDEO_MAP_ALIGN,
                             EnvideoMap_CpuWriteCombine | EnvideoMap_GpuUncacheable | EnvideoMap_UsageCmdbuf);
    if (err)
        goto fail;

    err = envideo_cmdbuf_create(priv->copy_channel, &priv->copy_cmdbuf);
    if (err)
        goto fail;

    err = envideo_cmdbuf_add_memory(priv->copy_cmdbuf, priv->copy_cmdbuf_map, 0,
                                    envideo_map_get_size(priv->copy_cmdbuf_map));
    if (err)
        goto fail;

    return 0;

fail:
    envideo_dev_uninit(ctx);
    return err;
}

static int envideo_dev_create(AVHWDeviceContext *ctx, const char *device,
                                 AVDictionary *opts, int flags)
{
    EnvideoDevicePriv       *priv = ctx->hwctx;
    AVEnvideoDeviceContext *hwctx = &priv->p;

    av_log(ctx, AV_LOG_DEBUG, "Creating envideo device\n");

    return envideo_device_create(&hwctx->device);
}

static int envideo_frames_get_constraints(AVHWDeviceContext *ctx, const void *hwconfig,
                                          AVHWFramesConstraints *constraints)
{
    av_log(ctx, AV_LOG_DEBUG, "Getting frame constraints for envideo device\n");

    constraints->valid_sw_formats = av_malloc_array(FF_ARRAY_ELEMS(supported_sw_formats) + 1,
                                                    sizeof(*constraints->valid_sw_formats));
    if (!constraints->valid_sw_formats)
        return AVERROR(ENOMEM);

    for (int i = 0; i < FF_ARRAY_ELEMS(supported_sw_formats); ++i)
        constraints->valid_sw_formats[i] = supported_sw_formats[i];
    constraints->valid_sw_formats[FF_ARRAY_ELEMS(supported_sw_formats)] = AV_PIX_FMT_NONE;

    constraints->valid_hw_formats = av_malloc_array(2, sizeof(*constraints->valid_hw_formats));
    if (!constraints->valid_hw_formats)
        return AVERROR(ENOMEM);

    constraints->valid_hw_formats[0] = AV_PIX_FMT_ENVIDEO;
    constraints->valid_hw_formats[1] = AV_PIX_FMT_NONE;

    return 0;
}

static void envideo_frame_free(void *opaque, uint8_t *data) {
    AVEnvideoFrame *frame = (AVEnvideoFrame *)data;

    if (!frame)
        return;

    envideo_map_destroy(frame->map);

    av_freep(&frame);
}

static AVBufferRef *envideo_pool_alloc(void *opaque, size_t size) {
    AVHWFramesContext        *ctx = opaque;
    EnvideoDevicePriv       *priv = ctx->device_ctx->hwctx;
    AVEnvideoDeviceContext *hwctx = &priv->p;

    AVBufferRef   *buffer = NULL;
    AVEnvideoFrame *frame = NULL;
    int err;

    av_log(ctx, AV_LOG_DEBUG, "Creating surface from envideo device\n");

    frame = av_mallocz(sizeof(*frame));
    if (!frame)
        goto fail;

    err = envideo_map_create(hwctx->device, &frame->map, size, ENVIDEO_MAP_ALIGN,
                             EnvideoMap_CpuWriteCombine | EnvideoMap_GpuCacheable | EnvideoMap_UsageFramebuffer);
    if (err < 0)
        goto fail;

    buffer = av_buffer_create((uint8_t *)frame, sizeof(*frame), envideo_frame_free, ctx, 0);
    if (!buffer)
        goto fail;

    return buffer;

fail:
    av_log(ctx, AV_LOG_ERROR, "Failed to create buffer\n");
    envideo_frame_free(opaque, (uint8_t *)frame);
    return NULL;
}

static int envideo_frames_init(AVHWFramesContext *ctx) {
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(ctx->sw_format);

    uint32_t width_aligned, height_aligned, size;
    int bpp;

    av_log(ctx, AV_LOG_DEBUG, "Initializing frame pool for the envideo device\n");

    if (!ctx->pool) {
        bpp = desc->comp[0].step;
        width_aligned  = FFALIGN(ctx->width,  ENVIDEO_WIDTH_ALIGN (bpp));
        height_aligned = FFALIGN(ctx->height, ENVIDEO_HEIGHT_ALIGN(bpp));

        size = av_image_get_buffer_size(ctx->sw_format, width_aligned, height_aligned,
                                        ENVIDEO_WIDTH_ALIGN(bpp));

        ffhwframesctx(ctx)->pool_internal = av_buffer_pool_init2(size, ctx, envideo_pool_alloc, NULL);
        if (!ffhwframesctx(ctx)->pool_internal)
            return AVERROR(ENOMEM);
    }

    return 0;
}

static void envideo_frames_uninit(AVHWFramesContext *ctx) {
    av_log(ctx, AV_LOG_DEBUG, "Deinitializing frame pool for the envideo device\n");
}

static int envideo_get_buffer(AVHWFramesContext *ctx, AVFrame *frame) {
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(ctx->sw_format);

    AVEnvideoFrame *f;
    uint32_t width_aligned, height_aligned;
    int bpp, err;

    av_log(ctx, AV_LOG_DEBUG, "Getting frame buffer for envideo device\n");

    frame->buf[0] = av_buffer_pool_get(ctx->pool);
    if (!frame->buf[0])
        return AVERROR(ENOMEM);

    f = (AVEnvideoFrame *)frame->buf[0]->data;

    bpp = desc->comp[0].step;
    width_aligned  = FFALIGN(ctx->width,  ENVIDEO_WIDTH_ALIGN (bpp));
    height_aligned = FFALIGN(ctx->height, ENVIDEO_HEIGHT_ALIGN(bpp));

    err = av_image_fill_arrays(frame->data, frame->linesize, envideo_map_get_cpu_addr(f->map),
                               ctx->sw_format, width_aligned, height_aligned,
                               ENVIDEO_WIDTH_ALIGN(bpp));
    if (err < 0)
        return err;

    frame->format = AV_PIX_FMT_ENVIDEO;
    frame->width  = ctx->width;
    frame->height = ctx->height;

    return 0;
}

static int envideo_transfer_get_formats(AVHWFramesContext *ctx,
                                        enum AVHWFrameTransferDirection dir,
                                        enum AVPixelFormat **formats)
{
    enum AVPixelFormat *fmts;

    av_log(ctx, AV_LOG_DEBUG, "Getting transfer formats for envideo device\n");

    fmts = av_malloc_array(2, sizeof(**formats));
    if (!fmts)
        return AVERROR(ENOMEM);

    fmts[0] = ctx->sw_format;
    fmts[1] = AV_PIX_FMT_NONE;

    *formats = fmts;
    return 0;
}

static int envideo_transfer_data(AVHWFramesContext *ctx, AVFrame *dst, const AVFrame *src) {
    EnvideoDevicePriv       *priv = ctx->device_ctx->hwctx;
    AVEnvideoDeviceContext *hwctx = &priv->p;

    bool from;
    const AVFrame *swframe, *hwframe;
    AVEnvideoFrame *enframe;
    const AVPixFmtDescriptor *desc;
    EnvideoMap *maps[4] = {0};
    uint8_t *map_bases[4];
    uint32_t map_offsets[4];
    EnvideoMap *plane_maps[4] = {0};
    uint32_t plane_offsets[4];
    int plane_bpp[4] = {0};
    int num_planes = 0, num_maps = 0, i, j, err;

    from    = !dst->hw_frames_ctx;
    swframe = from ? dst : src, hwframe = from ? src : dst;
    enframe = (AVEnvideoFrame *)hwframe->buf[0]->data;

    if (swframe->hw_frames_ctx)
        return AVERROR(ENOSYS);

    desc       = av_pix_fmt_desc_get(ctx->sw_format);
    num_planes = av_pix_fmt_count_planes(swframe->format);

    for (i = 0; i < desc->nb_components; ++i)
        plane_bpp[desc->comp[i].plane] = desc->comp[i].step;

    /* Create a map for each frame backing buffer */
    for (i = 0; i < FF_ARRAY_ELEMS(maps); num_maps = ++i) {
        if (!swframe->buf[i])
            break;

        /**
         * In order to avoid a full-frame copy on the CPU, the provided memory
         * is mapped into the GPU and used directly as the target location.
         * The address and size are aligned to page boundaries.
         * Cache management is performed manually to not affect data outside the buffer.
         */
        map_bases  [i] = (uint8_t *)((uintptr_t)swframe->buf[i]->data & ~0xfff);
        map_offsets[i] = (uintptr_t)swframe->buf[i]->data & 0xfff;
        err = envideo_map_from_va(hwctx->device, &maps[i], map_bases[i], swframe->buf[i]->size + map_offsets[i], 0x100,
                                  EnvideoMap_CpuCacheable | EnvideoMap_GpuCacheable | EnvideoMap_UsageFramebuffer);
        if (err < 0)
            goto fail;

        /* Flush/invalidate the CPU cache prior to the transfer */
        err = envideo_map_cache_op(maps[i], map_offsets[i], swframe->buf[i]->size,
                                   !from ? EnvideoCache_Writeback : EnvideoCache_Invalidate);
        if (err < 0)
            goto fail;
    }

    /* Find the corresponding map object and its offset for each plane  */
    for (i = 0; i < num_planes; ++i) {
        for (j = 0; j < FF_ARRAY_ELEMS(swframe->buf); ++j) {
            if ((swframe->buf[j]->data <= swframe->data[i]) &&
                    (swframe->data[i] < swframe->buf[j]->data + swframe->buf[j]->size))
                break;
        }

        plane_maps   [i] = maps[j];
        plane_offsets[i] = swframe->data[i] - map_bases[j];
    }

    err = envideo_cmdbuf_clear(priv->copy_cmdbuf);
    if (err)
        goto fail;

    /* If transferring from a hardware frames, wait until former operations on the source data have completed */
    if (from) {
        err = envideo_cmdbuf_begin(priv->copy_cmdbuf, EnvideoEngine_Host);
        if (err < 0)
            return err;

        err = envideo_cmdbuf_wait_fence(priv->copy_cmdbuf, enframe->fence);
        if (err < 0)
            return err;

        err = envideo_cmdbuf_end(priv->copy_cmdbuf);
        if (err < 0)
            return err;
    }

    for (i = 0; i < num_planes; ++i) {
        EnvideoSurfaceInfo src_info = {
            .map        = from ? av_envideo_frame_get_fbuf_map(src) : plane_maps[i],
            .map_offset = from ? src->data[i] - src->data[0]        : plane_offsets[i],
            .width      = AV_CEIL_RSHIFT(src->width,  i ? desc->log2_chroma_w : 0) * plane_bpp[i],
            .height     = AV_CEIL_RSHIFT(src->height, i ? desc->log2_chroma_h : 0),
            .stride     = src->linesize[i],
            .tiled      = from && !enframe->is_pitch,
            .gob_height = from ? 2 : 0, /* Engine code assumes GOB_HEIGHT = 2 */
        };

        EnvideoSurfaceInfo dst_info = {
            .map        = !from ? av_envideo_frame_get_fbuf_map(dst) : plane_maps[i],
            .map_offset = !from ? dst->data[i] - dst->data[0]        : plane_offsets[i],
            .width      = AV_CEIL_RSHIFT(dst->width,  i ? desc->log2_chroma_w : 0) * plane_bpp[i],
            .height     = AV_CEIL_RSHIFT(dst->height, i ? desc->log2_chroma_h : 0),
            .stride     = dst->linesize[i],
            .tiled      = !from && !enframe->is_pitch,
            .gob_height = !from ? 2 : 0, /* Engine code assumes GOB_HEIGHT = 2 */
        };

        err = envideo_surface_transfer(priv->copy_cmdbuf, &src_info, &dst_info);
        if (err < 0)
            return err;
    }

    /* L2 cache flush will be performed by the kernel during map teardown */
    err = envideo_channel_submit(priv->copy_channel, priv->copy_cmdbuf, &enframe->fence);
    if (err)
        goto fail;

    err = envideo_fence_wait(hwctx->device, enframe->fence, UINT64_MAX);
    if (err)
        goto fail;

fail:
    for (i = 0; i < num_maps; ++i)
        envideo_map_destroy(maps[i]);

    return err;
}

const HWContextType ff_hwcontext_type_envideo = {
    .type                   = AV_HWDEVICE_TYPE_ENVIDEO,
    .name                   = "envideo",

    .device_hwctx_size      = sizeof(EnvideoDevicePriv),
    .device_hwconfig_size   = 0,
    .frames_hwctx_size      = 0,

    .device_create          = &envideo_dev_create,
    .device_init            = &envideo_dev_init,
    .device_uninit          = &envideo_dev_uninit,

    .frames_get_constraints = &envideo_frames_get_constraints,
    .frames_init            = &envideo_frames_init,
    .frames_uninit          = &envideo_frames_uninit,
    .frames_get_buffer      = &envideo_get_buffer,

    .transfer_get_formats   = &envideo_transfer_get_formats,
    .transfer_data_to       = &envideo_transfer_data,
    .transfer_data_from     = &envideo_transfer_data,

    .pix_fmts = (const enum AVPixelFormat[]) {
        AV_PIX_FMT_ENVIDEO,
        AV_PIX_FMT_NONE,
    },
};
