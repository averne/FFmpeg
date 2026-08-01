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
#include <string.h>

#include "avcodec.h"
#include "hwaccel_internal.h"
#include "internal.h"
#include "hwconfig.h"
#include "vp8.h"
#include "vp8data.h"
#include "decode.h"
#include "envideo_decode.h"

#include "libavutil/pixdesc.h"

typedef struct EnvideoVP8DecodeContextShared {
    EnvideoMap *common_map;
    uint32_t prob_data_off, history_off;
    uint32_t history_size;
} EnvideoVP8DecodeContextShared;

typedef struct EnvideoVP8DecodeContext {
    FFEnvideoDecodeContext core;
    EnvideoVP8DecodeContextShared *shared;

    AVFrame *golden_frame, *altref_frame,
            *previous_frame;
} EnvideoVP8DecodeContext;

/* Size (width, height) of a macroblock */
#define MB_SIZE 16

static int envideo_vp8_decode_uninit(AVCodecContext *avctx) {
    EnvideoVP8DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    int err;

    av_log(avctx, AV_LOG_DEBUG, "Deinitializing vp8-envideo decoder\n");

    err = ff_envideo_decode_uninit(avctx, &ctx->core);
    if (err < 0)
        return err;

    av_refstruct_unref(&ctx->shared);

    return 0;
}

static void envideo_vp8_init_probs(void *p) {
    int i, j, k;
    uint8_t *ptr = p;

    memset(p, 0, 0x4cc);

    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 8; ++j) {
            for (k = 0; k < 3; ++k) {
                memcpy(ptr, vp8_token_default_probs[i][j][k], NUM_DCT_TOKENS - 1);
                ptr += NUM_DCT_TOKENS;
            }
        }
    }

    memcpy(ptr, vp8_pred16x16_prob_inter, sizeof(vp8_pred16x16_prob_inter));
    ptr += 4;

    memcpy(ptr, vp8_pred8x8c_prob_inter, sizeof(vp8_pred8x8c_prob_inter));
    ptr += 4;

    for (i = 0; i < 2; ++i) {
        memcpy(ptr, vp8_mv_default_prob[i], 19);
        ptr += 20;
    }
}

static void envideo_vp8_shared_free(AVRefStructOpaque opaque, void *obj) {
    EnvideoVP8DecodeContextShared *shared = obj;

    envideo_map_destroy(shared->common_map);
}

static int envideo_vp8_decode_init(AVCodecContext *avctx) {
    EnvideoVP8DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    AVHWDeviceContext      *hw_device_ctx;
    AVEnvideoDeviceContext *device_hwctx;
    EnvideoVP8DecodeContextShared *ss;
    FFEnvideoDecodeContextShared *sc;
    uint32_t width_in_mbs, common_map_size;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Initializing vp8-envideo decoder\n");

    ctx->shared = av_refstruct_alloc_ext(sizeof(*ctx->shared), 0, NULL, envideo_vp8_shared_free);
    if (!ctx->shared) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    err = ff_envideo_alloc_shared(&ctx->core);
    if (err < 0)
        goto fail;

    ss = ctx->shared;
    sc = ctx->core.shared;

    /* Ignored: histogram map, size 0x400 */
    sc->pic_setup_off        = 0;
    sc->status_off           = FFALIGN(sc->pic_setup_off + sizeof(nvdec_vp8_pic_s),
                                       ENVIDEO_MAP_ALIGN);
    sc->cmdbuf_off           = FFALIGN(sc->status_off    + sizeof(nvdec_status_s),
                                       ENVIDEO_MAP_ALIGN);
    sc->bitstream_off        = FFALIGN(sc->cmdbuf_off    + ENVIDEO_MAP_ALIGN,
                                       ENVIDEO_MAP_ALIGN);
    ctx->core.input_map_size = FFALIGN(sc->bitstream_off + ff_envideo_decode_pick_bitstream_buffer_size(avctx),
                                       0x1000);

    sc->max_cmdbuf_size          = sc->bitstream_off        - sc->cmdbuf_off;
    ctx->core.max_bitstream_size = ctx->core.input_map_size - sc->bitstream_off;

    err = ff_envideo_decode_init(avctx, &ctx->core);
    if (err < 0)
        goto fail;

    hw_device_ctx = (AVHWDeviceContext *)sc->hw_device_ref->data;
    device_hwctx  = hw_device_ctx->hwctx;

    width_in_mbs = FFALIGN(avctx->coded_width, MB_SIZE) / MB_SIZE;
    ss->history_size = width_in_mbs * 0x200;

    ss->prob_data_off = 0;
    ss->history_off   = FFALIGN(ss->prob_data_off + 0x4b00,           ENVIDEO_MAP_ALIGN);
    common_map_size   = FFALIGN(ss->history_off   + ss->history_size, 0x1000);

    err = envideo_map_create(device_hwctx->device, &ss->common_map, common_map_size, ENVIDEO_MAP_ALIGN,
                             EnvideoMap_CpuWriteCombine | EnvideoMap_GpuCacheable |
                             EnvideoMap_LocationDevice  | EnvideoMap_UsageEngine);
    if (err < 0)
        goto fail;

    err = envideo_map_pin(ss->common_map, sc->channel);
    if (err < 0)
        goto fail;

    envideo_vp8_init_probs((uint8_t *)envideo_map_get_cpu_addr(ss->common_map) + ss->prob_data_off);

    return 0;

fail:
    envideo_vp8_decode_uninit(avctx);
    return err;
}

static void envideo_vp8_prepare_frame_setup(nvdec_vp8_pic_s *setup, VP8Context *h,
                                            EnvideoVP8DecodeContext *ctx)
{
    AVFrame          *frame = h->framep[VP8_FRAME_CURRENT]->tf.f;
    AVEnvideoFrame *evframe = (AVEnvideoFrame *)frame->buf[0]->data;

    *setup = (nvdec_vp8_pic_s){
        .gptimer_timeout_value            = 0, /* Default value */

        .FrameWidth                       = FFALIGN(frame->width,  MB_SIZE),
        .FrameHeight                      = FFALIGN(frame->height, MB_SIZE),

        .keyFrame                         = h->keyframe,
        .version                          = h->profile,

        .tileFormat                       = !ctx->core.shared->is_tegra, /* Tegra/GPU block linear */
        .gob_height                       = ff_ctz(evframe->gob_height) - 1,

        .errorConcealOn                   = 1,

        .firstPartSize                    = h->header_partition_size,

        .HistBufferSize                   = ctx->shared->history_size / 256,

        .FrameStride                      = {
            frame->linesize[0] / MB_SIZE,
            frame->linesize[1] / MB_SIZE,
        },

        .luma_top_offset                  = 0,
        .luma_bot_offset                  = 0,
        .luma_frame_offset                = 0,
        .chroma_top_offset                = 0,
        .chroma_bot_offset                = 0,
        .chroma_frame_offset              = 0,

        .current_output_memory_layout     = 0,           /* NV12 */
        .output_memory_layout             = { 0, 0, 0 }, /* NV12 */

        /* ???: Official code sets this value at offset 0x8d (ie. reserved1[0]), so just set both */
        .segmentation_feature_data_update = h->segmentation.enabled ? h->segmentation.update_feature_data : 0,
        .reserved1[0]                     = h->segmentation.enabled ? h->segmentation.update_feature_data : 0,

        .resultValue                      = 0,
    };
}

static int envideo_vp8_prepare_cmdbuf(EnvideoCmdbuf *cmdbuf, VP8Context *h,
                                      EnvideoVP8DecodeContext *ctx, AVFrame *cur_frame)
{
    EnvideoVP8DecodeContextShared *ss = ctx->shared;
    FFEnvideoDecodeContextShared  *sc = ctx->core.shared;
    FFEnvideoDecodeField       *field = ff_envideo_get_priv(cur_frame, false);
    AVEnvideoJob                 *job = (AVEnvideoJob *)field->operation.job_ref->data;
    EnvideoMap             *input_map = job->input_map;

    int err;

    err = envideo_cmdbuf_begin(cmdbuf, EnvideoEngine_Nvdec);
    if (err < 0)
        return err;

    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_APPLICATION_ID,
                          DRF_DEF(C9B0, _SET_APPLICATION_ID, _ID, _VP8));
    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_CONTROL_PARAMS,
                          DRF_DEF(C9B0, _SET_CONTROL_PARAMS, _CODEC_TYPE,     _VP8) |
                          DRF_NUM(C9B0, _SET_CONTROL_PARAMS, _ERR_CONCEAL_ON, 1)    |
                          DRF_NUM(C9B0, _SET_CONTROL_PARAMS, _GPTIMER_ON,     1));
    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_PICTURE_INDEX,
                          DRF_NUM(C9B0, _SET_PICTURE_INDEX, _INDEX, ctx->core.frame_idx));

    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_DRV_PIC_SETUP_OFFSET, input_map, sc->pic_setup_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_IN_BUF_BASE_OFFSET,   input_map, sc->bitstream_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_NVDEC_STATUS_OFFSET,  input_map, sc->status_off);

    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_VP8_SET_PROB_DATA_OFFSET, ss->common_map, ss->prob_data_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_HISTORY_OFFSET,       ss->common_map, ss->history_off);

#define PUSH_FRAME(fr, offset) ({                                                               \
    FF_ENVIDEO_PUSH_RELOC_TILED(cmdbuf, NVC9B0_SET_PICTURE_LUMA_OFFSET0   + offset * 4,         \
                                av_envideo_frame_get_fbuf_map(fr), 0);                          \
    FF_ENVIDEO_PUSH_RELOC_TILED(cmdbuf, NVC9B0_SET_PICTURE_CHROMA_OFFSET0 + offset * 4,         \
                                av_envideo_frame_get_fbuf_map(fr), fr->data[1] - fr->data[0]);  \
})

    PUSH_FRAME(ctx->golden_frame,   0);
    PUSH_FRAME(ctx->altref_frame,   1);
    PUSH_FRAME(ctx->previous_frame, 2);
    PUSH_FRAME(cur_frame,           3);

    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_EXECUTE,
                          DRF_DEF(C9B0, _EXECUTE, _AWAKEN, _ENABLE));

    err = envideo_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    return 0;
}

static int envideo_vp8_start_frame(AVCodecContext *avctx, const AVBufferRef *buf_ref,
                                   const uint8_t *buf, uint32_t buf_size)
{
    VP8Context                *h = avctx->priv_data;
    AVFrame               *frame = h->framep[VP8_FRAME_CURRENT]->tf.f;
    EnvideoVP8DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    FFEnvideoDecodeField *field;
    AVEnvideoJob *job;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Starting vp8-envideo frame with pixel format %s\n",
           av_get_pix_fmt_name(avctx->sw_pix_fmt));

    err = ff_envideo_start_frame(avctx, frame, false, &ctx->core);
    if (err < 0)
        return err;

    field = ff_envideo_get_priv(frame, false);
    job   = (AVEnvideoJob *)field->operation.job_ref->data;
    mem   = envideo_map_get_cpu_addr(job->input_map);

    envideo_vp8_prepare_frame_setup((nvdec_vp8_pic_s *)(mem + ctx->core.shared->pic_setup_off), h, ctx);

#define SAFE_REF(type) (h->framep[(type)] ?: h->framep[VP8_FRAME_CURRENT])
    ctx->golden_frame   = ff_envideo_safe_get_ref(SAFE_REF(VP8_FRAME_GOLDEN)  ->tf.f, frame);
    ctx->altref_frame   = ff_envideo_safe_get_ref(SAFE_REF(VP8_FRAME_ALTREF)  ->tf.f, frame);
    ctx->previous_frame = ff_envideo_safe_get_ref(SAFE_REF(VP8_FRAME_PREVIOUS)->tf.f, frame);

    return 0;
}

static int envideo_vp8_end_frame(AVCodecContext *avctx) {
    VP8Context                *h = avctx->priv_data;
    EnvideoVP8DecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    AVFrame               *frame = h->framep[VP8_FRAME_CURRENT]->tf.f;
    FrameDecodeData         *fdd = (FrameDecodeData *)frame->private_ref;
    FFEnvideoDecodeField  *field = ff_envideo_get_priv(frame, false);

    AVEnvideoJob *job;
    FFEnvideoOperation *op;
    nvdec_vp8_pic_s *setup;
    uint8_t *mem;
    int err;

    if (!fdd || !field)
        return 0;

    job = (AVEnvideoJob *)field->operation.job_ref->data;
    op  = &field->operation;

    av_log(avctx, AV_LOG_DEBUG, "Ending vp8-envideo frame with %u slices -> %u bytes\n",
        op->num_slices, op->bitstream_len);

    mem = envideo_map_get_cpu_addr(job->input_map);

    setup = (nvdec_vp8_pic_s *)(mem + ctx->core.shared->pic_setup_off);
    setup->VLDBufferSize = op->bitstream_len;

    err = envideo_vp8_prepare_cmdbuf(job->cmdbuf, h, ctx, frame);
    if (err < 0)
        return err;

    return ff_envideo_end_frame(avctx, frame, false, &ctx->core, NULL, 0);
}

static int envideo_vp8_decode_slice(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    VP8Context  *h = avctx->priv_data;
    AVFrame *frame = h->framep[VP8_FRAME_CURRENT]->tf.f;

    int offset = h->keyframe ? 10 : 3;

    return ff_envideo_decode_slice(avctx, frame, false, buf + offset, buf_size - offset, false);
}

#if CONFIG_VP8_ENVIDEO_HWACCEL
const FFHWAccel ff_vp8_envideo_hwaccel = {
    .p.name         = "vp8_envideo",
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_VP8,
    .p.pix_fmt      = AV_PIX_FMT_ENVIDEO,
    .start_frame    = &envideo_vp8_start_frame,
    .end_frame      = &envideo_vp8_end_frame,
    .decode_slice   = &envideo_vp8_decode_slice,
    .init           = &envideo_vp8_decode_init,
    .uninit         = &envideo_vp8_decode_uninit,
    .frame_params   = &ff_envideo_frame_params,
    .priv_data_size = sizeof(EnvideoVP8DecodeContext),
    .caps_internal  = HWACCEL_CAP_ASYNC_SAFE,
};
#endif
