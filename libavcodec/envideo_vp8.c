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

typedef struct EnvideoVP8DecodeContext {
    FFEnvideoDecodeContext core;

    EnvideoMap *common_map;
    uint32_t prob_data_off, history_off;
    uint32_t history_size;

    AVFrame *golden_frame, *altref_frame,
            *previous_frame;
} EnvideoVP8DecodeContext;

/* Size (width, height) of a macroblock */
#define MB_SIZE 16

static int envideo_vp8_decode_uninit(AVCodecContext *avctx) {
    EnvideoVP8DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    int err;

    av_log(avctx, AV_LOG_DEBUG, "Deinitializing vp8-envideo decoder\n");

    err = envideo_map_destroy(ctx->common_map);
    if (err < 0)
        return err;

    err = ff_envideo_decode_uninit(avctx, &ctx->core);
    if (err < 0)
        return err;

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

static int envideo_vp8_decode_init(AVCodecContext *avctx) {
    EnvideoVP8DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    AVHWDeviceContext      *hw_device_ctx;
    AVEnvideoDeviceContext *device_hwctx;
    uint32_t width_in_mbs, common_map_size;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Initializing vp8-envideo decoder\n");

    /* Ignored: histogram map, size 0x400 */
    ctx->core.pic_setup_off  = 0;
    ctx->core.status_off     = FFALIGN(ctx->core.pic_setup_off + sizeof(nvdec_vp8_pic_s),
                                       ENVIDEO_MAP_ALIGN);
    ctx->core.cmdbuf_off     = FFALIGN(ctx->core.status_off    + sizeof(nvdec_status_s),
                                       ENVIDEO_MAP_ALIGN);
    ctx->core.bitstream_off  = FFALIGN(ctx->core.cmdbuf_off    + ENVIDEO_MAP_ALIGN,
                                       ENVIDEO_MAP_ALIGN);
    ctx->core.input_map_size = FFALIGN(ctx->core.bitstream_off + ff_envideo_decode_pick_bitstream_buffer_size(avctx),
                                       0x1000);

    ctx->core.max_cmdbuf_size    = ctx->core.bitstream_off  - ctx->core.cmdbuf_off;
    ctx->core.max_bitstream_size = ctx->core.input_map_size - ctx->core.bitstream_off;

    err = ff_envideo_decode_init(avctx, &ctx->core);
    if (err < 0)
        goto fail;

    hw_device_ctx = (AVHWDeviceContext *)ctx->core.hw_device_ref->data;
    device_hwctx  = hw_device_ctx->hwctx;

    width_in_mbs = FFALIGN(avctx->coded_width, MB_SIZE) / MB_SIZE;
    ctx->history_size = width_in_mbs * 0x200;

    ctx->prob_data_off = 0;
    ctx->history_off   = FFALIGN(ctx->prob_data_off + 0x4b00,            ENVIDEO_MAP_ALIGN);
    common_map_size    = FFALIGN(ctx->history_off   + ctx->history_size, 0x1000);

    err = envideo_map_create(device_hwctx->device, &ctx->common_map, common_map_size, ENVIDEO_MAP_ALIGN,
                             EnvideoMap_CpuWriteCombine | EnvideoMap_GpuCacheable | EnvideoMap_UsageEngine);
    if (err < 0)
        goto fail;

    envideo_vp8_init_probs((uint8_t *)envideo_map_get_cpu_addr(ctx->common_map) + ctx->prob_data_off);

    return 0;

fail:
    envideo_vp8_decode_uninit(avctx);
    return err;
}

static void envideo_vp8_prepare_frame_setup(nvdec_vp8_pic_s *setup, VP8Context *h,
                                            EnvideoVP8DecodeContext *ctx)
{
    *setup = (nvdec_vp8_pic_s){
        .gptimer_timeout_value            = 0, /* Default value */

        .FrameWidth                       = FFALIGN(h->framep[VP8_FRAME_CURRENT]->tf.f->width,  MB_SIZE),
        .FrameHeight                      = FFALIGN(h->framep[VP8_FRAME_CURRENT]->tf.f->height, MB_SIZE),

        .keyFrame                         = h->keyframe,
        .version                          = h->profile,

        .tileFormat                       = 0, /* TBL */
        .gob_height                       = 0, /* GOB_2 */

        .errorConcealOn                   = 1,

        .firstPartSize                    = h->header_partition_size,

        .HistBufferSize                   = ctx->history_size / 256,

        .FrameStride                      = {
            h->framep[VP8_FRAME_CURRENT]->tf.f->linesize[0] / MB_SIZE,
            h->framep[VP8_FRAME_CURRENT]->tf.f->linesize[1] / MB_SIZE,
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
    FrameDecodeData     *fdd = (FrameDecodeData *)cur_frame->private_ref->data;
    FFEnvideoDecodeFrame *tf = fdd->hwaccel_priv;
    EnvideoMap    *input_map = (EnvideoMap *)tf->operation.input_map_ref->data;

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

    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_DRV_PIC_SETUP_OFFSET, input_map, ctx->core.pic_setup_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_IN_BUF_BASE_OFFSET,   input_map, ctx->core.bitstream_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_NVDEC_STATUS_OFFSET,  input_map, ctx->core.status_off);

    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_VP8_SET_PROB_DATA_OFFSET, ctx->common_map, ctx->prob_data_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_HISTORY_OFFSET,       ctx->common_map, ctx->history_off);

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

static int envideo_vp8_start_frame(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    VP8Context                *h = avctx->priv_data;
    AVFrame               *frame = h->framep[VP8_FRAME_CURRENT]->tf.f;
    FrameDecodeData         *fdd = (FrameDecodeData *)frame->private_ref->data;
    EnvideoVP8DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    FFEnvideoDecodeFrame *tf;
    EnvideoMap *input_map;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Starting vp8-envideo frame with pixel format %s\n",
           av_get_pix_fmt_name(avctx->sw_pix_fmt));

    err = ff_envideo_start_frame(avctx, frame, &ctx->core);
    if (err < 0)
        return err;

    tf = fdd->hwaccel_priv;
    input_map = (EnvideoMap *)tf->operation.input_map_ref->data;
    mem = envideo_map_get_cpu_addr(input_map);

    envideo_vp8_prepare_frame_setup((nvdec_vp8_pic_s *)(mem + ctx->core.pic_setup_off), h, ctx);

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
    FrameDecodeData         *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame     *tf = fdd->hwaccel_priv;

    nvdec_vp8_pic_s *setup;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Ending vp8-envideo frame with %u slices -> %u bytes\n",
           ctx->core.num_slices, ctx->core.bitstream_len);

    if (!tf || !ctx->core.num_slices)
        return 0;

    mem = envideo_map_get_cpu_addr((EnvideoMap *)tf->operation.input_map_ref->data);

    setup = (nvdec_vp8_pic_s *)(mem + ctx->core.pic_setup_off);
    setup->VLDBufferSize = ctx->core.bitstream_len;

    err = envideo_vp8_prepare_cmdbuf(ctx->core.cmdbuf, h, ctx, frame);
    if (err < 0)
        return err;

    return ff_envideo_end_frame(avctx, frame, &ctx->core, NULL, 0);
}

static int envideo_vp8_decode_slice(AVCodecContext *avctx, const uint8_t *buf,
                                    uint32_t buf_size)
{
    VP8Context  *h = avctx->priv_data;
    AVFrame *frame = h->framep[VP8_FRAME_CURRENT]->tf.f;

    int offset = h->keyframe ? 10 : 3;

    return ff_envideo_decode_slice(avctx, frame, buf + offset, buf_size - offset, false);
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
