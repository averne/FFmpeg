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

#include "config_components.h"

#include "avcodec.h"
#include "hwaccel_internal.h"
#include "internal.h"
#include "hwconfig.h"
#include "vp8.h"
#include "vp8data.h"
#include "decode.h"
#include "tx1_decode.h"

#include "libavutil/pixdesc.h"
#include "libavutil/tx1_host1x.h"

typedef struct TX1VP8DecodeContext {
    TX1DecodeContext core;

    AVTX1Map common_map;
    uint32_t prob_data_off, history_off;
    uint32_t history_size;

    AVFrame *golden_frame,   *altref_frame,
            *previous_frame, *current_frame;
} TX1VP8DecodeContext;

/* Size (width, height) of a macroblock */
#define MB_SIZE 16

static int tx1_vp8_decode_uninit(AVCodecContext *avctx) {
    TX1VP8DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    int err;

    av_log(avctx, AV_LOG_DEBUG, "Deinitializing TX1 VP8 decoder\n");

    err = ff_tx1_map_destroy(&ctx->common_map);
    if (err < 0)
        return err;

    err = ff_tx1_decode_uninit(avctx, &ctx->core);
    if (err < 0)
        return err;

    return 0;
}

static void tx1_vp8_init_probs(void *p) {
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

static int tx1_vp8_decode_init(AVCodecContext *avctx) {
    TX1VP8DecodeContext *ctx = avctx->internal->hwaccel_priv_data;
#ifdef __SWITCH__
    AVHWDeviceContext *hw_device_ctx;
    AVTX1DeviceContext *device_hwctx;
#endif

    uint32_t width_in_mbs, common_map_size;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Initializing TX1 VP8 decoder\n");

    /* Ignored: histogram map, size 0x400 */
    ctx->core.pic_setup_off  = 0;
    ctx->core.status_off     = FFALIGN(ctx->core.pic_setup_off + sizeof(nvdec_vp8_pic_s),
                                       FF_TX1_MAP_ALIGN);
    ctx->core.cmdbuf_off     = FFALIGN(ctx->core.status_off    + sizeof(nvdec_status_s),
                                       FF_TX1_MAP_ALIGN);
    ctx->core.bitstream_off  = FFALIGN(ctx->core.cmdbuf_off    + FF_TX1_MAP_ALIGN,
                                       FF_TX1_MAP_ALIGN);
    ctx->core.input_map_size = FFALIGN(ctx->core.bitstream_off + ff_tx1_decode_pick_bitstream_buffer_size(avctx),
                                       0x1000);

    ctx->core.max_cmdbuf_size    = ctx->core.bitstream_off  - ctx->core.cmdbuf_off;
    ctx->core.max_bitstream_size = ctx->core.input_map_size - ctx->core.bitstream_off;

    err = ff_tx1_decode_init(avctx, &ctx->core);
    if (err < 0)
        goto fail;

    width_in_mbs = FFALIGN(avctx->coded_width, MB_SIZE) / MB_SIZE;
    ctx->history_size = width_in_mbs * 0x200;

    ctx->prob_data_off = 0;
    ctx->history_off   = FFALIGN(ctx->prob_data_off + 0x4b00,            FF_TX1_MAP_ALIGN);
    common_map_size    = FFALIGN(ctx->history_off   + ctx->history_size, 0x1000);

#ifdef __SWITCH__
    hw_device_ctx = (AVHWDeviceContext *)ctx->core.hw_device_ref->data;
    device_hwctx  = hw_device_ctx->hwctx;

    ctx->common_map.owner = device_hwctx->nvdec_channel.channel.fd;
#endif

    err = ff_tx1_map_create(&ctx->common_map, common_map_size, 0x100, NVMAP_CACHE_OP_INV);
    if (err < 0)
        goto fail;

    tx1_vp8_init_probs((uint8_t *)ff_tx1_map_get_addr(&ctx->common_map) + ctx->prob_data_off);

    return 0;

fail:
    tx1_vp8_decode_uninit(avctx);
    return err;
}

static void tx1_vp8_prepare_frame_setup(nvdec_vp8_pic_s *setup, VP8Context *h,
                                        TX1VP8DecodeContext *ctx)
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

        /* ??? */
        /* Official code sets this value at 0x8d (reserved1[0]), so just set both */
        .segmentation_feature_data_update = h->segmentation.enabled ? h->segmentation.update_feature_data : 0,
        .reserved1[0]                     = h->segmentation.enabled ? h->segmentation.update_feature_data : 0,

        .resultValue                      = 0,
    };
}

static int tx1_vp8_prepare_cmdbuf(AVTX1Cmdbuf *cmdbuf, VP8Context *h,
                                  TX1VP8DecodeContext *ctx, AVFrame *cur_frame)
{
    FrameDecodeData *fdd = (FrameDecodeData *)cur_frame->private_ref->data;
    TX1Frame         *tf = fdd->hwaccel_priv;
    AVTX1Map  *input_map = (AVTX1Map *)tf->input_map_ref->data;

    int err;

    err = ff_tx1_cmdbuf_begin(cmdbuf, HOST1X_CLASS_NVDEC);
    if (err < 0)
        return err;

    FF_TX1_PUSH_VALUE(cmdbuf, NVC5B0_SET_APPLICATION_ID,
                      FF_TX1_ENUM(NVC5B0_SET_APPLICATION_ID, ID, VP8));
    FF_TX1_PUSH_VALUE(cmdbuf, NVC5B0_SET_CONTROL_PARAMS,
                      FF_TX1_ENUM(NVC5B0_SET_CONTROL_PARAMS,  CODEC_TYPE,     VP8) |
                      FF_TX1_VALUE(NVC5B0_SET_CONTROL_PARAMS, ERR_CONCEAL_ON, 1) |
                      FF_TX1_VALUE(NVC5B0_SET_CONTROL_PARAMS, GPTIMER_ON,     1));
    FF_TX1_PUSH_VALUE(cmdbuf, NVC5B0_SET_PICTURE_INDEX,
                      FF_TX1_VALUE(NVC5B0_SET_PICTURE_INDEX, INDEX, ctx->core.frame_idx));

    FF_TX1_PUSH_RELOC(cmdbuf, NVC5B0_SET_DRV_PIC_SETUP_OFFSET,
                      input_map,        ctx->core.pic_setup_off, NVHOST_RELOC_TYPE_DEFAULT);
    FF_TX1_PUSH_RELOC(cmdbuf, NVC5B0_SET_IN_BUF_BASE_OFFSET,
                      input_map,        ctx->core.bitstream_off, NVHOST_RELOC_TYPE_DEFAULT);
    FF_TX1_PUSH_RELOC(cmdbuf, NVC5B0_SET_NVDEC_STATUS_OFFSET,
                      input_map,        ctx->core.status_off,    NVHOST_RELOC_TYPE_DEFAULT);

    FF_TX1_PUSH_RELOC(cmdbuf, NVC5B0_VP8_SET_PROB_DATA_OFFSET,
                      &ctx->common_map, ctx->prob_data_off,      NVHOST_RELOC_TYPE_DEFAULT);
    FF_TX1_PUSH_RELOC(cmdbuf, NVC5B0_SET_HISTORY_OFFSET,
                      &ctx->common_map, ctx->history_off,        NVHOST_RELOC_TYPE_DEFAULT);

#define PUSH_FRAME(fr, offset) ({                                                   \
    FF_TX1_PUSH_RELOC(cmdbuf, NVC5B0_SET_PICTURE_LUMA_OFFSET0   + offset * 4,       \
                      ff_tx1_frame_get_fbuf_map(fr), 0, NVHOST_RELOC_TYPE_DEFAULT); \
    FF_TX1_PUSH_RELOC(cmdbuf, NVC5B0_SET_PICTURE_CHROMA_OFFSET0 + offset * 4,       \
                      ff_tx1_frame_get_fbuf_map(fr), fr->data[1] - fr->data[0],     \
                      NVHOST_RELOC_TYPE_DEFAULT);                                   \
})

    PUSH_FRAME(ctx->golden_frame,   0);
    PUSH_FRAME(ctx->altref_frame,   1);
    PUSH_FRAME(ctx->previous_frame, 2);
    PUSH_FRAME(ctx->current_frame,  3);

    FF_TX1_PUSH_VALUE(cmdbuf, NVC5B0_EXECUTE,
                      FF_TX1_ENUM(NVC5B0_EXECUTE, AWAKEN, ENABLE));

    err = ff_tx1_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    return 0;
}

static int tx1_vp8_start_frame(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    VP8Context            *h = avctx->priv_data;
    AVFrame           *frame = h->framep[VP8_FRAME_CURRENT]->tf.f;
    FrameDecodeData     *fdd = (FrameDecodeData *)frame->private_ref->data;
    TX1VP8DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    TX1Frame *tf;
    AVTX1Map *input_map;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Starting VP8-TX1 frame with pixel format %s\n",
           av_get_pix_fmt_name(avctx->sw_pix_fmt));

    err = ff_tx1_start_frame(avctx, frame, &ctx->core);
    if (err < 0)
        return err;

    tf = fdd->hwaccel_priv;
    input_map = (AVTX1Map *)tf->input_map_ref->data;
    mem = ff_tx1_map_get_addr(input_map);

    tx1_vp8_prepare_frame_setup((nvdec_vp8_pic_s *)(mem + ctx->core.pic_setup_off), h, ctx);

#define SAFE_REF(type) (h->framep[(type)] ?: h->framep[VP8_FRAME_CURRENT])
    ctx->current_frame  =  h->framep[VP8_FRAME_CURRENT]->tf.f;
    ctx->golden_frame   = ff_tx1_safe_get_ref(SAFE_REF(VP8_FRAME_GOLDEN)  ->tf.f, ctx->current_frame);
    ctx->altref_frame   = ff_tx1_safe_get_ref(SAFE_REF(VP8_FRAME_ALTREF)  ->tf.f, ctx->current_frame);
    ctx->previous_frame = ff_tx1_safe_get_ref(SAFE_REF(VP8_FRAME_PREVIOUS)->tf.f, ctx->current_frame);

    return 0;
}

static int tx1_vp8_end_frame(AVCodecContext *avctx) {
    VP8Context            *h = avctx->priv_data;
    TX1VP8DecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    AVFrame           *frame = ctx->current_frame;
    FrameDecodeData     *fdd = (FrameDecodeData *)frame->private_ref->data;
    TX1Frame             *tf = fdd->hwaccel_priv;
    AVTX1Map      *input_map = (AVTX1Map *)tf->input_map_ref->data;

    nvdec_vp8_pic_s *setup;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Ending VP8-TX1 frame with %u slices -> %u bytes\n",
           ctx->core.num_slices, ctx->core.bitstream_len);

    mem = ff_tx1_map_get_addr(input_map);

    setup = (nvdec_vp8_pic_s *)(mem + ctx->core.pic_setup_off);
    setup->VLDBufferSize = ctx->core.bitstream_len;

    err = tx1_vp8_prepare_cmdbuf(&ctx->core.cmdbuf, h, ctx, frame);
    if (err < 0)
        return err;

    return ff_tx1_end_frame(avctx, frame, &ctx->core, NULL, 0);
}

static int tx1_vp8_decode_slice(AVCodecContext *avctx, const uint8_t *buf,
                                uint32_t buf_size)
{
    VP8Context            *h = avctx->priv_data;
    TX1VP8DecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    AVFrame           *frame = ctx->current_frame;

    int offset = h->keyframe ? 10 : 3;

    return ff_tx1_decode_slice(avctx, frame, buf + offset, buf_size - offset, false);
}

#if CONFIG_VP8_TX1_HWACCEL
const FFHWAccel ff_vp8_tx1_hwaccel = {
    .p.name               = "vp8_tx1",
    .p.type               = AVMEDIA_TYPE_VIDEO,
    .p.id                 = AV_CODEC_ID_VP8,
    .p.pix_fmt            = AV_PIX_FMT_TX1,
    .start_frame          = &tx1_vp8_start_frame,
    .end_frame            = &tx1_vp8_end_frame,
    .decode_slice         = &tx1_vp8_decode_slice,
    .init                 = &tx1_vp8_decode_init,
    .uninit               = &tx1_vp8_decode_uninit,
    .frame_params         = &ff_tx1_frame_params,
    .priv_data_size       = sizeof(TX1VP8DecodeContext),
    .caps_internal        = HWACCEL_CAP_ASYNC_SAFE,
};
#endif
