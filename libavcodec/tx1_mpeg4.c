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
#include "mpeg4video.h"
#include "mpeg4videodec.h"
#include "mpeg4videodefs.h"
#include "decode.h"
#include "tx1_decode.h"

#include "libavutil/pixdesc.h"
#include "libavutil/tx1_host1x.h"

typedef struct TX1MPEG4DecodeContext {
    TX1DecodeContext core;

    AVTX1Map common_map;
    uint32_t coloc_off, history_off, scratch_off;
    uint32_t history_size, scratch_size;

    AVFrame *current_frame, *prev_frame, *next_frame;
} TX1MPEG4DecodeContext;

/* Size (width, height) of a macroblock */
#define MB_SIZE 16

static const uint8_t bitstream_end_sequence[16] = {
    0x00, 0x00, 0x01, 0xb1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xb1, 0x00, 0x00, 0x00, 0x00,
};

static int tx1_mpeg4_decode_uninit(AVCodecContext *avctx) {
    TX1MPEG4DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    int err;

    av_log(avctx, AV_LOG_DEBUG, "Deinitializing TX1 MPEG4 decoder\n");

    err = ff_tx1_map_destroy(&ctx->common_map);
    if (err < 0)
        return err;

    err = ff_tx1_decode_uninit(avctx, &ctx->core);
    if (err < 0)
        return err;

    return 0;
}

static int tx1_mpeg4_decode_init(AVCodecContext *avctx) {
    TX1MPEG4DecodeContext *ctx = avctx->internal->hwaccel_priv_data;
#ifdef __SWITCH__
    AVHWDeviceContext *hw_device_ctx;
    AVTX1DeviceContext *device_hwctx;
#endif

    uint32_t width_in_mbs, height_in_mbs,
             coloc_size, history_size, scratch_size, common_map_size;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Initializing TX1 MPEG4 decoder\n");

    /* Ignored: histogram map, size 0x400 */
    ctx->core.pic_setup_off  = 0;
    ctx->core.status_off     = FFALIGN(ctx->core.pic_setup_off + sizeof(nvdec_mpeg4_pic_s),
                                       FF_TX1_MAP_ALIGN);
    ctx->core.cmdbuf_off     = FFALIGN(ctx->core.status_off    + sizeof(nvdec_status_s),
                                       FF_TX1_MAP_ALIGN);
    ctx->core.bitstream_off  = FFALIGN(ctx->core.cmdbuf_off    + FF_TX1_MAP_ALIGN,
                                       FF_TX1_MAP_ALIGN);
    ctx->core.input_map_size = FFALIGN(ctx->core.bitstream_off + ff_tx1_decode_pick_bitstream_buffer_size(avctx),
                                       0x1000);

    ctx->core.max_cmdbuf_size    =  ctx->core.bitstream_off  - ctx->core.cmdbuf_off;
    ctx->core.max_bitstream_size =  ctx->core.input_map_size - ctx->core.bitstream_off;

    err = ff_tx1_decode_init(avctx, &ctx->core);
    if (err < 0)
        goto fail;

    width_in_mbs        = FFALIGN(avctx->width,  MB_SIZE) / MB_SIZE;
    height_in_mbs       = FFALIGN(avctx->height, MB_SIZE) / MB_SIZE;
    coloc_size          = FFALIGN(FFALIGN(height_in_mbs, 2) * (width_in_mbs * 64) - 63, 0x100);
    history_size        = FFALIGN(width_in_mbs * 0x100 + 0x1100, 0x100);
    scratch_size        = 0x400;

    ctx->coloc_off   = 0;
    ctx->history_off = FFALIGN(ctx->coloc_off   + coloc_size,   FF_TX1_MAP_ALIGN);
    ctx->scratch_off = FFALIGN(ctx->history_off + history_size, FF_TX1_MAP_ALIGN);
    common_map_size  = FFALIGN(ctx->scratch_off + scratch_size, 0x1000);

#ifdef __SWITCH__
    hw_device_ctx = (AVHWDeviceContext *)ctx->core.hw_device_ref->data;
    device_hwctx  = hw_device_ctx->hwctx;

    ctx->common_map.owner = device_hwctx->nvdec_channel.channel.fd;
#endif

    err = ff_tx1_map_create(&ctx->common_map, common_map_size, 0x100, NVMAP_CACHE_OP_INV);
    if (err < 0)
        goto fail;

    ctx->history_size = history_size;
    ctx->scratch_size = scratch_size;

    return 0;

fail:
    tx1_mpeg4_decode_uninit(avctx);
    return err;
}

static void tx1_mpeg4_prepare_frame_setup(nvdec_mpeg4_pic_s *setup, AVCodecContext *avctx,
                                          TX1MPEG4DecodeContext *ctx)
{
    Mpeg4DecContext *m = avctx->priv_data;
    MpegEncContext  *s = &m->m;

    int i;

    *setup = (nvdec_mpeg4_pic_s){
        .scratch_pic_buffer_size      = ctx->scratch_size,

        .gptimer_timeout_value        = 0, /* Default value */

        .FrameWidth                   = FFALIGN(s->width,  MB_SIZE),
        .FrameHeight                  = FFALIGN(s->height, MB_SIZE),

        .vop_time_increment_bitcount  = m->time_increment_bits,
        .resync_marker_disable        = !m->resync_marker,

        .tileFormat                   = 0, /* TBL */
        .gob_height                   = 0, /* GOB_2 */

        .width                        = FFALIGN(s->width,  MB_SIZE),
        .height                       = FFALIGN(s->height, MB_SIZE),

        .FrameStride                  = {
            s->current_picture.f->linesize[0],
            s->current_picture.f->linesize[1],
        },

        .luma_top_offset              = 0,
        .luma_bot_offset              = 0,
        .luma_frame_offset            = 0,
        .chroma_top_offset            = 0,
        .chroma_bot_offset            = 0,
        .chroma_frame_offset          = 0,

        .HistBufferSize               = ctx->history_size / 256,

        .trd                          = { s->pp_time, s->pp_field_time >> 1 },
        .trb                          = { s->pb_time, s->pb_field_time >> 1 },

        .vop_fcode_forward            = s->f_code,
        .vop_fcode_backward           = s->b_code,

        .interlaced                   = s->interlaced_dct,
        .quant_type                   = s->mpeg_quant,
        .quarter_sample               = s->quarter_sample,
        .short_video_header           = avctx->codec->id == AV_CODEC_ID_H263,

        .curr_output_memory_layout    = 0, /* NV12 */

        .ptype                        = s->pict_type - AV_PICTURE_TYPE_I,
        .rnd                          = s->no_rounding,
        .alternate_vertical_scan_flag = s->alternate_scan,

        .ref_memory_layout            = { 0, 0 }, /* NV12 */
    };

    for (i = 0; i < 64; ++i) {
        setup->intra_quant_mat   [i] = s->intra_matrix[i];
        setup->nonintra_quant_mat[i] = s->inter_matrix[i];
    }
}

static int tx1_mpeg4_prepare_cmdbuf(AVTX1Cmdbuf *cmdbuf, MpegEncContext *s, TX1MPEG4DecodeContext *ctx,
                                    AVFrame *cur_frame, AVFrame *prev_frame, AVFrame *next_frame)
{
    FrameDecodeData *fdd = (FrameDecodeData *)cur_frame->private_ref->data;
    TX1Frame         *tf = fdd->hwaccel_priv;
    AVTX1Map  *input_map = (AVTX1Map *)tf->input_map_ref->data;

    int err;

    err = ff_tx1_cmdbuf_begin(cmdbuf, HOST1X_CLASS_NVDEC);
    if (err < 0)
        return err;

    FF_TX1_PUSH_VALUE(cmdbuf, NVC5B0_SET_APPLICATION_ID,
                      FF_TX1_ENUM(NVC5B0_SET_APPLICATION_ID, ID, MPEG4));
    FF_TX1_PUSH_VALUE(cmdbuf, NVC5B0_SET_CONTROL_PARAMS,
                      FF_TX1_ENUM (NVC5B0_SET_CONTROL_PARAMS, CODEC_TYPE,     MPEG4) |
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

    FF_TX1_PUSH_RELOC(cmdbuf, NVC5B0_SET_COLOC_DATA_OFFSET,
                      &ctx->common_map, ctx->coloc_off,          NVHOST_RELOC_TYPE_DEFAULT);
    FF_TX1_PUSH_RELOC(cmdbuf, NVC5B0_SET_HISTORY_OFFSET,
                      &ctx->common_map, ctx->history_off,        NVHOST_RELOC_TYPE_DEFAULT);
    FF_TX1_PUSH_RELOC(cmdbuf, NVC5B0_SET_PIC_SCRATCH_BUF_OFFSET,
                      &ctx->common_map, ctx->scratch_off,        NVHOST_RELOC_TYPE_DEFAULT);

#define PUSH_FRAME(fr, offset) ({                                                   \
    FF_TX1_PUSH_RELOC(cmdbuf, NVC5B0_SET_PICTURE_LUMA_OFFSET0 + offset * 4,         \
                      ff_tx1_frame_get_fbuf_map(fr), 0, NVHOST_RELOC_TYPE_DEFAULT); \
    FF_TX1_PUSH_RELOC(cmdbuf, NVC5B0_SET_PICTURE_CHROMA_OFFSET0 + offset * 4,       \
                      ff_tx1_frame_get_fbuf_map(fr), fr->data[1] - fr->data[0],     \
                      NVHOST_RELOC_TYPE_DEFAULT);                                   \
})

    PUSH_FRAME(cur_frame,  0);
    PUSH_FRAME(prev_frame, 1);
    PUSH_FRAME(next_frame, 2);

    FF_TX1_PUSH_VALUE(cmdbuf, NVC5B0_EXECUTE,
                      FF_TX1_ENUM(NVC5B0_EXECUTE, AWAKEN, ENABLE));

    err = ff_tx1_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    return 0;
}

static int tx1_mpeg4_start_frame(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    Mpeg4DecContext         *m = avctx->priv_data;
    MpegEncContext          *s = &m->m;
    AVFrame             *frame = s->current_picture.f;
    FrameDecodeData       *fdd = (FrameDecodeData *)frame->private_ref->data;
    TX1MPEG4DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    TX1Frame *tf;
    AVTX1Map *input_map;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Starting MPEG4-TX1 frame with pixel format %s\n",
           av_get_pix_fmt_name(avctx->sw_pix_fmt));

    err = ff_tx1_start_frame(avctx, frame, &ctx->core);
    if (err < 0)
        return err;

    tf = fdd->hwaccel_priv;
    input_map = (AVTX1Map *)tf->input_map_ref->data;
    mem = ff_tx1_map_get_addr(input_map);

    tx1_mpeg4_prepare_frame_setup((nvdec_mpeg4_pic_s *)(mem + ctx->core.pic_setup_off), avctx, ctx);

    ctx->prev_frame    = (s->pict_type != AV_PICTURE_TYPE_I) ? s->last_picture.f : frame;
    ctx->next_frame    = (s->pict_type == AV_PICTURE_TYPE_B) ? s->next_picture.f : frame;
    ctx->current_frame = frame;

    return 0;
}

static int tx1_mpeg4_end_frame(AVCodecContext *avctx) {
    Mpeg4DecContext         *m = avctx->priv_data;
    MpegEncContext          *s = &m->m;
    TX1MPEG4DecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    AVFrame             *frame = ctx->current_frame;
    FrameDecodeData       *fdd = (FrameDecodeData *)frame->private_ref->data;
    TX1Frame               *tf = fdd->hwaccel_priv;
    AVTX1Map        *input_map = (AVTX1Map *)tf->input_map_ref->data;

    nvdec_mpeg4_pic_s *setup;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Ending MPEG4-TX1 frame with %u slices -> %u bytes\n",
           ctx->core.num_slices, ctx->core.bitstream_len);

    mem = ff_tx1_map_get_addr(input_map);

    setup = (nvdec_mpeg4_pic_s *)(mem + ctx->core.pic_setup_off);
    setup->stream_len  = ctx->core.bitstream_len + sizeof(bitstream_end_sequence);
    setup->slice_count = ctx->core.num_slices;

    err = tx1_mpeg4_prepare_cmdbuf(&ctx->core.cmdbuf, s, ctx, frame,
                                   ctx->prev_frame, ctx->next_frame);
    if (err < 0)
        return err;

    return ff_tx1_end_frame(avctx, frame, &ctx->core, bitstream_end_sequence,
                            sizeof(bitstream_end_sequence));
}

static int tx1_mpeg4_decode_slice(AVCodecContext *avctx, const uint8_t *buf,
                                  uint32_t buf_size)
{
    TX1MPEG4DecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    AVFrame             *frame = ctx->current_frame;

    /* Rewind the bitstream looking for the VOP start marker */
    while (*(uint32_t *)buf != AV_BE2NE32C(VOP_STARTCODE))
        buf -= 1, buf_size += 1;

    return ff_tx1_decode_slice(avctx, frame, buf, buf_size, false);
}

#if CONFIG_MPEG4_TX1_HWACCEL
const FFHWAccel ff_mpeg4_tx1_hwaccel = {
    .p.name               = "mpeg4_tx1",
    .p.type               = AVMEDIA_TYPE_VIDEO,
    .p.id                 = AV_CODEC_ID_MPEG4,
    .p.pix_fmt            = AV_PIX_FMT_TX1,
    .start_frame          = &tx1_mpeg4_start_frame,
    .end_frame            = &tx1_mpeg4_end_frame,
    .decode_slice         = &tx1_mpeg4_decode_slice,
    .init                 = &tx1_mpeg4_decode_init,
    .uninit               = &tx1_mpeg4_decode_uninit,
    .frame_params         = &ff_tx1_frame_params,
    .priv_data_size       = sizeof(TX1MPEG4DecodeContext),
    .caps_internal        = HWACCEL_CAP_ASYNC_SAFE,
};
#endif
