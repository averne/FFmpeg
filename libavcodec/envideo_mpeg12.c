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
#include "mpegvideo.h"
#include "mpegutils.h"
#include "decode.h"
#include "envideo_decode.h"

#include "libavutil/intmath.h"
#include "libavutil/pixdesc.h"

typedef struct EnvideoMPEG12DecodeContext {
    FFEnvideoDecodeContext core;

    AVFrame *prev_frame, *next_frame;
} EnvideoMPEG12DecodeContext;

/* Size (width, height) of a macroblock */
#define MB_SIZE 16

#define SECOND_FIELD(s) ((s)->picture_structure != PICT_FRAME && !(s)->first_field)

static const uint8_t bitstream_end_sequence[16] = {
    0x00, 0x00, 0x01, 0xb7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xb7, 0x00, 0x00, 0x00, 0x00,
};

static int envideo_mpeg12_decode_uninit(AVCodecContext *avctx) {
    EnvideoMPEG12DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    int err;

    av_log(avctx, AV_LOG_DEBUG, "Deinitializing mpeg12-envideo decoder\n");

    err = ff_envideo_decode_uninit(avctx, &ctx->core);
    if (err < 0)
        return err;

    return 0;
}

static int envideo_mpeg12_decode_init(AVCodecContext *avctx) {
    EnvideoMPEG12DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    FFEnvideoDecodeContextShared *sc;
    uint32_t num_slices;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Initializing mpeg12-envideo decoder\n");

    err = ff_envideo_alloc_shared(&ctx->core);
    if (err < 0)
        goto fail;

    sc = ctx->core.shared;

    num_slices = (FFALIGN(avctx->coded_width,  MB_SIZE) / MB_SIZE) *
                 (FFALIGN(avctx->coded_height, MB_SIZE) / MB_SIZE);
    num_slices = FFMIN(num_slices, 8160);

    /* Ignored: histogram map, size 0x400 */
    sc->pic_setup_off        = 0;
    sc->status_off           = FFALIGN(sc->pic_setup_off     + sizeof(nvdec_mpeg2_pic_s),
                                       ENVIDEO_MAP_ALIGN);
    sc->cmdbuf_off           = FFALIGN(sc->status_off        + sizeof(nvdec_status_s),
                                       ENVIDEO_MAP_ALIGN);
    sc->slice_offsets_off    = FFALIGN(sc->cmdbuf_off        + ENVIDEO_MAP_ALIGN,
                                       ENVIDEO_MAP_ALIGN);
    sc->bitstream_off        = FFALIGN(sc->slice_offsets_off + num_slices * sizeof(uint32_t),
                                       ENVIDEO_MAP_ALIGN);
    ctx->core.input_map_size = FFALIGN(sc->bitstream_off     + ff_envideo_decode_pick_bitstream_buffer_size(avctx),
                                       0x1000);

    sc->max_cmdbuf_size          =  sc->slice_offsets_off    - sc->cmdbuf_off;
    sc->max_num_slices           = (sc->bitstream_off        - sc->slice_offsets_off) / sizeof(uint32_t);
    ctx->core.max_bitstream_size =  ctx->core.input_map_size - sc->bitstream_off;

    err = ff_envideo_decode_init(avctx, &ctx->core);
    if (err < 0)
        goto fail;

    return 0;

fail:
    envideo_mpeg12_decode_uninit(avctx);
    return err;
}

static void envideo_mpeg12_prepare_frame_setup(nvdec_mpeg2_pic_s *setup, MpegEncContext *s,
                                               EnvideoMPEG12DecodeContext *ctx)
{
    AVFrame          *frame = s->cur_pic.ptr->f;
    AVEnvideoFrame *evframe = (AVEnvideoFrame *)frame->buf[0]->data;

    *setup = (nvdec_mpeg2_pic_s){
        .gptimer_timeout_value      = 0, /* Default value */

        .FrameWidth                 = FFALIGN(s->width,  MB_SIZE),
        .FrameHeight                = FFALIGN(s->height, MB_SIZE),

        .picture_structure          = s->picture_structure,
        .picture_coding_type        = s->pict_type,
        .intra_dc_precision         = s->intra_dc_precision,
        .frame_pred_frame_dct       = s->frame_pred_frame_dct,
        .concealment_motion_vectors = s->concealment_motion_vectors,
        .intra_vlc_format           = s->intra_vlc_format,

        .tileFormat                 = !ctx->core.shared->is_tegra, /* Tegra/GPU block linear */
        .gob_height                 = ff_ctz(evframe->gob_height) - 1,

        .f_code                     = {
            s->mpeg_f_code[0][0], s->mpeg_f_code[0][1],
            s->mpeg_f_code[1][0], s->mpeg_f_code[1][1],
        },

        .PicWidthInMbs              = FFALIGN(s->width,  MB_SIZE) / MB_SIZE,
        .FrameHeightInMbs           = FFALIGN(s->height, MB_SIZE) / MB_SIZE,
        .pitch_luma                 = frame->linesize[0],
        .pitch_chroma               = frame->linesize[1],
        .luma_top_offset            = 0,
        .luma_bot_offset            = 0,
        .luma_frame_offset          = 0,
        .chroma_top_offset          = 0,
        .chroma_bot_offset          = 0,
        .chroma_frame_offset        = 0,
        .alternate_scan             = s->alternate_scan,
        .secondfield                = SECOND_FIELD(s),
        .rounding_type              = 0,
        .q_scale_type               = s->q_scale_type,
        .top_field_first            = s->top_field_first,
        .full_pel_fwd_vector        = (s->codec_id != AV_CODEC_ID_MPEG2VIDEO) ? s->full_pel[0] : 0,
        .full_pel_bwd_vector        = (s->codec_id != AV_CODEC_ID_MPEG2VIDEO) ? s->full_pel[1] : 0,
        .output_memory_layout       = 0,        /* NV12 */
        .ref_memory_layout          = { 0, 0 }, /* NV12 */
    };

    for (int i = 0; i < FF_ARRAY_ELEMS(setup->quant_mat_8x8intra); ++i) {
        setup->quant_mat_8x8intra   [i] = (uint8_t)s->intra_matrix[i];
        setup->quant_mat_8x8nonintra[i] = (uint8_t)s->inter_matrix[i];
    }
}

static int envideo_mpeg12_prepare_cmdbuf(EnvideoCmdbuf *cmdbuf, MpegEncContext *s, EnvideoMPEG12DecodeContext *ctx,
                                         AVFrame *current_frame, AVFrame *prev_frame, AVFrame *next_frame)
{
    FFEnvideoDecodeContextShared *sc = ctx->core.shared;
    FFEnvideoDecodeField      *field = ff_envideo_get_priv(current_frame, SECOND_FIELD(s));
    AVEnvideoJob                *job = (AVEnvideoJob *)field->operation.job_ref->data;
    EnvideoMap            *input_map = job->input_map;

    int err, codec_id;

    err = envideo_cmdbuf_begin(cmdbuf, EnvideoEngine_Nvdec);
    if (err < 0)
        return err;

    switch (s->codec_id) {
        case AV_CODEC_ID_MPEG1VIDEO:
            codec_id = DRF_DEF(C9B0, _SET_CONTROL_PARAMS, _CODEC_TYPE, _MPEG1);
            break;
        case AV_CODEC_ID_MPEG2VIDEO:
            codec_id = DRF_DEF(C9B0, _SET_CONTROL_PARAMS, _CODEC_TYPE, _MPEG2);
            break;
        default:
            return AVERROR(EINVAL);
    }

    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_APPLICATION_ID,
                          DRF_DEF(C9B0, _SET_APPLICATION_ID, _ID, _MPEG12));
    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_CONTROL_PARAMS, codec_id            |
                          DRF_NUM(C9B0, _SET_CONTROL_PARAMS, _ERR_CONCEAL_ON, 1) |
                          DRF_NUM(C9B0, _SET_CONTROL_PARAMS, _GPTIMER_ON,     1));
    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_PICTURE_INDEX,
                          DRF_NUM(C9B0, _SET_PICTURE_INDEX, _INDEX, ctx->core.frame_idx));

    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_DRV_PIC_SETUP_OFFSET,     input_map, sc->pic_setup_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_IN_BUF_BASE_OFFSET,       input_map, sc->bitstream_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_SLICE_OFFSETS_BUF_OFFSET, input_map, sc->slice_offsets_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_NVDEC_STATUS_OFFSET,      input_map, sc->status_off);

#define PUSH_FRAME(fr, offset) ({                                                               \
    FF_ENVIDEO_PUSH_RELOC_TILED(cmdbuf, NVC9B0_SET_PICTURE_LUMA_OFFSET0   + offset * 4,         \
                                av_envideo_frame_get_fbuf_map(fr), 0);                          \
    FF_ENVIDEO_PUSH_RELOC_TILED(cmdbuf, NVC9B0_SET_PICTURE_CHROMA_OFFSET0 + offset * 4,         \
                                av_envideo_frame_get_fbuf_map(fr), fr->data[1] - fr->data[0]);  \
})

    PUSH_FRAME(current_frame, 0);
    PUSH_FRAME(prev_frame,    1);
    PUSH_FRAME(next_frame,    2);

    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_EXECUTE,
                          DRF_DEF(C9B0, _EXECUTE, _AWAKEN, _ENABLE));

    err = envideo_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    return 0;
}

static int envideo_mpeg12_start_frame(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    MpegEncContext               *s = avctx->priv_data;
    AVFrame                  *frame = s->cur_pic.ptr->f;
    EnvideoMPEG12DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    FFEnvideoDecodeField *field;
    AVEnvideoJob *job;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Starting mpeg12-envideo frame with pixel format %s\n",
           av_get_pix_fmt_name(avctx->sw_pix_fmt));

    err = ff_envideo_start_frame(avctx, frame, SECOND_FIELD(s), &ctx->core);
    if (err < 0)
        return err;

    field = ff_envideo_get_priv(frame, SECOND_FIELD(s));
    job   = (AVEnvideoJob *)field->operation.job_ref->data;
    mem   = envideo_map_get_cpu_addr(job->input_map);

    envideo_mpeg12_prepare_frame_setup((nvdec_mpeg2_pic_s *)(mem + ctx->core.shared->pic_setup_off), s, ctx);

    ctx->prev_frame = (s->pict_type != AV_PICTURE_TYPE_I && s->last_pic.ptr) ? s->last_pic.ptr->f : frame;
    ctx->next_frame = (s->pict_type == AV_PICTURE_TYPE_B && s->next_pic.ptr) ? s->next_pic.ptr->f : frame;

    return 0;
}

static int envideo_mpeg12_end_frame(AVCodecContext *avctx) {
    MpegEncContext               *s = avctx->priv_data;
    EnvideoMPEG12DecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    AVFrame                  *frame = s->cur_pic.ptr->f;
    FrameDecodeData            *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeField     *field = ff_envideo_get_priv(frame, SECOND_FIELD(s));

    AVEnvideoJob *job;
    FFEnvideoOperation *op;
    nvdec_mpeg2_pic_s *setup;
    uint8_t *mem;
    int err;

    if (!fdd || !field)
        return 0;

    job = (AVEnvideoJob *)field->operation.job_ref->data;
    op  = &field->operation;

    av_log(avctx, AV_LOG_DEBUG, "Ending mpeg12-envideo frame with %u slices -> %u bytes\n",
           op->num_slices, op->bitstream_len);

    mem = envideo_map_get_cpu_addr(job->input_map);

    setup = (nvdec_mpeg2_pic_s *)(mem + ctx->core.shared->pic_setup_off);
    setup->stream_len  = op->bitstream_len + sizeof(bitstream_end_sequence);
    setup->slice_count = op->num_slices;

    err = envideo_mpeg12_prepare_cmdbuf(job->cmdbuf, s, ctx, frame,
                                        ctx->prev_frame, ctx->next_frame);
    if (err < 0)
        return err;

    return ff_envideo_end_frame(avctx, frame, false, &ctx->core,
                                bitstream_end_sequence, sizeof(bitstream_end_sequence));
}

static int envideo_mpeg12_decode_slice(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    MpegEncContext *s = avctx->priv_data;
    AVFrame    *frame = s->cur_pic.ptr->f;

    return ff_envideo_decode_slice(avctx, frame, false, buf, buf_size, false);
}

static int envideo_mpeg12_update_thread_context(AVCodecContext *dst, const AVCodecContext *src) {
    EnvideoMPEG12DecodeContext *src_ctx = src->internal->hwaccel_priv_data;
    EnvideoMPEG12DecodeContext *dst_ctx = dst->internal->hwaccel_priv_data;

    return ff_envideo_update_thread_context(&dst_ctx->core, &src_ctx->core);
}

#if CONFIG_MPEG1_ENVIDEO_HWACCEL
const FFHWAccel ff_mpeg1_envideo_hwaccel = {
    .p.name                = "mpeg1_envideo",
    .p.type                = AVMEDIA_TYPE_VIDEO,
    .p.id                  = AV_CODEC_ID_MPEG1VIDEO,
    .p.pix_fmt             = AV_PIX_FMT_ENVIDEO,
    .start_frame           = &envideo_mpeg12_start_frame,
    .end_frame             = &envideo_mpeg12_end_frame,
    .decode_slice          = &envideo_mpeg12_decode_slice,
    .init                  = &envideo_mpeg12_decode_init,
    .uninit                = &envideo_mpeg12_decode_uninit,
    .frame_params          = &ff_envideo_frame_params,
    .update_thread_context = &envideo_mpeg12_update_thread_context,
    .priv_data_size        = sizeof(EnvideoMPEG12DecodeContext),
    .caps_internal         = HWACCEL_CAP_ASYNC_SAFE | HWACCEL_CAP_THREAD_SAFE,
};
#endif

#if CONFIG_MPEG2_ENVIDEO_HWACCEL
const FFHWAccel ff_mpeg2_envideo_hwaccel = {
    .p.name                = "mpeg2_envideo",
    .p.type                = AVMEDIA_TYPE_VIDEO,
    .p.id                  = AV_CODEC_ID_MPEG2VIDEO,
    .p.pix_fmt             = AV_PIX_FMT_ENVIDEO,
    .start_frame           = &envideo_mpeg12_start_frame,
    .end_frame             = &envideo_mpeg12_end_frame,
    .decode_slice          = &envideo_mpeg12_decode_slice,
    .init                  = &envideo_mpeg12_decode_init,
    .uninit                = &envideo_mpeg12_decode_uninit,
    .frame_params          = &ff_envideo_frame_params,
    .update_thread_context = &envideo_mpeg12_update_thread_context,
    .priv_data_size        = sizeof(EnvideoMPEG12DecodeContext),
    .caps_internal         = HWACCEL_CAP_ASYNC_SAFE | HWACCEL_CAP_THREAD_SAFE,
};
#endif
