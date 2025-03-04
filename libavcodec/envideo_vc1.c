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
#include "vc1.h"
#include "decode.h"

/* Hack to work around duplicate enumeration name */
#define DQPROFILE_SINGLE_EDGE DQPROFILE_SINGLE_EDGE_
#include "envideo_decode.h"

#include "libavutil/pixdesc.h"

typedef struct EnvideoVC1DecodeContextShared {
    EnvideoMap *common_map;
    uint32_t coloc_off, history_off, scratch_off;
    uint32_t history_size, scratch_size;
} EnvideoVC1DecodeContextShared;

typedef struct EnvideoVC1DecodeContext {
    FFEnvideoDecodeContext core;
    EnvideoVC1DecodeContextShared *shared;

    bool is_first_slice;

    AVFrame *prev_frame, *next_frame;
} EnvideoVC1DecodeContext;

/* Size (width, height) of a macroblock */
#define MB_SIZE 16

static const uint8_t bitstream_end_sequence[] = {
    0x00, 0x00, 0x01, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0a, 0x00, 0x00, 0x00, 0x00,
};

static int envideo_vc1_decode_uninit(AVCodecContext *avctx) {
    EnvideoVC1DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    int err;

    av_log(avctx, AV_LOG_DEBUG, "Deinitializing vc1-envideo decoder\n");

    av_refstruct_unref(&ctx->shared);

    err = ff_envideo_decode_uninit(avctx, &ctx->core);
    if (err < 0)
        return err;

    return 0;
}

static void envideo_vc1_shared_free(AVRefStructOpaque opaque, void *obj) {
    EnvideoVC1DecodeContextShared *shared = obj;

    envideo_map_destroy(shared->common_map);
}

static int envideo_vc1_decode_init(AVCodecContext *avctx) {
    EnvideoVC1DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    AVHWDeviceContext      *hw_device_ctx;
    AVEnvideoDeviceContext *device_hwctx;
    EnvideoVC1DecodeContextShared *ss;
    FFEnvideoDecodeContextShared *sc;
    uint32_t width_in_mbs, height_in_mbs, num_slices,
             coloc_size, history_size, scratch_size, common_map_size;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Initializing vc1-envideo decoder\n");

    ctx->shared = av_refstruct_alloc_ext(sizeof(*ctx->shared), 0, NULL, envideo_vc1_shared_free);
    if (!ctx->shared) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    err = ff_envideo_alloc_shared(&ctx->core);
    if (err < 0)
        goto fail;

    ss = ctx->shared;
    sc = ctx->core.shared;

    width_in_mbs  = FFALIGN(avctx->coded_width,  MB_SIZE) / MB_SIZE;
    height_in_mbs = FFALIGN(avctx->coded_height, MB_SIZE) / MB_SIZE;

    num_slices = width_in_mbs * height_in_mbs;

    /* Ignored: histogram map, size 0x400 */
    sc->pic_setup_off        = 0;
    sc->status_off           = FFALIGN(sc->pic_setup_off     + sizeof(nvdec_vc1_pic_s),
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

    hw_device_ctx = (AVHWDeviceContext *)sc->hw_device_ref->data;
    device_hwctx  = hw_device_ctx->hwctx;

    coloc_size   = 3 * FFALIGN(width_in_mbs * FFALIGN(height_in_mbs, 2) * 0x40 - 0x3f, ENVIDEO_MAP_ALIGN);
    history_size = FFALIGN(width_in_mbs, 2) * 0x300;
    scratch_size = 0x400;

    ss->coloc_off   = 0;
    ss->history_off = FFALIGN(ss->coloc_off   + coloc_size,   ENVIDEO_MAP_ALIGN);
    ss->scratch_off = FFALIGN(ss->history_off + history_size, ENVIDEO_MAP_ALIGN);
    common_map_size = FFALIGN(ss->scratch_off + scratch_size, 0x1000);

    err = envideo_map_create(device_hwctx->device, &ss->common_map, common_map_size, ENVIDEO_MAP_ALIGN,
                             EnvideoMap_CpuWriteCombine | EnvideoMap_GpuCacheable | EnvideoMap_UsageEngine);
    if (err < 0)
        goto fail;

    err = envideo_map_pin(ss->common_map, sc->channel);
    if (err < 0)
        goto fail;

    mem = envideo_map_get_cpu_addr(ss->common_map);

    memset(mem + ss->coloc_off,   0, coloc_size);
    memset(mem + ss->history_off, 0, history_size);
    memset(mem + ss->scratch_off, 0, scratch_size);

    ss->history_size = history_size;
    ss->scratch_size = scratch_size;

    return 0;

fail:
    envideo_vc1_decode_uninit(avctx);
    return err;
}

static void envideo_vc1_prepare_frame_setup(nvdec_vc1_pic_s *setup, AVCodecContext *avctx,
                                            EnvideoVC1DecodeContext *ctx)
{
    VC1Context     *v = avctx->priv_data;
    MpegEncContext *s = &v->s;
    AVFrame    *frame = s->cur_pic.ptr->f;

    /**
     * Notes:
     * - s->current_picture.f->linesize is unconsistently doubled for interlaced content
     *   between I-frames and others, so s->current_pic_ptr is used
     * - a lot of fields in this structure are unused by official software,
     *   here we reproduce this logic.
     */
    *setup = (nvdec_vc1_pic_s){
        .scratch_pic_buffer_size = ctx->shared->scratch_size,

        .gptimer_timeout_value   = 0, /* Default value */

        .bitstream_offset        = 0,

        .FrameStride             = {
            frame->linesize[0],
            frame->linesize[1],
        },

        .luma_top_offset         = 0,
        .luma_bot_offset         = 0,
        .luma_frame_offset       = 0,
        .chroma_top_offset       = 0,
        .chroma_bot_offset       = 0,
        .chroma_frame_offset     = 0,

        .CodedWidth              = FFALIGN(avctx->coded_width,
                                           (v->profile == PROFILE_ADVANCED) ? 1 : MB_SIZE),
        .CodedHeight             = FFALIGN(avctx->coded_height,
                                           (v->profile == PROFILE_ADVANCED) ? 1 : MB_SIZE),

        .HistBufferSize          = ctx->shared->history_size / 256,

        .loopfilter              = s->loop_filter,

        .output_memory_layout    = 0, /* NV12 */
        .ref_memory_layout       = {
            0, 0, /* NV12 */
        },

        .fastuvmc                = v->fastuvmc,

        .FrameWidth              = FFALIGN(frame->width,
                                           (v->profile == PROFILE_ADVANCED) ? 1 : MB_SIZE),
        .FrameHeight             = FFALIGN(frame->height,
                                           (v->profile == PROFILE_ADVANCED) ? 1 : MB_SIZE),

        .profile                 = (v->profile != PROFILE_ADVANCED) ? 1 : 2,

        .postprocflag            = v->postprocflag,
        .pulldown                = v->broadcast,
        .interlace               = v->interlace,

        .tfcntrflag              = v->tfcntrflag,
        .finterpflag             = v->finterpflag,

        .tileFormat              = 0, /* TBL */

        .psf                     = v->psf,

        .multires                = v->multires,
        .syncmarker              = v->resync_marker,
        .rangered                = v->rangered,
        .maxbframes              = s->max_b_frames,
        .panscan_flag            = v->panscanflag,
        .dquant                  = v->dquant,
        .refdist_flag            = v->refdist_flag,
        .quantizer               = v->quantizer_mode,
        .overlap                 = v->overlap,
        .vstransform             = v->vstransform,
        .extended_mv             = v->extended_mv,
        .extended_dmv            = v->extended_dmv,
    };

    if (v->profile == PROFILE_ADVANCED) {
        setup->displayPara.enableTFOutput = 1;
        setup->displayPara.VC1MapYFlag    = v->range_mapy_flag;
        setup->displayPara.MapYValue      = v->range_mapy;
        setup->displayPara.VC1MapUVFlag   = v->range_mapuv_flag;
        setup->displayPara.MapUVValue     = v->range_mapuv;
    } else if (v->rangered && v->rangeredfrm) {
        setup->displayPara.enableTFOutput = 1;
        setup->displayPara.VC1MapYFlag    = 1;
        setup->displayPara.MapYValue      = 7;
        setup->displayPara.VC1MapUVFlag   = 1;
        setup->displayPara.MapUVValue     = 7;
    }

    if (v->range_mapy_flag || v->range_mapuv_flag) {
        setup->displayPara.OutputBottom[0] = 0;
        setup->displayPara.OutputBottom[1] = 0;
        setup->displayPara.OutputStructure = v->interlace & 1;
        setup->displayPara.OutStride       = frame->linesize[0] & 0xff;
    }
}

static int envideo_vc1_prepare_cmdbuf(EnvideoCmdbuf *cmdbuf, VC1Context *v, EnvideoVC1DecodeContext *ctx,
                                      AVFrame *cur_frame, AVFrame *prev_frame, AVFrame *next_frame)
{
    EnvideoVC1DecodeContextShared *ss = ctx->shared;
    FFEnvideoDecodeContextShared  *sc = ctx->core.shared;
    FrameDecodeData              *fdd = (FrameDecodeData *)cur_frame->private_ref->data;
    FFEnvideoDecodeFrame          *tf = fdd->hwaccel_priv;
    AVEnvideoJob                 *job = (AVEnvideoJob *)tf->operation.job_ref->data;
    EnvideoMap             *input_map = job->input_map;

    int err;

    err = envideo_cmdbuf_begin(cmdbuf, EnvideoEngine_Nvdec);
    if (err < 0)
        return err;

    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_APPLICATION_ID,
                          DRF_DEF(C9B0, _SET_APPLICATION_ID, _ID, _VC1));
    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_CONTROL_PARAMS,
                          DRF_DEF(C9B0, _SET_CONTROL_PARAMS, _CODEC_TYPE,     _VC1)  |
                          DRF_NUM(C9B0, _SET_CONTROL_PARAMS, _ERR_CONCEAL_ON, 1)     |
                          DRF_NUM(C9B0, _SET_CONTROL_PARAMS, _GPTIMER_ON,     1));
    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_PICTURE_INDEX,
                          DRF_NUM(C9B0, _SET_PICTURE_INDEX, _INDEX, ctx->core.frame_idx));

    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_DRV_PIC_SETUP_OFFSET,     input_map, sc->pic_setup_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_IN_BUF_BASE_OFFSET,       input_map, sc->bitstream_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_SLICE_OFFSETS_BUF_OFFSET, input_map, sc->slice_offsets_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_NVDEC_STATUS_OFFSET,      input_map, sc->status_off);

    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_COLOC_DATA_OFFSET,      ss->common_map, ss->coloc_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_HISTORY_OFFSET,         ss->common_map, ss->history_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_PIC_SCRATCH_BUF_OFFSET, ss->common_map, ss->scratch_off);

#define PUSH_FRAME(fr, offset) ({                                                               \
    FF_ENVIDEO_PUSH_RELOC_TILED(cmdbuf, NVC9B0_SET_PICTURE_LUMA_OFFSET0   + offset * 4,         \
                                av_envideo_frame_get_fbuf_map(fr), 0);                          \
    FF_ENVIDEO_PUSH_RELOC_TILED(cmdbuf, NVC9B0_SET_PICTURE_CHROMA_OFFSET0 + offset * 4,         \
                                av_envideo_frame_get_fbuf_map(fr), fr->data[1] - fr->data[0]);  \
})

    PUSH_FRAME(cur_frame,  0);
    PUSH_FRAME(prev_frame, 1);
    PUSH_FRAME(next_frame, 2);

    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_EXECUTE,
                          DRF_DEF(C9B0, _EXECUTE, _AWAKEN, _ENABLE));

    err = envideo_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    return 0;
}

static int envideo_vc1_start_frame(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    VC1Context                *v = avctx->priv_data;
    MpegEncContext            *s = &v->s;
    AVFrame               *frame = s->cur_pic.ptr->f;
    FrameDecodeData         *fdd = (FrameDecodeData *)frame->private_ref->data;
    EnvideoVC1DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    FFEnvideoDecodeFrame *tf;
    AVEnvideoJob *job;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Starting vc1-envideo frame with pixel format %s\n",
           av_get_pix_fmt_name(avctx->sw_pix_fmt));

    ctx->is_first_slice = true;

    err = ff_envideo_start_frame(avctx, frame, &ctx->core);
    if (err < 0)
        return err;

    tf  = fdd->hwaccel_priv;
    job = (AVEnvideoJob *)tf->operation.job_ref->data;
    mem = envideo_map_get_cpu_addr(job->input_map);

    envideo_vc1_prepare_frame_setup((nvdec_vc1_pic_s *)(mem + ctx->core.shared->pic_setup_off), avctx, ctx);

    ctx->prev_frame = ff_envideo_safe_get_ref(s->last_pic.ptr ? s->last_pic.ptr->f : frame, frame);
    ctx->next_frame = ff_envideo_safe_get_ref(s->next_pic.ptr ? s->next_pic.ptr->f : frame, frame);

    return 0;
}

static int envideo_vc1_end_frame(AVCodecContext *avctx) {
    VC1Context                *v = avctx->priv_data;
    EnvideoVC1DecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    AVFrame               *frame = v->s.cur_pic.ptr->f;
    FrameDecodeData         *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame     *tf = fdd->hwaccel_priv;
    AVEnvideoJob            *job = (AVEnvideoJob *)tf->operation.job_ref->data;

    nvdec_vc1_pic_s *setup;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Ending vc1-envideo frame with %u slices -> %u bytes\n",
           tf->operation.num_slices, tf->operation.bitstream_len);

    if (!tf || !tf->operation.num_slices)
        return 0;

    mem = envideo_map_get_cpu_addr(job->input_map);

    setup = (nvdec_vc1_pic_s *)(mem + ctx->core.shared->pic_setup_off);
    setup->stream_len  = tf->operation.bitstream_len + sizeof(bitstream_end_sequence);
    setup->slice_count = tf->operation.num_slices;

    err = envideo_vc1_prepare_cmdbuf(job->cmdbuf, v, ctx, frame,
                                     ctx->prev_frame, ctx->next_frame);
    if (err < 0)
        return err;

    return ff_envideo_end_frame(avctx, frame, &ctx->core, bitstream_end_sequence,
                                sizeof(bitstream_end_sequence));
}

static int envideo_vc1_decode_slice(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    VC1Context                    *v = avctx->priv_data;
    EnvideoVC1DecodeContext     *ctx = avctx->internal->hwaccel_priv_data;
    FFEnvideoDecodeContextShared *sc = ctx->core.shared;
    AVFrame                   *frame = v->s.cur_pic.ptr->f;
    FrameDecodeData             *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame         *tf = fdd->hwaccel_priv;
    AVEnvideoJob                *job = (AVEnvideoJob *)tf->operation.job_ref->data;

    uint8_t *mem;
    enum VC1Code marker;

    mem = envideo_map_get_cpu_addr(job->input_map);

    if (!ctx->is_first_slice)
        marker = VC1_CODE_SLICE;
    else if (v->profile == PROFILE_ADVANCED &&
                v->fcm == ILACE_FIELD && v->second_field)
        marker = VC1_CODE_FIELD;
    else
        marker = VC1_CODE_FRAME;

    if (AV_RB32(buf) != marker) {
        AV_WB32(mem + sc->bitstream_off + tf->operation.bitstream_len, marker);
        tf->operation.bitstream_len += sizeof(marker);
    }

    ctx->is_first_slice = false;

    return ff_envideo_decode_slice(avctx, frame, buf, buf_size, false);
}

static int envideo_vc1_update_thread_context(AVCodecContext *dst, const AVCodecContext *src) {
    EnvideoVC1DecodeContext *src_ctx = src->internal->hwaccel_priv_data;
    EnvideoVC1DecodeContext *dst_ctx = dst->internal->hwaccel_priv_data;

    av_refstruct_replace(&dst_ctx->shared, src_ctx->shared);
    dst_ctx->is_first_slice = src_ctx->is_first_slice;
    dst_ctx->prev_frame     = src_ctx->prev_frame;
    dst_ctx->next_frame     = src_ctx->next_frame;

    return ff_envideo_update_thread_context(&dst_ctx->core, &src_ctx->core);
}

#if CONFIG_VC1_ENVIDEO_HWACCEL
const FFHWAccel ff_vc1_envideo_hwaccel = {
    .p.name                = "vc1_envideo",
    .p.type                = AVMEDIA_TYPE_VIDEO,
    .p.id                  = AV_CODEC_ID_VC1,
    .p.pix_fmt             = AV_PIX_FMT_ENVIDEO,
    .start_frame           = &envideo_vc1_start_frame,
    .end_frame             = &envideo_vc1_end_frame,
    .decode_slice          = &envideo_vc1_decode_slice,
    .init                  = &envideo_vc1_decode_init,
    .uninit                = &envideo_vc1_decode_uninit,
    .frame_params          = &ff_envideo_frame_params,
    .update_thread_context = &envideo_vc1_update_thread_context,
    .priv_data_size        = sizeof(EnvideoVC1DecodeContext),
    .caps_internal         = HWACCEL_CAP_ASYNC_SAFE | HWACCEL_CAP_THREAD_SAFE,
};
#endif

#if CONFIG_WMV3_ENVIDEO_HWACCEL
const FFHWAccel ff_wmv3_envideo_hwaccel = {
    .p.name                = "wmv3_envideo",
    .p.type                = AVMEDIA_TYPE_VIDEO,
    .p.id                  = AV_CODEC_ID_WMV3,
    .p.pix_fmt             = AV_PIX_FMT_ENVIDEO,
    .start_frame           = &envideo_vc1_start_frame,
    .end_frame             = &envideo_vc1_end_frame,
    .decode_slice          = &envideo_vc1_decode_slice,
    .init                  = &envideo_vc1_decode_init,
    .uninit                = &envideo_vc1_decode_uninit,
    .frame_params          = &ff_envideo_frame_params,
    .update_thread_context = &envideo_vc1_update_thread_context,
    .priv_data_size        = sizeof(EnvideoVC1DecodeContext),
    .caps_internal         = HWACCEL_CAP_ASYNC_SAFE | HWACCEL_CAP_THREAD_SAFE,
};
#endif
