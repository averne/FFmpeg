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
#include "h264dec.h"
#include "decode.h"
#include "envideo_decode.h"

#include "libavutil/pixdesc.h"
#include "libavutil/refstruct.h"

typedef struct EnvideoH264FrameData {
    uint8_t pic_idx;
    uint8_t dpb_idx;
    bool pic_initialized, dpb_initialized;
} EnvideoH264FrameData;

typedef struct EnvideoH264DecodeContextShared {
    EnvideoMap *common_map;
    uint32_t coloc_off, mbhist_off, history_off;
    uint32_t mbhist_size, history_size;
} EnvideoH264DecodeContextShared;

typedef struct EnvideoH264DecodeContext {
    FFEnvideoDecodeContext core;
    EnvideoH264DecodeContextShared *shared;

    H264Picture *dpb[16], *scratch_ref;
    uint32_t dpb_mask, pic_idx_mask;
} EnvideoH264DecodeContext;

/* Size (width, height) of a macroblock */
#define MB_SIZE 16

static const uint8_t bitstream_end_sequence[16] = {
    0x00, 0x00, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00,
};

static int envideo_h264_decode_uninit(AVCodecContext *avctx) {
    EnvideoH264DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    int err;

    av_log(avctx, AV_LOG_DEBUG, "Deinitializing h264-envideo decoder\n");

    av_refstruct_unref(&ctx->shared);

    err = ff_envideo_decode_uninit(avctx, &ctx->core);
    if (err < 0)
        return err;

    return 0;
}

static void envideo_h264_shared_free(AVRefStructOpaque opaque, void *obj) {
    EnvideoH264DecodeContextShared *shared = obj;

    envideo_map_destroy(shared->common_map);
}

static int envideo_h264_decode_init(AVCodecContext *avctx) {
    H264Context                *h = avctx->priv_data;
    const SPS                *sps = h->ps.sps;
    EnvideoH264DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    AVHWDeviceContext      *hw_device_ctx;
    AVEnvideoDeviceContext *device_hwctx;
    EnvideoH264DecodeContextShared *ss;
    FFEnvideoDecodeContextShared *sc;
    uint32_t aligned_width, aligned_height,
             width_in_mbs, height_in_mbs, num_slices,
             coloc_size, mbhist_size, history_size, common_map_size;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Initializing h264-envideo decoder\n");

    ctx->shared = av_refstruct_alloc_ext(sizeof(*ctx->shared), 0, NULL, envideo_h264_shared_free);
    if (!ctx->shared) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    err = ff_envideo_alloc_shared(&ctx->core);
    if (err < 0)
        goto fail;

    ss = ctx->shared;
    sc = ctx->core.shared;

    aligned_width  = FFALIGN(avctx->coded_width,  MB_SIZE);
    aligned_height = FFALIGN(avctx->coded_height, MB_SIZE);
    width_in_mbs   = aligned_width  / MB_SIZE;
    height_in_mbs  = aligned_height / MB_SIZE;

    num_slices = width_in_mbs * height_in_mbs;

    /* Ignored: histogram map, size 0x400 */
    sc->pic_setup_off        = 0;
    sc->status_off           = FFALIGN(sc->pic_setup_off     + sizeof(nvdec_h264_pic_s),
                                       ENVIDEO_MAP_ALIGN);
    sc->cmdbuf_off           = FFALIGN(sc->status_off        + sizeof(nvdec_status_s),
                                       ENVIDEO_MAP_ALIGN);
    sc->slice_offsets_off    = FFALIGN(sc->cmdbuf_off        + 3*ENVIDEO_MAP_ALIGN,
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

    coloc_size   = FFALIGN(FFALIGN(height_in_mbs, 2) * (width_in_mbs * 0x40) - 0x3f, 0x100);
    coloc_size  *= sps->ref_frame_count + 1; /* Max number of references frames, plus current frame */
    mbhist_size  = FFALIGN(width_in_mbs * 0x68, 0x100);
    history_size = FFALIGN(width_in_mbs * 0x200 + 0x1100, 0x200);

    ss->coloc_off   = 0;
    ss->mbhist_off  = FFALIGN(ss->coloc_off   + coloc_size,   ENVIDEO_MAP_ALIGN);
    ss->history_off = FFALIGN(ss->mbhist_off  + mbhist_size,  ENVIDEO_MAP_ALIGN);
    common_map_size = FFALIGN(ss->history_off + history_size, 0x1000);

    err = envideo_map_create(device_hwctx->device, &ss->common_map, common_map_size, ENVIDEO_MAP_ALIGN,
                             EnvideoMap_CpuWriteCombine | EnvideoMap_GpuCacheable | EnvideoMap_UsageEngine);
    if (err < 0)
        goto fail;

    err = envideo_map_pin(ss->common_map, sc->channel);
    if (err < 0)
        goto fail;

    ss->mbhist_size  = mbhist_size;
    ss->history_size = history_size;

    return 0;

fail:
    envideo_h264_decode_uninit(avctx);
    return err;
}

static inline int field_poc(int poc[2], bool top) {
    return (poc[!top] != INT_MAX) ? poc[!top] : 0;
}

static void dpb_add(H264Context *h, nvdec_dpb_entry_s *dst, H264Picture *src) {
    EnvideoH264FrameData *fr_priv = src->hwaccel_picture_private;
    int marking;

    marking = src->long_ref ? 2 : 1;
    *dst = (nvdec_dpb_entry_s){
        .index                = fr_priv->pic_idx,
        .col_idx              = fr_priv->pic_idx,
        .state                = src->reference,
        .is_long_term         = src->long_ref,
        .not_existing         = src->invalid_gap,
        .is_field             = src->field_picture,
        .top_field_marking    = (src->reference & PICT_TOP_FIELD)    ? marking : 0,
        .bottom_field_marking = (src->reference & PICT_BOTTOM_FIELD) ? marking : 0,
        .output_memory_layout = 0, /* NV12 */
        .FieldOrderCnt        = {
            field_poc(src->field_poc, true),
            field_poc(src->field_poc, false),
        },
        .FrameIdx             = src->long_ref ? src->pic_id : src->frame_num,
    };
}

static inline void register_ref(EnvideoH264DecodeContext *ctx, H264Picture *fr) {
    EnvideoH264FrameData *fr_priv = fr->hwaccel_picture_private;

    ctx->dpb[fr_priv->dpb_idx] = fr;
    ctx->dpb_mask     |= 1 << fr_priv->dpb_idx;
    ctx->pic_idx_mask |= 1 << fr_priv->pic_idx;
}

static inline int find_slot(uint32_t *mask) {
    int slot = ff_ctz(~*mask);
    *mask |= (1 << slot);
    return slot;
}

static void envideo_h264_prepare_frame_setup(nvdec_h264_pic_s *setup, H264Context *h,
                                             EnvideoH264DecodeContext *ctx)
{
    const PPS *pps = h->ps.pps;
    const SPS *sps = h->ps.sps;

    H264Picture *refs[16+1] = {0};
    EnvideoH264FrameData *fr_priv;
    int num_refs, max, i, diff;

    *setup = (nvdec_h264_pic_s){
        .mbhist_buffer_size                     = ctx->shared->mbhist_size,

        .gptimer_timeout_value                  = 0, /* Default value */

        .log2_max_pic_order_cnt_lsb_minus4      = FFMAX(sps->log2_max_poc_lsb - 4, 0),
        .delta_pic_order_always_zero_flag       = sps->delta_pic_order_always_zero_flag,
        .frame_mbs_only_flag                    = sps->frame_mbs_only_flag,

        .PicWidthInMbs                          = h->mb_width,
        .FrameHeightInMbs                       = h->mb_height,

        .tileFormat                             = 0, /* TBL */
        .gob_height                             = 0, /* GOB_2 */

        .entropy_coding_mode_flag               = pps->cabac,
        .pic_order_present_flag                 = pps->pic_order_present,
        .num_ref_idx_l0_active_minus1           = pps->ref_count[0] - 1,
        .num_ref_idx_l1_active_minus1           = pps->ref_count[1] - 1,
        .deblocking_filter_control_present_flag = pps->deblocking_filter_parameters_present,
        .redundant_pic_cnt_present_flag         = pps->redundant_pic_cnt_present,
        .transform_8x8_mode_flag                = pps->transform_8x8_mode,

        .pitch_luma                             = h->cur_pic_ptr->f->linesize[0],
        .pitch_chroma                           = h->cur_pic_ptr->f->linesize[1],

        .luma_top_offset                        = 0,
        .luma_bot_offset                        = 0,
        .luma_frame_offset                      = 0,
        .chroma_top_offset                      = 0,
        .chroma_bot_offset                      = 0,
        .chroma_frame_offset                    = 0,

        .HistBufferSize                         = ctx->shared->history_size / 256,

        .MbaffFrameFlag                         = sps->mb_aff && !FIELD_PICTURE(h),
        .direct_8x8_inference_flag              = sps->direct_8x8_inference_flag,
        .weighted_pred_flag                     = pps->weighted_pred,
        .constrained_intra_pred_flag            = pps->constrained_intra_pred,
        .ref_pic_flag                           = h->nal_ref_idc != 0,
        .field_pic_flag                         = FIELD_PICTURE(h),
        .bottom_field_flag                      = h->picture_structure == PICT_BOTTOM_FIELD,
        .second_field                           = FIELD_PICTURE(h) && !h->first_field,
        .log2_max_frame_num_minus4              = sps->log2_max_frame_num - 4,
        .chroma_format_idc                      = sps->chroma_format_idc,
        .pic_order_cnt_type                     = sps->poc_type,
        .pic_init_qp_minus26                    = pps->init_qp - 26,
        .chroma_qp_index_offset                 = pps->chroma_qp_index_offset[0],
        .second_chroma_qp_index_offset          = pps->chroma_qp_index_offset[1],

        .weighted_bipred_idc                    = pps->weighted_bipred_idc,
        .frame_num                              = h->cur_pic_ptr->frame_num,
        .output_memory_layout                   = 0, /* NV12 */

        .CurrFieldOrderCnt                      = {
            field_poc(h->cur_pic_ptr->field_poc, true),
            field_poc(h->cur_pic_ptr->field_poc, false),
        },

        .lossless_ipred8x8_filter_enable        = true,
        .qpprime_y_zero_transform_bypass_flag   = sps->transform_bypass,
    };

    /**
     * Decoded frames need to be allocated two indices, respectively for colocated
     * and decoded data (pic_idx). For simplicity the two are kept at the same value,
     * in both this code and the official driver. These indices must remain fixed
     * until the frame is dropped from the DPB.
     * Furthermore, a third index (dpb_idx) must be allocated when (and if) the
     * decoded frame is subsequently used as a reference.
     * Since decoding a frame will initialize its colocated data, but will not
     * insert it (yet) in the DPB array, this last index must be decoupled from
     * the previous two. The pic_idx is allocated when decoding a frame, while
     * dpb_idx is allocated when it is used as a reference.
     */

    /* Build concatenated list of references */
    num_refs = 0, max = FFMIN(h->short_ref_count, FF_ARRAY_ELEMS(refs) - num_refs);
    for (i = 0; i < max; ++i)
        refs[num_refs++] = h->short_ref[i];

    max = FFMIN(16, FF_ARRAY_ELEMS(refs) - num_refs);
    for (i = 0; i < max; ++i)
        if (h->long_ref[i])
            refs[num_refs++] = h->short_ref[i];

    /* Add all frames with an already allocated DPB index to our ref list */
    for (i = 0; i < num_refs; ++i) {
        EnvideoH264FrameData *fr_priv = refs[i]->hwaccel_picture_private;
        if (!fr_priv->dpb_initialized)
            continue;

        register_ref(ctx, refs[i]);
    }

    /* Allocate a DPB index for new frames and add them to our ref list */
    for (i = 0; i < num_refs; ++i) {
        EnvideoH264FrameData *fr_priv = refs[i]->hwaccel_picture_private;
        if (fr_priv->dpb_initialized || !fr_priv->pic_initialized)
            continue;

        fr_priv->dpb_idx         = find_slot(&ctx->dpb_mask);
        fr_priv->dpb_initialized = true;

        register_ref(ctx, refs[i]);
    }

    /* Allocate a picture idx for the current frame */
    fr_priv = h->cur_pic_ptr->hwaccel_picture_private;
    if (!fr_priv->pic_initialized) {
        *fr_priv = (EnvideoH264FrameData){
            .pic_idx         = find_slot(&ctx->pic_idx_mask),
            .pic_initialized = true,
        };
    }

    setup->CurrPicIdx = setup->CurrColIdx = fr_priv->pic_idx;

    /* Finally, fill the NVDEC DPB */
    for (i = 0; i < FF_ARRAY_ELEMS(setup->dpb); ++i) {
        if (ctx->dpb_mask & (1 << i))
            dpb_add(h, &setup->dpb[i], ctx->dpb[i]);
    }

    memcpy(setup->WeightScale,       pps->scaling_matrix4,    sizeof(setup->WeightScale));
    memcpy(setup->WeightScale8x8[0], pps->scaling_matrix8[0], sizeof(setup->WeightScale8x8[0]));
    memcpy(setup->WeightScale8x8[1], pps->scaling_matrix8[3], sizeof(setup->WeightScale8x8[1]));

    /* Find the temporally closest frame to be used as a scratch ref, or use the current one */
    diff = INT_MAX;
    ctx->scratch_ref = h->cur_pic_ptr;
    for (i = 0; i < FF_ARRAY_ELEMS(ctx->dpb); ++i) {
        if (!(ctx->dpb_mask & (1 << i)))
            continue;

        if (FFABS(h->cur_pic_ptr->frame_num - ctx->dpb[i]->frame_num) < diff)
            ctx->scratch_ref = ctx->dpb[i];
    }
}

static int envideo_h264_prepare_cmdbuf(EnvideoCmdbuf *cmdbuf, H264Context *h,
                                       AVFrame *cur_frame, EnvideoH264DecodeContext *ctx)
{
    EnvideoH264DecodeContextShared *ss = ctx->shared;
    FFEnvideoDecodeContextShared   *sc = ctx->core.shared;
    FrameDecodeData               *fdd = (FrameDecodeData *)cur_frame->private_ref->data;
    FFEnvideoDecodeFrame           *tf = fdd->hwaccel_priv;
    AVEnvideoJob                  *job = (AVEnvideoJob *)tf->operation.job_ref->data;
    EnvideoMap              *input_map = job->input_map;

    H264Picture *refs[16+1];
    EnvideoH264FrameData *fr_priv;
    int err, i;

    err = envideo_cmdbuf_begin(cmdbuf, EnvideoEngine_Nvdec);
    if (err < 0)
        return err;

    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_APPLICATION_ID,
                          DRF_DEF(C9B0, _SET_APPLICATION_ID, _ID, _H264));
    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_CONTROL_PARAMS,
                          DRF_DEF(C9B0, _SET_CONTROL_PARAMS, _CODEC_TYPE,     _H264) |
                          DRF_NUM(C9B0, _SET_CONTROL_PARAMS, _ERR_CONCEAL_ON, 1)     |
                          DRF_NUM(C9B0, _SET_CONTROL_PARAMS, _GPTIMER_ON,     1));
    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_PICTURE_INDEX,
                          DRF_NUM(C9B0, _SET_PICTURE_INDEX, _INDEX, ctx->core.frame_idx));

    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_DRV_PIC_SETUP_OFFSET,     input_map, sc->pic_setup_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_IN_BUF_BASE_OFFSET,       input_map, sc->bitstream_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_SLICE_OFFSETS_BUF_OFFSET, input_map, sc->slice_offsets_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_NVDEC_STATUS_OFFSET,      input_map, sc->status_off);

    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_COLOC_DATA_OFFSET,      ss->common_map, ss->coloc_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_H264_SET_MBHIST_BUF_OFFSET, ss->common_map, ss->mbhist_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_HISTORY_OFFSET,         ss->common_map, ss->history_off);

    /* Build list of references sorted by picture idx */
    for (i = 0; i < FF_ARRAY_ELEMS(refs); ++i)
        refs[i] = ctx->scratch_ref;

    fr_priv = h->cur_pic_ptr->hwaccel_picture_private;
    refs[fr_priv->pic_idx] = h->cur_pic_ptr;

    for (i = 0; i < FF_ARRAY_ELEMS(ctx->dpb); ++i) {
        if (!(ctx->dpb_mask & (1 << i)))
            continue;
        fr_priv = ctx->dpb[i]->hwaccel_picture_private;
        refs[fr_priv->pic_idx] = ctx->dpb[i];
    }

#define PUSH_FRAME(fr, offset) ({                                                               \
    FF_ENVIDEO_PUSH_RELOC_TILED(cmdbuf, NVC9B0_SET_PICTURE_LUMA_OFFSET0   + offset * 4,         \
                                av_envideo_frame_get_fbuf_map(fr), 0);                          \
    FF_ENVIDEO_PUSH_RELOC_TILED(cmdbuf, NVC9B0_SET_PICTURE_CHROMA_OFFSET0 + offset * 4,         \
                                av_envideo_frame_get_fbuf_map(fr), fr->data[1] - fr->data[0]);  \
})

    for (i = 0; i < FF_ARRAY_ELEMS(refs); ++i)
        PUSH_FRAME(refs[i]->f, i);

    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_EXECUTE,
                          DRF_DEF(C9B0, _EXECUTE, _AWAKEN, _ENABLE));

    err = envideo_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    return 0;
}

static int envideo_h264_start_frame(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    H264Context                *h = avctx->priv_data;
    AVFrame                *frame = h->cur_pic_ptr->f;
    FrameDecodeData          *fdd = (FrameDecodeData *)frame->private_ref->data;
    EnvideoH264DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    FFEnvideoDecodeFrame *tf;
    AVEnvideoJob *job;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Starting h264-envideo frame with pixel format %s\n",
           av_get_pix_fmt_name(avctx->sw_pix_fmt));

    err = ff_envideo_start_frame(avctx, frame, &ctx->core);
    if (err < 0)
        return err;

    tf  = fdd->hwaccel_priv;
    job = (AVEnvideoJob *)tf->operation.job_ref->data;
    mem = envideo_map_get_cpu_addr(job->input_map);

    memset(ctx->dpb, 0, sizeof(ctx->dpb));
    ctx->dpb_mask = ctx->pic_idx_mask = 0;

    envideo_h264_prepare_frame_setup((nvdec_h264_pic_s *)(mem + ctx->core.shared->pic_setup_off), h, ctx);

    return 0;
}

static int envideo_h264_end_frame(AVCodecContext *avctx) {
    H264Context                *h = avctx->priv_data;
    EnvideoH264DecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    AVFrame                *frame = h->cur_pic_ptr->f;
    FrameDecodeData          *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame      *tf = fdd->hwaccel_priv;
    AVEnvideoJob             *job = (AVEnvideoJob *)tf->operation.job_ref->data;

    nvdec_h264_pic_s *setup;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Ending h264-envideo frame with %u slices -> %u bytes\n",
           tf->operation.num_slices, tf->operation.bitstream_len);

    if (!tf || !tf->operation.num_slices)
        return 0;

    mem = envideo_map_get_cpu_addr(job->input_map);

    setup = (nvdec_h264_pic_s *)(mem + ctx->core.shared->pic_setup_off);
    setup->stream_len  = tf->operation.bitstream_len + sizeof(bitstream_end_sequence);
    setup->slice_count = tf->operation.num_slices;

    err = envideo_h264_prepare_cmdbuf(job->cmdbuf, h, frame, ctx);
    if (err < 0)
        return err;

    return ff_envideo_end_frame(avctx, frame, &ctx->core, bitstream_end_sequence,
                                sizeof(bitstream_end_sequence));
}

static int envideo_h264_decode_slice(AVCodecContext *avctx, const uint8_t *buf,
                                     uint32_t buf_size)
{
    H264Context *h = avctx->priv_data;
    AVFrame *frame = h->cur_pic_ptr->f;

    return ff_envideo_decode_slice(avctx, frame, buf, buf_size, true);
}

static int envideo_h264_update_thread_context(AVCodecContext *dst, const AVCodecContext *src) {
    EnvideoH264DecodeContext *src_ctx = src->internal->hwaccel_priv_data;
    EnvideoH264DecodeContext *dst_ctx = dst->internal->hwaccel_priv_data;

    av_refstruct_replace(&dst_ctx->shared, src_ctx->shared);
    memcpy(dst_ctx->dpb, src_ctx->dpb, sizeof(dst_ctx->dpb));
    dst_ctx->scratch_ref  = src_ctx->scratch_ref;
    dst_ctx->dpb_mask     = src_ctx->dpb_mask;
    dst_ctx->pic_idx_mask = src_ctx->pic_idx_mask;

    return ff_envideo_update_thread_context(&dst_ctx->core, &src_ctx->core);
}

#if CONFIG_H264_ENVIDEO_HWACCEL
const FFHWAccel ff_h264_envideo_hwaccel = {
    .p.name                = "h264_envideo",
    .p.type                = AVMEDIA_TYPE_VIDEO,
    .p.id                  = AV_CODEC_ID_H264,
    .p.pix_fmt             = AV_PIX_FMT_ENVIDEO,
    .start_frame           = &envideo_h264_start_frame,
    .end_frame             = &envideo_h264_end_frame,
    .decode_slice          = &envideo_h264_decode_slice,
    .init                  = &envideo_h264_decode_init,
    .uninit                = &envideo_h264_decode_uninit,
    .frame_params          = &ff_envideo_frame_params,
    .update_thread_context = &envideo_h264_update_thread_context,
    .frame_priv_data_size  = sizeof(EnvideoH264FrameData),
    .priv_data_size        = sizeof(EnvideoH264DecodeContext),
    .caps_internal         = HWACCEL_CAP_ASYNC_SAFE | HWACCEL_CAP_THREAD_SAFE,
};
#endif
