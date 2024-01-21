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

#include <stdbool.h>
#include <string.h>

#include "config_components.h"

#include "avcodec.h"
#include "hwaccel_internal.h"
#include "internal.h"
#include "hwconfig.h"
#include "h264dec.h"
#include "decode.h"
#include "nvtegra_decode.h"

#include "libavutil/pixdesc.h"
#include "libavutil/nvtegra_host1x.h"

typedef struct NVTegraH264DecodeContext {
    NVTegraDecodeContext core;

    AVNVTegraMap common_map;
    uint32_t coloc_off, mbhist_off, history_off;
    uint32_t mbhist_size, history_size;

    struct NVTegraH264RefFrame {
        AVNVTegraMap *map;
        uint32_t chroma_off;
        int16_t frame_num;
        int16_t pic_id;
    } refs[16+1];

    uint8_t ordered_dpb_map[16+1],
        pic_id_map[16+1], scratch_ref, cur_frame;

    uint64_t refs_mask, ordered_dpb_mask, pic_id_mask;
} NVTegraH264DecodeContext;

/* Size (width, height) of a macroblock */
#define MB_SIZE 16

static const uint8_t bitstream_end_sequence[16] = {
    0x00, 0x00, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00,
};

static int nvtegra_h264_decode_uninit(AVCodecContext *avctx) {
    NVTegraH264DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    int err;

    av_log(avctx, AV_LOG_DEBUG, "Deinitializing NVTEGRA H264 decoder\n");

    err = ff_nvtegra_map_destroy(&ctx->common_map);
    if (err < 0)
        return err;

    err = ff_nvtegra_decode_uninit(avctx, &ctx->core);
    if (err < 0)
        return err;

    return 0;
}

static int nvtegra_h264_decode_init(AVCodecContext *avctx) {
    H264Context                *h = avctx->priv_data;
    const SPS                *sps = h->ps.sps;
    NVTegraH264DecodeContext *ctx = avctx->internal->hwaccel_priv_data;
#ifdef __SWITCH__
    AVHWDeviceContext      *hw_device_ctx;
    AVNVTegraDeviceContext *device_hwctx;
#endif

    uint32_t aligned_width, aligned_height,
             width_in_mbs, height_in_mbs, num_slices,
             coloc_size, mbhist_size, history_size, common_map_size;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Initializing NVTEGRA H264 decoder\n");

    aligned_width  = FFALIGN(avctx->coded_width,  MB_SIZE);
    aligned_height = FFALIGN(avctx->coded_height, MB_SIZE);
    width_in_mbs   = aligned_width  / MB_SIZE;
    height_in_mbs  = aligned_height / MB_SIZE;

    num_slices = width_in_mbs * height_in_mbs;

    /* Ignored: histogram map, size 0x400 */
    ctx->core.pic_setup_off     = 0;
    ctx->core.status_off        = FFALIGN(ctx->core.pic_setup_off     + sizeof(nvdec_h264_pic_s),
                                          FF_NVTEGRA_MAP_ALIGN);
    ctx->core.cmdbuf_off        = FFALIGN(ctx->core.status_off        + sizeof(nvdec_status_s),
                                          FF_NVTEGRA_MAP_ALIGN);
    ctx->core.slice_offsets_off = FFALIGN(ctx->core.cmdbuf_off        + 3*FF_NVTEGRA_MAP_ALIGN,
                                          FF_NVTEGRA_MAP_ALIGN);
    ctx->core.bitstream_off     = FFALIGN(ctx->core.slice_offsets_off + num_slices * sizeof(uint32_t),
                                          FF_NVTEGRA_MAP_ALIGN);
    ctx->core.input_map_size    = FFALIGN(ctx->core.bitstream_off     + ff_nvtegra_decode_pick_bitstream_buffer_size(avctx),
                                          0x1000);

    ctx->core.max_cmdbuf_size    =  ctx->core.slice_offsets_off - ctx->core.cmdbuf_off;
    ctx->core.max_num_slices     = (ctx->core.bitstream_off     - ctx->core.slice_offsets_off) / sizeof(uint32_t);
    ctx->core.max_bitstream_size =  ctx->core.input_map_size    - ctx->core.bitstream_off;

    err = ff_nvtegra_decode_init(avctx, &ctx->core);
    if (err < 0)
        goto fail;

    coloc_size   = FFALIGN(FFALIGN(height_in_mbs, 2) * (width_in_mbs * 64) - 63, 0x100);
    coloc_size  *= sps->ref_frame_count + 1; /* Max number of references frames, plus current frame */
    mbhist_size  = FFALIGN(width_in_mbs * 104, 0x100);
    history_size = FFALIGN(width_in_mbs * 0x200 + 0x1100, 0x200);

    ctx->coloc_off   = 0;
    ctx->mbhist_off  = FFALIGN(ctx->coloc_off   + coloc_size,   FF_NVTEGRA_MAP_ALIGN);
    ctx->history_off = FFALIGN(ctx->mbhist_off  + mbhist_size,  FF_NVTEGRA_MAP_ALIGN);
    common_map_size  = FFALIGN(ctx->history_off + history_size, 0x1000);

#ifdef __SWITCH__
    hw_device_ctx = (AVHWDeviceContext *)ctx->core.hw_device_ref->data;
    device_hwctx  = hw_device_ctx->hwctx;

    ctx->common_map.owner = device_hwctx->nvdec_channel.channel.fd;
#endif

    err = ff_nvtegra_map_create(&ctx->common_map, common_map_size, 0x100,
                                NVMAP_HEAP_IOVMM, NVMAP_HANDLE_WRITE_COMBINE);
    if (err < 0)
        goto fail;

    ctx->mbhist_size  = mbhist_size;
    ctx->history_size = history_size;

    memset(ctx->ordered_dpb_map, -1, sizeof(ctx->ordered_dpb_map));
    memset(ctx->pic_id_map,      -1, sizeof(ctx->pic_id_map));

    return 0;

fail:
    nvtegra_h264_decode_uninit(avctx);
    return err;
}

static int field_poc(int flags, int poc[2], bool top) {
    if (top)
        return ((flags & PICT_TOP_FIELD)    && poc[0] != INT_MAX) ? poc[0] : 0;
    else
        return ((flags & PICT_BOTTOM_FIELD) && poc[1] != INT_MAX) ? poc[1] : 0;
}

static void dpb_add(H264Context *h, nvdec_dpb_entry_s *dst,
                    H264Picture *src, int pic_id)
{
    int marking;

    marking = src->long_ref ? 2 : 1;
    *dst = (nvdec_dpb_entry_s){
        .index                = pic_id,
        .col_idx              = pic_id,
        .state                = src->reference,
        .is_long_term         = src->long_ref,
        .not_existing         = src->invalid_gap,
        .is_field             = src->field_picture,
        .top_field_marking    = (src->reference & PICT_TOP_FIELD)    ? marking : 0,
        .bottom_field_marking = (src->reference & PICT_BOTTOM_FIELD) ? marking : 0,
        .output_memory_layout = 0, /* NV12 */
        .FieldOrderCnt        = {
            field_poc(src->reference, src->field_poc, true),
            field_poc(src->reference, src->field_poc, false),
        },
        .FrameIdx             = src->long_ref ? src->pic_id : src->frame_num,
    };
}

static inline int find_slot(uint64_t *mask) {
    int slot = __builtin_ctzll(~*mask);
    *mask |= (1 << slot);
    return slot;
}

static void nvtegra_h264_prepare_frame_setup(nvdec_h264_pic_s *setup, H264Context *h,
                                             NVTegraH264DecodeContext *ctx)
{
    const PPS *pps = h->ps.pps;
    const SPS *sps = h->ps.sps;

    int dpb_size, i, j, diff;
    H264Picture *refs [16+1] = {0};
    uint8_t dpb_to_ref[16+1] = {0};

    *setup = (nvdec_h264_pic_s){
        .mbhist_buffer_size                     = ctx->mbhist_size,

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

        .HistBufferSize                         = ctx->history_size / 256,

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
            field_poc(h->picture_structure, h->cur_pic_ptr->field_poc, true),
            field_poc(h->picture_structure, h->cur_pic_ptr->field_poc, false),
        },

        .lossless_ipred8x8_filter_enable        = true,
        .qpprime_y_zero_transform_bypass_flag   = sps->transform_bypass,
    };

    /* Build concatenated ref list for this frame */
    dpb_size = 0;
    for (i = 0; i < h->short_ref_count; ++i)
        refs[dpb_size++] = h->short_ref[i];

    for (i = 0; i < 16; ++i)
        if (h->long_ref[i])
            refs[dpb_size++] = h->long_ref[i];

    /* Remove stale references from our ref list */
    for (i = 0; i < FF_ARRAY_ELEMS(ctx->refs); ++i) {
        if (!ctx->refs[i].map)
            continue;

        for (j = 0; j < dpb_size; ++j) {
            if (ff_nvtegra_frame_get_fbuf_map(refs[j]->f) == ctx->refs[i].map)
                break;
        }

        if (j == dpb_size) {
            ctx->pic_id_mask &= ~(1 << ctx->refs[i].pic_id);
            ctx->pic_id_map[ctx->refs[i].pic_id] = -1;

            ctx->refs_mask &= ~(1 << i);
            ctx->refs[i].map = NULL;
        } else {
            dpb_to_ref[i] = j;
        }
    }

    /* Update the ordered DPB and pic id masks */
    for (i = 0; i < FF_ARRAY_ELEMS(ctx->ordered_dpb_map); ++i) {
        if (!(ctx->ordered_dpb_mask & (1 << i)))
            continue;
        if (!ctx->refs[ctx->ordered_dpb_map[i]].map) {
            ctx->ordered_dpb_mask &= ~(1 << i);
            ctx->ordered_dpb_map[i] = -1;
        }
    }

    /* Add new frames to the ordered DPB */
    for (i = 0; i < FF_ARRAY_ELEMS(ctx->refs); ++i) {
        if (!ctx->refs[i].map)
            continue;

        for (j = 0; j < FF_ARRAY_ELEMS(ctx->ordered_dpb_map); ++j) {
            if (ctx->ordered_dpb_map[j] == i)
                break;
        }

        if (j == FF_ARRAY_ELEMS(ctx->ordered_dpb_map))
            ctx->ordered_dpb_map[find_slot(&ctx->ordered_dpb_mask)] = i;
    }

    /* In the case of interlaced video, the new frame can be the same as the last */
    if (ctx->refs[ctx->cur_frame].map != ff_nvtegra_frame_get_fbuf_map(h->cur_pic_ptr->f)) {
        /* Allocate a pic id for the current frame */
        i = find_slot(&ctx->pic_id_mask);

        /* Insert it in our ref list */
        ctx->cur_frame = find_slot(&ctx->refs_mask);
        ctx->pic_id_map[i] = ctx->cur_frame;
        ctx->refs[ctx->cur_frame] = (struct NVTegraH264RefFrame){
            .map        = ff_nvtegra_frame_get_fbuf_map(h->cur_pic_ptr->f),
            .chroma_off = h->cur_pic_ptr->f->data[1] - h->cur_pic_ptr->f->data[0],
            .frame_num  = h->cur_pic_ptr->frame_num,
            .pic_id     = i,
        };
    }

    setup->CurrPicIdx = setup->CurrColIdx = ctx->refs[ctx->cur_frame].pic_id;

    /* Find the temporally closest frame to be used as a scratch ref, or use the current one */
    diff = INT_MAX;
    ctx->scratch_ref = ctx->cur_frame;
    for (i = 0; i < FF_ARRAY_ELEMS(ctx->ordered_dpb_map); ++i) {
        j = ctx->ordered_dpb_map[i];
        if ((ctx->ordered_dpb_mask & (1 << i)) &&
                FFABS(h->cur_pic_ptr->frame_num - refs[dpb_to_ref[j]]->frame_num) < diff)
            ctx->scratch_ref = j;
    }

    /* Build the NVDEC DPB */
    for (i = 0; i < FF_ARRAY_ELEMS(setup->dpb); ++i) {
        if (ctx->ordered_dpb_mask & (1 << i)) {
            j = ctx->ordered_dpb_map[i];
            dpb_add(h, &setup->dpb[i], refs[dpb_to_ref[j]], ctx->refs[j].pic_id);
        }
    }

    memcpy(setup->WeightScale,       pps->scaling_matrix4,    sizeof(setup->WeightScale));
    memcpy(setup->WeightScale8x8[0], pps->scaling_matrix8[0], sizeof(setup->WeightScale8x8[0]));
    memcpy(setup->WeightScale8x8[1], pps->scaling_matrix8[3], sizeof(setup->WeightScale8x8[1]));
}

static int nvtegra_h264_prepare_cmdbuf(AVNVTegraCmdbuf *cmdbuf, H264Context *h,
                                       AVFrame *cur_frame, NVTegraH264DecodeContext *ctx)
{
    FrameDecodeData    *fdd = (FrameDecodeData *)cur_frame->private_ref->data;
    NVTegraFrame        *tf = fdd->hwaccel_priv;
    AVNVTegraMap *input_map = (AVNVTegraMap *)tf->input_map_ref->data;

    int err, i;

    err = ff_nvtegra_cmdbuf_begin(cmdbuf, HOST1X_CLASS_NVDEC);
    if (err < 0)
        return err;

    FF_NVTEGRA_PUSH_VALUE(cmdbuf, NVC5B0_SET_APPLICATION_ID,
                          FF_NVTEGRA_ENUM(NVC5B0_SET_APPLICATION_ID, ID, H264));
    FF_NVTEGRA_PUSH_VALUE(cmdbuf, NVC5B0_SET_CONTROL_PARAMS,
                          FF_NVTEGRA_ENUM (NVC5B0_SET_CONTROL_PARAMS, CODEC_TYPE,     H264) |
                          FF_NVTEGRA_VALUE(NVC5B0_SET_CONTROL_PARAMS, ERR_CONCEAL_ON, 1)    |
                          FF_NVTEGRA_VALUE(NVC5B0_SET_CONTROL_PARAMS, GPTIMER_ON,     1));
    FF_NVTEGRA_PUSH_VALUE(cmdbuf, NVC5B0_SET_PICTURE_INDEX,
                          FF_NVTEGRA_VALUE(NVC5B0_SET_PICTURE_INDEX, INDEX, ctx->core.frame_idx));

    FF_NVTEGRA_PUSH_RELOC(cmdbuf, NVC5B0_SET_DRV_PIC_SETUP_OFFSET,
                          input_map,        ctx->core.pic_setup_off,     NVHOST_RELOC_TYPE_DEFAULT);
    FF_NVTEGRA_PUSH_RELOC(cmdbuf, NVC5B0_SET_IN_BUF_BASE_OFFSET,
                          input_map,        ctx->core.bitstream_off,     NVHOST_RELOC_TYPE_DEFAULT);
    FF_NVTEGRA_PUSH_RELOC(cmdbuf, NVC5B0_SET_SLICE_OFFSETS_BUF_OFFSET,
                          input_map,        ctx->core.slice_offsets_off, NVHOST_RELOC_TYPE_DEFAULT);
    FF_NVTEGRA_PUSH_RELOC(cmdbuf, NVC5B0_SET_NVDEC_STATUS_OFFSET,
                          input_map,        ctx->core.status_off,        NVHOST_RELOC_TYPE_DEFAULT);

    FF_NVTEGRA_PUSH_RELOC(cmdbuf, NVC5B0_SET_COLOC_DATA_OFFSET,
                          &ctx->common_map, ctx->coloc_off,              NVHOST_RELOC_TYPE_DEFAULT);
    FF_NVTEGRA_PUSH_RELOC(cmdbuf, NVC5B0_H264_SET_MBHIST_BUF_OFFSET,
                          &ctx->common_map, ctx->mbhist_off,             NVHOST_RELOC_TYPE_DEFAULT);
    FF_NVTEGRA_PUSH_RELOC(cmdbuf, NVC5B0_SET_HISTORY_OFFSET,
                          &ctx->common_map, ctx->history_off,            NVHOST_RELOC_TYPE_DEFAULT);

#define PUSH_FRAME(ref, offset) ({                                                \
    FF_NVTEGRA_PUSH_RELOC(cmdbuf, NVC5B0_SET_PICTURE_LUMA_OFFSET0   + offset * 4, \
                          ref.map, 0, NVHOST_RELOC_TYPE_DEFAULT);                 \
    FF_NVTEGRA_PUSH_RELOC(cmdbuf, NVC5B0_SET_PICTURE_CHROMA_OFFSET0 + offset * 4, \
                          ref.map, ref.chroma_off, NVHOST_RELOC_TYPE_DEFAULT);    \
})

    for (i = 0; i < 16 + 1; ++i) {
        if (i == ctx->cur_frame)
            PUSH_FRAME(ctx->refs[i], i);
        else if (ctx->pic_id_mask & (1 << i))
            PUSH_FRAME(ctx->refs[ctx->pic_id_map[i]], i);
        else
            PUSH_FRAME(ctx->refs[ctx->scratch_ref], i);
    }

    FF_NVTEGRA_PUSH_VALUE(cmdbuf, NVC5B0_EXECUTE,
                          FF_NVTEGRA_ENUM(NVC5B0_EXECUTE, AWAKEN, ENABLE));

    err = ff_nvtegra_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    return 0;
}

static int nvtegra_h264_start_frame(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    H264Context                *h = avctx->priv_data;
    AVFrame                *frame = h->cur_pic_ptr->f;
    FrameDecodeData          *fdd = (FrameDecodeData *)frame->private_ref->data;
    NVTegraH264DecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    NVTegraFrame *tf;
    AVNVTegraMap *input_map;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Starting H264-NVTEGRA frame with pixel format %s\n",
           av_get_pix_fmt_name(avctx->sw_pix_fmt));

    err = ff_nvtegra_start_frame(avctx, frame, &ctx->core);
    if (err < 0)
        return err;

    tf = fdd->hwaccel_priv;
    input_map = (AVNVTegraMap *)tf->input_map_ref->data;
    mem = ff_nvtegra_map_get_addr(input_map);

    nvtegra_h264_prepare_frame_setup((nvdec_h264_pic_s *)(mem + ctx->core.pic_setup_off), h, ctx);

    return 0;
}

static int nvtegra_h264_end_frame(AVCodecContext *avctx) {
    H264Context                *h = avctx->priv_data;
    NVTegraH264DecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    AVFrame                *frame = h->cur_pic_ptr->f;
    FrameDecodeData          *fdd = (FrameDecodeData *)frame->private_ref->data;
    NVTegraFrame              *tf = fdd->hwaccel_priv;

    nvdec_h264_pic_s *setup;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Ending H264-NVTEGRA frame with %u slices -> %u bytes\n",
           ctx->core.num_slices, ctx->core.bitstream_len);

    if (!tf || !ctx->core.num_slices)
        return 0;

    mem = ff_nvtegra_map_get_addr((AVNVTegraMap *)tf->input_map_ref->data);

    setup = (nvdec_h264_pic_s *)(mem + ctx->core.pic_setup_off);
    setup->stream_len  = ctx->core.bitstream_len + sizeof(bitstream_end_sequence);
    setup->slice_count = ctx->core.num_slices;

    err = nvtegra_h264_prepare_cmdbuf(&ctx->core.cmdbuf, h, frame, ctx);
    if (err < 0)
        return err;

    return ff_nvtegra_end_frame(avctx, frame, &ctx->core, bitstream_end_sequence,
                                sizeof(bitstream_end_sequence));
}

static int nvtegra_h264_decode_slice(AVCodecContext *avctx, const uint8_t *buf,
                                     uint32_t buf_size)
{
    H264Context *h = avctx->priv_data;
    AVFrame *frame = h->cur_pic_ptr->f;

    return ff_nvtegra_decode_slice(avctx, frame, buf, buf_size, true);
}

#if CONFIG_H264_NVTEGRA_HWACCEL
const FFHWAccel ff_h264_nvtegra_hwaccel = {
    .p.name         = "h264_nvtegra",
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_H264,
    .p.pix_fmt      = AV_PIX_FMT_NVTEGRA,
    .start_frame    = &nvtegra_h264_start_frame,
    .end_frame      = &nvtegra_h264_end_frame,
    .decode_slice   = &nvtegra_h264_decode_slice,
    .init           = &nvtegra_h264_decode_init,
    .uninit         = &nvtegra_h264_decode_uninit,
    .frame_params   = &ff_nvtegra_frame_params,
    .priv_data_size = sizeof(NVTegraH264DecodeContext),
    .caps_internal  = HWACCEL_CAP_ASYNC_SAFE,
};
#endif
