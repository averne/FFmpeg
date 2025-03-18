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
#include "hevc/hevcdec.h"
#include "hevc/data.h"
#include "decode.h"
#include "envideo_decode.h"

#include "libavutil/pixdesc.h"

typedef struct EnvideoHEVCFrameData {
    bool initialized;
    uint8_t dpb_idx;
} EnvideoHEVCFrameData;

typedef struct EnvideoHEVCDecodeContextShared {
    EnvideoMap *common_map;
    uint32_t tile_sizes_off, scaling_list_off,
             coloc_off, filter_off, intra_top_off;
    uint32_t col_mv_size, sao_offset, bsd_offset,
             flt_above_offset, sao_above_offset, slice_edge_offset;
} EnvideoHEVCDecodeContextShared;

typedef struct EnvideoHEVCDecodeContext {
    FFEnvideoDecodeContext core;
    EnvideoHEVCDecodeContextShared *shared;

    uint8_t pattern_id;

    HEVCFrame *refs[16], *scratch_ref;
    uint32_t refs_mask;
} EnvideoHEVCDecodeContext;

/* Maximum size (width, height) of a coding tree unit */
#define CTU_SIZE 64

static int envideo_hevc_decode_uninit(AVCodecContext *avctx) {
    EnvideoHEVCDecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    int err;

    av_log(avctx, AV_LOG_DEBUG, "Deinitializing hevc-envideo decoder\n");

    av_refstruct_unref(&ctx->shared);

    err = ff_envideo_decode_uninit(avctx, &ctx->core);
    if (err < 0)
        return err;

    return 0;
}

static void envideo_hevc_shared_free(AVRefStructOpaque opaque, void *obj) {
    EnvideoHEVCDecodeContextShared *shared = obj;

    envideo_map_destroy(shared->common_map);
}

static int envideo_hevc_decode_init(AVCodecContext *avctx) {
    EnvideoHEVCDecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    AVHWDeviceContext      *hw_device_ctx;
    AVEnvideoDeviceContext *device_hwctx;
    EnvideoHEVCDecodeContextShared *ss;
    FFEnvideoDecodeContextShared *sc;
    uint32_t aligned_width, aligned_height, a, b,
             col_mv_size, something_size, sao_size, bsd_size,
             flt_above_size, sao_above_size, slice_edge_size,
             coloc_size, filter_size, common_map_size;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Initializing hevc-envideo decoder\n");

    ctx->shared = av_refstruct_alloc_ext(sizeof(*ctx->shared), 0, NULL, envideo_hevc_shared_free);
    if (!ctx->shared) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    err = ff_envideo_alloc_shared(&ctx->core);
    if (err < 0)
        goto fail;

    ss = ctx->shared;
    sc = ctx->core.shared;

    sc->pic_setup_off        = 0;
    sc->status_off           = FFALIGN(sc->pic_setup_off    + sizeof(nvdec_hevc_pic_s),
                                       ENVIDEO_MAP_ALIGN);
    sc->cmdbuf_off           = FFALIGN(sc->status_off       + sizeof(nvdec_status_s),
                                       ENVIDEO_MAP_ALIGN);
    ss->tile_sizes_off       = FFALIGN(sc->cmdbuf_off       + 3*ENVIDEO_MAP_ALIGN,
                                       ENVIDEO_MAP_ALIGN);
    ss->scaling_list_off     = FFALIGN(ss->tile_sizes_off   + 0x900,
                                       ENVIDEO_MAP_ALIGN);
    sc->bitstream_off        = FFALIGN(ss->scaling_list_off + 0x400,
                                       ENVIDEO_MAP_ALIGN);
    ctx->core.input_map_size = FFALIGN(sc->bitstream_off    + ff_envideo_decode_pick_bitstream_buffer_size(avctx),
                                       0x1000);

    sc->max_cmdbuf_size          = ss->tile_sizes_off       - sc->cmdbuf_off;
    ctx->core.max_bitstream_size = ctx->core.input_map_size - sc->bitstream_off;

    err = ff_envideo_decode_init(avctx, &ctx->core);
    if (err < 0)
        goto fail;

    hw_device_ctx = (AVHWDeviceContext *)sc->hw_device_ref->data;
    device_hwctx  = hw_device_ctx->hwctx;

    aligned_width  = FFALIGN(avctx->coded_width,  CTU_SIZE);
    aligned_height = FFALIGN(avctx->coded_height, CTU_SIZE);

    if (aligned_width * aligned_height > 0x220000)
        a = aligned_width >> 5, b = aligned_height >> 5;
    else
        a = aligned_width >> 4, b = aligned_height >> 4;

    col_mv_size     = aligned_width * aligned_height / 16;
    something_size  = FFALIGN(aligned_height * 0x260,  ENVIDEO_MAP_ALIGN);
    sao_size        = FFALIGN(aligned_height * 0x1300, ENVIDEO_MAP_ALIGN);
    bsd_size        = FFALIGN(aligned_height * 0x4c,   ENVIDEO_MAP_ALIGN);
    flt_above_size  = FFALIGN((b + a * b) * 0xb00,     ENVIDEO_MAP_ALIGN);
    sao_above_size  = FFALIGN(a * b * 0x300,           ENVIDEO_MAP_ALIGN);
    slice_edge_size = ENVIDEO_MAP_ALIGN;

    ss->col_mv_size       = col_mv_size                           / 256;
    ss->sao_offset        = something_size                        / 256;
    ss->bsd_offset        = ss->sao_offset       + sao_size       / 256;
    ss->flt_above_offset  = ss->bsd_offset       + bsd_size       / 256;
    ss->sao_above_offset  = ss->flt_above_offset + flt_above_size / 256;
    ss->slice_edge_offset = ss->sao_above_offset + sao_above_size / 256;

    coloc_size  = col_mv_size * 17;
    filter_size = something_size + sao_size       + bsd_size +
                  flt_above_size + sao_above_size + slice_edge_size;

    ss->coloc_off     = 0;
    ss->filter_off    = FFALIGN(ss->coloc_off     + coloc_size,  ENVIDEO_MAP_ALIGN);
    ss->intra_top_off = FFALIGN(ss->filter_off    + filter_size, ENVIDEO_MAP_ALIGN);
    common_map_size   = FFALIGN(ss->intra_top_off + 0x10000,     0x1000);

    err = envideo_map_create(device_hwctx->device, &ss->common_map, common_map_size, ENVIDEO_MAP_ALIGN,
                             EnvideoMap_CpuWriteCombine | EnvideoMap_GpuCacheable | EnvideoMap_UsageEngine);
    if (err < 0)
        goto fail;

    err = envideo_map_pin(ss->common_map, sc->channel);
    if (err < 0)
        goto fail;

    memset(envideo_map_get_cpu_addr(ss->common_map), 0, envideo_map_get_size(ss->common_map));

    return 0;

fail:
    envideo_hevc_decode_uninit(avctx);
    return err;
}

static uint16_t envideo_hevc_calc_skip_len(HEVCContext *s) {
    SliceHeader    *sh = &s->sh;
    const HEVCPPS *pps = s->pps;
    const HEVCSPS *sps = pps->sps;
    const HEVCVPS *vps = sps->vps;

    /**
     * This value is derived from syntax elements in the slice segment header,
     * starting from pic_output_flag (included) to slice_temporal_mvp_enabled_flag (excluded).
     * Here we reproduce logic from the parsing routine in hevcdec.c,
     * and count the number of associated bits.
     * See also spec section 7.3.6.1 "General slice segment header syntax"
     */

    uint16_t len = 0;

    if (pps->output_flag_present_flag)
        len += 1;

    if (sps->separate_colour_plane)
        len += 2;

    if (!IS_IDR(s) ||
        (s->nuh_layer_id > 0 &&
            !(vps->poc_lsb_not_present & (1 << vps->layer_idx[s->nuh_layer_id]))))
        len += sps->log2_max_poc_lsb;

    if (!IS_IDR(s))
        len += 1 + sh->short_term_ref_pic_set_size + sh->long_term_ref_pic_set_size;

    return len;
}

static void envideo_hevc_set_scaling_list(nvdec_hevc_scaling_list_s *list, HEVCContext *s) {
    const ScalingList *sl = s->pps->scaling_list_data_present_flag ?
                            &s->pps->scaling_list : &s->pps->sps->scaling_list;

    int i, j, k;

    for (i = 0; i < FF_ARRAY_ELEMS(list->ScalingListDCCoeff16x16); ++i)
        list->ScalingListDCCoeff16x16[i] = sl->sl_dc[0][i];
    for (i = 0; i < FF_ARRAY_ELEMS(list->ScalingListDCCoeff32x32); ++i)
        list->ScalingListDCCoeff32x32[i] = sl->sl_dc[1][i * 3];

#define COPY_LIST(dst, src, n) ({                    \
    for (j = 0; j < (n); ++j)                        \
        for (k = 0; k < (n); ++k)                    \
            (dst)[k * (n) + j] = (src)[j * (n) + k]; \
})

    for (i = 0; i < 6; ++i) {
        COPY_LIST(list->ScalingList4x4  [i], sl->sl[0][i], 4);
        COPY_LIST(list->ScalingList8x8  [i], sl->sl[1][i], 8);
        COPY_LIST(list->ScalingList16x16[i], sl->sl[2][i], 8);
    }

    COPY_LIST(list->ScalingList32x32[0], sl->sl[3][0], 8);
    COPY_LIST(list->ScalingList32x32[1], sl->sl[3][3], 8);
}

static void envideo_hevc_set_tile_sizes(uint16_t *sizes, HEVCContext *s) {
    const HEVCPPS *pps = s->pps;
    const HEVCSPS *sps = pps->sps;

    int i, j, sum;

    uint16_t *tile_thing = sizes + 0x380;
    if (pps->uniform_spacing_flag) {
        for (i = 0; i < pps->num_tile_columns; ++i)
            *tile_thing++ = (i + 1) * sps->ctb_width  / pps->num_tile_columns <<
                (sps->log2_diff_max_min_coding_block_size + sps->log2_min_cb_size - 4);
        for (i = 0; i < pps->num_tile_rows; ++i)
            *tile_thing++ = (i + 1) * sps->ctb_height / pps->num_tile_rows    <<
                (sps->log2_diff_max_min_coding_block_size + sps->log2_min_cb_size - 4);
    } else {
        sum = 0;
        for (i = 0; i < pps->num_tile_columns; ++i)
            *tile_thing++ = (sum += pps->column_width[i]) <<
                (sps->log2_diff_max_min_coding_block_size + sps->log2_min_cb_size - 4);
        sum = 0;
        for (i = 0; i < pps->num_tile_rows; ++i)
            *tile_thing++ = (sum += pps->row_height[i])   <<
                (sps->log2_diff_max_min_coding_block_size + sps->log2_min_cb_size - 4);
    }

    for (i = 0; i < pps->num_tile_rows; ++i) {
        for (j = 0; j < pps->num_tile_columns; ++j) {
            sizes[0] = pps->column_width[j];
            sizes[1] = pps->row_height  [i];
            sizes += 2;
        }
    }
}

static enum RPSType find_ref_rps_type(HEVCContext *s, HEVCFrame *f) {
    int i;

#define CHECK_SET(set) ({                       \
    for (i = 0; i < s->rps[set].nb_refs; ++i) { \
        if (s->rps[set].ref[i] == f)            \
            return set;                         \
    }                                           \
})

    CHECK_SET(ST_CURR_BEF);
    CHECK_SET(ST_CURR_AFT);
    CHECK_SET(ST_FOLL);
    CHECK_SET(LT_CURR);
    CHECK_SET(LT_FOLL);

    return -1;
}

static inline int find_slot(uint32_t *mask) {
    int slot = ff_ctz(~*mask);
    *mask |= (1 << slot);
    return slot;
}

static void envideo_hevc_prepare_frame_setup(nvdec_hevc_pic_s *setup, AVCodecContext *avctx,
                                             AVFrame *frame, EnvideoHEVCDecodeContext *ctx)
{
    FrameDecodeData          *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame      *tf = fdd->hwaccel_priv;
    AVEnvideoJob             *job = (AVEnvideoJob *)tf->operation.job_ref->data;
    EnvideoMap         *input_map = job->input_map;
    AVHWFramesContext *frames_ctx = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
    HEVCContext                *s = avctx->priv_data;
    HEVCLayerContext           *l = &s->layers[s->cur_layer];
    SliceHeader               *sh = &s->sh;
    const HEVCPPS            *pps = s->pps;
    const HEVCSPS            *sps = pps->sps;

    HEVCFrame *fr;
    EnvideoHEVCFrameData *fr_priv;
    enum RPSType st;
    uint8_t *mem;
    uint16_t *tile_sizes;
    int output_mode, chroma_format, cur_frame, scratch_ref_diff_poc, i, j;
    int8_t rps_stcurrbef[8], rps_stcurraft[8], rps_ltcurr[8];

    mem = envideo_map_get_cpu_addr(input_map);

    /* Match source color depth regardless of colorspace */
    output_mode = (sps->bit_depth == 8) ? 0 : 1; /* 8-bit bt709, 10-bit bt709 */

    switch (frames_ctx->sw_format) {
        default:
        case AV_PIX_FMT_NV12:
        case AV_PIX_FMT_P010LE:
        case AV_PIX_FMT_P012LE:
            chroma_format = 1; /* 4:2:0 */
            break;
        case AV_PIX_FMT_NV16:
        case AV_PIX_FMT_P210LE:
        case AV_PIX_FMT_P212LE:
            chroma_format = 2; /* 4:2:2 */
            break;
        case AV_PIX_FMT_NV24:
        case AV_PIX_FMT_P410LE:
        case AV_PIX_FMT_P412LE:
            chroma_format = 3; /* 4:4:4 */
            break;
    }

    *setup = (nvdec_hevc_pic_s){
        .gptimer_timeout_value                       = 0, /* Default value */

        .tileformat                                  = !ctx->core.shared->is_tegra, /* Tegra/GPU block linear */
        .gob_height                                  = 0,                           /* GOB_2 */

        .sw_start_code_e                             = 1,
        .disp_output_mode                            = output_mode,

        /* Divide by two if we are decoding to a 2bpp surface */
        .framestride                                 = {
            s->cur_frame->f->linesize[0] / ((output_mode == 1) ? 2 : 1),
            s->cur_frame->f->linesize[1] / ((output_mode == 1) ? 2 : 1),
        },

        .colMvBuffersize                             = ctx->shared->col_mv_size,
        .HevcSaoBufferOffset                         = ctx->shared->sao_offset,
        .HevcBsdCtrlOffset                           = ctx->shared->bsd_offset,

        .pic_width_in_luma_samples                   = sps->width,
        .pic_height_in_luma_samples                  = sps->height,

        .chroma_format_idc                           = chroma_format,
        .bit_depth_luma                              = sps->bit_depth,
        .bit_depth_chroma                            = sps->bit_depth_chroma,
        .log2_min_luma_coding_block_size             = sps->log2_min_cb_size,
        .log2_max_luma_coding_block_size             = sps->log2_diff_max_min_coding_block_size + sps->log2_min_cb_size,
        .log2_min_transform_block_size               = sps->log2_min_tb_size,
        .log2_max_transform_block_size               = sps->log2_max_trafo_size,

        .max_transform_hierarchy_depth_inter         = sps->max_transform_hierarchy_depth_inter,
        .max_transform_hierarchy_depth_intra         = sps->max_transform_hierarchy_depth_intra,
        .scalingListEnable                           = sps->scaling_list_enabled,
        .amp_enable_flag                             = sps->amp_enabled,
        .sample_adaptive_offset_enabled_flag         = sps->sao_enabled,
        .pcm_enabled_flag                            = sps->pcm_enabled,
        .pcm_sample_bit_depth_luma                   = sps->pcm_enabled ? sps->pcm.bit_depth            : 0,
        .pcm_sample_bit_depth_chroma                 = sps->pcm_enabled ? sps->pcm.bit_depth_chroma     : 0,
        .log2_min_pcm_luma_coding_block_size         = sps->pcm_enabled ? sps->pcm.log2_min_pcm_cb_size : 0,
        .log2_max_pcm_luma_coding_block_size         = sps->pcm_enabled ? sps->pcm.log2_max_pcm_cb_size : 0,
        .pcm_loop_filter_disabled_flag               = sps->pcm_loop_filter_disabled,
        .sps_temporal_mvp_enabled_flag               = sps->temporal_mvp_enabled,
        .strong_intra_smoothing_enabled_flag         = sps->strong_intra_smoothing_enabled,

        .dependent_slice_segments_enabled_flag       = pps->dependent_slice_segments_enabled_flag,
        .output_flag_present_flag                    = pps->output_flag_present_flag,
        .num_extra_slice_header_bits                 = pps->num_extra_slice_header_bits,
        .sign_data_hiding_enabled_flag               = pps->sign_data_hiding_flag,
        .cabac_init_present_flag                     = pps->cabac_init_present_flag,
        .num_ref_idx_l0_default_active               = pps->num_ref_idx_l0_default_active,
        .num_ref_idx_l1_default_active               = pps->num_ref_idx_l1_default_active,
        .init_qp                                     = pps->pic_init_qp_minus26 + 26 + (sps->bit_depth - 8) * 6,
        .constrained_intra_pred_flag                 = pps->constrained_intra_pred_flag,
        .transform_skip_enabled_flag                 = pps->transform_skip_enabled_flag,
        .cu_qp_delta_enabled_flag                    = pps->cu_qp_delta_enabled_flag,
        .diff_cu_qp_delta_depth                      = pps->diff_cu_qp_delta_depth,

        .pps_cb_qp_offset                            = pps->cb_qp_offset,
        .pps_cr_qp_offset                            = pps->cr_qp_offset,
        .pps_beta_offset                             = pps->beta_offset,
        .pps_tc_offset                               = pps->tc_offset,
        .pps_slice_chroma_qp_offsets_present_flag    = pps->pic_slice_level_chroma_qp_offsets_present_flag,
        .weighted_pred_flag                          = pps->weighted_pred_flag,
        .weighted_bipred_flag                        = pps->weighted_bipred_flag,
        .transquant_bypass_enabled_flag              = pps->transquant_bypass_enable_flag,
        .tiles_enabled_flag                          = pps->tiles_enabled_flag,
        .entropy_coding_sync_enabled_flag            = pps->entropy_coding_sync_enabled_flag,
        .num_tile_columns                            = pps->tiles_enabled_flag ? pps->num_tile_columns : 0,
        .num_tile_rows                               = pps->tiles_enabled_flag ? pps->num_tile_rows    : 0,
        .loop_filter_across_tiles_enabled_flag       = pps->tiles_enabled_flag ? pps->loop_filter_across_tiles_enabled_flag : 0,
        .loop_filter_across_slices_enabled_flag      = pps->seq_loop_filter_across_slices_enabled_flag,
        .deblocking_filter_control_present_flag      = pps->deblocking_filter_control_present_flag,
        .deblocking_filter_override_enabled_flag     = pps->deblocking_filter_override_enabled_flag,
        .pps_deblocking_filter_disabled_flag         = pps->disable_dbf,
        .lists_modification_present_flag             = pps->lists_modification_present_flag,
        .log2_parallel_merge_level                   = pps->log2_parallel_merge_level,
        .slice_segment_header_extension_present_flag = pps->slice_header_extension_present_flag,

        .num_ref_frames                              = ff_hevc_frame_nb_refs(sh, pps, s->cur_layer),

        .IDR_picture_flag                            = IS_IDR(s),
        .RAP_picture_flag                            = IS_IRAP(s),
        .pattern_id                                  = ((output_mode == 0) || (output_mode == 1)) ? 2 : ctx->pattern_id, /* Disable/enable dithering */
        .sw_hdr_skip_length                          = envideo_hevc_calc_skip_len(s),

        /**
         * Ignored in official code
        .separate_colour_plane_flag                  = sps->separate_colour_plane_flag,
        .log2_max_pic_order_cnt_lsb_minus4           = sps->log2_max_poc_lsb - 4,
        .num_short_term_ref_pic_sets                 = sps->nb_st_rps,
        .num_long_term_ref_pics_sps                  = sps->num_long_term_ref_pics_sps,
        .num_delta_pocs_of_rps_idx                   = s->sh.short_term_rps ? s->sh.short_term_rps->rps_idx_num_delta_pocs : 0,
        .long_term_ref_pics_present_flag             = sps->long_term_ref_pics_present_flag,
        .num_bits_short_term_ref_pics_in_slice       = sh->short_term_ref_pic_set_size;
        */

        .v1 = {
            .error_recovery_start_pos                = 1, /* Start of slice segment */

            .hevc_main10_444_ext = {
                .HevcFltAboveOffset                  = ctx->shared->flt_above_offset,
                .HevcSaoAboveOffset                  = ctx->shared->sao_above_offset,

                .transformSkipRotationEnableFlag     = sps->range_extension ? sps->transform_skip_rotation_enabled    : 0,
                .transformSkipContextEnableFlag      = sps->range_extension ? sps->transform_skip_context_enabled     : 0,
                .intraBlockCopyEnableFlag            = 0,
                .implicitRdpcmEnableFlag             = sps->range_extension ? sps->implicit_rdpcm_enabled             : 0,
                .explicitRdpcmEnableFlag             = sps->range_extension ? sps->explicit_rdpcm_enabled             : 0,
                .extendedPrecisionProcessingFlag     = sps->range_extension ? sps->extended_precision_processing      : 0,
                .intraSmoothingDisabledFlag          = sps->range_extension ? sps->intra_smoothing_disabled           : 0,
                .highPrecisionOffsetsEnableFlag      = sps->range_extension ? sps->high_precision_offsets_enabled     : 0,
                .fastRiceAdaptationEnableFlag        = sps->range_extension ? sps->persistent_rice_adaptation_enabled : 0,
                .cabacBypassAlignmentEnableFlag      = sps->range_extension ? sps->cabac_bypass_alignment_enabled     : 0,

                .log2MaxTransformSkipSize            = pps->pps_range_extensions_flag ? pps->log2_max_transform_skip_block_size      : 2,
                .crossComponentPredictionEnableFlag  = pps->pps_range_extensions_flag ? pps->cross_component_prediction_enabled_flag : 0,
                .chromaQpAdjustmentEnableFlag        = pps->pps_range_extensions_flag ? pps->chroma_qp_offset_list_enabled_flag      : 0,
                .diffCuChromaQpAdjustmentDepth       = pps->pps_range_extensions_flag ? pps->diff_cu_chroma_qp_offset_depth          : 0,
                .chromaQpAdjustmentTableSize         = pps->pps_range_extensions_flag ? pps->chroma_qp_offset_list_len_minus1        : 0,
                .log2SaoOffsetScaleLuma              = pps->pps_range_extensions_flag ? pps->log2_sao_offset_scale_luma              : 0,
                .log2SaoOffsetScaleChroma            = pps->pps_range_extensions_flag ? pps->log2_sao_offset_scale_chroma            : 0,
            }
        },

        .v3.HevcSliceEdgeOffset                      = ctx->shared->slice_edge_offset,
    };

    for (i = 0; i < pps->chroma_qp_offset_list_len_minus1 + 1; ++i) {
        setup->v1.hevc_main10_444_ext.cb_qp_adjustment[i] = pps->cb_qp_offset_list[i];
        setup->v1.hevc_main10_444_ext.cr_qp_adjustment[i] = pps->cr_qp_offset_list[i];
    }

    /**
     * Decoded frames need to be allocated an index that represents its position
     * in the data pointers array (pushed to the cmdbuf) and in the metadata
     * sent to the hardware.
     * This index must remain fixed until the frame is dropped from the DPB.
     */

    /* Build ordered reflist from the DPB */
    for (i = 0; i < FF_ARRAY_ELEMS(l->DPB); ++i) {
        fr      = &l->DPB[i];
        fr_priv = fr->hwaccel_picture_private;

        if ((fr->flags & (HEVC_FRAME_FLAG_LONG_REF | HEVC_FRAME_FLAG_SHORT_REF)) &&
            (fr != s->cur_frame) && fr_priv->initialized)
        {
            ctx->refs[fr_priv->dpb_idx] = fr;
            ctx->refs_mask |= 1 << fr_priv->dpb_idx;
        }
    }

    /* Try to find a valid reference, or use the current one */
    ctx->scratch_ref = s->cur_frame, scratch_ref_diff_poc = 0;
    for (i = 0; i < FF_ARRAY_ELEMS(ctx->refs); ++i) {
        fr = ctx->refs[i];
        if (!(ctx->refs_mask & (1 << i)) || (fr == s->cur_frame))
            continue;

        st = find_ref_rps_type(s, fr);
        if ((st != ST_CURR_BEF) && (st != ST_CURR_AFT) && (st != LT_CURR))
            continue;

        ctx->scratch_ref     = fr;
        scratch_ref_diff_poc = av_clip_int8(s->cur_frame->poc - fr->poc);
        break;
    }

    /* Add the current frame to our ref list */
    setup->curr_pic_idx = cur_frame = find_slot(&ctx->refs_mask);
    ctx->refs[cur_frame] = s->cur_frame;

    fr_priv = s->cur_frame->hwaccel_picture_private;
    *fr_priv = (EnvideoHEVCFrameData){
        .dpb_idx     = cur_frame,
        .initialized = true,
    };

    /* Fill the POC metadata */
    for (i = 0; i < FF_ARRAY_ELEMS(setup->RefDiffPicOrderCnts); ++i) {
        if (i == cur_frame)
            continue;

        if (ctx->refs_mask & (1 << i)) {
            fr = ctx->refs[i];
            setup->RefDiffPicOrderCnts[i] = av_clip_int8(s->cur_frame->poc - fr->poc);
            setup->longtermflag |= !!(fr->flags & HEVC_FRAME_FLAG_LONG_REF) << (15 - i);
        } else {
            setup->RefDiffPicOrderCnts[i] = scratch_ref_diff_poc;
        }
    }

#define RPS_TO_DPB_IDX(set, array) ({                       \
    for (i = 0; i < s->rps[set].nb_refs; ++i) {             \
        for (j = 0; j < FF_ARRAY_ELEMS(ctx->refs); ++j) {   \
            if (s->rps[set].ref[i] == ctx->refs[j]) {       \
                array[i] = j;                               \
                break;                                      \
            }                                               \
        }                                                   \
    }                                                       \
})

    RPS_TO_DPB_IDX(ST_CURR_BEF, rps_stcurrbef);
    RPS_TO_DPB_IDX(ST_CURR_AFT, rps_stcurraft);
    RPS_TO_DPB_IDX(LT_CURR,     rps_ltcurr);

#define FILL_REFLIST(list, set, array) ({         \
    int len = FFMIN(s->rps[set].nb_refs, 16 - i); \
    memcpy(&setup->list[i], array, len);          \
    i += len;                                     \
})

    /* Fill the RPS metadata */
    if (s->rps[ST_CURR_BEF].nb_refs + s->rps[ST_CURR_AFT].nb_refs + s->rps[LT_CURR].nb_refs) {
        for (i = 0; i < 16;) {
            FILL_REFLIST(initreflistidxl0, ST_CURR_BEF, rps_stcurrbef);
            FILL_REFLIST(initreflistidxl0, ST_CURR_AFT, rps_stcurraft);
            FILL_REFLIST(initreflistidxl0, LT_CURR,     rps_ltcurr);
        }

        for (i = 0; i < 16;) {
            FILL_REFLIST(initreflistidxl1, ST_CURR_AFT, rps_stcurraft);
            FILL_REFLIST(initreflistidxl1, ST_CURR_BEF, rps_stcurrbef);
            FILL_REFLIST(initreflistidxl1, LT_CURR,     rps_ltcurr);
        }
    }

    ctx->pattern_id ^= 1;

    if (sps->scaling_list_enabled)
        envideo_hevc_set_scaling_list((nvdec_hevc_scaling_list_s *)(mem + ctx->shared->scaling_list_off), s);

    tile_sizes = (uint16_t *)(mem + ctx->shared->tile_sizes_off);
    if (pps->tiles_enabled_flag) {
        envideo_hevc_set_tile_sizes(tile_sizes, s);
    } else {
        tile_sizes[0] = pps->column_width[0];
        tile_sizes[1] = pps->row_height  [0];
    }
}

static int envideo_hevc_prepare_cmdbuf(EnvideoCmdbuf *cmdbuf, HEVCContext *s,
                                       EnvideoHEVCDecodeContext *ctx, AVFrame *cur_frame)
{
    EnvideoHEVCDecodeContextShared *ss = ctx->shared;
    FFEnvideoDecodeContextShared   *sc = ctx->core.shared;
    FrameDecodeData               *fdd = (FrameDecodeData *)cur_frame->private_ref->data;
    FFEnvideoDecodeFrame           *tf = fdd->hwaccel_priv;
    AVEnvideoJob                  *job = (AVEnvideoJob *)tf->operation.job_ref->data;
    EnvideoMap              *input_map = job->input_map;

    int i;
    int err;

    err = envideo_cmdbuf_begin(cmdbuf, EnvideoEngine_Nvdec);
    if (err < 0)
        return err;

    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_APPLICATION_ID,
                          DRF_DEF(C9B0, _SET_APPLICATION_ID, _ID, _HEVC));
    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_CONTROL_PARAMS,
                          DRF_DEF(C9B0, _SET_CONTROL_PARAMS, _CODEC_TYPE,     _HEVC) |
                          DRF_NUM(C9B0, _SET_CONTROL_PARAMS, _ERR_CONCEAL_ON, 1)    |
                          DRF_NUM(C9B0, _SET_CONTROL_PARAMS, _GPTIMER_ON,     1));
    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_SET_PICTURE_INDEX,
                          DRF_NUM(C9B0, _SET_PICTURE_INDEX, _INDEX, ctx->core.frame_idx));

    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_DRV_PIC_SETUP_OFFSET, input_map, sc->pic_setup_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_IN_BUF_BASE_OFFSET,   input_map, sc->bitstream_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_NVDEC_STATUS_OFFSET,  input_map, sc->status_off);

    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_HEVC_SET_SCALING_LIST_OFFSET,  input_map,      ss->scaling_list_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_HEVC_SET_TILE_SIZES_OFFSET,    input_map,      ss->tile_sizes_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_HEVC_SET_FILTER_BUFFER_OFFSET, ss->common_map, ss->filter_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_COLOC_DATA_OFFSET,         ss->common_map, ss->coloc_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVC9B0_SET_INTRA_TOP_BUF_OFFSET,      ss->common_map, ss->intra_top_off);

#define PUSH_FRAME(fr, offset) ({                                                               \
    FF_ENVIDEO_PUSH_RELOC_TILED(cmdbuf, NVC9B0_SET_PICTURE_LUMA_OFFSET0   + offset * 4,         \
                                av_envideo_frame_get_fbuf_map(fr), 0);                          \
    FF_ENVIDEO_PUSH_RELOC_TILED(cmdbuf, NVC9B0_SET_PICTURE_CHROMA_OFFSET0 + offset * 4,         \
                                av_envideo_frame_get_fbuf_map(fr), fr->data[1] - fr->data[0]);  \
})

    for (i = 0; i < FF_ARRAY_ELEMS(ctx->refs); ++i) {
        if (ctx->refs_mask & (1 << i))
            PUSH_FRAME(ctx->refs[i]->f,     i);
        else
            PUSH_FRAME(ctx->scratch_ref->f, i);
    }

    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVC9B0_EXECUTE,
                          DRF_DEF(C9B0, _EXECUTE, _AWAKEN, _ENABLE));

    err = envideo_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    return 0;
}

static int envideo_hevc_start_frame(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    HEVCContext                *s = avctx->priv_data;
    AVFrame                *frame = s->cur_frame->f;
    FrameDecodeData          *fdd = (FrameDecodeData *)frame->private_ref->data;
    EnvideoHEVCDecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    FFEnvideoDecodeFrame *tf;
    AVEnvideoJob *job;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Starting hevc-envideo frame with pixel format %s\n",
           av_get_pix_fmt_name(avctx->sw_pix_fmt));

    err = ff_envideo_start_frame(avctx, frame, &ctx->core);
    if (err < 0)
        return err;

    memset(ctx->refs, 0, sizeof(ctx->refs));
    ctx->refs_mask = 0;

    tf  = fdd->hwaccel_priv;
    job = (AVEnvideoJob *)tf->operation.job_ref->data;
    mem = envideo_map_get_cpu_addr(job->input_map);

    envideo_hevc_prepare_frame_setup((nvdec_hevc_pic_s *)(mem + ctx->core.shared->pic_setup_off),
                                     avctx, frame, ctx);

    return 0;
}

static int envideo_hevc_end_frame(AVCodecContext *avctx) {
    HEVCContext                *s = avctx->priv_data;
    EnvideoHEVCDecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    AVFrame                *frame = s->cur_frame->f;
    FrameDecodeData          *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame      *tf = fdd->hwaccel_priv;
    AVEnvideoJob             *job = (AVEnvideoJob *)tf->operation.job_ref->data;

    nvdec_hevc_pic_s *setup;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Ending hevc-envideo frame with %u slices -> %u bytes\n",
           tf->operation.num_slices, tf->operation.bitstream_len);

    if (!tf || !tf->operation.num_slices)
        return 0;

    mem = envideo_map_get_cpu_addr(job->input_map);

    setup = (nvdec_hevc_pic_s *)(mem + ctx->core.shared->pic_setup_off);
    setup->stream_len = tf->operation.bitstream_len;

    err = envideo_hevc_prepare_cmdbuf(job->cmdbuf, s, ctx, frame);
    if (err < 0)
        return err;

    return ff_envideo_end_frame(avctx, frame, &ctx->core, NULL, 0);
}

static int envideo_hevc_decode_slice(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    HEVCContext                   *s = avctx->priv_data;
    AVFrame                   *frame = s->cur_frame->f;
    FrameDecodeData             *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame         *tf = fdd->hwaccel_priv;
    AVEnvideoJob                *job = (AVEnvideoJob *)tf->operation.job_ref->data;
    EnvideoHEVCDecodeContext    *ctx = avctx->internal->hwaccel_priv_data;
    FFEnvideoDecodeContextShared *sc = ctx->core.shared;

    uint8_t *mem;

    mem = envideo_map_get_cpu_addr(job->input_map);

    /**
     * Official code adds a 4-byte 00000001 startcode,
     * though decoding was observed to work without it
     */
    AV_WB8(mem + sc->bitstream_off + tf->operation.bitstream_len, 0);
    tf->operation.bitstream_len += 1;

    return ff_envideo_decode_slice(avctx, frame, buf, buf_size, AV_RB24(buf) != 1);
}

static int envideo_hevc_update_thread_context(AVCodecContext *dst, const AVCodecContext *src) {
    EnvideoHEVCDecodeContext *src_ctx = src->internal->hwaccel_priv_data;
    EnvideoHEVCDecodeContext *dst_ctx = dst->internal->hwaccel_priv_data;

    av_refstruct_replace(&dst_ctx->shared, src_ctx->shared);
    memcpy(dst_ctx->refs, src_ctx->refs, sizeof(dst_ctx->refs));
    dst_ctx->scratch_ref = src_ctx->scratch_ref;
    dst_ctx->refs_mask   = src_ctx->refs_mask;
    dst_ctx->pattern_id  = src_ctx->pattern_id;

    return ff_envideo_update_thread_context(&dst_ctx->core, &src_ctx->core);
}

#if CONFIG_HEVC_ENVIDEO_HWACCEL
const FFHWAccel ff_hevc_envideo_hwaccel = {
    .p.name                = "hevc_envideo",
    .p.type                = AVMEDIA_TYPE_VIDEO,
    .p.id                  = AV_CODEC_ID_HEVC,
    .p.pix_fmt             = AV_PIX_FMT_ENVIDEO,
    .start_frame           = &envideo_hevc_start_frame,
    .end_frame             = &envideo_hevc_end_frame,
    .decode_slice          = &envideo_hevc_decode_slice,
    .init                  = &envideo_hevc_decode_init,
    .uninit                = &envideo_hevc_decode_uninit,
    .frame_params          = &ff_envideo_frame_params,
    .update_thread_context = &envideo_hevc_update_thread_context,
    .frame_priv_data_size  = sizeof(EnvideoHEVCFrameData),
    .priv_data_size        = sizeof(EnvideoHEVCDecodeContext),
    .caps_internal         = HWACCEL_CAP_ASYNC_SAFE | HWACCEL_CAP_THREAD_SAFE,
};
#endif
