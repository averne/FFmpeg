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
#include "mjpegdec.h"
#include "decode.h"
#include "envideo_decode.h"

#include "libavutil/pixdesc.h"

typedef struct EnvideoMJPEGDecodeContext {
    FFEnvideoDecodeContext core;
} EnvideoMJPEGDecodeContext;

static int envideo_mjpeg_decode_uninit(AVCodecContext *avctx) {
    EnvideoMJPEGDecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    int err;

    av_log(avctx, AV_LOG_DEBUG, "Deinitializing mjpeg-envideo decoder\n");

    err = ff_envideo_decode_uninit(avctx, &ctx->core);
    if (err < 0)
        return err;

    return 0;
}

static int envideo_mjpeg_decode_init(AVCodecContext *avctx) {
    MJpegDecodeContext          *s = avctx->priv_data;
    EnvideoMJPEGDecodeContext *ctx = avctx->internal->hwaccel_priv_data;


    enum AVPixelFormat fmt;
    int luma, err;

    av_log(avctx, AV_LOG_DEBUG, "Initializing mjpeg-envideo decoder\n");

    /* Reject encodes with known hardware issues */
    if (avctx->profile != AV_PROFILE_MJPEG_HUFFMAN_BASELINE_DCT) {
        av_log(avctx, AV_LOG_ERROR, "Non-baseline encoded jpegs are not supported by NVJPG\n");
        return AVERROR(EINVAL);
    }

    fmt = s->avctx->pix_fmt, luma = s->comp_index[0];
    if ((fmt == AV_PIX_FMT_YUV444P || fmt == AV_PIX_FMT_YUVJ444P)
            && (s->h_count[luma] != 1 || s->v_count[luma] != 1)) {
        av_log(avctx, AV_LOG_ERROR, "Subsampled YUV444 is not supported by NVJPG\n");
        return AVERROR(EINVAL);
    }

    ctx->core.pic_setup_off  = 0;
    ctx->core.status_off     = FFALIGN(ctx->core.pic_setup_off + sizeof(nvjpg_dec_drv_pic_setup_s),
                                       ENVIDEO_MAP_ALIGN);
    ctx->core.cmdbuf_off     = FFALIGN(ctx->core.status_off    + sizeof(nvjpg_dec_status),
                                       ENVIDEO_MAP_ALIGN);
    ctx->core.bitstream_off  = FFALIGN(ctx->core.cmdbuf_off    + ENVIDEO_MAP_ALIGN,
                                       ENVIDEO_MAP_ALIGN);
    ctx->core.input_map_size = FFALIGN(ctx->core.bitstream_off + ff_envideo_decode_pick_bitstream_buffer_size(avctx),
                                       0x1000);

    ctx->core.max_cmdbuf_size    =  ctx->core.slice_offsets_off - ctx->core.cmdbuf_off;
    ctx->core.max_bitstream_size =  ctx->core.input_map_size    - ctx->core.bitstream_off;

    ctx->core.is_nvjpg = true;

    err = ff_envideo_decode_init(avctx, &ctx->core);
    if (err < 0)
        goto fail;

    return 0;

fail:
    envideo_mjpeg_decode_uninit(avctx);
    return err;
}

static void envideo_mjpeg_prepare_frame_setup(nvjpg_dec_drv_pic_setup_s *setup, MJpegDecodeContext *s,
                                              EnvideoMJPEGDecodeContext *ctx)
{
    int input_chroma_mode, output_chroma_mode, memory_mode;
    int i, j;

    switch (s->hwaccel_sw_pix_fmt) {
        case AV_PIX_FMT_GRAY8:
            input_chroma_mode  = 0; /* Monochrome */
            output_chroma_mode = 0; /* Monochrome */
            memory_mode        = 3; /* YUV420, for some reason decoding fails with NV12 */
            break;
        default:
        case AV_PIX_FMT_YUV420P:
        case AV_PIX_FMT_YUVJ420P:
            input_chroma_mode  = 1; /* YUV420 */
            output_chroma_mode = 1; /* YUV420 */
            memory_mode        = 0; /* NV12 */
            break;
        case AV_PIX_FMT_YUV422P:
        case AV_PIX_FMT_YUVJ422P:
            input_chroma_mode  = 2; /* YUV422H */
            output_chroma_mode = 1; /* YUV420 */
            memory_mode        = 0; /* NV12 */
            break;
        case AV_PIX_FMT_YUV440P:
        case AV_PIX_FMT_YUVJ440P:
            input_chroma_mode  = 3; /* YUV422V (ie. YUV440) */
            output_chroma_mode = 1; /* YUV420 */
            memory_mode        = 0; /* NV12 */
            break;
        case AV_PIX_FMT_YUV444P:
        case AV_PIX_FMT_YUVJ444P:
            input_chroma_mode  = 4; /* YUV444 */
            output_chroma_mode = 1; /* YUV420 */
            memory_mode        = 0; /* NV12 */
            break;
    }

    *setup = (nvjpg_dec_drv_pic_setup_s){
        .restart_interval     = s->restart_interval,
        .frame_width          = s->width,
        .frame_height         = s->height,
        .mcu_width            = s->mb_width,
        .mcu_height           = s->mb_height,
        .comp                 = s->nb_components,

        .stream_chroma_mode   = input_chroma_mode,
        .output_chroma_mode   = output_chroma_mode,
        .output_pixel_format  = 0,  /* YUV */
        .output_stride_luma   = s->picture->linesize[0],
        .output_stride_chroma = s->picture->linesize[1],

        .tile_mode            = 0,  /* Pitch linear (tiled formats are unsupported by the T210) */
        .memory_mode          = memory_mode,
        .power2_downscale     = 0,
        .motion_jpeg_type     = 0,  /* Type A */

        .start_mcu_x          = 0,
        .start_mcu_y          = 0,
    };

    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 16; ++j) {
            setup->huffTab[0][i].codeNum[j] = s->raw_huffman_lengths[0][i][j];
            setup->huffTab[1][i].codeNum[j] = s->raw_huffman_lengths[1][i][j];
        }

        memcpy(setup->huffTab[0][i].symbol, s->raw_huffman_values[0][i], sizeof(setup->huffTab[0][i].symbol));
        memcpy(setup->huffTab[1][i].symbol, s->raw_huffman_values[1][i], sizeof(setup->huffTab[1][i].symbol));
    }

    for (i = 0; i < s->nb_components; ++i) {
        j = s->comp_index[i];
        setup->blkPar[j].ac     = s->ac_index   [i];
        setup->blkPar[j].dc     = s->dc_index   [i];
        setup->blkPar[j].hblock = s->h_count    [i];
        setup->blkPar[j].vblock = s->v_count    [i];
        setup->blkPar[j].quant  = s->quant_index[i];
    }

    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 64; ++j)
            setup->quant[i][j] = s->quant_matrixes[i][j];
    }
}

static int envideo_mjpeg_prepare_cmdbuf(EnvideoCmdbuf *cmdbuf, MJpegDecodeContext *s,
                                        EnvideoMJPEGDecodeContext *ctx, AVFrame *current_frame)
{
    FrameDecodeData     *fdd = (FrameDecodeData *)current_frame->private_ref->data;
    FFEnvideoDecodeFrame *tf = fdd->hwaccel_priv;
    AVEnvideoJob        *job = (AVEnvideoJob *)tf->operation.job_ref->data;
    EnvideoMap    *input_map = job->input_map;

    int err;

    err = envideo_cmdbuf_begin(cmdbuf, EnvideoEngine_Nvjpg);
    if (err < 0)
        return err;

    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVE7D0_SET_APPLICATION_ID,
                          DRF_DEF(E7D0, _SET_APPLICATION_ID, _ID, _NVJPG_DECODER));
    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVE7D0_SET_CONTROL_PARAMS,
                          DRF_NUM(E7D0, _SET_CONTROL_PARAMS, _DUMP_CYCLE_COUNT, 1) |
                          DRF_NUM(E7D0, _SET_CONTROL_PARAMS, _GPTIMER_ON,       1));
    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVE7D0_SET_PICTURE_INDEX,
                          DRF_NUM(E7D0, _SET_PICTURE_INDEX, _INDEX, ctx->core.frame_idx));

    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVE7D0_SET_IN_DRV_PIC_SETUP, input_map, ctx->core.pic_setup_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVE7D0_SET_BITSTREAM,        input_map, ctx->core.bitstream_off);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVE7D0_SET_OUT_STATUS,       input_map, ctx->core.status_off);

    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVE7D0_SET_CUR_PIC,
                          av_envideo_frame_get_fbuf_map(current_frame), 0);
    FF_ENVIDEO_PUSH_RELOC(cmdbuf, NVE7D0_SET_CUR_PIC_CHROMA_U,
                          av_envideo_frame_get_fbuf_map(current_frame),
                          current_frame->data[1] - current_frame->data[0]);

    FF_ENVIDEO_PUSH_VALUE(cmdbuf, NVE7D0_EXECUTE,
                          DRF_DEF(E7D0, _EXECUTE, _AWAKEN, _ENABLE));

    err = envideo_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    return 0;
}

static int envideo_mjpeg_start_frame(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    MJpegDecodeContext          *s = avctx->priv_data;
    AVFrame                 *frame = s->picture;
    EnvideoMJPEGDecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    int err;

    av_log(avctx, AV_LOG_DEBUG, "Starting mjpeg-envideo frame with pixel format %s\n",
           av_get_pix_fmt_name(avctx->sw_pix_fmt));

    err = ff_envideo_start_frame(avctx, frame, &ctx->core);
    if (err < 0)
        return err;

    return 0;
}

static int envideo_mjpeg_end_frame(AVCodecContext *avctx) {
    MJpegDecodeContext          *s = avctx->priv_data;
    EnvideoMJPEGDecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    AVFrame                 *frame = s->picture;
    AVEnvideoFrame        *enframe = (AVEnvideoFrame *)frame->buf[0]->data;
    FrameDecodeData           *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame       *tf = fdd->hwaccel_priv;
    AVEnvideoJob              *job = (AVEnvideoJob *)tf->operation.job_ref->data;


    nvjpg_dec_drv_pic_setup_s *setup;
    uint8_t *mem;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Ending mjpeg-envideo frame with %u slices -> %u bytes\n",
           tf->operation.num_slices, tf->operation.bitstream_len);

    if (!tf || !tf->operation.num_slices)
        return 0;

    mem = envideo_map_get_cpu_addr(job->input_map);

    setup = (nvjpg_dec_drv_pic_setup_s *)(mem + ctx->core.pic_setup_off);
    setup->bitstream_offset = 0;
    setup->bitstream_size   = tf->operation.bitstream_len;

    err = envideo_mjpeg_prepare_cmdbuf(job->cmdbuf, s, ctx, frame);
    if (err < 0)
        return err;

    enframe->is_pitch = true;

    return ff_envideo_end_frame(avctx, frame, &ctx->core, NULL, 0);
}

static int envideo_mjpeg_decode_slice(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    MJpegDecodeContext          *s = avctx->priv_data;
    EnvideoMJPEGDecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    AVFrame                 *frame = s->picture;
    FrameDecodeData           *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame       *tf = fdd->hwaccel_priv;
    AVEnvideoJob              *job = (AVEnvideoJob *)tf->operation.job_ref->data;

    uint8_t *mem;

    tf = fdd->hwaccel_priv;
    mem = envideo_map_get_cpu_addr(job->input_map);

    /* The JFIF headers haven't been entirely parsed yet when the start_frame callback is invoked */
    envideo_mjpeg_prepare_frame_setup((nvjpg_dec_drv_pic_setup_s *)(mem + ctx->core.pic_setup_off), s, ctx);

    return ff_envideo_decode_slice(avctx, frame, buf, buf_size, false);
}

#if CONFIG_MJPEG_ENVIDEO_HWACCEL
const FFHWAccel ff_mjpeg_envideo_hwaccel = {
    .p.name         = "mjpeg_envideo",
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_MJPEG,
    .p.pix_fmt      = AV_PIX_FMT_ENVIDEO,
    .start_frame    = &envideo_mjpeg_start_frame,
    .end_frame      = &envideo_mjpeg_end_frame,
    .decode_slice   = &envideo_mjpeg_decode_slice,
    .init           = &envideo_mjpeg_decode_init,
    .uninit         = &envideo_mjpeg_decode_uninit,
    .frame_params   = &ff_envideo_frame_params,
    .priv_data_size = sizeof(EnvideoMJPEGDecodeContext),
    .caps_internal  = HWACCEL_CAP_ASYNC_SAFE,
};
#endif
