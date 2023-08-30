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
#include "mjpegdec.h"
#include "decode.h"
#include "tx1_decode.h"

#include "libavutil/pixdesc.h"
#include "libavutil/tx1_host1x.h"

typedef struct TX1MJPEGDecodeContext {
    TX1DecodeContext core;

    AVFrame *current_frame;
} TX1MJPEGDecodeContext;

static int tx1_mjpeg_decode_uninit(AVCodecContext *avctx) {
    TX1MJPEGDecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    int err;

    av_log(avctx, AV_LOG_DEBUG, "Deinitializing TX1 MJPEG decoder\n");

    err = ff_tx1_decode_uninit(avctx, &ctx->core);
    if (err < 0)
        return err;

    return 0;
}

static int tx1_mjpeg_decode_init(AVCodecContext *avctx) {
    TX1MJPEGDecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    int err;

    av_log(avctx, AV_LOG_DEBUG, "Initializing TX1 MJPEG decoder\n");

    ctx->core.pic_setup_off  = 0;
    ctx->core.status_off     = FFALIGN(ctx->core.pic_setup_off + sizeof(nvjpg_dec_drv_pic_setup_s),
                                       FF_TX1_MAP_ALIGN);
    ctx->core.cmdbuf_off     = FFALIGN(ctx->core.status_off    + sizeof(nvjpg_dec_status),
                                       FF_TX1_MAP_ALIGN);
    ctx->core.bitstream_off  = FFALIGN(ctx->core.cmdbuf_off    + FF_TX1_MAP_ALIGN,
                                       FF_TX1_MAP_ALIGN);
    ctx->core.input_map_size = FFALIGN(ctx->core.bitstream_off + ff_tx1_decode_pick_bitstream_buffer_size(avctx),
                                       0x1000);

    ctx->core.max_cmdbuf_size    =  ctx->core.slice_offsets_off - ctx->core.cmdbuf_off;
    ctx->core.max_bitstream_size =  ctx->core.input_map_size    - ctx->core.bitstream_off;

    ctx->core.is_nvjpg = true;

    err = ff_tx1_decode_init(avctx, &ctx->core);
    if (err < 0)
        goto fail;

    return 0;

fail:
    tx1_mjpeg_decode_uninit(avctx);
    return err;
}

static void tx1_mjpeg_prepare_frame_setup(nvjpg_dec_drv_pic_setup_s *setup, MJpegDecodeContext *s,
                                          TX1MJPEGDecodeContext *ctx)
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
            input_chroma_mode  = 2; /* YUV422H (not sure what nvidia means by that) */
            output_chroma_mode = 1; /* YUV420 */
            memory_mode        = 0; /* NV12 */
            break;
        case AV_PIX_FMT_YUV440P:
        case AV_PIX_FMT_YUVJ440P:
            input_chroma_mode  = 3; /* YUV422V (ditto) */
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
        .output_stride_luma   = ctx->current_frame->linesize[0],
        .output_stride_chroma = ctx->current_frame->linesize[1],

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

        memcpy(setup->huffTab[0][i].symbol, s->raw_huffman_values[0][i], 162);
        memcpy(setup->huffTab[1][i].symbol, s->raw_huffman_values[1][i], 162);
    }

    for (i = 0; i < s->nb_components; ++i) {
        j = s->component_id[i] - 1;
        setup->blkPar[j].ac     = s->ac_index[i];
        setup->blkPar[j].dc     = s->dc_index[i];
        setup->blkPar[j].hblock = s->h_count[i];
        setup->blkPar[j].vblock = s->v_count[i];
        setup->blkPar[j].quant  = s->quant_index[i];
    }

    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 64; ++j)
            setup->quant[i][j] = s->quant_matrixes[i][j];
    }
}

static int tx1_mjpeg_prepare_cmdbuf(AVTX1Cmdbuf *cmdbuf, MJpegDecodeContext *s, TX1MJPEGDecodeContext *ctx,
                                    AVFrame *current_frame)
{
    FrameDecodeData *fdd = (FrameDecodeData *)current_frame->private_ref->data;
    TX1Frame         *tf = fdd->hwaccel_priv;
    AVTX1Map  *input_map = (AVTX1Map *)tf->input_map_ref->data;

    int err;

    err = ff_tx1_cmdbuf_begin(cmdbuf, HOST1X_CLASS_NVJPG);
    if (err < 0)
        return err;

    FF_TX1_PUSH_VALUE(cmdbuf, NVE7D0_SET_APPLICATION_ID,
                      FF_TX1_ENUM(NVE7D0_SET_APPLICATION_ID, ID, NVJPG_DECODER));
    FF_TX1_PUSH_VALUE(cmdbuf, NVE7D0_SET_CONTROL_PARAMS,
                      FF_TX1_VALUE(NVE7D0_SET_CONTROL_PARAMS, DUMP_CYCLE_COUNT, 1));
    FF_TX1_PUSH_VALUE(cmdbuf, NVE7D0_SET_PICTURE_INDEX,
                      FF_TX1_VALUE(NVE7D0_SET_PICTURE_INDEX, INDEX, ctx->core.frame_idx));

    FF_TX1_PUSH_RELOC(cmdbuf, NVE7D0_SET_IN_DRV_PIC_SETUP,
                      input_map, ctx->core.pic_setup_off, NVHOST_RELOC_TYPE_DEFAULT);
    FF_TX1_PUSH_RELOC(cmdbuf, NVE7D0_SET_BITSTREAM,
                      input_map, ctx->core.bitstream_off, NVHOST_RELOC_TYPE_DEFAULT);
    FF_TX1_PUSH_RELOC(cmdbuf, NVE7D0_SET_OUT_STATUS,
                      input_map, ctx->core.status_off,    NVHOST_RELOC_TYPE_DEFAULT);

    FF_TX1_PUSH_RELOC(cmdbuf, NVE7D0_SET_CUR_PIC, ff_tx1_frame_get_fbuf_map(current_frame),
                      0, NVHOST_RELOC_TYPE_DEFAULT);
    FF_TX1_PUSH_RELOC(cmdbuf, NVE7D0_SET_CUR_PIC_CHROMA_U, ff_tx1_frame_get_fbuf_map(current_frame),
                      current_frame->data[1] - current_frame->data[0], NVHOST_RELOC_TYPE_DEFAULT);

    FF_TX1_PUSH_VALUE(cmdbuf, NVE7D0_EXECUTE,
                      FF_TX1_ENUM(NVE7D0_EXECUTE, AWAKEN, ENABLE));

    err = ff_tx1_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    return 0;
}

static int tx1_mjpeg_start_frame(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    MJpegDecodeContext      *s = avctx->priv_data;
    AVFrame             *frame = s->picture;
    TX1MJPEGDecodeContext *ctx = avctx->internal->hwaccel_priv_data;

    int err;

    av_log(avctx, AV_LOG_DEBUG, "Starting MJPEG-TX1 frame with pixel format %s\n",
           av_get_pix_fmt_name(avctx->sw_pix_fmt));

    err = ff_tx1_start_frame(avctx, frame, &ctx->core);
    if (err < 0)
        return err;

    ctx->current_frame = frame;

    return 0;
}

static int tx1_mjpeg_end_frame(AVCodecContext *avctx) {
    MJpegDecodeContext      *s = avctx->priv_data;
    TX1MJPEGDecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    AVFrame             *frame = ctx->current_frame;
    FrameDecodeData       *fdd = (FrameDecodeData *)frame->private_ref->data;
    TX1Frame               *tf = fdd->hwaccel_priv;
    AVTX1Map        *input_map = (AVTX1Map *)tf->input_map_ref->data;

    nvjpg_dec_drv_pic_setup_s *setup;
    uint8_t *mem;
    AVTX1Map *output_map;
    int err;

    av_log(avctx, AV_LOG_DEBUG, "Ending MJPEG-TX1 frame with %u slices -> %u bytes\n",
           ctx->core.num_slices, ctx->core.bitstream_len);

    mem = ff_tx1_map_get_addr(input_map);

    setup = (nvjpg_dec_drv_pic_setup_s *)(mem + ctx->core.pic_setup_off);
    setup->bitstream_offset = 0;
    setup->bitstream_size   = ctx->core.bitstream_len;

    err = tx1_mjpeg_prepare_cmdbuf(&ctx->core.cmdbuf, s, ctx, frame);
    if (err < 0)
        return err;

    output_map = ff_tx1_frame_get_fbuf_map(frame);
    output_map->is_linear = true;

    return ff_tx1_end_frame(avctx, frame, &ctx->core, NULL, 0);
}

static int tx1_mjpeg_decode_slice(AVCodecContext *avctx, const uint8_t *buf, uint32_t buf_size) {
    MJpegDecodeContext      *s = avctx->priv_data;
    TX1MJPEGDecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    AVFrame             *frame = ctx->current_frame;
    FrameDecodeData       *fdd = (FrameDecodeData *)frame->private_ref->data;

    TX1Frame *tf;
    AVTX1Map *input_map;
    uint8_t *mem;

    tf = fdd->hwaccel_priv;
    input_map = (AVTX1Map *)tf->input_map_ref->data;
    mem = ff_tx1_map_get_addr(input_map);

    /* In tx1_mjpeg_start_frame the JFIF headers haven't been entirely parsed yet */
    tx1_mjpeg_prepare_frame_setup((nvjpg_dec_drv_pic_setup_s *)(mem + ctx->core.pic_setup_off), s, ctx);

    return ff_tx1_decode_slice(avctx, frame, buf, buf_size, false);
}

static int tx1_mjpeg_frame_params(AVCodecContext *avctx, AVBufferRef *hw_frames_ctx) {
    AVHWFramesContext *frames_ctx = (AVHWFramesContext *)hw_frames_ctx->data;

    int err;

    err = ff_tx1_frame_params(avctx, hw_frames_ctx);
    if (err < 0)
        return err;

    /*
     * NVJPG can only decode to pitch linear surfaces, which have a
     * 256b alignment requirement in VIC
     */
    frames_ctx->width  = FFALIGN(frames_ctx->width,  256);
    frames_ctx->height = FFALIGN(frames_ctx->height, 4);

    return 0;
}

#if CONFIG_MJPEG_TX1_HWACCEL
const FFHWAccel ff_mjpeg_tx1_hwaccel = {
    .p.name               = "mjpeg_tx1",
    .p.type               = AVMEDIA_TYPE_VIDEO,
    .p.id                 = AV_CODEC_ID_MJPEG,
    .p.pix_fmt            = AV_PIX_FMT_TX1,
    .start_frame          = &tx1_mjpeg_start_frame,
    .end_frame            = &tx1_mjpeg_end_frame,
    .decode_slice         = &tx1_mjpeg_decode_slice,
    .init                 = &tx1_mjpeg_decode_init,
    .uninit               = &tx1_mjpeg_decode_uninit,
    .frame_params         = &tx1_mjpeg_frame_params,
    .priv_data_size       = sizeof(TX1MJPEGDecodeContext),
    .caps_internal        = HWACCEL_CAP_ASYNC_SAFE,
};
#endif
