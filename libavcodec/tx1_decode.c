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

#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_tx1.h"
#include "libavutil/tx1_host1x.h"
#include "libavutil/pixdesc.h"
#include "libavutil/pixfmt.h"
#include "libavutil/intreadwrite.h"

#include "avcodec.h"
#include "internal.h"
#include "decode.h"
#include "tx1_decode.h"

static void tx1_input_map_free(void *opaque, uint8_t *data) {
    AVTX1Map *map = (AVTX1Map *)data;

    if (!data)
        return;

    ff_tx1_map_destroy(map);

    av_freep(&map);
}

static AVBufferRef *tx1_input_map_alloc(void *opaque, size_t size) {
    TX1DecodeContext            *ctx = opaque;

    AVBufferRef *buffer;
    AVTX1Map    *map;
    int err;

    map = av_mallocz(sizeof(*map));
    if (!map)
        return NULL;

#ifdef __SWITCH__
    map->owner = ctx->channel->channel.fd;
#endif

    err = ff_tx1_map_create(map, ctx->input_map_size,
                            0x100, NVMAP_CACHE_OP_INV);
    if (err < 0)
        return NULL;

    buffer = av_buffer_create((uint8_t *)map, sizeof(map), tx1_input_map_free, ctx, 0);
    if (!buffer)
        goto fail;

    ctx->new_input_buffer = true;

    return buffer;

fail:
    av_log(ctx, AV_LOG_ERROR, "Failed to create buffer\n");
    ff_tx1_map_destroy(map);
    av_freep(map);
    return NULL;
}

int ff_tx1_decode_init(AVCodecContext *avctx, TX1DecodeContext *ctx) {
    AVHWFramesContext *frames_ctx;
    AVHWDeviceContext *hw_device_ctx;
    AVTX1DeviceContext *device_hwctx;

    int err;

    err = ff_decode_get_hw_frames_ctx(avctx, AV_HWDEVICE_TYPE_TX1);
    if (err < 0)
        goto fail;

    frames_ctx    = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
    hw_device_ctx = (AVHWDeviceContext *)frames_ctx->device_ref->data;
    device_hwctx  = hw_device_ctx->hwctx;

    if ((!ctx->is_nvjpg && !device_hwctx->has_nvdec) || (ctx->is_nvjpg && !device_hwctx->has_nvjpg))
        return AVERROR(EACCES);

    ctx->hw_device_ref = av_buffer_ref(frames_ctx->device_ref);
    if (!ctx->hw_device_ref) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    ctx->decoder_pool = av_buffer_pool_init2(sizeof(AVTX1Map), ctx,
                                             tx1_input_map_alloc, NULL);
    if (!ctx->decoder_pool) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    ctx->channel = !ctx->is_nvjpg ? &device_hwctx->nvdec_channel : &device_hwctx->nvjpg_channel;

    err = ff_tx1_cmdbuf_init(&ctx->cmdbuf);
    if (err < 0)
        goto fail;

    err = ff_tx1_dfs_init(hw_device_ctx, ctx->channel, avctx->coded_width, avctx->coded_height,
                          av_q2d(avctx->framerate));
    if (err < 0)
        goto fail;

    return 0;

fail:
    ff_tx1_decode_uninit(avctx, ctx);
    return err;
}

int ff_tx1_decode_uninit(AVCodecContext *avctx, TX1DecodeContext *ctx) {
    AVHWFramesContext    *frames_ctx = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
    AVHWDeviceContext *hw_device_ctx = (AVHWDeviceContext *)frames_ctx->device_ref->data;

    av_buffer_pool_uninit(&ctx->decoder_pool);

    av_buffer_unref(&ctx->hw_device_ref);

    ff_tx1_cmdbuf_deinit(&ctx->cmdbuf);

    ff_tx1_dfs_uninit(hw_device_ctx, ctx->channel);

    return 0;
}

static void tx1_fdd_priv_free(void *priv) {
    TX1Frame          *tf = priv;
    TX1DecodeContext *ctx = tf->ctx;

    if (!tf)
        return;

    if (tf->in_flight)
        ff_tx1_syncpt_wait(ctx->channel, tf->fence, -1);

    av_buffer_unref(&tf->input_map_ref);
    av_freep(&tf);
}

int ff_tx1_wait_decode(void *logctx, AVFrame *frame) {
    FrameDecodeData             *fdd = (FrameDecodeData *)frame->private_ref->data;
    TX1Frame                     *tf = fdd->hwaccel_priv;
    TX1DecodeContext            *ctx = tf->ctx;
    AVTX1Map              *input_map = (AVTX1Map *)tf->input_map_ref->data;
    AVHWDeviceContext *hw_device_ctx = (AVHWDeviceContext *)ctx->hw_device_ref->data;

    nvdec_status_s *nvdec_status;
    nvjpg_dec_status *nvjpg_status;
    uint32_t decode_cycles;
    uint8_t *mem;
    int err;

    if (!tf->in_flight)
        return 0;

    mem = ff_tx1_map_get_addr(input_map);

    err = ff_tx1_syncpt_wait(ctx->channel, tf->fence, -1);
    if (err < 0)
        return err;

    tf->in_flight = false;

    if (!ctx->is_nvjpg) {
        nvdec_status = (nvdec_status_s *)(mem + ctx->status_off);
        if (nvdec_status->error_status != 0 || nvdec_status->mbs_in_error != 0)
            return AVERROR_UNKNOWN;

        decode_cycles = nvdec_status->decode_cycles * 16;
    } else {
        nvjpg_status = (nvjpg_dec_status *)(mem + ctx->status_off);
        if (nvjpg_status->error_status != 0 || nvjpg_status->bytes_offset == 0)
            return AVERROR_UNKNOWN;

        decode_cycles = nvjpg_status->decode_cycles;
    }

    /* Decode time in µs: decode_cycles * 1000000 / ctx->channel->clock */
    err = ff_tx1_dfs_update(hw_device_ctx, ctx->channel, tf->bitstream_len, decode_cycles);
    if (err < 0)
        return err;

    return 0;
}

int ff_tx1_start_frame(AVCodecContext *avctx, AVFrame *frame, TX1DecodeContext *ctx) {
    FrameDecodeData *fdd = (FrameDecodeData *)frame->private_ref->data;

    TX1Frame *tf = NULL;
    int err;

    ctx->bitstream_len = ctx->num_slices = 0;

    if (fdd->hwaccel_priv) {
        /*
        * For interlaced video, both fields use the same fdd,
        * however by proceeding we might overwrite the input buffer
        * during the decoding, so wait for the previous operation to complete
        */
       err = ff_tx1_wait_decode(avctx, frame);
        if (err < 0)
            return err;
    } else {
        tf = av_mallocz(sizeof(*tf));
        if (!tf)
            return AVERROR(ENOMEM);

        fdd->hwaccel_priv      = tf;
        fdd->hwaccel_priv_free = tx1_fdd_priv_free;
        fdd->post_process      = ff_tx1_wait_decode;

        tf->ctx = ctx;

        tf->input_map_ref = av_buffer_pool_get(ctx->decoder_pool);
        if (!tf->input_map_ref) {
            err = AVERROR(ENOMEM);
            goto fail;
        }
    }

    tf = fdd->hwaccel_priv;
    tf->in_flight = false;

    err = ff_tx1_cmdbuf_add_memory(&ctx->cmdbuf, (AVTX1Map *)tf->input_map_ref->data,
                                   ctx->cmdbuf_off, ctx->max_cmdbuf_size);
    if (err < 0)
        return err;

    err = ff_tx1_cmdbuf_clear(&ctx->cmdbuf);
    if (err < 0)
        return err;

    return 0;

fail:
    tx1_fdd_priv_free(tf);
    return err;
}

int ff_tx1_decode_slice(AVCodecContext *avctx, AVFrame *frame, const uint8_t *buf, uint32_t buf_size,
                        bool add_startcode)
{
    TX1DecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    FrameDecodeData  *fdd = (FrameDecodeData *)frame->private_ref->data;
    TX1Frame          *tf = fdd->hwaccel_priv;
    AVTX1Map   *input_map = (AVTX1Map *)tf->input_map_ref->data;

    bool need_bitstream_move;
    uint32_t old_bitstream_off, startcode_size;
    uint8_t *mem;
    int err;

    mem = ff_tx1_map_get_addr(input_map);

    startcode_size = add_startcode ? 3 : 0;

    /* Reserve 16 bytes for the termination sequence */
    if (ctx->bitstream_len + buf_size + startcode_size >= ctx->max_bitstream_size - 16) {
        ctx->input_map_size += ctx->max_bitstream_size + buf_size;
        ctx->input_map_size = FFALIGN(ctx->input_map_size, 0x1000);

        ctx->max_bitstream_size = ctx->input_map_size - ctx->bitstream_off;

        need_bitstream_move = false;
    }

    /* Reserve 4 bytes for the bitstream size */
    if (ctx->max_num_slices &&  ctx->num_slices >= ctx->max_num_slices - 1) {
        ctx->input_map_size += ctx->max_num_slices * sizeof(uint32_t);
        ctx->input_map_size = FFALIGN(ctx->input_map_size, 0x1000);

        ctx->max_num_slices *= 2;

        old_bitstream_off = ctx->bitstream_off;
        ctx->bitstream_off = ctx->slice_offsets_off + ctx->max_num_slices * sizeof(uint32_t);

        need_bitstream_move = true;
    }

    if (ctx->input_map_size != ff_tx1_map_get_size(input_map)) {
        err = ff_tx1_map_realloc(input_map, ctx->input_map_size, 0x100, NVMAP_CACHE_OP_INV);
        if (err < 0)
            return err;

        mem = ff_tx1_map_get_addr(input_map);

        err = ff_tx1_cmdbuf_add_memory(&ctx->cmdbuf, input_map,
                                       ctx->cmdbuf_off, ctx->max_cmdbuf_size);
        if (err < 0)
            return err;

        /* Running out of slice offsets mem shouldn't happen so the extra memmove is fine */
        if (need_bitstream_move)
            memmove(mem + ctx->bitstream_off, mem + old_bitstream_off, ctx->bitstream_len);
    }

    if (ctx->max_num_slices)
        ((uint32_t *)(mem + ctx->slice_offsets_off))[ctx->num_slices] = ctx->bitstream_len;

    /* NAL startcode 000001 */
    if (add_startcode) {
        AV_WB24(mem + ctx->bitstream_off + ctx->bitstream_len, 1);
        ctx->bitstream_len += 3;
    }

    memcpy(mem + ctx->bitstream_off + ctx->bitstream_len, buf, buf_size);
    ctx->bitstream_len += buf_size;

    ctx->num_slices++;

    return 0;
}

int ff_tx1_end_frame(AVCodecContext *avctx, AVFrame *frame, TX1DecodeContext *ctx,
                     const uint8_t *end_sequence, int end_sequence_size)
{
    FrameDecodeData *fdd = (FrameDecodeData *)frame->private_ref->data;
    TX1Frame         *tf = fdd->hwaccel_priv;
    AVTX1Map  *input_map = (AVTX1Map *)tf->input_map_ref->data;

    uint8_t *mem;
    int err;

    mem = ff_tx1_map_get_addr(input_map);

    /* Last slice data range */
    if (ctx->max_num_slices)
        ((uint32_t *)(mem + ctx->slice_offsets_off))[ctx->num_slices] = ctx->bitstream_len;

    /* Termination sequence for the bitstream data */
    if (end_sequence_size)
        memcpy(mem + ctx->bitstream_off + ctx->bitstream_len, end_sequence, end_sequence_size);

    /* Insert syncpt increment to signal the end of the decoding */
    err = ff_tx1_cmdbuf_begin(&ctx->cmdbuf, !ctx->is_nvjpg ? HOST1X_CLASS_NVDEC : HOST1X_CLASS_NVJPG);
    if (err < 0)
        return err;

    err = ff_tx1_cmdbuf_push_word(&ctx->cmdbuf, host1x_opcode_nonincr(NV_PVIC_THI_INCR_SYNCPT, 1));
    if (err < 0)
        return err;

    err = ff_tx1_cmdbuf_push_word(&ctx->cmdbuf,
        FF_TX1_VALUE(NV_PVIC_THI_INCR_SYNCPT, INDX, ctx->channel->syncpt) |
        FF_TX1_ENUM (NV_PVIC_THI_INCR_SYNCPT, COND, OP_DONE));
    if (err < 0)
        return err;

    err = ff_tx1_cmdbuf_end(&ctx->cmdbuf);
    if (err < 0)
        return err;

    err = ff_tx1_cmdbuf_add_syncpt_incr(&ctx->cmdbuf, ctx->channel->syncpt, 1, 0);
    if (err < 0)
        return err;

    err = ff_tx1_channel_submit(ctx->channel, &ctx->cmdbuf, &tf->fence);
    if (err < 0)
        return err;

    tf->bitstream_len = ctx->bitstream_len;
    tf->in_flight     = true;

    ctx->frame_idx++;

    ctx->new_input_buffer = false;

    return 0;
}

int ff_tx1_frame_params(AVCodecContext *avctx,  AVBufferRef *hw_frames_ctx) {
    AVHWFramesContext *frames_ctx = (AVHWFramesContext *)hw_frames_ctx->data;
    const AVPixFmtDescriptor *sw_desc;

    frames_ctx->format = AV_PIX_FMT_TX1;
    frames_ctx->width  = FFALIGN(avctx->coded_width,  2); /* NVDEC only supports even sizes */
    frames_ctx->height = FFALIGN(avctx->coded_height, 2);

    sw_desc = av_pix_fmt_desc_get(avctx->sw_pix_fmt);
    if (!sw_desc)
        return AVERROR_BUG;

    switch (sw_desc->comp[0].depth) {
        case 8:
            frames_ctx->sw_format = (sw_desc->nb_components > 1) ?
                                    AV_PIX_FMT_NV12 : AV_PIX_FMT_GRAY8;
            break;
        case 10:
            frames_ctx->sw_format = (sw_desc->nb_components > 1) ?
                                    AV_PIX_FMT_P010 : AV_PIX_FMT_GRAY10;
            break;
        default:
            return AVERROR(EINVAL);
    }

    return 0;
}
