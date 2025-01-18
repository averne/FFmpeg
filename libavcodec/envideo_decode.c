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

#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavutil/pixfmt.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_envideo.h"

#include "avcodec.h"
#include "codec_desc.h"
#include "internal.h"
#include "decode.h"
#include "envideo_decode.h"

static void envideo_input_map_free(void *opaque, uint8_t *data) {
    EnvideoMap *map = (EnvideoMap *)data;

    if (!data)
        return;

    envideo_map_destroy(map);
}

static AVBufferRef *envideo_input_map_alloc(void *opaque, size_t size) {
    FFEnvideoDecodeContext        *ctx = opaque;
    AVHWDeviceContext   *hw_device_ctx = (AVHWDeviceContext *)ctx->hw_device_ref->data;
    AVEnvideoDeviceContext *device_ctx = hw_device_ctx->hwctx;

    AVBufferRef *buffer;
    EnvideoMap  *map;
    int err;

    err = envideo_map_create(device_ctx->device, &map, ctx->input_map_size, ENVIDEO_MAP_ALIGN,
                             EnvideoMap_CpuWriteCombine | EnvideoMap_GpuCacheable | EnvideoMap_UsageEngine);
    if (err < 0)
        return NULL;

    buffer = av_buffer_create((uint8_t *)map, sizeof(map), envideo_input_map_free, ctx, 0);
    if (!buffer)
        goto fail;

    ctx->new_input_buffer = true;

    return buffer;

fail:
    av_log(hw_device_ctx, AV_LOG_ERROR, "Failed to create input buffer for decode job\n");
    envideo_map_destroy(map);
    av_freep(map);
    return NULL;
}

int ff_envideo_decode_init(AVCodecContext *avctx, FFEnvideoDecodeContext *ctx) {
    AVHWFramesContext      *frames_ctx;
    AVHWDeviceContext      *hw_device_ctx;
    AVEnvideoDeviceContext *device_ctx;

    int err;

    err = ff_decode_get_hw_frames_ctx(avctx, AV_HWDEVICE_TYPE_ENVIDEO);
    if (err < 0)
        goto fail;

    frames_ctx    = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
    hw_device_ctx = (AVHWDeviceContext *)frames_ctx->device_ref->data;
    device_ctx    = hw_device_ctx->hwctx;

    ctx->hw_device_ref = av_buffer_ref(frames_ctx->device_ref);
    if (!ctx->hw_device_ref) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    ctx->decoder_pool = av_buffer_pool_init2(ctx->input_map_size, ctx,
                                             envideo_input_map_alloc, NULL);
    if (!ctx->decoder_pool) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    err = envideo_channel_create(device_ctx->device, &ctx->channel,
                                 !ctx->is_nvjpg ? EnvideoEngine_Nvdec : EnvideoEngine_Nvjpg);
    if (err < 0)
        goto fail;

    err = envideo_cmdbuf_create(ctx->channel, &ctx->cmdbuf);
    if (err < 0)
        goto fail;

    return 0;

fail:
    ff_envideo_decode_uninit(avctx, ctx);
    return err;
}

int ff_envideo_decode_uninit(AVCodecContext *avctx, FFEnvideoDecodeContext *ctx) {
    AVHWFramesContext      *frames_ctx;
    AVHWDeviceContext      *hw_device_ctx;
    AVEnvideoDeviceContext *device_ctx;
    int i;

    if (avctx->hw_frames_ctx) {
        frames_ctx    = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
        hw_device_ctx = (AVHWDeviceContext *)frames_ctx->device_ref->data;
        device_ctx    = hw_device_ctx->hwctx;

        for (i = 0; i < ctx->num_operations; ++i) {
            envideo_fence_wait(device_ctx->device, ctx->operations[i].fence, UINT64_MAX);
            av_buffer_unref(&ctx->operations[i].input_map_ref);
        }
    }

    av_freep(&ctx->operations);
    ctx->num_operations = 0;

    av_buffer_pool_uninit(&ctx->decoder_pool);

    av_buffer_unref(&ctx->hw_device_ref);

    envideo_cmdbuf_destroy(ctx->cmdbuf);

    envideo_channel_destroy(ctx->channel);

    return 0;
}

static void envideo_fdd_priv_free(void *priv) {
    FFEnvideoDecodeFrame *tf = priv;
    if (!tf)
        return;

    av_buffer_unref(&tf->operation.input_map_ref);
    av_freep(&tf);
}

int ff_envideo_wait_decode(void *logctx, AVFrame *frame) {
    FrameDecodeData               *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame           *tf = fdd->hwaccel_priv;
    FFEnvideoDecodeContext        *ctx = tf->ctx;
    AVHWDeviceContext   *hw_device_ctx = (AVHWDeviceContext *)ctx->hw_device_ref->data;
    AVEnvideoDeviceContext *device_ctx = hw_device_ctx->hwctx;

    int err;

    if (!tf->in_flight)
        return 0;

    err = envideo_fence_wait(device_ctx->device, tf->operation.fence, UINT64_MAX);
    if (err < 0)
        return err;

    tf->in_flight = false;

    return 0;
}

int ff_envideo_start_frame(AVCodecContext *avctx, AVFrame *frame, FFEnvideoDecodeContext *ctx) {
    AVHWFramesContext      *frames_ctx = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
    FrameDecodeData               *fdd = (FrameDecodeData *)frame->private_ref->data;
    AVHWDeviceContext   *hw_device_ctx = (AVHWDeviceContext *)ctx->hw_device_ref->data;
    AVEnvideoDeviceContext *device_ctx = hw_device_ctx->hwctx;

    FFEnvideoDecodeFrame *tf = NULL;
    bool is_done;
    int i, err;

    /* Abort on resolution changes that wouldn't fit into the frame */
    if ((frame->width > frames_ctx->width) || (frame->height > frames_ctx->height))
        return AVERROR(EINVAL);

    ctx->bitstream_len = ctx->num_slices = 0;

    /* Free up input buffers from the pool if the associated job is completed */
    for (i = 0; i < ctx->num_operations; ++i) {
        err = envideo_fence_poll(device_ctx->device, ctx->operations[i].fence, &is_done);
        if (err < 0)
            continue;

        if (is_done)
            av_buffer_unref(&ctx->operations[i].input_map_ref);
    }

    if (fdd->hwaccel_priv) {
        /**
        * For interlaced video, both fields use the same fdd,
        * however by proceeding we might overwrite the input buffer
        * during the decoding, so wait for the previous operation to complete.
        */
       err = ff_envideo_wait_decode(avctx, frame);
        if (err < 0)
            return err;
    } else {
        tf = av_mallocz(sizeof(*tf));
        if (!tf)
            return AVERROR(ENOMEM);

        fdd->hwaccel_priv      = tf;
        fdd->hwaccel_priv_free = envideo_fdd_priv_free;

        tf->ctx = ctx;

        tf->operation.input_map_ref = av_buffer_pool_get(ctx->decoder_pool);
        if (!tf->operation.input_map_ref) {
            err = AVERROR(ENOMEM);
            goto fail;
        }
    }

    tf = fdd->hwaccel_priv;
    tf->in_flight = false;

    err = envideo_cmdbuf_add_memory(ctx->cmdbuf, (EnvideoMap *)tf->operation.input_map_ref->data,
                                    ctx->cmdbuf_off, ctx->max_cmdbuf_size);
    if (err < 0)
        return err;

    err = envideo_cmdbuf_clear(ctx->cmdbuf);
    if (err < 0)
        return err;

    return 0;

fail:
    envideo_fdd_priv_free(tf);
    return err;
}

int ff_envideo_decode_slice(AVCodecContext *avctx, AVFrame *frame,
                            const uint8_t *buf, uint32_t buf_size, bool add_startcode)
{
    FFEnvideoDecodeContext *ctx = avctx->internal->hwaccel_priv_data;
    FrameDecodeData        *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame    *tf = fdd->hwaccel_priv;
    EnvideoMap       *input_map = (EnvideoMap *)tf->operation.input_map_ref->data;

    bool need_bitstream_move = false;
    uint32_t old_bitstream_off, startcode_size;
    uint8_t *mem;
    int err;

    mem = envideo_map_get_cpu_addr(input_map);

    startcode_size = add_startcode ? 3 : 0;

    /* Reserve 16 bytes for the termination sequence */
    if (ctx->bitstream_len + buf_size + startcode_size >= ctx->max_bitstream_size - 16) {
        ctx->input_map_size += ctx->max_bitstream_size + buf_size;
        ctx->input_map_size  = FFALIGN(ctx->input_map_size, 0x1000);

        ctx->max_bitstream_size = ctx->input_map_size - ctx->bitstream_off;

        need_bitstream_move = false;
    }

    /* Reserve 4 bytes for the bitstream size */
    if (ctx->max_num_slices &&  ctx->num_slices >= ctx->max_num_slices - 1) {
        ctx->input_map_size += ctx->max_num_slices * sizeof(uint32_t);
        ctx->input_map_size  = FFALIGN(ctx->input_map_size, 0x1000);

        ctx->max_num_slices *= 2;

        old_bitstream_off = ctx->bitstream_off;
        ctx->bitstream_off = ctx->slice_offsets_off + ctx->max_num_slices * sizeof(uint32_t);

        need_bitstream_move = true;
    }

    if (ctx->input_map_size != envideo_map_get_size(input_map)) {
        err = envideo_map_realloc(input_map, ctx->input_map_size, ENVIDEO_MAP_ALIGN);
        if (err < 0)
            return err;

        mem = envideo_map_get_cpu_addr(input_map);

        err = envideo_cmdbuf_add_memory(ctx->cmdbuf, input_map,
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

int ff_envideo_end_frame(AVCodecContext *avctx, AVFrame *frame, FFEnvideoDecodeContext *ctx,
                         const uint8_t *end_sequence, int end_sequence_size)
{
    FrameDecodeData        *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame    *tf = fdd->hwaccel_priv;
    EnvideoMap       *input_map = (EnvideoMap *)tf->operation.input_map_ref->data;
    AVEnvideoFrame     *evframe = (AVEnvideoFrame *)frame->buf[0]->data;

    uint8_t *mem;
    int i, err;

    mem = envideo_map_get_cpu_addr(input_map);

    /* Last slice data range */
    if (ctx->max_num_slices)
        ((uint32_t *)(mem + ctx->slice_offsets_off))[ctx->num_slices] = ctx->bitstream_len;

    /* Termination sequence for the bitstream data */
    if (end_sequence_size)
        memcpy(mem + ctx->bitstream_off + ctx->bitstream_len, end_sequence, end_sequence_size);

    for (i = 0; i < ctx->num_operations; ++i) {
        if (!ctx->operations[i].input_map_ref)
            break;
    }

    if (i == ctx->num_operations) {
        ctx->operations = av_realloc_array(ctx->operations, ctx->num_operations + 1, sizeof(FFEnvideoOperation));
        if (!ctx->operations)
            return AVERROR(ENOMEM);

        ctx->operations[ctx->num_operations++] = (FFEnvideoOperation){0};
    }

    err = envideo_channel_submit(ctx->channel, ctx->cmdbuf, &tf->operation.fence);
    if (err < 0)
        return err;

    tf->in_flight = true;

    err = av_buffer_replace(&ctx->operations[i].input_map_ref, tf->operation.input_map_ref);
    if (err < 0)
        return err;

    evframe->fence = ctx->operations[i].fence = tf->operation.fence;

    ctx->frame_idx++;
    ctx->new_input_buffer = false;

    return 0;
}

static int envideo_get_size_constraints(enum AVCodecID codec,
                                        int *min_width, int *min_height,
                                        int *max_width, int *max_height,
                                        int *align,     int *max_mbs)
{
    switch (codec) {
        case AV_CODEC_ID_MPEG1VIDEO:
        case AV_CODEC_ID_MPEG2VIDEO:
            *min_width = 48,    *min_height = 1;
            *max_width = 4096,  *max_height = 4096;
            *align     = 16,    *max_mbs    = 0x20000;
            break;

        case AV_CODEC_ID_MPEG4:
            *min_width = 48,    *min_height = 1;
            *max_width = 2048,  *max_height = 2048;
            *align     = 16,    *max_mbs    = 0x2000;
            break;

        case AV_CODEC_ID_VC1:
        case AV_CODEC_ID_WMV3:
            *min_width = 48,    *min_height = 1;
            *max_width = 2048,  *max_height = 2048;
            *align     = 1,     *max_mbs    = -1;
            break;

        case AV_CODEC_ID_H264:
            *min_width = 48,    *min_height = 1;
            *max_width = 4096,  *max_height = 4096;
            *align     = 16,    *max_mbs    = 0x20000;
            break;

        case AV_CODEC_ID_HEVC:
            /* Note: on nvdec 4.0+ (tegra 194) max dimensions are 8192, and max mbs 0x80000 */
            *min_width = 144,   *min_height = 144;
            *max_width = 4096,  *max_height = 4096;
            *align     = 64,    *max_mbs    = 0x20000;
            break;

        case AV_CODEC_ID_VP8:
            *min_width = 48,    *min_height = 1;
            *max_width = 4096,  *max_height = 4096;
            *align     = 16,    *max_mbs    = 0x20000;
            break;

        case AV_CODEC_ID_VP9:
            /* Note: on nvdec 4.0+ (tegra 194) max dimensions are 8192, and max mbs 0x40000 */
            *min_width = 144,   *min_height = 144;
            *max_width = 4096,  *max_height = 4096;
            *align     = 16,    *max_mbs    = 0x10000;
            break;

        case AV_CODEC_ID_MJPEG:
            *min_width = 1,     *min_height = 1;
            *max_width = 16384, *max_height = 16384;
            *align     = 1,     *max_mbs    = -1;
            break;

        #if 0
        case AV_CODEC_ID_AV1:
            /* Note: on nvdec 4.0+ (tegra 194) max dimensions are 8192, and max mbs 0x80000 */
            *min_width = 128,   *min_height = 128;
            *max_width = 4096,  *max_height = 4096;
            *align     = 64,    *max_mbs    = 0x20000;
            break;
        #endif

        default:
            return AVERROR(EINVAL);
    }

    return 0;
}

int ff_envideo_frame_params(AVCodecContext *avctx, AVBufferRef *hw_frames_ctx) {
    AVHWFramesContext *frames_ctx = (AVHWFramesContext *)hw_frames_ctx->data;
    const AVPixFmtDescriptor *sw_desc;

    int min_width, min_height, max_width, max_height, align, max_mbs,
        aligned_width, aligned_height, num_mbs;
    int err;

    err = envideo_get_size_constraints(avctx->codec_id, &min_width, &min_height,
                                       &max_width, &max_height, &align, &max_mbs);
    if (err < 0)
        return err;

    aligned_width  = FFALIGN(avctx->coded_width,  align);
    aligned_height = FFALIGN(avctx->coded_height, align);
    num_mbs = (aligned_width / 16) * (aligned_height / 16);

    if ((aligned_width  < min_width)  || (aligned_width  > max_width) ||
        (aligned_height < min_height) || (aligned_height > max_height))
    {
        av_log(avctx, AV_LOG_ERROR, "Dimensions %dx%d (min. %dx%d, max. %dx%d) "
                                    "are not supported by the hardware for codec %s\n",
               avctx->coded_width, avctx->coded_height,
               min_width, min_height, max_width, max_height,
               avctx->codec_descriptor->name);
        return AVERROR(EINVAL);
    }

    if ((max_mbs > 0) && (num_mbs > max_mbs)) {
        av_log(avctx, AV_LOG_ERROR, "Number of macroblocks %d exceeds maximum %d "
                                    "for codec %s\n",
               num_mbs, max_mbs, avctx->codec_descriptor->name);
        return AVERROR(EINVAL);
    }

    frames_ctx->format = AV_PIX_FMT_ENVIDEO;
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
