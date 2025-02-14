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
#include "libavutil/refstruct.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_envideo.h"

#include "avcodec.h"
#include "codec_desc.h"
#include "internal.h"
#include "decode.h"
#include "envideo_decode.h"

static EnvideoCodec avcodec_to_envideo(enum AVCodecID c) {
    switch (c) {
        case AV_CODEC_ID_MJPEG:      return EnvideoCodec_Mjpeg;
        case AV_CODEC_ID_MPEG1VIDEO: return EnvideoCodec_Mpeg1;
        case AV_CODEC_ID_MPEG2VIDEO: return EnvideoCodec_Mpeg2;
        case AV_CODEC_ID_MPEG4:      return EnvideoCodec_Mpeg4;
        case AV_CODEC_ID_WMV3:
        case AV_CODEC_ID_VC1:        return EnvideoCodec_Vc1;
        case AV_CODEC_ID_H264:       return EnvideoCodec_H264;
        case AV_CODEC_ID_HEVC:       return EnvideoCodec_H265;
        case AV_CODEC_ID_VP8:        return EnvideoCodec_Vp8;
        case AV_CODEC_ID_VP9:        return EnvideoCodec_Vp9;
        case AV_CODEC_ID_AV1:        return EnvideoCodec_Av1;
        default:                     return (EnvideoCodec)-1;
    }
}

static void envideo_shared_free(AVRefStructOpaque opaque, void *obj) {
    FFEnvideoDecodeContextShared *shared = obj;

    av_envideo_job_pool_uninit(&shared->pool);

    if (shared->channel) {
        envideo_dfs_finalize(shared->channel);
        envideo_channel_destroy(shared->channel);
    }

    av_buffer_unref(&shared->hw_device_ref);
}

int ff_envideo_alloc_shared(FFEnvideoDecodeContext *ctx) {
    ctx->shared = av_refstruct_alloc_ext(sizeof(*ctx->shared), 0, NULL, envideo_shared_free);
    return !ctx->shared ? AVERROR(ENOMEM) : 0;
}

int ff_envideo_decode_init(AVCodecContext *avctx, FFEnvideoDecodeContext *ctx) {
    FFEnvideoDecodeContextShared *s = ctx->shared;

    AVHWFramesContext      *frames_ctx;
    AVEnvideoDeviceContext *device_ctx;
    int err;

    err = ff_decode_get_hw_frames_ctx(avctx, AV_HWDEVICE_TYPE_ENVIDEO);
    if (err < 0)
        goto fail;

    frames_ctx = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
    device_ctx = frames_ctx->device_ctx->hwctx;

    s->hw_device_ref = av_buffer_ref(frames_ctx->device_ref);
    if (!s->hw_device_ref) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    err = envideo_channel_create(device_ctx->device, &s->channel,
                                 !s->is_nvjpg ? EnvideoEngine_Nvdec : EnvideoEngine_Nvjpg);
    if (err < 0)
        goto fail;

    err = envideo_dfs_initialize(s->channel, av_q2d(avctx->framerate));
    if (err < 0)
        goto fail;

    err = av_envideo_job_pool_init(&s->pool, device_ctx->device, s->channel,
                                   ctx->input_map_size, ENVIDEO_MAP_ALIGN,
                                   EnvideoMap_CpuWriteCombine | EnvideoMap_GpuUncacheable | EnvideoMap_UsageCmdbuf,
                                   s->cmdbuf_off, s->max_cmdbuf_size);
    if (err < 0)
        goto fail;

    return 0;

fail:
    ff_envideo_decode_uninit(avctx, ctx);
    return err;
}

int ff_envideo_decode_uninit(AVCodecContext *avctx, FFEnvideoDecodeContext *ctx) {
    AVHWFramesContext      *frames_ctx;
    AVEnvideoDeviceContext *device_ctx;
    FFEnvideoOperation *op;
    int i;

    if (avctx->hw_frames_ctx) {
        frames_ctx    = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
        device_ctx    = frames_ctx->device_ctx->hwctx;

        for (i = 0; i < ctx->num_operations; ++i) {
            op  = &ctx->operations[i];
            envideo_fence_wait(device_ctx->device, op->fence, UINT64_MAX);
            av_buffer_unref(&op->job_ref);
        }
    }

    av_freep(&ctx->operations);
    ctx->num_operations = 0;

    av_refstruct_unref(&ctx->shared);

    return 0;
}

static void envideo_fdd_priv_free(void *priv) {
    FFEnvideoDecodeFrame *tf = priv;
    if (!tf)
        return;

    av_buffer_unref(&tf->operation.job_ref);
    av_freep(&tf);
}

int ff_envideo_wait_decode(void *logctx, AVFrame *frame) {
    FrameDecodeData               *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame           *tf = fdd->hwaccel_priv;
    FFEnvideoDecodeContext        *ctx = tf->ctx;
    AVHWDeviceContext   *hw_device_ctx = (AVHWDeviceContext *)ctx->shared->hw_device_ref->data;
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
    AVHWDeviceContext   *hw_device_ctx = (AVHWDeviceContext *)ctx->shared->hw_device_ref->data;
    AVEnvideoDeviceContext *device_ctx = hw_device_ctx->hwctx;
    FFEnvideoDecodeContextShared   *sc = ctx->shared;

    FFEnvideoOperation   *op = NULL;
    FFEnvideoDecodeFrame *tf = NULL;
    AVEnvideoJob *job;
    nvdec_status_s   *nvdec_status;
    nvjpg_dec_status *nvjpg_status;
    uint32_t decode_cycles;
    uint8_t *mem;
    bool is_done, new_buffer = false;
    int i, err = 0;

    /* Abort on resolution changes that wouldn't fit into the frame */
    if ((frame->width > frames_ctx->width) || (frame->height > frames_ctx->height))
        return AVERROR(EINVAL);

    /**
     * Free up input buffers from the pool if the associated job is completed.
     * Simultaneously, check for the statuses of the decoding operations,
     * and update the frequency scaling state.
     */
    for (i = 0; i < ctx->num_operations; ++i) {
        op = &ctx->operations[i];
        if (!op->job_ref)
            continue;

        err = envideo_fence_poll(device_ctx->device, op->fence, &is_done);
        if (err < 0 || !is_done)
            continue;

        job = (AVEnvideoJob *)op->job_ref->data;
        mem = envideo_map_get_cpu_addr(job->input_map);

        if (!sc->is_nvjpg) {
            nvdec_status = (nvdec_status_s *)(mem + sc->status_off);
            if (nvdec_status->error_status != 0 || nvdec_status->mbs_in_error != 0)
                err = AVERROR_UNKNOWN;

            decode_cycles = nvdec_status->cycle_count * 16;
        } else {
            nvjpg_status = (nvjpg_dec_status *)(mem + sc->status_off);
            if (nvjpg_status->error_status != 0 || nvjpg_status->bytes_offset == 0)
                err = AVERROR_UNKNOWN;

            decode_cycles = nvjpg_status->cycle_count;
        }

        av_buffer_unref(&op->job_ref);

        if (err < 0)
            break;

        err = envideo_dfs_update(sc->channel, op->bitstream_len, decode_cycles);
        if (err < 0)
            break;
    }

    if (err < 0)
        return err;

    /* Perform frequency scaling */
    err = envideo_dfs_commit(sc->channel);
    if (err < 0)
        return err;

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

        tf->operation.job_ref = av_envideo_job_pool_get(&sc->pool, &new_buffer);
        if (!tf->operation.job_ref) {
            err = AVERROR(ENOMEM);
            goto fail;
        }
    }

    tf = fdd->hwaccel_priv;
    tf->in_flight        = false;
    tf->new_input_buffer = new_buffer;

    op  = &tf->operation;
    job = (AVEnvideoJob *)op->job_ref->data;
    op->bitstream_len = op->num_slices = 0;

    err = envideo_cmdbuf_clear(job->cmdbuf);
    if (err < 0)
        return err;

    err = envideo_map_pin(av_envideo_frame_get_fbuf_map(frame), sc->channel);
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
    FFEnvideoDecodeContext      *ctx = avctx->internal->hwaccel_priv_data;
    FFEnvideoDecodeContextShared *sc = ctx->shared;
    FrameDecodeData             *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame         *tf = fdd->hwaccel_priv;
    FFEnvideoOperation           *op = &tf->operation;
    AVEnvideoJob                *job = (AVEnvideoJob *)op->job_ref->data;
    EnvideoMap            *input_map = job->input_map;

    uint32_t startcode_size;
    uint8_t *mem;
    int err;

    startcode_size = add_startcode ? 3 : 0;

    /* Reserve 4 bytes for the bitstream size */
    if (sc->max_num_slices && op->num_slices >= sc->max_num_slices - 1)
        return AVERROR(ENOMEM);

    /* Reserve 16 bytes for the termination sequence */
    if (op->bitstream_len + buf_size + startcode_size >= ctx->max_bitstream_size - 16) {
        ctx->input_map_size += ctx->max_bitstream_size + buf_size;
        ctx->input_map_size  = FFALIGN(ctx->input_map_size, 0x1000);

        ctx->max_bitstream_size = ctx->input_map_size - sc->bitstream_off;
    }

    if (ctx->input_map_size != envideo_map_get_size(input_map)) {
        err = av_envideo_job_realloc(&sc->pool, job, ctx->input_map_size, ENVIDEO_MAP_ALIGN);
        if (err < 0)
            return err;
    }

    mem = envideo_map_get_cpu_addr(input_map);

    if (sc->max_num_slices)
        ((uint32_t *)(mem + sc->slice_offsets_off))[op->num_slices] = op->bitstream_len;

    /* NAL startcode 000001 */
    if (add_startcode) {
        AV_WB24(mem + sc->bitstream_off + op->bitstream_len, 1);
        op->bitstream_len += 3;
    }

    memcpy(mem + sc->bitstream_off + op->bitstream_len, buf, buf_size);
    op->bitstream_len += buf_size;
    op->num_slices++;

    return 0;
}

int ff_envideo_end_frame(AVCodecContext *avctx, AVFrame *frame, FFEnvideoDecodeContext *ctx,
                         const uint8_t *end_sequence, int end_sequence_size)
{
    FFEnvideoDecodeContextShared *sc = ctx->shared;
    FrameDecodeData             *fdd = (FrameDecodeData *)frame->private_ref->data;
    FFEnvideoDecodeFrame         *tf = fdd->hwaccel_priv;
    FFEnvideoOperation           *op = &tf->operation;
    AVEnvideoJob                *job = (AVEnvideoJob *)op->job_ref->data;
    EnvideoMap            *input_map = job->input_map;
    AVEnvideoFrame          *evframe = (AVEnvideoFrame *)frame->buf[0]->data;

    uint8_t *mem;
    int i, err;

    mem = envideo_map_get_cpu_addr(input_map);

    /* Last slice data range */
    if (sc->max_num_slices)
        ((uint32_t *)(mem + sc->slice_offsets_off))[op->num_slices] = op->bitstream_len;

    /* Termination sequence for the bitstream data */
    if (end_sequence_size)
        memcpy(mem + sc->bitstream_off + op->bitstream_len, end_sequence, end_sequence_size);

    for (i = 0; i < ctx->num_operations; ++i) {
        if (!ctx->operations[i].job_ref)
            break;
    }

    if (i == ctx->num_operations) {
        ctx->operations = av_realloc_array(ctx->operations, ctx->num_operations + 1, sizeof(FFEnvideoOperation));
        if (!ctx->operations)
            return AVERROR(ENOMEM);

        ctx->operations[ctx->num_operations++] = (FFEnvideoOperation){0};
    }

    op = &ctx->operations[i];

    err = envideo_channel_submit(sc->channel, job->cmdbuf, &tf->operation.fence);
    if (err < 0)
        return err;

    tf->in_flight = true;

    err = av_buffer_replace(&op->job_ref, tf->operation.job_ref);
    if (err < 0)
        return err;

    op->fence         = tf->operation.fence;
    op->bitstream_len = tf->operation.bitstream_len;
    evframe->fence    = op->fence;

    ctx->frame_idx++;

    return 0;
}

int ff_envideo_update_thread_context(FFEnvideoDecodeContext *dst, const FFEnvideoDecodeContext *src) {
    av_refstruct_replace(&dst->shared, src->shared);
    dst->frame_idx          = src->frame_idx;
    dst->input_map_size     = src->input_map_size;
    dst->max_bitstream_size = src->max_bitstream_size;

    return 0;
}

int ff_envideo_frame_params(AVCodecContext *avctx, AVBufferRef *hw_frames_ctx) {
    AVHWFramesContext      *frames_ctx = (AVHWFramesContext *)hw_frames_ctx->data;
    AVEnvideoDeviceContext *device_ctx = frames_ctx->device_ctx->hwctx;

    const AVPixFmtDescriptor *sw_desc;
    EnvideoDecodeConstraints constraints;
    int num_planes, num_mbs, err;

    sw_desc = av_pix_fmt_desc_get(avctx->sw_pix_fmt);
    if (!sw_desc)
        return AVERROR_BUG;

    constraints.codec = avcodec_to_envideo(avctx->codec_id);
    constraints.depth = sw_desc->comp[0].depth;

    num_planes = av_pix_fmt_count_planes(avctx->sw_pix_fmt);
    if (num_planes == 1)
        constraints.subsample = EnvideoSubsampling_Monochrome;
    else if (sw_desc->log2_chroma_w == 1 && sw_desc->log2_chroma_h == 1)
        constraints.subsample = EnvideoSubsampling_420;
    else if (sw_desc->log2_chroma_w == 1 && sw_desc->log2_chroma_h == 0)
        constraints.subsample = EnvideoSubsampling_422;
    else if (sw_desc->log2_chroma_w == 0 && sw_desc->log2_chroma_h == 0)
        constraints.subsample = EnvideoSubsampling_444;
    else
        return AVERROR(EINVAL);

    err = envideo_get_decode_constraints(device_ctx->device, &constraints);
    if (err < 0)
        return err;

    if (!constraints.supported) {
        av_log(avctx, AV_LOG_ERROR, "Codec %s is not supported by the hardware\n",
               avctx->codec_descriptor->name);
        return AVERROR(EINVAL);
    }

    if ((avctx->coded_width  < constraints.min_width)  || (avctx->coded_width  > constraints.max_width) ||
        (avctx->coded_height < constraints.min_height) || (avctx->coded_height > constraints.max_height))
    {
        av_log(avctx, AV_LOG_ERROR, "Dimensions %dx%d (min. %dx%d, max. %dx%d) "
                                    "are not supported by the hardware for codec %s\n",
               avctx->coded_width, avctx->coded_height,
               constraints.min_width, constraints.min_height,
               constraints.max_width, constraints.max_height,
               avctx->codec_descriptor->name);
        return AVERROR(EINVAL);
    }

    num_mbs = (FFALIGN(avctx->coded_width, 16) / 16) * (FFALIGN(avctx->coded_height, 16) / 16);
    if ((constraints.max_mbs > 0) && (num_mbs > constraints.max_mbs)) {
        av_log(avctx, AV_LOG_ERROR, "Number of macroblocks %d exceeds maximum %d "
                                    "for codec %s\n",
               num_mbs, constraints.max_mbs, avctx->codec_descriptor->name);
        return AVERROR(EINVAL);
    }

    frames_ctx->format = AV_PIX_FMT_ENVIDEO;
    frames_ctx->width  = avctx->coded_width;
    frames_ctx->height = avctx->coded_height;

    switch (sw_desc->comp[0].depth) {
        case 8:
            switch (constraints.subsample) {
                case EnvideoSubsampling_Monochrome:
                    frames_ctx->sw_format = AV_PIX_FMT_GRAY8;
                    break;
                case EnvideoSubsampling_420:
                    frames_ctx->sw_format = AV_PIX_FMT_NV12;
                    break;
                case EnvideoSubsampling_422:
                    frames_ctx->sw_format = AV_PIX_FMT_NV16;
                    break;
                case EnvideoSubsampling_444:
                    frames_ctx->sw_format = AV_PIX_FMT_YUV444P;
                    break;
            }
            break;
        case 10:
            switch (constraints.subsample) {
                case EnvideoSubsampling_Monochrome:
                    frames_ctx->sw_format = AV_PIX_FMT_GRAY10LE;
                    break;
                case EnvideoSubsampling_420:
                    frames_ctx->sw_format = AV_PIX_FMT_P010LE;
                    break;
                case EnvideoSubsampling_422:
                    frames_ctx->sw_format = AV_PIX_FMT_P210LE;
                    break;
                case EnvideoSubsampling_444:
                    frames_ctx->sw_format = AV_PIX_FMT_YUV444P10LE;
                    break;
            }
            break;
        case 12:
            switch (constraints.subsample) {
                case EnvideoSubsampling_Monochrome:
                    frames_ctx->sw_format = AV_PIX_FMT_GRAY12LE;
                    break;
                case EnvideoSubsampling_420:
                    frames_ctx->sw_format = AV_PIX_FMT_P012LE;
                    break;
                case EnvideoSubsampling_422:
                    frames_ctx->sw_format = AV_PIX_FMT_P212LE;
                    break;
                case EnvideoSubsampling_444:
                    frames_ctx->sw_format = AV_PIX_FMT_YUV444P12LE;
                    break;
            }
            break;
        default:
            return AVERROR(EINVAL);
    }

    return 0;
}
