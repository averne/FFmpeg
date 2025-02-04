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
    AVHWDeviceContext      *hw_device_ctx;
    AVEnvideoDeviceContext *device_ctx;
    int err;

    err = ff_decode_get_hw_frames_ctx(avctx, AV_HWDEVICE_TYPE_ENVIDEO);
    if (err < 0)
        goto fail;

    frames_ctx    = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
    hw_device_ctx = (AVHWDeviceContext *)frames_ctx->device_ref->data;
    device_ctx    = hw_device_ctx->hwctx;

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

    return 0;

fail:
    ff_envideo_decode_uninit(avctx, ctx);
    return err;
}

int ff_envideo_decode_uninit(AVCodecContext *avctx, FFEnvideoDecodeContext *ctx) {
    AVHWFramesContext      *frames_ctx;
    AVHWDeviceContext      *hw_device_ctx;
    AVEnvideoDeviceContext *device_ctx;
    FFEnvideoOperation *op;
    int i;

    if (avctx->hw_frames_ctx) {
        frames_ctx    = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
        hw_device_ctx = (AVHWDeviceContext *)frames_ctx->device_ref->data;
        device_ctx    = hw_device_ctx->hwctx;

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
