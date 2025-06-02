/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "proresdec.h"
#include "vulkan_decode.h"
#include "hwaccel_internal.h"
#include "libavutil/mem.h"
#include "libavutil/vulkan.h"
#include "libavutil/vulkan_loader.h"
#include "libavutil/vulkan_spirv.h"

const FFVulkanDecodeDescriptor ff_vk_dec_prores_desc = {
    .codec_id         = AV_CODEC_ID_PRORES,
    // .decode_extension = FF_VK_EXT_PUSH_DESCRIPTOR,
    .queue_flags      = VK_QUEUE_COMPUTE_BIT,
};

typedef struct ProresVulkanDecodePicture {
    FFVulkanDecodePicture vp;
} ProresVulkanDecodePicture;

typedef struct ProresVulkanDecodeContext {

} ProresVulkanDecodeContext;

static int vk_prores_start_frame(AVCodecContext          *avctx,
                                 const AVBufferRef       *buffer_ref,
                                 av_unused const uint8_t *buffer,
                                 av_unused uint32_t       size)
{
    return 0;
}

static int vk_prores_decode_slice(AVCodecContext *avctx,
                                  const uint8_t  *data,
                                  uint32_t        size)
{
    return 0;
}

static int vk_prores_end_frame(AVCodecContext *avctx)
{
    return 0;
}

static void vk_decode_prores_uninit(FFVulkanDecodeShared *ctx)
{
    ProresVulkanDecodeContext *pv = ctx->sd_ctx;
}


static int vk_decode_prores_init(AVCodecContext *avctx)
{
    FFVulkanDecodeContext *dec = avctx->internal->hwaccel_priv_data;
    FFVulkanDecodeShared  *ctx = NULL;

    ProresVulkanDecodeContext *pv;
    FFVkSPIRVCompiler *spv;
    int err;

    spv = ff_vk_spirv_init();
    if (!spv) {
        av_log(avctx, AV_LOG_ERROR, "Unable to initialize SPIR-V compiler!\n");
        return AVERROR_EXTERNAL;
    }

    err = ff_vk_decode_init(avctx);
    if (err < 0)
        return err;
    ctx = dec->shared_ctx;

    pv = ctx->sd_ctx = av_mallocz(sizeof(*pv));
    if (!pv) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    ctx->sd_ctx_free = &vk_decode_prores_uninit;

    av_log(avctx, AV_LOG_DEBUG, "Initialized Vulkan-Prores decoder\n");

    err = 0;

fail:
    return err;
}

static void vk_prores_free_frame_priv(AVRefStructOpaque _hwctx, void *data)
{
    return;
}

const FFHWAccel ff_prores_vulkan_hwaccel = {
    .p.name                = "prores_vulkan",
    .p.type                = AVMEDIA_TYPE_VIDEO,
    .p.id                  = AV_CODEC_ID_PRORES,
    .p.pix_fmt             = AV_PIX_FMT_VULKAN,
    .start_frame           = &vk_prores_start_frame,
    .decode_slice          = &vk_prores_decode_slice,
    .end_frame             = &vk_prores_end_frame,
    .free_frame_priv       = &vk_prores_free_frame_priv,
    .frame_priv_data_size  = sizeof(ProresVulkanDecodePicture),
    .init                  = &vk_decode_prores_init,
    .update_thread_context = &ff_vk_update_thread_context,
    .decode_params         = &ff_vk_params_invalidate,
    .flush                 = &ff_vk_decode_flush,
    .uninit                = &ff_vk_decode_uninit,
    .frame_params          = &ff_vk_frame_params,
    .priv_data_size        = sizeof(FFVulkanDecodeContext),
    .caps_internal         = HWACCEL_CAP_ASYNC_SAFE | HWACCEL_CAP_THREAD_SAFE,
};
