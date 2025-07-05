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

extern const char *ff_source_common_comp;
extern const char *ff_source_prores_vld_comp;

const FFVulkanDecodeDescriptor ff_vk_dec_prores_desc = {
    .codec_id         = AV_CODEC_ID_PRORES,
    // .decode_extension = FF_VK_EXT_PUSH_DESCRIPTOR,
    .queue_flags      = VK_QUEUE_COMPUTE_BIT,
};

typedef struct ProresVulkanDecodePicture {
    FFVulkanDecodePicture vp;

    AVBufferRef *slice_offset_buf;
    AVBufferRef *slice_context_buf;
    uint32_t slice_num;

    uint32_t bitstream_start;
    uint32_t bitstream_size;
} ProresVulkanDecodePicture;

typedef struct ProresVulkanDecodeContext {
    FFVulkanShader vld;

    AVBufferPool *slice_offset_pool;
    AVBufferPool *slice_context_pool;
} ProresVulkanDecodeContext;

typedef struct ProresVkParameters {
    VkDeviceAddress slice_data;
    uint32_t bitstream_start;
    uint32_t bitstream_size;

    uint32_t slice_width;
    uint32_t slice_height;
    uint32_t mb_width;
    uint32_t mb_height;
    uint32_t alpha_info;
} ProresVkParameters;

typedef struct {
    uint32_t mb_x;
    uint32_t mb_y;
    uint32_t mb_count;
    uint32_t qscale;
} ProresVkSliceContext;

static int vk_prores_start_frame(AVCodecContext          *avctx,
                                 const AVBufferRef       *buffer_ref,
                                 av_unused const uint8_t *buffer,
                                 av_unused uint32_t       size)
{
    ProresContext             *pr = avctx->priv_data;
    FFVulkanDecodeContext    *dec = avctx->internal->hwaccel_priv_data;
    FFVulkanDecodeShared     *ctx = dec->shared_ctx;
    ProresVulkanDecodeContext *pv = ctx->sd_ctx;
    ProresVulkanDecodePicture *pp = pr->hwaccel_picture_private;
    FFVulkanDecodePicture     *vp = &pp->vp;

    FFVkBuffer *slice_context_buf;
    ProresVkSliceContext *slice_context;
    int i, err;

#ifdef CONFIG_RENDERDOC
    av_vk_start_capture(ctx->s.device);

    av_log(avctx, AV_LOG_DEBUG, "Is capturing frame: %d\n", av_vk_is_capturing(ctx->s.device));
#endif

    /* Host map the input slices data if supported */
    if (ctx->s.extensions & FF_VK_EXT_EXTERNAL_HOST_MEMORY)
        ff_vk_host_map_buffer(&ctx->s, &vp->slices_buf, buffer_ref->data,
                              buffer_ref,
                              VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                              VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);

    /* Allocate slice offsets buffer */
    err = ff_vk_get_pooled_buffer(&ctx->s, &pv->slice_offset_pool,
                                  &pp->slice_offset_buf,
                                  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                  VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                  NULL, (pr->slice_count + 1) * sizeof(uint32_t),
                                  VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);

    /* Allocate slice context buffer */
    err = ff_vk_get_pooled_buffer(&ctx->s, &pv->slice_context_pool,
                                  &pp->slice_context_buf,
                                  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                  VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                  NULL, pr->slice_count * sizeof(ProresVkSliceContext),
                                  VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
    if (err < 0)
        return err;

    /* Prepare frame to be used */
    err = ff_vk_decode_prepare_frame_sdr(dec, pr->frame, vp, 1,
                                         FF_VK_REP_NATIVE, 0);
    if (err < 0)
        return err;

    slice_context_buf = (FFVkBuffer *)pp->slice_context_buf->data;
    slice_context     = (ProresVkSliceContext *)slice_context_buf->mapped_mem;
    slice_context_buf->stage = VK_PIPELINE_STAGE_2_HOST_BIT;

    for (i = 0; i < pr->slice_count; ++i) {
        slice_context[i] = (ProresVkSliceContext) {
            .mb_x     = pr->slices[i].mb_x,
            .mb_y     = pr->slices[i].mb_y,
            .mb_count = pr->slices[i].mb_count,
        };
    }

    return 0;
}

static int vk_prores_decode_slice(AVCodecContext *avctx,
                                  const uint8_t  *data,
                                  uint32_t        size)
{
    ProresContext             *pr = avctx->priv_data;
    ProresVulkanDecodePicture *pp = pr->hwaccel_picture_private;
    FFVulkanDecodePicture     *vp = &pp->vp;

    FFVkBuffer *slice_offset = (FFVkBuffer *)pp->slice_offset_buf->data;
    FFVkBuffer *slices_buf   = vp->slices_buf ? (FFVkBuffer *)vp->slices_buf->data : NULL;

    /* Skip picture header */
    if (slices_buf && slices_buf->host_ref && !pp->slice_num)
        pp->bitstream_start = data - slices_buf->mapped_mem;

    AV_WN32(slice_offset->mapped_mem + (pp->slice_num + 0) * sizeof(uint32_t),
            pp->bitstream_size);
    AV_WN32(slice_offset->mapped_mem + (pp->slice_num + 1) * sizeof(uint32_t),
            pp->bitstream_size += size);

    if (!slices_buf || !slices_buf->host_ref) {
        int err = ff_vk_decode_add_slice(avctx, vp, data, size, 0,
                                         &pp->slice_num, NULL);
        if (err < 0)
            return err;
    } else {
        pp->slice_num++;
    }

    return 0;
}

static int vk_prores_end_frame(AVCodecContext *avctx)
{
    ProresContext             *pr = avctx->priv_data;
    FFVulkanDecodeContext    *dec = avctx->internal->hwaccel_priv_data;
    FFVulkanDecodeShared     *ctx = dec->shared_ctx;
    FFVulkanFunctions         *vk = &ctx->s.vkfn;
    ProresVulkanDecodeContext *pv = ctx->sd_ctx;
    ProresVulkanDecodePicture *pp = pr->hwaccel_picture_private;
    FFVulkanDecodePicture     *vp = &pp->vp;

    AVVkFrame *vkf;
    ProresVkParameters pd;
    FFVkBuffer *slice_context;
    VkImageMemoryBarrier2 img_bar[AV_NUM_DATA_POINTERS];
    VkBufferMemoryBarrier2 buf_bar[2];
    int nb_img_bar = 0, nb_buf_bar = 0, i, err;

    slice_context = (FFVkBuffer *)pp->slice_context_buf->data;

    pd = (ProresVkParameters) {
        .slice_data      = ((FFVkBuffer *)vp->slices_buf->data)->address,
        .bitstream_start = pp->bitstream_start,
        .bitstream_size  = pp->bitstream_size,

        .slice_width     = pr->slice_count / pr->mb_height,
        .slice_height    = pr->mb_height,
        .mb_width        = pr->mb_width,
        .mb_height       = pr->mb_height,
        .alpha_info      = pr->alpha_info,
    };

    FFVkExecContext *exec = ff_vk_exec_get(&ctx->s, &ctx->exec_pool);
    RET(ff_vk_exec_start(&ctx->s, exec));

    /* Prepare deps */
    RET(ff_vk_exec_add_dep_frame(&ctx->s, exec, pr->frame,
                                 VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                                 VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT));

    RET(ff_vk_exec_mirror_sem_value(&ctx->s, exec, &vp->sem, &vp->sem_value,
                                    pr->frame));

    RET(ff_vk_exec_add_dep_buf(&ctx->s, exec, &vp->slices_buf,        1, 1));
    RET(ff_vk_exec_add_dep_buf(&ctx->s, exec, &pp->slice_offset_buf,  1, 1));
    RET(ff_vk_exec_add_dep_buf(&ctx->s, exec, &pp->slice_context_buf, 1, 1));

    /* Input frame barrier */
    nb_img_bar = 0;
    ff_vk_frame_barrier(&ctx->s, exec, pr->frame, img_bar, &nb_img_bar,
                        VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                        VK_PIPELINE_STAGE_2_CLEAR_BIT,
                        VK_ACCESS_TRANSFER_WRITE_BIT,
                        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        VK_QUEUE_FAMILY_IGNORED);

    vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
        .sType                    = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
        .pImageMemoryBarriers     = img_bar,
        .imageMemoryBarrierCount  = nb_img_bar,
    });

    /* Clear input frame as intermediary entropy decoding results will be written to it */
    vkf = (AVVkFrame *)pr->frame->data[0];
    for (i = 0; i < ff_vk_count_images(vkf); ++i) {
        vk->CmdClearColorImage(exec->buf, vkf->img[i], VK_IMAGE_LAYOUT_GENERAL,
            &((VkClearColorValue) { 0 }),
            1, &((VkImageSubresourceRange) {
                .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
                .levelCount = 1,
                .layerCount = 1,
            }
        ));
    }

    /* Entropy decode */
    ff_vk_shader_update_desc_buffer(&ctx->s, exec, &pv->vld,
                                    0, 0, 0,
                                    (FFVkBuffer *)pp->slice_offset_buf->data,
                                    0, (pp->slice_num + 1) * sizeof(uint32_t),
                                    VK_FORMAT_UNDEFINED);
    ff_vk_shader_update_desc_buffer(&ctx->s, exec, &pv->vld,
                                    0, 1, 0,
                                    (FFVkBuffer *)pp->slice_context_buf->data,
                                    0, pp->slice_num * sizeof(ProresVkSliceContext),
                                    VK_FORMAT_UNDEFINED);
    ff_vk_shader_update_img_array(&ctx->s, exec, &pv->vld,
                                  pr->frame, vp->view.out,
                                  0, 2,
                                  VK_IMAGE_LAYOUT_GENERAL,
                                  VK_NULL_HANDLE);

    ff_vk_exec_bind_shader(&ctx->s, exec, &pv->vld);

    ff_vk_shader_update_push_const(&ctx->s, exec, &pv->vld,
                                   VK_SHADER_STAGE_COMPUTE_BIT,
                                   0, sizeof(pd), &pd);

    /* Input frame barrier after clear */
    nb_img_bar = 0;
    ff_vk_frame_barrier(&ctx->s, exec, pr->frame, img_bar, &nb_img_bar,
                        VK_PIPELINE_STAGE_2_CLEAR_BIT,
                        VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                        VK_ACCESS_SHADER_WRITE_BIT,
                        VK_IMAGE_LAYOUT_GENERAL,
                        VK_QUEUE_FAMILY_IGNORED);

    /* Slice context barrier */
    nb_buf_bar = 0;
    buf_bar[nb_buf_bar++] = (VkBufferMemoryBarrier2) {
        .sType               = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2,
        .srcStageMask        = slice_context->stage,
        .dstStageMask        = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
        .srcAccessMask       = slice_context->access,
        .dstAccessMask       = VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                               VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
        .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .buffer              = slice_context->buf,
        .offset              = 0,
        .size                = pr->slice_count * sizeof(ProresVkSliceContext),
    };
    slice_context->stage  = buf_bar[0].dstStageMask;
    slice_context->access = buf_bar[0].dstAccessMask;

    vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
        .sType                    = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
        .pBufferMemoryBarriers    = buf_bar,
        .bufferMemoryBarrierCount = nb_buf_bar,
        .pImageMemoryBarriers     = img_bar,
        .imageMemoryBarrierCount  = nb_img_bar,
    });

    vk->CmdDispatch(exec->buf, AV_CEIL_RSHIFT(pr->slice_count / pr->mb_height, 3), AV_CEIL_RSHIFT(pr->mb_height, 3), 1);

    RET(ff_vk_exec_submit(&ctx->s, exec));

#ifdef CONFIG_RENDERDOC
    av_vk_end_capture(ctx->s.device);
#endif

fail:
    return err;
}

static int add_shared_code(FFVulkanShader *shd)
{
    GLSLC(0, struct SliceContext {                                 );
    GLSLC(1,     uint mb_x;                                        );
    GLSLC(1,     uint mb_y;                                        );
    GLSLC(1,     uint mb_count;                                    );
    GLSLC(1,     uint qscale;                                      );
    GLSLC(0, };                                                    );

    return 0;
}

static int add_push_data(FFVulkanShader *shd)
{
    GLSLC(0, layout(push_constant, scalar) uniform pushConstants { );
    GLSLC(1,    u8buf slice_data;                                  );
    GLSLC(1,    uint  bitstream_start;                             );
    GLSLC(1,    uint  bitstream_size;                              );
    GLSLC(0,                                                       );
    GLSLC(1,    uint  slice_width;                                 );
    GLSLC(1,    uint  slice_height;                                );
    GLSLC(1,    uint  mb_width;                                    );
    GLSLC(1,    uint  mb_height;                                   );
    GLSLC(1,    uint  alpha_info;                                  );
    GLSLC(0, };                                                    );

    return ff_vk_shader_add_push_const(shd, 0, sizeof(ProresVkParameters),
                                       VK_SHADER_STAGE_COMPUTE_BIT);
}

static int init_vld_shader(AVCodecContext *avctx, FFVulkanContext *s,
                           FFVkExecPool *pool, FFVkSPIRVCompiler *spv,
                           AVHWFramesContext *out_frames_ctx,
                           FFVulkanShader *shd)
{
    FFVulkanDescriptorSetBinding *desc_set;
    uint8_t *spv_data;
    size_t spv_len;
    void *spv_opaque = NULL;
    int max_num_slices, err;

    max_num_slices = (avctx->coded_width >> 4) * (avctx->coded_height >> 4);

    RET(ff_vk_shader_init(s, shd, "prores_dec_vld",
                          VK_SHADER_STAGE_COMPUTE_BIT,
                          (const char *[]) { "GL_EXT_buffer_reference",
                                             "GL_EXT_buffer_reference2" }, 2,
                          8, 8, 3,
                          0));

    /* Common code */
    RET(add_shared_code(shd));
    GLSLD(ff_source_common_comp);

    /* Push constants layout */
    RET(add_push_data(shd));

    desc_set = (FFVulkanDescriptorSetBinding []) {
        {
            .name        = "slice_offsets_buf",
            .type        = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            .stages      = VK_SHADER_STAGE_COMPUTE_BIT,
            .mem_quali   = "readonly",
            .buf_content = "uint32_t slice_offsets",
            .buf_elems   = max_num_slices + 1,
        },
        {
            .name        = "slice_context_buf",
            .type        = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            .stages      = VK_SHADER_STAGE_COMPUTE_BIT,
            .buf_content = "SliceContext slice_context",
            .buf_elems   = max_num_slices + 1,
        },
        {
            .name       = "dst",
            .type       = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
            .dimensions = 2,
            .mem_layout = ff_vk_shader_rep_fmt(out_frames_ctx->sw_format,
                                               FF_VK_REP_NATIVE),
            .mem_quali  = "writeonly",
            .elems      = av_pix_fmt_count_planes(out_frames_ctx->sw_format),
            .stages     = VK_SHADER_STAGE_COMPUTE_BIT,
        },
    };
    RET(ff_vk_shader_add_descriptor_set(s, shd, desc_set, 3, 1, 0));

    /* Main code */
    GLSLD(ff_source_prores_vld_comp);

    RET(spv->compile_shader(s, spv, shd, &spv_data, &spv_len, "main",
                            &spv_opaque));
    RET(ff_vk_shader_link(s, shd, spv_data, spv_len, "main"));

    RET(ff_vk_shader_register_exec(s, pool, shd));

fail:
    if (spv_opaque)
        spv->free_shader(spv, &spv_opaque);

    return 0;
}

static void vk_decode_prores_uninit(FFVulkanDecodeShared *ctx)
{
    ProresVulkanDecodeContext *pv = ctx->sd_ctx;

    ff_vk_shader_free(&ctx->s, &pv->vld);

    av_buffer_pool_uninit(&pv->slice_offset_pool);
    av_buffer_pool_uninit(&pv->slice_context_pool);

    av_freep(&pv);
}

static int vk_decode_prores_init(AVCodecContext *avctx)
{
    FFVulkanDecodeContext *dec = avctx->internal->hwaccel_priv_data;
    FFVulkanDecodeShared  *ctx = NULL;

    ProresVulkanDecodeContext *pv;
    FFVkSPIRVCompiler *spv;
    int err;

    switch (avctx->profile) {
        case AV_PROFILE_PRORES_4444:
        case AV_PROFILE_PRORES_XQ:
            break;
        default:
            av_log(avctx, AV_LOG_ERROR, "Profile is not supported!\n");
            return AVERROR(ENOTSUP);
    }

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

    ctx->sd_ctx_free = vk_decode_prores_uninit;

    RET(init_vld_shader(avctx, &ctx->s, &ctx->exec_pool, spv,
                        (AVHWFramesContext *)avctx->hw_frames_ctx->data,
                        &pv->vld));

    av_log(avctx, AV_LOG_DEBUG, "Initialized Vulkan-Prores decoder\n");

    err = 0;

fail:
    spv->uninit(&spv);

    return err;
}

static void vk_prores_free_frame_priv(AVRefStructOpaque _hwctx, void *data)
{
    AVHWDeviceContext    *dev_ctx = _hwctx.nc;
    ProresVulkanDecodePicture *pp = data;
    FFVulkanDecodePicture     *vp = &pp->vp;

    ff_vk_decode_free_frame(dev_ctx, vp);

    av_buffer_unref(&vp->slices_buf);
    av_buffer_unref(&pp->slice_offset_buf);
    av_buffer_unref(&pp->slice_context_buf);
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
