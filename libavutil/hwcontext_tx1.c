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
#include <fcntl.h>
#include <unistd.h>

#include "config.h"
#include "pixdesc.h"
#include "pixfmt.h"
#include "imgutils.h"
#include "internal.h"
#include "time.h"

#include "hwcontext.h"
#include "hwcontext_internal.h"

#include "nvhost_ioctl.h"
#include "nvmap_ioctl.h"
#include "tx1_host1x.h"
#include "clb0b6.h"
#include "vic_drv.h"

#include "hwcontext_tx1.h"

#ifndef __SWITCH__
int g_nvmap_fd = 0, g_nvhost_fd = 0;
#endif

/* 3x3 color conversion matrix plus 1x3 color offsets */
static float mat_rgb_to_ycbcr_bt601lim[3][4] = {
    { 0.25678825,  0.5041294,   0.09790588, 0.0627451 },
    {-0.1482229,  -0.2909928,   0.4392157,  0.50196075},
    { 0.4392157,  -0.3677883,  -0.07142737, 0.50196075},
};

static float mat_rgb_to_ycbcr_bt601full[3][4] = {
    { 0.29899999,  0.587,       0.114,      0.0       },
    {-0.16873589, -0.3312641,   0.5,        0.50196075},
    { 0.5,        -0.41868758, -0.08131241, 0.50196075},
};

static float mat_rgb_to_ycbcr_bt709lim[3][4] = {
    { 0.18258588,  0.6142306,   0.06200706, 0.0627451 },
    {-0.10064201, -0.33856615,  0.43920815, 0.50196075},
    { 0.4392148,  -0.39894137, -0.04027344, 0.50196075},
};

static float mat_rgb_to_ycbcr_bt709full[3][4] = {
    { 0.21259999,  0.7152,      0.0722,     0.0       },
    {-0.11457014, -0.38542128,  0.49999142, 0.50196075},
    { 0.49999899, -0.45415199, -0.04584699, 0.50196075},
};

static float mat_rgb_to_ycbcr_bt2020lim[3][4] = {
    { 0.22561294,  0.58228236,  0.05092824, 0.0627451 },
    {-0.12265543, -0.31656027,  0.4392157,  0.50196075},
    { 0.4392157,  -0.4038902,  -0.0353255,  0.50196075},
};

/* Note: not dumped from official code */
static float mat_rgb_to_ycbcr_bt2020full[3][4] = {
    { 0.2627,      0.678,       0.0593,     0.0       },
    {-0.13963006, -0.36036994,  0.5,        0.50196075},
    { 0.5       , -0.4597857,  -0.0402143,  0.50196075},
};

static float mat_ycbcr_bt601lim_to_rgb[3][4] = {
    {1.1643835,  0.0,         1.5960268,  -0.8742022},
    {1.1643835, -0.3917623,  -0.81296766,  0.5316678},
    {1.1643835,  2.0172322,   0.0,        -1.0856308},
};

static float mat_ycbcr_bt601full_to_rgb[3][4] = {
    {1.0,        0.0,         1.402,      -0.703749 },
    {1.0,       -0.3441363,  -0.7141363,   0.5312113},
    {1.0,        1.772,       0.0,        -0.8894745},
};

static float mat_ycbcr_bt709lim_to_rgb[3][4] = {
    {1.1643835,  0.0,         1.7927446,  -0.9729469},
    {1.1643835, -0.21325228, -0.5329104,   0.3014850},
    {1.1643835,  2.112438,    0.0,        -1.1334205},
};

static float mat_ycbcr_bt709full_to_rgb[3][4] = {
    {1.0,        0.0,         1.5748031,  -0.7904894},
    {1.0,       -0.18732749, -0.46812522,  0.3290116},
    {1.0,        1.8556318,   0.0,        -0.9314544},
};

static float mat_ycbcr_bt2020lim_to_rgb[3][4] = {
    {1.1643835,  0.0,         1.6786741,  -0.9156879},
    {1.1643835, -0.1873261,  -0.6504243,   0.3474585},
    {1.1643835,  2.1417723,   0.0,        -1.1481451},
};

/* Note: not dumped from official code */
static float mat_ycbcr_bt2020full_to_rgb[3][4] = {
    {1.0,        0.0,         1.4746,     -0.7401914},
    {1.0,       -0.16455313, -0.57135313,  0.3693961},
    {1.0,        1.8814,      0.0,        -0.9443890},
};

/* Colorspace rotation matrices */
static float mat_colorgamut_bt709_to_2020[3][4] = {
    { 0.6274039,   0.32928303, 0.04331307, 0.0},
    { 0.06909729,  0.9195404,  0.01136232, 0.0},
    { 0.01639144,  0.08801331, 0.89559525, 0.0},
};

static float mat_colorgamut_bt2020_to_bt709[3][4] = {
    { 1.660491,   -0.5876411, -0.07284986, 0.0},
    {-0.12455047,  1.1328999, -0.00834942, 0.0},
    {-0.01815076, -0.1005789,  1.1187297,  0.0},
};

static const enum AVPixelFormat supported_sw_formats[] = {
    AV_PIX_FMT_GRAY8,
    AV_PIX_FMT_NV12,
    AV_PIX_FMT_P010,
    AV_PIX_FMT_YUV420P,
};

int ff_tx1_map_vic_pic_fmt(enum AVPixelFormat fmt) {
    switch (fmt) {
        case AV_PIX_FMT_GRAY8:
            return NVB0B6_T_L8;
        case AV_PIX_FMT_NV12:
            return NVB0B6_T_Y8___U8V8_N420;
        case AV_PIX_FMT_YUV420P:
            return NVB0B6_T_Y8___U8___V8_N420;
        case AV_PIX_FMT_RGB565:
            return NVB0B6_T_R5G6B5;
        case AV_PIX_FMT_RGB32:
            return NVB0B6_T_A8R8G8B8;
        case AV_PIX_FMT_BGR32:
            return NVB0B6_T_A8B8G8R8;
        case AV_PIX_FMT_RGB32_1:
            return NVB0B6_T_R8G8B8A8;
        case AV_PIX_FMT_BGR32_1:
            return NVB0B6_T_B8G8R8A8;
        case AV_PIX_FMT_0RGB32:
            return NVB0B6_T_X8R8G8B8;
        case AV_PIX_FMT_0BGR32:
            return NVB0B6_T_X8B8G8R8;
        default:
            return -1;
    }
}

static uint32_t tx1_surface_get_width_align(enum AVPixelFormat fmt, const AVComponentDescriptor *comp) {
    int step = comp->step;

    if (fmt != AV_PIX_FMT_TX1)
        return 256 / step; /* Pitch linear surfaces must be aligned to 256B for VIC */

    /*
     * GOBs are 64B wide.
     * In addition, we use a 32Bx8 cache width in VIC for block linear surfaces.
     */
    return 64 / step;
}

static uint32_t tx1_surface_get_height_align(enum AVPixelFormat fmt, const AVComponentDescriptor *comp) {
    /* Height alignment is in terms of lines, not bytes, therefore we don't divide by the sample step */
    if (fmt != AV_PIX_FMT_TX1)
        return 4; /* We use 64Bx4 cache width in VIC for pitch linear surfaces */

    /*
     * GOBs are 8B high, and we use a GOB height of 2.
     * In addition, we use a 32Bx8 cache width in VIC for block linear surfaces.
     * We double this requirement to make sure it is respected for the subsampled chroma plane.
     */
    return 32;
}

static int tx1_channel_set_freq(AVTX1Channel *channel, uint32_t freq) {
    int err;
#ifndef __SWITCH__
    err = ff_tx1_channel_set_clock_rate(channel, channel->module_id, freq);
    if (err < 0)
        return err;

    err = ff_tx1_channel_get_clock_rate(channel, channel->module_id, &channel->clock);
    if (err < 0)
        return err;
#else
    err = AVERROR(mmuRequestSetAndWait(&channel->mmu_request, freq, -1));
    if (err < 0)
        return err;

    err = AVERROR(mmuRequestGet(&channel->mmu_request, &channel->clock));
    if (err < 0)
        return err;
#endif
    return 0;
}

static void tx1_device_uninit(AVHWDeviceContext *ctx) {
    AVTX1DeviceContext *hwctx = ctx->hwctx;

    av_log(ctx, AV_LOG_DEBUG, "Deinitializing TX1 device\n");

    ff_tx1_cmdbuf_deinit(&hwctx->vic_cmdbuf);

    ff_tx1_map_destroy(&hwctx->vic_map);

    if (hwctx->has_nvdec)
        ff_tx1_channel_close(&hwctx->nvdec_channel);
    if (hwctx->has_nvjpg)
        ff_tx1_channel_close(&hwctx->nvjpg_channel);
    ff_tx1_channel_close(&hwctx->vic_channel);

#ifndef __SWITCH__
    if (g_nvmap_fd > 0)
        close(g_nvmap_fd);

    if (g_nvhost_fd > 0)
        close(g_nvhost_fd);
#else
    nvFenceExit();
    nvMapExit();
    nvExit();
    if (hwctx->has_nvdec)
        mmuRequestFinalize(&hwctx->nvdec_channel.mmu_request);
    if (hwctx->has_nvjpg)
        mmuRequestFinalize(&hwctx->nvjpg_channel.mmu_request);
    mmuExit();
#endif
}

static int tx1_device_init(AVHWDeviceContext *ctx) {
    AVTX1DeviceContext *hwctx = ctx->hwctx;

    uint32_t vic_map_size;
    int err;

    av_log(ctx, AV_LOG_DEBUG, "Initializing TX1 device\n");

    err = ff_tx1_channel_open(&hwctx->nvdec_channel, "/dev/nvhost-nvdec");
    hwctx->has_nvdec = err == 0;

    err = ff_tx1_channel_open(&hwctx->nvjpg_channel, "/dev/nvhost-nvjpg");
    hwctx->has_nvjpg = err == 0;

    err = ff_tx1_channel_open(&hwctx->vic_channel, "/dev/nvhost-vic");
    if (err < 0)
        goto fail;

    /* Note: Official code only sets this for the nvdec channel */
    if (hwctx->has_nvdec) {
        err = ff_tx1_channel_set_submit_timeout(&hwctx->nvdec_channel, 1000);
        if (err < 0)
            goto fail;
    }

    if (hwctx->has_nvjpg) {
        err = ff_tx1_channel_set_submit_timeout(&hwctx->nvjpg_channel, 1000);
        if (err < 0)
            goto fail;
    }

#ifdef __SWITCH__
    hwctx->vic_map.owner = hwctx->vic_channel.channel.fd;
#endif

    hwctx->vic_setup_off  = 0;
    hwctx->vic_cmdbuf_off = FFALIGN(hwctx->vic_setup_off  + sizeof(VicConfigStruct),
                                    FF_TX1_MAP_ALIGN);
    hwctx->vic_filter_off = FFALIGN(hwctx->vic_cmdbuf_off + FF_TX1_MAP_ALIGN,
                                    FF_TX1_MAP_ALIGN);
    vic_map_size          = FFALIGN(hwctx->vic_filter_off + 0x3000,
                                    0x1000);

    hwctx->vic_max_cmdbuf_size = hwctx->vic_filter_off - hwctx->vic_cmdbuf_off;

    err = ff_tx1_map_create(&hwctx->vic_map, vic_map_size, 0x100, NVMAP_CACHE_OP_INV);
    if (err < 0)
        goto fail;

    err = ff_tx1_cmdbuf_init(&hwctx->vic_cmdbuf);
    if (err < 0)
        goto fail;

    err = ff_tx1_cmdbuf_add_memory(&hwctx->vic_cmdbuf, &hwctx->vic_map,
                                   hwctx->vic_cmdbuf_off, hwctx->vic_max_cmdbuf_size);
    if (err < 0)
        goto fail;

#ifndef __SWITCH__
    hwctx->nvdec_channel.module_id = 0x75;
    hwctx->nvjpg_channel.module_id = 0x76;
#else
    /*
     * The NVHOST_IOCTL_CHANNEL_SET_CLK_RATE ioctl also exists on HOS but the clock rate
     * will be reset when the console goes to sleep.
     */

    if (hwctx->has_nvdec) {
        err = AVERROR(mmuRequestInitialize(&hwctx->nvdec_channel.mmu_request, (MmuModuleId)5, 8, false));
        if (err < 0)
            goto fail;
    }

    if (hwctx->has_nvjpg) {
        err = AVERROR(mmuRequestInitialize(&hwctx->nvjpg_channel.mmu_request, MmuModuleId_Nvjpg, 8, false));
        if (err < 0)
            goto fail;
    }
#endif

    return 0;

fail:
    tx1_device_uninit(ctx);
    return err;
}

static int tx1_device_create(AVHWDeviceContext *ctx, const char *device,
                             AVDictionary *opts, int flags)
{
    int err;

    av_log(ctx, AV_LOG_DEBUG, "Creating TX1 device\n");

#ifndef __SWITCH__
    if (!g_nvmap_fd) {
        err = open("/dev/nvmap", O_RDWR | O_SYNC);
        if (err < 0)
            return AVERROR(errno);
        g_nvmap_fd = err;
    }

    if (!g_nvhost_fd) {
        err = open("/dev/nvhost-ctrl", O_RDWR | O_SYNC);
        if (err < 0)
            return AVERROR(errno);
        g_nvhost_fd = err;
    }
#else
    err = AVERROR(nvInitialize());
    if (R_FAILED(err))
        return err;

    err = AVERROR(nvMapInit());
    if (R_FAILED(err))
        return err;

    err = AVERROR(nvFenceInit());
    if (R_FAILED(err))
        return err;

    err = AVERROR(mmuInitialize());
    if (R_FAILED(err))
        return err;
#endif

    return 0;
}

static int tx1_frames_get_constraints(AVHWDeviceContext *ctx, const void *hwconfig,
                                      AVHWFramesConstraints *constraints)
{
    av_log(ctx, AV_LOG_DEBUG, "Getting frame constraints for TX1 device\n");

    constraints->valid_sw_formats = av_malloc_array(FF_ARRAY_ELEMS(supported_sw_formats) + 1,
                                                    sizeof(*constraints->valid_sw_formats));
    if (!constraints->valid_sw_formats)
        return AVERROR(ENOMEM);

    for (int i = 0; i < FF_ARRAY_ELEMS(supported_sw_formats); ++i)
        constraints->valid_sw_formats[i] = supported_sw_formats[i];
    constraints->valid_sw_formats[FF_ARRAY_ELEMS(supported_sw_formats)] = AV_PIX_FMT_NONE;

    constraints->valid_hw_formats = av_malloc_array(2, sizeof(*constraints->valid_hw_formats));
    if (!constraints->valid_hw_formats)
        return AVERROR(ENOMEM);

    constraints->valid_hw_formats[0] = AV_PIX_FMT_TX1;
    constraints->valid_hw_formats[1] = AV_PIX_FMT_NONE;

    return 0;
}

static void tx1_buffer_free(void *opaque, uint8_t *data) {
    AVTX1Map *map = (AVTX1Map *)data;

    av_log(opaque, AV_LOG_DEBUG, "Freeing surface from TX1 device\n");

    if (!data)
        return;

    ff_tx1_map_destroy(map);

    av_freep(&map);
}

static AVBufferRef *tx1_pool_alloc(void *opaque, size_t size) {
    AVHWFramesContext    *ctx = opaque;
#ifdef __SWITCH__
    AVTX1DeviceContext *hwctx = ctx->device_ctx->hwctx;
#endif

    AVBufferRef *buffer;
    AVTX1Map    *map;
    int err;

    av_log(ctx, AV_LOG_DEBUG, "Creating surface from TX1 device\n");

    map = av_mallocz(sizeof(*map));
    if (!map)
        return NULL;

#ifdef __SWITCH__
    map->owner = hwctx->nvdec_channel.channel.fd;
#endif

    err = ff_tx1_map_create(map, size, 0x100, NVMAP_CACHE_OP_INV);
    if (err < 0)
        goto fail;

    buffer = av_buffer_create((uint8_t *)map, sizeof(map), tx1_buffer_free, ctx, 0);
    if (!buffer)
        goto fail;

    return buffer;

fail:
    av_log(ctx, AV_LOG_ERROR, "Failed to create buffer\n");
    ff_tx1_map_destroy(map);
    av_freep(map);
    return NULL;
}

static int tx1_frames_init(AVHWFramesContext *ctx) {
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(ctx->sw_format);

    uint32_t width_aligned, height_aligned, size;

    av_log(ctx, AV_LOG_DEBUG, "Initializing frame pool for the TX1 device\n");

    if (!ctx->pool) {
        width_aligned  = FFALIGN(ctx->width,  tx1_surface_get_width_align (ctx->format, &desc->comp[0]));
        height_aligned = FFALIGN(ctx->height, tx1_surface_get_height_align(ctx->format, &desc->comp[0]));

        size = av_image_get_buffer_size(ctx->sw_format, width_aligned, height_aligned,
                                        tx1_surface_get_width_align(ctx->format, &desc->comp[0]));

        ctx->internal->pool_internal = av_buffer_pool_init2(size, ctx, tx1_pool_alloc, NULL);
        if (!ctx->internal->pool_internal)
            return AVERROR(ENOMEM);
    }

    return 0;
}

static void tx1_frames_uninit(AVHWFramesContext *ctx) {
    av_log(ctx, AV_LOG_DEBUG, "Deinitializing frame pool for the TX1 device\n");
}

static int tx1_get_buffer(AVHWFramesContext *ctx, AVFrame *frame) {
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(ctx->sw_format);

    AVTX1Map *map;
    uint32_t width_aligned, height_aligned;
    int err;

    av_log(ctx, AV_LOG_DEBUG, "Getting frame buffer for TX1 device\n");

    frame->buf[0] = av_buffer_pool_get(ctx->pool);
    if (!frame->buf[0])
        return AVERROR(ENOMEM);

    map = ff_tx1_frame_get_fbuf_map(frame);

    width_aligned  = FFALIGN(ctx->width,  tx1_surface_get_width_align (ctx->format, &desc->comp[0]));
    height_aligned = FFALIGN(ctx->height, tx1_surface_get_height_align(ctx->format, &desc->comp[0]));

    err = av_image_fill_arrays(frame->data, frame->linesize, ff_tx1_map_get_addr(map),
                               ctx->sw_format, width_aligned, height_aligned,
                               tx1_surface_get_width_align(ctx->format, &desc->comp[0]));
    if (err < 0)
        return err;

    frame->format = AV_PIX_FMT_TX1;
    frame->width  = ctx->width;
    frame->height = ctx->height;

    return 0;
}

/*
 * Possible frequencies on Icosa and Mariko+, in MHz
 * (see tegra210-core-dvfs.c and tegra210b01-core-dvfs.c in l4t kernel sources, respectively):
 * for NVDEC:
 *   268.8, 384.0, 448.0, 486.4, 550.4, 576.0, 614.4, 652.8, 678.4, 691.2, 716.8
 *   460.8, 499.2, 556.8, 633.6, 652.8, 710.4, 748.8, 787.2, 825.6, 844.8, 883.2, 902.4, 921.6, 940.8, 960.0, 979.2
 * for NVJPG:
 *   192.0, 307.2, 345.6, 409.6, 486.4, 524.8, 550.4, 576.0, 588.8, 614.4, 627.2
 *   422.4, 441.6, 499.2, 518.4, 537.6, 556.8, 576.0, 595.2, 614.4, 633.6, 652.8
 */

int ff_tx1_dfs_init(AVHWDeviceContext *ctx, AVTX1Channel *channel, int width, int height, double framerate_hz) {
    AVTX1DeviceContext *hwctx = ctx->hwctx;

    uint32_t max_freq, lowcorner;
    int num_mbs, err;

    hwctx->dfs_num_samples        = 20;
    hwctx->dfs_decode_ema_damping = 0.1;
    hwctx->dfs_sampling_start_ts  = av_gettime_relative();

    /*
     * Initialize low-corner frequency (reproduces official code)
     * Framerate might be unavailable (or variable), but this is official logic
     */
    num_mbs = width / 16 * height / 16;
    if (num_mbs <= 3600)
        lowcorner = 100000000;  /* 480p */
    else if (num_mbs <= 8160)
        lowcorner = 180000000;  /* 720p */
    else if (num_mbs <= 32400)
        lowcorner = 345000000;  /* 1080p */
    else
        lowcorner = 576000000;  /* 4k */

    if (framerate_hz >= 0.1 && isfinite(framerate_hz))
        lowcorner = FFMIN(lowcorner, lowcorner * framerate_hz / 30.0);

    hwctx->dfs_lowcorner = lowcorner;

    hwctx->dfs_bitrate_samples = av_malloc_array(hwctx->dfs_num_samples, sizeof(*hwctx->dfs_bitrate_samples));
    if (!hwctx->dfs_bitrate_samples)
        return AVERROR(ENOMEM);

    av_log(ctx, AV_LOG_DEBUG, "DFS: Initializing lowcorner to %d Hz, using %u samples\n",
           hwctx->dfs_lowcorner, hwctx->dfs_num_samples);

    /*
     * Initialize channel to the max possible frequency (the kernel driver will clamp to an allowed value)
     * Note: Official code passes INT_MAX kHz then multiplies by 1000 (to Hz) and converts to u32,
     * resulting in this value.
     */
    max_freq = (UINT64_C(1)<<32) - 1000 & UINT32_MAX;

    err = tx1_channel_set_freq(channel, max_freq);
    if (err < 0)
        return err;

    return 0;
}

int ff_tx1_dfs_update(AVHWDeviceContext *ctx, AVTX1Channel *channel, int bitstream_len, int decode_cycles) {
    AVTX1DeviceContext *hwctx = ctx->hwctx;

    double avg;
    uint32_t sum, clock;
    int64_t time;
    int i, err;

    /*
     * Official software implements DFS using a flat average of the decoder pool occupancy.
     * We instead use the decode cycles as reported by NVDEC microcode, and the "bitrate"
     * (bitstream bits fed to the hardware in a given clock time interval, NOT video time),
     * to calculate a suitable frequency, and multiply it by 1.2 for good measure:
     *   Freq = decode_cycles_per_bit * bits_per_second * 1.2
     */

    /* Convert to bits */
    bitstream_len *= 8;

    /* Exponential moving average of decode cycles per bitstream bit */
    hwctx->dfs_decode_cycles_ema = hwctx->dfs_decode_ema_damping * (double)decode_cycles/bitstream_len +
        (1.0 - hwctx->dfs_decode_ema_damping) * hwctx->dfs_decode_cycles_ema;

    hwctx->dfs_cur_sample = (hwctx->dfs_cur_sample + 1) % hwctx->dfs_num_samples;
    hwctx->dfs_bitrate_samples[hwctx->dfs_cur_sample] = bitstream_len;

    /* Reclock if we collected enough samples */
    if (hwctx->dfs_cur_sample == 0) {
        /* Flat average of bitstream bits per time interval */
        for (sum = i = 0; i < hwctx->dfs_num_samples; ++i)
            sum += hwctx->dfs_bitrate_samples[i];

        time = av_gettime_relative();
        avg = sum * 1e6 / (time - hwctx->dfs_sampling_start_ts);

        clock = hwctx->dfs_decode_cycles_ema * avg * 1.2;
        clock = FFMAX(clock, hwctx->dfs_lowcorner);

        err = tx1_channel_set_freq(channel, clock);
        if (err < 0)
            return err;

        av_log(ctx, AV_LOG_DEBUG, "DFS: %.0f cycles/b (ema), %.0f b/s -> clock %u Hz (lowcorner %u Hz)\n",
               hwctx->dfs_decode_cycles_ema, avg, clock, hwctx->dfs_lowcorner);

        hwctx->dfs_sampling_start_ts = time;
    }

    return 0;
}

int ff_tx1_dfs_uninit(AVHWDeviceContext *ctx, AVTX1Channel *channel) {
    AVTX1DeviceContext *hwctx = ctx->hwctx;

    av_free(hwctx->dfs_bitrate_samples);

    return 0;
}

static int tx1_transfer_get_formats(AVHWFramesContext *ctx,
                                    enum AVHWFrameTransferDirection dir,
                                    enum AVPixelFormat **formats)
{
    enum AVPixelFormat *fmts;

    av_log(ctx, AV_LOG_DEBUG, "Getting transfer formats for TX1 device\n");

    fmts = av_malloc_array(2, sizeof(**formats));
    if (!fmts)
        return AVERROR(ENOMEM);

    fmts[0] = ctx->sw_format;
    fmts[1] = AV_PIX_FMT_NONE;

    *formats = fmts;
    return 0;
}

static int pack_to_fixed_point(float val, bool sign, int integer, int fractional) {
    return (int)(val * (1 << fractional) + 0.5f) &
        ((1 << sign + integer + fractional) - 1);
}

static void set_matrix_struct(VicMatrixStruct *dst, float src[3][4],
                              bool sign, int integer, int fractional)
{
    dst->matrix_enable  = 1;
    dst->matrix_r_shift = fractional - 8;
    dst->matrix_coeff00 = pack_to_fixed_point(src[0][0], sign, integer, fractional);
    dst->matrix_coeff10 = pack_to_fixed_point(src[1][0], sign, integer, fractional);
    dst->matrix_coeff20 = pack_to_fixed_point(src[2][0], sign, integer, fractional);
    dst->matrix_coeff01 = pack_to_fixed_point(src[0][1], sign, integer, fractional);
    dst->matrix_coeff11 = pack_to_fixed_point(src[1][1], sign, integer, fractional);
    dst->matrix_coeff21 = pack_to_fixed_point(src[2][1], sign, integer, fractional);
    dst->matrix_coeff02 = pack_to_fixed_point(src[0][2], sign, integer, fractional);
    dst->matrix_coeff12 = pack_to_fixed_point(src[1][2], sign, integer, fractional);
    dst->matrix_coeff22 = pack_to_fixed_point(src[2][2], sign, integer, fractional);
    dst->matrix_coeff03 = (int)(src[0][3] * 0x3ff00 + 0.5f);
    dst->matrix_coeff13 = (int)(src[1][3] * 0x3ff00 + 0.5f);
    dst->matrix_coeff23 = (int)(src[2][3] * 0x3ff00 + 0.5f);
}

static void tx1_vic_preprare_config(VicConfigStruct *config, AVFrame *dst, const AVFrame *src,
                                    enum AVPixelFormat fmt, bool is_10b_chroma)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(fmt);
    AVTX1Map *input_map = ff_tx1_frame_get_fbuf_map(src);

    /* Subsampled dimensions when emulating 10B chroma transfers, as input is always NV12 */
    int divider = (!is_10b_chroma ? 1 : 2);
    int src_width = src->width / divider, src_height = src->height / divider;
    int dst_width = dst->width / divider, dst_height = dst->height / divider;

    *config = (VicConfigStruct){
        .pipeConfig = {
            .DownsampleHoriz            = 1 << 2, /* U9.2 */
            .DownsampleVert             = 1 << 2, /* U9.2 */
        },
        .outputConfig = {
            .AlphaFillMode              = !is_10b_chroma ? NVB0B6_DXVAHD_ALPHA_FILL_MODE_OPAQUE :
                                                           NVB0B6_DXVAHD_ALPHA_FILL_MODE_SOURCE_STREAM,
            .BackgroundAlpha            = 0,
            .BackgroundR                = 0,
            .BackgroundG                = 0,
            .BackgroundB                = 0,
            .TargetRectLeft             = 0,
            .TargetRectRight            = dst_width  - 1,
            .TargetRectTop              = 0,
            .TargetRectBottom           = dst_height - 1,
        },
        .outputSurfaceConfig = {
            .OutPixelFormat             = ff_tx1_map_vic_pic_fmt(fmt),
            .OutSurfaceWidth            = dst_width  - 1,
            .OutSurfaceHeight           = dst_height - 1,
            .OutBlkKind                 = NVB0B6_BLK_KIND_PITCH,
            .OutLumaWidth               = (dst->linesize[0] / desc->comp[0].step) - 1,
            .OutLumaHeight              = dst_height - 1,
            .OutChromaWidth             = (desc->flags & AV_PIX_FMT_FLAG_RGB) ?
                                          -1 : (dst->linesize[1] / desc->comp[1].step) - 1,
            .OutChromaHeight            = (desc->flags & AV_PIX_FMT_FLAG_RGB) ?
                                          -1 : (dst->height >> desc->log2_chroma_h) - 1,
        },
        .slotStruct = {
            {
                .slotConfig = {
                    .SlotEnable         = 1,
                    .CurrentFieldEnable = 1,
                    .SoftClampLow       = 0,
                    .SoftClampHigh      = 1023,
                    .PlanarAlpha        = 1023,
                    .ConstantAlpha      = 1,
                    .SourceRectLeft     = 0,
                    .SourceRectRight    = (src_width  - 1) << 16, /* U14.16 (for subpixel positioning) */
                    .SourceRectTop      = 0,
                    .SourceRectBottom   = (src_height - 1) << 16,
                    .DestRectLeft       = 0,
                    .DestRectRight      = src_width  - 1,
                    .DestRectTop        = 0,
                    .DestRectBottom     = src_height - 1,
                },
                .slotSurfaceConfig = {
                    .SlotPixelFormat    = ff_tx1_map_vic_pic_fmt(fmt),
                    .SlotChromaLocHoriz = ((desc->flags & AV_PIX_FMT_FLAG_RGB)          ||
                                           src->chroma_location == AVCHROMA_LOC_TOPLEFT ||
                                           src->chroma_location == AVCHROMA_LOC_LEFT    ||
                                           src->chroma_location == AVCHROMA_LOC_BOTTOMLEFT) ? 0 : 1,
                    .SlotChromaLocVert  = ((desc->flags & AV_PIX_FMT_FLAG_RGB)          ||
                                           src->chroma_location == AVCHROMA_LOC_TOPLEFT ||
                                           src->chroma_location == AVCHROMA_LOC_TOP) ? 0 :
                                          (src->chroma_location == AVCHROMA_LOC_LEFT ||
                                           src->chroma_location == AVCHROMA_LOC_CENTER) ? 1 : 2,
                    .SlotBlkKind        = !input_map->is_linear ?
                                          NVB0B6_BLK_KIND_GENERIC_16Bx2 : NVB0B6_BLK_KIND_PITCH,
                    .SlotBlkHeight      = !input_map->is_linear ? 1 : 0, /* GOB height 2 */
                    .SlotCacheWidth     = !input_map->is_linear ? 1 : 2, /* 32Bx8 for block, 64Bx4 for pitch (as recommended by the TRM) */
                    .SlotSurfaceWidth   = src_width  - 1,
                    .SlotSurfaceHeight  = src_height - 1,
                    .SlotLumaWidth      = (src->linesize[0] / desc->comp[0].step) - 1,
                    .SlotLumaHeight     = src_height - 1,
                    .SlotChromaWidth    = (desc->flags & AV_PIX_FMT_FLAG_RGB) ?
                                          -1 : (src->linesize[1] / desc->comp[1].step) - 1,
                    .SlotChromaHeight   = (desc->flags & AV_PIX_FMT_FLAG_RGB) ?
                                          -1 : (src->height >> desc->log2_chroma_h) - 1,
                },
            },
        },
    };

    /* Disabled for now, applications will handle colorspace conversion themselves */
    if (false && ((desc->flags & AV_PIX_FMT_FLAG_RGB) || (src->colorspace != dst->colorspace))) {
        float (*src_mat)[4] = NULL, (*dst_mat)[4] = NULL, (*src_gamut_mat)[4] = NULL;

        switch (src->colorspace) {
            case AVCOL_SPC_SMPTE170M:
            default:
                /* Assume bt601 */
                src_mat = (src->color_range == AVCOL_RANGE_MPEG) ?
                    mat_ycbcr_bt601lim_to_rgb  : mat_ycbcr_bt601full_to_rgb;
                break;
            case AVCOL_SPC_BT709:
                config->slotStruct[0].slotConfig.DegammaMode =
                    config->outputConfig.RegammaMode = 2; /* bt709 gamma curve */
                src_mat = (src->color_range == AVCOL_RANGE_MPEG) ?
                    mat_ycbcr_bt709lim_to_rgb  : mat_ycbcr_bt709full_to_rgb;
                break;
            case AVCOL_SPC_BT2020_CL:
            case AVCOL_SPC_BT2020_NCL:
                config->slotStruct[0].slotConfig.DegammaMode =
                    config->outputConfig.RegammaMode = 2; /* Nothing available for bt2020 */
                src_mat = (src->color_range == AVCOL_RANGE_MPEG) ?
                    mat_ycbcr_bt2020lim_to_rgb : mat_ycbcr_bt2020full_to_rgb;
                src_gamut_mat = mat_colorgamut_bt2020_to_bt709;
                break;
        }

        switch (dst->colorspace) {
            case AVCOL_SPC_SMPTE170M:
            default:
                dst_mat = (dst->color_range == AVCOL_RANGE_MPEG) ?
                    mat_rgb_to_ycbcr_bt601lim  : mat_rgb_to_ycbcr_bt601full;
                break;
            case AVCOL_SPC_BT709:
                dst_mat = (dst->color_range == AVCOL_RANGE_MPEG) ?
                    mat_rgb_to_ycbcr_bt709lim  : mat_rgb_to_ycbcr_bt709full;
                break;
            case AVCOL_SPC_BT2020_CL:
            case AVCOL_SPC_BT2020_NCL:
                dst_mat = (dst->color_range == AVCOL_RANGE_MPEG) ?
                    mat_rgb_to_ycbcr_bt2020lim : mat_rgb_to_ycbcr_bt2020full;
                src_gamut_mat = mat_colorgamut_bt709_to_2020;
                break;
        }

        if (desc->flags & AV_PIX_FMT_FLAG_RGB)
            dst_mat = NULL;

        if (src_mat)
            set_matrix_struct(&config->slotStruct[0].colorMatrixStruct, src_mat, true, 2, 17);

        if (src_gamut_mat)
            set_matrix_struct(&config->slotStruct[0].gamutMatrixStruct, src_gamut_mat, true, 1, 18);

        if (dst_mat)
            set_matrix_struct(&config->outColorMatrixStruct, dst_mat, true, 0, 19);
    }
}

static int tx1_vic_prepare_cmdbuf(AVTX1DeviceContext *hwctx, AVTX1Map *map,
                                  uint32_t *map_offsets, int num_comps,
                                  const AVFrame *src, enum AVPixelFormat fmt)
{
    AVTX1Cmdbuf *cmdbuf = &hwctx->vic_cmdbuf;

    AVTX1Map *src_map;
    int input_reloc_type, err;

    src_map = ff_tx1_frame_get_fbuf_map(src);

    input_reloc_type = !src_map->is_linear ?
                       NVHOST_RELOC_TYPE_BLOCK_LINEAR : NVHOST_RELOC_TYPE_PITCH_LINEAR;

    err = ff_tx1_cmdbuf_begin(cmdbuf, HOST1X_CLASS_VIC);
    if (err < 0)
        return err;

    FF_TX1_PUSH_VALUE(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_APPLICATION_ID,
                      FF_TX1_VALUE(NVB0B6_VIDEO_COMPOSITOR_SET_APPLICATION_ID, ID, 1));
    FF_TX1_PUSH_VALUE(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS,
                      FF_TX1_VALUE(NVB0B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS, CONFIG_STRUCT_SIZE, sizeof(VicConfigStruct) >> 4) |
                      FF_TX1_VALUE(NVB0B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS, GPTIMER_ON,         1));
    FF_TX1_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_CONFIG_STRUCT_OFFSET,
                      &hwctx->vic_map, hwctx->vic_setup_off,  NVHOST_RELOC_TYPE_DEFAULT);
    FF_TX1_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_FILTER_STRUCT_OFFSET,
                      &hwctx->vic_map, hwctx->vic_filter_off, NVHOST_RELOC_TYPE_DEFAULT);

    for (int i = 0; i < num_comps; ++i)
        FF_TX1_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_OUTPUT_SURFACE_LUMA_OFFSET + i * sizeof(uint32_t),
                          map, map_offsets[i], NVHOST_RELOC_TYPE_PITCH_LINEAR);

    switch (fmt) {
        case AV_PIX_FMT_RGB565:
            /* 16-bit luma transfer */
            FF_TX1_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_SURFACE0_LUMA_OFFSET(0),
                              src_map, 0, input_reloc_type);
            break;
        case AV_PIX_FMT_RGB32:
            /* 16-bit chroma transfer */
            FF_TX1_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_SURFACE0_LUMA_OFFSET(0),
                              src_map, src->data[1] - src->data[0], input_reloc_type);
            break;
        case AV_PIX_FMT_NV12:
            /* Normal transfer */
            FF_TX1_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_SURFACE0_LUMA_OFFSET(0),
                              src_map, 0, input_reloc_type);
            FF_TX1_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_SURFACE0_CHROMA_U_OFFSET(0),
                              src_map, src->data[1] - src->data[0], input_reloc_type);
            break;
    }

    FF_TX1_PUSH_VALUE(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_EXECUTE,
                      FF_TX1_ENUM(NVB0B6_VIDEO_COMPOSITOR_EXECUTE, AWAKEN, ENABLE));

    err = ff_tx1_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    /* Insert syncpt increment to signal the end of the conversion */
    err = ff_tx1_cmdbuf_begin(cmdbuf, HOST1X_CLASS_VIC);
    if (err < 0)
        return err;

    err = ff_tx1_cmdbuf_push_word(cmdbuf, host1x_opcode_nonincr(NV_THI_INCR_SYNCPT, 1));
    if (err < 0)
        return err;

    err = ff_tx1_cmdbuf_push_word(cmdbuf,
        FF_TX1_VALUE(NV_THI_INCR_SYNCPT, INDX, hwctx->vic_channel.syncpt) |
        FF_TX1_ENUM (NV_THI_INCR_SYNCPT, COND, OP_DONE));
    if (err < 0)
        return err;

    err = ff_tx1_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    err = ff_tx1_cmdbuf_add_syncpt_incr(cmdbuf, hwctx->vic_channel.syncpt, 1, 0);
    if (err < 0)
        return err;

    return 0;
}

static int tx1_vic_transfer_data(AVHWFramesContext *ctx, AVFrame *dst, const AVFrame *src,
                                 enum AVPixelFormat fmt, AVTX1Map *map, uint32_t *plane_offsets,
                                 int num_planes, bool is_chroma)
{
    AVTX1DeviceContext *hwctx = ctx->device_ctx->hwctx;
    AVTX1Cmdbuf       *cmdbuf = &hwctx->vic_cmdbuf;

    uint32_t render_fence;
    uint8_t *mem;
    int err;

    mem = ff_tx1_map_get_addr(&hwctx->vic_map);

    tx1_vic_preprare_config((VicConfigStruct *)(mem + hwctx->vic_setup_off),
                            dst, src, fmt, is_chroma);

    err = ff_tx1_cmdbuf_clear(cmdbuf);
    if (err < 0)
        return err;

    err = tx1_vic_prepare_cmdbuf(hwctx, map, plane_offsets, num_planes, src, fmt);
    if (err < 0)
        goto fail;

    err = ff_tx1_channel_submit(&hwctx->vic_channel, cmdbuf, &render_fence);
    if (err < 0)
        goto fail;

    err = ff_tx1_syncpt_wait(&hwctx->vic_channel, render_fence, -1);
    if (err < 0)
        goto fail;

fail:
    return err;
}

static int tx1_transfer_data_from(AVHWFramesContext *ctx, AVFrame *dst, const AVFrame *src) {
#ifdef __SWITCH__
    AVTX1DeviceContext *hwctx = ctx->device_ctx->hwctx;
#endif

    AVTX1Map map = {0};
    uint8_t *map_base;
    uint32_t plane_offsets[4];
    int num_planes, i;
    int err;

    av_log(ctx, AV_LOG_DEBUG, "Transferring data from TX1 device, %s -> %s\n",
           av_get_pix_fmt_name(src->format), av_get_pix_fmt_name(dst->format));

    if (!src->hw_frames_ctx || dst->hw_frames_ctx)
        return AVERROR(ENOSYS);

    /*
     * HOS specific optimization: the frame buffer is directly used as backbuffer for VIC
     * Doing this on linux with NVMAP_IOC_FROM_VA leads to issues that seem to be related to cache
     * (syncpt is signalled when the buffer is still partially empty)
     * Making the map cpu uncacheable leads to heap corruption (due to overlap with other heap blocks?)
     */
#ifndef __SWITCH__
    map_base = dst->data[0];
    err = ff_tx1_map_allocate(&map, dst->buf[0]->size, 0x100, NVMAP_CACHE_OP_INV);
    if (err < 0)
        goto fail;
#else
    map.owner = hwctx->vic_channel.channel.fd;
    map_base = (uint8_t *)((uintptr_t)dst->buf[0]->data & ~0xfff);
    err = ff_tx1_map_from_va(&map, map_base, dst->buf[0]->size + ((uintptr_t)dst->buf[0]->data & 0xfff),
                             0x100, NVMAP_CACHE_OP_WB);
    if (err < 0)
        goto fail;
#endif

    err = ff_tx1_map_map(&map);
    if (err < 0)
        goto fail;

    num_planes = av_pix_fmt_count_planes(dst->format);
    for (i = 0; i < num_planes; ++i)
        plane_offsets[i] = (uintptr_t)(dst->data[i] - map_base);

    /* VIC expects planes in the reversed order */
    if (dst->format == AV_PIX_FMT_YUV420P)
        FFSWAP(uint32_t, plane_offsets[1], plane_offsets[2]);

    /*
     * VIC on the TX1 does not support 16-bit YUV surfaces.
     * Here we emulate them using two separates transfers for the luma and chroma planes
     * (16-bit and 32-bit widths respectively).
     */
    if (dst->format == AV_PIX_FMT_P010) {
        err = tx1_vic_transfer_data(ctx, dst, src, AV_PIX_FMT_RGB565,
                                    &map, &plane_offsets[0], 1, false);
        if (err < 0)
            goto fail;

        err = tx1_vic_transfer_data(ctx, dst, src, AV_PIX_FMT_RGB32,
                                    &map, &plane_offsets[1], 1, true);
        if (err < 0)
            goto fail;
    } else {
        err = tx1_vic_transfer_data(ctx, dst, src, dst->format,
                                    &map, plane_offsets, num_planes, false);
        if (err < 0)
            goto fail;
    }

#ifndef __SWITCH__
    memcpy(dst->buf[0]->data, ff_tx1_map_get_addr(&map), dst->buf[0]->size);
#endif

fail:
    ff_tx1_map_unmap(&map);

#ifndef __SWITCH__
    ff_tx1_map_free(&map);
#else
    ff_tx1_map_close(&map);
#endif

    return err;
}

const HWContextType ff_hwcontext_type_tx1 = {
    .type                   = AV_HWDEVICE_TYPE_TX1,
    .name                   = "tx1",

    .device_hwctx_size      = sizeof(AVTX1DeviceContext),
    .device_priv_size       = 0,
    .device_hwconfig_size   = 0,
    .frames_hwctx_size      = 0,
    .frames_priv_size       = 0,

    .device_create          = &tx1_device_create,
    .device_init            = &tx1_device_init,
    .device_uninit          = &tx1_device_uninit,

    .frames_get_constraints = &tx1_frames_get_constraints,
    .frames_init            = &tx1_frames_init,
    .frames_uninit          = &tx1_frames_uninit,
    .frames_get_buffer      = &tx1_get_buffer,

    .transfer_get_formats   = &tx1_transfer_get_formats,
    .transfer_data_from     = &tx1_transfer_data_from,

    .pix_fmts = (const enum AVPixelFormat[]) {
        AV_PIX_FMT_TX1,
        AV_PIX_FMT_NONE,
    },
};
