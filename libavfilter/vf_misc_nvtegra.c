/*
 * Copyright (c) 2024 averne <averne381@gmail.com>
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

#include <string.h>

#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/nvtegra_host1x.h"

#include "avfilter.h"
#include "video.h"
#include "internal.h"

#include "nvtegra_vpp.h"

/* Sharpening min/max/default values */
#define SHARPNESS_MIN      0.0
#define SHARPNESS_MAX      1.0
#define SHARPNESS_DEFAULT  0.0

/* Denoising min/max/default values */
#define DENOISE_MIN        0.0
#define DENOISE_MAX        1.0
#define DENOISE_DEFAULT    0.0

/* Dimension min/max/default values */
#define FILTER_TAP_MIN     NVB0B6_FILTER_LENGTH_5TAP
#define FILTER_TAP_MAX     NVB0B6_FILTER_LENGTH_10TAP
#define FILTER_TAP_DEFAULT NVB0B6_FILTER_LENGTH_5TAP

typedef struct NVTegraSpatialFilterContext {
    FFNVTegraVppContext core;

    bool has_filter_init;
    AVNVTegraMap filter_map;

    int filter_dimensions;
    float sharpness, denoise;
} NVTegraSpatialFilterContext;

/*
 * This table contains filter coefficients used during filtering and scaling operations in 5- and 10-tap modes.
 * Each word contains 3 10-bit values used for sharpening/denoising/normal filters respectively.
 * The data was dumped from the l4t firmware, and verified to be identical on HOS.
 */
static uint32_t g_VicFilterData[] = {
    /* 5-tap non-substream, 1:1 scaling ratio */
    0x00000000, 0x00100001, 0x3f203800, 0x3f2037ff, 0x3c70e400, 0x3bf0dbfa, 0x0391103f, 0x0351083c, 0x3bb037e4, 0x3d00f406, 0x3ff0a014, 0x03e11842, 0x0c51cc9c, 0x3fe003ff, 0x0811606c, 0x3f203c00,
    0x00300002, 0x00400002, 0x3f302fff, 0x3f302fff, 0x3b80cbf5, 0x3b20bff1, 0x03010439, 0x02c0fc35, 0x3da1040d, 0x3e511015, 0x04311c45, 0x04812448, 0x3fb003fe, 0x3f9003fc, 0x3f104401, 0x3f104802,
    0x00500002, 0x00500003, 0x3f4027ff, 0x3f4023ff, 0x3ad0b3ed, 0x3aa0a3e9, 0x0270f832, 0x0230f030, 0x3f21201d, 0x3ff13026, 0x04c1284b, 0x0511304e, 0x3f5003fb, 0x3f2007f9, 0x3f205003, 0x3f205403,
    0x00500403, 0x00500403, 0x3f601fff, 0x3f701bfe, 0x3a7097e6, 0x3a608be4, 0x01f0e82d, 0x01b0e02a, 0x00e1402f, 0x01d14c38, 0x05613451, 0x05a13c54, 0x3ee007f7, 0x3ea00bf6, 0x3f205c04, 0x3f306006,

    /* 5-tap non-substream, 2:1 scaling ratio */
    0x00500403, 0x00500402, 0x3f7017fe, 0x3f8017ff, 0x3a507fe2, 0x3a6073e1, 0x0170d827, 0x0140d424, 0x02e15c42, 0x03f16c4d, 0x05f14057, 0x0641445a, 0x3e500ff4, 0x3e000ff2, 0x3f406807, 0x3f406c08,
    0x00400402, 0x00300402, 0x3fa013ff, 0x3fa00fff, 0x3a706be0, 0x3a905fe0, 0x0100cc22, 0x00d0c41f, 0x05117858, 0x06318863, 0x0681485d, 0x06d14c60, 0x3db013f0, 0x3d6017ee, 0x3f507409, 0x3f607c0b,
    0x00300402, 0x00200001, 0x3fc00fff, 0x3fd00bff, 0x3ab057e0, 0x3af04fe1, 0x0090bc1c, 0x0070b41a, 0x0771986e, 0x08a1a47a, 0x07115063, 0x07515465, 0x3d001beb, 0x3cb023e9, 0x3f80840c, 0x3f908c0e,
    0x00100001, 0x00100000, 0x3fe007ff, 0x3ff00400, 0x3b2047e2, 0x3b703fe2, 0x0040b018, 0x0010a816, 0x09e1b485, 0x0b11c091, 0x07915868, 0x07d15c6a, 0x3c6027e7, 0x3c002fe6, 0x3fb0900f, 0x3fd09811,

    /* 5-tap non-substream, 4:1 scaling ratio */
    0x00a04c0e, 0x0090440d, 0x01204c13, 0x01004811, 0x0460f442, 0x0440f040, 0x0470f041, 0x0450ec40, 0x0240a827, 0x0480fc43, 0x02c0a82b, 0x0480f442, 0x05c15859, 0x00b05010, 0x05415855, 0x01305414,
    0x0080400c, 0x0070380b, 0x00f04410, 0x00d0400f, 0x0410ec3e, 0x03f0e83d, 0x0430e43e, 0x0420e43d, 0x04910044, 0x04b10846, 0x0490fc44, 0x04a10045, 0x00d05411, 0x00e05812, 0x01405815, 0x01605c17,
    0x0060340a, 0x00503009, 0x00c0380e, 0x00b0380c, 0x03d0e43b, 0x03b0e039, 0x0400dc3c, 0x03e0dc3a, 0x04d10c48, 0x04e11449, 0x04b10846, 0x04c10c48, 0x00f06013, 0x01106415, 0x01806418, 0x0190681a,
    0x00402c08, 0x00302807, 0x0090300b, 0x00802c09, 0x0390d838, 0x0370d436, 0x03d0d439, 0x03b0d038, 0x05011c4b, 0x0511204c, 0x04c11449, 0x04e11c4a, 0x01206c16, 0x01407018, 0x01b0701b, 0x01d0741d,

    /* 5-tap non-substream, 8:1 scaling ratio */
    0x00302406, 0x00202005, 0x00702809, 0x00602007, 0x0340d034, 0x0320cc32, 0x0390cc36, 0x0380c835, 0x0531244e, 0x05412c50, 0x04f1204b, 0x04f1284d, 0x01507419, 0x01707c1b, 0x01e07c1e, 0x02008020,
    0x00101c04, 0x00101803, 0x00502006, 0x00401805, 0x0300c431, 0x02e0c02f, 0x0360c434, 0x0350c032, 0x05613451, 0x05713853, 0x05012c4e, 0x0511344f, 0x0180801d, 0x01a0881e, 0x02208421, 0x02308c23,
    0x00101002, 0x00000c02, 0x00301404, 0x00200c03, 0x02b0bc2d, 0x02a0b82c, 0x0330bc31, 0x0310b830, 0x05814054, 0x05914455, 0x05113c50, 0x05214051, 0x01c08c20, 0x01e09421, 0x02509025, 0x02709826,
    0x00000801, 0x00000400, 0x00100c02, 0x00100401, 0x0280b02a, 0x0260ac28, 0x0300b42e, 0x02e0ac2d, 0x05a14c57, 0x05b15458, 0x05314852, 0x05315053, 0x02009c23, 0x0220a025, 0x02809c28, 0x02a0a429,

    /* 5-tap non-substream, 16:1 scaling ratio */
    0x01405014, 0x01204812, 0x00000000, 0x00000000, 0x0470ec41, 0x0460e840, 0x3bd10c00, 0x3c010000, 0x02e0ac2c, 0x0480f442, 0x3e606800, 0x3c411404, 0x05215454, 0x01505415, 0x09a19880, 0x3ff00400,
    0x01004811, 0x00f0400f, 0x00000000, 0x00000000, 0x0440e43f, 0x0420e03d, 0x3c30f400, 0x3c60e800, 0x0480f843, 0x04a10045, 0x3cc1200a, 0x3d51280f, 0x01705816, 0x01806018, 0x3fe00800, 0x3fc01000,
    0x00d03c0e, 0x00c0380d, 0x00000000, 0x00000000, 0x0410dc3c, 0x03f0d83b, 0x3c80dc00, 0x3cb0d400, 0x04a10846, 0x04b10c47, 0x3df13415, 0x3ea13c1c, 0x01a0641a, 0x01c0681b, 0x3fb01400, 0x3fa01800,
    0x00b0340c, 0x0090300b, 0x00000000, 0x00000000, 0x03d0d43a, 0x03c0d038, 0x3ce0c800, 0x3d10bc00, 0x04c11048, 0x04d11849, 0x3f614423, 0x0031502b, 0x01d0701c, 0x01f0741e, 0x3f802000, 0x3f702400,


    /* 5-tap substream, 1:1 scaling ratio */
    0x00802809, 0x00702408, 0x00000000, 0x00000000, 0x03b0cc37, 0x0390c836, 0x3d30b000, 0x3d60a800, 0x04e1204b, 0x04e1284c, 0x01115833, 0x0201603b, 0x02007c1f, 0x02208021, 0x3f502c00, 0x3f303000,
    0x00602007, 0x00501805, 0x00000000, 0x00000000, 0x0370c434, 0x0360c033, 0x3d909c00, 0x3db09400, 0x04f12c4d, 0x04f1344e, 0x02f16844, 0x0401704e, 0x02308822, 0x02508c24, 0x3f203800, 0x3f004000,
    0x00401404, 0x00201003, 0x00000000, 0x00000000, 0x0340bc32, 0x0330b831, 0x3dd08c00, 0x3e008000, 0x0501384f, 0x05014050, 0x05117857, 0x06218061, 0x02709426, 0x02909827, 0x3ee04800, 0x3ec05000,
    0x00200802, 0x00100401, 0x00000000, 0x00000000, 0x0310b42f, 0x0300b02e, 0x3e207800, 0x3e407000, 0x05114851, 0x05115052, 0x0751886b, 0x08719075, 0x02a09c29, 0x02c0a42b, 0x3ea05800, 0x3e806000,

    /* 5-tap substream, 2:1 scaling ratio */
    0x3ee04800, 0x3ef04400, 0x01004c12, 0x00f04811, 0x0490f443, 0x0440f040, 0x03f0f03d, 0x03e0ec3c, 0x00a0a41a, 0x04c0fc45, 0x0280a829, 0x0400f43f, 0x07615c66, 0x3ef04c01, 0x05815857, 0x01205013,
    0x3f104000, 0x3f203800, 0x00e04410, 0x00d0400e, 0x03f0ec3d, 0x03b0e83a, 0x03c0e43b, 0x03b0e43a, 0x04f10048, 0x0521084a, 0x0420fc40, 0x04410042, 0x3f005002, 0x3f105804, 0x01305814, 0x01405c16,
    0x3f303400, 0x3f403000, 0x00c0380d, 0x00b0340c, 0x0360e437, 0x0320dc35, 0x0390dc39, 0x0380dc37, 0x0561104d, 0x0581144f, 0x04510844, 0x04710c45, 0x3f205c05, 0x3f406406, 0x01606417, 0x01706819,
    0x3f502c00, 0x3f602800, 0x0090300b, 0x00802c0a, 0x02e0d832, 0x0290d42f, 0x0370d836, 0x0350d035, 0x05c11c51, 0x05f12054, 0x04811447, 0x04a11c48, 0x3f506808, 0x3f706c09, 0x01906c1a, 0x01a0741b,

    /* 5-tap substream, 4:1 scaling ratio */
    0x3f702400, 0x3f802000, 0x00702809, 0x00602008, 0x0260cc2c, 0x0220c82a, 0x0340cc33, 0x0330c832, 0x06212856, 0x06513058, 0x04b1204a, 0x04d1284c, 0x3f80740b, 0x3fa0780c, 0x01c0781d, 0x01d0801e,
    0x3fa01800, 0x3fb01400, 0x00501c06, 0x00401805, 0x01e0c427, 0x01a0c025, 0x0310c431, 0x0300c030, 0x0671345a, 0x06a13c5c, 0x04f1304d, 0x0501344f, 0x3fc0800e, 0x3fe08410, 0x01f08420, 0x02008c21,
    0x3fc01000, 0x3fd00c00, 0x00301404, 0x00301003, 0x0170bc23, 0x0130b420, 0x02e0bc2f, 0x02d0b82d, 0x06c1405e, 0x06f14860, 0x05213850, 0x05314052, 0x00008c12, 0x00309414, 0x02209023, 0x02309425,
    0x3fe00800, 0x3ff00400, 0x00200c02, 0x00100401, 0x0100b01e, 0x00d0ac1c, 0x02b0b42c, 0x02a0ac2b, 0x07215062, 0x07415464, 0x05514854, 0x05615055, 0x00509816, 0x0070a018, 0x02509c26, 0x0260a428,

    /* 5-tap substream, 8:1 scaling ratio */
    0x01305013, 0x01204812, 0x01405014, 0x01204812, 0x03c0ec3c, 0x03b0e83b, 0x03b0ec3b, 0x03a0e83a, 0x02a0ac2a, 0x03d0f43d, 0x02b0ac2b, 0x03d0f43d, 0x05615456, 0x01405414, 0x05515455, 0x01505415,
    0x01104411, 0x00f04010, 0x01104811, 0x01004010, 0x03a0e439, 0x0390e039, 0x0390e439, 0x0380e038, 0x03f0fc3f, 0x04010040, 0x03f0f83f, 0x04010040, 0x01505816, 0x01706017, 0x01605816, 0x01806018,
    0x00e03c0e, 0x00d0380d, 0x00f03c0f, 0x00e0380e, 0x0380dc37, 0x0370d837, 0x0370dc37, 0x0360d836, 0x04210442, 0x04410c43, 0x04110441, 0x04310c43, 0x01806419, 0x0190681a, 0x01906419, 0x01a0681a,
    0x00c0300c, 0x00b02c0b, 0x00c0340c, 0x00b0300b, 0x0360d436, 0x0350d034, 0x0350d435, 0x0340d034, 0x04511445, 0x04711847, 0x04511045, 0x04611846, 0x01b0701b, 0x01c0741d, 0x01c0701c, 0x01d0741d,

    /* 5-tap substream, 16:1 scaling ratio */
    0x0090280a, 0x00802408, 0x00a0280a, 0x00902409, 0x0330cc33, 0x0320c832, 0x0330cc33, 0x0320c832, 0x04912048, 0x04a1284a, 0x04812048, 0x0491284a, 0x01e07c1e, 0x02008020, 0x01f07c1f, 0x02008020,
    0x00702007, 0x00601806, 0x00802008, 0x00601806, 0x0310c431, 0x0300c030, 0x0310c431, 0x0300c030, 0x04c12c4c, 0x04d1344d, 0x04b12c4b, 0x04d1344d, 0x02108421, 0x02208c23, 0x02108821, 0x02308c23,
    0x00401405, 0x00300c04, 0x00501405, 0x00301003, 0x02f0bc2f, 0x02e0b82e, 0x02f0bc2f, 0x02e0b82e, 0x04f13c4e, 0x05114050, 0x04f1384f, 0x05014050, 0x02409024, 0x02509825, 0x02409424, 0x02609826,
    0x00200803, 0x00100401, 0x00300802, 0x00100401, 0x02c0b42d, 0x02b0b02b, 0x02d0b42d, 0x02b0b02c, 0x05314852, 0x05415054, 0x05214852, 0x05415054, 0x02709c27, 0x0290a429, 0x02709c27, 0x0290a429,


    /* 10-tap non-substream, 1:1 scaling ratio */
    0x00000000, 0x00100000, 0x00000000, 0x00000000, 0x3fa01800, 0x3f701bff, 0x3de01bf2, 0x3df01bf3, 0x00000000, 0x00600003, 0x3e307400, 0x3e1073fe, 0x3c90dc00, 0x3be0cff9, 0x0610e84d, 0x05c0e44a,
    0x00300802, 0x3d50e808, 0x00200402, 0x0650e850, 0x3ed00bf7, 0x3fa003fd, 0x3f2007fa, 0x3e607402, 0x02501015, 0x3fd01801, 0x3d3043f1, 0x3dd01ff2, 0x396033d1, 0x3ff003ff, 0x0170ac21, 0x00000000,
    0x0d51b0a1, 0x00000000, 0x0a210c72, 0x00000000, 0x00200001, 0x00200001, 0x00100000, 0x00100001, 0x3f501bfd, 0x3f301bfc, 0x3e001bf3, 0x3e2017f3, 0x00b00006, 0x01000008, 0x3df06ffd, 0x3dd06bfc,
    0x3b40c3f2, 0x3ac0b7ec, 0x0570e048, 0x0530dc45, 0x3e30f410, 0x3f110419, 0x06a0ec53, 0x06f0f055, 0x3f3003fa, 0x3ec003f7, 0x3e807803, 0x3ea07c05, 0x3ff01803, 0x00201804, 0x3dc01ff1, 0x3db023f1,
    0x3fe003ff, 0x3fd003ff, 0x3ff00000, 0x3fe003ff, 0x00000400, 0x00000400, 0x00000000, 0x00000000, 0x00300001, 0x00300002, 0x00200001, 0x00200001, 0x3f1017fb, 0x3f0017fa, 0x3e3017f4, 0x3e4013f4,
    0x0150040b, 0x0190040d, 0x3dc067fa, 0x3da063f9, 0x3a40a7e7, 0x39d09be2, 0x04e0d842, 0x0490d43f, 0x3ff11022, 0x00f1202b, 0x0730f058, 0x0770f45a, 0x3e5007f3, 0x3de007f0, 0x3ed08007, 0x3f008409,
    0x00501806, 0x00801807, 0x3d9023f1, 0x3d9027f1, 0x3fc003fe, 0x3fa003fd, 0x3fe003ff, 0x3fd003ff, 0x00000400, 0x00000400, 0x00000400, 0x00100400, 0x00400002, 0x00400002, 0x00200001, 0x00200001,
    0x3ee017fa, 0x3ed017f9, 0x3e6013f5, 0x3e7013f5, 0x01d0040f, 0x02000410, 0x3d905ff8, 0x3d805ff7, 0x39708fdd, 0x393083da, 0x0440d43c, 0x03f0d03a, 0x01f12c35, 0x03013c40, 0x07c0f85d, 0x0800f85f,
    0x3d700bec, 0x3cf00be9, 0x3f30880b, 0x3f608c0d, 0x00c01809, 0x00f0180a, 0x3d7027f0, 0x3d702bf0, 0x3f9007fd, 0x3f7007fc, 0x3fc003fe, 0x3fb003fe, 0x3ff00400, 0x00000400, 0x00100401, 0x00100001,

    /* 10-tap non-substream, 2:1 scaling ratio */
    0x00400002, 0x00400402, 0x00200001, 0x00200001, 0x3ec013f8, 0x3eb013f8, 0x3e800ff6, 0x3ea00ff6, 0x02200812, 0x02400813, 0x3d705bf6, 0x3d6057f6, 0x38f07bd7, 0x38d06fd4, 0x03b0cc37, 0x0360c834,
    0x0421484a, 0x05415855, 0x0840fc62, 0x0880fc64, 0x3c800fe5, 0x3c100fe2, 0x3f90900f, 0x3fd09411, 0x0120180c, 0x0150180d, 0x3d602bf0, 0x3d502ff0, 0x3f6007fb, 0x3f5007fb, 0x3fb003fd, 0x3fa003fd,
    0x00000401, 0x00000401, 0x00100401, 0x00100401, 0x00400402, 0x00400402, 0x00300001, 0x00200002, 0x3eb013f7, 0x3eb00ff7, 0x3eb00ff7, 0x3ec00bf7, 0x02600814, 0x02700c15, 0x3d5053f5, 0x3d404ff4,
    0x38c063d2, 0x38b05bd1, 0x0310c431, 0x02d0c02f, 0x06616460, 0x0781706a, 0x08c10066, 0x09010468, 0x3ba013df, 0x3b3017dc, 0x00009813, 0x00409c15, 0x0180180f, 0x01b01810, 0x3d5033f0, 0x3d4033f0,
    0x3f3007fa, 0x3f2007fa, 0x3f9003fd, 0x3f8007fc, 0x00000801, 0x00100801, 0x00100401, 0x00200401, 0x00400403, 0x00400402, 0x00200001, 0x00200002, 0x3eb00ff7, 0x3eb00ff7, 0x3ed00bf8, 0x3ef00bf8,
    0x02700c15, 0x02700c15, 0x3d404ff3, 0x3d304bf3, 0x38c053d0, 0x38d04bcf, 0x0280bc2c, 0x0240b829, 0x08b18076, 0x09e18c81, 0x0941046b, 0x0981086d, 0x3ac01bd9, 0x3a6023d7, 0x0070a017, 0x00b0a41a,
    0x01d01411, 0x02001412, 0x3d4037f0, 0x3d3037f0, 0x3f1007f9, 0x3f0007f9, 0x3f7007fc, 0x3f6007fb, 0x00100801, 0x00100802, 0x00200401, 0x00200401, 0x00300402, 0x00300402, 0x00200001, 0x00200402,
    0x3eb00ff7, 0x3ec00ff7, 0x3f000bf9, 0x3f100bf9, 0x02701015, 0x02601015, 0x3d3047f2, 0x3d3043f2, 0x39003fd0, 0x39303bd0, 0x0200b426, 0x01b0b024, 0x0b01988b, 0x0c31a496, 0x09b1086f, 0x09e10c70,
    0x3a0027d5, 0x39b02bd3, 0x00f0a81c, 0x0130a81f, 0x02201413, 0x02401414, 0x3d303bf1, 0x3d303ff1, 0x3ef00bf8, 0x3ee00bf8, 0x3f5007fb, 0x3f4007fa, 0x00200402, 0x00200402, 0x00200401, 0x00200401,

    /* 10-tap non-substream, 4:1 scaling ratio */
    0x3f403000, 0x3f502c00, 0x00303007, 0x00203007, 0x3fb0640a, 0x3fa06409, 0x00e06413, 0x00e06013, 0x01d0841f, 0x01c0841f, 0x02008821, 0x01f08821, 0x04709837, 0x04609836, 0x0320982d, 0x0320982c,
    0x3fa017ff, 0x04809c37, 0x00001803, 0x0330982c, 0x3f404c03, 0x01f08420, 0x0070480d, 0x02108821, 0x00a07414, 0x3fb0640a, 0x0170741a, 0x00e06414, 0x0330902c, 0x3f403000, 0x02a09427, 0x00303008,
    0x05509c3e, 0x00000000, 0x0380982f, 0x00000000, 0x3f502c00, 0x3f502800, 0x00202c07, 0x00202c06, 0x3f906009, 0x3f906008, 0x00d06012, 0x00d05c12, 0x01b0801d, 0x01a0801d, 0x01f08420, 0x01e08420,
    0x04509835, 0x04309835, 0x0310982c, 0x0310982b, 0x04909c38, 0x04a09c38, 0x0330982d, 0x0340982d, 0x02108421, 0x02208422, 0x02208822, 0x02208c22, 0x3fc0640b, 0x3fd0680b, 0x00f06414, 0x00f06415,
    0x3f403400, 0x3f403400, 0x00303408, 0x00303408, 0x3ff00400, 0x3ff00400, 0x00000400, 0x00000401, 0x3f502800, 0x3f602400, 0x00202806, 0x00102806, 0x3f806008, 0x3f805c07, 0x00c05c12, 0x00c05811,
    0x0190801c, 0x0170801b, 0x01e0841f, 0x01d0841f, 0x04209834, 0x04109833, 0x0300982b, 0x0300982b, 0x04b09c39, 0x04c09c39, 0x0340982d, 0x0340982d, 0x02308422, 0x02508823, 0x02308c23, 0x02308c23,
    0x3fe0680c, 0x3ff0680d, 0x01006815, 0x01106815, 0x3f403400, 0x3f303801, 0x00303408, 0x00403809, 0x3ff00400, 0x3fe00400, 0x00000401, 0x00000801, 0x3f6027ff, 0x3f6023ff, 0x00102805, 0x00102405,
    0x3f705c07, 0x3f705c07, 0x00b05811, 0x00b05810, 0x01607c1b, 0x01507c1a, 0x01d0801e, 0x01c0801e, 0x04009833, 0x03f09432, 0x02f0982b, 0x02f0982a, 0x04d09c3a, 0x04e09c3a, 0x0350982e, 0x0350982e,
    0x02608824, 0x02708825, 0x02408c23, 0x02408c24, 0x0000680d, 0x00106c0e, 0x01106816, 0x01206c16, 0x3f303801, 0x3f303c01, 0x00403809, 0x00403c0a, 0x3fe00800, 0x3fd00800, 0x00000801, 0x00000801,

    /* 10-tap non-substream, 8:1 scaling ratio */
    0x3f7023ff, 0x3f7023ff, 0x00102405, 0x00102005, 0x3f605806, 0x3f605806, 0x00a05410, 0x00a0540f, 0x01307c19, 0x01207c18, 0x01b0801d, 0x01b0801d, 0x03e09432, 0x03d09431, 0x02e0982a, 0x02d0942a,
    0x04f09c3b, 0x05009c3c, 0x0360982e, 0x0360982e, 0x02908c26, 0x02a08826, 0x02508c24, 0x02509025, 0x00206c0e, 0x00306c0f, 0x01206c17, 0x01306c17, 0x3f303c01, 0x3f304002, 0x00503c0a, 0x0050400a,
    0x3fd00800, 0x3fc00bff, 0x00000c01, 0x00000c01, 0x3f701fff, 0x3f801fff, 0x00102004, 0x00102004, 0x3f505805, 0x3f505405, 0x00a0500f, 0x0090500f, 0x01107818, 0x01007817, 0x01a07c1d, 0x01a07c1c,
    0x03b09430, 0x03a09430, 0x02d09429, 0x02c09429, 0x05109c3c, 0x05109c3c, 0x0360982e, 0x0370982e, 0x02c08c27, 0x02d08c28, 0x02609025, 0x02709025, 0x00406c10, 0x00506c10, 0x01307017, 0x01407018,
    0x3f304002, 0x3f304402, 0x0050400b, 0x0050400b, 0x3fc00fff, 0x3fc00fff, 0x00001002, 0x00001002, 0x3f801bff, 0x3f901bff, 0x00101c04, 0x00001c04, 0x3f505405, 0x3f405004, 0x0090500e, 0x00804c0e,
    0x00f07816, 0x00d07816, 0x01907c1c, 0x0190781b, 0x0390942f, 0x0380902e, 0x02c09428, 0x02c09428, 0x05209c3d, 0x05309c3d, 0x0370982f, 0x0370982f, 0x02e08c29, 0x03008c29, 0x02709026, 0x02809026,
    0x00607011, 0x00707012, 0x01407018, 0x01507419, 0x3f304402, 0x3f304803, 0x0060440b, 0x0060440b, 0x3fb00fff, 0x3fb013ff, 0x00001002, 0x00001402, 0x3f901bff, 0x3f9017ff, 0x00001c03, 0x00001803,
    0x3f405004, 0x3f404c04, 0x00804c0d, 0x00804c0d, 0x00c07415, 0x00b07414, 0x0180781b, 0x0170781b, 0x0360902d, 0x0350902c, 0x02b09428, 0x02a09427, 0x0540a03e, 0x0540a03e, 0x0380982f, 0x0380982f,
    0x03108c2a, 0x0320902b, 0x02809026, 0x02909027, 0x00807012, 0x00907413, 0x01607419, 0x0160741a, 0x3f304803, 0x3f404803, 0x0060440c, 0x0070480c, 0x3fb013ff, 0x3fa013ff, 0x00001403, 0x00001403,

    /* 10-tap non-substream, 16:1 scaling ratio */
    0x00802c0a, 0x00802809, 0x00000000, 0x00000000, 0x01505415, 0x01505415, 0x00000000, 0x00000000, 0x02308021, 0x02207c21, 0x00000000, 0x00000000, 0x02b0a02a, 0x02b0a029, 0x3b213800, 0x3b413000,
    0x00301404, 0x02b0a42a, 0x00000000, 0x3c413c09, 0x00e0400f, 0x02308021, 0x00000000, 0x3fe00800, 0x01c06c1b, 0x01605816, 0x00000000, 0x00000000, 0x02809026, 0x00802c0a, 0x3da09800, 0x00000000,
    0x02b0b02c, 0x00000000, 0x0a616880, 0x00000000, 0x00802809, 0x00702808, 0x00000000, 0x00000000, 0x01405415, 0x01405014, 0x00000000, 0x00000000, 0x02207c20, 0x02107c20, 0x00000000, 0x00000000,
    0x02b0a029, 0x02a0a029, 0x3b612800, 0x3b812000, 0x02b0a42a, 0x02b0a42a, 0x3d713c13, 0x3ea1401d, 0x02308022, 0x02408022, 0x3fc01000, 0x3fa01800, 0x01605816, 0x01705816, 0x00000000, 0x00000000,
    0x00902c0a, 0x0090300b, 0x00000000, 0x00000000, 0x00000400, 0x00100401, 0x00000000, 0x00000000, 0x00702408, 0x00602408, 0x00000000, 0x00000000, 0x01305014, 0x01305013, 0x00000000, 0x00000000,
    0x0210781f, 0x0210781f, 0x00000000, 0x00000000, 0x02a09c29, 0x02a09c29, 0x3ba11800, 0x3bc11000, 0x02b0a82a, 0x02b0a82a, 0x3fe14427, 0x01114431, 0x02408422, 0x02408423, 0x3f802000, 0x3f602800,
    0x01705c17, 0x01705c17, 0x00000000, 0x00000000, 0x00a0300b, 0x00a0300b, 0x00000000, 0x00000000, 0x00100401, 0x00100401, 0x00000000, 0x00000000, 0x00602407, 0x00602007, 0x00000000, 0x00000000,
    0x01304c13, 0x01204c13, 0x00000000, 0x00000000, 0x0200781f, 0x0200741e, 0x00000000, 0x00000000, 0x02a09c28, 0x02909c28, 0x3bf10400, 0x3c10fc00, 0x02b0a82b, 0x02b0a82b, 0x0231483b, 0x03514c44,
    0x02408423, 0x02508823, 0x3f403400, 0x3f103c00, 0x01805c18, 0x01806018, 0x00000000, 0x00000000, 0x00a0340b, 0x00b0340c, 0x00000000, 0x00000000, 0x00100801, 0x00100802, 0x00000000, 0x00000000,


    /* 10-tap substream, 1:1 scaling ratio */
    0x00602007, 0x00501c06, 0x00000000, 0x00000000, 0x01204c12, 0x01104812, 0x00000000, 0x00000000, 0x01f0741e, 0x01f0741e, 0x00000000, 0x00000000, 0x02909828, 0x02909827, 0x3c40f000, 0x3c60e800,
    0x02b0a82b, 0x02b0ac2b, 0x0461504d, 0x05615055, 0x02508824, 0x02508824, 0x3ef04400, 0x3ed05000, 0x01906018, 0x01906019, 0x00000000, 0x00000000, 0x00b0340c, 0x00c0380d, 0x00000000, 0x00000000,
    0x00100c02, 0x00200c02, 0x00000000, 0x00000000, 0x00501c06, 0x00501c06, 0x00000000, 0x00000000, 0x01104811, 0x01004811, 0x00000000, 0x00000000, 0x01e0701e, 0x01e0701d, 0x00000000, 0x00000000,
    0x02909827, 0x02909427, 0x3c90dc00, 0x3cc0d000, 0x02b0ac2b, 0x02b0ac2b, 0x0651545d, 0x07315864, 0x02608824, 0x02608c24, 0x3ea05800, 0x3e706400, 0x01906419, 0x01a0641a, 0x00000000, 0x00000000,
    0x00c0380d, 0x00c0380d, 0x00000000, 0x00000000, 0x00200c03, 0x00201003, 0x00000000, 0x00000000, 0x00501c05, 0x00401805, 0x00000000, 0x00000000, 0x01004411, 0x01004410, 0x00000000, 0x00000000,
    0x01e0701d, 0x01d06c1c, 0x00000000, 0x00000000, 0x02809427, 0x02809427, 0x3ce0c800, 0x3d10bc00, 0x02b0ac2b, 0x02b0b02c, 0x08015c6b, 0x08b15c71, 0x02608c24, 0x02608c25, 0x3e506c00, 0x3e207800,
    0x01a0641a, 0x01b0681a, 0x00000000, 0x00000000, 0x00d03c0e, 0x00d03c0e, 0x00000000, 0x00000000, 0x00201003, 0x00301003, 0x00000000, 0x00000000, 0x00401805, 0x00301804, 0x00000000, 0x00000000,
    0x00f04410, 0x00f0400f, 0x00000000, 0x00000000, 0x01d06c1c, 0x01c06c1c, 0x00000000, 0x00000000, 0x02809426, 0x02809026, 0x3d40b000, 0x3d70a400, 0x02b0b02c, 0x02b0b02c, 0x09516077, 0x09f1647c,
    0x02708c25, 0x02709025, 0x3df08400, 0x3dc09000, 0x01b0681a, 0x01c0681b, 0x00000000, 0x00000000, 0x00d03c0e, 0x00e0400f, 0x00000000, 0x00000000, 0x00301004, 0x00301404, 0x00000000, 0x00000000,

    /* 10-tap substream, 2:1 scaling ratio */
    0x00000000, 0x00000000, 0x3f503000, 0x3f502c00, 0x3ec05000, 0x3ed04c00, 0x00f06014, 0x00e06013, 0x3da09800, 0x3da09800, 0x02a08826, 0x02a08826, 0x06e0b84e, 0x06b0b84c, 0x0350982e, 0x03509c2e,
    0x00000000, 0x06f0b84f, 0x3fb01400, 0x03509c2e, 0x3f702400, 0x3dd09802, 0x3ff04809, 0x02a08826, 0x3e207800, 0x3ec05000, 0x01f0741e, 0x01006014, 0x0230a826, 0x00000000, 0x02f0942a, 0x3f503000,
    0x0840bc5a, 0x00000000, 0x03809c2f, 0x00000000, 0x00000000, 0x00000000, 0x3f502c00, 0x3f502800, 0x3ee04800, 0x3ee04400, 0x00d05c12, 0x00c05c11, 0x3db09400, 0x3db09400, 0x02908825, 0x02908425,
    0x0670b44a, 0x0630b448, 0x0340982d, 0x0340982d, 0x0700b84f, 0x0720b850, 0x03609c2e, 0x03609c2e, 0x3e109c04, 0x3e509c06, 0x02b08827, 0x02b08c27, 0x3eb05400, 0x3ea05800, 0x01106415, 0x01206416,
    0x3ff00400, 0x3ff00400, 0x3f603001, 0x3f603401, 0x00000000, 0x00000000, 0x3ff00400, 0x3ff00400, 0x00000000, 0x00000000, 0x3f602800, 0x3f602800, 0x3ef04400, 0x3f004000, 0x00b05c11, 0x00a05810,
    0x3dc09000, 0x3dc09000, 0x02808424, 0x02808424, 0x05f0b446, 0x05b0b444, 0x0330982d, 0x0330982d, 0x0730b851, 0x0740b851, 0x03609c2e, 0x0360982e, 0x3e909c08, 0x3ed09c0a, 0x02b08c27, 0x02c08c27,
    0x3ea05800, 0x3e905c00, 0x01306416, 0x01406817, 0x3fe00800, 0x3fe00800, 0x3f703402, 0x3f703402, 0x00000000, 0x00000000, 0x3ff00400, 0x3fe00800, 0x00000000, 0x00000000, 0x3f702400, 0x3f702400,
    0x3f103c00, 0x3f103c00, 0x0090580f, 0x0080540f, 0x3dd09000, 0x3dd08c00, 0x02708024, 0x02608023, 0x0560b041, 0x0510b03f, 0x0320982c, 0x0320982c, 0x0760bc52, 0x0770bc53, 0x03609c2f, 0x03709c2f,
    0x3f109c0d, 0x3f60a00f, 0x02c08c27, 0x02c08c28, 0x3e806000, 0x3e806000, 0x01506818, 0x01606c18, 0x3fd00c00, 0x3fd00c00, 0x3f803803, 0x3f803803, 0x00000000, 0x00000000, 0x3fe00800, 0x3fe00800,

    /* 10-tap substream, 4:1 scaling ratio */
    0x00000000, 0x00000000, 0x3f802000, 0x3f802000, 0x3f203800, 0x3f303400, 0x0070540e, 0x0060500d, 0x3dd08800, 0x3de08800, 0x02508022, 0x02507c22, 0x04d0b03c, 0x0480b03a, 0x0320982c, 0x0310982c,
    0x0790bc54, 0x07a0bc54, 0x0370982f, 0x03709c2f, 0x3fa0a011, 0x3ff0a014, 0x02c08c28, 0x02d09028, 0x3e706400, 0x3e606800, 0x01706c19, 0x01806c1a, 0x3fc01000, 0x3fc01000, 0x3f903c04, 0x3fa03c04,
    0x00000000, 0x00000000, 0x3fd00c00, 0x3fd00c00, 0x00000000, 0x00000000, 0x3f802000, 0x3f901c00, 0x3f303400, 0x3f403000, 0x0050500d, 0x0040500c, 0x3de08800, 0x3df08400, 0x02407c22, 0x02307c21,
    0x0430ac37, 0x03e0ac34, 0x0310982b, 0x0310942b, 0x07c0bc55, 0x07d0bc56, 0x03709c2f, 0x0380982f, 0x0040a016, 0x0090a419, 0x02d09028, 0x02d09029, 0x3e606800, 0x3e506c00, 0x01906c1a, 0x01a0701b,
    0x3fb01400, 0x3fa01400, 0x3fb03c05, 0x3fb04005, 0x00000000, 0x00000000, 0x3fd00c00, 0x3fc01000, 0x00000000, 0x00000000, 0x3f901c00, 0x3f901c00, 0x3f502c00, 0x3f502800, 0x00304c0b, 0x00204c0a,
    0x3e008000, 0x3e008000, 0x02207c20, 0x02107820, 0x0380ac32, 0x0330ac2f, 0x0300942b, 0x0300942b, 0x07e0bc57, 0x0800bc57, 0x03809c2f, 0x03809c2f, 0x00e0a41b, 0x0130a41e, 0x02e09029, 0x02e09029,
    0x3e407000, 0x3e407000, 0x01b0701c, 0x01c0701c, 0x3fa01800, 0x3f901c00, 0x3fc04006, 0x3fd04407, 0x00000000, 0x00000000, 0x3fc01000, 0x3fc01000, 0x00000000, 0x00000000, 0x3fa01800, 0x3fa01800,
    0x3f602800, 0x3f702400, 0x0010480a, 0x00004809, 0x3e107c00, 0x3e107c00, 0x0210781f, 0x0200741f, 0x02e0a82c, 0x0280a829, 0x0300942a, 0x02f0942a, 0x0810bc58, 0x0820bc59, 0x03809c2f, 0x03809c2f,
    0x0180a821, 0x01e0a824, 0x02e0902a, 0x02f0942a, 0x3e307400, 0x3e307400, 0x01d0741d, 0x01e0741d, 0x3f901c00, 0x3f802000, 0x3fe04407, 0x3ff04408, 0x00000000, 0x00000000, 0x3fb01400, 0x3fb01400,

    /* 10-tap substream, 8:1 scaling ratio */
    0x00d02c0c, 0x00c0280b, 0x00b0280b, 0x00b0280a, 0x01b05418, 0x01b05418, 0x01605415, 0x01605015, 0x02407c22, 0x02407c22, 0x02007c1f, 0x02007c1f, 0x0250a426, 0x0250a027, 0x0280a429, 0x0280a428,
    0x00601405, 0x0240a427, 0x00501405, 0x0290a429, 0x01404012, 0x02508022, 0x01003c10, 0x02007c20, 0x02006c1d, 0x01b05818, 0x01b0681b, 0x01605416, 0x02509025, 0x00d02c0c, 0x02509024, 0x00b02c0b,
    0x0210b027, 0x00000000, 0x02b0b82c, 0x00000000, 0x00c0280b, 0x00b0280a, 0x00a0280a, 0x00a0240a, 0x01a05417, 0x01905017, 0x01505015, 0x01505014, 0x02407c22, 0x02407c21, 0x01f0781f, 0x01f0781f,
    0x0250a026, 0x0250a026, 0x0280a028, 0x0270a028, 0x0240a427, 0x0240a427, 0x0290a429, 0x0290a829, 0x02508022, 0x02508023, 0x02108020, 0x02108020, 0x01b05819, 0x01c05819, 0x01705416, 0x01705816,
    0x00d02c0c, 0x00e0300d, 0x00b02c0b, 0x00c02c0c, 0x00100401, 0x00100401, 0x00100401, 0x00100401, 0x00b0240a, 0x00a0240a, 0x00a02409, 0x00902409, 0x01905017, 0x01904c16, 0x01504c14, 0x01404c13,
    0x02407821, 0x02307821, 0x01f0781e, 0x01f0741e, 0x02509c26, 0x02509c26, 0x0270a027, 0x02709c27, 0x0240a827, 0x0230a826, 0x0290a82a, 0x0290a82a, 0x02508423, 0x02508423, 0x02108021, 0x02108421,
    0x01c05c19, 0x01d05c1a, 0x01705817, 0x01805817, 0x00e0300d, 0x00f0300d, 0x00c0300c, 0x00c0300c, 0x00100401, 0x00200802, 0x00100401, 0x00200802, 0x00a02009, 0x00a02009, 0x00902008, 0x00802008,
    0x01804c16, 0x01804c15, 0x01404c13, 0x01304813, 0x02307820, 0x02307420, 0x01e0741e, 0x01e0741d, 0x02509c26, 0x02509c26, 0x02709c27, 0x02709c27, 0x0230a827, 0x0230a827, 0x0290ac2a, 0x02a0ac2a,
    0x02508423, 0x02508823, 0x02208421, 0x02208422, 0x01d05c1a, 0x01d0601b, 0x01805817, 0x01805c18, 0x0100340e, 0x0100340e, 0x00d0300d, 0x00d0340d, 0x00200802, 0x00200802, 0x00200802, 0x00200802,

    /* 10-tap substream, 16:1 scaling ratio */
    0x00902008, 0x00901c08, 0x00802008, 0x00801c07, 0x01704c15, 0x01704814, 0x01304813, 0x01304812, 0x02207420, 0x0220741f, 0x01e0701d, 0x01d0701d, 0x02509826, 0x02509826, 0x02609826, 0x02609826,
    0x0230a826, 0x0220ac27, 0x02a0ac2a, 0x02a0b02b, 0x02508823, 0x02508824, 0x02208822, 0x02208822, 0x01e0601b, 0x01e0601b, 0x01805c18, 0x01905c18, 0x0100340f, 0x0110380f, 0x00e0340d, 0x00e0340e,
    0x00300c03, 0x00300c03, 0x00300c03, 0x00300c03, 0x00801c08, 0x00801c07, 0x00701c07, 0x00701c07, 0x01704814, 0x01604814, 0x01304412, 0x01204412, 0x0220701f, 0x0220701f, 0x01d0701c, 0x01d06c1c,
    0x02509826, 0x02509425, 0x02609826, 0x02609825, 0x0220ac27, 0x0220ac26, 0x02a0b02b, 0x02a0b02b, 0x02508824, 0x02508c24, 0x02308822, 0x02308823, 0x01e0641b, 0x01f0641c, 0x01906019, 0x01906019,
    0x0110380f, 0x01103810, 0x00e0380e, 0x00e0380e, 0x00400c03, 0x00401004, 0x00300c03, 0x00401004, 0x00801c07, 0x00701807, 0x00701807, 0x00601806, 0x01604413, 0x01504413, 0x01204411, 0x01104011,
    0x0210701f, 0x02106c1e, 0x01c06c1c, 0x01c06c1b, 0x02509425, 0x02509425, 0x02509425, 0x02509425, 0x0220ac27, 0x0220b027, 0x02a0b02b, 0x02b0b42c, 0x02508c24, 0x02508c24, 0x02308c23, 0x02408c23,
    0x01f0641c, 0x01f0681c, 0x01a06419, 0x01a0641a, 0x01203c10, 0x01203c11, 0x00f0380f, 0x00f0380f, 0x00401004, 0x00501004, 0x00401004, 0x00401004, 0x00701806, 0x00601406, 0x00601806, 0x00601405,
    0x01504412, 0x01404012, 0x01104011, 0x01104010, 0x02106c1e, 0x02106c1d, 0x01c0681b, 0x01b0681b, 0x02509425, 0x02509025, 0x02509425, 0x02409024, 0x0210b027, 0x0210b027, 0x02b0b42c, 0x02b0b42c,
    0x02508c24, 0x02509025, 0x02408c23, 0x02409024, 0x01f0681d, 0x0200681d, 0x01a0641a, 0x01b0681a, 0x01303c11, 0x01304011, 0x01003c0f, 0x01003c10, 0x00501005, 0x00601405, 0x00401404, 0x00501405,
};

static void nvtegra_spatialfilter_uninit(AVFilterContext *avctx) {
    NVTegraSpatialFilterContext *ctx = avctx->priv;

    if (ctx->has_filter_init)
        av_nvtegra_map_destroy(&ctx->filter_map);

    ff_nvtegra_vpp_ctx_uninit(avctx);
}

static int nvtegra_spatialfilter_init_filter_map(NVTegraSpatialFilterContext *ctx) {
    int err;

    err = av_nvtegra_map_create(&ctx->filter_map, ctx->core.pool.channel,
                                FFALIGN(sizeof(g_VicFilterData), 0x1000), 0x100,
                                NVMAP_HEAP_IOVMM, NVMAP_HANDLE_WRITE_COMBINE);
    if (err < 0)
        return err;

    memcpy(av_nvtegra_map_get_addr(&ctx->filter_map), g_VicFilterData, sizeof(g_VicFilterData));

    ctx->has_filter_init = true;

    return 0;
}

static int nvtegra_spatialfilter_prepare_config(NVTegraSpatialFilterContext *ctx, VicConfigStruct *config) {
    VicSlotStruct *slot = &config->slotStruct[0];

    int weight;

    if (ctx->sharpness != SHARPNESS_MIN) {
        weight = (int)(ctx->sharpness * 1023.0f + 0.5f);

        slot->slotConfig.FilterLengthX = ctx->filter_dimensions;
        slot->slotConfig.FilterLengthY = ctx->filter_dimensions;
        slot->slotConfig.FilterDetail  = weight;
        slot->slotConfig.ChromaDetail  = weight;
    }

    if (ctx->denoise != DENOISE_MIN) {
        weight = (int)(ctx->denoise * 1023.0f + 0.5f);

        slot->slotConfig.FilterLengthX = ctx->filter_dimensions;
        slot->slotConfig.FilterLengthY = ctx->filter_dimensions;
        slot->slotConfig.FilterNoise  = weight;
        slot->slotConfig.ChromaNoise  = weight;
    }

    return 0;
}

static int nvtegra_spatialfilter_prepare_cmdbuf(NVTegraSpatialFilterContext *ctx, AVNVTegraJobPool *pool,
                                                AVNVTegraJob *job, const AVFrame *in, const AVFrame *out)
{
    AVNVTegraCmdbuf *cmdbuf = &job->cmdbuf;

    const AVPixFmtDescriptor *input_desc, *output_desc;
    AVNVTegraMap *input_map, *output_map;
    int reloc_type, i, err;

    input_desc  = av_pix_fmt_desc_get(ctx->core.input_format);
    output_desc = av_pix_fmt_desc_get(ctx->core.output_format);

    input_map  = av_nvtegra_frame_get_fbuf_map(in);
    output_map = av_nvtegra_frame_get_fbuf_map(out);

    err = av_nvtegra_cmdbuf_begin(cmdbuf, HOST1X_CLASS_VIC);
    if (err < 0)
        return err;

    AV_NVTEGRA_PUSH_VALUE(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS,
                          AV_NVTEGRA_VALUE(NVB0B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS, CONFIG_STRUCT_SIZE, sizeof(VicConfigStruct) >> 4) |
                          AV_NVTEGRA_VALUE(NVB0B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS, GPTIMER_ON,         1)                            |
                          AV_NVTEGRA_VALUE(NVB0B6_VIDEO_COMPOSITOR_SET_CONTROL_PARAMS, FALCON_CONTROL,     1));
    AV_NVTEGRA_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_CONFIG_STRUCT_OFFSET,
                          &job->input_map,  ctx->core.vic_setup_off, NVHOST_RELOC_TYPE_DEFAULT);
    AV_NVTEGRA_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_FILTER_STRUCT_OFFSET,
                          &ctx->filter_map, 0,                       NVHOST_RELOC_TYPE_DEFAULT);

    reloc_type = !input_map->is_linear  ? NVHOST_RELOC_TYPE_BLOCK_LINEAR : NVHOST_RELOC_TYPE_PITCH_LINEAR;
    for (i = 0; i < input_desc->nb_components; ++i) {
        AV_NVTEGRA_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_SURFACE0_LUMA_OFFSET(0)    + i * sizeof(uint32_t),
                              input_map,  in->data[i]  - in->data[0],  reloc_type);
    }

    reloc_type = !output_map->is_linear ? NVHOST_RELOC_TYPE_BLOCK_LINEAR : NVHOST_RELOC_TYPE_PITCH_LINEAR;
    for (i = 0; i < output_desc->nb_components; ++i) {
        AV_NVTEGRA_PUSH_RELOC(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_SET_OUTPUT_SURFACE_LUMA_OFFSET + i * sizeof(uint32_t),
                              output_map, out->data[i] - out->data[0], reloc_type);
    }

    AV_NVTEGRA_PUSH_VALUE(cmdbuf, NVB0B6_VIDEO_COMPOSITOR_EXECUTE,
                          AV_NVTEGRA_ENUM(NVB0B6_VIDEO_COMPOSITOR_EXECUTE, AWAKEN, ENABLE));

    err = av_nvtegra_cmdbuf_begin(cmdbuf, HOST1X_CLASS_VIC);
    if (err < 0)
        return err;

    err = av_nvtegra_cmdbuf_add_syncpt_incr(cmdbuf, pool->channel->syncpt, 0);
    if (err < 0)
        return err;

    err = av_nvtegra_cmdbuf_end(cmdbuf);
    if (err < 0)
        return err;

    return 0;
}

static int nvtegra_spatialfilter_filter_frame(AVFilterLink *link, AVFrame *in) {
    AVFilterContext           *avctx = link->dst;
    NVTegraSpatialFilterContext *ctx = avctx->priv;
    AVFilterLink            *outlink = avctx->outputs[0];

    AVBufferRef *job_ref;
    AVNVTegraJob *job;
    AVFrame *out = NULL;
    VicConfigStruct *config;
    int err;

    if (!ctx->has_filter_init) {
        err = nvtegra_spatialfilter_init_filter_map(ctx);
        if (err < 0)
            return err;
    }

    job_ref = av_nvtegra_job_pool_get(&ctx->core.pool);
    if (!job_ref) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    job    = (AVNVTegraJob *)job_ref->data;
    config = (VicConfigStruct *)((uint8_t *)av_nvtegra_map_get_addr(&job->input_map) + ctx->core.vic_setup_off);

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    err = av_frame_copy_props(out, in);
    if (err < 0)
        goto fail;

    err = ff_nvtegra_vpp_init_config(&ctx->core, config, out, &in, 1);
    if (err < 0)
        goto fail;

    err = nvtegra_spatialfilter_prepare_config(ctx, config);
    if (err < 0)
        goto fail;

    err = av_nvtegra_cmdbuf_clear(&job->cmdbuf);
    if (err < 0)
        return err;

    err = nvtegra_spatialfilter_prepare_cmdbuf(ctx, &ctx->core.pool, job, in, out);
    if (err < 0)
        goto fail;

    err = av_nvtegra_job_submit(&ctx->core.pool, job);
    if (err < 0)
        goto fail;

    err = av_nvtegra_job_wait(&ctx->core.pool, job, -1);
    if (err < 0)
        goto fail;

    av_buffer_unref(&job_ref);

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);

fail:
    av_buffer_unref(&job_ref);
    av_frame_free(&in);
    av_frame_free(&out);
    return err;
}

#define SOFFSET(x) offsetof(NVTegraSpatialFilterContext, x)
#define FLAGS (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_RUNTIME_PARAM | AV_OPT_FLAG_FILTERING_PARAM)

static const AVOption nvtegra_sharpness_options[] = {
    { "sharpness", "sharpening level",
      SOFFSET(sharpness), AV_OPT_TYPE_FLOAT, { .dbl = SHARPNESS_DEFAULT }, SHARPNESS_MIN, SHARPNESS_MAX, .flags = FLAGS },

    { "dimensions", "Filter dimensions", SOFFSET(filter_dimensions), AV_OPT_TYPE_INT,
      { .i64 = FILTER_TAP_DEFAULT }, FILTER_TAP_MIN, FILTER_TAP_MAX, FLAGS, "dimensions" },
    { "5tap",  "5-tap filtering",  0, AV_OPT_TYPE_CONST,
      { .i64 = NVB0B6_FILTER_LENGTH_5TAP },  0, 0, FLAGS, "dimensions" },
    { "10tap", "10-tap filtering", 0, AV_OPT_TYPE_CONST,
      { .i64 = NVB0B6_FILTER_LENGTH_10TAP }, 0, 0, FLAGS, "dimensions" },

    { NULL },
};

static const AVOption nvtegra_denoise_options[] = {
    { "denoise", "denoising level",
      SOFFSET(denoise), AV_OPT_TYPE_FLOAT, { .dbl = DENOISE_DEFAULT }, DENOISE_MIN, DENOISE_MAX, .flags = FLAGS },

    { "dimensions", "Filter dimensions", SOFFSET(filter_dimensions), AV_OPT_TYPE_INT,
      { .i64 = FILTER_TAP_DEFAULT }, FILTER_TAP_MIN, FILTER_TAP_MAX, FLAGS, "dimensions" },
    { "5tap",  "5-tap filtering",  0, AV_OPT_TYPE_CONST,
      { .i64 = NVB0B6_FILTER_LENGTH_5TAP },  0, 0, FLAGS, "dimensions" },
    { "10tap", "10-tap filtering", 0, AV_OPT_TYPE_CONST,
      { .i64 = NVB0B6_FILTER_LENGTH_10TAP }, 0, 0, FLAGS, "dimensions" },

    { NULL },
};

AVFILTER_DEFINE_CLASS(nvtegra_sharpness);
AVFILTER_DEFINE_CLASS(nvtegra_denoise);

static const AVFilterPad nvtegra_spatialfilter_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = &nvtegra_spatialfilter_filter_frame,
        .config_props = &ff_nvtegra_vpp_config_input,
    },
};

static const AVFilterPad nvtegra_spatialfilter_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = &ff_nvtegra_vpp_config_output,
    },
};

#if CONFIG_SHARPNESS_NVTEGRA_FILTER
const AVFilter ff_vf_sharpness_nvtegra = {
    .name           = "sharpness_nvtegra",
    .description    = NULL_IF_CONFIG_SMALL("NVTegra accelerated sharpening"),
    .priv_size      = sizeof(NVTegraSpatialFilterContext),
    .init           = &ff_nvtegra_vpp_ctx_init,
    .uninit         = &ff_nvtegra_vpp_ctx_uninit,
    FILTER_INPUTS(nvtegra_spatialfilter_inputs),
    FILTER_OUTPUTS(nvtegra_spatialfilter_outputs),
    FILTER_SINGLE_PIXFMT(AV_PIX_FMT_NVTEGRA),
    .priv_class     = &nvtegra_sharpness_class,
    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
};
#endif

#if CONFIG_DENOISE_NVTEGRA_FILTER
const AVFilter ff_vf_denoise_nvtegra = {
    .name           = "denoise_nvtegra",
    .description    = NULL_IF_CONFIG_SMALL("NVTegra accelerated denoising"),
    .priv_size      = sizeof(NVTegraSpatialFilterContext),
    .init           = &ff_nvtegra_vpp_ctx_init,
    .uninit         = &ff_nvtegra_vpp_ctx_uninit,
    FILTER_INPUTS(nvtegra_spatialfilter_inputs),
    FILTER_OUTPUTS(nvtegra_spatialfilter_outputs),
    FILTER_SINGLE_PIXFMT(AV_PIX_FMT_NVTEGRA),
    .priv_class     = &nvtegra_denoise_class,
    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
};
#endif
