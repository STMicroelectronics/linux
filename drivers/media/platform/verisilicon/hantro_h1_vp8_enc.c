// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics SA 2023
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          for STMicroelectronics.
 *
 * Copyright (C) 2023 Collabora Ltd.
 *	Andrzej Pietrasiewicz <andrzej.p@collabora.com>
 *
 * This work is derived from:
 *
 * https://chromium.googlesource.com/chromiumos/third_party/kernel/+/refs/heads/chromeos-5.10/
 * drivers/media/platform/verisilicon/rk3399_vpu_hw_vp8_enc.c
 *
 * which is:
 *
 * Rockchip VPU codec driver
 *
 * Copyright (C) 2016 Rockchip Electronics Co., Ltd.
 *	Alpha Lin <Alpha.Lin@rock-chips.com>
 *
 * Substantial portions are based on MIT-licensed code
 *
 * https://github.com/rockchip-linux/mpp/blob/develop/mpp/hal/vpu/vp8e/hal_vp8e_entropy.{c|h}
 * https://github.com/rockchip-linux/mpp/blob/develop/mpp/hal/vpu/vp8e/hal_vp8e_table.{c|h}
 *
 * which is:
 *
 * Copyright (c) 2017 Rockchip Electronics Co., Ltd.
 *
 */
#include <linux/minmax.h>

#include "hantro.h"
#include "hantro_hw.h"
#include "hantro_h1_regs.h"

/* threshold of MBs count to disable quarter pixel mv for encode speed */
#define MAX_MB_COUNT_TO_DISABLE_QUARTER_PIXEL_MV	3600

/* threshold of MBs count to disable multi mv in one macro block */
#define MAX_MB_COUNT_TO_DISABLE_SPLIT_MV		1584

#define QINDEX_RANGE 128
#define VP8_PROB_COUNT_MV_OFFSET 222

#define MODE_DELTA_BPRED	0
#define MODE_DELTA_ZEROMV	1
#define MODE_DELTA_NEWMV	2
#define MODE_DELTA_SPLITMV	3

#define REF_DELTA_INTRA_MB	0
#define REF_DELTA_LAST_REF	1
#define REF_DELTA_GOLDEN	2
#define REF_DELTA_ALT_REF	3

/* experimentally fitted, 24.893*exp(0.02545*qp) */
static const s32 split_penalty[QINDEX_RANGE] = {
	24, 25, 26, 26, 27, 28, 29, 29,
	30, 31, 32, 32, 33, 34, 35, 36,
	37, 38, 39, 40, 41, 42, 43, 44,
	45, 47, 48, 49, 50, 52, 53, 54,
	56, 57, 59, 60, 62, 63, 65, 67,
	68, 70, 72, 74, 76, 78, 80, 82,
	84, 86, 88, 91, 93, 95, 98, 100,
	103, 106, 108, 111, 114, 117, 120, 123,
	126, 130, 133, 136, 140, 144, 147, 151,
	155, 159, 163, 167, 172, 176, 181, 185,
	190, 195, 200, 205, 211, 216, 222, 227,
	233, 239, 245, 252, 258, 265, 272, 279,
	286, 293, 301, 309, 317, 325, 333, 342,
	351, 360, 369, 379, 388, 398, 409, 419,
	430, 441, 453, 464, 476, 488, 501, 514,
	527, 541, 555, 569, 584, 599, 614, 630
};

static const int dc_q_lookup[QINDEX_RANGE] = {
	4,   5,   6,   7,   8,   9,   10,  10,  11,  12,
	13,  14,  15,  16,  17,  17,  18,  19,  20,  20,
	21,  21,  22,  22,  23,  23,  24,  25,  25,  26,
	27,  28,  29,  30,  31,  32,  33,  34,  35,  36,
	37,  37,  38,  39,  40,  41,  42,  43,  44,  45,
	46,  46,  47,  48,  49,  50,  51,  52,  53,  54,
	55,  56,  57,  58,  59,  60,  61,  62,  63,  64,
	65,  66,  67,  68,  69,  70,  71,  72,  73,  74,
	75,  76,  76,  77,  78,  79,  80,  81,  82,  83,
	84,  85,  86,  87,  88,  89,  91,  93,  95,  96,
	98,  100, 101, 102, 104, 106, 108, 110, 112, 114,
	116, 118, 122, 124, 126, 128, 130, 132, 134, 136,
	138, 140, 143, 145, 148, 151, 154, 157
};

static const int ac_q_lookup[QINDEX_RANGE] = {
	4,   5,   6,   7,   8,   9,   10,  11,  12,  13,
	14,  15,  16,  17,  18,  19,  20,  21,  22,  23,
	24,  25,  26,  27,  28,  29,  30,  31,  32,  33,
	34,  35,  36,  37,  38,  39,  40,  41,  42,  43,
	44,  45,  46,  47,  48,  49,  50,  51,  52,  53,
	54,  55,  56,  57,  58,  60,  62,  64,  66,  68,
	70,  72,  74,  76,  78,  80,  82,  84,  86,  88,
	90,  92,  94,  96,  98,  100, 102, 104, 106, 108,
	110, 112, 114, 116, 119, 122, 125, 128, 131, 134,
	137, 140, 143, 146, 149, 152, 155, 158, 161, 164,
	167, 170, 173, 177, 181, 185, 189, 193, 197, 201,
	205, 209, 213, 217, 221, 225, 229, 234, 239, 245,
	249, 254, 259, 264, 269, 274, 279, 284
};

static const s32 qrounding_factors[QINDEX_RANGE] = {
	56, 56, 56, 56, 56, 56, 56, 56, 48, 48,
	48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
	48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
	48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
	48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
	48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
	48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
	48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
	48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
	48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
	48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
	48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
	48, 48, 48, 48, 48, 48, 48, 48
};

static const s32 qzbin_factors[QINDEX_RANGE] = {
	64, 64, 64, 64, 80, 80, 80, 80, 80, 80,
	80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
	80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
	80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
	80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
	80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
	80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
	80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
	80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
	80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
	80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
	80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
	80, 80, 80, 80, 80, 80, 80, 80
};

/* Intra 16x16 mode tree penalty values */
static s32 intra_16_tree_penalty[] = {
	305, 841, 914, 1082
};

/* Intra 4x4 mode tree penalty values */
static s32 intra_4_tree_penalty[] = {
	280, 622, 832, 1177, 1240, 1341, 1085, 1259, 1357, 1495
};


static int32_t const offset_tbl[] = {
	-1, -1, -1,  0,  1,  2, -1,  3,  4, -1,  5,  6, -1,  7,  8, -1,
	9, 10, -1, 11, 12, 13, 14, 15, -1, 16, 17, -1, 18, 19, -1, 20,
	21, -1, 22, 23, -1, 24, 25, -1, 26, 27, 28, 29, 30, -1, 31, 32,
	-1, 33, 34, -1, 35, 36, -1, 37, 38, -1, 39, 40, -1, 41, 42, 43,
	44, 45, -1, 46, 47, -1, 48, 49, -1, 50, 51, -1, 52, 53, -1, 54,
	55, -1, 56, 57, -1, -1, -1, 58, 59, 60, 61, 62, 63, 64, 65, 66,
	67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82,
	83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98,
	99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114,
	115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130,
	131, 132, 133, 134, 135, 136, 137, 138, -1, -1, -1, 139, 140, 141, 142, 143,
	144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
	160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175,
	176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191,
	192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207,
	208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219
};

/* ~round((2*(2+exp((x+22)/39)) + (2+exp((x+15)/32)))/3) */
static s32 weight[QINDEX_RANGE] = {
	4,  4,  4,  4,  4,  4,  4,  4,  4,  4,
	4,  4,  4,  4,  4,  5,  5,  5,  5,  5,
	5,  5,  5,  5,  5,  5,  5,  6,  6,  6,
	6,  6,  6,  6,  6,  6,  6,  7,  7,  7,
	7,  7,  7,  7,  7,  8,  8,  8,  8,  8,
	8,  8,  9,  9,  9,  9,  9, 10, 10, 10,
	10, 11, 11, 11, 12, 12, 13, 13, 13, 13,
	14, 14, 14, 14, 15, 15, 15, 16, 16, 17,
	17, 18, 18, 19, 19, 20, 20, 20, 21, 22,
	23, 23, 24, 24, 25, 25, 26, 27, 28, 28,
	29, 30, 31, 32, 32, 33, 34, 35, 36, 37,
	38, 39, 40, 41, 42, 44, 44, 46, 47, 48,
	50, 51, 52, 54, 55, 57, 58, 61
};

struct tree {
	s32 value;       /* Bits describe the bool tree  */
	s32 number;      /* Number, valid bit count in above tree */
	s32 index[9];    /* Probability table index */
};

/* Motion vector tree */
static struct tree mv_tree[] = {
	{ 0, 3, { 0, 1, 2 } },        /* mv_0 000 */
	{ 1, 3, { 0, 1, 2 } },        /* mv_1 001 */
	{ 2, 3, { 0, 1, 3 } },        /* mv_2 010 */
	{ 3, 3, { 0, 1, 3 } },        /* mv_3 011 */
	{ 4, 3, { 0, 4, 5 } },        /* mv_4 100 */
	{ 5, 3, { 0, 4, 5 } },        /* mv_5 101 */
	{ 6, 3, { 0, 4, 6 } },        /* mv_6 110 */
	{ 7, 3, { 0, 4, 6 } },        /* mv_7 111 */
};

/* If probability being zero is p, then average number of bits used to encode 0
 * is log2(1/p), to encode 1 is log2(1/(1-p)).
 *
 * For example, if the probability of being zero is 0.5
 * bin = 0 -> average bits used is log2(1/0.5)      = 1 bits/bin
 * bin = 1 -> average bits used is log2(1/(1 - 0.5) = 1 bits/bin
 *
 * For example, if the probability of being zero is 0.95
 * bin = 0 -> average bits used is log2(1/0.95)      = 0.074 bits/bin
 * bin = 1 -> average bits used is log2(1/(1 - 0.95) = 4.321 bits/bin
 *
 * The cost[p] is average number of bits used to encode 0 if the probability is
 * p / 256, scaled by a magic number 256,
 * i.e., cost[p] = round(log2(256 / p) * 256).
 */
static const s32 vp8_prob_cost[] = {
	2048, 2048, 1792, 1642, 1536, 1454, 1386, 1329, 1280, 1236,
	1198, 1162, 1130, 1101, 1073, 1048, 1024, 1002,  980,  961,
	942,  924,  906,  890,  874,  859,  845,  831,  817,  804,
	792,  780,  768,  757,  746,  735,  724,  714,  705,  695,
	686,  676,  668,  659,  650,  642,  634,  626,  618,  611,
	603,  596,  589,  582,  575,  568,  561,  555,  548,  542,
	536,  530,  524,  518,  512,  506,  501,  495,  490,  484,
	479,  474,  468,  463,  458,  453,  449,  444,  439,  434,
	430,  425,  420,  416,  412,  407,  403,  399,  394,  390,
	386,  382,  378,  374,  370,  366,  362,  358,  355,  351,
	347,  343,  340,  336,  333,  329,  326,  322,  319,  315,
	312,  309,  305,  302,  299,  296,  292,  289,  286,  283,
	280,  277,  274,  271,  268,  265,  262,  259,  256,  253,
	250,  247,  245,  242,  239,  236,  234,  231,  228,  226,
	223,  220,  218,  215,  212,  210,  207,  205,  202,  200,
	197,  195,  193,  190,  188,  185,  183,  181,  178,  176,
	174,  171,  169,  167,  164,  162,  160,  158,  156,  153,
	151,  149,  147,  145,  143,  140,  138,  136,  134,  132,
	130,  128,  126,  124,  122,  120,  118,  116,  114,  112,
	110,  108,  106,  104,  102,  101,   99,   97,   95,   93,
	91,   89,   87,   86,   84,   82,   80,   78,   77,   75,
	73,   71,   70,   68,   66,   64,   63,   61,   59,   58,
	56,   54,   53,   51,   49,   48,   46,   44,   43,   41,
	40,   38,   36,   35,   33,   32,   30,   28,   27,   25,
	24,   22,   21,   19,   18,   16,   15,   13,   12,   10,
	9,    7,    6,    4,    3,    1
};

/* Approximate bit cost of bin at given probability prob */
#define COST_BOOL(prob, bin)	vp8_prob_cost[(bin) ? 255 - (prob) : (prob)]

static uint32_t const coeff_update_prob[4][8][3][11] = {
	{
		{
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{176, 246, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{223, 241, 252, 255, 255, 255, 255, 255, 255, 255, 255},
			{249, 253, 253, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 244, 252, 255, 255, 255, 255, 255, 255, 255, 255},
			{234, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{253, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 246, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{239, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{254, 255, 254, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 248, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{251, 255, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{251, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{254, 255, 254, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 254, 253, 255, 254, 255, 255, 255, 255, 255, 255},
			{250, 255, 254, 255, 254, 255, 255, 255, 255, 255, 255},
			{254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		},
	}, {
		{
			{217, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{225, 252, 241, 253, 255, 255, 254, 255, 255, 255, 255},
			{234, 250, 241, 250, 253, 255, 253, 254, 255, 255, 255},
		}, {
			{255, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{223, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{238, 253, 254, 254, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 248, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{249, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 253, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{247, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{252, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{253, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 254, 253, 255, 255, 255, 255, 255, 255, 255, 255},
			{250, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		},
	}, {
		{
			{186, 251, 250, 255, 255, 255, 255, 255, 255, 255, 255},
			{234, 251, 244, 254, 255, 255, 255, 255, 255, 255, 255},
			{251, 251, 243, 253, 254, 255, 254, 255, 255, 255, 255},
		}, {
			{255, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{236, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{251, 253, 253, 254, 254, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		},
	}, {
		{
			{248, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{250, 254, 252, 254, 255, 255, 255, 255, 255, 255, 255},
			{248, 254, 249, 253, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 253, 253, 255, 255, 255, 255, 255, 255, 255, 255},
			{246, 253, 253, 255, 255, 255, 255, 255, 255, 255, 255},
			{252, 254, 251, 254, 254, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 254, 252, 255, 255, 255, 255, 255, 255, 255, 255},
			{248, 254, 253, 255, 255, 255, 255, 255, 255, 255, 255},
			{253, 255, 254, 254, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 251, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{245, 251, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{253, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 251, 253, 255, 255, 255, 255, 255, 255, 255, 255},
			{252, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 252, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{249, 255, 254, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 254, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 255, 253, 255, 255, 255, 255, 255, 255, 255, 255},
			{250, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		}, {
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
		},
	},
};

static uint32_t const mv_update_prob[2][19] = {
	{
		237, 246, 253, 253, 254, 254, 254, 254, 254, 254,
		254, 254, 254, 254, 250, 250, 252, 254, 254
	}, {
		231, 243, 245, 253, 254, 254, 254, 254, 254, 254,
		254, 254, 254, 254, 251, 251, 254, 254, 254
	}
};

static uint8_t const default_prob_skip_false[128] = {
	255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255,
	251, 248, 244, 240, 236, 232, 229, 225,
	221, 217, 213, 208, 204, 199, 194, 190,
	187, 183, 179, 175, 172, 168, 164, 160,
	157, 153, 149, 145, 142, 138, 134, 130,
	127, 124, 120, 117, 114, 110, 107, 104,
	101, 98,  95,  92,  89,  86,  83, 80,
	77,  74,  71,  68,  65,  62,  59, 56,
	53,  50,  47,  44,  41,  38,  35, 32,
	30,  28,  26,  24,  22,  20,  18, 16,
};

static int32_t const y_mode_prob[4] = {
	112, 86, 140, 37
};

static int32_t const uv_mode_prob[3] = {
	162, 101, 204
};

static uint32_t const default_prob_coeff[4][8][3][11] = {
	{
		{
			{128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128},
			{128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128},
			{128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128},
		}, {
			{253, 136, 254, 255, 228, 219, 128, 128, 128, 128, 128},
			{189, 129, 242, 255, 227, 213, 255, 219, 128, 128, 128},
			{106, 126, 227, 252, 214, 209, 255, 255, 128, 128, 128},
		}, {
			{  1,  98, 248, 255, 236, 226, 255, 255, 128, 128, 128},
			{181, 133, 238, 254, 221, 234, 255, 154, 128, 128, 128},
			{ 78, 134, 202, 247, 198, 180, 255, 219, 128, 128, 128},
		}, {
			{  1, 185, 249, 255, 243, 255, 128, 128, 128, 128, 128},
			{184, 150, 247, 255, 236, 224, 128, 128, 128, 128, 128},
			{ 77, 110, 216, 255, 236, 230, 128, 128, 128, 128, 128},
		}, {
			{  1, 101, 251, 255, 241, 255, 128, 128, 128, 128, 128},
			{170, 139, 241, 252, 236, 209, 255, 255, 128, 128, 128},
			{ 37, 116, 196, 243, 228, 255, 255, 255, 128, 128, 128},
		}, {
			{  1, 204, 254, 255, 245, 255, 128, 128, 128, 128, 128},
			{207, 160, 250, 255, 238, 128, 128, 128, 128, 128, 128},
			{102, 103, 231, 255, 211, 171, 128, 128, 128, 128, 128},
		}, {
			{  1, 152, 252, 255, 240, 255, 128, 128, 128, 128, 128},
			{177, 135, 243, 255, 234, 225, 128, 128, 128, 128, 128},
			{ 80, 129, 211, 255, 194, 224, 128, 128, 128, 128, 128},
		}, {
			{  1,   1, 255, 128, 128, 128, 128, 128, 128, 128, 128},
			{246,   1, 255, 128, 128, 128, 128, 128, 128, 128, 128},
			{255, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128},
		},
	}, {
		{
			{198,  35, 237, 223, 193, 187, 162, 160, 145, 155,  62},
			{131,  45, 198, 221, 172, 176, 220, 157, 252, 221,   1},
			{ 68,  47, 146, 208, 149, 167, 221, 162, 255, 223, 128},
		}, {
			{  1, 149, 241, 255, 221, 224, 255, 255, 128, 128, 128},
			{184, 141, 234, 253, 222, 220, 255, 199, 128, 128, 128},
			{ 81,  99, 181, 242, 176, 190, 249, 202, 255, 255, 128},
		}, {
			{  1, 129, 232, 253, 214, 197, 242, 196, 255, 255, 128},
			{ 99, 121, 210, 250, 201, 198, 255, 202, 128, 128, 128},
			{ 23,  91, 163, 242, 170, 187, 247, 210, 255, 255, 128},
		}, {
			{  1, 200, 246, 255, 234, 255, 128, 128, 128, 128, 128},
			{109, 178, 241, 255, 231, 245, 255, 255, 128, 128, 128},
			{ 44, 130, 201, 253, 205, 192, 255, 255, 128, 128, 128},
		}, {
			{  1, 132, 239, 251, 219, 209, 255, 165, 128, 128, 128},
			{ 94, 136, 225, 251, 218, 190, 255, 255, 128, 128, 128},
			{ 22, 100, 174, 245, 186, 161, 255, 199, 128, 128, 128},
		}, {
			{  1, 182, 249, 255, 232, 235, 128, 128, 128, 128, 128},
			{124, 143, 241, 255, 227, 234, 128, 128, 128, 128, 128},
			{ 35,  77, 181, 251, 193, 211, 255, 205, 128, 128, 128},
		}, {
			{  1, 157, 247, 255, 236, 231, 255, 255, 128, 128, 128},
			{121, 141, 235, 255, 225, 227, 255, 255, 128, 128, 128},
			{ 45,  99, 188, 251, 195, 217, 255, 224, 128, 128, 128},
		}, {
			{  1,   1, 251, 255, 213, 255, 128, 128, 128, 128, 128},
			{203,   1, 248, 255, 255, 128, 128, 128, 128, 128, 128},
			{137,   1, 177, 255, 224, 255, 128, 128, 128, 128, 128},
		},
	}, {
		{
			{253,   9, 248, 251, 207, 208, 255, 192, 128, 128, 128},
			{175,  13, 224, 243, 193, 185, 249, 198, 255, 255, 128},
			{ 73,  17, 171, 221, 161, 179, 236, 167, 255, 234, 128},
		}, {
			{  1,  95, 247, 253, 212, 183, 255, 255, 128, 128, 128},
			{239,  90, 244, 250, 211, 209, 255, 255, 128, 128, 128},
			{155,  77, 195, 248, 188, 195, 255, 255, 128, 128, 128},
		}, {
			{  1,  24, 239, 251, 218, 219, 255, 205, 128, 128, 128},
			{201,  51, 219, 255, 196, 186, 128, 128, 128, 128, 128},
			{ 69,  46, 190, 239, 201, 218, 255, 228, 128, 128, 128},
		}, {
			{  1, 191, 251, 255, 255, 128, 128, 128, 128, 128, 128},
			{223, 165, 249, 255, 213, 255, 128, 128, 128, 128, 128},
			{141, 124, 248, 255, 255, 128, 128, 128, 128, 128, 128},
		}, {
			{  1,  16, 248, 255, 255, 128, 128, 128, 128, 128, 128},
			{190,  36, 230, 255, 236, 255, 128, 128, 128, 128, 128},
			{149,   1, 255, 128, 128, 128, 128, 128, 128, 128, 128},
		}, {
			{  1, 226, 255, 128, 128, 128, 128, 128, 128, 128, 128},
			{247, 192, 255, 128, 128, 128, 128, 128, 128, 128, 128},
			{240, 128, 255, 128, 128, 128, 128, 128, 128, 128, 128},
		}, {
			{  1, 134, 252, 255, 255, 128, 128, 128, 128, 128, 128},
			{213,  62, 250, 255, 255, 128, 128, 128, 128, 128, 128},
			{ 55,  93, 255, 128, 128, 128, 128, 128, 128, 128, 128},
		}, {
			{128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128},
			{128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128},
			{128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128},
		},
	}, {
		{
			{202,  24, 213, 235, 186, 191, 220, 160, 240, 175, 255},
			{126,  38, 182, 232, 169, 184, 228, 174, 255, 187, 128},
			{ 61,  46, 138, 219, 151, 178, 240, 170, 255, 216, 128},
		}, {
			{  1, 112, 230, 250, 199, 191, 247, 159, 255, 255, 128},
			{166, 109, 228, 252, 211, 215, 255, 174, 128, 128, 128},
			{ 39,  77, 162, 232, 172, 180, 245, 178, 255, 255, 128},
		}, {
			{  1,  52, 220, 246, 198, 199, 249, 220, 255, 255, 128},
			{124,  74, 191, 243, 183, 193, 250, 221, 255, 255, 128},
			{ 24,  71, 130, 219, 154, 170, 243, 182, 255, 255, 128},
		}, {
			{  1, 182, 225, 249, 219, 240, 255, 224, 128, 128, 128},
			{149, 150, 226, 252, 216, 205, 255, 171, 128, 128, 128},
			{ 28, 108, 170, 242, 183, 194, 254, 223, 255, 255, 128},
		}, {
			{  1,  81, 230, 252, 204, 203, 255, 192, 128, 128, 128},
			{123, 102, 209, 247, 188, 196, 255, 233, 128, 128, 128},
			{ 20,  95, 153, 243, 164, 173, 255, 203, 128, 128, 128},
		}, {
			{  1, 222, 248, 255, 216, 213, 128, 128, 128, 128, 128},
			{168, 175, 246, 252, 235, 205, 255, 255, 128, 128, 128},
			{ 47, 116, 215, 255, 211, 212, 255, 255, 128, 128, 128},
		}, {
			{  1, 121, 236, 253, 212, 214, 255, 255, 128, 128, 128},
			{141,  84, 213, 252, 201, 202, 255, 219, 128, 128, 128},
			{ 42,  80, 160, 240, 162, 185, 255, 205, 128, 128, 128},
		}, {
			{  1,   1, 255, 128, 128, 128, 128, 128, 128, 128, 128},
			{244,   1, 255, 128, 128, 128, 128, 128, 128, 128, 128},
			{238,   1, 255, 128, 128, 128, 128, 128, 128, 128, 128},
		},
	}
};

static uint32_t const default_prob_mv[2][19] = {
	{
		162, 128, 225, 146, 172, 147, 214,  39, 156, 128,
		129, 132,  75, 145, 178, 206, 239, 254, 254,
	}, {
		164, 128, 204, 170, 119, 235, 140, 230, 228, 128,
		130, 130,  74, 148, 180, 203, 236, 254, 254,
	},
};

struct hantro_vp8_enc_ctrl_buf {
	u32 hw_hdr_size;
	u32 dct_size;
	u8 reserved[1016];
};

static inline bool is_intra(struct hantro_ctx *ctx)
{
	return ctx->vp8_enc.is_intra;
}

static inline size_t frame_tag_size(struct hantro_ctx *ctx)
{
	return is_intra(ctx) ? 10 : 3;
}

static void hantro_h1_vp8_enc_write_coeff_prob(struct hantro_ctx *ctx)
{
	struct hantro_boolenc *e = &ctx->vp8_enc.boolenc;
	struct hantro_vp8_entropy *entropy = &ctx->vp8_enc.entropy;

	int i, j, k, l;
	uint32_t prob, new, old;

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 8; j++) {
			for (k = 0; k < 3; k++) {
				for (l = 0; l < 11; l++) {
					prob = coeff_update_prob[i][j][k][l];

					old = entropy->coeff_prob_old[i][j][k][l];
					new = entropy->coeff_prob[i][j][k][l];

					if (new == old) {
						hantro_boolenc_write_bool(e, prob, 0);
					} else {
						hantro_boolenc_write_bool(e, prob, 1);
						hantro_boolenc_write_lit(e, new, 8);
					}
				}
			}
		}
	}
}

static void hantro_h1_vp8_enc_write_mv_prob(struct hantro_ctx *ctx)
{
	struct hantro_boolenc *e = &ctx->vp8_enc.boolenc;
	struct hantro_vp8_entropy *entropy = &ctx->vp8_enc.entropy;

	int i, j;
	uint32_t prob, new, old;

	for (i = 0; i < 2; i++) {
		for (j = 0; j < 19; j++) {
			prob = mv_update_prob[i][j];
			old = entropy->mv_prob_old[i][j];
			new = entropy->mv_prob[i][j];

			if (new == old) {
				hantro_boolenc_write_bool(e, prob, 0);
			} else {
				hantro_boolenc_write_bool(e, prob, 1);
				hantro_boolenc_write_lit(e, new >> 1, 7);
			}
		}
	}
}

static void hantro_h1_vp8_enc_write_filter_level_delta(struct hantro_ctx *ctx)
{
	struct hantro_boolenc *e = &ctx->vp8_enc.boolenc;
	int32_t i, tmp;
	uint8_t  update = 0;
	int32_t mode_update[4];
	int32_t ref_update[4];

	if (is_intra(ctx)) {
		memset(ctx->vp8_enc.old_mode_delta, 0, sizeof(ctx->vp8_enc.old_mode_delta));
		memset(ctx->vp8_enc.old_ref_delta, 0, sizeof(ctx->vp8_enc.old_ref_delta));
	}

	for (i = 0; i < 4; i++) {
		mode_update[i] = ctx->vp8_enc.mode_delta[i] != ctx->vp8_enc.old_mode_delta[i];
		ref_update[i] = ctx->vp8_enc.ref_delta[i] != ctx->vp8_enc.old_ref_delta[i];
		if (mode_update[i] || ref_update[i])
			update = 1;
	}

	if (!ctx->vp8_enc.refresh_entropy_probs)
		update = 1;

	hantro_boolenc_write_lit(e, update, 1);
	if (!update)
		return;

	for (i = 0; i < 4; i++) {
		hantro_boolenc_write_lit(e, ref_update[i], 1);
		if (ref_update[i]) {
			tmp = ctx->vp8_enc.ref_delta[i];
			hantro_boolenc_write_lit(e, abs(tmp), 6);
			hantro_boolenc_write_lit(e, tmp < 0, 1);
		}
	}

	for (i = 0; i < 4; i++) {
		hantro_boolenc_write_lit(e, mode_update[i], 1);
		if (mode_update[i]) {
			tmp = ctx->vp8_enc.mode_delta[i];
			hantro_boolenc_write_lit(e, abs(tmp), 6);
			hantro_boolenc_write_lit(e, tmp < 0, 1);
		}
	}

	memcpy(ctx->vp8_enc.old_mode_delta, ctx->vp8_enc.mode_delta,
	       sizeof(ctx->vp8_enc.old_mode_delta));
	memcpy(ctx->vp8_enc.old_ref_delta, ctx->vp8_enc.ref_delta,
	       sizeof(ctx->vp8_enc.old_ref_delta));
}

static void hantro_h1_vp8_enc_write_header(struct hantro_ctx *ctx, u32 qp,
					       struct v4l2_ctrl_vp8_encode_params *params)
{
	struct hantro_boolenc *e = &ctx->vp8_enc.boolenc;
	struct vb2_v4l2_buffer *dst_buf = hantro_get_dst_buf(ctx);
	void *vaddr = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);

	ctx->vp8_enc.frame_tag = vaddr;
	memset(ctx->vp8_enc.frame_tag, 0, sizeof(*ctx->vp8_enc.frame_tag));
	ctx->vp8_enc.frame_tag->tag[0] |= (1 << 1);	/* version 1 */
	if (ctx->vp8_enc.show_frame)
		ctx->vp8_enc.frame_tag->tag[0] |= (1 << 4);
	/* first partition size is known only after encoding */

	if (is_intra(ctx)) {
		ctx->vp8_enc.frame_tag->magic[0] = 0x9d;
		ctx->vp8_enc.frame_tag->magic[1] = 0x01;
		ctx->vp8_enc.frame_tag->magic[2] = 0x2a;

		ctx->vp8_enc.frame_tag->width[0] = ctx->dst_fmt.width & 0xff;
		ctx->vp8_enc.frame_tag->width[1] = (ctx->dst_fmt.width >> 8) & 0xff;

		ctx->vp8_enc.frame_tag->height[0] = ctx->dst_fmt.height & 0xff;
		ctx->vp8_enc.frame_tag->height[1] = (ctx->dst_fmt.height >> 8) & 0xff;
	} else {
		ctx->vp8_enc.frame_tag->tag[0] |= 0x1;
	}

	hantro_boolenc_init(e, vaddr + frame_tag_size(ctx));

	if (is_intra(ctx)) {
		hantro_boolenc_write_lit(e, params->color_space, 1);
		hantro_boolenc_write_lit(e, params->clamping_type, 1);
	}

	hantro_boolenc_write_lit(e, !!(params->flags & V4L2_VP8_FRAME_FLAG_SEGMENT_ENABLED), 1);

	hantro_boolenc_write_lit(e, params->loop_filter_type, 1);
	hantro_boolenc_write_lit(e, params->loop_filter_level, 6);
	hantro_boolenc_write_lit(e, params->sharpness_level, 3);
	hantro_boolenc_write_lit(e, !!(params->flags & V4L2_VP8_FRAME_FLAG_LOOP_FILTER_ADJ_ENABLED),
				 1);
	if (params->flags & V4L2_VP8_FRAME_FLAG_LOOP_FILTER_ADJ_ENABLED)
		hantro_h1_vp8_enc_write_filter_level_delta(ctx);

	hantro_boolenc_write_lit(e, params->log2_nbr_of_dct_partitions, 2);

	hantro_boolenc_write_lit(e, qp, 7);

	hantro_boolenc_write_lit(e, 0, 1);
	hantro_boolenc_write_lit(e, 0, 1);
	hantro_boolenc_write_lit(e, 0, 1);
	hantro_boolenc_write_lit(e, 0, 1);
	hantro_boolenc_write_lit(e, 0, 1);

	if (!is_intra(ctx)) {
		hantro_boolenc_write_lit(e, 0, 1); /* refresh golden */
		hantro_boolenc_write_lit(e, 0, 1); /* refresh alt */
		hantro_boolenc_write_lit(e, 0, 2); /* copy to golden */
		hantro_boolenc_write_lit(e, 0, 2); /* copy to alt */
		hantro_boolenc_write_lit(e, 0, 1); /* sign bias golden */
		hantro_boolenc_write_lit(e, 0, 1); /* sign bias alt */
		hantro_boolenc_write_lit(e, ctx->vp8_enc.refresh_entropy_probs, 1);
		hantro_boolenc_write_lit(e, 1, 1);
	} else {
		hantro_boolenc_write_lit(e, ctx->vp8_enc.refresh_entropy_probs, 1);
	}

	hantro_h1_vp8_enc_write_coeff_prob(ctx);

	hantro_boolenc_write_lit(e, 1, 1);
	hantro_boolenc_write_lit(e, ctx->vp8_enc.prob_skip_false, 8);

	if (is_intra(ctx))
		return;

	hantro_boolenc_write_lit(e, ctx->vp8_enc.prob_intra, 8);
	hantro_boolenc_write_lit(e, ctx->vp8_enc.prob_last, 8);
	hantro_boolenc_write_lit(e, ctx->vp8_enc.prob_gf, 8);
	hantro_boolenc_write_lit(e, 0, 1); /* intra 16x16 prob update flag */
	hantro_boolenc_write_lit(e, 0, 1); /* intra chroma prob update flag */

	hantro_h1_vp8_enc_write_mv_prob(ctx);
}

static uint32_t calc_mvprob(uint32_t left, uint32_t right, uint32_t prob)
{
	uint32_t p;

	if (left + right) {
		p = (left * 255) / (left + right);
		p &= -2;
		if (!p)
			p = 1;
	} else {
		p = prob;
	}

	return p;
}

static uint32_t update_prob(uint32_t prob, uint32_t left, uint32_t right,
			    uint32_t old_prob, uint32_t new_prob, uint32_t fixed)
{
	int32_t u, s;

	u = (int32_t)fixed + ((vp8_prob_cost[255 - prob] - vp8_prob_cost[prob]) >> 8);
	s = ((int32_t)left * (vp8_prob_cost[old_prob] - vp8_prob_cost[new_prob]) +
		(int32_t)right *
		(vp8_prob_cost[255 - old_prob] - vp8_prob_cost[255 - new_prob])) >> 8;

	return (s > u);
}

static void hantro_h1_vp8_enc_update_entropy(struct hantro_ctx *ctx)
{
	struct hantro_vp8_entropy *entropy = &ctx->vp8_enc.entropy;
	uint16_t *p_cnt = (uint16_t *)ctx->vp8_enc.priv_dst.cpu;

	int32_t i, j, k, l;

	uint32_t p, left, right;
	uint32_t old_p, upd_p = 0;

	uint32_t type;
	uint32_t branch_cnt[2];
	uint16_t *p_tmp = NULL;

	if (!ctx->vp8_enc.refresh_entropy_probs || is_intra(ctx)) {
		memcpy(entropy->coeff_prob, default_prob_coeff, sizeof(default_prob_coeff));
		memcpy(entropy->mv_prob, default_prob_mv, sizeof(default_prob_mv));
	}

	memcpy(entropy->coeff_prob_old, entropy->coeff_prob, sizeof(entropy->coeff_prob));
	if (ctx->vp8_enc.frame_counter == 0 || !ctx->vp8_enc.last_intra)
		memcpy(entropy->mv_prob_old, entropy->mv_prob, sizeof(entropy->mv_prob));

	if (!ctx->vp8_enc.frame_counter)
		return;

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 7; j++) {
			for (k = 0; k < 3; k++) {
				int32_t tmp, ii;

				tmp = i * 7 * 3 + j * 3 + k;
				tmp += 2 * 4 * 7 * 3;
				ii = offset_tbl[tmp];

				right = ii >= 0 ? p_cnt[ii] : 0;

				for (l = 2; l--;) {
					old_p = entropy->coeff_prob[i][j][k][l];
					old_p = coeff_update_prob[i][j][k][l];

					tmp -= 4 * 7 * 3;
					ii = offset_tbl[tmp];
					left = ii >= 0 ? p_cnt[ii] : 0;
					if (left + right) {
						p = ((left * 256) + ((left + right) >> 1)) /
						    (left + right);
						if (p > 255)
							p = 255;
					} else {
						p = old_p;
					}

					if (update_prob(upd_p, left, right, old_p, p, 8))
						entropy->coeff_prob[i][j][k][l] = p;

					right += left;
				}
			}
		}
	}

	p_tmp = p_cnt + VP8_PROB_COUNT_MV_OFFSET;
	for (i = 0; i < 2; i++) {
		left  = *p_tmp++;
		right = *p_tmp++;

		p = calc_mvprob(left, right, entropy->mv_prob_old[i][0]);

		if (update_prob(mv_update_prob[i][0], left, right,
				entropy->mv_prob_old[i][0], p, 6))
			entropy->mv_prob[i][0] = p;

		right += left;
		left = *p_tmp++;
		right -= left - p_tmp[0];

		p = calc_mvprob(left, right, entropy->mv_prob_old[i][1]);
		if (update_prob(mv_update_prob[i][1], left, right,
				entropy->mv_prob_old[i][1], p, 6))
			entropy->mv_prob[i][1] = p;

		for (j = 0; j < 2; j++) {
			left = *p_tmp++;
			right = *p_tmp++;
			p = calc_mvprob(left, right, entropy->mv_prob_old[i][4 + j]);
			if (update_prob(mv_update_prob[i][4 + j], left, right,
					entropy->mv_prob_old[i][4 + j], p, 6))
				entropy->mv_prob[i][4 + j] = p;
			branch_cnt[j] = left + right;
		}

		p = calc_mvprob(branch_cnt[0], branch_cnt[1], entropy->mv_prob_old[i][3]);
		if (update_prob(mv_update_prob[i][3], branch_cnt[0], branch_cnt[1],
				entropy->mv_prob_old[i][3], p, 6))
			entropy->mv_prob[i][3] = p;

		type = branch_cnt[0] + branch_cnt[1];

		for (j = 0; j < 2; j++) {
			left = *p_tmp++;
			right = *p_tmp++;
			p = calc_mvprob(left, right, entropy->mv_prob_old[i][7 + j]);
			if (update_prob(mv_update_prob[i][7 + j], left, right,
					entropy->mv_prob_old[i][7 + j], p, 6))
				entropy->mv_prob[i][7 + j] = p;
			branch_cnt[j] = left + right;
		}

		p = calc_mvprob(branch_cnt[0], branch_cnt[1], entropy->mv_prob_old[i][6]);
		if (update_prob(mv_update_prob[i][6], branch_cnt[0], branch_cnt[1],
				entropy->mv_prob_old[i][6], p, 6))
			entropy->mv_prob[i][6] = p;

		p = calc_mvprob(type, branch_cnt[0] + branch_cnt[1],
				entropy->mv_prob_old[i][2]);
		if (update_prob(mv_update_prob[i][2], type, branch_cnt[0] + branch_cnt[1],
				entropy->mv_prob_old[i][2], p, 6))
			entropy->mv_prob[i][2] = p;
	}

	memcpy(entropy->y_mode_prob, y_mode_prob, sizeof(y_mode_prob));
	memcpy(entropy->uv_mode_prob, uv_mode_prob, sizeof(uv_mode_prob));
}

static inline int coeff_prob_idx(int index)
{
	return (index / 8) * 8 + 7 - (index % 8);
}

static void hantro_h1_vp8_enc_write_entropy(struct hantro_ctx *ctx)
{
	struct hantro_vp8_entropy *entropy = &ctx->vp8_enc.entropy;
	uint8_t *entropy_table = ctx->vp8_enc.priv_src.cpu;
	int index, i, j, k, l;

	memset(entropy_table, 0, 56);
	entropy_table[7] = ctx->vp8_enc.prob_skip_false;
	entropy_table[6] = ctx->vp8_enc.prob_intra;
	entropy_table[5] = ctx->vp8_enc.prob_last;
	entropy_table[4] = ctx->vp8_enc.prob_gf;
	entropy_table[3] = 0; /* segment_prob[0]; */
	entropy_table[2] = 0; /* segment_prob[1]; */
	entropy_table[1] = 0; /* segment_prob[2]; */

	entropy_table[15] = entropy->y_mode_prob[0];
	entropy_table[14] = entropy->y_mode_prob[1];
	entropy_table[13] = entropy->y_mode_prob[2];
	entropy_table[12] = entropy->y_mode_prob[3];
	entropy_table[11] = entropy->uv_mode_prob[0];
	entropy_table[10] = entropy->uv_mode_prob[1];
	entropy_table[9] = entropy->uv_mode_prob[2];

	entropy_table[23] = entropy->mv_prob[1][0];
	entropy_table[22] = entropy->mv_prob[0][0];
	entropy_table[21] = entropy->mv_prob[1][1];
	entropy_table[20] = entropy->mv_prob[0][1];
	entropy_table[19] = entropy->mv_prob[1][17];
	entropy_table[18] = entropy->mv_prob[1][18];
	entropy_table[17] = entropy->mv_prob[0][17];
	entropy_table[16] = entropy->mv_prob[0][18];

	for (i = 0; i < 8; ++i)
		entropy_table[24 + 7 - i] = entropy->mv_prob[1][9 + i];

	for (i = 0; i < 8; ++i)
		entropy_table[32 + 7 - i] = entropy->mv_prob[0][9 + i];

	for (i = 0; i < 7; ++i)
		entropy_table[40 + 7 - i] = entropy->mv_prob[1][2 + i];

	for (i = 0; i < 7; ++i)
		entropy_table[48 + 7 - i] = entropy->mv_prob[0][2 + i];

	index = 56;

	for (i = 0; i < 4; ++i)
		for (j = 0; j < 8; ++j)
			for (k = 0; k < 3; ++k) {
				for (l = 0; l < 3; ++l)
					entropy_table[coeff_prob_idx(index++)] =
						entropy->coeff_prob[i][j][k][l];
				entropy_table[coeff_prob_idx(index++)] = 0;
			}

	for (i = 0; i < 4; ++i)
		for (j = 0; j < 8; ++j)
			for (k = 0; k < 3; ++k) {
				for (l = 4; l < 11; ++l)
					entropy_table[coeff_prob_idx(index++)] =
						entropy->coeff_prob[i][j][k][l];
				entropy_table[coeff_prob_idx(index++)] =
					entropy->coeff_prob[i][j][k][3];
			}
}

static inline size_t sw_hdr_size(struct hantro_ctx *ctx)
{
	return ctx->vp8_enc.boolenc.bytes_written + frame_tag_size(ctx);
}

static void
hantro_h1_vp8_enc_set_buffers(struct hantro_dev *vpu, struct hantro_ctx *ctx, u32 qp,
				  struct v4l2_ctrl_vp8_encode_params *params)
{
	const u32 src_addr_regs[] = { H1_REG_ADDR_IN_PLANE_0,
				      H1_REG_ADDR_IN_PLANE_1,
				      H1_REG_ADDR_IN_PLANE_2 };
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct v4l2_pix_format_mplane *src_fmt = &ctx->src_fmt;
	size_t luma_size;
	dma_addr_t dst_dma;
	size_t dst_size;
	int i, ref_idx;

	src_buf = hantro_get_src_buf(ctx);
	dst_buf = hantro_get_dst_buf(ctx);

	luma_size = hantro_rounded_luma_size(ctx->src_fmt.width,
					     ctx->src_fmt.height);

	dst_dma = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	dst_size = vb2_plane_size(&dst_buf->vb2_buf, 0);

	/*
	 * We assume here that 1/10 of the buffer is enough for headers.
	 * DCT partition will be placed in remaining 9/10 of the buffer.
	 */
	ctx->vp8_enc.estimated_hdr_size = round_up(dst_size / 10, 8);

	/* Destination buffer. */
	/*
	 * We "abused" the first portion of this 1/10 of the buffer when generating
	 * the frame tag and compressed header, so TODO: verify that the 1/10 can hold both the
	 * compressed header and the "extended header" generated in hw.
	 */
	vepu_write_relaxed(vpu, dst_dma + round_up(sw_hdr_size(ctx), 8),
			   H1_REG_ADDR_OUTPUT_STREAM);
	vepu_write_relaxed(vpu, dst_dma + ctx->vp8_enc.estimated_hdr_size,
			   H1_REG_ADDR_VP8_DCT_PART(0));
	vepu_write_relaxed(vpu, dst_size - ctx->vp8_enc.estimated_hdr_size,
			   H1_REG_STR_BUF_LIMIT);

	/* Auxiliary buffers. */
	vepu_write_relaxed(vpu, ctx->vp8_enc.ctrl_buf.dma,
			   H1_REG_ADDR_OUTPUT_CTRL);
	memset(ctx->vp8_enc.ctrl_buf.cpu, 0, ctx->vp8_enc.ctrl_buf.size);
	vepu_write_relaxed(vpu, ctx->vp8_enc.mv_buf.dma,
			   H1_REG_ADDR_MV_OUT);
	vepu_write_relaxed(vpu, ctx->vp8_enc.priv_dst.dma,
			   H1_REG_ADDR_VP8_PROB_CNT);
	memset(ctx->vp8_enc.priv_dst.cpu, 0, ctx->vp8_enc.priv_dst.size);
	vepu_write_relaxed(vpu, ctx->vp8_enc.priv_src.dma,
			   H1_REG_ADDR_CABAC_TBL);

	hantro_h1_vp8_enc_write_entropy(ctx);

	vepu_write_relaxed(vpu, ctx->vp8_enc.priv_src.dma + 151 * 8,
			   H1_REG_ADDR_VP8_SEG_MAP);

	/* Reference buffers. */
	switch (params->reference_type) {
	case V4L2_VP8_FRAME_REF_LAST:
		ref_idx = ctx->vp8_enc.last_ref;
		break;
	case V4L2_VP8_FRAME_REF_GOLDEN:
		ref_idx = ctx->vp8_enc.golden_ref;
		break;
	case V4L2_VP8_FRAME_REF_ALT:
		ref_idx = ctx->vp8_enc.alt_ref;
		break;
	}
	if (ref_idx < 0)
		ref_idx = 0;
	vepu_write_relaxed(vpu, ctx->vp8_enc.ref_frames[ref_idx].dma, H1_REG_ADDR_REF_LUMA);
	vepu_write_relaxed(vpu, ctx->vp8_enc.ref_frames[ref_idx].dma + luma_size,
			   H1_REG_ADDR_REF_CHROMA);

	/* Reconstruction buffers. */
	ctx->vp8_enc.reconstructed =  ctx->vp8_enc.first_free;
	vepu_write_relaxed(vpu, ctx->vp8_enc.ref_frames[ctx->vp8_enc.reconstructed].dma,
			   H1_REG_ADDR_REC_LUMA);
	vepu_write_relaxed(vpu, ctx->vp8_enc.ref_frames[ctx->vp8_enc.reconstructed].dma + luma_size,
			   H1_REG_ADDR_REC_CHROMA);

	/* Source buffer. */
	vepu_write_relaxed(vpu,
			   vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0) +
			   src_buf->vb2_buf.planes[0].data_offset,
			   src_addr_regs[0]);
	vepu_write_relaxed(vpu,
			   vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0) +
			   src_buf->vb2_buf.planes[0].data_offset +
			   luma_size,
			   src_addr_regs[1]);

	for (i = 1; i < src_fmt->num_planes; ++i)
		/* Multiplanes. */
		vepu_write_relaxed(vpu,
				   vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, i) +
				   src_buf->vb2_buf.planes[i].data_offset,
				   src_addr_regs[i]);
}

static inline uint32_t compute_filter_level(int32_t qp)
{
	return clamp((qp * 64) / 128 + 8, 0, 63);
}

static s32 cost_tree(struct tree *tree, const s32 *prob)
{
	s32 value = tree->value;
	s32 number = tree->number;
	s32 *index = tree->index;
	s32 bit_cost = 0;

	while (number--)
		bit_cost += COST_BOOL(prob[*index++], (value >> number) & 1);

	return bit_cost;
}

static s32 cost_mv(s32 mvd, const s32 *mv_prob)
{
	s32 i, tmp, bit_cost = 0;

	tmp = abs(mvd >> 1);

	/* Short Tree */
	if (tmp < 8) {
		bit_cost += COST_BOOL(mv_prob[0], 0);
		bit_cost += cost_tree(&mv_tree[tmp], mv_prob + 2);
		if (!tmp)
			return bit_cost;

		/* Sign */
		bit_cost += COST_BOOL(mv_prob[1], mvd < 0);
		return bit_cost;
	}

	/* Long Tree */
	bit_cost += COST_BOOL(mv_prob[0], 1);

	/* Bits 0, 1, 2 */
	for (i = 0; i < 3; i++)
		bit_cost += COST_BOOL(mv_prob[9 + i], (tmp >> i) & 1);

	/* Bits 9, 8, 7, 6, 5, 4 */
	for (i = 9; i > 3; i--)
		bit_cost += COST_BOOL(mv_prob[9 + i], (tmp >> i) & 1);

	/*
	 * Bit 3: if ABS(mvd) < 8, it is coded with short tree, so if here
	 * ABS(mvd) <= 15, bit 3 must be one (because here we code values
	 * 8,...,15) and is not explicitly coded.
	 */
	if (tmp > 15)
		bit_cost += COST_BOOL(mv_prob[9 + 3], (tmp >> 3) & 1);

	/* Sign */
	bit_cost += COST_BOOL(mv_prob[1], mvd < 0);

	return bit_cost;
}

static void hantro_h1_vp8_enc_set_params(struct hantro_dev *vpu, struct hantro_ctx *ctx, u32 qp,
					     struct v4l2_ctrl_vp8_encode_params *params)
{
	struct hantro_vp8_entropy *entropy = &ctx->vp8_enc.entropy;
	int i;
	unsigned int mb_width = MB_WIDTH(ctx->src_fmt.width);
	unsigned int mb_height = MB_HEIGHT(ctx->src_fmt.height);
	u32 deq;
	u32 tmp;
	u32 reg;
	s16 inter_favor;

	vepu_write_relaxed(vpu, 0, H1_REG_STR_HDR_REM_MSB);
	vepu_write_relaxed(vpu, 0, H1_REG_STR_HDR_REM_LSB);

	reg = 0;
	if (mb_width * mb_height > MAX_MB_COUNT_TO_DISABLE_QUARTER_PIXEL_MV)
		reg = H1_REG_ENC_CTRL2_DISABLE_QUARTER_PIXMV;
	reg |= H1_REG_ENC_CTRL2_VP8_BOOLENC_ENABLE;

	inter_favor = 128 - ctx->vp8_enc.prob_intra;
	if (inter_favor >= 0)
		/*
		 * It seems that hardware expects a negative s16 non-0 value
		 * for inter_favor otherwise P frames have pink artefacts...
		 */
		/*inter_favor = max(0, (int32_t)(qp * 2 - 40));*/
		inter_favor = -max(1, (int32_t)(qp * 2 - 40));

	reg |= H1_REG_ENC_CTRL2_INTRA16X16_MODE(qp * 1024 / 128);
	vepu_write_relaxed(vpu, reg, H1_REG_ENC_CTRL2);//in VEPU_REG_INTRA_INTER_MODE

	reg = H1_REG_ENC_CTRL5_INTER_MODE(inter_favor);//FIXME replace VEPU_REG_INTER_MODE(inter_favor of VEPU_REG_INTRA_INTER_MODE
	reg |= H1_REG_ENC_CTRL5_MACROBLOCK_PENALTY(qp >= 100 ? (3 * qp / 4) : 0);//FIXME, guess !!
	vepu_write_relaxed(vpu, reg, H1_REG_ENC_CTRL5);//in VEPU_REG_INTRA_INTER_MODE

	reg = H1_REG_ENC_CTRL3_MV_PENALTY_1P(60 / 2 * 32)
		| H1_REG_ENC_CTRL3_MV_PENALTY_1_4P(8)
		| H1_REG_ENC_CTRL3_MV_PENALTY_4P(64 / 2);
	if (mb_width * mb_height < MAX_MB_COUNT_TO_DISABLE_SPLIT_MV)
		reg |= H1_REG_ENC_CTRL3_MUTIMV_EN;
	vepu_write_relaxed(vpu, reg, H1_REG_ENC_CTRL3);

	reg = H1_REG_ENC_CTRL4_MV_PENALTY_16X8_8X16(
			min(1023, split_penalty[qp] / 2))
		| H1_REG_ENC_CTRL4_MV_PENALTY_8X8(
			min(1023, (2 * split_penalty[qp] + 40) / 4))
		| H1_REG_ENC_CTRL4_8X4_4X8(0x3ff);
	/* no 8x4 or 4x8 block define in vp8 */
	vepu_write_relaxed(vpu, reg, H1_REG_ENC_CTRL4);

	reg = H1_REG_PENALTY_4X4MV(min(511,
				       (8 * split_penalty[qp] + 500) / 16))
		| H1_REG_ZERO_MV_FAVOR_D2(0);
	vepu_write_relaxed(vpu, reg, H1_REG_QP_MV_MVC_CTRL);

	/* initialize quant table for segment0 */
	deq = dc_q_lookup[qp];
	reg = H1_REG_VP8_SEG0_QUT_DC_Y1(min((1 << 16) / deq, 0x3FFFu));
	reg |= H1_REG_VP8_SEG0_ZBIN_DC_Y1(((qzbin_factors[qp] * deq) + 64) >>
					    7);
	reg |= H1_REG_VP8_SEG0_RND_DC_Y1((qrounding_factors[qp] * deq) >> 7);
	vepu_write_relaxed(vpu, reg, H1_REG_VP8_SEG0_QUANT_DC_Y1);

	deq = ac_q_lookup[qp];
	reg = H1_REG_VP8_SEG0_QUT_AC_Y1(min((1 << 16) / deq, 0x3FFFu));
	reg |= H1_REG_VP8_SEG0_ZBIN_AC_Y1(((qzbin_factors[qp] * deq) + 64) >>
					    7);
	reg |= H1_REG_VP8_SEG0_RND_AC_Y1((qrounding_factors[qp] * deq) >> 7);
	vepu_write_relaxed(vpu, reg, H1_REG_VP8_SEG0_QUANT_AC_Y1);

	deq = dc_q_lookup[qp] * 2;
	reg = H1_REG_VP8_SEG0_QUT_DC_Y2(min((1 << 16) / deq, 0x3FFFu));
	reg |= H1_REG_VP8_SEG0_ZBIN_DC_Y2((qzbin_factors[qp] * deq + 64) >>
					    7);
	reg |= H1_REG_VP8_SEG0_RND_DC_Y2((qrounding_factors[qp] * deq) >> 7);
	vepu_write_relaxed(vpu, reg, H1_REG_VP8_SEG0_QUANT_DC_Y2);

	deq = max(ac_q_lookup[qp] * 155 / 100, 8);
	reg = H1_REG_VP8_SEG0_QUT_AC_Y2(min((1 << 16) / deq, 0x3FFFu));
	reg |= H1_REG_VP8_SEG0_ZBIN_AC_Y2((qzbin_factors[qp] * deq + 64) >>
					    7);
	reg |= H1_REG_VP8_SEG0_RND_AC_Y2((qrounding_factors[qp] * deq) >> 7);
	vepu_write_relaxed(vpu, reg, H1_REG_VP8_SEG0_QUANT_AC_Y2);

	deq = min(dc_q_lookup[qp], 132);
	reg = H1_REG_VP8_SEG0_QUT_DC_CHR(min((1 << 16) / deq, 0x3FFFu));
	reg |= H1_REG_VP8_SEG0_ZBIN_DC_CHR((qzbin_factors[qp] * deq + 64) >>
					     7);
	reg |= H1_REG_VP8_SEG0_RND_DC_CHR((qrounding_factors[qp] * deq) >> 7);
	vepu_write_relaxed(vpu, reg, H1_REG_VP8_SEG0_QUANT_DC_CHR);

	deq = ac_q_lookup[qp];
	reg = H1_REG_VP8_SEG0_QUT_AC_CHR(min((1 << 16) / deq, 0x3FFFu));
	reg |= H1_REG_VP8_SEG0_ZBIN_AC_CHR((qzbin_factors[qp] * deq + 64) >>
					     7);
	reg |= H1_REG_VP8_SEG0_RND_AC_CHR((qrounding_factors[qp] * deq) >> 7);
	vepu_write_relaxed(vpu, reg, H1_REG_VP8_SEG0_QUANT_AC_CHR);

	reg = H1_REG_VP8_MV_REF_IDX1(params->reference_type);
	reg |= H1_REG_VP8_SEG0_DQUT_DC_Y1(dc_q_lookup[qp]);
	reg |= H1_REG_VP8_SEG0_DQUT_AC_Y1(ac_q_lookup[qp]);
	reg |= H1_REG_VP8_SEG0_DQUT_DC_Y2(dc_q_lookup[qp] * 2);
	vepu_write_relaxed(vpu, reg, H1_REG_VP8_SEG0_QUANT_DQUT);

	reg = H1_REG_VP8_MV_REF_IDX2(0);
	reg |= H1_REG_VP8_SEG0_DQUT_DC_CHR(min(dc_q_lookup[qp], 132));
	reg |= H1_REG_VP8_SEG0_DQUT_AC_CHR(ac_q_lookup[qp]);
	reg |= H1_REG_VP8_SEG0_DQUT_AC_Y2(max(ac_q_lookup[qp] * 155 / 100, 8));
	if (is_intra(ctx))
		reg |= H1_REG_VP8_SEGMENT_MAP_UPDATE;
	vepu_write_relaxed(vpu, reg, H1_REG_VP8_SEG0_QUANT_DQUT_1);

	vepu_write_relaxed(vpu, ctx->vp8_enc.boolenc.bottom,
			   H1_REG_VP8_BOOL_ENC);

	reg = H1_REG_VP8_DCT_PARTITION_CNT(0);
	reg |= H1_REG_VP8_FILTER_LEVEL(compute_filter_level(qp));
	reg |= H1_REG_VP8_FILTER_SHARPNESS(params->sharpness_level);
	reg |= H1_REG_VP8_ZERO_MV_PENALTY_FOR_REF2(0);
	reg |= H1_REG_VP8_BOOL_ENC_VALUE_BITS(24 - ctx->vp8_enc.boolenc.bit_count);
	reg |= H1_REG_VP8_BOOL_ENC_RANGE(ctx->vp8_enc.boolenc.range);
	vepu_write_relaxed(vpu, reg, H1_REG_VP8_PENALTY_FILTER);

	vepu_write_relaxed(vpu, 0xffffffff, H1_REG_FIRST_ROI_AREA);
	vepu_write_relaxed(vpu, 0xffffffff, H1_REG_SECOND_ROI_AREA);
	vepu_write_relaxed(vpu, 0, H1_REG_STABILIZATION_OUTPUT);
	vepu_write_relaxed(vpu, 0, H1_REG_CIR_INTRA_CTRL);
	vepu_write_relaxed(vpu, 0xffffffff, H1_REG_INTRA_AREA_CTRL);

	/* Intra 4x4 mode */
	//FIXME VEPU=12bits (0xfff) + 16bits align ? while H1=10bits in regmap & 10bits aligned
	tmp = qp * 2 + 8;
	for (i = 0; i < 5; i++) {
		reg = H1_REG_VP8_INTRA_4X4_PENALTY_0
			((intra_4_tree_penalty[i * 2] * tmp) >> 8);
		reg |= HI_REG_VP8_INTRA_4X4_PENALTY_1
			((intra_4_tree_penalty[i * 2 + 1] * tmp) >> 8);
		vepu_write_relaxed(vpu, reg,
				   H1_REG_VP8_INTRA_4X4_PENALTY(i));
	}

	/* Intra 16x16 mode */
	//FIXME VEPU=12bits (0xfff) + 16bits align ? while H1=10bits in regmap & 10bits aligned
	tmp = qp * 2 + 64;
	for (i = 0; i < 2; i++) {
		reg = H1_REG_VP8_INTRA_16X16_PENALTY_0
			((intra_16_tree_penalty[2 * i] * tmp) >> 8);
		reg |= HI_REG_VP8_INTRA_16X16_PENALTY_1
			((intra_16_tree_penalty[2 * i + 1] * tmp) >> 8);
		vepu_write_relaxed(vpu, reg,
				   H1_REG_VP8_INTRA_16X16_PENALTY(i));//VEPU_REG_VP8_INTRA_16X16_PENALTY
	}

	//FIXME H1 slightly different...
	reg = H1_REG_VP8_LF_REF_DELTA_INTRA_MB(ctx->vp8_enc.ref_delta[REF_DELTA_INTRA_MB])
		| H1_REG_VP8_LF_REF_DELTA_LAST_REF(ctx->vp8_enc.ref_delta[REF_DELTA_LAST_REF])
		| H1_REG_VP8_LF_REF_DELTA_GOLDEN(ctx->vp8_enc.ref_delta[REF_DELTA_GOLDEN])
		| H1_REG_VP8_LF_REF_DELTA_ALT_REF(ctx->vp8_enc.ref_delta[REF_DELTA_ALT_REF]);
	vepu_write_relaxed(vpu, reg, H1_REG_VP8_LOOP_FILTER_REF_DELTA);

	reg = H1_REG_VP8_LF_MODE_DELTA_BPRED(ctx->vp8_enc.mode_delta[MODE_DELTA_BPRED])
		| H1_REG_VP8_LF_MODE_DELTA_ZEROMV(ctx->vp8_enc.mode_delta[MODE_DELTA_ZEROMV])
		| H1_REG_VP8_LF_MODE_DELTA_NEWMV(ctx->vp8_enc.mode_delta[MODE_DELTA_NEWMV])
		| H1_REG_VP8_LF_MODE_DELTA_SPLITMV(ctx->vp8_enc.mode_delta[MODE_DELTA_SPLITMV]);
	vepu_write_relaxed(vpu, reg, H1_REG_VP8_LOOP_FILTER_MODE_DELTA);

	//HFR no match for VEPU_REG_VP8_INTER_TYPE_BIT_COST, but put to 0...


	for (i = 0; i < 128; i += 4) {
		u32 x;
		u32 y;

		reg = H1_REG_DMV_4P_1P_PENALTY_BIT(i * 2, 3);
		reg |= H1_REG_DMV_4P_1P_PENALTY_BIT((i + 1) * 2, 2);
		reg |= H1_REG_DMV_4P_1P_PENALTY_BIT((i + 2) * 2, 1);
		reg |= H1_REG_DMV_4P_1P_PENALTY_BIT((i + 3) * 2, 0);
		vepu_write_relaxed(vpu, reg, H1_REG_DMV_4P_1P_PENALTY(i / 4));

		y = cost_mv(i * 2, entropy->mv_prob[0]);	/* mv y */
		x = cost_mv(i * 2, entropy->mv_prob[1]);	/* mv x */

		reg = H1_REG_DMV_QPEL_PENALTY_BIT(
			min(255u, (y + x + 1) / 2 * weight[qp] >> 8), 3);

		y = cost_mv((i + 1) * 2, entropy->mv_prob[0]); /* mv y */
		x = cost_mv((i + 1) * 2, entropy->mv_prob[1]); /* mv x */
		reg |= H1_REG_DMV_QPEL_PENALTY_BIT(
			min(255u, (y + x + 1) / 2 * weight[qp] >> 8), 2);

		y = cost_mv((i + 2) * 2, entropy->mv_prob[0]); /* mv y */
		x = cost_mv((i + 2) * 2, entropy->mv_prob[1]); /* mv x */
		reg |= H1_REG_DMV_QPEL_PENALTY_BIT(
			min(255u, (y + x + 1) / 2 * weight[qp] >> 8), 1);

		y = cost_mv((i + 3) * 2, entropy->mv_prob[0]); /* mv y */
		x = cost_mv((i + 3) * 2, entropy->mv_prob[1]); /* mv x */
		reg |= H1_REG_DMV_QPEL_PENALTY_BIT(
			min(255u, (y + x + 1) / 2 * weight[qp] >> 8), 0);

		vepu_write_relaxed(vpu, reg,
				   H1_REG_DMV_QPEL_PENALTY(i / 4));
	}
}

static char *encode_params_str(struct v4l2_ctrl_vp8_encode_params *p,
			       char *str,
			       unsigned int len)
{
	if (!p)
		return NULL;

	snprintf(str, len,
		 "v4l2_ctrl_vp8_encode_params\n"
		 " flags:\t\t\t %s %s %s %s %s %s\n"
		 " frame_type:\t\t%s\n"
		 " color_space:\t\t%s\n"
		 " clamping_type:\t\t%s\n"
		 " loop_filter_type:\t%s\n"
		 " loop_filter_level:\t%d\n"
		 " sharpness_level:\t%d\n"
		 " log2_nbr_of_dct_parts:\t%s\n"
		 " prob_intra:\t\t%d\n"
		 " prob_last:\t\t%d\n"
		 " prob_gf:\t\t%d\n"
		 " copy_to_golden:\t\t%d\n"
		 " copy_to_alternate:\t%d\n"
		 " reference_type:\t\t%s\n",
		 (p->flags & V4L2_VP8_FRAME_FLAG_SHOWFRAME) ? "SHOW" : "",
		 (p->flags & V4L2_VP8_FRAME_FLAG_GOLDEN_REFRESH) ? "GOLDEN" : "",
		 (p->flags & V4L2_VP8_FRAME_FLAG_ALTREF_REFRESH) ? "ALTREF" : "",
		 (p->flags & V4L2_VP8_FRAME_FLAG_SEGMENT_ENABLED) ? "SEGMENT" : "",
		 (p->flags & V4L2_VP8_FRAME_FLAG_LOOP_FILTER_ADJ_ENABLED) ? "LOOPFILTER" : "",
		 (p->flags & V4L2_VP8_FRAME_FLAG_REFRESH_ENTROPY_PROBS) ? "REFRESH_ENTROPY" : "",
		 (p->frame_type == V4L2_VP8_FRAME_TYPE_KEYFRAME) ? "KEYFRAME" : "INTER",
		 (p->color_space == V4L2_VP8_FRAME_COLOR_SPACE_YUV) ? "YUV" : "RESERVED",
		 (p->clamping_type == V4L2_VP8_FRAME_CLAMPING_REQUIRED) ? "REQ" : "NO",
		 (p->loop_filter_type == V4L2_VP8_FRAME_FILTER_TYPE_NORMAL) ? "NORMAL" : "SIMPLE",
		 p->loop_filter_level,
		 p->sharpness_level,
		 (p->log2_nbr_of_dct_partitions == V4L2_VP8_FRAME_NBR_DCT_PARTITIONS_1) ? "1" :
		 (p->log2_nbr_of_dct_partitions == V4L2_VP8_FRAME_NBR_DCT_PARTITIONS_2) ? "2" :
		 (p->log2_nbr_of_dct_partitions == V4L2_VP8_FRAME_NBR_DCT_PARTITIONS_4) ? "4" : "8",
		 p->prob_intra,
		 p->prob_last,
		 p->prob_gf,
		 p->copy_buffer_to_golden,
		 p->copy_buffer_to_alternate,
		 (p->reference_type == V4L2_VP8_FRAME_REF_LAST) ? "LAST" :
		 (p->reference_type == V4L2_VP8_FRAME_REF_GOLDEN) ? "GOLDEN" : "ALT");

	return str;
}

int hantro_h1_vp8_enc_run(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct v4l2_ctrl_vp8_encode_params *params;
	struct v4l2_ctrl *ctrl;
	u32 reg, qp;
	int i;

	hantro_start_prepare_run(ctx);

	params = hantro_get_ctrl(ctx, V4L2_CID_STATELESS_VP8_ENCODE_PARAMS);
	if (WARN_ON(!params))
		return -EINVAL;

	vpu_debug(1, "%s\n", encode_params_str(params, ctx->str, sizeof(ctx->str)));

	if (params->flags & V4L2_VP8_FRAME_FLAG_SEGMENT_ENABLED)
		return -EINVAL;

	ctx->vp8_enc.is_intra = params->frame_type == V4L2_VP8_FRAME_TYPE_KEYFRAME;

	if (is_intra(ctx)) {
		if (params->loop_filter_level != 26)
			return -EINVAL;
	} else {
		if (params->loop_filter_level != 12)
			return -EINVAL;
	}

	ctrl = v4l2_ctrl_find(&ctx->ctrl_handler, V4L2_CID_STATELESS_VP8_ENCODE_QP);
	if (WARN_ON(!ctrl))
		return -EINVAL;

	ctx->vp8_enc.show_frame = params->flags & V4L2_VP8_FRAME_FLAG_SHOWFRAME;
	ctx->vp8_enc.refresh_golden_frame = params->flags & V4L2_VP8_FRAME_FLAG_GOLDEN_REFRESH;
	ctx->vp8_enc.refresh_alternate_frame = params->flags & V4L2_VP8_FRAME_FLAG_ALTREF_REFRESH;
	ctx->vp8_enc.refresh_entropy_probs =
		params->flags & V4L2_VP8_FRAME_FLAG_REFRESH_ENTROPY_PROBS;
	ctx->vp8_enc.copy_buffer_to_golden = params->copy_buffer_to_golden;
	ctx->vp8_enc.copy_buffer_to_alternate = params->copy_buffer_to_alternate;
	ctx->vp8_enc.prob_intra = params->prob_intra;
	ctx->vp8_enc.prob_last = params->prob_last;
	ctx->vp8_enc.prob_gf = params->prob_gf;

	hantro_h1_vp8_enc_update_entropy(ctx);

	qp = *ctrl->p_cur.p_s32;
	ctx->vp8_enc.prob_skip_false = default_prob_skip_false[qp];
	hantro_h1_vp8_enc_write_header(ctx, qp, params);

	/*
	 * Program the hardware.
	 */
	vepu_write_relaxed(vpu, H1_REG_ENC_CTRL_ENC_MODE_VP8, H1_REG_ENC_CTRL);

	hantro_h1_vp8_enc_set_params(vpu, ctx, qp, params);
	hantro_h1_set_src_img_ctrl(vpu, ctx);
	hantro_h1_vp8_enc_set_buffers(vpu, ctx, qp, params);

	hantro_h1_set_color_conv(vpu, ctx);

	/* Make sure that all registers are written at this point. */
	hantro_h1_set_axi_ctrl(vpu, ctx);

	/* Start the hardware. */
	reg =     H1_REG_ENC_CTRL_TIMEOUT_EN
		| H1_REG_ENC_CTRL_MV_WRITE //for VEPU_REG_MV_WRITE_EN in VEPU_REG_INTERRUPT
		| H1_REG_ENC_CTRL_WIDTH(MB_WIDTH(ctx->src_fmt.width))
		| H1_REG_ENC_CTRL_HEIGHT(MB_HEIGHT(ctx->src_fmt.height))
		| H1_REG_ENC_CTRL_ENC_MODE_VP8
		| H1_REG_ENC_CTRL_EN_BIT;

	if (is_intra(ctx))
		reg |= H1_REG_ENC_PIC_INTRA;
	else
		reg |= H1_REG_ENC_PIC_INTER;

	/* Kick the watchdog and start encoding */
	hantro_end_prepare_run(ctx);

	for (i = 0; i < 184; ++i)
		vepu_read(vpu, i * 4);

	vepu_write(vpu, reg, H1_REG_ENC_CTRL);

	return 0;
}

void hantro_h1_vp8_enc_done(struct hantro_ctx *ctx)
{
	struct vb2_v4l2_buffer *dst_buf;
	struct hantro_vp8_enc_ctrl_buf *ctrl_buf = ctx->vp8_enc.ctrl_buf.cpu;
	int part1_sz = ctx->vp8_enc.boolenc.bytes_written + ctrl_buf->hw_hdr_size;
	void *dst = ctx->vp8_enc.frame_tag;
	int old_g = ctx->vp8_enc.golden_ref, old_a = ctx->vp8_enc.alt_ref;
	int i;

	ctx->vp8_enc.frame_tag->tag[0] |= ((part1_sz << 5) & 0xff);
	ctx->vp8_enc.frame_tag->tag[1] = ((part1_sz << 5) >> 8) & 0xff;
	ctx->vp8_enc.frame_tag->tag[2] = ((part1_sz << 5) >> 16) & 0xff;

	dst_buf = hantro_get_dst_buf(ctx);

	/* assemble dst frame */
	vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
			      sw_hdr_size(ctx) + ctrl_buf->hw_hdr_size + ctrl_buf->dct_size);

	memmove(dst + sw_hdr_size(ctx), dst + round_up(sw_hdr_size(ctx), 8), ctrl_buf->hw_hdr_size);
	memmove(dst + sw_hdr_size(ctx) + ctrl_buf->hw_hdr_size,
		dst + ctx->vp8_enc.estimated_hdr_size, ctrl_buf->dct_size);

	/* currently reconstructed frame becames last ref */
	hantro_vp8_clr_bit(&ctx->vp8_enc, VP8_BIT_LAST, ctx->vp8_enc.last_ref);
	ctx->vp8_enc.last_ref = ctx->vp8_enc.reconstructed;
	hantro_vp8_set_bit(&ctx->vp8_enc, VP8_BIT_LAST, ctx->vp8_enc.last_ref);

	/* after update (potentially) new slots will be used */
	hantro_vp8_clr_bit(&ctx->vp8_enc, VP8_BIT_GOLDEN, old_g);
	hantro_vp8_clr_bit(&ctx->vp8_enc, VP8_BIT_ALT, old_a);

	/* VP8 spec, section 9.7. Refresh Golden Frame and Altref Frame */
	if (is_intra(ctx)) {
		ctx->vp8_enc.golden_ref = ctx->vp8_enc.last_ref;
		ctx->vp8_enc.alt_ref = ctx->vp8_enc.last_ref;
	} else {
		if (ctx->vp8_enc.refresh_golden_frame) {
			ctx->vp8_enc.golden_ref = ctx->vp8_enc.last_ref;
		} else if (ctx->vp8_enc.copy_buffer_to_golden) {
			switch (ctx->vp8_enc.copy_buffer_to_golden) {
			case V4L2_VP8_FRAME_GOLDEN_KEEP:
				break;
			case V4L2_VP8_FRAME_GOLDEN_LASTFRAME:
				ctx->vp8_enc.golden_ref = ctx->vp8_enc.last_ref;
				break;
			case V4L2_VP8_FRAME_GOLDEN_ALTREF:
				ctx->vp8_enc.golden_ref = old_a;
				break;
			}
		}

		if (ctx->vp8_enc.refresh_alternate_frame) {
			ctx->vp8_enc.alt_ref = ctx->vp8_enc.last_ref;
		} else if (ctx->vp8_enc.copy_buffer_to_alternate) {
			switch (ctx->vp8_enc.copy_buffer_to_alternate) {
			case V4L2_VP8_FRAME_ALTREF_KEEP:
				break;
			case V4L2_VP8_FRAME_ALTREF_LASTFRAME:
				ctx->vp8_enc.alt_ref = ctx->vp8_enc.last_ref;
				break;
			case V4L2_VP8_FRAME_ALTREF_GOLDEN:
				ctx->vp8_enc.alt_ref = old_g;
				break;
			}
		}
	}

	/* mark updated golden and alt slots used */
	hantro_vp8_set_bit(&ctx->vp8_enc, VP8_BIT_GOLDEN, ctx->vp8_enc.golden_ref);
	hantro_vp8_set_bit(&ctx->vp8_enc, VP8_BIT_ALT, ctx->vp8_enc.alt_ref);

	/* find first free slot after updates */
	for (i = 0; i < ARRAY_SIZE(ctx->vp8_enc.ref_bitmaps); ++i)
		if (!ctx->vp8_enc.ref_bitmaps[i]) {
			ctx->vp8_enc.first_free = i;
			break;
		}
	WARN_ON(i >= 4);

	ctx->vp8_enc.last_intra = is_intra(ctx);
	++ctx->vp8_enc.frame_counter;

	if (is_intra(ctx))
		dst_buf->flags |= V4L2_BUF_FLAG_KEYFRAME;
	else
		dst_buf->flags |= V4L2_BUF_FLAG_PFRAME;
}
