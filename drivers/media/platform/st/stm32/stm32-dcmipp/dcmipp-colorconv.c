// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for STM32 Digital Camera Memory Interface Pixel Processor
 *
 * Copyright (C) STMicroelectronics SA 2021
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          Alain Volmat <alain.volmat@foss.st.com>
 *          for STMicroelectronics.
 */
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#include "dcmipp-colorconv.h"

/* Macro for negative coefficient, 11 bits coded */
#define N11(val) (((val) ^ 0x7FF) + 1)
/* Macro for added value, 10 bits coded */
#define N10(val) (((val) ^ 0x3FF) + 1)

/* Macro to convert row matrix to DCMIPP PxCCCyy register value */
#define ROW(c1, c2, c3, add) ((c2) << 16 | (c1)), ((add) << 16 | (c3))

static const u32 rgbfull_to_yuv601full[] = {
/*	     R		G		B		Add */
/* Cr */ ROW(131,	N11(110),	N11(21),	128),
/*  Y */ ROW(77,	150,		29,		0),
/* Cb */ ROW(N11(44),	N11(87),	131,		128),
};

static const u32 rgbfull_to_yuv601lim[] = {
/*	     R		G		B		Add */
/* Cr */ ROW(112,	N11(94),	N11(18),	128),
/*  Y */ ROW(66,	129,		25,		16),
/* Cb */ ROW(N11(38),	N11(74),	112,		128),
};

static const u32 rgbfull_to_yuv709full[] = {
/*	     R		G		B		Add */
/* Cr */ ROW(131,	N11(119),	N11(12),	128),
/*  Y */ ROW(55,	183,		18,		0),
/* Cb */ ROW(N11(30),	N11(101),	131,		128),
};

static const u32 rgbfull_to_yuv709lim[] = {
/*	     R		G		B		Add */
/* Cr */ ROW(112,	N11(102),	N11(10),	128),
/*  Y */ ROW(47,	157,		16,		16),
/* Cb */ ROW(N11(26),	N11(87),	112,		128),
};

static const u32 rgblim_to_yuv601lim[] = {
/*	     R		G		B		Add */
/* Cr */ ROW(131,	N11(110),	N11(21),	128),
/*  Y */ ROW(77,	150,		29,		0),
/* Cb */ ROW(N11(44),	N11(87),	131,		128),
};

static const u32 rgblim_to_yuv709lim[] = {
/*	     R		G		B		Add */
/* Cr */ ROW(131,	N11(119),	N11(12),	128),
/*  Y */ ROW(55,	183,		18,		0),
/* Cb */ ROW(N11(30),	N11(101),	131,		128),
};

static const u32 yuv601full_to_rgbfull[] = {
/*	     Cr		Y		Cb		Add */
/* R */  ROW(351,	256,		0,		N10(175)),
/* G */  ROW(N11(179),	256,		N11(86),	132),
/* B */  ROW(0,		256,		443,		N10(222)),
};

static const u32 yuv601lim_to_rgbfull[] = {
/*	     Cr		Y		Cb		Add */
/* R */  ROW(409,	298,		0,		N10(223)),
/* G */  ROW(N11(208),	298,		N11(100),	135),
/* B */  ROW(0,		298,		517,		N10(277)),
};

static const u32 yuv601lim_to_rgblim[] = {
/*	     Cr		Y		Cb		Add */
/* R */  ROW(351,	256,		0,		N10(175)),
/* G */  ROW(N11(179),	256,		N11(86),	132),
/* B */  ROW(0,		256,		443,		N10(222)),
};

static const u32 yuv709full_to_rgbfull[] = {
/*	     Cr		Y		Cb		Add */
/* R */  ROW(394,	256,		0,		N10(197)),
/* G */  ROW(N11(118),	256,		N11(47),	82),
/* B */  ROW(0,		256,		456,		N10(232)),
};

static const u32 yuv709lim_to_rgbfull[] = {
/*	     Cr		Y		Cb		Add */
/* R */  ROW(459,	298,		0,		N10(248)),
/* G */  ROW(N11(137),	298,		N11(55),	77),
/* B */  ROW(0,		298,		541,		N10(289)),
};

static const u32 yuv709lim_to_rgblim[] = {
/*	     Cr		Y		Cb		Add */
/* R */  ROW(394,	256,		0,		N10(197)),
/* G */  ROW(N11(118),	256,		N11(47),	82),
/* B */  ROW(0,		256,		465,		N10(232)),
};

/* cconv_matrices[src_fmt][src_range][sink_fmt][sink_range] */
static const u32 *cconv_matrices[3][2][3][2] = {
	/* RGB */
	{
		/* RGB full range */
		{
			/* RGB full range => RGB */
			{
				/* RGB full range => RGB full range */
				NULL,
				/* RGB full range => RGB limited range */
				NULL,
			},
			/* RGB full range => YUV601 */
			{
				/* RGB full range => YUV601 full range */
				rgbfull_to_yuv601full,
				/* RGB full range => YUV601 limited range */
				rgbfull_to_yuv601lim,
			},
			/* RGB full range => YUV709 */
			{
				/* RGB full range => YUV709 full range */
				rgbfull_to_yuv709full,
				/* RGB full range => YUV709 limited range */
				rgbfull_to_yuv709lim,
			},
		},
		/* RGB limited range */
		{
			/* RGB limited range => RGB */
			{
				/* RGB limited range => RGB full range */
				NULL,
				/* RGB limited range => RGB limited range */
				NULL,
			},
			/* RGB limited range => YUV601 */
			{
				/* RGB limited range => YUV601 full range */
				NULL,
				/* RGB limited range => YUV601 limited range */
				rgblim_to_yuv601lim,
			},
			/* RGB limited range => YUV709 */
			{
				/* RGB limited range => YUV709 full range */
				NULL,
				/* RGB limited range => YUV709 limited range */
				rgblim_to_yuv709lim,
			},
		},
	},
	/* YUV601 */
	{
		/* YUV601 full range */
		{
			/* YUV601 full range => RGB */
			{
				/* YUV601 full range => RGB full range */
				yuv601full_to_rgbfull,
				/* YUV601 full range => RGB limited range */
				NULL,
			},
			/* YUV601 full range => YUV601 */
			{
				/* YUV601 full range => YUV601 full range */
				NULL,
				/* YUV601 full range => YUV601 limited range */
				NULL,
			},
			/* YUV601 full range => YUV709 */
			{
				/* YUV601 full range => YUV709 full range */
				NULL,
				/* YUV601 full range => YUV709 limited range */
				NULL,
			},
		},
		/* YUV601 limited range */
		{
			/* YUV601 limited range => RGB */
			{
				/* YUV601 limited range => RGB full range */
				yuv601lim_to_rgbfull,
				/* YUV601 limited range => RGB limited range */
				yuv601lim_to_rgblim,
			},
			/* YUV601 limited range => YUV601 */
			{
				/* YUV601 limited range => YUV601 full range */
				NULL,
				/* YUV601 limited range => YUV601 limited range */
				NULL,
			},
			/* YUV601 limited range => YUV709 */
			{
				/* YUV601 limited range => YUV709 full range */
				NULL,
				/* YUV601 limited range => YUV709 limited range */
				NULL,
			},
		},
	},
	/* YUV709 */
	{
		/* YUV709 full range */
		{
			/* YUV709 full range => RGB */
			{
				/* YUV709 full range => RGB full range */
				yuv709full_to_rgbfull,
				/* YUV709 full range => RGB limited range */
				NULL,
			},
			/* YUV709 full range => YUV601 */
			{
				/* YUV709 full range => YUV601 full range */
				NULL,
				/* YUV709 full range => YUV601 limited range */
				NULL,
			},
			/* YUV709 full range => YUV709 */
			{
				/* YUV709 full range => YUV709 full range */
				NULL,
				/* YUV709 full range => YUV709 limited range */
				NULL,
			},
		},
		/* YUV709 limited range */
		{
			/* YUV709 limited range => RGB */
			{
				/* YUV709 limited range => RGB full range */
				yuv709lim_to_rgbfull,
				/* YUV709 limited range => RGB limited range */
				yuv709lim_to_rgblim,
			},
			/* YUV709 limited range => YUV601 */
			{
				/* YUV709 limited range => YUV601 full range */
				NULL,
				/* YUV709 limited range => YUV601 limited range */
				NULL,
			},
			/* YUV709 limited range => YUV709 */
			{
				/* YUV709 limited range => YUV709 full range */
				NULL,
				/* YUV709 limited range => YUV709 limited range */
				NULL,
			},
		},
	},
};

enum cconv_fmt {
	FMT_RGB = 0,
	FMT_YUV601,
	FMT_YUV709
};

static inline enum cconv_fmt to_cconv_fmt(struct v4l2_mbus_framefmt *fmt)
{
	/* YUV format codes are within the 0x2xxx */
	if (fmt->code >= MEDIA_BUS_FMT_Y8_1X8 &&
	    fmt->code < MEDIA_BUS_FMT_SBGGR8_1X8) {
		if (fmt->ycbcr_enc == V4L2_YCBCR_ENC_709)
			return FMT_YUV709;
		else
			return FMT_YUV601;
	}

	/* All other formats are referred as RGB, indeed, demosaicing bloc
	 * generate RGB format
	 */
	return FMT_RGB;
};

#define FMT_STR(f) ({					\
	typeof(f) __f = (f);				\
	(__f) == FMT_RGB ? "RGB" :			\
	(__f) == FMT_YUV601 ? "YUV601" :		\
	(__f) == FMT_YUV709 ? "YUV709" : "?"; })

enum cconv_range {
	RANGE_FULL = 0,
	RANGE_LIMITED,
};

static inline enum cconv_range to_cconv_range(struct v4l2_mbus_framefmt *fmt)
{
	if (fmt->quantization == V4L2_QUANTIZATION_FULL_RANGE)
		return RANGE_FULL;

	return RANGE_LIMITED;
};

#define RANGE_STR(range) ((range) == RANGE_FULL ? "full" : "limited")

int dcmipp_colorconv_configure(struct device *dev,
			       struct v4l2_mbus_framefmt *sink,
			       struct v4l2_mbus_framefmt *src,
			       struct dcmipp_colorconv_config *cfg)
{
	const u32 *cconv_matrix;
	enum cconv_fmt sink_fmt;
	enum cconv_range sink_range;
	enum cconv_fmt src_fmt;
	enum cconv_range src_range;
	int i;

	sink_fmt = to_cconv_fmt(sink);
	sink_range = to_cconv_range(sink);
	src_fmt = to_cconv_fmt(src);
	src_range = to_cconv_range(src);

	if (sink_fmt == src_fmt &&
	    sink_range == src_range) {
		cfg->enable = false;
		return 0;
	}

	/* color conversion */
	cconv_matrix = cconv_matrices[sink_fmt][sink_range][src_fmt][src_range];

	dev_dbg(dev, "cconv_matrices[%d][%d][%d][%d]=%p\n",
		sink_fmt, sink_range, src_fmt, src_range, cconv_matrix);
	if (!cconv_matrix) {
		dev_err(dev, "Unsupported color conversion %s-%s => %s-%s\n",
			FMT_STR(sink_fmt), RANGE_STR(sink_range),
			FMT_STR(src_fmt), RANGE_STR(src_range));
		return -EINVAL;
	}

	dev_dbg(dev, "color conversion %s-%s => %s-%s\n",
		FMT_STR(sink_fmt), RANGE_STR(sink_range),
		FMT_STR(src_fmt), RANGE_STR(src_range));

	if (src_range == RANGE_LIMITED) {
		cfg->clamping = true;
		if (src->code >= MEDIA_BUS_FMT_Y8_1X8 && src->code < MEDIA_BUS_FMT_SBGGR8_1X8)
			cfg->clamping_as_rgb = false;
		else
			cfg->clamping_as_rgb = true;
	}

	cfg->enable = true;

	for (i = 0; i < ARRAY_SIZE(cfg->conv_matrix); i++)
		cfg->conv_matrix[i] = cconv_matrix[i];

	return 0;
}
