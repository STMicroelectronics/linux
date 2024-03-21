// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics SA 2023
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          for STMicroelectronics.
 */

#include "hantro.h"
#include "hantro_hw.h"
#include "hantro_h1_regs.h"

enum cconv_fmt {
	FMT_RGB = 0,
	FMT_YUV601,
	FMT_YUV709
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

#define RANGE_STR(range) ((range) == RANGE_FULL ? "full" : "limited")

/*
 * Color conversion based on coefficient a,b,c,e,f:
 *
 * Y  = (a * R + b * G + c * B) / 65536
 * Cb = (e * (B - Y)) / 65536 + 128
 * Cr = (f * (R - Y)) / 65536 + 128
 *
 * alternatively Cb=f(R,G,B), Cr=f(R,G,B):
 *
 * Y  = (  a                     *R  + b               *G  + c                    *B) / 65536
 * Cb = ( -(a * e / 65536)       *R  - (b * e / 65536) *G  + (e - (c * e / 65536))*B) / 65536 + 128
 * Cr = (  (f - (a * f / 65536)) *R  - (b * f / 65536) *G  - (c * f / 65536)      *B) / 65536 + 128
 *
 */

/*
 *  RGB full range => YUV601 full range
 *
 *  Y  =  0.2989 * R + 0.5866 * G + 0.1145 * B
 *  Cb = -0.1688 * R - 0.3312 * G + 0.5000 * B
 *  Cr =  0.5000 * R - 0.4184 * G - 0.0817 * B
 */
/*                                              a      b     c      e      f */
static const u32 rgbfull_to_yuv601full[] = {19589, 38443, 7504, 37008, 46740};

/*
 *  RGB full range => YUV709 full range
 *
 *  Y  =  0.2126 * R + 0.7152 * G + 0.0722 * B
 *  Cb = -0.1146 * R - 0.3854 * G + 0.4999 * B
 *  Cr =  0.4999 * R - 0.4541 * G - 0.0458 * B
 */
/*                                              a      b     c      e      f */
static const u32 rgbfull_to_yuv709full[] = {13933, 46871, 4732, 35317, 41615};

/*
 *  RGB limited range => YUV601 limited range
 *
 *  Y  =  0.2989 * R + 0.5866 * G + 0.1145 * B
 *  Cb = -0.1720 * R - 0.3375 * G + 0.5095 * B
 *  Cr =  0.5110 * R - 0.4275 * G - 0.0835 * B
 */
/*                                            a      b     c      e      f */
static const u32 rgblim_to_yuv601lim[] = {19589, 38443, 7504, 37712, 47767};

/*
 *  RGB limited range => YUV709 limited range
 *
 *  Y  =  0.2126 * R + 0.7152 * G + 0.0722 * B
 *  Cb = -0.1171 * R - 0.3940 * G + 0.5111 * B
 *  Cr =  0.5112 * R - 0.4643 * G - 0.0469 * B
 */
/*                                            a      b     c      e      f */
static const u32 rgblim_to_yuv709lim[] = {13933, 46871, 4732, 36100, 42550};

static const u32 *cconv_coeffss[2][3][2] = {
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
			NULL,
		},
		/* RGB full range => YUV709 */
		{
			/* RGB full range => YUV709 full range */
			rgbfull_to_yuv709full,
			/* RGB full range => YUV709 limited range */
			NULL,
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
};

static inline bool is_rgb16(struct v4l2_pix_format_mplane *fmt)
{
	return (fmt->pixelformat == V4L2_PIX_FMT_RGB565);
};

static inline bool is_rgb32(struct v4l2_pix_format_mplane *fmt)
{
	return ((fmt->pixelformat == V4L2_PIX_FMT_XBGR32) ||
		(fmt->pixelformat == V4L2_PIX_FMT_ABGR32) ||
		(fmt->pixelformat == V4L2_PIX_FMT_BGR32) ||
		(fmt->pixelformat == V4L2_PIX_FMT_RGBX32) ||
		(fmt->pixelformat == V4L2_PIX_FMT_RGBA32) ||
		(fmt->pixelformat == V4L2_PIX_FMT_BGRX32) ||
		(fmt->pixelformat == V4L2_PIX_FMT_BGRA32) ||
		(fmt->pixelformat == V4L2_PIX_FMT_XRGB32) ||
		(fmt->pixelformat == V4L2_PIX_FMT_ARGB32) ||
		(fmt->pixelformat == V4L2_PIX_FMT_RGB32));
};

static inline enum cconv_fmt to_cconv_fmt(struct v4l2_pix_format_mplane *fmt)
{
	if (is_rgb16(fmt) || is_rgb32(fmt))
		return FMT_RGB;

	/* All other formats are treated as YUV */

	if (fmt->colorspace == V4L2_COLORSPACE_SMPTE170M)
		return FMT_YUV601;
	else
		return FMT_YUV709;
};

static inline enum cconv_range to_cconv_range(struct v4l2_pix_format_mplane *fmt)
{
	if (fmt->quantization == V4L2_QUANTIZATION_FULL_RANGE)
		return RANGE_FULL;

	return RANGE_LIMITED;
};

static inline u32 to_cconv_mask(struct v4l2_pix_format_mplane *fmt)
{
	u32 r_mask_msb, g_mask_msb, b_mask_msb;

	/*
	 * ! see hantro_h1_set_axi_ctrl() for input swap
	 * of RGB 16 and 32 bits formats
	 */

	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_RGB565:
		/*
		 * <MSB>      <LSB>
		 * RRRRRGGGGGGBBBBB
		 * |    |     |
		 * |    |      \___ bit  4 <MSB of B>
		 * |     \_________ bit 10 <MSB of G>
		 * \_______________ bit 15 <MSB of R>
		 */
		r_mask_msb = 15;
		g_mask_msb = 10;
		b_mask_msb =  4;
		break;

	case V4L2_PIX_FMT_XBGR32:
	case V4L2_PIX_FMT_ABGR32:
	case V4L2_PIX_FMT_BGR32:
		/*
		 * <MSB>                      <LSB>
		 * XXXXXXXXRRRRRRRRGGGGGGGGBBBBBBBB
		 *         |       |       |
		 *         |       |       \__ bit  7 <MSB of B>
		 *         |        \_________ bit 15 <MSB of G>
		 *         \__________________ bit 23 <MSB of R>
		 */
		r_mask_msb = 23;
		g_mask_msb = 15;
		b_mask_msb =  7;
		break;

	case V4L2_PIX_FMT_RGBX32:
	case V4L2_PIX_FMT_RGBA32:
		/*
		 * <MSB>                      <LSB>
		 * XXXXXXXXBBBBBBBBGGGGGGGGRRRRRRRR
		 *         |       |       |
		 *         |       |       \__ bit  7 <MSB of R>
		 *         |        \_________ bit 15 <MSB of G>
		 *         \__________________ bit 23 <MSB of B>
		 */
		r_mask_msb = 7;
		g_mask_msb = 15;
		b_mask_msb = 23;
		break;

	case V4L2_PIX_FMT_BGRX32:
	case V4L2_PIX_FMT_BGRA32:
		/*
		 * <MSB>                      <LSB>
		 * RRRRRRRRGGGGGGGGBBBBBBBBXXXXXXXX
		 * |       |       |
		 * |       |       \__ bit 15 <MSB of B>
		 * |        \_________ bit 23 <MSB of G>
		 * \__________________ bit 31 <MSB of R>
		 */
		r_mask_msb = 31;
		g_mask_msb = 23;
		b_mask_msb = 15;
		break;

	case V4L2_PIX_FMT_XRGB32:
	case V4L2_PIX_FMT_ARGB32:
	case V4L2_PIX_FMT_RGB32:
		/*
		 * <MSB>                      <LSB>
		 * BBBBBBBBGGGGGGGGRRRRRRRRXXXXXXXX
		 * |       |       |
		 * |       |       \__ bit 15 <MSB of R>
		 * |        \_________ bit 23 <MSB of G>
		 * \__________________ bit 31 <MSB of B>
		 */
		r_mask_msb = 15;
		g_mask_msb = 23;
		b_mask_msb = 31;
		break;

	default:
		r_mask_msb = 0;
		g_mask_msb = 0;
		b_mask_msb = 0;
		break;
	}

	return (H1_REG_RGB_YUV_R_MASK_MSB(r_mask_msb) |
		H1_REG_RGB_YUV_G_MASK_MSB(g_mask_msb) |
		H1_REG_RGB_YUV_B_MASK_MSB(b_mask_msb));
};

void hantro_h1_set_color_conv(struct hantro_dev *vpu, struct hantro_ctx *ctx)
{
	const u32 *cconv_coeffs;
	enum cconv_fmt dst_fmt;
	enum cconv_range dst_range;
	enum cconv_fmt src_fmt;
	enum cconv_range src_range;

	dst_fmt = to_cconv_fmt(&ctx->dst_fmt);
	dst_range = to_cconv_range(&ctx->dst_fmt);
	src_fmt = to_cconv_fmt(&ctx->src_fmt);
	src_range = to_cconv_range(&ctx->src_fmt);

	if (src_fmt != FMT_RGB)
		return;

	if (dst_fmt == src_fmt &&
	    dst_range == src_range)
		return;

	/* color conversion */
	cconv_coeffs = cconv_coeffss[src_range][dst_fmt][dst_range];
	if (!cconv_coeffs) {
		vpu_debug(0, "Unsupported color conversion %s-%s => %s-%s\n",
			  FMT_STR(src_fmt), RANGE_STR(src_range),
			  FMT_STR(dst_fmt), RANGE_STR(dst_range));
		return;
	}

	vpu_debug(4, "color conversion %s-%s => %s-%s\n",
		  FMT_STR(src_fmt), RANGE_STR(src_range),
		  FMT_STR(dst_fmt), RANGE_STR(dst_range));

	vepu_write_relaxed(vpu, H1_REG_RGB_YUV_COEFF_A(cconv_coeffs[0]) |
				H1_REG_RGB_YUV_COEFF_B(cconv_coeffs[1]),
			   H1_REG_RGB_YUV_COEFF_AB);
	vepu_write_relaxed(vpu, H1_REG_RGB_YUV_COEFF_C(cconv_coeffs[2]) |
				H1_REG_RGB_YUV_COEFF_E(cconv_coeffs[3]),
			   H1_REG_RGB_YUV_COEFF_CE);
	vepu_write_relaxed(vpu, H1_REG_RGB_YUV_COEFF_F(cconv_coeffs[4]) |
				to_cconv_mask(&ctx->src_fmt),
			   H1_REG_RGB_YUV_COEFF_F_MASK);
}

void hantro_h1_set_axi_ctrl(struct hantro_dev *vpu, struct hantro_ctx *ctx)
{
	bool input_swap8, input_swap16, input_swap32;
	u32 reg;

	if (is_rgb32(&ctx->src_fmt)) {
		input_swap32 = 1;
		input_swap16 = 0;
		input_swap8  = 0;
	} else if (is_rgb16(&ctx->src_fmt)) {
		input_swap32 = 1;
		input_swap16 = 1;
		input_swap8  = 0;
	} else {
		/* YUV */
		input_swap32 = 1;
		input_swap16 = 1;
		input_swap8  = 1;
	}

	reg =     H1_REG_AXI_CTRL_OUTPUT_SWAP16
		| (input_swap16 ? H1_REG_AXI_CTRL_INPUT_SWAP16 : 0)
		| H1_REG_AXI_CTRL_BURST_LEN(16)
		| H1_REG_AXI_CTRL_OUTPUT_SWAP32
		| (input_swap32 ? H1_REG_AXI_CTRL_INPUT_SWAP32 : 0)
		| H1_REG_AXI_CTRL_OUTPUT_SWAP8
		| (input_swap8 ? H1_REG_AXI_CTRL_INPUT_SWAP8 : 0);

	vepu_write(vpu, reg, H1_REG_AXI_CTRL);
}

static inline unsigned int hantro_rotation(int rotation)
{
	if (rotation == 90)
		return 1;
	else if (rotation == 270)
		return 2;

	return 0;
}

void hantro_h1_set_src_img_ctrl(struct hantro_dev *vpu, struct hantro_ctx *ctx)
{
	u32 overfill_r, overfill_b;
	u32 reg;

	/*
	 * The format width and height are already macroblock aligned
	 * by .vidioc_s_fmt_vid_cap_mplane() callback. Destination
	 * format width and height can be further modified by
	 * .vidioc_s_selection(), and the width is 4-aligned.
	 */
	if (!ctx->rotation) {
		overfill_r = ctx->src_fmt.width - ctx->dst_fmt.width;
		overfill_b = ctx->src_fmt.height - ctx->dst_fmt.height;
	} else {
		/* FIXME Overfilled lines are seen when rotating image */
		overfill_r = 0;
		overfill_b = 0;
	}

	reg = H1_REG_IN_IMG_CTRL_ROW_LEN(ctx->src_fmt.width)
		| H1_REG_IN_IMG_CTRL_OVRFLR_D4(overfill_r / 4)
		| H1_REG_IN_IMG_CTRL_OVRFLB(overfill_b)
		| H1_REG_IN_IMG_CTRL_FMT(ctx->vpu_src_fmt->enc_fmt)
		| H1_REG_IN_IMG_CTRL_ROTATION(hantro_rotation(ctx->rotation));
	vepu_write_relaxed(vpu, reg, H1_REG_IN_IMG_CTRL);
}
