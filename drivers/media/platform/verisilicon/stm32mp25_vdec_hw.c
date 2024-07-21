// SPDX-License-Identifier: GPL-2.0
/*
 * STM32MP25 VDEC video decoder driver
 *
 * Copyright (C) STMicroelectronics SA 2022
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          for STMicroelectronics.
 *
 */

#include "hantro.h"

/*
 * Supported formats.
 */

static const struct hantro_fmt stm32mp25_vdec_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.codec_mode = HANTRO_MODE_NONE,
		.frmsize = {
			.min_width = 96,
			.max_width = 4080,
			.step_width = MB_DIM,
			.min_height = 96,
			.max_height = 4080,
			.step_height = MB_DIM,
		},
	},
	{
		.fourcc = V4L2_PIX_FMT_VP8_FRAME,
		.codec_mode = HANTRO_MODE_VP8_DEC,
		.max_depth = 2,
		.frmsize = {
			.min_width = 96,
			.max_width = 4080,
			.step_width = MB_DIM,
			.min_height = 96,
			.max_height = 4080,
			.step_height = MB_DIM,
		},
	},
	{
		.fourcc = V4L2_PIX_FMT_H264_SLICE,
		.codec_mode = HANTRO_MODE_H264_DEC,
		.max_depth = 2,
		.frmsize = {
			.min_width = 96,
			.max_width = 4080,
			.step_width = MB_DIM,
			.min_height = 96,
			.max_height = 4080,
			.step_height = MB_DIM,
		},
	},
};

/*
 * Supported codec ops.
 */

static const struct hantro_codec_ops stm32mp25_vdec_codec_ops[] = {
	[HANTRO_MODE_VP8_DEC] = {
		.run = hantro_g1_vp8_dec_run,
		.reset = hantro_g1_reset,
		.init = hantro_vp8_dec_init,
		.exit = hantro_vp8_dec_exit,
	},
	[HANTRO_MODE_H264_DEC] = {
		.run = hantro_g1_h264_dec_run,
		.reset = hantro_g1_reset,
		.init = hantro_h264_dec_init,
		.exit = hantro_h264_dec_exit,
	},
};

static const struct hantro_irq stm32mp25_irqs[] = {
	{ "vdec", hantro_g1_irq },
};

static const char * const stm32mp25_clk_names[] = { "vdec-clk" };

const struct hantro_variant stm32mp25_vdec_variant = {
	.dec_fmts = stm32mp25_vdec_fmts,
	.num_dec_fmts = ARRAY_SIZE(stm32mp25_vdec_fmts),
	.codec = HANTRO_VP8_DECODER | HANTRO_H264_DECODER,
	.codec_ops = stm32mp25_vdec_codec_ops,
	.irqs = stm32mp25_irqs,
	.num_irqs = ARRAY_SIZE(stm32mp25_irqs),
	.clk_names = stm32mp25_clk_names,
	.num_clocks = ARRAY_SIZE(stm32mp25_clk_names),
};
