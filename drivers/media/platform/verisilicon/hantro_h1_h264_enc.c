// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics SA 2023
 * Authors:
 *	Andrzej Pietrasiewicz <andrzej.p@collabora.com>
 *
 * Some portions of code are derived from
 * https://chromium.googlesource.com/chromiumos/third_party/kernel/+/refs/heads/chromeos-5.10/
 * drivers/staging/media/hantro/rk3399_vpu_hw_h264_enc.c
 *
 * which is licensed GPL-2.0
 *
 * Rockchip rk3399 VPU codec driver
 *
 * Copyright (C) 2016 Rockchip Electronics Co., Ltd.
 *	Alpha Lin <Alpha.Lin@rock-chips.com>
 *	Jung Zhao <jung.zhao@rock-chips.com>
 */
#include "hantro.h"
#include "hantro_hw.h"
#include "hantro_h1_regs.h"

/* threshold of MBs count to disable quarter pixel mv for encode speed, e.g. 1920x1080 */
#define MAX_MB_COUNT_TO_DISABLE_QUARTER_PIXEL_MV	8160

/* threshold of MBs count to disable multi mv in one macro block, e.g. 1920x1080 */
#define MAX_MB_COUNT_TO_DISABLE_SPLIT_MV		8160

#define HANTRO_H264_QP_NUM				52

#define ZERO_16x16_MV_FAVOR_DIV2			10

/* H.264 motion estimation parameters */
static const u32 h264_prev_mode_favor[HANTRO_H264_QP_NUM] = {
	7, 7, 8, 8, 9, 9, 10, 10, 11, 12, 12, 13, 14, 15, 16, 17, 18,
	19, 20, 21, 22, 24, 25, 27, 29, 30, 32, 34, 36, 38, 41, 43, 46,
	49, 51, 55, 58, 61, 65, 69, 73, 78, 82, 87, 93, 98, 104, 110,
	117, 124, 132, 140
};

/* sqrt(2^((qp-12)/3))*8 */
static const u32 h264_diff_mv_penalty[HANTRO_H264_QP_NUM] = {
	2, 2, 3, 3, 3, 4, 4, 4, 5, 6,
	6, 7, 8, 9, 10, 11, 13, 14, 16, 18,
	20, 23, 26, 29, 32, 36, 40, 45, 51, 57,
	64, 72, 81, 91, 102, 114, 128, 144, 161, 181,
	203, 228, 256, 287, 323, 362, 406, 456, 512, 575,
	645, 724
};

/* 31*sqrt(2^((qp-12)/3))/4 */
static const u32 h264_diff_mv_penalty4p[HANTRO_H264_QP_NUM] = {
	2, 2, 2, 3, 3, 3, 4, 4, 5, 5,
	6, 7, 8, 9, 10, 11, 12, 14, 16, 17,
	20, 22, 25, 28, 31, 35, 39, 44, 49, 55,
	62, 70, 78, 88, 98, 110, 124, 139, 156, 175,
	197, 221, 248, 278, 312, 351, 394, 442, 496, 557,
	625, 701
};

static const u32 h264_intra16_favor[HANTRO_H264_QP_NUM] = {
	24, 24, 24, 26, 27, 30, 32, 35, 39, 43, 48, 53, 58, 64, 71, 78,
	85, 93, 102, 111, 121, 131, 142, 154, 167, 180, 195, 211, 229,
	248, 271, 296, 326, 361, 404, 457, 523, 607, 714, 852, 1034,
	1272, 1588, 2008, 2568, 3318, 4323, 5672, 7486, 9928, 13216,
	17648
};

static const u32 h264_inter_favor[HANTRO_H264_QP_NUM] = {
	40, 40, 41, 42, 43, 44, 45, 48, 51, 53, 55, 60, 62, 67, 69, 72,
	78, 84, 90, 96, 110, 120, 135, 152, 170, 189, 210, 235, 265,
	297, 335, 376, 420, 470, 522, 572, 620, 670, 724, 770, 820,
	867, 915, 970, 1020, 1076, 1132, 1180, 1230, 1275, 1320, 1370
};

static const u32 h264_skip_sad_penalty[HANTRO_H264_QP_NUM] = {
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 233, 205, 182, 163,
	146, 132, 120, 109, 100,  92,  84,  78,  71,  66,  61,  56,  52,  48,
	44,  41,  38,  35,  32,  30,  27,  25,  23,  21,  19,  17,  15,  14,
	12,  11,   9,   8,   7,   5,   4,   3,   2,   1
};

static const s32 h264_dmv_penalty[128] = { /* 4*sqrt(i*4*6) */
	0,   19,   27,   33,   39,   43,   48,   51,   55,   58,
	61,   64,   67,   70,   73,   75,   78,   80,   83,   85,
	87,   89,   91,   93,   96,   97,   99,  101,  103,  105,
	107,  109,  110,  112,  114,  115,  117,  119,  120,  122,
	123,  125,  126,  128,  129,  131,  132,  134,  135,  137,
	138,  139,  141,  142,  144,  145,  146,  147,  149,  150,
	151,  153,  154,  155,  156,  157,  159,  160,  161,  162,
	163,  165,  166,  167,  168,  169,  170,  171,  173,  174,
	175,  176,  177,  178,  179,  180,  181,  182,  183,  184,
	185,  186,  187,  188,  189,  190,  192,  192,  193,  194,
	195,  196,  197,  198,  199,  200,  201,  202,  203,  204,
	205,  206,  207,  208,  209,  210,  211,  211,  212,  213,
	214,  215,  216,  217,  218,  219,  219,  220
};

static s32 exp_golomb_signed(s32 val)
{
	s32 tmp = 0;

	if (val > 0)
		val = 2 * val;
	else
		val = -2 * val + 1;
	while (val >> ++tmp)
		;
	return tmp * 2 - 1;
}

static void hantro_h1_h264_enc_set_params(struct hantro_dev *vpu, struct hantro_ctx *ctx,
					  struct v4l2_ctrl_h264_encode_params *params, int qp)
{
	unsigned int mb_width = MB_WIDTH(ctx->src_fmt.width);
	unsigned int mb_height = MB_HEIGHT(ctx->src_fmt.height);
	u32 prev_mode_favor = h264_prev_mode_favor[qp];
	s32 scaler;
	u32 skip_penalty;
	u8 dmv_qpel_penalty[128];
	u32 reg;
	int i;

	reg = H1_REG_ENC_CTRL0_INIT_QP(params->pic_init_qp_minus26 + 26)
		| H1_REG_ENC_CTRL0_SLICE_ALPHA(params->slice_alpha_c0_offset_div2)
		| H1_REG_ENC_CTRL0_SLICE_BETA(params->slice_beta_offset_div2)
		| H1_REG_ENC_CTRL0_CHROMA_QP_OFFSET(params->chroma_qp_index_offset)
		| H1_REG_ENC_CTRL0_IDR_PICID(params->idr_pic_id);
	vepu_write_relaxed(vpu, reg, H1_REG_ENC_CTRL0);

	reg = H1_REG_ENC_CTRL1_PPS_ID(params->pic_parameter_set_id)
		| H1_REG_ENC_CTRL1_INTRA_PRED_MODE(prev_mode_favor)
		| H1_REG_ENC_CTRL1_FRAME_NUM(params->frame_num);
	vepu_write_relaxed(vpu, reg, H1_REG_ENC_CTRL1);

	reg = 0;
	if (mb_width * mb_height > MAX_MB_COUNT_TO_DISABLE_QUARTER_PIXEL_MV)
		reg = H1_REG_ENC_CTRL2_DISABLE_QUARTER_PIXMV;
	reg |= H1_REG_ENC_CTRL2_DEBLOCKING_FILTER_MODE(params->disable_deblocking_filter_idc)
		| H1_REG_ENC_CTRL2_H264_SLICE_SIZE(0)
		| H1_REG_ENC_CTRL2_CABAC_INIT_IDC(params->cabac_init_idc)
		| H1_REG_ENC_CTRL2_H264_INTER4X4_MODE
		| H1_REG_ENC_CTRL2_INTRA16X16_MODE(h264_intra16_favor[qp]);
	if (params->flags & V4L2_H264_ENCODE_FLAG_ENTROPY_CABAC)
		reg |= H1_REG_ENC_CTRL2_ENTROPY_CABAC;
	vepu_write_relaxed(vpu, reg, H1_REG_ENC_CTRL2);

	reg = H1_REG_ENC_CTRL3_MV_PENALTY_1P(h264_diff_mv_penalty[qp])
		| H1_REG_ENC_CTRL3_MV_PENALTY_1_4P(h264_diff_mv_penalty[qp])
		| H1_REG_ENC_CTRL3_MV_PENALTY_4P(h264_diff_mv_penalty4p[qp]);
	if (mb_width * mb_height <= MAX_MB_COUNT_TO_DISABLE_SPLIT_MV)
		reg |= H1_REG_ENC_CTRL3_MUTIMV_EN;
	vepu_write_relaxed(vpu, reg, H1_REG_ENC_CTRL3);

	scaler = max(1U, 200 / (mb_width + mb_height));
	skip_penalty = min(255U, h264_skip_sad_penalty[qp] * scaler);
	reg = H1_REG_ENC_CTRL5_MACROBLOCK_PENALTY(skip_penalty)
		| H1_REG_ENC_CTRL5_COMPLETE_SLICES(0)
		| H1_REG_ENC_CTRL5_INTER_MODE(h264_inter_favor[qp]);
	vepu_write_relaxed(vpu, reg, H1_REG_ENC_CTRL5);

	/*
	 * The hardware only writes at 64-bit aligned addresses. If there's
	 * a header generated with the CPU in the destination (encoded) buffer
	 * then potentially up to 7 bytes will be overwritten by the hardware.
	 * To preserve them, they are passed to the hardware via
	 * H1_REG_STR_HDR_REM_MSB/LSB and the first free bit to be used by the hw
	 * is passed via H1_REG_RLC_CTRL. This driver does not generate any headers,
	 * so there's no bytes to pass to the hw and the hw-generated data
	 * may be written at offset 0.
	 */
	vepu_write_relaxed(vpu, 0, H1_REG_STR_HDR_REM_MSB);
	vepu_write_relaxed(vpu, 0, H1_REG_STR_HDR_REM_LSB);
	vepu_write_relaxed(vpu, 0, H1_REG_RLC_CTRL);

	/* Don't use MAD */
	reg = H1_REG_MAD_CTRL_QP_ADJUST(0)
		| H1_REG_MAD_CTRL_MAD_THRESHOLD(0)
		| H1_REG_MAD_CTRL_QP_SUM_DIV2(0);
	vepu_write_relaxed(vpu, reg, H1_REG_MAD_CTRL);

	reg = H1_REG_QP_VAL_LUM(qp)
		| H1_REG_QP_VAL_MAX(51)
		| H1_REG_QP_VAL_MIN(0)
		| H1_REG_QP_VAL_CHECKPOINT_DISTAN(0);
	vepu_write_relaxed(vpu, reg, H1_REG_QP_VAL);

	/* Don't use ROI */
	vepu_write_relaxed(vpu, 0, H1_REG_FIRST_ROI_AREA);
	vepu_write_relaxed(vpu, 0, H1_REG_SECOND_ROI_AREA);

	/* Don't use stabilization */
	vepu_write_relaxed(vpu, 0, H1_REG_STABILIZATION_OUTPUT);

	/* Disable CIR */
	vepu_write_relaxed(vpu, 0, H1_REG_CIR_INTRA_CTRL);

	/* Disable intra area */
	vepu_write_relaxed(vpu, 0, H1_REG_INTRA_AREA_CTRL);

	reg = H1_REG_ZERO_MV_FAVOR_D2(ZERO_16x16_MV_FAVOR_DIV2)
		| H1_REG_PENALTY_4X4MV(0);
	vepu_write_relaxed(vpu, reg, H1_REG_QP_MV_MVC_CTRL);

	for (i = 0; i < 128; i++)
		dmv_qpel_penalty[i] = min(255, exp_golomb_signed(i));

	/* See rk3399_vpu_hw_h264_enc.c mentioned in the copyright section */
	for (i = 0; i < 128; i += 4) {
		reg = H1_REG_DMV_4P_1P_PENALTY_BIT((h264_dmv_penalty[i] + 1) / 2, 3)
			| H1_REG_DMV_4P_1P_PENALTY_BIT((h264_dmv_penalty[i + 1] + 1) / 2, 2)
			| H1_REG_DMV_4P_1P_PENALTY_BIT((h264_dmv_penalty[i + 2] + 1) / 2, 1)
			| H1_REG_DMV_4P_1P_PENALTY_BIT((h264_dmv_penalty[i + 3] + 1) / 2, 0);
		vepu_write_relaxed(vpu, reg, H1_REG_DMV_4P_1P_PENALTY(i / 4));

		reg = H1_REG_DMV_QPEL_PENALTY_BIT(dmv_qpel_penalty[i], 3)
			| H1_REG_DMV_QPEL_PENALTY_BIT(dmv_qpel_penalty[i + 1], 2)
			| H1_REG_DMV_QPEL_PENALTY_BIT(dmv_qpel_penalty[i + 2], 1)
			| H1_REG_DMV_QPEL_PENALTY_BIT(dmv_qpel_penalty[i + 3], 0);
		vepu_write_relaxed(vpu, reg, H1_REG_DMV_QPEL_PENALTY(i / 4));
	}
}

static void hantro_h1_h264_enc_set_buffers(struct hantro_dev *vpu, struct hantro_ctx *ctx,
					   struct v4l2_ctrl_h264_encode_params *params)
{
	const u32 src_addr_regs[] = { H1_REG_ADDR_IN_PLANE_0,
				      H1_REG_ADDR_IN_PLANE_1,
				      H1_REG_ADDR_IN_PLANE_2 };
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct v4l2_pix_format_mplane *src_fmt = &ctx->src_fmt;
	size_t luma_size;
	dma_addr_t dst_dma;
	size_t dst_size;
	int i;

	src_buf = hantro_get_src_buf(ctx);
	dst_buf = hantro_get_dst_buf(ctx);

	luma_size = hantro_rounded_luma_size(ctx->src_fmt.width,
					     ctx->src_fmt.height);

	dst_dma = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	dst_size = vb2_plane_size(&dst_buf->vb2_buf, 0);

	/* Destination buffer */
	vepu_write_relaxed(vpu, dst_dma, H1_REG_ADDR_OUTPUT_STREAM);
	vepu_write_relaxed(vpu, dst_size, H1_REG_STR_BUF_LIMIT);

	/* Auxiliary buffers */
	memset(ctx->h264_enc.nal_table.cpu, 0, ctx->h264_enc.nal_table.size);
	vepu_write_relaxed(vpu, ctx->h264_enc.nal_table.dma, H1_REG_ADDR_OUTPUT_CTRL);

	vepu_write_relaxed(vpu, ctx->h264_enc.mv_buf.dma, H1_REG_ADDR_MV_OUT);

	vepu_write_relaxed(vpu, ctx->h264_enc.cabac_ctx[params->cabac_init_idc].dma,
			   H1_REG_ADDR_CABAC_TBL);

	/* remember current frame timestamp */
	ctx->h264_enc.reference_ts[ctx->h264_enc.reconstr_idx] = src_buf->vb2_buf.timestamp;

	/* reference buffers */
	if (params->slice_type) {
		/* intra */
		if (params->flags & V4L2_H264_ENCODE_FLAG_MARK_LONGTERM) {
			ctx->h264_enc.ltr_idx = ctx->h264_enc.reconstr_idx;
			vepu_write_relaxed(vpu, H1_REG_H264_MARK_LONGTRERM,
					   H1_REG_H264_REFERENCE_CTRL);
		}
	} else {
		/* inter */
		if (params->reference_ts == ctx->h264_enc.reference_ts[ctx->h264_enc.last_idx]) {
			i = ctx->h264_enc.last_idx;
			vepu_write_relaxed(vpu, H1_REG_H264_MV_REF_IDX(0),
					   H1_REG_H264_REFERENCE_IDX);
		} else {
			i = ctx->h264_enc.ltr_idx;
			vepu_write_relaxed(vpu, H1_REG_H264_MV_REF_IDX(1),
					   H1_REG_H264_REFERENCE_IDX);
		}

		vepu_write_relaxed(vpu, ctx->h264_enc.luma_internal[i].dma,
				   H1_REG_ADDR_REF_LUMA);
		vepu_write_relaxed(vpu, ctx->h264_enc.chroma_internal[i].dma,
				   H1_REG_ADDR_REF_CHROMA);
	}

	/* Reconstruction buffers */
	i = ctx->h264_enc.reconstr_idx;
	vepu_write_relaxed(vpu, ctx->h264_enc.luma_internal[i].dma, H1_REG_ADDR_REC_LUMA);
	vepu_write_relaxed(vpu, ctx->h264_enc.chroma_internal[i].dma, H1_REG_ADDR_REC_CHROMA);

	/* Source buffer */
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
		/* Multiplanes */
		vepu_write_relaxed(vpu,
				   vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, i) +
				   src_buf->vb2_buf.planes[i].data_offset,
				   src_addr_regs[i]);
}

static int hantro_h1_validate_references(struct hantro_ctx *ctx,
					 struct v4l2_ctrl_h264_encode_params *params)
{
	u64 last_ts, ltr_ts, ref_ts;

	/* last_idx exists no matter what because we're a P frame */
	last_ts = ctx->h264_enc.reference_ts[ctx->h264_enc.last_idx];
	ref_ts = params->reference_ts;

	/* last frame is OK as reference */
	if (ref_ts == last_ts)
		return 0;

	/* if long term reference exists then maybe it is requested? */
	if (ctx->h264_enc.ltr_idx >= 0) {
		ltr_ts = ctx->h264_enc.reference_ts[ctx->h264_enc.ltr_idx];
		/* long term reference is OK */
		if (ref_ts == ltr_ts)
			return 0;
	}

	return -EINVAL;
}

int hantro_h1_h264_enc_run(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct v4l2_ctrl_h264_encode_params *params;
	struct v4l2_ctrl_h264_encode_rc *rc;
	u32 reg;

	hantro_start_prepare_run(ctx);

	params = hantro_get_ctrl(ctx, V4L2_CID_STATELESS_H264_ENCODE_PARAMS);
	if (WARN_ON(!params))
		return -EINVAL;

	/* This driver only supports I and P slices */
	if (params->slice_type != V4L2_H264_SLICE_TYPE_I &&
	    params->slice_type != V4L2_H264_SLICE_TYPE_P)
		return -EINVAL;

	/* This driver only supports NALU type 5 for I slices */
	if (params->slice_type == V4L2_H264_SLICE_TYPE_I &&
	    params->nalu_type != V4L2_H264_NAL_CODED_SLICE_IDR_PIC)
		return -EINVAL;

	/* This driver only supports NALU type 1 for P slices */
	if (params->slice_type == V4L2_H264_SLICE_TYPE_P &&
	    params->nalu_type != V4L2_H264_NAL_CODED_SLICE_NON_IDR_PIC)
		return -EINVAL;

	rc = hantro_get_ctrl(ctx, V4L2_CID_STATELESS_H264_ENCODE_RC);
	if (WARN_ON(!rc))
		return -EINVAL;

	if (params->slice_type == V4L2_H264_SLICE_TYPE_P &&
	    hantro_h1_validate_references(ctx, params))
		return -EINVAL;

	/* Program the hardware */
	vepu_write_relaxed(vpu, H1_REG_ENC_CTRL_ENC_MODE_H264, H1_REG_ENC_CTRL);

	hantro_h1_h264_enc_set_params(vpu, ctx, params, rc->qp);
	hantro_h1_set_src_img_ctrl(vpu, ctx);
	hantro_h1_h264_enc_set_buffers(vpu, ctx, params);

	hantro_h1_set_color_conv(vpu, ctx);

	reg = H1_REG_DEVICE_CTRL_SCALE_OUTPUT_SWAP8
		| H1_REG_DEVICE_CTRL_SCALE_OUTPUT_SWAP16
		| H1_REG_DEVICE_CTRL_SCALE_OUTPUT_SWAP32
		| H1_REG_DEVICE_CTRL_MV_OUTPUT_SWAP8
		| H1_REG_DEVICE_CTRL_MV_OUTPUT_SWAP16
		| H1_REG_DEVICE_CTRL_MV_OUTPUT_SWAP32;
	vepu_write(vpu, reg, H1_REG_DEVICE_CTRL);

	/* Make sure that all registers are written at this point. */
	hantro_h1_set_axi_ctrl(vpu, ctx);

	/* Start the hardware */
	reg = H1_REG_ENC_CTRL_TIMEOUT_EN
		| H1_REG_ENC_CTRL_MV_WRITE
		| H1_REG_ENC_CTRL_NAL_MODE_BIT
		| H1_REG_ENC_CTRL_WIDTH(MB_WIDTH(ctx->src_fmt.width))
		| H1_REG_ENC_CTRL_HEIGHT(MB_HEIGHT(ctx->src_fmt.height))
		| H1_REG_ENC_CTRL_ENC_MODE_H264
		| H1_REG_ENC_CTRL_EN_BIT;
	if (params->slice_type) {
		struct vb2_v4l2_buffer *dst_buf;

		reg |= H1_REG_ENC_PIC_INTRA;
		dst_buf = hantro_get_dst_buf(ctx);
		dst_buf->flags |= V4L2_BUF_FLAG_KEYFRAME;
	} else {
		reg |= H1_REG_ENC_PIC_INTER;
	}

	/* Kick the watchdog and start encoding */
	hantro_end_prepare_run(ctx);

	/* Start the hardware */
	vepu_write(vpu, reg, H1_REG_ENC_CTRL);

	return 0;
}

static inline void hantro_h1_bump_reconstr_idx(struct hantro_ctx *ctx)
{
	ctx->h264_enc.reconstr_idx++;
	ctx->h264_enc.reconstr_idx %= HANTRO_H264_NUM_INTERNAL_FRAMES;
}

void hantro_h1_h264_enc_done(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct vb2_v4l2_buffer *dst_buf;
	u32 encoded_size;

	dst_buf = hantro_get_dst_buf(ctx);

	encoded_size = vepu_read(vpu, H1_REG_STR_BUF_LIMIT) / 8;

	vb2_set_plane_payload(&dst_buf->vb2_buf, 0, encoded_size);

	ctx->h264_enc.last_prev_idx = ctx->h264_enc.last_idx;
	ctx->h264_enc.last_idx = ctx->h264_enc.reconstr_idx;

	hantro_h1_bump_reconstr_idx(ctx);
	if (ctx->h264_enc.reconstr_idx == ctx->h264_enc.ltr_idx)
		hantro_h1_bump_reconstr_idx(ctx);
}
