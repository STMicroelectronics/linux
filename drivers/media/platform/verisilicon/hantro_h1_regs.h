/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Hantro VPU codec driver
 *
 * Copyright 2018 Google LLC.
 *	Tomasz Figa <tfiga@chromium.org>
 */

#ifndef HANTRO_H1_REGS_H_
#define HANTRO_H1_REGS_H_

/* Encoder registers. */
#define H1_REG_INTERRUPT				0x004
#define     H1_REG_INTERRUPT_FRAME_RDY			BIT(2)
#define     H1_REG_INTERRUPT_DIS_BIT			BIT(1)
#define     H1_REG_INTERRUPT_BIT			BIT(0)
#define H1_REG_AXI_CTRL					0x008
#define     H1_REG_AXI_CTRL_OUTPUT_SWAP16		BIT(15)
#define     H1_REG_AXI_CTRL_INPUT_SWAP16		BIT(14)
#define     H1_REG_AXI_CTRL_BURST_LEN(x)		((x) << 8)
#define     H1_REG_AXI_CTRL_GATE_BIT			BIT(4)
#define     H1_REG_AXI_CTRL_OUTPUT_SWAP32		BIT(3)
#define     H1_REG_AXI_CTRL_INPUT_SWAP32		BIT(2)
#define     H1_REG_AXI_CTRL_OUTPUT_SWAP8		BIT(1)
#define     H1_REG_AXI_CTRL_INPUT_SWAP8			BIT(0)
#define H1_REG_DEVICE_CTRL				0x00c
#define     H1_REG_DEVICE_CTRL_SCALE_OUTPUT_SWAP8	BIT(27)
#define     H1_REG_DEVICE_CTRL_SCALE_OUTPUT_SWAP16	BIT(26)
#define     H1_REG_DEVICE_CTRL_SCALE_OUTPUT_SWAP32	BIT(25)
#define     H1_REG_DEVICE_CTRL_MV_OUTPUT_SWAP8		BIT(24)
#define     H1_REG_DEVICE_CTRL_MV_OUTPUT_SWAP16		BIT(23)
#define     H1_REG_DEVICE_CTRL_MV_OUTPUT_SWAP32		BIT(22)
#define     H1_REG_DEVICE_CTRL_INPUT_READ_1MB		BIT(21)
#define     H1_REG_DEVICE_CTRL_AXI_DUAL_CHANNEL		BIT(20)
#define H1_REG_ADDR_OUTPUT_STREAM			0x014
#define H1_REG_ADDR_OUTPUT_CTRL				0x018
#define H1_REG_ADDR_REF_LUMA				0x01c
#define H1_REG_ADDR_REF_CHROMA				0x020
#define H1_REG_ADDR_REC_LUMA				0x024
#define H1_REG_ADDR_REC_CHROMA				0x028
#define H1_REG_ADDR_IN_PLANE_0				0x02c
#define H1_REG_ADDR_IN_PLANE_1				0x030
#define H1_REG_ADDR_IN_PLANE_2				0x034
#define H1_REG_ENC_CTRL					0x038
#define     H1_REG_ENC_CTRL_TIMEOUT_EN			BIT(31)
#define     H1_REG_ENC_CTRL_MV_WRITE			BIT(30)
#define     H1_REG_ENC_CTRL_NAL_MODE_BIT		BIT(29)
#define     H1_REG_ENC_CTRL_WIDTH(w)			((w) << 19)
#define     H1_REG_ENC_CTRL_HEIGHT(h)			((h) << 10)
#define     H1_REG_ENC_REC_WRITE_BUFFER_4MB		BIT(7)
#define     H1_REG_ENC_REC_WRITE_DISABLE		BIT(6)
#define     H1_REG_ENC_PIC_INTER			(0x0 << 3)
#define     H1_REG_ENC_PIC_INTRA			(0x1 << 3)
#define     H1_REG_ENC_PIC_MVCINTER			(0x2 << 3)
#define     H1_REG_ENC_CTRL_ENC_MODE_H264		(0x3 << 1)
#define     H1_REG_ENC_CTRL_ENC_MODE_JPEG		(0x2 << 1)
#define     H1_REG_ENC_CTRL_ENC_MODE_VP8		(0x1 << 1)
#define     H1_REG_ENC_CTRL_EN_BIT			BIT(0)
#define H1_REG_IN_IMG_CTRL				0x03c
#define     H1_REG_IN_IMG_CTRL_ROW_LEN(x)		((x) << 12)
#define     H1_REG_IN_IMG_CTRL_OVRFLR_D4(x)		((x) << 10)
#define     H1_REG_IN_IMG_CTRL_OVRFLB(x)		((x) << 6)
#define     H1_REG_IN_IMG_CTRL_FMT(x)			((x) << 2)
#define     H1_REG_IN_IMG_CTRL_ROTATION(x)		((x))
#define H1_REG_ENC_CTRL0				0x040
#define    H1_REG_ENC_CTRL0_INIT_QP(x)			((x) << 26)
#define    H1_REG_ENC_CTRL0_SLICE_ALPHA(x)		((x) << 22)
#define    H1_REG_ENC_CTRL0_SLICE_BETA(x)		((x) << 18)
#define    H1_REG_ENC_CTRL0_CHROMA_QP_OFFSET(x)		((x) << 13)
#define    H1_REG_ENC_CTRL0_FILTER_DIS(x)		((x) << 5)
#define    H1_REG_ENC_CTRL0_IDR_PICID(x)		((x) << 1)
#define    H1_REG_ENC_CTRL0_CONSTR_INTRA_PRED		BIT(0)
#define H1_REG_ENC_CTRL1				0x044
#define    H1_REG_ENC_CTRL1_PPS_ID(x)			((x) << 24)
#define    H1_REG_ENC_CTRL1_INTRA_PRED_MODE(x)		((x) << 16)
#define    H1_REG_ENC_CTRL1_FRAME_NUM(x)		((x))
#define H1_REG_ENC_CTRL2				0x048
#define    H1_REG_ENC_CTRL2_DEBLOCKING_FILTER_MODE(x)	((x) << 30)
#define    H1_REG_ENC_CTRL2_H264_SLICE_SIZE(x)		((x) << 23)
#define    H1_REG_ENC_CTRL2_DISABLE_QUARTER_PIXMV	BIT(22)
#define    H1_REG_ENC_CTRL2_TRANS8X8_MODE_EN		BIT(21)
#define    H1_REG_ENC_CTRL2_CABAC_INIT_IDC(x)		((x) << 19)
#define    H1_REG_ENC_CTRL2_VP8_BOOLENC_ENABLE		BIT(18)
#define    H1_REG_ENC_CTRL2_ENTROPY_CABAC		BIT(18)
#define    H1_REG_ENC_CTRL2_H264_INTER4X4_MODE		BIT(17)
#define    H1_REG_ENC_CTRL2_H264_STREAM_MODE		BIT(16)
#define    H1_REG_ENC_CTRL2_INTRA16X16_MODE(x)		((x))
#define H1_REG_ENC_CTRL3				0x04c
#define    H1_REG_ENC_CTRL3_MUTIMV_EN			BIT(30)
#define    H1_REG_ENC_CTRL3_MV_PENALTY_1_4P(x)		((x) << 20)
#define    H1_REG_ENC_CTRL3_MV_PENALTY_4P(x)		((x) << 10)
#define    H1_REG_ENC_CTRL3_MV_PENALTY_1P(x)		((x))
#define H1_REG_ENC_CTRL4				0x050
#define    H1_REG_ENC_CTRL4_MV_PENALTY_16X8_8X16(x)	((x) << 20)
#define    H1_REG_ENC_CTRL4_MV_PENALTY_8X8(x)		((x) << 10)
#define    H1_REG_ENC_CTRL4_8X4_4X8(x)			((x))
#define H1_REG_ENC_CTRL5				0x054
#define    H1_REG_ENC_CTRL5_MACROBLOCK_PENALTY(x)	((x) << 24)
#define    H1_REG_ENC_CTRL5_COMPLETE_SLICES(x)		((x) << 16)
#define    H1_REG_ENC_CTRL5_INTER_MODE(x)		((x))
#define H1_REG_STR_HDR_REM_MSB				0x058
#define H1_REG_STR_HDR_REM_LSB				0x05c
#define H1_REG_STR_BUF_LIMIT				0x060
#define H1_REG_MAD_CTRL					0x064
#define    H1_REG_MAD_CTRL_QP_ADJUST(x)			((x) << 28)
#define    H1_REG_MAD_CTRL_MAD_THRESHOLD(x)		((x) << 22)
#define    H1_REG_MAD_CTRL_QP_SUM_DIV2(x)		((x))
#define H1_REG_ADDR_VP8_PROB_CNT			0x068
#define H1_REG_QP_VAL					0x06c
#define    H1_REG_QP_VAL_LUM(x)				((x) << 26)
#define    H1_REG_QP_VAL_MAX(x)				((x) << 20)
#define    H1_REG_QP_VAL_MIN(x)				((x) << 14)
#define    H1_REG_QP_VAL_CHECKPOINT_DISTAN(x)		((x))
#define H1_REG_VP8_QP_VAL(i)				(0x06c + ((i) * 0x4))
#define H1_REG_VP8_SEG0_QUANT_DC_Y1			0x06c
#define     H1_REG_VP8_SEG0_RND_DC_Y1(x)		(((x) & 0xff) << 23)
#define     H1_REG_VP8_SEG0_ZBIN_DC_Y1(x)		(((x) & 0x1ff) << 14)
#define     H1_REG_VP8_SEG0_QUT_DC_Y1(x)		(((x) & 0x3fff) << 0)
#define H1_REG_CHECKPOINT(i)				(0x070 + ((i) * 0x4))
#define     H1_REG_CHECKPOINT_CHECK0(x)			(((x) & 0xffff))
#define     H1_REG_CHECKPOINT_CHECK1(x)			(((x) & 0xffff) << 16)
#define     H1_REG_CHECKPOINT_RESULT(x)			((((x) >> (16 - 16 \
							 * (i & 1))) & 0xffff) \
							 * 32)
#define H1_REG_VP8_SEG0_QUANT_AC_Y1			0x070
#define     H1_REG_VP8_SEG0_RND_AC_Y1(x)		(((x) & 0xff) << 23)
#define     H1_REG_VP8_SEG0_ZBIN_AC_Y1(x)		(((x) & 0x1ff) << 14)
#define     H1_REG_VP8_SEG0_QUT_AC_Y1(x)		(((x) & 0x3fff) << 0)
#define H1_REG_VP8_SEG0_QUANT_DC_Y2			0x074
#define     H1_REG_VP8_SEG0_RND_DC_Y2(x)		(((x) & 0xff) << 23)
#define     H1_REG_VP8_SEG0_ZBIN_DC_Y2(x)		(((x) & 0x1ff) << 14)
#define     H1_REG_VP8_SEG0_QUT_DC_Y2(x)		(((x) & 0x3fff) << 0)
#define H1_REG_VP8_SEG0_QUANT_AC_Y2			0x078
#define     H1_REG_VP8_SEG0_RND_AC_Y2(x)		(((x) & 0xff) << 23)
#define     H1_REG_VP8_SEG0_ZBIN_AC_Y2(x)		(((x) & 0x1ff) << 14)
#define     H1_REG_VP8_SEG0_QUT_AC_Y2(x)		(((x) & 0x3fff) << 0)
#define H1_REG_VP8_SEG0_QUANT_DC_CHR			0x07c
#define     H1_REG_VP8_SEG0_RND_DC_CHR(x)		(((x) & 0xff) << 23)
#define     H1_REG_VP8_SEG0_ZBIN_DC_CHR(x)		(((x) & 0x1ff) << 14)
#define     H1_REG_VP8_SEG0_QUT_DC_CHR(x)		(((x) & 0x3fff) << 0)
#define H1_REG_VP8_SEG0_QUANT_AC_CHR			0x080
#define     H1_REG_VP8_SEG0_RND_AC_CHR(x)		(((x) & 0xff) << 23)
#define     H1_REG_VP8_SEG0_ZBIN_AC_CHR(x)		(((x) & 0x1ff) << 14)
#define     H1_REG_VP8_SEG0_QUT_AC_CHR(x)		(((x) & 0x3fff) << 0)
#define H1_REG_CHKPT_WORD_ERR(i)			(0x084 + ((i) * 0x4))
#define     H1_REG_CHKPT_WORD_ERR_CHK0(x)		(((x) & 0xffff))
#define     H1_REG_CHKPT_WORD_ERR_CHK1(x)		(((x) & 0xffff) << 16)
#define H1_REG_VP8_SEG0_QUANT_DQUT			0x084
#define     H1_REG_VP8_MV_REF_IDX1(x)			(((x) & 0x03) << 26)
#define     H1_REG_VP8_SEG0_DQUT_DC_Y2(x)		(((x) & 0x1ff) << 17)
#define     H1_REG_VP8_SEG0_DQUT_AC_Y1(x)		(((x) & 0x1ff) << 8)
#define     H1_REG_VP8_SEG0_DQUT_DC_Y1(x)		(((x) & 0xff) << 0)
#define H1_REG_H264_REFERENCE_IDX			0x084
#define     H1_REG_H264_MV_REF_IDX(x)			(((x) & 0x03) << 26)
#define H1_REG_VP8_SEG0_QUANT_DQUT_1			0x088
#define     H1_REG_VP8_SEGMENT_MAP_UPDATE		BIT(30)
#define     H1_REG_VP8_SEGMENT_EN			BIT(29)
#define     H1_REG_VP8_MV_REF_IDX2_EN			BIT(28)
#define     H1_REG_VP8_MV_REF_IDX2(x)			(((x) & 0x03) << 26)
#define     H1_REG_VP8_SEG0_DQUT_AC_CHR(x)		(((x) & 0x1ff) << 17)
#define     H1_REG_VP8_SEG0_DQUT_DC_CHR(x)		(((x) & 0xff) << 9)
#define     H1_REG_VP8_SEG0_DQUT_AC_Y2(x)		(((x) & 0x1ff) << 0)
#define H1_REG_H264_REFERENCE_CTRL			0x088
#define     H1_REG_H264_MARK_LONGTRERM			BIT(26)
#define H1_REG_VP8_BOOL_ENC				0x08c
#define H1_REG_CHKPT_DELTA_QP				0x090
#define     H1_REG_CHKPT_DELTA_QP_CHK0(x)		(((x) & 0x0f) << 0)
#define     H1_REG_CHKPT_DELTA_QP_CHK1(x)		(((x) & 0x0f) << 4)
#define     H1_REG_CHKPT_DELTA_QP_CHK2(x)		(((x) & 0x0f) << 8)
#define     H1_REG_CHKPT_DELTA_QP_CHK3(x)		(((x) & 0x0f) << 12)
#define     H1_REG_CHKPT_DELTA_QP_CHK4(x)		(((x) & 0x0f) << 16)
#define     H1_REG_CHKPT_DELTA_QP_CHK5(x)		(((x) & 0x0f) << 20)
#define     H1_REG_CHKPT_DELTA_QP_CHK6(x)		(((x) & 0x0f) << 24)
#define H1_REG_VP8_PENALTY_FILTER			0x090
#define     H1_REG_VP8_ZERO_MV_PENALTY_FOR_REF2(x)	(((x) & 0xff) << 24)
#define     H1_REG_VP8_FILTER_SHARPNESS(x)		(((x) & 0x07) << 21)
#define     H1_REG_VP8_FILTER_LEVEL(x)		(((x) & 0x3f) << 15)
#define     H1_REG_VP8_DCT_PARTITION_CNT(x)		(((x) & 0x03) << 13)
#define     H1_REG_VP8_BOOL_ENC_VALUE_BITS(x)		(((x) & 0x1f) << 8)
#define     H1_REG_VP8_BOOL_ENC_RANGE(x)		(((x) & 0xff) << 0)
#define H1_REG_RLC_CTRL					0x094
#define     H1_REG_RLC_CTRL_STR_OFFS_SHIFT		23
#define     H1_REG_RLC_CTRL_STR_OFFS_MASK		(0x3f << 23)
#define     H1_REG_RLC_CTRL_RLC_SUM(x)			((x))
#define H1_REG_MB_CTRL					0x098
#define     H1_REG_MB_CNT_OUT(x)			(((x) & 0xffff))
#define     H1_REG_MB_CNT_SET(x)			(((x) & 0xffff) << 16)
#define H1_REG_ADDR_NEXT_PIC				0x09c
#define	H1_REG_JPEG_LUMA_QUAT(i)			(0x100 + ((i) * 0x4))
#define	H1_REG_JPEG_CHROMA_QUAT(i)			(0x140 + ((i) * 0x4))
#define H1_REG_STABILIZATION_OUTPUT			0x0A0
#define H1_REG_ADDR_CABAC_TBL				0x0cc
#define H1_REG_ADDR_MV_OUT				0x0d0
#define H1_REG_RGB_YUV_COEFF_AB				0x0d4
#define   H1_REG_RGB_YUV_COEFF_A(x)			(((x) & 0xffff) << 0)
#define   H1_REG_RGB_YUV_COEFF_B(x)			(((x) & 0xffff) << 16)
#define H1_REG_RGB_YUV_COEFF_CE				0x0d8
#define   H1_REG_RGB_YUV_COEFF_C(x)			(((x) & 0xffff) << 0)
#define   H1_REG_RGB_YUV_COEFF_E(x)			(((x) & 0xffff) << 16)
#define H1_REG_RGB_YUV_COEFF_F_MASK			0x0dc
#define   H1_REG_RGB_YUV_COEFF_F(x)			(((x) & 0xffff) << 0)
#define   H1_REG_RGB_YUV_R_MASK_MSB(x)			(((x) & 0x1f) << 16)
#define   H1_REG_RGB_YUV_G_MASK_MSB(x)			(((x) & 0x1f) << 21)
#define   H1_REG_RGB_YUV_B_MASK_MSB(x)			(((x) & 0x1f) << 26)
#define H1_REG_INTRA_AREA_CTRL				0x0e0
#define H1_REG_CIR_INTRA_CTRL				0x0e4
#define H1_REG_INTRA_SLICE_BITMAP(i)			(0x0e8 + ((i) * 0x4))
#define H1_REG_ADDR_VP8_DCT_PART(i)			(0x0e8 + ((i) * 0x4))
#define H1_REG_FIRST_ROI_AREA				0x0f0
#define H1_REG_SECOND_ROI_AREA				0x0f4
#define H1_REG_QP_MV_MVC_CTRL				0x0f8
#define     H1_REG_ZERO_MV_FAVOR_D2(x)			(((x) & 0xf) << 28)
#define     H1_REG_PENALTY_4X4MV(x)			(((x) & 0x1ff) << 19)
#define H1_REG_VP8_INTRA_16X16_PENALTY(i)		(0x100 + ((i) * 0x4))
#define     H1_REG_VP8_INTRA_16X16_PENALTY_0(x)	(((x) & 0x3ff) << 0)
#define     HI_REG_VP8_INTRA_16X16_PENALTY_1(x)	(((x) & 0x3ff) << 10)
#define H1_REG_VP8_INTRA_4X4_PENALTY(i)			(0x108 + ((i) * 0x4))
#define     H1_REG_VP8_INTRA_4X4_PENALTY_0(x)		(((x) & 0x3ff) << 0)
#define     HI_REG_VP8_INTRA_4X4_PENALTY_1(x)		(((x) & 0x3ff) << 10)
#define H1_REG_ADDR_VP8_SEG_MAP				0x11c
#define H1_REG_VP8_SEG_QP(i)				(0x120 + ((i) * 0x4))
#define H1_REG_DMV_4P_1P_PENALTY(i)			(0x180 + ((i) * 0x4))
#define     H1_REG_DMV_4P_1P_PENALTY_BIT(x, i)		((x) << (i) * 8)
#define H1_REG_DMV_QPEL_PENALTY(i)			(0x200 + ((i) * 0x4))
#define     H1_REG_DMV_QPEL_PENALTY_BIT(x, i)		((x) << (i) * 8)
#define H1_REG_VP8_CTRL1				0x280
#define H1_REG_VP8_BIT_COST_GOLDEN			0x284
#define H1_REG_VP8_LOOP_FILTER_REF_DELTA		0x288
#define     H1_REG_VP8_LF_REF_DELTA_ALT_REF(x)		(((x) & 0x7f) << 21)
#define     H1_REG_VP8_LF_REF_DELTA_GOLDEN(x)		(((x) & 0x7f) << 14)
#define     H1_REG_VP8_LF_REF_DELTA_LAST_REF(x)		(((x) & 0x7f) << 7)
#define     H1_REG_VP8_LF_REF_DELTA_INTRA_MB(x)		(((x) & 0x7f) << 0)
#define H1_REG_VP8_LOOP_FILTER_MODE_DELTA		0x28c
#define     H1_REG_VP8_LF_MODE_DELTA_SPLITMV(x)		(((x) & 0x7f) << 21)
#define     H1_REG_VP8_LF_MODE_DELTA_NEWMV(x)		(((x) & 0x7f) << 14)
#define     H1_REG_VP8_LF_MODE_DELTA_ZEROMV(x)		(((x) & 0x7f) << 7)
#define     H1_REG_VP8_LF_MODE_DELTA_BPRED(x)		(((x) & 0x7f) << 0)

#endif /* HANTRO_H1_REGS_H_ */
