// SPDX-License-Identifier: GPL-2.0
/*
 * Hantro VPU codec driver
 *
 * Copyright (C) 2018 Rockchip Electronics Co., Ltd.
 */

#include <asm/unaligned.h>
#include <media/v4l2-mem2mem.h>
#include "hantro_jpeg.h"
#include "hantro.h"
#include "hantro_v4l2.h"
#include "hantro_hw.h"
#include "hantro_h1_regs.h"

#define H1_JPEG_QUANT_TABLE_COUNT 16

static void hantro_h1_jpeg_enc_set_buffers(struct hantro_dev *vpu,
					   struct hantro_ctx *ctx,
					   struct vb2_buffer *src_buf,
					   struct vb2_buffer *dst_buf)
{
	const u32 src_addr_regs[] = { H1_REG_ADDR_IN_PLANE_0,
				      H1_REG_ADDR_IN_PLANE_1,
				      H1_REG_ADDR_IN_PLANE_2 };
	struct v4l2_pix_format_mplane *pix_fmt = &ctx->src_fmt;
	size_t luma_size;
	u32 size_left;
	int i;

	size_left = vb2_plane_size(dst_buf, 0) - ctx->vpu_dst_fmt->header_size;
	if (WARN_ON(vb2_plane_size(dst_buf, 0) < ctx->vpu_dst_fmt->header_size))
		size_left = 0;

	WARN_ON(pix_fmt->num_planes > 3);

	vepu_write_relaxed(vpu, vb2_dma_contig_plane_dma_addr(dst_buf, 0) +
				ctx->vpu_dst_fmt->header_size,
			   H1_REG_ADDR_OUTPUT_STREAM);
	vepu_write_relaxed(vpu, size_left, H1_REG_STR_BUF_LIMIT);

	luma_size = hantro_rounded_luma_size(ctx->src_fmt.width,
					     ctx->src_fmt.height);

	vepu_write_relaxed(vpu,
			   vb2_dma_contig_plane_dma_addr(src_buf, 0) +
			   src_buf->planes[0].data_offset,
			   src_addr_regs[0]);
	vepu_write_relaxed(vpu,
			   vb2_dma_contig_plane_dma_addr(src_buf, 0) +
			   src_buf->planes[0].data_offset +
			   luma_size,
			   src_addr_regs[1]);

	for (i = 1; i < pix_fmt->num_planes; ++i)
		/* Multiplanes. */
		vepu_write_relaxed(vpu,
				   vb2_dma_contig_plane_dma_addr(src_buf, i) +
				   src_buf->planes[i].data_offset,
				   src_addr_regs[i]);
}

static void
hantro_h1_jpeg_enc_set_qtable(struct hantro_dev *vpu,
			      unsigned char *luma_qtable,
			      unsigned char *chroma_qtable)
{
	u32 reg, i;
	__be32 *luma_qtable_p;
	__be32 *chroma_qtable_p;

	luma_qtable_p = (__be32 *)luma_qtable;
	chroma_qtable_p = (__be32 *)chroma_qtable;

	/*
	 * Quantization table registers must be written in contiguous blocks.
	 * DO NOT collapse the below two "for" loops into one.
	 */
	for (i = 0; i < H1_JPEG_QUANT_TABLE_COUNT; i++) {
		reg = get_unaligned_be32(&luma_qtable_p[i]);
		vepu_write_relaxed(vpu, reg, H1_REG_JPEG_LUMA_QUAT(i));
	}

	for (i = 0; i < H1_JPEG_QUANT_TABLE_COUNT; i++) {
		reg = get_unaligned_be32(&chroma_qtable_p[i]);
		vepu_write_relaxed(vpu, reg, H1_REG_JPEG_CHROMA_QUAT(i));
	}
}

int hantro_h1_jpeg_enc_run(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct hantro_jpeg_ctx jpeg_ctx;
	u32 reg;

	src_buf = hantro_get_src_buf(ctx);
	dst_buf = hantro_get_dst_buf(ctx);

	hantro_start_prepare_run(ctx);

	memset(&jpeg_ctx, 0, sizeof(jpeg_ctx));
	jpeg_ctx.buffer = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);
	jpeg_ctx.width = ctx->dst_fmt.width;
	jpeg_ctx.height = ctx->dst_fmt.height;
	jpeg_ctx.quality = ctx->jpeg_quality;
	hantro_jpeg_header_assemble(&jpeg_ctx);

	/* Switch to JPEG encoder mode before writing registers */
	vepu_write_relaxed(vpu, H1_REG_ENC_CTRL_ENC_MODE_JPEG,
			   H1_REG_ENC_CTRL);

	hantro_h1_set_src_img_ctrl(vpu, ctx);
	hantro_h1_jpeg_enc_set_buffers(vpu, ctx, &src_buf->vb2_buf,
				       &dst_buf->vb2_buf);
	hantro_h1_jpeg_enc_set_qtable(vpu, jpeg_ctx.hw_luma_qtable,
				      jpeg_ctx.hw_chroma_qtable);

	hantro_h1_set_color_conv(vpu, ctx);

	/* Make sure that all registers are written at this point. */
	hantro_h1_set_axi_ctrl(vpu, ctx);

	reg = H1_REG_ENC_CTRL_WIDTH(MB_WIDTH(ctx->src_fmt.width))
		| H1_REG_ENC_CTRL_HEIGHT(MB_HEIGHT(ctx->src_fmt.height))
		| H1_REG_ENC_CTRL_ENC_MODE_JPEG
		| H1_REG_ENC_PIC_INTRA
		| H1_REG_ENC_CTRL_EN_BIT;

	hantro_end_prepare_run(ctx);

	vepu_write(vpu, reg, H1_REG_ENC_CTRL);

	return 0;
}

void hantro_h1_jpeg_enc_done(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	u32 bytesused = vepu_read(vpu, H1_REG_STR_BUF_LIMIT) / 8;
	struct vb2_v4l2_buffer *dst_buf = hantro_get_dst_buf(ctx);

	vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
			      ctx->vpu_dst_fmt->header_size + bytesused);
}
