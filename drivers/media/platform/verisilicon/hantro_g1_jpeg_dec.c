// SPDX-License-Identifier: GPL-2.0
/*
 * Hantro JPEG decoder driver
 *
 * Copyright (C) STMicroelectronics SA 2024
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          for STMicroelectronics.
 *
 * This code is inspired from coda-jpeg.c
 *
 */

#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <media/v4l2-jpeg.h>
#include <media/v4l2-mem2mem.h>
#include "hantro.h"
#include "hantro_g1_regs.h"
#include "hantro_hw.h"
#include "hantro_jpeg.h"

static void set_buffers(struct hantro_dev *vpu, struct hantro_ctx *ctx,
			struct vb2_buffer *src_buf,
			struct vb2_buffer *dst_buf,
			struct v4l2_jpeg_header *header)
{
	dma_addr_t src_dma, dst_dma;

	/* Source (stream) buffer. */
	src_dma = vb2_dma_contig_plane_dma_addr(src_buf, 0);

	/* Skip header */
	src_dma += header->ecs_offset;

	vdpu_write_relaxed(vpu, src_dma, G1_REG_ADDR_STR);

	/* Destination (decoded frame) buffer. */
	dst_dma = hantro_get_dec_buf_addr(ctx, dst_buf);
	vdpu_write_relaxed(vpu, dst_dma, G1_REG_ADDR_DST);
	vdpu_write_relaxed(vpu, dst_dma +
				ctx->dst_fmt.height * ctx->dst_fmt.width,
			   G1_REG_ADDR_DST_CHROMA);

	/* Auxiliary buffer prepared in hantro_jpeg_prepare_vlc_hw_table(). */
	vdpu_write_relaxed(vpu, ctx->jpeg_dec.priv.dma, G1_REG_ADDR_QTABLE);
}

static int to_jpeg_mode(int subsampling)
{
	switch (subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		return 3;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
		return 2;
	/*
	 * Currently unsupported:
	 * case V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY:
	 * case V4L2_JPEG_CHROMA_SUBSAMPLING_444:
	 * case V4L2_JPEG_CHROMA_SUBSAMPLING_411:
	 */
	default:
		return 2;
	}
}

static u32 to_pixelformat(int subsampling)
{
	switch (subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		return V4L2_PIX_FMT_NV16;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
		return V4L2_PIX_FMT_NV12;
	/*
	 * Currently unsupported:
	 * case V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY:
	 * case V4L2_JPEG_CHROMA_SUBSAMPLING_444:
	 * case V4L2_JPEG_CHROMA_SUBSAMPLING_411:
	 */
	default:
		return 0;
	}
}

static int write_vlc_code_lengths(struct hantro_ctx *ctx,
				  struct v4l2_jpeg_header *header)
{
	struct hantro_dev *vpu = ctx->dev;
	u32 reg;
	u8 *bits1;
	u8 *bits2;

	if (header->huffman_tables[0].length < 16 ||
	    header->huffman_tables[1].length < 16 ||
	    header->huffman_tables[2].length < 16 ||
	    header->huffman_tables[3].length < 16) {
		dev_err(vpu->dev, "Invalid huffman table size: at least 16 bytes expected");
		return -EINVAL;
	}

	/* First 16 bits are BITS code lengths */
	bits1 = (u8 *)header->huffman_tables[2].start;/* AC luma */
	bits2 = (u8 *)header->huffman_tables[3].start;/* AC chroma  */

	/* Hardware requires that AC1 registers contains luma table */
	if (header->scan->component[0].ac_entropy_coding_table_selector == 1) /* Ta[0] == 1 */
		swap(bits1, bits2);

	/* AC1 table code lengths (luma) */
	reg = G1_REG_DEC_JPEG_VLC_AC1_CODE1_CNT(bits1[0]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE2_CNT(bits1[1]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE3_CNT(bits1[2]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE4_CNT(bits1[3]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE5_CNT(bits1[4]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE6_CNT(bits1[5]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_A);

	reg = G1_REG_DEC_JPEG_VLC_AC1_CODE7_CNT(bits1[6]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE8_CNT(bits1[7]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE9_CNT(bits1[8]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE10_CNT(bits1[9]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_B);

	reg = G1_REG_DEC_JPEG_VLC_AC1_CODE11_CNT(bits1[10]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE12_CNT(bits1[11]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE13_CNT(bits1[12]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE14_CNT(bits1[13]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_C);

	reg = G1_REG_DEC_JPEG_VLC_AC1_CODE15_CNT(bits1[14]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE16_CNT(bits1[15]) |

	/* AC2 table code lengths (the not-luma) */
	      G1_REG_DEC_JPEG_VLC_AC2_CODE1_CNT(bits2[0]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE2_CNT(bits2[1]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE3_CNT(bits2[2]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE4_CNT(bits2[3]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_D);

	reg = G1_REG_DEC_JPEG_VLC_AC2_CODE5_CNT(bits2[4]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE6_CNT(bits2[5]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE7_CNT(bits2[6]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE8_CNT(bits2[7]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_E);

	reg = G1_REG_DEC_JPEG_VLC_AC2_CODE9_CNT(bits2[8]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE10_CNT(bits2[9]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE11_CNT(bits2[10]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE12_CNT(bits2[11]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_F);

	reg = G1_REG_DEC_JPEG_VLC_AC2_CODE13_CNT(bits2[12]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE14_CNT(bits2[13]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE15_CNT(bits2[14]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE16_CNT(bits2[15]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_G);

	bits1 = (u8 *)header->huffman_tables[0].start;/* DC luma */
	bits2 = (u8 *)header->huffman_tables[1].start;/* DC chroma  */

	/* Hardware requires that DC1 registers contains luma table */
	if (header->scan->component[0].dc_entropy_coding_table_selector == 1) /* Td[0] == 1 */
		swap(bits1, bits2);

	/* DC1 table code lengths (luma) */
	reg = G1_REG_DEC_JPEG_VLC_DC1_CODE1_CNT(bits1[0]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE2_CNT(bits1[1]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE3_CNT(bits1[2]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE4_CNT(bits1[3]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE5_CNT(bits1[4]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE6_CNT(bits1[5]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE7_CNT(bits1[6]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE8_CNT(bits1[7]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_DC_CODE_LENGTHS_A);

	reg = G1_REG_DEC_JPEG_VLC_DC1_CODE9_CNT(bits1[8]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE10_CNT(bits1[9]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE11_CNT(bits1[10]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE12_CNT(bits1[11]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE13_CNT(bits1[12]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE14_CNT(bits1[13]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE15_CNT(bits1[14]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE16_CNT(bits1[15]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_DC_CODE_LENGTHS_B);

	/* DC2 table code lengths (not-luma) */
	reg = G1_REG_DEC_JPEG_VLC_DC2_CODE1_CNT(bits2[0]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE2_CNT(bits2[1]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE3_CNT(bits2[2]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE4_CNT(bits2[3]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE5_CNT(bits2[4]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE6_CNT(bits2[5]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE7_CNT(bits2[6]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE8_CNT(bits2[7]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_DC_CODE_LENGTHS_C);

	reg = G1_REG_DEC_JPEG_VLC_DC2_CODE9_CNT(bits2[8]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE10_CNT(bits2[9]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE11_CNT(bits2[10]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE12_CNT(bits2[11]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE13_CNT(bits2[12]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE14_CNT(bits2[13]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE15_CNT(bits2[14]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE16_CNT(bits2[15]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_DC_CODE_LENGTHS_D);

	return 0;
}

static int stream_bit_offset(struct v4l2_jpeg_header *header)
{
	int page_ptr, word_ptr, bit_ptr;
	u32 ecs_offset = header->ecs_offset;

	page_ptr = ecs_offset / 256;
	word_ptr = (ecs_offset % 256) / 4;
	if (page_ptr & 1)
		word_ptr += 64;
	bit_ptr = (ecs_offset % 4) * 8;
	if (word_ptr & 1)
		bit_ptr += 32;

	return bit_ptr;
}

int hantro_g1_jpeg_dec_run(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	size_t height = ctx->dst_fmt.height;
	size_t width = ctx->dst_fmt.width;
	struct v4l2_jpeg_scan_header scan_header;
	struct v4l2_jpeg_reference quantization_tables[4] = { };
	struct v4l2_jpeg_reference huffman_tables[4] = { };
	struct v4l2_jpeg_header header;
	u32 mb_width, mb_height;
	u32 dst_pixelformat;
	u32 jpeg_size;
	u32 reg;
	int ret;
	u32 i;

	hantro_g1_check_idle(vpu);

	hantro_start_prepare_run(ctx);

	src_buf = hantro_get_src_buf(ctx);
	dst_buf = hantro_get_dst_buf(ctx);

	jpeg_size = vb2_get_plane_payload(&src_buf->vb2_buf, 0);

	memset(&header, 0, sizeof(header));
	header.scan = &scan_header;
	header.quantization_tables = quantization_tables;
	header.huffman_tables = huffman_tables;

	ret = v4l2_jpeg_parse_header(vb2_plane_vaddr(&src_buf->vb2_buf, 0),
				     jpeg_size,
				     &header);
	if (ret < 0) {
		dev_err(vpu->dev, "Error parsing JPEG stream markers\n");
		hantro_end_prepare_run(ctx);
		return ret;
	}

	/* Check JPEG width/height */
	if (header.frame.width != width ||
	    MB_HEIGHT(header.frame.height) != MB_HEIGHT(height)) {
		dev_err(vpu->dev,
			"Resolution mismatch: %dx%d (JPEG) versus %lux%lu (user)",
			header.frame.width, header.frame.height, width, height);
		hantro_end_prepare_run(ctx);
		return -EINVAL;
	}

	/* Check JPEG number of components */
	if (header.frame.num_components > V4L2_JPEG_MAX_COMPONENTS) {
		dev_err(vpu->dev, "JPEG number of components should be <=%d",
			V4L2_JPEG_MAX_COMPONENTS);
		hantro_end_prepare_run(ctx);
		return -EINVAL;
	}

	/* Check JPEG quantization tables */
	if (header.quantization_tables[3].start) {
		dev_err(vpu->dev, "Maximum 3 quantization tables are supported\n");
		hantro_end_prepare_run(ctx);
		return -EINVAL;
	}

	for (i = 0; i < 3; i++) {
		if (!header.quantization_tables[i].start)
			continue;

		if (header.quantization_tables[i].length != JPEG_QUANT_SIZE) {
			dev_err(vpu->dev, "Only 8-bit quantization tables supported\n");
			hantro_end_prepare_run(ctx);
			return -EINVAL;
		}
	}

	/* Check JPEG huffman tables */
	if (!header.num_dht)
		hantro_jpeg_get_default_huffman_tables(header.huffman_tables);

	for (i = 0; i < 4; i++) {
		if (!header.huffman_tables[i].start) {
			dev_err(vpu->dev, "Missing Huffman[%d] table\n", i);
			hantro_end_prepare_run(ctx);
			return -EINVAL;
		}
		/* AC tables should be between 17 -> 178, DC between 17 -> 28 */
		if (header.huffman_tables[i].length < 17 ||
		    header.huffman_tables[i].length > 178 ||
		    ((i & 2) == 0 && header.huffman_tables[i].length > 28)) {
			dev_err(vpu->dev,
				"invalid Huffman table %d length: %zu\n",
				i, header.huffman_tables[i].length);
			hantro_end_prepare_run(ctx);
			return -EINVAL;
		}
	}

	/* Check destination pixel format match with JPEG subsampling */
	dst_pixelformat = to_pixelformat(header.frame.subsampling);
	if (!dst_pixelformat) {
		dev_err(vpu->dev, "Unsupported JPEG subsampling (%d)\n",
			header.frame.subsampling);
		hantro_end_prepare_run(ctx);
		return -EINVAL;
	}

	if (dst_pixelformat != ctx->dst_fmt.pixelformat) {
		dev_err(vpu->dev, "Decoder pixel format mismatch (expected %4.4s but got %4.4s)\n",
			(char *)&dst_pixelformat, (char *)&ctx->dst_fmt.pixelformat);
		hantro_end_prepare_run(ctx);
		return -EINVAL;
	}

	/* Write VLC table code lengths */
	ret = write_vlc_code_lengths(ctx, &header);
	if (ret) {
		hantro_end_prepare_run(ctx);
		return ret;
	}

	/* Prepare VLC QP/AC/DC hardware tables */
	ret = hantro_jpeg_prepare_vlc_hw_table(&header, ctx->jpeg_dec.priv.cpu);
	if (ret) {
		dev_err(vpu->dev, "Error when preparing VLC table");
		hantro_end_prepare_run(ctx);
		return ret;
	}

	reg = G1_REG_DEC_CTRL0_DEC_MODE(3) |
	      G1_REG_DEC_CTRL0_FILTERING_DIS;
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_CTRL0);

	/* Frame dimensions */
	mb_width = MB_WIDTH(width);
	mb_height = MB_HEIGHT(height);
	reg = G1_REG_DEC_CTRL1_PIC_MB_WIDTH(mb_width) |
	      G1_REG_DEC_CTRL1_PIC_MB_HEIGHT_P(mb_height) |
	      G1_REG_DEC_CTRL1_PIC_MB_W_EXT(mb_width >> 9) |
	      G1_REG_DEC_CTRL1_PIC_MB_H_EXT(mb_height >> 8);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_CTRL1);

	reg = G1_REG_DEC_CTRL2_JPEG_MODE(to_jpeg_mode(header.frame.subsampling)) |
	      G1_REG_DEC_CTRL2_JPEG_STREAM_ALL;

	/*
	 * For luminance the first table is always used
	 * For chrominance: check jpeg header
	 */
	reg |= G1_REG_DEC_CTRL2_JPEG_QTABLES(header.scan->num_components);

	if (header.scan->num_components > 0) {
		if (header.scan->component[1].dc_entropy_coding_table_selector == 1)/* Td[1]==1 */
			reg |= G1_REG_DEC_CTRL2_CB_DC_VLCTABLE;

		if (header.scan->component[1].ac_entropy_coding_table_selector == 1)/* Ta[1]==1 */
			reg |= G1_REG_DEC_CTRL2_CB_AC_VLCTABLE;
	}
	if (header.scan->num_components > 1) {
		if (header.scan->component[2].dc_entropy_coding_table_selector == 1)/* Td[2]==1 */
			reg |= G1_REG_DEC_CTRL2_CR_DC_VLCTABLE;

		if (header.scan->component[2].ac_entropy_coding_table_selector == 1)/* Ta[2]==1 */
			reg |= G1_REG_DEC_CTRL2_CR_AC_VLCTABLE;
	}

	reg |= G1_REG_DEC_CTRL2_STRM_START_BIT(stream_bit_offset(&header));

	if (header.restart_interval) {
		reg |= G1_REG_DEC_CTRL2_SYNC_MARKER_E;
		vdpu_write_relaxed(vpu,
				   G1_REG_DEC_CTRL5_PJPEG_REST_FREQ(header.restart_interval),
				   G1_REG_DEC_CTRL5);
	}

	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_CTRL2);

	reg = G1_REG_DEC_CTRL3_STREAM_LEN(jpeg_size);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_CTRL3);

	/* Vertical alignment */
	reg = IS_ALIGNED(header.frame.height, MB_DIM) ?
		0 : G1_REG_DEC_CTRL4_PJPEG_FILDOWN_E;

	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_CTRL4);

	/* Disable slice mode */
	vdpu_write_relaxed(vpu, G1_REG_JPEG_CTRL_SLICE_H(0), G1_REG_JPEG_CTRL);

	set_buffers(vpu, ctx, &src_buf->vb2_buf, &dst_buf->vb2_buf, &header);

	hantro_end_prepare_run(ctx);

	/* Start decoding! */
	vdpu_write_relaxed(vpu,
			   G1_REG_CONFIG_DEC_AXI_RD_ID(0xffu) |
			   G1_REG_CONFIG_DEC_OUT_ENDIAN |
			   G1_REG_CONFIG_DEC_STRENDIAN_E |
			   G1_REG_CONFIG_DEC_MAX_BURST(16) |
			   G1_REG_CONFIG_DEC_OUTSWAP32_E |
			   G1_REG_CONFIG_DEC_INSWAP32_E |
			   G1_REG_CONFIG_DEC_STRSWAP32_E |
			   G1_REG_CONFIG_DEC_CLK_GATE_E |
			   G1_REG_CONFIG_DEC_STRSWAP32_E,
			   G1_REG_CONFIG);
	vdpu_write(vpu, G1_REG_INTERRUPT_DEC_E, G1_REG_INTERRUPT);

	return 0;
}
