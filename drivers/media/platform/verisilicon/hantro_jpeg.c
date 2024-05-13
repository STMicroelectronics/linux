// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) Collabora, Ltd.
 *
 * Based on GSPCA and CODA drivers:
 * Copyright (C) Jean-Francois Moine (http://moinejf.free.fr)
 * Copyright (C) 2014 Philipp Zabel, Pengutronix
 * Copyright (C) STMicroelectronics SA 2024
 *	Hugues Fruchet <hugues.fruchet@foss.st.com>
 *
 */

#include <linux/align.h>
#include <linux/build_bug.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <media/v4l2-jpeg.h>
#include "hantro_jpeg.h"
#include "hantro.h"

#define LUMA_QUANT_OFF		25
#define CHROMA_QUANT_OFF	90
#define HEIGHT_OFF		159
#define WIDTH_OFF		161

#define HUFF_LUMA_DC_OFF	178
#define HUFF_LUMA_AC_OFF	211
#define HUFF_CHROMA_DC_OFF	394
#define HUFF_CHROMA_AC_OFF	427

static const u32 hw_reorder[] = {
	 0,  8, 16, 24,  1,  9, 17, 25,
	32, 40, 48, 56, 33, 41, 49, 57,
	 2, 10, 18, 26,  3, 11, 19, 27,
	34, 42, 50, 58, 35, 43, 51, 59,
	 4, 12, 20, 28,  5, 13, 21, 29,
	36, 44, 52, 60, 37, 45, 53, 61,
	 6, 14, 22, 30,  7, 15, 23, 31,
	38, 46, 54, 62, 39, 47, 55, 63
};

/* For simplicity, we keep a pre-formatted JPEG header,
 * and we'll use fixed offsets to change the width, height
 * quantization tables, etc.
 */
static const unsigned char hantro_jpeg_header[] = {
	/* SOI */
	0xff, 0xd8,

	/* JFIF-APP0 */
	0xff, 0xe0, 0x00, 0x10, 0x4a, 0x46, 0x49, 0x46,
	0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01,
	0x00, 0x00,

	/* DQT */
	0xff, 0xdb, 0x00, 0x84,

	0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	/* SOF */
	0xff, 0xc0, 0x00, 0x11, 0x08, 0x00, 0xf0, 0x01,
	0x40, 0x03, 0x01, 0x22, 0x00, 0x02, 0x11, 0x01,
	0x03, 0x11, 0x01,

	/* DHT */
	0xff, 0xc4, 0x00, 0x1f, 0x00,

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,

	/* DHT */
	0xff, 0xc4, 0x00, 0xb5, 0x10,

	0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	/* DHT */
	0xff, 0xc4, 0x00, 0x1f, 0x01,

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,

	/* DHT */
	0xff, 0xc4, 0x00, 0xb5, 0x11,

	0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	/* COM */
	0xff, 0xfe, 0x00, 0x03, 0x00,

	/* SOS */
	0xff, 0xda, 0x00, 0x0c, 0x03, 0x01, 0x00, 0x02,
	0x11, 0x03, 0x11, 0x00, 0x3f, 0x00,
};

/*
 * JPEG_HEADER_SIZE is used in other parts of the driver in lieu of
 * "sizeof(hantro_jpeg_header)". The two must be equal.
 */
static_assert(sizeof(hantro_jpeg_header) == JPEG_HEADER_SIZE);

/*
 * hantro_jpeg_header is padded with a COM segment, so that the payload
 * of the SOS segment (the entropy-encoded image scan), which should
 * trail the whole header, is 8-byte aligned for the hardware to write
 * to directly.
 */
static_assert(IS_ALIGNED(sizeof(hantro_jpeg_header), 8),
	      "Hantro JPEG header size needs to be 8-byte aligned.");

static unsigned char jpeg_scale_qp(const unsigned char qp, int scale)
{
	unsigned int temp;

	temp = DIV_ROUND_CLOSEST((unsigned int)qp * scale, 100);
	if (temp <= 0)
		temp = 1;
	if (temp > 255)
		temp = 255;

	return (unsigned char)temp;
}

static void
jpeg_scale_quant_table(unsigned char *file_q_tab,
		       unsigned char *reordered_q_tab,
		       const unsigned char *tab, int scale)
{
	int i;

	BUILD_BUG_ON(ARRAY_SIZE(v4l2_jpeg_zigzag_scan_index) != JPEG_QUANT_SIZE);
	BUILD_BUG_ON(ARRAY_SIZE(hw_reorder) != JPEG_QUANT_SIZE);

	for (i = 0; i < JPEG_QUANT_SIZE; i++) {
		file_q_tab[i] = jpeg_scale_qp(tab[v4l2_jpeg_zigzag_scan_index[i]], scale);
		reordered_q_tab[i] = jpeg_scale_qp(tab[hw_reorder[i]], scale);
	}
}

static void jpeg_set_quality(struct hantro_jpeg_ctx *ctx)
{
	int scale;
	/*
	 * Non-linear scaling factor:
	 * [5,50] -> [1000..100], [51,100] -> [98..0]
	 */
	if (ctx->quality < 50)
		scale = 5000 / ctx->quality;
	else
		scale = 200 - 2 * ctx->quality;

	BUILD_BUG_ON(ARRAY_SIZE(v4l2_jpeg_ref_table_luma_qt) != JPEG_QUANT_SIZE);
	BUILD_BUG_ON(ARRAY_SIZE(v4l2_jpeg_ref_table_chroma_qt) != JPEG_QUANT_SIZE);
	BUILD_BUG_ON(ARRAY_SIZE(ctx->hw_luma_qtable) != JPEG_QUANT_SIZE);
	BUILD_BUG_ON(ARRAY_SIZE(ctx->hw_chroma_qtable) != JPEG_QUANT_SIZE);

	jpeg_scale_quant_table(ctx->buffer + LUMA_QUANT_OFF,
			       ctx->hw_luma_qtable,
			       (const unsigned char *)v4l2_jpeg_ref_table_luma_qt, scale);
	jpeg_scale_quant_table(ctx->buffer + CHROMA_QUANT_OFF,
			       ctx->hw_chroma_qtable,
			       (const unsigned char *)v4l2_jpeg_ref_table_chroma_qt, scale);
}

void hantro_jpeg_header_assemble(struct hantro_jpeg_ctx *ctx)
{
	char *buf = ctx->buffer;

	memcpy(buf, hantro_jpeg_header,
	       sizeof(hantro_jpeg_header));

	buf[HEIGHT_OFF + 0] = ctx->height >> 8;
	buf[HEIGHT_OFF + 1] = ctx->height;
	buf[WIDTH_OFF + 0] = ctx->width >> 8;
	buf[WIDTH_OFF + 1] = ctx->width;

	memcpy(buf + HUFF_LUMA_DC_OFF, v4l2_jpeg_ref_table_luma_dc_ht, V4L2_JPEG_REF_HT_DC_LEN);
	memcpy(buf + HUFF_LUMA_AC_OFF, v4l2_jpeg_ref_table_luma_ac_ht, V4L2_JPEG_REF_HT_AC_LEN);
	memcpy(buf + HUFF_CHROMA_DC_OFF, v4l2_jpeg_ref_table_chroma_dc_ht, V4L2_JPEG_REF_HT_DC_LEN);
	memcpy(buf + HUFF_CHROMA_AC_OFF, v4l2_jpeg_ref_table_chroma_ac_ht, V4L2_JPEG_REF_HT_AC_LEN);

	jpeg_set_quality(ctx);
}

void hantro_jpeg_get_default_huffman_tables(struct v4l2_jpeg_reference *huffman_tables)
{
	huffman_tables[0].start = (u8 *)v4l2_jpeg_ref_table_luma_dc_ht;
	huffman_tables[0].length = sizeof(v4l2_jpeg_ref_table_luma_dc_ht);
	huffman_tables[1].start = (u8 *)v4l2_jpeg_ref_table_chroma_dc_ht;
	huffman_tables[1].length = sizeof(v4l2_jpeg_ref_table_chroma_dc_ht);
	huffman_tables[2].start = (u8 *)v4l2_jpeg_ref_table_luma_ac_ht;
	huffman_tables[2].length = sizeof(v4l2_jpeg_ref_table_luma_ac_ht);
	huffman_tables[3].start = (u8 *)v4l2_jpeg_ref_table_chroma_ac_ht;
	huffman_tables[3].length = sizeof(v4l2_jpeg_ref_table_chroma_ac_ht);
}

#define JPEG_HUFFVAL_OFF 16
#define JPEG_AC_SIZE 162
#define JPEG_DC_SIZE 12

/* Decoder VLC hardware table */
struct hantro_jpeg_vlc_hw_table {
	u8 qp_tables[3][JPEG_QUANT_SIZE];
	u8 ac_tables[2][JPEG_AC_SIZE];
	u8 dc_tables[2][JPEG_DC_SIZE];
	u8 padding[4];
};

int hantro_jpeg_prepare_vlc_hw_table(struct v4l2_jpeg_header *header,
				     u8 *vlc_hw_table)
{
	u32 i, j = 0;
	u8 *huffvals1;
	u8 *huffvals2;
	u32 hufflength1, hufflength2;
	u8 *table;
	u32 *hw_table_le;
	struct hantro_jpeg_vlc_hw_table *hw_table =
		(struct hantro_jpeg_vlc_hw_table *)vlc_hw_table;

	/* Only 3 quantization tables supported */
	if (header->quantization_tables[3].start)
		return -EINVAL;

	/* If chroma table is missing, take the luma */
	if (!header->quantization_tables[1].start) {
		header->quantization_tables[1].start =
			header->quantization_tables[0].start;
		header->quantization_tables[1].length =
			header->quantization_tables[0].length;
	}

	/* If second chroma table is missing, take the first one */
	if (!header->quantization_tables[2].start) {
		header->quantization_tables[2].start =
			header->quantization_tables[1].start;
		header->quantization_tables[2].length =
			header->quantization_tables[1].length;
	}

	/* QP tables for all components */
	for (j = 0; j < 3; j++) {
		table = header->quantization_tables[j].start;
		if (!table)
			continue;

		if (header->quantization_tables[j].length != JPEG_QUANT_SIZE)
			return -EINVAL;

		for (i = 0; i < JPEG_QUANT_SIZE; i++)
			hw_table->qp_tables[j][v4l2_jpeg_zigzag_scan_index[i]] = table[i];
	}

	/* HUFFVAL code values are located after the first 16 bytes BITS code lengths */
	if (!header->huffman_tables[2].start ||
	    header->huffman_tables[2].length != JPEG_AC_SIZE + JPEG_HUFFVAL_OFF)
		return -EINVAL;

	huffvals1 = (u8 *)(header->huffman_tables[2].start + JPEG_HUFFVAL_OFF);
	hufflength1 = header->huffman_tables[2].length - JPEG_HUFFVAL_OFF;

	if (!header->huffman_tables[3].start ||
	    header->huffman_tables[3].length != JPEG_AC_SIZE + JPEG_HUFFVAL_OFF)
		return -EINVAL;

	huffvals2 = (u8 *)(header->huffman_tables[3].start + JPEG_HUFFVAL_OFF);
	hufflength2 = header->huffman_tables[3].length - JPEG_HUFFVAL_OFF;

	/* Hardware requires that AC Table 1 is luma table */
	if (header->scan->component[0].ac_entropy_coding_table_selector == 1) { /* Ta[0] == 1 */
		swap(huffvals1, huffvals2);
		swap(hufflength1, hufflength2);
	}

	/* AC table values */
	memcpy(&hw_table->ac_tables[0], huffvals1,
	       min(hufflength1, (u32)JPEG_AC_SIZE));
	memcpy(&hw_table->ac_tables[1], huffvals2,
	       min(hufflength2, (u32)JPEG_AC_SIZE));

	/* HUFFVAL code values are located after the first 16 bytes BITS code lengths */
	if (!header->huffman_tables[0].start ||
	    header->huffman_tables[0].length != JPEG_DC_SIZE + JPEG_HUFFVAL_OFF)
		return -EINVAL;

	huffvals1 = (u8 *)(header->huffman_tables[0].start + JPEG_HUFFVAL_OFF);
	hufflength1 = header->huffman_tables[0].length - JPEG_HUFFVAL_OFF;

	if (!header->huffman_tables[1].start ||
	    header->huffman_tables[1].length != JPEG_DC_SIZE + JPEG_HUFFVAL_OFF)
		return -EINVAL;

	huffvals2 = (u8 *)(header->huffman_tables[1].start + JPEG_HUFFVAL_OFF);
	hufflength2 = header->huffman_tables[1].length - JPEG_HUFFVAL_OFF;

	/* Hardware requires that DC Table 1 is luma table */
	if (header->scan->component[0].dc_entropy_coding_table_selector == 1) { /* Td[0] == 1 */
		swap(huffvals1, huffvals2);
		swap(hufflength1, hufflength2);
	}

	/* DC table values */
	memcpy(&hw_table->dc_tables[0], huffvals1,
	       min(hufflength1, (u32)JPEG_DC_SIZE));
	memcpy(&hw_table->dc_tables[1], huffvals2,
	       min(hufflength2, (u32)JPEG_DC_SIZE));

	/* Padding of 4 last bytes */
	memset(hw_table->padding, 0, 4);

	/* Hardware requires 32-bits little-endian ordering */
	hw_table_le = (u32 *)hw_table;
	for (i = 0; i < (sizeof(*hw_table) / sizeof(u32)); i++)
		hw_table_le[i] = __swab32(hw_table_le[i]);

	return 0;
}

void hantro_jpeg_dec_exit(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct hantro_jpeg_dec_hw_ctx *jpeg_dec = &ctx->jpeg_dec;
	struct hantro_aux_buf *priv = &jpeg_dec->priv;

	dma_free_coherent(vpu->dev, priv->size, priv->cpu, priv->dma);
}

int hantro_jpeg_dec_init(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct hantro_jpeg_dec_hw_ctx *jpeg_dec = &ctx->jpeg_dec;
	struct hantro_aux_buf *priv = &jpeg_dec->priv;
	struct hantro_jpeg_vlc_hw_table *hw_table;

	priv->cpu = dma_alloc_coherent(vpu->dev, sizeof(*hw_table), &priv->dma,
				       GFP_KERNEL);
	if (!priv->cpu)
		return -ENOMEM;

	priv->size = sizeof(*hw_table);

	return 0;
}

