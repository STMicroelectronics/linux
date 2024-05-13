/* SPDX-License-Identifier: GPL-2.0+ */

#include <media/v4l2-jpeg.h>

#define JPEG_HEADER_SIZE	624
#define JPEG_QUANT_SIZE		64

struct hantro_jpeg_ctx {
	int width;
	int height;
	int quality;
	unsigned char *buffer;
	unsigned char hw_luma_qtable[JPEG_QUANT_SIZE];
	unsigned char hw_chroma_qtable[JPEG_QUANT_SIZE];
};

void hantro_jpeg_header_assemble(struct hantro_jpeg_ctx *ctx);
void hantro_jpeg_get_default_huffman_tables(struct v4l2_jpeg_reference *huffman_tables);
int hantro_jpeg_prepare_vlc_hw_table(struct v4l2_jpeg_header *header,
				     u8 *vlc_hw_table);
