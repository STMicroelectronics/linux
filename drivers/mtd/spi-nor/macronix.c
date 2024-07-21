// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 */

#include <linux/mtd/spi-nor.h>

#include "core.h"

#define SPINOR_OP_MXIC_RD_ANY_REG	0x71	     /* Read volatile configuration register 2 */
#define SPINOR_OP_MXIC_WR_ANY_REG	0x72	     /* Write volatile configuration register 2 */
#define SPINOR_REG_MXIC_CR2_MODE	0x00000000   /* CR2 address for setting octal DTR mode */
#define SPINOR_REG_MXIC_CR2_DC		0x00000300   /* CR2 address for setting dummy cycles */
#define SPINOR_REG_MXIC_OPI_DTR_EN	0x2	     /* Enable Octal DTR */
#define SPINOR_REG_MXIC_SPI_EN		0x0	     /* Enable SPI */
#define SPINOR_REG_MXIC_ADDR_BYTES	4	     /* Fixed R/W volatile address bytes to 4 */
/* Convert dummy cycles to bit pattern */
#define SPINOR_REG_MXIC_DC(p) \
		((20 - (p)) / 2)

/* Macronix SPI NOR flash operations. */
#define MXIC_NOR_WR_ANY_REG_OP(naddr, addr, ndata, buf)			\
	SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_MXIC_WR_ANY_REG, 0),	\
		   SPI_MEM_OP_ADDR(naddr, addr, 0),			\
		   SPI_MEM_OP_NO_DUMMY,					\
		   SPI_MEM_OP_DATA_OUT(ndata, buf, 0))

static int
mx25l25635_post_bfpt_fixups(struct spi_nor *nor,
			    const struct sfdp_parameter_header *bfpt_header,
			    const struct sfdp_bfpt *bfpt)
{
	/*
	 * MX25L25635F supports 4B opcodes but MX25L25635E does not.
	 * Unfortunately, Macronix has re-used the same JEDEC ID for both
	 * variants which prevents us from defining a new entry in the parts
	 * table.
	 * We need a way to differentiate MX25L25635E and MX25L25635F, and it
	 * seems that the F version advertises support for Fast Read 4-4-4 in
	 * its BFPT table.
	 */
	if (bfpt->dwords[BFPT_DWORD(5)] & BFPT_DWORD5_FAST_READ_4_4_4)
		nor->flags |= SNOR_F_4B_OPCODES;

	return 0;
}

static const struct spi_nor_fixups mx25l25635_fixups = {
	.post_bfpt = mx25l25635_post_bfpt_fixups,
};

static const struct flash_info macronix_nor_parts[] = {
	/* Macronix */
	{ "mx25l512e",   INFO(0xc22010, 0, 64 * 1024,   1)
		NO_SFDP_FLAGS(SECT_4K) },
	{ "mx25l2005a",  INFO(0xc22012, 0, 64 * 1024,   4)
		NO_SFDP_FLAGS(SECT_4K) },
	{ "mx25l4005a",  INFO(0xc22013, 0, 64 * 1024,   8)
		NO_SFDP_FLAGS(SECT_4K) },
	{ "mx25l8005",   INFO(0xc22014, 0, 64 * 1024,  16) },
	{ "mx25l1606e",  INFO(0xc22015, 0, 64 * 1024,  32)
		NO_SFDP_FLAGS(SECT_4K) },
	{ "mx25l3205d",  INFO(0xc22016, 0, 64 * 1024,  64)
		NO_SFDP_FLAGS(SECT_4K) },
	{ "mx25l3255e",  INFO(0xc29e16, 0, 64 * 1024,  64)
		NO_SFDP_FLAGS(SECT_4K) },
	{ "mx25l6405d",  INFO(0xc22017, 0, 64 * 1024, 128)
		NO_SFDP_FLAGS(SECT_4K) },
	{ "mx25u2033e",  INFO(0xc22532, 0, 64 * 1024,   4)
		NO_SFDP_FLAGS(SECT_4K) },
	{ "mx25u3235f",	 INFO(0xc22536, 0, 64 * 1024,  64)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx25u4035",   INFO(0xc22533, 0, 64 * 1024,   8)
		NO_SFDP_FLAGS(SECT_4K) },
	{ "mx25u8035",   INFO(0xc22534, 0, 64 * 1024,  16)
		NO_SFDP_FLAGS(SECT_4K) },
	{ "mx25u6435f",  INFO(0xc22537, 0, 64 * 1024, 128)
		NO_SFDP_FLAGS(SECT_4K) },
	{ "mx25l12805d", INFO(0xc22018, 0, 64 * 1024, 256)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_4BIT_BP)
		NO_SFDP_FLAGS(SECT_4K) },
	{ "mx25l12855e", INFO(0xc22618, 0, 64 * 1024, 256) },
	{ "mx25r1635f",  INFO(0xc22815, 0, 64 * 1024,  32)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx25r3235f",  INFO(0xc22816, 0, 64 * 1024,  64)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx25u12835f", INFO(0xc22538, 0, 64 * 1024, 256)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx25l25635e", INFO(0xc22019, 0, 64 * 1024, 512)
		NO_SFDP_FLAGS(SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
		.fixups = &mx25l25635_fixups },
	{ "mx25u25635f", INFO(0xc22539, 0, 64 * 1024, 512)
		NO_SFDP_FLAGS(SECT_4K)
		FIXUP_FLAGS(SPI_NOR_4B_OPCODES) },
	{ "mx25u51245g", INFO(0xc2253a, 0, 64 * 1024, 1024)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
		FIXUP_FLAGS(SPI_NOR_4B_OPCODES) },
	{ "mx25v8035f",  INFO(0xc22314, 0, 64 * 1024,  16)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx25l25655e", INFO(0xc22619, 0, 64 * 1024, 512) },
	{ "mx66l51235f", INFO(0xc2201a, 0, 64 * 1024, 1024)
		NO_SFDP_FLAGS(SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
		FIXUP_FLAGS(SPI_NOR_4B_OPCODES) },
	{ "mx66u51235f", INFO(0xc2253a, 0, 64 * 1024, 1024)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
		FIXUP_FLAGS(SPI_NOR_4B_OPCODES) },
	{ "mx66l1g45g",  INFO(0xc2201b, 0, 64 * 1024, 2048)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx66l1g55g",  INFO(0xc2261b, 0, 64 * 1024, 2048)
		NO_SFDP_FLAGS(SPI_NOR_QUAD_READ) },
	{ "mx66u2g45g",	 INFO(0xc2253c, 0, 64 * 1024, 4096)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
		FIXUP_FLAGS(SPI_NOR_4B_OPCODES) },
	{ "mx66uw2g345gx0", INFO(0xc2943c, 0, 0, 0)
		PARSE_SFDP },
	{ "mx66lm1g45g", INFO(0xc2853b, 0, 0, 0)
		PARSE_SFDP },
	{ "mx25lm51245g", INFO(0xc2853a, 0, 0, 0)
		PARSE_SFDP },
	{ "mx25lw51245g", INFO(0xc2863a, 0, 0, 0)
		PARSE_SFDP },
	{ "mx25lm25645g", INFO(0xc28539, 0, 0, 0)
		PARSE_SFDP },
	{ "mx66uw2g345g", INFO(0xc2843c, 0, 0, 0)
		PARSE_SFDP },
	{ "mx66um1g45g", INFO(0xc2803b, 0, 0, 0)
		PARSE_SFDP },
	{ "mx66uw1g45g", INFO(0xc2813b, 0, 0, 0)
		PARSE_SFDP },
	{ "mx25uw51345g", INFO(0xc2843a, 0, 0, 0)
		PARSE_SFDP },
	{ "mx25um25645g", INFO(0xc28039, 0, 0, 0)
		PARSE_SFDP },
	{ "mx25uw25645g", INFO(0xc28139, 0, 0, 0)
		PARSE_SFDP },
	{ "mx25um25345g", INFO(0xc28339, 0, 0, 0)
		PARSE_SFDP },
	{ "mx25uw25345g", INFO(0xc28439, 0, 0, 0)
		PARSE_SFDP },
	{ "mx25uw12845g", INFO(0xc28138, 0, 0, 0)
		PARSE_SFDP },
	{ "mx25uw12345g", INFO(0xc28438, 0, 0, 0)
		PARSE_SFDP },
	{ "mx25uw6445g", INFO(0xc28137, 0, 0, 0)
		PARSE_SFDP },
	{ "mx25uw6345g", INFO(0xc28437, 0, 0, 0)
		PARSE_SFDP },
};

static int macronix_nor_octal_dtr_en(struct spi_nor *nor)
{
	struct spi_mem_op op;
	u8 *buf = nor->bouncebuf;
	int ret;

	/* Use dummy cycles which is parse by SFDP and convert to bit pattern. */
	buf[0] = SPINOR_REG_MXIC_DC(nor->params->reads[SNOR_CMD_READ_8_8_8_DTR].num_wait_states);
	op = (struct spi_mem_op)
		MXIC_NOR_WR_ANY_REG_OP(SPINOR_REG_MXIC_ADDR_BYTES,
				       SPINOR_REG_MXIC_CR2_DC, 1, buf);

	ret = spi_nor_write_any_volatile_reg(nor, &op, nor->reg_proto);
	if (ret)
		return ret;

	/* Set the octal and DTR enable bits. */
	buf[0] = SPINOR_REG_MXIC_OPI_DTR_EN;
	op = (struct spi_mem_op)
		MXIC_NOR_WR_ANY_REG_OP(SPINOR_REG_MXIC_ADDR_BYTES,
				       SPINOR_REG_MXIC_CR2_MODE, 1, buf);
	ret = spi_nor_write_any_volatile_reg(nor, &op, nor->reg_proto);
	if (ret)
		return ret;

	/* Read flash ID to make sure the switch was successful. */
	ret = spi_nor_read_id(nor, 4, 4, buf, SNOR_PROTO_8_8_8_DTR);
	if (ret) {
		dev_dbg(nor->dev, "error %d reading JEDEC ID after enabling 8D-8D-8D mode\n", ret);
		return ret;
	}

	if (memcmp(buf, nor->info->id, nor->info->id_len))
		return -EINVAL;

	return 0;
}

static int macronix_nor_octal_dtr_dis(struct spi_nor *nor)
{
	struct spi_mem_op op;
	u8 *buf = nor->bouncebuf;
	int ret;

	/*
	 * The register is 1-byte wide, but 1-byte transactions are not
	 * allowed in 8D-8D-8D mode. Since there is no register at the
	 * next location, just initialize the value to 0 and let the
	 * transaction go on.
	 */
	buf[0] = SPINOR_REG_MXIC_SPI_EN;
	buf[1] = 0x0;
	op = (struct spi_mem_op)
		MXIC_NOR_WR_ANY_REG_OP(SPINOR_REG_MXIC_ADDR_BYTES,
				       SPINOR_REG_MXIC_CR2_MODE, 2, buf);
	ret = spi_nor_write_any_volatile_reg(nor, &op, SNOR_PROTO_8_8_8_DTR);
	if (ret)
		return ret;

	/* Read flash ID to make sure the switch was successful. */
	ret = spi_nor_read_id(nor, 0, 0, buf, SNOR_PROTO_1_1_1);
	if (ret) {
		dev_dbg(nor->dev, "error %d reading JEDEC ID after disabling 8D-8D-8D mode\n", ret);
		return ret;
	}

	if (memcmp(buf, nor->info->id, nor->info->id_len))
		return -EINVAL;

	return 0;
}

static int macronix_nor_set_octal_dtr(struct spi_nor *nor, bool enable)
{
	return enable ? macronix_nor_octal_dtr_en(nor) :
			macronix_nor_octal_dtr_dis(nor);
}

static void macronix_nor_default_init(struct spi_nor *nor)
{
	nor->params->quad_enable = spi_nor_sr1_bit6_quad_enable;
}

static void macronix_nor_late_init(struct spi_nor *nor)
{
	nor->params->set_4byte_addr_mode = spi_nor_set_4byte_addr_mode;
	nor->params->octal_dtr_enable = macronix_nor_set_octal_dtr;
}

static int macronix_nor_read_id(struct spi_nor *nor, u8 naddr, u8 ndummy, u8 *id,
				enum spi_nor_protocol proto)
{
	int i, ret;

	ret = spi_nor_default_read_id(nor, naddr, ndummy, id, proto);
	/* Retrieve odd array and re-sort id because of read id format will be A-A-B-B-C-C
	 * after enter into octal dtr mode for Macronix flashes.
	 */
	if (proto == SNOR_PROTO_8_8_8_DTR) {
		for (i = 0; i < nor->info->id_len; i++)
			id[i] = id[i * 2];
	}

	return ret;
}

static const struct spi_nor_fixups macronix_nor_fixups = {
	.default_init = macronix_nor_default_init,
	.late_init = macronix_nor_late_init,
	.read_id = macronix_nor_read_id,
};

const struct spi_nor_manufacturer spi_nor_macronix = {
	.name = "macronix",
	.parts = macronix_nor_parts,
	.nparts = ARRAY_SIZE(macronix_nor_parts),
	.fixups = &macronix_nor_fixups,
};
