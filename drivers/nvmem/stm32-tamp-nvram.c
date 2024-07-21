// SPDX-License-Identifier: GPL-2.0
/*
 * STM32 Tamp backup registers access driver
 *
 * Copyright (C) 2023, STMicroelectronics - All Rights Reserved
 * Author: Simeon Marijon <simeon.marijon@foss.st.com> for STMicroelectronics.
 */

#include <linux/align.h>
#include <linux/bits.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/nvmem-provider.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/types.h>

#define RIF_CID1			0x1
#define CURRENT_CID			RIF_CID1
#define NB_ZONES_STM32MP1		3
#define NB_ZONES_STM32MP2		7
#define NB_REGS_STM32MP1		32
#define NB_REGS_STM32MP2		128

#define _TAMP_SECCFGR			0x20U
#define _TAMP_BKPRIFR(x)		(0x70U + 0x4U * ((x) - 1))
#define _TAMP_RXCIDCFGR(x)		(0x80U + 0x4U * ((x)))

#define BKPREG_PROTECTION_ZONE_1	0
#define BKPREG_PROTECTION_ZONE_2	1
#define BKPREG_PROTECTION_ZONE_3	2

#define BKPREG_PROTECTION_ZONE_1_RIF1	0
#define BKPREG_PROTECTION_ZONE_1_RIF2	1
#define BKPREG_PROTECTION_ZONE_2_RIF1	2
#define BKPREG_PROTECTION_ZONE_2_RIF2	3
#define BKPREG_PROTECTION_ZONE_3_RIF1	4
#define BKPREG_PROTECTION_ZONE_3_RIF0	5
#define BKPREG_PROTECTION_ZONE_3_RIF2	6
#define NB_COMPARTMENT_STM32MP2		3

enum stm32_tamp_bkpreg_access {
	BKP_READ_WRITE,
	BKP_READ,
	BKP_NO
};

struct stm32_tamp_nvram_priv {
	struct nvmem_config cfg;
	const struct stm32_tamp_nvram_plat *data;
	int *idx_bkpreg_zones_end;
	struct device *dev;
	struct regmap *config_regmap;
	struct regmap *bkpregs_regmap;
	enum stm32_tamp_bkpreg_access *bkpreg_access;
};

struct stm32_tamp_nvram_plat {
	const unsigned int nb_zones;
	const unsigned int nb_regs;
	const struct regmap_config *bkpregs_regmap_cfg;
	const struct reg_field *config_reg_fields;
};

static const struct regmap_config stm32mp_tamp_nvram_regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,

};

static const struct reg_field stm32mp1_tamp_nvram_zone_cfg_fields[NB_ZONES_STM32MP1 - 1] = {
	[BKPREG_PROTECTION_ZONE_1] = REG_FIELD(_TAMP_SECCFGR, 0, 7),
	[BKPREG_PROTECTION_ZONE_2] = REG_FIELD(_TAMP_SECCFGR, 16, 23),
};

static const struct reg_field stm32mp25_tamp_nvram_zone_cfg_fields[NB_ZONES_STM32MP2 - 1] = {
	[BKPREG_PROTECTION_ZONE_1_RIF1] = REG_FIELD(_TAMP_BKPRIFR(1), 0,  7),
	[BKPREG_PROTECTION_ZONE_1_RIF2] = REG_FIELD(_TAMP_SECCFGR,    0,  7),
	[BKPREG_PROTECTION_ZONE_2_RIF1] = REG_FIELD(_TAMP_BKPRIFR(2), 0,  7),
	[BKPREG_PROTECTION_ZONE_2_RIF2] = REG_FIELD(_TAMP_SECCFGR,   16, 23),
	[BKPREG_PROTECTION_ZONE_3_RIF1] = REG_FIELD(_TAMP_BKPRIFR(3), 0,  7),
	[BKPREG_PROTECTION_ZONE_3_RIF0] = REG_FIELD(_TAMP_BKPRIFR(3), 16, 23),
};

static const struct reg_field stm32mp25_tamp_nvram_rxcidcfg_cfen_fields[NB_COMPARTMENT_STM32MP2] = {
	REG_FIELD(_TAMP_RXCIDCFGR(0), 0, 0),
	REG_FIELD(_TAMP_RXCIDCFGR(1), 0, 0),
	REG_FIELD(_TAMP_RXCIDCFGR(2), 0, 0),
};

static const struct reg_field stm32mp25_tamp_nvram_rxcidcfg_fields[NB_COMPARTMENT_STM32MP2] = {
	REG_FIELD(_TAMP_RXCIDCFGR(0), 4, 6),
	REG_FIELD(_TAMP_RXCIDCFGR(1), 4, 6),
	REG_FIELD(_TAMP_RXCIDCFGR(2), 4, 6),
};

static enum stm32_tamp_bkpreg_access stm32mp1_tamp_bkpreg_access[NB_ZONES_STM32MP1] = {
	[BKPREG_PROTECTION_ZONE_1] = BKP_NO,
	[BKPREG_PROTECTION_ZONE_2] = BKP_READ,
	[BKPREG_PROTECTION_ZONE_3] = BKP_READ_WRITE,
};

static const struct stm32_tamp_nvram_plat stm32mp1_tamp_nvram = {
	.nb_zones = NB_ZONES_STM32MP1,
	.nb_regs = NB_REGS_STM32MP1,
	.bkpregs_regmap_cfg = &stm32mp_tamp_nvram_regmap_cfg,
	.config_reg_fields = stm32mp1_tamp_nvram_zone_cfg_fields,
};

static const struct stm32_tamp_nvram_plat stm32mp25_tamp_nvram = {
	.nb_zones = NB_ZONES_STM32MP2,
	.nb_regs = NB_REGS_STM32MP2,
	.bkpregs_regmap_cfg = &stm32mp_tamp_nvram_regmap_cfg,
	.config_reg_fields = stm32mp25_tamp_nvram_zone_cfg_fields,
};

static int stm32_tamp_is_compartment_isolation_enabled_mp2X(struct stm32_tamp_nvram_priv *priv)
{
	int nb_compartment_enabled = 0;
	struct device *dev = priv->dev;
	u32 cfen;
	struct regmap_field *cfen_field;

	for (int i = 0; i < NB_COMPARTMENT_STM32MP2; i++) {
		cfen_field = devm_regmap_field_alloc(dev,
						     priv->config_regmap,
						     stm32mp25_tamp_nvram_rxcidcfg_cfen_fields[i]);
		if (IS_ERR_OR_NULL(cfen_field)) {
			dev_err(dev, "Can't allocate field for reading configuration\n");
			return -ENOMEM;
		}
		if (regmap_field_read(cfen_field, &cfen)) {
			dev_err(dev, "Can't read field for registers zones\n");
			devm_regmap_field_free(dev, cfen_field);
			return -EINVAL;
		}
		nb_compartment_enabled += cfen;
		devm_regmap_field_free(dev, cfen_field);
	}

	if (!nb_compartment_enabled)
		return 0;
	else if (nb_compartment_enabled == NB_COMPARTMENT_STM32MP2)
		return 1;
	else
		return -EINVAL;
}

static bool *stm32_tamp_get_compartment_owner_mp2X(struct stm32_tamp_nvram_priv *priv)
{
	struct device *dev = priv->dev;
	struct regmap_field *cid_field;
	u32 cid_per_zone;
	int isolation_enabled;
	bool *compartment_owner;

	isolation_enabled = stm32_tamp_is_compartment_isolation_enabled_mp2X(priv);
	if (isolation_enabled < 0)
		return NULL;

	compartment_owner = devm_kcalloc(dev,
					 NB_COMPARTMENT_STM32MP2,
					 sizeof(*compartment_owner),
					 GFP_KERNEL);
	if (!compartment_owner)
		return ERR_PTR(-ENOMEM);

	for (int i = 0; i < NB_COMPARTMENT_STM32MP2; i++) {
		if (isolation_enabled) {
			cid_field = devm_regmap_field_alloc(dev,
							    priv->config_regmap,
							    stm32mp25_tamp_nvram_rxcidcfg_fields[i]
							    );

			if (regmap_field_read(cid_field, &cid_per_zone)) {
				dev_err(dev, "Can't read field for registers zones\n");
				devm_regmap_field_free(dev, cid_field);
				devm_kfree(dev, compartment_owner);
				return ERR_PTR(-EINVAL);
			}
			if (cid_per_zone == CURRENT_CID)
				compartment_owner[i] = true;
			else
				compartment_owner[i] = false;

			devm_regmap_field_free(dev, cid_field);
		} else {
			compartment_owner[i] = true;
		}
	}

	return compartment_owner;
}

static enum stm32_tamp_bkpreg_access *stm32_tamp_get_access_rights_mp2X(struct stm32_tamp_nvram_priv
									*priv)
{
	struct device *dev = priv->dev;
	unsigned int nb_zones = priv->data->nb_zones;
	bool *compartment_owner;
	enum stm32_tamp_bkpreg_access *bkpreg_access;

	compartment_owner = stm32_tamp_get_compartment_owner_mp2X(priv);
	if (IS_ERR(compartment_owner))
		return ERR_PTR(-ENODEV);

	bkpreg_access = devm_kcalloc(dev,
				     NB_ZONES_STM32MP2,
				     sizeof(*bkpreg_access),
				     GFP_KERNEL);

	for (int protection_zone_idx = 0; protection_zone_idx < nb_zones;
	     protection_zone_idx++) {
		switch (protection_zone_idx) {
		case BKPREG_PROTECTION_ZONE_1_RIF1:
			bkpreg_access[protection_zone_idx] = BKP_NO;
			break;
		case BKPREG_PROTECTION_ZONE_1_RIF2:
			bkpreg_access[protection_zone_idx] = BKP_NO;
			break;
		case BKPREG_PROTECTION_ZONE_2_RIF1:
			if (compartment_owner[1] || compartment_owner[2])
				bkpreg_access[protection_zone_idx] = BKP_READ;
			else
				bkpreg_access[protection_zone_idx] = BKP_NO;
			break;
		case BKPREG_PROTECTION_ZONE_2_RIF2:
			if (compartment_owner[1] || compartment_owner[2])
				bkpreg_access[protection_zone_idx] = BKP_READ;
			else
				bkpreg_access[protection_zone_idx] = BKP_NO;
			break;
		case BKPREG_PROTECTION_ZONE_3_RIF1:
			if (compartment_owner[1])
				bkpreg_access[protection_zone_idx] = BKP_READ_WRITE;
			else if (compartment_owner[0] || compartment_owner[2])
				bkpreg_access[protection_zone_idx] = BKP_READ;
			else
				bkpreg_access[protection_zone_idx] = BKP_NO;
			break;
		case BKPREG_PROTECTION_ZONE_3_RIF0:
			if (compartment_owner[0])
				bkpreg_access[protection_zone_idx] = BKP_READ_WRITE;
			else if (compartment_owner[1] || compartment_owner[2])
				bkpreg_access[protection_zone_idx] = BKP_READ;
			else
				bkpreg_access[protection_zone_idx] = BKP_NO;
			break;
		case BKPREG_PROTECTION_ZONE_3_RIF2:
			if (compartment_owner[2])
				bkpreg_access[protection_zone_idx] = BKP_READ_WRITE;
			else if (compartment_owner[0] || compartment_owner[1])
				bkpreg_access[protection_zone_idx] = BKP_READ;
			else
				bkpreg_access[protection_zone_idx] = BKP_NO;
			break;
		default:
			devm_kfree(dev, bkpreg_access);
			return ERR_PTR(-ENODEV);
		}
	}

	return bkpreg_access;
}

static int stm32_tamp_nvram_bkpreg_get_zone(struct stm32_tamp_nvram_priv *priv, int reg)
{
	int *idx_bkpreg_zones_end = priv->idx_bkpreg_zones_end;
	int nb_zones = priv->data->nb_zones;
	int protection_zone_idx;

	if (reg < 0)
		return -1; // negative reg is the boundary of an empty zone

	for (protection_zone_idx = 0; protection_zone_idx < nb_zones; protection_zone_idx++) {
		if (reg <= idx_bkpreg_zones_end[protection_zone_idx])
			break;
	}

	if (protection_zone_idx >= nb_zones)
		return -1; // the reg is not a part of any zone

	return protection_zone_idx;
}

static bool stm32_tamp_nvram_rights(struct stm32_tamp_nvram_priv *priv, int reg, bool read_only)
{
	struct device *dev = priv->dev;
	int protection_zone_idx = stm32_tamp_nvram_bkpreg_get_zone(priv, reg);

	if (protection_zone_idx < 0)
		return false;

	switch (priv->bkpreg_access[protection_zone_idx]) {
	case BKP_READ_WRITE:
		return true;
	case BKP_READ:
		return read_only;
	case BKP_NO:
		return false;
	default:
		dev_err(dev, "Can't get access rights for the zone\n");
		return false;
	}

	return false;
}

static int stm32_tamp_nvram_write_byte(struct stm32_tamp_nvram_priv *priv, u32 offset, u8 byte)
{
	int offset_aligned = ALIGN_DOWN(offset, sizeof(u32));
	int byte_in_word = offset - offset_aligned;
	u32 read_value, to_be_writen_value;
	u32 reg_idx = offset_aligned / sizeof(u32);

	if (!stm32_tamp_nvram_rights(priv, reg_idx, false))
		return -EIO;

	regmap_read(priv->bkpregs_regmap, offset_aligned, &read_value);
	to_be_writen_value = read_value & ~(0xFFUL << byte_in_word * 8);
	to_be_writen_value |=  (u32)byte << (byte_in_word * 8);

	return regmap_write(priv->bkpregs_regmap, offset_aligned, to_be_writen_value);
}

static int stm32_tamp_nvram_read_byte(struct stm32_tamp_nvram_priv *priv, u32 offset, u8 *byte)
{
	int offset_aligned = ALIGN_DOWN(offset, sizeof(u32));
	int byte_in_word = offset - offset_aligned;
	u32 read_value;
	u32 reg_idx = offset_aligned / sizeof(u32);

	if (!stm32_tamp_nvram_rights(priv, reg_idx, true))
		return -EIO;

	regmap_read(priv->bkpregs_regmap, offset_aligned, &read_value);
	*byte = (read_value >> (byte_in_word * 8)) & 0xFF;

	return 0;
}

static int stm32_tamp_nvram_read(void *context, unsigned int offset, void *buf, size_t bytes)
{
	struct stm32_tamp_nvram_priv *priv = context;
	struct device *dev = priv->dev;
	u8 byte;
	u8 *buf_u8 = buf;
	u32 temp_u32;
	int i, ret;
	size_t total = offset + bytes;
	u32 reg_idx;

	i = offset;
	while (i < total)  {
		reg_idx = i / sizeof(u32);
		if (i + sizeof(u32) <= total && IS_ALIGNED(i, sizeof(u32))) {
			if (!stm32_tamp_nvram_rights(priv, reg_idx, true)) {
				dev_dbg(dev, "Backup register %u is not allowed to be read\n",
					reg_idx);
				temp_u32 = 0;
			} else {
				regmap_read(priv->bkpregs_regmap, i, &temp_u32);
			}
			memcpy(buf_u8, &temp_u32, sizeof(u32));
			buf_u8 += sizeof(u32);
			i += sizeof(u32);
		} else {
			ret = stm32_tamp_nvram_read_byte(priv, i, &byte);
			if (ret) {
				dev_dbg(dev, "Backup register %u is not allowed to be read\n",
					reg_idx);
				byte = 0;
			}
			*buf_u8 = byte;
			i++;
			buf_u8++;
		}
	}

	return 0;
}

static int stm32_tamp_nvram_write(void *context, unsigned int offset, void *buf, size_t bytes)
{
	struct stm32_tamp_nvram_priv *priv = context;
	struct device *dev = priv->dev;
	u8 *buf_u8 = (u8 *)buf;
	u32 temp_u32;
	size_t total = offset + bytes;
	int i, ret;
	u32 reg_idx;

	i = offset;
	while (i < total)  {
		reg_idx = i / sizeof(u32);
		if (i + sizeof(u32) <= total && IS_ALIGNED(i, sizeof(u32))) {
			if (stm32_tamp_nvram_rights(priv, reg_idx, false)) {
				memcpy(&temp_u32, buf_u8, sizeof(u32));
				regmap_write(priv->bkpregs_regmap, i, temp_u32);
			} else {
				dev_dbg(dev, "Backup register %u is not allowed to be written",
					reg_idx);
			}
			buf_u8 += sizeof(u32);
			i += sizeof(u32);
		} else {
			ret = stm32_tamp_nvram_write_byte(priv, i, *buf_u8);
			if (ret)
				dev_dbg(dev, "Backup register %u is not allowed to be written",
					reg_idx);
			i++;
			buf_u8++;
		}
	}

	return 0;
}

static int *stm32_tamp_nvram_get_backup_zones(struct stm32_tamp_nvram_priv *priv)
{
	struct device *dev = priv->dev;
	int nb_zones = priv->data->nb_zones;
	int zone_idx;
	u32 *idx_bkpreg_zones_end;
	struct regmap *tamp_regmap = priv->config_regmap;
	u32 offset_field;

	idx_bkpreg_zones_end = devm_kcalloc(dev,
					    sizeof(*idx_bkpreg_zones_end),
					    nb_zones,
					    GFP_KERNEL);
	if (!idx_bkpreg_zones_end) {
		dev_err(dev, "Can't allocate registers zones\n");
		return ERR_PTR(-ENOMEM);
	}

	//Get the n-1 frontiers of zone within the tamp configuration registers
	for (zone_idx = 0; zone_idx < nb_zones - 1; zone_idx++) {
		const struct reg_field reg_field = priv->data->config_reg_fields[zone_idx];
		struct regmap_field *field = devm_regmap_field_alloc(dev,
								     tamp_regmap,
								     reg_field);

		if (IS_ERR_OR_NULL(field)) {
			dev_err(dev, "Can't allocate registers zones\n");
			devm_kfree(dev, idx_bkpreg_zones_end);
			return ERR_PTR(-ENOMEM);
		}
		if (regmap_field_read(field, &offset_field)) {
			dev_err(dev, "Can't read field for registers zones\n");
			devm_kfree(dev, idx_bkpreg_zones_end);
			return ERR_PTR(-EIO);
		}

		idx_bkpreg_zones_end[zone_idx] = offset_field - 1;
	}

	//The last zone end is defined by the number of registers in TAMP
	idx_bkpreg_zones_end[zone_idx] = priv->data->nb_regs - 1;

	return idx_bkpreg_zones_end;
}

static const struct of_device_id stm32_tamp_nvram_of_match[] = {
	{ .compatible = "st,stm32mp15-tamp-nvram",
	  .data = &stm32mp1_tamp_nvram },
	{ .compatible = "st,stm32mp25-tamp-nvram",
	  .data = &stm32mp25_tamp_nvram },
	{},
};

static int stm32_tamp_nvram_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stm32_tamp_nvram_priv *priv;
	struct resource *res;
	struct device_node *of_node;
	const struct of_device_id *of_id;
	void __iomem *base_addr;
	unsigned int *zones_end;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;

	of_node = dev_of_node(dev);
	if (!of_node) {
		dev_err(dev, "Can't get of node of the device\n");
		return -ENODEV;
	}
	of_id = of_match_node(stm32_tamp_nvram_of_match, of_node);
	if (IS_ERR_OR_NULL(of_id)) {
		dev_err(dev, "Can't get of_id of the device\n");
		return -ENODEV;
	}
	priv->data = (struct stm32_tamp_nvram_plat *)of_id->data;
	if (IS_ERR_OR_NULL(priv->data)) {
		dev_err(dev, "Can't get platform data of the node\n");
		return -ENODEV;
	}

	priv->config_regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR_OR_NULL(priv->config_regmap)) {
		dev_dbg(dev, "Deferring till parent probed\n");
		return -EPROBE_DEFER;
	}

	priv->idx_bkpreg_zones_end = stm32_tamp_nvram_get_backup_zones(priv);
	if (IS_ERR_OR_NULL(priv->idx_bkpreg_zones_end)) {
		dev_dbg(dev, "Can't determine protection zone\n");
		return -ENODEV;
	}
	zones_end = priv->idx_bkpreg_zones_end;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Can't get resource\n");
		return -ENODEV;
	}
	base_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR_OR_NULL(base_addr)) {
		dev_err(dev, "Can't remap resource\n");
		return PTR_ERR(base_addr);
	}
	priv->bkpregs_regmap = devm_regmap_init_mmio(dev,
						     base_addr,
						     priv->data->bkpregs_regmap_cfg);
	priv->cfg.name = "stm32-tamp-nvram";
	priv->cfg.word_size = 1;
	priv->cfg.stride = 1;
	priv->cfg.dev = dev;
	priv->cfg.priv = priv;
	priv->cfg.owner = THIS_MODULE;
	priv->cfg.type = NVMEM_TYPE_BATTERY_BACKED;
	priv->cfg.size = resource_size(res);
	priv->cfg.reg_read = stm32_tamp_nvram_read;
	priv->cfg.reg_write = stm32_tamp_nvram_write;

	if (of_device_is_compatible(pdev->dev.of_node,
				    "st,stm32mp25-tamp-nvram")) {
		priv->bkpreg_access = stm32_tamp_get_access_rights_mp2X(priv);
		if (IS_ERR_OR_NULL(priv->bkpreg_access))
			return -ENODEV;

		dev_dbg(dev, "\n"
			"Zone 1-RIF1 %3d - %3d %c%c\n"
			"Zone 1-RIF2 %3d - %3d %c%c\n"
			"Zone 2-RIF1 %3d - %3d %c%c\n"
			"Zone 2-RIF2 %3d - %3d %c%c\n"
			"Zone 3-RIF1 %3d - %3d %c%c\n"
			"Zone 3-RIF0 %3d - %3d %c%c\n"
			"Zone 3-RIF2 %3d - %3d %c%c\n",
			0,
			zones_end[BKPREG_PROTECTION_ZONE_1_RIF1],
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_1_RIF1],
						true) ? 'R' : '-',
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_1_RIF1],
						false) ? 'W' : '-',
			zones_end[BKPREG_PROTECTION_ZONE_1_RIF1] + 1,
			zones_end[BKPREG_PROTECTION_ZONE_1_RIF2],
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_1_RIF2],
						true) ? 'R' : '-',
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_1_RIF2],
						false) ? 'W' : '-',
			zones_end[BKPREG_PROTECTION_ZONE_1_RIF2] + 1,
			zones_end[BKPREG_PROTECTION_ZONE_2_RIF1],
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_2_RIF1],
						true) ? 'R' : '-',
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_2_RIF1],
						false) ? 'W' : '-',
			zones_end[BKPREG_PROTECTION_ZONE_2_RIF1] + 1,
			zones_end[BKPREG_PROTECTION_ZONE_2_RIF2],
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_2_RIF2],
						true) ? 'R' : '-',
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_2_RIF2],
						false) ? 'W' : '-',
			zones_end[BKPREG_PROTECTION_ZONE_2_RIF2] + 1,
			zones_end[BKPREG_PROTECTION_ZONE_3_RIF1],
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_3_RIF1],
						true) ? 'R' : '-',
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_3_RIF1],
						false) ? 'W' : '-',
			zones_end[BKPREG_PROTECTION_ZONE_3_RIF1] + 1,
			zones_end[BKPREG_PROTECTION_ZONE_3_RIF0],
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_3_RIF0],
						true) ? 'R' : '-',
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_3_RIF0],
						false) ? 'W' : '-',
			zones_end[BKPREG_PROTECTION_ZONE_3_RIF0] + 1,
			zones_end[BKPREG_PROTECTION_ZONE_3_RIF2],
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_3_RIF2],
						true) ? 'R' : '-',
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_3_RIF2],
						false) ? 'W' : '-'
				);
	} else if (of_device_is_compatible(pdev->dev.of_node, "st,stm32mp15-tamp-nvram")) {
		priv->bkpreg_access = stm32mp1_tamp_bkpreg_access;

		dev_dbg(dev, "\n"
			"Zone 1 %3d - %3d %c%c\n"
			"Zone 2 %3d - %3d %c%c\n"
			"Zone 3 %3d - %3d %c%c\n",
			0,
			zones_end[BKPREG_PROTECTION_ZONE_1],
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_1],
						true) ? 'R' : '-',
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_1],
						false) ? 'W' : '-',
			zones_end[BKPREG_PROTECTION_ZONE_1] + 1,
			zones_end[BKPREG_PROTECTION_ZONE_2],
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_2],
						true) ? 'R' : '-',
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_2],
						false) ? 'W' : '-',
			zones_end[BKPREG_PROTECTION_ZONE_2] + 1,
			zones_end[BKPREG_PROTECTION_ZONE_3],
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_3],
						true) ? 'R' : '-',
			stm32_tamp_nvram_rights(priv,
						zones_end[BKPREG_PROTECTION_ZONE_3],
						false) ? 'W' : '-');
	}

	platform_set_drvdata(pdev, priv);

	return PTR_ERR_OR_ZERO(devm_nvmem_register(dev, &priv->cfg));
}

MODULE_DEVICE_TABLE(of, stm32_tamp_nvram_of_match);

static struct platform_driver stm32_tamp_nvram_driver = {
	.probe = stm32_tamp_nvram_probe,
	.driver = {
		.name = "stm32-tamp-nvram",
		.of_match_table = of_match_ptr(stm32_tamp_nvram_of_match),
	},
};

module_platform_driver(stm32_tamp_nvram_driver)
MODULE_AUTHOR("Simeon Marijon <simeon.marijon@foss.st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 NVMEM TAMP Backup registeristers");
MODULE_ALIAS("platform:nvmem-stm32-tamp-nvram");
MODULE_LICENSE("GPL");
