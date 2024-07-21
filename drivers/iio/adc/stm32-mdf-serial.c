// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * This file is part of STM32 MDF driver
 *
 * Copyright (C) 2023, STMicroelectronics - All Rights Reserved
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>

#include "stm32-mdf.h"

#define STM32_MDF_MODE_SZ 12

enum {
	STM32_MDF_SCKSRC_CCK0,
	STM32_MDF_SCKSRC_CCK1,
	STM32_MDF_SCKSRC_CLK,
	STM32_MDF_SCKSRC_NONE,
};

struct stm32_mdf_sf_mode {
	const char *name;
	u32 mode;
};

static const struct stm32_mdf_sf_mode stm32_mdf_mode[STM32_MDF_MODE_NB] = {
	{ "spi", STM32_MDF_MODE_SPI },
	{ "lf_spi", STM32_MDF_MODE_LF_SPI },
	{ "manchester_r", STM32_MDF_MODE_MANCHESTER_R },
	{ "manchester_f", STM32_MDF_MODE_MANCHESTER_F },
};

static bool stm32_mdf_sitf_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MDF_SITFCR_REG:
		return true;
	default:
		return false;
	}
}

static bool stm32_mdf_sitf_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MDF_SITFCR_REG:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config stm32_sitf_regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = sizeof(u32),
	.max_register = MDF_SITFCR_REG,
	.readable_reg = stm32_mdf_sitf_readable_reg,
	.writeable_reg = stm32_mdf_sitf_writeable_reg,
	.fast_io = true,
	/* Do not use regcache as it does not support single register map */
};

int stm32_mdf_sitf_start(struct stm32_mdf_sitf *sitf)
{
	int ret;

	spin_lock(&sitf->lock);

	ret = regmap_set_bits(sitf->regmap, MDF_SITFCR_REG, MDF_SITFCR_SITFEN);
	if (!ret)
		sitf->refcnt++;

	spin_unlock(&sitf->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(stm32_mdf_sitf_start);

int stm32_mdf_sitf_stop(struct stm32_mdf_sitf *sitf)
{
	int ret = 0;

	spin_lock(&sitf->lock);

	if (!sitf->refcnt) {
		dev_err(sitf->dev, "Unbalanced serial interface stop ?\n");
		ret = -EPERM;
		goto out;
	} else {
		sitf->refcnt--;
	}

	if (!sitf->refcnt)
		ret = regmap_clear_bits(sitf->regmap, MDF_SITFCR_REG, MDF_SITFCR_SITFEN);

out:
	spin_unlock(&sitf->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(stm32_mdf_sitf_stop);

static int stm32_mdf_sitf_get_clk(struct device *dev, struct stm32_mdf_sitf *sitf)
{
	struct clk *sck;

	/* Optional clock. Clock not needed in Manchester mode */
	sck = clk_get_optional(sitf->dev, 0);
	if (IS_ERR(sck))
		return dev_err_probe(sitf->dev, PTR_ERR(sck), "Can't get serial clock\n");

	sitf->sck = sck;

	if (sitf->sck) {
		if (!strncmp(__clk_get_name(sck), STM32_MDF_CCK0, sizeof(STM32_MDF_CCK0)))
			sitf->scksrc = STM32_MDF_SCKSRC_CCK0;
		else if (!strncmp(__clk_get_name(sck), STM32_MDF_CCK1, sizeof(STM32_MDF_CCK1)))
			sitf->scksrc = STM32_MDF_SCKSRC_CCK1;
		else
			sitf->scksrc = STM32_MDF_SCKSRC_CLK;
	}

	return 0;
};

static int stm32_mdf_sitf_parse(struct platform_device *pdev, struct stm32_mdf_sitf *sitf)
{
	struct device *dev = &pdev->dev;
	const char *str;
	int ret, i = 0;
	u32 idx, mode, sitfcr;

	ret = device_property_read_u32(dev, "reg", &idx);
	if (ret) {
		dev_err(dev, "Could not get interface index: %d\n", ret);
		return ret;
	}

	if (idx % 0x80) {
		dev_err(dev, "Unexpected reg property value [%x]\n", idx);
		return -EINVAL;
	}

	idx = (idx >> 7) - 1;
	if (idx > sitf->mdf->nbf) {
		dev_err(dev, "Interface index [%d] exceeds maximum [%d]\n", idx, sitf->mdf->nbf);
		return -EINVAL;
	}

	/* Get SITF mode */
	ret = device_property_read_string(dev, "st,sitf-mode", &str);
	if (ret) {
		dev_err(dev, "Could not get interface mode: %d\n", ret);
		return ret;
	}

	while (i < STM32_MDF_MODE_NB) {
		if (!strncmp(stm32_mdf_mode[i].name, str, STM32_MDF_MODE_SZ)) {
			mode = stm32_mdf_mode[i].mode;
			break;
		}
		i++;
	}

	if (i >= STM32_MDF_MODE_NB) {
		dev_err(dev, "Unknown serial interface mode [%s]\n", str);
		return -EINVAL;
	}

	sitf->mode = mode;
	sitfcr = MDF_SITFCR_SITFMOD(mode);

	ret = stm32_mdf_sitf_get_clk(dev, sitf);
	if (ret)
		return ret;

	if (mode == STM32_MDF_MODE_SPI && sitf->scksrc == STM32_MDF_SCKSRC_NONE) {
		dev_err(dev, "Missing clock for serial interface [%d] in SPI mode\n", idx);
		return -EINVAL;
	}

	if (mode == STM32_MDF_MODE_LF_SPI && (sitf->scksrc != STM32_MDF_SCKSRC_CCK0 &&
					      sitf->scksrc != STM32_MDF_SCKSRC_CCK1)) {
		dev_err(dev, "Missing CCKx clock for serial interface [%d] in LF_SPI mode\n", idx);
		return -EINVAL;
	}

	sitf->id = idx;
	sitf->node = dev_fwnode(dev);
	spin_lock_init(&sitf->lock);

	sitfcr |= MDF_SITFCR_SCKSRC(sitf->scksrc);

	/* Configure SITF register */
	regmap_set_bits(sitf->regmap, MDF_SITFCR_REG, sitfcr);

	dev_dbg(dev, "Serial interface [%d] registered\n", idx);

	return 0;
}

static int stm32_mdf_sitf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stm32_mdf_sitf *sitf;
	struct regmap *regmap;
	struct resource *res;
	void __iomem *base;
	int ret;

	sitf = devm_kzalloc(&pdev->dev, sizeof(*sitf), GFP_KERNEL);
	if (!sitf)
		return -ENOMEM;
	sitf->dev = dev;
	sitf->mdf = dev_get_drvdata(dev->parent);

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio_clk(dev, "ker_ck", base, &stm32_sitf_regmap_cfg);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "Failed to init regmap\n");
	sitf->regmap = regmap;

	ret = stm32_mdf_sitf_parse(pdev, sitf);
	if (ret < 0)
		return ret;

	list_add(&sitf->entry, &sitf->mdf->sitf_list);

	platform_set_drvdata(pdev, sitf);

	return 0;
}

static int stm32_mdf_sitf_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stm32_mdf_sitf *sitf = dev_get_drvdata(dev);

	list_del(&sitf->entry);

	return 0;
}

static int stm32_mdf_sitf_suspend(struct device *dev)
{
	struct stm32_mdf_sitf *sitf = dev_get_drvdata(dev);
	int ret;

	ret = pinctrl_pm_select_sleep_state(dev);

	regmap_read(sitf->regmap, MDF_SITFCR_REG, &sitf->sitfcr);

	return ret;
}

static int stm32_mdf_sitf_resume(struct device *dev)
{
	struct stm32_mdf_sitf *sitf = dev_get_drvdata(dev);

	regmap_write(sitf->regmap, MDF_SITFCR_REG, sitf->sitfcr);

	return pinctrl_pm_select_default_state(dev);
}

static int stm32_mdf_sitf_runtime_suspend(struct device *dev)
{
	return 0;
}

static int stm32_mdf_sitf_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops stm32_mdf_sitf_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stm32_mdf_sitf_suspend, stm32_mdf_sitf_resume)
	SET_RUNTIME_PM_OPS(stm32_mdf_sitf_runtime_suspend, stm32_mdf_sitf_runtime_resume, NULL)
};

static const struct of_device_id stm32_mdf_sitf_of_match[] = {
	{ .compatible = "st,stm32mp25-sitf-mdf" },
	{}
};
MODULE_DEVICE_TABLE(of, stm32_mdf_sitf_of_match);

static struct platform_driver stm32_mdf_sitf_driver = {
	.probe = stm32_mdf_sitf_probe,
	.remove = stm32_mdf_sitf_remove,
	.driver = {
		.name = "stm32-mdf-sitf",
		.of_match_table = stm32_mdf_sitf_of_match,
		.pm = &stm32_mdf_sitf_pm_ops,
	},
};

module_platform_driver(stm32_mdf_sitf_driver);

MODULE_AUTHOR("Olivier Moysan <olivier.moysan@foss.st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 MDF serial driver");
MODULE_LICENSE("GPL");
