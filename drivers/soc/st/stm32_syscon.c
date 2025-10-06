// SPDX-License-Identifier: GPL-2.0-only
/*
 * STM32MP System Control Driver
 *
 * Copyright (C) 2025 STMicroelectronics
 *
 * Author: Antonio Borneo <antonio.borneo@foss.st.com>
 */

#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define STM32MP25_CA35SS_SYSCFG_ETR_LPI_CR		0x2000
#define STM32MP25_CA35SS_SYSCFG_ETR_LPI_SR		0x2004
#define STM32MP25_CA35SS_SYSCFG_STM_NSGUAREN_CR		0x2008
#define STM32MP25_CA35SS_SYSCFG_TRACE_CLK_DIV_CR	0x200c
#define STM32MP25_CA35SS_SYSCFG_DBGPWR_CR		0x2010
#define STM32MP25_CA35SS_SYSCFG_DBGL1RSTDISABLE_CR	0x2014
#define STM32MP25_CA35SS_SYSCFG_DBGPWR_SR		0x2018
#define STM32MP25_CA35SS_SYSCFG_EDBGACK_SR		0x201c
#define STM32MP25_CA35SS_SYSCFG_GIC_CFGR		0x2020
#define STM32MP25_CA35SS_SYSCFG_LP_SR			0x2024
#define STM32MP25_CA35SS_SYSCFG_RSTACK_SR		0x2028
#define STM32MP25_CA35SS_SYSCFG_AARCH_MODE_CR		0x2080
#define STM32MP25_CA35SS_SYSCFG_VBAR_CR			0x2084
#define STM32MP25_CA35SS_SYSCFG_M33_ACCESS_CR		0x2088
#define STM32MP25_CA35SS_SYSCFG_M33_TZEN_CR		0x20a0
#define STM32MP25_CA35SS_SYSCFG_M33_INITSVTOR_CR	0x20a4
#define STM32MP25_CA35SS_SYSCFG_M33_INITNSVTOR_CR	0x20a8

static bool stm32_syscon_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case STM32MP25_CA35SS_SYSCFG_ETR_LPI_CR:
	case STM32MP25_CA35SS_SYSCFG_ETR_LPI_SR:
	case STM32MP25_CA35SS_SYSCFG_STM_NSGUAREN_CR:
	case STM32MP25_CA35SS_SYSCFG_TRACE_CLK_DIV_CR:
	case STM32MP25_CA35SS_SYSCFG_DBGPWR_CR:
	case STM32MP25_CA35SS_SYSCFG_DBGL1RSTDISABLE_CR:
	case STM32MP25_CA35SS_SYSCFG_DBGPWR_SR:
	case STM32MP25_CA35SS_SYSCFG_EDBGACK_SR:
	case STM32MP25_CA35SS_SYSCFG_GIC_CFGR:
	case STM32MP25_CA35SS_SYSCFG_LP_SR:
	case STM32MP25_CA35SS_SYSCFG_RSTACK_SR:
	case STM32MP25_CA35SS_SYSCFG_M33_TZEN_CR:
	case STM32MP25_CA35SS_SYSCFG_M33_INITSVTOR_CR:
	case STM32MP25_CA35SS_SYSCFG_M33_INITNSVTOR_CR:
		return true;

	case STM32MP25_CA35SS_SYSCFG_AARCH_MODE_CR:
	case STM32MP25_CA35SS_SYSCFG_VBAR_CR:
	case STM32MP25_CA35SS_SYSCFG_M33_ACCESS_CR:
	default:
		return false;
	}
}

static bool stm32_syscon_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case STM32MP25_CA35SS_SYSCFG_ETR_LPI_CR:
	case STM32MP25_CA35SS_SYSCFG_STM_NSGUAREN_CR:
	case STM32MP25_CA35SS_SYSCFG_TRACE_CLK_DIV_CR:
	case STM32MP25_CA35SS_SYSCFG_DBGPWR_CR:
	case STM32MP25_CA35SS_SYSCFG_DBGL1RSTDISABLE_CR:
	case STM32MP25_CA35SS_SYSCFG_GIC_CFGR:
	case STM32MP25_CA35SS_SYSCFG_M33_TZEN_CR:
	case STM32MP25_CA35SS_SYSCFG_M33_INITSVTOR_CR:
	case STM32MP25_CA35SS_SYSCFG_M33_INITNSVTOR_CR:
		return true;

	case STM32MP25_CA35SS_SYSCFG_ETR_LPI_SR:
	case STM32MP25_CA35SS_SYSCFG_DBGPWR_SR:
	case STM32MP25_CA35SS_SYSCFG_EDBGACK_SR:
	case STM32MP25_CA35SS_SYSCFG_LP_SR:
	case STM32MP25_CA35SS_SYSCFG_RSTACK_SR:
	case STM32MP25_CA35SS_SYSCFG_AARCH_MODE_CR:
	case STM32MP25_CA35SS_SYSCFG_VBAR_CR:
	case STM32MP25_CA35SS_SYSCFG_M33_ACCESS_CR:
	default:
		return false;
	}
}

static const struct regmap_config stm32_syscon_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.readable_reg = stm32_syscon_readable_reg,
	.writeable_reg = stm32_syscon_writeable_reg,
};

static int stm32_syscon_probe(struct platform_device *pdev)
{
	struct regmap_config syscon_config = stm32_syscon_regmap_config;
	struct device *dev = &pdev->dev;
	struct regmap *regmap;
	struct resource *res;
	void __iomem *base;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;

	base = devm_ioremap(dev, res->start, resource_size(res));
	if (!base)
		return -ENOMEM;

	syscon_config.max_register = resource_size(res) - syscon_config.reg_stride;

	regmap = devm_regmap_init_mmio(dev, base, &syscon_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "regmap init failed\n");

	return devm_of_syscon_register_regmap(dev, dev->of_node, regmap);
}

static const struct of_device_id stm32_syscon_ids[] = {
	{ .compatible = "st,stm32mp25-a35ss-syscfg" },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_syscon_ids);

static struct platform_driver stm32_syscon_driver = {
	.probe	= stm32_syscon_probe,
	.driver	= {
		.name = "stm32_syscon",
		.of_match_table = stm32_syscon_ids,
	},
};

module_platform_driver(stm32_syscon_driver);

MODULE_AUTHOR("Antonio Borneo <antonio.borneo@foss.st.com>");
MODULE_DESCRIPTION("STM32MP SYSCON driver");
MODULE_LICENSE("GPL");
