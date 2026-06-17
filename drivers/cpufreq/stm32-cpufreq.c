// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) STMicroelectronics 2019 - All Rights Reserved
 * Authors: Alexandre Torgue <alexandre.torgue@st.com> for STMicroelectronics.
 */

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>

struct stm32_cpufreq_priv {
	int token;
	struct platform_device *cpufreq_dt_pdev;
};

static int stm32_cpufreq_probe(struct platform_device *pdev)
{
	struct stm32_cpufreq_priv *priv;
	struct device_node *opp_node;
	struct device *cpu_dev;
	u8 part_number;
	u32 supported_hw;
	int ret;

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev) {
		dev_err(&pdev->dev, "failed to get cpu0 device\n");
		return -ENODEV;
	}
	opp_node = dev_pm_opp_of_get_opp_desc_node(cpu_dev);
	if (!opp_node) {
		dev_dbg(&pdev->dev, "OPP-v2 not supported\n");
		return -ENODEV;
	}

	/* Get chip info */
	ret = nvmem_cell_read_u8(cpu_dev, "speed_grade", &part_number);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "Failed to get chip info: %d\n", ret);
		return ret;
	}

	supported_hw = BIT((part_number & 0x80) >> 7);

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->token = dev_pm_opp_set_supported_hw(cpu_dev, &supported_hw, 1);
	if (priv->token < 0) {
		ret = priv->token;
		dev_err_probe(&pdev->dev, ret, "Failed to set supported opp: %d\n", ret);
		return ret;
	}

	of_node_put(opp_node);
	priv->cpufreq_dt_pdev = platform_device_register_simple("cpufreq-dt",
								-1, NULL, 0);

	platform_set_drvdata(pdev, priv);

	return 0;
}

static void stm32_cpufreq_remove(struct platform_device *pdev)
{
	struct stm32_cpufreq_priv *priv	= platform_get_drvdata(pdev);

	platform_device_unregister(priv->cpufreq_dt_pdev);
	dev_pm_opp_put_supported_hw(priv->token);
}

static int stm32_cpufreq_init(void)
{
	platform_device_register_simple("stm32-cpufreq", -1, NULL, 0);

	return 0;
}
module_init(stm32_cpufreq_init);

static struct platform_driver stm32_cpufreq_platdrv = {
	.driver = {
		.name	= "stm32-cpufreq",
	},
	.probe		= stm32_cpufreq_probe,
	.remove		= stm32_cpufreq_remove,
};
module_platform_driver(stm32_cpufreq_platdrv);

MODULE_DESCRIPTION("STM32 CPU freq driver");
MODULE_AUTHOR("Alexandre Torgue <alexandre.torgue@st.com>");
MODULE_LICENSE("GPL");
