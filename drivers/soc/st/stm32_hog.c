// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024, STMicroelectronics - All Rights Reserved
 */

#include <linux/clk.h>
#include "linux/device.h"
#include <linux/of_address.h>
#include <linux/platform_device.h>

struct stm32_hog_pdata {
	struct clk_bulk_data *clks;
	int num_clks;
};

static void stm32_hog_remove(struct platform_device *pdev)
{
	struct stm32_hog_pdata *priv = platform_get_drvdata(pdev);

	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);
}

static int stm32_hog_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stm32_hog_pdata *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->num_clks = devm_clk_bulk_get_all(dev, &priv->clks);
	if (priv->num_clks < 1)
		return -ENODEV;

	ret = clk_bulk_prepare_enable(priv->num_clks, priv->clks);
	if (ret) {
		dev_err(dev, "failed to enable bulk clks %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	return 0;
}

static const struct of_device_id stm32_hog_match[] = {
	{ .compatible = "st,stm32-hog", },
	{}
};
MODULE_DEVICE_TABLE(of, stm32_hog_match);

static struct platform_driver stm32_hog_driver = {
	.probe  = stm32_hog_probe,
	.remove = stm32_hog_remove,
	.driver = {
		.name = "stm32-hog",
		.of_match_table = stm32_hog_match,
	},
};
module_platform_driver(stm32_hog_driver);
MODULE_AUTHOR("Gabriel Fernandez <gabriel.fernandez@foss.st.com>");
MODULE_LICENSE("GPL");
