// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) STMicroelectronics 2023 - All Rights Reserved
 * Author: Gabriel Fernandez <gabriel.fernandez@foss.st.com> for STMicroelectronics.
 */

#include <linux/clk-provider.h>
#include <linux/clk-regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define SYSCFG_DISPLAYCLKCR	0x5000

static const char * const clk_pix_ltdc_src[] = {
	"txbyteclk", "clk_pix_lvds", "ck_ker_ltdc"
};

static u8 clk_regmap_mux_get_parent(struct clk_hw *hw)
{
	return clk_regmap_mux_ops.get_parent(hw);
}

static int clk_regmap_mux_set_parent(struct clk_hw *hw, u8 index)
{
	return clk_regmap_mux_ops.set_parent(hw, index);
}

static int clk_regmap_mux_determine_rate(struct clk_hw *hw,
					 struct clk_rate_request *req)
{
	return clk_regmap_mux_ops.determine_rate(hw, req);
}

#ifdef CONFIG_PM_SLEEP
static void clk_regmap_mux_pm_restore(struct clk_hw *hw)
{
	struct clk_hw *hwp = clk_hw_get_parent(hw);
	int index;

	if (hwp != NULL) {
		index = clk_hw_get_parent_index(hw);
		clk_regmap_mux_set_parent(hw, index);
	}
}
#endif

static const struct clk_ops clk_syscfg_mux_ops = {
	.get_parent = clk_regmap_mux_get_parent,
	.set_parent = clk_regmap_mux_set_parent,
	.determine_rate = clk_regmap_mux_determine_rate,
#ifdef CONFIG_PM_SLEEP
	.restore_context = clk_regmap_mux_pm_restore,
#endif
};

static struct clk_regmap clk_pix_ltdc = {
	.data = &(struct clk_regmap_mux_data){
		.offset = SYSCFG_DISPLAYCLKCR,
		.mask = 0x3,
		.shift = 0,
	},
	.hw.init = CLK_HW_INIT_PARENTS("clk_pix_ltdc", clk_pix_ltdc_src, &clk_syscfg_mux_ops,
				       CLK_SET_RATE_PARENT | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_hw *stm32_syscfg_mp25_hw_clocks[] = {
		&clk_pix_ltdc.hw,
};

struct stm32_syscfg_match_data {
	unsigned int	num_clocks;
	struct clk_hw	**tab_clocks;
};

static struct stm32_syscfg_match_data syscfg_mp25_data = {
	.tab_clocks	= stm32_syscfg_mp25_hw_clocks,
	.num_clocks	= ARRAY_SIZE(stm32_syscfg_mp25_hw_clocks),
};

static int stm32mp25_syscfg_clocks_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	const struct stm32_syscfg_match_data *data;
	struct regmap *regmap;
	struct clk_hw_onecell_data *hw_data;
	int ret, n;

	data = device_get_match_data(dev);
	if (!data)
		return -EINVAL;

	regmap = syscon_node_to_regmap(np);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	hw_data = devm_kzalloc(dev, struct_size(hw_data, hws, data->num_clocks), GFP_KERNEL);
	if (!hw_data)
		return -ENOMEM;

	hw_data->num = data->num_clocks;

	for (n = 0; n < data->num_clocks; n++) {
		struct clk_hw *hw = data->tab_clocks[n];
		struct clk_regmap *clk = to_clk_regmap(hw);

		clk->map = regmap;

		ret = devm_clk_hw_register(dev, hw);
		if (ret) {
			dev_warn(dev, "failed to register %s\n", hw->init->name);
			hw = ERR_PTR(-ENOENT);
		}

		hw_data->hws[n] = hw;
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, hw_data);
	if (ret)
		return ret;

	return ret;
}

static const struct of_device_id stm32mp25_syscfg_match_data[] = {
	{
		.compatible = "st,stm32mp25-syscfg",
		.data = &syscfg_mp25_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, stm32mp25_syscfg_match_data);

static struct platform_driver stm32mp25_syscfg_driver = {
	.probe = stm32mp25_syscfg_clocks_probe,
	.driver = {
		.name = "stm32mp25_syscfg",
		.of_match_table = stm32mp25_syscfg_match_data,
	},
};
module_platform_driver(stm32mp25_syscfg_driver);
