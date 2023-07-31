// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) STMicroelectronics 2023 - All Rights Reserved
 * Author: Gabriel Fernandez <gabriel.fernandez@foss.st.com> for STMicroelectronics.
 */

#include <linux/clk-provider.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define SYSCFG_DISPLAYCLKCR	0x5000

static const char * const clk_pix_ltdc_src[] = {
	"txbyteclk", "clk_pix_lvds", "ck_ker_ltdc"
};

/**
 * struct clk_regmap - regmap backed clock
 *
 * @hw:		handle between common and hardware-specific interfaces
 * @map:	pointer to the regmap structure controlling the clock
 * @data:	data specific to the clock type
 *
 * Clock which is controlled by regmap backed registers. The actual type
 * of the clock is controlled by the clock_ops and data.
 */
struct clk_regmap {
	struct clk_hw	hw;
	struct regmap	*map;
	void		*data;
};

static inline struct clk_regmap *to_clk_regmap(struct clk_hw *hw)
{
	return container_of(hw, struct clk_regmap, hw);
}

/**
 * struct clk_regmap_mux_data - regmap backed multiplexer clock specific data
 *
 * @hw:		handle between common and hardware-specific interfaces
 * @offset:	offset of theregister controlling multiplexer
 * @table:	array of parent indexed register values
 * @shift:	shift to multiplexer bit field
 * @mask:	mask of mutliplexer bit field
 * @flags:	hardware-specific flags
 *
 * Flags:
 * Same as clk_divider except CLK_MUX_HIWORD_MASK which is ignored
 */
struct clk_regmap_mux_data {
	unsigned int	offset;
	u32		*table;
	u32		mask;
	u8		shift;
	u8		flags;
};

static inline struct clk_regmap_mux_data *
clk_get_regmap_mux_data(struct clk_regmap *clk)
{
	return (struct clk_regmap_mux_data *)clk->data;
}

static u8 clk_regmap_mux_get_parent(struct clk_hw *hw)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct clk_regmap_mux_data *mux = clk_get_regmap_mux_data(clk);
	unsigned int val;
	int ret;

	ret = regmap_read(clk->map, mux->offset, &val);
	if (ret)
		return ret;

	val >>= mux->shift;
	val &= mux->mask;
	return clk_mux_val_to_index(hw, mux->table, mux->flags, val);
}

static int clk_regmap_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct clk_regmap_mux_data *mux = clk_get_regmap_mux_data(clk);
	unsigned int val = clk_mux_index_to_val(mux->table, mux->flags, index);

	return regmap_update_bits(clk->map, mux->offset,
				  mux->mask << mux->shift,
				  val << mux->shift);
}

static int clk_regmap_mux_determine_rate(struct clk_hw *hw,
					 struct clk_rate_request *req)
{
	struct clk_regmap *clk = to_clk_regmap(hw);
	struct clk_regmap_mux_data *mux = clk_get_regmap_mux_data(clk);

	return clk_mux_determine_rate_flags(hw, req, mux->flags);
}

#ifdef CONFIG_PM_SLEEP
static void clk_regmap_mux_pm_restore(struct clk_hw *hw)
{
	struct clk_hw *hwp = clk_hw_get_parent(hw);
	int index;

	if (hwp) {
		index = clk_hw_get_parent_index(hw);
		clk_regmap_mux_set_parent(hw, index);
	}
}
#endif

static const struct clk_ops clk_regmap_mux_ops = {
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
	.hw.init = CLK_HW_INIT_PARENTS("clk_pix_ltdc", clk_pix_ltdc_src,
				       &clk_regmap_mux_ops,
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
