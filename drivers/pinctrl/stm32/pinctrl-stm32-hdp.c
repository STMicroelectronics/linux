// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) STMicroelectronics 2024 - All Rights Reserved
 * Author: Clément Le Goffic <clement.legoffic@foss.st.com> for STMicroelectronics.
 */
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#include "../core.h"
#include "../../gpio/gpiolib.h"

#define DRIVER_NAME		"stm32_hdp"
#define HDP_CTRL_ENABLE		1
#define HDP_CTRL_DISABLE	0
#define NB_FUNCTIONS		16

#define HDP_CTRL		0x000
#define HDP_MUX			0x004
#define HDP_VAL			0x010
#define HDP_GPOSET		0x014
#define HDP_GPOCLR		0x018
#define HDP_GPOVAL		0x01C
#define HDP_VERR		0x3F4
#define HDP_IPIDR		0x3F8
#define HDP_SIDR		0x3FC

#define HDP_MUX_SHIFT(n)	((n) * 4)
#define HDP_MUX_MASK(n)		(GENMASK(3, 0) << HDP_MUX_SHIFT(n))
#define HDP_MUX_GPOVAL(n)	(0xF << HDP_MUX_SHIFT(n))

struct stm32_hdp {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct pinctrl_dev *pctl_dev;
	struct gpio_chip gpio_chip;
	u32 mux_conf;
	u32 gposet_conf;
};

static const struct pinctrl_pin_desc stm32_hdp_pins[] = {
	PINCTRL_PIN(0, "hdp0"),
	PINCTRL_PIN(1, "hdp1"),
	PINCTRL_PIN(2, "hdp2"),
	PINCTRL_PIN(3, "hdp3"),
	PINCTRL_PIN(4, "hdp4"),
	PINCTRL_PIN(5, "hdp5"),
	PINCTRL_PIN(6, "hdp6"),
	PINCTRL_PIN(7, "hdp7"),
};

static const char * const func_name[] = {
	"0", "1", "2", "3",
	"4", "5", "6", "7",
	"8", "9", "10", "11",
	"12", "13", "14", "15",
};

static const char * const stm32_hdp_pins_group[] = {
	"hdp0",
	"hdp1",
	"hdp2",
	"hdp3",
	"hdp4",
	"hdp5",
	"hdp6",
	"hdp7"
};

static int stm32_hdp_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	return GPIO_LINE_DIRECTION_OUT;
}

static int stm32_hdp_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct stm32_hdp *hdp = gpiochip_get_data(gc);

	if (((hdp->mux_conf & HDP_MUX_MASK(offset))) == HDP_MUX_GPOVAL(offset))
		return !!(readl_relaxed(hdp->base + HDP_GPOVAL) & BIT(offset));
	else
		return !!(readl_relaxed(hdp->base + HDP_VAL) & BIT(offset));
}

static void stm32_hdp_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct stm32_hdp *hdp = gpiochip_get_data(gc);

	if (value)
		writel_relaxed(BIT(offset), hdp->base + HDP_GPOSET);
	else
		writel_relaxed(BIT(offset), hdp->base + HDP_GPOCLR);
}

static int stm32_hdp_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(stm32_hdp_pins);
}

static const char *stm32_hdp_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
						    unsigned int selector)
{
	return stm32_hdp_pins[selector].name;
}

static int stm32_hdp_pinctrl_get_group_pins(struct pinctrl_dev *pctldev, unsigned int selector,
					    const unsigned int **pins, unsigned int *num_pins)
{
	*pins = &stm32_hdp_pins[selector].number;
	*num_pins = 1;

	return 0;
}

static const struct pinctrl_ops stm32_hdp_pinctrl_ops = {
	.get_groups_count = stm32_hdp_pinctrl_get_groups_count,
	.get_group_name	  = stm32_hdp_pinctrl_get_group_name,
	.get_group_pins	  = stm32_hdp_pinctrl_get_group_pins,
	.dt_node_to_map	  = pinconf_generic_dt_node_to_map_all,
	.dt_free_map	  = pinconf_generic_dt_free_map,
};

static int stm32_hdp_pinmux_get_functions_count(struct pinctrl_dev *pctldev)
{
	return NB_FUNCTIONS;
}

static const char *stm32_hdp_pinmux_get_function_name(struct pinctrl_dev *pctldev,
						      unsigned int selector)
{
	return func_name[selector];
}

static int stm32_hdp_pinmux_get_function_groups(struct pinctrl_dev *pctldev, unsigned int selector,
						const char *const **groups,
						unsigned int *num_groups)
{
	*groups = stm32_hdp_pins_group;
	*num_groups = ARRAY_SIZE(stm32_hdp_pins_group);

	return 0;
}

static int stm32_hdp_pinmux_set_mux(struct pinctrl_dev *pctldev, unsigned int func_selector,
				    unsigned int group_selector)
{
	struct stm32_hdp *hdp = pinctrl_dev_get_drvdata(pctldev);
	unsigned int pin = stm32_hdp_pins[group_selector].number;
	u32 mux;

	mux = readl_relaxed(hdp->base + HDP_MUX);
	mux &= ~HDP_MUX_MASK(pin);
	mux |= func_selector << HDP_MUX_SHIFT(pin);

	writel_relaxed(mux, hdp->base + HDP_MUX);
	hdp->mux_conf = mux;

	return 0;
}

static const struct pinmux_ops stm32_hdp_pinmux_ops = {
	.get_functions_count = stm32_hdp_pinmux_get_functions_count,
	.get_function_name   = stm32_hdp_pinmux_get_function_name,
	.get_function_groups = stm32_hdp_pinmux_get_function_groups,
	.set_mux	     = stm32_hdp_pinmux_set_mux,
	.gpio_set_direction  = NULL,
};

static struct pinctrl_desc stm32_hdp_pdesc = {
	.name	 = DRIVER_NAME,
	.pins	 = stm32_hdp_pins,
	.npins	 = ARRAY_SIZE(stm32_hdp_pins),
	.pctlops = &stm32_hdp_pinctrl_ops,
	.pmxops	 = &stm32_hdp_pinmux_ops,
	.owner	 = THIS_MODULE,
};

static const struct of_device_id stm32_hdp_of_match[] = {
	{
		.compatible = "st,stm32mp-hdp",
	},
	{}
};

static int stm32_hdp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stm32_hdp *hdp;
	u8 version;
	int err;

	hdp = devm_kzalloc(dev, sizeof(*hdp), GFP_KERNEL);
	if (!hdp)
		return -ENOMEM;
	hdp->dev = dev;

	platform_set_drvdata(pdev, hdp);

	hdp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hdp->base))
		return PTR_ERR(hdp->base);

	hdp->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(hdp->clk))
		return dev_err_probe(dev, PTR_ERR(hdp->clk), "No HDP clock provided\n");

	err = devm_pinctrl_register_and_init(dev, &stm32_hdp_pdesc, hdp, &hdp->pctl_dev);
	if (err) {
		dev_err(dev, "pinctrl register failed\n");
		return err;
	}

	err = pinctrl_enable(hdp->pctl_dev);
	if (err) {
		dev_err(dev, "pinctrl enable failed\n");
		return err;
	}

	hdp->gpio_chip.label	     = "stm32-hdp";
	hdp->gpio_chip.parent	     = dev;
	hdp->gpio_chip.get_direction = stm32_hdp_gpio_get_direction;
	hdp->gpio_chip.get	     = stm32_hdp_gpio_get;
	hdp->gpio_chip.set	     = stm32_hdp_gpio_set;
	hdp->gpio_chip.base	     = -1;
	hdp->gpio_chip.ngpio	     = ARRAY_SIZE(stm32_hdp_pins);
	hdp->gpio_chip.can_sleep     = true;
	hdp->gpio_chip.names	     = stm32_hdp_pins_group;

	err = devm_gpiochip_add_data(dev, &hdp->gpio_chip, hdp);
	if (err) {
		dev_err(dev, "Failed to add gpiochip\n");
		return err;
	}

	writel_relaxed(HDP_CTRL_ENABLE, hdp->base + HDP_CTRL);

	version = readl_relaxed(hdp->base + HDP_VERR);
	dev_dbg(dev, "STM32 HDP version %u.%u initialized\n", version >> 4, version & 0x0F);

	return 0;
}

static int stm32_hdp_remove(struct platform_device *pdev)
{
	struct stm32_hdp *hdp = platform_get_drvdata(pdev);

	writel_relaxed(HDP_CTRL_DISABLE, hdp->base + HDP_CTRL);

	return 0;
}

static int stm32_hdp_suspend(struct device *dev)
{
	struct stm32_hdp *hdp = dev_get_drvdata(dev);

	hdp->gposet_conf = readl_relaxed(hdp->base + HDP_GPOSET);

	pinctrl_pm_select_sleep_state(dev);

	clk_disable_unprepare(hdp->clk);

	return 0;
}

static int stm32_hdp_resume(struct device *dev)
{
	struct stm32_hdp *hdp = dev_get_drvdata(dev);
	int err;

	err = clk_prepare_enable(hdp->clk);
	if (err) {
		dev_err(hdp->dev, "failed to prepare_enable clk (%d)\n", err);
		return err;
	}
	writel_relaxed(HDP_CTRL_ENABLE, hdp->base + HDP_CTRL);
	writel_relaxed(hdp->gposet_conf, hdp->base + HDP_GPOSET);
	writel_relaxed(hdp->mux_conf, hdp->base + HDP_MUX);

	pinctrl_pm_select_default_state(dev);

	return 0;
}

DEFINE_SIMPLE_DEV_PM_OPS(stm32_hdp_pm_ops, stm32_hdp_suspend, stm32_hdp_resume);

static struct platform_driver stm32_hdp_driver = {
	.probe = stm32_hdp_probe,
	.remove = stm32_hdp_remove,
	.driver = {
		.name = DRIVER_NAME,
		.pm = pm_sleep_ptr(&stm32_hdp_pm_ops),
		.of_match_table = stm32_hdp_of_match,
	}
};

module_platform_driver(stm32_hdp_driver);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Clément Le Goffic");
MODULE_DESCRIPTION("STMicroelectronics STM32 Hardware Debug Port driver");
MODULE_LICENSE("GPL");
