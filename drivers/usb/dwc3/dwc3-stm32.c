// SPDX-License-Identifier: GPL-2.0-only
/*
 * dwc3-stm32.c - DWC3 Specific Glue layer for STM32
 *
 * Copyright (C) 2022 STMicroelectronics – All Rights Reserved
 *
 * Author: Pankaj Dev <pankaj.dev@st.com> for STMicroelectronics.
 *
 * Inspired by dwc3-st.c
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/usb/of.h>

#include "core.h"
#include "io.h"

#define SYSCFG_USB3DRCR_HOST_PORT_POWER_CONTROL_PRESENT_MASK     BIT(0)
#define SYSCFG_USB3DRCR_OVRCUR_POLARITY_MASK     BIT(1)
#define SYSCFG_USB3DRCR_VBUSEN_POLARITY_MASK     BIT(2)
#define SYSCFG_USB3DRCR_USB2ONLYH_MASK           BIT(3)             // USB2-only Mode for Host
#define SYSCFG_USB3DRCR_USB2ONLYD_MASK           BIT(4)             // USB2-only Mode for Device

/**
 * struct stm32_dwc3 - dwc3-stm32 driver private structure
 * @dev:			device pointer
 * @regmap:			regmap pointer for getting syscfg
 * @syscfg_usb3drcr_reg_off:	usb3drcr syscfg control offset
 * @dr_mode:			drd static host/device config
 * @usb2only_conf:		If only usb2-phy is present
 * @vbusen_polarity_low:	vbusen signal polarity
 * @ovrcur_polarity_low:	ovrcur signal polarity
 * @prt_pwr_ctrl:		enable port power control
 * @irq_wakeup:			wakeup irq
 */
struct stm32_dwc3 {
	struct device *dev;
	struct regmap *regmap;
	int syscfg_usb3drcr_reg_off;
	bool usb2only_conf;
	bool vbusen_polarity_low;
	bool ovrcur_polarity_low;
	bool prt_pwr_ctrl;
	int irq_wakeup;
};

/**
 * stm32_dwc3_init: init the controller via glue logic
 * @dwc3_data: driver private structure
 */
static int stm32_dwc3_init(struct stm32_dwc3 *dwc3_data)
{
	return regmap_update_bits(dwc3_data->regmap, dwc3_data->syscfg_usb3drcr_reg_off,
				  SYSCFG_USB3DRCR_HOST_PORT_POWER_CONTROL_PRESENT_MASK |
				  SYSCFG_USB3DRCR_OVRCUR_POLARITY_MASK |
				  SYSCFG_USB3DRCR_VBUSEN_POLARITY_MASK |
				  SYSCFG_USB3DRCR_USB2ONLYD_MASK |
				  SYSCFG_USB3DRCR_USB2ONLYH_MASK,
				  FIELD_PREP(SYSCFG_USB3DRCR_HOST_PORT_POWER_CONTROL_PRESENT_MASK,
					     dwc3_data->prt_pwr_ctrl) |
				  FIELD_PREP(SYSCFG_USB3DRCR_OVRCUR_POLARITY_MASK,
					     dwc3_data->ovrcur_polarity_low) |
				  FIELD_PREP(SYSCFG_USB3DRCR_VBUSEN_POLARITY_MASK,
					     dwc3_data->vbusen_polarity_low) |
				  FIELD_PREP(SYSCFG_USB3DRCR_USB2ONLYD_MASK,
					    dwc3_data->usb2only_conf ? 1 : 0) |
				  FIELD_PREP(SYSCFG_USB3DRCR_USB2ONLYH_MASK,
					    dwc3_data->usb2only_conf ? 1 : 0));

}

static irqreturn_t stm32_dwc3_irq_wakeup_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static int stm32_dwc3_set_wakeup_capable(struct device *dev, void *data)
{
	device_set_wakeup_capable(dev, true);

	return 0;
}

static int stm32_dwc3_probe(struct platform_device *pdev)
{
	struct stm32_dwc3 *dwc3_data;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node, *child;
	struct regmap *regmap;
	int ret, irq;
	bool wakeup_source;
	struct reset_control *reset;

	dwc3_data = devm_kzalloc(dev, sizeof(*dwc3_data), GFP_KERNEL);
	if (!dwc3_data)
		return -ENOMEM;

	regmap = syscon_regmap_lookup_by_phandle(node, "st,syscfg");
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ret = of_property_read_u32_index(node, "st,syscfg", 1,
					 &dwc3_data->syscfg_usb3drcr_reg_off);
	if (ret) {
		dev_err(dev, "can't get usb3drcr offset (%d)\n", ret);
		return ret;
	}
	dev_vdbg(&pdev->dev, "syscfg-usb3drcr-reg offset 0x%x\n",
		 dwc3_data->syscfg_usb3drcr_reg_off);

	dwc3_data->dev = dev;
	dwc3_data->regmap = regmap;

	if (device_property_read_bool(dev, "st,vbusen-active-low"))
		dwc3_data->vbusen_polarity_low = true;
	if (device_property_read_bool(dev, "st,ovrcur-active-low"))
		dwc3_data->ovrcur_polarity_low = true;

	if (device_property_read_bool(dev, "st,enable-port-power-control"))
		dwc3_data->prt_pwr_ctrl = true;

	child = of_get_child_by_name(node, "usb");
	if (!child)
		return dev_err_probe(dev, -ENODEV, "failed to find dwc3 core node\n");

	/*
	 * Reset DWC3 IP required to clear the usb2only settings inside the ctrl
	 * since if any other module (loaded before linux) uses dwc3 in usb2only mode
	 * then there is an issue inside ctrl unless reset is asserted. Dwc3 core
	 * deasserts the reset line during probe which is not enough since if reset
	 * is already deasserted by some bootloader module then no reset is performed
	 * on dwc3 ctrl, hence here we forcefully assert the ctrl reset line
	 */
	reset = of_reset_control_array_get_exclusive(child);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "failed to get reset handle\n");

	ret = reset_control_assert(reset);
	reset_control_put(reset);
	if (ret) {
		dev_err(dev, "failed to assert reset (%d)\n", ret);
		return ret;
	}

	/* check if usb3-phy present, means NO usb2only mode */
	dwc3_data->usb2only_conf = true;
	if (of_count_phandle_with_args(child, "usb-phy", NULL) == 2) {
		dwc3_data->usb2only_conf = false;
	} else {
		if (of_property_match_string(child, "phy-names", "usb3-phy") >= 0)
			dwc3_data->usb2only_conf = false;
	}
	dev_info(&pdev->dev, "configured in %s mode\n", dwc3_data->usb2only_conf ? "usb2" : "usb3");

	of_node_put(child);

	/* ST glue logic init */
	ret = stm32_dwc3_init(dwc3_data);
	if (ret)
		return dev_err_probe(dev, ret, "dwc3 glue init failed\n");

	wakeup_source = device_property_read_bool(dev, "wakeup-source");
	if (wakeup_source) {
		irq = platform_get_irq(pdev, 0);
		if (irq < 0)
			return dev_err_probe(&pdev->dev, irq, "failed to get IRQ\n");
		dwc3_data->irq_wakeup = irq;

		ret = devm_request_threaded_irq(dev, dwc3_data->irq_wakeup, NULL,
						stm32_dwc3_irq_wakeup_handler, IRQF_ONESHOT,
						NULL, NULL);
		if (ret)
			return dev_err_probe(dev, ret, "unable to request wake IRQ %d\n",
					     dwc3_data->irq_wakeup);

		device_set_wakeup_capable(dev, true);
	}

	/* Allocate and initialize the core */
	ret = devm_of_platform_populate(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add dwc3 core\n");

	platform_set_drvdata(pdev, dwc3_data);

	if (wakeup_source)
		device_for_each_child(dev, NULL, stm32_dwc3_set_wakeup_capable);

	return 0;
}

static int __maybe_unused stm32_dwc3_suspend(struct device *dev)
{
	struct stm32_dwc3 *dwc3_data = dev_get_drvdata(dev);

	if (device_may_wakeup(dev) || device_wakeup_path(dev)) {
		enable_irq_wake(dwc3_data->irq_wakeup);
		return 0;
	}

	return pinctrl_pm_select_sleep_state(dev);
}

static int __maybe_unused stm32_dwc3_resume(struct device *dev)
{
	struct stm32_dwc3 *dwc3_data = dev_get_drvdata(dev);
	int ret;

	if (device_may_wakeup(dev) || device_wakeup_path(dev)) {
		disable_irq_wake(dwc3_data->irq_wakeup);
		return 0;
	}

	ret = pinctrl_pm_select_default_state(dev);
	if (ret) {
		dev_err(dev, "pinctrl select during resume (%d)\n", ret);
		return ret;
	}

	/* ST glue logic init */
	ret = stm32_dwc3_init(dwc3_data);
	if (ret) {
		dev_err(dev, "err setting syscfg_usb3drcr_reg (%d)\n", ret);
		return ret;
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(stm32_dwc3_dev_pm_ops, stm32_dwc3_suspend, stm32_dwc3_resume);

static const struct of_device_id stm32_dwc3_match[] = {
	{ .compatible = "st,stm32mp25-dwc3" },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, stm32_dwc3_match);

static struct platform_driver stm32_dwc3_driver = {
	.probe = stm32_dwc3_probe,
	.driver = {
		.name = "usb-stm32-dwc3",
		.of_match_table = stm32_dwc3_match,
		.pm = &stm32_dwc3_dev_pm_ops,
	},
};

module_platform_driver(stm32_dwc3_driver);

MODULE_AUTHOR("Pankaj Dev <pankaj.dev@st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 DWC3 Specific Glue layer");
MODULE_LICENSE("GPL v2");
