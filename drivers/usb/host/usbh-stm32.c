// SPDX-License-Identifier: GPL-2.0-only
/*
 * STM32 USB2 EHCI/OHCI (USBH) controller glue driver
 *
 * Copyright (C) 2022 STMicroelectronics – All Rights Reserved
 *
 * Author: Pankaj Dev <pankaj.dev@st.com>
 */

#include <linux/usb.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>

#define SYSCFG_USBHCR_OVRCUR_POLARITY_MASK      BIT(0)
#define SYSCFG_USBHCR_VBUSEN_POLARITY_MASK      BIT(1)

/**
 * struct stm32_usbh - usbh-stm32 driver private structure
 * @dev:			device pointer
 * @regmap:			regmap pointer for getting syscfg
 * @syscfg_usbhcr_reg_off:	usbhcr syscfg control offset
 * @vbusen_polarity_low:	vbusen signal polarity
 * @ovrcur_polarity_low:	ovrcur signal polarity
 * @irq_wakeup:			wakeup irq
 */
struct stm32_usbh {
	struct device *dev;
	struct regmap *regmap;
	int syscfg_usbhcr_reg_off;
	bool vbusen_polarity_low;
	bool ovrcur_polarity_low;
	int irq_wakeup;
};

/**
 * stm32_usbh_init: init the controller via glue logic
 * @usbh_data: driver private structure
 */
static int stm32_usbh_init(struct stm32_usbh *usbh_data)
{
	return regmap_update_bits(usbh_data->regmap, usbh_data->syscfg_usbhcr_reg_off,
				  SYSCFG_USBHCR_OVRCUR_POLARITY_MASK |
				  SYSCFG_USBHCR_VBUSEN_POLARITY_MASK,
				  FIELD_PREP(SYSCFG_USBHCR_OVRCUR_POLARITY_MASK,
					     usbh_data->ovrcur_polarity_low) |
				  FIELD_PREP(SYSCFG_USBHCR_VBUSEN_POLARITY_MASK,
					     usbh_data->vbusen_polarity_low));
}

static irqreturn_t stm32_usbh_irq_wakeup_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static int stm32_usbh_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct regmap *regmap;
	struct stm32_usbh *usbh_data;
	int ret, irq;

	if (usb_disabled())
		return -ENODEV;

	usbh_data = devm_kzalloc(dev, sizeof(*usbh_data), GFP_KERNEL);
	if (!usbh_data)
		return -ENOMEM;

	regmap = syscon_regmap_lookup_by_phandle(node, "st,syscfg");
	if (IS_ERR(regmap))
		return dev_err_probe(&pdev->dev, PTR_ERR(regmap), "no st,syscfg node found\n");

	ret = of_property_read_u32_index(node, "st,syscfg", 1, &usbh_data->syscfg_usbhcr_reg_off);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "can't get usbhcr offset\n");

	dev_vdbg(&pdev->dev, "syscfg-usbhcr-reg offset 0x%x\n", usbh_data->syscfg_usbhcr_reg_off);

	usbh_data->dev = dev;
	usbh_data->regmap = regmap;

	if (device_property_read_bool(dev, "st,vbusen-active-low"))
		usbh_data->vbusen_polarity_low = true;
	if (device_property_read_bool(dev, "st,ovrcur-active-low"))
		usbh_data->ovrcur_polarity_low = true;

	/* ST USBH glue logic init */
	ret = stm32_usbh_init(usbh_data);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "err setting syscfg_usbhcr_reg\n");

	if (device_property_read_bool(dev, "wakeup-source")) {
		irq = platform_get_irq(pdev, 0);
		if (irq < 0)
			return dev_err_probe(dev, irq, "failed to get IRQ\n");
		usbh_data->irq_wakeup = irq;

		ret = devm_request_threaded_irq(dev, usbh_data->irq_wakeup, NULL,
						stm32_usbh_irq_wakeup_handler, IRQF_ONESHOT,
						NULL, NULL);
		if (ret)
			return dev_err_probe(dev, ret, "unable to request wake IRQ %d\n",
					     usbh_data->irq_wakeup);

		device_set_wakeup_capable(dev, true);
	}

	/* Populate the ehci and ohci child nodes */
	ret = devm_of_platform_populate(dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to add ehci/ohci devices\n");

	platform_set_drvdata(pdev, usbh_data);

	return 0;
}

static int __maybe_unused stm32_usbh_suspend(struct device *dev)
{
	struct stm32_usbh *usbh_data = dev_get_drvdata(dev);

	if (device_may_wakeup(dev) || device_wakeup_path(dev)) {
		enable_irq_wake(usbh_data->irq_wakeup);
		return 0;
	}

	return pinctrl_pm_select_sleep_state(dev);
}

static int __maybe_unused stm32_usbh_resume(struct device *dev)
{
	struct stm32_usbh *usbh_data = dev_get_drvdata(dev);
	int err;

	if (device_may_wakeup(dev) || device_wakeup_path(dev)) {
		disable_irq_wake(usbh_data->irq_wakeup);
		return 0;
	}

	err = pinctrl_pm_select_default_state(dev);
	if (err) {
		dev_err(dev, "pinctrl select during resume error:%d\n", err);
		return err;
	}

	/* ST USBH glue logic init */
	err = stm32_usbh_init(usbh_data);
	if (err) {
		dev_err(dev, "err setting syscfg_usbhcr_reg (%d)\n", err);
		return err;
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(stm32_usbh_pm_ops, stm32_usbh_suspend, stm32_usbh_resume);

static const struct of_device_id stm32_usbh_match[] = {
	{ .compatible = "st,stm32mp25-usbh" },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, stm32_usbh_match);

static struct platform_driver stm32_usbh_driver = {
	.probe = stm32_usbh_probe,
	.driver = {
		.name = "stm32-usbh",
		.of_match_table = stm32_usbh_match,
		.pm = &stm32_usbh_pm_ops,
	},
};

module_platform_driver(stm32_usbh_driver);

MODULE_AUTHOR("Pankaj Dev <pankaj.dev@st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 EHCI/OHCI Specific Glue layer");
MODULE_LICENSE("GPL v2");
