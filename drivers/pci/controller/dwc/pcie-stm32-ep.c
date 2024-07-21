// SPDX-License-Identifier: GPL-2.0-only
/*
 * ST PCIe Endpoint driver for STM32-MP25 SoC
 *
 * Copyright (C) 2023 ST Microelectronics - All Rights Reserved
 * Author: Christian Bruel <christian.bruel@foss.st.com>
 */

#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/phy/phy.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include "pcie-designware.h"
#include "pcie-stm32.h"

enum stm32_pcie_ep_link_status {
	STM32_PCIE_EP_LINK_DISABLED,
	STM32_PCIE_EP_LINK_ENABLED,
};

struct stm32_pcie {
	struct dw_pcie *pci;
	struct regmap *regmap;
	struct reset_control *rst;
	struct phy *phy;
	struct clk *clk;
	struct gpio_desc *reset_gpio;
	enum stm32_pcie_ep_link_status link_status;
	unsigned int perst_irq;
};

static const struct of_device_id stm32_pcie_ep_of_match[] = {
	{ .compatible = "st,stm32mp25-pcie-ep" },
	{},
};

static void stm32_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct stm32_pcie *stm32_pcie = to_stm32_pcie(pci);
	enum pci_barno bar;

	for (bar = BAR_0; bar <= PCI_STD_NUM_BARS; bar++)
		dw_pcie_ep_reset_bar(pci, bar);

	/* Defer Completion Requests until link started */
	regmap_update_bits(stm32_pcie->regmap, SYSCFG_PCIECR,
			   STM32MP25_PCIECR_REQ_RETRY_EN,
			   STM32MP25_PCIECR_REQ_RETRY_EN);
}

static int stm32_pcie_enable_link(struct dw_pcie *pci)
{
	struct stm32_pcie *stm32_pcie = to_stm32_pcie(pci);
	int ret;

	regmap_update_bits(stm32_pcie->regmap, SYSCFG_PCIECR,
			   STM32MP25_PCIECR_LTSSM_EN,
			   STM32MP25_PCIECR_LTSSM_EN);

	ret = dw_pcie_wait_for_link(pci);
	if (ret)
		return ret;

	regmap_update_bits(stm32_pcie->regmap, SYSCFG_PCIECR,
			   STM32MP25_PCIECR_REQ_RETRY_EN,
			   0);

	return 0;
}

static void stm32_pcie_disable_link(struct dw_pcie *pci)
{
	struct stm32_pcie *stm32_pcie = to_stm32_pcie(pci);

	regmap_update_bits(stm32_pcie->regmap, SYSCFG_PCIECR,
			   STM32MP25_PCIECR_REQ_RETRY_EN,
			   STM32MP25_PCIECR_REQ_RETRY_EN);

	regmap_update_bits(stm32_pcie->regmap, SYSCFG_PCIECR, STM32MP25_PCIECR_LTSSM_EN, 0);
}

static int stm32_pcie_start_link(struct dw_pcie *pci)
{
	struct stm32_pcie *stm32_pcie = to_stm32_pcie(pci);
	int ret;

	if (stm32_pcie->link_status == STM32_PCIE_EP_LINK_ENABLED) {
		dev_dbg(pci->dev, "Link is already enabled\n");
		return 0;
	}

	ret = stm32_pcie_enable_link(pci);
	if (ret) {
		dev_err(pci->dev, "PCIe cannot establish link: %d\n", ret);
		return ret;
	}

	stm32_pcie->link_status = STM32_PCIE_EP_LINK_ENABLED;

	enable_irq(stm32_pcie->perst_irq);

	return 0;
}

static void stm32_pcie_stop_link(struct dw_pcie *pci)
{
	struct stm32_pcie *stm32_pcie = to_stm32_pcie(pci);

	if (stm32_pcie->link_status == STM32_PCIE_EP_LINK_DISABLED) {
		dev_dbg(pci->dev, "Link is already disabled\n");
		return;
	}

	disable_irq(stm32_pcie->perst_irq);

	stm32_pcie_disable_link(pci);

	stm32_pcie->link_status = STM32_PCIE_EP_LINK_DISABLED;
}

static int stm32_pcie_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
				enum pci_epc_irq_type type, u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return dw_pcie_ep_raise_legacy_irq(ep, func_no);
	case PCI_EPC_IRQ_MSI:
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	default:
		dev_err(pci->dev, "UNKNOWN IRQ type\n");
		return -EINVAL;
	}

	return 0;
}

static const struct pci_epc_features stm32_pcie_epc_features = {
	.force_core_init = true,
	.core_init_notifier = true,
	.msi_capable = true,
	.align = 1 << 16,
};

static const struct pci_epc_features*
stm32_pcie_get_features(struct dw_pcie_ep *ep)
{
	return &stm32_pcie_epc_features;
}

static const struct dw_pcie_ep_ops stm32_pcie_ep_ops = {
	.ep_init = stm32_pcie_ep_init,
	.raise_irq = stm32_pcie_raise_irq,
	.get_features = stm32_pcie_get_features,
};

static const struct dw_pcie_ops dw_pcie_ops = {
	.start_link = stm32_pcie_start_link,
	.stop_link = stm32_pcie_stop_link,
};

static int stm32_pcie_enable_resources(struct stm32_pcie *stm32_pcie)
{
	int ret;

	ret = phy_init(stm32_pcie->phy);
	if (ret)
		return ret;

	ret = clk_prepare_enable(stm32_pcie->clk);
	if (ret)
		phy_exit(stm32_pcie->phy);

	return ret;
}

static void stm32_pcie_disable_resources(struct stm32_pcie *stm32_pcie)
{
	clk_disable_unprepare(stm32_pcie->clk);

	phy_exit(stm32_pcie->phy);
}

static void stm32_pcie_perst_assert(struct dw_pcie *pci)
{
	struct stm32_pcie *stm32_pcie = to_stm32_pcie(pci);
	struct device *dev = pci->dev;

	dev_dbg(dev, "PERST asserted by host. Shutting down the PCIe link\n");

	/*
	 * Do not try to release resources if the PERST# is
	 * asserted before the link is started.
	 */
	if (stm32_pcie->link_status == STM32_PCIE_EP_LINK_DISABLED) {
		dev_dbg(pci->dev, "Link is already disabled\n");
		return;
	}

	stm32_pcie_disable_link(pci);

	stm32_pcie_disable_resources(stm32_pcie);

	pm_runtime_put_sync(dev);

	stm32_pcie->link_status = STM32_PCIE_EP_LINK_DISABLED;
}

static void stm32_pcie_perst_deassert(struct dw_pcie *pci)
{
	struct stm32_pcie *stm32_pcie = to_stm32_pcie(pci);
	struct device *dev = pci->dev;
	struct dw_pcie_ep *ep = &pci->ep;
	int ret;

	if (stm32_pcie->link_status == STM32_PCIE_EP_LINK_ENABLED) {
		dev_dbg(pci->dev, "Link is already enabled\n");
		return;
	}

	dev_dbg(dev, "PERST de-asserted by host. Starting link training\n");

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		dev_err(dev, "pm runtime resume failed: %d\n", ret);
		return;
	}

	ret = stm32_pcie_enable_resources(stm32_pcie);
	if (ret) {
		dev_err(dev, "Failed to enable resources: %d\n", ret);
		pm_runtime_put_sync(dev);
		return;
	}

	ret = stm32_pcie_enable_link(pci);
	if (ret) {
		dev_err(dev, "PCIe Cannot establish link: %d\n", ret);
		stm32_pcie_disable_resources(stm32_pcie);
		pm_runtime_put_sync(dev);
		return;
	}

	ret = dw_pcie_ep_init_complete(ep);
	if (ret) {
		dev_err(dev, "Failed to complete initialization: %d\n", ret);
		stm32_pcie_disable_link(pci);
		stm32_pcie_disable_resources(stm32_pcie);
		pm_runtime_put_sync(dev);
		return;
	}

	dw_pcie_ep_init_notify(ep);

	stm32_pcie->link_status = STM32_PCIE_EP_LINK_ENABLED;
}

static irqreturn_t stm32_pcie_ep_perst_irq_thread(int irq, void *data)
{
	struct stm32_pcie *stm32_pcie = data;
	struct dw_pcie *pci = stm32_pcie->pci;
	u32 perst;

	perst = gpiod_get_value(stm32_pcie->reset_gpio);
	if (perst)
		stm32_pcie_perst_assert(pci);
	else
		stm32_pcie_perst_deassert(pci);

	return IRQ_HANDLED;
}

static int stm32_add_pcie_ep(struct stm32_pcie *stm32_pcie,
			     struct platform_device *pdev)
{
	struct dw_pcie *pci = stm32_pcie->pci;
	struct dw_pcie_ep *ep = &pci->ep;
	struct device *dev = &pdev->dev;
	int ret;

	ret = regmap_update_bits(stm32_pcie->regmap, SYSCFG_PCIECR,
				 STM32MP25_PCIECR_TYPE_MASK,
				 STM32MP25_PCIECR_EP);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		dev_err(dev, "pm runtime resume failed: %d\n", ret);
		return ret;
	}

	ret = reset_control_reset(stm32_pcie->rst);
	if (ret) {
		dev_err(dev, "reset_control_reset failed %d\n", ret);
		pm_runtime_put_sync(dev);
		return ret;
	}

	ep->ops = &stm32_pcie_ep_ops;

	ret = stm32_pcie_enable_resources(stm32_pcie);
	if (ret) {
		dev_err(dev, "failed to enable resources: %d\n", ret);
		pm_runtime_put_sync(dev);
		return ret;
	}

	ret = dw_pcie_ep_init(ep);
	if (ret) {
		dev_err(dev, "failed to initialize ep: %d\n", ret);
		stm32_pcie_disable_resources(stm32_pcie);
		pm_runtime_put_sync(dev);
		return ret;
	}

	return 0;
}

static int stm32_pcie_probe(struct platform_device *pdev)
{
	struct stm32_pcie *stm32_pcie;
	struct dw_pcie *dw;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	stm32_pcie = devm_kzalloc(dev, sizeof(*stm32_pcie), GFP_KERNEL);
	if (!stm32_pcie)
		return -ENOMEM;

	dw = devm_kzalloc(dev, sizeof(*dw), GFP_KERNEL);
	if (!dw)
		return -ENOMEM;
	stm32_pcie->pci = dw;

	dw->dev = dev;
	dw->ops = &dw_pcie_ops;

	/* regmap registers for PCIe IP configuration */
	stm32_pcie->regmap = syscon_regmap_lookup_by_phandle(np, "st,syscfg");
	if (IS_ERR(stm32_pcie->regmap))
		return dev_err_probe(dev, PTR_ERR(stm32_pcie->regmap),
				     "No syscfg phandle specified\n");

	stm32_pcie->phy = devm_phy_get(dev, "pcie-phy");
	if (IS_ERR(stm32_pcie->phy))
		return dev_err_probe(dev, PTR_ERR(stm32_pcie->phy),
				     "failed to get pcie-phy\n");

	stm32_pcie->clk = devm_clk_get(dev, "core");
	if (IS_ERR(stm32_pcie->clk))
		return dev_err_probe(dev, PTR_ERR(stm32_pcie->clk),
				     "Failed to get PCIe clock source\n");

	stm32_pcie->rst = devm_reset_control_get(dev, "pcie");
	if (IS_ERR(stm32_pcie->rst))
		return dev_err_probe(dev, PTR_ERR(stm32_pcie->rst),
				     "Failed to get PCIe reset\n");

	stm32_pcie->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_IN);
	if (IS_ERR(stm32_pcie->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(stm32_pcie->reset_gpio),
				     "Failed to get reset GPIO\n");

	ret = phy_set_mode(stm32_pcie->phy, PHY_MODE_PCIE);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, stm32_pcie);

	ret = devm_pm_runtime_enable(dev);
	if (ret < 0) {
		dev_err(dev, "Failed to enable pm runtime %d\n", ret);
		return ret;
	}

	stm32_pcie->perst_irq = gpiod_to_irq(stm32_pcie->reset_gpio);

	/* Will be enabled in start_link when device is initialized. */
	irq_set_status_flags(stm32_pcie->perst_irq, IRQ_NOAUTOEN);

	ret = devm_request_threaded_irq(dev, stm32_pcie->perst_irq, NULL,
					stm32_pcie_ep_perst_irq_thread,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"perst_irq", stm32_pcie);
	if (ret) {
		dev_err(dev, "Failed to request PERST IRQ: %d\n", ret);
		return ret;
	}

	return stm32_add_pcie_ep(stm32_pcie, pdev);
}

static int stm32_pcie_remove(struct platform_device *pdev)
{
	struct stm32_pcie *stm32_pcie = platform_get_drvdata(pdev);
	struct dw_pcie_ep *ep = &stm32_pcie->pci->ep;

	disable_irq(stm32_pcie->perst_irq);

	dw_pcie_ep_exit(ep);

	stm32_pcie_disable_resources(stm32_pcie);

	pm_runtime_put_sync(&pdev->dev);

	return 0;
}

static struct platform_driver stm32_pcie_ep_driver = {
	.probe = stm32_pcie_probe,
	.remove	= stm32_pcie_remove,
	.driver = {
		.name = "stm32-ep-pcie",
		.of_match_table = stm32_pcie_ep_of_match,
	},
};

module_platform_driver(stm32_pcie_ep_driver);
MODULE_DESCRIPTION("STM32MP25 PCIe Endpoint Controller driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, stm32_pcie_ep_of_match);
