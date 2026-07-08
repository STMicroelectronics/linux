// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics STM32MP25 PCIe root complex driver.
 *
 * Copyright (C) 2025 STMicroelectronics
 * Author: Christian Bruel <christian.bruel@foss.st.com>
 */

#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeirq.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include "pcie-designware.h"
#include "pcie-stm32.h"
#include "../../pci.h"
#include "../../pcie/portdrv.h"

struct stm32_pcie {
	struct dw_pcie pci;
	struct regmap *regmap;
	struct reset_control *rst;
	struct phy *phy;
	struct clk *clk;
	struct gpio_desc *perst_gpio;
	struct gpio_desc *wake_gpio;
	int aer_irq;
	int pme_irq;
};

static void stm32_pcie_deassert_perst(struct stm32_pcie *stm32_pcie)
{
	if (stm32_pcie->perst_gpio) {
		msleep(PCIE_T_PVPERL_MS);
		gpiod_set_value(stm32_pcie->perst_gpio, 0);
	}

	msleep(PCIE_RESET_CONFIG_WAIT_MS);
}

static void stm32_pcie_assert_perst(struct stm32_pcie *stm32_pcie)
{
	gpiod_set_value(stm32_pcie->perst_gpio, 1);
}

static int stm32_pcie_start_link(struct dw_pcie *pci)
{
	struct stm32_pcie *stm32_pcie = to_stm32_pcie(pci);

	return regmap_update_bits(stm32_pcie->regmap, SYSCFG_PCIECR,
				  STM32MP25_PCIECR_LTSSM_EN,
				  STM32MP25_PCIECR_LTSSM_EN);
}

static void stm32_pcie_stop_link(struct dw_pcie *pci)
{
	struct stm32_pcie *stm32_pcie = to_stm32_pcie(pci);

	regmap_update_bits(stm32_pcie->regmap, SYSCFG_PCIECR,
			   STM32MP25_PCIECR_LTSSM_EN, 0);
}

static int stm32_pcie_suspend_noirq(struct device *dev)
{
	struct stm32_pcie *stm32_pcie = dev_get_drvdata(dev);
	int ret;

	ret = dw_pcie_suspend_noirq(&stm32_pcie->pci);
	if (ret)
		return ret;

	stm32_pcie_assert_perst(stm32_pcie);

	clk_disable_unprepare(stm32_pcie->clk);

	if (!device_wakeup_path(dev))
		phy_exit(stm32_pcie->phy);

	return pinctrl_pm_select_sleep_state(dev);
}

static int stm32_pcie_resume_noirq(struct device *dev)
{
	struct stm32_pcie *stm32_pcie = dev_get_drvdata(dev);
	int ret;

	/*
	 * The core clock is gated with CLKREQ# from the COMBOPHY REFCLK,
	 * thus if no device is present, must deassert it with a GPIO from
	 * pinctrl pinmux before accessing the DBI registers.
	 */
	ret = pinctrl_pm_select_init_state(dev);
	if (ret) {
		dev_err(dev, "Failed to activate pinctrl pm state: %d\n", ret);
		return ret;
	}

	if (!device_wakeup_path(dev)) {
		ret = phy_init(stm32_pcie->phy);
		if (ret) {
			pinctrl_pm_select_default_state(dev);
			return ret;
		}
	}

	ret = clk_prepare_enable(stm32_pcie->clk);
	if (ret)
		goto err_phy_exit;

	stm32_pcie_deassert_perst(stm32_pcie);

	ret = dw_pcie_resume_noirq(&stm32_pcie->pci);
	/*
	 * Ignore timeout, without link we still need the controller clock
	 * to access dbi registers.
	 */
	if (ret && ret != -ETIMEDOUT)
		goto err_disable_clk;

	pinctrl_pm_select_default_state(dev);

	return 0;

err_disable_clk:
	stm32_pcie_assert_perst(stm32_pcie);
	clk_disable_unprepare(stm32_pcie->clk);

err_phy_exit:
	phy_exit(stm32_pcie->phy);
	pinctrl_pm_select_default_state(dev);

	return ret;
}

static const struct dev_pm_ops stm32_pcie_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(stm32_pcie_suspend_noirq,
				  stm32_pcie_resume_noirq)
};

static const struct dw_pcie_host_ops stm32_pcie_host_ops = {
};

static const struct dw_pcie_ops dw_pcie_ops = {
	.start_link = stm32_pcie_start_link,
	.stop_link = stm32_pcie_stop_link
};

/*
 * PME/AER platform irq in this driver only re-initialises the platform IRQ glue logic.
 * The handling is done by the generic pci drivers (trough platform IRQ hook), through shared IRQ
 */
static irqreturn_t stm32_pcie_pme_msi_irq_handler(int irq, void *priv)
{
	struct stm32_pcie *stm32_pcie = priv;

	regmap_write(stm32_pcie->regmap, SYSCFG_PCIEPMEMSICR, 1);
	regmap_write(stm32_pcie->regmap, SYSCFG_PCIEPMEMSICR, 0);

	return IRQ_HANDLED;
}

static irqreturn_t stm32_pcie_aer_msi_irq_handler(int irq, void *priv)
{
	struct stm32_pcie *stm32_pcie = priv;

	regmap_write(stm32_pcie->regmap, SYSCFG_PCIEAERRCMSICR, 1);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t stm32_pcie_aer_msi_isr_handler(int irq, void *priv)
{
	struct stm32_pcie *stm32_pcie = priv;

	regmap_write(stm32_pcie->regmap, SYSCFG_PCIEAERRCMSICR, 0);

	return IRQ_HANDLED;
}

static int stm32_pcie_port_irqs(struct pci_dev *pci_dev, u32 *pme, u32 *aer, u32 *dpc)
{
	struct dw_pcie_rp *pp = pci_dev->bus->sysdata;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct stm32_pcie *stm32_pcie = to_stm32_pcie(pci);
	int nirqs = 0;

	if (aer && stm32_pcie->aer_irq)
		nirqs++;
	if (pme && stm32_pcie->pme_irq)
		nirqs++;

	nirqs = pci_alloc_irq_vectors(pci_dev, nirqs, nirqs, PCI_IRQ_MSI);
	if (nirqs < 0)
		return nirqs;

	if (aer && stm32_pcie->aer_irq)
		*aer = stm32_pcie->aer_irq;
	if (pme && stm32_pcie->pme_irq)
		*pme = stm32_pcie->pme_irq;

	return 0;
}

static int stm32_add_pcie_port(struct stm32_pcie *stm32_pcie, struct platform_device *pdev)
{
	struct device *dev = stm32_pcie->pci.dev;
	unsigned int wake_irq;
	int ret;

	/* Use generic AER driver if we can register the port_drv */
	if (pci_aer_available()) {
		stm32_pcie->aer_irq = platform_get_irq_byname_optional(pdev, "aer");
		if (stm32_pcie->aer_irq < 0) {
			if (stm32_pcie->aer_irq != -ENXIO)
				return dev_err_probe(dev, stm32_pcie->aer_irq,
						     "failed to get AER IRQ\n");
			stm32_pcie->aer_irq = 0;
		}

		if (stm32_pcie->aer_irq) {
			ret = devm_request_threaded_irq(dev, stm32_pcie->aer_irq,
							stm32_pcie_aer_msi_irq_handler,
							stm32_pcie_aer_msi_isr_handler,
							IRQF_SHARED, "stm32-aer-msi", stm32_pcie);
			if (ret < 0)
				return dev_err_probe(dev, stm32_pcie->aer_irq,
						     "failed to request AER MSI IRQ\n");
		}
	}

	stm32_pcie->pme_irq = platform_get_irq_byname_optional(pdev, "pme");
	if (stm32_pcie->pme_irq < 0) {
		if (stm32_pcie->pme_irq != -ENXIO)
			return dev_err_probe(dev, stm32_pcie->pme_irq,
					     "failed to get PME MSI IRQ\n");
		stm32_pcie->pme_irq = 0;
	}

	if (stm32_pcie->pme_irq) {
		ret = devm_request_irq(dev, stm32_pcie->pme_irq,
				       stm32_pcie_pme_msi_irq_handler,
				       IRQF_SHARED, "stm32-pme-msi", stm32_pcie);
		if (ret < 0)
			return dev_err_probe(dev, stm32_pcie->pme_irq,
					     "failed to request PME MSI IRQ\n");
	}

	/* Register AER/PME pordrv hook */
	if (stm32_pcie->pme_irq || stm32_pcie->aer_irq)
		pcie_port_set_irqs_hook(stm32_pcie_port_irqs);

	ret = phy_set_mode(stm32_pcie->phy, PHY_MODE_PCIE);
	if (ret)
		return ret;

	ret = phy_init(stm32_pcie->phy);
	if (ret)
		return ret;

	ret = regmap_update_bits(stm32_pcie->regmap, SYSCFG_PCIECR,
				 STM32MP25_PCIECR_TYPE_MASK,
				 STM32MP25_PCIECR_RC);
	if (ret)
		goto err_phy_exit;

	stm32_pcie_deassert_perst(stm32_pcie);

	if (stm32_pcie->wake_gpio) {
		wake_irq = gpiod_to_irq(stm32_pcie->wake_gpio);
		ret = dev_pm_set_dedicated_wake_irq(dev, wake_irq);
		if (ret) {
			dev_err(dev, "Failed to enable wakeup irq %d\n", ret);
			goto err_assert_perst;
		}
		irq_set_irq_type(wake_irq, IRQ_TYPE_EDGE_FALLING);
	}

	return 0;

err_assert_perst:
	stm32_pcie_assert_perst(stm32_pcie);

err_phy_exit:
	phy_exit(stm32_pcie->phy);

	return ret;
}

static void stm32_remove_pcie_port(struct stm32_pcie *stm32_pcie)
{
	dev_pm_clear_wake_irq(stm32_pcie->pci.dev);

	stm32_pcie_assert_perst(stm32_pcie);

	pcie_port_set_irqs_hook(NULL);

	phy_exit(stm32_pcie->phy);
}

static int stm32_pcie_parse_port(struct stm32_pcie *stm32_pcie)
{
	struct device *dev = stm32_pcie->pci.dev;
	struct device_node *root_port;

	root_port = of_get_next_available_child(dev->of_node, NULL);

	stm32_pcie->phy = devm_of_phy_get(dev, root_port, NULL);
	if (IS_ERR(stm32_pcie->phy)) {
		of_node_put(root_port);
		return dev_err_probe(dev, PTR_ERR(stm32_pcie->phy),
				     "Failed to get pcie-phy\n");
	}

	stm32_pcie->perst_gpio = devm_fwnode_gpiod_get(dev, of_fwnode_handle(root_port),
						       "reset", GPIOD_OUT_HIGH, NULL);
	if (IS_ERR(stm32_pcie->perst_gpio)) {
		if (PTR_ERR(stm32_pcie->perst_gpio) != -ENOENT) {
			of_node_put(root_port);
			return dev_err_probe(dev, PTR_ERR(stm32_pcie->perst_gpio),
					     "Failed to get reset GPIO\n");
		}
		stm32_pcie->perst_gpio = NULL;
	}

	stm32_pcie->wake_gpio = devm_fwnode_gpiod_get(dev, of_fwnode_handle(root_port),
						      "wake", GPIOD_IN, NULL);

	if (IS_ERR(stm32_pcie->wake_gpio)) {
		if (PTR_ERR(stm32_pcie->wake_gpio) != -ENOENT) {
			of_node_put(root_port);
			return dev_err_probe(dev, PTR_ERR(stm32_pcie->wake_gpio),
					     "Failed to get wake GPIO\n");
		}
		stm32_pcie->wake_gpio = NULL;
	}

	of_node_put(root_port);

	return 0;
}

static int stm32_pcie_probe(struct platform_device *pdev)
{
	struct stm32_pcie *stm32_pcie;
	struct device *dev = &pdev->dev;
	int ret;

	stm32_pcie = devm_kzalloc(dev, sizeof(*stm32_pcie), GFP_KERNEL);
	if (!stm32_pcie)
		return -ENOMEM;

	stm32_pcie->pci.dev = dev;
	stm32_pcie->pci.ops = &dw_pcie_ops;
	stm32_pcie->pci.pp.ops = &stm32_pcie_host_ops;

	stm32_pcie->regmap = syscon_regmap_lookup_by_compatible("st,stm32mp25-syscfg");
	if (IS_ERR(stm32_pcie->regmap))
		return dev_err_probe(dev, PTR_ERR(stm32_pcie->regmap),
				     "No syscfg specified\n");

	stm32_pcie->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(stm32_pcie->clk))
		return dev_err_probe(dev, PTR_ERR(stm32_pcie->clk),
				     "Failed to get PCIe clock source\n");

	stm32_pcie->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(stm32_pcie->rst))
		return dev_err_probe(dev, PTR_ERR(stm32_pcie->rst),
				     "Failed to get PCIe reset\n");

	ret = stm32_pcie_parse_port(stm32_pcie);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, stm32_pcie);

	ret = stm32_add_pcie_port(stm32_pcie, pdev);
	if (ret)
		return ret;

	reset_control_assert(stm32_pcie->rst);
	reset_control_deassert(stm32_pcie->rst);

	ret = clk_prepare_enable(stm32_pcie->clk);
	if (ret) {
		dev_err(dev, "Core clock enable failed %d\n", ret);
		goto err_remove_port;
	}

	ret = pm_runtime_set_active(dev);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to activate runtime PM\n");
		goto err_disable_clk;
	}

	pm_runtime_no_callbacks(dev);

	ret = devm_pm_runtime_enable(dev);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to enable runtime PM\n");
		goto err_disable_clk;
	}

	stm32_pcie->pci.pp.use_atu_msg = true;
	stm32_pcie->pci.pp.skip_l23_ready = true;

	ret = dw_pcie_host_init(&stm32_pcie->pci.pp);
	if (ret)
		goto err_disable_clk;

	if (stm32_pcie->wake_gpio)
		device_set_wakeup_capable(dev, true);

	return 0;

err_disable_clk:
	clk_disable_unprepare(stm32_pcie->clk);

err_remove_port:
	stm32_remove_pcie_port(stm32_pcie);

	return ret;
}

static void stm32_pcie_remove(struct platform_device *pdev)
{
	struct stm32_pcie *stm32_pcie = platform_get_drvdata(pdev);
	struct dw_pcie_rp *pp = &stm32_pcie->pci.pp;

	if (stm32_pcie->wake_gpio)
		device_set_wakeup_capable(&pdev->dev, false);

	dw_pcie_host_deinit(pp);

	clk_disable_unprepare(stm32_pcie->clk);

	stm32_remove_pcie_port(stm32_pcie);

	pm_runtime_put_noidle(&pdev->dev);
}

static const struct of_device_id stm32_pcie_of_match[] = {
	{ .compatible = "st,stm32mp25-pcie-rc" },
	{},
};

static struct platform_driver stm32_pcie_driver = {
	.probe = stm32_pcie_probe,
	.remove = stm32_pcie_remove,
	.driver = {
		.name = "stm32-pcie",
		.of_match_table = stm32_pcie_of_match,
		.pm = &stm32_pcie_pm_ops,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

module_platform_driver(stm32_pcie_driver);

MODULE_AUTHOR("Christian Bruel <christian.bruel@foss.st.com>");
MODULE_DESCRIPTION("STM32MP25 PCIe Controller driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, stm32_pcie_of_match);
