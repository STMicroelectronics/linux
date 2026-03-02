// SPDX-License-Identifier: GPL-2.0-only
/*
 * Implements PM domain using the generic PM domain for STM32MP SoC.
 *
 * Copyright (C) STMicroelectronics 2024-2025
 * Author(s): Patrick Delaunay <patrick.delaunay@foss.st.com>
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/slab.h>

static const struct of_device_id stm32mp_pm_domain_matches[] = {
	{ .compatible = "st,stm32mp-pm-domain", },
	{ },
};

static void stm32mp_pm_domain_genpd_remove(void *data)
{
	pm_genpd_remove(data);
}

static int stm32mp_pm_domain_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct generic_pm_domain *pd;
	int ret;

	if (!np)
		return -ENODEV;

	if (!of_property_present(np, "#power-domain-cells")) {
		pr_warn("%pOF lacks #power-domain-cells\n", np);
		return 0;
	}

	pd = kzalloc(sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->name = dev_name(dev);

	/*
	 * Domain based only on PM clk framework, no GENPD_FLAG_ACTIVE_WAKEUP
	 * support, the wake-up is handled by power domain of interrupt driver
	 */
	pd->flags = GENPD_FLAG_PM_CLK | GENPD_FLAG_IRQ_SAFE;

	ret = pm_genpd_init(pd, NULL, false);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, stm32mp_pm_domain_genpd_remove, pd);
	if (ret)
		return ret;

	ret = of_genpd_add_provider_simple(np, pd);
	if (ret)
		return ret;

	ret = pm_genpd_add_subdomain(pd_to_genpd(dev->pm_domain), pd);
	if (ret)
		return ret;

	pm_runtime_enable(dev);

	return 0;
}

static void stm32mp_pm_domain_remove(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;

	of_genpd_del_provider(np);
}

static struct platform_driver stm32mp_pm_domain_driver = {
	.driver = {
		.name = "stm32mp_pm_domain",
		.of_match_table = stm32mp_pm_domain_matches,
	},
	.probe  = stm32mp_pm_domain_probe,
	.remove  = stm32mp_pm_domain_remove,
};

module_platform_driver(stm32mp_pm_domain_driver);

MODULE_AUTHOR("Patrick Delaunay <patrick.delaunay@foss.st.com>");
MODULE_DESCRIPTION("ST STM32MP power domain driver");
MODULE_LICENSE("GPL");
