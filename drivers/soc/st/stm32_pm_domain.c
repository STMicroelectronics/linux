// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2018 - All Rights Reserved
 * Author: Alexandre Torgue <alexandre.torgue@st.com> for STMicroelectronics.
 * Author: Olivier Bideau <olivier.bideau@st.com> for STMicroelectronics.
 */

#include <linux/arm-smccc.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>

#define SMC(domain, state)				\
{							\
	struct arm_smccc_res res;			\
	arm_smccc_smc(0x82001008, domain, state, 0,	\
		      0, 0, 0, 0, &res);		\
}

#define STM32_SMC_PD_DOMAIN_ON	0
#define STM32_SMC_PD_DOMAIN_OFF	1

struct stm32_pm_domain {
	struct device *dev;
	struct generic_pm_domain genpd;
	int id;
};

static int stm32_pd_power_off(struct generic_pm_domain *domain)
{
	struct stm32_pm_domain *priv = container_of(domain,
						    struct stm32_pm_domain,
						    genpd);

	SMC(priv->id, STM32_SMC_PD_DOMAIN_OFF);

	dev_dbg(priv->dev, "%s OFF\n", domain->name);

	return 0;
}

static int stm32_pd_power_on(struct generic_pm_domain *domain)
{
	struct stm32_pm_domain *priv = container_of(domain,
						    struct stm32_pm_domain,
						    genpd);

	SMC(priv->id, STM32_SMC_PD_DOMAIN_ON);

	dev_dbg(priv->dev, "%s ON\n", domain->name);

	return 0;
}

static void stm32_pm_domain_remove(struct stm32_pm_domain *domain)
{
	int ret;

	ret = pm_genpd_remove(&domain->genpd);
	if (ret)
		dev_err(domain->dev, "failed to remove PM domain %s: %d\n",
			domain->genpd.name, ret);
}

static int stm32_pm_domain_add(struct stm32_pm_domain *domain,
			       struct device *dev,
			       struct device_node *np)
{
	int ret;

	domain->dev = dev;
	domain->genpd.name = np->name;
	domain->genpd.power_off = stm32_pd_power_off;
	domain->genpd.power_on = stm32_pd_power_on;
	domain->genpd.flags |= GENPD_FLAG_ACTIVE_WAKEUP;

	ret = of_property_read_u32(np, "reg", &domain->id);
	if (ret) {
		dev_err(domain->dev, "no domain ID\n");
		return ret;
	}

	ret = pm_genpd_init(&domain->genpd, NULL, 0);
	if (ret < 0) {
		dev_err(domain->dev, "failed to initialise PM domain %s: %d\n",
			np->name, ret);
		return ret;
	}

	ret = of_genpd_add_provider_simple(np, &domain->genpd);
	if (ret < 0) {
		dev_err(domain->dev, "failed to register PM domain %s: %d\n",
			np->name, ret);
		stm32_pm_domain_remove(domain);
		return ret;
	}

	dev_info(domain->dev, "domain %s registered\n", np->name);

	return 0;
}

static void stm32_pm_subdomain_add(struct stm32_pm_domain *domain,
				   struct device *dev,
				   struct device_node *np)
{
	struct device_node *np_child;
	int ret;

	for_each_child_of_node(np, np_child) {
		struct stm32_pm_domain *sub_domain;

		sub_domain = devm_kzalloc(dev, sizeof(*sub_domain), GFP_KERNEL);
		if (!sub_domain)
			continue;

		sub_domain->dev = dev;
		sub_domain->genpd.name = np_child->name;
		sub_domain->genpd.power_off = stm32_pd_power_off;
		sub_domain->genpd.power_on = stm32_pd_power_on;
		sub_domain->genpd.flags |= GENPD_FLAG_ACTIVE_WAKEUP;

		ret = of_property_read_u32(np_child, "reg", &sub_domain->id);
		if (ret) {
			dev_err(sub_domain->dev, "no domain ID\n");
			devm_kfree(dev, sub_domain);
			continue;
		}

		ret = pm_genpd_init(&sub_domain->genpd, NULL, 0);
		if (ret < 0) {
			dev_err(sub_domain->dev, "failed to initialise PM domain %s: %d\n"
				, np_child->name, ret);
			devm_kfree(dev, sub_domain);
			continue;
		}

		ret = of_genpd_add_provider_simple(np_child,
						   &sub_domain->genpd);
		if (ret < 0) {
			dev_err(sub_domain->dev, "failed to register PM domain %s: %d\n"
				, np_child->name, ret);
			stm32_pm_domain_remove(sub_domain);
			devm_kfree(dev, sub_domain);
			continue;
		}

		ret = pm_genpd_add_subdomain(&domain->genpd,
					     &sub_domain->genpd);

		if (ret < 0) {
			dev_err(sub_domain->dev, "failed to add Sub PM domain %s: %d\n"
				, np_child->name, ret);
			stm32_pm_domain_remove(sub_domain);
			devm_kfree(dev, sub_domain);
			continue;
		}

		dev_info(sub_domain->dev, "subdomain %s registered\n",
			 np_child->name);
	}
}

static int stm32_pm_domain_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *child_np;
	int ret;

	for_each_child_of_node(np, child_np) {
		struct stm32_pm_domain *domain;

		domain = devm_kzalloc(dev, sizeof(*domain), GFP_KERNEL);
		if (!domain)
			continue;

		ret = stm32_pm_domain_add(domain, dev, child_np);
		if (ret) {
			devm_kfree(dev, domain);
			continue;
		}

		stm32_pm_subdomain_add(domain, dev, child_np);
	}

	dev_info(dev, "domains probed\n");

	return 0;
}

static const struct of_device_id stm32_pm_domain_matches[] = {
	{ .compatible = "st,stm32mp157c-pd", },
	{ },
};

static struct platform_driver stm32_pm_domains_driver = {
	.probe = stm32_pm_domain_probe,
	.driver = {
		.name   = "stm32-pm-domain",
		.of_match_table = stm32_pm_domain_matches,
	},
};

static int __init stm32_pm_domains_init(void)
{
	return platform_driver_register(&stm32_pm_domains_driver);
}
core_initcall(stm32_pm_domains_init);
