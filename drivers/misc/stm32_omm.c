// SPDX-License-Identifier: GPL
/*
 * Copyright (C) STMicroelectronics 2022 - All Rights Reserved
 * Author(s): Patrice Chotard <patrice.chotard@foss.st.com> for STMicroelectronics.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <memory/stm32-omi.h>

/* Temporary */
#include "../bus/stm32_sys_bus.h"

#define OMM_CR			0
#define CR_MUXEN		BIT(0)
#define CR_MUXENMODE_MASK	GENMASK(1, 0)
#define CR_CSSEL_OVR_EN		BIT(4)
#define CR_CSSEL_OVR_MASK	GENMASK(6, 5)
#define CR_REQ2ACK_MASK		GENMASK(23, 16)

#define OMM_CHILD_NB		2

struct stm32_omm {
	struct device_node *child_node[OMM_CHILD_NB];
	struct device *child_dev[OMM_CHILD_NB];
	struct clk *child_clk[OMM_CHILD_NB];
	struct resource *mm_res;
	struct clk *clk;
	void __iomem *io_base;
	u32 cr;
	u8 nb_child;
	bool restore_omm;
};

static int stm32_omm_set_amcr(struct device *dev, bool set)
{
	struct stm32_omm *omm = dev_get_drvdata(dev);
	struct regmap *syscfg_regmap;
	struct device_node *node;
	struct resource res, res1;
	resource_size_t mm_ospi2_size = 0;
	static const char * const mm_name[] = { "mm_ospi1", "mm_ospi2" };
	u32 amcr_base, amcr_mask;
	int ret, i, idx;
	unsigned int amcr, read_amcr;

	for (i = 0; i < omm->nb_child; i++) {
		idx = of_property_match_string(dev->of_node,
					       "memory-region-names",
					       mm_name[i]);
		if (idx < 0)
			continue;

		/* res1 only used on second loop iteration */
		res1.start = res.start;
		res1.end = res.end;

		node = of_parse_phandle(dev->of_node, "memory-region", idx);
		ret = of_address_to_resource(node, 0, &res);
		if (ret) {
			dev_err(dev, "unable to resolve memory region\n");
			return ret;
		}

		/* check that memory region fits inside OMM memory map area */
		if (!resource_contains(omm->mm_res, &res)) {
			dev_err(dev, "%s doesn't fit inside OMM memory map area\n",
				mm_name[i]);
			dev_err(dev, "[0x%llx-0x%llx] doesn't fit inside [0x%llx-0x%llx]\n",
				res.start, res.end,
				omm->mm_res->start, omm->mm_res->end);

			return -EFAULT;
		}

		if (i == 1) {
			mm_ospi2_size = resource_size(&res);

			/* check that OMM memory region 1 doesn't overlap memory region 2 */
			if (resource_overlaps(&res, &res1)) {
				dev_err(dev, "OMM memory-region %s overlaps memory region %s\n",
					mm_name[0], mm_name[1]);
				dev_err(dev, "[0x%llx-0x%llx] overlaps [0x%llx-0x%llx]\n",
					res1.start, res1.end, res.start, res.end);

				return -EFAULT;
			}
		}
	}

	syscfg_regmap = syscon_regmap_lookup_by_phandle(dev->of_node, "st,syscfg-amcr");
	if (IS_ERR(syscfg_regmap)) {
		dev_err(dev, "Failed to get st,syscfg-amcr property\n");
		return PTR_ERR(syscfg_regmap);
	}

	ret = of_property_read_u32_index(dev->of_node, "st,syscfg-amcr", 1,
					 &amcr_base);
	if (ret)
		return ret;

	ret = of_property_read_u32_index(dev->of_node, "st,syscfg-amcr", 2,
					 &amcr_mask);
	if (ret)
		return ret;

	amcr = mm_ospi2_size / SZ_64M;

	if (set)
		regmap_update_bits(syscfg_regmap, amcr_base, amcr_mask, amcr);

	/* read AMCR and check coherency with memory-map areas defined in DT */
	regmap_read(syscfg_regmap, amcr_base, &read_amcr);
	read_amcr = read_amcr >> (ffs(amcr_mask) - 1);

	if (amcr != read_amcr) {
		dev_err(dev, "AMCR value not coherent with DT memory-map areas\n");
		ret = -EINVAL;
	}

	return ret;
}

static int stm32_omm_enable_child_clock(struct device *dev, bool enable)
{
	/* As there is only 2 children, remember first child in case of error */
	struct clk *first_child_clk = NULL;
	struct stm32_omm *omm = dev_get_drvdata(dev);
	u8 i;
	int ret;

	for (i = 0; i < omm->nb_child; i++) {
		if (enable) {
			ret = clk_prepare_enable(omm->child_clk[i]);
			if (ret) {
				if (first_child_clk)
					clk_disable_unprepare(first_child_clk);

				dev_err(dev, "Can not enable clock\n");
				return ret;
			}
		} else {
			clk_disable_unprepare(omm->child_clk[i]);
		}

		first_child_clk = omm->child_clk[i];
	}

	return 0;
}

static int stm32_omm_configure(struct device *dev)
{
	struct stm32_omm *omm = dev_get_drvdata(dev);
	struct reset_control *rstc;
	unsigned long clk_rate, clk_rate_max = 0;
	int ret;
	u8 i;
	u32 mux = 0;
	u32 cssel_ovr = 0;
	u32 req2ack = 0;

	omm->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(omm->clk)) {
		dev_err(dev, "Failed to get OMM clock (%ld)\n",
			PTR_ERR(omm->clk));

		return PTR_ERR(omm->clk);
	}

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	/* parse children's clock */
	for (i = 0; i < omm->nb_child; i++) {
		clk_rate = clk_get_rate(omm->child_clk[i]);
		if (!clk_rate) {
			dev_err(dev, "Invalid clock rate\n");
			goto err_clk_disable;
		}

		if (clk_rate > clk_rate_max)
			clk_rate_max = clk_rate;
	}

	rstc = devm_reset_control_get_optional(dev, NULL);
	if (IS_ERR(rstc)) {
		ret = dev_err_probe(dev, PTR_ERR(rstc), "reset get failed\n");
		goto err_clk_disable;
	}

	reset_control_assert(rstc);
	udelay(2);
	reset_control_deassert(rstc);

	omm->cr = readl_relaxed(omm->io_base + OMM_CR);
	/* optional */
	ret = of_property_read_u32(dev->of_node, "st,omm-mux", &mux);
	if (!ret) {
		if (mux & CR_MUXEN) {
			ret = of_property_read_u32(dev->of_node, "st,omm-req2ack-ns",
						  &req2ack);
			if (!ret && !req2ack) {
				req2ack = DIV_ROUND_UP(req2ack, NSEC_PER_SEC / clk_rate_max) - 1;

				if (req2ack > 256)
					req2ack = 256;
			}

			req2ack = FIELD_PREP(CR_REQ2ACK_MASK, req2ack);

			omm->cr &= ~CR_REQ2ACK_MASK;
			omm->cr |= FIELD_PREP(CR_REQ2ACK_MASK, req2ack);

			/*
			 * If the mux is enabled, the 2 OSPI clocks have to be
			 * always enabled
			 */
			ret = stm32_omm_enable_child_clock(dev, true);
			if (ret)
				goto err_clk_disable;
		}

		omm->cr &= ~CR_MUXENMODE_MASK;
		omm->cr |= FIELD_PREP(CR_MUXENMODE_MASK, mux);
	}

	/* optional */
	ret = of_property_read_u32(dev->of_node, "st,omm-cssel-ovr", &cssel_ovr);
	if (!ret) {
		omm->cr &= ~CR_CSSEL_OVR_MASK;
		omm->cr |= FIELD_PREP(CR_CSSEL_OVR_MASK, cssel_ovr);
		omm->cr |= CR_CSSEL_OVR_EN;
	}

	omm->restore_omm = true;
	writel_relaxed(omm->cr, omm->io_base + OMM_CR);

	ret = stm32_omm_set_amcr(dev, true);

err_clk_disable:
	pm_runtime_put_sync_suspend(dev);

	return ret;
}

static int stm32_omm_check_access(struct device *dev, struct device_node *np)
{
	int err;
	u32 feature_domain_cell[2];
	u32 id_bus;

	/* Get reg from device node */
	err = of_property_read_u32_array(np, "feature-domains", feature_domain_cell, 2);
	if (err) {
		dev_err(dev, "Unable to find feature-domains property\n");
		return -ENODEV;
	}

	id_bus = feature_domain_cell[1];

	return stm32_rifsc_get_access_by_id(id_bus);
}

static int stm32_omm_disable_child(struct device *dev)
{
	struct stm32_omm *omm = dev_get_drvdata(dev);
	struct regmap *omi_regmap;
	u8 i;
	int ret;

	for (i = 0; i < omm->nb_child; i++) {
		ret = clk_prepare_enable(omm->child_clk[i]);
		if (ret) {
			dev_err(dev, "Can not enable clock\n");
			return ret;
		}

		omi_regmap = device_node_to_regmap(omm->child_node[i]);
		if (IS_ERR(omi_regmap)) {
			dev_err(dev, "failed to get OMI regmap for %s (%ld)",
				omm->child_node[i]->full_name, PTR_ERR(omi_regmap));
			return PTR_ERR(omi_regmap);
		}

		regmap_clear_bits(omi_regmap, OSPI_CR, CR_EN);
		clk_disable_unprepare(omm->child_clk[i]);
	}

	return 0;
}

static int stm32_omm_probe(struct platform_device *pdev)
{
	struct platform_device *vdev;
	struct device *dev = &pdev->dev;
	struct device_node *child;
	struct stm32_omm *omm;
	int ret;
	u8 child_access_granted = 0;
	u8 i, j;
	bool child_access[OMM_CHILD_NB];

	omm = devm_kzalloc(dev, sizeof(*omm), GFP_KERNEL);
	if (!omm)
		return -ENOMEM;

	omm->io_base = devm_platform_ioremap_resource_byname(pdev, "omm");
	if (IS_ERR(omm->io_base))
		return PTR_ERR(omm->io_base);

	omm->mm_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "omm_mm");

	/* check child's access */
	for_each_child_of_node(dev->of_node, child) {
		if (omm->nb_child >= OMM_CHILD_NB) {
			dev_err(dev, "Bad DT, found too much children\n");
			of_node_put(child);
			ret = -E2BIG;
			goto err_clk_release;
		}

		if (!of_device_is_compatible(child, "st,stm32mp25-omi")) {
			ret = -EINVAL;
			of_node_put(child);
			goto err_clk_release;
		}

		ret = stm32_omm_check_access(dev, child);
		if (ret < 0 && ret != -EACCES) {
			of_node_put(child);
			goto err_clk_release;
		}

		child_access[omm->nb_child] = false;
		if (!ret) {
			child_access_granted++;
			child_access[omm->nb_child] = true;
		}

		omm->child_node[omm->nb_child] = child;
		omm->child_clk[omm->nb_child] = of_clk_get(child, 0);
		omm->nb_child++;
	}

	if (omm->nb_child != OMM_CHILD_NB) {
		ret = -EINVAL;
		goto err_clk_release;
	}

	platform_set_drvdata(pdev, omm);

	pm_runtime_enable(dev);

	/* check if OMM's resource access is granted */
	ret = stm32_omm_check_access(dev, dev->of_node);
	if (ret < 0 && ret != -EACCES)
		goto err_clk_release;

	if (!ret && child_access_granted == OMM_CHILD_NB) {
		/* Ensure both OSPI instance are disabled before configuring OMM */
		ret = stm32_omm_disable_child(dev);
		if (ret)
			goto err_clk_release;

		ret = stm32_omm_configure(dev);
		if (ret)
			goto err_clk_release;
	} else {
		dev_dbg(dev, "Octo Memory Manager resource's access not granted\n");
		/*
		 * AMCR can't be set, so check if current value is coherent
		 * with memory-map areas defined in DT
		 */
		ret = stm32_omm_set_amcr(dev, false);
		if (ret)
			goto err_clk_release;
	}

	/* for each child, if resource access is granted and status "okay", probe it */
	for (i = 0; i < omm->nb_child; i++) {
		if (!child_access[i] || !of_device_is_available(omm->child_node[i]))
			continue;

		vdev = of_platform_device_create(omm->child_node[i], NULL, NULL);
		if (!vdev) {
			dev_err(dev, "Failed to create Octo Memory Manager child\n");
			for (j = i; j > 0; --j) {
				if (omm->child_dev[j])
					of_platform_device_destroy(omm->child_dev[j], NULL);
			}

			ret = -EINVAL;
			goto err_clk_release;
		}
		omm->child_dev[i] = &vdev->dev;
	}

	return 0;

err_clk_release:
	for (i = 0; i < omm->nb_child; i++)
		clk_put(omm->child_clk[i]);

	return ret;
}

static int stm32_omm_remove(struct platform_device *pdev)
{
	struct stm32_omm *omm = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < omm->nb_child; i++)
		if (omm->child_dev[i])
			of_platform_device_destroy(omm->child_dev[i], NULL);

	if (omm->cr & CR_MUXEN)
		stm32_omm_enable_child_clock(&pdev->dev, false);

	for (i = 0; i < omm->nb_child; i++)
		clk_put(omm->child_clk[i]);

	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id stm32_omm_of_match[] = {
	{ .compatible = "st,stm32mp25-omm", },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_omm_of_match);

static int __maybe_unused stm32_omm_runtime_suspend(struct device *dev)
{
	struct stm32_omm *omm = dev_get_drvdata(dev);

	clk_disable_unprepare(omm->clk);

	return 0;
}

static int __maybe_unused stm32_omm_runtime_resume(struct device *dev)
{
	struct stm32_omm *omm = dev_get_drvdata(dev);

	return clk_prepare_enable(omm->clk);
}

static int __maybe_unused stm32_omm_suspend(struct device *dev)
{
	struct stm32_omm *omm = dev_get_drvdata(dev);

	if (omm->restore_omm && omm->cr & CR_MUXEN)
		stm32_omm_enable_child_clock(dev, false);

	return pinctrl_pm_select_sleep_state(dev);
}

static int __maybe_unused stm32_omm_resume(struct device *dev)
{
	struct stm32_omm *omm = dev_get_drvdata(dev);
	int ret;

	pinctrl_pm_select_default_state(dev);

	if (!omm->restore_omm)
		return 0;

	/* Ensure both OSPI instance are disabled before configuring OMM */
	ret = stm32_omm_disable_child(dev);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	writel_relaxed(omm->cr, omm->io_base + OMM_CR);
	ret = stm32_omm_set_amcr(dev, true);
	pm_runtime_put_sync_suspend(dev);
	if (ret)
		return ret;

	if (omm->cr & CR_MUXEN)
		ret = stm32_omm_enable_child_clock(dev, true);

	return ret;
}

static const struct dev_pm_ops stm32_omm_pm_ops = {
	SET_RUNTIME_PM_OPS(stm32_omm_runtime_suspend,
			   stm32_omm_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(stm32_omm_suspend, stm32_omm_resume)
};

static struct platform_driver stm32_omm_driver = {
	.probe	= stm32_omm_probe,
	.remove = stm32_omm_remove,
	.driver	= {
		.name = "stm32-omm",
		.of_match_table = stm32_omm_of_match,
		.pm = &stm32_omm_pm_ops,
	},
};
module_platform_driver(stm32_omm_driver);

MODULE_DESCRIPTION("STMicroelectronics Octo Memory Manager driver");
MODULE_LICENSE("GPL");
