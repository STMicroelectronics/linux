// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2018 - All Rights Reserved
 * Author: Fabien Dessenne <fabien.dessenne@st.com> for STMicroelectronics.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/component.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/remoteproc.h>

#include "rproc_srm_core.h"

struct rproc_srm_clk_info {
	struct list_head list;
	unsigned int index;
	struct clk *clk;
	const char *name;
	bool parent_enabled;
};

struct rproc_srm_regu_info {
	struct list_head list;
	unsigned int index;
	struct regulator *regu;
	const char *name;
	bool enabled;
};

struct rproc_srm_irq_info {
	struct list_head list;
	unsigned int index;
	char *name;
	int irq;
	bool enabled;
};

struct rproc_srm_dev {
	struct device *dev;
	struct rproc_srm_core *core;
	struct notifier_block nb;
	bool early_boot;

	struct list_head clk_list_head;
	struct list_head regu_list_head;
	struct list_head irq_list_head;
};

/* Irqs */
static void rproc_srm_dev_irqs_put(struct rproc_srm_dev *rproc_srm_dev)
{
	struct device *dev = rproc_srm_dev->dev;
	struct rproc_srm_irq_info *i, *tmp;

	list_for_each_entry_safe(i, tmp, &rproc_srm_dev->irq_list_head, list) {
		devm_free_irq(dev, i->irq, NULL);
		dev_dbg(dev, "Put irq %d (%s)\n", i->irq, i->name);
		list_del(&i->list);
	}
}

static irqreturn_t rproc_srm_dev_irq_handler(int irq, void *dev)
{
	dev_warn(dev, "Spurious interrupt\n");
	return IRQ_HANDLED;
}

static int rproc_srm_dev_irqs_get(struct rproc_srm_dev *rproc_srm_dev)
{
	struct device *dev = rproc_srm_dev->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *np = dev->of_node;
	struct rproc_srm_irq_info *info;
	const char *name;
	int nr, ret, irq;
	unsigned int i;

	if (!np)
		return 0;

	nr = platform_irq_count(pdev);
	if (!nr)
		return 0;

	if (rproc_srm_dev->early_boot)
		/*
		 * Do not overwrite the irq configuration.
		 * No need to parse irq from DT since the resource manager does
		 * not offer any service to update the irq config.
		 */
		return 0;

	for (i = 0; i < nr; i++) {
		info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
		if (!info) {
			ret = -ENOMEM;
			goto err;
		}

		irq = platform_get_irq(pdev, i);
		if (irq <= 0) {
			ret = irq;
			dev_err(dev, "Failed to get irq (%d)\n", ret);
			goto err;
		}

		info->irq = irq;

		/* Register a dummy irq handleras not used by Linux */
		ret = devm_request_irq(dev, info->irq,
				       rproc_srm_dev_irq_handler, 0,
				       dev_name(dev), NULL);
		if (ret) {
			dev_err(dev, "Failed to request irq (%d)\n", ret);
			goto err;
		}

		/*
		 * Disable IRQ. Since it is used by the remote processor we
		 * must not use the 'irq lazy disable' optimization
		 */
		irq_set_status_flags(info->irq, IRQ_DISABLE_UNLAZY);
		disable_irq(info->irq);

		/* Note: "interrupt-names" is optional */
		if (!of_property_read_string_index(np, "interrupt-names", i,
						   &name))
			info->name = devm_kstrdup(dev, name, GFP_KERNEL);
		else
			info->name = devm_kstrdup(dev, "", GFP_KERNEL);

		info->index = i;

		list_add_tail(&info->list, &rproc_srm_dev->irq_list_head);
		dev_dbg(dev, "Got irq %d (%s)\n", info->irq, info->name);
	}

	return 0;

err:
	rproc_srm_dev_irqs_put(rproc_srm_dev);

	return ret;
}

/* Clocks */
static void rproc_srm_dev_clocks_unsetup(struct rproc_srm_dev *rproc_srm_dev)
{
	struct rproc_srm_clk_info *c;

	list_for_each_entry(c, &rproc_srm_dev->clk_list_head, list) {
		if (!c->parent_enabled)
			continue;

		clk_disable_unprepare(clk_get_parent(c->clk));
		c->parent_enabled = false;
		dev_dbg(rproc_srm_dev->dev, "clk %d (%s) unsetup\n",
			c->index, c->name);
	}
}

static int rproc_srm_dev_clocks_setup(struct rproc_srm_dev *rproc_srm_dev)
{
	struct rproc_srm_clk_info *c;
	int ret;

	/*
	 * Prepare and enable the parent clocks.
	 * Since the clock tree is under the exclusive control of the master
	 * processor, we need to configure the clock tree of the targeted clock.
	 * We do not want to enable the clock itself, which is under the
	 * responsibility of the remote processor.
	 * Hence we prepare and enable the parent clock.
	 */

	list_for_each_entry(c, &rproc_srm_dev->clk_list_head, list) {
		if (c->parent_enabled)
			continue;

		ret = clk_prepare_enable(clk_get_parent(c->clk));
		if (ret) {
			dev_err(rproc_srm_dev->dev,
				"clk %d (%s) parent enable failed\n",
				c->index, c->name);
			rproc_srm_dev_clocks_unsetup(rproc_srm_dev);
			return ret;
		}
		c->parent_enabled = true;
		dev_dbg(rproc_srm_dev->dev, "clk %d (%s) parent enabled\n",
			c->index, c->name);
	}

	return 0;
}

static struct rproc_srm_clk_info
	*rproc_srm_dev_clock_find(struct rproc_srm_dev *rproc_srm_dev,
				  struct clock_cfg *cfg)
{
	struct rproc_srm_clk_info *ci;

	/* Search by index (if valid value) otherwise search by name */
	list_for_each_entry(ci, &rproc_srm_dev->clk_list_head, list) {
		if (cfg->index != U32_MAX) {
			if (ci->index == cfg->index)
				return ci;
		} else {
			if (!strcmp(ci->name, cfg->name))
				return ci;
		}
	}

	return NULL;
}

static int rproc_srm_dev_clock_set_cfg(struct rproc_srm_dev *rproc_srm_dev,
				       struct clock_cfg *cfg)
{
	struct rproc_srm_clk_info *c;
	struct device *dev = rproc_srm_dev->dev;
	int ret;

	c = rproc_srm_dev_clock_find(rproc_srm_dev, cfg);

	if (!c) {
		dev_err(dev, "unknown clock (id %d)\n", cfg->index);
		return -EINVAL;
	}

	if (cfg->rate && clk_get_rate(c->clk) != cfg->rate) {
		ret = clk_set_rate(c->clk, cfg->rate);
		if (ret) {
			dev_err(dev, "clk set rate failed\n");
			return ret;
		}

		dev_dbg(dev, "clk %d (%s) rate = %d\n", c->index, c->name,
			cfg->rate);
	}

	return 0;
}

static int rproc_srm_dev_clock_get_cfg(struct rproc_srm_dev *rproc_srm_dev,
				       struct clock_cfg *cfg)
{
	struct rproc_srm_clk_info *c;

	c = rproc_srm_dev_clock_find(rproc_srm_dev, cfg);
	if (!c) {
		dev_err(rproc_srm_dev->dev, "unknown clock (%d)\n", cfg->index);
		return -EINVAL;
	}

	strscpy(cfg->name, c->name, sizeof(cfg->name));
	cfg->index = c->index;
	cfg->rate = (u32)clk_get_rate(c->clk);

	return 0;
}

static void rproc_srm_dev_clocks_put(struct rproc_srm_dev *rproc_srm_dev)
{
	struct device *dev = rproc_srm_dev->dev;
	struct rproc_srm_clk_info *c, *tmp;

	list_for_each_entry_safe(c, tmp, &rproc_srm_dev->clk_list_head, list) {
		clk_put(c->clk);
		dev_dbg(dev, "put clock %d (%s)\n", c->index, c->name);
		list_del(&c->list);
	}
}

static int rproc_srm_dev_clocks_get(struct rproc_srm_dev *rproc_srm_dev)
{
	struct device *dev = rproc_srm_dev->dev;
	struct device_node *np = dev->of_node;
	struct rproc_srm_clk_info *c;
	const char *name;
	int nb_c, ret;
	unsigned int i;

	if (!np)
		return 0;

	nb_c = of_clk_get_parent_count(np);
	if (!nb_c)
		return 0;

	for (i = 0; i < nb_c; i++) {
		c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
		if (!c) {
			ret = -ENOMEM;
			goto err;
		}

		c->clk = of_clk_get(np, i);
		if (IS_ERR(c->clk)) {
			dev_err(dev, "clock %d KO (%ld)\n", i,
				PTR_ERR(c->clk));
			ret = -ENOMEM;
			goto err;
		}

		/* Note: "clock-names" is optional */
		if (!of_property_read_string_index(np, "clock-names", i,
						   &name))
			c->name = devm_kstrdup(dev, name, GFP_KERNEL);
		else
			c->name = devm_kstrdup(dev, "", GFP_KERNEL);

		c->index = i;

		list_add_tail(&c->list, &rproc_srm_dev->clk_list_head);
		dev_dbg(dev, "got clock %d (%s)\n", c->index, c->name);
	}

	return 0;

err:
	rproc_srm_dev_clocks_put(rproc_srm_dev);
	return ret;
}

/* Regulators */
static void rproc_srm_dev_regus_unsetup(struct rproc_srm_dev *rproc_srm_dev)
{
	struct rproc_srm_regu_info *r;
	struct device *dev = rproc_srm_dev->dev;

	list_for_each_entry(r, &rproc_srm_dev->regu_list_head, list) {
		if (!r->enabled)
			continue;

		if (regulator_disable(r->regu)) {
			dev_warn(dev, "regu %d disabled failed\n", r->index);
			continue;
		}

		r->enabled = false;
		dev_dbg(dev, "regu %d (%s) disabled\n", r->index, r->name);
	}
}

static int rproc_srm_dev_regus_setup(struct rproc_srm_dev *rproc_srm_dev)
{
	struct rproc_srm_regu_info *r;
	int ret;

	/* Enable all the regulators */
	list_for_each_entry(r, &rproc_srm_dev->regu_list_head, list) {
		if (r->enabled)
			continue;

		/* in early_boot mode sync on hw */
		if (rproc_srm_dev->early_boot && !regulator_is_enabled(r->regu))
			continue;

		ret = regulator_enable(r->regu);
		if (ret) {
			dev_err(rproc_srm_dev->dev, "regu %d (%s) failed\n",
				r->index, r->name);
			rproc_srm_dev_regus_unsetup(rproc_srm_dev);
			return ret;
		}
		r->enabled = true;
		dev_dbg(rproc_srm_dev->dev, "regu %d (%s) enabled\n",
			r->index, r->name);
	}

	return 0;
}

static struct rproc_srm_regu_info
	*rproc_srm_dev_regu_find(struct rproc_srm_dev *rproc_srm_dev,
				 struct regu_cfg *cfg)
{
	struct rproc_srm_regu_info *ri;

	list_for_each_entry(ri, &rproc_srm_dev->regu_list_head, list) {
		if (cfg->index != U32_MAX) {
			if (ri->index == cfg->index)
				return ri;
		} else {
			if (!strcmp(ri->name, cfg->name))
				return ri;
		}
	}

	return NULL;
}

static int rproc_srm_dev_regu_set_cfg(struct rproc_srm_dev *rproc_srm_dev,
				      struct regu_cfg *cfg)
{
	struct rproc_srm_regu_info *r;
	struct device *dev = rproc_srm_dev->dev;
	int ret;

	r = rproc_srm_dev_regu_find(rproc_srm_dev, cfg);
	if (!r) {
		dev_err(dev, "unknown regu (%d)\n", cfg->index);
		return -EINVAL;
	}

	if (!r->enabled && cfg->enable) {
		ret = regulator_enable(r->regu);
		if (ret) {
			dev_err(dev, "regu %d enable failed\n", r->index);
			return ret;
		}
		r->enabled = true;
		dev_dbg(dev, "regu %d (%s) enabled\n", r->index, r->name);
	} else if (r->enabled && !cfg->enable) {
		ret = regulator_disable(r->regu);
		if (ret) {
			dev_err(dev, "regu %d disable failed\n", r->index);
			return ret;
		}
		r->enabled = false;
		dev_dbg(dev, "regu %d (%s) disabled\n", r->index, r->name);
	}

	if (cfg->min_voltage_mv || cfg->max_voltage_mv) {
		ret = regulator_set_voltage(r->regu, cfg->min_voltage_mv * 1000,
					    cfg->max_voltage_mv * 1000);
		if (ret) {
			dev_err(dev, "regu %d set voltage failed\n", r->index);
			return ret;
		}

		dev_dbg(dev, "regu %d (%s) voltage = [%d - %d] mv\n", r->index,
			r->name, cfg->min_voltage_mv, cfg->max_voltage_mv);
	}

	return 0;
}

static int rproc_srm_dev_regu_get_cfg(struct rproc_srm_dev *rproc_srm_dev,
				      struct regu_cfg *cfg)
{
	struct rproc_srm_regu_info *r;
	struct device *dev = rproc_srm_dev->dev;
	int v;

	r = rproc_srm_dev_regu_find(rproc_srm_dev, cfg);
	if (!r) {
		dev_err(dev, "unknown regu (%d)\n", cfg->index);
		return -EINVAL;
	}

	strscpy(cfg->name, r->name, sizeof(cfg->name));
	cfg->index = r->index;
	cfg->enable = r->enabled;
	cfg->min_voltage_mv = 0;
	cfg->max_voltage_mv = 0;

	v = regulator_get_voltage(r->regu);
	if (v < 0) {
		dev_warn(dev, "cannot get %s voltage\n", r->name);
		cfg->curr_voltage_mv = 0;
	} else {
		cfg->curr_voltage_mv = v / 1000;
	}

	return 0;
}

static void rproc_srm_dev_regus_put(struct rproc_srm_dev *rproc_srm_dev)
{
	struct device *dev = rproc_srm_dev->dev;
	struct rproc_srm_regu_info *r, *tmp;

	list_for_each_entry_safe(r, tmp, &rproc_srm_dev->regu_list_head, list) {
		devm_regulator_put(r->regu);
		dev_dbg(dev, "put regu %d (%s)\n", r->index, r->name);
		list_del(&r->list);
	}
}

static int rproc_srm_dev_regus_get(struct rproc_srm_dev *rproc_srm_dev)
{
	struct device *dev = rproc_srm_dev->dev;
	struct device_node *np = dev->of_node;
	struct property *p;
	const char *n;
	char *name;
	struct rproc_srm_regu_info *r;
	int ret, nb_s = 0;

	if (!np)
		return 0;

	for_each_property_of_node(np, p) {
		n = strstr(p->name, "-supply");
		if (!n || n == p->name)
			continue;

		r = devm_kzalloc(dev, sizeof(*r), GFP_KERNEL);
		if (!r) {
			ret = -ENOMEM;
			goto err_list;
		}

		name = devm_kstrdup(dev, p->name, GFP_KERNEL);
		name[strlen(p->name) - strlen("-supply")] = '\0';
		r->name = name;

		r->regu = devm_regulator_get(dev, r->name);
		if (IS_ERR(r->regu)) {
			dev_err(dev, "cannot get regu %s\n", r->name);
			ret = -EINVAL;
			goto err_list;
		}

		r->index = nb_s++;

		list_add_tail(&r->list, &rproc_srm_dev->regu_list_head);
		dev_dbg(dev, "got regu %d (%s)\n", r->index, r->name);
	}

	return 0;

err_list:
	rproc_srm_dev_regus_put(rproc_srm_dev);
	return ret;
}

/* Core */
static int rproc_srm_dev_notify_cb(struct notifier_block *nb, unsigned long evt,
				   void *data)
{
	struct rproc_srm_dev *rproc_srm_dev =
			container_of(nb, struct rproc_srm_dev, nb);
	struct device *dev = rproc_srm_dev->dev;
	struct rpmsg_srm_msg_desc *desc;
	struct rpmsg_srm_msg *i, o;
	int ret = 0;

	desc = (struct rpmsg_srm_msg_desc *)data;
	i = desc->msg;
	o = *i;

	/* Check if 'device_id' (name / addr ) matches this device */
	if (!strstr(dev_name(dev), i->device_id))
		return NOTIFY_DONE;

	switch (i->message_type) {
	case RPROC_SRM_MSG_SETCONFIG:
		switch (i->rsc_type) {
		case RPROC_SRM_RSC_CLOCK:
			ret = rproc_srm_dev_clock_set_cfg(rproc_srm_dev,
							  &i->clock_cfg);
			if (!ret)
				ret = rproc_srm_dev_clock_get_cfg(rproc_srm_dev,
								  &o.clock_cfg);
			break;
		case RPROC_SRM_RSC_REGU:
			ret = rproc_srm_dev_regu_set_cfg(rproc_srm_dev,
							 &i->regu_cfg);
			if (!ret)
				ret = rproc_srm_dev_regu_get_cfg(rproc_srm_dev,
								 &o.regu_cfg);
			break;
		default:
			dev_warn(dev, "bad rsc type (%d)\n", i->rsc_type);
			ret = -EINVAL;
			break;
		}
		break;
	case RPROC_SRM_MSG_GETCONFIG:
		switch (i->rsc_type) {
		case RPROC_SRM_RSC_CLOCK:
			ret = rproc_srm_dev_clock_get_cfg(rproc_srm_dev,
							  &o.clock_cfg);
			break;
		case RPROC_SRM_RSC_REGU:
			ret = rproc_srm_dev_regu_get_cfg(rproc_srm_dev,
							 &o.regu_cfg);
			break;
		default:
			dev_warn(dev, "bad rsc type (%d)\n", i->rsc_type);
			ret = -EINVAL;
			break;
		}
		break;
	default:
		dev_warn(dev, "bad msg type (%d)\n", i->message_type);
		ret = -EINVAL;
		break;
	}

	/* Send return msg */
	if (ret)
		o.message_type = RPROC_SRM_MSG_ERROR;

	ret = rpmsg_srm_send(desc->ept, &o);

	return ret ? NOTIFY_BAD : NOTIFY_STOP;
}

static void
rproc_srm_dev_unbind(struct device *dev, struct device *master, void *data)
{
	struct rproc_srm_dev *rproc_srm_dev = dev_get_drvdata(dev);

	rproc_srm_dev_regus_unsetup(rproc_srm_dev);
	rproc_srm_dev_clocks_unsetup(rproc_srm_dev);

	/* For IRQs: nothing to unsetup */
}

static int
rproc_srm_dev_bind(struct device *dev, struct device *master, void *data)
{
	struct rproc_srm_dev *rproc_srm_dev = dev_get_drvdata(dev);
	int ret;

	ret = rproc_srm_dev_clocks_setup(rproc_srm_dev);
	if (ret)
		return ret;

	ret = rproc_srm_dev_regus_setup(rproc_srm_dev);
	if (ret)
		return ret;

	/* For IRQs: nothing to setup */
	return 0;
}

static const struct component_ops rproc_srm_dev_ops = {
	.bind = rproc_srm_dev_bind,
	.unbind = rproc_srm_dev_unbind,
};

static int rproc_srm_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc_srm_dev *rproc_srm_dev;
	struct rproc *rproc;
	int ret;

	dev_dbg(dev, "%s for node %s\n", __func__, dev->of_node->name);

	rproc_srm_dev = devm_kzalloc(dev, sizeof(struct rproc_srm_dev),
				     GFP_KERNEL);
	if (!rproc_srm_dev)
		return -ENOMEM;

	rproc_srm_dev->dev = dev;
	rproc = (struct rproc *)dev_get_drvdata(dev->parent->parent);
	rproc_srm_dev->early_boot = (rproc->state == RPROC_DETACHED);
	rproc_srm_dev->core = dev_get_drvdata(dev->parent);

	INIT_LIST_HEAD(&rproc_srm_dev->clk_list_head);
	INIT_LIST_HEAD(&rproc_srm_dev->regu_list_head);
	INIT_LIST_HEAD(&rproc_srm_dev->irq_list_head);

	/* Get clocks, regu and irqs */
	ret = rproc_srm_dev_clocks_get(rproc_srm_dev);
	if (ret)
		return ret;

	ret = rproc_srm_dev_regus_get(rproc_srm_dev);
	if (ret)
		goto err_get;

	ret = rproc_srm_dev_irqs_get(rproc_srm_dev);
	if (ret)
		goto err_get;

	rproc_srm_dev->nb.notifier_call = rproc_srm_dev_notify_cb;
	ret = rproc_srm_core_register_notifier(rproc_srm_dev->core,
					       &rproc_srm_dev->nb);
	if (ret)
		goto err_register;

	dev_set_drvdata(dev, rproc_srm_dev);

	return component_add(dev, &rproc_srm_dev_ops);

err_register:
	rproc_srm_core_unregister_notifier(rproc_srm_dev->core,
					   &rproc_srm_dev->nb);
err_get:
	rproc_srm_dev_irqs_put(rproc_srm_dev);
	rproc_srm_dev_regus_put(rproc_srm_dev);
	rproc_srm_dev_clocks_put(rproc_srm_dev);
	return ret;
}

static void rproc_srm_dev_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc_srm_dev *rproc_srm_dev = dev_get_drvdata(dev);

	component_del(dev, &rproc_srm_dev_ops);

	rproc_srm_core_unregister_notifier(rproc_srm_dev->core,
					   &rproc_srm_dev->nb);

	rproc_srm_dev_irqs_put(rproc_srm_dev);
	rproc_srm_dev_regus_put(rproc_srm_dev);
	rproc_srm_dev_clocks_put(rproc_srm_dev);
}

static const struct of_device_id rproc_srm_dev_match[] = {
	{ .compatible = "rproc-srm-dev", },
	{},
};

MODULE_DEVICE_TABLE(of, rproc_srm_dev_match);

static struct platform_driver rproc_srm_dev_driver = {
	.probe = rproc_srm_dev_probe,
	.remove = rproc_srm_dev_remove,
	.driver = {
		.name = "rproc-srm-dev",
		.of_match_table = of_match_ptr(rproc_srm_dev_match),
	},
};

module_platform_driver(rproc_srm_dev_driver);

MODULE_AUTHOR("Fabien Dessenne <fabien.dessenne@st.com>");
MODULE_DESCRIPTION("Remoteproc System Resource Manager driver - dev");
MODULE_LICENSE("GPL");
