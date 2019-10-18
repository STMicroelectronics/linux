// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2018 - All Rights Reserved
 * Author: Fabien Dessenne <fabien.dessenne@st.com> for STMicroelectronics.
 */

#include <linux/component.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/remoteproc.h>

#define BIND_TIMEOUT 10000

struct rproc_srm_core {
	struct device *dev;
	struct completion all_bound;
	int bind_status;
	atomic_t prepared;
	struct rproc_subdev subdev;
};

#define to_rproc_srm_core(s) container_of(s, struct rproc_srm_core, subdev)

static int compare_of(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static void release_of(struct device *dev, void *data)
{
	of_node_put(data);
}

static void rproc_srm_core_unbind(struct device *dev)
{
	component_unbind_all(dev, NULL);
}

static int rproc_srm_core_bind(struct device *dev)
{
	struct rproc_srm_core *rproc_srm_core = dev_get_drvdata(dev);

	rproc_srm_core->bind_status = component_bind_all(dev, NULL);
	complete(&rproc_srm_core->all_bound);

	return rproc_srm_core->bind_status;
}

static const struct component_master_ops srm_comp_ops = {
	.bind = rproc_srm_core_bind,
	.unbind = rproc_srm_core_unbind,
};

static int rproc_srm_core_prepare(struct rproc_subdev *subdev)
{
	struct rproc_srm_core *rproc_srm_core = to_rproc_srm_core(subdev);
	struct device *dev = rproc_srm_core->dev;
	struct device_node *node = dev->of_node;
	struct device_node *child_np;
	struct component_match *match = NULL;
	int ret;

	init_completion(&rproc_srm_core->all_bound);

	ret = devm_of_platform_populate(dev);
	if (ret) {
		dev_err(dev, "cannot populate node (%d)\n", ret);
		return ret;
	}

	child_np = of_get_next_available_child(node, NULL);

	while (child_np) {
		of_node_get(child_np);
		component_match_add_release(dev, &match, release_of, compare_of,
					    child_np);
		child_np = of_get_next_available_child(node, child_np);
	}

	if (!match) {
		dev_dbg(dev, "No available child\n");
		goto done;
	}

	ret = component_master_add_with_match(dev, &srm_comp_ops, match);
	if (ret)
		goto depopulate;

	/* Wait for every child to be bound */
	if (!wait_for_completion_timeout(&rproc_srm_core->all_bound,
					 msecs_to_jiffies(BIND_TIMEOUT))) {
		dev_err(dev, "failed to bind one or more system resource device(s)\n");
		ret = -ETIMEDOUT;
		goto master;
	}

	ret = rproc_srm_core->bind_status;
	if (ret) {
		dev_err(dev, "failed to bind\n");
		goto master;
	}
done:
	atomic_inc(&rproc_srm_core->prepared);

	return 0;

master:
	component_master_del(dev, &srm_comp_ops);
depopulate:
	devm_of_platform_depopulate(dev);
	return ret;
}

static void rproc_srm_core_unprepare(struct rproc_subdev *subdev)
{
	struct rproc_srm_core *rproc_srm_core = to_rproc_srm_core(subdev);
	struct device *dev = rproc_srm_core->dev;

	if (!atomic_read(&rproc_srm_core->prepared))
		return;

	atomic_dec(&rproc_srm_core->prepared);

	component_master_del(dev, &srm_comp_ops);
	devm_of_platform_depopulate(dev);
}

static int rproc_srm_core_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc *rproc = dev_get_drvdata(dev->parent);
	struct rproc_srm_core *rproc_srm_core;

	rproc_srm_core = devm_kzalloc(dev, sizeof(struct rproc_srm_core),
				      GFP_KERNEL);
	if (!rproc_srm_core)
		return -ENOMEM;

	rproc_srm_core->dev = dev;

	/* Register rproc subdevice with (un)prepare ops */
	rproc_srm_core->subdev.prepare = rproc_srm_core_prepare;
	rproc_srm_core->subdev.unprepare = rproc_srm_core_unprepare;
	rproc_add_subdev(rproc, &rproc_srm_core->subdev);

	dev_set_drvdata(dev, rproc_srm_core);

	return 0;
}

static void rproc_srm_core_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc_srm_core *rproc_srm_core = dev_get_drvdata(dev);
	struct rproc *rproc = dev_get_drvdata(dev->parent);

	if (atomic_read(&rproc->power) > 0)
		dev_warn(dev, "Releasing resources while firmware running!\n");

	rproc_srm_core_unprepare(&rproc_srm_core->subdev);
}

static const struct of_device_id rproc_srm_core_match[] = {
	{ .compatible = "rproc-srm-core", },
	{},
};

MODULE_DEVICE_TABLE(of, rproc_srm_core_match);

static struct platform_driver rproc_srm_core_driver = {
	.probe = rproc_srm_core_probe,
	.remove = rproc_srm_core_remove,
	.driver = {
		.name = "rproc-srm-core",
		.of_match_table = of_match_ptr(rproc_srm_core_match),
	},
};

module_platform_driver(rproc_srm_core_driver);

MODULE_AUTHOR("Fabien Dessenne <fabien.dessenne@st.com>");
MODULE_DESCRIPTION("Remoteproc System Resource Manager driver - core");
MODULE_LICENSE("GPL");
