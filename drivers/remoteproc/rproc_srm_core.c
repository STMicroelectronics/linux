// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2018 - All Rights Reserved
 * Author: Fabien Dessenne <fabien.dessenne@st.com> for STMicroelectronics.
 */

#include <linux/component.h>
#include <linux/fwnode.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/remoteproc.h>
#include <linux/rpmsg.h>

#include "rproc_srm_core.h"

#define BIND_TIMEOUT 10000

struct rproc_srm_core {
	struct device *dev;
	struct completion all_bound;
	int bind_status;
	atomic_t prepared;
	struct rproc_subdev subdev;
	struct rpmsg_driver rpdrv;
	struct blocking_notifier_head notifier;
};

#define to_rproc_srm_core(s) container_of(s, struct rproc_srm_core, subdev)

static struct rproc_srm_core *rpmsg_srm_to_core(struct rpmsg_device *rpdev)
{
	struct rpmsg_driver *rpdrv;
	struct rproc_srm_core *core;

	rpdrv = container_of(rpdev->dev.driver, struct rpmsg_driver, drv);
	core = container_of(rpdrv, struct rproc_srm_core, rpdrv);

	return core;
}

int rpmsg_srm_send(struct rpmsg_endpoint *ept, struct rpmsg_srm_msg *msg)
{
	int ret;

	ret = rpmsg_send(ept, (void *)msg, sizeof(*msg));
	if (ret)
		dev_err(&ept->rpdev->dev, "rpmsg_send failed: %d\n", ret);

	return ret;
}
EXPORT_SYMBOL(rpmsg_srm_send);

static int rpmsg_srm_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	struct rproc_srm_core *core = rpmsg_srm_to_core(rpdev);
	struct rpmsg_srm_msg_desc desc;
	int ret;

	desc.ept = rpdev->ept;
	desc.msg = data;

	ret = blocking_notifier_call_chain(&core->notifier, 0, &desc);

	if (!(ret & NOTIFY_STOP_MASK)) {
		dev_warn(&rpdev->dev, "unknown device\n");
		desc.msg->message_type = RPROC_SRM_MSG_ERROR;
		rpmsg_srm_send(desc.ept, desc.msg);
	}

	return 0;
}

static int rpmsg_srm_probe(struct rpmsg_device *rpdev)
{
	int ret;

	/* Send an empty message to complete the initialization */
	ret = rpmsg_send(rpdev->ept, NULL, 0);
	if (ret)
		dev_err(&rpdev->dev, "failed to send init message\n");

	return ret;
}

static void rpmsg_srm_remove(struct rpmsg_device *rpdev)
{
	/* Note : the remove ops is mandatory */
}

static struct rpmsg_device_id rpmsg_srm_id_table[] = {
	{ .name	= "rproc-srm" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_srm_id_table);

static struct rpmsg_driver rpmsg_srm_drv = {
	.drv.name	= "rpmsg_srm",
	.id_table	= rpmsg_srm_id_table,
	.probe		= rpmsg_srm_probe,
	.callback	= rpmsg_srm_cb,
	.remove		= rpmsg_srm_remove,
};

int rproc_srm_core_register_notifier(struct rproc_srm_core *core,
				     struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&core->notifier, nb);
}
EXPORT_SYMBOL(rproc_srm_core_register_notifier);

int rproc_srm_core_unregister_notifier(struct rproc_srm_core *core,
				       struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&core->notifier, nb);
}
EXPORT_SYMBOL(rproc_srm_core_unregister_notifier);

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

	/* Register rpmsg driver for dynamic management */
	rproc_srm_core->rpdrv = rpmsg_srm_drv;
	ret = register_rpmsg_driver(&rproc_srm_core->rpdrv);
	if (ret) {
		dev_err(dev, "failed to register rpmsg drv\n");
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

	unregister_rpmsg_driver(&rproc_srm_core->rpdrv);

	component_master_del(dev, &srm_comp_ops);
	devm_of_platform_depopulate(dev);
}

/*
 * SRM sub-nodes are populated as platform devices only when the coprocessor
 * starts, so fw_devlink cannot resolve their supplier links at boot and
 * emits spurious "device link" errors. Drop those dangling fwnode links.
 */
static void rproc_srm_core_purge_fwnode_links(struct device *dev)
{
	struct device_node *child;

	/* Links already picked up on the SRM core node itself. */
	fwnode_links_purge(of_fwnode_handle(dev->of_node));

	/* Links still attached to the not-yet-populated sub-nodes. */
	for_each_child_of_node_scoped(dev->of_node, child)
		fwnode_links_purge(of_fwnode_handle(child));
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
	BLOCKING_INIT_NOTIFIER_HEAD(&rproc_srm_core->notifier);

	rproc_srm_core_purge_fwnode_links(dev);

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
