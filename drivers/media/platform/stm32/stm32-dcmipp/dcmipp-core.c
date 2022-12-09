// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for STM32 Digital Camera Memory Interface Pixel Processor
 *
 * Copyright (C) STMicroelectronics SA 2021
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          Alain Volmat <alain.volmat@foss.st.com>
 *          for STMicroelectronics.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <media/media-device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#include "dcmipp-common.h"

#define DCMIPP_MDEV_MODEL_NAME "DCMIPP MDEV"

#define DCMIPP_ENT_LINK(src, srcpad, sink, sinkpad, link_flags) {	\
	.src_ent = src,						\
	.src_pad = srcpad,					\
	.sink_ent = sink,					\
	.sink_pad = sinkpad,					\
	.flags = link_flags,					\
}

#define DCMIPP_CMHWCFGR (0x200)
#define DCMIPP_P0HWCFGR (0x400)
#define DCMIPP_VERR (0xFF4)

struct dcmipp_device {
	/* The platform device */
	struct platform_device		pdev;
	struct device			*dev;

	/* Hardware resources */
	struct reset_control		*rstc;
	void __iomem			*regs;
	struct clk			*kclk;

	/* The pipeline configuration */
	const struct dcmipp_pipeline_config	*pipe_cfg;

	/* The Associated media_device parent */
	struct media_device		mdev;

	/* Internal v4l2 parent device*/
	struct v4l2_device		v4l2_dev;

	/* Subdevices */
	struct platform_device		**subdevs;

	struct v4l2_async_notifier	notifier;
};

static inline struct dcmipp_device *notifier_to_dcmipp(struct v4l2_async_notifier *n)
{
	return container_of(n, struct dcmipp_device, notifier);
}

/* Structure which describes individual configuration for each entity */
struct dcmipp_ent_config {
	const char *name;
	const char *drv;
};

/* Structure which describes links between entities */
struct dcmipp_ent_link {
	unsigned int src_ent;
	u16 src_pad;
	unsigned int sink_ent;
	u16 sink_pad;
	u32 flags;
};

/* Structure which describes the whole topology */
struct dcmipp_pipeline_config {
	const struct dcmipp_ent_config *ents;
	size_t num_ents;
	const struct dcmipp_ent_link *links;
	size_t num_links;
};

/* --------------------------------------------------------------------------
 * Topology Configuration
 */

static const struct dcmipp_ent_config stm32mp13_ent_config[] = {
	{
		.name = "dcmipp_parallel",
		.drv = "dcmipp-parallel",
	},
	{
		.name = "dcmipp_dump_postproc",
		.drv = "dcmipp-byteproc",
	},
	{
		.name = "dcmipp_dump_capture",
		.drv = "dcmipp-bytecap",
	},
};

#define ID_PARALLEL 0
#define ID_DUMP_BYTEPROC 1
#define ID_DUMP_CAPTURE 2

static const struct dcmipp_ent_link stm32mp13_ent_links[] = {
	DCMIPP_ENT_LINK(ID_PARALLEL,      1, ID_DUMP_BYTEPROC, 0,
			MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE),
	DCMIPP_ENT_LINK(ID_DUMP_BYTEPROC, 1, ID_DUMP_CAPTURE,  0,
			MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE),
};

static const struct dcmipp_pipeline_config stm32mp13_pipe_cfg = {
	.ents		= stm32mp13_ent_config,
	.num_ents	= ARRAY_SIZE(stm32mp13_ent_config),
	.links		= stm32mp13_ent_links,
	.num_links	= ARRAY_SIZE(stm32mp13_ent_links)
};

/* -------------------------------------------------------------------------- */
#define LINK_FLAG_TO_STR(f) ((f) == 0 ? "" :\
			     (f) == MEDIA_LNK_FL_ENABLED ? "ENABLED" :\
			     (f) == MEDIA_LNK_FL_IMMUTABLE ? "IMMUTABLE" :\
			     (f) == (MEDIA_LNK_FL_ENABLED |\
				     MEDIA_LNK_FL_IMMUTABLE) ?\
					"ENABLED, IMMUTABLE" :\
			     "UNKNOWN")

static int dcmipp_create_links(struct dcmipp_device *dcmipp)
{
	unsigned int i;
	int ret;

	/* Initialize the links between entities */
	for (i = 0; i < dcmipp->pipe_cfg->num_links; i++) {
		const struct dcmipp_ent_link *link = &dcmipp->pipe_cfg->links[i];
		/*
		 * TODO: Check another way of retrieving ved struct without
		 * relying on platform_get_drvdata
		 */
		struct dcmipp_ent_device *ved_src =
			platform_get_drvdata(dcmipp->subdevs[link->src_ent]);
		struct dcmipp_ent_device *ved_sink =
			platform_get_drvdata(dcmipp->subdevs[link->sink_ent]);

		dev_dbg(dcmipp->dev, "Create link \"%s\":%d -> %d:\"%s\" [%s]\n",
			dcmipp->pipe_cfg->ents[link->src_ent].name,
			link->src_pad,
			link->sink_pad,
			dcmipp->pipe_cfg->ents[link->sink_ent].name,
			LINK_FLAG_TO_STR(link->flags));

		ret = media_create_pad_link(ved_src->ent, link->src_pad,
					    ved_sink->ent, link->sink_pad,
					    link->flags);
		if (ret)
			return ret;
	}

	return 0;
}

static int dcmipp_graph_init(struct dcmipp_device *dcmipp);

static int dcmipp_comp_bind(struct device *master)
{
	struct dcmipp_device *dcmipp = platform_get_drvdata(to_platform_device(master));
	struct dcmipp_bind_data bind_data;
	int ret;

	/* Register the v4l2 struct */
	ret = v4l2_device_register(dcmipp->mdev.dev, &dcmipp->v4l2_dev);
	if (ret) {
		dev_err(dcmipp->mdev.dev,
			"v4l2 device register failed (err=%d)\n", ret);
		return ret;
	}

	/* Bind subdevices */
	bind_data.v4l2_dev = &dcmipp->v4l2_dev;
	bind_data.rstc = dcmipp->rstc;
	bind_data.regs = dcmipp->regs;
	ret = component_bind_all(master, &bind_data);
	if (ret)
		goto err_v4l2_unregister;

	/* Initialize links */
	ret = dcmipp_create_links(dcmipp);
	if (ret)
		goto err_comp_unbind_all;

	ret = dcmipp_graph_init(dcmipp);
	if (ret < 0)
		return ret;

	return 0;

	media_device_unregister(&dcmipp->mdev);
	media_device_cleanup(&dcmipp->mdev);
err_comp_unbind_all:
	component_unbind_all(master, NULL);
err_v4l2_unregister:
	v4l2_device_unregister(&dcmipp->v4l2_dev);

	return ret;
}

static void dcmipp_comp_unbind(struct device *master)
{
	struct dcmipp_device *dcmipp = platform_get_drvdata(to_platform_device(master));

	v4l2_async_notifier_unregister(&dcmipp->notifier);
	v4l2_async_notifier_cleanup(&dcmipp->notifier);

	media_device_unregister(&dcmipp->mdev);
	media_device_cleanup(&dcmipp->mdev);
	component_unbind_all(master, NULL);
	v4l2_device_unregister(&dcmipp->v4l2_dev);
}

static int dcmipp_comp_compare(struct device *comp, void *data)
{
	return comp == data;
}

static struct component_match *dcmipp_add_subdevs(struct dcmipp_device *dcmipp)
{
	struct component_match *match = NULL;
	struct dcmipp_platform_data pdata;
	int i;

	for (i = 0; i < dcmipp->pipe_cfg->num_ents; i++) {
		dev_dbg(dcmipp->dev, "new pdev for %s (%s)\n",
			dcmipp->pipe_cfg->ents[i].drv,
			dcmipp->pipe_cfg->ents[i].name);

		strscpy(pdata.entity_name, dcmipp->pipe_cfg->ents[i].name,
			sizeof(pdata.entity_name));

		dcmipp->subdevs[i] =
			platform_device_register_data
				(dcmipp->dev,
				 dcmipp->pipe_cfg->ents[i].drv,
				 PLATFORM_DEVID_AUTO,
				 &pdata,
				 sizeof(pdata));
		if (IS_ERR(dcmipp->subdevs[i])) {
			match = ERR_CAST(dcmipp->subdevs[i]);
			while (--i >= 0)
				platform_device_unregister(dcmipp->subdevs[i]);

			dev_err(dcmipp->mdev.dev,
				"%s error (err=%ld)\n", __func__,
				PTR_ERR(match));
			return match;
		}

		component_match_add(dcmipp->dev, &match, dcmipp_comp_compare,
				    &dcmipp->subdevs[i]->dev);
	}

	return match;
}

static void dcmipp_rm_subdevs(struct dcmipp_device *dcmipp)
{
	unsigned int i;

	for (i = 0; i < dcmipp->pipe_cfg->num_ents; i++)
		platform_device_unregister(dcmipp->subdevs[i]);
}

static const struct component_master_ops dcmipp_comp_ops = {
	.bind = dcmipp_comp_bind,
	.unbind = dcmipp_comp_unbind,
};

static const struct of_device_id dcmipp_of_match[] = {
	{ .compatible = "st,stm32mp13-dcmipp", .data = &stm32mp13_pipe_cfg},
	{ /* end node */ },
};
MODULE_DEVICE_TABLE(of, dcmipp_of_match);

static int dcmipp_graph_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct dcmipp_device *dcmipp = notifier_to_dcmipp(notifier);
	int ret;

	/* Register the media device */
	ret = media_device_register(&dcmipp->mdev);
	if (ret) {
		dev_err(dcmipp->mdev.dev,
			"media device register failed (err=%d)\n", ret);
		return ret;
	}

	/* Expose all subdev's nodes*/
	ret = v4l2_device_register_subdev_nodes(&dcmipp->v4l2_dev);
	if (ret) {
		dev_err(dcmipp->mdev.dev,
			"dcmipp subdev nodes registration failed (err=%d)\n",
			ret);
		media_device_unregister(&dcmipp->mdev);
		return ret;
	}

	dev_dbg(dcmipp->dev, "Notify complete !\n");

	return 0;
}

static void dcmipp_graph_notify_unbind(struct v4l2_async_notifier *notifier,
				       struct v4l2_subdev *sd,
				       struct v4l2_async_subdev *asd)
{
	struct dcmipp_device *dcmipp = notifier_to_dcmipp(notifier);

	dev_dbg(dcmipp->dev, "Removing %s\n", sd->name);
}

static irqreturn_t dcmipp_irq_thread(int irq, void *arg)
{
	struct dcmipp_device *dcmipp = arg;
	struct dcmipp_ent_device *ved;
	unsigned int i;

	/* Call irq thread of each entities of pipeline */
	for (i = 0; i < dcmipp->pipe_cfg->num_ents; i++) {
		ved = platform_get_drvdata(dcmipp->subdevs[i]);
		if (ved->thread_fn && ved->handler_ret == IRQ_WAKE_THREAD)
			ved->thread_fn(irq, ved);
	}

	return IRQ_HANDLED;
}

static irqreturn_t dcmipp_irq_callback(int irq, void *arg)
{
	struct dcmipp_device *dcmipp = arg;
	struct dcmipp_ent_device *ved;
	irqreturn_t ret = IRQ_HANDLED;
	unsigned int i;

	/* Call irq handler of each entities of pipeline */
	for (i = 0; i < dcmipp->pipe_cfg->num_ents; i++) {
		ved = platform_get_drvdata(dcmipp->subdevs[i]);
		if (ved->handler)
			ved->handler_ret = ved->handler(irq, ved);
		else if (ved->thread_fn)
			ved->handler_ret = IRQ_WAKE_THREAD;
		else
			ved->handler_ret = IRQ_HANDLED;
		if (ved->handler_ret != IRQ_HANDLED)
			ret = ved->handler_ret;
	}

	return ret;
}

static int dcmipp_graph_notify_bound(struct v4l2_async_notifier *notifier,
				     struct v4l2_subdev *subdev,
				     struct v4l2_async_subdev *asd)
{
	struct dcmipp_device *dcmipp = notifier_to_dcmipp(notifier);
	unsigned int ret;
	int src_pad;
	struct dcmipp_ent_device *sink;
	struct device_node *np = dcmipp->dev->of_node;
	struct v4l2_fwnode_endpoint ep = { .bus_type = 0 };

	dev_dbg(dcmipp->dev, "Subdev \"%s\" bound\n", subdev->name);

	/*
	 * Link this sub-device to DCMIPP, it could be
	 * a parallel camera sensor or a CSI-2 to parallel bridge
	 */

	src_pad = media_entity_get_fwnode_pad(&subdev->entity,
					      subdev->fwnode,
					      MEDIA_PAD_FL_SOURCE);

	/* Get bus characteristics from devicetree */
	np = of_graph_get_next_endpoint(np, NULL);
	if (!np) {
		dev_err(dcmipp->dev, "Could not find the endpoint\n");
		of_node_put(np);
		return -ENODEV;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(np), &ep);
	of_node_put(np);
	if (ret) {
		dev_err(dcmipp->dev, "Could not parse the endpoint\n");
		return ret;
	}

	if ((ep.bus_type == V4L2_MBUS_PARALLEL ||
	     ep.bus_type == V4L2_MBUS_BT656) &&
	     ep.bus.parallel.bus_width > 0) {
		/* Only 8 bits bus width supported with BT656 bus */
		if (ep.bus_type == V4L2_MBUS_BT656 &&
		    ep.bus.parallel.bus_width != 8) {
			dev_err(dcmipp->dev, "BT656 bus conflicts with %u bits bus width (8 bits required)\n",
				ep.bus.parallel.bus_width);
			return -ENODEV;
		}

		/*
		 * Parallel input device detected
		 * Connect it to parallel subdev
		 */
		sink = platform_get_drvdata(dcmipp->subdevs[ID_PARALLEL]);
		sink->bus.flags = ep.bus.parallel.flags;
		sink->bus.bus_width = ep.bus.parallel.bus_width;
		sink->bus.data_shift = ep.bus.parallel.data_shift;
		sink->bus_type = ep.bus_type;
		ret = media_create_pad_link(&subdev->entity, src_pad,
					    sink->ent, 0,
					    MEDIA_LNK_FL_IMMUTABLE |
					    MEDIA_LNK_FL_ENABLED);
		if (ret)
			dev_err(dcmipp->dev, "Failed to create media pad link with subdev \"%s\"\n",
				subdev->name);
		else
			dev_dbg(dcmipp->dev, "DCMIPP is now linked to \"%s\"\n",
				subdev->name);

		return 0;
	}

	return ret;
}

static const struct v4l2_async_notifier_operations dcmipp_graph_notify_ops = {
	.bound = dcmipp_graph_notify_bound,
	.unbind = dcmipp_graph_notify_unbind,
	.complete = dcmipp_graph_notify_complete,
};

static int dcmipp_graph_init(struct dcmipp_device *dcmipp)
{
	struct v4l2_async_subdev *asd;
	struct device_node *ep;
	int ret;

	ep = of_graph_get_next_endpoint(dcmipp->dev->of_node, NULL);
	if (!ep) {
		dev_err(dcmipp->dev, "Failed to get next endpoint\n");
		return -EINVAL;
	}

	v4l2_async_notifier_init(&dcmipp->notifier);

	asd = v4l2_async_notifier_add_fwnode_remote_subdev
		(&dcmipp->notifier, of_fwnode_handle(ep),
		 struct v4l2_async_subdev);

	of_node_put(ep);

	if (IS_ERR(asd)) {
		dev_err(dcmipp->dev, "Failed to add fwnode remote subdev\n");
		return PTR_ERR(asd);
	}

	dcmipp->notifier.ops = &dcmipp_graph_notify_ops;

	ret = v4l2_async_notifier_register(&dcmipp->v4l2_dev, &dcmipp->notifier);
	if (ret < 0) {
		dev_err(dcmipp->dev, "Failed to register notifier\n");
		v4l2_async_notifier_cleanup(&dcmipp->notifier);
		return ret;
	}

	return 0;
}

static int dcmipp_probe(struct platform_device *pdev)
{
	struct dcmipp_device *dcmipp;
	struct component_match *comp_match = NULL;
	struct resource *res;
	struct clk *kclk;
	const struct dcmipp_pipeline_config *pipe_cfg;
	int irq;
	int ret;

	dcmipp = devm_kzalloc(&pdev->dev, sizeof(struct dcmipp_device), GFP_KERNEL);
	if (!dcmipp)
		return -ENOMEM;

	dcmipp->dev = &pdev->dev;

	pipe_cfg = of_device_get_match_data(&pdev->dev);
	if (!pipe_cfg) {
		dev_err(&pdev->dev, "Can't get device data\n");
		return -ENODEV;
	}
	dcmipp->pipe_cfg = pipe_cfg;

	platform_set_drvdata(pdev, dcmipp);

	/* Get hardware resources from devicetree */
	dcmipp->rstc = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(dcmipp->rstc))
		return dev_err_probe(&pdev->dev, PTR_ERR(dcmipp->rstc),
				     "Could not get reset control\n");

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		if (irq != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Could not get irq\n");
		return irq ? irq : -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Could not get resource\n");
		return -ENODEV;
	}

	dcmipp->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dcmipp->regs)) {
		dev_err(&pdev->dev, "Could not map registers\n");
		return PTR_ERR(dcmipp->regs);
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, dcmipp_irq_callback,
					dcmipp_irq_thread, IRQF_ONESHOT,
					dev_name(&pdev->dev), dcmipp);
	if (ret) {
		dev_err(&pdev->dev, "Unable to request irq %d\n", irq);
		return ret;
	}

	/* Reset device */
	ret = reset_control_assert(dcmipp->rstc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to assert the reset line\n");
		return ret;
	}

	usleep_range(3000, 5000);

	ret = reset_control_deassert(dcmipp->rstc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to deassert the reset line\n");
		return ret;
	}

	kclk = devm_clk_get(&pdev->dev, "kclk");
	if (IS_ERR(kclk))
		return dev_err_probe(&pdev->dev, PTR_ERR(kclk),
				     "Unable to get kclk\n");
	dcmipp->kclk = kclk;

	/* Create platform_device for each entity in the topology */
	dcmipp->subdevs = devm_kcalloc(&pdev->dev, dcmipp->pipe_cfg->num_ents,
				       sizeof(*dcmipp->subdevs), GFP_KERNEL);
	if (!dcmipp->subdevs)
		return -ENOMEM;

	comp_match = dcmipp_add_subdevs(dcmipp);
	if (IS_ERR(comp_match))
		return PTR_ERR(comp_match);

	/* Link the media device within the v4l2_device */
	dcmipp->v4l2_dev.mdev = &dcmipp->mdev;

	/* Initialize media device */
	strscpy(dcmipp->mdev.model, DCMIPP_MDEV_MODEL_NAME,
		sizeof(dcmipp->mdev.model));
	snprintf(dcmipp->mdev.bus_info, sizeof(dcmipp->mdev.bus_info),
		 "platform:%s", DCMIPP_PDEV_NAME);
	dcmipp->mdev.dev = &pdev->dev;
	media_device_init(&dcmipp->mdev);

	/* Add self to the component system */
	ret = component_master_add_with_match(&pdev->dev, &dcmipp_comp_ops,
					      comp_match);
	if (ret) {
		media_device_cleanup(&dcmipp->mdev);
		dcmipp_rm_subdevs(dcmipp);
		return ret;
	}

	pm_runtime_enable(dcmipp->dev);

	dev_info(&pdev->dev, "Probe done");

	return 0;
}

static int dcmipp_remove(struct platform_device *pdev)
{
	struct dcmipp_device *dcmipp = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);

	component_master_del(&pdev->dev, &dcmipp_comp_ops);
	dcmipp_rm_subdevs(dcmipp);

	return 0;
}

static __maybe_unused int dcmipp_runtime_suspend(struct device *dev)
{
	struct dcmipp_device *dcmipp = dev_get_drvdata(dev);

	clk_disable_unprepare(dcmipp->kclk);

	return 0;
}

static __maybe_unused int dcmipp_runtime_resume(struct device *dev)
{
	struct dcmipp_device *dcmipp = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(dcmipp->kclk);
	if (ret)
		dev_err(dev, "%s: Failed to prepare_enable k clock\n", __func__);

	return ret;
}

static __maybe_unused int dcmipp_suspend(struct device *dev)
{
	/* disable clock */
	pm_runtime_force_suspend(dev);

	/* change pinctrl state */
	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static __maybe_unused int dcmipp_resume(struct device *dev)
{
	/* restore pinctl default state */
	pinctrl_pm_select_default_state(dev);

	/* clock enable */
	pm_runtime_force_resume(dev);

	return 0;
}

static const struct dev_pm_ops dcmipp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dcmipp_suspend, dcmipp_resume)
	SET_RUNTIME_PM_OPS(dcmipp_runtime_suspend,
			   dcmipp_runtime_resume, NULL)
};

static struct platform_driver dcmipp_pdrv = {
	.probe		= dcmipp_probe,
	.remove		= dcmipp_remove,
	.driver		= {
		.name	= DCMIPP_PDEV_NAME,
		.of_match_table = of_match_ptr(dcmipp_of_match),
		.pm = &dcmipp_pm_ops,
	},
};

module_platform_driver(dcmipp_pdrv);

MODULE_AUTHOR("Hugues Fruchet <hugues.fruchet@foss.st.com>");
MODULE_AUTHOR("Alain Volmat <alain.volmat@foss.st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 Digital Camera Memory Interface with Pixel Processor driver");
MODULE_LICENSE("GPL");
