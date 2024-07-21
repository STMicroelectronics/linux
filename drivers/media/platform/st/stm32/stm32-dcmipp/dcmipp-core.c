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
#include <media/v4l2-mc.h>

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
#define DCMIPP_CMSR2 (0x3F8)
#define DCMIPP_P0HWCFGR (0x400)
#define DCMIPP_VERR (0xFF4)

struct dcmipp_device {
	/* The platform device */
	struct platform_device		pdev;
	struct device			*dev;

	/* Hardware resources */
	struct reset_control		*rstc;
	void __iomem			*regs;
	struct clk			*mclk;
	struct clk			*kclk;

	/* The pipeline configuration */
	const struct dcmipp_pipeline_config	*pipe_cfg;

	/* The Associated media_device parent */
	struct media_device		mdev;

	/* Internal v4l2 parent device*/
	struct v4l2_device		v4l2_dev;

	/* Entities */
	struct dcmipp_ent_device	**entity;

	struct v4l2_async_notifier	notifier;
};

static inline struct dcmipp_device *notifier_to_dcmipp(struct v4l2_async_notifier *n)
{
	return container_of(n, struct dcmipp_device, notifier);
}

/* Structure which describes individual configuration for each entity */
struct dcmipp_ent_config {
	const char *name;
	struct dcmipp_ent_device *(*init)
		(struct device *dev, const char *entity_name,
		 struct v4l2_device *v4l2_dev, void __iomem *regs);
	void (*release)(struct dcmipp_ent_device *ved);
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
	u32 hw_revision;
};

/* --------------------------------------------------------------------------
 * Topology Configuration
 */

static const struct dcmipp_ent_config stm32mp13_ent_config[] = {
	{
		.name = "dcmipp_input",
		.init = dcmipp_inp_ent_init,
		.release = dcmipp_inp_ent_release,
	},
	{
		.name = "dcmipp_dump_postproc",
		.init = dcmipp_byteproc_ent_init,
		.release = dcmipp_byteproc_ent_release,
	},
	{
		.name = "dcmipp_dump_capture",
		.init = dcmipp_bytecap_ent_init,
		.release = dcmipp_bytecap_ent_release,
	},
};

#define ID_INPUT 0
#define ID_DUMP_BYTEPROC 1
#define ID_DUMP_CAPTURE 2

static const struct dcmipp_ent_link stm32mp13_ent_links[] = {
	DCMIPP_ENT_LINK(ID_INPUT,	  1, ID_DUMP_BYTEPROC, 0,
			MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE),
	DCMIPP_ENT_LINK(ID_DUMP_BYTEPROC, 1, ID_DUMP_CAPTURE,  0,
			MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE),
};

#define DCMIPP_STM32MP13_VERR	0x10
static const struct dcmipp_pipeline_config stm32mp13_pipe_cfg = {
	.ents		= stm32mp13_ent_config,
	.num_ents	= ARRAY_SIZE(stm32mp13_ent_config),
	.links		= stm32mp13_ent_links,
	.num_links	= ARRAY_SIZE(stm32mp13_ent_links),
	.hw_revision	= DCMIPP_STM32MP13_VERR
};

#define	ID_MAIN_ISP 3
#define	ID_MAIN_POSTPROC 4
#define	ID_MAIN_CAPTURE	5
#define	ID_AUX_POSTPROC 6
#define	ID_AUX_CAPTURE 7
#define	ID_ISP_STAT_CAPTURE 8
#define	ID_ISP_PARAMS_OUTPUT 9
static const struct dcmipp_ent_config stm32mp25_ent_config[] = {
	{
		.name = "dcmipp_input",
		.init = dcmipp_inp_ent_init,
		.release = dcmipp_inp_ent_release,
	},
	{
		.name = "dcmipp_dump_postproc",
		.init = dcmipp_byteproc_ent_init,
		.release = dcmipp_byteproc_ent_release,
	},
	{
		.name = "dcmipp_dump_capture",
		.init = dcmipp_bytecap_ent_init,
		.release = dcmipp_bytecap_ent_release,
	},
	{
		.name = "dcmipp_main_isp",
		.init = dcmipp_isp_ent_init,
		.release = dcmipp_isp_ent_release,
	},
	{
		.name = "dcmipp_main_postproc",
		.init = dcmipp_pixelproc_ent_init,
		.release = dcmipp_pixelproc_ent_release,
	},
	{
		.name = "dcmipp_main_capture",
		.init = dcmipp_pixelcap_ent_init,
		.release = dcmipp_pixelcap_ent_release,
	},
	{
		.name = "dcmipp_aux_postproc",
		.init = dcmipp_pixelproc_ent_init,
		.release = dcmipp_pixelproc_ent_release,
	},
	{
		.name = "dcmipp_aux_capture",
		.init = dcmipp_pixelcap_ent_init,
		.release = dcmipp_pixelcap_ent_release,
	},
	{
		.name = "dcmipp_main_isp_stat_capture",
		.init = dcmipp_statcap_ent_init,
		.release = dcmipp_statcap_ent_release,
	},
	{
		.name = "dcmipp_main_isp_params_output",
		.init = dcmipp_isp_params_ent_init,
		.release = dcmipp_isp_params_ent_release,
	},
};

static const struct dcmipp_ent_link stm32mp25_ent_links[] = {
	DCMIPP_ENT_LINK(ID_INPUT,	  1, ID_DUMP_BYTEPROC, 0,
			MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE),
	DCMIPP_ENT_LINK(ID_DUMP_BYTEPROC, 1, ID_DUMP_CAPTURE,  0,
			MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE),
	DCMIPP_ENT_LINK(ID_INPUT,	2, ID_MAIN_ISP,  0,
			MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE),
	DCMIPP_ENT_LINK(ID_MAIN_ISP,	1, ID_MAIN_POSTPROC,  0,
			MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE),
	DCMIPP_ENT_LINK(ID_MAIN_ISP,	1, ID_AUX_POSTPROC,  0, 0),
	DCMIPP_ENT_LINK(ID_MAIN_POSTPROC,	1, ID_MAIN_CAPTURE,  0,
			MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE),
	DCMIPP_ENT_LINK(ID_INPUT,	3, ID_AUX_POSTPROC,  0, 0),
	DCMIPP_ENT_LINK(ID_AUX_POSTPROC,	1, ID_AUX_CAPTURE,  0,
			MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE),
	DCMIPP_ENT_LINK(ID_ISP_PARAMS_OUTPUT,	0, ID_MAIN_ISP,	2,
			MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE),
	DCMIPP_ENT_LINK(ID_MAIN_ISP,	3, ID_ISP_STAT_CAPTURE,  0,
			MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE),
};

#define DCMIPP_STM32MP25_VERR	0x30
static const struct dcmipp_pipeline_config stm32mp25_pipe_cfg = {
	.ents		= stm32mp25_ent_config,
	.num_ents	= ARRAY_SIZE(stm32mp25_ent_config),
	.links		= stm32mp25_ent_links,
	.num_links	= ARRAY_SIZE(stm32mp25_ent_links),
	.hw_revision	= DCMIPP_STM32MP25_VERR
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
		const struct dcmipp_ent_link *link =
			&dcmipp->pipe_cfg->links[i];
		struct dcmipp_ent_device *ved_src =
			dcmipp->entity[link->src_ent];
		struct dcmipp_ent_device *ved_sink =
			dcmipp->entity[link->sink_ent];

		dev_dbg(dcmipp->dev, "Create link \"%s\":%d -> %d:\"%s\" [%s]\n",
			dcmipp->pipe_cfg->ents[link->src_ent].name,
			link->src_pad, link->sink_pad,
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

static int dcmipp_create_subdevs(struct dcmipp_device *dcmipp)
{
	int ret, i;

	/* Call all subdev inits */
	for (i = 0; i < dcmipp->pipe_cfg->num_ents; i++) {
		const char *name = dcmipp->pipe_cfg->ents[i].name;

		dev_dbg(dcmipp->dev, "add subdev %s\n", name);
		dcmipp->entity[i] =
			dcmipp->pipe_cfg->ents[i].init(dcmipp->dev, name,
						       &dcmipp->v4l2_dev,
						       dcmipp->regs);
		if (IS_ERR(dcmipp->entity[i])) {
			dev_err(dcmipp->dev, "failed to init subdev %s\n",
				name);
			ret = PTR_ERR(dcmipp->entity[i]);
			goto err_init_entity;
		}
	}

	/* Initialize links */
	ret = dcmipp_create_links(dcmipp);
	if (ret)
		goto err_init_entity;

	ret = dcmipp_graph_init(dcmipp);
	if (ret < 0)
		goto err_init_entity;

	return 0;

err_init_entity:
	while (i-- > 0)
		dcmipp->pipe_cfg->ents[i - 1].release(dcmipp->entity[i - 1]);
	return ret;
}

static const struct of_device_id dcmipp_of_match[] = {
	{ .compatible = "st,stm32mp13-dcmipp", .data = &stm32mp13_pipe_cfg},
	{ .compatible = "st,stm32mp25-dcmipp", .data = &stm32mp25_pipe_cfg},
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
		ved = dcmipp->entity[i];
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
	u32 cmsr2;

	/* Centralized read of CMSR2 */
	cmsr2 = reg_read(dcmipp, DCMIPP_CMSR2);

	/* Call irq handler of each entities of pipeline */
	for (i = 0; i < dcmipp->pipe_cfg->num_ents; i++) {
		ved = dcmipp->entity[i];
		ved->cmsr2 = cmsr2;
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

	dev_dbg(dcmipp->dev, "Subdev \"%s\" bound\n", subdev->name);

	/* Link this sub-device to DCMIPP input subdev */
	src_pad = media_entity_get_fwnode_pad(&subdev->entity,
					      subdev->fwnode,
					      MEDIA_PAD_FL_SOURCE);

	ret = media_create_pad_link(&subdev->entity, src_pad,
				    dcmipp->entity[ID_INPUT]->ent, 0,
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

	v4l2_async_nf_init(&dcmipp->notifier);

	asd = v4l2_async_nf_add_fwnode_remote
		(&dcmipp->notifier, of_fwnode_handle(ep),
		 struct v4l2_async_subdev);

	of_node_put(ep);

	if (IS_ERR(asd)) {
		dev_err(dcmipp->dev, "Failed to add fwnode remote subdev\n");
		return PTR_ERR(asd);
	}

	dcmipp->notifier.ops = &dcmipp_graph_notify_ops;

	ret = v4l2_async_nf_register(&dcmipp->v4l2_dev, &dcmipp->notifier);
	if (ret < 0) {
		dev_err(dcmipp->dev, "Failed to register notifier\n");
		v4l2_async_nf_cleanup(&dcmipp->notifier);
		return ret;
	}

	return 0;
}

static const struct media_device_ops dcmipp_media_ops = {
	.link_notify = v4l2_pipeline_link_notify,
};

static int dcmipp_probe(struct platform_device *pdev)
{
	struct dcmipp_device *dcmipp;
	struct resource *res;
	struct clk *kclk, *mclk;
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

	if (!of_device_is_compatible(pdev->dev.of_node, "st,stm32mp13-dcmipp")) {
		mclk = devm_clk_get(&pdev->dev, "mclk");
		if (IS_ERR(mclk))
			return dev_err_probe(&pdev->dev, PTR_ERR(mclk),
					     "Unable to get mclk\n");
		dcmipp->mclk = mclk;
	}

	dcmipp->entity = devm_kcalloc(&pdev->dev, dcmipp->pipe_cfg->num_ents,
				      sizeof(*dcmipp->entity), GFP_KERNEL);
	if (!dcmipp->entity)
		return -ENOMEM;

	/* Register the v4l2 struct */
	ret = v4l2_device_register(&pdev->dev, &dcmipp->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev,
			"v4l2 device register failed (err=%d)\n", ret);
		return ret;
	}

	/* Link the media device within the v4l2_device */
	dcmipp->v4l2_dev.mdev = &dcmipp->mdev;

	/* Initialize media device */
	strscpy(dcmipp->mdev.model, DCMIPP_MDEV_MODEL_NAME,
		sizeof(dcmipp->mdev.model));
	snprintf(dcmipp->mdev.bus_info, sizeof(dcmipp->mdev.bus_info),
		 "platform:%s", DCMIPP_PDEV_NAME);
	dcmipp->mdev.hw_revision = pipe_cfg->hw_revision;
	dcmipp->mdev.dev = &pdev->dev;
	dcmipp->mdev.ops = &dcmipp_media_ops;
	media_device_init(&dcmipp->mdev);

	pm_runtime_enable(dcmipp->dev);

	/* Initialize subdevs */
	ret = dcmipp_create_subdevs(dcmipp);
	if (ret) {
		media_device_cleanup(&dcmipp->mdev);
		v4l2_device_unregister(&dcmipp->v4l2_dev);
		return ret;
	}

	dev_info(&pdev->dev, "Probe done");

	return 0;
}

static int dcmipp_remove(struct platform_device *pdev)
{
	struct dcmipp_device *dcmipp = platform_get_drvdata(pdev);
	unsigned int i;

	pm_runtime_disable(&pdev->dev);

	v4l2_async_nf_unregister(&dcmipp->notifier);
	v4l2_async_nf_cleanup(&dcmipp->notifier);

	for (i = 0; i < dcmipp->pipe_cfg->num_ents; i++)
		dcmipp->pipe_cfg->ents[i].release(dcmipp->entity[i]);

	media_device_unregister(&dcmipp->mdev);
	media_device_cleanup(&dcmipp->mdev);

	v4l2_device_unregister(&dcmipp->v4l2_dev);

	return 0;
}

static __maybe_unused int dcmipp_runtime_suspend(struct device *dev)
{
	struct dcmipp_device *dcmipp = dev_get_drvdata(dev);

	clk_disable_unprepare(dcmipp->kclk);
	clk_disable_unprepare(dcmipp->mclk);

	return 0;
}

static __maybe_unused int dcmipp_runtime_resume(struct device *dev)
{
	struct dcmipp_device *dcmipp = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(dcmipp->mclk);
	if (ret)
		dev_err(dev, "%s: Failed to prepare_enable clock\n", __func__);

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
