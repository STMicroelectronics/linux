// SPDX-License-Identifier: GPL-2.0
/*
 * STM32 Low-Power Timer Trigger driver
 *
 * Copyright (C) STMicroelectronics 2017
 *
 * Author: Fabrice Gasnier <fabrice.gasnier@st.com>.
 *
 * Inspired by Benjamin Gaignard's stm32-timer-trigger driver
 */

#include <linux/iio/timer/stm32-lptim-trigger.h>
#include <linux/mfd/stm32-lptimer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define MAX_TRIGGERS 2

struct stm32_lptim_cfg {
	const char * const (*triggers)[MAX_TRIGGERS];
	unsigned int nb_triggers;
};

/* List Low-Power Timer triggers for H7, MP13, MP15 */
static const char * const stm32_lptim_triggers[][MAX_TRIGGERS] = {
	{ LPTIM1_OUT,},
	{ LPTIM2_OUT,},
	{ LPTIM3_OUT,},
};

/* List Low-Power Timer triggers for STM32MP25 */
static const char * const stm32mp25_lptim_triggers[][MAX_TRIGGERS] = {
	{ LPTIM1_CH1, LPTIM1_CH2, },
	{ LPTIM2_CH1, LPTIM2_CH2, },
	{ LPTIM3_CH1,},
	{ LPTIM4_CH1,},
	{ LPTIM5_OUT,},
};

static const struct stm32_lptim_cfg stm32mp15_lptim_cfg = {
	.triggers = stm32_lptim_triggers,
	.nb_triggers = ARRAY_SIZE(stm32_lptim_triggers),
};

static const struct stm32_lptim_cfg stm32mp25_lptim_cfg = {
	.triggers = stm32mp25_lptim_triggers,
	.nb_triggers = ARRAY_SIZE(stm32mp25_lptim_triggers),
};

struct stm32_lptim_trigger {
	struct device *dev;
	const char * const *triggers;
	struct list_head tr_list;
};

static int stm32_lptim_validate_device(struct iio_trigger *trig,
				       struct iio_dev *indio_dev)
{
	if (indio_dev->modes & INDIO_HARDWARE_TRIGGERED)
		return 0;

	return -EINVAL;
}

static const struct iio_trigger_ops stm32_lptim_trigger_ops = {
	.validate_device = stm32_lptim_validate_device,
};

/**
 * is_stm32_lptim_trigger
 * @trig: trigger to be checked
 *
 * return true if the trigger is a valid STM32 IIO Low-Power Timer Trigger
 * either return false
 */
bool is_stm32_lptim_trigger(struct iio_trigger *trig)
{
	return (trig->ops == &stm32_lptim_trigger_ops);
}
EXPORT_SYMBOL(is_stm32_lptim_trigger);

static void stm32_lptim_unregister_triggers(struct stm32_lptim_trigger *priv)
{
	struct iio_trigger *tr;

	list_for_each_entry(tr, &priv->tr_list, alloc_list)
		iio_trigger_unregister(tr);
}

static int stm32_lptim_register_triggers(struct stm32_lptim_trigger *priv)
{
	const char * const *cur = priv->triggers;
	int ret;

	INIT_LIST_HEAD(&priv->tr_list);

	while (cur && *cur) {
		struct iio_trigger *trig;

		trig = devm_iio_trigger_alloc(priv->dev, "%s", *cur);
		if  (!trig)
			return -ENOMEM;

		trig->dev.parent = priv->dev->parent;
		trig->ops = &stm32_lptim_trigger_ops;
		iio_trigger_set_drvdata(trig, priv);

		ret = iio_trigger_register(trig);
		if (ret)
			return ret;

		list_add_tail(&trig->alloc_list, &priv->tr_list);
		cur++;
	}

	return 0;
}

static int stm32_lptim_trigger_probe(struct platform_device *pdev)
{
	struct stm32_lptim_trigger *priv;
	struct stm32_lptim_cfg const *lptim_cfg;
	u32 index;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (device_property_read_u32(&pdev->dev, "reg", &index))
		return -EINVAL;

	lptim_cfg = device_get_match_data(&pdev->dev);

	if (index >= lptim_cfg->nb_triggers)
		return -EINVAL;

	priv->dev = &pdev->dev;
	priv->triggers = lptim_cfg->triggers[index];

	ret = stm32_lptim_register_triggers(priv);
	if (ret) {
		stm32_lptim_unregister_triggers(priv);
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int stm32_lptim_trigger_remove(struct platform_device *pdev)
{
	struct stm32_lptim_trigger *priv = platform_get_drvdata(pdev);

	stm32_lptim_unregister_triggers(priv);

	return 0;
}

static const struct of_device_id stm32_lptim_trig_of_match[] = {
	{ .compatible = "st,stm32-lptimer-trigger", .data = (void *)&stm32mp15_lptim_cfg },
	{ .compatible = "st,stm32mp25-lptimer-trigger", .data = (void *)&stm32mp25_lptim_cfg},
	{},
};
MODULE_DEVICE_TABLE(of, stm32_lptim_trig_of_match);

static struct platform_driver stm32_lptim_trigger_driver = {
	.probe = stm32_lptim_trigger_probe,
	.remove = stm32_lptim_trigger_remove,
	.driver = {
		.name = "stm32-lptimer-trigger",
		.of_match_table = stm32_lptim_trig_of_match,
	},
};
module_platform_driver(stm32_lptim_trigger_driver);

MODULE_AUTHOR("Fabrice Gasnier <fabrice.gasnier@st.com>");
MODULE_ALIAS("platform:stm32-lptimer-trigger");
MODULE_DESCRIPTION("STMicroelectronics STM32 LPTIM trigger driver");
MODULE_LICENSE("GPL v2");
