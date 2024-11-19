// SPDX-License-Identifier: GPL-2.0-only
/*
 * Type-C Driver with ADC Monitoring and Power Profiles
 *
 * This driver monitors the Type-C connector using ADC channels to determine
 * the cable orientation and VBUS current. It supports power supply properties
 * and handles debounce logic for state changes.
 *
 * Copyright (C) 2025 STMicroelectronics
 * Author: Ram Dayal <ram.dayal@st.com>
 */

#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_altmode.h>
#include <linux/workqueue.h>

#define VBUS_CURRENT_MAX	3000
#define VBUS_CURRENT_500MA	500
#define VBUS_CURRENT_1500MA	1500
#define VBUS_CURRENT_3000MA	3000

#define HRTIMER_DELAY_MS	10
#define DEBOUNCE_DELAY_MS	10

struct typec_adc_driver_data {
	struct device *dev;
	struct iio_channel *cc1_channel;
	struct iio_channel *cc2_channel;
	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	struct hrtimer timer;
	struct hrtimer debounce_timer;
	struct typec_port *port;
	struct typec_capability cap;
	struct work_struct work;
	struct work_struct debounce_work;
	struct workqueue_struct *wq;
	enum typec_pwr_opmode opmode;
	int current_now;
	bool debounce_work_queued;
};

static enum power_supply_property typec_adc_power_supply_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

/**
 * typec_adc_power_supply_get_property - Get property of the power supply.
 * @psy: Power supply instance.
 * @psp: Property to get.
 * @val: Value of the property.
 *
 * Return: 0 on success, -EINVAL on invalid property.
 */
static int typec_adc_power_supply_get_property(struct power_supply *psy,
					       enum power_supply_property psp,
					       union power_supply_propval *val)
{
	struct typec_adc_driver_data *data = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = data->current_now * 1000;  /* Convert to uA */
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = VBUS_CURRENT_MAX * 1000;  /* Convert to uA */
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * typec_adc_psy_register - Register the power supply.
 * @data: Driver data.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int typec_adc_psy_register(struct typec_adc_driver_data *data)
{
	struct power_supply_desc *psy_desc = &data->psy_desc;
	struct power_supply_config psy_cfg = {};
	char *psy_name;

	psy_name = devm_kasprintf(data->dev, GFP_KERNEL, "psy-%s",
				  dev_name(data->dev));
	if (!psy_name)
		return -ENOMEM;

	psy_desc->name = psy_name;
	psy_desc->type = POWER_SUPPLY_TYPE_USB;
	psy_desc->usb_types = BIT(POWER_SUPPLY_USB_TYPE_C);
	psy_desc->properties = typec_adc_power_supply_props;
	psy_desc->num_properties = ARRAY_SIZE(typec_adc_power_supply_props);
	psy_desc->get_property = typec_adc_power_supply_get_property;

	psy_cfg.drv_data = data;
	data->psy = devm_power_supply_register(data->dev, psy_desc, &psy_cfg);

	if (IS_ERR(data->psy))
		return PTR_ERR(data->psy);

	return 0;
}

/**
 * typec_adc_detect_current_now - Detect the current VBUS current.
 * @data: Driver data.
 * @cc1_voltage: Voltage on CC1 line.
 * @cc2_voltage: Voltage on CC2 line.
 *
 * Return: Detected VBUS current in mA.
 */
static int typec_adc_detect_current_now(struct typec_adc_driver_data *data,
					int cc1_voltage, int cc2_voltage)
{
	int current_now_mA = 0, cc_voltage = 0;

	if (cc1_voltage < 200 && cc2_voltage < 200) {
		dev_dbg(data->dev, "Error: Both CC lines unattached\n");
		return 0;
	} else if (cc1_voltage >= 200 && cc2_voltage >= 200) {
		dev_err(data->dev, "Error: Both CC lines attached\n");
		return 0;
	} else if (cc1_voltage >= 200 && cc2_voltage < 200) {
		dev_dbg(data->dev, "Cable Orientation: A (CC1 connected)\n");
		cc_voltage = cc1_voltage;
	} else if (cc2_voltage >= 200 && cc1_voltage < 200) {
		dev_dbg(data->dev, "Cable Orientation: B (CC2 connected)\n");
		cc_voltage = cc2_voltage;
	}

	/* Now detect max current */
	if (cc_voltage >= 200 && cc_voltage < 660) {
		current_now_mA = VBUS_CURRENT_500MA;
		data->opmode = TYPEC_PWR_MODE_USB;
		dev_dbg(data->dev, "Connected to 500mA source\n");
	} else if (cc_voltage >= 660 && cc_voltage < 1230) {
		current_now_mA = VBUS_CURRENT_1500MA;
		data->opmode = TYPEC_PWR_MODE_1_5A;
		dev_dbg(data->dev, "Connected to 1500mA source\n");
	} else if (cc_voltage >= 1230 && cc_voltage < 2150) {
		current_now_mA = VBUS_CURRENT_3000MA;
		data->opmode = TYPEC_PWR_MODE_3_0A;
		dev_dbg(data->dev, "Connected to 3000mA source\n");
	} else {
		dev_err(data->dev, "Unexpected CC voltage\n");
	}

	return current_now_mA;
}

/**
 * typec_adc_detect_cable_orientation_and_current_now - Detect cable orientation and VBUS current.
 * @data: Driver data.
 */
static int typec_adc_detect_cable_orientation_and_current_now(struct typec_adc_driver_data *data)
{
	int cc1_voltage = 0, cc2_voltage = 0;
	int ret;

	ret = iio_read_channel_processed(data->cc1_channel, &cc1_voltage);
	if (ret < 0) {
		dev_err(data->dev, "Failed to read CC1 voltage: %d\n", ret);
		return 0;
	}

	ret = iio_read_channel_processed(data->cc2_channel, &cc2_voltage);
	if (ret < 0) {
		dev_err(data->dev, "Failed to read CC2 voltage: %d\n", ret);
		return 0;
	}

	dev_dbg(data->dev, "CC1 Voltage: %d mV\n", cc1_voltage);
	dev_dbg(data->dev, "CC2 Voltage: %d mV\n", cc2_voltage);

	return typec_adc_detect_current_now(data, cc1_voltage, cc2_voltage);
}

/**
 * typec_adc_work - Work function to detect cable orientation and VBUS current.
 * @work: Work structure.
 */
static void typec_adc_work(struct work_struct *work)
{
	struct typec_adc_driver_data *data = container_of(work, struct typec_adc_driver_data,
							  work);
	int current_now;

	current_now = typec_adc_detect_cable_orientation_and_current_now(data);

	/* Check if the power rating has changed */
	if (current_now != data->current_now) {
		/*
		 * It may be possible that the timer is not active after it has queued the work.
		 * Start the debounce timer only if it is not already running and the debounce
		 * work is not already queued. This ensures that we do not start multiple
		 * instances of the debounce timer or queue multiple debounce work items.
		 */
		if (!hrtimer_active(&data->debounce_timer) && !data->debounce_work_queued)
			hrtimer_start(&data->debounce_timer, ms_to_ktime(DEBOUNCE_DELAY_MS),
				      HRTIMER_MODE_REL);
	}
}

/**
 * typec_adc_debounce_work - Work function to handle debounce logic.
 * @work: Work structure.
 */
static void typec_adc_debounce_work(struct work_struct *work)
{
	struct typec_adc_driver_data *data = container_of(work, struct typec_adc_driver_data,
							  debounce_work);
	int current_now;

	current_now = typec_adc_detect_cable_orientation_and_current_now(data);

	/* Check if the power rating is still different */
	if (current_now != data->current_now) {
		power_supply_changed(data->psy);
		dev_dbg(data->dev, "Power supply current changed.\n");

		if (current_now)
			typec_set_pwr_opmode(data->port, data->opmode);
		else
			typec_set_pwr_opmode(data->port, TYPEC_PWR_MODE_USB);

		data->current_now = current_now;
	}

	data->debounce_work_queued = false;
}

/**
 * typec_adc_debounce_timer_callback - Debounce timer callback.
 * @timer: High-resolution timer.
 *
 * Return: HRTIMER_NORESTART to stop the timer.
 */
static enum hrtimer_restart typec_adc_debounce_timer_callback(struct hrtimer *timer)
{
	struct typec_adc_driver_data *data = container_of(timer, struct typec_adc_driver_data,
							  debounce_timer);

	data->debounce_work_queued = true;

	queue_work(data->wq, &data->debounce_work);

	return HRTIMER_NORESTART;
}

/**
 * typec_adc_timer_callback - Timer callback to schedule work.
 * @timer: High-resolution timer.
 *
 * Return: HRTIMER_RESTART to restart the timer.
 */
static enum hrtimer_restart typec_adc_timer_callback(struct hrtimer *timer)
{
	struct typec_adc_driver_data *data = container_of(timer, struct typec_adc_driver_data,
							  timer);

	queue_work(data->wq, &data->work);

	/* Restart the timer */
	hrtimer_forward_now(timer, ms_to_ktime(HRTIMER_DELAY_MS));

	return HRTIMER_RESTART;
}

/**
 * typec_adc_driver_probe - Probe function for the platform driver.
 * @pdev: Platform device.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int typec_adc_driver_probe(struct platform_device *pdev)
{
	struct typec_adc_driver_data *data;
	struct device *dev = &pdev->dev;
	struct fwnode_handle *fwnode;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = &pdev->dev;

	data->cc1_channel = devm_iio_channel_get(dev, "cc1");
	if (IS_ERR(data->cc1_channel))
		return dev_err_probe(dev, PTR_ERR(data->cc1_channel),
				     "Failed to get IIO channel for CC1");

	data->cc2_channel = devm_iio_channel_get(dev, "cc2");
	if (IS_ERR(data->cc2_channel))
		return dev_err_probe(dev, PTR_ERR(data->cc2_channel),
				     "Failed to get IIO channel for CC2");

	ret = typec_adc_psy_register(data);
	if (ret) {
		dev_err(dev, "Failed to register power supply %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, data);

	/* Initialize and start the high-resolution timer */
	hrtimer_setup(&data->timer, typec_adc_timer_callback, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	/* Initialize the debounce timer */
	hrtimer_setup(&data->debounce_timer, typec_adc_debounce_timer_callback, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL);

	/* Initialize the high-priority workqueue */
	data->wq = create_workqueue("typec_adc_wq");
	if (!data->wq) {
		dev_err(dev, "Failed to create workqueue\n");
		return -ENOMEM;
	}

	/* Initialize the work structures */
	INIT_WORK(&data->work, typec_adc_work);
	INIT_WORK(&data->debounce_work, typec_adc_debounce_work);

	fwnode = device_get_named_child_node(dev, "connector");
	if (!fwnode) {
		ret = -ENODEV;
		goto freewq;
	}

	/* Get Type-C port capabilities from the device tree */
	ret = typec_get_fw_cap(&data->cap, fwnode);
	if (ret) {
		dev_err(dev, "Failed to get Type-C capabilities from device tree\n");
		goto freewq;
	}

	/* Register the Type-C port */
	data->port = typec_register_port(dev, &data->cap);
	if (IS_ERR(data->port)) {
		dev_err(dev, "Failed to register Type-C port\n");
		goto freewq;
	}

	hrtimer_start(&data->timer, ms_to_ktime(HRTIMER_DELAY_MS), HRTIMER_MODE_REL);

	return 0;

freewq:
	destroy_workqueue(data->wq);

	return ret;
}

static void typec_adc_driver_remove(struct platform_device *pdev)
{
	struct typec_adc_driver_data *data = platform_get_drvdata(pdev);

	/* Cancel the timers */
	hrtimer_cancel(&data->timer);
	hrtimer_cancel(&data->debounce_timer);

	/* Flush and destroy the workqueue to ensure all scheduled work is done */
	flush_workqueue(data->wq);
	destroy_workqueue(data->wq);

	/* Unregister the Type-C port */
	typec_unregister_port(data->port);
}

static int typec_adc_suspend(struct device *dev)
{
	struct typec_adc_driver_data *data = dev_get_drvdata(dev);

	/* Cancel the timers */
	hrtimer_cancel(&data->timer);
	hrtimer_cancel(&data->debounce_timer);

	/* Flush the workqueue to ensure all scheduled work is done */
	flush_workqueue(data->wq);

	/*
	 * Make debounce_work_queued false, It may be possible
	 * that suspend is called while debounce work is queued
	 */
	data->debounce_work_queued = false;

	return 0;
}

static int typec_adc_resume(struct device *dev)
{
	struct typec_adc_driver_data *data = dev_get_drvdata(dev);

	/* Restart the timers */
	hrtimer_start(&data->timer, ms_to_ktime(HRTIMER_DELAY_MS), HRTIMER_MODE_REL);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(typec_adc_pm_ops, typec_adc_suspend, typec_adc_resume);

static const struct of_device_id typec_adc_driver_of_match[] = {
	{ .compatible = "adc-usb-c-connector", },
	{ }
};
MODULE_DEVICE_TABLE(of, typec_adc_driver_of_match);

static struct platform_driver typec_adc_driver = {
	.probe = typec_adc_driver_probe,
	.remove = typec_adc_driver_remove,
	.driver = {
		.name = "typec_adc_driver",
		.of_match_table = typec_adc_driver_of_match,
		.pm = pm_sleep_ptr(&typec_adc_pm_ops),
	},
};

module_platform_driver(typec_adc_driver);

MODULE_AUTHOR("Ram Dayal <ram.dayal@st.com>");
MODULE_DESCRIPTION("Type-C Driver with ADC Monitoring and Power Profiles");
MODULE_LICENSE("GPL");
