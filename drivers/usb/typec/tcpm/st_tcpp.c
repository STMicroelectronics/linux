// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics USB-C PD protection TCPP02/03 driver
 *
 * Copyright (C) 2026 STMicroelectronics
 */

#include <linux/i2c.h>
#include <linux/of_irq.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/usb/pd.h>
#include <linux/usb/tcpm.h>
#include "st_tcpp.h"

struct st_tcpp {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *enable_gpio;
	struct workqueue_struct *wq;
	struct delayed_work vbus_work;
	unsigned long delay;
	bool vbus;
	tcpp_vbus_cb_t vbus_cb;
	/* lock for sharing states with stm32_ucpd */
	struct mutex lock;
	void *vbus_cb_data;
};

#define TCPP_CR0 0
#define TCPP_SR1 1
#define TCPP_SR2 2

/* writing register CR0 */
#define TCPP_VCONN	GENMASK(1, 0)
#define TCPP_V1		BIT(0)
#define TCPP_V2		BIT(1)
#define TCPP_GD		GENMASK(3, 2)
#define TCPP_GDP	BIT(2)		/* Provider Gate */
#define TCPP_GDP_OPEN	0
#define TCPP_GDP_CLOSED	1
#define TCPP_GDC	BIT(3)		/* Consumer Gate */
#define TCPP_GDC_OPEN	1
#define TCPP_GDC_CLOSED 0

#define TCPP_PM       GENMASK(5, 4)
#define PM_HIBERNATE  0
#define PM_NORMAL     1
#define PM_LP	      2
#define TCPP_VBUSD    BIT(6)

/* reading register SR1 */
#define TCPP_VBUS_OK  BIT(5)

#define VBUS_POLL_INTERVAL_MS	10

int tcpp_register_vbus_notifier(struct device *dev, tcpp_vbus_cb_t cb, void *data)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st_tcpp *tcpp = i2c_get_clientdata(client);

	tcpp->vbus_cb = cb;
	tcpp->vbus_cb_data = data;

	return 0;
}
EXPORT_SYMBOL_GPL(tcpp_register_vbus_notifier);

void tcpp_unregister_vbus_notifier(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st_tcpp *tcpp = i2c_get_clientdata(client);

	tcpp->vbus_cb = NULL;
	tcpp->vbus_cb_data = NULL;
}
EXPORT_SYMBOL_GPL(tcpp_unregister_vbus_notifier);

static int tcpp_update_cr0_bits(struct st_tcpp *tcpp, u32 msk, u32 val)
{
	u32 sr1;
	int ret;

	ret = regmap_read(tcpp->regmap, TCPP_SR1, &sr1);
	if (ret)
		dev_err(tcpp->dev, "%s: Unable to read SR1: %d\n", __func__, ret);

	sr1 &= ~msk;
	sr1 |= val;

	ret = regmap_write(tcpp->regmap, TCPP_CR0, sr1);
	if (ret)
		dev_err(tcpp->dev, "%s: Unable to update CR0: %d\n", __func__, ret);

	return ret;
}

static int tcpp_set_pm(struct st_tcpp *tcpp, int pm)
{
	dev_dbg(tcpp->dev, "%s: set %s mode\n", __func__,
		pm == PM_NORMAL ? "NORMAL" :
		pm == PM_LP ? "LP" :
		pm == PM_HIBERNATE ? "HIBERNATE" : "NA");

	return tcpp_update_cr0_bits(tcpp, TCPP_PM, FIELD_PREP(TCPP_PM, pm));
}

int tcpp_init(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st_tcpp *tcpp = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&tcpp->lock);
	gpiod_set_value_cansleep(tcpp->enable_gpio, true);

	ret = tcpp_set_pm(tcpp, PM_LP);
	if (ret) {
		gpiod_set_value_cansleep(tcpp->enable_gpio, false);
		goto err;
	}

	enable_irq(client->irq);

err:
	mutex_unlock(&tcpp->lock);

	return ret;
}
EXPORT_SYMBOL(tcpp_init);

void tcpp_deinit(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st_tcpp *tcpp = i2c_get_clientdata(client);

	disable_irq(client->irq);
	/* vbus_work may take the lock : cancel it without holding the mutex */
	cancel_delayed_work_sync(&tcpp->vbus_work);

	mutex_lock(&tcpp->lock);
	tcpp_set_pm(tcpp, PM_HIBERNATE);
	gpiod_set_value_cansleep(tcpp->enable_gpio, false);
	tcpp->vbus = false;

	mutex_unlock(&tcpp->lock);
}
EXPORT_SYMBOL(tcpp_deinit);

int tcpp_get_vbus(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st_tcpp *tcpp = i2c_get_clientdata(client);
	u32 sr2;
	int ret;

	mutex_lock(&tcpp->lock);
	ret = regmap_read(tcpp->regmap, TCPP_SR2, &sr2);
	if (ret) {
		dev_err(tcpp->dev, "Unable to get SR2: %d\n", ret);
		goto err;
	}

	ret = !!(sr2 & TCPP_VBUS_OK);

err:
	mutex_unlock(&tcpp->lock);

	return ret;
}
EXPORT_SYMBOL(tcpp_get_vbus);

int tcpp_set_vbus(struct device *dev, bool on, bool charge)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st_tcpp *tcpp = i2c_get_clientdata(client);
	bool gdp, gdc;
	int ret;

	dev_dbg(dev, "provider: %s, consumer: %s\n", str_enable_disable(on),
		str_enable_disable(charge));

	if (on) {
		/* Set to normal mode to allow provider gate control via i2c */
		ret = tcpp_set_pm(tcpp, PM_NORMAL);
		if (ret)
			return ret;
	}

	gdp = on ? TCPP_GDP_CLOSED : TCPP_GDP_OPEN;
	gdc = charge ? TCPP_GDC_CLOSED : TCPP_GDC_OPEN;

	ret = tcpp_update_cr0_bits(tcpp, TCPP_GD,
				   FIELD_PREP(TCPP_GDP, gdp) | FIELD_PREP(TCPP_GDC, gdc));
	if (ret)
		return ret;

	/* Already done (switched ON or OFF) */
	if ((on || charge) == tcpp->vbus)
		return 0;

	/*
	 * Not getting IRQ on Vbus change in NORMAL mode (only failures).
	 * Need to check Vbus state change to notify the tcpm state machine,
	 * e.g. either vbus has been en(dis)abled as provider or consumer,
	 * and vbus is in the expected en(dis)abled state (read it by i2c).
	 */
	if (on)
		tcpp->delay = jiffies + msecs_to_jiffies(PD_T_SRC_TURN_ON);
	else
		tcpp->delay = jiffies + msecs_to_jiffies(PD_T_SAFE_0V);

	mod_delayed_work(tcpp->wq, &tcpp->vbus_work, 0);

	return 0;
}
EXPORT_SYMBOL(tcpp_set_vbus);

int tcpp_set_vconn(struct device *dev, bool on, enum typec_cc_polarity polarity)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st_tcpp *tcpp = i2c_get_clientdata(client);
	u32 vconn = 0;
	int ret;

	if (on) {
		/* Set to normal mode to allow Vconn control via i2c */
		ret = tcpp_set_pm(tcpp, PM_NORMAL);
		if (ret)
			return ret;

		if (polarity == TYPEC_POLARITY_CC1)
			vconn = TCPP_V2;
		else
			vconn = TCPP_V1;
	}

	dev_dbg(dev, "%s: set vconn 0x%x\n", __func__, vconn);

	return tcpp_update_cr0_bits(tcpp, TCPP_VCONN, vconn);
}
EXPORT_SYMBOL(tcpp_set_vconn);

int tcpp_attach(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st_tcpp *tcpp = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&tcpp->lock);
	ret = tcpp_set_pm(tcpp, PM_NORMAL);
	mutex_unlock(&tcpp->lock);

	return ret;
}
EXPORT_SYMBOL(tcpp_attach);

int tcpp_detach(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st_tcpp *tcpp = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&tcpp->lock);
	ret = tcpp_set_pm(tcpp, PM_LP);
	mutex_unlock(&tcpp->lock);
	if (ret)
		return ret;

	/* Already OFF ? No need to poll. */
	if (!tcpp->vbus)
		return 0;

	/*
	 * Earlier in NORMAL mode, don't get IRQ on vbus change (only failures),
	 * Check Vbus upon detach, to notify Vbus off.
	 */
	tcpp->delay = jiffies + msecs_to_jiffies(PD_T_SAFE_0V);
	mod_delayed_work(tcpp->wq, &tcpp->vbus_work, 0);

	return ret;
}
EXPORT_SYMBOL(tcpp_detach);

static void tcpp_vbus_poll_work(struct work_struct *work)
{
	struct st_tcpp *tcpp = container_of(work, struct st_tcpp, vbus_work.work);
	bool vbus_on = !!tcpp_get_vbus(tcpp->dev);

	dev_dbg(tcpp->dev, "%s vbus: %s\n", __func__, str_on_off(vbus_on));

	if (tcpp->vbus != vbus_on) {
		mutex_lock(&tcpp->lock);
		if (tcpp->vbus_cb)
			tcpp->vbus_cb(tcpp->vbus_cb_data, vbus_on);
		mutex_unlock(&tcpp->lock);
		tcpp->vbus = vbus_on;
		return;
	}

	if (time_is_after_jiffies(tcpp->delay))
		mod_delayed_work(tcpp->wq, &tcpp->vbus_work,
				 msecs_to_jiffies(VBUS_POLL_INTERVAL_MS));
}

static irqreturn_t tcpp_irq_thread(int irq, void *data)
{
	struct st_tcpp *tcpp = data;

	mod_delayed_work(tcpp->wq, &tcpp->vbus_work, 0);

	return IRQ_HANDLED;
}

static const struct regmap_config tcpp_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int tcpp_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct st_tcpp *tcpp;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	tcpp = devm_kzalloc(dev, sizeof(*tcpp), GFP_KERNEL);
	if (!tcpp)
		return -ENOMEM;

	mutex_init(&tcpp->lock);

	tcpp->dev = dev;
	tcpp->regmap = devm_regmap_init_i2c(client, &tcpp_regmap);

	tcpp->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(tcpp->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(tcpp->enable_gpio),
				     "Failed to request TCPP enable GPIO\n");

	irq_set_status_flags(client->irq, IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, client->irq, NULL, tcpp_irq_thread,
					IRQF_ONESHOT, "tcpp", tcpp);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ\n");

	tcpp->wq = alloc_ordered_workqueue(dev_name(tcpp->dev), 0);
	if (!tcpp->wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&tcpp->vbus_work, tcpp_vbus_poll_work);

	/*
	 * clientdata is used by stm32_ucpd_probe() to defer ucpd probing
	 * until the mutex, GPIO, and IRQ have been initialized. They
	 * are used by tcpp_init(), which is called by tcpm_register_port().
	 */
	i2c_set_clientdata(client, tcpp);

	return 0;
}

static void tcpp_remove(struct i2c_client *client)
{
	struct st_tcpp *tcpp = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&tcpp->vbus_work);
	destroy_workqueue(tcpp->wq);
}

static int tcpp_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st_tcpp *tcpp = i2c_get_clientdata(client);

	flush_workqueue(tcpp->wq);
	disable_irq(client->irq);

	return 0;
}

static int tcpp_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	enable_irq(client->irq);

	return 0;
}

static const struct dev_pm_ops tcpp_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(tcpp_suspend, tcpp_resume)
};

static const struct i2c_device_id tcpp_id[] = {
	{ "tcpp", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, tcpp_id);

static const struct of_device_id stm_tcpp_match_table[] = {
	{ .compatible = "st,tcpp03" },
	{},
};
MODULE_DEVICE_TABLE(of, stm_tcpp_match_table);

static struct i2c_driver tcpp_driver = {
	.driver = {
		.name = "tcpp",
		.pm = &tcpp_pm_ops,
		.of_match_table = stm_tcpp_match_table,
	},
	.probe = tcpp_i2c_probe,
	.remove = tcpp_remove,
	.id_table = tcpp_id,
};
module_i2c_driver(tcpp_driver);

MODULE_DESCRIPTION("TCPP02/03 USB Type-C PD protection driver");
MODULE_AUTHOR("Christian Bruel <christian.bruel@foss.st.com>");
MODULE_LICENSE("GPL");
