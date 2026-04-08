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
#include "st_tcpp.h"

struct st_tcpp {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *enable_gpio;
	tcpp_vbus_cb_t vbus_cb;
	/* lock for sharing states with stm32_ucpd */
	struct mutex lock;
	void *vbus_cb_data;
};

#define TCPP_CR0 0
#define TCPP_SR1 1
#define TCPP_SR2 2

/* writing register CR0 */
#define TCPP_PM       GENMASK(5, 4)
#define PM_HIBERNATE  0
#define PM_NORMAL     1
#define PM_LP	      2
#define TCPP_VBUSD    BIT(6)

/* reading register SR1 */
#define TCPP_VBUS_OK  BIT(5)

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

static int tcpp_set_pm(struct st_tcpp *tcpp, int pm)
{
	u32 sr1;
	int ret;

	ret = regmap_read(tcpp->regmap, TCPP_SR1, &sr1);
	if (ret)
		dev_err(tcpp->dev, "%s: Unable to read SR1: %d\n", __func__, ret);

	sr1 &= ~TCPP_PM;
	sr1 |= FIELD_PREP(TCPP_PM, pm);

	ret = regmap_write(tcpp->regmap, TCPP_CR0, sr1);
	if (ret)
		dev_err(tcpp->dev, "%s: Unable to update CR0: %d\n", __func__, ret);

	return ret;
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

	mutex_lock(&tcpp->lock);

	disable_irq(client->irq);
	tcpp_set_pm(tcpp, PM_HIBERNATE);
	gpiod_set_value_cansleep(tcpp->enable_gpio, false);

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
	dev_dbg(dev, "%s VBUS\n", str_enable_disable(on));

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(tcpp_set_vbus);

int tcpp_attach(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st_tcpp *tcpp = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&tcpp->lock);
	dev_dbg(dev, "%s: set NORMAL mode\n", __func__);

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
	dev_dbg(dev, "%s: set LP mode\n", __func__);

	ret = tcpp_set_pm(tcpp, PM_LP);
	mutex_unlock(&tcpp->lock);

	return ret;
}
EXPORT_SYMBOL(tcpp_detach);

static irqreturn_t tcpp_irq_thread(int irq, void *data)
{
	struct st_tcpp *tcpp = data;

	if (tcpp->vbus_cb)
		tcpp->vbus_cb(tcpp->vbus_cb_data, true);

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

	/*
	 * clientdata is used by stm32_ucpd_probe() to defer ucpd probing
	 * until the mutex, GPIO, and IRQ have been initialized. They
	 * are used by tcpp_init(), which is called by tcpm_register_port().
	 */
	i2c_set_clientdata(client, tcpp);

	return 0;
}

static int tcpp_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

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
	.id_table = tcpp_id,
};
module_i2c_driver(tcpp_driver);

MODULE_DESCRIPTION("TCPP02/03 USB Type-C PD protection driver");
MODULE_AUTHOR("Christian Bruel <christian.bruel@foss.st.com>");
MODULE_LICENSE("GPL");
