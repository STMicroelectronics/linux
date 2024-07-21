// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Virtual Driver for RPMSG based I2C bus controller
 *
 * Copyright (c) 2023 STMicroelectronics.
 *
 */

#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

/* rpmsg_i2c_msg result flag */
#define RPMSG_I2C_ACK	0x01
#define RPMSG_I2C_NACK	0x02

static LIST_HEAD(rpmsg_i2c_list);   /* i2c adapter instances list */
static DEFINE_MUTEX(rpmsg_i2c_lock);  /* protect adapter list */

/**
 * struct rpmsg_i2c_msg - client specific data
 * @count: number of bytes to be transferred
 * @addr: 8-bit slave addr, including r/w bit
 * @result: used by the slave as an acknowledge
 * @reserved: header padding for 32-bit alignment
 * @buf: data buffer
 */
struct rpmsg_i2c_msg {
	u32 count;
	u8 addr;
	u8 result;
	u8 reserved[2];
	u8 buf[0];
} __packed;

/**
 * struct rpmsg_i2c_dev - device private data
 * @is_read: indicate that driver waiting a rd callback
 * @is_write: indicate that driver waiting a wr callback
 * @complete: completion for waiting data
 * @adap: i2c adapter handler
 * @list: rpmsg_i2c adapter list
 * @rpdrv: rpmsg driver handler
 * @dev_id: rpmsg driver device id
 * @dev: rpmsg i2c device declared in the device tree
 * @rpdev: rpmsg device handler
 * @msg:  i2c client specific data
 * @i2cscan_work: to fix context issue of rpmsg_register_device
 * call on context of nameservicing notification.
 */
struct rpmsg_i2c_dev {
	bool is_read;
	bool is_write;
	struct completion complete;
	struct i2c_adapter *adap;

	struct list_head list; /* adapter device list */
	struct rpmsg_driver rpdrv;
	struct rpmsg_device_id dev_id[2];
	struct device *dev;

	struct rpmsg_device *rpdev;
	struct rpmsg_i2c_msg *msg;

	struct work_struct i2cscan_work;
};

static int rpmsg_i2c_cb(struct rpmsg_device *rpdev, void *data, int len, void *priv, u32 src)
{
	struct rpmsg_i2c_dev *ri2c_dev = dev_get_drvdata(&rpdev->dev);
	struct rpmsg_i2c_msg *msg = ri2c_dev->msg;

	msg->result = 0;

	dev_dbg(&rpdev->dev, "incoming msg (src: 0x%x), R:%d W:%d R:0x%x (len=%d)\n", src,
		ri2c_dev->is_read, ri2c_dev->is_write, msg->result, len);

	if (ri2c_dev->is_read || ri2c_dev->is_write) {
		memcpy(msg, data, len);

		if (ri2c_dev->is_read || msg->result)
			complete(&ri2c_dev->complete);
		else
			dev_err(&rpdev->dev, "Expecting ack or nack\n");
	} else {
		/*
		 * Invalid message. As it not straightforward to determine the reason, or
		 * to propagate the error, just discard the message and display an error
		 * message.
		 */
		dev_err(&rpdev->dev, "unexpected message\n");
	}

	return 0;
}

static int rpmsg_i2c_write(struct rpmsg_i2c_dev *ri2c_dev, struct i2c_msg *msg)
{
	struct rpmsg_device *rpdev = ri2c_dev->rpdev;
	struct rpmsg_i2c_msg *r_msg = ri2c_dev->msg;
	unsigned long time_left;
	int msg_size = sizeof(struct rpmsg_i2c_msg) + msg->len;
	int ret;

	if (!rpdev)
		return -EIO;

	if (msg_size > rpmsg_get_mtu(rpdev->ept))
		return -EINVAL;

	r_msg->addr = i2c_8bit_addr_from_msg(msg);
	r_msg->count = msg->len;
	r_msg->result = 0;
	memcpy(r_msg->buf, msg->buf, msg->len);

	dev_dbg(&rpdev->dev, "%s: %x (len=%d)", __func__, r_msg->addr, r_msg->count);
	ri2c_dev->is_write = true;
	reinit_completion(&ri2c_dev->complete);

	ret = rpmsg_send(rpdev->ept, r_msg, msg_size);
	if (ret) {
		dev_err(&rpdev->dev, "write: rpmsg_send failed: %d\n", ret);
		goto err_wr;
	}

	time_left = wait_for_completion_timeout(&ri2c_dev->complete, ri2c_dev->adap->timeout);
	if (!time_left)
		ret = -ETIMEDOUT;

	if (r_msg->result & RPMSG_I2C_NACK)
		ret = -ENXIO;

err_wr:
	ri2c_dev->is_write = false;

	return ret;
}

static int rpmsg_i2c_read(struct rpmsg_i2c_dev *ri2c_dev, struct i2c_msg *msg)
{
	struct rpmsg_device *rpdev = ri2c_dev->rpdev;
	struct rpmsg_i2c_msg *r_msg = ri2c_dev->msg;
	unsigned long time_left;
	int msg_size = sizeof(struct rpmsg_i2c_msg);
	int ret;

	if (!rpdev)
		return -EIO;

	r_msg->addr = i2c_8bit_addr_from_msg(msg);
	r_msg->count = msg->len;
	r_msg->result = 0;

	ri2c_dev->is_read = true;
	reinit_completion(&ri2c_dev->complete);

	dev_dbg(&rpdev->dev, "%s: %x (len=%d)", __func__, r_msg->addr, r_msg->count);

	ret = rpmsg_send(rpdev->ept, r_msg, msg_size);
	if (ret) {
		dev_err(&rpdev->dev, "read: rpmsg_send failed: %d\n", ret);
		goto err_rd;
	}

	time_left = wait_for_completion_timeout(&ri2c_dev->complete, ri2c_dev->adap->timeout);
	ri2c_dev->is_read = false;
	if (time_left) {
		if (r_msg->result & RPMSG_I2C_NACK)
			ret = -ENXIO;

		if (msg->len != r_msg->count)
			dev_warn(&rpdev->dev, "Unexpected read size (%u), expected (%u)\n",
				 r_msg->count, msg->len);

		memcpy(msg->buf, r_msg->buf, msg->len);
	} else {
		ret = -ETIMEDOUT;
		goto err_rd;
	}

	return ret;

err_rd:
	ri2c_dev->is_read = false;
	return ret;
}

static int rpmsg_i2c_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[], int num)
{
	struct rpmsg_i2c_dev *ri2c_dev = i2c_get_adapdata(i2c_adap);
	struct i2c_msg *pmsg;
	int err = 0, count = 0, i;
	u32 status;

	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];
		status = pmsg->flags;

		if ((status & I2C_M_RD) != false)
			err = rpmsg_i2c_read(ri2c_dev, pmsg);
		else
			err = rpmsg_i2c_write(ri2c_dev, pmsg);

		if (err < 0)
			break;
		count++;
	}
	return (err < 0) ? err : count;
}

static u32 rpmsg_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm rpmsg_i2c_algo = {
	.master_xfer = rpmsg_i2c_xfer,
	.functionality = rpmsg_i2c_func,
};

static void rpmsg_i2c_scan_work(struct work_struct *ws)
{
	struct rpmsg_i2c_dev *ri2c_dev = container_of(ws, struct rpmsg_i2c_dev, i2cscan_work);
	struct rpmsg_device *rpdev = ri2c_dev->rpdev;
	int ret;

	mutex_lock(&rpmsg_i2c_lock);

	if (!ri2c_dev->adap)
		goto out;

	ret = i2c_add_adapter(ri2c_dev->adap);
	if (ret) {
		dev_err(&rpdev->dev, "failed to add I2C adapter: %d\n", ret);
		goto out;
	}

out:
	mutex_unlock(&rpmsg_i2c_lock);
}

static int rpmsg_i2c_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_i2c_dev *ri2c_dev;
	struct i2c_adapter *adap;
	bool adap_found = false;
	int ret;

	mutex_lock(&rpmsg_i2c_lock);

	list_for_each_entry(ri2c_dev, &rpmsg_i2c_list, list) {
		if (!strcmp(ri2c_dev->dev_id[0].name, rpdev->id.name)) {
			if (ri2c_dev->rpdev) {
				ret = -EBUSY;
				goto out;
			}
			adap_found = true;
			break;
		}
	}

	if (!adap_found) {
		ret = -ENODEV;
		goto out;
	}

	adap = devm_kzalloc(&rpdev->dev, sizeof(*adap), GFP_KERNEL);
	ri2c_dev->msg = devm_kzalloc(&rpdev->dev, rpmsg_get_mtu(rpdev->ept), GFP_KERNEL);
	if (!adap || !ri2c_dev->msg) {
		ret = -ENOMEM;
		goto out;
	}

	i2c_set_adapdata(adap, ri2c_dev);
	ri2c_dev->adap = adap;

	snprintf(adap->name, sizeof(adap->name), "%s-%#x adapter ",
		 rpdev->id.name, rpdev->src);
	adap->owner = THIS_MODULE;
	adap->timeout = msecs_to_jiffies(100); /* 100 ms timeout */
	adap->retries = 0;
	adap->algo = &rpmsg_i2c_algo;
	adap->dev.parent = ri2c_dev->dev;
	adap->dev.of_node = ri2c_dev->dev->of_node;
	init_completion(&ri2c_dev->complete);

	/* match between proc id and service name */
	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
		 rpdev->src, rpdev->dst);

	ri2c_dev->rpdev = rpdev;
	dev_set_drvdata(&rpdev->dev, ri2c_dev);

	mutex_unlock(&rpmsg_i2c_lock);

	/*
	 * The RPMsg driver can be called under interrupt context to ensure that the
	 * I2C protocol (Ack/Nack) is running in interrupt context.
	 * To be sure to call i2c_add_adapter in normal context, schedule a work.
	 */
	schedule_work(&ri2c_dev->i2cscan_work);

	return 0;

out:
	mutex_unlock(&rpmsg_i2c_lock);

	return ret;
}

static void rpmsg_i2c_remove(struct rpmsg_device *rpdev)
{
	struct rpmsg_i2c_dev *ri2c_dev = dev_get_drvdata(&rpdev->dev);

	cancel_work_sync(&ri2c_dev->i2cscan_work);
	if (ri2c_dev->adap)
		i2c_del_adapter(ri2c_dev->adap);

	ri2c_dev->adap = NULL;
	ri2c_dev->rpdev = NULL;
}

static int rpmsg_i2c_platform_probe(struct platform_device *pdev)
{
	struct rpmsg_i2c_dev *ri2c_dev;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const char *dev_id_name;
	int ret;

	ri2c_dev = devm_kzalloc(&pdev->dev, sizeof(*ri2c_dev), GFP_KERNEL);
	if (!ri2c_dev)
		return -ENOMEM;

	ri2c_dev->dev = dev;
	ri2c_dev->rpdev = NULL;
	ri2c_dev->is_read = false;
	ri2c_dev->is_write = false;

	platform_set_drvdata(pdev, ri2c_dev);

	INIT_WORK(&ri2c_dev->i2cscan_work, rpmsg_i2c_scan_work);

	/* Generation of a specific rpmsg driver for this particular instance */
	ret = of_property_read_string(np, "rpmsg,dev-id", &dev_id_name);
	if (ret) {
		dev_err(dev, "Error, proc-id property is missing: %d\n", ret);
		return ret;
	}
	strncpy(ri2c_dev->dev_id[0].name, dev_id_name, RPMSG_NAME_SIZE - 1);

	ri2c_dev->rpdrv.drv.name = "rpmsg_i2c";
	ri2c_dev->rpdrv.id_table = ri2c_dev->dev_id;
	ri2c_dev->rpdrv.probe = rpmsg_i2c_probe;
	ri2c_dev->rpdrv.callback = rpmsg_i2c_cb;
	ri2c_dev->rpdrv.remove = rpmsg_i2c_remove;

	ret = register_rpmsg_driver(&ri2c_dev->rpdrv);
	if (ret)
		return ret;

	/*
	 * Adding the adapter inside a list head to match it later with the
	 * corresponding rpmsg device
	 */
	mutex_lock(&rpmsg_i2c_lock);
	list_add_tail(&ri2c_dev->list, &rpmsg_i2c_list);
	mutex_unlock(&rpmsg_i2c_lock);

	return 0;
}

static int rpmsg_i2c_platform_remove(struct platform_device *pdev)
{
	struct rpmsg_i2c_dev *ri2c_dev = platform_get_drvdata(pdev);

	unregister_rpmsg_driver(&ri2c_dev->rpdrv);
	list_del(&ri2c_dev->list);

	return 0;
}

static const struct of_device_id rpmsg_i2c_dt_ids[] = {
	{ .compatible = "rpmsg,i2c-controller" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rpmsg_i2c_dt_ids);

static struct platform_driver rpmsg_i2c_platform_driver = {
	.driver = {
		.name	= "rpmsg_i2c",
		.of_match_table = rpmsg_i2c_dt_ids,
	},
	.probe		= rpmsg_i2c_platform_probe,
	.remove		= rpmsg_i2c_platform_remove
};

static int __init rpmsg_i2c_init(void)
{
	return platform_driver_register(&rpmsg_i2c_platform_driver);
}

static void __exit rpmsg_i2c_exit(void)
{
	platform_driver_unregister(&rpmsg_i2c_platform_driver);
}

module_init(rpmsg_i2c_init);
module_exit(rpmsg_i2c_exit);

MODULE_AUTHOR("Maxime Mere <maxime.mere@st.com>");
MODULE_DESCRIPTION("Remote processor messaging I2C driver");
MODULE_LICENSE("GPL v2");
