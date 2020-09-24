// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2018 - All Rights Reserved
 * Author: Arnaud Pouliquen <arnaud.pouliquen@st.com> for STMicroelectronics.
 * Derived from the imx-rmpsg and omap-rpmsg implementations.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/virtio.h>

#define MAX_TTY_RPMSG_INDEX	32 /* Should be enough for a while */

static LIST_HEAD(rpmsg_tty_list);   /* tty instances list */
static DEFINE_MUTEX(rpmsg_tty_lock);  /* protect tty list */

static struct tty_driver *rpmsg_tty_driver;
static struct tty_port_operations  rpmsg_tty_port_ops = { };

struct rpmsg_tty_port {
	struct tty_port		*port;	/* TTY port data */
	struct list_head	list;	/* TTY device list */
	u32			id;	/* tty rpmsg index */
	spinlock_t		rx_lock; /* message reception lock */
	struct rpmsg_device	*rpdev;	/* handle rpmsg device */
};

static int rpmsg_tty_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	int space;
	unsigned char *cbuf;
	struct rpmsg_tty_port *cport = dev_get_drvdata(&rpdev->dev);

	/* Flush the recv-ed none-zero data to tty node */
	if (len == 0)
		return -EINVAL;

	dev_dbg(&rpdev->dev, "msg(<- src 0x%x) len %d\n", src, len);

	print_hex_dump_debug(__func__, DUMP_PREFIX_NONE, 16, 1, data, len,
			     true);

	spin_lock(&cport->rx_lock);
	space = tty_prepare_flip_string(cport->port, &cbuf, len);
	if (space <= 0) {
		dev_err(&rpdev->dev, "No memory for tty_prepare_flip_string\n");
		spin_unlock(&cport->rx_lock);
		return -ENOMEM;
	}

	if (space != len)
		dev_warn(&rpdev->dev, "Trunc buffer from %d to %d\n",
			 len, space);

	memcpy(cbuf, data, space);
	tty_flip_buffer_push(cport->port);
	spin_unlock(&cport->rx_lock);

	return 0;
}

static struct rpmsg_tty_port *rpmsg_tty_get(unsigned int index)
{
	struct rpmsg_tty_port *cport;

	mutex_lock(&rpmsg_tty_lock);
	list_for_each_entry(cport, &rpmsg_tty_list, list) {
		if (index == cport->id) {
			mutex_unlock(&rpmsg_tty_lock);
			return cport;
		}
	}
	mutex_unlock(&rpmsg_tty_lock);

	return NULL;
}

static int rpmsg_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct rpmsg_tty_port *cport = rpmsg_tty_get(tty->index);

	if (!cport)
		return -ENODEV;

	return tty_port_install(cport->port, driver, tty);
}

static int rpmsg_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct rpmsg_tty_port *cport = rpmsg_tty_get(tty->index);

	if (!cport)
		return -ENODEV;

	return tty_port_open(tty->port, tty, filp);
}

static void rpmsg_tty_close(struct tty_struct *tty, struct file *filp)
{
	tty_port_close(tty->port, tty, filp);
}

static int rpmsg_tty_write(struct tty_struct *tty, const unsigned char *buf,
			   int total)
{
	int count, ret = 0;
	const unsigned char *tbuf;
	struct rpmsg_tty_port *cport = rpmsg_tty_get(tty->index);

	struct rpmsg_device *rpdev = cport->rpdev;
	int msg_size;

	dev_dbg(&rpdev->dev, "%s: send message from tty->index = %d\n",
		__func__, tty->index);

	if (!buf) {
		dev_err(&rpdev->dev, "buf shouldn't be null.\n");
		return -ENOMEM;
	}

	msg_size = rpmsg_get_buffer_size(rpdev->ept);
	if (msg_size < 0)
		return msg_size;

	count = total;
	tbuf = buf;
	do {
		/* send a message to our remote processor */
		ret = rpmsg_trysend(rpdev->ept, (void *)tbuf,
				    count > msg_size ? msg_size : count);
		if (ret) {
			dev_dbg(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
			return 0;
		}

		if (count > msg_size) {
			count -= msg_size;
			tbuf += msg_size;
		} else {
			count = 0;
		}
	} while (count > 0);

	tty_port_tty_wakeup(cport->port);

	return total;
}

static int rpmsg_tty_write_room(struct tty_struct *tty)
{
	struct rpmsg_tty_port *cport = rpmsg_tty_get(tty->index);
	struct rpmsg_device *rpdev = cport->rpdev;

	/* report the space in the rpmsg buffer */
	return rpmsg_get_buffer_size(rpdev->ept);
}

static const struct tty_operations rpmsg_tty_ops = {
	.install		= rpmsg_tty_install,
	.open			= rpmsg_tty_open,
	.close			= rpmsg_tty_close,
	.write			= rpmsg_tty_write,
	.write_room		= rpmsg_tty_write_room,
};

static int rpmsg_tty_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_tty_port *cport, *tmp;
	unsigned int index;
	struct device *tty_dev;
	int ret = 0;

	cport = devm_kzalloc(&rpdev->dev, sizeof(*cport), GFP_KERNEL);
	if (!cport)
		return -ENOMEM;

	mutex_lock(&rpmsg_tty_lock);

	/* Get free index */
	for (index = 0; index < MAX_TTY_RPMSG_INDEX; index++) {
		bool id_found = false;

		list_for_each_entry(tmp, &rpmsg_tty_list, list) {
			if (index == tmp->id) {
				id_found = true;
				break;
			}
		}
		if (!id_found)
			break;
	}

	if (index >= MAX_TTY_RPMSG_INDEX) {
		ret = -ENOSPC;
		goto end_probe;
	}

	/*
	 * the tty port allocation has to be independent from the tty device
	 * The reason is that it can be use after device removing, for instance
	 * if a user has opened it.
	 * So it is not possible to release the port on device remove.
	 * A solution is to store the port in the driver structure. The port
	 * structure is reused on next probe to save memory.
	 */
	if (!rpmsg_tty_driver->ports[index]) {
		/* first allocation of the port associated to the index */
		rpmsg_tty_driver->ports[index] = kzalloc(sizeof(*cport->port),
							 GFP_KERNEL);
		if (!rpmsg_tty_driver->ports[index]) {
			ret = -ENOMEM;
			goto end_probe;
		}
	}

	cport->port = rpmsg_tty_driver->ports[index];
	tty_port_init(cport->port);
	cport->port->ops = &rpmsg_tty_port_ops;

	spin_lock_init(&cport->rx_lock);
	cport->port->low_latency = cport->port->flags | ASYNC_LOW_LATENCY;
	cport->rpdev = rpdev;

	tty_dev = tty_port_register_device(cport->port, rpmsg_tty_driver,
					   index, &rpdev->dev);
	if (IS_ERR(tty_dev)) {
		dev_err(&rpdev->dev, "failed to register tty port\n");
		tty_port_destroy(cport->port);
		ret = PTR_ERR(tty_dev);
		goto end_probe;
	}

	cport->id = index;
	list_add_tail(&cport->list, &rpmsg_tty_list);
	dev_set_drvdata(&rpdev->dev, cport);

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x : ttyRPMSG%d\n",
		 rpdev->src, rpdev->dst, index);
end_probe:
	mutex_unlock(&rpmsg_tty_lock);

	return ret;
}

static void rpmsg_tty_remove(struct rpmsg_device *rpdev)
{
	struct rpmsg_tty_port *cport = dev_get_drvdata(&rpdev->dev);
	struct tty_struct *tty;

	tty_unregister_device(rpmsg_tty_driver, cport->id);
	tty = tty_port_tty_get(cport->port);
	if (tty) {
		tty_vhangup(cport->port->tty);
		tty_kref_put(tty);
	}
	list_del(&cport->list);

	dev_info(&rpdev->dev, "rpmsg tty device %d is removed\n", cport->id);
}

static struct rpmsg_device_id rpmsg_driver_tty_id_table[] = {
	{ .name	= "rpmsg-tty-channel" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_tty_id_table);

static struct rpmsg_driver rpmsg_tty_rmpsg_drv = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_driver_tty_id_table,
	.probe		= rpmsg_tty_probe,
	.callback	= rpmsg_tty_cb,
	.remove		= rpmsg_tty_remove,
};

static int __init rpmsg_tty_init(void)
{
	int err;

	rpmsg_tty_driver = tty_alloc_driver(MAX_TTY_RPMSG_INDEX, 0);
	if (IS_ERR(rpmsg_tty_driver))
		return PTR_ERR(rpmsg_tty_driver);

	rpmsg_tty_driver->driver_name = "rpmsg_tty";
	rpmsg_tty_driver->name = "ttyRPMSG";
	rpmsg_tty_driver->major = TTYAUX_MAJOR;
	rpmsg_tty_driver->minor_start = 3;
	rpmsg_tty_driver->type = TTY_DRIVER_TYPE_CONSOLE;
	rpmsg_tty_driver->init_termios = tty_std_termios;
	rpmsg_tty_driver->flags = TTY_DRIVER_REAL_RAW |
				  TTY_DRIVER_DYNAMIC_DEV;

	tty_set_operations(rpmsg_tty_driver, &rpmsg_tty_ops);

	/* Disable unused mode by default */
	rpmsg_tty_driver->init_termios = tty_std_termios;
	rpmsg_tty_driver->init_termios.c_lflag &= ~(ECHO | ICANON);
	rpmsg_tty_driver->init_termios.c_oflag &= ~(OPOST | ONLCR);

	err = tty_register_driver(rpmsg_tty_driver);
	if (err < 0) {
		pr_err("Couldn't install rpmsg tty driver: err %d\n", err);
		goto tty_error;
	}
	err = register_rpmsg_driver(&rpmsg_tty_rmpsg_drv);

	if (!err)
		return 0;

	tty_unregister_driver(rpmsg_tty_driver);

tty_error:
	put_tty_driver(rpmsg_tty_driver);

	return err;
}

static void __exit rpmsg_tty_exit(void)
{
	unsigned int index;

	unregister_rpmsg_driver(&rpmsg_tty_rmpsg_drv);
	tty_unregister_driver(rpmsg_tty_driver);

	/* release port allocations */
	for (index = 0; index < MAX_TTY_RPMSG_INDEX; index++)
		kfree(rpmsg_tty_driver->ports[index]);
	put_tty_driver(rpmsg_tty_driver);
}

module_init(rpmsg_tty_init);
module_exit(rpmsg_tty_exit);

MODULE_AUTHOR("Arnaud Pouliquen <arnaud.pouliquen@st.com>");
MODULE_DESCRIPTION("virtio remote processor messaging tty driver");
MODULE_LICENSE("GPL v2");
