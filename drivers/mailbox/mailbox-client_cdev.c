// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2024
 */
#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define MBOX_DEV_MAX (MINORMASK + 1)

#define cdev_to_mbxdev(i_cdev) container_of(i_cdev, struct mbox_cdev_ddata, cdev)

static struct class *mbox_cl_class;
static dev_t mbox_major;

enum  mbox_cdev_request_state {
	NO_REQ,
	REQ_SENT,
	REQ_ANSWERED,
};

/**
 * struct mbox_cdev_mbox - Mailbox client structure
 * @name: Name of the mailbox channel
 * @chan: Mailbox channel
 * @client: Mailbox client
 */
struct mbox_cdev_mbox {
	const unsigned char name[10];
	struct mbox_chan *chan;
	struct mbox_client client;
};

/**
 * struct mbox_cdev_ddata - Mailbox character device data
 * @dev: Device structure
 * @cdev: Character device structure
 * @mb: Mailbox client structure
 * @resm: Pointer to the mapped memory region
 * @resm_size: Size of the mapped memory region
 * @req_state: request state
 */
struct mbox_cdev_ddata {
	struct device dev;
	struct cdev cdev;
	struct mbox_cdev_mbox mb;
	void __iomem *resm;
	size_t resm_size;
	unsigned int req_state;
};

static void mbox_cdev_mb_callback(struct mbox_client *cl, void *data)
{
	struct mbox_cdev_mbox *mb = container_of(cl, struct mbox_cdev_mbox, client);
	struct mbox_cdev_ddata *mbxdev = container_of(mb, struct mbox_cdev_ddata, mb);

	dev_dbg(&mbxdev->dev, "Answer received\n");
	mbxdev->req_state = REQ_ANSWERED;
}

static const struct mbox_cdev_mbox rx_tx_mbox = {
	.name = "rx-tx",
	.client = {
		.rx_callback = mbox_cdev_mb_callback,
		.tx_block = true,
		.tx_done = NULL,
		.tx_tout = 500, /* 500 ms time out */
		.knows_txdone = false,
	},
};

static ssize_t mbox_cdev_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	struct mbox_cdev_ddata *mbxdev = cdev_to_mbxdev(filep->f_inode->i_cdev);

	if (len > mbxdev->resm_size)
		return -EINVAL;

	if (mbxdev->req_state == NO_REQ)
		return -EPERM;

	if (mbxdev->req_state == REQ_SENT)
		return -EBUSY;

	if (copy_to_user(buffer, mbxdev->resm, min(len, mbxdev->resm_size)))
		return -EFAULT;

	mbxdev->req_state = NO_REQ;

	return mbxdev->resm_size;
}

static ssize_t mbox_cdev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	struct mbox_cdev_ddata *mbxdev = cdev_to_mbxdev(filep->f_inode->i_cdev);
	int ret;

	if (len > mbxdev->resm_size)
		return -EINVAL;

	if (mbxdev->req_state == REQ_SENT)
		dev_warn(&mbxdev->dev, "Previous request not answered\n");

	if (copy_from_user(mbxdev->resm, buffer, len))
		return -EFAULT;

	ret = mbox_send_message(mbxdev->mb.chan, mbxdev->resm);
	if (ret < 0) {
		dev_err(&mbxdev->dev, "Failed to send message via mailbox\n");
		return ret;
	}

	dev_dbg(&mbxdev->dev, "Request sent\n");
	mbxdev->req_state = REQ_SENT;

	return len;
}

static const struct file_operations mbox_cdev_fops = {
	.read = mbox_cdev_read,
	.write = mbox_cdev_write,
};

static int mbox_cdev_request_mbox(struct device *dev, struct mbox_cdev_ddata *mbxdev)
{
	struct mbox_cdev_mbox *mb = &mbxdev->mb;
	struct mbox_client *cl;

	memcpy(mb, &rx_tx_mbox, sizeof(*mb));

	cl = &mb->client;
	cl->dev = dev;

	mb->chan = mbox_request_channel_byname(cl, mb->name);
	if (IS_ERR(mb->chan)) {
		dev_err_probe(dev, PTR_ERR(mb->chan), "Failed to request mailbox %s\n", mb->name);
		return PTR_ERR(mb->chan);
	}

	return 0;
}

static int mbox_cdev_get_memory_region(struct device *dev, struct mbox_cdev_ddata *mbxdev)
{
	struct device_node *np = dev->of_node;
	struct device_node *res_node;
	struct reserved_mem *rmem;

	res_node = of_parse_phandle(np, "memory-region", 0);
	if (!res_node) {
		dev_err(dev, "Unable to acquire memory region\n");
		return -ENODEV;
	}

	rmem = of_reserved_mem_lookup(res_node);
	if (!rmem) {
		of_node_put(res_node);
		dev_err(dev, "unable to acquire memory-region\n");
		return -EINVAL;
	}

	mbxdev->resm = devm_ioremap_wc(dev, rmem->base, rmem->size);
	if (IS_ERR(mbxdev->resm)) {
		dev_err(dev, "unable to map memory region\n");
		return PTR_ERR(mbxdev->resm);
	}
	mbxdev->resm_size = rmem->size;

	return 0;
}

static int mbxdev_char_device_add(struct platform_device *pdev, struct mbox_cdev_ddata *mbxdev)
{
	struct device *dev;
	int ret;

	dev = &mbxdev->dev;
	device_initialize(dev);
	dev->parent = &pdev->dev;

	cdev_init(&mbxdev->cdev, &mbox_cdev_fops);
	mbxdev->cdev.owner = THIS_MODULE;

	dev->devt = MKDEV(MAJOR(mbox_major), 0);
	cdev_set_parent(&mbxdev->cdev, &dev->kobj);
	ret = cdev_add(&mbxdev->cdev, dev->devt, 1);
	if (ret < 0)
		dev_err(&mbxdev->dev, "Failed to add char dev\n");

	dev = device_create(mbox_cl_class, &pdev->dev, dev->devt, NULL, "mailbox%d",
			    MINOR(dev->devt));
	if (IS_ERR(dev)) {
		cdev_del(&mbxdev->cdev);
		dev_err(&mbxdev->dev, "Failed to create device node\n");
		return PTR_ERR(dev);
	}
	return ret;
}

static void mbox_cdev_driver_remove(struct platform_device *pdev)
{
	struct mbox_cdev_ddata *mbxdev = platform_get_drvdata(pdev);

	device_destroy(mbox_cl_class, mbxdev->dev.devt);
	mbox_free_channel(mbxdev->mb.chan);
	cdev_del(&mbxdev->cdev);
}

static int mbox_cdev_driver_probe(struct platform_device *pdev)
{
	struct device *dev =  &pdev->dev;
	struct mbox_cdev_ddata *mbxdev;
	int ret;

	mbxdev = devm_kzalloc(&pdev->dev, sizeof(*mbxdev), GFP_KERNEL);
	if (!mbxdev)
		return -ENOMEM;

	ret = mbox_cdev_get_memory_region(dev, mbxdev);
	if (ret) {
		dev_err(dev, "Unable to acquire memory region\n");
		return ret;
	}

	/* Initialize mailbox client */
	ret = mbox_cdev_request_mbox(dev, mbxdev);
	if (ret)
		return ret;

	ret = mbxdev_char_device_add(pdev, mbxdev);
	if (ret)
		goto free_mbx;

	platform_set_drvdata(pdev, mbxdev);

	return 0;

free_mbx:
	mbox_free_channel(mbxdev->mb.chan);

	return ret;
}

static const struct of_device_id mbox_cdev_match[] = {
	{ .compatible = "mbox-cdev", },
	{},
};
MODULE_DEVICE_TABLE(of, mbox_cdev_match);

static struct platform_driver mbox_cdev_driver = {
	.probe = mbox_cdev_driver_probe,
	.remove = mbox_cdev_driver_remove,
	.driver = {
		.name = "mbox-cdev",
		.of_match_table = mbox_cdev_match,
	},
};

static int __init mbox_cdev_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&mbox_major, 0, MBOX_DEV_MAX, "mailbox");
	if (ret < 0) {
		pr_err("failed to allocate char dev region\n");
		return ret;
	}

	mbox_cl_class = class_create("mailbox");
		if (IS_ERR(mbox_cl_class)) {
			unregister_chrdev_region(mbox_major, MBOX_DEV_MAX);
			pr_err("Failed to create class\n");
		return PTR_ERR(mbox_cl_class);
	}

	return platform_driver_register(&mbox_cdev_driver);
}

static void __exit mbox_cdev_exit(void)
{
	platform_driver_unregister(&mbox_cdev_driver);
	class_destroy(mbox_cl_class);
	unregister_chrdev_region(mbox_major, MBOX_DEV_MAX);
}

module_init(mbox_cdev_init);
module_exit(mbox_cdev_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("A simple char driver for communication using a mailbox and a shared memory");
