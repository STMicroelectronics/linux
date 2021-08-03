// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) STMicroelectronics 2019 - All Rights Reserved
 * Author: Jean-Philippe Romain <jean-philippe.romain@st.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/eventfd.h>
#include <linux/of_platform.h>
#include <linux/list.h>
#include <asm/cacheflush.h>

#define RPMSG_SDB_DRIVER_VERSION "1.0"

/*
 * Static global variables
 */
static const char rpmsg_sdb_driver_name[] = "stm32-rpmsg-sdb";

static int LastBufferId;

struct rpmsg_sdb_ioctl_set_efd {
	int bufferId, eventfd;
};

struct rpmsg_sdb_ioctl_get_data_size {
	int bufferId;
	uint32_t size;
};

struct rpmsg_sdb_ioctl_get_phys_addr {
	int bufferId;
	uint32_t paddr;
};

struct rpmsg_sdb_ioctl_set_sync {
	int bufferId;
};

/* ioctl numbers */
/* _IOW means userland is writing and kernel is reading */
/* _IOR means userland is reading and kernel is writing */
/* _IOWR means userland and kernel can both read and write */
#define RPMSG_SDB_IOCTL_SET_EFD			_IOW('R', 0x00, struct rpmsg_sdb_ioctl_set_efd *)
#define RPMSG_SDB_IOCTL_GET_DATA_SIZE	_IOWR('R', 0x01, struct rpmsg_sdb_ioctl_get_data_size *)
#define RPMSG_SDB_IOCTL_GET_PHYS_ADDR	_IOWR('R', 0x02, struct rpmsg_sdb_ioctl_get_phys_addr *)
#define RPMSG_SDB_IOCTL_SET_SYNC_CPU	_IOW('R', 0x03, struct rpmsg_sdb_ioctl_set_sync *)
#define RPMSG_SDB_IOCTL_SET_SYNC_DEV	_IOW('R', 0x04, struct rpmsg_sdb_ioctl_set_sync *)
#define RPMSG_SDB_IOCTL_SET_CACHE_FLUSH	_IOW('R', 0x05, struct rpmsg_sdb_ioctl_set_sync *)

struct sdb_buf_t {
	int index; /* index of buffer */
	size_t size; /* buffer size */
	size_t writing_size; /* size of data written by copro */
	dma_addr_t paddr; /* physical address*/
	void *vaddr; /* virtual address */
	void *uaddr_start; /* mapped address start for userland */
	void *uaddr_end;  /* mapped address end for userland */
	struct eventfd_ctx *efd_ctx; /* eventfd context */
	struct list_head buflist; /* reference in the buffers list */
	struct vm_area_struct *vma;
};

struct rpmsg_sdb_t {
	struct mutex	mutex; /* mutex to protect the ioctls */
	struct miscdevice mdev; /* misc device ref */
	struct rpmsg_device	*rpdev;	/* handle rpmsg device */
	struct list_head buffer_list; /* buffer instances list */
};

struct device *rpmsg_sdb_dev;
//---------------------------------------------------------------------------------------------------
//  Datum - for now disable the internal message channel.   We may want this back when DMA is working
//---------------------------------------------------------------------------------------------------

// static int rpmsg_sdb_format_txbuf_string(struct sdb_buf_t *buffer, char *bufinfo_str, size_t bufinfo_str_size)
// {
// 	return snprintf(bufinfo_str, bufinfo_str_size, "B%dA%08xL%08x", buffer->index, buffer->paddr, buffer->size);
// }

// static long rpmsg_sdb_decode_rxbuf_string(char *rxbuf_str, int *buffer_id, size_t *size)
// {
// 	int ret = 0;
// 	char *sub_str;
// 	long bsize;
// 	long bufid;
// 	const char delimiter[1] = {'L'};

// 	//pr_err("%s: rxbuf_str:%s\n", __func__, rxbuf_str);

// 	/* Get first part containing the buffer id */
// 	sub_str = strsep(&rxbuf_str, delimiter);

// 	//pr_err("%s: sub_str:%s\n", __func__, sub_str);

// 	/* Save Buffer id and size: template BxLyyyyyyyy*/
// 	ret = kstrtol(&sub_str[1], 10, &bufid);
// 	if (ret < 0) {
// 		pr_err("%s: extract of buffer id failed(%d)", __func__, ret);
// 		goto out;
// 	}

// 	ret = kstrtol(&rxbuf_str[2], 16, &bsize);
// 	if (ret < 0) {
// 		pr_err("%s: extract of buffer size failed(%d)", __func__, ret);
// 		goto out;
// 	}

// 	*size = (size_t)bsize;
// 	*buffer_id = (int)bufid;

// out:
// 	return ret;
// }

// static int rpmsg_sdb_send_buf_info(struct rpmsg_sdb_t *rpmsg_sdb, struct sdb_buf_t *buffer)
// {
// 	int count = 0, ret = 0;
// 	const unsigned char *tbuf;
// 	char mybuf[32];
// 	int msg_size;
// 	struct rpmsg_device *_rpdev;

// 	_rpdev = rpmsg_sdb->rpdev;
// 	msg_size = rpmsg_get_buffer_size(_rpdev->ept);

// 	if (msg_size < 0)
// 		return msg_size;

// 	count = rpmsg_sdb_format_txbuf_string(buffer, mybuf, 32);
// 	tbuf = &mybuf[0];

// 	do {
// 		/* send a message to our remote processor */
// 		ret = rpmsg_send(_rpdev->ept, (void *)tbuf,
// 				 count > msg_size ? msg_size : count);
// 		if (ret) {
// 			dev_err(&_rpdev->dev, "rpmsg_send failed: %d\n", ret);
// 			return ret;
// 		}

// 		if (count > msg_size) {
// 			count -= msg_size;
// 			tbuf += msg_size;
// 		} else {
// 			count = 0;
// 		}
// 	} while (count > 0);

// 	return count;
// }
 static int rpmsg_sdb_send_buf_info(struct rpmsg_sdb_t *rpmsg_sdb, struct sdb_buf_t *buffer)
 {
	 return 0;
 }


static int rpmsg_sdb_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long size = PAGE_ALIGN(vsize);
	unsigned long NumPages = size >> PAGE_SHIFT;
	unsigned long align = get_order(size);
	pgprot_t prot = vma->vm_page_prot;
	struct rpmsg_sdb_t *_rpmsg_sdb;
	struct sdb_buf_t *_buffer;

	if (align > CONFIG_CMA_ALIGNMENT)
		align = CONFIG_CMA_ALIGNMENT;

	if (rpmsg_sdb_dev == NULL)
		return -ENOMEM;

	rpmsg_sdb_dev->coherent_dma_mask = DMA_BIT_MASK(32);
	rpmsg_sdb_dev->dma_mask = &rpmsg_sdb_dev->coherent_dma_mask;

	_rpmsg_sdb = container_of(file->private_data, struct rpmsg_sdb_t,
								mdev);

	/* Field the last buffer entry which is the last one created */
	if (!list_empty(&_rpmsg_sdb->buffer_list)) {
		_buffer = list_last_entry(&_rpmsg_sdb->buffer_list,
									struct sdb_buf_t, buflist);

		_buffer->uaddr_start = NULL;
		_buffer->size = NumPages * PAGE_SIZE;
		_buffer->writing_size = -1;
		//_buffer->vaddr = dma_alloc_wc(rpmsg_sdb_dev,
		//										_buffer->size,
		//										&_buffer->paddr,
		//										GFP_KERNEL);

		_buffer->vaddr = dma_alloc_coherent(rpmsg_sdb_dev,
												_buffer->size,
												&_buffer->paddr,
												GFP_KERNEL);

		if (!_buffer->vaddr) {
			pr_err("%s: Memory allocation issue\n", __func__);
			return -ENOMEM;
		}

		//pr_debug("%s - dma_alloc_wc done - paddr[%d]:%x - vaddr[%d]:%p\n", __func__, _buffer->index, _buffer->paddr, _buffer->index, _buffer->vaddr);
		pr_debug("%s - dma_alloc_coherent done - paddr[%d]:%x - vaddr[%d]:%p\n", __func__, _buffer->index, _buffer->paddr, _buffer->index, _buffer->vaddr);

		/* Get address for userland */
		if (remap_pfn_range(vma, vma->vm_start,
							(_buffer->paddr >> PAGE_SHIFT) + vma->vm_pgoff,
							size, prot))
			return -EAGAIN;

		_buffer->uaddr_start = (void *)vma->vm_start;
		_buffer->uaddr_end = (void *)vma->vm_end;
		_buffer->vma = vma;

		/* Send information to remote proc */
		rpmsg_sdb_send_buf_info(_rpmsg_sdb, _buffer);
	} else {
		dev_err(rpmsg_sdb_dev, "No existing buffer entry exist in the list !!!");
		return -EINVAL;
	}

	/* Increment for number of requested buffer */
	LastBufferId++;

	return 0;
}

/**
 * rpmsg_sdb_open - Open Session
 *
 * @inode:	inode struct
 * @file:	file struct
 *
 * Return:
 *	0 - Success
 *	Non-zero - Failure
 */
static int rpmsg_sdb_open(struct inode *inode, struct file *file)
{
	struct rpmsg_sdb_t *_rpmsg_sdb;

	_rpmsg_sdb = container_of(file->private_data, struct rpmsg_sdb_t,
								mdev);

	/* Initialize the buffer list*/
	INIT_LIST_HEAD(&_rpmsg_sdb->buffer_list);

	mutex_init(&_rpmsg_sdb->mutex);

	return 0;
}

/**
 * rpmsg_sdb_close - Close Session
 *
 * @inode:	inode struct
 * @file:	file struct
 *
 * Return:
 *	0 - Success
 *	Non-zero - Failure
 */
static int rpmsg_sdb_close(struct inode *inode, struct file *file)
{
	struct rpmsg_sdb_t *_rpmsg_sdb;
	struct sdb_buf_t *pos, *next;

	_rpmsg_sdb = container_of(file->private_data, struct rpmsg_sdb_t,
												mdev);

	list_for_each_entry_safe(pos, next, &_rpmsg_sdb->buffer_list, buflist) {
		/* Free the CMA allocation */
		// Use for DMA transfers
		//dma_free_wc(rpmsg_sdb_dev, pos->size, pos->vaddr,
		//			pos->paddr);
		// Use for non-DMA (memcpy) transfers
		dma_free_coherent(rpmsg_sdb_dev, pos->size, pos->vaddr,
					pos->paddr);
		/* Remove the buffer from the list */
		list_del(&pos->buflist);
		/* Free the buffer */
		kfree(pos);
	}

	/* Reset LastBufferId */
	LastBufferId = 0;

	return 0;
}

/**
 * rpmsg_sdb_ioctl - IOCTL
 *
 * @session:	ibmvmc_file_session struct
 * @cmd:	cmd field
 * @arg:	Argument field
 *
 * Return:
 *	0 - Success
 *	Non-zero - Failure
 */
static long rpmsg_sdb_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int idx = 0;

	struct rpmsg_sdb_t *_rpmsg_sdb;
	struct sdb_buf_t *buffer, *lastbuffer;

	struct list_head *pos;
	struct sdb_buf_t *datastructureptr = NULL;

	struct rpmsg_sdb_ioctl_set_efd q_set_efd;
	struct rpmsg_sdb_ioctl_get_data_size q_get_dat_size;
	struct rpmsg_sdb_ioctl_get_phys_addr q_get_phys_addr;
	struct rpmsg_sdb_ioctl_set_sync q_set_sync;

	void __user *argp = (void __user *)arg;

	if (rpmsg_sdb_dev == NULL)
		return -ENOMEM;

	_rpmsg_sdb = container_of(file->private_data, struct rpmsg_sdb_t, mdev);

	switch (cmd) {
	case RPMSG_SDB_IOCTL_SET_EFD:
		mutex_lock(&_rpmsg_sdb->mutex);

		/* Get index from the last buffer in the list */
		if (!list_empty(&_rpmsg_sdb->buffer_list))
		{
			lastbuffer = list_last_entry(&_rpmsg_sdb->buffer_list, struct sdb_buf_t, buflist);
			idx = lastbuffer->index;
			/* increment this index for the next buffer creation*/
			idx++;
		}

		if (copy_from_user(&q_set_efd, (struct rpmsg_sdb_ioctl_set_efd *)argp, sizeof(struct rpmsg_sdb_ioctl_set_efd)))
		{
			pr_warn("rpmsg_sdb: RPMSG_SDB_IOCTL_GET_DATA_SIZE: copy to user failed.\n");
			mutex_unlock(&_rpmsg_sdb->mutex);
			return -EFAULT;
		}

		/* create a new buffer which will be added in the buffer list */
		buffer = kmalloc(sizeof(struct sdb_buf_t), GFP_KERNEL);

		buffer->index = idx;
		buffer->efd_ctx = eventfd_ctx_fdget(q_set_efd.eventfd);
		list_add_tail(&buffer->buflist, &_rpmsg_sdb->buffer_list);

		mutex_unlock(&_rpmsg_sdb->mutex);
		break;

	case RPMSG_SDB_IOCTL_GET_DATA_SIZE:
		if (copy_from_user(&q_get_dat_size, (struct rpmsg_sdb_ioctl_get_data_size *)argp, sizeof(struct rpmsg_sdb_ioctl_get_data_size)))
		{
			pr_warn("rpmsg_sdb: RPMSG_SDB_IOCTL_GET_DATA_SIZE: copy from user failed.\n");
			return -EFAULT;
		}

		/* Get the index of the requested buffer and then look-up in the buffer list*/
		idx = q_get_dat_size.bufferId;

		list_for_each(pos, &_rpmsg_sdb->buffer_list)
		{
			datastructureptr = list_entry(pos, struct sdb_buf_t, buflist);
			if (datastructureptr->index == idx)
			{
				/* Get the writing size*/
				q_get_dat_size.size = datastructureptr->writing_size;
				break;
			}
		}

		if (copy_to_user((struct rpmsg_sdb_ioctl_get_data_size *)argp, &q_get_dat_size, sizeof(struct rpmsg_sdb_ioctl_get_data_size)))
		{
			pr_warn("rpmsg_sdb: RPMSG_SDB_IOCTL_GET_DATA_SIZE: copy to user failed.\n");
			return -EFAULT;
		}

		/* Reset the writing size*/
		datastructureptr->writing_size = -1;
		break;

	case RPMSG_SDB_IOCTL_GET_PHYS_ADDR:
		if (copy_from_user(&q_get_phys_addr, (struct rpmsg_sdb_ioctl_get_phys_addr *)argp, sizeof(struct rpmsg_sdb_ioctl_get_phys_addr)))
		{
			pr_warn("rpmsg_sdb: RPMSG_SDB_IOCTL_GET_PHYS_ADDR: copy from user failed.\n");
			return -EFAULT;
		}

		/* Get the index of the requested buffer and then look-up in the buffer list*/
		idx = q_get_phys_addr.bufferId;

		list_for_each(pos, &_rpmsg_sdb->buffer_list)
		{
			datastructureptr = list_entry(pos, struct sdb_buf_t, buflist);
			if (datastructureptr->index == idx)
			{
				/* Get the physical address*/
				q_get_phys_addr.paddr = (uint32_t)(datastructureptr->paddr);
				break;
			}
		}
		if (copy_to_user((struct rpmsg_sdb_ioctl_get_phys_addr *)argp, &q_get_phys_addr, sizeof(struct rpmsg_sdb_ioctl_get_phys_addr)))
		{
			pr_warn("rpmsg_sdb: RPMSG_SDB_IOCTL_GET_PHYS_ADDR: copy to user failed.\n");
			return -EFAULT;
		}
		break;

	case RPMSG_SDB_IOCTL_SET_SYNC_CPU:
		if(copy_from_user(&q_set_sync, (struct rpmsg_sdb_ioctl_set_sync *)argp, sizeof(struct rpmsg_sdb_ioctl_set_sync)))
		{
			pr_warn("rpmsg_sdb: RPMSG_SDB_IOCTL_SYNC_CPU: copy from user failed.\n");
			return -EFAULT;
		}

		/* Get the index of the requested buffer and then look-up in the buffer list*/
		idx = q_set_sync.bufferId;

		list_for_each(pos, &_rpmsg_sdb->buffer_list)
		{
			datastructureptr = list_entry(pos, struct sdb_buf_t, buflist);
			if (datastructureptr->index == idx)
			{
				/* sync dcache for cpu access */
				pr_debug("dma_sync_cpu: idx:%d, paddr:%x, vaddr:%p, writing_size:%d\n", idx, datastructureptr->paddr, datastructureptr->vaddr, datastructureptr->writing_size);
				dma_sync_single_for_cpu(rpmsg_sdb_dev, datastructureptr->paddr, datastructureptr->size, DMA_BIDIRECTIONAL);
				break;
			}
		}
		break;

	case RPMSG_SDB_IOCTL_SET_SYNC_DEV:
		if(copy_from_user(&q_set_sync, (struct rpmsg_sdb_ioctl_set_sync *)argp, sizeof(struct rpmsg_sdb_ioctl_set_sync)))
		{
			pr_warn("rpmsg_sdb: RPMSG_SDB_IOCTL_SYNC_DEV: copy from user failed.\n");
			return -EFAULT;
		}

		/* Get the index of the requested buffer and then look-up in the buffer list*/
		idx = q_set_sync.bufferId;

		list_for_each(pos, &_rpmsg_sdb->buffer_list)
		{
			datastructureptr = list_entry(pos, struct sdb_buf_t, buflist);
			if (datastructureptr->index == idx)
			{
				/* sync dcache for device access */
				pr_debug("dma_sync_dev: idx:%d, paddr:%x, vaddr:%p, writing_size:%d\n", idx, datastructureptr->paddr, datastructureptr->vaddr, datastructureptr->writing_size);
				dma_sync_single_for_device(rpmsg_sdb_dev, datastructureptr->paddr, datastructureptr->size, DMA_BIDIRECTIONAL);
				break;
			}
		}
		break;

	case RPMSG_SDB_IOCTL_SET_CACHE_FLUSH:
		if(copy_from_user(&q_set_sync, (struct rpmsg_sdb_ioctl_set_sync *)argp, sizeof(struct rpmsg_sdb_ioctl_set_sync)))
		{
			pr_warn("rpmsg_sdb: RPMSG_SDB_IOCTL_SYNC_CPU: copy from user failed.\n");
			return -EFAULT;
		}

		/* Get the index of the requested buffer and then look-up in the buffer list*/
		idx = q_set_sync.bufferId;

		list_for_each(pos, &_rpmsg_sdb->buffer_list)
		{
			datastructureptr = list_entry(pos, struct sdb_buf_t, buflist);
			if (datastructureptr->index == idx)
			{
				/* force dcache sync */
				pr_debug("flush_cache_range: idx:%d, paddr:%x, vaddr:%p, writing_size:%d\n", idx, datastructureptr->paddr, datastructureptr->vaddr, datastructureptr->writing_size);
				flush_cache_range(datastructureptr->vma, (unsigned long)datastructureptr->uaddr_start, (unsigned long)datastructureptr->uaddr_end);
				break;
			}
		}
		break;

	default:
		return -EINVAL;
		break;
	}

	return 0;
}

static const struct file_operations rpmsg_sdb_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= rpmsg_sdb_ioctl,
	.mmap			= rpmsg_sdb_mmap,
	.open           = rpmsg_sdb_open,
	.release        = rpmsg_sdb_close,
};

//  Datum - for now disable the internal message channel.   We may want this back when DMA is working
// static int rpmsg_sdb_drv_cb(struct rpmsg_device *rpdev, void *data, int len,
// 			void *priv, u32 src)
// {
// 	int ret = 0;
// 	int buffer_id = 0;
// 	size_t buffer_size;
// 	char rpmsg_RxBuf[len+1];
// 	struct list_head *pos;
// 	struct sdb_buf_t *datastructureptr = NULL;

// 	struct rpmsg_sdb_t *drv = dev_get_drvdata(&rpdev->dev);

// 	if (len == 0) {
// 		dev_err(rpmsg_sdb_dev, "(%s) Empty length requested\n", __func__);
// 		return -EINVAL;
// 	}

//     dev_err(rpmsg_sdb_dev, "(%s) length: %d\n", __func__,len);

// 	memcpy(rpmsg_RxBuf, data, len);

//     rpmsg_RxBuf[len] = 0;

// 	ret = rpmsg_sdb_decode_rxbuf_string(rpmsg_RxBuf, &buffer_id, &buffer_size);
// 	if (ret < 0)
// 		goto out;

// 	if (buffer_id > LastBufferId) {
// 		ret = -EINVAL;
// 		goto out;
// 	}

// 	/* Signal to User space application */
// 	list_for_each(pos, &drv->buffer_list)
// 	{
// 		datastructureptr = list_entry(pos, struct sdb_buf_t, buflist);
// 		if (datastructureptr->index == buffer_id) {
// 			datastructureptr->writing_size = buffer_size;

// 			if (datastructureptr->writing_size > datastructureptr->size) {
// 				dev_err(rpmsg_sdb_dev, "(%s) Writing size is bigger than buffer size\n", __func__);
// 				ret = -EINVAL;
// 				goto out;
// 			}

// 			eventfd_signal(datastructureptr->efd_ctx, 1);
// 			break;
// 		}
// 		/* TODO: quid if nothing find during the loop ? */
// 	}

// out:
// 	return ret;
// }
static int rpmsg_sdb_drv_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	return 0;
}

static int rpmsg_sdb_drv_probe(struct rpmsg_device *rpdev)
{
	int ret = 0;
	struct device *dev = &rpdev->dev;
	struct rpmsg_sdb_t *rpmsg_sdb;

	rpmsg_sdb = devm_kzalloc(dev, sizeof(*rpmsg_sdb), GFP_KERNEL);
	if (!rpmsg_sdb)
		return -ENOMEM;

	mutex_init(&rpmsg_sdb->mutex);

	rpmsg_sdb->rpdev = rpdev;

	rpmsg_sdb->mdev.name = "rpmsg-sdb";
	rpmsg_sdb->mdev.minor = MISC_DYNAMIC_MINOR;
	rpmsg_sdb->mdev.fops = &rpmsg_sdb_fops;

	dev_set_drvdata(&rpdev->dev, rpmsg_sdb);

	/* Register misc device */
	ret = misc_register(&rpmsg_sdb->mdev);

	if (ret) {
		dev_err(dev, "Failed to register device\n");
		goto err_out;
	}

	rpmsg_sdb_dev = rpmsg_sdb->mdev.this_device;

	dev_info(dev, "%s probed\n", rpmsg_sdb_driver_name);

err_out:
	return ret;
}

static void rpmsg_sdb_drv_remove(struct rpmsg_device *rpmsgdev)
{
	struct rpmsg_sdb_t *drv = dev_get_drvdata(&rpmsgdev->dev);

	misc_deregister(&drv->mdev);
}

static struct rpmsg_device_id rpmsg_driver_sdb_id_table[] = {
	{ .name	= "rpmsg-sdb-channel" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_sdb_id_table);

static struct rpmsg_driver rpmsg_sdb_rmpsg_drv = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_driver_sdb_id_table,
	.probe		= rpmsg_sdb_drv_probe,
	.callback	= rpmsg_sdb_drv_cb,
	.remove		= rpmsg_sdb_drv_remove,
};

static int __init rpmsg_sdb_drv_init(void)
{
	int ret = 0;

	/* Register rpmsg device */
	ret = register_rpmsg_driver(&rpmsg_sdb_rmpsg_drv);

	if (ret) {
		pr_err("%s(rpmsg_sdb): Failed to register device\n", __func__);
		return ret;
	}

	pr_info("%s(rpmsg_sdb): Init done\n", __func__);

	return ret;
}

static void __exit rpmsg_sdb_drv_exit(void)
{
	unregister_rpmsg_driver(&rpmsg_sdb_rmpsg_drv);
	pr_info("%s(rpmsg_sdb): Exit\n", __func__);
}

module_init(rpmsg_sdb_drv_init);
module_exit(rpmsg_sdb_drv_exit);


MODULE_AUTHOR("Jean-Philippe Romain <jean-philippe.romain@st.com>");
MODULE_DESCRIPTION("shared data buffer over RPMSG");
MODULE_VERSION(RPMSG_SDB_DRIVER_VERSION);
MODULE_LICENSE("GPL v2");