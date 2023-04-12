#define DEBUG

// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/uio/uio_dsi_mcc.c
 *
 * Userspace I/O platform driver with generic IRQ handling code.
 *
 * Copyright (C) 2023 by Datum Systems, Inc.
 *
 * Based on uio_pdrv_genirq.c by Magnus Damm
 * Copyright (c) 2008 by Magnus Damm
 * Based on uio_pdrv.c by Uwe Kleine-Koenig,
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 */

#include <linux/hwspinlock.h>
#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/stringify.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/irq.h>

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#define RX_nTX_FLAG 0x80000000
#define INTERRUPT_DISABLE 0x40000000
#define RESET_RQST_CM4_TO_CA7 0x20000000
#define RESET_RQST_CA7_TO_CM4 0x10000000

#define HWSPNLCK_TIMEOUT 1000 /* usec */
#define DRIVER_NAME "uio_dsi_mcc"

struct uio_dsi_mcc_ctl {
	void __iomem *mcctx_rd;
	void __iomem *mcctx_wr;
	void __iomem *mccrx_rd;
	void __iomem *mccrx_wr;
} __attribute__((packed));

struct uio_dsi_mcc_platdata {
	struct uio_info *uioinfo;
	spinlock_t lock;
	struct hwspinlock *hwlock_tx;
	struct hwspinlock *hwlock_rx;
	int hwlock_id_tx;
	int hwlock_id_rx;
	struct gpio_desc *tx_irq_gpio;
	struct uio_dsi_mcc_ctl ctl;
	unsigned long flags;
	struct platform_device *pdev;
};

/* Bits in uio_dsi_mcc_platdata.flags */
enum { UIO_IRQ_DISABLED = 0,
};

static int uio_dsi_mcc_open(struct uio_info *info, struct inode *inode)
{
	struct uio_dsi_mcc_platdata *priv = info->priv;

	/* Wait until the Runtime PM code has woken up the device */
	pm_runtime_get_sync(&priv->pdev->dev);
	return 0;
}

static int uio_dsi_mcc_release(struct uio_info *info, struct inode *inode)
{
	struct uio_dsi_mcc_platdata *priv = info->priv;

	/* Tell the Runtime PM code that the device has become idle */
	pm_runtime_put_sync(&priv->pdev->dev);
	return 0;
}

static irqreturn_t uio_dsi_mcc_handler(int irq, struct uio_info *dev_info)
{
	struct uio_dsi_mcc_platdata *priv = dev_info->priv;

	/* Just disable the interrupt in the interrupt controller, and
	 * remember the state so we can allow user space to enable it later.
	 */

	spin_lock(&priv->lock);
	if (!__test_and_set_bit(UIO_IRQ_DISABLED, &priv->flags))
		disable_irq_nosync(irq);
	spin_unlock(&priv->lock);

	return IRQ_HANDLED;
}

static int uio_dsi_mcc_irqcontrol(struct uio_info *dev_info, s32 control)
{
	struct uio_dsi_mcc_platdata *priv = dev_info->priv;
	struct hwspinlock *hwlock;

	unsigned long flags;
	int err;

	spin_lock_irqsave(&priv->lock, flags);

	/* The control integer is mapped as following:
	 * MSB:  1 bit   : 1 = operation is rx
	                 : 0 = operation is tx
	 *       1 bit   : 0 = enable cm4 rx interrupt and update rx rd offset if operation is rx
	 *                 1 = disable cm4 rx interrupt and update rx rd offset if operation is rx
	 *                 0 = toggle cm4 tx interrupt and update tx wr offset if opeartion is tx
	 *                 1 = update tx wr offset only, no cm4 interrupt toggle.
	 *       1 bit     1 = reset request from CM4 to CA7
	 *       1 bit     1 = reset reuest from CA& to CM4
	 *       13 bits : spare
	 * LSB:  16 bits : new tx or rx offset
	 */

	if (control & RX_nTX_FLAG) {
		hwlock = priv->hwlock_rx;
		/* rx interrupt control*/
		if (hwlock) {
			err = hwspin_lock_timeout_in_atomic(hwlock,
							    HWSPNLCK_TIMEOUT);
			if (err) {
				pr_err("%s can't get rx hwspinlock (%d)\n",
				       __func__, err);
				goto hwunlock;
			}
		}
		/* update rd offset*/
		iowrite32(control & 0x0000ffff, priv->ctl.mccrx_rd);
		/* Clear rx interrupt if requested*/
		if (control & INTERRUPT_DISABLE) {
			if (!__test_and_set_bit(UIO_IRQ_DISABLED, &priv->flags))
				disable_irq_nosync(dev_info->irq);
		} else {
			if (__test_and_clear_bit(UIO_IRQ_DISABLED,
						 &priv->flags))
				enable_irq(dev_info->irq);
		}
	} else {
		/* tx interrupt control*/
		hwlock = priv->hwlock_tx;
		if (hwlock) {
			err = hwspin_lock_timeout_in_atomic(hwlock,
							    HWSPNLCK_TIMEOUT);
			if (err) {
				pr_err("%s can't get tx hwspinlock (%d)\n",
				       __func__, err);
				goto hwunlock;
			}
		}
		if (control & RESET_RQST_CA7_TO_CM4) {
			printk("uio-tx_brmcc:  reset offsets\n");
			// reset buffers / CM4 reset reqeust flag
			iowrite32(RESET_RQST_CA7_TO_CM4, priv->ctl.mcctx_wr);
		} else {
			iowrite32(control & 0x0000ffff, priv->ctl.mcctx_wr);
			//printk("uio-tx_brmcc:  update mcctx_wr=%d\n", ioread32( priv->ctl.mcctx_wr));
		}
		/* Set tx interrupt if requested */
		if (!(control & INTERRUPT_DISABLE)) {
			// toggle irq pin for CM4
			gpiod_set_value(priv->tx_irq_gpio, 1);
			gpiod_set_value(priv->tx_irq_gpio, 0);
		}
	}

hwunlock:
	if (hwlock)
		hwspin_unlock_in_atomic(hwlock);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void uio_dsi_mcc_cleanup(void *data)
{
	struct device *dev = data;

	pm_runtime_disable(dev);
}

static int uio_dsi_mcc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct uio_info *uioinfo = dev_get_platdata(dev);
	struct device_node *node = pdev->dev.of_node;
	struct uio_dsi_mcc_platdata *priv;
	struct uio_mem *uiomem;
	int ret = -EINVAL;
	struct hwspinlock *hwlock_tx, *hwlock_rx;
	int hwlock_id_tx, hwlock_id_rx;
	int i;

	if (node) {
		const char *name;

		/* alloc uioinfo for one device */
		uioinfo = devm_kzalloc(dev, sizeof(*uioinfo), GFP_KERNEL);
		if (!uioinfo) {
			dev_err(dev, "unable to kmalloc\n");
			return -ENOMEM;
		}
		/* check for hwspinlocks which may not be available yet*/
		hwlock_id_tx = of_hwspin_lock_get_id(node, 0);
		if (hwlock_id_tx == -EPROBE_DEFER)
			/* hwspinlock framwork not yet ready */
			return hwlock_id_tx;

		if (hwlock_id_tx >= 0) {
			hwlock_tx = devm_hwspin_lock_request_specific(
				dev, hwlock_id_tx);
			if (!hwlock_tx) {
				dev_err(dev,
					"Failed to request tx hwspinlock\n");
				return -EINVAL;
			}
		} else {
			dev_err(dev, "Failed to get tx hwspinlock\n");
			return ret;
		}
		hwlock_id_rx = of_hwspin_lock_get_id(node, 1);
		if (hwlock_id_rx >= 0) {
			hwlock_rx = devm_hwspin_lock_request_specific(
				dev, hwlock_id_rx);
			if (!hwlock_rx) {
				dev_err(dev,
					"Failed to request rx hwspinlock\n");
				return -EINVAL;
			}
		} else {
			dev_err(dev, "Failed to get rx hwspinlock\n");
			return ret;
		}
		dev_dbg(dev, "hwlocks tx(%d), rx(%d)\n", hwlock_id_tx,
			hwlock_id_rx);

		if (!of_property_read_string(node, "linux,uio-name", &name))
			uioinfo->name = devm_kstrdup(dev, name, GFP_KERNEL);
		else
			uioinfo->name =
				devm_kasprintf(dev, GFP_KERNEL, "%pOFn", node);

		uioinfo->version = "devicetree";
		/* Multiple IRQs are not supported */
	}

	if (!uioinfo || !uioinfo->name || !uioinfo->version) {
		dev_err(dev, "missing platform_data\n");
		return ret;
	}

	if (uioinfo->handler || uioinfo->irqcontrol ||
	    uioinfo->irq_flags & IRQF_SHARED) {
		dev_err(dev, "interrupt configuration error\n");
		return ret;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "unable to kmalloc\n");
		return -ENOMEM;
	}

	priv->uioinfo = uioinfo;
	spin_lock_init(&priv->lock);
	priv->flags = 0; /* interrupt is enabled to begin with */
	priv->pdev = pdev;
	priv->hwlock_tx = hwlock_tx;
	priv->hwlock_id_tx = hwlock_id_tx;
	priv->hwlock_rx = hwlock_rx;
	priv->hwlock_id_rx = hwlock_id_rx;

	/* mcc-tx irq gpio */
	if (!priv->tx_irq_gpio) {
		priv->tx_irq_gpio =
			devm_gpiod_get(&pdev->dev, "tx-irq", GPIOD_OUT_LOW);
		if (IS_ERR(priv->tx_irq_gpio)) {
			dev_err(dev, "missing tx irq gpio\n");
			return -EINVAL;
		}
		gpiod_set_value(priv->tx_irq_gpio, 0);
	}

	/* mcc-rx irq gpio */
	if (!uioinfo->irq) {
		ret = platform_get_irq_optional(pdev, 0);
		uioinfo->irq = ret;
		if (ret == -ENXIO)
			uioinfo->irq = UIO_IRQ_NONE;
		else if (ret == -EPROBE_DEFER)
			return ret;
		else if (ret < 0) {
			dev_err(dev, "failed to get IRQ\n");
			return ret;
		}
	}

	if (uioinfo->irq) {
		struct irq_data *irq_data = irq_get_irq_data(uioinfo->irq);

		/*
		 * If a level interrupt, dont do lazy disable. Otherwise the
		 * irq will fire again since clearing of the actual cause, on
		 * device level, is done in userspace
		 * irqd_is_level_type() isn't used since isn't valid until
		 * irq is configured.
		 */
		if (irq_data &&
		    irqd_get_trigger_type(irq_data) & IRQ_TYPE_LEVEL_MASK) {
			dev_dbg(dev, "disable lazy unmask\n");
			irq_set_status_flags(uioinfo->irq, IRQ_DISABLE_UNLAZY);
		}
	}

	uiomem = &uioinfo->mem[0];

	for (i = 0; i < pdev->num_resources; ++i) {
		struct resource *r = &pdev->resource[i];

		if (r->flags != IORESOURCE_MEM)
			continue;

		if (uiomem >= &uioinfo->mem[MAX_UIO_MAPS]) {
			dev_warn(
				dev,
				"device has more than " __stringify(
					MAX_UIO_MAPS) " I/O memory resources.\n");
			break;
		}

		uiomem->memtype = UIO_MEM_PHYS;
		uiomem->addr = r->start & PAGE_MASK;
		uiomem->offs = r->start & ~PAGE_MASK;
		uiomem->size =
			(uiomem->offs + resource_size(r) + PAGE_SIZE - 1) &
			PAGE_MASK;
		uiomem->name = r->name;
		if (!strcmp(uiomem->name, "mcc-ctl")) {
			uiomem->internal_addr =
				ioremap_wc(uiomem->addr, uiomem->size);
			priv->ctl.mcctx_rd =
				uiomem->internal_addr + 0 * sizeof(u32);
			priv->ctl.mcctx_wr =
				uiomem->internal_addr + 1 * sizeof(u32);
			priv->ctl.mccrx_rd =
				uiomem->internal_addr + 2 * sizeof(u32);
			priv->ctl.mccrx_wr =
				uiomem->internal_addr + 3 * sizeof(u32);
		}

		dev_dbg(dev,
			"mem[%d], paddr=0x%08x, off=0x%08lx, size=0x%08x, iaddr=%p, name=%s\n",
			i, uiomem->addr, uiomem->offs, uiomem->size,
			uiomem->internal_addr, uiomem->name);

		++uiomem;
	}

	while (uiomem < &uioinfo->mem[MAX_UIO_MAPS]) {
		uiomem->size = 0;
		++uiomem;
	}

	/* This driver requires no hardware specific kernel code to handle
	 * interrupts. Instead, the interrupt handler simply disables the
	 * interrupt in the interrupt controller. User space is responsible
	 * for performing hardware specific acknowledge and re-enabling of
	 * the interrupt in the interrupt controller.
	 *
	 * Interrupt sharing is not supported.
	 */

	uioinfo->handler = uio_dsi_mcc_handler;
	uioinfo->irqcontrol = uio_dsi_mcc_irqcontrol;
	uioinfo->open = uio_dsi_mcc_open;
	uioinfo->release = uio_dsi_mcc_release;
	uioinfo->priv = priv;

	/* Enable Runtime PM for this device:
	 * The device starts in suspended state to allow the hardware to be
	 * turned off by default. The Runtime PM bus code should power on the
	 * hardware and enable clocks at open().
	 */
	pm_runtime_enable(dev);

	ret = devm_add_action_or_reset(dev, uio_dsi_mcc_cleanup, dev);
	if (ret)
		return ret;

	ret = uio_register_device(dev, priv->uioinfo);
	if (ret)
		dev_err(dev, "unable to register uio dsi_mcc device\n");

	return ret;
}

static int uio_dsi_mcc_runtime_nop(struct device *dev)
{
	/* Runtime PM callback shared between ->runtime_suspend()
	 * and ->runtime_resume(). Simply returns success.
	 *
	 * In this driver pm_runtime_get_sync() and pm_runtime_put_sync()
	 * are used at open() and release() time. This allows the
	 * Runtime PM code to turn off power to the device while the
	 * device is unused, ie before open() and after release().
	 *
	 * This Runtime PM callback does not need to save or restore
	 * any registers since user space is responsbile for hardware
	 * register reinitialization after open().
	 */
	return 0;
}

static const struct dev_pm_ops uio_dsi_mcc_dev_pm_ops = {
	.runtime_suspend = uio_dsi_mcc_runtime_nop,
	.runtime_resume = uio_dsi_mcc_runtime_nop,
};

/* export these or move hwspin_locks to uio device file uio.c*/
int uio_dsi_mcc_tx_offsets(void *priv, int *rd_offset, int *wr_offset)
{
	struct uio_dsi_mcc_platdata *p = priv;
	struct hwspinlock *hwlock = p->hwlock_tx;
	int err;
	int ret = 0;

	if (hwlock) {
		err = hwspin_lock_timeout_in_atomic(hwlock, HWSPNLCK_TIMEOUT);
		if (err) {
			pr_err("%s can't get rx hwspinlock (%d)\n", __func__,
			       err);
			ret = -1;
			goto hwunlock;
		}
	}
	/* read offsets*/
	if (!p->ctl.mcctx_rd || !p->ctl.mcctx_wr) {
		pr_err("%s mccctl not initalized\n", __func__);
		ret = -1;
		goto hwunlock;
	}
	*rd_offset = ioread32(p->ctl.mcctx_rd) & 0xffff;
	*wr_offset = ioread32(p->ctl.mcctx_wr) & 0xffff;

hwunlock:
	if (hwlock)
		hwspin_unlock_in_atomic(hwlock);
	return ret;
}
//EXPORT_SYMBOL_GPL(uio_dsi_mcc_tx_offsets);

int uio_dsi_mcc_rx_offsets(void *priv, int *rd_offset, int *wr_offset)
{
	struct uio_dsi_mcc_platdata *p = priv;
	struct hwspinlock *hwlock = p->hwlock_rx;
	int err;
	int ret = 0;

	if (hwlock) {
		err = hwspin_lock_timeout_in_atomic(hwlock, HWSPNLCK_TIMEOUT);
		if (err) {
			pr_err("%s can't get rx hwspinlock (%d)\n", __func__,
			       err);
			ret = -1;
			goto hwunlock;
		}
	}
	/* read offsets*/
	if (!p->ctl.mccrx_rd || !p->ctl.mccrx_wr) {
		pr_err("%s mccctl not initalized\n", __func__);
		ret = -1;
		goto hwunlock;
	}
	*rd_offset = ioread32(p->ctl.mccrx_rd);
	*wr_offset = ioread32(p->ctl.mccrx_wr);

hwunlock:
	if (hwlock)
		hwspin_unlock_in_atomic(hwlock);
	return ret;
}
//EXPORT_SYMBOL_GPL(uio_dsi_mcc_rx_offsets);

#ifdef CONFIG_OF
static struct of_device_id uio_of_genirq_match[] = {
	{ .compatible = "datum-uio-mcc" },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, uio_of_genirq_match);
module_param_string(of_id, uio_of_genirq_match[0].compatible, 128, 0);
MODULE_PARM_DESC(of_id, "Openfirmware id of the device to be handled by uio");
#endif

static struct platform_driver uio_dsi_mcc = {
	.probe = uio_dsi_mcc_probe,
	.driver =
		{
			.name = DRIVER_NAME,
			.pm = &uio_dsi_mcc_dev_pm_ops,
			.of_match_table = of_match_ptr(uio_of_genirq_match),
		},
};

module_platform_driver(uio_dsi_mcc);

MODULE_AUTHOR("Mark Carlin");
MODULE_DESCRIPTION("Userspace I/O MCC platform driver with IRQ handling");
MODULE_VERSION("0.0.1");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
