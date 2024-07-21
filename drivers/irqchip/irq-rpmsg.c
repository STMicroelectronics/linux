// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2023 STMicroelectronics.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/rpmsg.h>

#define VIRT_INTC_PEND		0x01U
#define VIRT_INTC_CFG		0x02U

#define IRQS_PER_BANK 32
#define TO_BANK_ID(_hwirq)	((_hwirq) / IRQS_PER_BANK)
#define TO_BANK_IRQ(_hwirq)	((_hwirq) % IRQS_PER_BANK)
#define RPMSG_INTC_TIMEOUT	msecs_to_jiffies(50)

struct rpmsg_intc_msg {
	u8 reg;
	u32 val;
	u32 bank_id;
};

struct rpmsg_intc_bank {
	u32 mask;		/* 1: activated 0: disabled */
};

struct rpmsg_irq_dev {
	struct platform_device	*pdev;
	atomic_t initialized;

	struct irq_domain *irqd;

	u32 nb_bank;
	struct rpmsg_intc_bank *banks;

	struct rpmsg_device *rpdev;
	struct rpmsg_driver rpdrv;

	struct completion complete;
	struct work_struct irqrpmsg_work;
};

#define to_rpmsg_driver(__d) container_of(__d, struct rpmsg_driver, drv)
#define to_rpmsg_irq(__d) container_of(__d, struct rpmsg_irq_dev, rpdrv)

static void irq_rpmsg_mask_irq(struct irq_data *d)
{
	struct rpmsg_irq_dev *rirq_dev = irq_data_get_irq_chip_data(d);
	struct rpmsg_intc_bank *bank = &rirq_dev->banks[TO_BANK_ID(d->hwirq)];

	bank->mask &= ~BIT(TO_BANK_IRQ(d->hwirq));
}

static void irq_rpmsg_unmask_irq(struct irq_data *d)
{
	struct rpmsg_irq_dev *rirq_dev = irq_data_get_irq_chip_data(d);
	struct rpmsg_intc_bank *bank = &rirq_dev->banks[TO_BANK_ID(d->hwirq)];

	bank->mask |= BIT(TO_BANK_IRQ(d->hwirq));
}

static struct irq_chip irq_rpmsg_chip = {
	.name		= "irq-rpmsg",
	.irq_mask	= irq_rpmsg_mask_irq,
	.irq_unmask	= irq_rpmsg_unmask_irq,
	.flags		= IRQCHIP_SKIP_SET_WAKE,
};

static int irq_rpmsg_map(struct irq_domain *d,
			 unsigned int virq, irq_hw_number_t hw)
{
	struct rpmsg_irq_dev *rirq_dev = d->host_data;

	if (!atomic_read(&rirq_dev->initialized))
		return -EPROBE_DEFER;

	irq_set_status_flags(virq, IRQ_LEVEL);
	irq_set_chip_and_handler(virq, &irq_rpmsg_chip, handle_level_irq);
	irq_set_chip_data(virq, d->host_data);
	irq_set_nested_thread(virq, 1);

	return 0;
}

static const struct irq_domain_ops irq_rpmsg_domain_ops = {
	.map = irq_rpmsg_map,
	.xlate = irq_domain_xlate_onecell,
};

static int irq_rpmsg_event(struct rpmsg_irq_dev *rirq_dev,
			   struct rpmsg_intc_msg *msg)
{
	struct rpmsg_intc_bank *bank = &rirq_dev->banks[msg->bank_id];
	unsigned long pending = msg->val;
	unsigned int irq_offset = msg->bank_id * IRQS_PER_BANK;
	u32 hwirq;

	if (msg->bank_id >= rirq_dev->nb_bank)
		return -EINVAL;

	pending &= bank->mask;

	for_each_set_bit(hwirq, &pending, IRQS_PER_BANK)
		handle_nested_irq(irq_find_mapping(rirq_dev->irqd,
						   irq_offset + hwirq));

	return 0;
}

static int irq_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	struct rpmsg_irq_dev *rirq_dev = dev_get_drvdata(&rpdev->dev);
	struct rpmsg_intc_msg *msg = data;
	int ret = 0;

	if (len != sizeof(struct rpmsg_intc_msg)) {
		dev_err(&rpdev->dev, "wrong msg size\n");
		return -EINVAL;
	}

	switch (msg->reg) {
	case VIRT_INTC_PEND:
		ret = irq_rpmsg_event(rirq_dev, msg);
		break;

	case VIRT_INTC_CFG:
		rirq_dev->nb_bank = msg->val;
		complete(&rirq_dev->complete);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int irq_rpmsg_read_val(struct rpmsg_irq_dev *rirq_dev,
			      struct rpmsg_intc_msg *msg)
{
	struct rpmsg_device *rpdev = rirq_dev->rpdev;
	int ret;

	init_completion(&rirq_dev->complete);
	ret = rpmsg_send(rpdev->ept, msg, sizeof(struct rpmsg_intc_msg));
	if (ret)
		return ret;

	if (!wait_for_completion_timeout(&rirq_dev->complete, RPMSG_INTC_TIMEOUT))
		return -ETIMEDOUT;

	return 0;
}

static void irq_rpmsg_setup_work(struct work_struct *ws)
{
	struct rpmsg_irq_dev *rirq_dev = container_of(ws, struct rpmsg_irq_dev,
						      irqrpmsg_work);
	struct rpmsg_device *rpdev = rirq_dev->rpdev;
	struct rpmsg_intc_msg msg = {VIRT_INTC_CFG, 0, 0};
	int ret;

	ret = irq_rpmsg_read_val(rirq_dev, &msg);
	if (ret) {
		dev_err(&rpdev->dev, "read intc cfg failed: %d\n", ret);
		goto err;
	}

	/*
	 * Workaround
	 * we can't manage IRQ domain creation here, this would leads to crash on remove.
	 * There is no devlink between the IRQ domain client and the rpmsg device, so the IRQ domain
	 * can be removed before the IRQs are freed.
	 */
	if (rirq_dev->nb_bank > 1) {
		dev_err(&rpdev->dev, "only one IRQ bank supported\n");
		goto err;
	}

	atomic_set(&rirq_dev->initialized, 1);

	return;

err:
	dev_err(&rpdev->dev, "unregister rpmsg device: %s\n", rpdev->id.name);
	device_unregister(&rpdev->dev);
	put_device(&rpdev->dev);
}

/*
 * WARNING
 * Not multi instance, all rpdev have same rpmsg_irq_dev
 */
static int irq_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_driver *rpdrv = to_rpmsg_driver(rpdev->dev.driver);
	struct rpmsg_irq_dev *rirq_dev = to_rpmsg_irq(rpdrv);

	/*
	 * WORKAROUND RPMsg
	 * The RPMsg device is call under RPMsg Name Service callback
	 * (irq context). Init and schedule a work to probe irq device in
	 * task context.
	 */
	rirq_dev->rpdev = rpdev;
	dev_set_drvdata(&rpdev->dev, rirq_dev);
	rpdev->dev.of_node = rirq_dev->pdev->dev.of_node;
	rpdev->dev.fwnode = rirq_dev->pdev->dev.fwnode;

	atomic_set(&rirq_dev->initialized, 0);

	INIT_WORK(&rirq_dev->irqrpmsg_work, irq_rpmsg_setup_work);
	schedule_work(&rirq_dev->irqrpmsg_work);

	return 0;
}

static void irq_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct rpmsg_irq_dev *rirq_dev = dev_get_drvdata(&rpdev->dev);

	cancel_work_sync(&rirq_dev->irqrpmsg_work);

	atomic_set(&rirq_dev->initialized, 0);
}

static struct rpmsg_device_id irq_rpmsg_ids[] = {
	{ .name	= "rpmsg-intc" },
	{ },
};

static struct rpmsg_driver rpmsg_irq_drv = {
	.drv.name	= "rpmsg_intc",
	.id_table	= irq_rpmsg_ids,
	.probe		= irq_rpmsg_probe,
	.callback	= irq_rpmsg_cb,
	.remove		= irq_rpmsg_remove,
};

static int irq_rpmsg_plat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpmsg_irq_dev *rirq_dev;
	int ret;

	rirq_dev = devm_kcalloc(dev, 1, sizeof(*rirq_dev), GFP_KERNEL);
	if (!rirq_dev)
		return -ENOMEM;

	rirq_dev->pdev = pdev;
	platform_set_drvdata(pdev, rirq_dev);

	rirq_dev->rpdrv = rpmsg_irq_drv;

	rirq_dev->banks = devm_kmalloc(dev, sizeof(struct rpmsg_intc_bank), GFP_KERNEL);
	if (!rirq_dev->banks)
		return -ENOMEM;

	rirq_dev->irqd = irq_domain_create_linear(dev->fwnode, IRQS_PER_BANK, &irq_rpmsg_domain_ops,
						  rirq_dev);
	if (!rirq_dev->irqd) {
		dev_err(dev, "Failed to create IRQ domain\n");
		return -EINVAL;
	}

	ret = register_rpmsg_driver(&rirq_dev->rpdrv);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register rpmsg drv\n");
		irq_domain_remove(rirq_dev->irqd);

		return ret;
	}

	return 0;
}

static int irq_rpmsg_plat_remove(struct platform_device *pdev)
{
	struct rpmsg_irq_dev *rirq_dev = platform_get_drvdata(pdev);

	unregister_rpmsg_driver(&rirq_dev->rpdrv);

	irq_domain_remove(rirq_dev->irqd);

	return 0;
}

static const struct of_device_id irq_rpmsg_plat_ids[] = {
	{ .compatible = "rpmsg,intc", },
	{},
};
MODULE_DEVICE_TABLE(of, irq_rpmsg_plat_ids);

static struct platform_driver irq_rpmsg_plat_driver = {
	.probe	= &irq_rpmsg_plat_probe,
	.remove = &irq_rpmsg_plat_remove,
	.driver = {
		.name	= "irq_rpmsg",
		.of_match_table	= of_match_ptr(irq_rpmsg_plat_ids),
	}
};

static int __init irq_rpmsg_init(void)
{
	return platform_driver_register(&irq_rpmsg_plat_driver);
}

static void __exit irq_rpmsg_exit(void)
{
	platform_driver_unregister(&irq_rpmsg_plat_driver);
}

subsys_initcall(irq_rpmsg_init);
module_exit(irq_rpmsg_exit);

MODULE_DESCRIPTION("RPMsg IRQC Driver");
MODULE_LICENSE("GPL v2");
