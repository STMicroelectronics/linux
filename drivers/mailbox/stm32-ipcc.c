// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2018 - All Rights Reserved
 * Authors: Ludovic Barre <ludovic.barre@st.com> for STMicroelectronics.
 *          Fabien Dessenne <fabien.dessenne@st.com> for STMicroelectronics.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/workqueue.h>

#define IPCC_XCR		0x000
#define XCR_RXOIE		BIT(0)
#define XCR_TXOIE		BIT(16)

#define IPCC_XMR		0x004
#define IPCC_XSCR		0x008
#define IPCC_XTOYSR		0x00c

#define IPCC_PROC_OFFST		0x010

#define IPCC_SECCFGR(x)		(x ? 0x90 : 0x80)
#define IPCC_PRIVCFGR(x)	(x ? 0x94 : 0x84)

#define IPCC_CIDCFGR(x)		(x ? 0x98 : 0x88)
#define IPCC_CIDCFGR_CFEN_MASK	BIT(0)
#define IPCC_CIDCFGR_SCID_MASK	GENMASK(7, 4)

#define IPCC_HWCFGR		0x3f0
#define IPCFGR_CHAN_MASK	GENMASK(7, 0)
#define IPCFGR_SECCTRL_MASK	GENMASK(11, 8)
#define IPCFGR_CIDWIDTH_MASK	GENMASK(27, 24)

#define IPCC_VER		0x3f4
#define VER_MINREV_MASK		GENMASK(3, 0)
#define VER_MAJREV_MASK		GENMASK(7, 4)

#define RX_BIT_MASK		GENMASK(15, 0)
#define RX_BIT_CHAN(chan)	BIT(chan)
#define TX_BIT_SHIFT		16
#define TX_BIT_MASK		GENMASK(31, 16)
#define TX_BIT_CHAN(chan)	BIT(TX_BIT_SHIFT + (chan))

#define STM32_MAX_PROCS		2

#define IPCC_CORE_ID		1

/* Use virtual ID by setting bit 8 of a channel ID to keep interrupt context for client */
#define STM32_IPCC_CH_ID_MASK		GENMASK(7, 0)
#define STM32_IPCC_CH_HIGH_PRIO_MASK	GENMASK(8, 8)

enum {
	IPCC_IRQ_RX,
	IPCC_IRQ_TX,
	IPCC_IRQ_NUM,
};

struct stm32_ipcc_ch {
	struct mbox_chan   *mbox;
	struct work_struct rx_work;
	unsigned long      chan;
	bool               irq_ctx;
};

struct stm32_ipcc {
	struct mbox_controller controller;
	void __iomem *reg_base;
	void __iomem *reg_proc;
	struct clk *clk;
	struct stm32_ipcc_ch *chnl;
	struct workqueue_struct *workqueue;
	spinlock_t lock; /* protect access to IPCC registers */
	int irqs[IPCC_IRQ_NUM];
	u32 proc_id;
	u32 n_chans;
	u32 xcr;
	u32 xmr;
	u32 sec_mode;
};

static inline void stm32_ipcc_set_bits(spinlock_t *lock, void __iomem *reg,
				       u32 mask)
{
	unsigned long flags;

	spin_lock_irqsave(lock, flags);
	writel_relaxed(readl_relaxed(reg) | mask, reg);
	spin_unlock_irqrestore(lock, flags);
}

static inline void stm32_ipcc_clr_bits(spinlock_t *lock, void __iomem *reg,
				       u32 mask)
{
	unsigned long flags;

	spin_lock_irqsave(lock, flags);
	writel_relaxed(readl_relaxed(reg) & ~mask, reg);
	spin_unlock_irqrestore(lock, flags);
}

/*
 * Message receiver(workqueue)
 */
static void stm32_ipcc_rx_work(struct work_struct *work)
{
	struct stm32_ipcc_ch *chnl = container_of(work, struct stm32_ipcc_ch, rx_work);

	mbox_chan_received_data(chnl->mbox, NULL);
};

static irqreturn_t stm32_ipcc_rx_irq(int irq, void *data)
{
	struct stm32_ipcc *ipcc = data;
	struct device *dev = ipcc->controller.dev;
	struct stm32_ipcc_ch *chnl;
	u32 status, mr, tosr, chan;
	irqreturn_t ret = IRQ_NONE;
	int proc_offset;

	/* read 'channel occupied' status from other proc */
	proc_offset = ipcc->proc_id ? -IPCC_PROC_OFFST : IPCC_PROC_OFFST;
	tosr = readl_relaxed(ipcc->reg_proc + proc_offset + IPCC_XTOYSR);
	mr = readl_relaxed(ipcc->reg_proc + IPCC_XMR);

	/* search for unmasked 'channel occupied' */
	status = tosr & FIELD_GET(RX_BIT_MASK, ~mr);

	for (chan = 0; chan < ipcc->n_chans; chan++) {
		if (!(status & (1 << chan)))
			continue;

		dev_dbg(dev, "%s: chan:%d rx\n", __func__, chan);

		chnl = &ipcc->chnl[chan];

		/*
		 * Depending on the DT parameter,call the client under interrupt context
		 * or use workqueue to call the callback in normal context
		 */

		if (chnl->irq_ctx)
			mbox_chan_received_data(chnl->mbox, NULL);
		else
			queue_work(ipcc->workqueue, &chnl->rx_work);

		stm32_ipcc_set_bits(&ipcc->lock, ipcc->reg_proc + IPCC_XSCR,
				    RX_BIT_CHAN(chan));

		ret = IRQ_HANDLED;
	}

	return ret;
}

static irqreturn_t stm32_ipcc_tx_irq(int irq, void *data)
{
	struct stm32_ipcc *ipcc = data;
	struct device *dev = ipcc->controller.dev;
	u32 status, mr, tosr, chan;
	irqreturn_t ret = IRQ_NONE;

	tosr = readl_relaxed(ipcc->reg_proc + IPCC_XTOYSR);
	mr = readl_relaxed(ipcc->reg_proc + IPCC_XMR);

	/* search for unmasked 'channel free' */
	status = ~tosr & FIELD_GET(TX_BIT_MASK, ~mr);

	for (chan = 0; chan < ipcc->n_chans ; chan++) {
		if (!(status & (1 << chan)))
			continue;

		dev_dbg(dev, "%s: chan:%d tx\n", __func__, chan);

		/* mask 'tx channel free' interrupt */
		stm32_ipcc_set_bits(&ipcc->lock, ipcc->reg_proc + IPCC_XMR,
				    TX_BIT_CHAN(chan));

		mbox_chan_txdone(&ipcc->controller.chans[chan], 0);

		ret = IRQ_HANDLED;
	}

	return ret;
}

static int stm32_ipcc_send_data(struct mbox_chan *link, void *data)
{
	struct stm32_ipcc_ch *chnl = (struct stm32_ipcc_ch *)link->con_priv;
	unsigned long chan = chnl->chan;
	struct stm32_ipcc *ipcc = container_of(link->mbox, struct stm32_ipcc,
					       controller);

	dev_dbg(ipcc->controller.dev, "%s: chan:%lu\n", __func__, chan);

	/* set channel n occupied */
	stm32_ipcc_set_bits(&ipcc->lock, ipcc->reg_proc + IPCC_XSCR,
			    TX_BIT_CHAN(chan));

	/* unmask 'tx channel free' interrupt */
	stm32_ipcc_clr_bits(&ipcc->lock, ipcc->reg_proc + IPCC_XMR,
			    TX_BIT_CHAN(chan));

	return 0;
}

static int stm32_ipcc_startup(struct mbox_chan *link)
{
	struct stm32_ipcc_ch *chnl = (struct stm32_ipcc_ch *)link->con_priv;
	unsigned long chan = chnl->chan;
	struct stm32_ipcc *ipcc = container_of(link->mbox, struct stm32_ipcc,
					       controller);
	int ret;

	ret = clk_prepare_enable(ipcc->clk);
	if (ret) {
		dev_err(ipcc->controller.dev, "can not enable the clock\n");
		return ret;
	}

	/* unmask 'rx channel occupied' interrupt */
	stm32_ipcc_clr_bits(&ipcc->lock, ipcc->reg_proc + IPCC_XMR,
			    RX_BIT_CHAN(chan));

	return 0;
}

static void stm32_ipcc_shutdown(struct mbox_chan *link)
{
	struct stm32_ipcc_ch *chnl = (struct stm32_ipcc_ch *)link->con_priv;
	unsigned long chan = chnl->chan;
	struct stm32_ipcc *ipcc = container_of(link->mbox, struct stm32_ipcc,
					       controller);

	/* mask rx/tx interrupt */
	stm32_ipcc_set_bits(&ipcc->lock, ipcc->reg_proc + IPCC_XMR,
			    RX_BIT_CHAN(chan) | TX_BIT_CHAN(chan));

	if (!chnl->irq_ctx)
		flush_work(&chnl->rx_work);

	clk_disable_unprepare(ipcc->clk);
}

static int stm32_ipcc_check_rif(struct stm32_ipcc *ipcc, unsigned long chan)
{
	struct device *dev = ipcc->controller.dev;
	u32 cfgr;

	if (!ipcc->sec_mode)
		return 0;

	cfgr = readl_relaxed(ipcc->reg_base + IPCC_SECCFGR(ipcc->proc_id));
	if (cfgr & BIT(chan)) {
		dev_err(dev, "Unexpected secure configuration for chan %lu\n", chan);
		return -EACCES;
	}

	/*
	 * The privilege state is not checked as the client driver has privileged access right.
	 * It is only read for debug information.
	 */
	cfgr = readl_relaxed(ipcc->reg_base + IPCC_PRIVCFGR(ipcc->proc_id));
	dev_dbg(dev, "chan %lu is configured as %sprivileged\n", chan,
		(cfgr & BIT(chan)) ? "" : "un");

	return 0;
}

static const struct mbox_chan_ops stm32_ipcc_ops = {
	.send_data	= stm32_ipcc_send_data,
	.startup	= stm32_ipcc_startup,
	.shutdown	= stm32_ipcc_shutdown,
};

static struct mbox_chan *stm32_ipcc_xlate(struct mbox_controller *mbox,
					  const struct of_phandle_args *sp)
{
	u32 ind = sp->args[0] & STM32_IPCC_CH_ID_MASK;
	struct stm32_ipcc *ipcc = container_of(mbox, struct stm32_ipcc, controller);
	struct stm32_ipcc_ch *chnl;
	int ret;

	if (ind >= mbox->num_chans)
		return ERR_PTR(-EINVAL);

	chnl = (struct stm32_ipcc_ch *)mbox->chans[ind].con_priv;
	ret = stm32_ipcc_check_rif(ipcc, ind);
	if (ret)
		return ERR_PTR(ret);

	chnl->mbox = &mbox->chans[ind];
	chnl->chan = ind;
	chnl->irq_ctx = !!(sp->args[0] & STM32_IPCC_CH_HIGH_PRIO_MASK);

	if (!chnl->irq_ctx)
		INIT_WORK(&chnl->rx_work, stm32_ipcc_rx_work);

	return &mbox->chans[ind];
}

static int stm32_ipcc_probe(struct platform_device *pdev)
{
	static const char * const irq_name[] = {"rx", "tx"};
	irq_handler_t irq_thread[] = {stm32_ipcc_rx_irq, stm32_ipcc_tx_irq};
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct stm32_ipcc *ipcc;
	u32 ip_ver, hwcfg, cidcfgr, cid, cid_mask, cfen;
	unsigned long i;
	int ret;

	if (!np) {
		dev_err(dev, "No DT found\n");
		return -ENODEV;
	}

	ipcc = devm_kzalloc(dev, sizeof(*ipcc), GFP_KERNEL);
	if (!ipcc)
		return -ENOMEM;

	spin_lock_init(&ipcc->lock);

	/* proc_id */
	if (of_property_read_u32(np, "st,proc-id", &ipcc->proc_id)) {
		dev_err(dev, "Missing st,proc-id\n");
		return -ENODEV;
	}

	if (ipcc->proc_id >= STM32_MAX_PROCS) {
		dev_err(dev, "Invalid proc_id (%d)\n", ipcc->proc_id);
		return -EINVAL;
	}

	/* regs */
	ipcc->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ipcc->reg_base))
		return PTR_ERR(ipcc->reg_base);

	ipcc->reg_proc = ipcc->reg_base + ipcc->proc_id * IPCC_PROC_OFFST;

	/* clock */
	ipcc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(ipcc->clk))
		return PTR_ERR(ipcc->clk);

	ret = clk_prepare_enable(ipcc->clk);
	if (ret) {
		dev_err(dev, "can not enable the clock\n");
		return ret;
	}

	/* irq */
	for (i = 0; i < IPCC_IRQ_NUM; i++) {
		ipcc->irqs[i] = platform_get_irq_byname(pdev, irq_name[i]);
		if (ipcc->irqs[i] < 0) {
			ret = ipcc->irqs[i];
			goto err_clk;
		}

		ret = devm_request_threaded_irq(dev, ipcc->irqs[i], NULL,
						irq_thread[i], IRQF_ONESHOT,
						dev_name(dev), ipcc);
		if (ret) {
			dev_err(dev, "failed to request irq %lu (%d)\n", i, ret);
			goto err_clk;
		}
	}

	hwcfg = readl_relaxed(ipcc->reg_base + IPCC_HWCFGR);
	ipcc->n_chans = FIELD_GET(IPCFGR_CHAN_MASK, hwcfg);
	ipcc->sec_mode = FIELD_GET(IPCFGR_SECCTRL_MASK, hwcfg);
	cid_mask = FIELD_GET(IPCFGR_CIDWIDTH_MASK, hwcfg);

	/* Check RIF CID*/
	if (cid_mask) {
		cidcfgr = readl(ipcc->reg_base + IPCC_CIDCFGR(ipcc->proc_id));
		cid = FIELD_GET(IPCC_CIDCFGR_SCID_MASK, cidcfgr);
		cfen = FIELD_GET(IPCC_CIDCFGR_CFEN_MASK, cidcfgr);
		if (cfen && ((cid & cid_mask) != IPCC_CORE_ID)) {
			dev_err(dev, "Unexpected CID%u, no access right\n", cid & cid_mask);
			ret = -EPERM;
			goto err_init_wkp;
		}
	}

	/*
	 * Mask and enable all rx/tx irq.
	 * If CHnSEC bit is enabled, the CHnFM bit will not be updated by hardware
	 */
	stm32_ipcc_set_bits(&ipcc->lock, ipcc->reg_proc + IPCC_XMR,
			    RX_BIT_MASK | TX_BIT_MASK);
	stm32_ipcc_set_bits(&ipcc->lock, ipcc->reg_proc + IPCC_XCR,
			    XCR_RXOIE | XCR_TXOIE);

	/* wakeup */
	if (of_property_read_bool(np, "wakeup-source")) {
		device_set_wakeup_capable(dev, true);

		ret = dev_pm_set_wake_irq(dev, ipcc->irqs[IPCC_IRQ_RX]);
		if (ret) {
			dev_err(dev, "Failed to set wake up irq\n");
			goto err_init_wkp;
		}
	}

	ipcc->controller.dev = dev;
	ipcc->controller.txdone_irq = true;
	ipcc->controller.ops = &stm32_ipcc_ops;
	ipcc->controller.of_xlate = &stm32_ipcc_xlate;
	ipcc->controller.num_chans = ipcc->n_chans;
	ipcc->controller.chans = devm_kcalloc(dev, ipcc->controller.num_chans,
					      sizeof(*ipcc->controller.chans),
					      GFP_KERNEL);
	if (!ipcc->controller.chans) {
		ret = -ENOMEM;
		goto err_irq_wkp;
	}

	ipcc->chnl = devm_kcalloc(dev, ipcc->controller.num_chans,
				  sizeof(*ipcc->chnl), GFP_KERNEL);
	if (!ipcc->chnl) {
		ret = -ENOMEM;
		goto err_irq_wkp;
	}

	for (i = 0; i < ipcc->controller.num_chans; i++)
		ipcc->controller.chans[i].con_priv = (void *)&ipcc->chnl[i];

	ret = devm_mbox_controller_register(dev, &ipcc->controller);
	if (ret)
		goto err_irq_wkp;

	ipcc->workqueue = create_workqueue(dev_name(dev));
	if (!ipcc->workqueue) {
		dev_err(dev, "cannot create workqueue\n");
		ret = -ENOMEM;
		goto err_irq_wkp;
	}

	platform_set_drvdata(pdev, ipcc);

	ip_ver = readl_relaxed(ipcc->reg_base + IPCC_VER);

	dev_info(dev, "ipcc rev:%ld.%ld enabled, %d chans, proc %d\n",
		 FIELD_GET(VER_MAJREV_MASK, ip_ver),
		 FIELD_GET(VER_MINREV_MASK, ip_ver),
		 ipcc->controller.num_chans, ipcc->proc_id);

	clk_disable_unprepare(ipcc->clk);
	return 0;

err_irq_wkp:
	if (of_property_read_bool(np, "wakeup-source"))
		dev_pm_clear_wake_irq(dev);
err_init_wkp:
	device_set_wakeup_capable(dev, false);
err_clk:
	clk_disable_unprepare(ipcc->clk);
	return ret;
}

static int stm32_ipcc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stm32_ipcc *ipcc = dev_get_drvdata(dev);

	if (of_property_read_bool(dev->of_node, "wakeup-source"))
		dev_pm_clear_wake_irq(&pdev->dev);

	destroy_workqueue(ipcc->workqueue);

	device_set_wakeup_capable(dev, false);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int stm32_ipcc_suspend(struct device *dev)
{
	struct stm32_ipcc *ipcc = dev_get_drvdata(dev);

	ipcc->xmr = readl_relaxed(ipcc->reg_proc + IPCC_XMR);
	ipcc->xcr = readl_relaxed(ipcc->reg_proc + IPCC_XCR);

	return 0;
}

static int stm32_ipcc_resume(struct device *dev)
{
	struct stm32_ipcc *ipcc = dev_get_drvdata(dev);

	writel_relaxed(ipcc->xmr, ipcc->reg_proc + IPCC_XMR);
	writel_relaxed(ipcc->xcr, ipcc->reg_proc + IPCC_XCR);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(stm32_ipcc_pm_ops,
			 stm32_ipcc_suspend, stm32_ipcc_resume);

static const struct of_device_id stm32_ipcc_of_match[] = {
	{ .compatible = "st,stm32mp1-ipcc" },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_ipcc_of_match);

static struct platform_driver stm32_ipcc_driver = {
	.driver = {
		.name = "stm32-ipcc",
		.pm = &stm32_ipcc_pm_ops,
		.of_match_table = stm32_ipcc_of_match,
	},
	.probe		= stm32_ipcc_probe,
	.remove		= stm32_ipcc_remove,
};

module_platform_driver(stm32_ipcc_driver);

MODULE_AUTHOR("Ludovic Barre <ludovic.barre@st.com>");
MODULE_AUTHOR("Fabien Dessenne <fabien.dessenne@st.com>");
MODULE_DESCRIPTION("STM32 IPCC driver");
MODULE_LICENSE("GPL v2");
