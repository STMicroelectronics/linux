// SPDX-License-Identifier: GPL
/*
 * Copyright (C) 2022, STMicroelectronics - All Rights Reserved
 * Author(s): Patrice Chotard <patrice.chotard@foss.st.com> for STMicroelectronics.
 */

#include <memory/stm32-omi.h>

static void stm32_omi_read_fifo(u8 *val, void __iomem *addr)
{
	*val = readb_relaxed(addr);
}

static void stm32_omi_write_fifo(u8 *val, void __iomem *addr)
{
	writeb_relaxed(*val, addr);
}

int stm32_omi_abort(struct stm32_omi *omi)
{
	void __iomem *regs_base = omi->regs_base;
	u32 cr;
	int timeout;

	cr = readl_relaxed(regs_base + OSPI_CR) | CR_ABORT;
	writel_relaxed(cr, regs_base + OSPI_CR);

	/* wait clear of abort bit by hw */
	timeout = readl_relaxed_poll_timeout_atomic(regs_base + OSPI_CR,
						    cr, !(cr & CR_ABORT), 1,
						    STM32_ABT_TIMEOUT_US);

	if (timeout && !omi->calibration)
		dev_err(omi->dev, "%s abort timeout:%d\n", __func__, timeout);

	return timeout;
}
EXPORT_SYMBOL(stm32_omi_abort);

int stm32_omi_tx_poll(struct stm32_omi *omi, u8 *buf, u32 len, bool read)
{
	void __iomem *regs_base = omi->regs_base;
	void (*tx_fifo)(u8 *val, void __iomem *addr);
	u32 sr;
	int ret;

	if (read)
		tx_fifo = stm32_omi_read_fifo;
	else
		tx_fifo = stm32_omi_write_fifo;

	while (len--) {
		ret = readl_relaxed_poll_timeout_atomic(regs_base + OSPI_SR,
							sr, sr & SR_FTF, 1,
							STM32_FIFO_TIMEOUT_US);
		if (ret) {
			if (!omi->calibration)
				dev_err(omi->dev,
					"fifo timeout (len:%d stat:%#x)\n",
					len, sr);
			return ret;
		}
		tx_fifo(buf++, regs_base + OSPI_DR);
	}

	return 0;
}
EXPORT_SYMBOL(stm32_omi_tx_poll);

int stm32_omi_wait_nobusy(struct stm32_omi *omi)
{
	u32 sr;

	return readl_relaxed_poll_timeout_atomic(omi->regs_base + OSPI_SR,
						 sr, !(sr & SR_BUSY), 1,
						 STM32_BUSY_TIMEOUT_US);
}
EXPORT_SYMBOL(stm32_omi_wait_nobusy);

int stm32_omi_wait_cmd(struct stm32_omi *omi)
{
	void __iomem *regs_base = omi->regs_base;
	u32 cr, sr;
	int err = 0;

	if ((readl_relaxed(regs_base + OSPI_SR) & SR_TCF) ||
	    omi->fmode == CR_FMODE_APM)
		goto out;

	reinit_completion(&omi->data_completion);
	cr = readl_relaxed(regs_base + OSPI_CR);
	writel_relaxed(cr | CR_TCIE | CR_TEIE, regs_base + OSPI_CR);

	if (!wait_for_completion_timeout(&omi->data_completion,
				msecs_to_jiffies(STM32_COMP_TIMEOUT_MS))) {
		err = -ETIMEDOUT;
	} else {
		sr = readl_relaxed(regs_base + OSPI_SR);
		if (sr & SR_TEF)
			err = -EIO;
	}

out:
	/* clear flags */
	writel_relaxed(FCR_CTCF | FCR_CTEF, regs_base + OSPI_FCR);

	if (!err)
		err = stm32_omi_wait_nobusy(omi);

	return err;
}
EXPORT_SYMBOL(stm32_omi_wait_cmd);

void stm32_omi_dma_callback(void *arg)
{
	struct completion *dma_completion = arg;

	complete(dma_completion);
}
EXPORT_SYMBOL(stm32_omi_dma_callback);

static irqreturn_t stm32_omi_irq(int irq, void *dev_id)
{
	struct stm32_omi *omi = (struct stm32_omi *)dev_id;
	void __iomem *regs_base = omi->regs_base;
	u32 cr, sr;

	cr = readl_relaxed(regs_base + OSPI_CR);
	sr = readl_relaxed(regs_base + OSPI_SR);

	if (cr & CR_SMIE && sr & SR_SMF) {
		/* disable irq */
		cr &= ~CR_SMIE;
		writel_relaxed(cr, regs_base + OSPI_CR);
		complete(&omi->match_completion);

		return IRQ_HANDLED;
	}

	if (sr & (SR_TEF | SR_TCF)) {
		/* disable irq */
		cr = readl_relaxed(regs_base + OSPI_CR);
		cr &= ~CR_TCIE & ~CR_TEIE;
		writel_relaxed(cr, regs_base + OSPI_CR);
		complete(&omi->data_completion);
	}

	return IRQ_HANDLED;
}

void stm32_omi_dma_setup(struct stm32_omi *omi,
			 struct dma_slave_config *dma_cfg)
{
	if (dma_cfg && omi->dma_chrx) {
		if (dmaengine_slave_config(omi->dma_chrx, dma_cfg)) {
			dev_err(omi->dev, "dma rx config failed\n");
			dma_release_channel(omi->dma_chrx);
			omi->dma_chrx = NULL;
		}
	}

	if (dma_cfg && omi->dma_chtx) {
		if (dmaengine_slave_config(omi->dma_chtx, dma_cfg)) {
			dev_err(omi->dev, "dma tx config failed\n");
			dma_release_channel(omi->dma_chtx);
			omi->dma_chtx = NULL;
		}
	}

	init_completion(&omi->dma_completion);
}
EXPORT_SYMBOL(stm32_omi_dma_setup);

static int stm32_omi_dlyb_set_tap(struct stm32_omi *omi, u8 tap, bool rx_tap)
{
	u32 sr, mask, ack;
	int ret;
	u8 shift;

	if (rx_tap) {
		mask = DLYBOS_CR_RXTAPSEL_MASK;
		shift = DLYBOS_CR_RXTAPSEL_SHIFT;
		ack = DLYBOS_SR_RXTAPSEL_ACK;
	} else {
		mask = DLYBOS_CR_TXTAPSEL_MASK;
		shift = DLYBOS_CR_TXTAPSEL_SHIFT;
		ack = DLYBOS_SR_TXTAPSEL_ACK;
	}

	regmap_update_bits(omi->regmap,
			   omi->dlyb_base + SYSCFG_DLYBOS_CR,
			   mask, mask & (tap << shift));

	ret = regmap_read_poll_timeout(omi->regmap,
				       omi->dlyb_base + SYSCFG_DLYBOS_SR,
				       sr, sr & ack, 1,
				       1000 * STM32_DLYBOS_TIMEOUT_MS);
	if (ret)
		dev_err(omi->dev, "%s delay Block phase configuration timeout\n",
			rx_tap ? "RX" : "TX");

	return ret;
}

int stm32_omi_dlyb_find_tap(struct stm32_omi *omi, bool rx_only, u8 *window_len)
{
	struct stm32_tap_window rx_tap_w[DLYBOS_TAPSEL_NB];
	int ret = 0;
	u8 rx_len, rx_window_len = 0, rx_window_end = 0;
	u8 tx_len, tx_window_len, tx_window_end;
	u8 rx_tap, tx_tap, tx_tap_max, tx_tap_min, best_tx_tap = 0;
	u8 score, score_max;

	tx_len = 0;
	tx_window_len = 0;
	tx_window_end = 0;

	for (tx_tap = 0;
	     tx_tap < (rx_only ? 1 : DLYBOS_TAPSEL_NB);
	     tx_tap++) {
		ret = stm32_omi_dlyb_set_tap(omi, tx_tap, false);
		if (ret)
			return ret;

		rx_len = 0;
		rx_window_len = 0;
		rx_window_end = 0;

		for (rx_tap = 0; rx_tap < DLYBOS_TAPSEL_NB; rx_tap++) {
			ret = stm32_omi_dlyb_set_tap(omi, rx_tap, true);
			if (ret)
				return ret;

			omi->calibration = true;
			ret = omi->check_transfer(omi);
			omi->calibration = false;
			if (ret) {
				if ((!rx_only && ret == -ETIMEDOUT) ||
				    ret == -EOPNOTSUPP)
					break;

				rx_len = 0;
			} else {
				rx_len++;
				if (rx_len > rx_window_len) {
					rx_window_len = rx_len;
					rx_window_end = rx_tap;
				}
			}
		}

		if (ret == -EOPNOTSUPP)
			break;

		rx_tap_w[tx_tap].end = rx_window_end;
		rx_tap_w[tx_tap].length = rx_window_len;

		if (!rx_window_len) {
			tx_len = 0;
		} else {
			tx_len++;
			if (tx_len > tx_window_len) {
				tx_window_len = tx_len;
				tx_window_end = tx_tap;
			}
		}
		dev_dbg(omi->dev, "rx_tap_w[%02d].end = %d rx_tap_w[%02d].length = %d\n",
			tx_tap, rx_tap_w[tx_tap].end, tx_tap, rx_tap_w[tx_tap].length);
	}

	if (ret == -EOPNOTSUPP) {
		dev_err(omi->dev, "Calibration can not be done on this device\n");
		return ret;
	}

	if (rx_only) {
		if (!rx_window_len) {
			dev_err(omi->dev, "Can't find RX phase settings\n");
			return -EIO;
		}

		rx_tap = rx_window_end - rx_window_len / 2;
		*window_len = rx_window_len;
		dev_dbg(omi->dev, "RX_TAP_SEL set to %d\n", rx_tap);

		return stm32_omi_dlyb_set_tap(omi, rx_tap, true);
	}

	/* find the best duet TX/RX TAP */
	tx_tap_min = tx_window_end - tx_window_len + 1;
	tx_tap_max = tx_window_end;
	score_max = 0;
	for (tx_tap = tx_tap_min; tx_tap <= tx_tap_max; tx_tap++) {
		score = min_t(u8, tx_tap - tx_tap_min, tx_tap_max - tx_tap) +
			rx_tap_w[tx_tap].length;
		if (score > score_max) {
			score_max = score;
			best_tx_tap = tx_tap;
		}
	}

	rx_tap = rx_tap_w[best_tx_tap].end - rx_tap_w[best_tx_tap].length / 2;

	dev_dbg(omi->dev, "RX_TAP_SEL set to %d\n", rx_tap);
	ret = stm32_omi_dlyb_set_tap(omi, rx_tap, true);
	if (ret)
		return ret;

	dev_dbg(omi->dev, "TX_TAP_SEL set to %d\n", best_tx_tap);

	return stm32_omi_dlyb_set_tap(omi, best_tx_tap, false);
}
EXPORT_SYMBOL(stm32_omi_dlyb_find_tap);

int stm32_omi_dlyb_set_cr(struct stm32_omi *omi, u32 dlyb_cr)
{
	bool bypass_mode = false;
	int ret;
	u16 period_ps;
	u8 rx_tap, tx_tap;

	period_ps = FIELD_GET(DLYBOS_BYP_CMD_MASK, dlyb_cr);
	if ((dlyb_cr & DLYBOS_BYP_EN) == DLYBOS_BYP_EN)
		bypass_mode = true;

	ret = stm32_omi_dlyb_init(omi, bypass_mode, period_ps);
	if (ret)
		return ret;

	/* restore Rx and TX tap */
	rx_tap = FIELD_GET(DLYBOS_CR_RXTAPSEL_MASK, dlyb_cr);
	ret = stm32_omi_dlyb_set_tap(omi, rx_tap, true);
	if (ret)
		return ret;

	tx_tap = FIELD_GET(DLYBOS_CR_TXTAPSEL_MASK, dlyb_cr);
	return stm32_omi_dlyb_set_tap(omi, tx_tap, false);
}
EXPORT_SYMBOL(stm32_omi_dlyb_set_cr);

void stm32_omi_dlyb_get_cr(struct stm32_omi *omi, u32 *dlyb_cr)
{
	regmap_read(omi->regmap, omi->dlyb_base + SYSCFG_DLYBOS_CR, dlyb_cr);
}
EXPORT_SYMBOL(stm32_omi_dlyb_get_cr);

/* ½ memory clock period in pico second */
static const u16 dlybos_delay_ps[STM32_DLYBOS_DELAY_NB] = {
2816, 4672, 6272, 7872, 9472, 11104, 12704, 14304, 15904, 17536, 19136, 20736,
22336, 23968, 25568, 27168, 28768, 30400, 32000, 33600, 35232, 36832, 38432, 40032
};

static u32 stm32_omi_find_byp_cmd(u16 period_ps)
{
	u16 half_period_ps = period_ps / 2;
	u8 max = STM32_DLYBOS_DELAY_NB - 1;
	u8 i, min = 0;

	/* find closest value in dlybos_delay_ps[] with half_period_ps*/
	if (half_period_ps < dlybos_delay_ps[0])
		return FIELD_PREP(DLYBOS_BYP_CMD_MASK, 1);

	if (half_period_ps > dlybos_delay_ps[max])
		return FIELD_PREP(DLYBOS_BYP_CMD_MASK, STM32_DLYBOS_DELAY_NB);

	while (max - min > 1) {
		i = DIV_ROUND_UP(min + max, 2);
		if (half_period_ps > dlybos_delay_ps[i])
			min = i;
		else
			max = i;
	}

	if ((dlybos_delay_ps[max] - half_period_ps) >
	    (half_period_ps - dlybos_delay_ps[min]))
		return FIELD_PREP(DLYBOS_BYP_CMD_MASK, min + 1);
	else
		return FIELD_PREP(DLYBOS_BYP_CMD_MASK, max + 1);
}

void stm32_omi_dlyb_stop(struct stm32_omi *omi)
{
	/* disable delay block */
	regmap_write(omi->regmap, omi->dlyb_base + SYSCFG_DLYBOS_CR, 0x0);
}
EXPORT_SYMBOL(stm32_omi_dlyb_stop);

int stm32_omi_dlyb_init(struct stm32_omi *omi, bool bypass_mode,
			u16 period_ps)
{
	u32 sr, val, mask;
	int ret;

	if (bypass_mode) {
		val = DLYBOS_BYP_EN;
		val |= stm32_omi_find_byp_cmd(period_ps);
		mask = DLYBOS_BYP_CMD_MASK | DLYBOS_BYP_EN;
	} else {
		val = DLYBOS_CR_EN;
		mask = DLYBOS_CR_EN;
	}

	/* enable Delay Block */
	regmap_update_bits(omi->regmap, omi->dlyb_base + SYSCFG_DLYBOS_CR,
			   mask, val);

	if (bypass_mode)
		return 0;

	/* in lock mode, wait for lock status bit */
	ret = regmap_read_poll_timeout(omi->regmap,
				       omi->dlyb_base + SYSCFG_DLYBOS_SR,
				       sr, sr & DLYBOS_SR_LOCK, 1,
				       1000 * STM32_DLYBOS_TIMEOUT_MS);
	if (ret) {
		dev_err(omi->dev, "Delay Block lock timeout\n");
		stm32_omi_dlyb_stop(omi);
	}

	return ret;
}
EXPORT_SYMBOL(stm32_omi_dlyb_init);

static int stm32_omi_probe(struct platform_device *pdev)
{
	struct platform_device *vdev;
	struct device *dev = &pdev->dev;
	struct device_node *child;
	struct stm32_omi *omi;
	struct resource *res;
	struct reserved_mem *rmem = NULL;
	struct device_node *node;
	const char *name;
	u8 hyperflash_count = 0;
	u8 spi_flash_count = 0;
	u8 child_count = 0;
	int ret;
	bool jedec_flash = false;

	/*
	 * Flash subnodes sanity check:
	 *        2 spi-nand/spi-nor flashes			=> supported
	 *        1 HyperFlash					=> supported
	 *	  All other flash node configuration		=> not supported
	 */
	for_each_available_child_of_node(dev->of_node, child) {
		if (of_device_is_compatible(child, "cfi-flash"))
			hyperflash_count++;

		if (of_device_is_compatible(child, "jedec-flash")) {
			hyperflash_count++;
			jedec_flash = true;
		}

		if (of_device_is_compatible(child, "jedec,spi-nor") ||
		    of_device_is_compatible(child, "spi-nand") ||
		    of_device_is_compatible(child, "st,octal-bus"))
			spi_flash_count++;

		child_count++;
	}

	if (!child_count) {
		dev_err(dev, "Missing flash node\n");
		return -ENODEV;
	}

	if ((!hyperflash_count && !spi_flash_count) ||
	    child_count != (hyperflash_count + spi_flash_count)) {
		dev_err(dev, "Unknown flash type\n");
		return -ENODEV;
	}

	if ((hyperflash_count && spi_flash_count) ||
	     hyperflash_count > 1) {
		dev_err(dev, "Flash node configuration not supported\n");
		return -EINVAL;
	}

	if (spi_flash_count)
		name = "stm32-ospi";
	else
		name = "stm32-hyperflash";

	omi = devm_kzalloc(dev, sizeof(*omi), GFP_KERNEL);
	if (!omi)
		return -ENOMEM;

	omi->jedec_flash = jedec_flash;
	omi->regs_base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(omi->regs_base))
		return PTR_ERR(omi->regs_base);

	omi->regs_phys_base = res->start;

	omi->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(omi->clk))
		return dev_err_probe(dev, PTR_ERR(omi->clk),
				     "Can't get clock\n");

	omi->clk_rate = clk_get_rate(omi->clk);
	if (!omi->clk_rate) {
		dev_err(dev, "Invalid clock rate\n");
		return -EINVAL;
	}

	omi->irq = platform_get_irq(pdev, 0);
	if (omi->irq < 0) {
		dev_err(dev, "Can't get irq %d\n", omi->irq);
		return omi->irq;
	}

	ret = devm_request_irq(dev, omi->irq, stm32_omi_irq, 0,
			       dev_name(dev), omi);
	if (ret) {
		dev_err(dev, "Failed to request irq\n");
		return ret;
	}

	omi->rstc = devm_reset_control_array_get(dev, false, true);
	if (IS_ERR(omi->rstc))
		return dev_err_probe(dev, PTR_ERR(omi->rstc),
				     "Can't get reset\n");

	omi->regmap = syscon_regmap_lookup_by_phandle(dev->of_node,
						      "st,syscfg-dlyb");
	if (IS_ERR(omi->regmap)) {
		dev_err(dev, "Can't find st,syscfg-dlyb property\n");
		return PTR_ERR(omi->regmap);
	} else {
		ret = of_property_read_u32_index(dev->of_node, "st,syscfg-dlyb",
						 1, &omi->dlyb_base);
		if (ret) {
			dev_err(dev, "Can't read delay block base address\n");
			return ret;
		}
	}

	omi->dma_chrx = dma_request_chan(dev, "rx");
	if (IS_ERR(omi->dma_chrx)) {
		ret = PTR_ERR(omi->dma_chrx);
		omi->dma_chrx = NULL;
		if (ret == -EPROBE_DEFER)
			goto err_dma;
	}

	omi->dma_chtx = dma_request_chan(dev, "tx");
	if (IS_ERR(omi->dma_chtx)) {
		ret = PTR_ERR(omi->dma_chtx);
		omi->dma_chtx = NULL;
		if (ret == -EPROBE_DEFER)
			goto err_dma;
	}

	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (node)
		rmem = of_reserved_mem_lookup(node);
	of_node_put(node);

	if (rmem) {
		omi->mm_size = rmem->size;
		omi->mm_base = devm_ioremap(dev, rmem->base, rmem->size);
		if (IS_ERR(omi->mm_base)) {
			dev_err(dev, "unable to map memory region: %pa+%pa\n",
				&rmem->base, &rmem->size);
			ret = PTR_ERR(omi->mm_base);
			goto err_dma;
		}

		if (omi->mm_size > STM32_OMI_MAX_MMAP_SZ) {
			dev_err(dev, "Memory map size outsize bounds\n");
			ret = -EINVAL;
			goto err_dma;
		}
	} else {
		dev_info(dev, "No memory-map region found\n");
	}

	init_completion(&omi->data_completion);
	init_completion(&omi->match_completion);

	vdev = platform_device_alloc(name, PLATFORM_DEVID_AUTO);
	if (!vdev) {
		ret = -ENOMEM;
		goto err_dma;
	}

	vdev->dev.parent = dev;

	omi->dev = dev;
	omi->vdev = vdev;
	platform_set_drvdata(pdev, omi);

	ret = platform_device_add(vdev);
	if (ret) {
		platform_device_put(vdev);
		goto err_dma;
	}

	return 0;

err_dma:
	if (omi->dma_chtx)
		dma_release_channel(omi->dma_chtx);
	if (omi->dma_chrx)
		dma_release_channel(omi->dma_chrx);

	return ret;
}

static int stm32_omi_remove(struct platform_device *pdev)
{
	struct stm32_omi *omi = platform_get_drvdata(pdev);

	platform_device_unregister(omi->vdev);

	if (omi->dma_chtx)
		dma_release_channel(omi->dma_chtx);
	if (omi->dma_chrx)
		dma_release_channel(omi->dma_chrx);

	return 0;
}

static int __maybe_unused stm32_omi_runtime_suspend(struct device *dev)
{
	struct stm32_omi *omi = dev_get_drvdata(dev);

	clk_disable_unprepare(omi->clk);

	return 0;
}

static int __maybe_unused stm32_omi_runtime_resume(struct device *dev)
{
	struct stm32_omi *omi = dev_get_drvdata(dev);

	return clk_prepare_enable(omi->clk);
}

static const struct dev_pm_ops stm32_omi_pm_ops = {
	SET_RUNTIME_PM_OPS(stm32_omi_runtime_suspend,
			   stm32_omi_runtime_resume, NULL)
};

static const struct of_device_id stm32_omi_of_match[] = {
	{ .compatible = "st,stm32mp25-omi", },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_omi_of_match);

static struct platform_driver stm32_omi_driver = {
	.probe	= stm32_omi_probe,
	.remove	= stm32_omi_remove,
	.driver = {
		.name =	"stm32-omi",
		.of_match_table = stm32_omi_of_match,
		.pm = &stm32_omi_pm_ops,
	},
};
module_platform_driver(stm32_omi_driver);

MODULE_DESCRIPTION("STMicroelectronics STM32 OSPI Memory Interface driver");
MODULE_LICENSE("GPL");
