// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2025 - All Rights Reserved
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/spi/spi-mem.h>
#include <linux/types.h>

#define OSPI_CR			0x00
#define CR_EN			BIT(0)
#define CR_ABORT		BIT(1)
#define CR_DMAEN		BIT(2)
#define CR_FTHRES_SHIFT		8
#define CR_SMIE			BIT(19)
#define CR_APMS			BIT(22)
#define CR_CSSEL		BIT(24)
#define CR_FMODE_MASK		GENMASK(29, 28)
#define CR_FMODE_INDW		(0U)
#define CR_FMODE_INDR		(1U)
#define CR_FMODE_APM		(2U)
#define CR_FMODE_MM		(3U)

#define OSPI_DCR1		0x08
#define DCR1_DLYBYP		BIT(3)
#define DCR1_DEVSIZE_MASK	GENMASK(20, 16)
#define DCR1_MTYP_MASK		GENMASK(26, 24)
#define DCR1_MTYP_MX_MODE	1
#define DCR1_MTYP_HP_MEMMODE	4

#define OSPI_DCR2		0x0c
#define DCR2_PRESC_MASK		GENMASK(7, 0)

#define OSPI_SR			0x20
#define SR_TEF			BIT(0)
#define SR_TCF			BIT(1)
#define SR_FTF			BIT(2)
#define SR_SMF			BIT(3)
#define SR_BUSY			BIT(5)

#define OSPI_FCR		0x24
#define FCR_CTEF		BIT(0)
#define FCR_CTCF		BIT(1)
#define FCR_CSMF		BIT(3)

#define OSPI_DLR		0x40
#define OSPI_AR			0x48
#define OSPI_DR			0x50
#define OSPI_PSMKR		0x80
#define OSPI_PSMAR		0x88

#define OSPI_CCR		0x100
#define CCR_IMODE_MASK		GENMASK(2, 0)
#define CCR_IDTR		BIT(3)
#define CCR_ISIZE_MASK		GENMASK(5, 4)
#define CCR_ADMODE_MASK		GENMASK(10, 8)
#define CCR_ADMODE_8LINES	4
#define CCR_ADDTR		BIT(11)
#define CCR_ADSIZE_MASK		GENMASK(13, 12)
#define CCR_ADSIZE_32BITS	3
#define CCR_DMODE_MASK		GENMASK(26, 24)
#define CCR_DMODE_8LINES	4
#define CCR_DQSE		BIT(29)
#define CCR_DDTR		BIT(27)
#define CCR_BUSWIDTH_0		0x0
#define CCR_BUSWIDTH_1		0x1
#define CCR_BUSWIDTH_2		0x2
#define CCR_BUSWIDTH_4		0x3
#define CCR_BUSWIDTH_8		0x4

#define OSPI_TCR		0x108
#define TCR_DCYC_MASK		GENMASK(4, 0)
#define TCR_DHQC		BIT(28)
#define TCR_SSHIFT		BIT(30)

#define OSPI_IR			0x110

#define SYSCFG_DLYBOS_CR		0
#define DLYBOS_CR_EN			BIT(0)
#define DLYBOS_CR_RXTAPSEL_SHIFT	1
#define DLYBOS_CR_RXTAPSEL_MASK		GENMASK(6, 1)
#define DLYBOS_CR_TXTAPSEL_SHIFT	7
#define DLYBOS_CR_TXTAPSEL_MASK		GENMASK(12, 7)
#define DLYBOS_TAPSEL_NB		33
#define DLYBOS_BYP_EN			BIT(16)
#define DLYBOS_BYP_CMD_MASK		GENMASK(21, 17)

#define SYSCFG_DLYBOS_SR	4
#define DLYBOS_SR_LOCK		BIT(0)
#define DLYBOS_SR_RXTAPSEL_ACK	BIT(1)
#define DLYBOS_SR_TXTAPSEL_ACK	BIT(2)

#define STM32_OSPI_MAX_MMAP_SZ	SZ_256M
#define STM32_OSPI_MAX_NORCHIP	2

#define STM32_FIFO_TIMEOUT_US		30000
#define STM32_ABT_TIMEOUT_US		100000
#define STM32_COMP_TIMEOUT_MS		5000
#define STM32_BUSY_TIMEOUT_US		100000
#define STM32_WAIT_CMD_TIMEOUT_US	5000
#define STM32_DLYB_FREQ_THRESHOLD	50000000
#define STM32_DLYBOS_TIMEOUT_MS		1000
#define STM32_DLYBOS_DELAY_NB		24

#define STM32_AUTOSUSPEND_DELAY -1

#define MACRONIX_ID		0xc2

struct stm32_ospi_flash {
	struct spi_device *spi;
	u32 presc;
	u32 dlyb_cr_str;
	u32 dlyb_cr_dtr;
	u32 dcr_reg;
	u64 str_idcode;
	u64 dtr_idcode;
	bool is_spi_nor;
	bool octal_dtr;
	bool is_str_calibration;
	bool dtr_calibration_done_once;
	bool sample_later;
	bool str_sample_later;
	bool dtr_mode;
};

struct stm32_ospi {
	struct device *dev;
	struct spi_controller *ctrl;
	struct clk *clk;
	struct reset_control *rstc;
	struct regmap *regmap;
	struct stm32_ospi_flash flash[STM32_OSPI_MAX_NORCHIP];

	struct completion match_completion;

	struct dma_chan *dma_chtx;
	struct dma_chan *dma_chrx;
	struct completion dma_completion;

	void __iomem *regs_base;
	void __iomem *mm_base;
	phys_addr_t regs_phys_base;
	resource_size_t mm_size;
	u64 id_buf;
	u32 clk_rate;
	u32 fmode;
	u32 cr_reg;
	int last_cs;
	u32 dlyb_base;
	int irq;
	unsigned long status_timeout;
	bool calibration;

	int (*check_transfer)(struct stm32_ospi *ospi);
	/*
	 * To protect device configuration, could be different between
	 * 2 flash access
	 */
	struct mutex lock;
};

struct stm32_tap_window {
	u8 end;
	u8 length;
};

static void stm32_ospi_read_fifo(void *val, void __iomem *addr, u8 len)
{
	switch (len) {
	case sizeof(u32):
		*((u32 *)val) = readl_relaxed(addr);
		break;
	case sizeof(u16):
		*((u16 *)val) = readw_relaxed(addr);
		break;
	case sizeof(u8):
		*((u8 *)val) = readb_relaxed(addr);
	}
}

static void stm32_ospi_write_fifo(void *val, void __iomem *addr, u8 len)
{
	switch (len) {
	case sizeof(u32):
		writel_relaxed(*((u32 *)val), addr);
		break;
	case sizeof(u16):
		writew_relaxed(*((u16 *)val), addr);
		break;
	case sizeof(u8):
		writeb_relaxed(*((u8 *)val), addr);
	}
}

static int stm32_ospi_abort(struct stm32_ospi *ospi)
{
	void __iomem *regs_base = ospi->regs_base;
	u32 cr;
	int timeout;

	cr = readl_relaxed(regs_base + OSPI_CR) | CR_ABORT;
	writel_relaxed(cr, regs_base + OSPI_CR);

	/* wait clear of abort bit by hw */
	timeout = readl_relaxed_poll_timeout_atomic(regs_base + OSPI_CR,
						    cr, !(cr & CR_ABORT), 1,
						    STM32_ABT_TIMEOUT_US);

	if (timeout && !ospi->calibration)
		dev_err(ospi->dev, "%s abort timeout:%d\n", __func__, timeout);

	return timeout;
}

static int stm32_ospi_poll(struct stm32_ospi *ospi, void *buf, u32 len, bool read)
{
	void __iomem *regs_base = ospi->regs_base;
	void (*fifo)(void *val, void __iomem *addr, u8 len);
	u32 sr;
	int ret;
	u8 step;

	if (read)
		fifo = stm32_ospi_read_fifo;
	else
		fifo = stm32_ospi_write_fifo;

	while (len) {
		ret = readl_relaxed_poll_timeout_atomic(regs_base + OSPI_SR,
							sr, sr & SR_FTF, 1,
							STM32_FIFO_TIMEOUT_US);
		if (ret) {
			if (!ospi->calibration)
				dev_err(ospi->dev,
					"fifo timeout (len:%d stat:%#x)\n",
					len, sr);
			return ret;
		}

		if (len >= sizeof(u32))
			step = sizeof(u32);
		else if (len >= sizeof(u16))
			step = sizeof(u16);
		else
			step = sizeof(u8);

		fifo(buf, regs_base + OSPI_DR, step);
		len -= step;
		buf += step;
	}

	return 0;
}

static int stm32_ospi_wait_nobusy(struct stm32_ospi *ospi)
{
	u32 sr;

	return readl_relaxed_poll_timeout_atomic(ospi->regs_base + OSPI_SR,
						 sr, !(sr & SR_BUSY), 1,
						 STM32_BUSY_TIMEOUT_US);
}

static int stm32_ospi_wait_cmd(struct stm32_ospi *ospi)
{
	void __iomem *regs_base = ospi->regs_base;
	u32 sr;
	int err = 0;

	if (ospi->fmode == CR_FMODE_APM)
		goto out;

	err = readl_relaxed_poll_timeout_atomic(ospi->regs_base + OSPI_SR, sr,
						(sr & (SR_TEF | SR_TCF)), 1,
						STM32_WAIT_CMD_TIMEOUT_US);

	if (sr & SR_TCF)
		/* avoid false timeout */
		err = 0;
	if (sr & SR_TEF)
		err = -EIO;

out:
	/* clear flags */
	writel_relaxed(FCR_CTCF | FCR_CTEF, regs_base + OSPI_FCR);

	if (!err)
		err = stm32_ospi_wait_nobusy(ospi);

	return err;
}

static void stm32_ospi_dma_callback(void *arg)
{
	struct completion *dma_completion = arg;

	complete(dma_completion);
}

static irqreturn_t stm32_ospi_irq(int irq, void *dev_id)
{
	struct stm32_ospi *ospi = (struct stm32_ospi *)dev_id;
	void __iomem *regs_base = ospi->regs_base;
	u32 cr, sr;

	cr = readl_relaxed(regs_base + OSPI_CR);
	sr = readl_relaxed(regs_base + OSPI_SR);

	if (sr & SR_SMF) {
		/* disable irq */
		cr &= ~CR_SMIE;
		writel_relaxed(cr, regs_base + OSPI_CR);
		complete(&ospi->match_completion);
	}

	return IRQ_HANDLED;
}

static int stm32_ospi_dma_setup(struct stm32_ospi *ospi,
				struct dma_slave_config *dma_cfg)
{
	struct dma_slave_caps caps;
	int ret = 0;

	if (dma_cfg && ospi->dma_chrx) {
		ret = dma_get_slave_caps(ospi->dma_chrx, &caps);
		if (ret)
			return ret;

		dma_cfg->src_maxburst = caps.max_burst / dma_cfg->src_addr_width;

		if (dmaengine_slave_config(ospi->dma_chrx, dma_cfg)) {
			dev_err(ospi->dev, "dma rx config failed\n");
			dma_release_channel(ospi->dma_chrx);
			ospi->dma_chrx = NULL;
		}
	}

	if (dma_cfg && ospi->dma_chtx) {
		ret = dma_get_slave_caps(ospi->dma_chtx, &caps);
		if (ret)
			return ret;

		dma_cfg->dst_maxburst = caps.max_burst / dma_cfg->dst_addr_width;

		if (dmaengine_slave_config(ospi->dma_chtx, dma_cfg)) {
			dev_err(ospi->dev, "dma tx config failed\n");
			dma_release_channel(ospi->dma_chtx);
			ospi->dma_chtx = NULL;
		}
	}

	init_completion(&ospi->dma_completion);

	return ret;
}

static int stm32_ospi_dlyb_set_tap(struct stm32_ospi *ospi, u8 tap, bool rx_tap)
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

	regmap_update_bits(ospi->regmap,
			   ospi->dlyb_base + SYSCFG_DLYBOS_CR,
			   mask, mask & (tap << shift));

	ret = regmap_read_poll_timeout(ospi->regmap,
				       ospi->dlyb_base + SYSCFG_DLYBOS_SR,
				       sr, sr & ack, 1,
				       1000 * STM32_DLYBOS_TIMEOUT_MS);
	if (ret)
		dev_err(ospi->dev, "%s delay Block phase configuration timeout\n",
			rx_tap ? "RX" : "TX");

	return ret;
}

static int stm32_ospi_dlyb_find_tap(struct stm32_ospi *ospi, bool rx_only, u8 *window_len)
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
		ret = stm32_ospi_dlyb_set_tap(ospi, tx_tap, false);
		if (ret)
			return ret;

		rx_len = 0;
		rx_window_len = 0;
		rx_window_end = 0;

		for (rx_tap = 0; rx_tap < DLYBOS_TAPSEL_NB; rx_tap++) {
			ret = stm32_ospi_dlyb_set_tap(ospi, rx_tap, true);
			if (ret)
				return ret;

			ospi->calibration = true;
			ret = ospi->check_transfer(ospi);
			ospi->calibration = false;
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
		dev_dbg(ospi->dev, "rx_tap_w[%02d].end = %d rx_tap_w[%02d].length = %d\n",
			tx_tap, rx_tap_w[tx_tap].end, tx_tap, rx_tap_w[tx_tap].length);
	}

	if (ret == -EOPNOTSUPP) {
		dev_err(ospi->dev, "Calibration can not be done on this device\n");
		return ret;
	}

	if (rx_only) {
		if (!rx_window_len) {
			dev_err(ospi->dev, "Can't find RX phase settings\n");
			return -EIO;
		}

		rx_tap = rx_window_end - rx_window_len / 2;
		*window_len = rx_window_len;
		dev_dbg(ospi->dev, "RX_TAP_SEL set to %d\n", rx_tap);

		return stm32_ospi_dlyb_set_tap(ospi, rx_tap, true);
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

	dev_dbg(ospi->dev, "RX_TAP_SEL set to %d\n", rx_tap);
	ret = stm32_ospi_dlyb_set_tap(ospi, rx_tap, true);
	if (ret)
		return ret;

	dev_dbg(ospi->dev, "TX_TAP_SEL set to %d\n", best_tx_tap);

	return stm32_ospi_dlyb_set_tap(ospi, best_tx_tap, false);
}

/* ½ memory clock period in pico second */
static const u16 dlybos_delay_ps[STM32_DLYBOS_DELAY_NB] = {
2816, 4672, 6272, 7872, 9472, 11104, 12704, 14304, 15904, 17536, 19136, 20736,
22336, 23968, 25568, 27168, 28768, 30400, 32000, 33600, 35232, 36832, 38432, 40032
};

static u32 stm32_ospi_find_byp_cmd(u16 period_ps)
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

static void stm32_ospi_dlyb_get_cr(struct stm32_ospi *ospi, u32 *dlyb_cr)
{
	regmap_read(ospi->regmap, ospi->dlyb_base + SYSCFG_DLYBOS_CR, dlyb_cr);
}

static void stm32_ospi_dlyb_stop(struct stm32_ospi *ospi)
{
	/* disable delay block */
	regmap_write(ospi->regmap, ospi->dlyb_base + SYSCFG_DLYBOS_CR, 0x0);
}

static int stm32_ospi_dlyb_init(struct stm32_ospi *ospi, bool bypass_mode,
				u16 period_ps)
{
	u32 sr, val, mask;
	int ret;

	if (bypass_mode) {
		val = DLYBOS_BYP_EN;
		val |= stm32_ospi_find_byp_cmd(period_ps);
		mask = DLYBOS_BYP_CMD_MASK | DLYBOS_BYP_EN;
	} else {
		val = DLYBOS_CR_EN;
		mask = DLYBOS_CR_EN;
	}

	/* enable Delay Block */
	regmap_update_bits(ospi->regmap, ospi->dlyb_base + SYSCFG_DLYBOS_CR,
			   mask, val);

	if (bypass_mode)
		return 0;

	/* in lock mode, wait for lock status bit */
	ret = regmap_read_poll_timeout(ospi->regmap,
				       ospi->dlyb_base + SYSCFG_DLYBOS_SR,
				       sr, sr & DLYBOS_SR_LOCK, 1,
				       1000 * STM32_DLYBOS_TIMEOUT_MS);
	if (ret) {
		dev_err(ospi->dev, "Delay Block lock timeout\n");
		stm32_ospi_dlyb_stop(ospi);
	}

	return ret;
}

static int stm32_ospi_dlyb_set_cr(struct stm32_ospi *ospi, u32 dlyb_cr)
{
	bool bypass_mode = false;
	int ret;
	u16 period_ps;
	u8 rx_tap, tx_tap;

	period_ps = FIELD_GET(DLYBOS_BYP_CMD_MASK, dlyb_cr);
	if ((dlyb_cr & DLYBOS_BYP_EN) == DLYBOS_BYP_EN)
		bypass_mode = true;

	ret = stm32_ospi_dlyb_init(ospi, bypass_mode, period_ps);
	if (ret)
		return ret;

	/* restore Rx and TX tap */
	rx_tap = FIELD_GET(DLYBOS_CR_RXTAPSEL_MASK, dlyb_cr);
	ret = stm32_ospi_dlyb_set_tap(ospi, rx_tap, true);
	if (ret)
		return ret;

	tx_tap = FIELD_GET(DLYBOS_CR_TXTAPSEL_MASK, dlyb_cr);
	return stm32_ospi_dlyb_set_tap(ospi, tx_tap, false);
}

static int stm32_ospi_tx_mm(struct stm32_ospi *ospi,
			    const struct spi_mem_op *op)
{
	memcpy_fromio(op->data.buf.in, ospi->mm_base + op->addr.val,
		      op->data.nbytes);
	return 0;
}

static int stm32_ospi_tx_dma(struct stm32_ospi *ospi,
			     const struct spi_mem_op *op)
{
	struct dma_async_tx_descriptor *desc;
	struct stm32_ospi_flash *flash = &ospi->flash[ospi->last_cs];
	void __iomem *regs_base = ospi->regs_base;
	enum dma_transfer_direction dma_dir;
	struct dma_chan *dma_ch;
	struct sg_table sgt;
	dma_cookie_t cookie;
	u32 cr, t_out;
	int err, ret;
	u8 dummy = 0xff;

	if (op->data.dir == SPI_MEM_DATA_IN) {
		dma_dir = DMA_DEV_TO_MEM;
		dma_ch = ospi->dma_chrx;
	} else {
		dma_dir = DMA_MEM_TO_DEV;
		dma_ch = ospi->dma_chtx;
	}

	if (flash->octal_dtr && op->addr.val % 2) {
		/* Read/write dummy byte */
		ret = stm32_ospi_poll(ospi, &dummy, 1, op->data.dir == SPI_MEM_DATA_IN);
		if (ret)
			return ret;
	}

	/*
	 * Spi_map_buf return -EINVAL if the buffer is not DMA-able
	 * (DMA-able: in vmalloc | kmap | virt_addr_valid)
	 */
	err = spi_controller_dma_map_mem_op_data(ospi->ctrl, op, &sgt);
	if (err)
		return err;

	desc = dmaengine_prep_slave_sg(dma_ch, sgt.sgl, sgt.nents,
				       dma_dir, DMA_PREP_INTERRUPT);
	if (!desc) {
		err = -ENOMEM;
		goto out_unmap;
	}

	cr = readl_relaxed(regs_base + OSPI_CR);

	reinit_completion(&ospi->dma_completion);
	desc->callback = stm32_ospi_dma_callback;
	desc->callback_param = &ospi->dma_completion;
	cookie = dmaengine_submit(desc);
	err = dma_submit_error(cookie);
	if (err)
		goto out;

	dma_async_issue_pending(dma_ch);

	writel_relaxed(cr | CR_DMAEN, regs_base + OSPI_CR);

	t_out = sgt.nents * STM32_COMP_TIMEOUT_MS;
	if (!wait_for_completion_timeout(&ospi->dma_completion,
					 msecs_to_jiffies(t_out)))
		err = -ETIMEDOUT;

	if (err)
		dmaengine_terminate_all(dma_ch);

out:
	writel_relaxed(cr & ~CR_DMAEN, regs_base + OSPI_CR);
out_unmap:
	spi_controller_dma_unmap_mem_op_data(ospi->ctrl, op, &sgt);

	if (flash->octal_dtr && !err && (op->addr.val + op->data.nbytes) % 2) {
		/* Read/write dummy byte */
		ret = stm32_ospi_poll(ospi, &dummy, 1,
				      op->data.dir == SPI_MEM_DATA_IN);
		if (ret)
			return ret;
	}

	return err;
}

static int stm32_ospi_xfer(struct stm32_ospi *ospi, const struct spi_mem_op *op)
{
	struct stm32_ospi_flash *flash = &ospi->flash[ospi->last_cs];
	u8 *buf;
	int ret;
	u8 dummy = 0xff;

	if (!op->data.nbytes)
		return 0;

	if (ospi->fmode == CR_FMODE_MM)
		return stm32_ospi_tx_mm(ospi, op);
	else if (((op->data.dir == SPI_MEM_DATA_IN && ospi->dma_chrx) ||
		 (op->data.dir == SPI_MEM_DATA_OUT && ospi->dma_chtx)) &&
		  op->data.nbytes > 8)
		if (!stm32_ospi_tx_dma(ospi, op))
			return 0;

	if (op->data.dir == SPI_MEM_DATA_IN)
		buf = op->data.buf.in;
	else
		buf = (void *)op->data.buf.out;

	if (flash->octal_dtr && op->addr.val % 2) {
		/* Read/write dummy byte */
		ret = stm32_ospi_poll(ospi, &dummy, 1,
				      op->data.dir == SPI_MEM_DATA_IN);
		if (ret)
			return ret;
	}

	ret = stm32_ospi_poll(ospi, buf, op->data.nbytes,
			      op->data.dir == SPI_MEM_DATA_IN);
	if (ret)
		return ret;

	if (flash->octal_dtr && (op->addr.val + op->data.nbytes) % 2) {
		/* Read/write dummy byte */
		ret = stm32_ospi_poll(ospi, &dummy, 1,
				      op->data.dir == SPI_MEM_DATA_IN);
		if (ret)
			return ret;
	}

	return ret;
}

static int stm32_ospi_wait_poll_status(struct stm32_ospi *ospi,
				       const struct spi_mem_op *op)
{
	void __iomem *regs_base = ospi->regs_base;
	u32 cr;

	reinit_completion(&ospi->match_completion);
	cr = readl_relaxed(regs_base + OSPI_CR);
	writel_relaxed(cr | CR_SMIE, regs_base + OSPI_CR);

	if (!wait_for_completion_timeout(&ospi->match_completion,
					 msecs_to_jiffies(ospi->status_timeout))) {
		u32 sr = readl_relaxed(regs_base + OSPI_SR);

		/* Avoid false timeout */
		if (!(sr & SR_SMF))
			return -ETIMEDOUT;
	}

	writel_relaxed(FCR_CSMF, regs_base + OSPI_FCR);

	return 0;
}

static int stm32_ospi_get_mode(u8 buswidth)
{
	switch (buswidth) {
	case 8:
		return CCR_BUSWIDTH_8;
	case 4:
		return CCR_BUSWIDTH_4;
	default:
		return buswidth;
	}
}

static void stm32_ospi_set_prescaler(struct stm32_ospi *ospi, u32 presc)
{
	void __iomem *regs_base = ospi->regs_base;
	u32 dcr2;

	/* Set prescaler */
	dcr2 = readl_relaxed(regs_base + OSPI_DCR2);
	dcr2 &= ~DCR2_PRESC_MASK;
	dcr2 |= FIELD_PREP(DCR2_PRESC_MASK, presc);
	writel_relaxed(dcr2, regs_base + OSPI_DCR2);
}

static int stm32_ospi_send(struct spi_device *spi, const struct spi_mem_op *op)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(spi->controller);
	struct stm32_ospi_flash *flash;
	void __iomem *regs_base = ospi->regs_base;
	u32 ccr, cr, tcr = 0;
	u64 addr = op->addr.val;
	unsigned int nbytes = op->data.nbytes;
	int timeout, err = 0, err_poll_status = 0;
	u8 cs = spi->chip_select[ffs(spi->cs_index_mask) - 1];

	dev_dbg(ospi->dev, "cmd:%#x dtr:%d mode:%d.%d.%d.%d addr:%#llx len:%#x\n",
		op->cmd.opcode, op->cmd.dtr, op->cmd.buswidth,
		op->addr.buswidth, op->dummy.buswidth, op->data.buswidth,
		op->addr.val, op->data.nbytes);

	cr = readl_relaxed(ospi->regs_base + OSPI_CR);
	cr &= ~CR_CSSEL;
	cr |= FIELD_PREP(CR_CSSEL, cs);
	cr &= ~CR_FMODE_MASK;
	cr |= FIELD_PREP(CR_FMODE_MASK, ospi->fmode);
	writel_relaxed(cr, regs_base + OSPI_CR);

	flash = &ospi->flash[cs];
	/*
	 * When DTR mode and indirect read/write mode are set, there is a
	 * constraint on the address and the number of bytes read or write
	 * that should be even.
	 */
	if (flash->octal_dtr && ospi->fmode != CR_FMODE_MM && op->data.nbytes) {
		if (op->addr.val % 2) {
			addr--;
			nbytes++;
		}

		if ((op->addr.val + op->data.nbytes) % 2)
			nbytes++;
	}
	writel_relaxed(flash->dcr_reg, regs_base + OSPI_DCR1);

	if (op->data.nbytes && ospi->fmode != CR_FMODE_MM)
		writel_relaxed(nbytes - 1, regs_base + OSPI_DLR);

	ccr = FIELD_PREP(CCR_IMODE_MASK, stm32_ospi_get_mode(op->cmd.buswidth));
	ccr |= FIELD_PREP(CCR_ISIZE_MASK, op->cmd.nbytes - 1);

	if (op->cmd.dtr) {
		ccr |= CCR_IDTR;
		ccr |= CCR_DQSE;

		if (FIELD_GET(DCR2_PRESC_MASK,
			      readl_relaxed(regs_base + OSPI_DCR2)))
			tcr |= TCR_DHQC;
	}

	if (op->addr.dtr)
		ccr |= CCR_ADDTR;

	if (op->data.dtr)
		ccr |= CCR_DDTR;

	if (op->data.swap16)
		writel_relaxed(flash->dcr_reg |
			       FIELD_PREP(DCR1_MTYP_MASK, DCR1_MTYP_MX_MODE),
			       regs_base + OSPI_DCR1);

	if (op->addr.nbytes) {
		ccr |= FIELD_PREP(CCR_ADMODE_MASK,
				  stm32_ospi_get_mode(op->addr.buswidth));
		ccr |= FIELD_PREP(CCR_ADSIZE_MASK, op->addr.nbytes - 1);
	}

	if (flash->sample_later)
		tcr |= TCR_SSHIFT;

	if (op->dummy.buswidth && op->dummy.nbytes) {
		u8 dcyc = op->dummy.nbytes * 8 / op->dummy.buswidth;

		if (op->dummy.dtr)
			dcyc /= 2;

		tcr |= FIELD_PREP(TCR_DCYC_MASK, dcyc);
	}

	writel_relaxed(tcr, regs_base + OSPI_TCR);

	if (op->data.nbytes)
		ccr |= FIELD_PREP(CCR_DMODE_MASK,
				  stm32_ospi_get_mode(op->data.buswidth));

	writel_relaxed(ccr, regs_base + OSPI_CCR);

	/* set instruction, must be set after ccr register update */
	writel_relaxed(op->cmd.opcode, regs_base + OSPI_IR);

	if (op->addr.nbytes && ospi->fmode != CR_FMODE_MM)
		writel_relaxed(addr, regs_base + OSPI_AR);

	if (ospi->fmode == CR_FMODE_APM)
		err_poll_status = stm32_ospi_wait_poll_status(ospi, op);

	err = stm32_ospi_xfer(ospi, op);

	/*
	 * Abort in:
	 * -error case
	 * -read memory map: prefetching must be stopped if we read the last
	 *  byte of device (device size - fifo size). like device size is not
	 *  knows, the prefetching is always stop.
	 */
	if (err || err_poll_status || ospi->fmode == CR_FMODE_MM)
		goto abort;

	/* Wait end of tx in indirect mode */
	err = stm32_ospi_wait_cmd(ospi);
	if (err)
		goto abort;

	return 0;

abort:
	timeout = stm32_ospi_abort(ospi);
	writel_relaxed(FCR_CTCF | FCR_CSMF, regs_base + OSPI_FCR);

	if (!ospi->calibration && (err || err_poll_status || timeout))
		dev_err(ospi->dev, "%s err:%d err_poll_status:%d abort timeout:%d\n",
			__func__, err, err_poll_status, timeout);

	return err;
}

static int stm32_ospi_readid(struct stm32_ospi *ospi)
{
	struct stm32_ospi_flash *flash = &ospi->flash[ospi->last_cs];
	struct spi_device *spi = flash->spi;
	struct spi_mem_op readid_op;
	struct gpio_desc *cs_gpiod = spi->cs_gpiod[ffs(spi->cs_index_mask) - 1];
	u64 *rx_buf = &ospi->id_buf;
	int ret;
	u8 nb_dummy_bytes = flash->is_spi_nor ? 0 : 1;

	ospi->fmode = CR_FMODE_INDR;
	if (flash->is_str_calibration) {
		readid_op = (struct spi_mem_op)
			    SPI_MEM_OP(SPI_MEM_OP_CMD(0x9f, 1),
				       SPI_MEM_OP_NO_ADDR,
				       SPI_MEM_OP_DUMMY(nb_dummy_bytes, 1),
				       SPI_MEM_OP_DATA_IN(sizeof(*rx_buf),
							  (u8 *)rx_buf, 1));
	} else {
		if (flash->octal_dtr && flash->is_spi_nor) {
			u16 opcode;
			u8 nb_addr_bytes;
			u8 nb_dummy_bytes;

			if ((flash->dtr_idcode & 0xff) == MACRONIX_ID) {
				opcode = 0x9f60;
				nb_addr_bytes = 4;
				nb_dummy_bytes = 8;
			} else {
				/*
				 * All memory providers are not currently
				 * supported, feel free to add them
				 */
				return -EOPNOTSUPP;
			}

			readid_op = (struct spi_mem_op)
				    SPI_MEM_OP(SPI_MEM_OP_CMD(opcode, 8),
					       SPI_MEM_OP_ADDR(nb_addr_bytes, 0, 8),
					       SPI_MEM_OP_DUMMY(nb_dummy_bytes, 8),
					       SPI_MEM_OP_DATA_IN(sizeof(*rx_buf),
								  (u8 *)rx_buf, 8));
			readid_op.cmd.dtr = true;
			readid_op.addr.dtr = true;
			readid_op.dummy.dtr = true;
			readid_op.data.dtr = true;
			readid_op.cmd.nbytes = 2;
		} else {
			/*
			 * Only OCTAL DTR calibration on SPI NOR devices
			 * is currently supported
			 */
			return -EOPNOTSUPP;
		}
	}

	if (cs_gpiod)
		gpiod_set_value_cansleep(cs_gpiod, true);

	ret = stm32_ospi_send(spi, &readid_op);

	if (cs_gpiod)
		gpiod_set_value_cansleep(cs_gpiod, false);

	if (ret)
		return ret;

	dev_dbg(ospi->dev, "Flash ID 0x%08llx\n", *rx_buf);

	if (flash->is_str_calibration) {
	/*
	 * On stm32_ospi_readid() first execution, save the golden
	 * read id
	 */
	if (!flash->str_idcode)
		flash->str_idcode = *rx_buf;

			if (flash->is_spi_nor) {
				/* Build DTR id code */
				if ((*rx_buf & 0xff) == MACRONIX_ID) {
					/*
					 * Retrieve odd array and re-sort id
					 * because of read id format will be
					 * A-A-B-B-C-C after enter into octal
					 * dtr mode for Macronix flashes.
					 */
					flash->dtr_idcode = *rx_buf & 0xff;
					flash->dtr_idcode |= (*rx_buf & 0xff) << 8;
					flash->dtr_idcode |= (*rx_buf & 0xff00) << 8;
					flash->dtr_idcode |= (*rx_buf & 0xff00) << 16;
					flash->dtr_idcode |= (*rx_buf & 0xff0000) << 16;
					flash->dtr_idcode |= (*rx_buf & 0xff0000) << 24;
					flash->dtr_idcode |= (*rx_buf & 0xff000000) << 24;
					flash->dtr_idcode |= (*rx_buf & 0xff000000) << 32;
				} else {
					flash->dtr_idcode = *rx_buf;
				}
			}
	if (*rx_buf == flash->str_idcode)
		return 0;
	} else if (*rx_buf == flash->dtr_idcode) {
		return 0;
	}

	return -EIO;
}

static int stm32_ospi_str_calibration(struct stm32_ospi *ospi)
{
	struct stm32_ospi_flash *flash = &ospi->flash[ospi->last_cs];
	struct spi_device *spi = flash->spi;
	u32 bus_freq;
	u8 window_len_tcr0 = 0, window_len_tcr1 = 0;
	int ret, ret_tcr0, ret_tcr1;

	/*
	 * Set memory device at low frequency (50MHz) and sent
	 * READID (0x9F) command, save the answer as golden answer
	 */
	flash->presc = DIV_ROUND_UP(ospi->clk_rate, STM32_DLYB_FREQ_THRESHOLD) - 1;
	stm32_ospi_set_prescaler(ospi, flash->presc);

	ret = stm32_ospi_readid(ospi);
	if (ret)
		return ret;

	/*
	 * Set memory device at expected frequency read from DT
	 * prescaler should be set before locking the DLL
	 */
	flash->presc = DIV_ROUND_UP(ospi->clk_rate, spi->max_speed_hz) - 1;
	stm32_ospi_set_prescaler(ospi, flash->presc);
	bus_freq = DIV_ROUND_UP(ospi->clk_rate, flash->presc + 1);

	/* Calibration needed above 50MHz */
	if (bus_freq <= STM32_DLYB_FREQ_THRESHOLD)
		return 0;

	flash->dcr_reg &= ~DCR1_DLYBYP;
	writel_relaxed(flash->dcr_reg, ospi->regs_base + OSPI_DCR1);

	ret = stm32_ospi_dlyb_init(ospi, false, 0);
	if (ret)
		return ret;

	/*
	 * Perform only Rx TAP selection
	 * When DTR support will be added, Rx/Tx TAP selection will have to
	 * be performed
	 */
	ret_tcr0 = stm32_ospi_dlyb_find_tap(ospi, true, &window_len_tcr0);
	if (!ret_tcr0)
		/*
		 * Save flash delay block configuration, will be restored
		 * each time this flash is addressed
		 */
		stm32_ospi_dlyb_get_cr(ospi, &flash->dlyb_cr_str);

	stm32_ospi_dlyb_stop(ospi);
	ret = stm32_ospi_dlyb_init(ospi, false, 0);
	if (ret)
		return ret;

	flash->sample_later = true;
	ret_tcr1 = stm32_ospi_dlyb_find_tap(ospi, true, &window_len_tcr1);
	if (ret_tcr0 && ret_tcr1) {
		dev_info(ospi->dev, "Calibration phase failed\n");
		return ret_tcr0;
	}

	if (window_len_tcr0 >= window_len_tcr1) {
		flash->sample_later = false;
		stm32_ospi_dlyb_stop(ospi);

		ret = stm32_ospi_dlyb_set_cr(ospi, flash->dlyb_cr_str);
		if (ret)
			return ret;
	} else {
		stm32_ospi_dlyb_get_cr(ospi, &flash->dlyb_cr_str);
	}

	flash->str_sample_later = flash->sample_later;

	return 0;
}

static int stm32_ospi_dtr_calibration(struct stm32_ospi *ospi)
{
	struct stm32_ospi_flash *flash = &ospi->flash[ospi->last_cs];
	void __iomem *regs_base = ospi->regs_base;
	u32 prescaler, bus_freq;
	u16 period_ps = 0;
	u8 window_len = 0;
	int ret;
	bool bypass_mode = false;

	flash->dcr_reg &= ~DCR1_DLYBYP;
	writel_relaxed(flash->dcr_reg, regs_base + OSPI_DCR1);

	prescaler = FIELD_GET(DCR2_PRESC_MASK,
			      readl_relaxed(regs_base + OSPI_DCR2));
	bus_freq = DIV_ROUND_UP(ospi->clk_rate, prescaler + 1);

	if (bus_freq <= STM32_DLYB_FREQ_THRESHOLD) {
		bypass_mode = true;
		period_ps = NSEC_PER_SEC / (bus_freq / 1000);
	}

	ret = stm32_ospi_dlyb_init(ospi, bypass_mode, period_ps);
	if (ret) {
		dev_err(ospi->dev, "Can't enable delay block\n");
		return 0;
	}

	if (bypass_mode || prescaler)
		/* Perform only RX TAP selection */
		ret = stm32_ospi_dlyb_find_tap(ospi, true, &window_len);
	else
		/* Perform RX/TX TAP selection */
		ret = stm32_ospi_dlyb_find_tap(ospi, false, &window_len);

	if (ret) {
		dev_err(ospi->dev, "Calibration failed\n");
		if (!bypass_mode)
			/* Stop delay block when configured in lock mode */
			stm32_ospi_dlyb_stop(ospi);
	} else {
		stm32_ospi_dlyb_get_cr(ospi, &flash->dlyb_cr_dtr);
		flash->dtr_calibration_done_once = true;
	}

	return ret;
}

static int stm32_ospi_prepare_to_send(struct spi_device *spi,
				      const struct spi_mem_op *op)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(spi->controller);
	struct stm32_ospi_flash *flash;
	int ret = 0;
	u8 cs = spi->chip_select[ffs(spi->cs_index_mask) - 1];
	bool set_dlyb = false;

	flash = &ospi->flash[cs];

	if (ospi->last_cs != cs) {
		ospi->last_cs = cs;

		stm32_ospi_dlyb_stop(ospi);
		stm32_ospi_set_prescaler(ospi, flash->presc);
		set_dlyb = true;
	};

	if (op->cmd.dtr && !flash->dtr_calibration_done_once) {
		stm32_ospi_dlyb_stop(ospi);
		flash->sample_later = false;
		flash->octal_dtr = (op->cmd.nbytes == 2);

		ret = stm32_ospi_dtr_calibration(ospi);
		if (ret)
			return ret;

		flash->dtr_mode = true;
	}

	/*
	 * if flash is switched from DTR to STR or from STR to DTR
	 * restore the correct calibration value
	 */
	if ((flash->dtr_mode && !op->cmd.dtr) ||
	    (!flash->dtr_mode && op->cmd.dtr)) {
		flash->dtr_mode = !flash->dtr_mode;
		stm32_ospi_dlyb_stop(ospi);
		set_dlyb = true;
	}

	/*
	 * restore DLYB CR register depending on simple or
	 * double transfer rate
	 */
	if (set_dlyb) {
		if (op->cmd.dtr) {
			ret = stm32_ospi_dlyb_set_cr(ospi, flash->dlyb_cr_dtr);
			flash->sample_later = false;
			flash->octal_dtr = (op->cmd.nbytes == 2);
		} else {
			ret = stm32_ospi_dlyb_set_cr(ospi, flash->dlyb_cr_str);
			flash->sample_later = flash->str_sample_later;
			flash->octal_dtr = false;
		}
	}

	return ret;
}

static int stm32_ospi_poll_status(struct spi_mem *mem,
				  const struct spi_mem_op *op,
				  u16 mask, u16 match,
				  unsigned long initial_delay_us,
				  unsigned long polling_rate_us,
				  unsigned long timeout_ms)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(mem->spi->controller);
	void __iomem *regs_base = ospi->regs_base;
	int ret;

	ret = pm_runtime_resume_and_get(ospi->dev);
	if (ret < 0)
		return ret;

	mutex_lock(&ospi->lock);
	writel_relaxed(mask, regs_base + OSPI_PSMKR);
	writel_relaxed(match, regs_base + OSPI_PSMAR);
	ospi->fmode = CR_FMODE_APM;
	ospi->status_timeout = timeout_ms;

	ret = stm32_ospi_prepare_to_send(mem->spi, op);
	if (!ret)
		ret = stm32_ospi_send(mem->spi, op);

	mutex_unlock(&ospi->lock);
	pm_runtime_put_autosuspend(ospi->dev);

	return ret;
}

static int stm32_ospi_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(mem->spi->controller);
	int ret;

	ret = pm_runtime_resume_and_get(ospi->dev);
	if (ret < 0)
		return ret;

	mutex_lock(&ospi->lock);
	if (op->data.dir == SPI_MEM_DATA_IN && op->data.nbytes)
		ospi->fmode = CR_FMODE_INDR;
	else
		ospi->fmode = CR_FMODE_INDW;

	ret = stm32_ospi_prepare_to_send(mem->spi, op);
	if (!ret)
		ret = stm32_ospi_send(mem->spi, op);

	mutex_unlock(&ospi->lock);
	pm_runtime_put_autosuspend(ospi->dev);

	return ret;
}

static int stm32_ospi_dirmap_create(struct spi_mem_dirmap_desc *desc)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(desc->mem->spi->controller);

	if (desc->info.op_tmpl.data.dir == SPI_MEM_DATA_OUT)
		return -EOPNOTSUPP;

	/* Should never happen, as mm_base == null is an error probe exit condition */
	if (!ospi->mm_base && desc->info.op_tmpl.data.dir == SPI_MEM_DATA_IN)
		return -EOPNOTSUPP;

	if (!ospi->mm_size)
		return -EOPNOTSUPP;

	return 0;
}

static ssize_t stm32_ospi_dirmap_read(struct spi_mem_dirmap_desc *desc,
				      u64 offs, size_t len, void *buf)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(desc->mem->spi->controller);
	struct spi_mem_op op;
	u32 addr_max;
	int ret;

	ret = pm_runtime_resume_and_get(ospi->dev);
	if (ret < 0)
		return ret;

	mutex_lock(&ospi->lock);
	/*
	 * Make a local copy of desc op_tmpl and complete dirmap rdesc
	 * spi_mem_op template with offs, len and *buf in  order to get
	 * all needed transfer information into struct spi_mem_op
	 */
	memcpy(&op, &desc->info.op_tmpl, sizeof(struct spi_mem_op));
	dev_dbg(ospi->dev, "%s len = 0x%zx offs = 0x%llx buf = 0x%p\n", __func__, len, offs, buf);

	op.data.nbytes = len;
	op.addr.val = desc->info.offset + offs;
	op.data.buf.in = buf;

	addr_max = op.addr.val + op.data.nbytes + 1;
	if (addr_max < ospi->mm_size && op.addr.buswidth)
		ospi->fmode = CR_FMODE_MM;
	else
		ospi->fmode = CR_FMODE_INDR;

	ret = stm32_ospi_prepare_to_send(desc->mem->spi, &op);
	if (!ret)
		ret = stm32_ospi_send(desc->mem->spi, &op);

	mutex_unlock(&ospi->lock);
	pm_runtime_put_autosuspend(ospi->dev);

	return ret ?: len;
}

static int stm32_ospi_transfer_one_message(struct spi_controller *ctrl,
					   struct spi_message *msg)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(ctrl);
	struct spi_transfer *transfer;
	struct spi_device *spi = msg->spi;
	struct spi_mem_op op;
	struct gpio_desc *cs_gpiod = spi->cs_gpiod[ffs(spi->cs_index_mask) - 1];
	int ret = 0;

	if (!cs_gpiod)
		return -EOPNOTSUPP;

	ret = pm_runtime_resume_and_get(ospi->dev);
	if (ret < 0)
		return ret;

	mutex_lock(&ospi->lock);

	gpiod_set_value_cansleep(cs_gpiod, true);

	list_for_each_entry(transfer, &msg->transfers, transfer_list) {
		u8 dummy_bytes = 0;

		memset(&op, 0, sizeof(op));

		dev_dbg(ospi->dev, "tx_buf:%p tx_nbits:%d rx_buf:%p rx_nbits:%d len:%d dummy_data:%d\n",
			transfer->tx_buf, transfer->tx_nbits,
			transfer->rx_buf, transfer->rx_nbits,
			transfer->len, transfer->dummy_data);

		/*
		 * OSPI hardware supports dummy bytes transfer.
		 * If current transfer is dummy byte, merge it with the next
		 * transfer in order to take into account OSPI block constraint
		 */
		if (transfer->dummy_data) {
			op.dummy.buswidth = transfer->tx_nbits;
			op.dummy.nbytes = transfer->len;
			dummy_bytes = transfer->len;

			/* If happens, means that message is not correctly built */
			if (list_is_last(&transfer->transfer_list, &msg->transfers)) {
				ret = -EINVAL;
				goto end_of_transfer;
			}

			transfer = list_next_entry(transfer, transfer_list);
		}

		op.data.nbytes = transfer->len;

		if (transfer->rx_buf) {
			ospi->fmode = CR_FMODE_INDR;
			op.data.buswidth = transfer->rx_nbits;
			op.data.dir = SPI_MEM_DATA_IN;
			op.data.buf.in = transfer->rx_buf;
		} else {
			ospi->fmode = CR_FMODE_INDW;
			op.data.buswidth = transfer->tx_nbits;
			op.data.dir = SPI_MEM_DATA_OUT;
			op.data.buf.out = transfer->tx_buf;
		}

		ret = stm32_ospi_prepare_to_send(spi, &op);
		if (ret)
			goto end_of_transfer;

		ret = stm32_ospi_send(spi, &op);
		if (ret)
			goto end_of_transfer;

		msg->actual_length += transfer->len + dummy_bytes;
	}

end_of_transfer:
	gpiod_set_value_cansleep(cs_gpiod, false);

	mutex_unlock(&ospi->lock);

	msg->status = ret;
	spi_finalize_current_message(ctrl);

	pm_runtime_put_autosuspend(ospi->dev);

	return ret;
}

static int stm32_ospi_setup(struct spi_device *spi)
{
	struct stm32_ospi_flash *flash;
	struct spi_controller *ctrl = spi->controller;
	struct stm32_ospi *ospi = spi_controller_get_devdata(ctrl);
	void __iomem *regs_base = ospi->regs_base;
	int ret;
	u8 cs = spi->chip_select[ffs(spi->cs_index_mask) - 1];

	if (ctrl->busy)
		return -EBUSY;

	if (!spi->max_speed_hz)
		return -EINVAL;

	ret = pm_runtime_resume_and_get(ospi->dev);
	if (ret < 0)
		return ret;

	flash = &ospi->flash[cs];
	flash->presc = DIV_ROUND_UP(ospi->clk_rate, spi->max_speed_hz) - 1;
	flash->spi = spi;

	mutex_lock(&ospi->lock);

	ospi->cr_reg = CR_APMS | 3 << CR_FTHRES_SHIFT | CR_EN;
	writel_relaxed(ospi->cr_reg, regs_base + OSPI_CR);

	/* Stop the DLL */
	stm32_ospi_dlyb_stop(ospi);

	/* Set dcr fsize to max address */
	flash->dcr_reg = DCR1_DEVSIZE_MASK | DCR1_DLYBYP;
	writel_relaxed(flash->dcr_reg, regs_base + OSPI_DCR1);

	stm32_ospi_set_prescaler(ospi, flash->presc);

	ospi->last_cs = cs;
	flash->is_str_calibration = true;

	ret = stm32_ospi_str_calibration(ospi);
	if (ret) {
		dev_info(ospi->dev, "Set flash frequency to a safe value (%d Hz)\n",
			 STM32_DLYB_FREQ_THRESHOLD);

		/* Stop the DLL */
		stm32_ospi_dlyb_stop(ospi);
		flash->sample_later = false;

		flash->presc = DIV_ROUND_UP(ospi->clk_rate,
					    STM32_DLYB_FREQ_THRESHOLD) - 1;
		stm32_ospi_set_prescaler(ospi, flash->presc);

		flash->dcr_reg |= DCR1_DLYBYP;
		writel_relaxed(flash->dcr_reg, regs_base + OSPI_DCR1);
	}

	flash->is_str_calibration = false;
	mutex_unlock(&ospi->lock);
	pm_runtime_put_autosuspend(ospi->dev);

	return 0;
}

static bool stm32_ospi_supports_mem_op(struct spi_mem *mem,
				       const struct spi_mem_op *op)
{
	bool all_true, all_false;

	/*
	 * Op->dummy.dtr is required for converting nbytes into ncycles.
	 * Also, don't check the dtr field of the op phase having zero nbytes.
	 */
	all_true = op->cmd.dtr &&
		   (!op->addr.nbytes || op->addr.dtr) &&
		   (!op->dummy.nbytes || op->dummy.dtr) &&
		   (!op->data.nbytes || op->data.dtr);

	all_false = !op->cmd.dtr && !op->addr.dtr && !op->dummy.dtr &&
		    !op->data.dtr;

	if (all_true) {
		/* Right now we only support 8-8-8 DTR mode. */
		if (op->cmd.nbytes && op->cmd.buswidth != 8)
			return false;
		if (op->addr.nbytes && op->addr.buswidth != 8)
			return false;
		if (op->data.nbytes && op->data.buswidth != 8)
			return false;
	} else if (!all_false) {
		/* Mixed DTR modes are not supported. */
		return false;
	}

	if (op->data.buswidth > 8 || op->addr.buswidth > 8 ||
	    op->dummy.buswidth > 8 || op->cmd.buswidth > 8)
		return false;

	if (op->cmd.nbytes > 4 || op->addr.nbytes > 4)
		return false;

	if ((!op->dummy.dtr && op->dummy.nbytes > 32) ||
	    (op->dummy.dtr && op->dummy.nbytes > 64))
		return false;

	return spi_mem_default_supports_op(mem, op);
}

/*
 * No special host constraint, so use default spi_mem_default_supports_op
 * to check supported mode.
 */
static const struct spi_controller_mem_ops stm32_ospi_mem_ops = {
	.exec_op	= stm32_ospi_exec_op,
	.dirmap_create	= stm32_ospi_dirmap_create,
	.dirmap_read	= stm32_ospi_dirmap_read,
	.poll_status	= stm32_ospi_poll_status,
	.supports_op = stm32_ospi_supports_mem_op,
};

static const struct spi_controller_mem_caps stm32_ospi_mem_caps = {
	.dtr = true,
	.swap16 = true,
};

static int stm32_ospi_get_resources(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stm32_ospi *ospi = platform_get_drvdata(pdev);
	struct resource *res, _res;
	int ret;

	ospi->regs_base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(ospi->regs_base))
		return PTR_ERR(ospi->regs_base);

	ospi->regs_phys_base = res->start;

	ospi->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(ospi->clk))
		return dev_err_probe(dev, PTR_ERR(ospi->clk),
				     "Can't get clock\n");

	ospi->clk_rate = clk_get_rate(ospi->clk);
	if (!ospi->clk_rate) {
		dev_err(dev, "Invalid clock rate\n");
		return -EINVAL;
	}

	ospi->irq = platform_get_irq(pdev, 0);
	if (ospi->irq < 0)
		return ospi->irq;

	ret = devm_request_irq(dev, ospi->irq, stm32_ospi_irq, 0,
			       dev_name(dev), ospi);
	if (ret) {
		dev_err(dev, "Failed to request irq\n");
		return ret;
	}

	ospi->rstc = devm_reset_control_array_get_exclusive_released(dev);
	if (IS_ERR(ospi->rstc))
		return dev_err_probe(dev, PTR_ERR(ospi->rstc),
				     "Can't get reset\n");

	ospi->regmap = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "st,syscfg-dlyb");
	if (IS_ERR(ospi->regmap)) {
		dev_err(dev, "Can't find st,syscfg-dlyb property\n");
		return PTR_ERR(ospi->regmap);
	}

	ret = of_property_read_u32_index(dev->of_node, "st,syscfg-dlyb",
					 1, &ospi->dlyb_base);
	if (ret) {
		dev_err(dev, "Can't read delay block base address\n");
		return ret;
	}

	ospi->dma_chrx = dma_request_chan(dev, "rx");
	if (IS_ERR(ospi->dma_chrx)) {
		ret = PTR_ERR(ospi->dma_chrx);
		ospi->dma_chrx = NULL;
		if (ret == -EPROBE_DEFER)
			goto err_dma;
	}

	ospi->dma_chtx = dma_request_chan(dev, "tx");
	if (IS_ERR(ospi->dma_chtx)) {
		ret = PTR_ERR(ospi->dma_chtx);
		ospi->dma_chtx = NULL;
		if (ret == -EPROBE_DEFER)
			goto err_dma;
	}

	res = &_res;
	ret = of_reserved_mem_region_to_resource(dev->of_node, 0, res);
	if (!ret) {
		ospi->mm_size = resource_size(res);
		ospi->mm_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(ospi->mm_base)) {
			dev_err(dev, "unable to map memory region: %pR\n", res);
			ret = PTR_ERR(ospi->mm_base);
			goto err_dma;
		}

		if (ospi->mm_size > STM32_OSPI_MAX_MMAP_SZ) {
			dev_err(dev, "Memory map size outsize bounds\n");
			ret = -EINVAL;
			goto err_dma;
		}
	} else {
		dev_info(dev, "No memory-map region found\n");
	}

	init_completion(&ospi->match_completion);

	return 0;

err_dma:
	dev_info(dev, "Can't get all resources (%d)\n", ret);

	if (ospi->dma_chtx)
		dma_release_channel(ospi->dma_chtx);
	if (ospi->dma_chrx)
		dma_release_channel(ospi->dma_chrx);

	return ret;
};

static int stm32_ospi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctrl;
	struct stm32_ospi *ospi;
	struct dma_slave_config dma_cfg;
	struct device_node *child;
	int ret;
	u8 spi_flash_count = 0;

	/*
	 * Flash subnodes sanity check:
	 *        1 or 2 spi-nand/spi-nor flashes		=> supported
	 *	  All other flash node configuration		=> not supported
	 */
	for_each_available_child_of_node(dev->of_node, child) {
		if (of_device_is_compatible(child, "jedec,spi-nor") ||
		    of_device_is_compatible(child, "spi-nand"))
			spi_flash_count++;
	}

	if (spi_flash_count == 0 || spi_flash_count > 2) {
		dev_err(dev, "Incorrect DT flash node\n");
		return -ENODEV;
	}

	ctrl = devm_spi_alloc_host(dev, sizeof(*ospi));
	if (!ctrl)
		return -ENOMEM;

	ospi = spi_controller_get_devdata(ctrl);
	ospi->ctrl = ctrl;
	ospi->check_transfer = stm32_ospi_readid;

	ospi->dev = &pdev->dev;
	platform_set_drvdata(pdev, ospi);

	ret = stm32_ospi_get_resources(pdev);
	if (ret)
		return ret;

	memset(&dma_cfg, 0, sizeof(dma_cfg));
	dma_cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_cfg.src_addr = ospi->regs_phys_base + OSPI_DR;
	dma_cfg.dst_addr = ospi->regs_phys_base + OSPI_DR;
	ret = stm32_ospi_dma_setup(ospi, &dma_cfg);
	if (ret)
		return ret;

	mutex_init(&ospi->lock);

	ctrl->mode_bits = SPI_RX_DUAL | SPI_RX_QUAD |
			  SPI_TX_DUAL | SPI_TX_QUAD |
			  SPI_TX_OCTAL | SPI_RX_OCTAL;
	ctrl->flags = SPI_CONTROLLER_HALF_DUPLEX;
	ctrl->setup = stm32_ospi_setup;
	ctrl->bus_num = -1;
	ctrl->mem_ops = &stm32_ospi_mem_ops;
	ctrl->mem_caps = &stm32_ospi_mem_caps;
	ctrl->use_gpio_descriptors = true;
	ctrl->transfer_one_message = stm32_ospi_transfer_one_message;
	ctrl->num_chipselect = STM32_OSPI_MAX_NORCHIP;
	ctrl->dev.of_node = dev->of_node;

	pm_runtime_enable(ospi->dev);
	pm_runtime_set_autosuspend_delay(ospi->dev, STM32_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(ospi->dev);

	ret = pm_runtime_resume_and_get(ospi->dev);
	if (ret < 0)
		goto err_pm_enable;

	ret = reset_control_acquire(ospi->rstc);
	if (ret) {
		dev_err_probe(dev, ret, "Can not acquire reset %d\n", ret);
		goto err_pm_resume;
	}

	reset_control_assert(ospi->rstc);
	udelay(2);
	reset_control_deassert(ospi->rstc);

	/* Find memory model on each child node (SPI NOR or SPI NAND) */
	for_each_available_child_of_node(ospi->dev->of_node, child) {
		u32 cs;

		ret = of_property_read_u32(child, "reg", &cs);
		if (ret) {
			dev_err(ospi->dev, "could not retrieve reg property: %d\n",
				ret);
			of_node_put(child);
			goto err_pm_resume;
		}

		if (cs >= STM32_OSPI_MAX_NORCHIP) {
			dev_err(ospi->dev, "invalid reg value: %d\n", cs);
			of_node_put(child);
			ret = -EINVAL;
			goto err_pm_resume;
		}

		if (of_device_is_compatible(child, "jedec,spi-nor")) {
			struct stm32_ospi_flash *flash = &ospi->flash[cs];

			flash->is_spi_nor = true;
		}
	}

	ret = spi_register_controller(ctrl);
	if (ret) {
		/* Disable ospi */
		writel_relaxed(0, ospi->regs_base + OSPI_CR);
		goto err_reset_control;
	}

	pm_runtime_put_autosuspend(ospi->dev);

	return 0;

err_reset_control:
	reset_control_release(ospi->rstc);
err_pm_resume:
	pm_runtime_put_sync_suspend(ospi->dev);

err_pm_enable:
	pm_runtime_force_suspend(ospi->dev);
	mutex_destroy(&ospi->lock);
	if (ospi->dma_chtx)
		dma_release_channel(ospi->dma_chtx);
	if (ospi->dma_chrx)
		dma_release_channel(ospi->dma_chrx);

	return ret;
}

static void stm32_ospi_remove(struct platform_device *pdev)
{
	struct stm32_ospi *ospi = platform_get_drvdata(pdev);

	pm_runtime_resume_and_get(ospi->dev);

	spi_unregister_controller(ospi->ctrl);
	/* Disable ospi */
	writel_relaxed(0, ospi->regs_base + OSPI_CR);
	stm32_ospi_dlyb_stop(ospi);
	mutex_destroy(&ospi->lock);

	if (ospi->dma_chtx)
		dma_release_channel(ospi->dma_chtx);
	if (ospi->dma_chrx)
		dma_release_channel(ospi->dma_chrx);

	reset_control_release(ospi->rstc);

	pm_runtime_put_sync_suspend(ospi->dev);
	pm_runtime_force_suspend(ospi->dev);
}

static int stm32_ospi_suspend(struct device *dev)
{
	struct stm32_ospi *ospi = dev_get_drvdata(dev);

	pinctrl_pm_select_sleep_state(dev);

	reset_control_release(ospi->rstc);

	return pm_runtime_force_suspend(ospi->dev);
}

static int stm32_ospi_resume(struct device *dev)
{
	struct stm32_ospi *ospi = dev_get_drvdata(dev);
	void __iomem *regs_base = ospi->regs_base;
	int ret;

	ret = pm_runtime_force_resume(ospi->dev);
	if (ret < 0)
		return ret;

	pinctrl_pm_select_default_state(dev);

	ret = pm_runtime_resume_and_get(ospi->dev);
	if (ret < 0)
		return ret;

	ret = reset_control_acquire(ospi->rstc);
	if (ret) {
		dev_err(dev, "Can not acquire reset\n");
		return ret;
	}

	writel_relaxed(ospi->cr_reg, regs_base + OSPI_CR);
	ospi->last_cs = -1;
	pm_runtime_put_autosuspend(ospi->dev);

	return 0;
}

static int stm32_ospi_runtime_suspend(struct device *dev)
{
	struct stm32_ospi *ospi = dev_get_drvdata(dev);

	clk_disable_unprepare(ospi->clk);

	return 0;
}

static int stm32_ospi_runtime_resume(struct device *dev)
{
	struct stm32_ospi *ospi = dev_get_drvdata(dev);

	return clk_prepare_enable(ospi->clk);
}

static const struct dev_pm_ops stm32_ospi_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(stm32_ospi_suspend, stm32_ospi_resume)
	RUNTIME_PM_OPS(stm32_ospi_runtime_suspend, stm32_ospi_runtime_resume, NULL)
};

static const struct of_device_id stm32_ospi_of_match[] = {
	{ .compatible = "st,stm32mp25-ospi" },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_ospi_of_match);

static struct platform_driver stm32_ospi_driver = {
	.probe	= stm32_ospi_probe,
	.remove	= stm32_ospi_remove,
	.driver	= {
		.name = "stm32-ospi",
		.pm = pm_ptr(&stm32_ospi_pm_ops),
		.of_match_table = stm32_ospi_of_match,
	},
};
module_platform_driver(stm32_ospi_driver);

MODULE_DESCRIPTION("STMicroelectronics STM32 OCTO SPI driver");
MODULE_LICENSE("GPL");
