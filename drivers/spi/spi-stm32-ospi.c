// SPDX-License-Identifier: GPL
/*
 * Copyright (C) STMicroelectronics 2023 - All Rights Reserved
 * Author(s): Patrice Chotard <patrice.chotard@foss.st.com> for STMicroelectronics.
 */

#include <memory/stm32-omi.h>

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
	bool is_octal_bus;
	bool dtr_mode;
};

struct stm32_ospi {
	struct device *dev;
	struct spi_controller *ctrl;
	struct stm32_omi *omi;
	struct stm32_ospi_flash flash[STM32_OMI_MAX_NORCHIP];

	u32 cr_reg;
	u64 id_buf;
	int last_cs;
	unsigned long status_timeout;

	/*
	 * To protect device configuration, could be different between
	 * 2 flash access
	 */
	struct mutex lock;
};

static int stm32_ospi_tx_mm(struct stm32_ospi *ospi,
			    const struct spi_mem_op *op)
{
	struct stm32_omi *omi = ospi->omi;

	memcpy_fromio(op->data.buf.in, omi->mm_base + op->addr.val,
		      op->data.nbytes);
	return 0;
}

static int stm32_ospi_tx_dma(struct stm32_ospi *ospi,
			     const struct spi_mem_op *op)
{
	struct dma_async_tx_descriptor *desc;
	struct stm32_omi *omi = ospi->omi;
	struct stm32_ospi_flash *flash = &ospi->flash[ospi->last_cs];
	void __iomem *regs_base = omi->regs_base;
	enum dma_transfer_direction dma_dir;
	struct dma_chan *dma_ch;
	struct sg_table sgt;
	dma_cookie_t cookie;
	u32 cr, t_out;
	u8 dummy = 0xff;
	int err, ret;

	if (op->data.dir == SPI_MEM_DATA_IN) {
		dma_dir = DMA_DEV_TO_MEM;
		dma_ch = omi->dma_chrx;
	} else {
		dma_dir = DMA_MEM_TO_DEV;
		dma_ch = omi->dma_chtx;
	}

	if (flash->octal_dtr && op->addr.val % 2) {
		/* Read/write dummy byte */
		ret = stm32_omi_tx_poll(omi, &dummy, 1,
					op->data.dir == SPI_MEM_DATA_IN);
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

	reinit_completion(&omi->dma_completion);
	desc->callback = stm32_omi_dma_callback;
	desc->callback_param = &omi->dma_completion;
	cookie = dmaengine_submit(desc);
	err = dma_submit_error(cookie);
	if (err)
		goto out;

	dma_async_issue_pending(dma_ch);

	writel_relaxed(cr | CR_DMAEN, regs_base + OSPI_CR);

	t_out = sgt.nents * STM32_COMP_TIMEOUT_MS;
	if (!wait_for_completion_timeout(&omi->dma_completion,
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
		ret = stm32_omi_tx_poll(omi, &dummy, 1,
					op->data.dir == SPI_MEM_DATA_IN);
		if (ret)
			return ret;
	}

	return err;
}

static int stm32_ospi_tx(struct stm32_ospi *ospi, const struct spi_mem_op *op)
{
	struct stm32_omi *omi = ospi->omi;
	struct stm32_ospi_flash *flash = &ospi->flash[ospi->last_cs];
	u8 *buf;
	u8 dummy = 0xff;
	int ret;

	if (!op->data.nbytes)
		return 0;

	if (omi->fmode == CR_FMODE_MM)
		return stm32_ospi_tx_mm(ospi, op);
	else if (((op->data.dir == SPI_MEM_DATA_IN && omi->dma_chrx) ||
		 (op->data.dir == SPI_MEM_DATA_OUT && omi->dma_chtx)) &&
		  op->data.nbytes > 8)
		if (!stm32_ospi_tx_dma(ospi, op))
			return 0;

	if (op->data.dir == SPI_MEM_DATA_IN)
		buf = op->data.buf.in;
	else
		buf = (u8 *)op->data.buf.out;

	if (flash->octal_dtr && op->addr.val % 2) {
		/* Read/write dummy byte */
		ret = stm32_omi_tx_poll(omi, &dummy, 1,
					op->data.dir == SPI_MEM_DATA_IN);
		if (ret)
			return ret;
	}

	ret = stm32_omi_tx_poll(omi, buf, op->data.nbytes,
				op->data.dir == SPI_MEM_DATA_IN);
	if (ret)
		return ret;

	if (flash->octal_dtr && (op->addr.val + op->data.nbytes) % 2) {
		/* Read/write dummy byte */
		ret = stm32_omi_tx_poll(omi, &dummy, 1,
					op->data.dir == SPI_MEM_DATA_IN);
		if (ret)
			return ret;
	}

	return ret;
}

static int stm32_ospi_wait_poll_status(struct stm32_ospi *ospi,
				       const struct spi_mem_op *op)
{
	struct stm32_omi *omi = ospi->omi;
	void __iomem *regs_base = omi->regs_base;
	u32 cr;

	reinit_completion(&omi->match_completion);
	cr = readl_relaxed(regs_base + OSPI_CR);
	writel_relaxed(cr | CR_SMIE, regs_base + OSPI_CR);

	if (!wait_for_completion_timeout(&omi->match_completion,
					 msecs_to_jiffies(ospi->status_timeout)))
		return -ETIMEDOUT;

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
	struct stm32_omi *omi = ospi->omi;
	void __iomem *regs_base = omi->regs_base;
	u32 dcr2;

	/* Set prescaler */
	dcr2 = readl_relaxed(regs_base + OSPI_DCR2);
	dcr2 &= ~DCR2_PRESC_MASK;
	dcr2 |= FIELD_PREP(DCR2_PRESC_MASK, presc);
	writel_relaxed(dcr2, regs_base + OSPI_DCR2);
}

static int stm32_ospi_send(struct spi_device *spi, const struct spi_mem_op *op)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(spi->master);
	struct stm32_ospi_flash *flash = &ospi->flash[spi->chip_select];
	struct stm32_omi *omi = ospi->omi;
	void __iomem *regs_base = omi->regs_base;
	u32 ccr, cr, tcr = 0;
	u64 addr = op->addr.val;
	unsigned int nbytes = op->data.nbytes;
	int timeout, err = 0, err_poll_status = 0;

	dev_dbg(ospi->dev, "cmd:%#x dtr:%d mode:%d.%d.%d.%d addr:%#llx len:%#x\n",
		op->cmd.opcode, op->cmd.dtr, op->cmd.buswidth,
		op->addr.buswidth, op->dummy.buswidth, op->data.buswidth,
		op->addr.val, op->data.nbytes);

	cr = readl_relaxed(regs_base + OSPI_CR);
	cr &= ~CR_CSSEL;
	cr |= FIELD_PREP(CR_CSSEL, spi->chip_select);
	cr &= ~CR_FMODE_MASK;
	cr |= FIELD_PREP(CR_FMODE_MASK, omi->fmode);
	writel_relaxed(cr, regs_base + OSPI_CR);

	/*
	 * When DTR mode and indirect read/write mode are set, there is a
	 * constraint on the address and the number of bytes read or write
	 * that should be even.
	 */
	if (flash->octal_dtr && omi->fmode != CR_FMODE_MM && op->data.nbytes) {
		if (op->addr.val % 2) {
			addr--;
			nbytes++;
		}

		if ((op->addr.val + op->data.nbytes) % 2)
			nbytes++;
	}

	writel_relaxed(flash->dcr_reg, regs_base + OSPI_DCR1);

	if (op->data.nbytes && omi->fmode != CR_FMODE_MM)
		writel_relaxed(nbytes - 1, regs_base + OSPI_DLR);

	ccr = FIELD_PREP(CCR_IMODE_MASK, stm32_ospi_get_mode(op->cmd.buswidth));
	ccr |= FIELD_PREP(CCR_ISIZE_MASK, op->cmd.nbytes - 1);

	if (op->cmd.dtr) {
		ccr |= CCR_IDTR;
		ccr |= CCR_DQSE;
	}

	if (op->addr.dtr)
		ccr |= CCR_ADDTR;

	if (op->data.dtr)
		ccr |= CCR_DDTR;

	if (op->data.dtr_swab16)
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

	writel_relaxed(op->cmd.opcode, regs_base + OSPI_IR);

	if (op->addr.nbytes && omi->fmode != CR_FMODE_MM)
		writel_relaxed(addr, regs_base + OSPI_AR);

	if (omi->fmode == CR_FMODE_APM)
		err_poll_status = stm32_ospi_wait_poll_status(ospi, op);

	err = stm32_ospi_tx(ospi, op);

	/*
	 * Abort in:
	 * -error case
	 * -read memory map: prefetching must be stopped if we read the last
	 *  byte of device (device size - fifo size). like device size is not
	 *  knows, the prefetching is always stop.
	 */
	if (err || err_poll_status || omi->fmode == CR_FMODE_MM)
		goto abort;

	/* Wait end of tx in indirect mode */
	err = stm32_omi_wait_cmd(omi);
	if (err)
		goto abort;

	return 0;

abort:
	timeout = stm32_omi_abort(omi);
	writel_relaxed(FCR_CTCF | FCR_CSMF, regs_base + OSPI_FCR);

	if (!omi->calibration && (err || err_poll_status || timeout))
		dev_err(ospi->dev, "%s err:%d err_poll_status:%d abort timeout:%d\n",
			__func__, err, err_poll_status, timeout);

	return err;
}

static int stm32_ospi_readid(struct stm32_omi *omi)
{
	struct stm32_ospi *ospi = platform_get_drvdata(omi->vdev);
	struct stm32_ospi_flash *flash = &ospi->flash[ospi->last_cs];
	struct spi_device *spi = flash->spi;
	u64 *rx_buf = &ospi->id_buf;
	struct spi_mem_op readid_op;
	int ret;

	omi->fmode = CR_FMODE_INDR;

	if (flash->is_str_calibration) {
		u8 nb_dummy_bytes = flash->is_spi_nor ? 0 : 1;

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

	if (spi->cs_gpiod)
		gpiod_set_value_cansleep(spi->cs_gpiod, true);

	ret = stm32_ospi_send(spi, &readid_op);

	if (spi->cs_gpiod)
		gpiod_set_value_cansleep(spi->cs_gpiod, false);

	if (ret)
		return ret;

	dev_dbg(ospi->dev, "Flash ID 0x%08llx\n", *rx_buf);

	if (flash->is_str_calibration) {
		/*
		 * On stm32_ospi_readid() first execution, save the golden
		 * read id
		 */
		if (!flash->str_idcode) {
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
	struct stm32_omi *omi = ospi->omi;
	struct stm32_ospi_flash *flash = &ospi->flash[ospi->last_cs];
	struct spi_device *spi = flash->spi;
	void __iomem *regs_base = omi->regs_base;
	u32 bus_freq;
	u8 window_len_tcr0 = 0, window_len_tcr1 = 0;
	int ret, ret_tcr0, ret_tcr1;

	/*
	 * Set memory device at low frequency (50MHz) and sent
	 * READID (0x9F) command, save the answer as golden answer
	 */
	flash->presc = DIV_ROUND_UP(omi->clk_rate,
				    STM32_DLYB_FREQ_THRESHOLD) - 1;
	stm32_ospi_set_prescaler(ospi, flash->presc);

	ret = stm32_ospi_readid(omi);
	if (ret)
		return ret;

	/*
	 * Set memory device at expected frequency read from DT
	 * prescaler should be set before locking the DLL
	 */
	flash->presc = DIV_ROUND_UP(omi->clk_rate, spi->max_speed_hz) - 1;
	stm32_ospi_set_prescaler(ospi, flash->presc);
	bus_freq = DIV_ROUND_UP(omi->clk_rate, flash->presc + 1);

	/* Calibration needed above 50MHz */
	if (bus_freq <= STM32_DLYB_FREQ_THRESHOLD)
		return 0;

	flash->dcr_reg &= ~DCR1_DLYBYP;
	writel_relaxed(flash->dcr_reg, regs_base + OSPI_DCR1);

	ret = stm32_omi_dlyb_init(omi, false, 0);
	if (ret)
		return ret;

	/*
	 * Perform only Rx TAP selection
	 * When DTR support will be added, Rx/Tx TAP selection will have to
	 * be performed
	 */
	ret_tcr0 = stm32_omi_dlyb_find_tap(omi, true, &window_len_tcr0);
	if (!ret_tcr0)
		/*
		 * Save flash delay block configuration, will be restored
		 * each time this flash is addressed
		 */
		stm32_omi_dlyb_get_cr(omi, &flash->dlyb_cr_str);

	stm32_omi_dlyb_stop(omi);
	ret = stm32_omi_dlyb_init(omi, false, 0);
	if (ret)
		return ret;

	flash->sample_later = true;
	ret_tcr1 = stm32_omi_dlyb_find_tap(omi, true, &window_len_tcr1);
	if (ret_tcr0 && ret_tcr1) {
		dev_info(ospi->dev, "Calibration phase failed\n");
		return ret_tcr0;
	}

	if (window_len_tcr0 >= window_len_tcr1) {
		flash->sample_later = false;
		stm32_omi_dlyb_stop(omi);

		ret = stm32_omi_dlyb_set_cr(omi, flash->dlyb_cr_str);
		if (ret)
			return ret;
	} else {
		stm32_omi_dlyb_get_cr(omi, &flash->dlyb_cr_str);
	}

	flash->str_sample_later = flash->sample_later;

	return 0;
}

static int stm32_ospi_dtr_calibration(struct stm32_ospi *ospi)
{
	struct stm32_omi *omi = ospi->omi;
	struct stm32_ospi_flash *flash = &ospi->flash[ospi->last_cs];
	void __iomem *regs_base = omi->regs_base;
	u32 prescaler, bus_freq;
	u16 period_ps = 0;
	u8 window_len = 0;
	int ret;
	bool bypass_mode = false;

	flash->dcr_reg &= ~DCR1_DLYBYP;
	writel_relaxed(flash->dcr_reg, regs_base + OSPI_DCR1);

	prescaler = FIELD_GET(DCR2_PRESC_MASK,
			      readl(regs_base + OSPI_DCR2));
	bus_freq = DIV_ROUND_UP(omi->clk_rate, prescaler + 1);

	if (prescaler)
		writel_relaxed(TCR_DHQC, regs_base + OSPI_TCR);

	if (bus_freq <= STM32_DLYB_FREQ_THRESHOLD) {
		bypass_mode = true;
		period_ps = NSEC_PER_SEC / (bus_freq / 1000);
	}

	ret = stm32_omi_dlyb_init(omi, bypass_mode, period_ps);
	if (ret) {
		dev_err(ospi->dev, "Can't enable delay block\n");
		return 0;
	}

	if (bypass_mode || prescaler)
		/* Perform only RX TAP selection */
		ret = stm32_omi_dlyb_find_tap(omi, true, &window_len);
	else
		/* Perform RX/TX TAP selection */
		ret = stm32_omi_dlyb_find_tap(omi, false, &window_len);

	if (ret) {
		dev_err(ospi->dev, "Calibration failed\n");
		if (!bypass_mode)
			/* Stop delay block when configured in lock mode */
			stm32_omi_dlyb_stop(omi);
	} else {
		stm32_omi_dlyb_get_cr(omi, &flash->dlyb_cr_dtr);
		flash->dtr_calibration_done_once = true;
	}

	return ret;
}

static int stm32_ospi_prepare_to_send(struct spi_device *spi,
				      const struct spi_mem_op *op)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(spi->master);
	struct stm32_ospi_flash *flash = &ospi->flash[spi->chip_select];
	struct stm32_omi *omi = ospi->omi;
	int ret = 0;
	bool set_dlyb = false;

	if (ospi->last_cs != spi->chip_select) {
		ospi->last_cs = spi->chip_select;

		stm32_omi_dlyb_stop(omi);
		stm32_ospi_set_prescaler(ospi, flash->presc);
		set_dlyb = true;
	};

	if (op->cmd.dtr && !flash->dtr_calibration_done_once) {
		stm32_omi_dlyb_stop(omi);
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
		stm32_omi_dlyb_stop(omi);
		set_dlyb = true;
	}

	/*
	 * restore DLYB CR register depending on simple or
	 * double transfer rate
	 */
	if (set_dlyb) {
		if (op->cmd.dtr) {
			ret = stm32_omi_dlyb_set_cr(omi, flash->dlyb_cr_dtr);
			flash->sample_later = false;
			flash->octal_dtr = (op->cmd.nbytes == 2);
		} else {
			ret = stm32_omi_dlyb_set_cr(omi, flash->dlyb_cr_str);
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
	struct stm32_ospi *ospi = spi_controller_get_devdata(mem->spi->master);
	struct stm32_omi *omi = ospi->omi;
	void __iomem *regs_base = omi->regs_base;
	int ret;

	ret = pm_runtime_resume_and_get(omi->dev);
	if (ret < 0)
		return ret;

	mutex_lock(&ospi->lock);
	writel_relaxed(mask, regs_base + OSPI_PSMKR);
	writel_relaxed(match, regs_base + OSPI_PSMAR);
	omi->fmode = CR_FMODE_APM;
	ospi->status_timeout = timeout_ms;

	ret = stm32_ospi_prepare_to_send(mem->spi, op);
	if (!ret)
		ret = stm32_ospi_send(mem->spi, op);

	mutex_unlock(&ospi->lock);
	pm_runtime_mark_last_busy(omi->dev);
	pm_runtime_put_autosuspend(omi->dev);

	return ret;
}

static int stm32_ospi_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(mem->spi->master);
	struct stm32_omi *omi = ospi->omi;
	int ret;

	ret = pm_runtime_resume_and_get(omi->dev);
	if (ret < 0)
		return ret;

	mutex_lock(&ospi->lock);
	if (op->data.dir == SPI_MEM_DATA_IN && op->data.nbytes)
		omi->fmode = CR_FMODE_INDR;
	else
		omi->fmode = CR_FMODE_INDW;

	ret = stm32_ospi_prepare_to_send(mem->spi, op);
	if (!ret)
		ret = stm32_ospi_send(mem->spi, op);

	mutex_unlock(&ospi->lock);
	pm_runtime_mark_last_busy(omi->dev);
	pm_runtime_put_autosuspend(omi->dev);

	return ret;
}

static int stm32_ospi_dirmap_create(struct spi_mem_dirmap_desc *desc)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(desc->mem->spi->master);
	struct stm32_omi *omi = ospi->omi;

	if (desc->info.op_tmpl.data.dir == SPI_MEM_DATA_OUT)
		return -EOPNOTSUPP;

	/* Should never happen, as mm_base == null is an error probe exit condition */
	if (!omi->mm_base && desc->info.op_tmpl.data.dir == SPI_MEM_DATA_IN)
		return -EOPNOTSUPP;

	if (!omi->mm_size)
		return -EOPNOTSUPP;

	return 0;
}

static ssize_t stm32_ospi_dirmap_read(struct spi_mem_dirmap_desc *desc,
				      u64 offs, size_t len, void *buf)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(desc->mem->spi->master);
	struct stm32_omi *omi = ospi->omi;
	struct spi_mem_op op;
	u32 addr_max;
	int ret;

	ret = pm_runtime_resume_and_get(omi->dev);
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
	if (addr_max < omi->mm_size && op.addr.buswidth)
		omi->fmode = CR_FMODE_MM;
	else
		omi->fmode = CR_FMODE_INDR;

	ret = stm32_ospi_prepare_to_send(desc->mem->spi, &op);
	if (!ret)
		ret = stm32_ospi_send(desc->mem->spi, &op);

	mutex_unlock(&ospi->lock);
	pm_runtime_mark_last_busy(omi->dev);
	pm_runtime_put_autosuspend(omi->dev);

	return ret ?: len;
}

static int stm32_ospi_transfer_one_message(struct spi_controller *ctrl,
					   struct spi_message *msg)
{
	struct stm32_ospi *ospi = spi_controller_get_devdata(ctrl);
	struct stm32_omi *omi = ospi->omi;
	struct spi_transfer *transfer;
	struct spi_device *spi = msg->spi;
	struct spi_mem_op op;
	int ret = 0;

	if (!spi->cs_gpiod)
		return -EOPNOTSUPP;

	ret = pm_runtime_resume_and_get(omi->dev);
	if (ret < 0)
		return ret;

	mutex_lock(&ospi->lock);

	gpiod_set_value_cansleep(spi->cs_gpiod, true);

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
			omi->fmode = CR_FMODE_INDR;
			op.data.buswidth = transfer->rx_nbits;
			op.data.dir = SPI_MEM_DATA_IN;
			op.data.buf.in = transfer->rx_buf;
		} else {
			omi->fmode = CR_FMODE_INDW;
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
	gpiod_set_value_cansleep(spi->cs_gpiod, false);

	mutex_unlock(&ospi->lock);

	msg->status = ret;
	spi_finalize_current_message(ctrl);

	pm_runtime_mark_last_busy(omi->dev);
	pm_runtime_put_autosuspend(omi->dev);

	return ret;
}

static int stm32_ospi_setup(struct spi_device *spi)
{
	struct spi_controller *ctrl = spi->master;
	struct stm32_ospi *ospi = spi_controller_get_devdata(ctrl);
	struct stm32_omi *omi = ospi->omi;
	struct stm32_ospi_flash *flash;
	void __iomem *regs_base = omi->regs_base;
	u32 bus_freq;
	int ret;

	if (ctrl->busy)
		return -EBUSY;

	if (!spi->max_speed_hz)
		return -EINVAL;

	ret = pm_runtime_resume_and_get(omi->dev);
	if (ret < 0)
		return ret;

	flash = &ospi->flash[spi->chip_select];
	flash->presc = DIV_ROUND_UP(omi->clk_rate, spi->max_speed_hz) - 1;
	flash->spi = spi;

	mutex_lock(&ospi->lock);

	ospi->cr_reg = CR_APMS | 3 << CR_FTHRES_SHIFT | CR_EN;
	writel_relaxed(ospi->cr_reg, regs_base + OSPI_CR);

	/* Stop the DLL */
	stm32_omi_dlyb_stop(omi);

	/* Set dcr fsize to max address */
	flash->dcr_reg = DCR1_DEVSIZE_MASK | DCR1_DLYBYP;
	writel_relaxed(flash->dcr_reg, regs_base + OSPI_DCR1);

	stm32_ospi_set_prescaler(ospi, flash->presc);

	ospi->last_cs = spi->chip_select;
	if (flash->is_octal_bus) {
		if (gpiod_count(omi->dev, "cs") == -ENOENT) {
			dev_err(omi->dev, "Can't find cs-gpio property in DT\n");

			ret = -EINVAL;
			goto exit;
		}

		bus_freq = DIV_ROUND_UP(omi->clk_rate, flash->presc + 1);
		if (bus_freq > STM32_DLYB_FREQ_THRESHOLD) {
			dev_err(ospi->dev, "Octal bus mode not supported above %d Hz)\n",
				STM32_DLYB_FREQ_THRESHOLD);

			ret = -EINVAL;
			goto exit;
		}
	} else {
		flash->is_str_calibration = true;

		ret = stm32_ospi_str_calibration(ospi);
		if (ret) {
			ret = 0;

			dev_info(ospi->dev, "Set flash frequency to a safe value (%d Hz)\n",
				 STM32_DLYB_FREQ_THRESHOLD);

			/* Stop the DLL */
			stm32_omi_dlyb_stop(omi);
			flash->sample_later = false;

			flash->presc = DIV_ROUND_UP(omi->clk_rate,
						    STM32_DLYB_FREQ_THRESHOLD) - 1;
			stm32_ospi_set_prescaler(ospi, flash->presc);

			flash->dcr_reg |= DCR1_DLYBYP;
			writel_relaxed(flash->dcr_reg, regs_base + OSPI_DCR1);
		}

		flash->is_str_calibration = false;
	}

exit:
	mutex_unlock(&ospi->lock);
	pm_runtime_mark_last_busy(omi->dev);
	pm_runtime_put_autosuspend(omi->dev);

	return ret;
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
	.exec_op = stm32_ospi_exec_op,
	.dirmap_create = stm32_ospi_dirmap_create,
	.dirmap_read = stm32_ospi_dirmap_read,
	.poll_status = stm32_ospi_poll_status,
	.supports_op = stm32_ospi_supports_mem_op,
};

static const struct spi_controller_mem_caps stm32_ospi_mem_caps = {
	.dtr = true,
	.dtr_swab16 = true,
};

static int stm32_ospi_probe(struct platform_device *pdev)
{
	struct device *parent = pdev->dev.parent;
	struct device *dev = &pdev->dev;
	struct spi_controller *ctrl;
	struct stm32_ospi *ospi;
	struct stm32_omi *omi = dev_get_drvdata(parent);
	struct dma_slave_config dma_cfg;
	struct device_node *child;
	int ret;

	ctrl = devm_spi_alloc_master(dev, sizeof(*ospi));
	if (!ctrl)
		return -ENOMEM;

	ospi = spi_controller_get_devdata(ctrl);
	ospi->ctrl = ctrl;
	ospi->omi = omi;
	omi->check_transfer = stm32_ospi_readid;

	ospi->dev = &pdev->dev;
	platform_set_drvdata(pdev, ospi);

	memset(&dma_cfg, 0, sizeof(dma_cfg));
	dma_cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_cfg.src_addr = omi->regs_phys_base + OSPI_DR;
	dma_cfg.dst_addr = omi->regs_phys_base + OSPI_DR;
	dma_cfg.src_maxburst = 4;
	dma_cfg.dst_maxburst = 4;
	stm32_omi_dma_setup(omi, &dma_cfg);

	mutex_init(&ospi->lock);

	ctrl->mode_bits = SPI_RX_DUAL | SPI_RX_QUAD |
			  SPI_TX_DUAL | SPI_TX_QUAD |
			  SPI_TX_OCTAL | SPI_RX_OCTAL;
	ctrl->setup = stm32_ospi_setup;
	ctrl->bus_num = -1;
	ctrl->mem_ops = &stm32_ospi_mem_ops;
	ctrl->mem_caps = &stm32_ospi_mem_caps;
	ctrl->use_gpio_descriptors = true;
	ctrl->transfer_one_message = stm32_ospi_transfer_one_message;
	ctrl->num_chipselect = STM32_OMI_MAX_NORCHIP;
	ctrl->dev.of_node = parent->of_node;

	pm_runtime_enable(omi->dev);
	pm_runtime_set_autosuspend_delay(omi->dev, STM32_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(omi->dev);

	ret = pm_runtime_resume_and_get(omi->dev);
	if (ret < 0)
		goto err_pm_enable;

	if (omi->rstc) {
		reset_control_assert(omi->rstc);
		udelay(2);
		reset_control_deassert(omi->rstc);
	}

	/* Find memory model on each child node (SPI NOR or SPI NAND) */
	for_each_available_child_of_node(omi->dev->of_node, child) {
		u32 cs;

		ret = of_property_read_u32(child, "reg", &cs);
		if (ret) {
			dev_err(ospi->dev, "could not retrieve reg property: %d\n",
				ret);
			of_node_put(child);
			goto err_pm_resume;
		}

		if (cs >= STM32_OMI_MAX_NORCHIP) {
			dev_err(ospi->dev, "invalid reg value: %d\n", cs);
			of_node_put(child);
			ret = -EINVAL;
			goto err_pm_resume;
		}

		if (of_device_is_compatible(child, "jedec,spi-nor")) {
			struct stm32_ospi_flash *flash = &ospi->flash[cs];

			flash->is_spi_nor = true;
		}

		if (of_device_is_compatible(child, "st,octal-bus")) {
			struct stm32_ospi_flash *flash = &ospi->flash[cs];

			flash->is_octal_bus = true;
		}
	}

	ret = spi_register_master(ctrl);
	if (ret) {
		/* Disable ospi */
		writel_relaxed(0, omi->regs_base + OSPI_CR);
		goto err_pm_resume;
	}

	pm_runtime_mark_last_busy(omi->dev);
	pm_runtime_put_autosuspend(omi->dev);

	return 0;

err_pm_resume:
	pm_runtime_put_sync_suspend(omi->dev);

err_pm_enable:
	pm_runtime_force_suspend(omi->dev);
	mutex_destroy(&ospi->lock);

	return ret;
}

static int stm32_ospi_remove(struct platform_device *pdev)
{
	struct stm32_ospi *ospi = platform_get_drvdata(pdev);
	struct stm32_omi *omi = ospi->omi;
	int ret;

	ret = pm_runtime_resume_and_get(omi->dev);
	if (ret < 0)
		return ret;

	spi_unregister_master(ospi->ctrl);
	/* Disable ospi */
	writel_relaxed(0, omi->regs_base + OSPI_CR);
	stm32_omi_dlyb_stop(omi);
	mutex_destroy(&ospi->lock);
	pm_runtime_put_sync_suspend(omi->dev);
	pm_runtime_force_suspend(omi->dev);

	return 0;
}

static int __maybe_unused stm32_ospi_suspend(struct device *dev)
{
	struct stm32_ospi *ospi = dev_get_drvdata(dev);
	struct stm32_omi *omi = ospi->omi;

	pinctrl_pm_select_sleep_state(dev);

	return pm_runtime_force_suspend(omi->dev);
}

static int __maybe_unused stm32_ospi_resume(struct device *dev)
{
	struct stm32_ospi *ospi = dev_get_drvdata(dev);
	struct stm32_omi *omi = ospi->omi;
	void __iomem *regs_base = omi->regs_base;
	int ret;

	ret = pm_runtime_force_resume(omi->dev);
	if (ret < 0)
		return ret;

	pinctrl_pm_select_default_state(dev);

	ret = pm_runtime_resume_and_get(omi->dev);
	if (ret < 0)
		return ret;

	writel_relaxed(ospi->cr_reg, regs_base + OSPI_CR);
	ospi->last_cs = -1;
	pm_runtime_mark_last_busy(omi->dev);
	pm_runtime_put_autosuspend(omi->dev);

	return 0;
}

static const struct dev_pm_ops stm32_ospi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stm32_ospi_suspend, stm32_ospi_resume)
};

static struct platform_driver stm32_ospi_driver = {
	.probe	= stm32_ospi_probe,
	.remove	= stm32_ospi_remove,
	.driver	= {
		.name = "stm32-ospi",
		.pm = &stm32_ospi_pm_ops,
	},
};
module_platform_driver(stm32_ospi_driver);

MODULE_DESCRIPTION("STMicroelectronics STM32 OCTO SPI driver");
MODULE_LICENSE("GPL");
