/*
 * STMicroelectronics STM32 SPI Controller driver (master mode only)
 *
 * Copyright (C) 2017, STMicroelectronics - All Rights Reserved
 * Author(s): Amelie Delaunay <amelie.delaunay@st.com> for STMicroelectronics.
 *
 * License terms: GPL V2.0.
 *
 * spi_stm32 driver is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * spi_stm32 driver is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * spi_stm32 driver. If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>

#define DRIVER_NAME "spi_stm32"

/* STM32 SPI registers */
#define STM32_SPI_CR1		0x00
#define STM32_SPI_CR2		0x04
#define STM32_SPI_CFG1		0x08
#define STM32_SPI_CFG2		0x0C
#define STM32_SPI_IER		0x10
#define STM32_SPI_SR		0x14
#define STM32_SPI_IFCR		0x18
#define STM32_SPI_TXDR		0x20
#define STM32_SPI_RXDR		0x30
#define STM32_SPI_UDRDR		0x4C
#define STM32_SPI_I2SCFGR	0x50

/* STM32_SPI_CR1 bit fields */
#define SPI_CR1_SPE		BIT(0)
#define SPI_CR1_MASRX		BIT(8)
#define SPI_CR1_CSTART		BIT(9)
#define SPI_CR1_CSUSP		BIT(10)
#define SPI_CR1_HDDIR		BIT(11)
#define SPI_CR1_SSI		BIT(12)

/* STM32_SPI_CR2 bit fields */
#define SPI_CR2_TSIZE		GENMASK(15, 0)
#define SPI_TSIZE_MAX		FIELD_GET(SPI_CR2_TSIZE, SPI_CR2_TSIZE)

/* STM32_SPI_CFG1 bit fields */
#define SPI_CFG1_DSIZE		GENMASK(4, 0)
#define SPI_CFG1_FTHLV		GENMASK(8, 5)
#define SPI_CFG1_UDRDET		GENMASK(12, 11)
#define SPI_CFG1_UDRCFG		GENMASK(10, 9)
#define SPI_CFG1_RXDMAEN	BIT(14)
#define SPI_CFG1_TXDMAEN	BIT(15)
#define SPI_CFG1_MBR		GENMASK(30, 28)
#define SPI_CFG1_MBR_MIN	0
#define SPI_CFG1_MBR_MAX	FIELD_GET(SPI_CFG1_MBR, SPI_CFG1_MBR)

/* STM32_SPI_CFG2 bit fields */
#define SPI_CFG2_MIDI		GENMASK(7, 4)
#define SPI_CFG2_COMM		GENMASK(18, 17)
#define SPI_CFG2_SP		GENMASK(21, 19)
#define SPI_CFG2_MASTER		BIT(22)
#define SPI_CFG2_LSBFRST	BIT(23)
#define SPI_CFG2_CPHA		BIT(24)
#define SPI_CFG2_CPOL		BIT(25)
#define SPI_CFG2_SSM		BIT(26)
#define SPI_CFG2_SSOE		BIT(29)
#define SPI_CFG2_AFCNTR		BIT(31)

/* STM32_SPI_IER bit fields */
#define SPI_IER_RXPIE		BIT(0)
#define SPI_IER_TXPIE		BIT(1)
#define SPI_IER_DXPIE		BIT(2)
#define SPI_IER_EOTIE		BIT(3)
#define SPI_IER_TXTFIE		BIT(4)
#define SPI_IER_UDRIE		BIT(5)
#define SPI_IER_OVRIE		BIT(6)
#define SPI_IER_ALL		GENMASK(10, 0)

/* STM32_SPI_SR bit fields */
#define SPI_SR_RXP		BIT(0)
#define SPI_SR_TXP		BIT(1)
#define SPI_SR_EOT		BIT(3)
#define SPI_SR_TXTF		BIT(4)
#define SPI_SR_OVR		BIT(6)
#define SPI_SR_UDR		BIT(5)
#define SPI_SR_SUSP		BIT(11)
#define SPI_SR_RXPLVL		GENMASK(14, 13)
#define SPI_SR_RXWNE		BIT(15)

/* STM32_SPI_IFCR bit fields */
#define SPI_IFCR_ALL		GENMASK(11, 3)

/* STM32_SPI_I2SCFGR bit fields */
#define SPI_I2SCFGR_I2SMOD	BIT(0)

/* SPI Master Baud Rate min/max divisor */
#define SPI_MBR_DIV_MIN		(2 << SPI_CFG1_MBR_MIN)
#define SPI_MBR_DIV_MAX		(2 << SPI_CFG1_MBR_MAX)

/* SPI Communication mode */
#define SPI_FULL_DUPLEX		0
#define SPI_SIMPLEX_TX		1
#define SPI_SIMPLEX_RX		2
#define SPI_HALF_DUPLEX		3
#define SPI_UDRDET_MODE		2

/**
 * struct stm32_spi - private data of the SPI controller
 * @dev: driver model representation of the controller
 * @master: controller master interface
 * @base: virtual memory area
 * @clk: hw kernel clock feeding the SPI clock generator
 * @clk_rate: rate of the hw kernel clock feeding the SPI clock generator
 * @rst: SPI controller reset line
 * @lock: prevent I/O concurrent access
 * @irq: SPI controller interrupt line
 * @fifo_size: size of the embedded fifo in bytes
 * @cur_midi: master inter-data idleness in ns
 * @cur_speed: speed configured in Hz
 * @cur_bpw: number of bits in a single SPI data frame
 * @cur_fthlv: fifo threshold level (data frames in a single data packet)
 * @cur_comm: SPI communication mode
 * @cur_xferlen: current transfer length in bytes
 * @cur_usedma: boolean to know if dma is used in current transfer
 * @tx_buf: data to be written, or NULL
 * @rx_buf: data to be read, or NULL
 * @tx_len: number of data to be written in bytes
 * @rx_len: number of data to be read in bytes
 * @dma_tx: dma channel for TX transfer
 * @dma_rx: dma channel for RX transfer
 * @phys_addr: SPI registers physical base address
 */
struct stm32_spi {
	struct device *dev;
	struct spi_master *master;
	void __iomem *base;
	struct clk *clk;
	u32 clk_rate;
	struct reset_control *rst;
	spinlock_t lock; /* prevent I/O concurrent access */
	int irq;
	unsigned int fifo_size;

	unsigned int cur_midi;
	unsigned int cur_speed;
	unsigned int cur_bpw;
	unsigned int cur_fthlv;
	unsigned int cur_comm;
	unsigned int cur_xferlen;
	bool cur_usedma;

	const void *tx_buf;
	void *rx_buf;
	int tx_len;
	int rx_len;
	struct dma_chan *dma_tx;
	struct dma_chan *dma_rx;
	struct completion dma_completion;
	dma_addr_t phys_addr;
	struct completion xfer_completion;
	int xfer_status;

	/* Slave mode */
	bool slave_mode;
	bool slave_aborted;
};

static inline void stm32_spi_set_bits(struct stm32_spi *spi,
				      u32 offset, u32 bits)
{
	writel_relaxed(readl_relaxed(spi->base + offset) | bits,
		       spi->base + offset);
}

static inline void stm32_spi_clr_bits(struct stm32_spi *spi,
				      u32 offset, u32 bits)
{
	writel_relaxed(readl_relaxed(spi->base + offset) & ~bits,
		       spi->base + offset);
}

/**
 * stm32_spi_get_fifo_size - Return fifo size
 * @spi: pointer to the spi controller data structure
 */
static int stm32_spi_get_fifo_size(struct stm32_spi *spi)
{
	unsigned long flags;
	u32 count = 0;

	spin_lock_irqsave(&spi->lock, flags);

	stm32_spi_set_bits(spi, STM32_SPI_CR1, SPI_CR1_SPE);

	while (readl_relaxed(spi->base + STM32_SPI_SR) & SPI_SR_TXP)
		writeb_relaxed(++count, spi->base + STM32_SPI_TXDR);

	stm32_spi_clr_bits(spi, STM32_SPI_CR1, SPI_CR1_SPE);

	spin_unlock_irqrestore(&spi->lock, flags);

	dev_dbg(spi->dev, "%d x 8-bit fifo size\n", count);

	return count;
}

/**
 * stm32_spi_get_bpw_mask - Return bits per word mask
 * @spi: pointer to the spi controller data structure
 */
static int stm32_spi_get_bpw_mask(struct stm32_spi *spi)
{
	unsigned long flags;
	u32 cfg1, max_bpw;

	spin_lock_irqsave(&spi->lock, flags);

	/*
	 * The most significant bit at DSIZE bit field is reserved when the
	 * maximum data size of periperal instances is limited to 16-bit
	 */
	stm32_spi_set_bits(spi, STM32_SPI_CFG1, SPI_CFG1_DSIZE);

	cfg1 = readl_relaxed(spi->base + STM32_SPI_CFG1);
	max_bpw = FIELD_GET(SPI_CFG1_DSIZE, cfg1) + 1;

	spin_unlock_irqrestore(&spi->lock, flags);

	dev_dbg(spi->dev, "%d-bit maximum data frame\n", max_bpw);

	return SPI_BPW_RANGE_MASK(4, max_bpw);
}

/**
 * stm32_spi_prepare_mbr - Determine SPI_CFG1.MBR value
 * @spi: pointer to the spi controller data structure
 * @speed_hz: requested speed
 *
 * Return SPI_CFG1.MBR value in case of success or -EINVAL
 */
static int stm32_spi_prepare_mbr(struct stm32_spi *spi, u32 speed_hz)
{
	u32 div, mbrdiv;

	/* Ensure spi->clk_rate is even */
	div = DIV_ROUND_UP(spi->clk_rate & ~0x1, speed_hz);

	/*
	 * SPI framework set xfer->speed_hz to master->max_speed_hz if
	 * xfer->speed_hz is greater than master->max_speed_hz, and it returns
	 * an error when xfer->speed_hz is lower than master->min_speed_hz, so
	 * no need to check it there.
	 * However, we need to ensure the following calculations.
	 */
	if (div < SPI_MBR_DIV_MIN ||
	    div > SPI_MBR_DIV_MAX)
		return -EINVAL;

	/* Determine the first power of 2 greater than or equal to div */
	if (div & (div - 1))
		mbrdiv = fls(div);
	else
		mbrdiv = fls(div) - 1;

	spi->cur_speed = spi->clk_rate / (1 << mbrdiv);

	return mbrdiv - 1;
}

/**
 * stm32_spi_prepare_fthlv - Determine FIFO threshold level
 * @spi: pointer to the spi controller data structure
 */
static u32 stm32_spi_prepare_fthlv(struct stm32_spi *spi, u32 xfer_len)
{
	u32 fthlv, half_fifo, packet;

	/* data packet should not exceed 1/2 of fifo space */
	half_fifo = (spi->fifo_size / 2);

	/* data_packet should not exceed transfer length */
	packet = (half_fifo > xfer_len) ? xfer_len : half_fifo;

	if (spi->cur_bpw <= 8)
		fthlv = packet;
	else if (spi->cur_bpw <= 16)
		fthlv = packet / 2;
	else
		fthlv = packet / 4;

	/* align packet size with data registers access */
	if (spi->cur_bpw > 8)
		fthlv -= (fthlv % 2); /* multiple of 2 */
	else
		fthlv -= (fthlv % 4); /* multiple of 4 */

	if (!fthlv)
		fthlv = 1;

	return fthlv;
}

/**
 * stm32_spi_write_txfifo - Write bytes in Transmit Data Register
 * @spi: pointer to the spi controller data structure
 *
 * Read from tx_buf depends on remaining bytes to avoid to read beyond
 * tx_buf end.
 */
static void stm32_spi_write_txfifo(struct stm32_spi *spi)
{
	while ((spi->tx_len > 0) &&
	       (readl_relaxed(spi->base + STM32_SPI_SR) & SPI_SR_TXP)) {
		u32 offs = spi->cur_xferlen - spi->tx_len;

		if (spi->tx_len >= sizeof(u32)) {
			const u32 *tx_buf32 = (const u32 *)(spi->tx_buf + offs);

			writel_relaxed(*tx_buf32, spi->base + STM32_SPI_TXDR);
			spi->tx_len -= sizeof(u32);
		} else if (spi->tx_len >= sizeof(u16)) {
			const u16 *tx_buf16 = (const u16 *)(spi->tx_buf + offs);

			writew_relaxed(*tx_buf16, spi->base + STM32_SPI_TXDR);
			spi->tx_len -= sizeof(u16);
		} else {
			const u8 *tx_buf8 = (const u8 *)(spi->tx_buf + offs);

			writeb_relaxed(*tx_buf8, spi->base + STM32_SPI_TXDR);
			spi->tx_len -= sizeof(u8);
		}
	}

	dev_dbg(spi->dev, "%s: %d bytes left\n", __func__, spi->tx_len);
}

/**
 * stm32_spi_read_rxfifo - Read bytes in Receive Data Register
 * @spi: pointer to the spi controller data structure
 *
 * Write in rx_buf depends on remaining bytes to avoid to write beyond
 * rx_buf end.
 */
static void stm32_spi_read_rxfifo(struct stm32_spi *spi)
{
	u32 sr = readl_relaxed(spi->base + STM32_SPI_SR);
	u32 rxplvl = FIELD_GET(SPI_SR_RXPLVL, sr);

	while ((spi->rx_len > 0) &&
	       ((sr & SPI_SR_RXP) ||
		((sr & SPI_SR_EOT) && ((sr & SPI_SR_RXWNE) || (rxplvl > 0))))) {
		u32 offs = spi->cur_xferlen - spi->rx_len;

		if ((spi->rx_len >= sizeof(u32)) || (sr & SPI_SR_RXWNE)) {
			u32 *rx_buf32 = (u32 *)(spi->rx_buf + offs);

			*rx_buf32 = readl_relaxed(spi->base + STM32_SPI_RXDR);
			spi->rx_len -= sizeof(u32);
		} else if ((spi->rx_len >= sizeof(u16)) ||
			   (!(sr & SPI_SR_RXWNE) &&
			    (rxplvl >= 2 || spi->cur_bpw > 8))) {
			u16 *rx_buf16 = (u16 *)(spi->rx_buf + offs);

			*rx_buf16 = readw_relaxed(spi->base + STM32_SPI_RXDR);
			spi->rx_len -= sizeof(u16);
		} else {
			u8 *rx_buf8 = (u8 *)(spi->rx_buf + offs);

			*rx_buf8 = readb_relaxed(spi->base + STM32_SPI_RXDR);
			spi->rx_len -= sizeof(u8);
		}

		sr = readl_relaxed(spi->base + STM32_SPI_SR);
		rxplvl = FIELD_GET(SPI_SR_RXPLVL, sr);
	}

	dev_dbg(spi->dev, "%s: %d bytes left (sr=%08x)\n",
		__func__, spi->rx_len, sr);
}

/**
 * stm32_spi_enable - Enable SPI controller
 * @spi: pointer to the spi controller data structure
 *
 * SPI data transfer is enabled but spi_ker_ck is idle.
 * SPI_CFG1 and SPI_CFG2 are now write protected.
 */
static void stm32_spi_enable(struct stm32_spi *spi)
{
	dev_dbg(spi->dev, "enable controller\n");

	stm32_spi_set_bits(spi, STM32_SPI_CR1, SPI_CR1_SPE);
}

/**
 * stm32_spi_disable - Disable SPI controller
 * @spi: pointer to the spi controller data structure
 *
 * RX-Fifo is flushed when SPI controller is disabled. To prevent any data
 * loss, use stm32_spi_read_rxfifo to read the remaining bytes in RX-Fifo.
 */
static void stm32_spi_disable(struct stm32_spi *spi)
{
	unsigned long flags;
	u32 cr1, sr;

	dev_dbg(spi->dev, "disable controller\n");

	spin_lock_irqsave(&spi->lock, flags);

	cr1 = readl_relaxed(spi->base + STM32_SPI_CR1);

	if (!(cr1 & SPI_CR1_SPE)) {
		spin_unlock_irqrestore(&spi->lock, flags);
		return;
	}

	/* Wait on EOT or suspend the flow */
	if (readl_relaxed_poll_timeout_atomic(spi->base + STM32_SPI_SR,
					      sr, !(sr & SPI_SR_EOT),
					      10, 100000) < 0) {
		if (cr1 & SPI_CR1_CSTART) {
			writel_relaxed(cr1 | SPI_CR1_CSUSP,
				       spi->base + STM32_SPI_CR1);
			if (readl_relaxed_poll_timeout_atomic(
						spi->base + STM32_SPI_SR,
						sr, !(sr & SPI_SR_SUSP),
						10, 100000) < 0)
				dev_warn(spi->dev,
					 "Suspend request timeout\n");
		}
	}

	if (!spi->cur_usedma && spi->rx_buf && (spi->rx_len > 0))
		stm32_spi_read_rxfifo(spi);

	if (spi->cur_usedma && spi->tx_buf)
		dmaengine_terminate_all(spi->dma_tx);
	if (spi->cur_usedma && spi->rx_buf)
		dmaengine_terminate_all(spi->dma_rx);

	stm32_spi_clr_bits(spi, STM32_SPI_CR1, SPI_CR1_SPE);

	stm32_spi_clr_bits(spi, STM32_SPI_CFG1, SPI_CFG1_TXDMAEN |
						SPI_CFG1_RXDMAEN);

	/* Disable interrupts and clear status flags */
	writel_relaxed(0, spi->base + STM32_SPI_IER);
	writel_relaxed(SPI_IFCR_ALL, spi->base + STM32_SPI_IFCR);

	spin_unlock_irqrestore(&spi->lock, flags);
}

/**
 * stm32_spi_can_dma - Determine if the transfer is eligible for DMA use
 *
 * If the current transfer size is greater than fifo size, use DMA.
 */
static bool stm32_spi_can_dma(struct spi_master *master,
			      struct spi_device *spi_dev,
			      struct spi_transfer *transfer)
{
	struct stm32_spi *spi = spi_master_get_devdata(master);

	dev_dbg(spi->dev, "%s: %s\n", __func__,
		(transfer->len > spi->fifo_size) ? "true" : "false");

	return (transfer->len > spi->fifo_size);
}

/**
 * stm32_spi_irq - Interrupt handler for SPI controller events
 * @irq: interrupt line
 * @dev_id: SPI controller master interface
 */
static irqreturn_t stm32_spi_irq(int irq, void *dev_id)
{
	struct spi_master *master = dev_id;
	struct stm32_spi *spi = spi_master_get_devdata(master);
	u32 sr, ier, mask, ifcr;
	unsigned long flags;
	bool end = false;

	spin_lock_irqsave(&spi->lock, flags);

	sr = readl_relaxed(spi->base + STM32_SPI_SR);
	ier = readl_relaxed(spi->base + STM32_SPI_IER);
	ifcr = 0;

	mask = ier;
	/*
	 * EOTIE enables irq from EOT, SUSP and TXC events. We need to set
	 * SUSP to acknowledge it later. TXC is automatically cleared
	 */
	mask |= SPI_SR_SUSP;
	/*
	 * DXPIE is set in Full-Duplex, one IT will be raised if TXP and RXP
	 * are set. So in case of Full-Duplex, need to poll TXP and RXP event.
	 */
	if ((spi->cur_comm == SPI_FULL_DUPLEX) && (!spi->cur_usedma))
		mask |= SPI_SR_TXP | SPI_SR_RXP;

	mask &= sr;

	if (!mask) {
		dev_warn(spi->dev, "spurious IT (sr=0x%08x, ier=0x%08x)\n",
			 sr, ier);
		spin_unlock_irqrestore(&spi->lock, flags);
		return IRQ_NONE;
	}

	if (mask & SPI_SR_SUSP) {
		dev_warn_once(spi->dev,
			      "System too slow is limiting data throughput\n");

		if (!spi->cur_usedma && (spi->rx_buf && (spi->rx_len > 0)))
			stm32_spi_read_rxfifo(spi);

		ifcr |= SPI_SR_SUSP;
	}

	if (mask & SPI_SR_OVR) {
		dev_err(spi->dev, "Overrun: RX data lost\n");
		spi->xfer_status = -EIO;
		end = true;
		ifcr |= SPI_SR_OVR;
	}

	if (mask & SPI_SR_UDR) {
		dev_err(spi->dev, "Underrun: TX data empty\n");
		spi->xfer_status = -EIO;
		end = true;
		if(spi->slave_mode)
			ifcr |= SPI_SR_UDR;
	}

	if (mask & SPI_SR_TXTF)
		ifcr |= SPI_SR_TXTF;

	if (mask & SPI_SR_EOT) {
		if (!spi->cur_usedma && (spi->rx_buf && (spi->rx_len > 0)))
			stm32_spi_read_rxfifo(spi);
		end = true;
		ifcr |= SPI_SR_EOT;
	}

	if (mask & SPI_SR_TXP)
		if (!spi->cur_usedma && (spi->tx_buf && (spi->tx_len > 0)))
			stm32_spi_write_txfifo(spi);

	if (mask & SPI_SR_RXP)
		if (!spi->cur_usedma && (spi->rx_buf && (spi->rx_len > 0)))
			stm32_spi_read_rxfifo(spi);

	if (end) {
		/* Disable interrupts and clear status flags */
		writel_relaxed(0, spi->base + STM32_SPI_IER);
		writel_relaxed(SPI_IFCR_ALL, spi->base + STM32_SPI_IFCR);

		complete(&spi->xfer_completion);
	} else {
		writel_relaxed(ifcr, spi->base + STM32_SPI_IFCR);
	}

	spin_unlock_irqrestore(&spi->lock, flags);
	return IRQ_HANDLED;
}

/**
 * stm32_spi_setup - setup device chip select
 */
static int stm32_spi_setup(struct spi_device *spi_dev)
{
	int ret = 0;
	struct stm32_spi *spi = spi_master_get_devdata(spi_dev->master);

	if(spi->slave_mode)
		return 0;

	if (!gpio_is_valid(spi_dev->cs_gpio))
		return 0;

	dev_dbg(&spi_dev->dev, "%s: set gpio%d output %s\n", __func__,
		spi_dev->cs_gpio,
		(spi_dev->mode & SPI_CS_HIGH) ? "low" : "high");

	ret = gpio_direction_output(spi_dev->cs_gpio,
				    !(spi_dev->mode & SPI_CS_HIGH));

	return ret;
}

/**
 * stm32_spi_prepare_msg - set up the controller to transfer a single message
 */
static int stm32_spi_prepare_msg(struct spi_master *master,
				 struct spi_message *msg)
{
	struct stm32_spi *spi = spi_master_get_devdata(master);
	struct spi_device *spi_dev = msg->spi;
	struct device_node *np = spi_dev->dev.of_node;
	unsigned long flags;
	u32 cfg2_clrb = 0, cfg2_setb = 0;
	int ret;

	/* SPI slave device may need time between data frames */
	spi->cur_midi = 0;
	if (np && !of_property_read_u32(np, "st,spi-midi-ns", &spi->cur_midi))
		dev_dbg(spi->dev, "%dns inter-data idleness\n", spi->cur_midi);

	if (spi_dev->mode & SPI_CPOL)
		cfg2_setb |= SPI_CFG2_CPOL;
	else
		cfg2_clrb |= SPI_CFG2_CPOL;

	if (spi_dev->mode & SPI_CPHA)
		cfg2_setb |= SPI_CFG2_CPHA;
	else
		cfg2_clrb |= SPI_CFG2_CPHA;

	if (spi_dev->mode & SPI_LSB_FIRST)
		cfg2_setb |= SPI_CFG2_LSBFRST;
	else
		cfg2_clrb |= SPI_CFG2_LSBFRST;

	dev_dbg(spi->dev, "cpol=%d cpha=%d lsb_first=%d cs_high=%d\n",
		spi_dev->mode & SPI_CPOL,
		spi_dev->mode & SPI_CPHA,
		spi_dev->mode & SPI_LSB_FIRST,
		spi_dev->mode & SPI_CS_HIGH);

	/*
	 * On STM32H7, messages should not exceed a maximum size
	 * In order to ensure that, split large messages into several messages
	 */
	ret = spi_split_transfers_maxsize(master, msg,
					  SPI_TSIZE_MAX,
					  GFP_KERNEL | GFP_DMA);
	if (ret)
		return ret;

	spin_lock_irqsave(&spi->lock, flags);

	if (cfg2_clrb || cfg2_setb)
		writel_relaxed(
			(readl_relaxed(spi->base + STM32_SPI_CFG2) &
				~cfg2_clrb) | cfg2_setb,
			       spi->base + STM32_SPI_CFG2);

	spin_unlock_irqrestore(&spi->lock, flags);

	return 0;
}

/**
 * stm32_spi_dma_cb - dma callback
 *
 * DMA callback is called when the transfer is complete.
 */
static void stm32_spi_dma_cb(void *data)
{
	struct stm32_spi *spi = data;
	unsigned long flags;

	spin_lock_irqsave(&spi->lock, flags);

	complete(&spi->dma_completion);

	spin_unlock_irqrestore(&spi->lock, flags);
}

/**
 * stm32_spi_dma_config - configure dma slave channel depending on current
 *			  transfer bits_per_word.
 */
static void stm32_spi_dma_config(struct stm32_spi *spi,
				 struct dma_slave_config *dma_conf,
				 enum dma_transfer_direction dir)
{
	enum dma_slave_buswidth buswidth;
	u32 maxburst;

	if (spi->cur_bpw <= 8)
		buswidth = DMA_SLAVE_BUSWIDTH_1_BYTE;
	else if (spi->cur_bpw <= 16)
		buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
	else
		buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;

	/* Valid for DMA Half or Full Fifo threshold */
	if (spi->cur_fthlv == 2)
		maxburst = 1;
	else
		maxburst = spi->cur_fthlv;

	memset(dma_conf, 0, sizeof(struct dma_slave_config));
	dma_conf->direction = dir;
	if (dma_conf->direction == DMA_DEV_TO_MEM) { /* RX */
		dma_conf->src_addr = spi->phys_addr + STM32_SPI_RXDR;
		dma_conf->src_addr_width = buswidth;
		dma_conf->src_maxburst = maxburst;

		dev_dbg(spi->dev, "Rx DMA config buswidth=%d, maxburst=%d\n",
			buswidth, maxburst);
	} else if (dma_conf->direction == DMA_MEM_TO_DEV) { /* TX */
		dma_conf->dst_addr = spi->phys_addr + STM32_SPI_TXDR;
		dma_conf->dst_addr_width = buswidth;
		dma_conf->dst_maxburst = maxburst;

		dev_dbg(spi->dev, "Tx DMA config buswidth=%d, maxburst=%d\n",
			buswidth, maxburst);
	}
}

/**
 * stm32_spi_transfer_one_irq - transfer a single spi_transfer using
 *				interrupts
 */
static void stm32_spi_transfer_one_irq(struct stm32_spi *spi)
{
	unsigned long flags;
	u32 ier = 0;

	/* Enable the interrupts relative to the current communication mode */
	if (spi->tx_buf && spi->rx_buf)	/* Full Duplex */
		ier |= SPI_IER_DXPIE;
	else if (spi->tx_buf)		/* Half-Duplex TX dir or Simplex TX */
		ier |= SPI_IER_TXPIE;
	else if (spi->rx_buf)		/* Half-Duplex RX dir or Simplex RX */
		ier |= SPI_IER_RXPIE;

	/* Enable the interrupts relative to the end of transfer */
	ier |= SPI_IER_EOTIE | SPI_IER_TXTFIE |	SPI_IER_OVRIE;

	if(spi->slave_mode)
		ier |= SPI_IER_UDRIE;

	spin_lock_irqsave(&spi->lock, flags);

	stm32_spi_enable(spi);

	if(!spi->slave_mode){
		/* Be sure to have data in fifo before starting data transfer */
		if (spi->tx_buf)
			stm32_spi_write_txfifo(spi);

		stm32_spi_set_bits(spi, STM32_SPI_CR1, SPI_CR1_CSTART);
	} else {
		writeb_relaxed(0xee, spi->base + STM32_SPI_UDRDR);
		spi->slave_aborted = false;
	}

	writel_relaxed(ier, spi->base + STM32_SPI_IER);

	spin_unlock_irqrestore(&spi->lock, flags);
}

/**
 * stm32_spi_transfer_one_dma - transfer a single spi_transfer using DMA
 */
static void stm32_spi_transfer_one_dma(struct stm32_spi *spi,
				       struct spi_transfer *xfer)
{
	dma_async_tx_callback rx_done = NULL, tx_done = NULL;
	struct dma_slave_config tx_dma_conf, rx_dma_conf;
	struct dma_async_tx_descriptor *tx_dma_desc, *rx_dma_desc;
	unsigned long flags;
	u32 ier = 0;

	spin_lock_irqsave(&spi->lock, flags);

	if (spi->rx_buf)
		rx_done = stm32_spi_dma_cb;
	else if (spi->tx_buf)
		tx_done = stm32_spi_dma_cb;

	reinit_completion(&spi->dma_completion);

	rx_dma_desc = NULL;
	if (spi->rx_buf) {
		stm32_spi_dma_config(spi, &rx_dma_conf, DMA_DEV_TO_MEM);
		dmaengine_slave_config(spi->dma_rx, &rx_dma_conf);

		/* Enable Rx DMA request */
		stm32_spi_set_bits(spi, STM32_SPI_CFG1, SPI_CFG1_RXDMAEN);

		rx_dma_desc = dmaengine_prep_slave_sg(
					spi->dma_rx, xfer->rx_sg.sgl,
					xfer->rx_sg.nents,
					rx_dma_conf.direction,
					DMA_PREP_INTERRUPT);
	}

	tx_dma_desc = NULL;
	if (spi->tx_buf) {
		stm32_spi_dma_config(spi, &tx_dma_conf, DMA_MEM_TO_DEV);
		dmaengine_slave_config(spi->dma_tx, &tx_dma_conf);

		tx_dma_desc = dmaengine_prep_slave_sg(
					spi->dma_tx, xfer->tx_sg.sgl,
					xfer->tx_sg.nents,
					tx_dma_conf.direction,
					DMA_PREP_INTERRUPT);
	}

	if ((spi->tx_buf && !tx_dma_desc) ||
	    (spi->rx_buf && !rx_dma_desc))
		goto dma_desc_error;

	if (rx_dma_desc) {
		rx_dma_desc->callback = rx_done;
		rx_dma_desc->callback_param = spi;

		if (dma_submit_error(dmaengine_submit(rx_dma_desc))) {
			dev_err(spi->dev, "Rx DMA submit failed\n");
			goto dma_desc_error;
		}
		/* Enable Rx DMA channel */
		dma_async_issue_pending(spi->dma_rx);
	}

	if (tx_dma_desc) {
		if (spi->cur_comm == SPI_SIMPLEX_TX) {
			tx_dma_desc->callback = tx_done;
			tx_dma_desc->callback_param = spi;
		}

		if (dma_submit_error(dmaengine_submit(tx_dma_desc))) {
			dev_err(spi->dev, "Tx DMA submit failed\n");
			goto dma_submit_error;
		}
		/* Enable Tx DMA channel */
		dma_async_issue_pending(spi->dma_tx);

		/* Enable Tx DMA request */
		stm32_spi_set_bits(spi, STM32_SPI_CFG1, SPI_CFG1_TXDMAEN);
	}

	/* Enable the interrupts relative to the end of transfer */
	ier |= SPI_IER_EOTIE | SPI_IER_TXTFIE |	SPI_IER_OVRIE;
	writel_relaxed(ier, spi->base + STM32_SPI_IER);

	stm32_spi_enable(spi);

	stm32_spi_set_bits(spi, STM32_SPI_CR1, SPI_CR1_CSTART);

	spin_unlock_irqrestore(&spi->lock, flags);

	return;

dma_submit_error:
	if (spi->rx_buf)
		dmaengine_terminate_all(spi->dma_rx);

dma_desc_error:
	stm32_spi_clr_bits(spi, STM32_SPI_CFG1, SPI_CFG1_RXDMAEN);

	spin_unlock_irqrestore(&spi->lock, flags);

	dev_info(spi->dev, "DMA issue: fall back to irq transfer\n");

	spi->cur_usedma = false;
	stm32_spi_transfer_one_irq(spi);
}

/**
 * stm32_spi_transfer_one_setup - common setup to transfer a single
 *				  spi_transfer either using DMA or
 *				  interrupts.
 */
static int stm32_spi_transfer_one_setup(struct stm32_spi *spi,
					struct spi_device *spi_dev,
					struct spi_transfer *transfer)
{
	unsigned long flags;
	u32 cfg1_clrb = 0, cfg1_setb = 0, cfg2_clrb = 0, cfg2_setb = 0;
	u32 fthlv, mode, nb_words, bpw;
	int mbr, ret = 0;

	spin_lock_irqsave(&spi->lock, flags);

	spi->cur_bpw = transfer->bits_per_word;
	bpw = spi->cur_bpw - 1;

	cfg1_clrb |= SPI_CFG1_DSIZE;
	cfg1_setb |= FIELD_PREP(SPI_CFG1_DSIZE, bpw);

	spi->cur_fthlv = stm32_spi_prepare_fthlv(spi, transfer->len);
	fthlv = spi->cur_fthlv - 1;

	cfg1_clrb |= SPI_CFG1_FTHLV;
	cfg1_setb |= FIELD_PREP(SPI_CFG1_FTHLV, fthlv);

	if(!spi->slave_mode){
		/* Update spi->cur_speed with real clock speed */
		mbr = stm32_spi_prepare_mbr(spi, transfer->speed_hz);
		if (mbr < 0) {
			ret = mbr;
			goto out;
		}

		transfer->speed_hz = spi->cur_speed;

		cfg1_clrb |= SPI_CFG1_MBR;
		cfg1_setb |= FIELD_PREP(SPI_CFG1_MBR, (u32)mbr);
	} else {
		cfg1_clrb |= SPI_CFG1_UDRDET;
		cfg1_setb |= FIELD_PREP(SPI_CFG1_UDRDET, (u32)SPI_UDRDET_MODE);
	}

	writel_relaxed((readl_relaxed(spi->base + STM32_SPI_CFG1) &
			~cfg1_clrb) | cfg1_setb,
		       spi->base + STM32_SPI_CFG1);

	mode = SPI_FULL_DUPLEX;
	if (spi_dev->mode & SPI_3WIRE) { /* MISO/MOSI signals shared */
		/*
		 * SPI_3WIRE and xfer->tx_buf != NULL and xfer->rx_buf != NULL
		 * is forbidden und unvalidated by SPI subsystem so depending
		 * on the valid buffer, we can determine the direction of the
		 * transfer.
		 */
		mode = SPI_HALF_DUPLEX;
		if (!transfer->tx_buf)
			stm32_spi_clr_bits(spi, STM32_SPI_CR1, SPI_CR1_HDDIR);
		else if (!transfer->rx_buf)
			stm32_spi_set_bits(spi, STM32_SPI_CR1, SPI_CR1_HDDIR);
	} else {
		if (!transfer->tx_buf)
			mode = SPI_SIMPLEX_RX;
		else if (!transfer->rx_buf)
			mode = SPI_SIMPLEX_TX;
	}
	spi->cur_comm = mode;

	cfg2_clrb |= SPI_CFG2_COMM;
	cfg2_setb |= FIELD_PREP(SPI_CFG2_COMM, mode);

	cfg2_clrb |= SPI_CFG2_MIDI;
	if ((transfer->len > 1) && (spi->cur_midi > 0)) {
		u32 sck_period_ns = DIV_ROUND_UP(NSEC_PER_SEC, spi->cur_speed);
		u32 midi = min_t(u32,
				 DIV_ROUND_UP(spi->cur_midi, sck_period_ns),
				 FIELD_GET(SPI_CFG2_MIDI, SPI_CFG2_MIDI));

		dev_dbg(spi->dev, "period=%dns, midi=%d(=%dns)\n",
			sck_period_ns, midi, midi * sck_period_ns);

		cfg2_setb |= FIELD_PREP(SPI_CFG2_MIDI, midi);
	}

	writel_relaxed((readl_relaxed(spi->base + STM32_SPI_CFG2) &
			~cfg2_clrb) | cfg2_setb,
		       spi->base + STM32_SPI_CFG2);

	if (spi->cur_bpw <= 8)
		nb_words = transfer->len;
	else if (spi->cur_bpw <= 16)
		nb_words = DIV_ROUND_UP(transfer->len * 8, 16);
	else
		nb_words = DIV_ROUND_UP(transfer->len * 8, 32);

	if (nb_words <= SPI_TSIZE_MAX) {
		writel_relaxed(FIELD_PREP(SPI_CR2_TSIZE, nb_words),
			       spi->base + STM32_SPI_CR2);
	} else {
		ret = -EMSGSIZE;
		goto out;
	}

	spi->cur_xferlen = transfer->len;

	dev_dbg(spi->dev, "transfer communication mode set to %d\n",
		spi->cur_comm);
	dev_dbg(spi->dev,
		"data frame of %d-bit, data packet of %d data frames\n",
		spi->cur_bpw, spi->cur_fthlv);
	dev_dbg(spi->dev, "speed set to %dHz\n", spi->cur_speed);
	dev_dbg(spi->dev, "transfer of %d bytes (%d data frames)\n",
		spi->cur_xferlen, nb_words);
	dev_dbg(spi->dev, "dma %s\n",
		(spi->cur_usedma) ? "enabled" : "disabled");

out:
	spin_unlock_irqrestore(&spi->lock, flags);

	return ret;
}

/**
 * stm32_spi_transfer_one - transfer a single spi_transfer
 *
 * It must return 0 if the transfer is finished or 1 if the transfer is still
 * in progress.
 */
static int stm32_spi_transfer_one(struct spi_master *master,
				  struct spi_device *spi_dev,
				  struct spi_transfer *transfer)
{
	struct stm32_spi *spi = spi_master_get_devdata(master);
	u32 xfer_time, midi_delay_ns;
	unsigned long timeout;
	int ret;

	/* Don't do anything on 0 bytes transfers */
	if (transfer->len == 0) {
		spi->xfer_status = 0;
		goto finalize;
	}

	spi->tx_buf = transfer->tx_buf;
	spi->rx_buf = transfer->rx_buf;
	spi->tx_len = spi->tx_buf ? transfer->len : 0;
	spi->rx_len = spi->rx_buf ? transfer->len : 0;

	spi->cur_usedma = (master->can_dma &&
			   stm32_spi_can_dma(master, spi_dev, transfer));

	ret = stm32_spi_transfer_one_setup(spi, spi_dev, transfer);
	if (ret) {
		dev_err(spi->dev, "SPI transfer setup failed\n");
		return ret;
	}

	reinit_completion(&spi->xfer_completion);
	spi->xfer_status = 0;

	if (spi->cur_usedma)
		stm32_spi_transfer_one_dma(spi, transfer);
	else
		stm32_spi_transfer_one_irq(spi);

	if (!spi->slave_mode){
		/* Wait for transfer to complete */
		xfer_time = spi->cur_xferlen * 8 * MSEC_PER_SEC / spi->cur_speed;
		midi_delay_ns = spi->cur_xferlen * 8 / spi->cur_bpw * spi->cur_midi;
		xfer_time += DIV_ROUND_UP(midi_delay_ns, NSEC_PER_MSEC);
		xfer_time = max(2 * xfer_time, 100U);
		timeout = msecs_to_jiffies(xfer_time);

		timeout = wait_for_completion_timeout(&spi->xfer_completion, timeout);
		if (timeout && spi->cur_usedma)
			timeout = wait_for_completion_timeout(&spi->dma_completion,
								timeout);

		if (!timeout) {
			dev_err(spi->dev, "SPI transfer timeout (%u ms)\n", xfer_time);
			spi->xfer_status = -ETIMEDOUT;
		}
	} else {
		if (wait_for_completion_interruptible(&spi->xfer_completion)
			|| spi->slave_aborted) {
			dev_err(spi->dev, "xfer interrupted\n");
			spi->xfer_status = -EINTR;
		} else {
			if (spi->cur_usedma){
				if(wait_for_completion_interruptible(&spi->dma_completion)){
					dev_err(spi->dev, "dma interrupted\n");
					spi->xfer_status = -EINTR;
				}
			}
		}
	}
	stm32_spi_disable(spi);

finalize:
	spi_finalize_current_transfer(master);

	return spi->xfer_status;
}

/**
 * stm32_spi_unprepare_msg - relax the hardware
 *
 * Normally, if TSIZE has been configured, we should relax the hardware at the
 * reception of the EOT interrupt. But in case of error, EOT will not be
 * raised. So the subsystem unprepare_message call allows us to properly
 * complete the transfer from an hardware point of view.
 */
static int stm32_spi_unprepare_msg(struct spi_master *master,
				   struct spi_message *msg)
{
	struct stm32_spi *spi = spi_master_get_devdata(master);

	stm32_spi_disable(spi);

	return 0;
}

/**
 * stm32_spi_config - Configure SPI controller as SPI master
 */
static int stm32_spi_config(struct stm32_spi *spi)
{
	unsigned long flags;

	spin_lock_irqsave(&spi->lock, flags);

	/* Ensure I2SMOD bit is kept cleared */
	stm32_spi_clr_bits(spi, STM32_SPI_I2SCFGR, SPI_I2SCFGR_I2SMOD);

	if(spi->slave_mode){
		/*
		* - SS input value high
		*/
		stm32_spi_set_bits(spi, STM32_SPI_CR1, SPI_CR1_SSI);

		/*
		* - Set the slave mode (default Motorola mode)
		* - Consider 1 master/n slaves configuration and
		*   SS input value is determined by the NSS pin
		* - keep control of all associated GPIOs
		*/
		stm32_spi_set_bits(spi, STM32_SPI_CFG2,
							SPI_CFG2_AFCNTR);
	} else {
		/*
		* - SS input value high
		* - transmitter half duplex direction
		* - automatic communication suspend when RX-Fifo is full
		*/
		stm32_spi_set_bits(spi, STM32_SPI_CR1, SPI_CR1_SSI |
							SPI_CR1_HDDIR |
							SPI_CR1_MASRX);

		/*
		* - Set the master mode (default Motorola mode)
		* - Consider 1 master/n slaves configuration and
		*   SS input value is determined by the SSI bit
		* - keep control of all associated GPIOs
		*/
		stm32_spi_set_bits(spi, STM32_SPI_CFG2, SPI_CFG2_MASTER |
							SPI_CFG2_SSM | SPI_CFG2_SSOE |
							SPI_CFG2_AFCNTR);
	}
	spin_unlock_irqrestore(&spi->lock, flags);

	return 0;
}

static int stm32_spi_slave_abort(struct spi_master *master)
{
	struct stm32_spi *spi = spi_master_get_devdata(master);
	spi->slave_aborted = true;
	complete(&spi->xfer_completion);

	return 0;
}

static const struct of_device_id stm32_spi_of_match[] = {
	{ .compatible = "st,stm32h7-spi", },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_spi_of_match);

static int stm32_spi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct spi_master *master;
	struct stm32_spi *spi;
	struct resource *res;
	int i, ret, num_cs, cs_gpio;
	bool slave_mode;

	if (!np) {
		dev_err(&pdev->dev, "can't get the platform data\n");
		return -EINVAL;
	}

	slave_mode = of_property_read_bool(np, "spi-slave");
	dev_info(&pdev->dev, slave_mode ? "spi slave mode .\n" : "spi master mode.\n");
	if (slave_mode)
		master = spi_alloc_slave(&pdev->dev, sizeof(struct stm32_spi));
	else
		master = spi_alloc_master(&pdev->dev, sizeof(struct stm32_spi));
	if (!master) {
		dev_err(&pdev->dev, "spi master allocation failed\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, master);

	spi = spi_master_get_devdata(master);
	spi->dev = &pdev->dev;
	spi->master = master;
	spi->slave_mode = slave_mode;
	spin_lock_init(&spi->lock);
	init_completion(&spi->xfer_completion);
	init_completion(&spi->dma_completion);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	spi->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(spi->base)) {
		ret = PTR_ERR(spi->base);
		goto err_master_put;
	}
	spi->phys_addr = (dma_addr_t)res->start;

	spi->irq = platform_get_irq(pdev, 0);
	if (spi->irq <= 0) {
		ret = spi->irq;
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to get irq: %d\n", ret);
		goto err_master_put;
	}
	ret = devm_request_threaded_irq(&pdev->dev, spi->irq, NULL,
					stm32_spi_irq, IRQF_ONESHOT,
					pdev->name, master);
	if (ret) {
		dev_err(&pdev->dev, "irq%d request failed: %d\n", spi->irq,
			ret);
		goto err_master_put;
	}

	spi->clk = devm_clk_get(&pdev->dev, 0);
	if (IS_ERR(spi->clk)) {
		ret = PTR_ERR(spi->clk);
		dev_err(&pdev->dev, "clk get failed: %d\n", ret);
		goto err_master_put;
	}

	ret = clk_prepare_enable(spi->clk);
	if (ret) {
		dev_err(&pdev->dev, "clk enable failed: %d\n", ret);
		goto err_master_put;
	}
	spi->clk_rate = clk_get_rate(spi->clk);
	if (!spi->clk_rate) {
		dev_err(&pdev->dev, "clk rate = 0\n");
		ret = -EINVAL;
		goto err_clk_disable;
	}

	spi->rst = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (!IS_ERR(spi->rst)) {
		reset_control_assert(spi->rst);
		udelay(2);
		reset_control_deassert(spi->rst);
	}

	spi->fifo_size = stm32_spi_get_fifo_size(spi);

	ret = stm32_spi_config(spi);
	if (ret) {
		dev_err(&pdev->dev, "controller configuration failed: %d\n",
			ret);
		goto err_clk_disable;
	}

	master->dev.of_node = pdev->dev.of_node;
	master->auto_runtime_pm = true;
	master->bus_num = pdev->id;
	master->mode_bits = SPI_MODE_3 | SPI_CS_HIGH | SPI_LSB_FIRST |
			    SPI_3WIRE | SPI_LOOP;
	master->bits_per_word_mask = stm32_spi_get_bpw_mask(spi);
	master->max_speed_hz = spi->clk_rate / SPI_MBR_DIV_MIN;
	master->min_speed_hz = spi->clk_rate / SPI_MBR_DIV_MAX;
	master->setup = stm32_spi_setup;
	master->prepare_message = stm32_spi_prepare_msg;
	master->transfer_one = stm32_spi_transfer_one;
	master->unprepare_message = stm32_spi_unprepare_msg;
	master->slave_abort = stm32_spi_slave_abort;

	spi->dma_tx = dma_request_slave_channel(spi->dev, "tx");
	if (!spi->dma_tx)
		dev_warn(&pdev->dev, "failed to request tx dma channel\n");
	else
		master->dma_tx = spi->dma_tx;

	spi->dma_rx = dma_request_slave_channel(spi->dev, "rx");
	if (!spi->dma_rx)
		dev_warn(&pdev->dev, "failed to request rx dma channel\n");
	else
		master->dma_rx = spi->dma_rx;

	if (spi->dma_tx || spi->dma_rx)
		master->can_dma = stm32_spi_can_dma;

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	num_cs = of_gpio_named_count(pdev->dev.of_node, "cs-gpios");
	if(!spi->slave_mode){
		for (i = 0; i < num_cs; i++) {
			cs_gpio = of_get_named_gpio(pdev->dev.of_node, "cs-gpios", i);
			if (cs_gpio == -EPROBE_DEFER) {
				ret = -EPROBE_DEFER;
				goto err_dma_release;
			}

			if (gpio_is_valid(cs_gpio)) {
				ret = devm_gpio_request(&pdev->dev, cs_gpio,
							DRIVER_NAME);
				if (ret) {
					dev_err(&pdev->dev, "can't get CS gpio %i\n",
						cs_gpio);
					goto err_dma_release;
				}
			}
		}
	}

	ret = spi_register_master(master);
	if (ret) {
		dev_err(&pdev->dev, "spi master registration failed: %d\n",
			ret);
		goto err_dma_release;
	}

	dev_info(&pdev->dev, "driver initialized\n");

	return 0;

err_dma_release:
	if (spi->dma_tx)
		dma_release_channel(spi->dma_tx);
	if (spi->dma_rx)
		dma_release_channel(spi->dma_rx);

	pm_runtime_disable(&pdev->dev);
err_clk_disable:
	clk_disable_unprepare(spi->clk);
err_master_put:
	spi_master_put(master);

	return ret;
}

static int stm32_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct stm32_spi *spi = spi_master_get_devdata(master);

	pm_runtime_get_sync(&pdev->dev);

	spi_unregister_master(master);
	stm32_spi_disable(spi);

	if (master->dma_tx)
		dma_release_channel(master->dma_tx);
	if (master->dma_rx)
		dma_release_channel(master->dma_rx);

	clk_disable_unprepare(spi->clk);

	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);

	pinctrl_pm_select_sleep_state(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int stm32_spi_runtime_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct stm32_spi *spi = spi_master_get_devdata(master);

	clk_disable_unprepare(spi->clk);

	return pinctrl_pm_select_sleep_state(dev);
}

static int stm32_spi_runtime_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct stm32_spi *spi = spi_master_get_devdata(master);
	int ret;

	ret = pinctrl_pm_select_default_state(dev);
	if (ret)
		return ret;

	return clk_prepare_enable(spi->clk);
}
#endif

#ifdef CONFIG_PM_SLEEP
static int stm32_spi_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	int ret;

	ret = spi_master_suspend(master);
	if (ret)
		return ret;

	return pm_runtime_force_suspend(dev);
}

static int stm32_spi_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct stm32_spi *spi = spi_master_get_devdata(master);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;

	ret = spi_master_resume(master);
	if (ret) {
		clk_disable_unprepare(spi->clk);
		return ret;
	}

	ret = pm_runtime_get_sync(dev);
	if (ret) {
		dev_err(dev, "Unable to power device:%d\n", ret);
		return ret;
	}

	stm32_spi_config(spi);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;
}
#endif

static const struct dev_pm_ops stm32_spi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stm32_spi_suspend, stm32_spi_resume)
	SET_RUNTIME_PM_OPS(stm32_spi_runtime_suspend,
			   stm32_spi_runtime_resume, NULL)
};

static struct platform_driver stm32_spi_driver = {
	.probe = stm32_spi_probe,
	.remove = stm32_spi_remove,
	.driver = {
		.name = DRIVER_NAME,
		.pm = &stm32_spi_pm_ops,
		.of_match_table = stm32_spi_of_match,
	},
};

module_platform_driver(stm32_spi_driver);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_DESCRIPTION("STMicroelectronics STM32 SPI Controller driver");
MODULE_AUTHOR("Amelie Delaunay <amelie.delaunay@st.com>");
MODULE_LICENSE("GPL v2");
