// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for STM32 DMA controller
 *
 * Inspired by dma-jz4740.c and tegra20-apb-dma.c
 *
 * Copyright (C) M'boumba Cedric Madianga 2015
 * Author: M'boumba Cedric Madianga <cedric.madianga@gmail.com>
 *         Pierre-Yves Mordret <pierre-yves.mordret@st.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/init.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "virt-dma.h"

#define STM32_DMA_LISR			0x0000 /* DMA Low Int Status Reg */
#define STM32_DMA_HISR			0x0004 /* DMA High Int Status Reg */
#define STM32_DMA_LIFCR			0x0008 /* DMA Low Int Flag Clear Reg */
#define STM32_DMA_HIFCR			0x000c /* DMA High Int Flag Clear Reg */
#define STM32_DMA_TCI			BIT(5) /* Transfer Complete Interrupt */
#define STM32_DMA_HTI			BIT(4) /* Half Transfer Interrupt */
#define STM32_DMA_TEI			BIT(3) /* Transfer Error Interrupt */
#define STM32_DMA_DMEI			BIT(2) /* Direct Mode Error Interrupt */
#define STM32_DMA_FEI			BIT(0) /* FIFO Error Interrupt */
#define STM32_DMA_MASKI			(STM32_DMA_TCI \
					 | STM32_DMA_TEI \
					 | STM32_DMA_DMEI \
					 | STM32_DMA_FEI)

/* DMA Stream x Configuration Register */
#define STM32_DMA_SCR(x)		(0x0010 + 0x18 * (x)) /* x = 0..7 */
#define STM32_DMA_SCR_REQ(n)		((n & 0x7) << 25)
#define STM32_DMA_SCR_MBURST_MASK	GENMASK(24, 23)
#define STM32_DMA_SCR_MBURST(n)	        ((n & 0x3) << 23)
#define STM32_DMA_SCR_PBURST_MASK	GENMASK(22, 21)
#define STM32_DMA_SCR_PBURST(n)	        ((n & 0x3) << 21)
#define STM32_DMA_SCR_PL_MASK		GENMASK(17, 16)
#define STM32_DMA_SCR_PL(n)		((n & 0x3) << 16)
#define STM32_DMA_SCR_MSIZE_MASK	GENMASK(14, 13)
#define STM32_DMA_SCR_MSIZE(n)		((n & 0x3) << 13)
#define STM32_DMA_SCR_PSIZE_MASK	GENMASK(12, 11)
#define STM32_DMA_SCR_PSIZE(n)		((n & 0x3) << 11)
#define STM32_DMA_SCR_PSIZE_GET(n)	((n & STM32_DMA_SCR_PSIZE_MASK) >> 11)
#define STM32_DMA_SCR_DIR_MASK		GENMASK(7, 6)
#define STM32_DMA_SCR_DIR(n)		((n & 0x3) << 6)
#define STM32_DMA_SCR_TRBUFF		BIT(20) /* Bufferable transfer for USART/UART */
#define STM32_DMA_SCR_CT		BIT(19) /* Target in double buffer */
#define STM32_DMA_SCR_DBM		BIT(18) /* Double Buffer Mode */
#define STM32_DMA_SCR_PINCOS		BIT(15) /* Peripheral inc offset size */
#define STM32_DMA_SCR_MINC		BIT(10) /* Memory increment mode */
#define STM32_DMA_SCR_PINC		BIT(9) /* Peripheral increment mode */
#define STM32_DMA_SCR_CIRC		BIT(8) /* Circular mode */
#define STM32_DMA_SCR_PFCTRL		BIT(5) /* Peripheral Flow Controller */
#define STM32_DMA_SCR_TCIE		BIT(4) /* Transfer Complete Int Enable
						*/
#define STM32_DMA_SCR_TEIE		BIT(2) /* Transfer Error Int Enable */
#define STM32_DMA_SCR_DMEIE		BIT(1) /* Direct Mode Err Int Enable */
#define STM32_DMA_SCR_EN		BIT(0) /* Stream Enable */
#define STM32_DMA_SCR_CFG_MASK		(STM32_DMA_SCR_PINC \
					| STM32_DMA_SCR_MINC \
					| STM32_DMA_SCR_PINCOS \
					| STM32_DMA_SCR_PL_MASK)
#define STM32_DMA_SCR_IRQ_MASK		(STM32_DMA_SCR_TCIE \
					| STM32_DMA_SCR_TEIE \
					| STM32_DMA_SCR_DMEIE)

/* DMA Stream x number of data register */
#define STM32_DMA_SNDTR(x)		(0x0014 + 0x18 * (x))

/* DMA stream peripheral address register */
#define STM32_DMA_SPAR(x)		(0x0018 + 0x18 * (x))

/* DMA stream x memory 0 address register */
#define STM32_DMA_SM0AR(x)		(0x001c + 0x18 * (x))

/* DMA stream x memory 1 address register */
#define STM32_DMA_SM1AR(x)		(0x0020 + 0x18 * (x))

/* DMA stream x FIFO control register */
#define STM32_DMA_SFCR(x)		(0x0024 + 0x18 * (x))
#define STM32_DMA_SFCR_FTH_MASK		GENMASK(1, 0)
#define STM32_DMA_SFCR_FTH(n)		(n & STM32_DMA_SFCR_FTH_MASK)
#define STM32_DMA_SFCR_FEIE		BIT(7) /* FIFO error interrupt enable */
#define STM32_DMA_SFCR_DMDIS		BIT(2) /* Direct mode disable */
#define STM32_DMA_SFCR_MASK		(STM32_DMA_SFCR_FEIE \
					| STM32_DMA_SFCR_DMDIS)

/* DMA direction */
#define STM32_DMA_DEV_TO_MEM		0x00
#define	STM32_DMA_MEM_TO_DEV		0x01
#define	STM32_DMA_MEM_TO_MEM		0x02

/* DMA priority level */
#define STM32_DMA_PRIORITY_LOW		0x00
#define STM32_DMA_PRIORITY_MEDIUM	0x01
#define STM32_DMA_PRIORITY_HIGH		0x02
#define STM32_DMA_PRIORITY_VERY_HIGH	0x03

/* DMA FIFO threshold selection */
#define STM32_DMA_FIFO_THRESHOLD_1QUARTERFULL		0x00
#define STM32_DMA_FIFO_THRESHOLD_HALFFULL		0x01
#define STM32_DMA_FIFO_THRESHOLD_3QUARTERSFULL		0x02
#define STM32_DMA_FIFO_THRESHOLD_FULL			0x03
#define STM32_DMA_FIFO_THRESHOLD_NONE			0x04

#define STM32_DMA_MAX_DATA_ITEMS	0xffff
#define STM32_DMA_SRAM_GRANULARITY	PAGE_SIZE
/*
 * Valid transfer starts from @0 to @0xFFFE leading to unaligned scatter
 * gather at boundary. Thus it's safer to round down this value on FIFO
 * size (16 Bytes)
 */
#define STM32_DMA_ALIGNED_MAX_DATA_ITEMS	\
	ALIGN_DOWN(STM32_DMA_MAX_DATA_ITEMS, 16)
#define STM32_DMA_MAX_CHANNELS		0x08
#define STM32_DMA_MAX_REQUEST_ID	0x08
#define STM32_DMA_MAX_DATA_PARAM	0x03
#define STM32_DMA_FIFO_SIZE		16	/* FIFO is 16 bytes */
#define STM32_DMA_MIN_BURST		4
#define STM32_DMA_MAX_BURST		16

/* DMA Features */
#define STM32_DMA_THRESHOLD_FTR_MASK	GENMASK(1, 0)
#define STM32_DMA_THRESHOLD_FTR_GET(n)	((n) & STM32_DMA_THRESHOLD_FTR_MASK)
#define STM32_DMA_DIRECT_MODE_MASK	BIT(2)
#define STM32_DMA_DIRECT_MODE_GET(n)	(((n) & STM32_DMA_DIRECT_MODE_MASK) >> 2)
#define STM32_DMA_ALT_ACK_MODE_MASK	BIT(4)
#define STM32_DMA_ALT_ACK_MODE_GET(n)	(((n) & STM32_DMA_ALT_ACK_MODE_MASK) >> 4)
#define STM32_DMA_MDMA_CHAIN_FTR_MASK	BIT(31)
#define STM32_DMA_MDMA_CHAIN_FTR_GET(n)	(((n) & STM32_DMA_MDMA_CHAIN_FTR_MASK) >> 31)
#define STM32_DMA_MDMA_SRAM_SIZE_MASK	GENMASK(30, 29)
#define STM32_DMA_MDMA_SRAM_SIZE_GET(n)	(((n) & STM32_DMA_MDMA_SRAM_SIZE_MASK) >> 29)

enum stm32_dma_width {
	STM32_DMA_BYTE,
	STM32_DMA_HALF_WORD,
	STM32_DMA_WORD,
};

enum stm32_dma_burst_size {
	STM32_DMA_BURST_SINGLE,
	STM32_DMA_BURST_INCR4,
	STM32_DMA_BURST_INCR8,
	STM32_DMA_BURST_INCR16,
};

/**
 * struct stm32_dma_cfg - STM32 DMA custom configuration
 * @channel_id: channel ID
 * @request_line: DMA request
 * @stream_config: 32bit mask specifying the DMA channel configuration
 * @features: 32bit mask specifying the DMA Feature list
 */
struct stm32_dma_cfg {
	u32 channel_id;
	u32 request_line;
	u32 stream_config;
	u32 features;
};

struct stm32_dma_chan_reg {
	u32 dma_lisr;
	u32 dma_hisr;
	u32 dma_lifcr;
	u32 dma_hifcr;
	u32 dma_scr;
	u32 dma_sndtr;
	u32 dma_spar;
	u32 dma_sm0ar;
	u32 dma_sm1ar;
	u32 dma_sfcr;
};

struct stm32_dma_mdma_desc {
	struct sg_table sgt;
	struct dma_async_tx_descriptor *desc;
};

struct stm32_dma_mdma {
	struct dma_chan *chan;
	enum dma_transfer_direction dir;
	dma_addr_t sram_buf;
	u32 sram_period;
};

struct stm32_dma_sg_req {
	struct scatterlist stm32_sgl_req;
	struct stm32_dma_chan_reg chan_reg;
	struct stm32_dma_mdma_desc m_desc;
};

struct stm32_dma_desc {
	struct virt_dma_desc vdesc;
	bool cyclic;
	u32 num_sgs;
	dma_addr_t dma_buf;
	void *dma_buf_cpu;
	u32 dma_buf_size;
	struct stm32_dma_sg_req sg_req[];
};

struct stm32_dma_chan {
	struct virt_dma_chan vchan;
	bool config_init;
	bool busy;
	u32 id;
	u32 irq;
	struct stm32_dma_desc *desc;
	u32 next_sg;
	struct dma_slave_config	dma_sconfig;
	struct stm32_dma_chan_reg chan_reg;
	u32 threshold;
	u32 mem_burst;
	u32 mem_width;
	enum dma_status status;
	struct stm32_dma_mdma mchan;
	u32 use_mdma;
	u32 sram_size;
	u32 residue_after_drain;
	struct workqueue_struct *mdma_wq;
	struct work_struct mdma_work;
};

struct stm32_dma_device {
	struct dma_device ddev;
	void __iomem *base;
	struct clk *clk;
	bool mem2mem;
	struct stm32_dma_chan chan[STM32_DMA_MAX_CHANNELS];
	struct gen_pool *sram_pool;
};

static struct stm32_dma_device *stm32_dma_get_dev(struct stm32_dma_chan *chan)
{
	return container_of(chan->vchan.chan.device, struct stm32_dma_device,
			    ddev);
}

static struct stm32_dma_chan *to_stm32_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct stm32_dma_chan, vchan.chan);
}

static struct stm32_dma_desc *to_stm32_dma_desc(struct virt_dma_desc *vdesc)
{
	return container_of(vdesc, struct stm32_dma_desc, vdesc);
}

static struct device *chan2dev(struct stm32_dma_chan *chan)
{
	return &chan->vchan.chan.dev->device;
}

static u32 stm32_dma_read(struct stm32_dma_device *dmadev, u32 reg)
{
	return readl_relaxed(dmadev->base + reg);
}

static void stm32_dma_write(struct stm32_dma_device *dmadev, u32 reg, u32 val)
{
	writel_relaxed(val, dmadev->base + reg);
}

static int stm32_dma_get_width(struct stm32_dma_chan *chan,
			       enum dma_slave_buswidth width)
{
	switch (width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		return STM32_DMA_BYTE;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		return STM32_DMA_HALF_WORD;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		return STM32_DMA_WORD;
	default:
		dev_err(chan2dev(chan), "Dma bus width not supported\n");
		return -EINVAL;
	}
}

static enum dma_slave_buswidth stm32_dma_get_max_width(u32 buf_len,
						       u64 buf_addr,
						       u32 threshold)
{
	enum dma_slave_buswidth max_width;

	if (threshold == STM32_DMA_FIFO_THRESHOLD_FULL)
		max_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	else
		max_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

	while ((buf_len < max_width  || buf_len % max_width) &&
	       max_width > DMA_SLAVE_BUSWIDTH_1_BYTE)
		max_width = max_width >> 1;

	if (buf_addr & (max_width - 1))
		max_width = DMA_SLAVE_BUSWIDTH_1_BYTE;

	return max_width;
}

static bool stm32_dma_fifo_threshold_is_allowed(u32 burst, u32 threshold,
						enum dma_slave_buswidth width)
{
	u32 remaining;

	if (threshold == STM32_DMA_FIFO_THRESHOLD_NONE)
		return false;

	if (width != DMA_SLAVE_BUSWIDTH_UNDEFINED) {
		if (burst != 0) {
			/*
			 * If number of beats fit in several whole bursts
			 * this configuration is allowed.
			 */
			remaining = ((STM32_DMA_FIFO_SIZE / width) *
				     (threshold + 1) / 4) % burst;

			if (remaining == 0)
				return true;
		} else {
			return true;
		}
	}

	return false;
}

static bool stm32_dma_is_burst_possible(u32 buf_len, u32 threshold)
{
	/* If FIFO direct mode, burst is not possible */
	if (threshold == STM32_DMA_FIFO_THRESHOLD_NONE)
		return false;

	/*
	 * Buffer or period length has to be aligned on FIFO depth.
	 * Otherwise bytes may be stuck within FIFO at buffer or period
	 * length.
	 */
	return ((buf_len % ((threshold + 1) * 4)) == 0);
}

static u32 stm32_dma_get_best_burst(u32 buf_len, u32 max_burst, u32 threshold,
				    enum dma_slave_buswidth width)
{
	u32 best_burst = max_burst;

	if (best_burst == 1 || !stm32_dma_is_burst_possible(buf_len, threshold))
		return 0;

	while ((buf_len < best_burst * width && best_burst > 1) ||
	       !stm32_dma_fifo_threshold_is_allowed(best_burst, threshold,
						    width)) {
		if (best_burst > STM32_DMA_MIN_BURST)
			best_burst = best_burst >> 1;
		else
			best_burst = 0;
	}

	return best_burst;
}

static int stm32_dma_get_burst(struct stm32_dma_chan *chan, u32 maxburst)
{
	switch (maxburst) {
	case 0:
	case 1:
		return STM32_DMA_BURST_SINGLE;
	case 4:
		return STM32_DMA_BURST_INCR4;
	case 8:
		return STM32_DMA_BURST_INCR8;
	case 16:
		return STM32_DMA_BURST_INCR16;
	default:
		dev_err(chan2dev(chan), "Dma burst size not supported\n");
		return -EINVAL;
	}
}

static void stm32_dma_set_fifo_config(struct stm32_dma_chan *chan,
				      u32 src_burst, u32 dst_burst)
{
	chan->chan_reg.dma_sfcr &= ~STM32_DMA_SFCR_MASK;
	chan->chan_reg.dma_scr &= ~STM32_DMA_SCR_DMEIE;

	if (!src_burst && !dst_burst) {
		/* Using direct mode */
		chan->chan_reg.dma_scr |= STM32_DMA_SCR_DMEIE;
	} else {
		/* Using FIFO mode */
		chan->chan_reg.dma_sfcr |= STM32_DMA_SFCR_MASK;
	}
}

static void stm32_dma_slave_caps(struct dma_chan *c, struct dma_slave_caps *caps)
{
	struct stm32_dma_chan *chan = to_stm32_dma_chan(c);

	if (chan->use_mdma)
		caps->max_sg_burst = 0; /* unlimited */
	else
		caps->max_sg_burst = STM32_DMA_ALIGNED_MAX_DATA_ITEMS;
}

static int stm32_dma_slave_config(struct dma_chan *c,
				  struct dma_slave_config *config)
{
	struct stm32_dma_chan *chan = to_stm32_dma_chan(c);

	memcpy(&chan->dma_sconfig, config, sizeof(*config));

	chan->config_init = true;

	return 0;
}

static u32 stm32_dma_irq_status(struct stm32_dma_chan *chan)
{
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	u32 flags, dma_isr;

	/*
	 * Read "flags" from DMA_xISR register corresponding to the selected
	 * DMA channel at the correct bit offset inside that register.
	 *
	 * If (ch % 4) is 2 or 3, left shift the mask by 16 bits.
	 * If (ch % 4) is 1 or 3, additionally left shift the mask by 6 bits.
	 */

	if (chan->id & 4)
		dma_isr = stm32_dma_read(dmadev, STM32_DMA_HISR);
	else
		dma_isr = stm32_dma_read(dmadev, STM32_DMA_LISR);

	flags = dma_isr >> (((chan->id & 2) << 3) | ((chan->id & 1) * 6));

	return flags & STM32_DMA_MASKI;
}

static void stm32_dma_irq_clear(struct stm32_dma_chan *chan, u32 flags)
{
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	u32 dma_ifcr;

	/*
	 * Write "flags" to the DMA_xIFCR register corresponding to the selected
	 * DMA channel at the correct bit offset inside that register.
	 *
	 * If (ch % 4) is 2 or 3, left shift the mask by 16 bits.
	 * If (ch % 4) is 1 or 3, additionally left shift the mask by 6 bits.
	 */
	flags &= STM32_DMA_MASKI;
	dma_ifcr = flags << (((chan->id & 2) << 3) | ((chan->id & 1) * 6));

	if (chan->id & 4)
		stm32_dma_write(dmadev, STM32_DMA_HIFCR, dma_ifcr);
	else
		stm32_dma_write(dmadev, STM32_DMA_LIFCR, dma_ifcr);
}

static int stm32_dma_disable_chan(struct stm32_dma_chan *chan)
{
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	u32 dma_scr, id, reg;

	id = chan->id;
	reg = STM32_DMA_SCR(id);
	dma_scr = stm32_dma_read(dmadev, reg);

	if (dma_scr & STM32_DMA_SCR_EN) {
		dma_scr &= ~STM32_DMA_SCR_EN;
		stm32_dma_write(dmadev, reg, dma_scr);

		return readl_relaxed_poll_timeout_atomic(dmadev->base + reg,
					dma_scr, !(dma_scr & STM32_DMA_SCR_EN),
					10, 1000000);
	}

	return 0;
}

static void stm32_dma_stop(struct stm32_dma_chan *chan)
{
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	u32 dma_scr, dma_sfcr, status;
	int ret;

	/* Disable interrupts */
	dma_scr = stm32_dma_read(dmadev, STM32_DMA_SCR(chan->id));
	dma_scr &= ~STM32_DMA_SCR_IRQ_MASK;
	stm32_dma_write(dmadev, STM32_DMA_SCR(chan->id), dma_scr);
	dma_sfcr = stm32_dma_read(dmadev, STM32_DMA_SFCR(chan->id));
	dma_sfcr &= ~STM32_DMA_SFCR_FEIE;
	stm32_dma_write(dmadev, STM32_DMA_SFCR(chan->id), dma_sfcr);

	/* Disable DMA */
	ret = stm32_dma_disable_chan(chan);
	if (ret < 0)
		return;

	/* Clear interrupt status if it is there */
	status = stm32_dma_irq_status(chan);
	if (status) {
		dev_dbg(chan2dev(chan), "%s(): clearing interrupt: 0x%08x\n",
			__func__, status);
		stm32_dma_irq_clear(chan, status);
	}

	chan->busy = false;
	chan->status = DMA_COMPLETE;
}

static int stm32_dma_terminate_all(struct dma_chan *c)
{
	struct stm32_dma_chan *chan = to_stm32_dma_chan(c);
	struct stm32_dma_mdma *mchan = &chan->mchan;
	unsigned long flags;
	LIST_HEAD(head);

	if (chan->use_mdma) {
		spin_lock_irqsave_nested(&chan->vchan.lock, flags, SINGLE_DEPTH_NESTING);
		dmaengine_terminate_async(mchan->chan);
	} else {
		spin_lock_irqsave(&chan->vchan.lock, flags);
	}

	if (chan->desc) {
		dma_cookie_complete(&chan->desc->vdesc.tx);
		vchan_terminate_vdesc(&chan->desc->vdesc);
		if (chan->busy)
			stm32_dma_stop(chan);
		chan->desc = NULL;
	}

	vchan_get_all_descriptors(&chan->vchan, &head);
	spin_unlock_irqrestore(&chan->vchan.lock, flags);
	vchan_dma_desc_free_list(&chan->vchan, &head);

	return 0;
}

static u32 stm32_dma_get_remaining_bytes(struct stm32_dma_chan *chan)
{
	u32 dma_scr, width, ndtr;
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);

	dma_scr = stm32_dma_read(dmadev, STM32_DMA_SCR(chan->id));
	width = STM32_DMA_SCR_PSIZE_GET(dma_scr);
	ndtr = stm32_dma_read(dmadev, STM32_DMA_SNDTR(chan->id));

	return ndtr << width;
}

static int stm32_dma_mdma_drain(struct stm32_dma_chan *chan)
{
	struct stm32_dma_mdma *mchan = &chan->mchan;
	struct stm32_dma_sg_req *sg_req;
	struct dma_device *ddev = mchan->chan->device;
	struct dma_async_tx_descriptor *desc = NULL;
	enum dma_status status;
	dma_addr_t src_buf, dst_buf;
	u32 mdma_residue, mdma_wrote, dma_to_write, len;
	struct dma_tx_state state;
	int ret;
	unsigned long flags;

	flush_workqueue(chan->mdma_wq);

	/* DMA/MDMA chain: drain remaining data in SRAM */

	/* Get the residue on MDMA side */
	status = dmaengine_tx_status(mchan->chan, mchan->chan->cookie, &state);
	if (status == DMA_COMPLETE)
		return status;

	mdma_residue = state.residue;
	sg_req = &chan->desc->sg_req[chan->next_sg - 1];
	len = sg_dma_len(&sg_req->stm32_sgl_req);

	/*
	 * Total = mdma blocks * sram_period + rest (< sram_period)
	 * so mdma blocks * sram_period = len - mdma residue - rest
	 */
	mdma_wrote = len - mdma_residue - (len % mchan->sram_period);

	/* Remaining data stuck in SRAM */
	dma_to_write = mchan->sram_period - stm32_dma_get_remaining_bytes(chan);
	if (dma_to_write > 0) {
		spin_lock_irqsave_nested(&chan->vchan.lock, flags, SINGLE_DEPTH_NESTING);

		/* Terminate current MDMA to initiate a new one */
		dmaengine_terminate_async(mchan->chan);

		/* Stop DMA current operation */
		stm32_dma_disable_chan(chan);

		spin_unlock_irqrestore(&chan->vchan.lock, flags);

		/* Double buffer management */
		src_buf = mchan->sram_buf +
			  ((mdma_wrote / mchan->sram_period) & 0x1) * mchan->sram_period;
		dst_buf = sg_dma_address(&sg_req->stm32_sgl_req) + mdma_wrote;

		desc = ddev->device_prep_dma_memcpy(mchan->chan, dst_buf, src_buf, dma_to_write,
						    DMA_PREP_INTERRUPT);
		if (!desc)
			return -EINVAL;

		ret = dma_submit_error(dmaengine_submit(desc));
		if (ret < 0)
			return ret;

		status = dma_wait_for_async_tx(desc);
		if (status != DMA_COMPLETE) {
			dev_err(chan2dev(chan), "%s dma_wait_for_async_tx error\n", __func__);
			dmaengine_terminate_async(mchan->chan);
			return -EBUSY;
		}

		/* We need to store residue for tx_status() */
		chan->residue_after_drain = len - (mdma_wrote + dma_to_write);
	}

	return 0;
}

static void stm32_dma_synchronize(struct dma_chan *c)
{
	struct stm32_dma_chan *chan = to_stm32_dma_chan(c);
	struct stm32_dma_mdma *mchan = &chan->mchan;

	if (chan->desc && chan->use_mdma && mchan->dir == DMA_DEV_TO_MEM)
		if (stm32_dma_mdma_drain(chan))
			dev_err(chan2dev(chan), "%s: can't drain DMA\n", __func__);

	if (chan->use_mdma)
		dmaengine_synchronize(mchan->chan);

	vchan_synchronize(&chan->vchan);
}

static void stm32_dma_dump_reg(struct stm32_dma_chan *chan)
{
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	u32 scr = stm32_dma_read(dmadev, STM32_DMA_SCR(chan->id));
	u32 ndtr = stm32_dma_read(dmadev, STM32_DMA_SNDTR(chan->id));
	u32 spar = stm32_dma_read(dmadev, STM32_DMA_SPAR(chan->id));
	u32 sm0ar = stm32_dma_read(dmadev, STM32_DMA_SM0AR(chan->id));
	u32 sm1ar = stm32_dma_read(dmadev, STM32_DMA_SM1AR(chan->id));
	u32 sfcr = stm32_dma_read(dmadev, STM32_DMA_SFCR(chan->id));

	dev_dbg(chan2dev(chan), "SCR:   0x%08x\n", scr);
	dev_dbg(chan2dev(chan), "NDTR:  0x%08x\n", ndtr);
	dev_dbg(chan2dev(chan), "SPAR:  0x%08x\n", spar);
	dev_dbg(chan2dev(chan), "SM0AR: 0x%08x\n", sm0ar);
	dev_dbg(chan2dev(chan), "SM1AR: 0x%08x\n", sm1ar);
	dev_dbg(chan2dev(chan), "SFCR:  0x%08x\n", sfcr);
}

static int stm32_dma_dummy_memcpy_xfer(struct stm32_dma_chan *chan)
{
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	struct dma_device *ddev = &dmadev->ddev;
	struct stm32_dma_chan_reg reg;
	u8 src_buf, dst_buf;
	dma_addr_t dma_src_buf, dma_dst_buf;
	u32 ndtr, status;
	int len, ret;

	ret = 0;
	src_buf = 0;
	len = 1;

	dma_src_buf = dma_map_single(ddev->dev, &src_buf, len, DMA_TO_DEVICE);
	ret = dma_mapping_error(ddev->dev, dma_src_buf);
	if (ret < 0) {
		dev_err(chan2dev(chan), "Source buffer map failed\n");
		return ret;
	}

	dma_dst_buf = dma_map_single(ddev->dev, &dst_buf, len, DMA_FROM_DEVICE);
	ret = dma_mapping_error(ddev->dev, dma_dst_buf);
	if (ret < 0) {
		dev_err(chan2dev(chan), "Destination buffer map failed\n");
		dma_unmap_single(ddev->dev, dma_src_buf, len, DMA_TO_DEVICE);
		return ret;
	}

	reg.dma_scr =	STM32_DMA_SCR_DIR(STM32_DMA_MEM_TO_MEM) |
			STM32_DMA_SCR_PBURST(STM32_DMA_BURST_SINGLE) |
			STM32_DMA_SCR_MBURST(STM32_DMA_BURST_SINGLE) |
			STM32_DMA_SCR_MINC | STM32_DMA_SCR_PINC |
			STM32_DMA_SCR_TEIE;
	reg.dma_spar = dma_src_buf;
	reg.dma_sm0ar = dma_dst_buf;
	reg.dma_sfcr = STM32_DMA_SFCR_MASK | STM32_DMA_SFCR_FTH(STM32_DMA_FIFO_THRESHOLD_FULL);
	reg.dma_sm1ar = dma_dst_buf;
	reg.dma_sndtr = 1;

	stm32_dma_write(dmadev, STM32_DMA_SCR(chan->id), reg.dma_scr);
	stm32_dma_write(dmadev, STM32_DMA_SPAR(chan->id), reg.dma_spar);
	stm32_dma_write(dmadev, STM32_DMA_SM0AR(chan->id), reg.dma_sm0ar);
	stm32_dma_write(dmadev, STM32_DMA_SFCR(chan->id), reg.dma_sfcr);
	stm32_dma_write(dmadev, STM32_DMA_SM1AR(chan->id), reg.dma_sm1ar);
	stm32_dma_write(dmadev, STM32_DMA_SNDTR(chan->id), reg.dma_sndtr);

	/* Clear interrupt status if it is there */
	status = stm32_dma_irq_status(chan);
	if (status)
		stm32_dma_irq_clear(chan, status);

	stm32_dma_dump_reg(chan);

	chan->busy = true;
	chan->status = DMA_IN_PROGRESS;
	/* Start DMA */
	reg.dma_scr |= STM32_DMA_SCR_EN;
	stm32_dma_write(dmadev, STM32_DMA_SCR(chan->id), reg.dma_scr);

	ret = readl_relaxed_poll_timeout_atomic(dmadev->base + STM32_DMA_SNDTR(chan->id),
						ndtr, !ndtr, 10, 1000);
	if (ret) {
		dev_err(chan2dev(chan), "%s: timeout!\n", __func__);
		ret = -EBUSY;
	}

	chan->busy = false;
	chan->status = DMA_COMPLETE;

	ret = stm32_dma_disable_chan(chan);
	status = stm32_dma_irq_status(chan);
	if (status)
		stm32_dma_irq_clear(chan, status);

	dma_unmap_single(ddev->dev, dma_src_buf, len, DMA_TO_DEVICE);
	dma_unmap_single(ddev->dev, dma_dst_buf, len, DMA_FROM_DEVICE);

	return ret;
}

static int stm32_dma_mdma_flush_remaining(struct stm32_dma_chan *chan)
{
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	struct stm32_dma_mdma *mchan = &chan->mchan;
	struct stm32_dma_sg_req *sg_req;
	struct dma_device *ddev = mchan->chan->device;
	struct dma_async_tx_descriptor *desc = NULL;
	enum dma_status status;
	dma_addr_t src_buf, dst_buf;
	u32 residue, remain, len, dma_scr;
	int ret;

	residue = stm32_dma_get_remaining_bytes(chan);
	if (!residue)
		return 0;

	dma_scr = stm32_dma_read(dmadev, STM32_DMA_SCR(chan->id));
	if (!(dma_scr & STM32_DMA_SCR_EN))
		return -EPERM;

	sg_req = &chan->desc->sg_req[chan->next_sg - 1];
	len = sg_dma_len(&sg_req->stm32_sgl_req);
	remain = len % mchan->sram_period;

	if (len > mchan->sram_period && ((len % mchan->sram_period) != 0)) {
		unsigned long dma_sync_wait_timeout = jiffies + msecs_to_jiffies(5000);

		while (residue > 0 && residue > (mchan->sram_period - remain)) {
			if (time_after_eq(jiffies, dma_sync_wait_timeout)) {
				dev_err(chan2dev(chan),
					"%s timeout pending last %d bytes\n", __func__, residue);
				return -EBUSY;
			}
			cpu_relax();
			residue = stm32_dma_get_remaining_bytes(chan);
		}
		stm32_dma_disable_chan(chan);

		src_buf = mchan->sram_buf + ((len / mchan->sram_period) & 0x1) * mchan->sram_period;
		dst_buf = sg_dma_address(&sg_req->stm32_sgl_req) + len - (len % mchan->sram_period);

		desc = ddev->device_prep_dma_memcpy(mchan->chan, dst_buf, src_buf,
						    len % mchan->sram_period, DMA_PREP_INTERRUPT);

		if (!desc)
			return -EINVAL;

		ret = dma_submit_error(dmaengine_submit(desc));
		if (ret < 0)
			return ret;

		status = dma_wait_for_async_tx(desc);
		if (status != DMA_COMPLETE) {
			dmaengine_terminate_async(mchan->chan);
			return -EBUSY;
		}
	}

	return 0;
}

static void stm32_dma_start_transfer(struct stm32_dma_chan *chan);

static void stm32_mdma_chan_complete_worker(struct work_struct *work)
{
	struct stm32_dma_chan *chan = container_of(work, struct stm32_dma_chan, mdma_work);
	unsigned long flags;
	int ret;

	chan->busy = false;
	chan->status = DMA_COMPLETE;
	ret = stm32_dma_mdma_flush_remaining(chan);
	if (ret) {
		dev_err(chan2dev(chan), "Can't flush DMA: %d\n", ret);
		return;
	}

	spin_lock_irqsave_nested(&chan->vchan.lock, flags, SINGLE_DEPTH_NESTING);

	/*
	 * Added NULL check in stm32_mdma_chan_complete_worker to ensure that
	 * the chan->desc is not NULL. This is necessary because the desc can
	 * be deleted by an asynchronous call made to stm32_dma_terminate_all.
	 * In this scenario, when the stm32_mdma_chan_complete_worker is
	 * executed the chan->desc becomes NULL and the Kernel crashes.
	 */
	if (chan->desc && chan->next_sg == chan->desc->num_sgs) {
		vchan_cookie_complete(&chan->desc->vdesc);
		chan->desc = NULL;
	}

	/*
	 * It does not make much of a difference if we call this function in case
	 * chan->desc is NULL. The stm32_dma_start_transfer will handle the NULL
	 * for chan->desc and vdesc both.
	 */
	stm32_dma_start_transfer(chan);

	spin_unlock_irqrestore(&chan->vchan.lock, flags);
}

static void stm32_mdma_chan_complete(void *param, const struct dmaengine_result *result)
{
	struct stm32_dma_chan *chan = param;

	chan->busy = false;
	chan->status = DMA_COMPLETE;

	if (result->result != DMA_TRANS_NOERROR) {
		dev_err(chan2dev(chan), "MDMA transfer error: %d\n", result->result);
		return;
	}
	if (!queue_work(chan->mdma_wq, &chan->mdma_work))
		dev_warn(chan2dev(chan), "Work already queued\n");
}

static int stm32_dma_mdma_start(struct stm32_dma_chan *chan, struct stm32_dma_sg_req *sg_req)
{
	struct stm32_dma_mdma *mchan = &chan->mchan;
	struct stm32_dma_mdma_desc *m_desc = &sg_req->m_desc;
	int ret;

	ret = dma_submit_error(dmaengine_submit(m_desc->desc));
	if (ret < 0) {
		dev_err(chan2dev(chan), "MDMA submit failed\n");
		goto error;
	}

	dma_async_issue_pending(mchan->chan);

	/*
	 * In case of M2D transfer, we have to generate dummy DMA transfer to
	 * copy 1st sg data into SRAM
	 */
	if (mchan->dir == DMA_MEM_TO_DEV) {
		ret = stm32_dma_dummy_memcpy_xfer(chan);
		if (ret < 0) {
			dmaengine_terminate_async(mchan->chan);
			goto error;
		}
	}

	return 0;
error:
	return ret;
}

static void stm32_dma_sg_inc(struct stm32_dma_chan *chan)
{
	chan->next_sg++;
	if (chan->desc->cyclic && (chan->next_sg == chan->desc->num_sgs))
		chan->next_sg = 0;
}

static void stm32_dma_configure_next_sg(struct stm32_dma_chan *chan);

static void stm32_dma_start_transfer(struct stm32_dma_chan *chan)
{
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	struct virt_dma_desc *vdesc;
	struct stm32_dma_sg_req *sg_req;
	struct stm32_dma_chan_reg *reg;
	u32 status;
	int ret;

	ret = stm32_dma_disable_chan(chan);
	if (ret < 0)
		return;

	if (!chan->desc) {
		vdesc = vchan_next_desc(&chan->vchan);
		if (!vdesc)
			return;

		list_del(&vdesc->node);

		chan->desc = to_stm32_dma_desc(vdesc);
		chan->next_sg = 0;
	} else {
		vdesc = &chan->desc->vdesc;
	}

	if (chan->next_sg == chan->desc->num_sgs)
		chan->next_sg = 0;

	sg_req = &chan->desc->sg_req[chan->next_sg];
	reg = &sg_req->chan_reg;

	/* Clear interrupt status if it is there */
	status = stm32_dma_irq_status(chan);
	if (status)
		stm32_dma_irq_clear(chan, status);

	if (chan->use_mdma) {
		if (chan->next_sg == 0) {
			struct stm32_dma_mdma_desc *m_desc;

			m_desc = &sg_req->m_desc;
			if (chan->desc->cyclic) {
				/* If one callback is set, it will be called by MDMA driver. */
				if (vdesc->tx.callback) {
					m_desc->desc->callback = vdesc->tx.callback;
					m_desc->desc->callback_param = vdesc->tx.callback_param;
					vdesc->tx.callback = NULL;
					vdesc->tx.callback_param = NULL;
				}
			}
		}

		if (chan->mchan.dir == DMA_MEM_TO_DEV) {
			ret = stm32_dma_dummy_memcpy_xfer(chan);
			if (ret < 0) {
				dmaengine_terminate_async(chan->mchan.chan);
				chan->desc = NULL;
				return;
			}
		} else {
			reg->dma_scr &= ~STM32_DMA_SCR_TCIE;
		}

		if (!chan->desc->cyclic) {
			/*  MDMA already started */
			if (chan->mchan.dir != DMA_MEM_TO_DEV &&
			    sg_dma_len(&sg_req->stm32_sgl_req) > chan->mchan.sram_period)
				reg->dma_scr |= STM32_DMA_SCR_DBM;
			ret = stm32_dma_mdma_start(chan, sg_req);
			if (ret < 0) {
				chan->desc = NULL;
				return;
			}
		}
	}

	stm32_dma_sg_inc(chan);

	reg->dma_scr &= ~STM32_DMA_SCR_EN;
	stm32_dma_write(dmadev, STM32_DMA_SCR(chan->id), reg->dma_scr);
	stm32_dma_write(dmadev, STM32_DMA_SPAR(chan->id), reg->dma_spar);
	stm32_dma_write(dmadev, STM32_DMA_SM0AR(chan->id), reg->dma_sm0ar);
	stm32_dma_write(dmadev, STM32_DMA_SFCR(chan->id), reg->dma_sfcr);
	stm32_dma_write(dmadev, STM32_DMA_SM1AR(chan->id), reg->dma_sm1ar);
	stm32_dma_write(dmadev, STM32_DMA_SNDTR(chan->id), reg->dma_sndtr);

	if (chan->desc->cyclic)
		stm32_dma_configure_next_sg(chan);

	stm32_dma_dump_reg(chan);

	/* Start DMA */
	chan->busy = true;
	chan->status = DMA_IN_PROGRESS;
	reg->dma_scr |= STM32_DMA_SCR_EN;
	stm32_dma_write(dmadev, STM32_DMA_SCR(chan->id), reg->dma_scr);

	dev_dbg(chan2dev(chan), "vchan %pK: started\n", &chan->vchan);
}

static void stm32_dma_configure_next_sg(struct stm32_dma_chan *chan)
{
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	struct stm32_dma_sg_req *sg_req;
	u32 dma_scr, dma_sm0ar, dma_sm1ar, id;

	id = chan->id;
	dma_scr = stm32_dma_read(dmadev, STM32_DMA_SCR(id));

	sg_req = &chan->desc->sg_req[chan->next_sg];

	if (dma_scr & STM32_DMA_SCR_CT) {
		dma_sm0ar = sg_req->chan_reg.dma_sm0ar;
		stm32_dma_write(dmadev, STM32_DMA_SM0AR(id), dma_sm0ar);
		dev_dbg(chan2dev(chan), "CT=1 <=> SM0AR: 0x%08x\n",
			stm32_dma_read(dmadev, STM32_DMA_SM0AR(id)));
	} else {
		dma_sm1ar = sg_req->chan_reg.dma_sm1ar;
		stm32_dma_write(dmadev, STM32_DMA_SM1AR(id), dma_sm1ar);
		dev_dbg(chan2dev(chan), "CT=0 <=> SM1AR: 0x%08x\n",
			stm32_dma_read(dmadev, STM32_DMA_SM1AR(id)));
	}
}

static void stm32_dma_handle_chan_paused(struct stm32_dma_chan *chan)
{
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	u32 dma_scr;

	/*
	 * Read and store current remaining data items and peripheral/memory addresses to be
	 * updated on resume
	 */
	dma_scr = stm32_dma_read(dmadev, STM32_DMA_SCR(chan->id));
	/*
	 * Transfer can be paused while between a previous resume and reconfiguration on transfer
	 * complete. If transfer is cyclic and CIRC and DBM have been deactivated for resume, need
	 * to set it here in SCR backup to ensure a good reconfiguration on transfer complete.
	 */
	if (chan->desc && chan->desc->cyclic) {
		if (chan->desc->num_sgs == 1)
			dma_scr |= STM32_DMA_SCR_CIRC;
		else
			dma_scr |= STM32_DMA_SCR_DBM;
	}
	chan->chan_reg.dma_scr = dma_scr;

	/*
	 * Need to temporarily deactivate CIRC/DBM until next Transfer Complete interrupt, otherwise
	 * on resume NDTR autoreload value will be wrong (lower than the initial period length)
	 */
	if (chan->desc && chan->desc->cyclic) {
		dma_scr &= ~(STM32_DMA_SCR_DBM | STM32_DMA_SCR_CIRC);
		stm32_dma_write(dmadev, STM32_DMA_SCR(chan->id), dma_scr);
	}

	chan->chan_reg.dma_sndtr = stm32_dma_read(dmadev, STM32_DMA_SNDTR(chan->id));

	chan->status = DMA_PAUSED;

	dev_dbg(chan2dev(chan), "vchan %pK: paused\n", &chan->vchan);
}

static void stm32_dma_post_resume_reconfigure(struct stm32_dma_chan *chan)
{
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	struct stm32_dma_sg_req *sg_req;
	u32 dma_scr, status, id;

	id = chan->id;
	dma_scr = stm32_dma_read(dmadev, STM32_DMA_SCR(id));

	/* Clear interrupt status if it is there */
	status = stm32_dma_irq_status(chan);
	if (status)
		stm32_dma_irq_clear(chan, status);

	if (!chan->next_sg)
		sg_req = &chan->desc->sg_req[chan->desc->num_sgs - 1];
	else
		sg_req = &chan->desc->sg_req[chan->next_sg - 1];

	/* Reconfigure NDTR with the initial value */
	stm32_dma_write(dmadev, STM32_DMA_SNDTR(chan->id), sg_req->chan_reg.dma_sndtr);

	/* Restore SPAR */
	stm32_dma_write(dmadev, STM32_DMA_SPAR(id), sg_req->chan_reg.dma_spar);

	/* Restore SM0AR/SM1AR whatever DBM/CT as they may have been modified */
	stm32_dma_write(dmadev, STM32_DMA_SM0AR(id), sg_req->chan_reg.dma_sm0ar);
	stm32_dma_write(dmadev, STM32_DMA_SM1AR(id), sg_req->chan_reg.dma_sm1ar);

	/* Reactivate CIRC/DBM if needed */
	if (chan->chan_reg.dma_scr & STM32_DMA_SCR_DBM) {
		dma_scr |= STM32_DMA_SCR_DBM;
		/* Restore CT */
		if (chan->chan_reg.dma_scr & STM32_DMA_SCR_CT)
			dma_scr &= ~STM32_DMA_SCR_CT;
		else
			dma_scr |= STM32_DMA_SCR_CT;
	} else if (chan->chan_reg.dma_scr & STM32_DMA_SCR_CIRC) {
		dma_scr |= STM32_DMA_SCR_CIRC;
	}
	stm32_dma_write(dmadev, STM32_DMA_SCR(chan->id), dma_scr);

	stm32_dma_configure_next_sg(chan);

	stm32_dma_dump_reg(chan);

	dma_scr |= STM32_DMA_SCR_EN;
	stm32_dma_write(dmadev, STM32_DMA_SCR(chan->id), dma_scr);

	dev_dbg(chan2dev(chan), "vchan %pK: reconfigured after pause/resume\n", &chan->vchan);
}

static void stm32_dma_handle_chan_done(struct stm32_dma_chan *chan, u32 scr)
{
	if (!chan->desc)
		return;

	if (chan->desc->cyclic) {
		vchan_cyclic_callback(&chan->desc->vdesc);
		if (chan->use_mdma)
			return;
		stm32_dma_sg_inc(chan);
		/* cyclic while CIRC/DBM disable => post resume reconfiguration needed */
		if (!(scr & (STM32_DMA_SCR_CIRC | STM32_DMA_SCR_DBM)))
			stm32_dma_post_resume_reconfigure(chan);
		else if (scr & STM32_DMA_SCR_DBM)
			stm32_dma_configure_next_sg(chan);
	} else {
		if (chan->use_mdma && chan->mchan.dir != DMA_MEM_TO_DEV)
			return; /* wait for callback */
		chan->busy = false;
		chan->status = DMA_COMPLETE;
		if (chan->next_sg == chan->desc->num_sgs) {
			vchan_cookie_complete(&chan->desc->vdesc);
			chan->desc = NULL;
		}
		stm32_dma_start_transfer(chan);
	}
}

static irqreturn_t stm32_dma_chan_irq(int irq, void *devid)
{
	struct stm32_dma_chan *chan = devid;
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	u32 status, scr, sfcr;

	spin_lock(&chan->vchan.lock);

	status = stm32_dma_irq_status(chan);
	scr = stm32_dma_read(dmadev, STM32_DMA_SCR(chan->id));
	sfcr = stm32_dma_read(dmadev, STM32_DMA_SFCR(chan->id));

	if (status & STM32_DMA_FEI) {
		stm32_dma_irq_clear(chan, STM32_DMA_FEI);
		status &= ~STM32_DMA_FEI;
		if (sfcr & STM32_DMA_SFCR_FEIE) {
			if (!(scr & STM32_DMA_SCR_EN) &&
			    !(status & STM32_DMA_TCI))
				dev_err(chan2dev(chan), "FIFO Error\n");
			else
				dev_dbg(chan2dev(chan), "FIFO over/underrun\n");
		}
	}
	if (status & STM32_DMA_DMEI) {
		stm32_dma_irq_clear(chan, STM32_DMA_DMEI);
		status &= ~STM32_DMA_DMEI;
		if (sfcr & STM32_DMA_SCR_DMEIE)
			dev_dbg(chan2dev(chan), "Direct mode overrun\n");
	}

	if (status & STM32_DMA_TCI) {
		stm32_dma_irq_clear(chan, STM32_DMA_TCI);
		if (scr & STM32_DMA_SCR_TCIE) {
			if (chan->status != DMA_PAUSED)
				stm32_dma_handle_chan_done(chan, scr);
		}
		status &= ~STM32_DMA_TCI;
	}

	if (status & STM32_DMA_HTI) {
		stm32_dma_irq_clear(chan, STM32_DMA_HTI);
		status &= ~STM32_DMA_HTI;
	}

	if (status) {
		stm32_dma_irq_clear(chan, status);
		dev_err(chan2dev(chan), "DMA error: status=0x%08x\n", status);
		if (!(scr & STM32_DMA_SCR_EN))
			dev_err(chan2dev(chan), "chan disabled by HW\n");
	}

	spin_unlock(&chan->vchan.lock);

	return IRQ_HANDLED;
}

static void stm32_dma_issue_pending(struct dma_chan *c)
{
	struct stm32_dma_chan *chan = to_stm32_dma_chan(c);
	unsigned long flags;

	if (chan->use_mdma)
		spin_lock_irqsave_nested(&chan->vchan.lock, flags, SINGLE_DEPTH_NESTING);
	else
		spin_lock_irqsave(&chan->vchan.lock, flags);

	if (vchan_issue_pending(&chan->vchan) && !chan->desc && !chan->busy) {
		dev_dbg(chan2dev(chan), "vchan %pK: issued\n", &chan->vchan);
		stm32_dma_start_transfer(chan);
	}

	spin_unlock_irqrestore(&chan->vchan.lock, flags);
}

static int stm32_dma_pause(struct dma_chan *c)
{
	struct stm32_dma_chan *chan = to_stm32_dma_chan(c);
	unsigned long flags;
	int ret;

	if (chan->status != DMA_IN_PROGRESS || chan->use_mdma)
		return -EPERM;

	spin_lock_irqsave(&chan->vchan.lock, flags);

	ret = stm32_dma_disable_chan(chan);
	if (!ret)
		stm32_dma_handle_chan_paused(chan);

	spin_unlock_irqrestore(&chan->vchan.lock, flags);

	return ret;
}

static int stm32_dma_resume(struct dma_chan *c)
{
	struct stm32_dma_chan *chan = to_stm32_dma_chan(c);
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	struct stm32_dma_chan_reg chan_reg = chan->chan_reg;
	u32 id = chan->id, scr, ndtr, offset, spar, sm0ar, sm1ar;
	struct stm32_dma_sg_req *sg_req;
	unsigned long flags;

	if (chan->status != DMA_PAUSED || chan->use_mdma)
		return -EPERM;

	scr = stm32_dma_read(dmadev, STM32_DMA_SCR(id));
	if (WARN_ON(scr & STM32_DMA_SCR_EN))
		return -EPERM;

	spin_lock_irqsave(&chan->vchan.lock, flags);

	/* sg_reg[prev_sg] contains original ndtr, sm0ar and sm1ar before pausing the transfer */
	if (!chan->next_sg)
		sg_req = &chan->desc->sg_req[chan->desc->num_sgs - 1];
	else
		sg_req = &chan->desc->sg_req[chan->next_sg - 1];

	ndtr = sg_req->chan_reg.dma_sndtr;
	offset = (ndtr - chan_reg.dma_sndtr) << STM32_DMA_SCR_PSIZE_GET(chan_reg.dma_scr);
	spar = sg_req->chan_reg.dma_spar;
	sm0ar = sg_req->chan_reg.dma_sm0ar;
	sm1ar = sg_req->chan_reg.dma_sm1ar;

	/*
	 * The peripheral and/or memory addresses have to be updated in order to adjust the
	 * address pointers. Need to check increment.
	 */
	if (chan_reg.dma_scr & STM32_DMA_SCR_PINC)
		stm32_dma_write(dmadev, STM32_DMA_SPAR(id), spar + offset);
	else
		stm32_dma_write(dmadev, STM32_DMA_SPAR(id), spar);

	if (!(chan_reg.dma_scr & STM32_DMA_SCR_MINC))
		offset = 0;

	/*
	 * In case of DBM, the current target could be SM1AR.
	 * Need to temporarily deactivate CIRC/DBM to finish the current transfer, so
	 * SM0AR becomes the current target and must be updated with SM1AR + offset if CT=1.
	 */
	if ((chan_reg.dma_scr & STM32_DMA_SCR_DBM) && (chan_reg.dma_scr & STM32_DMA_SCR_CT))
		stm32_dma_write(dmadev, STM32_DMA_SM1AR(id), sm1ar + offset);
	else
		stm32_dma_write(dmadev, STM32_DMA_SM0AR(id), sm0ar + offset);

	/* NDTR must be restored otherwise internal HW counter won't be correctly reset */
	stm32_dma_write(dmadev, STM32_DMA_SNDTR(id), chan_reg.dma_sndtr);

	/*
	 * Need to temporarily deactivate CIRC/DBM until next Transfer Complete interrupt,
	 * otherwise NDTR autoreload value will be wrong (lower than the initial period length)
	 */
	if (chan_reg.dma_scr & (STM32_DMA_SCR_CIRC | STM32_DMA_SCR_DBM))
		chan_reg.dma_scr &= ~(STM32_DMA_SCR_CIRC | STM32_DMA_SCR_DBM);

	if (chan_reg.dma_scr & STM32_DMA_SCR_DBM)
		stm32_dma_configure_next_sg(chan);

	stm32_dma_dump_reg(chan);

	/* The stream may then be re-enabled to restart transfer from the point it was stopped */
	chan->status = DMA_IN_PROGRESS;
	chan_reg.dma_scr |= STM32_DMA_SCR_EN;
	stm32_dma_write(dmadev, STM32_DMA_SCR(id), chan_reg.dma_scr);

	spin_unlock_irqrestore(&chan->vchan.lock, flags);

	dev_dbg(chan2dev(chan), "vchan %pK: resumed\n", &chan->vchan);

	return 0;
}

static int stm32_dma_set_xfer_param(struct stm32_dma_chan *chan,
				    enum dma_transfer_direction direction,
				    enum dma_slave_buswidth *buswidth,
				    u32 buf_len, u64 buf_addr)
{
	enum dma_slave_buswidth src_addr_width, dst_addr_width;
	int src_bus_width, dst_bus_width;
	int src_burst_size, dst_burst_size;
	u32 src_maxburst, dst_maxburst, src_best_burst, dst_best_burst;
	u32 dma_scr, fifoth;

	src_addr_width = chan->dma_sconfig.src_addr_width;
	dst_addr_width = chan->dma_sconfig.dst_addr_width;
	src_maxburst = chan->dma_sconfig.src_maxburst;
	dst_maxburst = chan->dma_sconfig.dst_maxburst;
	fifoth = chan->threshold;

	switch (direction) {
	case DMA_MEM_TO_DEV:
		/* Set device data size */
		dst_bus_width = stm32_dma_get_width(chan, dst_addr_width);
		if (dst_bus_width < 0)
			return dst_bus_width;

		/* Set device burst size */
		dst_best_burst = stm32_dma_get_best_burst(buf_len,
							  dst_maxburst,
							  fifoth,
							  dst_addr_width);

		dst_burst_size = stm32_dma_get_burst(chan, dst_best_burst);
		if (dst_burst_size < 0)
			return dst_burst_size;

		/* Set memory data size */
		src_addr_width = stm32_dma_get_max_width(buf_len, buf_addr,
							 fifoth);
		chan->mem_width = src_addr_width;
		src_bus_width = stm32_dma_get_width(chan, src_addr_width);
		if (src_bus_width < 0)
			return src_bus_width;

		/*
		 * Set memory burst size - burst not possible if address is not aligned on
		 * the address boundary equal to the size of the transfer
		 */
		if (buf_addr & (buf_len - 1))
			src_maxburst = 1;
		else
			src_maxburst = STM32_DMA_MAX_BURST;
		src_best_burst = stm32_dma_get_best_burst(buf_len,
							  src_maxburst,
							  fifoth,
							  src_addr_width);
		src_burst_size = stm32_dma_get_burst(chan, src_best_burst);
		if (src_burst_size < 0)
			return src_burst_size;

		dma_scr = STM32_DMA_SCR_DIR(STM32_DMA_MEM_TO_DEV) |
			STM32_DMA_SCR_PSIZE(dst_bus_width) |
			STM32_DMA_SCR_MSIZE(src_bus_width) |
			STM32_DMA_SCR_PBURST(dst_burst_size) |
			STM32_DMA_SCR_MBURST(src_burst_size);

		/* Set FIFO threshold */
		chan->chan_reg.dma_sfcr &= ~STM32_DMA_SFCR_FTH_MASK;
		if (fifoth != STM32_DMA_FIFO_THRESHOLD_NONE)
			chan->chan_reg.dma_sfcr |= STM32_DMA_SFCR_FTH(fifoth);

		/* Set peripheral address */
		chan->chan_reg.dma_spar = chan->dma_sconfig.dst_addr;
		*buswidth = dst_addr_width;
		break;

	case DMA_DEV_TO_MEM:
		/* Set device data size */
		src_bus_width = stm32_dma_get_width(chan, src_addr_width);
		if (src_bus_width < 0)
			return src_bus_width;

		/* Set device burst size */
		src_best_burst = stm32_dma_get_best_burst(buf_len,
							  src_maxburst,
							  fifoth,
							  src_addr_width);
		chan->mem_burst = src_best_burst;
		src_burst_size = stm32_dma_get_burst(chan, src_best_burst);
		if (src_burst_size < 0)
			return src_burst_size;

		/* Set memory data size */
		dst_addr_width = stm32_dma_get_max_width(buf_len, buf_addr,
							 fifoth);
		chan->mem_width = dst_addr_width;
		dst_bus_width = stm32_dma_get_width(chan, dst_addr_width);
		if (dst_bus_width < 0)
			return dst_bus_width;

		/*
		 * Set memory burst size - burst not possible if address is not aligned on
		 * the address boundary equal to the size of the transfer
		 */
		if (buf_addr & (buf_len - 1))
			dst_maxburst = 1;
		else
			dst_maxburst = STM32_DMA_MAX_BURST;
		dst_best_burst = stm32_dma_get_best_burst(buf_len,
							  dst_maxburst,
							  fifoth,
							  dst_addr_width);
		chan->mem_burst = dst_best_burst;
		dst_burst_size = stm32_dma_get_burst(chan, dst_best_burst);
		if (dst_burst_size < 0)
			return dst_burst_size;

		dma_scr = STM32_DMA_SCR_DIR(STM32_DMA_DEV_TO_MEM) |
			STM32_DMA_SCR_PSIZE(src_bus_width) |
			STM32_DMA_SCR_MSIZE(dst_bus_width) |
			STM32_DMA_SCR_PBURST(src_burst_size) |
			STM32_DMA_SCR_MBURST(dst_burst_size);

		/* Set FIFO threshold */
		chan->chan_reg.dma_sfcr &= ~STM32_DMA_SFCR_FTH_MASK;
		if (fifoth != STM32_DMA_FIFO_THRESHOLD_NONE)
			chan->chan_reg.dma_sfcr |= STM32_DMA_SFCR_FTH(fifoth);

		/* Set peripheral address */
		chan->chan_reg.dma_spar = chan->dma_sconfig.src_addr;
		*buswidth = chan->dma_sconfig.src_addr_width;
		break;

	default:
		dev_err(chan2dev(chan), "Dma direction is not supported\n");
		return -EINVAL;
	}

	stm32_dma_set_fifo_config(chan, src_best_burst, dst_best_burst);

	/* Set DMA control register */
	chan->chan_reg.dma_scr &= ~(STM32_DMA_SCR_DIR_MASK |
			STM32_DMA_SCR_PSIZE_MASK | STM32_DMA_SCR_MSIZE_MASK |
			STM32_DMA_SCR_PBURST_MASK | STM32_DMA_SCR_MBURST_MASK);
	chan->chan_reg.dma_scr |= dma_scr;

	return 0;
}

static void stm32_dma_clear_reg(struct stm32_dma_chan_reg *regs)
{
	memset(regs, 0, sizeof(struct stm32_dma_chan_reg));
}

static int stm32_dma_mdma_prep_slave_sg(struct stm32_dma_chan *chan,
					struct scatterlist *sgl, u32 sg_len,
					struct stm32_dma_desc *desc, unsigned long flags)
{
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	struct stm32_dma_mdma *mchan = &chan->mchan;
	struct scatterlist *sg, *m_sg;
	dma_addr_t dma_buf;
	u32 len, num_sgs, sram_period;
	int i, j, ret;

	desc->dma_buf_cpu = gen_pool_dma_alloc(dmadev->sram_pool, chan->sram_size, &desc->dma_buf);
	if (!desc->dma_buf_cpu)
		return -ENOMEM;
	desc->dma_buf_size = chan->sram_size;

	sram_period = chan->sram_size / 2;

	for_each_sg(sgl, sg, sg_len, i) {
		struct stm32_dma_mdma_desc *m_desc = &desc->sg_req[i].m_desc;
		struct dma_slave_config config;

		len = sg_dma_len(sg);
		desc->sg_req[i].stm32_sgl_req = *sg;
		num_sgs = 1;

		if (mchan->dir == DMA_MEM_TO_DEV) {
			if (len > chan->sram_size) {
				dev_err(chan2dev(chan),
					"max buf size = %d bytes\n", chan->sram_size);
				ret = -EINVAL;
				goto free_alloc;
			}
		} else {
			/*
			 * Build new sg for MDMA transfer
			 * Scatter DMA Req into several SDRAM transfer
			 */
			if (len > sram_period)
				num_sgs = len / sram_period;
		}

		ret = sg_alloc_table(&m_desc->sgt, num_sgs, GFP_ATOMIC);
		if (ret) {
			dev_err(chan2dev(chan), "MDMA sg table alloc failed\n");
			ret = -ENOMEM;
			goto err;
		}

		dma_buf = sg_dma_address(sg);
		for_each_sg(m_desc->sgt.sgl, m_sg, num_sgs, j) {
			size_t bytes = min_t(size_t, len, sram_period);

			sg_dma_address(m_sg) = dma_buf;
			sg_dma_len(m_sg) = bytes;
			dma_buf += bytes;
			len -= bytes;
		}

		/* Configure MDMA channel */
		memset(&config, 0, sizeof(config));
		if (mchan->dir == DMA_MEM_TO_DEV)
			config.dst_addr = desc->dma_buf;
		else
			config.src_addr = desc->dma_buf;

		ret = dmaengine_slave_config(mchan->chan, &config);
		if (ret < 0)
			goto err;

		/* Prepare MDMA descriptor */
		m_desc->desc = dmaengine_prep_slave_sg(mchan->chan,
						       m_desc->sgt.sgl, m_desc->sgt.nents,
						       mchan->dir, DMA_PREP_INTERRUPT);

		if (!m_desc->desc) {
			ret = -EINVAL;
			goto err;
		}

		if (flags & DMA_CTRL_REUSE)
			dmaengine_desc_set_reuse(m_desc->desc);

		if (mchan->dir != DMA_MEM_TO_DEV) {
			m_desc->desc->callback_result = stm32_mdma_chan_complete;
			m_desc->desc->callback_param = chan;
			INIT_WORK(&chan->mdma_work, stm32_mdma_chan_complete_worker);
		}
	}

	chan->mchan.sram_buf = desc->dma_buf;
	chan->mchan.sram_period = sram_period;

	return 0;

err:
	for (j = 0; j < i; j++) {
		struct stm32_dma_mdma_desc *m_desc = &desc->sg_req[j].m_desc;

		m_desc->desc = NULL;
		sg_free_table(&desc->sg_req[j].m_desc.sgt);
	}
free_alloc:
	gen_pool_free(dmadev->sram_pool, (unsigned long)desc->dma_buf_cpu, desc->dma_buf_size);
	return ret;
}

static int stm32_dma_setup_sg_requests(struct stm32_dma_chan *chan,
				       struct scatterlist *sgl, unsigned int sg_len,
				       enum dma_transfer_direction direction,
				       struct stm32_dma_desc *desc)
{
	struct scatterlist *sg;
	u32 nb_data_items;
	int i, ret;
	enum dma_slave_buswidth buswidth;

	for_each_sg(sgl, sg, sg_len, i) {
		ret = stm32_dma_set_xfer_param(chan, direction, &buswidth, sg_dma_len(sg),
					       (u64)sg_dma_address(sg));
		if (ret < 0)
			return ret;

		nb_data_items = sg_dma_len(sg) / buswidth;
		if (nb_data_items > STM32_DMA_ALIGNED_MAX_DATA_ITEMS) {
			dev_err(chan2dev(chan), "nb items not supported\n");
			return -EINVAL;
		}

		stm32_dma_clear_reg(&desc->sg_req[i].chan_reg);
		desc->sg_req[i].chan_reg.dma_scr = chan->chan_reg.dma_scr;
		desc->sg_req[i].chan_reg.dma_sfcr = chan->chan_reg.dma_sfcr;
		desc->sg_req[i].chan_reg.dma_spar = chan->chan_reg.dma_spar;
		desc->sg_req[i].chan_reg.dma_sm0ar = sg_dma_address(sg);
		desc->sg_req[i].chan_reg.dma_sm1ar = sg_dma_address(sg);
		if (chan->use_mdma)
			desc->sg_req[i].chan_reg.dma_sm1ar += chan->mchan.sram_period;
		desc->sg_req[i].chan_reg.dma_sndtr = nb_data_items;
	}

	desc->num_sgs = sg_len;

	return 0;
}

static struct dma_async_tx_descriptor *stm32_dma_prep_slave_sg(
	struct dma_chan *c, struct scatterlist *sgl,
	u32 sg_len, enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	struct stm32_dma_chan *chan = to_stm32_dma_chan(c);
	struct stm32_dma_desc *desc;
	int i, ret;

	if (!chan->config_init) {
		dev_err(chan2dev(chan), "dma channel is not configured\n");
		return NULL;
	}

	if (sg_len < 1) {
		dev_err(chan2dev(chan), "Invalid segment length %d\n", sg_len);
		return NULL;
	}

	desc = kzalloc(struct_size(desc, sg_req, sg_len), GFP_NOWAIT);
	if (!desc)
		return NULL;

	/* Set peripheral flow controller */
	if (chan->dma_sconfig.device_fc)
		chan->chan_reg.dma_scr |= STM32_DMA_SCR_PFCTRL;
	else
		chan->chan_reg.dma_scr &= ~STM32_DMA_SCR_PFCTRL;

	if (chan->use_mdma) {
		struct sg_table new_sgt;
		struct scatterlist *s, *_sgl;

		chan->mchan.dir = direction;
		ret = stm32_dma_mdma_prep_slave_sg(chan, sgl, sg_len, desc, flags);
		if (ret < 0)
			return NULL;

		ret = sg_alloc_table(&new_sgt, sg_len, GFP_ATOMIC);
		if (ret)
			dev_err(chan2dev(chan), "DMA sg table alloc failed\n");

		for_each_sg(new_sgt.sgl, s, sg_len, i) {
			_sgl = sgl;
			sg_dma_len(s) = min(sg_dma_len(_sgl), chan->mchan.sram_period);
			s->dma_address = chan->mchan.sram_buf;
			_sgl = sg_next(_sgl);
		}

		ret = stm32_dma_setup_sg_requests(chan, new_sgt.sgl, sg_len, direction, desc);
		sg_free_table(&new_sgt);
		if (ret < 0)
			goto err;
	} else {
		/* Prepare a normal DMA transfer */
		ret = stm32_dma_setup_sg_requests(chan, sgl, sg_len, direction, desc);
		if (ret < 0)
			goto err;
	}

	desc->cyclic = false;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
err:
	if (chan->use_mdma) {
		struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);

		for (i = 0; i < sg_len; i++)
			sg_free_table(&desc->sg_req[i].m_desc.sgt);

		gen_pool_free(dmadev->sram_pool, (unsigned long)desc->dma_buf_cpu,
			      desc->dma_buf_size);
	}
	kfree(desc);
	return NULL;
}

static int stm32_dma_mdma_prep_dma_cyclic(struct stm32_dma_chan *chan,
					  dma_addr_t buf_addr, size_t buf_len,
					  size_t period_len, struct stm32_dma_desc *desc)
{
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	struct stm32_dma_mdma *mchan = &chan->mchan;
	struct stm32_dma_mdma_desc *m_desc = &desc->sg_req[0].m_desc;
	struct dma_slave_config config;
	int ret;

	chan->sram_size = ALIGN(period_len, STM32_DMA_SRAM_GRANULARITY);
	desc->dma_buf_cpu = gen_pool_dma_alloc(dmadev->sram_pool, 2 * chan->sram_size,
					       &desc->dma_buf);
	if (!desc->dma_buf_cpu)
		return -ENOMEM;
	desc->dma_buf_size = 2 * chan->sram_size;

	memset(&config, 0, sizeof(config));

	/* Configure MDMA channel */
	if (chan->mchan.dir == DMA_MEM_TO_DEV)
		config.dst_addr = desc->dma_buf;
	else
		config.src_addr = desc->dma_buf;
	ret = dmaengine_slave_config(mchan->chan, &config);
	if (ret < 0)
		goto err;

	/* Prepare MDMA descriptor */
	m_desc->desc = dmaengine_prep_dma_cyclic(mchan->chan, buf_addr, buf_len,
						 period_len, chan->mchan.dir,
						 DMA_PREP_INTERRUPT);

	if (!m_desc->desc) {
		ret = -EINVAL;
		goto err;
	}

	ret = dma_submit_error(dmaengine_submit(m_desc->desc));
	if (ret < 0) {
		dev_err(chan2dev(chan), "MDMA submit failed\n");
		goto err;
	}

	dma_async_issue_pending(mchan->chan);

	/*
	 * In case of M2D transfer, we have to generate dummy DMA transfer to
	 * copy 1 period of data into SRAM
	 */
	if (chan->mchan.dir == DMA_MEM_TO_DEV) {
		ret = stm32_dma_dummy_memcpy_xfer(chan);
		if (ret < 0) {
			dev_err(chan2dev(chan), "stm32_dma_dummy_memcpy_xfer failed\n");
			dmaengine_terminate_async(mchan->chan);
			goto err;
		}
	}

	return 0;
err:
	gen_pool_free(dmadev->sram_pool, (unsigned long)desc->dma_buf_cpu, desc->dma_buf_size);
	return ret;
}

static struct dma_async_tx_descriptor *stm32_dma_prep_dma_cyclic(
	struct dma_chan *c, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction direction,
	unsigned long flags)
{
	struct stm32_dma_chan *chan = to_stm32_dma_chan(c);
	struct stm32_dma_chan_reg *chan_reg = &chan->chan_reg;
	struct stm32_dma_desc *desc;
	enum dma_slave_buswidth buswidth;
	u32 num_periods, nb_data_items;
	dma_addr_t dma_buf = 0;
	int i, ret;

	if (!buf_len || !period_len) {
		dev_err(chan2dev(chan), "Invalid buffer/period len\n");
		return NULL;
	}

	if (!chan->config_init) {
		dev_err(chan2dev(chan), "dma channel is not configured\n");
		return NULL;
	}

	if (buf_len % period_len) {
		dev_err(chan2dev(chan), "buf_len not multiple of period_len\n");
		return NULL;
	}

	/*
	 * We allow to take more number of requests till DMA is
	 * not started. The driver will loop over all requests.
	 * Once DMA is started then new requests can be queued only after
	 * terminating the DMA.
	 */
	if (chan->busy) {
		dev_err(chan2dev(chan), "Request not allowed when dma busy\n");
		return NULL;
	}

	ret = stm32_dma_set_xfer_param(chan, direction, &buswidth, period_len, (u64)buf_addr);
	if (ret < 0)
		return NULL;

	nb_data_items = period_len / buswidth;
	if (nb_data_items > STM32_DMA_ALIGNED_MAX_DATA_ITEMS) {
		dev_err(chan2dev(chan), "number of items not supported\n");
		return NULL;
	}

	/*  Enable Circular mode or double buffer mode */
	if (buf_len == period_len) {
		chan->chan_reg.dma_scr |= STM32_DMA_SCR_CIRC;
	} else {
		chan->chan_reg.dma_scr |= STM32_DMA_SCR_DBM;
		chan->chan_reg.dma_scr &= ~STM32_DMA_SCR_CT;
	}

	/* Clear periph ctrl if client set it */
	chan->chan_reg.dma_scr &= ~STM32_DMA_SCR_PFCTRL;

	if (chan->use_mdma)
		num_periods = 1;
	else
		num_periods = buf_len / period_len;

	desc = kzalloc(struct_size(desc, sg_req, num_periods), GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->num_sgs = num_periods;
	desc->cyclic = true;

	if (chan->use_mdma) {
		chan->mchan.dir = direction;

		ret = stm32_dma_mdma_prep_dma_cyclic(chan, buf_addr, buf_len, period_len, desc);
		if (ret < 0)
			return NULL;
		dma_buf = desc->dma_buf;
	} else {
		dma_buf = buf_addr;
	}

	for (i = 0; i < num_periods; i++) {
		sg_dma_len(&desc->sg_req[i].stm32_sgl_req) = period_len;
		sg_dma_address(&desc->sg_req[i].stm32_sgl_req) = dma_buf;
		stm32_dma_clear_reg(&desc->sg_req[i].chan_reg);
		desc->sg_req[i].chan_reg.dma_scr = chan_reg->dma_scr;
		desc->sg_req[i].chan_reg.dma_sfcr = chan_reg->dma_sfcr;
		desc->sg_req[i].chan_reg.dma_spar = chan_reg->dma_spar;
		if (chan->use_mdma) {
			desc->sg_req[i].chan_reg.dma_sm0ar = desc->dma_buf;
			desc->sg_req[i].chan_reg.dma_sm1ar = desc->dma_buf + chan->sram_size;
		} else {
			desc->sg_req[i].chan_reg.dma_sm0ar = dma_buf;
			desc->sg_req[i].chan_reg.dma_sm1ar = dma_buf;
			dma_buf += period_len;
		}
		desc->sg_req[i].chan_reg.dma_sndtr = nb_data_items;
	}

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static struct dma_async_tx_descriptor *stm32_dma_prep_dma_memcpy(
	struct dma_chan *c, dma_addr_t dest,
	dma_addr_t src, size_t len, unsigned long flags)
{
	struct stm32_dma_chan *chan = to_stm32_dma_chan(c);
	enum dma_slave_buswidth max_width;
	struct stm32_dma_desc *desc;
	size_t xfer_count, offset;
	u32 num_sgs, best_burst, dma_burst, threshold;
	int i;

	num_sgs = DIV_ROUND_UP(len, STM32_DMA_ALIGNED_MAX_DATA_ITEMS);
	desc = kzalloc(struct_size(desc, sg_req, num_sgs), GFP_NOWAIT);
	if (!desc)
		return NULL;

	threshold = chan->threshold;

	for (offset = 0, i = 0; offset < len; offset += xfer_count, i++) {
		xfer_count = min_t(size_t, len - offset,
				   STM32_DMA_ALIGNED_MAX_DATA_ITEMS);

		/* Compute best burst size */
		max_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		best_burst = stm32_dma_get_best_burst(len, STM32_DMA_MAX_BURST,
						      threshold, max_width);
		dma_burst = stm32_dma_get_burst(chan, best_burst);

		stm32_dma_clear_reg(&desc->sg_req[i].chan_reg);
		desc->sg_req[i].chan_reg.dma_scr =
			STM32_DMA_SCR_DIR(STM32_DMA_MEM_TO_MEM) |
			STM32_DMA_SCR_PBURST(dma_burst) |
			STM32_DMA_SCR_MBURST(dma_burst) |
			STM32_DMA_SCR_MINC |
			STM32_DMA_SCR_PINC |
			STM32_DMA_SCR_TCIE |
			STM32_DMA_SCR_TEIE;
		desc->sg_req[i].chan_reg.dma_sfcr &= ~STM32_DMA_SFCR_MASK;
		desc->sg_req[i].chan_reg.dma_sfcr |=
			STM32_DMA_SFCR_FTH(threshold);
		desc->sg_req[i].chan_reg.dma_spar = src + offset;
		desc->sg_req[i].chan_reg.dma_sm0ar = dest + offset;
		desc->sg_req[i].chan_reg.dma_sndtr = xfer_count;
		sg_dma_len(&desc->sg_req[i].stm32_sgl_req) = xfer_count;
	}

	desc->num_sgs = num_sgs;
	desc->cyclic = false;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

/**
 * stm32_dma_is_current_sg - check that expected sg_req is currently transferred
 * @chan: dma channel
 *
 * This function called when IRQ are disable, checks that the hardware has not
 * switched on the next transfer in double buffer mode. The test is done by
 * comparing the next_sg memory address with the hardware related register
 * (based on CT bit value).
 *
 * Returns true if expected current transfer is still running or double
 * buffer mode is not activated.
 */
static bool stm32_dma_is_current_sg(struct stm32_dma_chan *chan)
{
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	struct stm32_dma_sg_req *sg_req;
	u32 dma_scr, dma_smar, id, period_len;

	id = chan->id;
	dma_scr = stm32_dma_read(dmadev, STM32_DMA_SCR(id));

	/* In cyclic CIRC but not DBM, CT is not used */
	if (!(dma_scr & STM32_DMA_SCR_DBM))
		return true;

	sg_req = &chan->desc->sg_req[chan->next_sg];
	period_len = sg_dma_len(&sg_req->stm32_sgl_req);

	/* DBM - take care of a previous pause/resume not yet post reconfigured */
	if (dma_scr & STM32_DMA_SCR_CT) {
		dma_smar = stm32_dma_read(dmadev, STM32_DMA_SM0AR(id));
		/*
		 * If transfer has been pause/resumed,
		 * SM0AR is in the range of [SM0AR:SM0AR+period_len]
		 */
		return (dma_smar >= sg_req->chan_reg.dma_sm0ar &&
			dma_smar < sg_req->chan_reg.dma_sm0ar + period_len);
	}

	dma_smar = stm32_dma_read(dmadev, STM32_DMA_SM1AR(id));
	/*
	 * If transfer has been pause/resumed,
	 * SM1AR is in the range of [SM1AR:SM1AR+period_len]
	 */
	return (dma_smar >= sg_req->chan_reg.dma_sm1ar &&
		dma_smar < sg_req->chan_reg.dma_sm1ar + period_len);
}

static size_t stm32_dma_desc_residue(struct stm32_dma_chan *chan,
				     struct stm32_dma_desc *desc,
				     u32 next_sg)
{
	u32 modulo, burst_size;
	u32 residue;
	u32 n_sg = next_sg;
	struct stm32_dma_sg_req *sg_req = &chan->desc->sg_req[chan->next_sg];
	int i;

	/* Drain case */
	if (chan->residue_after_drain)
		return chan->residue_after_drain;

	/*
	 * Calculate the residue means compute the descriptors
	 * information:
	 * - the sg_req currently transferred
	 * - the Hardware remaining position in this sg (NDTR bits field).
	 *
	 * A race condition may occur if DMA is running in cyclic or double
	 * buffer mode, since the DMA register are automatically reloaded at end
	 * of period transfer. The hardware may have switched to the next
	 * transfer (CT bit updated) just before the position (SxNDTR reg) is
	 * read.
	 * In this case the SxNDTR reg could (or not) correspond to the new
	 * transfer position, and not the expected one.
	 * The strategy implemented in the stm32 driver is to:
	 *  - read the SxNDTR register
	 *  - crosscheck that hardware is still in current transfer.
	 * In case of switch, we can assume that the DMA is at the beginning of
	 * the next transfer. So we approximate the residue in consequence, by
	 * pointing on the beginning of next transfer.
	 *
	 * This race condition doesn't apply for none cyclic mode, as double
	 * buffer is not used. In such situation registers are updated by the
	 * software.
	 */

	residue = stm32_dma_get_remaining_bytes(chan);

	if (chan->desc->cyclic && !stm32_dma_is_current_sg(chan)) {
		n_sg++;
		if (n_sg == chan->desc->num_sgs)
			n_sg = 0;
		residue = sg_dma_len(&sg_req->stm32_sgl_req);
	}

	/*
	 * In cyclic mode, for the last period, residue = remaining bytes
	 * from NDTR,
	 * else for all other periods in cyclic mode, and in sg mode,
	 * residue = remaining bytes from NDTR + remaining
	 * periods/sg to be transferred
	 */
	if (!chan->desc->cyclic || n_sg != 0)
		for (i = n_sg; i < desc->num_sgs; i++)
			residue += sg_dma_len(&desc->sg_req[i].stm32_sgl_req);

	if (!chan->mem_burst)
		return residue;

	burst_size = chan->mem_burst * chan->mem_width;
	modulo = residue % burst_size;
	if (modulo)
		residue = residue - modulo + burst_size;

	return residue;
}

static enum dma_status stm32_dma_tx_status(struct dma_chan *c,
					   dma_cookie_t cookie,
					   struct dma_tx_state *state)
{
	struct stm32_dma_chan *chan = to_stm32_dma_chan(c);
	struct stm32_dma_mdma *mchan = &chan->mchan;
	struct virt_dma_desc *vdesc;
	enum dma_status status;
	unsigned long flags;
	u32 residue = 0;

	/*
	 * When DMA/MDMA chain is used, we return the status of MDMA in cyclic
	 * mode and for D2M transfer in sg mode in order to return the correct
	 * residue if any
	 */
	if (chan->desc && chan->use_mdma &&
	    (mchan->dir != DMA_MEM_TO_DEV || chan->desc->cyclic) &&
	    !chan->residue_after_drain)
		return dmaengine_tx_status(mchan->chan, mchan->chan->cookie, state);

	status = dma_cookie_status(c, cookie, state);
	if (status == DMA_COMPLETE)
		return status;

	status = chan->status;

	if (!state)
		return status;

	spin_lock_irqsave(&chan->vchan.lock, flags);
	vdesc = vchan_find_desc(&chan->vchan, cookie);
	if (chan->desc && cookie == chan->desc->vdesc.tx.cookie)
		residue = stm32_dma_desc_residue(chan, chan->desc,
						 chan->next_sg);
	else if (vdesc)
		residue = stm32_dma_desc_residue(chan,
						 to_stm32_dma_desc(vdesc), 0);
	dma_set_residue(state, residue);

	spin_unlock_irqrestore(&chan->vchan.lock, flags);

	return status;
}

static int stm32_dma_alloc_chan_resources(struct dma_chan *c)
{
	struct stm32_dma_chan *chan = to_stm32_dma_chan(c);
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	int ret;

	chan->config_init = false;

	ret = pm_runtime_resume_and_get(dmadev->ddev.dev);
	if (ret < 0)
		return ret;

	ret = stm32_dma_disable_chan(chan);
	if (ret < 0)
		pm_runtime_put(dmadev->ddev.dev);

	return ret;
}

static void stm32_dma_free_chan_resources(struct dma_chan *c)
{
	struct stm32_dma_chan *chan = to_stm32_dma_chan(c);
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	unsigned long flags;

	dev_dbg(chan2dev(chan), "Freeing channel %d\n", chan->id);

	if (chan->busy) {
		spin_lock_irqsave(&chan->vchan.lock, flags);
		stm32_dma_stop(chan);
		chan->desc = NULL;
		spin_unlock_irqrestore(&chan->vchan.lock, flags);
	}

	pm_runtime_put(dmadev->ddev.dev);

	vchan_free_chan_resources(to_virt_chan(c));
	stm32_dma_clear_reg(&chan->chan_reg);
	chan->threshold = 0;
	chan->use_mdma = false;
	chan->sram_size = 0;
}

static void stm32_dma_desc_free(struct virt_dma_desc *vdesc)
{
	struct stm32_dma_desc *desc = to_stm32_dma_desc(vdesc);
	struct stm32_dma_chan *chan = to_stm32_dma_chan(vdesc->tx.chan);
	struct stm32_dma_device *dmadev = stm32_dma_get_dev(chan);
	int i;

	if (chan->use_mdma) {
		struct stm32_dma_mdma_desc *m_desc;

		for (i = 0; i < desc->num_sgs; i++) {
			m_desc = &desc->sg_req[i].m_desc;
			if (dmaengine_desc_test_reuse(&vdesc->tx))
				dmaengine_desc_free(m_desc->desc);
			m_desc->desc = NULL;
			sg_free_table(&m_desc->sgt);
		}

		gen_pool_free(dmadev->sram_pool, (unsigned long)desc->dma_buf_cpu,
			      desc->dma_buf_size);
	}

	kfree(desc);
}

static void stm32_dma_set_config(struct stm32_dma_chan *chan,
				 struct stm32_dma_cfg *cfg)
{
	stm32_dma_clear_reg(&chan->chan_reg);
	chan->chan_reg.dma_scr = cfg->stream_config & STM32_DMA_SCR_CFG_MASK;
	chan->chan_reg.dma_scr |= STM32_DMA_SCR_REQ(cfg->request_line);
	chan->chan_reg.dma_scr |= STM32_DMA_SCR_TEIE | STM32_DMA_SCR_TCIE;
	chan->threshold = STM32_DMA_THRESHOLD_FTR_GET(cfg->features);
	if (STM32_DMA_DIRECT_MODE_GET(cfg->features))
		chan->threshold = STM32_DMA_FIFO_THRESHOLD_NONE;
	if (STM32_DMA_ALT_ACK_MODE_GET(cfg->features))
		chan->chan_reg.dma_scr |= STM32_DMA_SCR_TRBUFF;
	chan->use_mdma = STM32_DMA_MDMA_CHAIN_FTR_GET(cfg->features);
	chan->sram_size = (1 << STM32_DMA_MDMA_SRAM_SIZE_GET(cfg->features)) *
			  STM32_DMA_SRAM_GRANULARITY;
}

static struct dma_chan *stm32_dma_of_xlate(struct of_phandle_args *dma_spec,
					   struct of_dma *ofdma)
{
	struct stm32_dma_device *dmadev = ofdma->of_dma_data;
	struct device *dev = dmadev->ddev.dev;
	struct stm32_dma_cfg cfg;
	struct stm32_dma_chan *chan;
	struct dma_chan *c;

	if (dma_spec->args_count < 4) {
		dev_err(dev, "Bad number of cells\n");
		return NULL;
	}

	cfg.channel_id = dma_spec->args[0];
	cfg.request_line = dma_spec->args[1];
	cfg.stream_config = dma_spec->args[2];
	cfg.features = dma_spec->args[3];

	if (cfg.channel_id >= STM32_DMA_MAX_CHANNELS ||
	    cfg.request_line >= STM32_DMA_MAX_REQUEST_ID) {
		dev_err(dev, "Bad channel and/or request id\n");
		return NULL;
	}

	chan = &dmadev->chan[cfg.channel_id];

	c = dma_get_slave_channel(&chan->vchan.chan);
	if (!c) {
		dev_err(dev, "No more channels available\n");
		return NULL;
	}

	stm32_dma_set_config(chan, &cfg);

	if (!dmadev->sram_pool || !chan->mchan.chan)
		chan->use_mdma = 0;

	return c;
}

static const struct of_device_id stm32_dma_of_match[] = {
	{ .compatible = "st,stm32-dma", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, stm32_dma_of_match);

static int stm32_dma_probe(struct platform_device *pdev)
{
	struct stm32_dma_chan *chan;
	struct stm32_dma_mdma *mchan;
	struct stm32_dma_device *dmadev;
	struct dma_device *dd;
	const struct of_device_id *match;
	struct resource *res;
	struct reset_control *rst;
	char dma_wq_name[12];
	char name[4];
	int i, ret;

	match = of_match_device(stm32_dma_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		return -ENODEV;
	}

	dmadev = devm_kzalloc(&pdev->dev, sizeof(*dmadev), GFP_KERNEL);
	if (!dmadev)
		return -ENOMEM;

	dd = &dmadev->ddev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dmadev->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dmadev->base))
		return PTR_ERR(dmadev->base);

	dmadev->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dmadev->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(dmadev->clk), "Can't get clock\n");

	ret = clk_prepare_enable(dmadev->clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "clk_prep_enable error: %d\n", ret);
		return ret;
	}

	dmadev->mem2mem = of_property_read_bool(pdev->dev.of_node,
						"st,mem2mem");

	rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(rst)) {
		ret = PTR_ERR(rst);
		if (ret == -EPROBE_DEFER)
			goto clk_free;
	} else {
		reset_control_assert(rst);
		udelay(2);
		reset_control_deassert(rst);
	}

	dmadev->sram_pool = of_gen_pool_get(pdev->dev.of_node, "sram", 0);
	if (!dmadev->sram_pool)
		dev_info(&pdev->dev, "no dma pool: can't use MDMA: %d\n", ret);
	else
		dev_dbg(&pdev->dev,
			"SRAM pool: %zu KiB\n", gen_pool_size(dmadev->sram_pool) / 1024);

	dma_set_max_seg_size(&pdev->dev, STM32_DMA_ALIGNED_MAX_DATA_ITEMS);

	dma_cap_set(DMA_SLAVE, dd->cap_mask);
	dma_cap_set(DMA_PRIVATE, dd->cap_mask);
	dma_cap_set(DMA_CYCLIC, dd->cap_mask);
	dd->device_alloc_chan_resources = stm32_dma_alloc_chan_resources;
	dd->device_free_chan_resources = stm32_dma_free_chan_resources;
	dd->device_tx_status = stm32_dma_tx_status;
	dd->device_issue_pending = stm32_dma_issue_pending;
	dd->device_prep_slave_sg = stm32_dma_prep_slave_sg;
	dd->device_prep_dma_cyclic = stm32_dma_prep_dma_cyclic;
	dd->device_caps = stm32_dma_slave_caps;
	dd->device_config = stm32_dma_slave_config;
	dd->device_pause = stm32_dma_pause;
	dd->device_resume = stm32_dma_resume;
	dd->device_terminate_all = stm32_dma_terminate_all;
	dd->device_synchronize = stm32_dma_synchronize;
	dd->src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
		BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
		BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dd->dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
		BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
		BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dd->directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	dd->residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	dd->copy_align = DMAENGINE_ALIGN_32_BYTES;
	dd->max_burst = STM32_DMA_MAX_BURST;
	dd->descriptor_reuse = true;
	dd->dev = &pdev->dev;
	INIT_LIST_HEAD(&dd->channels);

	if (dmadev->mem2mem) {
		dma_cap_set(DMA_MEMCPY, dd->cap_mask);
		dd->device_prep_dma_memcpy = stm32_dma_prep_dma_memcpy;
		dd->directions |= BIT(DMA_MEM_TO_MEM);
	}

	for (i = 0; i < STM32_DMA_MAX_CHANNELS; i++) {
		chan = &dmadev->chan[i];
		chan->id = i;
		chan->vchan.desc_free = stm32_dma_desc_free;
		vchan_init(&chan->vchan, dd);
	}

	ret = dma_async_device_register(dd);
	if (ret)
		goto err_dma;

	for (i = 0; i < STM32_DMA_MAX_CHANNELS; i++) {
		chan = &dmadev->chan[i];
		ret = platform_get_irq(pdev, i);
		if (ret < 0)
			goto err_unregister;
		chan->irq = ret;

		ret = devm_request_irq(&pdev->dev, chan->irq,
				       stm32_dma_chan_irq, 0,
				       dev_name(chan2dev(chan)), chan);
		if (ret) {
			dev_err(&pdev->dev,
				"request_irq failed with err %d channel %d\n",
				ret, i);
			goto err_unregister;
		}

		mchan = &chan->mchan;
		if (dmadev->sram_pool) {
			snprintf(name, sizeof(name), "ch%d", chan->id);
			mchan->chan = dma_request_chan(dd->dev, name);
			if (IS_ERR(mchan->chan)) {
				ret = PTR_ERR(mchan->chan);
				mchan->chan = NULL;
				if (ret == -EPROBE_DEFER)
					goto err_unregister;

				dev_info(&pdev->dev, "can't request MDMA chan for %s\n", name);
			} else {
				/*
				 * Allocate workqueue per channel in case of MDMA/DMA chaining, to
				 * avoid deadlock with MDMA callback stm32_mdma_chan_complete() when
				 * MDMA interrupt handler is executed in a thread (which is the
				 * case in Linux-RT kernel or if force_irqthreads is set).
				 */
				snprintf(dma_wq_name,
					 sizeof(dma_wq_name), "dma%d%swq", dd->dev_id, name);
				chan->mdma_wq = alloc_ordered_workqueue(dma_wq_name,
									WQ_MEM_RECLAIM |
									WQ_HIGHPRI);
				if (!chan->mdma_wq) {
					dma_release_channel(mchan->chan);
					mchan->chan = NULL;
					dev_warn(&pdev->dev,
						 "can't alloc MDMA workqueue for %s\n", name);
				} else {
					dev_dbg(&pdev->dev, "Workqueue name: %s\n", dma_wq_name);
				}
			}
		}
	}

	ret = of_dma_controller_register(pdev->dev.of_node,
					 stm32_dma_of_xlate, dmadev);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"STM32 DMA DMA OF registration failed %d\n", ret);
		goto err_unregister;
	}

	platform_set_drvdata(pdev, dmadev);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_put(&pdev->dev);

	dev_info(&pdev->dev, "STM32 DMA driver registered\n");

	return 0;

err_unregister:
	dma_async_device_unregister(dd);
err_dma:
	for (i = 0; i < STM32_DMA_MAX_CHANNELS; i++)
		if (dmadev->chan[i].mchan.chan)
			dma_release_channel(dmadev->chan[i].mchan.chan);
clk_free:
	clk_disable_unprepare(dmadev->clk);

	return ret;
}

#ifdef CONFIG_PM
static int stm32_dma_runtime_suspend(struct device *dev)
{
	struct stm32_dma_device *dmadev = dev_get_drvdata(dev);

	clk_disable_unprepare(dmadev->clk);

	return 0;
}

static int stm32_dma_runtime_resume(struct device *dev)
{
	struct stm32_dma_device *dmadev = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(dmadev->clk);
	if (ret) {
		dev_err(dev, "failed to prepare_enable clock\n");
		return ret;
	}

	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int stm32_dma_pm_suspend(struct device *dev)
{
	struct stm32_dma_device *dmadev = dev_get_drvdata(dev);
	int id, ret, scr;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	for (id = 0; id < STM32_DMA_MAX_CHANNELS; id++) {
		scr = stm32_dma_read(dmadev, STM32_DMA_SCR(id));
		if (scr & STM32_DMA_SCR_EN) {
			dev_warn(dev, "Suspend is prevented by Chan %i\n", id);
			return -EBUSY;
		}
	}

	pm_runtime_put_sync(dev);

	pm_runtime_force_suspend(dev);

	return 0;
}

static int stm32_dma_pm_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}
#endif

static const struct dev_pm_ops stm32_dma_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stm32_dma_pm_suspend, stm32_dma_pm_resume)
	SET_RUNTIME_PM_OPS(stm32_dma_runtime_suspend,
			   stm32_dma_runtime_resume, NULL)
};

static struct platform_driver stm32_dma_driver = {
	.driver = {
		.name = "stm32-dma",
		.of_match_table = stm32_dma_of_match,
		.pm = &stm32_dma_pm_ops,
	},
	.probe = stm32_dma_probe,
};

static int __init stm32_dma_init(void)
{
	return platform_driver_register(&stm32_dma_driver);
}
device_initcall(stm32_dma_init);
