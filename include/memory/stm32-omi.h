/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, STMicroelectronics - All Rights Reserved
 * Author(s): Patrice Chotard <patrice.chotard@foss.st.com> for STMicroelectronics.
 */

#ifndef __STM32_OMI_H
#define __STM32_OMI_H

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mtd/hyperbus.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
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
#define CR_TCEN			BIT(3)
#define CR_FTHRES_SHIFT		8
#define CR_TEIE			BIT(16)
#define CR_TCIE			BIT(17)
#define CR_SMIE			BIT(19)
#define CR_APMS			BIT(22)
#define CR_CSSEL		BIT(24)
#define CR_FMODE_MASK		GENMASK(29, 28)
#define CR_FMODE_INDW		(0U)
#define CR_FMODE_INDR		(1U)
#define CR_FMODE_APM		(2U)
#define CR_FMODE_MM		(3U)

#define OSPI_DCR1		0x08
#define DCR1_FRCK		BIT(1)
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

#define OSPI_LPTR		0x130
#define LPTR_TIMEOUT_MASK	GENMASK(15, 0)

#define OSPI_WCCR		0x180
#define WCCR_DQSE		BIT(29)
#define WCCR_DDTR		BIT(27)
#define WCCR_DMODE_MASK		GENMASK(26, 24)
#define WCCR_DMODE_8LINES	4
#define WCCR_ADSIZE_MASK	GENMASK(13, 12)
#define WCCR_ADSIZE_32BITS	3
#define WCCR_ADDTR		BIT(11)
#define WCCR_ADMODE_MASK	GENMASK(10, 8)
#define WCCR_ADMODE_8LINES	4

#define OSPI_HLCR		0x200
#define HLCR_WZL		BIT(1)
#define HLCR_TACC_MASK		GENMASK(15, 8)

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

#define STM32_OMI_MAX_MMAP_SZ	SZ_256M
#define STM32_OMI_MAX_NORCHIP	2

#define STM32_FIFO_TIMEOUT_US		30000
#define STM32_ABT_TIMEOUT_US		100000
#define STM32_COMP_TIMEOUT_MS		5000
#define STM32_BUSY_TIMEOUT_US		100000
#define STM32_DLYB_FREQ_THRESHOLD	50000000
#define STM32_DLYBOS_TIMEOUT_MS		1000
#define STM32_DLYBOS_DELAY_NB		24

struct stm32_omi {
	struct device *dev;
	struct platform_device *vdev;
	struct clk *clk;
	struct reset_control *rstc;
	struct regmap *regmap;

	struct completion data_completion;
	struct completion match_completion;

	struct dma_chan *dma_chtx;
	struct dma_chan *dma_chrx;
	struct completion dma_completion;

	void __iomem *regs_base;
	void __iomem *mm_base;
	phys_addr_t regs_phys_base;
	resource_size_t mm_size;
	u32 clk_rate;
	u32 fmode;
	u32 dlyb_base;
	int irq;
	bool calibration;
	bool jedec_flash;

	int (*check_transfer)(struct stm32_omi *omi);
};

struct stm32_tap_window {
	u8 end;
	u8 length;
};

int stm32_omi_abort(struct stm32_omi *omi);
int stm32_omi_dlyb_init(struct stm32_omi *omi, bool bypass_mode,
			u16 period_ps);
int stm32_omi_dlyb_find_tap(struct stm32_omi *omi, bool rx_only,
			    u8 *window_len);
int stm32_omi_dlyb_set_cr(struct stm32_omi *omi, u32 dlyb_cr);
void stm32_omi_dlyb_get_cr(struct stm32_omi *omi, u32 *dlyb_cr);
void stm32_omi_dlyb_stop(struct stm32_omi *omi);
void stm32_omi_dma_callback(void *arg);
void stm32_omi_dma_setup(struct stm32_omi *omi,
			 struct dma_slave_config *dma_cfg);
int stm32_omi_tx_poll(struct stm32_omi *omi, u8 *buf, u32 len, bool read);
int stm32_omi_wait_cmd(struct stm32_omi *omi);
int stm32_omi_wait_nobusy(struct stm32_omi *omi);

#endif /* __STM32_OMI_H */
