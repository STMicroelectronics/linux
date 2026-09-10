// SPDX-License-Identifier: GPL-2.0-only
/*
 * STM32 I3C controller driver
 *
 * Copyright (C) STMicroelectronics 2026
 * Author(s): Amelie Delaunay <amelie.delaunay@foss.st.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/i3c/master.h>
#include <linux/iopoll.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/units.h>

#define STM32_I3C_CR            0x00
#define STM32_I3C_CFGR          0x04
#define STM32_I3C_RDR           0x10
#define STM32_I3C_RDWR          0x14
#define STM32_I3C_TDR           0x18
#define STM32_I3C_TDWR          0x1c
#define STM32_I3C_IBIDR         0x20
#define STM32_I3C_TGTTDR        0x24 /* Target only */
#define STM32_I3C_SR            0x30
#define STM32_I3C_SER           0x34
#define STM32_I3C_RMR           0x40
#define STM32_I3C_EVR           0x50
#define STM32_I3C_IER           0x54
#define STM32_I3C_CEVR          0x58
#define STM32_I3C_DEVR0         0x60
#define STM32_I3C_DEVR(x)       (0x64 + (0x4 * (x)))
#define STM32_I3C_MAXRLR        0x90 /* Target only */
#define STM32_I3C_MAXWLR        0x94 /* Target only */
#define STM32_I3C_TIMINGR0      0xa0
#define STM32_I3C_TIMINGR1      0xa4
#define STM32_I3C_TIMINGR2      0xa8
#define STM32_I3C_BCR           0xc0 /* Target only */
#define STM32_I3C_DCR           0xc4 /* Target only */
#define STM32_I3C_GETCAPR       0xc8 /* Target only */
#define STM32_I3C_CRCAPR        0xcc /* Target only */
#define STM32_I3C_GETMXDSR      0xd0 /* Target only */
#define STM32_I3C_EPIDR         0xd4 /* Target only */
#define STM32_I3C_HWCFGR        0x3f0
#define STM32_I3C_VERR          0x3f4

/* I3C_CR I3C message control register */
#define I3C_CR_DCNT		GENMASK(15, 0)
#define I3C_CR_RNW		BIT(16)
#define I3C_CR_ADD		GENMASK(23, 17)
#define I3C_CR_CCC		GENMASK(23, 16) /* [alternate] I3C_CR, for CCC */
#define I3C_CR_MTYPE		GENMASK(30, 27)
#define I3C_CR_MEND		BIT(31)
#define MTYPE_I3C_SCL_STOP	0 /* in case of CE1 error */
#define MTYPE_I3C_HEADER_MSG	1 /* ?in case of escalation stage */
#define MTYPE_I3C_PRIV_MSG	2 /* .priv_xfers */
#define MTYPE_I3C_DIRECT_MSG	3 /* second part of .send_ccc_cmd(direct) */
#define MTYPE_I2C_MSG		4 /* .i2c_xfers */
#define MTYPE_I3C_CCC_CMD	6 /* .send_ccc_cmd(broadcast/direct), .do_daa */

/* I3C_CFGR I3C configuration register */
#define I3C_CFGR_EN		BIT(0)
#define I3C_CFGR_CRINIT		BIT(1)
#define I3C_CFGR_NOARBH		BIT(2)
#define I3C_CFGR_RSTPTRN	BIT(3)
#define I3C_CFGR_EXITPTRN	BIT(4)
#define I3C_CFGR_HKSDAEN	BIT(5)
#define I3C_CFGR_HJACK		BIT(7)
#define I3C_CFGR_RXDMAEN	BIT(8)
#define I3C_CFGR_RXFLUSH	BIT(9)
#define I3C_CFGR_RXTHRES	BIT(10)
#define I3C_CFGR_TXDMAEN	BIT(12)
#define I3C_CFGR_TXFLUSH	BIT(13)
#define I3C_CFGR_TXTHRES	BIT(14)
#define I3C_CFGR_SDMAEN		BIT(16)
#define I3C_CFGR_SFLUSH		BIT(17)
#define I3C_CFGR_SMODE		BIT(18)
#define I3C_CFGR_TMODE		BIT(19)
#define I3C_CFGR_CDMAEN		BIT(20)
#define I3C_CFGR_CFLUSH		BIT(21)
#define I3C_CFGR_TSFSET		BIT(30)

/* I3C_RDR I3C receive data byte register */
#define I3C_RDR_RDB0		GENMASK(7, 0)

/* I3C_RDWR I3C receive data word register */
#define I3C_RDWR_RDB0		GENMASK(7, 0)
#define I3C_RDWR_RDB1		GENMASK(15, 8)
#define I3C_RDWR_RDB2		GENMASK(23, 16)
#define I3C_RDWR_RDB3		GENMASK(31, 24)

/* I3C_TDR I3C transmit data byte register */
#define I3C_RDR_TDB0		GENMASK(7, 0)

/* I3C_TDWR I3C receive data word register */
#define I3C_TDWR_TDB0		GENMASK(7, 0)
#define I3C_TDWR_TDB1		GENMASK(15, 8)
#define I3C_TDWR_TDB2		GENMASK(23, 16)
#define I3C_TDWR_TDB3		GENMASK(31, 24)

/* I3C_IBIDR I3C IBI payload data register */
#define I3C_IBIDR_IBIDB0	GENMASK(7, 0)
#define I3C_IBIDR_IBIDB1	GENMASK(15, 8)
#define I3C_IBIDR_IBIDB2	GENMASK(23, 16)
#define I3C_IBIDR_IBIDB3	GENMASK(31, 24)

/* I3C_SR I3C status register */
#define I3C_SR_XDCNT		GENMASK(15, 0)
#define I3C_SR_ABT		BIT(17)
#define I3C_SR_DIR		BIT(18)
#define I3C_SR_MID		GENMASK(31, 24)

/* I3C_SER I3C status error register */
#define I3C_SER_CODERR		GENMASK(3, 0)
#define I3C_SER_PERR		BIT(4)
#define I3C_SER_STALL		BIT(5)
#define I3C_SER_DOVR		BIT(6)
#define I3C_SER_COVR		BIT(7)
#define I3C_SER_ANACK		BIT(8)
#define I3C_SER_DNACK		BIT(9)
#define I3C_SER_DERR		BIT(10)
/* I3C_SER.CODERR[3:0] protocol error codes, when acting as controller */
#define CODERR_CE0		0 /* Illegally formatted CCC */
#define CODERR_CE1		1 /* Monitoring error */
#define CODERR_CE2		2 /* No response to broadcast address 0b111_1110 */
#define CODERR_CE3		3 /* Failed controller-role hand-off */

/* I3C_RMR I3C received message register */
#define I3C_RMR_IBIRDCNT	GENMASK(2, 0)
#define I3C_RMR_RCODE		GENMASK(15, 8)
#define I3C_RMR_RADD		GENMASK(23, 17)

/* I3C_EVR I3C event register */
#define I3C_EVR_CFEF		BIT(0)
#define I3C_EVR_TXFEF		BIT(1)
#define I3C_EVR_CFNFF		BIT(2)
#define I3C_EVR_SFNEF		BIT(3)
#define I3C_EVR_TXFNFF		BIT(4)
#define I3C_EVR_RXFNEF		BIT(5)
#define I3C_EVR_TXLASTF		BIT(6)
#define I3C_EVR_RXLASTF		BIT(7)
#define I3C_EVR_FCF		BIT(9)
#define I3C_EVR_RXTGTENDF	BIT(10)
#define I3C_EVR_ERRF		BIT(11)
#define I3C_EVR_IBIF		BIT(15)
#define I3C_EVR_IBIENDF		BIT(16)
#define I3C_EVR_CRF		BIT(17)
#define I3C_EVR_CRUPDF		BIT(18)
#define I3C_EVR_HJF		BIT(19)
#define I3C_EVR_WKPF		BIT(21) /* Target */
#define I3C_EVR_GETF		BIT(22) /* Target */
#define I3C_EVR_STAF		BIT(23) /* Target */
#define I3C_EVR_DAUPDF		BIT(24) /* Target */
#define I3C_EVR_MWLUPDF		BIT(25) /* Target */
#define I3C_EVR_MRLUPDF		BIT(26) /* Target */
#define I3C_EVR_RSTF		BIT(27) /* Target */
#define I3C_EVR_ASUPDF		BIT(28) /* Target */
#define I3C_EVR_INTUPDF		BIT(29) /* Target */
#define I3C_EVR_DEFF		BIT(30) /* Target */
#define I3C_EVR_GRPF		BIT(31) /* Target */

/* I3C_IER I3C interrupt enable register */
#define I3C_IER_CFNFIE		BIT(2)
#define I3C_IER_SFNEIE		BIT(3)
#define I3C_IER_TXFNFIE		BIT(4)
#define I3C_IER_RXFNEIE		BIT(5)
#define I3C_IER_FCIE		BIT(9)
#define I3C_IER_RXTGTENDIE	BIT(10)
#define I3C_IER_ERRIE		BIT(11)
#define I3C_IER_IBIIE		BIT(15)
#define I3C_IER_IBIENDIE	BIT(16) /* Target */
#define I3C_IER_CRIE		BIT(17)
#define I3C_IER_CRUPDIE		BIT(18) /* Target */
#define I3C_IER_HJIE		BIT(19)
#define I3C_IER_WKPIE		BIT(21) /* Target */
#define I3C_IER_GETIE		BIT(22) /* Target */
#define I3C_IER_STAIE		BIT(23) /* Target */
#define I3C_IER_DAUPDIE		BIT(24) /* Target */
#define I3C_IER_MWLUPDIE	BIT(25) /* Target */
#define I3C_IER_MRLUPDIE	BIT(26) /* Target */
#define I3C_IER_RSTIE		BIT(27) /* Target */
#define I3C_IER_ASUPDIE		BIT(28) /* Target */
#define I3C_IER_INTUPDIE	BIT(29) /* Target */
#define I3C_IER_DEFIE		BIT(30) /* Target */
#define I3C_IER_GRPIE		BIT(31) /* Target */
#define I3C_IER_CTRL_IE		(I3C_IER_FCIE | I3C_IER_RXTGTENDIE | I3C_IER_ERRIE | \
				 I3C_IER_IBIIE | I3C_IER_CRIE | I3C_IER_HJIE)
#define I3C_IER_FIFO_IE		(I3C_IER_CFNFIE | I3C_IER_SFNEIE | \
				 I3C_IER_TXFNFIE | I3C_IER_RXFNEIE)

/* I3C_CEVR I3C clear event register */
#define I3C_CEVR_CFCF		BIT(9)
#define I3C_CEVR_CRXTGTENDF	BIT(10)
#define I3C_CEVR_CERRF		BIT(11)
#define I3C_CEVR_CIBIF		BIT(15)
#define I3C_CEVR_CIBIENDF	BIT(16)
#define I3C_CEVR_CCRF		BIT(17)
#define I3C_CEVR_CCRUPDF	BIT(18)
#define I3C_CEVR_CHJF		BIT(19)
#define I3C_CEVR_CWKPF		BIT(21) /* Target */
#define I3C_CEVR_CGETF		BIT(22) /* Target */
#define I3C_CEVR_CSTAF		BIT(23) /* Target */
#define I3C_CEVR_CDAUPDF	BIT(24) /* Target */
#define I3C_CEVR_CMWLUPDF	BIT(25) /* Target */
#define I3C_CEVR_CMRLUPDF	BIT(26) /* Target */
#define I3C_CEVR_CRSTF		BIT(27) /* Target */
#define I3C_CEVR_CASUPDF	BIT(28) /* Target */
#define I3C_CEVR_CINTUPDF	BIT(29) /* Target */
#define I3C_CEVR_CDEFF		BIT(30) /* Target */
#define I3C_CEVR_CGRPF		BIT(31) /* Target */
/* Only these I3C_EVR flags are clearable, all the other I3C_CEVR bits are reserved */
#define I3C_CEVR_CTRL_MASK	(I3C_CEVR_CFCF | I3C_CEVR_CRXTGTENDF | I3C_CEVR_CERRF | \
				 I3C_CEVR_CIBIF | I3C_CEVR_CCRF | I3C_CEVR_CHJF)

/* I3C_DEVR0 I3C own device characteristics register */
#define I3C_DEVR0_DAVAL		BIT(0) /* Target */
#define I3C_DEVR0_DA		GENMASK(7, 1)
#define I3C_DEVR0_IBIEN		BIT(16) /* Target */
#define I3C_DEVR0_CREN		BIT(17) /* Target */
#define I3C_DEVR0_HJEN		BIT(19) /* Target */
#define I3C_DEVR0_AS		GENMASK(21, 20) /* Target */
#define I3C_DEVR0_RSTACT	GENMASK(23, 22) /* Target */
#define I3C_DEVR0_RSTVAL	BIT(24) /* Target */

/* I3C_DEVRx I3C_device x characteristics register */
#define I3C_DEVR_DA		GENMASK(7, 1)
#define I3C_DEVR_IBIACK		BIT(16)
#define I3C_DEVR_CRACK		BIT(17)
#define I3C_DEVR_IBIDEN		BIT(18)
#define I3C_DEVR_SUSP		BIT(19)
#define I3C_DEVR_DIS		BIT(31)

/* I3C_MAXRLR maximum read length register */
/* I3C_MAXWLR maximum write length register */

/* I3C_TIMINGR0 I3C timing register 0 */
#define I3C_TIMINGR0_SCLL_PP	GENMASK(7, 0)
#define I3C_TIMINGR0_SCLH_I3C	GENMASK(15, 8)
#define I3C_TIMINGR0_SCLL_OD	GENMASK(23, 16)
#define I3C_TIMINGR0_SCLH_I2C	GENMASK(31, 24)

/* I3C_TIMINGR1 I3C timing register 1 */
#define I3C_TIMINGR1_AVAL	GENMASK(7, 0)
#define I3C_TIMINGR1_ASNCR	GENMASK(9, 8)
#define I3C_TIMINGR1_FREE	GENMASK(22, 16)
#define I3C_TIMINGR1_SDA_HD	BIT(28)

/* I3C_TIMINGR2 I3C timing register 2 */
#define I3C_TIMINGR2_STALLT	BIT(0)
#define I3C_TIMINGR2_STALLD	BIT(1)
#define I3C_TIMINGR2_STALLC	BIT(2)
#define I3C_TIMINGR2_STALLA	BIT(3)
#define I3C_TIMINGR2_STALL	GENMASK(15, 8)

/* I3C_BCR I3C bus characteristics register */
/* I3C_DCR I3C device characteristics register */
/* I3C_GETCAPR I3C get capability register */
/* I3C_CRCAPR I3C controller-role capability register */
/* I3C_GETMXDSR I3C get capability (maximum data speed) register */
/* I3C_EPIDR I3C extended provisioned ID register */

/* I3C_HWCFGR I3C hardware configuration register */
#define I3C_HWCFGR_CSIZE        GENMASK(3, 0)
#define I3C_HWCFGR_SSIZE        GENMASK(7, 4)
#define I3C_HWCFGR_TSIZE        GENMASK(11, 8)
#define I3C_HWCFGR_RSIZE        GENMASK(15, 12)
#define I3C_HWCFGR_NBT          GENMASK(19, 16)

/* I3C_VERR I3C version register */
#define I3C_VERR_MINREV         GENMASK(3, 0)
#define I3C_VERR_MAJREV         GENMASK(7, 4)

#define I3C_BUS_THIGH_MIN_NS		24	/* 24ns tHIGH Min */
#define I3C_BUS_TLOW_PP_MIN_NS		24	/* 24ns tLOW PP Min */
#define I3C_BUS_TLOW_MIXED_FM_MIN_NS	1300	/* 1300ns tLOW Min I2C FM */
#define I3C_BUS_THIGH_MIXED_FM_MIN_NS	600	/* 600ns tHIGH Min I2C FM */
#define I3C_BUS_TLOW_MIXED_FMP_MIN_NS	500	/* 500ns tLOW Min I2C FMP */
#define I3C_BUS_THIGH_MIXED_FMP_MIN_NS	260	/* 260ns tHIGH Min I2C FMP */
#define I3C_BUS_HD_PP_MIN_NS		3	/* 3ns tHD_PP Min*/
#define I3C_BUS_TCAS_MIN_NS		39	/* 38.4ns tCAS Min rounded up */
#define I3C_BUS_TBUF_MIXED_FMP_MIN_NS	500	/* 500ns tBUF Min I2C FMP */

#define I3C_IBI_MAX_PAYLOAD_LEN		4 /* 4 bytes (IBIDB0..3) in I3C_IBIDR */

#define STM32_I3C_AUTOSUSPEND_DELAY_MS	1000

struct stm32_i3c_cmd {
	u32 msg_cr; /* Used to prepare MTYPE, (ADD, RNW/CCC) */
	unsigned int data_len;
	union {
		const void *tx;
		void *rx;
	} data;
	enum i3c_error_code error;
};

struct stm32_i3c_xfer {
	struct list_head node;
	struct completion comp;
	int ret;
	int cur_cmd;
	unsigned int cur_tx;
	unsigned int cur_rx;
	unsigned int ncmds;
	struct stm32_i3c_cmd cmds[] __counted_by(ncmds);
};

struct stm32_i3c_dev_ibi_data {
	struct i3c_ibi_slot *ibi_slot;
	struct i3c_generic_ibi_pool *ibi_pool;
	unsigned int max_len;
};

struct stm32_i3c_dev_data {
	int dev_idx;
	struct stm32_i3c_dev_ibi_data *ibi_data;
};

struct stm32_i3c_ddata {
	struct device *dev;
	struct i3c_master_controller master;
	void __iomem *base;
	struct clk *clk;
	int irq;
	struct work_struct hotjoin_work;
	bool hotjoin_enabled;
	u32 nslots; /* Number of I3C targets, I2C targets are not limited */
	unsigned long *slots;
	struct {
		struct list_head list;
		struct stm32_i3c_xfer *cur_xfer;
		/* Prevent races between transfers */
		spinlock_t lock;
	} xferqueue;
	u32 timingr0_normal;
};

static inline struct stm32_i3c_ddata *to_stm32_i3c_ddata(struct i3c_master_controller *m)
{
	return container_of(m, struct stm32_i3c_ddata, master);
}

static inline struct i3c_dev_desc *addr_to_i3c_dev_desc(struct i3c_master_controller *m, u8 addr)
{
	struct i3c_bus *bus = i3c_master_get_bus(m);
	struct i3c_dev_desc *i3cdev;

	i3c_bus_for_each_i3cdev(bus, i3cdev) {
		if (i3cdev->info.dyn_addr == addr)
			return i3cdev;
	}
	return NULL;
}

static void stm32_i3c_master_hotjoin_work(struct work_struct *work)
{
	struct stm32_i3c_ddata *ddata = container_of(work, struct stm32_i3c_ddata, hotjoin_work);
	int ret;

	if (!READ_ONCE(ddata->hotjoin_enabled))
		return;

	ret = i3c_master_do_daa(&ddata->master);
	if (ret)
		dev_err(ddata->dev, "Hot-Join DAA failed: %d\n", ret);
}

static void stm32_i3c_master_send_stop(struct stm32_i3c_ddata *ddata)
{
	u32 cr = FIELD_PREP(I3C_CR_MTYPE, MTYPE_I3C_SCL_STOP) | I3C_CR_MEND;

	writel_relaxed(cr, ddata->base + STM32_I3C_CR);
	dev_warn(ddata->dev, "Stop SCL\n");

	/*
	 * Must wait at least 150us before emitting another message.
	 * stm32_i3c_master_send_stop() can be called from atomic context, hence udelay.
	 */
	udelay(150);
}

/*
 * The C-FIFO index (cur_cmd) runs ahead of the messages actually being executed on the bus:
 * the hardware requests the next control word (CFNFF) as soon as there is room in the C-FIFO.
 * Data must therefore be taken from/put into the command matching the message being executed.
 * TX and RX commands need independent cursors because TX payloads may be preloaded before
 * an earlier read command has completed.
 * Messages without payload (DCNT = 0), such as the first part of a direct CCC, never trigger
 * a TX/RX request and are skipped.
 */
static struct stm32_i3c_cmd *stm32_i3c_master_next_data_cmd(struct stm32_i3c_ddata *ddata,
							    bool read)
{
	struct stm32_i3c_xfer *cur_xfer = ddata->xferqueue.cur_xfer;
	struct stm32_i3c_cmd *cmd;
	unsigned int *cur_data;
	u32 mtype;

	if (!cur_xfer)
		return NULL;

	cur_data = read ? &cur_xfer->cur_rx : &cur_xfer->cur_tx;
	while (*cur_data < cur_xfer->ncmds) {
		cmd = &cur_xfer->cmds[*cur_data];
		mtype = FIELD_GET(I3C_CR_MTYPE, cmd->msg_cr);
		if (cmd->data_len &&
		    (mtype != MTYPE_I3C_CCC_CMD &&
		     !!(cmd->msg_cr & I3C_CR_RNW)) == read)
			break;
		(*cur_data)++;
	}

	if (*cur_data >= cur_xfer->ncmds)
		return NULL;

	return &cur_xfer->cmds[(*cur_data)++];
}

static int stm32_i3c_master_write_txfifo(struct stm32_i3c_ddata *ddata)
{
	struct stm32_i3c_cmd *cmd = stm32_i3c_master_next_data_cmd(ddata, false);
	u32 offset = 0, len, evr;
	const u8 *p;
	int ret;

	if (!cmd) /* TODO: flush? */
		return -EINVAL;

	p = cmd->data.tx;
	len = cmd->data_len;
	/* FIXME: keep offset of data pointer to resume a partial message */
	while (offset < len) {
		ret = readl_relaxed_poll_timeout_atomic(ddata->base + STM32_I3C_EVR, evr,
							evr & I3C_EVR_TXFNFF, 0, 1000);
		if (ret)
			return ret;

		/* TODO I3C_CFGR.TXTHRES=1
		 * writel_relaxed(p[offset], ddata->base + STM32_I3C_TDWR);
		 * offset += sizeof(u32);
		 */
		writeb_relaxed(p[offset++], ddata->base + STM32_I3C_TDR);
	}

	return 0;
}

static int stm32_i3c_master_set_speed(struct i3c_master_controller *m,
				      enum i3c_open_drain_speed speed)
{
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	unsigned long i3cclk_rate;
	u32 timingr0, sclhi3c, cfgr;
	int ret;

	ret = pm_runtime_resume_and_get(ddata->dev);
	if (ret)
		return ret;

	switch (speed) {
	case I3C_OPEN_DRAIN_SLOW_SPEED:
		i3cclk_rate = clk_get_rate(ddata->clk);
		if (!i3cclk_rate) {
			ret = -EINVAL;
			goto out;
		}
		sclhi3c = DIV_ROUND_UP_ULL((u64)I3C_BUS_THIGH_INIT_OD_MIN_NS * i3cclk_rate,
					   NSEC_PER_SEC) - 1;
		if (sclhi3c > FIELD_MAX(I3C_TIMINGR0_SCLH_I3C)) {
			ret = -EINVAL;
			goto out;
		}

		timingr0 = ddata->timingr0_normal & ~I3C_TIMINGR0_SCLH_I3C;
		timingr0 |= FIELD_PREP(I3C_TIMINGR0_SCLH_I3C, sclhi3c);
		break;
	case I3C_OPEN_DRAIN_NORMAL_SPEED:
		timingr0 = ddata->timingr0_normal;
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	/* TIMINGR0 is read-only when the I3C peripheral is enabled hence ensure I3C peripheral is
	 * disabled when writing into it
	 */
	cfgr = readl_relaxed(ddata->base + STM32_I3C_CFGR);
	writel_relaxed(cfgr & ~I3C_CFGR_EN, ddata->base + STM32_I3C_CFGR);

	writel_relaxed(timingr0, ddata->base + STM32_I3C_TIMINGR0);

	writel_relaxed(cfgr, ddata->base + STM32_I3C_CFGR);

out:
	pm_runtime_put_autosuspend(ddata->dev);
	return ret;
}

static int stm32_i3c_master_read_rxfifo(struct stm32_i3c_ddata *ddata, unsigned int max_len)
{
	struct stm32_i3c_cmd *cmd = stm32_i3c_master_next_data_cmd(ddata, true);
	u32 len, evr;
	u8 *p;
	int ret;

	if (!cmd) /* TODO: flush? */
		return -EINVAL;

	p = cmd->data.rx;
	len = min(cmd->data_len, max_len);
	/* FIXME: keep offset of data pointer to resume a partial message */
	while (len) {
		ret = readl_relaxed_poll_timeout_atomic(ddata->base + STM32_I3C_EVR, evr,
							evr & I3C_EVR_RXFNEF, 0, 1000);
		if (ret)
			return ret;

		/* TODO I3C_CFGR.RXTHRES=1
		 * *p = readl_relaxed(ddata->base + STM32_I3C_RDWR);
		 * if (len < sizeof(u32)) {
		 *	p += len;
		 *	len = 0;
		 * } else {
		 *	p += sizeof(u32);
		 *	len -= sizeof(u32);
		 * }
		 */
		*p = readb_relaxed(ddata->base + STM32_I3C_RDR);
		p++;
		len--;
	}

	return 0;
}

static u32 stm32_i3c_master_read_error(struct stm32_i3c_ddata *ddata)
{
	u32 ser = readl_relaxed(ddata->base + STM32_I3C_SER);
	const char *bus_name = dev_name(&ddata->master.dev);

	if (ser & I3C_SER_DERR)
		dev_warn(ddata->dev, "(%s) data error during controller-role hand-off\n", bus_name);

	if (ser & I3C_SER_DNACK)
		dev_warn(ddata->dev, "(%s) data not acknowledged\n", bus_name);

	if (ser & I3C_SER_ANACK)
		dev_warn(ddata->dev, "(%s) address not acknowledged\n", bus_name);

	if (ser & I3C_SER_COVR)
		dev_warn(ddata->dev, "(%s) C-FIFO underrun or S-FIFO overrun\n", bus_name);

	if (ser & I3C_SER_DOVR)
		dev_warn(ddata->dev, "(%s) RX-FIFO overrun or TX-FIFO underrun\n", bus_name);

	if (ser & I3C_SER_PERR) {
		u32 coderr = FIELD_GET(I3C_SER_CODERR, ser);

		switch (coderr) {
		case CODERR_CE0:
			dev_warn(ddata->dev, "(%s) illegally formatted CCC [CE0]\n", bus_name);
			break;
		case CODERR_CE1:
			dev_warn(ddata->dev, "(%s) monitoring error [CE1]\n", bus_name);
			break;
		case CODERR_CE2:
			dev_warn(ddata->dev, "(%s) No response to broadcast address [CE2]\n",
				 bus_name);
			break;
		case CODERR_CE3:
			dev_warn(ddata->dev, "(%s) failed controller-role hand-off [CE3]\n",
				 bus_name);
			break;
		default:
			dev_warn(ddata->dev, "(%s) protocol error [%#x]\n", bus_name, coderr);
			break;
		}
	}

	return ser;
}

/* Map the I3C_SER.CODERR[3:0] controller error code onto the I3C core error code */
static enum i3c_error_code stm32_i3c_master_coderr_to_error(u32 coderr)
{
	switch (coderr) {
	case CODERR_CE0:
		return I3C_ERROR_M0;
	case CODERR_CE1:
		return I3C_ERROR_M1;
	case CODERR_CE2:
		return I3C_ERROR_M2;
	default:
		return I3C_ERROR_UNKNOWN;
	}
}

static u32 stm32_i3c_master_read_status(struct stm32_i3c_ddata *ddata)
{
	u32 sr = readl_relaxed(ddata->base + STM32_I3C_SR);

	dev_dbg(ddata->dev, "%s: %s transfer %lu, data counter=%lu\n", __func__,
		FIELD_GET(I3C_SR_DIR, sr) == 0 ? "Write" : "Read",
		FIELD_GET(I3C_SR_MID, sr) + 1, FIELD_GET(I3C_SR_XDCNT, sr));

	return sr;
}

static void stm32_i3c_master_fifo_flush(struct stm32_i3c_ddata *ddata)
{
	u32 cfgr = readl_relaxed(ddata->base + STM32_I3C_CFGR);

	cfgr |= I3C_CFGR_CFLUSH | I3C_CFGR_SFLUSH | I3C_CFGR_TXFLUSH | I3C_CFGR_RXFLUSH;

	writel_relaxed(cfgr, ddata->base + STM32_I3C_CFGR);
}

static void stm32_i3c_master_request_xfer(struct stm32_i3c_ddata *ddata)
{
	u32 cfgr = readl_relaxed(ddata->base + STM32_I3C_CFGR);

	writel_relaxed(cfgr | I3C_CFGR_TSFSET, ddata->base + STM32_I3C_CFGR);

	dev_dbg(ddata->dev, "%s: cfgr=%#x\n", __func__, cfgr);
	/*
	 * This causes the HW to raise I3C_EVR.CFNFF=1,
	 * to request a first control word I3C_CR to be written.
	 */

	/*
	 * Alternative is to write a first control word into I3C_CR, while I3C_EVR.CFEF = 1,
	 * if I3C_CR.MEND = 0, this will cause the HW to raise I3C_EVR.CNFF=1 to request the next
	 * control word.
	 */
}

static void stm32_i3c_master_set_arb_header(struct stm32_i3c_ddata *ddata, bool enable)
{
	u32 cfgr = readl_relaxed(ddata->base + STM32_I3C_CFGR);

	/*
	 * This is a more performing option (when the emission of the 0x7E arbitrable header is
	 * useless), but must be used only when the controller is sure that the addressed target
	 * device cannot emit concurrently an IBI or a controller-role request (to insure no
	 * misinterpretation and no potential conflict between the address emitted by the controller
	 * in open-drain mode and the same address a target device can emit after a start, for IBI
	 * or MR).
	 */
	if (enable)
		cfgr &= ~I3C_CFGR_NOARBH; /* Arbitration header (7'h7E) is sent after Start */
	else
		cfgr |= I3C_CFGR_NOARBH; /* Target address is sent after Start */

	writel_relaxed(cfgr, ddata->base + STM32_I3C_CFGR);
}

static void stm32_i3c_master_set_fifo_preload(struct stm32_i3c_ddata *ddata, bool enable)
{
	u32 cfgr = readl_relaxed(ddata->base + STM32_I3C_CFGR);

	if (enable)
		cfgr |= I3C_CFGR_TMODE; /* C-FIFO (& TX-FIFO if needed) preload before Start */
	else
		cfgr &= ~I3C_CFGR_TMODE; /* Start as soon as 1st control word written in C-FIFO */

	writel_relaxed(cfgr, ddata->base + STM32_I3C_CFGR);
}

static void stm32_i3c_master_dequeue_xfer(struct stm32_i3c_ddata *ddata,
					  struct stm32_i3c_xfer *xfer)
{
	if (ddata->xferqueue.cur_xfer == xfer) {
		ddata->xferqueue.cur_xfer = NULL;

		/* FIXME: reset/flush FIFOs? */
	} else {
		list_del_init(&xfer->node);
	}
}

static void stm32_i3c_master_start_xfer_locked(struct stm32_i3c_ddata *ddata)
{
	struct stm32_i3c_xfer *xfer = ddata->xferqueue.cur_xfer;
	u32 mtype;

	if (!xfer) {
		dev_dbg(ddata->dev, "%s: no cur_xfer\n", __func__);
		return;
	}

	mtype = FIELD_GET(I3C_CR_MTYPE, xfer->cmds[0].msg_cr);
	if (mtype == MTYPE_I2C_MSG || mtype == MTYPE_I3C_PRIV_MSG)
		stm32_i3c_master_set_arb_header(ddata, false);
	else
		stm32_i3c_master_set_arb_header(ddata, true);

	stm32_i3c_master_set_fifo_preload(ddata, true);

	stm32_i3c_master_request_xfer(ddata);
}

static void stm32_i3c_master_abort_xfer(struct stm32_i3c_ddata *ddata,
					struct stm32_i3c_xfer *xfer)
{
	unsigned long flags;

	/*
	 * The threaded handler services complete payloads and can retain xfer outside
	 * the queue lock. Disabling the IRQ waits for that handler before the caller frees xfer.
	 */
	disable_irq(ddata->irq);
	spin_lock_irqsave(&ddata->xferqueue.lock, flags);
	stm32_i3c_master_dequeue_xfer(ddata, xfer);
	if (!ddata->xferqueue.cur_xfer) {
		xfer = list_first_entry_or_null(&ddata->xferqueue.list,
						struct stm32_i3c_xfer, node);
		if (xfer)
			list_del_init(&xfer->node);
		ddata->xferqueue.cur_xfer = xfer;
		stm32_i3c_master_fifo_flush(ddata);
		stm32_i3c_master_start_xfer_locked(ddata);
	}
	spin_unlock_irqrestore(&ddata->xferqueue.lock, flags);
	enable_irq(ddata->irq);
}

static void stm32_i3c_master_end_xfer_locked(struct stm32_i3c_ddata *ddata, u32 evr)
{
	struct stm32_i3c_xfer *xfer = ddata->xferqueue.cur_xfer;
	enum i3c_error_code code = I3C_ERROR_UNKNOWN;
	u32 status, id, error;
	int i, ret;

	if (!xfer)
		return;

	ret = xfer->ret == -ETIMEDOUT ? 0 : xfer->ret;
	status = stm32_i3c_master_read_status(ddata);
	id = FIELD_GET(I3C_SR_MID, status);
	if (!(evr & I3C_EVR_ERRF))
		goto skip_error;

	error = stm32_i3c_master_read_error(ddata);

	if ((error & I3C_SER_COVR) | (error & I3C_SER_DOVR)) {
		ret = -ENOSPC;
	} else if ((error & I3C_SER_DNACK) | (error & I3C_SER_ANACK)) {
		ret = -EIO;
	} else if (error & I3C_SER_DERR) {
		ret = -EINVAL;
	} else if (error & I3C_SER_PERR) {
		u32 coderr = FIELD_GET(I3C_SER_CODERR, error);

		/*
		 * On a CE1 error, SCL is left running by the hardware: a start, repeated start or
		 * stop cannot be generated anymore until the software forces SCL to stop.
		 */
		if (coderr == CODERR_CE1 && !(evr & I3C_EVR_RXTGTENDF))
			stm32_i3c_master_send_stop(ddata);
		code = stm32_i3c_master_coderr_to_error(coderr);
		ret = -EPROTO;
	}

	/* In case error, the message ID stored in status register is the faulty one */
	if (ret)
		for (i = id; i < xfer->ncmds; i++)
			xfer->cmds[i].error = code;

skip_error:
	if (!ret)
		dev_dbg(ddata->dev, "Transfer %s (%u/%u) success\n",
			(evr & I3C_EVR_RXTGTENDF) ? "prematurely ended" : "ended",
			id + 1, xfer->ncmds);
	else
		dev_dbg(ddata->dev, "Transfer end (%u/%u) %s [%pe]\n", id + 1, xfer->ncmds,
			code == I3C_ERROR_M2 ? "nacked" : "error", ERR_PTR(ret));

	xfer->ret = ret;
	complete(&xfer->comp);

	stm32_i3c_master_fifo_flush(ddata);

	xfer = list_first_entry_or_null(&ddata->xferqueue.list, struct stm32_i3c_xfer, node);
	if (xfer)
		list_del_init(&xfer->node);

	ddata->xferqueue.cur_xfer = xfer;
	stm32_i3c_master_start_xfer_locked(ddata);
}

static void stm32_i3c_master_queue_xfer(struct stm32_i3c_ddata *ddata,
					struct stm32_i3c_xfer *xfer)
{
	unsigned long flags;

	init_completion(&xfer->comp);
	spin_lock_irqsave(&ddata->xferqueue.lock, flags);
	if (ddata->xferqueue.cur_xfer) {
		dev_dbg(ddata->dev, "%s: added to xferqueue\n", __func__);
		list_add_tail(&xfer->node, &ddata->xferqueue.list);
	} else {
		dev_dbg(ddata->dev, "%s: started\n", __func__);
		ddata->xferqueue.cur_xfer = xfer;
		stm32_i3c_master_start_xfer_locked(ddata);
	}
	spin_unlock_irqrestore(&ddata->xferqueue.lock, flags);
}

static struct stm32_i3c_xfer *stm32_i3c_master_alloc_xfer(struct stm32_i3c_ddata *ddata,
							  unsigned int ncmds)
{
	struct stm32_i3c_xfer *xfer;

	xfer = kzalloc(struct_size(xfer, cmds, ncmds), GFP_KERNEL);
	if (!xfer)
		return NULL;

	INIT_LIST_HEAD(&xfer->node);
	xfer->ncmds = ncmds;
	xfer->ret = -ETIMEDOUT;
	/* The first CFNFF event pushes the control word of cmds[0] */
	xfer->cur_cmd = -1;
	xfer->cur_tx = 0;
	xfer->cur_rx = 0;

	return xfer;
}

static void stm32_i3c_master_free_xfer(struct stm32_i3c_xfer *xfer)
{
	kfree(xfer);
}

static void stm32_i3c_master_handle_ibi(struct stm32_i3c_ddata *ddata)
{
	struct stm32_i3c_dev_ibi_data *ibi_data;
	struct stm32_i3c_dev_data *i3cdev_data;
	struct i3c_dev_desc *i3cdev;
	u8 addr, len;
	u32 rmr, ibidr;
	unsigned int i;
	u8 *p;

	rmr = readl_relaxed(ddata->base + STM32_I3C_RMR);
	addr = FIELD_GET(I3C_RMR_RADD, rmr);
	len = FIELD_GET(I3C_RMR_IBIRDCNT, rmr);

	dev_dbg(ddata->dev, "%s: %#x\n", __func__, rmr);

	i3cdev = addr_to_i3c_dev_desc(&ddata->master, addr);
	if (!i3cdev) {
		dev_err(ddata->dev, "IBI received but unknown target %#x\n", addr);
		return;
	}

	i3cdev_data = i3c_dev_get_master_data(i3cdev);
	if (!i3cdev_data) {
		dev_err(ddata->dev, "IBI received but unattached target %#x\n", addr);
		return;
	}

	ibi_data = i3cdev_data->ibi_data;
	if (!ibi_data) {
		dev_err(ddata->dev, "IBI received but IBI disabled for target %#x\n", addr);
		return;
	}

	if (len > ibi_data->max_len) {
		dev_err(ddata->dev, "IBI payload size too big (%dB > %dB)\n",
			len, ibi_data->max_len);
		return;
	}

	ibi_data->ibi_slot = i3c_generic_ibi_get_free_slot(ibi_data->ibi_pool);
	if (!ibi_data->ibi_slot) {
		dev_err(ddata->dev, "No free slot for target %#x IBI\n", addr);
		return;
	}

	ibi_data->ibi_slot->len = len;
	p = ibi_data->ibi_slot->data;

	/*
	 * The I3C_IBIDR register is not a FIFO: it is a plain register in which the hardware stores
	 * the whole (up to 4-byte) IBI payload, the earliest received byte (mandatory data byte
	 * MDB) in IBIDB0[7:0] up to the latest one in IBIDB3[31:24].
	 * Read it once and unpack the number of bytes reported by IBIRDCNT.
	 */
	ibidr = readl_relaxed(ddata->base + STM32_I3C_IBIDR);
	for (i = 0; i < len; i++)
		p[i] = ibidr >> (8 * i);

	/* Hand the filled slot over to the core so the client gets notified */
	i3c_master_queue_ibi(i3cdev, ibi_data->ibi_slot);
}

static irqreturn_t stm32_i3c_irq_handler(int irq, void *dev_id)
{
	struct stm32_i3c_ddata *ddata = dev_id;
	struct stm32_i3c_xfer *cur_xfer;
	u32 evr, ier, rx_len = UINT_MAX;

	/* IRQF_ONESHOT ensures the IRQ line kept disabled until this handler has been run */

	evr = readl_relaxed(ddata->base + STM32_I3C_EVR);
	ier = readl_relaxed(ddata->base + STM32_I3C_IER);
	dev_dbg(ddata->dev, "%s events: %#x/%#x\n", __func__, evr, ier);

	evr &= ier;
	if (!evr)
		return IRQ_NONE;

	/* I3C specific requests */
	if (evr & I3C_EVR_IBIF) {
		dev_dbg(ddata->dev, "IBI request\n");
		stm32_i3c_master_handle_ibi(ddata);
	}

	if (evr & I3C_EVR_HJF) {
		dev_dbg(ddata->dev, "Hot-Join request\n");
		if (READ_ONCE(ddata->hotjoin_enabled))
			schedule_work(&ddata->hotjoin_work);
	}

	if (evr & I3C_EVR_CRF) {
		/* TODO Controller-role request */
		dev_err(ddata->dev, "Controller-role request not supported yet\n");
	}

	if (evr & I3C_EVR_RXTGTENDF) {
		u32 status = stm32_i3c_master_read_status(ddata);

		rx_len = FIELD_GET(I3C_SR_XDCNT, status);
		dev_dbg(ddata->dev, "Target ended read after %u byte(s)\n", rx_len);
	}

	/* FIFOs management */
	if (evr & I3C_EVR_CFNFF) {
		cur_xfer = ddata->xferqueue.cur_xfer;
		dev_dbg(ddata->dev, "Data word is to be written to C-FIFO? %s\n",
			cur_xfer ? "yes" : "no");
		if (cur_xfer && cur_xfer->cur_cmd + 1 < (int)cur_xfer->ncmds) {
			cur_xfer->cur_cmd++;
			writel_relaxed(cur_xfer->cmds[cur_xfer->cur_cmd].msg_cr,
				       ddata->base + STM32_I3C_CR);
		}
	}

	if (evr & I3C_EVR_RXFNEF) {
		dev_dbg(ddata->dev, "Data byte/word is to be read from RX-FIFO\n");
		stm32_i3c_master_read_rxfifo(ddata, rx_len);
	}

	if (evr & I3C_EVR_TXFNFF) {
		dev_dbg(ddata->dev, "Data byte/word is to be written to TX-FIFO\n");
		stm32_i3c_master_write_txfifo(ddata);
	}

	if (evr & I3C_EVR_SFNEF) {
		/* Never set if I3C_CFGR.SMODE = 0 */
		stm32_i3c_master_read_status(ddata);
	}

	/* End of frame management */
	if (evr & I3C_EVR_RXTGTENDF) {
		u32 id;
		int i;

		spin_lock(&ddata->xferqueue.lock);
		cur_xfer = ddata->xferqueue.cur_xfer;
		if (cur_xfer) {
			id = FIELD_GET(I3C_SR_MID, readl_relaxed(ddata->base + STM32_I3C_SR));
			cur_xfer->ret = -EMSGSIZE;
			for (i = id; i < cur_xfer->ncmds; i++)
				cur_xfer->cmds[i].error = I3C_ERROR_UNKNOWN;
		}
		spin_unlock(&ddata->xferqueue.lock);
	}

	if (evr & (I3C_EVR_FCF | I3C_EVR_ERRF)) {
		if (evr & I3C_EVR_ERRF)
			dev_dbg(ddata->dev, "Error\n");
		else
			dev_dbg(ddata->dev, "Frame complete\n");

		if (ddata->xferqueue.cur_xfer) {
			spin_lock(&ddata->xferqueue.lock);
			stm32_i3c_master_end_xfer_locked(ddata, evr);
			spin_unlock(&ddata->xferqueue.lock);
		} else {
			stm32_i3c_master_read_error(ddata);
		}
	}

	/*
	 * In Controller mode, only the flags listed in I3C_CEVR_CTRL_MASK can be cleared, all the
	 * other I3C_CEVR bits are reserved and must be kept at reset value.
	 * The FIFO flags are de-asserted by the hardware itself once the FIFOs have been serviced.
	 */
	writel_relaxed(evr & I3C_CEVR_CTRL_MASK, ddata->base + STM32_I3C_CEVR);

	return IRQ_HANDLED;
}

static int stm32_i3c_pure_timing_cfg(struct i3c_master_controller *m, unsigned long i3cclk_rate,
				     struct i2c_timings *t)
{
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct i3c_bus *bus = i3c_master_get_bus(m);
	u64 i3c_period_cycles, sclh_i3c_cycles, scll_pp_min_cycles, scll_od_min_cycles, cas;
	u32 sclhi3c, scllpp, scllod, sdahold, free, aval, timingr0, timingr1;
	unsigned long ti3cclk_ns;

	dev_dbg(ddata->dev, "mode=%u, i3c freq=%lu\n", bus->mode, bus->scl_rate.i3c);

	if (!i3cclk_rate || !bus->scl_rate.i3c)
		return -EINVAL;

	ti3cclk_ns = DIV_ROUND_CLOSEST_ULL(NANO, i3cclk_rate);
	/* Preserve a 50% duty cycle, rounding the period to whole clock cycles. */
	i3c_period_cycles = DIV_ROUND_UP_ULL(i3cclk_rate, bus->scl_rate.i3c);
	sclh_i3c_cycles = DIV_ROUND_CLOSEST_ULL(i3c_period_cycles, 2);
	sclh_i3c_cycles = max_t(u64, sclh_i3c_cycles,
				DIV_ROUND_UP_ULL((u64)I3C_BUS_THIGH_MIN_NS * i3cclk_rate, NANO));
	sclh_i3c_cycles -= DIV_ROUND_UP_ULL((u64)t->scl_rise_ns * i3cclk_rate, NANO);

	scll_pp_min_cycles = DIV_ROUND_UP_ULL((u64)I3C_BUS_TLOW_PP_MIN_NS * i3cclk_rate, NANO);
	if (i3c_period_cycles < sclh_i3c_cycles + scll_pp_min_cycles) {
		dev_err(ddata->dev, "Cannot meet I3C Push-Pull SCL timing at %luHz input clock\n",
			i3cclk_rate);
		return -EINVAL;
	}

	scll_od_min_cycles = DIV_ROUND_UP_ULL((u64)I3C_BUS_TLOW_OD_MIN_NS * i3cclk_rate, NANO);
	scll_od_min_cycles -= DIV_ROUND_UP_ULL((u64)t->scl_fall_ns * i3cclk_rate, NANO);

	sclhi3c = sclh_i3c_cycles - 1;
	scllpp = (i3c_period_cycles - sclh_i3c_cycles) - 1;
	scllod = max_t(u64, scll_od_min_cycles, i3c_period_cycles - sclh_i3c_cycles) - 1;

	/*
	 * By default, SDA generation is delayed from one I3CCLK half-period.
	 * For high I3CCLK frequencies, one I3CCLK period delay can be added by setting the bit
	 * I3C_TIMINGR1.SDA_HD.
	 * tHD_PPmin > 3ns <=> (0.5 * tI3CCLK) > 3ns <=> tI3CCLK > 2 * 3ns
	 */
	if (ti3cclk_ns > 2 * I3C_BUS_HD_PP_MIN_NS)
		sdahold = 0;
	else
		sdahold = 1;

	/*
	 * Bus free condition: tBUF = tCAS = [(FREE[6:0] + 1) * 2 - (0.5 + SDA_HD)] * tI3CCLK
	 * FREE[6:0] = (tCAS / tI3CCLK + 0.5 + SDA_HD) / 2) - 1
	 * FREE[6:0] = ((2 * (tCAS / tI3CCLK) + 1 + 2 * SDA_HD)) / 4) - 1
	 * tBUF/tCAS is the amount of time that the controller may take to generate the SCL clock
	 * (drive SCL Low).
	 */
	cas = DIV_ROUND_UP(I3C_BUS_TCAS_MIN_NS + t->scl_fall_ns, ti3cclk_ns);
	free = DIV_ROUND_UP(2 * cas + 1 + 2 * sdahold, 4) - 1;
	if (free > FIELD_MAX(I3C_TIMINGR1_FREE)) {
		dev_err(ddata->dev, "SCL fall time is too high\n");
		return -EINVAL;
	}

	/* Bus available condition : tAVAL = 1us, 1us = 1000ns */
	aval = DIV_ROUND_UP(I3C_BUS_TAVAL_MIN_NS, ti3cclk_ns) - 2;
	if (aval > FIELD_MAX(I3C_TIMINGR1_AVAL)) {
		dev_err(ddata->dev, "Cannot meet I3C tAVAL timing at %luHz input clock\n",
			i3cclk_rate);
		return -EINVAL;
	}

	timingr0 = FIELD_PREP(I3C_TIMINGR0_SCLL_PP, scllpp) |
		   FIELD_PREP(I3C_TIMINGR0_SCLH_I3C, sclhi3c) |
		   FIELD_PREP(I3C_TIMINGR0_SCLL_OD, scllod);
		   /* I2C SCL Clock High Period (SCLH_I2C) not used when I3C_BUS_MODE_PURE */

	if (scllpp != FIELD_GET(I3C_TIMINGR0_SCLL_PP, timingr0) ||
	    sclhi3c != FIELD_GET(I3C_TIMINGR0_SCLH_I3C, timingr0) ||
	    scllod != FIELD_GET(I3C_TIMINGR0_SCLL_OD, timingr0)) {
		dev_err(ddata->dev, "Cannot configure SCL at %luHz input clock\n", i3cclk_rate);
		return -EINVAL;
	}

	timingr1 = FIELD_PREP(I3C_TIMINGR1_AVAL, aval) |
		   FIELD_PREP(I3C_TIMINGR1_FREE, free) |
		   FIELD_PREP(I3C_TIMINGR1_SDA_HD, sdahold);

	writel_relaxed(timingr0, ddata->base + STM32_I3C_TIMINGR0);
	writel_relaxed(timingr1, ddata->base + STM32_I3C_TIMINGR1);
	writel_relaxed(0, ddata->base + STM32_I3C_TIMINGR2);
	ddata->timingr0_normal = timingr0;

	dev_dbg(ddata->dev, "%s: timingr0=%#x timingr1=%#x\n", __func__, timingr0, timingr1);

	return 0;
}

static int stm32_i3c_mixed_timing_cfg(struct i3c_master_controller *m, unsigned long i3cclk_rate,
				      struct i2c_timings *t)
{
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct i3c_bus *bus = i3c_master_get_bus(m);
	u64 i2c_period_cycles, sclh_i2c_cycles, scll_od_min_cycles, buf;
	u64 i3c_period_cycles, sclh_i3c_cycles, scll_pp_min_cycles;
	u32 thigh_min_ns, tlow_min_ns, tbuf_min_ns, sdahold, free, aval;
	u32 sclhi2c, scllod, sclhi3c, scllpp, timingr0, timingr1;
	unsigned long ti3cclk_ns;

	dev_dbg(ddata->dev, "mode=%u, i3c freq=%lu, i2c freq=%lu\n",
		bus->mode, bus->scl_rate.i3c, bus->scl_rate.i2c);

	if (!i3cclk_rate || !bus->scl_rate.i3c || !bus->scl_rate.i2c)
		return -EINVAL;

	ti3cclk_ns = DIV_ROUND_CLOSEST_ULL(NANO, i3cclk_rate);
	if (bus->scl_rate.i2c > I3C_BUS_I2C_FM_SCL_MAX_RATE) {
		tlow_min_ns = I3C_BUS_TLOW_MIXED_FMP_MIN_NS;
		thigh_min_ns = I3C_BUS_THIGH_MIXED_FMP_MIN_NS;
		tbuf_min_ns = I3C_BUS_TBUF_MIXED_FMP_MIN_NS;
	} else { /* Fast Mode */
		tlow_min_ns = I3C_BUS_TLOW_MIXED_FM_MIN_NS;
		thigh_min_ns = I3C_BUS_THIGH_MIXED_FM_MIN_NS;
		tbuf_min_ns = I3C_BUS_TBUF_MIXED_FM_MIN_NS;
	}

	/*
	 * Preserve the I2C timing duty cycle while enforcing the I2C minima.
	 * The OD low period is shared with I3C open-drain transfers.
	 */
	i2c_period_cycles = DIV_ROUND_CLOSEST_ULL(i3cclk_rate, bus->scl_rate.i2c);
	sclh_i2c_cycles = DIV_ROUND_CLOSEST_ULL(i2c_period_cycles * 40, 100);
	sclh_i2c_cycles = max_t(u64, sclh_i2c_cycles,
				DIV_ROUND_UP_ULL((u64)thigh_min_ns * i3cclk_rate, NANO));
	sclh_i2c_cycles -= DIV_ROUND_UP_ULL((u64)t->scl_rise_ns * i3cclk_rate, NANO);

	scll_od_min_cycles = DIV_ROUND_CLOSEST_ULL(i2c_period_cycles * 60, 100);
	scll_od_min_cycles = max_t(u64, scll_od_min_cycles,
				   DIV_ROUND_UP_ULL((u64)tlow_min_ns * i3cclk_rate, NANO));
	scll_od_min_cycles -= DIV_ROUND_UP_ULL((u64)t->scl_fall_ns * i3cclk_rate, NANO);

	/* Preserve a 50% duty cycle, rounding the period to whole clock cycles. */
	i3c_period_cycles = DIV_ROUND_UP_ULL(i3cclk_rate, bus->scl_rate.i3c);
	sclh_i3c_cycles = DIV_ROUND_CLOSEST_ULL(i3c_period_cycles, 2);
	sclh_i3c_cycles = max_t(u64, sclh_i3c_cycles,
				DIV_ROUND_UP_ULL((u64)I3C_BUS_THIGH_MIN_NS * i3cclk_rate, NANO));
	sclh_i3c_cycles -= DIV_ROUND_UP_ULL((u64)t->scl_rise_ns * i3cclk_rate, NANO);

	scll_pp_min_cycles = DIV_ROUND_UP_ULL((u64)I3C_BUS_TLOW_PP_MIN_NS * i3cclk_rate, NANO);
	if (i3c_period_cycles < sclh_i3c_cycles + scll_pp_min_cycles) {
		dev_err(ddata->dev, "Cannot meet I3C Push-Pull SCL timing at %luHz input clock\n",
			i3cclk_rate);
		return -EINVAL;
	}

	sclhi2c = sclh_i2c_cycles - 1;
	scllod = scll_od_min_cycles - 1;
	sclhi3c = sclh_i3c_cycles - 1;
	scllpp = (i3c_period_cycles - sclh_i3c_cycles) - 1;

	/*
	 * By default, SDA generation is delayed from one I3CCLK half-period.
	 * For high I3CCLK frequencies, one I3CCLK period delay can be added by setting the bit
	 * I3C_TIMINGR1.SDA_HD.
	 * tHD_PPmin > 3ns <=> (0.5 * tI3CCLK) > 3ns <=> tI3CCLK > 2 * 3ns
	 */
	if (ti3cclk_ns > 2 * I3C_BUS_HD_PP_MIN_NS)
		sdahold = 0;
	else
		sdahold = 1;

	/*
	 * Bus free condition: tBUF = tCAS = [(FREE[6:0] + 1) * 2 - (0.5 + SDA_HD)] * tI3CCLK
	 * FREE[6:0] = (tCAS / tI3CCLK + 0.5 + SDA_HD) / 2) - 1
	 * FREE[6:0] = ((2 * (tCAS / tI3CCLK) + 1 + 2 * SDA_HD)) / 4) - 1
	 * tBUF/tCAS is the amount of time that the controller may take to generate the SCL clock
	 * (drive SCL Low).
	 */
	buf = DIV_ROUND_UP(tbuf_min_ns + t->scl_fall_ns, ti3cclk_ns);
	free = DIV_ROUND_UP(2 * buf + 1 + 2 * sdahold, 4) - 1;
	if (free > FIELD_MAX(I3C_TIMINGR1_FREE)) {
		dev_err(ddata->dev, "SCL fall time is too high\n");
		return -EINVAL;
	}

	/* Bus available condition : tAVAL = 1us, 1us = 1000ns */
	aval = DIV_ROUND_UP(I3C_BUS_TAVAL_MIN_NS, ti3cclk_ns) - 2;
	if (aval > FIELD_MAX(I3C_TIMINGR1_AVAL)) {
		dev_err(ddata->dev, "Cannot meet I3C tAVAL timing at %luHz input clock\n",
			i3cclk_rate);
		return -EINVAL;
	}

	timingr0 = FIELD_PREP(I3C_TIMINGR0_SCLL_PP, scllpp) |
		   FIELD_PREP(I3C_TIMINGR0_SCLH_I3C, sclhi3c) |
		   FIELD_PREP(I3C_TIMINGR0_SCLL_OD, scllod) |
		   FIELD_PREP(I3C_TIMINGR0_SCLH_I2C, sclhi2c);

	if (scllpp != FIELD_GET(I3C_TIMINGR0_SCLL_PP, timingr0) ||
	    sclhi3c != FIELD_GET(I3C_TIMINGR0_SCLH_I3C, timingr0) ||
	    scllod != FIELD_GET(I3C_TIMINGR0_SCLL_OD, timingr0) ||
	    sclhi2c != FIELD_GET(I3C_TIMINGR0_SCLH_I2C, timingr0)) {
		dev_err(ddata->dev, "Cannot configure SCL at %luHz input clock\n", i3cclk_rate);
		return -EINVAL;
	}

	timingr1 = FIELD_PREP(I3C_TIMINGR1_AVAL, aval) |
		   FIELD_PREP(I3C_TIMINGR1_FREE, free) |
		   FIELD_PREP(I3C_TIMINGR1_SDA_HD, sdahold);

	writel_relaxed(timingr0, ddata->base + STM32_I3C_TIMINGR0);
	writel_relaxed(timingr1, ddata->base + STM32_I3C_TIMINGR1);
	writel_relaxed(0, ddata->base + STM32_I3C_TIMINGR2);
	ddata->timingr0_normal = timingr0;

	dev_dbg(ddata->dev, "%s: timingr0=%#x timingr1=%#x\n", __func__, timingr0, timingr1);

	return 0;
}

static int stm32_i3c_master_bus_init(struct i3c_master_controller *m)
{
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct i3c_bus *bus = i3c_master_get_bus(m);
	struct i3c_device_info info = {};
	unsigned long ti3cclk_rate;
	struct i2c_timings timings, *t = &timings;
	struct pinctrl *pctl;
	u32 cfgr, ier;
	int ret;

	ret = pm_runtime_resume_and_get(ddata->dev);
	if (ret) {
		dev_err(ddata->dev, "<%s> cannot resume i3c bus master, err: %d\n",
			__func__, ret);
		return ret;
	}

	ti3cclk_rate = clk_get_rate(ddata->clk);
	if (!ti3cclk_rate) {
		ret = -EINVAL;
		goto out;
	}

	/* Set I3C as controller */
	cfgr = I3C_CFGR_CRINIT;

	/* Enable/disable SDA high keeper */
	cfgr |= I3C_CFGR_HKSDAEN; //TODO: when using it?

	/*
	 * The I3C node may specify i3c-scl-hz and i2c-scl-hz. Since i2c_parse_fw_timings()
	 * does not populate timings.bus_freq_hz from i2c-scl-hz, initialize it from
	 * m->bus.scl_rate.[i2c|i3c] and initialize the default values for I3C Pure and Mixed modes;
	 * firmware timing properties can override them.
	 */
	switch (bus->mode) {
	case I3C_BUS_MODE_PURE:
		t->bus_freq_hz = m->bus.scl_rate.i3c;
		t->scl_fall_ns = 35,
		t->scl_rise_ns = 0,
		i2c_parse_fw_timings(ddata->dev, t, false);
		ret = stm32_i3c_pure_timing_cfg(m, ti3cclk_rate, t);
		break;
	case I3C_BUS_MODE_MIXED_FAST:
	case I3C_BUS_MODE_MIXED_LIMITED:
	case I3C_BUS_MODE_MIXED_SLOW:
		t->bus_freq_hz = m->bus.scl_rate.i2c,
		t->scl_fall_ns = 35,
		t->scl_rise_ns = 0,
		i2c_parse_fw_timings(ddata->dev, t, false);
		ret = stm32_i3c_mixed_timing_cfg(m, ti3cclk_rate, t);
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	if (ret)
		goto out;

	/* Set own dynamic address */
	ret = i3c_master_get_free_addr(m, 0);
	if (ret < 0)
		goto out;

	info.dyn_addr = ret;

	writel_relaxed(FIELD_PREP(I3C_DEVR0_DA, info.dyn_addr), ddata->base + STM32_I3C_DEVR0);

	ret = i3c_master_set_info(&ddata->master, &info);
	if (ret)
		goto out;

	/* TODO
	 * Configure controller features in I3C_CFGR
	 * DMA for FIFOs: TXDMAEN, CDMAEN, RXDMAEN, SDMAEN
	 * TX/RX-FIFO threshold: TXTHRES, RXTHRES
	 * TX-FIFO and C-FIFO preload: TMODE
	 * S-FIFO: SMODE
	 * Hot-join Ack: HJACK
	 * HDR exit pattern: EXITPTRN
	 * HDR reset pattern: RSTPTRN
	 * arbitrable header: NOARBH
	 */
	//cfgr |= I3C_CFGR_TMODE;
	//cfgr |= I3C_CFGR_SMODE;

	ier = I3C_IER_CTRL_IE; /* FCIE, TXTGTENDIE, ERRIE, IBIIE, CRIE, HJIE */
	ier |= I3C_IER_FIFO_IE; /* CFNFIE, SFNEIE, TXFNFIE, RXFNEIE */
	if (!(cfgr & I3C_CFGR_SMODE))
		ier &= ~I3C_IER_SFNEIE;
	writel_relaxed(ier, ddata->base + STM32_I3C_IER);

	/* Move from I3C state = DISABLED to I3C state = IDLE, safely with init pin configuration */
	writel_relaxed(cfgr | I3C_CFGR_EN, ddata->base + STM32_I3C_CFGR);

	/* Anticipate default pin configuration needed for transfers done in i3c_master_bus_init */
	pctl = devm_pinctrl_get_select_default(ddata->dev);
	if (IS_ERR(pctl))
		dev_warn(ddata->dev, "Pins are not well configured\n");

out:
	pm_runtime_put_autosuspend(ddata->dev);
	return ret;
}

static void stm32_i3c_master_bus_cleanup(struct i3c_master_controller *m)
{
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	int ret;

	ret = pm_runtime_resume_and_get(ddata->dev);
	if (ret) {
		dev_err(ddata->dev, "<%s> cannot resume i3c bus master, err: %d\n",
			__func__, ret);
		return;
	}

	/* TODO: all the possible target requests must be disable using DISEC CCC */

	/* Move to I3C state = DISABLED */
	writel_relaxed(0, ddata->base + STM32_I3C_CFGR);

	/* Disable interrupts */
	writel_relaxed(0, ddata->base + STM32_I3C_IER);

	pm_runtime_put_autosuspend(ddata->dev);
}

static int stm32_i3c_master_enable_hotjoin(struct i3c_master_controller *m)
{
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	u32 cfgr;
	int ret;

	ret = pm_runtime_resume_and_get(ddata->dev);
	if (ret)
		return ret;

	cfgr = readl_relaxed(ddata->base + STM32_I3C_CFGR);
	writel_relaxed(cfgr | I3C_CFGR_HJACK, ddata->base + STM32_I3C_CFGR);
	WRITE_ONCE(ddata->hotjoin_enabled, true);

	return 0;
}

static int stm32_i3c_master_disable_hotjoin(struct i3c_master_controller *m)
{
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	u32 cfgr;

	WRITE_ONCE(ddata->hotjoin_enabled, false);
	cfgr = readl_relaxed(ddata->base + STM32_I3C_CFGR);
	writel_relaxed(cfgr & ~I3C_CFGR_HJACK, ddata->base + STM32_I3C_CFGR);
	pm_runtime_put_autosuspend(ddata->dev);

	return 0;
}

static int stm32_i3c_master_get_slot(struct stm32_i3c_ddata *ddata)
{
	u32 slot;

	slot = find_first_zero_bit(ddata->slots, ddata->nslots);
	if (slot >= ddata->nslots)
		return -ENXIO;
	__set_bit(slot, ddata->slots);

	return slot;
}

static void stm32_i3c_master_put_slot(struct stm32_i3c_ddata *ddata, u32 slot)
{
	__clear_bit(slot, ddata->slots);
}

static int stm32_i3c_master_attach_i3c_dev(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct stm32_i3c_dev_data *dev_data;
	int slot, ret;
	u32 devr;

	slot = stm32_i3c_master_get_slot(ddata);
	if (slot < 0)
		return slot;

	dev_data = kzalloc(sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data) {
		stm32_i3c_master_put_slot(ddata, slot);
		return -ENOMEM;
	}

	dev_dbg(ddata->dev, "%s: idx=%d addr=%#x bcr=%#x\n",
		__func__, slot, dev->info.dyn_addr ? : dev->info.static_addr, dev->info.bcr);

	dev_data->dev_idx = slot;

	ret = pm_runtime_resume_and_get(ddata->dev);
	if (ret) {
		dev_err(ddata->dev, "<%s> cannot resume i3c bus master, err: %d\n",
			__func__, ret);
		kfree(dev_data);
		stm32_i3c_master_put_slot(ddata, slot);
		return ret;
	}

	devr = FIELD_PREP(I3C_DEVR_DA, dev->info.dyn_addr ? : dev->info.static_addr);
	writel_relaxed(devr, ddata->base + STM32_I3C_DEVR(slot));

	pm_runtime_put_autosuspend(ddata->dev);

	i3c_dev_set_master_data(dev, dev_data);

	return 0;
}

static int stm32_i3c_master_reattach_i3c_dev(struct i3c_dev_desc *dev, u8 old_dyn_addr)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct stm32_i3c_dev_data *dev_data = i3c_dev_get_master_data(dev);
	u32 devr;
	int ret;

	dev_dbg(ddata->dev, "%s: idx=%d addr=%#x\n",
		__func__, dev_data->dev_idx, dev->info.dyn_addr ? : dev->info.static_addr);

	ret = pm_runtime_resume_and_get(ddata->dev);
	if (ret) {
		dev_err(ddata->dev, "<%s> cannot resume i3c bus master, err: %d\n",
			__func__, ret);
		return ret;
	}

	/* Preserve the IBI/controller-role acknowledge policy of the target */
	devr = readl_relaxed(ddata->base + STM32_I3C_DEVR(dev_data->dev_idx));
	devr &= ~I3C_DEVR_DA;
	devr |= FIELD_PREP(I3C_DEVR_DA, dev->info.dyn_addr ? : dev->info.static_addr);
	writel_relaxed(devr, ddata->base + STM32_I3C_DEVR(dev_data->dev_idx));

	pm_runtime_put_autosuspend(ddata->dev);

	return 0;
}

static void stm32_i3c_master_detach_i3c_dev(struct i3c_dev_desc *dev)
{
	struct stm32_i3c_dev_data *dev_data = i3c_dev_get_master_data(dev);
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	int ret;

	dev_dbg(ddata->dev, "%s: idx=%d addr=%#x\n",
		__func__, dev_data->dev_idx, dev->info.dyn_addr ? : dev->info.static_addr);

	ret = pm_runtime_resume_and_get(ddata->dev);
	if (ret) {
		dev_err(ddata->dev, "<%s> cannot resume i3c bus master, err: %d\n",
			__func__, ret);
		return;
	}

	/* Release the slot: clears DA[6:0] but also IBIACK, CRACK, IBIDEN and SUSP */
	writel_relaxed(0, ddata->base + STM32_I3C_DEVR(dev_data->dev_idx));

	pm_runtime_put_autosuspend(ddata->dev);

	stm32_i3c_master_put_slot(ddata, dev_data->dev_idx);

	kfree(dev_data);
}

static int stm32_i3c_master_entdaa(struct stm32_i3c_ddata *ddata, u8 *addrs, unsigned int max_count,
				   unsigned int *count)
{
	u32 ier, cfgr, cr, evr, sr, ser;
	unsigned int _count = 0;
	int ret;

	ret = pm_runtime_resume_and_get(ddata->dev);
	if (ret) {
		dev_err(ddata->dev, "<%s> cannot resume i3c bus master, err: %d\n",
			__func__, ret);
		return ret;
	}

	cr = FIELD_PREP(I3C_CR_MTYPE, MTYPE_I3C_CCC_CMD) |
	     FIELD_PREP(I3C_CR_CCC, I3C_CCC_ENTDAA) |
	     I3C_CR_MEND;

	/* Disable interrupts: ENTDAA is managed in polling mode */
	ier = readl_relaxed(ddata->base + STM32_I3C_IER);
	writel_relaxed(0, ddata->base + STM32_I3C_IER);

	/* Set 1-word/4-bytes RX FIFO threshold */
	cfgr = readl_relaxed(ddata->base + STM32_I3C_CFGR);
	writel_relaxed(cfgr | I3C_CFGR_RXTHRES, ddata->base + STM32_I3C_CFGR);

	stm32_i3c_master_set_arb_header(ddata, true);
	stm32_i3c_master_set_fifo_preload(ddata, false);

	/* Send ENTDAA CCC */
	ret = readl_relaxed_poll_timeout(ddata->base + STM32_I3C_EVR, evr,
					 evr & I3C_EVR_CFEF, 0, 1000);
	if (ret) {
		dev_err(ddata->dev, "C-FIFO not empty: evr=%#x\n", evr);
		goto abort_restore_regs;
	}
	writel_relaxed(cr, ddata->base + STM32_I3C_CR);

	/*
	 * While DAA process is not over, read 48-bit provisional ID + BCR and DCR,
	 * then write addr 1 byte.
	 */
	evr = readl_relaxed(ddata->base + STM32_I3C_EVR);
	while (!(evr & I3C_EVR_FCF) && _count < max_count) {
		u64 payload;

		/* A specific HW mechanism set TXFNFF=1 when RX-FIFO is empty & ENTDAA */
		ret = readl_relaxed_poll_timeout(ddata->base + STM32_I3C_EVR, evr,
						 evr & (I3C_EVR_TXFNFF | I3C_EVR_FCF), 0, 1000000);
		if (ret || (evr & I3C_EVR_FCF)) {
			if (ret)
				dev_dbg(ddata->dev,
					"RX-FIFO empty: evr=%#x, count=%d\n", evr, _count);
			goto exit_daa;
		}

		/* 48-bit Provisioned ID followed by BCR and DCR (using Big Endian bit order) */
		payload = readl_relaxed(ddata->base + STM32_I3C_RDWR);
		payload |= ((u64)readl_relaxed(ddata->base + STM32_I3C_RDWR)) << 32;
		dev_dbg(ddata->dev, "target#%d payload=%#llx", _count, be64_to_cpu(payload));

		/* Assign address to winning target */
		ret = readl_relaxed_poll_timeout(ddata->base + STM32_I3C_EVR, evr,
						 evr & (I3C_EVR_TXFNFF | I3C_EVR_FCF), 0, 1000000);
		if (ret || (evr & I3C_EVR_FCF)) {
			if (ret)
				dev_dbg(ddata->dev,
					"TX-FIFO full: evr=%#x count=%d\n", evr, _count);
			goto exit_daa;
		}

		/*
		 * HW automatically performs a single-retry if addr is NACK-ed for the first time,
		 * or automatically emits a stop if addr is NACK-ed for the second time.
		 */
		writeb_relaxed(addrs[_count], ddata->base + STM32_I3C_TDR);
		_count++;
	}
exit_daa:
	/* Wait on DAA end until timeout */
	ret = readl_relaxed_poll_timeout(ddata->base + STM32_I3C_EVR, evr,
					 evr & (I3C_EVR_FCF | I3C_EVR_ERRF), 0, 1000000);

	sr = stm32_i3c_master_read_status(ddata);
	ser = stm32_i3c_master_read_error(ddata);

	if (ret) {
		u32 coderr = FIELD_GET(I3C_SER_CODERR, ser);

		if (coderr == CODERR_CE1)
			stm32_i3c_master_send_stop(ddata);
		if (coderr == CODERR_CE2)
			ret = 0;
	}

	/* Clear flags */
	writel_relaxed(evr & I3C_CEVR_CTRL_MASK, ddata->base + STM32_I3C_CEVR);

	stm32_i3c_master_fifo_flush(ddata);

	if (_count != FIELD_GET(I3C_SR_XDCNT, sr)) {
		dev_err(ddata->dev, "%u address(es) sent but %lu target(s) detected\n",
			_count, FIELD_GET(I3C_SR_XDCNT, sr));
		ret = -EINVAL;
	} else {
		dev_dbg(ddata->dev, "%u target(s) assigned\n", _count);
	}

abort_restore_regs:
	writel_relaxed(cfgr, ddata->base + STM32_I3C_CFGR);
	writel_relaxed(ier, ddata->base + STM32_I3C_IER);
	*count = _count;
	pm_runtime_put_autosuspend(ddata->dev);

	return ret;
}

static int stm32_i3c_master_do_daa(struct i3c_master_controller *m)
{
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	unsigned int count = 0, nb_reserved = 0, nb_assigned = 0, i;
	u8 next_addr = 0, *addrs;
	int *slots, ret;

	addrs = kcalloc(ddata->nslots, sizeof(*addrs), GFP_KERNEL);
	if (!addrs)
		return -ENOMEM;

	slots = kcalloc(ddata->nslots, sizeof(*slots), GFP_KERNEL);
	if (!slots) {
		kfree(addrs);
		return -ENOMEM;
	}

	for (;;) {
		ret = stm32_i3c_master_get_slot(ddata);
		if (ret < 0)
			break;
		slots[nb_reserved++] = ret;

		ret = i3c_master_get_free_addr(m, next_addr + 1);
		if (ret < 0)
			break;
		next_addr = ret;

		dev_dbg(&m->dev, "next_addr = 0x%02x, DAA using #%d", next_addr,
			slots[nb_reserved - 1]);

		addrs[count++] = next_addr;
	}

	if (!count) {
		dev_err(&m->dev, "No free slot for device\n");
		ret = -ENOSPC;
		for (i = 0; i < nb_reserved; i++)
			stm32_i3c_master_put_slot(ddata, slots[i]);
		goto free_mem;
	}

	ret = stm32_i3c_master_entdaa(ddata, addrs, count, &nb_assigned);

	for (i = 0; i < nb_reserved; i++)
		stm32_i3c_master_put_slot(ddata, slots[i]);

	if (ret)
		goto free_mem;

	/* Register devices to the core */
	for (i = 0; i < nb_assigned; i++)
		i3c_master_add_i3c_dev_locked(m, addrs[i]);

free_mem:
	kfree(slots);
	kfree(addrs);

	return ret;
}

/* Optional stm32_i3c_master_supports_ccc_cmd */
static bool stm32_i3c_master_supports_ccc_cmd(struct i3c_master_controller *m,
					      const struct i3c_ccc_cmd *cmd)
{
	/*
	 * TODO fwk limitation or HW limitation?
	 *  @ndests: number of destinations. Should always be one for broadcast commands
	 */
	if (cmd->ndests > 1)
		return false;

	switch (cmd->id) {
	case I3C_CCC_ENEC(true):			//0x00
	case I3C_CCC_DISEC(true):			//0x01
	case I3C_CCC_ENTAS(0, true):			//0x02
	case I3C_CCC_ENTAS(1, true):			//0x03
	case I3C_CCC_ENTAS(2, true):			//0x04
	case I3C_CCC_ENTAS(3, true):			//0x05
	case I3C_CCC_RSTDAA(true):			//0x06
	case I3C_CCC_ENTDAA:				//0x07
	case I3C_CCC_DEFSLVS:				//0x08 (DEFTGTS)
	case I3C_CCC_SETMWL(true):			//0x09
	case I3C_CCC_SETMRL(true):			//0x0A
	case I3C_CCC_ENTTM:				//0x0B
	case I3C_CCC_SETXTIME(true):			//0x28
	//case I3C_CCC_RSTACT(true):			//0x2A

	case I3C_CCC_ENEC(false):			//0x80
	case I3C_CCC_DISEC(false):			//0x81
	case I3C_CCC_ENTAS(0, false):			//0x82
	case I3C_CCC_ENTAS(1, false):			//0x83
	case I3C_CCC_ENTAS(2, false):			//0x84
	case I3C_CCC_ENTAS(3, false):			//0x85
	case I3C_CCC_SETDASA:				//0x87
	case I3C_CCC_SETNEWDA:				//0x88
	case I3C_CCC_SETMWL(false):			//0x89
	case I3C_CCC_SETMRL(false):			//0x8A
	case I3C_CCC_GETMWL:				//0x8B
	case I3C_CCC_GETMRL:				//0x8C
	case I3C_CCC_GETPID:				//0x8D
	case I3C_CCC_GETBCR:				//0x8E
	case I3C_CCC_GETDCR:				//0x8F
	case I3C_CCC_GETSTATUS:				//0x90
	case I3C_CCC_GETACCMST: //GETACCCR		//0x91
	case I3C_CCC_GETMXDS:				//0x94
	case I3C_CCC_GETHDRCAP: //GETCAPS		//0x95
	case I3C_CCC_SETXTIME(false):			//0x98
	case I3C_CCC_GETXTIME:				//0x99
	//case I3C_CCC_RSTACT(false):			//0x9A
		return true;
	default:
	//case I3C_CCC_ENTHDR(x) not supported		//0x20+(x)
	//case I3C_CCC_SETAASA				//0x29
	//case I3C_CCC_DEFGRPA(true)			//0x2B
	//case I3C_CCC_RSTGRPA(true)			//0x2C
	//case I3C_CCC_VENDOR(id, true) not supported	//0x61

	//case I3C_CCC_RSTDAA(false) not supported	//0x86
	//case I3C_CCC_SETBRGTGT not supported		//0x93
	//case I3C_CCC_D2DXFER?				//0x97
	//case I3C_CCC_DEFGRPA/SETGRPA(false)?		//0x9B
	//case I3C_CCC_RSTGRPA(false)?			//0x9C
	//case I3C_CCC_VENDOR(id, false) not supported	//0xE0
		return false;
	}
}

static int stm32_i3c_master_send_bdcast_ccc_cmd(struct stm32_i3c_ddata *ddata,
						struct i3c_ccc_cmd *cmd)
{
	struct stm32_i3c_xfer *xfer;
	struct stm32_i3c_cmd *_cmd;
	int ret;

	ret = pm_runtime_resume_and_get(ddata->dev);
	if (ret) {
		dev_err(ddata->dev, "<%s> cannot resume i3c bus master, err: %d\n",
			__func__, ret);
		return ret;
	}

	xfer = stm32_i3c_master_alloc_xfer(ddata, 1);
	if (!xfer) {
		ret = -ENOMEM;
		goto abort_bdcast_ccc;
	}

	_cmd = xfer->cmds;
	_cmd->msg_cr = FIELD_PREP(I3C_CR_MTYPE, MTYPE_I3C_CCC_CMD) |
		       FIELD_PREP(I3C_CR_CCC, cmd->id) | I3C_CR_MEND;
	_cmd->data_len = cmd->dests[0].payload.len;
	_cmd->data.tx = cmd->dests[0].payload.data;
	_cmd->msg_cr |= FIELD_PREP(I3C_CR_DCNT, _cmd->data_len);

	stm32_i3c_master_queue_xfer(ddata, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, msecs_to_jiffies(1000)))
		stm32_i3c_master_abort_xfer(ddata, xfer);

	ret = xfer->ret;
	cmd->err = xfer->cmds->error;

	stm32_i3c_master_free_xfer(xfer);

abort_bdcast_ccc:
	pm_runtime_put_autosuspend(ddata->dev);

	return ret;
}

static int stm32_i3c_master_send_direct_ccc_cmd(struct stm32_i3c_ddata *ddata,
						struct i3c_ccc_cmd *cmd)
{
	struct stm32_i3c_xfer *xfer;
	struct stm32_i3c_cmd *_cmd;
	int ret;

	ret = pm_runtime_resume_and_get(ddata->dev);
	if (ret) {
		dev_err(ddata->dev, "<%s> cannot resume i3c bus master, err: %d\n",
			__func__, ret);
		return ret;
	}

	xfer = stm32_i3c_master_alloc_xfer(ddata, 2);
	if (!xfer) {
		ret = -ENOMEM;
		goto abort_direct_ccc;
	}

	/* Broadcasted message: the CCC code itself, without CCC defining byte */
	_cmd = &xfer->cmds[0];
	_cmd->msg_cr = FIELD_PREP(I3C_CR_MTYPE, MTYPE_I3C_CCC_CMD) |
		       FIELD_PREP(I3C_CR_CCC, cmd->id);
	_cmd->data_len = 0;

	/* Directed message */
	_cmd = &xfer->cmds[1];
	_cmd->msg_cr = FIELD_PREP(I3C_CR_MTYPE, MTYPE_I3C_DIRECT_MSG) |
		       FIELD_PREP(I3C_CR_ADD, cmd->dests[0].addr);
	_cmd->data_len = cmd->dests[0].payload.len;
	_cmd->msg_cr |= FIELD_PREP(I3C_CR_DCNT, _cmd->data_len);
	if (cmd->rnw) {
		_cmd->msg_cr |= I3C_CR_RNW;
		_cmd->data.rx = cmd->dests[0].payload.data;
	} else {
		_cmd->data.tx = cmd->dests[0].payload.data;
	}
	_cmd->msg_cr |= I3C_CR_MEND;

	stm32_i3c_master_queue_xfer(ddata, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, msecs_to_jiffies(1000)))
		stm32_i3c_master_abort_xfer(ddata, xfer);

	ret = xfer->ret;
	cmd->err = xfer->cmds[1].error;

	stm32_i3c_master_free_xfer(xfer);

abort_direct_ccc:
	pm_runtime_put_autosuspend(ddata->dev);

	return ret;
}

/* TODO: to remove when mature */
static char *ccc_to_string(u8 id)
{
	switch (id) {
	case I3C_CCC_ENEC(true):
	case I3C_CCC_ENEC(false):
		return "ENEC";
	case I3C_CCC_DISEC(true):
	case I3C_CCC_DISEC(false):
		return "DISEC";
	case I3C_CCC_ENTAS(0, true):
	case I3C_CCC_ENTAS(1, true):
	case I3C_CCC_ENTAS(2, true):
	case I3C_CCC_ENTAS(3, true):
	case I3C_CCC_ENTAS(0, false):
	case I3C_CCC_ENTAS(1, false):
	case I3C_CCC_ENTAS(2, false):
	case I3C_CCC_ENTAS(3, false):
		return "ENTAS";
	case I3C_CCC_RSTDAA(true):
	case I3C_CCC_RSTDAA(false):
		return "RSTDAA";
	case I3C_CCC_SETMWL(true):
	case I3C_CCC_SETMWL(false):
		return "SETMWL";
	case I3C_CCC_SETMRL(true):
	case I3C_CCC_SETMRL(false):
		return "SETMRL";
	case I3C_CCC_SETXTIME(true):
	case I3C_CCC_SETXTIME(false):
		return "SETXTIME";
	//case I3C_CCC_VENDOR(id, broadcast):
	case I3C_CCC_ENTDAA:
		return "ENTDAA";
	case I3C_CCC_DEFSLVS:
		return "DEFSLVS";
	case I3C_CCC_ENTTM:
		return "ENTTM";
	case I3C_CCC_SETDASA:
		return "SETDASA";
	case I3C_CCC_SETNEWDA:
		return "SETNEWDA";
	case I3C_CCC_GETMWL:
		return "GETMWL";
	case I3C_CCC_GETMRL:
		return "GETMRL";
	case I3C_CCC_GETPID:
		return "GETPID";
	case I3C_CCC_GETBCR:
		return "GETBCR";
	case I3C_CCC_GETDCR:
		return "GETDCR";
	case I3C_CCC_GETSTATUS:
		return "GETSTATUS";
	case I3C_CCC_GETACCMST:
		return "GETACCMST";
	case I3C_CCC_GETMXDS:
		return "GETMXDS";
	case I3C_CCC_GETHDRCAP:
		return "GETHDRCAP";
	case I3C_CCC_GETXTIME:
		return "GETXTIME";
	default:
		return "not supported";
	}
}

static int stm32_i3c_master_send_ccc_cmd(struct i3c_master_controller *m, struct i3c_ccc_cmd *cmd)
{
	/*
	 * cmd->id & I3C_CCC_DIRECT => CCC[7] = 1 => I3C SDR direct CCC command
	 * !(cmd->id & I3C_CCC_DIRECT) => CCC[7] = 0 => I3C SDR broadcast CCC command
	 */
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	bool direct = cmd->id & I3C_CCC_DIRECT;

	dev_dbg(&m->dev, "%s: id=%s(%#x), rnw=%u, ndest=%u, dest.addr=%#x, dest.payload.len=%d\n",
		__func__, ccc_to_string(cmd->id), cmd->id, cmd->rnw, cmd->ndests,
		cmd->dests[0].addr, cmd->dests[0].payload.len);

	if (direct)
		return stm32_i3c_master_send_direct_ccc_cmd(ddata, cmd);
	else
		return stm32_i3c_master_send_bdcast_ccc_cmd(ddata, cmd);
}

static int stm32_i3c_master_priv_xfers(struct i3c_dev_desc *dev, struct i3c_priv_xfer *xfers,
				       int nxfers)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct stm32_i3c_xfer *xfer;
	int i, ret;

	ret = pm_runtime_resume_and_get(ddata->dev);
	if (ret) {
		dev_err(ddata->dev, "<%s> cannot resume i3c bus master, err: %d\n",
			__func__, ret);
		return ret;
	}

	xfer = stm32_i3c_master_alloc_xfer(ddata, nxfers);
	if (!xfer) {
		ret = -ENOMEM;
		goto abort_priv_xfers;
	}

	for (i = 0; i < nxfers; i++) {
		struct stm32_i3c_cmd *_cmd = &xfer->cmds[i];

		_cmd->msg_cr = FIELD_PREP(I3C_CR_MTYPE, MTYPE_I3C_PRIV_MSG);
		_cmd->msg_cr |= FIELD_PREP(I3C_CR_ADD, dev->info.dyn_addr);
		_cmd->msg_cr |= FIELD_PREP(I3C_CR_DCNT, xfers[i].len);
		_cmd->data_len = xfers[i].len;

		if (xfers[i].rnw) {
			_cmd->msg_cr |= I3C_CR_RNW;
			_cmd->data.rx = xfers[i].data.in;
		} else {
			_cmd->data.tx = xfers[i].data.out;
		}

		if (i + 1 == nxfers) /* Last */
			_cmd->msg_cr |= I3C_CR_MEND;
	}

	stm32_i3c_master_queue_xfer(ddata, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, msecs_to_jiffies(1000)))
		stm32_i3c_master_abort_xfer(ddata, xfer);

	ret = xfer->ret;

	stm32_i3c_master_free_xfer(xfer);

abort_priv_xfers:
	pm_runtime_put_autosuspend(ddata->dev);

	return ret;
}

static int stm32_i3c_master_i2c_xfers(struct i2c_dev_desc *dev, struct i2c_msg *xfers,
				      int nxfers)
{
	struct i3c_master_controller *m = i2c_dev_get_master(dev);
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct stm32_i3c_xfer *xfer;
	int i, ret;

	dev_dbg(&m->dev, "%s: nxfers=%i, rnw=%u, dest_addr=%x, dest_payload_len=%d\n",
		__func__, nxfers, xfers[0].flags & I2C_M_RD, xfers[0].addr, xfers[0].len);

	/* length must be non-null for a legacy I2C message */
	for (i = 0; i < nxfers; i++) {
		if (!xfers[i].len)
			return -EOPNOTSUPP;
	}

	ret = pm_runtime_resume_and_get(ddata->dev);
	if (ret) {
		dev_err(ddata->dev, "<%s> cannot resume i3c bus master, err: %d\n",
			__func__, ret);
		return ret;
	}

	xfer = stm32_i3c_master_alloc_xfer(ddata, nxfers);
	if (!xfer) {
		ret = -ENOMEM;
		goto abort_i2c_xfers;
	}

	for (i = 0; i < nxfers; i++) {
		struct stm32_i3c_cmd *_cmd = &xfer->cmds[i];

		_cmd->msg_cr = FIELD_PREP(I3C_CR_MTYPE, MTYPE_I2C_MSG);
		_cmd->msg_cr |= FIELD_PREP(I3C_CR_ADD, xfers[i].addr);
		_cmd->msg_cr |= FIELD_PREP(I3C_CR_DCNT, xfers[i].len);
		_cmd->data_len = xfers[i].len;

		if (xfers[i].flags & I2C_M_RD) {
			_cmd->msg_cr |= I3C_CR_RNW;
			_cmd->data.rx = xfers[i].buf;

		} else {
			_cmd->data.tx = xfers[i].buf;
		}

		if (i + 1 == nxfers) /* Last */
			_cmd->msg_cr |= I3C_CR_MEND;
	}

	stm32_i3c_master_queue_xfer(ddata, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, msecs_to_jiffies(1000)))
		stm32_i3c_master_abort_xfer(ddata, xfer);

	ret = xfer->ret;

	stm32_i3c_master_free_xfer(xfer);

abort_i2c_xfers:
	pm_runtime_put_autosuspend(ddata->dev);

	return ret;
}

static int stm32_i3c_master_request_ibi(struct i3c_dev_desc *dev, const struct i3c_ibi_setup *req)
{
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(i3c_dev_get_master(dev));
	struct stm32_i3c_dev_data *dev_data = i3c_dev_get_master_data(dev);
	struct i3c_generic_ibi_pool *ibi_pool;
	struct stm32_i3c_dev_ibi_data *dev_ibi;

	dev_dbg(ddata->dev, "%s: idx=%d addr=%#x, ibi_len=%d\n", __func__, dev_data->dev_idx,
		dev->info.dyn_addr ? : dev->info.static_addr, dev->ibi->max_payload_len);

	if (dev->ibi->max_payload_len > I3C_IBI_MAX_PAYLOAD_LEN) {
		dev_err(ddata->dev, "IBI payload size too big (%dB > %dB)\n",
			dev->ibi->max_payload_len, I3C_IBI_MAX_PAYLOAD_LEN);
		return -EMSGSIZE;
	}

	dev_ibi = kmalloc(sizeof(*dev_ibi), GFP_KERNEL);
	if (!dev_ibi)
		return -ENOMEM;

	ibi_pool = i3c_generic_ibi_alloc_pool(dev, req);
	if (IS_ERR(ibi_pool)) {
		kfree(dev_ibi);
		return PTR_ERR(ibi_pool);
	}

	dev_ibi->ibi_pool = ibi_pool;
	dev_ibi->max_len = req->max_payload_len;
	dev_data->ibi_data = dev_ibi;

	return 0;
}

static void stm32_i3c_master_free_ibi(struct i3c_dev_desc *dev)
{
	struct stm32_i3c_dev_data *dev_data = i3c_dev_get_master_data(dev);
	struct stm32_i3c_dev_ibi_data *dev_ibi = dev_data->ibi_data;

	dev_data->ibi_data = NULL;
	i3c_generic_ibi_free_pool(dev_ibi->ibi_pool);
	kfree(dev_ibi);
}

static int stm32_i3c_master_enable_ibi(struct i3c_dev_desc *dev)
{
	struct stm32_i3c_dev_data *dev_data = i3c_dev_get_master_data(dev);
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	u32 devr;
	int ret;

	ret = pm_runtime_resume_and_get(ddata->dev);
	if (ret) {
		dev_err(ddata->dev, "<%s> cannot resume i3c bus master, err: %d\n",
			__func__, ret);
		return ret;
	}

	devr = readl_relaxed(ddata->base + STM32_I3C_DEVR(dev_data->dev_idx));
	devr |= I3C_DEVR_IBIACK;
	if (dev->info.bcr & I3C_BCR_IBI_PAYLOAD)
		devr |= I3C_DEVR_IBIDEN;
	else
		devr &= ~I3C_DEVR_IBIDEN;
	writel_relaxed(devr, ddata->base + STM32_I3C_DEVR(dev_data->dev_idx));

	ret = i3c_master_enec_locked(m, dev->info.dyn_addr, I3C_CCC_EVENT_SIR);
	if (ret) {
		devr &= ~(I3C_DEVR_IBIDEN | I3C_DEVR_IBIACK);
		writel_relaxed(devr, ddata->base + STM32_I3C_DEVR(dev_data->dev_idx));
		pm_runtime_put_autosuspend(ddata->dev);
	}

	return ret;
}

static int stm32_i3c_master_disable_ibi(struct i3c_dev_desc *dev)
{
	struct stm32_i3c_dev_data *dev_data = i3c_dev_get_master_data(dev);
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	u32 devr;
	int ret;

	/* Don't drop the runtime PM reference since IBI is not disabled and could still occur */
	ret = i3c_master_disec_locked(m, dev->info.dyn_addr, I3C_CCC_EVENT_SIR);
	if (ret)
		return ret;

	devr = readl_relaxed(ddata->base + STM32_I3C_DEVR(dev_data->dev_idx));
	devr &= ~(I3C_DEVR_IBIDEN | I3C_DEVR_IBIACK);
	writel_relaxed(devr, ddata->base + STM32_I3C_DEVR(dev_data->dev_idx));

	pm_runtime_put_autosuspend(ddata->dev);

	return 0;
}

static void stm32_i3c_master_recycle_ibi_slot(struct i3c_dev_desc *dev, struct i3c_ibi_slot *slot)
{
	struct stm32_i3c_dev_data *dev_data = i3c_dev_get_master_data(dev);
	struct stm32_i3c_dev_ibi_data *dev_ibi = dev_data->ibi_data;

	i3c_generic_ibi_recycle_slot(dev_ibi->ibi_pool, slot);
}

static const struct i3c_master_controller_ops stm32_i3c_master_ops = {
	.bus_init = stm32_i3c_master_bus_init,			/* Mandatory */
	.bus_cleanup = stm32_i3c_master_bus_cleanup,
	.attach_i3c_dev = stm32_i3c_master_attach_i3c_dev,
	.reattach_i3c_dev = stm32_i3c_master_reattach_i3c_dev,
	.detach_i3c_dev = stm32_i3c_master_detach_i3c_dev,
	.do_daa = stm32_i3c_master_do_daa,			/* Required */
	.supports_ccc_cmd = stm32_i3c_master_supports_ccc_cmd,
	.send_ccc_cmd = stm32_i3c_master_send_ccc_cmd,		/* Required */
	.priv_xfers = stm32_i3c_master_priv_xfers,		/* Required */
	.i2c_xfers = stm32_i3c_master_i2c_xfers,		/* Required */
	.request_ibi = stm32_i3c_master_request_ibi,		/* Optional but... */
	.free_ibi = stm32_i3c_master_free_ibi,			/* Required if .request_ibi */
	.enable_ibi = stm32_i3c_master_enable_ibi,		/* Required if .request_ibi */
	.disable_ibi = stm32_i3c_master_disable_ibi,		/* Required if .request_ibi */
	.recycle_ibi_slot = stm32_i3c_master_recycle_ibi_slot,	/* Required if .request_ibi */
	.enable_hotjoin = stm32_i3c_master_enable_hotjoin,
	.disable_hotjoin = stm32_i3c_master_disable_hotjoin,
	.set_speed = stm32_i3c_master_set_speed,
};

static int stm32_i3c_probe(struct platform_device *pdev)
{
	struct stm32_i3c_ddata *ddata;
	struct reset_control *rst;
	u32 hwcfgr, verr;
	int irq, ret;

	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ddata->base))
		return PTR_ERR(ddata->base);

	ddata->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(ddata->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(ddata->clk), "Failed to get clk\n");

	rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(rst), "Failed to get reset\n");

	INIT_LIST_HEAD(&ddata->xferqueue.list);
	spin_lock_init(&ddata->xferqueue.lock);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ddata->irq = irq;

	reset_control_reset(rst);

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL, stm32_i3c_irq_handler, IRQF_ONESHOT,
					dev_name(&pdev->dev), ddata);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to request IRQ\n");

	hwcfgr = readl_relaxed(ddata->base + STM32_I3C_HWCFGR);

	/* Number of I3C targets whose IBI/controller-role can be managed by this I3C controller */
	ddata->nslots = FIELD_GET(I3C_HWCFGR_NBT, hwcfgr);
	if (!ddata->nslots)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "Invalid hardware configuration (hwcfgr=%#x)\n", hwcfgr);

	ddata->slots = devm_bitmap_zalloc(&pdev->dev, ddata->nslots, GFP_KERNEL);
	if (!ddata->slots)
		return -ENOMEM;

	ddata->dev = &pdev->dev;
	INIT_WORK(&ddata->hotjoin_work, stm32_i3c_master_hotjoin_work);
	platform_set_drvdata(pdev, ddata);

	/* Read controller version before enabling autosuspend to ensure it is clocked */
	verr = readl_relaxed(ddata->base + STM32_I3C_VERR);

	pm_runtime_set_autosuspend_delay(&pdev->dev, STM32_I3C_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = i3c_master_register(&ddata->master, ddata->dev, &stm32_i3c_master_ops, false);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "Failed to register I3C master: %d\n", ret);
		goto err_disable_pm;
	}

	pm_runtime_put_autosuspend(&pdev->dev);

	dev_info(ddata->dev, "STM32 I3C registered rev:%lu.%lu\n",
		 FIELD_GET(I3C_VERR_MAJREV, verr), FIELD_GET(I3C_VERR_MINREV, verr));

	return 0;

err_disable_pm:
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);

	return ret;
}

static void stm32_i3c_remove(struct platform_device *pdev)
{
	struct stm32_i3c_ddata *ddata = platform_get_drvdata(pdev);

	cancel_work_sync(&ddata->hotjoin_work);
	pm_runtime_get_sync(&pdev->dev);

	i3c_master_unregister(&ddata->master);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
}

static int stm32_i3c_master_runtime_suspend(struct device *dev)
{
	struct stm32_i3c_ddata *ddata = dev_get_drvdata(dev);

	clk_disable(ddata->clk);
	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int stm32_i3c_master_runtime_resume(struct device *dev)
{
	struct stm32_i3c_ddata *ddata = dev_get_drvdata(dev);

	pinctrl_pm_select_default_state(dev);

	return clk_enable(ddata->clk);
}

static const struct dev_pm_ops stm32_i3c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(stm32_i3c_master_runtime_suspend, stm32_i3c_master_runtime_resume, NULL)
};

static const struct of_device_id stm32_i3c_of_match[] = {
	{ .compatible = "st,stm32mp25-i3c" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, stm32_i3c_of_match);

static struct platform_driver stm32_i3c_driver = {
	.probe = stm32_i3c_probe,
	.remove = stm32_i3c_remove,
	.driver = {
		.name = "stm32-i3c",
		.of_match_table = stm32_i3c_of_match,
		.pm = pm_sleep_ptr(&stm32_i3c_pm_ops),
	},
};
module_platform_driver(stm32_i3c_driver);

MODULE_DESCRIPTION("STM32 I3C controller driver");
MODULE_AUTHOR("Amelie Delaunay <amelie.delaunay@foss.st.com>");
MODULE_LICENSE("GPL");
