// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for STM32 Camera Serial Interface
 *
 * Copyright (C) STMicroelectronics SA 2021
 * Authors: Alain Volmat <alain.volmat@foss.st.com>
 *
 * This driver is based on cdns-csi2rx.c
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Uncomment the following define to show catch all CSI2HOST errors */
/* #define CSI2HOST_ERROR_HANDLING */

#define CSI_CR					0x0000
#define CSI_CR_CSIEN				BIT(0)
#define CSI_CR_VCxSTART(x)			BIT(2 + ((x) * 4))
#define CSI_CR_VCxSTOP(x)			BIT(3 + ((x) * 4))
#define CSI_PCR					0x0004
#define CSI_PCR_DL1EN				BIT(3)
#define CSI_PCR_DL0EN				BIT(2)
#define CSI_PCR_CLEN				BIT(1)
#define CSI_PCR_PWRDOWN				BIT(0)
#define CSI_VCxCFGR1(x)				((((x) + 1) * 0x0010) + 0x0)
#define CSI_VCxCFGR1_ALLDT			BIT(0)
#define CSI_VCxCFGR1_DT0EN			BIT(1)
#define CSI_VCxCFGR1_DT1EN			BIT(2)
#define CSI_VCxCFGR1_CDTFT_SHIFT		8
#define CSI_VCxCFGR1_DT0_SHIFT			16
#define CSI_VCxCFGR1_DT0FT_SHIFT		24
#define CSI_VCxCFGR2(x)				((((x) + 1) * 0x0010) + 0x4)
#define CSI_VCxCFGR2_DT1_SHIFT			0
#define CSI_VCxCFGR2_DT1FT_SHIFT		8
#define CSI_VCxCFGRx_DTxFT_BPP8			2
#define CSI_VCxCFGRx_DTxFT_BPP10		3
#define CSI_VCxCFGRx_DTxFT_BPP12		4
#define CSI_VCxCFGRx_DTxFT_BPP14		5
#define CSI_LMCFGR				0x0070
#define CSI_LMCFGR_LANENB_SHIFT			8
#define CSI_LMCFGR_LANENB_MASK			GENMASK(10, 8)
#define CSI_LMCFGR_DL0MAP_SHIFT			16
#define CSI_LMCFGR_DL0MAP_MASK			GENMASK(18, 16)
#define CSI_LMCFGR_DL1MAP_SHIFT			20
#define CSI_LMCFGR_DL1MAP_MASK			GENMASK(22, 20)
#define CSI_WDR					0x0078
#define CSI_IER0				0x0080
#define CSI_IER1				0x0084
#define CSI_SR0					0x0090
#define CSI_SR0_SYNCERRF			BIT(30)
#define CSI_SR0_SPKTERRF			BIT(28)
#define CSI_SR0_IDERRF				BIT(27)
#define CSI_SR0_CECCERRF			BIT(26)
#define CSI_SR0_ECCERRF				BIT(25)
#define CSI_SR0_CRCERRF				BIT(24)
#define CSI_SR0_CCFIFOFF			BIT(21)
#define CSI_SR0_VCxSTATEF(x)			BIT(17 + (x))
#define CSI_SR0_EOF3F				BIT(15)
#define CSI_SR0_EOF2F				BIT(14)
#define CSI_SR0_EOF1F				BIT(13)
#define CSI_SR0_EOF0F				BIT(12)
#define CSI_SR0_SOF3F				BIT(11)
#define CSI_SR0_SOF2F				BIT(10)
#define CSI_SR0_SOF1F				BIT(9)
#define CSI_SR0_SOF0F				BIT(8)
#define CSI_SR1					0x0094
#define CSI_SR1_STOPCLF				BIT(28)
#define CSI_SR1_STOPDL1F			BIT(25)
#define CSI_SR1_STOPDL0F			BIT(19)
#define CSI_SR1_ECTRLDL1F			BIT(12)
#define CSI_SR1_ESYNCESCDL1F			BIT(11)
#define CSI_SR1_EESCDL1F			BIT(10)
#define CSI_SR1_ESOTSYNCDL1F			BIT(9)
#define CSI_SR1_ESOTDL1F			BIT(8)
#define CSI_SR1_ECTRLDL0F			BIT(4)
#define CSI_SR1_ESYNCESCDL0F			BIT(3)
#define CSI_SR1_EESCDL0F			BIT(2)
#define CSI_SR1_ESOTSYNCDL0F			BIT(1)
#define CSI_SR1_ESOTDL0F			BIT(0)
#define CSI_FCR0				0x0100
#define CSI_FCR1				0x0104
#define CSI_SPDFR				0x0110
#define DT_MASK	0x3f
#define VC_MASK	0x03
#define CSI_ERR1				0x0114
#define CSI_ERR1_IDVCERR_SHIFT			22
#define CSI_ERR1_IDDTERR_SHIFT			16
#define CSI_ERR1_CECCVCERR_SHIFT		14
#define CSI_ERR1_CECCDTERR_SHIFT		8
#define CSI_ERR1_CRCVCERR_SHIFT			6
#define CSI_ERR1_CRCDTERR_SHIFT			0
#define CSI_ERR2				0x0118
#define CSI_ERR2_SYNCVCERR_SHIFT		18
#define CSI_ERR2_SPKTVCERR_SHIFT		6
#define CSI_ERR2_SPKTDTERR_SHIFT		0
#define CSI_PRCR				0x1000
#define CSI_PRCR_PEN				BIT(1)
#define CSI_PMCR				0x1004
#define CSI_PFCR				0x1008
#define CSI_PFCR_CCFR_MASK			GENMASK(5, 0)
#define CSI_PFCR_CCFR_SHIFT			0
#define CSI_PFCR_HSFR_MASK			GENMASK(14, 8)
#define CSI_PFCR_HSFR_SHIFT			8
#define CSI_PFCR_DLD				BIT(16)
#define CSI_PTCR0				0x1010
#define CSI_PTCR0_TCKEN				BIT(0)
#define CSI_PTCR1				0x1014
#define CSI_PTCR1_TWM				BIT(16)
#define CSI_PTCR1_TDI_MASK			GENMASK(7, 0)
#define CSI_PTCR1_TDI_SHIFT			0
#define CSI_PTSR				0x1018
#define CSI_HWCFGR				0x1ff0
#define CSI_HWCFGR_LANES_MASK			GENMASK(7, 4)
#define CSI_HWCFGR_LANES_SHIFT			4

#define CSI2HOST_LANES_MAX	2

#define CSI_SR0_ERRORS	(CSI_SR0_SYNCERRF | CSI_SR0_SPKTERRF | CSI_SR0_IDERRF |\
			 CSI_SR0_CECCERRF | CSI_SR0_ECCERRF | CSI_SR0_CRCERRF |\
			 CSI_SR0_CCFIFOFF)
#define CSI_SR1_DL0_ERRORS	(CSI_SR1_ECTRLDL0F | CSI_SR1_ESYNCESCDL0F |\
				 CSI_SR1_EESCDL0F | CSI_SR1_ESOTSYNCDL0F |\
				 CSI_SR1_ESOTDL0F)
#define CSI_SR1_DL1_ERRORS	(CSI_SR1_ECTRLDL1F | CSI_SR1_ESYNCESCDL1F |\
				 CSI_SR1_EESCDL1F | CSI_SR1_ESOTSYNCDL1F |\
				 CSI_SR1_ESOTDL1F)
#define CSI_SR1_ERRORS	(CSI_SR1_DL0_ERRORS | CSI_SR1_DL1_ERRORS)

enum csi2host_pads {
	CSI2HOST_PAD_SINK,
	CSI2HOST_PAD_SOURCE_STREAM0,
	CSI2HOST_PAD_SOURCE_STREAM1,
	CSI2HOST_PAD_SOURCE_STREAM2,
	CSI2HOST_PAD_SOURCE_STREAM3,
	CSI2HOST_PAD_SOURCE_STREAM4,
	CSI2HOST_PAD_SOURCE_STREAM5,
	CSI2HOST_PAD_SOURCE_STREAM6,
	CSI2HOST_PAD_SOURCE_STREAM7,
	CSI2HOST_PAD_MAX,
};

struct csi2host_priv {
	struct device			*dev;
	unsigned int			count;

	/*
	 * Used to prevent race conditions between multiple,
	 * concurrent calls to start and stop.
	 */
	struct mutex			lock;

	void __iomem			*base;
	struct clk			*pclk;
	struct clk			*txesc;
	struct clk			*csi2phy;
	struct regulator_bulk_data	supplies[2];
	struct reset_control		*rstc;

	u8				lanes[CSI2HOST_LANES_MAX];
	u8				num_lanes;
	u8				max_lanes;

	struct v4l2_subdev		subdev;
	struct v4l2_async_notifier	notifier;
	struct media_pad		pads[CSI2HOST_PAD_MAX];

	struct v4l2_mbus_framefmt	mf[CSI2HOST_PAD_MAX];

	/* Remote source */
	struct v4l2_subdev		*source_subdev;

#ifdef CSI2HOST_ERROR_HANDLING
	/* Backup registers used within irq handlers */
	u32 sr0;
	u32 sr1;
	u32 err1;
	u32 err2;
#endif
};

struct csi2host_fmts {
	u32 code;
	unsigned int datatype;
	unsigned int datatype_fmt;
	unsigned int bpp;
};

#define FMT_MBUS_DT_DTFMT_BPP(mbus, dt, dtfmt, byteperpixel)	\
		{						\
			.code = MEDIA_BUS_FMT_##mbus,		\
			.datatype = MIPI_CSI2_DT_##dt,		\
			.datatype_fmt = CSI_VCxCFGRx_DTxFT_##dtfmt,	\
			.bpp = byteperpixel,	\
		}
static const struct csi2host_fmts csi2host_formats[] = {
	/* YUV 422 8 bit */
	FMT_MBUS_DT_DTFMT_BPP(UYVY8_2X8, YUV422_8B, BPP8, 8),
	FMT_MBUS_DT_DTFMT_BPP(YUYV8_2X8, YUV422_8B, BPP8, 8),
	FMT_MBUS_DT_DTFMT_BPP(YVYU8_2X8, YUV422_8B, BPP8, 8),
	FMT_MBUS_DT_DTFMT_BPP(VYUY8_2X8, YUV422_8B, BPP8, 8),

	/* Raw Bayer */
	/* 8 bit */
	FMT_MBUS_DT_DTFMT_BPP(SBGGR8_1X8, RAW8, BPP8, 8),
	FMT_MBUS_DT_DTFMT_BPP(SGBRG8_1X8, RAW8, BPP8, 8),
	FMT_MBUS_DT_DTFMT_BPP(SGRBG8_1X8, RAW8, BPP8, 8),
	FMT_MBUS_DT_DTFMT_BPP(SRGGB8_1X8, RAW8, BPP8, 8),
	/* 10 bit */
	FMT_MBUS_DT_DTFMT_BPP(SRGGB10_1X10, RAW10, BPP10, 10),
	FMT_MBUS_DT_DTFMT_BPP(SGBRG10_1X10, RAW10, BPP10, 10),
	FMT_MBUS_DT_DTFMT_BPP(SGRBG10_1X10, RAW10, BPP10, 10),
	FMT_MBUS_DT_DTFMT_BPP(SRGGB10_1X10, RAW10, BPP10, 10),
	/* 12 bit */
	FMT_MBUS_DT_DTFMT_BPP(SRGGB12_1X12, RAW12, BPP12, 12),
	FMT_MBUS_DT_DTFMT_BPP(SGBRG12_1X12, RAW12, BPP12, 12),
	FMT_MBUS_DT_DTFMT_BPP(SGRBG12_1X12, RAW12, BPP12, 12),
	FMT_MBUS_DT_DTFMT_BPP(SRGGB12_1X12, RAW12, BPP12, 12),
	/* 14 bit */
	FMT_MBUS_DT_DTFMT_BPP(SRGGB14_1X14, RAW14, BPP14, 14),
	FMT_MBUS_DT_DTFMT_BPP(SGBRG14_1X14, RAW14, BPP14, 14),
	FMT_MBUS_DT_DTFMT_BPP(SGRBG14_1X14, RAW14, BPP14, 14),
	FMT_MBUS_DT_DTFMT_BPP(SRGGB14_1X14, RAW14, BPP14, 14),

	/* RGB 565 */
	FMT_MBUS_DT_DTFMT_BPP(RGB565_2X8_LE, RGB565, BPP8, 8),

	/* JPEG (datatype isn't used) */
	FMT_MBUS_DT_DTFMT_BPP(JPEG_1X8, NULL, BPP8, 8),
};

struct csi2host_mbps_phy_reg {
	unsigned int mbps;
	unsigned int hsfreqrange;
	unsigned int osc_freq_target;
};

/*
 * Table describing configuration of the PHY depending on the
 * intended Bit Rate. From table 5-8 Frequency Ranges and Defaults
 * of the Synopsis DWC MIPI PHY databook
 */
/* FIXME - this table should be configurable and depend on the compatible */
static const struct csi2host_mbps_phy_reg snps_stm32mp257[] = {
	{ .mbps =   80,	.hsfreqrange = 0x00,	.osc_freq_target = 460 },
	{ .mbps =   90, .hsfreqrange = 0x10,	.osc_freq_target = 460 },
	{ .mbps =  100, .hsfreqrange = 0x20,	.osc_freq_target = 460 },
	{ .mbps =  110, .hsfreqrange = 0x30,	.osc_freq_target = 460 },
	{ .mbps =  120, .hsfreqrange = 0x01,	.osc_freq_target = 460 },
	{ .mbps =  130, .hsfreqrange = 0x11,	.osc_freq_target = 460 },
	{ .mbps =  140, .hsfreqrange = 0x21,	.osc_freq_target = 460 },
	{ .mbps =  150, .hsfreqrange = 0x31,	.osc_freq_target = 460 },
	{ .mbps =  160, .hsfreqrange = 0x02,	.osc_freq_target = 460 },
	{ .mbps =  170, .hsfreqrange = 0x12,	.osc_freq_target = 460 },
	{ .mbps =  180, .hsfreqrange = 0x22,	.osc_freq_target = 460 },
	{ .mbps =  190, .hsfreqrange = 0x32,	.osc_freq_target = 460 },
	{ .mbps =  205, .hsfreqrange = 0x03,	.osc_freq_target = 460 },
	{ .mbps =  220, .hsfreqrange = 0x13,	.osc_freq_target = 460 },
	{ .mbps =  235, .hsfreqrange = 0x23,	.osc_freq_target = 460 },
	{ .mbps =  250, .hsfreqrange = 0x33,	.osc_freq_target = 460 },
	{ .mbps =  275, .hsfreqrange = 0x04,	.osc_freq_target = 460 },
	{ .mbps =  300, .hsfreqrange = 0x14,	.osc_freq_target = 460 },
	{ .mbps =  325, .hsfreqrange = 0x25,	.osc_freq_target = 460 },
	{ .mbps =  350, .hsfreqrange = 0x35,	.osc_freq_target = 460 },
	{ .mbps =  400, .hsfreqrange = 0x05,	.osc_freq_target = 460 },
	{ .mbps =  450, .hsfreqrange = 0x16,	.osc_freq_target = 460 },
	{ .mbps =  500, .hsfreqrange = 0x26,	.osc_freq_target = 460 },
	{ .mbps =  550, .hsfreqrange = 0x37,	.osc_freq_target = 460 },
	{ .mbps =  600, .hsfreqrange = 0x07,	.osc_freq_target = 460 },
	{ .mbps =  650, .hsfreqrange = 0x18,	.osc_freq_target = 460 },
	{ .mbps =  700, .hsfreqrange = 0x28,	.osc_freq_target = 460 },
	{ .mbps =  750, .hsfreqrange = 0x39,	.osc_freq_target = 460 },
	{ .mbps =  800, .hsfreqrange = 0x09,	.osc_freq_target = 460 },
	{ .mbps =  850, .hsfreqrange = 0x19,	.osc_freq_target = 460 },
	{ .mbps =  900, .hsfreqrange = 0x29,	.osc_freq_target = 460 },
	{ .mbps =  950, .hsfreqrange = 0x3a,	.osc_freq_target = 460 },
	{ .mbps = 1000, .hsfreqrange = 0x0a,	.osc_freq_target = 460 },
	{ .mbps = 1050, .hsfreqrange = 0x1a,	.osc_freq_target = 460 },
	{ .mbps = 1100, .hsfreqrange = 0x2a,	.osc_freq_target = 460 },
	{ .mbps = 1150, .hsfreqrange = 0x3b,	.osc_freq_target = 460 },
	{ .mbps = 1200, .hsfreqrange = 0x0b,	.osc_freq_target = 460 },
	{ .mbps = 1250, .hsfreqrange = 0x1b,	.osc_freq_target = 460 },
	{ .mbps = 1300, .hsfreqrange = 0x2b,	.osc_freq_target = 460 },
	{ .mbps = 1350, .hsfreqrange = 0x3c,	.osc_freq_target = 460 },
	{ .mbps = 1400, .hsfreqrange = 0x0c,	.osc_freq_target = 460 },
	{ .mbps = 1450, .hsfreqrange = 0x1c,	.osc_freq_target = 460 },
	{ .mbps = 1500, .hsfreqrange = 0x2c,	.osc_freq_target = 460 },
	{ .mbps = 1550, .hsfreqrange = 0x3d,	.osc_freq_target = 285 },
	{ .mbps = 1600, .hsfreqrange = 0x0d,	.osc_freq_target = 295 },
	{ .mbps = 1650, .hsfreqrange = 0x1d,	.osc_freq_target = 304 },
	{ .mbps = 1700, .hsfreqrange = 0x2e,	.osc_freq_target = 313 },
	{ .mbps = 1750, .hsfreqrange = 0x3e,	.osc_freq_target = 322 },
	{ .mbps = 1800, .hsfreqrange = 0x0e,	.osc_freq_target = 331 },
	{ .mbps = 1850, .hsfreqrange = 0x1e,	.osc_freq_target = 341 },
	{ .mbps = 1900, .hsfreqrange = 0x2f,	.osc_freq_target = 350 },
	{ .mbps = 1950, .hsfreqrange = 0x3f,	.osc_freq_target = 359 },
	{ .mbps = 2000, .hsfreqrange = 0x0f,	.osc_freq_target = 368 },
	{ .mbps = 2050, .hsfreqrange = 0x40,	.osc_freq_target = 377 },
	{ .mbps = 2100, .hsfreqrange = 0x41,	.osc_freq_target = 387 },
	{ .mbps = 2150, .hsfreqrange = 0x42,	.osc_freq_target = 396 },
	{ .mbps = 2200, .hsfreqrange = 0x43,	.osc_freq_target = 405 },
	{ .mbps = 2250, .hsfreqrange = 0x44,	.osc_freq_target = 414 },
	{ .mbps = 2300, .hsfreqrange = 0x45,	.osc_freq_target = 423 },
	{ .mbps = 2350, .hsfreqrange = 0x46,	.osc_freq_target = 432 },
	{ .mbps = 2400, .hsfreqrange = 0x47,	.osc_freq_target = 442 },
	{ .mbps = 2450, .hsfreqrange = 0x48,	.osc_freq_target = 451 },
	{ .mbps = 2500, .hsfreqrange = 0x49,	.osc_freq_target = 460 },
	{ /* sentinel */ },
};

static const struct csi2host_fmts *csi2host_code_to_fmt(unsigned int code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(csi2host_formats); i++)
		if (csi2host_formats[i].code == code)
			return &csi2host_formats[i];

	return NULL;
}

static inline
struct csi2host_priv *v4l2_subdev_to_csi2priv(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct csi2host_priv, subdev);
}

static int csi2host_setup_lane_merger(struct csi2host_priv *csi2priv)
{
	int i;
	u32 lmcfgr;

	lmcfgr = readl_relaxed(csi2priv->base + CSI_LMCFGR);
	lmcfgr &= ~(CSI_LMCFGR_LANENB_MASK | CSI_LMCFGR_DL0MAP_MASK |
		    CSI_LMCFGR_DL1MAP_MASK);

	for (i = 0; i < csi2priv->num_lanes; i++) {
		/* Check that lane ID is < max number of lane */
		if (csi2priv->lanes[i] >= csi2priv->max_lanes) {
			dev_err(csi2priv->dev, "Invalid lane id (%d)\n",
				csi2priv->lanes[i]);
			return -EINVAL;
		}
		lmcfgr |= ((csi2priv->lanes[i] + 1) << ((i * 4) +
			   CSI_LMCFGR_DL0MAP_SHIFT));
	}

	lmcfgr |= (csi2priv->num_lanes << CSI_LMCFGR_LANENB_SHIFT);

	writel_relaxed(lmcfgr, csi2priv->base + CSI_LMCFGR);

	return 0;
}

static void csi2host_phy_reg_write(struct csi2host_priv *csi2priv, uint32_t addr, uint32_t val)
{
	/* Based on sequence described at section 5.2.3.2 of DesignWave document */
	/* For writing the 4-bit testcode MSBs */
	/* Set testen to high */
	writel_relaxed(CSI_PTCR1_TWM, csi2priv->base + CSI_PTCR1);

	/* Set testclk to high */
	writel_relaxed(CSI_PTCR0_TCKEN, csi2priv->base + CSI_PTCR0);

	/* Place 0x00 in testdin */
	writel_relaxed(CSI_PTCR1_TWM, csi2priv->base + CSI_PTCR1);

	/*
	 * Set testclk to low (with the falling edge on testclk, the testdin signal content is
	 * latched internally)
	 */
	writel_relaxed(0, csi2priv->base + CSI_PTCR0);

	/* Set testen to low */
	writel_relaxed(0, csi2priv->base + CSI_PTCR1);

	/* Place the 8-bit word corresponding to the testcode MSBs in testdin */
	writel_relaxed(((addr >> 8) & CSI_PTCR1_TDI_MASK) << CSI_PTCR1_TDI_SHIFT,
		       csi2priv->base + CSI_PTCR1);

	/* Set testclk to high */
	writel_relaxed(CSI_PTCR0_TCKEN, csi2priv->base + CSI_PTCR0);

	/* For writing the 8-bit testcode LSBs */
	/* Set testclk to low */
	writel_relaxed(0, csi2priv->base + CSI_PTCR0);

	/* Set testen to high */
	writel_relaxed(CSI_PTCR1_TWM, csi2priv->base + CSI_PTCR1);

	/* Set testclk to high */
	writel_relaxed(CSI_PTCR0_TCKEN, csi2priv->base + CSI_PTCR0);

	/* Place the 8-bit word test data in testdin */
	writel_relaxed((addr & CSI_PTCR1_TDI_MASK) << CSI_PTCR1_TDI_SHIFT | CSI_PTCR1_TWM,
		       csi2priv->base + CSI_PTCR1);

	/*
	 * Set testclk to low (with the falling edge on testclk, the testdin signal content is
	 * latched internally)
	 */
	writel_relaxed(0, csi2priv->base + CSI_PTCR0);

	/* Set testen to low */
	writel_relaxed(0, csi2priv->base + CSI_PTCR1);

	/* For writing the data */
	/* Place the 8-bit word corresponding to the page offset in testdin */
	writel_relaxed((val & CSI_PTCR1_TDI_MASK) << CSI_PTCR1_TDI_SHIFT,
		       csi2priv->base + CSI_PTCR1);

	/* Set testclk to high (test data is programmed internally */
	writel_relaxed(CSI_PTCR0_TCKEN, csi2priv->base + CSI_PTCR0);

	/* Finish by setting testclk to low */
	writel_relaxed(0, csi2priv->base + CSI_PTCR0);
}

static int csi2host_start(struct csi2host_priv *csi2priv)
{
	const struct csi2host_fmts *fmt;
	const struct csi2host_mbps_phy_reg *phy_regs;
	int ret, i, mbps;
	unsigned long phy_clk_frate;
	u32 lanes_ie = 0;
	u32 lanes_en = 0;
	u32 lanes_stop = 0;
	u32 ccfr;
	s64 link_freq;

	dev_dbg(csi2priv->dev, "Starting the CSI2\n");
	/* Get format info to get BPP - FIXME, is this ok to check pad 0 ?? */
	fmt = csi2host_code_to_fmt(csi2priv->mf[0].code);

	/* Get the remote sensor link frequency */
	if (!csi2priv->source_subdev)
		return -EIO;

	link_freq = v4l2_get_link_freq(csi2priv->source_subdev->ctrl_handler,
				       fmt->bpp, 2 * csi2priv->num_lanes);
	if (link_freq < 0)
		return link_freq;

	/* MBPS is expressed in Mbps, hence link_freq / 100000 * 2 */
	mbps = link_freq / 500000;
	dev_dbg(csi2priv->dev, "Computed Mbps: %u\n", mbps);

	for (phy_regs = snps_stm32mp257; phy_regs->mbps != 0; phy_regs++)
		if (phy_regs->mbps >= mbps)
			break;

	if (!phy_regs->mbps) {
		dev_err(csi2priv->dev, "Unsupported PHY speed (%u Mbps)", mbps);
		return -ERANGE;
	}

	dev_dbg(csi2priv->dev, "PHY settings: (%u Mbps, %u HS FRange, %u OSC Freq)\n",
		phy_regs->mbps, phy_regs->hsfreqrange,
		phy_regs->osc_freq_target);

	/* Prepare lanes related configuration bits */
	for (i = 0; i < csi2priv->num_lanes; i++) {
		if (!csi2priv->lanes[i]) {
			lanes_ie |= CSI_SR1_DL0_ERRORS;
			lanes_en |= CSI_PCR_DL0EN;
			lanes_stop |= CSI_SR1_STOPDL0F;
		} else {
			lanes_ie |= CSI_SR1_DL1_ERRORS;
			lanes_en |= CSI_PCR_DL1EN;
			lanes_stop |= CSI_SR1_STOPDL1F;
		}
	}

	ret = pm_runtime_get_sync(csi2priv->dev);
	if (ret < 0)
		return ret;

	/* Retrieve CSI2PHY clock rate to compute CCFR value */
	phy_clk_frate = clk_get_rate(csi2priv->csi2phy);
	if (!phy_clk_frate) {
		pm_runtime_put(csi2priv->dev);
		dev_err(csi2priv->dev, "CSI2PHY clock rate invalid (0)\n");
		return ret;
	}

	ret = csi2host_setup_lane_merger(csi2priv);
	if (ret) {
		pm_runtime_put(csi2priv->dev);
		return ret;
	}

	/* FIXME - how to set the watchdog value .. */
	writel_relaxed(0, csi2priv->base + CSI_WDR);

	/* Enable the CSI */
	writel_relaxed(CSI_CR_CSIEN, csi2priv->base + CSI_CR);

#ifdef CSI2HOST_ERROR_HANDLING
	/* Enable some global CSI related interrupts - bits are same as SR0 */
	writel_relaxed(CSI_SR0_ERRORS, csi2priv->base + CSI_IER0);

	/* Enable lanes related error interrupts */
	writel_relaxed(lanes_ie, csi2priv->base + CSI_IER1);
#endif

	/* Initialization of the D-PHY */
	/* Stop the D-PHY */
	writel_relaxed(0, csi2priv->base + CSI_PRCR);

	/* Keep the D-PHY in power down state */
	writel_relaxed(0, csi2priv->base + CSI_PCR);

	/* Enable testclr clock during 15ns */
	writel_relaxed(CSI_PTCR0_TCKEN, csi2priv->base + CSI_PTCR0);
	udelay(1);
	writel_relaxed(0, csi2priv->base + CSI_PTCR0);

	/* Set hsfreqrange */
	phy_clk_frate /= 1000000;
	ccfr = (phy_clk_frate - 17) * 4;
	writel_relaxed((ccfr << CSI_PFCR_CCFR_SHIFT) |
		       (phy_regs->hsfreqrange << CSI_PFCR_HSFR_SHIFT),
		       csi2priv->base + CSI_PFCR);

	/* set reg @08 deskew_polarity_rw 1'b1 */
	csi2host_phy_reg_write(csi2priv, 0x08, 0x38);

	/* set reg @0xE4 counter_for_des_en_config_if_rx 0x10 + DLL prog EN */
	/* This is because 13<= cfgclkfreqrange[5:0]<=38 */
	csi2host_phy_reg_write(csi2priv, 0xe4, 0x11);

	/* set reg @0xe2 & reg @0xe3 value DLL target oscilation freq */
	/* Based on the table page 77, osc_freq_target */
	csi2host_phy_reg_write(csi2priv, 0xe2, phy_regs->osc_freq_target & 0xFF);
	csi2host_phy_reg_write(csi2priv, 0xe3, (phy_regs->osc_freq_target >> 8) & 0x0F);

	writel_relaxed(CSI_PFCR_DLD | readl_relaxed(csi2priv->base + CSI_PFCR),
		       csi2priv->base + CSI_PFCR);

	/* Enable Lanes */
	writel_relaxed(lanes_en | CSI_PCR_CLEN, csi2priv->base + CSI_PCR);
	writel_relaxed(lanes_en | CSI_PCR_CLEN | CSI_PCR_PWRDOWN,
		       csi2priv->base + CSI_PCR);

	writel_relaxed(CSI_PRCR_PEN, csi2priv->base + CSI_PRCR);

	/* Remove the force */
	writel_relaxed(0, csi2priv->base + CSI_PMCR);

	return ret;
}

static void csi2host_stop(struct csi2host_priv *csi2priv)
{
	dev_dbg(csi2priv->dev, "Stopping the CSI2\n");

	/* Disable the D-PHY */
	writel_relaxed(0, csi2priv->base + CSI_PCR);

#ifdef CSI2HOST_ERROR_HANDLING
	/* Disable ITs */
	writel_relaxed(0, csi2priv->base + CSI_IER0);
	writel_relaxed(0, csi2priv->base + CSI_IER1);
#endif

	/* Disable the CSI */
	writel_relaxed(0, csi2priv->base + CSI_CR);

	pm_runtime_put(csi2priv->dev);
}

static int csi2host_start_vc(struct csi2host_priv *csi2priv, uint32_t vc)
{
	const struct csi2host_fmts *fmt;
	struct media_pad *rpad;
	int ret = 0;
	u32 cfgr1 = 0, cfgr2 = 0;
	u32 dt0 = 0, dt1 = 0;
	u32 status;

	/* Get DT codes of the two related source pads */
	rpad = media_pad_remote_pad_first(&csi2priv->pads[1 + (2 * vc)]);
	if (rpad)
		dt0 = csi2priv->mf[1 + (2 * vc)].code;

	rpad = media_pad_remote_pad_first(&csi2priv->pads[1 + (2 * vc) + 1]);
	if (rpad)
		dt1 = csi2priv->mf[1 + (2 * vc) + 1].code;

	/* If either of the two output is JPEG, don't perform filtering */
	if (dt0 == MEDIA_BUS_FMT_JPEG_1X8 || dt1 == MEDIA_BUS_FMT_JPEG_1X8) {
		cfgr1 |= CSI_VCxCFGR1_ALLDT;
		cfgr1 |= (CSI_VCxCFGRx_DTxFT_BPP8 << CSI_VCxCFGR1_CDTFT_SHIFT);
		dev_dbg(csi2priv->dev, "VC%d: enable AllDT in BPP8 mode\n", vc);
	} else {
		/* Configure DT0 */
		if (dt0) {
			fmt = csi2host_code_to_fmt(dt0);
			cfgr1 |= fmt->datatype << CSI_VCxCFGR1_DT0_SHIFT;
			cfgr1 |= fmt->datatype_fmt << CSI_VCxCFGR1_DT0FT_SHIFT;
			cfgr1 |= CSI_VCxCFGR1_DT0EN;
			dev_dbg(csi2priv->dev, "VC%d: enable DT0 (0x%x) - DT0FT (0x%x)\n",
				vc, fmt->datatype, fmt->datatype_fmt);
		}

		/* Configure DT1 */
		if (dt1) {
			fmt = csi2host_code_to_fmt(dt1);
			cfgr2 |= fmt->datatype << CSI_VCxCFGR2_DT1_SHIFT;
			cfgr2 |= fmt->datatype_fmt << CSI_VCxCFGR2_DT1FT_SHIFT;
			cfgr1 |= CSI_VCxCFGR1_DT1EN;
			dev_dbg(csi2priv->dev, "VC%d: enable DT1 (0x%x) - DT1FT (0x%x)\n",
				vc, fmt->datatype, fmt->datatype_fmt);
			writel_relaxed(cfgr2, csi2priv->base + CSI_VCxCFGR2(vc));
		}
	}

	writel_relaxed(cfgr1, csi2priv->base + CSI_VCxCFGR1(vc));

	/* Start the Virtual Channel if necessary */
	if (!cfgr1)
		return 0;

	writel_relaxed(readl_relaxed(csi2priv->base + CSI_CR) | CSI_CR_VCxSTART(vc),
		       csi2priv->base + CSI_CR);

	ret = readl_relaxed_poll_timeout(csi2priv->base + CSI_SR0,
					 status,
					 status & CSI_SR0_VCxSTATEF(vc),
					 1000, 1000000);
	if (ret) {
		dev_err(csi2priv->dev, "failed to start VC(%d)\n", vc);
		return ret;
	}

	return 0;
}

static int csi2host_stop_vc(struct csi2host_priv *csi2priv, uint32_t vc)
{
	int ret = 0;
	u32 status;

	/* Stop the Virtual Channel */
	writel_relaxed(readl_relaxed(csi2priv->base + CSI_CR) | CSI_CR_VCxSTOP(vc),
		       csi2priv->base + CSI_CR);

	/*
	 * FIXME - timeout is probably not correct. CSI2HOST will wait
	 * until getting an EOF before resetting the VC
	 */
	ret = readl_relaxed_poll_timeout(csi2priv->base + CSI_SR0,
					 status,
					 !(status & CSI_SR0_VCxSTATEF(vc)),
					 1000, 1000000);
	if (ret) {
		dev_err(csi2priv->dev, "failed to stop VC(%d)\n", vc);
		return ret;
	}

	/* Disable all DTs */
	writel_relaxed(0, csi2priv->base + CSI_VCxCFGR1(vc));
	writel_relaxed(0, csi2priv->base + CSI_VCxCFGR2(vc));

	return 0;
}

static int csi2host_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct csi2host_priv *csi2priv = v4l2_subdev_to_csi2priv(subdev);
	int ret, i;

	mutex_lock(&csi2priv->lock);

	if (enable) {
		/*
		 * If we're not the first users, there's no need to
		 * enable the whole controller.
		 */
		if (!csi2priv->count) {
			ret = csi2host_start(csi2priv);
			if (ret)
				goto out;
		}

		/* Check and configure all 4 VCs */
		for (i = 0; i < 4; i++) {
			ret = csi2host_start_vc(csi2priv, i);
			/* FIXME - better error handling needed here */
			if (ret)
				dev_err(csi2priv->dev, "Failed to start VC%d\n", i);
		}

		csi2priv->count++;
	} else {
		/* Stop all 4 VCs */
		for (i = 0; i < 4; i++) {
			ret = csi2host_stop_vc(csi2priv, i);
			if (ret) {
				dev_err(csi2priv->dev, "Failed to stop VC%d, turning off CSI to recover\n",
					i);
				csi2host_stop(csi2priv);
			}
		}

		csi2priv->count--;

		/*
		 * Let the last user turn off the lights.
		 */
		if (!csi2priv->count)
			csi2host_stop(csi2priv);
	}

out:
	mutex_unlock(&csi2priv->lock);
	return ret;
}

static int csi2host_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(csi2host_formats))
		return -EINVAL;

	code->code = csi2host_formats[code->index].code;
	return 0;
}

static int csi2host_set_pad_format(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_format *format)
{
	struct csi2host_priv *csi2priv = v4l2_subdev_to_csi2priv(sd);
	const struct csi2host_fmts *fmt;

	struct v4l2_mbus_framefmt *framefmt;

	fmt = csi2host_code_to_fmt(format->format.code);
	if (!fmt) {
		dev_dbg(csi2priv->dev, "Unsupported code %d, use default\n",
			format->format.code);
		format->format.code = csi2host_formats[0].code;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		csi2priv->mf[format->pad] = format->format;
		/* Same format is exposed to the source pad as well */
		if (format->pad == CSI2HOST_PAD_SINK)
			csi2priv->mf[CSI2HOST_PAD_SOURCE_STREAM0] =
					format->format;
	} else {
		framefmt = v4l2_subdev_get_try_format(sd, state, format->pad);
		*framefmt = format->format;
		if (format->pad == CSI2HOST_PAD_SINK) {
			framefmt = v4l2_subdev_get_try_format(sd, state,
							      CSI2HOST_PAD_SOURCE_STREAM0);
			*framefmt = format->format;
		}
	}

	return 0;
}

static int csi2host_get_pad_format(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_format *format)
{
	struct csi2host_priv *csi2priv = v4l2_subdev_to_csi2priv(sd);

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		format->format = csi2priv->mf[format->pad];
	else
		format->format =
			*v4l2_subdev_get_try_format(sd, state, format->pad);

	return 0;
}

static const struct v4l2_subdev_video_ops csi2host_video_ops = {
	.s_stream	= csi2host_s_stream,
};

static const struct v4l2_subdev_pad_ops csi2host_pad_ops = {
	.enum_mbus_code	= csi2host_enum_mbus_code,
	.set_fmt	= csi2host_set_pad_format,
	.get_fmt	= csi2host_get_pad_format,
};

static const struct v4l2_subdev_ops csi2host_subdev_ops = {
	.video		= &csi2host_video_ops,
	.pad		= &csi2host_pad_ops,
};

static int csi2host_async_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *s_subdev,
				struct v4l2_async_subdev *asd)
{
	struct v4l2_subdev *subdev = notifier->sd;
	struct csi2host_priv *csi2priv = v4l2_subdev_to_csi2priv(subdev);
	int remote_pad;

	remote_pad = media_entity_get_fwnode_pad(&s_subdev->entity,
						 s_subdev->fwnode,
						 MEDIA_PAD_FL_SOURCE);
	if (remote_pad < 0) {
		dev_err(csi2priv->dev, "Couldn't find output pad for subdev %s\n",
			s_subdev->name);
		return remote_pad;
	}

	csi2priv->source_subdev = s_subdev;

	return media_create_pad_link(&csi2priv->source_subdev->entity,
				     remote_pad, &csi2priv->subdev.entity, 0,
				     MEDIA_LNK_FL_ENABLED |
				     MEDIA_LNK_FL_IMMUTABLE);
}

static const struct v4l2_async_notifier_operations csi2host_notifier_ops = {
	.bound		= csi2host_async_bound,
};

#ifdef CSI2HOST_ERROR_HANDLING
static irqreturn_t csi2host_irq_thread(int irq, void *arg)
{
	struct csi2host_priv *csi2priv = arg;

	if (csi2priv->sr0 & CSI_SR0_SYNCERRF)
		dev_dbg(csi2priv->dev, "Invalid synchronization error VC(%d)\n",
			(csi2priv->err2 >> CSI_ERR2_SYNCVCERR_SHIFT) & VC_MASK);

	if (csi2priv->sr0 & CSI_SR0_SPKTERRF)
		dev_dbg(csi2priv->dev, "Short packet error VC(%d)/DT(%d)\n",
			(csi2priv->err2 >> CSI_ERR2_SPKTVCERR_SHIFT) & VC_MASK,
			(csi2priv->err2 >> CSI_ERR2_SPKTDTERR_SHIFT) & DT_MASK);

	if (csi2priv->sr0 & CSI_SR0_IDERRF)
		dev_dbg(csi2priv->dev, "Data type ID error VC(%d)/DT(%d)\n",
			(csi2priv->err1 >> CSI_ERR1_IDVCERR_SHIFT) & VC_MASK,
			(csi2priv->err1 >> CSI_ERR1_IDDTERR_SHIFT) & DT_MASK);

	if (csi2priv->sr0 & CSI_SR0_CECCERRF)
		dev_dbg(csi2priv->dev, "Corrected ECC error VC(%d)/DT(%d)\n",
			(csi2priv->err1 >> CSI_ERR1_CECCVCERR_SHIFT) & VC_MASK,
			(csi2priv->err1 >> CSI_ERR1_CECCDTERR_SHIFT) & DT_MASK);

	if (csi2priv->sr0 & CSI_SR0_ECCERRF)
		dev_dbg(csi2priv->dev, "ECC error\n");

	if (csi2priv->sr0 & CSI_SR0_CRCERRF)
		dev_dbg(csi2priv->dev, "CRC error VC(%d)/DT(%d)\n",
			(csi2priv->err1 >> CSI_ERR1_CRCVCERR_SHIFT) & VC_MASK,
			(csi2priv->err1 >> CSI_ERR1_CRCDTERR_SHIFT) & DT_MASK);

	if (csi2priv->sr0 & CSI_SR0_CCFIFOFF)
		dev_dbg(csi2priv->dev, "Clock changer FIFO full error\n");

	if (csi2priv->sr1 & CSI_SR1_ECTRLDL1F)
		dev_dbg(csi2priv->dev, "L1: D-PHY_RX lane 1 control error\n");

	if (csi2priv->sr1 & CSI_SR1_ESYNCESCDL1F)
		dev_dbg(csi2priv->dev, "L1: D-PHY_RX lane 1 low power data transmission synchronization error\n");

	if (csi2priv->sr1 & CSI_SR1_EESCDL1F)
		dev_dbg(csi2priv->dev, "L1: D-PHY_RX lane 1 escape entry error\n");

	if (csi2priv->sr1 & CSI_SR1_ESOTSYNCDL1F)
		dev_dbg(csi2priv->dev, "L1: Start of transmission synchronization error\n");

	if (csi2priv->sr1 & CSI_SR1_ESOTDL1F)
		dev_dbg(csi2priv->dev, "L1: Start of transmission error\n");

	if (csi2priv->sr1 & CSI_SR1_ECTRLDL0F)
		dev_dbg(csi2priv->dev, "L0: D-PHY_RX lane 0 control error\n");

	if (csi2priv->sr1 & CSI_SR1_ESYNCESCDL0F)
		dev_dbg(csi2priv->dev, "L0: D-PHY_RX lane 0 low power data transmission synchronization error\n");

	if (csi2priv->sr1 & CSI_SR1_EESCDL0F)
		dev_dbg(csi2priv->dev, "L0: D-PHY_RX lane 0 escape entry error\n");

	if (csi2priv->sr1 & CSI_SR1_ESOTSYNCDL0F)
		dev_dbg(csi2priv->dev, "L0: Start of transmission synchronization error\n");

	if (csi2priv->sr1 & CSI_SR1_ESOTDL0F)
		dev_dbg(csi2priv->dev, "L0: Start of transmission error\n");

	return IRQ_HANDLED;
}

static irqreturn_t csi2host_irq_callback(int irq, void *arg)
{
	struct csi2host_priv *csi2priv = arg;

	csi2priv->sr0 = readl_relaxed(csi2priv->base + CSI_SR0);
	csi2priv->sr1 = readl_relaxed(csi2priv->base + CSI_SR1);
	csi2priv->err1 = readl_relaxed(csi2priv->base + CSI_ERR1);
	csi2priv->err2 = readl_relaxed(csi2priv->base + CSI_ERR2);

	/* Clear interrupt */
	writel_relaxed(csi2priv->sr0 & CSI_SR0_ERRORS, csi2priv->base + CSI_FCR0);
	writel_relaxed(csi2priv->sr1 & CSI_SR1_ERRORS, csi2priv->base + CSI_FCR1);

	return IRQ_WAKE_THREAD;
}
#endif

static int csi2host_get_resources(struct csi2host_priv *csi2priv,
				  struct platform_device *pdev)
{
	struct resource *res;
#ifdef CSI2HOST_ERROR_HANDLING
	int irq;
#endif
	int ret;
	u32 dev_cfg;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	csi2priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(csi2priv->base))
		return dev_err_probe(&pdev->dev, PTR_ERR(csi2priv->base),
				     "Failed to ioremap resource\n");

	csi2priv->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(csi2priv->pclk))
		return dev_err_probe(&pdev->dev, PTR_ERR(csi2priv->pclk),
				     "Couldn't get pclk\n");

	csi2priv->txesc = devm_clk_get(&pdev->dev, "txesc");
	if (IS_ERR(csi2priv->txesc))
		return dev_err_probe(&pdev->dev, PTR_ERR(csi2priv->txesc),
				     "Couldn't get txesc\n");

	csi2priv->csi2phy = devm_clk_get(&pdev->dev, "csi2phy");
	if (IS_ERR(csi2priv->csi2phy))
		return dev_err_probe(&pdev->dev, PTR_ERR(csi2priv->csi2phy),
				     "Couldn't get csi2phy\n");

	csi2priv->rstc = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(csi2priv->rstc))
		return dev_err_probe(&pdev->dev, PTR_ERR(csi2priv->rstc),
				     "Couldn't get reset control\n");

	csi2priv->supplies[0].supply = "vdd";
	csi2priv->supplies[1].supply = "vdda18";
	ret = devm_regulator_bulk_get(&pdev->dev, ARRAY_SIZE(csi2priv->supplies),
				      csi2priv->supplies);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to request regulator vdd\n");

#ifdef CSI2HOST_ERROR_HANDLING
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return irq ? irq : -ENXIO;

	ret = devm_request_threaded_irq(&pdev->dev, irq, csi2host_irq_callback,
					csi2host_irq_thread, IRQF_ONESHOT,
					dev_name(&pdev->dev), csi2priv);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Unable to request irq");
#endif

	clk_prepare_enable(csi2priv->pclk);
	dev_cfg = readl_relaxed(csi2priv->base + CSI_HWCFGR);
	clk_disable_unprepare(csi2priv->pclk);

	csi2priv->max_lanes = (dev_cfg & CSI_HWCFGR_LANES_MASK)
				>> CSI_HWCFGR_LANES_SHIFT;
	if (csi2priv->max_lanes > CSI2HOST_LANES_MAX) {
		dev_err(&pdev->dev, "Invalid number of lanes: %u\n",
			csi2priv->max_lanes);
		return -EINVAL;
	}

	return 0;
}

static int csi2host_parse_dt(struct csi2host_priv *csi2priv)
{
	struct v4l2_fwnode_endpoint v4l2_ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct v4l2_async_subdev *asd;
	struct fwnode_handle *fwh;
	struct device_node *ep;
	int ret;

	ep = of_graph_get_endpoint_by_regs(csi2priv->dev->of_node, 0, 0);
	if (!ep)
		return -EINVAL;

	fwh = of_fwnode_handle(ep);
	ret = v4l2_fwnode_endpoint_parse(fwh, &v4l2_ep);
	if (ret) {
		dev_err(csi2priv->dev, "Could not parse v4l2 endpoint\n");
		of_node_put(ep);
		return ret;
	}
	of_node_put(ep);

	csi2priv->num_lanes = v4l2_ep.bus.mipi_csi2.num_data_lanes;
	if (csi2priv->num_lanes > csi2priv->max_lanes) {
		dev_err(csi2priv->dev, "Unsupported number of data-lanes: %d\n",
			csi2priv->num_lanes);
		of_node_put(ep);
		return -EINVAL;
	}

	memcpy(csi2priv->lanes, v4l2_ep.bus.mipi_csi2.data_lanes,
	       sizeof(csi2priv->lanes));

	ep = of_graph_get_next_endpoint(csi2priv->dev->of_node, NULL);
	if (!ep) {
		dev_err(csi2priv->dev, "Failed to get next endpoint\n");
		return -EINVAL;
	}

	v4l2_async_nf_init(&csi2priv->notifier);

	asd = v4l2_async_nf_add_fwnode_remote(&csi2priv->notifier, of_fwnode_handle(ep),
					      struct v4l2_async_subdev);

	of_node_put(ep);

	if (IS_ERR(asd)) {
		dev_err(csi2priv->dev, "Failed to add fwnode remote subdev\n");
		return PTR_ERR(asd);
	}

	csi2priv->notifier.ops = &csi2host_notifier_ops;

	ret = v4l2_async_subdev_nf_register(&csi2priv->subdev, &csi2priv->notifier);
	if (ret)
		v4l2_async_nf_cleanup(&csi2priv->notifier);

	return ret;
}

static int csi2host_probe(struct platform_device *pdev)
{
	struct csi2host_priv *csi2priv;
	unsigned int i;
	int ret;

	csi2priv = devm_kzalloc(&pdev->dev, sizeof(*csi2priv), GFP_KERNEL);
	if (!csi2priv)
		return -ENOMEM;
	platform_set_drvdata(pdev, csi2priv);
	csi2priv->dev = &pdev->dev;
	mutex_init(&csi2priv->lock);

	ret = csi2host_get_resources(csi2priv, pdev);
	if (ret)
		goto err_free_priv;

	ret = csi2host_parse_dt(csi2priv);
	if (ret)
		goto err_free_priv;

	csi2priv->subdev.owner = THIS_MODULE;
	csi2priv->subdev.dev = &pdev->dev;
	v4l2_subdev_init(&csi2priv->subdev, &csi2host_subdev_ops);
	v4l2_set_subdevdata(&csi2priv->subdev, &pdev->dev);
	snprintf(csi2priv->subdev.name, V4L2_SUBDEV_NAME_SIZE, "%s.%s",
		 KBUILD_MODNAME, dev_name(&pdev->dev));

	/* Create our media pads */
	csi2priv->subdev.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	csi2priv->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	csi2priv->pads[CSI2HOST_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2priv->mf[CSI2HOST_PAD_SINK].code = MEDIA_BUS_FMT_UYVY8_2X8;
	for (i = CSI2HOST_PAD_SOURCE_STREAM0; i < CSI2HOST_PAD_MAX; i++) {
		csi2priv->pads[i].flags = MEDIA_PAD_FL_SOURCE;
		csi2priv->mf[i].code = MEDIA_BUS_FMT_UYVY8_2X8;
	}

	ret = media_entity_pads_init(&csi2priv->subdev.entity, CSI2HOST_PAD_MAX,
				     csi2priv->pads);
	if (ret)
		goto err_cleanup;

	ret = v4l2_async_register_subdev(&csi2priv->subdev);
	if (ret < 0)
		goto err_cleanup;

	/* Reset device */
	ret = reset_control_assert(csi2priv->rstc);
	if (ret) {
		ret = dev_err_probe(&pdev->dev, ret,
				    "Failed to assert the reset line\n");
		goto err_cleanup;
	}

	usleep_range(3000, 5000);

	ret = reset_control_deassert(csi2priv->rstc);
	if (ret) {
		ret = dev_err_probe(&pdev->dev, ret,
				    "Failed to deassert the reset line\n");
		goto err_cleanup;
	}

	pm_runtime_enable(&pdev->dev);

	dev_info(&pdev->dev,
		 "Probed CSI2HOST with %u lanes\n", csi2priv->num_lanes);

	return 0;

err_cleanup:
	v4l2_async_nf_cleanup(&csi2priv->notifier);
err_free_priv:
	return ret;
}

static int csi2host_remove(struct platform_device *pdev)
{
	struct csi2host_priv *csi2priv = platform_get_drvdata(pdev);

	v4l2_async_unregister_subdev(&csi2priv->subdev);

	pm_runtime_disable(&pdev->dev);

	return 0;
}

static int __maybe_unused csi2host_runtime_suspend(struct device *dev)
{
	struct csi2host_priv *csi2priv = dev_get_drvdata(dev);
	int ret;

	clk_disable_unprepare(csi2priv->csi2phy);
	clk_disable_unprepare(csi2priv->txesc);
	clk_disable_unprepare(csi2priv->pclk);

	ret = regulator_bulk_disable(ARRAY_SIZE(csi2priv->supplies), csi2priv->supplies);
	if (ret < 0)
		dev_err(dev, "cannot disable regulators %d\n", ret);

	return 0;
}

static int __maybe_unused csi2host_runtime_resume(struct device *dev)
{
	struct csi2host_priv *csi2priv = dev_get_drvdata(dev);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(csi2priv->supplies), csi2priv->supplies);
	if (ret)
		goto error_out;

	ret = clk_prepare_enable(csi2priv->pclk);
	if (ret)
		goto error_disable_supplies;

	ret = clk_prepare_enable(csi2priv->txesc);
	if (ret)
		goto error_disable_pclk;

	ret = clk_prepare_enable(csi2priv->csi2phy);
	if (ret)
		goto error_disable_txesc;

	return 0;

error_disable_txesc:
	clk_disable_unprepare(csi2priv->txesc);
error_disable_pclk:
	clk_disable_unprepare(csi2priv->pclk);
error_disable_supplies:
	ret = regulator_bulk_disable(ARRAY_SIZE(csi2priv->supplies), csi2priv->supplies);
	if (ret < 0)
		dev_err(dev, "cannot disable regulators %d\n", ret);
error_out:
	dev_err(csi2priv->dev, "Failed to resume: %d\n", ret);

	return ret;
}

static const struct of_device_id csi2host_of_table[] = {
	{ .compatible = "st,stm32-csi2host" },
	{ },
};
MODULE_DEVICE_TABLE(of, csi2host_of_table);

static const struct dev_pm_ops csi2host_pm_ops = {
	SET_RUNTIME_PM_OPS(csi2host_runtime_suspend, csi2host_runtime_resume, NULL)
};

static struct platform_driver csi2host_driver = {
	.probe	= csi2host_probe,
	.remove	= csi2host_remove,

	.driver	= {
		.name		= "stm32-csi2host",
		.of_match_table	= csi2host_of_table,
		.pm		= &csi2host_pm_ops,
	},
};
module_platform_driver(csi2host_driver);
MODULE_AUTHOR("Alain Volmat <alain.volmat@foss.st.com>");
MODULE_DESCRIPTION("STM32 CSI controller");
MODULE_LICENSE("GPL");
