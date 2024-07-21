// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022, STMicroelectronics - All Rights Reserved
 * Author(s): Raphaël GALLAIS-POU <raphael.gallais-pou@foss.st.com> for STMicroelectronics.
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_device.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

/* LVDS Host registers */
#define LVDS_CR		0x0000  /* configuration register */
#define LVDS_DMLCR0	0x0004  /* data mapping lsb configuration register 0	*/
#define LVDS_DMMCR0	0x0008  /* data mapping msb configuration register 0	*/
#define LVDS_DMLCR1	0x000C  /* data mapping lsb configuration register 1	*/
#define LVDS_DMMCR1	0x0010  /* data mapping msb configuration register 1	*/
#define LVDS_DMLCR2	0x0014  /* data mapping lsb configuration register 2	*/
#define LVDS_DMMCR2	0x0018  /* data mapping msb configuration register 2	*/
#define LVDS_DMLCR3	0x001C  /* data mapping lsb configuration register 3	*/
#define LVDS_DMMCR3	0x0020  /* data mapping msb configuration register 3	*/
#define LVDS_DMLCR4	0x0024  /* data mapping lsb configuration register 4	*/
#define LVDS_DMMCR4	0x0028  /* data mapping msb configuration register 4	*/
#define LVDS_CDL1CR	0x002C  /* channel distrib link 1 configuration register	*/
#define LVDS_CDL2CR	0x0030  /* channel distrib link 2 configuration register	*/

#define CDL1CR_DEFAULT	0x04321	/* Default value for CDL1CR */
#define CDL2CR_DEFAULT	0x59876	/* Default value for CDL2CR */

/* LVDS Wrapper registers */
#define LVDS_WCLKCR	0x11B0  /* Wrapper clock control register */

#define LVDS_HWCFGR	0x1FF0  /* HW configuration register	*/
#define LVDS_VERR	0x1FF4  /* Version register	*/
#define LVDS_IPIDR	0x1FF8  /* Identification register	*/
#define LVDS_SIDR	0x1FFC  /* Size Identification register	*/

#define CR_LVDSEN	BIT(0)  /* LVDS PHY Enable */
#define CR_HSPOL	BIT(1)  /* HS Polarity (horizontal sync) */
#define CR_VSPOL	BIT(2)  /* VS Polarity (vertical sync) */
#define CR_DEPOL	BIT(3)  /* DE Polarity (data enable) */
#define CR_CI		BIT(4)  /* Control Internal (software controlled bit) */
#define CR_LKMOD	BIT(5)  /* Link Mode, for both Links */
#define CR_LKPHA	BIT(6)  /* Link Phase, for both Links */
#define CR_LK1POL	GENMASK(20, 16)  /* Link-1 output Polarity */
#define CR_LK2POL	GENMASK(25, 21)  /* Link-2 output Polarity */

#define DMMCRx_MAP0	GENMASK(4, 0)
#define DMMCRx_MAP1	GENMASK(9, 5)
#define DMMCRx_MAP2	GENMASK(14, 10)
#define DMMCRx_MAP3	GENMASK(19, 15)
#define DMLCRx_MAP4	GENMASK(4, 0)
#define DMLCRx_MAP5	GENMASK(9, 5)
#define DMLCRx_MAP6	GENMASK(14, 10)

#define CDLCRx_DISTR0	GENMASK(3, 0)
#define CDLCRx_DISTR1	GENMASK(7, 4)
#define CDLCRx_DISTR2	GENMASK(11, 8)
#define CDLCRx_DISTR3	GENMASK(15, 12)
#define CDLCRx_DISTR4	GENMASK(19, 16)

#define FREF_INDEX	0
#define NDIV_INDEX	1
#define FPFD_INDEX	2
#define MDIV_INDEX	3
#define FVCO_INDEX	4
#define BDIV_INDEX	5
#define FBIT_INDEX	6
#define FLS_INDEX	7
#define FDP_INDEX	8

#define PHY_GCR_BIT_CLK_OUT	BIT(0)
#define PHY_GCR_LS_CLK_OUT	BIT(4)
#define PHY_GCR_DP_CLK_OUT	BIT(8)
#define PHY_GCR_RSTZ		BIT(24)
#define PHY_GCR_DIV_RSTN	BIT(25)

#define PHY_PxPLLTESTCR_TDIV	GENMASK(25, 16)
#define PHY_PxPLLCR2_NDIV	GENMASK(25, 16)
#define PHY_PxPLLCR2_BDIV	GENMASK(9, 0)
#define PHY_PxPLLSDCR1_MDIV	GENMASK(9, 0)

#define PLL_EN		BIT(0)
#define PLL_LOCK	BIT(0)
#define CM_EN_DL	(BIT(28) | BIT(20) | BIT(12) | BIT(4))
#define CM_EN_DL4	BIT(4)
#define VM_EN_DL	(BIT(16) | BIT(12) | BIT(8) | BIT(4) | BIT(0))
#define EN_BIAS_DL	(BIT(16) | BIT(12) | BIT(8) | BIT(4) | BIT(0))
#define EN_DIG_DL	GENMASK(4, 0)
#define BIAS_EN		BIT(28)
#define POWER_OK	BIT(12)

#define WCLKCR_SLV_CLKPIX_SEL	BIT(0)
#define WCLKCR_SRCSEL		BIT(8)

/* Sleep & timeout for pll lock/unlock */
#define SLEEP_US	1000
#define TIMEOUT_US	200000

#define PHY_SLV_OFS	0x100

/*
 * The link phase defines whether an ODD pixel is carried over together with
 * the next EVEN pixel or together with the previous EVEN pixel.
 *
 * LVDS_DUAL_LINK_EVEN_ODD_PIXELS (LKPHA = 0)
 *
 * ,--------.  ,--------.  ,--------.  ,--------.  ,---------.
 * | ODD  LK \/ PIXEL  3 \/ PIXEL  1 \/ PIXEL' 1 \/ PIXEL' 3 |
 * | EVEN LK /\ PIXEL  2 /\ PIXEL' 0 /\ PIXEL' 2 /\ PIXEL' 4 |
 * `--------'  `--------'  `--------'  `--------'  `---------'
 *
 * LVDS_DUAL_LINK_ODD_EVEN_PIXELS (LKPHA = 1)
 *
 * ,--------.  ,--------.  ,--------.  ,--------.  ,---------.
 * | ODD  LK \/ PIXEL  3 \/ PIXEL  1 \/ PIXEL' 1 \/ PIXEL' 3 |
 * | EVEN LK /\ PIXEL  4 /\ PIXEL  2 /\ PIXEL' 0 /\ PIXEL' 2 |
 * `--------'  `--------'  `--------'  `--------'  `---------'
 *
 */
enum lvds_link_type {
	LVDS_SINGLE_LINK_MASTER = 0,
	LVDS_SINGLE_LINK_SLAVE = 1,
	LVDS_DUAL_LINK_EVEN_ODD_PIXELS = 2,
	LVDS_DUAL_LINK_ODD_EVEN_PIXELS = 3,
};

enum lvds_pixel {
	PIX_R_0		= 0x00,
	PIX_R_1		= 0x01,
	PIX_R_2		= 0x02,
	PIX_R_3		= 0x03,
	PIX_R_4		= 0x04,
	PIX_R_5		= 0x05,
	PIX_R_6		= 0x06,
	PIX_R_7		= 0x07,
	PIX_G_0		= 0x08,
	PIX_G_1		= 0x09,
	PIX_G_2		= 0x0A,
	PIX_G_3		= 0x0B,
	PIX_G_4		= 0x0C,
	PIX_G_5		= 0x0D,
	PIX_G_6		= 0x0E,
	PIX_G_7		= 0x0F,
	PIX_B_0		= 0x10,
	PIX_B_1		= 0x11,
	PIX_B_2		= 0x12,
	PIX_B_3		= 0x13,
	PIX_B_4		= 0x14,
	PIX_B_5		= 0x15,
	PIX_B_6		= 0x16,
	PIX_B_7		= 0x17,
	PIX_H_S		= 0x18,
	PIX_V_S		= 0x19,
	PIX_D_E		= 0x1A,
	PIX_C_E		= 0x1B,
	PIX_C_I		= 0x1C,
	PIX_TOG		= 0x1D,
	PIX_ONE		= 0x1E,
	PIX_ZER		= 0x1F,
};

struct phy_offsets {
	u32 PxGCR;	/* Global Control Register	*/
	u32 PxCMCR1;    /* Current Mode Control Register 1 */
	u32 PxCMCR2;    /* Current Mode Control Register 2 */
	u32 PxSCR;      /* Serial Control Register	*/
	u32 PxBCR1;     /* Bias Control Register 1	*/
	u32 PxBCR2;     /* Bias Control Register 2	*/
	u32 PxBCR3;     /* Bias Control Register 3	*/
	u32 PxMPLCR;    /* Monitor PLL Lock Control Register */
	u32 PxDCR;      /* Debug Control Register	*/
	u32 PxSSR1;     /* Spare Status Register 1	*/
	u32 PxCFGCR;    /* Configuration Control Register */
	u32 PxPLLCR1;   /* PLL_MODE 1 Control Register	*/
	u32 PxPLLCR2;   /* PLL_MODE 2 Control Register	*/
	u32 PxPLLSR;    /* PLL Status Register	*/
	u32 PxPLLSDCR1; /* PLL_SD_1 Control Register	*/
	u32 PxPLLSDCR2; /* PLL_SD_2 Control Register	*/
	u32 PxPLLTWGCR1;/* PLL_TWG_1 Control Register	*/
	u32 PxPLLTWGCR2;/* PLL_TWG_2 Control Register	*/
	u32 PxPLLCPCR;	/* PLL_CP Control Register	*/
	u32 PxPLLTESTCR;/* PLL_TEST Control Register	*/
};

struct lvds_phy_info {
	u32 base;
	struct phy_offsets ofs;
};

static struct lvds_phy_info lvds_phy_16ff_master = {
	.base = 0x1000,
	.ofs = {
		.PxGCR = 0x0,
		.PxCMCR1 = 0xC,
		.PxCMCR2 = 0x10,
		.PxSCR = 0x20,
		.PxBCR1 = 0x2C,
		.PxBCR2 = 0x30,
		.PxBCR3 = 0x34,
		.PxMPLCR = 0x64,
		.PxDCR = 0x84,
		.PxSSR1 = 0x88,
		.PxCFGCR = 0xA0,
		.PxPLLCR1 = 0xC0,
		.PxPLLCR2 = 0xC4,
		.PxPLLSR = 0xC8,
		.PxPLLSDCR1 = 0xCC,
		.PxPLLSDCR2 = 0xD0,
		.PxPLLTWGCR1 = 0xD4,
		.PxPLLTWGCR2 = 0xD8,
		.PxPLLCPCR = 0xE0,
		.PxPLLTESTCR = 0xE8,
	}
};

static struct lvds_phy_info lvds_phy_16ff_slave = {
	.base = 0x1100,
	.ofs = {
		.PxGCR = 0x0,
		.PxCMCR1 = 0xC,
		.PxCMCR2 = 0x10,
		.PxSCR = 0x20,
		.PxBCR1 = 0x2C,
		.PxBCR2 = 0x30,
		.PxBCR3 = 0x34,
		.PxMPLCR = 0x64,
		.PxDCR = 0x84,
		.PxSSR1 = 0x88,
		.PxCFGCR = 0xA0,
		.PxPLLCR1 = 0xC0,
		.PxPLLCR2 = 0xC4,
		.PxPLLSR = 0xC8,
		.PxPLLSDCR1 = 0xCC,
		.PxPLLSDCR2 = 0xD0,
		.PxPLLTWGCR1 = 0xD4,
		.PxPLLTWGCR2 = 0xD8,
		.PxPLLCPCR = 0xE0,
		.PxPLLTESTCR = 0xE8,
	}
};

struct lvds_clk_data {
	const char *clk_name;
	const struct clk_ops *clk_ops;
	const char * const *clk_src;
	unsigned int max_clk_src;
};

struct stm_lvds {
	void __iomem *base;
	struct device *dev;
	struct clk *pclk;		/* APB bus clock */
	struct clk *pllref_clk;		/* HSE / Flexclkgen */
	struct clk_hw lvds_ck_px;	/* Pixel clock */
	u32 pixel_clock_rate;		/* Pixel clock rate */
	struct {
		u32 hw_version;
		u32 link_type;
	} config;
	struct lvds_phy_info *phy_master;
	struct lvds_phy_info *phy_slave;

	struct drm_bridge	lvds_bridge;
	struct drm_bridge	*next_bridge;
	struct drm_connector	connector;
	struct drm_encoder	*encoder;
	struct drm_panel	*panel;

	struct regulator *vdd_supply;
	struct regulator *vdda18_supply;
};

#define bridge_to_stm_lvds(b) \
	container_of(b, struct stm_lvds, lvds_bridge)

#define connector_to_stm_lvds(c) \
	container_of(c, struct stm_lvds, connector)

#define lvds_is_dual_link(lvds) \
	({	\
	typeof(lvds) __lvds = (lvds);	\
	__lvds == LVDS_DUAL_LINK_EVEN_ODD_PIXELS ||	\
	__lvds == LVDS_DUAL_LINK_ODD_EVEN_PIXELS;	\
	})

static inline void lvds_write(struct stm_lvds *lvds, u32 reg, u32 val)
{
	writel(val, lvds->base + reg);
}

static inline u32 lvds_read(struct stm_lvds *lvds, u32 reg)
{
	return readl(lvds->base + reg);
}

static inline void lvds_set(struct stm_lvds *lvds, u32 reg, u32 mask)
{
	lvds_write(lvds, reg, lvds_read(lvds, reg) | mask);
}

static inline void lvds_clear(struct stm_lvds *lvds, u32 reg, u32 mask)
{
	lvds_write(lvds, reg, lvds_read(lvds, reg) & ~mask);
}

static inline void lvds_update_bits(struct stm_lvds *lvds, u32 reg,
				    u32 mask, u32 val)
{
	lvds_write(lvds, reg, (lvds_read(lvds, reg) & ~mask) | val);
}

/*
 * Expected JEIDA-RGB888 data to be sent in LSB format
 *	    bit6 ............................bit0
 * CHAN0   {ONE, ONE, ZERO, ZERO, ZERO, ONE, ONE}
 * CHAN1   {G2,  R7,  R6,   R5,   R4,   R3,  R2}
 * CHAN2   {B3,  B2,  G7,   G6,   G5,   G4,  G3}
 * CHAN3   {DE,  VS,  HS,   B7,   B6,   B5,  B4}
 * CHAN4   {CE,  B1,  B0,   G1,   G0,   R1,  R0}
 */
const enum lvds_pixel lvds_bitmap_jeida_rgb888[5][7] = {
	{
		PIX_ONE, PIX_ONE, PIX_ZER, PIX_ZER, PIX_ZER, PIX_ONE, PIX_ONE
	}, {
		PIX_G_2, PIX_R_7, PIX_R_6, PIX_R_5, PIX_R_4, PIX_R_3, PIX_R_2
	}, {
		PIX_B_3, PIX_B_2, PIX_G_7, PIX_G_6, PIX_G_5, PIX_G_4, PIX_G_3
	}, {
		PIX_D_E, PIX_V_S, PIX_H_S, PIX_B_7, PIX_B_6, PIX_B_5, PIX_B_4
	}, {
		PIX_C_E, PIX_B_1, PIX_B_0, PIX_G_1, PIX_G_0, PIX_R_1, PIX_R_0
	}
};

/*
 * Expected VESA-RGB888 data to be sent in LSB format
 *	    bit6 ............................bit0
 * CHAN0   {ONE, ONE, ZERO, ZERO, ZERO, ONE, ONE}
 * CHAN1   {G0,  R5,  R4,   R3,   R2,   R1,  R0}
 * CHAN2   {B1,  B0,  G5,   G4,   G3,   G2,  G1}
 * CHAN3   {DE,  VS,  HS,   B5,   B4,   B3,  B2}
 * CHAN4   {CE,  B7,  B6,   G7,   G6,   R7,  R6}
 */
const enum lvds_pixel lvds_bitmap_vesa_rgb888[5][7] = {
	{
		PIX_ONE, PIX_ONE, PIX_ZER, PIX_ZER, PIX_ZER, PIX_ONE, PIX_ONE
	}, {
		PIX_G_0, PIX_R_5, PIX_R_4, PIX_R_3, PIX_R_2, PIX_R_1, PIX_R_0
	}, {
		PIX_B_1, PIX_B_0, PIX_G_5, PIX_G_4, PIX_G_3, PIX_G_2, PIX_G_1
	}, {
		PIX_D_E, PIX_V_S, PIX_H_S, PIX_B_5, PIX_B_4, PIX_B_3, PIX_B_2
	}, {
		PIX_C_E, PIX_B_7, PIX_B_6, PIX_G_7, PIX_G_6, PIX_R_7, PIX_R_6
	}
};

static int lvds_pll_enable(struct stm_lvds *lvds, struct lvds_phy_info *phy)
{
	u32 lvds_gcr;
	int ret, val;

	/* PLL lock timing control for the monitor unmask after startup (pll_en) */
	/* Adjust the value so that the masking window is opened at start-up */
	/* MST_MON_PLL_LOCK_UNMASK_TUNE */
	lvds_write(lvds, phy->base + phy->ofs.PxMPLCR, (0x200 - 0x160) << 16);

	lvds_write(lvds, phy->base + phy->ofs.PxBCR2, BIAS_EN);

	lvds_gcr = PHY_GCR_DP_CLK_OUT | PHY_GCR_LS_CLK_OUT | PHY_GCR_BIT_CLK_OUT;
	lvds_set(lvds, phy->base + phy->ofs.PxGCR, lvds_gcr);

	/* TODO hardcoded values for now */
	lvds_set(lvds, phy->base + phy->ofs.PxPLLTESTCR, BIT(8) /* PLL_TEST_DIV_EN */);
	lvds_set(lvds, phy->base + phy->ofs.PxPLLCR1, BIT(8) /* PLL_DIVIDERS_ENABLE */);

	lvds_set(lvds, phy->base + phy->ofs.PxSCR, BIT(16) /* SER_DATA_OK */);

	/* Enable the LVDS PLL & wait for its lock */
	lvds_set(lvds, phy->base + phy->ofs.PxPLLCR1, PLL_EN);
	ret = readl_poll_timeout_atomic(lvds->base + phy->base + phy->ofs.PxPLLSR,
					val, val & PLL_LOCK,
					SLEEP_US, TIMEOUT_US);
	if (ret)
		DRM_ERROR("!TIMEOUT! waiting PLL, let's continue\n");

	/* Select MST PHY clock as pixel clock for the LDITX instead of FREF */
	/* WCLKCR_SLV_CLKPIX_SEL is for dual link */
	lvds_write(lvds, LVDS_WCLKCR, WCLKCR_SLV_CLKPIX_SEL);

	/* JF Duret */
	lvds_set(lvds, phy->ofs.PxPLLTESTCR, BIT(0));

	return ret;
}

/* Integer mode */
#define EN_SD		0
#define EN_TWG		0
#define DOWN_SPREAD	0
#define TEST_DIV	70

static int pll_get_clkout_khz(int clkin_khz, int bdiv, int mdiv, int ndiv)
{
	int divisor = ndiv * bdiv;

	/* Prevents from division by 0 */
	if (!divisor)
		return 0;

	return clkin_khz * mdiv / divisor;
}

#define NDIV_MIN	2
#define NDIV_MAX	6
#define BDIV_MIN	2
#define BDIV_MAX	6
#define MDIV_MIN	1
#define MDIV_MAX	1023

static int lvds_pll_get_params(struct stm_lvds *lvds,
			       unsigned int clkin_khz, unsigned int clkout_khz,
			       unsigned int *bdiv, unsigned int *mdiv, unsigned int *ndiv)
{
	int i, o, n;
	int delta, best_delta; /* all in khz */

	/* Early checks preventing division by 0 & odd results */
	if (clkin_khz <= 0 || clkout_khz <= 0)
		return -EINVAL;

	best_delta = 1000000; /* big started value (1000000khz) */

	for (i = NDIV_MIN; i <= NDIV_MAX; i++) {
		for (o = BDIV_MIN; o <= BDIV_MAX; o++) {
			n = DIV_ROUND_CLOSEST(i * o * clkout_khz, clkin_khz);
			/* Check ndiv according to vco range */
			if (n < MDIV_MIN || n > MDIV_MAX)
				continue;
			/* Check if new delta is better & saves parameters */
			delta = pll_get_clkout_khz(clkin_khz, i, n, o) - clkout_khz;
			if (delta < 0)
				delta = -delta;
			if (delta < best_delta) {
				*ndiv = i;
				*mdiv = n;
				*bdiv = o;
				best_delta = delta;
			}
			/* fast return in case of "perfect result" */
			if (!delta)
				return 0;
		}
	}

	return 0;
}

static void lvds_pll_config(struct stm_lvds *lvds, struct lvds_phy_info *phy)
{
	/* Set PLL Slv & Mst configs and timings */
	struct clk_hw *hwclk;
	unsigned int pll_in_khz, bdiv, mdiv, ndiv;
	int multiplier;

	/*
	 * The LVDS PLL is made of a pre-divider and a multiplier (strangely
	 * enough called M and N respectively), followed by a post-divider E.
	 *
	 *          ,------.         ,-----.     ,-----.
	 * Fref --> | NDIV | -Fpdf-> | PFD | --> | VCO | --> Fvco
	 *          `------'     ,-> |     |     `-----'  |
	 *                       |   `-----'              |
	 *                       |         ,------.       |
	 *                       `-------- | MDIV | <-----'
	 *                                 `------'
	 *
	 * The clock output by the PLL is then further divided by a programmable
	 * divider DIV to achieve the desired target frequency. Finally, an
	 * optional fixed /7 divider is used to convert the bit clock to a pixel
	 * clock (as LVDS transmits 7 bits per lane per clock sample).
	 *
	 *          ,------.     ,-----.     ,-------.     |\
	 * Fvco --> | BDIV | --> | 1/7 | --> | 1/3.5 | --> | |
	 *          `------'  |  `-----'     `-------'     | | --> dot clock
	 *                    `--------------------------> | |
	 *                                                 |/
	 *
	 * The /7 divider is optional, it is enabled when the LVDS PLL is used
	 * to drive the LVDS encoder, and disabled when used to generate a dot
	 * clock for the display unit RGB output, without using the LVDS
	 * encoder.
	 *
	 * The PLL allowed input frequency range is 12 MHz to 192 MHz.
	 */

	/* TODO resolv blocking subroutine */
	hwclk = __clk_get_hw(lvds->pllref_clk);
	if (!hwclk)
		return;

	pll_in_khz = clk_hw_get_rate(hwclk) / 1000;

	if (lvds_is_dual_link(lvds->config.link_type))
		multiplier = 2;
	else
		multiplier = 1;

	lvds_pll_get_params(lvds, pll_in_khz,
			    lvds->pixel_clock_rate * 7 / 1000 / multiplier,
			    &bdiv, &mdiv, &ndiv);

	/* MST_PLL_INPUT_DIV */
	lvds_write(lvds, phy->base + phy->ofs.PxPLLCR2, ndiv << 16);
	/* MST_PLL_BIT_DIV */
	lvds_set(lvds, phy->base + phy->ofs.PxPLLCR2, bdiv);
	/* MST_PLL_SD_INT_RATIO */
	lvds_write(lvds, phy->base + phy->ofs.PxPLLSDCR1, mdiv);
	/* MST_PLL_TEST_DIV_SETTINGS */
	lvds_write(lvds, phy->base + phy->ofs.PxPLLTESTCR, TEST_DIV << 16);

	/*
	 * PLL integer mode:
	 *	- MST_PLL_TWG_STEP = MST_PLL_SD_INT_RATIO
	 *	- EN_TWG = 0
	 *	- EN_SD = 0
	 *	- DOWN_SPREAD = 0
	 *
	 * PLL fractional mode:
	 *	- EN_TWG = 0
	 *	- EN_SD = 1
	 *	- DOWN_SPREAD = 0
	 */

	/* For now, PLL just need to be in integer mode */
	lvds_clear(lvds, phy->base + phy->ofs.PxPLLCR1, EN_TWG | EN_SD); /* Disable TWG and SD */

	/* Power up bias and PLL dividers */
	lvds_set(lvds, phy->base + phy->ofs.PxDCR, POWER_OK);

	lvds_set(lvds, phy->base + phy->ofs.PxCMCR1, CM_EN_DL);
	lvds_set(lvds, phy->base + phy->ofs.PxCMCR2, CM_EN_DL4);

	/* JF Duret */
	lvds_set(lvds, phy->base + phy->ofs.PxPLLCPCR, 0x1);

	lvds_set(lvds, phy->base + phy->ofs.PxBCR3, VM_EN_DL);

	lvds_set(lvds, phy->base + phy->ofs.PxBCR1, EN_BIAS_DL);

	lvds_set(lvds, phy->base + phy->ofs.PxCFGCR, EN_DIG_DL);
}

/* TODO enhance & make clearer logic */
static int lvds_pixel_clk_enable(struct clk_hw *hw)
{
	/* PLL enable here */
	struct stm_lvds *lvds = container_of(hw, struct stm_lvds, lvds_ck_px);
	struct lvds_phy_info *phy;
	int ret;

	ret = clk_prepare_enable(lvds->pclk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(lvds->pllref_clk);
	if (ret)
		goto err_pclk;

	/*
	 * In case we are operating in dual link, PHY Slv is set before PHY
	 * Mst.
	 */
	if (lvds->phy_slave) {
		phy = lvds->phy_slave;

		lvds_set(lvds, phy->base + phy->ofs.PxGCR, PHY_GCR_DIV_RSTN | PHY_GCR_RSTZ);
		lvds_pll_config(lvds, phy);

		ret = lvds_pll_enable(lvds, phy);
		if (ret) {
			DRM_ERROR("Unable to enable PHY PLL Slv: %d\n", ret);
			goto err_pllref;
		}
	}

	if (lvds->phy_master) {
		phy = lvds->phy_master;

		/* Release LVDS PHY from reset mode */
		lvds_set(lvds, phy->base + phy->ofs.PxGCR, PHY_GCR_DIV_RSTN | PHY_GCR_RSTZ);
		lvds_pll_config(lvds, phy);

		ret = lvds_pll_enable(lvds, phy);
		if (ret) {
			DRM_ERROR("Unable to enable PHY PLL Mst: %d\n", ret);
			goto err_pllref;
		}
	}

	return 0;

err_pllref:
	clk_disable_unprepare(lvds->pllref_clk);
err_pclk:
	clk_disable_unprepare(lvds->pclk);

	return ret;
}

static void lvds_pixel_clk_disable(struct clk_hw *hw)
{
	/* PLL disable here */
	struct stm_lvds *lvds = container_of(hw, struct stm_lvds, lvds_ck_px);

	/* TODO Disable D-PHY clocks and digital */
//	lvds_clear(lvds, lvds->phy_master->base + lvds->phy_master->ofs.PxGCR,
//		   (PHY_GCR_DP_CLK_OUT | PHY_GCR_LS_CLK_OUT | PHY_GCR_BIT_CLK_OUT));

	/* Assert LVDS PHY Master & Slave in reset mode */
	if (lvds->phy_master)
		lvds_clear(lvds, lvds->phy_master->base + lvds->phy_master->ofs.PxGCR,
			   PHY_GCR_DIV_RSTN | PHY_GCR_RSTZ);

	if (lvds->phy_slave)
		lvds_clear(lvds, lvds->phy_slave->base + lvds->phy_slave->ofs.PxGCR,
			   PHY_GCR_DIV_RSTN | PHY_GCR_RSTZ);

	clk_disable_unprepare(lvds->pllref_clk);
	clk_disable_unprepare(lvds->pclk);
}

static unsigned long lvds_pixel_clk_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct stm_lvds *lvds = container_of(hw, struct stm_lvds, lvds_ck_px);
	unsigned int pll_in_khz, bdiv, mdiv, ndiv;
	int multiplier;

	pll_in_khz = (unsigned int)(parent_rate / 1000);

	if (lvds_is_dual_link(lvds->config.link_type))
		multiplier = 2;
	else
		multiplier = 1;

	lvds_pll_get_params(lvds, pll_in_khz,
			    lvds->pixel_clock_rate * 7 / 1000 / multiplier,
			    &bdiv, &mdiv, &ndiv);

	/* X7 because for each pixel in 1 lane there is 7 bits
	 * We want pixclk, not bitclk
	 */
	lvds->pixel_clock_rate = (unsigned long)pll_get_clkout_khz(pll_in_khz, bdiv, mdiv, ndiv)
					 * 1000 * multiplier / 7;

	return (unsigned long)lvds->pixel_clock_rate;
}

static long lvds_pixel_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	struct stm_lvds *lvds = container_of(hw, struct stm_lvds, lvds_ck_px);
	unsigned int pll_in_khz, bdiv, mdiv, ndiv;
	const struct drm_display_mode *mode;
	const struct drm_connector *connector;
	int multiplier;

	connector = &lvds->connector;
	if (!connector)
		return -EINVAL;

	if (list_empty(&connector->modes)) {
		dev_dbg(lvds->dev, "connector: empty modes list\n");
		return -EINVAL;
	}

	mode = list_first_entry(&connector->modes,
				struct drm_display_mode, head);

	pll_in_khz = (unsigned int)(*parent_rate / 1000);

	if (lvds_is_dual_link(lvds->config.link_type))
		multiplier = 2;
	else
		multiplier = 1;

	lvds_pll_get_params(lvds, pll_in_khz, mode->clock * 7 / multiplier, &bdiv, &mdiv, &ndiv);

	/* X7 because for each pixel in 1 lane there is 7 bits
	 * We want pixclk, not bitclk
	 */
	lvds->pixel_clock_rate = (unsigned long)pll_get_clkout_khz(pll_in_khz, bdiv, mdiv, ndiv)
					 * 1000 * multiplier / 7;

	return lvds->pixel_clock_rate;
}

static const struct clk_ops lvds_pixel_clk_ops = {
	.enable = lvds_pixel_clk_enable,
	.disable = lvds_pixel_clk_disable,
	.recalc_rate = lvds_pixel_clk_recalc_rate,
	.round_rate = lvds_pixel_clk_round_rate,
};

static const struct lvds_clk_data clk_data = {
	.clk_name = "clk_pix_lvds",
	.clk_ops = &lvds_pixel_clk_ops,
	.clk_src = (const char * []) {"ck_ker_lvdsphy"},
	.max_clk_src = 1,
};

static void lvds_pixel_clk_unregister(void *data)
{
	struct stm_lvds *lvds = data;

	of_clk_del_provider(lvds->dev->of_node);
	clk_hw_unregister(&lvds->lvds_ck_px);
}

static int lvds_pixel_clk_register(struct stm_lvds *lvds)
{
	struct device_node *node = lvds->dev->of_node;
	struct clk_init_data init = {
		.name = clk_data.clk_name,
		.ops = clk_data.clk_ops,
		.parent_names = clk_data.clk_src,
		.num_parents = clk_data.max_clk_src,
		.flags = CLK_IGNORE_UNUSED,
	};
	int ret;

	lvds->lvds_ck_px.init = &init;

	/* set the rate by default at 148500000 */
	lvds->pixel_clock_rate = 148500000;

	ret = clk_hw_register(lvds->dev, &lvds->lvds_ck_px);
	if (ret)
		return ret;

	ret = of_clk_add_hw_provider(node, of_clk_hw_simple_get,
				     &lvds->lvds_ck_px);
	if (ret)
		clk_hw_unregister(&lvds->lvds_ck_px);

	return ret;
}

/* ------------------------------------------------------------------------- */
static void lvds_config_data_mapping(struct stm_lvds *lvds)
{
	const struct drm_display_info *info;
	u32 lvds_dmlcr[5], lvds_dmmcr[5]; /* 4 data lanes + 1 clk lane */
	int i;

	info = &(&lvds->connector)->display_info;
	if (!info->num_bus_formats || !info->bus_formats) {
		dev_warn(lvds->dev, "No LVDS bus format reported\n");
		return;
	}

/*      Mode mirror : TODO
 *      info->bus_flags & DRM_BUS_FLAG_DATA_LSB_TO_MSB
 */

	switch (info->bus_formats[0]) {
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG: /* VESA-RGB666 */
		DRM_WARN("Pixel format with data mapping not yet compatible.\n");
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG: /* VESA-RGB888 */
		for (i = 0; i < 5; i++) {
			lvds_dmlcr[i] = ((lvds_bitmap_vesa_rgb888[i][0])
					 + (lvds_bitmap_vesa_rgb888[i][1] << 5)
					 + (lvds_bitmap_vesa_rgb888[i][2] << 10)
					 + (lvds_bitmap_vesa_rgb888[i][3] << 15));
			lvds_dmmcr[i] = ((lvds_bitmap_vesa_rgb888[i][4])
					 + (lvds_bitmap_vesa_rgb888[i][5] << 5)
					 + (lvds_bitmap_vesa_rgb888[i][6] << 10));
		}
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA: /* JEIDA-RGB888 */
		for (i = 0; i < 5; i++) {
			lvds_dmlcr[i] = ((lvds_bitmap_jeida_rgb888[i][0])
					+ (lvds_bitmap_jeida_rgb888[i][1] << 5)
					+ (lvds_bitmap_jeida_rgb888[i][2] << 10)
					+ (lvds_bitmap_jeida_rgb888[i][3] << 15));
			lvds_dmmcr[i] = ((lvds_bitmap_jeida_rgb888[i][4])
					+ (lvds_bitmap_jeida_rgb888[i][5] << 5)
					+ (lvds_bitmap_jeida_rgb888[i][6] << 10));
		}
		break;
	default:
		dev_warn(lvds->dev, "Unsupported LVDS bus format 0x%04x\n",
			 info->bus_formats[0]);
	}

	/* Write registers at the end of computations */
	/* TODO use for loop */
	lvds_write(lvds, LVDS_DMLCR0, lvds_dmlcr[0]);
	lvds_write(lvds, LVDS_DMMCR0, lvds_dmmcr[0]);
	lvds_write(lvds, LVDS_DMLCR1, lvds_dmlcr[1]);
	lvds_write(lvds, LVDS_DMMCR1, lvds_dmmcr[1]);
	lvds_write(lvds, LVDS_DMLCR2, lvds_dmlcr[2]);
	lvds_write(lvds, LVDS_DMMCR2, lvds_dmmcr[2]);
	lvds_write(lvds, LVDS_DMLCR3, lvds_dmlcr[3]);
	lvds_write(lvds, LVDS_DMMCR3, lvds_dmmcr[3]);
	lvds_write(lvds, LVDS_DMLCR4, lvds_dmlcr[4]);
	lvds_write(lvds, LVDS_DMMCR4, lvds_dmmcr[4]);
}

static int lvds_config_mode(struct stm_lvds *lvds)
{
	const struct drm_display_mode *mode;
	const struct drm_connector *connector;
	u32 bus_flags, lvds_cr, lvds_cdl1cr, lvds_cdl2cr;

	lvds_cr = 0;
	lvds_cdl1cr = 0;
	lvds_cdl2cr = 0;

	connector = &lvds->connector;
	if (!connector)
		return -EINVAL;

	if (list_empty(&connector->modes)) {
		dev_dbg(lvds->dev, "connector: empty modes list\n");
		return -EINVAL;
	}

	bus_flags = connector->display_info.bus_flags;
	mode = list_first_entry(&connector->modes,
				struct drm_display_mode, head);

	lvds_clear(lvds, LVDS_CR, CR_LKMOD);
	lvds_clear(lvds, LVDS_CDL1CR, CDLCRx_DISTR0 | CDLCRx_DISTR1 | CDLCRx_DISTR2
					| CDLCRx_DISTR3 | CDLCRx_DISTR4);
	lvds_clear(lvds, LVDS_CDL2CR, CDLCRx_DISTR0 | CDLCRx_DISTR1 | CDLCRx_DISTR2
					| CDLCRx_DISTR3 | CDLCRx_DISTR4);

	/* Set channel distribution */
	/* TODO Hardcoded values for now */
	if (lvds->phy_master)
		lvds_cdl1cr = CDL1CR_DEFAULT;

	if (lvds->phy_slave) {
		lvds_cr |= CR_LKMOD;
		lvds_cdl2cr = CDL2CR_DEFAULT;
	}

	/* Set signal polarity */
	if (bus_flags & DRM_BUS_FLAG_DE_LOW)
		lvds_cr |= CR_DEPOL;

	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		lvds_cr |= CR_HSPOL;

	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		lvds_cr |= CR_VSPOL;

	switch (lvds->config.link_type) {
	case LVDS_DUAL_LINK_EVEN_ODD_PIXELS: /* LKPHA = 0 */
		lvds_cr &= ~CR_LKPHA;
		break;
	case LVDS_DUAL_LINK_ODD_EVEN_PIXELS: /* LKPHA = 1 */
		lvds_cr |= CR_LKPHA;
		break;
	default:
		dev_warn(lvds->dev, "No phase precised, setting default\n");
		lvds_cr &= ~CR_LKPHA;
		break;
	}

	/* Write config to registers */
	lvds_set(lvds, LVDS_CR, lvds_cr);
	lvds_write(lvds, LVDS_CDL1CR, lvds_cdl1cr);
	lvds_write(lvds, LVDS_CDL2CR, lvds_cdl2cr);

	/* Set Data Mapping */
	lvds_config_data_mapping(lvds);

	return 0;
}

/* ------------------------------------------------------------------------- */
static int lvds_connector_get_modes(struct drm_connector *connector)
{
	struct stm_lvds *lvds = connector_to_stm_lvds(connector);
	int ret;

	ret = drm_panel_get_modes(lvds->panel, connector);

	return ret;
}

static int lvds_connector_atomic_check(struct drm_connector *connector,
				       struct drm_atomic_state *state)
{
	struct stm_lvds *lvds = connector_to_stm_lvds(connector);
	const struct drm_display_mode *panel_mode;
	struct drm_connector_state *conn_state;
	struct drm_crtc_state *crtc_state;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (!conn_state->crtc)
		return 0;

	if (list_empty(&connector->modes)) {
		dev_dbg(lvds->dev, "connector: empty modes list\n");
		return -EINVAL;
	}

	panel_mode = list_first_entry(&connector->modes,
				      struct drm_display_mode, head);

	/* We're not allowed to modify the resolution. */
	crtc_state = drm_atomic_get_crtc_state(state, conn_state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	if (crtc_state->mode.hdisplay != panel_mode->hdisplay ||
	    crtc_state->mode.vdisplay != panel_mode->vdisplay)
		return -EINVAL;

	/* The flat panel mode is fixed, just copy it to the adjusted mode. */
	drm_mode_copy(&crtc_state->adjusted_mode, panel_mode);

	return 0;
}

static const struct drm_connector_helper_funcs lvds_conn_helper_funcs = {
	.get_modes = lvds_connector_get_modes,
	.atomic_check = lvds_connector_atomic_check,
};

static const struct drm_connector_funcs lvds_conn_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

/* ------------------------------------------------------------------------- */
static int lvds_attach(struct drm_bridge *bridge,
		       enum drm_bridge_attach_flags flags)
{
	struct stm_lvds *lvds = bridge_to_stm_lvds(bridge);
	struct drm_connector *connector = &lvds->connector;
	struct drm_encoder *encoder = bridge->encoder;
	int ret = -ENODEV;

	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found\n");
		return -ENODEV;
	}

	/* Set the encoder type as caller does not know it */
	bridge->encoder->encoder_type = DRM_MODE_ENCODER_LVDS;

	/* No cloning support */
	bridge->encoder->possible_clones = 0;

	/* If we have a next bridge just attach it. */
	if (lvds->next_bridge)
		return drm_bridge_attach(bridge->encoder, lvds->next_bridge,
					 bridge, flags);

	if (flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR) {
		DRM_ERROR("Fix bridge driver to make connector optional!");
		return -EINVAL;
	}

	/* Otherwise if we have a panel, create a connector. */
	if (!lvds->panel)
		return 0;

	ret = drm_connector_init(bridge->dev, connector,
				 &lvds_conn_funcs, DRM_MODE_CONNECTOR_LVDS);
	if (ret < 0)
		return ret;

	drm_connector_helper_add(connector, &lvds_conn_helper_funcs);

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret < 0)
		return ret;

	return 0;
}

static void lvds_detach(struct drm_bridge *bridge)
{}

static void __lvds_atomic_enable(struct drm_bridge *bridge,
				 struct drm_atomic_state *state,
				 struct drm_crtc *crtc,
				 struct drm_connector *connector)
{
	struct stm_lvds *lvds = bridge_to_stm_lvds(bridge);

	lvds_config_mode(lvds);

	/* Turn the output on. */
	lvds_set(lvds, LVDS_CR, CR_LVDSEN);

	if (lvds->panel) {
		drm_panel_prepare(lvds->panel);
		drm_panel_enable(lvds->panel);
	}
}

static void lvds_atomic_enable(struct drm_bridge *bridge,
			       struct drm_bridge_state *old_bridge_state)
{
	struct stm_lvds *lvds = bridge_to_stm_lvds(bridge);
	struct drm_atomic_state *state = old_bridge_state->base.state;
	struct drm_connector *connector;
	struct drm_crtc *crtc;
	int ret;

	if (!pm_runtime_active(lvds->dev)) {
		ret = pm_runtime_get_sync(lvds->dev);
		if (ret < 0) {
			DRM_ERROR("Failed to set mode, cannot get sync\n");
			return;
		}
	}

	connector = drm_atomic_get_new_connector_for_encoder(state,
							     bridge->encoder);
	crtc = drm_atomic_get_new_connector_state(state, connector)->crtc;

	__lvds_atomic_enable(bridge, state, crtc, connector);
}

static void lvds_atomic_disable(struct drm_bridge *bridge,
				struct drm_bridge_state *old_bridge_state)
{
	struct stm_lvds *lvds = bridge_to_stm_lvds(bridge);

	if (lvds->panel) {
		drm_panel_disable(lvds->panel);
		drm_panel_unprepare(lvds->panel);
	}

	/* Check the LVDS handle allocation ? */

	/* Disable LVDS module */
	lvds_clear(lvds, LVDS_CR, CR_LVDSEN);

	/* Shutdown the LVDS PLL */
	/*
	 * TODO: bug if PLL_EN cleared
	 * Do NOT uncomment this line
	 */
	//lvds_clear(lvds, lvds->phy_master->base + lvds->phy_master->ofs.PxPLLCR1, PLL_EN);
	pm_runtime_put(lvds->dev);
}

static const struct drm_bridge_funcs lvds_bridge_funcs = {
	.attach = lvds_attach,
	.detach = lvds_detach,
	.atomic_enable = lvds_atomic_enable,
	.atomic_disable = lvds_atomic_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
};

/* ------------------------------------------------------------------------- */
static int lvds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *port1, *port2, *remote;
	struct stm_lvds *lvds;
	struct reset_control *rstc;
	struct lvds_phy_info *phy;
	unsigned int pll_in_khz, bdiv, mdiv, ndiv;
	int multiplier, rate;
	int ret, dual_link;

	DRM_DEBUG_DRIVER("Probing LVDS driver...\n");

	lvds = devm_kzalloc(dev, sizeof(*lvds), GFP_KERNEL);
	if (!lvds)
		return -ENOMEM;

	lvds->dev = dev;

	ret = drm_of_find_panel_or_bridge(lvds->dev->of_node, 1, 0,
					  &lvds->panel, &lvds->next_bridge);
	if (ret) {
		dev_err_probe(dev, ret, "Panel not found\n");
		return ret;
	}

	lvds->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lvds->base)) {
		ret = PTR_ERR(lvds->base);
		DRM_ERROR("Unable to get regs %d\n", ret);
		return ret;
	}

	lvds->vdd_supply = devm_regulator_get(dev, "vdd");
	if (IS_ERR(lvds->vdd_supply)) {
		dev_err_probe(dev, ret, "Failed to request regulator\n");
		return PTR_ERR(lvds->vdd_supply);
	}

	lvds->vdda18_supply = devm_regulator_get(dev, "vdda18");
	if (IS_ERR(lvds->vdda18_supply)) {
		dev_err_probe(dev, ret, "Failed to request regulator\n");
		return  PTR_ERR(lvds->vdda18_supply);
	}

	/* To obtain a continuous display after the probe, reset shouldn't be done */
	if (!device_property_read_bool(dev, "default-on")) {
		rstc = devm_reset_control_get_exclusive(dev, NULL);

		if (IS_ERR(rstc)) {
			ret = PTR_ERR(rstc);
			return ret;
		}

		reset_control_assert(rstc);
		usleep_range(10, 20);
		reset_control_deassert(rstc);
	}

	port1 = of_graph_get_port_by_id(dev->of_node, 1);
	port2 = of_graph_get_port_by_id(dev->of_node, 2);
	dual_link = drm_of_lvds_get_dual_link_pixel_order(port1, port2);

	switch (dual_link) {
	case DRM_LVDS_DUAL_LINK_ODD_EVEN_PIXELS:
		lvds->config.link_type = LVDS_DUAL_LINK_ODD_EVEN_PIXELS;
		lvds->phy_master = &lvds_phy_16ff_master;
		lvds->phy_slave = &lvds_phy_16ff_slave;
		break;
	case DRM_LVDS_DUAL_LINK_EVEN_ODD_PIXELS:
		lvds->config.link_type = LVDS_DUAL_LINK_EVEN_ODD_PIXELS;
		lvds->phy_master = &lvds_phy_16ff_master;
		lvds->phy_slave = &lvds_phy_16ff_slave;
		break;
	case -EINVAL:
		/*
		 * drm_of_lvds_get_dual_pixel_order returns 4 possible values.
		 * In the case where the returned value is an error, it can be
		 * either ENODEV or EINVAL. Seeing the structure of this
		 * function, EINVAL means that either port1 or port2 is not
		 * present in the device tree.
		 * In that case, the lvds panel can be a single link panel, or
		 * there is a semantical error in the device tree code.
		 */
		remote = of_get_next_available_child(port1, NULL);
		if (remote) {
			if (of_graph_get_remote_endpoint(remote)) {
				lvds->config.link_type = LVDS_SINGLE_LINK_MASTER;
				lvds->phy_master = &lvds_phy_16ff_master;
				lvds->phy_slave = NULL;
			} else {
				ret = -EINVAL;
			}

			of_node_put(remote);
		}

		remote = of_get_next_available_child(port2, NULL);
		if (remote) {
			if (of_graph_get_remote_endpoint(remote)) {
				lvds->config.link_type = LVDS_SINGLE_LINK_SLAVE;
				lvds->phy_master = NULL;
				lvds->phy_slave = &lvds_phy_16ff_slave;
			} else {
				ret = (ret == -EINVAL) ? -EINVAL : 0;
			}

			of_node_put(remote);
		}
		break;
	default:
		return -EINVAL;
	}
	of_node_put(port1);
	of_node_put(port2);

	lvds->pclk = devm_clk_get(lvds->dev, "pclk");
	if (IS_ERR(lvds->pclk)) {
		ret = PTR_ERR(lvds->pclk);
		DRM_ERROR("Unable to get peripheral clock: %d\n", ret);
		return ret;
	}

	lvds->pllref_clk = devm_clk_get(lvds->dev, "ref");
	if (IS_ERR(lvds->pllref_clk)) {
		ret = PTR_ERR(lvds->pllref_clk);
		DRM_ERROR("Unable to get reference clock: %d\n", ret);
		return ret;
	}

	ret = lvds_pixel_clk_register(lvds);
	if (ret) {
		DRM_ERROR("Failed to register LVDS pixel clock: %d\n", ret);
		return ret;
	}

	lvds->lvds_bridge.funcs = &lvds_bridge_funcs;
	lvds->lvds_bridge.of_node = dev->of_node;

	drm_bridge_add(&lvds->lvds_bridge);
	platform_set_drvdata(pdev, lvds);

	pm_runtime_enable(dev);

	/*
	 * To obtain a continuous display after the probe,
	 *  the clocks must remain activated
	 */
	if (device_property_read_bool(dev, "default-on")) {
		pm_runtime_get_sync(dev);

		if (lvds->phy_master) {
			if (lvds_is_dual_link(lvds->config.link_type))
				multiplier = 2;
			else
				multiplier = 1;

			phy = lvds->phy_master;
			pll_in_khz = clk_get_rate(lvds->pllref_clk) / 1000;

			ndiv = lvds_read(lvds, phy->base + phy->ofs.PxPLLCR2) >> 16;
			bdiv = lvds_read(lvds, phy->base + phy->ofs.PxPLLCR2) & 0xFFFF;
			mdiv = lvds_read(lvds, phy->base + phy->ofs.PxPLLSDCR1);

			/* X7 because for each pixel in 1 lane there is 7 bits
			 * We want pixclk, not bitclk
			 */
			rate = pll_get_clkout_khz(pll_in_khz, bdiv, mdiv, ndiv);
			lvds->pixel_clock_rate = (unsigned long) rate  * 1000 * multiplier / 7;
		}
	}

	return 0;
}

static int lvds_remove(struct platform_device *pdev)
{
	struct stm_lvds *lvds = platform_get_drvdata(pdev);

	lvds_pixel_clk_unregister(lvds);
	pm_runtime_disable(&pdev->dev);

	/* Unregister LVDS bridge */
	drm_bridge_remove(&lvds->lvds_bridge);

	return 0;
}

static int __maybe_unused lvds_runtime_suspend(struct device *dev)
{
	struct stm_lvds *lvds = dev_get_drvdata(dev);

	DRM_DEBUG_DRIVER("\n");

	clk_disable_unprepare(lvds->pllref_clk);
	clk_disable_unprepare(lvds->pclk);
	regulator_disable(lvds->vdd_supply);
	regulator_disable(lvds->vdda18_supply);

	return 0;
}

static int __maybe_unused lvds_runtime_resume(struct device *dev)
{
	struct stm_lvds *lvds = dev_get_drvdata(dev);
	int ret;

	DRM_DEBUG_DRIVER("\n");

	ret = regulator_enable(lvds->vdda18_supply);
	if (ret) {
		DRM_ERROR("Failed to enable regulator vdda18: %d\n", ret);
		return ret;
	}

	ret = regulator_enable(lvds->vdd_supply);
	if (ret) {
		regulator_disable(lvds->vdda18_supply);
		DRM_ERROR("Failed to enable regulator vdd: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(lvds->pclk);
	if (ret) {
		regulator_disable(lvds->vdd_supply);
		regulator_disable(lvds->vdda18_supply);
		DRM_ERROR("Failed to enable pclk: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(lvds->pllref_clk);
	if (ret) {
		clk_disable_unprepare(lvds->pclk);
		regulator_disable(lvds->vdd_supply);
		regulator_disable(lvds->vdda18_supply);
		DRM_ERROR("Failed to enable pllref_clk: %d\n", ret);
	}

	return ret;
}

static const struct of_device_id lvds_dt_ids[] = {
	{
		.compatible = "st,stm32mp25-lvds",
		.data = NULL
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, lvds_dt_ids);

static const struct dev_pm_ops lvds_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(lvds_runtime_suspend, lvds_runtime_resume, NULL)
};

static struct platform_driver lvds_platform_driver = {
	.probe	  = lvds_probe,
	.remove	 = lvds_remove,
	.driver	 = {
		.name   = "stm32-display-lvds",
		.owner  = THIS_MODULE,
		.of_match_table = lvds_dt_ids,
		.pm = &lvds_pm_ops,
	},
};

module_platform_driver(lvds_platform_driver);

MODULE_AUTHOR("Raphaël Gallais-Pou <raphael.gallais-pou@foss.st.com>");
MODULE_AUTHOR("Philippe Cornu <philippe.cornu@foss.st.com>");
MODULE_AUTHOR("Yannick Fertre <yannick.fertre@foss.st.com>");
MODULE_DESCRIPTION("STMicroelectronics LVDS Display Interface Transmitter DRM driver");
MODULE_LICENSE("GPL v2");
