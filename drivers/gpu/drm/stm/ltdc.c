// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics SA 2017
 *
 * Authors: Philippe Cornu <philippe.cornu@st.com>
 *          Yannick Fertre <yannick.fertre@st.com>
 *          Fabien Dessenne <fabien.dessenne@st.com>
 *          Mickael Reulier <mickael.reulier@st.com>
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_reserved_mem.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_bridge.h>
#include <drm/drm_device.h>
#include <drm/drm_edid.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

/* Temporary (wait to firewall interface) */
#include <dt-bindings/bus/stm32mp25_sys_bus.h>
#include "../../../bus/stm32_sys_bus.h"

#include <video/videomode.h>

#include "ltdc.h"

#define NB_CRTC 1
#define CRTC_MASK GENMASK(NB_CRTC - 1, 0)

#define MAX_IRQ 4

#define HWVER_10200 0x010200
#define HWVER_10300 0x010300
#define HWVER_20101 0x020101
#define HWVER_40100 0x040100
#define HWVER_40101 0x040101

/*
 * The address of some registers depends on the HW version: such registers have
 * an extra offset specified with layer_ofs.
 */
#define LAY_OFS_0	0x80
#define LAY_OFS_1	0x100
#define LAY_OFS	(ldev->caps.layer_ofs)

/* Global register offsets */
#define LTDC_IDR	0x0000		/* IDentification */
#define LTDC_LCR	0x0004		/* Layer Count */
#define LTDC_SSCR	0x0008		/* Synchronization Size Configuration */
#define LTDC_BPCR	0x000C		/* Back Porch Configuration */
#define LTDC_AWCR	0x0010		/* Active Width Configuration */
#define LTDC_TWCR	0x0014		/* Total Width Configuration */
#define LTDC_GCR	0x0018		/* Global Control */
#define LTDC_GC1R	0x001C		/* Global Configuration 1 */
#define LTDC_GC2R	0x0020		/* Global Configuration 2 */
#define LTDC_SRCR	0x0024		/* Shadow Reload Configuration */
#define LTDC_GACR	0x0028		/* GAmma Correction */
#define LTDC_BCCR	0x002C		/* Background Color Configuration */
#define LTDC_IER	0x0034		/* Interrupt Enable */
#define LTDC_ISR	0x0038		/* Interrupt Status */
#define LTDC_ICR	0x003C		/* Interrupt Clear */
#define LTDC_LIPCR	0x0040		/* Line Interrupt Position Conf. */
#define LTDC_CPSR	0x0044		/* Current Position Status */
#define LTDC_CDSR	0x0048		/* Current Display Status */
#define LTDC_EDCR	0x0060		/* External Display Control */
#define LTDC_CCRCR	0x007C		/* Computed CRC value */
#define LTDC_RB0AR	0x0080		/* Rotation Buffer 0 address */
#define LTDC_RB1AR	0x0084		/* Rotation Buffer 1 address */
#define LTDC_RBPR	0x0088		/* Rotation Buffer Pitch */
#define LTDC_RIFCR	0x008C		/* Rotation Intermediate Frame Color */
#define LTDC_FUT	0x0090		/* Fifo underrun Threshold */

/* Layer register offsets */
#define LTDC_L1C0R	(ldev->caps.layer_regs[0])	/* L1 configuration 0 */
#define LTDC_L1C1R	(ldev->caps.layer_regs[1])	/* L1 configuration 1 */
#define LTDC_L1RCR	(ldev->caps.layer_regs[2])	/* L1 reload control */
#define LTDC_L1CR	(ldev->caps.layer_regs[3])	/* L1 control register */
#define LTDC_L1WHPCR	(ldev->caps.layer_regs[4])	/* L1 window horizontal position configuration */
#define LTDC_L1WVPCR	(ldev->caps.layer_regs[5])	/* L1 window vertical position configuration */
#define LTDC_L1CKCR	(ldev->caps.layer_regs[6])	/* L1 color keying configuration */
#define LTDC_L1PFCR	(ldev->caps.layer_regs[7])	/* L1 pixel format configuration */
#define LTDC_L1CACR	(ldev->caps.layer_regs[8])	/* L1 constant alpha configuration */
#define LTDC_L1DCCR	(ldev->caps.layer_regs[9])	/* L1 default color configuration */
#define LTDC_L1BFCR	(ldev->caps.layer_regs[10])	/* L1 blending factors configuration */
#define LTDC_L1BLCR	(ldev->caps.layer_regs[11])	/* L1 burst length configuration */
#define LTDC_L1PCR	(ldev->caps.layer_regs[12])	/* L1 planar configuration */
#define LTDC_L1CFBAR	(ldev->caps.layer_regs[13])	/* L1 color frame buffer address */
#define LTDC_L1CFBLR	(ldev->caps.layer_regs[14])	/* L1 color frame buffer length */
#define LTDC_L1CFBLNR	(ldev->caps.layer_regs[15])	/* L1 color frame buffer line number */
#define LTDC_L1AFBA0R	(ldev->caps.layer_regs[16])	/* L1 auxiliary frame buffer address 0 */
#define LTDC_L1AFBA1R	(ldev->caps.layer_regs[17])	/* L1 auxiliary frame buffer address 1 */
#define LTDC_L1AFBLR	(ldev->caps.layer_regs[18])	/* L1 auxiliary frame buffer length */
#define LTDC_L1AFBLNR	(ldev->caps.layer_regs[19])	/* L1 auxiliary frame buffer line number */
#define LTDC_L1CLUTWR	(ldev->caps.layer_regs[20])	/* L1 CLUT write */
#define LTDC_L1SISR	(ldev->caps.layer_regs[21])	/* L1 scaler input size */
#define LTDC_L1SOSR	(ldev->caps.layer_regs[22])	/* L1 scaler output size */
#define LTDC_L1SVSFR	(ldev->caps.layer_regs[23])	/* L1 scaler vertical scaling factor */
#define LTDC_L1SVSPR	(ldev->caps.layer_regs[24])	/* L1 scaler vertical scaling phase */
#define LTDC_L1SHSFR	(ldev->caps.layer_regs[25])	/* L1 scaler horizontal scaling factor */
#define LTDC_L1SHSPR	(ldev->caps.layer_regs[26])	/* L1 scaler horizontal scaling phase */
#define LTDC_L1CYR0R	(ldev->caps.layer_regs[27])	/* L1 Conversion YCbCr RGB 0 */
#define LTDC_L1CYR1R	(ldev->caps.layer_regs[28])	/* L1 Conversion YCbCr RGB 1 */
#define LTDC_L1FPF0R	(ldev->caps.layer_regs[29])	/* L1 Flexible Pixel Format 0 */
#define LTDC_L1FPF1R	(ldev->caps.layer_regs[30])	/* L1 Flexible Pixel Format 1 */

/* Bit definitions */
#define SSCR_VSH	GENMASK(10, 0)	/* Vertical Synchronization Height */
#define SSCR_HSW	GENMASK(27, 16)	/* Horizontal Synchronization Width */

#define BPCR_AVBP	GENMASK(10, 0)	/* Accumulated Vertical Back Porch */
#define BPCR_AHBP	GENMASK(27, 16)	/* Accumulated Horizontal Back Porch */

#define AWCR_AAH	GENMASK(10, 0)	/* Accumulated Active Height */
#define AWCR_AAW	GENMASK(27, 16)	/* Accumulated Active Width */

#define TWCR_TOTALH	GENMASK(10, 0)	/* TOTAL Height */
#define TWCR_TOTALW	GENMASK(27, 16)	/* TOTAL Width */

#define GCR_LTDCEN	BIT(0)		/* LTDC ENable */
#define GCR_ROTEN	BIT(2)		/* ROTation ENable */
#define GCR_DEN		BIT(16)		/* Dither ENable */
#define GCR_CRCEN	BIT(19)		/* CRC ENable */
#define GCR_PCPOL	BIT(28)		/* Pixel Clock POLarity-Inverted */
#define GCR_DEPOL	BIT(29)		/* Data Enable POLarity-High */
#define GCR_VSPOL	BIT(30)		/* Vertical Synchro POLarity-High */
#define GCR_HSPOL	BIT(31)		/* Horizontal Synchro POLarity-High */

#define GC1R_WBCH	GENMASK(3, 0)	/* Width of Blue CHannel output */
#define GC1R_WGCH	GENMASK(7, 4)	/* Width of Green Channel output */
#define GC1R_WRCH	GENMASK(11, 8)	/* Width of Red Channel output */
#define GC1R_PBEN	BIT(12)		/* Precise Blending ENable */
#define GC1R_DT		GENMASK(15, 14)	/* Dithering Technique */
#define GC1R_GCT	GENMASK(19, 17)	/* Gamma Correction Technique */
#define GC1R_SHREN	BIT(21)		/* SHadow Registers ENabled */
#define GC1R_BCP	BIT(22)		/* Background Colour Programmable */
#define GC1R_BBEN	BIT(23)		/* Background Blending ENabled */
#define GC1R_LNIP	BIT(24)		/* Line Number IRQ Position */
#define GC1R_TP		BIT(25)		/* Timing Programmable */
#define GC1R_IPP	BIT(26)		/* IRQ Polarity Programmable */
#define GC1R_SPP	BIT(27)		/* Sync Polarity Programmable */
#define GC1R_DWP	BIT(28)		/* Dither Width Programmable */
#define GC1R_STREN	BIT(29)		/* STatus Registers ENabled */
#define GC1R_BMEN	BIT(31)		/* Blind Mode ENabled */

#define GC2R_EDCA	BIT(0)		/* External Display Control Ability  */
#define GC2R_STSAEN	BIT(1)		/* Slave Timing Sync Ability ENabled */
#define GC2R_DVAEN	BIT(2)		/* Dual-View Ability ENabled */
#define GC2R_DPAEN	BIT(3)		/* Dual-Port Ability ENabled */
#define GC2R_BW		GENMASK(6, 4)	/* Bus Width (log2 of nb of bytes) */
#define GC2R_EDCEN	BIT(7)		/* External Display Control ENabled */
#define GC2R_ROTA	BIT(10)		/* ROTAtion support ability */

#define SRCR_IMR	BIT(0)		/* IMmediate Reload */
#define SRCR_VBR	BIT(1)		/* Vertical Blanking Reload */

#define BCCR_BCBLACK	0x00		/* Background Color BLACK */
#define BCCR_BCBLUE	GENMASK(7, 0)	/* Background Color BLUE */
#define BCCR_BCGREEN	GENMASK(15, 8)	/* Background Color GREEN */
#define BCCR_BCRED	GENMASK(23, 16)	/* Background Color RED */
#define BCCR_BCWHITE	GENMASK(23, 0)	/* Background Color WHITE */

#define DCCR_DCBLACK	0x00		/* Default Color BLACK */
#define DCCR_DCBLUE	GENMASK(7, 0)	/* Default Color BLUE */
#define DCCR_DCGREEN	GENMASK(15, 8)	/* Default Color GREEN */
#define DCCR_DCRED	GENMASK(23, 16)	/* Default Color RED */
#define DCCR_DCWHITE	GENMASK(23, 0)	/* Default Color WHITE */

#define IER_LIE		BIT(0)		/* Line Interrupt Enable */
#define IER_FUWIE	BIT(1)		/* Fifo Underrun Warning Interrupt Enable */
#define IER_TERRIE	BIT(2)		/* Transfer ERRor Interrupt Enable */
#define IER_RRIE	BIT(3)		/* Register Reload Interrupt Enable */
#define IER_FUEIE	BIT(6)		/* Fifo Underrun Error Interrupt Enable */
#define IER_CRCIE	BIT(7)		/* CRC Error Interrupt Enable */
#define IER_FURIE	BIT(8)		/* Fifo Underrun Rotation Interrupt Enable */
#define IER_MASK (IER_LIE | IER_FUWIE | IER_TERRIE | IER_RRIE | IER_FUEIE | IER_CRCIE | IER_FURIE)

#define CPSR_CYPOS	GENMASK(15, 0)	/* Current Y position */

#define ISR_LIF		BIT(0)		/* Line Interrupt Flag */
#define ISR_FUWIF	BIT(1)		/* Fifo Underrun Warning Interrupt Flag */
#define ISR_TERRIF	BIT(2)		/* Transfer ERRor Interrupt Flag */
#define ISR_RRIF	BIT(3)		/* Register Reload Interrupt Flag */
#define ISR_FUEIF	BIT(6)		/* Fifo Underrun Error Interrupt Flag */
#define ISR_CRCIF	BIT(7)		/* CRC Error Interrupt Flag */
#define ISR_FURIF	BIT(8)		/* Fifo Underrun Rotation Interrupt Flag */

#define EDCR_OCYEN	BIT(25)		/* Output Conversion to YCbCr 422: ENable */
#define EDCR_OCYSEL	BIT(26)		/* Output Conversion to YCbCr 422: SELection of the CCIR */
#define EDCR_OCYCO	BIT(27)		/* Output Conversion to YCbCr 422: Chrominance Order */

#define LXCR_LEN	BIT(0)		/* Layer ENable */
#define LXCR_COLKEN	BIT(1)		/* Color Keying Enable */
#define LXCR_CLUTEN	BIT(4)		/* Color Look-Up Table ENable */
#define LXCR_HMEN	BIT(8)		/* Horizontal Mirroring ENable */
#define LXCR_SCEN	BIT(10)		/* SCaler ENable */
#define LXCR_MASK (LXCR_LEN | LXCR_COLKEN | LXCR_CLUTEN | LXCR_HMEN | LXCR_SCEN)

#define LXWHPCR_WHSTPOS	GENMASK(11, 0)	/* Window Horizontal StarT POSition */
#define LXWHPCR_WHSPPOS	GENMASK(27, 16)	/* Window Horizontal StoP POSition */

#define LXWVPCR_WVSTPOS	GENMASK(10, 0)	/* Window Vertical StarT POSition */
#define LXWVPCR_WVSPPOS	GENMASK(26, 16)	/* Window Vertical StoP POSition */

#define LXPFCR_PF	GENMASK(2, 0)	/* Pixel Format */
#define PF_FLEXIBLE	0x7		/* Flexible Pixel Format selected */

#define LXCACR_CONSTA	GENMASK(7, 0)	/* CONSTant Alpha */

#define LXBFCR_BF2	GENMASK(2, 0)	/* Blending Factor 2 */
#define LXBFCR_BF1	GENMASK(10, 8)	/* Blending Factor 1 */
#define LXBFCR_BOR	GENMASK(18, 16) /* Blending ORder */

#define LXCFBLR_CFBLL	GENMASK(12, 0)	/* Color Frame Buffer Line Length */
#define LXCFBLR_CFBP	GENMASK(31, 16) /* Color Frame Buffer Pitch in bytes */

#define LXCFBLNR_CFBLN	GENMASK(10, 0)	/* Color Frame Buffer Line Number */

#define LXCR_C1R_YIA	BIT(0)		/* Ycbcr 422 Interleaved Ability */
#define LXCR_C1R_YSPA	BIT(1)		/* Ycbcr 420 Semi-Planar Ability */
#define LXCR_C1R_YFPA	BIT(2)		/* Ycbcr 420 Full-Planar Ability */
#define LXCR_C1R_SCA	BIT(31)		/* SCaling Ability*/

#define LxPCR_YREN	BIT(9)		/* Y Rescale Enable for the color dynamic range */
#define LxPCR_OF	BIT(8)		/* Odd pixel First */
#define LxPCR_CBF	BIT(7)		/* CB component First */
#define LxPCR_YF	BIT(6)		/* Y component First */
#define LxPCR_YCM	GENMASK(5, 4)	/* Ycbcr Conversion Mode */
#define YCM_I		0x0		/* Interleaved 422 */
#define YCM_SP		0x1		/* Semi-Planar 420 */
#define YCM_FP		0x2		/* Full-Planar 420 */
#define LxPCR_YCEN	BIT(3)		/* YCbCr-to-RGB Conversion Enable */

#define LXRCR_IMR	BIT(0)		/* IMmediate Reload */
#define LXRCR_VBR	BIT(1)		/* Vertical Blanking Reload */
#define LXRCR_GRMSK	BIT(2)		/* Global (centralized) Reload MaSKed */

#define CLUT_SIZE	256

#define CONSTA_MAX	0xFF		/* CONSTant Alpha MAX= 1.0 */
#define BF1_PAXCA	0x600		/* Pixel Alpha x Constant Alpha */
#define BF1_CA		0x400		/* Constant Alpha */
#define BF2_1PAXCA	0x007		/* 1 - (Pixel Alpha x Constant Alpha) */
#define BF2_1CA		0x005		/* 1 - Constant Alpha */

#define NB_PF		8		/* Max nb of HW pixel format */

#define FUT_DFT		128		/* Default value of fifo underrun threshold */

/*
 * Skip the first value and the second in case CRC was enabled during
 * the thread irq. This is to be sure CRC value is relevant for the
 * frame.
 */
#define CRC_SKIP_FRAMES 2

enum ltdc_pix_fmt {
	PF_NONE,
	/* RGB formats */
	PF_ARGB8888,		/* ARGB [32 bits] */
	PF_RGBA8888,		/* RGBA [32 bits] */
	PF_ABGR8888,		/* ABGR [32 bits] */
	PF_BGRA8888,		/* BGRA [32 bits] */
	PF_RGB888,		/* RGB [24 bits] */
	PF_BGR888,		/* BGR [24 bits] */
	PF_RGB565,		/* RGB [16 bits] */
	PF_BGR565,		/* BGR [16 bits] */
	PF_ARGB1555,		/* ARGB A:1 bit RGB:15 bits [16 bits] */
	PF_ARGB4444,		/* ARGB A:4 bits R/G/B: 4 bits each [16 bits] */
	/* Indexed formats */
	PF_L8,			/* Indexed 8 bits [8 bits] */
	PF_AL44,		/* Alpha:4 bits + indexed 4 bits [8 bits] */
	PF_AL88			/* Alpha:8 bits + indexed 8 bits [16 bits] */
};

/* The index gives the encoding of the pixel format for an HW version */
static const enum ltdc_pix_fmt ltdc_pix_fmt_a0[NB_PF] = {
	PF_ARGB8888,		/* 0x00 */
	PF_RGB888,		/* 0x01 */
	PF_RGB565,		/* 0x02 */
	PF_ARGB1555,		/* 0x03 */
	PF_ARGB4444,		/* 0x04 */
	PF_L8,			/* 0x05 */
	PF_AL44,		/* 0x06 */
	PF_AL88			/* 0x07 */
};

static const enum ltdc_pix_fmt ltdc_pix_fmt_a1[NB_PF] = {
	PF_ARGB8888,		/* 0x00 */
	PF_RGB888,		/* 0x01 */
	PF_RGB565,		/* 0x02 */
	PF_RGBA8888,		/* 0x03 */
	PF_AL44,		/* 0x04 */
	PF_L8,			/* 0x05 */
	PF_ARGB1555,		/* 0x06 */
	PF_ARGB4444		/* 0x07 */
};

static const enum ltdc_pix_fmt ltdc_pix_fmt_a2[NB_PF] = {
	PF_ARGB8888,		/* 0x00 */
	PF_ABGR8888,		/* 0x01 */
	PF_RGBA8888,		/* 0x02 */
	PF_BGRA8888,		/* 0x03 */
	PF_RGB565,		/* 0x04 */
	PF_BGR565,		/* 0x05 */
	PF_RGB888,		/* 0x06 */
	PF_NONE			/* 0x07 */
};

static const u32 ltdc_drm_fmt_a0[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_ARGB1555,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_XRGB4444,
	DRM_FORMAT_C8
};

static const u32 ltdc_drm_fmt_a1[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_ARGB1555,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_XRGB4444,
	DRM_FORMAT_C8
};

static const u32 ltdc_drm_fmt_a2[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888,
	DRM_FORMAT_ARGB1555,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_XRGB4444,
	DRM_FORMAT_C8
};

static const u32 ltdc_drm_fmt_ycbcr_cp[] = {
	DRM_FORMAT_YUYV,
	DRM_FORMAT_YVYU,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_VYUY
};

static const u32 ltdc_drm_fmt_ycbcr_sp[] = {
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21
};

static const u32 ltdc_drm_fmt_ycbcr_fp[] = {
	DRM_FORMAT_YUV420,
	DRM_FORMAT_YVU420
};

/* Layer register offsets */
static const u32 ltdc_layer_regs_a0[] = {
	0x80,	/* L1 configuration 0 */
	0x00,	/* not available */
	0x00,	/* not available */
	0x84,	/* L1 control register */
	0x88,	/* L1 window horizontal position configuration */
	0x8c,	/* L1 window vertical position configuration */
	0x90,	/* L1 color keying configuration */
	0x94,	/* L1 pixel format configuration */
	0x98,	/* L1 constant alpha configuration */
	0x9c,	/* L1 default color configuration */
	0xa0,	/* L1 blending factors configuration */
	0x00,	/* not available */
	0x00,	/* not available */
	0xac,	/* L1 color frame buffer address */
	0xb0,	/* L1 color frame buffer length */
	0xb4,	/* L1 color frame buffer line number */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0xc4,	/* L1 CLUT write */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00	/* not available */
};

static const u32 ltdc_layer_regs_a1[] = {
	0x80,	/* L1 configuration 0 */
	0x84,	/* L1 configuration 1 */
	0x00,	/* L1 reload control */
	0x88,	/* L1 control register */
	0x8c,	/* L1 window horizontal position configuration */
	0x90,	/* L1 window vertical position configuration */
	0x94,	/* L1 color keying configuration */
	0x98,	/* L1 pixel format configuration */
	0x9c,	/* L1 constant alpha configuration */
	0xa0,	/* L1 default color configuration */
	0xa4,	/* L1 blending factors configuration */
	0xa8,	/* L1 burst length configuration */
	0x00,	/* not available */
	0xac,	/* L1 color frame buffer address */
	0xb0,	/* L1 color frame buffer length */
	0xb4,	/* L1 color frame buffer line number */
	0xb8,	/* L1 auxiliary frame buffer address 0 */
	0xbc,	/* L1 auxiliary frame buffer address 1 */
	0xc0,	/* L1 auxiliary frame buffer length */
	0xc4,	/* L1 auxiliary frame buffer line number */
	0xc8,	/* L1 CLUT write */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00,	/* not available */
	0x00	/* not available */
};

static const u32 ltdc_layer_regs_a2[] = {
	0x100,	/* L1 configuration 0 */
	0x104,	/* L1 configuration 1 */
	0x108,	/* L1 reload control */
	0x10c,	/* L1 control register */
	0x110,	/* L1 window horizontal position configuration */
	0x114,	/* L1 window vertical position configuration */
	0x118,	/* L1 color keying configuration */
	0x11c,	/* L1 pixel format configuration */
	0x120,	/* L1 constant alpha configuration */
	0x124,	/* L1 default color configuration */
	0x128,	/* L1 blending factors configuration */
	0x12c,	/* L1 burst length configuration */
	0x130,	/* L1 planar configuration */
	0x134,	/* L1 color frame buffer address */
	0x138,	/* L1 color frame buffer length */
	0x13c,	/* L1 color frame buffer line number */
	0x140,	/* L1 auxiliary frame buffer address 0 */
	0x144,	/* L1 auxiliary frame buffer address 1 */
	0x148,	/* L1 auxiliary frame buffer length */
	0x14c,	/* L1 auxiliary frame buffer line number */
	0x150,	/* L1 CLUT write */
	0x154,	/* L1 scaler input size */
	0x158,	/* L1 scaler output size */
	0x15c,	/* L1 scaler vertical scaling factor */
	0x160,	/* L1 scaler vertical scaling phase */
	0x164,	/* L1 scaler horizontal scaling factor */
	0x168,	/* L1 scaler horizontal scaling phase */
	0x16c,	/* L1 Conversion YCbCr RGB 0 */
	0x170,	/* L1 Conversion YCbCr RGB 1 */
	0x174,	/* L1 Flexible Pixel Format 0 */
	0x178	/* L1 Flexible Pixel Format 1 */
};

static const u64 ltdc_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static struct regmap_config stm32_ltdc_regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = sizeof(u32),
	.max_register = 0x400,
	.use_relaxed_mmio = true,
	.cache_type = REGCACHE_NONE,
};

static const u32 ltdc_ycbcr2rgb_coeffs[DRM_COLOR_ENCODING_MAX][DRM_COLOR_RANGE_MAX][2] = {
	[DRM_COLOR_YCBCR_BT601][DRM_COLOR_YCBCR_LIMITED_RANGE] = {
		0x02040199,	/* (b_cb = 516 / r_cr = 409) */
		0x006400D0	/* (g_cb = 100 / g_cr = 208) */
	},
	[DRM_COLOR_YCBCR_BT601][DRM_COLOR_YCBCR_FULL_RANGE] = {
		0x01C60167,	/* (b_cb = 454 / r_cr = 359) */
		0x005800B7	/* (g_cb = 88 / g_cr = 183) */
	},
	[DRM_COLOR_YCBCR_BT709][DRM_COLOR_YCBCR_LIMITED_RANGE] = {
		0x021D01CB,	/* (b_cb = 541 / r_cr = 459) */
		0x00370089	/* (g_cb = 55 / g_cr = 137) */
	},
	[DRM_COLOR_YCBCR_BT709][DRM_COLOR_YCBCR_FULL_RANGE] = {
		0x01DB0193,	/* (b_cb = 475 / r_cr = 403) */
		0x00300078	/* (g_cb = 48 / g_cr = 120) */
	}
	/* BT2020 not supported */
};

static inline struct ltdc_device *crtc_to_ltdc(struct drm_crtc *crtc)
{
	return (struct ltdc_device *)crtc->dev->dev_private;
}

static inline struct ltdc_device *plane_to_ltdc(struct drm_plane *plane)
{
	return (struct ltdc_device *)plane->dev->dev_private;
}

static inline struct ltdc_device *encoder_to_ltdc(struct drm_encoder *enc)
{
	return (struct ltdc_device *)enc->dev->dev_private;
}

static inline enum ltdc_pix_fmt to_ltdc_pixelformat(u32 drm_fmt)
{
	enum ltdc_pix_fmt pf;

	switch (drm_fmt) {
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
		pf = PF_ARGB8888;
		break;
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
		pf = PF_ABGR8888;
		break;
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
		pf = PF_RGBA8888;
		break;
	case DRM_FORMAT_BGRA8888:
	case DRM_FORMAT_BGRX8888:
		pf = PF_BGRA8888;
		break;
	case DRM_FORMAT_RGB888:
		pf = PF_RGB888;
		break;
	case DRM_FORMAT_BGR888:
		pf = PF_BGR888;
		break;
	case DRM_FORMAT_RGB565:
		pf = PF_RGB565;
		break;
	case DRM_FORMAT_BGR565:
		pf = PF_BGR565;
		break;
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
		pf = PF_ARGB1555;
		break;
	case DRM_FORMAT_ARGB4444:
	case DRM_FORMAT_XRGB4444:
		pf = PF_ARGB4444;
		break;
	case DRM_FORMAT_C8:
		pf = PF_L8;
		break;
	default:
		pf = PF_NONE;
		break;
		/* Note: There are no DRM_FORMAT for AL44 and AL88 */
	}

	return pf;
}

static inline u32 ltdc_set_flexible_pixel_format(struct drm_plane *plane, enum ltdc_pix_fmt pix_fmt)
{
	struct ltdc_device *ldev = plane_to_ltdc(plane);
	u32 lofs = plane->index * LAY_OFS, ret = PF_FLEXIBLE;
	int psize, alen, apos, rlen, rpos, glen, gpos, blen, bpos;

	switch (pix_fmt) {
	case PF_BGR888:
		psize = 3;
		alen = 0; apos = 0; rlen = 8; rpos = 0;
		glen = 8; gpos = 8; blen = 8; bpos = 16;
	break;
	case PF_ARGB1555:
		psize = 2;
		alen = 1; apos = 15; rlen = 5; rpos = 10;
		glen = 5; gpos = 5;  blen = 5; bpos = 0;
	break;
	case PF_ARGB4444:
		psize = 2;
		alen = 4; apos = 12; rlen = 4; rpos = 8;
		glen = 4; gpos = 4; blen = 4; bpos = 0;
	break;
	case PF_L8:
		psize = 1;
		alen = 0; apos = 0; rlen = 8; rpos = 0;
		glen = 8; gpos = 0; blen = 8; bpos = 0;
	break;
	case PF_AL44:
		psize = 1;
		alen = 4; apos = 4; rlen = 4; rpos = 0;
		glen = 4; gpos = 0; blen = 4; bpos = 0;
	break;
	case PF_AL88:
		psize = 2;
		alen = 8; apos = 8; rlen = 8; rpos = 0;
		glen = 8; gpos = 0; blen = 8; bpos = 0;
	break;
	default:
		ret = NB_PF; /* error case, trace msg is handled by the caller */
	break;
	}

	if (ret == PF_FLEXIBLE) {
		regmap_write(ldev->regmap, LTDC_L1FPF0R + lofs,
			     (rlen << 14)  + (rpos << 9) + (alen << 5) + apos);

		regmap_write(ldev->regmap, LTDC_L1FPF1R + lofs,
			     (psize << 18) + (blen << 14)  + (bpos << 9) + (glen << 5) + gpos);
	}

	return ret;
}

/*
 * All non-alpha color formats derived from native alpha color formats are
 * either characterized by a FourCC format code
 */
static inline u32 is_xrgb(u32 drm)
{
	return ((drm & 0xFF) == 'X' || ((drm >> 8) & 0xFF) == 'X');
}

static inline void ltdc_set_ycbcr_config(struct drm_plane *plane, u32 drm_pix_fmt)
{
	struct ltdc_device *ldev = plane_to_ltdc(plane);
	struct drm_plane_state *state = plane->state;
	u32 lofs = plane->index * LAY_OFS;
	u32 val;

	switch (drm_pix_fmt) {
	case DRM_FORMAT_YUYV:
		val = (YCM_I << 4) | LxPCR_YF | LxPCR_CBF;
		break;
	case DRM_FORMAT_YVYU:
		val = (YCM_I << 4) | LxPCR_YF;
		break;
	case DRM_FORMAT_UYVY:
		val = (YCM_I << 4) | LxPCR_CBF;
		break;
	case DRM_FORMAT_VYUY:
		val = (YCM_I << 4);
		break;
	case DRM_FORMAT_NV12:
		val = (YCM_SP << 4) | LxPCR_CBF;
		break;
	case DRM_FORMAT_NV21:
		val = (YCM_SP << 4);
		break;
	case DRM_FORMAT_YUV420:
	case DRM_FORMAT_YVU420:
		val = (YCM_FP << 4);
		break;
	default:
		/* RGB or not a YCbCr supported format */
		DRM_ERROR("Unsupported pixel format: %u\n", drm_pix_fmt);
		return;
	}

	/* Enable limited range */
	if (state->color_range == DRM_COLOR_YCBCR_LIMITED_RANGE)
		val |= LxPCR_YREN;

	/* enable ycbcr conversion */
	val |= LxPCR_YCEN;

	regmap_write(ldev->regmap, LTDC_L1PCR + lofs, val);
}

static inline void ltdc_set_ycbcr_coeffs(struct drm_plane *plane)
{
	struct ltdc_device *ldev = plane_to_ltdc(plane);
	struct drm_plane_state *state = plane->state;
	enum drm_color_encoding enc = state->color_encoding;
	enum drm_color_range ran = state->color_range;
	u32 lofs = plane->index * LAY_OFS;

	if (enc != DRM_COLOR_YCBCR_BT601 && enc != DRM_COLOR_YCBCR_BT709) {
		DRM_ERROR("color encoding %d not supported, use bt601 by default\n", enc);
		/* set by default color encoding to DRM_COLOR_YCBCR_BT601 */
		enc = DRM_COLOR_YCBCR_BT601;
	}

	if (ran != DRM_COLOR_YCBCR_LIMITED_RANGE && ran != DRM_COLOR_YCBCR_FULL_RANGE) {
		DRM_ERROR("color range %d not supported, use limited range by default\n", ran);
		/* set by default color range to DRM_COLOR_YCBCR_LIMITED_RANGE */
		ran = DRM_COLOR_YCBCR_LIMITED_RANGE;
	}

	DRM_DEBUG_DRIVER("Color encoding=%d, range=%d\n", enc, ran);
	regmap_write(ldev->regmap, LTDC_L1CYR0R + lofs,
		     ltdc_ycbcr2rgb_coeffs[enc][ran][0]);
	regmap_write(ldev->regmap, LTDC_L1CYR1R + lofs,
		     ltdc_ycbcr2rgb_coeffs[enc][ran][1]);
}

static irqreturn_t ltdc_irq_thread(int irq, void *arg)
{
	struct drm_device *ddev = arg;
	struct ltdc_device *ldev = ddev->dev_private;
	struct drm_crtc *crtc = drm_crtc_from_index(ddev, 0);

	/* Line IRQ : trigger the vblank event */
	if (ldev->irq_status & ISR_LIF) {
		if (ldev->vblank_active)
			drm_crtc_handle_vblank(crtc);

		if (ldev->crc_active) {
			if (ldev->crc_skip_count < CRC_SKIP_FRAMES)
				ldev->crc_skip_count++;
			else
				/* Report to DRM the CRC (hw dependent feature) */
				drm_crtc_add_crc_entry(crtc, true,
						       drm_crtc_accurate_vblank_count(crtc),
						       &ldev->crc);
		}
	}

	mutex_lock(&ldev->err_lock);
	if (ldev->irq_status & ISR_TERRIF)
		ldev->transfer_err++;
	if (ldev->irq_status & ISR_FUEIF)
		ldev->fifo_err++;
	if (ldev->irq_status & ISR_FUWIF)
		ldev->fifo_warn++;
	if (ldev->irq_status & ISR_FURIF)
		ldev->fifo_rot++;
	mutex_unlock(&ldev->err_lock);

	return IRQ_HANDLED;
}

static irqreturn_t ltdc_irq(int irq, void *arg)
{
	struct drm_device *ddev = arg;
	struct ltdc_device *ldev = ddev->dev_private;

	/*
	 *  Read & Clear the interrupt status
	 *  In order to write / read registers in this critical section
	 *  very quickly, the regmap functions are not used.
	 */
	ldev->irq_status = readl_relaxed(ldev->regs + LTDC_ISR);
	writel_relaxed(ldev->irq_status, ldev->regs + LTDC_ICR);
	if (ldev->crc_active)
		ldev->crc = readl_relaxed(ldev->regs + LTDC_CCRCR);

	return IRQ_WAKE_THREAD;
}

/*
 * DRM_CRTC
 */

static void ltdc_crtc_atomic_enable(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct ltdc_device *ldev = crtc_to_ltdc(crtc);
	struct drm_device *ddev = crtc->dev;
	struct drm_connector_list_iter iter;
	struct drm_connector *connector = NULL;
	struct drm_encoder *encoder = NULL, *en_iter;
	struct drm_bridge *bridge = NULL, *br_iter;
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	int orientation = DRM_MODE_PANEL_ORIENTATION_UNKNOWN;
	u32 bus_formats = MEDIA_BUS_FMT_RGB888_1X24;
	u32 bus_flags = 0;
	u32 pitch, rota0_buf, rota1_buf;
	u32 val;
	int ret;

	DRM_DEBUG_DRIVER("\n");

	/* get encoder from crtc */
	drm_for_each_encoder(en_iter, ddev)
		if (en_iter->crtc == crtc) {
			encoder = en_iter;
			break;
		}

	if (encoder) {
		/* get bridge from encoder */
		list_for_each_entry(br_iter, &encoder->bridge_chain, chain_node)
			if (br_iter->encoder == encoder) {
				bridge = br_iter;
				break;
			}

		/* Get the connector from encoder */
		drm_connector_list_iter_begin(ddev, &iter);
		drm_for_each_connector_iter(connector, &iter)
			if (connector->encoder == encoder)
				break;
		drm_connector_list_iter_end(&iter);
	}

	if (bridge && bridge->timings) {
		bus_flags = bridge->timings->input_bus_flags;
	} else if (connector) {
		bus_flags = connector->display_info.bus_flags;
		if (connector->display_info.num_bus_formats)
			bus_formats = connector->display_info.bus_formats[0];

		orientation = connector->display_info.panel_orientation;
	}

	if (!pm_runtime_active(ddev->dev)) {
		ret = pm_runtime_get_sync(ddev->dev);
		if (ret) {
			DRM_ERROR("Failed to set mode, cannot get sync\n");
			return;
		}
	}

	/* Configures the HS, VS, DE and PC polarities. Default Active Low */
	val = 0;

	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		val |= GCR_HSPOL;

	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		val |= GCR_VSPOL;

	if (bus_flags & DRM_BUS_FLAG_DE_LOW)
		val |= GCR_DEPOL;

	if (bus_flags & DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE)
		val |= GCR_PCPOL;

	if (connector && connector->state->dithering == DRM_MODE_DITHERING_ON)
		val |= GCR_DEN;

	regmap_update_bits(ldev->regmap, LTDC_GCR,
			   GCR_HSPOL | GCR_VSPOL | GCR_DEPOL | GCR_PCPOL | GCR_DEN, val);

	/* Configure the output format (hw version dependent) */
	if (ldev->caps.ycbcr_output) {
		/* Input video dynamic_range & colorimetry */
		int vic = drm_match_cea_mode(mode);
		u32 val;

		if (vic == 6 || vic == 7 || vic == 21 || vic == 22 ||
		    vic == 2 || vic == 3 || vic == 17 || vic == 18)
			/* ITU-R BT.601 */
			val = 0;
		else
			/* ITU-R BT.709 */
			val = EDCR_OCYSEL;

		switch (bus_formats) {
		case MEDIA_BUS_FMT_YUYV8_1X16:
			/* enable ycbcr output converter */
			regmap_write(ldev->regmap, LTDC_EDCR, EDCR_OCYEN | val);
			break;
		case MEDIA_BUS_FMT_YVYU8_1X16:
			/* enable ycbcr output converter & invert chrominance order */
			regmap_write(ldev->regmap, LTDC_EDCR, EDCR_OCYEN | EDCR_OCYCO | val);
			break;
		default:
			/* disable ycbcr output converter */
			regmap_write(ldev->regmap, LTDC_EDCR, 0);
			break;
		}
	}

	/* check that an output rotation is required */
	if (ldev->caps.crtc_rotation &&
	    (orientation == DRM_MODE_PANEL_ORIENTATION_LEFT_UP ||
	     orientation == DRM_MODE_PANEL_ORIENTATION_RIGHT_UP)) {
		/*
		 * Size of the rotation buffer must be larger than the size
		 * of two frames (format RGB24).
		 */
		if (ldev->rot_mem->size < mode->hdisplay * mode->vdisplay * 2 * 3) {
			DRM_WARN("Rotation buffer too small");
			return;
		}

		/* The width of the framebuffer must not exceed 1366 pixels */
		if (mode->vdisplay > 1366)
			return;

		rota0_buf = (u32)ldev->rot_mem->base;
		rota1_buf = (u32)ldev->rot_mem->base + (ldev->rot_mem->size >> 1);

		regmap_write(ldev->regmap, LTDC_RB0AR, rota0_buf);
		regmap_write(ldev->regmap, LTDC_RB1AR, rota1_buf);

		/*
		 * LTDC_RBPR register is used define the pitch (line-to-line address increment)
		 * of the stored rotation buffer. The pitch is proportional to the width of the
		 * composed display (before rotation) and,(after rotation) proportional to the
		 * non-raster dimension of the display panel.
		 */
		pitch = ((mode->hdisplay + 9) / 10) * 64;
		regmap_write(ldev->regmap, LTDC_RBPR, pitch);

		DRM_DEBUG_DRIVER("Rotation buffer0 address %x\n", rota0_buf);
		DRM_DEBUG_DRIVER("Rotation buffer1 address %x\n", rota1_buf);
		DRM_DEBUG_DRIVER("Rotation buffer picth %x\n", pitch);

		if (orientation == DRM_MODE_PANEL_ORIENTATION_LEFT_UP ||
		    orientation == DRM_MODE_PANEL_ORIENTATION_RIGHT_UP)
			regmap_set_bits(ldev->regmap, LTDC_GCR, GCR_ROTEN);
		else
			regmap_clear_bits(ldev->regmap, LTDC_GCR, GCR_ROTEN);
	}

	/* Sets the background color value */
	regmap_write(ldev->regmap, LTDC_BCCR, BCCR_BCBLACK);

	/* Enable error IRQ */
	regmap_set_bits(ldev->regmap, LTDC_IER, IER_FUWIE | IER_FUEIE | IER_TERRIE);

	if (ldev->caps.crtc_rotation)
		regmap_set_bits(ldev->regmap, LTDC_IER, IER_FURIE);

	/* Commit shadow registers = update planes at next vblank */
	if (!ldev->caps.plane_reg_shadow)
		regmap_set_bits(ldev->regmap, LTDC_SRCR, SRCR_VBR);

	drm_crtc_vblank_on(crtc);

	/* set fifo underrun threshold register */
	if (ldev->caps.fifo_threshold)
		regmap_write(ldev->regmap, LTDC_FUT, ldev->fifo_threshold);

	/* Enable LTDC */
	regmap_set_bits(ldev->regmap, LTDC_GCR, GCR_LTDCEN);
}

static void ltdc_crtc_atomic_disable(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct ltdc_device *ldev = crtc_to_ltdc(crtc);
	struct drm_device *ddev = crtc->dev;
	int layer_index = 0;
	int ret;

	DRM_DEBUG_DRIVER("\n");

	drm_crtc_vblank_off(crtc);

	/* Disable all layers */
	for (layer_index = 0; layer_index < ldev->caps.nb_layers; layer_index++)
		regmap_write_bits(ldev->regmap, LTDC_L1CR + layer_index * LAY_OFS, LXCR_MASK, 0);

	/* disable IRQ errors */
	regmap_clear_bits(ldev->regmap, LTDC_IER, IER_FUWIE | IER_FUEIE | IER_TERRIE | IER_FURIE);

	/* immediately commit disable of layers before switching off LTDC */
	if (!ldev->caps.plane_reg_shadow)
		regmap_set_bits(ldev->regmap, LTDC_SRCR, SRCR_IMR);
	else
		for (layer_index = 0; layer_index < ldev->caps.nb_layers; layer_index++)
			regmap_write_bits(ldev->regmap, LTDC_L1RCR + layer_index * LAY_OFS,
					  LXRCR_IMR | LXRCR_VBR | LXRCR_GRMSK, LXRCR_IMR);

	/* Disable LTDC */
	regmap_clear_bits(ldev->regmap, LTDC_GCR, GCR_LTDCEN);

	/* Set to sleep state the pinctrl whatever type of encoder */
	pinctrl_pm_select_sleep_state(ddev->dev);

	pm_runtime_put_sync_suspend(ddev->dev);

	/* restore to kernel ltdc clock as parent of pixel clock */
	if (ldev->ltdc_clk) {
		ret = clk_set_parent(ldev->pixel_clk, ldev->ltdc_clk);
		if (ret) {
			DRM_ERROR("Could not set parent clock: %d\n", ret);
			return;
		}
	}

	/*  clear interrupt error counters */
	mutex_lock(&ldev->err_lock);
	ldev->transfer_err = 0;
	ldev->fifo_err = 0;
	ldev->fifo_warn = 0;
	ldev->fifo_rot = 0;
	mutex_unlock(&ldev->err_lock);
}

static void ltdc_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct ltdc_device *ldev = crtc_to_ltdc(crtc);
	struct drm_device *ddev = crtc->dev;
	struct drm_connector_list_iter iter;
	struct drm_connector *connector = NULL;
	struct drm_encoder *encoder = NULL, *en_iter;
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	int orientation = DRM_MODE_PANEL_ORIENTATION_UNKNOWN;
	int rate = mode->clock * 1000;
	u32 hsync, vsync, accum_hbp, accum_vbp, accum_act_w, accum_act_h;
	u32 total_width, total_height;
	u32 val;
	int ret;

	DRM_DEBUG_DRIVER("\n");

	/* disable to stream frame if previous session is still alive */
	if (pm_runtime_active(ddev->dev)) {
		regmap_clear_bits(ldev->regmap, LTDC_GCR, GCR_LTDCEN);
		pm_runtime_put_sync_suspend(ddev->dev);
	}

	/* get encoder from crtc */
	drm_for_each_encoder(en_iter, ddev)
		if (en_iter->crtc == crtc) {
			encoder = en_iter;
			break;
		}

	if (encoder) {
		/* Get the connector from encoder */
		drm_connector_list_iter_begin(ddev, &iter);
		drm_for_each_connector_iter(connector, &iter)
			if (connector->encoder == encoder)
				break;
		drm_connector_list_iter_end(&iter);
	}

	if (connector)
		orientation = connector->display_info.panel_orientation;

	if (encoder->encoder_type == DRM_MODE_ENCODER_LVDS) {
		if (ldev->lvds_clk) {
			ret = clk_set_parent(ldev->pixel_clk, ldev->lvds_clk);
			if (ret) {
				DRM_ERROR("Could not set parent clock: %d\n", ret);
				return;
			}
		}
	} else {
		if (ldev->ltdc_clk) {
			ret = clk_set_parent(ldev->pixel_clk, ldev->ltdc_clk);
			if (ret) {
				DRM_ERROR("Could not set parent clock: %d\n", ret);
				return;
			}
		}
	}

	if (clk_set_rate(ldev->pixel_clk, rate) < 0) {
		DRM_ERROR("Cannot set rate (%dHz) for pixel clk\n", rate);
		return;
	}

	/*
	 * Set to default state the pinctrl only with DPI type.
	 * Others types like DSI, don't need pinctrl due to
	 * internal bridge (the signals do not come out of the chipset).
	 */
	if (encoder->encoder_type == DRM_MODE_ENCODER_DPI)
		pinctrl_pm_select_default_state(ddev->dev);
	else
		pinctrl_pm_select_sleep_state(ddev->dev);

	ret = pm_runtime_get_sync(ddev->dev);
	if (ret) {
		DRM_ERROR("Failed to set mode, cannot get sync\n");
		return;
	}

	DRM_DEBUG_DRIVER("CRTC:%d mode:%s\n", crtc->base.id, mode->name);
	DRM_DEBUG_DRIVER("Video mode: %dx%d", mode->hdisplay, mode->vdisplay);
	DRM_DEBUG_DRIVER(" hfp %d hbp %d hsl %d vfp %d vbp %d vsl %d\n",
			 mode->hsync_start - mode->hdisplay,
			 mode->htotal - mode->hsync_end,
			 mode->hsync_end - mode->hsync_start,
			 mode->vsync_start - mode->vdisplay,
			 mode->vtotal - mode->vsync_end,
			 mode->vsync_end - mode->vsync_start);

	/* Convert video timings to ltdc timings */
	hsync = mode->hsync_end - mode->hsync_start - 1;
	vsync = mode->vsync_end - mode->vsync_start - 1;
	accum_hbp = mode->htotal - mode->hsync_start - 1;
	accum_vbp = mode->vtotal - mode->vsync_start - 1;
	accum_act_w = accum_hbp + mode->hdisplay;
	accum_act_h = accum_vbp + mode->vdisplay;
	total_width = mode->htotal - 1;
	total_height = mode->vtotal - 1;

	/* check that an output rotation is required */
	if (ldev->caps.crtc_rotation &&
	    (orientation == DRM_MODE_PANEL_ORIENTATION_LEFT_UP ||
	     orientation == DRM_MODE_PANEL_ORIENTATION_RIGHT_UP)) {
		/* Set Synchronization size */
		val = (vsync << 16) | hsync;
		regmap_update_bits(ldev->regmap, LTDC_SSCR, SSCR_VSH | SSCR_HSW, val);

		/* Set Accumulated Back porch */
		val = (accum_vbp << 16) | accum_hbp;
		regmap_update_bits(ldev->regmap, LTDC_BPCR, BPCR_AVBP | BPCR_AHBP, val);

		/* Set Accumulated Active Width */
		val = (accum_act_h << 16) | accum_act_w;
		regmap_update_bits(ldev->regmap, LTDC_AWCR, AWCR_AAW | AWCR_AAH, val);

		/* Set total width & height */
		val = (total_height << 16) | total_width;
		regmap_update_bits(ldev->regmap, LTDC_TWCR, TWCR_TOTALH | TWCR_TOTALW, val);

		regmap_write(ldev->regmap, LTDC_LIPCR, (accum_act_w + 1));
	} else {
		/* Set Synchronization size */
		val = (hsync << 16) | vsync;
		regmap_update_bits(ldev->regmap, LTDC_SSCR, SSCR_VSH | SSCR_HSW, val);

		/* Set Accumulated Back porch */
		val = (accum_hbp << 16) | accum_vbp;
		regmap_update_bits(ldev->regmap, LTDC_BPCR, BPCR_AVBP | BPCR_AHBP, val);

		/* Set Accumulated Active Width */
		val = (accum_act_w << 16) | accum_act_h;
		regmap_update_bits(ldev->regmap, LTDC_AWCR, AWCR_AAW | AWCR_AAH, val);

		/* Set total width & height */
		val = (total_width << 16) | total_height;
		regmap_update_bits(ldev->regmap, LTDC_TWCR, TWCR_TOTALH | TWCR_TOTALW, val);

		regmap_write(ldev->regmap, LTDC_LIPCR, (accum_act_h + 1));
	}
}

static void ltdc_crtc_atomic_flush(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	struct ltdc_device *ldev = crtc_to_ltdc(crtc);
	struct drm_device *ddev = crtc->dev;
	struct drm_pending_vblank_event *event = crtc->state->event;

	DRM_DEBUG_ATOMIC("\n");

	/* Commit shadow registers = update planes at next vblank */
	if (!ldev->caps.plane_reg_shadow)
		regmap_set_bits(ldev->regmap, LTDC_SRCR, SRCR_VBR);

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&ddev->event_lock);
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&ddev->event_lock);
	}
}

static int ltdc_crtc_atomic_check(struct drm_crtc *crtc,
				  struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	DRM_DEBUG_ATOMIC("\n");

	/* force a full mode set if active state changed */
	if (crtc_state->active_changed)
		crtc_state->mode_changed = true;

	return 0;
}

static bool ltdc_crtc_get_scanout_position(struct drm_crtc *crtc,
					   bool in_vblank_irq,
					   int *vpos, int *hpos,
					   ktime_t *stime, ktime_t *etime,
					   const struct drm_display_mode *mode)
{
	struct drm_device *ddev = crtc->dev;
	struct ltdc_device *ldev = ddev->dev_private;
	int line, vactive_start, vactive_end, vtotal;

	if (stime)
		*stime = ktime_get();

	/* The active area starts after vsync + front porch and ends
	 * at vsync + front porc + display size.
	 * The total height also include back porch.
	 * We have 3 possible cases to handle:
	 * - line < vactive_start: vpos = line - vactive_start and will be
	 * negative
	 * - vactive_start < line < vactive_end: vpos = line - vactive_start
	 * and will be positive
	 * - line > vactive_end: vpos = line - vtotal - vactive_start
	 * and will negative
	 *
	 * Computation for the two first cases are identical so we can
	 * simplify the code and only test if line > vactive_end
	 */
	if (pm_runtime_active(ddev->dev)) {
		regmap_read(ldev->regmap, LTDC_CPSR, &line);
		line &= CPSR_CYPOS;
		regmap_read(ldev->regmap, LTDC_BPCR, &vactive_start);
		vactive_start &= BPCR_AVBP;
		regmap_read(ldev->regmap, LTDC_AWCR, &vactive_end);
		vactive_end &= AWCR_AAH;
		regmap_read(ldev->regmap, LTDC_TWCR, &vtotal);
		vtotal &= TWCR_TOTALH;

		if (line > vactive_end)
			*vpos = line - vtotal - vactive_start;
		else
			*vpos = line - vactive_start;
	} else {
		*vpos = 0;
	}

	*hpos = 0;

	if (etime)
		*etime = ktime_get();

	return true;
}

static const struct drm_crtc_helper_funcs ltdc_crtc_helper_funcs = {
	.mode_set_nofb = ltdc_crtc_mode_set_nofb,
	.atomic_flush = ltdc_crtc_atomic_flush,
	.atomic_enable = ltdc_crtc_atomic_enable,
	.atomic_disable = ltdc_crtc_atomic_disable,
	.atomic_check = ltdc_crtc_atomic_check,
	.get_scanout_position = ltdc_crtc_get_scanout_position,
};

static int ltdc_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct ltdc_device *ldev = crtc_to_ltdc(crtc);
	struct drm_crtc_state *state = crtc->state;

	DRM_DEBUG_DRIVER("\n");

	if (state->enable) {
		ldev->vblank_active = true;
		regmap_set_bits(ldev->regmap, LTDC_IER, IER_LIE);
	} else
		return -EPERM;

	return 0;
}

static void ltdc_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct ltdc_device *ldev = crtc_to_ltdc(crtc);

	DRM_DEBUG_DRIVER("\n");

	ldev->vblank_active = false;

	if (!ldev->vblank_active && !ldev->crc_active)
		regmap_clear_bits(ldev->regmap, LTDC_IER, IER_LIE);
}

static int ltdc_crtc_set_crc_source(struct drm_crtc *crtc, const char *source)
{
	struct ltdc_device *ldev;
	int ret;

	DRM_DEBUG_DRIVER("\n");

	if (!crtc)
		return -ENODEV;

	ldev = crtc_to_ltdc(crtc);

	if (source && strcmp(source, "auto") == 0) {
		ldev->crc_active = true;
		regmap_set_bits(ldev->regmap, LTDC_IER, IER_LIE);
		ret = regmap_set_bits(ldev->regmap, LTDC_GCR, GCR_CRCEN);
	} else if (!source) {
		ldev->crc_active = false;
		if (!ldev->vblank_active && !ldev->crc_active)
			regmap_clear_bits(ldev->regmap, LTDC_IER, IER_LIE);
		ret = regmap_clear_bits(ldev->regmap, LTDC_GCR, GCR_CRCEN);
	} else {
		ret = -EINVAL;
	}

	ldev->crc_skip_count = 0;
	return ret;
}

static int ltdc_crtc_verify_crc_source(struct drm_crtc *crtc,
				       const char *source, size_t *values_cnt)
{
	DRM_DEBUG_DRIVER("\n");

	if (!crtc)
		return -ENODEV;

	if (source && strcmp(source, "auto") != 0) {
		DRM_DEBUG_DRIVER("Unknown CRC source %s for %s\n",
				 source, crtc->name);
		return -EINVAL;
	}

	*values_cnt = 1;
	return 0;
}

static void ltdc_crtc_atomic_print_state(struct drm_printer *p,
					 const struct drm_crtc_state *state)
{
	struct drm_crtc *crtc = state->crtc;
	struct ltdc_device *ldev = crtc_to_ltdc(crtc);

	drm_printf(p, "\ttransfer_error=%d\n", ldev->transfer_err);
	drm_printf(p, "\tfifo_underrun_error=%d\n", ldev->fifo_err);
	drm_printf(p, "\tfifo_underrun_warning=%d\n", ldev->fifo_warn);
	drm_printf(p, "\tfifo_underrun_rotation=%d\n", ldev->fifo_rot);
	drm_printf(p, "\tfifo_underrun_threshold=%d\n", ldev->fifo_threshold);
}

static const struct drm_crtc_funcs ltdc_crtc_funcs = {
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = ltdc_crtc_enable_vblank,
	.disable_vblank = ltdc_crtc_disable_vblank,
	.get_vblank_timestamp = drm_crtc_vblank_helper_get_vblank_timestamp,
	.atomic_print_state = ltdc_crtc_atomic_print_state,
};

static const struct drm_crtc_funcs ltdc_crtc_with_crc_support_funcs = {
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = ltdc_crtc_enable_vblank,
	.disable_vblank = ltdc_crtc_disable_vblank,
	.get_vblank_timestamp = drm_crtc_vblank_helper_get_vblank_timestamp,
	.set_crc_source = ltdc_crtc_set_crc_source,
	.verify_crc_source = ltdc_crtc_verify_crc_source,
	.atomic_print_state = ltdc_crtc_atomic_print_state,
};

/*
 * DRM_PLANE
 */

static int ltdc_plane_atomic_check(struct drm_plane *plane,
				   struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = new_plane_state->fb;
	struct ltdc_device *ldev = plane_to_ltdc(plane);
	u32 src_w, src_h, crtc_w, crtc_h;

	DRM_DEBUG_DRIVER("\n");

	if (!fb)
		return 0;

	/* convert src_ from 16:16 format */
	src_w = new_plane_state->src_w >> 16;
	src_h = new_plane_state->src_h >> 16;
	crtc_w = new_plane_state->crtc_w;
	crtc_h = new_plane_state->crtc_h;

	if (ldev->caps.plane_scaling[plane->index]) {
		if (fb->format->is_yuv) {
			if (src_w != crtc_w || src_h != crtc_h) {
				DRM_DEBUG_DRIVER("Scaling is not supported with yuv pixel format");
				return -EINVAL;
			}
		}

		if (src_w > crtc_w || src_h > crtc_h) {
			DRM_DEBUG_DRIVER("Downscaling is not supported");
			return -EINVAL;
		}
	} else {
		if (src_w != crtc_w || src_h != crtc_h) {
			DRM_DEBUG_DRIVER("Scaling is not supported on layer %d", plane->index);
			return -EINVAL;
		}
	}

	return 0;
}

#define SCALER_FRACTION 12

static u32 calculateScalingFactor(u32 in, u32 out)
{
	u32 factor = 0x1000;

	if (out != 1)
		factor = ((in - 1) << SCALER_FRACTION) / (out - 1);

	return factor & 0xFFFF;
}

static void ltdc_plane_update_clut(struct drm_plane *plane,
				   struct drm_plane_state *state)
{
	struct ltdc_device *ldev = plane_to_ltdc(plane);
	struct drm_crtc *crtc = state->crtc;
	struct drm_color_lut *lut;
	u32 lofs = plane->index * LAY_OFS;
	u32 val;
	int lut_size;
	int i;

	if (!crtc || !crtc->state)
		return;

	if (!crtc->state->color_mgmt_changed || !crtc->state->gamma_lut)
		return;

	lut = (struct drm_color_lut *)crtc->state->gamma_lut->data;
	lut_size = drm_color_lut_size(crtc->state->gamma_lut);
	if (lut_size > CLUT_SIZE)
		return;

	for (i = 0; i < CLUT_SIZE; i++, lut++) {
		val = ((lut->red << 8) & 0xff0000) | (lut->green & 0xff00) |
		      (lut->blue >> 8) | (i << 24);
		regmap_write(ldev->regmap, LTDC_L1CLUTWR + lofs, val);
	}
}

static void ltdc_plane_atomic_update(struct drm_plane *plane,
				     struct drm_atomic_state *state)
{
	struct ltdc_device *ldev = plane_to_ltdc(plane);
	struct drm_device *ddev = plane->dev;
	struct device *dev = ddev->dev;
	struct drm_plane_state *newstate = drm_atomic_get_new_plane_state(state,
									  plane);
	struct drm_framebuffer *fb = newstate->fb;
	u32 lofs = plane->index * LAY_OFS;
	u32 val, pitch_in_bytes, line_length, line_number, ahbp, avbp, bpcr;
	u32 paddr, paddr1, paddr2, lxcr;
	enum ltdc_pix_fmt pf;
	unsigned int plane_rotation = newstate->rotation;
	struct drm_connector_list_iter co_iter;
	struct drm_connector *connector = NULL;
	struct drm_encoder *encoder = NULL, *en_iter;
	struct drm_rect dst, src;
	int orientation = DRM_MODE_PANEL_ORIENTATION_UNKNOWN;

	if (!newstate->crtc || !fb) {
		DRM_DEBUG_DRIVER("fb or crtc NULL");
		return;
	}

	/* get encoder from crtc */
	drm_for_each_encoder(en_iter, ddev)
		if (en_iter->crtc == newstate->crtc) {
			encoder = en_iter;
			break;
		}

	if (encoder) {
		/* Get the connector from encoder */
		drm_connector_list_iter_begin(ddev, &co_iter);
		drm_for_each_connector_iter(connector, &co_iter)
			if (connector->encoder == encoder)
				break;
		drm_connector_list_iter_end(&co_iter);

		if (connector)
			orientation = connector->display_info.panel_orientation;
	}

	/* convert src_ from 16:16 format */
	drm_rect_init(&src, newstate->src_x >> 16, newstate->src_y >> 16,
		      newstate->src_w >> 16, newstate->src_h >> 16);

	drm_rect_init(&dst, newstate->crtc_x, newstate->crtc_y,
		      newstate->crtc_w, newstate->crtc_h);

	DRM_DEBUG_DRIVER("plane:%d fb:%d src: " DRM_RECT_FMT " -> crtc: " DRM_RECT_FMT "\n",
			 plane->base.id, fb->base.id, DRM_RECT_ARG(&src), DRM_RECT_ARG(&dst));

	if (!pm_runtime_active(ddev->dev))
		return;

	regmap_read(ldev->regmap, LTDC_BPCR, &bpcr);

	if (ldev->caps.crtc_rotation &&
	    (orientation == DRM_MODE_PANEL_ORIENTATION_RIGHT_UP ||
	     orientation == DRM_MODE_PANEL_ORIENTATION_LEFT_UP)) {
		avbp = (bpcr & BPCR_AHBP) >> 16;
		ahbp = bpcr & BPCR_AVBP;

		/* Configures the horizontal start and stop position */
		val = (dst.x1 + 1 + ahbp) + ((dst.x2 + ahbp) << 16);
		regmap_write_bits(ldev->regmap, LTDC_L1WHPCR + lofs,
				  LXWHPCR_WHSTPOS | LXWHPCR_WHSPPOS, val);

		/* Configures the vertical start and stop position */
		val = (dst.y1 + 1 + avbp) + ((dst.y2 + avbp) << 16);
		regmap_write_bits(ldev->regmap, LTDC_L1WVPCR + lofs,
				  LXWVPCR_WVSTPOS | LXWVPCR_WVSPPOS, val);

		/* need to mirroring on X (rotation will switch lines & columns,
		   not a real rotate */
		if (orientation == DRM_MODE_PANEL_ORIENTATION_RIGHT_UP) {
			if (plane_rotation & DRM_MODE_REFLECT_X)
				plane_rotation &= ~DRM_MODE_REFLECT_X;
			else
				plane_rotation |= DRM_MODE_REFLECT_X;
		}

		/* need to mirroring on Y (rotation will switch lines & columns,
		   not a real rotate */
		if (orientation == DRM_MODE_PANEL_ORIENTATION_LEFT_UP) {
			if (plane_rotation & DRM_MODE_REFLECT_Y)
				plane_rotation &= ~DRM_MODE_REFLECT_Y;
			else
				plane_rotation |= DRM_MODE_REFLECT_Y;
		}
	} else {
		ahbp = (bpcr & BPCR_AHBP) >> 16;
		avbp = bpcr & BPCR_AVBP;

		/* Configures the horizontal start and stop position */
		val = ((dst.x2 + ahbp) << 16) + (dst.x1 + 1 + ahbp);
		regmap_write_bits(ldev->regmap, LTDC_L1WHPCR + lofs,
				  LXWHPCR_WHSTPOS | LXWHPCR_WHSPPOS, val);

		/* Configures the vertical start and stop position */
		val = ((dst.y2 + avbp) << 16) + (dst.y1 + 1 + avbp);
		regmap_write_bits(ldev->regmap, LTDC_L1WVPCR + lofs,
				  LXWVPCR_WVSTPOS | LXWVPCR_WVSPPOS, val);

		if (orientation == DRM_MODE_PANEL_ORIENTATION_BOTTOM_UP) {
			if (plane_rotation & DRM_MODE_REFLECT_X)
				plane_rotation &= ~DRM_MODE_REFLECT_X;
			else
				plane_rotation |= DRM_MODE_REFLECT_X;

			if (plane_rotation & DRM_MODE_REFLECT_Y)
				plane_rotation &= ~DRM_MODE_REFLECT_Y;
			else
				plane_rotation |= DRM_MODE_REFLECT_Y;
		}
	}

	/* Specifies the pixel format */
	pf = to_ltdc_pixelformat(fb->format->format);
	for (val = 0; val < NB_PF; val++)
		if (ldev->caps.pix_fmt_hw[val] == pf)
			break;

	/* Use the flexible color format feature if necessary and available */
	if (ldev->caps.pix_fmt_flex && val == NB_PF)
		val = ltdc_set_flexible_pixel_format(plane, pf);

	if (val == NB_PF) {
		DRM_ERROR("Pixel format %.4s not supported\n",
			  (char *)&fb->format->format);
		val = 0;	/* set by default ARGB 32 bits */
	}
	regmap_write_bits(ldev->regmap, LTDC_L1PFCR + lofs, LXPFCR_PF, val);

	/* Specifies the constant alpha value */
	val = newstate->alpha >> 8;
	regmap_write_bits(ldev->regmap, LTDC_L1CACR + lofs, LXCACR_CONSTA, val);

	/* Sets the default color to black */
	regmap_write(ldev->regmap, LTDC_L1DCCR + lofs, DCCR_DCBLACK);

	/* Specifies the blending factors */
	val = BF1_PAXCA | BF2_1PAXCA;
	if (!fb->format->has_alpha)
		val = BF1_CA | BF2_1CA;

	/* Manage hw-specific capabilities */
	if (ldev->caps.non_alpha_only_l1 &&
	    plane->type != DRM_PLANE_TYPE_PRIMARY)
		val = BF1_PAXCA | BF2_1PAXCA;

	if (ldev->caps.dynamic_zorder) {
		val |= (newstate->normalized_zpos << 16);
		regmap_write_bits(ldev->regmap, LTDC_L1BFCR + lofs,
				  LXBFCR_BF2 | LXBFCR_BF1 | LXBFCR_BOR, val);
	} else {
		regmap_write_bits(ldev->regmap, LTDC_L1BFCR + lofs,
				  LXBFCR_BF2 | LXBFCR_BF1, val);
	}

	/* Sets the FB address */
	paddr = (u32)drm_fb_dma_get_gem_addr(fb, newstate, 0);

	if (plane_rotation & DRM_MODE_REFLECT_X)
		paddr += (fb->format->cpp[0] * drm_rect_width(&src)) - 1;

	if (plane_rotation & DRM_MODE_REFLECT_Y)
		paddr += (fb->pitches[0] * (drm_rect_height(&src) - 1));

	DRM_DEBUG_DRIVER("fb: phys 0x%08x", paddr);
	regmap_write(ldev->regmap, LTDC_L1CFBAR + lofs, paddr);

	/* Configures the color frame buffer pitch in bytes & line length */
	line_length = fb->format->cpp[0] * drm_rect_width(&src) + (ldev->caps.bus_width >> 3) - 1;

	if (plane_rotation & DRM_MODE_REFLECT_Y)
		/* Compute negative value (signed on 16 bits) for the picth */
		pitch_in_bytes = 0x10000 - fb->pitches[0];
	else
		pitch_in_bytes = fb->pitches[0];

	val = (pitch_in_bytes << 16) | line_length;
	regmap_write_bits(ldev->regmap, LTDC_L1CFBLR + lofs, LXCFBLR_CFBLL | LXCFBLR_CFBP, val);

	/* Configures the frame buffer line number */
	line_number = drm_rect_height(&src);
	regmap_write_bits(ldev->regmap, LTDC_L1CFBLNR + lofs, LXCFBLNR_CFBLN, line_number);

	if (ldev->caps.ycbcr_input) {
		if (fb->format->is_yuv) {
			switch (fb->format->format) {
			case DRM_FORMAT_NV12:
			case DRM_FORMAT_NV21:
			/* Configure the auxiliary frame buffer address 0 */
			paddr1 = (u32)drm_fb_dma_get_gem_addr(fb, newstate, 1);

			if (plane_rotation & DRM_MODE_REFLECT_X)
				paddr1 += ((fb->format->cpp[1] * drm_rect_width(&src)) >> 1) - 1;

			if (plane_rotation & DRM_MODE_REFLECT_Y)
				paddr1 += (fb->pitches[1] * (drm_rect_height(&src) - 1)) >> 1;

			regmap_write(ldev->regmap, LTDC_L1AFBA0R + lofs, paddr1);
			break;
			case DRM_FORMAT_YUV420:
			/* Configure the auxiliary frame buffer address 0 & 1 */
			paddr1 = (u32)drm_fb_dma_get_gem_addr(fb, newstate, 1);
			paddr2 = (u32)drm_fb_dma_get_gem_addr(fb, newstate, 2);

			if (plane_rotation & DRM_MODE_REFLECT_X) {
				paddr1 += ((fb->format->cpp[1] * drm_rect_width(&src)) >> 1) - 1;
				paddr2 += ((fb->format->cpp[2] * drm_rect_width(&src)) >> 1) - 1;
			}

			if (plane_rotation & DRM_MODE_REFLECT_Y) {
				paddr1 += (fb->pitches[1] * (drm_rect_height(&src) - 1)) >> 1;
				paddr2 += (fb->pitches[2] * (drm_rect_height(&src) - 1)) >> 1;
			}

			regmap_write(ldev->regmap, LTDC_L1AFBA0R + lofs, paddr1);
			regmap_write(ldev->regmap, LTDC_L1AFBA1R + lofs, paddr2);
			break;
			case DRM_FORMAT_YVU420:
			/* Configure the auxiliary frame buffer address 0 & 1 */
			paddr1 = (u32)drm_fb_dma_get_gem_addr(fb, newstate, 2);
			paddr2 = (u32)drm_fb_dma_get_gem_addr(fb, newstate, 1);

			if (plane_rotation & DRM_MODE_REFLECT_X) {
				paddr1 += ((fb->format->cpp[1] * drm_rect_width(&src)) >> 1) - 1;
				paddr2 += ((fb->format->cpp[2] * drm_rect_width(&src)) >> 1) - 1;
			}

			if (plane_rotation & DRM_MODE_REFLECT_Y) {
				paddr1 += (fb->pitches[1] * (drm_rect_height(&src) - 1)) >> 1;
				paddr2 += (fb->pitches[2] * (drm_rect_height(&src) - 1)) >> 1;
			}

			regmap_write(ldev->regmap, LTDC_L1AFBA0R + lofs, paddr1);
			regmap_write(ldev->regmap, LTDC_L1AFBA1R + lofs, paddr2);
			break;
			}

			/*
			 * Set the length and the number of lines of the auxiliary
			 * buffers if the framebuffer contains more than one plane.
			 */
			if (fb->format->num_planes > 1) {
				if (plane_rotation & DRM_MODE_REFLECT_Y)
					/*
					 * Compute negative value (signed on 16 bits)
					 * for the picth
					 */
					pitch_in_bytes = 0x10000 - fb->pitches[1];
				else
					pitch_in_bytes = fb->pitches[1];

				line_length = ((fb->format->cpp[1] * drm_rect_width(&src)) >> 1) +
					      (ldev->caps.bus_width >> 3) - 1;

				/* Configure the auxiliary buffer length */
				val = (pitch_in_bytes << 16) | line_length;
				regmap_write(ldev->regmap, LTDC_L1AFBLR + lofs, val);

				/* Configure the auxiliary frame buffer line number */
				val = line_number >> 1;
				regmap_write(ldev->regmap, LTDC_L1AFBLNR + lofs, val);
			}

			/* Configure YCbC conversion coefficient */
			ltdc_set_ycbcr_coeffs(plane);

			/* Configure YCbCr format and enable/disable conversion */
			ltdc_set_ycbcr_config(plane, fb->format->format);
		} else {
			/* disable ycbcr conversion */
			regmap_write(ldev->regmap, LTDC_L1PCR + lofs, 0);
		}
	}

	/* Configure burst length */
	if (of_device_is_compatible(dev->of_node, "st,stm32mp25-ltdc"))
		regmap_write(ldev->regmap, LTDC_L1BLCR + lofs, ldev->max_burst_length);

	/* set color look-up table */
	if (fb->format->format == DRM_FORMAT_C8)
		ltdc_plane_update_clut(plane, newstate);

	/* Enable layer and CLUT if needed */
	lxcr = fb->format->format == DRM_FORMAT_C8 ? LXCR_CLUTEN : 0;
	lxcr |= LXCR_LEN;

	/* Enable horizontal mirroring if requested */
	if (plane_rotation & DRM_MODE_REFLECT_X)
		lxcr |= LXCR_HMEN;

	if (ldev->caps.plane_scaling[plane->index] &&
	    (drm_rect_width(&src) != drm_rect_width(&dst) ||
	     drm_rect_height(&src) != drm_rect_height(&dst))) {
		lxcr |= LXCR_SCEN;

		/* Configure the scaler input size */
		val = (drm_rect_height(&src) << 16U) | drm_rect_width(&src);
		regmap_write(ldev->regmap, LTDC_L1SISR + lofs, val);

		/* Configure the scaler output size */
		val = (drm_rect_height(&dst) << 16U) | drm_rect_width(&dst);
		regmap_write(ldev->regmap, LTDC_L1SOSR + lofs, val);

		/* Configure the vertical scaling factor */
		val = calculateScalingFactor(drm_rect_height(&src), drm_rect_height(&dst));
		regmap_write(ldev->regmap, LTDC_L1SVSFR + lofs, val);

		/* Configure the vertical scaling phase */
		regmap_write(ldev->regmap, LTDC_L1SHSPR + lofs, val);

		/* Configure the horizontal scaling factor */
		val = calculateScalingFactor(drm_rect_width(&src), drm_rect_width(&dst));
		regmap_write(ldev->regmap, LTDC_L1SHSFR + lofs, val);

		/* Configure the vertical scaling phase */
		regmap_write(ldev->regmap, LTDC_L1SVSPR + lofs, val + (1 << SCALER_FRACTION));
	}

	regmap_write_bits(ldev->regmap, LTDC_L1CR + lofs, LXCR_MASK, lxcr);

	/* Commit shadow registers = update plane at next vblank */
	if (ldev->caps.plane_reg_shadow)
		regmap_write_bits(ldev->regmap, LTDC_L1RCR + lofs,
				  LXRCR_IMR | LXRCR_VBR | LXRCR_GRMSK, LXRCR_VBR);

	ldev->plane_fpsi[plane->index].counter++;

	mutex_lock(&ldev->err_lock);
	if (ldev->transfer_err) {
		DRM_WARN("ltdc transfer error: %d\n", ldev->transfer_err);
		ldev->transfer_err = 0;
	}

	if (ldev->fifo_rot) {
		DRM_WARN("ltdc fifo rotation error\n");
		ldev->fifo_rot = 0;
	}

	if (ldev->caps.fifo_threshold) {
		if (ldev->fifo_err) {
			DRM_WARN("ltdc fifo underrun: please verify display mode\n");
			ldev->fifo_err = 0;
		}
	} else {
		if (ldev->fifo_warn >= ldev->fifo_threshold) {
			DRM_WARN("ltdc fifo underrun: please verify display mode\n");
			ldev->fifo_warn = 0;
		}
	}
	mutex_unlock(&ldev->err_lock);
}

static void ltdc_plane_atomic_disable(struct drm_plane *plane,
				      struct drm_atomic_state *state)
{
	struct drm_plane_state *oldstate = drm_atomic_get_old_plane_state(state,
									  plane);
	struct ltdc_device *ldev = plane_to_ltdc(plane);
	struct drm_device *ddev = plane->dev;
	u32 lofs = plane->index * LAY_OFS;

	if (!pm_runtime_active(ddev->dev))
		return;

	/* Disable layer */
	regmap_write_bits(ldev->regmap, LTDC_L1CR + lofs, LXCR_MASK, 0);

	/* Set the transparency of the layer to the default value */
	regmap_write_bits(ldev->regmap, LTDC_L1CACR + lofs, LXCACR_CONSTA, 0x00);

	/* Commit shadow registers = update plane at next vblank */
	if (ldev->caps.plane_reg_shadow)
		regmap_write_bits(ldev->regmap, LTDC_L1RCR + lofs,
				  LXRCR_IMR | LXRCR_VBR | LXRCR_GRMSK, LXRCR_VBR);

	DRM_DEBUG_DRIVER("CRTC:%d plane:%d\n",
			 oldstate->crtc->base.id, plane->base.id);
}

static void ltdc_plane_atomic_print_state(struct drm_printer *p,
					  const struct drm_plane_state *state)
{
	struct drm_plane *plane = state->plane;
	struct ltdc_device *ldev = plane_to_ltdc(plane);
	struct fps_info *fpsi = &ldev->plane_fpsi[plane->index];
	int ms_since_last;
	ktime_t now;

	now = ktime_get();
	ms_since_last = ktime_to_ms(ktime_sub(now, fpsi->last_timestamp));

	drm_printf(p, "\tuser_updates=%dfps\n",
		   DIV_ROUND_CLOSEST(fpsi->counter * 1000, ms_since_last));

	fpsi->last_timestamp = now;
	fpsi->counter = 0;
}

static const struct drm_plane_funcs ltdc_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
	.atomic_print_state = ltdc_plane_atomic_print_state,
};

static const struct drm_plane_helper_funcs ltdc_plane_helper_funcs = {
	.atomic_check = ltdc_plane_atomic_check,
	.atomic_update = ltdc_plane_atomic_update,
	.atomic_disable = ltdc_plane_atomic_disable,
};

static struct drm_plane *ltdc_plane_create(struct drm_device *ddev,
					   enum drm_plane_type type,
					   int index)
{
	unsigned long possible_crtcs = CRTC_MASK;
	struct ltdc_device *ldev = ddev->dev_private;
	struct device *dev = ddev->dev;
	struct drm_plane *plane;
	unsigned int i, nb_fmt = 0;
	u32 *formats;
	u32 drm_fmt;
	const u64 *modifiers = ltdc_format_modifiers;
	u32 lofs = index * LAY_OFS;
	u32 val;
	int ret;

	/* Allocate the biggest size according to supported color formats */
	formats = devm_kzalloc(dev, (ldev->caps.pix_fmt_nb +
			       ARRAY_SIZE(ltdc_drm_fmt_ycbcr_cp) +
			       ARRAY_SIZE(ltdc_drm_fmt_ycbcr_sp) +
			       ARRAY_SIZE(ltdc_drm_fmt_ycbcr_fp)) *
			       sizeof(*formats), GFP_KERNEL);

	for (i = 0; i < ldev->caps.pix_fmt_nb; i++) {
		drm_fmt = ldev->caps.pix_fmt_drm[i];

		/* Manage hw-specific capabilities */
		if (ldev->caps.non_alpha_only_l1)
			/* XR24 & RX24 like formats supported only on primary layer */
			if (type != DRM_PLANE_TYPE_PRIMARY && is_xrgb(drm_fmt))
				continue;

		formats[nb_fmt++] = drm_fmt;
	}

	/* Add YCbCr supported pixel formats */
	if (ldev->caps.ycbcr_input) {
		regmap_read(ldev->regmap, LTDC_L1C1R + lofs, &val);
		if (val & LXCR_C1R_YIA) {
			memcpy(&formats[nb_fmt], ltdc_drm_fmt_ycbcr_cp,
			       ARRAY_SIZE(ltdc_drm_fmt_ycbcr_cp) * sizeof(*formats));
			nb_fmt += ARRAY_SIZE(ltdc_drm_fmt_ycbcr_cp);
		}

		/*
		 * Soc MP25 doesn't support pixel formats yuv semiplanar &
		 * planar on layer1 only.
		 */
		if (!(of_device_is_compatible(dev->of_node, "st,stm32mp25-ltdc") && !index)) {
			if (val & LXCR_C1R_YSPA) {
				memcpy(&formats[nb_fmt], ltdc_drm_fmt_ycbcr_sp,
				       ARRAY_SIZE(ltdc_drm_fmt_ycbcr_sp) * sizeof(*formats));
				nb_fmt += ARRAY_SIZE(ltdc_drm_fmt_ycbcr_sp);
			}
			if (val & LXCR_C1R_YFPA) {
				memcpy(&formats[nb_fmt], ltdc_drm_fmt_ycbcr_fp,
				       ARRAY_SIZE(ltdc_drm_fmt_ycbcr_fp) * sizeof(*formats));
				nb_fmt += ARRAY_SIZE(ltdc_drm_fmt_ycbcr_fp);
			}
		}
	}

	plane = devm_kzalloc(dev, sizeof(*plane), GFP_KERNEL);
	if (!plane)
		return NULL;

	ret = drm_universal_plane_init(ddev, plane, possible_crtcs,
				       &ltdc_plane_funcs, formats, nb_fmt,
				       modifiers, type, NULL);
	if (ret < 0)
		return NULL;

	if (ldev->caps.ycbcr_input) {
		if (val & (LXCR_C1R_YIA | LXCR_C1R_YSPA | LXCR_C1R_YFPA))
			drm_plane_create_color_properties(plane,
							  BIT(DRM_COLOR_YCBCR_BT601) |
							  BIT(DRM_COLOR_YCBCR_BT709),
							  BIT(DRM_COLOR_YCBCR_LIMITED_RANGE) |
							  BIT(DRM_COLOR_YCBCR_FULL_RANGE),
							  DRM_COLOR_YCBCR_BT601,
							  DRM_COLOR_YCBCR_LIMITED_RANGE);
	}

	drm_plane_helper_add(plane, &ltdc_plane_helper_funcs);

	drm_plane_create_alpha_property(plane);

	DRM_DEBUG_DRIVER("plane:%d created\n", plane->base.id);

	return plane;
}

static void ltdc_plane_destroy_all(struct drm_device *ddev)
{
	struct drm_plane *plane, *plane_temp;

	list_for_each_entry_safe(plane, plane_temp,
				 &ddev->mode_config.plane_list, head)
		drm_plane_cleanup(plane);
}

static int ltdc_crtc_init(struct drm_device *ddev, struct drm_crtc *crtc)
{
	struct ltdc_device *ldev = ddev->dev_private;
	struct drm_plane *primary, *overlay;
	int supported_rotations = DRM_MODE_ROTATE_0 | DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y;
	unsigned int i;
	int ret;
	struct drm_connector *connector = NULL;
	struct drm_connector_list_iter iter;

	/* Add the dithering property to all connectors */
	drm_connector_list_iter_begin(ddev, &iter);
	drm_for_each_connector_iter(connector, &iter)
		drm_connector_attach_dithering_property(connector,
							BIT(DRM_MODE_DITHERING_OFF) |
							BIT(DRM_MODE_DITHERING_ON));
	drm_connector_list_iter_end(&iter);

	primary = ltdc_plane_create(ddev, DRM_PLANE_TYPE_PRIMARY, 0);
	if (!primary) {
		DRM_ERROR("Can not create primary plane\n");
		return -EINVAL;
	}

	if (ldev->caps.dynamic_zorder)
		drm_plane_create_zpos_property(primary, 0, 0, ldev->caps.nb_layers - 1);
	else
		drm_plane_create_zpos_immutable_property(primary, 0);

	if (ldev->caps.plane_rotation)
		drm_plane_create_rotation_property(primary, DRM_MODE_ROTATE_0,
						   supported_rotations);

	/* Init CRTC according to its hardware features */
	if (ldev->caps.crc)
		ret = drm_crtc_init_with_planes(ddev, crtc, primary, NULL,
						&ltdc_crtc_with_crc_support_funcs, NULL);
	else
		ret = drm_crtc_init_with_planes(ddev, crtc, primary, NULL,
						&ltdc_crtc_funcs, NULL);
	if (ret) {
		DRM_ERROR("Can not initialize CRTC\n");
		goto cleanup;
	}

	drm_crtc_helper_add(crtc, &ltdc_crtc_helper_funcs);

	drm_mode_crtc_set_gamma_size(crtc, CLUT_SIZE);
	drm_crtc_enable_color_mgmt(crtc, 0, false, CLUT_SIZE);

	DRM_DEBUG_DRIVER("CRTC:%d created\n", crtc->base.id);

	/* Add planes. Note : the first layer is used by primary plane */
	for (i = 1; i < ldev->caps.nb_layers; i++) {
		overlay = ltdc_plane_create(ddev, DRM_PLANE_TYPE_OVERLAY, i);
		if (!overlay) {
			ret = -ENOMEM;
			DRM_ERROR("Can not create overlay plane %d\n", i);
			goto cleanup;
		}
		if (ldev->caps.dynamic_zorder)
			drm_plane_create_zpos_property(overlay, i, 0, ldev->caps.nb_layers - 1);
		else
			drm_plane_create_zpos_immutable_property(overlay, i);

		if (ldev->caps.plane_rotation)
			drm_plane_create_rotation_property(overlay, DRM_MODE_ROTATE_0,
							   supported_rotations);
	}

	return 0;

cleanup:
	ltdc_plane_destroy_all(ddev);
	return ret;
}

#define CLK_TOLERANCE_HZ 50

static enum drm_mode_status ltdc_encoder_mode_valid(struct drm_encoder *encoder,
						    const struct drm_display_mode *mode)
{
	struct drm_device *ddev = encoder->dev;
	struct ltdc_device *ldev =  ddev->dev_private;
	struct device *dev = ddev->dev;
	struct drm_connector *connector = NULL;
	struct drm_connector_list_iter iter;
	int orientation = DRM_MODE_PANEL_ORIENTATION_UNKNOWN;
	int target = mode->clock * 1000;
	int target_min = target - CLK_TOLERANCE_HZ;
	int target_max = target + CLK_TOLERANCE_HZ;
	int result;

	if (of_device_is_compatible(dev->of_node, "st,stm32mp25-ltdc")) {
		if (encoder->encoder_type == DRM_MODE_ENCODER_LVDS)
			result = clk_round_rate(ldev->lvds_clk, target);
		else
			result = clk_round_rate(ldev->ltdc_clk, target);
	} else {
		result = clk_round_rate(ldev->pixel_clk, target);
	}

	DRM_DEBUG_DRIVER("clk rate target %d, available %d\n", target, result);

	/* Get the connector from encoder */
	drm_connector_list_iter_begin(ddev, &iter);
	drm_for_each_connector_iter(connector, &iter)
		if (connector->encoder == encoder)
			break;
	drm_connector_list_iter_end(&iter);

	if (connector)
		orientation = connector->display_info.panel_orientation;

	/* check that an output rotation is required */
	if (ldev->caps.crtc_rotation &&
	    (orientation == DRM_MODE_PANEL_ORIENTATION_LEFT_UP ||
	     orientation == DRM_MODE_PANEL_ORIENTATION_RIGHT_UP)) {
		/*
		 * Size of the rotation buffer must be larger than the size
		 * of two frames (format RGB24).
		 */
		if (ldev->rot_mem->size < mode->hdisplay * mode->vdisplay * 2 * 3)
			return MODE_MEM;

		/* The width of the framebuffer must not exceed 1366 pixels */
		if (mode->vdisplay > 1366)
			return MODE_BAD_WIDTH;
	}

	/* Filter modes according to the max frequency supported by the pads */
	if (result > ldev->caps.pad_max_freq_hz)
		return MODE_CLOCK_HIGH;

	/*
	 * Accept all "preferred" modes:
	 * - this is important for panels because panel clock tolerances are
	 *   bigger than hdmi ones and there is no reason to not accept them
	 *   (the fps may vary a little but it is not a problem).
	 * - the hdmi preferred mode will be accepted too, but userland will
	 *   be able to use others hdmi "valid" modes if necessary.
	 */
	if (mode->type & DRM_MODE_TYPE_PREFERRED)
		return MODE_OK;

	/*
	 * Filter modes according to the clock value, particularly useful for
	 * hdmi modes that require precise pixel clocks.
	 */
	if (result < target_min || result > target_max)
		return MODE_CLOCK_RANGE;

	return MODE_OK;
}

static bool ltdc_encoder_mode_fixup(struct drm_encoder *encoder,
				    const struct drm_display_mode *mode,
				    struct drm_display_mode *adjusted_mode)
{
	struct drm_device *ddev = encoder->dev;
	struct ltdc_device *ldev =  ddev->dev_private;
	int rate = mode->clock * 1000;

	if (clk_set_rate(ldev->pixel_clk, rate) < 0) {
		DRM_ERROR("Cannot set rate (%dHz) for pixel clk\n", rate);
		return false;
	}

	adjusted_mode->clock = clk_get_rate(ldev->pixel_clk) / 1000;

	DRM_DEBUG_DRIVER("requested clock %dkHz, adjusted clock %dkHz\n",
			 mode->clock, adjusted_mode->clock);

	return true;
}

static const struct drm_encoder_helper_funcs ltdc_encoder_helper_funcs = {
	.mode_fixup = ltdc_encoder_mode_fixup,
	.mode_valid = ltdc_encoder_mode_valid,
};

static int ltdc_encoder_init(struct drm_device *ddev, struct drm_bridge *bridge)
{
	struct drm_encoder *encoder;
	int ret;

	encoder = devm_kzalloc(ddev->dev, sizeof(*encoder), GFP_KERNEL);
	if (!encoder)
		return -ENOMEM;

	encoder->possible_crtcs = CRTC_MASK;
	encoder->possible_clones = 0;	/* No cloning support */

	drm_simple_encoder_init(ddev, encoder, DRM_MODE_ENCODER_DPI);

	drm_encoder_helper_add(encoder, &ltdc_encoder_helper_funcs);

	ret = drm_bridge_attach(encoder, bridge, NULL, 0);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			drm_encoder_cleanup(encoder);
		return ret;
	}

	DRM_DEBUG_DRIVER("Bridge encoder:%d created\n", encoder->base.id);

	return 0;
}

static int ltdc_get_caps(struct drm_device *ddev)
{
	struct ltdc_device *ldev = ddev->dev_private;
	struct device *dev = ddev->dev;
	u32 bus_width_log2, lcr, gc2r, lxc1r;
	const struct ltdc_plat_data *pdata = of_device_get_match_data(ddev->dev);
	int ret;

	/*
	 * at least 1 layer must be managed & the number of layers
	 * must not exceed LTDC_MAX_LAYER
	 */
	regmap_read(ldev->regmap, LTDC_LCR, &lcr);

	ldev->caps.nb_layers = clamp((int)lcr, 1, LTDC_MAX_LAYER);

	/*
	 * Check the security of layer 2.
	 * Do not expose this layer to the user (do not create a plan)
	 * if this one is reserved for secure application.
	 */
	if (of_device_is_compatible(dev->of_node, "st,stm32mp25-ltdc")) {
		ret = stm32_rifsc_check_access_by_id(STM32MP25_RIFSC_LTDC_L2_ID);
		if (ret)
			ldev->caps.nb_layers--;
	}

	/* set data bus width */
	regmap_read(ldev->regmap, LTDC_GC2R, &gc2r);
	bus_width_log2 = (gc2r & GC2R_BW) >> 4;
	ldev->caps.bus_width = 8 << bus_width_log2;
	regmap_read(ldev->regmap, LTDC_IDR, &ldev->caps.hw_version);

	ldev->caps.pad_max_freq_hz = pdata->pad_max_freq_hz;

	switch (ldev->caps.hw_version) {
	case HWVER_10200:
	case HWVER_10300:
		ldev->caps.layer_ofs = LAY_OFS_0;
		ldev->caps.layer_regs = ltdc_layer_regs_a0;
		ldev->caps.pix_fmt_hw = ltdc_pix_fmt_a0;
		ldev->caps.pix_fmt_drm = ltdc_drm_fmt_a0;
		ldev->caps.pix_fmt_nb = ARRAY_SIZE(ltdc_drm_fmt_a0);
		ldev->caps.pix_fmt_flex = false;
		/*
		 * Hw older versions support non-alpha color formats derived
		 * from native alpha color formats only on the primary layer.
		 * For instance, RG16 native format without alpha works fine
		 * on 2nd layer but XR24 (derived color format from AR24)
		 * does not work on 2nd layer.
		 */
		ldev->caps.non_alpha_only_l1 = true;
		if (ldev->caps.hw_version == HWVER_10200)
			ldev->caps.pad_max_freq_hz = 65000000;
		ldev->caps.nb_irq = 2;
		ldev->caps.ycbcr_input = false;
		ldev->caps.ycbcr_output = false;
		ldev->caps.plane_reg_shadow = false;
		ldev->caps.crc = false;
		ldev->caps.dynamic_zorder = false;
		ldev->caps.plane_rotation = false;
		ldev->caps.crtc_rotation = false;
		ldev->caps.fifo_threshold = false;
		break;
	case HWVER_20101:
		ldev->caps.layer_ofs = LAY_OFS_0;
		ldev->caps.layer_regs = ltdc_layer_regs_a1;
		ldev->caps.pix_fmt_hw = ltdc_pix_fmt_a1;
		ldev->caps.pix_fmt_drm = ltdc_drm_fmt_a1;
		ldev->caps.pix_fmt_nb = ARRAY_SIZE(ltdc_drm_fmt_a1);
		ldev->caps.pix_fmt_flex = false;
		ldev->caps.non_alpha_only_l1 = false;
		ldev->caps.pad_max_freq_hz = 150000000;
		ldev->caps.nb_irq = 4;
		ldev->caps.ycbcr_input = false;
		ldev->caps.ycbcr_output = false;
		ldev->caps.plane_reg_shadow = false;
		ldev->caps.crc = false;
		ldev->caps.dynamic_zorder = false;
		ldev->caps.plane_rotation = false;
		ldev->caps.crtc_rotation = false;
		ldev->caps.fifo_threshold = false;
		break;
	case HWVER_40100:
	case HWVER_40101:
		ldev->caps.layer_ofs = LAY_OFS_1;
		ldev->caps.layer_regs = ltdc_layer_regs_a2;
		ldev->caps.pix_fmt_hw = ltdc_pix_fmt_a2;
		ldev->caps.pix_fmt_drm = ltdc_drm_fmt_a2;
		ldev->caps.pix_fmt_nb = ARRAY_SIZE(ltdc_drm_fmt_a2);
		ldev->caps.pix_fmt_flex = true;
		ldev->caps.non_alpha_only_l1 = false;
		ldev->caps.nb_irq = 2;
		ldev->caps.ycbcr_input = true;
		ldev->caps.ycbcr_output = true;
		ldev->caps.plane_reg_shadow = true;
		ldev->caps.crc = true;
		ldev->caps.dynamic_zorder = true;
		ldev->caps.plane_rotation = true;
		/* check if the outuput rotaion is available */
		if (gc2r & GC2R_ROTA)
			ldev->caps.crtc_rotation = true;
		else
			ldev->caps.crtc_rotation = false;
		ldev->caps.fifo_threshold = true;

		for (int i = 0; i < lcr; i++) {
			/* read 1st register of layer's configuration */
			regmap_read(ldev->regmap, LTDC_L1C1R + i * LAY_OFS, &lxc1r);

			if (lxc1r & LXCR_C1R_SCA)
				ldev->caps.plane_scaling[i] = true;
			else
				ldev->caps.plane_scaling[i] = false;
		}
		break;
	default:
		return -ENODEV;
	}

	return 0;
}

void ltdc_suspend(struct ltdc_device *ldev)
{
	DRM_DEBUG_DRIVER("\n");

	clk_disable_unprepare(ldev->pixel_clk);
	if (ldev->bus_clk)
		clk_disable_unprepare(ldev->bus_clk);
}

int ltdc_resume(struct ltdc_device *ldev)
{
	int ret;

	DRM_DEBUG_DRIVER("\n");

	ret = clk_prepare_enable(ldev->pixel_clk);
	if (ret) {
		DRM_ERROR("failed to enable pixel clock (%d)\n", ret);
		return ret;
	}
	if (ldev->bus_clk) {
		if (clk_prepare_enable(ldev->bus_clk)) {
			DRM_ERROR("Unable to prepare bus clock\n");
			return -ENODEV;
		}
	}

	return 0;
}

int ltdc_load(struct drm_device *ddev)
{
	struct platform_device *pdev = to_platform_device(ddev->dev);
	struct ltdc_device *ldev = ddev->dev_private;
	struct device *dev = ddev->dev;
	struct device_node *np = dev->of_node;
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	struct drm_crtc *crtc;
	struct reset_control *rstc;
	int irq, i, nb_endpoints;
	int ret = -ENODEV;
	u32 mbl;
	bool def_value;

	DRM_DEBUG_DRIVER("\n");

	/* Get number of endpoints */
	nb_endpoints = of_graph_get_endpoint_count(np);
	if (!nb_endpoints)
		return -ENODEV;

	if (of_device_is_compatible(np, "st,stm32mp25-ltdc")) {
		/* Get max burst length */
		ret = of_property_read_u32(np, "st,burstlen", &mbl);
		if (ret)
			/* set to max burst length value */
			ldev->max_burst_length = 0;
		else
			ldev->max_burst_length = mbl / 8;
	}

	/* Get endpoints if any */
	for (i = 0; i < nb_endpoints; i++) {
		ret = drm_of_find_panel_or_bridge(np, 0, i, &panel, &bridge);

		/*
		 * If at least one endpoint is -ENODEV, continue probing,
		 * else if at least one endpoint returned an error
		 * (ie -EPROBE_DEFER) then stop probing.
		 */
		if (ret == -ENODEV)
			continue;
		else if (ret)
			return ret;

		if (panel) {
			bridge = drm_panel_bridge_add_typed(panel,
							    DRM_MODE_CONNECTOR_DPI);
			if (IS_ERR(bridge)) {
				DRM_ERROR("panel-bridge endpoint %d\n", i);
				ret = PTR_ERR(bridge);
				return ret;
			}
		}

		if (bridge) {
			ret = ltdc_encoder_init(ddev, bridge);
			if (ret) {
				if (ret != -EPROBE_DEFER)
					DRM_ERROR("init encoder endpoint %d\n", i);
				return ret;
			}
		}
	}

	rstc = devm_reset_control_get_exclusive(dev, NULL);

	mutex_init(&ldev->err_lock);

	def_value = device_property_read_bool(dev, "default-on");

	/*
	 * To obtain a continuous display after the probe, the clocks must
	 * remain activated and reset shouldn't be done
	 */
	if (!def_value) {
		if (!IS_ERR(rstc)) {
			reset_control_assert(rstc);
			usleep_range(10, 20);
			reset_control_deassert(rstc);
		}
	}

	ldev->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ldev->regs)) {
		DRM_ERROR("Unable to get ltdc registers\n");
		ret = PTR_ERR(ldev->regs);
		goto err;
	}

	/*
	 * Check the security of layer 2.
	 * If layer 2 is secure then its registers are not accessible
	 * (reduce mapping of ltdc registers to common registers and layers 0 and 1 registers).
	 */
	if (of_device_is_compatible(dev->of_node, "st,stm32mp25-ltdc")) {
		ret = stm32_rifsc_check_access_by_id(STM32MP25_RIFSC_LTDC_L2_ID);
		if (ret)
			stm32_ltdc_regmap_cfg.max_register = 0x300;
	}

	ldev->regmap = devm_regmap_init_mmio(&pdev->dev, ldev->regs, &stm32_ltdc_regmap_cfg);
	if (IS_ERR(ldev->regmap)) {
		DRM_ERROR("Unable to regmap ltdc registers\n");
		ret = PTR_ERR(ldev->regmap);
		goto err;
	}

	ret = ltdc_get_caps(ddev);
	if (ret) {
		DRM_ERROR("hardware identifier (0x%08x) not supported!\n",
			  ldev->caps.hw_version);
		goto err;
	}

	/* Disable all interrupts */
	regmap_clear_bits(ldev->regmap, LTDC_IER, IER_MASK);

	DRM_DEBUG_DRIVER("ltdc hw version 0x%08x\n", ldev->caps.hw_version);

	/* initialize default value for fifo underrun threshold & clear interrupt error counters */
	ldev->transfer_err = 0;
	ldev->fifo_err = 0;
	ldev->fifo_warn = 0;
	ldev->fifo_rot = 0;
	ldev->fifo_threshold = FUT_DFT;

	/* initialize default value for vblank & crc active flags */
	ldev->crc_active = false;
	ldev->vblank_active = false;

	for (i = 0; i < ldev->caps.nb_irq; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq < 0) {
			ret = irq;
			goto err;
		}

		ret = devm_request_threaded_irq(dev, irq, ltdc_irq,
						ltdc_irq_thread, IRQF_ONESHOT,
						dev_name(dev), ddev);
		if (ret) {
			DRM_ERROR("Failed to register LTDC interrupt\n");
			goto err;
		}
	}

	crtc = devm_kzalloc(dev, sizeof(*crtc), GFP_KERNEL);
	if (!crtc) {
		DRM_ERROR("Failed to allocate crtc\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = ltdc_crtc_init(ddev, crtc);
	if (ret) {
		DRM_ERROR("Failed to init crtc\n");
		goto err;
	}

	ret = drm_vblank_init(ddev, NB_CRTC);
	if (ret) {
		DRM_ERROR("Failed calling drm_vblank_init()\n");
		goto err;
	}

	pm_runtime_set_active(ddev->dev);
	pm_runtime_enable(ddev->dev);

	if (def_value) {
		/* keep runtime active after the probe */
		pm_runtime_get_sync(ddev->dev);
	} else {
		/* set to sleep state the pinctrl to stop data trasfert */
		pinctrl_pm_select_sleep_state(ddev->dev);
	}

	/* Get the secure rotation buffer memory resource */
	np = of_parse_phandle(dev->of_node, "rotation-memory", 0);
	if (np)
		ldev->rot_mem = of_reserved_mem_lookup(np);

	/* fail to get reserved memory for rotation */
	if (!ldev->rot_mem)
		ldev->caps.crtc_rotation = false;

	return 0;
err:
	of_reserved_mem_device_release(dev);

	for (i = 0; i < nb_endpoints; i++)
		drm_of_panel_bridge_remove(ddev->dev->of_node, 0, i);

	return ret;
}

void ltdc_unload(struct drm_device *ddev)
{
	struct device *dev = ddev->dev;
	int nb_endpoints, i;

	DRM_DEBUG_DRIVER("\n");

	nb_endpoints = of_graph_get_endpoint_count(dev->of_node);

	for (i = 0; i < nb_endpoints; i++)
		drm_of_panel_bridge_remove(ddev->dev->of_node, 0, i);

	pm_runtime_disable(ddev->dev);
}

int ltdc_parse_device_tree(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	int i, nb_endpoints;
	int ret = -ENODEV;

	DRM_DEBUG_DRIVER("\n");

	/* Get number of endpoints */
	nb_endpoints = of_graph_get_endpoint_count(np);
	if (!nb_endpoints)
		return -ENODEV;

	/* Get endpoints if any */
	for (i = 0; i < nb_endpoints; i++) {
		ret = drm_of_find_panel_or_bridge(np, 0, i, &panel, &bridge);

		/*
		 * If at least one endpoint is -ENODEV, continue probing,
		 * else if at least one endpoint returned an error
		 * (ie -EPROBE_DEFER) then stop probing.
		 */
		if (ret == -ENODEV)
			continue;
		else if (ret)
			return ret;
	}

	return 0;
}

int ltdc_get_clk(struct device *dev, struct ltdc_device *ldev)
{
	struct device_node *node;
	int ret;

	DRM_DEBUG_DRIVER("\n");

	ldev->pixel_clk = devm_clk_get(dev, "lcd");
	if (IS_ERR(ldev->pixel_clk)) {
		if (PTR_ERR(ldev->pixel_clk) != -EPROBE_DEFER)
			DRM_ERROR("Unable to get lcd clock\n");
		return PTR_ERR(ldev->pixel_clk);
	}

	if (of_device_is_compatible(dev->of_node, "st,stm32mp25-ltdc")) {
		ldev->bus_clk = devm_clk_get(dev, "bus");
		if (IS_ERR(ldev->bus_clk))
			return dev_err_probe(dev, PTR_ERR(ldev->bus_clk),
					     "Unable to get bus clock\n");

		ldev->ltdc_clk = devm_clk_get(dev, "ref");
		if (IS_ERR(ldev->ltdc_clk))
			return dev_err_probe(dev, PTR_ERR(ldev->ltdc_clk),
					     "Unable to get ltdc clock\n");

		/*
		 * The lvds output clock is not available if the lvds is not probed.
		 * This is a usual case, it is necessary to check the node to avoid
	 * looking for a clock that will never be available.
		 */
		node = of_find_compatible_node(NULL, NULL, "st,stm32mp25-lvds");
		if (!IS_ERR(node)) {
			if (of_device_is_available(node)) {
				ldev->lvds_clk = devm_clk_get(dev, "lvds");
				if (IS_ERR(ldev->lvds_clk)) {
					return dev_err_probe(dev, PTR_ERR(ldev->lvds_clk),
							     "Unable to get lvds clock\n");
				}
			}
			of_node_put(node);
		}

		/*
		 * Parent of the pixel clock should default to the reference clock (rcc clock).
		 * If the driver has already been started, this action is not necessary and
		 *  may cause an issue on register reading/writing.
		 */
		if (!device_property_read_bool(dev, "default-on")) {
			ret = clk_set_parent(ldev->pixel_clk, ldev->ltdc_clk);
			if (ret)
				return dev_err_probe(dev, PTR_ERR(ldev->lvds_clk),
						     "Could not set parent clock\n");
		}
	}

	return 0;
}

MODULE_AUTHOR("Philippe Cornu <philippe.cornu@st.com>");
MODULE_AUTHOR("Yannick Fertre <yannick.fertre@st.com>");
MODULE_AUTHOR("Fabien Dessenne <fabien.dessenne@st.com>");
MODULE_AUTHOR("Mickael Reulier <mickael.reulier@st.com>");
MODULE_DESCRIPTION("STMicroelectronics ST DRM LTDC driver");
MODULE_LICENSE("GPL v2");
