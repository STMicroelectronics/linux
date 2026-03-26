/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR MIT) */
/*
 * STM32 DCMIPP ISP userspace API
 * Copyright (C) STMicroelectronics SA 2023
 */

#ifndef _UAPI_STM32_DCMIPP_CONFIG_H
#define _UAPI_STM32_DCMIPP_CONFIG_H

#include <linux/types.h>

/* DCMIPP specific V4L2 control */
#define V4L2_CID_PIXELPROC_GAMMA_CORRECTION	(V4L2_CID_USER_BASE | 0x1001)

/* Bad Pixel Removal */
#define STM32_DCMIPP_ISP_BPR		(1U << 0)
/* Black Level Correction */
#define STM32_DCMIPP_ISP_BLC		(1U << 1)
/* Exposure Control */
#define STM32_DCMIPP_ISP_EX		(1U << 2)
/* Demosaicing filters */
#define STM32_DCMIPP_ISP_DM		(1U << 3)
/* Color conversion Control */
#define STM32_DCMIPP_ISP_CC		(1U << 4)
/* Contrast Enhancement */
#define STM32_DCMIPP_ISP_CE		(1U << 5)
/* Histogram Extraction */
#define STM32_DCMIPP_ISP_HISTO		(1U << 6)

/**
 * struct stm32_dcmipp_isp_bpr_cfg - STM32 DCMIPP ISP bad pixel removal
 *
 * @en: enable / disable the bad pixel removal block
 * @strength: strength (aggressiveness) of the bad pixel detection
 */
struct stm32_dcmipp_isp_bpr_cfg {
	__u32 en;
	__u32 strength;
};

/**
 * struct stm32_dcmipp_isp_blc_cfg - STM32 DCMIPP ISP black level correction
 *
 * @en: enable / disable the black level correction block
 * @blc_r: Correction on the red component
 * @blc_g: Correction on the green component
 * @blc_b: Correction on the blue component
 */
struct stm32_dcmipp_isp_blc_cfg {
	__u32 en;
	__u8 blc_r;
	__u8 blc_g;
	__u8 blc_b;
};

/**
 * struct stm32_dcmipp_isp_ex_cfg - STM32 DCMIPP ISP exposure control
 *
 * @en: enable / disable the exposure control block
 * @shift_r: red component exposure shift
 * @mult_r: red component exposure multiplier
 * @shift_g: green component exposure shift
 * @mult_g: green component exposure multiplier
 * @shift_b: blue component exposure shift
 * @mult_b: blue component exposure multiplier
 */
struct stm32_dcmipp_isp_ex_cfg {
	__u32 en;
	__u8 shift_r;
	__u8 mult_r;
	__u8 shift_g;
	__u8 mult_g;
	__u8 shift_b;
	__u8 mult_b;
};

/**
 * struct stm32_dcmipp_isp_dm_cfg - STM32 DCMIPP ISP demosaicing filters
 *
 * @en: enable / disable the demosaicing block
 * @edge: strength of the edge detection
 * @lineh: strength of the horizontal line detection
 * @linev: strength of the vertical line detection
 * @peak: strength of the peak detection
 */
struct stm32_dcmipp_isp_dm_cfg {
	__u32 en;
	__u8 edge;
	__u8 lineh;
	__u8 linev;
	__u8 peak;
};

enum stm32_dcmipp_isp_cc_clamp {
	STM32_DCMIPP_ISP_CC_CLAMP_DISABLED,
	STM32_DCMIPP_ISP_CC_CLAMP_Y235_U240_V240,
	STM32_DCMIPP_ISP_CC_CLAMP_YUV235,
};

/**
 * struct stm32_dcmipp_isp_cc_cfg - STM32 DCMIPP ISP color conversion
 *
 * @en: enable / disable the color conversion block
 * @clamp: clamp configuration (from enum stm32_dcmipp_isp_cc_clamp)
 * @rr: row 1 col 1 value of the matrix
 * @rg: row 1 col 2 value of the matrix
 * @rb: row 1 col 3 value of the matrix
 * @ra: row 1 added value of the matrix
 * @gr: row 2 col 1 value of the matrix
 * @gg: row 2 col 2 value of the matrix
 * @gb: row 2 col 3 value of the matrix
 * @ga: row 2 added value of the matrix
 * @br: row 3 col 1 value of the matrix
 * @bg: row 3 col 2 value of the matrix
 * @bb: row 3 col 3 value of the matrix
 * @ba: row 3 added value of the matrix
 */
struct stm32_dcmipp_isp_cc_cfg {
	__u32 en;
	__u32 clamp;
	__u16 rr;
	__u16 rg;
	__u16 rb;
	__u16 ra;
	__u16 gr;
	__u16 gg;
	__u16 gb;
	__u16 ga;
	__u16 br;
	__u16 bg;
	__u16 bb;
	__u16 ba;
};

/**
 * struct stm32_dcmipp_isp_ce_cfg - STM32 DCMIPP ISP contrast enhancement
 *
 * @en: enable / disable the contrast enhancement block
 * @lum: 9 elements table of luminance enhancement (value 16 is neutral)
 */
struct stm32_dcmipp_isp_ce_cfg {
	__u32 en;
	__u8 lum[9];
};

/**
 * struct stm32_dcmipp_isp_histo_cfg - STM32 DCMIPP ISP histogram output
 *
 * @left: left start (in pixel) of histogram area
 * @top: top start (in pixel) of histogram area
 * @width: width (in pixel) of one region
 * @height: height (in pixel) of one region
 * @bin: bin amount per histogram
 * @dyn: dynamic of input components
 * @comp: component to be extracted
 * @vdec: vertical decimation of input before histogram extraction
 * @hdec: horizontal decimation of input before histogram extraction
 * @hreg: amount of horizontal regions
 * @vreg: amount of vertical regions
 * @src: stage within ISP for extraction of histogram
 */
#define STM32_DCMIPP_ISP_HISTO_BIN_4	0
#define STM32_DCMIPP_ISP_HISTO_BIN_16	1
#define STM32_DCMIPP_ISP_HISTO_BIN_64	2
#define STM32_DCMIPP_ISP_HISTO_BIN_256	3

#define STM32_DCMIPP_ISP_HISTO_DYN_LIGHT	0
#define STM32_DCMIPP_ISP_HISTO_DYN_SEMI_LIGHT	1
#define STM32_DCMIPP_ISP_HISTO_DYN_SEMI_DARK	2
#define STM32_DCMIPP_ISP_HISTO_DYN_DARK		3

#define STM32_DCMIPP_ISP_HISTO_COMP_R_R_V_128	0
#define STM32_DCMIPP_ISP_HISTO_COMP_GR_G_Y_Y	1
#define STM32_DCMIPP_ISP_HISTO_COMP_B_B_U_128	2
#define STM32_DCMIPP_ISP_HISTO_COMP_GB_L_L_L	3
#define STM32_DCMIPP_ISP_HISTO_COMP_ALL		4

#define STM32_DCMIPP_ISP_HISTO_VHDEC_NONE	0
#define STM32_DCMIPP_ISP_HISTO_VHDEC_2		1
#define STM32_DCMIPP_ISP_HISTO_VHDEC_4		2
#define STM32_DCMIPP_ISP_HISTO_VHDEC_8		3
#define STM32_DCMIPP_ISP_HISTO_VHDEC_16		4

#define STM32_DCMIPP_ISP_HISTO_MAX_VHREG	16

#define STM32_DCMIPP_ISP_HISTO_SRC_POST_DEC	0
#define STM32_DCMIPP_ISP_HISTO_SRC_POST_BLC	1
#define STM32_DCMIPP_ISP_HISTO_SRC_POST_EX	2
#define STM32_DCMIPP_ISP_HISTO_SRC_POST_DM	3
#define STM32_DCMIPP_ISP_HISTO_SRC_POST_CC	4
#define STM32_DCMIPP_ISP_HISTO_SRC_POST_CE	5

struct stm32_dcmipp_isp_histo_cfg {
	__u16 left;
	__u16 top;
	__u16 width;
	__u16 height;
	__u8 bin;
	__u8 dyn;
	__u8 comp;
	__u8 vdec;
	__u8 hdec;
	__u8 vreg;
	__u8 hreg;
	__u8 src;
};

/**
 * struct stm32_dcmipp_isp_ctrls_cfg - STM32 DCMIPP ISP Controls
 *
 * @bpr_cfg: configuration of the bad pixel removal block
 * @blc_cfg: configuration of the black level correction block
 * @ex_cfg: configuration of the exposure block
 * @dm_cfg: configuration of the demosaicing filters block
 * @cc_cfg: configuration of the color conversion block
 * @ce_cfg: configuration of the contrast enhancement block
 * @histo_cfg: configuration of the histogram block
 */
struct stm32_dcmipp_isp_ctrls_cfg {
	struct stm32_dcmipp_isp_bpr_cfg bpr_cfg;
	struct stm32_dcmipp_isp_blc_cfg blc_cfg;
	struct stm32_dcmipp_isp_ex_cfg ex_cfg;
	struct stm32_dcmipp_isp_dm_cfg dm_cfg;
	struct stm32_dcmipp_isp_cc_cfg cc_cfg;
	struct stm32_dcmipp_isp_ce_cfg ce_cfg;
	struct stm32_dcmipp_isp_histo_cfg histo_cfg;
};

/**
 * struct stm32_dcmipp_params_cfg - STM32 DCMIPP ISP Input Parameters Meta Data
 *
 * @module_cfg_update: mask the config bits of which module should be updated
 * @ctrls: configuration of other ISP blocks
 */
struct stm32_dcmipp_params_cfg {
	__u32 module_cfg_update;

	struct stm32_dcmipp_isp_ctrls_cfg ctrls;
};

/**
 * struct stm32_dcmipp_stat_avr_bins - average & bins statistics
 *
 * @average_rgb[3]: average value of R/G/B components
 * @bins[12]: 12 values histogram
 */
struct stm32_dcmipp_stat_avr_bins {
	__u32 average_RGB[3];
	__u32 bins[12];
};

/**
 * struct stm32_dcmipp_stat_buf - statistics buffer
 *
 * @pre: average & bins statistics at pre-demosaicing location
 * @post: average & bins statistics at post-demosaicing location
 * @bad_pixel_count: number of bad pixels detected in the frame
 */
#define STM32_DCMIPP_HISTO_BIN_MAX	(320 * 16)
struct stm32_dcmipp_stat_buf {
	struct stm32_dcmipp_stat_avr_bins pre;
	struct stm32_dcmipp_stat_avr_bins post;
	__u32 bad_pixel_count;
	__u32 reserved;
	__u16 histograms[STM32_DCMIPP_HISTO_BIN_MAX];
};

#endif
