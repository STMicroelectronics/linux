/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * This file is part of STM32 MDF driver
 *
 * Copyright (C) 2023, STMicroelectronics - All Rights Reserved
 * Author: Olivier Moysan <olivier.moysan@foss.st.com>.
 */

#include <linux/bitfield.h>

/* Register definitions */
#define MDF_GCR_REG 0x00
#define MDF_CKGCR_REG 0x04
#define MDF_HWCFGR_REG 0xff0
#define MDF_VERR_REG 0xff4
#define MDF_IPIDR_REG 0xff8
#define MDF_SIDR_REG 0xffc

/* Registers offsets relative to sitf base address */
#define MDF_SITFCR_REG 0x00 /* 0x80 * x + offset */

/* Registers offsets relative to filter base address */
#define MDF_BSMXCR_REG 0x00 /* 0x84 * x + offset */
#define MDF_DFLTCR_REG 0x04
#define MDF_DFLTCICR_REG 0x08
#define MDF_DFLTRSFR_REG 0x0c
#define MDF_DFLTINTR_REG 0x10
#define MDF_OLDCR_REG 0x14
#define MDF_OLDTHLR_REG 0x18
#define MDF_OLDTHHR_REG 0x1c
#define MDF_DLYCR_REG 0x20
#define MDF_SCDCR_REG 0x24
#define MDF_DFLTIER_REG 0x28
#define MDF_DFLTISR_REG 0x2C
#define MDF_OECCR_REG 0x30
#define MDF_SNPSDR 0x64
#define MDF_DFLTDR_REG 0x6c

/* MDF_GCR: Global Control Register */
#define MDF_GCR_TRGO BIT(0)
#define MDF_GCR_ILVNB_MASK GENMASK(7, 4)
#define MDF_GCR_ILVNB(v) FIELD_PREP(MDF_GCR_ILVNB_MASK, v)

/* MDF_CKGCR: Clock Generator Control Register */
#define MDF_CKG_CKGDEN BIT(0)
#define MDF_CKG_CCK0EN BIT(1)
#define MDF_CKG_CCK1EN BIT(2)
#define MDF_CKG_CKGMOD BIT(4)
#define MDF_CKG_CCK0DIR BIT(5)
#define MDF_CKG_CCK1DIR BIT(6)
#define MDF_CKG_TRGSENS BIT(8)
#define MDF_CKG_TRGSRC_MASK GENMASK(15, 12)
#define MDF_CKG_TRGSRC(v) FIELD_PREP(MDF_CKG_TRGSRC_MASK, v)
#define MDF_CKG_CCKDIV_MASK GENMASK(19, 16)
#define MDF_CKG_CCKDIV(v) FIELD_PREP(MDF_CKG_CCKDIV_MASK, v)
#define MDF_CKG_CCKDIV_MAX (FIELD_MAX(MDF_CKG_CCKDIV_MASK) + 1)
#define MDF_CKG_PROCDIV_MASK GENMASK(30, 24)
#define MDF_CKG_PROCDIV(v) FIELD_PREP(MDF_CKG_PROCDIV_MASK, v)
#define MDF_PROCDIV_MAX (FIELD_MAX(MDF_CKG_PROCDIV_MASK) + 1)
#define MDF_CKG_ACTIVE BIT(31)

/* MDF_OR: Option Register */
#define MDF_OR_OPTION_MASK OPTION(31, 0)
#define MDF_OR_OPTION(v) FIELD_PREP(MDF_OR_OPTION_MASK, v)

/* MDF_SITFCR: Serial Interface Control Register */
#define MDF_SITFCR_SITFEN BIT(0)
#define MDF_SITFCR_SCKSRC_MASK GENMASK(2, 1)
#define MDF_SITFCR_SCKSRC(v) FIELD_PREP(MDF_SITFCR_SCKSRC_MASK, v)
#define MDF_SITFCR_SITFMOD_MASK GENMASK(5, 4)
#define MDF_SITFCR_SITFMOD(v) FIELD_PREP(MDF_SITFCR_SITFMOD_MASK, v)
#define MDF_SITFCR_STH_MASK GENMASK(12, 8)
#define MDF_SITFCR_STH(v) FIELD_PREP(MDF_SITFCR_STH_MASK, v)
#define MDF_SITFCR_ACTIVE BIT(31)

/* MDF_BSMXCR: Bitstream Matrix Control Register */
#define MDF_BSMXCR_BSSEL_MASK GENMASK(4, 0)
#define MDF_BSMXCR_BSSEL(v) FIELD_PREP(MDF_BSMXCR_BSSEL_MASK, v)
#define MDF_BSMXCR_ACTIVE BIT(31)

/* MDF_DFLTCR: Digital Filter Control Register */
#define MDF_DFLTCR_DFLTEN BIT(0)
#define MDF_DFLTCR_DMAEN BIT(1)
#define MDF_DFLTCR_FTH BIT(2)
#define MDF_DFLTCR_ACQMOD_MASK GENMASK(6, 4)
#define MDF_DFLTCR_ACQMOD(v) FIELD_PREP(MDF_DFLTCR_ACQMOD_MASK, v)
#define MDF_DFLTCR_TRGSENS BIT(8)
#define MDF_DFLTCR_TRGSENS_SET(v) FIELD_PREP(MDF_DFLTCR_TRGSENS, v)
#define MDF_DFLTCR_TRGSRC_MASK GENMASK(15, 12)
#define MDF_DFLTCR_TRGSRC(v) FIELD_PREP(MDF_DFLTCR_TRGSRC_MASK, v)
#define MDF_DFLTCR_SNPSFMT BIT(16)
#define MDF_DFLTCR_NBDIS_MASK GENMASK(27, 20)
#define MDF_DFLTCR_NBDIS(v) FIELD_PREP(MDF_DFLTCR_NBDIS_MASK, v)
#define MDF_DFLTCR_NBDIS_MAX FIELD_MAX(MDF_DFLTCR_NBDIS_MASK)
#define MDF_DFLTCR_DFLTRUN BIT(30)
#define MDF_DFLTCR_ACTIVE BIT(31)

/* MDF_DFLTCICR: Digital Filter Configuration Register */
#define MDF_DFLTCICR_DATSRC_MASK GENMASK(1, 0)
#define MDF_DFLTCICR_DATSRC(v) FIELD_PREP(MDF_DFLTCICR_DATSRC_MASK, v)
#define MDF_DFLTCICR_CICMOD_MASK GENMASK(6, 4)
#define MDF_DFLTCICR_CICMOD(v) FIELD_PREP(MDF_DFLTCICR_CICMOD_MASK, v)
#define MDF_DFLTCICR_MCICD_MASK GENMASK(16, 8)
#define MDF_DFLTCICR_MCICD(v) FIELD_PREP(MDF_DFLTCICR_MCICD_MASK, v)
#define MDF_DFLTCICR_MCICD_MIN 2
#define MDF_DFLTCICR_SCALE_MASK GENMASK(25, 20)
#define MDF_DFLTCICR_SCALE(v) FIELD_PREP(MDF_DFLTCICR_SCALE_MASK, v)

/* MDF_DFLTRSFR: Reshape Filter Configuration Register */
#define MDF_DFLTRSFR_RSFLTBYP BIT(0)
#define MDF_DFLTRSFR_RSFLTD BIT(4)
#define MDF_DFLTRSFR_HPFBYP BIT(7)
#define MDF_DFLTRSFR_HPFC_MASK GENMASK(9, 8)
#define MDF_DFLTRSFR_HPFC(v) FIELD_PREP(MDF_DFLTRSFR_HPFC_MASK, v)

/* MDF_DFLTINTR: Integrator Configuration Register */
#define MDF_DFLTINTR_INTDIV_MASK GENMASK(1, 0)
#define MDF_DFLTINTR_INTDIV(v) FIELD_PREP(MDF_DFLTINTR_INTDIV_MASK, v)
#define MDF_DFLTINTR_INTVAL_MASK GENMASK(10, 4)
#define MDF_DFLTINTR_INTVAL(v) FIELD_PREP(MDF_DFLTINTR_INTVAL_MASK, v)

/* MDF_OLDCR: Out-Of Limit Detector Control Register */
#define MDF_OLDCR_OLDEN_MASK BIT(0)
#define MDF_OLDCR_OLDEN(v) FIELD_PREP(MDF_OLDCR_OLDEN_MASK, v)
#define MDF_OLDCR_THINB_MASK BIT(1)
#define MDF_OLDCR_THINB(v) FIELD_PREP(MDF_OLDCR_THINB_MASK, v)
#define MDF_OLDCR_BKOLD_MASK GENMASK(7, 4)
#define MDF_OLDCR_BKOLD(v) FIELD_PREP(MDF_OLDCR_BKOLD_MASK, v)
#define MDF_OLDCR_ACICN_MASK GENMASK(13, 12)
#define MDF_OLDCR_ACICN(v) FIELD_PREP(MDF_OLDCR_ACICN_MASK, v)
#define MDF_OLDCR_ACICD_MASK GENMASK(21, 17)
#define MDF_OLDCR_ACICD(v) FIELD_PREP(MDF_OLDCR_ACICD_MASK, v)
#define MDF_OLDCR_ACTIVE_MASK BIT(31)
#define MDF_OLDCR_ACTIVE(v) FIELD_PREP(MDF_OLDCR_ACTIVE_MASK, v)

/* MDF_OLDxTHLR: OLD Threshold Low Register */
#define MDF_OLDTHLR_OLDTHL_MASK GENMASK(25, 0)
#define MDF_OLDTHLR_OLDTHL(v) FIELD_PREP(MDF_OLDTHLR_OLDTHL_MASK, v)

/* MDF_OLDxTHHR: OLD Threshold High Register */
#define MDF_OLDTHHR_OLDTHH_MASK GENMASK(25, 0)
#define MDF_OLDTHHR_OLDTHH(v) FIELD_PREP(MDF_OLDTHHR_OLDTHH_MASK, v)

/* MDF_DLYCR: Delay control Register */
#define MDF_DLYCR_SKPDLY_MASK GENMASK(6, 0)
#define MDF_DLYCR_SKPDLY(v) FIELD_PREP(MDF_DLYCR_SKPDLY_MASK, v)

/* MDF_SCDCR: Short circuit detector control Register */
#define MDF_SCDCR_CDEN_MASK BIT(0)
#define MDF_SCDCR_CDEN(v) FIELD_PREP(MDF_SCDCR_CDEN_MASK, v)
#define MDF_SCDCR_BKSCD_MASK GENMASK(7, 4)
#define MDF_SCDCR_BKSCD(v) FIELD_PREP(MDF_SCDCR_BKSCD_MASK, v)
#define MDF_SCDCR_SCDT_MASK GENMASK(19, 12)
#define MDF_SCDCR_SCDT(v) FIELD_PREP(MDF_SCDCR_SCDT_MASK, v)
#define MDF_SCDCR_ACTIVE_MASK BIT(31)
#define MDF_SCDCR_ACTIVE(v) FIELD_PREP(MDF_SCDCR_ACTIVE_MASK, v)

/* MDF_DFLTIER: DFLT Interrupt enable Register */
#define MDF_DFLTIER_FTHIE_MASK BIT(0)
#define MDF_DFLTIER_FTHIE(v) FIELD_PREP(MDF_DFLTIER_FTHIE_MASK, v)
#define MDF_DFLTIER_DOVRIE_MASK BIT(1)
#define MDF_DFLTIER_DOVRIE(v) FIELD_PREP(MDF_DFLTIER_DOVRIE_MASK, v)
#define MDF_DFLTIER_SSDRIE_MASK BIT(2)
#define MDF_DFLTIER_SSDRIE(v) FIELD_PREP(MDF_DFLTIER_SSDRIE_MASK, v)
#define MDF_DFLTIER_OLDIE_MASK BIT(4)
#define MDF_DFLTIER_OLDIE(v) FIELD_PREP(MDF_DFLTIER_OLDIE_MASK, v)
#define MDF_DFLTIER_SSOVRIE_MASK BIT(7)
#define MDF_DFLTIER_SSOVRIE(v) FIELD_PREP(MDF_DFLTIER_SSOVRIE_MASK, v)
#define MDF_DFLTIER_SCDIE_MASK BIT(8)
#define MDF_DFLTIER_SCDIE(v) FIELD_PREP(MDF_DFLTIER_SCDIE_MASK, v)
#define MDF_DFLTIER_SATIE_MASK BIT(9)
#define MDF_DFLTIER_SATIE(v) FIELD_PREP(MDF_DFLTIER_SATIE_MASK, v)
#define MDF_DFLTIER_CKABIE_MASK BIT(10)
#define MDF_DFLTIER_CKABIE(v) FIELD_PREP(MDF_DFLTIER_CKABIE_MASK, v)
#define MDF_DFLTIER_RFOVRIE_MASK BIT(11)
#define MDF_DFLTIER_RFOVRIE(v) FIELD_PREP(MDF_DFLTIER_RFOVRIE_MASK, v)

/* MDF_DFLTISR: DFLT Interrupt status Register */
#define MDF_DFLTISR_FTHF_MASK BIT(0)
#define MDF_DFLTISR_FTHF(v) FIELD_PREP(MDF_DFLTISR_FTHF_MASK, v)
#define MDF_DFLTISR_DOVRF_MASK BIT(1)
#define MDF_DFLTISR_DOVRF(v) FIELD_PREP(MDF_DFLTISR_DOVRF_MASK, v)
#define MDF_DFLTISR_SSDRF_MASK BIT(2)
#define MDF_DFLTISR_SSDRF(v) FIELD_PREP(MDF_DFLTISR_SSDRF_MASK, v)
#define MDF_DFLTISR_OLDF_MASK BIT(4)
#define MDF_DFLTISR_OLDF(v) FIELD_PREP(MDF_DFLTISR_OLDF_MASK, v)
#define MDF_DFLTISR_SSOVRF_MASK BIT(7)
#define MDF_DFLTISR_SSOVRF(v) FIELD_PREP(MDF_DFLTISR_SSOVRF_MASK, v)
#define MDF_DFLTISR_SCDF_MASK BIT(8)
#define MDF_DFLTISR_SCDF(v) FIELD_PREP(MDF_DFLTISR_SCDF_MASK, v)
#define MDF_DFLTISR_SATF_MASK BIT(9)
#define MDF_DFLTISR_SATF(v) FIELD_PREP(MDF_DFLTISR_SATF_MASK, v)
#define MDF_DFLTISR_CKABF_MASK BIT(10)
#define MDF_DFLTISR_CKABF(v) FIELD_PREP(MDF_DFLTISR_CKABF_MASK, v)
#define MDF_DFLTISR_RFOVRF_MASK BIT(11)
#define MDF_DFLTISR_RFOVRF(v) FIELD_PREP(MDF_DFLTISR_RFOVRF_MASK, v)

/* MDF_OECCR: Offset Error Compensation Control Register */
#define MDF_OECCR_OFFSET_MASK GENMASK(25, 0)
#define MDF_OECCR_OFFSET(v) FIELD_PREP(MDF_OECCR_OFFSET_MASK, v)

/* MDF_SNPSDR:  Snapshot Data Register */
#define MDF_SNPSDR_MCICDC_MASK GENMASK(8, 0)
#define MDF_SNPSDR_MCICDC(v) FIELD_PREP(MDF_SNPSDR_MCICDC_MASK, v)
#define MDF_SNPSDR_EXTSDR_MASK GENMASK(15, 9)
#define MDF_SNPSDR_EXTSDR(v) FIELD_PREP(MDF_SNPSDR_EXTSDR_MASK, v)
#define MDF_SNPSDR_SDR_MASK GENMASK(31, 16)
#define MDF_SNPSDR_SDR(v) FIELD_PREP(MDF_SNPSDR_SDR_MASK, v)

/* MDF_DFLTDR: Digital Filter Data Register */
#define MDF_DFLTDR_DR_MASK GENMASK(31, 8)
#define MDF_DFLTDR_DR(v) FIELD_PREP(MDF_DFLTDR_DR_MASK, v)

/* MDF_HWCFGR: Hardware configuration register */
#define MDF_HWCFGR_FIFO_SIZE_MASK GENMASK(7, 0)
#define MDF_HWCFGR_FIFO_SIZE(v) FIELD_PREP(MDF_HWCFGR_FIFO_SIZE_MASK, v)
#define MDF_HWCFGR_NBF_MASK GENMASK(15, 8)
#define MDF_HWCFGR_NBF(v) FIELD_PREP(MDF_HWCFGR_NBF_MASK, v)
#define MDF_HWCFGR_FLT_CFG_MASK GENMASK(19, 16)
#define MDF_HWCFGR_FLT_CFG(v) FIELD_PREP(MDF_HWCFGR_FLT_CFG_MASK, v)
#define MDF_HWCFGR_SAD_MASK GENMASK(23, 20)
#define MDF_HWCFGR_SAD(v) FIELD_PREP(MDF_HWCFGR_SAD_MASK, v)
#define MDF_HWCFGR_OPTION_MASK GENMASK(31, 24)
#define MDF_HWCFGR_OPTION(v) FIELD_PREP(MDF_HWCFGR_OPTION_MASK, v)

/* MDF_VERR: Version Register */
#define MDF_VERR_MINREV_MASK GENMASK(3, 0)
#define MDF_VERR_MINREV(v) FIELD_PREP(MDF_VERR_MINREV_MASK, v)
#define MDF_VERR_MAJREV_MASK GENMASK(7, 4)
#define MDF_VERR_MAJREV(v) FIELD_PREP(MDF_VERR_MAJREV_MASK, v)

/* MDF_IPIDR: Identification Register */
#define MDF_IPIDR_ID_MASK GENMASK(31, 0)
#define MDF_IPIDR_ID_MINREV(v) FIELD_PREP(MDF_IPIDR_ID_MASK, v)

/* MDF_SIDR: Size Identification Register */
#define MDF_SIDR_SID_MASK GENMASK(31, 0)
#define MDF_SIDR_SID(v) FIELD_PREP(MDF_SIDR_SID_MASK, v)

#define STM32MP25_MDF_IPIDR_NUMBER 0x00110032

#define STM32_MDF_CCK0 "clk_cck0"
#define STM32_MDF_CCK1 "clk_cck1"

enum stm32_sitf_mode {
	STM32_MDF_MODE_LF_SPI,
	STM32_MDF_MODE_SPI,
	STM32_MDF_MODE_MANCHESTER_R,
	STM32_MDF_MODE_MANCHESTER_F,
	STM32_MDF_MODE_NB,
};

/*
 * struct stm32_mdf_sitf - STM32 MDF serial interface data
 * @entry: serial interface list head
 * @dev: pointer to parent device
 * @regmap: regmap for register read/write
 * @mdf: mdf common data pointer
 * @node: serial interface node handle
 * @sck: sitf clock handle
 * @base: sitf registers base cpu addr
 * @lock: sitf state manegement lock
 * @refcnt: sitf usage reference counter
 * @scksrc: sitf clock source
 * @id: serial interface index
 * @mode: sitf mode
 * @sitfcr: sitf configuration register backup
 */
struct stm32_mdf_sitf {
	struct list_head entry;
	struct device *dev;
	struct regmap *regmap;
	struct stm32_mdf *mdf;
	struct fwnode_handle *node;
	struct clk *sck;
	void __iomem *base;
	spinlock_t lock; /* Manage race conditions on sitf state */
	unsigned int refcnt;
	unsigned int scksrc;
	u32 id;
	u32 mode;
	u32 sitfcr;
};

/*
 * struct stm32_mdf - STM32 MDF driver common data (for all instances)
 * @sitf_list: pointer to serial interfaces list
 * @filter_list: pointer to filter interfaces list
 * @fh_interleave: interleaved filter handle list pointer
 * @fproc: processing clock frequency
 * @nbf: total number of digital filters
 * @nb_interleave: number of interleaved filters
 */
struct stm32_mdf {
	struct list_head sitf_list;
	struct list_head filter_list;
	struct fwnode_handle **fh_interleave;
	unsigned long fproc;
	unsigned int nbf;
	unsigned int nb_interleave;
};

int stm32_mdf_core_start_mdf(struct stm32_mdf *mdf);
int stm32_mdf_core_stop_mdf(struct stm32_mdf *mdf);
int stm32_mdf_core_trigger(struct stm32_mdf *mdf);
int stm32_mdf_core_restore_cck(struct stm32_mdf *mdf);
int stm32_mdf_core_lock_kclk_rate(struct stm32_mdf *mdf);
void stm32_mdf_core_unlock_kclk_rate(struct stm32_mdf *mdf);
unsigned long stm32_mdf_core_get_cck(struct stm32_mdf *mdf);

int stm32_mdf_sitf_start(struct stm32_mdf_sitf *sitf);
int stm32_mdf_sitf_stop(struct stm32_mdf_sitf *sitf);
