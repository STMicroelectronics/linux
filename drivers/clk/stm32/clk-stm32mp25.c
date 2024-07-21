// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) STMicroelectronics 2023 - All Rights Reserved
 * Author: Gabriel Fernandez <gabriel.fernandez@foss.st.com> for STMicroelectronics.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "clk-stm32-core.h"

#include <dt-bindings/bus/stm32mp25_sys_bus.h>
#include <dt-bindings/clock/stm32mp25-clks.h>

#include "stm32mp25_rcc.h"

/* Temporary */
#include "../../bus/stm32_sys_bus.h"

/* Clock security definition */
#define SECF_NONE		-1

#define RCC_REG_SIZE	32
#define RCC_SECCFGR(x)	(((x) / RCC_REG_SIZE) * 0x4 + RCC_SECCFGR0)
#define RCC_CIDCFGR(x)	((x) * 0x8 + RCC_R0CIDCFGR)
#define RCC_SEMCR(x)	((x) * 0x8 + RCC_R0SEMCR)
#define RCC_CID1	1

/* Register: RIFSC_CIDCFGR */
#define RCC_CIDCFGR_CFEN	BIT(0)
#define RCC_CIDCFGR_SEM_EN	BIT(1)
#define RCC_CIDCFGR_SEMWLC1_EN	BIT(17)
#define RCC_CIDCFGR_SCID_MASK	GENMASK(6, 4)

/* Register: RIFSC_SEMCR */
#define RCC_SEMCR_SEMCID_MASK	GENMASK(6, 4)

#define STM32MP25_RIFRCC_DBG_ID		73
#define STM32MP25_RIFRCC_IS2M_ID	107
#define STM32MP25_RIFRCC_MCO1_ID	108
#define STM32MP25_RIFRCC_MCO2_ID	109
#define STM32MP25_RIFRCC_OSPI1_ID	110
#define STM32MP25_RIFRCC_OSPI2_ID	111

#define SEC_RIFSC_FLAG		BIT(31)
#define SEC_RIFRCC(_id)		(STM32MP25_RIFRCC_##_id##_ID)
#define SEC_RIFSC(_name)	(STM32MP25_RIFSC_##_name##_ID | SEC_RIFSC_FLAG)

static const char * const adc12_src[] = {
	"ck_flexgen_46", "ck_icn_ls_mcu"
};

static const char * const adc3_src[] = {
	"ck_flexgen_47", "ck_icn_ls_mcu", "ck_flexgen_46"
};

static const char * const usb2phy1_src[] = {
	"ck_flexgen_57", "hse_div2_ck"
};

static const char * const usb2phy2_src[] = {
	"ck_flexgen_58", "hse_div2_ck"
};

static const char * const usb3pciphy_src[] = {
	"ck_flexgen_34", "hse_div2_ck"
};

static const char * const dsiblane_src[] = {
	"txbyteclk", "ck_ker_ltdc"
};

static const char * const dsiphy_src[] = {
	"ck_flexgen_28", "hse_ck"
};

static const char * const lvdsphy_src[] = {
	"ck_flexgen_32", "hse_ck"
};

static const char * const dts_src[] = {
	"hsi_ck", "hse_ck", "msi_ck"
};

static const char * const mco1_src[] = {
	"ck_flexgen_61", "ck_obs0"
};

static const char * const mco2_src[] = {
	"ck_flexgen_62", "ck_obs1"
};

enum enum_mux_cfg {
	MUX_MCO1,
	MUX_MCO2,
	MUX_ADC12,
	MUX_ADC3,
	MUX_USB2PHY1,
	MUX_USB2PHY2,
	MUX_USB3PCIEPHY,
	MUX_DSIBLANE,
	MUX_DSIPHY,
	MUX_LVDSPHY,
	MUX_DTS,

#ifdef CONFIG_DEBUG_FS
	MUX_MUXSEL0,
	MUX_MUXSEL1,
	MUX_MUXSEL2,
	MUX_MUXSEL3,
	MUX_MUXSEL4,
	MUX_MUXSEL5,
	MUX_MUXSEL6,
	MUX_MUXSEL7,
	MUX_XBARSEL,
	MUX_RTC,
	MUX_CPU1,
	MUX_D3PER,
#endif
	MUX_NB
};

#define MUX_CFG(id, _offset, _shift, _witdh)\
	[id] = {\
		.offset		= (_offset),\
		.shift		= (_shift),\
		.width		= (_witdh),\
	}

static const struct stm32_mux_cfg stm32mp25_muxes[MUX_NB] = {
	MUX_CFG(MUX_MCO1,		RCC_MCO1CFGR,		0,	1),
	MUX_CFG(MUX_MCO2,		RCC_MCO2CFGR,		0,	1),
	MUX_CFG(MUX_ADC12,		RCC_ADC12CFGR,		12,	1),
	MUX_CFG(MUX_ADC3,		RCC_ADC3CFGR,		12,	2),
	MUX_CFG(MUX_USB2PHY1,		RCC_USB2PHY1CFGR,	15,	1),
	MUX_CFG(MUX_USB2PHY2,		RCC_USB2PHY2CFGR,	15,	1),
	MUX_CFG(MUX_USB3PCIEPHY,	RCC_USB3PCIEPHYCFGR,	15,	1),
	MUX_CFG(MUX_DSIBLANE,		RCC_DSICFGR,		12,	1),
	MUX_CFG(MUX_DSIPHY,		RCC_DSICFGR,		15,	1),
	MUX_CFG(MUX_LVDSPHY,		RCC_LVDSCFGR,		15,	1),
	MUX_CFG(MUX_DTS,		RCC_DTSCFGR,		12,	2),

#ifdef CONFIG_DEBUG_FS
	MUX_CFG(MUX_MUXSEL0,		RCC_MUXSELCFGR,		0,	2),
	MUX_CFG(MUX_MUXSEL1,		RCC_MUXSELCFGR,		4,	2),
	MUX_CFG(MUX_MUXSEL2,		RCC_MUXSELCFGR,		8,	2),
	MUX_CFG(MUX_MUXSEL3,		RCC_MUXSELCFGR,		12,	2),
	MUX_CFG(MUX_MUXSEL4,		RCC_MUXSELCFGR,		16,	2),
	MUX_CFG(MUX_MUXSEL5,		RCC_MUXSELCFGR,		20,	2),
	MUX_CFG(MUX_MUXSEL6,		RCC_MUXSELCFGR,		24,	2),
	MUX_CFG(MUX_MUXSEL7,		RCC_MUXSELCFGR,		28,	2),
	MUX_CFG(MUX_XBARSEL,		RCC_XBAR0CFGR,		0,	4),
	MUX_CFG(MUX_RTC,		RCC_BDCR,		16,	2),
	MUX_CFG(MUX_D3PER,		RCC_D3DCR,		16,	2),
#endif
};

enum enum_gate_cfg {
	GATE_MCO1,
	GATE_MCO2,
	GATE_OSPI1,
	GATE_OSPI2,
	GATE_DBG,
	GATE_TRACE,
	GATE_STM,
	GATE_IS2M,
	GATE_TIM1,
	GATE_TIM2,
	GATE_TIM3,
	GATE_TIM4,
	GATE_TIM5,
	GATE_TIM6,
	GATE_TIM7,
	GATE_TIM8,
	GATE_TIM10,
	GATE_TIM11,
	GATE_TIM12,
	GATE_TIM13,
	GATE_TIM14,
	GATE_TIM15,
	GATE_TIM16,
	GATE_TIM17,
	GATE_TIM20,
	GATE_LPTIM1,
	GATE_LPTIM2,
	GATE_LPTIM3,
	GATE_LPTIM4,
	GATE_LPTIM5,
	GATE_SPI1,
	GATE_SPI2,
	GATE_SPI3,
	GATE_SPI4,
	GATE_SPI5,
	GATE_SPI6,
	GATE_SPI7,
	GATE_SPI8,
	GATE_SPDIFRX,
	GATE_USART1,
	GATE_USART2,
	GATE_USART3,
	GATE_UART4,
	GATE_UART5,
	GATE_USART6,
	GATE_UART7,
	GATE_UART8,
	GATE_UART9,
	GATE_LPUART1,
	GATE_I2C1,
	GATE_I2C2,
	GATE_I2C3,
	GATE_I2C4,
	GATE_I2C5,
	GATE_I2C6,
	GATE_I2C7,
	GATE_I2C8,
	GATE_SAI1,
	GATE_SAI2,
	GATE_SAI3,
	GATE_SAI4,
	GATE_MDF1,
	GATE_ADF1,
	GATE_FDCAN,
	GATE_HDP,
	GATE_ADC12,
	GATE_ADC3,
	GATE_ETH1MAC,
	GATE_ETH1,
	GATE_ETH1TX,
	GATE_ETH1RX,
	GATE_ETH1STP,
	GATE_ETH2MAC,
	GATE_ETH2,
	GATE_ETH2STP,
	GATE_ETH2TX,
	GATE_ETH2RX,
	GATE_USB2,
	GATE_USB2PHY1,
	GATE_USB2PHY2,
	GATE_USB3DR,
	GATE_USB3PCIEPHY,
	GATE_PCIE,
	GATE_USBTC,
	GATE_ETHSWMAC,
	GATE_ETHSW,
	GATE_ETHSWREF,
	GATE_ETHSWACMCFG,
	GATE_ETHSWACMMSG,
	GATE_STGEN,
	GATE_SDMMC1,
	GATE_SDMMC2,
	GATE_SDMMC3,
	GATE_GPU,
	GATE_LTDC,
	GATE_DSI,
	GATE_LVDS,
	GATE_CSI,
	GATE_DCMIPP,
	GATE_CCI,
	GATE_VDEC,
	GATE_VENC,
	GATE_RNG,
	GATE_PKA,
	GATE_SAES,
	GATE_HASH,
	GATE_CRYP1,
	GATE_CRYP2,
	GATE_IWDG1,
	GATE_IWDG2,
	GATE_IWDG3,
	GATE_IWDG4,
	GATE_IWDG5,
	GATE_WWDG1,
	GATE_WWDG2,
	GATE_VREF,
	GATE_DTS,
	GATE_CRC,
	GATE_SERC,
	GATE_OSPIIOM,
	GATE_I3C1,
	GATE_I3C2,
	GATE_I3C3,
	GATE_I3C4,

#ifdef CONFIG_DEBUG_FS
	GATE_HSI,
	GATE_HSE,
	GATE_LSE,
	GATE_LSI,
	GATE_MSI,
	GATE_HSEDIV2,
	GATE_PLL1,
	GATE_PLL2,
	GATE_PLL3,
	GATE_PLL4,
	GATE_PLL5,
	GATE_PLL6,
	GATE_PLL7,
	GATE_PLL8,
	GATE_RTCCK,
	GATE_C3,
	GATE_LPTIM3C3,
	GATE_LPTIM4C3,
	GATE_LPTIM5C3,
	GATE_SPI8C3,
	GATE_LPUART1C3,
	GATE_I2C8C3,
	GATE_ADF1C3,
	GATE_GPIOZC3,
	GATE_LPDMAC3,
	GATE_RTCC3,
	GATE_I3C4C3,
	GATE_DDRCP,
	GATE_DDRCAPB,
	GATE_DDRPHYCAPB,
	GATE_DDRPHYC,
	GATE_DDRCFG,
	GATE_SYSRAM,
	GATE_VDERAM,
	GATE_SRAM1,
	GATE_SRAM2,
	GATE_RETRAM,
	GATE_BKPSRAM,
	GATE_LPSRAM1,
	GATE_LPSRAM2,
	GATE_LPSRAM3,
	GATE_FMC,
	GATE_ETR,
	GATE_GPIOA,
	GATE_GPIOB,
	GATE_GPIOC,
	GATE_GPIOD,
	GATE_GPIOE,
	GATE_GPIOF,
	GATE_GPIOG,
	GATE_GPIOH,
	GATE_GPIOI,
	GATE_GPIOJ,
	GATE_GPIOK,
	GATE_GPIOZ,
	GATE_HPDMA1,
	GATE_HPDMA2,
	GATE_HPDMA3,
	GATE_LPDMA,
	GATE_HSEM,
	GATE_IPCC1,
	GATE_IPCC2,
	GATE_RTC,
	GATE_SYSCPU1,
	GATE_BSEC,
	GATE_GICV2M,
#endif
	GATE_NB
};

#define GATE_CFG(id, _offset, _bit_idx, _offset_clr)\
	[id] = {\
		.offset		= (_offset),\
		.bit_idx	= (_bit_idx),\
		.set_clr	= (_offset_clr),\
	}

static const struct stm32_gate_cfg stm32mp25_gates[GATE_NB] = {
	GATE_CFG(GATE_MCO1,		RCC_MCO1CFGR,		8,	0),
	GATE_CFG(GATE_MCO2,		RCC_MCO2CFGR,		8,	0),
	GATE_CFG(GATE_OSPI1,		RCC_OSPI1CFGR,		1,	0),
	GATE_CFG(GATE_OSPI2,		RCC_OSPI2CFGR,		1,	0),
	GATE_CFG(GATE_DBG,		RCC_DBGCFGR,		8,	0),
	GATE_CFG(GATE_TRACE,		RCC_DBGCFGR,		9,	0),
	GATE_CFG(GATE_STM,		RCC_STMCFGR,		1,	0),
	GATE_CFG(GATE_IS2M,		RCC_IS2MCFGR,		1,	0),
	GATE_CFG(GATE_TIM1,		RCC_TIM1CFGR,		1,	0),
	GATE_CFG(GATE_TIM2,		RCC_TIM2CFGR,		1,	0),
	GATE_CFG(GATE_TIM3,		RCC_TIM3CFGR,		1,	0),
	GATE_CFG(GATE_TIM4,		RCC_TIM4CFGR,		1,	0),
	GATE_CFG(GATE_TIM5,		RCC_TIM5CFGR,		1,	0),
	GATE_CFG(GATE_TIM6,		RCC_TIM6CFGR,		1,	0),
	GATE_CFG(GATE_TIM7,		RCC_TIM7CFGR,		1,	0),
	GATE_CFG(GATE_TIM8,		RCC_TIM8CFGR,		1,	0),
	GATE_CFG(GATE_TIM10,		RCC_TIM10CFGR,		1,	0),
	GATE_CFG(GATE_TIM11,		RCC_TIM11CFGR,		1,	0),
	GATE_CFG(GATE_TIM12,		RCC_TIM12CFGR,		1,	0),
	GATE_CFG(GATE_TIM13,		RCC_TIM13CFGR,		1,	0),
	GATE_CFG(GATE_TIM14,		RCC_TIM14CFGR,		1,	0),
	GATE_CFG(GATE_TIM15,		RCC_TIM15CFGR,		1,	0),
	GATE_CFG(GATE_TIM16,		RCC_TIM16CFGR,		1,	0),
	GATE_CFG(GATE_TIM17,		RCC_TIM17CFGR,		1,	0),
	GATE_CFG(GATE_TIM20,		RCC_TIM20CFGR,		1,	0),
	GATE_CFG(GATE_LPTIM1,		RCC_LPTIM1CFGR,		1,	0),
	GATE_CFG(GATE_LPTIM2,		RCC_LPTIM2CFGR,		1,	0),
	GATE_CFG(GATE_LPTIM3,		RCC_LPTIM3CFGR,		1,	0),
	GATE_CFG(GATE_LPTIM4,		RCC_LPTIM4CFGR,		1,	0),
	GATE_CFG(GATE_LPTIM5,		RCC_LPTIM5CFGR,		1,	0),
	GATE_CFG(GATE_SPI1,		RCC_SPI1CFGR,		1,	0),
	GATE_CFG(GATE_SPI2,		RCC_SPI2CFGR,		1,	0),
	GATE_CFG(GATE_SPI3,		RCC_SPI3CFGR,		1,	0),
	GATE_CFG(GATE_SPI4,		RCC_SPI4CFGR,		1,	0),
	GATE_CFG(GATE_SPI5,		RCC_SPI5CFGR,		1,	0),
	GATE_CFG(GATE_SPI6,		RCC_SPI6CFGR,		1,	0),
	GATE_CFG(GATE_SPI7,		RCC_SPI7CFGR,		1,	0),
	GATE_CFG(GATE_SPI8,		RCC_SPI8CFGR,		1,	0),
	GATE_CFG(GATE_SPDIFRX,		RCC_SPDIFRXCFGR,	1,	0),
	GATE_CFG(GATE_USART1,		RCC_USART1CFGR,		1,	0),
	GATE_CFG(GATE_USART2,		RCC_USART2CFGR,		1,	0),
	GATE_CFG(GATE_USART3,		RCC_USART3CFGR,		1,	0),
	GATE_CFG(GATE_UART4,		RCC_UART4CFGR,		1,	0),
	GATE_CFG(GATE_UART5,		RCC_UART5CFGR,		1,	0),
	GATE_CFG(GATE_USART6,		RCC_USART6CFGR,		1,	0),
	GATE_CFG(GATE_UART7,		RCC_UART7CFGR,		1,	0),
	GATE_CFG(GATE_UART8,		RCC_UART8CFGR,		1,	0),
	GATE_CFG(GATE_UART9,		RCC_UART9CFGR,		1,	0),
	GATE_CFG(GATE_LPUART1,		RCC_LPUART1CFGR,	1,	0),
	GATE_CFG(GATE_I2C1,		RCC_I2C1CFGR,		1,	0),
	GATE_CFG(GATE_I2C2,		RCC_I2C2CFGR,		1,	0),
	GATE_CFG(GATE_I2C3,		RCC_I2C3CFGR,		1,	0),
	GATE_CFG(GATE_I2C4,		RCC_I2C4CFGR,		1,	0),
	GATE_CFG(GATE_I2C5,		RCC_I2C5CFGR,		1,	0),
	GATE_CFG(GATE_I2C6,		RCC_I2C6CFGR,		1,	0),
	GATE_CFG(GATE_I2C7,		RCC_I2C7CFGR,		1,	0),
	GATE_CFG(GATE_I2C8,		RCC_I2C8CFGR,		1,	0),
	GATE_CFG(GATE_SAI1,		RCC_SAI1CFGR,		1,	0),
	GATE_CFG(GATE_SAI2,		RCC_SAI2CFGR,		1,	0),
	GATE_CFG(GATE_SAI3,		RCC_SAI3CFGR,		1,	0),
	GATE_CFG(GATE_SAI4,		RCC_SAI4CFGR,		1,	0),
	GATE_CFG(GATE_MDF1,		RCC_MDF1CFGR,		1,	0),
	GATE_CFG(GATE_ADF1,		RCC_ADF1CFGR,		1,	0),
	GATE_CFG(GATE_FDCAN,		RCC_FDCANCFGR,		1,	0),
	GATE_CFG(GATE_HDP,		RCC_HDPCFGR,		1,	0),
	GATE_CFG(GATE_ADC12,		RCC_ADC12CFGR,		1,	0),
	GATE_CFG(GATE_ADC3,		RCC_ADC3CFGR,		1,	0),
	GATE_CFG(GATE_ETH1MAC,		RCC_ETH1CFGR,		1,	0),
	GATE_CFG(GATE_ETH1STP,		RCC_ETH1CFGR,		4,	0),
	GATE_CFG(GATE_ETH1,		RCC_ETH1CFGR,		5,	0),
	GATE_CFG(GATE_ETH1TX,		RCC_ETH1CFGR,		8,	0),
	GATE_CFG(GATE_ETH1RX,		RCC_ETH1CFGR,		10,	0),
	GATE_CFG(GATE_ETH2MAC,		RCC_ETH2CFGR,		1,	0),
	GATE_CFG(GATE_ETH2STP,		RCC_ETH2CFGR,		4,	0),
	GATE_CFG(GATE_ETH2,		RCC_ETH2CFGR,		5,	0),
	GATE_CFG(GATE_ETH2TX,		RCC_ETH2CFGR,		8,	0),
	GATE_CFG(GATE_ETH2RX,		RCC_ETH2CFGR,		10,	0),
	GATE_CFG(GATE_USB2,		RCC_USB2CFGR,		1,	0),
	GATE_CFG(GATE_USB2PHY1,		RCC_USB2PHY1CFGR,	1,	0),
	GATE_CFG(GATE_USB2PHY2,		RCC_USB2PHY2CFGR,	1,	0),
	GATE_CFG(GATE_USB3DR,		RCC_USB3DRCFGR,	1,	0),
	GATE_CFG(GATE_USB3PCIEPHY,	RCC_USB3PCIEPHYCFGR,	1,	0),
	GATE_CFG(GATE_PCIE,		RCC_PCIECFGR,		1,	0),
	GATE_CFG(GATE_USBTC,		RCC_USBTCCFGR,		1,	0),
	GATE_CFG(GATE_ETHSWMAC,		RCC_ETHSWCFGR,		1,	0),
	GATE_CFG(GATE_ETHSW,		RCC_ETHSWCFGR,		5,	0),
	GATE_CFG(GATE_ETHSWREF,		RCC_ETHSWCFGR,		21,	0),
	GATE_CFG(GATE_ETHSWACMCFG,	RCC_ETHSWACMCFGR,	1,	0),
	GATE_CFG(GATE_ETHSWACMMSG,	RCC_ETHSWACMMSGCFGR,	1,	0),
	GATE_CFG(GATE_STGEN,		RCC_STGENCFGR,		1,	0),
	GATE_CFG(GATE_SDMMC1,		RCC_SDMMC1CFGR,		1,	0),
	GATE_CFG(GATE_SDMMC2,		RCC_SDMMC2CFGR,		1,	0),
	GATE_CFG(GATE_SDMMC3,		RCC_SDMMC3CFGR,		1,	0),
	GATE_CFG(GATE_GPU,		RCC_GPUCFGR,		1,	0),
	GATE_CFG(GATE_LTDC,		RCC_LTDCCFGR,		1,	0),
	GATE_CFG(GATE_DSI,		RCC_DSICFGR,		1,	0),
	GATE_CFG(GATE_LVDS,		RCC_LVDSCFGR,		1,	0),
	GATE_CFG(GATE_CSI,		RCC_CSICFGR,		1,	0),
	GATE_CFG(GATE_DCMIPP,		RCC_DCMIPPCFGR,		1,	0),
	GATE_CFG(GATE_CCI,		RCC_CCICFGR,		1,	0),
	GATE_CFG(GATE_VDEC,		RCC_VDECCFGR,		1,	0),
	GATE_CFG(GATE_VENC,		RCC_VENCCFGR,		1,	0),
	GATE_CFG(GATE_RNG,		RCC_RNGCFGR,		1,	0),
	GATE_CFG(GATE_PKA,		RCC_PKACFGR,		1,	0),
	GATE_CFG(GATE_SAES,		RCC_SAESCFGR,		1,	0),
	GATE_CFG(GATE_HASH,		RCC_HASHCFGR,		1,	0),
	GATE_CFG(GATE_CRYP1,		RCC_CRYP1CFGR,		1,	0),
	GATE_CFG(GATE_CRYP2,		RCC_CRYP2CFGR,		1,	0),
	GATE_CFG(GATE_IWDG1,		RCC_IWDG1CFGR,		1,	0),
	GATE_CFG(GATE_IWDG2,		RCC_IWDG2CFGR,		1,	0),
	GATE_CFG(GATE_IWDG3,		RCC_IWDG3CFGR,		1,	0),
	GATE_CFG(GATE_IWDG4,		RCC_IWDG4CFGR,		1,	0),
	GATE_CFG(GATE_IWDG5,		RCC_IWDG5CFGR,		1,	0),
	GATE_CFG(GATE_WWDG1,		RCC_WWDG1CFGR,		1,	0),
	GATE_CFG(GATE_WWDG2,		RCC_WWDG2CFGR,		1,	0),
	GATE_CFG(GATE_VREF,		RCC_VREFCFGR,		1,	0),
	GATE_CFG(GATE_DTS,		RCC_DTSCFGR,		1,	0),
	GATE_CFG(GATE_CRC,		RCC_CRCCFGR,		1,	0),
	GATE_CFG(GATE_SERC,		RCC_SERCCFGR,		1,	0),
	GATE_CFG(GATE_OSPIIOM,		RCC_OSPIIOMCFGR,	1,	0),
	GATE_CFG(GATE_I3C1,		RCC_I3C1CFGR,		1,	0),
	GATE_CFG(GATE_I3C2,		RCC_I3C2CFGR,		1,	0),
	GATE_CFG(GATE_I3C3,		RCC_I3C3CFGR,		1,	0),
	GATE_CFG(GATE_I3C4,		RCC_I3C4CFGR,		1,	0),

#ifdef CONFIG_DEBUG_FS
	GATE_CFG(GATE_LSE,		RCC_BDCR,		0,	0),
	GATE_CFG(GATE_LSI,		RCC_BDCR,		9,	0),
	GATE_CFG(GATE_RTCCK,		RCC_BDCR,		20,	0),
	GATE_CFG(GATE_MSI,		RCC_D3DCR,		0,	0),
	GATE_CFG(GATE_PLL1,		RCC_PLL2CFGR1,		8,	0),
	GATE_CFG(GATE_PLL2,		RCC_PLL2CFGR1,		8,	0),
	GATE_CFG(GATE_PLL3,		RCC_PLL3CFGR1,		8,	0),
	GATE_CFG(GATE_PLL4,		RCC_PLL4CFGR1,		8,	0),
	GATE_CFG(GATE_PLL5,		RCC_PLL5CFGR1,		8,	0),
	GATE_CFG(GATE_PLL6,		RCC_PLL6CFGR1,		8,	0),
	GATE_CFG(GATE_PLL7,		RCC_PLL7CFGR1,		8,	0),
	GATE_CFG(GATE_PLL8,		RCC_PLL8CFGR1,		8,	0),
	GATE_CFG(GATE_C3,		RCC_C3CFGR,		1,	0),
	GATE_CFG(GATE_LPTIM3C3,		RCC_C3CFGR,		16,	0),
	GATE_CFG(GATE_LPTIM4C3,		RCC_C3CFGR,		17,	0),
	GATE_CFG(GATE_LPTIM5C3,		RCC_C3CFGR,		18,	0),
	GATE_CFG(GATE_SPI8C3,		RCC_C3CFGR,		19,	0),
	GATE_CFG(GATE_LPUART1C3,	RCC_C3CFGR,		20,	0),
	GATE_CFG(GATE_I2C8C3,		RCC_C3CFGR,		21,	0),
	GATE_CFG(GATE_ADF1C3,		RCC_C3CFGR,		23,	0),
	GATE_CFG(GATE_GPIOZC3,		RCC_C3CFGR,		24,	0),
	GATE_CFG(GATE_LPDMAC3,		RCC_C3CFGR,		25,	0),
	GATE_CFG(GATE_RTCC3,		RCC_C3CFGR,		26,	0),
	GATE_CFG(GATE_I3C4C3,		RCC_C3CFGR,		27,	0),
	GATE_CFG(GATE_HSI,		RCC_OCENSETR,		0,	1),
	GATE_CFG(GATE_HSEDIV2,		RCC_OCENSETR,		5,	1),
	GATE_CFG(GATE_HSE,		RCC_OCENSETR,		8,	1),
	GATE_CFG(GATE_DDRCP,		RCC_DDRCPCFGR,		1,	0),
	GATE_CFG(GATE_DDRCAPB,		RCC_DDRCAPBCFGR,	1,	0),
	GATE_CFG(GATE_DDRPHYCAPB,	RCC_DDRPHYCAPBCFGR,	1,	0),
	GATE_CFG(GATE_DDRPHYC,		RCC_DDRPHYCCFGR,	1,	0),
	GATE_CFG(GATE_DDRCFG,		RCC_DDRCFGR,		1,	0),
	GATE_CFG(GATE_SYSRAM,		RCC_SYSRAMCFGR,		1,	0),
	GATE_CFG(GATE_VDERAM,		RCC_VDERAMCFGR,		1,	0),
	GATE_CFG(GATE_SRAM1,		RCC_SRAM1CFGR,		1,	0),
	GATE_CFG(GATE_SRAM2,		RCC_SRAM2CFGR,		1,	0),
	GATE_CFG(GATE_RETRAM,		RCC_RETRAMCFGR,		1,	0),
	GATE_CFG(GATE_BKPSRAM,		RCC_BKPSRAMCFGR,	1,	0),
	GATE_CFG(GATE_LPSRAM1,		RCC_LPSRAM1CFGR,	1,	0),
	GATE_CFG(GATE_LPSRAM2,		RCC_LPSRAM2CFGR,	1,	0),
	GATE_CFG(GATE_LPSRAM3,		RCC_LPSRAM3CFGR,	1,	0),
	GATE_CFG(GATE_FMC,		RCC_FMCCFGR,		1,	0),
	GATE_CFG(GATE_ETR,		RCC_ETRCFGR,		1,	0),
	GATE_CFG(GATE_GPIOA,		RCC_GPIOACFGR,		1,	0),
	GATE_CFG(GATE_GPIOB,		RCC_GPIOBCFGR,		1,	0),
	GATE_CFG(GATE_GPIOC,		RCC_GPIOCCFGR,		1,	0),
	GATE_CFG(GATE_GPIOD,		RCC_GPIODCFGR,		1,	0),
	GATE_CFG(GATE_GPIOE,		RCC_GPIOECFGR,		1,	0),
	GATE_CFG(GATE_GPIOF,		RCC_GPIOFCFGR,		1,	0),
	GATE_CFG(GATE_GPIOG,		RCC_GPIOGCFGR,		1,	0),
	GATE_CFG(GATE_GPIOH,		RCC_GPIOHCFGR,		1,	0),
	GATE_CFG(GATE_GPIOI,		RCC_GPIOICFGR,		1,	0),
	GATE_CFG(GATE_GPIOJ,		RCC_GPIOJCFGR,		1,	0),
	GATE_CFG(GATE_GPIOK,		RCC_GPIOKCFGR,		1,	0),
	GATE_CFG(GATE_GPIOZ,		RCC_GPIOZCFGR,		1,	0),
	GATE_CFG(GATE_HPDMA1,		RCC_HPDMA1CFGR,		1,	0),
	GATE_CFG(GATE_HPDMA2,		RCC_HPDMA2CFGR,		1,	0),
	GATE_CFG(GATE_HPDMA3,		RCC_HPDMA3CFGR,		1,	0),
	GATE_CFG(GATE_LPDMA,		RCC_LPDMACFGR,		1,	0),
	GATE_CFG(GATE_HSEM,		RCC_HSEMCFGR,		1,	0),
	GATE_CFG(GATE_IPCC1,		RCC_IPCC1CFGR,		1,	0),
	GATE_CFG(GATE_IPCC2,		RCC_IPCC2CFGR,		1,	0),
	GATE_CFG(GATE_RTC,		RCC_RTCCFGR,		1,	0),
	GATE_CFG(GATE_SYSCPU1,		RCC_SYSCPU1CFGR,	1,	0),
	GATE_CFG(GATE_BSEC,		RCC_BSECCFGR,		1,	0),
	GATE_CFG(GATE_GICV2M,		RCC_GICV2MCFGR,		1,	0),
#endif
};

#ifdef CONFIG_DEBUG_FS
enum enum_div_cfg {
	DIV_LSMCU,
	DIV_APB1,
	DIV_APB2,
	DIV_APB3,
	DIV_APB4,
	DIV_APBDBG,
	DIV_RTC,
	DIV_NB
};

static const struct clk_div_table apb_div_table[] = {
	{ 0, 1 },  { 1, 2 },  { 2, 4 },  { 3, 8 }, { 4, 16 },
	{ 5, 16 }, { 6, 16 }, { 7, 16 }, { 0 },
};

#define DIV_CFG(_id, _offset, _shift, _width, _table)\
	[(_id)] = {\
		.offset	= (_offset),\
		.shift	= (_shift),\
		.width	= (_width),\
		.table	= (_table),\
	}

static const struct stm32_div_cfg stm32mp25_dividers[DIV_NB] = {
	DIV_CFG(DIV_RTC,	RCC_RTCDIVR,	0, 6, NULL),
	DIV_CFG(DIV_APB1,	RCC_APB1DIVR,	0, 3, apb_div_table),
	DIV_CFG(DIV_APB2,	RCC_APB2DIVR,	0, 3, apb_div_table),
	DIV_CFG(DIV_APB3,	RCC_APB3DIVR,	0, 3, apb_div_table),
	DIV_CFG(DIV_APB4,	RCC_APB4DIVR,	0, 3, apb_div_table),
	DIV_CFG(DIV_APBDBG,	RCC_APBDBGDIVR,	0, 3, apb_div_table),
	DIV_CFG(DIV_LSMCU,	RCC_LSMCUDIVR,	0, 1, NULL),
};
#endif

#define CLK_STM32_GATE(_name, _parent, _flags, _gate_id)\
struct clk_stm32_gate _name = {\
	.gate_id = _gate_id,\
	.hw.init = CLK_HW_INIT(#_name, _parent, &clk_stm32_gate_ops, _flags),\
}

#define CLK_STM32_MUX(_name, _parents, _flags, _mux_id)\
struct clk_stm32_mux _name = {\
	.mux_id = _mux_id,\
	.hw.init = CLK_HW_INIT_PARENTS(#_name, _parents, &clk_stm32_mux_ops, _flags),\
}

#define CLK_STM32_DIV(_name, _parent, _flags, _div_id)\
struct clk_stm32_div _name = {\
	.div_id = _div_id,\
	.hw.init = CLK_HW_INIT(#_name, _parent, &clk_stm32_divider_ops, _flags),\
}

#define CLK_STM32_COMPOSITE(_name, _parents, _flags, _gate_id, _mux_id, _div_id)\
struct clk_stm32_composite _name = {\
	.gate_id = _gate_id,\
	.mux_id = _mux_id,\
	.div_id = _div_id,\
	.hw.init = CLK_HW_INIT_PARENTS(#_name, _parents, &clk_stm32_composite_ops, _flags),\
}

/* ADC */
static CLK_STM32_GATE(ck_icn_p_adc12, "ck_icn_ls_mcu", 0, GATE_ADC12);
static CLK_STM32_COMPOSITE(ck_ker_adc12, adc12_src, 0, GATE_ADC12, MUX_ADC12, NO_STM32_DIV);
static CLK_STM32_GATE(ck_icn_p_adc3, "ck_icn_ls_mcu", 0, GATE_ADC3);
static CLK_STM32_COMPOSITE(ck_ker_adc3, adc3_src, 0, GATE_ADC3, MUX_ADC3, NO_STM32_DIV);

/* ADF */
static CLK_STM32_GATE(ck_icn_p_adf1, "ck_icn_ls_mcu", 0, GATE_ADF1);
static CLK_STM32_GATE(ck_ker_adf1, "ck_flexgen_42", 0, GATE_ADF1);

/* DCMI */
static CLK_STM32_GATE(ck_icn_p_cci, "ck_icn_ls_mcu", 0, GATE_CCI);

/* CSI-HOST */
static CLK_STM32_GATE(ck_icn_p_csi, "ck_icn_apb4", 0, GATE_CSI);
static CLK_STM32_GATE(ck_ker_csi, "ck_flexgen_29", 0, GATE_CSI);
static CLK_STM32_GATE(ck_ker_csitxesc, "ck_flexgen_30", 0, GATE_CSI);

/* CSI-PHY */
static CLK_STM32_GATE(ck_ker_csiphy, "ck_flexgen_31", 0, GATE_CSI);

/* DCMIPP */
static CLK_STM32_GATE(ck_icn_p_dcmipp, "ck_icn_apb4", 0, GATE_DCMIPP);

/* CRC */
static CLK_STM32_GATE(ck_icn_p_crc, "ck_icn_ls_mcu", 0, GATE_CRC);

/* CRYP */
static CLK_STM32_GATE(ck_icn_p_cryp1, "ck_icn_ls_mcu", 0, GATE_CRYP1);
static CLK_STM32_GATE(ck_icn_p_cryp2, "ck_icn_ls_mcu", 0, GATE_CRYP2);

/* DBG & TRACE*/
/* Trace and debug clocks are managed by SCMI */

/* LTDC */
static CLK_STM32_GATE(ck_icn_p_ltdc, "ck_icn_apb4", 0, GATE_LTDC);
static CLK_STM32_GATE(ck_ker_ltdc, "ck_flexgen_27", CLK_SET_RATE_PARENT, GATE_LTDC);

/* DSI */
static CLK_STM32_GATE(ck_icn_p_dsi, "ck_icn_apb4", 0, GATE_DSI);
static CLK_STM32_COMPOSITE(clk_lanebyte, dsiblane_src, 0, GATE_DSI, MUX_DSIBLANE, NO_STM32_DIV);

/* LVDS */
static CLK_STM32_GATE(ck_icn_p_lvds, "ck_icn_apb4", 0, GATE_LVDS);

/* DSI PHY */
static CLK_STM32_COMPOSITE(clk_phy_dsi, dsiphy_src, 0, GATE_DSI, MUX_DSIPHY, NO_STM32_DIV);

/* LVDS PHY */
static CLK_STM32_COMPOSITE(ck_ker_lvdsphy, lvdsphy_src, 0, GATE_LVDS, MUX_LVDSPHY, NO_STM32_DIV);

/* DTS */
static CLK_STM32_COMPOSITE(ck_ker_dts, dts_src, 0, GATE_DTS, MUX_DTS, NO_STM32_DIV);

/* ETHERNET */
static CLK_STM32_GATE(ck_icn_p_eth1, "ck_icn_ls_mcu", 0, GATE_ETH1);
static CLK_STM32_GATE(ck_ker_eth1stp, "ck_icn_ls_mcu", 0, GATE_ETH1STP);
static CLK_STM32_GATE(ck_ker_eth1, "ck_flexgen_54", 0, GATE_ETH1);
static CLK_STM32_GATE(ck_ker_eth1ptp, "ck_flexgen_56", 0, GATE_ETH1);
static CLK_STM32_GATE(ck_ker_eth1mac, "ck_icn_ls_mcu", 0, GATE_ETH1MAC);
static CLK_STM32_GATE(ck_ker_eth1tx, "ck_icn_ls_mcu", 0, GATE_ETH1TX);
static CLK_STM32_GATE(ck_ker_eth1rx, "ck_icn_ls_mcu", 0, GATE_ETH1RX);

static CLK_STM32_GATE(ck_icn_p_eth2, "ck_icn_ls_mcu", 0, GATE_ETH2);
static CLK_STM32_GATE(ck_ker_eth2stp, "ck_icn_ls_mcu", 0, GATE_ETH2STP);
static CLK_STM32_GATE(ck_ker_eth2, "ck_flexgen_55", 0, GATE_ETH2);
static CLK_STM32_GATE(ck_ker_eth2ptp, "ck_flexgen_56", 0, GATE_ETH2);
static CLK_STM32_GATE(ck_ker_eth2mac, "ck_icn_ls_mcu", 0, GATE_ETH2MAC);
static CLK_STM32_GATE(ck_ker_eth2tx, "ck_icn_ls_mcu", 0, GATE_ETH2TX);
static CLK_STM32_GATE(ck_ker_eth2rx, "ck_icn_ls_mcu", 0, GATE_ETH2RX);

static CLK_STM32_GATE(ck_icn_p_ethsw, "ck_icn_ls_mcu", 0, GATE_ETHSWMAC);
static CLK_STM32_GATE(ck_ker_ethsw, "ck_flexgen_54", 0, GATE_ETHSW);
static CLK_STM32_GATE(ck_ker_ethswref, "ck_flexgen_60", 0, GATE_ETHSWREF);

static CLK_STM32_GATE(ck_icn_p_ethsw_acm_cfg, "ck_icn_ls_mcu", 0, GATE_ETHSWACMCFG);
static CLK_STM32_GATE(ck_icn_p_ethsw_acm_msg, "ck_icn_ls_mcu", 0, GATE_ETHSWACMMSG);

/* FDCAN */
static CLK_STM32_GATE(ck_icn_p_fdcan, "ck_icn_apb2", 0, GATE_FDCAN);
static CLK_STM32_GATE(ck_ker_fdcan, "ck_flexgen_26", 0, GATE_FDCAN);

/* GPU */
static CLK_STM32_GATE(ck_icn_m_gpu, "ck_flexgen_59", 0, GATE_GPU);
static CLK_STM32_GATE(ck_ker_gpu, "ck_pll3", 0, GATE_GPU);

/* HASH */
static CLK_STM32_GATE(ck_icn_p_hash, "ck_icn_ls_mcu", 0, GATE_HASH);

/* HDP */
static CLK_STM32_GATE(ck_icn_p_hdp, "ck_icn_apb3", 0, GATE_HDP);

/* I2C */
static CLK_STM32_GATE(ck_icn_p_i2c8, "ck_icn_ls_mcu", 0, GATE_I2C8);
static CLK_STM32_GATE(ck_icn_p_i2c1, "ck_icn_apb1", 0, GATE_I2C1);
static CLK_STM32_GATE(ck_icn_p_i2c2, "ck_icn_apb1", 0, GATE_I2C2);
static CLK_STM32_GATE(ck_icn_p_i2c3, "ck_icn_apb1", 0, GATE_I2C3);
static CLK_STM32_GATE(ck_icn_p_i2c4, "ck_icn_apb1", 0, GATE_I2C4);
static CLK_STM32_GATE(ck_icn_p_i2c5, "ck_icn_apb1", 0, GATE_I2C5);
static CLK_STM32_GATE(ck_icn_p_i2c6, "ck_icn_apb1", 0, GATE_I2C6);
static CLK_STM32_GATE(ck_icn_p_i2c7, "ck_icn_apb1", 0, GATE_I2C7);

static CLK_STM32_GATE(ck_ker_i2c1, "ck_flexgen_12", 0, GATE_I2C1);
static CLK_STM32_GATE(ck_ker_i2c2, "ck_flexgen_12", 0, GATE_I2C2);
static CLK_STM32_GATE(ck_ker_i2c3, "ck_flexgen_13", 0, GATE_I2C3);
static CLK_STM32_GATE(ck_ker_i2c5, "ck_flexgen_13", 0, GATE_I2C5);
static CLK_STM32_GATE(ck_ker_i2c4, "ck_flexgen_14", 0, GATE_I2C4);
static CLK_STM32_GATE(ck_ker_i2c6, "ck_flexgen_14", 0, GATE_I2C6);
static CLK_STM32_GATE(ck_ker_i2c7, "ck_flexgen_15", 0, GATE_I2C7);
static CLK_STM32_GATE(ck_ker_i2c8, "ck_flexgen_38", 0, GATE_I2C8);

/* I3C */
static CLK_STM32_GATE(ck_icn_p_i3c1, "ck_icn_apb1", 0, GATE_I3C1);
static CLK_STM32_GATE(ck_icn_p_i3c2, "ck_icn_apb1", 0, GATE_I3C2);
static CLK_STM32_GATE(ck_icn_p_i3c3, "ck_icn_apb1", 0, GATE_I3C3);
static CLK_STM32_GATE(ck_icn_p_i3c4, "ck_icn_ls_mcu", 0, GATE_I3C4);

static CLK_STM32_GATE(ck_ker_i3c1, "ck_flexgen_12", 0, GATE_I3C1);
static CLK_STM32_GATE(ck_ker_i3c2, "ck_flexgen_12", 0, GATE_I3C2);
static CLK_STM32_GATE(ck_ker_i3c3, "ck_flexgen_13", 0, GATE_I3C3);
static CLK_STM32_GATE(ck_ker_i3c4, "ck_flexgen_36", 0, GATE_I3C4);

/* I2S */
static CLK_STM32_GATE(ck_icn_p_is2m, "ck_icn_apb3", 0, GATE_IS2M);

/* IWDG */
static CLK_STM32_GATE(ck_icn_p_iwdg1, "ck_icn_apb3", 0, GATE_IWDG1);
static CLK_STM32_GATE(ck_icn_p_iwdg2, "ck_icn_apb3", 0, GATE_IWDG2);
static CLK_STM32_GATE(ck_icn_p_iwdg3, "ck_icn_apb3", 0, GATE_IWDG3);
static CLK_STM32_GATE(ck_icn_p_iwdg4, "ck_icn_apb3", 0, GATE_IWDG4);
static CLK_STM32_GATE(ck_icn_p_iwdg5, "ck_icn_ls_mcu", 0, GATE_IWDG5);

/* LPTIM */
static CLK_STM32_GATE(ck_icn_p_lptim1, "ck_icn_apb1", 0, GATE_LPTIM1);
static CLK_STM32_GATE(ck_icn_p_lptim2, "ck_icn_apb1", 0, GATE_LPTIM2);
static CLK_STM32_GATE(ck_icn_p_lptim3, "ck_icn_ls_mcu", 0, GATE_LPTIM3);
static CLK_STM32_GATE(ck_icn_p_lptim4, "ck_icn_ls_mcu", 0, GATE_LPTIM4);
static CLK_STM32_GATE(ck_icn_p_lptim5, "ck_icn_ls_mcu", 0, GATE_LPTIM5);

static CLK_STM32_GATE(ck_ker_lptim1, "ck_flexgen_07", 0, GATE_LPTIM1);
static CLK_STM32_GATE(ck_ker_lptim2, "ck_flexgen_07", 0, GATE_LPTIM2);
static CLK_STM32_GATE(ck_ker_lptim3, "ck_flexgen_40", 0, GATE_LPTIM3);
static CLK_STM32_GATE(ck_ker_lptim4, "ck_flexgen_41", 0, GATE_LPTIM4);
static CLK_STM32_GATE(ck_ker_lptim5, "ck_flexgen_41", 0, GATE_LPTIM5);

/* LPUART */
static CLK_STM32_GATE(ck_icn_p_lpuart1, "ck_icn_ls_mcu", 0, GATE_LPUART1);
static CLK_STM32_GATE(ck_ker_lpuart1, "ck_flexgen_39", 0, GATE_LPUART1);

/* MCO1 & MCO2 */
static CLK_STM32_COMPOSITE(ck_mco1, mco1_src, 0, GATE_MCO1, MUX_MCO1, NO_STM32_DIV);
static CLK_STM32_COMPOSITE(ck_mco2, mco2_src, 0, GATE_MCO2, MUX_MCO2, NO_STM32_DIV);

/* MDF */
static CLK_STM32_GATE(ck_icn_p_mdf1, "ck_icn_ls_mcu", 0, GATE_MDF1);
static CLK_STM32_GATE(ck_ker_mdf1, "ck_flexgen_23", CLK_SET_RATE_PARENT, GATE_MDF1);

/* OSPI */
static CLK_STM32_GATE(ck_icn_p_ospiiom, "ck_icn_ls_mcu", 0, GATE_OSPIIOM);

/* PCIE */
static CLK_STM32_GATE(ck_icn_p_pcie, "ck_icn_ls_mcu", 0, GATE_PCIE);

/* PKA */
static CLK_STM32_GATE(ck_icn_p_pka, "ck_icn_ls_mcu", 0, GATE_PKA);

/* RNG */
static CLK_STM32_GATE(ck_icn_p_rng, "ck_icn_ls_mcu", 0, GATE_RNG);

/* SAES */
static CLK_STM32_GATE(ck_icn_p_saes, "ck_icn_ls_mcu", 0, GATE_SAES);

/* SAI */
static CLK_STM32_GATE(ck_icn_p_sai1, "ck_icn_apb2", 0, GATE_SAI1);
static CLK_STM32_GATE(ck_icn_p_sai2, "ck_icn_apb2", 0, GATE_SAI2);
static CLK_STM32_GATE(ck_icn_p_sai3, "ck_icn_apb2", 0, GATE_SAI3);
static CLK_STM32_GATE(ck_icn_p_sai4, "ck_icn_apb2", 0, GATE_SAI4);
static CLK_STM32_GATE(ck_ker_sai1, "ck_flexgen_23", CLK_SET_RATE_PARENT, GATE_SAI1);
static CLK_STM32_GATE(ck_ker_sai2, "ck_flexgen_24", CLK_SET_RATE_PARENT, GATE_SAI2);
static CLK_STM32_GATE(ck_ker_sai3, "ck_flexgen_25", CLK_SET_RATE_PARENT, GATE_SAI3);
static CLK_STM32_GATE(ck_ker_sai4, "ck_flexgen_25", CLK_SET_RATE_PARENT, GATE_SAI4);

/* SDMMC */
static CLK_STM32_GATE(ck_icn_m_sdmmc1, "ck_icn_sdmmc", 0, GATE_SDMMC1);
static CLK_STM32_GATE(ck_icn_m_sdmmc2, "ck_icn_sdmmc", 0, GATE_SDMMC2);
static CLK_STM32_GATE(ck_icn_m_sdmmc3, "ck_icn_sdmmc", 0, GATE_SDMMC3);
static CLK_STM32_GATE(ck_ker_sdmmc1, "ck_flexgen_51", 0, GATE_SDMMC1);
static CLK_STM32_GATE(ck_ker_sdmmc2, "ck_flexgen_52", 0, GATE_SDMMC2);
static CLK_STM32_GATE(ck_ker_sdmmc3, "ck_flexgen_53", 0, GATE_SDMMC3);

/* SERC */
static CLK_STM32_GATE(ck_icn_p_serc, "ck_icn_apb3", 0, GATE_SERC);

/* SPDIF */
static CLK_STM32_GATE(ck_icn_p_spdifrx, "ck_icn_apb1", 0, GATE_SPDIFRX);
static CLK_STM32_GATE(ck_ker_spdifrx, "ck_flexgen_11", 0, GATE_SPDIFRX);

/* SPI */
static CLK_STM32_GATE(ck_icn_p_spi1, "ck_icn_apb2", 0, GATE_SPI1);
static CLK_STM32_GATE(ck_icn_p_spi2, "ck_icn_apb1", 0, GATE_SPI2);
static CLK_STM32_GATE(ck_icn_p_spi3, "ck_icn_apb1", 0, GATE_SPI3);
static CLK_STM32_GATE(ck_icn_p_spi4, "ck_icn_apb2", 0, GATE_SPI4);
static CLK_STM32_GATE(ck_icn_p_spi5, "ck_icn_apb2", 0, GATE_SPI5);
static CLK_STM32_GATE(ck_icn_p_spi6, "ck_icn_apb2", 0, GATE_SPI6);
static CLK_STM32_GATE(ck_icn_p_spi7, "ck_icn_apb2", 0, GATE_SPI7);
static CLK_STM32_GATE(ck_icn_p_spi8, "ck_icn_ls_mcu", 0, GATE_SPI8);

static CLK_STM32_GATE(ck_ker_spi1, "ck_flexgen_16", CLK_SET_RATE_PARENT, GATE_SPI1);
static CLK_STM32_GATE(ck_ker_spi2, "ck_flexgen_10", CLK_SET_RATE_PARENT, GATE_SPI2);
static CLK_STM32_GATE(ck_ker_spi3, "ck_flexgen_10", CLK_SET_RATE_PARENT, GATE_SPI3);
static CLK_STM32_GATE(ck_ker_spi4, "ck_flexgen_17", 0, GATE_SPI4);
static CLK_STM32_GATE(ck_ker_spi5, "ck_flexgen_17", 0, GATE_SPI5);
static CLK_STM32_GATE(ck_ker_spi6, "ck_flexgen_18", 0, GATE_SPI6);
static CLK_STM32_GATE(ck_ker_spi7, "ck_flexgen_18", 0, GATE_SPI7);
static CLK_STM32_GATE(ck_ker_spi8, "ck_flexgen_37", 0, GATE_SPI8);

/* Timers */
static CLK_STM32_GATE(ck_icn_p_tim2, "ck_icn_apb1", 0, GATE_TIM2);
static CLK_STM32_GATE(ck_icn_p_tim3, "ck_icn_apb1", 0, GATE_TIM3);
static CLK_STM32_GATE(ck_icn_p_tim4, "ck_icn_apb1", 0, GATE_TIM4);
static CLK_STM32_GATE(ck_icn_p_tim5, "ck_icn_apb1", 0, GATE_TIM5);
static CLK_STM32_GATE(ck_icn_p_tim6, "ck_icn_apb1", 0, GATE_TIM6);
static CLK_STM32_GATE(ck_icn_p_tim7, "ck_icn_apb1", 0, GATE_TIM7);
static CLK_STM32_GATE(ck_icn_p_tim10, "ck_icn_apb1", 0, GATE_TIM10);
static CLK_STM32_GATE(ck_icn_p_tim11, "ck_icn_apb1", 0, GATE_TIM11);
static CLK_STM32_GATE(ck_icn_p_tim12, "ck_icn_apb1", 0, GATE_TIM12);
static CLK_STM32_GATE(ck_icn_p_tim13, "ck_icn_apb1", 0, GATE_TIM13);
static CLK_STM32_GATE(ck_icn_p_tim14, "ck_icn_apb1", 0, GATE_TIM14);

static CLK_STM32_GATE(ck_icn_p_tim1, "ck_icn_apb2", 0, GATE_TIM1);
static CLK_STM32_GATE(ck_icn_p_tim8, "ck_icn_apb2", 0, GATE_TIM8);
static CLK_STM32_GATE(ck_icn_p_tim15, "ck_icn_apb2", 0, GATE_TIM15);
static CLK_STM32_GATE(ck_icn_p_tim16, "ck_icn_apb2", 0, GATE_TIM16);
static CLK_STM32_GATE(ck_icn_p_tim17, "ck_icn_apb2", 0, GATE_TIM17);
static CLK_STM32_GATE(ck_icn_p_tim20, "ck_icn_apb2", 0, GATE_TIM20);

static CLK_STM32_GATE(ck_ker_tim2, "timg1_ck", 0, GATE_TIM2);
static CLK_STM32_GATE(ck_ker_tim3, "timg1_ck", 0, GATE_TIM3);
static CLK_STM32_GATE(ck_ker_tim4, "timg1_ck", 0, GATE_TIM4);
static CLK_STM32_GATE(ck_ker_tim5, "timg1_ck", 0, GATE_TIM5);
static CLK_STM32_GATE(ck_ker_tim6, "timg1_ck", 0, GATE_TIM6);
static CLK_STM32_GATE(ck_ker_tim7, "timg1_ck", 0, GATE_TIM7);
static CLK_STM32_GATE(ck_ker_tim10, "timg1_ck", 0, GATE_TIM10);
static CLK_STM32_GATE(ck_ker_tim11, "timg1_ck", 0, GATE_TIM11);
static CLK_STM32_GATE(ck_ker_tim12, "timg1_ck", 0, GATE_TIM12);
static CLK_STM32_GATE(ck_ker_tim13, "timg1_ck", 0, GATE_TIM13);
static CLK_STM32_GATE(ck_ker_tim14, "timg1_ck", 0, GATE_TIM14);

static CLK_STM32_GATE(ck_ker_tim1, "timg2_ck", 0, GATE_TIM1);
static CLK_STM32_GATE(ck_ker_tim8, "timg2_ck", 0, GATE_TIM8);
static CLK_STM32_GATE(ck_ker_tim15, "timg2_ck", 0, GATE_TIM15);
static CLK_STM32_GATE(ck_ker_tim16, "timg2_ck", 0, GATE_TIM16);
static CLK_STM32_GATE(ck_ker_tim17, "timg2_ck", 0, GATE_TIM17);
static CLK_STM32_GATE(ck_ker_tim20, "timg2_ck", 0, GATE_TIM20);

/* UART/USART */
static CLK_STM32_GATE(ck_icn_p_usart2, "ck_icn_apb1", 0, GATE_USART2);
static CLK_STM32_GATE(ck_icn_p_usart3, "ck_icn_apb1", 0, GATE_USART3);
static CLK_STM32_GATE(ck_icn_p_uart4, "ck_icn_apb1", 0, GATE_UART4);
static CLK_STM32_GATE(ck_icn_p_uart5, "ck_icn_apb1", 0, GATE_UART5);
static CLK_STM32_GATE(ck_icn_p_usart1, "ck_icn_apb2", 0, GATE_USART1);
static CLK_STM32_GATE(ck_icn_p_usart6, "ck_icn_apb2", 0, GATE_USART6);
static CLK_STM32_GATE(ck_icn_p_uart7, "ck_icn_apb2", 0, GATE_UART7);
static CLK_STM32_GATE(ck_icn_p_uart8, "ck_icn_apb2", 0, GATE_UART8);
static CLK_STM32_GATE(ck_icn_p_uart9, "ck_icn_apb2", 0, GATE_UART9);

static CLK_STM32_GATE(ck_ker_usart2, "ck_flexgen_08", 0, GATE_USART2);
static CLK_STM32_GATE(ck_ker_uart4, "ck_flexgen_08", 0, GATE_UART4);
static CLK_STM32_GATE(ck_ker_usart3, "ck_flexgen_09", 0, GATE_USART3);
static CLK_STM32_GATE(ck_ker_uart5, "ck_flexgen_09", 0, GATE_UART5);
static CLK_STM32_GATE(ck_ker_usart1, "ck_flexgen_19", 0, GATE_USART1);
static CLK_STM32_GATE(ck_ker_usart6, "ck_flexgen_20", 0, GATE_USART6);
static CLK_STM32_GATE(ck_ker_uart7, "ck_flexgen_21", 0, GATE_UART7);
static CLK_STM32_GATE(ck_ker_uart8, "ck_flexgen_21", 0, GATE_UART8);
static CLK_STM32_GATE(ck_ker_uart9, "ck_flexgen_22", 0, GATE_UART9);

/* USB2PHY1 */
static CLK_STM32_COMPOSITE(ck_ker_usb2phy1, usb2phy1_src, 0,
			   GATE_USB2PHY1, MUX_USB2PHY1, NO_STM32_DIV);

/* USBH */
static CLK_STM32_GATE(ck_icn_m_usb2ehci, "ck_icn_hsl", 0, GATE_USB2);
static CLK_STM32_GATE(ck_icn_m_usb2ohci, "ck_icn_hsl", 0, GATE_USB2);

/* USB2PHY2 */
static CLK_STM32_COMPOSITE(ck_ker_usb2phy2_en, usb2phy2_src, 0,
			   GATE_USB2PHY2, MUX_USB2PHY2, NO_STM32_DIV);

/* USB3 PCIe COMBOPHY */
static CLK_STM32_GATE(ck_icn_p_usb3pciephy, "ck_icn_apb4", 0, GATE_USB3PCIEPHY);

static CLK_STM32_COMPOSITE(ck_ker_usb3pciephy, usb3pciphy_src, 0,
			   GATE_USB3PCIEPHY, MUX_USB3PCIEPHY, NO_STM32_DIV);

/* USB3 DRD */
static CLK_STM32_GATE(ck_icn_m_usb3dr, "ck_icn_hsl", 0, GATE_USB3DR);
static CLK_STM32_GATE(ck_ker_usb2phy2, "ck_flexgen_58", 0, GATE_USB3DR);

/* USBTC */
static CLK_STM32_GATE(ck_icn_p_usbtc, "ck_icn_apb4", 0, GATE_USBTC);
static CLK_STM32_GATE(ck_ker_usbtc, "ck_flexgen_35", 0, GATE_USBTC);

/* VDEC / VENC */
static CLK_STM32_GATE(ck_icn_p_vdec, "ck_icn_apb4", 0, GATE_VDEC);
static CLK_STM32_GATE(ck_icn_p_venc, "ck_icn_apb4", 0, GATE_VENC);

/* VREF */
static CLK_STM32_GATE(ck_icn_p_vref, "ck_icn_apb3", 0, GATE_VREF);

/* WWDG */
static CLK_STM32_GATE(ck_icn_p_wwdg1, "ck_icn_apb3", 0, GATE_WWDG1);
static CLK_STM32_GATE(ck_icn_p_wwdg2, "ck_icn_ls_mcu", 0, GATE_WWDG2);

static int stm32_rcc_get_access(void __iomem *base, u32 index)
{
	u32 seccfgr, cidcfgr, semcr;
	int bit, cid;

	bit = index % RCC_REG_SIZE;

	seccfgr = readl(base + RCC_SECCFGR(index));
	if (seccfgr & BIT(bit))
		return -EACCES;

	cidcfgr = readl(base + RCC_CIDCFGR(index));
	if (!(cidcfgr & RCC_CIDCFGR_CFEN))
		/* CID filtering is turned off: access granted */
		return 0;

	if (!(cidcfgr & RCC_CIDCFGR_SEM_EN)) {
		/* Static CID mode */
		cid = FIELD_GET(RCC_CIDCFGR_SCID_MASK, cidcfgr);
		if (cid != RCC_CID1)
			return -EACCES;
		return 0;
	}

	/* Pass-list with semaphore mode */
	if (!(cidcfgr & RCC_CIDCFGR_SEMWLC1_EN))
		return -EACCES;

	semcr = readl(base + RCC_SEMCR(index));

	cid = FIELD_GET(RCC_SEMCR_SEMCID_MASK, semcr);
	if (cid != RCC_CID1)
		return -EACCES;

	return 0;
}

static int stm32mp25_check_security(void __iomem *base,
				    const struct clock_config *cfg)
{
	int ret = 0;

	if (cfg->sec_id != SECF_NONE) {
		u32 index = (u32)cfg->sec_id;

		if (index & SEC_RIFSC_FLAG)
			ret = stm32_rifsc_check_access_by_id(index & ~SEC_RIFSC_FLAG);
		else
			ret = stm32_rcc_get_access(base, cfg->sec_id & ~SEC_RIFSC_FLAG);
	}

	return ret;
}

static const struct clock_config stm32mp25_clock_cfg[] = {
	STM32_GATE_CFG(CK_BUS_ETH1, ck_icn_p_eth1, SEC_RIFSC(ETH1)),
	STM32_GATE_CFG(CK_BUS_ETH2, ck_icn_p_eth2, SEC_RIFSC(ETH2)),
	STM32_GATE_CFG(CK_BUS_PCIE, ck_icn_p_pcie, SEC_RIFSC(PCIE)),
	STM32_GATE_CFG(CK_BUS_ETHSW, ck_icn_p_ethsw, SEC_RIFSC(ETHSW_DEIP)),
	STM32_GATE_CFG(CK_BUS_ADC12, ck_icn_p_adc12, SEC_RIFSC(ADC12)),
	STM32_GATE_CFG(CK_BUS_ADC3, ck_icn_p_adc3, SEC_RIFSC(ADC3)),
	STM32_GATE_CFG(CK_BUS_CCI, ck_icn_p_cci, SEC_RIFSC(DCMI_PSSI)),
	STM32_GATE_CFG(CK_BUS_CRC, ck_icn_p_crc, SEC_RIFSC(CRC)),
	STM32_GATE_CFG(CK_BUS_MDF1, ck_icn_p_mdf1, SEC_RIFSC(MDF1)),
	STM32_GATE_CFG(CK_BUS_OSPIIOM, ck_icn_p_ospiiom, SEC_RIFSC(OCTOSPIM)),
	STM32_GATE_CFG(CK_BUS_HASH, ck_icn_p_hash, SEC_RIFSC(HASH)),
	STM32_GATE_CFG(CK_BUS_RNG, ck_icn_p_rng, SEC_RIFSC(RNG)),
	STM32_GATE_CFG(CK_BUS_CRYP1, ck_icn_p_cryp1, SEC_RIFSC(CRYP1)),
	STM32_GATE_CFG(CK_BUS_CRYP2, ck_icn_p_cryp2, SEC_RIFSC(CRYP2)),
	STM32_GATE_CFG(CK_BUS_SAES, ck_icn_p_saes, SEC_RIFSC(SAES)),
	STM32_GATE_CFG(CK_BUS_PKA, ck_icn_p_pka, SEC_RIFSC(PKA)),
	STM32_GATE_CFG(CK_BUS_ADF1, ck_icn_p_adf1, SEC_RIFSC(ADF1)),
	STM32_GATE_CFG(CK_BUS_SPI8, ck_icn_p_spi8, SEC_RIFSC(SPI8)),
	STM32_GATE_CFG(CK_BUS_LPUART1, ck_icn_p_lpuart1, SEC_RIFSC(LPUART1)),
	STM32_GATE_CFG(CK_BUS_I2C8, ck_icn_p_i2c8, SEC_RIFSC(I2C8)),
	STM32_GATE_CFG(CK_BUS_LPTIM3, ck_icn_p_lptim3, SEC_RIFSC(LPTIM3)),
	STM32_GATE_CFG(CK_BUS_LPTIM4, ck_icn_p_lptim4, SEC_RIFSC(LPTIM4)),
	STM32_GATE_CFG(CK_BUS_LPTIM5, ck_icn_p_lptim5, SEC_RIFSC(LPTIM5)),
	STM32_GATE_CFG(CK_BUS_IWDG5, ck_icn_p_iwdg5, SEC_RIFSC(IWDG5)),
	STM32_GATE_CFG(CK_BUS_WWDG2, ck_icn_p_wwdg2, SEC_RIFSC(WWDG2)),
	STM32_GATE_CFG(CK_BUS_I3C4, ck_icn_p_i3c4, SEC_RIFSC(I3C4)),
	STM32_GATE_CFG(CK_BUS_SDMMC1, ck_icn_m_sdmmc1, SEC_RIFSC(SDMMC1)),
	STM32_GATE_CFG(CK_BUS_SDMMC2, ck_icn_m_sdmmc2, SEC_RIFSC(SDMMC2)),
	STM32_GATE_CFG(CK_BUS_SDMMC3, ck_icn_m_sdmmc3, SEC_RIFSC(SDMMC3)),
	STM32_GATE_CFG(CK_BUS_USB2OHCI, ck_icn_m_usb2ohci, SEC_RIFSC(USBH)),
	STM32_GATE_CFG(CK_BUS_USB2EHCI, ck_icn_m_usb2ehci, SEC_RIFSC(USBH)),
	STM32_GATE_CFG(CK_BUS_USB3DR, ck_icn_m_usb3dr, SEC_RIFSC(USB3DR)),
	STM32_GATE_CFG(CK_BUS_TIM2, ck_icn_p_tim2, SEC_RIFSC(TIM2)),
	STM32_GATE_CFG(CK_BUS_TIM3, ck_icn_p_tim3, SEC_RIFSC(TIM3)),
	STM32_GATE_CFG(CK_BUS_TIM4, ck_icn_p_tim4, SEC_RIFSC(TIM4)),
	STM32_GATE_CFG(CK_BUS_TIM5, ck_icn_p_tim5, SEC_RIFSC(TIM5)),
	STM32_GATE_CFG(CK_BUS_TIM6, ck_icn_p_tim6, SEC_RIFSC(TIM6)),
	STM32_GATE_CFG(CK_BUS_TIM7, ck_icn_p_tim7, SEC_RIFSC(TIM7)),
	STM32_GATE_CFG(CK_BUS_TIM10, ck_icn_p_tim10, SEC_RIFSC(TIM10)),
	STM32_GATE_CFG(CK_BUS_TIM11, ck_icn_p_tim11, SEC_RIFSC(TIM11)),
	STM32_GATE_CFG(CK_BUS_TIM12, ck_icn_p_tim12, SEC_RIFSC(TIM12)),
	STM32_GATE_CFG(CK_BUS_TIM13, ck_icn_p_tim13, SEC_RIFSC(TIM13)),
	STM32_GATE_CFG(CK_BUS_TIM14, ck_icn_p_tim14, SEC_RIFSC(TIM14)),
	STM32_GATE_CFG(CK_BUS_LPTIM1, ck_icn_p_lptim1, SEC_RIFSC(LPTIM1)),
	STM32_GATE_CFG(CK_BUS_LPTIM2, ck_icn_p_lptim2, SEC_RIFSC(LPTIM2)),
	STM32_GATE_CFG(CK_BUS_SPI2, ck_icn_p_spi2, SEC_RIFSC(SPI2)),
	STM32_GATE_CFG(CK_BUS_SPI3, ck_icn_p_spi3, SEC_RIFSC(SPI3)),
	STM32_GATE_CFG(CK_BUS_SPDIFRX, ck_icn_p_spdifrx, SEC_RIFSC(SPDIFRX)),
	STM32_GATE_CFG(CK_BUS_USART2, ck_icn_p_usart2, SEC_RIFSC(USART2)),
	STM32_GATE_CFG(CK_BUS_USART3, ck_icn_p_usart3, SEC_RIFSC(USART3)),
	STM32_GATE_CFG(CK_BUS_UART4, ck_icn_p_uart4, SEC_RIFSC(UART4)),
	STM32_GATE_CFG(CK_BUS_UART5, ck_icn_p_uart5, SEC_RIFSC(UART5)),
	STM32_GATE_CFG(CK_BUS_I2C1, ck_icn_p_i2c1, SEC_RIFSC(I2C1)),
	STM32_GATE_CFG(CK_BUS_I2C2, ck_icn_p_i2c2, SEC_RIFSC(I2C2)),
	STM32_GATE_CFG(CK_BUS_I2C3, ck_icn_p_i2c3, SEC_RIFSC(I2C3)),
	STM32_GATE_CFG(CK_BUS_I2C4, ck_icn_p_i2c4, SEC_RIFSC(I2C4)),
	STM32_GATE_CFG(CK_BUS_I2C5, ck_icn_p_i2c5, SEC_RIFSC(I2C5)),
	STM32_GATE_CFG(CK_BUS_I2C6, ck_icn_p_i2c6, SEC_RIFSC(I2C6)),
	STM32_GATE_CFG(CK_BUS_I2C7, ck_icn_p_i2c7, SEC_RIFSC(I2C7)),
	STM32_GATE_CFG(CK_BUS_I3C1, ck_icn_p_i3c1, SEC_RIFSC(I3C1)),
	STM32_GATE_CFG(CK_BUS_I3C2, ck_icn_p_i3c2, SEC_RIFSC(I3C2)),
	STM32_GATE_CFG(CK_BUS_I3C3, ck_icn_p_i3c3, SEC_RIFSC(I3C3)),
	STM32_GATE_CFG(CK_BUS_TIM1, ck_icn_p_tim1, SEC_RIFSC(TIM1)),
	STM32_GATE_CFG(CK_BUS_TIM8, ck_icn_p_tim8, SEC_RIFSC(TIM8)),
	STM32_GATE_CFG(CK_BUS_TIM15, ck_icn_p_tim15, SEC_RIFSC(TIM15)),
	STM32_GATE_CFG(CK_BUS_TIM16, ck_icn_p_tim16, SEC_RIFSC(TIM16)),
	STM32_GATE_CFG(CK_BUS_TIM17, ck_icn_p_tim17, SEC_RIFSC(TIM17)),
	STM32_GATE_CFG(CK_BUS_TIM20, ck_icn_p_tim20, SEC_RIFSC(TIM20)),
	STM32_GATE_CFG(CK_BUS_SAI1, ck_icn_p_sai1, SEC_RIFSC(SAI1)),
	STM32_GATE_CFG(CK_BUS_SAI2, ck_icn_p_sai2, SEC_RIFSC(SAI2)),
	STM32_GATE_CFG(CK_BUS_SAI3, ck_icn_p_sai3, SEC_RIFSC(SAI3)),
	STM32_GATE_CFG(CK_BUS_SAI4, ck_icn_p_sai4, SEC_RIFSC(SAI4)),
	STM32_GATE_CFG(CK_BUS_USART1, ck_icn_p_usart1, SEC_RIFSC(USART1)),
	STM32_GATE_CFG(CK_BUS_USART6, ck_icn_p_usart6, SEC_RIFSC(USART6)),
	STM32_GATE_CFG(CK_BUS_UART7, ck_icn_p_uart7, SEC_RIFSC(UART7)),
	STM32_GATE_CFG(CK_BUS_UART8, ck_icn_p_uart8, SEC_RIFSC(UART8)),
	STM32_GATE_CFG(CK_BUS_UART9, ck_icn_p_uart9, SEC_RIFSC(UART9)),
	STM32_GATE_CFG(CK_BUS_FDCAN, ck_icn_p_fdcan, SEC_RIFSC(FDCAN)),
	STM32_GATE_CFG(CK_BUS_SPI1, ck_icn_p_spi1, SEC_RIFSC(SPI1)),
	STM32_GATE_CFG(CK_BUS_SPI4, ck_icn_p_spi4, SEC_RIFSC(SPI4)),
	STM32_GATE_CFG(CK_BUS_SPI5, ck_icn_p_spi5, SEC_RIFSC(SPI5)),
	STM32_GATE_CFG(CK_BUS_SPI6, ck_icn_p_spi6, SEC_RIFSC(SPI6)),
	STM32_GATE_CFG(CK_BUS_SPI7, ck_icn_p_spi7, SEC_RIFSC(SPI7)),
	STM32_GATE_CFG(CK_BUS_IWDG1, ck_icn_p_iwdg1, SEC_RIFSC(IWDG1)),
	STM32_GATE_CFG(CK_BUS_IWDG2, ck_icn_p_iwdg2, SEC_RIFSC(IWDG2)),
	STM32_GATE_CFG(CK_BUS_IWDG3, ck_icn_p_iwdg3, SEC_RIFSC(IWDG3)),
	STM32_GATE_CFG(CK_BUS_IWDG4, ck_icn_p_iwdg4, SEC_RIFSC(IWDG4)),
	STM32_GATE_CFG(CK_BUS_WWDG1, ck_icn_p_wwdg1, SEC_RIFSC(WWDG1)),
	STM32_GATE_CFG(CK_BUS_VREF, ck_icn_p_vref, SEC_RIFSC(VREFBUF)),
	STM32_GATE_CFG(CK_BUS_SERC, ck_icn_p_serc, SEC_RIFSC(SERC)),
	STM32_GATE_CFG(CK_BUS_HDP, ck_icn_p_hdp, SEC_RIFSC(HDP)),
	STM32_GATE_CFG(CK_BUS_IS2M, ck_icn_p_is2m, SEC_RIFRCC(IS2M)),
	STM32_GATE_CFG(CK_BUS_DSI, ck_icn_p_dsi, SEC_RIFSC(DSI_CMN)),
	STM32_GATE_CFG(CK_BUS_LTDC, ck_icn_p_ltdc, SEC_RIFSC(LTDC_CMN)),
	STM32_GATE_CFG(CK_BUS_CSI, ck_icn_p_csi, SEC_RIFSC(CSI)),
	STM32_GATE_CFG(CK_BUS_DCMIPP, ck_icn_p_dcmipp, SEC_RIFSC(DCMIPP)),
	STM32_GATE_CFG(CK_BUS_LVDS, ck_icn_p_lvds, SEC_RIFSC(LVDS)),
	STM32_GATE_CFG(CK_BUS_USBTC, ck_icn_p_usbtc, SEC_RIFSC(UCPD1)),
	STM32_GATE_CFG(CK_BUS_USB3PCIEPHY, ck_icn_p_usb3pciephy, SEC_RIFSC(USB3DR)),
	STM32_GATE_CFG(CK_BUS_VDEC, ck_icn_p_vdec, SEC_RIFSC(VDEC)),
	STM32_GATE_CFG(CK_BUS_VENC, ck_icn_p_venc, SEC_RIFSC(VENC)),
	STM32_GATE_CFG(CK_KER_TIM2, ck_ker_tim2, SEC_RIFSC(TIM2)),
	STM32_GATE_CFG(CK_KER_TIM3, ck_ker_tim3, SEC_RIFSC(TIM3)),
	STM32_GATE_CFG(CK_KER_TIM4, ck_ker_tim4, SEC_RIFSC(TIM4)),
	STM32_GATE_CFG(CK_KER_TIM5, ck_ker_tim5, SEC_RIFSC(TIM5)),
	STM32_GATE_CFG(CK_KER_TIM6, ck_ker_tim6, SEC_RIFSC(TIM6)),
	STM32_GATE_CFG(CK_KER_TIM7, ck_ker_tim7, SEC_RIFSC(TIM7)),
	STM32_GATE_CFG(CK_KER_TIM10, ck_ker_tim10, SEC_RIFSC(TIM10)),
	STM32_GATE_CFG(CK_KER_TIM11, ck_ker_tim11, SEC_RIFSC(TIM11)),
	STM32_GATE_CFG(CK_KER_TIM12, ck_ker_tim12, SEC_RIFSC(TIM12)),
	STM32_GATE_CFG(CK_KER_TIM13, ck_ker_tim13, SEC_RIFSC(TIM13)),
	STM32_GATE_CFG(CK_KER_TIM14, ck_ker_tim14, SEC_RIFSC(TIM14)),
	STM32_GATE_CFG(CK_KER_TIM1, ck_ker_tim1, SEC_RIFSC(TIM1)),
	STM32_GATE_CFG(CK_KER_TIM8, ck_ker_tim8, SEC_RIFSC(TIM8)),
	STM32_GATE_CFG(CK_KER_TIM15, ck_ker_tim15, SEC_RIFSC(TIM15)),
	STM32_GATE_CFG(CK_KER_TIM16, ck_ker_tim16, SEC_RIFSC(TIM16)),
	STM32_GATE_CFG(CK_KER_TIM17, ck_ker_tim17, SEC_RIFSC(TIM17)),
	STM32_GATE_CFG(CK_KER_TIM20, ck_ker_tim20, SEC_RIFSC(TIM20)),
	STM32_GATE_CFG(CK_KER_LPTIM1, ck_ker_lptim1, SEC_RIFSC(LPTIM1)),
	STM32_GATE_CFG(CK_KER_LPTIM2, ck_ker_lptim2, SEC_RIFSC(LPTIM2)),
	STM32_GATE_CFG(CK_KER_USART2, ck_ker_usart2, SEC_RIFSC(USART2)),
	STM32_GATE_CFG(CK_KER_UART4, ck_ker_uart4, SEC_RIFSC(UART4)),
	STM32_GATE_CFG(CK_KER_USART3, ck_ker_usart3, SEC_RIFSC(USART3)),
	STM32_GATE_CFG(CK_KER_UART5, ck_ker_uart5, SEC_RIFSC(UART5)),
	STM32_GATE_CFG(CK_KER_SPI2, ck_ker_spi2, SEC_RIFSC(SPI2)),
	STM32_GATE_CFG(CK_KER_SPI3, ck_ker_spi3, SEC_RIFSC(SPI3)),
	STM32_GATE_CFG(CK_KER_SPDIFRX, ck_ker_spdifrx, SEC_RIFSC(SPDIFRX)),
	STM32_GATE_CFG(CK_KER_I2C1, ck_ker_i2c1, SEC_RIFSC(I2C1)),
	STM32_GATE_CFG(CK_KER_I2C2, ck_ker_i2c2, SEC_RIFSC(I2C2)),
	STM32_GATE_CFG(CK_KER_I3C1, ck_ker_i3c1, SEC_RIFSC(I3C1)),
	STM32_GATE_CFG(CK_KER_I3C2, ck_ker_i3c2, SEC_RIFSC(I3C2)),
	STM32_GATE_CFG(CK_KER_I2C3, ck_ker_i2c3, SEC_RIFSC(I2C3)),
	STM32_GATE_CFG(CK_KER_I2C5, ck_ker_i2c5, SEC_RIFSC(I2C5)),
	STM32_GATE_CFG(CK_KER_I3C3, ck_ker_i3c3, SEC_RIFSC(I3C3)),
	STM32_GATE_CFG(CK_KER_I2C4, ck_ker_i2c4, SEC_RIFSC(I2C4)),
	STM32_GATE_CFG(CK_KER_I2C6, ck_ker_i2c6, SEC_RIFSC(I2C6)),
	STM32_GATE_CFG(CK_KER_I2C7, ck_ker_i2c7, SEC_RIFSC(I2C7)),
	STM32_GATE_CFG(CK_KER_SPI1, ck_ker_spi1, SEC_RIFSC(SPI1)),
	STM32_GATE_CFG(CK_KER_SPI4, ck_ker_spi4, SEC_RIFSC(SPI4)),
	STM32_GATE_CFG(CK_KER_SPI5, ck_ker_spi5, SEC_RIFSC(SPI5)),
	STM32_GATE_CFG(CK_KER_SPI6, ck_ker_spi6, SEC_RIFSC(SPI6)),
	STM32_GATE_CFG(CK_KER_SPI7, ck_ker_spi7, SEC_RIFSC(SPI7)),
	STM32_GATE_CFG(CK_KER_USART1, ck_ker_usart1, SEC_RIFSC(USART1)),
	STM32_GATE_CFG(CK_KER_USART6, ck_ker_usart6, SEC_RIFSC(USART6)),
	STM32_GATE_CFG(CK_KER_UART7, ck_ker_uart7, SEC_RIFSC(UART7)),
	STM32_GATE_CFG(CK_KER_UART8, ck_ker_uart8, SEC_RIFSC(UART8)),
	STM32_GATE_CFG(CK_KER_UART9, ck_ker_uart9, SEC_RIFSC(UART9)),
	STM32_GATE_CFG(CK_KER_MDF1, ck_ker_mdf1, SEC_RIFSC(MDF1)),
	STM32_GATE_CFG(CK_KER_SAI1, ck_ker_sai1, SEC_RIFSC(SAI1)),
	STM32_GATE_CFG(CK_KER_SAI2, ck_ker_sai2, SEC_RIFSC(SAI2)),
	STM32_GATE_CFG(CK_KER_SAI3, ck_ker_sai3, SEC_RIFSC(SAI3)),
	STM32_GATE_CFG(CK_KER_SAI4, ck_ker_sai4, SEC_RIFSC(SAI4)),
	STM32_GATE_CFG(CK_KER_FDCAN, ck_ker_fdcan, SEC_RIFSC(FDCAN)),
	STM32_GATE_CFG(CK_KER_CSI, ck_ker_csi, SEC_RIFSC(CSI)),
	STM32_GATE_CFG(CK_KER_CSITXESC, ck_ker_csitxesc, SEC_RIFSC(CSI)),
	STM32_GATE_CFG(CK_KER_CSIPHY, ck_ker_csiphy, SEC_RIFSC(CSI)),
	STM32_GATE_CFG(CK_KER_USBTC, ck_ker_usbtc, SEC_RIFSC(UCPD1)),
	STM32_GATE_CFG(CK_KER_I3C4, ck_ker_i3c4, SEC_RIFSC(I3C4)),
	STM32_GATE_CFG(CK_KER_SPI8, ck_ker_spi8, SEC_RIFSC(SPI8)),
	STM32_GATE_CFG(CK_KER_I2C8, ck_ker_i2c8, SEC_RIFSC(I2C8)),
	STM32_GATE_CFG(CK_KER_LPUART1, ck_ker_lpuart1, SEC_RIFSC(LPUART1)),
	STM32_GATE_CFG(CK_KER_LPTIM3, ck_ker_lptim3, SEC_RIFSC(LPTIM3)),
	STM32_GATE_CFG(CK_KER_LPTIM4, ck_ker_lptim4, SEC_RIFSC(LPTIM4)),
	STM32_GATE_CFG(CK_KER_LPTIM5, ck_ker_lptim5, SEC_RIFSC(LPTIM5)),
	STM32_GATE_CFG(CK_KER_ADF1, ck_ker_adf1, SEC_RIFSC(ADF1)),
	STM32_GATE_CFG(CK_KER_SDMMC1, ck_ker_sdmmc1, SEC_RIFSC(SDMMC1)),
	STM32_GATE_CFG(CK_KER_SDMMC2, ck_ker_sdmmc2, SEC_RIFSC(SDMMC2)),
	STM32_GATE_CFG(CK_KER_SDMMC3, ck_ker_sdmmc3, SEC_RIFSC(SDMMC3)),
	STM32_GATE_CFG(CK_KER_ETH1, ck_ker_eth1, SEC_RIFSC(ETH1)),
	STM32_GATE_CFG(CK_ETH1_STP, ck_ker_eth1stp, SEC_RIFSC(ETH1)),
	STM32_GATE_CFG(CK_KER_ETHSW, ck_ker_ethsw, SEC_RIFSC(ETHSW_DEIP)),
	STM32_GATE_CFG(CK_KER_ETH2, ck_ker_eth2, SEC_RIFSC(ETH2)),
	STM32_GATE_CFG(CK_ETH2_STP, ck_ker_eth2stp, SEC_RIFSC(ETH2)),
	STM32_GATE_CFG(CK_KER_ETH1PTP, ck_ker_eth1ptp, SEC_RIFSC(ETH1)),
	STM32_GATE_CFG(CK_KER_ETH2PTP, ck_ker_eth2ptp, SEC_RIFSC(ETH2)),
	STM32_GATE_CFG(CK_BUS_GPU, ck_icn_m_gpu, SEC_RIFSC(GPU)),
	STM32_GATE_CFG(CK_KER_GPU, ck_ker_gpu, SEC_RIFSC(GPU)),
	STM32_GATE_CFG(CK_KER_ETHSWREF, ck_ker_ethswref, SEC_RIFSC(ETHSW_DEIP)),
	STM32_GATE_CFG(CK_BUS_ETHSWACMCFG, ck_icn_p_ethsw_acm_cfg, SEC_RIFSC(ETHSW_ACM_CFG)),
	STM32_GATE_CFG(CK_BUS_ETHSWACMMSG, ck_icn_p_ethsw_acm_msg, SEC_RIFSC(ETHSW_ACM_MSGBUF)),
	STM32_GATE_CFG(CK_ETH1_MAC, ck_ker_eth1mac, SEC_RIFSC(ETH1)),
	STM32_GATE_CFG(CK_ETH1_TX, ck_ker_eth1tx, SEC_RIFSC(ETH1)),
	STM32_GATE_CFG(CK_ETH1_RX, ck_ker_eth1rx, SEC_RIFSC(ETH1)),
	STM32_GATE_CFG(CK_ETH2_MAC, ck_ker_eth2mac, SEC_RIFSC(ETH2)),
	STM32_GATE_CFG(CK_ETH2_TX, ck_ker_eth2tx, SEC_RIFSC(ETH2)),
	STM32_GATE_CFG(CK_ETH2_RX, ck_ker_eth2rx, SEC_RIFSC(ETH2)),
	STM32_COMPOSITE_CFG(CK_MCO1, ck_mco1, SEC_RIFRCC(MCO1)),
	STM32_COMPOSITE_CFG(CK_MCO2, ck_mco2, SEC_RIFRCC(MCO2)),
	STM32_COMPOSITE_CFG(CK_KER_ADC12, ck_ker_adc12, SEC_RIFSC(ADC12)),
	STM32_COMPOSITE_CFG(CK_KER_ADC3, ck_ker_adc3, SEC_RIFSC(ADC3)),
	STM32_COMPOSITE_CFG(CK_KER_USB2PHY1, ck_ker_usb2phy1, SEC_RIFSC(USBH)),
	STM32_GATE_CFG(CK_KER_USB2PHY2, ck_ker_usb2phy2, SEC_RIFSC(USBH)),
	STM32_COMPOSITE_CFG(CK_KER_USB2PHY2EN, ck_ker_usb2phy2_en, SEC_RIFSC(USBH)),
	STM32_COMPOSITE_CFG(CK_KER_USB3PCIEPHY, ck_ker_usb3pciephy, SEC_RIFSC(USB3DR)),
	STM32_COMPOSITE_CFG(CK_KER_DSIBLANE, clk_lanebyte, SEC_RIFSC(DSI_CMN)),
	STM32_COMPOSITE_CFG(CK_KER_DSIPHY, clk_phy_dsi, SEC_RIFSC(DSI_CMN)),
	STM32_COMPOSITE_CFG(CK_KER_LVDSPHY, ck_ker_lvdsphy, SEC_RIFSC(LVDS)),
	STM32_COMPOSITE_CFG(CK_KER_DTS, ck_ker_dts, SEC_RIFSC(DTS)),
	STM32_GATE_CFG(CK_KER_LTDC, ck_ker_ltdc, SEC_RIFSC(LTDC_CMN)),
};

u16 stm32mp25_cpt_gate[GATE_NB];

#ifdef CONFIG_DEBUG_FS
static struct clock_summary clock_summary_mp25;
#endif

struct clk_stm32_clock_data stm32mp25_clock_data = {
	.gate_cpt	= stm32mp25_cpt_gate,
	.gates		= stm32mp25_gates,
	.muxes		= stm32mp25_muxes,
#ifdef CONFIG_DEBUG_FS
	.dividers	= stm32mp25_dividers,
#endif
};

static struct stm32_rcc_match_data stm32mp25_data = {
	.tab_clocks	= stm32mp25_clock_cfg,
	.num_clocks	= ARRAY_SIZE(stm32mp25_clock_cfg),
	.maxbinding	= STM32MP25_LAST_CLK,
	.clock_data	= &stm32mp25_clock_data,
	.check_security = &stm32mp25_check_security,
	.reset_us	= 2,
#ifdef CONFIG_DEBUG_FS
	.clock_summary	= &clock_summary_mp25,
#endif
};

static const struct of_device_id stm32mp25_match_data[] = {
	{
		.compatible = "st,stm32mp25-rcc",
		.data = &stm32mp25_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, stm32mp25_match_data);

static int stm32mp25_rcc_init(struct device *dev)
{
	void __iomem *base;
	int ret;

	base = of_iomap(dev_of_node(dev), 0);
	if (!base) {
		dev_err(dev, "%pOFn: unable to map resource", dev_of_node(dev));
		ret = -ENOMEM;
		goto out;
	}

	ret = stm32_rcc_init(dev, stm32mp25_match_data, base);

out:
	if (ret) {
		if (base)
			iounmap(base);

		of_node_put(dev_of_node(dev));
	}

	return ret;
}

static int get_clock_deps(struct device *dev)
{
	static const char * const clock_deps_name[] = {
		"hsi", "hse", "msi", "lsi", "lse",
	};
	size_t deps_size = sizeof(struct clk *) * ARRAY_SIZE(clock_deps_name);
	struct clk **clk_deps;
	int i;

	clk_deps = devm_kzalloc(dev, deps_size, GFP_KERNEL);
	if (!clk_deps)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(clock_deps_name); i++) {
		struct clk *clk;

		clk = of_clk_get_by_name(dev_of_node(dev), clock_deps_name[i]);

		if (IS_ERR(clk)) {
			if (PTR_ERR(clk) != -EINVAL && PTR_ERR(clk) != -ENOENT)
				return PTR_ERR(clk);
		} else {
			/* Device gets a reference count on the clock */
			clk_deps[i] = devm_clk_get(dev, __clk_get_name(clk));
			clk_put(clk);
		}
	}

	return 0;
}

static int stm32mp25_rcc_clocks_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = get_clock_deps(dev);

	if (!ret)
		ret = stm32mp25_rcc_init(dev);

	return ret;
}

static int stm32mp25_rcc_clocks_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *child, *np = dev_of_node(dev);

	for_each_available_child_of_node(np, child)
		of_clk_del_provider(child);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int stm32_rcc_suspend(struct device *dev)
{
	clk_save_context();

	return 0;
}

static int stm32_rcc_resume(struct device *dev)
{
	clk_restore_context();

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops stm32_rcc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stm32_rcc_suspend, stm32_rcc_resume)
};

static struct platform_driver stm32mp25_rcc_clocks_driver = {
	.driver	= {
		.name = "stm32mp25_rcc",
		.of_match_table = stm32mp25_match_data,
		.pm = &stm32_rcc_pm_ops,
	},
	.probe = stm32mp25_rcc_clocks_probe,
	.remove = stm32mp25_rcc_clocks_remove,
};

static int __init stm32mp25_clocks_init(void)
{
	return platform_driver_register(&stm32mp25_rcc_clocks_driver);
}

core_initcall(stm32mp25_clocks_init);

#ifdef CONFIG_DEBUG_FS

#include <linux/debugfs.h>

static const char * const pll_src[] = {
	"hsi_ck", "hse_ck", "msi_ck"
};

static const char * const flexgen_src[] = {
	"ck_pll4", "ck_pll5", "ck_pll6", "ck_pll7", "ck_pll8",
	"hsi_ck", "hse_ck", "msi_ck", "hsi_ck", "hse_ck", "msi_ck",
	"spdifsymb", "i2sckin", "lsi_ck", "lse_ck"
};

static unsigned long clk_summary_div_recalc_rate(struct clk_stm32_clock_data *data,
						 struct clk_summary *c,
						 unsigned long parent_rate)
{
	return stm32_divider_get_rate(data->base, data, c->div_id, parent_rate);
}

#define PARENT(_parent)	((const char *[]) { _parent})

#define CS_DIV(_name, _parent, _div) \
{\
	.name		= _name,\
	.nb_parents	= 1,\
	.parent_names	= PARENT(_parent),\
	.div_id		= _div,\
	.gate_id	= NO_STM32_GATE,\
	.mux_id		= NO_STM32_MUX,\
	.get_rate	= clk_summary_div_recalc_rate,\
}

#define CS_GATE(_name, _parent, _gate) \
{\
	.name		= _name,\
	.nb_parents	= 1,\
	.parent_names	= PARENT(_parent),\
	.gate_id	= _gate,\
	.mux_id		= NO_STM32_MUX,\
	.div_id		= NO_STM32_DIV,\
}

#define CS_GATEMUX(_name, _parents, _gate, _mux) \
{\
	.name		= _name,\
	.nb_parents	= ARRAY_SIZE(_parents),\
	.parent_names	= _parents,\
	.gate_id	= _gate,\
	.mux_id		= _mux,\
	.div_id		= NO_STM32_DIV,\
}

#define CS_GATEDIV(_name, _parent, _gate, _div) \
{\
	.name		= _name,\
	.nb_parents	= 1,\
	.parent_names	= PARENT(_parent),\
	.gate_id	= _gate,\
	.mux_id		= NO_STM32_MUX,\
	.div_id		= _div,\
}

static unsigned long clk_summary_clk_recalc_rate(struct clk_stm32_clock_data *data,
						 struct clk_summary *c,
						 unsigned long parent_rate)
{
	struct clk *clk = __clk_lookup(c->name);

	if (clk)
		return clk_get_rate(clk);

	return 0;
}

#define CS_OSC(_name, _gate) \
{\
	.name		= _name,\
	.nb_parents	= 0,\
	.gate_id	= _gate,\
	.mux_id		= NO_STM32_MUX,\
	.div_id		= NO_STM32_DIV,\
	.get_rate	= clk_summary_clk_recalc_rate,\
}

static unsigned long clk_summary_hsediv2_recalc_rate(struct clk_stm32_clock_data *data,
						     struct clk_summary *c,
						     unsigned long parent_rate)
{
	void __iomem *addr = data->base + RCC_OCENSETR;

	if ((readl(addr) & RCC_OCENSETR_HSEDIV2BYP) != 0U)
		return parent_rate;

	return parent_rate / 2;
}

#define CS_HSE_DIV2(_name, _parent, _gate) \
{\
	.name		= _name,\
	.nb_parents	= 1,\
	.parent_names	= PARENT(_parent),\
	.gate_id	= _gate,\
	.mux_id		= NO_STM32_MUX,\
	.div_id		= NO_STM32_DIV,\
	.get_rate	= clk_summary_hsediv2_recalc_rate,\
}

static bool clk_summary_cpu1_is_enabled(struct clk_stm32_clock_data *data,
					struct clk_summary *c)
{
	return true;
}

#define CS_CPU1(_name) \
{\
	.name		= _name,\
	.nb_parents	= 0,\
	.div_id		= NO_STM32_DIV,\
	.gate_id	= NO_STM32_GATE,\
	.mux_id		= NO_STM32_MUX,\
	.get_rate	= clk_summary_clk_recalc_rate,\
	.is_enabled	= clk_summary_cpu1_is_enabled,\
}

/* PLL configuration registers offsets from RCC_PLLxCFGR1 */
#define RCC_OFFSET_PLLXCFGR1	0x00
#define RCC_OFFSET_PLLXCFGR2	0x04
#define RCC_OFFSET_PLLXCFGR3	0x08
#define RCC_OFFSET_PLLXCFGR4	0x0C
#define RCC_OFFSET_PLLXCFGR5	0x10
#define RCC_OFFSET_PLLXCFGR6	0x18
#define RCC_OFFSET_PLLXCFGR7	0x1C

struct cs_pll {
	u32 offset;
};

static unsigned long clk_get_pll_fvco(struct clk_stm32_clock_data *data, u32 offset_base,
				      unsigned long prate)
{
	void __iomem *pllxcfgr1 = data->base + offset_base;
	void __iomem *pllxcfgr2 = pllxcfgr1 + RCC_OFFSET_PLLXCFGR2;
	void __iomem *pllxcfgr3 = pllxcfgr1 + RCC_OFFSET_PLLXCFGR3;
	unsigned long fvco = 0UL;
	u32 fracin, fbdiv, refdiv;

	fracin = readl(pllxcfgr3) & RCC_PLLxCFGR3_FRACIN_MASK;
	fbdiv = (readl(pllxcfgr2) & RCC_PLLxCFGR2_FBDIV_MASK) >>
		RCC_PLLxCFGR2_FBDIV_SHIFT;

	refdiv = readl(pllxcfgr2) & RCC_PLLxCFGR2_FREFDIV_MASK;

	if (fracin != 0U) {
		unsigned long long numerator, denominator;

		numerator = ((unsigned long long)fbdiv << 24) + fracin;
		fvco = prate * numerator;
		denominator = (unsigned long long)refdiv << 24;
		do_div(fvco, denominator);

	} else {
		fvco = (u64)prate * fbdiv;
		do_div(fvco, refdiv);
	}

	return fvco;
}

static unsigned long clk_summary_pll_frac_div_recalc_rate(struct clk_stm32_clock_data *data,
							  struct clk_summary *c,
							  unsigned long prate)
{
	struct cs_pll *cfg = c->data;
	void __iomem *pllxcfgr1 = data->base + cfg->offset;
	void __iomem *pllxcfgr4 = pllxcfgr1 + RCC_OFFSET_PLLXCFGR4;
	void __iomem *pllxcfgr6 = pllxcfgr1 + RCC_OFFSET_PLLXCFGR6;
	void __iomem *pllxcfgr7 = pllxcfgr1 + RCC_OFFSET_PLLXCFGR7;
	unsigned long dfout;
	u32 postdiv1, postdiv2;

	postdiv1 = readl(pllxcfgr6) & RCC_PLLxCFGR6_POSTDIV1_MASK;
	postdiv2 = readl(pllxcfgr7) & RCC_PLLxCFGR7_POSTDIV2_MASK;

	if ((readl(pllxcfgr4) & RCC_PLLxCFGR4_BYPASS) != 0U) {
		dfout = prate;
	} else {
		if (postdiv1 == 0U || postdiv2 == 0U)
			dfout = prate;
		else
			dfout = clk_get_pll_fvco(data, cfg->offset, prate) /
						 (postdiv1 * postdiv2);
	}

	return dfout;
}

#define CS_PLL(_name, _parents, _gate, _mux, _offset)\
{\
	.name		= _name,\
	.nb_parents	= ARRAY_SIZE(_parents),\
	.parent_names	= _parents,\
	.gate_id	= _gate,\
	.mux_id		= _mux,\
	.div_id		= NO_STM32_DIV,\
	.data		=  &(struct cs_pll) {\
		.offset		= _offset,\
	},\
	.get_rate	= clk_summary_pll_frac_div_recalc_rate,\
}

struct cs_flexgen {
	u32 channel;
};

static bool clk_summary_flexgen_is_enabled(struct clk_stm32_clock_data *data,
					   struct clk_summary *c)
{
	struct cs_flexgen *cfg = c->data;

	return !!(readl(data->base + RCC_FINDIV0CFGR + (0x4 * cfg->channel)) &
		RCC_FINDIVxCFGR_FINDIVxEN);
}

static u8 clk_summary_flexgen_get_parent(struct clk_stm32_clock_data *data,
					 struct clk_summary *c)
{
	struct cs_flexgen *cfg = c->data;
	void __iomem *address = data->base + RCC_XBAR0CFGR + (cfg->channel * 4);

	return readl(address) & RCC_XBARxCFGR_XBARxSEL_MASK;
}

static unsigned long clk_summary_flexgen_recalc_rate(struct clk_stm32_clock_data *data,
						     struct clk_summary *c,
						     unsigned long prate)
{
	struct cs_flexgen *cfg = c->data;
	u8 channel = cfg->channel;
	u32 prediv, findiv;
	unsigned long freq = prate;

	prediv = readl(data->base + RCC_PREDIV0CFGR + (0x4 * channel)) &
		       RCC_PREDIVxCFGR_PREDIVx_MASK;

	findiv = readl(data->base + RCC_FINDIV0CFGR + (0x4 * channel)) &
		       RCC_FINDIVxCFGR_FINDIVx_MASK;

	if (freq == 0)
		return 0;

	switch (prediv) {
	case 0x0:
		break;

	case 0x1:
		freq /= 2;
		break;

	case 0x3:
		freq /= 4;
		break;

	case 0x3FF:
		freq /= 1024;
		break;
	}

	freq /= (findiv + 1);

	return freq;
}

#define CS_FLEXGEN(_name, _channel)\
{\
	.name		= _name,\
	.nb_parents	= ARRAY_SIZE(flexgen_src),\
	.parent_names	= flexgen_src,\
	.gate_id	= NO_STM32_GATE,\
	.mux_id		= NO_STM32_MUX,\
	.div_id		= NO_STM32_DIV,\
	.data		=  &(struct cs_flexgen) {\
		.channel		= _channel,\
	},\
	.is_enabled	= clk_summary_flexgen_is_enabled,\
	.get_parent	= clk_summary_flexgen_get_parent,\
	.get_rate	= clk_summary_flexgen_recalc_rate,\
}

struct cs_stm32_timer {
	u32 apbdiv;
	u32 timpre;
};

#define APB_DIV_MASK 0x07
#define TIM_PRE_MASK 0x01

static unsigned long clk_stm32_timer_recalc_rate(struct clk_stm32_clock_data *data,
						 struct clk_summary *c,
						 unsigned long parent_rate)
{
	struct cs_stm32_timer *tim = c->data;
	u32 prescaler, timpre;

	prescaler = readl(data->base + tim->apbdiv) & APB_DIV_MASK;

	timpre = readl(data->base + tim->timpre) & TIM_PRE_MASK;

	if (prescaler == 0U)
		return parent_rate;

	return parent_rate * (timpre + 1U) * 2U;
}

#define CS_STM32_TIMER(_name, _parent, _apbdiv, _timpre) \
{\
	.name		= _name,\
	.nb_parents	= 1,\
	.parent_names	= PARENT(_parent),\
	.div_id		= NO_STM32_DIV,\
	.gate_id	= NO_STM32_GATE,\
	.mux_id		= NO_STM32_MUX,\
	.data		=  &(struct cs_stm32_timer) {\
		.apbdiv		= _apbdiv,\
		.timpre		= _timpre,\
	},\
	.get_rate	= clk_stm32_timer_recalc_rate,\
}

static const char * const rtc_src[] = {
	"off", "hse_ck", "lsi_ck", "ck_hse_rtc"
};

static struct clk_summary stm32mp25_clock_summary[] = {
	CS_OSC("hsi_ck", GATE_HSI),
	CS_OSC("lsi_ck", GATE_LSI),
	CS_OSC("msi_ck", GATE_MSI),
	CS_OSC("hse_ck", GATE_HSE),
	CS_OSC("lse_ck", GATE_LSE),

	CS_HSE_DIV2("hse_div2_ck", "hse_ck", GATE_HSEDIV2),
	CS_DIV("ck_hse_rtc", "hse_ck", DIV_RTC),

	CS_PLL("ck_pll2", pll_src, GATE_PLL2, MUX_MUXSEL6, RCC_PLL2CFGR1),
	CS_PLL("ck_pll3", pll_src, GATE_PLL3, MUX_MUXSEL7, RCC_PLL3CFGR1),
	CS_PLL("ck_pll4", pll_src, GATE_PLL4, MUX_MUXSEL0, RCC_PLL4CFGR1),
	CS_PLL("ck_pll5", pll_src, GATE_PLL5, MUX_MUXSEL1, RCC_PLL5CFGR1),
	CS_PLL("ck_pll6", pll_src, GATE_PLL6, MUX_MUXSEL2, RCC_PLL6CFGR1),
	CS_PLL("ck_pll7", pll_src, GATE_PLL7, MUX_MUXSEL3, RCC_PLL7CFGR1),
	CS_PLL("ck_pll8", pll_src, GATE_PLL8, MUX_MUXSEL4, RCC_PLL8CFGR1),

	CS_CPU1("ck_cpu1"),

	CS_FLEXGEN("ck_icn_hs_mcu", 0),

	CS_DIV("ck_icn_ls_mcu", "ck_icn_hs_mcu", DIV_LSMCU),

	CS_FLEXGEN("ck_icn_sdmmc", 1),
	CS_FLEXGEN("ck_icn_ddr", 2),
	CS_FLEXGEN("ck_icn_display", 3),
	CS_FLEXGEN("ck_icn_hsl", 4),
	CS_FLEXGEN("ck_icn_nic", 5),
	CS_FLEXGEN("ck_icn_vid", 6),
	CS_FLEXGEN("ck_flexgen_07", 7),
	CS_FLEXGEN("ck_flexgen_08", 8),
	CS_FLEXGEN("ck_flexgen_09", 9),
	CS_FLEXGEN("ck_flexgen_10", 10),
	CS_FLEXGEN("ck_flexgen_11", 11),
	CS_FLEXGEN("ck_flexgen_12", 12),
	CS_FLEXGEN("ck_flexgen_13", 13),
	CS_FLEXGEN("ck_flexgen_14", 14),
	CS_FLEXGEN("ck_flexgen_15", 15),
	CS_FLEXGEN("ck_flexgen_16", 16),
	CS_FLEXGEN("ck_flexgen_17", 17),
	CS_FLEXGEN("ck_flexgen_18", 18),
	CS_FLEXGEN("ck_flexgen_19", 19),
	CS_FLEXGEN("ck_flexgen_20", 20),
	CS_FLEXGEN("ck_flexgen_21", 21),
	CS_FLEXGEN("ck_flexgen_22", 22),
	CS_FLEXGEN("ck_flexgen_23", 23),
	CS_FLEXGEN("ck_flexgen_24", 24),
	CS_FLEXGEN("ck_flexgen_25", 25),
	CS_FLEXGEN("ck_flexgen_26", 26),
	CS_FLEXGEN("ck_flexgen_27", 27),
	CS_FLEXGEN("ck_flexgen_28", 28),
	CS_FLEXGEN("ck_flexgen_29", 29),
	CS_FLEXGEN("ck_flexgen_30", 30),
	CS_FLEXGEN("ck_flexgen_31", 31),
	CS_FLEXGEN("ck_flexgen_32", 32),
	CS_FLEXGEN("ck_flexgen_33", 33),
	CS_FLEXGEN("ck_flexgen_34", 34),
	CS_FLEXGEN("ck_flexgen_35", 35),
	CS_FLEXGEN("ck_flexgen_36", 36),
	CS_FLEXGEN("ck_flexgen_37", 37),
	CS_FLEXGEN("ck_flexgen_38", 38),
	CS_FLEXGEN("ck_flexgen_39", 39),
	CS_FLEXGEN("ck_flexgen_40", 40),
	CS_FLEXGEN("ck_flexgen_41", 41),
	CS_FLEXGEN("ck_flexgen_42", 42),
	CS_FLEXGEN("ck_flexgen_43", 43),
	CS_FLEXGEN("ck_flexgen_44", 44),
	CS_FLEXGEN("ck_flexgen_45", 45),
	CS_FLEXGEN("ck_flexgen_46", 46),
	CS_FLEXGEN("ck_flexgen_47", 47),
	CS_FLEXGEN("ck_flexgen_48", 48),
	CS_FLEXGEN("ck_flexgen_49", 49),
	CS_FLEXGEN("ck_flexgen_50", 50),
	CS_FLEXGEN("ck_flexgen_51", 51),
	CS_FLEXGEN("ck_flexgen_52", 52),
	CS_FLEXGEN("ck_flexgen_53", 53),
	CS_FLEXGEN("ck_flexgen_54", 54),
	CS_FLEXGEN("ck_flexgen_55", 55),
	CS_FLEXGEN("ck_flexgen_56", 56),
	CS_FLEXGEN("ck_flexgen_57", 57),
	CS_FLEXGEN("ck_flexgen_58", 58),
	CS_FLEXGEN("ck_flexgen_59", 59),
	CS_FLEXGEN("ck_flexgen_60", 60),
	CS_FLEXGEN("ck_flexgen_61", 61),
	CS_FLEXGEN("ck_flexgen_62", 62),
	CS_FLEXGEN("ck_flexgen_63", 63),

	CS_DIV("ck_icn_apb1", "ck_icn_ls_mcu", DIV_APB1),
	CS_DIV("ck_icn_apb2", "ck_icn_ls_mcu", DIV_APB2),
	CS_DIV("ck_icn_apb3", "ck_icn_ls_mcu", DIV_APB3),
	CS_DIV("ck_icn_apb4", "ck_icn_ls_mcu", DIV_APB4),
	CS_GATEDIV("ck_icn_apbdbg", "ck_icn_ls_mcu", GATE_DBG, DIV_APBDBG),

	CS_STM32_TIMER("ck_timg1", "ck_icn_apb1", RCC_APB1DIVR, RCC_TIMG1PRER),
	CS_STM32_TIMER("ck_timg2", "ck_icn_apb2", RCC_APB2DIVR, RCC_TIMG2PRER),

	CS_GATE("ck_icn_s_sysram", "ck_icn_hs_mcu", GATE_SYSRAM),
	CS_GATE("ck_icn_s_vderam", "ck_icn_hs_mcu", GATE_VDERAM),
	CS_GATE("ck_icn_s_retram", "ck_icn_hs_mcu", GATE_RETRAM),
	CS_GATE("ck_icn_s_sram1", "ck_icn_hs_mcu", GATE_SRAM1),
	CS_GATE("ck_icn_s_sram2", "ck_icn_hs_mcu", GATE_SRAM2),
	CS_GATE("ck_icn_s_ospi1", "ck_icn_hs_mcu", GATE_OSPI1),
	CS_GATE("ck_icn_s_ospi2", "ck_icn_hs_mcu", GATE_OSPI2),
	CS_GATE("ck_icn_p_otfd1", "ck_icn_hs_mcu", GATE_OSPI1),
	CS_GATE("ck_icn_p_otfd2", "ck_icn_hs_mcu", GATE_OSPI2),
	CS_GATE("ck_icn_s_bkpsram", "ck_icn_ls_mcu", GATE_BKPSRAM),
	CS_GATE("ck_icn_p_ddrphyc", "ck_icn_ls_mcu", GATE_DDRPHYCAPB),
	CS_GATE("ck_icn_p_syscpu1", "ck_icn_ls_mcu", GATE_SYSCPU1),
	CS_GATE("ck_icn_p_hpdma1", "ck_icn_ls_mcu", GATE_HPDMA1),
	CS_GATE("ck_icn_p_hpdma2", "ck_icn_ls_mcu", GATE_HPDMA2),
	CS_GATE("ck_icn_p_hpdma3", "ck_icn_ls_mcu", GATE_HPDMA3),
	CS_GATE("ck_icn_p_ipcc1", "ck_icn_ls_mcu", GATE_IPCC1),
	CS_GATE("ck_icn_p_ipcc2", "ck_icn_ls_mcu", GATE_IPCC2),
	CS_GATE("ck_icn_p_cci", "ck_icn_ls_mcu", GATE_CCI),
	CS_GATE("ck_icn_p_crc", "ck_icn_ls_mcu", GATE_CRC),
	CS_GATE("ck_icn_p_ospiiom", "ck_icn_ls_mcu", GATE_OSPIIOM),
	CS_GATE("ck_icn_p_hash", "ck_icn_ls_mcu", GATE_HASH),
	CS_GATE("ck_icn_p_rng", "ck_icn_ls_mcu", GATE_RNG),
	CS_GATE("ck_icn_p_cryp1", "ck_icn_ls_mcu", GATE_CRYP1),
	CS_GATE("ck_icn_p_cryp2", "ck_icn_ls_mcu", GATE_CRYP2),
	CS_GATE("ck_icn_p_saes", "ck_icn_ls_mcu", GATE_SAES),
	CS_GATE("ck_icn_p_pka", "ck_icn_ls_mcu", GATE_PKA),
	CS_GATE("ck_icn_p_gpioa", "ck_icn_ls_mcu", GATE_GPIOA),
	CS_GATE("ck_icn_p_gpiob", "ck_icn_ls_mcu", GATE_GPIOB),
	CS_GATE("ck_icn_p_gpioc", "ck_icn_ls_mcu", GATE_GPIOC),
	CS_GATE("ck_icn_p_gpiod", "ck_icn_ls_mcu", GATE_GPIOD),
	CS_GATE("ck_icn_p_gpioe", "ck_icn_ls_mcu", GATE_GPIOE),
	CS_GATE("ck_icn_p_gpiof", "ck_icn_ls_mcu", GATE_GPIOF),
	CS_GATE("ck_icn_p_gpiog", "ck_icn_ls_mcu", GATE_GPIOG),
	CS_GATE("ck_icn_p_gpioh", "ck_icn_ls_mcu", GATE_GPIOH),
	CS_GATE("ck_icn_p_gpioi", "ck_icn_ls_mcu", GATE_GPIOI),
	CS_GATE("ck_icn_p_gpioj", "ck_icn_ls_mcu", GATE_GPIOJ),
	CS_GATE("ck_icn_p_gpiok", "ck_icn_ls_mcu", GATE_GPIOK),
	CS_GATE("ck_icn_s_lpsram1", "ck_icn_ls_mcu", GATE_LPSRAM1),
	CS_GATE("ck_icn_s_lpsram2", "ck_icn_ls_mcu", GATE_LPSRAM2),
	CS_GATE("ck_icn_s_lpsram3", "ck_icn_ls_mcu", GATE_LPSRAM3),
	CS_GATE("ck_icn_p_gpioz", "ck_icn_ls_mcu", GATE_GPIOZ),
	CS_GATE("ck_icn_p_lpdma", "ck_icn_ls_mcu", GATE_LPDMA),
	CS_GATE("ck_icn_p_adf1", "ck_icn_ls_mcu", GATE_ADF1),
	CS_GATE("ck_icn_p_hsem", "ck_icn_ls_mcu", GATE_HSEM),
	CS_GATE("ck_icn_p_rtc", "ck_icn_ls_mcu", GATE_RTC),
	CS_GATE("ck_icn_p_iwdg5", "ck_icn_ls_mcu", GATE_IWDG5),
	CS_GATE("ck_icn_p_wwdg2", "ck_icn_ls_mcu", GATE_WWDG2),
	CS_GATE("ck_icn_s_stm", "ck_icn_ls_mcu", GATE_STM),
	CS_GATE("ck_icn_p_fmc", "ck_icn_ls_mcu", GATE_FMC),
	CS_GATE("ck_icn_p_eth1", "ck_icn_ls_mcu", GATE_ETH1),
	CS_GATE("ck_icn_p_ethsw", "ck_icn_ls_mcu", GATE_ETHSWMAC),
	CS_GATE("ck_icn_p_eth2", "ck_icn_ls_mcu", GATE_ETH2),
	CS_GATE("ck_icn_p_pcie", "ck_icn_ls_mcu", GATE_PCIE),
	CS_GATE("ck_icn_p_adc12", "ck_icn_ls_mcu", GATE_ADC12),
	CS_GATE("ck_icn_p_adc3", "ck_icn_ls_mcu", GATE_ADC3),
	CS_GATE("ck_icn_p_mdf1", "ck_icn_ls_mcu", GATE_MDF1),
	CS_GATE("ck_icn_p_spi8", "ck_icn_ls_mcu", GATE_SPI8),
	CS_GATE("ck_icn_p_lpuart1", "ck_icn_ls_mcu", GATE_LPUART1),
	CS_GATE("ck_icn_p_i2c8", "ck_icn_ls_mcu", GATE_I2C8),
	CS_GATE("ck_icn_p_lptim3", "ck_icn_ls_mcu", GATE_LPTIM3),
	CS_GATE("ck_icn_p_lptim4", "ck_icn_ls_mcu", GATE_LPTIM4),
	CS_GATE("ck_icn_p_lptim5", "ck_icn_ls_mcu", GATE_LPTIM5),
	CS_GATE("ck_icn_p_risaf4", "ck_icn_ls_mcu", GATE_DDRCP),
	CS_GATE("ck_icn_m_sdmmc1", "ck_icn_sdmmc", GATE_SDMMC1),
	CS_GATE("ck_icn_m_sdmmc2", "ck_icn_sdmmc", GATE_SDMMC2),
	CS_GATE("ck_icn_m_sdmmc3", "ck_icn_sdmmc", GATE_SDMMC3),
	CS_GATE("ck_icn_s_ddr", "ck_icn_ddr", GATE_DDRCP),
	CS_GATE("ck_icn_m_usb2ohci", "ck_icn_hsl", GATE_USB2),
	CS_GATE("ck_icn_m_usb2ehci", "ck_icn_hsl", GATE_USB2),
	CS_GATE("ck_icn_m_usb3dr", "ck_icn_hsl", GATE_USB3DR),
	CS_GATE("ck_icn_p_tim2", "ck_icn_apb1", GATE_TIM2),
	CS_GATE("ck_icn_p_tim3", "ck_icn_apb1", GATE_TIM3),
	CS_GATE("ck_icn_p_tim4", "ck_icn_apb1", GATE_TIM4),
	CS_GATE("ck_icn_p_tim5", "ck_icn_apb1", GATE_TIM5),
	CS_GATE("ck_icn_p_tim6", "ck_icn_apb1", GATE_TIM6),
	CS_GATE("ck_icn_p_tim7", "ck_icn_apb1", GATE_TIM7),
	CS_GATE("ck_icn_p_tim10", "ck_icn_apb1", GATE_TIM10),
	CS_GATE("ck_icn_p_tim11", "ck_icn_apb1", GATE_TIM11),
	CS_GATE("ck_icn_p_tim12", "ck_icn_apb1", GATE_TIM12),
	CS_GATE("ck_icn_p_tim13", "ck_icn_apb1", GATE_TIM13),
	CS_GATE("ck_icn_p_tim14", "ck_icn_apb1", GATE_TIM14),
	CS_GATE("ck_icn_p_lptim1", "ck_icn_apb1", GATE_LPTIM1),
	CS_GATE("ck_icn_p_lptim2", "ck_icn_apb1", GATE_LPTIM2),
	CS_GATE("ck_icn_p_spi2", "ck_icn_apb1", GATE_SPI2),
	CS_GATE("ck_icn_p_spi3", "ck_icn_apb1", GATE_SPI3),
	CS_GATE("ck_icn_p_spdifrx", "ck_icn_apb1", GATE_SPDIFRX),
	CS_GATE("ck_icn_p_usart2", "ck_icn_apb1", GATE_USART2),
	CS_GATE("ck_icn_p_usart3", "ck_icn_apb1", GATE_USART3),
	CS_GATE("ck_icn_p_uart4", "ck_icn_apb1", GATE_UART4),
	CS_GATE("ck_icn_p_uart5", "ck_icn_apb1", GATE_UART5),
	CS_GATE("ck_icn_p_i2c1", "ck_icn_apb1", GATE_I2C1),
	CS_GATE("ck_icn_p_i2c2", "ck_icn_apb1", GATE_I2C2),
	CS_GATE("ck_icn_p_i2c3", "ck_icn_apb1", GATE_I2C3),
	CS_GATE("ck_icn_p_i2c4", "ck_icn_apb1", GATE_I2C4),
	CS_GATE("ck_icn_p_i2c5", "ck_icn_apb1", GATE_I2C5),
	CS_GATE("ck_icn_p_i2c6", "ck_icn_apb1", GATE_I2C6),
	CS_GATE("ck_icn_p_i2c7", "ck_icn_apb1", GATE_I2C7),
	CS_GATE("ck_icn_p_i3c1", "ck_icn_apb1", GATE_I3C1),
	CS_GATE("ck_icn_p_i3c2", "ck_icn_apb1", GATE_I3C2),
	CS_GATE("ck_icn_p_i3c3", "ck_icn_apb1", GATE_I3C3),
	CS_GATE("ck_icn_p_i3c4", "ck_icn_ls_mcu", GATE_I3C4),
	CS_GATE("ck_icn_p_tim1", "ck_icn_apb2", GATE_TIM1),
	CS_GATE("ck_icn_p_tim8", "ck_icn_apb2", GATE_TIM8),
	CS_GATE("ck_icn_p_tim15", "ck_icn_apb2", GATE_TIM15),
	CS_GATE("ck_icn_p_tim16", "ck_icn_apb2", GATE_TIM16),
	CS_GATE("ck_icn_p_tim17", "ck_icn_apb2", GATE_TIM17),
	CS_GATE("ck_icn_p_tim20", "ck_icn_apb2", GATE_TIM20),
	CS_GATE("ck_icn_p_sai1", "ck_icn_apb2", GATE_SAI1),
	CS_GATE("ck_icn_p_sai2", "ck_icn_apb2", GATE_SAI2),
	CS_GATE("ck_icn_p_sai3", "ck_icn_apb2", GATE_SAI3),
	CS_GATE("ck_icn_p_sai4", "ck_icn_apb2", GATE_SAI4),
	CS_GATE("ck_icn_p_usart1", "ck_icn_apb2", GATE_USART1),
	CS_GATE("ck_icn_p_usart6", "ck_icn_apb2", GATE_USART6),
	CS_GATE("ck_icn_p_uart7", "ck_icn_apb2", GATE_UART7),
	CS_GATE("ck_icn_p_uart8", "ck_icn_apb2", GATE_UART8),
	CS_GATE("ck_icn_p_uart9", "ck_icn_apb2", GATE_UART9),
	CS_GATE("ck_icn_p_fdcan", "ck_icn_apb2", GATE_FDCAN),
	CS_GATE("ck_icn_p_spi1", "ck_icn_apb2", GATE_SPI1),
	CS_GATE("ck_icn_p_spi4", "ck_icn_apb2", GATE_SPI4),
	CS_GATE("ck_icn_p_spi5", "ck_icn_apb2", GATE_SPI5),
	CS_GATE("ck_icn_p_spi6", "ck_icn_apb2", GATE_SPI6),
	CS_GATE("ck_icn_p_spi7", "ck_icn_apb2", GATE_SPI7),
	CS_GATE("ck_icn_p_bsec", "ck_icn_apb3", GATE_BSEC),
	CS_GATE("ck_icn_p_iwdg1", "ck_icn_apb3", GATE_IWDG1),
	CS_GATE("ck_icn_p_iwdg2", "ck_icn_apb3", GATE_IWDG2),
	CS_GATE("ck_icn_p_iwdg3", "ck_icn_apb3", GATE_IWDG3),
	CS_GATE("ck_icn_p_iwdg4", "ck_icn_apb3", GATE_IWDG4),
	CS_GATE("ck_icn_p_wwdg1", "ck_icn_apb3", GATE_WWDG1),
	CS_GATE("ck_icn_p_vref", "ck_icn_apb3", GATE_VREF),
	CS_GATE("ck_icn_p_dts", "ck_icn_apb3", GATE_DTS),
	CS_GATE("ck_icn_p_serc", "ck_icn_apb3", GATE_SERC),
	CS_GATE("ck_icn_p_hdp", "ck_icn_apb3", GATE_HDP),
	CS_GATE("ck_icn_p_is2m", "ck_icn_apb3", GATE_IS2M),
	CS_GATE("ck_icn_p_dsi", "ck_icn_apb4", GATE_DSI),
	CS_GATE("ck_icn_p_ltdc", "ck_icn_apb4", GATE_LTDC),
	CS_GATE("ck_icn_p_csi2", "ck_icn_apb4", GATE_CSI),
	CS_GATE("ck_icn_p_dcmipp", "ck_icn_apb4", GATE_DCMIPP),
	CS_GATE("ck_icn_p_ddrc", "ck_icn_apb4", GATE_DDRCAPB),
	CS_GATE("ck_icn_p_ddrcfg", "ck_icn_apb4", GATE_DDRCFG),
	CS_GATE("ck_icn_p_lvds", "ck_icn_apb4", GATE_LVDS),
	CS_GATE("ck_icn_p_gicv2m", "ck_icn_apb4", GATE_GICV2M),
	CS_GATE("ck_icn_p_usbtc", "ck_icn_apb4", GATE_USBTC),
	CS_GATE("ck_icn_p_usb3pciephy", "ck_icn_apb4", GATE_USB3PCIEPHY),
	CS_GATE("ck_icn_p_stgen", "ck_icn_apb4", GATE_STGEN),
	CS_GATE("ck_icn_p_vdec", "ck_icn_apb4", GATE_VDEC),
	CS_GATE("ck_icn_p_venc", "ck_icn_apb4", GATE_VENC),
	CS_GATE("ck_sys_dbg", "ck_icn_apbdbg", GATE_DBG),
	CS_GATE("ck_icn_p_stm", "ck_icn_apbdbg", GATE_STM),
	CS_GATE("ck_icn_p_etr", "ck_icn_apbdbg", GATE_ETR),
	CS_GATE("ck_ker_tim2", "ck_timg1", GATE_TIM2),
	CS_GATE("ck_ker_tim3", "ck_timg1", GATE_TIM3),
	CS_GATE("ck_ker_tim4", "ck_timg1", GATE_TIM4),
	CS_GATE("ck_ker_tim5", "ck_timg1", GATE_TIM5),
	CS_GATE("ck_ker_tim6", "ck_timg1", GATE_TIM6),
	CS_GATE("ck_ker_tim7", "ck_timg1", GATE_TIM7),
	CS_GATE("ck_ker_tim10", "ck_timg1", GATE_TIM10),
	CS_GATE("ck_ker_tim11", "ck_timg1", GATE_TIM11),
	CS_GATE("ck_ker_tim12", "ck_timg1", GATE_TIM12),
	CS_GATE("ck_ker_tim13", "ck_timg1", GATE_TIM13),
	CS_GATE("ck_ker_tim14", "ck_timg1", GATE_TIM14),
	CS_GATE("ck_ker_tim1", "ck_timg2", GATE_TIM1),
	CS_GATE("ck_ker_tim8", "ck_timg2", GATE_TIM8),
	CS_GATE("ck_ker_tim15", "ck_timg2", GATE_TIM15),
	CS_GATE("ck_ker_tim16", "ck_timg2", GATE_TIM16),
	CS_GATE("ck_ker_tim17", "ck_timg2", GATE_TIM17),
	CS_GATE("ck_ker_tim20", "ck_timg2", GATE_TIM20),
	CS_GATE("ck_ker_lptim1", "ck_flexgen_07", GATE_LPTIM1),
	CS_GATE("ck_ker_lptim2", "ck_flexgen_07", GATE_LPTIM2),
	CS_GATE("ck_ker_usart2", "ck_flexgen_08", GATE_USART2),
	CS_GATE("ck_ker_uart4", "ck_flexgen_08", GATE_UART4),
	CS_GATE("ck_ker_usart3", "ck_flexgen_09", GATE_USART3),
	CS_GATE("ck_ker_uart5", "ck_flexgen_09", GATE_UART5),
	CS_GATE("ck_ker_spi2", "ck_flexgen_10", GATE_SPI2),
	CS_GATE("ck_ker_spi3", "ck_flexgen_10", GATE_SPI3),
	CS_GATE("ck_ker_spdifrx", "ck_flexgen_11", GATE_SPDIFRX),
	CS_GATE("ck_ker_i2c1", "ck_flexgen_12", GATE_I2C1),
	CS_GATE("ck_ker_i2c2", "ck_flexgen_12", GATE_I2C2),
	CS_GATE("ck_ker_i3c1", "ck_flexgen_12", GATE_I3C1),
	CS_GATE("ck_ker_i3c2", "ck_flexgen_12", GATE_I3C2),
	CS_GATE("ck_ker_i2c3", "ck_flexgen_13", GATE_I2C3),
	CS_GATE("ck_ker_i2c5", "ck_flexgen_13", GATE_I2C5),
	CS_GATE("ck_ker_i3c3", "ck_flexgen_13", GATE_I3C3),
	CS_GATE("ck_ker_i2c4", "ck_flexgen_14", GATE_I2C4),
	CS_GATE("ck_ker_i2c6", "ck_flexgen_14", GATE_I2C6),
	CS_GATE("ck_ker_i2c7", "ck_flexgen_15", GATE_I2C7),
	CS_GATE("ck_ker_spi1", "ck_flexgen_16", GATE_SPI1),
	CS_GATE("ck_ker_spi4", "ck_flexgen_17", GATE_SPI4),
	CS_GATE("ck_ker_spi5", "ck_flexgen_17", GATE_SPI5),
	CS_GATE("ck_ker_spi6", "ck_flexgen_18", GATE_SPI6),
	CS_GATE("ck_ker_spi7", "ck_flexgen_18", GATE_SPI7),
	CS_GATE("ck_ker_usart1", "ck_flexgen_19", GATE_USART1),
	CS_GATE("ck_ker_usart6", "ck_flexgen_20", GATE_USART6),
	CS_GATE("ck_ker_uart7", "ck_flexgen_21", GATE_UART7),
	CS_GATE("ck_ker_uart8", "ck_flexgen_21", GATE_UART8),
	CS_GATE("ck_ker_uart9", "ck_flexgen_22", GATE_UART9),
	CS_GATE("ck_ker_mdf1", "ck_flexgen_23", GATE_MDF1),
	CS_GATE("ck_ker_sai1", "ck_flexgen_23", GATE_SAI1),
	CS_GATE("ck_ker_sai2", "ck_flexgen_24", GATE_SAI2),
	CS_GATE("ck_ker_sai3", "ck_flexgen_25", GATE_SAI3),
	CS_GATE("ck_ker_sai4", "ck_flexgen_25", GATE_SAI4),
	CS_GATE("ck_ker_fdcan", "ck_flexgen_26", GATE_FDCAN),
	CS_GATE("ck_ker_csi2", "ck_flexgen_29", GATE_CSI),
	CS_GATE("ck_ker_csi2txesc", "ck_flexgen_30", GATE_CSI),
	CS_GATE("ck_ker_csi2phy", "ck_flexgen_31", GATE_CSI),
	CS_GATE("ck_ker_stgen", "ck_flexgen_33", GATE_STGEN),
	CS_GATE("ck_ker_usbtc", "ck_flexgen_35", GATE_USBTC),
	CS_GATE("ck_ker_i3c4", "ck_flexgen_36", GATE_I3C4),
	CS_GATE("ck_ker_spi8", "ck_flexgen_37", GATE_SPI8),
	CS_GATE("ck_ker_i2c8", "ck_flexgen_38", GATE_I2C8),
	CS_GATE("ck_ker_lpuart1", "ck_flexgen_39", GATE_LPUART1),
	CS_GATE("ck_ker_lptim3", "ck_flexgen_40", GATE_LPTIM3),
	CS_GATE("ck_ker_lptim4", "ck_flexgen_41", GATE_LPTIM4),
	CS_GATE("ck_ker_lptim5", "ck_flexgen_41", GATE_LPTIM5),
	CS_GATE("ck_ker_adf1", "ck_flexgen_42", GATE_ADF1),
	CS_GATE("ck_ker_tsdbg", "ck_flexgen_43", GATE_DBG),
	CS_GATE("ck_ker_tpiu", "ck_flexgen_44", GATE_TRACE),
	CS_GATE("ck_icn_m_etr", "ck_flexgen_45", GATE_ETR),
	CS_GATE("ck_sys_atb", "ck_flexgen_45", GATE_DBG),
	CS_GATE("ck_ker_ospi1", "ck_flexgen_48", GATE_OSPI1),
	CS_GATE("ck_ker_ospi2", "ck_flexgen_49", GATE_OSPI2),
	CS_GATE("ck_ker_fmc", "ck_flexgen_50", GATE_FMC),
	CS_GATE("ck_ker_sdmmc1", "ck_flexgen_51", GATE_SDMMC1),
	CS_GATE("ck_ker_sdmmc2", "ck_flexgen_52", GATE_SDMMC2),
	CS_GATE("ck_ker_sdmmc3", "ck_flexgen_53", GATE_SDMMC3),
	CS_GATE("ck_ker_eth1", "ck_flexgen_54", GATE_ETH1),
	CS_GATE("ck_ker_ethsw", "ck_flexgen_54", GATE_ETHSW),
	CS_GATE("ck_ker_eth2", "ck_flexgen_55", GATE_ETH2),
	CS_GATE("ck_ker_eth1ptp", "ck_flexgen_56", GATE_ETH1),
	CS_GATE("ck_ker_eth2ptp", "ck_flexgen_56", GATE_ETH2),
	CS_GATE("ck_ker_usb2phy2", "ck_flexgen_58", GATE_USB3DR),
	CS_GATE("ck_icn_m_gpu", "ck_flexgen_59", GATE_GPU),
	CS_GATE("ck_ker_gpu", "ck_pll3", GATE_GPU),
	CS_GATE("ck_ker_ethswref", "ck_flexgen_60", GATE_ETHSWREF),
	CS_GATE("ck_ker_eth1stp", "ck_icn_ls_mcu", GATE_ETH1STP),
	CS_GATE("ck_ker_eth2stp", "ck_icn_ls_mcu", GATE_ETH2STP),
	CS_GATE("ck_ker_ltdc", "ck_flexgen_27", GATE_LTDC),

	CS_GATEMUX("ck_mco1", mco1_src, GATE_MCO1, MUX_MCO1),
	CS_GATEMUX("ck_mco2", mco2_src, GATE_MCO2, MUX_MCO2),
	CS_GATEMUX("ck_ker_adc12", adc12_src, GATE_ADC12, MUX_ADC12),
	CS_GATEMUX("ck_ker_adc3", adc3_src, GATE_ADC3, MUX_ADC3),
	CS_GATEMUX("ck_ker_usb2phy1", usb2phy1_src, GATE_USB2PHY1, MUX_USB2PHY1),
	CS_GATEMUX("ck_ker_usb2phy2_en", usb2phy2_src, GATE_USB2PHY2, MUX_USB2PHY2),
	CS_GATEMUX("ck_ker_usb3pciephy", usb3pciphy_src, GATE_USB3PCIEPHY, MUX_USB3PCIEPHY),
	CS_GATEMUX("clk_lanebyte", dsiblane_src, GATE_DSI, MUX_DSIBLANE),
	CS_GATEMUX("clk_phy_dsi", dsiphy_src, GATE_DSI, MUX_DSIPHY),
	CS_GATEMUX("ck_ker_lvdsphy", lvdsphy_src, GATE_LVDS, MUX_LVDSPHY),
	CS_GATEMUX("ck_ker_dts", dts_src, GATE_DTS, MUX_DTS),
	CS_GATEMUX("ck_rtc", rtc_src, GATE_RTCCK, MUX_RTC),
};

static struct clock_summary clock_summary_mp25 = {
	.clocks		= stm32mp25_clock_summary,
	.nb_clocks	= ARRAY_SIZE(stm32mp25_clock_summary),
};

#endif
