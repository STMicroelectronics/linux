// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2022 - All Rights Reserved
 * Author: Gabriel Fernandez <gabriel.fernandez@foss.st.com> for STMicroelectronics.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <dt-bindings/clock/stm32mp13-clks.h>
#include "clk-stm32-core.h"
#include "reset-stm32.h"
#include "stm32mp13_rcc.h"

#define STM32MP1_RESET_ID_MASK GENMASK(15, 0)
#define RCC_CLR_OFFSET		0x4

/* STM32 Gates definition */
enum enum_gate_cfg {
#ifdef CONFIG_DEBUG_FS
	GATE_LSE,
	GATE_LSE_RDY,
	GATE_LSI,
	GATE_LSI_RDY,
	GATE_HSI,
	GATE_HSI_RDY,
	GATE_CSI,
	GATE_CSI_RDY,
	GATE_HSE,
	GATE_HSE_RDY,
	GATE_PLL1,
	GATE_PLL1_RDY,
	GATE_PLL2,
	GATE_PLL2_RDY,
	GATE_PLL3,
	GATE_PLL3_RDY,
	GATE_PLL4,
	GATE_PLL4_RDY,
	GATE_HSIDIVRDY,
	GATE_MPUSRCRDY,
	GATE_AXISSRCRDY,
	GATE_MCUSSRCRDY,
	GATE_PLL12SRCRDY,
	GATE_PLL3SRCRDY,
	GATE_PLL4SRCRDY,
	GATE_MPUDIVRDY,
	GATE_AXIDIVRDY,
	GATE_MLAHBDIVRDY,
	GATE_APB1DIVRDY,
	GATE_APB2DIVRDY,
	GATE_APB3DIVRDY,
	GATE_APB4DIVRDY,
	GATE_APB5DIVRDY,
	GATE_APB6DIVRDY,
	GATE_RTCCK,
	GATE_PLL1_DIVP,
	GATE_PLL1_DIVQ,
	GATE_PLL1_DIVR,
	GATE_PLL2_DIVP,
	GATE_PLL2_DIVQ,
	GATE_PLL2_DIVR,
	GATE_PLL3_DIVP,
	GATE_PLL3_DIVQ,
	GATE_PLL3_DIVR,
	GATE_PLL4_DIVP,
	GATE_PLL4_DIVQ,
	GATE_PLL4_DIVR,
#endif
	GATE_MCO1,
	GATE_MCO2,
	GATE_DBGCK,
	GATE_TRACECK,
	GATE_DDRC1,
	GATE_DDRC1LP,
	GATE_DDRPHYC,
	GATE_DDRPHYCLP,
	GATE_DDRCAPB,
	GATE_DDRCAPBLP,
	GATE_AXIDCG,
	GATE_DDRPHYCAPB,
	GATE_DDRPHYCAPBLP,
	GATE_TIM2,
	GATE_TIM3,
	GATE_TIM4,
	GATE_TIM5,
	GATE_TIM6,
	GATE_TIM7,
	GATE_LPTIM1,
	GATE_SPI2,
	GATE_SPI3,
	GATE_USART3,
	GATE_UART4,
	GATE_UART5,
	GATE_UART7,
	GATE_UART8,
	GATE_I2C1,
	GATE_I2C2,
	GATE_SPDIF,
	GATE_TIM1,
	GATE_TIM8,
	GATE_SPI1,
	GATE_USART6,
	GATE_SAI1,
	GATE_SAI2,
	GATE_DFSDM,
	GATE_ADFSDM,
	GATE_FDCAN,
	GATE_LPTIM2,
	GATE_LPTIM3,
	GATE_LPTIM4,
	GATE_LPTIM5,
	GATE_VREF,
	GATE_DTS,
	GATE_PMBCTRL,
	GATE_HDP,
	GATE_SYSCFG,
	GATE_DCMIPP,
	GATE_DDRPERFM,
	GATE_IWDG2APB,
	GATE_USBPHY,
	GATE_STGENRO,
	GATE_LTDC,
	GATE_RTCAPB,
	GATE_TZC,
	GATE_ETZPC,
	GATE_IWDG1APB,
	GATE_BSEC,
	GATE_STGENC,
	GATE_USART1,
	GATE_USART2,
	GATE_SPI4,
	GATE_SPI5,
	GATE_I2C3,
	GATE_I2C4,
	GATE_I2C5,
	GATE_TIM12,
	GATE_TIM13,
	GATE_TIM14,
	GATE_TIM15,
	GATE_TIM16,
	GATE_TIM17,
	GATE_DMA1,
	GATE_DMA2,
	GATE_DMAMUX1,
	GATE_DMA3,
	GATE_DMAMUX2,
	GATE_ADC1,
	GATE_ADC2,
	GATE_USBO,
	GATE_TSC,
	GATE_GPIOA,
	GATE_GPIOB,
	GATE_GPIOC,
	GATE_GPIOD,
	GATE_GPIOE,
	GATE_GPIOF,
	GATE_GPIOG,
	GATE_GPIOH,
	GATE_GPIOI,
	GATE_PKA,
	GATE_SAES,
	GATE_CRYP1,
	GATE_HASH1,
	GATE_RNG1,
	GATE_BKPSRAM,
	GATE_AXIMC,
	GATE_MCE,
	GATE_ETH1CK,
	GATE_ETH1TX,
	GATE_ETH1RX,
	GATE_ETH1MAC,
	GATE_FMC,
	GATE_QSPI,
	GATE_SDMMC1,
	GATE_SDMMC2,
	GATE_CRC1,
	GATE_USBH,
	GATE_ETH2CK,
	GATE_ETH2TX,
	GATE_ETH2RX,
	GATE_ETH2MAC,
	GATE_ETH1STP,
	GATE_ETH2STP,
	GATE_MDMA,
	GATE_NB
};

#define _CFG_GATE(_id, _offset, _bit_idx, _offset_clr)\
	[(_id)] = {\
		.offset		= (_offset),\
		.bit_idx	= (_bit_idx),\
		.set_clr	= (_offset_clr),\
	}

#define CFG_GATE(_id, _offset, _bit_idx)\
	_CFG_GATE(_id, _offset, _bit_idx, 0)

#define CFG_GATE_SETCLR(_id, _offset, _bit_idx)\
	_CFG_GATE(_id, _offset, _bit_idx, RCC_CLR_OFFSET)

static struct stm32_gate_cfg stm32mp13_gates[] = {
#ifdef CONFIG_DEBUG_FS
	CFG_GATE(GATE_LSE,		RCC_BDCR,		0),
	CFG_GATE(GATE_LSE_RDY,		RCC_BDCR,		2),
	CFG_GATE(GATE_RTCCK,		RCC_BDCR,		20),
	CFG_GATE(GATE_LSI,		RCC_RDLSICR,		0),
	CFG_GATE(GATE_LSI_RDY,		RCC_RDLSICR,		1),
	CFG_GATE_SETCLR(GATE_HSI,	RCC_OCENSETR,		0),
	CFG_GATE(GATE_HSI_RDY,		RCC_OCRDYR,		0),
	CFG_GATE_SETCLR(GATE_CSI,	RCC_OCENSETR,		4),
	CFG_GATE(GATE_CSI_RDY,		RCC_OCRDYR,		4),
	CFG_GATE_SETCLR(GATE_HSE,	RCC_OCENSETR,		8),
	CFG_GATE(GATE_HSE_RDY,		RCC_OCRDYR,		8),
	CFG_GATE(GATE_HSIDIVRDY,	RCC_OCRDYR,		2),
	CFG_GATE(GATE_MPUSRCRDY,	RCC_MPCKSELR,		31),
	CFG_GATE(GATE_AXISSRCRDY,	RCC_ASSCKSELR,		31),
	CFG_GATE(GATE_MCUSSRCRDY,	RCC_MSSCKSELR,		31),
	CFG_GATE(GATE_PLL12SRCRDY,	RCC_RCK12SELR,		31),
	CFG_GATE(GATE_PLL3SRCRDY,	RCC_RCK3SELR,		31),
	CFG_GATE(GATE_PLL4SRCRDY,	RCC_RCK4SELR,		31),
	CFG_GATE(GATE_MPUDIVRDY,	RCC_MPCKDIVR,		31),
	CFG_GATE(GATE_AXIDIVRDY,	RCC_AXIDIVR,		31),
	CFG_GATE(GATE_MLAHBDIVRDY,	RCC_MLAHBDIVR,		31),
	CFG_GATE(GATE_APB1DIVRDY,	RCC_APB1DIVR,		31),
	CFG_GATE(GATE_APB2DIVRDY,	RCC_APB2DIVR,		31),
	CFG_GATE(GATE_APB3DIVRDY,	RCC_APB3DIVR,		31),
	CFG_GATE(GATE_APB4DIVRDY,	RCC_APB4DIVR,		31),
	CFG_GATE(GATE_APB5DIVRDY,	RCC_APB5DIVR,		31),
	CFG_GATE(GATE_APB6DIVRDY,	RCC_APB6DIVR,		31),
	CFG_GATE(GATE_PLL1,		RCC_PLL1CR,		0),
	CFG_GATE(GATE_PLL1_RDY,		RCC_PLL1CR,		1),
	CFG_GATE(GATE_PLL1_DIVP,	RCC_PLL1CR,		4),
	CFG_GATE(GATE_PLL1_DIVQ,	RCC_PLL1CR,		5),
	CFG_GATE(GATE_PLL1_DIVR,	RCC_PLL1CR,		6),
	CFG_GATE(GATE_PLL2,		RCC_PLL2CR,		0),
	CFG_GATE(GATE_PLL2_RDY,		RCC_PLL2CR,		1),
	CFG_GATE(GATE_PLL2_DIVP,	RCC_PLL2CR,		4),
	CFG_GATE(GATE_PLL2_DIVQ,	RCC_PLL2CR,		5),
	CFG_GATE(GATE_PLL2_DIVR,	RCC_PLL2CR,		6),
	CFG_GATE(GATE_PLL3,		RCC_PLL3CR,		0),
	CFG_GATE(GATE_PLL3_RDY,		RCC_PLL3CR,		1),
	CFG_GATE(GATE_PLL3_DIVP,	RCC_PLL3CR,		4),
	CFG_GATE(GATE_PLL3_DIVQ,	RCC_PLL3CR,		5),
	CFG_GATE(GATE_PLL3_DIVR,	RCC_PLL3CR,		6),
	CFG_GATE(GATE_PLL4,		RCC_PLL4CR,		0),
	CFG_GATE(GATE_PLL4_RDY,		RCC_PLL4CR,		1),
	CFG_GATE(GATE_PLL4_DIVP,	RCC_PLL4CR,		4),
	CFG_GATE(GATE_PLL4_DIVQ,	RCC_PLL4CR,		5),
	CFG_GATE(GATE_PLL4_DIVR,	RCC_PLL4CR,		6),
#endif
	CFG_GATE(GATE_MCO1,		RCC_MCO1CFGR,		12),
	CFG_GATE(GATE_MCO2,		RCC_MCO2CFGR,		12),
	CFG_GATE(GATE_DBGCK,		RCC_DBGCFGR,		8),
	CFG_GATE(GATE_TRACECK,		RCC_DBGCFGR,		9),
	CFG_GATE(GATE_DDRC1,		RCC_DDRITFCR,		0),
	CFG_GATE(GATE_DDRC1LP,		RCC_DDRITFCR,		1),
	CFG_GATE(GATE_DDRPHYC,		RCC_DDRITFCR,		4),
	CFG_GATE(GATE_DDRPHYCLP,	RCC_DDRITFCR,		5),
	CFG_GATE(GATE_DDRCAPB,		RCC_DDRITFCR,		6),
	CFG_GATE(GATE_DDRCAPBLP,	RCC_DDRITFCR,		7),
	CFG_GATE(GATE_AXIDCG,		RCC_DDRITFCR,		8),
	CFG_GATE(GATE_DDRPHYCAPB,	RCC_DDRITFCR,		9),
	CFG_GATE(GATE_DDRPHYCAPBLP,	RCC_DDRITFCR,		10),
	CFG_GATE_SETCLR(GATE_TIM2,	RCC_MP_APB1ENSETR,	0),
	CFG_GATE_SETCLR(GATE_TIM3,	RCC_MP_APB1ENSETR,	1),
	CFG_GATE_SETCLR(GATE_TIM4,	RCC_MP_APB1ENSETR,	2),
	CFG_GATE_SETCLR(GATE_TIM5,	RCC_MP_APB1ENSETR,	3),
	CFG_GATE_SETCLR(GATE_TIM6,	RCC_MP_APB1ENSETR,	4),
	CFG_GATE_SETCLR(GATE_TIM7,	RCC_MP_APB1ENSETR,	5),
	CFG_GATE_SETCLR(GATE_LPTIM1,	RCC_MP_APB1ENSETR,	9),
	CFG_GATE_SETCLR(GATE_SPI2,	RCC_MP_APB1ENSETR,	11),
	CFG_GATE_SETCLR(GATE_SPI3,	RCC_MP_APB1ENSETR,	12),
	CFG_GATE_SETCLR(GATE_USART3,	RCC_MP_APB1ENSETR,	15),
	CFG_GATE_SETCLR(GATE_UART4,	RCC_MP_APB1ENSETR,	16),
	CFG_GATE_SETCLR(GATE_UART5,	RCC_MP_APB1ENSETR,	17),
	CFG_GATE_SETCLR(GATE_UART7,	RCC_MP_APB1ENSETR,	18),
	CFG_GATE_SETCLR(GATE_UART8,	RCC_MP_APB1ENSETR,	19),
	CFG_GATE_SETCLR(GATE_I2C1,	RCC_MP_APB1ENSETR,	21),
	CFG_GATE_SETCLR(GATE_I2C2,	RCC_MP_APB1ENSETR,	22),
	CFG_GATE_SETCLR(GATE_SPDIF,	RCC_MP_APB1ENSETR,	26),
	CFG_GATE_SETCLR(GATE_TIM1,	RCC_MP_APB2ENSETR,	0),
	CFG_GATE_SETCLR(GATE_TIM8,	RCC_MP_APB2ENSETR,	1),
	CFG_GATE_SETCLR(GATE_SPI1,	RCC_MP_APB2ENSETR,	8),
	CFG_GATE_SETCLR(GATE_USART6,	RCC_MP_APB2ENSETR,	13),
	CFG_GATE_SETCLR(GATE_SAI1,	RCC_MP_APB2ENSETR,	16),
	CFG_GATE_SETCLR(GATE_SAI2,	RCC_MP_APB2ENSETR,	17),
	CFG_GATE_SETCLR(GATE_DFSDM,	RCC_MP_APB2ENSETR,	20),
	CFG_GATE_SETCLR(GATE_ADFSDM,	RCC_MP_APB2ENSETR,	21),
	CFG_GATE_SETCLR(GATE_FDCAN,	RCC_MP_APB2ENSETR,	24),
	CFG_GATE_SETCLR(GATE_LPTIM2,	RCC_MP_APB3ENSETR,	0),
	CFG_GATE_SETCLR(GATE_LPTIM3,	RCC_MP_APB3ENSETR,	1),
	CFG_GATE_SETCLR(GATE_LPTIM4,	RCC_MP_APB3ENSETR,	2),
	CFG_GATE_SETCLR(GATE_LPTIM5,	RCC_MP_APB3ENSETR,	3),
	CFG_GATE_SETCLR(GATE_VREF,	RCC_MP_APB3ENSETR,	13),
	CFG_GATE_SETCLR(GATE_DTS,	RCC_MP_APB3ENSETR,	16),
	CFG_GATE_SETCLR(GATE_PMBCTRL,	RCC_MP_APB3ENSETR,	17),
	CFG_GATE_SETCLR(GATE_HDP,	RCC_MP_APB3ENSETR,	20),
	CFG_GATE_SETCLR(GATE_SYSCFG,	RCC_MP_NS_APB3ENSETR,	0),
	CFG_GATE_SETCLR(GATE_DCMIPP,	RCC_MP_APB4ENSETR,	1),
	CFG_GATE_SETCLR(GATE_DDRPERFM,	RCC_MP_APB4ENSETR,	8),
	CFG_GATE_SETCLR(GATE_IWDG2APB,	RCC_MP_APB4ENSETR,	15),
	CFG_GATE_SETCLR(GATE_USBPHY,	RCC_MP_APB4ENSETR,	16),
	CFG_GATE_SETCLR(GATE_STGENRO,	RCC_MP_APB4ENSETR,	20),
	CFG_GATE_SETCLR(GATE_LTDC,	RCC_MP_NS_APB4ENSETR,	0),
	CFG_GATE_SETCLR(GATE_RTCAPB,	RCC_MP_APB5ENSETR,	8),
	CFG_GATE_SETCLR(GATE_TZC,	RCC_MP_APB5ENSETR,	11),
	CFG_GATE_SETCLR(GATE_ETZPC,	RCC_MP_APB5ENSETR,	13),
	CFG_GATE_SETCLR(GATE_IWDG1APB,	RCC_MP_APB5ENSETR,	15),
	CFG_GATE_SETCLR(GATE_BSEC,	RCC_MP_APB5ENSETR,	16),
	CFG_GATE_SETCLR(GATE_STGENC,	RCC_MP_APB5ENSETR,	20),
	CFG_GATE_SETCLR(GATE_USART1,	RCC_MP_APB6ENSETR,	0),
	CFG_GATE_SETCLR(GATE_USART2,	RCC_MP_APB6ENSETR,	1),
	CFG_GATE_SETCLR(GATE_SPI4,	RCC_MP_APB6ENSETR,	2),
	CFG_GATE_SETCLR(GATE_SPI5,	RCC_MP_APB6ENSETR,	3),
	CFG_GATE_SETCLR(GATE_I2C3,	RCC_MP_APB6ENSETR,	4),
	CFG_GATE_SETCLR(GATE_I2C4,	RCC_MP_APB6ENSETR,	5),
	CFG_GATE_SETCLR(GATE_I2C5,	RCC_MP_APB6ENSETR,	6),
	CFG_GATE_SETCLR(GATE_TIM12,	RCC_MP_APB6ENSETR,	7),
	CFG_GATE_SETCLR(GATE_TIM13,	RCC_MP_APB6ENSETR,	8),
	CFG_GATE_SETCLR(GATE_TIM14,	RCC_MP_APB6ENSETR,	9),
	CFG_GATE_SETCLR(GATE_TIM15,	RCC_MP_APB6ENSETR,	10),
	CFG_GATE_SETCLR(GATE_TIM16,	RCC_MP_APB6ENSETR,	11),
	CFG_GATE_SETCLR(GATE_TIM17,	RCC_MP_APB6ENSETR,	12),
	CFG_GATE_SETCLR(GATE_DMA1,	RCC_MP_AHB2ENSETR,	0),
	CFG_GATE_SETCLR(GATE_DMA2,	RCC_MP_AHB2ENSETR,	1),
	CFG_GATE_SETCLR(GATE_DMAMUX1,	RCC_MP_AHB2ENSETR,	2),
	CFG_GATE_SETCLR(GATE_DMA3,	RCC_MP_AHB2ENSETR,	3),
	CFG_GATE_SETCLR(GATE_DMAMUX2,	RCC_MP_AHB2ENSETR,	4),
	CFG_GATE_SETCLR(GATE_ADC1,	RCC_MP_AHB2ENSETR,	5),
	CFG_GATE_SETCLR(GATE_ADC2,	RCC_MP_AHB2ENSETR,	6),
	CFG_GATE_SETCLR(GATE_USBO,	RCC_MP_AHB2ENSETR,	8),
	CFG_GATE_SETCLR(GATE_TSC,	RCC_MP_AHB4ENSETR,	15),
	CFG_GATE_SETCLR(GATE_GPIOA,	RCC_MP_NS_AHB4ENSETR,	0),
	CFG_GATE_SETCLR(GATE_GPIOB,	RCC_MP_NS_AHB4ENSETR,	1),
	CFG_GATE_SETCLR(GATE_GPIOC,	RCC_MP_NS_AHB4ENSETR,	2),
	CFG_GATE_SETCLR(GATE_GPIOD,	RCC_MP_NS_AHB4ENSETR,	3),
	CFG_GATE_SETCLR(GATE_GPIOE,	RCC_MP_NS_AHB4ENSETR,	4),
	CFG_GATE_SETCLR(GATE_GPIOF,	RCC_MP_NS_AHB4ENSETR,	5),
	CFG_GATE_SETCLR(GATE_GPIOG,	RCC_MP_NS_AHB4ENSETR,	6),
	CFG_GATE_SETCLR(GATE_GPIOH,	RCC_MP_NS_AHB4ENSETR,	7),
	CFG_GATE_SETCLR(GATE_GPIOI,	RCC_MP_NS_AHB4ENSETR,	8),
	CFG_GATE_SETCLR(GATE_PKA,	RCC_MP_AHB5ENSETR,	2),
	CFG_GATE_SETCLR(GATE_SAES,	RCC_MP_AHB5ENSETR,	3),
	CFG_GATE_SETCLR(GATE_CRYP1,	RCC_MP_AHB5ENSETR,	4),
	CFG_GATE_SETCLR(GATE_HASH1,	RCC_MP_AHB5ENSETR,	5),
	CFG_GATE_SETCLR(GATE_RNG1,	RCC_MP_AHB5ENSETR,	6),
	CFG_GATE_SETCLR(GATE_BKPSRAM,	RCC_MP_AHB5ENSETR,	8),
	CFG_GATE_SETCLR(GATE_AXIMC,	RCC_MP_AHB5ENSETR,	16),
	CFG_GATE_SETCLR(GATE_MCE,	RCC_MP_AHB6ENSETR,	1),
	CFG_GATE_SETCLR(GATE_ETH1CK,	RCC_MP_AHB6ENSETR,	7),
	CFG_GATE_SETCLR(GATE_ETH1TX,	RCC_MP_AHB6ENSETR,	8),
	CFG_GATE_SETCLR(GATE_ETH1RX,	RCC_MP_AHB6ENSETR,	9),
	CFG_GATE_SETCLR(GATE_ETH1MAC,	RCC_MP_AHB6ENSETR,	10),
	CFG_GATE_SETCLR(GATE_FMC,	RCC_MP_AHB6ENSETR,	12),
	CFG_GATE_SETCLR(GATE_QSPI,	RCC_MP_AHB6ENSETR,	14),
	CFG_GATE_SETCLR(GATE_SDMMC1,	RCC_MP_AHB6ENSETR,	16),
	CFG_GATE_SETCLR(GATE_SDMMC2,	RCC_MP_AHB6ENSETR,	17),
	CFG_GATE_SETCLR(GATE_CRC1,	RCC_MP_AHB6ENSETR,	20),
	CFG_GATE_SETCLR(GATE_USBH,	RCC_MP_AHB6ENSETR,	24),
	CFG_GATE_SETCLR(GATE_ETH2CK,	RCC_MP_AHB6ENSETR,	27),
	CFG_GATE_SETCLR(GATE_ETH2TX,	RCC_MP_AHB6ENSETR,	28),
	CFG_GATE_SETCLR(GATE_ETH2RX,	RCC_MP_AHB6ENSETR,	29),
	CFG_GATE_SETCLR(GATE_ETH2MAC,	RCC_MP_AHB6ENSETR,	30),
	CFG_GATE_SETCLR(GATE_ETH1STP,	RCC_MP_AHB6LPENSETR,	11),
	CFG_GATE_SETCLR(GATE_ETH2STP,	RCC_MP_AHB6LPENSETR,	31),
	CFG_GATE_SETCLR(GATE_MDMA,	RCC_MP_NS_AHB6ENSETR,	0),
};

/* STM32 Divivers definition */
enum enum_div_cfg {
#ifdef CONFIG_DEBUG_FS
	DIV_PLL1DIVP,
	DIV_PLL2DIVP,
	DIV_PLL2DIVQ,
	DIV_PLL2DIVR,
	DIV_PLL3DIVP,
	DIV_PLL3DIVQ,
	DIV_PLL3DIVR,
	DIV_PLL4DIVP,
	DIV_PLL4DIVQ,
	DIV_PLL4DIVR,
	DIV_MPU,
	DIV_AXI,
	DIV_MLAHB,
	DIV_APB1,
	DIV_APB2,
	DIV_APB3,
	DIV_APB4,
	DIV_APB5,
	DIV_APB6,
#endif
	DIV_RTC,
	DIV_HSI,
	DIV_MCO1,
	DIV_MCO2,
	DIV_TRACE,
	DIV_ETH1PTP,
	DIV_ETH2PTP,
	DIV_NB
};

#ifdef CONFIG_DEBUG_FS
static const struct clk_div_table axi_div_table[] = {
	{ 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 },
	{ 4, 4 }, { 5, 4 }, { 6, 4 }, { 7, 4 },
	{ 0 },
};

static const struct clk_div_table mlahb_div_table[] = {
	{ 0, 1 }, { 1, 2 }, { 2, 4 }, { 3, 8 },
	{ 4, 16 }, { 5, 32 }, { 6, 64 }, { 7, 128 },
	{ 8, 256 }, { 9, 512 }, { 10, 512}, { 11, 512 },
	{ 12, 512 }, { 13, 512 }, { 14, 512}, { 15, 512 },
	{ 0 },
};

static const struct clk_div_table apb_div_table[] = {
	{ 0, 1 }, { 1, 2 }, { 2, 4 }, { 3, 8 },
	{ 4, 16 }, { 5, 16 }, { 6, 16 }, { 7, 16 },
	{ 0 },
};
#endif

static const struct clk_div_table ck_trace_div_table[] = {
	{ 0, 1 }, { 1, 2 }, { 2, 4 }, { 3, 8 },
	{ 4, 16 }, { 5, 16 }, { 6, 16 }, { 7, 16 },
	{ 0 },
};

#define CFG_DIV(_id, _offset, _shift, _width, _flags, _table, _ready)\
	[(_id)] = {\
		.offset	= (_offset),\
		.shift	= (_shift),\
		.width	= (_width),\
		.flags	= (_flags),\
		.table	= (_table),\
		.ready	= (_ready),\
	}

static const struct stm32_div_cfg stm32mp13_dividers[DIV_NB] = {
#ifdef CONFIG_DEBUG_FS
	CFG_DIV(DIV_MPU, RCC_MPCKDIVR, 0, 4, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_AXI, RCC_AXIDIVR, 0, 3, 0, axi_div_table, DIV_NO_RDY),
	CFG_DIV(DIV_MLAHB, RCC_MLAHBDIVR, 0, 4, 0, mlahb_div_table, DIV_NO_RDY),
	CFG_DIV(DIV_APB1, RCC_APB1DIVR, 0, 3, 0, apb_div_table, DIV_NO_RDY),
	CFG_DIV(DIV_APB2, RCC_APB2DIVR, 0, 3, 0, apb_div_table, DIV_NO_RDY),
	CFG_DIV(DIV_APB3, RCC_APB3DIVR, 0, 3, 0, apb_div_table, DIV_NO_RDY),
	CFG_DIV(DIV_APB4, RCC_APB4DIVR, 0, 3, 0, apb_div_table, DIV_NO_RDY),
	CFG_DIV(DIV_APB5, RCC_APB5DIVR, 0, 3, 0, apb_div_table, DIV_NO_RDY),
	CFG_DIV(DIV_APB6, RCC_APB6DIVR, 0, 3, 0, apb_div_table, DIV_NO_RDY),
	CFG_DIV(DIV_HSI, RCC_HSICFGR, 0, 2, CLK_DIVIDER_POWER_OF_TWO, NULL, DIV_NO_RDY),

	CFG_DIV(DIV_PLL1DIVP, RCC_PLL1CFGR2, 0, 7, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_PLL2DIVP, RCC_PLL2CFGR2, 0, 7, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_PLL2DIVQ, RCC_PLL2CFGR2, 8, 7, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_PLL2DIVR, RCC_PLL2CFGR2, 16, 7, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_PLL3DIVP, RCC_PLL3CFGR2, 0, 7, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_PLL3DIVQ, RCC_PLL3CFGR2, 8, 7, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_PLL3DIVR, RCC_PLL3CFGR2, 16, 7, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_PLL4DIVP, RCC_PLL4CFGR2, 0, 7, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_PLL4DIVQ, RCC_PLL4CFGR2, 8, 7, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_PLL4DIVR, RCC_PLL4CFGR2, 16, 7, 0, NULL, DIV_NO_RDY),
#endif
	CFG_DIV(DIV_RTC, RCC_RTCDIVR, 0, 6, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_MCO1, RCC_MCO1CFGR, 4, 4, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_MCO2, RCC_MCO2CFGR, 4, 4, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_TRACE, RCC_DBGCFGR, 0, 3, 0, ck_trace_div_table, DIV_NO_RDY),
	CFG_DIV(DIV_ETH1PTP, RCC_ETH12CKSELR, 4, 4, 0, NULL, DIV_NO_RDY),
	CFG_DIV(DIV_ETH2PTP, RCC_ETH12CKSELR, 12, 4, 0, NULL, DIV_NO_RDY),
};

/* STM32 Muxes definition */
enum enum_mux_cfg {
#ifdef CONFIG_DEBUG_FS
	MUX_MPU,
	MUX_AXI,
	MUX_MLAHB,
	MUX_PLL12,
	MUX_PLL3,
	MUX_PLL4,
	MUX_RTC,
	MUX_CKPER,
#endif
	MUX_ADC1,
	MUX_ADC2,
	MUX_DCMIPP,
	MUX_ETH1,
	MUX_ETH2,
	MUX_FDCAN,
	MUX_FMC,
	MUX_I2C12,
	MUX_I2C3,
	MUX_I2C4,
	MUX_I2C5,
	MUX_LPTIM1,
	MUX_LPTIM2,
	MUX_LPTIM3,
	MUX_LPTIM45,
	MUX_MCO1,
	MUX_MCO2,
	MUX_QSPI,
	MUX_RNG1,
	MUX_SAES,
	MUX_SAI1,
	MUX_SAI2,
	MUX_SDMMC1,
	MUX_SDMMC2,
	MUX_SPDIF,
	MUX_SPI1,
	MUX_SPI23,
	MUX_SPI4,
	MUX_SPI5,
	MUX_STGEN,
	MUX_UART1,
	MUX_UART2,
	MUX_UART4,
	MUX_UART6,
	MUX_UART35,
	MUX_UART78,
	MUX_USBO,
	MUX_USBPHY,
	MUX_NB
};

#define _CFG_MUX(_id, _offset, _shift, _witdh, _ready, _flags)\
	[_id] = {\
		.offset		= (_offset),\
		.shift		= (_shift),\
		.width		= (_witdh),\
		.ready		= (_ready),\
		.flags		= (_flags),\
	}

#define CFG_MUX(_id, _offset, _shift, _witdh)\
	_CFG_MUX(_id, _offset, _shift, _witdh, MUX_NO_RDY, 0)

#define CFG_MUX_SAFE(_id, _offset, _shift, _witdh)\
	_CFG_MUX(_id, _offset, _shift, _witdh, MUX_NO_RDY, MUX_SAFE)

static const struct stm32_mux_cfg stm32mp13_muxes[] = {
#ifdef CONFIG_DEBUG_FS
	CFG_MUX(MUX_MPU,	RCC_MPCKSELR,		0, 2),
	CFG_MUX(MUX_AXI,	RCC_ASSCKSELR,		0, 3),
	CFG_MUX(MUX_MLAHB,	RCC_MSSCKSELR,		0, 2),
	CFG_MUX(MUX_PLL12,	RCC_RCK12SELR,		0, 2),
	CFG_MUX(MUX_PLL3,	RCC_RCK3SELR,		0, 2),
	CFG_MUX(MUX_PLL4,	RCC_RCK4SELR,		0, 2),
	CFG_MUX(MUX_CKPER,	RCC_CPERCKSELR,		0, 2),
	CFG_MUX(MUX_RTC,	RCC_BDCR,		16, 2),
#endif
	CFG_MUX(MUX_I2C12,	RCC_I2C12CKSELR,	0, 3),
	CFG_MUX(MUX_LPTIM45,	RCC_LPTIM45CKSELR,	0, 3),
	CFG_MUX(MUX_SPI23,	RCC_SPI2S23CKSELR,	0, 3),
	CFG_MUX(MUX_UART35,	RCC_UART35CKSELR,	0, 3),
	CFG_MUX(MUX_UART78,	RCC_UART78CKSELR,	0, 3),
	CFG_MUX(MUX_ADC1,	RCC_ADC12CKSELR,	0, 2),
	CFG_MUX(MUX_ADC2,	RCC_ADC12CKSELR,	2, 2),
	CFG_MUX(MUX_DCMIPP,	RCC_DCMIPPCKSELR,	0, 2),
	CFG_MUX(MUX_ETH1,	RCC_ETH12CKSELR,	0, 2),
	CFG_MUX(MUX_ETH2,	RCC_ETH12CKSELR,	8, 2),
	CFG_MUX(MUX_FDCAN,	RCC_FDCANCKSELR,	0, 2),
	CFG_MUX(MUX_I2C3,	RCC_I2C345CKSELR,	0, 3),
	CFG_MUX(MUX_I2C4,	RCC_I2C345CKSELR,	3, 3),
	CFG_MUX(MUX_I2C5,	RCC_I2C345CKSELR,	6, 3),
	CFG_MUX(MUX_LPTIM1,	RCC_LPTIM1CKSELR,	0, 3),
	CFG_MUX(MUX_LPTIM2,	RCC_LPTIM23CKSELR,	0, 3),
	CFG_MUX(MUX_LPTIM3,	RCC_LPTIM23CKSELR,	3, 3),
	CFG_MUX(MUX_MCO1,	RCC_MCO1CFGR,		0, 3),
	CFG_MUX(MUX_MCO2,	RCC_MCO2CFGR,		0, 3),
	CFG_MUX(MUX_RNG1,	RCC_RNG1CKSELR,		0, 2),
	CFG_MUX(MUX_SAES,	RCC_SAESCKSELR,		0, 2),
	CFG_MUX(MUX_SAI1,	RCC_SAI1CKSELR,		0, 3),
	CFG_MUX(MUX_SAI2,	RCC_SAI2CKSELR,		0, 3),
	CFG_MUX(MUX_SPDIF,	RCC_SPDIFCKSELR,	0, 2),
	CFG_MUX(MUX_SPI1,	RCC_SPI2S1CKSELR,	0, 3),
	CFG_MUX(MUX_SPI4,	RCC_SPI45CKSELR,	0, 3),
	CFG_MUX(MUX_SPI5,	RCC_SPI45CKSELR,	3, 3),
	CFG_MUX(MUX_STGEN,	RCC_STGENCKSELR,	0, 2),
	CFG_MUX(MUX_UART1,	RCC_UART12CKSELR,	0, 3),
	CFG_MUX(MUX_UART2,	RCC_UART12CKSELR,	3, 3),
	CFG_MUX(MUX_UART4,	RCC_UART4CKSELR,	0, 3),
	CFG_MUX(MUX_UART6,	RCC_UART6CKSELR,	0, 3),
	CFG_MUX(MUX_USBO,	RCC_USBCKSELR,		4, 1),
	CFG_MUX(MUX_USBPHY,	RCC_USBCKSELR,		0, 2),
	CFG_MUX_SAFE(MUX_FMC,	RCC_FMCCKSELR,		0, 2),
	CFG_MUX_SAFE(MUX_QSPI,	RCC_QSPICKSELR,		0, 2),
	CFG_MUX_SAFE(MUX_SDMMC1, RCC_SDMMC12CKSELR,	0, 3),
	CFG_MUX_SAFE(MUX_SDMMC2, RCC_SDMMC12CKSELR,	3, 3),
};

struct clk_stm32_securiy {
	u32	offset;
	u8	bit_idx;
	unsigned long scmi_id;
};

enum security_clk {
	SECF_NONE,
	SECF_LPTIM2,
	SECF_LPTIM3,
	SECF_VREF,
	SECF_DCMIPP,
	SECF_USBPHY,
	SECF_TZC,
	SECF_ETZPC,
	SECF_IWDG1,
	SECF_BSEC,
	SECF_STGENC,
	SECF_STGENRO,
	SECF_USART1,
	SECF_USART2,
	SECF_SPI4,
	SECF_SPI5,
	SECF_I2C3,
	SECF_I2C4,
	SECF_I2C5,
	SECF_TIM12,
	SECF_TIM13,
	SECF_TIM14,
	SECF_TIM15,
	SECF_TIM16,
	SECF_TIM17,
	SECF_DMA3,
	SECF_DMAMUX2,
	SECF_ADC1,
	SECF_ADC2,
	SECF_USBO,
	SECF_TSC,
	SECF_PKA,
	SECF_SAES,
	SECF_CRYP1,
	SECF_HASH1,
	SECF_RNG1,
	SECF_BKPSRAM,
	SECF_MCE,
	SECF_FMC,
	SECF_QSPI,
	SECF_SDMMC1,
	SECF_SDMMC2,
	SECF_ETH1CK,
	SECF_ETH1TX,
	SECF_ETH1RX,
	SECF_ETH1MAC,
	SECF_ETH1STP,
	SECF_ETH2CK,
	SECF_ETH2TX,
	SECF_ETH2RX,
	SECF_ETH2MAC,
	SECF_ETH2STP,
	SECF_MCO1,
	SECF_MCO2
};

#define SECF(_sec_id, _offset, _bit_idx)[_sec_id] = {\
	.offset	= _offset,\
	.bit_idx	= _bit_idx,\
	.scmi_id	= -1,\
}

static const struct clk_stm32_securiy stm32mp13_security[] = {
	SECF(SECF_LPTIM2, RCC_APB3SECSR, RCC_APB3SECSR_LPTIM2SECF),
	SECF(SECF_LPTIM3, RCC_APB3SECSR, RCC_APB3SECSR_LPTIM3SECF),
	SECF(SECF_VREF, RCC_APB3SECSR, RCC_APB3SECSR_VREFSECF),
	SECF(SECF_DCMIPP, RCC_APB4SECSR, RCC_APB4SECSR_DCMIPPSECF),
	SECF(SECF_USBPHY, RCC_APB4SECSR, RCC_APB4SECSR_USBPHYSECF),
	SECF(SECF_TZC, RCC_APB5SECSR, RCC_APB5SECSR_TZCSECF),
	SECF(SECF_ETZPC, RCC_APB5SECSR, RCC_APB5SECSR_ETZPCSECF),
	SECF(SECF_IWDG1, RCC_APB5SECSR, RCC_APB5SECSR_IWDG1SECF),
	SECF(SECF_BSEC, RCC_APB5SECSR, RCC_APB5SECSR_BSECSECF),
	SECF(SECF_STGENC, RCC_APB5SECSR, RCC_APB5SECSR_STGENCSECF),
	SECF(SECF_STGENRO, RCC_APB5SECSR, RCC_APB5SECSR_STGENROSECF),
	SECF(SECF_USART1, RCC_APB6SECSR, RCC_APB6SECSR_USART1SECF),
	SECF(SECF_USART2, RCC_APB6SECSR, RCC_APB6SECSR_USART2SECF),
	SECF(SECF_SPI4, RCC_APB6SECSR, RCC_APB6SECSR_SPI4SECF),
	SECF(SECF_SPI5, RCC_APB6SECSR, RCC_APB6SECSR_SPI5SECF),
	SECF(SECF_I2C3, RCC_APB6SECSR, RCC_APB6SECSR_I2C3SECF),
	SECF(SECF_I2C4, RCC_APB6SECSR, RCC_APB6SECSR_I2C4SECF),
	SECF(SECF_I2C5, RCC_APB6SECSR, RCC_APB6SECSR_I2C5SECF),
	SECF(SECF_TIM12, RCC_APB6SECSR, RCC_APB6SECSR_TIM12SECF),
	SECF(SECF_TIM13, RCC_APB6SECSR, RCC_APB6SECSR_TIM13SECF),
	SECF(SECF_TIM14, RCC_APB6SECSR, RCC_APB6SECSR_TIM14SECF),
	SECF(SECF_TIM15, RCC_APB6SECSR, RCC_APB6SECSR_TIM15SECF),
	SECF(SECF_TIM16, RCC_APB6SECSR, RCC_APB6SECSR_TIM16SECF),
	SECF(SECF_TIM17, RCC_APB6SECSR, RCC_APB6SECSR_TIM17SECF),
	SECF(SECF_DMA3, RCC_AHB2SECSR, RCC_AHB2SECSR_DMA3SECF),
	SECF(SECF_DMAMUX2, RCC_AHB2SECSR, RCC_AHB2SECSR_DMAMUX2SECF),
	SECF(SECF_ADC1, RCC_AHB2SECSR, RCC_AHB2SECSR_ADC1SECF),
	SECF(SECF_ADC2, RCC_AHB2SECSR, RCC_AHB2SECSR_ADC2SECF),
	SECF(SECF_USBO, RCC_AHB2SECSR, RCC_AHB2SECSR_USBOSECF),
	SECF(SECF_TSC, RCC_AHB4SECSR, RCC_AHB4SECSR_TSCSECF),
	SECF(SECF_PKA, RCC_AHB5SECSR, RCC_AHB5SECSR_PKASECF),
	SECF(SECF_SAES, RCC_AHB5SECSR, RCC_AHB5SECSR_SAESSECF),
	SECF(SECF_CRYP1, RCC_AHB5SECSR, RCC_AHB5SECSR_CRYP1SECF),
	SECF(SECF_HASH1, RCC_AHB5SECSR, RCC_AHB5SECSR_HASH1SECF),
	SECF(SECF_RNG1, RCC_AHB5SECSR, RCC_AHB5SECSR_RNG1SECF),
	SECF(SECF_BKPSRAM, RCC_AHB5SECSR, RCC_AHB5SECSR_BKPSRAMSECF),
	SECF(SECF_MCE, RCC_AHB6SECSR, RCC_AHB6SECSR_MCESECF),
	SECF(SECF_FMC, RCC_AHB6SECSR, RCC_AHB6SECSR_FMCSECF),
	SECF(SECF_QSPI, RCC_AHB6SECSR, RCC_AHB6SECSR_QSPISECF),
	SECF(SECF_SDMMC1, RCC_AHB6SECSR, RCC_AHB6SECSR_SDMMC1SECF),
	SECF(SECF_SDMMC2, RCC_AHB6SECSR, RCC_AHB6SECSR_SDMMC2SECF),
	SECF(SECF_ETH1CK, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH1CKSECF),
	SECF(SECF_ETH1TX, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH1TXSECF),
	SECF(SECF_ETH1RX, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH1RXSECF),
	SECF(SECF_ETH1MAC, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH1MACSECF),
	SECF(SECF_ETH1STP, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH1STPSECF),
	SECF(SECF_ETH2CK, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH2CKSECF),
	SECF(SECF_ETH2TX, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH2TXSECF),
	SECF(SECF_ETH2RX, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH2RXSECF),
	SECF(SECF_ETH2MAC, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH2MACSECF),
	SECF(SECF_ETH2STP, RCC_AHB6SECSR, RCC_AHB6SECSR_ETH2STPSECF),
	SECF(SECF_MCO1, RCC_SECCFGR, RCC_SECCFGR_MCO1SEC),
	SECF(SECF_MCO2, RCC_SECCFGR, RCC_SECCFGR_MCO2SEC),
};

static const char * const adc12_src[] = {
	"pll4_r", "ck_per", "pll3_q"
};

static const char * const dcmipp_src[] = {
	"ck_axi", "pll2_q", "pll4_p", "ck_per",
};

static const char * const eth12_src[] = {
	"pll4_p", "pll3_q"
};

static const char * const fdcan_src[] = {
	"ck_hse", "pll3_q", "pll4_q", "pll4_r"
};

static const char * const fmc_src[] = {
	"ck_axi", "pll3_r", "pll4_p", "ck_per"
};

static const char * const i2c12_src[] = {
	"pclk1", "pll4_r", "ck_hsi", "ck_csi"
};

static const char * const i2c345_src[] = {
	"pclk6", "pll4_r", "ck_hsi", "ck_csi"
};

static const char * const lptim1_src[] = {
	"pclk1", "pll4_p", "pll3_q", "ck_lse", "ck_lsi", "ck_per"
};

static const char * const lptim23_src[] = {
	"pclk3", "pll4_q", "ck_per", "ck_lse", "ck_lsi"
};

static const char * const lptim45_src[] = {
	"pclk3", "pll4_p", "pll3_q", "ck_lse", "ck_lsi", "ck_per"
};

static const char * const mco1_src[] = {
	"ck_hsi", "ck_hse", "ck_csi", "ck_lsi", "ck_lse"
};

static const char * const mco2_src[] = {
	"ck_mpu", "ck_axi", "ck_mlahb", "pll4_p", "ck_hse", "ck_hsi"
};

static const char * const qspi_src[] = {
	"ck_axi", "pll3_r", "pll4_p", "ck_per"
};

static const char * const rng1_src[] = {
	"ck_csi", "pll4_r", "reserved", "ck_lsi"
};

static const char * const saes_src[] = {
	"ck_axi", "ck_per", "pll4_r", "ck_lsi"
};

static const char * const sai1_src[] = {
	"pll4_q", "pll3_q", "i2s_ckin", "ck_per", "pll3_r"
};

static const char * const sai2_src[] = {
	"pll4_q", "pll3_q", "i2s_ckin", "ck_per", "spdif_ck_symb", "pll3_r"
};

static const char * const sdmmc12_src[] = {
	"ck_axi", "pll3_r", "pll4_p", "ck_hsi"
};

static const char * const spdif_src[] = {
	"pll4_p", "pll3_q", "ck_hsi"
};

static const char * const spi123_src[] = {
	"pll4_p", "pll3_q", "i2s_ckin", "ck_per", "pll3_r"
};

static const char * const spi4_src[] = {
	"pclk6", "pll4_q", "ck_hsi", "ck_csi", "ck_hse", "i2s_ckin"
};

static const char * const spi5_src[] = {
	"pclk6", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const stgen_src[] = {
	"ck_hsi", "ck_hse"
};

static const char * const usart12_src[] = {
	"pclk6", "pll3_q", "ck_hsi", "ck_csi", "pll4_q", "ck_hse"
};

static const char * const usart34578_src[] = {
	"pclk1", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const usart6_src[] = {
	"pclk2", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const usbo_src[] = {
	"pll4_r", "ck_usbo_48m"
};

static const char * const usbphy_src[] = {
	"ck_hse", "pll4_r", "clk-hse-div2"
};

/* Timer clocks */
static struct clk_stm32_gate tim2_k = {
	.gate_id = GATE_TIM2,
	.hw.init = CLK_HW_INIT("tim2_k", "timg1_ck", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

static struct clk_stm32_gate tim3_k = {
	.gate_id = GATE_TIM3,
	.hw.init = CLK_HW_INIT("tim3_k", "timg1_ck", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

static struct clk_stm32_gate tim4_k = {
	.gate_id = GATE_TIM4,
	.hw.init = CLK_HW_INIT("tim4_k", "timg1_ck", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

static struct clk_stm32_gate tim5_k = {
	.gate_id = GATE_TIM5,
	.hw.init = CLK_HW_INIT("tim5_k", "timg1_ck", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

static struct clk_stm32_gate tim6_k = {
	.gate_id = GATE_TIM6,
	.hw.init = CLK_HW_INIT("tim6_k", "timg1_ck", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

static struct clk_stm32_gate tim7_k = {
	.gate_id = GATE_TIM7,
	.hw.init = CLK_HW_INIT("tim7_k", "timg1_ck", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

static struct clk_stm32_gate tim1_k = {
	.gate_id = GATE_TIM1,
	.hw.init = CLK_HW_INIT("tim1_k", "timg2_ck", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

static struct clk_stm32_gate tim8_k = {
	.gate_id = GATE_TIM8,
	.hw.init = CLK_HW_INIT("tim8_k", "timg2_ck", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

static struct clk_stm32_gate tim12_k = {
	.gate_id = GATE_TIM12,
	.hw.init = CLK_HW_INIT("tim12_k", "timg3_ck", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

static struct clk_stm32_gate tim13_k = {
	.gate_id = GATE_TIM13,
	.hw.init = CLK_HW_INIT("tim13_k", "timg3_ck", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

static struct clk_stm32_gate tim14_k = {
	.gate_id = GATE_TIM14,
	.hw.init = CLK_HW_INIT("tim14_k", "timg3_ck", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

static struct clk_stm32_gate tim15_k = {
	.gate_id = GATE_TIM15,
	.hw.init = CLK_HW_INIT("tim15_k", "timg3_ck", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

static struct clk_stm32_gate tim16_k = {
	.gate_id = GATE_TIM16,
	.hw.init = CLK_HW_INIT("tim16_k", "timg3_ck", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

static struct clk_stm32_gate tim17_k = {
	.gate_id = GATE_TIM17,
	.hw.init = CLK_HW_INIT("tim17_k", "timg3_ck", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

/* Peripheral clocks */
static struct clk_stm32_gate spi2 = {
	.gate_id = GATE_SPI2,
	.hw.init = CLK_HW_INIT("spi2", "pclk1", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate spi3 = {
	.gate_id = GATE_SPI3,
	.hw.init = CLK_HW_INIT("spi3", "pclk1", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate sai1 = {
	.gate_id = GATE_SAI1,
	.hw.init = CLK_HW_INIT("sai1", "pclk2", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate sai2 = {
	.gate_id = GATE_SAI2,
	.hw.init = CLK_HW_INIT("sai2", "pclk2", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate spi1 = {
	.gate_id = GATE_SPI1,
	.hw.init = CLK_HW_INIT("spi1", "pclk2", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate syscfg = {
	.gate_id = GATE_SYSCFG,
	.hw.init = CLK_HW_INIT("syscfg", "pclk3", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate vref = {
	.gate_id = GATE_VREF,
	.hw.init = CLK_HW_INIT("vref", "pclk3", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate dts = {
	.gate_id = GATE_DTS,
	.hw.init = CLK_HW_INIT("dts", "pclk3", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate pmbctrl = {
	.gate_id = GATE_PMBCTRL,
	.hw.init = CLK_HW_INIT("pmbctrl", "pclk3", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate hdp = {
	.gate_id = GATE_HDP,
	.hw.init = CLK_HW_INIT("hdp", "pclk3", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate iwdg2 = {
	.gate_id = GATE_IWDG2APB,
	.hw.init = CLK_HW_INIT("iwdg2", "pclk4", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate stgenro = {
	.gate_id = GATE_STGENRO,
	.hw.init = CLK_HW_INIT("stgenro", "pclk4", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate gpioa = {
	.gate_id = GATE_GPIOA,
	.hw.init = CLK_HW_INIT("gpioa", "pclk4", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate gpiob = {
	.gate_id = GATE_GPIOB,
	.hw.init = CLK_HW_INIT("gpiob", "pclk4", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate gpioc = {
	.gate_id = GATE_GPIOC,
	.hw.init = CLK_HW_INIT("gpioc", "pclk4", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate gpiod = {
	.gate_id = GATE_GPIOD,
	.hw.init = CLK_HW_INIT("gpiod", "pclk4", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate gpioe = {
	.gate_id = GATE_GPIOE,
	.hw.init = CLK_HW_INIT("gpioe", "pclk4", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate gpiof = {
	.gate_id = GATE_GPIOF,
	.hw.init = CLK_HW_INIT("gpiof", "pclk4", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate gpiog = {
	.gate_id = GATE_GPIOG,
	.hw.init = CLK_HW_INIT("gpiog", "pclk4", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate gpioh = {
	.gate_id = GATE_GPIOH,
	.hw.init = CLK_HW_INIT("gpioh", "pclk4", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate gpioi = {
	.gate_id = GATE_GPIOI,
	.hw.init = CLK_HW_INIT("gpioi", "pclk4", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate tsc = {
	.gate_id = GATE_TSC,
	.hw.init = CLK_HW_INIT("tsc", "pclk4", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate ddrperfm = {
	.gate_id = GATE_DDRPERFM,
	.hw.init = CLK_HW_INIT("ddrperfm", "pclk4", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate tzpc = {
	.gate_id = GATE_TZC,
	.hw.init = CLK_HW_INIT("tzpc", "pclk5", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate iwdg1 = {
	.gate_id = GATE_IWDG1APB,
	.hw.init = CLK_HW_INIT("iwdg1", "pclk5", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate bsec = {
	.gate_id = GATE_BSEC,
	.hw.init = CLK_HW_INIT("bsec", "pclk5", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate spi4 = {
	.gate_id = GATE_SPI4,
	.hw.init = CLK_HW_INIT("spi4", "pclk6", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate spi5 = {
	.gate_id = GATE_SPI5,
	.hw.init = CLK_HW_INIT("spi5", "pclk6", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate dma1 = {
	.gate_id = GATE_DMA1,
	.hw.init = CLK_HW_INIT("dma1", "ck_mlahb", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate dma2 = {
	.gate_id = GATE_DMA2,
	.hw.init = CLK_HW_INIT("dma2", "ck_mlahb", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate dmamux1 = {
	.gate_id = GATE_DMAMUX1,
	.hw.init = CLK_HW_INIT("dmamux1", "ck_mlahb", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate dma3 = {
	.gate_id = GATE_DMA3,
	.hw.init = CLK_HW_INIT("dma3", "ck_mlahb", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate dmamux2 = {
	.gate_id = GATE_DMAMUX2,
	.hw.init = CLK_HW_INIT("dmamux2", "ck_mlahb", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate adc1 = {
	.gate_id = GATE_ADC1,
	.hw.init = CLK_HW_INIT("adc1", "ck_mlahb", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate adc2 = {
	.gate_id = GATE_ADC2,
	.hw.init = CLK_HW_INIT("adc2", "ck_mlahb", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate pka = {
	.gate_id = GATE_PKA,
	.hw.init = CLK_HW_INIT("pka", "ck_axi", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate cryp1 = {
	.gate_id = GATE_CRYP1,
	.hw.init = CLK_HW_INIT("cryp1", "ck_axi", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate hash1 = {
	.gate_id = GATE_HASH1,
	.hw.init = CLK_HW_INIT("hash1", "ck_axi", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate bkpsram = {
	.gate_id = GATE_BKPSRAM,
	.hw.init = CLK_HW_INIT("bkpsram", "ck_axi", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate mdma = {
	.gate_id = GATE_MDMA,
	.hw.init = CLK_HW_INIT("mdma", "ck_axi", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate eth1tx = {
	.gate_id = GATE_ETH1TX,
	.hw.init = CLK_HW_INIT("eth1tx", "ck_axi", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate eth1rx = {
	.gate_id = GATE_ETH1RX,
	.hw.init = CLK_HW_INIT("eth1rx", "ck_axi", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate eth1mac = {
	.gate_id = GATE_ETH1MAC,
	.hw.init = CLK_HW_INIT("eth1mac", "ck_axi", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate eth2tx = {
	.gate_id = GATE_ETH2TX,
	.hw.init = CLK_HW_INIT("eth2tx", "ck_axi", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate eth2rx = {
	.gate_id = GATE_ETH2RX,
	.hw.init = CLK_HW_INIT("eth2rx", "ck_axi", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate eth2mac = {
	.gate_id = GATE_ETH2MAC,
	.hw.init = CLK_HW_INIT("eth2mac", "ck_axi", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate crc1 = {
	.gate_id = GATE_CRC1,
	.hw.init = CLK_HW_INIT("crc1", "ck_axi", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate usbh = {
	.gate_id = GATE_USBH,
	.hw.init = CLK_HW_INIT("usbh", "ck_axi", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate eth1stp = {
	.gate_id = GATE_ETH1STP,
	.hw.init = CLK_HW_INIT("eth1stp", "ck_axi", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate eth2stp = {
	.gate_id = GATE_ETH2STP,
	.hw.init = CLK_HW_INIT("eth2stp", "ck_axi", &clk_stm32_gate_ops, 0),
};

/* Kernel clocks */
static struct clk_stm32_composite sdmmc1_k = {
	.gate_id = GATE_SDMMC1,
	.mux_id = MUX_SDMMC1,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("sdmmc1_k", sdmmc12_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite sdmmc2_k = {
	.gate_id = GATE_SDMMC2,
	.mux_id = MUX_SDMMC2,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("sdmmc2_k", sdmmc12_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite fmc_k = {
	.gate_id = GATE_FMC,
	.mux_id = MUX_FMC,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("fmc_k", fmc_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite qspi_k = {
	.gate_id = GATE_QSPI,
	.mux_id = MUX_QSPI,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("qspi_k", qspi_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite spi2_k = {
	.gate_id = GATE_SPI2,
	.mux_id = MUX_SPI23,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("spi2_k", spi123_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite spi3_k = {
	.gate_id = GATE_SPI3,
	.mux_id = MUX_SPI23,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("spi3_k", spi123_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite i2c1_k = {
	.gate_id = GATE_I2C1,
	.mux_id = MUX_I2C12,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("i2c1_k", i2c12_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite i2c2_k = {
	.gate_id = GATE_I2C2,
	.mux_id = MUX_I2C12,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("i2c2_k", i2c12_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite lptim4_k = {
	.gate_id = GATE_LPTIM4,
	.mux_id = MUX_LPTIM45,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("lptim4_k", lptim45_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite lptim5_k = {
	.gate_id = GATE_LPTIM5,
	.mux_id = MUX_LPTIM45,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("lptim5_k", lptim45_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite usart3_k = {
	.gate_id = GATE_USART3,
	.mux_id = MUX_UART35,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("usart3_k", usart34578_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite uart5_k = {
	.gate_id = GATE_UART5,
	.mux_id = MUX_UART35,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("uart5_k", usart34578_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite uart7_k = {
	.gate_id = GATE_UART7,
	.mux_id = MUX_UART78,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("uart7_k", usart34578_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite uart8_k = {
	.gate_id = GATE_UART8,
	.mux_id = MUX_UART78,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("uart8_k", usart34578_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite sai1_k = {
	.gate_id = GATE_SAI1,
	.mux_id = MUX_SAI1,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("sai1_k", sai1_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite adfsdm_k = {
	.gate_id = GATE_ADFSDM,
	.mux_id = MUX_SAI1,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("adfsdm_k", sai1_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite sai2_k = {
	.gate_id = GATE_SAI2,
	.mux_id = MUX_SAI2,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("sai2_k", sai2_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite adc1_k = {
	.gate_id = GATE_ADC1,
	.mux_id = MUX_ADC1,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("adc1_k", adc12_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite adc2_k = {
	.gate_id = GATE_ADC2,
	.mux_id = MUX_ADC2,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("adc2_k", adc12_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite rng1_k = {
	.gate_id = GATE_RNG1,
	.mux_id = MUX_RNG1,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("rng1_k", rng1_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite usbphy_k = {
	.gate_id = GATE_USBPHY,
	.mux_id = MUX_USBPHY,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("usbphy_k", usbphy_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite stgen_k = {
	.gate_id = GATE_STGENC,
	.mux_id = MUX_STGEN,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("stgen_k", stgen_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite spdif_k = {
	.gate_id = GATE_SPDIF,
	.mux_id = MUX_SPDIF,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("spdif_k", spdif_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite spi1_k = {
	.gate_id = GATE_SPI1,
	.mux_id = MUX_SPI1,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("spi1_k", spi123_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite spi4_k = {
	.gate_id = GATE_SPI4,
	.mux_id = MUX_SPI4,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("spi4_k", spi4_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite spi5_k = {
	.gate_id = GATE_SPI5,
	.mux_id = MUX_SPI5,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("spi5_k", spi5_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite i2c3_k = {
	.gate_id = GATE_I2C3,
	.mux_id = MUX_I2C3,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("i2c3_k", i2c345_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite i2c4_k = {
	.gate_id = GATE_I2C4,
	.mux_id = MUX_I2C4,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("i2c4_k", i2c345_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite i2c5_k = {
	.gate_id = GATE_I2C5,
	.mux_id = MUX_I2C5,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("i2c5_k", i2c345_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite lptim1_k = {
	.gate_id = GATE_LPTIM1,
	.mux_id = MUX_LPTIM1,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("lptim1_k", lptim1_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite lptim2_k = {
	.gate_id = GATE_LPTIM2,
	.mux_id = MUX_LPTIM2,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("lptim2_k", lptim23_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite lptim3_k = {
	.gate_id = GATE_LPTIM3,
	.mux_id = MUX_LPTIM3,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("lptim3_k", lptim23_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite usart1_k = {
	.gate_id = GATE_USART1,
	.mux_id = MUX_UART1,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("usart1_k", usart12_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite usart2_k = {
	.gate_id = GATE_USART2,
	.mux_id = MUX_UART2,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("usart2_k", usart12_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite uart4_k = {
	.gate_id = GATE_UART4,
	.mux_id = MUX_UART4,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("uart4_k", usart34578_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite uart6_k = {
	.gate_id = GATE_USART6,
	.mux_id = MUX_UART6,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("uart6_k", usart6_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite fdcan_k = {
	.gate_id = GATE_FDCAN,
	.mux_id = MUX_FDCAN,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("fdcan_k", fdcan_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite dcmipp_k = {
	.gate_id = GATE_DCMIPP,
	.mux_id = MUX_DCMIPP,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("dcmipp_k", dcmipp_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite usbo_k = {
	.gate_id = GATE_USBO,
	.mux_id = MUX_USBO,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("usbo_k", usbo_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite saes_k = {
	.gate_id = GATE_SAES,
	.mux_id = MUX_SAES,
	.div_id = NO_STM32_DIV,
	.hw.init = CLK_HW_INIT_PARENTS("saes_k", saes_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_gate dfsdm_k = {
	.gate_id = GATE_DFSDM,
	.hw.init = CLK_HW_INIT("dfsdm_k", "ck_mlahb", &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_gate ltdc_px = {
	.gate_id = GATE_LTDC,
	.hw.init = CLK_HW_INIT("ltdc_px", "pll4_q", &clk_stm32_gate_ops, CLK_SET_RATE_PARENT),
};

static struct clk_stm32_mux ck_ker_eth1 = {
	.mux_id = MUX_ETH1,
	.hw.init = CLK_HW_INIT_PARENTS("ck_ker_eth1", eth12_src, &clk_stm32_mux_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_gate eth1ck_k = {
	.gate_id = GATE_ETH1CK,
	.hw.init = CLK_HW_INIT_HW("eth1ck_k", &ck_ker_eth1.hw, &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_div eth1ptp_k = {
	.div_id = DIV_ETH1PTP,
	.hw.init = CLK_HW_INIT_HW("eth1ptp_k", &ck_ker_eth1.hw, &clk_stm32_divider_ops,
				  CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_mux ck_ker_eth2 = {
	.mux_id = MUX_ETH2,
	.hw.init = CLK_HW_INIT_PARENTS("ck_ker_eth2", eth12_src, &clk_stm32_mux_ops,
					    CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_gate eth2ck_k = {
	.gate_id = GATE_ETH2CK,
	.hw.init = CLK_HW_INIT_HW("eth2ck_k", &ck_ker_eth2.hw, &clk_stm32_gate_ops, 0),
};

static struct clk_stm32_div eth2ptp_k = {
	.div_id = DIV_ETH2PTP,
	.hw.init = CLK_HW_INIT_HW("eth2ptp_k", &ck_ker_eth2.hw, &clk_stm32_divider_ops,
				  CLK_SET_RATE_NO_REPARENT),
};

static struct clk_stm32_composite ck_mco1 = {
	.gate_id = GATE_MCO1,
	.mux_id = MUX_MCO1,
	.div_id = DIV_MCO1,
	.hw.init = CLK_HW_INIT_PARENTS("ck_mco1", mco1_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT |
				       CLK_IGNORE_UNUSED),
};

static struct clk_stm32_composite ck_mco2 = {
	.gate_id = GATE_MCO2,
	.mux_id = MUX_MCO2,
	.div_id = DIV_MCO2,
	.hw.init = CLK_HW_INIT_PARENTS("ck_mco2", mco2_src, &clk_stm32_composite_ops,
				       CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_NO_REPARENT |
				       CLK_IGNORE_UNUSED),
};

/* Debug clocks */
static struct clk_stm32_gate ck_sys_dbg = {
	.gate_id = GATE_DBGCK,
	.hw.init = CLK_HW_INIT("ck_sys_dbg", "ck_axi", &clk_stm32_gate_ops, CLK_IS_CRITICAL),
};

static struct clk_stm32_composite ck_trace = {
	.gate_id = GATE_TRACECK,
	.mux_id = NO_STM32_MUX,
	.div_id = DIV_TRACE,
	.hw.init = CLK_HW_INIT("ck_trace", "ck_axi", &clk_stm32_composite_ops, CLK_IGNORE_UNUSED),
};

static const struct clock_config stm32mp13_clock_cfg[] = {
	/* Timer clocks */
	STM32_GATE_CFG(TIM2_K, tim2_k, SECF_NONE),
	STM32_GATE_CFG(TIM3_K, tim3_k, SECF_NONE),
	STM32_GATE_CFG(TIM4_K, tim4_k, SECF_NONE),
	STM32_GATE_CFG(TIM5_K, tim5_k, SECF_NONE),
	STM32_GATE_CFG(TIM6_K, tim6_k, SECF_NONE),
	STM32_GATE_CFG(TIM7_K, tim7_k, SECF_NONE),
	STM32_GATE_CFG(TIM1_K, tim1_k, SECF_NONE),
	STM32_GATE_CFG(TIM8_K, tim8_k, SECF_NONE),
	STM32_GATE_CFG(TIM12_K, tim12_k, SECF_TIM12),
	STM32_GATE_CFG(TIM13_K, tim13_k, SECF_TIM13),
	STM32_GATE_CFG(TIM14_K, tim14_k, SECF_TIM14),
	STM32_GATE_CFG(TIM15_K, tim15_k, SECF_TIM15),
	STM32_GATE_CFG(TIM16_K, tim16_k, SECF_TIM16),
	STM32_GATE_CFG(TIM17_K, tim17_k, SECF_TIM17),

	/* Peripheral clocks */
	STM32_GATE_CFG(SPI1, spi1, SECF_NONE),
	STM32_GATE_CFG(SPI2, spi2, SECF_NONE),
	STM32_GATE_CFG(SPI3, spi3, SECF_NONE),
	STM32_GATE_CFG(SPI4, spi4, SECF_SPI4),
	STM32_GATE_CFG(SPI5, spi5, SECF_SPI5),
	STM32_GATE_CFG(SAI1, sai1, SECF_NONE),
	STM32_GATE_CFG(SAI2, sai2, SECF_NONE),
	STM32_GATE_CFG(SYSCFG, syscfg, SECF_NONE),
	STM32_GATE_CFG(VREF, vref, SECF_VREF),
	STM32_GATE_CFG(DTS, dts, SECF_NONE),
	STM32_GATE_CFG(PMBCTRL, pmbctrl, SECF_NONE),
	STM32_GATE_CFG(HDP, hdp, SECF_NONE),
	STM32_GATE_CFG(IWDG2, iwdg2, SECF_NONE),
	STM32_GATE_CFG(STGENRO, stgenro, SECF_STGENRO),
	STM32_GATE_CFG(TZPC, tzpc, SECF_TZC),
	STM32_GATE_CFG(IWDG1, iwdg1, SECF_IWDG1),
	STM32_GATE_CFG(BSEC, bsec, SECF_BSEC),
	STM32_GATE_CFG(DMA1, dma1, SECF_NONE),
	STM32_GATE_CFG(DMA2, dma2, SECF_NONE),
	STM32_GATE_CFG(DMAMUX1, dmamux1, SECF_NONE),
	STM32_GATE_CFG(DMA3, dma3, SECF_DMA3),
	STM32_GATE_CFG(DMAMUX2, dmamux2, SECF_DMAMUX2),
	STM32_GATE_CFG(ADC1, adc1, SECF_ADC1),
	STM32_GATE_CFG(ADC2, adc2, SECF_ADC2),
	STM32_GATE_CFG(GPIOA, gpioa, SECF_NONE),
	STM32_GATE_CFG(GPIOB, gpiob, SECF_NONE),
	STM32_GATE_CFG(GPIOC, gpioc, SECF_NONE),
	STM32_GATE_CFG(GPIOD, gpiod, SECF_NONE),
	STM32_GATE_CFG(GPIOE, gpioe, SECF_NONE),
	STM32_GATE_CFG(GPIOF, gpiof, SECF_NONE),
	STM32_GATE_CFG(GPIOG, gpiog, SECF_NONE),
	STM32_GATE_CFG(GPIOH, gpioh, SECF_NONE),
	STM32_GATE_CFG(GPIOI, gpioi, SECF_NONE),
	STM32_GATE_CFG(TSC, tsc, SECF_TZC),
	STM32_GATE_CFG(PKA, pka, SECF_PKA),
	STM32_GATE_CFG(CRYP1, cryp1, SECF_CRYP1),
	STM32_GATE_CFG(HASH1, hash1, SECF_HASH1),
	STM32_GATE_CFG(BKPSRAM, bkpsram, SECF_BKPSRAM),
	STM32_GATE_CFG(MDMA, mdma, SECF_NONE),
	STM32_GATE_CFG(ETH1TX, eth1tx, SECF_ETH1TX),
	STM32_GATE_CFG(ETH1RX, eth1rx, SECF_ETH1RX),
	STM32_GATE_CFG(ETH1MAC, eth1mac, SECF_ETH1MAC),
	STM32_GATE_CFG(ETH2TX, eth2tx, SECF_ETH2TX),
	STM32_GATE_CFG(ETH2RX, eth2rx, SECF_ETH2RX),
	STM32_GATE_CFG(ETH2MAC, eth2mac, SECF_ETH2MAC),
	STM32_GATE_CFG(CRC1, crc1, SECF_NONE),
	STM32_GATE_CFG(USBH, usbh, SECF_NONE),
	STM32_GATE_CFG(DDRPERFM, ddrperfm, SECF_NONE),
	STM32_GATE_CFG(ETH1STP, eth1stp, SECF_ETH1STP),
	STM32_GATE_CFG(ETH2STP, eth2stp, SECF_ETH2STP),

	/* Kernel clocks */
	STM32_COMPOSITE_CFG(SDMMC1_K, sdmmc1_k, SECF_SDMMC1),
	STM32_COMPOSITE_CFG(SDMMC2_K, sdmmc2_k, SECF_SDMMC2),
	STM32_COMPOSITE_CFG(FMC_K, fmc_k, SECF_FMC),
	STM32_COMPOSITE_CFG(QSPI_K, qspi_k, SECF_QSPI),
	STM32_COMPOSITE_CFG(SPI2_K, spi2_k, SECF_NONE),
	STM32_COMPOSITE_CFG(SPI3_K, spi3_k, SECF_NONE),
	STM32_COMPOSITE_CFG(I2C1_K, i2c1_k, SECF_NONE),
	STM32_COMPOSITE_CFG(I2C2_K, i2c2_k, SECF_NONE),
	STM32_COMPOSITE_CFG(LPTIM4_K, lptim4_k, SECF_NONE),
	STM32_COMPOSITE_CFG(LPTIM5_K, lptim5_k, SECF_NONE),
	STM32_COMPOSITE_CFG(USART3_K, usart3_k, SECF_NONE),
	STM32_COMPOSITE_CFG(UART5_K, uart5_k, SECF_NONE),
	STM32_COMPOSITE_CFG(UART7_K, uart7_k, SECF_NONE),
	STM32_COMPOSITE_CFG(UART8_K, uart8_k, SECF_NONE),
	STM32_COMPOSITE_CFG(SAI1_K, sai1_k, SECF_NONE),
	STM32_COMPOSITE_CFG(SAI2_K, sai2_k, SECF_NONE),
	STM32_COMPOSITE_CFG(ADFSDM_K, adfsdm_k, SECF_NONE),
	STM32_COMPOSITE_CFG(ADC1_K, adc1_k, SECF_ADC1),
	STM32_COMPOSITE_CFG(ADC2_K, adc2_k, SECF_ADC2),
	STM32_COMPOSITE_CFG(RNG1_K, rng1_k, SECF_RNG1),
	STM32_COMPOSITE_CFG(USBPHY_K, usbphy_k, SECF_USBPHY),
	STM32_COMPOSITE_CFG(STGEN_K, stgen_k, SECF_STGENC),
	STM32_COMPOSITE_CFG(SPDIF_K, spdif_k, SECF_NONE),
	STM32_COMPOSITE_CFG(SPI1_K, spi1_k, SECF_NONE),
	STM32_COMPOSITE_CFG(SPI4_K, spi4_k, SECF_SPI4),
	STM32_COMPOSITE_CFG(SPI5_K, spi5_k, SECF_SPI5),
	STM32_COMPOSITE_CFG(I2C3_K, i2c3_k, SECF_I2C3),
	STM32_COMPOSITE_CFG(I2C4_K, i2c4_k, SECF_I2C4),
	STM32_COMPOSITE_CFG(I2C5_K, i2c5_k, SECF_I2C5),
	STM32_COMPOSITE_CFG(LPTIM1_K, lptim1_k, SECF_NONE),
	STM32_COMPOSITE_CFG(LPTIM2_K, lptim2_k, SECF_LPTIM2),
	STM32_COMPOSITE_CFG(LPTIM3_K, lptim3_k, SECF_LPTIM3),
	STM32_COMPOSITE_CFG(USART1_K, usart1_k, SECF_USART1),
	STM32_COMPOSITE_CFG(USART2_K, usart2_k, SECF_USART2),
	STM32_COMPOSITE_CFG(UART4_K, uart4_k, SECF_NONE),
	STM32_COMPOSITE_CFG(USART6_K, uart6_k, SECF_NONE),
	STM32_COMPOSITE_CFG(FDCAN_K, fdcan_k, SECF_NONE),
	STM32_COMPOSITE_CFG(DCMIPP_K, dcmipp_k, SECF_DCMIPP),
	STM32_COMPOSITE_CFG(USBO_K, usbo_k, SECF_USBO),
	STM32_COMPOSITE_CFG(SAES_K, saes_k, SECF_SAES),
	STM32_GATE_CFG(DFSDM_K, dfsdm_k, SECF_NONE),
	STM32_GATE_CFG(LTDC_PX, ltdc_px, SECF_NONE),

	STM32_MUX_CFG(NO_ID, ck_ker_eth1, SECF_ETH1CK),
	STM32_GATE_CFG(ETH1CK_K, eth1ck_k, SECF_ETH1CK),
	STM32_DIV_CFG(ETH1PTP_K, eth1ptp_k, SECF_ETH1CK),

	STM32_MUX_CFG(NO_ID, ck_ker_eth2, SECF_ETH2CK),
	STM32_GATE_CFG(ETH2CK_K, eth2ck_k, SECF_ETH2CK),
	STM32_DIV_CFG(ETH2PTP_K, eth2ptp_k, SECF_ETH2CK),

	STM32_GATE_CFG(CK_DBG, ck_sys_dbg, SECF_NONE),
	STM32_COMPOSITE_CFG(CK_TRACE, ck_trace, SECF_NONE),

	STM32_COMPOSITE_CFG(CK_MCO1, ck_mco1, SECF_MCO1),
	STM32_COMPOSITE_CFG(CK_MCO2, ck_mco2, SECF_MCO2),
};

static int stm32mp13_clock_is_provided_by_secure(struct device_node *np, void __iomem *base,
						 const struct clock_config *cfg)
{
	int sec_id = cfg->sec_id;

	if (sec_id != SECF_NONE) {
		const struct clk_stm32_securiy *secf;

		secf = &stm32mp13_security[sec_id];

		return !!(readl(base + secf->offset) & BIT(secf->bit_idx));
	}

	return 0;
}

struct multi_mux {
	struct clk_hw *hw1;
	struct clk_hw *hw2;
};

static struct multi_mux *stm32_mp13_multi_mux[MUX_NB] = {
	[MUX_SPI23]	= &(struct multi_mux){ &spi2_k.hw,	&spi3_k.hw },
	[MUX_I2C12]	= &(struct multi_mux){ &i2c1_k.hw,	&i2c2_k.hw },
	[MUX_LPTIM45]	= &(struct multi_mux){ &lptim4_k.hw,	&lptim5_k.hw },
	[MUX_UART35]	= &(struct multi_mux){ &usart3_k.hw,	&uart5_k.hw },
	[MUX_UART78]	= &(struct multi_mux){ &uart7_k.hw,	&uart8_k.hw },
	[MUX_SAI1]	= &(struct multi_mux){ &sai1_k.hw,	&adfsdm_k.hw },
};

static struct clk_hw *stm32mp13_is_multi_mux(struct clk_hw *hw)
{
	struct clk_stm32_composite *composite = to_clk_stm32_composite(hw);
	struct multi_mux *mmux = stm32_mp13_multi_mux[composite->mux_id];

	if (mmux) {
		if (!(mmux->hw1 == hw))
			return mmux->hw1;
		else
			return mmux->hw2;
	}

	return NULL;
}

static u16 stm32mp13_cpt_gate[GATE_NB];

#ifdef CONFIG_DEBUG_FS
static struct clock_summary clock_summary_mp13;
#endif

static struct clk_stm32_clock_data stm32mp13_clock_data = {
	.gate_cpt	= stm32mp13_cpt_gate,
	.gates		= stm32mp13_gates,
	.muxes		= stm32mp13_muxes,
	.dividers	= stm32mp13_dividers,
	.is_multi_mux	= stm32mp13_is_multi_mux,
};

static struct clk_stm32_reset_data stm32mp13_reset_data = {
	.nr_lines	= STM32MP1_RESET_ID_MASK,
	.clear_offset	= RCC_CLR_OFFSET,
	.reset_us	= 2,
};

static const struct stm32_rcc_match_data stm32mp13_data = {
	.tab_clocks	= stm32mp13_clock_cfg,
	.num_clocks	= ARRAY_SIZE(stm32mp13_clock_cfg),
	.clock_data	= &stm32mp13_clock_data,
	.check_security = &stm32mp13_clock_is_provided_by_secure,
	.maxbinding	= STM32MP1_LAST_CLK,
	.reset_data	= &stm32mp13_reset_data,
#ifdef CONFIG_DEBUG_FS
	.clock_summary	= &clock_summary_mp13,
#endif
};

static const struct of_device_id stm32mp13_match_data[] = {
	{
		.compatible = "st,stm32mp13-rcc",
		.data = &stm32mp13_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, stm32mp13_match_data);

static int stm32mp1_rcc_clocks_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *base;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (WARN_ON(IS_ERR(base)))
		return PTR_ERR(base);

	return stm32_rcc_init(dev, stm32mp13_match_data, base);
}

static struct platform_driver stm32mp13_rcc_clocks_driver = {
	.driver	= {
		.name = "stm32mp13_rcc",
		.of_match_table = stm32mp13_match_data,
	},
	.probe = stm32mp1_rcc_clocks_probe,
};

static int __init stm32mp13_clocks_init(void)
{
	return platform_driver_register(&stm32mp13_rcc_clocks_driver);
}
core_initcall(stm32mp13_clocks_init);

#ifdef CONFIG_DEBUG_FS

static bool cs_stm32_gate_is_enabled(struct clk_stm32_clock_data *data, struct clk_summary *c)
{
	return stm32_gate_is_enabled(data->base, data, c->gate_id);
}

static u8 cs_stm32_mux_get_parent(struct clk_stm32_clock_data *data, struct clk_summary *c)
{
	return stm32_mux_get_parent(data->base, data, c->mux_id);
}

static unsigned long cs_stm32_div_get_rate(struct clk_stm32_clock_data *data,
					   struct clk_summary *c,
					   unsigned long parent_rate)
{
	return stm32_divider_get_rate(data->base, data, c->div_id, parent_rate);
}

static unsigned long cs_stm32_get_rate_by_name(struct clk_stm32_clock_data *data,
					       struct clk_summary *c,
					       unsigned long parent_rate)
{
	struct clk *clk = __clk_lookup(c->name);

	if (clk)
		return clk_get_rate(clk);

	return 0;
}

static unsigned long cs_hsediv2_recalc_rate(struct clk_stm32_clock_data *data,
					    struct clk_summary *c,
					    unsigned long parent_rate)
{
	return parent_rate / 2;
}

/* STM32 PLL */
struct clk_pll_fractional_divider {
	struct clk_hw hw;
	void __iomem *mreg;
	u8 mshift;
	u8 mwidth;
	u8 mflags;
	void __iomem *nreg;
	u8 nshift;
	u8 nwidth;
	u8 nflags;
	void __iomem *freg;
	u8 fshift;
	u8 fwidth;
};

struct cs_pll {
	u32 offset;
};

#define PLL_BIT_ON		0
#define PLL_BIT_RDY		1
#define PLL_MUX_SHIFT		0
#define PLL_MUX_MASK		3
#define PLL_DIVMN_OFFSET	4
#define PLL_DIVM_SHIFT		16
#define PLL_DIVM_WIDTH		6
#define PLL_DIVN_SHIFT		0
#define PLL_DIVN_WIDTH		9
#define PLL_FRAC_OFFSET		0xC
#define PLL_FRAC_SHIFT		3
#define PLL_FRAC_WIDTH		13

static unsigned long cs_stm32_pll_recalc_rate(struct clk_stm32_clock_data *data,
					      struct clk_summary *c,
					      unsigned long parent_rate)
{
	struct cs_pll *pll = (struct cs_pll *)c->data;
	struct clk_pll_fractional_divider fracdiv;
	struct clk_pll_fractional_divider *fd = &fracdiv;
	void __iomem *reg;
	u32 mmask;
	u32 nmask;
	u32 fmask;
	unsigned long m, n, f;
	u64 rate, frate = 0;
	u32 val;

	reg = data->base + pll->offset;
	fd->mreg = reg + PLL_DIVMN_OFFSET;
	fd->mshift = PLL_DIVM_SHIFT;
	fd->mwidth = PLL_DIVM_WIDTH;
	fd->mflags = CLK_FRAC_DIVIDER_ZERO_BASED;
	fd->nreg = reg + PLL_DIVMN_OFFSET;
	fd->nshift = PLL_DIVN_SHIFT;
	fd->nwidth = PLL_DIVN_WIDTH;
	fd->nflags = CLK_FRAC_DIVIDER_ZERO_BASED;
	fd->freg = reg + PLL_FRAC_OFFSET;
	fd->fshift = PLL_FRAC_SHIFT;
	fd->fwidth = PLL_FRAC_WIDTH;

	mmask = GENMASK(fd->mwidth - 1, 0) << fd->mshift;
	nmask = GENMASK(fd->nwidth - 1, 0) << fd->nshift;
	fmask = GENMASK(fd->fwidth - 1, 0) << fd->fshift;

	val = readl(fd->mreg);
	m = (val & mmask) >> fd->mshift;
	if (fd->mflags & CLK_FRAC_DIVIDER_ZERO_BASED)
		m++;

	val = readl(fd->nreg);
	n = (val & nmask) >> fd->nshift;
	if (fd->nflags & CLK_FRAC_DIVIDER_ZERO_BASED)
		n++;

	if (!n || !m)
		return parent_rate;

	rate = (u64)parent_rate * n;
	do_div(rate, m);

	val = readl(fd->freg);
	f = (val & fmask) >> fd->fshift;
	if (f) {
		frate = (u64)parent_rate * (u64)f;
		do_div(frate, (m * (1 << fd->fwidth)));
	}

	return rate + frate;
}

/* The divider of RTC clock concerns only ck_hse clock */
#define HSE_RTC 3

static unsigned long cs_rtc_recalc_rate(struct clk_stm32_clock_data *data,
					struct clk_summary *c,
					unsigned long parent_rate)
{
	u8 parent;

	parent = stm32_mux_get_parent(data->base, data, c->mux_id);
	if (parent == HSE_RTC)
		return stm32_divider_get_rate(data->base, data, c->div_id, parent_rate);

	return parent_rate;
}

struct cs_stm32_timer {
	u32 apbdiv;
	u32 timpre;
};

#define APB_DIV_MASK 0x07
#define TIM_PRE_MASK 0x01

static unsigned long cs_stm32_timer_recalc_rate(struct clk_stm32_clock_data *data,
						struct clk_summary *c,
						unsigned long parent_rate)
{
	struct cs_stm32_timer *tim = (struct cs_stm32_timer *)c->data;
	void __iomem *rcc_base = data->base;
	u32 prescaler, timpre;

	prescaler = readl(rcc_base + tim->apbdiv) & APB_DIV_MASK;

	timpre = readl(rcc_base + tim->timpre) & TIM_PRE_MASK;

	if (prescaler == 0U)
		return parent_rate;

	return parent_rate * (timpre + 1U) * 2U;
}

#define CS_CLOCK(_name) (&cs_##_name)

#define CS_GATE(_name, _parent, _gate)						\
	static struct clk_summary cs_##_name = {				\
		.name		= #_name,					\
		.gate_id	= (_gate),					\
		.is_enabled	= cs_stm32_gate_is_enabled,			\
		.nb_parents	= 1,						\
		.clks		= (struct clk_summary *[]) {  &cs_##_parent },	\
	}

#define CS_MUX(_name, _parents, _mux)						\
	static struct clk_summary cs_##_name = {				\
		.name		= #_name,					\
		.gate_id	= NO_STM32_GATE,				\
		.mux_id		= (_mux),					\
		.div_id		= NO_STM32_DIV,					\
		.get_parent	= cs_stm32_mux_get_parent,			\
		.nb_parents	= ARRAY_SIZE(_parents),				\
		.clks		= _parents,					\
	}

#define CS_DIV(_name, _parent, _div)						\
	static struct clk_summary cs_##_name = {				\
		.name		= #_name,					\
		.gate_id	= NO_STM32_GATE,				\
		.mux_id		= NO_STM32_MUX,					\
		.div_id		= _div,						\
		.get_rate	= cs_stm32_div_get_rate,			\
		.nb_parents	= 1,						\
		.clks		= (struct clk_summary *[]) {  &cs_##_parent },	\
	}

#define CS_MUX_DIV(_name, _parents, _mux,  _div)				\
	static struct clk_summary cs_##_name = {				\
		.name		= #_name,					\
		.gate_id	= NO_STM32_GATE,				\
		.mux_id		= (_mux),					\
		.div_id		= (_div),					\
		.get_rate	= cs_stm32_div_get_rate,			\
		.get_parent	= cs_stm32_mux_get_parent,			\
		.nb_parents	= ARRAY_SIZE(_parents),				\
		.clks		= _parents,					\
	}

#define CS_GATE_MUX(_name, _parents, _gate, _mux)				\
	static struct clk_summary cs_##_name = {				\
		.name		= #_name,					\
		.gate_id	= (_gate),					\
		.mux_id		= (_mux),					\
		.div_id		= NO_STM32_DIV,					\
		.is_enabled	= cs_stm32_gate_is_enabled,			\
		.get_parent	= cs_stm32_mux_get_parent,			\
		.nb_parents	= ARRAY_SIZE(_parents),				\
		.clks		= _parents,					\
	}

#define CS_GATE_DIV(_name, _parent, _gate,  _div)				\
	static struct clk_summary cs_##_name = {				\
		.name		= #_name,					\
		.gate_id	= (_gate),					\
		.mux_id		= NO_STM32_MUX,					\
		.div_id		= (_div),					\
		.is_enabled	= cs_stm32_gate_is_enabled,			\
		.get_rate	= cs_stm32_div_get_rate,			\
		.nb_parents	= 1,						\
		.clks		= (struct clk_summary *[]) {  &cs_##_parent },	\
	}

#define CS_GATE_MUX_DIV(_name, _parents, _gate, _mux, _div)			\
	static struct clk_summary cs_##_name = {				\
		.name		= #_name,					\
		.gate_id	= (_gate),					\
		.mux_id		= (_mux),					\
		.div_id		= (_div),					\
		.is_enabled	= cs_stm32_gate_is_enabled,			\
		.get_rate	= cs_stm32_div_get_rate,			\
		.get_parent	= cs_stm32_mux_get_parent,			\
		.nb_parents	= ARRAY_SIZE(_parents),				\
		.clks		= _parents,					\
	}

#define CS_OSC_EXT(_name, _gate_id)						\
	static struct clk_summary cs_##_name = {				\
		.name		= #_name,					\
		.gate_id	= _gate_id,					\
		.mux_id		= NO_STM32_MUX,					\
		.div_id		= NO_STM32_DIV,					\
		.is_enabled	= cs_stm32_gate_is_enabled,			\
		.get_rate	= cs_stm32_get_rate_by_name,			\
		.nb_parents	= 0,						\
	}

#define CS_OSC_INT(_name, _gate_id) CS_OSC_EXT(_name, _gate_id)

#define CS_EXT(_name)								\
	static struct clk_summary cs_##_name = {				\
		.name		= #_name,					\
		.gate_id	= NO_STM32_GATE,				\
		.mux_id		= NO_STM32_MUX,					\
		.div_id		= NO_STM32_DIV,					\
		.get_rate	= cs_stm32_get_rate_by_name,			\
		.nb_parents	= 0,\
	}

#define CS_HSE_DIV2(_name, _parent)						\
	static struct clk_summary cs_##_name = {				\
		.name		= #_name,					\
		.gate_id	= NO_STM32_GATE,				\
		.mux_id		= NO_STM32_MUX,					\
		.div_id		= NO_STM32_DIV,					\
		.get_rate	= cs_hsediv2_recalc_rate,			\
		.nb_parents	= 1,						\
		.clks		= (struct clk_summary *[]) {  &cs_##_parent },	\
	}

#define CS_PLL(_name, _parents, _gate, _mux, _offset)				\
	static struct clk_summary cs_##_name = {				\
		.name		= #_name,					\
		.gate_id	= (_gate),					\
		.mux_id		= (_mux),					\
		.div_id		= NO_STM32_DIV,					\
		.data		=  &(struct cs_pll) {				\
			.offset		= _offset,				\
		},								\
		.is_enabled	= cs_stm32_gate_is_enabled,			\
		.get_rate	= cs_stm32_pll_recalc_rate,			\
		.get_parent	= cs_stm32_mux_get_parent,			\
		.nb_parents	= ARRAY_SIZE(_parents),				\
		.clks		= _parents,					\
	}

#define CS_STM32_TIMER(_name, _parent, _apbdiv, _timpre)			\
	static struct clk_summary cs_##_name = {				\
		.name		= #_name,					\
		.data		=  &(struct cs_stm32_timer) {			\
			.apbdiv		= _apbdiv,				\
			.timpre		= _timpre,				\
		},								\
		.get_rate	= cs_stm32_timer_recalc_rate,			\
		.nb_parents	= 1,						\
		.clks		= (struct clk_summary *[]) {  &cs_##_parent },	\
	}

#define CS_RTC(_name, _parents, _gate, _mux, _div)				\
	static struct clk_summary cs_##_name = {				\
		.name		= #_name,					\
		.gate_id	= (_gate),					\
		.mux_id		= (_mux),					\
		.div_id		= (_div),					\
		.is_enabled	= cs_stm32_gate_is_enabled,			\
		.get_rate	= cs_rtc_recalc_rate,				\
		.get_parent	= cs_stm32_mux_get_parent,			\
		.nb_parents	= ARRAY_SIZE(_parents),				\
		.clks		= _parents,					\
	}

CS_OSC_INT(ck_hsi, GATE_HSI);
CS_OSC_INT(ck_lsi, GATE_LSI);
CS_OSC_INT(ck_csi, GATE_CSI);
CS_OSC_EXT(ck_hse, GATE_HSE);

CS_OSC_EXT(ck_lse, GATE_LSE);

CS_EXT(ck_usbo_48m);
CS_EXT(i2s_ckin);
CS_EXT(spdif_ck_symb);

CS_HSE_DIV2(hse_div2_ck, ck_hse);

static struct clk_summary *cs_ref12_parents[] = {
	CS_CLOCK(ck_hsi), CS_CLOCK(ck_hse)
};

static struct clk_summary *cs_ref3_parents[] = {
	CS_CLOCK(ck_hsi), CS_CLOCK(ck_hse), CS_CLOCK(ck_csi)
};

static struct clk_summary *cs_ref4_parents[] = {
	CS_CLOCK(ck_hsi), CS_CLOCK(ck_hse), CS_CLOCK(ck_csi), CS_CLOCK(i2s_ckin)
};

CS_PLL(pll1, cs_ref12_parents, GATE_PLL1, MUX_PLL12, RCC_PLL1CR);
CS_GATE_DIV(pll1_p, pll1, GATE_PLL1_DIVP, DIV_PLL1DIVP);
CS_DIV(pll1_p_div, pll1_p, DIV_MPU);

CS_PLL(pll2, cs_ref12_parents, GATE_PLL2, MUX_PLL12, RCC_PLL2CR);
CS_GATE_DIV(pll2_p, pll2, GATE_PLL2_DIVP, DIV_PLL2DIVP);
CS_GATE_DIV(pll2_q, pll2, GATE_PLL2_DIVQ, DIV_PLL2DIVQ);
CS_GATE_DIV(pll2_r, pll2, GATE_PLL2_DIVR, DIV_PLL2DIVR);

CS_PLL(pll3, cs_ref3_parents, GATE_PLL3, MUX_PLL3, RCC_PLL3CR);
CS_GATE_DIV(pll3_p, pll3, GATE_PLL3_DIVP, DIV_PLL3DIVP);
CS_GATE_DIV(pll3_q, pll3, GATE_PLL3_DIVQ, DIV_PLL3DIVQ);
CS_GATE_DIV(pll3_r, pll3, GATE_PLL3_DIVR, DIV_PLL3DIVR);

CS_PLL(pll4, cs_ref4_parents, GATE_PLL4, MUX_PLL4, RCC_PLL4CR);
CS_GATE_DIV(pll4_p, pll4, GATE_PLL4_DIVP, DIV_PLL4DIVP);
CS_GATE_DIV(pll4_q, pll4, GATE_PLL4_DIVQ, DIV_PLL4DIVQ);
CS_GATE_DIV(pll4_r, pll4, GATE_PLL4_DIVR, DIV_PLL4DIVR);

static struct clk_summary *cs_cpu_src[] = {
	CS_CLOCK(ck_hsi), CS_CLOCK(ck_hse), CS_CLOCK(pll1_p), CS_CLOCK(pll1_p_div)
};

static struct clk_summary *cs_axi_src[] = {
	CS_CLOCK(ck_hsi), CS_CLOCK(ck_hse), CS_CLOCK(pll2_p)
};

static struct clk_summary *cs_mlahb_src[] = {
	CS_CLOCK(ck_hsi), CS_CLOCK(ck_hse), CS_CLOCK(ck_csi), CS_CLOCK(pll3_p)
};

static struct clk_summary *cs_per_src[] = {
	CS_CLOCK(ck_hsi), CS_CLOCK(ck_csi), CS_CLOCK(ck_hse)
};

static struct clk_summary *cs_rtc_src[] = {
	NULL, CS_CLOCK(ck_lse), CS_CLOCK(ck_lsi), CS_CLOCK(ck_hse)
};

CS_MUX(ck_mpu, cs_cpu_src, MUX_MPU);
CS_MUX(ck_axi, cs_axi_src, MUX_AXI);
CS_MUX(ck_mlahb, cs_mlahb_src, MUX_MLAHB);
CS_MUX(ck_per, cs_per_src, MUX_CKPER);

CS_DIV(pclk1, ck_mlahb, DIV_APB1);
CS_DIV(pclk2, ck_mlahb, DIV_APB2);
CS_DIV(pclk3, ck_mlahb, DIV_APB3);
CS_DIV(pclk4, ck_axi, DIV_APB4);
CS_DIV(pclk5, ck_axi, DIV_APB5);
CS_DIV(pclk6, ck_mlahb, DIV_APB6);

CS_STM32_TIMER(timg1_ck, pclk1, RCC_APB1DIVR, RCC_TIMG1PRER);
CS_STM32_TIMER(timg2_ck, pclk2, RCC_APB2DIVR, RCC_TIMG2PRER);
CS_STM32_TIMER(timg3_ck, pclk1, RCC_APB6DIVR, RCC_TIMG3PRER);

CS_GATE(tim2_k, timg1_ck, GATE_TIM2);
CS_GATE(tim3_k, timg1_ck, GATE_TIM3);
CS_GATE(tim4_k, timg1_ck, GATE_TIM4);
CS_GATE(tim5_k, timg1_ck, GATE_TIM5);
CS_GATE(tim6_k, timg1_ck, GATE_TIM6);
CS_GATE(tim7_k, timg1_ck, GATE_TIM7);
CS_GATE(tim1_k, timg2_ck, GATE_TIM1);
CS_GATE(tim8_k, timg2_ck, GATE_TIM8);
CS_GATE(tim12_k, timg3_ck, GATE_TIM12);
CS_GATE(tim13_k, timg3_ck, GATE_TIM13);
CS_GATE(tim14_k, timg3_ck, GATE_TIM14);
CS_GATE(tim15_k, timg3_ck, GATE_TIM15);
CS_GATE(tim16_k, timg3_ck, GATE_TIM16);
CS_GATE(tim17_k, timg3_ck, GATE_TIM17);
CS_GATE(spi2, pclk1, GATE_SPI2);
CS_GATE(spi3, pclk1, GATE_SPI3);
CS_GATE(sai1, pclk2, GATE_SAI1);
CS_GATE(sai2, pclk2, GATE_SAI2);
CS_GATE(spi1, pclk2, GATE_SPI1);
CS_GATE(syscfg, pclk3, GATE_SYSCFG);
CS_GATE(vref, pclk3, GATE_VREF);
CS_GATE(dts, pclk3, GATE_DTS);
CS_GATE(pmbctrl, pclk3, GATE_PMBCTRL);
CS_GATE(hdp, pclk3, GATE_HDP);
CS_GATE(iwdg2, pclk4, GATE_IWDG2APB);
CS_GATE(stgenro, pclk4, GATE_STGENRO);
CS_GATE(gpioa, pclk4, GATE_GPIOA);
CS_GATE(gpiob, pclk4, GATE_GPIOB);
CS_GATE(gpioc, pclk4, GATE_GPIOC);
CS_GATE(gpiod, pclk4, GATE_GPIOD);
CS_GATE(gpioe, pclk4, GATE_GPIOE);
CS_GATE(gpiof, pclk4, GATE_GPIOF);
CS_GATE(gpiog, pclk4, GATE_GPIOG);
CS_GATE(gpioh, pclk4, GATE_GPIOH);
CS_GATE(gpioi, pclk4, GATE_GPIOI);
CS_GATE(tsc, pclk4, GATE_TSC);
CS_GATE(ddrperfm, pclk4, GATE_DDRPERFM);
CS_GATE(tzpc, pclk5, GATE_TZC);
CS_GATE(iwdg1, pclk5, GATE_IWDG1APB);
CS_GATE(bsec, pclk5, GATE_BSEC);
CS_GATE(spi4, pclk6, GATE_SPI4);
CS_GATE(spi5, pclk6, GATE_SPI5);
CS_GATE(dma1, ck_mlahb, GATE_DMA1);
CS_GATE(dma2, ck_mlahb, GATE_DMA2);
CS_GATE(dmamux1, ck_mlahb, GATE_DMAMUX1);
CS_GATE(dma3, ck_mlahb, GATE_DMA3);
CS_GATE(dmamux2, ck_mlahb, GATE_DMAMUX2);
CS_GATE(adc1, ck_mlahb, GATE_ADC1);
CS_GATE(adc2, ck_mlahb, GATE_ADC2);
CS_GATE(pka, ck_axi, GATE_PKA);
CS_GATE(cryp1, ck_axi, GATE_CRYP1);
CS_GATE(hash1, ck_axi, GATE_HASH1);
CS_GATE(bkpsram, ck_axi, GATE_BKPSRAM);
CS_GATE(mdma, ck_axi, GATE_MDMA);
CS_GATE(eth1tx, ck_axi, GATE_ETH1TX);
CS_GATE(eth1rx, ck_axi, GATE_ETH1RX);
CS_GATE(eth1mac, ck_axi, GATE_ETH1MAC);
CS_GATE(eth2tx, ck_axi, GATE_ETH2TX);
CS_GATE(eth2rx, ck_axi, GATE_ETH2RX);
CS_GATE(eth2mac, ck_axi, GATE_ETH2MAC);
CS_GATE(crc1, ck_axi, GATE_CRC1);
CS_GATE(usbh, ck_axi, GATE_USBH);
CS_GATE(eth1stp, ck_axi, GATE_ETH1STP);
CS_GATE(eth2stp, ck_axi, GATE_ETH2STP);
CS_GATE(dfsdm_k, ck_mlahb, GATE_DFSDM);
CS_GATE(ltdc_px, pll4_q, GATE_LTDC);
CS_GATE(ck_sys_dbg, ck_axi, GATE_DBGCK);
CS_GATE(rtcapb, pclk5, GATE_RTCAPB);
CS_GATE_DIV(ck_trace, ck_axi, GATE_TRACECK, DIV_TRACE);

static struct clk_summary *cs_adc12_src[] = {
	CS_CLOCK(pll4_r), CS_CLOCK(ck_per), CS_CLOCK(pll3_q)
};

static struct clk_summary *cs_dcmipp_src[] = {
	CS_CLOCK(ck_axi), CS_CLOCK(pll2_q), CS_CLOCK(pll4_p), CS_CLOCK(ck_per)
};

static struct clk_summary *cs_eth12_src[] = {
	CS_CLOCK(pll4_p), CS_CLOCK(pll3_q)
};

static struct clk_summary *cs_fdcan_src[] = {
	CS_CLOCK(ck_hse), CS_CLOCK(pll3_q), CS_CLOCK(pll4_q), CS_CLOCK(pll4_r)
};

static struct clk_summary *cs_fmc_src[] = {
	CS_CLOCK(ck_axi), CS_CLOCK(pll3_r), CS_CLOCK(pll4_p), CS_CLOCK(ck_per)
};

static struct clk_summary *cs_i2c12_src[] = {
	CS_CLOCK(pclk1), CS_CLOCK(pll4_r), CS_CLOCK(ck_hsi), CS_CLOCK(ck_csi)
};

static struct clk_summary *cs_i2c345_src[] = {
	CS_CLOCK(pclk6), CS_CLOCK(pll4_r), CS_CLOCK(ck_hsi), CS_CLOCK(ck_csi)
};

static struct clk_summary *cs_lptim1_src[] = {
	CS_CLOCK(pclk1), CS_CLOCK(pll4_p), CS_CLOCK(pll3_q),
	CS_CLOCK(ck_lse), CS_CLOCK(ck_lsi), CS_CLOCK(ck_per)
};

static struct clk_summary *cs_lptim23_src[] = {
	CS_CLOCK(pclk3), CS_CLOCK(pll4_q), CS_CLOCK(ck_per), CS_CLOCK(ck_lse), CS_CLOCK(ck_lsi)
};

static struct clk_summary *cs_lptim45_src[] = {
	CS_CLOCK(pclk3), CS_CLOCK(pll4_p), CS_CLOCK(pll3_q),
	CS_CLOCK(ck_lse), CS_CLOCK(ck_lsi), CS_CLOCK(ck_per)
};

static struct clk_summary *cs_mco1_src[] = {
	CS_CLOCK(ck_hsi), CS_CLOCK(ck_hse), CS_CLOCK(ck_csi), CS_CLOCK(ck_lsi), CS_CLOCK(ck_lse)
};

static struct clk_summary *cs_mco2_src[] = {
	CS_CLOCK(ck_mpu), CS_CLOCK(ck_axi), CS_CLOCK(ck_mlahb),
	CS_CLOCK(pll4_p), CS_CLOCK(ck_hse), CS_CLOCK(ck_hsi)
};

static struct clk_summary *cs_qspi_src[] = {
	CS_CLOCK(ck_axi), CS_CLOCK(pll3_r), CS_CLOCK(pll4_p), CS_CLOCK(ck_per)
};

static struct clk_summary *cs_rng1_src[] = {
	CS_CLOCK(ck_csi), CS_CLOCK(pll4_r), NULL, CS_CLOCK(ck_lsi)
};

static struct clk_summary *cs_saes_src[] = {
	CS_CLOCK(ck_axi), CS_CLOCK(ck_per), CS_CLOCK(pll4_r), CS_CLOCK(ck_lsi)
};

static struct clk_summary *cs_sai1_src[] = {
	CS_CLOCK(pll4_q), CS_CLOCK(pll3_q), CS_CLOCK(i2s_ckin), CS_CLOCK(ck_per), CS_CLOCK(pll3_r)
};

static struct clk_summary *cs_sai2_src[] = {
	CS_CLOCK(pll4_q), CS_CLOCK(pll3_q), CS_CLOCK(i2s_ckin),
	CS_CLOCK(ck_per), CS_CLOCK(spdif_ck_symb), CS_CLOCK(pll3_r)
};

static struct clk_summary *cs_sdmmc12_src[] = {
	CS_CLOCK(ck_axi), CS_CLOCK(pll3_r), CS_CLOCK(pll4_p), CS_CLOCK(ck_hsi)
};

static struct clk_summary *cs_spdif_src[] = {
	CS_CLOCK(pll4_p), CS_CLOCK(pll3_q), CS_CLOCK(ck_hsi)
};

static struct clk_summary *cs_spi123_src[] = {
	CS_CLOCK(pll4_p), CS_CLOCK(pll3_q), CS_CLOCK(i2s_ckin), CS_CLOCK(ck_per), CS_CLOCK(pll3_r)
};

static struct clk_summary *cs_spi4_src[] = {
	CS_CLOCK(pclk6), CS_CLOCK(pll4_q), CS_CLOCK(ck_hsi),
	CS_CLOCK(ck_csi), CS_CLOCK(ck_hse), CS_CLOCK(i2s_ckin)
};

static struct clk_summary *cs_spi5_src[] = {
	CS_CLOCK(pclk6), CS_CLOCK(pll4_q), CS_CLOCK(ck_hsi), CS_CLOCK(ck_csi), CS_CLOCK(ck_hse)
};

static struct clk_summary *cs_stgen_src[] = {
	CS_CLOCK(ck_hsi), CS_CLOCK(ck_hse)
};

static struct clk_summary *cs_usart12_src[] = {
	CS_CLOCK(pclk6), CS_CLOCK(pll3_q), CS_CLOCK(ck_hsi),
	CS_CLOCK(ck_csi), CS_CLOCK(pll4_q), CS_CLOCK(ck_hse)
};

static struct clk_summary *cs_usart34578_src[] = {
	CS_CLOCK(pclk1), CS_CLOCK(pll4_q), CS_CLOCK(ck_hsi), CS_CLOCK(ck_csi), CS_CLOCK(ck_hse)
};

static struct clk_summary *cs_usart6_src[] = {
	CS_CLOCK(pclk2), CS_CLOCK(pll4_q), CS_CLOCK(ck_hsi), CS_CLOCK(ck_csi), CS_CLOCK(ck_hse)
};

static struct clk_summary *cs_usbo_src[] = {
	CS_CLOCK(pll4_r), CS_CLOCK(ck_usbo_48m)
};

static struct clk_summary *cs_usbphy_src[] = {
	CS_CLOCK(ck_hse), CS_CLOCK(pll4_r), CS_CLOCK(hse_div2_ck)
};

CS_GATE_MUX(sdmmc1_k, cs_sdmmc12_src, GATE_SDMMC1, MUX_SDMMC1);
CS_GATE_MUX(sdmmc2_k, cs_sdmmc12_src, GATE_SDMMC2, MUX_SDMMC2);
CS_GATE_MUX(fmc_k, cs_fmc_src, GATE_FMC, MUX_FMC);
CS_GATE_MUX(qspi_k, cs_qspi_src, GATE_QSPI, MUX_QSPI);
CS_GATE_MUX(spi2_k, cs_spi123_src, GATE_SPI2, MUX_SPI23);
CS_GATE_MUX(spi3_k, cs_spi123_src, GATE_SPI3, MUX_SPI23);
CS_GATE_MUX(i2c1_k, cs_i2c12_src, GATE_I2C1, MUX_I2C12);
CS_GATE_MUX(i2c2_k, cs_i2c12_src, GATE_I2C2, MUX_I2C12);
CS_GATE_MUX(lptim4_k, cs_lptim45_src, GATE_LPTIM4, MUX_LPTIM45);
CS_GATE_MUX(lptim5_k, cs_lptim45_src, GATE_LPTIM5, MUX_LPTIM45);
CS_GATE_MUX(usart3_k, cs_usart34578_src, GATE_USART3, MUX_UART35);
CS_GATE_MUX(uart5_k, cs_usart34578_src, GATE_UART5, MUX_UART35);
CS_GATE_MUX(uart7_k, cs_usart34578_src, GATE_UART7, MUX_UART78);
CS_GATE_MUX(uart8_k, cs_usart34578_src, GATE_UART8, MUX_UART78);
CS_GATE_MUX(sai1_k, cs_sai1_src, GATE_SAI1, MUX_SAI1);
CS_GATE_MUX(adfsdm_k, cs_sai1_src, GATE_ADFSDM, MUX_SAI1);
CS_GATE_MUX(sai2_k, cs_sai2_src, GATE_SAI2, MUX_SAI2);
CS_GATE_MUX(adc1_k, cs_adc12_src, GATE_ADC1, MUX_ADC1);
CS_GATE_MUX(adc2_k, cs_adc12_src, GATE_ADC2, MUX_ADC2);
CS_GATE_MUX(rng1_k, cs_rng1_src, GATE_RNG1, MUX_RNG1);
CS_GATE_MUX(usbphy_k, cs_usbphy_src, GATE_USBPHY, MUX_USBPHY);
CS_GATE_MUX(stgen_k, cs_stgen_src, GATE_STGENC, MUX_STGEN);
CS_GATE_MUX(spdif_k, cs_spdif_src, GATE_SPDIF, MUX_SPDIF);
CS_GATE_MUX(spi1_k, cs_spi123_src, GATE_SPI1, MUX_SPI1);
CS_GATE_MUX(spi4_k, cs_spi4_src, GATE_SPI4, MUX_SPI4);
CS_GATE_MUX(spi5_k, cs_spi5_src, GATE_SPI5, MUX_SPI5);
CS_GATE_MUX(i2c3_k, cs_i2c345_src, GATE_I2C3, MUX_I2C3);
CS_GATE_MUX(i2c4_k, cs_i2c345_src, GATE_I2C4, MUX_I2C4);
CS_GATE_MUX(i2c5_k, cs_i2c345_src, GATE_I2C5, MUX_I2C5);
CS_GATE_MUX(lptim1_k, cs_lptim1_src, GATE_LPTIM1, MUX_LPTIM1);
CS_GATE_MUX(lptim2_k, cs_lptim23_src, GATE_LPTIM2, MUX_LPTIM2);
CS_GATE_MUX(lptim3_k, cs_lptim23_src, GATE_LPTIM3, MUX_LPTIM3);
CS_GATE_MUX(usart1_k, cs_usart12_src, GATE_USART1, MUX_UART1);
CS_GATE_MUX(usart2_k, cs_usart12_src, GATE_USART2, MUX_UART2);
CS_GATE_MUX(uart4_k, cs_usart34578_src, GATE_UART4, MUX_UART4);
CS_GATE_MUX(uart6_k, cs_usart6_src, GATE_USART6, MUX_UART6);
CS_GATE_MUX(fdcan_k, cs_fdcan_src, GATE_FDCAN, MUX_FDCAN);
CS_GATE_MUX(dcmipp_k, cs_dcmipp_src, GATE_DCMIPP, MUX_DCMIPP);
CS_GATE_MUX(usbo_k, cs_usbo_src, GATE_USBO, MUX_USBO);
CS_GATE_MUX(eth1ck_k, cs_eth12_src, GATE_ETH1CK, MUX_ETH1);
CS_GATE_MUX(eth2ck_k, cs_eth12_src, GATE_ETH2CK, MUX_ETH2);
CS_GATE_MUX(saes_k, cs_saes_src, GATE_SAES, MUX_SAES);

CS_MUX_DIV(eth1ptp_k, cs_eth12_src, MUX_ETH1, DIV_ETH1PTP);
CS_MUX_DIV(eth2ptp_k, cs_eth12_src, MUX_ETH2, DIV_ETH2PTP);

CS_GATE_MUX_DIV(ck_mco1, cs_mco1_src, GATE_MCO1, MUX_MCO1, DIV_MCO1);
CS_GATE_MUX_DIV(ck_mco2, cs_mco2_src, GATE_MCO2, MUX_MCO2, DIV_MCO2);

CS_RTC(ck_rtc, cs_rtc_src, GATE_RTCCK, MUX_RTC, DIV_RTC);

static struct clk_summary *stm32mp13_clock_summary[] = {
	CS_CLOCK(ck_hsi),
	CS_CLOCK(ck_lsi),
	CS_CLOCK(ck_csi),
	CS_CLOCK(ck_hse),
	CS_CLOCK(ck_lse),
	CS_CLOCK(ck_usbo_48m),
	CS_CLOCK(i2s_ckin),
	CS_CLOCK(spdif_ck_symb),
	CS_CLOCK(hse_div2_ck),
	CS_CLOCK(pll1),
	CS_CLOCK(pll1_p),
	CS_CLOCK(pll1_p_div),
	CS_CLOCK(pll2),
	CS_CLOCK(pll2_p),
	CS_CLOCK(pll2_q),
	CS_CLOCK(pll2_r),
	CS_CLOCK(pll3),
	CS_CLOCK(pll3_p),
	CS_CLOCK(pll3_q),
	CS_CLOCK(pll3_r),
	CS_CLOCK(pll4),
	CS_CLOCK(pll4_p),
	CS_CLOCK(pll4_q),
	CS_CLOCK(pll4_r),
	CS_CLOCK(ck_mpu),
	CS_CLOCK(ck_axi),
	CS_CLOCK(ck_mlahb),
	CS_CLOCK(ck_per),
	CS_CLOCK(pclk1),
	CS_CLOCK(pclk2),
	CS_CLOCK(pclk3),
	CS_CLOCK(pclk4),
	CS_CLOCK(pclk5),
	CS_CLOCK(pclk6),
	CS_CLOCK(timg1_ck),
	CS_CLOCK(timg2_ck),
	CS_CLOCK(timg3_ck),
	CS_CLOCK(tim2_k),
	CS_CLOCK(tim3_k),
	CS_CLOCK(tim4_k),
	CS_CLOCK(tim5_k),
	CS_CLOCK(tim6_k),
	CS_CLOCK(tim7_k),
	CS_CLOCK(tim1_k),
	CS_CLOCK(tim8_k),
	CS_CLOCK(tim12_k),
	CS_CLOCK(tim13_k),
	CS_CLOCK(tim14_k),
	CS_CLOCK(tim15_k),
	CS_CLOCK(tim16_k),
	CS_CLOCK(tim17_k),
	CS_CLOCK(spi2),
	CS_CLOCK(spi3),
	CS_CLOCK(sai1),
	CS_CLOCK(sai2),
	CS_CLOCK(spi1),
	CS_CLOCK(syscfg),
	CS_CLOCK(vref),
	CS_CLOCK(dts),
	CS_CLOCK(pmbctrl),
	CS_CLOCK(hdp),
	CS_CLOCK(iwdg2),
	CS_CLOCK(stgenro),
	CS_CLOCK(gpioa),
	CS_CLOCK(gpiob),
	CS_CLOCK(gpioc),
	CS_CLOCK(gpiod),
	CS_CLOCK(gpioe),
	CS_CLOCK(gpiof),
	CS_CLOCK(gpiog),
	CS_CLOCK(gpioh),
	CS_CLOCK(gpioi),
	CS_CLOCK(tsc),
	CS_CLOCK(ddrperfm),
	CS_CLOCK(tzpc),
	CS_CLOCK(iwdg1),
	CS_CLOCK(bsec),
	CS_CLOCK(spi4),
	CS_CLOCK(spi5),
	CS_CLOCK(dma1),
	CS_CLOCK(dma2),
	CS_CLOCK(dmamux1),
	CS_CLOCK(dma3),
	CS_CLOCK(dmamux2),
	CS_CLOCK(adc1),
	CS_CLOCK(adc2),
	CS_CLOCK(pka),
	CS_CLOCK(cryp1),
	CS_CLOCK(hash1),
	CS_CLOCK(bkpsram),
	CS_CLOCK(mdma),
	CS_CLOCK(eth1tx),
	CS_CLOCK(eth1rx),
	CS_CLOCK(eth1mac),
	CS_CLOCK(eth2tx),
	CS_CLOCK(eth2rx),
	CS_CLOCK(eth2mac),
	CS_CLOCK(crc1),
	CS_CLOCK(usbh),
	CS_CLOCK(eth1stp),
	CS_CLOCK(eth2stp),
	CS_CLOCK(dfsdm_k),
	CS_CLOCK(ltdc_px),
	CS_CLOCK(ck_sys_dbg),
	CS_CLOCK(rtcapb),
	CS_CLOCK(ck_trace),
	CS_CLOCK(sdmmc1_k),
	CS_CLOCK(sdmmc2_k),
	CS_CLOCK(fmc_k),
	CS_CLOCK(qspi_k),
	CS_CLOCK(spi2_k),
	CS_CLOCK(spi3_k),
	CS_CLOCK(i2c1_k),
	CS_CLOCK(i2c2_k),
	CS_CLOCK(lptim4_k),
	CS_CLOCK(lptim5_k),
	CS_CLOCK(usart3_k),
	CS_CLOCK(uart5_k),
	CS_CLOCK(uart7_k),
	CS_CLOCK(uart8_k),
	CS_CLOCK(sai1_k),
	CS_CLOCK(adfsdm_k),
	CS_CLOCK(sai2_k),
	CS_CLOCK(adc1_k),
	CS_CLOCK(adc2_k),
	CS_CLOCK(rng1_k),
	CS_CLOCK(usbphy_k),
	CS_CLOCK(stgen_k),
	CS_CLOCK(spdif_k),
	CS_CLOCK(spi1_k),
	CS_CLOCK(spi4_k),
	CS_CLOCK(spi5_k),
	CS_CLOCK(i2c3_k),
	CS_CLOCK(i2c4_k),
	CS_CLOCK(i2c5_k),
	CS_CLOCK(lptim1_k),
	CS_CLOCK(lptim2_k),
	CS_CLOCK(lptim3_k),
	CS_CLOCK(usart1_k),
	CS_CLOCK(usart2_k),
	CS_CLOCK(uart4_k),
	CS_CLOCK(uart6_k),
	CS_CLOCK(fdcan_k),
	CS_CLOCK(dcmipp_k),
	CS_CLOCK(usbo_k),
	CS_CLOCK(eth1ck_k),
	CS_CLOCK(eth2ck_k),
	CS_CLOCK(saes_k),
	CS_CLOCK(eth1ptp_k),
	CS_CLOCK(eth2ptp_k),
	CS_CLOCK(ck_mco1),
	CS_CLOCK(ck_mco2),
	CS_CLOCK(ck_rtc),
};

static struct clock_summary clock_summary_mp13 = {
	.clocks		= stm32mp13_clock_summary,
	.nb_clocks	= ARRAY_SIZE(stm32mp13_clock_summary),
};

#endif
