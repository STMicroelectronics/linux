/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ST PCIe driver definitions for STM32-MP25 SoC
 *
 * Copyright (C) 2023 ST Microelectronics - All Rights Reserved
 * Author: Christian Bruel <christian.bruel@foss.st.com>
 */

#define to_stm32_pcie(x)	dev_get_drvdata((x)->dev)

#define STM32MP25_PCIECR_TYPE_MASK	GENMASK(11, 8)
#define STM32MP25_PCIECR_EP		0
#define STM32MP25_PCIECR_RC		BIT(10)
#define STM32MP25_PCIECR_REQ_RETRY_EN	BIT(3)
#define STM32MP25_PCIECR_LTSSM_EN	BIT(2)

#define SYSCFG_PCIECR			0x6000
#define SYSCFG_PCIEPMEMSICR		0x6004
#define SYSCFG_PCIEAERRCMSICR		0x6008
#define SYSCFG_PCIESR1			0x6100
#define SYSCFG_PCIESR2			0x6104

#define PCIE_CAP_MAX_PAYLOAD_SIZE(x)	((x) << 5)
#define PCIE_CAP_MAX_READ_REQ_SIZE(x)	((x) << 12)
