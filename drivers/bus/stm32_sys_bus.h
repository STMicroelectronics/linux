/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Copyright (C) STMicroelectronics 2022 - All Rights Reserved
 */
#ifndef __ARCH_ARM_MACH_STM32MP25_SYSTEM_BUS
#define __ARCH_ARM_MACH_STM32MP25_SYSTEM_BUS

#include <linux/types.h>

int stm32_rifsc_get_access_by_id(u32 id);
int stm32_rifsc_check_access_by_id(u32 id);

#endif /* __ARCH_ARM_MACH_STM32MP25_SYSTEM_BUS */
