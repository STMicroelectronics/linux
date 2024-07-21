/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * This file discribe the STM32 MDF IIO driver API for audio part
 *
 * Copyright (C) 2023, STMicroelectronics.
 * Author(s): Olivier Moysan <olivier.moysan@foss.st.com>.
 */

#ifndef STM32_MDF_ADC_H
#define STM32_MDF_ADC_H

#include <linux/iio/iio.h>

int stm32_mdf_get_buff_cb(struct iio_dev *iio_dev,
			  int (*cb)(const void *data, size_t size, void *private), void *private);
int stm32_mdf_release_buff_cb(struct iio_dev *iio_dev);

#endif
