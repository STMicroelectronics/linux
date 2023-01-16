/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * This file describe the STM32 MDF IIO driver API for audio part
 *
 * Copyright (C) 2023, STMicroelectronics.
 * Author(s): Olivier Moysan <olivier.moysan@foss.st.com>.
 */

#ifndef STM32_MDF_ADC_H
#define STM32_MDF_ADC_H

#include <linux/iio/iio.h>

/*
 * Size of the buffer used to read IIO channel extended info.
 * "sub_channels_nb" info maximum value corresponds to the MDF filter number.
 * Use 3 bytes for 2 digit numbers plus the null trailing character.
 */
#define STM32_MDF_EXT_INFO_BUZ_SZ 3

int stm32_mdf_get_buff_cb(struct iio_dev *iio_dev,
			  int (*cb)(const void *data, size_t size, void *private), void *private);
int stm32_mdf_release_buff_cb(struct iio_dev *iio_dev);

#endif
