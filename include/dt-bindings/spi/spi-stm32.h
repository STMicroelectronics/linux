/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * This header provides constants for STM32_SPI bindings.
 */

#ifndef _DT_BINDINGS_SPI_SPI_STM32_H
#define _DT_BINDINGS_SPI_SPI_STM32_H

/* st,spi-slave-underrun first parameter */
#define SPI_NO_ACTION			        0
#define SPI_SEND_PATTERN		        1
#define SPI_REPEAT_LAST_RECEIVED_DATA	        2
#define SPI_REPEAT_LAST_TRANSMITTED_DATA        3

#endif
