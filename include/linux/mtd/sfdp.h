/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef __MTD_SFDP_H__
#define __MTD_SFDP_H__
#include <linux/mtd/gen_probe.h>

struct mtd_info *hyperbus_sfdp_probe(struct map_info *map);

#endif /* __MTD_SFDP_H__ */
