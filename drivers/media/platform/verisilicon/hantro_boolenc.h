/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/types.h>

struct hantro_boolenc {
	u8 *output;	/* next byte pointer */
	u32 range;
	u32 bottom;	/* 4 LSB of leftmost end of the range, MSB alread in output  */
	int bit_count;		/* how many shifts until next encoded byte available */
	int bytes_written;
};

void hantro_boolenc_init(struct hantro_boolenc *e, u8 *output);

void hantro_boolenc_write_bool(struct hantro_boolenc *e, u32 prob, bool bool_value);

static inline void hantro_boolenc_write_lit(struct hantro_boolenc *e, u32 val, u8 n)
{
	while (n--)
		hantro_boolenc_write_bool(e, 128, (val >> n) & 0x1);
}
