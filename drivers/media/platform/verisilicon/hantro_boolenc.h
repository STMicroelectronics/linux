/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/types.h>

struct hantro_boolenc {
	uint8_t *output;	/* next byte pointer */
	uint32_t range;
	uint32_t bottom;	/* 4 LSB of leftmost end of the range, MSB alread in output  */
	int bit_count;		/* how many shifts until next encoded byte available */
	int bytes_written;
};

void hantro_boolenc_init(struct hantro_boolenc *e, uint8_t *output);

void hantro_boolenc_write_bool(struct hantro_boolenc *e, uint32_t prob, bool bool_value);

static inline void hantro_boolenc_write_lit(struct hantro_boolenc *e, uint32_t val, uint8_t n)
{
	while (n--)
		hantro_boolenc_write_bool(e, 128, (val >> n) & 0x1);
}
