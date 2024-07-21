// SPDX-License-Identifier: GPL-2.0

#include <linux/bug.h>

#include "hantro_boolenc.h"

void hantro_boolenc_init(struct hantro_boolenc *e, uint8_t *output)
{
	e->output = output;
	e->range = 255;
	e->bottom = 0;
	e->bit_count = 24;
	e->bytes_written = 0;
}

/*
 * Add one to a number stored in bytes preceding *q.
 * There's exactly bytes_written such bytes.
 *
 * The spec guarantees that the backward search won't go beyond
 * the start of the buffer, so if we detect such condition we can
 * BUG_ON() - this should never happen anyway.
 */
static void add_one_to_output(uint8_t *q, int bytes_written)
{
	/*
	 * originally:
	 * while (*--q == 0xff)
	 *       *q = 0;
	 */
	for (;;) {
		BUG_ON(bytes_written-- <= 0); /* check if we're allowed to go back one byte */

		if (*--q == 0xff)	      /* adding a 1 overflows *--q? */
			*q = 0;		      /* yes, so zero *q */
		else
			break;		      /* no, we're good to add 1 */
	}

	++*q;				      /* add 1 */
}

void hantro_boolenc_write_bool(struct hantro_boolenc *e, uint32_t prob, bool bool_value)
{
	uint32_t split = 1 + (((e->range - 1) * prob) >> 8);

	if (bool_value) {
		e->bottom += split;
		e->range -= split;
	} else {
		e->range = split;
	}

	while (e->range < 128) {
		e->range <<= 1;

		if (e->bottom & (1 << 31))
			add_one_to_output(e->output, e->bytes_written);

		e->bottom <<= 1;

		if (!--e->bit_count) {
			*e->output++ = (uint8_t)(e->bottom >> 24);
			++e->bytes_written;
			e->bottom &= (1 << 24) - 1;
			e->bit_count = 8;
		}
	}
}
