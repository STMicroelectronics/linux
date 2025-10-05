#ifndef __U_SERIAL_H
#define __U_SERIAL_H

#include <linux/types.h>

struct gserial;

/* Allow function drivers (e.g. f_acm) to report host line state */
void gserial_set_dtr_rts(struct gserial *gser, bool dtr, bool rts);

#endif /* __U_SERIAL_H */