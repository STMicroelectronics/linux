/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 STMicroelectronics
 */

#ifndef __ST_TCPP_H__
#define __ST_TCPP_H__

typedef void (*tcpp_vbus_cb_t)(void *data, bool vbus_present);

int tcpp_init(struct device *tcpp);
void tcpp_deinit(struct device *tcpp);
int tcpp_get_vbus(struct device *tcpp);
int tcpp_set_vbus(struct device *tcpp, bool on, bool charge);
int tcpp_set_vconn(struct device *dev, bool on, enum typec_cc_polarity polarity);
int tcpp_attach(struct device *tcpp);
int tcpp_detach(struct device *tcpp);
int tcpp_register_vbus_notifier(struct device *dev, tcpp_vbus_cb_t cb, void *data);
void tcpp_unregister_vbus_notifier(struct device *dev);

#endif
