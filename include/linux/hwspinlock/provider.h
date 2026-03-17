/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Hardware spinlock public header for providers
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com
 * Copyright (C) 2026 Sang Engineering
 * Copyright (C) 2026 Renesas Solutions Corp.
 */

#ifndef __LINUX_HWSPINLOCK_PROVIDER_H
#define __LINUX_HWSPINLOCK_PROVIDER_H

#include <linux/err.h>

struct device;
struct hwspinlock;
struct hwspinlock_device;
struct of_phandle_args;

/**
 * struct hwspinlock_ops - platform-specific hwspinlock handlers
 *
 * @trylock:	make a single attempt to take the lock. returns 0 on
 *		failure and true on success. may _not_ sleep.
 * @unlock:	release the lock. always succeed. may _not_ sleep.
 * @bust:	optional, platform-specific bust handler, called by hwspinlock
 *		core to bust a specific lock.
 * @relax:	optional, platform-specific relax handler, called by hwspinlock
 *		core while spinning on a lock, between two successive
 *		invocations of @trylock. may _not_ sleep.
 * @xlate:	optional, platform-specific translate hwlock_spec to return a
 *		lock id.
 * @init_priv:	optional, callback used when registering the hwspinlock device.
 *		Its return value will be used to fill the per-lock 'priv' data.
 */
struct hwspinlock_ops {
	int (*trylock)(struct hwspinlock *lock);
	void (*unlock)(struct hwspinlock *lock);
	int (*bust)(struct hwspinlock *lock, unsigned int id);
	void (*relax)(struct hwspinlock *lock);
	int (*xlate)(const struct of_phandle_args *hwlock_spec);
	void *(*init_priv)(int local_id, void *init_data);
};

void *hwspin_lock_get_priv(struct hwspinlock *hwlock);
struct device *hwspin_lock_get_dev(struct hwspinlock *hwlock);
int hwlock_to_id(struct hwspinlock *hwlock);
struct hwspinlock_device *hwspin_lock_register(struct device *dev, const struct hwspinlock_ops *ops,
					       int base_id, int num_locks, void *init_data);
int hwspin_lock_unregister(struct hwspinlock_device *bank);

struct hwspinlock_device *devm_hwspin_lock_register(struct device *dev, const struct hwspinlock_ops *ops,
						    int base_id, int num_locks, void *init_data);
int devm_hwspin_lock_unregister(struct device *dev,
				struct hwspinlock_device *bank);

static inline int devm_hwspin_lock_register_errno(struct device *dev,
						  const struct hwspinlock_ops *ops,
						  int base_id, int num_locks, void *init_data)
{
	return PTR_ERR_OR_ZERO(devm_hwspin_lock_register(dev, ops, base_id, num_locks, init_data));
}

#endif /* __LINUX_HWSPINLOCK_PROVIDER_H */
