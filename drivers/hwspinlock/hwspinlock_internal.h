/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Hardware spinlocks internal header
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com
 *
 * Contact: Ohad Ben-Cohen <ohad@wizery.com>
 */

#ifndef __HWSPINLOCK_HWSPINLOCK_H
#define __HWSPINLOCK_HWSPINLOCK_H

#include <linux/spinlock.h>
#include <linux/device.h>

struct hwspinlock_device;

/**
 * struct hwspinlock - this struct represents a single hwspinlock instance
 * @bank: the hwspinlock_device structure which owns this lock
 * @lock: initialized and used by hwspinlock core
 * @priv: private data, owned by the underlying platform-specific hwspinlock drv
 */
struct hwspinlock {
	struct hwspinlock_device *bank;
	spinlock_t lock;
	void *priv;
};

/**
 * struct hwspinlock_device - a device which usually spans numerous hwspinlocks
 * @dev: underlying device, will be used to invoke runtime PM api
 * @ops: platform-specific hwspinlock handlers
 * @base_id: id index of the first lock in this device
 * @num_locks: number of locks in this device
 * @lock: dynamically allocated array of 'struct hwspinlock'
 */
struct hwspinlock_device {
	struct device *dev;
	const struct hwspinlock_ops *ops;
	int base_id;
	int num_locks;
	struct hwspinlock lock[];
};

#endif /* __HWSPINLOCK_HWSPINLOCK_H */
