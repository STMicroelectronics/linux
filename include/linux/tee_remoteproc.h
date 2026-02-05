/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright(c) 2023 STMicroelectronics - All Rights Reserved
 */

#ifndef TEE_REMOTEPROC_H
#define TEE_REMOTEPROC_H

#include <linux/remoteproc.h>
#include <linux/tee_drv.h>

/**
 * struct tee_rproc - TEE remoteproc structure
 * @node:		Reference in list
 * @rproc:		Remoteproc reference
 * @parent:		Parent device
 * @rproc_id:		Identifier of the target firmware
 * @session_id:		TEE session identifier
 * @rsc_va:		Resource table virtual address.
 */
struct tee_rproc {
	struct list_head node;
	struct rproc *rproc;
	struct device *parent;
	u32 rproc_id;
	u32 session_id;
	void __iomem *rsc_va;
};

#if IS_ENABLED(CONFIG_TEE_REMOTEPROC)

struct tee_rproc *tee_rproc_register(struct device *dev, unsigned int rproc_id);
int tee_rproc_unregister(struct tee_rproc *trproc);

int tee_rproc_load_fw(struct tee_rproc *trproc, const struct firmware *fw);
int rproc_tee_get_rsc_table(struct tee_rproc *trproc);
struct resource_table *tee_rproc_get_loaded_rsc_table(struct tee_rproc *trproc);
int tee_rproc_start(struct tee_rproc *trproc);
int tee_rproc_stop(struct tee_rproc *trproc);

#else

static inline struct tee_rproc *tee_rproc_register(struct device *dev, unsigned int rproc_id)
{
	return ERR_PTR(-ENODEV);
}

static inline int tee_rproc_unregister(struct tee_rproc *trproc)
{
	/* This shouldn't be possible */
	WARN_ON(1);

	return 0;
}

static inline int tee_rproc_load_fw(struct tee_rproc *trproc,
				    const struct firmware *fw)
{
	/* This shouldn't be possible */
	WARN_ON(1);

	return 0;
}

static inline int tee_rproc_start(struct tee_rproc *trproc)
{
	/* This shouldn't be possible */
	WARN_ON(1);

	return 0;
}

static inline int tee_rproc_stop(struct tee_rproc *trproc)
{
	/* This shouldn't be possible */
	WARN_ON(1);

	return 0;
}

static inline int rproc_tee_get_rsc_table(struct tee_rproc *trproc)
{
	/* This shouldn't be possible */
	WARN_ON(1);

	return 0;
}

static inline struct resource_table *
	tee_rproc_get_loaded_rsc_table(struct tee_rproc *trproc)
{
	/* This shouldn't be possible */
	WARN_ON(1);

	return NULL;
}

#endif /* CONFIG_TEE_REMOTEPROC */
#endif /* TEE_REMOTEPROC_H */
