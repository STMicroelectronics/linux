// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) STMicroelectronics 2023 - All Rights Reserved
 * Author: Arnaud Pouliquen <arnaud.pouliquen@st.com>
 */

#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/remoteproc.h>
#include <linux/slab.h>
#include <linux/tee_drv.h>
#include <linux/tee_remoteproc.h>

#include "remoteproc_internal.h"

#define MAX_TEE_PARAM_ARRY_MEMBER	4

/*
 * Authentication of the firmware and load in the remote processor memory
 *
 * [in]  params[0].value.a:	unique 32bit identifier of the remote processor
 * [in]	 params[1].memref:	buffer containing the image of the buffer
 */
#define TA_RPROC_FW_CMD_LOAD_FW		1

/*
 * Start the remote processor
 *
 * [in]  params[0].value.a:	unique 32bit identifier of the remote processor
 */
#define TA_RPROC_FW_CMD_START_FW	2

/*
 * Stop the remote processor
 *
 * [in]  params[0].value.a:	unique 32bit identifier of the remote processor
 */
#define TA_RPROC_FW_CMD_STOP_FW		3

/*
 * Return the address of the resource table, or 0 if not found
 * No check is done to verify that the address returned is accessible by
 * the non secure context. If the resource table is loaded in a protected
 * memory the access by the non secure context will lead to a data abort.
 *
 * [in]  params[0].value.a:	unique 32bit identifier of the remote processor
 * [out]  params[1].value.a:	32bit LSB resource table memory address
 * [out]  params[1].value.b:	32bit MSB resource table memory address
 * [out]  params[2].value.a:	32bit LSB resource table memory size
 * [out]  params[2].value.b:	32bit MSB resource table memory size
 */
#define TA_RPROC_FW_CMD_GET_RSC_TABLE	4

/*
 * Return the address of the core dump
 *
 * [in]  params[0].value.a:	unique 32bit identifier of the remote processor
 * [out] params[1].memref:	address of the core dump image if exist,
 *				else return Null
 */
#define TA_RPROC_FW_CMD_GET_COREDUMP	5

struct tee_rproc_mem {
	char name[20];
	void __iomem *cpu_addr;
	phys_addr_t bus_addr;
	u32 dev_addr;
	size_t size;
};

struct tee_rproc_context {
	struct list_head sessions;
	struct tee_context *tee_ctx;
	struct device *dev;
};

static struct tee_rproc_context *tee_rproc_ctx;

static void prepare_args(struct tee_rproc *trproc, int cmd, struct tee_ioctl_invoke_arg *arg,
			 struct tee_param *param, unsigned int num_params)
{
	memset(arg, 0, sizeof(*arg));
	memset(param, 0, MAX_TEE_PARAM_ARRY_MEMBER * sizeof(*param));

	arg->func = cmd;
	arg->session = trproc->session_id;
	arg->num_params = num_params + 1;

	param[0] = (struct tee_param) {
		.attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT,
		.u.value.a = trproc->rproc_id,
	};
}

int tee_rproc_load_fw(struct tee_rproc *trproc, const struct firmware *fw)
{
	struct tee_ioctl_invoke_arg arg;
	struct tee_param param[MAX_TEE_PARAM_ARRY_MEMBER];
	struct tee_shm *fw_shm;
	int ret;

	fw_shm = tee_shm_register_kernel_buf(tee_rproc_ctx->tee_ctx, (void *)fw->data, fw->size);
	if (IS_ERR(fw_shm))
		return PTR_ERR(fw_shm);

	prepare_args(trproc, TA_RPROC_FW_CMD_LOAD_FW, &arg, param, 1);

	/* Provide the address of the firmware image */
	param[1] = (struct tee_param) {
		.attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT,
		.u.memref = {
			.shm = fw_shm,
			.size = fw->size,
			.shm_offs = 0,
		},
	};

	ret = tee_client_invoke_func(tee_rproc_ctx->tee_ctx, &arg, param);
	if (ret < 0 || arg.ret != 0) {
		dev_err(tee_rproc_ctx->dev,
			"TA_RPROC_FW_CMD_LOAD_FW invoke failed TEE err: %x, ret:%x\n",
			arg.ret, ret);
		if (!ret)
			ret = -EIO;
	}

	tee_shm_free(fw_shm);

	return ret;
}
EXPORT_SYMBOL_GPL(tee_rproc_load_fw);

int rproc_tee_get_rsc_table(struct tee_rproc *trproc)
{
	struct tee_ioctl_invoke_arg arg;
	struct tee_param param[MAX_TEE_PARAM_ARRY_MEMBER];
	struct rproc *rproc = trproc->rproc;
	size_t rsc_size;
	int ret;

	prepare_args(trproc, TA_RPROC_FW_CMD_GET_RSC_TABLE, &arg, param, 2);

	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
	param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;

	ret = tee_client_invoke_func(tee_rproc_ctx->tee_ctx, &arg, param);
	if (ret < 0 || arg.ret != 0) {
		dev_err(tee_rproc_ctx->dev,
			"TA_RPROC_FW_CMD_GET_RSC_TABLE invoke failed TEE err: %x, ret:%x\n",
			arg.ret, ret);
		return -EIO;
	}

	rsc_size = param[2].u.value.a;

	/* If the size is null no resource table defined in the image */
	if (!rsc_size)
		return 0;

	/* Store the resource table address that would be updated by the remote core . */
	trproc->rsc_va = ioremap_wc(param[1].u.value.a, rsc_size);
	if (IS_ERR_OR_NULL(trproc->rsc_va)) {
		dev_err(tee_rproc_ctx->dev, "Unable to map memory region: %lld+%zx\n",
			param[1].u.value.a, rsc_size);
		trproc->rsc_va = NULL;
		return -ENOMEM;
	}

	/*
	 * A cached table is requested as the physical address is not mapped yet
	 * but remoteproc needs to parse the table for resources.
	 */
	rproc->cached_table = kmemdup((__force void *)trproc->rsc_va, rsc_size, GFP_KERNEL);
	if (!rproc->cached_table)
		return -ENOMEM;

	rproc->table_ptr = rproc->cached_table;
	rproc->table_sz = rsc_size;

	return 0;
}
EXPORT_SYMBOL_GPL(rproc_tee_get_rsc_table);

struct resource_table *tee_rproc_get_loaded_rsc_table(struct tee_rproc *trproc)
{
	return (__force struct resource_table *)trproc->rsc_va;
}
EXPORT_SYMBOL_GPL(tee_rproc_get_loaded_rsc_table);

int tee_rproc_start(struct tee_rproc *trproc)
{
	struct tee_ioctl_invoke_arg arg;
	struct tee_param param[MAX_TEE_PARAM_ARRY_MEMBER];
	int ret;

	prepare_args(trproc, TA_RPROC_FW_CMD_START_FW, &arg, param, 0);

	ret = tee_client_invoke_func(tee_rproc_ctx->tee_ctx, &arg, param);
	if (ret < 0 || arg.ret != 0) {
		dev_err(tee_rproc_ctx->dev,
			"TA_RPROC_FW_CMD_START_FW invoke failed TEE err: %x, ret:%x\n",
			arg.ret, ret);
		if (!ret)
			ret = -EIO;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(tee_rproc_start);

int tee_rproc_stop(struct tee_rproc *trproc)
{
	struct tee_ioctl_invoke_arg arg;
	struct tee_param param[MAX_TEE_PARAM_ARRY_MEMBER];
	int ret;

	prepare_args(trproc, TA_RPROC_FW_CMD_STOP_FW, &arg, param, 0);

	ret = tee_client_invoke_func(tee_rproc_ctx->tee_ctx, &arg, param);
	if (ret < 0 || arg.ret != 0) {
		dev_err(tee_rproc_ctx->dev,
			"TA_RPROC_FW_CMD_STOP_FW invoke failed TEE err: %x, ret:%x\n",
			arg.ret, ret);
		if (!ret)
			ret = -EIO;
	}
	if (trproc->rsc_va)
		iounmap(trproc->rsc_va);
	trproc->rsc_va = NULL;

	return ret;
}
EXPORT_SYMBOL_GPL(tee_rproc_stop);

static const struct tee_client_device_id stm32_tee_rproc_id_table[] = {
	{UUID_INIT(0x80a4c275, 0x0a47, 0x4905,
		   0x82, 0x85, 0x14, 0x86, 0xa9, 0x77, 0x1a, 0x08)},
	{}
};

struct tee_rproc *tee_rproc_register(struct device *dev, unsigned int rproc_id)
{
	struct tee_client_device *rproc_tee_device;
	struct tee_ioctl_open_session_arg sess_arg;
	struct tee_param param[MAX_TEE_PARAM_ARRY_MEMBER];
	struct tee_rproc *trproc;
	int ret;

	/*
	 * The device is not probed by the TEE bus. We ignore the reason (bus could be not yet
	 * probed or service not available in the secure firmware)
	 * Assumption here is that the TEE bus is not probed.
	 */
	if (!tee_rproc_ctx)
		return ERR_PTR(-EPROBE_DEFER);

	trproc =  devm_kzalloc(dev, sizeof(*trproc), GFP_KERNEL);
	if (!trproc)
		return ERR_PTR(-ENOMEM);

	rproc_tee_device = to_tee_client_device(tee_rproc_ctx->dev);
	memset(&sess_arg, 0, sizeof(sess_arg));

	/* Open session with rproc_tee load the OP-TEE Trusted Application */
	memcpy(sess_arg.uuid, rproc_tee_device->id.uuid.b, TEE_IOCTL_UUID_LEN);

	sess_arg.clnt_login = TEE_IOCTL_LOGIN_REE_KERNEL;
	sess_arg.num_params = 1;

	param[0] = (struct tee_param) {
		.attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT,
		.u.value.a = rproc_id,
	};

	ret = tee_client_open_session(tee_rproc_ctx->tee_ctx, &sess_arg, param);
	if (ret < 0 || sess_arg.ret != 0) {
		dev_err(dev, "tee_client_open_session failed, err: %x\n", sess_arg.ret);
		return ERR_PTR(-EINVAL);
	}

	trproc->parent =  dev;
	trproc->rproc_id = rproc_id;
	trproc->session_id = sess_arg.session;

	list_add_tail(&trproc->node, &tee_rproc_ctx->sessions);

	return trproc;
}
EXPORT_SYMBOL_GPL(tee_rproc_register);

int tee_rproc_unregister(struct tee_rproc *trproc)
{
	int ret;

	if (!tee_rproc_ctx)
		return -ENODEV;

	ret = tee_client_close_session(tee_rproc_ctx->tee_ctx, trproc->session_id);
	if (ret < 0)
		dev_err(trproc->parent,	"tee_client_close_session failed, err: %x\n", ret);

	list_del(&trproc->node);

	return ret;
}
EXPORT_SYMBOL_GPL(tee_rproc_unregister);

static int tee_ctx_match(struct tee_ioctl_version_data *ver, const void *data)
{
	/* Today we support only the OP-TEE, could be extend to other tees */
	return (ver->impl_id == TEE_IMPL_ID_OPTEE);
}

static int tee_rproc_probe(struct device *dev)
{
	struct tee_context *tee_ctx;
	int ret;

	/* Only one RPROC OP-TEE device allowed */
	if (tee_rproc_ctx) {
		dev_err(dev, "An RPROC OP-TEE device was already initialized: only one allowed\n");
		return -EBUSY;
	}

	/* Open context with TEE driver */
	tee_ctx = tee_client_open_context(NULL, tee_ctx_match, NULL, NULL);
	if (IS_ERR(tee_ctx))
		return PTR_ERR(tee_ctx);

	tee_rproc_ctx = devm_kzalloc(dev, sizeof(*tee_rproc_ctx), GFP_KERNEL);
	if (!tee_rproc_ctx) {
		ret = -ENOMEM;
		goto err;
	}

	tee_rproc_ctx->dev = dev;
	tee_rproc_ctx->tee_ctx = tee_ctx;
	INIT_LIST_HEAD(&tee_rproc_ctx->sessions);

	return 0;
err:
	tee_client_close_context(tee_ctx);

	return ret;
}

static int tee_rproc_remove(struct device *dev)
{
	struct tee_rproc *entry, *tmp;

	list_for_each_entry_safe(entry, tmp, &tee_rproc_ctx->sessions, node) {
		tee_client_close_session(tee_rproc_ctx->tee_ctx, entry->session_id);
		list_del(&entry->node);
		kfree(entry);
	}

	tee_client_close_context(tee_rproc_ctx->tee_ctx);

	return 0;
}

MODULE_DEVICE_TABLE(tee, stm32_tee_rproc_id_table);

static struct tee_client_driver tee_rproc_fw_driver = {
	.id_table	= stm32_tee_rproc_id_table,
	.driver		= {
		.name		= KBUILD_MODNAME,
		.bus		= &tee_bus_type,
		.probe		= tee_rproc_probe,
		.remove		= tee_rproc_remove,
	},
};

static int __init tee_rproc_fw_mod_init(void)
{
	return driver_register(&tee_rproc_fw_driver.driver);
}

static void __exit tee_rproc_fw_mod_exit(void)
{
	driver_unregister(&tee_rproc_fw_driver.driver);
}

module_init(tee_rproc_fw_mod_init);
module_exit(tee_rproc_fw_mod_exit);

MODULE_DESCRIPTION(" TEE remote processor control driver");
MODULE_LICENSE("GPL");
