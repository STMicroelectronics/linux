/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015, Linaro Limited
 */

#ifndef OPTEE_PRIVATE_H
#define OPTEE_PRIVATE_H

#include <linux/arm-smccc.h>
#include <linux/semaphore.h>
#include <linux/tee_drv.h>
#include <linux/types.h>
#include "optee_msg.h"

#define OPTEE_MAX_ARG_SIZE	1024

/* Some Global Platform error codes used in this driver */
#define TEEC_SUCCESS			0x00000000
#define TEEC_ERROR_CANCEL		0xFFFF0002
#define TEEC_ERROR_BAD_PARAMETERS	0xFFFF0006
#define TEEC_ERROR_NOT_SUPPORTED	0xFFFF000A
#define TEEC_ERROR_COMMUNICATION	0xFFFF000E
#define TEEC_ERROR_OUT_OF_MEMORY	0xFFFF000C
#define TEEC_ERROR_SHORT_BUFFER		0xFFFF0010
#define TEEC_ERROR_TARGET_DEAD		0xFFFF3024

#define TEEC_ORIGIN_COMMS		0x00000002

/*
 * This value should be larger than the number threads in secure world to
 * meet the need from secure world. The number of threads in secure world
 * are usually not even close to 255 so we should be safe for now.
 */
#define OPTEE_DEFAULT_MAX_NOTIF_VALUE	255

#define OPTEE_MAX_IT 32

typedef void (optee_invoke_fn)(unsigned long, unsigned long, unsigned long,
				unsigned long, unsigned long, unsigned long,
				unsigned long, unsigned long,
				struct arm_smccc_res *);

struct optee_call_queue {
	/* Serializes access to this struct */
	struct mutex mutex;
	struct list_head waiters;
};

struct optee_notif {
	u_int max_key;
	struct tee_context *ctx;
	/* Serializes access to the elements below in this struct */
	spinlock_t lock;
	struct list_head db;
	u_long *bitmap;
};

/**
 * struct optee_supp - supplicant synchronization struct
 * @ctx			the context of current connected supplicant.
 *			if !NULL the supplicant device is available for use,
 *			else busy
 * @mutex:		held while accessing content of this struct
 * @req_id:		current request id if supplicant is doing synchronous
 *			communication, else -1
 * @reqs:		queued request not yet retrieved by supplicant
 * @idr:		IDR holding all requests currently being processed
 *			by supplicant
 * @reqs_c:		completion used by supplicant when waiting for a
 *			request to be queued.
 */
struct optee_supp {
	/* Serializes access to this struct */
	struct mutex mutex;
	struct tee_context *ctx;

	int req_id;
	struct list_head reqs;
	struct idr idr;
	struct completion reqs_c;
};

/**
 * struct optee - main service struct
 * @supp_teedev:	supplicant device
 * @teedev:		client device
 * @ctx:		driver internal TEE context
 * @invoke_fn:		function to issue smc or hvc
 * @call_queue:		queue of threads waiting to call @invoke_fn
 * @notif:		notification synchronization struct
 * @supp:		supplicant synchronization struct for RPC to supplicant
 * @pool:		shared memory pool
 * @memremaped_shm	virtual address of memory in shared memory pool
 * @sec_caps:		secure world capabilities defined by
 *			OPTEE_SMC_SEC_CAP_* in optee_smc.h
 * @scan_bus_done	flag if device registation was already done.
 * @scan_bus_wq		workqueue to scan optee bus and register optee drivers
 * @scan_bus_work	workq to scan optee bus and register optee drivers
 * @optee_pcpu		per_cpu optee instance
 * @notif_pcpu_wq	workqueue for per cpu aynchronous notification
 * @notif_pcpu_work	work for per cpu asynchronous notification
 */
struct optee {
	struct tee_device *supp_teedev;
	struct tee_device *teedev;
	optee_invoke_fn *invoke_fn;
	struct tee_context *ctx;
	struct optee_call_queue call_queue;
	struct optee_notif notif;
	struct optee_supp supp;
	struct tee_shm_pool *pool;
	void *memremaped_shm;
	u32 sec_caps;
	bool   scan_bus_done;
	struct workqueue_struct *scan_bus_wq;
	struct work_struct scan_bus_work;
	unsigned int notif_irq;
	unsigned int notif_pcpu_irq;
	struct optee_pcpu __percpu *optee_pcpu;
	struct workqueue_struct *notif_pcpu_wq;
	struct work_struct notif_pcpu_work;
	struct irq_domain *domain;
};

struct optee_call_waiter {
	struct list_head list_node;
	struct completion c;
};

struct optee_pcpu {
	struct optee *optee;
};

/**
 * struct optee_call_ctx - holds context that is preserved during one STD call
 * @pages_list:		list of pages allocated for RPC requests
 * @num_entries:	number of pages in 'pages_list'
 * @ctx:		TEE context whence the OCALL originated, if any
 * @msg_shm:		shared memory object used for calling into OP-TEE
 * @msg_arg:		arguments used for calling into OP-TEE, namely the data
 *			behind 'msg_shm'
 * @msg_parg:		physical pointer underlying 'msg_shm'
 * @rpc_must_release:	indicates that OCALL parameters have had their refcount
 *			increased and must be decreased on cancellation
 * @rpc_shm:		shared memory object used for responding to RPCs
 * @rpc_arg:		arguments used for responding to RPCs, namely the data
 *			behind 'rpc_shm'
 * @thread_id:		secure thread Id whence the OCALL originated and which
 *			must be resumed when replying to the OCALL
 * @waiter:		object used to wait until a secure thread becomes
 *			available is the previous call into OP-TEE failed
 *			because all secure threads are in use
 * @ocall_pages_list:   list of pages allocated for OCALL requests
 * @ocall_num_entries:  number of pages in 'ocall_pages_list'
 */
struct optee_call_ctx {
	/* Information about pages list used in last allocation */
	void *pages_list;
	size_t num_entries;

	/* OCALL support */
	struct tee_context *ctx;

	struct tee_shm *msg_shm;
	struct optee_msg_arg *msg_arg;
	phys_addr_t msg_parg;

	bool rpc_must_release;
	struct tee_shm *rpc_shm;
	struct optee_msg_arg *rpc_arg;

	u32 thread_id;
	struct optee_call_waiter waiter;

	void *ocall_pages_list;
	size_t ocall_num_entries;
};

struct optee_session {
	/* Serializes access to this struct */
	struct semaphore sem;
	struct list_head list_node;
	u32 session_id;
	struct optee_call_ctx call_ctx;
};

struct optee_context_data {
	/* Serializes access to this struct */
	struct mutex mutex;
	struct list_head sess_list;
	struct idr tmp_sess_list;
};

struct optee_rpc_param {
	u32	a0;
	u32	a1;
	u32	a2;
	u32	a3;
	u32	a4;
	u32	a5;
	u32	a6;
	u32	a7;
};

/*
 * RPC support
 */

void optee_handle_rpc(struct tee_context *ctx, struct optee_rpc_param *param,
		      struct optee_call_ctx *call_ctx);
bool optee_rpc_is_ocall(struct optee_rpc_param *param,
			struct optee_call_ctx *call_ctx);
void optee_rpc_finalize_call(struct optee_call_ctx *call_ctx);

/*
 * Wait queue
 */

int optee_notif_init(struct optee *optee, u_int max_key);
void optee_notif_uninit(struct optee *optee);
int optee_notif_wait(struct optee *optee, u_int key);
int optee_notif_send(struct optee *optee, u_int key);

/*
 * Call queue
 */

void optee_cq_wait_init(struct optee_call_queue *cq,
			struct optee_call_waiter *w);
void optee_cq_wait_for_completion(struct optee_call_queue *cq,
				  struct optee_call_waiter *w);
void optee_cq_complete_one(struct optee_call_queue *cq);
void optee_cq_wait_final(struct optee_call_queue *cq,
			 struct optee_call_waiter *w);

/*
 * Supplicant
 */

u32 optee_supp_thrd_req(struct tee_context *ctx, u32 func, size_t num_params,
			struct tee_param *param);

int optee_supp_read(struct tee_context *ctx, void __user *buf, size_t len);
int optee_supp_write(struct tee_context *ctx, void __user *buf, size_t len);
void optee_supp_init(struct optee_supp *supp);
void optee_supp_uninit(struct optee_supp *supp);
void optee_supp_release(struct optee_supp *supp);

int optee_supp_recv(struct tee_context *ctx, u32 *func, u32 *num_params,
		    struct tee_param *param);
int optee_supp_send(struct tee_context *ctx, u32 ret, u32 num_params,
		    struct tee_param *param);

/*
 * Calls into OP-TEE
 */

u32 optee_do_call_with_arg(struct tee_context *ctx, phys_addr_t parg);

/*
 * Sessions
 */

int optee_open_session(struct tee_context *ctx,
		       struct tee_ioctl_open_session_arg *arg,
		       struct tee_param *normal_param, u32 num_normal_params,
		       struct tee_param *ocall_param);
int optee_close_session(struct tee_context *ctx, u32 session);

/*
 * Function invocations
 */

int optee_invoke_func(struct tee_context *ctx, struct tee_ioctl_invoke_arg *arg,
		      struct tee_param *normal_param, u32 num_normal_params,
		      struct tee_param *ocall_param);

/*
 * Cancellations
 */

int optee_cancel_req(struct tee_context *ctx, u32 cancel_id, u32 session);

/*
 * Shared memory
 */

void optee_enable_shm_cache(struct optee *optee);
void optee_disable_shm_cache(struct optee *optee);
void optee_disable_unmapped_shm_cache(struct optee *optee);

int optee_shm_register(struct tee_context *ctx, struct tee_shm *shm,
		       struct page **pages, size_t num_pages,
		       unsigned long start);
int optee_shm_unregister(struct tee_context *ctx, struct tee_shm *shm);

int optee_shm_register_supp(struct tee_context *ctx, struct tee_shm *shm,
			    struct page **pages, size_t num_pages,
			    unsigned long start);
int optee_shm_unregister_supp(struct tee_context *ctx, struct tee_shm *shm);

/*
 * Paremeters
 */

int optee_from_msg_param(struct tee_param *params, size_t num_params,
			 const struct optee_msg_param *msg_params);
int optee_to_msg_param(struct optee_msg_param *msg_params, size_t num_params,
		       const struct tee_param *params);
struct tee_shm *optee_get_msg_arg(struct tee_context *ctx, size_t num_params,
				  struct optee_msg_arg **msg_arg,
				  phys_addr_t *msg_parg);

/*
 * RPC memory
 */

u64 *optee_allocate_pages_list(size_t num_entries);
void optee_free_pages_list(void *array, size_t num_entries);
void optee_fill_pages_list(u64 *dst, struct page **pages, int num_pages,
			   size_t page_offset);

/*
 * Devices
 */

#define PTA_CMD_GET_DEVICES		0x0
#define PTA_CMD_GET_DEVICES_SUPP	0x1
int optee_enumerate_devices(u32 func);
void optee_unregister_devices(void);

/*
 * OCALLs
 */

void optee_cancel_open_session_ocall(struct optee_session *sess);
void optee_cancel_invoke_function_ocall(struct optee_call_ctx *call_ctx);

/*
 * Small helpers
 */

static inline void *reg_pair_to_ptr(u32 reg0, u32 reg1)
{
	return (void *)(unsigned long)(((u64)reg0 << 32) | reg1);
}

static inline void reg_pair_from_64(u32 *reg0, u32 *reg1, u64 val)
{
	*reg0 = val >> 32;
	*reg1 = val;
}

#endif /*OPTEE_PRIVATE_H*/
