// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2021 Linaro Ltd.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/tee_drv.h>
#include <linux/uuid.h>
#include <uapi/linux/tee.h>

#include "../common.h"

enum scmi_optee_pta_cmd {
	/*
	 * PTA_SCMI_CMD_CAPABILITIES - Get channel capabilities
	 *
	 * [out]    value[0].a: Capability bit mask (enum pta_scmi_caps)
	 * [out]    value[0].b: Extended capabilities or 0
	 */
	PTA_SCMI_CMD_CAPABILITIES = 0,

	/*
	 * PTA_SCMI_CMD_PROCESS_SMT_CHANNEL - Process SCMI message in SMT buffer
	 *
	 * [in]     value[0].a: Channel handle
	 *
	 * Shared memory used for SCMI message/response exhange is expected
	 * already identified and bound to channel handle in both SCMI agent
	 * and SCMI server (OP-TEE) parts.
	 * The memory uses SMT header to carry SCMI meta-data (protocol ID and
	 * protocol message ID).
	 */
	PTA_SCMI_CMD_PROCESS_SMT_CHANNEL = 1,

	/*
	 * PTA_SCMI_CMD_PROCESS_SMT_CHANNEL_MESSAGE - Process SMT/SCMI message
	 *
	 * [in]     value[0].a: Channel handle
	 * [in/out] memref[1]: Message/response buffer (SMT and SCMI payload)
	 *
	 * Shared memory used for SCMI message/response is a SMT buffer
	 * referenced by param[1]. It shall be 128 bytes large to fit response
	 * payload whatever message playload size.
	 * The memory uses SMT header to carry SCMI meta-data (protocol ID and
	 * protocol message ID).
	 */
	PTA_SCMI_CMD_PROCESS_SMT_CHANNEL_MESSAGE = 2,

	/*
	 * PTA_SCMI_CMD_GET_CHANNEL - Get channel handle
	 *
	 * SCMI shm information are 0 if agent expects to use OP-TEE regular SHM
	 *
	 * [in]     value[0].a: Channel identifier
	 * [out]    value[0].a: Returned channel handle
	 * [in]     value[0].b: Requested capabilities mask (enum pta_scmi_caps)
	 */
	PTA_SCMI_CMD_GET_CHANNEL = 3,

	/*
	 * PTA_SCMI_CMD_PROCESS_MSG_CHANNEL - Process SCMI message in a MSG
	 * buffer pointed by memref parameters
	 *
	 * [in]     value[0].a: Channel handle
	 * [in]     memref[1]: Message buffer (MSG and SCMI payload)
	 * [out]    memref[2]: Response buffer (MSG and SCMI payload)
	 *
	 * Shared memories used for SCMI message/response are MSG buffers
	 * referenced by param[1] and param[2]. MSG transport protocol
	 * uses a 32bit header to carry SCMI meta-data (protocol ID and
	 * protocol message ID) followed by the effective SCMI message
	 * payload.
	 */
	PTA_SCMI_CMD_PROCESS_MSG_CHANNEL = 4,

	/*
	 * PTA_SCMI_CMD_OCALL2_SMT_THREAD - Allocate a thread context using
	 * OCALL2 for processing of SMT messages.
	 *
	 * [in]     value[0].a: channel handle
	 *
	 * Use Ocall support to create a provisioned OP-TEE thread context for
	 * the channel. Successful creation of the thread makes this command to
	 * return with Ocall command PTA_SCMI_OCALL_CMD_THREAD_READY.
	 */
	PTA_SCMI_CMD_OCALL2_SMT_THREAD = 2048,

	/*
	 * PTA_SCMI_CMD_OCALL2_MSG_THREAD - Allocate an thread context using
	 * OCALL2 for processing of MSG messages.
	 *
	 * [in]     value[0].a: channel handle
	 * [in]     memref[1]: Message buffer (MSG and SCMI payload)
	 * [out]    memref[2]: Response buffer (MSG and SCMI payload)

	 * Use Ocall support to create a provisioned OP-TEE thread context for
	 * the channel. Successful creation of the thread makes this command to
	 * return with Ocall command PTA_SCMI_OCALL_CMD_THREAD_READY.
	 */
	PTA_SCMI_CMD_OCALL2_MSG_THREAD = 2049,
};

/* IDs defined in GPD TEE specification OP-TEE is based on */
#define TEEC_SUCCESS			0
#define TEEC_ERROR_GENERIC		0xffff0000
#define TEEC_ERROR_NOT_SUPPORTED	0xffff000a

/*
 * OP-TEE SCMI service capabilities bit flags (32bit)
 *
 * PTA_SCMI_CAPS_SMT_HEADER
 * When set, OP-TEE supports command using SMT header protocol (SCMI shmem) in
 * shared memory buffers to carry SCMI protocol synchronisation information.
 *
 * PTA_SCMI_CAPS_MSG_HEADER
 * When set, OP-TEE supports command using MSG header protocol in an OP-TEE
 * shared memory to carry SCMI protocol synchronisation information and SCMI
 * message payload.
 */
#define PTA_SCMI_CAPS_NONE		0
#define PTA_SCMI_CAPS_SMT_HEADER	BIT(0)
#define PTA_SCMI_CAPS_MSG_HEADER	BIT(1)
/*
 * Channel can use command PTA_SCMI_CMD_OCALL_THREAD to provision a
 * TEE thread for SCMI message passing.
 */
#define PTA_SCMI_CAPS_OCALL2_THREAD	BIT(31)

#define PTA_SCMI_CAPS_MASK		(PTA_SCMI_CAPS_SMT_HEADER | \
					 PTA_SCMI_CAPS_MSG_HEADER | \
					 PTA_SCMI_CAPS_OCALL2_THREAD)

/*
 * enum optee_scmi_ocall_cmd
 * enum optee_scmi_ocall_reply
 *
 * These enumerates define the IDs used by REE/TEE to communicate in the
 * established REE/TEE Ocall thread context.
 *
 * At channel setup, we start from the REE: caller requests an Ocall context.
 *
 * 0. REE opens a session toward PTA SCMI. REE invokes PTA command
 *    PTA_SCMI_CMD_GET_CHANNEL to get a channel handler.
 *
 * 1. REE invokes command PTA_SCMI_CMD_OCALL2_SMT_THREAD with an Ocall context.
 *    This is the initial invocation of the Ocall thread context. Any further
 *    error in the thread communication make the Ocall to return from REE to
 *    TEE with an error status (Ocall2 out_param1 == 0) upon which SCMI PTA
 *    will return from initial command PTA_SCMI_CMD_OCALL2_SMT_THREAD with an
 *    error result.
 *
 * 2. Upon support of Ocall the PTA creates an Ocall context and returns to
 *    REE with Ocall command PTA_SCMI_OCALL_CMD_THREAD_READY.
 *
 * 3. REE returns to the PTA, from the Ocall, with output out_param1
 *    set to PTA_SCMI_OCALL_PROCESS_SMT_MESSAGE to post an SCMI message.
 *    In such case, OP-TEE processes the message and returns to REE with
 *    Ocall command PTA_SCMI_OCALL_CMD_THREAD_READY. The SCMI response is in
 *    the shared memory buffer.
 *
 * 4. Alternatively REE can return from the Ocall with out_param1 set to
 *    PTA_SCMI_OCALL_CLOSE_THREAD. This requests OP-TEE to terminate the
 *    Ocall, release resources and return from initial command invocation at
 *    step 1. as if REE closes the SCMI communication.
 *
 * At anytime if an error is reported by Ocall command replies, SCMI PTA
 * releases the Ocall thread context and returns from initial invocation
 * at step 1. PTA_SCMI_OCALL_ERROR is used in Ocall return to force an error
 * report.
 *
 * REE channel initialization completes when returning from step 2.
 * REE agent posts an SCMI message through step 3.
 * At channel release, REE driver executes step 4.
 */

enum scmi_optee_ocall_cmd {
	/*
	 * PTA_SCMI_OCALL_CMD_THREAD_READY - SCMI PTA send this Ocall command
	 * when it is ready to process an SCMI message on return of this
	 * Ocall.
	 *
	 * Ocall2 parameters value:
	 * [in] param1: PTA_SCMI_OCALL_CMD_THREAD_READY
	 * [in] ocall_arg in_param2: unused.
	 *
	 * [out] param1: One of enum scmi_optee_ocall_reply
	 * [out] param2: unused.
	 */
	PTA_SCMI_OCALL_CMD_THREAD_READY = 0,
};

enum scmi_optee_ocall_reply {
	/* Ocall error: on return of Ocall, SCMI PTA closes the Ocall thread */
	PTA_SCMI_OCALL_ERROR = TEE_OCALL2_OUT_PARAM1_ERROR,
	/* On return of Ocall, SCMI PTA shall close the Ocall thread */
	PTA_SCMI_OCALL_CLOSE_THREAD = 1,
	/*
	 * On return of Ocall, SCMI PTA shall process channel's SCMI message and
	 * issue Ocall command PTA_SCMI_OCALL_CMD_THREAD_READY again.
	 */
	PTA_SCMI_OCALL_PROCESS_SMT_MESSAGE = 2,
};

/*
 * struct ocall_ctx - Context of the Ocall used for initial command
 * PTA_SCMI_CMD_OCALL2_*_THREAD and on return of Ocalls.
 *
 * @arg: TEE invoke command arguments
 * @param: TEE invoke command parameters
 * @ocall_arg: TEE Ocall2 arguments
 */
struct ocall_ctx {
	struct tee_ioctl_invoke_arg args;
	struct tee_param param[4];
	struct tee_ocall2_arg ocall_arg;
};

/**
 * struct scmi_optee_channel - Description of an OP-TEE SCMI channel
 *
 * @channel_id: OP-TEE channel ID used for this transport
 * @tee_session: TEE session identifier
 * @caps: OP-TEE SCMI channel capabilities
 * @rx_len: Response size
 * @mu: Mutex protection on channel access
 * @cinfo: SCMI channel information
 * @req: union for SCMI interface
 * @req.shmem: Virtual base address of the shared memory
 * @req.msg: Shared memory protocol handle for SCMI request and
 *   synchronous response
 * @io_ops: Transport specific I/O operations
 * @tee_shm: TEE shared memory handle @req or NULL if using IOMEM shmem
 * @ocall_ctx: OP-TEE Ocall context the SCMI channel is executing in
 * @agent: Handle of the SCMI OP-TEE agent the channel belongs to
 * @link: Reference in agent's channel list
 */
struct scmi_optee_channel {
	u32 channel_id;
	u32 tee_session;
	u32 caps;
	u32 rx_len;
	struct mutex mu;
	struct scmi_chan_info *cinfo;
	union {
		struct scmi_shared_mem __iomem *shmem;
		struct scmi_msg_payld *msg;
	} req;
	struct scmi_shmem_io_ops *io_ops;
	struct tee_shm *tee_shm;
	struct ocall_ctx *ocall_ctx;
	struct scmi_optee_agent *agent;
	struct list_head link;
};

/**
 * struct scmi_optee_agent - OP-TEE transport private data
 *
 * @dev: Device used for communication with TEE
 * @tee_ctx: TEE context used for communication
 * @caps: Supported channel capabilities
 * @mu: Mutex for protection of @channel_list
 * @channel_list: List of all created channels for the agent
 */
struct scmi_optee_agent {
	struct device *dev;
	struct tee_context *tee_ctx;
	u32 caps;
	struct mutex mu;
	struct list_head channel_list;
};

static struct scmi_transport_core_operations *core;

/* There can be only 1 SCMI service in OP-TEE we connect to */
static struct scmi_optee_agent *scmi_optee_private;

/* Open a session toward SCMI OP-TEE service with REE_KERNEL identity */
static int open_session(struct scmi_optee_agent *agent, u32 *tee_session)
{
	struct device *dev = agent->dev;
	struct tee_client_device *scmi_pta = to_tee_client_device(dev);
	struct tee_ioctl_open_session_arg arg = { };
	int ret;

	memcpy(arg.uuid, scmi_pta->id.uuid.b, TEE_IOCTL_UUID_LEN);
	arg.clnt_login = TEE_IOCTL_LOGIN_REE_KERNEL;

	ret = tee_client_open_session(agent->tee_ctx, &arg, NULL);
	if (ret < 0 || arg.ret) {
		dev_err(dev, "Can't open tee session: %d / %#x\n", ret, arg.ret);
		return -EOPNOTSUPP;
	}

	*tee_session = arg.session;

	return 0;
}

static void close_session(struct scmi_optee_agent *agent, u32 tee_session)
{
	tee_client_close_session(agent->tee_ctx, tee_session);
}

static int get_capabilities(struct scmi_optee_agent *agent)
{
	struct tee_ioctl_invoke_arg arg = { };
	struct tee_param param[1] = { };
	u32 caps;
	u32 tee_session;
	int ret;

	ret = open_session(agent, &tee_session);
	if (ret)
		return ret;

	arg.func = PTA_SCMI_CMD_CAPABILITIES;
	arg.session = tee_session;
	arg.num_params = 1;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;

	ret = tee_client_invoke_func(agent->tee_ctx, &arg, param);

	close_session(agent, tee_session);

	if (ret < 0 || arg.ret) {
		dev_err(agent->dev, "Can't get capabilities: %d / %#x\n", ret, arg.ret);
		return -EOPNOTSUPP;
	}

	caps = param[0].u.value.a;

	if (!(caps & (PTA_SCMI_CAPS_SMT_HEADER | PTA_SCMI_CAPS_MSG_HEADER))) {
		dev_err(agent->dev, "OP-TEE SCMI PTA doesn't support SMT and MSG\n");
		return -EOPNOTSUPP;
	}

	agent->caps = caps;

	return 0;
}

static int get_channel(struct scmi_optee_channel *channel)
{
	struct device *dev = channel->agent->dev;
	struct tee_ioctl_invoke_arg arg = { };
	struct tee_param param[1] = { };
	unsigned int caps = 0;
	int ret;

	if (channel->tee_shm)
		caps = PTA_SCMI_CAPS_MSG_HEADER;
	else
		caps = PTA_SCMI_CAPS_SMT_HEADER;

	if (channel->agent->caps & PTA_SCMI_CAPS_OCALL2_THREAD)
		caps |= PTA_SCMI_CAPS_OCALL2_THREAD;

	arg.func = PTA_SCMI_CMD_GET_CHANNEL;
	arg.session = channel->tee_session;
	arg.num_params = 1;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	param[0].u.value.a = channel->channel_id;
	param[0].u.value.b = caps;

	ret = tee_client_invoke_func(channel->agent->tee_ctx, &arg, param);

	if (!ret && (caps & PTA_SCMI_CAPS_OCALL2_THREAD) &&
	    arg.ret == TEEC_ERROR_NOT_SUPPORTED) {
		dev_info(dev, "Ocall not supported, fallback to non-Ocall\n");

		caps &= ~PTA_SCMI_CAPS_OCALL2_THREAD;

		memset(&arg, 0, sizeof(arg));
		memset(&param, 0, sizeof(param));

		arg.func = PTA_SCMI_CMD_GET_CHANNEL;
		arg.session = channel->tee_session;
		arg.num_params = 1;

		param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
		param[0].u.value.a = channel->channel_id;
		param[0].u.value.b = caps;

		ret = tee_client_invoke_func(channel->agent->tee_ctx,
					     &arg, param);
	}

	if (ret || arg.ret) {
		dev_err(dev, "Can't get channel with caps %#x: %d / %#x\n", caps, ret, arg.ret);
		return -EOPNOTSUPP;
	}

	/* From now on use channel identifer provided by OP-TEE SCMI service */
	channel->channel_id = param[0].u.value.a;
	channel->caps = caps;

	return 0;
}

/*
 * Invoke function with Ocall context.
 * The below creates an Ocall thread context for SCMI agent to invoke by
 * returning from an Ocall instead of invoking a command. This provisions
 * a secure thread for SCMI system communication.
 */
static int invoke_optee_ocall(struct scmi_optee_channel *channel)
{
	return tee_client_invoke_func_ocall2(channel->agent->tee_ctx,
					     &channel->ocall_ctx->args,
					     channel->ocall_ctx->param,
					     &channel->ocall_ctx->ocall_arg);
}

static bool return_is_ocall(struct ocall_ctx *ocall_ctx)
{
	return tee_ocall_in_progress(&ocall_ctx->ocall_arg);
}

static int alloc_ocall_ctx(struct scmi_optee_channel *channel)
{
	if (WARN_ON(channel->ocall_ctx))
		return -EINVAL;

	channel->ocall_ctx = devm_kzalloc(channel->agent->dev,
					  sizeof(*channel->ocall_ctx),
					  GFP_KERNEL);
	if (!channel->ocall_ctx)
		return -ENOMEM;

	return 0;
}

static void free_ocall_ctx(struct scmi_optee_channel *channel)
{
	devm_kfree(channel->agent->dev, channel->ocall_ctx);
	channel->ocall_ctx = NULL;
}

static void abort_ocall(struct scmi_optee_channel *channel)
{
	struct ocall_ctx *ocall_ctx = channel->ocall_ctx;
	int ret;

	ocall_ctx->ocall_arg.out_param1 = PTA_SCMI_OCALL_ERROR;
	ocall_ctx->ocall_arg.out_param2 = 0;

	ret = invoke_optee_ocall(channel);

	WARN_ONCE(ret || return_is_ocall(ocall_ctx), "Unexpected error\n");
}

static bool ocall_thread_is_ready(struct scmi_optee_channel *channel)
{
	struct ocall_ctx *ocall_ctx = channel->ocall_ctx;
	struct device *dev = channel->agent->dev;

	if (!return_is_ocall(ocall_ctx)) {
		dev_err(dev, "Ocall expected\n");
		return false;
	}

	if (ocall_ctx->ocall_arg.in_param1 != PTA_SCMI_OCALL_CMD_THREAD_READY) {
		dev_err(dev, "Unexpected Ocall function %#x\n",
			ocall_ctx->ocall_arg.in_param1);
		return false;
	}

	return true;
}

static int setup_ocall_thread(struct scmi_optee_channel *channel)
{
	struct device *dev = channel->agent->dev;
	int ret;

	if (WARN_ONCE(channel->ocall_ctx, "Unexpected error\n"))
		return -EINVAL;

	ret = alloc_ocall_ctx(channel);
	if (ret)
		return ret;

	/*
	 * Setup parameters for initial TEE invocation with an Ocall
	 * context to return from tee_client_invoke_func() with
	 * a provisioned OP-TEE thread.
	 */
	if (channel->tee_shm) {
		*channel->ocall_ctx = (struct ocall_ctx){
			.args.func = PTA_SCMI_CMD_OCALL2_MSG_THREAD,
			.args.session = channel->tee_session,
			.args.num_params = 3,
			.param[0] = {
				.attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT,
				.u.value.a = channel->channel_id,
			},
			.param[1] = {
				.attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT,
				.u.memref.shm = channel->tee_shm,
				.u.memref.size = SCMI_SHMEM_MAX_PAYLOAD_SIZE,
			},
			.param[2] = {
				.attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT,
				.u.memref.shm = channel->tee_shm,
				.u.memref.size = SCMI_SHMEM_MAX_PAYLOAD_SIZE,
			},
			.ocall_arg = TEE_OCALL2_ARG_INIT,
		};
	} else {
		*channel->ocall_ctx = (struct ocall_ctx){
			.args.func = PTA_SCMI_CMD_OCALL2_SMT_THREAD,
			.args.session = channel->tee_session,
			.args.num_params = 1,
			.param[0] = {
				.attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT,
				.u.value.a = channel->channel_id,
			},
			.ocall_arg = TEE_OCALL2_ARG_INIT,
		};
	}

	/* This is the initial invocation that should return in an Ocall */
	ret = invoke_optee_ocall(channel);
	if (ret)
		goto err;

	if (ocall_thread_is_ready(channel))
		return 0;

	ret = -EPROTO;

	if (!return_is_ocall(channel->ocall_ctx)) {
		struct ocall_ctx *ocall_ctx = channel->ocall_ctx;

		switch (ocall_ctx->args.ret) {
		case TEEC_SUCCESS:
			dev_dbg(dev, "unexpected successful invocation\n");
			break;
		case TEEC_ERROR_NOT_SUPPORTED:
			ret = -EOPNOTSUPP;
			break;
		default:
			dev_dbg(dev, "invoke error %#x\n", ocall_ctx->args.ret);
			break;
		}
	} else {
		dev_dbg(dev, "Unexpected ocall context\n");
	}

err:
	if (return_is_ocall(channel->ocall_ctx))
		abort_ocall(channel);
	free_ocall_ctx(channel);

	return ret;
}

static int close_ocall_thread(struct scmi_optee_channel *channel)
{
	struct ocall_ctx *ocall_ctx = channel->ocall_ctx;
	int ret;

	if (!ocall_ctx)
		return 0;

	ocall_ctx->ocall_arg.out_param1 = PTA_SCMI_OCALL_CLOSE_THREAD;
	ocall_ctx->ocall_arg.out_param2 = 0;

	ret = invoke_optee_ocall(channel);

	if (ret) {
		dev_dbg(channel->agent->dev, "can't invoke OP-TEE: %d\n", ret);
	} else {
		if (return_is_ocall(channel->ocall_ctx)) {
			ret = -EPROTO;
			abort_ocall(channel);
		}
	}

	free_ocall_ctx(channel);

	return ret;
}

static int invoke_ocall_thread(struct scmi_optee_channel *channel,
			       size_t msg_size)
{
	struct ocall_ctx *ocall_ctx = channel->ocall_ctx;
	int ret;

	ocall_ctx->ocall_arg.out_param1 = PTA_SCMI_OCALL_PROCESS_SMT_MESSAGE;
	ocall_ctx->ocall_arg.out_param2 = msg_size;

	ret = invoke_optee_ocall(channel);

	if (!ret && ocall_thread_is_ready(channel))
		return 0;

	if (return_is_ocall(channel->ocall_ctx))
		abort_ocall(channel);

	free_ocall_ctx(channel);

	return -EPROTO;
}

static int invoke_process_smt_channel(struct scmi_optee_channel *channel)
{
	struct tee_ioctl_invoke_arg arg = {
		.func = PTA_SCMI_CMD_PROCESS_SMT_CHANNEL,
		.session = channel->tee_session,
		.num_params = 1,
	};
	struct tee_param param[1] = { };
	int ret;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = channel->channel_id;

	ret = tee_client_invoke_func(channel->agent->tee_ctx, &arg, param);
	if (ret < 0 || arg.ret) {
		dev_err(channel->agent->dev, "Can't invoke channel %u: %d / %#x\n",
			channel->channel_id, ret, arg.ret);
		return -EIO;
	}

	return 0;
}

static int invoke_process_msg_channel(struct scmi_optee_channel *channel, size_t msg_size)
{
	struct tee_ioctl_invoke_arg arg = {
		.func = PTA_SCMI_CMD_PROCESS_MSG_CHANNEL,
		.session = channel->tee_session,
		.num_params = 3,
	};
	struct tee_param param[3] = { };
	int ret;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = channel->channel_id;

	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	param[1].u.memref.shm = channel->tee_shm;
	param[1].u.memref.size = msg_size;

	param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT;
	param[2].u.memref.shm = channel->tee_shm;
	param[2].u.memref.size = SCMI_SHMEM_MAX_PAYLOAD_SIZE;

	ret = tee_client_invoke_func(channel->agent->tee_ctx, &arg, param);
	if (ret < 0 || arg.ret) {
		dev_err(channel->agent->dev, "Can't invoke channel %u: %d / %#x\n",
			channel->channel_id, ret, arg.ret);
		return -EIO;
	}

	/* Save response size */
	channel->rx_len = param[2].u.memref.size;

	return 0;
}

static bool scmi_optee_chan_available(struct device_node *of_node, int idx)
{
	u32 channel_id;

	return !of_property_read_u32_index(of_node, "linaro,optee-channel-id",
					   idx, &channel_id);
}

static void scmi_optee_clear_channel(struct scmi_chan_info *cinfo)
{
	struct scmi_optee_channel *channel = cinfo->transport_info;

	if (!channel->tee_shm)
		core->shmem->clear_channel(channel->req.shmem);
}

static int setup_dynamic_shmem(struct device *dev, struct scmi_optee_channel *channel)
{
	const size_t msg_size = SCMI_SHMEM_MAX_PAYLOAD_SIZE;
	void *shbuf;

	channel->tee_shm = tee_shm_alloc_kernel_buf(scmi_optee_private->tee_ctx, msg_size);
	if (IS_ERR(channel->tee_shm)) {
		dev_err(channel->cinfo->dev, "shmem allocation failed\n");
		return -ENOMEM;
	}

	shbuf = tee_shm_get_va(channel->tee_shm, 0);
	memset(shbuf, 0, msg_size);
	channel->req.msg = shbuf;
	channel->rx_len = msg_size;

	return 0;
}

static int setup_static_shmem(struct device *dev, struct scmi_chan_info *cinfo,
			      struct scmi_optee_channel *channel)
{
	channel->req.shmem = core->shmem->setup_iomap(cinfo, dev, true, NULL,
						      &channel->io_ops);
	if (IS_ERR(channel->req.shmem))
		return PTR_ERR(channel->req.shmem);

	return 0;
}

static int setup_shmem(struct device *dev, struct scmi_chan_info *cinfo,
		       struct scmi_optee_channel *channel)
{
	if (of_property_present(cinfo->dev->of_node, "shmem"))
		return setup_static_shmem(dev, cinfo, channel);
	else
		return setup_dynamic_shmem(dev, channel);
}

static int scmi_optee_chan_setup(struct scmi_chan_info *cinfo, struct device *dev, bool tx)
{
	struct scmi_optee_channel *channel;
	uint32_t channel_id;
	int ret;

	if (!tx)
		return -ENODEV;

	channel = devm_kzalloc(dev, sizeof(*channel), GFP_KERNEL);
	if (!channel)
		return -ENOMEM;

	ret = of_property_read_u32_index(cinfo->dev->of_node, "linaro,optee-channel-id",
					 0, &channel_id);
	if (ret)
		return ret;

	cinfo->transport_info = channel;
	channel->cinfo = cinfo;
	channel->channel_id = channel_id;
	channel->agent = scmi_optee_private;
	mutex_init(&channel->mu);

	ret = setup_shmem(dev, cinfo, channel);
	if (ret)
		return ret;

	ret = open_session(channel->agent, &channel->tee_session);
	if (ret)
		goto err_free_shm;

	ret = tee_client_system_session(scmi_optee_private->tee_ctx, channel->tee_session);
	if (ret)
		dev_warn(dev, "Could not switch to system session, do best effort\n");

	ret = get_channel(channel);
	if (ret)
		goto err_close_sess;

	if (channel->caps & PTA_SCMI_CAPS_OCALL2_THREAD) {
		ret = setup_ocall_thread(channel);
		if (ret) {
			if (ret != -EOPNOTSUPP)
				goto err_close_sess;

			dev_warn(dev, "Ocall failed, will use regular calls\n");
		}
	}

	/* Enable polling */
	cinfo->no_completion_irq = true;

	mutex_lock(&channel->agent->mu);
	list_add(&channel->link, &channel->agent->channel_list);
	mutex_unlock(&channel->agent->mu);

	return 0;

err_close_sess:
	close_session(channel->agent, channel->tee_session);
err_free_shm:
	if (channel->tee_shm)
		tee_shm_free(channel->tee_shm);

	return ret;
}

static int scmi_optee_chan_free(int id, void *p, void *data)
{
	struct scmi_chan_info *cinfo = p;
	struct scmi_optee_channel *channel = cinfo->transport_info;
	int ret;

	/*
	 * Different protocols might share the same chan info, so a previous
	 * call might have already freed the structure.
	 */
	if (!channel)
		return 0;

	ret = close_ocall_thread(channel);
	if (ret)
		return ret;

	mutex_lock(&channel->agent->mu);
	list_del(&channel->link);
	mutex_unlock(&channel->agent->mu);

	close_session(channel->agent, channel->tee_session);

	if (channel->tee_shm) {
		tee_shm_free(channel->tee_shm);
		channel->tee_shm = NULL;
	}

	cinfo->transport_info = NULL;
	channel->cinfo = NULL;

	devm_kfree(channel->agent->dev, channel);

	return 0;
}

static int scmi_optee_send_message(struct scmi_chan_info *cinfo,
				   struct scmi_xfer *xfer)
{
	struct scmi_optee_channel *channel = cinfo->transport_info;
	int ret;

	mutex_lock(&channel->mu);

	if (channel->tee_shm) {
		core->msg->tx_prepare(channel->req.msg, xfer);

		if (channel->ocall_ctx)
			ret = invoke_ocall_thread(channel,
						  core->msg->command_size(xfer));
		else
			ret = invoke_process_msg_channel(channel,
							 core->msg->command_size(xfer));
	} else {
		core->shmem->tx_prepare(channel->req.shmem, xfer, cinfo,
					channel->io_ops->toio);

		if (channel->ocall_ctx)
			ret = invoke_ocall_thread(channel, 0);
		else
			ret = invoke_process_smt_channel(channel);
	}

	if (ret)
		mutex_unlock(&channel->mu);

	return ret;
}

static void scmi_optee_fetch_response(struct scmi_chan_info *cinfo,
				      struct scmi_xfer *xfer)
{
	struct scmi_optee_channel *channel = cinfo->transport_info;

	if (channel->tee_shm)
		core->msg->fetch_response(channel->req.msg,
					  channel->rx_len, xfer);
	else
		core->shmem->fetch_response(channel->req.shmem, xfer,
					    channel->io_ops->fromio);
}

static void scmi_optee_mark_txdone(struct scmi_chan_info *cinfo, int ret,
				   struct scmi_xfer *__unused)
{
	struct scmi_optee_channel *channel = cinfo->transport_info;

	mutex_unlock(&channel->mu);
}

static const struct scmi_transport_ops scmi_optee_ops = {
	.chan_available = scmi_optee_chan_available,
	.chan_setup = scmi_optee_chan_setup,
	.chan_free = scmi_optee_chan_free,
	.send_message = scmi_optee_send_message,
	.mark_txdone = scmi_optee_mark_txdone,
	.fetch_response = scmi_optee_fetch_response,
	.clear_channel = scmi_optee_clear_channel,
};

static int scmi_optee_ctx_match(struct tee_ioctl_version_data *ver, const void *data)
{
	return ver->impl_id == TEE_IMPL_ID_OPTEE;
}

static struct scmi_desc scmi_optee_desc = {
	.ops = &scmi_optee_ops,
	.max_rx_timeout_ms = 30,
	.max_msg = 20,
	.max_msg_size = SCMI_SHMEM_MAX_PAYLOAD_SIZE,
	.sync_cmds_completed_on_ret = true,
};

static const struct of_device_id scmi_of_match[] = {
	{ .compatible = "linaro,scmi-optee" },
	{ /* Sentinel */ },
};

DEFINE_SCMI_TRANSPORT_DRIVER(scmi_optee, scmi_optee_driver, scmi_optee_desc,
			     scmi_of_match, core);

static int scmi_optee_service_probe(struct device *dev)
{
	struct scmi_optee_agent *agent;
	struct tee_context *tee_ctx;
	int ret;

	/* Only one SCMI OP-TEE device allowed */
	if (scmi_optee_private) {
		dev_err(dev, "An SCMI OP-TEE device was already initialized: only one allowed\n");
		return -EBUSY;
	}

	tee_ctx = tee_client_open_context(NULL, scmi_optee_ctx_match, NULL, NULL);
	if (IS_ERR(tee_ctx))
		return -ENODEV;

	agent = devm_kzalloc(dev, sizeof(*agent), GFP_KERNEL);
	if (!agent) {
		ret = -ENOMEM;
		goto err;
	}

	agent->dev = dev;
	agent->tee_ctx = tee_ctx;
	INIT_LIST_HEAD(&agent->channel_list);
	mutex_init(&agent->mu);

	ret = get_capabilities(agent);
	if (ret)
		goto err;

	/* Ensure agent resources are all visible before scmi_optee_private is */
	smp_mb();
	scmi_optee_private = agent;

	ret = platform_driver_register(&scmi_optee_driver);
	if (ret) {
		scmi_optee_private = NULL;
		goto err;
	}

	return 0;

err:
	tee_client_close_context(tee_ctx);

	return ret;
}

static int scmi_optee_service_remove(struct device *dev)
{
	struct scmi_optee_agent *agent = scmi_optee_private;

	if (!scmi_optee_private)
		return -EINVAL;

	platform_driver_unregister(&scmi_optee_driver);

	if (!list_empty(&scmi_optee_private->channel_list))
		return -EBUSY;

	/* Ensure cleared reference is visible before resources are released */
	smp_store_mb(scmi_optee_private, NULL);

	tee_client_close_context(agent->tee_ctx);

	return 0;
}

static const struct tee_client_device_id scmi_optee_service_id[] = {
	{
		UUID_INIT(0xa8cfe406, 0xd4f5, 0x4a2e,
			  0x9f, 0x8d, 0xa2, 0x5d, 0xc7, 0x54, 0xc0, 0x99)
	},
	{ }
};

MODULE_DEVICE_TABLE(tee, scmi_optee_service_id);

static struct tee_client_driver scmi_optee_service_driver = {
	.id_table	= scmi_optee_service_id,
	.driver		= {
		.name = "scmi-optee",
		.bus = &tee_bus_type,
		.probe = scmi_optee_service_probe,
		.remove = scmi_optee_service_remove,
	},
};

static int __init scmi_transport_optee_init(void)
{
	return driver_register(&scmi_optee_service_driver.driver);
}
module_init(scmi_transport_optee_init);

static void __exit scmi_transport_optee_exit(void)
{
	driver_unregister(&scmi_optee_service_driver.driver);
}
module_exit(scmi_transport_optee_exit);

MODULE_AUTHOR("Etienne Carriere <etienne.carriere@foss.st.com>");
MODULE_DESCRIPTION("SCMI OPTEE Transport driver");
MODULE_LICENSE("GPL");
