// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for STM32 Digital Camera Memory Interface Pixel Processor
 *
 * Copyright (C) STMicroelectronics SA 2022
 * Authors: Alain Volmat <alain.volmat@foss.st.com>
 *          Fabien Dessenne <fabien.dessenne@foss.st.com>
 *          Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          for STMicroelectronics.
 */

#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include <uapi/linux/stm32-dcmipp-config.h>

#include "dcmipp-common.h"

#define DCMIPP_STATCAP_DRV_NAME "dcmipp-statcap"

#define DCMIPP_CMSR2_P1VSYNCF BIT(18)

#define DCMIPP_P1BPRSR (0x828)
#define DCMIPP_P1BPRSR_BADCNT_MASK GENMASK(11, 0)

#define DCMIPP_P1STXCR(a)	(0x850 + ((a) * 0x4))
#define DCMIPP_P1STXCR_ENABLE		BIT(0)
#define DCMIPP_P1STXCR_BINS_SHIFT	2
#define DCMIPP_P1STXCR_SRC_COMP_SHIFT	4
#define DCMIPP_P1STXCR_SRC_LOC_PRE	0
#define DCMIPP_P1STXCR_SRC_LOC_POST	1
#define DCMIPP_P1STXCR_SRC_LOC_SHIFT	6
#define DCMIPP_P1STXCR_MODE_AVERAGE	0
#define DCMIPP_P1STXCR_MODE_BINS	BIT(7)

#define DCMIPP_P1STSTR	(0x85C)
#define DCMIPP_P1STSTR_HSTART_SHIFT	0
#define DCMIPP_P1STSTR_HSTART_MASK	GENMASK(11, 0)
#define DCMIPP_P1STSTR_VSTART_SHIFT	16
#define DCMIPP_P1STSTR_VSTART_MASK	GENMASK(27, 16)

#define DCMIPP_P1STSZR	(0x860)
#define DCMIPP_P1STSZR_HSIZE_SHIFT	0
#define DCMIPP_P1STSZR_HSIZE_MASK	GENMASK(11, 0)
#define DCMIPP_P1STSZR_VSIZE_SHIFT	16
#define DCMIPP_P1STSZR_VSIZE_MASK	GENMASK(27, 16)
#define DCMIPP_P1STSZR_ENABLE		BIT(31)

#define DCMIPP_P1STXSR(a) (0x864 + ((a) * 0x4))

#define DCMIPP_NB_STAT_REGION	1

struct dcmipp_buf {
	struct vb2_v4l2_buffer	vb;
	bool			prepared;
	dma_addr_t		paddr;
	size_t			size;
	struct list_head	list;
};

/*
 * This structure describe the state right after the VSYNC comes
 */
enum stat_capture_state {
	COLD_START,		/* Shadow: AVERAGE (RGB), Physical: stopped */
	/* Full capture profile */
	PHY_AV_RGB_SHA_BIN_0,	/* Shadow: BIN_0, Physical: AVERAGE (RGB) */
	PHY_BIN_0_SHA_BIN_1,	/* Shadow: BIN_1, Physical: BIN_0 */
	PHY_BIN_1_SHA_BIN_2,	/* Shadow: BIN_2, Physical: BIN_1 */
	PHY_BIN_2_SHA_BIN_3,	/* Shadow: BIN_3, Physical: BIN_2 */
	PHY_BIN_3_SHA_AV_RGB,	/* Shadow: AVERAGE (RGB), Physical: BIN_3 */
	/* Average pre-post profile */
	PHY_AV_RGB_SHA_AV_RGB,	/* Shadow: AVERAGE (RGB), Physical: AVERAGE (RGB) */
};

enum component {
	COMP_RED = 0,
	COMP_GREEN,
	COMP_BLUE,
	COMP_MAX
};

struct dcmipp_statcap_device {
	struct dcmipp_ent_device ved;
	struct video_device vdev;
	struct device *dev;
	struct v4l2_ctrl_handler ctrls;
	/* Protect ctrls */
	struct vb2_queue queue;
	struct list_head buffers;
	/* Protects the access of variables shared within the interrupt */
	spinlock_t irqlock;
	/* Protect this data structure */
	struct mutex lock;
	u32 sequence;
	struct media_pipeline pipe;
	u32 frame_format;
	struct v4l2_rect stat_region;
	enum v4l2_isp_stat_avg_filter avg_filter;
	enum v4l2_isp_stat_bin_comp bin_comp;
	enum v4l2_isp_stat_profile stat_profile;
	struct v4l2_subdev *s_subdev;

	/*
	 * indicate the current state of the capture stat machine,
	 * must be updated at the end of the VSYNC processing
	 */
	enum stat_capture_state capture_state;
	/*
	 * indicate the previous state of the capture stat machine,
	 * must be updated at the end of the VSYNC processing
	 * this is useful only for startup cases since, in case of startup
	 * we cannot capture BIN_3 at stage PHY_AV_RGB_SHA_BIN_0 since we
	 * have just started.
	 */
	enum stat_capture_state prev_capture_state;

	void __iomem *regs;

	struct stm32_dcmipp_stat_buf local_buf;
};

static int dcmipp_statcap_querycap(struct file *file, void *priv,
				   struct v4l2_capability *cap)
{
	strscpy(cap->driver, DCMIPP_PDEV_NAME, sizeof(cap->driver));
	strscpy(cap->card, KBUILD_MODNAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "platform:%s", DCMIPP_PDEV_NAME);

	return 0;
}

static int dcmipp_statcap_g_fmt_meta_cap(struct file *file, void *priv,
					 struct v4l2_format *f)
{
	struct v4l2_meta_format *meta = &f->fmt.meta;

	meta->dataformat = V4L2_META_FMT_ST_DCMIPP_ISP_STAT;
	meta->buffersize = sizeof(struct stm32_dcmipp_stat_buf);

	return 0;
}

static int dcmipp_statcap_enum_fmt_meta_cap(struct file *file, void *priv,
					    struct v4l2_fmtdesc *f)
{
	if (f->index > 0)
		return -EINVAL;

	f->type = V4L2_BUF_TYPE_META_CAPTURE;
	f->pixelformat = V4L2_META_FMT_ST_DCMIPP_ISP_STAT;

	return 0;
}

static int dcmipp_statcap_open(struct file *file)
{
	struct dcmipp_statcap_device *vcap = video_drvdata(file);
	int ret;

	ret = mutex_lock_interruptible(&vcap->lock);
	if (ret)
		return ret;

	ret = v4l2_fh_open(file);
	if (ret)
		goto err_unlock;

	ret = v4l2_pipeline_pm_get(&vcap->vdev.entity);
	if (ret)
		goto err_close;

	mutex_unlock(&vcap->lock);

	return 0;

err_close:
	v4l2_fh_release(file);
err_unlock:
	mutex_unlock(&vcap->lock);

	return ret;
}

static int dcmipp_statcap_close(struct file *file)
{
	struct dcmipp_statcap_device *vcap = video_drvdata(file);

	vb2_fop_release(file);

	v4l2_pipeline_pm_put(&vcap->vdev.entity);

	return 0;
}

static const struct v4l2_file_operations dcmipp_statcap_fops = {
	.owner		= THIS_MODULE,
	.open		= dcmipp_statcap_open,
	.release	= dcmipp_statcap_close,
	.read           = vb2_fop_read,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = vb2_fop_mmap,
};

static const struct v4l2_ioctl_ops dcmipp_statcap_ioctl_ops = {
	.vidioc_querycap = dcmipp_statcap_querycap,

	.vidioc_enum_fmt_meta_cap = dcmipp_statcap_enum_fmt_meta_cap,
	.vidioc_g_fmt_meta_cap = dcmipp_statcap_g_fmt_meta_cap,
	.vidioc_s_fmt_meta_cap = dcmipp_statcap_g_fmt_meta_cap,
	.vidioc_try_fmt_meta_cap = dcmipp_statcap_g_fmt_meta_cap,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static int dcmipp_pipeline_s_stream(struct dcmipp_statcap_device *vcap,
				    int state)
{
	struct media_entity *entity = &vcap->vdev.entity;
	struct media_device *mdev = entity->graph_obj.mdev;
	struct v4l2_subdev *subdev;
	struct media_pad *pad;
	int ret = 0;

	mutex_lock(&mdev->graph_mutex);

	/* Start/stop all entities within pipeline */
	while (1) {
		pad = &entity->pads[0];
		if (!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		pad = media_pad_remote_pad_first(pad);
		if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
			break;

		entity = pad->entity;
		subdev = media_entity_to_v4l2_subdev(entity);

		if (state) {
			/* Increment stream_count to indicate that entity is streamon */
			entity->stream_count++;

			/*
			 * Do not streamon entities already started and streamon
			 * by another capture pipeline
			 */
			if (entity->stream_count > 1)
				continue;
		} else {
			/* Decrement stream_count to indicate that entity is streamoff. */
			entity->stream_count--;

			/*
			 * Only streamoff if entity is not owned anymore
			 * by other pipelines
			 */
			if (entity->stream_count > 0)
				continue;
		}

		ret = v4l2_subdev_call(subdev, video, s_stream, state);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			dev_err(vcap->dev, "%s: \"%s\" failed to %s streaming (%d)\n",
				__func__, subdev->name,
				state ? "start" : "stop", ret);

			goto out;
		}

		dev_dbg(vcap->dev, "\"%s\" is %s\n",
			subdev->name, state ? "started" : "stopped");
	}

out:
	mutex_unlock(&mdev->graph_mutex);

	return ret;
}

static int dcmipp_statcap_start_streaming(struct vb2_queue *vq,
					  unsigned int count)
{
	struct dcmipp_statcap_device *vcap = vb2_get_drv_priv(vq);
	struct media_entity *entity = &vcap->vdev.entity;
	struct media_device *mdev = entity->graph_obj.mdev;
	struct media_pipeline *pipe;
	struct dcmipp_buf *buf, *node;
	struct v4l2_subdev_format fmt;
	struct media_pad *pad;
	int ret;

	vcap->sequence = 0;

	ret = pm_runtime_get_sync(vcap->dev);
	if (ret < 0) {
		dev_err(vcap->dev, "%s: Failed to start streaming, cannot get sync (%d)\n",
			__func__, ret);
		goto err_pm_put;
	}

	/*
	 * Start the media pipeline
	 *
	 * Pipeline is shared between all elements of the pipeline
	 * including video capture nodes.
	 * Instead of creating a common media_pipeline struct
	 * global variable, use the one of the first capture
	 * node. All the elements of the pipeline -including
	 * other capture nodes- will be then assigned to this
	 * pipeline (entity->pipe) in __media_pipeline_start().
	 */
	mutex_lock(&mdev->graph_mutex);
	pipe = entity->pads[0].pipe ? : &vcap->pipe;
	ret = __video_device_pipeline_start(&vcap->vdev, pipe);
	mutex_unlock(&mdev->graph_mutex);
	if (ret) {
		dev_err(vcap->dev, "%s: Failed to start streaming, media pipeline start error (%d)\n",
			__func__, ret);
		goto err_pm_put;
	}

	/* Start all the elements within pipeline */
	ret = dcmipp_pipeline_s_stream(vcap, 1);
	if (ret)
		goto err_media_pipeline_stop;

	/* Get pointer to the source subdev (if case of not yet set */
	if (!vcap->s_subdev) {
		pad = media_pad_remote_pad_first(&entity->pads[0]);
		if (!pad || !is_media_entity_v4l2_subdev(pad->entity)) {
			dev_err(vcap->dev, "%s: Failed to start streaming, can't find remote entity\n",
				__func__);
			ret = -EIO;
			goto err_media_pipeline_stop;
		}
		vcap->s_subdev = media_entity_to_v4l2_subdev(pad->entity);
	}

	/* Get frame format info from ISP sink pad */
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.pad = 0;
	ret = v4l2_subdev_call(vcap->s_subdev, pad, get_fmt, NULL, &fmt);
	if (ret < 0) {
		dev_err(vcap->dev, "%s: Failed to start streaming, can't get format (%d)\n",
			__func__, ret);
		goto err_media_pipeline_stop;
	}
	vcap->frame_format = fmt.format.code;

	/* Apply controls if not yet done */
	ret = v4l2_ctrl_handler_setup(&vcap->ctrls);
	if (ret < 0) {
		dev_err(vcap->dev, "Failed to set up control handlers (%d)\n", ret);
		goto err_media_pipeline_stop;
	}

	return 0;

err_media_pipeline_stop:
	media_pipeline_stop(entity->pads);
err_pm_put:
	pm_runtime_put(vcap->dev);
	spin_lock_irq(&vcap->irqlock);
	/*
	 * Return all buffers to vb2 in QUEUED state.
	 * This will give ownership back to userspace
	 */
	list_for_each_entry_safe(buf, node, &vcap->buffers, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
	}
	spin_unlock_irq(&vcap->irqlock);

	return ret;
}

/*
 * Stop the stream engine. Any remaining buffers in the stream queue are
 * dequeued and passed on to the vb2 framework marked as STATE_ERROR.
 */
static void dcmipp_statcap_stop_streaming(struct vb2_queue *vq)
{
	struct dcmipp_statcap_device *vcap = vb2_get_drv_priv(vq);
	struct dcmipp_buf *buf, *node;

	/* Reset the capture state machine */
	vcap->capture_state = COLD_START;

	dcmipp_pipeline_s_stream(vcap, 0);

	/* Stop the media pipeline */
	media_pipeline_stop(vcap->vdev.entity.pads);

	spin_lock_irq(&vcap->irqlock);

	/* Return all queued buffers to vb2 in ERROR state */
	list_for_each_entry_safe(buf, node, &vcap->buffers, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	INIT_LIST_HEAD(&vcap->buffers);

	spin_unlock_irq(&vcap->irqlock);

	pm_runtime_put(vcap->dev);
}

static int dcmipp_statcap_buf_prepare(struct vb2_buffer *vb)
{
	struct dcmipp_statcap_device *vcap =  vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = sizeof(struct stm32_dcmipp_stat_buf);

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(vcap->dev, "%s data will not fit into plane (%lu < %lu)\n",
			__func__, vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

static void dcmipp_statcap_buf_queue(struct vb2_buffer *vb2_buf)
{
	struct dcmipp_statcap_device *vcap = vb2_get_drv_priv(vb2_buf->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb2_buf);
	struct dcmipp_buf *buf = container_of(vbuf, struct dcmipp_buf, vb);

	spin_lock_irq(&vcap->irqlock);
	list_add_tail(&buf->list, &vcap->buffers);
	spin_unlock_irq(&vcap->irqlock);
}

static int dcmipp_statcap_queue_setup(struct vb2_queue *vq,
				      unsigned int *nbuffers,
				      unsigned int *nplanes,
				      unsigned int sizes[],
				      struct device *alloc_devs[])
{
	unsigned int size = sizeof(struct stm32_dcmipp_stat_buf);

	if (*nplanes) {
		if (sizes[0] < size)
			return -EINVAL;
	} else {
		sizes[0] = size;
	}

	if (vq->num_buffers + *nbuffers < 2)
		*nbuffers = 2 - vq->num_buffers;

	*nplanes = 1;
	return 0;
}

static int dcmipp_statcap_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct dcmipp_buf *buf = container_of(vbuf, struct dcmipp_buf, vb);

	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static const struct vb2_ops dcmipp_statcap_qops = {
	.start_streaming	= dcmipp_statcap_start_streaming,
	.stop_streaming		= dcmipp_statcap_stop_streaming,
	.buf_init		= dcmipp_statcap_buf_init,
	.buf_prepare		= dcmipp_statcap_buf_prepare,
	.buf_queue		= dcmipp_statcap_buf_queue,

	.queue_setup		= dcmipp_statcap_queue_setup,
	/*
	 * Since q->lock is set we can use the standard
	 * vb2_ops_wait_prepare/finish helper functions.
	 */
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static const struct media_entity_operations dcmipp_statcap_mops = {
	.link_validate		= dcmipp_link_validate,
};

static int dcmipp_statcap_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dcmipp_statcap_device *vcap =
			container_of(ctrl->handler, struct dcmipp_statcap_device, ctrls);
	struct media_entity *entity = &vcap->vdev.entity;
	struct v4l2_ctrl_isp_stat_region *region;
	struct v4l2_subdev_selection sel;
	struct media_pad *pad;
	int ret = 0;

	dev_dbg(vcap->dev, ">> %s: ctrl->id = 0x%x\n", __func__, ctrl->id);

	if (pm_runtime_get_if_in_use(vcap->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ISP_STAT_REGION:
		/* Get pointer to the source subdev (if case of not yet set */
		if (!vcap->s_subdev) {
			pad = media_pad_remote_pad_first(&entity->pads[0]);
			if (!pad || !is_media_entity_v4l2_subdev(pad->entity)) {
				ret = -EIO;
				goto out;
			}
			vcap->s_subdev = media_entity_to_v4l2_subdev(pad->entity);
		}

		region = (struct v4l2_ctrl_isp_stat_region *)ctrl->p_new.p;
		if (region->nb_regions > DCMIPP_NB_STAT_REGION) {
			dev_dbg(vcap->dev, "Unsupported number of stat region: %d vs max=%d\n",
				region->nb_regions, DCMIPP_NB_STAT_REGION);
			ret = -EINVAL;
			goto out;
		}

		/* Get frame information */
		sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		sel.pad = 0;
		sel.target = V4L2_SEL_TGT_COMPOSE;
		ret = v4l2_subdev_call(vcap->s_subdev, pad, get_selection, NULL, &sel);
		if (ret < 0) {
			dev_err(vcap->dev, "Failed to get frame size\n");
			goto out;
		}

		if (!region->nb_regions ||
		    (!region->width[0] && !region->height[0] &&
		     !region->left[0] && !region->top[0])) {
			spin_lock_irq(&vcap->irqlock);
			vcap->stat_region.width = sel.r.width;
			vcap->stat_region.height = sel.r.width;
			reg_clear(vcap, DCMIPP_P1STSZR, DCMIPP_P1STSZR_ENABLE);
			spin_unlock_irq(&vcap->irqlock);
			break;
		} else if (!region->width[0] || !region->height[0] ||
			   (region->left[0] + region->width[0]) > sel.r.width ||
			   (region->top[0] + region->height[0]) > sel.r.height) {
			dev_err(vcap->dev, "Invalid or stat region not fitting into frame\n");
			ret = -EINVAL;
			goto out;
		}

		spin_lock_irq(&vcap->irqlock);
		vcap->stat_region.width = region->width[0];
		vcap->stat_region.height = region->height[0];
		vcap->stat_region.left = region->left[0];
		vcap->stat_region.top = region->top[0];

		/* Update window size and start */
		reg_clear(vcap, DCMIPP_P1STSTR,
			  DCMIPP_P1STSTR_HSTART_MASK | DCMIPP_P1STSTR_VSTART_MASK);
		reg_set(vcap, DCMIPP_P1STSTR,
			(region->left[0] << DCMIPP_P1STSTR_HSTART_SHIFT) |
			(region->top[0] << DCMIPP_P1STSTR_VSTART_SHIFT));

		reg_clear(vcap, DCMIPP_P1STSZR,
			  DCMIPP_P1STSZR_HSIZE_MASK | DCMIPP_P1STSZR_VSIZE_MASK);
		reg_set(vcap, DCMIPP_P1STSZR,
			(region->width[0] << DCMIPP_P1STSZR_HSIZE_SHIFT) |
			(region->height[0] << DCMIPP_P1STSZR_VSIZE_SHIFT) |
			DCMIPP_P1STSZR_ENABLE);
		spin_unlock_irq(&vcap->irqlock);
		break;
	case V4L2_CID_ISP_STAT_AVG_FILTER:
		vcap->avg_filter = ctrl->val;
		break;
	case V4L2_CID_ISP_STAT_BIN_COMP:
		vcap->bin_comp = ctrl->val;
		break;
	case V4L2_CID_ISP_STAT_PROFILE:
		vcap->stat_profile = ctrl->val;
		break;
	}

out:
	pm_runtime_put(vcap->dev);

	return ret;
};

static const struct v4l2_ctrl_ops dcmipp_statcap_ctrl_ops = {
	.s_ctrl = dcmipp_statcap_s_ctrl,
};

static const struct v4l2_ctrl_config dcmipp_statcap_ctrls[] = {
	{
		.ops	= &dcmipp_statcap_ctrl_ops,
		.id	= V4L2_CID_ISP_STAT_REGION,
		.type	= V4L2_CTRL_TYPE_ISP_STAT_REGION,
		.name	= "ISP stat region control",
	}, {
		.ops	= &dcmipp_statcap_ctrl_ops,
		.id	= V4L2_CID_ISP_STAT_AVG_FILTER,
		.type	= V4L2_CTRL_TYPE_INTEGER,
		.name	= "ISP stat average filter control",
		.min	= 0,
		.max	= V4L2_STAT_AVG_FILTER_EXCL64,
		.step	= 1,
		.def	= 0,
		.flags	= 0,
	}, {
		.ops	= &dcmipp_statcap_ctrl_ops,
		.id	= V4L2_CID_ISP_STAT_BIN_COMP,
		.type	= V4L2_CTRL_TYPE_INTEGER,
		.name	= "ISP stat bin component control",
		.min	= 0,
		.max	= V4L2_STAT_BIN_COMP_L,
		.step	= 1,
		.def	= 0,
		.flags	= 0,
	}, {
		.ops	= &dcmipp_statcap_ctrl_ops,
		.id	= V4L2_CID_ISP_STAT_PROFILE,
		.type	= V4L2_CTRL_TYPE_INTEGER,
		.name	= "ISP stat profile control",
		.min	= 0,
		.max	= V4L2_STAT_PROFILE_AVERAGE_POST,
		.step	= 1,
		.def	= 0,
		.flags	= 0,
	}
};

static void dcmipp_statcap_release(struct video_device *vdev)
{
	struct dcmipp_statcap_device *vcap =
		container_of(vdev, struct dcmipp_statcap_device, vdev);

	dcmipp_pads_cleanup(vcap->ved.pads);
	kfree(vcap);
}

void dcmipp_statcap_ent_release(struct dcmipp_ent_device *ved)
{
	struct dcmipp_statcap_device *vcap =
		container_of(ved, struct dcmipp_statcap_device, ved);

	mutex_destroy(&vcap->lock);
	media_entity_cleanup(ved->ent);
	vb2_video_unregister_device(&vcap->vdev);
}

static void dcmipp_statcap_buffer_done(struct dcmipp_statcap_device *vcap)
{
	struct stm32_dcmipp_stat_buf *stat_buf;
	struct dcmipp_buf *cur_buf = NULL;

	/* Get an available buffer */
	if (!list_empty(&vcap->buffers)) {
		cur_buf = list_first_entry(&vcap->buffers, struct dcmipp_buf, list);
		list_del(&cur_buf->list);
	}
	if (!cur_buf)
		return;

	stat_buf = (struct stm32_dcmipp_stat_buf *)vb2_plane_vaddr(&cur_buf->vb.vb2_buf, 0);
	*stat_buf = vcap->local_buf;

	/* Send buffer */
	vb2_set_plane_payload(&cur_buf->vb.vb2_buf, 0,
			      sizeof(struct stm32_dcmipp_stat_buf));
	cur_buf->vb.sequence = vcap->sequence++;
	cur_buf->vb.vb2_buf.timestamp = ktime_get_ns();
	vb2_buffer_done(&cur_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

static u32 dcmipp_statcap_get_src(u32 location,
				  enum v4l2_isp_stat_bin_comp comp)
{
	return (location << DCMIPP_P1STXCR_SRC_LOC_SHIFT) | (comp << DCMIPP_P1STXCR_SRC_COMP_SHIFT);
}

static void dcmipp_statcap_read_avg_stats(struct dcmipp_statcap_device *vcap,
					  u32 location)
{
	struct stm32_dcmipp_stat_avr_bins *avr_bins =
		location == DCMIPP_P1STXCR_SRC_LOC_PRE ? &vcap->local_buf.pre :
							 &vcap->local_buf.post;
	int i;

	for (i = 0; i < ARRAY_SIZE(vcap->local_buf.pre.average_RGB); i++) {
		avr_bins->average_RGB[i] = reg_read(vcap, DCMIPP_P1STXSR(i));
		/* Normalize values */
		avr_bins->average_RGB[i] <<= 8;

		/* Depending on the position & component, need to adjust in case of Bayer */
		if (location == DCMIPP_P1STXCR_SRC_LOC_PRE &&
		    vcap->frame_format >= MEDIA_BUS_FMT_SBGGR8_1X8 &&
		    vcap->frame_format <= MEDIA_BUS_FMT_SRGGB16_1X16) {
			/* raw bayer: RGB component not present for all pixels */
			if (i == COMP_RED || i == COMP_BLUE)
				avr_bins->average_RGB[i] *= 4;
			else if (i == COMP_GREEN)
				avr_bins->average_RGB[i] *= 2;
		}

		/* Divide by number of pixels */
		avr_bins->average_RGB[i] /= (vcap->stat_region.width * vcap->stat_region.height);
	}
}

static irqreturn_t dcmipp_statcap_irq_thread(int irq, void *arg)
{
	struct dcmipp_statcap_device *vcap =
			container_of(arg, struct dcmipp_statcap_device, ved);
	struct dcmipp_ent_device *ved = arg;
	static u32 location = DCMIPP_P1STXCR_SRC_LOC_PRE;
	struct stm32_dcmipp_stat_avr_bins *avr_bins =
		location == DCMIPP_P1STXCR_SRC_LOC_PRE ? &vcap->local_buf.pre :
							 &vcap->local_buf.post;
	static bool stat_ready;
	int i;

	/* We only to do things if we are streaming */
	if (!vb2_is_streaming(&vcap->queue))
		return IRQ_HANDLED;

	/* We are only interested in VSYNC interrupts */
	if (!(ved->cmsr2 & DCMIPP_CMSR2_P1VSYNCF))
		return IRQ_HANDLED;

	spin_lock_irq(&vcap->irqlock);

	/* Read the bad pixel count stat and store it locally */
	vcap->local_buf.bad_pixel_count = reg_read(vcap, DCMIPP_P1BPRSR) &
						DCMIPP_P1BPRSR_BADCNT_MASK;

	/*
	 * This is the core function for statistic extraction, within the
	 * irq thread, on EACH VSYNC we update the shadow registers to
	 * read accumulators values, (except on first few frames), store
	 * them into the internal structure, and update the shadow registers
	 * to be able to get new values 2 VSYNCs after (the values read at
	 * the next VSYNC being already based on the configuration that has
	 * just been written on the previous VSYNC
	 */
	switch (vcap->capture_state) {
	case COLD_START:
		stat_ready = false;
		/*
		 * All stats profile starts from the PRE statistics, except the
		 * AVERAGE POST
		 */
		if (vcap->stat_profile == V4L2_STAT_PROFILE_AVERAGE_POST)
			location = DCMIPP_P1STXCR_SRC_LOC_POST;
		else
			location = DCMIPP_P1STXCR_SRC_LOC_PRE;
		/*
		 * We've just started, set control registers to capture
		 * AVERAGES (RGB) and leave
		 */
		for (i = 0; i < 3; i++)
			reg_write(vcap, DCMIPP_P1STXCR(i),
				  DCMIPP_P1STXCR_MODE_AVERAGE |
				  dcmipp_statcap_get_src(location, i) |
				  vcap->avg_filter << DCMIPP_P1STXCR_BINS_SHIFT |
				  DCMIPP_P1STXCR_ENABLE);
		break;

	case PHY_AV_RGB_SHA_BIN_0:
		/* Set control registers to capture the 1st set of BINS */
		for (i = 0; i < 3; i++)
			reg_write(vcap, DCMIPP_P1STXCR(i),
				  DCMIPP_P1STXCR_MODE_BINS |
				  dcmipp_statcap_get_src(location, vcap->bin_comp) |
				  0 << DCMIPP_P1STXCR_BINS_SHIFT |
				  DCMIPP_P1STXCR_ENABLE);

		if (vcap->prev_capture_state == PHY_BIN_3_SHA_AV_RGB) {
			/* Accumulators contains the 4th set of BINS */
			for (i = 0; i < 3; i++)
				avr_bins->bins[i + 9] = reg_read(vcap, DCMIPP_P1STXSR(i));
		}
		break;

	case PHY_BIN_0_SHA_BIN_1:
		/* Set control registers to capture the 2nd set of BINS */
		for (i = 0; i < 3; i++)
			reg_write(vcap, DCMIPP_P1STXCR(i),
				  DCMIPP_P1STXCR_MODE_BINS |
				  dcmipp_statcap_get_src(location, vcap->bin_comp) |
				  1 << DCMIPP_P1STXCR_BINS_SHIFT |
				  DCMIPP_P1STXCR_ENABLE);

		/* Accumulators contains the AVERAGES (RGB) */
		dcmipp_statcap_read_avg_stats(vcap, location);
		break;

	case PHY_BIN_1_SHA_BIN_2:
		/* Set control registers to capture the 3rd set of BINS */
		for (i = 0; i < 3; i++)
			reg_write(vcap, DCMIPP_P1STXCR(i),
				  DCMIPP_P1STXCR_MODE_BINS |
				  dcmipp_statcap_get_src(location, vcap->bin_comp) |
				  2 << DCMIPP_P1STXCR_BINS_SHIFT |
				  DCMIPP_P1STXCR_ENABLE);

		/* Accumulators contains the 1st set of BINS */
		for (i = 0; i < 3; i++)
			avr_bins->bins[i] = reg_read(vcap, DCMIPP_P1STXSR(i));
		break;

	case PHY_BIN_2_SHA_BIN_3:
		/* Set control registers to capture the 4th set of BINS */
		for (i = 0; i < 3; i++)
			reg_write(vcap, DCMIPP_P1STXCR(i),
				  DCMIPP_P1STXCR_MODE_BINS |
				  dcmipp_statcap_get_src(location, vcap->bin_comp) |
				  3 << DCMIPP_P1STXCR_BINS_SHIFT |
				  DCMIPP_P1STXCR_ENABLE);

		/* Accumulators contains the 2nd set of BINS */
		for (i = 0; i < 3; i++)
			avr_bins->bins[i + 3] = reg_read(vcap, DCMIPP_P1STXSR(i));
		break;

	case PHY_BIN_3_SHA_AV_RGB:
		/* Set control registers to capture the AVERAGES (RGB) */
		for (i = 0; i < 3; i++)
			/* Usage of !location is on purpose to switch to the other location */
			reg_write(vcap, DCMIPP_P1STXCR(i),
				  DCMIPP_P1STXCR_MODE_AVERAGE |
				  dcmipp_statcap_get_src(!location, i) |
				  vcap->avg_filter << DCMIPP_P1STXCR_BINS_SHIFT |
				  DCMIPP_P1STXCR_ENABLE);

		/* Accumulators contains the 3rd set of BINS */
		for (i = 0; i < 3; i++)
			avr_bins->bins[i + 6] = reg_read(vcap, DCMIPP_P1STXSR(i));
		break;

	case PHY_AV_RGB_SHA_AV_RGB:
		/* State used for the AVERAGE PRE capture mode */
		dcmipp_statcap_read_avg_stats(vcap, location);
		break;
	}

	/* If a full capture cycle has been done, output data to a buffer */
	if (stat_ready)
		dcmipp_statcap_buffer_done(vcap);

	spin_unlock_irq(&vcap->irqlock);

	/* Update the capture_state & prev_capture_state */
	switch (vcap->stat_profile) {
	case V4L2_STAT_PROFILE_FULL:
		vcap->prev_capture_state = vcap->capture_state;
		if (vcap->capture_state < PHY_BIN_3_SHA_AV_RGB) {
			vcap->capture_state++;
		} else {
			if (location == DCMIPP_P1STXCR_SRC_LOC_POST)
				stat_ready = true;
			location = !location;
			vcap->capture_state = PHY_AV_RGB_SHA_BIN_0;
		}
		break;

	case V4L2_STAT_PROFILE_AVERAGE_PRE:
	case V4L2_STAT_PROFILE_AVERAGE_POST:
		if (vcap->capture_state == COLD_START) {
			vcap->capture_state = PHY_AV_RGB_SHA_AV_RGB;
			stat_ready = true;
		}
		break;
	}

	return IRQ_HANDLED;
}

struct dcmipp_ent_device *
dcmipp_statcap_ent_init(struct device *dev, const char *entity_name,
			struct v4l2_device *v4l2_dev, void __iomem *regs)
{
	struct dcmipp_statcap_device *vcap;
	struct video_device *vdev;
	struct vb2_queue *q;
	int i, ret = 0;

	/* Allocate the dcmipp_cap_device struct */
	vcap = kzalloc(sizeof(*vcap), GFP_KERNEL);
	if (!vcap)
		return ERR_PTR(-ENOMEM);

	/* Allocate the pad */
	vcap->ved.pads =
		dcmipp_pads_init(1, (const unsigned long[1]) {MEDIA_PAD_FL_SINK});
	if (IS_ERR(vcap->ved.pads)) {
		ret = PTR_ERR(vcap->ved.pads);
		goto err_free_vcap;
	}

	/* Initialize the media entity */
	vcap->vdev.entity.name = entity_name;
	vcap->vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	ret = media_entity_pads_init(&vcap->vdev.entity,
				     1, vcap->ved.pads);
	if (ret)
		goto err_clean_pads;

	/* Initialize the lock */
	mutex_init(&vcap->lock);

	/* Initialize the vb2 queue */
	q = &vcap->queue;
	q->type = V4L2_BUF_TYPE_META_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_READ | VB2_DMABUF;
	q->lock = &vcap->lock;
	q->drv_priv = vcap;
	q->buf_struct_size = sizeof(struct dcmipp_buf);
	q->ops = &dcmipp_statcap_qops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 1;
	q->dev = dev;

	ret = vb2_queue_init(q);
	if (ret) {
		dev_err(dev, "%s: vb2 queue init failed (err=%d)\n",
			entity_name, ret);
		goto err_clean_m_ent;
	}

	/* Initialize buffer list and its lock */
	INIT_LIST_HEAD(&vcap->buffers);
	spin_lock_init(&vcap->irqlock);

	/* Fill the dcmipp_ent_device struct */
	vcap->ved.ent = &vcap->vdev.entity;
	vcap->ved.handler = NULL;
	vcap->ved.thread_fn = dcmipp_statcap_irq_thread;
	vcap->dev = dev;
	vcap->regs = regs;

	/* Initialize the video_device struct */
	vdev = &vcap->vdev;
	vdev->device_caps = V4L2_CAP_META_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
	vdev->entity.ops = &dcmipp_statcap_mops;
	vdev->release = dcmipp_statcap_release;
	vdev->fops = &dcmipp_statcap_fops;
	vdev->ioctl_ops = &dcmipp_statcap_ioctl_ops;
	vdev->lock = &vcap->lock;
	vdev->queue = q;
	vdev->v4l2_dev = v4l2_dev;
	strscpy(vdev->name, entity_name, sizeof(vdev->name));
	video_set_drvdata(vdev, &vcap->ved);

	/* Add controls */
	v4l2_ctrl_handler_init(&vcap->ctrls, ARRAY_SIZE(dcmipp_statcap_ctrls));

	for (i = 0; i < ARRAY_SIZE(dcmipp_statcap_ctrls); i++) {
		v4l2_ctrl_new_custom(&vcap->ctrls, &dcmipp_statcap_ctrls[i], NULL);
		ret = vcap->ctrls.error;
		if (ret < 0) {
			dev_err(vcap->dev, "Control initialization error %d\n",	ret);
			goto err_clean_ctrl_hdl;
		}
	}
	vcap->vdev.ctrl_handler = &vcap->ctrls;

	/* Register the video_device with the v4l2 and the media framework */
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(dev, "%s: video register failed (err=%d)\n",
			vcap->vdev.name, ret);
		goto err_clean_ctrl_hdl;
	}

	return &vcap->ved;

err_clean_ctrl_hdl:
	v4l2_ctrl_handler_free(&vcap->ctrls);
err_clean_m_ent:
	media_entity_cleanup(&vcap->vdev.entity);
err_clean_pads:
	dcmipp_pads_cleanup(vcap->ved.pads);
err_free_vcap:
	kfree(vcap);

	return ERR_PTR(ret);
}
