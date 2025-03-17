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

#include <linux/iopoll.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include <uapi/linux/stm32-dcmipp-config.h>

#include "dcmipp-common.h"

#define DCMIPP_CMIER		0x3f0
#define DCMIPP_CMIER_HSFRAMEF	BIT(20)
#define DCMIPP_CMIER_HSBCF	BIT(21)
#define DCMIPP_CMIER_HSOVRF	BIT(22)

#define DCMIPP_CMFCR		0x3fc

#define DCMIPP_CMSR2_P1VSYNCF	BIT(18)
#define DCMIPP_CMSR2_HSFRAMEF	BIT(20)
#define DCMIPP_CMSR2_HSBCF	BIT(21)
#define DCMIPP_CMSR2_HSOVRF	BIT(22)
#define DCMIPP_CMSR2_P2VSYNCF	BIT(26)

#define DCMIPP_P1BPRSR			0x828
#define DCMIPP_P1BPRSR_BADCNT_MASK	GENMASK(11, 0)

#define DCMIPP_P1STXCR(a)		(0x850 + ((a) * 0x4))
#define DCMIPP_P1STXCR_ENABLE		BIT(0)
#define DCMIPP_P1STXCR_BINS_SHIFT	2
#define DCMIPP_P1STXCR_SRC_COMP_SHIFT	4
#define DCMIPP_P1STXCR_SRC_LOC_PRE	0
#define DCMIPP_P1STXCR_SRC_LOC_POST	1
#define DCMIPP_P1STXCR_SRC_LOC_SHIFT	6
#define DCMIPP_P1STXCR_MODE_AVERAGE	0
#define DCMIPP_P1STXCR_MODE_BINS	BIT(7)

#define DCMIPP_P1STSTR			0x85c
#define DCMIPP_P1STSTR_HSTART_SHIFT	0
#define DCMIPP_P1STSTR_HSTART_MASK	GENMASK(11, 0)
#define DCMIPP_P1STSTR_VSTART_SHIFT	16
#define DCMIPP_P1STSTR_VSTART_MASK	GENMASK(27, 16)

#define DCMIPP_P1STSZR			0x860
#define DCMIPP_P1STSZR_HSIZE_SHIFT	0
#define DCMIPP_P1STSZR_HSIZE_MASK	GENMASK(11, 0)
#define DCMIPP_P1STSZR_VSIZE_SHIFT	16
#define DCMIPP_P1STSZR_VSIZE_MASK	GENMASK(27, 16)
#define DCMIPP_P1STSZR_ENABLE		BIT(31)

#define DCMIPP_P1STXSR(a)		(0x864 + ((a) * 0x4))

#define DCMIPP_NB_STAT_REGION	1

/* Histogram block - only available starting from stm32mp21 */
#define DCMIPP_P1HSCR			0x8b0
#define DCMIPP_P1HSCR_ENABLE		BIT(0)

#define DCMIPP_P1HSSTR			0x8b4
#define DCMIPP_P1HSSTR_START(x, y)	((x) | ((y) << 16))

#define DCMIPP_P1HSSZR			0x8b8
#define DCMIPP_P1HSSZR_SIZE(w, h)	((w) | ((h) << 16))

#define DCMIPP_P1HSMAR1			0x8bc

struct dcmipp_buf {
	struct vb2_v4l2_buffer	vb;
	bool			prepared;
	dma_addr_t		paddr;
	size_t			size;
	struct list_head	list;
};

/* This structure describe the state right after the VSYNC comes */
enum stat_capture_state {
	COLD_START,		/* Shadow: AVERAGE (RGB), Physical: stopped */
	/* Full capture profile */
	PHY_AV_RGB_SHA_BIN_0,	/* Shadow: BIN_0, Physical: AVERAGE (RGB) */
	PHY_BIN_0_SHA_BIN_1,	/* Shadow: BIN_1, Physical: BIN_0 */
	PHY_BIN_1_SHA_BIN_2,	/* Shadow: BIN_2, Physical: BIN_1 */
	PHY_BIN_2_SHA_BIN_3,	/* Shadow: BIN_3, Physical: BIN_2 */
	PHY_BIN_3_SHA_AV_RGB,	/* Shadow: AVERAGE (RGB), Physical: BIN_3 */
	/* Average pre-post profile */
	PHY_AV_RGB,	/* Shadow: AVERAGE (RGB), Physical: AVERAGE (RGB) */
	AV_READ,	/* Capturing AVERAGE / Accumulators with valid AVERAGE */
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
	/*
	 * The buffers list contains buffers ready for local_buf copy.
	 * This is the only list used when histogram are not used.
	 */
	struct list_head buffers;
	/*
	 * The queue_buffers list contains buffers which are empty of both histogram
	 * and local_buf copy.  This list is only used when histogram are being used.
	 */
	struct list_head queued_buffers;
	/* Protects the access of variables shared within the interrupt */
	spinlock_t irqlock;
	/* Protect this data structure */
	struct mutex lock;
	struct v4l2_subdev *s_subdev;
	struct media_pad *s_pad;
	u32 sequence;
	u32 frame_format;
	struct v4l2_rect stat_region;
	enum v4l2_isp_stat_avg_filter avg_filter;
	enum v4l2_isp_stat_bin_comp bin_comp;
	enum v4l2_isp_stat_profile stat_profile;
	u32 stat_location;
	bool stat_ready;

	enum dcmipp_state state;

	/*
	 * DCMIPP driver is handling 2 buffers
	 * active: buffer into which DCMIPP is currently writing into
	 * next: buffer given to the DCMIPP and which will become
	 *       automatically active on next VSYNC
	 */
	struct dcmipp_buf *active, *next;

	u32 cmier;
	u32 cmsr2;

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

	/* Local copy of statistics */
	struct stm32_dcmipp_stat_avr_bins stat_pre;
	struct stm32_dcmipp_stat_avr_bins stat_post;
	u32 bad_pixel_count;
};

static int dcmipp_statcap_get_subdev_pad(struct dcmipp_statcap_device *vcap,
					 struct v4l2_subdev **s_subdev,
					 struct media_pad **s_pad)
{
	struct media_pad *pad;

	if (!vcap->s_subdev || !vcap->s_pad) {
		pad = media_pad_remote_pad_first(&vcap->vdev.entity.pads[0]);
		if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
			return -EINVAL;
		vcap->s_subdev = media_entity_to_v4l2_subdev(pad->entity);
		vcap->s_pad = pad;
	}

	if (s_subdev)
		*s_subdev = vcap->s_subdev;
	if (s_pad)
		*s_pad = vcap->s_pad;

	return 0;
}

static int dcmipp_statcap_querycap(struct file *file, void *priv,
				   struct v4l2_capability *cap)
{
	strscpy(cap->driver, DCMIPP_PDEV_NAME, sizeof(cap->driver));
	strscpy(cap->card, KBUILD_MODNAME, sizeof(cap->card));

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

static const struct v4l2_file_operations dcmipp_statcap_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
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

static int dcmipp_statcap_start_streaming(struct vb2_queue *vq,
					  unsigned int count)
{
	struct dcmipp_statcap_device *vcap = vb2_get_drv_priv(vq);
	struct media_entity *entity = &vcap->vdev.entity;
	struct list_head *buffer_list;
	struct dcmipp_buf *buf, *node;
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev_selection sel;
	struct v4l2_subdev *s_subdev;
	struct media_pad *s_pad;
	int ret;

	vcap->sequence = 0;

	if (!vcap->ved.dcmipp->pipe_cfg->has_histo)
		buffer_list = &vcap->buffers;
	else
		buffer_list = &vcap->queued_buffers;

	ret = pm_runtime_resume_and_get(vcap->dev);
	if (ret < 0) {
		dev_err(vcap->dev, "%s: Failed to start streaming, cannot get sync (%d)\n",
			__func__, ret);
		goto err_buffer_done;
	}

	ret = media_pipeline_start(entity->pads, &vcap->ved.dcmipp->pipe);
	if (ret) {
		dev_dbg(vcap->dev, "%s: Failed to start streaming, media pipeline start error (%d)\n",
			__func__, ret);
		goto err_pm_put;
	}

	/* Get pointer to the source subdev */
	ret = dcmipp_statcap_get_subdev_pad(vcap, &s_subdev, &s_pad);
	if (ret)
		goto err_media_pipeline_stop;

	/* Start all the elements within pipeline */
	ret = v4l2_subdev_enable_streams(s_subdev, s_pad->index, BIT_ULL(0));
	if (ret) {
		dev_err(vcap->dev, "failed to start streaming (%d)\n", ret);
		goto err_media_pipeline_stop;
	}

	/* Get frame format info from ISP sink pad */
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.pad = 0;
	ret = v4l2_subdev_call(s_subdev, pad, get_fmt, NULL, &fmt);
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

	if (!vcap->ved.dcmipp->pipe_cfg->has_histo)
		return 0;

	/* Get compose size from the ISP sink pad */
	sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sel.pad = 0;
	sel.target = V4L2_SEL_TGT_COMPOSE;
	ret = v4l2_subdev_call(s_subdev, pad, get_selection, NULL, &sel);
	if (ret < 0) {
		dev_err(vcap->dev, "%s: Failed to start streaming, can't get compose (%d)\n",
			__func__, ret);
		goto err_media_pipeline_stop;
	}

	spin_lock_irq(&vcap->irqlock);

	/*
	 * vb2 framework guarantee that we have at least 'min_buffers_needed'
	 * buffers in the list at this moment
	 */
	vcap->next = list_first_entry(buffer_list, typeof(*buf), list);
	dev_dbg(vcap->dev, "Start with next [%d] %p phy=%pad\n",
		vcap->next->vb.vb2_buf.index, vcap->next, &vcap->next->paddr);

	reg_write(vcap, DCMIPP_P1HSMAR1, vcap->next->paddr);
	reg_set(vcap, DCMIPP_P1HSCR, DCMIPP_P1HSCR_ENABLE);

	/* Enable interruptions */
	vcap->cmier |= DCMIPP_CMIER_HSFRAMEF | DCMIPP_CMIER_HSBCF | DCMIPP_CMIER_HSOVRF;
	spin_lock(&vcap->vdev.v4l2_dev->lock);
	reg_set(vcap, DCMIPP_CMIER, vcap->cmier);
	spin_unlock(&vcap->vdev.v4l2_dev->lock);

	vcap->state = DCMIPP_RUNNING;

	spin_unlock_irq(&vcap->irqlock);

	return 0;

err_media_pipeline_stop:
	media_pipeline_stop(entity->pads);
err_pm_put:
	pm_runtime_put(vcap->dev);
err_buffer_done:
	spin_lock_irq(&vcap->irqlock);
	/*
	 * Return all buffers to vb2 in QUEUED state.
	 * This will give ownership back to userspace
	 */
	list_for_each_entry_safe(buf, node, buffer_list, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
	}
	vcap->active = NULL;
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
	struct v4l2_subdev *s_subdev;
	struct media_pad *s_pad;
	int ret;

	/* Reset the capture state machine */
	vcap->capture_state = COLD_START;

	ret = dcmipp_statcap_get_subdev_pad(vcap, &s_subdev, &s_pad);
	if (ret) {
		dev_warn(vcap->dev, "Failed to get source subdev / pad\n");
		return;
	}

	ret = v4l2_subdev_disable_streams(s_subdev, s_pad->index, BIT_ULL(0));
	if (ret)
		dev_err(vcap->dev, "failed to stop streaming (%d)\n", ret);

	/* Stop the media pipeline */
	media_pipeline_stop(vcap->vdev.entity.pads);

	/* Stop histogram capture */
	reg_clear(vcap, DCMIPP_P1HSCR, DCMIPP_P1HSCR_ENABLE);

	/* Disable interruptions */
	spin_lock(&vcap->vdev.v4l2_dev->lock);
	reg_clear(vcap, DCMIPP_CMIER, vcap->cmier);
	vcap->cmier = 0;
	spin_unlock(&vcap->vdev.v4l2_dev->lock);

	spin_lock_irq(&vcap->irqlock);

	/* Return all queued buffers to vb2 in ERROR state */
	list_for_each_entry_safe(buf, node, &vcap->buffers, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	INIT_LIST_HEAD(&vcap->buffers);
	if (vcap->ved.dcmipp->pipe_cfg->has_histo) {
		list_for_each_entry_safe(buf, node, &vcap->queued_buffers, list) {
			list_del_init(&buf->list);
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		}
		INIT_LIST_HEAD(&vcap->queued_buffers);
	}

	vcap->active = NULL;
	vcap->state = DCMIPP_STOPPED;

	spin_unlock_irq(&vcap->irqlock);

	pm_runtime_put(vcap->dev);
}

static int dcmipp_statcap_buf_prepare(struct vb2_buffer *vb)
{
	struct dcmipp_statcap_device *vcap =  vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct dcmipp_buf *buf = container_of(vbuf, struct dcmipp_buf, vb);
	unsigned long size = sizeof(struct stm32_dcmipp_stat_buf);
	dma_addr_t buf_head;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(vcap->dev, "%s data will not fit into plane (%lu < %lu)\n",
			__func__, vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	if (!buf->prepared) {
		buf_head = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
		buf->paddr = buf_head + offsetof(struct stm32_dcmipp_stat_buf, histograms);
		buf->size = vb2_plane_size(&buf->vb.vb2_buf, 0);

		buf->prepared = true;

		dev_dbg(vcap->dev, "buffer[%d] buf_head=%pad, size=%zu, histo=%pad\n",
			vb->index, &buf_head, buf->size, &buf->paddr);
	}

	return 0;
}

static void dcmipp_statcap_buf_queue(struct vb2_buffer *vb2_buf)
{
	struct dcmipp_statcap_device *vcap = vb2_get_drv_priv(vb2_buf->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb2_buf);
	struct dcmipp_buf *buf = container_of(vbuf, struct dcmipp_buf, vb);

	dev_dbg(vcap->dev, "Queue [%d] %p phy=%pad\n", buf->vb.vb2_buf.index,
		buf, &buf->paddr);

	spin_lock_irq(&vcap->irqlock);

	if (vcap->ved.dcmipp->pipe_cfg->has_histo)
		list_add_tail(&buf->list, &vcap->queued_buffers);
	else
		list_add_tail(&buf->list, &vcap->buffers);

	if (vcap->state == DCMIPP_WAIT_FOR_BUFFER) {
		vcap->next = buf;
		dev_dbg(vcap->dev, "Restart with next [%d] %p phy=%pad\n",
			buf->vb.vb2_buf.index, buf, &buf->paddr);

		reg_write(vcap, DCMIPP_P1HSMAR1, vcap->next->paddr);
		reg_set(vcap, DCMIPP_P1HSCR, DCMIPP_P1HSCR_ENABLE);

		vcap->state = DCMIPP_RUNNING;

		spin_unlock_irq(&vcap->irqlock);
		return;
	}

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

static int dcmipp_statcap_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dcmipp_statcap_device *vcap =
			container_of(ctrl->handler, struct dcmipp_statcap_device, ctrls);
	struct v4l2_ctrl_isp_stat_region *region;
	struct v4l2_subdev_selection sel;
	struct v4l2_subdev *s_subdev;
	int ret = 0;

	dev_dbg(vcap->dev, ">> %s: ctrl->id = 0x%x\n", __func__, ctrl->id);

	/* Get pointer to the source subdev (if case of not yet set */
	ret = dcmipp_statcap_get_subdev_pad(vcap, &s_subdev, NULL);
	if (ret)
		return ret;

	if (pm_runtime_get_if_in_use(vcap->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ISP_STAT_REGION:
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
		ret = v4l2_subdev_call(s_subdev, pad, get_selection, NULL, &sel);
		if (ret < 0) {
			dev_err(vcap->dev, "Failed to get frame size\n");
			goto out;
		}

		if (!region->nb_regions ||
		    (!region->width[0] && !region->height[0] &&
		     !region->left[0] && !region->top[0])) {
			spin_lock_irq(&vcap->irqlock);
			vcap->stat_region.width = sel.r.width;
			vcap->stat_region.height = sel.r.height;
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

		vcap->capture_state = COLD_START;
		spin_unlock_irq(&vcap->irqlock);
		break;
	case V4L2_CID_ISP_STAT_AVG_FILTER:
		spin_lock_irq(&vcap->irqlock);
		vcap->avg_filter = ctrl->val;
		vcap->capture_state = COLD_START;
		spin_unlock_irq(&vcap->irqlock);
		break;
	case V4L2_CID_ISP_STAT_BIN_COMP:
		spin_lock_irq(&vcap->irqlock);
		vcap->bin_comp = ctrl->val;
		vcap->capture_state = COLD_START;
		spin_unlock_irq(&vcap->irqlock);
		break;
	case V4L2_CID_ISP_STAT_PROFILE:
		spin_lock_irq(&vcap->irqlock);
		vcap->stat_profile = ctrl->val;
		vcap->capture_state = COLD_START;
		spin_unlock_irq(&vcap->irqlock);
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
		.def	= V4L2_STAT_BIN_COMP_L,
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
	mutex_destroy(&vcap->lock);

	kfree(vcap);
}

void dcmipp_statcap_ent_release(struct dcmipp_ent_device *ved)
{
	struct dcmipp_statcap_device *vcap =
		container_of(ved, struct dcmipp_statcap_device, ved);

	media_entity_cleanup(ved->ent);
	vb2_video_unregister_device(&vcap->vdev);
}

static irqreturn_t dcmipp_statcap_irq_callback(int irq, void *arg)
{
	struct dcmipp_statcap_device *vcap =
			container_of(arg, struct dcmipp_statcap_device, ved);
	struct dcmipp_ent_device *ved = arg;

	vcap->cmsr2 = ved->cmsr2 & (vcap->cmier | DCMIPP_CMSR2_P1VSYNCF | DCMIPP_CMSR2_P2VSYNCF);
	if (!vcap->cmsr2)
		return IRQ_HANDLED;

	/* Clear interrupt */
	reg_write(vcap, DCMIPP_CMFCR, ved->cmsr2 & vcap->cmier);

	return IRQ_WAKE_THREAD;
}

static void dcmipp_statcap_buffer_done(struct dcmipp_statcap_device *vcap,
				       struct dcmipp_buf *dcmipp_buf)
{
	struct stm32_dcmipp_stat_buf *stat_buf;

	stat_buf = (struct stm32_dcmipp_stat_buf *)vb2_plane_vaddr(&dcmipp_buf->vb.vb2_buf, 0);
	stat_buf->pre = vcap->stat_pre;
	stat_buf->post = vcap->stat_post;
	stat_buf->bad_pixel_count = vcap->bad_pixel_count;

	/* Send buffer */
	vb2_set_plane_payload(&dcmipp_buf->vb.vb2_buf, 0,
			      sizeof(struct stm32_dcmipp_stat_buf));
	dcmipp_buf->vb.sequence = vcap->sequence++;
	dcmipp_buf->vb.vb2_buf.timestamp = ktime_get_ns();
	vb2_buffer_done(&dcmipp_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

static u32 dcmipp_statcap_get_src(u32 location,
				  enum v4l2_isp_stat_bin_comp comp)
{
	return (location << DCMIPP_P1STXCR_SRC_LOC_SHIFT) | (comp << DCMIPP_P1STXCR_SRC_COMP_SHIFT);
}

static void dcmipp_statcap_read_avg_stats(struct dcmipp_statcap_device *vcap)
{
	struct stm32_dcmipp_stat_avr_bins *avr_bins =
		vcap->stat_location == DCMIPP_P1STXCR_SRC_LOC_PRE ? &vcap->stat_pre :
								    &vcap->stat_post;
	int i;

	for (i = 0; i < ARRAY_SIZE(vcap->stat_pre.average_RGB); i++) {
		avr_bins->average_RGB[i] = reg_read(vcap, DCMIPP_P1STXSR(i));
		/* Normalize values */
		avr_bins->average_RGB[i] <<= 8;

		/* Depending on the position & component, need to adjust in case of Bayer */
		if (vcap->stat_location == DCMIPP_P1STXCR_SRC_LOC_PRE &&
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

static void dcmipp_statcap_update_local_buf(struct dcmipp_statcap_device *vcap)
{
	struct stm32_dcmipp_stat_avr_bins *avr_bins =
		vcap->stat_location == DCMIPP_P1STXCR_SRC_LOC_PRE ? &vcap->stat_pre :
								    &vcap->stat_post;
	int i;

	/* Read the bad pixel count stat and store it locally */
	vcap->bad_pixel_count = reg_read(vcap, DCMIPP_P1BPRSR) &
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
		vcap->stat_ready = false;
		memset(&vcap->stat_pre, 0, sizeof(vcap->stat_pre));
		memset(&vcap->stat_post, 0, sizeof(vcap->stat_post));

		/*
		 * All stats profile starts from the PRE statistics, except the
		 * AVERAGE POST
		 */
		if (vcap->stat_profile == V4L2_STAT_PROFILE_AVERAGE_POST)
			vcap->stat_location = DCMIPP_P1STXCR_SRC_LOC_POST;
		else
			vcap->stat_location = DCMIPP_P1STXCR_SRC_LOC_PRE;
		/*
		 * We've just started, set control registers to capture
		 * AVERAGES (RGB) and leave
		 */
		for (i = 0; i < 3; i++)
			reg_write(vcap, DCMIPP_P1STXCR(i),
				  DCMIPP_P1STXCR_MODE_AVERAGE |
				  dcmipp_statcap_get_src(vcap->stat_location, i) |
				  vcap->avg_filter << DCMIPP_P1STXCR_BINS_SHIFT |
				  DCMIPP_P1STXCR_ENABLE);
		break;

	case PHY_AV_RGB_SHA_BIN_0:
		/* Set control registers to capture the 1st set of BINS */
		for (i = 0; i < 3; i++)
			reg_write(vcap, DCMIPP_P1STXCR(i),
				  DCMIPP_P1STXCR_MODE_BINS |
				  dcmipp_statcap_get_src(vcap->stat_location, vcap->bin_comp) |
				  0 << DCMIPP_P1STXCR_BINS_SHIFT |
				  DCMIPP_P1STXCR_ENABLE);

		if (vcap->prev_capture_state == PHY_BIN_3_SHA_AV_RGB) {
			/* The data capture refer to the previous location */
			avr_bins = !(vcap->stat_location == DCMIPP_P1STXCR_SRC_LOC_PRE) ?
					&vcap->stat_pre : &vcap->stat_post;
			/* Accumulators contains the 4th set of BINS */
			for (i = 0; i < 3; i++)
				avr_bins->bins[i + 9] = reg_read(vcap, DCMIPP_P1STXSR(i));
			/* By the time we get the 4th POST BINS, stat_location is again in PRE */
			if (vcap->stat_location == DCMIPP_P1STXCR_SRC_LOC_PRE)
				vcap->stat_ready = true;
		}
		break;

	case PHY_BIN_0_SHA_BIN_1:
		/* Set control registers to capture the 2nd set of BINS */
		for (i = 0; i < 3; i++)
			reg_write(vcap, DCMIPP_P1STXCR(i),
				  DCMIPP_P1STXCR_MODE_BINS |
				  dcmipp_statcap_get_src(vcap->stat_location, vcap->bin_comp) |
				  1 << DCMIPP_P1STXCR_BINS_SHIFT |
				  DCMIPP_P1STXCR_ENABLE);

		/* Accumulators contains the AVERAGES (RGB) */
		dcmipp_statcap_read_avg_stats(vcap);
		break;

	case PHY_BIN_1_SHA_BIN_2:
		/* Set control registers to capture the 3rd set of BINS */
		for (i = 0; i < 3; i++)
			reg_write(vcap, DCMIPP_P1STXCR(i),
				  DCMIPP_P1STXCR_MODE_BINS |
				  dcmipp_statcap_get_src(vcap->stat_location, vcap->bin_comp) |
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
				  dcmipp_statcap_get_src(vcap->stat_location, vcap->bin_comp) |
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
				  dcmipp_statcap_get_src(!vcap->stat_location, i) |
				  vcap->avg_filter << DCMIPP_P1STXCR_BINS_SHIFT |
				  DCMIPP_P1STXCR_ENABLE);

		/* Accumulators contains the 3rd set of BINS */
		for (i = 0; i < 3; i++)
			avr_bins->bins[i + 6] = reg_read(vcap, DCMIPP_P1STXSR(i));
		break;

	case AV_READ:
		/* State used for the AVERAGE PRE capture mode */
		dcmipp_statcap_read_avg_stats(vcap);
		vcap->stat_ready = true;
		break;

	default:
		break;
	}

	/* Update the capture_state & prev_capture_state */
	switch (vcap->stat_profile) {
	case V4L2_STAT_PROFILE_FULL:
		vcap->prev_capture_state = vcap->capture_state;
		if (vcap->capture_state < PHY_BIN_3_SHA_AV_RGB) {
			vcap->capture_state++;
		} else {
			vcap->stat_location = !vcap->stat_location;
			vcap->capture_state = PHY_AV_RGB_SHA_BIN_0;
		}
		break;

	case V4L2_STAT_PROFILE_AVERAGE_PRE:
	case V4L2_STAT_PROFILE_AVERAGE_POST:
		if (vcap->capture_state == COLD_START)
			vcap->capture_state = PHY_AV_RGB;
		else if (vcap->capture_state == PHY_AV_RGB)
			vcap->capture_state = AV_READ;
		break;
	}
}

/* irqlock must be held */
static void
dcmipp_statcap_set_next_buf_or_stop(struct dcmipp_statcap_device *vcap)
{
	if (!vcap->next && list_is_singular(&vcap->queued_buffers)) {
		/*
		 * If there is no available buffer (none or a single one in the list while two
		 * are expected), stop the capture (effective for next frame). On-going histo
		 * capture will continue till FRAME END but no further capture will be done.
		 */
		reg_clear(vcap, DCMIPP_P1HSCR, DCMIPP_P1HSCR_ENABLE);

		dev_dbg(vcap->dev,
			"Histogram capture restart deferred to next buffer queueing\n");
		vcap->state = DCMIPP_WAIT_FOR_BUFFER;
		return;
	}

	/* If we don't have buffer yet, pick the one after active */
	if (!vcap->next)
		vcap->next = list_next_entry(vcap->active, list);

	/*
	 * Set histogram address
	 * The register is shadowed and will be taken into
	 * account on next VSYNC (start of next frame)
	 */
	reg_write(vcap, DCMIPP_P1HSMAR1, vcap->next->paddr);
	dev_dbg(vcap->dev, "Write [%d] %p phy=%pad\n",
		vcap->next->vb.vb2_buf.index, vcap->next, &vcap->next->paddr);
}

static irqreturn_t dcmipp_statcap_irq_thread(int irq, void *arg)
{
	struct dcmipp_statcap_device *vcap =
			container_of(arg, struct dcmipp_statcap_device, ved);
	struct dcmipp_buf *cur_buf = NULL;

	/* We only to do things if we are streaming */
	if (!vb2_is_streaming(&vcap->queue))
		return IRQ_HANDLED;

	spin_lock_irq(&vcap->irqlock);

	if (vcap->cmsr2 & (DCMIPP_CMSR2_P1VSYNCF | DCMIPP_CMSR2_P2VSYNCF)) {
		dcmipp_statcap_update_local_buf(vcap);

		/* If a full capture cycle has been done, output data to a buffer */
		if (vcap->stat_ready) {
			/* Get an available buffer */
			if (!list_empty(&vcap->buffers)) {
				cur_buf = list_first_entry(&vcap->buffers, struct dcmipp_buf, list);
				list_del(&cur_buf->list);
			}
			if (cur_buf)
				dcmipp_statcap_buffer_done(vcap, cur_buf);
		}

		if (vcap->state == DCMIPP_RUNNING) {
			swap(vcap->active, vcap->next);
			dcmipp_statcap_set_next_buf_or_stop(vcap);
		}
	}

	if (vcap->cmsr2 & DCMIPP_CMIER_HSBCF) {
		/* If bad config is received, skip the HSFRAMEF is existing */
		dev_err(vcap->dev, "Received Histogram bad configuration error\n");
		if (vcap->active) {
			list_del(&vcap->active->list);
			vb2_buffer_done(&vcap->active->vb.vb2_buf, VB2_BUF_STATE_ERROR);
			vcap->active = NULL;
		}
		spin_unlock_irq(&vcap->irqlock);
		return IRQ_HANDLED;
	}

	if (vcap->cmsr2 & DCMIPP_CMIER_HSOVRF) {
		/* If bad config is received, skip the HSFRAMEF is existing */
		dev_err(vcap->dev, "Received Histogram overrun error\n");
		if (vcap->active) {
			list_del(&vcap->active->list);
			vb2_buffer_done(&vcap->active->vb.vb2_buf, VB2_BUF_STATE_ERROR);
			vcap->active = NULL;
		}
		spin_unlock_irq(&vcap->irqlock);
		return IRQ_HANDLED;
	}

	if (vcap->cmsr2 & DCMIPP_CMIER_HSFRAMEF) {
		if (vcap->active) {
			/* Remove from the queued_buffers and add into buffers */
			list_del_init(&vcap->active->list);
			list_add_tail(&vcap->active->list, &vcap->buffers);
			vcap->active = NULL;
		}
	}

	spin_unlock_irq(&vcap->irqlock);

	return IRQ_HANDLED;
}

struct dcmipp_ent_device *
dcmipp_statcap_ent_init(const char *entity_name, struct dcmipp_device *dcmipp)
{
	struct dcmipp_statcap_device *vcap;
	struct device *dev = dcmipp->dev;
	struct video_device *vdev;
	struct vb2_queue *q;
	const unsigned long pad_flag = MEDIA_PAD_FL_SINK;
	int i, ret = 0;

	/* Allocate the dcmipp_statcap_device struct */
	vcap = kzalloc(sizeof(*vcap), GFP_KERNEL);
	if (!vcap)
		return ERR_PTR(-ENOMEM);

	/* Allocate the pad */
	vcap->ved.pads = dcmipp_pads_init(1, &pad_flag);
	if (IS_ERR(vcap->ved.pads)) {
		ret = PTR_ERR(vcap->ved.pads);
		goto err_free_vcap;
	}

	vcap->ved.dcmipp = dcmipp;

	/* Initialize the media entity */
	vcap->vdev.entity.name = entity_name;
	vcap->vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	ret = media_entity_pads_init(&vcap->vdev.entity, 1, vcap->ved.pads);
	if (ret)
		goto err_clean_pads;

	/* Initialize the lock */
	mutex_init(&vcap->lock);

	/* Initialize the vb2 queue */
	q = &vcap->queue;
	q->type = V4L2_BUF_TYPE_META_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->lock = &vcap->lock;
	q->drv_priv = vcap;
	q->buf_struct_size = sizeof(struct dcmipp_buf);
	q->ops = &dcmipp_statcap_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 1;
	q->dev = dev;

	ret = vb2_queue_init(q);
	if (ret) {
		dev_err(dev, "%s: vb2 queue init failed (err=%d)\n",
			entity_name, ret);
		goto err_clean_m_ent;
	}

	/* Initialize buffer lists and lock */
	INIT_LIST_HEAD(&vcap->buffers);
	INIT_LIST_HEAD(&vcap->queued_buffers);
	spin_lock_init(&vcap->irqlock);

	/* Fill the dcmipp_ent_device struct */
	vcap->ved.ent = &vcap->vdev.entity;
	vcap->ved.handler = dcmipp_statcap_irq_callback;
	vcap->ved.thread_fn = dcmipp_statcap_irq_thread;
	vcap->dev = dev;
	vcap->regs = dcmipp->regs;

	/* Initialize the video_device struct */
	vdev = &vcap->vdev;
	vdev->device_caps = V4L2_CAP_META_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_IO_MC;
	vdev->release = dcmipp_statcap_release;
	vdev->fops = &dcmipp_statcap_fops;
	vdev->ioctl_ops = &dcmipp_statcap_ioctl_ops;
	vdev->lock = &vcap->lock;
	vdev->queue = q;
	vdev->v4l2_dev = &dcmipp->v4l2_dev;
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
