// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for STM32 Digital Camera Memory Interface Pixel Processor
 *
 * Copyright (C) STMicroelectronics SA 2021
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          Alain Volmat <alain.volmat@foss.st.com>
 *          for STMicroelectronics.
 */

#include <linux/component.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "dcmipp-common.h"

#define DCMIPP_BYTECAP_DRV_NAME "dcmipp-bytecap"

#define DCMIPP_PRSR (0x1F8)
#define DCMIPP_CMIER (0x3F0)
#define DCMIPP_CMIER_P0FRAMEIE BIT(9)
#define DCMIPP_CMIER_P0VSYNCIE BIT(10)
#define DCMIPP_CMIER_P0OVRIE BIT(15)
#define DCMIPP_CMIER_P0ALL (DCMIPP_CMIER_P0VSYNCIE |\
			    DCMIPP_CMIER_P0FRAMEIE |\
			    DCMIPP_CMIER_P0OVRIE)
#define DCMIPP_CMSR1 (0x3F4)
#define DCMIPP_CMSR2 (0x3F8)
#define DCMIPP_CMSR2_P0FRAMEF BIT(9)
#define DCMIPP_CMSR2_P0VSYNCF BIT(10)
#define DCMIPP_CMSR2_P0OVRF BIT(15)
#define DCMIPP_CMFCR (0x3FC)
#define DCMIPP_P0FSCR (0x404)
#define DCMIPP_P0FSCR_PIPEN BIT(31)
#define DCMIPP_P0FCTCR (0x500)
#define DCMIPP_P0FCTCR_CPTREQ BIT(3)
#define DCMIPP_P0DCCNTR (0x5B0)
#define DCMIPP_P0DCLMTR (0x5B4)
#define DCMIPP_P0DCLMTR_ENABLE BIT(31)
#define DCMIPP_P0DCLMTR_LIMIT_MASK GENMASK(23, 0)
#define DCMIPP_P0PPM0AR1 (0x5C4)
#define DCMIPP_P0SR (0x5F8)
#define DCMIPP_P0SR_CPTACT BIT(23)

struct dcmipp_bytecap_pix_map {
	unsigned int code;
	u32 pixelformat;
};

#define PIXMAP_MBUS_PFMT(mbus, fmt)				\
		{						\
			.code = MEDIA_BUS_FMT_##mbus,		\
			.pixelformat = V4L2_PIX_FMT_##fmt	\
		}

static const struct dcmipp_bytecap_pix_map dcmipp_bytecap_pix_map_list[] = {
	PIXMAP_MBUS_PFMT(RGB565_2X8_LE, RGB565),
	PIXMAP_MBUS_PFMT(YUYV8_2X8, YUYV),
	PIXMAP_MBUS_PFMT(YVYU8_2X8, YVYU),
	PIXMAP_MBUS_PFMT(UYVY8_2X8, UYVY),
	PIXMAP_MBUS_PFMT(VYUY8_2X8, VYUY),
	PIXMAP_MBUS_PFMT(Y8_1X8, GREY),
	PIXMAP_MBUS_PFMT(SBGGR8_1X8, SBGGR8),
	PIXMAP_MBUS_PFMT(SGBRG8_1X8, SGBRG8),
	PIXMAP_MBUS_PFMT(SGRBG8_1X8, SGRBG8),
	PIXMAP_MBUS_PFMT(SRGGB8_1X8, SRGGB8),
	PIXMAP_MBUS_PFMT(JPEG_1X8, JPEG),
};

static const struct dcmipp_bytecap_pix_map *dcmipp_bytecap_pix_map_by_pixelformat
						(u32 pixelformat)
{
	const struct dcmipp_bytecap_pix_map *l = dcmipp_bytecap_pix_map_list;
	unsigned int size = ARRAY_SIZE(dcmipp_bytecap_pix_map_list);
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (l[i].pixelformat == pixelformat)
			return &l[i];
	}

	return NULL;
}

static const struct dcmipp_bytecap_pix_map *dcmipp_bytecap_pix_map_by_index(unsigned int i)
{
	const struct dcmipp_bytecap_pix_map *l = dcmipp_bytecap_pix_map_list;
	unsigned int size = ARRAY_SIZE(dcmipp_bytecap_pix_map_list);

	if (i >= size)
		return NULL;

	return &l[i];
}

struct dcmipp_buf {
	struct vb2_v4l2_buffer	vb;
	bool			prepared;
	dma_addr_t		paddr;
	size_t			size;
	struct list_head	list;
};

enum state {
	STOPPED = 0,
	WAIT_FOR_BUFFER,
	RUNNING,
};

struct dcmipp_bytecap_device {
	struct dcmipp_ent_device ved;
	struct video_device vdev;
	struct device *dev;
	struct device *cdev;
	struct v4l2_pix_format format;
	struct vb2_queue queue;
	struct list_head buffers;
	/* Protects the access of variables shared within the interrupt */
	spinlock_t irqlock;
	/* Protect this data structure */
	struct mutex lock;
	u32 sequence;
	struct media_pipeline pipe;

	enum state state;

	/*
	 * DCMIPP driver is handling 2 buffers
	 * active: buffer into which DCMIPP is currently writing into
	 * next: buffer given to the DCMIPP and which will become
	 *       automatically active on next VSYNC
	 */
	struct dcmipp_buf *active, *next;

	void __iomem *regs;
	struct reset_control *rstc;

	u32 cmier;
	u32 cmsr2;

	int errors_count;
	int limit_count;
	int overrun_count;
	int buffers_count;
	int vsync_count;
	int frame_count;
	int it_count;
	int underrun_count;
	int nactive_count;
};

static const struct v4l2_pix_format fmt_default = {
	.width = DCMIPP_FMT_WIDTH_DEFAULT,
	.height = DCMIPP_FMT_HEIGHT_DEFAULT,
	.pixelformat = V4L2_PIX_FMT_RGB565,
	.field = V4L2_FIELD_NONE,
	.colorspace = DCMIPP_COLORSPACE_DEFAULT,
	.ycbcr_enc = DCMIPP_YCBCR_ENC_DEFAULT,
	.quantization = DCMIPP_QUANTIZATION_DEFAULT,
	.xfer_func = DCMIPP_XFER_FUNC_DEFAULT,
};

static inline int frame_size(u32 width, u32 height, u32 format)
{
	switch (format) {
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_GREY:
		return (width * height);
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		return (width * height * 2);
	case V4L2_PIX_FMT_JPEG:
		return (width * height);
	default:
		return 0;
	}
}

static inline int frame_stride(u32 width, u32 format)
{
	switch (format) {
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_JPEG:
		return width;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		return (width * 2);
	default:
		return 0;
	}
}

static inline int hdw_pixel_alignment(u32 format)
{
	/* 16 bytes alignment required by hardware */
	switch (format) {
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_JPEG:
		return 4;/* 2^4 = 16 pixels = 16 bytes */
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		return 3;/* 2^3  = 8 pixels = 16 bytes */
	default:
		return 0;
	}
}

static int dcmipp_bytecap_querycap(struct file *file, void *priv,
				   struct v4l2_capability *cap)
{
	strscpy(cap->driver, DCMIPP_PDEV_NAME, sizeof(cap->driver));
	strscpy(cap->card, KBUILD_MODNAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "platform:%s", DCMIPP_PDEV_NAME);

	return 0;
}

static void dcmipp_bytecap_get_format(struct dcmipp_ent_device *ved,
				      struct v4l2_pix_format *fmt)
{
	struct dcmipp_bytecap_device *vcap = container_of(ved, struct dcmipp_bytecap_device,
						    ved);

	*fmt = vcap->format;
}

static int dcmipp_bytecap_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct dcmipp_bytecap_device *vcap = video_drvdata(file);

	f->fmt.pix = vcap->format;

	return 0;
}

static int dcmipp_bytecap_try_fmt_vid_cap(struct file *file, void *priv,
					  struct v4l2_format *f)
{
	struct dcmipp_bytecap_device *vcap = video_drvdata(file);
	struct v4l2_pix_format *format = &f->fmt.pix;
	const struct dcmipp_bytecap_pix_map *vpix;
	u32 in_w, in_h;

	/* Don't accept a pixelformat that is not on the table */
	vpix = dcmipp_bytecap_pix_map_by_pixelformat(format->pixelformat);
	if (!vpix)
		format->pixelformat = fmt_default.pixelformat;

	/* Adjust width & height */
	in_w = format->width;
	in_h = format->height;
	v4l_bound_align_image(&format->width,
			      DCMIPP_FRAME_MIN_WIDTH, DCMIPP_FRAME_MAX_WIDTH,
			      hdw_pixel_alignment(format->pixelformat),
			      &format->height,
			      DCMIPP_FRAME_MIN_HEIGHT, DCMIPP_FRAME_MAX_HEIGHT,
			      hdw_pixel_alignment(format->pixelformat),
			      0);
	if (format->width != in_w || format->height != in_h)
		dev_dbg(vcap->dev,
			"resolution updated: %dx%d -> %dx%d\n",
			in_w, in_h, format->width, format->height);

	format->bytesperline = frame_stride(format->width, format->pixelformat);
	format->sizeimage = frame_size(format->width, format->height, format->pixelformat);

	if (format->field == V4L2_FIELD_ANY)
		format->field = fmt_default.field;

	dcmipp_colorimetry_clamp(format);

	return 0;
}

static int dcmipp_bytecap_s_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct dcmipp_bytecap_device *vcap = video_drvdata(file);
	int ret;

	/* Do not change the format while stream is on */
	if (vb2_is_busy(&vcap->queue))
		return -EBUSY;

	ret = dcmipp_bytecap_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	dev_dbg(vcap->dev, "%s: format update: old:%dx%d (0x%x, %d, %d, %d, %d) new:%dx%d (0x%x, %d, %d, %d, %d)\n",
		vcap->vdev.name,
		/* old */
		vcap->format.width, vcap->format.height,
		vcap->format.pixelformat, vcap->format.colorspace,
		vcap->format.quantization, vcap->format.xfer_func,
		vcap->format.ycbcr_enc,
		/* new */
		f->fmt.pix.width, f->fmt.pix.height,
		f->fmt.pix.pixelformat, f->fmt.pix.colorspace,
		f->fmt.pix.quantization, f->fmt.pix.xfer_func,
		f->fmt.pix.ycbcr_enc);

	vcap->format = f->fmt.pix;

	return 0;
}

static int dcmipp_bytecap_enum_fmt_vid_cap(struct file *file, void *priv,
					   struct v4l2_fmtdesc *f)
{
	const struct dcmipp_bytecap_pix_map *vpix = dcmipp_bytecap_pix_map_by_index(f->index);

	if (!vpix)
		return -EINVAL;

	f->pixelformat = vpix->pixelformat;

	return 0;
}

static int dcmipp_bytecap_enum_framesizes(struct file *file, void *fh,
					  struct v4l2_frmsizeenum *fsize)
{
	const struct dcmipp_bytecap_pix_map *vpix;

	if (fsize->index)
		return -EINVAL;

	/* Only accept code in the pix map table */
	vpix = dcmipp_bytecap_pix_map_by_pixelformat(fsize->pixel_format);
	if (!vpix)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = DCMIPP_FRAME_MIN_WIDTH;
	fsize->stepwise.max_width = DCMIPP_FRAME_MAX_WIDTH;
	fsize->stepwise.min_height = DCMIPP_FRAME_MIN_HEIGHT;
	fsize->stepwise.max_height = DCMIPP_FRAME_MAX_HEIGHT;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.step_height = 1;

	return 0;
}

/* TODO - based on the explanation text, should also use v4l2_pipeline_link_notify */
static int dcmipp_bytecap_open(struct file *file)
{
	struct dcmipp_bytecap_device *vcap = video_drvdata(file);
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

static int dcmipp_bytecap_close(struct file *file)
{
	struct dcmipp_bytecap_device *vcap = video_drvdata(file);

	vb2_fop_release(file);

	v4l2_pipeline_pm_put(&vcap->vdev.entity);

	return 0;
}

static const struct v4l2_file_operations dcmipp_bytecap_fops = {
	.owner		= THIS_MODULE,
	.open		= dcmipp_bytecap_open,
	.release	= dcmipp_bytecap_close,
	.read           = vb2_fop_read,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = vb2_fop_mmap,
};

static const struct v4l2_ioctl_ops dcmipp_bytecap_ioctl_ops = {
	.vidioc_querycap = dcmipp_bytecap_querycap,

	.vidioc_g_fmt_vid_cap = dcmipp_bytecap_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = dcmipp_bytecap_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = dcmipp_bytecap_try_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap = dcmipp_bytecap_enum_fmt_vid_cap,
	.vidioc_enum_framesizes = dcmipp_bytecap_enum_framesizes,

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

static int dcmipp_pipeline_s_stream(struct dcmipp_bytecap_device *vcap,
				    int state)
{
	struct media_entity *entity = &vcap->vdev.entity;
	struct v4l2_subdev *subdev;
	struct media_pad *pad;
	int ret;

	/* Start/stop all entities within pipeline */
	while (1) {
		pad = &entity->pads[0];
		if (!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		pad = media_entity_remote_pad(pad);
		if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
			break;

		entity = pad->entity;
		subdev = media_entity_to_v4l2_subdev(entity);

		ret = v4l2_subdev_call(subdev, video, s_stream, state);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			dev_err(vcap->dev, "%s: \"%s\" failed to %s streaming (%d)\n",
				__func__, subdev->name,
				state ? "start" : "stop", ret);

			if (!state)
				v4l2_subdev_call(subdev, core, s_power, state);

			return ret;
		}

		dev_dbg(vcap->dev, "\"%s\" is %s\n",
			subdev->name, state ? "started" : "stopped");
	}

	return 0;
}

static void dcmipp_start_capture(struct dcmipp_bytecap_device *vcap,
				 struct dcmipp_buf *buf)
{
	/* Set buffer address */
	reg_write(vcap, DCMIPP_P0PPM0AR1, buf->paddr);
	dev_dbg(vcap->dev, "Write [%d] %p phy=%pad\n", buf->vb.vb2_buf.index, buf, &buf->paddr);

	/* Set buffer size */
	reg_write(vcap, DCMIPP_P0DCLMTR, DCMIPP_P0DCLMTR_ENABLE |
		  ((buf->size / 4) & DCMIPP_P0DCLMTR_LIMIT_MASK));

	/* Capture request */
	reg_set(vcap, DCMIPP_P0FCTCR, DCMIPP_P0FCTCR_CPTREQ);
}

static int dcmipp_bytecap_start_streaming(struct vb2_queue *vq,
					  unsigned int count)
{
	struct dcmipp_bytecap_device *vcap = vb2_get_drv_priv(vq);
	struct media_entity *entity = &vcap->vdev.entity;
	struct dcmipp_buf *buf, *node;
	int ret;

	vcap->sequence = 0;
	vcap->errors_count = 0;
	vcap->limit_count = 0;
	vcap->overrun_count = 0;
	vcap->buffers_count = 0;
	vcap->vsync_count = 0;
	vcap->frame_count = 0;
	vcap->it_count = 0;
	vcap->underrun_count = 0;
	vcap->nactive_count = 0;

	ret = pm_runtime_get_sync(vcap->cdev);
	if (ret < 0) {
		dev_err(vcap->dev, "%s: Failed to start streaming, cannot get sync (%d)\n",
			__func__, ret);
		goto err_pm_put;
	}

	/* Start the media pipeline */
	ret = media_pipeline_start(entity, &vcap->pipe);
	if (ret) {
		dev_err(vcap->dev, "%s: Failed to start streaming, media pipeline start error (%d)\n",
			__func__, ret);
		goto err_pm_put;
	}

	/* Start all the elements within pipeline */
	ret = dcmipp_pipeline_s_stream(vcap, 1);
	if (ret)
		goto err_media_pipeline_stop;

	spin_lock_irq(&vcap->irqlock);

	/* Enable pipe at the end of programming */
	reg_set(vcap, DCMIPP_P0FSCR, DCMIPP_P0FSCR_PIPEN);

	/*
	 * Start capture if at least one buffer has been queued,
	 * otherwise start is deferred at next buffer queueing
	 */
	buf = list_first_entry_or_null(&vcap->buffers, typeof(*buf), list);
	if (!buf) {
		dev_dbg(vcap->dev, "Start streaming is deferred to next buffer queueing\n");
		vcap->next = NULL;
		vcap->state = WAIT_FOR_BUFFER;
		spin_unlock_irq(&vcap->irqlock);
		return 0;
	}
	vcap->next = buf;
	dev_dbg(vcap->dev, "Start with next [%d] %p phy=%pad\n",
		buf->vb.vb2_buf.index, buf, &buf->paddr);

	/* Start capture */
	dcmipp_start_capture(vcap, buf);

	/* Enable interruptions */
	vcap->cmier |= DCMIPP_CMIER_P0ALL;
	reg_set(vcap, DCMIPP_CMIER, vcap->cmier);

	vcap->state = RUNNING;

	spin_unlock_irq(&vcap->irqlock);

	return 0;

err_media_pipeline_stop:
	media_pipeline_stop(entity);
err_pm_put:
	pm_runtime_put(vcap->cdev);
	spin_lock_irq(&vcap->irqlock);
	/*
	 * Return all buffers to vb2 in QUEUED state.
	 * This will give ownership back to userspace
	 */
	list_for_each_entry_safe(buf, node, &vcap->buffers, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
	}
	vcap->active = NULL;
	spin_unlock_irq(&vcap->irqlock);

	return ret;
}

static void dcmipp_dump_status(struct dcmipp_bytecap_device *vcap)
{
	struct device *dev = vcap->dev;

	dev_dbg(dev, "[DCMIPP_PRSR]  =%#10.8x\n", reg_read(vcap, DCMIPP_PRSR));
	dev_dbg(dev, "[DCMIPP_P0SR] =%#10.8x\n", reg_read(vcap, DCMIPP_P0SR));
	dev_dbg(dev, "[DCMIPP_P0DCCNTR]=%#10.8x\n",
		reg_read(vcap, DCMIPP_P0DCCNTR));
	dev_dbg(dev, "[DCMIPP_CMSR1] =%#10.8x\n", reg_read(vcap, DCMIPP_CMSR1));
	dev_dbg(dev, "[DCMIPP_CMSR2] =%#10.8x\n", reg_read(vcap, DCMIPP_CMSR2));
}

/*
 * Stop the stream engine. Any remaining buffers in the stream queue are
 * dequeued and passed on to the vb2 framework marked as STATE_ERROR.
 */
static void dcmipp_bytecap_stop_streaming(struct vb2_queue *vq)
{
	struct dcmipp_bytecap_device *vcap = vb2_get_drv_priv(vq);
	struct dcmipp_buf *buf, *node;
	int ret;
	u32 status;

	dcmipp_pipeline_s_stream(vcap, 0);

	/* Stop the media pipeline */
	media_pipeline_stop(&vcap->vdev.entity);

	/* Disable interruptions */
	reg_clear(vcap, DCMIPP_CMIER, vcap->cmier);

	/* Stop capture */
	reg_clear(vcap, DCMIPP_P0FCTCR, DCMIPP_P0FCTCR_CPTREQ);

	/* Wait until CPTACT become 0 */
	ret = readl_relaxed_poll_timeout(vcap->regs + DCMIPP_P0SR,
					 status,
					 !(status & DCMIPP_P0SR_CPTACT),
					 20, 1000);
	if (ret)
		dev_warn(vcap->dev, "Timeout when stopping\n");

	/* Disable pipe */
	reg_clear(vcap, DCMIPP_P0FSCR, DCMIPP_P0FSCR_PIPEN);

	spin_lock_irq(&vcap->irqlock);

	/* Return all queued buffers to vb2 in ERROR state */
	list_for_each_entry_safe(buf, node, &vcap->buffers, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	INIT_LIST_HEAD(&vcap->buffers);

	vcap->active = NULL;
	vcap->state = STOPPED;

	dcmipp_dump_status(vcap);

	spin_unlock_irq(&vcap->irqlock);

	pm_runtime_put(vcap->cdev);

	if (ret) {
		/* Reset IP on timeout */
		if (reset_control_assert(vcap->rstc))
			dev_warn(vcap->dev, "Failed to assert the reset line\n");

		usleep_range(3000, 5000);

		if (reset_control_deassert(vcap->rstc))
			dev_warn(vcap->dev, "Failed to deassert the reset line\n");
	}

	if (vcap->errors_count)
		dev_warn(vcap->dev, "Some errors found while streaming: errors=%d (overrun=%d, limit=%d, nactive=%d), underrun=%d, buffers=%d\n",
			 vcap->errors_count, vcap->overrun_count, vcap->limit_count,
			 vcap->nactive_count, vcap->underrun_count, vcap->buffers_count);
	dev_dbg(vcap->dev, "Stop streaming, errors=%d (overrun=%d, limit=%d, nactive=%d), underrun=%d, vsync=%d, frame=%d, buffers=%d, it=%d\n",
		vcap->errors_count, vcap->overrun_count, vcap->limit_count,
		vcap->nactive_count, vcap->underrun_count, vcap->vsync_count,
		vcap->frame_count, vcap->buffers_count, vcap->it_count);
}

static int dcmipp_bytecap_buf_prepare(struct vb2_buffer *vb)
{
	struct dcmipp_bytecap_device *vcap =  vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct dcmipp_buf *buf = container_of(vbuf, struct dcmipp_buf, vb);
	unsigned long size;

	size = vcap->format.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(vcap->dev, "%s data will not fit into plane (%lu < %lu)\n",
			__func__, vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	if (!buf->prepared) {
		/* Get memory addresses */
		buf->paddr =
			vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
		buf->size = vb2_plane_size(&buf->vb.vb2_buf, 0);
		buf->prepared = true;

		vb2_set_plane_payload(&buf->vb.vb2_buf, 0, buf->size);

		dev_dbg(vcap->dev, "Setup [%d] phy=%pad size=%zu\n",
			vb->index, &buf->paddr, buf->size);
	}

	return 0;
}

static void dcmipp_bytecap_buf_queue(struct vb2_buffer *vb2_buf)
{
	struct dcmipp_bytecap_device *vcap =
		vb2_get_drv_priv(vb2_buf->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb2_buf);
	struct dcmipp_buf *buf = container_of(vbuf, struct dcmipp_buf, vb);

	spin_lock_irq(&vcap->irqlock);
	list_add_tail(&buf->list, &vcap->buffers);

	dev_dbg(vcap->dev, "Queue [%d] %p phy=%pad\n", buf->vb.vb2_buf.index, buf, &buf->paddr);

	if (vcap->state == WAIT_FOR_BUFFER) {
		vcap->next = buf;
		dev_dbg(vcap->dev, "Restart with next [%d] %p phy=%pad\n",
			buf->vb.vb2_buf.index, buf, &buf->paddr);

		dcmipp_start_capture(vcap, buf);

		vcap->state = RUNNING;

		spin_unlock_irq(&vcap->irqlock);

		return;
	}

	spin_unlock_irq(&vcap->irqlock);
}

static int dcmipp_bytecap_queue_setup(struct vb2_queue *vq,
				      unsigned int *nbuffers,
				      unsigned int *nplanes,
				      unsigned int sizes[],
				      struct device *alloc_devs[])
{
	struct dcmipp_bytecap_device *vcap = vb2_get_drv_priv(vq);
	unsigned int size;

	size = vcap->format.sizeimage;

	/* Make sure the image size is large enough */
	if (*nplanes)
		return sizes[0] < vcap->format.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = vcap->format.sizeimage;

	dev_dbg(vcap->dev, "Setup queue, count=%d, size=%d\n",
		*nbuffers, size);

	return 0;
}

static int dcmipp_bytecap_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct dcmipp_buf *buf = container_of(vbuf, struct dcmipp_buf, vb);

	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static const struct vb2_ops dcmipp_bytecap_qops = {
	.start_streaming	= dcmipp_bytecap_start_streaming,
	.stop_streaming		= dcmipp_bytecap_stop_streaming,
	.buf_init		= dcmipp_bytecap_buf_init,
	.buf_prepare		= dcmipp_bytecap_buf_prepare,
	.buf_queue		= dcmipp_bytecap_buf_queue,
	.queue_setup		= dcmipp_bytecap_queue_setup,
	/*
	 * Since q->lock is set we can use the standard
	 * vb2_ops_wait_prepare/finish helper functions.
	 */
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static const struct media_entity_operations dcmipp_bytecap_mops = {
	.link_validate		= dcmipp_link_validate,
};

static void dcmipp_bytecap_release(struct video_device *vdev)
{
	struct dcmipp_bytecap_device *vcap =
		container_of(vdev, struct dcmipp_bytecap_device, vdev);

	dcmipp_pads_cleanup(vcap->ved.pads);
	kfree(vcap);
}

static void dcmipp_bytecap_comp_unbind(struct device *comp,
				       struct device *master,
				       void *master_data)
{
	struct dcmipp_ent_device *ved = dev_get_drvdata(comp);
	struct dcmipp_bytecap_device *vcap =
		container_of(ved, struct dcmipp_bytecap_device, ved);

	media_entity_cleanup(ved->ent);
	vb2_video_unregister_device(&vcap->vdev);
}

static void dcmipp_buffer_done(struct dcmipp_bytecap_device *vcap,
			       struct dcmipp_buf *buf,
			       size_t bytesused,
			       int err)
{
	struct vb2_v4l2_buffer *vbuf;

	list_del_init(&buf->list);

	vbuf = &buf->vb;

	vbuf->sequence = vcap->sequence++;
	vbuf->field = V4L2_FIELD_NONE;
	vbuf->vb2_buf.timestamp = ktime_get_ns();
	vb2_set_plane_payload(&vbuf->vb2_buf, 0, bytesused);
	vb2_buffer_done(&vbuf->vb2_buf,
			err ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	dev_dbg(vcap->dev, "Done  [%d] %p phy=%pad\n", buf->vb.vb2_buf.index, buf, &buf->paddr);
	vcap->buffers_count++;
}

/* irqlock must be held */
static void dcmipp_bytecap_set_next_frame_or_stop(struct dcmipp_bytecap_device *vcap)
{
	if (!vcap->next && list_is_singular(&vcap->buffers)) {
		/*
		 * If there is no available buffer (none or a single one in the list while two
		 * are expected), stop the capture (effective for next frame). On-going frame
		 * capture will continue till FRAME END but no further capture will be done.
		 */
		reg_clear(vcap, DCMIPP_P0FCTCR, DCMIPP_P0FCTCR_CPTREQ);

		dev_dbg(vcap->dev, "Capture restart is deferred to next buffer queueing\n");
		vcap->next = NULL;
		vcap->state = WAIT_FOR_BUFFER;
		return;
	}

	/* If we don't have buffer yet, pick the one after active */
	if (!vcap->next)
		vcap->next = list_next_entry(vcap->active, list);

	/*
	 * Set buffer address
	 * This register is shadowed and will be taken into
	 * account on next VSYNC (start of next frame)
	 */
	reg_write(vcap, DCMIPP_P0PPM0AR1, vcap->next->paddr);
	dev_dbg(vcap->dev, "Write [%d] %p phy=%pad\n",
		vcap->next->vb.vb2_buf.index, vcap->next, &vcap->next->paddr);
}

/* irqlock must be held */
static void dcmipp_bytecap_process_frame(struct dcmipp_bytecap_device *vcap,
					 size_t bytesused)
{
	int err = 0;
	struct dcmipp_buf *buf = vcap->active;

	if (!buf) {
		vcap->nactive_count++;
		vcap->errors_count++;
		return;
	}

	if (bytesused > buf->size) {
		dev_dbg(vcap->dev, "frame larger than expected (%zu > %zu)\n",
			bytesused, buf->size);
		/* Clip to buffer size and return buffer to V4L2 in error */
		bytesused = buf->size;
		vcap->limit_count++;
		vcap->errors_count++;
		err = -EOVERFLOW;
	}

	dcmipp_buffer_done(vcap, buf, bytesused, err);
	vcap->active = NULL;
}

static irqreturn_t dcmipp_bytecap_irq_thread(int irq, void *arg)
{
	struct dcmipp_bytecap_device *vcap =
			container_of(arg, struct dcmipp_bytecap_device, ved);
	size_t bytesused = 0;
	u32 cmsr2;

	spin_lock_irq(&vcap->irqlock);

	cmsr2 = vcap->cmsr2 & vcap->cmier;

	/*
	 * If we have an overrun, a frame-end will probably not be generated, in that
	 * case the active buffer will be recycled as next buffer by the VSYNC handler
	 */
	if (cmsr2 & DCMIPP_CMSR2_P0OVRF) {
		vcap->errors_count++;
		vcap->overrun_count++;
	}

	if (cmsr2 & DCMIPP_CMSR2_P0FRAMEF) {
		vcap->frame_count++;

		/* Read captured buffer size */
		bytesused = reg_read(vcap, DCMIPP_P0DCCNTR);
		dcmipp_bytecap_process_frame(vcap, bytesused);
	}

	if (cmsr2 & DCMIPP_CMSR2_P0VSYNCF) {
		vcap->vsync_count++;
		if (vcap->state == WAIT_FOR_BUFFER) {
			vcap->underrun_count++;
			goto out;
		}

		/*
		 * On VSYNC, the previously set next buffer is going to become active thanks to
		 * the shadowing mechanism of the DCMIPP. In most of the cases, since a FRAMEEND
		 * has already come, pointer next is NULL since active is reset during the
		 * FRAMEEND handling. However, in case of framerate adjustment, there are more
		 * VSYNC than FRAMEEND. Thus we recycle the active (but not used) buffer and put it
		 * back into next.
		 */
		swap(vcap->active, vcap->next);
		dcmipp_bytecap_set_next_frame_or_stop(vcap);
	}

out:
	spin_unlock_irq(&vcap->irqlock);
	return IRQ_HANDLED;
}

static irqreturn_t dcmipp_bytecap_irq_callback(int irq, void *arg)
{
	struct dcmipp_bytecap_device *vcap =
			container_of(arg, struct dcmipp_bytecap_device, ved);

	/* Store interrupt status register */
	vcap->cmsr2 = reg_read(vcap, DCMIPP_CMSR2);
	vcap->it_count++;

	/* Clear interrupt */
	reg_write(vcap, DCMIPP_CMFCR, vcap->cmsr2);

	return IRQ_WAKE_THREAD;
}

static int dcmipp_bytecap_comp_bind(struct device *comp, struct device *master,
				    void *master_data)
{
	struct dcmipp_bind_data *bind_data = master_data;
	struct dcmipp_platform_data *pdata = comp->platform_data;
	struct dcmipp_bytecap_device *vcap;
	struct v4l2_pix_format *format;
	struct video_device *vdev;
	struct vb2_queue *q;
	int ret = 0;

	/* Allocate the dcmipp_bytecap_device struct */
	vcap = kzalloc(sizeof(*vcap), GFP_KERNEL);
	if (!vcap)
		return -ENOMEM;

	/* Allocate the pads */
	vcap->ved.pads =
		dcmipp_pads_init(1,
				 (const unsigned long[1]) {MEDIA_PAD_FL_SINK});
	if (IS_ERR(vcap->ved.pads)) {
		ret = PTR_ERR(vcap->ved.pads);
		goto err_free_vcap;
	}

	/* Initialize the media entity */
	vcap->vdev.entity.name = pdata->entity_name;
	vcap->vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	ret = media_entity_pads_init(&vcap->vdev.entity,
				     1, vcap->ved.pads);
	if (ret)
		goto err_clean_pads;

	/* Initialize the lock */
	mutex_init(&vcap->lock);

	/* Initialize the vb2 queue */
	q = &vcap->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_READ | VB2_DMABUF;
	q->lock = &vcap->lock;
	q->drv_priv = vcap;
	q->buf_struct_size = sizeof(struct dcmipp_buf);
	q->ops = &dcmipp_bytecap_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 1;
	q->dev = comp;

	ret = vb2_queue_init(q);
	if (ret) {
		dev_err(comp, "%s: vb2 queue init failed (err=%d)\n",
			pdata->entity_name, ret);
		goto err_clean_m_ent;
	}

	/* Initialize buffer list and its lock */
	INIT_LIST_HEAD(&vcap->buffers);
	spin_lock_init(&vcap->irqlock);

	/* Set default frame format */
	vcap->format = fmt_default;
	format = &vcap->format;
	format->bytesperline = frame_stride(format->width, format->pixelformat);
	format->sizeimage = frame_size(format->width,
				       format->height,
				       format->pixelformat);

	/* Fill the dcmipp_ent_device struct */
	vcap->ved.ent = &vcap->vdev.entity;
	vcap->ved.vdev_get_format = dcmipp_bytecap_get_format;
	vcap->ved.handler = dcmipp_bytecap_irq_callback;
	vcap->ved.thread_fn = dcmipp_bytecap_irq_thread;
	dev_set_drvdata(comp, &vcap->ved);
	vcap->dev = comp;
	vcap->regs = bind_data->regs;
	vcap->rstc = bind_data->rstc;
	vcap->cdev = master;

	/* Initialize the video_device struct */
	vdev = &vcap->vdev;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_READWRITE;
	vdev->entity.ops = &dcmipp_bytecap_mops;
	vdev->release = dcmipp_bytecap_release;
	vdev->fops = &dcmipp_bytecap_fops;
	vdev->ioctl_ops = &dcmipp_bytecap_ioctl_ops;
	vdev->lock = &vcap->lock;
	vdev->queue = q;
	vdev->v4l2_dev = bind_data->v4l2_dev;
	strscpy(vdev->name, pdata->entity_name, sizeof(vdev->name));
	video_set_drvdata(vdev, &vcap->ved);

	/* Register the video_device with the v4l2 and the media framework */
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(comp, "%s: video register failed (err=%d)\n",
			vcap->vdev.name, ret);
		goto err_clean_m_ent;
	}

	return 0;

err_clean_m_ent:
	media_entity_cleanup(&vcap->vdev.entity);
err_clean_pads:
	dcmipp_pads_cleanup(vcap->ved.pads);
err_free_vcap:
	kfree(vcap);

	return ret;
}

static const struct component_ops dcmipp_bytecap_comp_ops = {
	.bind = dcmipp_bytecap_comp_bind,
	.unbind = dcmipp_bytecap_comp_unbind,
};

static int dcmipp_bytecap_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &dcmipp_bytecap_comp_ops);
}

static int dcmipp_bytecap_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dcmipp_bytecap_comp_ops);

	return 0;
}

static const struct platform_device_id dcmipp_bytecap_driver_ids[] = {
	{
		.name	= DCMIPP_BYTECAP_DRV_NAME,
	},
	{ }
};

static struct platform_driver dcmipp_bytecap_pdrv = {
	.probe		= dcmipp_bytecap_probe,
	.remove		= dcmipp_bytecap_remove,
	.id_table	= dcmipp_bytecap_driver_ids,
	.driver		= {
		.name	= DCMIPP_BYTECAP_DRV_NAME,
	},
};

module_platform_driver(dcmipp_bytecap_pdrv);

MODULE_DEVICE_TABLE(platform, dcmipp_bytecap_driver_ids);

MODULE_AUTHOR("Hugues Fruchet <hugues.fruchet@foss.st.com>");
MODULE_AUTHOR("Alain Volmat <alain.volmat@foss.st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 Digital Camera Memory Interface with Pixel Processor driver");
MODULE_LICENSE("GPL");
