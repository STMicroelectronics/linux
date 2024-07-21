// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for STM32 Digital Camera Memory Interface Pixel Processor
 *
 * Copyright (C) STMicroelectronics SA 2021
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          Alain Volmat <alain.volmat@foss.st.com>
 *          for STMicroelectronics.
 */

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

#define DCMIPP_PIXELCAP_DRV_NAME "dcmipp-pixelcap"

#define DCMIPP_PRSR (0x1F8)
#define DCMIPP_CMIER (0x3F0)
#define DCMIPP_CMIER_P1FRAMEIE BIT(17)
#define DCMIPP_CMIER_P1VSYNCIE BIT(18)
#define DCMIPP_CMIER_P1OVRIE BIT(23)
#define DCMIPP_CMIER_P2FRAMEIE BIT(25)
#define DCMIPP_CMIER_P2VSYNCIE BIT(26)
#define DCMIPP_CMIER_P2OVRIE BIT(31)
#define DCMIPP_CMIER_PxALL(id) (((id) == 1) ? (DCMIPP_CMIER_P1VSYNCIE |\
					       DCMIPP_CMIER_P1FRAMEIE |\
					       DCMIPP_CMIER_P1OVRIE) :\
					      (DCMIPP_CMIER_P2VSYNCIE |\
					       DCMIPP_CMIER_P2FRAMEIE |\
					       DCMIPP_CMIER_P2OVRIE))
#define DCMIPP_CMSR1 (0x3F4)
#define DCMIPP_CMSR2 (0x3F8)
#define DCMIPP_CMSR2_P1FRAMEF BIT(17)
#define DCMIPP_CMSR2_P1VSYNCF BIT(18)
#define DCMIPP_CMSR2_P1OVRF BIT(23)
#define DCMIPP_CMSR2_P2FRAMEF BIT(25)
#define DCMIPP_CMSR2_P2VSYNCF BIT(26)
#define DCMIPP_CMSR2_P2OVRF BIT(31)
#define DCMIPP_CMSR2_PxFRAMEF(id) (((id) == 1) ? DCMIPP_CMSR2_P1FRAMEF :\
						 DCMIPP_CMSR2_P2FRAMEF)
#define DCMIPP_CMSR2_PxVSYNCF(id) (((id) == 1) ? DCMIPP_CMSR2_P1VSYNCF :\
						 DCMIPP_CMSR2_P2VSYNCF)
#define DCMIPP_CMSR2_PxOVRF(id) (((id) == 1) ? DCMIPP_CMSR2_P1OVRF :\
					       DCMIPP_CMSR2_P2OVRF)
#define DCMIPP_CMFCR (0x3FC)
#define DCMIPP_P1FSCR (0x804)
#define DCMIPP_P2FSCR (0xC04)
#define DCMIPP_PxFSCR(id) (((id) == 1) ? DCMIPP_P1FSCR :\
					 DCMIPP_P2FSCR)
#define DCMIPP_PxFSCR_PIPEN BIT(31)
#define DCMIPP_P1FCTCR (0x900)
#define DCMIPP_P2FCTCR (0xD00)
#define DCMIPP_PxFCTCR(id) (((id) == 1) ? DCMIPP_P1FCTCR :\
					  DCMIPP_P2FCTCR)
#define DCMIPP_PxFCTCR_CPTMODE BIT(2)
#define DCMIPP_PxFCTCR_CPTREQ BIT(3)
#define DCMIPP_P1PPM0AR1 (0x9C4)
#define DCMIPP_P2PPM0AR1 (0xDC4)
#define DCMIPP_PxPPM0AR1(id) (((id) == 1) ? DCMIPP_P1PPM0AR1 :\
					    DCMIPP_P2PPM0AR1)
#define DCMIPP_P1PPM0PR (0x9CC)
#define DCMIPP_P2PPM0PR (0xDCC)
#define DCMIPP_PxPPM0PR(id) (((id) == 1) ? DCMIPP_P1PPM0PR :\
					   DCMIPP_P2PPM0PR)
#define DCMIPP_P1PPM1AR1 (0x9D4)
#define DCMIPP_P1PPM1PR (0x9DC)
#define DCMIPP_P1PPM2AR1 (0x9E4)

#define DCMIPP_P1SR (0x9F8)
#define DCMIPP_P2SR (0xDF8)
#define DCMIPP_PxSR(id) (((id) == 1) ? DCMIPP_P1SR :\
				       DCMIPP_P2SR)
#define DCMIPP_PxSR_CPTACT BIT(23)

struct dcmipp_pixelcap_pix_map {
	unsigned int code;
	u32 pixelformat;
	u32 plane_nb;
};

#define PIXMAP_MBUS_PFMT(mbus, fmt, nb_plane)			\
		{						\
			.code = MEDIA_BUS_FMT_##mbus,		\
			.pixelformat = V4L2_PIX_FMT_##fmt,	\
			.plane_nb = nb_plane,			\
		}

static const struct dcmipp_pixelcap_pix_map dcmipp_pixelcap_pix_map_list[] = {
	/* Coplanar formats are supported on main & aux pipe */
	PIXMAP_MBUS_PFMT(RGB565_2X8_LE, RGB565, 1),
	PIXMAP_MBUS_PFMT(YUYV8_2X8, YUYV, 1),
	PIXMAP_MBUS_PFMT(YVYU8_2X8, YVYU, 1),
	PIXMAP_MBUS_PFMT(UYVY8_2X8, UYVY, 1),
	PIXMAP_MBUS_PFMT(VYUY8_2X8, VYUY, 1),
	PIXMAP_MBUS_PFMT(Y8_1X8, GREY, 1),
	PIXMAP_MBUS_PFMT(RGB888_1X24, RGB24, 1),
	PIXMAP_MBUS_PFMT(BGR888_1X24, BGR24, 1),
	PIXMAP_MBUS_PFMT(ARGB8888_1X32, ARGB32, 1),
	PIXMAP_MBUS_PFMT(AYUV8_1X32, AYUV32, 1),

	/* Semiplanar & planar formats (plane_nb > 1) are only supported on main pipe */
	PIXMAP_MBUS_PFMT(YUYV8_1_5X8, NV12, 2),   /* FIXME no mbus code for NV12 */
	PIXMAP_MBUS_PFMT(YVYU8_1_5X8, NV21, 2),   /* FIXME no mbus code for NV21 */
	PIXMAP_MBUS_PFMT(YUYV8_1X16, NV16, 2),    /* FIXME no mbus code for NV16 */
	PIXMAP_MBUS_PFMT(YVYU8_1X16, NV61, 2),    /* FIXME no mbus code for NV61 */
	PIXMAP_MBUS_PFMT(UYVY8_1_5X8, YUV420, 3), /* FIXME no mbus code for YUV420 */
	PIXMAP_MBUS_PFMT(VYUY8_1_5X8, YVU420, 3), /* FIXME no mbus code for YVU420 */
};

static const struct dcmipp_pixelcap_pix_map *dcmipp_pixelcap_pix_map_by_pixelformat
						(u32 pixelformat)
{
	const struct dcmipp_pixelcap_pix_map *l = dcmipp_pixelcap_pix_map_list;
	unsigned int size = ARRAY_SIZE(dcmipp_pixelcap_pix_map_list);
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (l[i].pixelformat == pixelformat)
			return &l[i];
	}

	return NULL;
}

static const struct dcmipp_pixelcap_pix_map *dcmipp_pixelcap_pix_map_by_index(unsigned int i)
{
	const struct dcmipp_pixelcap_pix_map *l = dcmipp_pixelcap_pix_map_list;
	unsigned int size = ARRAY_SIZE(dcmipp_pixelcap_pix_map_list);

	if (i >= size)
		return NULL;

	return &l[i];
}

struct dcmipp_buf {
	struct vb2_v4l2_buffer	vb;
	bool			prepared;
	dma_addr_t		paddr;
	size_t			size;
	dma_addr_t		paddrs[3];
	u32			strides[3];
	u64			sizes[3];
	struct list_head	list;
};

enum state {
	STOPPED = 0,
	WAIT_FOR_BUFFER,
	RUNNING,
};

struct dcmipp_pixelcap_device {
	struct dcmipp_ent_device ved;
	struct video_device vdev;
	struct device *dev;
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

	u32 pipe_id;

	u32 cmier;
	u32 cmsr2;

	int errors_count;
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
	.colorspace = V4L2_COLORSPACE_REC709,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
	.xfer_func = V4L2_XFER_FUNC_DEFAULT,
};

struct dcmipp_pixelcap_buffer {
	/*
	 * struct vb2_v4l2_buffer must be the first element
	 * the videobuf2 framework will allocate this struct based on
	 * buf_struct_size and use the first sizeof(struct vb2_buffer) bytes of
	 * memory as a vb2_buffer
	 */
	struct vb2_v4l2_buffer vb2;
	struct list_head list;
};

static inline int hdw_pixel_alignment(u32 format)
{
	/* 16 bytes alignment required by hardware */
	switch (format) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_GREY:
		return 4;/* 2^4 = 16 pixels = 16 bytes */
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		return 3;/* 2^3  = 8 pixels = 16 bytes */
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
		return 4;/* 2^4 = 16 pixels = 48 bytes */
	case V4L2_PIX_FMT_ARGB32:
		return 2;/* 2^2  = 4 pixels = 16 bytes */
	default:
		return 0;
	}
}

static inline int frame_planes(dma_addr_t base_addr, dma_addr_t addrs[],
			       u32 strides[], u64 sizes[],
			       u32 width, u32 height, u32 format)
{
	const struct v4l2_format_info *info;

	info = v4l2_format_info(format);
	if (!info)
		return -EINVAL;

	/* Fill-in each plane information */
	addrs[0] = base_addr;
	strides[0] = width * info->bpp[0];
	sizes[0] = strides[0] * height;

	if (info->comp_planes > 1) {
		addrs[1] = addrs[0] + sizes[0];
		strides[1] = width * info->bpp[1] / info->hdiv;
		sizes[1] = strides[1] * height / info->vdiv;
	}

	if (info->comp_planes > 2) {
		addrs[2] = addrs[1] + sizes[1];
		strides[2] = width * info->bpp[2] / info->hdiv;
		sizes[2] = strides[2] * height / info->vdiv;
	}

	return 0;
}

static int dcmipp_pixelcap_querycap(struct file *file, void *priv,
				    struct v4l2_capability *cap)
{
	strscpy(cap->driver, DCMIPP_PDEV_NAME, sizeof(cap->driver));
	strscpy(cap->card, KBUILD_MODNAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "platform:%s", DCMIPP_PDEV_NAME);

	return 0;
}

static void dcmipp_pixelcap_get_format(struct dcmipp_ent_device *ved,
				       struct v4l2_pix_format *fmt)
{
	struct dcmipp_pixelcap_device *vcap =
		container_of(ved, struct dcmipp_pixelcap_device, ved);

	*fmt = vcap->format;
}

static int dcmipp_pixelcap_g_fmt_vid_cap(struct file *file, void *priv,
					 struct v4l2_format *f)
{
	struct dcmipp_pixelcap_device *vcap = video_drvdata(file);

	f->fmt.pix = vcap->format;

	return 0;
}

static int dcmipp_pixelcap_try_fmt_vid_cap(struct file *file, void *priv,
					   struct v4l2_format *f)
{
	struct dcmipp_pixelcap_device *vcap = video_drvdata(file);
	struct v4l2_pix_format *format = &f->fmt.pix;
	const struct dcmipp_pixelcap_pix_map *vpix;
	u32 in_w, in_h;

	/* Don't accept a pixelformat that is not on the table */
	vpix = dcmipp_pixelcap_pix_map_by_pixelformat(format->pixelformat);
	if (!vpix) {
		format->pixelformat = fmt_default.pixelformat;
		vpix = dcmipp_pixelcap_pix_map_by_pixelformat(format->pixelformat);
	}

	/* Semiplanar & planar formats are only supported on main pipe */
	if (vpix->plane_nb > 1 && vcap->pipe_id != 1)
		format->pixelformat = fmt_default.pixelformat;

	/* Adjust width & height */
	in_w = format->width;
	in_h = format->height;
	format->width = clamp_t(u32, format->width, DCMIPP_FRAME_MIN_WIDTH,
				DCMIPP_FRAME_MAX_WIDTH);
	format->width = round_up(format->width,
				 1 << hdw_pixel_alignment(format->pixelformat));
	format->height = clamp_t(u32, format->height,
				 DCMIPP_FRAME_MIN_HEIGHT,
				 DCMIPP_FRAME_MAX_HEIGHT);
	if (format->width != in_w || format->height != in_h)
		dev_dbg(vcap->dev,
			"resolution updated: %dx%d -> %dx%d\n",
			in_w, in_h, format->width, format->height);

	v4l2_fill_pixfmt(format, format->pixelformat,
			 format->width, format->height);

	if (format->field == V4L2_FIELD_ANY)
		format->field = fmt_default.field;

	dcmipp_colorimetry_clamp(format);

	return 0;
}

static int dcmipp_pixelcap_s_fmt_vid_cap(struct file *file, void *priv,
					 struct v4l2_format *f)
{
	struct dcmipp_pixelcap_device *vcap = video_drvdata(file);
	int ret;

	/* Do not change the format while stream is on */
	if (vb2_is_busy(&vcap->queue))
		return -EBUSY;

	ret = dcmipp_pixelcap_try_fmt_vid_cap(file, priv, f);
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

static int dcmipp_pixelcap_enum_fmt_vid_cap(struct file *file, void *priv,
					    struct v4l2_fmtdesc *f)
{
	const struct dcmipp_pixelcap_pix_map *vpix = dcmipp_pixelcap_pix_map_by_index(f->index);
	struct dcmipp_pixelcap_device *vcap = video_drvdata(file);

	vpix = dcmipp_pixelcap_pix_map_by_index(f->index);
	if (!vpix)
		return -EINVAL;

	/* Semiplanar & planar formats are only supported on main pipe */
	if (vpix->plane_nb > 1 && vcap->pipe_id != 1)
		return -EINVAL;

	f->pixelformat = vpix->pixelformat;

	return 0;
}

static int dcmipp_pixelcap_enum_framesizes(struct file *file, void *fh,
					   struct v4l2_frmsizeenum *fsize)
{
	const struct dcmipp_pixelcap_pix_map *vpix;

	if (fsize->index)
		return -EINVAL;

	/* Only accept code in the pix map table */
	vpix = dcmipp_pixelcap_pix_map_by_pixelformat(fsize->pixel_format);
	if (!vpix)
		return -EINVAL;

	/* TODO - need to differentiate here format of MAIN vs AUX */

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = DCMIPP_FRAME_MIN_WIDTH;
	fsize->stepwise.max_width = DCMIPP_FRAME_MAX_WIDTH;
	fsize->stepwise.min_height = DCMIPP_FRAME_MIN_HEIGHT;
	fsize->stepwise.max_height = DCMIPP_FRAME_MAX_HEIGHT;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int dcmipp_pixelcap_open(struct file *file)
{
	struct dcmipp_pixelcap_device *vcap = video_drvdata(file);
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

static int dcmipp_pixelcap_close(struct file *file)
{
	struct dcmipp_pixelcap_device *vcap = video_drvdata(file);

	vb2_fop_release(file);

	v4l2_pipeline_pm_put(&vcap->vdev.entity);

	return 0;
}

static const struct v4l2_file_operations dcmipp_pixelcap_fops = {
	.owner		= THIS_MODULE,
	.open		= dcmipp_pixelcap_open,
	.release	= dcmipp_pixelcap_close,
	.read           = vb2_fop_read,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = vb2_fop_mmap,
};

static const struct v4l2_ioctl_ops dcmipp_pixelcap_ioctl_ops = {
	.vidioc_querycap = dcmipp_pixelcap_querycap,

	.vidioc_g_fmt_vid_cap = dcmipp_pixelcap_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = dcmipp_pixelcap_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = dcmipp_pixelcap_try_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap = dcmipp_pixelcap_enum_fmt_vid_cap,
	.vidioc_enum_framesizes = dcmipp_pixelcap_enum_framesizes,

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

static int dcmipp_pipeline_s_stream(struct dcmipp_pixelcap_device *vcap,
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

static int dcmipp_start_capture(struct dcmipp_pixelcap_device *vcap,
				struct dcmipp_buf *buf)
{
	/*
	 * Set frame addresses
	 * Those registers are taken into account immediately
	 */
	if (buf->paddrs[0]) {
		reg_write(vcap, DCMIPP_PxPPM0AR1(vcap->pipe_id), buf->paddrs[0]);
		reg_write(vcap, DCMIPP_PxPPM0PR(vcap->pipe_id), buf->strides[0]);
	}

	if (buf->paddrs[1]) {
		reg_write(vcap, DCMIPP_P1PPM1AR1, buf->paddrs[1]);
		reg_write(vcap, DCMIPP_P1PPM1PR, buf->strides[1]);
	}

	if (buf->paddrs[2])
		reg_write(vcap, DCMIPP_P1PPM2AR1, buf->paddrs[2]);

	/* Capture request */
	reg_set(vcap, DCMIPP_PxFCTCR(vcap->pipe_id), DCMIPP_PxFCTCR_CPTREQ);

	return 0;
}

static int dcmipp_pixelcap_start_streaming(struct vb2_queue *vq,
					   unsigned int count)
{
	struct dcmipp_pixelcap_device *vcap = vb2_get_drv_priv(vq);
	struct media_entity *entity = &vcap->vdev.entity;
	struct media_device *mdev = entity->graph_obj.mdev;
	struct media_pipeline *pipe;
	struct dcmipp_buf *buf, *node;
	int ret;

	vcap->sequence = 0;
	vcap->errors_count = 0;
	vcap->overrun_count = 0;
	vcap->buffers_count = 0;
	vcap->vsync_count = 0;
	vcap->frame_count = 0;
	vcap->it_count = 0;
	vcap->underrun_count = 0;
	vcap->nactive_count = 0;

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

	/* Enable interruptions */
	vcap->cmier |= DCMIPP_CMIER_PxALL(vcap->pipe_id);
	spin_lock(&vcap->vdev.v4l2_dev->lock);
	reg_set(vcap, DCMIPP_CMIER, vcap->cmier);
	spin_unlock(&vcap->vdev.v4l2_dev->lock);

	/* Enable pipe at the end of programming */
	reg_set(vcap, DCMIPP_PxFSCR(vcap->pipe_id), DCMIPP_PxFSCR_PIPEN);

	/*
	 * Start capture if at least one buffer has been queued,
	 * otherwise start is deferred at next buffer queueing
	 */
	buf = list_first_entry_or_null(&vcap->buffers, typeof(*buf), list);
	if (!buf) {
		dev_dbg(vcap->dev, "Start streaming is deferred to next buffer queueing\n");
		vcap->next = NULL;
		vcap->state = WAIT_FOR_BUFFER;
		return 0;
	}
	vcap->next = buf;
	dev_dbg(vcap->dev, "Start with next [%d] %p phy=%pad\n",
		buf->vb.vb2_buf.index, buf, &buf->paddr);

	vcap->state = RUNNING;

	/* Start capture */
	ret = dcmipp_start_capture(vcap, buf);
	if (ret)
		goto err_media_pipeline_stop;

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
	vcap->active = NULL;
	spin_unlock_irq(&vcap->irqlock);

	return ret;
}

static void dcmipp_dump_status(struct dcmipp_pixelcap_device *vcap)
{
	struct device *dev = vcap->dev;

	dev_dbg(dev, "[DCMIPP_PRSR]  =%#10.8x\n", reg_read(vcap, DCMIPP_PRSR));
	dev_dbg(dev, "[DCMIPP_PxSR] =%#10.8x\n", reg_read(vcap, DCMIPP_PxSR(vcap->pipe_id)));
	dev_dbg(dev, "[DCMIPP_CMSR1] =%#10.8x\n", reg_read(vcap, DCMIPP_CMSR1));
	dev_dbg(dev, "[DCMIPP_CMSR2] =%#10.8x\n", reg_read(vcap, DCMIPP_CMSR2));
}

/*
 * Stop the stream engine. Any remaining buffers in the stream queue are
 * dequeued and passed on to the vb2 framework marked as STATE_ERROR.
 */
static void dcmipp_pixelcap_stop_streaming(struct vb2_queue *vq)
{
	struct dcmipp_pixelcap_device *vcap = vb2_get_drv_priv(vq);
	struct dcmipp_buf *buf, *node;
	int ret;
	u32 status;

	dcmipp_pipeline_s_stream(vcap, 0);

	/* Stop the media pipeline */
	media_pipeline_stop(vcap->vdev.entity.pads);

	/* Disable interruptions */
	spin_lock(&vcap->vdev.v4l2_dev->lock);
	reg_clear(vcap, DCMIPP_CMIER, vcap->cmier);
	spin_unlock(&vcap->vdev.v4l2_dev->lock);

	/* Stop capture */
	reg_clear(vcap, DCMIPP_PxFCTCR(vcap->pipe_id), DCMIPP_PxFCTCR_CPTREQ);

	/* Wait until CPTACT become 0 */
	ret = readl_relaxed_poll_timeout(vcap->regs + DCMIPP_PxSR(vcap->pipe_id),
					 status,
					 !(status & DCMIPP_PxSR_CPTACT),
					 20, 1000);
	if (ret)
		dev_warn(vcap->dev, "Timeout when stopping\n");

	/* Disable pipe */
	reg_clear(vcap, DCMIPP_PxFSCR(vcap->pipe_id), DCMIPP_PxFSCR_PIPEN);

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

	pm_runtime_put(vcap->dev);

	if (vcap->errors_count)
		dev_warn(vcap->dev, "Some errors found while streaming: errors=%d (overrun=%d, nactive=%d), underrun=%d, buffers=%d\n",
			 vcap->errors_count, vcap->overrun_count, vcap->nactive_count,
			 vcap->underrun_count, vcap->buffers_count);
	dev_dbg(vcap->dev, "Stop streaming, errors=%d (overrun=%d, nactive=%d), underrun=%d, vsync=%d, frame=%d, buffers=%d, it=%d\n",
		vcap->errors_count, vcap->overrun_count, vcap->nactive_count,
		vcap->underrun_count, vcap->vsync_count,
		vcap->frame_count, vcap->buffers_count,
		vcap->it_count);
}

static int dcmipp_pixelcap_buf_prepare(struct vb2_buffer *vb)
{
	struct dcmipp_pixelcap_device *vcap =  vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct dcmipp_buf *buf = container_of(vbuf, struct dcmipp_buf, vb);
	struct v4l2_pix_format *format = &vcap->format;
	unsigned long size;
	int ret;

	size = format->sizeimage;

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

		ret = frame_planes(buf->paddr,
				   buf->paddrs, buf->strides, buf->sizes,
				   format->width, format->height,
				   format->pixelformat);
		if (ret) {
			dev_err(vcap->dev, "%s: Unsupported pixel format (%x)\n",
				__func__, format->pixelformat);
			return ret;
		}

		/* Check for 16 bytes alignment required by hardware */
		WARN_ON(buf->paddrs[0] & 15);
		WARN_ON(buf->strides[0] & 15);
		WARN_ON(buf->paddrs[1] & 15);
		WARN_ON(buf->strides[1] & 15);
		WARN_ON(buf->paddrs[2] & 15);

		buf->prepared = true;

		vb2_set_plane_payload(&buf->vb.vb2_buf, 0, buf->size);

		dev_dbg(vcap->dev, "buffer[%d] phy=%pad size=%zu\n",
			vb->index, &buf->paddr, buf->size);
	}

	return 0;
}

static void dcmipp_pixelcap_buf_queue(struct vb2_buffer *vb2_buf)
{
	struct dcmipp_pixelcap_device *vcap = vb2_get_drv_priv(vb2_buf->vb2_queue);
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

static int dcmipp_pixelcap_queue_setup(struct vb2_queue *vq,
				       unsigned int *nbuffers,
				       unsigned int *nplanes,
				       unsigned int sizes[],
				       struct device *alloc_devs[])
{
	struct dcmipp_pixelcap_device *vcap = vb2_get_drv_priv(vq);
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

static int dcmipp_pixelcap_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct dcmipp_buf *buf = container_of(vbuf, struct dcmipp_buf, vb);

	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static const struct vb2_ops dcmipp_pixelcap_qops = {
	.start_streaming	= dcmipp_pixelcap_start_streaming,
	.stop_streaming		= dcmipp_pixelcap_stop_streaming,
	.buf_init		= dcmipp_pixelcap_buf_init,
	.buf_prepare		= dcmipp_pixelcap_buf_prepare,
	.buf_queue		= dcmipp_pixelcap_buf_queue,
	.queue_setup		= dcmipp_pixelcap_queue_setup,
	/*
	 * Since q->lock is set we can use the standard
	 * vb2_ops_wait_prepare/finish helper functions.
	 */
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static const struct media_entity_operations dcmipp_pixelcap_mops = {
	.link_validate		= dcmipp_link_validate,
};

static void dcmipp_pixelcap_release(struct video_device *vdev)
{
	struct dcmipp_pixelcap_device *vcap =
		container_of(vdev, struct dcmipp_pixelcap_device, vdev);

	dcmipp_pads_cleanup(vcap->ved.pads);
	kfree(vcap);
}

void dcmipp_pixelcap_ent_release(struct dcmipp_ent_device *ved)
{
	struct dcmipp_pixelcap_device *vcap =
		container_of(ved, struct dcmipp_pixelcap_device, ved);

	mutex_destroy(&vcap->lock);
	media_entity_cleanup(ved->ent);
	vb2_video_unregister_device(&vcap->vdev);
}

static void dcmipp_buffer_done(struct dcmipp_pixelcap_device *vcap,
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

	vcap->buffers_count++;
}

/* irqlock must be held */
static void dcmipp_pixelcap_set_next_frame_or_stop(struct dcmipp_pixelcap_device *vcap)
{
	if (!vcap->next && list_is_singular(&vcap->buffers)) {
		/*
		 * If there is no available buffer (none or a single one in the list while two
		 * are expected), stop the capture (effective for next frame). On-going frame
		 * capture will continue till FRAME END but no further capture will be done.
		 */
		reg_clear(vcap, DCMIPP_PxFCTCR(vcap->pipe_id), DCMIPP_PxFCTCR_CPTREQ);

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
	reg_write(vcap, DCMIPP_PxPPM0AR1(vcap->pipe_id), vcap->next->paddrs[0]);
	if (vcap->next->paddrs[1])
		reg_write(vcap, DCMIPP_P1PPM1AR1, vcap->next->paddrs[1]);
	if (vcap->next->paddrs[2])
		reg_write(vcap, DCMIPP_P1PPM2AR1, vcap->next->paddrs[2]);
	dev_dbg(vcap->dev, "Write [%d] %p phy=%pad\n",
		vcap->next->vb.vb2_buf.index, vcap->next, &vcap->next->paddr);
}

/* irqlock must be held */
static void dcmipp_pixelcap_process_frame(struct dcmipp_pixelcap_device *vcap)
{
	int err = 0;
	struct dcmipp_buf *buf = vcap->active;

	if (!buf) {
		vcap->nactive_count++;
		vcap->errors_count++;
		return;
	}

	dcmipp_buffer_done(vcap, buf, vcap->format.sizeimage, err);
	vcap->active = NULL;
}

static irqreturn_t dcmipp_pixelcap_irq_thread(int irq, void *arg)
{
	struct dcmipp_pixelcap_device *vcap =
			container_of(arg, struct dcmipp_pixelcap_device, ved);
	u32 cmsr2_pxframef;
	u32 cmsr2_pxvsyncf;
	u32 cmsr2_pxovrf;

	spin_lock_irq(&vcap->irqlock);

	cmsr2_pxovrf = DCMIPP_CMSR2_PxOVRF(vcap->pipe_id);
	cmsr2_pxvsyncf = DCMIPP_CMSR2_PxVSYNCF(vcap->pipe_id);
	cmsr2_pxframef = DCMIPP_CMSR2_PxFRAMEF(vcap->pipe_id);

	if (vcap->cmsr2 & cmsr2_pxovrf) {
		vcap->errors_count++;
		vcap->overrun_count++;
	}

	if (vcap->cmsr2 & cmsr2_pxframef) {
		vcap->frame_count++;

		dcmipp_pixelcap_process_frame(vcap);
	}

	if (vcap->cmsr2 & cmsr2_pxvsyncf) {
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
		dcmipp_pixelcap_set_next_frame_or_stop(vcap);
	}

out:
	spin_unlock_irq(&vcap->irqlock);
	return IRQ_HANDLED;
}

static irqreturn_t dcmipp_pixelcap_irq_callback(int irq, void *arg)
{
	struct dcmipp_pixelcap_device *vcap =
			container_of(arg, struct dcmipp_pixelcap_device, ved);
	struct dcmipp_ent_device *ved = arg;

	vcap->cmsr2 = ved->cmsr2 & vcap->cmier;

	vcap->it_count++;

	/* Clear interrupt */
	reg_write(vcap, DCMIPP_CMFCR, vcap->cmsr2);

	return IRQ_WAKE_THREAD;
}

static int dcmipp_name_to_pipe_id(const char *name)
{
	if (strstr(name, "main"))
		return 1;
	else if (strstr(name, "aux"))
		return 2;
	else
		return -EINVAL;
}

struct dcmipp_ent_device *
dcmipp_pixelcap_ent_init(struct device *dev, const char *entity_name,
			 struct v4l2_device *v4l2_dev, void __iomem *regs)
{
	struct dcmipp_pixelcap_device *vcap;
	struct v4l2_pix_format *format;
	struct video_device *vdev;
	struct vb2_queue *q;
	int ret = 0;

	/* Allocate the dcmipp_cap_device struct */
	vcap = kzalloc(sizeof(*vcap), GFP_KERNEL);
	if (!vcap)
		return ERR_PTR(-ENOMEM);

	/* Retrieve the pipe_id */
	vcap->pipe_id = dcmipp_name_to_pipe_id(entity_name);
	if (vcap->pipe_id != 1 && vcap->pipe_id != 2) {
		dev_err(dev, "failed to retrieve pipe_id\n");
		goto err_free_vcap;
	}
	/* Allocate the pads */
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
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_READ | VB2_DMABUF;
	q->lock = &vcap->lock;
	q->drv_priv = vcap;
	q->buf_struct_size = sizeof(struct dcmipp_buf);
	q->ops = &dcmipp_pixelcap_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 1;
	q->dev = dev;

	ret = vb2_queue_init(q);
	if (ret) {
		dev_err(dev, "%s: vb2 queue init failed (err=%d)\n",
			entity_name, ret);
		goto err_mutex_destroy;
	}

	/* Initialize buffer list and its lock */
	INIT_LIST_HEAD(&vcap->buffers);
	spin_lock_init(&vcap->irqlock);

	/* Set default frame format */
	vcap->format = fmt_default;
	format = &vcap->format;
	v4l2_fill_pixfmt(format, format->pixelformat,
			 format->width, format->height);

	/* Fill the dcmipp_ent_device struct */
	vcap->ved.ent = &vcap->vdev.entity;
	vcap->ved.vdev_get_format = dcmipp_pixelcap_get_format;
	vcap->ved.handler = dcmipp_pixelcap_irq_callback;
	vcap->ved.thread_fn = dcmipp_pixelcap_irq_thread;
	vcap->dev = dev;
	vcap->regs = regs;

	/* Initialize the video_device struct */
	vdev = &vcap->vdev;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_IO_MC;
	vdev->entity.ops = &dcmipp_pixelcap_mops;
	vdev->release = dcmipp_pixelcap_release;
	vdev->fops = &dcmipp_pixelcap_fops;
	vdev->ioctl_ops = &dcmipp_pixelcap_ioctl_ops;
	vdev->lock = &vcap->lock;
	vdev->queue = q;
	vdev->v4l2_dev = v4l2_dev;
	strscpy(vdev->name, entity_name, sizeof(vdev->name));
	video_set_drvdata(vdev, &vcap->ved);

	/* Register the video_device with the v4l2 and the media framework */
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(dev, "%s: video register failed (err=%d)\n",
			vcap->vdev.name, ret);
		goto err_mutex_destroy;
	}

	return &vcap->ved;

err_mutex_destroy:
	mutex_destroy(&vcap->lock);
	media_entity_cleanup(&vcap->vdev.entity);
err_clean_pads:
	dcmipp_pads_cleanup(vcap->ved.pads);
err_free_vcap:
	kfree(vcap);

	return ERR_PTR(ret);
}
