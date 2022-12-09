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
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-rect.h>
#include <media/v4l2-subdev.h>

#include "dcmipp-common.h"

#define DCMIPP_BYTEPROC_DRV_NAME "dcmipp-byteproc"

#define DCMIPP_FMT_WIDTH_DEFAULT  640
#define DCMIPP_FMT_HEIGHT_DEFAULT 480

#define DCMIPP_P0FCTCR (0x500)
#define DCMIPP_P0FCTCR_FRATE_MASK GENMASK(1, 0)
#define DCMIPP_P0SCSTR (0x504)
#define DCMIPP_P0SCSTR_HSTART_SHIFT	0
#define DCMIPP_P0SCSTR_VSTART_SHIFT	16
#define DCMIPP_P0SCSZR (0x508)
#define DCMIPP_P0SCSZR_ENABLE BIT(31)
#define DCMIPP_P0SCSZR_HSIZE_SHIFT	0
#define DCMIPP_P0SCSZR_VSIZE_SHIFT	16
#define DCMIPP_P0PPCR (0x5C0)
#define DCMIPP_P0PPCR_BSM_1_2 0x1
#define DCMIPP_P0PPCR_BSM_1_4 0x2
#define DCMIPP_P0PPCR_BSM_2_4 0x3
#define DCMIPP_P0PPCR_BSM_MASK GENMASK(8, 7)
#define DCMIPP_P0PPCR_BSM_SHIFT 0x7
#define DCMIPP_P0PPCR_LSM BIT(10)
#define DCMIPP_P0PPCR_OELS BIT(11)

#define IS_SINK(pad) (!(pad))
#define IS_SRC(pad)  ((pad))
#define PAD_STR(pad) (IS_SRC((pad))) ? "src" : "sink"

#define BYTEPROC_MEDIA_BUS_FMT_DEFAULT MEDIA_BUS_FMT_RGB565_2X8_LE

struct dcmipp_byteproc_pix_map {
	unsigned int code;
	unsigned int bpp;
};

#define PIXMAP_MBUS_BPP(mbus, byteperpixel)	\
		{						\
			.code = MEDIA_BUS_FMT_##mbus,		\
			.bpp = byteperpixel,	\
		}
static const struct dcmipp_byteproc_pix_map dcmipp_byteproc_pix_map_list[] = {
	PIXMAP_MBUS_BPP(RGB565_2X8_LE, 2),
	PIXMAP_MBUS_BPP(YUYV8_2X8, 2),
	PIXMAP_MBUS_BPP(YVYU8_2X8, 2),
	PIXMAP_MBUS_BPP(UYVY8_2X8, 2),
	PIXMAP_MBUS_BPP(VYUY8_2X8, 2),
	PIXMAP_MBUS_BPP(Y8_1X8, 1),
	PIXMAP_MBUS_BPP(SBGGR8_1X8, 1),
	PIXMAP_MBUS_BPP(SGBRG8_1X8, 1),
	PIXMAP_MBUS_BPP(SGRBG8_1X8, 1),
	PIXMAP_MBUS_BPP(SRGGB8_1X8, 1),
	PIXMAP_MBUS_BPP(JPEG_1X8, 1),
};

static const struct dcmipp_byteproc_pix_map *dcmipp_byteproc_pix_map_by_index(unsigned int i)
{
	const struct dcmipp_byteproc_pix_map *l = dcmipp_byteproc_pix_map_list;
	unsigned int size = ARRAY_SIZE(dcmipp_byteproc_pix_map_list);

	if (i >= size)
		return NULL;

	return &l[i];
}

static const struct dcmipp_byteproc_pix_map *dcmipp_byteproc_pix_map_by_code(u32 code)
{
	const struct dcmipp_byteproc_pix_map *l = dcmipp_byteproc_pix_map_list;
	unsigned int size = ARRAY_SIZE(dcmipp_byteproc_pix_map_list);
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (l[i].code == code)
			return &l[i];
	}

	return NULL;
}

struct dcmipp_byteproc_device {
	struct dcmipp_ent_device ved;
	struct v4l2_subdev sd;
	struct device *dev;
	struct v4l2_mbus_framefmt sink_fmt;
	bool streaming;
	/* Protect this data structure */
	struct mutex lock;

	void __iomem *regs;

	struct v4l2_fract sink_interval;
	struct v4l2_fract src_interval;
	unsigned int frate;
	u32 src_code;
	struct v4l2_rect crop;
	struct v4l2_rect compose;
};

static const struct v4l2_mbus_framefmt fmt_default = {
	.width = DCMIPP_FMT_WIDTH_DEFAULT,
	.height = DCMIPP_FMT_HEIGHT_DEFAULT,
	.code = BYTEPROC_MEDIA_BUS_FMT_DEFAULT,
	.field = V4L2_FIELD_NONE,
	.colorspace = DCMIPP_COLORSPACE_DEFAULT,
	.ycbcr_enc = DCMIPP_YCBCR_ENC_DEFAULT,
	.quantization = DCMIPP_QUANTIZATION_DEFAULT,
	.xfer_func = DCMIPP_XFER_FUNC_DEFAULT,
};

static const struct v4l2_rect crop_min = {
	.width = DCMIPP_FRAME_MIN_WIDTH,
	.height = DCMIPP_FRAME_MIN_HEIGHT,
	.top = 0,
	.left = 0,
};

static struct v4l2_rect
dcmipp_byteproc_get_compose_bound(const struct v4l2_mbus_framefmt *fmt)
{
	/* Get the crop bounds to clamp the crop rectangle correctly */
	struct v4l2_rect r = {
		.left = 0,
		.top = 0,
		.width = fmt->width,
		.height = fmt->height,
	};

	return r;
}

static void dcmipp_byteproc_adjust_crop(struct v4l2_rect *r, struct v4l2_rect *compose)
{
	/* Disallow rectangles smaller than the minimal one. */
	v4l2_rect_set_min_size(r, &crop_min);
	v4l2_rect_map_inside(r, compose);
}

static void dcmipp_byteproc_adjust_compose(struct v4l2_rect *r,
					   const struct v4l2_mbus_framefmt *fmt)
{
	r->top = 0;
	r->left = 0;

	/* Compose is not possible for JPEG or Bayer formats */
	if (fmt->code == MEDIA_BUS_FMT_JPEG_1X8 ||
	    fmt->code == MEDIA_BUS_FMT_SBGGR8_1X8 || fmt->code == MEDIA_BUS_FMT_SGBRG8_1X8 ||
	    fmt->code == MEDIA_BUS_FMT_SGRBG8_1X8 || fmt->code == MEDIA_BUS_FMT_SRGGB8_1X8) {
		r->width = fmt->width;
		r->height = fmt->height;
		return;
	}

	/* Adjust height - we can only perform 1/2 decimation */
	if (r->height <= (fmt->height / 2))
		r->height = fmt->height / 2;
	else
		r->height = fmt->height;

	/* Adjust width - /2 or /4 for 8bits formats and /2 for 16bits formats */
	if (fmt->code == MEDIA_BUS_FMT_Y8_1X8 && r->width <= (fmt->width / 4))
		r->width = fmt->width / 4;
	else if (r->width <= (fmt->width / 2))
		r->width = fmt->width / 2;
	else
		r->width = fmt->width;
}

static void dcmipp_byteproc_adjust_fmt(struct v4l2_mbus_framefmt *fmt)
{
	const struct dcmipp_byteproc_pix_map *vpix;

	/* Only accept code in the pix map table */
	vpix = dcmipp_byteproc_pix_map_by_code(fmt->code);
	if (!vpix)
		fmt->code = fmt_default.code;

	fmt->width = clamp_t(u32, fmt->width, DCMIPP_FRAME_MIN_WIDTH,
			     DCMIPP_FRAME_MAX_WIDTH) & ~1;
	fmt->height = clamp_t(u32, fmt->height, DCMIPP_FRAME_MIN_HEIGHT,
			      DCMIPP_FRAME_MAX_HEIGHT) & ~1;

	if (fmt->field == V4L2_FIELD_ANY || fmt->field == V4L2_FIELD_ALTERNATE)
		fmt->field = fmt_default.field;

	dcmipp_colorimetry_clamp(fmt);
}

static int dcmipp_byteproc_init_cfg(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state)
{
	unsigned int i;

	for (i = 0; i < sd->entity.num_pads; i++) {
		struct v4l2_mbus_framefmt *mf;
		struct v4l2_rect *r;

		mf = v4l2_subdev_get_try_format(sd, sd_state, i);
		*mf = fmt_default;

		if (IS_SINK(i))
			r = v4l2_subdev_get_try_compose(sd, sd_state, i);
		else
			r = v4l2_subdev_get_try_crop(sd, sd_state, i);

		r->top = 0;
		r->left = 0;
		r->width = DCMIPP_FMT_WIDTH_DEFAULT;
		r->height = DCMIPP_FMT_HEIGHT_DEFAULT;
	}

	return 0;
}

static int dcmipp_byteproc_enum_mbus_code(struct v4l2_subdev *sd,
					  struct v4l2_subdev_state *sd_state,
					  struct v4l2_subdev_mbus_code_enum *code)
{
	const struct dcmipp_byteproc_pix_map *vpix;

	vpix = dcmipp_byteproc_pix_map_by_index(code->index);
	if (!vpix)
		return -EINVAL;

	code->code = vpix->code;

	return 0;
}

static int dcmipp_byteproc_enum_frame_size(struct v4l2_subdev *sd,
					   struct v4l2_subdev_state *sd_state,
					   struct v4l2_subdev_frame_size_enum *fse)
{
	const struct dcmipp_byteproc_pix_map *vpix;

	if (fse->index)
		return -EINVAL;

	/* Only accept code in the pix map table */
	vpix = dcmipp_byteproc_pix_map_by_code(fse->code);
	if (!vpix)
		return -EINVAL;

	fse->min_width = DCMIPP_FRAME_MIN_WIDTH;
	fse->max_width = DCMIPP_FRAME_MAX_WIDTH;
	fse->min_height = DCMIPP_FRAME_MIN_HEIGHT;
	fse->max_height = DCMIPP_FRAME_MAX_HEIGHT;

	return 0;
}

static int dcmipp_byteproc_get_fmt(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);
	struct v4l2_rect *crop_rect;

	mutex_lock(&byteproc->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		fmt->format = *v4l2_subdev_get_try_format(sd, sd_state, 0);
		crop_rect = v4l2_subdev_get_try_crop(sd, sd_state, 1);
	} else {
		fmt->format = byteproc->sink_fmt;
		crop_rect = &byteproc->crop;
	}

	if (IS_SRC(fmt->pad)) {
		fmt->format.width = crop_rect->width;
		fmt->format.height = crop_rect->height;
	}

	mutex_unlock(&byteproc->lock);

	return 0;
}

static int dcmipp_byteproc_set_fmt(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_rect *crop, *compose;
	int ret = 0;

	mutex_lock(&byteproc->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (byteproc->streaming) {
			ret = -EBUSY;
			goto out;
		}

		sink_fmt = &byteproc->sink_fmt;
		crop = &byteproc->crop;
		compose = &byteproc->compose;
	} else {
		sink_fmt = v4l2_subdev_get_try_format(sd, sd_state, 0);
		crop = v4l2_subdev_get_try_crop(sd, sd_state, 1);
		compose = v4l2_subdev_get_try_compose(sd, sd_state, 0);
	}

	if (IS_SRC(fmt->pad)) {
		fmt->format = *sink_fmt;
		fmt->format.width = crop->width;
		fmt->format.height = crop->height;
	} else {
		dcmipp_byteproc_adjust_fmt(&fmt->format);
		crop->top = 0;
		crop->left = 0;
		crop->width = fmt->format.width;
		crop->height = fmt->format.height;
		*compose = *crop;
		*sink_fmt = fmt->format;
	}

out:
	mutex_unlock(&byteproc->lock);

	return ret;
}

static int dcmipp_byteproc_get_selection(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *sd_state,
					 struct v4l2_subdev_selection *s)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_rect *crop, *compose;

	/*
	 * In the HW, the decimation block is located prior to the crop hence:
	 * Compose is done on the sink pad
	 * Crop is done on the src pad
	 */
	if ((s->target == V4L2_SEL_TGT_CROP ||
	     s->target == V4L2_SEL_TGT_CROP_BOUNDS ||
	     s->target == V4L2_SEL_TGT_CROP_DEFAULT) && IS_SINK(s->pad))
		return -EINVAL;

	if ((s->target == V4L2_SEL_TGT_COMPOSE ||
	     s->target == V4L2_SEL_TGT_COMPOSE_BOUNDS ||
	     s->target == V4L2_SEL_TGT_COMPOSE_DEFAULT) && IS_SRC(s->pad))
		return -EINVAL;

	if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sink_fmt = &byteproc->sink_fmt;
		crop = &byteproc->crop;
		compose = &byteproc->compose;
	} else {
		sink_fmt = v4l2_subdev_get_try_format(sd, sd_state, 0);
		crop = v4l2_subdev_get_try_crop(sd, sd_state, 1);
		compose = v4l2_subdev_get_try_compose(sd, sd_state, 0);
	}

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		s->r = *crop;
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r = *compose;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		s->r = *compose;
		break;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		s->r = dcmipp_byteproc_get_compose_bound(sink_fmt);
		break;
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		s->r.top = 0;
		s->r.left = 0;
		s->r.width = sink_fmt->width;
		s->r.height = sink_fmt->height;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int dcmipp_byteproc_set_selection(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *sd_state,
					 struct v4l2_subdev_selection *s)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_rect *crop, *compose;

	/*
	 * In the HW, the decimation block is located prior to the crop hence:
	 * Compose is done on the sink pad
	 * Crop is done on the src pad
	 */
	if ((s->target == V4L2_SEL_TGT_CROP ||
	     s->target == V4L2_SEL_TGT_CROP_BOUNDS ||
	     s->target == V4L2_SEL_TGT_CROP_DEFAULT) && IS_SINK(s->pad))
		return -EINVAL;

	if ((s->target == V4L2_SEL_TGT_COMPOSE ||
	     s->target == V4L2_SEL_TGT_COMPOSE_BOUNDS ||
	     s->target == V4L2_SEL_TGT_COMPOSE_DEFAULT) && IS_SRC(s->pad))
		return -EINVAL;

	if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sink_fmt = &byteproc->sink_fmt;
		crop = &byteproc->crop;
		compose = &byteproc->compose;
	} else {
		sink_fmt = v4l2_subdev_get_try_format(sd, sd_state, 0);
		crop = v4l2_subdev_get_try_crop(sd, sd_state, 1);
		compose = v4l2_subdev_get_try_compose(sd, sd_state, 0);
	}

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		dcmipp_byteproc_adjust_crop(&s->r, compose);

		*crop = s->r;

		dev_dbg(byteproc->dev, "s_selection: crop %ux%u@(%u,%u)\n",
			crop->width, crop->height, crop->left, crop->top);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		dcmipp_byteproc_adjust_compose(&s->r, sink_fmt);
		*compose = s->r;
		*crop = s->r;

		dev_dbg(byteproc->dev, "s_selection: compose %ux%u@(%u,%u)\n",
			compose->width, compose->height, compose->left, compose->top);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const unsigned int dcmipp_frates[] = {1, 2, 4, 8};

static int dcmipp_byteproc_enum_frame_interval
				(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_frame_interval_enum *fie)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);
	struct v4l2_fract *sink_interval = &byteproc->sink_interval;
	unsigned int ratio;
	int ret = 0;

	if (fie->pad > 1 ||
	    fie->index >= (IS_SRC(fie->pad) ? ARRAY_SIZE(dcmipp_frates) : 1) ||
	    fie->width > DCMIPP_FRAME_MAX_WIDTH ||
	    fie->height > DCMIPP_FRAME_MAX_HEIGHT)
		return -EINVAL;

	mutex_lock(&byteproc->lock);

	if (IS_SINK(fie->pad)) {
		fie->interval = *sink_interval;
		goto out;
	}

	ratio = dcmipp_frates[fie->index];

	fie->interval.numerator = sink_interval->numerator * ratio;
	fie->interval.denominator = sink_interval->denominator;

out:
	mutex_unlock(&byteproc->lock);
	return ret;
}

static const struct v4l2_subdev_pad_ops dcmipp_byteproc_pad_ops = {
	.init_cfg		= dcmipp_byteproc_init_cfg,
	.enum_mbus_code		= dcmipp_byteproc_enum_mbus_code,
	.enum_frame_size	= dcmipp_byteproc_enum_frame_size,
	.enum_frame_interval	= dcmipp_byteproc_enum_frame_interval,
	.get_fmt		= dcmipp_byteproc_get_fmt,
	.set_fmt		= dcmipp_byteproc_set_fmt,
	.get_selection		= dcmipp_byteproc_get_selection,
	.set_selection		= dcmipp_byteproc_set_selection,
};

static int dcmipp_byteproc_configure_scale_crop
			(struct dcmipp_byteproc_device *byteproc)
{
	const struct dcmipp_byteproc_pix_map *vpix;
	u32 hprediv, vprediv;
	struct v4l2_rect *crop = &byteproc->crop;
	u32 val = 0;

	/* find output format bpp */
	vpix = dcmipp_byteproc_pix_map_by_code(byteproc->sink_fmt.code);
	if (!vpix)
		return -EINVAL;

	/* clear decimation/crop */
	reg_clear(byteproc, DCMIPP_P0PPCR, DCMIPP_P0PPCR_BSM_MASK);
	reg_clear(byteproc, DCMIPP_P0PPCR, DCMIPP_P0PPCR_LSM);
	reg_write(byteproc, DCMIPP_P0SCSTR, 0);
	reg_write(byteproc, DCMIPP_P0SCSZR, 0);

	/* Ignore decimation/crop with JPEG */
	if (vpix->code == MEDIA_BUS_FMT_JPEG_1X8)
		return 0;

	/* decimation */
	hprediv = byteproc->sink_fmt.width / byteproc->compose.width;
	if (hprediv == 4)
		val |= DCMIPP_P0PPCR_BSM_1_4 << DCMIPP_P0PPCR_BSM_SHIFT;
	else if ((vpix->code == MEDIA_BUS_FMT_Y8_1X8) && (hprediv == 2))
		val |= DCMIPP_P0PPCR_BSM_1_2 << DCMIPP_P0PPCR_BSM_SHIFT;
	else if (hprediv == 2)
		val |= DCMIPP_P0PPCR_BSM_2_4 << DCMIPP_P0PPCR_BSM_SHIFT;

	vprediv = byteproc->sink_fmt.height / byteproc->compose.height;
	if (vprediv == 2)
		val |= DCMIPP_P0PPCR_LSM | DCMIPP_P0PPCR_OELS;

	/* decimate using bytes and lines skipping */
	if (val) {
		reg_set(byteproc, DCMIPP_P0PPCR, val);

		dev_dbg(byteproc->dev, "decimate to %dx%d [prediv=%dx%d]\n",
			byteproc->compose.width, byteproc->compose.height, hprediv, vprediv);
	}

	dev_dbg(byteproc->dev, "crop to %dx%d\n", crop->width, crop->height);

	/* expressed in 32-bits words on X axis, lines on Y axis */
	reg_write(byteproc, DCMIPP_P0SCSTR,
		  (((crop->left * vpix->bpp) / 4) << DCMIPP_P0SCSTR_HSTART_SHIFT) |
		  (crop->top << DCMIPP_P0SCSTR_VSTART_SHIFT));
	reg_write(byteproc, DCMIPP_P0SCSZR,
		  DCMIPP_P0SCSZR_ENABLE |
		  (((crop->width * vpix->bpp) / 4) << DCMIPP_P0SCSZR_HSIZE_SHIFT) |
		  (crop->height << DCMIPP_P0SCSZR_VSIZE_SHIFT));

	return 0;
}

static void dcmipp_byteproc_configure_framerate
			(struct dcmipp_byteproc_device *byteproc)
{
	/* Frame skipping */
	reg_clear(byteproc, DCMIPP_P0FCTCR, DCMIPP_P0FCTCR_FRATE_MASK);
	reg_set(byteproc, DCMIPP_P0FCTCR, byteproc->frate);
}

static int dcmipp_byteproc_g_frame_interval(struct v4l2_subdev *sd,
					    struct v4l2_subdev_frame_interval *fi)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);

	if (IS_SINK(fi->pad))
		fi->interval = byteproc->sink_interval;
	else
		fi->interval = byteproc->src_interval;

	return 0;
}

static int dcmipp_byteproc_s_frame_interval(struct v4l2_subdev *sd,
					    struct v4l2_subdev_frame_interval *fi)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);

	mutex_lock(&byteproc->lock);

	if (byteproc->streaming) {
		mutex_unlock(&byteproc->lock);
		return -EBUSY;
	}

	if (fi->interval.numerator == 0 || fi->interval.denominator == 0)
		fi->interval = byteproc->sink_interval;

	if (IS_SINK(fi->pad)) {
		/*
		 * Setting sink frame interval resets frame skipping.
		 * Sink frame interval is propagated to src.
		 */
		byteproc->frate = 0;
		byteproc->sink_interval = fi->interval;
		byteproc->src_interval = byteproc->sink_interval;
	} else {
		unsigned int ratio;

		/* Normalize ratio */
		ratio = (byteproc->sink_interval.denominator *
			 fi->interval.numerator) /
			(byteproc->sink_interval.numerator *
			 fi->interval.denominator);

		/* Hardware can skip 1 frame over 2, 4 or 8 */
		byteproc->frate = ratio >= 8 ? 3 :
				  ratio >= 4 ? 2 :
				  ratio >= 2 ? 1 : 0;

		/* Adjust src frame interval to what hardware can really do */
		byteproc->src_interval.numerator =
			byteproc->sink_interval.numerator * ratio;
		byteproc->src_interval.denominator =
			byteproc->sink_interval.denominator;
	}

	mutex_unlock(&byteproc->lock);

	return 0;
}

#define STOP_TIMEOUT_US 1000
#define POLL_INTERVAL_US  50
static int dcmipp_byteproc_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&byteproc->lock);
	if (enable) {
		dcmipp_byteproc_configure_framerate(byteproc);

		ret = dcmipp_byteproc_configure_scale_crop(byteproc);
		if (ret)
			goto err;
	}

err:
	mutex_unlock(&byteproc->lock);

	return ret;
}

static const struct v4l2_subdev_video_ops dcmipp_byteproc_video_ops = {
	.g_frame_interval = dcmipp_byteproc_g_frame_interval,
	.s_frame_interval = dcmipp_byteproc_s_frame_interval,
	.s_stream = dcmipp_byteproc_s_stream,
};

static const struct v4l2_subdev_ops dcmipp_byteproc_ops = {
	.pad = &dcmipp_byteproc_pad_ops,
	.video = &dcmipp_byteproc_video_ops,
};

/* FIXME */
static void dcmipp_byteproc_release(struct v4l2_subdev *sd)
{
	struct dcmipp_byteproc_device *byteproc = v4l2_get_subdevdata(sd);

	kfree(byteproc);
}

static const struct v4l2_subdev_internal_ops dcmipp_byteproc_int_ops = {
	.release = dcmipp_byteproc_release,
};

static void dcmipp_byteproc_comp_unbind(struct device *comp,
					struct device *master,
					void *master_data)
{
	struct dcmipp_ent_device *ved = dev_get_drvdata(comp);
	struct dcmipp_byteproc_device *byteproc =
			container_of(ved, struct dcmipp_byteproc_device, ved);

	dcmipp_ent_sd_unregister(ved, &byteproc->sd);
}

static int dcmipp_byteproc_comp_bind(struct device *comp, struct device *master,
				     void *master_data)
{
	struct dcmipp_bind_data *bind_data = master_data;
	struct dcmipp_platform_data *pdata = comp->platform_data;
	struct dcmipp_byteproc_device *byteproc;
	struct v4l2_rect r = {
		.top = 0,
		.left = 0,
		.width = DCMIPP_FMT_WIDTH_DEFAULT,
		.height = DCMIPP_FMT_HEIGHT_DEFAULT,
	};
	struct v4l2_fract interval = {
		.numerator = 1,
		.denominator = 30,
	};
	int ret;

	/* Allocate the byteproc struct */
	byteproc = kzalloc(sizeof(*byteproc), GFP_KERNEL);
	if (!byteproc)
		return -ENOMEM;

	byteproc->regs = bind_data->regs;

	/* Initialize the lock */
	mutex_init(&byteproc->lock);

	/* Initialize ved and sd */
	ret = dcmipp_ent_sd_register(&byteproc->ved, &byteproc->sd,
				     bind_data->v4l2_dev,
				     pdata->entity_name,
				     MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER, 2,
				     (const unsigned long[2]) {
				     MEDIA_PAD_FL_SINK,
				     MEDIA_PAD_FL_SOURCE,
				     },
				     &dcmipp_byteproc_int_ops,
				     &dcmipp_byteproc_ops,
				     NULL, NULL);
	if (ret) {
		kfree(byteproc);
		return ret;
	}

	dev_set_drvdata(comp, &byteproc->ved);
	byteproc->dev = comp;

	/* Initialize the frame format */
	byteproc->sink_fmt = fmt_default;
	byteproc->crop = r;
	byteproc->compose = r;
	byteproc->src_interval = interval;
	byteproc->sink_interval = interval;

	return 0;
}

static const struct component_ops dcmipp_byteproc_comp_ops = {
	.bind = dcmipp_byteproc_comp_bind,
	.unbind = dcmipp_byteproc_comp_unbind,
};

static int dcmipp_byteproc_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &dcmipp_byteproc_comp_ops);
}

static int dcmipp_byteproc_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dcmipp_byteproc_comp_ops);

	return 0;
}

static const struct platform_device_id dcmipp_byteproc_driver_ids[] = {
	{
		.name           = DCMIPP_BYTEPROC_DRV_NAME,
	},
	{ }
};

static struct platform_driver dcmipp_byteproc_pdrv = {
	.probe		= dcmipp_byteproc_probe,
	.remove		= dcmipp_byteproc_remove,
	.id_table	= dcmipp_byteproc_driver_ids,
	.driver		= {
		.name	= DCMIPP_BYTEPROC_DRV_NAME,
	},
};

module_platform_driver(dcmipp_byteproc_pdrv);

MODULE_DEVICE_TABLE(platform, dcmipp_byteproc_driver_ids);

MODULE_AUTHOR("Hugues Fruchet <hugues.fruchet@foss.st.com>");
MODULE_AUTHOR("Alain Volmat <alain.volmat@foss.st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 Digital Camera Memory Interface with Pixel Processor driver");
MODULE_LICENSE("GPL");
