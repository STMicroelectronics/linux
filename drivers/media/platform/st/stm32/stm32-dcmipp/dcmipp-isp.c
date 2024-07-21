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
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/vmalloc.h>
#include <linux/v4l2-mediabus.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-event.h>
#include <media/v4l2-rect.h>
#include <media/v4l2-subdev.h>

#include "dcmipp-common.h"
#include "dcmipp-colorconv.h"

#define DCMIPP_ISP_DRV_NAME "dcmipp-isp"

#define DCMIPP_FMT_WIDTH_DEFAULT  640
#define DCMIPP_FMT_HEIGHT_DEFAULT 480

#define DCMIPP_CMSR2_P1VSYNCF BIT(18)

#define DCMIPP_P1FSCR (0x804)
#define DCMIPP_P1FSCR_PIPEDIFF BIT(18)

#define DCMIPP_P1SRCR (0x820)
#define DCMIPP_P1SRCR_LASTLINE_SHIFT 0
#define DCMIPP_P1SRCR_LASTLINE_MASK GENMASK(11, 0)
#define DCMIPP_P1SRCR_FIRSTLINEDEL_SHIFT 12
#define DCMIPP_P1SRCR_FIRSTLINEDEL_MASK GENMASK(14, 12)
#define DCMIPP_P1SRCR_CROPEN BIT(15)

#define DCMIPP_P1DECR (0x830)
#define DCMIPP_P1DECR_ENABLE BIT(0)
#define DCMIPP_P1DECR_HDEC_SHIFT 1
#define DCMIPP_P1DECR_HDEC_MASK GENMASK(2, 1)
#define DCMIPP_P1DECR_VDEC_SHIFT 3
#define DCMIPP_P1DECR_VDEC_MASK GENMASK(4, 3)

#define DCMIPP_P1DMCR (0x870)
#define DCMIPP_P1DMCR_ENABLE BIT(0)
#define DCMIPP_P1DMCR_TYPE_SHIFT 1
#define DCMIPP_P1DMCR_TYPE_MASK GENMASK(2, 1)
#define DCMIPP_P1DMCR_TYPE_RGGB 0x0
#define DCMIPP_P1DMCR_TYPE_GRBG 0x1
#define DCMIPP_P1DMCR_TYPE_GBRG 0x2
#define DCMIPP_P1DMCR_TYPE_BGGR 0x3

#define DCMIPP_P1CCCR (0x880)
#define DCMIPP_P1CCCR_ENABLE BIT(0)
#define DCMIPP_P1CCCR_TYPE_YUV 0
#define DCMIPP_P1CCCR_TYPE_RGB BIT(1)
#define DCMIPP_P1CCCR_CLAMP BIT(2)
#define DCMIPP_P1CCRR1 (0x884)
#define DCMIPP_P1CCRR2 (0x888)
#define DCMIPP_P1CCGR1 (0x88C)
#define DCMIPP_P1CCGR2 (0x890)
#define DCMIPP_P1CCBR1 (0x894)
#define DCMIPP_P1CCBR2 (0x898)

#define IS_SINK(pad) (!(pad))
#define IS_SRC(pad)  ((pad))
#define PAD_STR(pad) (IS_SRC((pad))) ? "src" : "sink"

#define ISP_MEDIA_BUS_SINK_FMT_DEFAULT MEDIA_BUS_FMT_RGB565_2X8_LE
#define ISP_MEDIA_BUS_SRC_FMT_DEFAULT MEDIA_BUS_FMT_RGB888_1X24

static const unsigned int dcmipp_isp_sink_pix_map_list[] = {
	/* RGB565 */
	MEDIA_BUS_FMT_RGB565_2X8_LE,
	/* YUV422 */
	MEDIA_BUS_FMT_YUYV8_2X8,
	MEDIA_BUS_FMT_UYVY8_2X8,
	MEDIA_BUS_FMT_YVYU8_2X8,
	MEDIA_BUS_FMT_VYUY8_2X8,
	/* GREY */
	MEDIA_BUS_FMT_Y8_1X8,
	/* Raw Bayer */
	/* Raw 8 */
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	/* Raw 10 */
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	/* Raw 12 */
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	/* Raw 14 */
	MEDIA_BUS_FMT_SBGGR14_1X14,
	MEDIA_BUS_FMT_SGBRG14_1X14,
	MEDIA_BUS_FMT_SGRBG14_1X14,
	MEDIA_BUS_FMT_SRGGB14_1X14,
};

static const unsigned int dcmipp_isp_src_pix_map_list[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_YUV8_1X24,
};

static unsigned int
dcmipp_isp_pix_map_by_index(unsigned int i, unsigned int pad)
{
	const unsigned int *l;
	unsigned int size;

	if (IS_SRC(pad)) {
		l = dcmipp_isp_src_pix_map_list;
		size = ARRAY_SIZE(dcmipp_isp_src_pix_map_list);
	} else {
		l = dcmipp_isp_sink_pix_map_list;
		size = ARRAY_SIZE(dcmipp_isp_sink_pix_map_list);
	}

	if (i >= size)
		return 0;

	return l[i];
}

static unsigned int
dcmipp_isp_pix_map_by_code(u32 code, unsigned int pad)
{
	const unsigned int *l;
	unsigned int size;
	unsigned int i;

	if (IS_SRC(pad)) {
		l = dcmipp_isp_src_pix_map_list;
		size = ARRAY_SIZE(dcmipp_isp_src_pix_map_list);
	} else {
		l = dcmipp_isp_sink_pix_map_list;
		size = ARRAY_SIZE(dcmipp_isp_sink_pix_map_list);
	}

	for (i = 0; i < size; i++) {
		if (l[i] == code)
			return code;
	}

	return 0;
}

struct dcmipp_isp_device {
	struct dcmipp_ent_device ved;
	struct v4l2_subdev sd;
	struct device *dev;
	struct v4l2_mbus_framefmt sink_fmt;
	struct v4l2_mbus_framefmt src_fmt;
	unsigned int decimation;
	struct v4l2_rect crop;
	struct v4l2_rect compose;
	bool streaming;
	/* Protect this data structure */
	struct mutex lock;

	void __iomem *regs;
};

static const struct v4l2_mbus_framefmt fmt_default = {
	.width = DCMIPP_FMT_WIDTH_DEFAULT,
	.height = DCMIPP_FMT_HEIGHT_DEFAULT,
	.code = ISP_MEDIA_BUS_SINK_FMT_DEFAULT,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_REC709,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
	.xfer_func = V4L2_XFER_FUNC_DEFAULT,
};

static inline unsigned int dcmipp_isp_compute_decimation(unsigned int orig,
							 unsigned int req)
{
	unsigned int i;

	for (i = 0; i < 4; i++) {
		if (req >= (orig / (1 << i)))
			return i;
	}

	return (i - 1);
}

static void dcmipp_isp_adjust_fmt(struct v4l2_mbus_framefmt *fmt, u32 pad)
{
	/* Only accept code in the pix map table */
	if (!dcmipp_isp_pix_map_by_code(fmt->code, pad))
		fmt->code = IS_SRC(pad) ? ISP_MEDIA_BUS_SRC_FMT_DEFAULT :
					  ISP_MEDIA_BUS_SINK_FMT_DEFAULT;

	fmt->width = clamp_t(u32, fmt->width, DCMIPP_FRAME_MIN_WIDTH,
			     DCMIPP_FRAME_MAX_WIDTH) & ~1;
	fmt->height = clamp_t(u32, fmt->height, DCMIPP_FRAME_MIN_HEIGHT,
			      DCMIPP_FRAME_MAX_HEIGHT);

	if (fmt->field == V4L2_FIELD_ANY || fmt->field == V4L2_FIELD_ALTERNATE)
		fmt->field = V4L2_FIELD_NONE;

	dcmipp_colorimetry_clamp(fmt);
}

static int dcmipp_isp_init_cfg(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state)
{
	unsigned int i;

	for (i = 0; i < sd->entity.num_pads; i++) {
		struct v4l2_mbus_framefmt *mf;

		mf = v4l2_subdev_get_try_format(sd, state, i);
		*mf = fmt_default;
		mf->code = IS_SRC(i) ? ISP_MEDIA_BUS_SRC_FMT_DEFAULT :
				       ISP_MEDIA_BUS_SINK_FMT_DEFAULT;
	}

	return 0;
}

static int dcmipp_isp_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_mbus_code_enum *code)
{
	unsigned int pix_code;

	pix_code = dcmipp_isp_pix_map_by_index(code->index, code->pad);
	if (!pix_code)
		return -EINVAL;

	code->code = pix_code;

	return 0;
}

static int dcmipp_isp_enum_frame_size(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index)
		return -EINVAL;

	/* Only accept code in the pix map table */
	if (!dcmipp_isp_pix_map_by_code(fse->code, fse->pad))
		return -EINVAL;

	fse->min_width = DCMIPP_FRAME_MIN_WIDTH;
	fse->max_width = DCMIPP_FRAME_MAX_WIDTH;
	fse->min_height = DCMIPP_FRAME_MIN_HEIGHT;
	fse->max_height = DCMIPP_FRAME_MAX_HEIGHT;

	return 0;
}

static int dcmipp_isp_get_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *fmt)
{
	struct dcmipp_isp_device *isp = v4l2_get_subdevdata(sd);

	mutex_lock(&isp->lock);

	if (IS_SINK(fmt->pad))
		fmt->format = fmt->which == V4L2_SUBDEV_FORMAT_TRY ?
			      *v4l2_subdev_get_try_format(sd, state, fmt->pad) :
			      isp->sink_fmt;
	else
		fmt->format = fmt->which == V4L2_SUBDEV_FORMAT_TRY ?
			      *v4l2_subdev_get_try_format(sd, state, fmt->pad) :
			      isp->src_fmt;

	mutex_unlock(&isp->lock);

	return 0;
}

static int dcmipp_isp_set_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *fmt)
{
	struct dcmipp_isp_device *isp = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *pad_fmt;
	int ret = 0;

	mutex_lock(&isp->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (isp->streaming) {
			ret = -EBUSY;
			goto out;
		}

		if (IS_SINK(fmt->pad))
			pad_fmt = &isp->sink_fmt;
		else
			pad_fmt = &isp->src_fmt;

	} else {
		pad_fmt = v4l2_subdev_get_try_format(sd, state, fmt->pad);
	}

	dcmipp_isp_adjust_fmt(&fmt->format, fmt->pad);

	/* When setting sink format, we have to update the src format */
	if (IS_SINK(fmt->pad)) {
		struct v4l2_mbus_framefmt *src_pad_fmt;

		if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
			src_pad_fmt = &isp->src_fmt;
		else
			src_pad_fmt = v4l2_subdev_get_try_format(sd, state, 1);

		*src_pad_fmt = fmt->format;
		if (fmt->format.code >= MEDIA_BUS_FMT_Y8_1X8 &&
		    fmt->format.code < MEDIA_BUS_FMT_SBGGR8_1X8)
			src_pad_fmt->code = MEDIA_BUS_FMT_YUV8_1X24;
		else
			src_pad_fmt->code = MEDIA_BUS_FMT_RGB888_1X24;

		dev_dbg(isp->dev, "%s: source format update: new:%dx%d (0x%x, %d, %d, %d, %d)\n",
			isp->sd.name,
			src_pad_fmt->width, src_pad_fmt->height,
			src_pad_fmt->code, src_pad_fmt->colorspace,
			src_pad_fmt->quantization,
			src_pad_fmt->xfer_func, src_pad_fmt->ycbcr_enc);
	}

	dev_dbg(isp->dev, "%s: %s format update: old:%dx%d (0x%x, %d, %d, %d, %d) new:%dx%d (0x%x, %d, %d, %d, %d)\n",
		isp->sd.name,
		PAD_STR(fmt->pad),
		/* old */
		pad_fmt->width, pad_fmt->height, pad_fmt->code,
		pad_fmt->colorspace, pad_fmt->quantization,
		pad_fmt->xfer_func, pad_fmt->ycbcr_enc,
		/* new */
		fmt->format.width, fmt->format.height, fmt->format.code,
		fmt->format.colorspace, fmt->format.quantization,
		fmt->format.xfer_func, fmt->format.ycbcr_enc);

	*pad_fmt = fmt->format;

	/* Update sink pad crop */
	if (IS_SINK(fmt->pad) && fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		isp->crop.top = 0;
		isp->crop.left = 0;
		isp->crop.width = fmt->format.width;
		isp->crop.height = fmt->format.height;

		isp->compose.top = 0;
		isp->compose.left = 0;
		isp->compose.width = fmt->format.width;
		isp->compose.height = fmt->format.height;
		isp->decimation = 0;
	}

out:
	mutex_unlock(&isp->lock);

	return ret;
}

static struct v4l2_rect
dcmipp_isp_get_crop_bound(const struct v4l2_mbus_framefmt *fmt)
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

static void dcmipp_isp_adjust_crop(struct v4l2_rect *r,
				   const struct v4l2_mbus_framefmt *fmt,
				   unsigned int fmt_width)
{
	const struct v4l2_rect src_rect =
		dcmipp_isp_get_crop_bound(fmt);
	static struct v4l2_rect crop_min = {
		.width = DCMIPP_FMT_WIDTH_DEFAULT,
		.height = 1,
		.top = 8,
		.left = 0,
	};

	/* Disallow rectangles smaller than the minimal one. */
	crop_min.width = fmt_width;
	v4l2_rect_set_min_size(r, &crop_min);
	v4l2_rect_map_inside(r, &src_rect);
}

static int dcmipp_isp_get_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_selection *s)
{
	struct dcmipp_isp_device *isp = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_rect *crop, *compose;

	if (IS_SRC(s->pad))
		return -EINVAL;

	if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sink_fmt = &isp->sink_fmt;
		crop = &isp->crop;
		compose = &isp->compose;
	} else {
		sink_fmt = v4l2_subdev_get_try_format(sd, state, s->pad);
		crop = v4l2_subdev_get_try_crop(sd, state, s->pad);
		compose = v4l2_subdev_get_try_compose(sd, state, s->pad);
	}

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		s->r = *crop;
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r = dcmipp_isp_get_crop_bound(sink_fmt);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		s->r = *compose;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int dcmipp_isp_set_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_selection *s)
{
	struct dcmipp_isp_device *isp = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink_fmt, *src_fmt;
	struct v4l2_rect *crop, *compose;
	unsigned int dec;

	if (IS_SRC(s->pad))
		return -EINVAL;

	if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sink_fmt = &isp->sink_fmt;
		src_fmt = &isp->src_fmt;
		crop = &isp->crop;
		compose = &isp->compose;
	} else {
		sink_fmt = v4l2_subdev_get_try_format(sd, state, s->pad);
		src_fmt = v4l2_subdev_get_try_format(sd, state, 1);
		crop = v4l2_subdev_get_try_crop(sd, state, s->pad);
		compose = v4l2_subdev_get_try_compose(sd, state, s->pad);
	}

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		dcmipp_isp_adjust_crop(&s->r, sink_fmt, isp->sink_fmt.width);

		*crop = s->r;

		/* When we set the crop, this impact as well the compose */
		*compose = s->r;

		src_fmt->width = s->r.width;
		src_fmt->height = s->r.height;

		if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE)
			isp->decimation = 0;

		dev_dbg(isp->dev, "s_selection: crop %ux%u@(%u,%u)\n",
			crop->width, crop->height, crop->left, crop->top);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		s->r.top = 0;
		s->r.left = 0;
		if (s->r.width > crop->width)
			s->r.width = crop->width;
		if (s->r.height > crop->height)
			s->r.height = crop->height;
		dec = dcmipp_isp_compute_decimation(crop->width, s->r.width);
		s->r.width = crop->width / (1 << dec);
		if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			isp->decimation = 0;
			isp->decimation |= (dec << DCMIPP_P1DECR_HDEC_SHIFT);
		}
		dec = dcmipp_isp_compute_decimation(crop->height, s->r.height);
		s->r.height = crop->height / (1 << dec);
		if (crop->height % (1 << dec))
			s->r.height += 1;
		if (s->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			isp->decimation |= (dec << DCMIPP_P1DECR_VDEC_SHIFT);
			if (isp->decimation)
				isp->decimation |= DCMIPP_P1DECR_ENABLE;
		}

		*compose = s->r;

		src_fmt->width = s->r.width;
		src_fmt->height = s->r.height;

		dev_dbg(isp->dev, "s_selection: compose %ux%u@(%u,%u)\n",
			s->r.width, s->r.height, s->r.left, s->r.top);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops dcmipp_isp_pad_ops = {
	.init_cfg		= dcmipp_isp_init_cfg,
	.enum_mbus_code		= dcmipp_isp_enum_mbus_code,
	.enum_frame_size	= dcmipp_isp_enum_frame_size,
	.get_fmt		= dcmipp_isp_get_fmt,
	.set_fmt		= dcmipp_isp_set_fmt,
	.get_selection		= dcmipp_isp_get_selection,
	.set_selection		= dcmipp_isp_set_selection,
};

static void dcmipp_isp_config_demosaicing(struct dcmipp_isp_device *isp)
{
	unsigned int pix_code = isp->sink_fmt.code, val = 0;

	/* Disable demosaicing */
	reg_clear(isp, DCMIPP_P1DMCR, DCMIPP_P1DMCR_ENABLE | DCMIPP_P1DMCR_TYPE_MASK);

	if (pix_code >= 0x3000 && pix_code < 0x4000) {
		dev_dbg(isp->dev, "Input is RawBayer, enable Demosaicing\n");

		if (pix_code == MEDIA_BUS_FMT_SBGGR8_1X8 ||
		    pix_code == MEDIA_BUS_FMT_SBGGR10_1X10 ||
		    pix_code == MEDIA_BUS_FMT_SBGGR12_1X12 ||
		    pix_code == MEDIA_BUS_FMT_SBGGR14_1X14)
			val = DCMIPP_P1DMCR_TYPE_BGGR << DCMIPP_P1DMCR_TYPE_SHIFT;
		else if (pix_code == MEDIA_BUS_FMT_SGBRG8_1X8 ||
			 pix_code == MEDIA_BUS_FMT_SGBRG10_1X10 ||
			 pix_code == MEDIA_BUS_FMT_SGBRG12_1X12 ||
			 pix_code == MEDIA_BUS_FMT_SGBRG14_1X14)
			val = DCMIPP_P1DMCR_TYPE_GBRG << DCMIPP_P1DMCR_TYPE_SHIFT;
		else if (pix_code == MEDIA_BUS_FMT_SGRBG8_1X8 ||
			 pix_code == MEDIA_BUS_FMT_SGRBG10_1X10 ||
			 pix_code == MEDIA_BUS_FMT_SGRBG12_1X12 ||
			 pix_code == MEDIA_BUS_FMT_SGRBG14_1X14)
			val = DCMIPP_P1DMCR_TYPE_GRBG << DCMIPP_P1DMCR_TYPE_SHIFT;
		else if (pix_code == MEDIA_BUS_FMT_SRGGB8_1X8 ||
			 pix_code == MEDIA_BUS_FMT_SRGGB10_1X10 ||
			 pix_code == MEDIA_BUS_FMT_SRGGB12_1X12 ||
			 pix_code == MEDIA_BUS_FMT_SRGGB14_1X14)
			val = DCMIPP_P1DMCR_TYPE_RGGB << DCMIPP_P1DMCR_TYPE_SHIFT;

		val |= DCMIPP_P1DMCR_ENABLE;
	}

	if (val)
		reg_set(isp, DCMIPP_P1DMCR, val);
}

static int dcmipp_isp_colorconv_set(struct dcmipp_isp_device *isp,
				    struct dcmipp_colorconv_config *ccconf)
{
	u32 val = 0;
	int i;

	for (i = 0; i < 6; i++)
		reg_write(isp, DCMIPP_P1CCRR1 + (4 * i), ccconf->conv_matrix[i]);

	if (ccconf->clamping)
		val |= DCMIPP_P1CCCR_CLAMP;
	if (ccconf->clamping_as_rgb)
		val |= DCMIPP_P1CCCR_TYPE_RGB;
	if (ccconf->enable)
		val |= DCMIPP_P1CCCR_ENABLE;

	reg_write(isp, DCMIPP_P1CCCR, val);

	return 0;
}

static int dcmipp_isp_colorconv_auto(struct dcmipp_isp_device *isp)
{
	struct dcmipp_colorconv_config ccconf;
	int ret;

	/* Get the "src to sink" color conversion matrix */
	ret = dcmipp_colorconv_configure(isp->dev, &isp->sink_fmt, &isp->src_fmt, &ccconf);
	if (ret)
		return ret;

	return dcmipp_isp_colorconv_set(isp, &ccconf);
}

static bool dcmipp_isp_is_aux_output_enabled(struct dcmipp_isp_device *isp)
{
	struct media_link *link;

	for_each_media_entity_data_link(isp->ved.ent, link) {
		if (link->source != &isp->ved.pads[1])
			continue;

		if (!(link->flags & MEDIA_LNK_FL_ENABLED))
			continue;

		if (!strcmp(link->sink->entity->name, "dcmipp_aux_postproc"))
			return true;
	}

	return false;
}

static int dcmipp_isp_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dcmipp_isp_device *isp = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&isp->lock);
	if (enable) {
		/* Check if link between ISP & Pipe2 postproc is enabled */
		if (dcmipp_isp_is_aux_output_enabled(isp))
			reg_clear(isp, DCMIPP_P1FSCR, DCMIPP_P1FSCR_PIPEDIFF);
		else
			reg_set(isp, DCMIPP_P1FSCR, DCMIPP_P1FSCR_PIPEDIFF);

		/* Configure Statistic Removal */
		reg_write(isp, DCMIPP_P1SRCR,
			  ((isp->crop.top << DCMIPP_P1SRCR_FIRSTLINEDEL_SHIFT) |
			   (isp->crop.height << DCMIPP_P1SRCR_LASTLINE_SHIFT) |
			   DCMIPP_P1SRCR_CROPEN));

		/* Configure Decimation */
		reg_write(isp, DCMIPP_P1DECR, isp->decimation);

		/* Configure Demosaicing */
		dcmipp_isp_config_demosaicing(isp);

		/* Configure ColorConversion */
		ret = dcmipp_isp_colorconv_auto(isp);
		if (ret)
			goto out;
	}

	isp->streaming = enable;

out:
	mutex_unlock(&isp->lock);

	return ret;
}

static const struct v4l2_subdev_video_ops dcmipp_isp_video_ops = {
	.s_stream = dcmipp_isp_s_stream,
};

static const struct v4l2_subdev_ops dcmipp_isp_ops = {
	.pad = &dcmipp_isp_pad_ops,
	.video = &dcmipp_isp_video_ops,
};

void dcmipp_isp_ent_release(struct dcmipp_ent_device *ved)
{
	struct dcmipp_isp_device *isp =
			container_of(ved, struct dcmipp_isp_device, ved);

	dcmipp_ent_sd_unregister(ved, &isp->sd);
	mutex_destroy(&isp->lock);
	kfree(isp);
}

struct dcmipp_ent_device *dcmipp_isp_ent_init(struct device *dev,
					      const char *entity_name,
					      struct v4l2_device *v4l2_dev,
					      void __iomem *regs)
{
	struct dcmipp_isp_device *isp;
	int ret;

	/* Allocate the isp struct */
	isp = kzalloc(sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return ERR_PTR(-ENOMEM);

	isp->regs = regs;
	isp->dev = dev;

	/* Initialize the lock */
	mutex_init(&isp->lock);

	/* Initialize the frame format */
	isp->sink_fmt = fmt_default;
	isp->sink_fmt.code = ISP_MEDIA_BUS_SINK_FMT_DEFAULT;
	isp->src_fmt = fmt_default;
	isp->src_fmt.code = ISP_MEDIA_BUS_SRC_FMT_DEFAULT;

	/* Initialize ved and sd */
	ret = dcmipp_ent_sd_register(&isp->ved, &isp->sd,
				     v4l2_dev,
				     entity_name,
				     MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER, 4,
				     (const unsigned long[4]) {
				     MEDIA_PAD_FL_SINK,
				     MEDIA_PAD_FL_SOURCE,
				     MEDIA_PAD_FL_SINK,
				     MEDIA_PAD_FL_SOURCE,
				     },
				     NULL, &dcmipp_isp_ops,
				     NULL, NULL);
	if (ret) {
		mutex_destroy(&isp->lock);
		kfree(isp);
		return ERR_PTR(ret);
	}

	return &isp->ved;
}
