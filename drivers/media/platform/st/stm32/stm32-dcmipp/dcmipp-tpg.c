// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for STM32 Digital Camera Memory Interface Pixel Processor
 *
 * Copyright (C) STMicroelectronics SA 2024
 * Authors: Alain Volmat <alain.volmat@foss.st.com>
 *          for STMicroelectronics.
 */

#include <linux/clk.h>
#include <linux/minmax.h>
#include <linux/pm_runtime.h>
#include <linux/vmalloc.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>

#include "dcmipp-common.h"

#define DCMIPP_CMTPGCR1	0x210
#define DCMIPP_CMTPGCR1_WIDTH_SHIFT	0
#define DCMIPP_CMTPGCR1_HEIGHT_SHIFT	16
#define DCMIPP_CMTPGCR2	0x214
#define DCMIPP_CMTPGCR2_TPGEN	BIT(0)
#define DCMIPP_CMTPGCR2_LFLEN	BIT(1)
#define DCMIPP_CMTPGCR2_GSEN	BIT(2)
#define DCMIPP_CMTPGCR2_PATTERN	BIT(3)
#define DCMIPP_CMTPGCR2_FORMAT_SHIFT	8
#define DCMIPP_CMTPGCR2_FMT_YUV422	(0x1e << DCMIPP_CMTPGCR2_FORMAT_SHIFT)
#define DCMIPP_CMTPGCR2_FMT_RGB565	(0x22 << DCMIPP_CMTPGCR2_FORMAT_SHIFT)
#define DCMIPP_CMTPGCR2_FMT_RGB888	(0x24 << DCMIPP_CMTPGCR2_FORMAT_SHIFT)
#define DCMIPP_CMTPGCR2_FMT_RAW8	(0x2a << DCMIPP_CMTPGCR2_FORMAT_SHIFT)
#define DCMIPP_CMTPGCR2_FMT_RAW10	(0x2b << DCMIPP_CMTPGCR2_FORMAT_SHIFT)
#define DCMIPP_CMTPGCR2_FMT_RAW12	(0x2c << DCMIPP_CMTPGCR2_FORMAT_SHIFT)
#define DCMIPP_CMTPGCR2_FMT_RAW14	(0x2d << DCMIPP_CMTPGCR2_FORMAT_SHIFT)
#define DCMIPP_CMTPGCR2_FMT_Y8		(0x4a << DCMIPP_CMTPGCR2_FORMAT_SHIFT)
#define DCMIPP_CMTPGCR2_FMT_Y10		(0x4b << DCMIPP_CMTPGCR2_FORMAT_SHIFT)
#define DCMIPP_CMTPGCR2_FMT_Y12		(0x4c << DCMIPP_CMTPGCR2_FORMAT_SHIFT)
#define DCMIPP_CMTPGCR2_FMT_Y14		(0x4d << DCMIPP_CMTPGCR2_FORMAT_SHIFT)
#define DCMIPP_CMTPGCR2_RT_SHIFT	6
#define DCMIPP_CMTPGCR2_RT_RGGB		(0 << DCMIPP_CMTPGCR2_RT_SHIFT)
#define DCMIPP_CMTPGCR2_RT_GRBG		(1 << DCMIPP_CMTPGCR2_RT_SHIFT)
#define DCMIPP_CMTPGCR2_RT_GBRG		(2 << DCMIPP_CMTPGCR2_RT_SHIFT)
#define DCMIPP_CMTPGCR2_RT_BGGR		(3 << DCMIPP_CMTPGCR2_RT_SHIFT)
#define DCMIPP_CMTPGCR2_YT_BT601	0
#define DCMIPP_CMTPGCR2_YT_BT709	BIT(4)
#define DCMIPP_CMTPGCR2_YT_FULL		0
#define DCMIPP_CMTPGCR2_YT_REDUCED	BIT(5)
#define DCMIPP_CMTPGCR2_VBL_SHIFT	16

#define DCMIPP_TPG_MIN_WIDTH	16
#define DCMIPP_TPG_MAX_WIDTH	16383
#define DCMIPP_TPG_MIN_HEIGHT	2
#define DCMIPP_TPG_MAX_HEIGHT	16383

struct dcmipp_tpg_pix_map {
	unsigned int code;
	unsigned int tpg_format;
};

#define PIXMAP_MBUS_TPG(mbus, tpg_fmt)		\
	{						\
		.code = MEDIA_BUS_FMT_##mbus,		\
		.tpg_format = tpg_fmt,			\
	}
static const struct dcmipp_tpg_pix_map dcmipp_tpg_pix_map_list[] = {
	PIXMAP_MBUS_TPG(UYVY8_1X16, DCMIPP_CMTPGCR2_FMT_YUV422),
	PIXMAP_MBUS_TPG(RGB565_1X16, DCMIPP_CMTPGCR2_FMT_RGB565),
	PIXMAP_MBUS_TPG(RGB888_1X24, DCMIPP_CMTPGCR2_FMT_RGB888),
	PIXMAP_MBUS_TPG(SBGGR8_1X8, DCMIPP_CMTPGCR2_FMT_RAW8 |
				    DCMIPP_CMTPGCR2_RT_BGGR),
	PIXMAP_MBUS_TPG(SGBRG8_1X8, DCMIPP_CMTPGCR2_FMT_RAW8 |
				    DCMIPP_CMTPGCR2_RT_GBRG),
	PIXMAP_MBUS_TPG(SGRBG8_1X8, DCMIPP_CMTPGCR2_FMT_RAW8 |
				    DCMIPP_CMTPGCR2_RT_GRBG),
	PIXMAP_MBUS_TPG(SRGGB8_1X8, DCMIPP_CMTPGCR2_FMT_RAW8 |
				    DCMIPP_CMTPGCR2_RT_RGGB),
	PIXMAP_MBUS_TPG(SBGGR10_1X10, DCMIPP_CMTPGCR2_FMT_RAW10 |
				      DCMIPP_CMTPGCR2_RT_BGGR),
	PIXMAP_MBUS_TPG(SGBRG10_1X10, DCMIPP_CMTPGCR2_FMT_RAW10 |
				      DCMIPP_CMTPGCR2_RT_GBRG),
	PIXMAP_MBUS_TPG(SGRBG10_1X10, DCMIPP_CMTPGCR2_FMT_RAW10 |
				      DCMIPP_CMTPGCR2_RT_GRBG),
	PIXMAP_MBUS_TPG(SRGGB10_1X10, DCMIPP_CMTPGCR2_FMT_RAW10 |
				      DCMIPP_CMTPGCR2_RT_RGGB),
	PIXMAP_MBUS_TPG(SBGGR12_1X12, DCMIPP_CMTPGCR2_FMT_RAW12 |
				      DCMIPP_CMTPGCR2_RT_BGGR),
	PIXMAP_MBUS_TPG(SGBRG12_1X12, DCMIPP_CMTPGCR2_FMT_RAW12 |
				      DCMIPP_CMTPGCR2_RT_GBRG),
	PIXMAP_MBUS_TPG(SGRBG12_1X12, DCMIPP_CMTPGCR2_FMT_RAW12 |
				      DCMIPP_CMTPGCR2_RT_GRBG),
	PIXMAP_MBUS_TPG(SRGGB12_1X12, DCMIPP_CMTPGCR2_FMT_RAW12 |
				      DCMIPP_CMTPGCR2_RT_RGGB),
	PIXMAP_MBUS_TPG(SBGGR14_1X14, DCMIPP_CMTPGCR2_FMT_RAW14 |
				      DCMIPP_CMTPGCR2_RT_BGGR),
	PIXMAP_MBUS_TPG(SGBRG14_1X14, DCMIPP_CMTPGCR2_FMT_RAW14 |
				      DCMIPP_CMTPGCR2_RT_GBRG),
	PIXMAP_MBUS_TPG(SGRBG14_1X14, DCMIPP_CMTPGCR2_FMT_RAW14 |
				      DCMIPP_CMTPGCR2_RT_GRBG),
	PIXMAP_MBUS_TPG(SRGGB14_1X14, DCMIPP_CMTPGCR2_FMT_RAW14 |
				      DCMIPP_CMTPGCR2_RT_RGGB),
	PIXMAP_MBUS_TPG(Y8_1X8, DCMIPP_CMTPGCR2_FMT_Y8),
	PIXMAP_MBUS_TPG(Y10_1X10, DCMIPP_CMTPGCR2_FMT_Y10),
	PIXMAP_MBUS_TPG(Y12_1X12, DCMIPP_CMTPGCR2_FMT_Y12),
	PIXMAP_MBUS_TPG(Y14_1X14, DCMIPP_CMTPGCR2_FMT_Y14),
};

static const struct dcmipp_tpg_pix_map *dcmipp_tpg_pix_map_by_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dcmipp_tpg_pix_map_list); i++) {
		if (dcmipp_tpg_pix_map_list[i].code == code)
			return &dcmipp_tpg_pix_map_list[i];
	}

	return NULL;
}

struct dcmipp_tpg_device {
	struct dcmipp_ent_device ved;
	struct v4l2_subdev sd;
	struct device *dev;
	void __iomem *regs;
	unsigned long clk_proc_rate;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *test_pattern;
	bool streaming;
};

static const struct v4l2_mbus_framefmt fmt_default = {
	.width = DCMIPP_FMT_WIDTH_DEFAULT,
	.height = DCMIPP_FMT_HEIGHT_DEFAULT,
	.code = MEDIA_BUS_FMT_RGB565_1X16,
	.field = V4L2_FIELD_NONE,
	.colorspace = DCMIPP_COLORSPACE_DEFAULT,
	.ycbcr_enc = DCMIPP_YCBCR_ENC_DEFAULT,
	.quantization = DCMIPP_QUANTIZATION_DEFAULT,
	.xfer_func = DCMIPP_XFER_FUNC_DEFAULT,
};

static int
dcmipp_tpg_enum_mbus_code(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_mbus_code_enum *code)
{
	const struct dcmipp_tpg_pix_map *vpix;

	if (code->index >= ARRAY_SIZE(dcmipp_tpg_pix_map_list))
		return -EINVAL;

	vpix = &dcmipp_tpg_pix_map_list[code->index];
	code->code = vpix->code;

	return 0;
}

static int
dcmipp_tpg_enum_frame_size(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index)
		return -EINVAL;

	fse->min_width = DCMIPP_TPG_MIN_WIDTH;
	fse->max_width = DCMIPP_TPG_MAX_WIDTH;
	fse->min_height = DCMIPP_TPG_MIN_HEIGHT;
	fse->max_height = DCMIPP_TPG_MAX_HEIGHT;

	return 0;
}

/* Formula to compute horizontal blanking */
#define DCMIPP_HBLANK(width)	(max_t(u32, 16, (width) / 8) + 1)

static int dcmipp_tpg_set_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct dcmipp_tpg_device *tpg = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *mbus_fmt = &fmt->format;
	const struct dcmipp_tpg_pix_map *vpix;
	u64 pixel_rate, hblank, vblank;

	if (tpg->streaming)
		return -EBUSY;

	/* Only accept code in the pix map table */
	vpix = dcmipp_tpg_pix_map_by_code(mbus_fmt->code);
	if (!vpix)
		mbus_fmt->code = fmt_default.code;

	mbus_fmt->width = clamp_t(u32, mbus_fmt->width, DCMIPP_TPG_MIN_WIDTH,
				  DCMIPP_TPG_MAX_WIDTH);
	mbus_fmt->height = clamp_t(u32, mbus_fmt->height, DCMIPP_TPG_MIN_HEIGHT,
				   DCMIPP_TPG_MAX_HEIGHT);

	if (mbus_fmt->field == V4L2_FIELD_ANY ||
	    mbus_fmt->field == V4L2_FIELD_ALTERNATE)
		mbus_fmt->field = fmt_default.field;

	dcmipp_colorimetry_clamp(mbus_fmt);

	if (fmt->which != V4L2_SUBDEV_FORMAT_TRY) {
		if (mbus_fmt->code == MEDIA_BUS_FMT_RGB888_1X24)
			pixel_rate = tpg->clk_proc_rate / 2;
		else
			pixel_rate = tpg->clk_proc_rate;
		__v4l2_ctrl_s_ctrl_int64(tpg->pixel_rate, pixel_rate);

		hblank = DCMIPP_HBLANK(mbus_fmt->width);
		__v4l2_ctrl_modify_range(tpg->hblank, hblank, hblank,
					 1, hblank);

		vblank = div_u64(pixel_rate / 30, mbus_fmt->width + hblank) -
			 mbus_fmt->height;

		__v4l2_ctrl_s_ctrl(tpg->vblank, vblank);
	}

	*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;

	return 0;
}

static const struct v4l2_subdev_pad_ops dcmipp_tpg_pad_ops = {
	.enum_mbus_code		= dcmipp_tpg_enum_mbus_code,
	.enum_frame_size	= dcmipp_tpg_enum_frame_size,
	.get_fmt		= v4l2_subdev_get_fmt,
	.set_fmt		= dcmipp_tpg_set_fmt,
};

static int dcmipp_tpg_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dcmipp_tpg_device *tpg = v4l2_get_subdevdata(sd);
	const struct dcmipp_tpg_pix_map *vpix;
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	if (!enable) {
		reg_clear(tpg, DCMIPP_CMTPGCR2, DCMIPP_CMTPGCR2_TPGEN);
		tpg->streaming = enable;
		return 0;
	}

	state = v4l2_subdev_lock_and_get_active_state(&tpg->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);
	vpix = dcmipp_tpg_pix_map_by_code(fmt->code);

	/* Set width & height */
	reg_write(tpg, DCMIPP_CMTPGCR1,
		  fmt->width << DCMIPP_CMTPGCR1_WIDTH_SHIFT |
		  fmt->height << DCMIPP_CMTPGCR1_HEIGHT_SHIFT);

	reg_write(tpg, DCMIPP_CMTPGCR2, 0x0);

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(&tpg->ctrls);
	if (ret)
		goto unlock;

	/* Set YT if format is YUV422 */
	if (fmt->code == MEDIA_BUS_FMT_UYVY8_1X16) {
		if (fmt->quantization != V4L2_QUANTIZATION_FULL_RANGE)
			reg_set(tpg, DCMIPP_CMTPGCR2,
				DCMIPP_CMTPGCR2_YT_REDUCED);
		if (fmt->ycbcr_enc == V4L2_YCBCR_ENC_709)
			reg_set(tpg, DCMIPP_CMTPGCR2,
				DCMIPP_CMTPGCR2_YT_BT709);
	}

	/* Set format & enable */
	reg_set(tpg, DCMIPP_CMTPGCR2, vpix->tpg_format | DCMIPP_CMTPGCR2_TPGEN);

	tpg->streaming = enable;

unlock:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static const struct v4l2_subdev_core_ops dcmipp_tpg_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops dcmipp_tpg_video_ops = {
	.s_stream = dcmipp_tpg_s_stream,
};

static const struct v4l2_subdev_ops dcmipp_tpg_ops = {
	.core = &dcmipp_tpg_core_ops,
	.pad = &dcmipp_tpg_pad_ops,
	.video = &dcmipp_tpg_video_ops,
};

static const u8 test_pattern_val[] = {
	DCMIPP_CMTPGCR2_PATTERN,
	DCMIPP_CMTPGCR2_GSEN | DCMIPP_CMTPGCR2_PATTERN,
	0,
	DCMIPP_CMTPGCR2_GSEN,
	DCMIPP_CMTPGCR2_PATTERN | DCMIPP_CMTPGCR2_LFLEN,
	DCMIPP_CMTPGCR2_GSEN | DCMIPP_CMTPGCR2_PATTERN | DCMIPP_CMTPGCR2_LFLEN,
	DCMIPP_CMTPGCR2_LFLEN,
	DCMIPP_CMTPGCR2_GSEN | DCMIPP_CMTPGCR2_LFLEN,
};

static int dcmipp_tpg_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dcmipp_tpg_device *tpg =
		container_of(ctrl->handler,
			     struct dcmipp_tpg_device, ctrls);
	int ret = 0;
	u32 cr2;

	if (pm_runtime_get_if_in_use(tpg->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		cr2 = reg_read(tpg, DCMIPP_CMTPGCR2) &
		      ~(0xffff << DCMIPP_CMTPGCR2_VBL_SHIFT);
		cr2 |= ctrl->val << DCMIPP_CMTPGCR2_VBL_SHIFT;
		reg_write(tpg, DCMIPP_CMTPGCR2, cr2);
		break;
	case V4L2_CID_TEST_PATTERN:
		reg_clear(tpg, DCMIPP_CMTPGCR2, DCMIPP_CMTPGCR2_LFLEN |
			  DCMIPP_CMTPGCR2_GSEN | DCMIPP_CMTPGCR2_PATTERN);
		reg_set(tpg, DCMIPP_CMTPGCR2, test_pattern_val[ctrl->val]);
		break;
	default:
		break;
	}

	pm_runtime_put(tpg->dev);

	return ret;
};

static const struct v4l2_ctrl_ops dcmipp_tpg_ctrl_ops = {
	.s_ctrl = dcmipp_tpg_s_ctrl,
};

static int dcmipp_tpg_init_state(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state)
{
	*v4l2_subdev_state_get_format(sd_state, 0) = fmt_default;

	return 0;
}

static void dcmipp_tpg_release(struct v4l2_subdev *sd)
{
	struct dcmipp_tpg_device *tpg = v4l2_get_subdevdata(sd);

	kfree(tpg);
}

static const struct v4l2_subdev_internal_ops dcmipp_tpg_int_ops = {
	.init_state = dcmipp_tpg_init_state,
	.release = dcmipp_tpg_release,
};

void dcmipp_tpg_ent_release(struct dcmipp_ent_device *ved)
{
	struct dcmipp_tpg_device *tpg =
			container_of(ved, struct dcmipp_tpg_device, ved);

	dcmipp_ent_sd_unregister(ved, &tpg->sd);
}

static const char * const test_pattern_menu[] = {
	"Color bars",
	"Grayscale bars",
	"Color squares",
	"Grayscale squares",
	"Color bars with lifeline",
	"Grayscale bars with lifeline",
	"Color squares with lifeline",
	"Grayscale squares with lifeline",
};

struct dcmipp_ent_device *
dcmipp_tpg_ent_init(const char *entity_name, struct dcmipp_device *dcmipp)
{
	struct dcmipp_tpg_device *tpg;
	const unsigned long pads_flag[] = {
		MEDIA_PAD_FL_SOURCE,
	};
	struct clk *clk_proc;
	u32 hblank = DCMIPP_HBLANK(DCMIPP_FMT_WIDTH_DEFAULT);
	u32 vblank;
	int ret;

	/* Allocate the tpg struct */
	tpg = kzalloc(sizeof(*tpg), GFP_KERNEL);
	if (!tpg)
		return ERR_PTR(-ENOMEM);

	tpg->regs = dcmipp->regs;
	tpg->dev = dcmipp->dev;

	clk_proc = devm_clk_get(dcmipp->dev, "mclk");
	if (IS_ERR(clk_proc)) {
		kfree(tpg);
		return ERR_CAST(clk_proc);
	}
	tpg->clk_proc_rate = clk_get_rate(clk_proc);

	/* Initialize controls */
	v4l2_ctrl_handler_init(&tpg->ctrls, 4);

	tpg->pixel_rate = v4l2_ctrl_new_std(&tpg->ctrls, &dcmipp_tpg_ctrl_ops,
					    V4L2_CID_PIXEL_RATE,
					    tpg->clk_proc_rate / 2,
					    tpg->clk_proc_rate,
					    1, tpg->clk_proc_rate);

	tpg->hblank = v4l2_ctrl_new_std(&tpg->ctrls, &dcmipp_tpg_ctrl_ops,
					V4L2_CID_HBLANK,
					hblank, hblank, 1, hblank);

	/* Set default VBLANK for 30fps */
	vblank = tpg->clk_proc_rate / 30 / (DCMIPP_FMT_WIDTH_DEFAULT + hblank) -
		 DCMIPP_FMT_HEIGHT_DEFAULT;
	tpg->vblank = v4l2_ctrl_new_std(&tpg->ctrls, &dcmipp_tpg_ctrl_ops,
					V4L2_CID_VBLANK, 16, 0xffff, 1,
					vblank);

	tpg->test_pattern =
		v4l2_ctrl_new_std_menu_items(&tpg->ctrls, &dcmipp_tpg_ctrl_ops,
					     V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(test_pattern_menu) - 1,
					     0, 0, test_pattern_menu);

	tpg->sd.ctrl_handler = &tpg->ctrls;
	if (tpg->ctrls.error) {
		ret = tpg->ctrls.error;
		dev_err(tpg->dev, "control initialization error %d\n", ret);
		kfree(tpg);
		return ERR_PTR(ret);
	}

	tpg->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Initialize ved and sd */
	ret = dcmipp_ent_sd_register(&tpg->ved, &tpg->sd,
				     &dcmipp->v4l2_dev, entity_name,
				     MEDIA_ENT_F_CAM_SENSOR,
				     ARRAY_SIZE(pads_flag), pads_flag,
				     &dcmipp_tpg_int_ops,
				     &dcmipp_tpg_ops,
				     NULL, NULL);
	if (ret) {
		kfree(tpg);
		return ERR_PTR(ret);
	}

	tpg->ved.dcmipp = dcmipp;

	return &tpg->ved;
}
