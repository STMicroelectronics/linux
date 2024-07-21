// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for STM32 Digital Camera Memory Interface Pixel Processor
 *
 * Copyright (C) STMicroelectronics SA 2021
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          Alain Volmat <alain.volmat@foss.st.com>
 *          for STMicroelectronics.
 */

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/v4l2-mediabus.h>
#include <linux/vmalloc.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>

#include "dcmipp-common.h"

#define DCMIPP_INP_DRV_NAME "dcmipp-input"

#define DCMIPP_PRCR (0x104)
#define DCMIPP_PRCR_FORMAT_SHIFT 16
#define DCMIPP_PRCR_FORMAT_YUV422 0x1E
#define DCMIPP_PRCR_FORMAT_RGB565 0x22
#define DCMIPP_PRCR_FORMAT_RAW8 0x2A
#define DCMIPP_PRCR_FORMAT_RAW10 0x2B
#define DCMIPP_PRCR_FORMAT_RAW12 0x2C
#define DCMIPP_PRCR_FORMAT_RAW14 0x2D
#define DCMIPP_PRCR_FORMAT_G8 0x4A
#define DCMIPP_PRCR_FORMAT_BYTE_STREAM 0x5A
#define DCMIPP_PRCR_ESS BIT(4)
#define DCMIPP_PRCR_PCKPOL BIT(5)
#define DCMIPP_PRCR_HSPOL BIT(6)
#define DCMIPP_PRCR_VSPOL BIT(7)
#define DCMIPP_PRCR_ENABLE BIT(14)
#define DCMIPP_PRCR_SWAPCYCLES BIT(25)
#define DCMIPP_PRCR_SWAPBITS BIT(26)

#define DCMIPP_PRESCR (0x108)
#define DCMIPP_PRESUR (0x10c)

#define DCMIPP_CMCR (0x204)
#define DCMIPP_CMCR_INSEL BIT(0)

#define DCMIPP_P0FSCR (0x404)
#define DCMIPP_P0FSCR_DTMODE_MASK GENMASK(17, 16)
#define DCMIPP_P0FSCR_DTMODE_SHIFT 16
#define DCMIPP_P0FSCR_DTMODE_DTIDA	0x00
#define DCMIPP_P0FSCR_DTMODE_ALLDT	0x03
#define DCMIPP_P0FSCR_DTIDA_MASK GENMASK(5, 0)
#define DCMIPP_P0FSCR_DTIDA_SHIFT 0

#define DCMIPP_P1FSCR (0x804)
#define DCMIPP_P2FSCR (0xC04)
#define DCMIPP_PxFSCR_DTIDA_MASK GENMASK(5, 0)
#define DCMIPP_PxFSCR_DTIDA_SHIFT 0
#define DCMIPP_PxFSCR(id) (((id) == 1) ? DCMIPP_P1FSCR :\
			   DCMIPP_P2FSCR)

#define IS_SINK(pad) (!(pad))
#define IS_SRC(pad)  ((pad))

#define PAR_MEDIA_BUS_FMT_DEFAULT MEDIA_BUS_FMT_RGB565_2X8_LE

struct dcmipp_inp_pix_map {
	unsigned int code_sink;
	unsigned int code_src;
	/* Parallel related information */
	u8 prcr_format;
	u8 prcr_swapbits;
	u8 prcr_swapcycles;
	/* CSI related information */
	unsigned int dt;
};

#define PIXMAP_SINK_SRC_PRCR_SWAP(sink, src, prcr, swap, data_type)	\
		{						\
			.code_sink = MEDIA_BUS_FMT_##sink,		\
			.code_src = MEDIA_BUS_FMT_##src,		\
			.prcr_format = DCMIPP_PRCR_FORMAT_##prcr,	\
			.prcr_swapcycles = swap,		\
			.dt = data_type,			\
		}
static const struct dcmipp_inp_pix_map dcmipp_inp_pix_map_list[] = {
	/* RGB565 */
	PIXMAP_SINK_SRC_PRCR_SWAP(RGB565_2X8_LE, RGB565_2X8_LE, RGB565, 1, MIPI_CSI2_DT_RGB565),
	PIXMAP_SINK_SRC_PRCR_SWAP(RGB565_2X8_BE, RGB565_2X8_LE, RGB565, 0, MIPI_CSI2_DT_RGB565),
	/* YUV422 */
	PIXMAP_SINK_SRC_PRCR_SWAP(YUYV8_2X8, YUYV8_2X8, YUV422, 1, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_SINK_SRC_PRCR_SWAP(YUYV8_2X8, UYVY8_2X8, YUV422, 0, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_SINK_SRC_PRCR_SWAP(UYVY8_2X8, UYVY8_2X8, YUV422, 1, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_SINK_SRC_PRCR_SWAP(UYVY8_2X8, YUYV8_2X8, YUV422, 0, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_SINK_SRC_PRCR_SWAP(YVYU8_2X8, YVYU8_2X8, YUV422, 1, MIPI_CSI2_DT_YUV422_8B),
	PIXMAP_SINK_SRC_PRCR_SWAP(VYUY8_2X8, VYUY8_2X8, YUV422, 1, MIPI_CSI2_DT_YUV422_8B),
	/* GREY */
	PIXMAP_SINK_SRC_PRCR_SWAP(Y8_1X8, Y8_1X8, G8, 0, MIPI_CSI2_DT_RAW8),
	/* Raw Bayer */
	PIXMAP_SINK_SRC_PRCR_SWAP(SBGGR8_1X8, SBGGR8_1X8, RAW8, 0, MIPI_CSI2_DT_RAW8),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGBRG8_1X8, SGBRG8_1X8, RAW8, 0, MIPI_CSI2_DT_RAW8),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGRBG8_1X8, SGRBG8_1X8, RAW8, 0, MIPI_CSI2_DT_RAW8),
	PIXMAP_SINK_SRC_PRCR_SWAP(SRGGB8_1X8, SRGGB8_1X8, RAW8, 0, MIPI_CSI2_DT_RAW8),
	PIXMAP_SINK_SRC_PRCR_SWAP(SBGGR10_1X10, SBGGR10_1X10, RAW10, 0, MIPI_CSI2_DT_RAW10),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGBRG10_1X10, SGBRG10_1X10, RAW10, 0, MIPI_CSI2_DT_RAW10),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGRBG10_1X10, SGRBG10_1X10, RAW10, 0, MIPI_CSI2_DT_RAW10),
	PIXMAP_SINK_SRC_PRCR_SWAP(SRGGB10_1X10, SRGGB10_1X10, RAW10, 0, MIPI_CSI2_DT_RAW10),
	PIXMAP_SINK_SRC_PRCR_SWAP(SBGGR12_1X12, SBGGR12_1X12, RAW12, 0, MIPI_CSI2_DT_RAW12),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGBRG12_1X12, SGBRG12_1X12, RAW12, 0, MIPI_CSI2_DT_RAW12),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGRBG12_1X12, SGRBG12_1X12, RAW12, 0, MIPI_CSI2_DT_RAW12),
	PIXMAP_SINK_SRC_PRCR_SWAP(SRGGB12_1X12, SRGGB12_1X12, RAW12, 0, MIPI_CSI2_DT_RAW12),
	PIXMAP_SINK_SRC_PRCR_SWAP(SBGGR14_1X14, SBGGR14_1X14, RAW14, 0, MIPI_CSI2_DT_RAW14),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGBRG14_1X14, SGBRG14_1X14, RAW14, 0, MIPI_CSI2_DT_RAW14),
	PIXMAP_SINK_SRC_PRCR_SWAP(SGRBG14_1X14, SGRBG14_1X14, RAW14, 0, MIPI_CSI2_DT_RAW14),
	PIXMAP_SINK_SRC_PRCR_SWAP(SRGGB14_1X14, SRGGB14_1X14, RAW14, 0, MIPI_CSI2_DT_RAW14),
	/* JPEG */
	PIXMAP_SINK_SRC_PRCR_SWAP(JPEG_1X8, JPEG_1X8, BYTE_STREAM, 0, 0),
};

/*
 * Search through the pix_map table, skipping two consecutive entry with the
 * same code
 */
static inline const struct dcmipp_inp_pix_map *dcmipp_inp_pix_map_by_index
						(unsigned int index,
						 unsigned int pad)
{
	const struct dcmipp_inp_pix_map *l = dcmipp_inp_pix_map_list;
	unsigned int size = ARRAY_SIZE(dcmipp_inp_pix_map_list);
	unsigned int i = 0;
	u32 prev_code = 0, cur_code;

	while (i < size) {
		if (IS_SRC(pad))
			cur_code = l[i].code_src;
		else
			cur_code = l[i].code_sink;

		if (cur_code == prev_code) {
			i++;
			continue;
		} else {
			prev_code = cur_code;
		}

		if (index == 0)
			break;
		i++;
		index--;
	}

	if (i >= size)
		return NULL;

	return &l[i];
}

static inline const struct dcmipp_inp_pix_map *dcmipp_inp_pix_map_by_code
					(u32 code_sink, u32 code_src)
{
	const struct dcmipp_inp_pix_map *l = dcmipp_inp_pix_map_list;
	unsigned int size = ARRAY_SIZE(dcmipp_inp_pix_map_list);
	unsigned int i;

	for (i = 0; i < size; i++) {
		if ((l[i].code_sink == code_sink && l[i].code_src == code_src) ||
		    (l[i].code_sink == code_src && l[i].code_src == code_sink) ||
		    (l[i].code_sink == code_sink && code_src == 0) ||
		    (code_sink == 0 && l[i].code_src == code_src))
			return &l[i];
	}
	return NULL;
}

struct dcmipp_inp_device {
	struct dcmipp_ent_device ved;
	struct v4l2_subdev sd;
	struct device *dev;
	/* The active format */
	struct v4l2_mbus_framefmt sink_format;
	struct v4l2_mbus_framefmt src_format;
	bool streaming;
	void __iomem			*regs;
};

static const struct v4l2_mbus_framefmt fmt_default = {
	.width = DCMIPP_FMT_WIDTH_DEFAULT,
	.height = DCMIPP_FMT_HEIGHT_DEFAULT,
	.code = PAR_MEDIA_BUS_FMT_DEFAULT,
	.field = V4L2_FIELD_NONE,
	.colorspace = DCMIPP_COLORSPACE_DEFAULT,
	.ycbcr_enc = DCMIPP_YCBCR_ENC_DEFAULT,
	.quantization = DCMIPP_QUANTIZATION_DEFAULT,
	.xfer_func = DCMIPP_XFER_FUNC_DEFAULT,
};

static int dcmipp_inp_init_cfg(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *sd_state)
{
	unsigned int i;

	for (i = 0; i < sd->entity.num_pads; i++) {
		struct v4l2_mbus_framefmt *mf;

		mf = v4l2_subdev_get_try_format(sd, sd_state, i);
		*mf = fmt_default;
	}

	return 0;
}

static int dcmipp_inp_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_mbus_code_enum *code)
{
	const struct dcmipp_inp_pix_map *vpix =
		dcmipp_inp_pix_map_by_index(code->index, code->pad);

	if (!vpix)
		return -EINVAL;

	code->code = IS_SRC(code->pad) ? vpix->code_src : vpix->code_sink;

	return 0;
}

static int dcmipp_inp_enum_frame_size(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *sd_state,
				      struct v4l2_subdev_frame_size_enum *fse)
{
	const struct dcmipp_inp_pix_map *vpix;

	if (fse->index)
		return -EINVAL;

	/* Only accept code in the pix map table */
	vpix = dcmipp_inp_pix_map_by_code(IS_SINK(fse->pad) ? fse->code : 0,
					  IS_SRC(fse->pad) ? fse->code : 0);
	if (!vpix)
		return -EINVAL;

	fse->min_width = DCMIPP_FRAME_MIN_WIDTH;
	fse->max_width = DCMIPP_FRAME_MAX_WIDTH;
	fse->min_height = DCMIPP_FRAME_MIN_HEIGHT;
	fse->max_height = DCMIPP_FRAME_MAX_HEIGHT;

	return 0;
}

static int dcmipp_inp_get_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct dcmipp_inp_device *par =
				container_of(sd, struct dcmipp_inp_device, sd);

	fmt->format = fmt->which == V4L2_SUBDEV_FORMAT_TRY ?
		      *v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) :
		      (IS_SRC(fmt->pad) ? par->src_format : par->sink_format);

	return 0;
}

static void dcmipp_inp_adjust_fmt(struct dcmipp_inp_device *par,
				  struct v4l2_mbus_framefmt *fmt, __u32 pad)
{
	const struct dcmipp_inp_pix_map *vpix;

	/* Only accept code in the pix map table */
	vpix = dcmipp_inp_pix_map_by_code(IS_SINK(pad) ? fmt->code : 0,
					  IS_SRC(pad) ? fmt->code : 0);
	if (!vpix)
		fmt->code = fmt_default.code;

	/* Exclude JPEG if BT656 bus is selected */
	if (vpix && vpix->code_sink == MEDIA_BUS_FMT_JPEG_1X8 &&
	    par->ved.bus_type == V4L2_MBUS_BT656)
		fmt->code = fmt_default.code;

	fmt->width = clamp_t(u32, fmt->width, DCMIPP_FRAME_MIN_WIDTH,
			     DCMIPP_FRAME_MAX_WIDTH) & ~1;
	fmt->height = clamp_t(u32, fmt->height, DCMIPP_FRAME_MIN_HEIGHT,
			      DCMIPP_FRAME_MAX_HEIGHT) & ~1;

	if (fmt->field == V4L2_FIELD_ANY || fmt->field == V4L2_FIELD_ALTERNATE)
		fmt->field = fmt_default.field;

	dcmipp_colorimetry_clamp(fmt);
}

static int dcmipp_inp_set_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct dcmipp_inp_device *par = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *mf;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		/* Do not change the format while stream is on */
		if (par->streaming)
			return -EBUSY;

		mf = IS_SRC(fmt->pad) ? &par->src_format : &par->sink_format;
	} else {
		mf = v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
	}

	/* Set the new format */
	dcmipp_inp_adjust_fmt(par, &fmt->format, fmt->pad);

	dev_dbg(par->dev, "%s: format update: old:%dx%d (0x%x, %d, %d, %d, %d) new:%dx%d (0x%x, %d, %d, %d, %d)\n",
		par->sd.name,
		/* old */
		mf->width, mf->height, mf->code,
		mf->colorspace,	mf->quantization,
		mf->xfer_func, mf->ycbcr_enc,
		/* new */
		fmt->format.width, fmt->format.height, fmt->format.code,
		fmt->format.colorspace, fmt->format.quantization,
		fmt->format.xfer_func, fmt->format.ycbcr_enc);

	*mf = fmt->format;

	/* When setting the sink format, report that format on the src pad as well */
	if (IS_SINK(fmt->pad))
		par->src_format = fmt->format;

	return 0;
}

static const struct v4l2_subdev_pad_ops dcmipp_inp_pad_ops = {
	.init_cfg		= dcmipp_inp_init_cfg,
	.enum_mbus_code		= dcmipp_inp_enum_mbus_code,
	.enum_frame_size	= dcmipp_inp_enum_frame_size,
	.get_fmt		= dcmipp_inp_get_fmt,
	.set_fmt		= dcmipp_inp_set_fmt,
};

static int dcmipp_inp_configure_parallel(struct dcmipp_inp_device *par,
					 int enable)
{
	u32 val = 0;
	const struct dcmipp_inp_pix_map *vpix;

	if (!enable) {
		/* Disable parallel interface */
		reg_clear(par, DCMIPP_PRCR, DCMIPP_PRCR_ENABLE);

		return 0;
	}

	/* Set vertical synchronization polarity */
	if (par->ved.bus.flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH)
		val |= DCMIPP_PRCR_VSPOL;

	/* Set horizontal synchronization polarity */
	if (par->ved.bus.flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH)
		val |= DCMIPP_PRCR_HSPOL;

	/* Set pixel clock polarity */
	if (par->ved.bus.flags & V4L2_MBUS_PCLK_SAMPLE_RISING)
		val |= DCMIPP_PRCR_PCKPOL;

	/*
	 * BT656 embedded synchronisation bus mode.
	 *
	 * Default SAV/EAV mode is supported here with default codes
	 * SAV=0xff000080 & EAV=0xff00009d.
	 * With DCMIPP this means LSC=SAV=0x80 & LEC=EAV=0x9d.
	 */
	if (par->ved.bus_type == V4L2_MBUS_BT656) {
		val |= DCMIPP_PRCR_ESS;

		/* Unmask all codes */
		reg_write(par, DCMIPP_PRESUR, 0xffffffff);/* FEC:LEC:LSC:FSC */

		/* Trig on LSC=0x80 & LEC=0x9d codes, ignore FSC and FEC */
		reg_write(par, DCMIPP_PRESCR, 0xff9d80ff);/* FEC:LEC:LSC:FSC */
	}

	/* Set format */
	vpix = dcmipp_inp_pix_map_by_code(par->sink_format.code,
					  par->src_format.code);
	if (!vpix) {
		dev_err(par->dev, "Invalid sink/src format configuration\n");
		return -EINVAL;
	}

	val |= vpix->prcr_format << DCMIPP_PRCR_FORMAT_SHIFT;

	/* swap LSB vs MSB within one cycle */
	if (vpix->prcr_swapbits)
		val |= DCMIPP_PRCR_SWAPBITS;

	/* swap cycles */
	if (vpix->prcr_swapcycles)
		val |= DCMIPP_PRCR_SWAPCYCLES;

	reg_write(par, DCMIPP_PRCR, val);

	/* Select the DCMIPP parallel interface */
	reg_write(par, DCMIPP_CMCR, 0);

	/* Enable parallel interface */
	reg_set(par, DCMIPP_PRCR, DCMIPP_PRCR_ENABLE);

	return 0;
}

static int dcmipp_inp_configure_csi(struct dcmipp_inp_device *par, int enable)
{
	const struct dcmipp_inp_pix_map *vpix;

	if (!enable)
		return 0;

	/* Get format information */
	vpix = dcmipp_inp_pix_map_by_code(par->sink_format.code,
					  par->src_format.code);
	if (!vpix) {
		dev_err(par->dev, "Invalid sink/src format configuration\n");
		return -EINVAL;
	}

	/* Apply configuration on each input pipe */
	/* Pipe #0 */
	reg_clear(par, DCMIPP_P0FSCR,
		  DCMIPP_P0FSCR_DTMODE_MASK | DCMIPP_P0FSCR_DTIDA_MASK);

	/* If DT is unknown (such as for JPEG) we allow all data */
	if (!vpix->dt)
		reg_set(par, DCMIPP_P0FSCR,
			DCMIPP_P0FSCR_DTMODE_ALLDT << DCMIPP_P0FSCR_DTMODE_SHIFT);
	else
		reg_set(par, DCMIPP_P0FSCR,
			vpix->dt << DCMIPP_P0FSCR_DTIDA_SHIFT |
			DCMIPP_P0FSCR_DTMODE_DTIDA);

	if (of_device_is_compatible(par->dev->of_node, "st,stm32mp25-dcmipp")) {
		/* Pipe #1 */
		reg_clear(par, DCMIPP_P1FSCR, DCMIPP_PxFSCR_DTIDA_MASK);
		reg_set(par, DCMIPP_P1FSCR,
			vpix->dt << DCMIPP_PxFSCR_DTIDA_SHIFT);

		/* Only configure Pipe #2 input if is enabled */
		if (media_pad_remote_pad_first(&par->ved.pads[3])) {
			reg_clear(par, DCMIPP_P2FSCR, DCMIPP_PxFSCR_DTIDA_MASK);
			reg_set(par, DCMIPP_P2FSCR,
				vpix->dt << DCMIPP_PxFSCR_DTIDA_SHIFT);
		}
	}

	/* Select the DCMIPP CSI interface */
	reg_write(par, DCMIPP_CMCR, DCMIPP_CMCR_INSEL);

	return 0;
}

static int dcmipp_inp_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dcmipp_inp_device *par =
				container_of(sd, struct dcmipp_inp_device, sd);
	int ret = 0;

	if (par->ved.bus_type == V4L2_MBUS_PARALLEL ||
	    par->ved.bus_type == V4L2_MBUS_BT656) {
		ret = dcmipp_inp_configure_parallel(par, enable);
	} else if (par->ved.bus_type == V4L2_MBUS_CSI2_DPHY) {
		ret = dcmipp_inp_configure_csi(par, enable);
	} else {
		dev_err(par->dev, "Invalid bus_type: 0x%x, aborted\n",
			par->ved.bus_type);
		return -EINVAL;
	}

	if (!ret)
		par->streaming = enable;

	return ret;
}

static const struct v4l2_subdev_video_ops dcmipp_inp_video_ops = {
	.s_stream = dcmipp_inp_s_stream,
};

static const struct v4l2_subdev_ops dcmipp_inp_ops = {
	.pad = &dcmipp_inp_pad_ops,
	.video = &dcmipp_inp_video_ops,
};

static void dcmipp_inp_release(struct v4l2_subdev *sd)
{
	struct dcmipp_inp_device *par =
				container_of(sd, struct dcmipp_inp_device, sd);

	kfree(par);
}

static const struct v4l2_subdev_internal_ops dcmipp_inp_int_ops = {
	.release = dcmipp_inp_release,
};

void dcmipp_inp_ent_release(struct dcmipp_ent_device *ved)
{
	struct dcmipp_inp_device *par =
			container_of(ved, struct dcmipp_inp_device, ved);

	dcmipp_ent_sd_unregister(ved, &par->sd);
}

static int dcmipp_inp_get_input_bus_type(struct dcmipp_inp_device *par)
{
	struct device_node *np = par->dev->of_node;
	struct v4l2_fwnode_endpoint ep = { .bus_type = 0 };
	int ret;

	/* Get bus characteristics from devicetree */
	np = of_graph_get_next_endpoint(np, NULL);
	if (!np) {
		dev_err(par->dev, "Could not find the endpoint\n");
		of_node_put(np);
		return -ENODEV;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(np), &ep);
	of_node_put(np);
	if (ret) {
		dev_err(par->dev, "Could not parse the endpoint\n");
		return ret;
	}

	if ((ep.bus_type == V4L2_MBUS_PARALLEL ||
	     ep.bus_type == V4L2_MBUS_BT656) &&
	     ep.bus.parallel.bus_width > 0) {
		/* Only 8 bits bus width supported with BT656 bus */
		if (ep.bus_type == V4L2_MBUS_BT656 &&
		    ep.bus.parallel.bus_width != 8) {
			dev_err(par->dev, "BT656 bus conflicts with %u bits bus width (8 bits required)\n",
				ep.bus.parallel.bus_width);
			return -ENODEV;
		}

		par->ved.bus.flags = ep.bus.parallel.flags;
		par->ved.bus.bus_width = ep.bus.parallel.bus_width;
		par->ved.bus.data_shift = ep.bus.parallel.data_shift;
	}

	dev_dbg(par->dev, "DCMIPP input bus-type is: 0x%x\n", ep.bus_type);
	par->ved.bus_type = ep.bus_type;

	return 0;
}

#define DCMIPP_INP_SINK_PAD_NB_MP13	1
#define DCMIPP_INP_SINK_PAD_NB_MP25	3
struct dcmipp_ent_device *dcmipp_inp_ent_init(struct device *dev,
					      const char *entity_name,
					      struct v4l2_device *v4l2_dev,
					      void __iomem *regs)
{
	struct dcmipp_inp_device *par;
	const unsigned long pads_stm32mp25[DCMIPP_INP_SINK_PAD_NB_MP25 + 1] = {
		MEDIA_PAD_FL_SINK, MEDIA_PAD_FL_SOURCE,
		MEDIA_PAD_FL_SOURCE, MEDIA_PAD_FL_SOURCE,
	};
	u16 pads_nb = DCMIPP_INP_SINK_PAD_NB_MP25 + 1;
	int ret;

	if (of_device_is_compatible(dev->of_node, "st,stm32mp13-dcmipp"))
		pads_nb = DCMIPP_INP_SINK_PAD_NB_MP13 + 1;
	else if (of_device_is_compatible(dev->of_node, "st,stm32mp25-dcmipp"))
		pads_nb = DCMIPP_INP_SINK_PAD_NB_MP25 + 1;

	/* Allocate the par struct */
	par = kzalloc(sizeof(*par), GFP_KERNEL);
	if (!par)
		return ERR_PTR(-ENOMEM);

	par->regs = regs;

	/* Initialize ved and sd */
	ret = dcmipp_ent_sd_register
		(&par->ved, &par->sd, v4l2_dev,
		 entity_name,
		 MEDIA_ENT_F_VID_IF_BRIDGE, pads_nb,
		 pads_stm32mp25,
		 &dcmipp_inp_int_ops, &dcmipp_inp_ops,
		 NULL, NULL);
	if (ret)
		goto err_free_hdl;

	par->dev = dev;

	/* Retrieve and store the bus-type within the ent structure */
	ret = dcmipp_inp_get_input_bus_type(par);
	if (ret)
		goto err_free_hdl;

	/* Initialize the frame format */
	par->sink_format = fmt_default;
	par->src_format = fmt_default;

	return &par->ved;

err_free_hdl:
	kfree(par);

	return ERR_PTR(ret);
}
