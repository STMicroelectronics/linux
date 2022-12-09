/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Driver for STM32 Digital Camera Memory Interface Pixel Processor
 *
 * Copyright (C) STMicroelectronics SA 2021
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          Alain Volmat <alain.volmat@foss.st.com>
 *          for STMicroelectronics.
 */

#ifndef _DCMIPP_COMMON_H_
#define _DCMIPP_COMMON_H_

#include <linux/interrupt.h>
#include <linux/slab.h>
#include <media/media-device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define DCMIPP_PDEV_NAME "dcmipp"

/* DCMIPP-specific controls */
#define DCMIPP_CID_DCMIPP_BASE		(0x00f00000 | 0xf000)
#define DCMIPP_CID_DCMIPP_CLASS		(0x00f00000 | 1)
#define DCMIPP_CID_TEST_PATTERN		(DCMIPP_CID_DCMIPP_BASE + 0)

#define DCMIPP_FRAME_MAX_WIDTH 4096
#define DCMIPP_FRAME_MAX_HEIGHT 2160
#define DCMIPP_FRAME_MIN_WIDTH 16
#define DCMIPP_FRAME_MIN_HEIGHT 16

#define DCMIPP_FMT_WIDTH_DEFAULT  640
#define DCMIPP_FMT_HEIGHT_DEFAULT 480

#define DCMIPP_FRAME_INDEX(lin, col, width, bpp) \
	(((lin) * (width) + (col)) * (bpp))

#define DCMIPP_COLORSPACE_DEFAULT	V4L2_COLORSPACE_REC709
#define DCMIPP_YCBCR_ENC_DEFAULT	V4L2_YCBCR_ENC_DEFAULT
#define DCMIPP_QUANTIZATION_DEFAULT	V4L2_QUANTIZATION_DEFAULT
#define DCMIPP_XFER_FUNC_DEFAULT	V4L2_XFER_FUNC_DEFAULT

/**
 * struct dcmipp_colorimetry_clamp - Adjust colorimetry parameters
 *
 * @fmt:		the pointer to struct v4l2_pix_format or
 *			struct v4l2_mbus_framefmt
 *
 * Entities must check if colorimetry given by the userspace is valid, if not
 * then set them as DEFAULT
 */
#define dcmipp_colorimetry_clamp(fmt)					\
do {									\
	if ((fmt)->colorspace == V4L2_COLORSPACE_DEFAULT ||		\
	    (fmt)->colorspace > V4L2_COLORSPACE_DCI_P3) {		\
		(fmt)->colorspace = DCMIPP_COLORSPACE_DEFAULT;		\
		(fmt)->ycbcr_enc = DCMIPP_YCBCR_ENC_DEFAULT;		\
		(fmt)->quantization = DCMIPP_QUANTIZATION_DEFAULT;	\
		(fmt)->xfer_func = DCMIPP_XFER_FUNC_DEFAULT;		\
	}								\
	if ((fmt)->ycbcr_enc > V4L2_YCBCR_ENC_SMPTE240M)		\
		(fmt)->ycbcr_enc = DCMIPP_YCBCR_ENC_DEFAULT;		\
	if ((fmt)->quantization > V4L2_QUANTIZATION_LIM_RANGE)		\
		(fmt)->quantization = DCMIPP_QUANTIZATION_DEFAULT;	\
	if ((fmt)->xfer_func > V4L2_XFER_FUNC_SMPTE2084)		\
		(fmt)->xfer_func = DCMIPP_XFER_FUNC_DEFAULT;		\
} while (0)

/**
 * struct dcmipp_platform_data - platform data to components
 *
 * @entity_name:	The name of the entity to be created
 *
 * Board setup code will often provide additional information using the device's
 * platform_data field to hold additional information.
 * When injecting a new platform_device in the component system the core needs
 * to provide to the corresponding submodules the name of the entity that should
 * be used when registering the subdevice in the Media Controller system.
 */
struct dcmipp_platform_data {
	char entity_name[32];
};

struct dcmipp_bind_data {
	/* Internal v4l2 parent device*/
	struct v4l2_device		*v4l2_dev;

	/* Hardware resources */
	struct reset_control		*rstc;
	void __iomem			*regs;
};

/**
 * struct dcmipp_ent_device - core struct that represents a node in the topology
 *
 * @ent:		the pointer to struct media_entity for the node
 * @pads:		the list of pads of the node
 * @process_frame:	callback send a frame to that node
 * @vdev_get_format:	callback that returns the current format a pad, used
 *			only when is_media_entity_v4l2_video_device(ent) returns
 *			true
 *
 * Each node of the topology must create a dcmipp_ent_device struct. Depending on
 * the node it will be of an instance of v4l2_subdev or video_device struct
 * where both contains a struct media_entity.
 * Those structures should embedded the dcmipp_ent_device struct through
 * v4l2_set_subdevdata() and video_set_drvdata() respectivaly, allowing the
 * dcmipp_ent_device struct to be retrieved from the corresponding struct
 * media_entity
 */
struct dcmipp_ent_device {
	struct media_entity *ent;
	struct media_pad *pads;
	void * (*process_frame)(struct dcmipp_ent_device *ved,
				const void *frame);
	void (*vdev_get_format)(struct dcmipp_ent_device *ved,
				struct v4l2_pix_format *fmt);

	/* Parallel input device */
	struct v4l2_fwnode_bus_parallel	bus;
	enum v4l2_mbus_type		bus_type;
	irq_handler_t handler;
	irqreturn_t handler_ret;
	irq_handler_t thread_fn;
};

/**
 * dcmipp_pads_init - initialize pads
 *
 * @num_pads:	number of pads to initialize
 * @pads_flags:	flags to use in each pad
 *
 * Helper functions to allocate/initialize pads
 */
struct media_pad *dcmipp_pads_init(u16 num_pads,
				   const unsigned long *pads_flag);

/**
 * dcmipp_pads_cleanup - free pads
 *
 * @pads: pointer to the pads
 *
 * Helper function to free the pads initialized with dcmipp_pads_init
 */
static inline void dcmipp_pads_cleanup(struct media_pad *pads)
{
	kfree(pads);
}

/**
 * dcmipp_ent_sd_register - initialize and register a subdev node
 *
 * @ved:	the dcmipp_ent_device struct to be initialize
 * @sd:		the v4l2_subdev struct to be initialize and registered
 * @v4l2_dev:	the v4l2 device to register the v4l2_subdev
 * @name:	name of the sub-device. Please notice that the name must be
 *		unique.
 * @function:	media entity function defined by MEDIA_ENT_F_* macros
 * @num_pads:	number of pads to initialize
 * @pads_flag:	flags to use in each pad
 * @sd_int_ops:	pointer to &struct v4l2_subdev_internal_ops
 * @sd_ops:	pointer to &struct v4l2_subdev_ops.
 *
 * Helper function initialize and register the struct dcmipp_ent_device and struct
 * v4l2_subdev which represents a subdev node in the topology
 */
int dcmipp_ent_sd_register(struct dcmipp_ent_device *ved,
			   struct v4l2_subdev *sd,
			   struct v4l2_device *v4l2_dev,
			   const char *const name,
			   u32 function,
			   u16 num_pads,
			   const unsigned long *pads_flag,
			   const struct v4l2_subdev_internal_ops *sd_int_ops,
			   const struct v4l2_subdev_ops *sd_ops,
			   irq_handler_t handler,
			   irq_handler_t thread_fn);

/**
 * dcmipp_ent_sd_unregister - cleanup and unregister a subdev node
 *
 * @ved:	the dcmipp_ent_device struct to be cleaned up
 * @sd:		the v4l2_subdev struct to be unregistered
 *
 * Helper function cleanup and unregister the struct dcmipp_ent_device and struct
 * v4l2_subdev which represents a subdev node in the topology
 */
void dcmipp_ent_sd_unregister(struct dcmipp_ent_device *ved,
			      struct v4l2_subdev *sd);

/**
 * dcmipp_link_validate - validates a media link
 *
 * @link: pointer to &struct media_link
 *
 * This function call validates if a media link is valid for streaming.
 */
int dcmipp_link_validate(struct media_link *link);

#define reg_write(device, reg, val) \
	(reg_write_dbg((device)->dev, #reg, (device)->regs, (reg), (val)))
#define reg_read(device, reg) \
	 (reg_read_dbg((device)->dev, #reg, (device)->regs, (reg)))
#define reg_set(device, reg, mask) \
	 (reg_set_dbg((device)->dev, #reg, (device)->regs, (reg), (mask)))
#define reg_clear(device, reg, mask) \
	 (reg_clear_dbg((device)->dev, #reg, (device)->regs, (reg), (mask)))

static inline u32 reg_read_dbg(struct device *dev, const char *regname,
			       void __iomem *base, u32 reg)
{
	u32 val = readl_relaxed(base + reg);

	dev_dbg(dev, "RD  %s %#10.8x\n", regname, val);
	return val;
}

static inline void reg_write_dbg(struct device *dev, const char *regname,
				 void __iomem *base, u32 reg, u32 val)
{
	dev_dbg(dev, "WR  %s %#10.8x\n", regname, val);
	writel_relaxed(val, base + reg);
}

static inline void reg_set_dbg(struct device *dev, const char *regname,
			       void __iomem *base, u32 reg, u32 mask)
{
	dev_dbg(dev, "SET %s %#10.8x\n", regname, mask);
	reg_write_dbg(dev, regname, base, reg, readl_relaxed(base + reg) | mask);
}

static inline void reg_clear_dbg(struct device *dev, const char *regname,
				 void __iomem *base, u32 reg, u32 mask)
{
	dev_dbg(dev, "CLR %s %#10.8x\n", regname, mask);
	reg_write_dbg(dev, regname, base, reg, readl_relaxed(base + reg) & ~mask);
}

#endif

