// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for STM32 Digital Camera Memory Interface Pixel Processor
 *
 * Copyright (C) STMicroelectronics SA 2023
 * Authors: Alain Volmat <alain.volmat@foss.st.com>
 *          Fabien Dessenne <fabien.dessenne@foss.st.com>
 *          Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          for STMicroelectronics.
 */

#include <linux/iopoll.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include <uapi/linux/stm32-dcmipp-config.h>

#include "dcmipp-common.h"

#define DCMIPP_CMSR2_P1VSYNCF	BIT(18)
#define DCMIPP_CMSR2_P2VSYNCF	BIT(26)

struct dcmipp_buf {
	struct vb2_v4l2_buffer	vb;
	struct list_head	list;
};

struct dcmipp_isp_params_device {
	struct dcmipp_ent_device ved;
	struct video_device vdev;
	struct device *dev;
	struct vb2_queue queue;
	struct list_head buffers;
	/* Protects the access of variables shared within the interrupt */
	spinlock_t irqlock;
	/* mutex used as vdev and queue lock */
	struct mutex lock;
	struct v4l2_subdev *s_subdev;
	struct v4l2_subdev_selection s_sel;
	u32 sequence;

	void __iomem *regs;
};

static int dcmipp_isp_params_querycap(struct file *file, void *priv,
				      struct v4l2_capability *cap)
{
	strscpy(cap->driver, DCMIPP_PDEV_NAME, sizeof(cap->driver));
	strscpy(cap->card, KBUILD_MODNAME, sizeof(cap->card));

	return 0;
}

static int dcmipp_isp_params_g_fmt_meta_out(struct file *file, void *priv,
					    struct v4l2_format *f)
{
	struct v4l2_meta_format *meta = &f->fmt.meta;

	if (f->type != V4L2_BUF_TYPE_META_OUTPUT)
		return -EINVAL;

	meta->dataformat = V4L2_META_FMT_ST_DCMIPP_ISP_PARAMS;
	meta->buffersize = sizeof(struct stm32_dcmipp_params_cfg);

	return 0;
}

static int dcmipp_isp_params_enum_fmt_meta_out(struct file *file, void *priv,
					       struct v4l2_fmtdesc *f)
{
	if (f->index > 0 || f->type != V4L2_BUF_TYPE_META_OUTPUT)
		return -EINVAL;

	f->type = V4L2_BUF_TYPE_META_OUTPUT;
	f->pixelformat = V4L2_META_FMT_ST_DCMIPP_ISP_PARAMS;

	return 0;
}

static const struct v4l2_ioctl_ops dcmipp_isp_params_ioctl_ops = {
	.vidioc_querycap = dcmipp_isp_params_querycap,

	.vidioc_enum_fmt_meta_out = dcmipp_isp_params_enum_fmt_meta_out,
	.vidioc_g_fmt_meta_out = dcmipp_isp_params_g_fmt_meta_out,
	.vidioc_s_fmt_meta_out = dcmipp_isp_params_g_fmt_meta_out,
	.vidioc_try_fmt_meta_out = dcmipp_isp_params_g_fmt_meta_out,

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

static int dcmipp_isp_params_start_streaming(struct vb2_queue *vq,
					     unsigned int count)
{
	struct dcmipp_isp_params_device *vout = vb2_get_drv_priv(vq);
	struct media_entity *entity = &vout->vdev.entity;
	struct media_pad *pad;
	int ret;

	/* Get pointer to the source subdev (if case of not yet set */
	if (!vout->s_subdev) {
		pad = media_pad_remote_pad_first(&entity->pads[0]);
		if (!pad || !is_media_entity_v4l2_subdev(pad->entity)) {
			dev_err(vout->dev, "%s: Failed to start streaming, can't find remote entity\n",
				__func__);
			return -EIO;
		}
		vout->s_subdev = media_entity_to_v4l2_subdev(pad->entity);
	}

	/* Get frame information */
	vout->s_sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	vout->s_sel.pad = 0;
	vout->s_sel.target = V4L2_SEL_TGT_COMPOSE;
	ret = v4l2_subdev_call(vout->s_subdev, pad, get_selection, NULL, &vout->s_sel);
	if (ret < 0) {
		dev_err(vout->dev, "Failed to get frame size\n");
		return ret;
	}

	return 0;
}

/*
 * Stop the stream engine. Any remaining buffers in the stream queue are
 * dequeued and passed on to the vb2 framework marked as STATE_ERROR.
 */
static void dcmipp_isp_params_stop_streaming(struct vb2_queue *vq)
{
	struct dcmipp_isp_params_device *vout = vb2_get_drv_priv(vq);
	struct dcmipp_buf *buf, *node;

	spin_lock_irq(&vout->irqlock);

	/* Return all queued buffers to vb2 in ERROR state */
	list_for_each_entry_safe(buf, node, &vout->buffers, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	INIT_LIST_HEAD(&vout->buffers);

	spin_unlock_irq(&vout->irqlock);
}

static int dcmipp_isp_params_validate(struct dcmipp_isp_params_device *vout,
				      struct stm32_dcmipp_params_cfg *params);
static int dcmipp_isp_params_buf_prepare(struct vb2_buffer *vb)
{
	struct dcmipp_isp_params_device *vout =  vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = sizeof(struct stm32_dcmipp_params_cfg);
	struct stm32_dcmipp_params_cfg *params;

	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;

	params = (struct stm32_dcmipp_params_cfg *)vb2_plane_vaddr(vb, 0);
	if (dcmipp_isp_params_validate(vout, params) < 0)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

static void dcmipp_isp_params_buf_queue(struct vb2_buffer *vb2_buf)
{
	struct dcmipp_isp_params_device *vout =
		vb2_get_drv_priv(vb2_buf->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb2_buf);
	struct dcmipp_buf *buf = container_of(vbuf, struct dcmipp_buf, vb);

	spin_lock_irq(&vout->irqlock);
	list_add_tail(&buf->list, &vout->buffers);
	spin_unlock_irq(&vout->irqlock);
}

static int dcmipp_isp_params_queue_setup(struct vb2_queue *vq,
					 unsigned int *nbuffers,
					 unsigned int *nplanes,
					 unsigned int sizes[],
					 struct device *alloc_devs[])
{
	unsigned int size = sizeof(struct stm32_dcmipp_params_cfg);

	if (*nplanes) {
		if (sizes[0] < size)
			return -EINVAL;
	} else {
		sizes[0] = size;
	}

	*nplanes = 1;
	return 0;
}

static int dcmipp_isp_params_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct dcmipp_buf *buf = container_of(vbuf, struct dcmipp_buf, vb);

	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static const struct vb2_ops dcmipp_isp_params_qops = {
	.start_streaming	= dcmipp_isp_params_start_streaming,
	.stop_streaming		= dcmipp_isp_params_stop_streaming,
	.buf_init		= dcmipp_isp_params_buf_init,
	.buf_prepare		= dcmipp_isp_params_buf_prepare,
	.buf_queue		= dcmipp_isp_params_buf_queue,

	.queue_setup		= dcmipp_isp_params_queue_setup,

	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static void dcmipp_isp_params_release(struct video_device *vdev)
{
	struct dcmipp_isp_params_device *vout =
		container_of(vdev, struct dcmipp_isp_params_device, vdev);

	dcmipp_pads_cleanup(vout->ved.pads);
	mutex_destroy(&vout->lock);

	kfree(vout);
}

void dcmipp_isp_params_ent_release(struct dcmipp_ent_device *ved)
{
	struct dcmipp_isp_params_device *vout =
		container_of(ved, struct dcmipp_isp_params_device, ved);

	media_entity_cleanup(ved->ent);
	vb2_video_unregister_device(&vout->vdev);
}

#define DCMIPP_P1BPRCR			0x824
#define DCMIPP_P1BPRCR_ENABLE		BIT(0)
#define DCMIPP_P1BPRCR_STRENGTH_SHIFT	1
#define DCMIPP_P1BPRCR_STRENGTH_MASK	0x07
static inline int
dcmipp_isp_params_valid_bpr(struct stm32_dcmipp_isp_bpr_cfg *cfg)
{
	if (cfg->strength & ~DCMIPP_P1BPRCR_STRENGTH_MASK)
		return -EINVAL;

	return 0;
}

static inline void
dcmipp_isp_params_apply_bpr(struct dcmipp_isp_params_device *vout,
			    struct stm32_dcmipp_isp_bpr_cfg *cfg)
{
	reg_write(vout, DCMIPP_P1BPRCR,
		  (cfg->en ? DCMIPP_P1BPRCR_ENABLE : 0) |
		  cfg->strength << DCMIPP_P1BPRCR_STRENGTH_SHIFT);
}

#define DCMIPP_P1BLCCR			0x840
#define DCMIPP_P1BLCCR_ENABLE		BIT(0)
#define DCMIPP_P1BLCCR_BLCB_SHIFT	8
#define DCMIPP_P1BLCCR_BLCG_SHIFT	16
#define DCMIPP_P1BLCCR_BLCR_SHIFT	24
static inline void
dcmipp_isp_params_apply_blc(struct dcmipp_isp_params_device *vout,
			    struct stm32_dcmipp_isp_blc_cfg *cfg)
{
	reg_write(vout, DCMIPP_P1BLCCR,
		  (cfg->en ? DCMIPP_P1BLCCR_ENABLE : 0) |
		  cfg->blc_r << DCMIPP_P1BLCCR_BLCR_SHIFT |
		  cfg->blc_g << DCMIPP_P1BLCCR_BLCG_SHIFT |
		  cfg->blc_b << DCMIPP_P1BLCCR_BLCB_SHIFT);
}

#define DCMIPP_P1EXCR1			0x844
#define DCMIPP_P1EXCR1_ENABLE		BIT(0)
#define DCMIPP_P1EXCR1_MULTR_SHIFT	20
#define DCMIPP_P1EXCR1_SHFR_SHIFT	28
#define DCMIPP_P1EXCR1_SHFR_MASK	0x07

#define DCMIPP_P1EXCR2			0x848
#define DCMIPP_P1EXCR2_MULTB_SHIFT	4
#define DCMIPP_P1EXCR2_SHFB_SHIFT	12
#define DCMIPP_P1EXCR2_MULTG_SHIFT	20
#define DCMIPP_P1EXCR2_SHFG_SHIFT	28

static inline int
dcmipp_isp_params_valid_ex(struct stm32_dcmipp_isp_ex_cfg *cfg)
{
	if (cfg->shift_r & ~DCMIPP_P1EXCR1_SHFR_MASK ||
	    cfg->shift_g & ~DCMIPP_P1EXCR1_SHFR_MASK ||
	    cfg->shift_b & ~DCMIPP_P1EXCR1_SHFR_MASK)
		return -EINVAL;

	return 0;
}

static inline void
dcmipp_isp_params_apply_ex(struct dcmipp_isp_params_device *vout,
			   struct stm32_dcmipp_isp_ex_cfg *cfg)
{
	reg_write(vout, DCMIPP_P1EXCR1,
		  (cfg->en ? DCMIPP_P1EXCR1_ENABLE : 0) |
		  cfg->mult_r << DCMIPP_P1EXCR1_MULTR_SHIFT |
		  cfg->shift_r << DCMIPP_P1EXCR1_SHFR_SHIFT);

	reg_write(vout, DCMIPP_P1EXCR2,
		  cfg->mult_b << DCMIPP_P1EXCR2_MULTB_SHIFT |
		  cfg->shift_b << DCMIPP_P1EXCR2_SHFB_SHIFT |
		  cfg->mult_g << DCMIPP_P1EXCR2_MULTG_SHIFT |
		  cfg->shift_g << DCMIPP_P1EXCR2_SHFG_SHIFT);
}

#define DCMIPP_P1DMCR_ENABLE		BIT(0)
#define DCMIPP_P1DMCR			0x870
#define DCMIPP_P1DMCR_PEAK_SHIFT	16
#define DCMIPP_P1DMCR_PEAK_MASK		0x07
#define DCMIPP_P1DMCR_LINEV_SHIFT	20
#define DCMIPP_P1DMCR_LINEV_MASK	0x07
#define DCMIPP_P1DMCR_LINEH_SHIFT	24
#define DCMIPP_P1DMCR_LINEH_MASK	0x07
#define DCMIPP_P1DMCR_EDGE_SHIFT	28
#define DCMIPP_P1DMCR_EDGE_MASK		0x07
static inline int
dcmipp_isp_params_valid_dm(struct stm32_dcmipp_isp_dm_cfg *cfg)
{
	if (cfg->edge & ~DCMIPP_P1DMCR_EDGE_MASK ||
	    cfg->lineh & ~DCMIPP_P1DMCR_LINEH_MASK ||
	    cfg->linev & ~DCMIPP_P1DMCR_LINEV_MASK ||
	    cfg->peak & ~DCMIPP_P1DMCR_PEAK_MASK)
		return -EINVAL;

	return 0;
}

static inline void
dcmipp_isp_params_apply_dm(struct dcmipp_isp_params_device *vout,
			   struct stm32_dcmipp_isp_dm_cfg *cfg)
{
	u32 dmcr, mask;

	mask = DCMIPP_P1DMCR_PEAK_MASK << DCMIPP_P1DMCR_PEAK_SHIFT |
	       DCMIPP_P1DMCR_LINEV_MASK << DCMIPP_P1DMCR_LINEV_SHIFT |
	       DCMIPP_P1DMCR_LINEH_MASK << DCMIPP_P1DMCR_LINEH_SHIFT |
	       DCMIPP_P1DMCR_EDGE_MASK << DCMIPP_P1DMCR_EDGE_SHIFT;

	dmcr = reg_read(vout, DCMIPP_P1DMCR) & ~mask;
	reg_write(vout, DCMIPP_P1DMCR, (dmcr & ~DCMIPP_P1DMCR_ENABLE) |
		  (cfg->en ? DCMIPP_P1DMCR_ENABLE : 0) |
		  cfg->peak << DCMIPP_P1DMCR_PEAK_SHIFT |
		  cfg->lineh << DCMIPP_P1DMCR_LINEH_SHIFT |
		  cfg->linev << DCMIPP_P1DMCR_LINEV_SHIFT |
		  cfg->edge << DCMIPP_P1DMCR_EDGE_SHIFT);
}

#define DCMIPP_P1CCCR			0x880
#define DCMIPP_P1CCCR_ENABLE		BIT(0)
#define DCMIPP_P1CCCR_TYPE_YUV		0
#define DCMIPP_P1CCCR_TYPE_RGB		BIT(1)
#define DCMIPP_P1CCCR_CLAMP		BIT(2)
#define DCMIPP_P1CCRR_RGB_MASK		0x7ff
#define DCMIPP_P1CCRR_A_MASK		0x3ff
#define DCMIPP_P1CCRR1			0x884
#define DCMIPP_P1CCRR1_RG_SHIFT		16
#define DCMIPP_P1CCRR1_RR_SHIFT		0
#define DCMIPP_P1CCRR2			0x888
#define DCMIPP_P1CCRR2_RA_SHIFT		16
#define DCMIPP_P1CCRR2_RB_SHIFT		0
#define DCMIPP_P1CCGR1			0x88c
#define DCMIPP_P1CCGR1_GG_SHIFT		16
#define DCMIPP_P1CCGR1_GR_SHIFT		0
#define DCMIPP_P1CCGR2			0x890
#define DCMIPP_P1CCGR2_GA_SHIFT		16
#define DCMIPP_P1CCGR2_GB_SHIFT		0
#define DCMIPP_P1CCBR1			0x894
#define DCMIPP_P1CCBR1_BG_SHIFT		16
#define DCMIPP_P1CCBR1_BR_SHIFT		0
#define DCMIPP_P1CCBR2			0x898
#define DCMIPP_P1CCBR2_BA_SHIFT		16
#define DCMIPP_P1CCBR2_BB_SHIFT		0
static inline int
dcmipp_isp_params_valid_cc(struct stm32_dcmipp_isp_cc_cfg *cfg)
{
	if (cfg->rr & ~DCMIPP_P1CCRR_RGB_MASK || cfg->rg & ~DCMIPP_P1CCRR_RGB_MASK ||
	    cfg->rb & ~DCMIPP_P1CCRR_RGB_MASK || cfg->gr & ~DCMIPP_P1CCRR_RGB_MASK ||
	    cfg->gg & ~DCMIPP_P1CCRR_RGB_MASK || cfg->gb & ~DCMIPP_P1CCRR_RGB_MASK ||
	    cfg->br & ~DCMIPP_P1CCRR_RGB_MASK || cfg->bg & ~DCMIPP_P1CCRR_RGB_MASK ||
	    cfg->bb & ~DCMIPP_P1CCRR_RGB_MASK || cfg->ra & ~DCMIPP_P1CCRR_A_MASK ||
	    cfg->ga & ~DCMIPP_P1CCRR_A_MASK || cfg->ba & ~DCMIPP_P1CCRR_A_MASK)
		return -EINVAL;

	return 0;
}

static inline void
dcmipp_isp_params_apply_cc(struct dcmipp_isp_params_device *vout,
			   struct stm32_dcmipp_isp_cc_cfg *cfg)
{
	u32 cccr = cfg->en ? DCMIPP_P1CCCR_ENABLE : 0;

	if (cfg->clamp != STM32_DCMIPP_ISP_CC_CLAMP_DISABLED) {
		cccr |= DCMIPP_P1CCCR_CLAMP;
		cccr |= cfg->clamp == STM32_DCMIPP_ISP_CC_CLAMP_YUV235 ?
			DCMIPP_P1CCCR_TYPE_YUV : DCMIPP_P1CCCR_TYPE_RGB;
	}

	reg_write(vout, DCMIPP_P1CCCR, cccr);
	reg_write(vout, DCMIPP_P1CCRR1,
		  cfg->rr << DCMIPP_P1CCRR1_RR_SHIFT |
		  cfg->rg << DCMIPP_P1CCRR1_RG_SHIFT);
	reg_write(vout, DCMIPP_P1CCRR2,
		  cfg->ra << DCMIPP_P1CCRR2_RA_SHIFT |
		  cfg->rb << DCMIPP_P1CCRR2_RB_SHIFT);
	reg_write(vout, DCMIPP_P1CCGR1,
		  cfg->gr << DCMIPP_P1CCGR1_GR_SHIFT |
		  cfg->gg << DCMIPP_P1CCGR1_GG_SHIFT);
	reg_write(vout, DCMIPP_P1CCGR2,
		  cfg->gb << DCMIPP_P1CCGR2_GB_SHIFT |
		  cfg->ga << DCMIPP_P1CCGR2_GA_SHIFT);
	reg_write(vout, DCMIPP_P1CCBR1,
		  cfg->br << DCMIPP_P1CCBR1_BR_SHIFT |
		  cfg->bg << DCMIPP_P1CCBR1_BG_SHIFT);
	reg_write(vout, DCMIPP_P1CCBR2,
		  cfg->bb << DCMIPP_P1CCBR2_BB_SHIFT |
		  cfg->ba << DCMIPP_P1CCBR2_BA_SHIFT);
}

#define DCMIPP_P1CTCR1			0x8a0
#define DCMIPP_P1CTCR1_ENABLE		BIT(0)
#define DCMIPP_P1CTCR_LUM_MASK		0x3f
#define DCMIPP_P1CTCR1_LUM0_SHIFT	9

#define DCMIPP_P1CTCR2			0x8a4
#define DCMIPP_P1CTCR2_LUM4_SHIFT	1
#define DCMIPP_P1CTCR2_LUM3_SHIFT	9
#define DCMIPP_P1CTCR2_LUM2_SHIFT	17
#define DCMIPP_P1CTCR2_LUM1_SHIFT	25

#define DCMIPP_P1CTCR3			0x8a8
#define DCMIPP_P1CTCR3_LUM8_SHIFT	1
#define DCMIPP_P1CTCR3_LUM7_SHIFT	9
#define DCMIPP_P1CTCR3_LUM6_SHIFT	17
#define DCMIPP_P1CTCR3_LUM5_SHIFT	25
static inline int
dcmipp_isp_params_valid_ce(struct stm32_dcmipp_isp_ce_cfg *cfg)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cfg->lum); i++)
		if (cfg->lum[i] & ~DCMIPP_P1CTCR_LUM_MASK)
			return -EINVAL;

	return 0;
}

static inline void
dcmipp_isp_params_apply_ce(struct dcmipp_isp_params_device *vout,
			   struct stm32_dcmipp_isp_ce_cfg *cfg)
{
	reg_write(vout, DCMIPP_P1CTCR1,
		  (cfg->en ? DCMIPP_P1CTCR1_ENABLE : 0) |
		  cfg->lum[0] << DCMIPP_P1CTCR1_LUM0_SHIFT);
	reg_write(vout, DCMIPP_P1CTCR2,
		  cfg->lum[1] << DCMIPP_P1CTCR2_LUM1_SHIFT |
		  cfg->lum[2] << DCMIPP_P1CTCR2_LUM2_SHIFT |
		  cfg->lum[3] << DCMIPP_P1CTCR2_LUM3_SHIFT |
		  cfg->lum[4] << DCMIPP_P1CTCR2_LUM4_SHIFT);
	reg_write(vout, DCMIPP_P1CTCR3,
		  cfg->lum[5] << DCMIPP_P1CTCR3_LUM5_SHIFT |
		  cfg->lum[6] << DCMIPP_P1CTCR3_LUM6_SHIFT |
		  cfg->lum[7] << DCMIPP_P1CTCR3_LUM7_SHIFT |
		  cfg->lum[8] << DCMIPP_P1CTCR3_LUM8_SHIFT);
}

#define DCMIPP_P1HSCR			0x8b0
#define DCMIPP_P1HSCR_ENABLE		BIT(0)
#define DCMIPP_P1HSCR_SRC_SHIFT		1
#define DCMIPP_P1HSCR_HREG_SHIFT	4
#define DCMIPP_P1HSCR_VREG_SHIFT	8
#define DCMIPP_P1HSCR_HDEC_SHIFT	12
#define DCMIPP_P1HSCR_VDEC_SHIFT	16
#define DCMIPP_P1HSCR_COMP_SHIFT	20
#define DCMIPP_P1HSCR_DYN_SHIFT		24
#define DCMIPP_P1HSCR_BIN_SHIFT		26

#define DCMIPP_P1HSSTR			0x8b4
#define DCMIPP_P1HSSTR_HSTART_SHIFT	0
#define DCMIPP_P1HSSTR_VSTART_SHIFT	16

#define DCMIPP_P1HSSZR			0x8b8
#define DCMIPP_P1HSSZR_HSIZE_SHIFT	0
#define DCMIPP_P1HSSZR_VSIZE_SHIFT	16

#define DCMIPP_MAX_BINS_PER_LINE	320

static inline int
dcmipp_isp_params_get_bin_size(__u8 bin)
{
	if (bin == STM32_DCMIPP_ISP_HISTO_BIN_4)
		return 4;
	else if (bin == STM32_DCMIPP_ISP_HISTO_BIN_16)
		return 16;
	else if (bin == STM32_DCMIPP_ISP_HISTO_BIN_64)
		return 64;
	else if (bin == STM32_DCMIPP_ISP_HISTO_BIN_256)
		return 256;

	/* Unreached */
	return 0;
}

static inline int
dcmipp_isp_params_valid_histo(struct dcmipp_isp_params_device *vout,
			      struct stm32_dcmipp_isp_histo_cfg *cfg)
{
	unsigned int bins_per_line;

	if (cfg->bin > STM32_DCMIPP_ISP_HISTO_BIN_256)
		return -EINVAL;

	if (cfg->dyn > STM32_DCMIPP_ISP_HISTO_DYN_DARK)
		return -EINVAL;

	if (cfg->comp > STM32_DCMIPP_ISP_HISTO_COMP_ALL)
		return -EINVAL;

	if (cfg->vdec > STM32_DCMIPP_ISP_HISTO_VHDEC_16)
		return -EINVAL;

	if (cfg->hdec > STM32_DCMIPP_ISP_HISTO_VHDEC_16)
		return -EINVAL;

	if (!cfg->vreg || cfg->vreg > STM32_DCMIPP_ISP_HISTO_MAX_VHREG)
		return -EINVAL;

	if (!cfg->hreg || cfg->hreg > STM32_DCMIPP_ISP_HISTO_MAX_VHREG)
		return -EINVAL;

	if (cfg->src > STM32_DCMIPP_ISP_HISTO_SRC_POST_CE)
		return -EINVAL;

	if (cfg->top > DCMIPP_PIXEL_FRAME_MAX_HEIGHT ||
	    cfg->left > DCMIPP_PIXEL_FRAME_MAX_WIDTH ||
	    cfg->width > DCMIPP_PIXEL_FRAME_MAX_WIDTH ||
	    cfg->height > DCMIPP_PIXEL_FRAME_MAX_HEIGHT) {
		dev_err(vout->dev, "Invalid histogram top/left/width/height values\n");
		return -EINVAL;
	}

	/* Region width/height should be a multiple of decimation */
	if (cfg->width % (1 << cfg->hdec)) {
		dev_err(vout->dev, "Histogram region width should be multiple of decimation\n");
		return -EINVAL;
	}
	if (cfg->height % (1 << cfg->vdec)) {
		dev_err(vout->dev, "Histogram region height should be multiple of decimation\n");
		return -EINVAL;
	}

	/* Region width/height * nb of region should not exceed frame size */
	if (cfg->left + cfg->width * cfg->hreg > vout->s_sel.r.width ||
	    cfg->top + cfg->height * cfg->vreg > vout->s_sel.r.height) {
		dev_err(vout->dev, "Histogram region size exceed frame size\n");
		return -EINVAL;
	}

	/*
	 * The DCMIPP as a maximum of 320 BINS per line so we need to ensure that
	 * requested settings can be achieved
	 */
	bins_per_line = dcmipp_isp_params_get_bin_size(cfg->bin) * cfg->hreg;
	bins_per_line *= (cfg->comp == STM32_DCMIPP_ISP_HISTO_COMP_ALL ? 4 : 1);
	if (bins_per_line > DCMIPP_MAX_BINS_PER_LINE) {
		dev_err(vout->dev, "Histogram config exceed maximum BINS caps: %d instead of 320\n",
			bins_per_line);
		return -EINVAL;
	}

	return 0;
}

static inline void
dcmipp_isp_params_apply_histo(struct dcmipp_isp_params_device *vout,
			      struct stm32_dcmipp_isp_histo_cfg *cfg)
{
	bool is_enabled = reg_read(vout, DCMIPP_P1HSCR) & DCMIPP_P1HSCR_ENABLE;
	u32 hscr = 0;

	reg_write(vout, DCMIPP_P1HSSTR,
		  (cfg->left << DCMIPP_P1HSSTR_HSTART_SHIFT) |
		  (cfg->top << DCMIPP_P1HSSTR_VSTART_SHIFT));

	reg_write(vout, DCMIPP_P1HSSZR,
		  (cfg->width << DCMIPP_P1HSSZR_HSIZE_SHIFT) |
		  (cfg->height << DCMIPP_P1HSSZR_VSIZE_SHIFT));

	hscr = cfg->bin << DCMIPP_P1HSCR_BIN_SHIFT |
	       cfg->dyn << DCMIPP_P1HSCR_DYN_SHIFT |
	       cfg->comp << DCMIPP_P1HSCR_COMP_SHIFT |
	       cfg->vdec << DCMIPP_P1HSCR_VDEC_SHIFT |
	       cfg->hdec << DCMIPP_P1HSCR_HDEC_SHIFT |
	       (cfg->vreg - 1) << DCMIPP_P1HSCR_VREG_SHIFT |
	       (cfg->hreg - 1) << DCMIPP_P1HSCR_HREG_SHIFT |
	       cfg->src << DCMIPP_P1HSCR_SRC_SHIFT |
	       (is_enabled ? DCMIPP_P1HSCR_ENABLE : 0);

	reg_write(vout, DCMIPP_P1HSCR, hscr);
}

#define DCMIPP_MODULE_CFG_MASK	(STM32_DCMIPP_ISP_BPR | STM32_DCMIPP_ISP_BLC | \
				 STM32_DCMIPP_ISP_EX | STM32_DCMIPP_ISP_DM | \
				 STM32_DCMIPP_ISP_CC | STM32_DCMIPP_ISP_CE | \
				 STM32_DCMIPP_ISP_HISTO)
static int dcmipp_isp_params_validate(struct dcmipp_isp_params_device *vout,
				      struct stm32_dcmipp_params_cfg *params)
{
	int ret;

	if (params->module_cfg_update & ~DCMIPP_MODULE_CFG_MASK)
		return -EINVAL;
	if (params->module_cfg_update & STM32_DCMIPP_ISP_BPR) {
		ret = dcmipp_isp_params_valid_bpr(&params->ctrls.bpr_cfg);
		if (ret)
			return ret;
	}
	if (params->module_cfg_update & STM32_DCMIPP_ISP_EX) {
		ret = dcmipp_isp_params_valid_ex(&params->ctrls.ex_cfg);
		if (ret)
			return ret;
	}
	if (params->module_cfg_update & STM32_DCMIPP_ISP_DM) {
		ret = dcmipp_isp_params_valid_dm(&params->ctrls.dm_cfg);
		if (ret)
			return ret;
	}
	if (params->module_cfg_update & STM32_DCMIPP_ISP_CC) {
		ret = dcmipp_isp_params_valid_cc(&params->ctrls.cc_cfg);
		if (ret)
			return ret;
	}
	if (params->module_cfg_update & STM32_DCMIPP_ISP_CE) {
		ret = dcmipp_isp_params_valid_ce(&params->ctrls.ce_cfg);
		if (ret)
			return ret;
	}
	if (params->module_cfg_update & STM32_DCMIPP_ISP_HISTO) {
		if (!vout->ved.dcmipp->pipe_cfg->has_histo) {
			dev_err(vout->dev, "DCMIPP ISP Histogram not supported on this SOC\n");
			return -EINVAL;
		}

		ret = dcmipp_isp_params_valid_histo(vout, &params->ctrls.histo_cfg);
		if (ret)
			return ret;
	}

	return 0;
}

static inline void
dcmipp_isp_params_apply(struct dcmipp_isp_params_device *vout,
			struct stm32_dcmipp_params_cfg *buf)
{
	if (buf->module_cfg_update & STM32_DCMIPP_ISP_BPR)
		dcmipp_isp_params_apply_bpr(vout, &buf->ctrls.bpr_cfg);
	if (buf->module_cfg_update & STM32_DCMIPP_ISP_BLC)
		dcmipp_isp_params_apply_blc(vout, &buf->ctrls.blc_cfg);
	if (buf->module_cfg_update & STM32_DCMIPP_ISP_EX)
		dcmipp_isp_params_apply_ex(vout, &buf->ctrls.ex_cfg);
	if (buf->module_cfg_update & STM32_DCMIPP_ISP_DM)
		dcmipp_isp_params_apply_dm(vout, &buf->ctrls.dm_cfg);
	if (buf->module_cfg_update & STM32_DCMIPP_ISP_CC)
		dcmipp_isp_params_apply_cc(vout, &buf->ctrls.cc_cfg);
	if (buf->module_cfg_update & STM32_DCMIPP_ISP_CE)
		dcmipp_isp_params_apply_ce(vout, &buf->ctrls.ce_cfg);
	if (buf->module_cfg_update & STM32_DCMIPP_ISP_HISTO)
		dcmipp_isp_params_apply_histo(vout, &buf->ctrls.histo_cfg);
}

static irqreturn_t dcmipp_isp_params_irq_thread(int irq, void *arg)
{
	struct dcmipp_isp_params_device *vout =
			container_of(arg, struct dcmipp_isp_params_device, ved);
	struct dcmipp_ent_device *ved = arg;
	struct stm32_dcmipp_params_cfg *params_cfg;
	struct dcmipp_buf *cur_buf = NULL;

	/* We are only interested in VSYNC interrupts */
	if (!(ved->cmsr2 & DCMIPP_CMSR2_P1VSYNCF) &&
	    !(ved->cmsr2 & DCMIPP_CMSR2_P2VSYNCF))
		return IRQ_HANDLED;

	spin_lock_irq(&vout->irqlock);

	/* Get an available buffer */
	if (list_empty(&vout->buffers))
		goto out;

	cur_buf = list_first_entry(&vout->buffers, struct dcmipp_buf, list);
	list_del(&cur_buf->list);
	params_cfg =
		(struct stm32_dcmipp_params_cfg *)vb2_plane_vaddr(&cur_buf->vb.vb2_buf, 0);

	/* Handle the params configuration */
	dcmipp_isp_params_apply(vout, params_cfg);

	/* TODO - we need to set a proper sequence number */
	cur_buf->vb.sequence = 0;
	cur_buf->vb.vb2_buf.timestamp = ktime_get_ns();
	vb2_buffer_done(&cur_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

out:
	spin_unlock_irq(&vout->irqlock);

	return IRQ_HANDLED;
}

static const struct v4l2_file_operations dcmipp_isp_params_fops = {
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.mmap           = vb2_fop_mmap,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
};

struct dcmipp_ent_device *
dcmipp_isp_params_ent_init(const char *entity_name,
			   struct dcmipp_device *dcmipp)
{
	struct dcmipp_isp_params_device *vout;
	struct device *dev = dcmipp->dev;
	struct video_device *vdev;
	struct vb2_queue *q;
	const unsigned long pad_flag = MEDIA_PAD_FL_SOURCE;
	int ret = 0;

	/* Allocate the dcmipp_cap_device struct */
	vout = kzalloc(sizeof(*vout), GFP_KERNEL);
	if (!vout)
		return ERR_PTR(-ENOMEM);

	/* Allocate the pad */
	vout->ved.pads = dcmipp_pads_init(1, &pad_flag);
	if (IS_ERR(vout->ved.pads)) {
		ret = PTR_ERR(vout->ved.pads);
		goto err_free_vout;
	}

	vout->ved.dcmipp = dcmipp;

	/* Initialize the media entity */
	vout->vdev.entity.name = entity_name;
	vout->vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	ret = media_entity_pads_init(&vout->vdev.entity, 1, vout->ved.pads);
	if (ret)
		goto err_clean_pads;

	/* Initialize the lock */
	mutex_init(&vout->lock);

	/* Initialize the vb2 queue */
	q = &vout->queue;
	q->type = V4L2_BUF_TYPE_META_OUTPUT;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->lock = &vout->lock;
	q->drv_priv = vout;
	q->buf_struct_size = sizeof(struct dcmipp_buf);
	q->ops = &dcmipp_isp_params_qops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 1;
	q->dev = dev;

	ret = vb2_queue_init(q);
	if (ret) {
		dev_err(dev, "%s: vb2 queue init failed (err=%d)\n",
			entity_name, ret);
		goto err_entity_cleanup;
	}

	/* Initialize buffer list and its lock */
	INIT_LIST_HEAD(&vout->buffers);
	spin_lock_init(&vout->irqlock);

	/* Fill the dcmipp_ent_device struct */
	vout->ved.ent = &vout->vdev.entity;
	vout->ved.handler = NULL;
	vout->ved.thread_fn = dcmipp_isp_params_irq_thread;
	vout->dev = dev;
	vout->regs = dcmipp->regs;

	/* Initialize the video_device struct */
	vdev = &vout->vdev;
	vdev->device_caps = V4L2_CAP_META_OUTPUT | V4L2_CAP_STREAMING;
	vdev->vfl_dir = VFL_DIR_TX;
	vdev->release = dcmipp_isp_params_release;
	vdev->fops = &dcmipp_isp_params_fops;
	vdev->ioctl_ops = &dcmipp_isp_params_ioctl_ops;
	vdev->lock = &vout->lock;
	vdev->queue = q;
	vdev->v4l2_dev = &dcmipp->v4l2_dev;
	strscpy(vdev->name, entity_name, sizeof(vdev->name));
	video_set_drvdata(vdev, &vout->ved);

	/* Register the video_device with the v4l2 and the media framework */
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(dev, "%s: video register failed (err=%d)\n",
			vout->vdev.name, ret);
		goto err_entity_cleanup;
	}

	return &vout->ved;

err_entity_cleanup:
	media_entity_cleanup(&vout->vdev.entity);
	mutex_destroy(&vout->lock);
err_clean_pads:
	dcmipp_pads_cleanup(vout->ved.pads);
err_free_vout:
	kfree(vout);

	return ERR_PTR(ret);
}
