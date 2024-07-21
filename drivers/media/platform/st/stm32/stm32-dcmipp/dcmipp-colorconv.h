/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Driver for STM32 Digital Camera Memory Interface Pixel Processor
 *
 * Copyright (C) STMicroelectronics SA 2021
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          Alain Volmat <alain.volmat@foss.st.com>
 *          for STMicroelectronics.
 */

#ifndef _DCMIPP_COLORCONV_H_
#define _DCMIPP_COLORCONV_H_

struct dcmipp_colorconv_config {
	unsigned int conv_matrix[6];
	bool clamping;
	bool clamping_as_rgb;
	bool enable;
};

int dcmipp_colorconv_configure(struct device *dev,
			       struct v4l2_mbus_framefmt *sink,
			       struct v4l2_mbus_framefmt *src,
			       struct dcmipp_colorconv_config *cfg);

#endif

