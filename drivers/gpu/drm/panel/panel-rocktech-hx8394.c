// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics SA 2022
 *
 * Author: Yannick Fertre <yannick.fertre@foss.st.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct hx8394 {
	struct device *dev;
	struct drm_panel panel;
	struct gpio_desc *reset_gpio;
	struct regulator *supply;
	bool prepared;
	bool enabled;
};

static const struct drm_display_mode default_mode = {
	.clock = 54000,
	.hdisplay = 720,
	.hsync_start = 720 + 48,
	.hsync_end = 720 + 48 + 9,
	.htotal = 720 + 48 + 9 + 48,
	.vdisplay = 1280,
	.vsync_start = 1280 + 12,
	.vsync_end = 1280 + 12 + 5,
	.vtotal = 1280 + 12 + 5 + 12,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.width_mm = 68,
	.height_mm = 122,
};

#define MCS_SETPOWER	0xB1
#define MCS_SETDISP	0xB2
#define MCS_SETCYC	0xB4
#define MCS_SETVCOM	0xB6
#define MCS_SETEXTC	0xB9
#define MCS_SETMIPI	0xBA
#define MCS_SET_BANK	0xBD
#define MCS_NO_DOC1	0xBF
#define MCS_NO_DOC2	0xC0
#define MCS_NO_DOC3	0xC6
#define MCS_NO_DOC4	0xD8
#define MCS_NO_DOC5	0xD4
#define MCS_SETPANEL	0xCC
#define MCS_SETGIP_0	0xD3
#define MCS_SETGIP_1	0xD5
#define MCS_SETGIP_2	0xD6

#define MCS_SETGAMMA	0xE0
#define MCS_READ_ID1	0xDA
#define MCS_READ_ID2	0xDB
#define MCS_READ_ID3	0xDC

#define MY	BIT(7)	/* Row Address Order */
#define MX	BIT(6)	/* Column Address Order */
#define MV	BIT(5)	/* Row/Column Exchange */
#define ML	BIT(4)	/* Vertical Refresh Order */
#define RGB	BIT(3)	/* RGB-BGR Order */
#define DDL	BIT(2)	/* Display Data Latch Order */
#define FH	BIT(1)	/* Flip Horizontal */
#define FV	BIT(0)	/* Flip Vertical */

static inline struct hx8394 *panel_to_hx8394(struct drm_panel *panel)
{
	return container_of(panel, struct hx8394, panel);
}

#define dcs_write_cmd_seq(c, cmd, seq...)						\
({											\
	static const u8 d[] = { seq };							\
	struct mipi_dsi_device *dsi = to_mipi_dsi_device((c)->dev);			\
	int err;									\
	err = mipi_dsi_dcs_write(dsi, cmd, d, ARRAY_SIZE(d));				\
	if (err < 0)									\
		dev_err(ctx->dev, "MIPI DSI DCS write failed: %d\n", err);		\
})

static void hx8394_dcs_write_buf(struct hx8394 *ctx, const void *data,
				 size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int err;

	err = mipi_dsi_dcs_write_buffer(dsi, data, len);
	if (err < 0)
		dev_err_ratelimited(ctx->dev, "MIPI DSI DCS write buffer failed: %d\n", err);
}

#define dcs_write_seq(ctx, seq...)				\
({								\
	static const u8 d[] = { seq };				\
								\
	hx8394_dcs_write_buf(ctx, d, ARRAY_SIZE(d));	\
})

static void hx8394_init_sequence(struct hx8394 *ctx)
{
	dcs_write_cmd_seq(ctx, MCS_SETEXTC, 0xFF, 0x83, 0x94);
	dcs_write_cmd_seq(ctx, MCS_SETMIPI, 0x61, 0x03, 0x68, 0x6B, 0xB2, 0xC0);
	dcs_write_seq(ctx, MCS_SETPOWER, 0x48, 0x12, 0x72, 0x09, 0x32, 0x54, 0x71, 0x71, 0x57,
		      0x47);
	dcs_write_cmd_seq(ctx, MCS_SETDISP, 0x00, 0x80, 0x64, 0x0C, 0x0D, 0x2F);
	dcs_write_seq(ctx, MCS_SETCYC, 0x73, 0x74, 0x73, 0x74, 0x73, 0x74, 0x01, 0x0C, 0x86, 0x75,
		      0x00, 0x3F, 0x73, 0x74, 0x73, 0x74, 0x73, 0x74, 0x01, 0x0C, 0x86);
	dcs_write_seq(ctx, MCS_SETGIP_0, 0x00, 0x00, 0x07, 0x07, 0x40, 0x07, 0x0C, 0x00, 0x08, 0x10,
		      0x08, 0x00, 0x08, 0x54, 0x15, 0x0A, 0x05, 0x0A, 0x02, 0x15, 0x06, 0x05, 0x06,
		      0x47, 0x44, 0x0A, 0x0A, 0x4B, 0x10, 0x07, 0x07, 0x0C, 0x40);
	dcs_write_seq(ctx, MCS_SETGIP_1, 0x1C, 0x1C, 0x1D, 0x1D, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
		      0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x24, 0x25, 0x18, 0x18, 0x26, 0x27, 0x18,
		      0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
		      0x18, 0x18, 0x20, 0x21, 0x18, 0x18, 0x18, 0x18);
	dcs_write_seq(ctx, MCS_SETGIP_2, 0x1C, 0x1C, 0x1D, 0x1D, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02,
		      0x01, 0x00, 0x0B, 0x0A, 0x09, 0x08, 0x21, 0x20, 0x18, 0x18, 0x27, 0x26, 0x18,
		      0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
		      0x18, 0x18, 0x25, 0x24, 0x18, 0x18, 0x18, 0x18);
	dcs_write_cmd_seq(ctx, MCS_SETVCOM, 0x92, 0x92);
	dcs_write_seq(ctx, MCS_SETGAMMA, 0x00, 0x0A, 0x15, 0x1B, 0x1E, 0x21, 0x24, 0x22, 0x47, 0x56,
		      0x65, 0x66, 0x6E, 0x82, 0x88, 0x8B, 0x9A, 0x9D, 0x98, 0xA8, 0xB9, 0x5D, 0x5C,
		      0x61, 0x66, 0x6A, 0x6F, 0x7F, 0x7F, 0x00, 0x0A, 0x15, 0x1B, 0x1E, 0x21, 0x24,
		      0x22, 0x47, 0x56, 0x65, 0x65, 0x6E, 0x81, 0x87, 0x8B, 0x98, 0x9D, 0x99, 0xA8,
		      0xBA, 0x5D, 0x5D, 0x62, 0x67, 0x6B, 0x72, 0x7F,  0x7F);
	dcs_write_cmd_seq(ctx, MCS_NO_DOC2, 0x1F, 0x31);
	dcs_write_cmd_seq(ctx, MCS_SETPANEL, 0x03);
	dcs_write_cmd_seq(ctx, MCS_NO_DOC5, 0x02);
	dcs_write_cmd_seq(ctx, MCS_SET_BANK, 0x02);
	dcs_write_seq(ctx, MCS_NO_DOC4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		      0xFF, 0xFF);
	dcs_write_cmd_seq(ctx, MCS_SET_BANK, 0x00);
	dcs_write_cmd_seq(ctx, MCS_SET_BANK, 0x01);
	dcs_write_cmd_seq(ctx, MCS_SETPOWER, 0x00);
	dcs_write_cmd_seq(ctx, MCS_SET_BANK, 0x00);
	dcs_write_cmd_seq(ctx, MCS_NO_DOC1, 0x40, 0x81, 0x50, 0x00, 0x1A, 0xFC, 0x01);
	dcs_write_cmd_seq(ctx, MCS_NO_DOC3, 0xED);
	dcs_write_cmd_seq(ctx, MIPI_DCS_SET_ADDRESS_MODE, FH);
}

static int hx8394_read_id(struct hx8394 *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	u8 id1, id2, id3;
	int ret;

	ret = mipi_dsi_dcs_read(dsi, MCS_READ_ID1, &id1, 1);
	if (ret < 0) {
		dev_err(ctx->dev, "could not read MTP ID1\n");
		return ret;
	}
	ret = mipi_dsi_dcs_read(dsi, MCS_READ_ID2, &id2, 1);
	if (ret < 0) {
		dev_err(ctx->dev, "could not read MTP ID2\n");
		return ret;
	}
	ret = mipi_dsi_dcs_read(dsi, MCS_READ_ID3, &id3, 1);
	if (ret < 0) {
		dev_err(ctx->dev, "could not read MTP ID3\n");
		return ret;
	}

	dev_info(ctx->dev, "MTP ID manufacturer: %02x version: %02x driver: %02x\n", id1, id2, id3);

	return 0;
}

static int hx8394_enable(struct drm_panel *panel)
{
	struct hx8394 *ctx = panel_to_hx8394(panel);

	if (ctx->enabled)
		return 0;

	ctx->enabled = true;

	return 0;
}

static int hx8394_disable(struct drm_panel *panel)
{
	struct hx8394 *ctx = panel_to_hx8394(panel);

	if (!ctx->enabled)
		return 0;

	ctx->enabled = false;

	return 0;
}

static int hx8394_prepare(struct drm_panel *panel)
{
	struct hx8394 *ctx = panel_to_hx8394(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (ctx->prepared)
		return 0;

	ret = pm_runtime_get_sync(panel->dev);
	if (ret < 0) {
		pm_runtime_put_autosuspend(panel->dev);
		return ret;
	}

	ret = hx8394_read_id(ctx);
	if (ret < 0)
		return ret;

	hx8394_init_sequence(ctx);

	ret = mipi_dsi_dcs_set_tear_off(dsi);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret)
		return ret;

	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret)
		return ret;

	msleep(50);

	ctx->prepared = true;

	return 0;
}

static int hx8394_unprepare(struct drm_panel *panel)
{
	struct hx8394 *ctx = panel_to_hx8394(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret)
		dev_warn(panel->dev, "failed to set display off: %d\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret)
		dev_warn(panel->dev, "failed to enter sleep mode: %d\n", ret);

	pm_runtime_set_autosuspend_delay(ctx->dev, 1000);
	pm_runtime_mark_last_busy(panel->dev);
	ret = pm_runtime_put_autosuspend(panel->dev);
	if (ret < 0)
		return ret;

	ctx->prepared = false;

	return 0;
}

static int hx8394_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bus_flags = DRM_BUS_FLAG_DE_HIGH |
					    DRM_BUS_FLAG_PIXDATA_DRIVE_POSEDGE;

	return 1;
}

static const struct drm_panel_funcs hx8394_drm_funcs = {
	.disable = hx8394_disable,
	.unprepare = hx8394_unprepare,
	.prepare = hx8394_prepare,
	.enable = hx8394_enable,
	.get_modes = hx8394_get_modes,
};

static int hx8394_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct hx8394 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		ret = PTR_ERR(ctx->reset_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "cannot get reset GPIO: %d\n", ret);
		return ret;
	}

	ctx->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(ctx->supply)) {
		ret = PTR_ERR(ctx->supply);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "cannot get regulator: %d\n", ret);
		return ret;
	}

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	dsi->lanes = 2;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	pm_runtime_enable(ctx->dev);
	/* set delay to 60s to keep alive the panel to wait the splash screen */
	pm_runtime_set_autosuspend_delay(ctx->dev, 60000);
	pm_runtime_use_autosuspend(ctx->dev);

	drm_panel_init(&ctx->panel, dev, &hx8394_drm_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	pm_runtime_get_sync(ctx->dev);
	pm_runtime_mark_last_busy(ctx->dev);
	pm_runtime_put_autosuspend(ctx->dev);

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "mipi_dsi_attach() failed: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void hx8394_remove(struct mipi_dsi_device *dsi)
{
	struct hx8394 *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	pm_runtime_dont_use_autosuspend(ctx->dev);
	pm_runtime_disable(ctx->dev);
}

static __maybe_unused int rocktech_hx8394_suspend(struct device *dev)
{
	struct hx8394 *ctx = dev_get_drvdata(dev);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(20);

	regulator_disable(ctx->supply);

	return 0;
}

static __maybe_unused int rocktech_hx8394_resume(struct device *dev)
{
	struct hx8394 *ctx = dev_get_drvdata(dev);
	int ret;

	ret = regulator_enable(ctx->supply);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to enable supply: %d\n", ret);
		return ret;
	}

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(50);

	return 0;
}

static const struct dev_pm_ops rocktech_hx8394_pm_ops = {
	SET_RUNTIME_PM_OPS(rocktech_hx8394_suspend, rocktech_hx8394_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static const struct of_device_id rocktech_hx8394_of_match[] = {
	{ .compatible = "rocktech,hx8394" },
	{ }
};
MODULE_DEVICE_TABLE(of, rocktech_hx8394_of_match);

static struct mipi_dsi_driver rocktech_hx8394_driver = {
	.probe = hx8394_probe,
	.remove = hx8394_remove,
	.driver = {
		.name = "panel-rocktech-hx8394",
		.of_match_table = rocktech_hx8394_of_match,
		.pm = &rocktech_hx8394_pm_ops,
	},
};
module_mipi_dsi_driver(rocktech_hx8394_driver);

MODULE_AUTHOR("Yannick Fertre <yannick.fertre@foss.st.com>");
MODULE_DESCRIPTION("DRM Driver for rocktech HX8394 MIPI DSI panel");
MODULE_LICENSE("GPL v2");
