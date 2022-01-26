// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT36523 DriverIC panels driver
 *
 * Copyright (c) 2022 Jianhua Lu <lujianhua000@gmail.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

enum dsi_cmd_type {
	INIT_DCS_CMD,
	DELAY_CMD,
};

struct panel_init_cmd {
	enum dsi_cmd_type type;
	size_t len;
	const char *data;
};

#define _INIT_DCS_CMD(...) { \
	.type = INIT_DCS_CMD, \
	.len = sizeof((char[]){__VA_ARGS__}), \
	.data = (char[]){__VA_ARGS__} }

#define _INIT_DELAY_CMD(...) { \
	.type = DELAY_CMD,\
	.len = sizeof((char[]){__VA_ARGS__}), \
	.data = (char[]){__VA_ARGS__} }

struct panel_desc {
	const struct drm_display_mode *modes;
	const struct mipi_dsi_device_info dsi_info;

	unsigned int bpc;

	unsigned int width_mm;
	unsigned int height_mm;

	enum mipi_dsi_pixel_format format;
	unsigned long mode_flags;

	bool is_dual_dsi;

	unsigned int lanes;
};

static const char * const nt36523_regulator_names[] = {
	"vddio",
	"avdd",
	"avee",
};

static unsigned long const nt36523_regulator_enable_loads[] = {
	62000,
	100000,
	100000
};

struct panel_info {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	const struct panel_desc *desc;
	struct regulator_bulk_data supplies[ARRAY_SIZE(nt36523_regulator_names)];

	struct backlight_device *backlight;

	struct gpio_desc *reset_gpio;
	bool prepared;
};

static inline struct panel_info *to_panel_info(struct drm_panel *panel)
{
	return container_of(panel, struct panel_info, panel);
}

static int send_panel_init_cmds(struct drm_panel *panel, const struct panel_init_cmd *cmds)
{
	struct panel_info *pinfo = to_panel_info(panel);
	unsigned int i = 0;
	int err;

	if (!cmds)
		return -EFAULT;

	for (i = 0; cmds[i].len != 0; i++) {
		const struct panel_init_cmd *cmd = &cmds[i];

		switch(cmd->type) {
			case DELAY_CMD:
				msleep(cmd->data[0]);
				err = 0;
				break;
			case INIT_DCS_CMD:
				err = mipi_dsi_dcs_write(pinfo->dsi[0], cmd->data[0],
						cmd->len <= 1 ? NULL : &cmd->data[1],
						cmd->len - 1);
				if(pinfo->desc->is_dual_dsi) {
					if (err < 0) {
						dev_err(panel->dev,
								"failed to write command %u\n", i);
						return err;
					}
					err = mipi_dsi_dcs_write(pinfo->dsi[1], cmd->data[0],
							cmd->len <= 1 ? NULL : &cmd->data[1],
							cmd->len - 1);
				}
				break;
			default:
				err = -EINVAL;

		}
		if (err < 0) {
			dev_err(panel->dev,
					"failed to write command %u\n", i);
			return err;
		}
	}

	return 0;
}

static const struct panel_init_cmd nt36523_on_cmd[] = {
	_INIT_DCS_CMD(0xFF, 0x10),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0xB9, 0x05),
	_INIT_DCS_CMD(0xFF, 0x20),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x18, 0x40),
	_INIT_DCS_CMD(0xFF, 0x10),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0xB9, 0x02),
	_INIT_DCS_CMD(0xFF, 0xD0),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x02, 0xAF),
	_INIT_DCS_CMD(0x00, 0x30),
	_INIT_DCS_CMD(0x09, 0xEE),
	_INIT_DCS_CMD(0x1C, 0x99),
	_INIT_DCS_CMD(0x1D, 0x09),
	_INIT_DCS_CMD(0xFF, 0xF0),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x3A, 0x08),
	_INIT_DCS_CMD(0xFF, 0xE0),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x4F, 0x02),
	_INIT_DCS_CMD(0xFF, 0x20),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x58, 0x40),
	_INIT_DCS_CMD(0xFF, 0x10),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x35, 0x00),
	/* CABC SETTING CFG Start*/
	_INIT_DCS_CMD(0xFF, 0x23),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x00, 0x80),
	_INIT_DCS_CMD(0x01, 0x84),
	_INIT_DCS_CMD(0x05, 0x2D),
	_INIT_DCS_CMD(0x06, 0x00),
	_INIT_DCS_CMD(0x07, 0x00),
	_INIT_DCS_CMD(0x08, 0x01),
	_INIT_DCS_CMD(0x09, 0x45),
	_INIT_DCS_CMD(0x11, 0x02),
	_INIT_DCS_CMD(0x12, 0x80),
	_INIT_DCS_CMD(0x15, 0x83),
	_INIT_DCS_CMD(0x16, 0x0C),
	_INIT_DCS_CMD(0x29, 0x0A),
	_INIT_DCS_CMD(0x30, 0xFF),
	_INIT_DCS_CMD(0x31, 0xFE),
	_INIT_DCS_CMD(0x32, 0xFD),
	_INIT_DCS_CMD(0x33, 0xFB),
	_INIT_DCS_CMD(0x34, 0xF8),
	_INIT_DCS_CMD(0x35, 0xF5),
	_INIT_DCS_CMD(0x36, 0xF3),
	_INIT_DCS_CMD(0x37, 0xF2),
	_INIT_DCS_CMD(0x38, 0xF2),
	_INIT_DCS_CMD(0x39, 0xF2),
	_INIT_DCS_CMD(0x3A, 0xEF),
	_INIT_DCS_CMD(0x3B, 0xEC),
	_INIT_DCS_CMD(0x3D, 0xE9),
	_INIT_DCS_CMD(0x3F, 0xE5),
	_INIT_DCS_CMD(0x40, 0xE5),
	_INIT_DCS_CMD(0x41, 0xE5),
	_INIT_DCS_CMD(0x2A, 0x13),
	_INIT_DCS_CMD(0x45, 0xFF),
	_INIT_DCS_CMD(0x46, 0xF4),
	_INIT_DCS_CMD(0x47, 0xE7),
	_INIT_DCS_CMD(0x48, 0xDA),
	_INIT_DCS_CMD(0x49, 0xCD),
	_INIT_DCS_CMD(0x4A, 0xC0),
	_INIT_DCS_CMD(0x4B, 0xB3),
	_INIT_DCS_CMD(0x4C, 0xB2),
	_INIT_DCS_CMD(0x4D, 0xB2),
	_INIT_DCS_CMD(0x4E, 0xB2),
	_INIT_DCS_CMD(0x4F, 0x99),
	_INIT_DCS_CMD(0x50, 0x80),
	_INIT_DCS_CMD(0x51, 0x68),
	_INIT_DCS_CMD(0x52, 0x66),
	_INIT_DCS_CMD(0x53, 0x66),
	_INIT_DCS_CMD(0x54, 0x66),
	_INIT_DCS_CMD(0x2B, 0x0E),
	_INIT_DCS_CMD(0x58, 0xFF),
	_INIT_DCS_CMD(0x59, 0xFB),
	_INIT_DCS_CMD(0x5A, 0xF7),
	_INIT_DCS_CMD(0x5B, 0xF3),
	_INIT_DCS_CMD(0x5C, 0xEF),
	_INIT_DCS_CMD(0x5D, 0xE3),
	_INIT_DCS_CMD(0x5E, 0xDA),
	_INIT_DCS_CMD(0x5F, 0xD8),
	_INIT_DCS_CMD(0x60, 0xD8),
	_INIT_DCS_CMD(0x61, 0xD8),
	_INIT_DCS_CMD(0x62, 0xCB),
	_INIT_DCS_CMD(0x63, 0xBF),
	_INIT_DCS_CMD(0x64, 0xB3),
	_INIT_DCS_CMD(0x65, 0xB2),
	_INIT_DCS_CMD(0x66, 0xB2),
	_INIT_DCS_CMD(0x67, 0xB2),
	_INIT_DCS_CMD(0xFF, 0x10),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x51, 0x0F, 0xFF),
	_INIT_DCS_CMD(0x53, 0x2C),
	/* CABC SETTING CFG END */
	_INIT_DCS_CMD(0x55, 0x00),
	_INIT_DCS_CMD(0xBB, 0x13),
	_INIT_DCS_CMD(0x3B, 0x03, 0xAC, 0x1A, 0x04, 0x04),
	/* 30HZ pen code Start*/
	_INIT_DCS_CMD(0xFF, 0x2A),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x25, 0x46),
	_INIT_DCS_CMD(0x30, 0x46),
	_INIT_DCS_CMD(0x39, 0x46),
	_INIT_DCS_CMD(0xFF, 0x26),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x01, 0xB0),
	_INIT_DCS_CMD(0x19, 0x10),
	_INIT_DCS_CMD(0x1A, 0xE0),
	_INIT_DCS_CMD(0x1B, 0x10),
	_INIT_DCS_CMD(0x1C, 0x00),
	_INIT_DCS_CMD(0x2A, 0x10),
	_INIT_DCS_CMD(0x2B, 0xE0),
	/* 30HZ pen code END*/
	/* ESD code Start*/
	_INIT_DCS_CMD(0xFF, 0xF0),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x84, 0x08),
	_INIT_DCS_CMD(0x85, 0x0C),
	_INIT_DCS_CMD(0xFF, 0x20),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x51, 0x00),
	_INIT_DCS_CMD(0xFF, 0x25),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x91, 0x1F),
	_INIT_DCS_CMD(0x92, 0x0F),
	_INIT_DCS_CMD(0x93, 0x01),
	_INIT_DCS_CMD(0x94, 0x18),
	_INIT_DCS_CMD(0x95, 0x03),
	_INIT_DCS_CMD(0x96, 0x01),
	_INIT_DCS_CMD(0xFF, 0x10),
	_INIT_DCS_CMD(0xB0, 0x01),
	_INIT_DCS_CMD(0xFF, 0x25),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x19, 0x1F),
	_INIT_DCS_CMD(0x1B, 0x1B),
	_INIT_DCS_CMD(0xFF, 0x24),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0xB8, 0x28),
	_INIT_DCS_CMD(0xFF, 0x27),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0xD0, 0x31),
	_INIT_DCS_CMD(0xD1, 0x20),
	_INIT_DCS_CMD(0xD4, 0x08),
	_INIT_DCS_CMD(0xDE, 0x80),
	_INIT_DCS_CMD(0xDF, 0x02),
	_INIT_DCS_CMD(0xFF, 0x26),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x00, 0x81),
	_INIT_DCS_CMD(0x01, 0xB0),
	_INIT_DCS_CMD(0xFF, 0x22),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x6F, 0x01),
	_INIT_DCS_CMD(0x70, 0x11),
	_INIT_DCS_CMD(0x73, 0x01),
	_INIT_DCS_CMD(0x74, 0x4D),
	_INIT_DCS_CMD(0xA0, 0x3F),
	_INIT_DCS_CMD(0xA9, 0x50),
	_INIT_DCS_CMD(0xAA, 0x28),
	_INIT_DCS_CMD(0xAB, 0x28),
	_INIT_DCS_CMD(0xAD, 0x10),
	_INIT_DCS_CMD(0xB8, 0x00),
	_INIT_DCS_CMD(0xB9, 0x4B),
	_INIT_DCS_CMD(0xBA, 0x96),
	_INIT_DCS_CMD(0xBB, 0x4B),
	_INIT_DCS_CMD(0xBE, 0x07),
	_INIT_DCS_CMD(0xBF, 0x4B),
	_INIT_DCS_CMD(0xC0, 0x07),
	_INIT_DCS_CMD(0xC1, 0x5C),
	_INIT_DCS_CMD(0xC2, 0x00),
	_INIT_DCS_CMD(0xC5, 0x00),
	_INIT_DCS_CMD(0xC6, 0x3F),
	_INIT_DCS_CMD(0xC7, 0x00),
	_INIT_DCS_CMD(0xCA, 0x08),
	_INIT_DCS_CMD(0xCB, 0x40),
	_INIT_DCS_CMD(0xCE, 0x00),
	_INIT_DCS_CMD(0xCF, 0x08),
	_INIT_DCS_CMD(0xD0, 0x40),
	_INIT_DCS_CMD(0xD3, 0x08),
	_INIT_DCS_CMD(0xD4, 0x40),
	_INIT_DCS_CMD(0xFF, 0x25),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0xBC, 0x01),
	_INIT_DCS_CMD(0xBD, 0x1C),
	_INIT_DCS_CMD(0xFF, 0x2A),
	_INIT_DCS_CMD(0xFB, 0x01),
	_INIT_DCS_CMD(0x9A, 0x03),
	_INIT_DCS_CMD(0xFF, 0x10),
	_INIT_DCS_CMD(0x11),
	_INIT_DELAY_CMD(70),
	_INIT_DCS_CMD(0x29),
	{},
};


static void nt36523_reset(struct panel_info *pinfo)
{
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	usleep_range(12000, 13000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(12000, 13000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	usleep_range(12000, 13000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(12000, 13000);
}

static int nt36523_off(struct panel_info *pinfo)
{
	struct device *dev = &pinfo->dsi[0]->dev;
	int ret;

	ret = mipi_dsi_dcs_set_display_off(pinfo->dsi[0]);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
	}

	if (pinfo->desc->is_dual_dsi) {
		ret = mipi_dsi_dcs_set_display_off(pinfo->dsi[1]);
		if (ret < 0) {
			dev_err(dev, "Failed to set display off: %d\n", ret);
		}
	}

	ret = mipi_dsi_dcs_enter_sleep_mode(pinfo->dsi[0]);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
	}

	if (pinfo->desc->is_dual_dsi) {
		ret = mipi_dsi_dcs_enter_sleep_mode(pinfo->dsi[1]);
		if (ret < 0) {
			dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		}
	}
	msleep(70);

	return 0;
}

static int nt36523_prepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	struct device *dev = &pinfo->dsi[0]->dev;
	int ret;

	if (pinfo->prepared)
		return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(pinfo->supplies), pinfo->supplies);
	if (ret < 0)
		return ret;

	nt36523_reset(pinfo);

	ret = send_panel_init_cmds(&pinfo->panel, nt36523_on_cmd);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		goto end;
	}

	pinfo->prepared = true;

end:
	if (ret < 0) {
		regulator_bulk_disable(ARRAY_SIZE(pinfo->supplies), pinfo->supplies);
		return ret;
	}

	return 0;
}

static int nt36523_unprepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	struct device *dev = &pinfo->dsi[0]->dev;
	int ret;

	if (!pinfo->prepared)
		return 0;

	ret = nt36523_off(pinfo);
	if (ret < 0)
		dev_err(dev, "Failed to deinitialize panel: %d\n", ret);

	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(pinfo->supplies), pinfo->supplies);

	pinfo->prepared = false;
	return 0;
}

static int nt36523_remove(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(pinfo->dsi[0]);
	if (ret < 0)
		dev_err(&dsi->dev,
			"Failed to detach from DSI0 host: %d\n", ret);

	if (pinfo->dsi[1]) {
		ret = mipi_dsi_detach(pinfo->dsi[1]);
		if (ret < 0)
			dev_err(&dsi->dev,
				"Failed to detach from DSI1 host: %d\n", ret);
		mipi_dsi_device_unregister(pinfo->dsi[1]);
	}

	drm_panel_remove(&pinfo->panel);

	return 0;
}

static const struct drm_display_mode nt36523_mode = {
	.clock = (1600 + 200 + 40 + 52) * (2560 + 26 + 4 + 168) * 104 / 1000,
	.hdisplay = 1600,
	.hsync_start = 1600 + 200,
	.hsync_end = 1600 + 200 + 40,
	.htotal = 1600 + 200 + 40 + 52,
	.vdisplay = 2560,
	.vsync_start = 2560 + 26,
	.vsync_end = 2560 + 26 + 4,
	.vtotal = 2560 + 26 + 4 + 168,
};

static const struct panel_desc nt36523_desc = {
	.modes = &nt36523_mode,
	.dsi_info = {
		.type = "NT36523",
		.channel = 0,
		.node = NULL,
	},
	.width_mm = 1474,
	.height_mm = 2359,
	.bpc = 8,
	.lanes = 3,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.is_dual_dsi = true,
};

static int nt36523_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct panel_info *pinfo = to_panel_info(panel);
	const struct drm_display_mode *m = pinfo->desc->modes;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, m);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			m->hdisplay, m->vdisplay, drm_mode_vrefresh(m));
		return -ENOMEM;
	}

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = pinfo->desc->width_mm;
	connector->display_info.height_mm = pinfo->desc->height_mm;
	connector->display_info.bpc = pinfo->desc->bpc;

	return 1;
}

static const struct drm_panel_funcs nt36523_panel_funcs = {
	.prepare = nt36523_prepare,
	.unprepare = nt36523_unprepare,
	.get_modes = nt36523_get_modes,
};

static int nt36523_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_r;
	struct mipi_dsi_host *dsi_r_host;
	struct panel_info *pinfo;
	const struct mipi_dsi_device_info *info;
	int i, dsi_count = 1, ret;

	pinfo = devm_kzalloc(dev, sizeof(*pinfo), GFP_KERNEL);
	if (!pinfo)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(pinfo->supplies); i++)
		pinfo->supplies[i].supply = nt36523_regulator_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(pinfo->supplies),
				      pinfo->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	for (i = 0; i < ARRAY_SIZE(pinfo->supplies); i++) {
		ret = regulator_set_load(pinfo->supplies[i].consumer,
					 nt36523_regulator_enable_loads[i]);
		if (ret)
			return dev_err_probe(dev, ret, "failed to set regulator enable loads\n");
	}

	pinfo->desc = of_device_get_match_data(dev);
	if (!pinfo->desc)
		return -ENODEV;

	pinfo->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pinfo->reset_gpio)) {
		return dev_err_probe(dev, PTR_ERR(pinfo->reset_gpio),
				     "Failed to get reset gpio\n");
	}

	/* If the panel is connected on two DSIs then DSI0 left, DSI1 right */
	if (pinfo->desc->is_dual_dsi) {
		info = &pinfo->desc->dsi_info;
		dsi_r = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
		if (!dsi_r) {
			dev_err(dev, "Cannot get secondary DSI node.\n");
			return -ENODEV;
		}
		dsi_r_host = of_find_mipi_dsi_host_by_node(dsi_r);
		of_node_put(dsi_r);
		if (!dsi_r_host) {
			dev_err(dev, "Cannot get secondary DSI host\n");
			return -EPROBE_DEFER;
		}

		pinfo->dsi[1] = mipi_dsi_device_register_full(dsi_r_host, info);
		if (!pinfo->dsi[1]) {
			dev_err(dev, "Cannot get secondary DSI node\n");
			return -ENODEV;
		}
		dsi_count++;
	}

	pinfo->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, pinfo);

	drm_panel_init(&pinfo->panel, dev, &nt36523_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&pinfo->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&pinfo->panel);

	for (i = 0; i < dsi_count; i++) {
		pinfo->dsi[i]->lanes = pinfo->desc->lanes;
		pinfo->dsi[i]->format = pinfo->desc->format;
		pinfo->dsi[i]->mode_flags = pinfo->desc->mode_flags;

		ret = mipi_dsi_attach(pinfo->dsi[i]);
		if (ret < 0) {
			return dev_err_probe(dev, ret,
					     "Cannot attach to DSI%d host.\n", i);
		}
	}

	return 0;
}

static const struct of_device_id nt36523_of_match[] = {
	{ .compatible = "novatek,nt36523", .data = &nt36523_desc },
	{}
};
MODULE_DEVICE_TABLE(of, nt36523_of_match);

static struct mipi_dsi_driver nt36523_driver = {
	.probe = nt36523_probe,
	.remove = nt36523_remove,
	.driver = {
		.name = "panel-novatek-nt36523",
		.of_match_table = nt36523_of_match,
	},
};
module_mipi_dsi_driver(nt36523_driver);

MODULE_AUTHOR("Jianhua Lu <lujianhua000@gmail.com>");
MODULE_DESCRIPTION("Novatek NT36523 DriverIC panels driver");
MODULE_LICENSE("GPL v2");
