/*
 * LCD panel driver for UMT
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <plat/display.h>

struct UMT_data {
	struct backlight_device *bl;
};

static struct omap_video_timings UMT_ls_timings = {
		.x_res = 640,
		.y_res = 480,

		.pixel_clock	= 28000,

		.hsw		= 63,
		.hfp		= 16,
		.hbp		= 48,

		.vsw		= 3,
		.vfp		= 12,
		.vbp		= 32,
};

static int UMT_ls_bl_update_status(struct backlight_device *bl)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bl->dev);
	int level;
	int	rc;

	if (!dssdev->set_backlight) {
printk("%s:%d\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		level = bl->props.brightness;
	else
		level = 0;

	//return dssdev->set_backlight(dssdev, level);
	rc = dssdev->set_backlight(dssdev, level);
printk("%s:%d rc = %d\n", __func__, __LINE__, rc);
	return rc;
}

static int UMT_ls_bl_get_brightness(struct backlight_device *bl)
{
	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		return bl->props.brightness;

	return 0;
}

static const struct backlight_ops UMT_ls_bl_ops = {
	.get_brightness = UMT_ls_bl_get_brightness,
	.update_status  = UMT_ls_bl_update_status,
};



static int UMT_ls_panel_probe(struct omap_dss_device *dssdev)
{
	struct backlight_properties props;
	struct backlight_device *bl;
	struct UMT_data *sd;
	int r;

	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS;
	dssdev->panel.acb = 0x28;
	dssdev->panel.timings = UMT_ls_timings;

#if 0
	sd = kzalloc(sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	dev_set_drvdata(&dssdev->dev, sd);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = dssdev->max_backlight_level;

	bl = backlight_device_register("UMT-ls", &dssdev->dev, dssdev,
			&UMT_ls_bl_ops, &props);
	if (IS_ERR(bl)) {
		r = PTR_ERR(bl);
		kfree(sd);
		return r;
	}
	sd->bl = bl;

	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.brightness = dssdev->max_backlight_level;
	r = UMT_ls_bl_update_status(bl);
printk("%s:%d\n", __func__, __LINE__);
	if (r < 0)
		dev_err(&dssdev->dev, "failed to set lcd brightness\n");
#endif

	return 0;
}

static void UMT_ls_panel_remove(struct omap_dss_device *dssdev)
{
}

static int UMT_ls_power_on(struct omap_dss_device *dssdev)
{
	int r = 0;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	/* wait couple of vsyncs until enabling the LCD */
	msleep(50);

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void UMT_ls_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(100);

	omapdss_dpi_display_disable(dssdev);
}

static int UMT_ls_panel_enable(struct omap_dss_device *dssdev)
{
	int r;
	r = UMT_ls_power_on(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	return r;
}

static void UMT_ls_panel_disable(struct omap_dss_device *dssdev)
{
	UMT_ls_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int UMT_ls_panel_suspend(struct omap_dss_device *dssdev)
{
	UMT_ls_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int UMT_ls_panel_resume(struct omap_dss_device *dssdev)
{
	int r;
	r = UMT_ls_power_on(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	return r;
}

static struct omap_dss_driver UMT_ls_driver = {
	.probe		= UMT_ls_panel_probe,
	.remove		= UMT_ls_panel_remove,

	.enable		= UMT_ls_panel_enable,
	.disable	= UMT_ls_panel_disable,
	.suspend	= UMT_ls_panel_suspend,
	.resume		= UMT_ls_panel_resume,

	.driver         = {
		.name   = "URT_UMSH8272MD",
		.owner  = THIS_MODULE,
	},
};

static int __init UMT_ls_panel_drv_init(void)
{
	return omap_dss_register_driver(&UMT_ls_driver);
}

static void __exit UMT_ls_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&UMT_ls_driver);
}

module_init(UMT_ls_panel_drv_init);
module_exit(UMT_ls_panel_drv_exit);
MODULE_LICENSE("GPL");
