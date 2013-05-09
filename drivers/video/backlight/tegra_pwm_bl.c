/*
 * linux/drivers/video/backlight/tegra_pwm_bl.c
 *
 * Tegra pwm backlight driver
 *
 * Copyright (C) 2011 NVIDIA Corporation
 * Author: Renuka Apte <rapte@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/tegra_pwm_bl.h>
#include <mach/dc.h>

//[ECID:0000] ZTEBSP maxiaoping 20120116 for MIMOSA backlight control,start
#if 0
static void __iomem *displaya_base = IO_ADDRESS(TEGRA_DISPLAY_BASE);
#define DC_COM_PIN_OUTPUT_SELECT5_0     0x319
#define DC_CMD_DISPLAY_POWER_CONTROL_0  0x36
#define DC_COM_PM1_CONTROL_0            0x31E
#define DC_COM_PM1_DUTY_CYCLE_0         0x31F
#endif
//[ECID:0000] ZTEBSP maxiaoping 20120116 for MIMOSA backlight control,end

struct tegra_pwm_bl_data {
	struct device *dev;
	int which_dc;
	struct tegra_dc_pwm_params params;
};

//[ECID:0000] ZTEBSP maxiaoping 20120110 disable test print log. ++
//[ECID:0000] ZTEBSP maxiaoping 20111209   add for tegra pwm backlight adjust.++

extern unsigned int fb_blank_flag;//[ECID:0000] ZTEBSP DangXiao 20120129 do not open bl when the fb is blank.
static int tegra_pwm_backlight_update_status(struct backlight_device *bl)
{
	struct tegra_pwm_bl_data *tbl = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;
	struct tegra_dc *dc;
	
	//printk(KERN_NOTICE "PM_DEBUG_MXP: Enter tegra_pwm_backlight_update_status\n");
	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (brightness > max)
	{
		dev_err(&bl->dev, "Invalid brightness value: %d max: %d\n",
		brightness, max);
		tbl->params.duty_cycle = 51;
	}
	//[ECID:0000] ZTEBSP DangXiao 20120129 start, do not open bl when the fb is blank.
	if (fb_blank_flag==0)
	{
		//printk("Ninja: set brightness to 0 (fb_blank_flag==0) =====\n");
		brightness = 0;
	}
	//[ECID:0000] ZTEBSP maxiaoping 20120320, for MIMOSA backlight control,increase brightness,start
	//[ECID:0000] ZTEBSP DangXiao 20120129 end, do not open bl when the fb is blank.
	//[ECID:0000] ZTEBSP maxiaoping 20120116 for MIMOSA backlight control,start
	//[ECID:0000] ZTEBSP maxiaoping 20120112 for MIMOSA backlight control,start
	//[ECID:0000] ZTEBSP maxiaoping 20120105 for MIMOSA backlight control ++
	/* map API brightness range from (0~255) to hw range (0~128) */
	//tbl->params.duty_cycle = (brightness * 128) / 255;
	else if((brightness>102)&&(brightness<=255))//brightness=255-->duty_cycle=51
	{
		tbl->params.duty_cycle = (brightness * 10)/51+1;
	}
	else if((brightness>=10)&&(brightness<=102))//brightness=30-->duty_cycle=5;brightness=102-->duty_cycle=21
	{
		//[ECID:0000] ZTEBSP maxiaoping 20120510, for MIMOSA backlight control,reduce min brightness intensity,start.
		// brightness=30-->duty_cycle=5
		//tbl->params.duty_cycle = (brightness *11)/72+6;//brightness=30-->duty_cycle=10
		tbl->params.duty_cycle = (brightness *16)/72-1;//brightness=30-->duty_cycle=5,brightness=10-->duty_cycle=1
		//[ECID:0000] ZTEBSP maxiaoping 20120510, for MIMOSA backlight control,reduce min brightness intensity,end.
	}
	else if(brightness<10)
	{
		tbl->params.duty_cycle = 0;
	}
	//printk(KERN_NOTICE "PM_DEBUG_MXP:brightness=%d .\r\n",brightness);
	//printk(KERN_NOTICE "PM_DEBUG_MXP:duty_cycle=%d .\r\n",tbl->params.duty_cycle);
	//tbl->params.duty_cycle =brightness;
	//tbl->params.period= ((0x3F<<18) | (1<<9) | (2<<0));
	//[ECID:0000] ZTEBSP maxiaoping 20120105 for MIMOSA backlight control --
	//[ECID:0000] ZTEBSP maxiaoping 20120112 for MIMOSA backlight control,end
	//[ECID:0000] ZTEBSP maxiaoping 20120320, for MIMOSA backlight control,increase brightness,end
	
	/* Call tegra display controller function to update backlight */
	dc = tegra_dc_get_dc(tbl->which_dc);
	if (dc)
		tegra_dc_config_pwm(dc, &tbl->params);
	else
		dev_err(&bl->dev, "tegra display controller not available\n");

	/* 5. Set duty cycle (default 210) */
         //writel(brightness, (displaya_base + DC_COM_PM1_DUTY_CYCLE_0));

	return 0;
}
//[ECID:0000] ZTEBSP maxiaoping 20111209   add for tegra pwm backlight adjust.++
//[ECID:0000] ZTEBSP maxiaoping 20120110 disable test print log. --
//[ECID:0000] ZTEBSP maxiaoping 20120206 for MIMOSA backlight control,end

static int tegra_pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops tegra_pwm_backlight_ops = {
	.update_status	= tegra_pwm_backlight_update_status,
	.get_brightness	= tegra_pwm_backlight_get_brightness,
};

static int tegra_pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_tegra_pwm_backlight_data *data;
	struct backlight_device *bl;
	struct tegra_pwm_bl_data *tbl;
	int ret;
	
	//[ECID:0000] ZTEBSP maxiaoping 2011205 add for tegra pwm backlight adjust.
	//[ECID:0000] ZTEBSP maxiaoping 20120110 disable test print log. ++
	printk(KERN_NOTICE "PM_DEBUG_MXP: Enter tegra_pwm_backlight_probe\n");
	data = pdev->dev.platform_data;
	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	tbl = kzalloc(sizeof(*tbl), GFP_KERNEL);
	if (!tbl) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	tbl->dev = &pdev->dev;
	tbl->which_dc = data->which_dc;
	tbl->params.which_pwm = data->which_pwm;
	tbl->params.period = data->period;
	tbl->params.clk_div = data->clk_div;
	tbl->params.clk_select = data->clk_select;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = data->max_brightness;
	props.type = BACKLIGHT_RAW; //[ECID:000000] ZTEBSP zhangbo add for ics
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, tbl,
			&tegra_pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	bl->props.brightness = data->dft_brightness;
	//[ECID:0000] ZTEBSP maxiaoping 20111209   add for tegra pwm backlight adjust.++
	//backlight_update_status(bl);
	//[ECID:0000] ZTEBSP maxiaoping 20111209   add for tegra pwm backlight adjust.--
	
	platform_set_drvdata(pdev, bl);
	//printk(KERN_NOTICE "PM_DEBUG_MXP: Exit tegra_pwm_backlight_probe\n");
	//[ECID:0000] ZTEBSP maxiaoping 20120110 disable test print log. --
	return 0;

err_bl:
	kfree(tbl);
err_alloc:
	return ret;
}

static int tegra_pwm_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct tegra_pwm_bl_data *tbl = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	kfree(tbl);
	return 0;
}

static struct platform_driver tegra_pwm_backlight_driver = {
	.driver		= {
		.name	= "tegra-pwm-bl",
		.owner	= THIS_MODULE,
	},
	.probe		= tegra_pwm_backlight_probe,
	.remove		= tegra_pwm_backlight_remove,
};

static int __init tegra_pwm_backlight_init(void)
{
	return platform_driver_register(&tegra_pwm_backlight_driver);
}
module_init(tegra_pwm_backlight_init);

static void __exit tegra_pwm_backlight_exit(void)
{
	platform_driver_unregister(&tegra_pwm_backlight_driver);
}
module_exit(tegra_pwm_backlight_exit);

MODULE_DESCRIPTION("Tegra PWM Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tegra-pwm-backlight");


