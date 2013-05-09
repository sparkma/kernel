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
#include <linux/tps6586x_pwm_bl.h>
#include <mach/dc.h>
/*[ECID:000000] ZTEBSP zhangbo 20120110, trun off lcd backlight while rebooting, start*/
#include <linux/notifier.h>
#include <linux/reboot.h>
/*[ECID:000000] ZTEBSP zhangbo 20120110, trun off lcd backlight while rebooting, end*/

struct tps6586x_pwm_bl_data {
	struct device *dev;
        int (*backlight_notify)(struct device *unused, int brightness);
};
/*[ECID:000000] ZTEBP zhangbo 20120116, trun off lcd backlight while rebooting, start*/
int tps6586x_write_register(int reg, unsigned int val);
static void pmu_lcd_backlight_off(void)
{
		  tps6586x_write_register(0x57, 0);
                tps6586x_write_register(0x58, 0);
}
static int tegra_lcd_backlight_notify(struct notifier_block *this,
			    unsigned long code, void *dev)
{
	pmu_lcd_backlight_off();
	return NOTIFY_DONE;
}
static struct notifier_block tegra_lcd_backlight_nb = {
	.notifier_call = tegra_lcd_backlight_notify,
};
/*[ECID:000000] ZTEBP zhangbo 20120116, trun off lcd backlight while rebooting, end*/

static int tps6586x_pwm_backlight_update_status(struct backlight_device *bl)
{
	struct tps6586x_pwm_bl_data *tbl = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;

	//[ECID:0000] ZTEBSP maxiaoping 20111027 add for pmu backlight adjust.
	printk(KERN_NOTICE "PM_DEBUG_MXP: Enter pwm_backlight_update_status\n");
	
	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (brightness > max)
		dev_err(&bl->dev, "Invalid brightness value: %d max: %d\n",
		brightness, max);

	/* Call tps6586x register R/W function to update backlight */
        tbl->backlight_notify(NULL, brightness);

	return 0;
}

static int tps6586x_pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops tps6586x_pwm_backlight_ops = {
	.update_status	= tps6586x_pwm_backlight_update_status,
	.get_brightness	= tps6586x_pwm_backlight_get_brightness,
};

static int tps6586x_pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_tps6586x_pwm_backlight_data *data;
	struct backlight_device *bl;
	struct tps6586x_pwm_bl_data *tbl;
	int ret;
	
	//[ECID:0000] ZTEBSP maxiaoping 20111027 add for pmu backlight adjust.
	printk(KERN_NOTICE "PM_DEBUG_MXP: Enter pwm_backlight_probe\n");
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
        tbl->backlight_notify = data->notify;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, tbl,
			&tps6586x_pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);

	/*[ECID:000000] ZTEBP zhangbo 20120116, trun off lcd backlight while rebooting, start*/
	register_reboot_notifier(&tegra_lcd_backlight_nb);
	atomic_notifier_chain_register(&panic_notifier_list, &tegra_lcd_backlight_nb);
	/*[ECID:000000] ZTEBP zhangbo 20120116, trun off lcd backlight while rebooting, end*/
	

        printk("%s successfully. \n", __FUNCTION__);

	return 0;

err_bl:
	kfree(tbl);
err_alloc:
	return ret;
}

static int tps6586x_pwm_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct tps6586x_pwm_bl_data *tbl = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	kfree(tbl);
	/*[ECID:000000] ZTEBP zhangbo 20120116, trun off lcd backlight while rebooting, start*/
	unregister_reboot_notifier(&tegra_lcd_backlight_nb);
	atomic_notifier_chain_unregister(&panic_notifier_list, &tegra_lcd_backlight_nb);
	/*[ECID:000000] ZTEBP zhangbo 20120116, trun off lcd backlight while rebooting, end*/
	return 0;
}

static struct platform_driver tps6586x_pwm_backlight_driver = {
	.driver		= {
		.name	= "tps6586x-pwm-bl",
		.owner	= THIS_MODULE,
	},
	.probe		= tps6586x_pwm_backlight_probe,
	.remove		= tps6586x_pwm_backlight_remove,
};

static int __init tps6586x_pwm_backlight_init(void)
{
	return platform_driver_register(&tps6586x_pwm_backlight_driver);
}
module_init(tps6586x_pwm_backlight_init);

static void __exit tps6586x_pwm_backlight_exit(void)
{
	platform_driver_unregister(&tps6586x_pwm_backlight_driver);
}
module_exit(tps6586x_pwm_backlight_exit);

MODULE_DESCRIPTION("TPS6586X PWM Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tps6586x-pwm-backlight");

