/*
 * Copyright (C) 2012 ZTE
 *
 *[ECID:000000] ZTEBSP zhangbo173794 add this file for led driver
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <mach/dc.h>


/*
bit: function
0: red_enable
1: green_enable
2: blink enable
*/
//#define RED_GREEN_EXCHANGE

#define RED_ENABLE 0x10
#define GREEN_ENABLE 0x20
#define RED_BRINK_ENABLE 0x40
#define GREEN_BRINK_ENABLE 0x80

 int zte_tps6586x_read(int reg);
 int zte_tps6586x_write(int reg,uint8_t val);


struct tps6586x_led_bl_data {
	struct device *dev;
        int (*backlight_notify)(struct device *unused, int brightness);
};

struct platform_tps6586x_led_backlight_data {
	unsigned int max_brightness;
	unsigned int dft_brightness;
	int (*notify)(struct device *unused, int brightness);
};

int zte_tps6586x_led_red(void)
{
	int val = 0;
	int ret=0;

	//pr_info("%s entered\n", __func__);
	
	//³äµçÖ¸Ê¾µÆ
	val = zte_tps6586x_read(0x50);
	val &=~0x7f;
	val |= 0x0f;
	ret = zte_tps6586x_write(0x50,  (uint8_t)val);
	
	ret = zte_tps6586x_write( 0x51,  0x3f);
 
	ret = zte_tps6586x_write( 0x52,  0x80);

	return ret;
}

int zte_tps6586x_led_green(void)
{
	int val = 0;
	int ret=0;

	//pr_info("%s entered\n", __func__);
		
	//ÉèÖÃÂÌµÆÁÁ,ºìµÆÃð
	val = zte_tps6586x_read(0x50);
	val &=~0x7f;
	val |= 0x0f;
	ret = zte_tps6586x_write(0x50,  val);
	
	ret = zte_tps6586x_write(0x51,  0x20);
	
	ret = zte_tps6586x_write(0x52,  0x9f);

	return ret;
}

int zte_tps6586x_led_blue(void)
{
}

int zte_tps6586x_led_off(void)
{
	int ret=0;

	//pr_info("%s entered\n", __func__);
	ret = zte_tps6586x_write(0x52,  0x00);

	return ret;
}

int zte_tps6586x_led_red_blink(void)
{
	int val = 0;
	int ret=0;

	//pr_info("%s entered\n", __func__);
	
	val = zte_tps6586x_read(0x50);
	val &=~0x7f;
	val |= 0x02;
	ret = zte_tps6586x_write(0x50,  (uint8_t)val);
	
	ret = zte_tps6586x_write( 0x51,  0x3f);
 
	ret = zte_tps6586x_write( 0x52,  0x80);

	return ret;
}

int zte_tps6586x_led_green_blink(void)
{
	int val = 0;
	int ret=0;
	
	//pr_info("%s entered\n", __func__);
	val = zte_tps6586x_read(0x50);
	val &=~0x7f;
	val |= 0x02;
	ret = zte_tps6586x_write(0x50,  (uint8_t)val);
	
	ret = zte_tps6586x_write( 0x51,  0x20);
 
	ret = zte_tps6586x_write( 0x52,  0x9f);

	return ret;
}

int pmu_led_backlight_notify(struct device *unused, int brightness)
{
	if(brightness&RED_ENABLE)
	{
		if(brightness&RED_BRINK_ENABLE)
		{
			#ifdef RED_GREEN_EXCHANGE
			zte_tps6586x_led_green_blink();
			#else
			zte_tps6586x_led_red_blink();
			#endif
		}
		else
		{
			#ifdef RED_GREEN_EXCHANGE
			zte_tps6586x_led_green();
			#else
			zte_tps6586x_led_red();
			#endif
		}			
	}
	else if(brightness&GREEN_ENABLE)
	{
		if(brightness&GREEN_BRINK_ENABLE)
		{
			#ifdef RED_GREEN_EXCHANGE
			zte_tps6586x_led_red_blink();
			#else
			zte_tps6586x_led_green_blink();
			#endif
		}
		else
		{
			#ifdef RED_GREEN_EXCHANGE
			zte_tps6586x_led_red();
			#else
			zte_tps6586x_led_green();
			#endif
		}
	}
	else
	{
		zte_tps6586x_led_off();
	}
}

static int tps6586x_led_backlight_update_status(struct backlight_device *bl)
{
	struct tps6586x_led_bl_data *tbl = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;

	//pr_info("%s entered, brightness=%d\n", __func__,brightness);
	
	if (brightness > max)
		dev_err(&bl->dev, "Invalid brightness value: %d max: %d\n",
		brightness, max);

	/* Call tps6586x register R/W function to update backlight */
        tbl->backlight_notify(NULL, brightness);

	return 0;
}

static int tps6586x_led_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops tps6586x_led_backlight_ops = {
	.update_status	= tps6586x_led_backlight_update_status,
	.get_brightness	= tps6586x_led_backlight_get_brightness,
};

static int tps6586x_led_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_tps6586x_led_backlight_data *data;
	struct backlight_device *bl;
	struct tps6586x_led_bl_data *tbl;
	int ret;
	
	printk(KERN_NOTICE "tps6586x_led_backlight_probe\n");
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
	props.type = BACKLIGHT_RAW; //[ECID:000000] ZTEBSP zhangbo add for ics
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, tbl,
			&tps6586x_led_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);

        printk("%s successfully. \n", __FUNCTION__);

	return 0;

err_bl:
	kfree(tbl);
err_alloc:
	return ret;
}

static int tps6586x_led_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct tps6586x_led_bl_data *tbl = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	kfree(tbl);
	return 0;
}

static struct platform_driver tps6586x_led_backlight_driver = {
	.driver		= {
		.name	= "tegra_led",
		.owner	= THIS_MODULE,
	},
	.probe		= tps6586x_led_backlight_probe,
	.remove		= tps6586x_led_backlight_remove,
};

static int __init tps6586x_led_backlight_init(void)
{
	return platform_driver_register(&tps6586x_led_backlight_driver);
}
module_init(tps6586x_led_backlight_init);

static void __exit tps6586x_led_backlight_exit(void)
{
	platform_driver_unregister(&tps6586x_led_backlight_driver);
}
module_exit(tps6586x_led_backlight_exit);

MODULE_DESCRIPTION("TPS6586X LED Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tps6586x-led-backlight");

