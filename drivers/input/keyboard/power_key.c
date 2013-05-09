/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 
//[ECID:000000] ZTEBSP DangXiao 20120119,get this pwrkey patch from NV Martin


#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <asm/gpio.h>
#include <linux/io.h>
#include <mach/iomap.h>
#include <linux/i2c.h>
#include <linux/mfd/tps6586x.h>
#include <linux/pwr_key.h>

static void __iomem *pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);
#define PMC_WAKE_STATUS 0x14
#define PMC_WAKE_KEY_MASK (0x1 << 24)

#define DRIVER_NAME "power_key"

static struct platform_device *ppdev = NULL;

struct power_key {
        /* Configuration parameters */
        int code;               /* input event code (KEY_*, SW_*) */
        int gpio;
        int active_level;
        char *desc;
        int type;               /* input event type (EV_KEY, EV_SW) */
        int wakeup;             /* configure the key as a wake-up source */
        int delay;
};

struct power_key_drvdata {
        struct input_dev *input;
        struct work_struct work;
        struct power_key data[0];
};

extern int tps6586x_write_register(int reg, unsigned char val);
extern int tps6586x_read_register(int reg, unsigned char *val);

int ForceNormalMode(struct device *dev, bool on)
{
        unsigned char buff = 0;
        int err;

        err = tps6586x_read_register(0x14, &buff);
        if (err < 0) {
                dev_err(dev, "failed to read \n");
                return err;
        }
        if (on)
                buff = buff | 0x2;
        else
                buff = buff & (~0x2);

        err = tps6586x_write_register(0x14, buff);
        if (err < 0) {
                dev_err(dev, "failed to write \n");
                return err;
        }
        return 0;
}

static void power_key_report_event(struct work_struct *work)
{
        struct power_key_drvdata *bdata =
                container_of(work, struct power_key_drvdata, work);
        struct power_key *pwrkey = bdata->data;
        unsigned int code = pwrkey->code;
        int current_state = 0;
        static int expect_state = 0;
        int m_delay = pwrkey->delay;

        /* Get current status */
        /* 0 means press; 1 means release */
        current_state = gpio_get_value(pwrkey->gpio);

        if (current_state != expect_state) {
                if (current_state == 1) {
                        input_report_key(bdata->input, code, current_state);
                        input_sync(bdata->input);
                } else {
                        return;
                }
        }

        input_report_key(bdata->input, code, !current_state);
		printk("Got Power_Key here! current_state: %d !(0:press,1:release)=== \n",current_state);  //ZTEBSP DangXiao add
        input_sync(bdata->input);

        if (current_state) {
                ForceNormalMode(&ppdev->dev, false);
                expect_state = 0;
        } else {
                ForceNormalMode(&ppdev->dev, true);
                expect_state = 1;
        }
}

static irqreturn_t power_key_isr(int irq, void *dev_id)
{
        struct power_key_drvdata *ddata = dev_id;
        struct power_key *pwrkey = ddata->data;

        BUG_ON(irq != gpio_to_irq(pwrkey->gpio));
        schedule_work(&ddata->work);
        return IRQ_HANDLED;
}

static int __devinit power_key_probe(struct platform_device *pdev)
{
        struct power_key_platform_data *plat = pdev->dev.platform_data;
        struct power_key_drvdata *ddata;
        struct power_key *pwrkey;
        struct input_dev *input;
        int error;
        int wakeup = 0;
        int irq;

        if (plat == NULL) {
                dev_err(&pdev->dev, "no platform_data\n");
                return -EINVAL;
        }

        ppdev = pdev;
        ddata = kzalloc(sizeof(struct power_key_drvdata)
                        + sizeof(struct power_key), GFP_KERNEL);

        platform_set_drvdata(pdev, ddata);

        /* Input Interface */
        input = input_allocate_device();
        if (!ddata || !input) {
                error = -ENOMEM;
                goto fail1;
        }

        ddata->input = input;
        ddata->input->name = "power_key";

        /* Work Queue */
        INIT_WORK(&ddata->work, power_key_report_event);

        /* Gpio Interface */
        pwrkey = ddata->data;
        /* Customize power key according to acutal use case */
        pwrkey->gpio = (int)plat->pwrkey_pin;
        pwrkey->code = (int)KEY_POWER;
        pwrkey->desc = "pwrkey_irq";
        pwrkey->active_level = 0;
        pwrkey->type = EV_KEY;
        pwrkey->wakeup = 1;
        pwrkey->delay = 5000;

        /* Initialize GPIO */
        error = gpio_request(pwrkey->gpio, pwrkey->desc);
        if (error < 0) {
                pr_err("power_key: failed to request GPIO %d,"
                       " error %d\n", pwrkey->gpio, error);
                goto fail2;
        }
        error = gpio_direction_input(pwrkey->gpio);
        if (error < 0) {
                pr_err("power_key: failed to configure input"
                       " direction for GPIO %d, error %d\n",
                       pwrkey->gpio, error);
                gpio_free(pwrkey->gpio);
                goto fail2;
        }

        irq = gpio_to_irq(pwrkey->gpio);
        if (irq < 0) {
                error = irq;
                pr_err("power_key: Unable to get irq number"
                       " for GPIO %d, error %d\n",
                       pwrkey->gpio, error);
                gpio_free(pwrkey->gpio);
                goto fail2;
        }

        error = request_irq(irq, power_key_isr,
                            IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING|IRQF_SHARED,
                            pwrkey->desc,
                            ddata);
        if (error) {
                pr_err("power_key: Unable to claim irq %d; error %d\n",
                       irq, error);
                gpio_free(pwrkey->gpio);
                goto fail2;
        }

        if (pwrkey->wakeup)
                wakeup = 1;

        input_set_capability(input, pwrkey->type, pwrkey->code);

        error = input_register_device(input);
        if (error) {
                pr_err("gpio_key: Unable to register input device, "
                       "error: %d\n", error);
                goto fail2;
        }
        device_init_wakeup(&pdev->dev, wakeup);

		/*[ECID:000000] ZTEBSP zhangbo change for suspend, start*/
        enable_irq_wake(irq);
		/*[ECID:000000] ZTEBSP zhangbo change for suspend, end*/

        return 0;

fail2:
        free_irq(gpio_to_irq(ddata->data->gpio), &ddata->data);
        cancel_work_sync(&ddata->work);
        gpio_free(ddata->data->gpio);
        platform_set_drvdata(pdev, NULL);
fail1:
        input_free_device(input);
        kfree(ddata);
        return error;
}

static int __devexit power_key_remove(struct platform_device *pdev)
{
        struct power_key_drvdata *pdata = platform_get_drvdata(pdev);
        struct power_key *pwrkey = pdata->data;
        struct input_dev *input = pdata->input;
        int irq = gpio_to_irq(pwrkey->gpio);

        device_init_wakeup(&pdev->dev, 0);
        free_irq(irq, &pwrkey);
        cancel_work_sync(&pdata->work);
        gpio_free(pwrkey->gpio);
        platform_set_drvdata(pdev, NULL);
        input_unregister_device(input);
        kfree(pdata);
        return 0;
}

#ifdef CONFIG_PM
static int pwrkey_suspend(struct platform_device *pdev, pm_message_t state)
{
        /* Avoid shutdown */
        ForceNormalMode(&pdev->dev, true);
        return 0;
}

static int pwrkey_resume(struct platform_device *pdev)
{
        struct power_key_drvdata *pdata = platform_get_drvdata(pdev);
        unsigned long reg;

        if (pdata) {
                /* read PMC wake status register */
                reg = readl(pmc_base + PMC_WAKE_STATUS);
                if (reg & PMC_WAKE_KEY_MASK) {
                        input_report_key(pdata->input, KEY_POWER, 1);
                        input_sync(pdata->input);
                        input_report_key(pdata->input, KEY_POWER, 0);
                        input_sync(pdata->input);

                        /* clear PMC wake event for power on key. */
                        reg &= ~PMC_WAKE_KEY_MASK;
                        writel(reg, pmc_base + PMC_WAKE_STATUS);
                }
                return 0;
        }
        return -1;
}
#endif

static struct platform_driver power_key_driver = {
        .probe          = power_key_probe,
        .remove         = __devexit_p(power_key_remove),
#ifdef CONFIG_PM
        .suspend    = pwrkey_suspend,
        .resume     = pwrkey_resume,
#endif
        .driver = {
                .name = DRIVER_NAME,
                .owner  = THIS_MODULE,
        },
};

static int __init power_key_init(void)
{
        return platform_driver_register(&power_key_driver);
}

static void __exit power_key_exit(void)
{
        platform_driver_unregister(&power_key_driver);
}

module_init(power_key_init);
module_exit(power_key_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("power key driver");
MODULE_ALIAS("platform:power_key");
