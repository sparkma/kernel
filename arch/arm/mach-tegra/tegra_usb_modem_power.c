/*
 * arch/arm/mach-tegra/tegra_usb_modem_power.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

//zte_modify nv patch begin
#include <linux/version.h>
//zte_modify nv patch end
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/usb.h>
#include <linux/err.h>
//ZTEBSP DangXiao 20120702,nv patch,start
#include <linux/pm_runtime.h>
#include <linux/suspend.h>
//ZTEBSP DangXiao 20120702,nv patch,end
#include <linux/pm_qos_params.h>
#include <linux/wakelock.h>
#include <mach/tegra_usb_modem_power.h>

#define BOOST_CPU_FREQ_MIN	1000000
#define BOOST_CPU_FREQ_TIMEOUT	5000

struct tegra_usb_modem {
	unsigned int wake_gpio;	/* remote wakeup gpio */
	unsigned int wake_cnt;	/* remote wakeup counter */
	unsigned int wake_irq;	/* remote wakeup irq */
	unsigned int boot_gpio;	/* modem boot gpio */
	unsigned int boot_irq;	/* modem boot irq */
	struct mutex lock;
	struct wake_lock wake_lock;	/* modem wake lock */
	unsigned int vid;	/* modem vendor id */
	unsigned int pid;	/* modem product id */
	struct usb_device *udev;	/* modem usb device */
	struct usb_interface *intf;	/* first modem usb interface */
	struct workqueue_struct *wq;	/* modem workqueue */
	struct delayed_work recovery_work;	/* modem recovery work */
	struct pm_qos_request_list boost_cpu_freq_req;
	struct delayed_work pm_qos_work;
	const struct tegra_modem_operations *ops;	/* modem operations */
	unsigned int capability;	/* modem capability */
	//ZTEBSP DangXiao 20120702,nv patch,start
	int system_suspend;	/* system suspend flag */
	//ZTEBSP DangXiao 20120702,nv patch,end
};

static struct tegra_usb_modem * tegra_mdm = NULL;

/* supported modems */
static const struct usb_device_id modem_list[] = {
	{USB_DEVICE(0x1983, 0x0310),	/* Icera 450 rev1 */
	 .driver_info = TEGRA_MODEM_AUTOSUSPEND,
	 },
	{USB_DEVICE(0x1983, 0x0321),	/* Icera 450 rev2 */
	 .driver_info = TEGRA_MODEM_AUTOSUSPEND,
	 },
	{}
};

extern int usb_remove_device(struct usb_device *udev);
extern int usb_set_configuration(struct usb_device *udev, int configuration);

extern void tegra_start_usb_host();
extern void tegra_stop_usb_host();

static irqreturn_t tegra_usb_modem_wake_thread(int irq, void *data)
{
	struct tegra_usb_modem *modem = (struct tegra_usb_modem *)data;

	wake_lock_timeout(&modem->wake_lock, HZ);
	mutex_lock(&modem->lock);
	
	//ZTEBSP DangXiao 20120702,nv patch,start
	if (modem->udev && modem->udev->state != USB_STATE_NOTATTACHED) {
		if (!modem->system_suspend) {
			pr_info("modem wake (%u)\n", ++(modem->wake_cnt));
			usb_lock_device(modem->udev);
			if (usb_autopm_get_interface(modem->intf) == 0)
				usb_autopm_put_interface_async(modem->intf);
			usb_unlock_device(modem->udev);
			} else {
				pr_info("under system suspend process, avoid modem wake\n");
			}
	}
	//ZTEBSP DangXiao 20120702,nv patch,end
	mutex_unlock(&modem->lock);

	return IRQ_HANDLED;
}

static void pm_qos_worker(struct work_struct *work)
{
	pr_debug("%s - pm qos CPU back to normal\n", __func__);
	pm_qos_update_request(&tegra_mdm->boost_cpu_freq_req,
			(s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
}

//zte_modify nv patch begin
static irqreturn_t tegra_usb_modem_boot_thread(int irq, void *data)
{
	struct tegra_usb_modem *modem = (struct tegra_usb_modem *)data;

	if (gpio_get_value(modem->boot_gpio)) {
		pr_info("BB_RST_OUT high\n");
		pr_debug("%s: pm qos request CPU 1.0GHz\n", __func__);
		pm_qos_update_request(&modem->boost_cpu_freq_req, (s32)BOOST_CPU_FREQ_MIN);
		schedule_delayed_work(&modem->pm_qos_work, msecs_to_jiffies(BOOST_CPU_FREQ_TIMEOUT));
	} else {
		pr_info("BB_RST_OUT low\n");
		//if (modem->udev) {
		//	pr_info("Force remove modem usb device\n");
		//	usb_lock_device(modem->udev);
		//	usb_set_configuration(modem->udev, -1);
		//	usb_remove_device(modem->udev);
		//	usb_unlock_device(modem->udev);
		//}
	}

	/* hold wait lock to complete the enumeration */
	wake_lock_timeout(&modem->wake_lock, HZ * 10);

//#ifdef CONFIG_PM
	//mutex_lock(&modem->lock);
	//if (modem->parent)
	//	usb_disable_autosuspend(modem->parent);
	//mutex_unlock(&modem->lock);
//#endif

	return IRQ_HANDLED;
}
//zte_modify nv patch end

static void tegra_usb_modem_recovery(struct work_struct *ws)
{
	struct tegra_usb_modem *modem = container_of(ws, struct tegra_usb_modem,
						     recovery_work.work);

	mutex_lock(&modem->lock);
	if (!modem->udev) {	/* assume modem crashed */
		if (modem->ops && modem->ops->reset)
			modem->ops->reset();
	}
	mutex_unlock(&modem->lock);
}

static void device_add_handler(struct usb_device *udev)
{
	const struct usb_device_descriptor *desc = &udev->descriptor;
	struct usb_interface *intf = usb_ifnum_to_if(udev, 0);
	const struct usb_device_id *id = NULL;

	if (desc == NULL || intf == NULL) {
		pr_err("Get usb descriptor or interface failed, remove device!\n");
		usb_remove_device(udev);
		return;
	}

	id = usb_match_id(intf, modem_list);
	if (id) {
//zte_modify nv patch begin
		/* hold wakelock to ensure ril has enough time to restart */
		wake_lock_timeout(&tegra_mdm->wake_lock, HZ*10);
//zte_modify nv patch end

		pr_info("Add device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);

		mutex_lock(&tegra_mdm->lock);
		tegra_mdm->udev = udev;
//zte_modify nv patch begin
		//tegra_mdm.parent = udev->parent;
//zte_modify nv patch end
		tegra_mdm->intf = intf;
		tegra_mdm->vid = desc->idVendor;
		tegra_mdm->pid = desc->idProduct;
		tegra_mdm->wake_cnt = 0;
		tegra_mdm->capability = id->driver_info;
		mutex_unlock(&tegra_mdm->lock);

		pr_info("persist_enabled: %u\n", udev->persist_enabled);

//zte_modify nv patch begin
#ifdef CONFIG_PM
		//if (tegra_mdm.parent)
		//	usb_enable_autosuspend(tegra_mdm.parent);

		if (tegra_mdm->capability & TEGRA_MODEM_AUTOSUSPEND) {
//zte_modify nv patch begin
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39))
			pm_runtime_set_autosuspend_delay(&udev->dev, 2000);
#else
			udev->autosuspend_delay = 2000;
#endif
//zte_modify nv patch end
			usb_enable_autosuspend(udev);
			pr_info("enable autosuspend for %s %s\n",
				udev->manufacturer, udev->product);
		}
#endif
//zte_modify nv patch end
	}
}

static void device_remove_handler(struct usb_device *udev)
{
	const struct usb_device_descriptor *desc = &udev->descriptor;

	if (desc->idVendor == tegra_mdm->vid &&
	    desc->idProduct == tegra_mdm->pid) {
		pr_info("Remove device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);

		mutex_lock(&tegra_mdm->lock);
		tegra_mdm->udev = NULL;
		tegra_mdm->intf = NULL;
		tegra_mdm->vid = 0;
		mutex_unlock(&tegra_mdm->lock);

//zte_modify nv patch begin
//#ifdef CONFIG_PM
		//if (udev->parent)
		//	usb_disable_autosuspend(udev->parent);
//#endif
//zte_modify nv patch end

		if (tegra_mdm->capability & TEGRA_MODEM_RECOVERY)
			queue_delayed_work(tegra_mdm->wq,
					   &tegra_mdm->recovery_work, HZ * 10);
	}
}

static int usb_notify(struct notifier_block *self, unsigned long action,
		      void *blob)
{
	switch (action) {
	case USB_DEVICE_ADD:
		device_add_handler(blob);
		break;
	case USB_DEVICE_REMOVE:
		device_remove_handler(blob);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block usb_nb = {
	.notifier_call = usb_notify,
};
//ZTEBSP DangXiao 20120702,nv patch,start
static int mdm_pm_notifier(struct notifier_block *notifier, unsigned long pm_event,
			void *blob)
{
	mutex_lock(&tegra_mdm->lock);

	if (!tegra_mdm->udev) {
		mutex_unlock(&tegra_mdm->lock);
		return NOTIFY_DONE;
	}

	pr_info("%s event %ld\n", __func__, pm_event);
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
//		if (wake_lock_active(&tegra_mdm->wake_lock)) {
//			pr_warn("%s wakelock was active, aborting suspend\n");
//			mutext_unlock(&tegra_mdm->lock);
//			return NOTIFY_STOP;
//		}
		tegra_mdm->system_suspend = 1;
		mutex_unlock(&tegra_mdm->lock);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		tegra_mdm->system_suspend = 0;
		mutex_unlock(&tegra_mdm->lock);
		return NOTIFY_OK;
	}

	mutex_unlock(&tegra_mdm->lock);
}

static struct notifier_block pm_nb = {
	.notifier_call = mdm_pm_notifier,
};
//ZTEBSP DangXiao 20120702,nv patch,end

//zte_modify nv patch begin
static int mdm_request_wakeable_irq(struct tegra_usb_modem *modem,
				irq_handler_t thread_fn,
				unsigned int irq_gpio,
				unsigned long irq_flags,
				const char* label,
				unsigned int *irq)
{
	int ret;

	ret = gpio_request(irq_gpio, label);
	if (ret)
		return ret;

	tegra_gpio_enable(irq_gpio);

	/* enable IRQ for GPIO */
	*irq = gpio_to_irq(irq_gpio);

	/* request threaded irq for GPIO */
	ret = request_threaded_irq(*irq, NULL, thread_fn, irq_flags, label,
				   modem);
	if (ret)
		return ret;

	ret = enable_irq_wake(*irq);
	if (ret) {
		free_irq(*irq, modem);
		return ret;
	}

	return ret;
}
//zte_modify nv patch end

static ssize_t load_unload_usb_host(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int host;
	int err = 0;

	err = sscanf(buf, "%d", &host);
	if (err != 1) {
		return count;
	}

	if (host == 1) {
		tegra_start_usb_host();
	} else if (host == 0) {
		tegra_stop_usb_host();
	}

	return count;
}

static DEVICE_ATTR(load_host, 0644, NULL, load_unload_usb_host);

static int tegra_usb_modem_probe(struct platform_device *pdev)
{
	struct tegra_usb_modem_power_platform_data *pdata =
	    pdev->dev.platform_data;
	int ret;

	if (!pdata) {
		dev_dbg(&pdev->dev, "platform_data not available\n");
		return -EINVAL;
	}
	
	tegra_mdm = kzalloc(sizeof(struct tegra_usb_modem), GFP_KERNEL);
	if (!tegra_mdm) {
		dev_dbg(&pdev->dev, "allocate memory failed!\n");
		return ENOMEM;
	}
	/* get modem operations from platform data */
	tegra_mdm->ops = (const struct tegra_modem_operations *)pdata->ops;

	if (tegra_mdm->ops) {
		/* modem init */
		if (tegra_mdm->ops->init) {
			ret = tegra_mdm->ops->init();
			if (ret)
				return ret;
		}

		/* start modem */
		if (tegra_mdm->ops->start)
			tegra_mdm->ops->start();
	}

	mutex_init(&(tegra_mdm->lock));
	wake_lock_init(&(tegra_mdm->wake_lock), WAKE_LOCK_SUSPEND,
		       "tegra_usb_mdm_lock");

	/* create work queue */
	tegra_mdm->wq = create_workqueue("tegra_usb_mdm_queue");
	INIT_DELAYED_WORK(&(tegra_mdm->recovery_work), tegra_usb_modem_recovery);

	INIT_DELAYED_WORK(&tegra_mdm->pm_qos_work, pm_qos_worker);
	pm_qos_add_request(&tegra_mdm->boost_cpu_freq_req, PM_QOS_CPU_FREQ_MIN,
			(s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);

//zte_modify nv patch begin
	/* get remote wakeup gpio from platform data */
	tegra_mdm->wake_gpio = pdata->wake_gpio;

	ret = mdm_request_wakeable_irq(tegra_mdm,
					tegra_usb_modem_wake_thread,
					tegra_mdm->wake_gpio,
					pdata->wake_irq_flags,
					"mdm_wake",
					&tegra_mdm->wake_irq);
	if (ret) {
		dev_err(&pdev->dev, "%s: request wake irq error\n",
			__func__);
		return ret;
	}

	/* get modem boot gpio from platform data */
	tegra_mdm->boot_gpio = pdata->boot_gpio;

	ret = mdm_request_wakeable_irq(tegra_mdm,
					tegra_usb_modem_boot_thread,
					tegra_mdm->boot_gpio,
					pdata->boot_irq_flags,
					"mdm_boot",
					&tegra_mdm->boot_irq);
	if (ret) {
		dev_err(&pdev->dev, "%s: request boot irq error\n",
			__func__);
		disable_irq_wake(tegra_mdm->wake_irq);
		free_irq(tegra_mdm->wake_irq, tegra_mdm);
		return ret;
//zte_modify nv patch end
	}

	usb_register_notify(&usb_nb);
	//ZTEBSP DangXiao 20120702,nv patch,start
	register_pm_notifier(&pm_nb);
	//ZTEBSP DangXiao 20120702,nv patch,end
	ret = device_create_file(&pdev->dev, &dev_attr_load_host);
	if (ret) {
		dev_warn(&pdev->dev, "Can't register sysfs attribute for load unload USB host\n");
	}

	dev_info(&pdev->dev, "Initialized tegra_usb_modem_power\n");

	return 0;
}

static int __exit tegra_usb_modem_remove(struct platform_device *pdev)
{
	//ZTEBSP DangXiao 20120702,nv patch,start
	unregister_pm_notifier(&pm_nb);
	//ZTEBSP DangXiao 20120702,nv patch,end
	usb_unregister_notify(&usb_nb);

//zte_modify nv patch begin
	if (tegra_mdm->wake_irq) {
		disable_irq_wake(tegra_mdm->wake_irq);
		free_irq(tegra_mdm->wake_irq, tegra_mdm);
	}

	if (tegra_mdm->boot_irq) {
		disable_irq_wake(tegra_mdm->boot_irq);
		free_irq(tegra_mdm->boot_irq, tegra_mdm);
	}
//zte_modify nv patch end

	if (tegra_mdm) {
		kfree(tegra_mdm);
		tegra_mdm = NULL;
	}

	return 0;
}

#ifdef CONFIG_PM
static int tegra_usb_modem_suspend(struct platform_device *pdev,
				   pm_message_t state)
{
	/* send L3 hint to modem */
	if (tegra_mdm->ops && tegra_mdm->ops->suspend)
		tegra_mdm->ops->suspend();
	return 0;
}

static int tegra_usb_modem_resume(struct platform_device *pdev)
{
	/* send L3->L0 hint to modem */
	if (tegra_mdm->ops && tegra_mdm->ops->resume)
		tegra_mdm->ops->resume();
	return 0;
}
#endif

static struct platform_driver tegra_usb_modem_power_driver = {
	.driver = {
		   .name = "tegra_usb_modem_power",
		   .owner = THIS_MODULE,
		   },
	.probe = tegra_usb_modem_probe,
	.remove = __exit_p(tegra_usb_modem_remove),
#ifdef CONFIG_PM
	.suspend = tegra_usb_modem_suspend,
	.resume = tegra_usb_modem_resume,
#endif
};

static int __init tegra_usb_modem_power_init(void)
{
	return platform_driver_register(&tegra_usb_modem_power_driver);
}

subsys_initcall(tegra_usb_modem_power_init);

static void __exit tegra_usb_modem_power_exit(void)
{
	platform_driver_unregister(&tegra_usb_modem_power_driver);
	pm_qos_remove_request(&tegra_mdm->boost_cpu_freq_req);
}

module_exit(tegra_usb_modem_power_exit);

MODULE_DESCRIPTION("Tegra usb modem power management driver");
MODULE_LICENSE("GPL");
