/*
 * drivers/rtc/rtc-tps6586x.c
 *
 * RTC driver for TI TPS6586x
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/tps6586x.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>
/*[ECID:000000] ZTEBSP zhangbo 20120110, set 0xc1 register for reset, start*/
#include <linux/notifier.h>
#include <linux/reboot.h>
/*[ECID:000000] ZTEBSP zhangbo 20120110, set 0xc1 register for reset, end*/
/*[ECID:000000] ZTEBSP zhagnbo 20120302, enable alarm2, start*/
#include<mach/irqs.h>
/*[ECID:000000] ZTEBSP zhagnbo 20120302, enable alarm2, end*/


#define RTC_CTRL	0xc0
#define POR_RESET_N	BIT(7)
#define OSC_SRC_SEL	BIT(6)
#define RTC_ENABLE	BIT(5)	/* enables alarm */
#define RTC_BUF_ENABLE	BIT(4)	/* 32 KHz buffer enable */
#define PRE_BYPASS	BIT(3)	/* 0=1KHz or 1=32KHz updates */
#define CL_SEL_MASK	(BIT(2)|BIT(1))
#define CL_SEL_POS	1
#define RTC_ALARM1_HI	0xc1
#define RTC_COUNT4	0xc6
#define RTC_COUNT4_DUMMYREAD 0xc5  /* start a PMU RTC access by reading the register prior to the RTC_COUNT4 */
#define ALM1_VALID_RANGE_IN_SEC 0x3FFF /*only 14-bits width in second*/

/*[ECID:000000] ZTEBSP zhagnbo 20120302, enable alarm2, start*/
#define RTC_RSVDC00 	BIT(0)
#define RTC_ALARM2_HI    0xc4
/*[ECID:000000] ZTEBSP zhagnbo 20120302, enable alarm2, end*/

//[ECID:0000] ZTEBSP maxiaoping 20120425 for pmu alarm2 enable warning,match enable/disable irq&wakeup.start
static bool alarm2_irq_enable_once = false;
//[ECID:0000] ZTEBSP maxiaoping 20120425 for pmu alarm2 enable warning,match enable/disable irq&wakeup.end

//[ECID:0000] ZTEBSP maxiaoping 20120611 for alarm in power off charging,start
#define TPS6586x_RC4_RTC_ALARM2_HI          0xC4
#define TPS6586x_RC5_RTC_ALARM2_LO          0xC5

#define TPS6586x_RC6_RTC_COUNT4             0xC6
#define TPS6586x_RC7_RTC_COUNT3             0xC7
#define TPS6586x_RC8_RTC_COUNT2             0xC8
#define TPS6586x_RC9_RTC_COUNT1             0xC9
#define TPS6586x_RCA_RTC_COUNT0             0xCA
//[ECID:0000] ZTEBSP maxiaoping 20120611 for alarm in power off charging,end

struct tps6586x_rtc {
	unsigned long		epoch_start;
	int			irq;
	struct rtc_device	*rtc;
	bool			irq_en;
};

/*[ECID:000000] ZTEBSP zhangbo 20120110, set 0xc1 register for reset, start*/
#define BOOT_FLAG_SYS_RESET 0x57
static struct device *tps6586x_rtc_dev;
static int tegra_rtc_notify(struct notifier_block *this,
			    unsigned long code, void *dev)
{
	u8 buff[2]={BOOT_FLAG_SYS_RESET,0};
	if (code == SYS_RESTART )
	{
		tps6586x_writes(tps6586x_rtc_dev, RTC_ALARM1_HI, 1,buff);
	}
	else
	{
		tps6586x_writes(tps6586x_rtc_dev, RTC_ALARM1_HI, 1,buff+1);
	}
	return NOTIFY_DONE;
}
static int tegra_rtc_panic_notify(struct notifier_block *this, unsigned long event,
			void *ptr)
{
	u8 buff=BOOT_FLAG_SYS_RESET;

	tps6586x_writes(tps6586x_rtc_dev, RTC_ALARM1_HI, 1,&buff);
	return NOTIFY_DONE;
}
static struct notifier_block rtc_nb = {
	.notifier_call = tegra_rtc_notify,
};
static struct notifier_block rtc_panic_nb = {
	.notifier_call = tegra_rtc_panic_notify,
};
/*[ECID:000000] ZTEBSP zhangbo 20120110, set 0xc1 register for reset, end*/

//[ECID:0000] ZTEBSP maxiaoping 20120611 for alarm in power off charging,start
void zte_read_rtc_alarm2_lo(uint8_t* val)
{
	uint8_t   ReadAlarmCounter_LO;

	if(NULL == val)
	{
		printk(KERN_NOTICE "PM_DEBUG_MXP: zte_read_rtc_alarm2_lo: NULL == val.\r\n");
	}
	if(NULL == tps6586x_rtc_dev)
	{
		printk(KERN_NOTICE "PM_DEBUG_MXP: zte_read_rtc_alarm2_lo: NULL == tps6586x_rtc_dev.\r\n");
	}
	tps6586x_read(tps6586x_rtc_dev, TPS6586x_RC5_RTC_ALARM2_LO, &ReadAlarmCounter_LO);
	*val = ReadAlarmCounter_LO;
}

void zte_read_rtc_alarm2_hi(uint8_t* val)
{
	uint8_t   ReadAlarmCounter_HI;

	if(NULL == val)
	{
		printk(KERN_NOTICE "PM_DEBUG_MXP: zte_read_rtc_alarm2_hi: NULL == val.\r\n");
	}
	if(NULL == tps6586x_rtc_dev)
	{
		printk(KERN_NOTICE "PM_DEBUG_MXP: zte_read_rtc_alarm2_hi: NULL == tps6586x_rtc_dev.\r\n");
	}
	tps6586x_read(tps6586x_rtc_dev, TPS6586x_RC4_RTC_ALARM2_HI, &ReadAlarmCounter_HI);
	*val = ReadAlarmCounter_HI;
}

 int zte_tps6586x_rtc_read_time( unsigned long* tm)
{
	
	unsigned long long ticks = 0;
	unsigned long seconds;
	u8 buff[6];
	int err;
	int i;

	if(NULL == tm)
	{
		printk(KERN_NOTICE "PM_DEBUG_MXP: zte_tps6586x_rtc_read_time: NULL == tm.\r\n");
	}
	if(NULL == tps6586x_rtc_dev)
	{
		printk(KERN_NOTICE "PM_DEBUG_MXP: zte_tps6586x_rtc_read_time: NULL == tps6586x_rtc_dev.\r\n");
	}
	
	//here we read 6 regs form 0xc5 to 0xca to aviod read rtc reg errors.
	err = tps6586x_reads(tps6586x_rtc_dev, TPS6586x_RC5_RTC_ALARM2_LO, sizeof(buff), buff);
	if (err < 0) {
		printk(KERN_NOTICE "PM_DEBUG_MXP: zte_tps6586x_rtc_read_time: reads err = %d.\r\n",err);
		return err;
	}
	//get rid off reg 0xc5 'value(buff[0]) as it is useless,we need get the high 5 regs' value.(0xc6~0xca-->buff[1]~buff[5])
	for (i = 1; i < sizeof(buff); i++) {
		ticks <<= 8;
		ticks |= buff[i];
	}
	
	//remove 10 low bits data ,now we get the time of seconds.
	seconds = ticks >> 10;
	*tm = seconds;
	return err;
}
 //[ECID:0000] ZTEBSP maxiaoping 20120611 for alarm in power off charging,end
 
static inline struct device *to_tps6586x_dev(struct device *dev)
{
	return dev->parent;
}

static int tps6586x_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	unsigned long long ticks = 0;
	unsigned long seconds;
	u8 buff[6];
	int err;
	int i;

	err = tps6586x_reads(tps_dev, RTC_COUNT4_DUMMYREAD, sizeof(buff), buff);
	if (err < 0) {
		dev_err(dev, "failed to read counter\n");
		return err;
	}

	for (i = 1; i < sizeof(buff); i++) {
		ticks <<= 8;
		ticks |= buff[i];
	}

	seconds = ticks >> 10;

	seconds += rtc->epoch_start;
	rtc_time_to_tm(seconds, tm);
	return rtc_valid_tm(tm);
}

static int tps6586x_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	unsigned long long ticks;
	unsigned long seconds;
	u8 buff[5];
	int err;

	rtc_tm_to_time(tm, &seconds);

	if (WARN_ON(seconds < rtc->epoch_start)) {
		dev_err(dev, "requested time unsupported\n");
		return -EINVAL;
	}

	seconds -= rtc->epoch_start;

	ticks = (unsigned long long)seconds << 10;
	buff[0] = (ticks >> 32) & 0xff;
	buff[1] = (ticks >> 24) & 0xff;
	buff[2] = (ticks >> 16) & 0xff;
	buff[3] = (ticks >> 8) & 0xff;
	buff[4] = ticks & 0xff;

	err = tps6586x_clr_bits(tps_dev, RTC_CTRL, RTC_ENABLE);
	if (err < 0) {
		dev_err(dev, "failed to clear RTC_ENABLE\n");
		return err;
	}

	err = tps6586x_writes(tps_dev, RTC_COUNT4, sizeof(buff), buff);
	if (err < 0) {
		dev_err(dev, "failed to program new time\n");
		return err;
	}

	err = tps6586x_set_bits(tps_dev, RTC_CTRL, RTC_ENABLE);
	if (err < 0) {
		dev_err(dev, "failed to set RTC_ENABLE\n");
		return err;
	}

	return 0;
}

static int tps6586x_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	unsigned long seconds;
	unsigned long ticks;
	unsigned long rtc_current_time;
	unsigned long long rticks = 0;
	u8 buff[3];
	u8 rbuff[6];
	int err;
	int i;

	if (rtc->irq == -1)
		return -EIO;

	rtc_tm_to_time(&alrm->time, &seconds);

	if (WARN_ON(alrm->enabled && (seconds < rtc->epoch_start))) {
		dev_err(dev, "can't set alarm to requested time\n");
		return -EINVAL;
	}

	if (alrm->enabled && !rtc->irq_en) {
		enable_irq(rtc->irq);
		rtc->irq_en = true;
	} else if (!alrm->enabled && rtc->irq_en) {
		disable_irq(rtc->irq);
		rtc->irq_en = false;
	}

	seconds -= rtc->epoch_start;

	err = tps6586x_reads(tps_dev, RTC_COUNT4_DUMMYREAD, sizeof(rbuff), rbuff);
	if (err < 0) {
		dev_err(dev, "failed to read counter\n");
		return err;
	}

	for (i = 1; i < sizeof(rbuff); i++) {
		rticks <<= 8;
		rticks |= rbuff[i];
	}

	rtc_current_time = rticks >> 10;
	if ((seconds - rtc_current_time) > ALM1_VALID_RANGE_IN_SEC)
		seconds = rtc_current_time - 1;

	ticks = (unsigned long long)seconds << 10;

	buff[0] = (ticks >> 16) & 0xff;
	buff[1] = (ticks >> 8) & 0xff;
	buff[2] = ticks & 0xff;

	err = tps6586x_writes(tps_dev, RTC_ALARM1_HI, sizeof(buff), buff);
	if (err)
		dev_err(tps_dev, "unable to program alarm\n");

	return err;
}

static int tps6586x_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	unsigned long ticks;
	unsigned long seconds;
	u8 buff[3];
	int err;

	err = tps6586x_reads(tps_dev, RTC_ALARM1_HI, sizeof(buff), buff);
	if (err)
		return err;

	ticks = (buff[0] << 16) | (buff[1] << 8) | buff[2];
	seconds = ticks >> 10;
	seconds += rtc->epoch_start;

	rtc_time_to_tm(seconds, &alrm->time);

	return 0;
}

static int tps6586x_rtc_alarm_irq_enable(struct device *dev,
					 unsigned int enabled)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	u8 buff;
	int err;

	if (rtc->irq == -1)
		return -EIO;

	err = tps6586x_read(tps_dev, RTC_CTRL, &buff);
	if (err < 0) {
		dev_err(dev, "failed to read RTC_CTRL\n");
		return err;
	}

	if ((enabled && (buff & RTC_ENABLE)) ||
	    (!enabled && !(buff & RTC_ENABLE)))
		return 0;

	if (enabled) {
		err = tps6586x_set_bits(tps_dev, RTC_CTRL, RTC_ENABLE);
		if (err < 0) {
			dev_err(dev, "failed to set RTC_ENABLE\n");
			return err;
		}

		if (!rtc->irq_en) {
			enable_irq(rtc->irq);
			rtc->irq_en = true;
		}
	} else {
		err = tps6586x_clr_bits(tps_dev, RTC_CTRL, RTC_ENABLE);
		if (err < 0) {
			dev_err(dev, "failed to clear RTC_ENABLE\n");
			return err;
		}

		if (rtc->irq_en) {
			disable_irq(rtc->irq);
			rtc->irq_en = false;
		}
	}

	return 0;
}

static const struct rtc_class_ops tps6586x_rtc_ops = {
	.read_time	= tps6586x_rtc_read_time,
	.set_time	= tps6586x_rtc_set_time,
	.set_alarm	= tps6586x_rtc_set_alarm,
	.read_alarm	= tps6586x_rtc_read_alarm,
	.alarm_irq_enable = tps6586x_rtc_alarm_irq_enable,
};

static irqreturn_t tps6586x_rtc_irq(int irq, void *data)
{
	struct device *dev = data;
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);

	rtc_update_irq(rtc->rtc, 1, RTC_IRQF | RTC_AF);
	return IRQ_HANDLED;
}

/*[ECID:000000] ZTEBSP zhangbo 20120302, add power off alarm, start*/
int zte_set_poweroff_alarm_tps6586x(struct device *dev,unsigned long seconds)
{
      unsigned long now_time = get_seconds();
      int diff;  
      unsigned long ticks;
      struct tps6586x_rtc *rtc = dev_get_drvdata(dev);  
      struct device *tps_dev = to_tps6586x_dev(dev);
      struct rtc_time tm;
      u8 buff[2];
      //add for test
      u8 buff_1[2];
       unsigned long ticks_1;
       unsigned long seconds_1;
	unsigned long seconds_2;
      //add for test
      int err;
       diff = seconds-now_time;
      if(diff>0)
      	{      
	seconds -= rtc->epoch_start;
	ticks = (unsigned long long)seconds << 10;
       
       ticks >>= 12;
	buff[0] = (ticks >> 8) & 0xff;
	buff[1] = ticks & 0xff;
	seconds_2 = ticks&0xffff;
	err = tps6586x_writes(tps_dev,RTC_ALARM2_HI,sizeof(buff),buff);
	if (err)
		dev_err(tps_dev, "unable to program poweroff alarm\n");
	else {
//[ECID:0000] ZTEBSP maxiaoping 20120425 for pmu alarm2 enable warning,match enable/disable irq&wakeup.start		
			if(false == alarm2_irq_enable_once)
			{
				enable_irq_wake(TEGRA_NR_IRQS + TPS6586X_INT_RTC_ALM2);		
				enable_irq(TEGRA_NR_IRQS + TPS6586X_INT_RTC_ALM2);
				alarm2_irq_enable_once = true;
			}
//[ECID:0000] ZTEBSP maxiaoping 20120425 for pmu alarm2 enable warning,match enable/disable irq&wakeup.end			
	}

	return err;
      	}
	else
	{
		return 0;
	}
}
int zte_clear_poweroff_alarm_tps6586x(struct device *dev)
{
	struct device *tps_dev = to_tps6586x_dev(dev);
	u8 buff[2];
	buff[0]=0;
	buff[1]=0;
	tps6586x_writes(tps_dev,RTC_ALARM2_HI,sizeof(buff),buff);
	disable_irq(TEGRA_NR_IRQS + TPS6586X_INT_RTC_ALM2);
//[ECID:0000] ZTEBSP maxiaoping 20120425 for pmu alarm2 enable warning,match enable/disable irq&wakeup.start
	alarm2_irq_enable_once = false;
//[ECID:0000] ZTEBSP maxiaoping 20120425 for pmu alarm2 enable warning,match enable/disable irq&wakeup.end
	return 0;
}
/*[ECID:000000] ZTEBSP zhangbo 20120302, add power off alarm, end*/


static int __devinit tps6586x_rtc_probe(struct platform_device *pdev)
{
	struct tps6586x_rtc_platform_data *pdata = pdev->dev.platform_data;
	struct device *tps_dev = to_tps6586x_dev(&pdev->dev);
	struct tps6586x_rtc *rtc;
	int err;
	struct tps6586x_epoch_start *epoch;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform_data specified\n");
		return -EINVAL;
	}

	rtc = kzalloc(sizeof(*rtc), GFP_KERNEL);

	if (!rtc)
		return -ENOMEM;

	rtc->irq = -1;

	if (pdata->irq < 0)
		dev_warn(&pdev->dev, "no IRQ specified, wakeup is disabled\n");

	epoch = &pdata->start;
	rtc->epoch_start = mktime(epoch->year, epoch->month, epoch->day,
				  epoch->hour, epoch->min, epoch->sec);

	dev_set_drvdata(&pdev->dev, rtc);

	device_init_wakeup(&pdev->dev, 1);

	rtc->rtc = rtc_device_register("tps6586x-rtc", &pdev->dev,
				       &tps6586x_rtc_ops, THIS_MODULE);

	if (IS_ERR(rtc->rtc)) {
		err = PTR_ERR(rtc->rtc);
		goto fail;
	}

	/*[ECID:000000] ZTEBSP zhagnbo 20120302, enable alarm2, start*/
	/* 1 kHz tick mode, enable tick counting */
	err = tps6586x_update(tps_dev, RTC_CTRL,
		RTC_ENABLE | OSC_SRC_SEL | ((pdata->cl_sel << CL_SEL_POS) &
					    CL_SEL_MASK) |RTC_RSVDC00,
		RTC_ENABLE | OSC_SRC_SEL | PRE_BYPASS | CL_SEL_MASK |RTC_RSVDC00);
	/*[ECID:000000] ZTEBSP zhagnbo 20120302, enable alarm2, end*/
	if (err < 0) {
		dev_err(&pdev->dev, "unable to start counter\n");
		goto fail;
	}

	if (pdata && (pdata->irq >= 0)) {
		rtc->irq = pdata->irq;
		err = request_threaded_irq(pdata->irq, NULL, tps6586x_rtc_irq,
					   IRQF_ONESHOT, "tps6586x-rtc",
					   &pdev->dev);
		if (err) {
			dev_warn(&pdev->dev, "unable to request IRQ(%d)\n", rtc->irq);
			rtc->irq = -1;
		} else {
			enable_irq_wake(rtc->irq);
			disable_irq(rtc->irq);
			rtc->irq_en = false; //[ECID:000000] ZTEBSP zhangbo 20120302, remember irq status
		}
	}

	/*[ECID:000000] ZTEBSP zhangbo 20120110, set 0xc1 register for reset, start*/
	tps6586x_rtc_dev=tps_dev;
	register_reboot_notifier(&rtc_nb);
	atomic_notifier_chain_register(&panic_notifier_list, &rtc_panic_nb);
	/*[ECID:000000] ZTEBSP zhangbo 20120110, set 0xc1 register for reset, end*/

	return 0;

fail:
	if (!IS_ERR_OR_NULL(rtc->rtc))
		rtc_device_unregister(rtc->rtc);
	device_init_wakeup(&pdev->dev, 0);
	kfree(rtc);
	return err;
}

static int __devexit tps6586x_rtc_remove(struct platform_device *pdev)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(&pdev->dev);

	 /*[ECID:000000] ZTEBSP zhangbo 20120110, set 0xc1 register for reset, start*/
	unregister_reboot_notifier(&rtc_nb);
	atomic_notifier_chain_unregister(&panic_notifier_list, &rtc_panic_nb);
	/*[ECID:000000] ZTEBSP zhangbo 20120110, set 0xc1 register for reset, end*/

	if (rtc->irq != -1)
		free_irq(rtc->irq, rtc);
	rtc_device_unregister(rtc->rtc);
	kfree(rtc);
	return 0;
}

static struct platform_driver tps6586x_rtc_driver = {
	.driver	= {
		.name	= "tps6586x-rtc",
		.owner	= THIS_MODULE,
	},
	.probe	= tps6586x_rtc_probe,
	.remove	= __devexit_p(tps6586x_rtc_remove),
};

static int __init tps6586x_rtc_init(void)
{
	return platform_driver_register(&tps6586x_rtc_driver);
}
module_init(tps6586x_rtc_init);

static void __exit tps6586x_rtc_exit(void)
{
	platform_driver_unregister(&tps6586x_rtc_driver);
}
module_exit(tps6586x_rtc_exit);

MODULE_DESCRIPTION("TI TPS6586x RTC driver");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtc-tps6586x");
