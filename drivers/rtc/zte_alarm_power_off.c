/*
 *  kernel/drivers/rtc/zte_alarm_power_off.c
 *
 *  Copyright (C) 2012 ZTE
 *
 * Maxiaoping 20111223 Modified for P943T series alarm function during power off charging.
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/nmi.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>			/* For in_interrupt() */
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/security.h>
#include <linux/bootmem.h>
#include <linux/syscalls.h>
#include <linux/kexec.h>
#include <linux/kdb.h>
#include <linux/ratelimit.h>
#include <linux/kmsg_dump.h>
#include <linux/syslog.h>
#include <linux/cpu.h>
#include <linux/notifier.h>
#include <asm/uaccess.h>
//[ECID:0000] ZTEBSP maxiaoping 20120611 for alarm in power off charging,start
#include <linux/mfd/tps6586x.h>


extern void zte_read_rtc_alarm2_lo(uint8_t* val);
extern void zte_read_rtc_alarm2_hi(uint8_t* val);
 extern int zte_tps6586x_rtc_read_time( unsigned long* tm);
 
typedef struct 
{
	unsigned long alarm_triggered;	
 	char value[5];
} zte_power_off_alarm_type;

static zte_power_off_alarm_type zte_power_off_alarm_triggered_value[]=
{
	//{alarm_triggered,return_value}
    	{0,"FALSE"},
	{1,"TRUE"}	
};

static struct kobject *zte_power_off_alarm_kobj;

int zte_get_power_off_alarm_status(void)
{
	
        int rtl = 0;
        unsigned long ReadRtcCounter;
        uint8_t   ReadAlarmCounter_LO,ReadAlarmCounter_HI;
        unsigned long alarm_seconds;
        unsigned long RtcTimeSeconds;
        unsigned long temp;
		
	//printk(KERN_NOTICE "PM_DEBUG_MXP:Enter zte_get_power_off_alarm_status.\r\n");
	
          //Avoiding read RTC time sequence error,we use the system function here. 
          zte_tps6586x_rtc_read_time(&ReadRtcCounter);
	 //printk(KERN_NOTICE "PM_DEBUG_MXP: ReadRtcCounter =0x%lx.\r\n",ReadRtcCounter);

	 // The unit of the RTC count is second!!! 1024 tick = 1s.
         // Read all 40 bit and right move 10 = Read the hightest 32bit and right move 2
	 //ReadRtcBuffer contains the rtc data from rtc[10]-rtc[39],here we need only rtc[12]-rtc[27].
	 
	 /*
	 As Alarm2 has only 16bits data,and RTC second data starts from bit10~bit27,there is two extra 
	 bits data, so we need to get rid of these two bits to use 16 bits(bit12~bit27) in keeping with Rtc data.
	 but this cause 3 seconds time error.
	 */
	 
	 RtcTimeSeconds = (ReadRtcCounter >> 2) & 0xFFFF;
	 //printk(KERN_NOTICE "PM_DEBUG_MXP:RtcTimeSeconds=0x%lx.\r\n",RtcTimeSeconds);
	 //printk(KERN_NOTICE "PM_DEBUG_MXP:RtcTimeSeconds=%ld.\r\n",RtcTimeSeconds);

	 //first get low 8 bits data RTC_ALARM2_LO
	zte_read_rtc_alarm2_lo(&ReadAlarmCounter_LO);
	//then high 8 bits RTC_ALARM2_HI
	zte_read_rtc_alarm2_hi(&ReadAlarmCounter_HI);
	 // return second
         alarm_seconds = ((ReadAlarmCounter_HI<<8)|ReadAlarmCounter_LO)&0xFFFF;
	//printk(KERN_NOTICE "PM_DEBUG_MXP:alarm2_value=0x%lx.\r\n",alarm_seconds);
	//printk(KERN_NOTICE "PM_DEBUG_MXP:alarm2_value=%ld.\r\n",alarm_seconds);
	
	 if (alarm_seconds == 0)
	 {
	 	printk(KERN_NOTICE " PM_DEBUG_MXP: ALARM counter Null.\r\n");
	 	return 0;
	 }
	 //temp=(alarm_seconds -RtcTimeSeconds);
	 //here we can't use abs to calc them,as unsign counter has no minus value.
	 if (alarm_seconds > RtcTimeSeconds)//exchage the value.
	 {
	 	temp = alarm_seconds;
	 	alarm_seconds = RtcTimeSeconds;
	 	RtcTimeSeconds = temp;
	 }
	 //As RTC has 3 seconds' error and the startup may takes some time,here we give them 5 seconds redundancy.
	  if ((RtcTimeSeconds -alarm_seconds)<=5)
	  {
	 	printk(KERN_NOTICE " PM_DEBUG_MXP:RTC ALARM trigger detect.\r\n");
	 	rtl = 1;
	  }
	  else
	  {
		//printk(KERN_NOTICE " PM_DEBUG_MXP:NO RTC ALARM trigger detect.\r\n");
		rtl = 0;
	  }
	return rtl;
}

static ssize_t zte_power_off_alarm_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{	
	int power_off_alarm_triggered;
	power_off_alarm_triggered = zte_get_power_off_alarm_status();
	int return_value = 0;
	
	switch(power_off_alarm_triggered)
	{
		case 0:
		return_value = sprintf(buf, "%s\n", zte_power_off_alarm_triggered_value[0].value);
		break;

		case 1:
		return_value = sprintf(buf, "%s\n", zte_power_off_alarm_triggered_value[1].value);
		break;

		default:
		return_value = sprintf(buf, "%s\n", zte_power_off_alarm_triggered_value[0].value);
		break;	
	}
         return return_value;
}

static ssize_t zte_power_off_alarm_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t n)
{
		return n;
}

static struct kobj_attribute zte_power_off_alarm_attribute =
	__ATTR(power_off_alarm, 0755, zte_power_off_alarm_show, zte_power_off_alarm_store);


static struct attribute *attrs[] = {
	&zte_power_off_alarm_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static int __init zte_power_off_alarm_init(void)
{
	int retval;

	/*
	 * Create  /sys/zte_power_off_alarm/
	 */
	 
	zte_power_off_alarm_kobj = kobject_create_and_add("zte_power_off_alarm", NULL);
	if (!zte_power_off_alarm_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(zte_power_off_alarm_kobj, &attr_group);
	if (retval)
		kobject_put(zte_power_off_alarm_kobj);

	return retval;	
	
}

static void __exit zte_power_off_alarm_exit(void)
{	
	kobject_put(zte_power_off_alarm_kobj);
}

module_init(zte_power_off_alarm_init);
module_exit(zte_power_off_alarm_exit);

//[ECID:0000] ZTEBSP maxiaoping 20120611 for alarm in power off charging,start