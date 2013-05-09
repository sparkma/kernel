/*
 *  kernel/drivers/misc/zte_board.c
 *
 *  Copyright (C) 2011 ZTE
 *
 * Maxiaoping 20111223 Modified for zte hardware board ID control.
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
//[ECID:0000] ZTEBSP maxiaoping 20111223  for NV platform hardware board it acquisition,start.
//[ECID:0000] ZTEBSP maxiaoping 20120207  for NV platform hardware board it acquisition,start.
#include <linux/gpio.h>

//add gpio input detction.
#define HARDWARE_VERSION1  56 //TEGRA_GPIO_PH0
#define HARDWARE_VERSION2  57 //TEGRA_GPIO_PH1
#define HARDWARE_VERSION3  58 //TEGRA_GPIO_PH2
//[ECID:0000] ZTEBSP maxiaoping 20120207  for NV platform hardware board it acquisition,end.

typedef struct 
{
    unsigned long voltage_mv;
    char id[5];
} zte_hardware_version_type;

//按照电压从小到大排列硬件版本号，版本号最长4个字符
static zte_hardware_version_type zte_hardware_version[]=
{
	//{电压(mV)，版本号}
    	{900,"wdjA"},
	{900,"wdjB"},
	{900,"wdjC"}
};
//[ECID:0000] ZTEBSP maxiaoping 20120207  for NV platform hardware board it acquisition,end.

static struct kobject *zte_board_id_kobj;

//[ECID:0000] ZTEBSP maxiaoping 20120315  for log errors,start.
static int zte_hw_board_initialize = 0;

static void zte_hw_board_id_gpio_init(void)
{
	//enable 3 detect gpios
	tegra_gpio_enable(HARDWARE_VERSION1);
	tegra_gpio_enable(HARDWARE_VERSION2);
	tegra_gpio_enable(HARDWARE_VERSION3);
	//request gpios
	gpio_request(HARDWARE_VERSION1, "hw_board_id1");
	gpio_request(HARDWARE_VERSION2, "hw_board_id2");
	gpio_request(HARDWARE_VERSION3, "hw_board_id3");
	//change the var,init only once.
	zte_hw_board_initialize = 1;
}

//[ECID:0000] ZTEBSP maxiaoping 20120207  for NV platform hardware board it acquisition,start.
int zte_get_hw_board_id(void)
{
	int result=0;
	int rtl = 0;
	
	//printk(KERN_NOTICE "PM_DEBUG_MXP:Enter zte_get_hw_board_id.\r\n");
	
	if(zte_hw_board_initialize == 0)
	{
		zte_hw_board_id_gpio_init();
	}
	//set gpios as input
	gpio_direction_input(HARDWARE_VERSION1);
	gpio_direction_input(HARDWARE_VERSION2);
	gpio_direction_input(HARDWARE_VERSION3);
	//get gpios input value
	result = gpio_get_value(HARDWARE_VERSION1);
	//printk(KERN_NOTICE "PM_DEBUG_MXP:read HARDWARE_VERSION1 value =%d.\r\n",result);
	rtl =result;
	result = gpio_get_value(HARDWARE_VERSION2);
	//printk(KERN_NOTICE "PM_DEBUG_MXP:read HARDWARE_VERSION2 value =%d.\r\n",result);
	rtl |=(result<<1);
	result = gpio_get_value(HARDWARE_VERSION3);
	//printk(KERN_NOTICE "PM_DEBUG_MXP:read HARDWARE_VERSION3 value =%d.\r\n",result);
	rtl |=(result<<2);

	//printk(KERN_NOTICE "PM_DEBUG_MXP:read finally board_id value =%d.\r\n",rtl);
	return rtl;
}
//[ECID:0000] ZTEBSP maxiaoping 20120315  for log errors,end.
//[ECID:0000] ZTEBSP maxiaoping 20120220  modify this function,start.
static ssize_t zte_board_id_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{	
	int hw_board_id;
	hw_board_id = zte_get_hw_board_id();
	int return_value = 0;
	switch(hw_board_id)
	{
		case 0:
		return_value = sprintf(buf, "%s\n", zte_hardware_version[0].id);
		break;

		case 1:
		case 5:
		return_value = sprintf(buf, "%s\n", zte_hardware_version[1].id);
		break;

		case 2:
		case 6:
		return_value = sprintf(buf, "%s\n", zte_hardware_version[2].id);
		break;

		default:
		return_value = sprintf(buf, "%s\n", zte_hardware_version[0].id);
		break;	
	}
         return return_value;
}
//[ECID:0000] ZTEBSP maxiaoping 20120220  modify this function,end.
//[ECID:0000] ZTEBSP maxiaoping 20120207  for NV platform hardware board it acquisition,end.

static ssize_t zte_board_id_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t n)
{

         // this is a test code.
         #if 0
	if(n !=4)
	{
		return -EINVAL;
	}
	if(strcpy( zte_hardware_version[0].id,buf)!=&(zte_hardware_version[0].id))
	//if(!strict_strtoul(buf+2,16,&print_control_flag))
	{
		return -EINVAL;
	}
	#endif
		return n;
}

static struct kobj_attribute zte_board_id_attribute =
	__ATTR(board_id, 0755, zte_board_id_show, zte_board_id_store);

static struct attribute *attrs[] = {
	&zte_board_id_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static int __init zte_board_id_init(void)
{
	int retval;

	//[ECID:0000] ZTEBSP maxiaoping 20120315  for log errors,start.
	if(zte_hw_board_initialize == 0)
	{zte_hw_board_id_gpio_init();}
	//[ECID:0000] ZTEBSP maxiaoping 20120315  for log errors,end.
	
	/*
	 * Create  /sys/zte_board_id/
	 */
	 
	zte_board_id_kobj = kobject_create_and_add("zte_board_id", NULL);
	if (!zte_board_id_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(zte_board_id_kobj, &attr_group);
	if (retval)
		kobject_put(zte_board_id_kobj);

	return retval;	
	
}

//[ECID:0000] ZTEBSP maxiaoping 20120207  for NV platform hardware board it acquisition,start.
static void __exit zte_board_id_exit(void)
{	
	//free gpios
	gpio_free(HARDWARE_VERSION1);
	gpio_free(HARDWARE_VERSION2);
	gpio_free(HARDWARE_VERSION3);
	
	kobject_put(zte_board_id_kobj);
}
//[ECID:0000] ZTEBSP maxiaoping 20120207  for NV platform hardware board it acquisition,end.

module_init(zte_board_id_init);
module_exit(zte_board_id_exit);

//[ECID:0000] ZTEBSP maxiaoping 20111223  for NV platform hardware board it acquisition,end.