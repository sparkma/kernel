/*
 *  kernel/drivers/misc/bsp_debug.c
 *
 *  Copyright (C) 2011 ZTE
 *
 * Modified for bsp debug control
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
static unsigned long print_control_flag=0;
static struct kobject *bsp_debug_kobj;

static ssize_t print_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
        return sprintf(buf, "0x%lx\n", print_control_flag);
}
static ssize_t print_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t n)
{
	//buf= 0x....
	if(n<2)
	{
		return -EINVAL;
	}
	if((buf[0]=='0') && (buf[1]=='x' || (buf[1]=='X')))
	{
		if(!strict_strtoul(buf+2,16,&print_control_flag))
		{
			return -EINVAL;
		}
		return n;
	}
	
	return -EINVAL;
}

int is_bsp_printable(unsigned long int flag)
{
	return (flag&print_control_flag);
}
EXPORT_SYMBOL_GPL(is_bsp_printable);

static struct kobj_attribute printk_attribute =
	__ATTR(printk, 0755, print_show, print_store);


static struct attribute *attrs[] = {
	&printk_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};


static int __init bsp_debug_init(void)
{
	int retval;

	/*
	 * Create a /sys/bsp_debug/
	 *
	 */
	bsp_debug_kobj = kobject_create_and_add("bsp_debug", NULL);
	if (!bsp_debug_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(bsp_debug_kobj, &attr_group);
	if (retval)
		kobject_put(bsp_debug_kobj);

	return retval;	
	
}

static void __exit bsp_debug_exit(void)
{
	kobject_put(bsp_debug_kobj);
}

module_init(bsp_debug_init);
module_exit(bsp_debug_exit);

