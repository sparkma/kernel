/*
 * zi2848.c - zi2848 flash kernel driver
 *
 * Copyright (C) 2011 NVIDIA Corp.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */


/*ZTE yuxin add this file for flash ZI2848 chip,2011.10.25*/

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <media/zi2848.h>
//#include <kernel/arch/arm/mach-tegra/gpio-names.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>

//zi2848 FENS gpio pin
#define ZI2848_FLASH_EN_GPIO   220 //TEGRA_GPIO_PBB4


static  int zi2848_light_on(u8 gpio,u8 level)
{
   int i=0;

    printk("%s: GPIO is %d ,level is %d\n",__func__,gpio,level);
	
   //enable the gpio
   #if 0   //yuxin del ,move to board-whistler-sensor.c,whistler_camera_init()
   tegra_gpio_enable(gpio);
   gpio_request(gpio, "zi2848_flash_enable");
   gpio_direction_output(gpio, 0);
   gpio_export(gpio, false);
   gpio_set_value(gpio,0);  //set the value as 0 
  #endif

   if(level==0)
   	return -1;
   


  // 1 rising edge of the FENS indicates 1 level of the brightness 
  //zi2848 can recognise the rising edge from 15KHz  to 1MHz
   for(i=1; i<level; i++)
   	{
   	     gpio_set_value(gpio, 1);
	          udelay(25);           //20KHz,  can be changed
	     gpio_set_value(gpio, 0);
                 udelay(25);
   	}
   
   //keep the FENS high longer than 500us for flash working
    gpio_set_value(gpio, 1); //the last rising edge 
   udelay(1000);
    return 0;
}

static  int zi2848_light_off(u8 gpio)
{
    printk("%s: GPIO is %d \n",__func__,gpio);
	
    //if the FENS keep low for longer than 500us,the zi2848 shut down
      gpio_set_value(gpio, 0); 
	  udelay(1000);  
	return 0;

}


static long zi2848_ioctl(
	struct file *file,
	unsigned int cmd,
	unsigned long arg)
{
	u8 val = (u8)arg;
	printk("%s:val=%ld\n",__func__,arg);

	switch (cmd) {
	case ZI2848_IOCTL_MODE_SHUTDOWN:
	case ZI2848_IOCTL_MODE_STANDBY:
		
		zi2848_light_off(ZI2848_FLASH_EN_GPIO);
              break;
				  
	case ZI2848_IOCTL_MODE_FLASH:
		zi2848_light_on( ZI2848_FLASH_EN_GPIO,val);
              break;
	
	case ZI2848_IOCTL_MODE_LED:
	case ZI2848_IOCTL_MODE_TORCH:	
	default:
		return -1;
	}
	
   return 0;
}


static int zi2848_open(struct inode *inode, struct file *file)
{
       printk("%s \n",__func__);
	return 0;
}

int zi2848_release(struct inode *inode, struct file *file)
{
	printk("%s \n",__func__);
	return 0;
}


static const struct file_operations zi2848_fileops = {
	.owner = THIS_MODULE,
	.open = zi2848_open,
	.unlocked_ioctl = zi2848_ioctl,
	.release = zi2848_release,
};

static  struct miscdevice  zi2848_device={
	.minor = MISC_DYNAMIC_MINOR,
	.name = "zi2848",
	.fops = &zi2848_fileops,
     
};


static int __init zi2848_init(void)
{
	int err;
	printk("ZI2848_init \n");
	err = misc_register(&zi2848_device);
	return err;
}

static int __exit zi2848_exit(void)
{
	int err;
	printk("ZI2848_exit \n");
	err = misc_deregister(&zi2848_device);
	return err;
}

module_init(zi2848_init);
module_exit(zi2848_exit);

