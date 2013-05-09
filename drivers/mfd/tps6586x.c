/*
 * Core driver for TI TPS6586x PMIC family
 *
 * Copyright (c) 2010 CompuLab Ltd.
 * Mike Rapoport <mike@compulab.co.il>
 *
 * Based on da903x.c.
 * Copyright (C) 2008 Compulab, Ltd.
 * Mike Rapoport <mike@compulab.co.il>
 * Copyright (C) 2006-2008 Marvell International Ltd.
 * Eric Miao <eric.miao@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
//[ECID:000000]ZTEBSP DangXiao 20120129 start,for thermal sensor wake lock
#include <linux/earlysuspend.h>
//[ECID:000000]ZTEBSP DangXiao 20120129 end,for thermal sensor wake lock

#include <linux/mfd/core.h>
#include <linux/mfd/tps6586x.h>

/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111118*/
#include <linux/wakelock.h>
#include "../../../arch/arm/mach-tegra/cpu-tegra.h"
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111118*/

#define TPS6586X_SUPPLYENE  0x14
#define EXITSLREQ_BIT       BIT(1) /* Exit sleep mode request */
#define SLEEP_MODE_BIT      BIT(3) /* Sleep mode */

/* GPIO control registers */
#define TPS6586X_GPIOSET1	0x5d
#define TPS6586X_GPIOSET2	0x5e

/* interrupt control registers */
#define TPS6586X_INT_ACK1	0xb5
#define TPS6586X_INT_ACK2	0xb6
#define TPS6586X_INT_ACK3	0xb7
#define TPS6586X_INT_ACK4	0xb8

/* interrupt mask registers */
#define TPS6586X_INT_MASK1	0xb0
#define TPS6586X_INT_MASK2	0xb1
#define TPS6586X_INT_MASK3	0xb2
#define TPS6586X_INT_MASK4	0xb3
#define TPS6586X_INT_MASK5	0xb4

/* device id */
#define TPS6586X_VERSIONCRC	0xcd

/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20120203 */
#define NO_TEMP_TIMER

struct tps6586x_irq_data {
	u8	mask_reg;
	u8	mask_mask;
};

#define TPS6586X_IRQ(_reg, _mask)				\
	{							\
		.mask_reg = (_reg) - TPS6586X_INT_MASK1,	\
		.mask_mask = (_mask),				\
	}
/*[ECID:000000] ZTEBSP shiyan 20111009 start,  for charger */
static int charger_online = 0;
/*[ECID:000000] ZTEBSP shiyan 20111009 end,  for charger */

static const struct tps6586x_irq_data tps6586x_irqs[] = {
	[TPS6586X_INT_PLDO_0]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 0),
	[TPS6586X_INT_PLDO_1]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 1),
	[TPS6586X_INT_PLDO_2]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 2),
	[TPS6586X_INT_PLDO_3]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 3),
	[TPS6586X_INT_PLDO_4]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 4),
	[TPS6586X_INT_PLDO_5]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 5),
	[TPS6586X_INT_PLDO_6]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 6),
	[TPS6586X_INT_PLDO_7]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 7),
	[TPS6586X_INT_COMP_DET]	= TPS6586X_IRQ(TPS6586X_INT_MASK4, 1 << 0),
	[TPS6586X_INT_ADC]	= TPS6586X_IRQ(TPS6586X_INT_MASK2, 1 << 1),
	[TPS6586X_INT_PLDO_8]	= TPS6586X_IRQ(TPS6586X_INT_MASK2, 1 << 2),
	[TPS6586X_INT_PLDO_9]	= TPS6586X_IRQ(TPS6586X_INT_MASK2, 1 << 3),
	[TPS6586X_INT_PSM_0]	= TPS6586X_IRQ(TPS6586X_INT_MASK2, 1 << 4),
	[TPS6586X_INT_PSM_1]	= TPS6586X_IRQ(TPS6586X_INT_MASK2, 1 << 5),
	[TPS6586X_INT_PSM_2]	= TPS6586X_IRQ(TPS6586X_INT_MASK2, 1 << 6),
	[TPS6586X_INT_PSM_3]	= TPS6586X_IRQ(TPS6586X_INT_MASK2, 1 << 7),
	[TPS6586X_INT_RTC_ALM1]	= TPS6586X_IRQ(TPS6586X_INT_MASK5, 1 << 4),
	[TPS6586X_INT_ACUSB_OVP] = TPS6586X_IRQ(TPS6586X_INT_MASK5, 0x03),
	[TPS6586X_INT_USB_DET]	= TPS6586X_IRQ(TPS6586X_INT_MASK5, 1 << 2),
	[TPS6586X_INT_AC_DET]	= TPS6586X_IRQ(TPS6586X_INT_MASK5, 1 << 3),
	[TPS6586X_INT_BAT_DET]	= TPS6586X_IRQ(TPS6586X_INT_MASK3, 1 << 0),
	[TPS6586X_INT_CHG_STAT]	= TPS6586X_IRQ(TPS6586X_INT_MASK4, 0xfc),
	[TPS6586X_INT_CHG_TEMP]	= TPS6586X_IRQ(TPS6586X_INT_MASK3, 0x06),
	[TPS6586X_INT_PP]	= TPS6586X_IRQ(TPS6586X_INT_MASK3, 0xf0),
	[TPS6586X_INT_RESUME]	= TPS6586X_IRQ(TPS6586X_INT_MASK5, 1 << 5),
	[TPS6586X_INT_LOW_SYS]	= TPS6586X_IRQ(TPS6586X_INT_MASK5, 1 << 6),
	[TPS6586X_INT_RTC_ALM2] = TPS6586X_IRQ(TPS6586X_INT_MASK4, 1 << 1),
};

struct tps6586x {
	struct mutex		lock;
	struct device		*dev;
	struct i2c_client	*client;

	struct gpio_chip	gpio;
	struct irq_chip		irq_chip;
	struct mutex		irq_lock;
	int			irq_base;
	u32			irq_en;
	u8			mask_cache[5];
	u8			mask_reg[5];

#if 0
/*wangbing, for thermal sensor shutdown, 20111105, start */
	struct timer_list 		resume_reset_timer;
	u16			resume_reset_time;
	int			resume_gpio;
	struct work_struct 		resume_work;
	spinlock_t 	resume_lock; 	
	struct mutex		resume_mutex;
	int 		resume_timer_delete;
	signed int 	shutdown_temp;
/*wangbing, for thermal sensor shutdown, 20111105, end */
#endif

/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111118, start */
	struct timer_list 		temp_timer;
	u16			temp_time;
//	struct timer_list		delayed_pmu_could_sleep_timer;
	int			temp_gpio;
	struct timer_list 		gpio_debounce_timer;
	u16			debounce_time;
	struct work_struct 		temp_work;
	struct work_struct 		force_normal_work;
//	struct work_struct 		delayed_work;
	spinlock_t 	temp_lock; 	
	struct mutex		temp_mutex;
	int 		temp_timer_delete;
//	int		temp_pmu_could_sleep;
	signed int 	shutdown_temp;
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111118, end */

//[ECID:000000]ZTEBSP DangXiao 20120129 start,for thermal sensor wake lock
	struct early_suspend    early_suspend;
//[ECID:000000]ZTEBSP DangXiao 20120129 end,for thermal sensor wake lock
	
};



/*[ECID:000000] ZTEBSP shiyan 20111009  start, for charger  */
extern void zte_ac_present_irq(void); 
extern void zte_chg_state_change_irq(void); 
extern bool zte_bat_driver_registered(void);
/*[ECID:000000] ZTEBSP shiyan 20111009  end,  for charger  */

/*[ECID:000000] ZTEBSP khl, for usb-in suspend, 201112.30 start*/
 static struct wake_lock usb_wake_lock;
/*[ECID:000000] ZTEBSP khl, for usb-in suspend, 201112.30 end*/
static inline int __tps6586x_read(struct i2c_client *client,
				  int reg, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
		return ret;
	}

	*val = (uint8_t)ret;

	return 0;
}

static inline int __tps6586x_reads(struct i2c_client *client, int reg,
				   int len, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg, len, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading from 0x%02x\n", reg);
		return ret;
	}

	return 0;
}

static inline int __tps6586x_write(struct i2c_client *client,
				 int reg, uint8_t val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed writing 0x%02x to 0x%02x\n",
				val, reg);
		return ret;
	}

	return 0;
}

static inline int __tps6586x_writes(struct i2c_client *client, int reg,
				  int len, uint8_t *val)
{
	int ret, i;

	for (i = 0; i < len; i++) {
		ret = __tps6586x_write(client, reg + i, *(val + i));
		if (ret < 0)
			return ret;
	}

	return 0;
}

int tps6586x_write(struct device *dev, int reg, uint8_t val)
{
	return __tps6586x_write(to_i2c_client(dev), reg, val);
}
EXPORT_SYMBOL_GPL(tps6586x_write);

int tps6586x_writes(struct device *dev, int reg, int len, uint8_t *val)
{
	return __tps6586x_writes(to_i2c_client(dev), reg, len, val);
}
EXPORT_SYMBOL_GPL(tps6586x_writes);

int tps6586x_read(struct device *dev, int reg, uint8_t *val)
{
	return __tps6586x_read(to_i2c_client(dev), reg, val);
}
EXPORT_SYMBOL_GPL(tps6586x_read);

int tps6586x_reads(struct device *dev, int reg, int len, uint8_t *val)
{
	return __tps6586x_reads(to_i2c_client(dev), reg, len, val);
}
EXPORT_SYMBOL_GPL(tps6586x_reads);

int tps6586x_set_bits(struct device *dev, int reg, uint8_t bit_mask)
{
	struct tps6586x *tps6586x = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&tps6586x->lock);

	ret = __tps6586x_read(to_i2c_client(dev), reg, &reg_val);
	if (ret)
		goto out;

	if ((reg_val & bit_mask) == 0) {
		reg_val |= bit_mask;
		ret = __tps6586x_write(to_i2c_client(dev), reg, reg_val);
	}
out:
	mutex_unlock(&tps6586x->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tps6586x_set_bits);

int tps6586x_clr_bits(struct device *dev, int reg, uint8_t bit_mask)
{
	struct tps6586x *tps6586x = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&tps6586x->lock);

	ret = __tps6586x_read(to_i2c_client(dev), reg, &reg_val);
	if (ret)
		goto out;

	if (reg_val & bit_mask) {
		reg_val &= ~bit_mask;
		ret = __tps6586x_write(to_i2c_client(dev), reg, reg_val);
	}
out:
	mutex_unlock(&tps6586x->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tps6586x_clr_bits);

int tps6586x_update(struct device *dev, int reg, uint8_t val, uint8_t mask)
{
	struct tps6586x *tps6586x = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&tps6586x->lock);

	ret = __tps6586x_read(tps6586x->client, reg, &reg_val);
	if (ret)
		goto out;

	if ((reg_val & mask) != val) {
		reg_val = (reg_val & ~mask) | val;
		ret = __tps6586x_write(tps6586x->client, reg, reg_val);
	}
out:
	mutex_unlock(&tps6586x->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tps6586x_update);

static struct i2c_client *tps6586x_i2c_client = NULL;
int tps6586x_power_off(void)
{
	struct device *dev = NULL;
	int ret = -EINVAL;
	int val = 0;	//EC0000:ztebsp shiyan for clear reset flag 2012.3.22

	if (!tps6586x_i2c_client)
		return ret;

	dev = &tps6586x_i2c_client->dev;
	
//EC0000:ztebsp shiyan for clear reset flag 2012.2.21
	ret = tps6586x_write(dev,0xc1,0);
	if (ret)
		return ret;
//EC0000:ztebsp shiyan for clear reset flag 2012.2.21

	//EC0000:ztebsp shiyan for clear reset flag 2012.3.22
	ret = tps6586x_read(dev,0xbb,&val);
	if (ret)
		return ret;
	else
	{
		//数据线插入时 重启
		if((val & 0x04)==0x04)
		{
			ret = tps6586x_read(dev,0x14,&val);
			if(ret)
				return ret;
			else
			{
				ret = tps6586x_write(dev,0x14,val|0x01);
				if(ret)
					return ret;
			}
				
		}
		else
		{
			ret = tps6586x_clr_bits(dev, TPS6586X_SUPPLYENE, EXITSLREQ_BIT);
			if (ret)
				return ret;

			ret = tps6586x_set_bits(dev, TPS6586X_SUPPLYENE, SLEEP_MODE_BIT);
			if (ret)
				return ret;
		}
	}
	//EC0000:ztebsp shiyan for clear reset flag 2012.3.22
	return 0;
}

/*[ECID:000000] ZTEBSP shiyan 20111009  start, for charger */
int tps6586x_en_sm2(void)
{
        struct device *dev = NULL;
        volatile unsigned int val = 0;
        int ret = -EINVAL;

        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x12,&val);
        val |= 0x80;
        ret = tps6586x_write(dev,0x12,val);

        ret = tps6586x_read(dev,0x13,&val);
        val |= 0x80;
        ret = tps6586x_write(dev,0x13,val);
        return ret;
}

int tps686x_en_chg_norm(void)
{
        struct device *dev = NULL;
        volatile unsigned int val = 0;
        int ret = -EINVAL;

        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

	//设置恒流充电的电流
	//bit3:2 =10,  648mA   11 :864mA
	ret = tps6586x_read(dev,0x49,&val);
	val |= 0xcc;
	ret = tps6586x_write(dev,0x49,val);

	//设置USB的最大电流限定值
	//bit2=1   500mA
	//bit2:1=x1   2.1A
	ret = tps6586x_read(dev,0x4c,&val);
   	val &= ~0x06;
	val |= 0x04;   
	ret = tps6586x_write(dev,0x4c,val);
		
	// bit1=1使能充电，表示sleep和normal都充电
	//bit3:2= 01 充电电压 4.15V  10: 4.2V
	ret = tps6586x_read(dev,0x4a,&val);
	val &=~ 0x0c;  
	val |= 0x08;   //0x04
	val |= 0x02;	
	ret = tps6586x_write(dev,0x4a,val);

	//bit7=1 使能充电，设置截止电流和涓流电流
	//bit7=1  bit3:2=10  bit1:0=01   
	ret = tps6586x_read(dev,0x4b,&val);
	val |= 0x80;
	val &= ~0x0f;
	val |= 0x05;  //0x0d
	ret = tps6586x_write(dev,0x4b,val);

	//bit2=0  使能充电状态改变的中断
	ret = tps6586x_read(dev,0xb3,&val);
	val &= (~0x04);
	ret = tps6586x_write(dev,0xb3,val);


        return ret;
}

EXPORT_SYMBOL(tps6586x_en_sm2);

int tps6586x_dis_sm2(void)
{
        struct device *dev = NULL;
        volatile unsigned int val = 0;
        int ret = -EINVAL;

        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x12,&val);
        val &= (~(0x80));
        ret = tps6586x_write(dev,0x12,val);

        ret = tps6586x_read(dev,0x13,&val);
        val &= (~(0x80));
        ret = tps6586x_write(dev,0x13,val);

	//bit1=0  禁止充电状态改变的中断
	ret = tps6586x_read(dev,0xb3,&val);
	val |= 0x04;
	ret = tps6586x_write(dev,0xb3,val);
        return ret;
}
EXPORT_SYMBOL(tps6586x_dis_sm2);

int tps686x_dis_chg_norm(void)
{
        struct device *dev = NULL;
        volatile unsigned int val = 0;
        int ret = -EINVAL;

        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x4a,&val);
        val &= (~(0x02));
        ret = tps6586x_write(dev,0x4a,val);

}

int tps6586x_force_normal(u8 pmu_can_sleep)
{
        struct device *dev = NULL;
        volatile u8 val = 0;
        int ret = -EINVAL;

       //printk("%s enter, %d\n", __func__, pmu_can_sleep);
    
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x14,&val);
        //printk("PM_DEBUG_MXP:SUPPLYENE REG 0x14 = 0x%x\n",val);
        val |= 0x02;
        ret = tps6586x_write(dev,0x14,val);        
        //printk("PM_DEBUG_MXP:SUPPLYENE REG 0x14 = 0x%x after set tps normal state.\n",val);
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111105, start */
	 if (pmu_can_sleep) {
	 	 val &= ~0x02;
	        ret = tps6586x_write(dev,0x14,val);        
	        //printk("PM_DEBUG_MXP:SUPPLYENE REG 0x14 = 0x%x after clear tps normal state.\n",val);
	 }	
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111105, end */
	//printk("%s exit, %d\n", __func__,  pmu_can_sleep);
	return ret;	
	
}

/*[ECID:000000] ZTEBSP wangbing, for rtc out, 20111130, start */
int tps6586x_set_rtc_out(void)
{
        struct device *dev = NULL;
        volatile u8 val = 0;
        int ret = -EINVAL;

        if(!tps6586x_i2c_client)
            return ret;
        dev = &tps6586x_i2c_client->dev;

		printk("%s\n", __func__);		
        ret = tps6586x_read(dev,0x44,&val);
        val |= 0x38;
        ret = tps6586x_write(dev,0x44,val);    
}
/*[ECID:000000] ZTEBSP wangbing, for rtc out, 20111130, end */

void tps6586x_onkey_irq_enable(void)
{
        struct device *dev = NULL;
        volatile unsigned int val = 0;
        int ret = -EINVAL;

        printk("Enter into tps6586x_onkey_irq_enable\n");
    
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        //ret = tps6586x_read(dev,0xb4,&val);
        //printk("get mask reg: 0x%x\n",val);
        //val &= (~(0x20));
        //ret = tps6586x_write(dev,0xb4,val);
        //ret = tps6586x_write(dev,0xb0,0);
        //ret = tps6586x_write(dev,0xb1,0);
        ret = tps6586x_write(dev,0xb2,0);
        ret = tps6586x_write(dev,0xb3,0);
        ret = tps6586x_write(dev,0xb4,0);

        ret = tps6586x_read(dev,0x14,&val);
        printk("get pmu 0x14 reg: 0x%x\n",val);
        val |= 0x2;
        ret = tps6586x_write(dev,0x14,val);
        
         //ret = tps6586x_read(dev,0xb4,&val);
         //printk("5 secs later will trigger an int*******************************\n");

        //msleep(5000);
        // trigger an int
        //tps6586x_write(dev,0xb8,1);
         
        printk("write mask reg: 0x%x",val);
}

//[ECID:0000] ZTEBSP maxiaoping 20120203 for MIMOSA keypad led control,start
int tps6586x_keybd_led_on(void)
{
    struct device *dev = NULL;
        volatile unsigned int val = 0;
        int ret = -EINVAL;
    
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x59,&val);	
        //val |= 0xff;
        val |= 0x7f;
        ret = tps6586x_write(dev,0x59,val);
        ret = tps6586x_read(dev,0x59,&val);
}
//[ECID:0000] ZTEBSP maxiaoping 20120203 for MIMOSA keypad led control,end

EXPORT_SYMBOL(tps6586x_keybd_led_on);
int tps6586x_keybd_led_off(void)
{
    struct device *dev = NULL;
        volatile unsigned int val = 0;
        int ret = -EINVAL;
    
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x59,&val);
        ret = tps6586x_write(dev,0x59,0x0);
}
EXPORT_SYMBOL(tps6586x_keybd_led_off);

 int zte_tps6586x_read(int reg)
{
    struct device *dev = NULL;
        volatile unsigned int val = 0;
        int ret = -EINVAL;
    
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,reg,&val);
	return val;	
}
EXPORT_SYMBOL(zte_tps6586x_read);

 int zte_tps6586x_write(int reg,uint8_t val)
{
    struct device *dev = NULL;
        int ret = -EINVAL;
    
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_write(dev,reg,val);
	if (ret < 0) 
	{
		printk("failed writing 0x%02x to 0x%02x\n", val, reg);
	}		
	return ret;	
}
EXPORT_SYMBOL(zte_tps6586x_write);

/*[ECID:000000] ZTEBSP shiyan 20111009  end, for charger */

static int tps6586x_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct tps6586x *tps6586x = container_of(gc, struct tps6586x, gpio);
	uint8_t val;
	int ret;

	ret = __tps6586x_read(tps6586x->client, TPS6586X_GPIOSET2, &val);
	if (ret)
		return ret;

	return !!(val & (1 << offset));
}


static void tps6586x_gpio_set(struct gpio_chip *chip, unsigned offset,
			      int value)
{
	struct tps6586x *tps6586x = container_of(chip, struct tps6586x, gpio);

	tps6586x_update(tps6586x->dev, TPS6586X_GPIOSET2,
			value << offset, 1 << offset);
}

static int tps6586x_gpio_input(struct gpio_chip *gc, unsigned offset)
{
	/* FIXME: add handling of GPIOs as dedicated inputs */
	return -ENOSYS;
}

static int tps6586x_gpio_output(struct gpio_chip *gc, unsigned offset,
				int value)
{
	struct tps6586x *tps6586x = container_of(gc, struct tps6586x, gpio);
	uint8_t val, mask;
	int ret;

	val = value << offset;
	mask = 0x1 << offset;
	ret = tps6586x_update(tps6586x->dev, TPS6586X_GPIOSET2, val, mask);
	if (ret)
		return ret;

	val = 0x1 << (offset * 2);
	mask = 0x3 << (offset * 2);

	return tps6586x_update(tps6586x->dev, TPS6586X_GPIOSET1, val, mask);
}

static int tps6586x_gpio_init(struct tps6586x *tps6586x, int gpio_base)
{
	if (!gpio_base)
		return 0;

	tps6586x->gpio.owner		= THIS_MODULE;
	tps6586x->gpio.label		= tps6586x->client->name;
	tps6586x->gpio.dev		= tps6586x->dev;
	tps6586x->gpio.base		= gpio_base;
	tps6586x->gpio.ngpio		= 4;
	tps6586x->gpio.can_sleep	= 1;

	tps6586x->gpio.direction_input	= tps6586x_gpio_input;
	tps6586x->gpio.direction_output	= tps6586x_gpio_output;
	tps6586x->gpio.set		= tps6586x_gpio_set;
	tps6586x->gpio.get		= tps6586x_gpio_get;

	return gpiochip_add(&tps6586x->gpio);
}

static int __remove_subdev(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int tps6586x_remove_subdevs(struct tps6586x *tps6586x)
{
	return device_for_each_child(tps6586x->dev, NULL, __remove_subdev);
}

static void tps6586x_irq_lock(struct irq_data *data)
{
	struct tps6586x *tps6586x = irq_data_get_irq_chip_data(data);

	mutex_lock(&tps6586x->irq_lock);
}

static void tps6586x_irq_enable(struct irq_data *irq_data)
{
	struct tps6586x *tps6586x = irq_data_get_irq_chip_data(irq_data);
	unsigned int __irq = irq_data->irq - tps6586x->irq_base;
	const struct tps6586x_irq_data *data = &tps6586x_irqs[__irq];

	tps6586x->mask_reg[data->mask_reg] &= ~data->mask_mask;
	tps6586x->irq_en |= (1 << __irq);
}

static void tps6586x_irq_disable(struct irq_data *irq_data)
{
	struct tps6586x *tps6586x = irq_data_get_irq_chip_data(irq_data);

	unsigned int __irq = irq_data->irq - tps6586x->irq_base;
	const struct tps6586x_irq_data *data = &tps6586x_irqs[__irq];

	tps6586x->mask_reg[data->mask_reg] |= data->mask_mask;
	tps6586x->irq_en &= ~(1 << __irq);
}

static void tps6586x_irq_sync_unlock(struct irq_data *data)
{
	struct tps6586x *tps6586x = irq_data_get_irq_chip_data(data);
	int i;

	for (i = 0; i < ARRAY_SIZE(tps6586x->mask_reg); i++) {
		if (tps6586x->mask_reg[i] != tps6586x->mask_cache[i]) {
			if (!WARN_ON(tps6586x_write(tps6586x->dev,
						    TPS6586X_INT_MASK1 + i,
						    tps6586x->mask_reg[i])))
				tps6586x->mask_cache[i] = tps6586x->mask_reg[i];
		}
	}

	mutex_unlock(&tps6586x->irq_lock);
}

/*[ECID:000000] ZTEBSP shiyan 20111009  start, for charger  */
static int usb_flag=0;

int read_usb_flag(void)
{
	return usb_flag;
}
EXPORT_SYMBOL(read_usb_flag);

void set_usb_flag(void)
{
       /*[ECID:000000] ZTEBSP khl, for usb-in suspend, 201112.30 start*/
	wake_lock(&usb_wake_lock);
	printk("%s usb_wake_lock lock.\n", __func__);
	/*[ECID:000000] ZTEBSP khl, for usb-in suspend, 201112.30 end*/   
	 usb_flag =1;
}
EXPORT_SYMBOL(set_usb_flag);

static irqreturn_t tps6586x_irq(int irq, void *data)
{
	struct tps6586x *tps6586x = data;
	u32 acks;
	int val = 0;
	int ret = 0;

	//先禁止状态改变和插拔再次中断
	val = zte_tps6586x_read(0xb3);
	zte_tps6586x_write(0xb3, val|0x04);    //0x04
	val = zte_tps6586x_read(0xb4);
	zte_tps6586x_write(0xb4, val|0x04);    //0x04

	ret = tps6586x_reads(tps6586x->dev, TPS6586X_INT_ACK1,
			     sizeof(acks), (uint8_t *)&acks);

	if (ret < 0) 
	{
		dev_err(tps6586x->dev, "failed to read interrupt status\n");
		return IRQ_NONE;
	}
	
	acks = le32_to_cpu(acks);
	#ifdef BATTERY_DEBUG
	printk(" ****tps6586x_irq**** Reg(0xB8B7B6B5) is 0x%x \n",acks);
	#endif
	
	//充电器插拔中断 0xB7 :bit2
	if(acks&0x00040000)
    {
		//更新电池状态
		if(true == zte_bat_driver_registered())
		{
			zte_ac_present_irq();
		}
		
		ret = tps6586x_read(tps6586x->dev, 0xbb, &val);

		if (ret < 0) 
		{
			dev_err(tps6586x->dev, "failed to read interrupt status\n");
			return IRQ_NONE;
		}
		
		//充电器插入
		if(val & 0x04)  
        {
			charger_online =1;
			tps6586x_en_sm2();
			tps686x_en_chg_norm();
		}
		//充电器拔出
		else
		{
              /*[ECID:000000] ZTEBSP khl, for usb-in suspend, 201112.30 start*/
		if(1==read_usb_flag())
                	{
                     wake_unlock(&usb_wake_lock);
	              printk("%s khl usb_wake_lock unlock.\n", __func__);
			}
		/*[ECID:000000] ZTEBSP khl, for usb-in suspend, 201112.30 end*/
			charger_online =0;	
			tps6586x_dis_sm2();
			tps686x_dis_chg_norm();
			usb_flag=0;
		}
	}
		
	//充电状态改变中断
	else if(acks&0x00200000)
	{
		if(true == zte_bat_driver_registered())
		{
		//	zte_chg_state_change_irq();   //shiyan delete  2012.5.11
		}
	}
	
	while (acks) 
	{
		int i = __ffs(acks);

		if (tps6586x->irq_en & (1 << i))
			handle_nested_irq(tps6586x->irq_base + i);

		acks &= ~(1 << i);
	}
	//val = zte_tps6586x_read(0xb3);           //状态改变
	//zte_tps6586x_write(0xb3, val&0xfb);    //bit2=0   1111 1011
	val = zte_tps6586x_read(0xb4);
	zte_tps6586x_write(0xb4,  val&0xfb);  

	return IRQ_HANDLED;
}

int zte_charger_on(void)
{
       //插着充电器开机，待机界面不显示充电图标问题
       //ec 000000 luyanfei modify here 20111223  ++
       int val; 
	   
       val = zte_tps6586x_read(0xbb);
	 if (val&0x04)
	 {
	 	return  1;
	 }
	 else
	 {
	 	return 0;
	 }
}
EXPORT_SYMBOL(zte_charger_on);

int read_charger_on_flag(void)
{
	return  charger_online;
}
EXPORT_SYMBOL(read_charger_on_flag);

/*[ECID:000000] ZTEBSP shiyan 20111009  end, for charger  */	

static int __devinit tps6586x_irq_init(struct tps6586x *tps6586x, int irq,
				       int irq_base)
{
	int i, ret;
	u8 tmp[4];

	if (!irq_base) {
		dev_warn(tps6586x->dev, "No interrupt support on IRQ base\n");
		return -EINVAL;
	}

	mutex_init(&tps6586x->irq_lock);
	for (i = 0; i < 5; i++) {
		tps6586x->mask_cache[i] = 0xff;
		tps6586x->mask_reg[i] = 0xff;
		tps6586x_write(tps6586x->dev, TPS6586X_INT_MASK1 + i, 0xff);
	}

	tps6586x_reads(tps6586x->dev, TPS6586X_INT_ACK1, sizeof(tmp), tmp);

	tps6586x->irq_base = irq_base;

	tps6586x->irq_chip.name = "tps6586x";
	tps6586x->irq_chip.irq_enable = tps6586x_irq_enable;
	tps6586x->irq_chip.irq_disable = tps6586x_irq_disable;
	tps6586x->irq_chip.irq_bus_lock = tps6586x_irq_lock;
	tps6586x->irq_chip.irq_bus_sync_unlock = tps6586x_irq_sync_unlock;

	for (i = 0; i < ARRAY_SIZE(tps6586x_irqs); i++) {
		int __irq = i + tps6586x->irq_base;
		irq_set_chip_data(__irq, tps6586x);
		irq_set_chip_and_handler(__irq, &tps6586x->irq_chip,
					 handle_simple_irq);
		irq_set_nested_thread(__irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(__irq, IRQF_VALID);
#endif
	}

	ret = request_threaded_irq(irq, NULL, tps6586x_irq, IRQF_ONESHOT,
				   "tps6586x", tps6586x);

	if (!ret) {
		device_init_wakeup(tps6586x->dev, 1);
		enable_irq_wake(irq);
	}

	return ret;
}

static int __devinit tps6586x_add_subdevs(struct tps6586x *tps6586x,
					  struct tps6586x_platform_data *pdata)
{
	struct tps6586x_subdev_info *subdev;
	struct platform_device *pdev;
	int i, ret = 0;

	for (i = 0; i < pdata->num_subdevs; i++) {
		subdev = &pdata->subdevs[i];

		pdev = platform_device_alloc(subdev->name, subdev->id);
		if (!pdev) {
			ret = -ENOMEM;
			goto failed;
		}

		pdev->dev.parent = tps6586x->dev;
		pdev->dev.platform_data = subdev->platform_data;

		ret = platform_device_add(pdev);
		if (ret) {
			platform_device_put(pdev);
			goto failed;
		}
	}
	return 0;

failed:
	tps6586x_remove_subdevs(tps6586x);
	return ret;
}

#if 0
/*wangbing, for thermal sensor shutdown, 20111105, start */
static void tps6586x_resume_work_func(struct work_struct *work)
{
	struct tps6586x *tps6586x
		= container_of(work, struct tps6586x, resume_work);
	signed int data = -1;
	static u8 ext_temp_read_count = 0;
	static u8 ext_temp_higher_count = 0;

	printk("tps6586x resume work enter, throttling %d, ehc %d erc %d/n",  is_throttling, ext_temp_higher_count, ext_temp_read_count);	
	
	mutex_lock(&tps6586x->resume_mutex);

	if (is_throttling) {
		if ((data = nct1008_ext_temp()) < 0) 
			if (++ext_temp_read_count > 5)
				tps6586x_power_off();	
			else
				(void)tps6586x_force_normal(TRUE);
		else
			ext_temp_read_count = 0;
			if ((data > tps6586x->shutdown_temp) && (ext_temp_higher_count++ > 5))
				tps6586x_power_off();	
			else {
				ext_temp_higher_count = 0;
				(void)tps6586x_force_normal(TRUE);
			}		
	}	
	else {
		ext_temp_read_count = 0;
		ext_temp_higher_count = 0;
		(void)tps6586x_force_normal(TRUE);
	}	

	mutex_unlock(&tps6586x->resume_mutex);

	printk("tps6586x resume work exit, throttling %d/n",  is_throttling);	
	
}

static void tps6586x_resume_timer(unsigned long _data)
{
	struct tps6586x *tps6586x = (struct tps6586x*)_data;
	int gpio_val;
	
	schedule_work(&tps6586x->resume_work);

	if (tps6586x->resume_reset_timer) {
		spin_lock_irqsave(&tps6586x->resume_lock);
		if (tps6586x->resume_timer_delete == TRUE)
//		if ((gpio_val = gpio_get_value(gpio)))
			del_timer(&tps6586x->resume_reset_timer);
		else
			mod_timer(&tps6586x->resume_reset_timer,
				jiffies + msecs_to_jiffies(tps6586x->resume_reset_time));
		spin_unlock_irqrestore(&tps6586x->resume_lock);	
	}
}
	
static irqreturn_t resume_reset_timer_kickoff(int irq, void *dev_id)
{
	struct tps6586x *tps6586x = dev_id;
	int gpio = tps6586x->resume_gpio;
	int gpio_val;
	
	BUG_ON(irq != gpio_to_irq(gpio));

	if (tps6586x->resume_reset_timer) {
		spin_lock_irqsave(&tps6586x->resume_lock);
		if ((gpio_val = gpio_get_value(gpio)))
			tps6586x->resume_timer_delete = TRUE;
		else {
			tps6586x->resume_timer_delete = FALSE;
			if (!timer_pending(&tps6586x->resume_reset_timer))
				mod_timer(&tps6586x->resume_reset_timer,
					jiffies + msecs_to_jiffies(tps6586x->resume_reset_time));		
		}	
		spin_unlock_irqrestore(&tps6586x->resume_lock);
	}
	return IRQ_HANDLED;
}
/*wangbing, for thermal sensor shutdown, 20111105, end */
#endif

/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111118 start*/
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
extern bool tegra_is_throttling(void); 
#endif
extern signed int nct1008_local_temp(void);
extern signed int nct1008_ext_temp(void);
static struct wake_lock temp_wake_lock;
//static struct wake_lock temp_delayed_wake_lock;

static u8 temp_wake_unlock = 0;

/*
static void tps6586x_delayed_work_func(struct work_struct *work)
{
	struct tps6586x *tps6586x
		= container_of(work, struct tps6586x, delayed_work);
	int val;
	unsigned long flags;
	
	printk("%s enter.\n", __func__);
	
	mutex_lock(&tps6586x->temp_mutex);		
#if 1
	if ((val = gpio_get_value(tps6586x->temp_gpio)))	
		(void)tps6586x_force_normal(1);			
#endif

#if 0
	spin_lock_irqsave(&tps6586x->temp_lock, flags);
	wake_unlock(&temp_delayed_wake_lock);
	printk("%s temp_delayed_wake_lock unlock.\n", __func__);			
	spin_unlock_irqrestore(&tps6586x->temp_lock, flags);
#endif	
	mutex_unlock(&tps6586x->temp_mutex);

	printk("%s exit.\n", __func__);
	
}

static void tps6586x_delayed_pmu_could_sleep_timer(unsigned long _data)
{
	struct tps6586x *tps6586x = (struct tps6586x*)_data;
//	int gpio_val;

	printk("%s enter.\n", __func__);

	schedule_work(&tps6586x->delayed_work);

	printk("%s exit.\n", __func__);

}
*/

/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20120203 */
static void tps6586x_get_temp_work_func(void);

static void tps6586x_force_normal_work_func(struct work_struct *work)
{
	struct tps6586x *tps6586x
		= container_of(work, struct tps6586x, force_normal_work);
	int val;
	static int prev_val = 1;

/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20120203 */
	unsigned long flags;

	//printk("%s enter, prev_val %d\n", __func__, prev_val);
	
	mutex_lock(&tps6586x->temp_mutex);		
	
	val = gpio_get_value(tps6586x->temp_gpio); 
	if (val != prev_val) {
		if (val) {
		//whether enable the sleep control of RESUME or not is decided by the pmu status of RESUME.
		//but, we could not get RESUME from PMU reg. 
		//Therefore, delay for a while, then try to disable the force normal function of PMU.
//			mod_timer(&tps6586x->delayed_pmu_could_sleep_timer, jiffies + msecs_to_jiffies(1500));
			(void)tps6586x_force_normal(1);			
		}
		else
			(void)tps6586x_force_normal(0);

		prev_val = val;
	}

/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20120203, start */
#ifdef NO_TEMP_TIMER
	tps6586x_get_temp_work_func();

	spin_lock_irqsave(&tps6586x->temp_lock, flags);
	if (temp_wake_unlock) {
		wake_unlock(&temp_wake_lock);
		//printk("tps6586x temp work, temp_wake_lock unlock.\n");
	}
	spin_unlock_irqrestore(&tps6586x->temp_lock, flags);
#endif
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20120203, end */

	mutex_unlock(&tps6586x->temp_mutex);

	//printk("%s exit, prev_val %d\n", __func__, prev_val);
	
}

/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20120203, start */
#ifdef NO_TEMP_TIMER
static void tps6586x_get_temp_work_func(void)
{
	signed int data = -1;
	static u8 ext_temp_read_count = 0;
	static u8 ext_temp_higher_count = 0;
	static u8 local_temp_read_count = 0;
	static u8 local_temp_higher_count = 0;
	unsigned long flags;
	
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	printk("tps6586x temp work enter, %d\n",  tegra_is_throttling());	

	if (tegra_is_throttling()) {
	
/*external temperature monitor*/
		if ((data = nct1008_ext_temp()) < 0) {
			printk("tps6586x temp work, ext fail, read %d cnt %d\n", data, ext_temp_read_count);
//			if (++ext_temp_read_count > 3)
				tps6586x_power_off();	
		}	
		else {
				printk("tps6586x temp work, ext %d, higher cnt %d\n", data, ext_temp_higher_count);
				ext_temp_read_count = 0;
//				if (data > tps6586x->shutdown_temp) {
				if (data > 112) {

//					if (++ext_temp_higher_count> 3)
						tps6586x_power_off();	
				}	
				else
					ext_temp_higher_count = 0;				
		}
			
/*local temperature monitor*/
		if ((data = nct1008_local_temp()) < 0) {
			printk("tps6586x temp work, local fail, read %d cnt%d\n", data, local_temp_read_count);
//			if (++local_temp_read_count > 3)
				tps6586x_power_off();	
		}	
		else {
				printk("tps6586x temp work, local %d cnt %d\n", data, local_temp_higher_count);
				local_temp_read_count = 0;
				if (data > 112) {  /*tps6586x->shutdown_temp*/
//					if (++local_temp_higher_count> 3)
						tps6586x_power_off();	
				}	
				else 
					local_temp_higher_count = 0;
		}
			
	}	
	else {
		//printk("tps6586x temp work, no throttle.\n");		
		ext_temp_read_count = 0;
		ext_temp_higher_count = 0;
		local_temp_read_count = 0;
		local_temp_higher_count = 0;
//		(void)tps6586x_force_normal(FALSE);
	}	

#if 0
/*ooooh, the reg of 0xbb could not be used to get the RESUME status.*/
	if (!tps6586x_read(&tps6586x->dev, 0xbb, &val)) {
		rd_error_cnt = 0;
		if (val&0x20) {
			tps6586x->temp_pmu_could_sleep = false;		
		} 
		else {
			(void)tps6586x_force_normal(1);	
			tps6586x->temp_pmu_could_sleep = true;
		}	
	} 
	else {
		if (rd_error_cnt++ >= 3)
			tps6586x->temp_pmu_could_sleep = true;
		else
			tps6586x->temp_pmu_could_sleep = false;
		printk("tps6586x temp work, read 0xbb error, cnt %d.\n", rd_error_cnt);			
	}
#endif

	printk("tps6586x temp work exit, %d\n",  tegra_is_throttling());	
#else
	printk("tps6586x temp work enter, no operation due to no CONFIG_TEGRA_THERMAL_THROTTLE.\n");	
#endif	
	
}

#else

static void tps6586x_get_temp_work_func(struct work_struct *work)
{
	struct tps6586x *tps6586x
		= container_of(work, struct tps6586x, temp_work);
	signed int data = -1;
	static u8 ext_temp_read_count = 0;
	static u8 ext_temp_higher_count = 0;
	static u8 local_temp_read_count = 0;
	static u8 local_temp_higher_count = 0;
	unsigned long flags;
	
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	//printk("tps6586x temp work enter, %d\n",  tegra_is_throttling());	
	
	mutex_lock(&tps6586x->temp_mutex);

	if (tegra_is_throttling()) {
	
/*external temperature monitor*/
		if ((data = nct1008_ext_temp()) < 0) {
			printk("tps6586x temp work, ext fail, read %d cnt %d\n", data, ext_temp_read_count);
			if (++ext_temp_read_count > 3)
				tps6586x_power_off();	
		}	
		else {
				printk("tps6586x temp work, ext %d, higher cnt %d\n", data, ext_temp_higher_count);
				ext_temp_read_count = 0;
				if (data > tps6586x->shutdown_temp) {
					if (++ext_temp_higher_count> 3)
						tps6586x_power_off();	
				}	
				else
					ext_temp_higher_count = 0;				
		}
			
/*local temperature monitor*/
		if ((data = nct1008_local_temp()) < 0) {
			printk("tps6586x temp work, local fail, read %d cnt%d\n", data, local_temp_read_count);
			if (++local_temp_read_count > 3)
				tps6586x_power_off();	
		}	
		else {
				printk("tps6586x temp work, local %d cnt %d\n", data, local_temp_higher_count);
				local_temp_read_count = 0;
				if (data > tps6586x->shutdown_temp) {
					if (++local_temp_higher_count> 3)
						tps6586x_power_off();	
				}	
				else 
					local_temp_higher_count = 0;
		}
			
	}	
	else {
		//printk("tps6586x temp work, no throttle.\n");		
		ext_temp_read_count = 0;
		ext_temp_higher_count = 0;
		local_temp_read_count = 0;
		local_temp_higher_count = 0;
//		(void)tps6586x_force_normal(FALSE);
	}	

#if 0
/*ooooh, the reg of 0xbb could not be used to get the RESUME status.*/
	if (!tps6586x_read(&tps6586x->dev, 0xbb, &val)) {
		rd_error_cnt = 0;
		if (val&0x20) {
			tps6586x->temp_pmu_could_sleep = false;		
		} 
		else {
			(void)tps6586x_force_normal(1);	
			tps6586x->temp_pmu_could_sleep = true;
		}	
	} 
	else {
		if (rd_error_cnt++ >= 3)
			tps6586x->temp_pmu_could_sleep = true;
		else
			tps6586x->temp_pmu_could_sleep = false;
		printk("tps6586x temp work, read 0xbb error, cnt %d.\n", rd_error_cnt);			
	}
#endif

	spin_lock_irqsave(&tps6586x->temp_lock, flags);
	if (temp_wake_unlock) {
		wake_unlock(&temp_wake_lock);
		//printk("tps6586x temp work, temp_wake_lock unlock.\n");
	}
	spin_unlock_irqrestore(&tps6586x->temp_lock, flags);
	
	mutex_unlock(&tps6586x->temp_mutex);
	
	//printk("tps6586x temp work exit, %d\n",  tegra_is_throttling());	
#else
	printk("tps6586x temp work enter, no operation due to no CONFIG_TEGRA_THERMAL_THROTTLE.\n");	
#endif	
	
}
#endif
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20120203, end */


static void tps6586x_temp_timer(unsigned long _data)
{
	struct tps6586x *tps6586x = (struct tps6586x*)_data;
//	int gpio_val;
	unsigned long flags;

	//printk("%s enter, time %d, delete %d\n", __func__, tps6586x->temp_time, tps6586x->temp_timer_delete);
	schedule_work(&tps6586x->temp_work);
	if (tps6586x->temp_time) {
		spin_lock_irqsave(&tps6586x->temp_lock, flags);
#if 0
		if ((tps6586x->temp_timer_delete == true) 
			&& (tps6586x->temp_pmu_could_sleep == true)) {
#endif
		if (tps6586x->temp_timer_delete == true) {
			temp_wake_unlock = 1;
		}	
//		if ((gpio_val = gpio_get_value(gpio)))
//			del_timer(&tps6586x->temp_timer);
		else
			mod_timer(&tps6586x->temp_timer,
				jiffies + msecs_to_jiffies(tps6586x->temp_time));
		spin_unlock_irqrestore(&tps6586x->temp_lock,flags);	
	}
	//printk("%s exit, time %d, delete %d\n", __func__, tps6586x->temp_time, tps6586x->temp_timer_delete);

}


static void tps6586x_debounce_timer(unsigned long _data)
{
	struct tps6586x *tps6586x = (struct tps6586x*)_data;
	int gpio = tps6586x->temp_gpio;
	int gpio_val;
	unsigned long flags;

//	printk("%s enter, time %d\n", __func__, tps6586x->debounce_time);

	schedule_work(&tps6586x->force_normal_work);

#ifdef NO_TEMP_TIMER
#else
	if (tps6586x->temp_time) {

		spin_lock_irqsave(&tps6586x->temp_lock, flags);

		if ((gpio_val = gpio_get_value(gpio))) {
			tps6586x->temp_timer_delete = true;
		}			
		else {
			tps6586x->temp_timer_delete = false;
		}

		if (!timer_pending(&tps6586x->temp_timer))
			mod_timer(&tps6586x->temp_timer,
					jiffies + msecs_to_jiffies(tps6586x->temp_time));	

		spin_unlock_irqrestore(&tps6586x->temp_lock, flags);
	}
#endif	
//	printk("%s exit, gpio %d\n", __func__, gpio_val);

}

	
static irqreturn_t tps6586x_temp_isr(int irq, void *dev_id)
{
	struct tps6586x *tps6586x = dev_id;
	int gpio = tps6586x->temp_gpio;
	int gpio_val;
	unsigned long flags;
	
	BUG_ON(irq != gpio_to_irq(gpio));

//	printk("%s enter, time %d, delete %d\n", __func__, tps6586x->temp_time, tps6586x->temp_timer_delete);

//	schedule_work(&tps6586x->force_normal_work);

//	if (tps6586x->temp_time) {
	
	if (tps6586x->debounce_time) {
		spin_lock_irqsave(&tps6586x->temp_lock, flags);
#ifdef NO_TEMP_TIMER
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
			if (!tegra_is_throttling())
#else
	printk("%s, no operation due to no CONFIG_TEGRA_THERMAL_THROTTLE.\n", __func__);	
#endif
			{
				spin_unlock_irqrestore(&tps6586x->temp_lock, flags);				
//				printk("%s exit, no throttle\n", __func__);			
				return IRQ_HANDLED;
			}
#endif

		wake_lock(&temp_wake_lock);
		//printk("%s temp_wake_lock lock.\n", __func__);
		temp_wake_unlock = 0;
#if 0		
		if ((gpio_val = gpio_get_value(gpio))) {
			//whether stop the temp timer or not is decided by the pmu status of RESUME.
			tps6586x->temp_timer_delete = true;
/*			wake_lock(&temp_delayed_wake_lock);
			printk("%s temp_delayed_wake_lock lock.\n", __func__);*/			
		}			
		else {
			tps6586x->temp_timer_delete = false;
/*
			if (timer_pending(&tps6586x->delayed_pmu_could_sleep_timer))
				del_timer(&tps6586x->delayed_pmu_could_sleep_timer);	
*/			
			if (!timer_pending(&tps6586x->temp_timer))
				mod_timer(&tps6586x->temp_timer,
					jiffies + msecs_to_jiffies(tps6586x->temp_time));		
		}
//		printk("%s , exit spinlock\n", __func__);
#else
		mod_timer(&tps6586x->gpio_debounce_timer,
			jiffies + msecs_to_jiffies(tps6586x->debounce_time));		

#endif
		spin_unlock_irqrestore(&tps6586x->temp_lock, flags);
	}

//	printk("%s exit, time %d, delete %d\n", __func__, tps6586x->temp_time, tps6586x->temp_timer_delete);
	
	return IRQ_HANDLED;
}
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111118 end*/

/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111128 start*/
static int to_be_read_register = 0;

static ssize_t tps6586x_register_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = to_i2c_client(dev);
	u8 data = 0;
//	char creg[3] = {0};
//	unsigned long reg = 0;
//       char c = '\0';
	int ret=0;
	   
	if (!dev || !buf || !attr)
		return -EINVAL;

	//printk(KERN_ERR"%s: reg %x\n", __func__, to_be_read_register);
	ret = tps6586x_read(dev, to_be_read_register, &data);
	if (ret < 0) {
		dev_err(dev, "%s: failed to read register %x"
			, __func__, (u8)to_be_read_register);
		return -EINVAL;
	}
	return sprintf(buf, "the value of register %x is %x.\n", to_be_read_register, data);
}

static ssize_t tps6586x_register_write(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	char creg[3] = {0};
	char cval[3] = {0};
	unsigned long reg = 0;
	unsigned long val = 0;
       char c = '\0';
	int ret;
//	u32 buf_cnt = 0;
	u8 data = 0;

	if (!dev || !buf || !attr) {
		return -EINVAL;
	}
	if ((4 != (strlen(buf)-1)) && (2 != (strlen(buf)-1))) {
		return -EINVAL;	
	}

	sprintf(creg, "%c%c%c", buf[0], buf[1], c);
	strict_strtoul(creg, 16, &reg);
	//printk(KERN_ERR"%s: reg %x\n", __func__, (u8)reg);
	
	if (2 == (strlen(buf)-1)) {
		to_be_read_register = reg;
		ret = tps6586x_read(dev, reg, &data);
	  	if (ret < 0) {
	  		dev_err(dev, "%s: failed to read register %x"
	  			, __func__, (u8)reg);
	  	}	
		return count;
	}

	sprintf(cval, "%c%c%c", buf[2], buf[3], c);
	strict_strtoul(cval, 16, &val);

	//printk(KERN_ERR"%s: val %x\n", __func__, (u8)val);

	ret = tps6586x_write(dev, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to write register %x with %x"
			, __func__, (u8)reg, (u8)val);
		return -EINVAL;
	}

	return count;
}



static DEVICE_ATTR(read, S_IRUGO, tps6586x_register_read, NULL);
static DEVICE_ATTR(write, S_IWUSR, NULL, tps6586x_register_write); //[ECID:000000] ZTEBSP wanghaifei 20111226, modify permission

static struct attribute *tps6586x_attributes[] = {
	&dev_attr_read.attr,
	&dev_attr_write.attr,    
	NULL
};

static const struct attribute_group tps6586x_attr_group = {
	.attrs = tps6586x_attributes,
};
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111128 end*/

//[ECID:000000]ZTEBSP DangXiao 20120129 start,for thermal sensor wake lock
#ifdef CONFIG_HAS_EARLYSUSPEND
static int tps6586x_early_suspend(struct early_suspend *h)
{        
	wake_unlock(&temp_wake_lock);
        return 0;
}
static int tps6586x_late_resume(struct early_suspend *h)
{        
	return 0;
}
#endif
//[ECID:000000]ZTEBSP DangXiao 20120129 end,for thermal sensor wake lock

static int __devinit tps6586x_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct tps6586x_platform_data *pdata = client->dev.platform_data;
	struct tps6586x *tps6586x;
	int ret;
	//[ECID 000000] ZTEBSP luyanfei add for restart charging 20111229 ++
       int val;
       //[ECID 000000] ZTEBSP luyanfei add for restart charging 20111229 --
	if (!pdata) {
		dev_err(&client->dev, "tps6586x requires platform data\n");
		return -ENOTSUPP;
	}

	ret = i2c_smbus_read_byte_data(client, TPS6586X_VERSIONCRC);
	if (ret < 0) {
		dev_err(&client->dev, "Chip ID read failed: %d\n", ret);
		return -EIO;
	}

	dev_info(&client->dev, "VERSIONCRC is %02x\n", ret);

	tps6586x = kzalloc(sizeof(struct tps6586x), GFP_KERNEL);
	if (tps6586x == NULL)
		return -ENOMEM;

	tps6586x->client = client;
	tps6586x->dev = &client->dev;
	i2c_set_clientdata(client, tps6586x);

	mutex_init(&tps6586x->lock);

	if (client->irq) {
		ret = tps6586x_irq_init(tps6586x, client->irq,
					pdata->irq_base);
		if (ret) {
			dev_err(&client->dev, "IRQ init failed: %d\n", ret);
			goto err_irq_init;
		}
	}

	ret = tps6586x_gpio_init(tps6586x, pdata->gpio_base);
	if (ret) {
		dev_err(&client->dev, "GPIO registration failed: %d\n", ret);
		goto err_gpio_init;
	}

	ret = tps6586x_add_subdevs(tps6586x, pdata);
	if (ret) {
		dev_err(&client->dev, "add devices failed: %d\n", ret);
		goto err_add_devs;
	}

	tps6586x_i2c_client = client;


/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111128 start*/
	/* register sysfs hooks */
	ret = sysfs_create_group(&client->dev.kobj, &tps6586x_attr_group);
	if (ret) {
		dev_err(&client->dev, "add sysfs failed: %d\n", ret);
//		goto err_add_devs;
	}
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111128 end*/
	
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111118 start*/
	tps6586x->shutdown_temp = 109;  //85;
	tps6586x->temp_time = pdata->temp_time;
	tps6586x->temp_gpio = pdata->temp_gpio;
	tps6586x->temp_timer_delete = true;		
	tps6586x->debounce_time = 20;
//	tps6586x->temp_pmu_could_sleep = true;
	
	//printk("%s time %d gpio %d\n", __func__, tps6586x->temp_time, tps6586x->temp_gpio);

	wake_lock_init(&temp_wake_lock, WAKE_LOCK_SUSPEND, "tps6586x_temp");
	/*[ECID:000000] ZTEBSP khl, for usb-in suspend, 201112.30 start*/
	wake_lock_init(&usb_wake_lock, WAKE_LOCK_SUSPEND, "usb_in");
	/*[ECID:000000] ZTEBSP khl, for usb-in suspend, 201112.30 end*/
///	wake_lock_init(&temp_delayed_wake_lock, WAKE_LOCK_SUSPEND, "tps6586x_temp_delayed");	
	temp_wake_unlock = 0;	
	spin_lock_init(&tps6586x->temp_lock);
	mutex_init(&tps6586x->temp_mutex);
	INIT_WORK(&tps6586x->temp_work, tps6586x_get_temp_work_func);
	INIT_WORK(&tps6586x->force_normal_work, tps6586x_force_normal_work_func);
//	INIT_WORK(&tps6586x->delayed_work, tps6586x_delayed_work_func);

//[ECID:000000]ZTEBSP DangXiao 201200203 start,for thermal sensor early suspend,disable
#if 0//CONFIG_HAS_EARLYSUSPEND
	tps6586x->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	tps6586x->early_suspend.suspend = tps6586x_early_suspend;
	tps6586x->early_suspend.resume = tps6586x_late_resume;
	register_early_suspend(&tps6586x->early_suspend);
#endif
//[ECID:000000]ZTEBSP DangXiao 201200203 end,for thermal sensor early suspend

	if (tps6586x->temp_time) {
		int error,error1;
		int irq, gpio;
		unsigned long irqflags;
		int prev_val;
		unsigned long flags;

		init_timer(&tps6586x->temp_timer);
		tps6586x->temp_timer.data = (unsigned long)tps6586x;
		tps6586x->temp_timer.function = tps6586x_temp_timer;
		tps6586x->temp_timer.expires = jiffies + msecs_to_jiffies(tps6586x->temp_time);
//		add_timer(&tps6586x->resume_reset_timer);	

		init_timer(&tps6586x->gpio_debounce_timer);
		tps6586x->gpio_debounce_timer.data = (unsigned long)tps6586x;
		tps6586x->gpio_debounce_timer.function = tps6586x_debounce_timer;
		tps6586x->gpio_debounce_timer.expires = jiffies + msecs_to_jiffies(tps6586x->debounce_time);

/*		init_timer(&tps6586x->delayed_pmu_could_sleep_timer);
		tps6586x->delayed_pmu_could_sleep_timer.data = (unsigned long)tps6586x;
		tps6586x->delayed_pmu_could_sleep_timer.function = tps6586x_delayed_pmu_could_sleep_timer;
		tps6586x->delayed_pmu_could_sleep_timer.expires = jiffies + msecs_to_jiffies(5000);*/

		gpio = tps6586x->temp_gpio;
		tegra_gpio_enable(gpio);
	 	error1 = gpio_request(gpio, "tps65586x_rrt_gpio");
	 	if (error1 < 0) {
	 		dev_err(tps6586x->dev, "failed to request GPIO %d, error %d\n",
	 			gpio, error1);
	 	}
		error = gpio_direction_input(gpio);
		if (error < 0) {
			dev_err(tps6586x->dev, "failed to configure"
				" direction for GPIO %d, error %d\n",
				gpio, error);
		}
		if (error1 >=0)
			gpio_free(gpio);

		irq = gpio_to_irq(gpio);		
		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED;
		error = request_irq(irq, tps6586x_temp_isr, irqflags, "tps65586x_rrt_gpio", tps6586x);
		if (error) {
			dev_err(tps6586x->dev, "Unable to claim irq %d; error %d\n",
				irq, error);
			(void)tps6586x_force_normal(0);
			return 0;
		}		
		//printk("%s, have registered temp irq %d\n", __func__, irq);

		mutex_lock(&tps6586x->temp_mutex);

		if ((prev_val = gpio_get_value(gpio))) 
		{
		;
//			(void)tps6586x_force_normal(TRUE);
		}
		else
			(void)tps6586x_force_normal(0);


		//printk("%s, initial temp gpio %d %d\n", __func__, gpio, prev_val);
		mutex_unlock(&tps6586x->temp_mutex);

		spin_lock_irqsave(&tps6586x->temp_lock, flags);		
		prev_val = gpio_get_value(gpio);
//		printk("%s, gpio %d\n", __func__, prev_val);	

		if (!prev_val) {
//			printk("%s, start temp timer, time %d, gpio%d \n", __func__, tps6586x->temp_time, gpio);
//			mod_timer(&tps6586x->temp_timer, jiffies + msecs_to_jiffies(tps6586x->temp_time));
			tps6586x->temp_timer_delete = false;
		}
		spin_unlock_irqrestore(&tps6586x->temp_lock, flags);
	}
	else
		(void)tps6586x_force_normal(0);
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111118 end*/

/*[ECID:000000] ZTEBSP wangbing, for rtc out, 20111130*/
	tps6586x_set_rtc_out();	
	
//[ECID 000000] ZTEBSP luyanfei add for restart charging 20111229 ++
       val = zte_tps6586x_read(0xbb);
       if((val &0x04)==0x04)
       	{
       	//restart chging
       	charger_online =1;
              tps6586x_en_sm2();
	       tps686x_en_chg_norm();
       	}
//[ECID 000000] ZTEBSP luyanfei add for restart charging 20111229 --

//[ECID 000000] ZTEBSP shiyan add for lowbatt alarm 2012.2.4 ++
	//bit2=0  使能usb插拔的中断
	val = zte_tps6586x_read(0xb4);
	val &= ~0x4 ;
	val = zte_tps6586x_write(0xb4,val);

	//bit2=0  使能充电状态改变的中断
	val = zte_tps6586x_read(0xb3);
	val &= ~0x4;
	val = zte_tps6586x_write(0xb3,val);
//[ECID 000000] ZTEBSP shiyan add for lowbatt alarm 2012.2.4 --
	return 0;

err_add_devs:
	if (pdata->gpio_base) {
		ret = gpiochip_remove(&tps6586x->gpio);
		if (ret)
			dev_err(&client->dev, "Can't remove gpio chip: %d\n",
				ret);
	}
err_gpio_init:
	if (client->irq)
		free_irq(client->irq, tps6586x);
err_irq_init:
	kfree(tps6586x);
	return ret;
}

static int __devexit tps6586x_i2c_remove(struct i2c_client *client)
{
	struct tps6586x *tps6586x = i2c_get_clientdata(client);
	struct tps6586x_platform_data *pdata = client->dev.platform_data;
	int ret;

	if (client->irq)
		free_irq(client->irq, tps6586x);

	if (pdata->gpio_base) {
		ret = gpiochip_remove(&tps6586x->gpio);
		if (ret)
			dev_err(&client->dev, "Can't remove gpio chip: %d\n",
				ret);
	}

	tps6586x_remove_subdevs(tps6586x);
	kfree(tps6586x);
	return 0;
}

static const struct i2c_device_id tps6586x_id_table[] = {
	{ "tps6586x", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, tps6586x_id_table);

static struct i2c_driver tps6586x_driver = {
	.driver	= {
		.name	= "tps6586x",
		.owner	= THIS_MODULE,
	},
	.probe		= tps6586x_i2c_probe,
	.remove		= __devexit_p(tps6586x_i2c_remove),
	.id_table	= tps6586x_id_table,
};

static int __init tps6586x_init(void)
{
	return i2c_add_driver(&tps6586x_driver);
}
subsys_initcall(tps6586x_init);

static void __exit tps6586x_exit(void)
{
	i2c_del_driver(&tps6586x_driver);
}
module_exit(tps6586x_exit);

MODULE_DESCRIPTION("TPS6586X core driver");
MODULE_AUTHOR("Mike Rapoport <mike@compulab.co.il>");
MODULE_LICENSE("GPL");
