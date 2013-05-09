/*
 * drivers/power/tps6586x_battery.c
 *
 * Gas Gauge driver for TI's TPS6586X
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>

#include <mach/gpio.h>

//khl added 20120605
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include<linux/delay.h>  //[ECID:000000] ztebsp shiyan 2012.1.12 for delay 

enum {
	REG_MANUFACTURER_DATA,
	REG_TEMPERATURE,
	REG_VOLTAGE,
	REG_CURRENT,
	REG_TIME_TO_EMPTY,
	REG_TIME_TO_FULL,
	REG_STATUS,
	REG_CYCLE_COUNT,
	REG_CAPACITY,
	REG_SERIAL_NUMBER,
	REG_REMAINING_CAPACITY,
	REG_FULL_CHARGE_CAPACITY,
	REG_DESIGN_CAPACITY,
	REG_DESIGN_VOLTAGE,
	REG_MAX
};

#define BATTERY_MANUFACTURER_SIZE	12
#define BATTERY_NAME_SIZE		8

/* manufacturer access defines */
#define MANUFACTURER_ACCESS_STATUS	0x0006
#define MANUFACTURER_ACCESS_SLEEP	0x0011

/* battery status value bits */
#define BATTERY_INIT_DONE		0x80
#define BATTERY_DISCHARGING		0x40
#define BATTERY_FULL_CHARGED		0x20
#define BATTERY_FULL_DISCHARGED		0x10

//电池轮询时间30s
#define BATTERY_POLL_PERIOD		75000   //shiyan  from 30000

//shiyan added for chg forbit sleep
#define CHG_FORBIT_SLEEP  1

//shiyan added for max17040
#define USE_MAX17040  1

//shiyan added for final_cycle
#define FINAL_CHG_MIN	4   // 75s*80/60+150s*(20+4) /60=160min
//if batt<80%    75s*200/60=250min = 4.2h   
//else 75s*80/60 + 150*120/60 =100+300=400min =6.6h
#define CHG_TIMEOUT_MIN	200  

//[ECID:000000] ZTEBSP shiyan 20111120 start,  for charge
#define TPS_CHG1	0x49
#define TPS_CHG2	0x4A
#define TPS_CHG3	0x4B
#define TPS_ADC_INT		0x9A
#define TPS_STAT1	0xB9
#define TPS_STAT2	0xBA
#define TPS_STAT3	0xBB
#define TPS_STAT4	0xBC

#define CHG1_TSON  	(1<<4)
#define ADC_DONE  	(1<<7)
//充电电流小于截止电流
#define STAT2_ITERM  	(1<<4)  
#define STAT2_CHGSTAT  	0x06
#define STAT3_USBDET  	(1<<2)
#define STAT4_ADC0BUSY  	(1<<4)
//[ECID:000000] ZTEBSP shiyan  20111120 end, for charge

//ec 000000 luyanfei modify for battery cap display 20111216 ++
int last_battery_capacity = 1000;
bool tps6586x_bat_probe = false;
bool battery_timer_on = true;
bool tps6586x_suspend_flag = false;   //shiyan added for suspend flag 20120802

#ifdef USE_MAX17040
extern int max_17040_probled(void) ;
extern int zte_max17040_get_vcell(void);
extern int zte_max17040_get_soc(void);
#endif
//ec 000000 luyanfei modify for battery cap display 20111216 --

#define TPS6586X_DATA(_psp, _addr, _min_value, _max_value)	\
	{							\
		.psp = POWER_SUPPLY_PROP_##_psp,		\
		.addr = _addr,					\
		.min_value = _min_value,			\
		.max_value = _max_value,			\
	}

static struct tps6586x_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} tps6586x_data[] = {
	[REG_MANUFACTURER_DATA] = TPS6586X_DATA(PRESENT, 0x00, 0, 65535),
	[REG_TEMPERATURE]       = TPS6586X_DATA(TEMP, 0x08, 0, 65535),
	[REG_VOLTAGE]           = TPS6586X_DATA(VOLTAGE_NOW, 0x09, 0, 20000),
	[REG_CURRENT]           = TPS6586X_DATA(CURRENT_NOW, 0x0A, -32768, 32767),
	[REG_CAPACITY]          = TPS6586X_DATA(CAPACITY, 0x0e, 0, 100),
	[REG_REMAINING_CAPACITY] = TPS6586X_DATA(ENERGY_NOW, 0x0F, 0, 65535),
	[REG_FULL_CHARGE_CAPACITY] = TPS6586X_DATA(ENERGY_FULL, 0x10, 0, 65535),
	[REG_TIME_TO_EMPTY]     = TPS6586X_DATA(TIME_TO_EMPTY_AVG, 0x12, 0, 65535),
	[REG_TIME_TO_FULL]      = TPS6586X_DATA(TIME_TO_FULL_AVG, 0x13, 0, 65535),
	[REG_STATUS]            = TPS6586X_DATA(STATUS, 0x16, 0, 65535),
	[REG_CYCLE_COUNT]       = TPS6586X_DATA(CYCLE_COUNT, 0x17, 0, 65535),
	[REG_DESIGN_CAPACITY]   = TPS6586X_DATA(ENERGY_FULL_DESIGN, 0x18, 0, 65535),
	[REG_DESIGN_VOLTAGE]    = TPS6586X_DATA(VOLTAGE_MAX_DESIGN, 0x19, 0, 65535),
	[REG_SERIAL_NUMBER]     = TPS6586X_DATA(SERIAL_NUMBER, 0x1C, 0, 65535),
};

static enum power_supply_property tps6586x_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,				//是否充电
	POWER_SUPPLY_PROP_HEALTH,				//健康
	POWER_SUPPLY_PROP_PRESENT,				//是否在
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,  		
	POWER_SUPPLY_PROP_CURRENT_NOW,		
	POWER_SUPPLY_PROP_CAPACITY,			//容量
	POWER_SUPPLY_PROP_TEMP,				
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
};

static enum power_supply_property tps6586x_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

//[ECID:000000] ZTEBSP shiyan 20111018 start, for charging
typedef struct
{
    int index;    //index
    char temp;  //batt temp
    int res;        //ntc resistor
    int sample_vol;  //ntc  voltage  mv
} T_BATT_TEMP_DATA;

static const T_BATT_TEMP_DATA  batt_temp_volt[18] = 
/*********************************
*                        Rntc
*sample_vol = ------------- X 2.2(V)
*                      82K+Rntc
**********************************/
/*index, temp, res, sample_voltage*/
//实际值
{
/*上拉电阻为82K*/
   { 0, -40,  1748, 2125 },  //1747.9
   { 1, -30,   898,  2016 },  //898.5
   { 2, -20,   484,  1881 },  //483.9
   { 3, -10,   272,  1690 },  //271.7
   { 4,    0,   158,   1448 }, //158.2
   
   { 5,   10,    95,   1181 }, //95.2kΩ
   { 6,   15,    75,   1051 }, //74.7kΩ
   { 7,   20,    59,   892 },  //921
   { 8,   25,    47,   767 },  //802
   { 9,   30,    38,   674},   //697
   { 10,  35,    30,   563 },  //589
   { 11,  40,    25,   488 },  //514 
   { 12,  45,    20,   411 },  //431 
   { 13,  50,    16,    359 }, //16.4kΩ
   
   { 14, 60,    11,    260 }, //11.2
   { 15, 70,     8,     196 }, //7.8
   { 16, 80,     6,     150 }, //5.5
   { 17, 100,   3,     78 }  // 2.9
};


static int battery_cap_table[][2] = {{4200, 100}, {4190, 99}, {4100, 98}, {4090, 97}, {4080, 96},
				                  {4075, 95}, {4073, 94}, {4071, 93}, {4069, 92}, {4067, 91},						   
				                  {4065, 90}, {4056, 89}, {4047, 88}, {4038, 87}, {4029, 86},	
				                  {4020, 85}, {4012, 84}, {4004, 83}, {3996, 82}, {3988, 81},	
				                  {3980, 80}, {3973, 79}, {3967, 78}, {3960, 77}, {3953, 76},	
				                  {3945, 75}, {3937, 74}, {3930, 73}, {3923, 72}, {3915, 71},	
				                  {3907, 70}, {3900, 69}, {3892, 68}, {3884, 67}, {3875, 66},		
				                  {3866, 65}, {3858, 64}, {3850, 63}, {3842, 62}, {3834, 61},		
				                  {3825, 60}, {3820, 59}, {3816, 58}, {3812, 57}, {3807, 56},		
				                  {3802, 55}, {3798, 54}, {3794, 53}, {3790, 52}, {3785, 51},		
				                  {3780, 50}, {3777, 49}, {3774, 48}, {3771, 47}, {3768, 46},		
				                  {3766, 45}, {3764, 44}, {3762, 43}, {3759, 42}, {3755, 41},		
				                  {3753, 40}, {3750, 39}, {3747, 38}, {3745, 37}, {3743, 36},		
				                  {3741, 35}, {3739, 34}, {3737, 33}, {3735, 32}, {3733, 31},		
				                  {3730, 30}, {3727, 29}, {3724, 28}, {3721, 27}, {3718, 26},		
				                  {3715, 25}, {3712, 24}, {3709, 23}, {3706, 22}, {3703, 21},			
				                  {3700, 20}, {3694, 19}, {3688, 18}, {3682, 17}, {3676, 16},			
				                  {3670, 15}, {3663, 14}, {3658, 13}, {3651, 12}, {3644, 11},		
				                  {3636, 10}, {3625, 9},   {3615, 8},  {3605, 7},   {3580, 6},			
				                  {3560, 5},   {3530, 4},   {3500, 3},  {3440, 2},   {3300, 1},{3260, 0}};

//[ECID:000000] ZTEBSP shiyan 20111018 end, for charging

static char *power_supplied_to[] = {
	"battery",
};

static int tps6586x_bat_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
static int tps6586x_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

enum supply_type {
	SUPPLY_TYPE_BATTERY = 0,
	SUPPLY_TYPE_AC,
};

static struct power_supply tps6586x_supply[] = {
	[SUPPLY_TYPE_BATTERY] = {
		.name		= "battery",
		.type		= POWER_SUPPLY_TYPE_BATTERY,
		.properties	= tps6586x_battery_properties,
		.num_properties	= ARRAY_SIZE(tps6586x_battery_properties),
		//更新电池状态
		.get_property	= tps6586x_bat_get_property,
	},
	[SUPPLY_TYPE_AC] = {
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = power_supplied_to,
		.num_supplicants = ARRAY_SIZE(power_supplied_to),
		.properties = tps6586x_ac_properties,
		.num_properties = ARRAY_SIZE(tps6586x_ac_properties),
		//更新ac状态
		.get_property = tps6586x_ac_get_property,
	},
};

static struct tps6586x_device_info {
	struct timer_list	battery_poll_timer;
	struct i2c_client	*client;
	int irq;
	bool battery_present;
	//khl
	#ifdef CONFIG_HAS_EARLYSUSPEND
       struct early_suspend early_suspend_handler;
    #endif
} *tps6586x_device;

//[ECID:000000] ZTEBSP shiyan 20111018 start, modify for batt info 
static s32  last_batt_volt = 3801;
static s32  batt_temp = 26;   //default is 26 C
static s32  batt_temp_adc = 600;   //batt ID default is 600mV
static s32  charge_current = 0;  
static s32  current_reduce_flag = 0;  
static s32  charger_status_need_get =1; 
static s32  CHARGER_STATUS =2;   //POWER_SUPPLY_STATUS_DISCHARGING
static s32  batt_volt_need_adc =1; 
static s32  batt_temp_need_adc =1;
static s32  batt_cap_need_cal =1;
static s32  ac_status_need_get =1;
static int  batt_has_charged_full =0;
static int charger_on = 0;

extern bool in_call_state(void); 

extern int zte_tps6586x_read(int reg);
extern  int zte_tps6586x_write(int reg,int val);
extern int zte_charger_on(void);
extern int read_charger_on_flag(void);
extern int read_usb_flag(void);

#ifdef CHG_FORBIT_SLEEP
/*[ECID:000000] ZTEBSP shiyan, for usb-in suspend, 201112.30 start*/
static struct wake_lock charger_wake_lock;
static struct wake_lock adc_wake_lock;
static s32  charger_wake_lock_flag =0;
/*[ECID:000000] ZTEBSP shiyan, for usb-in suspend, 201112.30 end*/
#endif

static int tps6586x_get_ac_status(void)
{
 	int val = 0;

	if(!ac_status_need_get)
	{
		return charger_on;
	}
	
	else
	{
		ac_status_need_get = 0;
		
		charger_on = zte_charger_on();
		
		if(charger_on==1)
		{
		#ifdef CHG_FORBIT_SLEEP
			if(2==charger_wake_lock_flag)
			{
				if(!read_usb_flag())
				{
					#ifdef BATTERY_DEBUG
					printk("******shiyan : tps6586x_get_ac_status reset current =2.1A****** \n");	
					#endif
					//这里会导致状态的中断而死机
					val = zte_tps6586x_read(0xb3);
					zte_tps6586x_write(0xb3, val|0x04);    //0x04
					
					//停止充电 bit1=0
					val = zte_tps6586x_read(0x4a);
					val &= ~0x02;  
					val = zte_tps6586x_write(0x4a, val);

					//bit1=1  设置 最大限制电流为2.1A
					val = zte_tps6586x_read(0x4c);
					val |= 0x02;
					val = zte_tps6586x_write(0x4c,val);

					//使能充电 bit1=1
					val = zte_tps6586x_read(0x4a);
					val |= 0x02;  
					val = zte_tps6586x_write(0x4a,val);

					mdelay(50);

				}
				else
				{
					#ifdef BATTERY_DEBUG
					printk("******shiyan : tps6586x_get_ac_status  charger current =500mA****** \n");	
					#endif
				}
				
				//wake_unlock(&charger_wake_lock);
				//printk("shiyan: charger_wake_lock can unlock.\n");
				charger_wake_lock_flag =3;
			}
			
			else if(1 == charger_wake_lock_flag )
			{
				charger_wake_lock_flag =2;
			}
			//第一次加锁
			else if(0 == charger_wake_lock_flag )
			{
				wake_lock(&charger_wake_lock);
				#ifdef BATTERY_DEBUG
				printk("shiyan: charger_wake_lock lock.\n");
				#endif
				charger_wake_lock_flag =1;
			}
		#endif
		}
		
		else
		{
		#ifdef CHG_FORBIT_SLEEP
			if(charger_wake_lock_flag != 0 )
			{
				wake_unlock(&charger_wake_lock);
				wake_lock_timeout(&charger_wake_lock, 3000); //shiyan HZ/2
				#ifdef BATTERY_DEBUG
				printk("shiyan: charger_wake_lock unlock.\n");
				#endif
			}
			charger_wake_lock_flag =0;
		#endif
		}
		
		return charger_on;
	}
}

static int tps6586x_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	switch (psp) 
	{
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = tps6586x_get_ac_status();
		break;
	default:
		dev_err(&tps6586x_device->client->dev,
			"%s: INVALID property\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int tps6586x_get_battery_presence_and_health(
	struct i2c_client *client, enum power_supply_property psp,
	union power_supply_propval *val)
{
//	s32 ret;

#if 0   //shiyan 
	/* Write to ManufacturerAccess with
	 * ManufacturerAccess command and then
	 * read the status */
	ret = i2c_smbus_write_word_data(client,
		tps6586x_data[REG_MANUFACTURER_DATA].addr,
		MANUFACTURER_ACCESS_STATUS);
	if (ret < 0)
		return ret;


	ret = i2c_smbus_read_word_data(client,
		tps6586x_data[REG_MANUFACTURER_DATA].addr);
	if (ret < 0)
		return ret;

	if (ret < tps6586x_data[REG_MANUFACTURER_DATA].min_value ||
	    ret > tps6586x_data[REG_MANUFACTURER_DATA].max_value) 
	{
		val->intval = 0;
		return 0;
	}

	/* Mask the upper nibble of 2nd byte and
	 * lower byte of response then
	 * shift the result by 8 to get status*/
	ret &= 0x0F00;
	ret >>= 8;
#endif   //shiyan 

	if (psp == POWER_SUPPLY_PROP_PRESENT) 
	{
//		if (ret == 0x0F)
			/* battery removed */
//			val->intval = 0;
//		else
			val->intval = 1;
	} 
	else if (psp == POWER_SUPPLY_PROP_HEALTH) 
	{
#if 0  //shiyan
		if (ret == 0x09)
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else if (ret == 0x0B)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (ret == 0x0C)
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
		else
#endif
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
	}

	return 0;
}

static s32 tps6586x_adc(int chaanel)
{
	s32 ret =0 ;
	s32 adc_val = 0;
	s32 timeout = 0;

	//shiyan added
	//wake_lock_timeout(&adc_wake_lock, 5*HZ);
	if(tps6586x_suspend_flag ==false)
	{
		wake_lock(&adc_wake_lock);
		
		ret = zte_tps6586x_write(0x62,0x80);
		ret = zte_tps6586x_write(0x61, chaanel);
		ret = zte_tps6586x_write(0x62,0x21);  
		adc_val = zte_tps6586x_read(0x61);
		adc_val|=0x80;
		ret = zte_tps6586x_write(0x61,adc_val);    
		mdelay(30);
		timeout = 0;
		while(1)
		{
		       adc_val = zte_tps6586x_read(TPS_ADC_INT);
		        if (adc_val & 0x80)
		            break;
		        if (adc_val & 0x40)
		        {
		            printk("tps6586x_adc: ADC conversion error.\n");
				//shiyan added for adc  20120718
				wake_unlock(&adc_wake_lock);
		            return (-1);
		        }
		        udelay(70);
		        timeout += 70;
		        if (timeout >= 400)   //shiyan modify  20120802
		        {
				printk("tps6586x_adc:ADC conversion timeout.\n");
				//shiyan added for adc  20120718
				wake_unlock(&adc_wake_lock);
				return (-1);
		        }
		}
		adc_val = zte_tps6586x_read(0x94);  //high 8 bit
		adc_val = adc_val<<8;
		adc_val += zte_tps6586x_read(0x95);  //low 8 bit

		ret = zte_tps6586x_write(0x62, 0);    

		//shiyan added
		wake_unlock(&adc_wake_lock);
				
		return adc_val; 
	}
	else
	{
		return (-1);
	}
}
static int tps6586x_get_battery_property(struct i2c_client *client, int reg_offset,
	enum power_supply_property psp, union power_supply_propval *val)
{
//shiyan 
	s32 ret =99 ;
	s32 adc_val = 0;
	s32 num = 0 ;
	static int sample_num = 0;
	s32 timeout = 0;
	int batt_voltage[5] = {0,0,0,0,0};

	//获取充电器状态
	if (psp == POWER_SUPPLY_PROP_STATUS) 
	{
		//if last had get the status, no need to read i2c
		if(!charger_status_need_get)
		{
			val->intval = CHARGER_STATUS ;
		}
		else
		{
			//if (tps6586x_get_ac_status())   
			if (zte_charger_on())  
			{
				val->intval = POWER_SUPPLY_STATUS_CHARGING ;
	                    
			#if 0
				adc_val = zte_tps6586x_read(0xba);
		
				//note:有时候没充满时，读到的寄存器值也是0x02，
				//因此加入电压先规避一下
				//充电超时或完成
				if(((adc_val & 0x10)||((adc_val & 0x06)==0x02))&&(last_batt_volt > 4120))
			#endif
			
				if(batt_has_charged_full)
				{
					val->intval = POWER_SUPPLY_STATUS_FULL ;
				}
			}
			else
			{
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING ;	
			}
			CHARGER_STATUS = val->intval ;
			charger_status_need_get = 0;
		}
	}
	
	//获取电池电压
	else if(psp == POWER_SUPPLY_PROP_VOLTAGE_NOW)
	{
		val->intval = last_batt_volt ;
		
		//if adc had read sucessfully, no need to use adc
		if(!batt_volt_need_adc)
		{
			return 0;
		}
		else
		{
			batt_volt_need_adc =0;
			
		#ifdef USE_MAX17040    //has defined
			if(max_17040_probled())
			{
				adc_val = zte_max17040_get_vcell();
				#ifdef BATTERY_DEBUG
				printk("shiyan: zte_max17040_get_vcell  =%d mv \n", adc_val );	
				#endif
				if((adc_val >4300) ||(adc_val <2800))
				{
					adc_val = tps6586x_adc(0x19); 
					adc_val = (adc_val*4622)/ 1023 / 16; 
				}
				if((adc_val >4300) ||(adc_val <2800))
				{
					adc_val =4150;
				}
			}
			else
			{
				for(num=0;num<5;num++)
				{
					adc_val = tps6586x_adc(0x19); 
					adc_val = (adc_val*4622)/ 1023 / 16; 
					batt_voltage[num] = adc_val;
					mdelay(100);
				}
				//取5次中的最大值
				//取平均值 还是有 问题 
				adc_val= batt_voltage[0] ;
				for(num =1; num<5; num++)
				{
					adc_val = (adc_val >= batt_voltage[num] )?adc_val : batt_voltage[num] ;
				}
				#ifdef BATTERY_DEBUG
				printk("shiyan: adc batt_voltage max =%d mv \n", adc_val );	
				#endif
			}
		#endif
				if((adc_val < 4300)&&(adc_val > 2800))
				{
					val->intval = adc_val;
					last_batt_volt = adc_val ;
				}
				else
				{
					batt_volt_need_adc =1;
				}
		}
	}
	
	//获取电池温度
	else if(psp == POWER_SUPPLY_PROP_TEMP)
	{
		val->intval = batt_temp ;
		
		//if adc had read sucessfully, no need to use adc
		if(!batt_temp_need_adc)
		{
			return 0;
		}
		else
		{
			//如果充电器不在,需要打开TSbias
			if(!zte_charger_on())
			{
				adc_val = zte_tps6586x_read(TPS_CHG1);
				adc_val|= CHG1_TSON;
				ret = zte_tps6586x_write(TPS_CHG1,adc_val);    
				//增加延时，修改拔充电器瞬间采样错误的问题		
				mdelay(100);
			}
		
			adc_val = tps6586x_adc(0x14); 
			
			if(!zte_charger_on())
			{
				num = zte_tps6586x_read(TPS_CHG1);
				num &= ~CHG1_TSON;
				ret = zte_tps6586x_write(TPS_CHG1, num);    

				adc_val = (adc_val*2600)/ 1023 / 16;    	//voltage   mV     adc_count = x/2600*1023
			}
			else
			{
				// 1.06校正
				adc_val = (adc_val*2600)/ 1023 / 16 *100 /106;    	
			}
			#ifdef BATTERY_DEBUG
			printk("*****shiyan: temp_sample  =%d  mV \n", adc_val );	
			#endif
			
			//有时候采到错误的值2234和0
			if((adc_val > 100)&&(adc_val < 2100))
			{
				adc_val = (batt_temp_adc*3+adc_val)/4 ;
				batt_temp_adc = adc_val;
				#ifdef BATTERY_DEBUG
				printk("*****shiyan: avg temp_sample  =%d mV \n", batt_temp_adc );	
				#endif
				
				//校正补偿
				//adc_val+=60;
				for(num=0; num<17,adc_val < batt_temp_volt[num].sample_vol;  )
				{ num++;}
			   
				if(num==0)
				val->intval = 26;
				else   if(num==17)
				val->intval = 26;
				else
				val->intval = batt_temp_volt[num-1].temp+ \
				( batt_temp_volt[num].temp - batt_temp_volt[num-1].temp) *( batt_temp_volt[num-1].sample_vol - adc_val) \
				/(batt_temp_volt[num-1].sample_vol - batt_temp_volt[num].sample_vol) ;

				//采样值保存到临时值		
				batt_temp = val->intval  ;   

				#ifdef BATTERY_DEBUG
				printk("*****shiyan: avg batt_temp  =%d C \n", val->intval );	
				#endif

				batt_temp_need_adc = 0;
			}
	  #if 0	
			//[ECID:000000] ZTEBSP shiyan 20111018 start,  for charge temperature too high
			// 40? need to modify to fit up-resisitor
			//if((batt_temp>40)&&(zte_charger_on())&&(0==current_reduce_flag))
			if((batt_temp_adc < 502)&&(zte_charger_on())&&(0 == current_reduce_flag))
			{
				//停止充电 bit1=0
				adc_val = zte_tps6586x_read(0x4a);
				adc_val &= ~0x02;  
				adc_val = zte_tps6586x_write(0x4a, adc_val);

				//降低恒流充电的电流
				//bit3:2 =00:215mA,   01 :432mA  ,  10:  648mA   11 :864mA  
				adc_val = zte_tps6586x_read(0x49);
				charge_current = adc_val & 0x0c;
				adc_val &= ~0x0c;
				adc_val |= 0x0;
				ret = zte_tps6586x_write(0x49, adc_val);	
				current_reduce_flag =1;

				//使能充电 bit1=1
				adc_val = zte_tps6586x_read(0x4a);
				adc_val |= 0x02;  
				adc_val = zte_tps6586x_write(0x4a,adc_val);
				
				#ifdef BATTERY_DEBUG
				printk("shiyan :battery temp over 40C ! \n");	
				#endif
			}
			// 38? need to modify to fit up-resisitor
			//if((batt_temp<38)&&(zte_charger_on())&&(0==current_reduce_flag))
			if((batt_temp_adc > 540)&&(zte_charger_on())&&(1 == current_reduce_flag))
			{
				//停止充电 bit1=0
				adc_val = zte_tps6586x_read(0x4a);
				adc_val &= ~0x02;  
				adc_val = zte_tps6586x_write(0x4a, adc_val);
			
				//恢复恒流充电的电流
				//bit3:2 =00:215mA,   01 :432mA  ,  10:  648mA   11 :864mA  
				adc_val = zte_tps6586x_read(0x49);
				adc_val &= ~0x0c;
				adc_val |= charge_current;
				ret = zte_tps6586x_write(0x49, adc_val);	
				current_reduce_flag =0;
				
				//使能充电 bit1=1
				adc_val = zte_tps6586x_read(0x4a);
				adc_val |= 0x02;  
				adc_val = zte_tps6586x_write(0x4a,adc_val);
				
				#ifdef BATTERY_DEBUG
				printk("shiyan :battery temp low 38C ! \n");	
				#endif
			}
			//[ECID:000000] ZTEBSP shiyan 20111018 end,  for charge temperature too high
	#endif	
		}
	}
	else   
	{
		val->intval = ret;
	}
	
	return 0;
}

static void  tps6586x_unit_adjustment(struct i2c_client *client,
	enum power_supply_property psp, union power_supply_propval *val)
{
#define BASE_UNIT_CONVERSION		1000
#define BATTERY_MODE_CAP_MULT_WATT	(10 * BASE_UNIT_CONVERSION)
#define TIME_UNIT_CONVERSION		600
#define TEMP_KELVIN_TO_CELCIUS		2731
	switch (psp) {
	case POWER_SUPPLY_PROP_ENERGY_NOW:
	case POWER_SUPPLY_PROP_ENERGY_FULL:
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		val->intval *= BATTERY_MODE_CAP_MULT_WATT;    // *10*1000
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval *= BASE_UNIT_CONVERSION;   		//*1000
		break;

	case POWER_SUPPLY_PROP_TEMP:
		/* tps6586x provides battery tempreture in 0.1K
		 * so convert it to 0.1C */
	//	val->intval -= TEMP_KELVIN_TO_CELCIUS;		//-2731
		val->intval *=10;
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		val->intval *= TIME_UNIT_CONVERSION;			//*600
		break;

	default:
		dev_dbg(&client->dev,
			"%s: no need for unit conversion %d\n", __func__, psp);
	}
}

static int tps6586x_get_battery_capacity(struct i2c_client *client,
	int reg_offset, enum power_supply_property psp,
	union power_supply_propval *val)
{
 	int value = 0;
	int rel = 0;
	s32 num = 0 ;
	static long  chg_time =0;
	static long  chg_final_num =0;
	static int first_check_c2 =1; 
	#ifdef USE_MAX17040
	static int max17040_first =1;
	#endif
	//about rtc time 
	unsigned long  rtc_data_temp[5] ={0,0,0,0,0};
	long  rtc_time =0;
	static unsigned long  last_rtc_time = 0;
	long  gap_time =0;


	if(!batt_cap_need_cal)
	{
		val->intval  = last_battery_capacity  ; 
	}
	else
	{
		batt_cap_need_cal = 0;

		#ifdef BATTERY_DEBUG
		printk("*****shiyan : param  last_batt_volt =%d \n",  last_batt_volt);
		#endif

		//每次采样都需要检测参数是否正常			
		if((last_batt_volt <= 2800)||(last_batt_volt >=4300))
		{
			if(first_check_c2 == 0)
			{
				val->intval = last_battery_capacity;
				return 0;
			}
			else
			{
				//如果是第一次的参数异常，在下面处理
			}
		}

		//修改关机充满，再开机又显示不满的问题
		if(first_check_c2 == 1)
		{
			value = zte_tps6586x_read(0xC2);
			//保持bit5-4的状态
			rel = zte_tps6586x_write(0xC2, value&0x30);  
			#ifdef BATTERY_DEBUG
			printk("shiyan : first time read 0xc2 = 0x%x \n", value);
			#endif
			
			//bit5-4  充电完成标志
			if((value & 0x30)==0x30)
			{
				if(last_batt_volt > 4060)
				{
					val->intval = 100 ;	
					batt_has_charged_full = 1;
					
					first_check_c2 =0;
					return 0;
				}
				else
				{
					rel = zte_tps6586x_write(0xC2, 0);  
				}
			}
		}
					
		if(first_check_c2 == 0)
		{
			rtc_data_temp[0] = zte_tps6586x_read(0xC6);
			rtc_data_temp[1] = zte_tps6586x_read(0xC7);
			rtc_data_temp[2] = zte_tps6586x_read(0xC8);
			rtc_data_temp[3] = zte_tps6586x_read(0xC9);
			rtc_data_temp[4] = zte_tps6586x_read(0xCA);
			rtc_data_temp[4] = zte_tps6586x_read(0xC0);
			
			rtc_time =(((rtc_data_temp[0] <<24)|(rtc_data_temp[1] <<16)|(rtc_data_temp[2] <<8)|(rtc_data_temp[3]))>>2) & 0xFFFF;
			#ifdef BATTERY_DEBUG
			printk("******shiyan : rtc time:  0x%x sec \n", rtc_time);	
			#endif
			
			if(last_rtc_time ==0)
			{
				last_rtc_time = rtc_time;
			}
			//72s /60*100 =120min  
			gap_time = (rtc_time - last_rtc_time)/72;
			if(gap_time <= 0)
			{
				gap_time =0;
			}
			if(gap_time >=100)
			{
				gap_time =100;
			}
			last_rtc_time = rtc_time; 
			#ifdef BATTERY_DEBUG
			printk("******shiyan : gap  time:  %d min  \n", gap_time);
			#endif
		}
					
		if (zte_charger_on()||((first_check_c2 == 1)&&((value & 0x08)==0x08)))
		{
			if(chg_time <CHG_TIMEOUT_MIN)
			 	chg_time++;

			if(first_check_c2 == 1)
			{
				num = zte_tps6586x_read(0xC3);
				num = num*(4300-3000)/255+3000;
				#ifdef BATTERY_DEBUG
				printk("shiyan: first time read batt from 0xc3 =%d mv \n",  num);
				#endif
				
				if((num >= 3100)&&(num <=4300))
				{
					last_batt_volt = num;
				}
				else
				{
				//如果电压不在正常范围内时，last_batt_volt不变
				}
			
				//如果last_batt_volt仍然不在正常范围内，设为3400
				if((last_batt_volt < 3200)||(last_batt_volt >4300))
				{
					last_batt_volt = 3400;
				}
				
			}
	
			//二次充电不上报
			if(batt_has_charged_full)
			{
				val->intval = 100 ;
				return 0;
			}
			else
			{
				if(((chg_time >= CHG_TIMEOUT_MIN)&&(last_batt_volt > 4000)) \
					||((chg_final_num >= FINAL_CHG_MIN)&&(last_batt_volt > 4120)))
				{
					val->intval = 100 ;
					batt_has_charged_full = 1;
					rel = zte_tps6586x_write(0xC2, 0x30);  
					
					//打印充电完成原因
					#ifdef BATTERY_DEBUG
					if(chg_final_num >= FINAL_CHG_MIN)
						printk("shiyan:  charging complete ! \n");
					else
						printk("shiyan:  timeout 5h ! \n");
					#endif
				}
				else
				{
					for(num=0; num<100; num++)
					{
						if(last_batt_volt > battery_cap_table[num][0])
						break;
					}
					#ifdef BATTERY_DEBUG
					printk("shiyan: charging  battery_cap_table  num =%d \n", num);
					#endif
					val->intval = battery_cap_table[num][1];
					
					#ifdef USE_MAX17040
					if(max_17040_probled())
					{
						val->intval = zte_max17040_get_soc();
						#ifdef BATTERY_DEBUG
						printk("shiyan: max17040 capatity = %d \n", val->intval);
						#endif
						
						//如果max17040读取的不对，还用数组
						if((val->intval < 1)||(val->intval >105))
						{
							mdelay(100);
							val->intval = zte_max17040_get_soc();
							if((val->intval < 0)||(val->intval >105))
							{
								val->intval = battery_cap_table[num][1];
							}
							else if(val->intval ==0)
							{
								if((last_batt_volt > 3300)&&(last_batt_volt < 3750))
								val->intval =1;
								else
								//避免电量计上报错误的0%
								val->intval = battery_cap_table[num][1];
							}
							else
							{
								if(max17040_first)
								{
									max17040_first =0;
									first_check_c2 = 0;
									if(val->intval >=100)
										val->intval =100;

									return 0;
								}
							}
						}
						else
						{
							if(max17040_first)
							{
								max17040_first =0;
								first_check_c2 = 0;
								if(val->intval >=100)
									val->intval =100;

								return 0;
							}
						}
					}
					#endif
					
					if((val->intval <= last_battery_capacity)&&(last_battery_capacity != 1000))
					{
						val->intval = last_battery_capacity;
					}

					if((val->intval > last_battery_capacity)&&(last_battery_capacity != 1000))
					{
						val->intval = last_battery_capacity +1;
					}
								
					//100%电压改为4190V,   否则可能到不了4200  而一直是99%
					if (val->intval >= 99)
					{
						val->intval = 99;
						//在这里延时的好处:
						//避免电压到达4200    百分比还是80%的情况
						if(chg_final_num < FINAL_CHG_MIN)
						{
							chg_final_num++;
						}
					}
					
					if (val->intval <= 0)
					{
						val->intval = 0;
					}
					#ifdef BATTERY_DEBUG
					printk("shiyan: chg  battery_cap  is %d %% \n",val->intval);
					#endif
				}
			}
		}
		
		else
		{
			chg_final_num =0;
			chg_time = 0;
			
			if(first_check_c2 == 1)
			{
				num = zte_tps6586x_read(0xC3);
				num = num*(4300-3000)/255+3000;
				#ifdef BATTERY_DEBUG
				printk("shiyan: first time read batt from 0xc3 =%d mv \n",  num);
				#endif
				
				if((num >= 3100)&&(num <=4300))
				{
					last_batt_volt = num;
				}
				else
				{
				//如果电压不在正常范围内时，last_batt_volt不变
				}
				
				//如果last_batt_volt仍然不在正常范围内，设为3400
				if((last_batt_volt < 3200)||(last_batt_volt >4300))
				{
					last_batt_volt = 3400;
				}
			}
		
			
			if(((max_17040_probled())&&(last_batt_volt < 4120)) \
				||((!max_17040_probled())&&(last_batt_volt < 4080)))
			{
				if(batt_has_charged_full==1)
				{
					batt_has_charged_full = 0;
					rel = zte_tps6586x_write(0xC2, 0);  
				}
			}
			
			if(0 == batt_has_charged_full )
			{
				if(in_call_state())
				{
					last_batt_volt +=30;
				}

				for(num=0; num<100; num++)
				{
					if(last_batt_volt > battery_cap_table[num][0])
					break;
				}
				//用这样的比较，实际关机电压为battery_cap_table[99][0]
				#ifdef BATTERY_DEBUG
				printk("shiyan: not chg battery_cap_table num =%d  \n", num);
				#endif
				val->intval = battery_cap_table[num][1];
				
				#ifdef USE_MAX17040
				if(max_17040_probled())
				{
					val->intval = zte_max17040_get_soc();
					#ifdef BATTERY_DEBUG
					printk("shiyan: max17040 capatity = %d \n", val->intval);
					#endif
					
					//如果max17040读取的不对，还用数组
					if((val->intval < 1)||(val->intval >106))
					{
						mdelay(100);
						val->intval = zte_max17040_get_soc();
						if((val->intval < 0)||(val->intval >106))
						{
							val->intval = battery_cap_table[num][1];
						}
						else if(val->intval == 0)
						{
							if((last_batt_volt > 3300)&&(last_batt_volt < 3600))
							val->intval =1;
							else
							//避免电量计上报错误的0%
							val->intval = battery_cap_table[num][1];
						}
						else
						{
							if(max17040_first)
							{
								max17040_first =0;
								first_check_c2 = 0;
								if(val->intval >=100)
									val->intval =100;
								return 0;
							}
						}
					}
					else
					{
						if(max17040_first)
						{
							max17040_first =0;
							first_check_c2 = 0;
							if(val->intval >=100)
								val->intval =100;
							return 0;
						}
					}
				}
				#endif

				if((val->intval < last_battery_capacity)&&(last_battery_capacity != 1000))
				{
					if((last_battery_capacity - val->intval) ==1)
					{
						 val->intval =last_battery_capacity -1;
					}
					else if((last_battery_capacity - val->intval) >=2)
					{
						if(gap_time <=1)
						{
							//如果电量小于15%，允许下降2%
							if(last_battery_capacity<=10)
								val->intval = last_battery_capacity -2;
							else
								val->intval = last_battery_capacity -1;	
						}
						else
						{
							num = (last_battery_capacity - val->intval)<(gap_time/2) ? (last_battery_capacity - val->intval):(gap_time/2);
							val->intval = last_battery_capacity - num;
						}
					}
					#ifdef BATTERY_DEBUG
					printk("shiyan: battery down num =%d \n",val->intval);
					#endif
				}
			
				//电池电量不能上升
				if((val->intval >= last_battery_capacity)&&(last_battery_capacity != 1000))
				{
					val->intval = last_battery_capacity ;
				}

				if (val->intval >= 100)
				{
					val->intval = 100;
				}
				else if (val->intval <= 0)
				{
					val->intval = 0;
				}
				
				#ifdef BATTERY_DEBUG
				printk("shiyan: not chg battery_cap  is %d %%\n",val->intval);
				#endif
			}
			//即使再充电，也不上报
			else
			{
				val->intval = 100;
			}
		}
		
		first_check_c2 = 0;
	}
	
	return 0;
}

static char tps6586x_serial[5];
static int tps6586x_get_battery_serial_number(struct i2c_client *client,
	union power_supply_propval *val)
{
	int ret = 0xff ;
#if 0
	int ret;

	ret = i2c_smbus_read_word_data(client,
		tps6586x_data[REG_SERIAL_NUMBER].addr);
	if (ret < 0)
		return ret;

	ret = sprintf(tps6586x_serial, "%04x", ret);
	val->strval = tps6586x_serial;
#endif
	//把ret写到tps6586x_serial
	ret = sprintf(tps6586x_serial, "%04x", ret);
	val->strval = tps6586x_serial;
	
	return 0;
}

static int tps6586x_bat_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int count;
	int ret;
	struct i2c_client *client = tps6586x_device->client;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_HEALTH:
	//电池是否在，是否健康
		ret = tps6586x_get_battery_presence_and_health(client, psp, val);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_ENERGY_NOW:
	case POWER_SUPPLY_PROP_ENERGY_FULL:
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CAPACITY:
		//得到count值，也就是寄存器的偏移
		for (count = 0; count < ARRAY_SIZE(tps6586x_data); count++) 
		{
			if (psp == tps6586x_data[count].psp)
				break;
		}
		//电池的容量百分比
		//count是寄存器的值，psp和count是对应的
		ret = tps6586x_get_battery_capacity(client, count, psp, val);
	       //ec 000000 luyanfei modify for battery capacity display 20111216 ++
              last_battery_capacity = val->intval;
		//ec 000000 luyanfei modify for battery capacity display 20111216 --
		if (ret)
			return ret;

		break;

	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		ret = tps6586x_get_battery_serial_number(client, val);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		for (count = 0; count < REG_MAX; count++) 
		{
			if (psp == tps6586x_data[count].psp)
				break;
		}
		//电池的充电状态:充满还是正在充电
		ret = tps6586x_get_battery_property(client, count, psp, val);
		if (ret)
			return ret;

		break;

	default:
		dev_err(&tps6586x_device->client->dev,
			"%s: INVALID property\n", __func__);
		return -EINVAL;
	}

	/* Convert units to match requirements for power supply class */
	tps6586x_unit_adjustment(client, psp, val);

	dev_dbg(&client->dev,
		"%s: property = %d, value = %d\n", __func__, psp, val->intval);

	return 0;
}
//[ECID:000000] ZTEBSP shiyan 20111018 end, modify for batt info 

static irqreturn_t ac_present_irq(int irq, void *data)
{
	dev_err(&tps6586x_device->client->dev,"%s  \n",__func__);		
	
	//更新电池状态
	power_supply_changed(&tps6586x_supply[SUPPLY_TYPE_AC]);
	power_supply_changed(&tps6586x_supply[SUPPLY_TYPE_BATTERY]);
	return IRQ_HANDLED;
}

//[ECID:000000] ZTEBSP shiyan 20111018 start,  for batt info update 
 void  zte_ac_present_irq(void )
{
	charger_status_need_get = 1;
	ac_status_need_get = 1;
	power_supply_changed(&tps6586x_supply[SUPPLY_TYPE_AC]);
}
EXPORT_SYMBOL(zte_ac_present_irq);

 void  zte_chg_state_change_irq(void )
{
	batt_cap_need_cal = 1;
	power_supply_changed(&tps6586x_supply[SUPPLY_TYPE_BATTERY]);
}
EXPORT_SYMBOL(zte_chg_state_change_irq);

 bool  zte_bat_driver_registered(void )
{
	return tps6586x_bat_probe;
}
EXPORT_SYMBOL(zte_bat_driver_registered);
//[ECID:000000] ZTEBSP shiyan 20111018 end, modify for batt info update

static void battery_poll_timer_func(unsigned long unused)
{
	charger_status_need_get = 1;
	ac_status_need_get = 1;
	batt_cap_need_cal = 1;
	batt_volt_need_adc = 1;
	batt_temp_need_adc = 1;
	power_supply_changed(&tps6586x_supply[SUPPLY_TYPE_BATTERY]);
	power_supply_changed(&tps6586x_supply[SUPPLY_TYPE_AC]);

	//控制充电时间和接近充满时的百分比显示
	//75s*80/60+150s*(20+4) /60=160min
	if(read_charger_on_flag()&&(last_battery_capacity >=80))
	{
		mod_timer(&tps6586x_device->battery_poll_timer,
		jiffies + msecs_to_jiffies(BATTERY_POLL_PERIOD*2));    //150s
	}
	//修改低电过放的问题
	else if(!read_charger_on_flag()&&(last_battery_capacity<=5))
	{
		mod_timer(&tps6586x_device->battery_poll_timer,
		jiffies + msecs_to_jiffies(45000));    //45s
	}
	else
	{
		mod_timer(&tps6586x_device->battery_poll_timer,
		jiffies + msecs_to_jiffies(BATTERY_POLL_PERIOD));    //75s
	}
}

//khl
#ifdef CONFIG_HAS_EARLYSUSPEND
static int tps6586x_early_suspend(struct early_suspend *h)
{
	#if 0 //shiyan 
	s32 ret;
	struct tps6586x_device_info *tps6586x_device =
		container_of(h, struct tps6586x_device_info, early_suspend_handler);

	if (!tps6586x_device->battery_present)
		return 0;

	del_timer_sync(&tps6586x_device->battery_poll_timer);
	#endif //shiyan 

	return 0;
}

/* any smbus transaction will wake up tps6586x */
static int tps6586x_late_resume(struct early_suspend *h)
{
	struct tps6586x_device_info *tps6586x_device =
		container_of(h, struct tps6586x_device_info, early_suspend_handler);

	if (!tps6586x_device->battery_present)
		return 0;

	//shiyan added
	if(battery_timer_on == false)
	{
		setup_timer(&tps6586x_device->battery_poll_timer,
			battery_poll_timer_func, 0);
		mod_timer(&tps6586x_device->battery_poll_timer,
			jiffies + msecs_to_jiffies(300));   //shiyan modify from BATTERY_POLL_PERIOD
		battery_timer_on =true;
	}
	return 0;
}
#endif
//khl

static int tps6586x_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc, i, flags;
	int supply_index = SUPPLY_TYPE_BATTERY;

	tps6586x_device = kzalloc(sizeof(*tps6586x_device), GFP_KERNEL);
	if (!tps6586x_device)
		return -ENOMEM;

	tps6586x_device->client = client;
	flags = tps6586x_device->client->flags;
	tps6586x_device->client->flags &= ~I2C_M_IGNORE_NAK;

	{
		tps6586x_device->battery_present = true;
	}

	tps6586x_device->client->flags = flags;
	tps6586x_device->irq = client->irq;
	i2c_set_clientdata(client, tps6586x_device);

#ifdef CHG_FORBIT_SLEEP
	/*[ECID:000000] ZTEBSP shiyan, for usb-in suspend, 2012.1.30 start*/
	wake_lock_init(&charger_wake_lock, WAKE_LOCK_SUSPEND, "charger_in");
       wake_lock_init(&adc_wake_lock, WAKE_LOCK_SUSPEND, "adc_sample");
	/*[ECID:000000] ZTEBSP shiyan, for usb-in suspend, 201112.30 end*/
#endif

	for (i = supply_index; i < ARRAY_SIZE(tps6586x_supply); i++)
	{
//供电设备注册
		rc = power_supply_register(&client->dev,
			&tps6586x_supply[i]);
		if (rc) 
		{
			dev_err(&tps6586x_device->client->dev,
				"%s: Failed to register power supply\n",
				 __func__);
			goto fail_power_register;
		}
	}

	if (tps6586x_device->battery_present) 
	{
//轮询
		setup_timer(&tps6586x_device->battery_poll_timer,
			battery_poll_timer_func, 0);
		mod_timer(&tps6586x_device->battery_poll_timer,
			jiffies + msecs_to_jiffies(60000));  //  shiyan   modifiy from BATTERY_POLL_PERIOD
	}

	dev_info(&tps6586x_device->client->dev, "driver registered\n");
	//khl
#ifdef CONFIG_HAS_EARLYSUSPEND
	tps6586x_device->early_suspend_handler.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	tps6586x_device->early_suspend_handler.suspend = tps6586x_early_suspend;
	tps6586x_device->early_suspend_handler.resume = tps6586x_late_resume;
	register_early_suspend(&tps6586x_device->early_suspend_handler);
#endif
	//khl
	/*[ECID:000000] ZTEBSP shiyan 2012.2.14 start, for charger */
	tps6586x_bat_probe = true;
	/*[ECID:000000] ZTEBSP shiyan 2012.2.14 start, for charger */
	return 0;

fail_power_register:
	while (i--)
		power_supply_unregister(&tps6586x_supply[i]);
	free_irq(tps6586x_device->irq, tps6586x_device);
fail_irq:
	kfree(tps6586x_device);
	return rc;
}

static int tps6586x_remove(struct i2c_client *client)
{
	struct tps6586x_device_info *tps6586x_device =
		i2c_get_clientdata(client);
	int supply_index = 0, i;

	if (tps6586x_device->battery_present)
		del_timer_sync(&tps6586x_device->battery_poll_timer);
	else
		supply_index = SUPPLY_TYPE_AC;

	for (i = supply_index; i < ARRAY_SIZE(tps6586x_supply); i++)
		power_supply_unregister(&tps6586x_supply[i]);

	kfree(tps6586x_device);
	/*[ECID:000000] ZTEBSP shiyan 2012.2.14 start, for charger */
	tps6586x_bat_probe = false;
	/*[ECID:000000] ZTEBSP shiyan 2012.2.14 start, for charger */
	return 0;
}

#if defined (CONFIG_PM)
static int tps6586x_suspend(struct i2c_client *client,
	pm_message_t state)
{
	s32 ret;
	struct tps6586x_device_info *tps6586x_device =
		i2c_get_clientdata(client);

	tps6586x_suspend_flag = true;  //shiyan added 20120802

	if (!tps6586x_device->battery_present)
		return 0;

	//shiyan added
	if(battery_timer_on == true)
	{
		del_timer_sync(&tps6586x_device->battery_poll_timer);
		battery_timer_on = false;
	}

	return 0;
}

/* any smbus transaction will wake up tps6586x */
static int tps6586x_resume(struct i2c_client *client)
{

	tps6586x_suspend_flag = false;  //shiyan added 20120802

	#if 0  //shiyan 
	struct tps6586x_device_info *tps6586x_device =
		i2c_get_clientdata(client);

	if (!tps6586x_device->battery_present)
		return 0;

	setup_timer(&tps6586x_device->battery_poll_timer,
		battery_poll_timer_func, 0);
	mod_timer(&tps6586x_device->battery_poll_timer,
		jiffies + msecs_to_jiffies(1000));   //shiyan modify from BATTERY_POLL_PERIOD
	#endif

	return 0;
}
#endif

static const struct i2c_device_id tps6586x_id[] = {
	{ "tps6586x-battery", 0 },
	{},
};

static struct i2c_driver tps6586x_battery_driver = {
	.probe		= tps6586x_probe,
	.remove		= tps6586x_remove,
#if defined (CONFIG_PM)
	.suspend	= tps6586x_suspend,
	.resume		= tps6586x_resume,
#endif
	.id_table	= tps6586x_id,
	.driver = {
		.name	= "tps6586x-battery",
	},
};

static int __init tps6586x_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&tps6586x_battery_driver);
	if (ret)
		dev_err(&tps6586x_device->client->dev,
			"%s: i2c_add_driver failed\n", __func__);

	return ret;
}
module_init(tps6586x_battery_init);

static void __exit tps6586x_battery_exit(void)
{
	i2c_del_driver(&tps6586x_battery_driver);
}
module_exit(tps6586x_battery_exit);

MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("tps6586x battery monitor driver");
MODULE_LICENSE("GPL");
