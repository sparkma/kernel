/*
 *  max17040_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/max17040_battery.h>
#include <linux/slab.h>

#define MAX17040_VCELL_MSB	0x02
#define MAX17040_VCELL_LSB	0x03
#define MAX17040_SOC_MSB	0x04
#define MAX17040_SOC_LSB	0x05
#define MAX17040_MODE_MSB	0x06
#define MAX17040_MODE_LSB	0x07
#define MAX17040_VER_MSB	0x08
#define MAX17040_VER_LSB	0x09
#define MAX17040_RCOMP_MSB	0x0C
#define MAX17040_RCOMP_LSB	0x0D
#define MAX17040_CMD_MSB	0xFE
#define MAX17040_CMD_LSB	0xFF

#define MAX17040_DELAY		1000    //  1s
#define MAX17040_BATTERY_FULL	95

#define MAX17040_RETRY_COUNT 3

static int max17040_probled =0;

int max_17040_probled(void)
{
	return max17040_probled;
}
EXPORT_SYMBOL_GPL(max_17040_probled);

extern void zte_chg_state_change_irq(void); 
extern bool zte_bat_driver_registered(void);
// shiyan added ++

struct module_config {
	unsigned char addr;
	unsigned char content[16];
};

struct module_config configs[] = { // for P943T
#if 1
 // for P943T  1650mAh
	{0x40,{0xAD,0xB0,0xB4,0xF0,0xB8,0x00,0xBB,0x70,0xBB,0xF0,0xBC,0x70,0xBD,0x00,0xBD,0xA0}},
	{0x50,{0xBE,0x30,0xC0,0x20,0xC2,0x40,0xC4,0x50,0xC6,0x60,0xCA,0xA0,0xCD,0xA0,0xCF,0x40}},
	{0x60,{0x00,0xB0,0x18,0xE0,0x13,0xD0,0x60,0xF0,0x5C,0x00,0x7D,0xB0,0x46,0x90,0x4C,0xE0}},
	{0x70,{0x1E,0x70,0x1B,0x10,0x1C,0x00,0x1C,0x00,0x13,0x00,0x15,0x80,0x0C,0x60,0x0C,0x60}},
#else
	{0x40,{0x7F,0xE0,0xB6,0x70,0xB9,0x20,0xBA,0x20,0xBB,0x80,0xBC,0x90,0xBD,0x90,0xBE,0x80}},
	{0x50,{0xBF,0x80,0xC0,0xA0,0xC2,0xB0,0xC5,0x40,0xC6,0xE0,0xC9,0x00,0xCD,0x20,0xD1,0xE0}},
	{0x60,{0x00,0x20,0x0B,0x00,0x17,0x20,0x19,0x00,0x22,0x00,0x2B,0x00,0x16,0x50,0x0D,0xE0}},
	{0x70,{0x1A,0x70,0x0B,0xF0,0x0B,0xE0,0x0B,0x80,0x0B,0xF0,0x08,0x60,0x07,0x00,0x07,0x00}},
#endif
};
//shiyan added --

struct max17040_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		battery;
	struct max17040_platform_data	*pdata;

	/* State Of Connect */
	int online;
	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int status;
};

static struct max17040_chip *this_chip = NULL;
static int max17040_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17040_chip *chip = container_of(psy,
				struct max17040_chip, battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int max17040_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max17040_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

//shiyan added  ++
static int max17040_i2c_rx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_client *max17040_client = this_chip->client;
	struct i2c_msg msgs[] = {
		{
			.addr	= max17040_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= max17040_client->addr,
			.flags	= 1,    //I2C_M_RD
			.len	= len,
			.buf	= buf,
		}
	};
	if(NULL == this_chip || NULL == max17040_client || NULL == max17040_client->addr)
		return -EIO;

	for (i = 0; i < MAX17040_RETRY_COUNT; i++) 
	{
		if (i2c_transfer(max17040_client->adapter, msgs, 2) > 0) 
		{
			break;
		}
		mdelay(10);
	}

	if (i >= MAX17040_RETRY_COUNT) 
	{
		pr_err("%s: retry over %d\n", __FUNCTION__, MAX17040_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int max17040_i2c_tx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_client *max17040_client = this_chip->client;
	struct i2c_msg msg[] = {
		{
			.addr	= max17040_client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= buf,
		}
	};
	if(NULL == this_chip || NULL == max17040_client || NULL == max17040_client->addr)
		return -EIO;

	for (i = 0; i < MAX17040_RETRY_COUNT; i++) 
	{
		if (i2c_transfer(max17040_client->adapter, msg, 1) > 0) 
		{
			break;
		}
		mdelay(10);
	}

	if (i >= MAX17040_RETRY_COUNT) 
	{
		pr_err("%s: retry over %d\n", __FUNCTION__, MAX17040_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int max17040_update_rcomp(int temp)
{
	unsigned char data[4] = {0,0,0,0};
	const int StartingRCOMP = 74;// 20 C = 74
	int NewRCOMP;
	int ret;

	if(temp > 20)
		NewRCOMP = StartingRCOMP - (temp-20)/2;// -0.5
	else if(temp < 20)
		NewRCOMP = StartingRCOMP + (20-temp)*37/8;// -4.625;
	else
		NewRCOMP = StartingRCOMP;

	if(NewRCOMP > 255)
		NewRCOMP = 255;
	else if(NewRCOMP < 0)
		NewRCOMP = 0;

	data[0] = MAX17040_RCOMP_MSB;
	data[1] = NewRCOMP;
	data[2] = 0x00;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error\n",__FUNCTION__);
	}
	return ret;
}


static int max17040_load_model()
{
	unsigned char data[20];
	u8 OriginalRCOMP1,OriginalRCOMP2,OriginalOCV1,OriginalOCV2;
	int ret;
	unsigned char *config;

	memset(data,0,sizeof(data));

	// 1. Unlock Model Access
	data[0] = 0x3E;// OCV
	data[1] = 0x4A;
	data[2] = 0x57;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error1\n",__FUNCTION__);
		return ret;
	}

	// 2. Read RCOMP and OCV
	data[0] = MAX17040_RCOMP_MSB;
	ret = max17040_i2c_rx_data(data,4);
	if(ret)
	{
		printk("%s() error2\n",__FUNCTION__);
		return ret;
	}
	OriginalRCOMP1 = data[0];
	OriginalRCOMP2 = data[1];
	OriginalOCV1 = data[2];
	OriginalOCV2 = data[3];

	// 3. Write OCV
	//  OCVTest = 55616    
	data[0] = 0x0E;
	data[1] = 0xD9;//0xD9;   OCVTest  high byte
	data[2] = 0x40;//0x40;   OCVTest  low byte
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error3\n",__FUNCTION__);
		return ret;
	}

	//  4. Write RCOMP to a Maximum value of 0xFF00h
	data[0] = MAX17040_RCOMP_MSB;
	data[1] = 0xFF;
	data[2] = 0x00;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error4\n",__FUNCTION__);
		return ret;
	}

	//  5. Write the Model
	config = &(configs[0].addr);// 0x40
	ret = max17040_i2c_tx_data(config,17);
	if(ret)
	{
		printk("%s() error5\n",__FUNCTION__);
		return ret;
	}
	config = &(configs[1].addr);// 0x50
	ret = max17040_i2c_tx_data(config,17);
	if(ret)
	{
		printk("%s() error6\n",__FUNCTION__);
		return ret;
	}
	config = &(configs[2].addr);// 0x60
	ret = max17040_i2c_tx_data(config,17);
	if(ret)
	{
		printk("%s() error7\n",__FUNCTION__);
		return ret;
	}
	config = &(configs[3].addr);// 0x70
	ret = max17040_i2c_tx_data(config,17);
	if(ret)
	{
		printk("%s() error8\n",__FUNCTION__);
		return ret;
	}

	// 6. delay
	msleep(150);

	// 7. Write OCV
	data[0] = 0x0E;
	data[1] = 0xD9;//0xD9;
	data[2] = 0x40;//0x40;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error9\n",__FUNCTION__);
		return ret;
	}
	
	// 8. delay
	msleep(200);

	//  9. Read SOC and Compare to expected result
	data[0] = MAX17040_SOC_MSB;
	ret = max17040_i2c_rx_data(data,2);
	if(ret)
	{
		printk("%s() error10\n",__FUNCTION__);
		return ret;
	}
	// SOC1>=SOCCheckA  SOC1<=SOCCheckB 
	//if(data[0] >= 0x7B && data[0] <=0x7D)
	if(data[0] >= 0xE5 && data[0] <=0xE7)
	{
		printk("%s success, 0x%x\n",__FUNCTION__,data[0]);
	}
	else
	{
		printk("%s fail, 0x%x\n",__FUNCTION__,data[0]);
		return -1;
	}

	// 10. Restore RCOMP and OCV
	data[0] = MAX17040_RCOMP_MSB;
	data[1] = OriginalRCOMP1;
	data[2] = OriginalRCOMP2;
	data[3] = OriginalOCV1;
	data[4] = OriginalOCV2;
	ret = max17040_i2c_tx_data(data,5);
	if(ret)
	{
		printk("%s() error11\n",__FUNCTION__);
		return ret;
	}

	//11. Lock Model Access
	data[0] = 0x3E;
	data[1] = 0x00;
	data[2] = 0x00;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error12\n",__FUNCTION__);
		return ret;
	}

	return ret;
}


static int max17040_quick_start()
{
	unsigned char data[4] = {0,0,0,0};
	int ret;

	data[0] = MAX17040_MODE_MSB;
	data[1] = 0x40;
	data[2] = 0x00;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
		printk("%s() error\n",__FUNCTION__);
	return ret;
}
// shiyan added --


static int  max17040_reset(struct i2c_client *client)
{
//shiyan added
	int ret =0;

	ret = max17040_write_reg(client, MAX17040_CMD_MSB, 0x54);
	if (ret < 0)
	{
		return  ret;
	}
	ret = max17040_write_reg(client, MAX17040_CMD_LSB, 0x00);

	return ret;
}

static void max17040_get_vcell(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_VCELL_MSB);
	lsb = max17040_read_reg(client, MAX17040_VCELL_LSB);

	chip->vcell = (msb << 4) + (lsb >> 4);
}

static void max17040_get_soc(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_SOC_MSB);
	lsb = max17040_read_reg(client, MAX17040_SOC_LSB);

	chip->soc = msb;
}
//shiyan added
 int zte_max17040_get_vcell(void)
{
	struct i2c_client *max17040_client ;

	if(NULL != this_chip)
	{
		 max17040_client = this_chip->client;

		struct max17040_chip *chip = i2c_get_clientdata(max17040_client);
		u8 msb;
		u8 lsb;

		msb = max17040_read_reg(max17040_client, MAX17040_VCELL_MSB);
		lsb = max17040_read_reg(max17040_client, MAX17040_VCELL_LSB);

		chip->vcell = (msb << 4) + (lsb >> 4);
		chip->vcell = chip->vcell *5/4;   // 1.25mv  unit	
		
		return chip->vcell ;
	}
	else
	{
		return (-1);
	}
}
EXPORT_SYMBOL(zte_max17040_get_vcell);

int zte_max17040_get_soc(void)
{
	struct i2c_client *max17040_client = this_chip->client;

	if(NULL != this_chip)
	{
		 max17040_client = this_chip->client;
		struct max17040_chip *chip = i2c_get_clientdata(max17040_client);
		u8 msb;
		u8 lsb;

		msb = max17040_read_reg(max17040_client, MAX17040_SOC_MSB);
		lsb = max17040_read_reg(max17040_client, MAX17040_SOC_LSB);

		//bits= 19
		//chip->soc = (msb<<8 + lsb)/512;
		chip->soc = msb>>1;
		
		return chip->soc ;
	}
	else
	{
		return (-1);
	}
}
EXPORT_SYMBOL(zte_max17040_get_soc);
//shiyan added --

static void max17040_get_version(struct i2c_client *client)
{
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_VER_MSB);
	lsb = max17040_read_reg(client, MAX17040_VER_LSB);

	dev_info(&client->dev, "MAX17040 Fuel-Gauge Ver %d%d\n", msb, lsb);
}

//得到电池是否在的状态
static void max17040_get_online(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	if (chip->pdata->battery_online)
		chip->online = chip->pdata->battery_online();
	else
		chip->online = 1;
}

//得到充电状态
static void max17040_get_status(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	if (!chip->pdata->charger_online || !chip->pdata->charger_enable) 
	{
		chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
		return;
	}

	if (chip->pdata->charger_online()) 
	{
		if (chip->pdata->charger_enable())
			chip->status = POWER_SUPPLY_STATUS_CHARGING;
		else
			chip->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} 
	else 
	{
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	if (chip->soc > MAX17040_BATTERY_FULL)
		chip->status = POWER_SUPPLY_STATUS_FULL;
}

//定时器更新信息
static void max17040_work(struct work_struct *work)
{
	struct max17040_chip *chip;

	chip = container_of(work, struct max17040_chip, work.work);

	max17040_get_vcell(chip->client);
	max17040_get_soc(chip->client);
	max17040_get_online(chip->client);
	max17040_get_status(chip->client);

	schedule_delayed_work(&chip->work, MAX17040_DELAY);
}

static enum power_supply_property max17040_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static int __devinit max17040_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17040_chip *chip;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))//I2C_FUNC_SMBUS_BYTE
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	//指向相同的结构体
	this_chip = chip;
	
	chip->client = client;
	chip->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, chip);

	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17040_get_property;
	chip->battery.properties	= max17040_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17040_battery_props);

//shiyan
#if 0
	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		kfree(chip);
		return ret;
	}
#endif

	#if 0
	ret = max17040_reset(client);
	if(ret < 0)
	{
		printk("shiyan: max17040 not ready! \n");	
		return 0;
	}
	#endif
	
// shiyan added ++
	ret = max17040_load_model(client);
	if(ret < 0)
	{
		printk("shiyan: max17040 not ready! \n");	
		return 0;
	}
	
	#if 0
	ret = max17040_quick_start(client);
	if(ret < 0)
	{
		printk("shiyan: max17040 not ready! \n");	
		return 0;
	}
	#endif
// shiyan added --
	max17040_get_version(client);

// shiyan added ++
//暂时认为温度为20c
	max17040_update_rcomp(20);
// shiyan added --
//shiyan
#if 0
	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17040_work);
	schedule_delayed_work(&chip->work, MAX17040_DELAY);
#endif

	printk("shiyan: max17040_probe OK! \n");	
	max17040_probled = 1;  //0  not use max17040;  1  use it

	msleep(1000);
	if(true == zte_bat_driver_registered())
	{
		zte_chg_state_change_irq();
	}
	
	return 0;
}

static int __devexit max17040_remove(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->battery);
	cancel_delayed_work(&chip->work);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int max17040_suspend(struct i2c_client *client,
		pm_message_t state)
{
#if 0  //shiyan 
	struct max17040_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work(&chip->work);
#endif
	return 0;
}

static int max17040_resume(struct i2c_client *client)
{
#if 0  //shiyan 
	struct max17040_chip *chip = i2c_get_clientdata(client);

	schedule_delayed_work(&chip->work, MAX17040_DELAY);
#endif
	return 0;
}

#else

#define max17040_suspend NULL
#define max17040_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id max17040_id[] = {
	{ "max17040", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17040_id);

static struct i2c_driver max17040_i2c_driver = {
	.driver	= {
		.name	= "max17040",
	},
	.probe		= max17040_probe,
	.remove		= __devexit_p(max17040_remove),
	.suspend	= max17040_suspend,
	.resume		= max17040_resume,
	.id_table	= max17040_id,
};

static int __init max17040_init(void)
{
	printk("MAX17040 Fuel Gauge driver: initialize\n");
	return i2c_add_driver(&max17040_i2c_driver);
}
module_init(max17040_init);

static void __exit max17040_exit(void)
{
	i2c_del_driver(&max17040_i2c_driver);
}
module_exit(max17040_exit);

MODULE_AUTHOR("Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("MAX17040 Fuel Gauge");
MODULE_LICENSE("GPL");
