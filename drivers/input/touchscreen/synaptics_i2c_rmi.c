/* drivers/input/keyboard/synaptics_i2c_rmi.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* ========================================================================================
when         who        what, where, why                         comment tag
--------     ----       -------------------------------------    --------------------------
2012-3-12   zhangzhao  modify     use android virtual key system and modified the virtual key coordinate
2011-11-30   zhangzhao  change synaptics driver for TM2109 config                       
========================================================================================*/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/proc_fs.h>//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7

/*[ECID:00000] ZTEBSP zhangzhao modified tsc driver ,start 2011-11-18*/
#include <linux/gpio.h>
#include <linux/slab.h>     
#include "../../../arch/arm/mach-tegra/gpio-names.h"

#include <linux/regulator/consumer.h>
/*[ECID:00000] ZTEBSP zhangzhao modified tsc driver ,end 2011-11-18*/

//#define TSP_DEBUG  //[ECID:00000] ZTEBSP zhangzhao dis tsc print ,end 2012-3-13

#define WHISITLER_TSC_PWR_EN TEGRA_GPIO_PR7
//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7
#define FUNC34
//#define TOUCH_FIRMWARE_UPDATE  /*[ECID:0000] zhangzhao 2012-6-20 add auto update firmware func*/
 
#ifdef TOUCH_FIRMWARE_UPDATE/*[ECID:0000] zhangzhao 2012-6-20 add auto update firmware func*/
#define FIRMWARE_BYTE1  0x01
#define FIRMWARE_BYTE2  0x05
#endif
#ifdef FUNC34
#include <linux/firmware.h>
struct synaptics_rmi4 *ts_f34;
static u8 fw_ver[2];
#endif
//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7 end

#ifdef TOUCH_FIRMWARE_UPDATE
#include "syna.h"// /*[ECID:0000] zhangzhao 2012-6-20 add auto update firmware func ,the firmware file*/
#endif

#define BTN_F19 BTN_0
#define BTN_F30 BTN_0
#define SCROLL_ORIENTATION REL_Y


enum f11_finger_state {
	F11_NO_FINGER = 0,
	F11_PRESENT = 1,
	F11_INACCURATE = 2,
	F11_RESERVED = 3
};

static struct regulator *tegra_tsc_regulator_2v8;

static struct workqueue_struct *synaptics_wq;
static struct i2c_driver synaptics_ts_driver;

/* Register: EGR_0 */
#define EGR_PINCH_REG		0
#define EGR_PINCH 		(1 << 6)
#define EGR_PRESS_REG 		0
#define EGR_PRESS 		(1 << 5)
#define EGR_FLICK_REG 		0
#define EGR_FLICK 		(1 << 4)
#define EGR_EARLY_TAP_REG	0
#define EGR_EARLY_TAP		(1 << 3)
#define EGR_DOUBLE_TAP_REG	0
#define EGR_DOUBLE_TAP		(1 << 2)
#define EGR_TAP_AND_HOLD_REG	0
#define EGR_TAP_AND_HOLD	(1 << 1)
#define EGR_SINGLE_TAP_REG	0
#define EGR_SINGLE_TAP		(1 << 0)
/* Register: EGR_1 */
#define EGR_PALM_DETECT_REG	1
#define EGR_PALM_DETECT		(1 << 0)


struct synaptics_function_descriptor {
	__u8 queryBase;
	__u8 commandBase;
	__u8 controlBase;
	__u8 dataBase;

/*
	__u8 : 1;
	__u8 functionVersion : 2;
	__u8 : 2;
	__u8 interruptSourceCount : 3;
*/
	__u8 intSrc;
#define FUNCTION_VERSION(x) ((x >> 5) & 3)
#define INTERRUPT_SOURCE_COUNT(x) (x & 7)

	__u8 functionNumber;
};

#define FD_ADDR_MAX 0xE9
#define FD_ADDR_MIN 0x05
#define FD_BYTE_COUNT 6

#define CONFIG_SYNA_MULTI_TOUCH

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

#if defined(FUNC05)||defined(FUNC34)//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7
static struct i2c_client *client_synap;
#endif

static int synaptics_i2c_write(struct i2c_client *client, int reg, u8 data)
{
    u8 buf[2];
    int rc;
    int ret = 0;

    buf[0] = reg;
    buf[1] = data;
    rc = i2c_master_send(client, buf, 2);
    if (rc != 2)
    {
        dev_err(&client->dev, "synaptics_i2c_write FAILED: writing to reg %d\n", reg);
        ret = -1;
    }
    return ret;
}

static int synaptics_i2c_read(struct i2c_client *client, int reg, u8 * buf, int count)
{
    int rc;
    int ret = 0;

    buf[0] = reg;
    rc = i2c_master_send(client, buf, 1);
    if (rc != 1)
    {
        dev_err(&client->dev, "synaptics_i2c_read FAILED: read of register %d\n", reg);
        ret = -1;
        goto tp_i2c_rd_exit;
    }
    rc = i2c_master_recv(client, buf, count);
    if (rc != count)
    {
        dev_err(&client->dev, "synaptics_i2c_read FAILED: read %d bytes from reg %d\n", count, reg);
        ret = -1;
    }

  tp_i2c_rd_exit:
    return ret;
}

static int synaptics_rmi4_read_pdt(struct synaptics_rmi4 *ts)
{
	int ret = 0;
	int nFd = 0;
	int interruptCount = 0;
	__u8 data_length;

	struct i2c_msg fd_i2c_msg[2];
	__u8 fd_reg;
	struct synaptics_function_descriptor fd;

	struct i2c_msg query_i2c_msg[2];
	__u8 query[14];
	__u8 *egr;

	fd_i2c_msg[0].addr = ts->client->addr;
	fd_i2c_msg[0].flags = 0;
	fd_i2c_msg[0].buf = &fd_reg;
	fd_i2c_msg[0].len = 1;

	fd_i2c_msg[1].addr = ts->client->addr;
	fd_i2c_msg[1].flags = I2C_M_RD;
	fd_i2c_msg[1].buf = (__u8 *)(&fd);
	fd_i2c_msg[1].len = FD_BYTE_COUNT;

	query_i2c_msg[0].addr = ts->client->addr;
	query_i2c_msg[0].flags = 0;
	query_i2c_msg[0].buf = &fd.queryBase;
	query_i2c_msg[0].len = 1;

	query_i2c_msg[1].addr = ts->client->addr;
	query_i2c_msg[1].flags = I2C_M_RD;
	query_i2c_msg[1].buf = query;
	query_i2c_msg[1].len = sizeof(query);


	ts->hasF11 = false;
	ts->hasF19 = false;
	ts->hasF30 = false;
	ts->hasF34 = false;//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7
	ts->data_reg = 0xff;
	ts->data_length = 0;

	for (fd_reg = FD_ADDR_MAX; fd_reg >= FD_ADDR_MIN; fd_reg -= FD_BYTE_COUNT) {
         
		ret = i2c_transfer(ts->client->adapter, fd_i2c_msg, 2);
		if (ret < 0) {
			printk(KERN_ERR "I2C read failed querying RMI4 $%02X capabilities\n", ts->client->addr);
			return ret;
		}

		if (!fd.functionNumber) {
			/* End of PDT */
			ret = nFd;
			printk("Read %d functions from PDT\n", nFd);
			break;
		}
                            
		++nFd;
                #if 1
		printk("fd.functionNumber = %x\n", fd.functionNumber);
		printk("fd.queryBase = %x\n", fd.queryBase);
		printk("fd.commandBase = %x\n", fd.commandBase);
		printk("fd.controlBase = %x\n", fd.controlBase);
		printk("fd.dataBase = %x\n", fd.dataBase);
		printk("fd.intSrc = %x\n", fd.intSrc);
                #endif
		switch (fd.functionNumber) {
			case 0x01: /* Interrupt */
				ts->f01.data_offset = fd.dataBase;
				ts->f01.controlBase =  fd.controlBase;//zhangzhao add for 2202
				ts->f01.commandlBase = fd.commandBase;//ECID:0000 zhangzhao 2012-6-1 add tsc reset in probe function
				ts->f01.dataBase  = fd.dataBase;//ECID:0000 zhangzhao 2012-6-1 add tsc reset in probe function
				ts->f01.querryBase = fd.queryBase;//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7
					/*
				 * Can't determine data_length
				 * until whole PDT has been read to count interrupt sources
				 * and calculate number of interrupt status registers.
				 * Setting to 0 safely "ignores" for now.
				 */
				data_length = 0;
				break;
			case 0x11: /* 2D */
				ts->hasF11 = true;

				ts->f11.data_offset = fd.dataBase;
				ts->f11.interrupt_offset = interruptCount / 8;
				ts->f11.interrupt_mask = ((1 << INTERRUPT_SOURCE_COUNT(fd.intSrc)) - 1) << (interruptCount % 8);
                                                        
				ret = i2c_transfer(ts->client->adapter, query_i2c_msg, 2);
				if (ret < 0)
					printk(KERN_ERR "Error reading F11 query registers\n");

				ts->f11.points_supported = (query[1] & 7) + 1;
				if (ts->f11.points_supported == 6)
					ts->f11.points_supported = 10;

				ts->f11_fingers = kcalloc(ts->f11.points_supported,
				                          sizeof(*ts->f11_fingers), 0);

				printk("%d fingers\n", ts->f11.points_supported);
				ts->f11_has_gestures = (query[1] >> 5) & 1;
				ts->f11_has_relative = (query[1] >> 3) & 1;

				egr = &query[7];

#define EGR_DEBUG
#ifdef EGR_DEBUG
#define EGR_INFO printk
#else
#define EGR_INFO
#endif
				EGR_INFO("EGR features:\n");
				ts->hasEgrPinch = egr[EGR_PINCH_REG] & EGR_PINCH;
				EGR_INFO("\tpinch: %u\n", ts->hasEgrPinch);
				ts->hasEgrPress = egr[EGR_PRESS_REG] & EGR_PRESS;
				EGR_INFO("\tpress: %u\n", ts->hasEgrPress);
				ts->hasEgrFlick = egr[EGR_FLICK_REG] & EGR_FLICK;
				EGR_INFO("\tflick: %u\n", ts->hasEgrFlick);
				ts->hasEgrEarlyTap = egr[EGR_EARLY_TAP_REG] & EGR_EARLY_TAP;
				EGR_INFO("\tearly tap: %u\n", ts->hasEgrEarlyTap);
				ts->hasEgrDoubleTap = egr[EGR_DOUBLE_TAP_REG] & EGR_DOUBLE_TAP;
				EGR_INFO("\tdouble tap: %u\n", ts->hasEgrDoubleTap);
				ts->hasEgrTapAndHold = egr[EGR_TAP_AND_HOLD_REG] & EGR_TAP_AND_HOLD;
				EGR_INFO("\ttap and hold: %u\n", ts->hasEgrTapAndHold);
				ts->hasEgrSingleTap = egr[EGR_SINGLE_TAP_REG] & EGR_SINGLE_TAP;
				EGR_INFO("\tsingle tap: %u\n", ts->hasEgrSingleTap);
				ts->hasEgrPalmDetect = egr[EGR_PALM_DETECT_REG] & EGR_PALM_DETECT;
				EGR_INFO("\tpalm detect: %u\n", ts->hasEgrPalmDetect);

				query_i2c_msg[0].buf = &fd.controlBase;
				ret = i2c_transfer(ts->client->adapter, query_i2c_msg, 2);
				if (ret < 0)
					printk(KERN_ERR "Error reading F11 control registers\n");

				query_i2c_msg[0].buf = &fd.queryBase;

				ts->f11_max_x = ((query[7] & 0x0f) * 0x100) | query[6];
				ts->f11_max_y = ((query[9] & 0x0f) * 0x100) | query[8];
                                                        
				printk("max X: %d; max Y: %d\n", ts->f11_max_x, ts->f11_max_y);

				ts->f11.data_length = data_length =
					/* finger status, four fingers per register */
					((ts->f11.points_supported + 3) / 4)
					/* absolute data, 5 per finger */
					+ 5 * ts->f11.points_supported
					/* two relative registers */
					+ (ts->f11_has_relative ? 2 : 0)
					/* F11_2D_Data8 is only present if the egr_0 register is non-zero. */
					+ (egr[0] ? 1 : 0)
					/* F11_2D_Data9 is only present if either egr_0 or egr_1 registers are non-zero. */
					+ ((egr[0] || egr[1]) ? 1 : 0)
					/* F11_2D_Data10 is only present if EGR_PINCH or EGR_FLICK of egr_0 reports as 1. */
					+ ((ts->hasEgrPinch || ts->hasEgrFlick) ? 1 : 0)
					/* F11_2D_Data11 and F11_2D_Data12 are only present if EGR_FLICK of egr_0 reports as 1. */
					+ (ts->hasEgrFlick ? 2 : 0)
					;

				break;
			case 0x19: /* Cap Buttons */
				ts->hasF19 = true;

				ts->f19.data_offset = fd.dataBase;
				ts->f19.interrupt_offset = interruptCount / 8;
				ts->f19.interrupt_mask = ((1 < INTERRUPT_SOURCE_COUNT(fd.intSrc)) - 1) << (interruptCount % 8);
				//ret = i2c_transfer(ts->client->adapter, query_i2c_msg, 2);
				if (ret < 0)
					printk(KERN_ERR "Error reading F19 query registers\n");


				ts->f19.points_supported = query[1] & 0x1F;
				ts->f19.data_length = data_length = (ts->f19.points_supported + 7) / 8;

				printk(KERN_NOTICE "$%02X F19 has %d buttons\n", ts->client->addr, ts->f19.points_supported);

				break;
			case 0x30: /* GPIO */
				ts->hasF30 = true;

				ts->f30.data_offset = fd.dataBase;
				ts->f30.interrupt_offset = interruptCount / 8;
				ts->f30.interrupt_mask = ((1 < INTERRUPT_SOURCE_COUNT(fd.intSrc)) - 1) << (interruptCount % 8);

				ret = i2c_transfer(ts->client->adapter, query_i2c_msg, 2);
				if (ret < 0)
					printk(KERN_ERR "Error reading F30 query registers\n");


				ts->f30.points_supported = query[1] & 0x1F;
				ts->f30.data_length = data_length = (ts->f30.points_supported + 7) / 8;

				break;
			case 0x34://[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7
				ts->hasF30 = true;
				ts->f34.querryBase = fd.queryBase;
				ts->f34.dataBase = fd.dataBase;
				
				
			default:
				goto pdt_next_iter;
		}

		// Change to end address for comparison
		// NOTE: make sure final value of ts->data_reg is subtracted
		data_length += fd.dataBase;
		if (data_length > ts->data_length) {
			ts->data_length = data_length;
		}

		if (fd.dataBase < ts->data_reg) {
			ts->data_reg = fd.dataBase;
		}

pdt_next_iter:
		interruptCount += INTERRUPT_SOURCE_COUNT(fd.intSrc);
	}

	// Now that PDT has been read, interrupt count determined, F01 data length can be determined.
	ts->f01.data_length = data_length = 1 + ((interruptCount + 7) / 8);
	// Change to end address for comparison
	// NOTE: make sure final value of ts->data_reg is subtracted
	data_length += ts->f01.data_offset;
	if (data_length > ts->data_length) {
		ts->data_length = data_length;
	}

	// Change data_length back from end address to length
	// NOTE: make sure this was an address
	ts->data_length -= ts->data_reg;

	// Change all data offsets to be relative to first register read
	// TODO: add __u8 *data (= &ts->data[ts->f##.data_offset]) to struct rmi_function_info?
	ts->f01.data_offset -= ts->data_reg;
	ts->f11.data_offset -= ts->data_reg;
	ts->f19.data_offset -= ts->data_reg;
	ts->f30.data_offset -= ts->data_reg;

	ts->data = kcalloc(ts->data_length, sizeof(*ts->data), 0);
	if (ts->data == NULL) {
		printk(KERN_ERR "Not enough memory to allocate space for RMI4 data\n");
		ret = -ENOMEM;
	}

	ts->data_i2c_msg[0].addr = ts->client->addr;
	ts->data_i2c_msg[0].flags = 0;
	ts->data_i2c_msg[0].len = 1;
	ts->data_i2c_msg[0].buf = &ts->data_reg;

	ts->data_i2c_msg[1].addr = ts->client->addr;
	ts->data_i2c_msg[1].flags = I2C_M_RD;
	ts->data_i2c_msg[1].len = ts->data_length;
	ts->data_i2c_msg[1].buf = ts->data;

	printk(KERN_ERR "RMI4 $%02X data read: $%02X + %d\n",
		ts->client->addr, ts->data_reg, ts->data_length);

	return ret;
}
#ifdef FUNC34//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7

static int synaptics_i2c_multi_write(struct i2c_client *client, int reg, u8 * buf, int count)
{
    int rc;
    int ret = 0;
	int i;
	unsigned char *txbuf;
	unsigned char txbuf_most[17]; /* Use this buffer for fast writes of 16
					bytes or less.  The first byte will
					contain the address at which to start
					the write. */
					
	if (count < sizeof(txbuf_most)) {
		/* Avoid an allocation if we can help it. */
		txbuf = txbuf_most;
	} else {
		/* over 16 bytes write we'll need to allocate a temp buffer */
		txbuf = kzalloc(count + 1, GFP_KERNEL);
		if (!txbuf)
			return -ENOMEM;
	}

	/* Yes, it stinks here that we have to copy the buffer */
	/* We copy from valp to txbuf leaving
	the first location open for the address */
	for (i = 0; i < count; i++)
		txbuf[i + 1] = buf[i];
	
    txbuf[0] = reg; /* put the address in the first byte */
    rc = i2c_master_send(client, txbuf, count+1);
    if (rc != (count+1))
    {
        dev_err(&client->dev, "synaptics_i2c_write FAILED: writing to reg %d\n", reg);
        ret = -1;
    }
    return ret;
}

static void flash_program(void)
{
     u8 buf[20];
	 int ret=0;
	 int i;
	 int blockindex;
	 u8* data = NULL;
	const struct firmware *synap_fw = NULL;

 		printk(KERN_ERR "%s enter\n", __func__);

		disable_irq(client_synap->irq);

		/* read fw file */
		ret = request_firmware(&synap_fw, "syna.img", (struct device*)&(client_synap->dev));
		if (ret)
		{
			printk(KERN_CRIT " syna.img request failed %d\n",ret);
			return;
		}
		else 
		{
			data = (u8 *)synap_fw->data;
		}
		ret = synaptics_i2c_read(client_synap, (ts_f34->f01.querryBase+2), buf, 2); 
		printk(KERN_CRIT "ret=%d,old firmware version reg %x = 0x%x 0x%x\n",ret,(ts_f34->f01.querryBase+2),buf[0],buf[1]);


              synaptics_i2c_write(client_synap, (ts_f34->f01.controlBase+1), 0x00);       //only eanble flash INT
              printk(KERN_CRIT "enable flash Int reg = 0x%x\n",(ts_f34->f01.controlBase+1));
			  
	      //enable flash programming
	      ret = synaptics_i2c_read(client_synap, ts_f34->f34.querryBase, buf, 2); 
		printk(KERN_CRIT "ret=%d,bootloader id=( %x,%x)\n",ret,buf[0],buf[1]);

	       synaptics_i2c_multi_write(client_synap, (ts_f34->f34.dataBase+2), buf,2);

		ret = synaptics_i2c_read(client_synap, (ts_f34->f34.querryBase+3), buf, 2); 
		printk(KERN_CRIT "ret=%d,block size count=( %x,%x)\n",ret,buf[0],buf[1]);
		ret = synaptics_i2c_read(client_synap, (ts_f34->f34.querryBase+5), buf, 2); 
		printk(KERN_CRIT "ret=%d,firmware block num count=( %x,%x)\n",ret,buf[0],buf[1]);

		ret = synaptics_i2c_read(client_synap, (ts_f34->f34.querryBase+7), buf, 2); 
		printk(KERN_CRIT "ret=%d,configuration block num count=( %x,%x)\n",ret,buf[0],buf[1]);

	       synaptics_i2c_write(client_synap, (ts_f34->f34.dataBase+0x12), 0x0f);       //enable flash programming
              msleep(200);
      		i=0;
		do
		{
	               ret = synaptics_i2c_read(client_synap, (ts_f34->f34.dataBase+0x12), buf, 1); 
                      i++;
			printk(KERN_CRIT "command in progress,i=%x\n",i);
		}while(((ret!=0)||((buf[0]&0x80)==0)) &&(i<0xff));
	
	     printk(KERN_CRIT "flash cmd0x12=%x\n",buf[0]);
            synaptics_i2c_read(client_synap, (ts_f34->f01.dataBase+1), buf, 1);
	     printk(KERN_CRIT "INT status 0x14=%x\n",buf[0]);


	  //program the firmaware image 
	     ret = synaptics_i2c_read(client_synap, ts_f34->f34.querryBase, buf, 2); 
	     printk(KERN_CRIT "2 ret=%d,bootloader id=( %x,%x)\n",ret,buf[0],buf[1]);

	     synaptics_i2c_multi_write(client_synap, (ts_f34->f34.dataBase+2), buf,2);
	     synaptics_i2c_write(client_synap, (ts_f34->f34.dataBase+0x12), 0x03);       //erase flash
              msleep(700);
      i=0;
	do
	{
	      // msleep(30);
	       ret = synaptics_i2c_read(client_synap, (ts_f34->f34.dataBase+0x12), buf, 1); 
              i++;
		printk(KERN_CRIT "2 command in progress,i=%x\n",i);
		}while(((ret!=0)||((buf[0]&0x80)==0)) &&(i<0xff));
	     printk(KERN_CRIT " 2 flash cmd0x12=%x\n",buf[0]);
            synaptics_i2c_read(client_synap, (ts_f34->f01.dataBase+1), buf, 1);
	     printk(KERN_CRIT "2 INT status 0x14=%x\n",buf[0]);


buf[0] = 0;
buf[1] = 0;
synaptics_i2c_multi_write(client_synap, ts_f34->f34.dataBase, buf,2);
//program firmware
//the frist 0x100 bytes is the header message
data +=0x100;
for(blockindex=0;blockindex<0x6e0;blockindex++)
{

	/*printk(KERN_ALERT "blockindex(%x,%x);syna_bin:%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n", \
			buf[0],buf[1],data[0],data[1], data[2], data[3],	data[4], data[5] , data[6], data[7],data[8], data[9], data[10],data[11]);
	*/
	//synaptics_i2c_multi_write(client_synap, 0x00, buf,2);

     
	synaptics_i2c_multi_write(client_synap, (ts_f34->f34.dataBase+0x02), data,16);    //??
       synaptics_i2c_write(client_synap, (ts_f34->f34.dataBase+0x12), 0x02);       //programming firmware

	data +=16;
	msleep(20);
      i=0;
	do{
	               ret = synaptics_i2c_read(client_synap, (ts_f34->f34.dataBase+0x12), buf, 1); 
                      i++;
		printk(KERN_CRIT "3 command in progress,i=%x blockindex = %x\n",i,blockindex);
		}while(((ret!=0)||((buf[0]&0x80)==0)) &&(i<0xff));
     }
	     printk(KERN_ERR "---------------------\n");

	     printk(KERN_ERR "-------++++-blockindex =%x\n",blockindex);

buf[0] = 0;
buf[1] = 0;
synaptics_i2c_multi_write(client_synap, ts_f34->f34.dataBase, buf,2);
//program the configration image
for(blockindex=0;blockindex<0x20;blockindex++)
{
   
	/*printk(KERN_CRIT "blockindex(%x,%x);syna_bin:%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n", \
			buf[0],buf[1],data[0],data[1], data[2], data[3],	data[4], data[5] , data[6], data[7],data[8], data[9], data[10],data[11]);
	*/
	//synaptics_i2c_multi_write(client_synap, 0x00, buf,2);

     
	synaptics_i2c_multi_write(client_synap, (ts_f34->f34.dataBase+0x02), data,16);    //??
       synaptics_i2c_write(client_synap, (ts_f34->f34.dataBase+0x12), 0x06);       //enable flash programming

	data +=16;
	msleep(20);
      i=0;
	do
	{
	       ret = synaptics_i2c_read(client_synap, (ts_f34->f34.dataBase+0x12), buf, 1); 
              i++;
		printk(KERN_CRIT "31 command in progress,i=%x blockindex=%x\n",i,blockindex);
		}while(((ret!=0)||((buf[0]&0x80)==0)) &&(i<0xff));
	 //    printk(KERN_CRIT " 31 flash cmd0x12=%x\n",buf[0]);
        //  synaptics_i2c_read(client_synap, 0x14, buf, 1);
	  //   printk(KERN_CRIT "31 INT status 0x14=%x\n",buf[0]);
}
	     printk(KERN_ERR "--------blockindex=%x\n",blockindex);

//zhangqi add for test
ret = synaptics_i2c_read(client_synap,ts_f34->f01.dataBase, buf, 1); 
printk(KERN_CRIT "++++++before reset,0x13=%x\n",buf[0]);
//disable program	   
      synaptics_i2c_write(client_synap, ts_f34->f01.commandlBase, 0x01);       //enable flash programming

      msleep(30);

//zhangqi add for test
ret = synaptics_i2c_read(client_synap, ts_f34->f01.dataBase, buf, 1); 
printk(KERN_CRIT "-------after reset ,0x13=%x\n",buf[0]);
	i=0;
	do{
                      msleep(30);
	               ret = synaptics_i2c_read(client_synap, ts_f34->f01.dataBase, buf, 1); 
                      i++;
		printk(KERN_CRIT "4 command in progress,i=%x,0x13=%x\n",i,buf[0]);
	}while(((ret!=0)||((buf[0]&0x40)!=0)) &&(i<0x1ff));
	
          synaptics_i2c_read(client_synap, (ts_f34->f01.dataBase+1), buf, 1);
	     printk(KERN_CRIT "4 INT status 0x14=%x\n",buf[0]);


        ret = synaptics_rmi4_read_pdt(ts_f34);

      //init
       synaptics_i2c_write(client_synap, (ts_f34->f01.controlBase+1), 0x04);       //only eanble abs INT
       //synaptics_i2c_write(client_synap, (ts_f34->f01.controlBase+2), 0x01);       //reduced reporting mode

		ret = synaptics_i2c_read(client_synap, (ts_f34->f01.querryBase+2), fw_ver, 2); 
		printk(KERN_CRIT "ret=%d,new firmware version 0x%x 0x%x\n",ret,fw_ver[0],fw_ver[1]);

	   	    enable_irq(client_synap->irq);

}
#endif//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7 end
static void synaptics_tsc_enable(struct synaptics_rmi4 *ts)
{
//	printk("%s: %s()\n", ts->client->name, __func__);
	if (ts->use_irq)
		enable_irq(ts->client->irq);
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	ts->enable = 1;
}

#ifdef TOUCH_FIRMWARE_UPDATE/*[ECID:0000] zhangzhao 2012-6-20 add auto update firmware func*/
static void update_syna_file(void)
{
     u8 buf[20];
	 int ret=0;
	 int i;
	 int blockindex;
	 u8* data = NULL;
       u8 touch_ic_name[]="tm2109";// = NULL;			  
	//const struct firmware *synap_fw = NULL;

 		printk(KERN_ERR "%s enter\n", __func__);

             {
		int retry = 3;
		while (retry-- > 0)
		{
			printk("wly: synaptics_i2c_read, %s\n",touch_ic_name);

			ret = synaptics_i2c_read(client_synap, (ts_f34->f01.querryBase+11), touch_ic_name, 6);
			printk("wly: synaptics_i2c_read, ts->f01.querryBase+11 = %x,\n",(ts_f34->f01.querryBase+11));
			printk("wly: synaptics_i2c_read, %s\n",touch_ic_name);
			if (ret >= 0)
				break;
			msleep(10);
		}
		if (retry < 0)
		{
			ret = -1;
			printk("[TSP]: synaptics_i2c_read, ERROR \n");			
			return;
		}
        	}
        	if  (strcmp((char *)&touch_ic_name[0], "TM2109"))
        	{ 
        	 printk("firmware is not 3k sis, cant update!\n");
        	 return;
        	}

		
		//TODO here compare version

		
		ret = synaptics_i2c_read(client_synap, (ts_f34->f01.querryBase+2), buf, 2); 
		printk(KERN_CRIT "ret=%d,old firmware version reg %x = 0x%x 0x%x\n",ret,(ts_f34->f01.querryBase+2),buf[0],buf[1]);
              printk("[TPS] file 's firmware is %x ,%x \n",FIRMWARE_BYTE1,FIRMWARE_BYTE2);
		if((FIRMWARE_BYTE1 ==buf[0])&&(FIRMWARE_BYTE2==buf[1]))
			{
                	 printk("[TPS] same firmware,do not need update!\n");
                	 return;
			}

		disable_irq(client_synap->irq);
		data = &syna[0];
              synaptics_i2c_write(client_synap, (ts_f34->f01.controlBase+1), 0x00);       //only eanble flash INT
              printk(KERN_CRIT "enable flash Int reg = 0x%x\n",(ts_f34->f01.controlBase+1));
			  
	      //enable flash programming
	      ret = synaptics_i2c_read(client_synap, ts_f34->f34.querryBase, buf, 2); 
		printk(KERN_CRIT "ret=%d,bootloader id=( %x,%x)\n",ret,buf[0],buf[1]);

	       synaptics_i2c_multi_write(client_synap, (ts_f34->f34.dataBase+2), buf,2);

		ret = synaptics_i2c_read(client_synap, (ts_f34->f34.querryBase+3), buf, 2); 
		printk(KERN_CRIT "ret=%d,block size count=( %x,%x)\n",ret,buf[0],buf[1]);
		ret = synaptics_i2c_read(client_synap, (ts_f34->f34.querryBase+5), buf, 2); 
		printk(KERN_CRIT "ret=%d,firmware block num count=( %x,%x)\n",ret,buf[0],buf[1]);

		ret = synaptics_i2c_read(client_synap, (ts_f34->f34.querryBase+7), buf, 2); 
		printk(KERN_CRIT "ret=%d,configuration block num count=( %x,%x)\n",ret,buf[0],buf[1]);

	       synaptics_i2c_write(client_synap, (ts_f34->f34.dataBase+0x12), 0x0f);       //enable flash programming
              msleep(200);
      		i=0;
		do
		{
	               ret = synaptics_i2c_read(client_synap, (ts_f34->f34.dataBase+0x12), buf, 1); 
                      i++;
			printk(KERN_CRIT "command in progress,i=%x\n",i);
		}while(((ret!=0)||((buf[0]&0x80)==0)) &&(i<0xff));
	
	     printk(KERN_CRIT "flash cmd0x12=%x\n",buf[0]);
            synaptics_i2c_read(client_synap, (ts_f34->f01.dataBase+1), buf, 1);
	     printk(KERN_CRIT "INT status 0x14=%x\n",buf[0]);


	  //program the firmaware image 
	     ret = synaptics_i2c_read(client_synap, ts_f34->f34.querryBase, buf, 2); 
	     printk(KERN_CRIT "2 ret=%d,bootloader id=( %x,%x)\n",ret,buf[0],buf[1]);

	     synaptics_i2c_multi_write(client_synap, (ts_f34->f34.dataBase+2), buf,2);
	     synaptics_i2c_write(client_synap, (ts_f34->f34.dataBase+0x12), 0x03);       //erase flash
              msleep(700);
      i=0;
	do
	{
	      // msleep(30);
	       ret = synaptics_i2c_read(client_synap, (ts_f34->f34.dataBase+0x12), buf, 1); 
              i++;
		printk(KERN_CRIT "2 command in progress,i=%x\n",i);
		}while(((ret!=0)||((buf[0]&0x80)==0)) &&(i<0xff));
	     printk(KERN_CRIT " 2 flash cmd0x12=%x\n",buf[0]);
            synaptics_i2c_read(client_synap, (ts_f34->f01.dataBase+1), buf, 1);
	     printk(KERN_CRIT "2 INT status 0x14=%x\n",buf[0]);


buf[0] = 0;
buf[1] = 0;
synaptics_i2c_multi_write(client_synap, ts_f34->f34.dataBase, buf,2);
//program firmware
//the frist 0x100 bytes is the header message
data +=0x100;
for(blockindex=0;blockindex<0x6e0;blockindex++)
{

	/*printk(KERN_ALERT "blockindex(%x,%x);syna_bin:%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n", \
			buf[0],buf[1],data[0],data[1], data[2], data[3],	data[4], data[5] , data[6], data[7],data[8], data[9], data[10],data[11]);
	*/
	//synaptics_i2c_multi_write(client_synap, 0x00, buf,2);

     
	synaptics_i2c_multi_write(client_synap, (ts_f34->f34.dataBase+0x02), data,16);    //??
       synaptics_i2c_write(client_synap, (ts_f34->f34.dataBase+0x12), 0x02);       //programming firmware

	data +=16;
	msleep(20);
      i=0;
	do{
	               ret = synaptics_i2c_read(client_synap, (ts_f34->f34.dataBase+0x12), buf, 1); 
                      i++;
		//printk(KERN_CRIT "3 command in progress,i=%x blockindex = %x\n",i,blockindex);
		}while(((ret!=0)||((buf[0]&0x80)==0)) &&(i<0xff));
     }
	     printk(KERN_ERR "---------------------\n");

	     printk(KERN_ERR "-------++++-blockindex =%x\n",blockindex);

buf[0] = 0;
buf[1] = 0;
synaptics_i2c_multi_write(client_synap, ts_f34->f34.dataBase, buf,2);
//program the configration image
for(blockindex=0;blockindex<0x20;blockindex++)
{
   
	/*printk(KERN_CRIT "blockindex(%x,%x);syna_bin:%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n", \
			buf[0],buf[1],data[0],data[1], data[2], data[3],	data[4], data[5] , data[6], data[7],data[8], data[9], data[10],data[11]);
	*/
	//synaptics_i2c_multi_write(client_synap, 0x00, buf,2);

     
	synaptics_i2c_multi_write(client_synap, (ts_f34->f34.dataBase+0x02), data,16);    //??
       synaptics_i2c_write(client_synap, (ts_f34->f34.dataBase+0x12), 0x06);       //enable flash programming

	data +=16;
	msleep(20);
      i=0;
	do
	{
	       ret = synaptics_i2c_read(client_synap, (ts_f34->f34.dataBase+0x12), buf, 1); 
              i++;
		//printk(KERN_CRIT "31 command in progress,i=%x blockindex=%x\n",i,blockindex);
		}while(((ret!=0)||((buf[0]&0x80)==0)) &&(i<0xff));
	 //    printk(KERN_CRIT " 31 flash cmd0x12=%x\n",buf[0]);
        //  synaptics_i2c_read(client_synap, 0x14, buf, 1);
	  //   printk(KERN_CRIT "31 INT status 0x14=%x\n",buf[0]);
}
	     printk(KERN_ERR "--------blockindex=%x\n",blockindex);

//zhangqi add for test
ret = synaptics_i2c_read(client_synap,ts_f34->f01.dataBase, buf, 1); 
printk(KERN_CRIT "++++++before reset,0x13=%x\n",buf[0]);
//disable program	   
      synaptics_i2c_write(client_synap, ts_f34->f01.commandlBase, 0x01);       //enable flash programming

      msleep(30);

//zhangqi add for test
ret = synaptics_i2c_read(client_synap, ts_f34->f01.dataBase, buf, 1); 
printk(KERN_CRIT "-------after reset ,0x13=%x\n",buf[0]);
	i=0;
	do{
                      msleep(30);
	               ret = synaptics_i2c_read(client_synap, ts_f34->f01.dataBase, buf, 1); 
                      i++;
		printk(KERN_CRIT "4 command in progress,i=%x,0x13=%x\n",i,buf[0]);
	}while(((ret!=0)||((buf[0]&0x40)!=0)) &&(i<0x1ff));
	
          synaptics_i2c_read(client_synap, (ts_f34->f01.dataBase+1), buf, 1);
	     printk(KERN_CRIT "4 INT status 0x14=%x\n",buf[0]);


        ret = synaptics_rmi4_read_pdt(ts_f34);

      //init
       synaptics_i2c_write(client_synap, (ts_f34->f01.controlBase+1), 0x04);       //only eanble abs INT

		ret = synaptics_i2c_read(client_synap, (ts_f34->f01.querryBase+2), fw_ver, 2); 
		printk(KERN_CRIT "ret=%d,new firmware version 0x%x 0x%x\n",ret,fw_ver[0],fw_ver[1]);

	   	    enable_irq(client_synap->irq);

}
#endif/*[ECID:0000] zhangzhao 2012-6-20 add auto update firmware func end*/

//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7
static int proc_write_val(struct file *file, const char *buffer,
           unsigned long count, void *data)
{
		unsigned long val;
		sscanf(buffer, "%lu", &val);
		if (val >= 0) {
		if(val==1)
		{

		     //reset_touch_ic();
		}
#ifdef FUNC34
		else if (val==3)
		{
			printk("flash programming write \n");
			flash_program();	
		}
#endif
		else
		{
#ifdef FUNC05
                   	printk("reporting mode 2\n");
		     report_mode = 2;
		     image_report(2);     //delta data
#endif
		}
			
			return count;
		}
		return -EINVAL;
}
//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7 end
static void synaptics_ts_work_func(struct work_struct *work)
{
	int ret;
	struct synaptics_rmi4 *ts = container_of(work, struct synaptics_rmi4, work);
#ifdef FUNC05
	u8 reg, len, index;
#endif
	//ret = i2c_transfer(ts->client->adapter, ts->data_i2c_msg, 2);
       ret = synaptics_i2c_read(ts->client, *(ts->data_i2c_msg[0].buf), ts->data_i2c_msg[1].buf, ts->data_i2c_msg[1].len);

	if (ret < 0) {
		printk( "[TSP] %s: i2c_transfer failed\n", __func__);
	} else {
		__u8 *interrupt = &ts->data[ts->f01.data_offset + 1];
		#ifdef TSP_DEBUG
		printk("[TSP] int status: 0x%x,IntMask:0x%x\n", interrupt[ts->f11.interrupt_offset], ts->f11.interrupt_mask);
		#endif
		if (ts->hasF11 && (interrupt[ts->f11.interrupt_offset] & ts->f11.interrupt_mask))     //mengzf
		{
		       /* number of touch points - fingers down in this case */
 	              int fingerDownCount = 0;
			__u8 fsr_len = (ts->f11.points_supported + 3) / 4;
			__u8 *f11_data = &ts->data[ts->f11.data_offset];
	             int f;
				 
	             for (f = 0; f < ts->f11.points_supported; ++f) 
	             {
				/*finger status*/
				__u8 finger_status_reg = 0;
				__u8 finger_status;

				finger_status_reg = f11_data[f / 4];
				finger_status = (finger_status_reg >> ((f % 4) * 2)) & 3;
				if (finger_status == f11_finger_accurate ) 	{
					fingerDownCount++;
					ts->wasdown = true;
				}
	                   if (finger_status == f11_finger_accurate) {
			             __u8 reg;
			             __u8 *finger_reg;
			             u12 x;
			             u12 y;
					u4 wx = 0;
					u4 wy = 0;
					int z=0;
					
					reg = fsr_len + 5 * f;
				       finger_reg = &f11_data[reg];
					x = (finger_reg[0] * 0x10) | (finger_reg[2] % 0x10);	
				       y = (finger_reg[1] * 0x10) | (finger_reg[2] / 0x10);
					wx = finger_reg[3] % 0x10;
					wy = finger_reg[3] / 0x10;
					z = finger_reg[4];
					#ifdef TSP_DEBUG
					printk(  "[TSP] f: %d, s:%d, x: %d, y: %d\n", f,finger_status,x,y);
					#endif
	                            input_report_key(ts->input_dev,BTN_TOUCH, 1);
					
					/* if this is the first finger report normal
					ABS_X, ABS_Y, PRESSURE, TOOL_WIDTH events for
					non-MT apps. Apps that support Multi-touch
					will ignore these events and use the MT events.
					Apps that don't support Multi-touch will still
					function.
					*/
				#ifdef CONFIG_SYNA_MULTI_TOUCH//ecid:0000 zhangzhao 2012-5-18 decrease report event
					/* Report Multi-Touch events for each finger */
					/* major axis of touch area ellipse */
					input_report_abs(ts->input_dev, ABS_MT_PRESSURE, z);
					/* minor axis of touch area ellipse */
					//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
					//		max(wx, wy));
					/* Currently only 2 supported - 1 or 0 */
					//input_report_abs(ts->input_dev, ABS_MT_ORIENTATION,
					//	(wx > wy ? 1 : 0));
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
					/* TODO: Tracking ID needs to be reported but not used yet. */
					/* Could be formed by keeping an id per position and assiging */
					/* a new id when fingerStatus changes for that position.*/
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, f);
					/* MT sync between fingers */
					input_mt_sync(ts->input_dev);
				#endif
				}
			}

		/* if we had a finger down before and now we don't have any send a button up. */
			if ((fingerDownCount == 0) && ts->wasdown) {
				ts->wasdown = false;
			#ifdef CONFIG_SYNA_MULTI_TOUCH
				//input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
				//input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->oldX);
				//input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->oldY);
				//input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, f);
				input_mt_sync(ts->input_dev);
			#endif
				//input_report_abs(ts->input_dev, ABS_X, ts->oldX);
				//input_report_abs(ts->input_dev, ABS_Y, ts->oldY);
				//ts->oldX = ts->oldY = 0;
				#ifdef TSP_DEBUG
				printk( "[TSP] %s: Finger up.", __func__);
				#endif
	                     input_report_key(ts->input_dev,BTN_TOUCH, 0);
				
			}
	              input_sync(ts->input_dev);     /* sync after groups of events */
		} 


	}
	if (ts->use_irq) {
		enable_irq(ts->client->irq);
	}
}


static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_rmi4 *ts = container_of(timer, struct synaptics_rmi4, timer);
	/* printk("synaptics_ts_timer_func\n"); */

	queue_work(synaptics_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_rmi4 *ts = dev_id;

	//printk("synaptics_ts_irq_handler========\n"); 
	disable_irq_nosync(ts->client->irq);
	//printk("synaptics  disable_irq_nosync=====\n"); 
	queue_work(synaptics_wq, &ts->work);
	return IRQ_HANDLED;
}


/*************************virtual key*********************************************/
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_MENU) ":62:1010:115:80"
      ":"     __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":205:1010:120:80"
      ":"     __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":340:1010:120:80"
      ":"     __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":477:1010:115:80"
      "\n");
	/*key_type : key_value : center_x : cener_y : area_width : area_height*/
	/*ECID:0000 zhangzhao 2012-5-16 increase touch key area*/
}

static struct kobj_attribute virtuak_key_attr = {
         .attr  = {
		 .name = "virtualkeys.Synaptics_RMI4",
		 .mode = S_IRUGO,	
		 },
	  .show = &virtual_keys_show,	 
};

static struct attribute *properties_attrs[] = {
         &virtuak_key_attr.attr,
         NULL
};

static struct attribute_group proterties_attr_group = {
         . attrs =properties_attrs, 
};

struct kobject *properties_kobj;
static void virtual_keys_init(void)
{
    int ret;

    properties_kobj = kobject_create_and_add("board_properties",NULL);
    if(properties_kobj)
            ret = sysfs_create_group(properties_kobj,&proterties_attr_group);
	     if(!properties_kobj ||ret)
		printk(KERN_ERR "virtual_keys Unable to create ecryptfs version attributes\n");
}

/*************************virtual key*********************************************/

static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int i;
	int ret = 0;
              int min_x = 0;
              int min_y = 0;
              //int count = 500;
      	uint8_t buf1[10];
       u8 touch_ic_name[]="tm2109";// /*[ECID:0000] zhangzhao 2012-6-20 add auto update firmware func*/			  
 	uint8_t fw_version[2];
        
	struct synaptics_rmi4 *ts;
	struct proc_dir_entry *dir, *refresh;//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7
              
              struct synaptics_ts_platform_data *pdata = pdata = client->dev.platform_data;
              if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	      printk(KERN_ERR "probing for Synaptics RMI4 device %s at $%02X...\n", client->name, client->addr);
			  
              if (pdata->init_platform_hw)
		pdata->init_platform_hw();



#ifdef SKATE
	ret = gpio_request(WHISITLER_TSC_PWR_EN, "touch voltage");
	if (ret)
	{	
		printk("gpio WHISITLER_TSC_PWR_EN request is error!\n");
		goto err_check_functionality_failed;
	}   
	gpio_direction_output(WHISITLER_TSC_PWR_EN, 1);
       gpio_set_value(WHISITLER_TSC_PWR_EN,1);
	tegra_gpio_enable(WHISITLER_TSC_PWR_EN);
       printk(KERN_ERR "RMI4 enable tsc power\n");
	msleep(100);
#else
	tegra_tsc_regulator_2v8 = regulator_get(&client->dev, "vcc_tsc");	
	if (IS_ERR_OR_NULL(tegra_tsc_regulator_2v8)) {
               printk(KERN_ERR "RMI4 enable tsc power failed\n");
		goto err_regulator_get_failed;
	}
//[ECID:0000] zhangzhao 2012-2-21 for 943 ics touch	
	regulator_set_voltage(tegra_tsc_regulator_2v8, 3100, 3300);
	regulator_enable(tegra_tsc_regulator_2v8);
    msleep(250);

#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	//ts = (struct synaptics_rmi4 *)client->dev.platform_data;//deleted by wangweiping test for ts
	ts = kzalloc(sizeof(struct synaptics_rmi4), GFP_KERNEL); //added by wangweiping test for ts
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	client->driver = &synaptics_ts_driver;
//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7
#if defined(FUNC05)||defined(FUNC34)
	client_synap = ts->client;
#endif
#ifdef FUNC34
	ts_f34=ts;
#endif
              ret = synaptics_rmi4_read_pdt(ts);
	if (ret <= 0) {
		if (ret == 0)
			printk(KERN_ERR "Empty PDT\n");

		printk(KERN_ERR "Error identifying device (%d)\n", ret);
		ret = -ENODEV;
		goto err_pdt_read_failed;
	}

	synaptics_i2c_write(ts->client,ts->f01.commandlBase,0x01);
       msleep(200);	  
	synaptics_i2c_read(ts->client, ts->f01.dataBase, buf1, 1);
	printk("---init-- synaptics_i2c_read, commandbase=%x, database=%x,reg = %x\n",ts->f01.commandlBase,ts->f01.dataBase,buf1[0]);

      {
/*[ECID:0000] zhangzhao 2012-6-20 add auto update firmware func START*/	  
		int retry = 3;
		while (retry-- > 0)
		{
			printk("[TSP]: synaptics_i2c_read, %s\n",touch_ic_name);

			ret = synaptics_i2c_read(ts->client, (ts->f01.querryBase+11), touch_ic_name, 6);
			printk("[TSP]: synaptics_i2c_read, ts->f01.querryBase+11 = %x,\n",(ts->f01.querryBase+11));
			printk("[TSP]: synaptics_i2c_read, %s\n",touch_ic_name);
			if (ret >= 0)
				break;
			msleep(10);
		}
		
		if (retry < 0)
		{
			ret = -1;
			goto err_detect_failed;
		}
	}
	if  (!strcmp((char *)&touch_ic_name[0], "TM2109"))
	{ 
	 printk("firmware is 3k sis, can update!\n");
	}

      	{
       unsigned char product_id_ver[15] = {0,};
       unsigned char ver;
	ret = synaptics_i2c_read(ts->client, (ts->f01.querryBase+11), product_id_ver, 10);
	ret = synaptics_i2c_read(ts->client, (ts->f01.querryBase+3), &ver, 1);
       sprintf(&(product_id_ver[10]), "%-2x", ver);
       printk("img_ver:, product_id_ver:%s\n",  product_id_ver);
       }	
/*[ECID:0000] zhangzhao 2012-6-20 add auto update firmware func END*/	   
	   		
       synaptics_i2c_write(ts->client, (ts->f01.controlBase)+1, 0x00);       //only eanble abs INT

	printk(KERN_ERR " synaptics_ts_probe,max_x=%d, max_y=%d\n", ts->f11_max_x, ts->f11_max_y);

	synaptics_i2c_read(ts->client, (ts->f01.querryBase+2), fw_version, 2);
	printk("[TSP] firmware version: 0x%x 0x%x\n", fw_version[0], fw_version[1]);
//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7 end
	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		printk(KERN_ERR "failed to allocate input device.\n");
		ret = -EBUSY;
		goto err_alloc_dev_failed;
	}

	ts->input_dev->name = "Synaptics_RMI4";
	ts->input_dev->phys = client->name;

      // ts->f11_max_x = 1077;
       ts->f11_max_y = 1900;//ts->f11_max_y - 111;//ECID:0000 ZHANGZHAO 2012-5-19 increase touch key area
       min_x = 0;
       min_y = 0;
	   
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);

	/* set dummy key to make driver work with virtual keys */
	input_set_capability(ts->input_dev, EV_KEY, KEY_PROG1);


	set_bit(ABS_MT_PRESSURE, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	//set_bit(ABS_MT_WIDTH_MAJOR, ts->input_dev->absbit);

	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);

	if (ts->hasF11) {

	printk(KERN_DEBUG "%s: Set ranges X=[%d..%d] Y=[%d..%d].", __func__, min_x, ts->f11_max_x, min_y, ts->f11_max_y);


       /*set dummy key to make driver work with virtual keys*/
	input_set_capability(ts->input_dev, EV_KEY, KEY_PROG1);
	
	input_set_abs_params(ts->input_dev, ABS_X, min_x, ts->f11_max_x,0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, min_y, ts->f11_max_y,0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

#ifdef CONFIG_SYNA_MULTI_TOUCH
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	//input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MINOR, 0, 15, 0, 0);
	//input_set_abs_params(ts->input_dev, ABS_MT_ORIENTATION, 0, 1, 0, 0);
	//input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 1, 10, 0, 0);
	//input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);     

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, min_x, ts->f11_max_x,0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, min_y, ts->f11_max_y,0, 0);
#endif
		for (i = 0; i < ts->f11.points_supported; ++i) {

#ifdef CONFIG_SYNA_MULTI_TOUCH
			/* Linux 2.6.31 multi-touch */
                     input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 1, ts->f11.points_supported, 0, 0);
#endif

		}
		if (ts->hasEgrPalmDetect)
			set_bit(BTN_DEAD, ts->input_dev->keybit);
		if (ts->hasEgrFlick) {
			set_bit(REL_X, ts->input_dev->keybit);
			set_bit(REL_Y, ts->input_dev->keybit);
		}
		if (ts->hasEgrSingleTap)
			set_bit(BTN_TOUCH, ts->input_dev->keybit);
		if (ts->hasEgrDoubleTap)
			set_bit(BTN_TOOL_DOUBLETAP, ts->input_dev->keybit);
	}
	if (ts->hasF19) {
		set_bit(BTN_DEAD, ts->input_dev->keybit);
#ifdef CONFIG_SYNA_BUTTONS
		/* F19 does not (currently) report ABS_X but setting maximum X is a convenient way to indicate number of buttons */
		input_set_abs_params(ts->input_dev, ABS_X, 0, ts->f19.points_supported, 0, 0);
		for (i = 0; i < ts->f19.points_supported; ++i) {
			set_bit(BTN_F19 + i, ts->input_dev->keybit);
		}
#endif

#ifdef CONFIG_SYNA_BUTTONS_SCROLL
		set_bit(EV_REL, ts->input_dev->evbit);
		set_bit(SCROLL_ORIENTATION, ts->input_dev->relbit);
#endif
	}
	if (ts->hasF30) {
		for (i = 0; i < ts->f30.points_supported; ++i) {
			set_bit(BTN_F30 + i, ts->input_dev->keybit);
		}
	}


	/*
	 * Device will be /dev/input/event#
	 * For named device files, use udev
	 */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "synaptics_rmi4_probe: Unable to register %s \
				input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	} 
	else {
		printk("synaptics input device registered\n");
	}
              
	if (client->irq) {
              printk("Requesting IRQ...\n");
		//client->irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PC6);
			
		if (request_irq(client->irq, synaptics_ts_irq_handler,
				IRQF_TRIGGER_LOW, client->name, ts) >= 0) 
			{
  			      printk(KERN_ERR "Received IRQ!\n");
  			      ts->use_irq = 1;
  			     // if (set_irq_wake(client->irq, 1) < 0)
  				//printk(KERN_ERR "failed to set IRQ wake\n");
  		       } 
		else
			{
			    printk("Failed to request IRQ!\n");
		       }
	}
              
	if (!ts->use_irq) {
		printk(KERN_ERR "Synaptics RMI4 device %s in polling mode!\n", client->name);
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}


	ts->enable = 1;

	dev_set_drvdata(&ts->input_dev->dev, ts);
//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7
       dir = proc_mkdir("touchscreen", NULL);
	refresh = create_proc_entry("ts_information", 0644, dir);
	if (refresh) {
		refresh->data		= NULL;
		//refresh->read_proc  = proc_read_val;
		refresh->write_proc = proc_write_val;
	}
//[ecid:0000] ztebsp zhangzhao add for firmware update 2012-6-7 end
	#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
	#endif

    virtual_keys_init();
	  // synaptics_i2c_write(ts->client, 0x78, 0x08);       //
	   synaptics_i2c_write(ts->client, (ts->f01.controlBase)+1, 0x04);       //only eanble abs INT
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_YOUHUA//ECID:0000 ZHANGZHAO 2012-5-19 ADD MACRO FOR SWISSCOM
	 synaptics_i2c_write(ts->client, (ts->f01.controlBase), 0x04);//ECID:0000 zhangzhao 2012-5-18 increase first INS  
#endif	 
          //synaptics_i2c_write(ts->client, (ts->f01.controlBase)+2, 0x01);       //ecid:0000 zhangzhao 2012-5-16 avoid shacking
	/*if (sysfs_create_file(&ts->input_dev->dev.kobj, &dev_attr_synaptics_rmi4_enable.attr) < 0)
		printk("failed to create sysfs file for input device\n");*/   //shihuiqin

#ifdef TOUCH_FIRMWARE_UPDATE /*[ECID:0000] zhangzhao 2012-6-20 add auto update firmware func*/
       update_syna_file();
#endif
	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_alloc_dev_failed:
err_pdt_read_failed:
err_detect_failed:
               
              if(ts != NULL)
              {
                        kfree(ts);
              }
err_check_functionality_failed:
err_regulator_get_failed:	
       regulator_put(tegra_tsc_regulator_2v8);

	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_rmi4 *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
       if(ts != NULL)
          {
             kfree(ts);
           }
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    int ret;
	struct synaptics_rmi4 *ts = i2c_get_clientdata(client);
	printk(KERN_ERR "%s  %d enter\n", __func__,ts->f01.controlBase);     


	if (ts->use_irq)
    {
		disable_irq_nosync(client->irq);
	}
	else
	{
		hrtimer_cancel(&ts->timer);
	}
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
	{
	    printk(KERN_ERR "cancel_work_sync ret=%d",ret);
	    enable_irq(client->irq);
	}
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_YOUHUA//ECID:0000 ZHANGZHAO 2012-5-19 ADD MACRO FOR SWISSCOM	   
//ecid:0000 zhangzhao 2012-5-18 increase INS
	 ret =  synaptics_i2c_write(ts->client, ts->f01.controlBase, 0x00);       //-----------------
	if (ret < 0)
        printk(KERN_ERR "synaptics_ts_suspend: 111111111 failed\n");
#endif
	
    ret = synaptics_i2c_write(ts->client, (ts->f01.controlBase)+1, 0);     /* disable interrupt, */
	if (ret < 0)
        printk(KERN_ERR "synaptics_ts_suspend: 22222synaptics_i2c_write failed\n");

    ret = synaptics_i2c_write(client, ts->f01.controlBase, 0x01);      /* deep sleep */
	if (ret < 0)
        printk(KERN_ERR "synaptics_ts_suspend: 33333synaptics_i2c_write failed\n");
	
	ts->enable = 0;
//ecid:0000 zhangzhao 2012-5-18 increase INS

	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
    struct synaptics_rmi4 *ts = i2c_get_clientdata(client);

	printk(KERN_ERR "%s %d enter\n", __func__,ts->f01.controlBase);     

       synaptics_i2c_write(client, ts->f01.controlBase, 0x0);      /* wakeup */

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_YOUHUA//ECID:0000 ZHANGZHAO 2012-5-19 ADD MACRO FOR SWISSCOM	   
	 synaptics_i2c_write(client, ts->f01.controlBase, 0x04);       //ecid:0000 zhangzhao 2012-5-18 increase INS
#endif
	if (ts->use_irq)
	{
	    enable_irq(ts->client->irq);
	    synaptics_i2c_write(ts->client, (ts->f01.controlBase)+1, 4);     /* enable interrupt, */
	}
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	ts->enable = 1;
//ecid:0000 zhangzhao 2012-6-12  reset panle every resume
	synaptics_i2c_write(ts->client,ts->f01.commandlBase,0x01);//ecid:0000 zhangzhao 2012-6-12  reset panle every resume
	printk(KERN_ERR "%s %d rest the panel --\n", __func__,ts->f01.commandlBase);     
       msleep(2);	  
//ecid:0000 zhangzhao 2012-6-12  reset panle every resume end	   
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_rmi4 *ts;
	ts = container_of(h, struct synaptics_rmi4, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_rmi4 *ts;
	ts = container_of(h, struct synaptics_rmi4, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ "synaptics-rmi-ts", 0 },
	{ }
};

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= "synaptics-rmi-ts",
	},
};

static int __devinit synaptics_ts_init(void)
{
	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	//synaptics_wq = create_rt_workqueue("synaptics_wq");
	if (!synaptics_wq)
		{
	         printk(KERN_ERR "tsc mouduel inti error\n");		
		  return -ENOMEM;
		}
	printk(KERN_ERR "tsc mouduel inti ok\n");		
		
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");
