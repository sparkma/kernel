/*
 * ov7692.c - ov7692 sensor driver
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * Contributors:
 *      Abhinav Sinha <absinha@nvidia.com>
 *
 * Leverage OV2710.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

/**
 * SetMode Sequence for 640x480. Phase 0. Sensor Dependent.
 * This sequence should put sensor in streaming mode for 640x480
 * This is usually given by the FAE or the sensor vendor.
 */
 
/******************************************************
*ZTEBSP yuxin create this file for ov7692 sensor,2011.11.15
*******************************************************/

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/ov7692.h>

struct ov7692_reg {
    u8 addr;
    u8 val;
};

struct ov7692_info {
    int mode;
    struct i2c_client *i2c_client;
    struct ov7692_platform_data *pdata;
};

#define OV7692_TABLE_WAIT_MS 0
#define OV7692_TABLE_END     0xfe  //yuxin modify for not the same with the reg addr,2012.01.04
#define OV7692_MAX_RETRIES 3
/*[ECID:0000]ZTEBSP yuxin add for ov7692 WB,Color effect,exposure,2011.11.15 ++*/
static struct ov7692_reg ov7692_Whitebalance_Auto[] = {
  {0x13, 0xf7},
  {OV7692_TABLE_END, 0x00}
};
//白炽
static struct ov7692_reg ov7692_Whitebalance_Incandescent[] = {
{0x13, 0xf5},
{0x01, 0x62},
{0x02, 0x40},
{0x03, 0x41},
{OV7692_TABLE_END, 0x00}
};
// 日光 
static struct ov7692_reg ov7692_Whitebalance_Daylight[] = {
{0x13, 0xf5},
{0x01, 0x44},
{0x02, 0x55},
{0x03, 0x40},
{OV7692_TABLE_END, 0x00}
};
// 荧光
static struct ov7692_reg ov7692_Whitebalance_Fluorescent[] = {
{0x13, 0xf5},
{0x01, 0x5b},
{0x02, 0x4c},
{0x03, 0x40},
{OV7692_TABLE_END, 0x00}
};
// 阴天
static struct ov7692_reg ov7692_Whitebalance_Cloudy[] = {
{0x13, 0xf5},
{0x01, 0x40},
{0x02, 0x5d},
{0x03, 0x40},
{OV7692_TABLE_END, 0x00}
};

static struct ov7692_reg ov7692_exp_negative2[] = {
 // -1.7EV
 
    {OV7692_TABLE_END, 0x00}
};
static struct ov7692_reg ov7692_exp_negative1[] = {
  // -1.0EV

    {OV7692_TABLE_END, 0x00}
};
static struct ov7692_reg ov7692_exp_zero[] = {

    {OV7692_TABLE_END, 0x00}
};
static struct ov7692_reg ov7692_exp_one[] = {

   
    {OV7692_TABLE_END, 0x00}
};
static struct ov7692_reg ov7692_exp_two[] = {
    
    {OV7692_TABLE_END, 0x00}
};

static struct ov7692_reg ov7692_ColorEffect_None[] = { 
  {0xd2, 0x06},
  {OV7692_TABLE_END, 0x00}
};

static struct ov7692_reg ov7692_ColorEffect_Mono[] = {
  {0xd2, 0x1e},
  {0xda, 0x80},
  {0xdb, 0x80},
  {OV7692_TABLE_END, 0x00}
};

static struct ov7692_reg ov7692_ColorEffect_Sepia[] = {
  {0xd2, 0x1e},
  {0xda, 0x40},
  {0xdb, 0xa0},
  {OV7692_TABLE_END, 0x00}
};

static struct ov7692_reg ov7692_ColorEffect_Negative[] = {
  {0xd2, 0x46},
  {OV7692_TABLE_END, 0x00}
};

static struct ov7692_reg ov7692_ColorEffect_Solarize[] = {
  {OV7692_TABLE_END, 0x00}
};

//Sensor ISP Not Support this function
static struct ov7692_reg ov7692_ColorEffect_Posterize[] = {
  {OV7692_TABLE_END, 0x00}
};

//Brightness -4
static struct ov7692_reg ov7692_Brightness_level0[] = {
  {0xd3, 0x40},
  {0xdc, 0x08},
  {OV7692_TABLE_END, 0x00}
};
//Brightness -2
static struct ov7692_reg ov7692_Brightness_level1[] = {
  {0xd3, 0x20},
  {0xdc, 0x08},
  {OV7692_TABLE_END, 0x00}
};
//Default Brightness
static struct ov7692_reg ov7692_Brightness_level2[] = {
  {0xd3, 0x00},
  {0xdc, 0x00},
  {OV7692_TABLE_END, 0x00}
};
//Brightness +2
static struct ov7692_reg ov7692_Brightness_level3[] = {
  {0xd3, 0x40},
  {0xdc, 0x00},
  {OV7692_TABLE_END, 0x00}
};
//Brightness +4
static struct ov7692_reg ov7692_Brightness_level4[] = {
  {0xd3, 0x80},
  {0xdc, 0x00},
  {OV7692_TABLE_END, 0x00}
};


//Contrast -4
static struct ov7692_reg ov7692_Contrast_level0[] = {
  {0xd4, 0x15},
  {0xdc, 0x04},
  {OV7692_TABLE_END, 0x00}
};
//Contrast -2
static struct ov7692_reg ov7692_Contrast_level1[] = {
  {0xd4, 0x1B},
  {0xdc, 0x04},
  {OV7692_TABLE_END, 0x00}
};
//Default Contrast
static struct ov7692_reg ov7692_Contrast_level2[] = {
  {0xd4, 0x20},
  {0xdc, 0x00},
  {OV7692_TABLE_END, 0x00}
};
//Contrast +2
static struct ov7692_reg ov7692_Contrast_level3[] = {
  {0xd4, 0x28},
  {0xdc, 0x00},
  {OV7692_TABLE_END, 0x00}
};
//Contrast +4
static struct ov7692_reg ov7692_Contrast_level4[] = {
  {0xd3, 0x30},
  {0xdc, 0x00},
  {OV7692_TABLE_END, 0x00}
};


//Saturation -4
static struct ov7692_reg ov7692_Saturation_level0[] = {
  {0xd8, 0x20},
  {0xd9, 0x20},
  {OV7692_TABLE_END, 0x00}
};
//Saturation -2
static struct ov7692_reg ov7692_Saturation_level1[] = {
  {0xd8, 0x33},
  {0xd9, 0x33},
  {OV7692_TABLE_END, 0x00}
};
//Default Saturation
//OV default is 0x40
static struct ov7692_reg ov7692_Saturation_level2[] = {
  {0xd8, 0x50},
  {0xd9, 0x50},
  {OV7692_TABLE_END, 0x00}
};
//Saturation +2
static struct ov7692_reg ov7692_Saturation_level3[] = {
  {0xd8, 0x60},
  {0xd9, 0x60},
  {OV7692_TABLE_END, 0x00}
};
//Saturation +4
static struct ov7692_reg ov7692_Saturation_level4[] = {
  {0xd8, 0x80},
  {0xd9, 0x80},
  {OV7692_TABLE_END, 0x00}
};
/*[ECID:0000]ZTEBSP yuxin add for ov7692 WB,Color effect,exposure,2011.11.15 --*/
/*
@@ YUV_VGA_30fps_MIPI

;=============================================================
;Notes:
;
;1. Blacksun setting
;	for AVDD = 2.65 - 3.0V	--> Reg0x4c=0x7d
;	for AVDD = 2.60 - 2.7V	--> Reg0x4c=0x77
;
;2. When using Flip, pelase beware that 0x22 has to be changes as well
;	Sub-sampled OFF
;		Flip on		0x22[5:4] = 2'b00
;		Flip off	0x22[5:4] = 2'b00
;	Sub-sampled ON
;		Flip on		0x22[5:4] = 2'b10
;		Flip off	0x22[5:4] = 2'b01	
;	
;=============================================================
*/
static struct ov7692_reg mode_640x480[] = {


//2011.12.02参数++
  {OV7692_TABLE_WAIT_MS, 1},
  {0x12, 0x80},
// delay 5ms
  {OV7692_TABLE_WAIT_MS, 5},
  {0x0e, 0x08},
  {0x69, 0x52},
  {0x1e, 0xb3},
  {0x48, 0x42},
  {0xff, 0x01},
  {0xae, 0xa0},
  {0xa8, 0x26},
  {0xb4, 0xc0},
  {0xb5, 0x40},
  {0xff, 0x00},
   /*ZTEBSP yuxin modify for front cam preview correctly,
  after modify preview and snapshot orientation are different,2011.12.27*/
  {0x0c, 0x10}, //0x00 changed YUV order
  //{0x0c, 0xD0},
  //{0x0c, 0x90}, // flip-bit[7],mirror-bit[6]=1   
  {0x62, 0x10},
  {0x12, 0x00},
  {0x17, 0x65},
  {0x18, 0xa4},
  {0x19, 0x0a},
  {0x1a, 0xf6},
  {0x3e, 0x30},
  {0x64, 0x0a},
  {0xff, 0x01},
  {0x80, 0x24},
  {0xb4, 0xc0},
  {0xff, 0x00},
  {0x67, 0x20},
  {0x81, 0x3f},
  {0xcc, 0x02},
  {0xcd, 0x80},
  {0xce, 0x01},
  {0xcf, 0xe0},
  {0xc8, 0x02},
  {0xc9, 0x80},
  {0xca, 0x01},
  {0xcb, 0xe0},
  {0xd0, 0x48},
  {0x82, 0x03},
  //{0x0e, 0x00},//yuxin del 12.13  
  {0x70, 0x00},
  {0x71, 0x34},
  {0x74, 0x28},
  {0x75, 0x98},
  {0x76, 0x00},
  {0x77, 0x64},
  {0x78, 0x01},
  {0x79, 0xc2},
  {0x7a, 0x4e},
  {0x7b, 0x1f},
  {0x7c, 0x00},
  {0x11, 0x00},
  //{0x11, 0x01},
  //{0x31, 0x83},   //yuxin modify for 30fps,12.09
  //{0x31, 0x87},   //yuxin modify for 15fps,12.09
  {0x20, 0x00},
  {0x21, 0x23},
  {0x50, 0x9a},
  {0x51, 0x80},
  //{0x4c, 0x77},
  {0x4c, 0x7d},   //yuxin modify 12.13 消除曝光
  //{0x0e, 0x02},//yuxin del 12.13  
  {0x41, 0x43}, //yuxin add for modify pic quanlity,11.28
  {0x80, 0x7f},
  //{0x15, 0xb4},//auto frame rate control
  {0x15, 0xb0},  //yuxin modify 12.13
  //lens correction
  {0x85, 0x90},
  {0x86, 0x10},
  {0x87, 0x00},
  {0x88, 0x10},
  {0x89, 0x18},
  {0x8a, 0x10},
  {0x8b, 0x14},
// color matrix
  {0xbb, 0x80},
  {0xbc, 0x62},
  {0xbd, 0x1e},
  {0xbe, 0x26},
  {0xbf, 0x7b},
  {0xc0, 0xac},
  {0xc1, 0x1e},
  //edge/denoise/exposure
  {0xb4, 0x06},                  
  {0xb5, 0x05}, //auto, no meaning
  {0xb6, 0x00}, //auto, no meaning
  {0xb7, 0x02},
  {0xb8, 0x0b},
  {0xb9, 0x00},
  {0xba, 0x18},
// UV adjust
  {0x81, 0xff},                  
  {0x5A, 0x4A},
  {0x5B, 0x9F},
  {0x5C, 0x48},
  {0x5d, 0x32},
// AE target
  {0x24, 0x88},
  //{0x25, 0x78},
  {0x25, 0x7A},//yuxin 12.13 提高AE速度
  //{0x26, 0xb3},
  {0x26, 0xc4},//yuxin 12.13提高AE速度
  //gamma
  {0xa3, 0x08},
  {0xa4, 0x15},
  {0xa5, 0x24},
  {0xa6, 0x45},
  {0xa7, 0x55},
  {0xa8, 0x6a},
  {0xa9, 0x78},
  {0xaa, 0x87},
  {0xab, 0x96},
  {0xac, 0xa3},
  {0xad, 0xb4},
  {0xae, 0xc3},
  {0xaf, 0xd6},
  {0xb0, 0xe6},
  {0xb1, 0xf2},
  {0xb2, 0x12},
  //awb
  {0x8e, 0x92},
  {0x96, 0xff},
  {0x97, 0x00},
  {0x8c, 0x5d},
  {0x8d, 0x11},
  {0x8e, 0x12},
  {0x8f, 0x11},
  {0x90, 0x50},
  {0x91, 0x22},
  {0x92, 0xd1},
  {0x93, 0xa7},
  {0x94, 0x23},
  {0x95, 0x3b},
  {0x96, 0xff},
  {0x97, 0x00},
  {0x98, 0x4a},
  {0x99, 0x46},
  {0x9a, 0x3d},
  {0x9b, 0x3a},
  {0x9c, 0xf0},
  {0x9d, 0xf0},
  {0x9e, 0xf0},
  {0x9f, 0xff},
  {0xa0, 0x56},
  {0xa1, 0x55},
  {0xa2, 0x13},
  {0x50, 0x4d},
  {0x51, 0x3f},
  {0x21, 0x57},
  {0x20, 0x00},
  {0x14, 0x29},
  {0x68, 0xb0},
  {0xd2, 0x07},
  {0xd3, 0x10},//yuxin add for reduce brightness 12.13
  {0xdc, 0x08},//yuxin add for reduce brightness 12.13
  {0xd8, 0x50},//yuxin add for increase saturation ,sat_u,12.14
  {0xd9, 0x50},//yuxin add for increase saturation ,sat_v,12.14
  {0x0e, 0x00},
// delay 60ms
  {OV7692_TABLE_WAIT_MS, 60},
  {OV7692_TABLE_END,0x00},
//2011.12.02参数--



};
      


enum {
    OV7692_MODE_640x480,
};

static struct ov7692_reg *mode_table[] = {
    [OV7692_MODE_640x480] = mode_640x480,
};




static int ov7692_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
        int err;
        struct i2c_msg msg;
        u8 data=addr;

        if (!client->adapter)
                return -ENODEV;
		
        /*
	 * Send out the register address...
	 */
       // msg[0].addr = client->addr;
       msg.addr = 0x3c;  //yuxin modify for write to ov7692 i2c addr 0x3c
        msg.flags = 0;
        msg.len = 1;
        msg.buf = &data;
		
	err=i2c_transfer(client->adapter, &msg, 1);
	if(err<0)
	{
	    printk(KERN_ERR "Error %d on ov7692 register write\n", err);
          return err;
	}
	
       /*
	 * ...then read back the result.
	 */
        msg.flags = I2C_M_RD;
	 err=i2c_transfer(client->adapter, &msg, 1);
       if(err>=0){
	        *val=data; 	
	          return 0;
	  }
        return err;
}

static int ov7692_write_reg(struct i2c_client *client, u8 addr, u8 val)
{
        int err;
        struct i2c_msg msg;
        unsigned char data[2]={addr,val};
        int retry = 0;

        if (!client->adapter)
               return -ENODEV;

       
        //msg.addr = client->addr;
        msg.addr = 0x3c;  //yuxin modify for write to ov7692 i2c addr 0x3c
        msg.flags = 0;
        msg.len = 2;
        msg.buf = data;

        do {
            err = i2c_transfer(client->adapter, &msg, 1);
            if (err == 1)
	      {
	      //printk("%s: i2c write success, reg 0x%x ,val 0x%x\n",
			//__func__, addr, val);
            return 0;
		}
            retry++;
            pr_err("ov7692: i2c transfer failed, retrying %x %x\n",
            addr, val);
            msleep(3);
        } while (retry <=OV7692_MAX_RETRIES);

        return err;
}

static int ov7692_write_table(struct i2c_client *client,
                const struct ov7692_reg table[],
                const struct ov7692_reg override_list[],
                int num_override_regs)
{
        int err;
        const struct ov7692_reg *next;
        int i;
        u16 val;

        for (next = table; next->addr != OV7692_TABLE_END; next++) {
            if (next->addr == OV7692_TABLE_WAIT_MS) {
            msleep(next->val);
            continue;
        }

        val = next->val;

        /* When an override list is passed in, replace the reg */
        /* value to write if the reg is in the list            */
        if (override_list) {
        for (i = 0; i < num_override_regs; i++) {
        if (next->addr == override_list[i].addr) {
        val = override_list[i].val;
        break;
                }
            }
        }

        err = ov7692_write_reg(client, next->addr, val);
            if (err)
            return err;
        }
        return 0; 
}

static int ov7692_set_mode(struct ov7692_info *info, struct ov7692_mode *mode)
{
        int sensor_mode;
        int err;
        u16 status;

     
        pr_info("%s: xres %u yres %u\n", __func__, mode->xres, mode->yres);
        if (mode->xres == 640 && mode->yres == 480)
            sensor_mode = OV7692_MODE_640x480;
        else {
            pr_err("%s: invalid resolution supplied to set mode %d %d\n",
                    __func__, mode->xres, mode->yres);
            return -EINVAL;
        }
     //  msleep(100);
       err = ov7692_write_table(info->i2c_client, mode_table[sensor_mode],
            NULL, 0);
	
        if (err)
            return err;

        info->mode = sensor_mode;
        return 0;
}

static int ov7692_get_status(struct ov7692_info *info,
            struct ov7692_status *dev_status)
{
    return 0;
}

static long ov7692_ioctl(struct file *file,
        unsigned int cmd, unsigned long arg)
{
        int err;
        struct ov7692_info *info = file->private_data;
        pr_info("yuv ov7692_ioctl cmd %d\n",cmd);
        switch (cmd) {
        case OV7692_IOCTL_SET_MODE:
        {
            struct ov7692_mode mode;
                 printk("%s:l case OV7692_IOCTL_SET_MODE\n",__func__);
            if (copy_from_user(&mode,
                        (const void __user *)arg,
                            sizeof(struct ov7692_mode))) {
                       printk(" %s:case OV7692_IOCTL_SET_MODE error \n",__func__);     
                    return -EFAULT;
                }

                return ov7692_set_mode(info, &mode);
        }
        case OV7692_IOCTL_SET_COLOR_EFFECT:
        {
            u8 coloreffect;

            if (copy_from_user(&coloreffect,
                (const void __user *)arg,
                sizeof(coloreffect))) {
                printk(" %s:case OV7692_IOCTL_SET_COLOR_EFFECT error \n",__func__);     
                return -EFAULT;
            }
            printk("yuv coloreffect %d\n",coloreffect);

            switch(coloreffect)
            {
                case OV7692_ColorEffect_None:
                    err = ov7692_write_table(info->i2c_client, ov7692_ColorEffect_None,NULL, 0);
                    break;
                case OV7692_ColorEffect_Mono:
                    err = ov7692_write_table(info->i2c_client, ov7692_ColorEffect_Mono,NULL, 0);
                    break;
                case OV7692_ColorEffect_Sepia:
                    err = ov7692_write_table(info->i2c_client, ov7692_ColorEffect_Sepia,NULL, 0);
                    break;
                case OV7692_ColorEffect_Negative:
                    err = ov7692_write_table(info->i2c_client, ov7692_ColorEffect_Negative,NULL, 0);
                    break;
                case OV7692_ColorEffect_Solarize:
                    err = ov7692_write_table(info->i2c_client, ov7692_ColorEffect_Solarize,NULL, 0);
                    break;
                case OV7692_ColorEffect_Posterize:
                err = ov7692_write_table(info->i2c_client, ov7692_ColorEffect_Posterize,NULL, 0);
                    break;
                default:
                    break;
            }

            if (err)
            {
            pr_info("yuv coloreffect ERR%d\n");
            return err;
            }
            /* ZTE: add by yaoling for switch effect  20110818 */
            msleep(50);
            /* ZTE: add by yaoling for switch effect  20110818 */
            return 0;
        }
         case OV7692_IOCTL_SET_WHITE_BALANCE:
        {
            u8 whitebalance;

            pr_info("yuv whitebalance %lu\n",arg);
            if (copy_from_user(&whitebalance,
                (const void __user *)arg,
                sizeof(whitebalance))) {
                pr_info("yuv whitebalance ERR\n");	   
                return -EFAULT;
            }
            pr_info("yuv whitebalance %d\n",whitebalance);

            switch(whitebalance)
            {
                case OV7692_Whitebalance_Auto:
                    err = ov7692_write_table(info->i2c_client, ov7692_Whitebalance_Auto,NULL, 0);
                    break;
                case OV7692_Whitebalance_Incandescent:
                    err = ov7692_write_table(info->i2c_client, ov7692_Whitebalance_Incandescent,NULL, 0);
                    break;
                case OV7692_Whitebalance_Daylight:
                    err = ov7692_write_table(info->i2c_client, ov7692_Whitebalance_Daylight,NULL, 0);
                    break;
                case OV7692_Whitebalance_Fluorescent:
                    err = ov7692_write_table(info->i2c_client, ov7692_Whitebalance_Fluorescent,NULL, 0);
                    break;
		   case OV7692_Whitebalance_Cloudy:
                    err = ov7692_write_table(info->i2c_client, ov7692_Whitebalance_Cloudy,NULL, 0);
                    break;
                default:
                    break;
            }

            if (err)
            {
                pr_info("yuv whitebalance set err %d\n",err);
                return err;
            }

            return 0;
        }
      

         case OV7692_IOCTL_SET_EXPOSURE:
        {
            pr_info("yuv EXP %lu\n",arg);
            switch(arg)
            {
                case OV7692_Exposure_0:
                    pr_info("yuv SET_EXPOSURE 0\n");
                    err = ov7692_write_table(info->i2c_client, ov7692_exp_zero,NULL, 0);
                    break;
                case OV7692_Exposure_1:
                    pr_info("yuv SET_EXPOSURE 1\n");
                    err = ov7692_write_table(info->i2c_client,ov7692_exp_one,NULL, 0);
                    break;
                case OV7692_Exposure_2:
                    pr_info("yuv SET_EXPOSURE 2\n");
                    err = ov7692_write_table(info->i2c_client, ov7692_exp_two,NULL, 0);
                    break;
                case OV7692_Exposure_Negative_1:
                    pr_info("yuv SET_EXPOSURE -1\n");
                    err = ov7692_write_table(info->i2c_client, ov7692_exp_negative1,NULL, 0);
                    break;
                case OV7692_Exposure_Negative_2:
                    pr_info("yuv SET_EXPOSURE -2\n");
                    err = ov7692_write_table(info->i2c_client, ov7692_exp_negative2,NULL, 0);
                    break;
                default:
                    break;
            }
             if (err)
            {
                pr_info("yuv exp set err %d\n",err);
                return err;
            }

            return 0;
            
        }
       /* ZTE: add by yaoling for exposure function 20110812 -- */
	   
	/*ztebsp yuxin add for ov7692 effect,2011.12.14 ++*/

	case OV7692_IOCTL_SET_CONTRAST:
	{
          u8 contrast;
	   pr_info("yuv contrast %lu\n",arg);
            if (copy_from_user(&contrast,
                (const void __user *)arg,
                sizeof(contrast))) {
                pr_info("yuv contrast ERR\n");	   
                return -EFAULT;
            }
            pr_info("yuv contrast %d\n",contrast);

         switch(contrast)
         {
                case OV7692_Contrast_0:
                    err = ov7692_write_table(info->i2c_client, ov7692_Contrast_level0,NULL, 0);
                    break;
                case OV7692_Contrast_1:
                    err = ov7692_write_table(info->i2c_client, ov7692_Contrast_level1,NULL, 0);
                    break;
                case OV7692_Contrast_2:
                    err = ov7692_write_table(info->i2c_client, ov7692_Contrast_level2,NULL, 0);
                    break;
                case OV7692_Contrast_3:
                    err = ov7692_write_table(info->i2c_client, ov7692_Contrast_level3,NULL, 0);
                    break;
		   case OV7692_Contrast_4:
                    err = ov7692_write_table(info->i2c_client, ov7692_Contrast_level4,NULL, 0);
                    break;
		   	
                default:
                    break;
         }

		 if (err)
            {
                pr_info("yuv contrast set err %d\n",err);
                return err;
            }

            return 0;
	}
		
	case OV7692_IOCTL_SET_SATURATION:
	{
          u8 saturation;
	   pr_info("yuv saturation %lu\n",arg);
            if (copy_from_user(&saturation,
                (const void __user *)arg,
                sizeof(saturation))) {
                pr_info("yuv saturation ERR\n");	   
                return -EFAULT;
            }
            pr_info("yuv saturation %d\n",saturation);

         switch(saturation)
         {
                case OV7692_Saturation_0:
                    err = ov7692_write_table(info->i2c_client, ov7692_Saturation_level0,NULL, 0);
                    break;
                case OV7692_Saturation_1:
                    err = ov7692_write_table(info->i2c_client, ov7692_Saturation_level1,NULL, 0);
                    break;
                case OV7692_Saturation_2:
                    err = ov7692_write_table(info->i2c_client, ov7692_Saturation_level2,NULL, 0);
                    break;
                case OV7692_Saturation_3:
                    err = ov7692_write_table(info->i2c_client, ov7692_Saturation_level3,NULL, 0);
                    break;
		   case OV7692_Saturation_4:
                    err = ov7692_write_table(info->i2c_client, ov7692_Saturation_level4,NULL, 0);
                    break;
		   	
                default:
                    break;
         }

		 if (err)
            {
                pr_info("yuv saturation set err %d\n",err);
                return err;
            }

            return 0;
	} 

	case OV7692_IOCTL_SET_BRIGHTNESS:
	{
          u8 brightness;
	   pr_info("yuv brightness %lu\n",arg);
            if (copy_from_user(&brightness,
                (const void __user *)arg,
                sizeof(brightness))) {
                pr_info("yuv brightness ERR\n");	   
                return -EFAULT;
            }
            pr_info("yuv brightness %d\n",brightness);

         switch(brightness)
         {
                case OV7692_Brightness_0:
                    err = ov7692_write_table(info->i2c_client, ov7692_Brightness_level0,NULL, 0);
                    break;
                case OV7692_Brightness_1:
                    err = ov7692_write_table(info->i2c_client, ov7692_Brightness_level1,NULL, 0);
                    break;
                case OV7692_Brightness_2:
                    err = ov7692_write_table(info->i2c_client, ov7692_Brightness_level2,NULL, 0);
                    break;
                case OV7692_Brightness_3:
                    err = ov7692_write_table(info->i2c_client, ov7692_Brightness_level3,NULL, 0);
                    break;
		   case OV7692_Brightness_4:
                    err = ov7692_write_table(info->i2c_client, ov7692_Brightness_level4,NULL, 0);
                    break;
		   	
                default:
                    break;
         }

		 if (err)
            {
                pr_info("yuv Brightness set err %d\n",err);
                return err;
            }

            return 0;
	} 
		
       /*ztebsp yuxin add for ov7692 effect,2011.12.14 --*/ 
        case OV7692_IOCTL_GET_STATUS:
        {
                return 0;
        }
        default:
             return -EFAULT;


        }
     
}

static struct ov7692_info *info;

static int ov7692_open(struct inode *inode, struct file *file)
{
        struct ov7692_status dev_status;
        int err;
        pr_info("ov7692_open\n");
        file->private_data = info;
        if (info->pdata && info->pdata->power_on)
            info->pdata->power_on();

        dev_status.data = 0;
        dev_status.status = 0;
        err = ov7692_get_status(info, &dev_status);
        return err;
}

int ov7692_release(struct inode *inode, struct file *file)
{
        pr_info("ov7692_release\n");
        if (info->pdata && info->pdata->power_off)
                info->pdata->power_off();
        file->private_data = NULL;
        return 0;
}

static const struct file_operations ov7692_fileops = {
        .owner = THIS_MODULE,
        .open = ov7692_open,
        .unlocked_ioctl = ov7692_ioctl,
        .release = ov7692_release,
};

static struct miscdevice ov7692_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "ov7692",
        .fops = &ov7692_fileops,
};

static int ov7692_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
        int err;

        pr_info("ov7692: probing sensor.\n");

        info = kzalloc(sizeof(struct ov7692_info), GFP_KERNEL);
        if (!info) {
                pr_err("ov7692: Unable to allocate memory!\n");
                return -ENOMEM;
        }

        err = misc_register(&ov7692_device);
        if (err) {
                pr_err("ov7692: Unable to register misc device!\n");
                kfree(info);
                return err;
        }

        info->pdata = client->dev.platform_data;
        info->i2c_client = client;

        i2c_set_clientdata(client, info);
        return 0;
}

static int ov7692_remove(struct i2c_client *client)
{
        struct ov7692_info *info;
        info = i2c_get_clientdata(client);
        misc_deregister(&ov7692_device);
        kfree(info);
        return 0;
}

static const struct i2c_device_id ov7692_id[] = {
        { "ov7692", 0 },
        { },
};

MODULE_DEVICE_TABLE(i2c, ov7692_id);

static struct i2c_driver ov7692_i2c_driver = {
        .driver = {
                .name = "ov7692",
                .owner = THIS_MODULE,
        },
        .probe = ov7692_probe,
        .remove = ov7692_remove,
        .id_table = ov7692_id,
};

static int __init ov7692_init(void)
{
        pr_info("ov7692 sensor driver loading\n");
        return i2c_add_driver(&ov7692_i2c_driver);
}

static void __exit ov7692_exit(void)
{
        i2c_del_driver(&ov7692_i2c_driver);
}

module_init(ov7692_init);
module_exit(ov7692_exit);
