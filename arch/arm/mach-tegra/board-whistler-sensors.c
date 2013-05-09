/*
 * arch/arm/mach-tegra/board-whistler-sensors.c
 *
 * Copyright (c) 2010-2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <mach/gpio.h>
#include <media/ov5650.h>
#include <media/soc380.h>
/*[ECID:0000]ZTEBSP YUXIN START, for new camera 20110921 */
#include <media/ov5640.h>
#include <media/ov7692.h>   //yuxin add for ov7692,2011.11.15
/*[ECID:0000]ZTEBSP YUXIN END, for new camera 20110921 */
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/adt7461.h>
#include <generated/mach-types.h>
#include <linux/gpio.h>
#include <linux/i2c/pca953x.h>
/*[ECID:000000] ZTEBSP wangbing 20111020, add temperature sense driver */
#include <linux/nct1008.h>  
//[ECID:000000] ZTEBSP wanghaifei start 20120220, add for sensor
#include <linux/mpu.h>
#include <linux/taos_common.h>
//[ECID:000000] ZTEBSP wanghaifei end 20120220, add for sensor

#include <mach/tegra_odm_fuses.h>
#include "cpu-tegra.h"

#include "gpio-names.h"
/*[ECID:000000] ZTEBSP zhangbo 20111123, nfc pn544, start*/
#define CONFIG_PN544_NFC
#ifdef CONFIG_PN544_NFC
#include<linux/nfc/pn544.h>
#endif
/*[ECID:000000] ZTEBSP zhangbo 20111123, nfc pn544, end*/

/*[ECID:0000]ZTEBSP YUXIN START */
/* ZTE: added by yuxin for new camera gpio pins 20110921 ++ */
#include "board-whistler.h"
/* ZTE: added by yuxin for new camera gpio pins20110921 ++ */
/* ZTE: del by yuxin for ov5640 gpio pins 20110921 ++ */
/*
#define CAMERA1_PWDN_GPIO		TEGRA_GPIO_PT2
#define CAMERA1_RESET_GPIO		TEGRA_GPIO_PD2
#define CAMERA2_PWDN_GPIO		TEGRA_GPIO_PBB5
#define CAMERA2_RESET_GPIO		TEGRA_GPIO_PBB1
#define CAMERA_AF_PD_GPIO		TEGRA_GPIO_PT3
#define CAMERA_FLASH_EN1_GPIO	TEGRA_GPIO_PBB4
#define CAMERA_FLASH_EN2_GPIO	TEGRA_GPIO_PA0
*/
/* ZTE: del by yuxin for ov5640 gpio pins 20110921 -- */

/* ZTE: added by yuxin for ov5640 gpio pins 20110921 ++ */
#define CAMERA1_PWDN_GPIO		TEGRA_GPIO_PBB5
#define CAMERA1_RESET_GPIO		TEGRA_GPIO_PD2
#define CAMERA_FLASH_EN_GPIO	       TEGRA_GPIO_PBB4
/*[ECID:0000]ZTEBSP yuxin add for cam sensor ov7692 PWDN pin 20111115*/
#define CAMERA2_PWDN_GPIO              TEGRA_GPIO_PBB1  
/* ZTE: added by yuxin for ov5640 gpio pins 20110921 -- */
/*[ECID:0000]ZTEBSP YUXIN END*/
#define TCA6416_GPIO_BASE		(TEGRA_NR_GPIOS)
#define FUSE_POWER_EN_GPIO		(TCA6416_GPIO_BASE + 2)

#define ADXL34X_IRQ_GPIO		TEGRA_GPIO_PAA1
#define ISL29018_IRQ_GPIO		TEGRA_GPIO_PK2
#define ADT7461_IRQ_GPIO		TEGRA_GPIO_PI2
/*[ECID:000000] ZTEBSP wanghaifei start 20111021, add power and irq control for tmd2771*/
#define TMD2771_IRQ_GPIO                TEGRA_GPIO_PG1 //WCDMA
/*[ECID:000000] ZTEBSP wanghaifei end 20111021*/


/*[ECID:000000] ZTEBSP wangbing 20111020, add temperature sense driver */
#define NCT1008_THERM2_GPIO	TEGRA_GPIO_PK6 

//extern void tegra_throttling_enable(bool enable);

static struct regulator *reg_avdd_cam1; /* LDO9 */
static struct regulator *reg_vdd_af;    /* LDO13 */
static struct regulator *reg_vdd_mipi;  /* LDO17 */
static struct regulator *reg_vddio_vi;  /* LDO18 */
/*[ECID:000000] ZTEBSP wangbing 20111020, add temperature sense driver */
static struct regulator *reg_vcc_therm; /* LDO9 */ 

static int whistler_camera_init(void)
{
/*[ECID:0000]ZTEBSP YUXIN START */
/* ZTE: added by yuxin for new camera pin init20110921 ++ */
        int ret=0;
        printk(KERN_ERR "yuxin whistler_camera_init\n");
        tegra_gpio_enable(CAMERA1_PWDN_GPIO);
	  ret=gpio_request(CAMERA1_PWDN_GPIO, "camera1_powerdown");
        if(ret)
   	{
   	   gpio_free(CAMERA1_PWDN_GPIO);
   	   printk("%s:can't request the gpio for cam1 PWDN",__func__);
   	   return ret	;
   	}
        gpio_direction_output(CAMERA1_PWDN_GPIO, 0);
        gpio_export(CAMERA1_PWDN_GPIO, false);

        tegra_gpio_enable(CAMERA1_RESET_GPIO);
        ret=gpio_request(CAMERA1_RESET_GPIO, "camera1_reset");
	if(ret)
   	{
   	   gpio_free(CAMERA1_RESET_GPIO);
   	   printk("%s:can't request the gpio for cam1 RESET",__func__);
   	   return ret	;
   	}
        gpio_direction_output(CAMERA1_RESET_GPIO, 0);
        gpio_export(CAMERA1_RESET_GPIO, false);

        gpio_set_value(CAMERA1_PWDN_GPIO, 1);
        mdelay(5);
/* ZTE: added by yuxin for new camera pin init 20110921 -- */
/*[ECID:0000]ZTEBSP yuxin add for  sensor ov7692 PWDN pin 20111115 ++*/
         tegra_gpio_enable(CAMERA2_PWDN_GPIO);
	  ret=gpio_request(CAMERA2_PWDN_GPIO, "camera1_powerdown");
        if(ret)
   	{
   	   gpio_free(CAMERA2_PWDN_GPIO);
   	   printk("%s:can't request the gpio for cam2 PWDN",__func__);
   	   return ret	;
   	}
        gpio_direction_output(CAMERA2_PWDN_GPIO, 0);
        gpio_export(CAMERA2_PWDN_GPIO, false);
	  gpio_set_value(CAMERA2_PWDN_GPIO, 1);
	  mdelay(5);
	
/*[ECID:0000]ZTEBSP yuxin add for  sensor ov7692 PWDN pin 20111115 --*/

/* ZTE: added by yuxin for flash zi2848 enable 20111031 ++ */
       //enable the flash chip zi2848
     
       tegra_gpio_enable(CAMERA_FLASH_EN_GPIO);
       ret=gpio_request(CAMERA_FLASH_EN_GPIO, "zi2848_flash_enable");
	 if(ret)
   	{
   	   gpio_free(CAMERA_FLASH_EN_GPIO);
   	   printk("%s:can't request the gpio for flash EN ",__func__);
   	   return ret	;
   	}  
       gpio_direction_output(CAMERA_FLASH_EN_GPIO, 0);
       gpio_export(CAMERA_FLASH_EN_GPIO, false);
       gpio_set_value(CAMERA_FLASH_EN_GPIO,0);  //set the value as 0 
       
/* ZTE: added by yuxin for flash zi2848 enable 20111031  -- */
/* ZTE: del by yuxin for camera init 20110921 ++ */
#if 0
	tegra_gpio_enable(CAMERA2_PWDN_GPIO);
	gpio_request(CAMERA2_PWDN_GPIO, "camera2_powerdown");
	gpio_direction_output(CAMERA2_PWDN_GPIO, 0);
	gpio_export(CAMERA2_PWDN_GPIO, false);

	tegra_gpio_enable(CAMERA2_RESET_GPIO);
	gpio_request(CAMERA2_RESET_GPIO, "camera2_reset");
	gpio_direction_output(CAMERA2_RESET_GPIO, 0);
	gpio_export(CAMERA2_RESET_GPIO, false);

	tegra_gpio_enable(CAMERA_AF_PD_GPIO);
	gpio_request(CAMERA_AF_PD_GPIO, "camera_autofocus");
	gpio_direction_output(CAMERA_AF_PD_GPIO, 0);
	gpio_export(CAMERA_AF_PD_GPIO, false);

	tegra_gpio_enable(CAMERA_FLASH_EN1_GPIO);
	gpio_request(CAMERA_FLASH_EN1_GPIO, "camera_flash_en1");
	gpio_direction_output(CAMERA_FLASH_EN1_GPIO, 0);
	gpio_export(CAMERA_FLASH_EN1_GPIO, false);

	tegra_gpio_enable(CAMERA_FLASH_EN2_GPIO);
	gpio_request(CAMERA_FLASH_EN2_GPIO, "camera_flash_en2");
	gpio_direction_output(CAMERA_FLASH_EN2_GPIO, 0);
	gpio_export(CAMERA_FLASH_EN2_GPIO, false);

	gpio_set_value(CAMERA1_PWDN_GPIO, 1);
	mdelay(5);

#endif
/* ZTE: del by yuxin for camera init 20110921 -- */
	return 0;
}


/* ZTE: del by yuxin for  ov5640 20110921 ++ */
#if 0

static int whistler_ov5650_power_on(void)
{
	gpio_set_value(CAMERA1_PWDN_GPIO, 0);

	if (!reg_avdd_cam1) {
		reg_avdd_cam1 = regulator_get(NULL, "vdd_cam1");
		if (IS_ERR_OR_NULL(reg_avdd_cam1)) {
			pr_err("whistler_ov5650_power_on: vdd_cam1 failed\n");
			reg_avdd_cam1 = NULL;
			return PTR_ERR(reg_avdd_cam1);
		}
		regulator_enable(reg_avdd_cam1);
	}
	mdelay(5);

	if (!reg_vdd_mipi) {
		reg_vdd_mipi = regulator_get(NULL, "vddio_mipi");
		if (IS_ERR_OR_NULL(reg_vdd_mipi)) {
			pr_err("whistler_ov5650_power_on: vddio_mipi failed\n");
			reg_vdd_mipi = NULL;
			return PTR_ERR(reg_vdd_mipi);
		}
		regulator_enable(reg_vdd_mipi);
	}
	mdelay(5);

	if (!reg_vdd_af) {
		reg_vdd_af = regulator_get(NULL, "vdd_vcore_af");
		if (IS_ERR_OR_NULL(reg_vdd_af)) {
			pr_err("whistler_ov5650_power_on: vdd_vcore_af failed\n");
			reg_vdd_af = NULL;
			return PTR_ERR(reg_vdd_af);
		}
		regulator_enable(reg_vdd_af);
	}
	mdelay(5);

	gpio_set_value(CAMERA1_RESET_GPIO, 1);
	mdelay(10);
	gpio_set_value(CAMERA1_RESET_GPIO, 0);
	mdelay(5);
	gpio_set_value(CAMERA1_RESET_GPIO, 1);
	mdelay(20);
	gpio_set_value(CAMERA_AF_PD_GPIO, 1);

	return 0;
}

static int whistler_ov5650_power_off(void)
{
	gpio_set_value(CAMERA_AF_PD_GPIO, 0);
	gpio_set_value(CAMERA1_PWDN_GPIO, 1);
	gpio_set_value(CAMERA1_RESET_GPIO, 0);

	if (reg_avdd_cam1) {
		regulator_disable(reg_avdd_cam1);
		regulator_put(reg_avdd_cam1);
		reg_avdd_cam1 = NULL;
	}

	if (reg_vdd_mipi) {
		regulator_disable(reg_vdd_mipi);
		regulator_put(reg_vdd_mipi);
		reg_vdd_mipi = NULL;
	}

	if (reg_vdd_af) {
		regulator_disable(reg_vdd_af);
		regulator_put(reg_vdd_af);
		reg_vdd_af = NULL;
	}

	return 0;
}

static int whistler_soc380_power_on(void)
{
	gpio_set_value(CAMERA2_PWDN_GPIO, 0);

	if (!reg_vddio_vi) {
		reg_vddio_vi = regulator_get(NULL, "vddio_vi");
		if (IS_ERR_OR_NULL(reg_vddio_vi)) {
			pr_err("whistler_soc380_power_on: vddio_vi failed\n");
			reg_vddio_vi = NULL;
			return PTR_ERR(reg_vddio_vi);
		}
		regulator_set_voltage(reg_vddio_vi, 1800*1000, 1800*1000);
		mdelay(5);
		regulator_enable(reg_vddio_vi);
	}

	if (!reg_avdd_cam1) {
		reg_avdd_cam1 = regulator_get(NULL, "vdd_cam1");
		if (IS_ERR_OR_NULL(reg_avdd_cam1)) {
			pr_err("whistler_soc380_power_on: vdd_cam1 failed\n");
			reg_avdd_cam1 = NULL;
			return PTR_ERR(reg_avdd_cam1);
		}
		regulator_enable(reg_avdd_cam1);
	}
	mdelay(5);

	gpio_set_value(CAMERA2_RESET_GPIO, 1);
	mdelay(10);
	gpio_set_value(CAMERA2_RESET_GPIO, 0);
	mdelay(5);
	gpio_set_value(CAMERA2_RESET_GPIO, 1);
	mdelay(20);

	return 0;

}

static int whistler_soc380_power_off(void)
{
	gpio_set_value(CAMERA2_PWDN_GPIO, 1);
	gpio_set_value(CAMERA2_RESET_GPIO, 0);

	if (reg_avdd_cam1) {
		regulator_disable(reg_avdd_cam1);
		regulator_put(reg_avdd_cam1);
		reg_avdd_cam1 = NULL;
	}
	if (reg_vddio_vi) {
		regulator_disable(reg_vddio_vi);
		regulator_put(reg_vddio_vi);
		reg_vddio_vi = NULL;
	}

	return 0;
}

struct ov5650_platform_data whistler_ov5650_data = {
	.power_on = whistler_ov5650_power_on,
	.power_off = whistler_ov5650_power_off,
};

struct soc380_platform_data whistler_soc380_data = {
	.power_on = whistler_soc380_power_on,
	.power_off = whistler_soc380_power_off,
};

static int whistler_fuse_power_en(int enb)
{
	int ret;

	ret = gpio_request(FUSE_POWER_EN_GPIO, "fuse_power_en");
	if (ret) {
		pr_err("%s: gpio_request fail (%d)\n", __func__, ret);
		return ret;
	}

	ret = gpio_direction_output(FUSE_POWER_EN_GPIO, enb);
	if (ret) {
		pr_err("%s: gpio_direction_output fail (%d)\n", __func__, ret);
		return ret;
	}

	gpio_free(FUSE_POWER_EN_GPIO);
	return 0;
}

static struct pca953x_platform_data whistler_tca6416_data = {
	.gpio_base = TCA6416_GPIO_BASE,
};

static struct i2c_board_info whistler_i2c3_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650", 0x36),
		.platform_data = &whistler_ov5650_data,
	},
	{
		I2C_BOARD_INFO("ad5820", 0x0c),
	},
	{
		I2C_BOARD_INFO("soc380", 0x3C),
		.platform_data = &whistler_soc380_data,
	},
};
#endif
/* ZTE: del by yuxin for  ov5640 20110921 --*/

/* ZTE: added by yuxin for new camera power on ,off 20110921 ++ */
static int whistler_ov5640_power_on(void)
{
   printk(KERN_ERR "yuxin whistler_ov5640_power_on\n");
    gpio_set_value(CAMERA2_PWDN_GPIO, 1);//make the front cam pwr down
    mdelay(5); 
    gpio_set_value(CAMERA1_PWDN_GPIO, 1);
	   mdelay(5);  
    gpio_direction_output(CAM_1V8_GPIO_EN, 1);
    gpio_direction_output(CAM_2V8_GPIO_EN, 1);
	   mdelay(5);  
    gpio_set_value(CAMERA1_PWDN_GPIO, 0);
    mdelay(5);  
    mdelay(5);
    gpio_set_value(CAMERA1_RESET_GPIO, 1);
    mdelay(10);
    gpio_set_value(CAMERA1_RESET_GPIO, 0);
    mdelay(5);
    gpio_set_value(CAMERA1_RESET_GPIO, 1);
    mdelay(20);
 
    return 0;
}

static int whistler_ov5640_power_off(void)
{
       printk(KERN_ERR "yuxin whistler_ov5640_power_off\n");
        gpio_set_value(CAMERA1_PWDN_GPIO, 1);
        gpio_set_value(CAMERA1_RESET_GPIO, 0);
        gpio_direction_output(CAM_1V8_GPIO_EN, 0);
        gpio_direction_output(CAM_2V8_GPIO_EN, 0);

        return 0;
}

struct ov5640_platform_data whistler_ov5640_data = {
	.power_on = whistler_ov5640_power_on,
	.power_off = whistler_ov5640_power_off,
};

//[ECID:0000]ZTEBSP,yuxin add for cam ov7692 power on&off,2011.11.15,++ 
static int whistler_ov7692_power_on(void)
{
   printk(KERN_ERR "yuxin whistler_ov7692_power_on\n");
   /*
   *ZTEBSP yuxin add for shut down the ov5640 when open ov7692 
   *as the 2 sensors have the same I2C addr
   */
    gpio_set_value(CAMERA1_PWDN_GPIO, 1);    
	 mdelay(5); 
     gpio_set_value(CAMERA2_PWDN_GPIO, 1);    
	 mdelay(5);  
    gpio_direction_output(CAM_1V8_GPIO_EN, 1);
	mdelay(5);  
    gpio_direction_output(CAM_2V8_GPIO_EN, 1);
	 mdelay(5);  
    gpio_set_value(CAMERA2_PWDN_GPIO, 0);
    mdelay(10);  
    return 0;
}

static int whistler_ov7692_power_off(void)
{
         printk(KERN_ERR "yuxin whistler_ov7692_power_off\n");
        gpio_set_value(CAMERA2_PWDN_GPIO, 1);
        gpio_direction_output(CAM_1V8_GPIO_EN, 0);
        gpio_direction_output(CAM_2V8_GPIO_EN, 0);

        return 0;
}

struct ov7692_platform_data whistler_ov7692_data = {
	.power_on = whistler_ov7692_power_on,
	.power_off = whistler_ov7692_power_off,
};
//[ECID:0000]ZTEBSP,yuxin add for cam ov7692 power on&off,2011.11.15,--

static struct i2c_board_info whistler_i2c3_board_info[] = {
    { 
        I2C_BOARD_INFO("ov5640", 0x78>>1), 
        .platform_data = &whistler_ov5640_data,
    },
//[ECID:0000]ZTEBSP,yuxin add for cam ov7692 i2c config,2011.11.15,++
    { 
    //cause ov7692 and ov5640 i2c addresses are the same ,so ov7692 must be registered 
    //to another fake addr ,yuxin modify 2011.11.23
        I2C_BOARD_INFO("ov7692", 0x33), 
        .platform_data = &whistler_ov7692_data,
    },
//[ECID:0000]ZTEBSP,yuxin add for cam ov7692 i2c config,2011.11.15,--
};
/* ZTE: added by yuxin for new camera power on ,off 20110921 -- */
/*[ECID:0000]ZTEBSP YUXIN END*/

/*[ECID:000000] ZTEBSP wanghaifei start 20111021, delete useless*/
/*
static void whistler_adxl34x_init(void)
{
	tegra_gpio_enable(ADXL34X_IRQ_GPIO);
	gpio_request(ADXL34X_IRQ_GPIO, "adxl34x");
	gpio_direction_input(ADXL34X_IRQ_GPIO);
}

static void whistler_isl29018_init(void)
{
	tegra_gpio_enable(ISL29018_IRQ_GPIO);
	gpio_request(ISL29018_IRQ_GPIO, "isl29018");
	gpio_direction_input(ISL29018_IRQ_GPIO);
}

static struct i2c_board_info whistler_i2c1_board_info[] = {
	{
		I2C_BOARD_INFO("adxl34x", 0x1D),
		.irq = TEGRA_GPIO_TO_IRQ(ADXL34X_IRQ_GPIO),
	},
	{
		I2C_BOARD_INFO("isl29018", 0x44),
		.irq = TEGRA_GPIO_TO_IRQ(ISL29018_IRQ_GPIO),
	},
};

static void whistler_adt7461_init(void)
{
	tegra_gpio_enable(ADT7461_IRQ_GPIO);
	gpio_request(ADT7461_IRQ_GPIO, "adt7461");
	gpio_direction_input(ADT7461_IRQ_GPIO);
}

static struct adt7461_platform_data whistler_adt7461_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.therm2 = true,
	.conv_rate = 0x05,
	.offset = 0,
	.hysteresis = 0,
	.shutdown_ext_limit = 115,
	.shutdown_local_limit = 120,
	.throttling_ext_limit = 90,
	.alarm_fn = tegra_throttling_enable,
};
*/
/*[ECID:000000] ZTEBSP wanghaifei end 20111021*/

/*[ECID:000000] ZTEBSP wangbing 20111020, add temperature sense driver start */
static int whistler_nct1008_power_on(void)
{
	if (!reg_vcc_therm) {
		reg_vcc_therm = regulator_get(NULL, "vdd_therm");
		if (IS_ERR_OR_NULL(reg_vcc_therm)) {
			pr_err("whistler_nct1008_init: vdd_therm failed\n");
			reg_vcc_therm = NULL;
			return PTR_ERR(reg_vcc_therm);
		}
		//[ECID:000000]ZTEBSP DangXiao 20111116 start, qHD DSI LCD
		regulator_set_voltage(reg_vcc_therm, 3100*1000, 3100*1000);
		//[ECID:000000]ZTEBSP DangXiao 20111116 end.
		mdelay(5);
		regulator_enable(reg_vcc_therm);
		printk(KERN_ERR"whistler_nct1008_init ldo enabled\n");  //wangbing

	}
	mdelay(5);	

	return 0;
}

static int whistler_nct1008_power_off(void)
{
/*  //[ECID:000000]ZTEBSP DangXiao 20111116 start, qHD DSI LCD
	if (reg_vcc_therm) {
		regulator_disable(reg_vcc_therm);
		regulator_put(reg_vcc_therm);
		reg_vcc_therm = NULL;
	}
*/
	return 0;
}

static void whistler_nct1008_init(void)
{
       printk(KERN_ERR"whistler_nct1008_init\n");  //wangbing
	tegra_gpio_enable(NCT1008_THERM2_GPIO);
	gpio_request(NCT1008_THERM2_GPIO, "temp_alert");
	gpio_direction_input(NCT1008_THERM2_GPIO);
}

static struct nct1008_platform_data whistler_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.conv_rate = 0x08,
	.offset = 0,
	.hysteresis = 0,
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111130 start*/
	.shutdown_ext_limit = 115, //115,85
	.shutdown_local_limit = 115, //120,85
	.throttling_ext_limit = 90, //90,70
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111130 end*/
	.alarm_fn = tegra_throttling_enable,
	.power_on = whistler_nct1008_power_on,
	.power_off = whistler_nct1008_power_off,
};
/*[ECID:000000] ZTEBSP wangbing 20111020, add temperature sense driver end */



static struct i2c_board_info whistler_i2c4_board_info[] = {
#if 0
	{
		I2C_BOARD_INFO("adt7461", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(ADT7461_IRQ_GPIO),
		.platform_data = &whistler_adt7461_pdata,
	},
	{
		I2C_BOARD_INFO("tca6416", 0x20),
		.platform_data = &whistler_tca6416_data,
	},
#endif	
/*[ECID:000000] ZTEBSP wangbing 20111020, add temperature sense driver start */
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(NCT1008_THERM2_GPIO),
		.platform_data = &whistler_nct1008_pdata,
	},	
/*[ECID:000000] ZTEBSP wangbing 20111020, add temperature sense driver end */
};


/*[ECID:000000] ZTEBSP wanghaifei start 20111018, add light sensor i2c device init*/
static void taos_tmd2771_init(void)
{
        tegra_gpio_enable(TMD2771_IRQ_GPIO);
        gpio_request(TMD2771_IRQ_GPIO, "tmd2771_irq");
        gpio_direction_input(TMD2771_IRQ_GPIO);

}

static int light_sensor_power_init(void)
{
        return 0;
}

static struct taos_platform_data taos_data={
        .init_hw_power = light_sensor_power_init,
};

static struct i2c_board_info  taos_i2c0_boardinfo[] __initdata ={
        {   
                I2C_BOARD_INFO("taos", 0x39),
                .irq = TEGRA_GPIO_TO_IRQ(TMD2771_IRQ_GPIO),
                .platform_data = &taos_data,
        },  
};
/*[ECID:000000] ZTEBSP wanghaifei end 20111018*/


/*[ECID:000000] ZTEBSP zhangbo 20111123, nfc pn544, start*/
#ifdef CONFIG_PN544_NFC
static struct pn544_i2c_platform_data pn544_data = {
	.irq_gpio = TEGRA_GPIO_PG2,
	.ven_gpio = TEGRA_GPIO_PG3,
	.ven_gpio_second = TEGRA_GPIO_PE2,
	.firm_gpio = TEGRA_GPIO_PX5,
};

static struct i2c_board_info __initdata pn544_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO("pn544", 0x28),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PG2),
		.platform_data = &pn544_data,
	},
};

static void whistler_nfc_init(void)
{
	tegra_gpio_enable(pn544_data.irq_gpio);	
	tegra_gpio_enable(pn544_data.ven_gpio);
	tegra_gpio_enable(pn544_data.ven_gpio_second);
	tegra_gpio_enable(pn544_data.firm_gpio);

	gpio_export(pn544_data.irq_gpio, false);
	gpio_export(pn544_data.ven_gpio, false);
	gpio_export(pn544_data.ven_gpio_second, false);
	gpio_export(pn544_data.firm_gpio, false);
	
	i2c_register_board_info(0, pn544_i2c0_boardinfo,
	ARRAY_SIZE(pn544_i2c0_boardinfo));
}
#endif
/*[ECID:000000] ZTEBSP zhangbo 20111123, nfc pn544, end*/

/*[ECID:000000] ZTEBSP wanghaifei start 20120220,add mpu sensor init*/
static struct mpu_platform_data mpu3050_data = {
        .int_config     = 0x10,
        .level_shifter  = 0,
        .orientation    = MPU_GYRO_ORIENTATION, /* Located in board_[platformname].h    */
};

static struct ext_slave_platform_data mpu3050_mma845x_data = {
        .address        = MPU_MMA845X_ADDR,
        .irq            = 0,
        .adapt_num      = MPU_ACCEL_BUS_NUM,
        .bus            = EXT_SLAVE_BUS_SECONDARY,
        .orientation    = MPU_ACCEL_ORIENTATION,        /* Located in board_[platformname].h    */
};

static struct ext_slave_platform_data mpu3050_lis3dh_data = {
        .address        = MPU_LIS3DH_ADDR,
        .irq            = 0,
        .adapt_num      = MPU_ACCEL_BUS_NUM,
        .bus            = EXT_SLAVE_BUS_SECONDARY,
        .orientation    = MPU_ACCEL_ORIENTATION,        /* Located in board_[platformname].h    */
};

static struct ext_slave_platform_data mpu3050_kxtf9_data = {
        .address        = MPU_KXTF9_ADDR,
        .irq            = 0,
        .adapt_num      = MPU_ACCEL_BUS_NUM,
        .bus            = EXT_SLAVE_BUS_SECONDARY,
        .orientation    = MPU_ACCEL_ORIENTATION,        /* Located in board_[platformname].h    */
};
static struct ext_slave_platform_data mpu_compass_data = {
        .address        = MPU_COMPASS_ADDR,
        .irq            = 0,
        .adapt_num      = MPU_COMPASS_BUS_NUM,
        .bus            = EXT_SLAVE_BUS_PRIMARY,
        .orientation    = MPU_COMPASS_ORIENTATION,      /* Located in board_[platformname].h    */
};

static struct i2c_board_info __initdata inv_mpu_i2c2_board_info[] = {
        {
                I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
                .irq = TEGRA_GPIO_TO_IRQ(MPU_GYRO_IRQ_GPIO),
                .platform_data = &mpu3050_data,
        },
        {
                I2C_BOARD_INFO(MPU_MMA845X_NAME, MPU_MMA845X_ADDR),
//              .irq = TEGRA_GPIO_TO_IRQ(MPU_ACCEL_IRQ_GPIO1),
                .platform_data = &mpu3050_mma845x_data,
        },
        {
                I2C_BOARD_INFO(MPU_LIS3DH_NAME, MPU_LIS3DH_ADDR),
                .platform_data = &mpu3050_lis3dh_data,
        },
        {
                I2C_BOARD_INFO(MPU_KXTF9_NAME, MPU_KXTF9_ADDR),
                .platform_data = &mpu3050_kxtf9_data,
        },
};

static struct i2c_board_info __initdata compass_i2c2_board_info[] = {
        {
                I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
//              .irq = TEGRA_GPIO_TO_IRQ(MPU_COMPASS_IRQ_GPIO),
                .platform_data = &mpu_compass_data,
        },
};

static void mpuirq_init(void)
{
        int ret = 0;

        pr_info("*** MPU START *** mpuirq_init...\n");

        /* MPU-IRQ assignment */
        tegra_gpio_enable(MPU_GYRO_IRQ_GPIO);
        ret = gpio_request(MPU_GYRO_IRQ_GPIO, MPU_GYRO_NAME);
        if (ret < 0) {
                pr_err("%s: gpio_request gyro_irq failed %d\n", __func__, ret);
        }

        ret = gpio_direction_input(MPU_GYRO_IRQ_GPIO);
        if (ret < 0) {
                pr_err("%s: gpio_direction_input gyro_irq failed %d\n", __func__, ret);
        }

        /* ACCEL-IRQ assignment */
        tegra_gpio_enable(MPU_ACCEL_IRQ_GPIO1);
        ret = gpio_request(MPU_ACCEL_IRQ_GPIO1, "accel_irq1");
        if (ret < 0) {
                pr_err("%s: gpio_request accel_irq1 failed %d\n", __func__, ret);
        }

        ret = gpio_direction_input(MPU_ACCEL_IRQ_GPIO1);
        if (ret < 0) {
                pr_err("%s: gpio_direction_input accel_irq1 failed %d\n", __func__, ret);
        }

        tegra_gpio_enable(MPU_ACCEL_IRQ_GPIO2);
        ret = gpio_request(MPU_ACCEL_IRQ_GPIO2, "accel_irq2");
        if (ret < 0) {
                pr_err("%s: gpio_request accel_irq2 failed %d\n", __func__, ret);
        }

        ret = gpio_direction_input(MPU_ACCEL_IRQ_GPIO2);
        if (ret < 0) {
                pr_err("%s: gpio_direction_input accel_irq2 failed %d\n", __func__, ret);
        }

        /* COMPASS-IRQ assignment */
        tegra_gpio_enable(MPU_COMPASS_IRQ_GPIO);
        ret = gpio_request(MPU_COMPASS_IRQ_GPIO, MPU_COMPASS_NAME);
        if (ret < 0) {
                pr_err("%s: gpio_request compass_irq failed %d\n", __func__, ret);
        }

        ret = gpio_direction_input(MPU_COMPASS_IRQ_GPIO);
        if (ret < 0) {
                pr_err("%s: gpio_direction_input compass_irq failed %d\n", __func__, ret);
        }
        pr_info("*** MPU END *** mpuirq_init...\n");

        i2c_register_board_info(MPU_GYRO_BUS_NUM, inv_mpu_i2c2_board_info,
                ARRAY_SIZE(inv_mpu_i2c2_board_info));
        i2c_register_board_info(MPU_COMPASS_BUS_NUM, compass_i2c2_board_info,
                ARRAY_SIZE(compass_i2c2_board_info));
}
/*[ECID:000000] ZTEBSP wanghaifei end 20120220,add sensor init*/

int __init whistler_sensors_init(void)
{
/*[ECID:000000] ZTEBSP wangbing 20111020, add temperature sense driver start */
#if 0
	whistler_camera_init();

	whistler_adxl34x_init();

	whistler_isl29018_init();

	whistler_adt7461_init();

	i2c_register_board_info(0, whistler_i2c1_board_info,
		ARRAY_SIZE(whistler_i2c1_board_info));

	i2c_register_board_info(4, whistler_i2c4_board_info,
		ARRAY_SIZE(whistler_i2c4_board_info));

	i2c_register_board_info(3, whistler_i2c3_board_info,
		ARRAY_SIZE(whistler_i2c3_board_info));

	tegra_fuse_regulator_en = whistler_fuse_power_en;

#else
/* [ECID:0000]ZTEBSP: added by yuxin for new camera 20110921 begin */
      printk(KERN_ERR "yuxin whistler_sensors_init\n");
      whistler_camera_init();
      i2c_register_board_info(3, whistler_i2c3_board_info,
		          ARRAY_SIZE(whistler_i2c3_board_info));

/* [ECID:0000]ZTEBSP: added by yuxin for new camera 20110921 end */
	whistler_nct1008_init();  

	i2c_register_board_info(4, whistler_i2c4_board_info,
		ARRAY_SIZE(whistler_i2c4_board_info));

#endif
/*[ECID:000000] ZTEBSP wangbing 20111020, add temperature sense driver end */

/*[ECID:000000] ZTEBSP wanghaifei start 20120220,add sensor init*/
	mpuirq_init();
        taos_tmd2771_init();
        i2c_register_board_info(0, taos_i2c0_boardinfo,
                ARRAY_SIZE(taos_i2c0_boardinfo));
/*[ECID:000000] ZTEBSP wanghaifei end 20120220*/
/*[ECID:000000] ZTEBSP, zhangbo 20111123, nfc pn544, start*/
#ifdef CONFIG_PN544_NFC
	whistler_nfc_init();
#endif
/*[ECID:000000] ZTEBSP, zhangbo 20111123, nfc pn544, end*/

	return 0;
}

int __init whistler_sensor_late_init(void)
{
	int ret;

	if (!machine_is_whistler())
		return 0;
	
	/* [ECID:0000]ZTEBSP: del by yuxin for not used code 20110921 begin */
   #if 0
	reg_vddio_vi = regulator_get(NULL, "vddio_vi");
	if (IS_ERR_OR_NULL(reg_vddio_vi)) {
		pr_err("%s: Couldn't get regulator vddio_vi\n", __func__);
		return PTR_ERR(reg_vddio_vi);
	}

	//set vddio_vi voltage to 1.8v 
	ret = regulator_set_voltage(reg_vddio_vi, 1800*1000, 1800*1000);
	if (ret) {
		pr_err("%s: Failed to set vddio_vi to 1.8v\n", __func__);
		goto fail_put_regulator;
	}

	regulator_put(reg_vddio_vi);
	reg_vddio_vi = NULL;
  #endif
	/* [ECID:0000]ZTEBSP: del by yuxin for not used code 20110921 end */
	return 0;

fail_put_regulator:
	regulator_put(reg_vddio_vi);
	reg_vddio_vi = NULL;	
	return ret;
}

//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
//late_initcall(whistler_sensor_late_init);
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up

