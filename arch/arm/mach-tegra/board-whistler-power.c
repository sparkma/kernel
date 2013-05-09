/*
 * Copyright (C) 2010 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
//#include <linux/mfd/max8907c.h>
//#include <linux/regulator/max8907c-regulator.h>
#include <linux/mfd/tps6586x.h>
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up
#include <linux/gpio.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
#include <generated/mach-types.h>
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up

#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "wakeups-t2.h"
#include "board.h"
//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
#include "board-whistler.h"
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
static struct regulator_consumer_supply tps658621_sm0_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};
static struct regulator_consumer_supply tps658621_sm1_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};
static struct regulator_consumer_supply tps658621_sm2_supply[] = {
	REGULATOR_SUPPLY("vdd_sm2", NULL),
};
static struct regulator_consumer_supply tps658621_ldo0_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo0", NULL),
/*[ECID:0000]ZTEBSP yuxin start for cam mipi 2011.09.21 */
	//REGULATOR_SUPPLY("p_cam_avdd", NULL),
	REGULATOR_SUPPLY("cam_vcsi", "tegra_camera"), 
/*[ECID:0000]ZTEBSP yuxin end for cam mipi 2011.09.21 */

//[ECID:000000]ZTEBSP DangXiao start, Oscar qHD DSI LCD
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
//[ECID:000000]ZTEBSP DangXiao end
};

static struct regulator_consumer_supply tps658621_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo1", NULL),
	REGULATOR_SUPPLY("avdd_pll", NULL),
};

static struct regulator_consumer_supply tps658621_ldo2_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo2", NULL),
	REGULATOR_SUPPLY("vdd_rtc", NULL),
	REGULATOR_SUPPLY("vdd_aon", NULL),
};

static struct regulator_consumer_supply tps658621_ldo3_supply[] = {
	//[ECID:0000] ZTEBSP maxiaoping 20111011 here need add other peripheral
	REGULATOR_SUPPLY("vdd_ldo3", NULL),
	//REGULATOR_SUPPLY("avdd_usb", NULL),
	//REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	//[ECID:000000] ZTEBSP DangXiao 20111118 start, MimosaX touchscreen
	REGULATOR_SUPPLY("vcc_tsc", NULL),
	//[ECID:000000] ZTEBSP DangXiao 20111118 end
	REGULATOR_SUPPLY("vcc_lcd", NULL),
};

static struct regulator_consumer_supply tps658621_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo4", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	//[ECID:0000] ZTEBSP maxiaoping 20111011   for w_skate
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
};

static struct regulator_consumer_supply tps658621_ldo5_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo5", NULL),
	REGULATOR_SUPPLY("vmmc", "sdhci-tegra.3"),
	//[ECID:0000] ZTEBSP maxiaoping 20111011   for w_skate
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
};

static struct regulator_consumer_supply tps658621_ldo6_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo6", NULL),
	//[ECID:0000] ZTEBSP maxiaoping 20111011   for w_skate	
	//REGULATOR_SUPPLY("vcsi", "tegra_camera"),
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
};

static struct regulator_consumer_supply tps658621_ldo7_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo7", NULL),
	//REGULATOR_SUPPLY("avdd_hdmi", NULL),
	REGULATOR_SUPPLY("vdd_fuse", NULL),	
    /* [ECID:000000] ZTEBSP wangjianping start 20120214 add SD card driver */
	REGULATOR_SUPPLY("vmmc", "sdhci-tegra.2"),
    /* [ECID:000000] ZTEBSP wangjianping end 20120214 add SD card driver */	
};

static struct regulator_consumer_supply tps658621_ldo8_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo8", NULL),
	//[ECID:0000] ZTEBSP maxiaoping 20111011 for w_skate
	//REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
};

static struct regulator_consumer_supply tps658621_ldo9_supply[] = {
	//[ECID:0000] ZTEBSP maxiaoping 20111011
	REGULATOR_SUPPLY("vdd_ldo9", NULL),
	REGULATOR_SUPPLY("avdd_2v85", NULL),
	//REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
	REGULATOR_SUPPLY("avdd_amp", NULL),
/*[ECID:000000] ZTEBSP wangbing 20111020, add temperature sense driver */
	REGULATOR_SUPPLY("vdd_therm", NULL),
};

//ZTEBSP DangXiao 20120608, ++
static struct tps6586x_settings sm0_config = {
	.sm_pwm_mode = PWM_DEFAULT_VALUE,
	.slew_rate = SLEW_RATE_3520UV_PER_SEC,
};		
//ZTEBSP DangXiao 20120608, ++
/*
 * Current TPS6586x is known for having a voltage glitch if current load changes
 * from low to high in auto PWM/PFM mode for CPU's Vdd line.
 */
static struct tps6586x_settings sm1_config = {
	.sm_pwm_mode = PWM_ONLY,
	//ZTEBSP DangXiao 20120608, ++
	.slew_rate = SLEW_RATE_3520UV_PER_SEC,	
	//ZTEBSP DangXiao 20120608, ++
};

#define REGULATOR_INIT(_id, _minmv, _maxmv, on, config)			\
	{								\
		.constraints = {					\
			.min_uV = (_minmv)*1000,			\
			.max_uV = (_maxmv)*1000,			\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					     REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					   REGULATOR_CHANGE_STATUS |	\
					   REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = on,				\
		},							\
		.num_consumer_supplies = ARRAY_SIZE(tps658621_##_id##_supply),\
		.consumer_supplies = tps658621_##_id##_supply,		\
		.driver_data = config,					\
	}

#define ON	1
#define OFF	0

static struct regulator_init_data sm0_data = REGULATOR_INIT(sm0, 725, 1500, ON, &sm0_config);
static struct regulator_init_data sm1_data = REGULATOR_INIT(sm1, 725, 1500, ON, &sm1_config);
//[ECID:0000] ZTEBSP maxiaoping 20111011 change sm2 from always_on to can enable and disable
//static struct regulator_init_data sm2_data = REGULATOR_INIT(sm2, 3000, 4550, ON, NULL);
static struct regulator_init_data sm2_data = REGULATOR_INIT(sm2, 3000, 4550, OFF, NULL);

//[ECID:0000]ZTEBSP,yuxin modify the LDO0 output voltage range,as NV suggest,2010.10.20
//static struct regulator_init_data ldo0_data = REGULATOR_INIT(ldo0, 1250, 3300, OFF, NULL);
static struct regulator_init_data ldo0_data = REGULATOR_INIT(ldo0, 1200, 3350, OFF, NULL);

static struct regulator_init_data ldo1_data = REGULATOR_INIT(ldo1, 725, 1500, ON, NULL);
static struct regulator_init_data ldo2_data = REGULATOR_INIT(ldo2, 725, 1500, OFF, NULL);
static struct regulator_init_data ldo3_data = REGULATOR_INIT(ldo3, 1250, 3300, OFF, NULL);

//[ECID:0000] ZTEBSP maxiaoping 20111031 change LDO4 to uncontrol by the periherals.
//static struct regulator_init_data ldo4_data = REGULATOR_INIT(ldo4, 1700, 2475, OFF, NULL);
static struct regulator_init_data ldo4_data = REGULATOR_INIT(ldo4, 1700, 2475, ON, NULL);

static struct regulator_init_data ldo5_data = REGULATOR_INIT(ldo5, 1250, 3300, ON, NULL);
static struct regulator_init_data ldo6_data = REGULATOR_INIT(ldo6, 1250, 1800, OFF, NULL);
static struct regulator_init_data ldo7_data = REGULATOR_INIT(ldo7, 1250, 3300, OFF, NULL);
static struct regulator_init_data ldo8_data = REGULATOR_INIT(ldo8, 1250, 3300, OFF, NULL);
//[ECID:000000] ZTEBSP DangXiao 20111115 start, MimosaX qHD DSI LCD
static struct regulator_init_data ldo9_data = REGULATOR_INIT(ldo9, 1250, 3300, ON, NULL);
//static struct regulator_init_data ldo9_data = REGULATOR_INIT(ldo9, 1250, 3300, OFF, NULL);
//[ECID:000000] ZTEBSP DangXiao 20111115 end

static struct tps6586x_rtc_platform_data rtc_data = {
	.irq = TEGRA_NR_IRQS + TPS6586X_INT_RTC_ALM1,
	.start = {
		.year = 2009,
		.month = 1,
		.day = 1,
	},
	.cl_sel = TPS6586X_RTC_CL_SEL_1_5PF /* use lowest (external 20pF cap) */
};

#define TPS_REG(_id, _data)			\
	{					\
		.id = TPS6586X_ID_##_id,	\
		.name = "tps6586x-regulator",	\
		.platform_data = _data,		\
	}

static struct tps6586x_subdev_info tps_devs[] = {
	TPS_REG(SM_0, &sm0_data),
	TPS_REG(SM_1, &sm1_data),
	TPS_REG(SM_2, &sm2_data),
	TPS_REG(LDO_0, &ldo0_data),
	TPS_REG(LDO_1, &ldo1_data),
	TPS_REG(LDO_2, &ldo2_data),
	TPS_REG(LDO_3, &ldo3_data),
	TPS_REG(LDO_4, &ldo4_data),
	TPS_REG(LDO_5, &ldo5_data),
	TPS_REG(LDO_6, &ldo6_data),
	TPS_REG(LDO_7, &ldo7_data),
	TPS_REG(LDO_8, &ldo8_data),
	TPS_REG(LDO_9, &ldo9_data),
	{
		.id	= 0,
		.name	= "tps6586x-rtc",
		.platform_data = &rtc_data,
	},
};

static struct tps6586x_platform_data tps_platform = {
	.irq_base = TPS6586X_INT_BASE,
	.num_subdevs = ARRAY_SIZE(tps_devs),
	.subdevs = tps_devs,
	.gpio_base = TPS6586X_GPIO_BASE,
#if 0
/*wangbing, for thermal sensor shutdown, 20111105 */
	.resume_reset_time = 350,
#endif
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111118 start*/
	.temp_time = 2000,
	.temp_gpio = TEGRA_GPIO_PS0,
/*[ECID:000000] ZTEBSP wangbing, for thermal sensor shutdown, 20111118 end*/
};

static struct i2c_board_info __initdata whistler_regulators[] = {
	{
		I2C_BOARD_INFO("tps6586x", 0x34),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &tps_platform,
	},
};

//[ECID:000000] shiyan added for fuel gang 04.18 ++
static struct i2c_board_info __initdata max_17040[] = {
	{
		I2C_BOARD_INFO("max17040", 0x36),   //²»´ø¶ÁÐ´Î»
	},
};
//[ECID:000000]shiyan added for fuel gang 04.18--

/*[ECID:000000] ZTEBSP zhangbo change for suspend, start*/
static struct tegra_suspend_platform_data whistler_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 100,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0xf,
	.corereq_high	= false,
	.sysclkreq_high	= true,
	//.combined_req   = true,
};
/*[ECID:000000] ZTEBSP zhangbo change for suspend, end*/

int __init whistler_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	void __iomem *chip_id = IO_ADDRESS(TEGRA_APB_MISC_BASE) + 0x804;
	u32 pmc_ctrl;
	u32 minor;       

         //[ECID:0000]maxiaoping 201110   for w_skate
	printk(KERN_NOTICE "PM_DEBUG_MXP: Enter  whistler regulator  init.\r\n");
	minor = (readl(chip_id) >> 16) & 0xf;
	/* A03 (but not A03p) chips do not support LP0 */
//	if (minor == 3 && !(tegra_spare_fuse(18) || tegra_spare_fuse(19)))
//		whistler_suspend_data.suspend_mode = TEGRA_SUSPEND_LP1;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

//[ECID:000000] shiyan added for fuel gang 04.18 ++
	i2c_register_board_info(4, max_17040, 1);
//[ECID:000000]shiyan added for fuel gang 04.18--

	i2c_register_board_info(4, whistler_regulators, 1);

	//regulator_has_full_constraints();

	tegra_init_suspend(&whistler_suspend_data);

	return 0;
}

static int __init whistler_pcie_init(void)
{
	int ret;

	if (!machine_is_whistler())
		return 0;

	ret = gpio_request(TPS6586X_GPIO_BASE, "pcie_vdd");
	if (ret < 0)
		goto fail;

	ret = gpio_direction_output(TPS6586X_GPIO_BASE, 1);
	if (ret < 0)
		goto fail;

	gpio_export(TPS6586X_GPIO_BASE, false);

	return 0;

fail:
	pr_err("%s: gpio_request failed #%d\n", __func__, TPS6586X_GPIO_BASE);
	gpio_free(TPS6586X_GPIO_BASE);
	return ret;
}

//late_initcall(whistler_pcie_init);
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up
