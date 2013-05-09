/*
 * arch/arm/mach-tegra/board-whistler-panel.c
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/kernel.h>
#include <linux/pwm_backlight.h>
//[ECID:0000] ZTEBSP maxiaoping 20111209 modify for tegra pwm backlight control.++
//[ECID:0000] ZTEBSP maxiaoping 20111027 add for pmu backlight adjust.
#include <linux/tps6586x_pwm_bl.h>
#include <linux/tegra_pwm_bl.h>
//[ECID:0000] ZTEBSP maxiaoping 20111027 modify end.
//[ECID:0000] ZTEBSP maxiaoping 20111209 --

#include <linux/nvhost.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include "devices.h"
#include "gpio-names.h"
#include "board.h"

//[ECID:000000] ZTEBSP DangXiao 20111116 start, Oscar qHD DSI LCD
#include "board_p943t_panel_init_code.h"
#define WHISTLER_DSI_PANEL_RESET		TEGRA_GPIO_PV7  	//175  TEGRA_GPIO_PV7
#define enterprise_lcd_te		TEGRA_GPIO_PJ1
static bool kernel_1st_panel_init = true;      				//first init flag  
#define LCD_BL_CONTROL		TEGRA_GPIO_PW1     //BL control

#define TEGRA_DC_OUT_ONE_SHOT_MODE 1
#define DC_CTRL_MODE    TEGRA_DC_OUT_ONE_SHOT_MODE  	//Ninja:for cmd control
#define DSI_PANEL_RESET 1   								// Ninja:for Reset Control
static struct regulator *whistler_dsi_reg = NULL;   			//MIPI 1.2V 
static struct regulator *ventana_LDO9 = NULL; 
//[ECID:000000] ZTEBSP DangXiao 20111116 end, Oscar qHD DSI LCD

//[ECID:0000] ZTEBSP maxiaoping 20111209  for MIMOSA backlight control ++.
#if 0
static void __iomem *displaya_base = IO_ADDRESS(TEGRA_DISPLAY_BASE);
#define DC_COM_PIN_OUTPUT_SELECT5_0     0x319
#define DC_CMD_DISPLAY_POWER_CONTROL_0  0x36
#define DC_COM_PM1_CONTROL_0            0x31E
#define DC_COM_PM1_DUTY_CYCLE_0         0x31F
#endif
//[ECID:0000] ZTEBSP maxiaoping 20111209 --.

#define whistler_hdmi_hpd	TEGRA_GPIO_PN7

#ifdef CONFIG_TEGRA_DC
static struct regulator *whistler_hdmi_reg = NULL;
static struct regulator *whistler_hdmi_pll = NULL;
#endif
//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
extern int tps6586x_write_register(int reg, unsigned int val);

static int pmu_backlight_notify(struct device *unused, int brightness)
{
	#if 1
        uint8_t val0 = 0;
        uint8_t val1 = 0;

        //printk(KERN_NOTICE "PM_DEBUG_MXP:brightness=%d .\r\n",brightness);
        val0 = brightness << 3;
        val1 = 0x08 | ((brightness >> 5) & 0x07);
		
        //[ECID:0000] ZTEBSP maxiaoping 20111118  delay 50ms for LCD backlight control.
        mdelay(25);
		
        if(brightness != 0) {
                tps6586x_write_register(0x57, val0);
                tps6586x_write_register(0x58, val1);
        } else {
                tps6586x_write_register(0x57, 0);
                tps6586x_write_register(0x58, 0);
        }
 	#else
	unsigned int reg;
	printk(KERN_NOTICE "PM_DEBUG_MXP:brightness=%d .\r\n",brightness);
	
	/* Set duty cycle to change the PWM to adjust the LCD backlight */       
	if(brightness != 0) {
                writel(brightness, (displaya_base + DC_COM_PM1_DUTY_CYCLE_0));
        } else {
                writel(0, (displaya_base + DC_COM_PM1_DUTY_CYCLE_0));
        }
	#endif
        return brightness;
}
/*
 * In case which_pwm is TEGRA_PWM_PM0,
 * gpio_conf_to_sfio should be TEGRA_GPIO_PW0: set LCD_CS1_N pin to SFIO
 * In case which_pwm is TEGRA_PWM_PM1,
 * gpio_conf_to_sfio should be TEGRA_GPIO_PW1: set LCD_M1 pin to SFIO
 */
//[ECID:0000] ZTEBSP maxiaoping 20120112 for MIMOSA backlight control,start
static struct platform_tegra_pwm_backlight_data whistler_disp1_backlight_data = {
        .which_dc       = 0,
        .which_pwm      = TEGRA_PWM_PM1,     
        .max_brightness = 255,
        .dft_brightness = 102,

        .period         = 63,//255
        .clk_div           = 25,
        .clk_select         = 1,
};
//[ECID:0000] ZTEBSP maxiaoping 20120112 for MIMOSA backlight control,end

static struct platform_device whistler_disp1_backlight_device = {
	.name	= "tegra-pwm-bl",
	.id	= -1,
	.dev	= {
		.platform_data = &whistler_disp1_backlight_data,
	},
};
/*[ECID:000000] ZTEBSP zhangbo 20120117, add led driver, start*/
extern int pmu_led_backlight_notify(struct device *unused, int brightness);

static struct platform_tps6586x_pwm_backlight_data whistler_led_backlight_data = {
	.max_brightness	= 256,
	.dft_brightness	= 0,
        .notify = pmu_led_backlight_notify,
};


static struct platform_device whistler_led_backlight_device = {
	.name	= "tegra_led",
	.id	= -1,
	.dev	= {
		.platform_data = &whistler_led_backlight_data,
	},
};
/*[ECID:000000] ZTEBSP zhangbo 20120117, add led driver, end*/

#ifdef CONFIG_TEGRA_DC
static int whistler_hdmi_enable(void)
{
	if (!whistler_hdmi_reg) {
		whistler_hdmi_reg = regulator_get(NULL, "avdd_hdmi"); /* LD011 */
		if (IS_ERR_OR_NULL(whistler_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			whistler_hdmi_reg = NULL;
			return PTR_ERR(whistler_hdmi_reg);
		}
	}
	regulator_enable(whistler_hdmi_reg);

	if (!whistler_hdmi_pll) {
		whistler_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll"); /* LD06 */
		if (IS_ERR_OR_NULL(whistler_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			whistler_hdmi_pll = NULL;
			regulator_disable(whistler_hdmi_reg);
			whistler_hdmi_reg = NULL;
			return PTR_ERR(whistler_hdmi_pll);
		}
	}
	regulator_enable(whistler_hdmi_pll);
	return 0;
}

static int whistler_hdmi_disable(void)
{
	regulator_disable(whistler_hdmi_reg);
	regulator_disable(whistler_hdmi_pll);
	return 0;
}

static struct resource whistler_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
	},
//[ECID:000000] ZTEBSP DangXiao start 20111115, qHD MIPI LCD
	{
		.name = "dsi_regs",
		.start = TEGRA_DSI_BASE,
		.end = TEGRA_DSI_BASE + TEGRA_DSI_SIZE-1,
		.flags = IORESOURCE_MEM,
	},	
//[ECID:000000] ZTEBSP DangXiao end 20111115	
};

static struct resource whistler_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode whistler_dsi_panel_modes[] = {
	//[ECID:000000] ZTEBSP DangXiao start 20111117, qHD MIPI LCD
	{     //[ECID:000000]ZTEBSP DangXiao change for GPS test, 12-24
		.pclk = 12927000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 4,
		.h_sync_width = 4,
		.v_sync_width = 8,
		.h_back_porch = 52,
		.v_back_porch = 12,
		.h_active = 540,
		.v_active = 960,
		.h_front_porch = 60,
		.v_front_porch = 12,
	},
	//[ECID:000000] ZTEBSP DangXiao end 20111117	
};

static struct tegra_dc_out_pin whistler_dc_out_pins[] = {
	{
		.name	= TEGRA_DC_OUT_PIN_H_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_V_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_PIXEL_CLOCK,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
};

static u8 whistler_dc_out_pin_sel_config[] = {
	TEGRA_PIN_OUT_CONFIG_SEL_LM1_PM1,
};

//[ECID:000000] ZTEBSP DangXiao start 20111115, qHD MIPI LCD

static struct tegra_dsi_cmd dsi_init_cmd[]= {
	DSI_CMD_SHORT(0x05, 0x11, 0x00),
	DSI_DLY_MS(120), //[ECID:000000] ZTEBSP DangXiao 20111129 ,20->120
#if(DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE)
	DSI_CMD_SHORT(0x15, 0x35, 0x00),
#endif
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(20), //[ECID:000000] ZTEBSP DangXiao 20120129 ,120->20
};

static struct tegra_dsi_cmd dsi_early_suspend_cmd[] = {
	DSI_CMD_SHORT(0x05, 0x28, 0x00),
	DSI_DLY_MS(20),
#if(DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE)
	DSI_CMD_SHORT(0x05, 0x34, 0x00),
#endif
};

static struct tegra_dsi_cmd dsi_late_resume_cmd[] = {
#if(DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE)
	DSI_CMD_SHORT(0x15, 0x35, 0x00),
#endif
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(20),
};

static struct tegra_dsi_cmd dsi_suspend_cmd[] = {
	DSI_CMD_SHORT(0x05, 0x28, 0x00),
	DSI_DLY_MS(20), //[ECID:000000] ZTEBSP DangXiao 20120129 ,120->20
#if(DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE)
	DSI_CMD_SHORT(0x05, 0x34, 0x00),
#endif
	DSI_CMD_SHORT(0x05, 0x10, 0x00),
	DSI_DLY_MS(120),  //[ECID:000000] ZTEBSP DangXiao 20111129 ,20->120
};
static int whistler_dsi_panel_enable(void)
{
	int ret;
    	static bool bfirstBoot = true;
       // Ninja: Fixme  PMU enable , LDO ???

	//printk("Ninja:enter %s \n",__FUNCTION__);

	if (whistler_dsi_reg == NULL) {
		whistler_dsi_reg = regulator_get(NULL, "avdd_dsi_csi");
		if (IS_ERR_OR_NULL(whistler_dsi_reg)) {
			pr_err("Ninja: dsi: Could not get regulator avdd_dsi_csi\n");
				whistler_dsi_reg = NULL;
				return PTR_ERR(whistler_dsi_reg);
		}
	}
	ret = regulator_enable(whistler_dsi_reg);
	if (ret < 0) {
		printk(KERN_ERR
			"Ninja: DSI regulator avdd_dsi_csi could not be enabled\n");
		return ret;
	}


	if (!ventana_LDO9) {
		ventana_LDO9 = regulator_get(NULL, "vdd_ldo9"); /* LD03 */
		if (IS_ERR_OR_NULL(ventana_LDO9)) {
			pr_err("dsi: couldn't get regulator vdd_ldo9\n");
			ventana_LDO9 = NULL;
			return PTR_ERR(ventana_LDO9);
		}
        	/* set vddio_vi voltage to 2.85v */
       ret = regulator_set_voltage(ventana_LDO9, 3100*1000, 3100*1000);
	if (ret) {
		pr_err("%s: Failed to set vdd_ldo3 to 3.10v\n", __func__);
              regulator_put(ventana_LDO9);
		return PTR_ERR(ventana_LDO9);
	}
            regulator_enable(ventana_LDO9); 
	}

	return ret;
}

static int whistler_dsi_panel_disable(void)
{
	//printk("Ninja:enter %s \n",__FUNCTION__);
#if DSI_PANEL_RESET
	//if (kernel_1st_panel_init != true) 
	{
		//tegra_gpio_disable(WHISTLER_DSI_PANEL_RESET);
		//gpio_free(WHISTLER_DSI_PANEL_RESET);
	} 
	//else
	//	kernel_1st_panel_init = false;
//DangXiao MIPI power always on
/*
     if (whistler_dsi_reg) {
		regulator_disable(whistler_dsi_reg);
		regulator_put(whistler_dsi_reg);
		whistler_dsi_reg = NULL;
	}
*/
//[ECID:000000] ZTEBSP DangXiao 20111129 start
     if (ventana_LDO9) {
		regulator_disable(ventana_LDO9);
		regulator_put(ventana_LDO9);
		ventana_LDO9 = NULL;
	}
//[ECID:000000] ZTEBSP DangXiao 20111129 end
#endif
	return 0;
}

static struct tegra_dsi_out whistler_dsi_out = {
	.n_data_lanes = 2,
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.refresh_rate = 60,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,
	//[ECID:000000] ZTEBSP DangXiao 20120405 start.
	//.panel_has_frame_buffer = true,
	.panel_reset = 0,
	.power_saving_suspend = true,
	.dsi_init_cmd = &dsi_init_cmd, 	/*init cmd*/
	.n_init_cmd = ARRAY_SIZE(dsi_init_cmd),
	.dsi_early_suspend_cmd = &dsi_early_suspend_cmd,    				
	.n_early_suspend_cmd = ARRAY_SIZE(dsi_early_suspend_cmd),    
	.dsi_late_resume_cmd = &dsi_late_resume_cmd,	
	.n_late_resume_cmd = ARRAY_SIZE(dsi_late_resume_cmd),
	.dsi_suspend_cmd = &dsi_suspend_cmd,
	.n_suspend_cmd = ARRAY_SIZE(dsi_suspend_cmd),
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,
	.lp_cmd_mode_freq_khz = 10000, 

	/* TODO: Get the vender recommended freq */
	.lp_read_cmd_mode_freq_khz = 230000,
    .te_polarity_low = true,
	//[ECID:000000] ZTEBSP DangXiao 20120405 end.
};

static struct tegra_dc_out whistler_disp1_out = {
	.type		= TEGRA_DC_OUT_DSI,

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.modes	 	= whistler_dsi_panel_modes,
	.n_modes 	= ARRAY_SIZE(whistler_dsi_panel_modes),

	.dsi	= &whistler_dsi_out,

	.enable   = whistler_dsi_panel_enable,
	.disable = whistler_dsi_panel_disable,
};

//[ECID:000000] ZTEBSP DangXiao end 20111115	

static struct tegra_dc_out whistler_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus	= 1,
	.hotplug_gpio	= whistler_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= whistler_hdmi_enable,
	.disable	= whistler_hdmi_disable,
};

static struct tegra_fb_data whistler_fb_data = {
	.win		= 0,
//[ECID:000000] ZTEBSP DangXiao start 20111115, qHD MIPI LCD
	.xres		= 540,
	.yres		= 960,
//[ECID:000000] ZTEBSP DangXiao end 20111115, qHD MIPI LCD
	.bits_per_pixel	= 32,
	.flags			= TEGRA_FB_FLIP_ON_PROBE,  //ZTEBSP DangXiao Nvidia Martin add at 2011.1010
};

static struct tegra_fb_data whistler_hdmi_fb_data = {
	.win		= 0,
	//[ECID:000000] ZTEBSP DangXiao 20111001 start, for HDMI
	.xres		= 1366,  //DangXiao
	.yres		= 768, //DangXiao
	//[ECID:000000] ZTEBSP DangXiao 20111001 end, for HDMI
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};


static struct tegra_dc_platform_data whistler_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &whistler_disp1_out,
	//ZTEBSP DangXiao 20120528, start
	.emc_clk_rate	= 150000000,
	//ZTEBSP DangXiao 20120528, start
	.fb		= &whistler_fb_data,
};

static struct nvhost_device whistler_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= whistler_disp1_resources,
	.num_resources	= ARRAY_SIZE(whistler_disp1_resources),
	.dev = {
		.platform_data = &whistler_disp1_pdata,
	},
};

static struct tegra_dc_platform_data whistler_disp2_pdata = {
	.flags		= 0,
	.default_out	= &whistler_disp2_out,
	.fb		= &whistler_hdmi_fb_data,
};

static struct nvhost_device whistler_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= whistler_disp2_resources,
	.num_resources	= ARRAY_SIZE(whistler_disp2_resources),
	.dev = {
		.platform_data = &whistler_disp2_pdata,
	},
};
#endif

static struct nvmap_platform_carveout whistler_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0x18C00000,
		.size		= SZ_128M - 0xC00000,
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data whistler_nvmap_data = {
	.carveouts	= whistler_carveouts,
	.nr_carveouts	= ARRAY_SIZE(whistler_carveouts),
};

static struct platform_device whistler_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &whistler_nvmap_data,
	},
};

static struct platform_device *whistler_gfx_devices[] __initdata = {
	&whistler_nvmap_device,
#ifdef CONFIG_TEGRA_GRHOST
	&tegra_grhost_device,
#endif
	&whistler_disp1_backlight_device,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend whistler_panel_early_suspender;
unsigned int panel_early_suspend_flag = 0; //ZTEBSP DangXiao 20120618 

static void whistler_panel_early_suspend(struct early_suspend *h)
{
	//ZTEBSP DangXiao 20120618,start
	panel_early_suspend_flag = 1;
	//printk("Ninja: panel early suspend, Flag =1!\n ");
	//ZTEBSP DangXiao 20120618,start
        /* power down LCD, add use a black screen for HDMI */
        if (num_registered_fb > 0)
                fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
        if (num_registered_fb > 1)
                fb_blank(registered_fb[1], FB_BLANK_NORMAL);
/*[ECID:000000] ZTEBSP zhangbo update for ics, start*/

#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
#if 1
	cpufreq_save_default_governor();
	cpufreq_set_conservative_governor();
        cpufreq_set_conservative_governor_param("up_threshold",
			SET_CONSERVATIVE_GOVERNOR_UP_THRESHOLD);

	cpufreq_set_conservative_governor_param("down_threshold",
			SET_CONSERVATIVE_GOVERNOR_DOWN_THRESHOLD);

	cpufreq_set_conservative_governor_param("freq_step",
		SET_CONSERVATIVE_GOVERNOR_FREQ_STEP);
#endif
#endif
/*[ECID:000000] ZTEBSP zhangbo update for ics, end*/
}

static void whistler_panel_late_resume(struct early_suspend *h)
{
	unsigned i;
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_restore_default_governor();
#endif
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
	//ZTEBSP DangXiao 20120618,start
	panel_early_suspend_flag = 0;
	printk("Ninja: panel late resume, Flag =0!\n ");
	//ZTEBSP DangXiao 20120618,end
}
#endif

//[ECID:000000] ZTEBSP DangXiao 20111116 start, for qHD DSI LCD
extern int pm_tps6586x_LDO9_switch_ctl(bool onoff);
extern enum e_lcd_vendor_type LCD_Vendor_Type;
//[ECID:000000] ZTEBSP DangXiao 20111116 end
int __init whistler_panel_init(void)
{
	int err;
	struct resource __maybe_unused *res;

	tegra_gpio_enable(whistler_hdmi_hpd);
	gpio_request(whistler_hdmi_hpd, "hdmi_hpd");
	gpio_direction_input(whistler_hdmi_hpd);
	//[ECID:000000] ZTEBSP DangXiao 20120514 start, to adapt different LCD module vendors.
	switch(LCD_Vendor_Type)
	{
		//P943T  qHD-panel
		case panel_nt35516_sharp:
			printk("== LCD vendor ID is : panel_nt35516_sharp ! ==\r\n");
			break;
		case panel_nt35516_lead:
		case panel_nt35516_truly:
			whistler_dsi_out.dsi_init_cmd =&dsi_lead_init_cmd ;
			whistler_dsi_out.n_init_cmd = ARRAY_SIZE(dsi_lead_init_cmd),
			printk("== LCD vendor ID is : panel_nt35516_lead & truly! ==\r\n");
			break;
		case panel_nt35516_success:
			whistler_dsi_out.dsi_init_cmd =&dsi_success_init_cmd ;
			whistler_dsi_out.n_init_cmd = ARRAY_SIZE(dsi_success_init_cmd),
			printk("== LCD vendor ID is : panel_nt35516_success ! ==\r\n");			
			break;
		case panel_nt35516_tianma:
			printk("== LCD vendor ID is : panel_nt35516_tianma ! ==\r\n");
			break;
		case panel_nt35516_boe:
			printk("== LCD vendor ID is : panel_nt35516_boe ! ==\r\n");
			break;
		case panel_otm9608a_boe:
			whistler_dsi_out.dsi_init_cmd =&dsi_otm9608a_boe_init_cmd ;
			whistler_dsi_out.n_init_cmd = ARRAY_SIZE(dsi_otm9608a_boe_init_cmd),
			printk("== LCD vendor ID is : panel_otm9608a_boe ! ==\r\n");
			break;
		
		default:
			break;
	}
	
	//[ECID:000000] ZTEBSP DangXiao 20120514 end, to adapt different LCD module vendors.

#ifdef CONFIG_HAS_EARLYSUSPEND
	whistler_panel_early_suspender.suspend = whistler_panel_early_suspend;
	whistler_panel_early_suspender.resume = whistler_panel_late_resume;
	whistler_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&whistler_panel_early_suspender);
#endif
	whistler_carveouts[1].base = tegra_carveout_start;
	whistler_carveouts[1].size = tegra_carveout_size;

	err = platform_add_devices(whistler_gfx_devices,
				   ARRAY_SIZE(whistler_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&whistler_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;
#endif

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
		min(tegra_fb_size, tegra_bootloader_fb_size));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&whistler_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;

	//[ECID:000000] ZTEBSP DangXiao NVIDIA Martin start 20111010 for chaos Display
	/* Copy the bootloader fb to the fb. */		 
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,							  
							min(tegra_fb_size, tegra_bootloader_fb_size));
	//[ECID:000000] ZTEBSP DangXiao NVIDIA Martin end 20111010

	if (!err)
		err = nvhost_device_register(&whistler_disp1_device);

	if (!err)
		err = nvhost_device_register(&whistler_disp2_device);
#endif


	/*[ECID:000000] ZTEBSP zhangbo 20120105, add led driver, start*/
	platform_device_register(&whistler_led_backlight_device);
	/*[ECID:000000] ZTEBSP zhangbo 20120105, add led driver, end*/

	return err;
}

