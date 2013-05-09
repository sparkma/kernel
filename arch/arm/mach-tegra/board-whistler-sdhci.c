/*
 * arch/arm/mach-tegra/board-whistler-sdhci.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011 NVIDIA Corporation.
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>

#include "gpio-names.h"
#include "board.h"

//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
//#define WHISTLER_WLAN_PWR	TEGRA_GPIO_PK5
#define WHISTLER_WLAN_RST	TEGRA_GPIO_PX4   //TEGRA_GPIO_PK6
#define WHISTLER_WLAN_INT	TEGRA_GPIO_PI5 //zhangqi add 20111009  
/* [ECID:000000] ZTEBSP wangjianping start 20111124 modify for T card detect */
#define WHISTLER_EXT_SDCARD_DETECT	TEGRA_GPIO_PS2
/* [ECID:000000] ZTEBSP wangjianping 20111124 end */
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up
//[ECID:000000]ZTEBSP zhangqi 20111010 start for wifi
static struct clk *wifi_32k_clk;
//[ECID:000000]ZTEBSP zhangqi 20111010 end for wifi


//#define WHISTLER_WLAN_PWR	TEGRA_GPIO_PK5
//#define WHISTLER_WLAN_RST	TEGRA_GPIO_PK6

#define WHISTLER_WLAN_WOW	TEGRA_GPIO_PU5


static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int whistler_wifi_status_register(
		void (*sdhcicallback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = sdhcicallback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int whistler_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int whistler_wifi_power(int on)
{
//[ECID:000000]ZTEBSP zhangqi 20111010 start for wifi
   
	gpio_set_value(WHISTLER_WLAN_RST, 0);
	mdelay(100);
	if (on)
	{
		clk_enable(wifi_32k_clk);
		printk("ventana_wifi_power enable\n");
	}
	else
	{
		clk_disable(wifi_32k_clk);
	}
	
	mdelay(200);
	//gpio_set_value(WHISTLER_WLAN_PWR, on);
	//mdelay(100);
	printk("enter %s wifi on=%d",__FUNCTION__,on);
	gpio_set_value(WHISTLER_WLAN_RST, on);
	
	mdelay(400);
//[ECID:000000]ZTEBSP zhangqi 20111010 end for wifi

	return 0;
}

static int whistler_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}


static struct wifi_platform_data whistler_wifi_control = {
	.set_power      = whistler_wifi_power,
	.set_reset      = whistler_wifi_reset,
	.set_carddetect = whistler_wifi_set_carddetect,
};

static struct resource wifi_resource[] = {
	[0] = {
		.name	= "bcm4329_wlan_irq",
		.start	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU5),
		.end	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU5),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device whistler_wifi_device = {
	.name           = "bcm4329_wlan",
	.id             = 1,
	.num_resources	= 1,
	.resource	= wifi_resource,
	.dev            = {
		.platform_data = &whistler_wifi_control,
	},
};

//[ECID 000000] fanjiankang modified begin 2012.2.9

static struct resource sdhci_resource0[] = {
	[0] = {
		.start  = INT_SDMMC1,
		.end    = INT_SDMMC1,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

#if 0
static struct resource sdhci_resource1[] = {
	[0] = {
		.start  = INT_SDMMC2,
		.end    = INT_SDMMC2,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC2_BASE,
		.end	= TEGRA_SDMMC2_BASE + TEGRA_SDMMC2_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};
#endif
//[ECID 000000] fanjiankang modified end 2012.2.9


static struct resource sdhci_resource2[] = {
	[0] = {
		.start  = INT_SDMMC3,
		.end    = INT_SDMMC3,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start  = INT_SDMMC4,
		.end    = INT_SDMMC4,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

//[ECID 000000] fanjiankang modified begin 2012.2.9

#if 0
static struct embedded_sdio_data embedded_sdio_data1 = {
	.cccr   = {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.cis  = {
		.vendor         = 0x02d0,
		.device         = 0x4329,
	},
};
#else
static struct embedded_sdio_data embedded_sdio_data0 = {
	.cccr   = {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.cis  = {
		.vendor         = 0x02d0,
		.device         = 0x4329,
	},
};

#endif
//[ECID 000000] fanjiankang modified end 2012.2.9

#if 0
static struct tegra_sdhci_platform_data tegra_sdhci_platform_data1 = {
	.mmc_data = {
		.register_status_notify	= whistler_wifi_status_register,
		.embedded_sdio = &embedded_sdio_data1,
		.built_in = 1,
	},
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.max_clk_limit = 25000000,
};
#else
static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.mmc_data = {
		.register_status_notify	= whistler_wifi_status_register,
		.embedded_sdio = &embedded_sdio_data0,
		.built_in = 1,
	},
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	//khl modify for 1024 radio chanel
	.max_clk_limit = 46000000,
	//khl modify for 1024 radio chanel
};

#endif

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.cd_gpio = WHISTLER_EXT_SDCARD_DETECT,
	.wp_gpio = -1,
	.power_gpio = -1,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.mmc_data = {
		.built_in = 1,
	}
};

//[ECID 000000] fanjiankang modified  begin 2012.2.9
#if 0
static struct platform_device tegra_sdhci_device1 = {
	.name		= "sdhci-tegra",
	.id		= 1,
	.resource	= sdhci_resource1,
	.num_resources	= ARRAY_SIZE(sdhci_resource1),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data1,
	},
};
#else
static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data0,
	},
};
#endif
//[ECID 000000] fanjiankang modified  end 2012.2.9


static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static int __init whistler_wifi_init(void)
{
//[ECID:000000]ZTEBSP zhangqi 20111010 start for wifi
	wifi_32k_clk = clk_get_sys(NULL, "blink");//
	if (IS_ERR(wifi_32k_clk)) {
		pr_err("%s: unable to get blink clock\n", __func__);
		return PTR_ERR(wifi_32k_clk);
	}


	gpio_request(WHISTLER_WLAN_INT, "wlan_int");
	gpio_request(WHISTLER_WLAN_RST, "wlan_rst");
	gpio_request(WHISTLER_WLAN_WOW, "bcmsdh_sdmmc");

	tegra_gpio_enable(WHISTLER_WLAN_INT);
	tegra_gpio_enable(WHISTLER_WLAN_RST);
	tegra_gpio_enable(WHISTLER_WLAN_WOW);

	gpio_direction_input(WHISTLER_WLAN_INT);
	gpio_direction_output(WHISTLER_WLAN_RST, 0);
	gpio_direction_input(WHISTLER_WLAN_WOW);

	platform_device_register(&whistler_wifi_device);
	return 0;
}
int __init whistler_sdhci_init(void)
{
    /* [ECID:000000] ZTEBSP wangjianping start 20120214 add SD card driver */
    tegra_gpio_enable(WHISTLER_EXT_SDCARD_DETECT);
    /* [ECID:000000] ZTEBSP wangjianping end 20120214 add SD card driver */

	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device2);
	//[ECID 000000] fanjiankang modified  begin 2012.2.9
	#if 0
	platform_device_register(&tegra_sdhci_device1);
	#else
	platform_device_register(&tegra_sdhci_device0);
	#endif
	//[ECID 000000] fanjiankang modified  end 2012.2.9

	whistler_wifi_init();
	return 0;
}
