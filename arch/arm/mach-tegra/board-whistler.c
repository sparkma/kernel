/*
 * arch/arm/mach-tegra/board-whistler.c
 *
 * Copyright (c) 2010 - 2011, NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/dma-mapping.h>
#include <linux/spi-tegra.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/gpio_scrollwheel.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
#include <linux/mfd/tps6586x.h>
//#include <linux/mfd/max8907c.h>
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up
#include <linux/mfd/max8907c.h>
#include <linux/memblock.h>
#include <linux/tegra_uart.h>
//[ECID:000000] ZTEBSP liudongmei 20111022 start, for codec reset
#include <linux/aic326x.h>
//[ECID:000000] ZTEBSP liudongmei 20111022 end, for codec reset
//[ECID:000000] ZTEBSP maoke 20111104 start, for pwr key
#include <linux/pwr_key.h>
//[ECID:000000] ZTEBSP maoke 20111104 end, for pwr key

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <mach/tegra_wm8753_pdata.h>
//[ECID:000000] ZTEBSP jiaobaocun 20120220 start, for Codec
#include <mach/tegra_aic326x_pdata.h>
//[ECID:000000] ZTEBSP jiaobaocun 20120220 end, for Codec
#include <sound/tlv320aic326x.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>

#include "board.h"
#include "clock.h"
#include "board-whistler.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
//[ECID:000000]ZTE BSP khl modify for gpio_keys start
#include "wakeups-t2.h"
#include <linux/gpio_keys.h>
//[ECID:000000]ZTE BSP khl modify for gpio_keys end
#include "board-whistler-baseband.h"
//[ECID:000000] ZTEBSP jiaobaocun 20120308 start, for Codec
extern int zte_get_hw_board_id(void);
//[ECID:000000] ZTEBSP jiaobaocun 20120308 end, for Codec
//nv-modify,zhangbo,20120227,for ram reserve
#define SZ_3M (SZ_1M + SZ_2M)
#define SZ_4M (SZ_2M + SZ_2M)
#define SZ_152M (SZ_128M + SZ_16M + SZ_8M)
#define USB1_VBUS_GPIO TCA6416_GPIO_BASE

/*[ECID:000000] ZTEBSP zhangbo update for ics, start*/
static struct plat_serial8250_port debug_uart_platform_data[] = {
	{
		/*[ECID:000000] ZTEBSP zhangbo173794 20111002 start,uart change*/
		.membase	= IO_ADDRESS(TEGRA_UARTD_BASE),
		.mapbase	= TEGRA_UARTD_BASE,
		.irq		= INT_UARTD,
		/*[ECID:000000] ZTEBSP zhangbo173794 20111002 end,uart change*/
		.flags		= UPF_BOOT_AUTOCONF | UPF_FIXED_TYPE,
		.type           = PORT_TEGRA,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 216000000,
	}, {
		.flags		= 0,
	}
};

static struct platform_device debug_uart = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = debug_uart_platform_data,
	},
};
#if 0
static struct plat_serial8250_port debug_uartb_platform_data[] = {
	{
		.membase	= IO_ADDRESS(TEGRA_UARTB_BASE),
		.mapbase	= TEGRA_UARTB_BASE,
		.irq		= INT_UARTB,
		.flags		= UPF_BOOT_AUTOCONF | UPF_FIXED_TYPE,
		.type           = PORT_TEGRA,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 216000000,
	}, {
		.flags		= 0,
	}
};

static struct platform_device debug_uartb = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = debug_uartb_platform_data,
	},
};

static struct plat_serial8250_port debug_uarta_platform_data[] = {
	{
		.membase	= IO_ADDRESS(TEGRA_UARTA_BASE),
		.mapbase	= TEGRA_UARTA_BASE,
		.irq		= INT_UARTA,
		.flags		= UPF_BOOT_AUTOCONF | UPF_FIXED_TYPE,
		.type           = PORT_TEGRA,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 216000000,
	}, {
		.flags		= 0,
	}
};

static struct platform_device debug_uarta = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = debug_uarta_platform_data,
	},
};
#endif
/*[ECID:000000] ZTEBSP zhangbo update for ics, end*/

static struct platform_device *whistler_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	/*[ECID:000000] ZTEBSP zhangbo173794 20111002 start,uart change*/
        &tegra_uartd_device,
	/*[ECID:000000] ZTEBSP zhangbo173794 20111002 end,uart change*/
};

struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "pll_p"},
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
};

static struct tegra_uart_platform_data whistler_uart_pdata;

static void __init uart_debug_init(void)
{
	unsigned long rate;
	//struct clk *debug_uart_clk;
	struct clk *c;
	#if 0
	int modem_id = tegra_get_modem_id();

	if (modem_id == 0x1) {
    #endif
		/* UARTB is the debug port. */
	/*[ECID:000000] ZTEBSP zhangbo173794 20111002 start,uart change*/
	pr_info("Selecting UARTD as the debug console\n");
	whistler_uart_devices[3] = &debug_uart;
	debug_uart_clk = clk_get_sys("serial8250.0", "uartd");
	//ZTEBSP DangXiao 20120314, for uart_d bug.
	debug_uart_port_base = ((struct plat_serial8250_port *)(
							debug_uartd_device.dev.platform_data))->mapbase;
	/*[ECID:000000] ZTEBSP zhangbo173794 20111002 end,uart change*/

		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
		rate = debug_uart_platform_data[0].uartclk;
		//	rate = debug_uartb_platform_data[0].uartclk;
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, rate);
		} else {
			pr_err("Not getting the clock %s for debug console\n",
						debug_uart_clk->name);
#if 0
		}
	} else {
		/* UARTA is the debug port. */
		pr_info("Selecting UARTA as the debug console\n");
		whistler_uart_devices[0] = &debug_uarta;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");

		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			rate = debug_uarta_platform_data[0].uartclk;
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, rate);
		} else {
			pr_err("Not getting the clock %s for debug console\n",
						debug_uart_clk->name);
		}
#endif
	}
}

static void __init whistler_uart_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	whistler_uart_pdata.parent_clk_list = uart_parent_clk;
	whistler_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);

	tegra_uarta_device.dev.platform_data = &whistler_uart_pdata;
	tegra_uartb_device.dev.platform_data = &whistler_uart_pdata;
	tegra_uartc_device.dev.platform_data = &whistler_uart_pdata;
//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
        tegra_uartd_device.dev.platform_data = &whistler_uart_pdata;
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up

	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(whistler_uart_devices,
				ARRAY_SIZE(whistler_uart_devices));
}

#ifdef CONFIG_BCM4329_RFKILL
//[ECID:0000]  ZTEBSP dingzhihui for bt power up ,20120210,begin
static struct resource whistler_bcm4329_rfkill_resources[] = {
	{
		.name	= "bcm4329_nreset_gpio",
		.start	= TEGRA_GPIO_PR3,
		.end	= TEGRA_GPIO_PR3,
		.flags	= IORESOURCE_IO,
	},
};
//[ECID:0000]  ZTEBSP dingzhihui for bt power up ,20120210,end
static struct platform_device whistler_bcm4329_rfkill_device = {
	.name		= "bcm4329_rfkill",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(whistler_bcm4329_rfkill_resources),
	.resource	= whistler_bcm4329_rfkill_resources,
};
//[ECID:000000] ZTEBSP fanjiankang added begin
static noinline void __init whistler_bt_rfkill(void)
{
//[ECID:000000] ZTEBSP zhangqi 20111014 start for Wifi related defines modify bcm4330
	/*Add Clock Resource*/
	clk_add_alias("bcm4329_32k_clk", whistler_bcm4329_rfkill_device.name, \
				"blink", NULL);
//[ECID:000000] ZTEBSP zhangqi 20111014 end for Wifi related defines modify bcm4330
	platform_device_register(&whistler_bcm4329_rfkill_device);
	return;
}
#else
static inline void whistler_bt_rfkill(void) { }
#endif
//[ECID:000000] ZTEBSP fanjiankang added end

/*[ECID:000000] ZTEBSP zhangbo update for ics, start*/
static struct resource whistler_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PC7,
			.end    = TEGRA_GPIO_PC7,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
			.start  = TEGRA_GPIO_PR4,
			.end    = TEGRA_GPIO_PR4,
			.flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.start  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PC7),
			.end    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PC7),
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device whistler_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(whistler_bluesleep_resources),
	.resource       = whistler_bluesleep_resources,
};

static void __init whistler_setup_bluesleep(void)
{
	platform_device_register(&whistler_bluesleep_device);
	tegra_gpio_enable(TEGRA_GPIO_PC7);
	tegra_gpio_enable(TEGRA_GPIO_PR4);
	return;
}
/*[ECID:000000] ZTEBSP zhangbo update for ics, end*/

static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
		},
	[1] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
		},
};

static struct tegra_ulpi_config ulpi_phy_config = {
//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
	.reset_gpio = -1,
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up
	.clk = "cdev2",
};

static __initdata struct tegra_clk_init_table whistler_clk_init_table[] = {
	/* name		parent		rate		enabled */
//[ECID:000000] ZTEBSP zhangqi 20111014 start for Wifi related defines modify bcm4330
	{ "blink",	"clk_32k",	32768,		false},
//[ECID:000000] ZTEBSP zhangqi 20111014 end for Wifi related defines modify bcm4330
	{ "pwm",	"clk_32k",	32768,		false},
	{ "kbc",	"clk_32k",	32768,		true},
	{ "sdmmc2",	"pll_p",	25000000,	false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s2",	"pll_a_out0",	0,		false},
//[ECID:000000] ZTEBSP chengxin 20120220 start, for TI codec 
	//{ "spi",	"clk_m",	12000000,	true},
//[ECID:000000] ZTEBSP chengxin 20120220 end, for TI codec 	
	{ "spdif_out",	"pll_a_out0",	0,		false},
//[ECID:000000] ZTEBSP DangXiao 2011116,start Oscar qHD DSI LCD
	{ "pll_d",	NULL,		216000000,	true},  /*zte lipeng10094834 add  at 2011.07.30*/  
//[ECID:000000] ZTEBSP DangXiao 2011116,end Oscar qHD DSI LCD
	{ NULL,		NULL,		0,		0},
};

static struct tegra_i2c_platform_data whistler_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	//ZTEBSP DangXiao 20120317, set I2C slave addr to Non-0x00  for wake up failure case, start
	.slave_addr = 0xFC,
	//ZTEBSP DangXiao 20120317, set I2C slave addr to Non-0x00  for wake up failure case, end
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup	= TEGRA_PINGROUP_DDC,
	.func		= TEGRA_MUX_I2C2,
};

static const struct tegra_pingroup_config i2c2_gen2 = {
	.pingroup	= TEGRA_PINGROUP_PTA,
	.func		= TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data whistler_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 2,
	//[ECID:000000] ZTEBSP liudongmei 20111025 start, for voice
	.bus_clk_rate	= { 400000, 400000 },	//[ECID:000000] ZTEBSP zhangzhao 2011-11-11 change ddc i2c clock to 400k
	//[ECID:000000] ZTEBSP liudongmei 20111025 end, for voice
	.bus_mux	= { &i2c2_ddc, &i2c2_gen2 },
	.bus_mux_len	= { 1, 1 },
	//ZTEBSP DangXiao 20120317, set I2C slave addr to Non-0x00  for wake up failure case, start
	.slave_addr = 0xFC,
	//ZTEBSP DangXiao 20120317, set I2C slave addr to Non-0x00  for wake up failure case, end
	.scl_gpio		= {0, TEGRA_GPIO_PT5},
	.sda_gpio		= {0, TEGRA_GPIO_PT6},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data whistler_i2c3_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	//ZTEBSP DangXiao 20120317, set I2C slave addr to Non-0x00  for wake up failure case, start
	.slave_addr = 0xFC,
	//ZTEBSP DangXiao 20120317, set I2C slave addr to Non-0x00  for wake up failure case, end
	.scl_gpio		= {TEGRA_GPIO_PBB2, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB3, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data whistler_dvc_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_dvc		= true,
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct aic326x_pdata whistler_aic3262_pdata = {
	/* debounce time */
	.debounce_time_ms = 512,
	//[ECID:000000] ZTEBSP weiguohua 20120305 start, for codec
	.reset_pin = TEGRA_CODEC_GPIO_RESET,
	.cspin = TEGRA_CODEC_SPI_CS,
	//[ECID:000000] ZTEBSP weiguohua 20120305 end, for codec
};


//[ECID:000000] ZTEBSP weiguohua 20120305 start, for codec
#if 0
static struct i2c_board_info __initdata wm8753_board_info[] = {
	{
		I2C_BOARD_INFO("wm8753", 0x1a),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_HP_DET),
	},
	{
		I2C_BOARD_INFO("aic3262-codec", 0x18),
		.platform_data = &whistler_aic3262_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_HP_DET),
	},
};

#else
static struct i2c_board_info __initdata whistler_codec_info[] = {
	{
		I2C_BOARD_INFO("aic3262-codec", 0x18),
		.platform_data = &whistler_aic3262_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_HBT_DET),
	},
	{},
};
#endif
//[ECID:000000] ZTEBSP jiaobaocun 20120308 start, for Codec
static struct spi_board_info  __initdata aic3326x_spi_board_info[] = {
	/* spi slave */
	{
		.modalias = "aic3262-codec",
		.bus_num =2,
		.chip_select =2,
		.mode = SPI_MODE_1,
		.max_speed_hz = 18000000,
		.platform_data = &whistler_aic3262_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_HBT_DET),

	},
};

struct spi_clk_parent spi_parent_clk[] = {
        [0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
        [1] = {.name = "pll_m"},
        [2] = {.name = "clk_m"},
#else
        [1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data whistler_spi_pdata = {
        .is_dma_based           = true,
        .max_dma_buffer         = (16 * 1024),
        .is_clkon_always        = false,
        .max_rate               = 100000000,
};

//[ECID:000000] ZTEBSP jiaobaocun 20120308 end, for Codec
//[ECID:000000] ZTEBSP weiguohua 20120305 end, for codec
static void whistler_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &whistler_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &whistler_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &whistler_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &whistler_dvc_platform_data;

	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);
//[ECID:000000] ZTEBSP weiguohua 20120305 start, for codec
#if 0
	i2c_register_board_info(4, wm8753_board_info,
		ARRAY_SIZE(wm8753_board_info));
#endif
//[ECID:000000] ZTEBSP weiguohua 20120305 end, for codec
}

//[ECID:000000] ZTEBSP DangXiao 20111118 ,start, for keypad gpio
#ifdef CONFIG_KEYBOARD_GPIO
#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

static struct gpio_keys_button whistler_keys[] = {
	//[0] = GPIO_KEY(KEY_FIND, PQ3, 0),
	//[1] = GPIO_KEY(KEY_HOME, PQ1, 0),
	//[2] = GPIO_KEY(KEY_BACK, PQ2, 0),
	[0] = GPIO_KEY(KEY_VOLUMEUP, PQ1, 0),  //DangXiao
	[1] = GPIO_KEY(KEY_VOLUMEDOWN, PQ0, 0),
	//[5] = GPIO_KEY(KEY_POWER, PV2, 1),
	//[6] = GPIO_KEY(KEY_MENU, PC7, 0),
};

#define PMC_WAKE_STATUS 0x14
/*
static int whistler_wakeup_key(void)
{
	unsigned long status =
		readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);

	return status & TEGRA_WAKE_GPIO_PV2 ? KEY_POWER : KEY_RESERVED;
}
*/
static struct gpio_keys_platform_data whistler_keys_platform_data = {
	.buttons	= whistler_keys,
	.nbuttons	= ARRAY_SIZE(whistler_keys),
	//.wakeup_key	= whistler_wakeup_key,
};

static struct platform_device whistler_keys_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev	= {
		.platform_data	= &whistler_keys_platform_data,
	},
};

static void whistler_keys_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(whistler_keys); i++)
		tegra_gpio_enable(whistler_keys[i].gpio);
}
#endif

//[ECID:000000] ZTEBSP DangXiao 20111118 ,end for keypad gpio
//[ECID:000000] ZTEBSP maoke 20111104 start, for pwr key
struct power_key_platform_data power_key_platform_data = {
        .pwrkey_pin = TEGRA_GPIO_PS0,
};

static struct platform_device power_key_device = {
        .name           = "power_key",
        .id             = -1,
        .dev    = {
                .platform_data = &power_key_platform_data,
        },
};

static void whistle_power_key_register(void)
{
        tegra_gpio_enable(power_key_platform_data.pwrkey_pin);

        platform_device_register(&power_key_device);
}
//[ECID:000000] ZTEBSP maoke 20111104 end, for pwr key
#define GPIO_SCROLL(_pinaction, _gpio, _desc)	\
{	\
	.pinaction = GPIO_SCROLLWHEEL_PIN_##_pinaction, \
	.gpio = TEGRA_GPIO_##_gpio,	\
	.desc = _desc,	\
	.active_low = 1,	\
	.debounce_interval = 2,	\
}

static struct gpio_scrollwheel_button scroll_keys[] = {
	[0] = GPIO_SCROLL(ONOFF, PR3, "sw_onoff"),
	[1] = GPIO_SCROLL(PRESS, PQ5, "sw_press"),
	[2] = GPIO_SCROLL(ROT1, PQ3, "sw_rot1"),
	[3] = GPIO_SCROLL(ROT2, PQ4, "sw_rot2"),
};

static struct gpio_scrollwheel_platform_data whistler_scroll_platform_data = {
	.buttons = scroll_keys,
	.nbuttons = ARRAY_SIZE(scroll_keys),
};

static struct platform_device whistler_scroll_device = {
	.name	= "alps-gpio-scrollwheel",
	.id	= 0,
	.dev	= {
		.platform_data	= &whistler_scroll_platform_data,
	},
};

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct tegra_wm8753_platform_data whistler_audio_pdata = {
	.gpio_spkr_en = -1,
	.gpio_hp_det = TEGRA_GPIO_HP_DET,
	.gpio_hp_mute = -1,
	.gpio_int_mic_en = -1,
	.gpio_ext_mic_en = -1,
	.debounce_time_hp = 200,
};
//[ECID:000000] ZTEBSP weiguohua 20120305 start, for codec
static struct tegra_aic326x_platform_data  whistler_tegra_aic326x_platform_data ={

	.gpio_spkr_en = -1,
	.gpio_hp_det = TEGRA_GPIO_HP_DET,
	.gpio_hp_mute = -1,
	.gpio_int_mic_en = -1,
	.gpio_ext_mic_en = -1,
};
//[ECID:000000] ZTEBSP weiguohua 20120305 end, for codec
static struct platform_device whistler_audio_device1 = {
	.name	= "tegra-snd-aic326x",
	.id	= 0,
	.dev	= {
	//[ECID:000000] ZTEBSP weiguohua 20120305 start, for codec
	#if 0
		.platform_data  = &whistler_audio_pdata,
	#else
		.platform_data  = &whistler_tegra_aic326x_platform_data,
	#endif
	//[ECID:000000] ZTEBSP weiguohua 20120305 end, for codec	
	},
};

static struct platform_device whistler_audio_device2 = {
	.name	= "tegra-snd-wm8753",
	.id	= 0,
	.dev	= {
		.platform_data  = &whistler_audio_pdata,
	},
};

static struct platform_device *whistler_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_udc_device,
	&tegra_gart_device,
	&tegra_aes_device,
	&tegra_wdt_device,
	&tegra_avp_device,
//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
	//&whistler_scroll_device,
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up
#ifdef CONFIG_KEYBOARD_GPIO
	&whistler_keys_device,
#endif
	&tegra_camera,
	&tegra_i2s_device1,
	&tegra_i2s_device2,
//[ECID:000000] ZTEBSP liudongmei 20120420 start, for spdif
	&tegra_spdif_device,
//[ECID:000000] ZTEBSP liudongmei 20120420 end, for spdif	
	&tegra_das_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&baseband_dit_device,
//[ECID:000000] ZTEBSP dingzhihui 20120210 for bt power up start
//	&whistler_bcm4329_rfkill_device,
//[ECID:000000] ZTEBSP dingzhihui 20120210 for bt power up end
	&tegra_pcm_device,
	&whistler_audio_device1,
//[ECID:000000] ZTEBSP jiaobaocun 20120220 start, for Codec
	//&whistler_audio_device2,
//[ECID:000000] ZTEBSP jiaobaocun 20120220 end, for Codec
};

//[ECID:000000] ZTEBSP zhangzhao 20111009, add tsc driver start
static struct synaptics_ts_platform_data synaptics_pdata = {
	//.flags			= SYNAPTICS_FLIP_X | SYNAPTICS_FLIP_Y | SYNAPTICS_SWAP_XY,
	 .maxx = 539,  //479,      
        .maxy = 959,  //799,       // for tm1726
       .model = 2010,
       .x_plate_ohms = 300,
	.irq_flags		= IRQF_TRIGGER_LOW,
};

static const struct i2c_board_info whistler_i2c_touch_info[] = {
	{
		I2C_BOARD_INFO("synaptics-rmi-ts", 0x22),
		.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PC6), //[ECID:0000] zhangzhao 2012-3-20 delete
		.platform_data	= &synaptics_pdata,
	},
};

//[ECID:000000] ZTEBSP shiyan 20111014, for charge	
static struct i2c_board_info tps6586x_battery_info[] = {
	{
		I2C_BOARD_INFO("tps6586x-battery", 0x0B),   //0x0B is bq20z75 's i2c addr   not use 
	},
};
//[ECID:000000] ZTEBSP shiyan 20111014, for charge	

static int __init whistler_touch_init(void)
{
      gpio_request(TEGRA_GPIO_PC6, "touch-irq");	  
      gpio_direction_input(TEGRA_GPIO_PC6); 

	tegra_gpio_enable(TEGRA_GPIO_PC6);
	printk(KERN_ERR "enable tsc gpio_int");		
	i2c_register_board_info(2, whistler_i2c_touch_info, 1);

//[ECID:000000] ZTEBSP shiyan 20111014, for charge	
	i2c_register_board_info(2, tps6586x_battery_info, 1);	
//[ECID:000000] ZTEBSP shiyan 20111014, for charge	

	return 0;
}
//[ECID:000000] ZTEBSP zhangzhao 20111009, add tsc driver end

//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
#if 0
static int __init whistler_scroll_init(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(scroll_keys); i++)
		tegra_gpio_enable(scroll_keys[i].gpio);

	return 0;
}
#endif
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up

//[ECID:000000] ZTEBSP DangXiao start ,20111012 Add MHL driver

#define CONFIG_MHL_Sii8334
#ifdef CONFIG_MHL_Sii8334
#define CI2CA  true  // CI2CA depend on the CI2CA pin's level   
#ifdef CI2CA 
#define SII8334_plus 0x02  //Define sii8334's I2c Address of all pages by the status of CI2CA.
#else
#define SII8334_plus 0x00  //Define sii8334's I2c Address of all pages by the status of CI2CA.
#endif

#define Sii8834_RESET_N TEGRA_GPIO_PU5
#define Sii8334_HOTPLUG_DETECT  TEGRA_GPIO_PG0   //GMI_AD0
static void whistler_Sii8334_reset(void)
{		
	printk("Ninja: enter %s , MHL reset!! \n",__FUNCTION__);		
	tegra_gpio_enable(Sii8834_RESET_N);
	gpio_request(Sii8834_RESET_N, "Sii8334_mhl_reset");
	gpio_direction_output(Sii8834_RESET_N, 1);
	gpio_export(Sii8834_RESET_N, false);

	gpio_set_value(Sii8834_RESET_N, 0);
	msleep(100);
	gpio_set_value(Sii8834_RESET_N, 1);
}
struct MHL_platform_data {
	void (*reset) (void);
};
static struct MHL_platform_data Sii8334_data = {
	.reset = whistler_Sii8334_reset,
};
#endif

static const struct i2c_board_info whistler_i2c_mhl_info[] = {
#ifdef CONFIG_MHL_Sii8334
		{
		 .type = "sii8334_PAGE_TPI",
		 .addr = 0x39 + SII8334_plus, 
		 .irq = TEGRA_GPIO_TO_IRQ(Sii8334_HOTPLUG_DETECT),  //define the interrupt signal input pin
		 .platform_data = &Sii8334_data,
		},
		{
		 .type = "sii8334_PAGE_TX_L0",
		 .addr = 0x39 + SII8334_plus, 
		},
		{
		 .type = "sii8334_PAGE_TX_L1",
		 .addr = 0x3d + SII8334_plus, 
		},
		{
		 .type = "sii8334_PAGE_TX_2",
		 .addr = 0x49 + SII8334_plus, 
		},
		{
		 .type = "sii8334_PAGE_TX_3",
		 .addr = 0x4d + SII8334_plus, 
		},
		{
		 .type = "sii8334_PAGE_CBUS",
		 .addr = 0x64 + SII8334_plus, 
		},
	#endif
};

static int __init whistler_mhl_init(void)
{
      gpio_request(Sii8334_HOTPLUG_DETECT, "mhl-irq");	  
      gpio_direction_input(Sii8334_HOTPLUG_DETECT); 

	tegra_gpio_enable(Sii8334_HOTPLUG_DETECT);
	printk(KERN_ERR "enable mhl gpio_int");		
	
	//[ECID:000000] ZTEBSP DangXiao  20111012,  MHL I2C Config, bus 0
	i2c_register_board_info(0, whistler_i2c_mhl_info,ARRAY_SIZE(whistler_i2c_mhl_info));
	printk( "Ninja:enter %s! \n",__FUNCTION__);	  
	return 0;
}
//[ECID:000000] ZTEBSP DangXiao end ,20111012 Add MHL driver


static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
			.vbus_irq = TPS6586X_INT_BASE + TPS6586X_INT_USB_DET,
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up
			.vbus_gpio = TEGRA_GPIO_PN6,
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
	[2] = {
			.instance = 2,
			.vbus_gpio = -1,
	},
};

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.default_enable = false,
		},
	[1] = {
			.phy_config = &ulpi_phy_config,
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.default_enable = false,
		},
	[2] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.default_enable = false,
	},
};

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci_pdata[0],
};

static int __init whistler_gps_init(void)
{
//[ECID:000000] ZTEBSP wanghaifei10129378 start 20111010, request gps power gpio
        struct clk *clk32 = clk_get_sys(NULL, "blink");
        int ret_value;
        if (!IS_ERR(clk32)) {
                clk_set_rate(clk32,clk32->parent->rate);
                clk_enable(clk32);
        }   

        //gpio_free(TEGRA_GPIO_PU4);
        ret_value = gpio_request(TEGRA_GPIO_PU4, "gps_power");
        if (ret_value) {
                printk("%s:%s:gps request TEGRA_GPIO_PU4 fail\n", __FILE__, __func__ );  
        }   
        gpio_direction_output(TEGRA_GPIO_PU4, 0); 
        gpio_export(TEGRA_GPIO_PU4, true);
//[ECID:000000] ZTEBSP wanghaifei10129378 end 20111010
	tegra_gpio_enable(TEGRA_GPIO_PU4);
	return 0;
}

static void whistler_power_off(void)
{
	int ret;

//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
	ret = tps6586x_power_off();
	//ret = max8907c_power_off();
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up
	if (ret)
		pr_err("whistler: failed to power off\n");

	while (1);
}

static void __init whistler_power_off_init(void)
{
	pm_power_off = whistler_power_off;
}

static void whistler_usb_init(void)
{
	tegra_usb_phy_init(tegra_usb_phy_pdata, ARRAY_SIZE(tegra_usb_phy_pdata));

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

}
//[ECID:000000] ZTEBSP weiguohua 20120305 start, for codec
static void whistler_codec_init(void)
{
	int ret = 0;
	ret = gpio_request(TEGRA_GPIO_HBT_DET, "headphone--button-detect-gpio");
	if (ret)
		pr_err("hp gpio request failed\n");

	ret = gpio_direction_input(TEGRA_GPIO_HBT_DET);
	if (ret)
		pr_err(" hp gpio direction error\n");

	tegra_gpio_enable(TEGRA_GPIO_HBT_DET);
	
       tegra_gpio_enable(TEGRA_CODEC_GPIO_RESET);

//[ECID:000000] ZTEBSP jiaobaocun 20120308 start, for Codec
	   int versionid = zte_get_hw_board_id();
	 printk("this board id is %d\n",versionid);
	     if(versionid == 6||versionid == 5||versionid == 4){//5 for test spi
                    int i;
                     struct clk *c;

                     printk("this borad  spi init\n");
                     for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
                             c = tegra_get_clock_by_name(spi_parent_clk[i].name);
							
                             if (IS_ERR_OR_NULL(c)) {
                                     pr_err("Not able to get the clock for %s\n",
                                            spi_parent_clk[i].name);
                                     continue;
                             }
                             spi_parent_clk[i].parent_clk = c;
                             spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
                     }
                     whistler_spi_pdata.parent_clk_list = spi_parent_clk;
                     whistler_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
                     tegra_spi_device2.dev.platform_data = &whistler_spi_pdata;
			platform_device_register(&tegra_spi_device3);
			spi_register_board_info(aic3326x_spi_board_info, ARRAY_SIZE(aic3326x_spi_board_info));
		}else if(versionid == 0||versionid == 1){
			printk("this borad  i2c init\n");
			i2c_register_board_info(2, whistler_codec_info, 1);
		}
		else{
                    int i;
                     struct clk *c;

                     printk("this borad  spi init\n");
                     for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
                             c = tegra_get_clock_by_name(spi_parent_clk[i].name);
							
                             if (IS_ERR_OR_NULL(c)) {
                                     pr_err("Not able to get the clock for %s\n",
                                            spi_parent_clk[i].name);
                                     continue;
                             }
                             spi_parent_clk[i].parent_clk = c;
                             spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
                     }
                     whistler_spi_pdata.parent_clk_list = spi_parent_clk;
                     whistler_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
                     tegra_spi_device2.dev.platform_data = &whistler_spi_pdata;
			platform_device_register(&tegra_spi_device3);
			spi_register_board_info(aic3326x_spi_board_info, ARRAY_SIZE(aic3326x_spi_board_info));
		}
			
//[ECID:000000] ZTEBSP jiaobaocun 20120308 end, for Codec
	ret = gpio_request(TEGRA_GPIO_PV1, "mic_detection");
	if (ret)
		pr_err("hp gpio request failed\n");

	ret = gpio_direction_output(TEGRA_GPIO_PV1,0);
	if (ret)
		pr_err(" hp gpio direction error\n");

	tegra_gpio_enable(TEGRA_GPIO_PV1);

}
//[ECID:000000] ZTEBSP weiguohua 20120305 end, for codec
static void __init tegra_whistler_init(void)
{
	// int modem_id = tegra_get_modem_id(); //[ECID:000000] ZTEBSP zhangbo 20120223, no modem id
	tegra_clk_init_from_table(whistler_clk_init_table);
	whistler_pinmux_init();
	whistler_i2c_init();
	whistler_uart_init();
	platform_add_devices(whistler_devices, ARRAY_SIZE(whistler_devices));
	tegra_ram_console_debug_init();
	whistler_sdhci_init();
	whistler_regulator_init();
	whistler_panel_init();
	whistler_sensors_init();
	whistler_touch_init();
//[ECID:000000] ZTEBSP jiaobaocun 20120220 start, for Codec
	whistler_codec_init();
//[ECID:000000] ZTEBSP jiaobaocun 20120220 end, for Codec
	//whistler_kbc_init();
//[ECID:000000]ZTE BSP khl modify for gpio_keys start
	#ifdef CONFIG_KEYBOARD_GPIO
	whistler_keys_init();
       #endif
//[ECID:000000]ZTE BSP khl modify for gpio_keys end
//[ECID:000000]ZTE BSP fanjiankang added begin
     whistler_bt_rfkill();
//[ECID:000000]ZTE BSP fanjiankang added end	
	whistler_gps_init();
	whistler_usb_init();
//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
	//whistler_scroll_init();
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up

//[ECID:000000] ZTEBSP maoke 20111104 start, for pwr key
    whistle_power_key_register();
//[ECID:000000] ZTEBSP weiguohua 20111230 end, for TI  codec 
	whistler_power_off_init();
//[ECID:000000] ZTEBSP khl modify for dvfs start, 2011.12.24
	whistler_emc_init();
//[ECID:000000] ZTEBSP khl modify for dvfs end, 2011.12.24

	//if (modem_id == 0x1) ////[ECID:000000] ZTEBSP zhangbo 20120223, no modem id
		whistler_baseband_init();

//[ECID:000000] ZTEBSP DangXiao 20120416, add MHL feature for P943F10,start
	#ifdef CONFIG_P943F10_MHL_FEATURE
	//whistler_mhl_init();
	#endif
//[ECID:000000] ZTEBSP DangXiao 20120416, add MHL feature for P943F10,end
    
	whistler_setup_bluesleep();
	tegra_release_bootloader_fb();
}

int __init tegra_whistler_protected_aperture_init(void)
{
	tegra_protected_aperture_init(tegra_grhost_aperture);
	return 0;
}

void __init tegra_whistler_reserve(void)
{
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	//nv-modify,zhangbo,20120227,for ram reserve
	//tegra_reserve(SZ_160M, SZ_8M, SZ_16M);
	tegra_reserve(SZ_152M, SZ_4M, SZ_2M);

	tegra_ram_console_debug_reserve(SZ_1M);
}

MACHINE_START(WHISTLER, "whistler")
	.boot_params    = 0x00000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_whistler_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_whistler_init,
MACHINE_END
