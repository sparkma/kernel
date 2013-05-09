/*
 * arch/arm/mach-tegra/board-whistler.h
 *
 * Copyright (C) 2010 Google, Inc.
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

#ifndef _MACH_TEGRA_BOARD_WHISTLER_H
#define _MACH_TEGRA_BOARD_WHISTLER_H

int whistler_regulator_init(void);
int whistler_sdhci_init(void);
int whistler_pinmux_init(void);
int whistler_panel_init(void);
int whistler_kbc_init(void);
int whistler_sensors_init(void);
int whistler_baseband_init(void);
int whistler_emc_init(void);

/* Interrupt numbers from external peripherals */
//[ECID:000000] ZTEBSP weiguohua 20120305 start, for codec
/* Audio-related GPIOs */
#if 0
    #define TEGRA_GPIO_HP_DET		TEGRA_GPIO_PW3
#endif
//[ECID:000000] ZTEBSP weiguohua 20120305 start, for codec

//[ECID:000000] ZTEBSP maoke 20111001 start, for skate bring up
#define TPS6586X_INT_BASE    TEGRA_NR_IRQS
#define TPS6586X_INT_END     (TPS6586X_INT_BASE + 32)
#define TPS6586X_GPIO_BASE   TEGRA_NR_GPIOS
//[ECID:000000] ZTEBSP maoke 20111001 end, for skate bring up

/*[ECID:0000]ZTEBSP yuxin start for cam mipi 2011.09.21 */
#define CAM_1V8_GPIO_EN (TPS6586X_GPIO_BASE + 2) /* gpio3 ? */
#define CAM_2V8_GPIO_EN (TPS6586X_GPIO_BASE + 3) /* gpio4 ? */
/*[ECID:0000]ZTEBSP yuxin end for cam mipi 2011.09.21 */

//[ECID:000000] ZTEBSP jiaobaocun start 20120301, for Codec
/* Audio-related GPIOs */
#define TEGRA_GPIO_HP_DET		TEGRA_GPIO_PW3
#define TEGRA_GPIO_HBT_DET		TEGRA_GPIO_PW2
#define TEGRA_CODEC_GPIO_RESET  TEGRA_GPIO_PX0
#define TEGRA_CODEC_SPI_CS		TEGRA_GPIO_PN4	
//[ECID:000000] ZTEBSP jiaobaocun end 20120301, for Codec
/* TCA6416 GPIO expander */
#define TPS6586X_GPIO_BASE   TEGRA_NR_GPIOS

/* Invensense MPU Definitions */
//[ECID:000000] ZTEBSP wanghaifei start 20120220, for sensor init
#define MPU_GYRO_NAME           "mpu3050"
#define MPU_GYRO_IRQ_GPIO       TEGRA_GPIO_PX1
#define MPU_GYRO_ADDR           0x68
#define MPU_GYRO_BUS_NUM        0
#define MPU_GYRO_ORIENTATION    { 1, 0, 0, 0, 1, 0, 0, 0, 1 }

#define MPU_MMA845X_NAME        "mma845x"
#define MPU_LIS3DH_NAME         "lis3dh"
#define MPU_KXTF9_NAME		"kxtf9"
#define MPU_ACCEL_IRQ_GPIO1     TEGRA_GPIO_PK4
#define MPU_ACCEL_IRQ_GPIO2     TEGRA_GPIO_PJ2
#define MPU_MMA845X_ADDR        0x1D
#define MPU_LIS3DH_ADDR         0x19
#define MPU_KXTF9_ADDR		0x0f
#define MPU_ACCEL_BUS_NUM       0
#define MPU_ACCEL_ORIENTATION   { 0, 1, 0, -1, 0, 0, 0, 0, 1 }

#define MPU_COMPASS_NAME        "ak8975"
#define MPU_COMPASS_IRQ_GPIO    TEGRA_GPIO_PK3
#define MPU_COMPASS_ADDR        0x0C
#define MPU_COMPASS_BUS_NUM     4
#define MPU_COMPASS_ORIENTATION { 0, 1, 0, -1, 0, 0, 0, 0, 1 }

#define TMD2771_IRQ_GPIO	TEGRA_GPIO_PG1
//[ECID:000000] ZTEBSP wanghaifei end 20120220, for sensor init

#endif
