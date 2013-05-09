/*
 * Copyright (C) 2011 NVIDIA, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>

#include "board-whistler.h"
#include "tegra2_emc.h"
#include "board.h"
#include "fuse.h"

static const struct tegra_emc_table whistler_emc_tables_samsung_1G_300Mhz[] = {
	{
		.rate = 25000,   /* SDRAM frquency */
		.regs = {
			0x00000002,   /* RC */
			0x00000006,   /* RFC */
			0x00000003,   /* RAS */
			0x00000003,   /* RP */
			0x00000006,   /* R2W */
			0x00000004,   /* W2R */
			0x00000002,   /* R2P */
			0x0000000B,   /* W2P */
			0x00000003,   /* RD_RCD */
			0x00000003,   /* WR_RCD */
			0x00000002,   /* RRD */
			0x00000002,   /* REXT */
			0x00000003,   /* WDV */
			0x00000005,   /* QUSE */
			0x00000004,   /* QRST */
			0x00000008,   /* QSAFE */
			0x0000000C,   /* RDV */
			0x0000004D,   /* REFRESH */
			0x00000000,   /* BURST_REFRESH_NUM */
			0x00000003,   /* PDEX2WR */
			0x00000003,   /* PDEX2RD */
			0x00000003,   /* PCHG2PDEN */
			0x00000008,   /* ACT2PDEN */
			0x00000001,   /* AR2PDEN */
			0x0000000B,   /* RW2PDEN */
			0x00000004,   /* TXSR */
			0x00000003,   /* TCKE */
			0x00000008,   /* TFAW */
			0x00000004,   /* TRPAB */
			0x00000008,   /* TCLKSTABLE */
			0x00000002,   /* TCLKSTOP */
			0x00000068,   /* TREFBW */
			0x00000000,   /* QUSE_EXTRA */
			0x00000003,   /* FBIO_CFG6 */
			0x00000000,   /* ODT_WRITE */
			0x00000000,   /* ODT_READ */
			0x00000282,   /* FBIO_CFG5 */
			0xA0AE04AE,   /* CFG_DIG_DLL */
			0x007C6010,   /* DLL_XFORM_DQS */
			0x00000000,   /* DLL_XFORM_QUSE */
			0x00000000,   /* ZCAL_REF_CNT */
			0x00000003,   /* ZCAL_WAIT_CNT */
			0x00000000,   /* AUTO_CAL_INTERVAL */
			0x00000000,   /* CFG_CLKTRIM_0 */
			0x00000000,   /* CFG_CLKTRIM_1 */
			0x00000000,   /* CFG_CLKTRIM_2 */
		}
	},
	{
		.rate = 50000,   /* SDRAM frequency */
		.regs = {
			0x00000003,   /* RC */
            0x00000007,   /* RFC */
            0x00000003,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x0000000B,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000003,   /* WDV */
            0x00000006,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000C,   /* RDV */
            0x0000009F,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000B,   /* RW2PDEN */
            0x00000007,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x000000D0,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000000,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xA0AE04AE,   /* CFG_DIG_DLL */
            0x007BD010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000005,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
		}
	},
	{
		.rate = 75000,   /* SDRAM frequency */
		.regs = {
			 0x00000005,   /* RC */
            0x0000000A,   /* RFC */
            0x00000004,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x0000000B,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000003,   /* WDV */
            0x00000006,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000C,   /* RDV */
            0x000000FF,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000B,   /* RW2PDEN */
            0x0000000B,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x00000138,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000000,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xA0AE04AE,   /* CFG_DIG_DLL */
            0x007C8010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000007,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
		}
	},
	{
		.rate = 150000,   /* SDRAM frequency */
		.regs = {
			 0x00000009,   /* RC */
            0x00000014,   /* RFC */
            0x00000007,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x0000000B,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000003,   /* WDV */
            0x00000006,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000C,   /* RDV */
            0x0000021F,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000B,   /* RW2PDEN */
            0x00000015,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x00000270,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000001,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xA07C04AE,   /* CFG_DIG_DLL */
            0x007E2010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x0000000E,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
		}
	},
	{
		.rate = 300000,   /* SDRAM frequency */
		.regs = {
			0x00000012,   /* RC */
            0x00000027,   /* RFC */
            0x0000000D,   /* RAS */
            0x00000006,   /* RP */
            0x00000007,   /* R2W */
            0x00000005,   /* W2R */
            0x00000003,   /* R2P */
            0x0000000B,   /* W2P */
            0x00000006,   /* RD_RCD */
            0x00000006,   /* WR_RCD */
            0x00000003,   /* RRD */
            0x00000003,   /* REXT */
            0x00000003,   /* WDV */
            0x00000007,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000009,   /* QSAFE */
            0x0000000D,   /* RDV */
            0x0000045F,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000004,   /* PDEX2WR */
            0x00000004,   /* PDEX2RD */
            0x00000006,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000F,   /* RW2PDEN */
            0x0000002A,   /* TXSR */
            0x00000003,   /* TCKE */
            0x0000000F,   /* TFAW */
            0x00000007,   /* TRPAB */
            0x00000007,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x000004E0,   /* TREFBW */
            0x00000006,   /* QUSE_EXTRA */
            0x00000002,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xE059048B,   /* CFG_DIG_DLL */
            0x007E0010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x0000001B,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
		}
	}
};

static const struct tegra_emc_table whistler_emc_tables_samsung_512M_300Mhz[] = {
	{
		.rate = 25000,   /* SDRAM frquency */
		.regs = {
	    0x00000002,   /* RC */
            0x00000006,   /* RFC */
            0x00000003,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x0000000B,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000003,   /* WDV */
            0x00000005,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000C,   /* RDV */
            0x0000004D,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000B,   /* RW2PDEN */
            0x00000004,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x00000068,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000003,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xA0B804AE,   /* CFG_DIG_DLL */
            0x007A4010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000003,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
		}
	},
	{
		.rate = 50000,   /* SDRAM frequency */
		.regs = {
            0x00000003,   /* RC */
            0x00000007,   /* RFC */
            0x00000003,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x0000000B,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000003,   /* WDV */
            0x00000006,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000C,   /* RDV */
            0x0000009F,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000B,   /* RW2PDEN */
            0x00000007,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x000000D0,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000000,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xA0B804AE,   /* CFG_DIG_DLL */
            0x007A4010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000005,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
		}
	},
	{
		.rate = 75000,   /* SDRAM frequency */
		.regs = {
            0x00000005,   /* RC */
            0x0000000A,   /* RFC */
            0x00000004,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x0000000B,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000003,   /* WDV */
            0x00000006,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000C,   /* RDV */
            0x000000FF,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000B,   /* RW2PDEN */
            0x0000000B,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x00000138,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000000,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xA0B804AE,   /* CFG_DIG_DLL */
            0x007AE010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x00000007,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
		}
	},
	{
		.rate = 150000,   /* SDRAM frequency */
		.regs = {
	    0x00000009,   /* RC */
            0x00000014,   /* RFC */
            0x00000007,   /* RAS */
            0x00000003,   /* RP */
            0x00000006,   /* R2W */
            0x00000004,   /* W2R */
            0x00000002,   /* R2P */
            0x0000000B,   /* W2P */
            0x00000003,   /* RD_RCD */
            0x00000003,   /* WR_RCD */
            0x00000002,   /* RRD */
            0x00000002,   /* REXT */
            0x00000003,   /* WDV */
            0x00000006,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000008,   /* QSAFE */
            0x0000000C,   /* RDV */
            0x0000021F,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000003,   /* PDEX2WR */
            0x00000003,   /* PDEX2RD */
            0x00000003,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000B,   /* RW2PDEN */
            0x00000015,   /* TXSR */
            0x00000003,   /* TCKE */
            0x00000008,   /* TFAW */
            0x00000004,   /* TRPAB */
            0x00000008,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x00000270,   /* TREFBW */
            0x00000000,   /* QUSE_EXTRA */
            0x00000001,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xA08604AE,   /* CFG_DIG_DLL */
            0x007DE010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x0000000E,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
		}
	},
	{
		.rate = 300000,   /* SDRAM frequency */
		.regs = {
	    0x00000012,   /* RC */
            0x00000027,   /* RFC */
            0x0000000D,   /* RAS */
            0x00000006,   /* RP */
            0x00000007,   /* R2W */
            0x00000005,   /* W2R */
            0x00000003,   /* R2P */
            0x0000000B,   /* W2P */
            0x00000006,   /* RD_RCD */
            0x00000006,   /* WR_RCD */
            0x00000003,   /* RRD */
            0x00000003,   /* REXT */
            0x00000003,   /* WDV */
            0x00000007,   /* QUSE */
            0x00000004,   /* QRST */
            0x00000009,   /* QSAFE */
            0x0000000D,   /* RDV */
            0x0000045F,   /* REFRESH */
            0x00000000,   /* BURST_REFRESH_NUM */
            0x00000004,   /* PDEX2WR */
            0x00000004,   /* PDEX2RD */
            0x00000006,   /* PCHG2PDEN */
            0x00000008,   /* ACT2PDEN */
            0x00000001,   /* AR2PDEN */
            0x0000000F,   /* RW2PDEN */
            0x0000002A,   /* TXSR */
            0x00000003,   /* TCKE */
            0x0000000F,   /* TFAW */
            0x00000007,   /* TRPAB */
            0x00000007,   /* TCLKSTABLE */
            0x00000002,   /* TCLKSTOP */
            0x000004E0,   /* TREFBW */
            0x00000006,   /* QUSE_EXTRA */
            0x00000002,   /* FBIO_CFG6 */
            0x00000000,   /* ODT_WRITE */
            0x00000000,   /* ODT_READ */
            0x00000282,   /* FBIO_CFG5 */
            0xE05E048B,   /* CFG_DIG_DLL */
            0x007E2010,   /* DLL_XFORM_DQS */
            0x00000000,   /* DLL_XFORM_QUSE */
            0x00000000,   /* ZCAL_REF_CNT */
            0x0000001B,   /* ZCAL_WAIT_CNT */
            0x00000000,   /* AUTO_CAL_INTERVAL */
            0x00000000,   /* CFG_CLKTRIM_0 */
            0x00000000,   /* CFG_CLKTRIM_1 */
            0x00000000,   /* CFG_CLKTRIM_2 */
		}
	}
};

static const struct tegra_emc_chip whistler_emc_chips[] = {                 //WARNING: Please put the low memory size table ahead of the higher one  
	{
		.description = "Samsung 512M 300MHz",
		.mem_manufacturer_id = 0x0101,
		.mem_revision_id1 = 0x0101,
		.mem_revision_id2 = 0x0,
		.mem_pid = 0x1818,
		.table = whistler_emc_tables_samsung_512M_300Mhz,   
		.table_size = ARRAY_SIZE(whistler_emc_tables_samsung_512M_300Mhz)
	},
        {
                .description = "Samsung 1G 300MHz",
                .mem_manufacturer_id = 0x0101,
                .mem_revision_id1 = 0x0101,
                .mem_revision_id2 = 0x0,
                .mem_pid = 0x1818,
                .table = whistler_emc_tables_samsung_1G_300Mhz,
                .table_size = ARRAY_SIZE(whistler_emc_tables_samsung_1G_300Mhz)
        },      

};

#define TEGRA25_SKU 0x17

int __init whistler_emc_init(void)
{
	int sku_id = tegra_sku_id();

//	if (sku_id == TEGRA25_SKU)
//		tegra_init_emc(whistler_ap25_emc_chips,
//				ARRAY_SIZE(whistler_ap25_emc_chips));
//	else
	        tegra_init_emc(whistler_emc_chips,
		                ARRAY_SIZE(whistler_emc_chips));

	return 0;
}
