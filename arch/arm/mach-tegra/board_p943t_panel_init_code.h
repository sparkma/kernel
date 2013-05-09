/*============================================================================

                 LCD Panel Init Code for P943T

   DESCRIPTION
     Different panel init code supplied by various vendors.

   Copyright (c) 2012 by ZTE, Incorporated.  All Rights Reserved.
============================================================================*/

/*============================================================================

                      EDIT HISTORY FOR FILE

 This section contains comments describing changes made to this file.
 Notice that changes are listed in reverse chronological order.


 when      who    what, where, why
 --------  -----  ----------------------------------------------------------
 12/05/04  Ninja     Initial release 
 
============================================================================*/
#ifndef INCLUDED_PANEL_943T_PANEL_INIT_CODE_H
#define INCLUDED_PANEL_943T_PANEL_INIT_CODE_H

#include <mach/dc.h>

//--------------------------------------------------------------//
//---------------------LEAD  LCD -------------------------------//
//--------------------------------------------------------------//
static u8 nt315516_lead_0xff[5]={0xFF,0xAA,0x55,0x25,0x01}; 
static u8 nt315516_lead_0xf3[5] = {0xF3, 0x02, 0x03, 0x07, 0x15};
static u8 nt315516_lead_0xf0[6]={0xF0,0x55,0xAA,0x52,0x08,0x00};  
static u8 nt315516_lead_0xb8[5]={0xB8,0x01,0x02,0x02,0x02};  
static u8 nt315516_lead_0xbc[4]={0xBC,0x05,0x05,0x05}; 
static u8 nt315516_lead_0xf0_1[6]={0xF0,0x55,0xAA,0x52,0x08,0x01};
static u8 nt315516_lead_0xb0[4]={0xB0,0x05,0x05,0x05}; 
static u8 nt315516_lead_0xb1[4]={0xB1,0x05,0x05,0x05}; 
static u8 nt315516_lead_0xb6[4]={0xB6,0x44,0x44,0x44};
static u8 nt315516_lead_0xb7[4]={0xB7,0x34,0x34,0x34}; 
static u8 nt315516_lead_0xba[4]={0xBA,0x24,0x24,0x24};   
static u8 nt315516_lead_0xbc_1[4]={0xBC,0x00,0x88,0x00};
static u8 nt315516_lead_0xbd[4]={0xBD,0x00,0x88,0x00};
static u8 nt315516_lead_0xbe[2]={0xBE,0x4F};

// Positive Red Gamma
static u8 nt315516_lead_0xd1[17]={0xD1,0x00,0x29,0x00,0x32,0x00,0x46,0x00,0x59,0x00,0x69,0x00,0x86,0x00,0x9C,0x00,0xC3};
static u8 nt315516_lead_0xd2[17]={0xD2,0x00,0xE6,0x01,0x1B,0x01,0x43,0x01,0x83,0x01,0xB2,0x01,0xB3,0x01,0xDE,0x02,0x07};
static u8 nt315516_lead_0xd3[17]={0xD3,0x02,0x20,0x02,0x3F,0x02,0x53,0x02,0x78,0x02,0x95,0x02,0xC6,0x02,0xEE,0x03,0x30};
static u8 nt315516_lead_0xd4[5]  ={0xD4,0x03,0xF0,0x03,0xF4};
// Positive Green Gamma
static u8 nt315516_lead_0xd5[17]={0xD5,0x00,0x29,0x00,0x32,0x00,0x46,0x00,0x59,0x00,0x69,0x00,0x86,0x00,0x9C,0x00,0xC3};
static u8 nt315516_lead_0xd6[17]={0xD6,0x00,0xE6,0x01,0x1B,0x01,0x43,0x01,0x83,0x01,0xB2,0x01,0xB3,0x01,0xDE,0x02,0x07};
static u8 nt315516_lead_0xd7[17]={0xD7,0x02,0x20,0x02,0x3F,0x02,0x53,0x02,0x78,0x02,0x95,0x02,0xC6,0x02,0xEE,0x03,0x30};
static u8 nt315516_lead_0xd8[5]  ={0xD8,0x03,0xF0,0x03,0xF4};
                 
// Positive Blue Gamma
static u8 nt315516_lead_0xd9[17]={0xD9,0x00,0x29,0x00,0x32,0x00,0x46,0x00,0x59,0x00,0x69,0x00,0x86,0x00,0x9C,0x00,0xC3};
static u8 nt315516_lead_0xdd[17]={0xDD,0x00,0xE6,0x01,0x1B,0x01,0x43,0x01,0x83,0x01,0xB2,0x01,0xB3,0x01,0xDE,0x02,0x07};
static u8 nt315516_lead_0xde[17]={0xDE,0x02,0x20,0x02,0x3F,0x02,0x53,0x02,0x78,0x02,0x95,0x02,0xC6,0x02,0xEE,0x03,0x30};
static u8 nt315516_lead_0xdf[5]  ={0xDF,0x03,0xF0,0x03,0xF4};      
                 
// Negative Red Gamma
static u8 nt315516_lead_0xe0[17]={0xE0,0x00,0x29,0x00,0x32,0x00,0x46,0x00,0x59,0x00,0x69,0x00,0x86,0x00,0x9C,0x00,0xC3};
static u8 nt315516_lead_0xe1[17]={0xE1,0x00,0xE6,0x01,0x1B,0x01,0x43,0x01,0x83,0x01,0xB2,0x01,0xB3,0x01,0xDE,0x02,0x07};
static u8 nt315516_lead_0xe2[17]={0xE2,0x02,0x20,0x02,0x3F,0x02,0x53,0x02,0x78,0x02,0x95,0x02,0xC6,0x02,0xEE,0x03,0x30};
static u8 nt315516_lead_0xe3[5]  ={0xE3,0x03,0xF0,0x03,0xF4};           
                 
// Negative Green Gamma
static u8 nt315516_lead_0xe4[17]={0xE4,0x00,0x29,0x00,0x32,0x00,0x46,0x00,0x59,0x00,0x69,0x00,0x86,0x00,0x9C,0x00,0xC3};
static u8 nt315516_lead_0xe5[17]={0xE5,0x00,0xE6,0x01,0x1B,0x01,0x43,0x01,0x83,0x01,0xB2,0x01,0xB3,0x01,0xDE,0x02,0x07};
static u8 nt315516_lead_0xe6[17]={0xE6,0x02,0x20,0x02,0x3F,0x02,0x53,0x02,0x78,0x02,0x95,0x02,0xC6,0x02,0xEE,0x03,0x30};
static u8 nt315516_lead_0xe7[5]  ={0xE7,0x03,0xF0,0x03,0xF4};    
                 
// Negative Blue Gamma
static u8 nt315516_lead_0xe8[17]={0xE8,0x00,0x29,0x00,0x32,0x00,0x46,0x00,0x59,0x00,0x69,0x00,0x86,0x00,0x9C,0x00,0xC3};
static u8 nt315516_lead_0xe9[17]={0xE9,0x00,0xE6,0x01,0x1B,0x01,0x43,0x01,0x83,0x01,0xB2,0x01,0xB3,0x01,0xDE,0x02,0x07};
static u8 nt315516_lead_0xea[17]={0xEA,0x02,0x20,0x02,0x3F,0x02,0x53,0x02,0x78,0x02,0x95,0x02,0xC6,0x02,0xEE,0x03,0x30};
static u8 nt315516_lead_0xeb[5]={0xEB,0x03,0xF0,0x03,0xF4};         

//===============LEAD dsi Init Cmd===================
static struct tegra_dsi_cmd dsi_lead_init_cmd[]= {
	DSI_CMD_LONG(0x39, nt315516_lead_0xff),
	DSI_CMD_LONG(0x39, nt315516_lead_0xf3),
	DSI_CMD_LONG(0x39, nt315516_lead_0xf0),
	DSI_CMD_LONG(0x39, nt315516_lead_0xb8),
	DSI_CMD_LONG(0x39, nt315516_lead_0xbc),
	DSI_CMD_LONG(0x39, nt315516_lead_0xf0_1),
	DSI_CMD_LONG(0x39, nt315516_lead_0xb0),
	DSI_CMD_LONG(0x39, nt315516_lead_0xb1),
	DSI_CMD_LONG(0x39, nt315516_lead_0xb6),
	DSI_CMD_LONG(0x39, nt315516_lead_0xb7),
	DSI_CMD_LONG(0x39, nt315516_lead_0xba),
	DSI_CMD_LONG(0x39, nt315516_lead_0xbc_1),
	DSI_CMD_LONG(0x39, nt315516_lead_0xbd),
	DSI_CMD_SHORT(0x15, 0xBE,0x4F),

	DSI_CMD_LONG(0x39,nt315516_lead_0xd1),
	DSI_CMD_LONG(0x39,nt315516_lead_0xd2),
	DSI_CMD_LONG(0x39,nt315516_lead_0xd3),
	DSI_CMD_LONG(0x39,nt315516_lead_0xd4),
	
	DSI_CMD_LONG(0x39,nt315516_lead_0xd5),
	DSI_CMD_LONG(0x39,nt315516_lead_0xd6),
	DSI_CMD_LONG(0x39,nt315516_lead_0xd7),
	DSI_CMD_LONG(0x39,nt315516_lead_0xd8), 
	
	DSI_CMD_LONG(0x39,nt315516_lead_0xd9),
	DSI_CMD_LONG(0x39,nt315516_lead_0xdd),
	DSI_CMD_LONG(0x39,nt315516_lead_0xde),
	DSI_CMD_LONG(0x39,nt315516_lead_0xdf),
	
	DSI_CMD_LONG(0x39,nt315516_lead_0xe0),
	DSI_CMD_LONG(0x39,nt315516_lead_0xe1),
	DSI_CMD_LONG(0x39,nt315516_lead_0xe2),
	DSI_CMD_LONG(0x39,nt315516_lead_0xe3), 
	
	DSI_CMD_LONG(0x39,nt315516_lead_0xe4),
	DSI_CMD_LONG(0x39,nt315516_lead_0xe5),
	DSI_CMD_LONG(0x39,nt315516_lead_0xe6),
	DSI_CMD_LONG(0x39,nt315516_lead_0xe7),
	
	DSI_CMD_LONG(0x39,nt315516_lead_0xe8),
	DSI_CMD_LONG(0x39,nt315516_lead_0xe9),
	DSI_CMD_LONG(0x39,nt315516_lead_0xea),
	DSI_CMD_LONG(0x39,nt315516_lead_0xeb),

	DSI_CMD_SHORT(0x05, 0x11, 0x00),
	DSI_DLY_MS(120), 
#if(DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE)
	DSI_CMD_SHORT(0x15, 0x35, 0x00),
#endif
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(20),
};


//--------------------------------------------------------------//
//--------------------- SUCCESS  LCD ---------------------------//
//--------------------------------------------------------------//
static u8 nt315516_ss_0xff[6]={0xFF,0xAA,0x55,0x25,0x01,0x01};
static u8 nt315516_ss_0xf2[36]={0xF2,0x00,0x00,0x4A,0x0A,0xA8,0x00,0x00,0x00,
	   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0B,0x00,0x00,0x00,0x00,
	   0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x01,0x51,0x00,0x01,0x00,0x01};
static u8 nt315516_ss_0xf3[8] = {0xF3, 0x02,0x03,0x07,0x45,0x88,0xd1,0x0d};
static u8 nt315516_ss_0xf0[6]= {0xF0,0x55,0xAA,0x52,0x08,0x00};
static u8 nt315516_ss_0xb1[4]={0xB1,0xcc,0x00,0x00};
static u8 nt315516_ss_0xb8[5]={0xB8,0x01,0x02,0x02,0x02};  
static u8 nt315516_ss_0xc9[7]={0xC9,0x63,0x06,0x0d,0x1a,0x17,0x00};
static u8 nt315516_ss_0xf0_1[6]={0xF0,0x55,0xAA,0x52,0x08,0x01};
static u8 nt315516_ss_0xb0[4]={0xB0,0x05,0x05,0x05}; 
static u8 nt315516_ss_0xb1_1[4]={0xB1,0x05,0x05,0x05};
static u8 nt315516_ss_0xb2[4]={0xB2,0x01,0x01,0x01};
static u8 nt315516_ss_0xb3[4]={0xB3,0x0e,0x0e,0x0e};
static u8 nt315516_ss_0xb4[4]={0xB4,0x08,0x08,0x08};

static u8 nt315516_ss_0xb6[4]={0xB6,0x44,0x44,0x44};
static u8 nt315516_ss_0xb7[4]={0xB7,0x34,0x34,0x34};
static u8 nt315516_ss_0xb8_1[4]={0xB8,0x10,0x10,0x10};
static u8 nt315516_ss_0xb9[4]={0xB9,0x26,0x26,0x26};
static u8 nt315516_ss_0xba[4]={0xBA,0x34,0x34,0x34};   
static u8 nt315516_ss_0xbc[4]={0xBC,0x00,0xc8,0x00};
static u8 nt315516_ss_0xbd[4]={0xBD,0x00,0xc8,0x00};
static u8 nt315516_ss_0xbe[2]={0xBE,0x75};    //ZTEBSP DangXiao 20120604 , for Success LCD filcker
static u8 nt315516_ss_0xc0[3]={0xC0,0x04,0x00};
static u8 nt315516_ss_0xca[2]={0xCA,0x00};
static u8 nt315516_ss_0xd0[5]={0xD0,0x0a,0x10,0x0d,0x0f};

// Positive Red Gamma
static u8 nt315516_ss_0xd1[17]={0xD1,0x00,0x70,0x00,0xCE,0x00,0xF7,0x01,0x10,0x01,0x21,0x01,0x44,0x01,0x62,0x01,0x8D};
static u8 nt315516_ss_0xd2[17]={0xD2,0x01,0xAF,0x01,0xE4,0x02,0x0C,0x02,0x4D,0x02,0x82,0x02,0x84,0x02,0xB8,0x02,0xF0};
static u8 nt315516_ss_0xd3[17]={0xD3,0x03,0x14,0x03,0x42,0x03,0x5E,0x03,0x80,0x03,0x97,0x03,0xB0,0x03,0xC0,0x03,0xDF};
static u8 nt315516_ss_0xd4[5]  ={0xD4,0x03,0xFD,0x03,0xFF};
// Positive Green Gamma
static u8 nt315516_ss_0xd5[17]={0xD5,0x00,0x70,0x00,0xCE,0x00,0xf7,0x01,0x10,0x01,0x21,0x01,0x44,0x01,0x62,0x01,0x8D};
static u8 nt315516_ss_0xd6[17]={0xD6,0x01,0xAF,0x01,0xE4,0x02,0x0c,0x02,0x4d,0x02,0x82,0x02,0x84,0x02,0xB8,0x02,0xF0};
static u8 nt315516_ss_0xd7[17]={0xD7,0x03,0x14,0x03,0x42,0x03,0x5e,0x03,0x80,0x03,0x97,0x03,0xB0,0x03,0xC0,0x03,0xDF};
static u8 nt315516_ss_0xd8[5]  ={0xD8,0x03,0xFD,0x03,0xFF};
                 
// Positive Blue Gamma
static u8 nt315516_ss_0xd9[17]={0xD9,0x00,0x70,0x00,0xCE,0x00,0xF7,0x01,0x10,0x01,0x21,0x01,0x44,0x01,0x62,0x01,0x8D};
static u8 nt315516_ss_0xdd[17]={0xDD,0x01,0xAF,0x01,0xE4,0x02,0x0C,0x02,0x4D,0x02,0x82,0x02,0x84,0x02,0xB8,0x02,0xF0};
static u8 nt315516_ss_0xde[17]={0xDE,0x03,0x14,0x03,0x42,0x03,0x5E,0x03,0x80,0x03,0x97,0x03,0xB0,0x03,0xC0,0x03,0xDF};
static u8 nt315516_ss_0xdf[5]  ={0xDF,0x03,0xFD,0x03,0xFF};      
                 
// Negative Red Gamma
static u8 nt315516_ss_0xe0[17]={0xE0,0x00,0x70,0x00,0xCE,0x00,0xF7,0x01,0x10,0x01,0x21,0x01,0x44,0x01,0x62,0x01,0x8D};
static u8 nt315516_ss_0xe1[17]={0xE1,0x01,0xAF,0x01,0xE4,0x02,0x0C,0x02,0x4D,0x02,0x82,0x02,0x84,0x02,0xB8,0x02,0xF0};
static u8 nt315516_ss_0xe2[17]={0xE2,0x03,0x14,0x03,0x42,0x03,0x5E,0x03,0x80,0x03,0x97,0x03,0xB0,0x03,0xC0,0x03,0xDF};
static u8 nt315516_ss_0xe3[5]  ={0xE3,0x03,0xFD,0x03,0xFF};           
                 
// Negative Green Gamma
static u8 nt315516_ss_0xe4[17]={0xE4,0x00,0x70,0x00,0xCE,0x00,0xF7,0x01,0x10,0x01,0x21,0x01,0x44,0x01,0x62,0x01,0x8D};
static u8 nt315516_ss_0xe5[17]={0xE5,0x01,0xAF,0x01,0xE4,0x02,0x0C,0x02,0x4D,0x02,0x82,0x02,0x84,0x02,0xB8,0x02,0xF0};
static u8 nt315516_ss_0xe6[17]={0xE6,0x03,0x14,0x03,0x42,0x03,0x5E,0x03,0x80,0x03,0x97,0x03,0xB0,0x03,0xC0,0x03,0xDF};
static u8 nt315516_ss_0xe7[5]  ={0xE7,0x03,0xFD,0x03,0xFF};
                 
// Negative Blue Gamma
static u8 nt315516_ss_0xe8[17]={0xE8,0x00,0x70,0x00,0xCE,0x00,0xF7,0x01,0x10,0x01,0x21,0x01,0x44,0x01,0x62,0x01,0x8D};
static u8 nt315516_ss_0xe9[17]={0xE9,0x01,0xAF,0x01,0xE4,0x02,0x0C,0x02,0x4D,0x02,0x82,0x02,0x84,0x02,0xB8,0x02,0xF0};
static u8 nt315516_ss_0xea[17]={0xEA,0x03,0x14,0x03,0x42,0x03,0x5E,0x03,0x80,0x03,0x97,0x03,0xB0,0x03,0xC0,0x03,0xDF};
static u8 nt315516_ss_0xeb[5]  ={0xEB,0x03,0xFD,0x03,0xFF};         


//===============SUCCESS dsi Init Cmd===================
static struct tegra_dsi_cmd dsi_success_init_cmd[]= {
	DSI_CMD_LONG(0x39, nt315516_ss_0xff),
	DSI_CMD_LONG(0x39, nt315516_ss_0xf2),
	DSI_CMD_LONG(0x39, nt315516_ss_0xf3),
	DSI_CMD_LONG(0x39, nt315516_ss_0xf0),
	DSI_CMD_LONG(0x39, nt315516_ss_0xb1),
	DSI_CMD_LONG(0x39, nt315516_ss_0xb8),
	DSI_CMD_LONG(0x39, nt315516_ss_0xc9),
	DSI_CMD_LONG(0x39, nt315516_ss_0xf0_1),
	DSI_CMD_LONG(0x39, nt315516_ss_0xb0),
	DSI_CMD_LONG(0x39, nt315516_ss_0xb1_1),
	DSI_CMD_LONG(0x39, nt315516_ss_0xb2),
	DSI_CMD_LONG(0x39, nt315516_ss_0xb3),
	DSI_CMD_LONG(0x39, nt315516_ss_0xb4),
	DSI_CMD_LONG(0x39, nt315516_ss_0xb6),
	DSI_CMD_LONG(0x39, nt315516_ss_0xb7),
	DSI_CMD_LONG(0x39, nt315516_ss_0xb8_1),
	DSI_CMD_LONG(0x39, nt315516_ss_0xb9),
	DSI_CMD_LONG(0x39, nt315516_ss_0xba),
	DSI_CMD_LONG(0x39, nt315516_ss_0xbc),
	DSI_CMD_LONG(0x39, nt315516_ss_0xbd),
	
	DSI_CMD_SHORT(0x15, 0xBE,0x92),
	DSI_CMD_LONG(0x39, nt315516_ss_0xc0),
	DSI_CMD_SHORT(0x15, 0xCA,0x00),
	DSI_CMD_LONG(0x39, nt315516_ss_0xd0),
	
	DSI_CMD_LONG(0x39, nt315516_ss_0xd1),
	DSI_CMD_LONG(0x39, nt315516_ss_0xd2),
	DSI_CMD_LONG(0x39, nt315516_ss_0xd3),
	DSI_CMD_LONG(0x39, nt315516_ss_0xd4),

	DSI_CMD_LONG(0x39, nt315516_ss_0xd5),
	DSI_CMD_LONG(0x39, nt315516_ss_0xd6),
	DSI_CMD_LONG(0x39, nt315516_ss_0xd7),
	DSI_CMD_LONG(0x39, nt315516_ss_0xd8),

	DSI_CMD_LONG(0x39, nt315516_ss_0xd9),
	DSI_CMD_LONG(0x39, nt315516_ss_0xdd),
	DSI_CMD_LONG(0x39, nt315516_ss_0xde),
	DSI_CMD_LONG(0x39, nt315516_ss_0xdf),

	DSI_CMD_LONG(0x39, nt315516_ss_0xe0),
	DSI_CMD_LONG(0x39, nt315516_ss_0xe1),
	DSI_CMD_LONG(0x39, nt315516_ss_0xe2),
	DSI_CMD_LONG(0x39, nt315516_ss_0xe3),

	DSI_CMD_LONG(0x39, nt315516_ss_0xe4),
	DSI_CMD_LONG(0x39, nt315516_ss_0xe5),
	DSI_CMD_LONG(0x39, nt315516_ss_0xe6),
	DSI_CMD_LONG(0x39, nt315516_ss_0xe7),

	DSI_CMD_LONG(0x39, nt315516_ss_0xe8),
	DSI_CMD_LONG(0x39, nt315516_ss_0xe9),
	DSI_CMD_LONG(0x39, nt315516_ss_0xea),
	DSI_CMD_LONG(0x39, nt315516_ss_0xeb),

	
	DSI_CMD_SHORT(0x05, 0x11, 0x00),
	DSI_DLY_MS(120), 
	DSI_CMD_SHORT(0x05, 0x2C, 0x00),
	DSI_CMD_SHORT(0x05, 0x13, 0x00),
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(20),
};

//--------------------------------------------------------------//
//--------------------- BOE OTM9608A  LCD -------------- --------//
//--------------------------------------------------------------//
static u8 otm9608a_boe_0xFF01[4] = {0xFF,0x96,0x08,0x01};
static u8 otm9608a_boe_0xFF80[3] = {0xFF,0x96,0x08};
static u8 otm9608a_boe_0xB380[6] = {0xB3,0x00,0x00,0x00,0x21,0x00};
static u8 otm9608a_boe_0xC080[10] = {0xC0,0x00,0x48,0x00,0x10,0x10,0x00,0x47,0x1F,0x1F};
static u8 otm9608a_boe_0xC092[5] = {0xC0,0x00,0x0E,0x00,0x11};
static u8 otm9608a_boe_0xC0A2[4] = {0xC0,0x01,0x10,0x00};
static u8 otm9608a_boe_0xC0B3[3] = {0xC0,0x00,0x50};
static u8 otm9608a_boe_0xC480[4] = {0xC4,0x00,0x84,0xFA};
static u8 otm9608a_boe_0xC4A0[9] = {0xC4,0x33,0x09,0x90,0x2B,0x33,0x09,0x90,0x54};
static u8 otm9608a_boe_0xC580[5] = {0xC5,0x08,0x00,0x90,0x11};
static u8 otm9608a_boe_0xC590[8] = {0xC5,0x84,0x72,0x00,0x76,0x33,0x33,0x34};
static u8 otm9608a_boe_0xC5A0[8] = {0xC5,0x96,0x76,0x06,0x76,0x33,0x33,0x34};
static u8 otm9608a_boe_0xC5B0[3] = {0xC5,0x04,0xF9};
static u8 otm9608a_boe_0xC6B0[6] = {0xC6,0x03,0x10,0x00,0x1F,0x12};
static u8 otm9608a_boe_0xD100[3] = {0xD1,0x01,0x01};
static u8 otm9608a_boe_0xB0B1[3] = {0xB0,0x03,0x06};
static u8 otm9608a_boe_0xCB80[11] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCB90[16] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCBA0[16] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCBB0[11] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCBC0[16] = {0xCB,0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCBD0[16] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCBE0[11] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCBF0[11] = {0xCB,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static u8 otm9608a_boe_0xCC80[11] = {0xCC,0x00,0x00,0x0B,0x09,0x01,0x25,0x26,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCC90[16] = {0xCC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x0A,0x02};
static u8 otm9608a_boe_0xCCA0[16] = {0xCC,0x25,0x26,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCCB0[11] = {0xCC,0x00,0x00,0x0A,0x0C,0x02,0x26,0x25,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCCC0[16] = {0xCC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x09,0x0B,0x01};
static u8 otm9608a_boe_0xCCD0[16] = {0xCC,0x26,0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCE80[13] = {0xCE,0x86,0x01,0x06,0x85,0x01,0x06,0x0F,0x00,0x00,0x0F,0x00,0x00};
static u8 otm9608a_boe_0xCE90[15] = {0xCE,0xF0,0x00,0x00,0xF0,0x00,0x00,0xF0,0x00,0x00,0xF0,0x00,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCEA0[15] = {0xCE,0x18,0x02,0x03,0xC1,0x00,0x26,0x00,0x18,0x01,0x03,0xC2,0x00,0x26,0x00};
static u8 otm9608a_boe_0xCEB0[15] = {0xCE,0x18,0x04,0x03,0xC3,0x00,0x26,0x00,0x18,0x03,0x03,0xC4,0x00,0x26,0x00};
static u8 otm9608a_boe_0xCEC0[15] = {0xCE,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCED0[15] = {0xCE,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCF80[15] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCF90[15] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCFA0[15] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCFB0[15] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};
static u8 otm9608a_boe_0xCFC0[11] = {0xCF,0x02,0x02,0x20,0x20,0x00,0x00,0x01,0x04,0x00,0x00};
static u8 otm9608a_boe_0xD800[3] = {0xD8,0xE0,0xE0};
static u8 otm9608a_boe_0xE100[17] = {0xE1,0x02, 0x06, 0x0A, 0x0C, 0x04, 0x09, 0x0A, 0x09, 0x04, 0x07 ,0x14 ,0x10 ,0x00 ,0x01 ,0x01 ,0x05};
static u8 otm9608a_boe_0xE200[17] = {0xE2,0x02, 0x06, 0x0A, 0x0C, 0x04, 0x09, 0x0A, 0x09, 0x04, 0x07 ,0x14 ,0x10 ,0x00 ,0x01 ,0x01 ,0x05};
static u8 otm9608a_boe_0xFF00[4] = {0xFF,0xFF,0xFF,0xFF};


#define DT_GEN_LONG 0x39
#define DT_DCS_LONG 0x39
#define DT_GEN_SHORT_1P 0x13
#define DT_GEN_SHORT_2P 0x23
#define W_REG_OFFSET(offset)  DSI_CMD_SHORT(0x15, 0x00, offset)
#define W_GEN_L_1A1P(reg,data)  DSI_CMD_SHORT(0x15, reg, data)

static struct tegra_dsi_cmd dsi_otm9608a_boe_init_cmd[]= {
	
	 W_REG_OFFSET(0x00),
	 DSI_CMD_LONG( DT_GEN_LONG,otm9608a_boe_0xFF01),
	
	 W_REG_OFFSET(0x80),
	 DSI_CMD_LONG( DT_GEN_LONG,otm9608a_boe_0xFF80),
	
	 W_REG_OFFSET(0x00),
	 W_GEN_L_1A1P(0xA0,0x00),
	
	 W_REG_OFFSET(0x80),
	 DSI_CMD_LONG( DT_GEN_LONG,otm9608a_boe_0xB380),
	
	 W_REG_OFFSET(0x92),
	 W_GEN_L_1A1P(0xB3,0x01),
	
	 W_REG_OFFSET(0xC0),
	 W_GEN_L_1A1P(0xB3,0x19),
	
	 W_REG_OFFSET(0x80),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xC080),
	
	 W_REG_OFFSET(0x92),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xC092),
	
	 W_REG_OFFSET(0xA2),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xC0A2),
	
	 W_REG_OFFSET(0xB3),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xC0B3),
	
	 W_REG_OFFSET(0x81),
	 W_GEN_L_1A1P(0xC1,0x55),
	
	 W_REG_OFFSET(0x80),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xC480),
	
	 W_REG_OFFSET(0xA0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xC4A0),
	
	 W_REG_OFFSET(0x80),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xC580),
	
	 W_REG_OFFSET(0x90),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xC590),
	
	 W_REG_OFFSET(0xA0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xC5A0),
	
	 W_REG_OFFSET(0xB0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xC5B0),
	
	 W_REG_OFFSET(0x80),
	 W_GEN_L_1A1P(0xC6,0x64),
	
	 W_REG_OFFSET(0xB0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xC6B0),
	
	 W_REG_OFFSET(0xE1),
	 W_GEN_L_1A1P(0xC0,0x9F),
	
	 W_REG_OFFSET(0x00),
	 W_GEN_L_1A1P(0xD0,0x01),
	
	 W_REG_OFFSET(0x00),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xD100),
	
	 W_REG_OFFSET(0xB7),
	 W_GEN_L_1A1P(0xB0,0x10),
	
	 W_REG_OFFSET(0xC0),
	 W_GEN_L_1A1P(0xB0,0x55),
	
	 W_REG_OFFSET(0xB1),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xB0B1),
	
	 W_REG_OFFSET(0x80),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCB80),
	
	 W_REG_OFFSET(0x90),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCB90),
	
	 W_REG_OFFSET(0xA0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCBA0),
	
	 W_REG_OFFSET(0xB0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCBB0),
	
	 W_REG_OFFSET(0xC0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCBC0),
	
	 W_REG_OFFSET(0xD0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCBD0),
	
	 W_REG_OFFSET(0xE0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCBE0),
	
	 W_REG_OFFSET(0xF0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCBF0),
	
	 W_REG_OFFSET(0x80),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCC80),
	
	 W_REG_OFFSET(0x90),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCC90),
	
	 W_REG_OFFSET(0xA0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCCA0),
	
	 W_REG_OFFSET(0xB0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCCB0),
	
	 W_REG_OFFSET(0xC0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCCC0),
	
	 W_REG_OFFSET(0xD0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCCD0),
	
	 W_REG_OFFSET(0x80),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCE80),
	
	 W_REG_OFFSET(0x90),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCE90),
	
	 W_REG_OFFSET(0xA0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCEA0),
	
	 W_REG_OFFSET(0xB0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCEB0),
	
	 W_REG_OFFSET(0xC0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCEC0),
	
	 W_REG_OFFSET(0xD0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCED0),
	
	 W_REG_OFFSET(0x80),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCF80),
	
	 W_REG_OFFSET(0x90),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCF90),
	
	 W_REG_OFFSET(0xA0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCFA0),
	
	 W_REG_OFFSET(0xB0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCFB0),
	
	 W_REG_OFFSET(0xC0),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xCFC0),
	
	 W_REG_OFFSET(0x80),
	 W_GEN_L_1A1P(0xD6,0x08),
	
	 W_REG_OFFSET(0x00),
	 W_GEN_L_1A1P(0xD7,0x00),
	
	 W_REG_OFFSET(0x00),
	 DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xD800),
	
	 W_REG_OFFSET(0x00),
	 W_GEN_L_1A1P(0xD9,0x45),
	
	 W_REG_OFFSET(0x00),
	 DSI_CMD_LONG(DT_GEN_LONG, otm9608a_boe_0xE100),
	
	 W_REG_OFFSET(0x00),
	 DSI_CMD_LONG(DT_GEN_LONG, otm9608a_boe_0xE200),
	
	W_REG_OFFSET(0x00),
	DSI_CMD_LONG( DT_GEN_LONG, otm9608a_boe_0xFF00),

	//common
	DSI_CMD_SHORT(0x05, 0x11, 0x00),
	DSI_DLY_MS(120), 
#if(DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE)
	DSI_CMD_SHORT(0x15, 0x35, 0x00),
#endif
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(20),
};

#endif
