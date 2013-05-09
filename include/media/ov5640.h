/*
 * Copyright (C) 2010 Motorola, Inc.
 * Copyright (C) 2011 NVIDIA Corporation.
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

#ifndef __OV5640_H__
#define __OV5640_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define OV5640_IOCTL_SET_MODE		_IOW('o', 1, struct ov5640_mode)

/* ZTE: modify by yaoling for yuv balance , EFFECT,scene  20110812 ++ */
/* #define OV5640_IOCTL_SET_FRAME_LENGTH	_IOW('o', 2, __u32)
#define OV5640_IOCTL_SET_COARSE_TIME	_IOW('o', 3, __u32)
#define OV5640_IOCTL_SET_GAIN		_IOW('o', 4, __u16)
#define OV5640_IOCTL_GET_STATUS		_IOR('o', 5, __u8)
*/
#define OV5640_IOCTL_GET_STATUS		_IOR('o', 2, __u8)
#define OV5640_IOCTL_SET_COLOR_EFFECT   _IOW('o', 3, __u8)
#define OV5640_IOCTL_SET_WHITE_BALANCE  _IOW('o', 4,enum ov5640_balance_mode)
#define OV5640_IOCTL_SET_SCENE_MODE     _IOW('o', 5, enum ov5640_scene_mode)
// zte-modify: ysq 20120228 autofocus support ++++++
#define OV5640_IOCTL_SET_AF_MODE        _IOW('o', 6, __u8)
#define OV5640_IOCTL_GET_AF_STATUS      _IOW('o', 7, __u8)
// zte-modify: ysq 20120228 autofocus support ------
/* ZTE: add  by yaoling for yuv expose 20110812 ++ */
#define OV5640_IOCTL_SET_EXPOSURE       _IOW('o', 8, int)
 /* ZTE: add by yaoling for yuv expose 20110812 -- */
#define OV5640_IOCTL_TEST_PATTERN	    _IOW('o', 9, enum ov5640_test_pattern)
#define OV5640_IOCTL_SET_CAMERA_MODE	_IOW('o', 10, __u32)
/* ZTE: modify by yaoling for yuv balance , EFFECT,scene  20110812 ++ */
#define OV5640_IOCTL_GET_EXP            _IOR('o', 11, __u8)
/* ZTE: modify by yaoling for yuv balance,EFFECT,scene 20110812 -- */

/*[ECID:0000]ZTEBSP,yuxin add for ov5640 ISO effect 2011.12.06 ++*/
#define OV5640_IOCTL_SET_ISO     _IOW('o', 12, enum ov5640_ISO_mode)
#define OV5640_IOCTL_SET_ANTIBANDING   _IOW('o', 13, enum ov5640_Anti_Banding_mode)
#define OV5640_IOCTL_SET_BRIGHTNESS    _IOW('o', 14, enum ov5640_Brightness_mode)
#define OV5640_IOCTL_SET_CONTRAST    _IOW('o', 15, enum ov5640_Contrast_mode)
#define OV5640_IOCTL_SET_SATURATION    _IOW('o', 16, enum ov5640_Saturation_mode)
#define OV5640_IOCTL_SET_SHARPNESS      _IOW('o', 17, enum ov5640_Sharpness_mode)
/*[ECID:0000]ZTEBSP,yuxin add for ov5640 ISO effect 2011.12.06 --*/



enum ov5640_test_pattern {
        OV5640_TEST_PATTERN_NONE,
        OV5640_TEST_PATTERN_COLORBARS,
        OV5640_TEST_PATTERN_CHECKERBOARD

};
/* ZTE: add  by yaoling for yuv balance,expose,scene  20110812 ++ */
enum {
        OV5640_ColorEffect = 0,
        OV5640_Whitebalance,
        OV5640_SceneMode,
        OV5640_Exposure,
        OV5640_Flashmode,//zte ysq add
        OV5640_FlashControlOn,
/*[ECID:0000]ZTEBSP,yuxin add for ov5640 ISO effect 2011.12.06 ++*/
        OV5640_ISO,
        OV5640_AntiBanding,
        OV5640_Brightness,
        OV5640_Contrast,
        OV5640_Saturation,
        OV5640_Sharpness,
/*[ECID:0000]ZTEBSP,yuxin add for ov5640 ISO effect 2011.12.06 --*/

        
};
/*ZTEBSP yuxin modify WB issue number the same with menu setting,2012.01.09 ++*/
enum ov5640_balance_mode{
        OV5640_Whitebalance_Invalid = 0,
        OV5640_Whitebalance_Auto = 1,
        OV5640_Whitebalance_Incandescent = 2,
        OV5640_Whitebalance_Fluorescent = 3,
        OV5640_Whitebalance_WarmFluorescent = 4,
        OV5640_Whitebalance_Daylight = 5,
        OV5640_Whitebalance_Cloudy = 6,  /*zte yuxin add for WB,2011.12.22*/

};
/*ZTEBSP yuxin modify WB issue number the same with menu setting,2012.01.09 --*/

enum ov5640_scene_mode{
        OV5640_SceneMode_Invalid = 0,
        OV5640_SceneMode_Auto,
        OV5640_SceneMode_Action,
        OV5640_SceneMode_Portrait,
        OV5640_SceneMode_Landscape,
        OV5640_SceneMode_Beach,
        OV5640_SceneMode_Candlelight,
        OV5640_SceneMode_Fireworks,
        OV5640_SceneMode_Night,
};
enum ov5640_exposure_mode{
        OV5640_Exposure_0,
        OV5640_Exposure_1,
        OV5640_Exposure_2,
        OV5640_Exposure_Negative_1,
        OV5640_Exposure_Negative_2,
 
};
enum ov5640_ColorEffect_mode{
        OV5640_ColorEffect_Invalid = 0,
        OV5640_ColorEffect_Aqua,
        OV5640_ColorEffect_Blackboard,
        OV5640_ColorEffect_Mono,
        OV5640_ColorEffect_Negative,
        OV5640_ColorEffect_None,
        OV5640_ColorEffect_Posterize,
        OV5640_ColorEffect_Sepia,
        OV5640_ColorEffect_Solarize,
        OV5640_ColorEffect_Whiteboard
};

/* ZTE: add  by yaoling for yuv balance,expose,scene  20110812 -- */

/*[ECID:0000]ZTEBSP,yuxin add for ov5640 ISO effect 2011.12.06 ++*/
enum ov5640_ISO_mode{
        OV5640_ISO_auto=0,
        OV5640_ISO_100,
        OV5640_ISO_200,
        OV5640_ISO_400,
        OV5640_ISO_800,
        OV5640_ISO_1600,	
};

/*ZTEBSP yuxin modify antibanding issue number the same with menu setting,2012.01.09 ++*/
enum ov5640_Anti_Banding_mode{
	OV5640_AntiBanding_invalid = 0,
	OV5640_AntiBanding_50Hz   = 1,
	OV5640_AntiBanding_60Hz   = 2,
	OV5640_AntiBanding_auto    = 3,
	OV5640_AntiBanding_off       = 4,

};
/*ZTEBSP yuxin modify antibanding issue number the same with menu setting,2012.01.09 --*/

enum ov5640_Brightness_mode{
	OV5640_Brightness_0=0,
	OV5640_Brightness_1,
	OV5640_Brightness_2,
	OV5640_Brightness_3,
	OV5640_Brightness_4,
};

enum ov5640_Contrast_mode{
       OV5640_Contrast_0=0,
       OV5640_Contrast_1,
       OV5640_Contrast_2,
       OV5640_Contrast_3,
       OV5640_Contrast_4,

};
enum ov5640_Saturation_mode{
       OV5640_Saturation_0=0,
       OV5640_Saturation_1,
       OV5640_Saturation_2,
       OV5640_Saturation_3,
       OV5640_Saturation_4,
	   	
};



enum ov5640_Sharpness_mode{
       OV5640_Sharpness_0=0,
       OV5640_Sharpness_1,
       OV5640_Sharpness_2,
       OV5640_Sharpness_3,
       OV5640_Sharpness_4,
};
/*[ECID:0000]ZTEBSP,yuxin add for ov5640 ISO effect 2011.12.06 --*/

struct ov5640_mode {
	int xres;
	int yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
};
#ifdef __KERNEL__
struct ov5640_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __OV5640_H__ */

