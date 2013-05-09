/*
 * Copyright (c) 2011, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __OV7692_H__
#define __OV7692_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define OV7692_IOCTL_SET_MODE		_IOW('o', 1, struct ov7692_mode)
#define OV7692_IOCTL_GET_STATUS		_IOR('o', 2, struct ov7692_status)
#define OV7692_IOCTL_SET_COLOR_EFFECT   _IOW('o', 3, enum ov7692_coloreffect_mode)
#define OV7692_IOCTL_SET_WHITE_BALANCE  _IOW('o', 4,enum ov7692_balance_mode)
/*ztebsp yuxin add for ov7692 effect,2011.12.14 ++*/
#define OV7692_IOCTL_SET_CONTRAST  _IOW('o', 5,enum ov7692_contrast_mode)
#define OV7692_IOCTL_SET_SATURATION  _IOW('o', 6,enum ov7692_saturation_mode)
#define OV7692_IOCTL_SET_BRIGHTNESS  _IOW('o', 7,enum ov7692_brightness_mode)
/*ztebsp yuxin add for ov7692 effect,2011.12.14 --*/

#define OV7692_IOCTL_SET_EXPOSURE       _IOW('o', 8, enum ov7692_exposure_mode)
struct ov7692_mode {
	int xres;
	int yres;
};

struct ov7692_status {
	int data;
	int status;
};



enum {
        OV7692_ColorEffect = 0,
        OV7692_Whitebalance,
        OV7692_SceneMode,
        OV7692_Exposure,
     /*ztebsp yuxin add for ov7692 effect,2011.12.14 ++*/
     /*ZTEBSP yuxin modify for these settings number the same with menu setting,2012.01.09++*/
        OV7692_Brightness = 7, 
        OV7692_Contrast = 8,   
        OV7692_Saturation = 9,
     /*ZTEBSP yuxin modify for these settings number the same with menu setting,2012.01.09--*/
     /*ztebsp yuxin add for ov7692 effect,2011.12.14 --*/

};
/*ZTEBSP yuxin modify WB issue number the same with menu setting,2012.01.09 ++*/
enum ov7692_balance_mode{
        OV7692_Whitebalance_Invalid = 0,
        OV7692_Whitebalance_Auto = 1,
        OV7692_Whitebalance_Incandescent = 2,
        OV7692_Whitebalance_Fluorescent = 3,
        OV7692_Whitebalance_Daylight = 5,
        OV7692_Whitebalance_Cloudy = 6,
};
/*ZTEBSP yuxin modify WB issue number the same with menu setting,2012.01.09 --*/

enum ov7692_exposure_mode{
        OV7692_Exposure_0,
        OV7692_Exposure_1,
        OV7692_Exposure_2,
        OV7692_Exposure_Negative_1,
        OV7692_Exposure_Negative_2,
 
};
enum ov7692_coloreffect_mode{
        OV7692_ColorEffect_Invalid = 0,
        OV7692_ColorEffect_Aqua,
        OV7692_ColorEffect_Blackboard,
        OV7692_ColorEffect_Mono,
        OV7692_ColorEffect_Negative,
        OV7692_ColorEffect_None,
        OV7692_ColorEffect_Posterize,
        OV7692_ColorEffect_Sepia,
        OV7692_ColorEffect_Solarize,
        OV7692_ColorEffect_Whiteboard
};

/*ztebsp yuxin add for ov7692 effect,2011.12.14 ++*/
enum ov7692_contrast_mode{
        OV7692_Contrast_0=0,
        OV7692_Contrast_1,
        OV7692_Contrast_2,
        OV7692_Contrast_3,
        OV7692_Contrast_4,
};

enum ov7692_saturation_mode{
        OV7692_Saturation_0=0,
        OV7692_Saturation_1,
        OV7692_Saturation_2,
        OV7692_Saturation_3,
        OV7692_Saturation_4,
};

enum ov7692_brightness_mode{
        OV7692_Brightness_0=0,
        OV7692_Brightness_1,
        OV7692_Brightness_2,
        OV7692_Brightness_3,
        OV7692_Brightness_4,
};
/*ztebsp yuxin add for ov7692 effect,2011.12.14 --*/

#ifdef __KERNEL__
struct ov7692_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __OV7692_H__ */

