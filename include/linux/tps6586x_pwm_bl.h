/* TPS6586X PWM backlight data *
 *
 * Copyright (C) 2011 NVIDIA Corporation
 * Author: Renuka Apte <rapte@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef TPS6586X_PWM_BL_H
#define TPS6586X_PWM_BL_H

struct platform_tps6586x_pwm_backlight_data {
	unsigned int max_brightness;
	unsigned int dft_brightness;
        int (*notify)(struct device *unused, int brightness);
};

#endif /* TPS6586X_PWM_BL_H */
