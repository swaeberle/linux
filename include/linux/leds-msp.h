/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MSP LED chip driver.
 *
 * Copyright (C) 2015 MSC Technologies, Design Center Aachen
 *
 */

#ifndef __LINUX_MSP_H
#define __LINUX_MSP_H
#include <linux/leds.h>

struct msp_platform_data {
	struct led_platform_data leds;
#ifdef CONFIG_GPIOLIB
	int gpio_base;
	void (*setup)(unsigned int gpio_base, unsigned int ngpio);
#endif
};

#define	MSP_MAKE_GPIO 1
#endif /* __LINUX_MSP_H*/
