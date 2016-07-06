/*
 * User-defined LEDs
 *
 * Copyright (C) 2016 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __USER_LED_H
#define __USER_LED_H

#include <linux/cdev.h>
#include <linux/leds.h>

#define USER_LED_NAME_SIZE 50

struct user_led {
	/* set these before registering */
	char name[USER_LED_NAME_SIZE];
	/* private */
	struct led_classdev led_cdev;
	struct cdev cdev;
	int minor;
};

int user_led_register(struct user_led *led, struct device *parent);
void user_led_unregister(struct user_led *led);

#endif /* __USER_LED_H */
