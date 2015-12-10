/*
 * User-defined LEGO devices - Sensor driver
 *
 * Copyright (C) 2015 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <lego.h>
#include <lego_sensor_class.h>

/* The USER_LEGO_SENSOR_TEXT_VALUE_SIZE can be a maximum of PAGE_SIZE-2
 * to take into account a trailing newline (added later) and NULL. We are
 * limiting it to much less to save memory for each user_lego_sensor 
 * instance.
 */

#define USER_LEGO_SENSOR_TEXT_VALUE_SIZE (512-2)

struct user_lego_sensor_device {
	struct lego_sensor_device sensor;
	struct device dev;
        char text_value[USER_LEGO_SENSOR_TEXT_VALUE_SIZE];
};

extern int user_lego_sensor_register(struct user_lego_sensor_device *sensor,
				     struct device *parent);
extern void user_lego_sensor_unregister(struct user_lego_sensor_device *sensor);
