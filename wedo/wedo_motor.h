/*
 * Motor Definitions for LEGO WeDo
 *
 * Copyright (C) 2014 Ralph Hempel <rhemple@hempeldesigngroup.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/legoev3/dc_motor_class.h>

#include "wedo_port.h"

extern const struct dc_motor_ops wedo_motor_ops;

struct wedo_motor_data {
	struct wedo_port_device *wpd;
	struct dc_motor_device md;

	enum dc_motor_command command;
	enum dc_motor_direction direction;
};
