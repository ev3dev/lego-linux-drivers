/*
 * Input Port driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2013-2014 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_LEGOEV3_EV3_INPUT_PORT_H
#define __LINUX_LEGOEV3_EV3_INPUT_PORT_H

#include <mach/legoev3.h>

#include "legoev3_analog.h"

enum ev3_input_port_gpio_state {
	EV3_INPUT_PORT_GPIO_FLOAT,
	EV3_INPUT_PORT_GPIO_LOW,
	EV3_INPUT_PORT_GPIO_HIGH,
};

/**
 * ev3_host_platform_data - platform data used by analog host drivers
 * @inital_sensor: Name of sensor device to load during driver probe.
 */
struct ev3_analog_host_platform_data {
	const char *inital_sensor;
};

struct legoev3_port;

extern int ev3_input_port_register_i2c(struct legoev3_port *, struct device *);
extern void ev3_input_port_unregister_i2c(struct legoev3_port *);
extern int ev3_input_port_enable_uart(struct legoev3_port *in_port);
extern void ev3_input_port_disable_uart(struct legoev3_port *in_port);

#include "legoev3_ports.h"

#endif /* __LINUX_LEGOEV3_EV3_INPUT_PORT_H */
