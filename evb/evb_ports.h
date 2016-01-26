/*
 * Platform data for LEGO MINDSTORMS EV3 drivers
 *
 * Copyright (C) 2014,2016 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _EVB_PORTS_H
#define _EVB_PORTS_H


struct ev3_input_port_platform_data {
	u32 id;
	unsigned i2c_dev_id;
	unsigned i2c_pin_mux;
	unsigned uart_pin_mux;
	const char *uart_tty;
};

struct ev3_output_port_platform_data {
	u32 id;
};

struct evb_ports_platform_data {
	struct ev3_input_port_platform_data *input_port_data;
	struct ev3_output_port_platform_data *output_port_data;
};

struct ev3_motor_platform_data {
	unsigned tacho_int_gpio;
	unsigned tacho_dir_gpio;
};

#endif /* _EVB_PORTS_H */
