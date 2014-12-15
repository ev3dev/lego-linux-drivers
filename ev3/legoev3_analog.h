/*
 * Analog framework for LEGO Mindstorms EV3
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

#ifndef __LINUX_LEGOEV3_ANALOG_H
#define __LINUX_LEGOEV3_ANALOG_H

#include <linux/spi/spi.h>

#include <mach/legoev3.h>

#define to_legoev3_analog_device(x) container_of((x), struct legoev3_analog_device, dev)

typedef void (*legoev3_analog_cb_func_t)(void *context);

struct legoev3_analog_device;

extern struct legoev3_analog_device *get_legoev3_analog(void);
extern void put_legoev3_analog(struct legoev3_analog_device *);
extern u16 legoev3_analog_in_pin1_value(struct legoev3_analog_device *,
					enum legoev3_input_port_id);
extern u16 legoev3_analog_in_pin6_value(struct legoev3_analog_device *,
					enum legoev3_input_port_id);
extern u16 legoev3_analog_out_pin5_value(struct legoev3_analog_device *,
					 enum legoev3_output_port_id);
extern u16 legoev3_analog_batt_volt_value(struct legoev3_analog_device *);
extern u16 legoev3_analog_batt_curr_value(struct legoev3_analog_device *);
extern void legoev3_analog_register_in_cb(struct legoev3_analog_device *,
					  enum legoev3_input_port_id,
					  legoev3_analog_cb_func_t, void *);

extern struct spi_driver legoev3_analog_driver;

#endif /* __LINUX_LEGOEV3_ANALOG_H */
