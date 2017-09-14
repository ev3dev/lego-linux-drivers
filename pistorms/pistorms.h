/*
 * ev3dev driver for mindsensors.com PiStorms
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

#ifndef _PISTORMS_H_
#define _PISTORMS_H_

#include <linux/i2c.h>
#include <lego.h>

#define PISTORMS_NAME_SIZE	30

/**
 * struct pistorms_data - represents a single bank on a PiStorms
 *
 * @name: The base name for ports and leds.
 * @client: The i2c_client that is used to communicate with the bank.
 * @input_data: Pointer to the private data used by the input driver.
 * @leds_data: Pointer to the private data used by the led driver.
 * @in_port_data: Pointer to the private data used by the input port driver.
 * @out_port_data: Pointer to the private data used by the output port driver.
 */
struct pistorms_data {
	char			name[PISTORMS_NAME_SIZE];
	struct i2c_client	*client;
	void			*battery_data;
	void			*input_data;
	void			*leds_data;
	void			*in_port_data;
	void			*out_port_data;
};

int devm_pistorms_register_board(struct device *dev, struct pistorms_data *data);
extern int pistorms_battery_register(struct pistorms_data *data);
extern void pistorms_battery_unregister(struct pistorms_data *data);
extern int pistorms_input_register(struct pistorms_data *data);
extern void pistorms_input_unregister(struct pistorms_data *data);
extern int pistorms_leds_register(struct pistorms_data *data);
extern void pistorms_leds_unregister(struct pistorms_data *data);
extern int pistorms_in_ports_register(struct pistorms_data *data);
extern void pistorms_in_ports_unregister(struct pistorms_data *data);
extern int pistorms_out_ports_register(struct pistorms_data *data);
extern void pistorms_out_ports_unregister(struct pistorms_data *data);

extern const struct device_type pistorms_in_port_type;

#endif /* _PISTORMS_H_ */
