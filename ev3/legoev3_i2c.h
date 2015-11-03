/*
 * i2c-legoev3 interface to platform code
 *
 * Copyright (C) 2013-2014 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LINUX_I2C_LEGOEV3_H
#define _LINUX_I2C_LEGOEV3_H

#include <mach/legoev3.h>

/**
 * struct i2c_legoev3_platform_data - Platform-dependent data for i2c-legoev3
 * @sda_pin: GPIO pin ID to use for SDA.
 * @scl_pin: GPIO pin ID to use for SCL.
 * @irq_pin: GPIO pin ID to use for triggering an IRQ from the FIQ context.
 * 	We can use the other pin 5 gpio on the input port since we know
 * 	it is not being used as long as we are using I2C.
 * @port_id: The input port identifier.
 * @in_port: Pointer to the legoev3 port device that represents the input port.
 */
struct i2c_legoev3_platform_data {
	unsigned int sda_pin;
	unsigned int scl_pin;
	enum legoev3_input_port_id port_id;
	struct lego_port_device *in_port;
};

extern const struct i2c_algorithm i2c_legoev3_algo;

#endif /* _LINUX_I2C_LEGOEV3_H */
