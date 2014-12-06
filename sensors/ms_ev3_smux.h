/*
 * mindsensors.com EV3 Sensor Multiplexer device driver
 *
 * Copyright (C) 2014 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef MS_EV3_SMUX_H_
#define MS_EV3_SMUX_H_

#include <lego_port_class.h>

#include "nxt_i2c_sensor.h"

#define MS_EV3_SMUX_MODE_REG		0x52
#define MS_EV3_SMUX_DATA_REG		0x54
#define MS_EV3_SMUX_ANALOG_MODE_DATA	0x0F
#define MS_EV3_SMUX_UPDATE_REG		0x41
#define MS_EV3_SMUX_UPDATE_DATA		0xCC
#define MS_EV3_SMUX_MODE_NAME_REG	0x42
#define MS_EV3_SMUX_MODE_NAME_SIZE	11
#define MS_EV3_SMUX_RAW_DATA_SIZE	4

extern void ms_ev3_smux_poll_cb(struct nxt_i2c_sensor_data *data);
extern int ms_ev3_smux_probe_cb(struct nxt_i2c_sensor_data *data);
extern void ms_ev3_smux_remove_cb(struct nxt_i2c_sensor_data *data);

struct ms_ev3_smux_port_type_ops {
	int (*set_uart_sensor_mode)(struct lego_port_device *port, u8 mode);
	void (*set_raw_data_ptr)(struct lego_port_device *port, u8 *raw_data,
				 uint size);
};

extern const struct lego_port_type ms_ev3_smux_port_type;

#endif /* MS_EV3_SMUX_H_ */
