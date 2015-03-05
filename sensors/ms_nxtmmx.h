/*
 * mindsensors.com Motor Multiplexer device driver
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

#ifndef MS_NXTMMX_H_
#define MS_NXTMMX_H_

#include <lego_port_class.h>

#include "nxt_i2c_sensor.h"

extern int ms_nxtmmx_probe_cb(struct nxt_i2c_sensor_data *data);
extern void ms_nxtmmx_remove_cb(struct nxt_i2c_sensor_data *data);

#endif /* MS_NXTMMX_H_ */
