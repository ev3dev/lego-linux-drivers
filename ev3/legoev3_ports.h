/*
 * Support for the input and output ports on the LEGO MINDSTORMS EV3
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

#ifndef _LEGOEV3_PORTS_H_
#define _LEGOEV3_PORTS_H_

#include <linux/platform_data/legoev3.h>

extern struct lego_port_device
*ev3_input_port_register(struct ev3_input_port_platform_data *pdata,
			 struct device *parent);
extern void ev3_input_port_unregister(struct lego_port_device *port);
extern struct lego_port_device
*ev3_output_port_register(struct ev3_output_port_platform_data *pdata,
			  struct device *parent);
extern void ev3_output_port_unregister(struct lego_port_device *port);

#endif /* _LEGOEV3_PORTS_H_ */
