/*
 * Hub definitions for LEGO WeDo
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

#ifndef _LINUX_WEDO_HUB_H
#define _LINUX_WEDO_HUB_H

#include <linux/device.h>
#include <linux/types.h>
#include <linux/legoev3/dc_motor_class.h>

#include <lego_sensor_class.h>

#include "wedo_port.h"

#define WEDO_HUB_NAME_SIZE	30

struct wedo_hub_device {
	char name[WEDO_HUB_NAME_SIZE + 1];
	char port_name[WEDO_HUB_NAME_SIZE + 1];
	struct device dev;

	struct usb_wedo *wd;

	struct {
		struct {
			int clear_error;
			int high_power;
			int shut_down;
			int reset;
			int echo_bit;
		} status;
	} to_hub;

	struct {
		struct {
			int error;
			int high_power;
			int echo_bit;
		} status;
		unsigned char voltage;
	} from_hub;

	struct wedo_port_device *wpd[WEDO_PORT_MAX];

	void (*event_callback)(struct wedo_hub_device *);
};

#define to_wedo_hub_device(_dev) container_of(_dev, struct wedo_hub_device, dev)

extern struct bus_type wedo_bus_type;

extern int register_wedo_hub(struct wedo_hub_device *, struct device *);
extern void unregister_wedo_hub(struct wedo_hub_device *);

extern void wedo_hub_update_status(struct wedo_hub_device *);

#endif /* _LINUX_WEDO_HUB_H */

