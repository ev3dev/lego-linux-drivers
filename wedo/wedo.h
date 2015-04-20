/*
 * LEGO WeDo driver
 *
 * Copyright (C) 2014 Ralph Hempel <rhemple@hempeldesigngroup.com>
 * Copyright (C) 2014-2015 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _WEDO_H_
#define _WEDO_H_

#include <linux/usb.h>

#include <lego_port_class.h>
#include <lego_sensor_class.h>
#include <dc_motor_class.h>
#include <servo_motor_class.h>

enum wedo_ports {
	WEDO_PORT_1,
	WEDO_PORT_2,
	WEDO_PORT_MAX,
};

#define WEDO_OUTPUT_BRAKE		0x80
#define WEDO_OUTPUT_MAX_DUTY_CYCLE	127

extern void wedo_hub_request_output_update(struct usb_interface *interface);

struct wedo_port_data;

extern struct wedo_port_data *register_wedo_port(struct usb_interface *interface,
						 enum wedo_ports port_num);
extern void unregister_wedo_port(struct wedo_port_data *wpd);

extern void wedo_port_update_status(struct wedo_port_data *wpd);
extern void wedo_port_update_output(struct wedo_port_data *wpd, signed char value);

extern const struct dc_motor_ops wedo_motor_ops;

struct wedo_motor_data {
	struct wedo_port_data *wpd;
	struct dc_motor_device md;

	enum dc_motor_internal_command command;
	int duty_cycle;
};

extern const struct servo_motor_ops wedo_servo_ops;

struct wedo_servo_data {
	struct wedo_port_data *wpd;
	struct servo_motor_device sd;

	enum servo_motor_command command;
	int scaled_position;
};

enum wedo_sensor_types {
	WEDO_TILT_SENSOR,
	WEDO_MOTION_SENSOR,
};

/**
 * struct wedo_sensor_info
 * @name: The driver name. Must match name in id_table.
 * @ms_mode_info: Array of lego-sensor mode information for each sensor mode.
 * @wedo_mode_info: Array of wedo sensor specific mode information for each
 * 	sensor mode.
 * @num_modes: Number of valid elements in the mode_info array.
 */
struct wedo_sensor_info {
	const char* name;
	struct lego_sensor_mode_info mode_info[LEGO_SENSOR_MODE_MAX + 1];
	int num_modes;
};

extern const struct wedo_sensor_info wedo_sensor_defs[];

struct wedo_sensor_data {
	struct wedo_port_data *wpd;
	struct lego_sensor_device sensor;
	struct wedo_sensor_info info;
};

enum wedo_type_id {
	WEDO_TYPE_SHORTLO,
	WEDO_TYPE_BEND,
	WEDO_TYPE_TILT,
	WEDO_TYPE_FUTURE,
	WEDO_TYPE_RAW,
	WEDO_TYPE_TOUCH,
	WEDO_TYPE_SERVO,
	WEDO_TYPE_SOUND,
	WEDO_TYPE_TEMP,
	WEDO_TYPE_LIGHT,
	WEDO_TYPE_MOTION,
	WEDO_TYPE_LIGHTBRICK,
	WEDO_TYPE_22,
	WEDO_TYPE_OPEN,
	WEDO_TYPE_MOTOR,
	WEDO_TYPE_SHORTHI,
	WEDO_TYPE_MAX,
};

struct wedo_port_data {
	struct usb_interface *usb;
	struct lego_port_device port;
	struct wedo_sensor_data *sensor_data;
	struct wedo_motor_data *motor_data;
	struct wedo_led_data *led_data;
	struct wedo_servo_data *servo_data;
	struct work_struct register_device_work;
	unsigned type_debounce;
	enum wedo_type_id new_type_id;
	enum wedo_type_id type_id;
	unsigned char id;
	unsigned char input;
	signed char output;
};

#endif /* _WEDO_H_ */
