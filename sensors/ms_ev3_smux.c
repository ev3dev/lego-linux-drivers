/*
 * mindsensors.com EV3 Sensor Multiplexer device driver
 *
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

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <lego.h>
#include <lego_port_class.h>

#include "ev3_analog_sensor.h"
#include "ev3_uart_sensor.h"
#include "ms_ev3_smux.h"

enum ms_ev3_smux_mode {
	MS_EV3_SMUX_MODE_UART,
	MS_EV3_SMUX_MODE_ANALOG,
	NUM_MS_EV3_SMUX_MODES
};

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new modes have the same layout. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the ev3dev-kpkg repository.
 */

static const struct lego_port_mode_info ms_ev3_smux_port_mode_info[] = {
	/**
	 * @description: mindsensors.com EV3 Sensor Multiplexer Input Port
	 * @connection_types: EV3/UART, EV3/Analog
	 * @prefix: mux
	 */
	[MS_EV3_SMUX_MODE_UART] = {
		/**
		 * [^uart-mode]: UART sensor types are not automatically
		 * detected. By default, the `lego-ev3-color` driver
		 * will be loaded when this mode is set.
		 * Use `set_device` to load other sensor devices/drivers.
		 *
		 * Supported sensors are:
		 *
		 * | Sensor Name         | Device/Driver Name |
		 * |---------------------|--------------------|
		 * | LEGO EV3 Color      | `lego-ev3-color`   |
		 * | LEGO EV3 Ultrasonic | `lego-ev3-us`      |
		 * | LEGO EV3 GYRO       | `lego-ev3-gyro`    |
		 * | LEGO EV3 Infrared   | `lego-ev3-ir`      |
		 *
		 * @description: EV3/UART sensor
		 * @name_footnote: [^uart-mode]
		 */
		.name	= "uart",
	},
	[MS_EV3_SMUX_MODE_ANALOG] = {
		/**
		 * [^analog-mode]: Currently only the LEGO EV3 Touch sensor is
		 * supported. The `lego-ev3-touch` device/driver is loaded when
		 * this mode is set.
		 *
		 * @description: EV3/Analog sensor
		 * @name_footnote: [^analog-mode]
		 */
		.name	= "analog",
	},
};

static const char *ms_ev3_smux_uart_sensor_names[] = {
	LEGO_EV3_COLOR_NAME,
	LEGO_EV3_ULTRASONIC_NAME,
	LEGO_EV3_GYRO_NAME,
	LEGO_EV3_INFRARED_NAME,
	NULL
};

static const char *ms_ev3_smux_analog_sensor_names[] = {
	LEGO_EV3_TOUCH_SENSOR_NAME,
	NULL
};

static const char **ms_ev3_smux_sensor_names[] = {
	[MS_EV3_SMUX_MODE_UART] = ms_ev3_smux_uart_sensor_names,
	[MS_EV3_SMUX_MODE_ANALOG] = ms_ev3_smux_analog_sensor_names,
};

/**
 * struct ms_ev3_smux_data - Driver data for an input port on the mux
 * @port: The lego_port class device for this mux port.
 * @sensor: The sensor attached to this port.
 */
struct ms_ev3_smux_data {
	struct lego_port_device port;
	struct lego_device *sensor;
};

static const struct device_type ms_ev3_smux_device_type[] = {
	[MS_EV3_SMUX_MODE_UART] = {
		.name = "ev3-uart-sensor",
	},
	[MS_EV3_SMUX_MODE_ANALOG] = {
		.name = "ev3-analog-sensor",
	},
};

static int ms_ev3_smux_set_device(void* context, const char* name)
{
	struct nxt_i2c_sensor_data *data = context;
	struct ms_ev3_smux_data *smux = data->callback_data;
	struct lego_device *new_sensor;
	const char **match_name = ms_ev3_smux_sensor_names[smux->port.mode];

	while (*match_name) {
		if (!strcmp(name, *match_name))
			break;
		match_name++;
	}
	if (!*match_name)
		return -EINVAL;

	if (smux->sensor)
		lego_device_unregister(smux->sensor);
	new_sensor = lego_device_register(name,
				&ms_ev3_smux_device_type[smux->port.mode],
				&smux->port, NULL, 0);
	if (IS_ERR(new_sensor))
		return PTR_ERR(new_sensor);
	smux->sensor = new_sensor;

	return 0;
}

static int ms_ev3_smux_set_mode(void *context, u8 mode)
{
	struct nxt_i2c_sensor_data *data = context;
	struct ms_ev3_smux_data *smux = data->callback_data;
	int ret;

	ret = i2c_smbus_write_byte_data(data->client, MS_EV3_SMUX_MODE_REG,
					(mode == MS_EV3_SMUX_MODE_UART) ?
					0 : MS_EV3_SMUX_ANALOG_MODE_DATA);
	if (ret < 0)
		return ret;

	/*
	 * TODO: could potentially detect device here.
	 * Waiting on feedback from mindsensors
	 */
	ret = ms_ev3_smux_set_device(data, ms_ev3_smux_sensor_names[mode][0]);
	if (ret < 0)
		dev_warn(&smux->port.dev, "Failed to set device (%d)", ret);

	return 0;
}

void ms_ev3_smux_poll_cb(struct nxt_i2c_sensor_data *data)
{
	struct ms_ev3_smux_data *smux = data->callback_data;

	if (!smux->sensor || !smux->port.raw_data)
		return;

	i2c_smbus_read_i2c_block_data(data->client, MS_EV3_SMUX_DATA_REG,
		smux->port.raw_data_size, smux->port.raw_data);
	lego_port_call_raw_data_func(&smux->port);
}

static int ms_ev3_smux_set_ev3_uart_sensor_mode(void *context, u8 mode)
{
	struct nxt_i2c_sensor_data *data = context;

	return i2c_smbus_write_byte_data(data->client, MS_EV3_SMUX_MODE_REG,
					 mode);
}

static struct lego_port_ev3_uart_ops ms_ev3_smux_ev3_uart_ops = {
	.set_mode = ms_ev3_smux_set_ev3_uart_sensor_mode,
};

const struct device_type ms_ev3_smux_port_type = {
	.name	= "ms-ev3-smux-port",
};
EXPORT_SYMBOL_GPL(ms_ev3_smux_port_type);

int ms_ev3_smux_probe_cb(struct nxt_i2c_sensor_data *data)
{
	struct ms_ev3_smux_data *smux;
	int ret;

	smux = kzalloc(sizeof(struct ms_ev3_smux_data), GFP_KERNEL);
	if (!smux)
		return -ENOMEM;

	/*
	 * Expects sensor to return the string "CH1" (or 2/3) at 0x18, so we
	 * read the 3rd byte and convert ascii char to an integer.
	*/
	ret = i2c_smbus_read_byte_data(data->client, 0x1A);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"Could not read channel from mux (%d).\n", ret);
		goto err_get_channel;
	}
	ret -= '0';
	smux->port.name = ms_ev3_smux_port_type.name;
	snprintf(smux->port.port_name, LEGO_PORT_NAME_SIZE, "%s:mux%d",
		 data->port_name, ret);
	smux->port.num_modes = NUM_MS_EV3_SMUX_MODES;
	smux->port.mode_info = ms_ev3_smux_port_mode_info;
	smux->port.set_mode = ms_ev3_smux_set_mode;
	smux->port.set_device = ms_ev3_smux_set_device;
	smux->port.ev3_uart_ops = &ms_ev3_smux_ev3_uart_ops;
	smux->port.context = data;

	ret = lego_port_register(&smux->port, &ms_ev3_smux_port_type,
				 &data->client->dev);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"Failed to register lego-port. (%d)", ret);
		goto err_lego_port_register;
	}
	data->callback_data = smux;
	ret = ms_ev3_smux_set_mode(data, 0);
	if (ret < 0)
		dev_warn(&data->client->dev,
			 "Failed to set default mode (%d).\n", ret);

	return 0;

err_lego_port_register:
	data->callback_data = NULL;
err_get_channel:
	kfree(smux);
	return ret;
}

void ms_ev3_smux_remove_cb(struct nxt_i2c_sensor_data *data)
{
	struct ms_ev3_smux_data *smux = data->callback_data;

	if (smux) {
		if (smux->sensor)
			lego_device_unregister(smux->sensor);
		lego_port_unregister(&smux->port);
		data->callback_data = NULL;
		kfree(smux);
	}
}
