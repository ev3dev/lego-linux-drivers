/*
 * Dexter Industries BrickPi driver
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

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new modes have the same layout. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the ev3dev-kpkg repository.
 */

#include "brickpi_internal.h"
#include "../sensors/nxt_analog_sensor.h"

const struct device_type brickpi_in_port_type = {
	.name   = "brickpi-in-port",
};
EXPORT_SYMBOL_GPL(brickpi_in_port_type);

static const struct device_type brickpi_nxt_analog_sensor_device_type = {
	.name	= "nxt-analog-sensor",
};

static const struct lego_port_mode_info brickpi_in_port_mode_info[] = {
	/**
	 * @description: HiTechnic NXT Sensor Multiplexer Input Port
	 * @connection_types: I2C/NXT, Analog/NXT
	 * @prefix: mux
	 */
	[BRICKPI_IN_PORT_MODE_NONE] = {
		/**
		 * @description: No sensor
		 */
		.name	= "none",
	},
	[BRICKPI_IN_PORT_MODE_NXT_ANALOG] = {
		/**
		 * [^nxt-analog-mode]: The [nxt-analog] driver will be loaded when
		 * this mode is set. You must manually specify the correct
		 * driver for your sensor using `set_device` if you want to use
		 * another driver. Any driver with a connection type of
		 * Analog/NXT is allowed.
		 * ^
		 * [nxt-analog]: /docs/sensors/generic-nxt-analog-sensor
		 *
		 * @description: Analog/NXT sensor
		 * @name_footnote: [^nxt-analog-mode]
		 */
		.name	= "nxt-analog",
	},
	[BRICKPI_IN_PORT_MODE_NXT_COLOR] = {
		/**
		 * @description: LEGO NXT Color sensor
		 */
		.name	= "nxt-color",
	},
	[BRICKPI_IN_PORT_MODE_NXT_I2C] = {
		/**
		 * [^nxt-i2c-mode]: No sensors are loaded by default. You must
		 * manually specify the sensor that is connected by using the
		 * `set_device` attribute.
		 *
		 * @description: I2C/NXT sensor
		 * @name_footnote: [^nxt-i2c-mode]
		 */
		.name	= "nxt-i2c",
	},
	[BRICKPI_IN_PORT_MODE_EV3_ANALOG] = {
		/**
		 * [^ev3-analog-mode]: Only the LEGO EV3 Touch sensor is supported.
		 * The driver will load by default.
		 *
		 * @description: Analog/EV3 sensor
		 * @name_footnote: [^ev3-analog-mode]
		 */
		.name	= "ev3-analog",
	},
	[BRICKPI_IN_PORT_MODE_EV3_UART] = {
		/**
		 * [^ev3-uart-mode]: Only the LEGO EV3 Ultrasonic, Color, Gyro,
		 * and Infrared sensors are supported. They cannot be automatically
		 * detected, so you must specify the sensor manually using the
		 * `set_device` attribute.
		 *
		 * @description: UART/EV3 sensor
		 * @name_footnote: [^ev3-uart-mode]
		 */
		.name	= "ev3-uart",
	},
};

int brickpi_register_nxt_analog_sensor(struct brickpi_in_port_data *in_port,
				       const char *name)
{
	struct lego_device *new_sensor;

	new_sensor = lego_device_register(name,
		&brickpi_nxt_analog_sensor_device_type, &in_port->port, NULL, 0);
	if (IS_ERR(new_sensor))
		return PTR_ERR(new_sensor);

	in_port->sensor = new_sensor;

	return 0;
}

void brickpi_unregister_sensor(struct brickpi_in_port_data *in_port)
{
	if (in_port->sensor) {
		lego_device_unregister(in_port->sensor);
		in_port->sensor = NULL;
	}
}

static int brickpi_in_port_set_device(void *context, const char *name)
{
	struct brickpi_in_port_data *in_port = context;

	switch (in_port->port.mode) {
	case BRICKPI_IN_PORT_MODE_NXT_ANALOG:
		brickpi_unregister_sensor(in_port);
		return brickpi_register_nxt_analog_sensor(in_port, name);
	}

	return -EOPNOTSUPP;
}

static int brickpi_in_port_set_pin5_gpio(void *context,
					 enum lego_port_gpio_state state)
{
	struct brickpi_in_port_data *in_port = context;

	if (unlikely(in_port->sensor_type > BRICKPI_SENSOR_TYPE_NXT_ANALOG_MAX))
		return -EINVAL;

	switch (state) {
	case LEGO_PORT_GPIO_FLOAT:
		in_port->sensor_type &= ~BRICKPI_NXT_ANALOG_FLAG_PIN5_OUT;
		in_port->sensor_type &= ~BRICKPI_NXT_ANALOG_FLAG_PIN5_HIGH;
		break;
	case LEGO_PORT_GPIO_LOW:
		in_port->sensor_type |= BRICKPI_NXT_ANALOG_FLAG_PIN5_OUT;
		in_port->sensor_type &= ~BRICKPI_NXT_ANALOG_FLAG_PIN5_HIGH;
		break;
	case LEGO_PORT_GPIO_HIGH:
		in_port->sensor_type |= BRICKPI_NXT_ANALOG_FLAG_PIN5_OUT;
		in_port->sensor_type |= BRICKPI_NXT_ANALOG_FLAG_PIN5_HIGH;
		break;
	}

	return brickpi_set_sensors(in_port->ch_data);
}

static int brickpi_in_port_set_mode(void *context, u8 mode)
{
	struct brickpi_in_port_data *in_port = context;
	int err;

	brickpi_unregister_sensor(in_port);
	switch (mode) {
	case BRICKPI_IN_PORT_MODE_NONE:
		in_port->sensor_type = BRICKPI_SENSOR_TYPE_FW_VERSION;
		return brickpi_set_sensors(in_port->ch_data);
	case BRICKPI_IN_PORT_MODE_NXT_ANALOG:
		in_port->sensor_type = BRICKPI_SENSOR_TYPE_NXT_ANALOG;
		err = brickpi_set_sensors(in_port->ch_data);
		if (err < 0)
			return err;
		return brickpi_register_nxt_analog_sensor(in_port,
						GENERIC_NXT_ANALOG_SENSOR_NAME);
	}

	return -EOPNOTSUPP;
}

static struct lego_port_nxt_analog_ops brickpi_in_port_nxt_analog_ops = {
	.set_pin5_gpio = brickpi_in_port_set_pin5_gpio,
};

int brickpi_register_in_ports(struct brickpi_channel_data *ch_data,
			      struct device *parent)
{
	int i, err;

	for (i = 0; i < NUM_BRICKPI_PORT; i++) {
		struct lego_port_device *port = &ch_data->in_port[i].port;
		port->name = brickpi_in_port_type.name;
		snprintf(port->port_name, LEGO_PORT_NAME_SIZE, "%s:in%d",
			 dev_name(parent), ch_data->address * 2 + i - 1);
		port->num_modes = NUM_BRICKPI_IN_PORT_MODES;
		port->mode_info = brickpi_in_port_mode_info;
		port->set_mode = brickpi_in_port_set_mode;
		port->set_device = brickpi_in_port_set_device;
		port->context = &ch_data->in_port[i];
		port->nxt_analog_ops = &brickpi_in_port_nxt_analog_ops;
		//port->nxt_i2c_ops = &brickpi_in_port_nxt_i2c_ops;

		err = lego_port_register(port, &brickpi_in_port_type, parent);
		if (err) {
			dev_err(parent,
				"Failed to register BrickPi input port. (%d)\n",
				err);
			for (i--; i >= 0; i--)
				lego_port_unregister(port);
			return err;
		}
	}

	return 0;
}

void brickpi_unregister_in_ports(struct brickpi_channel_data *ch_data)
{
	int i;

	for (i = 0; i < NUM_BRICKPI_PORT; i++) {
		brickpi_unregister_sensor(&ch_data->in_port[i]);
		lego_port_unregister(&ch_data->in_port[i].port);
	}
}
