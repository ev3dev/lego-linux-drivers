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

#include "brickpi_internal.h"
#include "../sensors/ev3_analog_sensor.h"
#include "../sensors/ev3_uart_sensor.h"
#include "../sensors/nxt_analog_sensor.h"
#include "../sensors/nxt_i2c_sensor.h"

const struct device_type brickpi_in_port_type = {
	.name   = "brickpi-in-port",
};
EXPORT_SYMBOL_GPL(brickpi_in_port_type);


static const struct device_type brickpi_in_port_device_types[NUM_BRICKPI_IN_PORT_MODES] = {
	[BRICKPI_IN_PORT_MODE_NONE] = {
		.name = NULL,
	},
	[BRICKPI_IN_PORT_MODE_NXT_ANALOG] = {
		.name = "nxt-analog-sensor",
	},
	[BRICKPI_IN_PORT_MODE_NXT_COLOR] = {
		.name = "nxt-color-sensor",
	},
	[BRICKPI_IN_PORT_MODE_NXT_I2C] = {
		.name = "brickpi-i2c-sensor",
	},
	[BRICKPI_IN_PORT_MODE_EV3_ANALOG] = {
		.name = "ev3-analog-sensor",
	},
	[BRICKPI_IN_PORT_MODE_EV3_UART] = {
		.name = "ev3-uart-sensor",
	},
};

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new modes have the same layout. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the ev3dev-kpkg repository.
 */

static const struct lego_port_mode_info brickpi_in_port_mode_info[NUM_BRICKPI_IN_PORT_MODES] = {
	/**
	 * @description: Dexter Industries BrickPi Input Port
	 * @connection_types: NXT/Analog, NXT/I2C, EV3/Analog, EV3/UART
	 * @prefix: in
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
		 * NXT/Analog is allowed.
		 * ^
		 * [nxt-analog]: /docs/sensors/generic-nxt-analog-sensor
		 *
		 * @description: NXT/Analog sensor
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
		 * @description: NXT/I2C sensor
		 * @name_footnote: [^nxt-i2c-mode]
		 */
		.name	= "nxt-i2c",
	},
	[BRICKPI_IN_PORT_MODE_EV3_ANALOG] = {
		/**
		 * [^ev3-analog-mode]: Only the LEGO EV3 Touch sensor is supported.
		 * The driver will load by default.
		 *
		 * @description: EV3/Analog sensor
		 * @name_footnote: [^ev3-analog-mode]
		 */
		.name	= "ev3-analog",
	},
	[BRICKPI_IN_PORT_MODE_EV3_UART] = {
		/**
		 * [^ev3-uart-mode]: Only the LEGO EV3 Ultrasonic, Color, Gyro,
		 * and Infrared sensors are supported. They cannot be automatically
		 * detected, so you must specify the sensor manually using the
		 * `set_device` attribute. No sensors are loaded by default.
		 *
		 * @description: EV3/UART sensor
		 * @name_footnote: [^ev3-uart-mode]
		 */
		.name	= "ev3-uart",
	},
};

int brickpi_in_port_register_sensor(struct brickpi_in_port_data *in_port,
				    const struct device_type *device_type,
				    const char *name)
{
	struct lego_device *new_sensor;

	if (device_type == &brickpi_in_port_device_types[BRICKPI_IN_PORT_MODE_NXT_I2C]) {
		struct brickpi_i2c_sensor_platform_data pdata;

		pdata.address = in_port->i2c_msg[0].addr;
		new_sensor = lego_device_register(name, device_type,
			&in_port->port, &pdata,
			sizeof(struct brickpi_i2c_sensor_platform_data));
	} else {
		new_sensor = lego_device_register(name, device_type,
						  &in_port->port, NULL, 0);
	}

	if (IS_ERR(new_sensor))
		return PTR_ERR(new_sensor);

	in_port->sensor = new_sensor;

	return 0;
}

void brickpi_in_port_unregister_sensor(struct brickpi_in_port_data *in_port)
{
	if (in_port->sensor) {
		lego_device_unregister(in_port->sensor);
		in_port->sensor = NULL;
		in_port->sensor_type = BRICKPI_SENSOR_TYPE_FW_VERSION;
		brickpi_set_sensors(in_port->ch_data);
	}
}

static int brickpi_in_port_set_device(void *context, const char *name)
{
	struct brickpi_in_port_data *in_port = context;
	int mode = in_port->port.mode;

	brickpi_in_port_unregister_sensor(in_port);

	if (mode == BRICKPI_IN_PORT_MODE_NONE)
		return -EOPNOTSUPP;
	else if (mode == BRICKPI_IN_PORT_MODE_NXT_I2C) {
		char i2c_name[I2C_NAME_SIZE] = { 0 };
		char *blank, end;
		int ret, hex;

		/* TODO: Modify to allow multiple sensors */

		/* credit: parameter parsing code copied from i2c_core.c */
		blank = strchr(name, ' ');
		if (!blank) {
			dev_err(&in_port->port.dev, "%s: Missing parameters\n",
				"set_device");
			return -EINVAL;
		}
		if (blank - name >= I2C_NAME_SIZE) {
			dev_err(&in_port->port.dev, "%s: Invalid device name\n",
				"set_device");
			return -EINVAL;
		}
		memcpy(i2c_name, name, blank - name);

		/* Parse remaining parameters, reject extra parameters */
		ret = sscanf(++blank, "%hhu%c%x", &in_port->i2c_msg[0].addr,
			     &end, &hex);
		if (ret < 1) {
			dev_err(&in_port->port.dev, "%s: Can't parse I2C address\n",
				"set_device");
			return -EINVAL;
		}
		if (in_port->i2c_msg[0].addr == 0 && end == 'x')
			in_port->i2c_msg[0].addr = hex;
		in_port->num_i2c_msg = 1;
		name = i2c_name;
	}

	return brickpi_in_port_register_sensor(in_port,
					       &brickpi_in_port_device_types[mode], name);
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
	const char *name = NULL;
	int err;

	brickpi_in_port_unregister_sensor(in_port);
	switch (mode) {
	case BRICKPI_IN_PORT_MODE_NONE:
	case BRICKPI_IN_PORT_MODE_NXT_I2C:
		/* brickpi_i2c_sensor driver takes care of setting proper sensor_type later */
	case BRICKPI_IN_PORT_MODE_EV3_UART:
		/* We don't want to load the wrong UART sensor because it causes problems. */
		in_port->sensor_type = BRICKPI_SENSOR_TYPE_FW_VERSION;
		return brickpi_set_sensors(in_port->ch_data);
	case BRICKPI_IN_PORT_MODE_NXT_ANALOG:
		in_port->sensor_type = BRICKPI_SENSOR_TYPE_NXT_ANALOG;
		name = GENERIC_NXT_ANALOG_SENSOR_NAME;
		break;
	case BRICKPI_IN_PORT_MODE_EV3_ANALOG:
		in_port->sensor_type = BRICKPI_SENSOR_TYPE_EV3_TOUCH;
		name = LEGO_EV3_TOUCH_SENSOR_NAME;
		break;
	}

	/* TODO: This check shouldn't be needed when we have all of the modes implemented */
	if (!name)
		return -EOPNOTSUPP;

	err = brickpi_set_sensors(in_port->ch_data);
	if (err < 0)
		return err;

	return brickpi_in_port_register_sensor(in_port,
					       &brickpi_in_port_device_types[mode], name);
}

static struct lego_port_nxt_analog_ops brickpi_in_port_nxt_analog_ops = {
	.set_pin5_gpio = brickpi_in_port_set_pin5_gpio,
};

int brickpi_in_port_set_i2c_data(struct lego_device *sensor, bool slow,
				 enum lego_port_gpio_state pin1_state)
{
	struct brickpi_in_port_data *in_port =
		container_of(sensor->port, struct brickpi_in_port_data, port);

	in_port->i2c_msg[0].settings = 0;
	if (!strcmp(sensor->name, LEGO_NXT_ULTRASONIC_SENSOR_NAME))
		in_port->i2c_msg[0].settings = BRICKPI_I2C_EXTRA_CLK;
	in_port->i2c_msg[0].write_size = 0;
	in_port->i2c_msg[0].read_size = 0;
	in_port->num_i2c_msg = 0; /*gets set when setting mode */
	/*
	 * Theoretically !slow should be 100kHz (0), but in practice, many
	 * sensors have trouble so we are using 50kHz (5) instead.
	 * Slow, of course, is the LEGO standard 10KHz (10).
	 */
	in_port->i2c_speed = slow ? 10 : 5;
	in_port->sensor_type = pin1_state == LEGO_PORT_GPIO_HIGH ?
		BRICKPI_SENSOR_TYPE_NXT_I2C_9V : BRICKPI_SENSOR_TYPE_NXT_I2C;

	return 0;
}
EXPORT_SYMBOL_GPL(brickpi_in_port_set_i2c_data);

int brickpi_in_port_set_i2c_mode(struct lego_device *sensor, u8 set_mode_reg,
				 u8 set_mode_data, u8 read_reg, unsigned size)
{
	struct brickpi_in_port_data *in_port =
		container_of(sensor->port, struct brickpi_in_port_data, port);
	int err;

	if (size > BRICKPI_MAX_I2C_DATA_SIZE) {
		dev_err(&sensor->dev, "I2C message size is too big.\n");
		return -EINVAL;
	}

	in_port->num_i2c_msg = 1;
	in_port->i2c_msg[0].settings |= BRICKPI_I2C_SAME;
	if (set_mode_reg) {
		in_port->i2c_msg[0].write_size = 2;
		in_port->i2c_msg[0].write_data[0] = set_mode_reg;
		in_port->i2c_msg[0].write_data[1] = set_mode_data;
		in_port->i2c_msg[0].read_size = 0;
		err = brickpi_set_sensors(in_port->ch_data);
		if (err < 0)
			return err;
		err = brickpi_get_values(in_port->ch_data);
		if (err < 0)
			return err;
	}

	in_port->i2c_msg[0].write_size = 1;
	in_port->i2c_msg[0].write_data[0] = read_reg;
	in_port->i2c_msg[0].read_size = size;

	return brickpi_set_sensors(in_port->ch_data);
}
EXPORT_SYMBOL_GPL(brickpi_in_port_set_i2c_mode);

static int brickpi_in_port_set_ev3_uart_sensor_mode(void *context, u8 mode)
{
	struct brickpi_in_port_data *in_port = context;
	struct lego_device *sensor = in_port->sensor;

	if (!strncmp(sensor->name, LEGO_EV3_COLOR_NAME, LEGO_NAME_SIZE))
		in_port->sensor_type = BRICKPI_SENSOR_TYPE_EV3_COLOR_M0 + mode;
	else if (!strncmp(sensor->name, LEGO_EV3_ULTRASONIC_NAME, LEGO_NAME_SIZE))
		in_port->sensor_type = BRICKPI_SENSOR_TYPE_EV3_US_M0 + mode;
	else if (!strncmp(sensor->name, LEGO_EV3_GYRO_NAME, LEGO_NAME_SIZE))
		in_port->sensor_type = BRICKPI_SENSOR_TYPE_EV3_GYRO_M0 + mode;
	else if (!strncmp(sensor->name, LEGO_EV3_INFRARED_NAME, LEGO_NAME_SIZE))
		in_port->sensor_type = BRICKPI_SENSOR_TYPE_EV3_INFRARED_M0 + mode;
	else
		return -EINVAL;

	return brickpi_set_sensors(in_port->ch_data);
}

static const struct lego_port_ev3_uart_ops brickpi_ev3_uart_ops = {
	.set_mode = brickpi_in_port_set_ev3_uart_sensor_mode,
};

int brickpi_register_in_ports(struct brickpi_channel_data *ch_data,
			      struct device *parent)
{
	int i, err;

	for (i = 0; i < NUM_BRICKPI_PORT; i++) {
		struct lego_port_device *port = &ch_data->in_port[i].port;
		port->name = brickpi_in_port_type.name;
		snprintf(port->address, LEGO_NAME_SIZE, "in%d",
			 ch_data->address * 2 + i - 1);
		port->num_modes = NUM_BRICKPI_IN_PORT_MODES;
		/* only firmware version 2 supports EV3 sensors */
		if (ch_data->fw_version != 2)
			port->num_modes -= 2;
		port->mode_info = brickpi_in_port_mode_info;
		port->set_mode = brickpi_in_port_set_mode;
		port->set_device = brickpi_in_port_set_device;
		port->ev3_uart_ops = &brickpi_ev3_uart_ops;
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
		brickpi_in_port_unregister_sensor(&ch_data->in_port[i]);
		lego_port_unregister(&ch_data->in_port[i].port);
	}
}
