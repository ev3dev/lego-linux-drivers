/*
 * Dexter Industries BrickPi3 input port driver
 *
 * Copyright (C) 2017 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**
 * DOC: userspace
 *
 * The BrickPi3 has four input ports, labeled S1, S2, S3 and S4. These ports
 * are similar to the input ports on the EV3, but lack the ability to
 * automatically detect sensors.
 */

#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

#include "brickpi3.h"
#include "../sensors/ev3_analog_sensor.h"
#include "../sensors/ev3_uart_sensor.h"
#include "../sensors/nxt_analog_sensor.h"
#include "../sensors/nxt_i2c_sensor.h"

struct brickpi3_in_port {
	struct brickpi3 *bp;
	struct lego_port_device port;
	struct work_struct poll_work;
	struct hrtimer poll_timer;
	struct lego_device *sensor;
	struct i2c_adapter *i2c_adap;
	struct i2c_client *i2c_sensor;
	struct nxt_i2c_sensor_platform_data i2c_pdata;
	enum brickpi3_input_port index;
	enum brickpi3_sensor_type sensor_type;
	u8 address;
};

const struct device_type brickpi3_in_port_type = {
	.name   = "brickpi3-in-port",
};
EXPORT_SYMBOL_GPL(brickpi3_in_port_type);

static const struct device_type brickpi3_in_port_device_types[NUM_BRICKPI3_IN_PORT_MODES] = {
	[BRICKPI3_IN_PORT_MODE_NONE] = {
		.name = NULL,
	},
	[BRICKPI3_IN_PORT_MODE_NXT_ANALOG] = {
		.name = "nxt-analog-sensor",
	},
	[BRICKPI3_IN_PORT_MODE_NXT_COLOR] = {
		.name = "nxt-color-sensor",
	},
	[BRICKPI3_IN_PORT_MODE_NXT_I2C] = {
		.name = "nxt-i2c-sensor",
	},
	[BRICKPI3_IN_PORT_MODE_EV3_ANALOG] = {
		.name = "ev3-analog-sensor",
	},
	[BRICKPI3_IN_PORT_MODE_EV3_UART] = {
		.name = "ev3-uart-sensor",
	},
};

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new modes have the same layout. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the Documentation/json/ directory.
 */

static const struct lego_port_mode_info brickpi3_in_port_mode_info[NUM_BRICKPI3_IN_PORT_MODES] = {
	/**
	 * .. [#in-port-prefix] The full port name includes the parent device
	 *    node. So, the ``address`` attribute will return something like
	 *    ``spi0.1:S1``.
	 *
	 * @description: Dexter Industries BrickPi3 Input Port
	 * @connection_types: NXT/Analog, NXT/I2C, EV3/Analog, EV3/UART
	 * @prefix: S
	 * @prefix_footnote: [#in-port-prefix]_
	 * @module: brickpi3
	 */
	[BRICKPI3_IN_PORT_MODE_NONE] = {
		/**
		 * @description: No sensor
		 */
		.name	= "none",
	},
	[BRICKPI3_IN_PORT_MODE_NXT_ANALOG] = {
		/**
		 * .. [#nxt-analog-mode] The ``nxt-analog`` driver will be loaded
		 *    when this mode is set. You must manually specify the
		 *    correct driver for your sensor using ``set_device`` if you
		 *    want to use another driver. Any driver with a connection
		 *    type of NXT/Analog is allowed.
		 *
		 * .. _nxt-analog: /docs/sensors/generic-nxt-analog-sensor
		 *
		 * @description: NXT/Analog sensor
		 * @name_footnote: [#nxt-analog-mode]_
		 */
		.name	= "nxt-analog",
	},
	[BRICKPI3_IN_PORT_MODE_NXT_COLOR] = {
		/**
		 * @description: LEGO NXT Color sensor
		 */
		.name	= "nxt-color",
	},
	[BRICKPI3_IN_PORT_MODE_NXT_I2C] = {
		/**
		 * .. [#nxt-i2c-mode] No sensors are loaded by default. You
		 *    must manually specify the sensor that is connected and
		 *    its address by using the ``set_device`` attribute. This
		 *    is equivalent to `manually loading`_ I2C devices. The
		 *    sensor port address will be the BrickPi3 port address
		 *    with ``:i2c`` and the decimal I2C address appended.
		 *
		 * .. _manually loading: /docs/sensors/using-i2c-sensors/#manually-loading-devices
		 *
		 * @description: NXT/I2C sensor
		 * @name_footnote: [#nxt-i2c-mode]_
		 */
		.name	= "nxt-i2c",
	},
	[BRICKPI3_IN_PORT_MODE_EV3_ANALOG] = {
		/**
		 * .. [#ev3-analog-mode] Only the LEGO EV3 Touch sensor is
		 *    supported. The ``lego-ev3-touch`` sensor driver will
		 *    load automatically when this port mode is set.
		 *
		 * @description: EV3/Analog sensor
		 * @name_footnote: [#ev3-analog-mode]_
		 */
		.name	= "ev3-analog",
	},
	[BRICKPI3_IN_PORT_MODE_EV3_UART] = {
		/**
		 * .. [#ev3-uart-mode] Only the LEGO EV3 Ultrasonic, Color,
		 *    Gyro, and Infrared sensors are supported. They cannot be
		 *    automatically detected, so you must specify the sensor
		 *    manually using the ``set_device`` attribute. No sensor
		 *    drivers are loaded by default.
		 *
		 * @description: EV3/UART sensor
		 * @name_footnote: [#ev3-uart-mode]_
		 */
		.name	= "ev3-uart",
	},
};

static void brickpi3_in_port_poll_work(struct work_struct *work)
{
	struct brickpi3_in_port *data =
		container_of(work, struct brickpi3_in_port, poll_work);
	u8 *raw_data = data->port.raw_data;
	u8 msg[16];
	int ret;

	switch (data->sensor_type) {
	case BRICKPI3_SENSOR_TYPE_CUSTOM:
		ret = brickpi3_read_sensor(data->bp, data->address, data->index,
					   data->sensor_type, msg, 4);
		if (ret < 0)
			return;

		/* for now, just handling NXT analog */
		if (raw_data) {
			u16 raw = be16_to_cpu(*(__be16 *)msg);

			*(u16 *)raw_data = (raw * 5001) >> 12;
		}
		break;
	case BRICKPI3_SENSOR_TYPE_EV3_TOUCH:
		ret = brickpi3_read_sensor(data->bp, data->address, data->index,
					   data->sensor_type, msg, 1);
		if (ret < 0)
			return;

		/* convert to value that the EV3 analog driver expects  */
		if (raw_data)
			*(u16 *)raw_data = msg[0] ? 500 : 0;
		break;
	case BRICKPI3_SENSOR_TYPE_EV3_COLOR_REFLECTED:
	case BRICKPI3_SENSOR_TYPE_EV3_COLOR_AMBIENT:
	case BRICKPI3_SENSOR_TYPE_EV3_COLOR_COLOR:
	case BRICKPI3_SENSOR_TYPE_EV3_ULTRASONIC_LISTEN:
	case BRICKPI3_SENSOR_TYPE_EV3_INFRARED_PROXIMITY:
	case BRICKPI3_SENSOR_TYPE_EV3_INFRARED_REMOTE:
		ret = brickpi3_read_sensor(data->bp, data->address, data->index,
					   data->sensor_type, msg, 1);
		if (ret < 0)
			return;

		if (raw_data)
			raw_data[0] = msg[0];
		break;
	case BRICKPI3_SENSOR_TYPE_EV3_GYRO_ABS:
	case BRICKPI3_SENSOR_TYPE_EV3_GYRO_DPS:
	case BRICKPI3_SENSOR_TYPE_EV3_ULTRASONIC_CM:
	case BRICKPI3_SENSOR_TYPE_EV3_ULTRASONIC_INCHES:
		ret = brickpi3_read_sensor(data->bp, data->address, data->index,
					   data->sensor_type, msg, 2);
		if (ret < 0)
			return;

		if (raw_data) {
			raw_data[0] = msg[1];
			raw_data[1] = msg[0];
		}
		break;
	case BRICKPI3_SENSOR_TYPE_EV3_GYRO_ABS_DPS:
	case BRICKPI3_SENSOR_TYPE_EV3_COLOR_RAW_REFLECTED:
		ret = brickpi3_read_sensor(data->bp, data->address, data->index,
					   data->sensor_type, msg, 4);
		if (ret < 0)
			return;

		if (raw_data) {
			raw_data[0] = msg[3];
			raw_data[1] = msg[2];
			raw_data[2] = msg[1];
			raw_data[0] = msg[0];
		}
		break;
	case BRICKPI3_SENSOR_TYPE_EV3_COLOR_COLOR_COMPONENTS:
		ret = brickpi3_read_sensor(data->bp, data->address, data->index,
					   data->sensor_type, msg, 8);
		if (ret < 0)
			return;

		if (raw_data) {
			raw_data[0] = msg[1];
			raw_data[1] = msg[0];
			raw_data[2] = msg[3];
			raw_data[3] = msg[2];
			raw_data[4] = msg[5];
			raw_data[5] = msg[4];
			raw_data[6] = msg[7];
			raw_data[7] = msg[6];
		}
		break;
	case BRICKPI3_SENSOR_TYPE_EV3_INFRARED_SEEK:
		ret = brickpi3_read_sensor(data->bp, data->address, data->index,
					   data->sensor_type, msg, 8);
		if (ret < 0)
			return;

		if (raw_data)
			memcpy(raw_data, msg, 8);
		break;
	default:
		return;
	}

	if (raw_data) {
		lego_port_call_raw_data_func(&data->port);
	}
}

static enum hrtimer_restart brickpi3_in_port_poll_timer_function(struct hrtimer *timer)
{
	struct brickpi3_in_port *data =
		container_of(timer, struct brickpi3_in_port, poll_timer);

	/* TODO: make poll time configurable */
	hrtimer_forward_now(&data->poll_timer, ms_to_ktime(10));
	schedule_work(&data->poll_work);

	return HRTIMER_RESTART;
}

static int brickpi3_in_port_register_sensor(struct brickpi3_in_port *data,
					    const struct device_type *device_type,
					    const char *name, u8 i2c_addr)
{
	if (device_type == &brickpi3_in_port_device_types[BRICKPI3_IN_PORT_MODE_NXT_I2C]) {
		struct i2c_client *new_sensor;
		struct i2c_board_info info;

		memset(&info, 0, sizeof(info));
		strncpy(info.type, name, I2C_NAME_SIZE);
		info.addr = i2c_addr;
		info.platform_data = &data->i2c_pdata;

		new_sensor = i2c_new_device(data->i2c_adap, &info);
		if (IS_ERR(new_sensor))
			return PTR_ERR(new_sensor);

		data->i2c_sensor = new_sensor;
	} else {
		struct lego_device *new_sensor;

		new_sensor = lego_device_register(name, device_type,
						  &data->port, NULL, 0);

		if (IS_ERR(new_sensor))
			return PTR_ERR(new_sensor);

		data->sensor = new_sensor;
		hrtimer_start(&data->poll_timer, ms_to_ktime(10), HRTIMER_MODE_REL);
	}

	return 0;
}

static void brickpi3_in_port_unregister_sensor(struct brickpi3_in_port *data)
{
	if (data->sensor) {
		hrtimer_cancel(&data->poll_timer);
		cancel_work_sync(&data->poll_work);
		lego_device_unregister(data->sensor);
		data->sensor = NULL;
	}
	if (data->i2c_sensor) {
		i2c_unregister_device(data->i2c_sensor);
		data->i2c_sensor = NULL;
	}
}

static int brickpi3_in_port_set_device(void *context, const char *name)
{
	struct brickpi3_in_port *data = context;
	int mode = data->port.mode;
	u8 i2c_addr = 0;

	brickpi3_in_port_unregister_sensor(data);

	if (mode == BRICKPI3_IN_PORT_MODE_NONE)
		return -EOPNOTSUPP;
	else if (mode == BRICKPI3_IN_PORT_MODE_NXT_I2C) {
		char i2c_name[I2C_NAME_SIZE] = { 0 };
		char *blank, end;
		int ret, hex;

		/* TODO: Modify to allow multiple sensors */

		/* credit: parameter parsing code copied from i2c_core.c */
		blank = strchr(name, ' ');
		if (!blank) {
			dev_err(&data->port.dev, "%s: Missing parameters -"
				"requires both driver name and I2C address\n",
				"set_device");
			return -EINVAL;
		}
		if (blank - name >= I2C_NAME_SIZE) {
			dev_err(&data->port.dev, "%s: Invalid device name\n",
				"set_device");
			return -EINVAL;
		}
		memcpy(i2c_name, name, blank - name);

		/* Parse remaining parameters, reject extra parameters */
		ret = sscanf(++blank, "%hhu%c%x", &i2c_addr, &end, &hex);
		if (ret < 1) {
			dev_err(&data->port.dev, "%s: Can't parse I2C address\n",
				"set_device");
			return -EINVAL;
		}
		if (i2c_addr == 0 && end == 'x')
			i2c_addr = hex;
		name = i2c_name;
	}

	return brickpi3_in_port_register_sensor(data,
			&brickpi3_in_port_device_types[mode], name, i2c_addr);
}

static int brickpi3_in_port_set_mode(void *context, u8 mode)
{
	struct brickpi3_in_port *data = context;
	const char *name = NULL;
	int ret;

	brickpi3_in_port_unregister_sensor(data);
	switch (mode) {
	case BRICKPI3_IN_PORT_MODE_NONE:
	case BRICKPI3_IN_PORT_MODE_EV3_UART:
		/* We don't want to load the wrong UART sensor. */
		data->sensor_type = BRICKPI3_SENSOR_TYPE_NONE;
		return brickpi3_set_sensor_type(data->bp, data->address,
						data->index, data->sensor_type);
	case BRICKPI3_IN_PORT_MODE_NXT_ANALOG:
		data->sensor_type = BRICKPI3_SENSOR_TYPE_CUSTOM;
		ret = brickpi3_set_sensor_custom(data->bp, data->address,
					data->index, BRICKPI3_SENSOR_PIN1_ADC);
		if (ret < 0)
			return ret;
		name = GENERIC_NXT_ANALOG_SENSOR_NAME;
		break;
	case BRICKPI3_IN_PORT_MODE_NXT_I2C:
		data->sensor_type = BRICKPI3_SENSOR_TYPE_I2C;
		return brickpi3_set_sensor_i2c(data->bp, data->address,
					data->index, BRICKPI3_I2C_MID_CLOCK, 0);
		/* TODO: probe for I2C sensors here */
	case BRICKPI3_IN_PORT_MODE_EV3_ANALOG:
		data->sensor_type = BRICKPI3_SENSOR_TYPE_EV3_TOUCH;
		ret = brickpi3_set_sensor_type(data->bp, data->address,
					       data->index, data->sensor_type);
		if (ret < 0)
			return ret;
		name = LEGO_EV3_TOUCH_SENSOR_NAME;
		break;
	}

	/* TODO: This check shouldn't be needed when we have all of the modes implemented */
	if (!name)
		return -EOPNOTSUPP;

	return brickpi3_in_port_register_sensor(data,
				&brickpi3_in_port_device_types[mode], name, 0);
}

static int brickpi3_in_port_set_pin5_gpio(void *context,
					  enum lego_port_gpio_state state)
{
	struct brickpi3_in_port *data = context;
	enum brickpi3_sensor_pin_flags flags;

	/*
	 * FIXME: It is not clear how to actually set the pin high or low.
	 * Perhaps we need to use BRICKPI3_SENSOR_TYPE_NXT_LIGHT_ON/OFF.
	 */
	flags = BRICKPI3_SENSOR_PIN1_ADC;
	switch (state) {
	case LEGO_PORT_GPIO_FLOAT:
		flags |= BRICKPI3_SENSOR_PIN5_STATE;
		break;
	case LEGO_PORT_GPIO_LOW:
		flags |= BRICKPI3_SENSOR_PIN5_OUT;
		break;
	case LEGO_PORT_GPIO_HIGH:
		flags |= BRICKPI3_SENSOR_PIN5_OUT;
		break;
	}

	return brickpi3_set_sensor_custom(data->bp, data->address, data->index,
					  flags);
}

static struct lego_port_nxt_analog_ops brickpi3_in_port_nxt_analog_ops = {
	.set_pin5_gpio = brickpi3_in_port_set_pin5_gpio,
};

static int brickpi3_in_port_set_pin1_gpio(void *context,
					  enum lego_port_gpio_state state)
{
	struct brickpi3_in_port *data = context;
	enum brickpi3_i2c_flags flags;

	flags = BRICKPI3_I2C_MID_CLOCK;
	if (state == LEGO_PORT_GPIO_HIGH)
		flags |= BRICKPI3_I2C_PIN1_9V;

	return brickpi3_set_sensor_i2c(data->bp, data->address, data->index,
				       flags, 0);
}

static const struct lego_port_nxt_i2c_ops brickpi3_in_port_nxt_i2c_ops = {
	.set_pin1_gpio = brickpi3_in_port_set_pin1_gpio,
};

static int brickpi3_in_port_set_ev3_uart_sensor_mode(void *context, u8 type_id,
						     u8 mode)
{
	struct brickpi3_in_port *data = context;

	if (type_id == ev3_uart_sensor_defs[LEGO_EV3_COLOR].type_id)
		data->sensor_type = BRICKPI3_SENSOR_TYPE_EV3_COLOR_REFLECTED + mode;
	else if (type_id == ev3_uart_sensor_defs[LEGO_EV3_ULTRASONIC].type_id)
		data->sensor_type = BRICKPI3_SENSOR_TYPE_EV3_ULTRASONIC_CM + mode;
	else if (type_id == ev3_uart_sensor_defs[LEGO_EV3_GYRO].type_id)
		data->sensor_type = BRICKPI3_SENSOR_TYPE_EV3_GYRO_ABS + mode;
	else if (type_id == ev3_uart_sensor_defs[LEGO_EV3_INFRARED].type_id)
		data->sensor_type = BRICKPI3_SENSOR_TYPE_EV3_INFRARED_PROXIMITY + mode;
	else
		return -EINVAL;

	return brickpi3_set_sensor_type(data->bp, data->address, data->index,
					data->sensor_type);
}

static const struct lego_port_ev3_uart_ops brickpi3_ev3_uart_ops = {
	.set_mode = brickpi3_in_port_set_ev3_uart_sensor_mode,
};

static void brickpi3_ports_in_release(struct device *dev, void *res)
{
	struct brickpi3_in_port *data = res;

	brickpi3_in_port_unregister_sensor(data);
	lego_port_unregister(&data->port);
}

static int devm_brickpi3_port_in_register_one(struct device *dev,
					      struct brickpi3 *bp,
					      u8 address,
					      enum brickpi3_input_port port)
{
	struct brickpi3_in_port *data;
	int ret;

	data = devres_alloc(brickpi3_ports_in_release, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->bp = bp;
	data->address = address;
	data->index = port;
	INIT_WORK(&data->poll_work, brickpi3_in_port_poll_work);
	hrtimer_init(&data->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->poll_timer.function = brickpi3_in_port_poll_timer_function;
	data->i2c_pdata.in_port = &data->port;

	/*
	 * FIXME: there is no guarantee this is the correct adapter.
	 * This works as long as the device tree specifies aliases for 3 I2C
	 * adapters and there are no other dynamic I2C adapters.
	 */
	data->i2c_adap = i2c_get_adapter(port + 3);
	if (!data->i2c_adap) {
		dev_err(dev, "Could not get I2C adapter %d\n", port + 3);
		return -ENXIO;
	}
	/*
	 * This module also owns the i2c adapters, so we would cause a reference
	 * cycle if we held on to the reference, which would prevent the module
	 * from ever being unloaded.
	 */
	i2c_put_adapter(data->i2c_adap);

	data->port.name = brickpi3_in_port_type.name;
	snprintf(data->port.address, LEGO_NAME_SIZE, "%s:S%d", dev_name(dev),
		 (address - 1) * 4 + port + 1);
	data->port.num_modes = NUM_BRICKPI3_IN_PORT_MODES;
	data->port.supported_modes = LEGO_PORT_ALL_MODES;
	data->port.mode_info = brickpi3_in_port_mode_info;
	data->port.set_mode = brickpi3_in_port_set_mode;
	data->port.set_device = brickpi3_in_port_set_device;
	data->port.context = data;
	data->port.nxt_analog_ops = &brickpi3_in_port_nxt_analog_ops;
	data->port.nxt_i2c_ops = &brickpi3_in_port_nxt_i2c_ops;
	data->port.ev3_uart_ops = &brickpi3_ev3_uart_ops;

	ret = lego_port_register(&data->port, &brickpi3_in_port_type, dev);
	if (ret < 0) {
		dev_err(dev, "Failed to register input port %s\n",
			data->port.address);
		return ret;
	}

	devres_add(dev, data);

	return 0;
}

int devm_brickpi3_register_in_ports(struct device *dev, struct brickpi3 *bp,
				    u8 address)
{
	int i, ret;

	for (i = 0; i < NUM_BRICKPI3_INPUT_PORTS; i++) {
		ret = devm_brickpi3_port_in_register_one(dev, bp, address, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}
