/*
 * mindsensors.com PiStorms input port device driver
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

#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "pistorms.h"
#include "../sensors/ev3_analog_sensor.h"
#include "../sensors/ev3_uart_sensor.h"
#include "../sensors/nxt_analog_sensor.h"
#include "../sensors/nxt_i2c_sensor.h"

#define PS_DEFAULT_POLL_MS		1000

#define PS_SENSOR_PORT_1_REG		0x6F
#define PS_SENSOR_PORT_2_REG		0xA3
#define NUM_PS_PORT			2

#define PS_SENSOR_TYPE_OFFSET		0
#define PS_NXT_ANALOG_VALUE_OFFSET	1

#define PS_NXT_COLOR_SIZE		9
#define PS_NXT_COLOR_COLOR_OFFSET	1
#define PS_NXT_COLOR_RED_OFFSET		2
#define PS_NXT_COLOR_GREEN_OFFSET	3
#define PS_NXT_COLOR_BLUE_OFFSET	4
#define PS_NXT_COLOR_AMBIANT_OFFSET	5
#define PS_NXT_COLOR_RAW_OFFSET(type)	(PS_NXT_COLOR_##type##_OFFSET + 4)

#define PS_EV3_TOUCH_ID_OFFSET		2
#define PS_EV3_TOUCH_ID_SIZE		16
#define PS_EV3_TOUCH_VALUE_OFFSET	20
#define PS_EV3_TOUCH_COUNT_OFFSET	21

#define PS_EV3_UART_STATUS_SIZE		20
#define PS_EV3_UART_READY_OFFSET	1
#define PS_EV3_UART_ID_OFFSET		2
#define PS_EV3_UART_ID_SIZE		16
#define PS_EV3_UART_MODE_OFFSET		18
#define PS_EV3_UART_LENGTH_OFFSET	19
#define PS_EV3_UART_DATA_OFFSET		20
#define PS_EV3_UART_DATA_SIZE		32

#define PS_EV3_UART_STATUS_INDEX(offset) \
	(PS_EV3_UART_##offset##_OFFSET - PS_EV3_UART_READY_OFFSET)

enum pistorms_sensor_type {
	PS_SENSOR_TYPE_NONE		= 0,
	PS_SENSOR_TYPE_NXT_ANALOG	= 2,
	PS_SENSOR_TYPE_NXT_ANALOG_PIN5	= 3,
	PS_SENSOR_TYPE_I2C_THRU		= 9,
	PS_SENSOR_TYPE_NXT_COLOR_FULL	= 13,
	PS_SENSOR_TYPE_NXT_COLOR_RED	= 14,
	PS_SENSOR_TYPE_NXT_COLOR_GREEN	= 15,
	PS_SENSOR_TYPE_NXT_COLOR_BLUE	= 16,
	PS_SENSOR_TYPE_NXT_COLOR_NONE	= 17,
	PS_SENSOR_TYPE_EV3_TOUCH	= 18,
	PS_SENSOR_TYPE_EV3_UART		= 19,
};

struct pistorms_in_port_data {
	struct lego_port_device port;
	struct work_struct poll_work;
	struct hrtimer poll_timer;
	struct nxt_i2c_sensor_platform_data i2c_platform_data;
	struct lego_device *sensor;
	struct i2c_client *i2c_sensor;
	struct i2c_client *client;
	enum pistorms_sensor_type sensor_type;
	int poll_ms;
	u8 i2c_reg;
	u8 i2c_sensor_addr;
	bool stopping;
};

const struct device_type pistorms_in_port_type = {
	.name   = "pistorms-in-port",
};
EXPORT_SYMBOL_GPL(pistorms_in_port_type);

enum pistorms_in_port_mode {
	PS_IN_PORT_MODE_NONE,
	PS_IN_PORT_MODE_NXT_ANALOG,
	PS_IN_PORT_MODE_NXT_COLOR,
	PS_IN_PORT_MODE_I2C_THRU,
	PS_IN_PORT_MODE_EV3_ANALOG,
	PS_IN_PORT_MODE_EV3_UART,
	NUM_PS_IN_PORT_MODES
};

static const struct device_type
pistorms_in_port_device_types[NUM_PS_IN_PORT_MODES] = {
	[PS_IN_PORT_MODE_NONE] = {
		.name = NULL,
	},
	[PS_IN_PORT_MODE_NXT_ANALOG] = {
		.name = "nxt-analog-sensor",
	},
	[PS_IN_PORT_MODE_NXT_COLOR] = {
		.name = "nxt-color-sensor",
	},
	[PS_IN_PORT_MODE_I2C_THRU] = {
		.name = NULL,
	},
	[PS_IN_PORT_MODE_EV3_ANALOG] = {
		.name = "ev3-analog-sensor",
	},
	[PS_IN_PORT_MODE_EV3_UART] = {
		.name = "ev3-uart-sensor",
	},
};

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new modes have the same syntax. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the ev3dev-kpkg repository.
 */

static const struct lego_port_mode_info pistorms_in_port_mode_info[NUM_PS_IN_PORT_MODES] = {
	/**
	 * [^address-prefix]: The full `address` is in the format:
	 * ^
	 *        pistorms:B<b>:<prefix><n>
	 * ^
	 *    For example, if we are looking at the port labeled "BBS2" on the
	 *    PiStorms, the address will be `pistorms:BB:S2`.
	 *
	 * @description: mindsensors.com PiStorms Input Port
	 * @connection_types: NXT/Analog, NXT/Color, NXT/I2C, Other/I2C, EV3/Analog, EV3/UART
	 * @prefix: S
	 * @prefix_footnote: [^address-prefix]
	 */
	[PS_IN_PORT_MODE_NONE] = {
		/**
		 * @description: No sensor
		 */
		.name   = "none",
	},
	[PS_IN_PORT_MODE_NXT_ANALOG] = {
		/**
		 * [^nxt-analog-mode]: The generic [nxt-analog] driver will be
		 * loaded when this mode is set. You must manually specify the
		 * correct driver for your sensor using `set_device` if you want
		 * to use another driver. Any driver with a connection type of
		 * NXT/Analog is allowed.
		 * ^
		 * [nxt-analog]: /docs/sensors/generic-nxt-analog-sensor
		 *
		 * @description: NXT/Analog sensor
		 * @name_footnote: [^nxt-analog-mode]
		 */
		.name   = "nxt-analog",
	},
	[PS_IN_PORT_MODE_NXT_COLOR] = {
		/**
		 * @description: NXT/Color sensor
		 */
		.name   = "nxt-color",
	},
	[PS_IN_PORT_MODE_I2C_THRU] = {
		/**
		 * [^i2c-thru-mode]: I2C signals are passed through the
		 * input port to `i2c_arm` on the Raspberry Pi. This means that
		 * all 4 input ports share the same I2C master and the devices
		 * must have different addresses even if they are connected to
		 * different ports on the PiStorms. Additionally, the LEGO NXT
		 * Ultrasonic sensor is not supported on the PiStorms.
		 * ^
		 * NXT/I2C sensors will be automatically detected, otherwise
		 * you must manually specify the sensor that is connected by
		 * using the `set_device` attribute.
		 *
		 * @description: I2C pass through
		 * @name_footnote: [^i2c-thru-mode]
		 */
		.name   = "i2c-thru",
	},
	[PS_IN_PORT_MODE_EV3_ANALOG] = {
		/**
		 * [^ev3-analog-mode]: Only the LEGO EV3 Touch sensor is supported.
		 * The driver will load by default.
		 *
		 * @description: EV3/Analog sensor
		 * @name_footnote: [^ev3-analog-mode]
		 */
		.name   = "ev3-analog",
	},
	[PS_IN_PORT_MODE_EV3_UART] = {
		/**
		 * [^ev3-uart-mode]: Only the LEGO EV3 Ultrasonic, Color, Gyro,
		 * and Infrared sensors are supported. When this mode is set,
		 * a sensor device will be registered for the type of sensor
		 * that is attached (or was most recently attached).
		 *
		 * @description: EV3/UART sensor
		 * @name_footnote: [^ev3-uart-mode]
		 */
		.name   = "ev3-uart",
	},
};

static void pistorms_poll_work(struct work_struct *work)
{
	struct pistorms_in_port_data *in_port =
		container_of(work, struct pistorms_in_port_data, poll_work);
	u8 *raw_data = in_port->port.raw_data;
	int ret;

	if (!raw_data)
		return;

	switch (in_port->port.mode) {
	case PS_IN_PORT_MODE_NXT_ANALOG:
		ret = i2c_smbus_read_word_data(in_port->client,
				in_port->i2c_reg + PS_NXT_ANALOG_VALUE_OFFSET);
		if (ret < 0)
			return;
		*(u32 *)raw_data = ret * 5000 / 1024;
		break;
	case PS_IN_PORT_MODE_EV3_ANALOG:
		/* for now, only supports the LEGO EV3 Touch sensor */
		ret = i2c_smbus_read_byte_data(in_port->client,
				in_port->i2c_reg + PS_EV3_TOUCH_VALUE_OFFSET);
		if (ret < 0)
			return;
		*(u32 *)raw_data = ret;
		break;
	case PS_IN_PORT_MODE_EV3_UART:
#if 0
		/*
		 * TODO: Could use the id returned to make sure it matches the
		 * currently registered sensor and change sensors if it is not.
		 * uart_status[PS_EV3_UART_STATUS_INDEX(READY)] does not seem
		 * to do anything useful. The PiStorms seems to remember
		 * uart_status[PS_EV3_UART_STATUS_INDEX(MODE)] as well, so we
		 * don't need to worry about it changing.
		 */
		u8 uart_status[PS_EV3_UART_STATUS_SIZE];

		ret = i2c_smbus_read_i2c_block_data(in_port->client,
				in_port->i2c_reg + PS_EV3_UART_READY_OFFSET,
				PS_EV3_UART_STATUS_SIZE, uart_status);
		if (ret < 0)
			return;
#endif
		ret = i2c_smbus_read_i2c_block_data(in_port->client,
				in_port->i2c_reg + PS_EV3_UART_DATA_OFFSET,
				in_port->port.raw_data_size, raw_data);
		if (ret < 0)
			return;
		break;
	}

	lego_port_call_raw_data_func(&in_port->port);
}

enum hrtimer_restart pistorms_poll_timer_function(struct hrtimer *timer)
{
	struct pistorms_in_port_data *in_port =
		container_of(timer, struct pistorms_in_port_data, poll_timer);

	if (unlikely(!in_port->poll_ms || in_port->stopping))
		return HRTIMER_NORESTART;

	hrtimer_forward_now(timer, ms_to_ktime(in_port->poll_ms));

	schedule_work(&in_port->poll_work);

	return HRTIMER_RESTART;
}

static void pistorms_in_port_start_polling(struct pistorms_in_port_data *in_port)
{
	hrtimer_start(&in_port->poll_timer, ktime_set(0, 0), HRTIMER_MODE_REL);
}

static void pistorms_in_port_stop_polling(struct pistorms_in_port_data *in_port)
{
	in_port->stopping = true;
	hrtimer_cancel(&in_port->poll_timer);
	cancel_work_sync(&in_port->poll_work);
	in_port->stopping = false;
}

static inline int pistorms_set_sensor_type(struct pistorms_in_port_data *in_port)
{
	return i2c_smbus_write_byte_data(in_port->client,
				in_port->i2c_reg + PS_SENSOR_TYPE_OFFSET,
				in_port->sensor_type);
}

int pistorms_in_port_register_sensor(struct pistorms_in_port_data *in_port,
				     const struct device_type *device_type,
				     const char *name)
{
	struct lego_device *new_sensor;

	/* It is a programming error to call this function when there is already a sensor */
	WARN_ON(in_port->sensor);
	WARN_ON(in_port->i2c_sensor);

	if (device_type == &pistorms_in_port_device_types[PS_IN_PORT_MODE_I2C_THRU]) {
		struct i2c_board_info info;

		memset(&info, 0, sizeof(struct i2c_board_info));
		snprintf(info.type, I2C_NAME_SIZE, name);
		info.platform_data = &in_port->i2c_platform_data;
		info.addr = in_port->i2c_sensor_addr;

		in_port->i2c_sensor = i2c_new_device(in_port->client->adapter,
						     &info);
		if (!in_port->i2c_sensor)
			return -EINVAL;

		return 0;
	} else
		new_sensor = lego_device_register(name, device_type,
						  &in_port->port, NULL, 0);

	if (IS_ERR(new_sensor))
		return PTR_ERR(new_sensor);

	in_port->sensor = new_sensor;
	if (in_port->poll_ms)
		pistorms_in_port_start_polling(in_port);
	else
		schedule_work(&in_port->poll_work);

	return 0;
}

void pistorms_in_port_unregister_sensor(struct pistorms_in_port_data *in_port)
{
	if (in_port->sensor) {
		pistorms_in_port_stop_polling(in_port);
		lego_device_unregister(in_port->sensor);
		in_port->sensor = NULL;
		in_port->sensor_type = PS_SENSOR_TYPE_NONE;
		pistorms_set_sensor_type(in_port);
	}
	if (in_port->i2c_sensor) {
		i2c_unregister_device(in_port->i2c_sensor);
		in_port->i2c_sensor = NULL;
		in_port->sensor_type = PS_SENSOR_TYPE_NONE;
		pistorms_set_sensor_type(in_port);
	}
}

static int pistorms_in_port_set_device(void *context, const char *name)
{
	struct pistorms_in_port_data *in_port = context;
	int mode = in_port->port.mode;

	pistorms_in_port_unregister_sensor(in_port);

	if (mode == PS_IN_PORT_MODE_NONE)
		return -EOPNOTSUPP;
	else if (mode == PS_IN_PORT_MODE_I2C_THRU) {
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
		ret = sscanf(++blank, "%hhu%c%x", &in_port->i2c_sensor_addr,
				 &end, &hex);
		if (ret < 1) {
			dev_err(&in_port->port.dev, "%s: Can't parse I2C address\n",
				"set_device");
			return -EINVAL;
		}
		if (in_port->i2c_sensor_addr == 0 && end == 'x')
			in_port->i2c_sensor_addr = hex;
		name = i2c_name;
	}

	return pistorms_in_port_register_sensor(in_port,
				&pistorms_in_port_device_types[mode], name);
}

static int pistorms_in_port_set_pin5_gpio(void *context,
					  enum lego_port_gpio_state state)
{
	struct pistorms_in_port_data *in_port = context;

	switch (state) {
	case LEGO_PORT_GPIO_FLOAT:
	case LEGO_PORT_GPIO_LOW:
		in_port->sensor_type = PS_SENSOR_TYPE_NXT_ANALOG;
		break;
	case LEGO_PORT_GPIO_HIGH:
		in_port->sensor_type = PS_SENSOR_TYPE_NXT_ANALOG_PIN5;
		break;
	}

	return pistorms_set_sensor_type(in_port);
}

static int pistorms_in_port_set_mode(void *context, u8 mode)
{
	struct pistorms_in_port_data *in_port = context;
	const char *name = NULL;
	int err;

	pistorms_in_port_unregister_sensor(in_port);
	switch (mode) {
	case PS_IN_PORT_MODE_NONE:
		in_port->sensor_type = PS_SENSOR_TYPE_NONE;
		break;
	case PS_IN_PORT_MODE_NXT_ANALOG:
		in_port->sensor_type = PS_SENSOR_TYPE_NXT_ANALOG;
		name = GENERIC_NXT_ANALOG_SENSOR_NAME;
		break;
	case PS_IN_PORT_MODE_I2C_THRU:
		in_port->sensor_type = PS_SENSOR_TYPE_I2C_THRU;
		break;
	case PS_IN_PORT_MODE_EV3_ANALOG:
		in_port->sensor_type = PS_SENSOR_TYPE_EV3_TOUCH;
		name = LEGO_EV3_TOUCH_SENSOR_NAME;
		break;
	case PS_IN_PORT_MODE_EV3_UART:
		in_port->sensor_type = PS_SENSOR_TYPE_EV3_UART;
		/* name is set below */
		break;
	default:
		return -EINVAL;
	}

	err = pistorms_set_sensor_type(in_port);
	if (err < 0)
		return err;

	if (mode == PS_IN_PORT_MODE_I2C_THRU) {
		struct i2c_client dummy_client;
		struct i2c_board_info info;
		struct i2c_adapter *adap = in_port->client->adapter;
		const unsigned short *addr_list = nxt_i2c_sensor_driver.address_list;
		int i;

		memset(&dummy_client, 0, sizeof(struct i2c_client));
		memset(&info, 0, sizeof(struct i2c_board_info));
		dummy_client.adapter = adap;
		info.platform_data = &in_port->i2c_platform_data;

		for (i = 0; addr_list[i] != I2C_CLIENT_END; i++) {
			dummy_client.addr = addr_list[i];
			if (nxt_i2c_sensor_driver.detect(&dummy_client, &info) == 0) {
				info.addr = addr_list[i];
				in_port->i2c_sensor = i2c_new_device(adap, &info);
				if (in_port->i2c_sensor)
					return 0;
			}
		}
		return 0;
	}

	if (mode == PS_IN_PORT_MODE_EV3_UART) {
		u8 uart_id[PS_EV3_UART_ID_SIZE];
		int i;

		err = i2c_smbus_read_i2c_block_data(in_port->client,
				in_port->i2c_reg + PS_EV3_UART_ID_OFFSET,
				PS_EV3_UART_ID_SIZE, uart_id);
		if (err < 0)
			return 0; /* mode has been set already, so ignore err */


		for (i = 0; i < NUM_LEGO_EV3_SENSOR_TYPES; i++) {
			/* PiStorms always returns first mode as ID */
			if (!strncmp(uart_id,
			     ev3_uart_sensor_defs[i].mode_info[0].name,
			     PS_EV3_UART_ID_SIZE))
			{
				name = ev3_uart_sensor_defs[i].name;
				break;
			}
		}
	}

	if (!name)
		return 0;

	return pistorms_in_port_register_sensor(in_port,
				&pistorms_in_port_device_types[mode], name);
}

static struct lego_port_nxt_analog_ops pistorms_in_port_nxt_analog_ops = {
	.set_pin5_gpio = pistorms_in_port_set_pin5_gpio,
};


static struct lego_port_ev3_analog_ops pistorms_ev3_analog_ops = {
	.lego_touch_sensor_is_scaled = true,
};


static int pistorms_in_port_set_ev3_uart_sensor_mode(void *context, u8 type_id,
						     u8 mode)
{
	struct pistorms_in_port_data *in_port = context;

	return i2c_smbus_write_byte_data(in_port->client,
				in_port->i2c_reg + PS_EV3_UART_MODE_OFFSET,
				mode);
}

static const struct lego_port_ev3_uart_ops pistorms_ev3_uart_ops = {
	.set_mode = pistorms_in_port_set_ev3_uart_sensor_mode,
};

int pistorms_in_ports_register(struct pistorms_data *data)
{
	struct pistorms_in_port_data *ports;
	int i, err;

	ports = kzalloc(sizeof(struct pistorms_in_port_data) * NUM_PS_PORT,
			GFP_KERNEL);
	if (!ports)
		return -ENOMEM;

	data->in_port_data = ports;

	for (i = 0; i < NUM_PS_PORT; i++) {
		struct pistorms_in_port_data *in_port = &ports[i];

		in_port->client = data->client;
		in_port->sensor_type = PS_SENSOR_TYPE_NONE;
		in_port->poll_ms = PS_DEFAULT_POLL_MS;
		in_port->i2c_reg = (i == 0) ? PS_SENSOR_PORT_1_REG
					    : PS_SENSOR_PORT_2_REG;
		INIT_WORK(&in_port->poll_work, pistorms_poll_work);
		hrtimer_init(&in_port->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		in_port->poll_timer.function = pistorms_poll_timer_function;

		in_port->i2c_platform_data.in_port = &in_port->port;

		in_port->port.name = pistorms_in_port_type.name;
		snprintf(in_port->port.address, LEGO_NAME_SIZE, "%sS%d",
			 data->name, i + 1);
		in_port->port.num_modes = NUM_PS_IN_PORT_MODES;
		in_port->port.supported_modes = LEGO_PORT_ALL_MODES;
		in_port->port.mode_info = pistorms_in_port_mode_info;
		in_port->port.set_mode = pistorms_in_port_set_mode;
		in_port->port.set_device = pistorms_in_port_set_device;
		in_port->port.ev3_analog_ops = &pistorms_ev3_analog_ops;
		in_port->port.ev3_uart_ops = &pistorms_ev3_uart_ops;
		in_port->port.context = in_port;
		in_port->port.nxt_analog_ops = &pistorms_in_port_nxt_analog_ops;

		err = lego_port_register(&in_port->port, &pistorms_in_port_type,
					 &data->client->dev);
		if (err) {
			dev_err(&data->client->dev,
				"Failed to register PiStorms input port. (%d)\n",
				err);
			for (i--; i >= 0; i--)
				lego_port_unregister(&ports[i].port);
			data->in_port_data = NULL;
			return err;
		}

		pistorms_set_sensor_type(in_port);
	}

	return 0;
}

void pistorms_in_ports_unregister(struct pistorms_data *data)
{
	struct pistorms_in_port_data *ports = data->in_port_data;
	int i;

	for (i = 0; i < NUM_PS_PORT; i++) {
		struct pistorms_in_port_data *in_port = &ports[i];
		pistorms_in_port_unregister_sensor(in_port);
		lego_port_unregister(&in_port->port);
	}

	data->in_port_data = NULL;
	kfree(ports);
}
