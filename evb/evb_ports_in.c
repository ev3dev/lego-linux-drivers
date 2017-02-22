/*
 * EV3 Input port driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2013-2016 David Lechner <david@lechnology.com>
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
 * FatcatLab EVB has four input ports labeled 1, 2, 3 and 4. These ports
 * are used with LEGO MINDSTORMS EV3 and NXT compatible sensors and other
 * devices. Most sensors can be automatically detected when plugged in.
 * Unlike the EV3, however, NXT analog sensors cannot be detected, so to use
 * these sensors, the port has to be placed in ``nxt-analog`` mode manually.
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <lego.h>
#include <lego_port_class.h>

#include "evb_ports.h"
#include "../sensors/ev3_analog_sensor.h"
#include "../sensors/nxt_analog_sensor.h"

#ifndef I2C_CLASS_LEGO
#define I2C_CLASS_LEGO (1<<31)
#endif

#define INPUT_PORT_POLL_NS	10000000	/* 10 msec */
#define SETTLE_CNT		2		/* 20 msec */
#define ADD_CNT			35		/* 350 msec */
#define REMOVE_CNT		10		/* 100 msec */

#define PIN1_NEAR_5V		4900		/* 4.90V */
#define PIN1_NEAR_PIN2		3100		/* 3.1V */
#define PIN1_TOUCH_HIGH		950 		/* 0.95V */
#define PIN1_TOUCH_LOW		850		/* 0.85V */
#define PIN1_TOUCH_VAR		10		/* 0.01V */
#define PIN1_NEAR_GND		100		/* 0.1V */
#define PIN6_NEAR_GND		150		/* 0.15V */

#define PIN1_ID_01		206
#define PIN1_ID_02		417	/* EV3 touch sensor */
#define PIN1_ID_03		575
#define PIN1_ID_04		833
#define PIN1_ID_05		1063
#define PIN1_ID_06		1241
#define PIN1_ID_07		1403
#define PIN1_ID_08		1599
#define PIN1_ID_09		1795
#define PIN1_ID_10		2024
#define PIN1_ID_11		2204
#define PIN1_ID_12		2382
#define PIN1_ID_13		2619
#define PIN1_ID_14		2826	/* 3rd party */
#define PIN1_ID_VAR		50	/* IDs can be +/- 50mV */

enum sensor_type_id {
	SENSOR_TYPE_ID_NXT_TOUCH,
	SENSOR_TYPE_ID_NXT_LIGHT,
	SENSOR_TYPE_ID_NXT_ANALOG,
	SENSOR_TYPE_ID_NXT_COLOR,
	SENSOR_TYPE_ID_NXT_I2C,
	SENSOR_TYPE_ID_EV3_ANALOG_01,
	SENSOR_TYPE_ID_EV3_TOUCH,
	SENSOR_TYPE_ID_EV3_ANALOG_03,
	SENSOR_TYPE_ID_EV3_ANALOG_04,
	SENSOR_TYPE_ID_EV3_ANALOG_05,
	SENSOR_TYPE_ID_EV3_ANALOG_06,
	SENSOR_TYPE_ID_EV3_ANALOG_07,
	SENSOR_TYPE_ID_EV3_ANALOG_08,
	SENSOR_TYPE_ID_EV3_ANALOG_09,
	SENSOR_TYPE_ID_EV3_ANALOG_10,
	SENSOR_TYPE_ID_EV3_ANALOG_11,
	SENSOR_TYPE_ID_EV3_ANALOG_12,
	SENSOR_TYPE_ID_EV3_ANALOG_13,
	SENSOR_TYPE_ID_EV3_ANALOG_14,
	SENSOR_TYPE_ID_EV3_UART,
	SENSOR_TYPE_ID_UNKNOWN,
};

/* resistor ids for EV3 analog sensor devices */
enum ev3_analog_sensor_res_id {
	EV3_RESISTOR_ID_01,
	EV3_RESISTOR_ID_02,
	EV3_RESISTOR_ID_03,
	EV3_RESISTOR_ID_04,
	EV3_RESISTOR_ID_05,
	EV3_RESISTOR_ID_06,
	EV3_RESISTOR_ID_07,
	EV3_RESISTOR_ID_08,
	EV3_RESISTOR_ID_09,
	EV3_RESISTOR_ID_10,
	EV3_RESISTOR_ID_11,
	EV3_RESISTOR_ID_12,
	EV3_RESISTOR_ID_13,
	EV3_RESISTOR_ID_14,
	NUM_EV3_RESISTOR_ID,
};

struct ev3_analog_id_resistor_info {
	unsigned type_id;
	int min_mv;
	int max_mv;
};

static struct ev3_analog_id_resistor_info ev3_analog_id_resistor_infos[] = {
	[EV3_RESISTOR_ID_01] = {
		.type_id = SENSOR_TYPE_ID_EV3_ANALOG_01,
		.min_mv = PIN1_ID_01 - PIN1_ID_VAR,
		.max_mv = PIN1_ID_01 + PIN1_ID_VAR,
	},
	[EV3_RESISTOR_ID_02] = {
		.type_id = SENSOR_TYPE_ID_EV3_TOUCH,
		.min_mv = PIN1_ID_02 - PIN1_ID_VAR,
		.max_mv = PIN1_ID_02 + PIN1_ID_VAR,
	},
	[EV3_RESISTOR_ID_03] = {
		.type_id = SENSOR_TYPE_ID_EV3_ANALOG_03,
		.min_mv = PIN1_ID_03 - PIN1_ID_VAR,
		.max_mv = PIN1_ID_03 + PIN1_ID_VAR,
	},
	[EV3_RESISTOR_ID_04] = {
		.type_id = SENSOR_TYPE_ID_EV3_ANALOG_04,
		.min_mv = PIN1_ID_04 - PIN1_ID_VAR,
		.max_mv = PIN1_ID_04 + PIN1_ID_VAR,
	},
	[EV3_RESISTOR_ID_05] = {
		.type_id = SENSOR_TYPE_ID_EV3_ANALOG_05,
		.min_mv = PIN1_ID_05 - PIN1_ID_VAR,
		.max_mv = PIN1_ID_05 + PIN1_ID_VAR,
	},
	[EV3_RESISTOR_ID_06] = {
		.type_id = SENSOR_TYPE_ID_EV3_ANALOG_06,
		.min_mv = PIN1_ID_06 - PIN1_ID_VAR,
		.max_mv = PIN1_ID_06 + PIN1_ID_VAR,
	},
	[EV3_RESISTOR_ID_07] = {
		.type_id = SENSOR_TYPE_ID_EV3_ANALOG_07,
		.min_mv = PIN1_ID_07 - PIN1_ID_VAR,
		.max_mv = PIN1_ID_07 + PIN1_ID_VAR,
	},
	[EV3_RESISTOR_ID_08] = {
		.type_id = SENSOR_TYPE_ID_EV3_ANALOG_08,
		.min_mv = PIN1_ID_08 - PIN1_ID_VAR,
		.max_mv = PIN1_ID_08 + PIN1_ID_VAR,
	},
	[EV3_RESISTOR_ID_09] = {
		.type_id = SENSOR_TYPE_ID_EV3_ANALOG_09,
		.min_mv = PIN1_ID_09 - PIN1_ID_VAR,
		.max_mv = PIN1_ID_09 + PIN1_ID_VAR,
	},
	[EV3_RESISTOR_ID_10] = {
		.type_id = SENSOR_TYPE_ID_EV3_ANALOG_10,
		.min_mv = PIN1_ID_10 - PIN1_ID_VAR,
		.max_mv = PIN1_ID_10 + PIN1_ID_VAR,
	},
	[EV3_RESISTOR_ID_11] = {
		.type_id = SENSOR_TYPE_ID_EV3_ANALOG_11,
		.min_mv = PIN1_ID_11 - PIN1_ID_VAR,
		.max_mv = PIN1_ID_11 + PIN1_ID_VAR,
	},
	[EV3_RESISTOR_ID_12] = {
		.type_id = SENSOR_TYPE_ID_EV3_ANALOG_12,
		.min_mv = PIN1_ID_12 - PIN1_ID_VAR,
		.max_mv = PIN1_ID_12 + PIN1_ID_VAR,
	},
	[EV3_RESISTOR_ID_13] = {
		.type_id = SENSOR_TYPE_ID_EV3_ANALOG_13,
		.min_mv = PIN1_ID_13 - PIN1_ID_VAR,
		.max_mv = PIN1_ID_13 + PIN1_ID_VAR,
	},
	[EV3_RESISTOR_ID_14] = {
		.type_id = SENSOR_TYPE_ID_EV3_ANALOG_14,
		.min_mv = PIN1_ID_14 - PIN1_ID_VAR,
		.max_mv = PIN1_ID_14 + PIN1_ID_VAR,
	},
};

/**
 * to_ev3_analog_sensor_type_id - converts id resistor mV value to a Type ID
 * @mv: The value to convert.
 */
unsigned to_ev3_analog_sensor_type_id(int mv)
{
	enum ev3_analog_sensor_res_id res_id = NUM_EV3_RESISTOR_ID;

	while (res_id--) {
		if (mv >= ev3_analog_id_resistor_infos[res_id].min_mv
		    && mv <= ev3_analog_id_resistor_infos[res_id].max_mv)
			return ev3_analog_id_resistor_infos[res_id].type_id;
	}

	return SENSOR_TYPE_ID_UNKNOWN;
}

enum connection_state {
	CON_STATE_INIT,			/* Wait for sensor to unregister, then
						Set port to "float" state */
	CON_STATE_INIT_SETTLE,		/* Wait for port to settle */
	CON_STATE_NO_DEV,		/* No device present, wait until something
						interesting happens on one or more
						of the pins and a steady state is
						reached */
	CON_STATE_TEST_NXT_TOUCH,	/* We might have a NXT touch sensor that
						is pressed. We need to watch
						that pin 1 voltage doesn't change
						to be sure */
	CON_STATE_HAVE_NXT,		/* Wait for pin 2 to float */
	CON_STATE_HAVE_EV3,		/* Wait for pin 1 to float */
	CON_STATE_HAVE_I2C,		/* Wait for pin 6 to float */
	CON_STATE_HAVE_PIN5_ERR,	/* Wait for pin 5 to float */
	NUM_CON_STATE
};

enum pin_state_flag {
	PIN_STATE_FLAG_PIN2_LOW,
	PIN_STATE_FLAG_PIN1_LOADED,
	PIN_STATE_FLAG_PIN5_LOW,
	PIN_STATE_FLAG_PIN6_HIGH,
	NUM_PIN_STATE_FLAG
};

/* These states are used for auto modes */
enum sensor_type {
	SENSOR_NONE,
	SENSOR_NXT_ANALOG,
	SENSOR_NXT_COLOR,
	SENSOR_NXT_I2C,
	SENSOR_EV3_ANALOG,
	SENSOR_EV3_UART,
	SENSOR_ERR,
	NUM_SENSOR_TYPE
};

static const char * const evb_input_port_state_names[] = {
	[SENSOR_NONE]		= "no-sensor",
	[SENSOR_NXT_ANALOG]	= "nxt-analog",
	[SENSOR_NXT_COLOR]	= "nxt-color",
	[SENSOR_NXT_I2C]	= "nxt-i2c",
	[SENSOR_EV3_ANALOG]	= "ev3-analog",
	[SENSOR_EV3_UART]	= "ev3-uart",
	[SENSOR_ERR]		= "error",
};

enum evb_input_port_mode {
	EV3_INPUT_PORT_MODE_AUTO,
	EV3_INPUT_PORT_MODE_NXT_ANALOG,
	EV3_INPUT_PORT_MODE_NXT_COLOR,
	EV3_INPUT_PORT_MODE_NXT_I2C,
	EV3_INPUT_PORT_MODE_OTHER_I2C,
	EV3_INPUT_PORT_MODE_EV3_ANALOG,
	EV3_INPUT_PORT_MODE_EV3_UART,
	EV3_INPUT_PORT_MODE_OTHER_UART,
	EV3_INPUT_PORT_MODE_RAW,
	NUM_EV3_INPUT_PORT_MODE
};

struct evb_input_port_mode_name {
	enum evb_input_port_mode mode;
	const char *name;
};

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new modes have the same syntax. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the evbdev-kpkg repository.
 */

static const struct lego_port_mode_info evb_input_port_mode_info[] = {
	/**
	 * @description: EV3 Input Port
	 * @module: evb-ports
	 * @connection_types: NXT/Analog, NXT/I2C, Other/I2C, EV3/Analog, EV3/UART, Other/UART
	 * @prefix: in
	 * @module: evb_ports
	 */
	[EV3_INPUT_PORT_MODE_AUTO] = {
		/**
		 * .. [#in-port-auto-mode] In `auto` mode, the port will attempt
		 *    to automatically detect the type of sensor that was
		 *    connected and load the appropriate driver. See the list of
		 *    :ref:`supported-sensors` to determine if a sensor can be
		 *    automatically detected.
		 *
		 * @description: Automatically detect most sensors.
		 * @name_footnote: [#in-port-auto-mode]_
		 */
		.name	= "auto",
	},
	[EV3_INPUT_PORT_MODE_NXT_ANALOG] = {
		/**
		 * .. [#in-port-nxt-analog-mode] This loads the generic
		 *    NXT/Analog sensor ``nxt-analog`` driver. Use
		 *    ``set_device`` to load the appropriate device/driver.
		 *
		 * @description: Configures the port for NXT/Analog sensors
		 * @name_footnote: [#in-port-nxt-analog-mode]_
		 */
		.name	= "nxt-analog",
	},
	[EV3_INPUT_PORT_MODE_NXT_COLOR] = {
		/**
		 * .. [#in-port-nxt-color-mode] NXT Color sensor driver has not
		 *    been implemented yet, so right now, this mode does nothing.
		 *
		 * @description: Configures the port for the LEGO NXT Color sensor
		 * @name_footnote: [#in-port-nxt-color-mode]_
		 */
		.name=	"nxt-color",
	},
	[EV3_INPUT_PORT_MODE_NXT_I2C] ={
		/**
		 * @description: Configure for I2C communications and load the ``nxt-i2c-host`` device.
		 */
		.name	= "nxt-i2c",
	},
	[EV3_INPUT_PORT_MODE_OTHER_I2C] = {
		/**
		 * @description: Configure for I2C communications but do not probe for NXT sensors.
		 */
		.name	= "other-i2c",
	},
	[EV3_INPUT_PORT_MODE_EV3_ANALOG] = {
		/**
		 * @description: Configures the port for EV3/Analog sensors
		 */
		.name	= "ev3-analog",
	},
	[EV3_INPUT_PORT_MODE_EV3_UART] = {
		/**
		 * @description: Configure for UART communications and load the ``ev3-uart-host`` device.
		 */
		.name	= "ev3-uart",
	},
	[EV3_INPUT_PORT_MODE_OTHER_UART] = {
		/**
		 * @description: Configure for UART communications but do not load any device.
		 */
		.name	= "other-uart",
	},
	[EV3_INPUT_PORT_MODE_RAW] = {
		/**
		 * .. [#in-port-raw-mode] Exports gpios and analog/digital
		 *    converter values to sysfs so that they can be controlled
		 *    directly.
		 *
		 * @description: Provide access to low level drivers.
		 * @name_footnote: [#in-port-raw-mode]_
		 */
		.name	= "raw",
	},
};

struct device_type evb_input_port_sensor_types[] = {
	[SENSOR_NXT_ANALOG] = {
		.name	= "nxt-analog-sensor",
	},
	[SENSOR_NXT_COLOR] = {
		.name	= "nxt-color-sensor",
	},
	[SENSOR_NXT_I2C] = {
		.name	= "nxt-i2c-host",
	},
	[SENSOR_EV3_ANALOG] = {
		.name	= "ev3-analog-sensor",
	},
	[SENSOR_EV3_UART] = {
		.name	= "ev3-uart-host",
	},
};

const char *evb_input_port_sensor_table[] = {
	[SENSOR_TYPE_ID_NXT_TOUCH]	= LEGO_NXT_TOUCH_SENSOR_NAME,
	[SENSOR_TYPE_ID_NXT_LIGHT]	= LEGO_NXT_LIGHT_SENSOR_NAME,
	[SENSOR_TYPE_ID_NXT_ANALOG]	= GENERIC_NXT_ANALOG_SENSOR_NAME,
	[SENSOR_TYPE_ID_NXT_COLOR]	= "lego-nxt-color",
	[SENSOR_TYPE_ID_NXT_I2C]	= "nxt-i2c-host",
	[SENSOR_TYPE_ID_EV3_ANALOG_01]	= EV3_ANALOG_SENSOR_ID_01_NAME,
	[SENSOR_TYPE_ID_EV3_TOUCH]	= LEGO_EV3_TOUCH_SENSOR_NAME,
	[SENSOR_TYPE_ID_EV3_ANALOG_03]	= EV3_ANALOG_SENSOR_ID_03_NAME,
	[SENSOR_TYPE_ID_EV3_ANALOG_04]	= EV3_ANALOG_SENSOR_ID_04_NAME,
	[SENSOR_TYPE_ID_EV3_ANALOG_05]	= EV3_ANALOG_SENSOR_ID_05_NAME,
	[SENSOR_TYPE_ID_EV3_ANALOG_06]	= EV3_ANALOG_SENSOR_ID_06_NAME,
	[SENSOR_TYPE_ID_EV3_ANALOG_07]	= EV3_ANALOG_SENSOR_ID_07_NAME,
	[SENSOR_TYPE_ID_EV3_ANALOG_08]	= EV3_ANALOG_SENSOR_ID_08_NAME,
	[SENSOR_TYPE_ID_EV3_ANALOG_09]	= EV3_ANALOG_SENSOR_ID_09_NAME,
	[SENSOR_TYPE_ID_EV3_ANALOG_10]	= EV3_ANALOG_SENSOR_ID_10_NAME,
	[SENSOR_TYPE_ID_EV3_ANALOG_11]	= EV3_ANALOG_SENSOR_ID_11_NAME,
	[SENSOR_TYPE_ID_EV3_ANALOG_12]	= EV3_ANALOG_SENSOR_ID_12_NAME,
	[SENSOR_TYPE_ID_EV3_ANALOG_13]	= EV3_ANALOG_SENSOR_ID_13_NAME,
	[SENSOR_TYPE_ID_EV3_ANALOG_14]	= EV3_ANALOG_SENSOR_ID_14_NAME,
	[SENSOR_TYPE_ID_EV3_UART]	= "ev3-uart-host",
	[SENSOR_TYPE_ID_UNKNOWN]	= NULL,
};

static struct device_type evb_input_port_type = {
	.name	= "evb-input-port",
};

/**
 * struct evb_input_port_data - Driver data for an input port on the EV3 brick
 * @port: Pointer to the evb_port that is bound to this instance.
 * @iio_cb: IIO callback buffer for analog inputs.
 * @pin1_gpio: Controls 9V supply on Pin 1.
 * @pin2_gpio: Attached to pin 2. Discriminates EV3 sensors from NXT sensors.
 * @pin5_gpio: Sensor I/O on pin 5.
 * @pin6_gpio: Sensor I/O on pin 6.
 * @buf_ena_gpio: Enables buffer for UART.
 * @i2c_data: Platform data for i2c-gpio platform device.
 * @change_uevent_work: Needed when change is triggered in atomic context.
 * @work: Worker for registering and unregistering sensors when they are
 *	connected and disconnected.
 * @timer: Polling timer to monitor the port.
 * @timer_loop_cnt: Used to measure time in the polling loop.
 * @con_state: The current state of the port.
 * @pin_state_flags: Used in the polling loop to track certain changes in the
 *	state of the port's pins.
 * @pin1_index: Index of analog input pin 1 data in callback buffer.
 * @pin1_mv: Cached value for analog input on pin 1.
 * @pin6_index: Index of analog input pin 1 data in callback buffer.
 * @pin6_mv: Cached value for analog input on pin 6.
 * @prev_pin1_mv: Used in the polling loop to track changes in pin 1 voltage.
 * @sensor_type: The type of sensor currently connected.
 * @sensor_type_id: The sensor type id for EV3 sensors or -1 for NXT sensors.
 * @sensor: The sensor connected to the port
 * @i2c_adap: The i2c adapter associated with this port.
 * @pinctrl: The pinctrl for muxing pins on this port.
 * @pinctrl_default: The default pinctrl state.
 * @pinctrl_i2c: The pinctrl state for i2c communications.
 * @pinctrl_uart: The pinctrl state for uart communications.
 * @i2c_enabled: Flag indicating if port has i2c enabled.
 * @uart_enabled: Flag indicating that port has uart enabled.
 */
struct evb_input_port_data {
	struct lego_port_device port;
	struct evb_analog_device *analog;
	struct iio_cb_buffer *iio_cb;
	void (*sensor_cb)(struct evb_input_port_data *data);
	struct gpio_desc *pin1_gpio;
	struct gpio_desc *pin2_gpio;
	struct gpio_desc *pin5_gpio;
	struct gpio_desc *pin6_gpio;
	struct gpio_desc *buf_ena_gpio;
	struct work_struct change_uevent_work;
	struct work_struct work;
	struct hrtimer timer;
	unsigned timer_loop_cnt;
	enum connection_state con_state;
	unsigned pin_state_flags:NUM_PIN_STATE_FLAG;
	unsigned pin1_index;
	unsigned pin1_mv;
	unsigned pin6_index;
	unsigned pin6_mv;
	unsigned prev_pin1_mv;
	enum sensor_type sensor_type;
	enum sensor_type_id sensor_type_id;
	struct lego_device *sensor;
	struct i2c_adapter *i2c_adap;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_default;
	struct pinctrl_state *pinctrl_i2c;
	struct pinctrl_state *pinctrl_uart;
	unsigned i2c_enabled:1;
	unsigned uart_enabled:1;
};

static inline int evb_input_port_set_gpio(struct gpio_desc *gpio,
					  enum lego_port_gpio_state state)
{
	if (state == LEGO_PORT_GPIO_FLOAT)
		return gpiod_direction_input(gpio);

	return gpiod_direction_output(gpio, state == LEGO_PORT_GPIO_HIGH);
}

static int evb_input_port_set_pin1_gpio(void *context,
					enum lego_port_gpio_state state)
{
	struct evb_input_port_data *data = context;

	return evb_input_port_set_gpio(data->pin1_gpio, state);
}

static struct lego_port_nxt_i2c_ops evb_input_port_nxt_i2c_ops = {
	.set_pin1_gpio	= evb_input_port_set_pin1_gpio,
};

static int evb_input_port_set_pin5_gpio(void *context,
					enum lego_port_gpio_state state)
{
	struct evb_input_port_data *data = context;

	return evb_input_port_set_gpio(data->pin5_gpio, state);
}

static struct lego_port_nxt_analog_ops evb_input_port_nxt_analog_ops = {
	.set_pin5_gpio	= evb_input_port_set_pin5_gpio,
};

static void evb_input_port_nxt_analog_cb(struct evb_input_port_data *data)
{
	if (data->port.raw_data)
		*(s32 *)data->port.raw_data = data->pin1_mv;
	if (data->port.notify_raw_data_func)
		data->port.notify_raw_data_func(data->port.notify_raw_data_context);
}

static void evb_input_port_ev3_analog_cb(struct evb_input_port_data *data)
{
	if (data->port.raw_data)
		*(s32 *)data->port.raw_data = data->pin6_mv;
	if (data->port.notify_raw_data_func)
		data->port.notify_raw_data_func(data->port.notify_raw_data_context);
}

int evb_input_port_enable_i2c(struct evb_input_port_data *data, unsigned int class)
{
	int err;

	err = pinctrl_select_state(data->pinctrl, data->pinctrl_i2c);
	if (err)
		return err;

	gpiod_set_value(data->buf_ena_gpio, 1);

	data->i2c_adap->class = class;
	i2c_adapter_probe(data->i2c_adap);

	data->i2c_enabled = true;

	return 0;
}

void evb_input_port_disable_i2c(struct evb_input_port_data *data)
{
	/* NOP if evb_input_port_enable_i2c() was not called */
	if (!data->i2c_enabled)
		return;

	data->i2c_enabled = false;

	i2c_adapter_remove_probed(data->i2c_adap);
	pinctrl_select_state(data->pinctrl, data->pinctrl_default);
	gpiod_set_value(data->buf_ena_gpio, 0);
}

int evb_input_port_enable_uart(struct evb_input_port_data *data)
{
	int err;

	err = pinctrl_select_state(data->pinctrl, data->pinctrl_uart);
	if (err)
		return err;

	gpiod_set_value(data->buf_ena_gpio, 1);

	data->uart_enabled = true;

	return 0;
}

void evb_input_port_disable_uart(struct evb_input_port_data *data)
{
	/* NOP if evb_input_port_enable_uart() was not called */
	if (!data->uart_enabled)
		return;

	data->uart_enabled = false;

	pinctrl_select_state(data->pinctrl, data->pinctrl_default);
	gpiod_set_value(data->buf_ena_gpio, 0);
}

void evb_input_port_float(struct evb_input_port_data *data)
{
	if (data->pin1_gpio)
		gpiod_direction_output(data->pin1_gpio, 0);
	if (data->pin2_gpio)
		gpiod_direction_input(data->pin2_gpio);
	gpiod_direction_input(data->pin5_gpio);
	gpiod_direction_input(data->pin6_gpio);
	gpiod_direction_output(data->buf_ena_gpio, 0);
}

void evb_input_port_register_sensor(struct work_struct *work)
{
	struct evb_input_port_data *data =
			container_of(work, struct evb_input_port_data, work);
	struct lego_device *new_sensor;

	if (data->sensor_type == SENSOR_NONE
	    || data->sensor_type == SENSOR_ERR
	    || data->sensor_type >= NUM_SENSOR_TYPE)
	{
		dev_err(&data->port.dev,
			"Trying to register an invalid sensor on %s.\n",
			dev_name(&data->port.dev));
		return;
	}

	switch(data->sensor_type) {
	case SENSOR_NXT_ANALOG:
		data->sensor_cb = evb_input_port_nxt_analog_cb;
		break;
	case SENSOR_EV3_ANALOG:
		data->sensor_cb = evb_input_port_ev3_analog_cb;
		break;
	case SENSOR_NXT_I2C:
		/* Give the sensor time to boot */
		msleep(1000);
		evb_input_port_enable_i2c(data, I2C_CLASS_LEGO);
		/*
		 * I2C sensors are handled by the i2c stack, so we are just
		 * kicking the i2c adapter so that it doesn't break the
		 * automatic detection.
		 */
		break;
	case SENSOR_EV3_UART:
		evb_input_port_enable_uart(data);
		/*
		 * UART sensors are handled via the EV3 UART Line Discipline
		 * however, we still register a dummy "sensor" for use by udev
		 * so that it can call ldattach.
		 */
		break;
	default:
		/* prevent compiler warning by having default case */
		break;
	}

	new_sensor = lego_device_register(
		evb_input_port_sensor_table[data->sensor_type_id],
		&evb_input_port_sensor_types[data->sensor_type],
		&data->port, NULL, 0);
	if (IS_ERR(new_sensor)) {
		dev_err(&data->port.dev,
			"Could not register sensor on port %s. (%ld)\n",
			dev_name(&data->port.dev), PTR_ERR(new_sensor));
		return;
	}

	data->sensor = new_sensor;

	return;
}

void evb_input_port_unregister_sensor(struct work_struct *work)
{
	struct evb_input_port_data *data =
			container_of(work, struct evb_input_port_data, work);

	lego_device_unregister(data->sensor);
	data->sensor_type = SENSOR_NONE;
	data->sensor = NULL;
	data->sensor_cb = NULL;
	evb_input_port_disable_i2c(data);
	evb_input_port_disable_uart(data);
}

void evb_input_port_change_uevent_work(struct work_struct *work)
{
	struct evb_input_port_data *data =
		container_of(work, struct evb_input_port_data, change_uevent_work);

	kobject_uevent(&data->port.dev.kobj, KOBJ_CHANGE);
}

static enum hrtimer_restart evb_input_port_timer_callback(struct hrtimer *timer)
{
	struct evb_input_port_data *data =
			container_of(timer, struct evb_input_port_data, timer);
	enum sensor_type prev_sensor_type = data->sensor_type;
	unsigned new_pin_state_flags = 0;
	unsigned new_pin1_mv = 0;

	hrtimer_forward_now(timer, ktime_set(0, INPUT_PORT_POLL_NS));
	data->timer_loop_cnt++;

	switch(data->con_state) {
	case CON_STATE_INIT:
		if (!data->sensor) {
			evb_input_port_float(data);
			data->timer_loop_cnt = 0;
			data->sensor_type = SENSOR_NONE;
			data->sensor_type_id = SENSOR_TYPE_ID_UNKNOWN;
			data->con_state = CON_STATE_INIT_SETTLE;
		}
		break;
	case CON_STATE_INIT_SETTLE:
		if (data->timer_loop_cnt >= SETTLE_CNT) {
			data->timer_loop_cnt = 0;
			data->con_state = CON_STATE_NO_DEV;
		}
		break;
	case CON_STATE_NO_DEV:
		new_pin1_mv = data->pin1_mv;
		/*
		 * Pin 2 .
		 *
		 * Unfortunately, the FatcatLab EVB omitted the gpio for
		 * pin 2, so it cannot detect some NXT sensors.
		 */
		if (data->pin2_gpio && !gpiod_get_value(data->pin2_gpio))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN2_LOW);
		/* Normally, pin 1 is pulled up to 5V. */
		if (new_pin1_mv < PIN1_NEAR_5V)
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN1_LOADED);
		/* Pin 5 has pull up resistor */
		if (!gpiod_get_value(data->pin5_gpio))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN5_LOW);
		/* Pin 6 has pull down resistor */
		if (gpiod_get_value(data->pin6_gpio))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN6_HIGH);
		/*
		 * If the pin state changes, keep looping until we have a
		 * steady state.
		 */
		if (new_pin_state_flags != data->pin_state_flags)
			data->timer_loop_cnt = 0;
		else if (new_pin_state_flags && data->timer_loop_cnt >= ADD_CNT
			 && !work_busy(&data->work))
		{
			/*
			 * We have a steady state and at least one pin is different
			 * from the floating state. Time to figure out what it is.
			 */
			if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN2_LOW)) {
				/*
				 * NXT Sensors (except for early model NXT Touch sensors
				 * and possibly some 3rd party NXT sensors) have pin 2
				 * connected to pin 3 (GND) internally. So, if pin 2 is
				 * low, we know this is a NXT sensor.
				 */
				data->con_state = CON_STATE_HAVE_NXT;
				if ((~new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN5_LOW))
				    && (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN6_HIGH))) {
					if (new_pin1_mv < PIN1_NEAR_GND) {
						data->sensor_type = SENSOR_NXT_COLOR;
						data->sensor_type_id = SENSOR_TYPE_ID_NXT_COLOR;
					} else {
						data->sensor_type = SENSOR_NXT_I2C;
						data->sensor_type_id = SENSOR_TYPE_ID_NXT_I2C;
					}
				} else if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN5_LOW)) {
					data->sensor_type = SENSOR_NXT_ANALOG;
					if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN6_HIGH))
						data->sensor_type_id = SENSOR_TYPE_ID_NXT_ANALOG;
					else
						data->sensor_type_id = SENSOR_TYPE_ID_NXT_LIGHT;
				} else if (new_pin1_mv < PIN1_NEAR_GND) {
					data->sensor_type = SENSOR_NXT_COLOR;
					data->sensor_type_id = SENSOR_TYPE_ID_NXT_COLOR;
				} else if (new_pin1_mv > PIN1_NEAR_5V) {
					data->sensor_type = SENSOR_NXT_ANALOG;
					data->sensor_type_id = SENSOR_TYPE_ID_NXT_TOUCH;
				} else if (new_pin1_mv > PIN1_TOUCH_LOW
					 && new_pin1_mv < PIN1_TOUCH_HIGH) {
					data->con_state = CON_STATE_TEST_NXT_TOUCH;
					data->timer_loop_cnt = 0;
					data->prev_pin1_mv = new_pin1_mv;
				} else {
					data->sensor_type = SENSOR_NXT_ANALOG;
					data->sensor_type_id = SENSOR_TYPE_ID_NXT_ANALOG;
				}
			} else if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN1_LOADED)) {
				/*
				 * EV3 sensors use pin 1 to indicate the type of sensor.
				 * EV3/Analog sensors use an ID resistor to indicate
				 * the sensor that is attached. EV3/UART sensors pull
				 * pin 1 to ground.
				 */
				data->con_state = CON_STATE_HAVE_EV3;
				if (new_pin1_mv > PIN1_NEAR_PIN2) {
					/*
					 * Attaching a motor to an input port will
					 * short pin 1 to pin 2 through the motor
					 * windings.
					 */
					data->sensor_type = SENSOR_ERR;
				} else if (new_pin1_mv < PIN1_NEAR_GND) {
					data->sensor_type = SENSOR_EV3_UART;
					data->sensor_type_id = SENSOR_TYPE_ID_EV3_UART;
				} else {
					data->sensor_type = SENSOR_EV3_ANALOG;
					data->sensor_type_id = to_ev3_analog_sensor_type_id(new_pin1_mv);
					if (data->sensor_type_id == SENSOR_TYPE_ID_UNKNOWN)
						data->sensor_type = SENSOR_ERR;
				}
			} else if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN6_HIGH)) {
				/*
				 * This will catch 3rd party NXT/I2C sensors that don't
				 * have pin 2 tied to ground.
				 */
				data->con_state = CON_STATE_HAVE_I2C;
				data->sensor_type = SENSOR_NXT_I2C;
				data->sensor_type_id = SENSOR_TYPE_ID_NXT_I2C;
			} else {
				/*
				 * Something is pulling pin 5 low.
				 */
				data->con_state = CON_STATE_HAVE_PIN5_ERR;
				data->sensor_type = SENSOR_ERR;
				data->sensor_type_id = SENSOR_TYPE_ID_UNKNOWN;
			}
			data->timer_loop_cnt = 0;
			if (data->sensor_type != SENSOR_ERR) {
				INIT_WORK(&data->work, evb_input_port_register_sensor);
				schedule_work(&data->work);
			}
		}
		data->pin_state_flags = new_pin_state_flags;
		break;
	case CON_STATE_TEST_NXT_TOUCH:
		if (data->timer_loop_cnt >= SETTLE_CNT) {
			data->con_state = CON_STATE_HAVE_NXT;
			data->sensor_type = SENSOR_NXT_ANALOG;
			new_pin1_mv = data->pin1_mv;
			if (new_pin1_mv > (data->prev_pin1_mv - PIN1_TOUCH_VAR) &&
			    new_pin1_mv < (data->prev_pin1_mv + PIN1_TOUCH_VAR))
				data->sensor_type_id = SENSOR_TYPE_ID_NXT_TOUCH;
			else
				data->sensor_type_id = SENSOR_TYPE_ID_NXT_ANALOG;
		}
		break;
	case CON_STATE_HAVE_NXT:
		if (data->pin2_gpio && !gpiod_get_value(data->pin2_gpio))
		{
			data->timer_loop_cnt = 0;
		}
		break;
	case CON_STATE_HAVE_EV3:
		new_pin1_mv = data->pin1_mv;
		if (new_pin1_mv < PIN1_NEAR_5V)
			data->timer_loop_cnt = 0;
		break;
	case CON_STATE_HAVE_I2C:
		if (gpiod_get_value(data->pin6_gpio))
			data->timer_loop_cnt = 0;
		break;
	case CON_STATE_HAVE_PIN5_ERR:
		if (!gpiod_get_value(data->pin5_gpio))
			data->timer_loop_cnt = 0;
		break;
	default:
		data->con_state = CON_STATE_INIT;
		break;
	}
	/*
	 * data->sensor_type is used for lego-port class status, so we need to
	 * trigger a uevent when it changes.
	 */
	if (prev_sensor_type != data->sensor_type)
		schedule_work(&data->change_uevent_work);

	if (data->sensor_type != SENSOR_NONE
	    && data->timer_loop_cnt >= REMOVE_CNT && !work_busy(&data->work))
	{
		if (data->sensor) {
			INIT_WORK(&data->work, evb_input_port_unregister_sensor);
			schedule_work(&data->work);
		}
		data->con_state = CON_STATE_INIT;
	}

	return HRTIMER_RESTART;
}

int evb_input_port_enable_raw_mode(struct evb_input_port_data *data)
{
	if (data->pin1_gpio)
		gpiod_export(data->pin1_gpio, false);
	if (data->pin2_gpio)
		gpiod_export(data->pin2_gpio, false);
	gpiod_export(data->pin5_gpio, true);
	gpiod_export(data->pin6_gpio, true);
	/* TODO: export analog inputs as iio */

	return 0;
}

void evb_input_port_disable_raw_mode(struct evb_input_port_data *data)
{
	if (data->pin1_gpio)
		gpiod_unexport(data->pin1_gpio);
	if (data->pin2_gpio)
		gpiod_unexport(data->pin2_gpio);
	gpiod_unexport(data->pin5_gpio);
	gpiod_unexport(data->pin6_gpio);

	evb_input_port_float(data);
}

static int evb_input_port_set_mode(void *context, u8 mode)
{
	struct evb_input_port_data *data = context;
	int err;

	/*
	 * TODO: might be nice to check if we are in auto mode and see if the
	 * current loaded mode matches the new mode. In this case, it would not
	 * be necessary to unload and reload the same sensor.
	 */

	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->work);

	if (data->port.mode == EV3_INPUT_PORT_MODE_OTHER_I2C)
		evb_input_port_disable_i2c(data);
	if (data->port.mode == EV3_INPUT_PORT_MODE_OTHER_UART)
		evb_input_port_disable_uart(data);
	if (data->port.mode == EV3_INPUT_PORT_MODE_RAW)
		evb_input_port_disable_raw_mode(data);
	if (data->sensor)
		evb_input_port_unregister_sensor(&data->work);

	switch (mode) {
	case EV3_INPUT_PORT_MODE_AUTO:
		data->con_state = CON_STATE_INIT;
		hrtimer_start(&data->timer, ktime_set(0, INPUT_PORT_POLL_NS),
			      HRTIMER_MODE_REL);
		break;
	case EV3_INPUT_PORT_MODE_NXT_ANALOG:
		data->sensor_type = SENSOR_NXT_ANALOG;
		data->sensor_type_id = SENSOR_TYPE_ID_NXT_ANALOG;
		evb_input_port_register_sensor(&data->work);
		break;
	case EV3_INPUT_PORT_MODE_NXT_COLOR:
		data->sensor_type = SENSOR_NXT_COLOR;
		data->sensor_type_id = SENSOR_TYPE_ID_NXT_COLOR;
		evb_input_port_register_sensor(&data->work);
		break;
	case EV3_INPUT_PORT_MODE_NXT_I2C:
		data->sensor_type = SENSOR_NXT_I2C;
		data->sensor_type_id = SENSOR_TYPE_ID_NXT_I2C;
		evb_input_port_register_sensor(&data->work);
		break;
	case EV3_INPUT_PORT_MODE_OTHER_I2C:
		data->sensor_type = SENSOR_NONE;
		data->sensor_type_id = SENSOR_TYPE_ID_UNKNOWN;
		evb_input_port_enable_i2c(data, 0);
		break;
	case EV3_INPUT_PORT_MODE_EV3_ANALOG:
		data->sensor_type = SENSOR_EV3_ANALOG;
		data->sensor_type_id = SENSOR_TYPE_ID_EV3_ANALOG_01;
		evb_input_port_register_sensor(&data->work);
		break;
	case EV3_INPUT_PORT_MODE_EV3_UART:
		data->sensor_type = SENSOR_EV3_UART;
		data->sensor_type_id = SENSOR_TYPE_ID_EV3_UART;
		evb_input_port_register_sensor(&data->work);
		break;
	case EV3_INPUT_PORT_MODE_OTHER_UART:
		data->sensor_type = SENSOR_NONE;
		data->sensor_type_id = SENSOR_TYPE_ID_UNKNOWN;
		err = evb_input_port_enable_uart(data);
		if (err < 0)
			return err;
		break;
	case EV3_INPUT_PORT_MODE_RAW:
		data->sensor_type = SENSOR_NONE;
		data->sensor_type_id = SENSOR_TYPE_ID_UNKNOWN;
		err = evb_input_port_enable_raw_mode(data);
		if (err < 0)
			return err;
		break;
	default:
		/* should never get here, but keeps compiler happy */
		break;
	}

	return 0;
}

static int evb_input_port_set_device(void *context, const char *device_name)
{
	struct evb_input_port_data *data = context;
	struct lego_device *new_sensor;

	if (data->sensor_type != SENSOR_NXT_ANALOG)
		return -EOPNOTSUPP;

	lego_device_unregister(data->sensor);
	data->sensor = NULL;

	new_sensor = lego_device_register(device_name,
		&evb_input_port_sensor_types[data->sensor_type],
		&data->port, NULL, 0);
	if (IS_ERR(new_sensor))
		return PTR_ERR(new_sensor);

	data->sensor = new_sensor;

	return 0;
}

static const char *evb_input_port_get_status(void *context)
{
	struct evb_input_port_data *data = context;

	return evb_input_port_state_names[data->sensor_type];
}

static int evb_input_port_buf_cb(const void *buf_data, void *private)
{
	struct evb_input_port_data *data = private;
	const u16 *raw = buf_data;

	/*
	 * Making some assumptions about the data format here. This works for
	 * the ti-ads7957 driver, but won't work for just any old iio channels.
	 * To do this properly, we should be reading the iio_channel structs
	 * to determine how to properly decode the data.
	 */
	data->pin1_mv = ((raw[data->pin1_index] & 0xFFF) * 5002) >> 12;
	data->pin6_mv = ((raw[data->pin6_index] & 0xFFF) * 5002) >> 12;

	if (data->sensor_cb)
		data->sensor_cb(data);

	return 0;
}

static int evb_input_port_probe(struct platform_device *pdev)
{
	struct evb_input_port_data *data;
	struct iio_channel *cb_chans;
	int err, i;
	int pin1_channel = -1;
	int pin6_channel = -1;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(data->pinctrl)) {
		err = PTR_ERR(data->pinctrl);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Could not get pinctrl\n");
		return err;
	}

	data->pinctrl_default = pinctrl_lookup_state(data->pinctrl, "default");
	if (IS_ERR(data->pinctrl_default)) {
		err = PTR_ERR(data->pinctrl_default);
		dev_err(&pdev->dev, "Could not get pinctrl default\n");
		return err;
	}

	data->pinctrl_i2c = pinctrl_lookup_state(data->pinctrl, "i2c");
	if (IS_ERR(data->pinctrl_i2c)) {
		err = PTR_ERR(data->pinctrl_i2c);
		dev_err(&pdev->dev, "Could not get pinctrl i2c\n");
		return err;
	}

	data->pinctrl_uart = pinctrl_lookup_state(data->pinctrl, "uart");
	if (IS_ERR(data->pinctrl_uart)) {
		err = PTR_ERR(data->pinctrl_uart);
		dev_err(&pdev->dev, "Could not get pinctrl uart\n");
		return err;
	}

	data->iio_cb = iio_channel_get_all_cb(&pdev->dev, evb_input_port_buf_cb,
					      data);
	if (IS_ERR(data->iio_cb)) {
		err = PTR_ERR(data->iio_cb);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Could not get iio channel callbacks.\n");
		return err;
	}

	cb_chans = iio_channel_cb_get_channels(data->iio_cb);
	/* cb_chans is a null-terminated array */
	for (i = 0; cb_chans[i].channel; i++) {
		/*
		 * For now, assuming that the first channel is pin1 and the
		 * second channel is pin 2. Probably would be better if we
		 * checked device tree though.
		 */
		if (pin1_channel == -1)
			pin1_channel = cb_chans[i].channel->channel;
		else if (pin6_channel == -1)
			pin6_channel = cb_chans[i].channel->channel;
		/* Ignoring the case if we have more than 2 channels */
	}
	if (pin1_channel == -1) {
		err = -EINVAL;
		dev_err(&pdev->dev, "Could not find iio channel for pin 1.\n");
		goto err_release_iio_cb;
	}
	if (pin6_channel == -1) {
		err = -EINVAL;
		dev_err(&pdev->dev, "Could not find iio channel for pin 6.\n");
		goto err_release_iio_cb;
	}
	/*
	 * The data returned in the iio buffer callback is in the order of
	 * the channel number, not in the order of the channels in the
	 * device tree. So, if pin 6 has a lower channel number, it's data
	 * will be first.
	 */
	if (pin1_channel > pin6_channel)
		data->pin1_index = 1;
	else
		data->pin6_index = 1;

	err = iio_channel_start_all_cb(data->iio_cb);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to start iio callbacks.\n");
		goto err_release_iio_cb;
	}

	data->pin5_gpio = devm_gpiod_get(&pdev->dev, "pin5", GPIOD_IN);
	if (IS_ERR(data->pin5_gpio)) {
		err = PTR_ERR(data->pin5_gpio);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Pin 5 gpio is required.\n");
		goto err_stop_iio_cb;
	}
	data->pin6_gpio = devm_gpiod_get(&pdev->dev, "pin6", GPIOD_IN);
	if (IS_ERR(data->pin6_gpio)) {
		err = PTR_ERR(data->pin6_gpio);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Pin 6 gpio is required.\n");
		goto err_stop_iio_cb;
	}
	data->buf_ena_gpio = devm_gpiod_get(&pdev->dev, "buf-ena", GPIOD_OUT_LOW);
	if (IS_ERR(data->buf_ena_gpio)) {
		err = PTR_ERR(data->buf_ena_gpio);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Buffer enable gpio is required.\n");
		goto err_stop_iio_cb;
	}

	/* Save the optional pins for last to prevent spurious messages */
	data->pin1_gpio = devm_gpiod_get(&pdev->dev, "pin1", GPIOD_OUT_LOW);
	if (IS_ERR(data->pin1_gpio)) {
		err = PTR_ERR(data->pin1_gpio);
		if (err == -EPROBE_DEFER)
			goto err_stop_iio_cb;
		data->pin1_gpio = NULL;
	}
	data->pin2_gpio = devm_gpiod_get(&pdev->dev, "pin2", GPIOD_IN);
	if (IS_ERR(data->pin2_gpio)) {
		err = PTR_ERR(data->pin2_gpio);
		if (err == -EPROBE_DEFER)
			goto err_stop_iio_cb;
		data->pin2_gpio = NULL;
	}

	err = of_property_read_u32(pdev->dev.of_node, "ev3dev,i2c-adapter", &i);
	if (err) {
		dev_err(&pdev->dev, "No i2c adapter specified\n");
		goto err_stop_iio_cb;
	}
	data->i2c_adap = i2c_get_adapter(i);
	if (!data->i2c_adap) {
		err = -EPROBE_DEFER;
		goto err_stop_iio_cb;
	}

	data->port.name = evb_input_port_type.name;
	snprintf(data->port.address, LEGO_NAME_SIZE, "%s", dev_name(&pdev->dev));
	/* tty name is optional, so ignoring return value */
	of_property_read_string(pdev->dev.of_node, "ev3dev,tty-name",
				&data->port.port_alias);
	data->port.num_modes = NUM_EV3_INPUT_PORT_MODE;
	data->port.supported_modes = BIT(EV3_INPUT_PORT_MODE_AUTO)
				   | BIT(EV3_INPUT_PORT_MODE_NXT_ANALOG)
				   | BIT(EV3_INPUT_PORT_MODE_EV3_ANALOG)
				   | BIT(EV3_INPUT_PORT_MODE_EV3_UART)
				   | BIT(EV3_INPUT_PORT_MODE_OTHER_UART);

	data->port.mode_info = evb_input_port_mode_info;
	data->port.set_mode = evb_input_port_set_mode;
	data->port.set_device = evb_input_port_set_device;
	data->port.get_status = evb_input_port_get_status;
	data->port.nxt_analog_ops = &evb_input_port_nxt_analog_ops;
	if (data->pin1_gpio)
		data->port.nxt_i2c_ops = &evb_input_port_nxt_i2c_ops;
	data->port.context = data;
	dev_set_drvdata(&pdev->dev, data);

	err = lego_port_register(&data->port, &evb_input_port_type, &pdev->dev);
	if (err) {
		dev_err(&pdev->dev, "Failed to register input port.\n");
		goto err_put_i2c_adapter;
	}

	INIT_WORK(&data->change_uevent_work, evb_input_port_change_uevent_work);
	INIT_WORK(&data->work, NULL);
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = evb_input_port_timer_callback;

	data->con_state = CON_STATE_INIT;
	hrtimer_start(&data->timer, ktime_set(0, INPUT_PORT_POLL_NS),
		      HRTIMER_MODE_REL);

	if (!data->pin1_gpio)
		dev_warn(&pdev->dev, "Port does not support 9V on pin 1\n");
	if (!data->pin2_gpio)
		dev_warn(&pdev->dev, "Port cannot discriminate EV3/NXT sensors\n");

	return 0;

err_put_i2c_adapter:
	i2c_put_adapter(data->i2c_adap);
err_stop_iio_cb:
	iio_channel_stop_all_cb(data->iio_cb);
err_release_iio_cb:
	iio_channel_release_all_cb(data->iio_cb);

	return err;
}

static int evb_input_port_remove(struct platform_device *pdev)
{
	struct evb_input_port_data *data = dev_get_drvdata(&pdev->dev);

	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->change_uevent_work);
	cancel_work_sync(&data->work);
	if (data->port.mode == EV3_INPUT_PORT_MODE_OTHER_UART)
		evb_input_port_disable_uart(data);
	if (data->port.mode == EV3_INPUT_PORT_MODE_RAW)
		evb_input_port_disable_raw_mode(data);
	evb_input_port_unregister_sensor(&data->work);
	lego_port_unregister(&data->port);
	i2c_put_adapter(data->i2c_adap);
	iio_channel_stop_all_cb(data->iio_cb);
	iio_channel_release_all_cb(data->iio_cb);
	evb_input_port_float(data);

	return 0;
}

static const struct of_device_id evb_input_port_dt_ids[] = {
	{ .compatible = "ev3dev,evb-input-port", },
	{ }
};
MODULE_DEVICE_TABLE(of, evb_input_port_dt_ids);

static struct platform_driver evb_input_port_driver = {
	.driver	= {
		.name		= "evb-input-port",
		.of_match_table = evb_input_port_dt_ids,
	},
	.probe	= evb_input_port_probe,
	.remove	= evb_input_port_remove,
};
module_platform_driver(evb_input_port_driver);

MODULE_DESCRIPTION("Support for FatcatLab EVB input ports.");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:evb-input-port");
