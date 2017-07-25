/*
 * EV3 Input port driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2013-2015 David Lechner <david@lechnology.com>
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
 * LEGO MINDSTORMS EV3 has four input ports labeled 1, 2, 3 and 4. These ports
 * are used with LEGO MINDSTORMS EV3 and NXT compatible sensors and other
 * devices. Most sensors can be automatically detected when plugged in.
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/platform_data/legoev3.h>

#include <mach/mux.h>

#include <lego.h>
#include <lego_port_class.h>

#include "legoev3_analog.h"
#include "legoev3_i2c.h"
#include "legoev3_ports.h"
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

enum gpio_index {
	GPIO_PIN1,
	GPIO_PIN2,
	GPIO_PIN5,
	GPIO_PIN6,
	GPIO_BUF_ENA,
	GPIO_I2C_CLK,
	NUM_GPIO
};

enum pin5_mux_mode {
	PIN5_MUX_MODE_I2C,
	PIN5_MUX_MODE_UART,
	NUM_PIN5_MUX_MODE
};

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

static const char * const ev3_input_port_state_names[] = {
	[SENSOR_NONE]		= "no-sensor",
	[SENSOR_NXT_ANALOG]	= "nxt-analog",
	[SENSOR_NXT_COLOR]	= "nxt-color",
	[SENSOR_NXT_I2C]	= "nxt-i2c",
	[SENSOR_EV3_ANALOG]	= "ev3-analog",
	[SENSOR_EV3_UART]	= "ev3-uart",
	[SENSOR_ERR]		= "error",
};

enum ev3_input_port_mode {
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

struct ev3_input_port_mode_name {
	enum ev3_input_port_mode mode;
	const char *name;
};

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new modes have the same syntax. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the Documentation/json/ directory.
 */

static const struct lego_port_mode_info legoev3_input_port_mode_info[] = {
	/**
	 * @description: EV3 Input Port
	 * @module: legoev3-ports
	 * @connection_types: NXT/Analog, NXT/I2C, Other/I2C, EV3/Analog, EV3/UART, Other/UART
	 * @prefix: in
	 * @module: legoev3_ports
	 */
	[EV3_INPUT_PORT_MODE_AUTO] = {
		/**
		 * .. [#in-port-auto-mode] In auto mode, the port will attempt to
		 *    automatically detect the type of sensor that was connected and
		 *    load the appropriate driver. See the list of :ref:`supported-sensors`
		 *    to determine if a sensor can be automatically detected.
		 *
		 * @description: Automatically detect sensors.
		 * @name_footnote: [#in-port-auto-mode]_
		 */
		.name	= "auto",
	},
	[EV3_INPUT_PORT_MODE_NXT_ANALOG] = {
		/**
		 * .. [#in-port-nxt-analog-mode] This loads the generic
		 *    :ref:`nxt-analog-sensors` driver by default. Use
		 *    ``set_device`` to load the appropriate device/driver.
		 *
		 * @description: Configure the port for NXT/Analog sensors
		 * @name_footnote: [#in-port-nxt-analog-mode]_
		 */
		.name	= "nxt-analog",
	},
	[EV3_INPUT_PORT_MODE_NXT_COLOR] = {
		/**
		 * .. [#in-port-nxt-color-mode] NXT Color sensor driver has not been
		 *    implemented yet, so right now, this mode does nothing.
		 *
		 * @description: Configure the port for the LEGO NXT Color sensor
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
	[EV3_INPUT_PORT_MODE_OTHER_I2C] ={
		/**
		 * @description: Configure for I2C communications but do not probe for NXT sensors.
		 */
		.name	= "other-i2c",
	},
	[EV3_INPUT_PORT_MODE_EV3_ANALOG] = {
		/**
		 * @description: Configure the port for EV3/Analog sensors
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
		 * .. [#in-port-raw-mode] Exports gpios and analog/digital converter
		 *    values to sysfs so that they can be controlled directly.
		 *
		 * @description: Provide access to low level drivers.
		 * @name_footnote: [#in-port-raw-mode]_
		 */
		.name	= "raw",
	},
};

struct device_type ev3_input_port_sensor_types[] = {
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

const char *ev3_input_port_sensor_table[] = {
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

static struct device_type ev3_input_port_type = {
	.name	= "legoev3-input-port",
};

/**
 * struct ev3_input_port_data - Driver data for an input port on the EV3 brick
 * @id: Unique identifier for the port.
 * @port: Pointer to the legoev3_port that is bound to this instance.
 * @analog: pointer to the legoev3-analog device for accessing data from the
 *	analog/digital converter.
 * @gpio: Array of gpio pins used by this input port.
 * @pin5_mux: Pin mux info for the i2c clock and uart Tx pins. These two
 *	functions share a physical pin on the microprocessor so we have to
 *	change the pin mux each time we change which one we are using.
 * @i2c_data: Platform data for i2c-gpio platform device.
 * @i2c_pdev_info: Platform device information for creating a new i2c-gpio
 *	device each time we connect an i2c sensor.
 * @i2c_pdev: I2C platform device.
 * @change_uevent_work: Needed when change is triggered in atomic context.
 * @work: Worker for registering and unregistering sensors when they are
 *	connected and disconnected.
 * @timer: Polling timer to monitor the port.
 * @timer_loop_cnt: Used to measure time in the polling loop.
 * @con_state: The current state of the port.
 * @pin_state_flags: Used in the polling loop to track certain changes in the
 *	state of the port's pins.
 * @pin1_mv: Used in the polling loop to track changes in pin 1 voltage.
 * @sensor_type: The type of sensor currently connected.
 * @sensor_type_id: The sensor type id for EV3 sensors or -1 for NXT sensors.
 * @sensor: The sensor connected to the port
 */
struct ev3_input_port_data {
	enum legoev3_input_port_id id;
	struct lego_port_device port;
	struct legoev3_analog_device *analog;
	struct gpio gpio[NUM_GPIO];
	unsigned pin5_mux[NUM_PIN5_MUX_MODE];
	struct i2c_legoev3_platform_data i2c_data;
	struct platform_device_info i2c_pdev_info;
	struct platform_device *i2c_pdev;
	struct work_struct change_uevent_work;
	struct work_struct work;
	struct hrtimer timer;
	unsigned timer_loop_cnt;
	enum connection_state con_state;
	unsigned pin_state_flags:NUM_PIN_STATE_FLAG;
	unsigned pin1_mv;
	enum sensor_type sensor_type;
	enum sensor_type_id sensor_type_id;
	struct lego_device *sensor;
};

static int ev3_input_port_get_pin1_mv(struct ev3_input_port_data *data)
{
	return legoev3_analog_in_pin1_value(data->analog, data->id);
}

static int ev3_input_port_get_pin6_mv(struct ev3_input_port_data *data)
{
	return legoev3_analog_in_pin6_value(data->analog, data->id);
}

static int ev3_input_port_set_gpio(struct ev3_input_port_data *data,
				   unsigned pin,
				   enum lego_port_gpio_state state)
{
	if (state == LEGO_PORT_GPIO_FLOAT)
		return gpio_direction_input(data->gpio[pin].gpio);
	return gpio_direction_output(data->gpio[pin].gpio,
				     state == LEGO_PORT_GPIO_HIGH);
}

static int ev3_input_port_set_pin1_gpio(void *context,
					enum lego_port_gpio_state state)
{
	struct ev3_input_port_data *data = context;

	return ev3_input_port_set_gpio(data, GPIO_PIN1, state);
}

static struct lego_port_nxt_i2c_ops ev3_input_port_nxt_i2c_ops = {
	.set_pin1_gpio	= ev3_input_port_set_pin1_gpio,
};

static int ev3_input_port_set_pin5_gpio(void *context,
					enum lego_port_gpio_state state)
{
	struct ev3_input_port_data *data = context;

	return ev3_input_port_set_gpio(data, GPIO_PIN5, state);
}

static struct lego_port_nxt_analog_ops ev3_input_port_nxt_analog_ops = {
	.set_pin5_gpio	= ev3_input_port_set_pin5_gpio,
};

static void ev3_input_port_nxt_analog_cb(void *context)
{
	struct ev3_input_port_data *data = context;

	if (data->port.raw_data)
		*(s32 *)data->port.raw_data = ev3_input_port_get_pin1_mv(data);
	if (data->port.notify_raw_data_func)
		data->port.notify_raw_data_func(data->port.notify_raw_data_context);
}

static void ev3_input_port_ev3_analog_cb(void *context)
{
	struct ev3_input_port_data *data = context;

	if (data->port.raw_data)
		*(s32 *)data->port.raw_data = ev3_input_port_get_pin6_mv(data);
	if (data->port.notify_raw_data_func)
		data->port.notify_raw_data_func(data->port.notify_raw_data_context);
}

int ev3_input_port_register_i2c(struct ev3_input_port_data *data,
				unsigned int class)
{
	struct platform_device *pdev;
	int err;

	gpio_set_value(data->gpio[GPIO_BUF_ENA].gpio, 0); /* active low */
	err = davinci_cfg_reg(data->pin5_mux[PIN5_MUX_MODE_I2C]);
	if (err) {
		dev_err(&data->port.dev, "Pin 5 mux failed for i2c device.\n");
		goto davinci_cfg_reg_fail;
	}
	data->i2c_pdev_info.parent = &data->port.dev;
	data->i2c_data.class = class;
	pdev = platform_device_register_full(&data->i2c_pdev_info);
	if (IS_ERR(pdev)) {
		dev_err(&data->port.dev, "Could not register i2c device.\n");
		err = PTR_ERR(pdev);
		goto platform_device_register_fail;
	}
	data->i2c_pdev = pdev;

	return 0;

platform_device_register_fail:
davinci_cfg_reg_fail:
	gpio_set_value(data->gpio[GPIO_BUF_ENA].gpio, 1); /* active low */

	return err;
}

void ev3_input_port_unregister_i2c(struct ev3_input_port_data *data)
{
	if (!data->i2c_pdev)
		return;

	platform_device_unregister(data->i2c_pdev);
	data->i2c_pdev = NULL;
	gpio_set_value(data->gpio[GPIO_BUF_ENA].gpio, 1); /* active low */
}

int ev3_input_port_enable_uart(struct ev3_input_port_data *data)
{
	int err;

	err = davinci_cfg_reg(data->pin5_mux[PIN5_MUX_MODE_UART]);
	if (err) {
		dev_err(&data->port.dev, "Pin 5 mux failed for uart device.\n");
		return err;
	}
	gpio_set_value(data->gpio[GPIO_BUF_ENA].gpio, 0); /* active low */

	return 0;
}

void ev3_input_port_disable_uart(struct ev3_input_port_data *data)
{
	gpio_set_value(data->gpio[GPIO_BUF_ENA].gpio, 1); /* active low */
}

void ev3_input_port_float(struct ev3_input_port_data *data)
{
	gpio_direction_output(data->gpio[GPIO_PIN1].gpio, 0);
	gpio_direction_input(data->gpio[GPIO_PIN2].gpio);
	gpio_direction_input(data->gpio[GPIO_PIN5].gpio);
	gpio_direction_input(data->gpio[GPIO_PIN6].gpio);
	gpio_direction_output(data->gpio[GPIO_BUF_ENA].gpio, 1); /* active low */
}

void ev3_input_port_register_sensor(struct work_struct *work)
{
	struct ev3_input_port_data *data =
			container_of(work, struct ev3_input_port_data, work);
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
		legoev3_analog_register_in_cb(data->analog, data->id,
			ev3_input_port_nxt_analog_cb, data);
		break;
	case SENSOR_EV3_ANALOG:
		legoev3_analog_register_in_cb(data->analog, data->id,
			ev3_input_port_ev3_analog_cb, data);
		break;
	case SENSOR_NXT_I2C:
		/* Give the sensor time to boot */
		msleep(1000);
		ev3_input_port_register_i2c(data, I2C_CLASS_LEGO);
		/*
		 * I2C sensors are handled by the i2c stack, so we are just
		 * registering a fake device here so that it doesn't break
		 * the automatic detection.
		 */
		break;
	case SENSOR_EV3_UART:
		ev3_input_port_enable_uart(data);
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
		ev3_input_port_sensor_table[data->sensor_type_id],
		&ev3_input_port_sensor_types[data->sensor_type],
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

void ev3_input_port_unregister_sensor(struct work_struct *work)
{
	struct ev3_input_port_data *data =
			container_of(work, struct ev3_input_port_data, work);

	lego_device_unregister(data->sensor);
	data->sensor_type = SENSOR_NONE;
	data->sensor = NULL;
	legoev3_analog_register_in_cb(data->analog, data->id, NULL, NULL);
	ev3_input_port_unregister_i2c(data);
	ev3_input_port_disable_uart(data);
}

void ev3_input_port_change_uevent_work(struct work_struct *work)
{
	struct ev3_input_port_data *data =
		container_of(work, struct ev3_input_port_data, change_uevent_work);

	kobject_uevent(&data->port.dev.kobj, KOBJ_CHANGE);
}

static enum hrtimer_restart ev3_input_port_timer_callback(struct hrtimer *timer)
{
	struct ev3_input_port_data *data =
			container_of(timer, struct ev3_input_port_data, timer);
	enum sensor_type prev_sensor_type = data->sensor_type;
	unsigned new_pin_state_flags = 0;
	unsigned new_pin1_mv = 0;

	hrtimer_forward_now(timer, ktime_set(0, INPUT_PORT_POLL_NS));
	data->timer_loop_cnt++;

	switch(data->con_state) {
	case CON_STATE_INIT:
		if (!data->sensor) {
			ev3_input_port_float(data);
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
		new_pin1_mv = legoev3_analog_in_pin1_value(data->analog, data->id);
		if (!gpio_get_value(data->gpio[GPIO_PIN2].gpio))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN2_LOW);
		if (new_pin1_mv < PIN1_NEAR_5V)
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN1_LOADED);
		if (!gpio_get_value(data->gpio[GPIO_PIN5].gpio))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN5_LOW);
		if (gpio_get_value(data->gpio[GPIO_PIN6].gpio))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN6_HIGH);
		if (new_pin_state_flags != data->pin_state_flags)
			data->timer_loop_cnt = 0;
		else if (new_pin_state_flags && data->timer_loop_cnt >= ADD_CNT
			 && !work_busy(&data->work))
		{
			if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN2_LOW)) {
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
					data->pin1_mv = new_pin1_mv;
				} else {
					data->sensor_type = SENSOR_NXT_ANALOG;
					data->sensor_type_id = SENSOR_TYPE_ID_NXT_ANALOG;
				}
			} else if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN1_LOADED)) {
				data->con_state = CON_STATE_HAVE_EV3;
				if (new_pin1_mv > PIN1_NEAR_PIN2) {
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
				data->con_state = CON_STATE_HAVE_I2C;
				data->sensor_type = SENSOR_NXT_I2C;
				data->sensor_type_id = SENSOR_TYPE_ID_NXT_I2C;
			} else {
				data->con_state = CON_STATE_HAVE_PIN5_ERR;
				data->sensor_type = SENSOR_ERR;
				data->sensor_type_id = SENSOR_TYPE_ID_UNKNOWN;
			}
			data->timer_loop_cnt = 0;
			if (data->sensor_type != SENSOR_ERR) {
				INIT_WORK(&data->work, ev3_input_port_register_sensor);
				schedule_work(&data->work);
			}
		}
		data->pin_state_flags = new_pin_state_flags;
		break;
	case CON_STATE_TEST_NXT_TOUCH:
		if (data->timer_loop_cnt >= SETTLE_CNT) {
			data->con_state = CON_STATE_HAVE_NXT;
			data->sensor_type = SENSOR_NXT_ANALOG;
			new_pin1_mv = legoev3_analog_in_pin1_value(data->analog, data->id);
			if (new_pin1_mv > (data->pin1_mv - PIN1_TOUCH_VAR) &&
			    new_pin1_mv < (data->pin1_mv + PIN1_TOUCH_VAR))
				data->sensor_type_id = SENSOR_TYPE_ID_NXT_TOUCH;
			else
				data->sensor_type_id = SENSOR_TYPE_ID_NXT_ANALOG;
		}
		break;
	case CON_STATE_HAVE_NXT:
		if (!gpio_get_value(data->gpio[GPIO_PIN2].gpio))
			data->timer_loop_cnt = 0;
		break;
	case CON_STATE_HAVE_EV3:
		new_pin1_mv = legoev3_analog_in_pin1_value(data->analog, data->id);
		if (new_pin1_mv < PIN1_NEAR_5V)
			data->timer_loop_cnt = 0;
		break;
	case CON_STATE_HAVE_I2C:
		if (gpio_get_value(data->gpio[GPIO_PIN6].gpio))
			data->timer_loop_cnt = 0;
		break;
	case CON_STATE_HAVE_PIN5_ERR:
		if (!gpio_get_value(data->gpio[GPIO_PIN5].gpio))
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
			INIT_WORK(&data->work, ev3_input_port_unregister_sensor);
			schedule_work(&data->work);
		}
		data->con_state = CON_STATE_INIT;
	}

	return HRTIMER_RESTART;
}

/* TODO: This should be hwmon maybe? Dynamic attributes cause problems */

static ssize_t pin1_mv_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct ev3_input_port_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ev3_input_port_get_pin1_mv(data));
}

static ssize_t pin6_mv_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct ev3_input_port_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ev3_input_port_get_pin6_mv(data));
}

static DEVICE_ATTR_RO(pin1_mv);
static DEVICE_ATTR_RO(pin6_mv);

static struct attribute *ev3_input_port_raw_attrs[] = {
	&dev_attr_pin1_mv.attr,
	&dev_attr_pin6_mv.attr,
	NULL
};

ATTRIBUTE_GROUPS(ev3_input_port_raw);

int ev3_input_port_enable_raw_mode(struct ev3_input_port_data *data)
{
	int err, i;

	err = sysfs_create_groups(&data->port.dev.kobj, ev3_input_port_raw_groups);
	if (err < 0)
		return err;

	/* TODO: would be nice to create symlinks from exported gpios to in_port */
	for (i = 0; i < NUM_GPIO; i++) {
		err = gpio_export(data->gpio[i].gpio, true);
		if (err < 0) {
			for (i--; i >= 0; i--)
				gpio_unexport(data->gpio[i].gpio);
			return err;
		}
	}

	return 0;
}

void ev3_input_port_disable_raw_mode(struct ev3_input_port_data *data)
{
	int i;

	sysfs_remove_groups(&data->port.dev.kobj, ev3_input_port_raw_groups);
	for (i = 0; i < NUM_GPIO; i++)
		gpio_unexport(data->gpio[i].gpio);
	ev3_input_port_float(data);
}

static int ev3_input_port_set_mode(void *context, u8 mode)
{
	struct ev3_input_port_data *data = context;
	int err;

	/*
	 * TODO: might be nice to check if we are in auto mode and see if the
	 * current loaded mode matches the new mode. In this case, it would not
	 * be necessary to unload and reload the same sensor.
	 */

	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->work);

	if (data->port.mode == EV3_INPUT_PORT_MODE_OTHER_I2C) {
		ev3_input_port_unregister_i2c(data);
		ev3_input_port_set_pin1_gpio(data, LEGO_PORT_GPIO_LOW);
	}
	if (data->port.mode == EV3_INPUT_PORT_MODE_OTHER_UART)
		ev3_input_port_disable_uart(data);
	if (data->port.mode == EV3_INPUT_PORT_MODE_RAW)
		ev3_input_port_disable_raw_mode(data);
	if (data->sensor)
		ev3_input_port_unregister_sensor(&data->work);

	switch (mode) {
	case EV3_INPUT_PORT_MODE_AUTO:
		data->con_state = CON_STATE_INIT;
		hrtimer_start(&data->timer, ktime_set(0, INPUT_PORT_POLL_NS),
							HRTIMER_MODE_REL);
		break;
	case EV3_INPUT_PORT_MODE_NXT_ANALOG:
		data->sensor_type = SENSOR_NXT_ANALOG;
		data->sensor_type_id = SENSOR_TYPE_ID_NXT_ANALOG;
		ev3_input_port_register_sensor(&data->work);
		break;
	case EV3_INPUT_PORT_MODE_NXT_COLOR:
		data->sensor_type = SENSOR_NXT_COLOR;
		data->sensor_type_id = SENSOR_TYPE_ID_NXT_COLOR;
		ev3_input_port_register_sensor(&data->work);
		break;
	case EV3_INPUT_PORT_MODE_NXT_I2C:
		data->sensor_type = SENSOR_NXT_I2C;
		data->sensor_type_id = SENSOR_TYPE_ID_NXT_I2C;
		ev3_input_port_register_sensor(&data->work);
		break;
	case EV3_INPUT_PORT_MODE_OTHER_I2C:
		data->sensor_type = SENSOR_NONE;
		data->sensor_type_id = SENSOR_TYPE_ID_UNKNOWN;
		ev3_input_port_register_i2c(data, 0);
		ev3_input_port_set_pin1_gpio(data, LEGO_PORT_GPIO_HIGH);
		break;
	case EV3_INPUT_PORT_MODE_EV3_ANALOG:
		data->sensor_type = SENSOR_EV3_ANALOG;
		data->sensor_type_id = SENSOR_TYPE_ID_EV3_ANALOG_01;
		ev3_input_port_register_sensor(&data->work);
		break;
	case EV3_INPUT_PORT_MODE_EV3_UART:
		data->sensor_type = SENSOR_EV3_UART;
		data->sensor_type_id = SENSOR_TYPE_ID_EV3_UART;
		ev3_input_port_register_sensor(&data->work);
		break;
	case EV3_INPUT_PORT_MODE_OTHER_UART:
		data->sensor_type = SENSOR_NONE;
		data->sensor_type_id = SENSOR_TYPE_ID_UNKNOWN;
		err = ev3_input_port_enable_uart(data);
		if (err < 0)
			return err;
		break;
	case EV3_INPUT_PORT_MODE_RAW:
		data->sensor_type = SENSOR_NONE;
		data->sensor_type_id = SENSOR_TYPE_ID_UNKNOWN;
		err = ev3_input_port_enable_raw_mode(data);
		if (err < 0)
			return err;
		break;
	default:
		/* should never get here, but keeps compiler happy */
		break;
	}

	return 0;
}

static int ev3_input_port_set_device(void *context, const char *device_name)
{
	struct ev3_input_port_data *data = context;
	struct lego_device *new_sensor;

	if (data->sensor_type != SENSOR_NXT_ANALOG)
		return -EOPNOTSUPP;

	lego_device_unregister(data->sensor);
	data->sensor = NULL;

	new_sensor = lego_device_register(device_name,
		&ev3_input_port_sensor_types[data->sensor_type],
		&data->port, NULL, 0);
	if (IS_ERR(new_sensor))
		return PTR_ERR(new_sensor);

	data->sensor = new_sensor;

	return 0;
}

static const char *ev3_input_port_get_status(void *context)
{
	struct ev3_input_port_data *data = context;

	return ev3_input_port_state_names[data->sensor_type];
}


struct lego_port_device
*ev3_input_port_register(struct ev3_input_port_platform_data *pdata,
			 struct device *parent)
{
	struct ev3_input_port_data *data;
	int err;

	if (WARN(!pdata, "Platform data is required."))
		return ERR_PTR(-EINVAL);

	data = kzalloc(sizeof(struct ev3_input_port_data), GFP_KERNEL);
	if (!data)
		return ERR_PTR(-ENOMEM);

	data->id = pdata->id;
	data->analog = get_legoev3_analog();
	if (IS_ERR(data->analog)) {
		dev_err(parent, "Could not get legoev3-analog device.\n");
		err = PTR_ERR(data->analog);
		goto err_request_legoev3_analog;
	}

	data->gpio[GPIO_PIN1].gpio	= pdata->pin1_gpio;
	data->gpio[GPIO_PIN1].flags	= GPIOF_OUT_INIT_LOW;
	data->gpio[GPIO_PIN1].label	= "pin1";
	data->gpio[GPIO_PIN2].gpio	= pdata->pin2_gpio;
	data->gpio[GPIO_PIN2].flags	= GPIOF_IN;
	data->gpio[GPIO_PIN2].label	= "pin2";
	data->gpio[GPIO_PIN5].gpio	= pdata->pin5_gpio;
	data->gpio[GPIO_PIN5].flags	= GPIOF_IN;
	data->gpio[GPIO_PIN5].label	= "pin5";
	data->gpio[GPIO_PIN6].gpio	= pdata->pin6_gpio;
	data->gpio[GPIO_PIN6].flags	= GPIOF_IN;
	data->gpio[GPIO_PIN6].label	= "pin6";
	data->gpio[GPIO_BUF_ENA].gpio	= pdata->buf_ena_gpio;
	data->gpio[GPIO_BUF_ENA].flags	= GPIOF_OUT_INIT_HIGH;
	data->gpio[GPIO_BUF_ENA].label	= "buf_ena";
	data->gpio[GPIO_I2C_CLK].gpio	= pdata->i2c_clk_gpio;
	data->gpio[GPIO_I2C_CLK].flags	= GPIOF_IN;
	data->gpio[GPIO_I2C_CLK].label	= "i2c_clk";

	err = gpio_request_array(data->gpio, ARRAY_SIZE(data->gpio));
	if (err) {
		dev_err(parent, "Requesting GPIOs failed.\n");
		goto err_gpio_request_array;
	}

	data->pin5_mux[PIN5_MUX_MODE_I2C] = pdata->i2c_pin_mux;
	data->pin5_mux[PIN5_MUX_MODE_UART] = pdata->uart_pin_mux;

	data->i2c_data.sensor_platform_data.in_port = &data->port;
	data->i2c_data.sda_pin	= pdata->pin6_gpio;
	data->i2c_data.scl_pin	= pdata->i2c_clk_gpio;
	data->i2c_data.port_id	= pdata->id;
	/* data->i2c_data.class is dynamically set */

	data->i2c_pdev_info.name	= "i2c-legoev3";
	data->i2c_pdev_info.id		= pdata->i2c_dev_id;
	data->i2c_pdev_info.data	= &data->i2c_data;
	data->i2c_pdev_info.size_data	= sizeof(data->i2c_data);

	data->port.name = ev3_input_port_type.name;
	snprintf(data->port.address, LEGO_NAME_SIZE, "in%d",
		 pdata->id + 1);
	data->port.port_alias = pdata->uart_tty;
	data->port.num_modes = NUM_EV3_INPUT_PORT_MODE;
	data->port.supported_modes = LEGO_PORT_ALL_MODES;
	data->port.mode_info = legoev3_input_port_mode_info;
	data->port.set_mode = ev3_input_port_set_mode;
	data->port.set_device = ev3_input_port_set_device;
	data->port.get_status = ev3_input_port_get_status;
	data->port.nxt_analog_ops = &ev3_input_port_nxt_analog_ops;
	data->port.nxt_i2c_ops = &ev3_input_port_nxt_i2c_ops;
	data->port.context = data;
	err = lego_port_register(&data->port, &ev3_input_port_type, parent);
	if (err) {
		dev_err(parent, "Failed to register input port. (%d)\n", err);
		goto err_lego_port_register;
	}

	INIT_WORK(&data->change_uevent_work, ev3_input_port_change_uevent_work);
	INIT_WORK(&data->work, NULL);
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = ev3_input_port_timer_callback;

	data->con_state = CON_STATE_INIT;
	hrtimer_start(&data->timer, ktime_set(0, INPUT_PORT_POLL_NS),
		      HRTIMER_MODE_REL);

	return &data->port;

err_lego_port_register:
	gpio_free_array(data->gpio, ARRAY_SIZE(data->gpio));
err_gpio_request_array:
	put_legoev3_analog(data->analog);
err_request_legoev3_analog:
	kfree(data);

	return ERR_PTR(err);
}

void ev3_input_port_unregister(struct lego_port_device *port)
{
	struct ev3_input_port_data *data;

	/* port can be null if disabled via module parameter */
	if (!port)
		return;

	data =container_of(port, struct ev3_input_port_data, port);

	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->change_uevent_work);
	cancel_work_sync(&data->work);
	if (port->mode == EV3_INPUT_PORT_MODE_OTHER_UART)
		ev3_input_port_disable_uart(data);
	if (port->mode == EV3_INPUT_PORT_MODE_RAW)
		ev3_input_port_disable_raw_mode(data);
	ev3_input_port_unregister_sensor(&data->work);
	lego_port_unregister(&data->port);
	ev3_input_port_float(data);
	gpio_free_array(data->gpio, ARRAY_SIZE(data->gpio));
	put_legoev3_analog(data->analog);
	kfree(data);
}
