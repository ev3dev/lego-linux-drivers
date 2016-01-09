/*
 * Output port driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2016 Ralph Hempel <rhempel@hempeldesigngroup.com>
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_data/legoev3.h>

#include <mach/mux.h>

#include <lego.h>
#include <lego_port_class.h>
#include <dc_motor_class.h>

#include "legoev3_analog.h"
#include "legoev3_ports.h"
#include "../motors/ev3_motor.h"
 
#define OUTPUT_PORT_POLL_NS	10000000			/* 10 msec */
#define SETTLE_CNT		(20000000/OUTPUT_PORT_POLL_NS)	/* 20 msec */
#define ADD_CNT			(350000000/OUTPUT_PORT_POLL_NS)	/* 350 msec */
#define REMOVE_CNT		(100000000/OUTPUT_PORT_POLL_NS)	/* 100 msec */

#define ADC_REF              5000 /* [mV] Maximum voltage that the A/D can read */

#define PIN5_IIC_HIGH        3700 /* [mV] values in between these limits means that */
#define PIN5_IIC_LOW         2800 /* [mV] an NXT I2C sensor or NXT Color sensor is connected */

#define PIN5_MINITACHO_HIGH1 2000 /* [mV] values in between these limits means that */
#define PIN5_MINITACHO_LOW1  1600 /* [mV] a mini tacho motor is pulling high when pin5 is pulling low */

#define PIN5_BALANCE_HIGH    2600 /* [mV] values in between these limits means that */
#define PIN5_BALANCE_LOW     2400 /* [mV] connection 5 is floating */

#define PIN5_LIGHT_HIGH       850 /* [mV] values in between these limits means that */
#define PIN5_LIGHT_LOW        650 /* [mV] an old light sensor is connected */

#define PIN5_MINITACHO_HIGH2  450 /* [mV] values in between these limits means that */
#define PIN5_MINITACHO_LOW2   250 /* [mV] a mini tacho motor is pulling low when pin5 floats */

#define PIN5_NEAR_GND         100 /* [mV] lower  values mean that connection 5 is shorted to ground */

enum gpio_index {
	GPIO_PIN1,
	GPIO_PIN2,
	GPIO_PIN5,
	GPIO_PIN5_INT,
	GPIO_PIN6_DIR,
	NUM_GPIO
};

enum connection_state {
	CON_STATE_INIT,				/* Wait for motor to unregister, then */
						/* Set port to "float" state */
	CON_STATE_INIT_SETTLE,			/* Wait for port to settle */
	CON_STATE_NO_DEV,			/* No device present, wait until something */
						/* interesting happens on one or more */
						/* of the pins and a steady state is */
						/* reached */
	CON_STATE_PIN6_SETTLE,			/* Pin6 to settle after changing state */
	CON_STATE_CONNECTED,			/* We are ready to figure out what's connected */
	CON_STATE_PIN5_SETTLE,			/* Pin5 to settle after changing state */
	CON_STATE_DEVICE_CONNECTED,		/* We detected the connection of a device */
	CON_STATE_WAITING_FOR_DISCONNECT,	/* We are waiting for disconnect */
	NUM_CON_STATE
};

enum pin_state_flag {
	PIN_STATE_FLAG_PIN2_LOW,
	PIN_STATE_FLAG_PIN5_LOADED,
	PIN_STATE_FLAG_PIN5_LOW,
	PIN_STATE_FLAG_PIN6_HIGH,
	NUM_PIN_STATE_FLAG
};

enum legoev3_output_port_mode {
	EV3_OUTPUT_PORT_MODE_AUTO,
	EV3_OUTPUT_PORT_MODE_TACHO_MOTOR,
	EV3_OUTPUT_PORT_MODE_DC_MOTOR,
	EV3_OUTPUT_PORT_MODE_LED,
	EV3_OUTPUT_PORT_MODE_RAW,
	NUM_EV3_OUTPUT_PORT_MODE
};

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new modes have the same layout. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the ev3dev-kpkg repository.
 */

static const struct lego_port_mode_info legoev3_output_port_mode_info[] = {
	/**
	 * @description: EV3 Output Port
	 * @module: legoev3-ports
	 * @connection_types: tacho-motor, dc-motor, led
	 * @prefix: out
	 */
	[EV3_OUTPUT_PORT_MODE_AUTO] = {
		/**
		 * [^auto-mode]: Only the EV3/NXT large motors and the EV3
		 * medium motor can be automatically detected. All other devices
		 * must be manually configured.
		 *
		 * @description: Automatically detect motors when they are connected.
		 * @name_footnote: [^auto-mode]
		 */
		.name	= "auto",
	},
	[EV3_OUTPUT_PORT_MODE_TACHO_MOTOR] = {
		/**
		 * [^tacho-motor-mode]: Configures the port to use the
		 * [tacho-motor] driver module. The default driver is the
		 * EV3 Large Motor (`lego-ev3-l-motor`). You can change
		 * the driver using the `set_device` attribute.
		 * ^
		 * [tacho-motor]: /docs/drivers/tacho-motor
		 *
		 * @description: Load the [tacho-motor] device.
		 * @name_footnote: [^tacho-motor-mode]
		 */
		.name	= "tacho-motor",
	},
	[EV3_OUTPUT_PORT_MODE_DC_MOTOR] = {
		/**
		 * [^dc-motor-mode]: This can be use with MINDSTORMS RCX
		 * motors, Power Functions motors and any other 'plain' DC
		 * motor. By 'plain', we mean the motor is just a motor without
		 * any feedback.
		 * ^
		 * [dc-motor]: /docs/drivers/dc-motor
		 *
		 * @description: Load the [dc-motor] device.
		 * @name_footnote: [^dc-motor-mode]
		 */
		.name	= "dc-motor",
	},
	[EV3_OUTPUT_PORT_MODE_LED] = {
		/**
		 * [^led-mode]: This can be used with MINDSTORMS RCX LEDs,
		 * Power Functions LEDs or any other LED connected to pins 1
		 * and 2 of the output port.
		 * ^
		 * [led]: /docs/drivers/led
		 *
		 * @description: Load the [led] device.
		 * @name_footnote: [^led-mode]
		 */
		.name	= "led",
	},
	[EV3_OUTPUT_PORT_MODE_RAW] = {
		/**
		 * [^raw-mode]: Exports gpios, pwm and analog/digital converter
		 * values to sysfs so that they can be controlled directly.
		 *
		 * @description: Provide access to low level drivers.
		 * @name_footnote: [^raw-mode]
		 */
		.name	= "raw",
	},
};


enum motor_type {
	MOTOR_NONE,
	MOTOR_TACHO,
	MOTOR_DC,
	MOTOR_LED,
	MOTOR_ERR,
	NUM_MOTOR
};

static const struct device_type ev3_motor_device_types[] = {
	[MOTOR_NONE] = {
		.name	= NULL,
	},
	[MOTOR_TACHO] = {
		.name	= "legoev3-motor",
	},
	[MOTOR_DC] = {
		.name	= "rcx-motor",
	},
	[MOTOR_LED] = {
		.name	= "rcx-led",
	},
	[MOTOR_ERR] = {
		.name	= NULL,
	}
};

static const char *ev3_output_port_status_names[] = {
	[MOTOR_NONE]	= "no-motor",
	[MOTOR_TACHO]	= legoev3_output_port_mode_info[MOTOR_TACHO].name,
	[MOTOR_DC]	= legoev3_output_port_mode_info[MOTOR_DC].name,
	[MOTOR_LED]	= legoev3_output_port_mode_info[MOTOR_LED].name,
	[MOTOR_ERR]	= "error",
};

/**
 * struct ev3_output_port_data - Driver data for an output port on the EV3 brick
 * @id: Unique identifier for the port.
 * @out_port: lego-port class device that represents this port.
 * @analog: pointer to the legoev3-analog device for accessing data from the
 *	analog/digital converter.
 * @pwm: Pointer to the pwm device that is bound to this port
 * @gpio: Array of gpio pins used by this input port.
 * @change_uevent_work: Needed when change is triggered in atomic context.
 * @work: Worker for registering and unregistering sensors when they are
 *	connected and disconnected.
 * @timer: Polling timer to monitor the port connect/disconnect.
 * @timer_loop_cnt: Used to measure time in the polling loop.
 * @con_state: The current state of the port.
 * @pin_state_flags: Used in the polling loop to track certain changes in the
 *	state of the port's pins.
 * @pin5_float_mv: Used in the polling loop to track pin 5 voltage.
 * @pin5_low_mv: Used in the polling loop to track pin 5 voltage.
 * @motor_type: The type of motor that was detected or manually set by the mode.
 * @tacho_motor_type: The type of tacho motor that was detected.
 * @motor: Pointer to the motor device that is connected to the output port.
 * @command: The current command for the motor driver of the output port.
 */
struct ev3_output_port_data {
	enum legoev3_output_port_id id;
	struct lego_port_device out_port;
	struct legoev3_analog_device *analog;
	struct pwm_device *pwm;
	struct gpio gpio[NUM_GPIO];
	struct work_struct change_uevent_work;
	struct work_struct work;
	struct hrtimer timer;
	unsigned timer_loop_cnt;
	enum connection_state con_state;
	unsigned pin_state_flags:NUM_PIN_STATE_FLAG;
	unsigned pin5_float_mv;
	unsigned pin5_low_mv;
	enum motor_type motor_type;
	enum ev3_motor_id motor_id;
	struct lego_device *motor;
	enum dc_motor_internal_command command;
};

int ev3_output_port_set_direction_gpios(struct ev3_output_port_data *data)
{
	switch(data->command) {
	case DC_MOTOR_INTERNAL_COMMAND_RUN_FORWARD:
		gpio_direction_output(data->gpio[GPIO_PIN1].gpio, 1);
		gpio_direction_input(data->gpio[GPIO_PIN2].gpio);
		break;
	case DC_MOTOR_INTERNAL_COMMAND_RUN_REVERSE:
		gpio_direction_input(data->gpio[GPIO_PIN1].gpio);
		gpio_direction_output(data->gpio[GPIO_PIN2].gpio, 1);
		break;
	case DC_MOTOR_INTERNAL_COMMAND_BRAKE:
		gpio_direction_output(data->gpio[GPIO_PIN1].gpio, 1);
		gpio_direction_output(data->gpio[GPIO_PIN2].gpio, 1);
		break;
	case DC_MOTOR_INTERNAL_COMMAND_COAST:
		gpio_direction_output(data->gpio[GPIO_PIN1].gpio, 0);
		gpio_direction_output(data->gpio[GPIO_PIN2].gpio, 0);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static unsigned ev3_ouput_port_get_supported_commands(void* context)
{
	return BIT(DC_MOTOR_COMMAND_RUN_FOREVER) | BIT(DC_MOTOR_COMMAND_RUN_DIRECT)
		| BIT(DC_MOTOR_COMMAND_STOP);
}

static unsigned ev3_ouput_port_get_supported_stop_commands(void* context)
{
	return BIT(DC_MOTOR_STOP_COMMAND_COAST) | BIT(DC_MOTOR_STOP_COMMAND_BRAKE);
}

static enum dc_motor_internal_command ev3_output_port_get_command(void *context)
{
	struct ev3_output_port_data *data = context;

	return data->command;
}

static int ev3_output_port_set_command(void *context,
				       enum dc_motor_internal_command command)
{
	struct ev3_output_port_data *data = context;

	if (data->command == command)
		return 0;

	data->command = command;

	return ev3_output_port_set_direction_gpios(data);
}

static unsigned ev3_output_port_get_duty_cycle(void *context)
{
	struct ev3_output_port_data *data = context;
	unsigned period = pwm_get_period(data->pwm);

	if (unlikely(period == 0))
		return 0;
	return pwm_get_duty_cycle(data->pwm) * 100 / period;
}

static int ev3_output_port_set_duty_cycle(void *context, unsigned duty)
{
	struct ev3_output_port_data *data = context;
	unsigned period = pwm_get_period(data->pwm);

	if (duty > 100)
		return -EINVAL;
	return pwm_config(data->pwm, period * duty / 100, period);
}

static struct dc_motor_ops ev3_output_port_motor_ops = {
	.get_supported_commands	= ev3_ouput_port_get_supported_commands,
	.get_supported_stop_commands = ev3_ouput_port_get_supported_stop_commands,
	.get_command		= ev3_output_port_get_command,
	.set_command		= ev3_output_port_set_command,
	.set_duty_cycle		= ev3_output_port_set_duty_cycle,
	.get_duty_cycle		= ev3_output_port_get_duty_cycle,
};

void ev3_output_port_float(struct ev3_output_port_data *data)
{
	gpio_direction_output(data->gpio[GPIO_PIN1].gpio, 0);
	gpio_direction_output(data->gpio[GPIO_PIN2].gpio, 0);
	gpio_direction_input(data->gpio[GPIO_PIN5].gpio);
	gpio_direction_input(data->gpio[GPIO_PIN5_INT].gpio);
	gpio_direction_input(data->gpio[GPIO_PIN6_DIR].gpio);
	data->command = DC_MOTOR_INTERNAL_COMMAND_COAST;
}

void ev3_output_port_change_uevent_work(struct work_struct *work)
{
	struct ev3_output_port_data *data =
		container_of(work, struct ev3_output_port_data, change_uevent_work);

	kobject_uevent(&data->out_port.dev.kobj, KOBJ_CHANGE);
}

void ev3_output_port_register_motor(struct work_struct *work)
{
	struct ev3_output_port_data *data =
			container_of(work, struct ev3_output_port_data, work);
	struct lego_device *motor;
	struct ev3_motor_platform_data pdata;
	const char *driver_name;

	if (data->motor_type >= NUM_MOTOR
	    || !ev3_motor_device_types[data->motor_type].name)
	{
		dev_err(&data->out_port.dev,
			"Trying to register an invalid motor type on %s.\n",
			dev_name(&data->out_port.dev));
		return;
	}

	pdata.tacho_int_gpio = data->gpio[GPIO_PIN5_INT].gpio;
	pdata.tacho_dir_gpio = data->gpio[GPIO_PIN6_DIR].gpio;
	if (data->motor_type == MOTOR_TACHO)
		driver_name = ev3_motor_defs[data->motor_id].name;
	else
		driver_name = ev3_motor_device_types[data->motor_type].name;

	motor = lego_device_register(driver_name,
		&ev3_motor_device_types[data->motor_type],
		&data->out_port, &pdata, sizeof(struct ev3_motor_platform_data));
	if (IS_ERR(motor)) {
		dev_err(&data->out_port.dev,
			"Could not register motor on port %s.\n",
			dev_name(&data->out_port.dev));
		return;
	}

	data->motor = motor;

	return;
}

void ev3_output_port_unregister_motor(struct work_struct *work)
{
	struct ev3_output_port_data *data =
			container_of(work, struct ev3_output_port_data, work);

	lego_device_unregister(data->motor);
	data->motor_type = MOTOR_NONE;
	data->motor = NULL;
}

static enum hrtimer_restart ev3_output_port_timer_callback(struct hrtimer *timer)
{
	struct ev3_output_port_data *data =
			container_of(timer, struct ev3_output_port_data, timer);
	enum motor_type prev_motor_type = data->motor_type;
	unsigned new_pin_state_flags = 0;
	unsigned new_pin5_mv = 0;

	hrtimer_forward_now(timer, ktime_set(0, OUTPUT_PORT_POLL_NS));
	data->timer_loop_cnt++;

	switch(data->con_state) {
	case CON_STATE_INIT:
		if (!data->motor) {
			ev3_output_port_float(data);
			data->timer_loop_cnt = 0;
			data->motor_type = MOTOR_NONE;
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
		new_pin5_mv = legoev3_analog_out_pin5_value(data->analog, data->id);

		if (gpio_get_value(data->gpio[GPIO_PIN6_DIR].gpio))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN6_HIGH);
		if ((new_pin5_mv < PIN5_BALANCE_LOW) || (new_pin5_mv > PIN5_BALANCE_HIGH))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN5_LOADED);

		if (new_pin_state_flags != data->pin_state_flags) {
			data->pin_state_flags = new_pin_state_flags;
			data->timer_loop_cnt = 0;
		}

		if (data->pin_state_flags && (data->timer_loop_cnt >= ADD_CNT)) {
			data->pin5_float_mv = new_pin5_mv;
			data->timer_loop_cnt = 0;
			gpio_direction_output(data->gpio[GPIO_PIN6_DIR].gpio, 0);
			data->con_state = CON_STATE_PIN6_SETTLE;
		}
		break;

	case CON_STATE_PIN6_SETTLE:
		new_pin5_mv = legoev3_analog_out_pin5_value(data->analog, data->id);

		if (data->timer_loop_cnt >= SETTLE_CNT) {
			data->pin5_low_mv = new_pin5_mv;
			data->timer_loop_cnt = 0;
			gpio_direction_input(data->gpio[GPIO_PIN6_DIR].gpio);
			data->con_state = CON_STATE_CONNECTED;
			}
		break;

	case CON_STATE_CONNECTED:
		/*
		 * Make a temporary variable that we can use to determine the relative
		 * difference between pin5_float_mv and pin5_low_mv
		 */
		new_pin5_mv = ADC_REF + data->pin5_float_mv - data->pin5_low_mv;

		if ((new_pin5_mv > (ADC_REF - 50)) && (new_pin5_mv < (ADC_REF + 50))) {
			// The pin5 values are the same, let's see what we have!

			if ((data->pin5_float_mv >= PIN5_BALANCE_LOW)
				&& (data->pin5_float_mv <= PIN5_BALANCE_HIGH)
				&& (data->pin_state_flags & (0x01 << PIN_STATE_FLAG_PIN6_HIGH)))
			{
				/* NXT TOUCH SENSOR, NXT SOUND SENSOR or NEW UART SENSOR */
				data->motor_type = MOTOR_ERR;
				data->con_state = CON_STATE_WAITING_FOR_DISCONNECT;

			} else if (data->pin5_float_mv < PIN5_NEAR_GND) {
				/* NEW DUMB SENSOR */
				data->motor_type = MOTOR_ERR;
				data->con_state = CON_STATE_WAITING_FOR_DISCONNECT;

			} else if ((data->pin5_float_mv >= PIN5_LIGHT_LOW)
				&& (data->pin5_float_mv <= PIN5_LIGHT_HIGH))
			{
				/* NXT LIGHT SENSOR */
				data->motor_type = MOTOR_ERR;
				data->con_state = CON_STATE_WAITING_FOR_DISCONNECT;

			} else if ((data->pin5_float_mv >= PIN5_IIC_LOW)
				&& (data->pin5_float_mv <= PIN5_IIC_HIGH))
			{
				/* NXT IIC SENSOR */
				data->motor_type = MOTOR_ERR;
				data->con_state = CON_STATE_WAITING_FOR_DISCONNECT;

			} else if (data->pin5_float_mv < PIN5_BALANCE_LOW) {
				data->motor_type = MOTOR_TACHO;
				if (data->pin5_float_mv > PIN5_MINITACHO_HIGH2) {
					data->motor_id = LEGO_EV3_LARGE_MOTOR;
				} else if (data->pin5_float_mv > PIN5_MINITACHO_LOW2) {
					data->motor_id = LEGO_EV3_MEDIUM_MOTOR;
				} else {
					data->motor_id = LEGO_EV3_LARGE_MOTOR;
				}
				data->con_state = CON_STATE_DEVICE_CONNECTED;

			} else {
				gpio_direction_output(data->gpio[GPIO_PIN5].gpio, 1);
				data->timer_loop_cnt = 0;
				data->con_state = CON_STATE_PIN5_SETTLE;
			}

		/* Value5Float is NOT equal to Value5Low */
		} else if ((data->pin5_float_mv > PIN5_NEAR_GND)
			&& (data->pin5_float_mv < PIN5_BALANCE_LOW))
		{
			/* NEW ACTUATOR */
			data->motor_type = MOTOR_ERR;
			data->con_state = CON_STATE_WAITING_FOR_DISCONNECT;
		} else {
			data->motor_type = MOTOR_ERR;
			data->con_state = CON_STATE_WAITING_FOR_DISCONNECT;
		}
		break;

	case CON_STATE_PIN5_SETTLE:
		/* Update connection type, may need to force pin5 low to determine motor type */
		if (data->timer_loop_cnt >= SETTLE_CNT) {
			data->pin5_low_mv = legoev3_analog_out_pin5_value(data->analog, data->id);
			data->timer_loop_cnt = 0;
			gpio_direction_output(data->gpio[GPIO_PIN5].gpio, 0);

			if (data->pin5_low_mv < PIN5_MINITACHO_LOW1) {
				data->motor_type = MOTOR_ERR;
			} else {
				data->motor_type = MOTOR_TACHO;
				if (data->pin5_low_mv < PIN5_MINITACHO_HIGH1)
					data->motor_id = LEGO_EV3_MEDIUM_MOTOR;
				else
					data->motor_id = LEGO_EV3_LARGE_MOTOR;
			}

			data->con_state = CON_STATE_DEVICE_CONNECTED;
		}
		break;

	case CON_STATE_DEVICE_CONNECTED:
		data->timer_loop_cnt = 0;
		if (data->motor_type != MOTOR_ERR && !work_busy(&data->work)) {
			INIT_WORK(&data->work, ev3_output_port_register_motor);
			schedule_work(&data->work);
			data->con_state = CON_STATE_WAITING_FOR_DISCONNECT;
		}
		break;

	case CON_STATE_WAITING_FOR_DISCONNECT:
		new_pin5_mv = legoev3_analog_out_pin5_value(data->analog, data->id);

		if ((new_pin5_mv < PIN5_BALANCE_LOW) || (new_pin5_mv > PIN5_BALANCE_HIGH))
			data->timer_loop_cnt = 0;

		if ((data->timer_loop_cnt >= REMOVE_CNT) && !work_busy(&data->work) && data) {
			INIT_WORK(&data->work, ev3_output_port_unregister_motor);
			schedule_work(&data->work);
			data->con_state = CON_STATE_INIT;
		}
		break;

	default:
		data->con_state = CON_STATE_INIT;
		break;
	}
	/*
	 * data->tacho_motor_type determines the status for the lego-port class
	 * so we need to trigger a change uevent when it changes.
	 */
	if (prev_motor_type != data->motor_type)
		schedule_work(&data->change_uevent_work);

	return HRTIMER_RESTART;
}

static ssize_t pin5_mv_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct ev3_output_port_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",
			legoev3_analog_out_pin5_value(data->analog, data->id));
}

static DEVICE_ATTR_RO(pin5_mv);

static struct attribute *ev3_output_port_raw_attrs[] = {
	&dev_attr_pin5_mv.attr,
	NULL
};

ATTRIBUTE_GROUPS(ev3_output_port_raw);

void ev3_output_port_enable_raw_mode(struct ev3_output_port_data *data)
{
	int err, i;

	err = sysfs_create_groups(&data->out_port.dev.kobj,
						ev3_output_port_raw_groups);
	if (err)
		dev_err(&data->out_port.dev,
				"Failed to create raw mode sysfs groups.");
	/* TODO: would be nice to create symlinks from exported gpios to out_port */
	for (i = 0; i < NUM_GPIO; i++)
		gpio_export(data->gpio[i].gpio, true);
}

void ev3_output_port_disable_raw_mode(struct ev3_output_port_data * data)
{
	int i;

	sysfs_remove_groups(&data->out_port.dev.kobj, ev3_output_port_raw_groups);
	for (i = 0; i < NUM_GPIO; i++)
		gpio_unexport(data->gpio[i].gpio);
	ev3_output_port_float(data);
}

static int ev3_output_port_set_mode(void *context, u8 mode)
{
	struct ev3_output_port_data *data = context;

	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->work);

	if (data->motor)
		ev3_output_port_unregister_motor(&data->work);
	if (data->out_port.mode == EV3_OUTPUT_PORT_MODE_RAW)
		ev3_output_port_disable_raw_mode(data);
	switch (mode) {
	case EV3_OUTPUT_PORT_MODE_AUTO:
		data->con_state = CON_STATE_INIT;
		hrtimer_start(&data->timer, ktime_set(0, OUTPUT_PORT_POLL_NS),
			      HRTIMER_MODE_REL);
		break;
	case EV3_OUTPUT_PORT_MODE_TACHO_MOTOR:
		data->motor_type = MOTOR_TACHO;
		data->motor_id = LEGO_EV3_LARGE_MOTOR;
		ev3_output_port_register_motor(&data->work);
		break;
	case EV3_OUTPUT_PORT_MODE_DC_MOTOR:
		data->motor_type = MOTOR_DC;
		ev3_output_port_register_motor(&data->work);
		break;
	case EV3_OUTPUT_PORT_MODE_LED:
		data->motor_type = MOTOR_LED;
		ev3_output_port_register_motor(&data->work);
		break;
	case EV3_OUTPUT_PORT_MODE_RAW:
		ev3_output_port_enable_raw_mode(data);
		break;
	default:
		WARN_ON("Unknown mode.");
		break;
	}
	return 0;
}

static int ev3_output_port_set_device(void *context, const char *device_name)
{
	struct ev3_output_port_data *data = context;
	struct lego_device *new_motor;
	struct ev3_motor_platform_data pdata;

	switch (data->out_port.mode) {
	case EV3_OUTPUT_PORT_MODE_AUTO:
	case EV3_OUTPUT_PORT_MODE_TACHO_MOTOR:
		if (data->motor_type != MOTOR_TACHO)
			return -EOPNOTSUPP;

		pdata.tacho_int_gpio = data->gpio[GPIO_PIN5_INT].gpio;
		pdata.tacho_dir_gpio = data->gpio[GPIO_PIN6_DIR].gpio;

		break;
	case EV3_OUTPUT_PORT_MODE_DC_MOTOR:
	case EV3_OUTPUT_PORT_MODE_LED:
	case EV3_OUTPUT_PORT_MODE_RAW:
		return -EOPNOTSUPP;

	default:
		return -EINVAL;
	}

	lego_device_unregister(data->motor);
	data->motor = NULL;

	new_motor = lego_device_register(device_name,
		&ev3_motor_device_types[data->motor_type],
		&data->out_port, &pdata, sizeof(struct ev3_motor_platform_data));
	if (IS_ERR(new_motor))
		return PTR_ERR(new_motor);

	data->motor = new_motor;

	return 0;
}

static const char *ev3_output_port_get_status(void *context)
{
	struct ev3_output_port_data *data = context;

	if (data->out_port.mode == EV3_OUTPUT_PORT_MODE_AUTO)
		return ev3_output_port_status_names[data->motor_type];

	return legoev3_output_port_mode_info[data->out_port.mode].name;
}

static struct device_type ev3_output_port_type = {
	.name	= "legoev3-output-port",
};

struct lego_port_device
*ev3_output_port_register(struct ev3_output_port_platform_data *pdata,
			  struct device *parent)
{
	struct ev3_output_port_data *data;
	struct pwm_device *pwm;
	int err;

	if (WARN(!pdata, "Platform data is required."))
		return ERR_PTR(-EINVAL);

	data = kzalloc(sizeof(struct ev3_output_port_data), GFP_KERNEL);
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
	data->gpio[GPIO_PIN1].flags	= GPIOF_IN;
	data->gpio[GPIO_PIN1].label	= "pin1";

	data->gpio[GPIO_PIN2].gpio	= pdata->pin2_gpio;
	data->gpio[GPIO_PIN2].flags	= GPIOF_IN;
	data->gpio[GPIO_PIN2].label	= "pin2";

	data->gpio[GPIO_PIN5].gpio	= pdata->pin5_gpio;
	data->gpio[GPIO_PIN5].flags	= GPIOF_IN;
	data->gpio[GPIO_PIN5].label	= "pin5";

	data->gpio[GPIO_PIN5_INT].gpio	= pdata->pin5_int_gpio;
	data->gpio[GPIO_PIN5_INT].flags	= GPIOF_IN;
	data->gpio[GPIO_PIN5_INT].label	= "pin5_tacho";

	data->gpio[GPIO_PIN6_DIR].gpio	= pdata->pin6_dir_gpio;
	data->gpio[GPIO_PIN6_DIR].flags	= GPIOF_IN;
	data->gpio[GPIO_PIN6_DIR].label	= "pin6";

	err = gpio_request_array(data->gpio, ARRAY_SIZE(data->gpio));
	if (err) {
		dev_err(parent, "Requesting GPIOs failed.\n");
		goto err_gpio_request_array;
	}

	data->out_port.name = ev3_output_port_type.name;
	snprintf(data->out_port.address, LEGO_NAME_SIZE, "out%c",
		 data->id + 'A');
	pwm = pwm_get(NULL, data->out_port.address);
	if (IS_ERR(pwm)) {
		dev_err(parent, "Could not get pwm! (%ld)\n", PTR_ERR(pwm));
		err = PTR_ERR(pwm);
		goto err_pwm_get;
	}

	err = pwm_config(pwm, 0, NSEC_PER_SEC / 10000);
	if (err) {
		dev_err(parent,
			"Failed to set pwm duty percent and frequency! (%d)\n",
			err);
		goto err_pwm_config;
	}

	err = pwm_enable(pwm);
	if (err) {
		dev_err(parent, "Failed to start pwm! (%d)\n", err);
		goto err_pwm_start;
	}
	/* This lets us set the pwm duty cycle in an atomic context */
	pm_runtime_irq_safe(pwm->chip->dev);
	data->pwm = pwm;

	data->out_port.num_modes = NUM_EV3_OUTPUT_PORT_MODE;
	data->out_port.mode_info = legoev3_output_port_mode_info;
	data->out_port.set_mode = ev3_output_port_set_mode;
	data->out_port.set_device = ev3_output_port_set_device;
	data->out_port.get_status = ev3_output_port_get_status;
	data->out_port.dc_motor_ops = &ev3_output_port_motor_ops;
	data->out_port.context = data;
	err = lego_port_register(&data->out_port, &ev3_output_port_type, parent);
	if (err) {
		dev_err(parent, "Failed to register lego_port_device. (%d)\n",
			err);
		goto err_lego_port_register;
	}

	INIT_WORK(&data->change_uevent_work, ev3_output_port_change_uevent_work);
	INIT_WORK(&data->work, NULL);

	data->con_state = CON_STATE_INIT;

	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = ev3_output_port_timer_callback;
	hrtimer_start(&data->timer, ktime_set(0, OUTPUT_PORT_POLL_NS),
		      HRTIMER_MODE_REL);

	return &data->out_port;

err_lego_port_register:
	pwm_disable(pwm);
err_pwm_start:
err_pwm_config:
	pwm_put(pwm);
err_pwm_get:
	gpio_free_array(data->gpio, ARRAY_SIZE(data->gpio));
err_gpio_request_array:
	put_legoev3_analog(data->analog);
err_request_legoev3_analog:
	kfree(data);

	return ERR_PTR(err);
}

void ev3_output_port_unregister(struct lego_port_device *port)
{
	struct ev3_output_port_data *data;

	/* port can be null if disabled via module parameter */
	if (!port)
		return;

	data = container_of(port, struct ev3_output_port_data, out_port);
	pwm_disable(data->pwm);
	pwm_put(data->pwm);
	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->change_uevent_work);
	cancel_work_sync(&data->work);
	if (data->motor)
		ev3_output_port_unregister_motor(&data->work);
	if (port->mode == EV3_OUTPUT_PORT_MODE_RAW)
		ev3_output_port_disable_raw_mode(data);
	lego_port_unregister(&data->out_port);
	ev3_output_port_float(data);
	gpio_free_array(data->gpio, ARRAY_SIZE(data->gpio));
	put_legoev3_analog(data->analog);
	dev_set_drvdata(&port->dev, NULL);
	kfree(data);
}
