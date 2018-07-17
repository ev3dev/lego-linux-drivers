/*
 * Output port driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2013-2014 Ralph Hempel <rhempel@hempeldesigngroup.com>
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
 * The EV3 has four output ports labeled A, B, C and D. These ports
 * are used with LEGO MINDSTORMS EV3, NXT and RCX compatible motors and other
 * devices. EV3 motors can be automatically detected when plugged in. Other
 * motors with position feedback (such as the NXT motor) are usually detected
 * as an EV3 Large Motor. RCX motors and LEDs cannot be automatically detected
 * and the mode of the output port must be manually set to use these types of
 * devices.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <lego.h>
#include <lego_port_class.h>
#include <dc_motor_class.h>

#include "ev3_ports.h"
#include "../motors/ev3_motor.h"

#define OUTPUT_PORT_POLL_NS	10000000			  /*  10 msec */
#define SETTLE_CNT		(20000000 / OUTPUT_PORT_POLL_NS)  /*  20 msec */
#define ADD_CNT			(350000000 / OUTPUT_PORT_POLL_NS) /* 350 msec */
#define REMOVE_CNT		(100000000 / OUTPUT_PORT_POLL_NS) /* 100 msec */

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
	PIN_STATE_FLAG_PIN6_LOW,
	NUM_PIN_STATE_FLAG
};

enum ev3_output_port_mode {
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
 * parser can be found in the Documentation/json/ directory.
 */

static const struct lego_port_mode_info ev3_output_port_mode_info[] = {
	/**
	 * @description: EV3 Output Port
	 * @module: ev3-ports
	 * @connection_types: tacho-motor, dc-motor, led
	 * @prefix: out
	 * @module: ev3_ports
	 */
	[EV3_OUTPUT_PORT_MODE_AUTO] = {
		/**
		 * .. [#out-port-auto-mode] Only the EV3/NXT large motors and
		 *    the EV3 medium motor can be automatically detected. All
		 *    other devices must be manually configured.
		 *
		 * @description: Automatically detect motors when they are connected.
		 * @name_footnote: [#out-port-auto-mode]_
		 */
		.name	= "auto",
	},
	[EV3_OUTPUT_PORT_MODE_TACHO_MOTOR] = {
		/**
		 * .. [#out-port-tacho-motor-mode] Configures the port to use
		 *    the :ref:`tacho-motor-class`. The default driver is
		 *    the EV3 Large Motor (``lego-ev3-l-motor``). You can change
		 *    the driver using the ``set_device`` attribute.
		 *
		 * @description: Configure the port for NXT/EV3 motors.
		 * @name_footnote: [#out-port-tacho-motor-mode]_
		 */
		.name	= "tacho-motor",
	},
	[EV3_OUTPUT_PORT_MODE_DC_MOTOR] = {
		/**
		 * .. [#out-port-dc-motor-mode] This can be use with MINDSTORMS
		 *    RCX motors, Power Functions motors and any other "plain"
		 *    DC motor. By "plain", we mean the motor is just a motor
		 *    without any feedback.
		 *
		 * @description: Load the port for RCX/Power Functions motors.
		 * @name_footnote: [#out-port-dc-motor-mode]_
		 */
		.name	= "dc-motor",
	},
	[EV3_OUTPUT_PORT_MODE_LED] = {
		/**
		 * .. [#out-port-led-mode] This can be used with MINDSTORMS RCX
		 *    LEDs, Power Functions LEDs or any other LED connected to
		 *    pins 1 and 2 of the output port.
		 *
		 * @description: Load the port for RCX/Power Functions LEDs.
		 * @name_footnote: [#out-port-led-mode]_
		 */
		.name	= "led",
	},
	[EV3_OUTPUT_PORT_MODE_RAW] = {
		/**
		 * .. [#out-port-raw-mode] Exports gpios, pwm and analog/digital
		 *    converter values to sysfs so that they can be controlled
		 *    directly.
		 *
		 * @description: Provide access to low level drivers.
		 * @name_footnote: [#out-port-raw-mode]_
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

static const char * const ev3_output_port_status_names[] = {
	[MOTOR_NONE]	= "no-motor",
	[MOTOR_TACHO]	= ev3_output_port_mode_info[MOTOR_TACHO].name,
	[MOTOR_DC]	= ev3_output_port_mode_info[MOTOR_DC].name,
	[MOTOR_LED]	= ev3_output_port_mode_info[MOTOR_LED].name,
	[MOTOR_ERR]	= "error",
};

/**
 * struct ev3_output_port_data - Driver data for an output port on the EV3 brick
 * @id: Unique identifier for the port.
 * @out_port: lego-port class device that represents this port.
 * @iio_cb: IIO callback buffer for analog inputs.
 * @pwm: Pointer to the pwm device that is bound to this port
 * @pin1_gpio: Pin 1 motor controller input.
 * @pin2_gpio: Pin 2 motor controller input.
 * @pin5_det_gpio: Pin 5 detection shorting transistor.
 * @pin5_gpio: Pin 5 encoder interrupt.
 * @pin6_gpio: Pin 6 encoder direction.
 * @change_uevent_work: Needed when change is triggered in atomic context.
 * @work: Worker for registering and unregistering sensors when they are
 *	connected and disconnected.
 * @timer: Polling timer to monitor the port connect/disconnect.
 * @timer_loop_cnt: Used to measure time in the polling loop.
 * @con_state: The current state of the port.
 * @pin_state_flags: Used in the polling loop to track certain changes in the
 *	state of the port's pins.
 * @pin5_mv: Cached value for analog input on pin 5.
 * @pin5_float_mv: Used in the polling loop to track pin 5 voltage.
 * @pin5_low_mv: Used in the polling loop to track pin 5 voltage.
 * @motor_type: The type of motor that was detected or manually set by the mode.
 * @tacho_motor_type: The type of tacho motor that was detected.
 * @motor: Pointer to the motor device that is connected to the output port.
 * @command: The current command for the motor driver of the output port.
 * @debug: Handle to debugfs entry.
 */
struct ev3_output_port_data {
	struct lego_port_device out_port;
	struct iio_cb_buffer *iio_cb;
	struct pwm_device *pwm;
	struct gpio_desc *pin1_gpio;
	struct gpio_desc *pin2_gpio;
	struct gpio_desc *pin5_det_gpio;
	struct gpio_desc *pin5_gpio;
	struct gpio_desc *pin6_gpio;
	struct work_struct change_uevent_work;
	struct work_struct work;
	struct hrtimer timer;
	unsigned timer_loop_cnt;
	enum connection_state con_state;
	unsigned pin_state_flags:NUM_PIN_STATE_FLAG;
	unsigned pin5_mv;
	unsigned pin5_float_mv;
	unsigned pin5_low_mv;
	enum motor_type motor_type;
	enum ev3_motor_id motor_id;
	struct lego_device *motor;
	enum dc_motor_internal_command command;
	struct dentry *debug;
};

int ev3_output_port_set_direction_gpios(struct ev3_output_port_data *data)
{
	switch(data->command) {
	case DC_MOTOR_INTERNAL_COMMAND_RUN_FORWARD:
		gpiod_direction_output(data->pin1_gpio, 1);
		gpiod_direction_input(data->pin2_gpio);
		break;
	case DC_MOTOR_INTERNAL_COMMAND_RUN_REVERSE:
		gpiod_direction_input(data->pin1_gpio);
		gpiod_direction_output(data->pin2_gpio, 1);
		break;
	case DC_MOTOR_INTERNAL_COMMAND_BRAKE:
		gpiod_direction_output(data->pin1_gpio, 1);
		gpiod_direction_output(data->pin2_gpio, 1);
		break;
	case DC_MOTOR_INTERNAL_COMMAND_COAST:
		gpiod_direction_output(data->pin1_gpio, 0);
		gpiod_direction_output(data->pin2_gpio, 0);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static unsigned ev3_ouput_port_get_supported_commands(void *context)
{
	return BIT(DC_MOTOR_COMMAND_RUN_FOREVER) | BIT(DC_MOTOR_COMMAND_RUN_DIRECT)
		| BIT(DC_MOTOR_COMMAND_STOP);
}

static unsigned ev3_ouput_port_get_supported_stop_actions(void *context)
{
	return BIT(DC_MOTOR_STOP_ACTION_COAST) | BIT(DC_MOTOR_STOP_ACTION_BRAKE);
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
	struct pwm_state state;

	pwm_get_state(data->pwm, &state);

	return pwm_get_relative_duty_cycle(&state, 100);
}

static int ev3_output_port_set_duty_cycle(void *context, unsigned duty)
{
	struct ev3_output_port_data *data = context;
	struct pwm_state state;
	int ret;
	
	pwm_get_state(data->pwm, &state);
	ret = pwm_set_relative_duty_cycle(&state, duty, 100);
	if (ret)
		return ret;

	return pwm_apply_state(data->pwm, &state);
}

static struct dc_motor_ops ev3_output_port_motor_ops = {
	.get_supported_commands	= ev3_ouput_port_get_supported_commands,
	.get_supported_stop_actions = ev3_ouput_port_get_supported_stop_actions,
	.get_command		= ev3_output_port_get_command,
	.set_command		= ev3_output_port_set_command,
	.set_duty_cycle		= ev3_output_port_set_duty_cycle,
	.get_duty_cycle		= ev3_output_port_get_duty_cycle,
};

void ev3_output_port_float(struct ev3_output_port_data *data)
{
	gpiod_direction_output(data->pin1_gpio, 0);
	gpiod_direction_output(data->pin2_gpio, 0);
	gpiod_direction_output(data->pin5_det_gpio, 0);
	gpiod_direction_input(data->pin5_gpio);
	gpiod_direction_input(data->pin6_gpio);
	data->command = DC_MOTOR_INTERNAL_COMMAND_COAST;
}

void ev3_output_port_change_uevent_work(struct work_struct *work)
{
	struct ev3_output_port_data *data = container_of(work,
			struct ev3_output_port_data, change_uevent_work);

	kobject_uevent(&data->out_port.dev.kobj, KOBJ_CHANGE);
}

void ev3_output_port_register_motor(struct work_struct *work)
{
	struct ev3_output_port_data *data =
			container_of(work, struct ev3_output_port_data, work);
	struct lego_device *motor;
	const char *driver_name;

	if (data->motor_type >= NUM_MOTOR
	    || !ev3_motor_device_types[data->motor_type].name)
	{
		dev_err(&data->out_port.dev,
			"Trying to register an invalid motor type on %s.\n",
			dev_name(&data->out_port.dev));
		return;
	}

	if (data->motor_type == MOTOR_TACHO)
		driver_name = ev3_motor_defs[data->motor_id].name;
	else
		driver_name = ev3_motor_device_types[data->motor_type].name;

	motor = lego_device_register(driver_name,
		&ev3_motor_device_types[data->motor_type],
		&data->out_port, NULL, 0);
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
		new_pin5_mv = data->pin5_mv;

		if (!gpiod_get_value(data->pin6_gpio))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN6_LOW);
		if ((new_pin5_mv < PIN5_BALANCE_LOW) || (new_pin5_mv > PIN5_BALANCE_HIGH))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN5_LOADED);

		if (new_pin_state_flags != data->pin_state_flags) {
			data->pin_state_flags = new_pin_state_flags;
			data->timer_loop_cnt = 0;
		}

		if (data->pin_state_flags && (data->timer_loop_cnt >= ADD_CNT)) {
			data->pin5_float_mv = new_pin5_mv;
			data->timer_loop_cnt = 0;
			gpiod_direction_output(data->pin6_gpio, 1);
			data->con_state = CON_STATE_PIN6_SETTLE;
		}
		break;

	case CON_STATE_PIN6_SETTLE:
		new_pin5_mv = data->pin5_mv;

		if (data->timer_loop_cnt >= SETTLE_CNT) {
			data->pin5_low_mv = new_pin5_mv;
			data->timer_loop_cnt = 0;
			gpiod_direction_input(data->pin6_gpio);
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
				&& (data->pin_state_flags & (0x01 << PIN_STATE_FLAG_PIN6_LOW)))
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
				gpiod_direction_output(data->pin5_det_gpio, 1);
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
			data->pin5_low_mv = data->pin5_mv;
			data->timer_loop_cnt = 0;
			gpiod_direction_output(data->pin5_det_gpio, 0);

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
		new_pin5_mv = data->pin5_mv;

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

void ev3_output_port_enable_raw_mode(struct ev3_output_port_data *data)
{
	/* TODO: would be nice to create symlinks from exported gpios to out_port */
	gpiod_export(data->pin1_gpio, true);
	gpiod_export(data->pin2_gpio, true);
	gpiod_export(data->pin5_det_gpio, false);
	gpiod_export(data->pin5_gpio, true);
	gpiod_export(data->pin6_gpio, true);
	/* TODO: export pwm and iio */
}

void ev3_output_port_disable_raw_mode(struct ev3_output_port_data * data)
{
	gpiod_unexport(data->pin1_gpio);
	gpiod_unexport(data->pin2_gpio);
	gpiod_unexport(data->pin5_det_gpio);
	gpiod_unexport(data->pin5_gpio);
	gpiod_unexport(data->pin6_gpio);
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

	if (data->motor_type != MOTOR_TACHO)
		return -EOPNOTSUPP;

	lego_device_unregister(data->motor);
	data->motor = NULL;

	new_motor = lego_device_register(device_name,
		&ev3_motor_device_types[data->motor_type],
		&data->out_port, NULL, 0);
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

	return ev3_output_port_mode_info[data->out_port.mode].name;
}

static struct device_type ev3_output_port_type = {
	.name	= "ev3-output-port",
};

static int ev3_output_port_buf_cb(const void *buf_data, void *private)
{
	struct ev3_output_port_data *data = private;
	const u16 *raw = buf_data;

	/*
	 * Making some assumptions about the data format here. This works for
	 * the ti-ads7957 driver, but won't work for just any old iio channels.
	 * To do this properly, we should be reading the iio_channel structs
	 * to determine how to properly decode the data.
	 */
	data->pin5_mv = ((raw[0] & 0xFFF) * 5005) >> 12;

	return 0;
}

static void ev3_output_port_debug_init(struct ev3_output_port_data *data)
{
	data->debug = debugfs_create_dir(data->out_port.address, ev3_ports_debug);
	if (IS_ERR_OR_NULL(data->debug))
		return;

	debugfs_create_u32("con_state", 0444, data->debug, &data->con_state);
	debugfs_create_u32("command", 0444, data->debug, &data->command);
	debugfs_create_u32("pin5_mv", 0444, data->debug, &data->pin5_mv);
	debugfs_create_u32("pin5_float_mv", 0444, data->debug, &data->pin5_float_mv);
	debugfs_create_u32("pin5_low_mv", 0444, data->debug, &data->pin5_low_mv);
	debugfs_create_u32("pwm_duty_cycle", 0444, data->debug,
			   &data->pwm->state.duty_cycle);
	debugfs_create_u32("pwm_period", 0444, data->debug, &data->pwm->state.period);
}

int ev3_output_port_probe(struct platform_device *pdev)
{
	struct ev3_output_port_data *data;
	int err;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->iio_cb = iio_channel_get_all_cb(&pdev->dev, ev3_output_port_buf_cb,
					      data);
	if (IS_ERR(data->iio_cb)) {
		err = PTR_ERR(data->iio_cb);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Could not get iio channel callbacks.\n");
		return err;
	}

	/*
	 * TODO: Could confirm that there is only one channel and that it is
	 * named "pin5"
	 */

	err = iio_channel_start_all_cb(data->iio_cb);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to start iio callbacks.\n");
		goto err_release_iio_cb;
	}

	data->pin1_gpio = devm_gpiod_get(&pdev->dev, "pin1", GPIOD_OUT_LOW);
	if (IS_ERR(data->pin1_gpio)) {
		err = PTR_ERR(data->pin1_gpio);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Pin 1 gpio is required.\n");
		goto err_stop_iio_cb;
	}
	data->pin2_gpio = devm_gpiod_get(&pdev->dev, "pin2", GPIOD_OUT_LOW);
	if (IS_ERR(data->pin2_gpio)) {
		err = PTR_ERR(data->pin2_gpio);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Pin 2 gpio is required.\n");
		goto err_stop_iio_cb;
	}
	data->pin5_det_gpio = devm_gpiod_get(&pdev->dev, "pin5-det", GPIOD_OUT_LOW);
	if (IS_ERR(data->pin5_det_gpio)) {
		err = PTR_ERR(data->pin5_det_gpio);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Pin 5 detect gpio is required.\n");
		goto err_stop_iio_cb;
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

	data->pwm = devm_pwm_get(&pdev->dev, "motor");
	if (IS_ERR(data->pwm)) {
		err = PTR_ERR(data->pwm);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Could not get pwm.\n");
		goto err_stop_iio_cb;
	}

	pwm_apply_args(data->pwm);

	err = pwm_enable(data->pwm);
	if (err) {
		dev_err(&pdev->dev, "Failed to start pwm.\n");
		goto err_stop_iio_cb;
	}
	/* This lets us set the pwm duty cycle in an atomic context */
	pm_runtime_irq_safe(data->pwm->chip->dev);

	data->out_port.name = ev3_output_port_type.name;
	snprintf(data->out_port.address, LEGO_NAME_SIZE, "%s",
		 dev_name(&pdev->dev));
	data->out_port.num_modes = NUM_EV3_OUTPUT_PORT_MODE;
	data->out_port.supported_modes = LEGO_PORT_ALL_MODES;
	data->out_port.mode_info = ev3_output_port_mode_info;
	data->out_port.set_mode = ev3_output_port_set_mode;
	data->out_port.set_device = ev3_output_port_set_device;
	data->out_port.get_status = ev3_output_port_get_status;
	data->out_port.dc_motor_ops = &ev3_output_port_motor_ops;
	data->out_port.context = data;
	dev_set_drvdata(&pdev->dev, data);

	err = lego_port_register(&data->out_port, &ev3_output_port_type,
				 &pdev->dev);
	if (err) {
		dev_err(&pdev->dev, "Failed to register lego_port_device.\n");
		goto err_disable_pwm;
	}

	INIT_WORK(&data->change_uevent_work, ev3_output_port_change_uevent_work);
	INIT_WORK(&data->work, NULL);

	data->con_state = CON_STATE_INIT;

	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = ev3_output_port_timer_callback;
	hrtimer_start(&data->timer, ktime_set(0, OUTPUT_PORT_POLL_NS),
		      HRTIMER_MODE_REL);

	if (!IS_ERR_OR_NULL(ev3_ports_debug))
		ev3_output_port_debug_init(data);

	return 0;

err_disable_pwm:
	pwm_disable(data->pwm);
err_stop_iio_cb:
	iio_channel_stop_all_cb(data->iio_cb);
err_release_iio_cb:
	iio_channel_release_all_cb(data->iio_cb);

	return err;
}

int ev3_output_port_remove(struct platform_device *pdev)
{
	struct ev3_output_port_data *data = dev_get_drvdata(&pdev->dev);

	debugfs_remove_recursive(data->debug);

	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->change_uevent_work);
	cancel_work_sync(&data->work);
	if (data->motor)
		ev3_output_port_unregister_motor(&data->work);
	if (data->out_port.mode == EV3_OUTPUT_PORT_MODE_RAW)
		ev3_output_port_disable_raw_mode(data);
	lego_port_unregister(&data->out_port);
	ev3_output_port_float(data);
	pwm_disable(data->pwm);
	iio_channel_stop_all_cb(data->iio_cb);
	iio_channel_release_all_cb(data->iio_cb);

	return 0;
}

static const struct of_device_id ev3_output_port_dt_ids[] = {
	{ .compatible = "ev3dev,ev3-output-port", },
	{ }
};
MODULE_DEVICE_TABLE(of, ev3_output_port_dt_ids);

static struct platform_driver ev3_output_port_driver = {
	.driver	= {
		.name		= "ev3-output-port",
		.of_match_table = ev3_output_port_dt_ids,
	},
	.probe	= ev3_output_port_probe,
	.remove	= ev3_output_port_remove,
};
module_platform_driver(ev3_output_port_driver);

MODULE_DESCRIPTION("Support for LEGO MINDSTORMS EV3 output ports.");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ev3-output-port");
