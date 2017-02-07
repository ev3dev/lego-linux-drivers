/*
 * Dexter Industries BrickPi driver
 *
 * Copyright (C) 2015-2016 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <dc_motor_class.h>
#include <tacho_motor_class.h>

#include "brickpi_internal.h"
#include "../ev3/legoev3_motor.h"

const struct device_type brickpi_out_port_type = {
	.name = "brickpi-out-port",
};
EXPORT_SYMBOL_GPL(brickpi_out_port_type);

static const struct device_type
brickpi_out_port_device_types[NUM_BRICKPI_OUT_PORT_MODES] = {
	[BRICKPI_OUT_PORT_MODE_TACHO_MOTOR] = {
		.name = "ev3-motor",
	},
	[BRICKPI_OUT_PORT_MODE_DC_MOTOR] = {
		.name = "rcx-motor",
	},
	[BRICKPI_OUT_PORT_MODE_LED] = {
		.name = "rcx-led",
	},
};

static const char * const
brickpi_out_port_default_driver[NUM_BRICKPI_OUT_PORT_MODES] = {
	[BRICKPI_OUT_PORT_MODE_TACHO_MOTOR]	= "lego-nxt-motor",
	[BRICKPI_OUT_PORT_MODE_DC_MOTOR]	= "rcx-motor",
	[BRICKPI_OUT_PORT_MODE_LED] 		= "rcx-led",
};

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new modes have the same syntax. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the ev3dev-kpkg repository.
 */

static const struct lego_port_mode_info brickpi_out_port_mode_info[NUM_BRICKPI_OUT_PORT_MODES] = {
	/**
	 * @description: Dexter Industries BrickPi Output Port
	 * @connection_types: tacho-motor, dc-motor, led
	 * @prefix: out
	 */
	[BRICKPI_OUT_PORT_MODE_TACHO_MOTOR] = {
		/**
		 * @description: NXT/EV3 Large Motor
		 */
		.name	= "tacho-motor",
	},
	[BRICKPI_OUT_PORT_MODE_DC_MOTOR] = {
		/**
		 * @description: RCX/Power Functions motor
		 */
		.name	= "dc-motor",
	},
	[BRICKPI_OUT_PORT_MODE_LED] = {
		/**
		 * @description: RCX/Power Functions LED
		 */
		.name	= "led",
	},
};

static unsigned brickpi_out_port_get_supported_commands(void *context)
{
	return BIT(DC_MOTOR_COMMAND_RUN_FOREVER) | BIT(DC_MOTOR_COMMAND_STOP);
}

static unsigned brickpi_out_port_get_supported_stop_actions(void *context)
{
	return BIT(DC_MOTOR_STOP_ACTION_COAST);
}

static enum dc_motor_internal_command brickpi_out_port_get_command(void *context)
{
	struct brickpi_out_port_data *data = context;

	if (data->motor_enabled)
		if (data->motor_reversed)
			return DC_MOTOR_INTERNAL_COMMAND_RUN_REVERSE;
		return DC_MOTOR_INTERNAL_COMMAND_RUN_FORWARD;

	return DC_MOTOR_INTERNAL_COMMAND_COAST;
}

static int brickpi_out_port_set_command(void *context,
				        enum dc_motor_internal_command command)
{
	struct brickpi_out_port_data *data = context;

	switch(command) {
	case DC_MOTOR_INTERNAL_COMMAND_RUN_FORWARD:
		data->motor_enabled = true;
		data->motor_reversed = false;
		break;
	case DC_MOTOR_INTERNAL_COMMAND_RUN_REVERSE:
		data->motor_enabled = true;
		data->motor_reversed = true;
		break;
	default:
		data->motor_enabled = false;
		data->motor_reversed = false;
		break;
	}

	return 0;
}

static unsigned brickpi_out_port_get_duty_cycle(void *context)
{
	struct brickpi_out_port_data *data = context;

	return BRICKPI_DUTY_RAW_TO_PCT(data->motor_speed);
}

static int brickpi_out_port_set_duty_cycle(void *context, unsigned duty)
{
	struct brickpi_out_port_data *data = context;

	if (duty > 100)
		return -EINVAL;

	data->motor_speed = BRICKPI_DUTY_PCT_TO_RAW(duty);

	return 0;
}

static struct dc_motor_ops brickpi_out_port_dc_motor_ops = {
	.get_supported_commands	= brickpi_out_port_get_supported_commands,
	.get_supported_stop_actions = brickpi_out_port_get_supported_stop_actions,
	.get_command		= brickpi_out_port_get_command,
	.set_command		= brickpi_out_port_set_command,
	.set_duty_cycle		= brickpi_out_port_set_duty_cycle,
	.get_duty_cycle		= brickpi_out_port_get_duty_cycle,
};

static int brickpi_out_port_get_position(void *context, int *position)
{
	struct brickpi_out_port_data *data = context;

	*position = data->motor_position + data->motor_offset;

	return 0;
}

static int brickpi_out_port_set_position(void *context, int position)
{
	struct brickpi_out_port_data *data = context;

	if (data->motor_enabled)
		return -EBUSY;

	data->motor_offset = position - data->motor_position;

	return 0;
}

static int brickpi_out_port_get_duty_cycle2(void *context, int *duty_cycle)
{
	struct brickpi_out_port_data *data = context;

	*duty_cycle = BRICKPI_DUTY_PCT_TO_RAW(data->motor_speed);

	return 0;
}

static int brickpi_out_port_run_unregulated(void *context, int duty_cycle)
{
	struct brickpi_out_port_data *data = context;

	mutex_lock(&data->ch_data->data->tx_mutex);
	data->speed_pid_ena = false;
	data->hold_pid_ena = false;
	data->stop_at_target_position = false;
	data->direct_duty_cycle = duty_cycle;
	data->motor_enabled = true;
	mutex_unlock(&data->ch_data->data->tx_mutex);

	return 0;
}

TM_SPEED_GET_SPEED_FUNC(brickpi_out_port, brickpi_out_port_data, speed);

static int brickpi_out_port_run_regulated(void *context, int speed)
{
	struct brickpi_out_port_data *data = context;

	mutex_lock(&data->ch_data->data->tx_mutex);
	data->speed_pid_ena = true;
	data->hold_pid_ena = false;
	data->stop_at_target_position = false;
	data->speed_pid.setpoint = speed;
	data->motor_enabled = true;
	mutex_unlock(&data->ch_data->data->tx_mutex);

	return 0;
}

static int brickpi_out_port_run_to_pos(void *context, int pos, int speed,
				       enum tm_stop_action stop_action)
{
	struct brickpi_out_port_data *data = context;

	mutex_lock(&data->ch_data->data->tx_mutex);
	speed = abs(speed);
	if (data->motor_position > pos)
		speed *= -1;
	data->speed_pid_ena = true;
	data->hold_pid_ena = false;
	data->stop_at_target_position = true;
	data->speed_pid.setpoint = speed;
	data->target_position = pos - data->motor_offset;
	data->stop_action = stop_action;
	data->motor_enabled = true;
	mutex_unlock(&data->ch_data->data->tx_mutex);

	return 0;
}

TM_PID_GET_FUNC(brickpi_out_port, speed_Kp, brickpi_out_port_data, speed_pid.Kp);
TM_PID_SET_FUNC(brickpi_out_port, speed_Kp, brickpi_out_port_data, speed_pid.Kp);
TM_PID_GET_FUNC(brickpi_out_port, speed_Ki, brickpi_out_port_data, speed_pid.Ki);
TM_PID_SET_FUNC(brickpi_out_port, speed_Ki, brickpi_out_port_data, speed_pid.Ki);
TM_PID_GET_FUNC(brickpi_out_port, speed_Kd, brickpi_out_port_data, speed_pid.Kd);
TM_PID_SET_FUNC(brickpi_out_port, speed_Kd, brickpi_out_port_data, speed_pid.Kd);
TM_PID_GET_FUNC(brickpi_out_port, hold_Kp, brickpi_out_port_data, hold_pid.Kp);
TM_PID_SET_FUNC(brickpi_out_port, hold_Kp, brickpi_out_port_data, hold_pid.Kp);
TM_PID_GET_FUNC(brickpi_out_port, hold_Ki, brickpi_out_port_data, hold_pid.Ki);
TM_PID_SET_FUNC(brickpi_out_port, hold_Ki, brickpi_out_port_data, hold_pid.Ki);
TM_PID_GET_FUNC(brickpi_out_port, hold_Kd, brickpi_out_port_data, hold_pid.Kd);
TM_PID_SET_FUNC(brickpi_out_port, hold_Kd, brickpi_out_port_data, hold_pid.Kd);

static int brickpi_out_port_get_state(void *context)
{
	struct brickpi_out_port_data *data = context;
	unsigned state = 0;

	if (data->motor_enabled) {
		if (data->hold_pid_ena) {
			state |= BIT(TM_STATE_HOLDING);
			if (tm_pid_is_overloaded(&data->hold_pid))
				state |= BIT(TM_STATE_OVERLOADED);
		} else {
			state |= BIT(TM_STATE_RUNNING);
			if (tm_pid_is_overloaded(&data->speed_pid))
				state |= BIT(TM_STATE_OVERLOADED);
			if (tm_speed_get(&data->speed) == 0)
				state |= BIT(TM_STATE_STALLED);
		}
	}

	return state;
}

void _brickpi_out_port_stop(struct brickpi_out_port_data *data)
{
	data->speed_pid_ena = false;
	if (data->stop_action == TM_STOP_ACTION_HOLD) {
		data->hold_pid_ena = true;
		if (data->stop_at_target_position)
			data->hold_pid.setpoint = data->target_position;
		else
			data->hold_pid.setpoint = data->motor_position;
	} else
		data->hold_pid_ena = false;
	data->motor_speed = 0;
	data->motor_reversed = false;
	data->motor_enabled = data->hold_pid_ena;
	data->stop_at_target_position = false;
	tm_pid_reinit(&data->speed_pid);
}

static int brickpi_out_port_stop(void *context, enum tm_stop_action stop_action)
{
	struct brickpi_out_port_data *data = context;

	mutex_lock(&data->ch_data->data->tx_mutex);
	data->stop_action = stop_action;
	_brickpi_out_port_stop(data);
	mutex_unlock(&data->ch_data->data->tx_mutex);

	return 0;
}

void _brickpi_out_port_reset(struct brickpi_out_port_data *data)
{
	data->speed_pid_ena = false;
	data->hold_pid_ena = false;
	data->motor_speed = 0;
	data->motor_reversed = false;
	data->motor_enabled = false;
	data->stop_at_target_position = false;
	data->motor_offset = -data->motor_position;
	tm_pid_init(&data->speed_pid, 1000, 60, 0);
	tm_pid_init(&data->hold_pid, 20000, 0, 0);
}

static int brickpi_out_port_reset(void *context)
{
	struct brickpi_out_port_data *data = context;

	mutex_lock(&data->ch_data->data->tx_mutex);
	_brickpi_out_port_reset(data);
	mutex_unlock(&data->ch_data->data->tx_mutex);

	return 0;
}


static unsigned brickpi_out_port_get_stop_actions(void *context)
{
	return BIT(TM_STOP_ACTION_COAST) | BIT(TM_STOP_ACTION_HOLD);
}

struct tacho_motor_ops brickpi_out_port_tacho_motor_ops = {
	.get_position		= brickpi_out_port_get_position,
	.set_position		= brickpi_out_port_set_position,
	.get_duty_cycle		= brickpi_out_port_get_duty_cycle2,
	.get_speed		= brickpi_out_port_get_speed,
	.run_unregulated	= brickpi_out_port_run_unregulated,
	.run_regulated		= brickpi_out_port_run_regulated,
	.run_to_pos		= brickpi_out_port_run_to_pos,
	.stop			= brickpi_out_port_stop,
	.reset			= brickpi_out_port_reset,
	.get_speed_Kp		= brickpi_out_port_get_speed_Kp,
	.set_speed_Kp		= brickpi_out_port_set_speed_Kp,
	.get_speed_Ki		= brickpi_out_port_get_speed_Ki,
	.set_speed_Ki		= brickpi_out_port_set_speed_Ki,
	.get_speed_Kd		= brickpi_out_port_get_speed_Kd,
	.set_speed_Kd		= brickpi_out_port_set_speed_Kd,
	.get_hold_Kp		= brickpi_out_port_get_hold_Kp,
	.set_hold_Kp		= brickpi_out_port_set_hold_Kp,
	.get_hold_Ki		= brickpi_out_port_get_hold_Ki,
	.set_hold_Ki		= brickpi_out_port_set_hold_Ki,
	.get_hold_Kd		= brickpi_out_port_get_hold_Kd,
	.set_hold_Kd		= brickpi_out_port_set_hold_Kd,
	.get_state		= brickpi_out_port_get_state,
	.get_stop_actions	= brickpi_out_port_get_stop_actions,
};

int brickpi_out_port_register_motor(struct brickpi_out_port_data *out_port,
				    const struct device_type *device_type,
				    const char *name)
{
	struct lego_device *new_motor;

	new_motor = lego_device_register(name, device_type,
					 &out_port->port, NULL, 0);
	if (IS_ERR(new_motor))
		return PTR_ERR(new_motor);

	out_port->motor = new_motor;

	return 0;
}

void brickpi_out_port_unregister_motor(struct brickpi_out_port_data *out_port)
{
	if (out_port->motor) {
		lego_device_unregister(out_port->motor);
		out_port->motor = NULL;
		out_port->motor_enabled = false;
		brickpi_get_values(out_port->ch_data);
	}
}

static int brickpi_out_port_set_mode(void *context, u8 mode)
{
	struct brickpi_out_port_data *out_port = context;

	brickpi_out_port_unregister_motor(out_port);

	return brickpi_out_port_register_motor(out_port,
				&brickpi_out_port_device_types[mode],
				brickpi_out_port_default_driver[mode]);
}

int brickpi_register_out_ports(struct brickpi_channel_data *ch_data,
			       struct device *parent)
{
	int i, err;

	for (i = 0; i < NUM_BRICKPI_PORT; i++) {
		struct lego_port_device *port = &ch_data->out_port[i].port;
		port->name = brickpi_out_port_type.name;
		snprintf(port->address, LEGO_NAME_SIZE, "%s:M%c",
			 dev_name(parent), ch_data->address * 2 + i - 2 + 'A');
		port->num_modes = NUM_BRICKPI_OUT_PORT_MODES;
		port->supported_modes = LEGO_PORT_ALL_MODES;
		port->mode_info = brickpi_out_port_mode_info;
		port->set_mode = brickpi_out_port_set_mode;
		port->dc_motor_ops = &brickpi_out_port_dc_motor_ops;
		port->tacho_motor_ops = &brickpi_out_port_tacho_motor_ops;
		port->context = &ch_data->out_port[i];

		err = lego_port_register(port, &brickpi_out_port_type, parent);
		if (!err)
			err = brickpi_out_port_set_mode(&ch_data->out_port[i],
					BRICKPI_OUT_PORT_MODE_TACHO_MOTOR);
		if (err) {
			dev_err(parent,
				"Failed to register BrickPi output port. (%d)\n",
				err);
			for (i--; i >= 0; i--) {
				brickpi_out_port_unregister_motor(&ch_data->out_port[i]);
				lego_port_unregister(&ch_data->out_port[i].port);
			}
			return err;
		}
	}

	return 0;
}

void brickpi_unregister_out_ports(struct brickpi_channel_data *ch_data)
{
	int i;

	for (i = 0; i < NUM_BRICKPI_PORT; i++) {
		brickpi_out_port_unregister_motor(&ch_data->out_port[i]);
		lego_port_unregister(&ch_data->out_port[i].port);
	}
}
