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

static const char *brickpi_out_port_default_driver[NUM_BRICKPI_OUT_PORT_MODES] = {
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

static unsigned brickpi_out_port_get_supported_commands(void* context)
{
	return BIT(DC_MOTOR_COMMAND_RUN_FOREVER) | BIT(DC_MOTOR_COMMAND_STOP);
}

static unsigned brickpi_out_port_get_supported_stop_commands(void* context)
{
	return BIT(DC_MOTOR_STOP_COMMAND_COAST);
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

	return data->motor_speed * 100 / 255;
}

static int brickpi_out_port_set_duty_cycle(void *context, unsigned duty)
{
	struct brickpi_out_port_data *data = context;

	if (duty > 100)
		return -EINVAL;

	data->motor_speed = duty * 255 / 100;

	return 0;
}

static struct dc_motor_ops brickpi_out_port_dc_motor_ops = {
	.get_supported_commands	= brickpi_out_port_get_supported_commands,
	.get_supported_stop_commands = brickpi_out_port_get_supported_stop_commands,
	.get_command		= brickpi_out_port_get_command,
	.set_command		= brickpi_out_port_set_command,
	.set_duty_cycle		= brickpi_out_port_set_duty_cycle,
	.get_duty_cycle		= brickpi_out_port_get_duty_cycle,
};

static int brickpi_out_port_get_position(void *context, long *position)
{
	struct brickpi_out_port_data *data = context;

	*position = data->motor_position + data->motor_offset;

	return 0;
}

static int brickpi_out_port_set_position(void *context, long position)
{
	struct brickpi_out_port_data *data = context;

	if (data->motor_enabled)
		return -EBUSY;

	data->motor_offset = position - data->motor_position;

	return 0;
}

static int brickpi_out_port_get_state(void *context)
{
	struct brickpi_out_port_data *data = context;
	unsigned state = 0;

	if (data->motor_enabled)
		state |= BIT(TM_STATE_RUNNING);

	return state;
}

static int brickpi_out_port_get_count_per_rot(void *context)
{
	return 720;
}

static unsigned brickpi_out_port_get_commands(void *context)
{
	return BIT(TM_COMMAND_RUN_FOREVER) | BIT(TM_COMMAND_RUN_TO_ABS_POS)
		| BIT(TM_COMMAND_RUN_TO_REL_POS) | BIT(TM_COMMAND_STOP)
		| BIT(TM_COMMAND_RESET);
}

static int brickpi_out_port_send_command(void *context,
					 struct tacho_motor_params *params,
					 enum tacho_motor_command command)
{
	struct brickpi_out_port_data *data = context;

	mutex_lock(&data->ch_data->data->tx_mutex);
	if (IS_RUN_CMD(command)) {
		int duty_cycle_sp = params->duty_cycle_sp;
		int position_sp = params->position_sp;
		if (params->polarity == DC_MOTOR_POLARITY_INVERSED) {
			duty_cycle_sp *= -1;
			position_sp *= -1;
		}
		data->motor_speed = abs(params->duty_cycle_sp) * 255 / 100;
		if (IS_POS_CMD(command)) {
			if (command == TM_COMMAND_RUN_TO_ABS_POS)
				data->target_position = position_sp - data->motor_offset;
			else if (command == TM_COMMAND_RUN_TO_REL_POS)
				data->target_position = position_sp + data->motor_position;
			data->motor_reversed =
				(data->target_position < data->motor_position);
			data->stop_at_target_position = true;
		} else {
			data->motor_reversed = (params->duty_cycle_sp < 0);
			data->stop_at_target_position = false;
		}
		data->motor_enabled = true;
	} else {
		data->motor_speed = 0;
		data->motor_reversed = false;
		data->motor_enabled = false;
		data->stop_at_target_position = false;
		if (command == TM_COMMAND_RESET)
			data->motor_offset = data->motor_position;
	}
	mutex_unlock(&data->ch_data->data->tx_mutex);

	return 0;
}

static unsigned brickpi_out_port_get_speed_regulations(void *context)
{
	/*
	 * The BrickPi firmware does not have speed regulation.
	 */
	return BIT(TM_SPEED_REGULATION_OFF);
}

static unsigned brickpi_out_port_get_stop_commands(void *context)
{
	return BIT(TM_STOP_COMMAND_COAST);
}

struct tacho_motor_ops brickpi_out_port_tacho_motor_ops = {
	.get_position		= brickpi_out_port_get_position,
	.set_position		= brickpi_out_port_set_position,
	.get_state		= brickpi_out_port_get_state,
	.get_count_per_rot	= brickpi_out_port_get_count_per_rot,
	.get_commands		= brickpi_out_port_get_commands,
	.send_command		= brickpi_out_port_send_command,
	.get_speed_regulations	= brickpi_out_port_get_speed_regulations,
	.get_stop_commands	= brickpi_out_port_get_stop_commands,
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
