/*
 * Dexter Industries BrickPi3 output port driver
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
 * The BrickPi3 has four output ports, labeled M1, M2, M3 and M4, for driving
 * motors or other devices. The ports are similar to the output ports on the
 * EV3, except that they cannot automatically detect when a motor is connected.
 * By default, the ``lego-nxt-motor`` driver is loaded, so you don't need to
 * manually set the mode or device unless you want to use something else.
 *
 * .. warning:: Not all :ref:`tacho-motor-class` features are supported.
 *
 *    - The ``state`` attribute will never return the ``stalled`` flag.
 *    - The ``state`` attribute will only return the ``overloaded`` flag with
 *      firmware >= v1.4.3.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/workqueue.h>

#include <dc_motor_class.h>
#include <tacho_motor_class.h>

#include "brickpi3.h"
#include "../ev3/legoev3_motor.h"

/* On Brickpi3, setting a speed/duty cycle of 0 will brake the motor */
#define BRICKPI3_MOTOR_BRAKE (0)
#define BRICKPI3_MOTOR_COAST (-128)

struct brickpi3_out_port {
	struct brickpi3 *bp;
	struct lego_port_device port;
	struct lego_device *motor;
	struct delayed_work run_to_pos_work;
	enum tm_stop_action run_to_pos_stop_action;
	enum brickpi3_output_port index;
	s32 position_sp;
	s8 duty_cycle;
	bool running;
	bool holding;
	bool positioning;
	u8 address;
};

const struct device_type brickpi3_out_port_type = {
	.name = "brickpi3-out-port",
};
EXPORT_SYMBOL_GPL(brickpi3_out_port_type);

static const struct device_type
brickpi3_out_port_device_types[NUM_BRICKPI3_OUT_PORT_MODES] = {
	[BRICKPI3_OUT_PORT_MODE_TACHO_MOTOR] = {
		.name = "ev3-motor",
	},
	[BRICKPI3_OUT_PORT_MODE_DC_MOTOR] = {
		.name = "rcx-motor",
	},
	[BRICKPI3_OUT_PORT_MODE_LED] = {
		.name = "rcx-led",
	},
};

static const char * const
brickpi3_out_port_default_driver[NUM_BRICKPI3_OUT_PORT_MODES] = {
	[BRICKPI3_OUT_PORT_MODE_TACHO_MOTOR]	= "lego-nxt-motor",
	[BRICKPI3_OUT_PORT_MODE_DC_MOTOR]	= "rcx-motor",
	[BRICKPI3_OUT_PORT_MODE_LED] 		= "rcx-led",
};

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new modes have the same syntax. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the Documentation/json/ directory.
 */

static const struct lego_port_mode_info brickpi3_out_port_mode_info[NUM_BRICKPI3_OUT_PORT_MODES] = {
	/**
	 * .. [#out-port-prefix] The full port name includes the parent device
	 *    node. So, the ``address`` attribute will return something like
	 *    ``spi0.1:M1``.
	 *
	 * @description: Dexter Industries BrickPi3 Output Port
	 * @connection_types: tacho-motor, dc-motor, led
	 * @prefix: M
	 * @prefix_footnote: [#out-port-prefix]_
	 * @module: brickpi3
	 */
	[BRICKPI3_OUT_PORT_MODE_TACHO_MOTOR] = {
		/**
		 * @description: NXT/EV3 Large Motor
		 */
		.name	= "tacho-motor",
	},
	[BRICKPI3_OUT_PORT_MODE_DC_MOTOR] = {
		/**
		 * @description: RCX/Power Functions motor
		 */
		.name	= "dc-motor",
	},
	[BRICKPI3_OUT_PORT_MODE_LED] = {
		/**
		 * @description: RCX/Power Functions LED
		 */
		.name	= "led",
	},
};

static unsigned brickpi3_out_port_get_supported_commands(void *context)
{
	return BIT(DC_MOTOR_COMMAND_RUN_FOREVER) | BIT(DC_MOTOR_COMMAND_STOP);
}

static unsigned brickpi3_out_port_get_supported_stop_actions(void *context)
{
	return BIT(DC_MOTOR_STOP_ACTION_COAST) | BIT(DC_MOTOR_STOP_ACTION_BRAKE);
}

static enum dc_motor_internal_command brickpi3_out_port_get_command(void *context)
{
	struct brickpi3_out_port *data = context;

	if (data->duty_cycle == BRICKPI3_MOTOR_COAST)
		return DC_MOTOR_INTERNAL_COMMAND_COAST;
	if (data->duty_cycle == BRICKPI3_MOTOR_BRAKE)
		return DC_MOTOR_INTERNAL_COMMAND_BRAKE;
	if (data->duty_cycle > 0)
		return DC_MOTOR_INTERNAL_COMMAND_RUN_FORWARD;

	return DC_MOTOR_INTERNAL_COMMAND_RUN_REVERSE;
}

static int brickpi3_out_port_set_command(void *context,
					 enum dc_motor_internal_command command)
{
	struct brickpi3_out_port *data = context;

	switch(command) {
	case DC_MOTOR_INTERNAL_COMMAND_RUN_FORWARD:
		data->duty_cycle = abs(data->duty_cycle);
		break;
	case DC_MOTOR_INTERNAL_COMMAND_RUN_REVERSE:
		data->duty_cycle = -abs(data->duty_cycle);
		break;
	case DC_MOTOR_INTERNAL_COMMAND_COAST:
		data->duty_cycle = BRICKPI3_MOTOR_COAST;
		break;
	case DC_MOTOR_INTERNAL_COMMAND_BRAKE:
		data->duty_cycle = BRICKPI3_MOTOR_BRAKE;
		break;
	default:
		return -EINVAL;
	}

	return brickpi3_run_unregulated(data->bp, data->address, data->index,
					data->duty_cycle);
}

static unsigned brickpi3_out_port_get_duty_cycle(void *context)
{
	struct brickpi3_out_port *data = context;

	if (data->duty_cycle == BRICKPI3_MOTOR_COAST)
		return 0;

	return abs(data->duty_cycle);
}

static int brickpi3_out_port_set_duty_cycle(void *context, unsigned duty)
{
	struct brickpi3_out_port *data = context;

	if (data->duty_cycle < 0)
		data->duty_cycle = -duty;
	else
		data->duty_cycle = duty;

	return brickpi3_run_unregulated(data->bp, data->address, data->index,
					data->duty_cycle);
}

static struct dc_motor_ops brickpi3_out_port_dc_motor_ops = {
	.get_supported_commands	= brickpi3_out_port_get_supported_commands,
	.get_supported_stop_actions = brickpi3_out_port_get_supported_stop_actions,
	.get_command		= brickpi3_out_port_get_command,
	.set_command		= brickpi3_out_port_set_command,
	.set_duty_cycle		= brickpi3_out_port_set_duty_cycle,
	.get_duty_cycle		= brickpi3_out_port_get_duty_cycle,
};

static int brickpi3_out_port_get_position(void *context, int *position)
{
	struct brickpi3_out_port *data = context;

	return brickpi3_get_motor_encoder(data->bp, data->address, data->index,
					  position);
}

static int brickpi3_out_port_set_position(void *context, int position)
{
	struct brickpi3_out_port *data = context;
	int current_pos, ret;

	ret = brickpi3_get_motor_encoder(data->bp, data->address, data->index,
					 &current_pos);
	if (ret < 0)
		return ret;

	position += current_pos;

	return brickpi3_set_motor_offset(data->bp, data->address, data->index,
					 position);
}

static int brickpi3_out_port_run_unregulated(void *context, int duty_cycle)
{
	struct brickpi3_out_port *data = context;
	int ret;

	if (duty_cycle == 0)
		duty_cycle = BRICKPI3_MOTOR_COAST;

	ret = brickpi3_run_unregulated(data->bp, data->address, data->index,
				       duty_cycle);
	if (ret < 0)
		return ret;

	data->running = true;
	data->holding = false;
	data->positioning = false;

	return 0;
}

static int brickpi3_out_port_run_regulated(void *context, int speed)
{
	struct brickpi3_out_port *data = context;
	int ret;

	ret = brickpi3_run_regulated(data->bp, data->address, data->index, speed);
	if (ret < 0)
		return ret;

	data->running = true;
	data->holding = false;
	data->positioning = false;

	return 0;
}

static int brickpi3_out_port_run_to_pos(void *context, int pos, int speed,
					enum tm_stop_action stop_action)
{
	struct brickpi3_out_port *data = context;
	int ret;

	ret = brickpi3_set_motor_limits(data->bp, data->address, data->index, 0,
					speed);
	if (ret < 0)
		return ret;

	ret = brickpi3_run_to_position(data->bp, data->address, data->index, pos);
	if (ret < 0)
		return ret;

	data->run_to_pos_stop_action = stop_action;

	data->position_sp = pos;
	data->running = true;
	data->holding = false;
	data->positioning = true;

	schedule_delayed_work(&data->run_to_pos_work, 0);

	return 0;
}

static int brickpi3_out_port_get_state(void *context)
{
	struct brickpi3_out_port *data = context;
	enum brickpi3_motor_status flags;
	unsigned state = 0;
	int ret;

	if (data->running)
		state |= BIT(TM_STATE_RUNNING);
	if (data->holding)
		state |= BIT(TM_STATE_HOLDING);

	ret = brickpi3_read_motor(data->bp, data->address, data->index, &flags,
				  NULL, NULL, NULL);
	if (ret < 0)
		return ret;

	if (flags & BRICKPI3_MOTOR_STATUS_OVERLOADED)
		state |= BIT(TM_STATE_OVERLOADED);

	return state;
}

static int brickpi3_out_port_get_duty_cycle2(void *context, int *duty_cycle)
{
	struct brickpi3_out_port *data = context;

	return brickpi3_read_motor(data->bp, data->address, data->index, NULL,
				   duty_cycle, NULL, NULL);
}

static int brickpi3_out_port_get_speed(void *context, int *speed)
{
	struct brickpi3_out_port *data = context;

	return brickpi3_read_motor(data->bp, data->address, data->index, NULL,
				   NULL, NULL, speed);
}

static int brickpi3_out_port_stop(void *context, enum tm_stop_action stop_action)
{
	struct brickpi3_out_port *data = context;
	bool holding = false;
	int ret, pos;

	switch(stop_action) {
	case TM_STOP_ACTION_COAST:
		ret = brickpi3_run_unregulated(data->bp, data->address,
					       data->index, BRICKPI3_MOTOR_COAST);
		break;
	case TM_STOP_ACTION_BRAKE:
		ret = brickpi3_run_unregulated(data->bp, data->address,
					       data->index, BRICKPI3_MOTOR_BRAKE);
		break;
	case TM_STOP_ACTION_HOLD:
		ret = brickpi3_get_motor_encoder(data->bp, data->address,
						 data->index, &pos);
		if (ret < 0)
			return ret;
		ret = brickpi3_run_to_position(data->bp, data->address,
					       data->index, pos);
		data->position_sp = pos;
		holding = true;
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	data->running = holding;
	data->holding = holding;
	data->positioning = false;

	return 0;
}

static int brickpi3_out_port_reset(void *context)
{
	struct brickpi3_out_port *data = context;
	int ret;

	ret = brickpi3_out_port_stop(data, TM_STOP_ACTION_COAST);
	if (ret < 0)
		return ret;

	return brickpi3_out_port_set_position(data, 0);
}

static unsigned brickpi3_out_port_get_stop_actions(void *context)
{
	return BIT(TM_STOP_ACTION_COAST) |
	       BIT(TM_STOP_ACTION_BRAKE) |
	       BIT(TM_STOP_ACTION_HOLD);
}

struct tacho_motor_ops brickpi3_out_port_tacho_motor_ops = {
	.get_position		= brickpi3_out_port_get_position,
	.set_position		= brickpi3_out_port_set_position,
	.run_unregulated	= brickpi3_out_port_run_unregulated,
	.run_regulated		= brickpi3_out_port_run_regulated,
	.run_to_pos		= brickpi3_out_port_run_to_pos,
	.stop			= brickpi3_out_port_stop,
	.reset			= brickpi3_out_port_reset,
	.get_state		= brickpi3_out_port_get_state,
	.get_duty_cycle		= brickpi3_out_port_get_duty_cycle2,
	.get_speed		= brickpi3_out_port_get_speed,
	.get_stop_actions	= brickpi3_out_port_get_stop_actions,
};

static int brickpi3_out_port_register_motor(struct brickpi3_out_port *out_port,
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

static void brickpi3_out_port_unregister_motor(struct brickpi3_out_port *data)
{
	if (data->motor) {
		cancel_delayed_work_sync(&data->run_to_pos_work);
		lego_device_unregister(data->motor);
		data->motor = NULL;
		brickpi3_run_unregulated(data->bp, data->address, data->index,
					 BRICKPI3_MOTOR_COAST);
	}
}

static int brickpi3_out_port_set_mode(void *context, u8 mode)
{
	struct brickpi3_out_port *data = context;

	brickpi3_out_port_unregister_motor(data);

	return brickpi3_out_port_register_motor(data,
				&brickpi3_out_port_device_types[mode],
				brickpi3_out_port_default_driver[mode]);
}

static void brickpi3_run_to_pos_work(struct work_struct *work)
{
	struct delayed_work *delayed_work = to_delayed_work(work);
	struct brickpi3_out_port *data = container_of(delayed_work,
						      struct brickpi3_out_port,
						      run_to_pos_work);
	int ret, speed, position;

	/* another command was run, so positioning is canceled */
	if (!data->positioning)
		return;

	ret = brickpi3_read_motor(data->bp, data->address, data->index, NULL,
				  NULL, &position, &speed);

	if (ret == -EAGAIN) {
		/* do what it says, try again */
		schedule_delayed_work(&data->run_to_pos_work,
				      msecs_to_jiffies(20));
		return;
	} else if (ret < 0) {
		/* Giving up for now. Might be better if we did a retry. */
		return;
	}

	if (abs(data->position_sp - position) < 10 && speed == 0) {
		/* we have reached the target position */
		if (data->run_to_pos_stop_action == TM_STOP_ACTION_HOLD)
			data->holding = true;
		else
			brickpi3_out_port_stop(data,
					       data->run_to_pos_stop_action);
	} else  {
		/* keep polling... */
		schedule_delayed_work(&data->run_to_pos_work,
				      msecs_to_jiffies(100));
	}
}

static void brickpi3_out_port_release(struct device *dev, void *res)
{
	struct brickpi3_out_port *data = res;

	brickpi3_out_port_unregister_motor(data);
	lego_port_unregister(&data->port);
}

static int devm_brickpi3_out_port_register_one(struct device *dev,
					       struct brickpi3 *bp,
					       u8 address,
					       enum brickpi3_output_port port)
{
	struct brickpi3_out_port *data;
	int ret;

	data = devres_alloc(brickpi3_out_port_release, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->bp = bp;
	data->address = address;
	data->index = port;

	data->port.name = brickpi3_out_port_type.name;
	snprintf(data->port.address, LEGO_NAME_SIZE, "%s:M%c", dev_name(dev),
		 (address - 1) * 4 + port + 'A');
	data->port.num_modes = NUM_BRICKPI3_OUT_PORT_MODES;
	data->port.supported_modes = LEGO_PORT_ALL_MODES;
	data->port.mode_info = brickpi3_out_port_mode_info;
	data->port.set_mode = brickpi3_out_port_set_mode;
	data->port.dc_motor_ops = &brickpi3_out_port_dc_motor_ops;
	data->port.tacho_motor_ops = &brickpi3_out_port_tacho_motor_ops;
	data->port.context = data;

	INIT_DELAYED_WORK(&data->run_to_pos_work, brickpi3_run_to_pos_work);

	ret = lego_port_register(&data->port, &brickpi3_out_port_type, dev);
	if (ret < 0) {
		dev_err(dev, "Failed to register BrickPi3 output port\n");
		return ret;
	}

	devres_add(dev, data);

	ret = brickpi3_out_port_set_mode(data,
					 BRICKPI3_OUT_PORT_MODE_TACHO_MOTOR);
	if (ret < 0) {
		dev_err(dev, "Failed to set output port mode\n");
		return ret;
	}

	return 0;
}


int devm_brickpi3_register_out_ports(struct device *dev, struct brickpi3 *bp,
				     u8 address)
{
	int i, ret;

	for (i = 0; i < NUM_BRICKPI3_OUTPUT_PORTS; i++) {
		ret = devm_brickpi3_out_port_register_one (dev, bp, address, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}
