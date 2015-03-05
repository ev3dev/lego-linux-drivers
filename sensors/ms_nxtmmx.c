/*
 * mindsensors.com Motor Multiplexer device driver
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

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include <tacho_motor_class.h>

#include "ms_nxtmmx.h"

#define COMMAND_REG		0x41

#define WRITE_SIZE		8
#define ENCODER_SIZE		4

#define WRITE_REG		0x42
#define WRITE_ENCODER_TARGET	0
#define WRITE_SPEED		4
#define WRITE_TIME		5
#define WRITE_COMMAND_B		6
#define WRITE_COMMAND_A		7

#define READ_ENCODER_POS	0x62
#define READ_STATUS		0x72
#define READ_TASKS		0x74

#define PID_K_SIZE		2

#define ENCODER_PID_KP		0x7A
#define ENCODER_PID_KI		0x7C
#define ENCODER_PID_KD		0x7E

#define SPEED_PID_KP		0x7A
#define SPEED_PID_KI		0x7C
#define SPEED_PID_KD		0x7E

#define COMMAND_RESET_ALL	'R'
#define COMMAND_SYNC_START	'S'
#define COMMAND_FLOAT_STOP	'a'
#define COMMAND_SYNC_FLOAT_STOP	'c'
#define COMMAND_BRAKE_STOP	'A'
#define COMMAND_SYNC_BRAKE_STOP	'C'
#define COMMAND_RESET_ENCODER	'r'

#define CMD_FLAG_SPEED_CTRL	BIT(0)
#define CMD_FLAG_RAMP		BIT(1)
#define CMD_FLAG_RELATIVE	BIT(2)
#define CMD_FLAG_ENCODER_CTRL	BIT(3)
#define CMD_FLAG_BRAKE		BIT(4)
#define CMD_FLAG_HOLD		BIT(5)
#define CMD_FLAG_TIMED		BIT(6)
#define CMD_FLAG_GO		BIT(7)

struct ms_nxtmmx_data {
	struct tacho_motor_device tm;
	struct nxt_i2c_sensor_data *i2c;
	char port_name[LEGO_PORT_NAME_SIZE];
	int index;
	int duty_cycle_sp;
	int speed_sp;
	int position_sp;
	int time_sp;
	enum dc_motor_polarity polarity;
	unsigned command_flags;
};

static inline struct ms_nxtmmx_data *to_mx_nxtmmx(struct tacho_motor_device *tm)
{
	return container_of(tm, struct ms_nxtmmx_data, tm);
}

static int ms_nxtmmx_get_position(struct tacho_motor_device *tm, long *position)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);
	int offset, err;
	u8 bytes[ENCODER_SIZE];

	offset = ENCODER_SIZE * mmx->index;
	err = i2c_smbus_read_i2c_block_data(mmx->i2c->client,
		READ_ENCODER_POS + offset, ENCODER_SIZE, bytes);
	if (err < 0)
		return err;

	*position = le32_to_cpup((int *)bytes);

	return 0;
}

static int ms_nxtmmx_set_position(struct tacho_motor_device *tm, long position)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);
	int command, err;

	/* we can only reset the encoder reading to 0. */
	if (position != 0)
		return -EINVAL;

	command = COMMAND_RESET_ENCODER + mmx->index;
	err = i2c_smbus_write_byte_data(mmx->i2c->client, COMMAND_REG, command);
	if (err < 0)
		return err;

	return 0;
}

static int ms_nxtmmx_get_state(struct tacho_motor_device *tm)
{
	/* TODO: Need to finalize new tacho-motor class state implementation */
	return 0;
}

static int ms_nxtmmx_get_count_per_rot(struct tacho_motor_device *tm)
{
	/* Only supports LEGO motors */
	return 360;
}

static int ms_nxtmmx_get_duty_cycle_sp(struct tacho_motor_device *tm, int *sp)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);

	*sp = mmx->duty_cycle_sp;

	return 0;
}

static int ms_nxtmmx_set_duty_cycle_sp(struct tacho_motor_device *tm, int sp)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);

	mmx->duty_cycle_sp = sp;

	return 0;
}

/*
 * Docs don't say, so assuming max speed is 1000 counts per second. Same register
 * is used for regulated and unregulated speed.
 */

static int ms_nxtmmx_get_speed_sp(struct tacho_motor_device *tm, int *sp)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);

	*sp = mmx->speed_sp;

	return 0;
}

static int ms_nxtmmx_set_speed_sp(struct tacho_motor_device *tm, int sp)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);

	mmx->speed_sp = sp;

	return 0;
}

/*
 * TODO: NxtMMX only supports whole numbers of seconds. Would be better to
 * implement our own timed run.
 */

static int ms_nxtmmx_get_time_sp(struct tacho_motor_device *tm)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);

	return mmx->time_sp;
}

static int ms_nxtmmx_set_time_sp(struct tacho_motor_device *tm, int sp)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);

	return mmx->time_sp = sp;

	return 0;
}

static int ms_nxtmmx_get_position_sp(struct tacho_motor_device *tm, int *position)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);

	*position = mmx->position_sp;

	return 0;
}

static int ms_nxtmmx_set_position_sp(struct tacho_motor_device *tm, int position)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);

	mmx->position_sp = position;

	return 0;
}

static unsigned ms_nxtmmx_get_commands(struct tacho_motor_device *tm)
{
	return BIT(TM_COMMAND_RUN_FOREVER) | BIT (TM_COMMAND_RUN_TO_ABS_POS)
		| BIT(TM_COMMAND_RUN_TO_REL_POS) | BIT(TM_COMMAND_RUN_TIMED)
		| BIT(TM_COMMAND_STOP) | BIT(TM_COMMAND_RESET);
}

static int ms_nxtmmx_send_command(struct tacho_motor_device *tm,
				  enum tacho_motor_command command)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);
	int offset, err;
	u8 command_bytes[WRITE_SIZE];

	if (IS_RUN_CMD(command)) {
		/* fill in the setpoints with the correct polarity */

		if (mmx->polarity == DC_MOTOR_POLARITY_NORMAL) {
			*(int *)command_bytes = cpu_to_le32(mmx->position_sp);
			if (mmx->command_flags & CMD_FLAG_SPEED_CTRL)
				command_bytes[WRITE_SPEED] = mmx->speed_sp / 10;
			else
				command_bytes[WRITE_SPEED] = mmx->duty_cycle_sp;
		} else {
			*(int *)command_bytes = -cpu_to_le32(mmx->position_sp);
			if (mmx->command_flags & CMD_FLAG_SPEED_CTRL)
				command_bytes[WRITE_SPEED] = -mmx->speed_sp / 10;
			else
				command_bytes[WRITE_SPEED] = -mmx->duty_cycle_sp;
		}

		/* TODO: need to check the sign on speed setpoint when running to absolute position */

		/* set the appropriate command flags based on the run command */

		if (command == TM_COMMAND_RUN_TIMED)
			mmx->command_flags |= CMD_FLAG_TIMED;
		else
			mmx->command_flags &= ~CMD_FLAG_TIMED;

		if (IS_POS_CMD(command))
			mmx->command_flags |= CMD_FLAG_ENCODER_CTRL;
		else
			mmx->command_flags &= ~CMD_FLAG_ENCODER_CTRL;

		if (command == TM_COMMAND_RUN_TO_REL_POS)
			mmx->command_flags |= CMD_FLAG_RELATIVE;
		else
			mmx->command_flags &= ~CMD_FLAG_RELATIVE;

		mmx->command_flags |= CMD_FLAG_GO;

		command_bytes[WRITE_TIME] = mmx->time_sp / 1000;
		command_bytes[WRITE_COMMAND_B] = 0;
		command_bytes[WRITE_COMMAND_A] = mmx->command_flags;

		/* then write to the individual motor register to GO! */

		offset = WRITE_SIZE * mmx->index;
		err = i2c_smbus_write_i2c_block_data(mmx->i2c->client,
			WRITE_REG + offset, WRITE_SIZE, command_bytes);
		if (err < 0)
			return err;
	} else if (command == TM_COMMAND_STOP) {
		/* TODO: handle case for stop_command == hold */
		command_bytes[0] = (mmx->command_flags & CMD_FLAG_BRAKE)
			? COMMAND_BRAKE_STOP : COMMAND_FLOAT_STOP;
		command_bytes[0] += mmx->index;
		err = i2c_smbus_write_byte_data(mmx->i2c->client,
			COMMAND_REG, command_bytes[0]);
		if (err < 0)
			return err;
	} else if (command == TM_COMMAND_RESET) {
		command_bytes[0] = COMMAND_RESET_ALL + mmx->index;
		err = i2c_smbus_write_byte_data(mmx->i2c->client,
			COMMAND_REG, command_bytes[0]);
		if (err < 0)
			return err;
		mmx->duty_cycle_sp = 0;
		mmx->speed_sp = 0;
		mmx->position_sp = 0;
		mmx->time_sp = 0;
		mmx->polarity = DC_MOTOR_POLARITY_NORMAL;
		mmx->command_flags = 0;
	}

	return 0;
}

static int ms_nxtmmx_get_speed_regulation(struct tacho_motor_device *tm)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);

	return (mmx->command_flags & CMD_FLAG_SPEED_CTRL)
		? TM_SPEED_REGULATION_ON : TM_SPEED_REGULATION_OFF;
}

static int ms_nxtmmx_set_speed_regulation(struct tacho_motor_device *tm,
					  enum tacho_motor_speed_regulation reg)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);

	if (reg == TM_SPEED_REGULATION_ON)
		mmx->command_flags |= CMD_FLAG_SPEED_CTRL;
	else
		mmx->command_flags &= ~CMD_FLAG_SPEED_CTRL;

	return 0;
}

static unsigned ms_nxtmmx_get_stop_commands(struct tacho_motor_device *tm)
{
	return BIT(TM_STOP_COMMAND_COAST) | BIT(TM_STOP_COMMAND_BRAKE) |
		BIT(TM_STOP_COMMAND_HOLD);
}

static int ms_nxtmmx_get_stop_command(struct tacho_motor_device *tm)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);

	if (mmx->command_flags & CMD_FLAG_HOLD)
		return TM_STOP_COMMAND_HOLD;

	return (mmx->command_flags & CMD_FLAG_BRAKE)
		? TM_STOP_COMMAND_BRAKE : TM_STOP_COMMAND_COAST;
}

static int ms_nxtmmx_set_stop_command(struct tacho_motor_device *tm,
				      enum tacho_motor_stop_command command)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);

	if (command == TM_STOP_COMMAND_HOLD)
		mmx->command_flags |= CMD_FLAG_HOLD;
	else
		mmx->command_flags &= ~CMD_FLAG_HOLD;

	if (command == TM_STOP_COMMAND_BRAKE)
		mmx->command_flags |= CMD_FLAG_BRAKE;
	else
		mmx->command_flags &= ~CMD_FLAG_BRAKE;

	return 0;
}

static int ms_nxtmmx_get_polarity(struct tacho_motor_device *tm)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);

	return mmx->polarity;
}

static int ms_nxtmmx_set_polarity(struct tacho_motor_device *tm,
				  enum dc_motor_polarity polarity)
{
	struct ms_nxtmmx_data *mmx = to_mx_nxtmmx(tm);

	mmx->polarity = polarity;

	return 0;
}

struct tacho_motor_ops ms_nxtmmx_tacho_motor_ops = {
	.get_position		= ms_nxtmmx_get_position,
	.set_position		= ms_nxtmmx_set_position,
	.get_state		= ms_nxtmmx_get_state,
	.get_count_per_rot	= ms_nxtmmx_get_count_per_rot,
	.get_duty_cycle_sp	= ms_nxtmmx_get_duty_cycle_sp,
	.set_duty_cycle_sp	= ms_nxtmmx_set_duty_cycle_sp,
	.get_speed_sp		= ms_nxtmmx_get_speed_sp,
	.set_speed_sp		= ms_nxtmmx_set_speed_sp,
	.get_time_sp		= ms_nxtmmx_get_time_sp,
	.set_time_sp		= ms_nxtmmx_set_time_sp,
	.get_position_sp	= ms_nxtmmx_get_position_sp,
	.set_position_sp	= ms_nxtmmx_set_position_sp,
	.get_commands		= ms_nxtmmx_get_commands,
	.send_command		= ms_nxtmmx_send_command,
	.get_speed_regulation	= ms_nxtmmx_get_speed_regulation,
	.set_speed_regulation	= ms_nxtmmx_set_speed_regulation,
	.get_stop_commands	= ms_nxtmmx_get_stop_commands,
	.get_stop_command	= ms_nxtmmx_get_stop_command,
	.set_stop_command	= ms_nxtmmx_set_stop_command,
	.get_polarity		= ms_nxtmmx_get_polarity,
	.set_polarity		= ms_nxtmmx_set_polarity,
};

int ms_nxtmmx_probe_cb(struct nxt_i2c_sensor_data *data)
{
	struct ms_nxtmmx_data *mmx;
	int i, err;

	mmx = kzalloc(sizeof(struct ms_nxtmmx_data) * 2, GFP_KERNEL);
	if (!mmx)
		return -ENOMEM;

	data->info.callback_data = mmx;

	for (i = 0; i < 2; i++) {
		mmx[i].tm.driver_name = data->info.name;
		snprintf(mmx[i].port_name, LEGO_PORT_NAME_SIZE, "%s:i2c%d:mux%d",
			 data->in_port->port_name, data->client->addr, i);
		mmx[i].tm.port_name = mmx[i].port_name;
		mmx[i].tm.ops = &ms_nxtmmx_tacho_motor_ops;
		mmx[i].i2c = data;
		mmx[i].index = i;
	}

	err = register_tacho_motor(&mmx[0].tm, &data->client->dev);
	if (err)
		goto err_register_tacho_motor0;
	err = register_tacho_motor(&mmx[1].tm, &data->client->dev);
	if (err)
		goto err_register_tacho_motor1;

	data->poll_ms = 1000;

	return 0;

err_register_tacho_motor1:
	unregister_tacho_motor(&mmx[0].tm);
err_register_tacho_motor0:
	data->info.callback_data = NULL;
	kfree(mmx);

	return err;
}

void ms_nxtmmx_remove_cb(struct nxt_i2c_sensor_data *data)
{
	struct ms_nxtmmx_data *mmx = data->info.callback_data;

	unregister_tacho_motor(&mmx[1].tm);
	unregister_tacho_motor(&mmx[0].tm);
	data->info.callback_data = NULL;
	kfree(mmx);
}
