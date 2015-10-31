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

#define COMMAND_REG		0x41

#define WRITE_SIZE		8
#define ENCODER_SIZE		4

#define WRITE_REG(idx)		(0x42 + WRITE_SIZE * idx)
#define WRITE_ENCODER_TARGET	0
#define WRITE_SPEED		4
#define WRITE_TIME		5
#define WRITE_COMMAND_B		6
#define WRITE_COMMAND_A		7

#ifdef PISTORMS

/*
 * The PiStorms motor interface is virtually identical to the NXTMMX, so this
 * code is shared with both modules. Just a few register addresses are different.
 */

#define READ_ENCODER_POS_REG(idx)	(0x52 + ENCODER_SIZE * idx)
#define READ_STATUS_REG(idx)		(0x5A + idx)
#define READ_TASKS_REG(idx)		(0x5C + idx)

#define ENCODER_PID_KP_REG	0x5E
#define ENCODER_PID_KI_REG	0x50
#define ENCODER_PID_KD_REG	0x62

#define SPEED_PID_KP_REG	0x64
#define SPEED_PID_KI_REG	0x66
#define SPEED_PID_KD_REG	0x68

#else

#define READ_ENCODER_POS_REG(idx)	(0x62 + ENCODER_SIZE * idx)
#define READ_STATUS_REG(idx)		(0x72 + idx)
#define READ_TASKS_REG(idx)		(0x76 + idx)

#define ENCODER_PID_KP_REG	0x7A
#define ENCODER_PID_KI_REG	0x7C
#define ENCODER_PID_KD_REG	0x7E

#define SPEED_PID_KP_REG	0x80
#define SPEED_PID_KI_REG	0x82
#define SPEED_PID_KD_REG	0x84

#endif

#define PID_K_SIZE		2

#define COMMAND_RESET_ALL		'R'
#define COMMAND_SYNC_START		'S'
#define COMMAND_FLOAT_STOP(idx)		('a' + idx)
#define COMMAND_SYNC_FLOAT_STOP		'c'
#define COMMAND_BRAKE_STOP(idx)		('A' + idx)
#define COMMAND_SYNC_BRAKE_STOP		'C'
#define COMMAND_RESET_ENCODER(idx)	('r' + idx)

#define CMD_FLAG_SPEED_CTRL	BIT(0)
#define CMD_FLAG_RAMP		BIT(1)
#define CMD_FLAG_RELATIVE	BIT(2)
#define CMD_FLAG_ENCODER_CTRL	BIT(3)
#define CMD_FLAG_BRAKE		BIT(4)
#define CMD_FLAG_HOLD		BIT(5)
#define CMD_FLAG_TIMED		BIT(6)
#define CMD_FLAG_GO		BIT(7)

/* default value for mmx->command_flags */
#define CMD_FLAGS_DEFAULT_VALUE (CMD_FLAG_SPEED_CTRL | CMD_FLAG_BRAKE)
/* special value used when stop command is "hold" */
#define CMD_FLAGS_STOP_HOLD (CMD_FLAG_SPEED_CTRL | CMD_FLAG_ENCODER_CTRL | CMD_FLAG_HOLD | CMD_FLAG_GO)

#define STATUS_FLAG_SPEED_CTRL	BIT(0)
#define STATUS_FLAG_RAMPING	BIT(1)
#define STATUS_FLAG_POWERED	BIT(2)
#define STATUS_FLAG_POSITION	BIT(3)
#define STATUS_FLAG_BRAKE	BIT(4)
#define STATUS_FLAG_OVERLOAD	BIT(5)
#define STATUS_FLAG_TIMED	BIT(6)
#define STATUS_FLAG_STALL	BIT(7)

struct ms_nxtmmx_data {
	struct tacho_motor_device tm;
	struct i2c_client *i2c_client;
	char port_name[LEGO_PORT_NAME_SIZE];
	int index;
	unsigned command_flags;
};

/*
 * Converts speed in deg/sec to value that will be sent to the NxtMMX.
 * Scaling was determined by interpolation.
 */
static inline int ms_nxtmmx_scale_speed(int speed)
{
	int scaled = abs(speed);
	if (scaled <= 50)
		scaled = 0;
	else
		scaled -= 50;
	scaled /= 8;
	if (speed < 0)
		scaled *= -1;
	return scaled;
}

static int ms_nxtmmx_get_position(void *context, long *position)
{
	struct ms_nxtmmx_data *mmx = context;
	int err;
	u8 bytes[ENCODER_SIZE];

	err = i2c_smbus_read_i2c_block_data(mmx->i2c_client,
		READ_ENCODER_POS_REG(mmx->index), ENCODER_SIZE, bytes);
	if (err < 0)
		return err;

	*position = le32_to_cpup((int *)bytes);

	return 0;
}

static int ms_nxtmmx_set_position(void *context, long position)
{
	struct ms_nxtmmx_data *mmx = context;
	int ret;

	/* we can only reset the encoder reading to 0. */
	if (position != 0)
		return -EINVAL;

	/*
	 * Can't change the position while the motor is running or it will make
	 * the controller confused.
	 */
	ret = i2c_smbus_read_byte_data(mmx->i2c_client, READ_STATUS_REG(mmx->index));
	if (ret < 0)
		return ret;
	if (ret & STATUS_FLAG_POWERED)
		return -EBUSY;

	ret = i2c_smbus_write_byte_data(mmx->i2c_client, COMMAND_REG,
		COMMAND_RESET_ENCODER(mmx->index));
	if (ret < 0)
		return ret;

	return 0;
}

static int ms_nxtmmx_get_state(void *context)
{
	struct ms_nxtmmx_data *mmx = context;
	int ret;
	unsigned state = 0;

	ret = i2c_smbus_read_byte_data(mmx->i2c_client, READ_STATUS_REG(mmx->index));
	if (ret < 0)
		return ret;

	if (ret & STATUS_FLAG_POWERED)
		state |= BIT(TM_STATE_RUNNING);
	if (ret & STATUS_FLAG_RAMPING)
		state |= BIT(TM_STATE_RAMPING);
	if (ret & STATUS_FLAG_STALL)
		state |= BIT(TM_STATE_STALLED);

	/*
	 * If motor is powered and it might be holding position, then we have
	 * to check the tasks register as well. If the tasks register is > 0,
	 * then we are still running, otherwise we are holding.
	 */
	if ((ret & STATUS_FLAG_POWERED) && (mmx->command_flags & CMD_FLAG_HOLD)) {
		ret = i2c_smbus_read_byte_data(mmx->i2c_client, READ_TASKS_REG(mmx->index));
		if (ret < 0)
			return ret;
		if (!ret)
			state |= BIT(TM_STATE_HOLDING);
	}

	return state;
}

static int ms_nxtmmx_get_count_per_rot(void *context)
{
	/* Only supports LEGO motors */
	return 360;
}

static unsigned ms_nxtmmx_get_commands(void *context)
{
	return BIT(TM_COMMAND_RUN_FOREVER) | BIT (TM_COMMAND_RUN_TO_ABS_POS)
		| BIT(TM_COMMAND_RUN_TO_REL_POS)
		| BIT(TM_COMMAND_STOP) | BIT(TM_COMMAND_RESET);
}

static int ms_nxtmmx_send_command(void *context,
				  struct tacho_motor_params *param,
				  enum tacho_motor_command command)
{
	struct ms_nxtmmx_data *mmx = context;
	int err;
	u8 command_bytes[WRITE_SIZE];

	if (IS_RUN_CMD(command)) {
		/* fill in the setpoints with the correct polarity */
		*(int *)command_bytes = cpu_to_le32(mmx->tm.params.position_sp);
		command_bytes[WRITE_SPEED] = ms_nxtmmx_scale_speed(mmx->tm.params.speed_sp);
		if (mmx->tm.params.polarity == DC_MOTOR_POLARITY_INVERSED) {
			*(int *)command_bytes *= -1;
			command_bytes[WRITE_SPEED] *= -1;
		}

		/* set the appropriate command flags based on the run command */

		if (IS_POS_CMD(command))
			mmx->command_flags |= CMD_FLAG_ENCODER_CTRL;
		else
			mmx->command_flags &= ~CMD_FLAG_ENCODER_CTRL;

		if (command == TM_COMMAND_RUN_TO_REL_POS)
			mmx->command_flags |= CMD_FLAG_RELATIVE;
		else
			mmx->command_flags &= ~CMD_FLAG_RELATIVE;

		/* set bits for stop command */

		if (mmx->tm.params.stop_command == TM_STOP_COMMAND_HOLD)
			mmx->command_flags |= CMD_FLAG_HOLD;
		else
			mmx->command_flags &= ~CMD_FLAG_HOLD;

		if (mmx->tm.params.stop_command == TM_STOP_COMMAND_BRAKE)
			mmx->command_flags |= CMD_FLAG_BRAKE;
		else
			mmx->command_flags &= ~CMD_FLAG_BRAKE;

		mmx->command_flags |= CMD_FLAG_GO;

		command_bytes[WRITE_TIME] = 0; /* never use timed mode */
		command_bytes[WRITE_COMMAND_B] = 0;
		command_bytes[WRITE_COMMAND_A] = mmx->command_flags;

		/* then write to the individual motor register to GO! */

		err = i2c_smbus_write_i2c_block_data(mmx->i2c_client,
			WRITE_REG(mmx->index), WRITE_SIZE, command_bytes);
		if (err < 0)
			return err;
	} else if (command == TM_COMMAND_STOP) {
		mmx->command_flags = CMD_FLAGS_DEFAULT_VALUE;
		command_bytes[0] = (mmx->tm.params.stop_command == TM_STOP_COMMAND_COAST)
			? COMMAND_FLOAT_STOP(mmx->index) : COMMAND_BRAKE_STOP(mmx->index);
		err = i2c_smbus_write_byte_data(mmx->i2c_client,
			COMMAND_REG, command_bytes[0]);
		if (err < 0)
			return err;
		if (mmx->tm.params.stop_command == TM_STOP_COMMAND_HOLD) {
			/*
			 * Hold only happens when encoder mode is enabled, so
			 * we have to issue a run command to tell it to run to
			 * the current position so that it will hold that position.
			 */
			err = i2c_smbus_read_i2c_block_data(mmx->i2c_client,
				READ_ENCODER_POS_REG(mmx->index), ENCODER_SIZE, command_bytes);
			if (err < 0)
				return err;
			mmx->command_flags = CMD_FLAGS_STOP_HOLD;
			command_bytes[WRITE_SPEED] = 100;
			command_bytes[WRITE_TIME] = 0;
			command_bytes[WRITE_COMMAND_B] = 0;
			command_bytes[WRITE_COMMAND_A] = mmx->command_flags;
			err = i2c_smbus_write_i2c_block_data(mmx->i2c_client,
				WRITE_REG(mmx->index), WRITE_SIZE, command_bytes);
			if (err < 0)
				return err;
		}
	} else if (command == TM_COMMAND_RESET) {
		mmx->tm.params.speed_regulation = TM_SPEED_REGULATION_ON;
		mmx->command_flags = CMD_FLAGS_DEFAULT_VALUE;
		command_bytes[0] = COMMAND_FLOAT_STOP(mmx->index);
		err = i2c_smbus_write_byte_data(mmx->i2c_client,
			COMMAND_REG, command_bytes[0]);
		if (err < 0)
			return err;
		command_bytes[0] = COMMAND_RESET_ENCODER(mmx->index);
		err = i2c_smbus_write_byte_data(mmx->i2c_client,
			COMMAND_REG, command_bytes[0]);
		if (err < 0)
			return err;
	}

	return 0;
}

static unsigned ms_nxtmmx_get_speed_regulations(void *context)
{
	/*
	 * This controller only works with speed control enabled - except when
	 * the Encoder bit is set, in which case it runs at 100% duty cycle.
	 * So, we don't allow changing speed_regulation and it will return "on"
	 * when read.
	 */
	return BIT(TM_SPEED_REGULATION_ON);
}

static unsigned ms_nxtmmx_get_stop_commands(void *context)
{
	return BIT(TM_STOP_COMMAND_COAST) | BIT(TM_STOP_COMMAND_BRAKE) |
		BIT(TM_STOP_COMMAND_HOLD);
}

static int ms_nxtmmx_get_speed_Kp(void *context)
{
	struct ms_nxtmmx_data *mmx = context;
	int err;
	u8 k[PID_K_SIZE];

	err = i2c_smbus_read_i2c_block_data(mmx->i2c_client, SPEED_PID_KP_REG,
								PID_K_SIZE, k);
	if (err < 0)
		return err;

	return le16_to_cpu(*(s16*)k);
}

static int ms_nxtmmx_set_speed_Kp(void *context, int Kp)
{
	struct ms_nxtmmx_data *mmx = context;
	u8 k[PID_K_SIZE];

	*(s16*)k = cpu_to_le16(Kp);
	return i2c_smbus_write_i2c_block_data(mmx->i2c_client, SPEED_PID_KP_REG,
								PID_K_SIZE, k);
}

static int ms_nxtmmx_get_speed_Ki(void *context)
{
	struct ms_nxtmmx_data *mmx = context;
	int err;
	u8 k[PID_K_SIZE];

	err = i2c_smbus_read_i2c_block_data(mmx->i2c_client, SPEED_PID_KI_REG,
								PID_K_SIZE, k);
	if (err < 0)
		return err;

	return le16_to_cpu(*(s16*)k);
}

static int ms_nxtmmx_set_speed_Ki(void *context, int Ki)
{
	struct ms_nxtmmx_data *mmx = context;
	u8 k[PID_K_SIZE];

	*(s16*)k = cpu_to_le16(Ki);
	return i2c_smbus_write_i2c_block_data(mmx->i2c_client, SPEED_PID_KI_REG,
								PID_K_SIZE, k);
}

static int ms_nxtmmx_get_speed_Kd(void *context)
{
	struct ms_nxtmmx_data *mmx = context;
	int err;
	u8 k[PID_K_SIZE];

	err = i2c_smbus_read_i2c_block_data(mmx->i2c_client, SPEED_PID_KD_REG,
								PID_K_SIZE, k);
	if (err < 0)
		return err;

	return le16_to_cpu(*(s16*)k);
}

static int ms_nxtmmx_set_speed_Kd(void *context, int Kd)
{
	struct ms_nxtmmx_data *mmx = context;
	u8 k[PID_K_SIZE];

	*(s16*)k = cpu_to_le16(Kd);
	return i2c_smbus_write_i2c_block_data(mmx->i2c_client, SPEED_PID_KD_REG,
								PID_K_SIZE, k);
}

static int ms_nxtmmx_get_position_Kp(void *context)
{
	struct ms_nxtmmx_data *mmx = context;
	int err;
	u8 k[PID_K_SIZE];

	err = i2c_smbus_read_i2c_block_data(mmx->i2c_client,
					ENCODER_PID_KP_REG, PID_K_SIZE, k);
	if (err < 0)
		return err;

	return le16_to_cpu(*(s16*)k);
}

static int ms_nxtmmx_set_position_Kp(void *context, int Kp)
{
	struct ms_nxtmmx_data *mmx = context;
	u8 k[PID_K_SIZE];

	*(s16*)k = cpu_to_le16(Kp);
	return i2c_smbus_write_i2c_block_data(mmx->i2c_client,
					ENCODER_PID_KP_REG, PID_K_SIZE, k);
}

static int ms_nxtmmx_get_position_Ki(void *context)
{
	struct ms_nxtmmx_data *mmx = context;
	int err;
	u8 k[PID_K_SIZE];

	err = i2c_smbus_read_i2c_block_data(mmx->i2c_client,
					ENCODER_PID_KI_REG, PID_K_SIZE, k);
	if (err < 0)
		return err;

	return le16_to_cpu(*(s16*)k);
}

static int ms_nxtmmx_set_position_Ki(void *context, int Ki)
{
	struct ms_nxtmmx_data *mmx = context;
	u8 k[PID_K_SIZE];

	*(s16*)k = cpu_to_le16(Ki);
	return i2c_smbus_write_i2c_block_data(mmx->i2c_client,
					ENCODER_PID_KI_REG, PID_K_SIZE, k);
}

static int ms_nxtmmx_get_position_Kd(void *context)
{
	struct ms_nxtmmx_data *mmx = context;
	int err;
	u8 k[PID_K_SIZE];

	err = i2c_smbus_read_i2c_block_data(mmx->i2c_client,
					ENCODER_PID_KD_REG, PID_K_SIZE, k);
	if (err < 0)
		return err;

	return le16_to_cpu(*(s16*)k);
}

static int ms_nxtmmx_set_position_Kd(void *context, int Kd)
{
	struct ms_nxtmmx_data *mmx = context;
	u8 k[PID_K_SIZE];

	*(s16*)k = cpu_to_le16(Kd);
	return i2c_smbus_write_i2c_block_data(mmx->i2c_client,
					ENCODER_PID_KD_REG, PID_K_SIZE, k);
}

struct tacho_motor_ops ms_nxtmmx_tacho_motor_ops = {
	.get_position		= ms_nxtmmx_get_position,
	.set_position		= ms_nxtmmx_set_position,
	.get_state		= ms_nxtmmx_get_state,
	.get_count_per_rot	= ms_nxtmmx_get_count_per_rot,
	.get_commands		= ms_nxtmmx_get_commands,
	.send_command		= ms_nxtmmx_send_command,
	.get_speed_regulations	= ms_nxtmmx_get_speed_regulations,
	.get_stop_commands	= ms_nxtmmx_get_stop_commands,
	.get_speed_Kp		= ms_nxtmmx_get_speed_Kp,
	.set_speed_Kp		= ms_nxtmmx_set_speed_Kp,
	.get_speed_Ki		= ms_nxtmmx_get_speed_Ki,
	.set_speed_Ki		= ms_nxtmmx_set_speed_Ki,
	.get_speed_Kd		= ms_nxtmmx_get_speed_Kd,
	.set_speed_Kd		= ms_nxtmmx_set_speed_Kd,
	.get_hold_Kp		= ms_nxtmmx_get_position_Kp,
	.set_hold_Kp		= ms_nxtmmx_set_position_Kp,
	.get_hold_Ki		= ms_nxtmmx_get_position_Ki,
	.set_hold_Ki		= ms_nxtmmx_set_position_Ki,
	.get_hold_Kd		= ms_nxtmmx_get_position_Kd,
	.set_hold_Kd		= ms_nxtmmx_set_position_Kd,
};

#ifndef PISTORMS

#include "ms_nxtmmx.h"

int ms_nxtmmx_probe_cb(struct nxt_i2c_sensor_data *data)
{
	struct ms_nxtmmx_data *mmx;
	int i, err;

	mmx = kzalloc(sizeof(struct ms_nxtmmx_data) * 2, GFP_KERNEL);
	if (!mmx)
		return -ENOMEM;

	data->callback_data = mmx;

	for (i = 0; i < 2; i++) {
		mmx[i].tm.driver_name = data->info->name;
		snprintf(mmx[i].port_name, LEGO_PORT_NAME_SIZE, "%s:i2c%d:mux%d",
			 data->port_name, data->client->addr, i + 1);
		mmx[i].tm.port_name = mmx[i].port_name;
		mmx[i].tm.ops = &ms_nxtmmx_tacho_motor_ops;
		mmx[i].tm.context = &mmx[i];
		mmx[i].i2c_client = data->client;
		mmx[i].index = i;
		mmx[i].command_flags = CMD_FLAGS_DEFAULT_VALUE;
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
	data->callback_data = NULL;
	kfree(mmx);

	return err;
}

void ms_nxtmmx_remove_cb(struct nxt_i2c_sensor_data *data)
{
	struct ms_nxtmmx_data *mmx = data->callback_data;

	unregister_tacho_motor(&mmx[1].tm);
	unregister_tacho_motor(&mmx[0].tm);
	data->callback_data = NULL;
	kfree(mmx);
}

# endif
