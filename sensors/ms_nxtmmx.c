/*
 * mindsensors.com Motor Multiplexer device driver
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

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include <lego.h>
#include <tacho_motor_class.h>

#define COMMAND_REG		0x41

#define WRITE_SIZE		8
#define ENCODER_SIZE		4

#define WRITE_REG(idx)		(0x42 + WRITE_SIZE * (idx))
#define WRITE_ENCODER_TARGET	0
#define WRITE_SPEED		4
#define WRITE_TIME		5
#define WRITE_COMMAND_B		6
#define WRITE_COMMAND_A		7

#ifdef PISTORMS_NXTMMX

/*
 * The PiStorms motor interface is virtually identical to the NXTMMX, so this
 * code is shared with both modules. Just a few register addresses are different.
 */

#define READ_ENCODER_POS_REG(idx)	(0x52 + ENCODER_SIZE * (idx))
#define READ_STATUS_REG(idx)		(0x5A + (idx))
#define READ_TASKS_REG(idx)		(0x5C + (idx))

#define ENCODER_PID_KP_REG	0x5E
#define ENCODER_PID_KI_REG	0x50
#define ENCODER_PID_KD_REG	0x62

#define SPEED_PID_KP_REG	0x64
#define SPEED_PID_KI_REG	0x66
#define SPEED_PID_KD_REG	0x68

#else

#define READ_ENCODER_POS_REG(idx)	(0x62 + ENCODER_SIZE * (idx))
#define READ_STATUS_REG(idx)		(0x72 + (idx))
#define READ_TASKS_REG(idx)		(0x76 + (idx))

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
#define COMMAND_FLOAT_STOP(idx)		('a' + (idx))
#define COMMAND_SYNC_FLOAT_STOP		'c'
#define COMMAND_BRAKE_STOP(idx)		('A' + (idx))
#define COMMAND_SYNC_BRAKE_STOP		'C'
#define COMMAND_RESET_ENCODER(idx)	('r' + (idx))

#define CMD_FLAG_SPEED_CTRL	BIT(0)
#define CMD_FLAG_RAMP		BIT(1)
#define CMD_FLAG_RELATIVE	BIT(2)
#define CMD_FLAG_ENCODER_CTRL	BIT(3)
#define CMD_FLAG_BRAKE		BIT(4)
#define CMD_FLAG_HOLD		BIT(5)
#define CMD_FLAG_TIMED		BIT(6)
#define CMD_FLAG_GO		BIT(7)

/* special value used when stop command is "hold" */
#define CMD_FLAGS_STOP_HOLD \
(CMD_FLAG_SPEED_CTRL | CMD_FLAG_ENCODER_CTRL | CMD_FLAG_HOLD | CMD_FLAG_GO)

#define STATUS_FLAG_SPEED_CTRL	BIT(0)
#define STATUS_FLAG_RAMPING	BIT(1)
#define STATUS_FLAG_POWERED	BIT(2)
#define STATUS_FLAG_POSITION	BIT(3)
#define STATUS_FLAG_BRAKE	BIT(4)
#define STATUS_FLAG_OVERLOAD	BIT(5)
#define STATUS_FLAG_TIMED	BIT(6)
#define STATUS_FLAG_STALL	BIT(7)

enum ms_nxtmmx_out_port_mode {
	MS_NXTMMX_OUT_PORT_MODE_TACHO_MOTOR,
	NUM_MS_NXTMMX_OUT_PORT_MODES
};

struct ms_nxtmmx_data {
	char address[LEGO_NAME_SIZE];
	struct lego_port_device port;
	struct i2c_client *i2c_client;
	struct lego_device *motor;
	int index;
	unsigned holding:1;
};

const struct device_type ms_nxtmmx_out_port_type = {
	.name = "ms-nxtmmx-out-port",
};

static const struct device_type
ms_nxtmmx_out_port_device_types[NUM_MS_NXTMMX_OUT_PORT_MODES] = {
	[MS_NXTMMX_OUT_PORT_MODE_TACHO_MOTOR] = {
		.name = "ev3-motor",
	},
};

static const char *ms_nxtmmx_out_port_default_driver[NUM_MS_NXTMMX_OUT_PORT_MODES] = {
	[MS_NXTMMX_OUT_PORT_MODE_TACHO_MOTOR]	= "lego-nxt-motor",
};

/*
 * Documentation is automatically generated from this struct, so formatting is
 * very important. Make sure any new modes have the same syntax. The comments
 * are also parsed to provide more information for the documentation. The
 * parser can be found in the ev3dev-kpkg repository.
 */

static const struct lego_port_mode_info ms_nxtmmx_out_port_mode_info[NUM_MS_NXTMMX_OUT_PORT_MODES] = {
	/**
	 * [^prefix]: The full address will be something like `in2:i2c3:M1`
	 * depending on what port the motor multiplexer is plugged into.
	 *
	 * @description: mindsensors.com NXTMMX Output Port
	 * @connection_types: tacho-motor
	 * @prefix: M
	 * @prefix_footnote: [^prefix]
	 */
	[MS_NXTMMX_OUT_PORT_MODE_TACHO_MOTOR] = {
		/**
		 * @description: NXT/EV3 Large Motor
		 */
		.name	= "tacho-motor",
	},
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
	scaled = min(100, scaled);
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

	*position = le32_to_cpup((__le32 *)bytes);
#ifdef PISTORMS_NXTMMX
	/* Motor rotation on PiStorms is backwards from standard rotation */
	*position *= -1;
#endif

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
	if (ret & STATUS_FLAG_STALL)
		state |= BIT(TM_STATE_STALLED);

	/*
	 * If motor is powered and it might be holding position, then we have
	 * to check the tasks register as well. If the tasks register is > 0,
	 * then we are still running, otherwise we are holding.
	 */
	if ((ret & STATUS_FLAG_POWERED) && (mmx->holding)) {
		ret = i2c_smbus_read_byte_data(mmx->i2c_client, READ_TASKS_REG(mmx->index));
		if (ret < 0)
			return ret;
		if (!ret)
			state |= BIT(TM_STATE_HOLDING);
	}

	return state;
}

static int ms_nxtmmx_run_regulated(void *context, int speed)
{
	struct ms_nxtmmx_data *mmx = context;
	u8 command_bytes[WRITE_SIZE] = { 0 };
	u8 command_flags;
	int err;

#ifdef PISTORMS_NXTMMX
	/* Motor rotation on PiStorms is backwards from standard rotation */
	speed *= -1;
#endif
	command_flags = CMD_FLAG_SPEED_CTRL | CMD_FLAG_RAMP;
	command_flags |= CMD_FLAG_GO;

	command_bytes[WRITE_SPEED] = ms_nxtmmx_scale_speed(speed);
	command_bytes[WRITE_COMMAND_A] = command_flags;

	err = i2c_smbus_write_i2c_block_data(mmx->i2c_client,
			WRITE_REG(mmx->index), WRITE_SIZE, command_bytes);
	if (err < 0)
		return err;

	mmx->holding = false;

	return 0;
}

static int ms_nxtmmx_run_to_pos(void *context, int pos, int speed,
				enum tacho_motor_stop_command stop_action)
{
	struct ms_nxtmmx_data *mmx = context;
	u8 command_bytes[WRITE_SIZE];
	u8 command_flags;
	int err;

#ifdef PISTORMS_NXTMMX
	/* Motor rotation on PiStorms is backwards from standard rotation */
	pos *= -1;
	speed *= -1;
#endif
	*(__le32 *)command_bytes = cpu_to_le32(pos);
	command_bytes[WRITE_SPEED] = ms_nxtmmx_scale_speed(speed);

	command_flags = CMD_FLAG_SPEED_CTRL | CMD_FLAG_ENCODER_CTRL
			| CMD_FLAG_RAMP;
	if (stop_action == TM_STOP_COMMAND_HOLD)
		command_flags |= CMD_FLAG_HOLD;
	if (stop_action == TM_STOP_COMMAND_BRAKE)
		command_flags |= CMD_FLAG_BRAKE;
	command_flags |= CMD_FLAG_GO;

	command_bytes[WRITE_TIME] = 0;
	command_bytes[WRITE_COMMAND_B] = 0;
	command_bytes[WRITE_COMMAND_A] = command_flags;

	err = i2c_smbus_write_i2c_block_data(mmx->i2c_client,
			WRITE_REG(mmx->index), WRITE_SIZE, command_bytes);
	if (err < 0)
		return err;

	mmx->holding = false;

	return 0;
}

static int ms_nxtmmx_stop(void *context, enum tacho_motor_stop_command action)
{
	struct ms_nxtmmx_data *mmx = context;
	u8 command_bytes[WRITE_SIZE];
	int err;

	command_bytes[0] = (action == TM_STOP_COMMAND_COAST)
		? COMMAND_FLOAT_STOP(mmx->index)
		: COMMAND_BRAKE_STOP(mmx->index);

	err = i2c_smbus_write_byte_data(mmx->i2c_client,
		COMMAND_REG, command_bytes[0]);
	if (err < 0)
		return err;

	mmx->holding = false;

	if (action == TM_STOP_COMMAND_HOLD) {
		/*
		 * Hold only happens when encoder mode is enabled, so
		 * we have to issue a run command to tell it to run to
		 * the current position so that it will hold that position.
		 */
		err = i2c_smbus_read_i2c_block_data(mmx->i2c_client,
			READ_ENCODER_POS_REG(mmx->index), ENCODER_SIZE,
			command_bytes);
		if (err < 0)
			return err;

		command_bytes[WRITE_SPEED] = 100;
		command_bytes[WRITE_TIME] = 0;
		command_bytes[WRITE_COMMAND_B] = 0;
		command_bytes[WRITE_COMMAND_A] = CMD_FLAGS_STOP_HOLD;

		err = i2c_smbus_write_i2c_block_data(mmx->i2c_client,
			WRITE_REG(mmx->index), WRITE_SIZE, command_bytes);
		if (err < 0)
			return err;

		mmx->holding = true;
	}

	return 0;
}

static int ms_nxtmmx_reset(void *context)
{
	struct ms_nxtmmx_data *mmx = context;
	int err;

	err = i2c_smbus_write_byte_data(mmx->i2c_client, COMMAND_REG,
					COMMAND_FLOAT_STOP(mmx->index));
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(mmx->i2c_client, COMMAND_REG,
					COMMAND_RESET_ENCODER(mmx->index));
	if (err < 0)
		return err;

	mmx->holding = false;

	return 0;
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
	.run_regulated		= ms_nxtmmx_run_regulated,
	.run_to_pos		= ms_nxtmmx_run_to_pos,
	.stop			= ms_nxtmmx_stop,
	.reset			= ms_nxtmmx_reset,
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

int ms_nxtmmx_out_port_register_motor(struct ms_nxtmmx_data *mmx,
				      const struct device_type *device_type,
				      const char *name)
{
	struct lego_device *new_motor;

	new_motor = lego_device_register(name, device_type,
					 &mmx->port, NULL, 0);
	if (IS_ERR(new_motor))
		return PTR_ERR(new_motor);

	mmx->motor = new_motor;

	return 0;
}

void ms_nxtmmx_out_port_unregister_motor(struct ms_nxtmmx_data *mmx)
{
	if (mmx->motor) {
		lego_device_unregister(mmx->motor);
		mmx->motor = NULL;
	}
}

static int ms_nxtmmx_out_port_set_mode(void *context, u8 mode)
{
	struct ms_nxtmmx_data *mmx = context;

	ms_nxtmmx_out_port_unregister_motor(mmx);

	return ms_nxtmmx_out_port_register_motor(mmx,
				&ms_nxtmmx_out_port_device_types[mode],
				ms_nxtmmx_out_port_default_driver[mode]);
}

void ms_nxtmmx_unregister_out_port(struct ms_nxtmmx_data *mmx)
{
	ms_nxtmmx_out_port_unregister_motor(mmx);
	lego_port_unregister(&mmx->port);
}

int ms_nxtmmx_register_out_port(struct ms_nxtmmx_data *mmx)
{
	struct lego_port_device *port = &mmx->port;
	int err;

	port->name = ms_nxtmmx_out_port_type.name;
	strncpy(port->address, mmx->address, LEGO_NAME_SIZE);
	port->num_modes = NUM_MS_NXTMMX_OUT_PORT_MODES;
	port->supported_modes = LEGO_PORT_ALL_MODES;
	port->mode_info = ms_nxtmmx_out_port_mode_info;
	port->set_mode = ms_nxtmmx_out_port_set_mode;
	port->tacho_motor_ops = &ms_nxtmmx_tacho_motor_ops;
	port->context = mmx;

	err = lego_port_register(port, &ms_nxtmmx_out_port_type, &mmx->i2c_client->dev);
	if (err)
		return err;
	err = ms_nxtmmx_out_port_set_mode(mmx, MS_NXTMMX_OUT_PORT_MODE_TACHO_MOTOR);
	if (err) {
		lego_port_unregister(port);
		return err;
	}

	return 0;
}

#ifndef PISTORMS_NXTMMX

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
		snprintf(mmx[i].address, LEGO_NAME_SIZE, "%s:M%d",
			 data->address, i + 1);
		mmx[i].i2c_client = data->client;
		mmx[i].index = i;
	}
	err = ms_nxtmmx_register_out_port(&mmx[0]);
	if (err)
		goto err_register_out_port0;
	err = ms_nxtmmx_register_out_port(&mmx[1]);
	if (err)
		goto err_register_out_port1;

	data->poll_ms = 1000;

	return 0;

err_register_out_port1:
	ms_nxtmmx_unregister_out_port(&mmx[0]);
err_register_out_port0:
	data->callback_data = NULL;
	kfree(mmx);

	return err;
}

void ms_nxtmmx_remove_cb(struct nxt_i2c_sensor_data *data)
{
	struct ms_nxtmmx_data *mmx = data->callback_data;

	ms_nxtmmx_unregister_out_port(&mmx[1]);
	ms_nxtmmx_unregister_out_port(&mmx[0]);
	data->callback_data = NULL;
	kfree(mmx);
}

# endif
