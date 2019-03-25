/*
 * LEGO MINDSTORMS EV3 UART Sensor tty line discipline
 *
 * Copyright (C) 2014-2016,2018 David Lechner <david@lechnology.com>
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
 * The ``ev3-uart-sensor-ld`` module is a tty `line discipline`_ that runs on
 * top of a tty. It listens for the information data that is sent from EV3/UART
 * sensors. When it receives valid data, it negotiates with the sensor, telling
 * the sensor to enter data sending mode.
 *
 * This line discipline has been assigned the number 29. To attach this line
 * discipline to a tty, run ``ldattach 29 /dev/tty<N>`` where ``<N>`` is the name
 * of the tty you want to connect to.
 *
 * .. note:: This driver `works with any tty`_, which means the sensor does not
 *    necessarily have to be plugged into one of the input ports on the EV3.
 *
 * EV3/UART sensors do not require individual driver implementations like other
 * types of sensors. Instead, all of the needed info to sent from the sensor in
 * a common format.  As a result, the name returned by the ``driver_name``
 * attribute may not be a real driver name. For well-known sensors (the LEGO
 * EV3 sensors and FatcatLab sensors) it will return a name like ``lego-ev3-color``.
 * For unknown sensors it returns ``ev3-uart-<N>``, where ``<N>`` is the type id
 * of the sensor.
 *
 * .. _line discipline: https://en.wikipedia.org/wiki/Line_discipline
 * .. _works with any tty: http://lechnology.com/2014/09/using-uart-sensors-on-any-linux/
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/tty.h>

#include <lego.h>
#include <lego_port_class.h>
#include <lego_sensor_class.h>

#include "ev3_uart_sensor.h"

#ifdef DEBUG
#define debug_pr(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define debug_pr(fmt, ...) while(0) { }
#endif

#ifndef N_LEGOEV3
#define N_LEGOEV3 29
#endif

#define EV3_UART_MAX_DATA_SIZE		32
#define EV3_UART_MAX_MESSAGE_SIZE	(EV3_UART_MAX_DATA_SIZE + 2)

#define EV3_UART_MSG_TYPE_MASK		0xC0
#define EV3_UART_CMD_SIZE(byte)		(1 << (((byte) >> 3) & 0x7))
#define EV3_UART_MSG_CMD_MASK		0x07
#define EV3_UART_MAX_DATA_ERR		6

#define EV3_UART_TYPE_MAX		101
#define EV3_UART_TYPE_UNKNOWN		125
#define EV3_UART_SPEED_MIN		2400
#define EV3_UART_SPEED_MID		57600
#define EV3_UART_SPEED_MAX		460800
#define EV3_UART_MODE_MAX		7
#define EV3_UART_MODE_NAME_SIZE		11

#define EV3_UART_DATA_KEEP_ALIVE_TIMEOUT	100 /* msec */

#define EV3_UART_DEVICE_TYPE_NAME_SIZE		30
#define EV3_UART_UNITS_SIZE			4

enum ev3_uart_msg_type {
	EV3_UART_MSG_TYPE_SYS	= 0x00,
	EV3_UART_MSG_TYPE_CMD	= 0x40,
	EV3_UART_MSG_TYPE_INFO	= 0x80,
	EV3_UART_MSG_TYPE_DATA	= 0xC0,
};

enum ev3_uart_sys {
	EV3_UART_SYS_SYNC	= 0x0,
	EV3_UART_SYS_NACK	= 0x2,
	EV3_UART_SYS_ACK	= 0x4,
	EV3_UART_SYS_ESC	= 0x6,
};

enum ev3_uart_cmd {
	EV3_UART_CMD_TYPE	= 0x0,
	EV3_UART_CMD_MODES	= 0x1,
	EV3_UART_CMD_SPEED	= 0x2,
	EV3_UART_CMD_SELECT	= 0x3,
	EV3_UART_CMD_WRITE	= 0x4,
};

enum ev3_uart_info {
	EV3_UART_INFO_NAME	= 0x00,
	EV3_UART_INFO_RAW	= 0x01,
	EV3_UART_INFO_PCT	= 0x02,
	EV3_UART_INFO_SI	= 0x03,
	EV3_UART_INFO_UNITS	= 0x04,
	EV3_UART_INFO_FORMAT	= 0x80,
};

#define EV3_UART_INFO_BIT_CMD_TYPE	0
#define EV3_UART_INFO_BIT_CMD_MODES	1
#define EV3_UART_INFO_BIT_CMD_SPEED	2
#define EV3_UART_INFO_BIT_INFO_NAME	3
#define EV3_UART_INFO_BIT_INFO_RAW	4
#define EV3_UART_INFO_BIT_INFO_PCT	5
#define EV3_UART_INFO_BIT_INFO_SI	6
#define EV3_UART_INFO_BIT_INFO_UNITS	7
#define EV3_UART_INFO_BIT_INFO_FORMAT	8

enum ev3_uart_data_type {
	EV3_UART_DATA_8		= 0x00,
	EV3_UART_DATA_16	= 0x01,
	EV3_UART_DATA_32	= 0x02,
	EV3_UART_DATA_FLOAT	= 0x03,
};

enum ev3_uart_info_flags {
	EV3_UART_INFO_FLAG_CMD_TYPE	= BIT(EV3_UART_INFO_BIT_CMD_TYPE),
	EV3_UART_INFO_FLAG_CMD_MODES	= BIT(EV3_UART_INFO_BIT_CMD_MODES),
	EV3_UART_INFO_FLAG_CMD_SPEED	= BIT(EV3_UART_INFO_BIT_CMD_SPEED),
	EV3_UART_INFO_FLAG_INFO_NAME	= BIT(EV3_UART_INFO_BIT_INFO_NAME),
	EV3_UART_INFO_FLAG_INFO_RAW	= BIT(EV3_UART_INFO_BIT_INFO_RAW),
	EV3_UART_INFO_FLAG_INFO_PCT	= BIT(EV3_UART_INFO_BIT_INFO_PCT),
	EV3_UART_INFO_FLAG_INFO_SI	= BIT(EV3_UART_INFO_BIT_INFO_SI),
	EV3_UART_INFO_FLAG_INFO_UNITS	= BIT(EV3_UART_INFO_BIT_INFO_UNITS),
	EV3_UART_INFO_FLAG_INFO_FORMAT	= BIT(EV3_UART_INFO_BIT_INFO_FORMAT),
	EV3_UART_INFO_FLAG_ALL_INFO	= EV3_UART_INFO_FLAG_INFO_NAME
					| EV3_UART_INFO_FLAG_INFO_RAW
					| EV3_UART_INFO_FLAG_INFO_PCT
					| EV3_UART_INFO_FLAG_INFO_SI
					| EV3_UART_INFO_FLAG_INFO_UNITS
					| EV3_UART_INFO_FLAG_INFO_FORMAT,
	EV3_UART_INFO_FLAG_REQUIRED	= EV3_UART_INFO_FLAG_CMD_TYPE
					| EV3_UART_INFO_FLAG_CMD_MODES
					| EV3_UART_INFO_FLAG_INFO_NAME
					| EV3_UART_INFO_FLAG_INFO_FORMAT,
};

#define EV3_UART_TYPE_ID_COLOR		29
#define EV3_UART_TYPE_ID_ULTRASONIC	30
#define EV3_UART_TYPE_ID_GYRO		32
#define EV3_UART_TYPE_ID_INFRARED	33

/**
 * struct ev3_uart_data - Discipline data for EV3 UART Sensor communication
 * @device_name: The name of the device/driver.
 * @tty: Pointer to the tty device that the sensor is connected to
 * @in_port: The input port device associated with this tty.
 * @sensor: The lego-sensor class structure for the sensor.
 * @change_bitrate_work: Used to change the baud rate after a delay.
 * @keep_alive_timer: Sends a NACK every 100usec when a sensor is connected.
 * @keep_alive_tasklet: Does the actual sending of the NACK.
 * @set_mode_completion: Used to block until confirmation has been received from
 * 	the sensor that the mode was actually changed.
 * @mode_info: Array of information about each mode of the sensor
 * @requested_mode: Mode that was requested by user. Used to restore previous
 * 	mode in case of a reconnect.
 * @type_id: Type id returned by the sensor
 * @new_mode: The mode requested by set_mode.
 * @raw_min: Min/max values are sent as float data types. This holds the value
 * 	until we read the number of decimal places needed to convert this
 * 	value to an integer.
 * @raw_max: See raw_min.
 * @pct_min: See raw_min.
 * @pct_max: See raw_min.
 * @si_min: See raw_min.
 * @si_max: See raw_min.
 * @new_baud_rate: New baud rate that will be set with ev3_uart_change_bitrate
 * @info_flags: Flags indicating what information has already been read
 * 	from the sensor.
 * @msg: partial message from previous receive callback
 * @partial_msg_size: the size of the partial message
 * @last_err: Message to be printed in case of an error.
 * @num_data_err: Number of bad reads when receiving DATA messages.
 * @synced: Flag indicating communications are synchronized with the sensor.
 * @info_done: Flag indicating that all mode info has been received and it is
 * 	OK to start receiving DATA messages.
 * @data_rec: Flag that indicates that good DATA message has been received
 * 	since last watchdog timeout.
 * @closing: Flag to indicate that we are closing the connection and any data
 * 	received should be ignored.
 */
struct ev3_uart_port_data {
	char device_name[LEGO_NAME_SIZE + 1];
	struct tty_struct *tty;
	struct lego_port_device *in_port;
	struct lego_sensor_device sensor;
	struct work_struct change_bitrate_work;
	struct hrtimer keep_alive_timer;
	struct tasklet_struct keep_alive_tasklet;
	struct completion set_mode_completion;
	struct lego_sensor_mode_info mode_info[EV3_UART_MODE_MAX + 1];
	u8 requested_mode;
	u8 type_id;
	u8 new_mode;
	u32 raw_min;
	u32 raw_max;
	u32 pct_min;
	u32 pct_max;
	u32 si_min;
	u32 si_max;
	speed_t new_baud_rate;
	long unsigned info_flags;
	u8 msg[EV3_UART_MAX_MESSAGE_SIZE];
	u8 partial_msg_size;
	char *last_err;
	unsigned num_data_err;
	unsigned synced:1;
	unsigned info_done:1;
	unsigned data_rec:1;
	unsigned closing:1;
};

u8 ev3_uart_set_msg_hdr(u8 type, const unsigned long size, u8 cmd)
{
	u8 size_code = (find_last_bit(&size, sizeof(unsigned long)) & 0x7) << 3;

	return (type & EV3_UART_MSG_TYPE_MASK) | size_code
		| (cmd & EV3_UART_MSG_CMD_MASK);
}

static struct lego_sensor_mode_info ev3_uart_default_mode_info = {
	.raw_max = 1023,
	.pct_max = 100,
	.si_max = 1,
	.figures = 4,
};

static inline int ev3_uart_msg_size(u8 header)
{
	int size;

	if (!(header & EV3_UART_MSG_TYPE_MASK)) /* SYNC, NACK, ACK */
		return 1;

	size = EV3_UART_CMD_SIZE(header);
	size += 2; /* header and checksum */
	if ((header & EV3_UART_MSG_TYPE_MASK) == EV3_UART_MSG_TYPE_INFO)
		size++; /* extra command byte */

	return size;
}

int ev3_uart_write_byte(struct tty_struct *tty, const u8 byte)
{
	int ret;

	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	ret = tty_put_char(tty, byte);
	if (tty->ops->flush_chars)
		tty->ops->flush_chars(tty);

	return ret;
}

int ev3_uart_set_mode(void *context, const u8 mode)
{
	struct tty_struct *tty = context;
	struct ev3_uart_port_data *port;
	const int data_size = 3;
	u8 data[data_size];
	int retries = 10;
	int ret;

	if (!tty)
		return -ENODEV;

	port = tty->disc_data;
	if (!port->synced || !port->info_done)
		return -ENODEV;
	if (mode >= port->sensor.num_modes)
		return -EINVAL;
	if (!completion_done(&port->set_mode_completion))
		return -EBUSY;

	data[0] = ev3_uart_set_msg_hdr(EV3_UART_MSG_TYPE_CMD, data_size - 2,
				       EV3_UART_CMD_SELECT);
	data[1] = mode;
	data[2] = 0xFF ^ data[0] ^ data[1];

	port->new_mode = mode;
	reinit_completion(&port->set_mode_completion);
	while (retries--) {
		set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
		ret = tty->ops->write(tty, data, data_size);
		if (ret < 0)
			return ret;

		ret = wait_for_completion_timeout(&port->set_mode_completion,
						  msecs_to_jiffies(50));
		if (ret)
			break;
	}
	port->set_mode_completion.done++;
	if (!ret)
		return -ETIMEDOUT;

	port->requested_mode = mode;

	return 0;
}

static ssize_t ev3_uart_direct_write(void *context, char *data, loff_t off,
				     size_t count)
{
	struct tty_struct *tty = context;
	char uart_data[EV3_UART_MAX_MESSAGE_SIZE];
	int size, i, err;

	if (off != 0 || count > EV3_UART_MAX_DATA_SIZE)
		return -EINVAL;
	if (count == 0)
		return count;
	memset(uart_data + 1, 0, EV3_UART_MAX_DATA_SIZE);
	memcpy(uart_data + 1, data, count);
	if (count <= 2)
		size = count;
	else if (count <= 4)
		size = 4;
	else if (count <= 8)
		size = 8;
	else if (count <= 16)
		size = 16;
	else
		size = 32;
	uart_data[0] = ev3_uart_set_msg_hdr(EV3_UART_MSG_TYPE_CMD, size,
					    EV3_UART_CMD_WRITE);
	data[size + 1] = 0xFF;
	for (i = 0; i <= size; i++)
		uart_data[size + 1] ^= uart_data[i];
	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	err = tty->ops->write(tty, uart_data, size + 2);
	if (err < 0)
		return err;

	return count;
}

int ev3_uart_match_input_port(struct device *dev, const void *data)
{
	struct lego_port_device *pdev = to_lego_port_device(dev);
	const char *tty_name = data;

	if (!pdev->port_alias)
		return 0;

	return !strcmp(pdev->port_alias, tty_name);
}

static void ev3_uart_change_bitrate(struct ev3_uart_port_data *port)
{
	struct ktermios old_termios = port->tty->termios;

	tty_wait_until_sent(port->tty, 0);
	down_write(&port->tty->termios_rwsem);
	tty_encode_baud_rate(port->tty, port->new_baud_rate, port->new_baud_rate);
	if (port->tty->ops->set_termios)
		port->tty->ops->set_termios(port->tty, &old_termios);
	up_write(&port->tty->termios_rwsem);
	if (port->info_done) {
		hrtimer_start(&port->keep_alive_timer, ktime_set(0, 1000000),
			      HRTIMER_MODE_REL);
		/* restore the previous user-selected mode */
		if (port->sensor.mode != port->requested_mode)
			ev3_uart_set_mode(port->tty, port->requested_mode);
	}
}

static void ev3_uart_send_ack(struct ev3_uart_port_data *port)
{
	int err;

	ev3_uart_write_byte(port->tty, EV3_UART_SYS_ACK);
	if (!port->sensor.context && port->type_id <= EV3_UART_TYPE_MAX) {
		port->sensor.context = port->tty;
		err = register_lego_sensor(&port->sensor, port->tty->dev);
		if (err < 0) {
			port->sensor.context = NULL;
			if (port->in_port) {
				put_device(&port->in_port->dev);
				port->in_port = NULL;
			}
			dev_err(port->tty->dev,
				"Could not register UART sensor on tty %s",
				port->tty->name);
			return;
		}
	} else {
		dev_err(port->tty->dev, "Reconnected due to: %s\n",
			port->last_err);
	}

	mdelay(4);
	ev3_uart_change_bitrate(port);
}

static void ev3_uart_change_bitrate_work(struct work_struct *work)
{
	struct ev3_uart_port_data *port = container_of(work,
				struct ev3_uart_port_data, change_bitrate_work);

	ev3_uart_change_bitrate(port);
}

static void ev3_uart_send_keep_alive(unsigned long data)
{
	struct tty_struct *tty = (void *)data;

	/* NACK is sent as a keep-alive */
	ev3_uart_write_byte(tty, EV3_UART_SYS_NACK);
}

enum hrtimer_restart ev3_uart_keep_alive_timer_callback(struct hrtimer *timer)
{
	struct ev3_uart_port_data *port = container_of(timer,
				struct ev3_uart_port_data, keep_alive_timer);

	if (!port->synced || !port->info_done)
		return HRTIMER_NORESTART;

	hrtimer_forward_now(timer, ktime_set(0,
			    EV3_UART_DATA_KEEP_ALIVE_TIMEOUT * 1000000));
	if (!port->data_rec) {
		port->last_err = "No data since last keep-alive.";
		port->num_data_err++;
		if (port->num_data_err > EV3_UART_MAX_DATA_ERR) {
			port->synced = 0;
			port->partial_msg_size = 0;
			port->new_baud_rate = EV3_UART_SPEED_MIN;
			schedule_work(&port->change_bitrate_work);
			return HRTIMER_NORESTART;
		}
	}
	port->data_rec = 0;

	tasklet_schedule(&port->keep_alive_tasklet);

	return HRTIMER_RESTART;
}

static int ev3_uart_receive_buf2(struct tty_struct *tty,
				 const unsigned char *cp, char *fp, int count)
{
	struct ev3_uart_port_data *port = tty->disc_data;
	u8 message[EV3_UART_MAX_MESSAGE_SIZE];
	int i, speed, pos;
	u8 cmd, cmd2, type, mode, msg_type, msg_size, chksum;

	if (fp) {
		switch (*fp) {
		case TTY_NORMAL:
			break;
		case TTY_BREAK:
			port->last_err = "received break";
			goto err_invalid_state;
		case TTY_PARITY:
			port->last_err = "parity error";
			goto err_invalid_state;
		case TTY_FRAME:
			port->last_err = "frame error";
			goto err_invalid_state;
		case TTY_OVERRUN:
			port->last_err = "frame overrun";
			goto err_invalid_state;
		default:
			port->last_err = "receive_error - unknown flag";
			goto err_invalid_state;
		}
	}

	/* current position in *cp */
	pos = 0;

	/*
	 * To get in sync with the data stream from the sensor, we look
	 * for a valid TYPE command.
	 */
	while (!port->synced) {
		int num = min(count - pos, 3 - port->partial_msg_size);

		/* we are out of new data */
		if (num <= 0)
			return pos ? pos : count;

		/* collect up to 3 bytes in port->msg */
		memcpy(port->msg + port->partial_msg_size, cp + pos, num);
		port->partial_msg_size += num;
		pos += num;

		/* return early if we don't have 3 bytes yet */
		if (port->partial_msg_size < 3)
			return pos;

		cmd = port->msg[0];
		if (cmd != (EV3_UART_MSG_TYPE_CMD | EV3_UART_CMD_TYPE)) {
			port->msg[0] = port->msg[1];
			port->msg[1] = port->msg[2];
			port->partial_msg_size--;
			continue;
		}

		type = port->msg[1];
		if (!type || type > EV3_UART_TYPE_MAX) {
			port->msg[0] = port->msg[1];
			port->msg[1] = port->msg[2];
			port->partial_msg_size--;
			continue;
		}

		chksum = 0xFF ^ cmd ^ type;
		if (port->msg[2] != chksum) {
			port->msg[0] = port->msg[1];
			port->msg[1] = port->msg[2];
			port->partial_msg_size--;
			continue;
		}

		port->sensor.num_modes = 1;
		port->sensor.num_view_modes = 1;

		for (i = 0; i <= EV3_UART_MODE_MAX; i++)
			port->mode_info[i] = ev3_uart_default_mode_info;

		port->type_id = type;
		port->device_name[0] = 0;

		/* look up well-known driver names */
		for (i = 0; i < NUM_LEGO_EV3_SENSOR_TYPES; i++) {
			if (type == ev3_uart_sensor_defs[i].type_id) {
				snprintf(port->device_name, LEGO_SENSOR_NAME_SIZE,
					 "%s", ev3_uart_sensor_defs[i].name);
				break;
			}
		}

		/* or use generic name if well-known name is not found */
		if (!port->device_name[0])
			snprintf(port->device_name, LEGO_SENSOR_NAME_SIZE,
				 EV3_UART_SENSOR_NAME("%u"), type);

		port->partial_msg_size = 0;
		port->info_flags = EV3_UART_INFO_FLAG_CMD_TYPE;
		port->info_done = 0;
		port->data_rec = 0;
		port->num_data_err = 0;
		port->synced = 1;
	}

	while (pos < count) {
		if (port->partial_msg_size) {
			msg_size = ev3_uart_msg_size(port->msg[0]);
		} else if (cp[pos] == 0xFF) {
			/*
			* Sometimes we get 0xFF after switching baud rates, so
			* just ignore it.
			*/
			pos++;
			continue;
		} else {
			msg_size = ev3_uart_msg_size(cp[pos]);
		}

		if (msg_size > EV3_UART_MAX_MESSAGE_SIZE) {
			debug_pr("header: 0x%02x\n", cp[pos]);
			port->last_err = "Bad message size";
			goto err_invalid_state;
		}

		if (pos + msg_size - port->partial_msg_size > count) {
			/*
			 * Don't have a full message. Need to read drain the
			 * tty buffer so that we can get the next buffer.
			 */
			memcpy(port->msg + port->partial_msg_size, cp + pos, count - pos);
			port->partial_msg_size += count - pos;
			return count;
		}

		if (port->partial_msg_size) {
			/*
			 * If we had a parital message, concat the partial
			 * message and the remainder of the message we just
			 * received.
			 */
			memcpy(message, port->msg, port->partial_msg_size);
			memcpy(message + port->partial_msg_size, cp + pos,
			       msg_size - port->partial_msg_size);
			pos += msg_size - port->partial_msg_size;
			port->partial_msg_size = 0;
		} else {
			memcpy(message, cp + pos, msg_size);
			pos += msg_size;
		}

		msg_type = message[0] & EV3_UART_MSG_TYPE_MASK;
		cmd = message[0] & EV3_UART_MSG_CMD_MASK;
		mode = cmd;
		cmd2 = message[1];

		if (msg_size > 1) {
			chksum = 0xFF;
			for (i = 0; i < msg_size - 1; i++)
				chksum ^= message[i];
			debug_pr("chksum:%d, actual:%d\n",
				 chksum, message[msg_size - 1]);
			/*
			 * The LEGO EV3 color sensor sends bad checksums
			 * for RGB-RAW data (mode 4). The check here could be
			 * improved if someone can find a pattern.
			 */
			if (chksum != message[msg_size - 1]
			    && port->type_id != EV3_UART_TYPE_ID_COLOR
			    && message[0] != 0xDC)
			{
				port->last_err = "Bad checksum.";
				if (port->info_done) {
					port->num_data_err++;
					continue;
				} else {
					goto err_invalid_state;
				}
			}
		}

		switch (msg_type) {
		case EV3_UART_MSG_TYPE_SYS:
			debug_pr("SYS:%d\n", message[0] & EV3_UART_MSG_CMD_MASK);
			switch(cmd) {
			case EV3_UART_SYS_SYNC:
				/* IR sensor (type 33) sends checksum after SYNC */
				if (msg_size > 1 && (cmd ^ cmd2) == 0xFF)
					msg_size++;
				break;
			case EV3_UART_SYS_ACK:
				if (!port->sensor.num_modes) {
					port->last_err = "Received ACK before all mode INFO.";
					goto err_invalid_state;
				}
				if ((port->info_flags & EV3_UART_INFO_FLAG_REQUIRED)
				    != EV3_UART_INFO_FLAG_REQUIRED)
				{
					port->last_err = "Did not receive all required INFO.";
					goto err_invalid_state;
				}

				port->info_done = 1;

				mdelay(10);
				ev3_uart_send_ack(port);

				return count;
			}
			break;
		case EV3_UART_MSG_TYPE_CMD:
			debug_pr("CMD:%d\n", cmd);
			switch (cmd) {
			case EV3_UART_CMD_MODES:
				if (test_and_set_bit(EV3_UART_INFO_BIT_CMD_MODES,
						     &port->info_flags))
				{
					port->last_err = "Received duplicate modes INFO.";
					goto err_invalid_state;
				}
				if (cmd2 > EV3_UART_MODE_MAX) {
					port->last_err = "Number of modes is out of range.";
					goto err_invalid_state;
				}
				port->sensor.num_modes = cmd2 + 1;
				if (msg_size > 3)
					port->sensor.num_view_modes = message[2] + 1;
				else
					port->sensor.num_view_modes = port->sensor.num_modes;
				debug_pr("num_modes:%d, num_view_modes:%d\n",
					 port->sensor.num_modes, port->sensor.num_view_modes);
				break;
			case EV3_UART_CMD_SPEED:
				if (test_and_set_bit(EV3_UART_INFO_BIT_CMD_SPEED,
						     &port->info_flags))
				{
					port->last_err = "Received duplicate speed INFO.";
					goto err_invalid_state;
				}
				speed = *(int*)(message + 1);
				if (speed < EV3_UART_SPEED_MIN
				    || speed > EV3_UART_SPEED_MAX)
				{
					port->last_err = "Speed is out of range.";
					goto err_invalid_state;
				}
				port->new_baud_rate = speed;
				debug_pr("speed:%d\n", speed);
				break;
			default:
				port->last_err = "Unknown command.";
				goto err_invalid_state;
			}
			break;
		case EV3_UART_MSG_TYPE_INFO:
			debug_pr("INFO:%d, mode:%d\n", cmd2, mode);
			switch (cmd2) {
			case EV3_UART_INFO_NAME:
				port->info_flags &= ~EV3_UART_INFO_FLAG_ALL_INFO;
				if (message[2] < 'A' || message[2] > 'z') {
					port->last_err = "Invalid name INFO.";
					goto err_invalid_state;
				}
				/*
				 * Name may not have null terminator and we
				 * are done with the checksum at this point
				 * so we are writing 0 over the checksum to
				 * ensure a null terminator for the string
				 * functions.
				 */
				message[msg_size - 1] = 0;
				if (strlen(message + 2) > EV3_UART_MODE_NAME_SIZE) {
					port->last_err = "Name is too long.";
					goto err_invalid_state;
				}
				snprintf(port->mode_info[mode].name,
				         EV3_UART_MODE_NAME_SIZE + 1, "%s",
				         message + 2);
				port->info_flags |= EV3_UART_INFO_FLAG_INFO_NAME;
				debug_pr("mode %d name:%s\n",
				       mode, port->sensor.address);
				break;
			case EV3_UART_INFO_RAW:
				if (port->sensor.mode != mode) {
					port->last_err = "Received INFO for incorrect mode.";
					goto err_invalid_state;
				}
				if (test_and_set_bit(EV3_UART_INFO_BIT_INFO_RAW,
						     &port->info_flags))
				{
					port->last_err = "Received duplicate raw scaling INFO.";
					goto err_invalid_state;
				}
				port->raw_min = *(u32 *)(message + 2);
				port->raw_max = *(u32 *)(message + 6);
				debug_pr("mode %d raw_min:%08x, raw_max:%08x\n",
				       mode, port->mode_info[mode].raw_min,
				       port->mode_info[mode].raw_max);
				break;
			case EV3_UART_INFO_PCT:
				if (port->sensor.mode != mode) {
					port->last_err = "Received INFO for incorrect mode.";
					goto err_invalid_state;
				}
				if (test_and_set_bit(EV3_UART_INFO_BIT_INFO_PCT,
						     &port->info_flags))
				{
					port->last_err = "Received duplicate percent scaling INFO.";
					goto err_invalid_state;
				}
				port->pct_min = *(u32 *)(message + 2);
				port->pct_max = *(u32 *)(message + 6);
				debug_pr("mode %d pct_min:%08x, pct_max:%08x\n",
				       mode, port->mode_info[mode].pct_min,
				       port->mode_info[mode].pct_max);
				break;
			case EV3_UART_INFO_SI:
				if (port->sensor.mode != mode) {
					port->last_err = "Received INFO for incorrect mode.";
					goto err_invalid_state;
				}
				if (test_and_set_bit(EV3_UART_INFO_BIT_INFO_SI,
						     &port->info_flags))
				{
					port->last_err = "Received duplicate SI scaling INFO.";
					goto err_invalid_state;
				}
				port->si_min = *(u32 *)(message + 2);
				port->si_max = *(u32 *)(message + 6);
				debug_pr("mode %d si_min:%08x, si_max:%08x\n",
				       mode, port->mode_info[mode].si_min,
				       port->mode_info[mode].si_max);
				break;
			case EV3_UART_INFO_UNITS:
				if (port->sensor.mode != mode) {
					port->last_err = "Received INFO for incorrect mode.";
					goto err_invalid_state;
				}
				if (test_and_set_bit(EV3_UART_INFO_BIT_INFO_UNITS,
						     &port->info_flags))
				{
					port->last_err = "Received duplicate SI units INFO.";
					goto err_invalid_state;
				}
				/*
				 * Units may not have null terminator and we
				 * are done with the checksum at this point
				 * so we are writing 0 over the checksum to
				 * ensure a null terminator for the string
				 * functions.
				 */
				message[msg_size - 1] = 0;
				snprintf(port->mode_info[mode].units,
					 EV3_UART_UNITS_SIZE + 1, "%s",
					 message + 2);
				debug_pr("mode %d units:%s\n",
				       mode, port->mode_info[mode].units);
				break;
			case EV3_UART_INFO_FORMAT:
				if (port->sensor.mode != mode) {
					port->last_err = "Received INFO for incorrect mode.";
					goto err_invalid_state;
				}
				if (test_and_set_bit(EV3_UART_INFO_BIT_INFO_FORMAT,
						     &port->info_flags))
				{
					port->last_err = "Received duplicate format INFO.";
					goto err_invalid_state;
				}
				port->mode_info[mode].data_sets = message[2];
				if (!port->mode_info[mode].data_sets) {
					port->last_err = "Invalid number of data sets.";
					goto err_invalid_state;
				}
				if (msg_size < 7) {
					port->last_err = "Invalid format message size.";
					goto err_invalid_state;
				}
				if ((port->info_flags & EV3_UART_INFO_FLAG_REQUIRED)
						!= EV3_UART_INFO_FLAG_REQUIRED) {
					port->last_err = "Did not receive all required INFO.";
					goto err_invalid_state;
				}
				switch (message[3]) {
				case EV3_UART_DATA_8:
					port->mode_info[mode].data_type = LEGO_SENSOR_DATA_S8;
					break;
				case EV3_UART_DATA_16:
					port->mode_info[mode].data_type = LEGO_SENSOR_DATA_S16;
					break;
				case EV3_UART_DATA_32:
					port->mode_info[mode].data_type = LEGO_SENSOR_DATA_S32;
					break;
				case EV3_UART_DATA_FLOAT:
					port->mode_info[mode].data_type = LEGO_SENSOR_DATA_FLOAT;
					break;
				default:
					port->last_err = "Invalid data type.";
					goto err_invalid_state;
				}
				port->mode_info[mode].figures = message[4];
				port->mode_info[mode].decimals = message[5];
				if (port->info_flags & EV3_UART_INFO_FLAG_INFO_RAW) {
					port->mode_info[mode].raw_min =
						lego_sensor_ftoi(port->raw_min, 0);
					port->mode_info[mode].raw_max =
						lego_sensor_ftoi(port->raw_max, 0);
				}
				if (port->info_flags & EV3_UART_INFO_FLAG_INFO_PCT) {
					port->mode_info[mode].pct_min =
						lego_sensor_ftoi(port->pct_min, 0);
					port->mode_info[mode].pct_max =
						lego_sensor_ftoi(port->pct_max, 0);
				}
				if (port->info_flags & EV3_UART_INFO_FLAG_INFO_SI) {
					port->mode_info[mode].si_min =
						lego_sensor_ftoi(port->si_min,
							port->mode_info[mode].decimals);
					port->mode_info[mode].si_max =
						lego_sensor_ftoi(port->si_max,
							port->mode_info[mode].decimals);
				}
				if (port->sensor.mode)
					port->sensor.mode--;
				debug_pr("mode %d - data_sets:%d, data_type:%d, figures:%d, decimals:%d\n",
					 mode, port->mode_info[mode].data_sets,
					 port->mode_info[mode].data_type,
					 port->mode_info[mode].figures,
					 port->mode_info[mode].decimals);
				debug_pr("raw_min: %d, raw_max: %d\n",
					 port->mode_info[mode].raw_min,
					 port->mode_info[mode].raw_max);
				debug_pr("pct_min: %d, pct_max: %d\n",
					 port->mode_info[mode].pct_min,
					 port->mode_info[mode].pct_max);
				debug_pr("si_min: %d, si_max: %d\n",
					 port->mode_info[mode].si_min,
					 port->mode_info[mode].si_max);
				break;
			}
			break;
		case EV3_UART_MSG_TYPE_DATA:
			debug_pr("DATA:%d\n", message[0] & EV3_UART_MSG_CMD_MASK);
			if (!port->info_done) {
				port->last_err = "Received DATA before INFO was complete.";
				goto err_invalid_state;
			}
			if (mode > EV3_UART_MODE_MAX) {
				port->last_err = "Invalid mode received.";
				goto err_invalid_state;
			}
			if (mode != port->sensor.mode) {
				if (mode == port->new_mode) {
					port->sensor.mode = mode;
					kobject_uevent(&port->sensor.dev.kobj,
						       KOBJ_CHANGE);
				} else {
					port->last_err = "Unexpected mode.";
					goto err_invalid_state;
				}
			}
			if (!completion_done(&port->set_mode_completion)
			    && mode == port->new_mode)
				complete(&port->set_mode_completion);
			memcpy(port->mode_info[mode].raw_data, message + 1, msg_size - 2);
			port->data_rec = 1;
			if (port->num_data_err)
				port->num_data_err--;
			break;
		}
	}

	return pos;

err_invalid_state:
	debug_pr("invalid state: %s\n", port->last_err);
	port->synced = 0;
	port->partial_msg_size = 0;
	port->new_baud_rate = EV3_UART_SPEED_MIN;
	schedule_work(&port->change_bitrate_work);

	return count;
}

static int ev3_uart_open(struct tty_struct *tty)
{
	struct ktermios old_termios = tty->termios;
	struct ev3_uart_port_data *port;
	struct device *in_port_dev;

	port = kzalloc(sizeof(struct ev3_uart_port_data), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	port->tty = tty;
	port->new_baud_rate = EV3_UART_SPEED_MIN;
	port->type_id = EV3_UART_TYPE_UNKNOWN;
	port->sensor.name = port->device_name;
	/*
	 * This is a special case for the input ports on the EV3 brick.
	 * We use the name of the input port instead of the tty to make
	 * it easier to know which sensor is which.
	 */
	in_port_dev = class_find_device(&lego_port_class, NULL,
					port->tty->name,
					ev3_uart_match_input_port);
	if (in_port_dev) {
		port->in_port = to_lego_port_device(in_port_dev);
		port->sensor.address = port->in_port->address;
	} else {
		port->sensor.address = port->tty->name;
	}
	port->sensor.mode_info = port->mode_info;
	port->sensor.set_mode = ev3_uart_set_mode;
	port->sensor.direct_write = ev3_uart_direct_write;
	INIT_WORK(&port->change_bitrate_work, ev3_uart_change_bitrate_work);
	hrtimer_init(&port->keep_alive_timer, HRTIMER_BASE_MONOTONIC,
		     HRTIMER_MODE_REL);
	port->keep_alive_timer.function = ev3_uart_keep_alive_timer_callback;
	tasklet_init(&port->keep_alive_tasklet, ev3_uart_send_keep_alive,
		     (unsigned long)tty);
	init_completion(&port->set_mode_completion);
	tty->disc_data = port;

	/* set baud rate and other port settings */
	down_write(&tty->termios_rwsem);
	tty->termios.c_iflag &=
		~(IGNBRK	/* disable ignore break */
		| BRKINT	/* disable break causes interrupt */
		| PARMRK	/* disable mark parity errors */
		| ISTRIP	/* disable clear high bit of input characters */
		| INLCR		/* disable translate NL to CR */
		| IGNCR		/* disable ignore CR */
		| ICRNL		/* disable translate CR to NL */
		| IXON);	/* disable enable XON/XOFF flow control */

	/* disable postprocess output characters */
	tty->termios.c_oflag &= ~OPOST;

	tty->termios.c_lflag &=
		~(ECHO		/* disable echo input characters */
		| ECHONL	/* disable echo new line */
		| ICANON	/* disable erase, kill, werase, and rprnt
				   special characters */
		| ISIG		/* disable interrupt, quit, and suspend special
				   characters */
		| IEXTEN);	/* disable non-POSIX special characters */

	/* 2400 baud, 8bits, no parity, 1 stop */
	tty->termios.c_cflag = B2400 | CS8 | CREAD | HUPCL | CLOCAL;
	tty->ops->set_termios(tty, &old_termios);
	up_write(&tty->termios_rwsem);
	tty->ops->tiocmset(tty, 0, ~0); /* clear all */

	tty->receive_room = 65536;
	tty->port->low_latency = 1; // does not do anything since kernel 3.12

	/* flush any existing data in the buffer */
	if (tty->ldisc->ops->flush_buffer)
		tty->ldisc->ops->flush_buffer(tty);
	tty_driver_flush_buffer(tty);

	return 0;
}

static void ev3_uart_close(struct tty_struct *tty)
{
	struct ev3_uart_port_data *port = tty->disc_data;

	port->closing = true;
	if (!completion_done(&port->set_mode_completion))
		complete(&port->set_mode_completion);

	cancel_work_sync(&port->change_bitrate_work);
	hrtimer_cancel(&port->keep_alive_timer);
	tasklet_kill(&port->keep_alive_tasklet);
	if (port->sensor.context)
		unregister_lego_sensor(&port->sensor);
	if (port->in_port)
		put_device(&port->in_port->dev);
	tty->disc_data = NULL;
	kfree(port);
}

static int ev3_uart_ioctl(struct tty_struct *tty, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	return tty_mode_ioctl(tty, file, cmd, arg);
}

static void ev3_uart_write_wakeup(struct tty_struct *tty)
{
	debug_pr("%s\n", __func__);
}

static struct tty_ldisc_ops ev3_uart_ldisc = {
	.magic			= TTY_LDISC_MAGIC,
	.name			= "n_legoev3",
	.open			= ev3_uart_open,
	.close			= ev3_uart_close,
	.ioctl			= ev3_uart_ioctl,
	.receive_buf2		= ev3_uart_receive_buf2,
	.write_wakeup		= ev3_uart_write_wakeup,
	.owner			= THIS_MODULE,
};

static int __init ev3_uart_init(void)
{
	int err;

	err = tty_register_ldisc(N_LEGOEV3, &ev3_uart_ldisc);
	if (err) {
		pr_err("Could not register EV3 UART sensor line discipline. (%d)\n",
			err);
		return err;
	}

	pr_info("Registered EV3 UART sensor line discipline. (%d)\n", N_LEGOEV3);

	return 0;
}
module_init(ev3_uart_init);

static void __exit ev3_uart_exit(void)
{
	int err;

	err = tty_unregister_ldisc(N_LEGOEV3);
	if (err)
		pr_err("Could not unregister EV3 UART sensor line discipline. (%d)\n",
			err);
}
module_exit(ev3_uart_exit);

MODULE_DESCRIPTION("LEGO MINDSTORMS EV3 sensor tty line discipline");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS_LDISC(N_LEGOEV3);
