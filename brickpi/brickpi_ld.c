/*
 * Dexter Industries BrickPi driver
 *
 * Copyright (C) 2015 David Lechner <david@lechnology.com>
 *
 * Based on BrickPi.h by:
 *
 * Matthew Richardson <matthewrichardson37(at)gmail.com>
 * Jaikrishna T S <t.s.jaikrishna(at)gmail.com>
 * John Cole, Dexter Industries.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * Note: The comment block below is used to generate docs on the ev3dev website.
 * Use kramdown (markdown) syntax. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * Dexter Industries BrickPi Line Discipline
 *
 * This driver is a tty [line discipline] that runs on top of a tty. It provides
 * [lego-port] class instances for the datas themselves (excluding input data 5).
 * and [lego-sensor] and [tacho-motor] instances for the devices connected to
 * the datas.
 * .
 * [line discipline]: https://en.wikipedia.org/wiki/Line_discipline
 * [lego-data]: ../lego-port-class
 * [lego-sensor]: ../lego-sensor-class
 * [tacho-motor]: ../tacho-motor-class
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/tty.h>

#include "brickpi_internal.h"

//#define DEBUG
#ifdef DEBUG
#define debug_pr(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define debug_pr(fmt, ...) while(0) { }
#endif

#ifndef N_BRICKPI
#define N_BRICKPI 28
#endif

/* TODO: Make this a module parameter */
#define BRICKPI_POLL_MS		500
#define BRICKPI_POLL_PERIOD	msecs_to_jiffies(BRICKPI_POLL_MS)

/* tx_buffer offsets */
#define BRICKPI_TX_ADDR		0
#define BRICKPI_TX_CHECKSUM	1
#define BRICKPI_TX_SIZE		2
#define BRICKPI_TX_MESSAGE_TYPE	3
#define BRICKPI_TX_MESSAGE_DATA	4

#define BRICKPI_TX_BUFFER_TAIL_INIT (BRICKPI_TX_MESSAGE_DATA * 8)

/* rx_buffer offsets */
#define BRICKPI_RX_CHECKSUM	0
#define BRICKPI_RX_SIZE		1
#define BRICKPI_RX_MESSAGE_TYPE	2
#define BRICKPI_RX_MESSAGE_DATA	3

#define BRICKPI_RX_BUFFER_HEAD_INIT (BRICKPI_RX_MESSAGE_DATA * 8)

void brickpi_append_tx(struct brickpi_data *data, u8 size, long value)
{
	unsigned end = data->tx_buffer_tail + size;

	while (data->tx_buffer_tail < end) {
		if (value & 0x01)
			set_bit(data->tx_buffer_tail, (unsigned long *)data->tx_buffer);
		else
			clear_bit(data->tx_buffer_tail, (unsigned long *)data->tx_buffer);
		value >>= 1;
		data->tx_buffer_tail++;
	}
}

long brickpi_read_rx(struct brickpi_data *data, u8 size)
{
	long result = 0;
	int i = 0;

	while (i < size) {
		if (test_bit(data->rx_buffer_head, (unsigned long *)data->rx_buffer))
			result |= 1 << i;
		data->rx_buffer_head++;
		i++;
	}

	return result;
}

int brickpi_send_message(struct brickpi_data *data, u8 addr,
			 enum brickpi_message msg, unsigned timeout)
{
	unsigned size, i, ret;
	unsigned retries = 2;
	u8 checksum = 0;

	WARN_ON(!mutex_is_locked(&data->tx_mutex));
	size = (data->tx_buffer_tail + 7) / 8;
	data->tx_buffer[BRICKPI_TX_ADDR] = addr;
	data->tx_buffer[BRICKPI_TX_CHECKSUM] = 0;
	data->tx_buffer[BRICKPI_TX_SIZE] = size - 3;
	data->tx_buffer[BRICKPI_TX_MESSAGE_TYPE] = msg;
	for (i = 0; i < size; i++)
		checksum += data->tx_buffer[i];
	data->tx_buffer[BRICKPI_TX_CHECKSUM] = checksum;
	while (retries--) {
		data->rx_data_size = 0;
		reinit_completion(&data->rx_completion);
		set_bit(TTY_DO_WRITE_WAKEUP, &data->tty->flags);
		ret = data->tty->ops->write(data->tty, data->tx_buffer, size);
		if (ret < 0)
			return ret;
		ret = wait_for_completion_timeout(&data->rx_completion,
						  msecs_to_jiffies(timeout));
		if (ret)
			break;
	}
	if (!ret)
		return -ETIMEDOUT;

	if (data->rx_buffer[BRICKPI_RX_MESSAGE_TYPE] != data->tx_buffer[BRICKPI_TX_MESSAGE_TYPE])
		return -EPROTO;

	return 0;
}

int brickpi_set_sensors(struct brickpi_channel_data *ch_data)
{
	struct brickpi_data *data = ch_data->data;
	int err;

	if (data->closing)
		return 0;
	mutex_lock(&data->tx_mutex);
	data->tx_buffer_tail = BRICKPI_TX_BUFFER_TAIL_INIT;
	brickpi_append_tx(data, 8, ch_data->in_port[BRICKPI_PORT_1].sensor_type);
	brickpi_append_tx(data, 8, ch_data->in_port[BRICKPI_PORT_2].sensor_type);
	/* TODO: Handle I2C sensor stuff */
	err = brickpi_send_message(data, ch_data->address,
				   BRICK_PI_MESSAGE_SET_SENSOR, 5000);
	mutex_unlock(&data->tx_mutex);

	return err;
}

int brickpi_get_values(struct brickpi_channel_data *ch_data)
{
	struct brickpi_data *data = ch_data->data;
	int i, j, err;
	u8 port_size[NUM_BRICKPI_PORT];

	if (data->closing)
		return 0;
	mutex_lock(&data->tx_mutex);
	data->tx_buffer_tail = BRICKPI_TX_BUFFER_TAIL_INIT;
	for (i = 0; i < NUM_BRICKPI_PORT; i++) {
		brickpi_append_tx(data, 1, ch_data->out_port[i].motor_use_offset);
		/* TODO: handle use_offset */
	}
	for (i = 0; i < NUM_BRICKPI_PORT; i++) {
		brickpi_append_tx(data, 8, ch_data->out_port[i].motor_speed);
		brickpi_append_tx(data, 1, ch_data->out_port[i].motor_direction);
		brickpi_append_tx(data, 1, ch_data->out_port[i].motor_enable);
	}
	/* TODO: Handle I2C sensor stuff */
	err = brickpi_send_message(data, ch_data->address,
				   BRICK_PI_MESSAGE_GET_VALUES, 100);
	if (err < 0) {
		/* TODO: Set encoder offsets to 0 */
		mutex_unlock(&data->tx_mutex);
		return err;
	}

	data->rx_buffer_head = BRICKPI_RX_BUFFER_HEAD_INIT;
	port_size[BRICKPI_PORT_1] = brickpi_read_rx(data, 5);
	debug_pr("port_size[BRICKPI_PORT_1]: %u\n", port_size[BRICKPI_PORT_1]);
	port_size[BRICKPI_PORT_2] = brickpi_read_rx(data, 5);
	debug_pr("port_size[BRICKPI_PORT_2]: %u\n", port_size[BRICKPI_PORT_2]);
	for (i = 0; i < NUM_BRICKPI_PORT; i++) {
		u64 bits = brickpi_read_rx(data, port_size[i]);
		s64 position = bits >> 1;
		if (bits & 1)
			position *= -1;
		ch_data->out_port[i].motor_position = position;
		debug_pr("motor_position[%d]: %d\n", i, (int)position);
	}
	for (i = 0; i < NUM_BRICKPI_PORT; i++) {
		long *sensor_values = ch_data->in_port[i].sensor_values;
		u8 *raw_data = ch_data->in_port[i].port.raw_data;

		switch (ch_data->in_port[i].sensor_type) {
		case BRICKPI_SENSOR_TYPE_NXT_TOUCH:
			sensor_values[0] = brickpi_read_rx(data, 1);
			break;
		case BRICKPI_SENSOR_TYPE_NXT_ULTRASONIC_CONT:
		case BRICKPI_SENSOR_TYPE_NXT_ULTRASONIC_SS:
			sensor_values[0] = brickpi_read_rx(data, 8);
			break;
		case BRICKPI_SENSOR_TYPE_NXT_COLOR_FULL:
			sensor_values[0] = brickpi_read_rx(data, 3);
			sensor_values[1] = brickpi_read_rx(data, 10);
			sensor_values[2] = brickpi_read_rx(data, 10);
			sensor_values[3] = brickpi_read_rx(data, 10);
			sensor_values[4] = brickpi_read_rx(data, 10);
			break;
		case BRICKPI_SENSOR_TYPE_NXT_I2C:
		case BRICKPI_SENSOR_TYPE_NXT_I2C_9V:
			sensor_values[0] = brickpi_read_rx(data,
					ch_data->in_port[i].i2c_msg_count);
			for (j = 0; j < ch_data->in_port[i].i2c_msg_count; j++) {
				if (sensor_values[0] & (1 << j)) {
					int k;
					struct brickpi_i2c_msg_data *msg =
						&ch_data->in_port[i].i2c_msg[j];
					for (k = 0; k < msg->read_size; k++) {
						msg->read_data[k] =
							brickpi_read_rx(data, 8);
					}
				}
			}
			break;
		case BRICKPI_SENSOR_TYPE_EV3_US_M0:
		case BRICKPI_SENSOR_TYPE_EV3_US_M1:
		case BRICKPI_SENSOR_TYPE_EV3_US_M2:
		case BRICKPI_SENSOR_TYPE_EV3_US_M3:
		case BRICKPI_SENSOR_TYPE_EV3_US_M4:
		case BRICKPI_SENSOR_TYPE_EV3_US_M5:
		case BRICKPI_SENSOR_TYPE_EV3_US_M6:
		case BRICKPI_SENSOR_TYPE_EV3_COLOR_M0:
		case BRICKPI_SENSOR_TYPE_EV3_COLOR_M1:
		case BRICKPI_SENSOR_TYPE_EV3_COLOR_M2:
		case BRICKPI_SENSOR_TYPE_EV3_COLOR_M4:
		case BRICKPI_SENSOR_TYPE_EV3_COLOR_M5:
		case BRICKPI_SENSOR_TYPE_EV3_GYRO_M0:
		case BRICKPI_SENSOR_TYPE_EV3_GYRO_M1:
		case BRICKPI_SENSOR_TYPE_EV3_GYRO_M2:
		case BRICKPI_SENSOR_TYPE_EV3_GYRO_M4:
		case BRICKPI_SENSOR_TYPE_EV3_INFRARED_M0:
		case BRICKPI_SENSOR_TYPE_EV3_INFRARED_M1:
		case BRICKPI_SENSOR_TYPE_EV3_INFRARED_M3:
		case BRICKPI_SENSOR_TYPE_EV3_INFRARED_M4:
		case BRICKPI_SENSOR_TYPE_EV3_INFRARED_M5:
		case BRICKPI_SENSOR_TYPE_EV3_TOUCH:
			sensor_values[0] = brickpi_read_rx(data, 16);
			break;
		case BRICKPI_SENSOR_TYPE_EV3_COLOR_M3:
		case BRICKPI_SENSOR_TYPE_EV3_GYRO_M3:
		case BRICKPI_SENSOR_TYPE_EV3_INFRARED_M2:
			sensor_values[0] = brickpi_read_rx(data, 32);
			break;
		default:
			sensor_values[0] = brickpi_read_rx(data, 10);
			/* NXT Analog expects value in mV */
			if (ch_data->in_port[i].sensor_type <= BRICKPI_SENSOR_TYPE_NXT_ANALOG_MAX)
				sensor_values[0] = sensor_values[0] * 5000 / 1024;
			break;
		}
		debug_pr("ch_data->sensor_values[%d][0]: %ld\n", i,
			 sensor_values[0]);

		if (!raw_data)
			continue;
		memcpy(raw_data, sensor_values,
		       sizeof(long) * NUM_BRICKPI_SENSOR_VALUES);
		lego_port_call_raw_data_func(&ch_data->in_port[i].port);
	}
	mutex_unlock(&data->tx_mutex);

	return 0;
}

static void brickpi_handle_rx_data(struct work_struct *work)
{
	struct brickpi_data *data =
		container_of(work, struct brickpi_data, rx_data_work);
	int i;
	u8 size;
	u8 checksum = 0;

#ifdef DEBUG
	printk("received: ");
	for (i = 0; i < data->rx_data_size; i++)
		printk("0x%02x ", data->rx_buffer[i]);
	printk("(%d)\n", data->rx_data_size);
#endif

	/* Check if there is enough data to get the size */
	if (data->rx_data_size < BRICKPI_RX_SIZE)
		return;
	size = data->rx_buffer[BRICKPI_RX_SIZE] + 2;
	/* if the size is bigger than the buffer, throw out the data */
	if (size > BRICKPI_BUFFER_SIZE) {
		dev_err(data->tty->dev, "Buffer overrun.\n");
		data->rx_data_size = 0;
		return;
	}
	/* Check to see if we have received all of the data */
	if (data->rx_data_size < size)
		return;
	for (i = BRICKPI_RX_SIZE; i < size; i++)
		checksum += data->rx_buffer[i];

	/* throw out the data if the checksum is bad */
	if (checksum != data->rx_buffer[BRICKPI_RX_CHECKSUM]) {
		dev_err(data->tty->dev, "Bad checksum.\n");
		data->rx_data_size = 0;
		return;
	}

	complete(&data->rx_completion);
}

static void brickpi_poll_work(struct work_struct *work)
{
	struct brickpi_data *data = container_of(to_delayed_work(work),
						 struct brickpi_data, poll_work);
	int i, err;
	unsigned long time_since_last_poll;

	if (data->closing)
		return;

	for (i = 0; i < data->num_channels; i++) {
		struct brickpi_channel_data *ch_data = &data->channel_data[i];
		if (ch_data->init_ok) {
			err = brickpi_get_values(ch_data);
			if (err < 0)
				debug_pr("failed to get values for address %d. (%d)\n",
				ch_data->address, err);
		}
	}

	time_since_last_poll = jiffies - data->last_poll_timestamp;
	data->last_poll_timestamp = jiffies;
	schedule_delayed_work(&data->poll_work,
		time_since_last_poll >= BRICKPI_POLL_PERIOD
		? 0 : BRICKPI_POLL_PERIOD - time_since_last_poll);
}

static void brickpi_init_work(struct work_struct *work)
{
	struct brickpi_data *data = container_of(to_delayed_work(work),
						 struct brickpi_data, poll_work);
	int i, err;

	for (i = 0; i < data->num_channels; i++) {
		struct brickpi_channel_data *ch_data = &data->channel_data[i];

		ch_data->data = data;
		// TODO: add module parameter to get addresses
		ch_data->address = i + 1;
		ch_data->in_port[BRICKPI_PORT_1].ch_data = ch_data;
		ch_data->in_port[BRICKPI_PORT_1].sensor_type =
						BRICKPI_SENSOR_TYPE_FW_VERSION;
		ch_data->in_port[BRICKPI_PORT_2].ch_data = ch_data;
		ch_data->in_port[BRICKPI_PORT_2].sensor_type =
						BRICKPI_SENSOR_TYPE_FW_VERSION;
		err = brickpi_set_sensors(ch_data);
		if (err < 0) {
			dev_err(data->tty->dev,
				"Failed to init while setting sensors. (%d)\n",
				err);
			return;
		}
		err = brickpi_get_values(ch_data);
		if (err < 0) {
			dev_err(data->tty->dev,
				"Failed to init while getting values. (%d)\n",
				err);
			return;
		}
		ch_data->fw_version =
			ch_data->in_port[BRICKPI_PORT_1].sensor_values[0];
		debug_pr("address: %d, fw: %d\n", ch_data->address,
			 ch_data->fw_version);

		err = brickpi_register_in_ports(ch_data, data->tty->dev);
		if (err < 0) {
			dev_err(data->tty->dev,
				"Failed to register input ports (%d)",
				err);
			return;
		}
		ch_data->init_ok = true;
	}

	INIT_DELAYED_WORK(&data->poll_work, brickpi_poll_work);
	data->last_poll_timestamp = jiffies;
	schedule_delayed_work(&data->poll_work, 0);
}

static int brickpi_open(struct tty_struct *tty)
{
	struct ktermios old_termios = tty->termios;
	struct brickpi_data *data;
	int err;

	data = kzalloc(sizeof(struct brickpi_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->num_channels = 1;
	data->channel_data = kzalloc(sizeof(struct brickpi_channel_data)
		* data->num_channels, GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto err_channel_data_alloc;
	}

	data->tty = tty;
	mutex_init(&data->tx_mutex);
	init_completion(&data->rx_completion);
	INIT_WORK(&data->rx_data_work, brickpi_handle_rx_data);
	INIT_DELAYED_WORK(&data->poll_work, brickpi_init_work);
	tty->disc_data = data;

	/* set baud rate and other data settings */
	down_write(&tty->termios_rwsem);
	tty->termios.c_iflag &=
		~(IGNBRK    /* disable ignore break */
		| BRKINT    /* disable break causes interrupt */
		| PARMRK    /* disable mark parity errors */
		| ISTRIP    /* disable clear high bit of input characters */
		| INLCR     /* disable translate NL to CR */
		| IGNCR     /* disable ignore CR */
		| ICRNL     /* disable translate CR to NL */
		| IXON);    /* disable enable XON/XOFF flow control */

	/* disable postprocess output characters */
	tty->termios.c_oflag &= ~OPOST;

	tty->termios.c_lflag &=
		~(ECHO      /* disable echo input characters */
		| ECHONL    /* disable echo new line */
		| ICANON    /* disable erase, kill, werase, and rprnt
				   special characters */
		| ISIG      /* disable interrupt, quit, and suspend special
				   characters */
		| IEXTEN);  /* disable non-POSIX special characters */

	/* 500,000 baud, 8bits, no parity, 1 stop */
	tty->termios.c_cflag = B500000 | CS8 | CREAD | HUPCL | CLOCAL;
	tty->ops->set_termios(tty, &old_termios);
	up_write(&tty->termios_rwsem);
	tty->ops->tiocmset(tty, 0, ~0); /* clear all */

	tty->receive_room = 65536;
	tty->port->low_latency = 1; // does not do anything since kernel 3.12

	/* flush any existing data in the buffer */
	if (data->tty->ldisc->ops->flush_buffer)
		data->tty->ldisc->ops->flush_buffer(data->tty);
	tty_driver_flush_buffer(data->tty);

	schedule_delayed_work(&data->poll_work, 0);

	return 0;

err_channel_data_alloc:
	kfree(data);

	return err;
}

static void brickpi_close(struct tty_struct *tty)
{
	struct brickpi_data *data = tty->disc_data;
	int i;

	mutex_lock(&data->tx_mutex);
	data->closing = true;
	mutex_unlock(&data->tx_mutex);
	cancel_delayed_work_sync(&data->poll_work);
	cancel_work_sync(&data->rx_data_work);
	for (i = 0; i < data->num_channels; i++) {
		struct brickpi_channel_data *ch_data = &data->channel_data[i];
		if (ch_data->init_ok) {
			brickpi_unregister_in_ports(ch_data);
		}
	}
	tty->disc_data = NULL;
	kfree(data->channel_data);
	kfree(data);
}

static int brickpi_ioctl(struct tty_struct *tty, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	return tty_mode_ioctl(tty, file, cmd, arg);
}

static void brickpi_receive_buf(struct tty_struct *tty,
				const unsigned char *cp, char *fp, int count)
{
	struct brickpi_data *data = tty->disc_data;

	if (data->closing)
		return;

	/* check if there is enough room in the rx_buffer */
	if (data->rx_data_size + count > BRICKPI_BUFFER_SIZE)
		return;

	memcpy(data->rx_buffer + data->rx_data_size, cp, count);
	data->rx_data_size += count;

	schedule_work(&data->rx_data_work);
}

static struct tty_ldisc_ops brickpi_ldisc = {
	.magic		= TTY_LDISC_MAGIC,
	.name		= "n_brickpi",
	.open		= brickpi_open,
	.close		= brickpi_close,
	.ioctl		= brickpi_ioctl,
	.receive_buf	= brickpi_receive_buf,
	.owner		= THIS_MODULE,
};

static int __init brickpi_init(void)
{
	int err;

	err = tty_register_ldisc(N_BRICKPI, &brickpi_ldisc);
	if (err) {
		pr_err("Could not register BrickPi line discipline. (%d)\n",
			err);
		return err;
	}

	pr_info("Registered BrickPi line discipline. (%d)\n", N_BRICKPI);

	return 0;
}
module_init(brickpi_init);

static void __exit brickpi_exit(void)
{
	int err;

	err = tty_unregister_ldisc(N_BRICKPI);
	if (err)
		pr_err("Could not unregister BrickPi line discipline. (%d)\n",
			err);
}
module_exit(brickpi_exit);

MODULE_DESCRIPTION("Dexter Industries BrickPi tty line discipline");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS_LDISC(N_BRICKPI);
