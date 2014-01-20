/*
 * tty line discipline for LEGO Mindstorms EV3 UART sensors
 *
 * Copyright (C) 2014 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/tty.h>


#define BUFFER_SIZE 50
#define NAME_SIZE 11
#define UNITS_SIZE 4
#define SENOR_DATA_SIZE 32

#define LEGOEV3_UART_MSG_TYPE_MASK 0xC0
#define LEGOEV3_UART_CMD_SIZE(byte) (1 << ((byte >> 3) & 0x7))
#define LEGOEV3_UART_MSG_CMD_MASK 0x07
#define LEGOEV3_UART_TYPE_MAX 101
#define LEGOEV3_UART_MODE_MAX 7

#define LEGOEV3_UART_TYPE_UNKNOWN 125
#define LEGOEV3_UART_SPEED_MIN 2400

enum legoev3_uart_msg_type {
	LEGOEV3_UART_MSG_TYPE_SYS	= 0x00,
	LEGOEV3_UART_MSG_TYPE_CMD	= 0x40,
	LEGOEV3_UART_MSG_TYPE_INFO	= 0x80,
	LEGOEV3_UART_MSG_TYPE_DATA	= 0xC0,
};

enum legoev3_uart_sys {
	LEGOEV3_UART_SYS_SYNC		= 0x0,
	LEGOEV3_UART_SYS_NACK		= 0x2,
	LEGOEV3_UART_SYS_ACK		= 0x4,
	LEGOEV3_UART_SYS_ESC		= 0x6,
};

enum legoev3_uart_cmd {
	LEGOEV3_UART_CMD_TYPE		= 0x0,
	LEGOEV3_UART_CMD_MODES		= 0x1,
	LEGOEV3_UART_CMD_SPEED		= 0x2,
	LEGOEV3_UART_CMD_SELECT		= 0x3,
	LEGOEV3_UART_CMD_WRITE		= 0x4,
};

enum legoev3_uart_info {
	LEGOEV3_UART_INFO_NAME		= 0x00,
	LEGOEV3_UART_INFO_RAW		= 0x01,
	LEGOEV3_UART_INFO_PCT		= 0x02,
	LEGOEV3_UART_INFO_SI		= 0x03,
	LEGOEV3_UART_INFO_UNITS		= 0x04,
	LEGOEV3_UART_INFO_FORMAT	= 0x80,
};

#define LEGOEV3_UART_INFO_BIT_CMD_TYPE		0
#define LEGOEV3_UART_INFO_BIT_CMD_MODES		1
#define LEGOEV3_UART_INFO_BIT_CMD_SPEED		2
#define LEGOEV3_UART_INFO_BIT_INFO_NAME		3
#define LEGOEV3_UART_INFO_BIT_INFO_RAW		4
#define LEGOEV3_UART_INFO_BIT_INFO_PCT		5
#define LEGOEV3_UART_INFO_BIT_INFO_SI		6
#define LEGOEV3_UART_INFO_BIT_INFO_UNITS	7
#define LEGOEV3_UART_INFO_BIT_INFO_FORMAT	8

enum legoev3_uart_info_flags {
	LEGOEV3_UART_INFO_FLAG_CMD_TYPE		= BIT(LEGOEV3_UART_INFO_BIT_CMD_TYPE),
	LEGOEV3_UART_INFO_FLAG_CMD_MODES	= BIT(LEGOEV3_UART_INFO_BIT_CMD_MODES),
	LEGOEV3_UART_INFO_FLAG_CMD_SPEED	= BIT(LEGOEV3_UART_INFO_BIT_CMD_SPEED),
	LEGOEV3_UART_INFO_FLAG_INFO_NAME	= BIT(LEGOEV3_UART_INFO_BIT_INFO_NAME),
	LEGOEV3_UART_INFO_FLAG_INFO_RAW		= BIT(LEGOEV3_UART_INFO_BIT_INFO_RAW),
	LEGOEV3_UART_INFO_FLAG_INFO_PCT		= BIT(LEGOEV3_UART_INFO_BIT_INFO_PCT),
	LEGOEV3_UART_INFO_FLAG_INFO_SI		= BIT(LEGOEV3_UART_INFO_BIT_INFO_SI),
	LEGOEV3_UART_INFO_FLAG_INFO_UNITS	= BIT(LEGOEV3_UART_INFO_BIT_INFO_UNITS),
	LEGOEV3_UART_INFO_FLAG_INFO_FORMAT	= BIT(LEGOEV3_UART_INFO_BIT_INFO_FORMAT),
	LEGOEV3_UART_INFO_FLAG_INFO_ALL		= LEGOEV3_UART_INFO_FLAG_INFO_NAME
						| LEGOEV3_UART_INFO_FLAG_INFO_RAW
						| LEGOEV3_UART_INFO_FLAG_INFO_PCT
						| LEGOEV3_UART_INFO_FLAG_INFO_SI
						| LEGOEV3_UART_INFO_FLAG_INFO_UNITS
						| LEGOEV3_UART_INFO_FLAG_INFO_FORMAT,
	LEGOEV3_UART_INFO_FLAG_REQUIRED		= LEGOEV3_UART_INFO_FLAG_CMD_TYPE
						| LEGOEV3_UART_INFO_FLAG_CMD_MODES
						| LEGOEV3_UART_INFO_FLAG_INFO_NAME
						| LEGOEV3_UART_INFO_FLAG_INFO_FORMAT,
};

struct legoev3_uart_sensor_info {
	u8 type;
	u8 mode;
	u8 num_modes;
	unsigned speed;
	char name[NAME_SIZE + 1];
	/* min/max values are actually float data type */
	unsigned raw_min;
	unsigned raw_max;
	unsigned pct_min;
	unsigned pct_max;
	unsigned si_min;
	unsigned si_max;
	char units[UNITS_SIZE + 1];
	u8 data_sets;
	u8 format;
	u8 figures;
	u8 decimals;
};

static struct legoev3_uart_sensor_info default_sensor_info = {
	.type		= LEGOEV3_UART_TYPE_UNKNOWN,
	.speed		= LEGOEV3_UART_SPEED_MIN,
	.raw_max	= 0x447fc000,	/* 1023.0 */
	.pct_max	= 0x42c80000,	/*  100.0 */
	.si_max		= 0x3f800000,	/*    1.0 */
};

struct legoev3_uart_data {
	struct tty_struct *tty;
	struct legoev3_uart_sensor_info sensor_info[LEGOEV3_UART_MODE_MAX + 1];
	u8 type;
	u8 num_modes;
	u8 num_view_modes;
	u8 mode;
	long unsigned info_flags;
	u8 buffer[BUFFER_SIZE];
	unsigned write_ptr;
	unsigned synced:1;
};

static inline int legoev3_uart_msg_size(u8 header)
{
	int size;

	if (!(header & LEGOEV3_UART_MSG_TYPE_MASK)) /* SYNC, NACK, ACK */
		return 1;

	size = LEGOEV3_UART_CMD_SIZE(header);
	size += 2; /* header and checksum */
	if ((header & LEGOEV3_UART_MSG_TYPE_MASK) == LEGOEV3_UART_MSG_TYPE_INFO)
		size++; /* extra command byte */

	return size;
}

static int legoev3_uart_open(struct tty_struct *tty)
{
	struct ktermios old_termios = *tty->termios;
	struct legoev3_uart_data *data;
printk("%s\n", __func__);

	data = kzalloc(sizeof(struct legoev3_uart_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	tty->disc_data = data;

	/* set baud rate and other port settings */
	mutex_lock(&tty->termios_mutex);
	tty->termios->c_iflag &=
		 ~(IGNBRK	/* disable ignore break */
		| BRKINT	/* disable break causes interrupt */
		| PARMRK	/* disable mark parity errors */
		| ISTRIP	/* disable clear high bit of input characters */
		| INLCR		/* disable translate NL to CR */
		| IGNCR		/* disable ignore CR */
		| ICRNL		/* disable translate CR to NL */
		| IXON);	/* disable enable XON/XOFF flow control */

	/* disable postprocess output characters */
	tty->termios->c_oflag &= ~OPOST;

	tty->termios->c_lflag &=
		 ~(ECHO		/* disable echo input characters */
		| ECHONL	/* disable echo new line */
		| ICANON	/* disable erase, kill, werase, and rprnt
				   special characters */
		| ISIG		/* disable interrupt, quit, and suspend special
				   characters */
		| IEXTEN);	/* disable non-POSIX special characters */

	/* 2400 baud, 8bits, no parity, 1 stop */
	tty->termios->c_cflag = B2400 | CS8 | CREAD | HUPCL | CLOCAL;
	tty->ops->set_termios(tty, &old_termios);
	tty->ops->tiocmset(tty, 0, ~0); /* clear all */
	mutex_unlock(&tty->termios_mutex);

	tty->receive_room = 65536;

	/* flush any existing data in the buffer */
	if (tty->ldisc->ops->flush_buffer)
		tty->ldisc->ops->flush_buffer(tty);
	tty_driver_flush_buffer(tty);

	return 0;
}

static void legoev3_uart_close(struct tty_struct *tty)
{
printk("%s\n", __func__);
}

static int legoev3_uart_ioctl(struct tty_struct *tty, struct file *file,
			      unsigned int cmd, unsigned long arg)
{printk("%s\n", __func__);
	return tty_mode_ioctl(tty, file, cmd, arg);
}

static void legoev3_uart_receive_buf(struct tty_struct *tty,
				     const unsigned char *cp,
				     char *fp, int count)
{
	struct legoev3_uart_data *data = tty->disc_data;
	int i = 0;
	int j, speed;
	u8 cmd, cmd2, type, mode, msg_type, msg_size, chksum;

printk("received: ");
for (i = 0; i < count; i++)
	printk("0x%02x ", cp[i]);
printk(" (%d)\n", count);
i=0;

	/*
	 * to get in sync with the data stream from the sensor, we look
	 * for a valid TYPE command.
	 */
	while (!data->synced) {
		if (i + 2 >= count)
			return;
		cmd = cp[i++];
		if (cmd != (LEGOEV3_UART_MSG_TYPE_CMD | LEGOEV3_UART_CMD_TYPE))
			continue;
		type = cp[i];
		if (!type || type > LEGOEV3_UART_TYPE_MAX)
			continue;
		chksum = 0xFF ^ cmd ^ type;
		if (cp[i+1] != chksum)
			continue;
		for (j = 0; j <= LEGOEV3_UART_MODE_MAX; j++)
			data->sensor_info[j] = default_sensor_info;
		data->type = type;
		data->num_modes = 0;
		data->num_view_modes = 0;
		data->info_flags = LEGOEV3_UART_INFO_FLAG_CMD_TYPE;
		data->synced = 1;
		data->write_ptr = 0;
		i += 2;
	}
	if (!data->synced)
		return;

	/*
	 * Once we are synced, we keep reading data until we have read
	 * a complete command.
	 */
	while (i < count) {
		if (data->write_ptr >= BUFFER_SIZE)
			goto err_bad_data;
		data->buffer[data->write_ptr++] = cp[i++];
	}

	/*
	 * Process all complete messages that have been received.
	 */
	while ((msg_size = legoev3_uart_msg_size(data->buffer[0]))
	        <= data->write_ptr)
	{
printk("processing: ");
for (i = 0; i < data->write_ptr; i++)
	printk("0x%02x ", data->buffer[i]);
printk(" (%d)\n", data->write_ptr);
		printk("msg_size:%d\n", msg_size);
		msg_type = data->buffer[0] & LEGOEV3_UART_MSG_TYPE_MASK;
		cmd = data->buffer[0] & LEGOEV3_UART_MSG_CMD_MASK;
		mode = cmd;
		cmd2 = data->buffer[1];
		if (msg_size > 1) {
			chksum = 0xFF;
			for (i = 0; i < msg_size - 1; i++)
				chksum ^= data->buffer[i];
			printk("chksum:%d, actual:%d\n", chksum, data->buffer[msg_size - 1]);
			if (chksum != data->buffer[msg_size - 1])
				goto err_bad_data;
		}
		switch (msg_type) {
		case LEGOEV3_UART_MSG_TYPE_SYS:
			printk("SYS:%d\n", data->buffer[0] & LEGOEV3_UART_MSG_CMD_MASK);
			break;
		case LEGOEV3_UART_MSG_TYPE_CMD:
			printk("CMD:%d\n", cmd);
			/* TODO: handle ACK */
			switch (cmd) {
			case LEGOEV3_UART_CMD_MODES:
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_CMD_MODES, &data->info_flags))
					goto err_bad_data;
				if (!cmd2 || cmd2 > LEGOEV3_UART_MODE_MAX)
					goto err_bad_data;
				data->num_modes = cmd2 + 1;
				if (msg_size > 3)
					data->num_view_modes = data->buffer[2] + 1;
				else
					data->num_view_modes = data->num_modes;
				printk("num_modes:%d, num_view_modes:%d\n", data->num_modes, data->num_view_modes);
				break;
			case LEGOEV3_UART_CMD_SPEED:
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_CMD_SPEED, &data->info_flags))
					goto err_bad_data;
				speed = *(int*)(data->buffer + 1);
				/* TODO: change uart bit rate */
				printk("speed:%d\n", speed);
				break;
			default:
				goto err_bad_data;
			}
			break;
		case LEGOEV3_UART_MSG_TYPE_INFO:
			printk("INFO:%d, mode:%d\n", cmd2, mode);
			switch (cmd2) {
			case LEGOEV3_UART_INFO_NAME:
				data->info_flags &= ~LEGOEV3_UART_INFO_FLAG_INFO_ALL;
				if (data->buffer[2] < 'A' || data->buffer[2] > 'z')
					goto err_bad_data;
				if (strlen(data->buffer + 2) > NAME_SIZE)
					goto err_bad_data;
				snprintf(data->sensor_info[mode].name,
				         NAME_SIZE + 1, "%s", data->buffer + 2);
				data->mode = mode;
				data->info_flags |= LEGOEV3_UART_INFO_FLAG_INFO_NAME;
				printk("mode %d name:%s\n", mode, data->sensor_info[mode].name);
				break;
			case LEGOEV3_UART_INFO_RAW:
				if (data->mode != mode)
					goto err_bad_data;
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_RAW, &data->info_flags))
					goto err_bad_data;
				data->sensor_info[mode].raw_min = *(unsigned *)(data->buffer + 2);
				data->sensor_info[mode].raw_max = *(unsigned *)(data->buffer + 6);
				printk("mode %d raw_min:%08x, raw_max:%08x\n", mode, data->sensor_info[mode].raw_min, data->sensor_info[mode].raw_max);
				break;
			case LEGOEV3_UART_INFO_PCT:
				if (data->mode != mode)
					goto err_bad_data;
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_PCT, &data->info_flags))
					goto err_bad_data;
				data->sensor_info[mode].pct_min = *(unsigned *)(data->buffer + 2);
				data->sensor_info[mode].pct_max = *(unsigned *)(data->buffer + 6);
				printk("mode %d pct_min:%08x, pct_max:%08x\n", mode, data->sensor_info[mode].pct_min, data->sensor_info[mode].pct_max);
				break;
			case LEGOEV3_UART_INFO_SI:
				if (data->mode != mode)
					goto err_bad_data;
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_SI, &data->info_flags))
					goto err_bad_data;
				data->sensor_info[mode].si_min = *(unsigned *)(data->buffer + 2);
				data->sensor_info[mode].si_max = *(unsigned *)(data->buffer + 6);
				printk("mode %d si_min:%08x, si_max:%08x\n", mode, data->sensor_info[mode].si_min, data->sensor_info[mode].si_max);
				break;
			case LEGOEV3_UART_INFO_UNITS:
				if (data->mode != mode)
					goto err_bad_data;
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_UNITS, &data->info_flags))
					goto err_bad_data;
				snprintf(data->sensor_info[mode].units,
					 UNITS_SIZE + 1, "%s", data->buffer + 2);
				printk("mode %d units:%s\n", mode, data->sensor_info[mode].units);
				break;
			case LEGOEV3_UART_INFO_FORMAT:
				if (data->mode != mode)
					goto err_bad_data;
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_FORMAT, &data->info_flags))
					goto err_bad_data;
				data->sensor_info[mode].data_sets = data->buffer[2];
				if (!data->sensor_info[mode].data_sets)
					goto err_bad_data;
				if (msg_size < 7)
					goto err_bad_data;
				if ((data->info_flags & LEGOEV3_UART_INFO_FLAG_REQUIRED)
						!= LEGOEV3_UART_INFO_FLAG_REQUIRED)
					goto err_bad_data;
				data->sensor_info[mode].format = data->buffer[3];
				if (data->mode) {
					data->mode--;
					data->sensor_info[mode].type = data->type;
					data->sensor_info[mode].num_modes = data->num_modes;
					data->sensor_info[mode].figures = data->buffer[4];
					data->sensor_info[mode].decimals = data->buffer[5];
					/* TODO: copy IR Seeker hack from lms2012 */
				}
				printk("mode %d data_sets:%d, format:%d, figures:%d, decimals:%d\n",
				       mode, data->sensor_info[mode].data_sets, data->sensor_info[mode].format,
				       data->sensor_info[mode].figures, data->sensor_info[mode].decimals);
				break;
			}
			break;
		case LEGOEV3_UART_MSG_TYPE_DATA:
			printk("DATA:%d\n", data->buffer[0] & LEGOEV3_UART_MSG_CMD_MASK);
			break;
		}

		/*
		 * If there is leftover data, we move it to the beginning
		 * of the buffer.
		 */
		for (i = 0; i + msg_size < data->write_ptr; i++)
			data->buffer[i] = data->buffer[i + msg_size];
		data->write_ptr = i;
	}
	return;

err_bad_data:
	data->synced = 0;
}

static void legoev3_uart_write_wakeup(struct tty_struct *tty)
{
printk("%s\n", __func__);
}

static struct tty_ldisc_ops legoev3_uart_ldisc = {
	.magic			= TTY_LDISC_MAGIC,
	.name			= "n_legoev3",
	.open			= legoev3_uart_open,
	.close			= legoev3_uart_close,
	.ioctl			= legoev3_uart_ioctl,
	.receive_buf		= legoev3_uart_receive_buf,
	.write_wakeup		= legoev3_uart_write_wakeup,
	.owner			= THIS_MODULE,
};

static int __init legoev3_uart_init(void)
{
	int err;

	err = tty_register_ldisc(N_LEGOEV3, &legoev3_uart_ldisc);
	if (err) {
		pr_err("Could not register LEGOEV3 line discipline. (%d)\n",
			err);
		return err;
	}

	pr_info("Registered LEGOEV3 line discipline. (%d)\n", N_LEGOEV3);

	return 0;
}
module_init(legoev3_uart_init);

static void __exit legoev3_uart_exit(void)
{
	int err;

	err = tty_unregister_ldisc(N_LEGOEV3);
	if (err)
		pr_err("Could not unregister LEGOEV3 line discipline. (%d)\n",
			err);
}
module_exit(legoev3_uart_exit);

MODULE_DESCRIPTION("tty line discipline for LEGO Mindstorms EV3 sensors");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS_LDISC(N_LEGOEV3);
