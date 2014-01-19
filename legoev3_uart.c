/*
 * tty line dicipline for LEGO Mindstorms EV3 UART sensors
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

#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/tty.h>

#define BUFFER_SIZE 36

#define LEGOEV3_UART_MSG_TYPE_MASK 0xC0
#define LEGOEV3_UART_CMD_SIZE(byte) (1 << ((byte >> 3) & 0x7))
#define LEGOEV3_UART_MSG_CMD_MASK 0x07
#define LEGOEV3_UART_TYPE_MAX 101

enum legoev3_uart_msg_type {
	LEGOEV3_UART_MSG_TYPE_SYS	= 0x00,
	LEGOEV3_UART_MSG_TYPE_CMD	= 0x40,
	LEGOEV3_UART_MSG_TYPE_INFO	= 0x80,
	LEGOEV3_UART_MSG_TYPE_DATA	= 0xC0,
};

enum legoev3_uart_sys {
	LEGOEV3_UART_SYS_SYNC	= 0x0,
	LEGOEV3_UART_SYS_NACK	= 0x2,
	LEGOEV3_UART_SYS_ACK	= 0x4,
	LEGOEV3_UART_SYS_ESC	= 0x6,
};

enum legoev3_uart_cmd {
	LEGOEV3_UART_CMD_TYPE	= 0x0,
	LEGOEV3_UART_CMD_MODES	= 0x1,
	LEGOEV3_UART_CMD_SPEED	= 0x2,
	LEGOEV3_UART_CMD_SELECT	= 0x3,
	LEGOEV3_UART_CMD_WRITE	= 0x4,
};


struct legoev3_uart_data {
	struct tty_struct *tty;
	u8 buffer[BUFFER_SIZE];
	unsigned write_ptr;
	u8 type;
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

static void legoev3_uart_flush_buffer(struct tty_struct *tty)
{
printk("%s\n", __func__);
}

static ssize_t legoev3_uart_chars_in_buffer(struct tty_struct *tty)
{
printk("%s\n", __func__);
	return 0;
}

static ssize_t legoev3_uart_read(struct tty_struct *tty, struct file *file,
				 unsigned char __user *buf, size_t nr)
{printk("%s\n", __func__);
	return 0;
}

static ssize_t legoev3_uart_write(struct tty_struct *tty, struct file *file,
				  const unsigned char *data, size_t count)
{printk("%s\n", __func__);
	return count;
}

static int legoev3_uart_ioctl(struct tty_struct *tty, struct file *file,
			      unsigned int cmd, unsigned long arg)
{printk("%s\n", __func__);
	return tty_mode_ioctl(tty, file, cmd, arg);
}

static unsigned int legoev3_uart_poll(struct tty_struct *tty,
				      struct file *filp, poll_table *wait)
{printk("%s\n", __func__);
	return 0;
}

static void legoev3_uart_receive_buf(struct tty_struct *tty,
				     const unsigned char *cp,
				     char *fp, int count)
{
	struct legoev3_uart_data *data = tty->disc_data;
	int i = 0;
	u8 cmd, cmd2, type, msg_size, chksum;

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
		cmd2 = cp[i];
		if (!cmd2 || cmd2 > LEGOEV3_UART_TYPE_MAX)
			continue;
		chksum = 0xFF ^ cmd ^ cmd2;
		if (cp[i+1] != chksum)
			continue;
		data->type = cmd2;
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
		if (data->write_ptr >= BUFFER_SIZE) {
			data->synced = 0;
			return;printk("read too much\n");
		}
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
		type = data->buffer[0] & LEGOEV3_UART_MSG_TYPE_MASK;
		cmd2 = data->buffer[1];
		if (msg_size > 1) {
			chksum = 0xFF;
			for (i = 0; i < msg_size - 1; i++)
				chksum ^= data->buffer[i];
			printk("chksum:%d, actual:%d\n", chksum, data->buffer[msg_size - 1]);
			if (chksum != data->buffer[msg_size - 1]) {
				data->synced = 0;
				return;
			}
		}
		switch (type) {
		case LEGOEV3_UART_MSG_TYPE_SYS:
			printk("SYS:%d\n", data->buffer[0] & LEGOEV3_UART_MSG_CMD_MASK);
			break;
		case LEGOEV3_UART_MSG_TYPE_CMD:
			printk("CMD:%d\n", data->buffer[0] & LEGOEV3_UART_MSG_CMD_MASK);
			break;
		case LEGOEV3_UART_MSG_TYPE_INFO:
			printk("INFO:%d\n", data->buffer[0] & LEGOEV3_UART_MSG_CMD_MASK);
			break;
		case LEGOEV3_UART_MSG_TYPE_DATA:
			printk("DATA:%d\n", data->buffer[0] & LEGOEV3_UART_MSG_CMD_MASK);
			break;
		}

		/*
		 * If there is leftover data, we move it to the begining
		 * of the buffer.
		 */
		for (i = 0; i + msg_size < data->write_ptr; i++)
			data->buffer[i] = data->buffer[i + msg_size];
		data->write_ptr = i;
	}
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
//	.flush_buffer		= legoev3_uart_flush_buffer,
//	.chars_in_buffer	= legoev3_uart_chars_in_buffer,
//	.read			= legoev3_uart_read,
//	.write			= legoev3_uart_write,
	.ioctl			= legoev3_uart_ioctl,
//	.compat_ioctl		= NULL,
//	.set_termios		= NULL,
//	.poll			= legoev3_uart_poll,
	.receive_buf		= legoev3_uart_receive_buf,
	.write_wakeup		= legoev3_uart_write_wakeup,
//	.hangup			= NULL,
//	.dcd_change		= NULL,
	.owner			= THIS_MODULE,
};

static int __init legoev3_uart_init(void)
{
	int err;

	err = tty_register_ldisc(N_LEGOEV3, &legoev3_uart_ldisc);
	if (err) {
		pr_err("Could not register LEGOEV3 line dicipline. (%d)\n",
			err);
		return err;
	}

	pr_info("Registered LEGOEV3 line dicipline. (%d)\n", N_LEGOEV3);

	return 0;
}
module_init(legoev3_uart_init);

static void __exit legoev3_uart_exit(void)
{
	int err;

	err = tty_unregister_ldisc(N_LEGOEV3);
	if (err)
		pr_err("Could not unregister LEGOEV3 line dicipline. (%d)\n",
			err);
}
module_exit(legoev3_uart_exit);

MODULE_DESCRIPTION("tty line dicipline for LEGO Mindstorms EV3 sensors");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS_LDISC(N_LEGOEV3);
