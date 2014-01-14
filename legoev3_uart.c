/*
 * tty line dicipline for LEGO Mindstorms EV3 sensors
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
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/tty.h>

static int legoev3_uart_open(struct tty_struct *tty)
{
	struct ktermios old_termios = *tty->termios;
printk("%s\n", __func__);
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

	/* 8bits, no parity, 1 stop */
	tty->termios->c_cflag = B2400 | CS8 | CREAD | HUPCL | CLOCAL;
	tty->ops->set_termios(tty, &old_termios);
	tty->ops->tiocmset(tty, 0, ~0); /* clear all */
	mutex_unlock(&tty->termios_mutex);

	/* flush any existing data in the buffer */
	if (tty->ldisc->ops->flush_buffer)
		tty->ldisc->ops->flush_buffer(tty);
	tty_driver_flush_buffer(tty);

	return 0;
}

static void legoev3_uart_close(struct tty_struct *tty)
{printk("%s\n", __func__);
	printk("%s\n", __func__);
}

static void legoev3_uart_receive_buf(struct tty_struct *tty,
				     const unsigned char *cp,
				     char *fp, int count)
{printk("%s\n", __func__);
	printk("received: 0x%x\n", *cp);
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
	return n_tty_ioctl_helper(tty, file, cmd, arg);
}

static unsigned int legoev3_uart_poll(struct tty_struct *tty,
				      struct file *filp, poll_table *wait)
{printk("%s\n", __func__);
	return 0;
}

static int __init legoev3_uart_init(void)
{
	static struct tty_ldisc_ops legoev3_uart_ldisc;
	int err;

	memset(&legoev3_uart_ldisc, 0, sizeof(struct tty_ldisc_ops));
	legoev3_uart_ldisc.name		= "n_legoev3";
	legoev3_uart_ldisc.open		= legoev3_uart_open;
	legoev3_uart_ldisc.close	= legoev3_uart_close;
	legoev3_uart_ldisc.read		= legoev3_uart_read;
	legoev3_uart_ldisc.write	= legoev3_uart_write;
	legoev3_uart_ldisc.ioctl	= legoev3_uart_ioctl;
	legoev3_uart_ldisc.poll		= legoev3_uart_poll;
	legoev3_uart_ldisc.receive_buf	= legoev3_uart_receive_buf;
	legoev3_uart_ldisc.owner	= THIS_MODULE;

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
