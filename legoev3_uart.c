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
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/tty.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/msensor_class.h>

#ifdef DEBUG
#define debug_pr(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define debug_pr(fmt, ...) while(0) { }
#endif

#define LEGOEV3_UART_BUFFER_SIZE	256

#define LEGOEV3_UART_MSG_TYPE_MASK	0xC0
#define LEGOEV3_UART_CMD_SIZE(byte)	(1 << ((byte >> 3) & 0x7))
#define LEGOEV3_UART_MSG_CMD_MASK	0x07
#define LEGOEV3_UART_MAX_DATA_ERR	6

#define LEGOEV3_UART_TYPE_MAX		101
#define LEGOEV3_UART_SPEED_MIN		2400
#define LEGOEV3_UART_SPEED_MID		57600
#define LEGOEV3_UART_SPEED_MAX		460800
#define LEGOEV3_UART_MODE_NAME_SIZE	11

#define LEGOEV3_UART_SEND_ACK_DELAY		10 /* msec */
#define LEGOEV3_UART_DATA_KEEP_ALIVE_TIMEOUT	100 /* msec */

#define LEGOEV3_UART_DEVICE_TYPE_NAME_SIZE	30

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

enum legoev3_uart_data_type {
	LEGOEV3_UART_DATA_8		= 0x00,
	LEGOEV3_UART_DATA_16		= 0x01,
	LEGOEV3_UART_DATA_32		= 0x02,
	LEGOEV3_UART_DATA_FLOAT		= 0x03,
};

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
	LEGOEV3_UART_INFO_FLAG_ALL_INFO		= LEGOEV3_UART_INFO_FLAG_INFO_NAME
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

/**
 * struct legoev3_uart_data - Discipline data for EV3 UART Sensor communication
 * @tty: Pointer to the tty device that the sensor is connected to
 * @sensor: The real sensor device.
 * @in_port: The input port device associated with this tty.
 * @send_ack_work: Used to send ACK after a delay.
 * @change_bitrate_work: Used to change the baud rate after a delay.
 * @keep_alive_timer: Sends a NACK every 100usec when a sensor is connected.
 * @keep_alive_tasklet: Does the actual sending of the NACK.
 * @mode_info: Array of information about each mode of the sensor
 * @type: The type of sensor that we are connected to. *
 * @num_modes: The number of modes that the sensor has. (1-8)
 * @num_view_modes: Number of modes that can be used for data logging. (1-8)
 * @mode: The current mode.
 * @new_mode: The mode requested by set_mode.
 * @raw_min: Min/max values are sent as float data types. This holds the value
 * 	until we read the number of decimal places needed to convert this
 * 	value to an integer.
 * @raw_max: See raw_min.
 * @pct_min: See raw_min.
 * @pct_max: See raw_min.
 * @si_min: See raw_min.
 * @si_max: See raw_min.
 * @new_baud_rate: New baud rate that will be set with legoev3_uart_change_bitrate
 * @info_flags: Flags indicating what information has already been read
 * 	from the sensor.
 * @buffer: Byte array to store received data in between receive_buf interrupts.
 * @write_ptr: The current position in the buffer.
 * @data_watchdog: Watchdog timer for receiving DATA messages.
 * @num_data_err: Number of bad reads when receiving DATA messages.
 * @synced: Flag indicating communications are synchronized with the sensor.
 * @info_done: Flag indicating that all mode info has been received and it is
 * 	OK to start receiving DATA messages.
 * @data_rec: Flag that indicates that good DATA message has been received
 * 	since last watchdog timeout.
 */
struct legoev3_uart_port_data {
	struct tty_struct *tty;
	struct legoev3_port_device *sensor;
	struct legoev3_port_device *in_port;
	struct msensor_device ms;
	struct delayed_work send_ack_work;
	struct work_struct change_bitrate_work;
	struct hrtimer keep_alive_timer;
	struct tasklet_struct keep_alive_tasklet;
	struct completion set_mode_completion;
	struct msensor_mode_info mode_info[MSENSOR_MODE_MAX + 1];
	u8 mode;
	u8 new_mode;
	u32 raw_min;
	u32 raw_max;
	u32 pct_min;
	u32 pct_max;
	u32 si_min;
	u32 si_max;
	speed_t new_baud_rate;
	long unsigned info_flags;
	u8 buffer[LEGOEV3_UART_BUFFER_SIZE];
	unsigned write_ptr;
	char *last_err;
	unsigned num_data_err;
	unsigned synced:1;
	unsigned info_done:1;
	unsigned data_rec:1;
};

u8 legoev3_uart_set_msg_hdr(u8 type, const unsigned long size, u8 cmd)
{
	u8 size_code = (find_last_bit(&size, sizeof(unsigned long)) & 0x7) << 3;

	return (type & LEGOEV3_UART_MSG_TYPE_MASK) | size_code
		| (cmd & LEGOEV3_UART_MSG_CMD_MASK);
}

const struct attribute_group *legoev3_uart_sensor_device_type_attr_groups[] = {
	&legoev3_port_device_type_attr_grp,
	NULL
};

static struct device_type legoev3_uart_sensor_device_type = {
	.name	= "ev3-uart-sensor",
	.groups	= legoev3_uart_sensor_device_type_attr_groups,
};

static struct msensor_mode_info legoev3_uart_default_mode_info = {
	.raw_max = 1023,
	.pct_max = 100,
	.si_max = 1,
	.figures = 4,
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

int legoev3_uart_write_byte(struct tty_struct *tty, const u8 byte)
{
	int ret;

	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	ret = tty_put_char(tty, byte);
	if (tty->ops->flush_chars)
		tty->ops->flush_chars(tty);

	return ret;
}

u8 legoev3_uart_get_mode(void* context)
{
	struct tty_struct *tty = context;
	struct legoev3_uart_port_data *port;

	if (!tty)
		return -ENODEV;

	port = tty->disc_data;

	return port->mode;
}

int legoev3_uart_set_mode(void *context, const u8 mode)
{
	struct tty_struct *tty = context;
	struct legoev3_uart_port_data *port;
	const int data_size = 3;
	u8 data[data_size];
	int ret;

	if (!tty)
		return -ENODEV;

	port = tty->disc_data;
	if (mode >= port->ms.num_modes)
		return -EINVAL;

	if (!completion_done(&port->set_mode_completion))
		return -EBUSY;

	data[0] = legoev3_uart_set_msg_hdr(LEGOEV3_UART_MSG_TYPE_CMD,
					   data_size - 2,
					   LEGOEV3_UART_CMD_SELECT);
	data[1] = mode;
	data[2] = 0xFF ^ data[0] ^ data[1];

	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	ret = tty->ops->write(tty, data, data_size);
	if (ret < 0)
		return ret;

	port->new_mode = mode;
	INIT_COMPLETION(port->set_mode_completion);
	ret = wait_for_completion_timeout(&port->set_mode_completion,
					  msecs_to_jiffies(300));
	if (!ret)
		return -ETIMEDOUT;

	return 0;
}

static ssize_t legoev3_uart_write_data(void *context, char *data, loff_t off,
                                       size_t count)
{
	struct tty_struct *tty = context;
	char uart_data[MSENSOR_RAW_DATA_SIZE + 2];
	int size, i, err;

	if (off != 0 || count > MSENSOR_RAW_DATA_SIZE)
		return -EINVAL;
	if (count == 0)
		return count;
	memset(uart_data + 1, 0, MSENSOR_RAW_DATA_SIZE);
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
	uart_data[0] = legoev3_uart_set_msg_hdr(LEGOEV3_UART_MSG_TYPE_CMD, size,
						LEGOEV3_UART_CMD_WRITE);
	data[size + 1] = 0xFF;
	for (i = 0; i <= size; i++)
		uart_data[size + 1] ^= uart_data[i];
	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	err = tty->ops->write(tty, uart_data, size + 2);
	if (err < 0)
		return err;

	return count;
}

int legoev3_uart_match_input_port(struct device *dev, void *data)
{
	struct legoev3_port_device *pdev = to_legoev3_port_device(dev);
	struct ev3_input_port_platform_data *pdata;
	char *tty_name = data;

	if (strcmp(pdev->dev.type->name, "ev3-input-port"))
		return 0;
	pdata = dev->platform_data;

	return !strcmp(pdata->uart_tty, tty_name);
}

static void legoev3_uart_send_ack(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct legoev3_uart_port_data *port =
		container_of(dwork, struct legoev3_uart_port_data, send_ack_work);
	struct legoev3_port_device *sensor;
	struct device *in_port_dev;
	int err;

	if (!port->sensor && port->ms.type_id <= LEGOEV3_UART_TYPE_MAX) {
		sensor = legoev3_port_device_register(
				"ev3-uart-sensor",
				-1, /* TODO: get input port ID here */
				&legoev3_uart_sensor_device_type,
				port->ms.type_id,
				&port->ms,
				sizeof(struct msensor_device),
				port->tty->dev);
		if (IS_ERR(sensor)) {
			dev_err(port->tty->dev,
				"Could not register UART sensor on tty %s",
				port->tty->name);
			return;
		}
		port->sensor = sensor;
		/*
		 * This is a special case for the input ports on the EV3 brick.
		 * We use the name of the input port instead of the tty to make
		 * it easier to know which sensor is which.
		 */
		in_port_dev = bus_find_device(&legoev3_bus_type, NULL,
					      port->tty->name,
					      legoev3_uart_match_input_port);
		if (in_port_dev) {
			port->in_port = to_legoev3_port_device(in_port_dev);
			strncpy(port->ms.port_name, dev_name(&port->in_port->dev),
				MSENSOR_PORT_NAME_SIZE);
		} else
			strncpy(port->ms.port_name, port->tty->name,
				MSENSOR_PORT_NAME_SIZE);
		port->ms.context = port->tty;
		err = register_msensor(&port->ms, &port->sensor->dev);
		if (err < 0) {
			dev_err(port->tty->dev,
				"Could not register UART sensor on tty %s",
				port->tty->name);
			legoev3_port_device_unregister(sensor);
			port->sensor = NULL;
			return;
		}
	} else
		dev_err(port->tty->dev, "Reconnected due to: %s\n",
			port->last_err);

	legoev3_uart_write_byte(port->tty, LEGOEV3_UART_SYS_ACK);
	schedule_work(&port->change_bitrate_work);
}

static void legoev3_uart_change_bitrate(struct work_struct *work)
{
	struct legoev3_uart_port_data *port =
		container_of(work, struct legoev3_uart_port_data,
			     change_bitrate_work);
	struct ktermios old_termios = *port->tty->termios;

	tty_wait_until_sent(port->tty, 0);
	mutex_lock(&port->tty->termios_mutex);
	tty_encode_baud_rate(port->tty, port->new_baud_rate, port->new_baud_rate);
	if (port->tty->ops->set_termios)
		port->tty->ops->set_termios(port->tty, &old_termios);
	mutex_unlock(&port->tty->termios_mutex);
	if (port->info_done) {
		hrtimer_start(&port->keep_alive_timer,
			ktime_set(0, LEGOEV3_UART_DATA_KEEP_ALIVE_TIMEOUT / 2
				     * 1000000),
			HRTIMER_MODE_REL);
	}
}

static void legoev3_uart_send_keep_alive(unsigned long data)
{
	struct tty_struct *tty = (void *)data;

	/* NACK is sent as a keep-alive */
	legoev3_uart_write_byte(tty, LEGOEV3_UART_SYS_NACK);
}

enum hrtimer_restart legoev3_uart_keep_alive_timer_callback(struct hrtimer *timer)
{
	struct legoev3_uart_port_data *port =
		container_of(timer, struct legoev3_uart_port_data, keep_alive_timer);

	if (!port->synced || !port->info_done)
		return HRTIMER_NORESTART;

	hrtimer_forward_now(timer, ktime_set(0,
			    LEGOEV3_UART_DATA_KEEP_ALIVE_TIMEOUT * 1000000));
	if (!port->data_rec) {
		port->last_err = "No data since last keep-alive.";
		port->num_data_err++;
	}
	port->data_rec = 0;

	tasklet_schedule(&port->keep_alive_tasklet);

	return HRTIMER_RESTART;
}

static int legoev3_uart_open(struct tty_struct *tty)
{
	struct ktermios old_termios = *tty->termios;
	struct legoev3_uart_port_data *port;

	port = kzalloc(sizeof(struct legoev3_uart_port_data), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	port->tty = tty;
	port->new_baud_rate = LEGOEV3_UART_SPEED_MIN;
	port->ms.type_id = LEGOEV3_TYPE_ID_UNKNOWN;
	port->ms.mode_info = port->mode_info;
	port->ms.get_mode = legoev3_uart_get_mode;
	port->ms.set_mode = legoev3_uart_set_mode;
	port->ms.write_data = legoev3_uart_write_data;
	INIT_DELAYED_WORK(&port->send_ack_work, legoev3_uart_send_ack);
	INIT_WORK(&port->change_bitrate_work, legoev3_uart_change_bitrate);
	hrtimer_init(&port->keep_alive_timer, HRTIMER_BASE_MONOTONIC, HRTIMER_MODE_REL);
	port->keep_alive_timer.function = legoev3_uart_keep_alive_timer_callback;
	tasklet_init(&port->keep_alive_tasklet, legoev3_uart_send_keep_alive,
		     (unsigned long)tty);
	init_completion(&port->set_mode_completion);
	tty->disc_data = port;

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
	tty->low_latency = 1;

	/* flush any existing data in the buffer */
	if (tty->ldisc->ops->flush_buffer)
		tty->ldisc->ops->flush_buffer(tty);
	tty_driver_flush_buffer(tty);

	return 0;
}

static void legoev3_uart_close(struct tty_struct *tty)
{
	struct legoev3_uart_port_data *port = tty->disc_data;

	if (port->sensor) {
		unregister_msensor(&port->ms);
		legoev3_port_device_unregister(port->sensor);
	}
	if (port->in_port)
		put_device(&port->in_port->dev);
	cancel_delayed_work_sync(&port->send_ack_work);
	cancel_work_sync(&port->change_bitrate_work);
	hrtimer_cancel(&port->keep_alive_timer);
	tasklet_kill(&port->keep_alive_tasklet);
	if (!completion_done(&port->set_mode_completion))
		complete(&port->set_mode_completion);
	tty->disc_data = NULL;
	kfree(port);
}

static int legoev3_uart_ioctl(struct tty_struct *tty, struct file *file,
                              unsigned int cmd, unsigned long arg)
{
	return tty_mode_ioctl(tty, file, cmd, arg);
}

static void legoev3_uart_receive_buf(struct tty_struct *tty,
                                     const unsigned char *cp, char *fp,
                                     int count)
{
	struct legoev3_uart_port_data *port = tty->disc_data;
	int i = 0;
	int j, speed;
	u8 cmd, cmd2, type, mode, msg_type, msg_size, chksum;

#ifdef DEBUG
	printk("received: ");
	for (i = 0; i < count; i++)
		printk("0x%02x ", cp[i]);
	printk(" (%d)\n", count);
	i=0;
#endif

	/*
	 * To get in sync with the data stream from the sensor, we look
	 * for a valid TYPE command.
	 */
	while (!port->synced) {
		if (i + 2 >= count)
			return;
		cmd = cp[i++];
		if (cmd != (LEGOEV3_UART_MSG_TYPE_CMD | LEGOEV3_UART_CMD_TYPE))
			continue;
		type = cp[i];
		if (!type || type > LEGOEV3_UART_TYPE_MAX)
			continue;
		chksum = 0xFF ^ cmd ^ type;
		if (cp[i + 1] != chksum)
			continue;
		port->ms.num_modes = 1;
		port->ms.num_view_modes = 1;
		for (j = 0; j <= MSENSOR_MODE_MAX; j++)
			port->mode_info[j] = legoev3_uart_default_mode_info;
		port->ms.type_id = type;
		port->info_flags = LEGOEV3_UART_INFO_FLAG_CMD_TYPE;
		port->synced = 1;
		port->info_done = 0;
		port->write_ptr = 0;
		port->data_rec = 0;
		port->num_data_err = 0;
		i += 2;
	}
	if (!port->synced)
		return;

	/*
	 * Once we are synced, we keep reading data until we have read
	 * a complete command.
	 */
	while (i < count) {
		if (port->write_ptr >= LEGOEV3_UART_BUFFER_SIZE) {
			port->last_err = "Receive buffer overrun.";
			goto err_invalid_state;
		}
		port->buffer[port->write_ptr++] = cp[i++];
	}

	/*
	 * Process all complete messages that have been received.
	 */
	while ((msg_size = legoev3_uart_msg_size(port->buffer[0]))
	        <= port->write_ptr)
	{
#ifdef DEBUG
		printk("processing: ");
		for (i = 0; i < port->write_ptr; i++)
			printk("0x%02x ", port->buffer[i]);
		printk(" (%d)\n", port->write_ptr);
		printk("msg_size:%d\n", msg_size);
#endif
		/*
		 * The IR sensor sends 0xFF after SYNC (0x00). If these two
		 * bytes get split between two interrupts, then it will throw
		 * us off and prevent the sensor from being recognized. So,
		 * if the first byte is 0xFF, we just ignore it and continue
		 * with our loop.
		 */
		if (port->buffer[0] == 0xFF) {
			msg_size = 1;
			goto err_split_sync_checksum;
		}
		if (msg_size > MSENSOR_RAW_DATA_SIZE + 2) {
			port->last_err = "Bad message size.";
			goto err_invalid_state;
		}
		msg_type = port->buffer[0] & LEGOEV3_UART_MSG_TYPE_MASK;
		cmd = port->buffer[0] & LEGOEV3_UART_MSG_CMD_MASK;
		mode = cmd;
		cmd2 = port->buffer[1];
		if (msg_size > 1) {
			chksum = 0xFF;
			for (i = 0; i < msg_size - 1; i++)
				chksum ^= port->buffer[i];
			debug_pr("chksum:%d, actual:%d\n",
			         chksum, port->buffer[msg_size - 1]);
			/*
			 * The LEGO EV3 color sensor (type 29) sends bad checksums
			 * for RGB-RAW data (mode 4). The check here could be
			 * improved if someone can find a pattern.
			 */
			if (chksum != port->buffer[msg_size - 1]
			    && port->ms.type_id != 29 && port->buffer[0] != 0xDC)
			{
				port->last_err = "Bad checksum.";
				if (port->info_done) {
					port->num_data_err++;
					goto err_bad_data_msg_checksum;
				} else
					goto err_invalid_state;
			}
		}
		switch (msg_type) {
		case LEGOEV3_UART_MSG_TYPE_SYS:
			debug_pr("SYS:%d\n", port->buffer[0] & LEGOEV3_UART_MSG_CMD_MASK);
			switch(cmd) {
			case LEGOEV3_UART_SYS_SYNC:
				/* IR sensor (type 33) sends checksum after SYNC */
				if (msg_size > 1 && (cmd ^ cmd2) == 0xFF)
					msg_size++;
				break;
			case LEGOEV3_UART_SYS_ACK:
				if (!port->ms.num_modes) {
					port->last_err = "Received ACK before all mode INFO.";
					goto err_invalid_state;
				}
				if ((port->info_flags & LEGOEV3_UART_INFO_FLAG_REQUIRED)
				    != LEGOEV3_UART_INFO_FLAG_REQUIRED)
				{
					port->last_err = "Did not receive all required INFO.";
					goto err_invalid_state;
				}
				schedule_delayed_work(&port->send_ack_work,
				                      msecs_to_jiffies(LEGOEV3_UART_SEND_ACK_DELAY));
				port->info_done = 1;
				break;
			}
			break;
		case LEGOEV3_UART_MSG_TYPE_CMD:
			debug_pr("CMD:%d\n", cmd);
			switch (cmd) {
			case LEGOEV3_UART_CMD_MODES:
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_CMD_MODES,
						     &port->info_flags))
				{
					port->last_err = "Received duplicate modes INFO.";
					goto err_invalid_state;
				}
				if (!cmd2 || cmd2 > MSENSOR_MODE_MAX) {
					port->last_err = "Number of modes is out of range.";
					goto err_invalid_state;
				}
				port->ms.num_modes = cmd2 + 1;
				if (msg_size > 3)
					port->ms.num_view_modes = port->buffer[2] + 1;
				else
					port->ms.num_view_modes = port->ms.num_modes;
				debug_pr("num_modes:%d, num_view_modes:%d\n",
					 port->ms.num_modes, port->ms.num_view_modes);
				break;
			case LEGOEV3_UART_CMD_SPEED:
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_CMD_SPEED,
						     &port->info_flags))
				{
					port->last_err = "Received duplicate speed INFO.";
					goto err_invalid_state;
				}
				speed = *(int*)(port->buffer + 1);
				if (speed < LEGOEV3_UART_SPEED_MIN
				    || speed > LEGOEV3_UART_SPEED_MAX)
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
		case LEGOEV3_UART_MSG_TYPE_INFO:
			debug_pr("INFO:%d, mode:%d\n", cmd2, mode);
			switch (cmd2) {
			case LEGOEV3_UART_INFO_NAME:
				port->info_flags &= ~LEGOEV3_UART_INFO_FLAG_ALL_INFO;
				if (port->buffer[2] < 'A' || port->buffer[2] > 'z') {
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
				port->buffer[msg_size - 1] = 0;
				if (strlen(port->buffer + 2) > LEGOEV3_UART_MODE_NAME_SIZE) {
					port->last_err = "Name is too long.";
					goto err_invalid_state;
				}
				snprintf(port->mode_info[mode].name,
				         LEGOEV3_UART_MODE_NAME_SIZE + 1, "%s",
				         port->buffer + 2);
				port->mode = mode;
				port->info_flags |= LEGOEV3_UART_INFO_FLAG_INFO_NAME;
				debug_pr("mode %d name:%s\n",
				       mode, port->mode_info[mode].port_name);
				break;
			case LEGOEV3_UART_INFO_RAW:
				if (port->mode != mode) {
					port->last_err = "Received INFO for incorrect mode.";
					goto err_invalid_state;
				}
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_RAW,
						     &port->info_flags))
				{
					port->last_err = "Received duplicate raw scaling INFO.";
					goto err_invalid_state;
				}
				port->raw_min = *(u32 *)(port->buffer + 2);
				port->raw_max = *(u32 *)(port->buffer + 6);
				debug_pr("mode %d raw_min:%08x, raw_max:%08x\n",
				       mode, port->mode_info[mode].raw_min,
				       port->mode_info[mode].raw_max);
				break;
			case LEGOEV3_UART_INFO_PCT:
				if (port->mode != mode) {
					port->last_err = "Received INFO for incorrect mode.";
					goto err_invalid_state;
				}
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_PCT,
						     &port->info_flags))
				{
					port->last_err = "Received duplicate percent scaling INFO.";
					goto err_invalid_state;
				}
				port->pct_min = *(u32 *)(port->buffer + 2);
				port->pct_max = *(u32 *)(port->buffer + 6);
				debug_pr("mode %d pct_min:%08x, pct_max:%08x\n",
				       mode, port->mode_info[mode].pct_min,
				       port->mode_info[mode].pct_max);
				break;
			case LEGOEV3_UART_INFO_SI:
				if (port->mode != mode) {
					port->last_err = "Received INFO for incorrect mode.";
					goto err_invalid_state;
				}
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_SI,
						     &port->info_flags))
				{
					port->last_err = "Received duplicate SI scaling INFO.";
					goto err_invalid_state;
				}
				port->si_min = *(u32 *)(port->buffer + 2);
				port->si_max = *(u32 *)(port->buffer + 6);
				debug_pr("mode %d si_min:%08x, si_max:%08x\n",
				       mode, port->mode_info[mode].si_min,
				       port->mode_info[mode].si_max);
				break;
			case LEGOEV3_UART_INFO_UNITS:
				if (port->mode != mode) {
					port->last_err = "Received INFO for incorrect mode.";
					goto err_invalid_state;
				}
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_UNITS,
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
				port->buffer[msg_size - 1] = 0;
				snprintf(port->mode_info[mode].units,
					 MSENSOR_UNITS_SIZE + 1, "%s",
					 port->buffer + 2);
				debug_pr("mode %d units:%s\n",
				       mode, port->mode_info[mode].units);
				break;
			case LEGOEV3_UART_INFO_FORMAT:
				if (port->mode != mode) {
					port->last_err = "Received INFO for incorrect mode.";
					goto err_invalid_state;
				}
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_FORMAT,
						     &port->info_flags))
				{
					port->last_err = "Received duplicate format INFO.";
					goto err_invalid_state;
				}
				port->mode_info[mode].data_sets = port->buffer[2];
				if (!port->mode_info[mode].data_sets) {
					port->last_err = "Invalid number of data sets.";
					goto err_invalid_state;
				}
				if (msg_size < 7) {
					port->last_err = "Invalid format message size.";
					goto err_invalid_state;
				}
				if ((port->info_flags & LEGOEV3_UART_INFO_FLAG_REQUIRED)
						!= LEGOEV3_UART_INFO_FLAG_REQUIRED) {
					port->last_err = "Did not receive all required INFO.";
					goto err_invalid_state;
				}
				switch (port->buffer[3]) {
				case LEGOEV3_UART_DATA_8:
					port->mode_info[mode].data_type = MSENSOR_DATA_S8;
					break;
				case LEGOEV3_UART_DATA_16:
					port->mode_info[mode].data_type = MSENSOR_DATA_S16;
					break;
				case LEGOEV3_UART_DATA_32:
					port->mode_info[mode].data_type = MSENSOR_DATA_S32;
					break;
				case LEGOEV3_UART_DATA_FLOAT:
					port->mode_info[mode].data_type = MSENSOR_DATA_FLOAT;
					break;
				default:
					port->last_err = "Invalid data type.";
					goto err_invalid_state;
				}
				port->mode_info[mode].figures = port->buffer[4];
				port->mode_info[mode].decimals = port->buffer[5];
				if (port->info_flags & LEGOEV3_UART_INFO_FLAG_INFO_RAW) {
					port->mode_info[mode].raw_min =
						msensor_ftoi(port->raw_min,
							port->mode_info[mode].decimals);
					port->mode_info[mode].raw_max =
						msensor_ftoi(port->raw_max,
							port->mode_info[mode].decimals);
				}
				if (port->info_flags & LEGOEV3_UART_INFO_FLAG_INFO_PCT) {
					port->mode_info[mode].pct_min =
						msensor_ftoi(port->pct_min,
							port->mode_info[mode].decimals);
					port->mode_info[mode].pct_max =
						msensor_ftoi(port->pct_max,
							port->mode_info[mode].decimals);
				}
				if (port->info_flags & LEGOEV3_UART_INFO_FLAG_INFO_SI) {
					port->mode_info[mode].si_min =
						msensor_ftoi(port->si_min,
							port->mode_info[mode].decimals);
					port->mode_info[mode].si_max =
						msensor_ftoi(port->si_max,
							port->mode_info[mode].decimals);
				}
				if (port->mode)
					port->mode--;
				debug_pr("mode %d data_sets:%d, data_type:%d, figures:%d, decimals:%d\n",
					mode, port->mode_info[mode].data_sets,
					port->mode_info[mode].data_type,
					port->mode_info[mode].figures,
					port->mode_info[mode].decimals);
				break;
			}
			break;
		case LEGOEV3_UART_MSG_TYPE_DATA:
			debug_pr("DATA:%d\n", port->buffer[0] & LEGOEV3_UART_MSG_CMD_MASK);
			if (!port->info_done) {
				port->last_err = "Received DATA before INFO was complete.";
				goto err_invalid_state;
			}
			if (mode > MSENSOR_MODE_MAX) {
				port->last_err = "Invalid mode received.";
				goto err_invalid_state;
			}
			if (mode != port->mode) {
				if (mode == port->new_mode)
					port->mode = mode;
				else {
					port->last_err = "Unexpected mode.";
					goto err_invalid_state;
				}
			}
			if (!completion_done(&port->set_mode_completion)
			    && mode == port->new_mode)
				complete(&port->set_mode_completion);
			memcpy(port->mode_info[mode].raw_data,
			       port->buffer + 1, msg_size - 2);
			port->data_rec = 1;
			if (port->num_data_err)
				port->num_data_err--;
			break;
		}

err_bad_data_msg_checksum:
		if (port->info_done && port->num_data_err > LEGOEV3_UART_MAX_DATA_ERR)
			goto err_invalid_state;
err_split_sync_checksum:
		/*
		 * If there is leftover data, we move it to the beginning
		 * of the buffer.
		 */
		for (i = 0; i + msg_size < port->write_ptr; i++)
			port->buffer[i] = port->buffer[i + msg_size];
		port->write_ptr = i;
	}
	return;

err_invalid_state:
	port->synced = 0;
	port->new_baud_rate = LEGOEV3_UART_SPEED_MIN;
	schedule_work(&port->change_bitrate_work);
}

static void legoev3_uart_write_wakeup(struct tty_struct *tty)
{
	debug_pr("%s\n", __func__);
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
