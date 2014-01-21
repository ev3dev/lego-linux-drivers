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
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/tty.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/legoev3_uart.h>

#ifdef DEBUG
#define debug_pr(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define debug_pr(fmt, ...) while(0) { }
#endif

#define LEGOEV3_UART_BUFFER_SIZE	50
#define LEGOEV3_UART_NAME_SIZE		11
#define LEGOEV3_UART_UNITS_SIZE		4
#define LEGOEV3_UART_SENSOR_DATA_SIZE	32

#define LEGOEV3_UART_MSG_TYPE_MASK	0xC0
#define LEGOEV3_UART_CMD_SIZE(byte)	(1 << ((byte >> 3) & 0x7))
#define LEGOEV3_UART_MSG_CMD_MASK	0x07
#define LEGOEV3_UART_TYPE_MAX		101
#define LEGOEV3_UART_MODE_MAX		7
#define LEGOEV3_UART_MAX_DATA_ERR	6

#define LEGOEV3_UART_TYPE_UNKNOWN	125
#define LEGOEV3_UART_SPEED_MIN		2400
#define LEGOEV3_UART_SPEED_MID		57600
#define LEGOEV3_UART_SPEED_MAX		460800

#define LEGOEV3_UART_SEND_ACK_DELAY		10 /* msec */
#define LEGOEV3_UART_SET_BITRATE_DELAY		10 /* msec */
#define LEGOEV3_UART_DATA_WATCHDOG_TIMEOUT	100 /* msec */

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

enum legoev3_uart_data_type {
	LEGOEV3_UART_DATA_8,
	LEGOEV3_UART_DATA_16,
	LEGOEV3_UART_DATA_32,
	LEGOEV3_UART_DATA_FLOAT,
};

static char legoev3_uart_sensor_type_names[LEGOEV3_UART_TYPE_MAX + 1][LEGOEV3_UART_DEVICE_TYPE_NAME_SIZE];
static struct device_type legoev3_uart_sensor_device_types[LEGOEV3_UART_TYPE_MAX + 1];

/**
 * struct legoev3_uart_sensor_mode_info
 * @mode: The id of this mode. (0-7)
 * @name: The name of this mode
 * @raw_min: The minimum raw value of the data read.
 * @raw_min: The maximum raw value of the data read.
 * @pct_min: The minimum percentage value of the data read.
 * @pct_min: The maximum percentage value of the data read.
 * @si_min: The minimum scaled value of the data read.
 * @si_min: The maximum scaled value of the data read.
 * @units: Units of the scaled value.
 * @data_sets: Number of data points in DATA message.
 * @format: Format of data in DATA message.
 * @figures: Number of digits that should be displayed, including decimal point.
 * @decimals: Decimal point position.
 * @raw_data: Raw data read from the sensor.
 *
 * All of the min an max values are actually 32-bit float data type,
 * but kernel drivers don't do floating point, so we use unsigned instead.
 */
struct legoev3_uart_mode_info {
	u8 mode;
	char name[LEGOEV3_UART_NAME_SIZE + 1];
	unsigned raw_min;
	unsigned raw_max;
	unsigned pct_min;
	unsigned pct_max;
	unsigned si_min;
	unsigned si_max;
	char units[LEGOEV3_UART_UNITS_SIZE + 1];
	u8 data_sets;
	enum legoev3_uart_data_type format;
	u8 figures;
	u8 decimals;
	u8 raw_data[32];
};

static struct legoev3_uart_mode_info legoev3_uart_default_mode_info = {
	.raw_max	= 0x447fc000,	/* 1023.0 */
	.pct_max	= 0x42c80000,	/*  100.0 */
	.si_max		= 0x3f800000,	/*    1.0 */
	.figures	= 4,
};

/**
 * struct legoev3_uart_data - Discipline data for EV3 UART Sensor communication
 * @tty: Pointer to the tty device that the sensor is connected to
 * @sensor: The real sensor device.
 * @send_ack_work: Used to send ACK after a delay.
 * @change_bitrate_work: Used to change the baud rate after a delay.
 * @mode_info: Array of information about each mode of the sensor
 * @type: The type of sensor that we are connected to. *
 * @num_modes: The number of modes that the sensor has. (1-8)
 * @num_view_modes: Number of modes that can be used for data logging. (1-8)
 * @mode: The current mode.
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
struct legoev3_uart_data {
	struct tty_struct *tty;
	struct legoev3_port_device *sensor;
	struct legoev3_uart_sensor_platform_data pdata;
	struct delayed_work send_ack_work;
	struct delayed_work change_bitrate_work;
	struct legoev3_uart_mode_info mode_info[LEGOEV3_UART_MODE_MAX + 1];
	u8 type;
	u8 num_modes;
	u8 num_view_modes;
	u8 mode;
	speed_t new_baud_rate;
	long unsigned info_flags;
	u8 buffer[LEGOEV3_UART_BUFFER_SIZE];
	unsigned write_ptr;
	unsigned long data_watchdog;
	unsigned num_data_err;
	unsigned synced:1;
	unsigned info_done:1;
	unsigned data_rec:1;
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
	while (!(ret = tty->ops->write(tty, &byte, 1)));

	return ret < 0 ? ret : 0;
}

static void legoev3_uart_send_ack(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct legoev3_uart_data *data =
		container_of(dwork, struct legoev3_uart_data, send_ack_work);
	struct legoev3_port_device *sensor;

	if (!data->sensor && data->type <= LEGOEV3_UART_TYPE_MAX) {
		data->pdata.tty = data->tty;
		data->pdata.mode_info = data->mode_info;
		data->pdata.num_modes = data->num_modes;
		data->pdata.num_view_modes = data->num_view_modes;
		sensor = legoev3_port_device_register(
				legoev3_uart_sensor_type_names[data->type],
				-1, /* TODO: get input port ID here */
				&legoev3_uart_sensor_device_types[data->type],
				&data->pdata,
				sizeof(struct legoev3_uart_sensor_platform_data),
				data->tty->dev);
		if (IS_ERR(sensor)) {
			dev_err(data->tty->dev, "Could not register UART sensor on tty %s",
					data->tty->name);
			return;
		}
		data->sensor = sensor;
	}

	legoev3_uart_write_byte(data->tty, LEGOEV3_UART_SYS_ACK);
	schedule_delayed_work(&data->change_bitrate_work,
	                      msecs_to_jiffies(LEGOEV3_UART_SET_BITRATE_DELAY));
}

static void legoev3_uart_change_bitrate(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct legoev3_uart_data *data =
		container_of(dwork, struct legoev3_uart_data, change_bitrate_work);
	struct ktermios old_termios = *data->tty->termios;

	mutex_lock(&data->tty->termios_mutex);
	tty_encode_baud_rate(data->tty, data->new_baud_rate, data->new_baud_rate);
	if (data->tty->ops->set_termios)
			data->tty->ops->set_termios(data->tty, &old_termios);
	mutex_unlock(&data->tty->termios_mutex);
}

static int legoev3_uart_open(struct tty_struct *tty)
{
	struct ktermios old_termios = *tty->termios;
	struct legoev3_uart_data *data;

	data = kzalloc(sizeof(struct legoev3_uart_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->tty = tty;
	data->type = LEGOEV3_UART_TYPE_UNKNOWN;
	data->new_baud_rate = LEGOEV3_UART_SPEED_MIN;
	INIT_DELAYED_WORK(&data->send_ack_work, legoev3_uart_send_ack);
	INIT_DELAYED_WORK(&data->change_bitrate_work, legoev3_uart_change_bitrate);
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
	struct legoev3_uart_data *data = tty->disc_data;

	if (data->sensor)
		legoev3_port_device_unregister(data->sensor);
	cancel_delayed_work_sync(&data->send_ack_work);
	cancel_delayed_work_sync(&data->change_bitrate_work);
	tty->disc_data = NULL;
	kfree(data);
}

static int legoev3_uart_ioctl(struct tty_struct *tty, struct file *file,
			      unsigned int cmd, unsigned long arg)
{
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

#ifdef DEBUG
	printk("received: ");
	for (i = 0; i < count; i++)
		printk("0x%02x ", cp[i]);
	printk(" (%d)\n", count);
	i=0;
#endif

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
		data->num_modes = 1;
		data->num_view_modes = 1;
		for (j = 0; j <= LEGOEV3_UART_MODE_MAX; j++)
			data->mode_info[j] = legoev3_uart_default_mode_info;
		data->type = type;
		data->info_flags = LEGOEV3_UART_INFO_FLAG_CMD_TYPE;
		data->synced = 1;
		data->info_done = 0;
		data->write_ptr = 0;
		i += 2;
	}
	if (!data->synced)
		return;

	if (data->info_done && time_is_before_jiffies(data->data_watchdog)) {
		data->data_watchdog =
			jiffies + msecs_to_jiffies(LEGOEV3_UART_DATA_WATCHDOG_TIMEOUT);
		if (!data->data_rec) {
			data->num_data_err++;

		}
		/* it appears that NACK is sent as a keep-alive */
		legoev3_uart_write_byte(tty, LEGOEV3_UART_SYS_NACK);
		data->data_rec = 0;
	}
	/*
	 * Once we are synced, we keep reading data until we have read
	 * a complete command.
	 */
	while (i < count) {
		if (data->write_ptr >= LEGOEV3_UART_BUFFER_SIZE)
			goto err_invalid_state;
		data->buffer[data->write_ptr++] = cp[i++];
	}

	/*
	 * Process all complete messages that have been received.
	 */
	while ((msg_size = legoev3_uart_msg_size(data->buffer[0]))
	        <= data->write_ptr)
	{
#ifdef DEBUG
		printk("processing: ");
		for (i = 0; i < data->write_ptr; i++)
			printk("0x%02x ", data->buffer[i]);
		printk(" (%d)\n", data->write_ptr);
		printk("msg_size:%d\n", msg_size);
#endif
		msg_type = data->buffer[0] & LEGOEV3_UART_MSG_TYPE_MASK;
		cmd = data->buffer[0] & LEGOEV3_UART_MSG_CMD_MASK;
		mode = cmd;
		cmd2 = data->buffer[1];
		if (msg_size > 1) {
			chksum = 0xFF;
			for (i = 0; i < msg_size - 1; i++)
				chksum ^= data->buffer[i];
			debug_pr("chksum:%d, actual:%d\n",
			       chksum, data->buffer[msg_size - 1]);
			if (chksum != data->buffer[msg_size - 1]) {
				if (data->info_done) {
					data->num_data_err++;
					goto err_bad_data_msg_checksum;
				}
				else
					goto err_invalid_state;
			}
		}
		switch (msg_type) {
		case LEGOEV3_UART_MSG_TYPE_SYS:
			debug_pr("SYS:%d\n", data->buffer[0] & LEGOEV3_UART_MSG_CMD_MASK);
			switch(cmd) {
			case LEGOEV3_UART_SYS_SYNC:
				if ((cmd ^ cmd2) == 0xFF)
					msg_size++;
				break;
			case LEGOEV3_UART_SYS_ACK:
				if (!data->num_modes)
					/* we are still receiving mode data */
					goto err_invalid_state;
				if ((data->info_flags & LEGOEV3_UART_INFO_FLAG_REQUIRED)
						!= LEGOEV3_UART_INFO_FLAG_REQUIRED)
					goto err_invalid_state;
				schedule_delayed_work(&data->send_ack_work,
				                      msecs_to_jiffies(LEGOEV3_UART_SEND_ACK_DELAY));
				data->info_done = 1;
				data->data_watchdog = jiffies
					+ msecs_to_jiffies(LEGOEV3_UART_SEND_ACK_DELAY
							+ LEGOEV3_UART_SET_BITRATE_DELAY);
				data->num_data_err = 0;
				break;
			}
			break;
		case LEGOEV3_UART_MSG_TYPE_CMD:
			debug_pr("CMD:%d\n", cmd);
			switch (cmd) {
			case LEGOEV3_UART_CMD_MODES:
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_CMD_MODES,
						&data->info_flags))
					goto err_invalid_state;
				if (!cmd2 || cmd2 > LEGOEV3_UART_MODE_MAX)
					goto err_invalid_state;
				data->num_modes = cmd2 + 1;
				if (msg_size > 3)
					data->num_view_modes = data->buffer[2] + 1;
				else
					data->num_view_modes = data->num_modes;
				debug_pr("num_modes:%d, num_view_modes:%d\n",
				       data->num_modes, data->num_view_modes);
				break;
			case LEGOEV3_UART_CMD_SPEED:
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_CMD_SPEED,
						&data->info_flags))
					goto err_invalid_state;
				speed = *(int*)(data->buffer + 1);
				if (speed < LEGOEV3_UART_SPEED_MIN
						|| speed > LEGOEV3_UART_SPEED_MAX)
					goto err_invalid_state;
				data->new_baud_rate = speed;
				debug_pr("speed:%d\n", speed);
				break;
			default:
				goto err_invalid_state;
			}
			break;
		case LEGOEV3_UART_MSG_TYPE_INFO:
			debug_pr("INFO:%d, mode:%d\n", cmd2, mode);
			switch (cmd2) {
			case LEGOEV3_UART_INFO_NAME:
				data->info_flags &= ~LEGOEV3_UART_INFO_FLAG_ALL_INFO;
				if (data->buffer[2] < 'A' || data->buffer[2] > 'z')
					goto err_invalid_state;
				/*
				 * Name may not have null terminator and we
				 * are done with the checksum at this point
				 * so we are writing 0 over the checksum to
				 * ensure a null terminator for the string
				 * functions.
				 */
				data->buffer[msg_size - 1] = 0;
				if (strlen(data->buffer + 2) > LEGOEV3_UART_NAME_SIZE)
					goto err_invalid_state;
				snprintf(data->mode_info[mode].name,
				         LEGOEV3_UART_NAME_SIZE + 1, "%s",
				         data->buffer + 2);
				data->mode = mode;
				data->info_flags |= LEGOEV3_UART_INFO_FLAG_INFO_NAME;
				debug_pr("mode %d name:%s\n",
				       mode, data->mode_info[mode].name);
				break;
			case LEGOEV3_UART_INFO_RAW:
				if (data->mode != mode)
					goto err_invalid_state;
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_RAW,
						&data->info_flags))
					goto err_invalid_state;
				data->mode_info[mode].raw_min =
						*(unsigned *)(data->buffer + 2);
				data->mode_info[mode].raw_max =
						*(unsigned *)(data->buffer + 6);
				debug_pr("mode %d raw_min:%08x, raw_max:%08x\n",
				       mode, data->mode_info[mode].raw_min,
				       data->mode_info[mode].raw_max);
				break;
			case LEGOEV3_UART_INFO_PCT:
				if (data->mode != mode)
					goto err_invalid_state;
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_PCT,
						&data->info_flags))
					goto err_invalid_state;
				data->mode_info[mode].pct_min =
						*(unsigned *)(data->buffer + 2);
				data->mode_info[mode].pct_max =
						*(unsigned *)(data->buffer + 6);
				debug_pr("mode %d pct_min:%08x, pct_max:%08x\n",
				       mode, data->mode_info[mode].pct_min,
				       data->mode_info[mode].pct_max);
				break;
			case LEGOEV3_UART_INFO_SI:
				if (data->mode != mode)
					goto err_invalid_state;
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_SI,
						&data->info_flags))
					goto err_invalid_state;
				data->mode_info[mode].si_min =
						*(unsigned *)(data->buffer + 2);
				data->mode_info[mode].si_max =
						*(unsigned *)(data->buffer + 6);
				debug_pr("mode %d si_min:%08x, si_max:%08x\n",
				       mode, data->mode_info[mode].si_min,
				       data->mode_info[mode].si_max);
				break;
			case LEGOEV3_UART_INFO_UNITS:
				if (data->mode != mode)
					goto err_invalid_state;
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_UNITS,
						&data->info_flags))
					goto err_invalid_state;
				/*
				 * Units may not have null terminator and we
				 * are done with the checksum at this point
				 * so we are writing 0 over the checksum to
				 * ensure a null terminator for the string
				 * functions.
				 */
				data->buffer[msg_size - 1] = 0;
				snprintf(data->mode_info[mode].units,
					 LEGOEV3_UART_UNITS_SIZE + 1, "%s",
					 data->buffer + 2);
				debug_pr("mode %d units:%s\n",
				       mode, data->mode_info[mode].units);
				break;
			case LEGOEV3_UART_INFO_FORMAT:
				if (data->mode != mode)
					goto err_invalid_state;
				if (test_and_set_bit(LEGOEV3_UART_INFO_BIT_INFO_FORMAT,
						&data->info_flags))
					goto err_invalid_state;
				data->mode_info[mode].data_sets = data->buffer[2];
				if (!data->mode_info[mode].data_sets)
					goto err_invalid_state;
				if (msg_size < 7)
					goto err_invalid_state;
				if ((data->info_flags & LEGOEV3_UART_INFO_FLAG_REQUIRED)
						!= LEGOEV3_UART_INFO_FLAG_REQUIRED)
					goto err_invalid_state;
				data->mode_info[mode].format = data->buffer[3];
				if (data->mode) {
					data->mode--;
					data->mode_info[mode].figures = data->buffer[4];
					data->mode_info[mode].decimals = data->buffer[5];
					/* TODO: copy IR Seeker hack from lms2012 */
				}
				debug_pr("mode %d data_sets:%d, format:%d, figures:%d, decimals:%d\n",
				       mode, data->mode_info[mode].data_sets,
				       data->mode_info[mode].format,
				       data->mode_info[mode].figures,
				       data->mode_info[mode].decimals);
				break;
			}
			break;
		case LEGOEV3_UART_MSG_TYPE_DATA:
			debug_pr("DATA:%d\n", data->buffer[0] & LEGOEV3_UART_MSG_CMD_MASK);
			if (!data->info_done)
				goto err_invalid_state;
			memcpy(data->mode_info[mode].raw_data,
			       data->buffer + 1, msg_size - 2);
			data->data_rec = 1;
			if (data->num_data_err)
				data->num_data_err--;
			break;
		}

err_bad_data_msg_checksum:
		if (data->num_data_err > LEGOEV3_UART_MAX_DATA_ERR)
			goto err_invalid_state;

		/*
		 * If there is leftover data, we move it to the beginning
		 * of the buffer.
		 */
		for (i = 0; i + msg_size < data->write_ptr; i++)
			data->buffer[i] = data->buffer[i + msg_size];
		data->write_ptr = i;
	}
	return;

err_invalid_state:
	data->synced = 0;
	data->new_baud_rate = LEGOEV3_UART_SPEED_MIN;
	schedule_delayed_work(&data->change_bitrate_work,
	                      msecs_to_jiffies(LEGOEV3_UART_SET_BITRATE_DELAY));
}

static void legoev3_uart_write_wakeup(struct tty_struct *tty)
{
	debug_pr("%s\n", __func__);
}

const struct attribute_group *ev3_sensor_device_type_attr_groups[] = {
	&legoev3_port_device_type_attr_grp,
	NULL
};

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
	int err, i;

	err = tty_register_ldisc(N_LEGOEV3, &legoev3_uart_ldisc);
	if (err) {
		pr_err("Could not register LEGOEV3 line discipline. (%d)\n",
			err);
		return err;
	}
	for (i = 0; i <= LEGOEV3_UART_TYPE_MAX; i++){
		snprintf(legoev3_uart_sensor_type_names[i],
				LEGOEV3_UART_DEVICE_TYPE_NAME_SIZE,
				"ev3-uart-sensor-type-%d", i);
	}
	for (i = 0; i <= LEGOEV3_UART_TYPE_MAX; i++){
		struct device_type *type = &legoev3_uart_sensor_device_types[i];
		type->name = legoev3_uart_sensor_type_names[i];
		type->groups = ev3_sensor_device_type_attr_groups;
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
