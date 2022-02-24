/*
 * Dexter Industries BrickPi driver
 *
 * Copyright (C) 2015-2018 David Lechner <david@lechnology.com>
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

/**
 * DOC: board-info
 *
 * The BrickPi board info driver provides the following properties:
 *
 * ``BOARD_INFO_FW_VER``
 *
 *     This will be "1" or "2". Version 2 has (buggy) support for EV3 sensors.
 *
 * ``BOARD_INFO_MODEL``
 *
 *     Will be "Dexter Industries BrickPi". This is the same for both BrickPi
 *     and BrickPi+. There is not a way to tell the difference.
 *
 * ``BOARD_INFO_TYPE``
 *
 *     Always returns ``aux``.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "brickpi_internal.h"
#include "../linux/board_info/board_info.h"

#ifdef DEBUG
#define debug_pr(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define debug_pr(fmt, ...) while(0) { }
#endif


/*
 * This is just about as fast as we can go. Each poll is 2 messages -- 1 to
 * each channel. Each message takes about 1ms. This leaves 2ms open for sending
 * commands.
 */
#define BRICKPI_POLL_MS		4
#define BRICKPI_SPEED_PERIOD	20

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

static void brickpi_append_tx(struct brickpi_data *data, u8 size, long value)
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

static long brickpi_read_rx(struct brickpi_data *data, u8 size)
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

static int brickpi_send_message(struct brickpi_data *data, u8 addr,
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
		serdev_device_write_wakeup(data->serdev);
		ret = serdev_device_write_buf(data->serdev, data->tx_buffer, size);
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
	int i, err;

	mutex_lock(&data->tx_mutex);

	if (data->closing) {
		mutex_unlock(&data->tx_mutex);
		return 0;
	}

	data->tx_buffer_tail = BRICKPI_TX_BUFFER_TAIL_INIT;
	brickpi_append_tx(data, 8, ch_data->in_port[BRICKPI_PORT_1].sensor_type);
	brickpi_append_tx(data, 8, ch_data->in_port[BRICKPI_PORT_2].sensor_type);

	for (i = 0; i < NUM_BRICKPI_PORT; i++) {
		struct brickpi_in_port_data *port = &ch_data->in_port[i];
		int num_msg = port->num_i2c_msg;
		int j;

		if (port->sensor_type == BRICKPI_SENSOR_TYPE_NXT_I2C
		    || port->sensor_type == BRICKPI_SENSOR_TYPE_NXT_I2C_9V) {
			brickpi_append_tx(data, 8, port->i2c_speed);
			brickpi_append_tx(data, 3, num_msg -1);
			for (j = 0; j < num_msg; j++) {
				struct brickpi_i2c_msg_data *msg = &port->i2c_msg[j];

				brickpi_append_tx(data, 7, msg->addr);
				brickpi_append_tx(data, 2, msg->settings);

				if (msg->settings & BRICKPI_I2C_SAME) {
					int k;

					brickpi_append_tx(data, 4, msg->write_size);
					brickpi_append_tx(data, 4, msg->read_size);
					for (k = 0; k < msg->write_size; k++) {
						brickpi_append_tx(data, 8, msg->write_data[k]);
					}
				}
			}
		}
	}

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

	mutex_lock(&data->tx_mutex);

	if (data->closing) {
		mutex_unlock(&data->tx_mutex);
		return 0;
	}

	data->tx_buffer_tail = BRICKPI_TX_BUFFER_TAIL_INIT;

	for (i = 0; i < NUM_BRICKPI_PORT; i++) {
		brickpi_append_tx(data, 1, ch_data->out_port[i].motor_use_offset);
		/* TODO: handle use_offset */
	}

	for (i = 0; i < NUM_BRICKPI_PORT; i++) {
		brickpi_append_tx(data, 1, ch_data->out_port[i].motor_enabled);
		brickpi_append_tx(data, 1, ch_data->out_port[i].motor_reversed);
		brickpi_append_tx(data, 8, ch_data->out_port[i].motor_speed);
	}

	for (i = 0; i < NUM_BRICKPI_PORT; i++) {
		struct brickpi_in_port_data *port = &ch_data->in_port[i];
		int num_msg = port->num_i2c_msg;

		if (port->sensor_type == BRICKPI_SENSOR_TYPE_NXT_I2C
		    || port->sensor_type == BRICKPI_SENSOR_TYPE_NXT_I2C_9V) {
			for (j = 0; j < num_msg; j++) {
				struct brickpi_i2c_msg_data *msg = &port->i2c_msg[j];

				if (!(msg->settings & BRICKPI_I2C_SAME)) {
					int k;

					brickpi_append_tx(data, 4, msg->write_size);
					brickpi_append_tx(data, 4, msg->read_size);
					for (k = 0; k < msg->write_size; k++) {
						brickpi_append_tx(data, 8, msg->write_data[k]);
					}
				}
			}
		}
	}

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
		struct brickpi_out_port_data *port = &ch_data->out_port[i];
		u64 bits = brickpi_read_rx(data, port_size[i]);
		/*
		 * we are ignoring the least significant bit in the position
		 * so that it matches other platforms. The BrickPi counts both
		 * the rising and falling edges of the encoders as 2 counts
		 * whereas other platforms only count this as 1.
		 */
		s64 position = bits >> 2;

		if (bits & 1)
			position *= -1;

		port->motor_position = position;
		tm_speed_update(&port->speed, position, data->rx_time);
		debug_pr("motor_position[%d]: %d\n", i, (int)position);

		if (port->stop_at_target_position) {
			if ((port->motor_reversed
			     && position < port->target_position)
			    || (!port->motor_reversed
			     && position > port->target_position))
			{
				_brickpi_out_port_stop(port);
			}
		}
	}

	for (i = 0; i < NUM_BRICKPI_PORT; i++) {
		struct brickpi_in_port_data *port = &ch_data->in_port[i];
		s32 *sensor_values = port->sensor_values;
		u8 *raw_data = port->port.raw_data;

		switch (port->sensor_type) {
		case BRICKPI_SENSOR_TYPE_NXT_TOUCH:
		case BRICKPI_SENSOR_TYPE_NXT_TOUCH_DEBOUNCED:
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
					port->num_i2c_msg);
			for (j = 0; j < port->num_i2c_msg; j++) {
				if (sensor_values[0] & (1 << j)) {
					int k;
					struct brickpi_i2c_msg_data *msg =
						&port->i2c_msg[j];
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
			sensor_values[0] = brickpi_read_rx(data, 16);
			/*
			 * There is a race condition in the BrickPi firmware v2 that
			 * causes these errors to be returned quite frequently. This
			 * is unfortunate because these may actually be legitimate
			 * values. Hopefully this can be fixed in future versions
			 * of the frimware, in which case we can add a version check
			 * here. But for now, we just ignore these values.
			 */
			if (sensor_values[0] == (s16)(-2))
				continue;
			if (sensor_values[0] == (s16)(-4))
				continue;
			break;
		case BRICKPI_SENSOR_TYPE_EV3_COLOR_M3:
		case BRICKPI_SENSOR_TYPE_EV3_GYRO_M3:
		case BRICKPI_SENSOR_TYPE_EV3_INFRARED_M2:
			sensor_values[0] = brickpi_read_rx(data, 32);
			/* see comment above */
			if (sensor_values[0] == (s32)(-2))
				continue;
			if (sensor_values[0] == (s32)(-4))
				continue;
			break;
		case BRICKPI_SENSOR_TYPE_EV3_TOUCH:
		case BRICKPI_SENSOR_TYPE_EV3_TOUCH_DEBOUNCED:
			/*
			 * The EV3 Touch sensor returns a value of 0x07 or 0x10
			 * when pressed, so this converts it to an approx. mV
			 * value that is close enough for the ev3-analog-sensor
			 * driver to interpret correctly.
			 */
			sensor_values[0] = brickpi_read_rx(data, 16) << 8;
			break;
		default:
			/* NXT Analog expects value in mV */
			if (port->sensor_type <= BRICKPI_SENSOR_TYPE_NXT_ANALOG_MAX)
				sensor_values[0] = brickpi_read_rx(data, 10) * 5000 / 1024;
			else
				sensor_values[0] = brickpi_read_rx(data, 10);
			break;
		}
		debug_pr("ch_data->sensor_values[%d][0]: %ld\n", i,
			 sensor_values[0]);

		if (!raw_data)
			continue;

		if (port->sensor_type == BRICKPI_SENSOR_TYPE_NXT_I2C
		   || port->sensor_type == BRICKPI_SENSOR_TYPE_NXT_I2C_9V) {

			if (port->port.last_changed_time
			   && memcmp(raw_data, port->i2c_msg[0].read_data,
				     port->i2c_msg[0].read_size) != 0) {
				*port->port.last_changed_time = ktime_get();
				memcpy(raw_data, port->i2c_msg[0].read_data,
				       port->i2c_msg[0].read_size);
			}


		} else {
			int bytes = sizeof(s32) * NUM_BRICKPI_SENSOR_VALUES;

			if (port->port.last_changed_time
			   && memcmp(raw_data, sensor_values, bytes) != 0) {
				*port->port.last_changed_time = ktime_get();
				memcpy(raw_data, sensor_values, bytes);
			}

		}
		lego_port_call_raw_data_func(&port->port);
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

	data->rx_time = ktime_get();

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
		dev_err(&data->serdev->dev, "Buffer overrun.\n");
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
		dev_err(&data->serdev->dev, "Bad checksum.\n");
		data->rx_data_size = 0;
		return;
	}

	complete(&data->rx_completion);
}

static void brickpi_update_motor(struct brickpi_out_port_data *port)
{
	if (port->motor_enabled) {
		int duty_cycle;

		if (port->speed_pid_ena) {
			if (port->speed_pid.setpoint == 0) {
				duty_cycle = 0;
				tm_pid_reinit(&port->speed_pid);
			} else {
				duty_cycle = tm_pid_update(&port->speed_pid,
						tm_speed_get(&port->speed));
			}
		} else if (port->hold_pid_ena) {
			duty_cycle = tm_pid_update(&port->hold_pid,
							port->motor_position);
		} else
			duty_cycle = port->direct_duty_cycle;

		port->motor_speed = BRICKPI_DUTY_PCT_TO_RAW(abs(duty_cycle));
		port->motor_reversed = duty_cycle < 0;
	}
}

static const enum board_info_property brickpi_board_info_properties[] = {
	BOARD_INFO_FW_VER,
	BOARD_INFO_MODEL,
	BOARD_INFO_TYPE,
};

static int brickpi_board_info_get_property(struct board_info *info,
					   enum board_info_property prop,
					   const char **val)
{
	struct brickpi_data *data = board_info_get_drvdata(info);

	switch (prop) {
	case BOARD_INFO_FW_VER:
		*val = data->fw_ver;
		break;
	case BOARD_INFO_MODEL:
		*val = "Dexter Industries BrickPi";
		break;
	case BOARD_INFO_TYPE:
		*val = BOARD_INFO_TYPE_NAME_AUX;
		break;
	default:
	return -EINVAL;
	}

	return 0;
}

static void brickpi_poll_work(struct work_struct *work)
{
	struct brickpi_data *data = container_of(work, struct brickpi_data,
						 poll_work);
	int i, err;

	if (data->closing)
		return;

	for (i = 0; i < data->num_channels; i++) {
		struct brickpi_channel_data *ch_data = &data->channel_data[i];

		if (ch_data->init_ok) {
			brickpi_update_motor(&ch_data->out_port[BRICKPI_PORT_1]);
			brickpi_update_motor(&ch_data->out_port[BRICKPI_PORT_2]);
			err = brickpi_get_values(ch_data);

			if (err < 0)
				debug_pr("failed to get values for address %d. (%d)\n",
					 ch_data->address, err);
		}
	}
}

static int brickpi_serdev_init(struct brickpi_data *data)
{
	struct device *dev = &data->serdev->dev;
	int i, err;

	for (i = 0; i < data->num_channels; i++) {
		struct brickpi_channel_data *ch_data = &data->channel_data[i];
		struct brickpi_in_port_data *in_port_1 =
					&ch_data->in_port[BRICKPI_PORT_1];
		struct brickpi_in_port_data *in_port_2 =
					&ch_data->in_port[BRICKPI_PORT_2];
		struct brickpi_out_port_data *out_port_1 =
					&ch_data->out_port[BRICKPI_PORT_1];
		struct brickpi_out_port_data *out_port_2 =
					&ch_data->out_port[BRICKPI_PORT_2];

		ch_data->data = data;
		// TODO: add module parameter to get addresses
		ch_data->address = i + 1;
		in_port_1->ch_data = ch_data;
		in_port_1->sensor_type = BRICKPI_SENSOR_TYPE_FW_VERSION;
		in_port_2->ch_data = ch_data;
		in_port_2->sensor_type = BRICKPI_SENSOR_TYPE_FW_VERSION;

		err = brickpi_set_sensors(ch_data);
		if (err < 0) {
			dev_err(dev, "Failed to init while setting sensors\n");
			return err;
		}

		out_port_1->ch_data = ch_data;
		out_port_2->ch_data = ch_data;
		err = brickpi_get_values(ch_data);

		if (err < 0) {
			dev_err(dev, "Failed to init while getting values\n");
			return err;
		}

		tm_speed_init(&out_port_1->speed, out_port_1->motor_position,
			data->rx_time, BRICKPI_SPEED_PERIOD / BRICKPI_POLL_MS);
		tm_speed_init(&out_port_1->speed, out_port_1->motor_position,
			data->rx_time, BRICKPI_SPEED_PERIOD / BRICKPI_POLL_MS);
		_brickpi_out_port_reset(out_port_1);
		_brickpi_out_port_reset(out_port_2);
		ch_data->fw_version = in_port_1->sensor_values[0];
		dev_dbg(dev, "address: %d, fw: %d\n", ch_data->address,
			ch_data->fw_version);

		err = brickpi_register_in_ports(ch_data, dev);
		if (err < 0) {
			dev_err(dev, "Failed to register input ports");
			return err;
		}

		err = brickpi_register_out_ports(ch_data, dev);
		if (err < 0) {
			brickpi_unregister_in_ports(ch_data);
			dev_err(dev, "Failed to register output ports");
			return err;
		}
		ch_data->init_ok = true;
	}

	data->desc.properties = brickpi_board_info_properties;
	data->desc.num_properties = ARRAY_SIZE(brickpi_board_info_properties);
	data->desc.get_property = brickpi_board_info_get_property;
	/* Just using the first channel. Assuming both are the same. */
	snprintf(data->fw_ver, BRICKPI_FW_VERSION_SIZE, "%u",
		 data->channel_data[0].fw_version);

	data->board = board_info_register(dev, &data->desc, data);
	if (IS_ERR(data->board)) {
		dev_warn(dev, "Failed to register board info (%ld)\n",
			 PTR_ERR(data->board));
		data->board = NULL;
	}

	return 0;
}

static enum hrtimer_restart brickpi_poll_timer_function(struct hrtimer *timer)
{
	struct brickpi_data *data = container_of(timer, struct brickpi_data,
						 poll_timer);

	hrtimer_forward_now(timer, ms_to_ktime(BRICKPI_POLL_MS));

	if (data->closing)
		return HRTIMER_NORESTART;

	schedule_work(&data->poll_work);

	return HRTIMER_RESTART;
}

static int brickpi_serdev_receive_buf(struct serdev_device *serdev,
				      const unsigned char *buf, size_t count)
{
	struct brickpi_data *data = serdev_device_get_drvdata(serdev);

	if (data->closing)
		return 0;

	/* check if there is enough room in the rx_buffer */
	if (data->rx_data_size + count > BRICKPI_BUFFER_SIZE)
		return 0;

	memcpy(data->rx_buffer + data->rx_data_size, buf, count);
	data->rx_data_size += count;

	schedule_work(&data->rx_data_work);

	return count;
}

static void brickpi_serdev_write_wakeup(struct serdev_device *serdev)
{
	/* TODO: do we need to do anything here? */
}

static const struct serdev_device_ops brickpi_serdev_ops = {
	.receive_buf	= brickpi_serdev_receive_buf,
	.write_wakeup	= brickpi_serdev_write_wakeup,
};

static int brickpi_serdev_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct brickpi_data *data;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->num_channels = 2;
	data->channel_data = devm_kzalloc(dev, sizeof(*data->channel_data)
					  * data->num_channels, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->serdev = serdev;
	mutex_init(&data->tx_mutex);
	init_completion(&data->rx_completion);
	INIT_WORK(&data->rx_data_work, brickpi_handle_rx_data);
	INIT_WORK(&data->poll_work, brickpi_poll_work);
	hrtimer_init(&data->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->poll_timer.function = brickpi_poll_timer_function;

	serdev_device_set_drvdata(serdev, data);
	serdev_device_set_client_ops(serdev, &brickpi_serdev_ops);

	ret = serdev_device_open(serdev);
	if (ret) {
		dev_err(dev, "Unable to open serdev\n");
		return ret;
	}

	serdev_device_set_baudrate(serdev, 500000);
	serdev_device_set_flow_control(serdev, false);

	ret = brickpi_serdev_init(data);
	if (ret < 0) {
		serdev_device_close(serdev);
		return ret;
	}

	hrtimer_start(&data->poll_timer, ktime_set(0, 0), HRTIMER_MODE_REL);

	return 0;
}

static void brickpi_serdev_remove(struct serdev_device *serdev)
{
	struct brickpi_data *data = serdev_device_get_drvdata(serdev);
	int i;

	mutex_lock(&data->tx_mutex);
	data->closing = true;
	mutex_unlock(&data->tx_mutex);
	hrtimer_cancel(&data->poll_timer);
	cancel_work_sync(&data->poll_work);
	cancel_work_sync(&data->rx_data_work);
	board_info_unregister(data->board);

	for (i = 0; i < data->num_channels; i++) {
		struct brickpi_channel_data *ch_data = &data->channel_data[i];
		if (ch_data->init_ok) {
			brickpi_unregister_in_ports(ch_data);
			brickpi_unregister_out_ports(ch_data);
		}
	}

	serdev_device_close(serdev);
}

static const struct of_device_id brickpi_serdev_of_match[] = {
	{ .compatible = "ev3dev,brickpi" },
	{ }
};
MODULE_DEVICE_TABLE(of, brickpi_serdev_of_match);

static struct serdev_device_driver brickpi_serdev_driver = {
	.driver		= {
		.name	= "brickpi-serdev",
		.of_match_table = brickpi_serdev_of_match,
	},
	.probe	= brickpi_serdev_probe,
	.remove	= brickpi_serdev_remove,
};
module_serdev_device_driver(brickpi_serdev_driver);

MODULE_DESCRIPTION("Dexter Industries BrickPi tty line discipline");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
