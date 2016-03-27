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

#ifndef _BRICKPI_INTERNAL_H_
#define _BRICKPI_INTERNAL_H_

#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/types.h>

#include <lego.h>
#include <lego_port_class.h>
#include <lego_sensor_class.h>
#include <tacho_motor_helper.h>

#include "brickpi.h"

#define BRICKPI_DUTY_PCT_TO_RAW(d) ((d) * 255 / 100)
#define BRICKPI_DUTY_RAW_TO_PCT(d) ((d) * 100 / 255)

/**
 * struct brickpi_i2c_msg_data - Data for a single I2C message
 *
 * @addr: The I2C address
 * @settings: Special settings such as extra clock pulse for NXT Ultrasonic.
 * @write_size: The number of bytes to write. Max value is 16.
 * @write_data: The data to write. First byte is the register to write to.
 * @read_size: The number of bytes to read. Max value is 16.
 * @read_data: The data read back from the sensor.
 */
struct brickpi_i2c_msg_data {
	u8 addr;
	u8 settings;
	u8 write_size;
	u8 write_data[BRICKPI_MAX_I2C_DATA_SIZE];
	u8 read_size;
	u8 read_data[BRICKPI_MAX_I2C_DATA_SIZE];
};

struct brickpi_channel_data;

/**
 * struct brickpi_out_port_data - Data for a single BrickPi input port
 *
 * @ch_data: Pointer to the containing channel.
 * @port: The lego-port class device for each input port.
 * @sensor: The sensor attached to the port.
 * @sensor_values: The sensor value(s) read back from the Arduino.
 * @sensor_type: Tells the Arduino which type of sensor is attached.
 * @i2c_speed: Divider for slowing down the I2C clock. e.g. 10 => ~10kHz
 * @num_i2c_msg: The number of I2C messages to be sent. Max value is 8.
 * @i2c_msg: Data for each I2C message.
 */
struct brickpi_in_port_data {
	struct brickpi_channel_data *ch_data;
	struct lego_port_device port;
	struct lego_device *sensor;
	s32 sensor_values[NUM_BRICKPI_SENSOR_VALUES];
	enum brickpi_sensor_type sensor_type;
	u8 i2c_speed;
	u8 num_i2c_msg;
	struct brickpi_i2c_msg_data i2c_msg[NUM_BRICKPI_I2C_PER_PORT];
};

/**
 * struct brickpi_out_port_data - Data for a single BrickPi output port
 *
 * @ch_data: Pointer to the containing channel.
 * @port: The lego-port class device for each output port.
 * @motor: Pointer to hold device when it is registered.
 * @speed: Speed handling data.
 * @motor_use_offset:
 * @motor_enable: Tells the motor to run or not.
 * @motor_direction: The motor direction to send to the Arduino.
 * @stop_at_target_position: Indicates that we should stop when the target
 * 	position is reached.
 * @motor_speed: The motor speed to send to the Arduino.
 * @motor_position: The position read back from the Arduino.
 * @motor_offset: Keeps track of the difference between the userland position
 * 	value and the kernel position value. Could be sent to the BrickPi, but
 * 	currently just used internally in the kernel.
 * @target_position: The target for run-to-*-pos commands.
 */
struct brickpi_out_port_data {
	struct brickpi_channel_data *ch_data;
	struct lego_port_device port;
	struct lego_device *motor;
	struct tm_speed speed;
	bool motor_use_offset;
	bool motor_enabled;
	bool motor_reversed;
	bool stop_at_target_position;
	u8 motor_speed;
	long motor_position;
	long motor_offset;
	long target_position;
};

struct brickpi_data;

/**
 * struct brickpi_channel_data - Data for a single BrickPi channel
 *
 * By "channel", we mean 1 Arduino microcontroller on a BrickPi, so 2 input
 * ports and 2 output ports.
 *
 * @data: Pointer to containing data structure.
 * @in_port: Data structures for the 2 input ports.
 * @out_port: Data structures for the 2 output ports.
 * @address: The UART address of the Arduino microcontroller.
 * @fw_version: The firmware version of the Arduino.
 * @init_ok: The channel was successfully initialized.
 */
struct brickpi_channel_data {
	struct brickpi_data *data;
	struct brickpi_in_port_data in_port[NUM_BRICKPI_PORT];
	struct brickpi_out_port_data out_port[NUM_BRICKPI_PORT];
	u8 address;
	u8 fw_version;
	bool init_ok;
};

/**
 * struct brickpi_data - Discipline data for EV3 UART Sensor communication
 * @device_name: The name of the device/driver.
 * @tty: Pointer to the tty device that the sensor is connected to
 * @channel_data: Pointer to channel data array.
 * @num_channels: Number of items in channel_data array.
 * @tx_buffer: Array to store the data to be transmitted.
 * @tx_buffer_tail: The index *in bits* of the current end of the tx_buffer.
 * @tx_mutex: Mutex to ensure only on tx request is handled at a time.
 * @rx_buffer: Array to store the received data.
 * @rx_buffer_head: The index *in bits* of the current position in the rx_buffer.
 * @rx_data_size: Size of the received data.
 * @rx_time: Timestamp when data was received.
 * @rx_completion: Completion to wait for received data.
 * @rx_data_work: Workqueue item for handling received data.
 * @poll_work: Work for polling.
 * @poll_timer: Timer for polling.
 * @closing: Flag to indicate that we are closing the connection and any data
 *  received should be ignored.
 */
struct brickpi_data {
	char device_name[LEGO_NAME_SIZE + 1];
	struct tty_struct *tty;
	struct brickpi_channel_data *channel_data;
	unsigned num_channels;
	u8 tx_buffer[BRICKPI_BUFFER_SIZE];
	unsigned tx_buffer_tail;
	struct mutex tx_mutex;
	u8 rx_buffer[BRICKPI_BUFFER_SIZE];
	unsigned rx_buffer_head;
	unsigned rx_data_size;
	ktime_t rx_time;
	struct completion rx_completion;
	struct work_struct rx_data_work;
	struct work_struct poll_work;
	struct hrtimer poll_timer;
	bool closing;
};

int brickpi_set_sensors(struct brickpi_channel_data *ch_data);
int brickpi_get_values(struct brickpi_channel_data *ch_data);

int brickpi_register_in_ports(struct brickpi_channel_data *ch_data,
			      struct device *parent);
void brickpi_unregister_in_ports(struct brickpi_channel_data *ch_data);

int brickpi_register_out_ports(struct brickpi_channel_data *ch_data,
			       struct device *parent);
void brickpi_unregister_out_ports(struct brickpi_channel_data *ch_data);

#endif /* _BRICKPI_INTERNAL_H_ */
