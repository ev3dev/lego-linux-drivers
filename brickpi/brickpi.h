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

#ifndef _BRICKPI_H_
#define _BRICKPI_H_

#define BRICKPI_BUFFER_SIZE		265
#define BRICKPI_MAX_MESSAGE_SIZE	(BRICKPI_BUFFER_SIZE - 2)
#define NUM_BRICKPI_SENSOR_VALUES	5
#define NUM_BRICKPI_I2C_PER_PORT	8
#define BRICKPI_MAX_I2C_DATA_SIZE	16

/* each BrickPi "channel" has 2 input and 2 output ports */
enum brickpi_port {
	BRICKPI_PORT_1,
	BRICKPI_PORT_2,
	NUM_BRICKPI_PORT
};

enum brickpi_message {
	BRICK_PI_MESSAGE_CHANGE_ADDR = 1, /* Change the UART address. */
	BRICK_PI_MESSAGE_SET_SENSOR  = 2, /* Change/set the sensor type. */
	BRICK_PI_MESSAGE_GET_VALUES  = 3, /* Set the motor speed and direction */
	                                  /* and return the sensors and encoders. */
	BRICK_PI_MESSAGE_E_STOP      = 4, /* Float motors immediately */
	BRICK_PI_MESSAGE_SET_TIMEOUT = 5, /* Set the timeout */
};

enum brickpi_sensor_type {
	BRICKPI_SENSOR_TYPE_NXT_ANALOG         = 0,
/* These flags are used when brickpi_sensor_type == 0 */
#define BRICKPI_NXT_ANALOG_FLAG_PIN5_OUT  0x01
#define BRICKPI_NXT_ANALOG_FLAG_PIN6_OUT  0x02
/*	BRICKPI_NXT_ANALOG_FLAG_PIN1_9V   0x04 */
#define BRICKPI_NXT_ANALOG_FLAG_PIN5_HIGH 0x08
#define BRICKPI_NXT_ANALOG_FLAG_PIN6_HIGH 0x10
	BRICKPI_SENSOR_TYPE_NXT_ANALOG_MAX      = 31,
	BRICKPI_SENSOR_TYPE_NXT_TOUCH           = 32,
	BRICKPI_SENSOR_TYPE_NXT_ULTRASONIC_CONT = 33,
	BRICKPI_SENSOR_TYPE_NXT_ULTRASONIC_SS   = 34,
	BRICKPI_SENSOR_TYPE_RCX_LIGHT           = 35,
	BRICKPI_SENSOR_TYPE_NXT_COLOR_FULL      = 36,
	BRICKPI_SENSOR_TYPE_NXT_COLOR_RED       = 37,
	BRICKPI_SENSOR_TYPE_NXT_COLOR_GREEN     = 38,
	BRICKPI_SENSOR_TYPE_NXT_COLOR_BLUE      = 39,
	BRICKPI_SENSOR_TYPE_NXT_COLOR_NONE      = 40,
	BRICKPI_SENSOR_TYPE_NXT_I2C             = 41,
	BRICKPI_SENSOR_TYPE_NXT_I2C_9V          = 42,
	BRICKPI_SENSOR_TYPE_EV3_US_M0           = 43,
	BRICKPI_SENSOR_TYPE_EV3_US_M1           = 44,
	BRICKPI_SENSOR_TYPE_EV3_US_M2           = 45,
	BRICKPI_SENSOR_TYPE_EV3_US_M3           = 46,
	BRICKPI_SENSOR_TYPE_EV3_US_M4           = 47,
	BRICKPI_SENSOR_TYPE_EV3_US_M5           = 48,
	BRICKPI_SENSOR_TYPE_EV3_US_M6           = 49,
	BRICKPI_SENSOR_TYPE_EV3_COLOR_M0        = 50,
	BRICKPI_SENSOR_TYPE_EV3_COLOR_M1        = 51,
	BRICKPI_SENSOR_TYPE_EV3_COLOR_M2        = 52,
	BRICKPI_SENSOR_TYPE_EV3_COLOR_M3        = 53,
	BRICKPI_SENSOR_TYPE_EV3_COLOR_M4        = 54,
	BRICKPI_SENSOR_TYPE_EV3_COLOR_M5        = 55,
	BRICKPI_SENSOR_TYPE_EV3_GYRO_M0         = 56,
	BRICKPI_SENSOR_TYPE_EV3_GYRO_M1         = 57,
	BRICKPI_SENSOR_TYPE_EV3_GYRO_M2         = 58,
	BRICKPI_SENSOR_TYPE_EV3_GYRO_M3         = 59,
	BRICKPI_SENSOR_TYPE_EV3_GYRO_M4         = 60,
	BRICKPI_SENSOR_TYPE_EV3_INFRARED_M0     = 61,
	BRICKPI_SENSOR_TYPE_EV3_INFRARED_M1     = 62,
	BRICKPI_SENSOR_TYPE_EV3_INFRARED_M2     = 63,
	BRICKPI_SENSOR_TYPE_EV3_INFRARED_M3     = 64,
	BRICKPI_SENSOR_TYPE_EV3_INFRARED_M4     = 65,
	BRICKPI_SENSOR_TYPE_EV3_INFRARED_M5     = 66,
	BRICKPI_SENSOR_TYPE_EV3_TOUCH           = 67,
	BRICKPI_SENSOR_TYPE_EV3_TOUCH_DEBOUNCED = 68,
	BRICKPI_SENSOR_TYPE_NXT_TOUCH_DEBOUNCED = 69,
	BRICKPI_SENSOR_TYPE_FW_VERSION          = 70,
};

enum brickpi_in_port_mode {
	BRICKPI_IN_PORT_MODE_NONE,
	BRICKPI_IN_PORT_MODE_NXT_ANALOG,
	BRICKPI_IN_PORT_MODE_NXT_COLOR,
	BRICKPI_IN_PORT_MODE_NXT_I2C,
	BRICKPI_IN_PORT_MODE_EV3_ANALOG,
	BRICKPI_IN_PORT_MODE_EV3_UART,
	NUM_BRICKPI_IN_PORT_MODES
};

/* I2C Settings flags */

/* Extra clock pulse between stop and start for NXT Ultrasonic sensor */
#define BRICKPI_I2C_EXTRA_CLK	0x01
/* The read and write data will not be changed in subsequent calls to get values */
#define BRICKPI_I2C_SAME	0x02

extern const struct device_type brickpi_in_port_type;
extern const struct device_type brickpi_out_port_type;

struct brickpi_i2c_sensor_platform_data {
	u8 address;
};

extern int brickpi_in_port_set_i2c_data(struct lego_device *sensor, bool slow,
					enum lego_port_gpio_state pin1_state);
extern int brickpi_in_port_set_i2c_mode(struct lego_device *sensor, u8 set_mode_reg,
					u8 set_mode_data, u8 read_reg, unsigned size);
extern int brickpi_in_port_set_uart_sensor_mode(struct lego_device *sensor, u8 mode);

#endif /* _BRICKPI_H_ */
