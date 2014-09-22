/*
 * HiTechnic NXT sensor multiplexer device driver for LEGO Mindstorms EV3
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

#ifndef HT_SMUX_H_
#define HT_SMUX_H_

#define HT_SMUX_COMMAND_REG	0x20
#define HT_SMUX_STATUS_REG	0x21

enum ht_smux_channel {
	HT_SMUX_CH1,
	HT_SMUX_CH2,
	HT_SMUX_CH3,
	HT_SMUX_CH4,
	NUM_HT_SMUX_CH
};

#define HT_SMUX_CH1_CONFIG_REG 0x22
#define HT_SMUX_CH2_CONFIG_REG 0x27
#define HT_SMUX_CH3_CONFIG_REG 0x2C
#define HT_SMUX_CH4_CONFIG_REG 0x31

/* offset from ht_smux_config_reg */
#define HT_SMUX_CFG_MODE	0
#define HT_SMUX_CFG_TYPE	1
#define HT_SMUX_CFG_I2C_CNT	2
#define HT_SMUX_CFG_I2C_ADDR	3
#define HT_SMUX_CFG_I2C_REG	4

#define HT_SMUX_CH1_ANALOG_REG 0x36
#define HT_SMUX_CH2_ANALOG_REG 0x38
#define HT_SMUX_CH3_ANALOG_REG 0x3A
#define HT_SMUX_CH4_ANALOG_REG 0x3C

#define HT_SMUX_CH1_I2C_DATA_REG 0x40
#define HT_SMUX_CH2_I2C_DATA_REG 0x50
#define HT_SMUX_CH3_I2C_DATA_REG 0x60
#define HT_SMUX_CH4_I2C_DATA_REG 0x70

#define HT_SMUX_COMMAND_HALT	0
#define HT_SMUX_COMMAND_DETECT	1
#define HT_SMUX_COMMAND_RUN	2

#define HT_SMUX_STATUS_BATT	BIT(0) /* Battery is low/disconnected */
#define HT_SMUX_STATUS_BUSY	BIT(1) /* Sensor mux is in run mode */
#define HT_SMUX_STATUS_HALT	BIT(2) /* Sensor mux is halted */
#define HT_SMUX_STATUS_ERROR	BIT(3) /* Something bad happened */

#define HT_SMUX_CONFIG_I2C	BIT(0) /* Connected sensor is I2C */
#define HT_SMUX_CONFIG_9V_EN	BIT(1) /* Pin 1 set high (batt voltage) */
#define HT_SMUX_CONFIG_DIG0	BIT(2) /* Pin 5 set high */
#define HT_SMUX_CONFIG_DIG1	BIT(3) /* Pin 6 set high */
#define HT_SMUX_CONFIG_SLOW	BIT(4) /* I2C read rate is 20kHz instead of 80kHz */

enum ht_smux_detected_sensor {
	HT_SMUX_SENSOR_NXT_ANALOG,
	HT_SMUX_SENSOR_LEGO_ULTRASONIC,
	HT_SMUX_SENSOR_HT_COMPASS,
	HT_SMUX_SENSOR_HT_COLOR,
	HT_SMUX_SENSOR_HT_ACCEL,
	HT_SMUX_SENSOR_HT_IR_SEEKER,
	HT_SMUX_SENSOR_HT_PROTO_BOARD,
	HT_SMUX_SENSOR_HT_COLOR_V2,
	HT_SMUX_SENSOR_RESERVED,
	HT_SMUX_SENSOR_IR_SEEKER_V2,
	NUM_HT_SMUX_SENSOR_TYPE
};

struct ht_smux_input_port_platform_data {
	struct i2c_client *client;
	enum ht_smux_channel channel;
	u8 *sensor_data;
};

/*
 * This is essentially a union with ev3_analog_host_platform_data, so
 * initial_sensor must be first
 * */
struct ht_smux_i2c_host_platform_data {
	const char* inital_sensor;
	u8 address;
};

struct ht_smux_i2c_sensor_platform_data {
	u8 address;
};

#endif /* HT_SMUX_H_ */
