/*
 * HiTechnic NXT sensor multiplexer device driver
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

#ifndef HT_NXT_SMUX_H_
#define HT_NXT_SMUX_H_

#include <lego_port_class.h>

#include "nxt_i2c_sensor.h"

#define HT_NXT_SMUX_COMMAND_REG		0x20
#define HT_NXT_SMUX_STATUS_REG		0x21

enum ht_nxt_smux_channel {
	HT_NXT_SMUX_CH1,
	HT_NXT_SMUX_CH2,
	HT_NXT_SMUX_CH3,
	HT_NXT_SMUX_CH4,
	NUM_HT_NXT_SMUX_CH
};

#define HT_NXT_SMUX_CH1_CONFIG_REG	0x22
#define HT_NXT_SMUX_CH2_CONFIG_REG	0x27
#define HT_NXT_SMUX_CH3_CONFIG_REG	0x2C
#define HT_NXT_SMUX_CH4_CONFIG_REG	0x31

/* offset from ht_nxt_smux_config_reg */
#define HT_NXT_SMUX_CFG_MODE		0
#define HT_NXT_SMUX_CFG_TYPE		1
#define HT_NXT_SMUX_CFG_I2C_CNT		2
#define HT_NXT_SMUX_CFG_I2C_ADDR	3
#define HT_NXT_SMUX_CFG_I2C_REG		4

#define HT_NXT_SMUX_CH1_ANALOG_DATA_REG	0x36
#define HT_NXT_SMUX_CH2_ANALOG_DATA_REG	0x38
#define HT_NXT_SMUX_CH3_ANALOG_DATA_REG	0x3A
#define HT_NXT_SMUX_CH4_ANALOG_DATA_REG	0x3C

#define HT_NXT_SMUX_CH1_I2C_DATA_REG	0x40
#define HT_NXT_SMUX_CH2_I2C_DATA_REG	0x50
#define HT_NXT_SMUX_CH3_I2C_DATA_REG	0x60
#define HT_NXT_SMUX_CH4_I2C_DATA_REG	0x70

#define HT_NXT_SMUX_COMMAND_HALT	0
#define HT_NXT_SMUX_COMMAND_DETECT	1
#define HT_NXT_SMUX_COMMAND_RUN		2

#define HT_NXT_SMUX_STATUS_BATT		BIT(0) /* Battery is low/disconnected */
#define HT_NXT_SMUX_STATUS_BUSY		BIT(1) /* Sensor mux is in run mode */
#define HT_NXT_SMUX_STATUS_HALT		BIT(2) /* Sensor mux is halted */
#define HT_NXT_SMUX_STATUS_ERROR	BIT(3) /* Something bad happened */

#define HT_NXT_SMUX_CONFIG_I2C		BIT(0) /* Connected sensor is I2C */
#define HT_NXT_SMUX_CONFIG_9V_EN	BIT(1) /* Pin 1 set high (batt voltage) */
#define HT_NXT_SMUX_CONFIG_DIG0		BIT(2) /* Pin 5 set high */
#define HT_NXT_SMUX_CONFIG_DIG1		BIT(3) /* Pin 6 set high */
#define HT_NXT_SMUX_CONFIG_SLOW		BIT(4) /* I2C clock rate is 20kHz instead of 80kHz */

enum ht_nxt_smux_detected_sensor {
	HT_NXT_SMUX_SENSOR_ANALOG,
	HT_NXT_SMUX_SENSOR_LEGO_ULTRASONIC,
	HT_NXT_SMUX_SENSOR_HT_COMPASS,
	HT_NXT_SMUX_SENSOR_HT_COLOR,
	HT_NXT_SMUX_SENSOR_HT_ACCEL,
	HT_NXT_SMUX_SENSOR_HT_IR_SEEKER,
	HT_NXT_SMUX_SENSOR_HT_PROTO_BOARD,
	HT_NXT_SMUX_SENSOR_HT_COLOR_V2,
	HT_NXT_SMUX_SENSOR_RESERVED,
	HT_NXT_SMUX_SENSOR_IR_SEEKER_V2,
	NUM_HT_NXT_SMUX_SENSOR_TYPE
};

extern void ht_nxt_smux_port_set_pin1_gpio(struct lego_port_device *port,
					   enum lego_port_gpio_state state);
extern void ht_nxt_smux_port_set_pin5_gpio(struct lego_port_device *port,
					   enum lego_port_gpio_state state);
extern void ht_nxt_smux_port_set_i2c_addr(struct lego_port_device *port, u8 addr, bool slow);
extern void ht_nxt_smux_port_set_i2c_data_reg(struct lego_port_device *port, u8 reg, u8 count);

extern int ht_nxt_smux_send_cmd_pre_cb(struct nxt_i2c_sensor_data * sensor, u8 command);
extern void ht_nxt_smux_send_cmd_post_cb(struct nxt_i2c_sensor_data *data, u8 command);
extern void ht_nxt_smux_poll_cb(struct nxt_i2c_sensor_data *data);
extern int ht_nxt_smux_probe_cb(struct nxt_i2c_sensor_data *data);
extern void ht_nxt_smux_remove_cb(struct nxt_i2c_sensor_data *data);

struct ht_nxt_smux_i2c_sensor_platform_data {
	u8 address;
};

extern const struct lego_port_type ht_nxt_smux_port_type;

#endif /* HT_NXT_SMUX_H_ */
