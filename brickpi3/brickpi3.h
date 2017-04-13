/*
 * Dexter Industries BrickPi3 driver
 *
 * Copyright (C) 2017 David Lechner <david@lechnology.com>
 *
 * Based on brickpi3.h by:
 * Copyright (c) 2016 Dexter Industries
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _BRICKPI3_H_
#define _BRICKPI3_H_

enum brickpi3_input_port {
	BRICKPI3_PORT_IN1,
	BRICKPI3_PORT_IN2,
	BRICKPI3_PORT_IN3,
	BRICKPI3_PORT_IN4,
	NUM_BRICKPI3_INPUT_PORTS
};

enum brickpi3_output_port {
	BRICKPI3_PORT_OUTA,
	BRICKPI3_PORT_OUTB,
	BRICKPI3_PORT_OUTC,
	BRICKPI3_PORT_OUTD,
	NUM_BRICKPI3_OUTPUT_PORTS
};

enum brickpi3_message {
	BRICKPI3_MSG_NONE,
	BRICKPI3_MSG_GET_MANUFACTURER,
	BRICKPI3_MSG_GET_NAME,
	BRICKPI3_MSG_GET_HARDWARE_VERSION,
	BRICKPI3_MSG_GET_FIRMWARE_VERSION,
	BRICKPI3_MSG_GET_ID,
	BRICKPI3_MSG_SET_LED,
	BRICKPI3_MSG_GET_VOLTAGE_3V3,
	BRICKPI3_MSG_GET_VOLTAGE_5V,
	BRICKPI3_MSG_GET_VOLTAGE_9V,
	BRICKPI3_MSG_GET_VOLTAGE_VCC,
	BRICKPI3_MSG_SET_ADDRESS,
	BRICKPI3_MSG_SET_SENSOR_TYPE,
#define	BRICKPI3_MSG_GET_SENSOR \
	BRICKPI3_MSG_GET_SENSOR_1
	BRICKPI3_MSG_GET_SENSOR_1,
	BRICKPI3_MSG_GET_SENSOR_2,
	BRICKPI3_MSG_GET_SENSOR_3,
	BRICKPI3_MSG_GET_SENSOR_4,
#define	BRICKPI3_MSG_I2C_TRANSACT \
	BRICKPI3_MSG_I2C_TRANSACT_1
	BRICKPI3_MSG_I2C_TRANSACT_1,
	BRICKPI3_MSG_I2C_TRANSACT_2,
	BRICKPI3_MSG_I2C_TRANSACT_3,
	BRICKPI3_MSG_I2C_TRANSACT_4,
	BRICKPI3_MSG_SET_MOTOR_POWER,
	BRICKPI3_MSG_SET_MOTOR_POSITION,
	BRICKPI3_MSG_SET_MOTOR_POSITION_KP,
	BRICKPI3_MSG_SET_MOTOR_POSITION_KD,
	BRICKPI3_MSG_SET_MOTOR_DPS,
	BRICKPI3_MSG_SET_MOTOR_DPS_KP,
	BRICKPI3_MSG_SET_MOTOR_DPS_KD,
	BRICKPI3_MSG_SET_MOTOR_LIMITS,
	BRICKPI3_MSG_OFFSET_MOTOR_ENCODER,
#define	BRICKPI3_MSG_GET_MOTOR_ENCODER \
	BRICKPI3_MSG_GET_MOTOR_A_ENCODER
	BRICKPI3_MSG_GET_MOTOR_A_ENCODER,
	BRICKPI3_MSG_GET_MOTOR_B_ENCODER,
	BRICKPI3_MSG_GET_MOTOR_C_ENCODER,
	BRICKPI3_MSG_GET_MOTOR_D_ENCODER,
#define	BRICKPI3_MSG_GET_MOTOR_STATUS \
	BRICKPI3_MSG_GET_MOTOR_A_STATUS
	BRICKPI3_MSG_GET_MOTOR_A_STATUS,
	BRICKPI3_MSG_GET_MOTOR_B_STATUS,
	BRICKPI3_MSG_GET_MOTOR_C_STATUS,
	BRICKPI3_MSG_GET_MOTOR_D_STATUS,
};

enum brickpi3_sensor_type {
	BRICKPI3_SENSOR_TYPE_NONE = 1,
	BRICKPI3_SENSOR_TYPE_I2C,
	BRICKPI3_SENSOR_TYPE_CUSTOM,
	BRICKPI3_SENSOR_TYPE_TOUCH,
	BRICKPI3_SENSOR_TYPE_NXT_TOUCH,
	BRICKPI3_SENSOR_TYPE_EV3_TOUCH,
	BRICKPI3_SENSOR_TYPE_NXT_LIGHT_ON,
	BRICKPI3_SENSOR_TYPE_NXT_LIGHT_OFF,
	BRICKPI3_SENSOR_TYPE_NXT_COLOR_RED,
	BRICKPI3_SENSOR_TYPE_NXT_COLOR_GREEN,
	BRICKPI3_SENSOR_TYPE_NXT_COLOR_BLUE,
	BRICKPI3_SENSOR_TYPE_NXT_COLOR_FULL,
	BRICKPI3_SENSOR_TYPE_NXT_COLOR_OFF,
	BRICKPI3_SENSOR_TYPE_NXT_ULTRASONIC,
	BRICKPI3_SENSOR_TYPE_EV3_GYRO_ABS,
	BRICKPI3_SENSOR_TYPE_EV3_GYRO_DPS,
	BRICKPI3_SENSOR_TYPE_EV3_GYRO_ABS_DPS,
	BRICKPI3_SENSOR_TYPE_EV3_COLOR_REFLECTED,
	BRICKPI3_SENSOR_TYPE_EV3_COLOR_AMBIENT,
	BRICKPI3_SENSOR_TYPE_EV3_COLOR_COLOR,
	BRICKPI3_SENSOR_TYPE_EV3_COLOR_RAW_REFLECTED,
	BRICKPI3_SENSOR_TYPE_EV3_COLOR_COLOR_COMPONENTS,
	BRICKPI3_SENSOR_TYPE_EV3_ULTRASONIC_CM,
	BRICKPI3_SENSOR_TYPE_EV3_ULTRASONIC_INCHES,
	BRICKPI3_SENSOR_TYPE_EV3_ULTRASONIC_LISTEN,
	BRICKPI3_SENSOR_TYPE_EV3_INFRARED_PROXIMITY,
	BRICKPI3_SENSOR_TYPE_EV3_INFRARED_SEEK,
	BRICKPI3_SENSOR_TYPE_EV3_INFRARED_REMOTE,
};

enum brickpi3_sensor_state {
	BRICKPI3_SENSOR_STATE_VALID_DATA,
	BRICKPI3_SENSOR_STATE_NOT_CONFIGURED,
	BRICKPI3_SENSOR_STATE_CONFIGURING,
	BRICKPI3_SENSOR_STATE_NO_DATA,
	BRICKPI3_SENSOR_STATE_I2C_ERROR,
};

enum brickpi3_sensor_pin_flags {
	BRICKPI3_SENSOR_PIN1_9V		= 0x0002,
	BRICKPI3_SENSOR_PIN5_OUT	= 0x0010,
	BRICKPI3_SENSOR_PIN5_STATE	= 0x0020,
	BRICKPI3_SENSOR_PIN6_OUT	= 0x0100,
	BRICKPI3_SENSOR_PIN6_STATE	= 0x0200,
	BRICKPI3_SENSOR_PIN1_ADC	= 0x1000,
	BRICKPI3_SENSOR_PIN6_ADC	= 0x4000,
};

enum brickpi3_i2c_flags {
	BRICKPI3_I2C_MID_CLOCK	= 0x01,
	BRICKPI3_I2C_PIN1_9V	= 0x02,
	BRICKPI3_I2C_SAME	= 0x04,
};

enum brickpi3_motor_status {
	BRICKPI3_MOTOR_STATUS_LOW_VOLTAGE_FLOAT = 0x01,
};

#define BRICKPI3_I2C_MAX_WRITE_SIZE	16
#define BRICKPI3_I2C_MAX_READ_SIZE	14

enum brickpi3_result {
	BRICKPI3_RESULT_SUCCESS,
	BRICKPI3_RESULT_SPI_ERROR,
	BRICKPI3_RESULT_SENSOR_ERROR,
	BRICKPI3_RESULT_SENSOR_TYPE_ERROR,
};

enum brickpi3_in_port_mode {
	BRICKPI3_IN_PORT_MODE_NONE,
	BRICKPI3_IN_PORT_MODE_NXT_ANALOG,
	BRICKPI3_IN_PORT_MODE_NXT_COLOR,
	BRICKPI3_IN_PORT_MODE_NXT_I2C,
	BRICKPI3_IN_PORT_MODE_EV3_ANALOG,
	BRICKPI3_IN_PORT_MODE_EV3_UART,
	NUM_BRICKPI3_IN_PORT_MODES
};

enum brickpi3_out_port_mode {
	BRICKPI3_OUT_PORT_MODE_TACHO_MOTOR,
	BRICKPI3_OUT_PORT_MODE_DC_MOTOR,
	BRICKPI3_OUT_PORT_MODE_LED,
	NUM_BRICKPI3_OUT_PORT_MODES
};

struct brickpi3;

int brickpi3_write_u8(struct brickpi3 *bp, u8 address,
		      enum brickpi3_message msg, u8 value);
int brickpi3_write_u8_u8(struct brickpi3 *bp, u8 address,
			 enum brickpi3_message msg, u8 value1, u8 value2);
int brickpi3_read_u16(struct brickpi3 *bp, u8 address,
		      enum brickpi3_message msg, u16 *value);
int brickpi3_write_u16(struct brickpi3 *bp, u8 address,
		       enum brickpi3_message msg, u16 value);
int brickpi3_write_u8_u16(struct brickpi3 *bp, u8 address,
			  enum brickpi3_message msg, u8 value1, u16 value2);
int brickpi3_write_u24(struct brickpi3 *bp, u8 address,
		       enum brickpi3_message msg, u32 value);
int brickpi3_read_u32(struct brickpi3 *bp, u8 address,
		      enum brickpi3_message msg, u32 *value);
int brickpi3_write_u32(struct brickpi3 *bp, u8 address,
		       enum brickpi3_message msg, u32 value);
int brickpi3_write_u8_u32(struct brickpi3 *bp, u8 address,
			  enum brickpi3_message msg, u8 value1, u32 value2);
int brickpi3_read_string(struct brickpi3 *bp, u8 address,
			 enum brickpi3_message msg, char *value, size_t len);
int brickpi3_read_sensor(struct brickpi3 *bp, u8 address,
			 enum brickpi3_input_port port,
			 enum brickpi3_sensor_type type, char *value,
			 size_t len);
int brickpi3_set_sensor_custom(struct brickpi3 *bp, u8 address,
			       enum brickpi3_input_port port,
			       enum brickpi3_sensor_pin_flags flags);
int brickpi3_set_sensor_i2c(struct brickpi3 *bp, u8 address,
			    enum brickpi3_input_port port,
			    enum brickpi3_i2c_flags flags,
			    u8 speed);
int brickpi3_i2c_transact(struct brickpi3 *bp, u8 address,
			  enum brickpi3_input_port port, u8 addr, u8 *write_buf,
			  u8 write_size, u8 *read_buf, u8 read_size);
int brickpi3_set_motor_limits(struct brickpi3 *bp, u8 address,
			      enum brickpi3_output_port, u8 duty_cycle_sp,
			      u16 speed);

static inline int brickpi3_set_sensor_type(struct brickpi3 *bp, u8 address,
					   enum brickpi3_input_port port,
					   enum brickpi3_sensor_type type)
{
	return brickpi3_write_u8_u8(bp, address, BRICKPI3_MSG_SET_SENSOR_TYPE,
				    BIT(port), type);
}

static inline int brickpi3_get_motor_encoder(struct brickpi3 *bp, u8 address,
					     enum brickpi3_output_port port,
					     s32 *value)
{
	return brickpi3_read_u32(bp, address,
				 BRICKPI3_MSG_GET_MOTOR_ENCODER + port, value);
}

static inline int brickpi3_set_motor_offset(struct brickpi3 *bp, u8 address,
					    enum brickpi3_output_port port,
					    s32 value)
{
	return brickpi3_write_u8_u32(bp, address,
				     BRICKPI3_MSG_OFFSET_MOTOR_ENCODER,
				     BIT(port), value);
}

static inline int brickpi3_run_unregulated(struct brickpi3 *bp, u8 address,
					   enum brickpi3_output_port port,
					   s8 duty_cycle)
{
	return brickpi3_write_u8_u8(bp, address, BRICKPI3_MSG_SET_MOTOR_POWER,
				    BIT(port), duty_cycle);
}

static inline int brickpi3_run_regulated(struct brickpi3 *bp, u8 address,
					 enum brickpi3_output_port port,
					 s16 speed)
{
	return brickpi3_write_u8_u16(bp, address, BRICKPI3_MSG_SET_MOTOR_DPS,
				     BIT(port), speed);
}

static inline int brickpi3_run_to_position(struct brickpi3 *bp, u8 address,
					   enum brickpi3_output_port port,
					   s32 position)
{
	return brickpi3_write_u8_u32(bp, address,
				     BRICKPI3_MSG_SET_MOTOR_POSITION,
				     BIT(port), position);
}

int devm_brickpi3_register_i2c(struct device *dev, struct brickpi3 *bp, u8 address);
int devm_brickpi3_register_iio(struct device *dev, struct brickpi3 *bp, u8 address);
int devm_brickpi3_register_leds(struct device *dev, struct brickpi3 *bp, u8 address);
int devm_brickpi3_register_in_ports(struct device *dev, struct brickpi3 *bp, u8 address);
int devm_brickpi3_register_out_ports(struct device *dev, struct brickpi3 *bp, u8 address);

#endif /* _BRICKPI3_H_ */
