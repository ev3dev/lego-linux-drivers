# Makefile for LEGO device drivers

KBUILD_CFLAGS += -I$(src)/include

obj-y	+= core/
obj-y	+= sensors/

# EV3

obj-$(CONFIG_LEGOEV3_ANALOG)		+= legoev3_analog.o
obj-$(CONFIG_LEGOEV3_BLUETOOTH)		+= legoev3_bluetooth.o
obj-$(CONFIG_LEGOEV3_DEV_PORTS)		+= legoev3_ports.o
obj-$(CONFIG_LEGOEV3_DEV_PORTS)		+= ev3_input_port.o
obj-$(CONFIG_LEGOEV3_DEV_PORTS)		+= ev3_output_port.o

# Motor classes
obj-$(CONFIG_LEGOEV3_MOTORS)		+= tacho_motor_class.o
obj-$(CONFIG_LEGOEV3_SERVO_MOTORS)	+= servo_motor_class.o
obj-$(CONFIG_LEGOEV3_DC_MOTORS)		+= dc_motor_class.o

# Motors
obj-$(CONFIG_LEGOEV3_TACHO_MOTORS)	+= ev3_tacho_motor.o
obj-$(CONFIG_LEGOEV3_DC_MOTORS)		+= rcx_motor.o
obj-$(CONFIG_LEGOEV3_DC_MOTORS)		+= rcx_led.o

# Sensor classes
obj-$(CONFIG_LEGOEV3_MSENSORS)		+= msensor_class.o

# Analog sensors
obj-$(CONFIG_NXT_ANALOG_SENSORS)	+= nxt_analog_host.o
nxt_analog_sensor-objs := nxt_analog_sensor_core.o nxt_analog_sensor_defs.o
obj-$(CONFIG_NXT_ANALOG_SENSORS)	+= nxt_analog_sensor.o
obj-$(CONFIG_EV3_ANALOG_SENSORS)	+= ev3_analog_host.o

# I2C Sensors
obj-$(CONFIG_LEGOEV3_I2C_SENSORS)	+= nxt_i2c_host.o
nxt_i2c_sensor-objs := nxt_i2c_sensor_core.o nxt_i2c_sensor_defs.o sensors/ms_ev3_smux.o
obj-$(CONFIG_LEGOEV3_I2C_SENSORS)	+= nxt_i2c_sensor.o
obj-$(CONFIG_LEGOEV3_I2C_SENSORS)	+= ht_smux_input_port.o
obj-$(CONFIG_LEGOEV3_I2C_SENSORS)	+= ht_smux_i2c_host.o
obj-$(CONFIG_LEGOEV3_I2C_SENSORS)	+= ht_smux_i2c_sensor.o

# UART Sensors
obj-$(CONFIG_EV3_UART_SENSORS)		+= legoev3_uart.o
obj-$(CONFIG_EV3_UART_SENSORS)		+= ev3_uart_host.o
