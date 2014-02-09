# Makefile for LEGO Mindstorms EV3 device drivers

obj-$(CONFIG_LEGOEV3_ANALOG)		+= legoev3_analog.o
obj-$(CONFIG_LEGOEV3_BLUETOOTH)		+= legoev3_bluetooth.o
obj-$(CONFIG_LEGOEV3_DEV_PORTS)		+= legoev3_ports.o
obj-$(CONFIG_LEGOEV3_INPUT_PORT)	+= ev3_input_port.o
obj-$(CONFIG_LEGOEV3_OUTPUT_PORT)	+= ev3_output_port.o

# motor classes
obj-$(CONFIG_LEGOEV3_MOTORS)		+= tacho_motor_class.o

# Motors
obj-$(CONFIG_LEGOEV3_TACHO_MOTORS)	+= ev3_tacho_motor.o

# Sensor classes
obj-$(CONFIG_LEGOEV3_TOUCH_SENSORS)	+= touch_sensor_class.o
obj-$(CONFIG_LEGOEV3_MEASURE_SENSORS)	+= measure_sensor_class.o

# Analog sensors
obj-$(CONFIG_LEGOEV3_EV3_TOUCH_SENSOR)	+= ev3_touch_sensor.o
obj-$(CONFIG_LEGOEV3_NXT_TOUCH_SENSOR)	+= nxt_touch_sensor.o

# I2C Sensors
obj-$(CONFIG_LEGOEV3_I2C_SENSORS)	+= nxt_i2c_sensor.o
obj-$(CONFIG_LEGOEV3_NXT_ULTRASONIC)	+= nxt_ultrasonic.o
obj-$(CONFIG_LEGOEV3_MS_LIGHT_ARRAY)	+= ms_light_array.o

# UART Sensors
obj-$(CONFIG_LEGOEV3_UART_SENSORS)	+= legoev3_uart.o
obj-$(CONFIG_LEGOEV3_UART_SENSORS)	+= ev3_uart_sensor.o

