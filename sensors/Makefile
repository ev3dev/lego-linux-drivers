# Sensor class
obj-$(CONFIG_LEGOEV3_MSENSORS)		+= lego_sensor_class.o

# Analog Sensors
nxt_analog_sensor-objs := nxt_analog_sensor_core.o nxt_analog_sensor_defs.o
obj-$(CONFIG_NXT_ANALOG_SENSORS)	+= nxt_analog_sensor.o
ev3_analog_sensor-objs := ev3_analog_sensor_core.o ev3_analog_sensor_defs.o
obj-$(CONFIG_EV3_ANALOG_SENSORS)	+= ev3_analog_sensor.o

# I2C Sensors
nxt_i2c_sensor-objs := nxt_i2c_sensor_core.o nxt_i2c_sensor_defs.o ht_nxt_smux.o ms_ev3_smux.o
obj-$(CONFIG_LEGOEV3_I2C_SENSORS)	+= nxt_i2c_sensor.o
obj-$(CONFIG_LEGOEV3_I2C_SENSORS)	+= ht_nxt_smux_i2c_sensor.o

# UART Sensors
obj-$(CONFIG_EV3_UART_SENSORS)		+= legoev3_uart.o
ev3_uart_sensor-objs := ev3_uart_sensor_core.o ev3_uart_sensor_defs.o
obj-$(CONFIG_EV3_UART_SENSORS)		+= ev3_uart_sensor.o
