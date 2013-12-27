# Makefile for LEGO Mindstorms EV3 device drivers

obj-$(CONFIG_LEGOEV3_ANALOG)		+= legoev3_analog.o
obj-$(CONFIG_LEGOEV3_DEV_PORTS)		+= legoev3_ports.o
obj-$(CONFIG_LEGOEV3_INPUT_PORT)	+= legoev3_input_port.o
obj-$(CONFIG_LEGOEV3_TOUCH_SENSORS)	+= legoev3_touch_sensor_class.o
obj-$(CONFIG_LEGOEV3_EV3_TOUCH_SENSOR)	+= legoev3_ev3_touch_sensor.o
obj-$(CONFIG_LEGOEV3_NXT_TOUCH_SENSOR)	+= legoev3_nxt_touch_sensor.o
