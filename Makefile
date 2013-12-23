# Makefile for LEGO Mindstorms EV3 device drivers

obj-$(CONFIG_LEGOEV3_ANALOG)		+= legoev3_analog.o
obj-$(CONFIG_LEGOEV3_DEV_PORTS)		+= legoev3_ports.o
obj-$(CONFIG_LEGOEV3_INPUT_PORT)	+= legoev3_input_port.o
