# Makefile for LEGO device drivers

KBUILD_CFLAGS += -I$(src)/include

obj-y	+= core/
obj-y	+= sensors/
obj-y	+= motors/
obj-y	+= wedo/

# EV3

obj-$(CONFIG_LEGOEV3_ANALOG)		+= legoev3_analog.o
obj-$(CONFIG_LEGOEV3_BLUETOOTH)		+= legoev3_bluetooth.o
obj-$(CONFIG_LEGOEV3_DEV_PORTS)		+= legoev3_ports.o
obj-$(CONFIG_LEGOEV3_DEV_PORTS)		+= ev3_input_port.o
obj-$(CONFIG_LEGOEV3_DEV_PORTS)		+= ev3_output_port.o
