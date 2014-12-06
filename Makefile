# Makefile for LEGO device drivers

KBUILD_CFLAGS += -I$(src)/include

obj-y	+= core/
obj-y	+= sensors/
obj-y	+= wedo/

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
