# EV3

obj-$(CONFIG_LEGOEV3_BOARD)			+= ev3_board.o
obj-$(CONFIG_LEGOEV3_I2C)			+= legoev3_i2c.o
obj-$(CONFIG_LEGOEV3_BLUETOOTH)		+= legoev3_bluetooth.o
obj-$(CONFIG_LEGOEV3_MOTORS)		+= legoev3_motor.o
obj-$(CONFIG_LEGOEV3_PORTS)			+= ev3_ports_core.o
obj-$(CONFIG_LEGOEV3_PORTS)			+= ev3_ports_in.o
obj-$(CONFIG_LEGOEV3_PORTS)			+= ev3_ports_out.o
obj-$(CONFIG_LEGOEV3_PRU)		+= ev3_pru.o
# sound driver is named slightly different so that it looks like other sound drivers
snd-legoev3-objs := legoev3_sound.o
obj-$(CONFIG_LEGOEV3_SOUND) += snd-legoev3.o
