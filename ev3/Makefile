# EV3

obj-$(CONFIG_LEGOEV3_ANALOG)		+= legoev3_analog.o
obj-$(CONFIG_LEGOEV3_BATTERY)		+= legoev3_battery.o
obj-$(CONFIG_LEGOEV3_BLUETOOTH)		+= legoev3_bluetooth.o
legoev3_motor-objs := legoev3_motor_core.o legoev3_motor_defs.o
obj-$(CONFIG_LEGOEV3_MOTORS)		+= legoev3_motor.o
legoev3_ports-objs := legoev3_ports_core.o legoev3_ports_in.o legoev3_ports_out.o
obj-$(CONFIG_LEGOEV3_DEV_PORTS)		+= legoev3_ports.o
# sound driver is named slightly different so that it looks like other sound drivers
snd-legoev3-objs := legoev3_sound.o
obj-$(CONFIG_LEGOEV3_SOUND) += snd-legoev3.o
