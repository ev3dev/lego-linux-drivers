# Makefile for FatcatLab EVB

obj-$(CONFIG_FATCATLAB_EVB)			+= evb_battery.o
obj-$(CONFIG_FATCATLAB_EVB)			+= evb_input.o
obj-$(CONFIG_FATCATLAB_EVB)			+= evb_system_info.o
# Make module name match other sound modules
snd-evb-objs := evb_sound.o
obj-$(CONFIG_FATCATLAB_EVB)			+= snd-evb.o
