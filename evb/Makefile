# Makefile for FatcatLab EVB

# evb_analog currently not used in favor of ti-ads97xx
obj-$(CONFIG_FATCATLAB_EVB)			+= evb_battery.o
obj-$(CONFIG_FATCATLAB_EVB)			+= evb_input.o
obj-$(CONFIG_FATCATLAB_EVB)			+= evb_pru_i2c.o
# Make module name match other sound modules
snd-evb-objs := evb_sound.o
obj-$(CONFIG_FATCATLAB_EVB)			+= snd-evb.o
