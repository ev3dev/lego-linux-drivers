# Makefile for LEGO Mindstorms EV3 device drivers

obj-$(CONFIG_LEGOEV3_ANALOG)	+= legoev3_analog.o
obj-$(CONFIG_LEGOEV3_ADC)	+= legoev3_ads7957.o
#obj-$(CONFIG_LEGOEV3_DCM)	+= dcm.o
