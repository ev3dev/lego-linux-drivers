# Makefile for LEGO device drivers

KBUILD_CFLAGS += -I$(src)/include

obj-y	+= brickpi/
obj-y	+= core/
obj-y	+= ev3/
obj-y	+= evb/
obj-y	+= motors/
obj-y	+= pistorms/
obj-y	+= sensors/
obj-y	+= user/
obj-y	+= wedo/

obj-y	+= linux/
