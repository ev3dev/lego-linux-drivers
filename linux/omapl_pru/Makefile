
EXTRA_CFLAGS += -I$(srctree)/drivers/lego/linux/omapl_pru/pru/hal/uart/include -I$(srctree)/drivers/lego/linux/omapl_pru/pru/hal/common/include

suart_emu-objs := 	ti_omapl_pru_suart.o \
					pru/hal/uart/src/suart_api.o \
					pru/hal/uart/src/suart_utils.o \
					pru/hal/common/src/pru.o

obj-$(CONFIG_SERIAL_SUART_OMAPL_PRU)	+= suart_emu.o

ccflags-$(CONFIG_SUART_DEBUG_DEVICES) := -DDEBUG

