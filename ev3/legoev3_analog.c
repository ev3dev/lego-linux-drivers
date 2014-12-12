/*
 * Analog framework for LEGO Mindstorms EV3
 *
 * Copyright (C) 2013-2014 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 *------------------------------------------------------------------------------
 * This framework consists of a timer loop to poll the analog/digital converter
 * and functions to access the data that is recorded.
 *
 * Acknowledgments:
 *
 * This file is based on:
 *
 * Device1 in d_analog.c of LMS2012
 * Copyright (C) 2010-2013 The LEGO Group
 *
 * A hwmon driver for the TI ADS795x/6x A/D converter chip.
 * <https://github.com/nmenon/linux-2.6-playground/blob/devel/beaglebone/base/drivers/hwmon/ads79xx.c>
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *        Nishanth Menon
 * -----------------------------------------------------------------------------
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/legoev3/legoev3_analog.h>

#include <mach/legoev3.h>

#define DRVNAME "legoev3-analog"

/*
 * The EV3 uses a TI ADS7957 A/C converter chip for reading analog voltage
 * values. <http://www.ti.com/lit/ds/symlink/alg7957.pdf>
 */

#define ADS7957_NUM_CHANNELS	16
#define ADS7957_RESOLUTION	10
#define ADS7957_VALUE_MASK	((1 << ADS7957_RESOLUTION) - 1)
#define ADS7957_REF_UV		2500000
#define ADS7957_LSB_UV		(2 * ADS7957_REF_UV / ADS7957_VALUE_MASK)
#define ADS7957_MODE_MANUAL	0x1
#define ADS7957_MODE_AUTO1	0x2
#define ADS7957_PROG_ENA	0x1
#define ADS7957_RANGE_5V	0x1

#define ADS7957_COMMAND_MANUAL(channel) \
	((ADS7957_MODE_MANUAL	<< 12) |	\
	 (ADS7957_PROG_ENA 	<< 11) |	\
	 ((channel)		<< 7)  |	\
	 (ADS7957_RANGE_5V	<< 6))

#define ADS7957_COMMAND_AUTO \
	((ADS7957_MODE_AUTO1	<< 12) |	\
	 (ADS7957_PROG_ENA	<< 11) |	\
	 (ADS7957_RANGE_5V	<< 6))

#define UPDATE_SLOW_NS		10000000		/*  10 msec */
#define UPDATE_FAST_NS		(UPDATE_SLOW_NS / 10)	/*   1 msec */
#define UPDATE_COLOR_NS		(UPDATE_FAST_NS / 5)	/* 200 usec */

enum nxt_color_read_state {
	NXT_COLOR_READ_STATE_AMBIANT,
	NXT_COLOR_READ_STATE_RED,
	NXT_COLOR_READ_STATE_GREEN,
	NXT_COLOR_READ_STATE_BLUE,
	NUM_NXT_COLOR_READ_STATE,
};

struct legoev3_analog_callback_info {
	legoev3_analog_cb_func_t function;
	void *context;
};

/**
 * struct legoev3_analog_device - TI ADS7957 analog/digital converter
 * @name: Name of this device.
 * @dev: Device registered by this driver.
 * @spi: SPI device that communicates the the ADC chip.
 * @pdata: Platform data that defines channel assignments.
 * @timer: Timer used to poll the ADC.
 * @next_update_ns: Number of nanoseconds to load into the timer for the next
 *	update.
 * @msg_busy: Used to prevent sending a second SPI message before the current
 *	one has completed.
 * @read_one_tx_buf: Transmit buffer for a "read one channel" message.
 * @read_one_rx_buf: Receive buffer for a "read one channel" message.
 * @read_one_txf: Structure that binds the transmit and receive buffers to the
 *	"read one channel" message.
 * @read_one_msg: SPI message that reads only one channel of the ADC.
 * @read_all_tx_buf: Transmit buffer for a "read all channels" message.
 * @read_all_rx_buf: Receive buffer for a "read all channels" message.
 * @read_all_txf: Structure that binds the transmit and receive buffers to the
 *	"read all channels" message.
 * @read_all_msg: SPI message that we reads all of the channels of the ADC.
 * @current_command: Stores the most recent command that was sent to the ADC.
 * @raw_data: Buffer to hold the raw (unscaled) input from the ADC for each
 *	channel.
 * @callbacks: Callback functions for each channel. Called when data is updated.
 * @callback_tasklet: Tasklet to perform callbacks for each channel.
 * @num_connected: Number of devices connected to the input and output ports.
 * @read_nxt_color: Indicates if we should be reading NXT color data for each
 *	input port.
 * @current_nxt_color_port: Indicate the currently selected port for reading NXT
 *	color data.
 * @current_nxt_color_read_state: Indicates which color we are currently reading
 *	for the current port.
 * @nxt_color_raw_data: Buffer to hold the raw data ready from NXT color sensors.
 */
struct legoev3_analog_device {
	const char *name;
	struct device dev;
	struct spi_device *spi;
	struct legoev3_analog_platform_data *pdata;
	struct hrtimer timer;
	u64 next_update_ns;
	bool msg_busy;
	u16 read_one_tx_buf;
	u16 read_one_rx_buf;
	struct spi_transfer read_one_txf;
	struct spi_message read_one_msg;
	u16 read_all_tx_buf[ADS7957_NUM_CHANNELS + 1];
	u16 read_all_rx_buf[ADS7957_NUM_CHANNELS + 1];
	struct spi_transfer read_all_txf[ADS7957_NUM_CHANNELS + 1];
	struct spi_message read_all_msg;
	u16 current_command;
	u16 raw_data[ADS7957_NUM_CHANNELS];
	struct legoev3_analog_callback_info callbacks[ADS7957_NUM_CHANNELS];
	struct tasklet_struct callback_tasklet;
	u8 num_connected;
	bool read_nxt_color[NUM_EV3_PORT_IN];
	enum ev3_input_port_id current_nxt_color_port;
	enum nxt_color_read_state current_nxt_color_read_state;
	u16 nxt_color_raw_data[NUM_EV3_PORT_IN][NUM_NXT_COLOR_READ_STATE];
};

static void legoev3_analog_read_one_msg_complete(void* context)
{
	struct legoev3_analog_device *alg = context;
	u16 val;

	if (alg->read_one_msg.status) {
		dev_err(&alg->spi->dev, "%s: spi async fail %d\n",
					__func__,
					alg->read_one_msg.status);
		hrtimer_cancel(&alg->timer);
	} else if (alg->read_nxt_color[alg->current_nxt_color_port]) {
		val = alg->read_one_rx_buf & ADS7957_VALUE_MASK;
		alg->nxt_color_raw_data[alg->current_nxt_color_port][alg->current_nxt_color_read_state] = val;
		alg->current_nxt_color_read_state++;
		if (alg->current_nxt_color_read_state >= NUM_NXT_COLOR_READ_STATE) {
			alg->current_nxt_color_read_state = NXT_COLOR_READ_STATE_AMBIANT;
			alg->current_nxt_color_port++;
			if (alg->current_nxt_color_port >= NUM_EV3_PORT_IN)
				alg->current_nxt_color_port = EV3_PORT_IN1;
		}
		if (alg->current_nxt_color_read_state) {
			/* TODO: notify color sensor to update LED */
		}
	}
	alg->msg_busy = false;
}

static void legoev3_analog_read_all_msg_complete(void* context)
{
	struct legoev3_analog_device *alg = context;
	int i = ADS7957_NUM_CHANNELS;
	bool read_color = alg->read_nxt_color[alg->current_nxt_color_port];
	u16 val, channel;

	if (alg->read_all_msg.status) {
		dev_err(&alg->spi->dev, "%s: spi async fail %d\n",
					__func__,
					alg->read_all_msg.status);
		hrtimer_cancel(&alg->timer);
	} else {
		do {
			channel = alg->read_all_rx_buf[i] >> 12;
			val = (alg->read_all_rx_buf[i] >> (12 - ADS7957_RESOLUTION))
			      & ADS7957_VALUE_MASK;
			alg->raw_data[channel] = val;
		} while (--i);
		if (read_color) {
			/* TODO: turn on first LED */
		} else {
			alg->current_nxt_color_port++;
			if (alg->current_nxt_color_port >= NUM_EV3_PORT_IN)
				alg->current_nxt_color_port = EV3_PORT_IN1;
			alg->current_nxt_color_read_state = NXT_COLOR_READ_STATE_AMBIANT;
		}
		tasklet_schedule(&alg->callback_tasklet);
	}
	alg->msg_busy = false;
}

static enum hrtimer_restart legoev3_analog_timer_callback(struct hrtimer *timer)
{
	struct legoev3_analog_device *alg =
		container_of(timer, struct legoev3_analog_device, timer);
	struct spi_device *spi = alg->spi;
	bool read_all = alg->current_command == ADS7957_COMMAND_AUTO;
	bool read_color = alg->read_nxt_color[alg->current_nxt_color_port];
	bool last_color_read_state = alg->current_nxt_color_read_state ==
						NUM_NXT_COLOR_READ_STATE - 1;
	int ret;

	if (!alg->num_connected)
		alg->next_update_ns = UPDATE_SLOW_NS;
	else if (read_color)
		alg->next_update_ns = UPDATE_COLOR_NS;
	else
		alg->next_update_ns = UPDATE_FAST_NS;
	hrtimer_forward_now(timer, ktime_set(0, alg->next_update_ns));

	if (alg->msg_busy)
		return HRTIMER_RESTART;

	alg->current_command = (read_color && !last_color_read_state)
		? ADS7957_COMMAND_MANUAL(alg->pdata->in_pin1_ch[alg->current_nxt_color_port])
		: ADS7957_COMMAND_AUTO;
	alg->msg_busy = true;
	if (read_all) {
		alg->read_all_tx_buf[ADS7957_NUM_CHANNELS - 1] = alg->current_command;
		ret = spi_async(spi, &alg->read_all_msg);
	} else {
		alg->read_one_tx_buf = alg->current_command;
		ret = spi_async(spi, &alg->read_one_msg);
	}
	if (ret < 0) {
		dev_err(&spi->dev, "%s: spi async fail %d\n",
				__func__, ret);
		alg->msg_busy = false;
		return HRTIMER_NORESTART;
	}

	return HRTIMER_RESTART;
}

u16 legoev3_analog_get_value_for_ch(struct legoev3_analog_device *alg, u8 channel)
{
	int ret = -EINVAL;
	u16 val;

	if (channel >= ADS7957_NUM_CHANNELS) {
		dev_crit(&alg->dev, "%s: channel id %d >= available channels (%d)\n",
			 __func__, channel, ADS7957_NUM_CHANNELS);
		return ret;
	}
	val = alg->raw_data[channel];
	ret = val * ADS7957_LSB_UV / 1000;
	return ret;
}

u16 legoev3_analog_in_pin1_value(struct legoev3_analog_device *alg,
				 enum ev3_input_port_id id)
{
	if (id >= NUM_EV3_PORT_IN) {
		dev_crit(&alg->dev, "%s: id %d >= available ports (%d)\n",
			 __func__, id, NUM_EV3_PORT_IN);
		return -EINVAL;
	}
	return legoev3_analog_get_value_for_ch(alg, alg->pdata->in_pin1_ch[id]);
}
EXPORT_SYMBOL_GPL(legoev3_analog_in_pin1_value);

u16 legoev3_analog_in_pin6_value(struct legoev3_analog_device *alg,
				 enum ev3_input_port_id id)
{
	if (id >= NUM_EV3_PORT_IN) {
		dev_crit(&alg->dev, "%s: id %d >= available ports (%d)\n",
			 __func__, id, NUM_EV3_PORT_IN);
		return -EINVAL;
	}
	return legoev3_analog_get_value_for_ch(alg, alg->pdata->in_pin6_ch[id]);
}
EXPORT_SYMBOL_GPL(legoev3_analog_in_pin6_value);

u16 legoev3_analog_out_pin5_value(struct legoev3_analog_device *alg,
				  enum ev3_output_port_id id)
{
	if (id >= NUM_EV3_PORT_OUT) {
		dev_crit(&alg->dev, "%s: id %d >= available ports (%d)\n",
			 __func__, id, NUM_EV3_PORT_OUT);
		return -EINVAL;
	}
	return legoev3_analog_get_value_for_ch(alg, alg->pdata->out_pin5_ch[id]);
}
EXPORT_SYMBOL_GPL(legoev3_analog_out_pin5_value);

u16 legoev3_analog_batt_volt_value(struct legoev3_analog_device *alg)
{
	return legoev3_analog_get_value_for_ch(alg, alg->pdata->batt_volt_ch);
}
EXPORT_SYMBOL_GPL(legoev3_analog_batt_volt_value);

u16 legoev3_analog_batt_curr_value(struct legoev3_analog_device *alg)
{
	return legoev3_analog_get_value_for_ch(alg, alg->pdata->batt_curr_ch);
}
EXPORT_SYMBOL_GPL(legoev3_analog_batt_curr_value);

void legoev3_analog_register_cb_for_ch(struct legoev3_analog_device *alg,
				       u8 channel,
				       legoev3_analog_cb_func_t function,
				       void *context)
{
	if (channel >= ADS7957_NUM_CHANNELS) {
		dev_crit(&alg->dev, "%s: channel id %d >= available channels (%d)\n",
			 __func__, channel, ADS7957_NUM_CHANNELS);
		return;
	}

	alg->callbacks[channel].function = function;
	alg->callbacks[channel].context = context;
}

void legoev3_analog_register_in_cb(struct legoev3_analog_device *alg,
				   enum ev3_input_port_id id,
				   legoev3_analog_cb_func_t function,
				   void *context)
{
	if (id >= NUM_EV3_PORT_IN) {
		dev_crit(&alg->dev, "%s: id %d >= available ports (%d)\n",
			 __func__, id, NUM_EV3_PORT_IN);
		return;
	}
	legoev3_analog_register_cb_for_ch(alg, alg->pdata->in_pin1_ch[id],
	                                  function, context);
}
EXPORT_SYMBOL_GPL(legoev3_analog_register_in_cb);

static ssize_t legoev3_analog_show_name(struct device *dev,
					struct device_attribute *devattr,
					char *buf)
{
	return sprintf(buf, "%s\n", to_legoev3_analog_device(dev)->name);
}

static ssize_t legoev3_analog_raw_data_read(struct file *file, struct kobject *kobj,
				     struct bin_attribute *attr,
				     char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct legoev3_analog_device *alg = to_legoev3_analog_device(dev);
	size_t size = sizeof(alg->raw_data);

	if (off >= size || !count)
		return 0;
	size -= off;
	if (count < size)
		size = count;
	memcpy(buf + off, alg->raw_data, size);

	return size;
}

static ssize_t legoev3_analog_raw_nxt_color_data_read(struct file *file,
					       struct kobject *kobj,
					       struct bin_attribute *attr,
					       char *buf, loff_t off,
					       size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct legoev3_analog_device *alg = to_legoev3_analog_device(dev);
	size_t size = sizeof(alg->nxt_color_raw_data);

	if (off >= size || !count)
		return 0;
	size -= off;
	if (count < size)
		size = count;
	memcpy(buf + off, alg->nxt_color_raw_data, size);

	return size;
}

static DEVICE_ATTR(name, S_IRUGO, legoev3_analog_show_name, NULL);

static struct bin_attribute raw_data_attr = {
	.attr = {
		.name = "raw_data",
		.mode = S_IRUGO,
	},
	.size = ADS7957_NUM_CHANNELS * 2,
	.read = legoev3_analog_raw_data_read,
};

static struct bin_attribute raw_nxt_color_data_attr = {
	.attr = {
		.name = "raw_nxt_color_data",
		.mode = S_IRUGO,
	},
	.size = NUM_EV3_PORT_IN * NUM_NXT_COLOR_READ_STATE * 2,
	.read = legoev3_analog_raw_nxt_color_data_read,
};

static struct attribute *legoev3_analog_attrs[] = {
	&dev_attr_name.attr,
	NULL
};

static struct attribute_group legoev3_analog_attr_grp = {
	.attrs = legoev3_analog_attrs,
};

static const struct attribute_group *legoev3_analog_attr_groups[] = {
	&legoev3_analog_attr_grp,
	NULL
};

static struct device_type legoev3_analog_device_type = {
	.groups = legoev3_analog_attr_groups,
};

static void legoev3_analog_device_release(struct device *dev)
{
	struct legoev3_analog_device *alg = to_legoev3_analog_device(dev);

	hrtimer_cancel(&alg->timer);
	spi_set_drvdata(alg->spi, NULL);
	alg->spi = NULL;
}

int legoev3_analog_device_register(struct device *parent,
				   struct legoev3_analog_device *alg)
{
	struct device *dev = &alg->dev;
	int err;

	device_initialize(dev);
	dev->type = &legoev3_analog_device_type;
	dev->parent = parent;
	dev->release = legoev3_analog_device_release;

	err = kobject_set_name(&dev->kobj, "%s", alg->name);
	if (err)
		goto kobject_set_name_failed;

	err = device_add(dev);
	if (err)
		goto add_device_failed;

	err = sysfs_create_bin_file(&dev->kobj, &raw_data_attr);
	if (err < 0)
		goto sysfs_create_bin_file_raw_data_attr_fail;

	err = sysfs_create_bin_file(&dev->kobj, &raw_nxt_color_data_attr);
	if (err < 0)
		goto sysfs_create_bin_file_raw_nxt_color_data_attr_fail;

	return 0;

sysfs_create_bin_file_raw_nxt_color_data_attr_fail:
	sysfs_remove_bin_file(&dev->kobj, &raw_data_attr);
sysfs_create_bin_file_raw_data_attr_fail:
kobject_set_name_failed:
add_device_failed:
	put_device(dev);

	return err;
}

void legoev3_analog_tasklet_func(unsigned long data)
{
	struct legoev3_analog_device *alg = (void *)data;
	int i;

	for (i = 0; i < ADS7957_NUM_CHANNELS; i++) {
		if (alg->callbacks[i].function)
			alg->callbacks[i].function(alg->callbacks[i].context);
	}
}

struct legoev3_analog_device legoev3_analog = {
	.name	= "legoev3-analog",
};

/**
 * get_legoev3_analog - get the legoev3-analog device
 *
 * This will return a pointer to the legoev3-analog device if it has been
 * Initialized. Otherwise, it returns -ENODEV. If get_legoev3_analog succeeds,
 * the calling code needs to release the analog device when it is no longer
 * required using the put_legoev3_analog function.
 */
struct legoev3_analog_device *get_legoev3_analog(void)
{
	if (!legoev3_analog.spi)
		return ERR_PTR(-ENODEV);
	get_device(&legoev3_analog.spi->dev);
	return &legoev3_analog;
}
EXPORT_SYMBOL_GPL(get_legoev3_analog);

void put_legoev3_analog(struct legoev3_analog_device *alg)
{
	if (alg && alg->spi)
		put_device(&alg->spi->dev);
}
EXPORT_SYMBOL_GPL(put_legoev3_analog);

static int legoev3_analog_probe(struct spi_device *spi)
{
	struct legoev3_analog_device *alg = &legoev3_analog;
	int err, i;

	/* Configure the SPI bus */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 16;
	spi_setup(spi);

	if (alg->spi) {
		dev_err(&spi->dev, "%s: Device has already been registered!\n",
			 __func__);
		return -EINVAL;
	}

	if (!spi->dev.platform_data) {
		dev_err(&spi->dev, "%s: Platform data is required!\n",
			 __func__);
		err = -EINVAL;
		goto no_platform_data;
	}
	alg->pdata = spi->dev.platform_data;

	hrtimer_init(&alg->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	alg->timer.function = legoev3_analog_timer_callback;

	spi_message_init(&alg->read_one_msg);
	alg->read_one_txf.tx_buf = &alg->read_one_tx_buf;
	alg->read_one_txf.rx_buf = &alg->read_one_rx_buf;
	alg->read_one_txf.len = 2;
	spi_message_add_tail(&alg->read_one_txf, &alg->read_one_msg);

	alg->read_one_msg.complete = legoev3_analog_read_one_msg_complete;
	alg->read_one_msg.context = alg;

	spi_message_init(&alg->read_all_msg);
	/*
	 * each word has to have an individual transfer so that the CS line can
	 * be pulsed between each transfer. Last transfer does not pulse CS.
	 */
	for (i = 0; i <= ADS7957_NUM_CHANNELS; i++) {
		alg->read_all_txf[i].tx_buf = &alg->read_all_tx_buf[i];
		alg->read_all_txf[i].rx_buf = &alg->read_all_rx_buf[i];
		alg->read_all_txf[i].len = 2;
		if (i < ADS7957_NUM_CHANNELS)
			alg->read_all_txf[i].cs_change = 1;
		spi_message_add_tail(&alg->read_all_txf[i], &alg->read_all_msg);
	}
	alg->read_all_msg.complete = legoev3_analog_read_all_msg_complete;
	alg->read_all_msg.context = alg;

	tasklet_init(&alg->callback_tasklet, legoev3_analog_tasklet_func,
		     (unsigned long)alg);

	err = legoev3_analog_device_register(&spi->dev, alg);
	if (err < 0)
		goto legoev3_analog_device_register_fail;

	spi_set_drvdata(spi, alg);
	alg->spi = spi;
	hrtimer_start(&alg->timer, ktime_set(0, UPDATE_SLOW_NS), HRTIMER_MODE_REL);

	return 0;

legoev3_analog_device_register_fail:
no_platform_data:
	devm_kfree(&spi->dev, alg);
	return err;
}

static int legoev3_analog_remove(struct spi_device *spi)
{
	struct legoev3_analog_device *alg = spi_get_drvdata(spi);

	hrtimer_cancel(&alg->timer);
	tasklet_kill(&alg->callback_tasklet);
	device_unregister(&alg->dev);
	spi_set_drvdata(alg->spi, NULL);

	return 0;
}

struct spi_driver legoev3_analog_driver = {
	.driver = {
		.name = DRVNAME,
		.owner = THIS_MODULE,
	},
	.probe = legoev3_analog_probe,
	.remove = legoev3_analog_remove,
};
EXPORT_SYMBOL_GPL(legoev3_analog_driver);
module_spi_driver(legoev3_analog_driver);

MODULE_DESCRIPTION("Analog driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
