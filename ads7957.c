/*
 * Support for TI ADS7957 A/D converter on LEGO Mindstorms EV3 platform
 *
 * Copyright (C) 2013 David Lechner <david@lechnology.com>
 *
 * This file began its life as a hwmon driver...
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DRVNAME "legoev3-ads7957"
#define pr_fmt(fmt) DRVNAME ": " fmt

#define ADS7957_NUM_CHANNELS	16
#define ADS7957_RESOLUTION	12
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/legoev3/ads7957.h>

#include <mach/legoev3.h>

enum nxt_color_read_state {
	NXT_COLOR_READ_STATE_AMBIANT,
	NXT_COLOR_READ_STATE_RED,
	NXT_COLOR_READ_STATE_GREEN,
	NXT_COLOR_READ_STATE_BLUE,
	NUM_NXT_COLOR_READ_STATE,
};

/**
 * struct ads7957_device - TI ADS7957 analog/digital converter
 * @spi: SPI device that communicates the the ADC chip.
 * @dcm: Device connection manager platform device.
 * @pdata: Platform data that defines channel assignments.
 * @timer: Timer used to poll the ADC.
 * @lock: Spin lock.
 * @next_update_ns: Number of nanoseconds to load into the timer for the next
 *	update.
 * @msg_busy: Used to prevent sending a second SPI message before the current
 *	one has completed.
 * @read_one_tx_buf: Transmit buffer for a "read one channel" message.
 * @read_one_rx_buf: Receive buffer for a "read one channel" message.
 * @read_one_txf: Structure that binds the transmit and receive buffers to the
 *	"read one channel" message.
 * @read_one_msg: SPI message that reads only only channel of the ADC.
 * @read_all_tx_buf: Transmit buffer for a "read all channels" message.
 * @read_all_rx_buf: Receive buffer for a "read all channels" message.
 * @read_all_txf: Structure that binds the transmit and receive buffers to the
 *	"read all channels" message.
 * @read_all_msg: SPI message that we reads all of the channels of the ADC.
 * @current_command: Stores the most recent command that was sent to the ADC.
 * @raw_data: Buffer to hold the raw (unscaled) input from the ADC for each
 *	channel.
 * @num_connected: Number of devices connected to the input and output ports.
 * @read_nxt_color: Indicates if we should be reading NXT color data for each
 *	input port.
 * @current_nxt_color_port: Indicate the currently selected port for reading NXT
 *	color data.
 * @current_nxt_color_read_state: Indicates which color we are currently reading
 *	for the current port.
 * @nxt_color_raw_data: Buffer to hold the raw data ready from NXT color sensors.
 */
struct ads7957_device {
	struct spi_device *spi;
	struct platform_device *pdev;
	struct ads7957_platform_data *pdata;
	struct hrtimer timer;
	spinlock_t lock;
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
	u8 num_connected;
	bool read_nxt_color[LEGOEV3_NUM_PORT_IN];
	enum legoev3_input_port current_nxt_color_port;
	enum nxt_color_read_state current_nxt_color_read_state;
	u16 nxt_color_raw_data[LEGOEV3_NUM_PORT_IN][NUM_NXT_COLOR_READ_STATE];
};

static void ads7957_read_one_msg_complete(void* context)
{
	struct ads7957_device *ads = context;
	u16 val;

	if (ads->read_one_msg.status) {
		dev_err(&ads->spi->dev, "%s: spi async fail %d\n",
					__func__,
					ads->read_one_msg.status);
		hrtimer_cancel(&ads->timer);
	} else if (ads->read_nxt_color[ads->current_nxt_color_port]) {
		val = ads->read_one_rx_buf & ADS7957_VALUE_MASK;
		ads->nxt_color_raw_data[ads->current_nxt_color_port][ads->current_nxt_color_read_state] = val;
		ads->current_nxt_color_read_state++;
		if (ads->current_nxt_color_read_state >= NUM_NXT_COLOR_READ_STATE) {
			ads->current_nxt_color_read_state = NXT_COLOR_READ_STATE_AMBIANT;
			ads->current_nxt_color_port++;
			if (ads->current_nxt_color_port >= LEGOEV3_NUM_PORT_IN)
				ads->current_nxt_color_port = LEGOEV3_PORT_IN1;
		}
		if (ads->current_nxt_color_read_state) {
			/* TODO: notify color sensor to update LED */
		}
	}
	ads->msg_busy = false;
}

static void ads7957_read_all_msg_complete(void* context)
{
	struct ads7957_device *ads = context;
	int i = ADS7957_NUM_CHANNELS;
	bool read_color = ads->read_nxt_color[ads->current_nxt_color_port];
	u16 val, channel;

	if (ads->read_all_msg.status) {
		dev_err(&ads->spi->dev, "%s: spi async fail %d\n",
					__func__,
					ads->read_all_msg.status);
		hrtimer_cancel(&ads->timer);
	} else {
		do {
			channel = ads->read_all_rx_buf[i] >> 12;
			val = ads->read_all_rx_buf[i] & ADS7957_VALUE_MASK;
			ads->raw_data[channel] = val;
		} while (--i);
		if (read_color) {
			/* TODO: turn on first LED */
		} else {
			ads->current_nxt_color_port++;
			if (ads->current_nxt_color_port >= LEGOEV3_NUM_PORT_IN)
				ads->current_nxt_color_port = LEGOEV3_PORT_IN1;
			ads->current_nxt_color_read_state = NXT_COLOR_READ_STATE_AMBIANT;
		}
	}
	ads->msg_busy = false;
}

static enum hrtimer_restart ads7957_timer_callback(struct hrtimer *pTimer)
{
	struct ads7957_device *ads = container_of(pTimer, struct ads7957_device,
						  timer);
	struct spi_device *spi = ads->spi;
	bool read_all = ads->current_command == ADS7957_COMMAND_AUTO;
	bool read_color = ads->read_nxt_color[ads->current_nxt_color_port];
	bool last_color_read_state = ads->current_nxt_color_read_state ==
						NUM_NXT_COLOR_READ_STATE - 1;
	int ret;

	if (!ads->num_connected)
		ads->next_update_ns = UPDATE_SLOW_NS;
	else if (read_color)
		ads->next_update_ns = UPDATE_COLOR_NS;
	else
		ads->next_update_ns = UPDATE_FAST_NS;
	hrtimer_forward_now(pTimer, ktime_set(0, ads->next_update_ns));

	if (ads->msg_busy)
		return HRTIMER_RESTART;

	ads->current_command = (read_color && !last_color_read_state)
		? ADS7957_COMMAND_MANUAL(ads->pdata->in_pin1_ch[ads->current_nxt_color_port])
		: ADS7957_COMMAND_AUTO;
	ads->msg_busy = true;
	if (read_all) {
		ads->read_all_tx_buf[ADS7957_NUM_CHANNELS - 1] = ads->current_command;
		ret = spi_async(spi, &ads->read_all_msg);
	} else {
		ads->read_one_tx_buf = ads->current_command;
		ret = spi_async(spi, &ads->read_one_msg);
	}
	if (ret < 0) {
		dev_err(&spi->dev, "%s: spi async fail %d\n",
				__func__, ret);
		return HRTIMER_NORESTART;
	}

	return HRTIMER_RESTART;
}

u16 ads7957_get_data_for_ch(struct ads7957_device *ads, u8 channel)
{
	struct spi_device *spi = ads->spi;
	int ret = -EINVAL;
	u16 val;
	unsigned long flags;

	if (channel > ADS7957_NUM_CHANNELS) {
		dev_crit(&spi->dev, "%s: channel id %d >= availible channels (%d)\n",
			 __func__, channel, ADS7957_NUM_CHANNELS);
		return ret;
	}
	spin_lock_irqsave(&ads->lock, flags);
	val = ads->raw_data[channel];
	spin_unlock_irqrestore(&ads->lock, flags);
	ret = val * ADS7957_LSB_UV / 1000;
	return ret;
}
EXPORT_SYMBOL_GPL(ads7957_get_data_for_ch);

static ssize_t ads7957_show_name(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%s\n", to_spi_device(dev)->modalias);
}

static ssize_t ads7957_raw_data_read(struct file *file, struct kobject *kobj,
				     struct bin_attribute *attr,
				     char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct platform_device *pdev = to_platform_device(dev);
	struct ads7957_device *ads = platform_get_drvdata(pdev);
	size_t size = sizeof(ads->raw_data);

	if (off >= size || !count)
		return 0;
	size -= off;
	if (count < size)
		size = count;
	memcpy(buf + off, ads->raw_data, size);

	return size;
}

static ssize_t ads7957_raw_nxt_color_data_read(struct file *file,
					       struct kobject *kobj,
					       struct bin_attribute *attr,
					       char *buf, loff_t off,
					       size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct platform_device *pdev = to_platform_device(dev);
	struct ads7957_device *ads = platform_get_drvdata(pdev);
	size_t size = sizeof(ads->nxt_color_raw_data);

	if (off >= size || !count)
		return 0;
	size -= off;
	if (count < size)
		size = count;
	memcpy(buf + off, ads->nxt_color_raw_data, size);

	return size;
}

static DEVICE_ATTR(name, S_IRUGO, ads7957_show_name, NULL);

static struct bin_attribute raw_data_attr = {
	.attr = {
		.name = "raw_data",
		.mode = S_IRUGO,
	},
	.size = ADS7957_NUM_CHANNELS * 2,
	.read = ads7957_raw_data_read,
};

static struct bin_attribute raw_nxt_color_data_attr = {
	.attr = {
		.name = "raw_nxt_color_data",
		.mode = S_IRUGO,
	},
	.size = LEGOEV3_NUM_PORT_IN * NUM_NXT_COLOR_READ_STATE * 2,
	.read = ads7957_raw_nxt_color_data_read,
};

static struct attribute *ads7957_attrs[] = {
	&dev_attr_name.attr,
	NULL
};

static struct attribute_group ads7957_attr_grp = {
	.attrs = ads7957_attrs,
};

static int __devinit ads7957_probe(struct spi_device *spi)
{
	int err, i;
	struct ads7957_device *ads;

	/* Configure the SPI bus */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 16;
	spi_setup(spi);

	ads = devm_kzalloc(&spi->dev, sizeof(struct ads7957_device),
								GFP_KERNEL);
	if (!ads)
		return -ENOMEM;

	if (!spi->dev.platform_data) {
		dev_err(&spi->dev, "%s: Platform data is required!\n",
			 __func__);
		err = -EINVAL;
		goto err1;
	}
	ads->pdata = spi->dev.platform_data;

	ads->spi = spi;
	hrtimer_init(&ads->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ads->timer.function = ads7957_timer_callback;

	spi_message_init(&ads->read_one_msg);
	ads->read_one_txf.tx_buf = &ads->read_one_tx_buf;
	ads->read_one_txf.rx_buf = &ads->read_one_rx_buf;
	ads->read_one_txf.len = 2;
	spi_message_add_tail(&ads->read_one_txf, &ads->read_one_msg);

	ads->read_one_msg.complete = ads7957_read_one_msg_complete;
	ads->read_one_msg.context = ads;

	spi_message_init(&ads->read_all_msg);
	/*
	 * each word has to have an individual transfer so that the CS line can
	 * be pulsed between each transfer. Last transfer does not pulse CS.
	 */
	for (i = 0; i <= ADS7957_NUM_CHANNELS; i++) {
		ads->read_all_txf[i].tx_buf = &ads->read_all_tx_buf[i];
		ads->read_all_txf[i].rx_buf = &ads->read_all_rx_buf[i];
		ads->read_all_txf[i].len = 2;
		if (i < ADS7957_NUM_CHANNELS)
			ads->read_all_txf[i].cs_change = 1;
		spi_message_add_tail(&ads->read_all_txf[i], &ads->read_all_msg);
	}
	ads->read_all_msg.complete = ads7957_read_all_msg_complete;
	ads->read_all_msg.context = ads;

	ads->pdev = platform_device_register_simple("legoev3-analog", -1, NULL, 0);
	if (IS_ERR(ads->pdev)) {
		err = PTR_ERR(ads->pdev);
		goto err2;
	}
	platform_set_drvdata(ads->pdev, ads);

	err = sysfs_create_group(&ads->pdev->dev.kobj, &ads7957_attr_grp);
	if (err < 0)
		goto err3;
	err = sysfs_create_bin_file(&ads->pdev->dev.kobj, &raw_data_attr);
	if (err < 0)
		goto err4;
	err = sysfs_create_bin_file(&ads->pdev->dev.kobj, &raw_nxt_color_data_attr);
	if (err < 0)
		goto err5;

	spi_set_drvdata(spi, ads);
	hrtimer_start(&ads->timer, ktime_set(0, UPDATE_SLOW_NS), HRTIMER_MODE_REL);

	return 0;

err5:
	sysfs_remove_bin_file(&spi->dev.kobj, &raw_data_attr);
err4:
	sysfs_remove_group(&ads->pdev->dev.kobj, &ads7957_attr_grp);
err3:
	platform_set_drvdata(ads->pdev, NULL);
err2:
	platform_device_unregister(ads->pdev);
err1:
	devm_kfree(&spi->dev, ads);
	return err;
}

static int __devexit ads7957_remove(struct spi_device *spi)
{
	struct ads7957_device *ads = spi_get_drvdata(spi);

	hrtimer_cancel(&ads->timer);
	sysfs_remove_group(&ads->pdev->dev.kobj, &ads7957_attr_grp);
	sysfs_remove_bin_file(&ads->pdev->dev.kobj, &raw_data_attr);
	platform_device_unregister(ads->pdev);
	platform_set_drvdata(ads->pdev, NULL);
	devm_kfree(&spi->dev, ads);
	spi_set_drvdata(spi, NULL);

	return 0;
}

static struct spi_driver ads7957_driver = {
	.driver = {
		.name = DRVNAME,
		.owner = THIS_MODULE,
	},
	.probe = ads7957_probe,
	.remove = __devexit_p(ads7957_remove),
};
module_spi_driver(ads7957_driver);

MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_DESCRIPTION("TI ADS7957 A/D driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRVNAME);
