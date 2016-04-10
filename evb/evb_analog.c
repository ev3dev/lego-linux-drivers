/*
 * Analog framework for FatcatLab EVB
 *
 * Copyright (C) 2013-2014,2016 David Lechner <david@lechnology.com>
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

#include <linux/device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include "evb_analog.h"

#define DRVNAME "evb-analog"

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

/**
 * struct evb_analog_channel_data - Per channel data.
 *
 * @callback: Callback function. Called when data is updated.
 * @context: Passed as argument to callback.
 */
struct evb_analog_channel_data {
	evb_analog_cb_func_t	callback;
	void			*context;
};

/**
 * struct evb_analog_data - TI ADS7957 analog/digital converter
 * @name: Name of this device.
 * @spi: SPI device that communicates the the ADC chip.
 * @timer: Timer used to poll the ADC.
 * @callback_tasklet: Tasklet to perform callbacks for each channel.
 * @tx_buf: Transmit buffer.
 * @rx_buf: Receive buffer.
 * @transfer: SPI transfer.
 * @msg: SPI message that reads only one channel of the ADC.
 * @ch_data: Channel specific data for each channel.
 * @raw_data: Buffer to hold the raw (unscaled) input from the ADC for each
 *	channel.
 * @tx_channel: The currently active transmit channel or CHANNEL_NONE.
 * @rx_channel: The currently active receive channel or CHANNEL_NONE.
 */
struct evb_analog_data {
	const char			*name;
	struct spi_device		*spi;
	struct				hrtimer timer;
	struct tasklet_struct		callback_tasklet;
	u16				tx_buf[1];
	u16				rx_buf[1];
	struct spi_transfer		transfer;
	struct spi_message		msg;
	struct evb_analog_channel_data	ch_data[ADS7957_NUM_CHANNELS];
	u16				raw_data[ADS7957_NUM_CHANNELS];
	unsigned			enabled_channels;
#define ENABLED_CHANNEL_MASK ((1 << ADS7957_NUM_CHANNELS) - 1)
	u8				next_channel;
	bool				msg_busy;
#define CHANNEL_NONE (-1)
};

static inline void evb_analog_next_channel(struct evb_analog_data *data)
{
	if (unlikely(!(data->enabled_channels & ENABLED_CHANNEL_MASK)))
		return;

	do {
		data->next_channel++;
		if (data->next_channel >= ADS7957_NUM_CHANNELS)
			data->next_channel = 0;
	} while (!(BIT(data->next_channel) & data->enabled_channels));
}

static void evb_analog_tasklet_func(unsigned long _data)
{
	struct evb_analog_data *data = (void *)_data;
	struct evb_analog_channel_data *ch_data;
	u16 val, channel;
printk("%s\n", __func__);
	/* The data received is for the channel sent two messages ago */
	channel = data->rx_buf[0] >> 12;
	if (channel >= ADS7957_NUM_CHANNELS)
		return;

	val = (data->rx_buf[0] >> (12 - ADS7957_RESOLUTION))
	      & ADS7957_VALUE_MASK;
	data->raw_data[channel] = val;
printk("received ch %u: %u\n", channel, val);
	ch_data = &data->ch_data[channel];
	if (ch_data->callback)
		ch_data->callback(ch_data->context);

	data->msg_busy = false;
}

static void evb_analog_msg_complete(void *context)
{
	struct evb_analog_data *data = context;

	if (data->msg.status) {
		dev_err(&data->spi->dev, "%s: spi async fail %d\n", __func__,
			data->msg.status);
		hrtimer_cancel(&data->timer);

		return;
	}

	tasklet_schedule(&data->callback_tasklet);
}

static enum hrtimer_restart evb_analog_timer_callback(struct hrtimer *timer)
{
	struct evb_analog_data *data =
		container_of(timer, struct evb_analog_data, timer);
	struct spi_device *spi = data->spi;
	int ret;

	hrtimer_forward_now(timer, ktime_set(1, 10000000)); /* 10 ms */

	/* If we are still processing the data from last time... */
	if (data->msg_busy)
		return HRTIMER_RESTART;

	printk("requesting ch %u\n", data->next_channel);
	data->tx_buf[0] = ADS7957_COMMAND_MANUAL(data->next_channel);
	ret = spi_async(spi, &data->msg);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: spi async fail %d\n", __func__, ret);

		return HRTIMER_NORESTART;
	}

	data->msg_busy = true;
	evb_analog_next_channel(data);

	return HRTIMER_RESTART;
}

int evb_analog_get_value_for_ch(struct evb_analog_data *data, u8 ch, u16 *val)
{
	int new_val;

	if (ch >= ADS7957_NUM_CHANNELS) {
		dev_crit(&data->spi->dev,
			 "%s: channel id %d >= available channels (%d)\n",
			 __func__, ch, ADS7957_NUM_CHANNELS);
		return -EINVAL;
	}
	new_val = data->raw_data[ch];
	*val = new_val * ADS7957_LSB_UV / 1000;

	return 0;
}
EXPORT_SYMBOL_GPL(evb_analog_get_value_for_ch);

void evb_analog_register_cb_for_ch(struct evb_analog_data *data,
				   u8 channel,
				   evb_analog_cb_func_t function,
				   void *context)
{
	if (channel >= ADS7957_NUM_CHANNELS) {
		dev_crit(&data->spi->dev,
			 "%s: channel id %d >= available channels (%d)\n",
			 __func__, channel, ADS7957_NUM_CHANNELS);
		return;
	}

	data->ch_data[channel].callback = function;
	data->ch_data[channel].context = context;
}
EXPORT_SYMBOL_GPL(evb_analog_register_cb_for_ch);

static void evb_analog_parse_dt(struct evb_analog_data *data)
{
	struct device_node *node = data->spi->dev.of_node;
	struct property *prop;
	const __be32 *p;
	u32 value;

	if (!node)
		return;

	of_property_for_each_u32(node, "ev3dev,channels", prop, p, value) {
		if (value >= ADS7957_NUM_CHANNELS) {
			dev_warn(&data->spi->dev, "Channel out of range: %u\n",
				 value);
			continue;
		}
		data->enabled_channels |= BIT(value);
	}
}

static int evb_analog_probe(struct spi_device *spi)
{
	struct evb_analog_data *data;

	data = devm_kzalloc(&spi->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spi_set_drvdata(spi, data);
	data->spi = spi;
	evb_analog_parse_dt(data);

	/* If it wasn't set in device tree */
	if (!data->enabled_channels)
		data->enabled_channels = ENABLED_CHANNEL_MASK;
	/* move to first enabled channel */
	data->next_channel = ADS7957_NUM_CHANNELS - 1;
	evb_analog_next_channel(data);

	spi_message_init(&data->msg);
	data->transfer.tx_buf = &data->tx_buf;
	data->transfer.rx_buf = &data->rx_buf;
	data->transfer.bits_per_word = 16;
	data->transfer.len = 2;
	spi_message_add_tail(&data->transfer, &data->msg);

	tasklet_init(&data->callback_tasklet, evb_analog_tasklet_func,
		     (unsigned long)data);

	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = evb_analog_timer_callback;

	/*
	 * Run two messages so that we actually have valid rx data in the
	 * first message initiated by the timer callback.
	 */
	data->tx_buf[0] = ADS7957_COMMAND_MANUAL(data->next_channel);
	spi_sync(spi, &data->msg);
	evb_analog_next_channel(data);
	data->tx_buf[0] = ADS7957_COMMAND_MANUAL(data->next_channel);
	spi_sync(spi, &data->msg);
	evb_analog_next_channel(data);

	data->msg.complete = evb_analog_msg_complete;
	data->msg.context = data;

	hrtimer_start(&data->timer, ktime_set(0, 0), HRTIMER_MODE_REL);

	return 0;
}

static int evb_analog_remove(struct spi_device *spi)
{
	struct evb_analog_data *data = spi_get_drvdata(spi);

	hrtimer_cancel(&data->timer);
	tasklet_kill(&data->callback_tasklet);
	spi_set_drvdata(data->spi, NULL);

	return 0;
}

static const struct of_device_id evb_analog_dt_ids[] = {
	{ .compatible = "ev3dev,evb-analog", },
	{ }
};
MODULE_DEVICE_TABLE(of, evb_analog_dt_ids);

struct spi_driver evb_analog_driver = {
	.driver = {
		.name		= DRVNAME,
		.owner		= THIS_MODULE,
		.of_match_table	= evb_analog_dt_ids,
	},
	.probe	= evb_analog_probe,
	.remove	= evb_analog_remove,
};
module_spi_driver(evb_analog_driver);

MODULE_DESCRIPTION("Analog driver for FatcatLab EVB");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:evb-analog");
