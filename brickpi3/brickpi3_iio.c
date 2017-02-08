/*
 * Dexter Industries BrickPi3 IIO driver
 *
 * Copyright (C) 2017 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/iio/iio.h>

#include "brickpi3.h"

struct brickpi3_iio {
	struct iio_dev *iio;
	struct brickpi3 *bp;
};

static int brickpi3_iio_read_raw(struct iio_dev *iio,
				 struct iio_chan_spec const *chan,
				 int *val, int *val2, long mask)
{
	struct brickpi3_iio *data = iio_priv(iio);
	u16 value;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		ret = brickpi3_read_u16(data->bp, chan->address, &value);
		if (ret < 0)
			return ret;

		*val = value;

		return IIO_VAL_INT;
	}

	return -EINVAL;
}

#define BRICKPI3_V_CHAN(index, name)				\
{								\
	.type = IIO_VOLTAGE,					\
	.channel = index,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),	\
	.address = BRICKPI3_MSG_READ_VOLTAGE_##name,		\
	.datasheet_name = __stringify(name),			\
	.extend_name = __stringify(name),			\
	.scan_index = index,					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_CPU,				\
	},							\
}

static const struct iio_chan_spec brickpi3_iio_channels[] = {
	BRICKPI3_V_CHAN(0, 3V3),
	BRICKPI3_V_CHAN(1, 5V),
	BRICKPI3_V_CHAN(2, 9V),
	BRICKPI3_V_CHAN(3, VCC),
	IIO_CHAN_SOFT_TIMESTAMP(4),
};

static const struct iio_info brickpi3_iio_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &brickpi3_iio_read_raw,
};

int devm_brickpi3_register_iio(struct device *dev, struct brickpi3 *bp)
{
	struct brickpi3_iio *data;
	struct iio_dev *iio;
	int ret;

	iio = devm_iio_device_alloc(dev, sizeof(*data));
	if (!iio)
		return -ENOMEM;

	data = iio_priv(iio);
	data->iio = iio;
	data->bp = bp;

	iio->name = "BrickPi3";
	iio->dev.parent = dev;
	iio->modes = INDIO_DIRECT_MODE;
	iio->channels = brickpi3_iio_channels;
	iio->num_channels = ARRAY_SIZE(brickpi3_iio_channels);
	iio->info = &brickpi3_iio_info;

	ret = devm_iio_device_register(dev, iio);
	if (ret) {
		dev_err(dev, "Failed to register iio device.\n");
		return ret;
	}

	return 0;
}
