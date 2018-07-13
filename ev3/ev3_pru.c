// SPDX-License-Identifier: GPL-2.0
/*
 * PRU Driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2018 David Lechner <david@lechnology.com>
 */

#include <linux/dma-mapping.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/types.h>

/* from PRU */

#define EV3_PRU_TACHO_RING_BUF_SIZE 1024 /* must be power of 2! */

enum ev3_pru_tacho_msg_type {
	/* Host >< PRU: request memory map address of ring buffer data */
	EV3_PRU_TACHO_MSG_DATA_ADDR,
};

struct ev3_pru_tacho_msg {
	uint32_t type;
	uint32_t value;
};

enum ev3_pru_tacho {
	EV3_PRU_TACHO_A,
	EV3_PRU_TACHO_B,
	EV3_PRU_TACHO_C,
	EV3_PRU_TACHO_D,
	NUM_EV3_PRU_TACHO
};

struct ev3_pru_tacho_remote_data {
	uint32_t position[NUM_EV3_PRU_TACHO][EV3_PRU_TACHO_RING_BUF_SIZE];
	uint32_t timestamp[NUM_EV3_PRU_TACHO][EV3_PRU_TACHO_RING_BUF_SIZE];
	uint32_t head[NUM_EV3_PRU_TACHO];
};

/* end from PRU */

enum ev3_pru_tacho_scan_indexl {
	EV3_PRU_SCAN_INDEX_TACHO_A_COUNT,
	EV3_PRU_SCAN_INDEX_TACHO_B_COUNT,
	EV3_PRU_SCAN_INDEX_TACHO_C_COUNT,
	EV3_PRU_SCAN_INDEX_TACHO_D_COUNT,
	EV3_PRU_SCAN_INDEX_TACHO_A_COUNT_TIME,
	EV3_PRU_SCAN_INDEX_TACHO_B_COUNT_TIME,
	EV3_PRU_SCAN_INDEX_TACHO_C_COUNT_TIME,
	EV3_PRU_SCAN_INDEX_TACHO_D_COUNT_TIME,
	NUM_EV3_PRU_SCAN_INDEX
};

struct ev3_tacho_rpmsg_data {
	__iomem void *remote_data;
	struct iio_dev *indio_dev;
	struct rpmsg_device *rpdev;
	struct completion completion;
	u32 last_value;
};

static int ev3_tacho_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			      void *p, u32 src)
{
	struct ev3_tacho_rpmsg_data *priv = dev_get_drvdata(&rpdev->dev);
	struct ev3_pru_tacho_msg *msg = data;

	if (len < sizeof(*msg))
		return -EINVAL;

	switch (msg->type) {
	case EV3_PRU_TACHO_MSG_DATA_ADDR:
		priv->last_value = msg->value;
		complete(&priv->completion);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ev3_pru_tacho_get_freq(struct ev3_pru_tacho_remote_data *data,
				  enum ev3_pru_tacho index)
{
	s32 head = data->head[index];
	s32 tail;
	s32 head_pos, tail_pos;
	u32 head_time, tail_time;
	u32 x = 0;

	head_pos = data->position[index][head];
	head_time = data->timestamp[index][head];

	do {
		/*
		 * tacho counts are not equidistant, so we need to use multiple
		 * of 4 to get accurate values
		 */
		x += 4;

		tail = (head - x) & (EV3_PRU_TACHO_RING_BUF_SIZE - 1);

		tail_pos = data->position[index][tail];
		tail_time = data->timestamp[index][tail];

		if (head_pos == tail_pos)
			return 0;

		/*
		 * we need delta_t to be >= 20ms to be reasonably accurate.
		 * timer is 24MHz, thus * 3 / 125 converts to ns
		 */
		if (head_time - tail_time >= 20 * NSEC_PER_MSEC * 3 / 125)
			break;
	} while (x < 64);

	/* avoid divide by 0 - motor probably hasn't moved yet */
	if (head_time == tail_time)
		return 0;

	/* timer is 24MHz */
	return div_s64((s64)(head_pos - tail_pos) * 24000000,
			head_time - tail_time);
	
}

static int ev3_pru_tacho_read_raw(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  int *val, int *val2, long mask)
{
	struct ev3_tacho_rpmsg_data *priv = iio_priv(indio_dev);
	struct ev3_pru_tacho_remote_data *data = priv->remote_data;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->scan_index) {
		case EV3_PRU_SCAN_INDEX_TACHO_A_COUNT:
		case EV3_PRU_SCAN_INDEX_TACHO_B_COUNT:
		case EV3_PRU_SCAN_INDEX_TACHO_C_COUNT:
		case EV3_PRU_SCAN_INDEX_TACHO_D_COUNT:
			*val = data->position[chan->channel][data->head[chan->channel]];
			return IIO_VAL_INT;
		case EV3_PRU_SCAN_INDEX_TACHO_A_COUNT_TIME:
		case EV3_PRU_SCAN_INDEX_TACHO_B_COUNT_TIME:
		case EV3_PRU_SCAN_INDEX_TACHO_C_COUNT_TIME:
		case EV3_PRU_SCAN_INDEX_TACHO_D_COUNT_TIME:
			*val = data->timestamp[chan->channel][data->head[chan->channel]];
			return IIO_VAL_INT;
		}
		break;
	case IIO_CHAN_INFO_PROCESSED:
		switch (chan->type) {
		case IIO_FREQUENCY:
			*val = ev3_pru_tacho_get_freq(data, chan->channel);
			return IIO_VAL_INT;
		default:
			break;
		}
		break;
	}

	return -EINVAL;
}

#define EV3_PRU_IIO_MOTOR(n) {						\
		.type = IIO_COUNT,					\
		.indexed = 1,						\
		.channel = EV3_PRU_TACHO_##n,				\
		.scan_index = EV3_PRU_SCAN_INDEX_TACHO_##n##_COUNT,	\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 32,					\
			.storagebits = 32,				\
		},							\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.datasheet_name = "Motor##n",				\
	},								\
	{								\
		.type = IIO_COUNT,					\
		.indexed = 1,						\
		.channel = EV3_PRU_TACHO_##n,				\
		.scan_index = EV3_PRU_SCAN_INDEX_TACHO_##n##_COUNT_TIME,\
		.scan_type = {						\
			.sign = 'u',					\
			.realbits = 32,					\
			.storagebits = 32,				\
		},							\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.datasheet_name = "Motor##n",				\
		.extend_name = "time",					\
	},								\
	{								\
		.type = IIO_FREQUENCY,					\
		.indexed = 1,						\
		.channel = EV3_PRU_TACHO_##n,				\
		.scan_index = -1,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),	\
		.datasheet_name = "Motor##n",				\
	}

static const struct iio_chan_spec ev3_tacho_rpmsg_channels[] = {
	EV3_PRU_IIO_MOTOR(A),
	EV3_PRU_IIO_MOTOR(B),
	EV3_PRU_IIO_MOTOR(C),
	EV3_PRU_IIO_MOTOR(D),
};

static const struct iio_info ev3_pru_tacho_iio_info = {
	.driver_module = THIS_MODULE,
	.read_raw = ev3_pru_tacho_read_raw,
};

static int ev3_pru_tacho_buffer_enable(struct iio_dev *indio_dev)
{
	return -EOPNOTSUPP;
}

static int ev3_pru_tacho_buffer_disable(struct iio_dev *indio_dev)
{
	return -EOPNOTSUPP;
}

static const struct iio_buffer_setup_ops ev3_pru_tacho_buffer_setup_ops = {
	.postenable = &ev3_pru_tacho_buffer_enable,
	.predisable = &ev3_pru_tacho_buffer_disable,
};

static int ev3_tacho_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct ev3_pru_tacho_msg msg = { .type = EV3_PRU_TACHO_MSG_DATA_ADDR };
	struct device *dev = &rpdev->dev;
	struct iio_dev *indio_dev;
	struct ev3_tacho_rpmsg_data *priv;
	struct iio_buffer *buffer;
	int ret;

	dev_info(dev, "new channel: 0x%x -> 0x%x!\n", rpdev->src, rpdev->dst);

	indio_dev = devm_iio_device_alloc(dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);
	priv->indio_dev = indio_dev;
	priv->rpdev = rpdev;

	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->name = "ev3-tacho";
	indio_dev->dev.parent = dev;
	indio_dev->info = &ev3_pru_tacho_iio_info;
	indio_dev->channels = ev3_tacho_rpmsg_channels;
	indio_dev->num_channels = ARRAY_SIZE(ev3_tacho_rpmsg_channels);
	indio_dev->setup_ops = &ev3_pru_tacho_buffer_setup_ops;

	init_completion(&priv->completion);

	dev_set_drvdata(dev, priv);

	ret = rpmsg_send(rpdev->ept, &msg, sizeof(msg));
	if (ret < 0)
		return ret;

	ret = wait_for_completion_timeout(&priv->completion,
					  msecs_to_jiffies(10));
	if (ret == 0)
		return -ETIMEDOUT;

	priv->remote_data = devm_ioremap(dev, priv->last_value,
					 sizeof(struct ev3_pru_tacho_remote_data));
	if (!priv->remote_data)
		return -ENOMEM;

	memset(priv->remote_data, 0, sizeof(struct ev3_pru_tacho_remote_data));

	buffer = devm_iio_kfifo_allocate(&indio_dev->dev);
	if (!buffer)
		return -ENOMEM;

	iio_device_attach_buffer(indio_dev, buffer);

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret < 0)
		return ret;

	return 0;
}

static void ev3_tacho_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	struct ev3_tacho_rpmsg_data *priv = dev_get_drvdata(dev);

	/*
	 * Need to manually call this now to prevent crash. If we don't rproc
	 * device can be removed before iio device while we are still receiving
	 * rpmsg callbacks. By forcing iio removal here, all buffers will be
	 * removed and the PRU will be instructed to stop sending messages.
	 */
	devm_iio_device_unregister(dev, priv->indio_dev);

	dev_info(&rpdev->dev, "rpmsg sample client driver is removed\n");
}

static struct rpmsg_device_id ev3_tacho_rpmsg_id_table[] = {
	{ .name	= "ev3-tacho-rpmsg" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, ev3_tacho_rpmsg_id_table);

static struct of_device_id ev3_tacho_of_id_table[] = {
	{ .compatible = "ev3dev,ev3-pru-tacho" },
	{ },
};
MODULE_DEVICE_TABLE(of, ev3_tacho_of_id_table);

static struct rpmsg_driver ev3_tacho_rpmsg_client = {
	.drv.name		= KBUILD_MODNAME,
	.drv.of_match_table	= ev3_tacho_of_id_table,
	.id_table		= ev3_tacho_rpmsg_id_table,
	.probe			= ev3_tacho_rpmsg_probe,
	.callback		= ev3_tacho_rpmsg_cb,
	.remove			= ev3_tacho_rpmsg_remove,
};
module_rpmsg_driver(ev3_tacho_rpmsg_client);

MODULE_DESCRIPTION("PRU driver for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("rpmsg:ev3-tacho-rpmsg");
