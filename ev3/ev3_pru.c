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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/types.h>

enum ev3_pru_tacho_msg_type {
	EV3_PRU_TACHO_MSG_START,
	EV3_PRU_TACHO_MSG_STOP,
	EV3_PRU_TACHO_MSG_REQ_ONE,
	EV3_PRU_TACHO_MSG_UPDATE,
};

enum ev3_pru_tacho_iio_channel {
	EV3_PRU_IIO_CH_TIMESTAMP,
	EV3_PRU_IIO_CH_TACHO_A,
	EV3_PRU_IIO_CH_TACHO_B,
	EV3_PRU_IIO_CH_TACHO_C,
	EV3_PRU_IIO_CH_TACHO_D,
	NUM_EV3_PRU_IIO_CH
};

struct ev3_pru_tacho_msg {
	uint32_t type;
	uint32_t value[NUM_EV3_PRU_IIO_CH];
};

struct ev3_tacho_rpmsg_data {
	struct iio_dev *indio_dev;
	struct rpmsg_device *rpdev;
	struct completion completion;
	u32 last_value[NUM_EV3_PRU_IIO_CH];
};

static int ev3_tacho_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			      void *p, u32 src)
{
	struct ev3_tacho_rpmsg_data *priv = dev_get_drvdata(&rpdev->dev);
	struct ev3_pru_tacho_msg *msg = data;
	int ret;

	if (len < sizeof(*msg))
		return -EINVAL;

	switch (msg->type) {
	case EV3_PRU_TACHO_MSG_REQ_ONE:
		memcpy(priv->last_value, msg->value, 4 * NUM_EV3_PRU_IIO_CH);
		complete(&priv->completion);
		break;
	case EV3_PRU_TACHO_MSG_UPDATE:
		ret = iio_push_to_buffers(priv->indio_dev, &msg->value[0]);
		if (ret < 0)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define EV3_PRU_IIO_COUNT_MOTOR(x) {						\
		.type = IIO_COUNT,						\
		.indexed = 1,							\
		.channel = EV3_PRU_IIO_CH_TACHO_##x - EV3_PRU_IIO_CH_TACHO_A,	\
		.scan_index = EV3_PRU_IIO_CH_TACHO_##x,				\
		.scan_type = {							\
			.sign = 's',						\
			.realbits = 32,						\
			.storagebits = 32,					\
		},								\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
		.datasheet_name = "Motor" __stringify(x),			\
	}


static const struct iio_chan_spec ev3_tacho_rpmsg_channels[] = {
	[EV3_PRU_IIO_CH_TIMESTAMP] = {
		.type = IIO_TIMESTAMP,
		.scan_index = EV3_PRU_IIO_CH_TIMESTAMP,
		.scan_type = {
			.sign = 'u',
			.realbits = 32,
			.storagebits = 32,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	[EV3_PRU_IIO_CH_TACHO_A] = EV3_PRU_IIO_COUNT_MOTOR(A),
	[EV3_PRU_IIO_CH_TACHO_B] = EV3_PRU_IIO_COUNT_MOTOR(B),
	[EV3_PRU_IIO_CH_TACHO_C] = EV3_PRU_IIO_COUNT_MOTOR(C),
	[EV3_PRU_IIO_CH_TACHO_D] = EV3_PRU_IIO_COUNT_MOTOR(D),
};

static int ev3_pru_tacho_read_raw(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  int *value, int *shift, long mask)
{
	struct ev3_tacho_rpmsg_data *priv = iio_priv(indio_dev);
	enum ev3_pru_tacho_msg_type msg = EV3_PRU_TACHO_MSG_REQ_ONE;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		reinit_completion(&priv->completion);

		ret = rpmsg_send(priv->rpdev->ept, &msg, sizeof(msg));
		if (ret < 0) {
			mutex_unlock(&indio_dev->mlock);
			return ret;
		}

		ret = wait_for_completion_timeout(&priv->completion,
						  msecs_to_jiffies(10));
		if (ret == 0) {
			mutex_unlock(&indio_dev->mlock);
			return -ETIMEDOUT;
		}

		*value = priv->last_value[chan->scan_index];
		mutex_unlock(&indio_dev->mlock);

		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static const struct iio_info ev3_pru_tacho_iio_info = {
	.driver_module = THIS_MODULE,
	.read_raw = ev3_pru_tacho_read_raw,
};

static int ev3_pru_tacho_buffer_enable(struct iio_dev *indio_dev)
{
	struct ev3_tacho_rpmsg_data *priv = iio_priv(indio_dev);
	struct ev3_pru_tacho_msg msg = { .type = EV3_PRU_TACHO_MSG_START };

	return rpmsg_send(priv->rpdev->ept, &msg, sizeof(msg));
}

static int ev3_pru_tacho_buffer_disable(struct iio_dev *indio_dev)
{
	struct ev3_tacho_rpmsg_data *priv = iio_priv(indio_dev);
	struct ev3_pru_tacho_msg msg = { .type = EV3_PRU_TACHO_MSG_STOP };

	return rpmsg_send(priv->rpdev->ept, &msg, sizeof(msg));
}

static const struct iio_buffer_setup_ops ev3_pru_tacho_buffer_setup_ops = {
	.postenable = &ev3_pru_tacho_buffer_enable,
	.predisable = &ev3_pru_tacho_buffer_disable,
};

static int ev3_tacho_rpmsg_probe(struct rpmsg_device *rpdev)
{
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

static struct rpmsg_driver ev3_tacho_rpmsg_client = {
	.drv.name	= KBUILD_MODNAME,
	.id_table	= ev3_tacho_rpmsg_id_table,
	.probe		= ev3_tacho_rpmsg_probe,
	.callback	= ev3_tacho_rpmsg_cb,
	.remove		= ev3_tacho_rpmsg_remove,
};
module_rpmsg_driver(ev3_tacho_rpmsg_client);

MODULE_DESCRIPTION("PRU driver for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("rpmsg:ev3-tacho-rpmsg");
