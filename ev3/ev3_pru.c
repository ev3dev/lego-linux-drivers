// SPDX-License-Identifier: GPL-2.0
/*
 * PRU Driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2018 David Lechner <david@lechnology.com>
 */

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sw_trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/interrupt.h>
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

struct ev3_tacho_rpmsg_data {
	__iomem void *remote_data;
	struct iio_dev *indio_dev;
	struct iio_sw_trigger *sw_trigger;
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

static irqreturn_t ev3_pru_tacho_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ev3_tacho_rpmsg_data *priv = iio_priv(indio_dev);
	struct ev3_pru_tacho_remote_data *data = priv->remote_data;
	s32 buffer[8];
	unsigned long scan_mask = indio_dev->active_scan_mask ?
				 *indio_dev->active_scan_mask : 0;
	int i = 0;

	if (scan_mask & BIT(0))
		buffer[i++] = data->position[0][data->head[0]];
	if (scan_mask & BIT(1))
		buffer[i++] = ev3_pru_tacho_get_freq(data, 0);
	if (scan_mask & BIT(2))
		buffer[i++] = data->position[1][data->head[1]];
	if (scan_mask & BIT(3))
		buffer[i++] = ev3_pru_tacho_get_freq(data, 1);
	if (scan_mask & BIT(4))
		buffer[i++] = data->position[2][data->head[2]];
	if (scan_mask & BIT(5))
		buffer[i++] = ev3_pru_tacho_get_freq(data, 2);
	if (scan_mask & BIT(6))
		buffer[i++] = data->position[3][data->head[3]];
	if (scan_mask & BIT(7))
		buffer[i++] = ev3_pru_tacho_get_freq(data, 3);

	iio_push_to_buffers(indio_dev, buffer);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ev3_pru_tacho_read_raw(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  int *val, int *val2, long mask)
{
	struct ev3_tacho_rpmsg_data *priv = iio_priv(indio_dev);
	struct ev3_pru_tacho_remote_data *data = priv->remote_data;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_COUNT:
			*val = data->position[chan->channel][data->head[chan->channel]];
			return IIO_VAL_INT;
		default:
			break;
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
		.scan_index = EV3_PRU_TACHO_##n * 2,			\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 32,					\
			.storagebits = 32,				\
		},							\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.datasheet_name = "Motor##n",				\
	},								\
	{								\
		.type = IIO_FREQUENCY,					\
		.indexed = 1,						\
		.channel = EV3_PRU_TACHO_##n,				\
		.scan_index = EV3_PRU_TACHO_##n * 2 + 1,		\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 32,					\
			.storagebits = 32,				\
		},							\
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

static int ev3_tacho_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct ev3_pru_tacho_msg msg = { .type = EV3_PRU_TACHO_MSG_DATA_ADDR };
	struct device *dev = &rpdev->dev;
	struct iio_dev *indio_dev;
	struct ev3_tacho_rpmsg_data *priv;
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
	
	ret = devm_iio_triggered_buffer_setup(dev, indio_dev, NULL,
					&ev3_pru_tacho_trigger_handler, NULL);
	if (ret) {
		dev_err(dev, "Failed to setup triggered buffer.\n");
		return ret;
	}

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret < 0)
		return ret;

	/* Hack to create continuous polling mode */
	priv->sw_trigger = iio_sw_trigger_create("hrtimer", dev_name(dev));
	if (IS_ERR(priv->sw_trigger))
		return PTR_ERR(priv->sw_trigger);

	iio_hrtimer_set_sampling_frequency(priv->sw_trigger, 500);
	iio_trigger_set_immutable(indio_dev, priv->sw_trigger->trigger);

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
