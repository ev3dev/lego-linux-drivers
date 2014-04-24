/*
 * Measurement sensor device class for LEGO Mindstorms EV3
 *
 * Copyright (C) 2013 David Lechner <david@lechnology.com>
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
#include <linux/module.h>
#include <linux/legoev3/msensor_class.h>

size_t msensor_data_size[NUM_MSENSOR_DATA_TYPE] = {
	[MSENSOR_DATA_S8]	= 1,
	[MSENSOR_DATA_U8]	= 1,
	[MSENSOR_DATA_S16]	= 2,
	[MSENSOR_DATA_U16]	= 2,
	[MSENSOR_DATA_S16_BE]	= 2,
	[MSENSOR_DATA_S32]	= 4,
	[MSENSOR_DATA_U32]	= 4,
	[MSENSOR_DATA_FLOAT]	= 4,
};
EXPORT_SYMBOL_GPL(msensor_data_size);

/*
 * Some sensors (i.e. UART) send floating point numbers so we need to convert
 * them to integers to be able to handle them in the kernel.
 */

/**
 * msensor_ftoi - convert 32-bit IEEE 754 float to fixed point integer
 * @f: The floating point number.
 * @dp: The number of decimal places in the fixed-point integer.
 */
int msensor_ftoi(u32 f, unsigned dp)
{
	int s = (f & 0x80000000) ? -1 : 1;
	unsigned char e = (f & 0x7F800000) >> 23;
	unsigned long i = f & 0x007FFFFFL;
	unsigned long m;

	/* handle special cases for zero, +/- infinity and NaN */
	if (!e)
		return 0;
	if (e == 255)
		return s == 1 ? INT_MAX : INT_MIN;

	i += 1 << 23;
	while (dp--)
		i *= 10;
	if (e < 150) {
		m = i % (1L << (150 - e));
		i += m >> 1;
		i >>= 150 - e;
	}
	else
		i <<= e - 150;

	return s * i;
}
EXPORT_SYMBOL_GPL(msensor_ftoi);

/**
 * msensor_itof - convert fixed point integer to 32-bit IEEE 754 float
 * @i: The fixed-point integer.
 * @dp: The number of decimal places in the fixed-point integer.
 */
u32 msensor_itof(int i, unsigned dp)
{
	int s = i < 0 ? -1 : 1;
	unsigned char e = 127;
	unsigned long f = i * s;

	/* special case for zero */
	if (i == 0)
		return 0;

	f <<= 23;
	while (dp-- > 0)
		f /= 10;

	while (f >= (1 << 24)) {
		f >>= 1;
		e++;
	}
	while (f < (1 << 23)) {
		f <<= 1;
		e--;
	}
	f -= 1 << 23;
	if (s == -1)
		f |= 0x80000000;

	return f | e << 23;
}
EXPORT_SYMBOL_GPL(msensor_itof);

static ssize_t msensor_show_type_id(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);

	return sprintf(buf, "%d\n", ms->type_id);
}

static ssize_t msensor_show_port_name(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);

	return snprintf(buf, MSENSOR_PORT_NAME_SIZE, "%s\n", ms->port_name);
}

static ssize_t msensor_show_modes(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);
	int i;
	unsigned count = 0;

	for (i = 0; i < ms->num_modes; i++) {
		count += sprintf(buf + count, "%s", ms->mode_info[i].name);
		count += sprintf(buf + count, "%c", ' ');
	}
	if (count == 0)
		return -ENXIO;
	buf[count - 1] = '\n';

	return count;
}

static ssize_t msensor_show_mode(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);
	unsigned count = 0;

	count += sprintf(buf, "%s\n",
			 ms->mode_info[ms->get_mode(ms->context)].name);
	if (count == 0)
		return -ENXIO;

	return count;
}

static ssize_t msensor_store_mode(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct msensor_device *ms = to_msensor_device(dev);
	int i, err;

	for (i = 0; i < ms->num_modes; i++) {
		if (sysfs_streq(buf, ms->mode_info[i].name)) {
			err = ms->set_mode(ms->context, i);
			if (err)
				return err;
			return count;
		}
	}
	return -EINVAL;
}

static ssize_t msensor_show_units(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);
	int mode = ms->get_mode(ms->context);

	return sprintf(buf, "%s\n",ms->mode_info[mode].units);
}

static ssize_t msensor_show_dp(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);
	int mode = ms->get_mode(ms->context);

	return sprintf(buf, "%d\n", ms->mode_info[mode].decimals);
}

static ssize_t msensor_show_num_values(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);
	int mode = ms->get_mode(ms->context);

	return sprintf(buf, "%d\n", ms->mode_info[mode].data_sets);
}

int msensor_raw_u8_value(struct msensor_device *ms, int index)
{
	int mode = ms->get_mode(ms->context);

	return *(u8 *)(ms->mode_info[mode].raw_data + index);
}

int msensor_raw_s8_value(struct msensor_device *ms, int index)
{
	int mode = ms->get_mode(ms->context);

	return *(s8 *)(ms->mode_info[mode].raw_data + index);
}

int msensor_raw_u16_value(struct msensor_device *ms, int index)
{
	int mode = ms->get_mode(ms->context);

	return *(u16 *)(ms->mode_info[mode].raw_data + index * 2);
}

int msensor_raw_s16_value(struct msensor_device *ms, int index)
{
	int mode = ms->get_mode(ms->context);

	return *(s16 *)(ms->mode_info[mode].raw_data + index * 2);
}

int msensor_raw_s16_be_value(struct msensor_device *ms, int index)
{
	int mode = ms->get_mode(ms->context);

	return (s16)ntohs(*(u16 *)(ms->mode_info[mode].raw_data + index * 2));
}

int msensor_raw_u32_value(struct msensor_device *ms, int index)
{
	int mode = ms->get_mode(ms->context);

	return *(u32 *)(ms->mode_info[mode].raw_data + index * 4);
}

int msensor_raw_s32_value(struct msensor_device *ms, int index)
{
	int mode = ms->get_mode(ms->context);

	return *(s32 *)(ms->mode_info[mode].raw_data + index * 4);
}

int msensor_raw_float_value(struct msensor_device *ms, int index)
{
	int mode = ms->get_mode(ms->context);

	return msensor_ftoi(
		*(u32 *)(ms->mode_info[mode].raw_data + index * 4),
		ms->mode_info[mode].decimals);
}

static ssize_t msensor_show_value(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);
	int mode = ms->get_mode(ms->context);
	struct msensor_mode_info *mode_info = &ms->mode_info[mode];
	long int value;
	int index;

	if (strlen(attr->attr.name) < 6)
		return -ENXIO;
	if (sscanf(attr->attr.name + 5, "%d", &index) != 1)
		return -ENXIO;
	if (index < 0 || index >= mode_info->data_sets)
		return -ENXIO;

	switch (mode_info->data_type) {
	case MSENSOR_DATA_U8:
		value = msensor_raw_u8_value(ms, index);
		break;
	case MSENSOR_DATA_S8:
		value = msensor_raw_s8_value(ms, index);
		break;
	case MSENSOR_DATA_U16:
		value = msensor_raw_u16_value(ms, index);
		break;
	case MSENSOR_DATA_S16:
		value = msensor_raw_s16_value(ms, index);
		break;
	case MSENSOR_DATA_S16_BE:
		value = msensor_raw_s16_be_value(ms, index);
		break;
	case MSENSOR_DATA_U32:
		value = msensor_raw_u32_value(ms, index);
		break;
	case MSENSOR_DATA_S32:
		value = msensor_raw_s32_value(ms, index);
		break;
	case MSENSOR_DATA_FLOAT:
		value = msensor_raw_float_value(ms, index);
		break;
	default:
		return -ENXIO;
	}

	value = (value - mode_info->raw_min)
		* (mode_info->si_max - mode_info->si_min)
		/ (mode_info->raw_max - mode_info->raw_min)
		+ mode_info->si_min;

	return sprintf(buf, "%ld\n", value);
}

static ssize_t msensor_show_bin_data_format(struct device *dev,
                                            struct device_attribute *attr,
                                            char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);
	int mode = ms->get_mode(ms->context);
	char *value;

	switch (ms->mode_info[mode].data_type) {
	case MSENSOR_DATA_U8:
		value = "u8";
		break;
	case MSENSOR_DATA_S8:
		value = "s8";
		break;
	case MSENSOR_DATA_U16:
		value = "u16";
		break;
	case MSENSOR_DATA_S16:
		value = "s16";
		break;
	case MSENSOR_DATA_S16_BE:
		value = "s16_be";
		break;
	case MSENSOR_DATA_U32:
		value = "u32";
		break;
	case MSENSOR_DATA_S32:
		value = "s32";
		break;
	case MSENSOR_DATA_FLOAT:
		value = "float";
		break;
	default:
		return -ENXIO;
	}

	return sprintf(buf, "%s\n", value);
}

static ssize_t msensor_show_poll_ms(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);
	int ret;

	if (!ms->get_poll_ms)
		return -ENXIO;

	ret = ms->get_poll_ms(ms->context);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t msensor_store_poll_ms(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	struct msensor_device *ms = to_msensor_device(dev);
	unsigned value;
	int err;

	if (!ms->set_poll_ms)
		return -ENXIO;

	if (sscanf(buf, "%ud", &value) != 1)
		return -EINVAL;
	err = ms->set_poll_ms(ms->context, value);
	if (err < 0)
		return err;

	return count;
}

static ssize_t msensor_show_fw_version(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);

	return snprintf(buf, MSENSOR_FW_VERSION_SIZE, "%s\n", ms->fw_version);
}

static ssize_t msensor_show_i2c_addr(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);

	return sprintf(buf, "0x%02x\n", ms->i2c_addr);
}

static ssize_t msensor_read_bin_data(struct file *file, struct kobject *kobj,
                                     struct bin_attribute *attr,
                                     char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct msensor_device *ms = to_msensor_device(dev);
	int mode = ms->get_mode(ms->context);
	size_t size = attr->size;

	if (off >= size || !count)
		return 0;
	size -= off;
	if (count < size)
		size = count;
	memcpy(buf + off, ms->mode_info[mode].raw_data, size);

	return size;
}

static ssize_t msensor_write_bin_data(struct file *file ,struct kobject *kobj,
                                      struct bin_attribute *attr,
                                      char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct msensor_device *ms = to_msensor_device(dev);

	if (!ms->write_data)
		return -ENXIO;

	return ms->write_data(ms->context, buf, off, count);
}

static struct device_attribute msensor_device_attrs[] = {
	__ATTR(type_id, S_IRUGO, msensor_show_type_id, NULL),
	__ATTR(port_name, S_IRUGO, msensor_show_port_name, NULL),
	__ATTR(modes, S_IRUGO | S_IWUGO, msensor_show_modes, NULL),
	__ATTR(mode, S_IRUGO | S_IWUGO, msensor_show_mode, msensor_store_mode),
	__ATTR(units, S_IRUGO, msensor_show_units, NULL),
	__ATTR(dp, S_IRUGO, msensor_show_dp, NULL),
	__ATTR(num_values, S_IRUGO, msensor_show_num_values, NULL),
	__ATTR(bin_data_format, S_IRUGO, msensor_show_bin_data_format, NULL),
	/*
	 * Technically, it is possible to have 32 8-bit values from UART sensors
	 * and 255 8-bit values from I2C sensors, but known UART sensors so far
	 * have 8 data values or less and I2C sensors can arbitrarily be split
	 * into multiple modes, so we only expose 8 values to prevent sysfs
	 * overcrowding.
	 */
	__ATTR(value0, S_IRUGO , msensor_show_value, NULL),
	__ATTR(value1, S_IRUGO , msensor_show_value, NULL),
	__ATTR(value2, S_IRUGO , msensor_show_value, NULL),
	__ATTR(value3, S_IRUGO , msensor_show_value, NULL),
	__ATTR(value4, S_IRUGO , msensor_show_value, NULL),
	__ATTR(value5, S_IRUGO , msensor_show_value, NULL),
	__ATTR(value6, S_IRUGO , msensor_show_value, NULL),
	__ATTR(value7, S_IRUGO , msensor_show_value, NULL),
	__ATTR_NULL
};

static DEVICE_ATTR(poll_ms, S_IRUGO | S_IWUGO, msensor_show_poll_ms, msensor_store_poll_ms);
static DEVICE_ATTR(fw_version, S_IRUGO , msensor_show_fw_version, NULL);
static DEVICE_ATTR(i2c_addr, S_IRUGO , msensor_show_i2c_addr, NULL);

struct attribute *msensor_optional_attrs[] = {
	&dev_attr_poll_ms.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_i2c_addr.attr,
	NULL
};

static umode_t msensor_attr_is_visible (struct kobject *kobj,
					struct attribute *attr, int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct msensor_device *ms = to_msensor_device(dev);

	if (attr == &dev_attr_poll_ms.attr)
		return (ms->get_poll_ms || ms->set_poll_ms) ? attr->mode : 0;
	if (attr == &dev_attr_fw_version.attr)
		return ms->fw_version[0] ? attr->mode : 0;
	if (attr == &dev_attr_i2c_addr.attr)
		return ms->i2c_addr ? attr->mode : 0;

	return attr->mode;
}

static struct attribute_group msensor_optional_attr_grp = {
	.is_visible	= msensor_attr_is_visible,
	.attrs		= msensor_optional_attrs,
};

static struct bin_attribute msensor_device_bin_attrs[] = {
	{
		.attr	= {
			.name	= "bin_data",
			.mode	= S_IRUGO,
		},
		.size	= MSENSOR_RAW_DATA_SIZE,
		.read	= msensor_read_bin_data,
		.write	= msensor_write_bin_data,
	},
	__ATTR_NULL
};

static void msensor_release(struct device *dev)
{
}

static unsigned msensor_class_id = 0;

int register_msensor(struct msensor_device *ms, struct device *parent)
{
	int err;

	if (!ms || !ms->port_name || !parent)
		return -EINVAL;

	ms->dev.release = msensor_release;
	ms->dev.parent = parent;
	ms->dev.class = &msensor_class;
	dev_set_name(&ms->dev, "sensor%d", msensor_class_id++);

	err = device_register(&ms->dev);
	if (err)
		return err;

	err = sysfs_create_group(&ms->dev.kobj, &msensor_optional_attr_grp);
	if (err) {
		device_unregister(&ms->dev);
		return err;
	}

	dev_info(&ms->dev, "Mindstorms sensor registered.\n");

	return 0;
}
EXPORT_SYMBOL_GPL(register_msensor);

void unregister_msensor(struct msensor_device *ms)
{
	dev_info(&ms->dev, "Mindstorms sensor unregistered.\n");
	sysfs_remove_group(&ms->dev.kobj, &msensor_optional_attr_grp);
	device_unregister(&ms->dev);
}
EXPORT_SYMBOL_GPL(unregister_msensor);

static char *msensor_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "msensor/%s", dev_name(dev));
}

struct class msensor_class = {
	.name		= "msensor",
	.owner		= THIS_MODULE,
	.dev_attrs	= msensor_device_attrs,
	.dev_bin_attrs	= msensor_device_bin_attrs,
	.devnode	= msensor_devnode,
};
EXPORT_SYMBOL_GPL(msensor_class);

static int __init msensor_class_init(void)
{
	int err;

	err = class_register(&msensor_class);
	if (err) {
		pr_err("unable to register msensor device class\n");
		return err;
	}

	return 0;
}
module_init(msensor_class_init);

static void __exit msensor_class_exit(void)
{
	class_unregister(&msensor_class);
}
module_exit(msensor_class_exit);

MODULE_DESCRIPTION("Mindstorms sensor device class for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
