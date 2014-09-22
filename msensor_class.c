/*
 * Measurement sensor device class for LEGO Mindstorms EV3
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
 * Note: The comment block below is used to generate docs on the ev3dev website.
 * Use kramdown (markdown) format. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * MINDSTORMS Sensor (msensor) Class
 *
* The `msensor` class provides a uniform interface for using most of the sensors
* available for the EV3. The various underlying device drivers will create a
* `msensor` device for interacting with the sensors.
* .
* Sensors are primarily controlled by setting the `mode` and monitored by
* reading the `value<N>` attributes. Values can be converted to floating point
* if needed by `value<N>` / 10.0 ^ `dp`.
* .
* ### Identifying sensors
* .
* Since the name of the `sensor<N>` device node does not correspond to the port
* that a sensor is plugged in to, you must look at the port_name attribute if
* you need to know which port a sensor is plugged in to. However, if you don't
* have more than one sensor of each type, you can just look for a matching
* `name`. Then it will not matter which port a sensor is plugged in to - your
* program will still work. In the case of I2C sensors, you may also want to
* check the `address` since it is possible to have more than one I2C sensor
* connected to a single input port at a time.
* .
* ### sysfs Attributes
* .
* Sensors can be found at `/sys/class/msensor/sensor<N>`, where `<N>` is
* incremented each time a sensor is loaded (is is not related to which port
* the sensor is plugged in to).
* .
* `bin_data` (read/write)
* : Reading the file will give the same values in the `value<N>` attributes.
* .    Use `bin_data_format` and `num_values` to determine how to interpret
* .    the data. Writing will write data to the sensor (I2C, UART and NXT
* .    Color sensors only).
* .
* `bin_data_format` (read-only)
* : Returns the format of the values in `bin_data` for the current mode.
* .    Possible values are:
* .
* .    - `u8`: Unsigned 8-bit integer (byte)
* .    - `s8`: Signed 8-bit integer (sbyte)
* .    - `u16`: Unsigned 16-bit integer (ushort)
* .    - `s16`: Signed 16-bit integer (short)
* .    - `s16_be`: Signed 16-bit integer, big endian
* .    - `s32`: Signed 32-bit integer (int)
* .    - `float`: IEEE 754 32-bit floating point (float)
* .
* `dp` (read-only)
* : Returns the number of decimal places for the values in the `value<N>`
* .    attributes of the current mode.
* .
* `fw_version` (read-only)
* : Present only for [nxt-i2c-sensor] devices. Returns the firmware version of
* .    the sensor.
* .
* `address` (read-only)
* : Present only for I2C devices. Returns the address of the sensor.
* .
* `modes` (read-only)
* : Returns a space separated list of the valid modes for the sensor.
* .
* `mode` (read/write)
* : Returns the current mode. Writing one of the values returned by `modes` sets
* .    the sensor to that mode.
* .
* `num_values` (read-only)
* : Returns the number of `value<N>` attributes that will return a valid value
* .    for the current mode.
* .
* `poll_ms` (read/write)
* : | Present only for [nxt-i2c-sensor] devices. Returns the polling period of
* .    the sensor in milliseconds. Writing sets the polling period. Setting to
* .    0 disables polling. Minimum value is hard coded as 50 msec.
* .
* `port_name` (read-only)
* : Returns the name of the port that the sensor is connected to.
* .
* `units` (read-only)
* : Returns the units of the measured value for the current mode. May return
* .    empty string"
* .
* `name` (read-only)
* : Returns the name of the sensor driver. See the list of [supported sensors]
* .    for a complete list of drivers.
* .
* `value<N>` (read-only)
* : Returns the value or values measured by the sensor. Check `num_values` to
* .    see how many values there are. Values with N >= num_values will return an
* .    error. The values are fixed point numbers, so check `dp` to see if you
* .    need to divide to get the actual value.
* .
* [nxt-i2c-sensor]: ../nxt-i2c-sensor
* [supported sensors]: ../#supported-sensors
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

static ssize_t msensor_show_name(struct device *dev,
				 struct device_attribute *attr,  char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);

	return snprintf(buf, MSENSOR_NAME_SIZE, "%s\n", ms->name);
}

static ssize_t msensor_show_port_name(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);

	return snprintf(buf, MSENSOR_NAME_SIZE, "%s\n", ms->port_name);
}

static ssize_t msensor_show_modes(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);
	int i;
	unsigned count = 0;

	for (i = 0; i < ms->num_modes; i++)
		count += sprintf(buf + count, "%s ", ms->mode_info[i].name);
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

	return snprintf(buf, MSENSOR_FW_VERSION_SIZE + 2, "%s\n", ms->fw_version);
}

static ssize_t msensor_show_address(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct msensor_device *ms = to_msensor_device(dev);

	return sprintf(buf, "%u\n", ms->address);
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

static DEVICE_ATTR(name, S_IRUGO, msensor_show_name, NULL);
static DEVICE_ATTR(port_name, S_IRUGO, msensor_show_port_name, NULL);
static DEVICE_ATTR(address, S_IRUGO , msensor_show_address, NULL);
static DEVICE_ATTR(modes, S_IRUGO, msensor_show_modes, NULL);
static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, msensor_show_mode, msensor_store_mode);
static DEVICE_ATTR(units, S_IRUGO, msensor_show_units, NULL);
static DEVICE_ATTR(dp, S_IRUGO, msensor_show_dp, NULL);
static DEVICE_ATTR(num_values, S_IRUGO, msensor_show_num_values, NULL);
static DEVICE_ATTR(bin_data_format, S_IRUGO, msensor_show_bin_data_format, NULL);
/*
 * Technically, it is possible to have 32 8-bit values from UART sensors
 * and >200 8-bit values from I2C sensors, but known UART sensors so far
 * have 8 data values or less and I2C sensors can arbitrarily be split
 * into multiple modes, so we only expose 8 values to prevent sysfs
 * overcrowding.
 */
static DEVICE_ATTR(value0, S_IRUGO , msensor_show_value, NULL);
static DEVICE_ATTR(value1, S_IRUGO , msensor_show_value, NULL);
static DEVICE_ATTR(value2, S_IRUGO , msensor_show_value, NULL);
static DEVICE_ATTR(value3, S_IRUGO , msensor_show_value, NULL);
static DEVICE_ATTR(value4, S_IRUGO , msensor_show_value, NULL);
static DEVICE_ATTR(value5, S_IRUGO , msensor_show_value, NULL);
static DEVICE_ATTR(value6, S_IRUGO , msensor_show_value, NULL);
static DEVICE_ATTR(value7, S_IRUGO , msensor_show_value, NULL);

static struct attribute *msensor_class_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_port_name.attr,
	&dev_attr_address.attr,
	&dev_attr_modes.attr,
	&dev_attr_mode.attr,
	&dev_attr_units.attr,
	&dev_attr_dp.attr,
	&dev_attr_num_values.attr,
	&dev_attr_bin_data_format.attr,
	&dev_attr_value0.attr,
	&dev_attr_value1.attr,
	&dev_attr_value2.attr,
	&dev_attr_value3.attr,
	&dev_attr_value4.attr,
	&dev_attr_value5.attr,
	&dev_attr_value6.attr,
	&dev_attr_value7.attr,
	NULL
};

static BIN_ATTR(bin_data, S_IRUGO, msensor_read_bin_data,
		msensor_write_bin_data, MSENSOR_RAW_DATA_SIZE);

static struct bin_attribute *msensor_class_bin_attrs[] = {
	&bin_attr_bin_data,
	NULL
};

static const struct attribute_group msensor_class_group = {
	.attrs		= msensor_class_attrs,
	.bin_attrs	= msensor_class_bin_attrs,
};

static DEVICE_ATTR(poll_ms, S_IRUGO | S_IWUSR, msensor_show_poll_ms, msensor_store_poll_ms);
static DEVICE_ATTR(fw_version, S_IRUGO , msensor_show_fw_version, NULL);

struct attribute *msensor_class_optional_attrs[] = {
	&dev_attr_poll_ms.attr,
	&dev_attr_fw_version.attr,
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

	return attr->mode;
}

static const struct attribute_group msensor_class_optional_group = {
	.is_visible	= msensor_attr_is_visible,
	.attrs		= msensor_class_optional_attrs,
};

static const struct attribute_group *msensor_class_groups[] = {
	&msensor_class_group,
	&msensor_class_optional_group,
	NULL
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

	dev_info(&ms->dev, "Bound to device '%s'\n", dev_name(parent));

	return 0;
}
EXPORT_SYMBOL_GPL(register_msensor);

void unregister_msensor(struct msensor_device *ms)
{
	dev_info(&ms->dev, "Unregistered\n");
	device_unregister(&ms->dev);
}
EXPORT_SYMBOL_GPL(unregister_msensor);

static int msensor_dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct msensor_device *ms = to_msensor_device(dev);
	int ret;

	ret = add_uevent_var(env, "NAME=%s", ms->name);
	if (ret) {
		dev_err(dev, "failed to add uevent DEVNAME\n");
		return ret;
	}

	add_uevent_var(env, "PORT=%s", ms->port_name);
	if (ret) {
		dev_err(dev, "failed to add uevent PORT\n");
		return ret;
	}

	add_uevent_var(env, "ADDRESS=0x%02x", ms->address);
	if (ret) {
		dev_err(dev, "failed to add uevent ADDRESS\n");
		return ret;
	}

	return 0;
}

static char *msensor_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "msensor/%s", dev_name(dev));
}

struct class msensor_class = {
	.name		= "msensor",
	.owner		= THIS_MODULE,
	.dev_groups	= msensor_class_groups,
	.dev_uevent	= msensor_dev_uevent,
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
