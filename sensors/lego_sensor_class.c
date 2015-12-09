/*
 * LEGO sensor device class
 *
 * Copyright (C) 2013-2015 David Lechner <david@lechnology.com>
 * Copyright (C) 2015      Ralph Hempel <rhempel@hempeldesigngroup.com>
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
 * Use kramdown (markdown) syntax. Use a '.' as a placeholder when blank lines
 * or leading whitespace is important for the markdown syntax.
 */

/**
 * DOC: website
 *
 * LEGO Sensor Class
 *
 * The `lego-sensor` class provides a uniform interface for using most of the
 * sensors available for the EV3. The various underlying device drivers will
 * create a `lego-sensor` device for interacting with the sensors.
 * .
 * Sensors are primarily controlled by setting the `mode` and monitored by
 * reading the `value<N>` attributes. Values can be converted to floating point
 * if needed by `value<N>` / 10.0 ^ `decimals`.
 * .
 * ### Identifying sensors
 * .
 * Since the name of the `sensor<N>` device node does not correspond to the port
 * that a sensor is plugged in to, you must look at the `port_name` attribute if
 * you need to know which port a sensor is plugged in to. However, if you don't
 * have more than one sensor of each type, you can just look for a matching
 * `driver_name`. Then it will not matter which port a sensor is plugged in to - your
 * program will still work.
 * .
 * ### sysfs Attributes
 * .
 * Sensors can be found at `/sys/class/lego-sensor/sensor<N>`, where `<N>` is
 * incremented each time a sensor is loaded (it is not related to which port
 * the sensor is plugged in to).
 * .
 * `bin_data` (read-only)
 * : Reading the file will give the unscaled raw values in the `value<N>`
 *   attributes. Use `bin_data_format`, `num_values` and the individual sensor
 *   documentation to determine how to interpret the data.
 * .
 * `bin_data_format` (read-only)
 * : Returns the format of the values in `bin_data` for the current mode.
 *   Possible values are:
 * .
 * .    - `u8`: Unsigned 8-bit integer (byte)
 * .    - `s8`: Signed 8-bit integer (sbyte)
 * .    - `u16`: Unsigned 16-bit integer (ushort)
 * .    - `s16`: Signed 16-bit integer (short)
 * .    - `s16_be`: Signed 16-bit integer, big endian
 * .    - `s32`: Signed 32-bit integer (int)
 * .    - `float`: IEEE 754 32-bit floating point (float)
 * .
 * `command` (write-only)
 * : Sends a command to the sensor.
 * .
 * `commands` (read-only)
 * : Returns a space separated list of the valid commands for the sensor.
 *   Returns -EOPNOTSUPP if no commands are supported.
 * .
 * `direct` (read/write)
 * : Allows direct communication with the sensor for using advanced features
 *   that are not otherwise available through the lego-sensor class. Returns
 *   `-EOPNOTSUPP` if the sensor does not support this. Currently this only
 *   works with I2C sensors. For I2C sensors, use `seek()` to set the register
 *   to read or write from, then read or write the number of bytes required.
 * .
 * `decimals` (read-only)
 * : Returns the number of decimal places for the values in the `value<N>`
 *   attributes of the current mode.
 * .
 * `driver_name` (read-only)
 * : Returns the name of the sensor device/driver. See the list of [supported
 *   sensors] for a complete list of drivers.
 * .
 * `fw_version` (read-only)
 * : Returns the firmware version of the sensor if available. Currently only
 *   NXT/I2C sensors support this.
 * .
 * `mode` (read/write)
 * : Returns the current mode. Writing one of the values returned by `modes`
 *   sets the sensor to that mode.
 * .
 * `modes` (read-only)
 * : Returns a space separated list of the valid modes for the sensor.
 * .
 * `num_values` (read-only)
 * : Returns the number of `value<N>` attributes that will return a valid value
 *   for the current mode.
 * .
 * `poll_ms` (read/write)
 * : Returns the polling period of the sensor in milliseconds. Writing sets the
 *   polling period. Setting to 0 disables polling. Returns -EOPNOTSUPP if
 *   changing polling is not supported. Note: Setting poll_ms too low can cause
 *   the input port autodetection to fail. If this happens, use the `mode`
 *   attribute of the port to force the port to nxt-i2c mode.
 * .
 * `port_name` (read-only)
 * : Returns the name of the port that the sensor is connected to, e.g. `in1`.
 *   I2C sensors also include the I2C address (decimal), e.g. `in1:i2c8`.
 * .
 * `units` (read-only)
 * : Returns the units of the measured value for the current mode. May return
 *   empty string"
 * .
 * `value<N>` (read-only)
 * : Returns the value or values measured by the sensor. Check `num_values` to
 *   see how many values there are. Values with N >= num_values will return an
 *   error. The values are fixed point numbers, so check `decimals` to see if
 *   you need to divide to get the actual value.
 * .
 * `text_value` (read-only)
 * : Returns a string representing sensor-specific text value. The string may
 *   contain embedded line feed characters, is limited to PAGE_SIZE bytes
 *   in length, and has a trailing linefeed
 * .
 * ### Events
 * .
 * In addition to the usual "add" and "remove" events, the kernel "change"
 * event is emitted when `mode` or `poll_ms` is changed. The `value<N>`
 * attributes change too rapidly to be handled this way and therefore do not
 * trigger any uevents.
 * .
 * [nxt-i2c-sensor]: ../nxt-i2c-sensor
 * [supported sensors]: /docs/sensors#supported-sensors
 */

#include <linux/device.h>
#include <linux/module.h>

#include <lego_sensor_class.h>

size_t lego_sensor_data_size[NUM_LEGO_SENSOR_DATA_TYPE] = {
	[LEGO_SENSOR_DATA_S8]		= 1,
	[LEGO_SENSOR_DATA_U8]		= 1,
	[LEGO_SENSOR_DATA_S16]		= 2,
	[LEGO_SENSOR_DATA_U16]		= 2,
	[LEGO_SENSOR_DATA_S16_BE]	= 2,
	[LEGO_SENSOR_DATA_S32]		= 4,
	[LEGO_SENSOR_DATA_U32]		= 4,
	[LEGO_SENSOR_DATA_FLOAT]	= 4,
};
EXPORT_SYMBOL_GPL(lego_sensor_data_size);

/*
 * Some sensors (i.e. UART) send floating point numbers so we need to convert
 * them to integers to be able to handle them in the kernel.
 */

/**
 * lego_sensor_ftoi - convert 32-bit IEEE 754 float to fixed point integer
 * @f: The floating point number.
 * @dp: The number of decimal places in the fixed-point integer.
 */
int lego_sensor_ftoi(u32 f, unsigned dp)
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
EXPORT_SYMBOL_GPL(lego_sensor_ftoi);

/**
 * lego_sensor_itof - convert fixed point integer to 32-bit IEEE 754 float
 * @i: The fixed-point integer.
 * @dp: The number of decimal places in the fixed-point integer.
 */
u32 lego_sensor_itof(int i, unsigned dp)
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
EXPORT_SYMBOL_GPL(lego_sensor_itof);

static ssize_t driver_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);

	return snprintf(buf, LEGO_SENSOR_NAME_SIZE, "%s\n", sensor->name);
}

static ssize_t port_name_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);

	return snprintf(buf, LEGO_SENSOR_NAME_SIZE, "%s\n", sensor->port_name);
}

static ssize_t modes_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);
	int i;
	unsigned count = 0;

	for (i = 0; i < sensor->num_modes; i++)
		count += sprintf(buf + count, "%s ", sensor->mode_info[i].name);
	if (count == 0)
		return -ENXIO;
	buf[count - 1] = '\n';

	return count;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);

	return sprintf(buf, "%s\n", sensor->mode_info[sensor->mode].name);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);
	int i, err;

	for (i = 0; i < sensor->num_modes; i++) {
		if (sysfs_streq(buf, sensor->mode_info[i].name)) {
			err = sensor->set_mode(sensor->context, i);
			if (err)
				return err;
			if (sensor->mode != i) {
				sensor->mode = i;
				kobject_uevent(&dev->kobj, KOBJ_CHANGE);
			}
			return count;
		}
	}

	return -EINVAL;
}

static ssize_t commands_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);
	int i;
	unsigned count = 0;

	for (i = 0; i < sensor->num_commands; i++)
		count += sprintf(buf + count, "%s ", sensor->cmd_info[i].name);
	if (count == 0)
		return -EOPNOTSUPP;
	buf[count - 1] = '\n';

	return count;
}

static ssize_t command_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);
	int i, err;

	for (i = 0; i < sensor->num_commands; i++) {
		if (sysfs_streq(buf, sensor->cmd_info[i].name)) {
			err = sensor->send_command(sensor->context, i);
			if (err)
				return err;
			return count;
		}
	}
	return -EINVAL;
}

static ssize_t units_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);

	return sprintf(buf, "%s\n",sensor->mode_info[sensor->mode].units);
}

static ssize_t decimals_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);

	return sprintf(buf, "%d\n", sensor->mode_info[sensor->mode].decimals);
}

static ssize_t num_values_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);

	return sprintf(buf, "%d\n",
		lego_sensor_get_num_values(&sensor->mode_info[sensor->mode]));
}

int lego_sensor_default_scale(struct lego_sensor_mode_info *mode_info, u8 index,
			      long int *value)
{
	switch (mode_info->data_type) {
	case LEGO_SENSOR_DATA_U8:
		*value = *(u8 *)(mode_info->raw_data + index);
		break;
	case LEGO_SENSOR_DATA_S8:
		*value = *(s8 *)(mode_info->raw_data + index);
		break;
	case LEGO_SENSOR_DATA_U16:
		*value = *(u16 *)(mode_info->raw_data + index * 2);
		break;
	case LEGO_SENSOR_DATA_S16:
		*value = *(s16 *)(mode_info->raw_data + index * 2);
		break;
	case LEGO_SENSOR_DATA_S16_BE:
		*value = (s16)ntohs(*(u16 *)(mode_info->raw_data + index * 2));
		break;
	case LEGO_SENSOR_DATA_U32:
		*value = *(u32 *)(mode_info->raw_data + index * 4);
		break;
	case LEGO_SENSOR_DATA_S32:
		*value = *(s32 *)(mode_info->raw_data + index * 4);
		break;
	case LEGO_SENSOR_DATA_FLOAT:
		*value = lego_sensor_ftoi(
			*(u32 *)(mode_info->raw_data + index * 4),
			mode_info->decimals);
		break;
	default:
		return -ENXIO;
	}

	if (mode_info->raw_min != mode_info->raw_max) {
		*value = (*value - mode_info->raw_min)
			* (mode_info->si_max - mode_info->si_min)
			/ (mode_info->raw_max - mode_info->raw_min)
			+ mode_info->si_min;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(lego_sensor_default_scale);

static ssize_t value_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);
	struct lego_sensor_mode_info *mode_info = &sensor->mode_info[sensor->mode];
	long int value;
	int index, err;

	if (strlen(attr->attr.name) < 6)
		return -ENXIO;
	if (sscanf(attr->attr.name + 5, "%d", &index) != 1)
		return -ENXIO;
	if (index < 0 || index >= lego_sensor_get_num_values(mode_info))
		return -ENXIO;

	if (mode_info->scale)
		err = mode_info->scale(sensor->context, mode_info, index, &value);
	else
		err = lego_sensor_default_scale(mode_info, index, &value);
	if (err)
		return err;

	return sprintf(buf, "%ld\n", value);
}

const char *lego_sensor_bin_data_format_to_str(enum lego_sensor_data_type value)
{
	switch (value) {
	case LEGO_SENSOR_DATA_U8:
		return "u8";
	case LEGO_SENSOR_DATA_S8:
		return "s8";
	case LEGO_SENSOR_DATA_U16:
		return "u16";
	case LEGO_SENSOR_DATA_S16:
		return "s16";
	case LEGO_SENSOR_DATA_S16_BE:
		return "s16_be";
	case LEGO_SENSOR_DATA_U32:
		return "u32";
	case LEGO_SENSOR_DATA_S32:
		return "s32";
	case LEGO_SENSOR_DATA_FLOAT:
		return "float";
	default:
		return NULL;
	}
}
EXPORT_SYMBOL_GPL(lego_sensor_bin_data_format_to_str);

int lego_sensor_str_to_bin_data_format(const char *value)
{
	int i;

	for (i = 0; i < NUM_LEGO_SENSOR_DATA_TYPE; i++) {
		const char *other = lego_sensor_bin_data_format_to_str(i);
		if (sysfs_streq(value, other))
			return i;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(lego_sensor_str_to_bin_data_format);

static ssize_t bin_data_format_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);
	const char *value = lego_sensor_bin_data_format_to_str(
				sensor->mode_info[sensor->mode].data_type);

	if (!value)
		return -ENXIO;

	return sprintf(buf, "%s\n", value);
}

static ssize_t poll_ms_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);
	int ret;

	if (!sensor->get_poll_ms)
		return -EOPNOTSUPP;

	ret = sensor->get_poll_ms(sensor->context);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t poll_ms_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);
	unsigned value;
	int err;

	if (!sensor->set_poll_ms)
		return -EOPNOTSUPP;

	if (sscanf(buf, "%ud", &value) != 1)
		return -EINVAL;
	err = sensor->set_poll_ms(sensor->context, value);
	if (err < 0)
		return err;

	kobject_uevent(&dev->kobj, KOBJ_CHANGE);

	return count;
}

static ssize_t fw_version_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);

	return snprintf(buf, LEGO_SENSOR_FW_VERSION_SIZE + 2, "%s\n", sensor->fw_version);
}

static ssize_t text_value_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);

	if (!sensor->text_value_read)
		return -EOPNOTSUPP;

        return sensor->text_value_read(buf, PAGE_SIZE);
}

static ssize_t bin_data_read(struct file *file, struct kobject *kobj,
			     struct bin_attribute *attr,
			     char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);
	size_t size = attr->size;

	if (off >= size || !count)
		return 0;
	size -= off;
	if (count < size)
		size = count;
	memcpy(buf + off, sensor->mode_info[sensor->mode].raw_data, size);

	return size;
}

static ssize_t direct_read(struct file *file, struct kobject *kobj,
			   struct bin_attribute *attr,
			   char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);

	if (!sensor->direct_read)
		return -EOPNOTSUPP;

	return sensor->direct_read(sensor->context, buf, off, count);
}

static ssize_t direct_write(struct file *file, struct kobject *kobj,
			    struct bin_attribute *attr,
			    char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);

	if (!sensor->direct_write)
		return -EOPNOTSUPP;

	return sensor->direct_write(sensor->context, buf, off, count);
}

static DEVICE_ATTR_RO(driver_name);
static DEVICE_ATTR_RO(port_name);
static DEVICE_ATTR_RW(poll_ms);
static DEVICE_ATTR_RO(fw_version);
static DEVICE_ATTR_RO(modes);
static DEVICE_ATTR_RW(mode);
static DEVICE_ATTR_RO(commands);
static DEVICE_ATTR_WO(command);
static DEVICE_ATTR_RO(units);
static DEVICE_ATTR_RO(decimals);
static DEVICE_ATTR_RO(num_values);
static DEVICE_ATTR_RO(bin_data_format);
static DEVICE_ATTR_RO(text_value);
/*
 * Technically, it is possible to have 32 8-bit values from UART sensors
 * and >200 8-bit values from I2C sensors, but known UART sensors so far
 * have 8 data values or less and I2C sensors can arbitrarily be split
 * into multiple modes, so we only expose 8 values to prevent sysfs
 * overcrowding.
 */
static DEVICE_ATTR(value0, S_IRUGO, value_show, NULL);
static DEVICE_ATTR(value1, S_IRUGO, value_show, NULL);
static DEVICE_ATTR(value2, S_IRUGO, value_show, NULL);
static DEVICE_ATTR(value3, S_IRUGO, value_show, NULL);
static DEVICE_ATTR(value4, S_IRUGO, value_show, NULL);
static DEVICE_ATTR(value5, S_IRUGO, value_show, NULL);
static DEVICE_ATTR(value6, S_IRUGO, value_show, NULL);
static DEVICE_ATTR(value7, S_IRUGO, value_show, NULL);

static struct attribute *lego_sensor_class_attrs[] = {
	&dev_attr_driver_name.attr,
	&dev_attr_port_name.attr,
	&dev_attr_poll_ms.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_modes.attr,
	&dev_attr_mode.attr,
	&dev_attr_commands.attr,
	&dev_attr_command.attr,
	&dev_attr_units.attr,
	&dev_attr_decimals.attr,
	&dev_attr_num_values.attr,
	&dev_attr_bin_data_format.attr,
	&dev_attr_text_value.attr,
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

static BIN_ATTR_RO(bin_data, LEGO_SENSOR_RAW_DATA_SIZE);
static BIN_ATTR_RW(direct, 255);

static struct bin_attribute *lego_sensor_class_bin_attrs[] = {
	&bin_attr_bin_data,
	&bin_attr_direct,
	NULL
};

static const struct attribute_group lego_sensor_class_group = {
	.attrs		= lego_sensor_class_attrs,
	.bin_attrs	= lego_sensor_class_bin_attrs,
};

static const struct attribute_group *lego_sensor_class_groups[] = {
	&lego_sensor_class_group,
	NULL
};

static void lego_sensor_release(struct device *dev)
{
}

static unsigned lego_sensor_class_id = 0;

int register_lego_sensor(struct lego_sensor_device *sensor,
			 struct device *parent)
{
	int err;

	if (!sensor || !sensor->port_name || !parent)
		return -EINVAL;

	sensor->dev.release = lego_sensor_release;
	sensor->dev.parent = parent;
	sensor->dev.class = &lego_sensor_class;
	dev_set_name(&sensor->dev, "sensor%d", lego_sensor_class_id++);

	err = device_register(&sensor->dev);
	if (err)
		return err;

	dev_info(&sensor->dev, "Registered '%s' on '%s'.\n", sensor->name,
		 sensor->port_name);

	return 0;
}
EXPORT_SYMBOL_GPL(register_lego_sensor);

void unregister_lego_sensor(struct lego_sensor_device *sensor)
{
	dev_info(&sensor->dev, "Unregistered '%s' on '%s'.\n", sensor->name,
		 sensor->port_name);
	device_unregister(&sensor->dev);
}
EXPORT_SYMBOL_GPL(unregister_lego_sensor);

static int lego_sensor_dev_uevent(struct device *dev,
				  struct kobj_uevent_env *env)
{
	struct lego_sensor_device *sensor = to_lego_sensor_device(dev);
	int ret;

	ret = add_uevent_var(env, "LEGO_DRIVER_NAME=%s", sensor->name);
	if (ret) {
		dev_err(dev, "failed to add uevent LEGO_DRIVER_NAME\n");
		return ret;
	}

	add_uevent_var(env, "LEGO_PORT_NAME=%s", sensor->port_name);
	if (ret) {
		dev_err(dev, "failed to add uevent LEGO_PORT_NAME\n");
		return ret;
	}

	return 0;
}

static char *lego_sensor_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "lego-sensor/%s", dev_name(dev));
}

struct class lego_sensor_class = {
	.name		= "lego-sensor",
	.owner		= THIS_MODULE,
	.dev_groups	= lego_sensor_class_groups,
	.dev_uevent	= lego_sensor_dev_uevent,
	.devnode	= lego_sensor_devnode,
};
EXPORT_SYMBOL_GPL(lego_sensor_class);

static int __init lego_sensor_class_init(void)
{
	int err;

	err = class_register(&lego_sensor_class);
	if (err) {
		pr_err("unable to register lego-sensor device class\n");
		return err;
	}

	return 0;
}
module_init(lego_sensor_class_init);

static void __exit lego_sensor_class_exit(void)
{
	class_unregister(&lego_sensor_class);
}
module_exit(lego_sensor_class_exit);

MODULE_DESCRIPTION("LEGO sensor device class");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
