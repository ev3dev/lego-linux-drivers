/*
 * Servo motor device class for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2014 David Lechner <david@lechnology.com>
 * Copyright (C) 2014 Ralph Hempel <rhemple@hempeldesigngroup.com>
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
 * Servo Motor Class
 *
* The `servo-motor` class provides a uniform interface for using [hobby type
* servo motors](https://en.wikipedia.org/wiki/Servo_%%28radio_control%%29).
* .
* ### sysfs Attributes
* .
* Servo motors can be found at `/sys/class/servo-motor/motor<N>`, where `<N>`
* is incremented each time a servo is loaded (it is not related to which port
* the motor is plugged in to).
* .
* `command` (write-only)
* : Sets the command for the servo. Valid values are `run` and `float`. Setting
*   to `run` will cause the servo to be driven to the position_sp set in the
*   `position_sp` attribute. Setting to `float` will remove power from the motor.
* .
 * `driver_name` (read-only)
 * : Returns the name of the motor driver that loaded this device. See the list
 *   of [supported devices] for a list of drivers.
* .
* `max_pulse_sp` (read/write)
* : Used to set the pulse size in milliseconds for the signal that tells the
*   servo to drive to the maximum (clockwise) position_sp. Default value is 2400.
*   Valid values are 2300 to 2700. You must write to the position_sp attribute for
*   changes to this attribute to take effect.
* .
* `mid_pulse_sp` (read/write)
* : Used to set the pulse size in milliseconds for the signal that tells the
*   servo to drive to the mid position_sp. Default value is 1500. Valid
*   values are 1300 to 1700. For example, on a 180 degree servo, this would be
*   90 degrees. On continuous rotation servo, this is the "neutral" position_sp
*   where the motor does not turn. You must write to the position_sp attribute for
*   changes to this attribute to take effect.
* .
* `min_pulse_sp` (read/write)
* : Used to set the pulse size in milliseconds for the signal that tells the
*   servo to drive to the miniumum (counter-clockwise) position_sp. Default value
*   is 600. Valid values are 300 to 700. You must write to the position_sp
*   attribute for changes to this attribute to take effect.
* .
* `polarity` (read/write)
* : Sets the polarity of the servo. Valid values are `normal` and `inverted`.
*   Setting the value to `inverted` will cause the position_sp value to be
*   inverted. i.e `-100` will correspond to `max_pulse_sp`, and `100` will
*   correspond to `min_pulse_sp`.
* .
* `port_name` (read-only)
* : Returns the name of the port that the motor is connected to.
* .
* `position_sp` (read/write)
* : Reading returns the current position_sp of the servo. Writing instructs the
*   servo to move to the specified position_sp. Units are percent. Valid values
*   are -100 to 100 (-100% to 100%) where `-100` corresponds to `min_pulse_sp`,
*   `0` corresponds to `mid_pulse_sp` and `100` corresponds to `max_pulse_sp`.
* .
* `rate_sp` (read/write)
* : Sets the rate_sp at which the servo travels from 0 to 100.0% (half of the full
*   range of the servo). Units are in milliseconds. Example: Setting the rate_sp
*   to 1000 means that it will take a 180 degree servo 2 second to move from 0
*   to 180 degrees. Note: Some servo controllers may not support this in which
*   case reading and writing will fail with `-EOPNOTSUPP`. In continuous rotation
*   servos, this value will affect the rate_sp at which the speed ramps up or down.
 * .
 * `state` (read-only)
 * : Returns a space separated list of flags indicating the state of the servo.
 *   Possible values are:
 *   * `running`: Indicates that the motor is powered.
 * .
 * [supported devices]: /docs/motors/#supported-devices
*/

#include <linux/device.h>
#include <linux/module.h>

#include <servo_motor_class.h>

const char *servo_motor_command_values[] = {
	[SERVO_MOTOR_COMMAND_RUN]	= "run",
	[SERVO_MOTOR_COMMAND_FLOAT]	= "float",
};

const char *servo_motor_polarity_values[] = {
	[SERVO_MOTOR_POLARITY_NORMAL]	= "normal",
	[SERVO_MOTOR_POLARITY_INVERTED]	= "inverted",
};

inline bool has_fixed_pulse_ms(struct servo_motor_device *sd)
{
	return (   (0 != sd->fixed_min_pulse_sp)
		|| (0 != sd->fixed_mid_pulse_sp)
		|| (0 != sd->fixed_max_pulse_sp) );
}

inline int servo_motor_class_scale(int in_min, int in_max,
				   int out_min, int out_max,
				   int value)
{
	long scaled = value - in_min;
	scaled *= out_max - out_min;
	scaled /= in_max - in_min;
	scaled += out_min;
	return scaled;
}

int servo_motor_class_get_command(struct servo_motor_device *motor)
{
	int ret;

	ret = motor->ops->get_position(motor->context);
	if (ret < 0)
		return ret;

	return ret ? SERVO_MOTOR_COMMAND_RUN : SERVO_MOTOR_COMMAND_FLOAT;
}

int servo_motor_class_set_position(struct servo_motor_device *motor,
				   int new_position,
				   enum servo_motor_polarity new_polarity)
{
	int scaled_position;

	motor->polarity = new_polarity;
	motor->position_sp = new_position;

	if (motor->command == SERVO_MOTOR_COMMAND_RUN) {
		if (new_polarity == SERVO_MOTOR_POLARITY_INVERTED)
			new_position = -new_position;
		if (new_position > 0)
			scaled_position = servo_motor_class_scale(0, 100,
				motor->mid_pulse_sp, motor->max_pulse_sp,
				new_position);
		else
			scaled_position = servo_motor_class_scale(-100, 0,
				motor->min_pulse_sp, motor->mid_pulse_sp,
				new_position);
		return motor->ops->set_position(motor->context, scaled_position);
	}
	return 0;
}

static ssize_t driver_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);

	return snprintf(buf, SERVO_MOTOR_NAME_SIZE, "%s\n", motor->name);
}

static ssize_t port_name_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);

	return snprintf(buf, SERVO_MOTOR_NAME_SIZE, "%s\n", motor->port_name);
}

static ssize_t min_pulse_sp_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);

	return sprintf(buf, "%d\n", motor->min_pulse_sp);
}

static ssize_t min_pulse_sp_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);
	int value;

	if (has_fixed_pulse_ms (motor))
		return -ENOSYS;

	if (sscanf(buf, "%d", &value) != 1 || value > 700 || value < 300)
		return -EINVAL;
	motor->min_pulse_sp = value;

	return count;
}

static ssize_t mid_pulse_sp_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);

	return sprintf(buf, "%d\n", motor->mid_pulse_sp);
}

static ssize_t mid_pulse_sp_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);
	int value;

	if (has_fixed_pulse_ms (motor))
		return -ENOSYS;

	if (sscanf(buf, "%d", &value) != 1 || value > 1700 || value < 1300)
		return -EINVAL;
	motor->mid_pulse_sp = value;

	return count;
}

static ssize_t max_pulse_sp_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);

	return snprintf(buf, SERVO_MOTOR_NAME_SIZE, "%d\n", motor->max_pulse_sp);
}

static ssize_t max_pulse_sp_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);
	unsigned value;

	if (has_fixed_pulse_ms (motor))
		return -ENOSYS;

	if (sscanf(buf, "%d", &value) != 1 || value > 2700 || value < 2300)
		return -EINVAL;
	motor->max_pulse_sp = value;

	return count;
}

static ssize_t command_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);
	int i, err;

	for (i = 0; i < NUM_SERVO_MOTOR_COMMAND; i++) {
		if (!sysfs_streq(buf, servo_motor_command_values[i]))
			continue;
		if (motor->command == i)
			return size;

		motor->command = i;
		if (motor->command == SERVO_MOTOR_COMMAND_RUN)
			err = servo_motor_class_set_position(motor, motor->position_sp,
							     motor->polarity);
		else
			err = motor->ops->set_position(motor->context, 0);
		if (err)
			return err;
		return size;
	}

	return -EINVAL;
}

static ssize_t polarity_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);

	return sprintf(buf, "%s\n", servo_motor_polarity_values[motor->polarity]);
}

static ssize_t polarity_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);
	int i, err;

	for (i = 0; i < NUM_SERVO_MOTOR_POLARITY; i++) {
		if (!sysfs_streq(buf, servo_motor_polarity_values[i]))
			continue;

		if (motor->polarity != i) {
			err = servo_motor_class_set_position(motor,
							     motor->position_sp, i);
			if (err)
				return err;
		}
		return size;
	}

	return -EINVAL;
}

static ssize_t position_sp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);
	int ret;

	ret = motor->ops->get_position(motor->context);

	if (ret < motor->mid_pulse_sp)
		ret =  servo_motor_class_scale(motor->min_pulse_sp,
			motor->mid_pulse_sp, -100, 0, ret);
	else
		ret = servo_motor_class_scale(motor->mid_pulse_sp,
			motor->max_pulse_sp, 0, 100, ret);
	return sprintf(buf, "%d\n", ret);
}

static ssize_t position_sp_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);
	int value, err;

	if (sscanf(buf, "%d", &value) != 1 || value > 100 || value < -100)
		return -EINVAL;
	if (motor->position_sp != value) {
		err = servo_motor_class_set_position(motor, value,
						     motor->polarity);
		if (err)
			return err;
	}

	return count;
}

static ssize_t rate_sp_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);
	int ret;

	if (!motor->ops->get_rate)
		return -EOPNOTSUPP;
	ret = motor->ops->get_rate(motor->context);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%d\n", ret);
}

static ssize_t rate_sp_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);
	unsigned value;
	int err;

	if (!motor->ops->set_rate)
		return -EOPNOTSUPP;

	if (sscanf(buf, "%ud", &value) != 1)
		return -EINVAL;
	err = motor->ops->set_rate(motor->context, value);
	if (err < 0)
		return err;

	return count;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct servo_motor_device *motor = to_servo_motor_device(dev);

	return sprintf(buf, "%s\n",
		motor->command == SERVO_MOTOR_COMMAND_RUN ? "running" : "");
}

static DEVICE_ATTR_RO(driver_name);
static DEVICE_ATTR_RO(port_name);
static DEVICE_ATTR_RW(min_pulse_sp);
static DEVICE_ATTR_RW(mid_pulse_sp);
static DEVICE_ATTR_RW(max_pulse_sp);
static DEVICE_ATTR_WO(command);
static DEVICE_ATTR_RW(polarity);
static DEVICE_ATTR_RW(position_sp);
static DEVICE_ATTR_RW(rate_sp);
static DEVICE_ATTR_RO(state);

static struct attribute *servo_motor_class_attrs[] = {
	&dev_attr_driver_name.attr,
	&dev_attr_port_name.attr,
	&dev_attr_min_pulse_sp.attr,
	&dev_attr_mid_pulse_sp.attr,
	&dev_attr_max_pulse_sp.attr,
	&dev_attr_command.attr,
	&dev_attr_polarity.attr,
	&dev_attr_position_sp.attr,
	&dev_attr_rate_sp.attr,
	&dev_attr_state.attr,
	NULL
};

static const struct attribute_group servo_motor_class_group = {
	.attrs		= servo_motor_class_attrs,
};

static const struct attribute_group *servo_motor_class_groups[] = {
	&servo_motor_class_group,
	NULL
};

static void servo_motor_release(struct device *dev)
{
}

static unsigned servo_motor_class_id = 0;
struct class servo_motor_class;

int register_servo_motor(struct servo_motor_device *servo, struct device *parent)
{
	int ret;

	if (!servo || !servo->port_name || !parent)
		return -EINVAL;

	servo->dev.release = servo_motor_release;
	servo->dev.parent = parent;
	servo->dev.class = &servo_motor_class;
	dev_set_name(&servo->dev, "motor%d", servo_motor_class_id++);

	if (has_fixed_pulse_ms (servo)) {
		servo->min_pulse_sp = servo->fixed_min_pulse_sp;
		servo->mid_pulse_sp = servo->fixed_mid_pulse_sp;
		servo->max_pulse_sp = servo->fixed_max_pulse_sp;
	} else {
		servo->min_pulse_sp = 600;
		servo->mid_pulse_sp = 1500;
		servo->max_pulse_sp = 2400;
	}

	ret = servo_motor_class_get_command(servo);
	if (ret < 0)
		return ret;
	servo->command = ret;

	ret = device_register(&servo->dev);
	if (ret)
		return ret;

	dev_info(&servo->dev, "Registered '%s' on '%s'.\n", servo->name,
		 servo->port_name);

	return 0;
}
EXPORT_SYMBOL_GPL(register_servo_motor);

void unregister_servo_motor(struct servo_motor_device *servo)
{
	dev_info(&servo->dev, "Unregistered '%s' on '%s'.\n", servo->name,
		 servo->port_name);
	device_unregister(&servo->dev);
}
EXPORT_SYMBOL_GPL(unregister_servo_motor);

static int servo_motor_dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct servo_motor_device *servo = to_servo_motor_device(dev);
	int ret;

	ret = add_uevent_var(env, "LEGO_DRIVER_NAME=%s", servo->name);
	if (ret) {
		dev_err(dev, "failed to add uevent LEGO_DRIVER_NAME\n");
		return ret;
	}
	add_uevent_var(env, "LEGO_PORT_NAME=%s", servo->port_name);
	if (ret) {
		dev_err(dev, "failed to add uevent LEGO_PORT_NAME\n");
		return ret;
	}

	return 0;
}

static char *servo_motor_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "servo-motor/%s", dev_name(dev));
}

struct class servo_motor_class = {
	.name		= "servo-motor",
	.owner		= THIS_MODULE,
	.dev_groups	= servo_motor_class_groups,
	.dev_uevent	= servo_motor_dev_uevent,
	.devnode	= servo_motor_devnode,
};
EXPORT_SYMBOL_GPL(servo_motor_class);

static int __init servo_motor_class_init(void)
{
	int err;

	err = class_register(&servo_motor_class);
	if (err) {
		pr_err("unable to register servo motor device class\n");
		return err;
	}

	return 0;
}
module_init(servo_motor_class_init);

static void __exit servo_motor_class_exit(void)
{
	class_unregister(&servo_motor_class);
}
module_exit(servo_motor_class_exit);

MODULE_DESCRIPTION("Servo motor device class for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
