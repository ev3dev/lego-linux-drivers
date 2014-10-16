/*
 * DC motor device class for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2014 David Lechner <david@lechnology.com>
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
 * DC Motor Class
 *
* The `dc_motor` class provides a uniform interface for using regular DC motors
* with no fancy controls or feedback. This includes the LEGO Power Functions
* motors.
* .
* ### sysfs Attributes
* .
* DC motors can be found at `/sys/class/dc-motor/motor<N>`, where `<N>`
* is incremented each time a motor is loaded (it is not related to which port
* the motor is plugged in to).
* .
* `command` (read/write)
* : Sets the command for the motor. Possible values are `forward`, `reverse`
*   `brake` and `coast`. Not all commands may be supported, so be sure to check
*   the contents of the `commands` attribute.
* .
* `commands` (read only)
* : Returns a space separated list of commands supported by the motor controller.
* .
* `duty_cycle` (read/write)
* : Sets the duty cycle of the PWM signal sent to the motor. Values are 0 to
*   1000 (0.0 to 100.0%).
* .
* `name` (read-only)
* : Returns the name of the dc controller's driver.
* .
* `port_name` (read-only)
* : Returns the name of the port that the motor is connected to.
* .
* `ramp_down_ms` (read/write)
* : Sets the time in milliseconds that it take the motor to ramp down from 100%
*   to 0%. If the controller does not support ramping, then reading and writing
*   will fail with -ENOSYS.
* .
* `ramp_up_ms` (read/write)
* : Sets the time in milliseconds that it take the motor to up ramp from 0% to
*   100%. If the controller does not support ramping, then reading and writing
*   will fail with -ENOSYS.
*/

#include <linux/device.h>
#include <linux/module.h>
#include <linux/legoev3/dc_motor_class.h>

const char* dc_motor_command_names[]  = {
	[DC_MOTOR_COMMAND_FORWARD]	= "forward",
	[DC_MOTOR_COMMAND_REVERSE]	= "reverse",
	[DC_MOTOR_COMMAND_COAST]	= "coast",
	[DC_MOTOR_COMMAND_BRAKE]	= "brake",
};

static ssize_t name_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return snprintf(buf, DC_MOTOR_NAME_SIZE, "%s\n", motor->name);
}

static ssize_t port_name_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return snprintf(buf, DC_MOTOR_NAME_SIZE, "%s\n", motor->port_name);
}

static ssize_t ramp_up_ms_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return sprintf(buf, "%u\n", motor->ramp_up_ms);
}

static ssize_t ramp_up_ms_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned value;

	if (sscanf(buf, "%ud", &value) != 1)
		return -EINVAL;
	motor->ramp_up_ms = value;
	/* TODO: need to implement ramping */

	return count;
}

static ssize_t ramp_down_ms_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);

	return sprintf(buf, "%u\n", motor->ramp_down_ms);
}

static ssize_t ramp_down_ms_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned value;

	if (sscanf(buf, "%ud", &value) != 1)
		return -EINVAL;
	motor->ramp_down_ms = value;
	/* TODO: need to implement ramping */

	return count;
}

static ssize_t duty_cycle_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	int duty_cycle;

	duty_cycle = motor->ops.get_duty_cycle(motor->ops.context);

	return sprintf(buf, "%d\n", duty_cycle);
}

static ssize_t duty_cycle_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned value;
	int err;

	if (sscanf(buf, "%ud", &value) != 1 || value > 1000)
		return -EINVAL;
	err = motor->ops.set_duty_cycle(motor->ops.context, value);
	if (err)
		return err;

	return count;
}

static ssize_t commands_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned supported_commands;
	int i;
	int count = 0;

	supported_commands = motor->ops.get_supported_commands(motor->ops.context);
	for (i = 0; i < NUM_DC_MOTOR_COMMANDS; i++) {
		if (supported_commands & BIT(i))
			count += sprintf(buf + count, "%s ",
				dc_motor_command_names[i]);
	}
	buf[count - 1] = '\n';
	return count;
}

static ssize_t command_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	int ret;

	ret = motor->ops.get_command(motor->ops.context);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%s\n", dc_motor_command_names[ret]);
}

static ssize_t command_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	unsigned supported_commands;
	int i, err;

	supported_commands = motor->ops.get_supported_commands(motor->ops.context);
	for (i = 0; i < NUM_DC_MOTOR_COMMANDS; i++) {
		if (sysfs_streq(buf, dc_motor_command_names[i])) {
			if (supported_commands & BIT(i)) {
				err = motor->ops.set_command(motor->ops.context,
					BIT(i));
				if (err)
					return err;
				return count;
			}
		}

	}
	return -EINVAL;
}

static DEVICE_ATTR_RO(name);
static DEVICE_ATTR_RO(port_name);
static DEVICE_ATTR_RW(ramp_up_ms);
static DEVICE_ATTR_RW(ramp_down_ms);
static DEVICE_ATTR_RW(duty_cycle);
static DEVICE_ATTR_RO(commands);
static DEVICE_ATTR_RW(command);

static struct attribute *dc_motor_class_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_port_name.attr,
	&dev_attr_ramp_up_ms.attr,
	&dev_attr_ramp_down_ms.attr,
	&dev_attr_duty_cycle.attr,
	&dev_attr_commands.attr,
	&dev_attr_command.attr,
	NULL
};

static const struct attribute_group dc_motor_class_group = {
	.attrs		= dc_motor_class_attrs,
};

static const struct attribute_group *dc_motor_class_groups[] = {
	&dc_motor_class_group,
	NULL
};

static void dc_motor_release(struct device *dev)
{
}

static unsigned dc_motor_class_id = 0;
struct class dc_motor_class;

int register_dc_motor(struct dc_motor_device *dc, struct device *parent)
{
	int err;

	if (!dc || !dc->port_name || !parent)
		return -EINVAL;

	dc->dev.release = dc_motor_release;
	dc->dev.parent = parent;
	dc->dev.class = &dc_motor_class;
	dev_set_name(&dc->dev, "motor%d", dc_motor_class_id++);

	err = device_register(&dc->dev);
	if (err)
		return err;

	dev_info(&dc->dev, "Bound to device '%s'\n", dev_name(parent));

	return 0;
}
EXPORT_SYMBOL_GPL(register_dc_motor);

void unregister_dc_motor(struct dc_motor_device *dc)
{
	dev_info(&dc->dev, "Unregistered\n");
	device_unregister(&dc->dev);
}
EXPORT_SYMBOL_GPL(unregister_dc_motor);

static int dc_motor_dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct dc_motor_device *motor = to_dc_motor_device(dev);
	int ret;

	ret = add_uevent_var(env, "NAME=%s", motor->name);
	if (ret) {
		dev_err(dev, "failed to add uevent DEVNAME\n");
		return ret;
	}
	add_uevent_var(env, "PORT=%s", motor->port_name);
	if (ret) {
		dev_err(dev, "failed to add uevent PORT\n");
		return ret;
	}

	return 0;
}

static char *dc_motor_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "dc-motor/%s", dev_name(dev));
}

struct class dc_motor_class = {
	.name		= "dc-motor",
	.owner		= THIS_MODULE,
	.dev_groups	= dc_motor_class_groups,
	.dev_uevent	= dc_motor_dev_uevent,
	.devnode	= dc_motor_devnode,
};
EXPORT_SYMBOL_GPL(dc_motor_class);

static int __init dc_motor_class_init(void)
{
	int err;

	err = class_register(&dc_motor_class);
	if (err) {
		pr_err("unable to register DC motor device class\n");
		return err;
	}

	return 0;
}
module_init(dc_motor_class_init);

static void __exit dc_motor_class_exit(void)
{
	class_unregister(&dc_motor_class);
}
module_exit(dc_motor_class_exit);

MODULE_DESCRIPTION("DC motor device class for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
