/*
 * Tacho motor device class
 *
 * Copyright (C) 2013-2014,2016 Ralph Hempel <rhempel@hempeldesigngroup.com>
 * Copyright (C) 2015-2016 David Lechner <david@lechnology.com>
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
 * Tacho Motor Class
 *
 * The `tacho-motor` class provides a uniform interface for using motors with
 * positional and directional feedback such as the EV3 and NXT motors. This
 * feedback allows for precise control of the motors.
 * .
 * ### sysfs Attributes
 * .
 * Tacho motors can be found at `/sys/class/tacho-motor/motor<N>`, where `<N>`
 * is incremented each time a motor is loaded (it is not related to which port
 * the motor is plugged in to).
 * .
 * `address`
 * : (read-only) Returns the name of the port that the motor is connected to.
 * .
 * `command`
 * : (write-only) Sends a command to the motor controller. Possible values are:
 * .
 * .    - `run-forever`: Causes the motor to run until another command is sent.
 * .    - `run-to-abs-pos`: Runs the motor to an absolute position specified by
 * .      `position_sp` and then stops the motor using the command specified in
 * .      `stop_action`.
 * .    - `run-to-rel-pos`: Runs the motor to a position relative to the current
 * .      `position` value. The new position will be current `position` +
 * .      `position_sp`. When the new position is reached, the motor will stop
 * .       using the command specified by `stop_action`.
 * .    - `run-timed`: Run the motor for the amount of time specified in
 * .      `time_sp` and then stops the motor using the command specified by
 * .      `stop_action`.
 * .    - `run-direct`: Runs the motor using the duty cycle specified by
 * .      `duty_cycle_sp`. Unlike other run commands, changing `duty_cycle_sp`
 * .       while running *will* take effect immediately.
 * .    - `stop`: Stop any of the run commands before they are complete using
 * .      the command specified by `stop_action`.
 * .    - `reset`: Resets all of the motor parameter attributes to their default
 * .       values. This will also have the effect of stopping the motor.
 * .
 * .    Not all commands may be supported. Read the `commands` attribute to get
 * .    the list of commands supported by a particular driver.
 * .
 * `commands`
 * : (read-only) Returns a space separated list of commands that are supported
 *   by the motor controller.
 * .
 * `count_per_rot`
 * : (read-only) Returns the number of tacho counts in one rotation of the
 *   motor. Tacho counts are used by the position and speed attributes, so you
 *   can use this value to convert from rotations or degrees to tacho counts.
 *   (rotation motors only)
 * .
 * `count_per_m`
 * : (read-only) Returns the number of tacho counts in one meter of travel
 *   of the motor. Tacho counts are used by the position and speed
 *   attributes, so you can use this value to convert from distance to tacho
 *   counts. (linear motors only)
 * .
 * `full_travel_count`
 * : (read-only) Returns the number of tacho counts in the full travel of
 *   the motor. When combined with the `count_per_m` atribute, you can use
 *   this value to calculate the maximum travel distance of the motor.
 *   (linear motors only)
 * .
 * `driver_name`
 * : (read-only) Returns the name of the driver that provides this tacho motor
 *   device.
 * .
 * `duty_cycle`
 * : (read-only) Returns the current duty cycle of the motor. Units are percent.
 *   Values are -100 to 100.
 * .
 * `duty_cycle_sp`
 * : (read/write) Writing sets the duty cycle setpoint. Reading returns the
 *   current value. Units are in percent. Valid values are -100 to 100. A
 *   negative value causes the motor to rotate in reverse.
 * .
 * `encoder_polarity`
 * : (read/write) Sets the polarity of the rotary encoder. This is an advanced
 *   feature to use motors that send inverted encoder signals to the EV3. This
 *   should be set correctly by the driver of a device. It You only need to
 *   change this value if you are using a unsupported device. Valid values are
 *   `normal` and `inversed`.
 * .
 * `polarity`
 * : (read/write) Sets the polarity of the motor. Valid values are:
 * .
 * .    - `normal`: A positive duty cycle will cause the motor to rotate
 * .      clockwise.
 * .    - `inversed`: A positive duty cycle will cause the motor to rotate
 * .       counter-clockwise.
 * .
 * `position`
 * : (read/write) Returns the current position of the motor in pulses of the
 *   rotary encoder. When the motor rotates clockwise, the position will
 *   increase. Likewise, rotating counter-clockwise causes the position to
 *   decrease. Writing will set the position to that value. The range is
 *   -2,147,483,648 and +2,147,483,647 tachometer counts (32-bit signed integer).
 * .
 * `hold_pid/Kd`
 * : (read/write) The derivative constant for the position PID.
 * .
 * `hold_pid/Ki`
 * : (read/write) The integral constant for the position PID.
 * .
 * `hold_pid/Kp`
 * : (read/write) The proportional constant for the position PID.
 * .
 * `max_speed`
 * : (read) Returns the maximum value that is accepted by the `speed_sp`
 *   attribute. This may be slightly different than the maximum speed that
 *   a particular motor can reach - it's the maximum theoretical speed.
 * .
 * `position_sp`
 * : (read/write) Writing specifies the target position for the `run-to-abs-pos`
 *   and `run-to-rel-pos` commands. Reading returns the current value. Units are
 *   in tacho counts. You can use the value returned by `counts_per_rot` to
 *   convert tacho counts to/from rotations or degrees. The range is
 *   -2,147,483,648 and +2,147,483,647 tachometer counts (32-bit signed integer).
 * .
 * `speed`
 * : (read-only) Returns the current motor speed in tacho counts per second.
 *   Note, this is not necessarily degrees (although it is for LEGO motors).
 *   Use the `count_per_rot` attribute to convert this value to RPM or deg/sec.
 * .
 * `speed_sp`
 * : (read/write) Writing sets the target speed in tacho counts per second used
 *   for all `run-*` commands except `run-direct`. Reading returns the current
 *   value. A negative value causes the motor to rotate in reverse with the
 *   exception of `run-to-*-pos` commands where the sign is ignored. Use the
 *   `count_per_rot` attribute to convert RPM or deg/sec to tacho counts per
 *   second. Use the `count_per_m` attribute to convert m/s to tacho counts per
 *   second.
 * .
 * `ramp_up_sp`
 * : (read/write) Writing sets the ramp up setpoint. Reading returns the
 *   current value. Units are in milliseconds and must be positive. When set
 *   to a non-zero value, the motor speed will increase from 0 to 100% of
 *   `max_speed` over the span of this setpoint. The actual ramp time is the
 *   ratio of the difference between the `speed_sp` and the current `speed`
 *   and max_speed multiplied by `ramp_up_sp`.
 * .
 * `ramp_down_sp`
 * : (read/write) Writing sets the ramp down setpoint. Reading returns the
 *   current value. Units are in milliseconds and must be positive. When set
 *   to a non-zero value, the motor speed will decrease from 0 to 100% of
 *   `max_speed` over the span of this setpoint. The actual ramp time is the
 *   ratio of the difference between the `speed_sp` and the current `speed`
 *   and max_speed multiplied by `ramp_down_sp`.
 * .
 * `speed_pid/Kd`
 * : (read/write) The derivative constant for the speed regulation PID.
 * .
 * `speed_pid/Ki`
 * : (read/write) The integral constant for the speed regulation PID.
 * .
 * `speed_pid/Kp`
 * : (read/write) The proportional constant for the speed regulation PID.
 * .
 * `state`
 * : (read-only) Reading returns a space separated list of state flags.
 *   Possible flags are:
 * .
 * .    - `running`: Power is being sent to the motor.
 * .    - `ramping`: The motor is ramping up or down and has not yet reached a
 * .      constant output level.
 * .    - `holding`: The motor is not turning, but rather attempting to hold a
 * .      fixed position.
 * .    - `overloaded`: The motor is turning, but cannot reach its `speed_sp`.
 * .    - `stalled`: The motor is not turning when it should be.
 * .
 * `stop_action`
 * : (read/write) Reading returns the current stop action. Writing sets the
 *   stop action. The value determines the motors behavior when `command` is
 *   set to `stop`. Possible values are:
 * .
 * .    - `coast`: Removes power from the motor. The motor will freely coast to
 * .       a stop.
 * .    - `brake`: Removes power from the motor and creates a passive electrical
 * .       load. This is usually done by shorting the motor terminals together.
 * .       This load will absorb the energy from the rotation of the motors and
 * .       cause the motor to stop more quickly than coasting.
 * .    - `hold`: Causes the motor to actively try to hold the current position.
 * .       If an external force tries to turn the motor, the motor will "push
 * .       back" to maintain its position.
 * .
 * .    Not all actions may be supported. Read `stop_actions` to get the
 * .    actions available for a particular driver.
 * .
 * `stop_actions`
 * : (read-only) Returns a space-separated list of stop actions supported by the
 *   motor controller.
 * .
 * `time_sp`
 * : (read/write) Writing specifies the amount of time the motor will run when
 *   using the `run-timed` command. Reading returns the current value. Units
 *   are in milliseconds.
 */

#include <linux/device.h>
#include <linux/module.h>

#include <dc_motor_class.h>
#include <tacho_motor_class.h>

#include "ev3_motor.h"

#define RAMP_PERIOD	msecs_to_jiffies(100)

struct tacho_motor_value_names {
	const char *name;
};

static struct tacho_motor_value_names
tm_stop_action_names[NUM_TM_STOP_ACTION] = {
	[TM_STOP_ACTION_COAST]     =  { "coast" },
	[TM_STOP_ACTION_BRAKE]     =  { "brake" },
	[TM_STOP_ACTION_HOLD]      =  { "hold" },
};

static struct tacho_motor_value_names
tacho_motor_command_names[NUM_TM_COMMAND] = {
	[TM_COMMAND_RUN_FOREVER]	= { "run-forever" },
	[TM_COMMAND_RUN_TO_ABS_POS]	= { "run-to-abs-pos" },
	[TM_COMMAND_RUN_TO_REL_POS]	= { "run-to-rel-pos" },
	[TM_COMMAND_RUN_TIMED]		= { "run-timed" },
	[TM_COMMAND_RUN_DIRECT]		= { "run-direct" },
	[TM_COMMAND_STOP]		= { "stop" },
	[TM_COMMAND_RESET]		= { "reset" },
};

struct tacho_motor_type_item {
	const char *name;
};

struct tacho_motor_state_item {
	const char *name;
};

static struct tacho_motor_value_names tacho_motor_states[NUM_TM_STATE] = {
	[TM_STATE_RUNNING]	= { "running" },
	[TM_STATE_RAMPING]	= { "ramping" },
	[TM_STATE_HOLDING]	= { "holding" },
	[TM_STATE_OVERLOADED]	= { "overloaded" },
	[TM_STATE_STALLED]	= { "stalled" },
};

inline struct tacho_motor_device *to_tacho_motor(struct device *dev)
{
	return container_of(dev, struct tacho_motor_device, dev);
}

static int tm_do_one_ramp_step(struct tacho_motor_device *tm,
			       struct tacho_motor_params *params);

static int tacho_motor_class_start_motor_ramp(struct tacho_motor_device *tm,
					      struct tacho_motor_params *params)
{
	unsigned long ramp_sp, now;

	/* Determine if the target and current speed require a
	 * transition through the 0 setpoint. If yes, then we need
	 * to divide the ramp work into two pieces.
	 */

	if (!tm->ramping)
		tm->ops->get_speed(tm->context, &tm->ramp_last_speed);
	tm->ramp_start_speed = tm->ramp_last_speed;

	if (0 > (tm->ramp_start_speed * params->speed_sp))
		tm->ramp_end_speed = 0;
	else
		tm->ramp_end_speed = params->speed_sp;

	tm->ramp_delta_speed = tm->ramp_end_speed - tm->ramp_start_speed;

	/* Now determine if this leg of the ramp needs to increase or
	 * decrease the speed, and use the corresponding ramp_*_sp
	 */

	if (0 <= (tm->ramp_start_speed * tm->ramp_delta_speed))
		ramp_sp = msecs_to_jiffies(params->ramp_up_sp);
	else
		ramp_sp = msecs_to_jiffies(params->ramp_down_sp);

	tm->ramp_delta_time = (ramp_sp * abs(tm->ramp_delta_speed))
				/ tm->info->max_speed;

	/* Set the start time to about half a RAMP_PERIOD in the past so
	 * that the first expiry starts the motor running at a non-zero
	 * speed
	 */

	now = jiffies - RAMP_PERIOD/2;
	tm->ramp_end_time = now + tm->ramp_delta_time;
	tm->last_ramp_work_time = now;
	tm->ramping = true;

	return tm_do_one_ramp_step(tm, params);
}

static void tacho_motor_class_ramp_work(struct work_struct *work)
{
	struct tacho_motor_device *tm = container_of(to_delayed_work(work),
					struct tacho_motor_device, ramp_work);
	int err;

	err = tm_do_one_ramp_step(tm, &tm->active_params);
	WARN_ONCE(err, "Ramp failed.");
}

static int tm_do_one_ramp_step(struct tacho_motor_device *tm,
			       struct tacho_motor_params *params)
{
	unsigned long remaining_ramp_time = 0;
	unsigned long last_ramp_time;
	int err;

	/* Check to see if we are running and done ramping, or if we're
	 * running and have hit the 0 crossover point and need to restart
	 * the ramp
	 */

	if (params->speed_sp == tm->ramp_last_speed) {
		tm->ramping = false;
		if (tm->command == TM_COMMAND_STOP)
			return tm->ops->stop(tm->context,
					     params->stop_action);
		return 0;
	} else if (tm->ramp_end_speed == tm->ramp_last_speed)
		return tacho_motor_class_start_motor_ramp(tm, params);

	/*
	 * If we haven't reached the end of the ramp yet, set the speed
	 * to the appropriate point along the ramp, otherwise set the speed
	 * directly to the setpoint. Note that we need to take into account
	 * the fact that the target setpoint may cross the 0 point.
	 */

	if (time_is_after_jiffies(tm->ramp_end_time))
		remaining_ramp_time = tm->ramp_end_time - jiffies;
	else
		remaining_ramp_time = 0;

	if (tm->ramp_delta_time != 0) {
		tm->ramp_last_speed = tm->ramp_end_speed
			- ((tm->ramp_delta_speed * (int)remaining_ramp_time)
				/ tm->ramp_delta_time);
	} else {
		tm->ramp_last_speed = tm->ramp_end_speed;
		tm->ramping = false;
	}

	if (IS_POS_CMD(tm->command))
		err = tm->ops->run_to_pos(tm->context, params->position_sp,
			tm->ramp_last_speed, params->stop_action);
	else
		err = tm->ops->run_regulated(tm->context, tm->ramp_last_speed);
	if (err)
		return err;

	/*
	 * Measure how long it took since the last call to ramp_work and
	 * schedule the next ramp as close to RAMP_PERIOD as we can get
	 */
	last_ramp_time = jiffies - tm->last_ramp_work_time;
	tm->last_ramp_work_time = jiffies;
	queue_delayed_work(tm->wq, &tm->ramp_work, last_ramp_time >= RAMP_PERIOD
		? 0 : RAMP_PERIOD - last_ramp_time);

	return 0;
}

/*
 * Set all parameters to the default values.
 */
void tacho_motor_class_reset(struct tacho_motor_device *tm)
{
	tm->params.polarity		= DC_MOTOR_POLARITY_NORMAL;
	tm->params.encoder_polarity	= tm->info->encoder_polarity;
	tm->params.duty_cycle_sp	= 0;
	tm->params.speed_sp		= 0;
	tm->params.position_sp		= 0;
	tm->params.time_sp		= 0;
	tm->params.ramp_up_sp		= 0;
	tm->params.ramp_down_sp		= 0;
	tm->params.stop_action		= TM_STOP_ACTION_COAST;
}

static ssize_t address_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return snprintf(buf, LEGO_NAME_SIZE, "%s\n", tm->address);
}

static ssize_t driver_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%s\n", tm->driver_name);
}

static ssize_t position_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	long position;
	int err;

	err = tm->ops->get_position(tm->context, &position);
	if (err < 0)
		return err;

	return sprintf(buf, "%ld\n", position);
}

ssize_t position_store(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, position;

	err = kstrtoint(buf, 10, &position);
	if (err < 0)
		return err;

	err = tm->ops->set_position(tm->context, position);
	if (err < 0)
		return err;

	return size;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int ret, i;
	size_t size = 0;

	ret = tm->ops->get_state(tm->context);
	if (ret < 0)
		return ret;

	if (tm->ramping)
		ret |= BIT(TM_STATE_RAMPING);

	for (i = 0; i < NUM_TM_STATE; i++) {
		if (ret & BIT(i))
			size += sprintf(buf + size, "%s ", tacho_motor_states[i].name);
	}
	if (size == 0)
		size = sprintf(buf, "\n");
	else
		buf[size - 1] = '\n';

	return size;
}

static ssize_t count_per_rot_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->info->count_per_rot);
}

static ssize_t count_per_m_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->info->count_per_m);
}

static ssize_t full_travel_count_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->info->full_travel_count);
}

static ssize_t duty_cycle_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, duty_cycle;

	if (!tm->ops->get_duty_cycle)
		return -EOPNOTSUPP;

	err = tm->ops->get_duty_cycle(tm->context, &duty_cycle);
	if (err)
		return err;

	if (tm->active_params.polarity == DC_MOTOR_POLARITY_INVERSED)
		duty_cycle *= -1;

	return sprintf(buf, "%d\n", duty_cycle);
}

static ssize_t max_speed_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->info->max_speed);
}

static ssize_t speed_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, speed;

	if (!tm->ops->get_speed)
		return -EOPNOTSUPP;

	err = tm->ops->get_speed(tm->context, &speed);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n", speed);
}

static unsigned get_supported_commands(struct tacho_motor_device *tm)
{
	unsigned supported_commands = 0;

	if (tm->ops->run_unregulated)
		supported_commands |= BIT(TM_COMMAND_RUN_DIRECT);
	if (tm->ops->run_regulated)
		supported_commands |= BIT(TM_COMMAND_RUN_FOREVER);
	if (tm->ops->run_to_pos) {
		supported_commands |= BIT(TM_COMMAND_RUN_TO_ABS_POS);
		supported_commands |= BIT(TM_COMMAND_RUN_TO_REL_POS);
	}
	if (tm->ops->run_regulated && tm->ops->stop)
		supported_commands |= BIT(TM_COMMAND_RUN_TIMED);
	if (tm->ops->stop)
		supported_commands |= BIT(TM_COMMAND_STOP);
	if (tm->ops->reset)
		supported_commands |= BIT(TM_COMMAND_RESET);

	return supported_commands;
}

static ssize_t commands_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	unsigned supported_commands;
	int i;
	int size = 0;

	supported_commands = get_supported_commands(tm);
	for (i = 0; i < NUM_TM_COMMAND; i++) {
		if (supported_commands & BIT(i))
			size += sprintf(buf+size, "%s ",
				tacho_motor_command_names[i].name);
	}

	buf[size - 1] = '\n';

	return size;
}

static int tm_send_command(struct tacho_motor_device *tm,
			   enum tacho_motor_command cmd)
{
	struct tacho_motor_params new_params;
	int err;
	bool ramp = false;

	/* stop any previous async commands */
	cancel_delayed_work_sync(&tm->run_timed_work);
	cancel_delayed_work_sync(&tm->ramp_work);

	/* do any extra manipulation of params if needed */

	if (cmd == TM_COMMAND_RESET)
		tacho_motor_class_reset(tm);

	new_params = tm->params;

	if (!IS_RUN_CMD(cmd)) {
		new_params.duty_cycle_sp = 0;
		new_params.speed_sp = 0;
	}

	if (cmd == TM_COMMAND_RUN_TO_REL_POS) {
		/*
		 * To check the previous command, we MUST be looking at
		 * tm->command because that's the previous command that the
		 * user sent! If the previous command was also run-to-rel-pos
		 * then we try to be "smart" about the new setpoint by making
		 * it relative to the previous setpoint. This will eliminate
		 * cumulative error due to the motor not holding the position
		 * exactly when the position setpoint is reached.
		 */
		if (tm->command == TM_COMMAND_RUN_TO_REL_POS) {
			new_params.position_sp = tm->active_params.position_sp +
							tm->params.position_sp;
		} else {
			long position;

			tm->ops->get_position(tm->context, &position);
			new_params.position_sp = position +
							tm->params.position_sp;
		}
	}

	/* now actually send the command to the motor controller */

	switch (cmd) {
	case TM_COMMAND_RUN_DIRECT:
		err = tm->ops->run_unregulated(tm->context,
					       new_params.duty_cycle_sp);
		break;
	case TM_COMMAND_RUN_FOREVER:
	case TM_COMMAND_RUN_TIMED:
		if (new_params.ramp_up_sp || new_params.ramp_down_sp)
			ramp = true;
		else
			err = tm->ops->run_regulated(tm->context,
						     new_params.speed_sp);
		break;
	case TM_COMMAND_RUN_TO_ABS_POS:
	case TM_COMMAND_RUN_TO_REL_POS:
		if (new_params.ramp_up_sp || new_params.ramp_down_sp)
			ramp = true;
		else
			err = tm->ops->run_to_pos(tm->context,
						  new_params.position_sp,
						  new_params.speed_sp,
						  new_params.stop_action);
		break;
	case TM_COMMAND_STOP:
		if (new_params.ramp_up_sp || new_params.ramp_down_sp)
			ramp = true;
		else
			err = tm->ops->stop(tm->context,
					    new_params.stop_action);
		break;
	case TM_COMMAND_RESET:
		err = tm->ops->reset(tm->context);
		break;
	default:
		BUG();
	}
	if (ramp)
		err = tacho_motor_class_start_motor_ramp(tm, &new_params);
	if (err < 0)
		return err;

	tm->active_params = new_params;
	tm->command = cmd;

	if (cmd == TM_COMMAND_RUN_TIMED)
		queue_delayed_work(tm->wq, &tm->run_timed_work,
				   msecs_to_jiffies(new_params.time_sp));

	return 0;
}

static ssize_t command_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	unsigned supported_commands;
	int i;

	supported_commands = get_supported_commands(tm);

	for (i = 0; i < NUM_TM_COMMAND; i++) {
		if (!sysfs_streq(buf, tacho_motor_command_names[i].name))
			continue;

		if (supported_commands & BIT(i)) {
			int err = tm_send_command(tm, i);

			return err < 0 ? err : size;
		}
	}

	return -EINVAL;
}

static void tacho_motor_class_run_timed_work(struct work_struct *work)
{
	struct tacho_motor_device *tm = container_of(to_delayed_work(work),
				struct tacho_motor_device, run_timed_work);

	tm->command = TM_COMMAND_STOP;
	if (tm->active_params.ramp_down_sp) {
		tm->active_params.speed_sp = 0;
		tacho_motor_class_start_motor_ramp(tm, &tm->active_params);
	} else
		tm->ops->stop(tm->context, tm->active_params.stop_action);
}

static ssize_t stop_actions_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	unsigned commands;
	int i;
	int size = 0;

	commands = tm->ops->get_stop_actions(tm->context);

	for (i = 0; i < NUM_TM_STOP_ACTION; i++) {
		if (commands & BIT(i)) {
			size += sprintf(buf + size, "%s ",
				tm_stop_action_names[i].name);
		}
	}

	buf[size - 1] = '\n';

	return size;
}

static ssize_t stop_action_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%s\n",
		       tm_stop_action_names[tm->params.stop_action].name);
}

static ssize_t stop_action_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	unsigned i;

	for (i = 0; i < NUM_TM_STOP_ACTION; i++) {
		if (sysfs_streq(buf, tm_stop_action_names[i].name))
			break;
	}

	if (i >= NUM_TM_STOP_ACTION)
		return -EINVAL;

	if (!(BIT(i) & tm->ops->get_stop_actions(tm->context)))
		return -EINVAL;

	tm->params.stop_action = i;

	return size;
}

static ssize_t polarity_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%s\n", dc_motor_polarity_values[tm->params.polarity]);
}

static ssize_t polarity_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	unsigned i;

	for (i = 0; i < NUM_DC_MOTOR_POLARITY; i++) {
		if (sysfs_streq(buf, dc_motor_polarity_values[i])) {
			tm->params.polarity = i;
			return size;
		}
	}

	return -EINVAL;
}

static ssize_t encoder_polarity_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%s\n", dc_motor_polarity_values[tm->params.encoder_polarity]);
}

static ssize_t encoder_polarity_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	unsigned i;

	if (!tm->supports_encoder_polarity)
		return -EOPNOTSUPP;

	for (i = 0; i < NUM_DC_MOTOR_POLARITY; i++) {
		if (sysfs_streq(buf, dc_motor_polarity_values[i])) {
			tm->params.encoder_polarity = i;
			return size;
		}
	}

	return -EINVAL;
}

static ssize_t ramp_up_sp_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	if (!SUPPORTS_RAMPING(tm))
		return -EOPNOTSUPP;

	return sprintf(buf, "%d\n", tm->params.ramp_up_sp);
}

static ssize_t ramp_up_sp_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, ms;

	if (!SUPPORTS_RAMPING(tm))
		return -EOPNOTSUPP;

	err = kstrtoint(buf, 10, &ms);
	if (err < 0)
		return err;

	if (ms < 0 || ms > 60000)
		return -EINVAL;

	tm->params.ramp_up_sp = ms;

	return size;
}

static ssize_t ramp_down_sp_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	if (!SUPPORTS_RAMPING(tm))
		return -EOPNOTSUPP;

	return sprintf(buf, "%d\n", tm->params.ramp_down_sp);
}

static ssize_t ramp_down_sp_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, ms;

	if (!SUPPORTS_RAMPING(tm))
		return -EOPNOTSUPP;

	err = kstrtoint(buf, 10, &ms);
	if (err < 0)
		return err;

	if (ms < 0 || ms > 60000)
		return -EINVAL;

	tm->params.ramp_down_sp = ms;

	return size;
}

static ssize_t duty_cycle_sp_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->params.duty_cycle_sp);
}

static ssize_t duty_cycle_sp_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, duty_cycle;

	err = kstrtoint(buf, 10, &duty_cycle);
	if (err < 0)
		return err;

	if (duty_cycle < -DC_MOTOR_MAX_DUTY_CYCLE
		|| duty_cycle > DC_MOTOR_MAX_DUTY_CYCLE)
		return -EINVAL;

	/* If we're in run-direct mode, allow the duty_cycle_sp to change */
	if (tm->command == TM_COMMAND_RUN_DIRECT) {
		err = tm->ops->run_unregulated(tm->context, duty_cycle);
		if (err < 0)
			return err;
		tm->active_params.duty_cycle_sp = duty_cycle;
	}

	tm->params.duty_cycle_sp = duty_cycle;

	return size;
}

static ssize_t speed_sp_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->params.speed_sp);
}

static ssize_t speed_sp_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, speed;

	err = kstrtoint(buf, 10, &speed);
	if (err < 0)
		return err;

	if (abs(speed) > tm->info->max_speed)
		return -EINVAL;

	tm->params.speed_sp = speed;

	return size;
}

static ssize_t time_sp_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->params.time_sp);
}

static ssize_t time_sp_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err, time;

	if (!(BIT(TM_COMMAND_RUN_TIMED) & get_supported_commands(tm)))
		return -EOPNOTSUPP;

	err = kstrtoint(buf, 10, &time);
	if (err < 0)
		return err;

	if (time < 0)
		return -EINVAL;

	tm->params.time_sp = time;

	return size;
}

static ssize_t position_sp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);

	return sprintf(buf, "%d\n", tm->params.position_sp);
}

static ssize_t position_sp_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int err;
	long position;

	if (!((BIT(TM_COMMAND_RUN_TO_ABS_POS) | BIT(TM_COMMAND_RUN_TO_REL_POS))
						& get_supported_commands(tm))) {
		return -EOPNOTSUPP;
	}

	err = kstrtol(buf, 10, &position);
	if (err < 0)
		return err;

	tm->params.position_sp = position;

	return size;
}

static DEVICE_ATTR_RO(driver_name);
static DEVICE_ATTR_RO(address);
static DEVICE_ATTR_RW(position);
static DEVICE_ATTR_RO(state);
static DEVICE_ATTR_RO(count_per_rot);
static DEVICE_ATTR_RO(count_per_m);
static DEVICE_ATTR_RO(full_travel_count);
static DEVICE_ATTR_RO(duty_cycle);
static DEVICE_ATTR_RO(speed);
static DEVICE_ATTR_RO(max_speed);
static DEVICE_ATTR_RW(duty_cycle_sp);
static DEVICE_ATTR_RW(speed_sp);
static DEVICE_ATTR_RW(time_sp);
static DEVICE_ATTR_RW(position_sp);
static DEVICE_ATTR_RO(commands);
static DEVICE_ATTR_WO(command);
static DEVICE_ATTR_RO(stop_actions);
static DEVICE_ATTR_RW(stop_action);
static DEVICE_ATTR_RW(polarity);
static DEVICE_ATTR_RW(encoder_polarity);
static DEVICE_ATTR_RW(ramp_up_sp);
static DEVICE_ATTR_RW(ramp_down_sp);

static struct attribute *tacho_motor_class_attrs[] = {
	&dev_attr_driver_name.attr,
	&dev_attr_address.attr,
	&dev_attr_position.attr,
	&dev_attr_state.attr,
	&dev_attr_duty_cycle.attr,
	&dev_attr_speed.attr,
	&dev_attr_duty_cycle_sp.attr,
	&dev_attr_speed_sp.attr,
	&dev_attr_max_speed.attr,
	&dev_attr_time_sp.attr,
	&dev_attr_position_sp.attr,
	&dev_attr_commands.attr,
	&dev_attr_command.attr,
	&dev_attr_stop_actions.attr,
	&dev_attr_stop_action.attr,
	&dev_attr_polarity.attr,
	&dev_attr_encoder_polarity.attr,
	&dev_attr_ramp_up_sp.attr,
	&dev_attr_ramp_down_sp.attr,
	NULL
};

static const struct attribute_group tacho_motor_class_group = {
	.attrs = tacho_motor_class_attrs,
};

/* Note - this group of attributes is only created for rotating motors */

static struct attribute *tacho_motor_rotation_attrs[] = {
	&dev_attr_count_per_rot.attr,
	NULL
};

static const struct attribute_group tacho_motor_rotation_group = {
	.attrs = tacho_motor_rotation_attrs,
};

/* Note - this group of attributes is only created for linear motors */

static struct attribute *tacho_motor_linear_attrs[] = {
	&dev_attr_count_per_m.attr,
	&dev_attr_full_travel_count.attr,
	NULL
};

static const struct attribute_group tacho_motor_linear_group = {
	.attrs = tacho_motor_linear_attrs,
};

#define PID_ATTR_FUNCS(name)					\
static ssize_t name##_show(struct device *dev,			\
			   struct device_attribute *attr,	\
			   char *buf)				\
{								\
	struct tacho_motor_device *tm = to_tacho_motor(dev);	\
	int ret;						\
								\
	if (!tm->ops->get_##name)				\
		return -EOPNOTSUPP;				\
								\
	ret = tm->ops->get_##name(tm->context);			\
	if (ret < 0)						\
		return ret;					\
								\
	return sprintf(buf, "%d\n", ret);			\
}								\
								\
static ssize_t name##_store(struct device *dev,			\
			    struct device_attribute *attr,	\
			    const char *buf, size_t size)	\
{								\
	struct tacho_motor_device *tm = to_tacho_motor(dev);	\
	int err, k;						\
								\
	if (!tm->ops->set_##name)				\
		return -EOPNOTSUPP;				\
								\
	err = kstrtoint(buf, 10, &k);				\
	if (err < 0)						\
		return err;					\
								\
	if (k < 0)						\
		return -EINVAL;					\
								\
	err = tm->ops->set_##name(tm->context, k);		\
	if (err < 0)						\
		return err;					\
								\
	return size;						\
}

#define PID_ATTR(pid, k)					\
		struct device_attribute dev_attr_##pid##_##k =	\
		__ATTR(k, S_IWUSR | S_IRUGO, pid##_##k##_show,	\
					     pid##_##k##_store)

#define PID_ATTR_GROUP(pid)					\
PID_ATTR_FUNCS(pid##_Kp)					\
PID_ATTR_FUNCS(pid##_Ki)					\
PID_ATTR_FUNCS(pid##_Kd)					\
								\
PID_ATTR(pid, Kp);						\
PID_ATTR(pid, Ki);						\
PID_ATTR(pid, Kd);						\
								\
static struct attribute *tacho_motor_##pid##_pid_attrs[] = {	\
	&dev_attr_##pid##_Kp.attr,				\
	&dev_attr_##pid##_Ki.attr,				\
	&dev_attr_##pid##_Kd.attr,				\
	NULL							\
};								\
								\
static const struct attribute_group				\
	 tacho_motor_##pid##_pid_group = {			\
		.name = __stringify(pid) "_pid",		\
		.attrs = tacho_motor_##pid##_pid_attrs,		\
}

PID_ATTR_GROUP(speed);
PID_ATTR_GROUP(hold);

static const struct attribute_group *tacho_motor_rotation_groups[] = {
	&tacho_motor_class_group,
	&tacho_motor_rotation_group,
	&tacho_motor_speed_pid_group,
	&tacho_motor_hold_pid_group,
	NULL
};

static const struct attribute_group *tacho_motor_linear_groups[] = {
	&tacho_motor_class_group,
	&tacho_motor_linear_group,
	&tacho_motor_speed_pid_group,
	&tacho_motor_hold_pid_group,
	NULL
};

static void tacho_motor_release(struct device *dev)
{
}

void tacho_motor_notify_position_ramp_down(struct tacho_motor_device *tm)
{
	cancel_delayed_work_sync(&tm->run_timed_work);
	cancel_delayed_work_sync(&tm->ramp_work);

	tm->active_params.speed_sp = 0;
	tm->ramp_last_speed = 0;
}
EXPORT_SYMBOL_GPL(tacho_motor_notify_position_ramp_down);

void tacho_motor_notify_state_change(struct tacho_motor_device *tm)
{
	sysfs_notify(&tm->dev.kobj, NULL, "state");
}
EXPORT_SYMBOL_GPL(tacho_motor_notify_state_change);

static unsigned tacho_motor_class_id = 0;

int register_tacho_motor(struct tacho_motor_device *tm, struct device *parent)
{
	int err;

	if (!tm || !tm->address || !tm->info || !parent)
		return -EINVAL;

	tm->wq = create_singlethread_workqueue("tacho-motor");
	if (!tm->wq)
		return -ENOMEM;

	tm->dev.release = tacho_motor_release;
	tm->dev.parent = parent;

	switch (tm->info->motion_type) {
	case TM_MOTION_LINEAR:
		tacho_motor_class.dev_groups = tacho_motor_linear_groups;
		dev_set_name(&tm->dev, "linear%d", tacho_motor_class_id++);
		break;
	case TM_MOTION_ROTATION:
		tacho_motor_class.dev_groups = tacho_motor_rotation_groups;
		dev_set_name(&tm->dev, "motor%d", tacho_motor_class_id++);
		break;
	default:
		dev_err(&tm->dev, "unhandled motion type\n");
		destroy_workqueue(tm->wq);
		return -EINVAL;
	}

	tm->dev.class = &tacho_motor_class;

	tacho_motor_class_reset(tm);

	INIT_DELAYED_WORK(&tm->ramp_work, tacho_motor_class_ramp_work);
	INIT_DELAYED_WORK(&tm->run_timed_work, tacho_motor_class_run_timed_work);

	err = device_register(&tm->dev);
	if (err) {
		destroy_workqueue(tm->wq);
		return err;
	}

	dev_info(&tm->dev, "Registered '%s' on '%s'.\n", tm->driver_name,
		 tm->address);

	return 0;
}
EXPORT_SYMBOL_GPL(register_tacho_motor);

void unregister_tacho_motor(struct tacho_motor_device *tm)
{
	dev_info(&tm->dev, "Unregistered '%s' on '%s'.\n", tm->driver_name,
		 tm->address);
	cancel_delayed_work_sync(&tm->run_timed_work);
	cancel_delayed_work_sync(&tm->ramp_work);
	destroy_workqueue(tm->wq);
	device_unregister(&tm->dev);
}
EXPORT_SYMBOL_GPL(unregister_tacho_motor);

static int tacho_motor_dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct tacho_motor_device *tm = to_tacho_motor(dev);
	int ret;

	ret = add_uevent_var(env, "LEGO_DRIVER_NAME=%s", tm->driver_name);
		if (ret) {
			dev_err(dev, "failed to add uevent LEGO_DRIVER_NAME\n");
			return ret;
		}

	ret = add_uevent_var(env, "LEGO_ADDRESS=%s", tm->address);
	if (ret) {
		dev_err(dev, "failed to add uevent LEGO_ADDRESS\n");
		return ret;
	}

	return 0;
}

static char *tacho_motor_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "tacho-motor/%s", dev_name(dev));
}

/* Note: dev_groups gets filled in when the tacho motor is registered */

struct class tacho_motor_class = {
	.name		= "tacho-motor",
	.owner		= THIS_MODULE,
	.dev_uevent	= tacho_motor_dev_uevent,
	.devnode	= tacho_motor_devnode,
};

static int tacho_motor_class_init(void)
{
	int err;

	err = class_register(&tacho_motor_class);
	if (err) {
		pr_err("unable to register tacho_motor_class\n");
		return err;
	}

	return 0;
}
module_init(tacho_motor_class_init);

static void tacho_motor_class_exit(void)
{
	class_unregister(&tacho_motor_class);
}
module_exit(tacho_motor_class_exit);

MODULE_DESCRIPTION("Tacho Motor device class");
MODULE_AUTHOR("Ralph Hempel <rhempel@hempeldesigngroup.com>");
MODULE_LICENSE("GPL");
