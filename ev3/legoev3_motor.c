/*
 * Motor driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2013-2014,2016 Ralph Hempel <rhempel@hempeldesigngroup.com>
 * Copyright (C) 2015-2016,2018 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**
 * DOC: userspace
 *
 * The ``legoev3-motor`` module is used on devices like the EV3 where motors
 * are connected directly to the CPU rather than an external controller.
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <linux/interrupt.h>
#include <linux/math64.h>

#include <lego.h>
#include <lego_port_class.h>
#include <dc_motor_class.h>
#include <tacho_motor_class.h>
#include <tacho_motor_helper.h>

#include "legoev3_motor.h"
#include "../motors/ev3_motor.h"

#define TACHO_MOTOR_STALLED_MS  100

enum legoev3_motor_state {
	STATE_RUNNING,
	STATE_STOPPED,
	NUM_STATES,
};

struct legoev3_motor_data {
	struct tacho_motor_device tm;
	struct lego_device *ldev;
	struct iio_cb_buffer *cb_buffers;

	struct work_struct notify_position_ramp_down_work;
	struct tm_pid speed_pid;
	struct tm_pid hold_pid;

	int position_sp;
	int position_offset;

	ktime_t stalling_since;
	bool stalling;
	bool stalled;
	bool ramping;

	int position;
	int speed;
	int duty_cycle;
	enum legoev3_motor_state state;
	enum tm_stop_action run_to_pos_stop_action;
	bool run_to_pos_active;
	bool hold_pos_sp;
	bool speed_pid_ena;
	bool hold_pid_ena;

	struct dentry *debug;
};

static DEFINE_SPINLOCK(lock);

static void set_duty_cycle(struct legoev3_motor_data *ev3_tm, int duty_cycle)
{
	const struct dc_motor_ops *motor_ops = ev3_tm->ldev->port->dc_motor_ops;
	void *context = ev3_tm->ldev->port->context;
	int err;

	duty_cycle = min(duty_cycle, DC_MOTOR_MAX_DUTY_CYCLE);
	duty_cycle = max(duty_cycle, -DC_MOTOR_MAX_DUTY_CYCLE);

	if (duty_cycle == ev3_tm->duty_cycle)
		return;

	if (duty_cycle > 0)
		motor_ops->set_command(context,
				       DC_MOTOR_INTERNAL_COMMAND_RUN_FORWARD);
	else
		motor_ops->set_command(context,
				       DC_MOTOR_INTERNAL_COMMAND_RUN_REVERSE);

	err = motor_ops->set_duty_cycle(context, abs(duty_cycle));
	WARN_ONCE(err, "Failed to set pwm duty cycle! (%d)\n", err);

	ev3_tm->duty_cycle = duty_cycle;

}

static int legoev3_motor_stop(void *context, enum tm_stop_action action)
{
	struct legoev3_motor_data *ev3_tm = context;
	const struct dc_motor_ops *motor_ops = ev3_tm->ldev->port->dc_motor_ops;
	void *dc_ctx = ev3_tm->ldev->port->context;
	bool use_pos_sp_for_hold = ev3_tm->hold_pos_sp;
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);

	ev3_tm->run_to_pos_active = false;
	ev3_tm->hold_pos_sp = false;
	ev3_tm->speed_pid_ena = false;
	ev3_tm->hold_pid_ena = false;
	ev3_tm->duty_cycle = 0;
	/*
	 * Reset the PID terms here to avoid having these terms influence the
	 * motor operation at the beginning of the next sequence. The most
	 * common issue is having some residual integral value briefly turn
	 * the motor on hard if we're ramping up slowly.
	 */
	tm_pid_reinit(&ev3_tm->speed_pid);
	tm_pid_reinit(&ev3_tm->hold_pid);

	switch (action) {
	case TM_STOP_ACTION_COAST:
		motor_ops->set_command(dc_ctx, DC_MOTOR_INTERNAL_COMMAND_COAST);
		break;
	case TM_STOP_ACTION_BRAKE:
		motor_ops->set_command(dc_ctx, DC_MOTOR_INTERNAL_COMMAND_BRAKE);
		break;
	case TM_STOP_ACTION_HOLD:
		if (use_pos_sp_for_hold)
			ev3_tm->hold_pid.setpoint = ev3_tm->position_sp;
		else
			ev3_tm->hold_pid.setpoint = ev3_tm->position + ev3_tm->position_offset;
		ev3_tm->hold_pid_ena = true;
		break;
	default:
		WARN(true, "bad action: %d\n", action);
	}

	ev3_tm->state = STATE_STOPPED;

	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

/**
 * legoev3_motor_reset - reinitialize driver to default values.
 *
 * @context: The motor data structure.
 */
static int legoev3_motor_reset(void *context)
{
	struct legoev3_motor_data *ev3_tm = context;
	const struct legoev3_motor_info *info = &ev3_tm->tm.info->legoev3_info;
	unsigned long flags;

	legoev3_motor_stop(ev3_tm, TM_STOP_ACTION_COAST);

	spin_lock_irqsave(&lock, flags);

	ev3_tm->position_sp		= 0;
	ev3_tm->position_offset		= -ev3_tm->position;
	ev3_tm->speed			= 0;
	ev3_tm->duty_cycle		= 0;
	ev3_tm->stalled			= false;
	ev3_tm->stalling		= false;

	tm_pid_init(&ev3_tm->speed_pid, info->speed_pid_k.p,
		    info->speed_pid_k.i, info->speed_pid_k.d);
	tm_pid_init(&ev3_tm->hold_pid, info->position_pid_k.p,
		    info->position_pid_k.i, info->position_pid_k.d);

	spin_unlock_irqrestore(&lock, flags);

	return 0;
};

static void update_stall(struct legoev3_motor_data *ev3_tm)
{
	/* if the motor is running, but not moving, then maybe we are stalled */
	if (ev3_tm->state == STATE_RUNNING && ev3_tm->speed == 0) {
		if (ev3_tm->stalling) {
			if (TACHO_MOTOR_STALLED_MS < ktime_to_ms(ktime_sub(ktime_get(), ev3_tm->stalling_since)))
				ev3_tm->stalled = true;
		} else {
			ev3_tm->stalling_since = ktime_get();
			ev3_tm->stalling = true;
		}
	} else {
		ev3_tm->stalled = false;
		ev3_tm->stalling = false;
	}
}

static void update_position(struct legoev3_motor_data *ev3_tm)
{
	int rampdown_endpoint;
	int rampdown_time;
	int new_speed_sp;

	/* Calculate where we would end up if we started a
	 * linear ramp down right now, the 1 prevents the
	 * value from being 0 and causing a division problem
	 */

	rampdown_time = (abs(ev3_tm->speed) * ev3_tm->tm.active_params.ramp_down_sp)
				/ (1 + ev3_tm->tm.info->max_speed);

	rampdown_endpoint = ev3_tm->position + ev3_tm->position_offset
			  + ((ev3_tm->speed * rampdown_time) / (2*MSEC_PER_SEC));

	new_speed_sp = 2 * MSEC_PER_SEC * (ev3_tm->position_sp -
		(ev3_tm->position + ev3_tm->position_offset)) / (1 + rampdown_time);

	if (ev3_tm->speed_pid.setpoint > 0) {
		if (rampdown_endpoint > ev3_tm->position_sp) {
			ev3_tm->speed_pid.setpoint = new_speed_sp;
			ev3_tm->ramping = true;
		}

		if (ev3_tm->position + ev3_tm->position_offset >= ev3_tm->position_sp) {
			schedule_work(&ev3_tm->notify_position_ramp_down_work);
			ev3_tm->hold_pos_sp = true;
			legoev3_motor_stop(ev3_tm,
					   ev3_tm->run_to_pos_stop_action);
			ev3_tm->ramping = false;
		}
	} else if (ev3_tm->speed_pid.setpoint < 0) {
		if (rampdown_endpoint < ev3_tm->position_sp) {
			ev3_tm->speed_pid.setpoint = new_speed_sp;
			ev3_tm->ramping = true;
		}

		if (ev3_tm->position + ev3_tm->position_offset <= ev3_tm->position_sp) {
			schedule_work(&ev3_tm->notify_position_ramp_down_work);
			ev3_tm->hold_pos_sp = true;
			legoev3_motor_stop(ev3_tm,
					   ev3_tm->run_to_pos_stop_action);
			ev3_tm->ramping = false;
		}
	}
}

static int legoev3_motor_tacho_cb(const void *data, void *p)
{
	struct legoev3_motor_data *ev3_tm = p;
	const int *values = data;
	int duty_cycle = ev3_tm->duty_cycle;

	/* new tacho driver counts in 4x mode, but this driver expects 2x */
	ev3_tm->position = values[0] / 2;
	ev3_tm->speed = values[1] / 2;

	update_stall(ev3_tm);

	if (ev3_tm->speed_pid_ena) {
		if (ev3_tm->speed_pid.setpoint == 0) {
			duty_cycle = 0;
			tm_pid_reinit(&ev3_tm->speed_pid);
		} else
			duty_cycle = tm_pid_update(&ev3_tm->speed_pid,
						   ev3_tm->speed);
	} else if (ev3_tm->hold_pid_ena)
		duty_cycle = tm_pid_update(&ev3_tm->hold_pid,
				 ev3_tm->position + ev3_tm->position_offset);

	set_duty_cycle(ev3_tm, duty_cycle);

	if (ev3_tm->run_to_pos_active)
		update_position(ev3_tm);

	return 0;
}

static void legoev3_motor_notify_position_ramp_down_work(struct work_struct *work)
{
	struct legoev3_motor_data *ev3_tm =
		container_of(work, struct legoev3_motor_data, notify_position_ramp_down_work);

	tacho_motor_notify_position_ramp_down(&ev3_tm->tm);
}

static int legoev3_motor_get_position(void *context, int *position)
{
	struct legoev3_motor_data *ev3_tm = context;

	*position = ev3_tm->position + ev3_tm->position_offset;

	return 0;
}

static int legoev3_motor_get_state(void *context);

static int legoev3_motor_set_position(void *context, int position)
{
	struct legoev3_motor_data *ev3_tm = context;

	if (legoev3_motor_get_state(ev3_tm) & TM_STATE_RUNNING)
		return -EBUSY;

	ev3_tm->position_offset = position - ev3_tm->position;
	ev3_tm->position_sp = position;

	return 0;
}

static int legoev3_motor_get_duty_cycle(void *context,
					int *duty_cycle)
{
	struct legoev3_motor_data *ev3_tm = context;

	*duty_cycle = ev3_tm->duty_cycle;

	return 0;
}

static int legoev3_motor_run_unregulated(void *context, int duty_cycle)
{
	struct legoev3_motor_data *ev3_tm = context;
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);

	ev3_tm->run_to_pos_active = false;
	ev3_tm->speed_pid_ena = false;
	ev3_tm->hold_pid_ena = false;
	set_duty_cycle(ev3_tm, duty_cycle);
	ev3_tm->state = STATE_RUNNING;

	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

static int legoev3_motor_get_state(void *context)
{
	struct legoev3_motor_data *ev3_tm = context;
	unsigned state = 0;

	if (ev3_tm->state == STATE_RUNNING) {
		state |= BIT(TM_STATE_RUNNING);
		if (ev3_tm->speed_pid_ena
		    && tm_pid_is_overloaded(&ev3_tm->speed_pid))
			state |= BIT(TM_STATE_OVERLOADED);
		if (ev3_tm->stalled)
			state |= BIT(TM_STATE_STALLED);
		if (ev3_tm->ramping)
			state |= BIT(TM_STATE_RAMPING);
	}
	if (ev3_tm->hold_pid_ena) {
		state |= BIT(TM_STATE_HOLDING);
		if (tm_pid_is_overloaded(&ev3_tm->hold_pid))
			state |= BIT(TM_STATE_OVERLOADED);
	}

	return state;
}

static int legoev3_motor_get_speed(void *context, int *speed)
{
	struct legoev3_motor_data *ev3_tm = context;

	*speed = ev3_tm->speed;

	return 0;
}

static int legoev3_motor_run_regulated(void *context, int speed)
{
	struct legoev3_motor_data *ev3_tm = context;
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);

	ev3_tm->run_to_pos_active = false;
	ev3_tm->speed_pid_ena = true;
	ev3_tm->hold_pid_ena = false;
	ev3_tm->speed_pid.setpoint = speed;
	ev3_tm->state = STATE_RUNNING;

	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

static int legoev3_motor_run_to_pos(void *context, int pos, int speed,
				    enum tm_stop_action stop_action)
{
	struct legoev3_motor_data *ev3_tm = context;
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);

	speed = abs(speed);
	if (ev3_tm->position + ev3_tm->position_offset > pos)
		speed *= -1;

	ev3_tm->run_to_pos_active = true;
	ev3_tm->speed_pid_ena = true;
	ev3_tm->hold_pid_ena = false;
	ev3_tm->position_sp = pos;
	ev3_tm->speed_pid.setpoint = speed;
	ev3_tm->run_to_pos_stop_action = stop_action;
	ev3_tm->state = STATE_RUNNING;

	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

static unsigned legoev3_motor_get_stop_actions(void *context)
{
	return BIT(TM_STOP_ACTION_COAST) | BIT(TM_STOP_ACTION_BRAKE) |
		BIT(TM_STOP_ACTION_HOLD);
}

TM_PID_GET_FUNC(legoev3_motor, speed_Kp, legoev3_motor_data, speed_pid.Kp);
TM_PID_SET_FUNC(legoev3_motor, speed_Kp, legoev3_motor_data, speed_pid.Kp);
TM_PID_GET_FUNC(legoev3_motor, speed_Ki, legoev3_motor_data, speed_pid.Ki);
TM_PID_SET_FUNC(legoev3_motor, speed_Ki, legoev3_motor_data, speed_pid.Ki);
TM_PID_GET_FUNC(legoev3_motor, speed_Kd, legoev3_motor_data, speed_pid.Kd);
TM_PID_SET_FUNC(legoev3_motor, speed_Kd, legoev3_motor_data, speed_pid.Kd);
TM_PID_GET_FUNC(legoev3_motor, position_Kp, legoev3_motor_data, hold_pid.Kp);
TM_PID_SET_FUNC(legoev3_motor, position_Kp, legoev3_motor_data, hold_pid.Kp);
TM_PID_GET_FUNC(legoev3_motor, position_Ki, legoev3_motor_data, hold_pid.Ki);
TM_PID_SET_FUNC(legoev3_motor, position_Ki, legoev3_motor_data, hold_pid.Ki);
TM_PID_GET_FUNC(legoev3_motor, position_Kd, legoev3_motor_data, hold_pid.Kd);
TM_PID_SET_FUNC(legoev3_motor, position_Kd, legoev3_motor_data, hold_pid.Kd);

static const struct tacho_motor_ops legoev3_motor_ops = {
	.get_position		= legoev3_motor_get_position,
	.set_position		= legoev3_motor_set_position,

	.get_state		= legoev3_motor_get_state,
	.get_duty_cycle		= legoev3_motor_get_duty_cycle,
	.get_speed		= legoev3_motor_get_speed,

	.run_unregulated	= legoev3_motor_run_unregulated,
	.run_regulated		= legoev3_motor_run_regulated,
	.run_to_pos		= legoev3_motor_run_to_pos,
	.stop			= legoev3_motor_stop,
	.reset			= legoev3_motor_reset,

	.get_stop_actions	= legoev3_motor_get_stop_actions,

	.get_speed_Kp		= legoev3_motor_get_speed_Kp,
	.set_speed_Kp		= legoev3_motor_set_speed_Kp,
	.get_speed_Ki		= legoev3_motor_get_speed_Ki,
	.set_speed_Ki		= legoev3_motor_set_speed_Ki,
	.get_speed_Kd		= legoev3_motor_get_speed_Kd,
	.set_speed_Kd		= legoev3_motor_set_speed_Kd,

	.get_hold_Kp		= legoev3_motor_get_position_Kp,
	.set_hold_Kp		= legoev3_motor_set_position_Kp,
	.get_hold_Ki		= legoev3_motor_get_position_Ki,
	.set_hold_Ki		= legoev3_motor_set_position_Ki,
	.get_hold_Kd		= legoev3_motor_get_position_Kd,
	.set_hold_Kd		= legoev3_motor_set_position_Kd,
};

static int legoev3_motor_probe(struct lego_device *ldev)
{
	struct legoev3_motor_data *ev3_tm;
	int err;

	if (WARN_ON(!ldev->port->dc_motor_ops))
		return -EINVAL;
	if (WARN_ON(!ldev->entry_id))
		return -EINVAL;

	ev3_tm = devm_kzalloc(&ldev->dev, sizeof(*ev3_tm), GFP_KERNEL);
	if (!ev3_tm)
		return -ENOMEM;

	ev3_tm->cb_buffers = iio_channel_get_all_cb(&ldev->dev, legoev3_motor_tacho_cb, ev3_tm);
	err = PTR_ERR_OR_ZERO(ev3_tm->cb_buffers);
	if (err) {
		if (err != -EPROBE_DEFER)
			dev_err(&ldev->dev, "failed to get iio callback buffers (%d)\n", err);
		return err;
	}

	ev3_tm->ldev = ldev;

	ev3_tm->tm.driver_name = ldev->entry_id->name;
	ev3_tm->tm.address = ldev->port->address;
	ev3_tm->tm.ops = &legoev3_motor_ops;
	ev3_tm->tm.info = &ev3_motor_defs[ldev->entry_id->driver_data];
	ev3_tm->tm.context = ev3_tm;

	dev_set_drvdata(&ldev->dev, ev3_tm);

	INIT_WORK(&ev3_tm->notify_position_ramp_down_work,
		  legoev3_motor_notify_position_ramp_down_work);

	err = register_tacho_motor(&ev3_tm->tm, &ldev->dev);

	if (err)
		goto err_free_cb_buffers;

	legoev3_motor_reset(ev3_tm);

	iio_channel_start_all_cb(ev3_tm->cb_buffers);

	ev3_tm->debug = debugfs_create_dir(ev3_tm->tm.address, NULL);
	debugfs_create_u32("position_sp", 0444, ev3_tm->debug, &ev3_tm->position_sp);
	debugfs_create_u32("position_offset", 0444, ev3_tm->debug, &ev3_tm->position_offset);
	debugfs_create_bool("stalled", 0444, ev3_tm->debug, &ev3_tm->stalled);
	debugfs_create_bool("ramping", 0444, ev3_tm->debug, &ev3_tm->ramping);
	debugfs_create_u32("position", 0444, ev3_tm->debug, &ev3_tm->position);
	debugfs_create_u32("speed", 0444, ev3_tm->debug, &ev3_tm->speed);
	debugfs_create_u32("duty_cycle", 0444, ev3_tm->debug, &ev3_tm->duty_cycle);
	debugfs_create_u32("state", 0444, ev3_tm->debug, &ev3_tm->state);
	debugfs_create_u32("run_to_pos_stop_action", 0444, ev3_tm->debug, &ev3_tm->run_to_pos_stop_action);
	debugfs_create_bool("run_to_pos_active", 0444, ev3_tm->debug, &ev3_tm->run_to_pos_active);
	debugfs_create_bool("speed_pid_ena", 0444, ev3_tm->debug, &ev3_tm->speed_pid_ena);
	debugfs_create_bool("hold_pid_ena", 0444, ev3_tm->debug, &ev3_tm->hold_pid_ena);

	return 0;

err_free_cb_buffers:
	iio_channel_release_all_cb(ev3_tm->cb_buffers);

	return err;
}

static int legoev3_motor_remove(struct lego_device *ldev)
{
	struct legoev3_motor_data *ev3_tm = dev_get_drvdata(&ldev->dev);

	debugfs_remove_recursive(ev3_tm->debug);

	iio_channel_stop_all_cb(ev3_tm->cb_buffers);
	cancel_work_sync(&ev3_tm->notify_position_ramp_down_work);
	unregister_tacho_motor(&ev3_tm->tm);
	iio_channel_release_all_cb(ev3_tm->cb_buffers);
	return 0;
}

struct lego_device_id legoev3_motor_driver_id_table[NUM_EV3_MOTOR_ID] = {
	LEGO_DEVICE_ID(LEGO_NXT_MOTOR),
	LEGO_DEVICE_ID(LEGO_EV3_LARGE_MOTOR),
	LEGO_DEVICE_ID(LEGO_EV3_MEDIUM_MOTOR),
	LEGO_DEVICE_ID(ACT_L12_EV3_50),
	LEGO_DEVICE_ID(ACT_L12_EV3_100),
};

/* TODO: Make this a bus driver attribute instead so it is present for all drivers on the lego bus */
static ssize_t driver_names_show(struct device_driver *drv, char *buf)
{
	int i;
	int size = 0;

	for (i = 0; i < NUM_EV3_MOTOR_ID; i++) {
		size += sprintf(buf + size, "%s ",
			legoev3_motor_driver_id_table[i].name);
	}

	buf[size - 1] = '\n';

	return size;
}

static DRIVER_ATTR_RO(driver_names);

static struct attribute *legoev3_motor_attrs[] = {
	&driver_attr_driver_names.attr,
	NULL
};

ATTRIBUTE_GROUPS(legoev3_motor);

struct lego_device_driver legoev3_motor_driver = {
	.probe	= legoev3_motor_probe,
	.remove	= legoev3_motor_remove,
	.driver = {
		.name	= "legoev3-motor",
		.owner	= THIS_MODULE,
		.groups	= legoev3_motor_groups,
	},
	.id_table = legoev3_motor_driver_id_table,
};
lego_device_driver(legoev3_motor_driver);

MODULE_DESCRIPTION("Motor driver for LEGO MINDSTORMS EV3");
MODULE_AUTHOR("Ralph Hempel <rhempel@hempeldesigngroup.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lego:legoev3-motor");
