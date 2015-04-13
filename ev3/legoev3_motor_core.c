/*
 * Motor driver for LEGO MINDSTORMS EV3
 *
 * Copyright (C) 2013-2014 Ralph Hempel <rhempel@hempeldesigngroup.com>
 * Copyright (C) 2015 David Lechner <david@lechnology.com>
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
 * EV3/NXT Tacho Motor Driver
 *
 * This driver provides a [tacho-motor] interface for EV3 and NXT motors or any
 * other compatible motor with an [incremental rotary encoder] for position
 * and direction feedback that is connected to an output port. We call them
 * "tacho" motors because that is what the LMS2012 source code calls them. You
 * can find the devices bound to this driver in the directory
 * `/sys/bus/lego/drivers/legoev3-motor /`. There is not much of interest
 * there though - all of the useful stuff is in the [tacho-motor] class.
 * .
 * [tacho-motor]: ../tacho-motor-class
 * [incremental rotary encoder]: https://en.wikipedia.org/wiki/Rotary_encoder#Incremental_rotary_encoder
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/random.h>
#include <linux/platform_data/legoev3.h>

#include <mach/time.h>

#include <lego.h>
#include <lego_port_class.h>
#include <dc_motor_class.h>
#include <tacho_motor_class.h>

#include "legoev3_motor.h"

#define TACHO_MOTOR_POLL_NS	2000000	/* 2 msec */

#define TACHO_SAMPLES		128

#define MAX_PWM_CNT		10000
#define MAX_SPEED		100
#define MAX_POWER		100
#define MAX_SYNC_MOTORS		2

enum legoev3_motor_command {
	UNKNOWN,
	FORWARD,
	REVERSE,
	BRAKE,
	COAST,
};

enum legoev3_motor_position_mode {
	POSITION_ABSOLUTE,
	POSITION_RELATIVE,
	NUM_POSITION_MODES,
};

enum legoev3_motor_state
{
	STATE_RUN_FOREVER,
	STATE_SETUP_RAMP_POSITION,
	STATE_SETUP_RAMP_REGULATION,
	STATE_RAMP_UP,
	STATE_RAMP_CONST,
	STATE_POSITION_RAMP_DOWN,
	STATE_RAMP_DOWN,
	STATE_STOP,
	STATE_IDLE,
	NUM_STATES,
};

struct legoev3_motor_data {
	struct tacho_motor_device tm;
	struct tacho_motor_params active_params;
	struct lego_device *ldev;
	const struct legoev3_motor_info *info;

	struct hrtimer timer;
	struct work_struct notify_state_change_work;

	unsigned tacho_samples[TACHO_SAMPLES];
	unsigned tacho_samples_head;

	bool got_new_sample;

	int num_samples;
	int dir_chg_samples;

	int clock_ticks_per_sample;

	bool irq_mutex;

	struct {
		struct {
			int start;
			int end;
			int full;
		} up;

		struct {
			int start;
			int end;
			int full;
		} down;

		int percent;
		int direction;
		int position_sp;
		int count;	/* This must be set to either tacho or time increment! */
	} ramp;

	struct {
		int P;
		int I;
		int D;
		int speed_Kp;
		int speed_Ki;
		int speed_Kd;
		int prev_speed;
		int prev_position_error;
	} pid;

	int speed_reg_sp;
	int run_direction;

	int run;

	int tacho;
	int irq_tacho;	/* tacho and irq_tacho combine to make position - change name to pulse? */

	int speed;
	int duty_cycle;
	enum legoev3_motor_state state;

	enum tacho_motor_command run_command;
	enum legoev3_motor_position_mode position_mode;
};

static void set_num_samples_for_speed(struct legoev3_motor_data *ev3_tm, int speed)
{
	if (speed > 80) {
		ev3_tm->num_samples = ev3_tm->info->samples_for_speed[SPEED_ABOVE_80];
	} else if (speed > 60) {
		ev3_tm->num_samples = ev3_tm->info->samples_for_speed[SPEED_ABOVE_60];
	} else if (speed > 40) {
		ev3_tm->num_samples = ev3_tm->info->samples_for_speed[SPEED_ABOVE_40];
	} else {
		ev3_tm->num_samples = ev3_tm->info->samples_for_speed[SPEED_BELOW_40];
	}
}

/*
 * Handling the Tachometer Inputs
 *
 * The tacho motor driver uses two pins on each port to determine the direction
 * of rotation of the motor.
 *
 * `pdata->tacho_int_gpio` is the pin that is set up to trigger an interrupt
 * any edge change
 *
 * `pdata->tacho_dir_gpio` is the pin that helps to determine the direction
 * of rotation
 *
 * When int == dir then the encoder is turning in the forward direction
 * When int != dir then the encoder is turning in the reverse direction
 *
 * -----     --------           --------      -----
 *     |     |      |           |      |      |
 *     |     |      |           |      |      |
 *     -------      -------------       -------          DIRx signal
 *
 *  -------     --------     --------     --------       INTx signal
 *        |     |      |     |      |     |      |
 *        |     |      |     |      |     |      |
 *        -------      -------      -------      -----
 *        \     \      \     \      \     \      \
 *         ^     ^      ^     ^      ^     ^      ^      ISR handler
 *         +1    +1     +1    -1     -1    -1     -1     TachoCount
 *
 * All this works perfectly well when there are no missed interrupts, and when
 * the transitions on these pins are clean (no bounce or noise). It is possible
 * to get noisy operation when the transitions are very slow, and we have
 * observed signals similar to this:
 *
 * -------------                       -------------
 *             |                       |
 *             |                       |
 *             -------------------------                 DIRx signal
 *
 *    ---------------   ----                             INTx signal
 *    |             |   |  |
 *    |             |   |  |
 * ----             -----  -------------------------
 *    \              \   \  \
 *     ^              ^   ^  ^                           ISR Handler
 *     +1             +1  -1 +1                          TachoCount
 *                    A   B  C
 *
 * The example above has three transitions that we are interested in
 * labeled A, B, and C - they represent a noisy signal. As long as
 * all three transitions are caught by the ISR, then the count is
 * incremented by 2 as expected. But there are other outcomes possible.
 *
 * For example, if the A transition is handled, but the INT signal
 * is not measured until after B, then the final count value is 1.
 *
 * On the other hand, if the B transition is missed, and only A and
 * C are handled, then the final count value is 3.
 *
 * Either way, we need to figure out a way to clean things up, and as
 * long as at least two of the interrupts are caught, we can "undo"
 * a reading quite easily.
 *
 * The mini-tacho motor turns at a maximum of 1200 pulses per second, the
 * standard tacho motor has a maximum speed of 900 pulses per second. Taking
 * the highest value, this means that about 800 usec is the fastest time
 * between interrupts. If we see two interrupts with a delta of much less
 * than, say 400 usec, then we're probably looking at a noisy transition.
 *
 * In most cases that have been captured, the shortest delta is the A-B
 * transition, anywhere from 10 to 20 usec, which is faster than the ISR
 * response time. The B-C transition has been measured up to 150 usec.
 *
 * It is clear that the correct transition to use for changing the
 * value of `TachoCount` is C - so if the delta from A-C is less than
 * the threshold, we should "undo" whatever the A transition told us.
 */

static irqreturn_t tacho_motor_isr(int irq, void *id)
{
	struct legoev3_motor_data *ev3_tm = id;
	struct ev3_motor_platform_data *pdata = ev3_tm->ldev->dev.platform_data;

	bool int_state =  gpio_get_value(pdata->tacho_int_gpio);
	bool dir_state = !gpio_get_value(pdata->tacho_dir_gpio);

	unsigned long timer      = legoev3_hires_timer_read();
	unsigned long prev_timer = ev3_tm->tacho_samples[ev3_tm->tacho_samples_head];

	unsigned next_sample;

	int  next_direction = ev3_tm->run_direction;

	next_sample = (ev3_tm->tacho_samples_head + 1) % TACHO_SAMPLES;

	/* If the speed is high enough, just update the tacho counter based on direction */

	if ((35 < ev3_tm->speed) || (-35 > ev3_tm->speed)) {

		if (ev3_tm->dir_chg_samples < (TACHO_SAMPLES-1))
			ev3_tm->dir_chg_samples++;

	} else {

		/*
		 * Update the tacho count and motor direction for low speed, taking
		 * advantage of the fact that if state and dir match, then the motor
		 * is turning FORWARD!
		 *
		 * We also look after the polarity_mode and encoder_mode here as follows:
		 *
		 * polarity_mode | encoder_mode | next_direction
		 * --------------+--------------+---------------
		 * normal        | normal       | normal
		 * normal        | inverted     | inverted
		 * inverted      | normal       | inverted
		 * inverted      | inverted     | normal
		 *
		 * Yes, this could be compressed into a clever set of conditionals that
		 * results in only two assignments, or a lookup table, but it's clearer
		 * to write nested if statements in this case - it looks a lot more
		 * like the truth table
		 */

		if (ev3_tm->active_params.polarity == DC_MOTOR_POLARITY_NORMAL) {
			if (ev3_tm->active_params.encoder_polarity == DC_MOTOR_POLARITY_NORMAL) {
				next_direction = (int_state == dir_state) ? FORWARD : REVERSE;
			} else {
				next_direction = (int_state == dir_state) ? REVERSE : FORWARD;
			}
		} else {
			if (ev3_tm->active_params.encoder_polarity == DC_MOTOR_POLARITY_NORMAL) {
				next_direction = (int_state == dir_state) ? REVERSE : FORWARD;
			} else {
				next_direction = (int_state == dir_state) ? FORWARD : REVERSE;
			}
		}

		/*
		 * If the difference in timestamps is too small, then undo the
		 * previous increment - it's OK for a count to waver once in
		 * a while - better than being wrong!
		 *
		 * Here's what we'll do when the transition is too small:
		 *
		 * 1) UNDO the increment to the next timer sample update
		 *    dir_chg_samples count!
		 * 2) UNDO the previous run_direction count update
		 *
		 * TODO: Make this work out to 400 usec based on the clock rate!
		 */

		if ((400 * 33) > (timer - prev_timer)) {
			ev3_tm->tacho_samples[ev3_tm->tacho_samples_head] = timer;

			if (FORWARD == ev3_tm->run_direction)
				ev3_tm->irq_tacho--;
			else
				ev3_tm->irq_tacho++;

			next_sample = ev3_tm->tacho_samples_head;
		} else {
			/*
			 * If the saved and next direction states
			 * match, then update the dir_chg_sample count
			 */

			if (ev3_tm->run_direction == next_direction) {
				if (ev3_tm->dir_chg_samples < (TACHO_SAMPLES-1))
					ev3_tm->dir_chg_samples++;
			} else {
				ev3_tm->dir_chg_samples = 0;
			}
		}
	}

	ev3_tm->run_direction = next_direction;

	/* Grab the next incremental sample timestamp */

	ev3_tm->tacho_samples[next_sample] = timer;
	ev3_tm->tacho_samples_head = next_sample;

	ev3_tm->irq_mutex = true;

	if (FORWARD == ev3_tm->run_direction)
		ev3_tm->irq_tacho++;
	else
		ev3_tm->irq_tacho--;

	ev3_tm->got_new_sample = true;

	ev3_tm->irq_mutex = false;

	return IRQ_HANDLED;
}

void legoev3_motor_update_output(struct legoev3_motor_data *ev3_tm)
{
	const struct dc_motor_ops *motor_ops = ev3_tm->ldev->port->motor_ops;
	void *context = ev3_tm->ldev->port->context;
	int err;

	if (ev3_tm->duty_cycle > 0) {
		motor_ops->set_direction(context, ev3_tm->active_params.polarity);
		motor_ops->set_command(context, DC_MOTOR_COMMAND_RUN);
		if (ev3_tm->active_params.speed_regulation == TM_SPEED_REGULATION_OFF && ev3_tm->duty_cycle < 10)
			ev3_tm->duty_cycle = 10;
	} else if (ev3_tm->duty_cycle < 0) {
		motor_ops->set_direction(context, !ev3_tm->active_params.polarity);
		motor_ops->set_command(context, DC_MOTOR_COMMAND_RUN);
		if (ev3_tm->active_params.speed_regulation == TM_SPEED_REGULATION_OFF && ev3_tm->duty_cycle > -10)
			ev3_tm->duty_cycle = -10;
	} else {
		if (ev3_tm->active_params.stop_command == TM_STOP_COMMAND_COAST)
			motor_ops->set_command(context, DC_MOTOR_COMMAND_COAST);
		else if (TM_STOP_COMMAND_BRAKE == ev3_tm->active_params.stop_command)
			motor_ops->set_command(context, DC_MOTOR_COMMAND_BRAKE);
		else if (TM_STOP_COMMAND_HOLD == ev3_tm->active_params.stop_command)
			motor_ops->set_command(context, DC_MOTOR_COMMAND_BRAKE);
	}

	/* The power sets the duty cycle - 100% power == 100% duty cycle */
	err = motor_ops->set_duty_cycle(context, abs(ev3_tm->duty_cycle));
	WARN_ONCE(err, "Failed to set pwm duty cycle! (%d)\n", err);
}

static void legoev3_motor_set_power(struct legoev3_motor_data *ev3_tm, int power)
{
	if (ev3_tm->duty_cycle == power)
		return;

	if (power > MAX_POWER)
		power = MAX_POWER;
	else if (power < -MAX_POWER)
		power = -MAX_POWER;

	ev3_tm->duty_cycle = power;
	legoev3_motor_update_output(ev3_tm);
}

/**
 * legoev3_motor_reset
 *
 * This is the same as initializing a motor - we will set everything
 * to default values, as if it had just been plugged in
 */
static void legoev3_motor_reset(struct legoev3_motor_data *ev3_tm)
{
	memset(ev3_tm->tacho_samples, 0, sizeof(unsigned) * TACHO_SAMPLES);

	ev3_tm->tacho_samples_head	= 0;
	ev3_tm->got_new_sample		= false;
	ev3_tm->num_samples		= ev3_tm->info->samples_for_speed[SPEED_BELOW_40];
	ev3_tm->dir_chg_samples		= 0;
	ev3_tm->clock_ticks_per_sample	= ev3_tm->info->clock_ticks_per_sample;
	ev3_tm->speed			= 0;
	ev3_tm->irq_mutex		= false;
	ev3_tm->ramp.up.start		= 0;
	ev3_tm->ramp.up.end		= 0;
	ev3_tm->ramp.down.start		= 0;
	ev3_tm->ramp.down.end		= 0;
	ev3_tm->ramp.percent		= 0;
	ev3_tm->ramp.direction		= 0;
	ev3_tm->ramp.position_sp	= 0;
	ev3_tm->ramp.count		= 0;
	ev3_tm->pid.P			= 0;
	ev3_tm->pid.I			= 0;
	ev3_tm->pid.D			= 0;

	ev3_tm->pid.prev_position_error	= 0;
	ev3_tm->speed_reg_sp		= 0;
	ev3_tm->run_direction		= UNKNOWN;
	ev3_tm->run			= 0;

	ev3_tm->tacho			= 0;
	ev3_tm->irq_tacho		= 0;
	ev3_tm->speed			= 0;
	ev3_tm->duty_cycle		= 0;
	ev3_tm->state			= STATE_IDLE;

	ev3_tm->run_command		= TM_COMMAND_STOP;
	ev3_tm->position_mode		= POSITION_ABSOLUTE;

	ev3_tm->tm.params.encoder_polarity = ev3_tm->info->encoder_polarity;
};

/*
 * NOTE: The comments are from the original LEGO source code, but the code
 *       has been changed to reflect the per-motor data structures.
 *
 * NOTE: The original LEGO code used the 24 most significant bits of the
 *       free-running P3 timer by dividing the values captured at every
 *       interrupt edge by 256. Unfortunately, this results in having to
 *       mask off the 24 least significant bits in all subsequent calculations
 *       that involve the captured values.
 *
 *       The reason is probably to keep most of the calculations within the
 *       16 bit value size. This driver avoids that problem by simply scaling
 *       the results when needed. The code and comments are updated to
 *       reflect the change.
 *
 *  - Calculates the actual speed for a motor
 *
 *  - Returns true when a new speed has been calculated, false if otherwise
 *
 *  - Time is sampled every edge on the tacho
 *      - Timer used is 64bit timer plus (P3) module (dual 32bit un-chained mode)
 *      - 64bit timer is running 33Mhz (24Mhz (Osc) * 22 (Multiplier) / 2 (Post divider) / 2 (DIV2)) / 4 (T64 prescaler)
 *
 *  - Tacho counter is updated on every edge of the tacho INTx pin signal
 *  - Time capture is updated on every edge of the tacho INTx pin signal
 *
 *  - Speed is calculated from the following parameters
 *
 *      - Time is measured edge to edge of the tacho interrupt pin. Average of time is always minimum 2 pulses
 *        (1 high + 1 low period or 1 low + 1 high period) because the duty of the high and low period of the
 *        tacho pulses are not always 50%.
 *
 *        - Average of the large motor
 *          - Above speed 80 it is:          64 samples
 *          - Between speed 60 - 80 it is:   32 samples
 *          - Between speed 40 - 60 it is:   16 samples
 *          - below speed 40 it is:           4 samples
 *
 *        - Average of the medium motor
 *          - Above speed 80 it is:          16 samples
 *          - Between speed 60 - 80 it is:    8 samples
 *          - Between speed 40 - 60 it is:    4 samples
 *          - below speed 40 it is:           2 sample
 *
 *      - Number of samples is always determined based on 1 sample meaning 1 low period or 1 high period,
 *        this is to enable fast adoption to changes in speed. Medium motor has the critical timing because
 *        it can change speed and direction very fast.
 *
 *      - Large Motor
 *        - Maximum speed of the Large motor is approximately 2mS per tacho pulse (low + high period)
 *          resulting in minimum timer value of: 2mS / (1/(33MHz)) = 66000 T64 timer ticks.
 *          Because 1 sample is based on only half a period minimum speed is 66000/2 = 33000
 *        - Minimum speed of the large motor is a factor of 100 less than max. speed
 *          max. speed timer ticks * 100 => 66000 * 100 = 6,600,000 T64 timer ticks
 *          Because 1 sample is based on only half a period minimum speed is 6,600,000/2 = 3,300,000.
 *
 *      - Medium Motor
 *        - Maximum speed of the medium motor is approximately 1,25mS per tacho pulse (low + high period)
 *          resulting in minimum timer value of: 1.25mS / (1/(33MHz)) = 41250 (approximately)
 *          Because 1 sample is based on only half a period minimum speed is 41250/2 = 20625.
 *        - Minimum speed of the medium motor is a factor of 100 less than max. speed
 *          max. speed timer ticks * 100 => 41250 * 100 = 4,125,000 T64 timer ticks
 *          Because 1 sample is based on only half a period minimum speed is 4,125,000/2 = 2,062,500.
 *
 *      - Actual speed is then calculated as:
 *        - Large motor:
 *          3,300,000 * number of samples / actual time elapsed for number of samples
 *        - Medium motor:
 *          2,062,500 * number of samples / actual time elapsed for number of samples
 *
 *  - Parameters:
 *    - Input:
 *      - No        : Motor output number
 *      - *pSpeed   : Pointer to the speed value
 *
 *    - Output:
 *      - Status    : Indication of new speed available or not
 *  - Tacho pulse examples:
 *
 *
 *    - Normal
 *
 *      ----       ------       ------       ------
 *          |     |      |     |      |     |      |
 *          |     |      |     |      |     |      |
 *          -------      -------      -------      --- DIRx signal
 *
 *
 *         ----       ------       ------       ------ INTx signal
 *             |     |      |     |      |     |
 *             |     |      |     |      |     |
 *             -------      -------      -------
 *
 *             ^     ^      ^     ^      ^     ^
 *             |     |      |     |      |     |
 *             |   Timer    |   Timer    |   Timer
 *             |     +      |     +      |     +
 *             |  Counter   |  Counter   |  Counter
 *             |            |            |
 *           Timer        Timer        Timer
 *             +            +            +
 *          Counter      Counter      Counter
 *
 *
 *
 *    - Direction change
 *
 *      DirChgPtr variable is used to indicate how many timer samples have been sampled
 *      since direction has been changed. DirChgPtr is set to 0 when tacho interrupt detects
 *      direction change and then it is counted up for every timer sample. So when DirChgPtr
 *      has the value of 2 then there must be 2 timer samples in the the same direction
 *      available.
 *
 *      ----       ------       ------       ------       ---
 *          |     |      |     |      |     |      |     |
 *          |     |      |     |      |     |      |     |
 *          -------      -------      -------      -------   DIRx signal
 *
 *
 *       ------       -------------       ------       ------INTx signal
 *             |     |             |     |      |     |
 *             |     |             |     |      |     |
 *             -------             -------      -------
 *
 *             ^     ^             ^     ^      ^     ^
 *             |     |             |     |      |     |
 *           Timer   |           Timer   |    Timer   |
 *             +     |             +     |      +     |
 *          Counter  |          Counter  |   Counter  |
 *             +     |             +     |      +     |
 *       DirChgPtr++ |       DirChgPtr=0 |DirChgPtr++ |
 *                 Timer               Timer        Timer
 *                   +                   +            +
 *                Counter             Counter      Counter
 *                   +                   +            +
 *               DirChgPtr++         DirChgPtr++  DirChgPtr++
 *
 *
 *
 *
 *      ----       ------             ------        ----
 *          |     |      |           |      |      |
 *          |     |      |           |      |      |
 *          -------      -------------       -------          DIRx signal
 *
 *
 *       ------       ------       ------       ------        INTx signal
 *             |     |      |     |      |     |      |
 *             |     |      |     |      |     |      |
 *             -------      -------      -------       ----
 *
 *             ^     ^      ^     ^      ^     ^      ^
 *             |     |      |     |      |     |      |
 *           Timer   |    Timer   |    Timer   |    Timer
 *             +     |      +     |      +     |      +
 *          Counter  |   Counter  |   Counter  |   Counter
 *             +     |      +     |      +     |      +
 *        DirChgPtr++| DirChgPtr++| DirChgPtr++| DirChgPtr++
 *                 Timer        Timer        Timer
 *                   +            +            +
 *                Counter      Counter      Counter
 *                   +            +            +
 *               DirChgPtr++  DirChgPtr=0  DirChgPtr++
 *
 */

static bool calculate_speed(struct legoev3_motor_data *ev3_tm)
{
	unsigned diff_idx;
	unsigned diff;

	bool speed_updated = false;

	/* TODO - Don't run this if we're updating the ev3_tm in the isr! */

	/*
	 * Determine the approximate speed of the motor using the difference
	 * in time between this tacho pulse and the previous pulse.
	 *
	 * The old code had a conditional that forced the difference to be at
	 * least 1 by checking for the special case of a zero difference, which
	 * would be almost impossible to achieve in practice.
	 *
	 * This version simply ORs a 1 into the LSB of the difference - now
	 * that we're using the full 32 bit free running counter, the impact
	 * on an actual speed calculation is insignificant, and it avoids the
	 * issue with simply adding 1 in the obscure case that the difference
	 * is 0xFFFFFFFF!
	 *
	 * Only do this estimated speed calculation if we've accumulated at
	 * least two tacho pulses where the motor is turning in the same
	 * direction!
	 */

	diff_idx = ev3_tm->tacho_samples_head;

	/* TODO - This should really be a boolean value that gets set at the ISR level */
	/* TODO - Can/Should we change this to not set_samples_per_speed every time we're called? */

	if (ev3_tm->dir_chg_samples >= 1) {

		diff = ev3_tm->tacho_samples[diff_idx]
			- ev3_tm->tacho_samples[(diff_idx + TACHO_SAMPLES - 1) % TACHO_SAMPLES];

		diff |= 1;

		set_num_samples_for_speed(ev3_tm, ev3_tm->clock_ticks_per_sample / diff);
	}

	/*
	 * Now get a better estimate of the motor speed by using the total
	 * time used to accumulate the last n samples, where n is determined
	 * by the first approximation to the speed.
	 *
	 * The new speed can only be updated if we have accumulated at least
	 * as many samples as are required depending on the estimated speed
	 * of the motor.
	 *
	 * If the speed cannot be updated, then we need to check if the speed
	 * is 0!
	 */

	if (ev3_tm->got_new_sample && (ev3_tm->dir_chg_samples >= ev3_tm->num_samples)) {

		diff = ev3_tm->tacho_samples[diff_idx]
			- ev3_tm->tacho_samples[(diff_idx + TACHO_SAMPLES - ev3_tm->num_samples) % TACHO_SAMPLES];

		diff |= 1;

		/* TODO - This should be based on the low level clock rate */

		ev3_tm->speed = (33000000 * ev3_tm->num_samples) / diff;

		if (ev3_tm->run_direction == REVERSE)
			ev3_tm->speed  = -ev3_tm->speed ;

		speed_updated = true;

		ev3_tm->got_new_sample = false;

	} else if (ev3_tm->clock_ticks_per_sample < (legoev3_hires_timer_read() - ev3_tm->tacho_samples[diff_idx])) {

		ev3_tm->dir_chg_samples = 0;

		ev3_tm->speed = 0;

		/* TODO - This is where we can put in a calculation for a stalled motor! */

		speed_updated = true;
	}

	return(speed_updated);
}

static void regulate_speed(struct legoev3_motor_data *ev3_tm)
{
	int max_speed = ev3_tm->info->max_speed;
	int power;
	int speed_error;

	/* Make sure speed_reg_setpoint is within a reasonable range */

	if (ev3_tm->speed_reg_sp > max_speed) {
		ev3_tm->speed_reg_sp = max_speed;
	} else if (ev3_tm->speed_reg_sp < -max_speed) {
		ev3_tm->speed_reg_sp = -max_speed;
	}

	speed_error = ev3_tm->speed_reg_sp - ev3_tm->speed;

	/* TODO - Implement an attribute set for PID constants that adjusts based on speed */

	ev3_tm->pid.P = speed_error;

	/*
	 * The integral term can get quite large if the speed setpoint is higher
	 * than the  maximum speed that the motor can get to. This can happen if
	 * the motor is heavily loaded or if the setpoint is high and the battery
	 * voltage is low.
	 *
	 * To avoid the problem of "integral windup", we stop adding to the
	 * integral term if its contribution alone would set the power level to
	 * 100%
	 *
	 * Earlier versions of this algorithm did not allow the pid.I component
	 * to change once it hit the 100% limit. This algorithm allows the change
	 * if the absolute value of the result is less than 100.
	 */

	ev3_tm->pid.I = ev3_tm->pid.I + speed_error;

	ev3_tm->pid.D = ev3_tm->speed - ev3_tm->pid.prev_speed;

	ev3_tm->pid.prev_speed = ev3_tm->speed;

	power =  ((ev3_tm->pid.P * ev3_tm->info->speed_pid_k.p)
		+ (ev3_tm->pid.I * ev3_tm->info->speed_pid_k.i)
		+ (ev3_tm->pid.D * ev3_tm->info->speed_pid_k.d)) / 10000;

	/*
	 * Subtract the speed error to avoid integral windup if the resulting
	 * power is more than 100%
	 */

	if (100 < abs(power))
		ev3_tm->pid.I = ev3_tm->pid.I - speed_error;

	/*
	 * When regulation_mode is on, and the user sets the speed_sp to 0, the
	 * motor may have been running at a non-zero speed - which will make
	 * the motor oscillate to achieve the 0 speed. A check for the special
	 * condition of speed_sp equal to 0 will turn off the motor to prevent
	 * the oscillation.
	 */

	if (0 == ev3_tm->speed_reg_sp) {
		legoev3_motor_set_power(ev3_tm, 0    );
	} else {
		legoev3_motor_set_power(ev3_tm, power);
	}
}

/*
 * This function changes either the actual power setting for the motor, or the speed
 * regulation setpoint, depending on whether the regulation_mode is on or off.
 *
 * Note that it is assumed by this function and all of its callers that
 * legoev3_motor_set_power() checks whether there's an actual change
 * as well as limiting the range of input values.
 *
 * Similarly, the regulation function must verify the range of ev3_tm->speed_reg_setpoint
 * to avoid unreasonable values.
 *
 * By pushing the checks further down the line, we simplify the higher levels
 * of code!
 */

static void update_motor_speed_or_power(struct legoev3_motor_data *ev3_tm, int percent)
{
	if (TM_SPEED_REGULATION_OFF == ev3_tm->active_params.speed_regulation) {

		if (IS_POS_CMD(ev3_tm->run_command))
			legoev3_motor_set_power(ev3_tm, ev3_tm->ramp.direction * abs((ev3_tm->active_params.duty_cycle_sp * percent)/100));
		else
			legoev3_motor_set_power(ev3_tm,                             ((ev3_tm->active_params.duty_cycle_sp * percent)/100));

	} else if (TM_SPEED_REGULATION_ON == ev3_tm->active_params.speed_regulation) {

		if (IS_POS_CMD(ev3_tm->run_command))
			ev3_tm->speed_reg_sp = ev3_tm->ramp.direction * abs((ev3_tm->active_params.speed_sp * percent)/100);
		else
			ev3_tm->speed_reg_sp =                             ((ev3_tm->active_params.speed_sp * percent)/100);
	}
}

static void regulate_position(struct legoev3_motor_data *ev3_tm)
{
	int power;
	int position_error;


	/*
	 * Make sure that the irq_tacho value has been set to a value that
	 * represents the current error from the desired position so we can
	 * drive the motor towards the desired position hold point.
	 */

	position_error = 0 - ev3_tm->irq_tacho;

	ev3_tm->pid.P = position_error * 400;
	ev3_tm->pid.I = ((ev3_tm->pid.I * 99) / 100) + (position_error / 1);
	ev3_tm->pid.D = (((position_error - ev3_tm->pid.prev_position_error)
			* 4) / 2) *  2;


	ev3_tm->pid.prev_position_error = position_error;

	power = ((ev3_tm->pid.P + ev3_tm->pid.I + ev3_tm->pid.D) / 100);

	legoev3_motor_set_power(ev3_tm, power);
}

static void adjust_ramp_for_position(struct legoev3_motor_data *ev3_tm)
{
	long ramp_down_time = 0;
	long ramp_down_distance;

	/*
	 * The ramp down time is based on the current power level when regulation is off, and
	 * on the current speed when regulation is on - don't forget, we're not always at the
	 * end of the up ramp by the time we need to ramp down!
	 */

	if (TM_SPEED_REGULATION_OFF == ev3_tm->active_params.speed_regulation) {
		ramp_down_time  = abs((ev3_tm->active_params.ramp_down_sp * ev3_tm->duty_cycle) / 100);
	} else if (TM_SPEED_REGULATION_ON == ev3_tm->active_params.speed_regulation) {
		ramp_down_time  = abs((ev3_tm->active_params.ramp_down_sp * ev3_tm->speed)
			/ ev3_tm->info->max_speed);
	}

	/*
	 * The adjustment for ramp distance is to take into account that we'll have trouble hitting
	 * the position setpoint at low speeds...shorten the distance!
	 */

	ramp_down_distance = abs((ev3_tm->speed * ramp_down_time * 7) / (2000 * 10));

	/*
	 * Depending on the direction we are turning, figure out if we're going to overshoot
	 * the target position based on current speed. Note the calculation of ramp.down.end
	 * is relative to the current ramp.count, and that the ramp.down.start is recalculated
	 * backwards from the end so that the setpoint percentages work out properly!
	 *
	 * Remember, the timer callback function increments ramp.count by 2, so ramp.count
	 * always represents milliseconds!
	 */

	if (ev3_tm->ramp.direction > 0) {

		if ((ev3_tm->ramp.position_sp - ramp_down_distance) <= (ev3_tm->tacho + ev3_tm->irq_tacho)) {
			ev3_tm->ramp.up.end     = ev3_tm->ramp.count;
			ev3_tm->ramp.down.end   = ev3_tm->ramp.count + ramp_down_time;
			ev3_tm->ramp.down.start = ev3_tm->ramp.down.end - ev3_tm->active_params.ramp_down_sp;
		}

	} else {

		if ((ev3_tm->ramp.position_sp + ramp_down_distance) >= (ev3_tm->tacho + ev3_tm->irq_tacho)) {
			ev3_tm->ramp.up.end     = ev3_tm->ramp.count;
			ev3_tm->ramp.down.end   = ev3_tm->ramp.count + ramp_down_time;
			ev3_tm->ramp.down.start = ev3_tm->ramp.down.end - ev3_tm->active_params.ramp_down_sp;
		}
	}
}

/*
 * This function plays a key part in simplifying the calculation of ramp
 * progress in the code, and handles a number of special cases that can
 * cause odd behaviour.
 *
 * The strangest behaviour is when the numerator is two less than the
 * denominator - for cases where the denominator is small, this results in
 * very weird results for the speed, often many percent below the target
 * speed. ie 2/3 = 66 % , and the next iteration of the timer callback
 * adds 2 to the numerator so the ramp never gets re-evaluated!
 *
 * 1) If the denominator is less than or equal to the numerator, return 100
 * 2) If the denominator is 0
 * 3) If the denominator is two greater than the numerator, return 100
 */

static int calculate_ramp_progress(int numerator, int denominator)
{
	if (denominator <= (numerator + 2))
		return 100;
	else if (0 == denominator)
		return 100;
	else
		return((numerator * 100) / denominator);
}


static enum hrtimer_restart legoev3_motor_timer_callback(struct hrtimer *timer)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(timer, struct legoev3_motor_data, timer);
	const struct dc_motor_ops *motor_ops = ev3_tm->ldev->port->motor_ops;
	void *context = ev3_tm->ldev->port->context;
	int speed;
	bool reprocess = true;

	hrtimer_forward_now(timer, ktime_set(0, TACHO_MOTOR_POLL_NS));

	/* Continue with the actual calculations */

	speed = calculate_speed(ev3_tm);

	if (0 == ev3_tm->run)
		goto no_run;

	/*
	 * Update the ramp counter if we're in any of the ramp modes - the
	 * ramp counter always reflects milliseconds! Much cleaner this way.
	 *
	 * This has to be done outside of the main state processing
	 * loop, otherwise we can end up updating the counter multiple
	 * times.
	 */

	switch (ev3_tm->state) {
	case STATE_RAMP_UP:
	case STATE_RAMP_CONST:
	case STATE_POSITION_RAMP_DOWN:
	case STATE_RAMP_DOWN:
		ev3_tm->ramp.count = ev3_tm->ramp.count + 2;
		break;
	default:
		break;
	}

	while (reprocess) {

		/*
		 * Some cases (such as RAMP_XXX) may change the state of the
		 * handler and require reprocessing. If so, they must set the
		 * reprocess flag to force an extra evaluation
		 */

		reprocess = 0;

		switch (ev3_tm->state) {

		case STATE_RUN_FOREVER:
			ev3_tm->ramp.up.start = 0;

			/* Set the endpoint a long way out - an hour of milliseconds! */
			ev3_tm->ramp.down.end = 60*60*1000;

			/*
			 * First, we calculate ramp.up.full and ramp.down.full which are the point at which
			 * the ramp hits 100% of the setpoint - not the maximum theoretical speed or
			 * duty cycle.
			 *
			 * Why do we need this helper variable? It's because we also need to calculate the
			 * percentage completion of the ramp later on - and we must always maintain the
			 * rule that passing 100% to the update_motor_speed_or_power() function sets the
			 * speed or power to 100% of the setpoint, not the theoretical max.
			 */

			if (TM_SPEED_REGULATION_OFF == ev3_tm->active_params.speed_regulation) {
				ev3_tm->ramp.up.full    = ((abs(ev3_tm->active_params.duty_cycle_sp) * ev3_tm->active_params.ramp_up_sp  ) / 100);
				ev3_tm->ramp.down.full  = ((abs(ev3_tm->active_params.duty_cycle_sp) * ev3_tm->active_params.ramp_down_sp) / 100);
				ev3_tm->ramp.direction  = (ev3_tm->active_params.duty_cycle_sp >= 0 ? 1 : -1);

			} else if (TM_SPEED_REGULATION_ON == ev3_tm->active_params.speed_regulation) {
				ev3_tm->ramp.up.full    = ((abs(ev3_tm->active_params.speed_sp) * ev3_tm->active_params.ramp_up_sp)
					/ ev3_tm->info->max_speed);
				ev3_tm->ramp.down.full  = ((abs(ev3_tm->active_params.speed_sp) * ev3_tm->active_params.ramp_down_sp)
					/ ev3_tm->info->max_speed);
				ev3_tm->ramp.direction  = (ev3_tm->active_params.speed_sp >= 0 ? 1 : -1);
			}

			/*
			 * Now set the ramp.up and ramp.down start and end fields based on the values
			 * we just calculated for full in the previous step. We'll check for overlaps later
			 */

			ev3_tm->ramp.up.end     = ev3_tm->ramp.up.start + ev3_tm->ramp.up.full;
			ev3_tm->ramp.down.start = ev3_tm->ramp.down.end - ev3_tm->ramp.down.full;

			/*
			 * Now figure out if ramp.up.end is past ramp.down.start
			 * and adjust if needed using the intersection of the
			 * ramp up line and ramp down line.
			 *
			 * Basic high-school algebra and knowing ramp.up.end must
			 * equal ramp.down.start, and that the ramp.setpoint is
			 * reduced in proportion to how far the intersection is
			 * from the original end point gives us:
			 */

			if (ev3_tm->ramp.up.end > ev3_tm->ramp.down.start) {
				ev3_tm->ramp.up.end = ((ev3_tm->ramp.down.end * ev3_tm->active_params.ramp_up_sp)
					/ (ev3_tm->active_params.ramp_up_sp + ev3_tm->active_params.ramp_down_sp));
				ev3_tm->ramp.down.start = ev3_tm->ramp.up.end;
			}

			ev3_tm->state = STATE_SETUP_RAMP_REGULATION;
			reprocess = true;
			break;

		case STATE_SETUP_RAMP_POSITION:

			/*
			 * The position setups are a bit "interesting". We'll want
			 * use the same time based ramping mechanism, but we also
			 * need to take into account position.
			 *
			 * Since the ramp is a linear increase in velocity up to
			 * a setpoint, the position is the "area under the curve"
			 * which happens to be a triangle. The distance covered in
			 * the initial ramp up is 1/2(V*T) where V is measured in
			 * ticks per second and T is measured in milliseconds)
			 *
			 * It's easiest if we simply allow the speed to ramp up
			 * normally up to the speed setpoint and continuously
			 * estimate the ramp down start and end points based on
			 * the current speed. We have a nice attribute called
			 * spped and that value is calculated every time
			 * the speed is actually updated, about 500 times a
			 * second.
			 *
			 * Given the current speed and the ramp_down attribute, and
			 * assuming a linear ramp down from the current speed, we
			 * can estimate the time it will take to ramp down as:
			 *
			 * ramp_time = ((speed * ramp_down) / MaxPulsesPerSec[ev3_tm->motor_type] ) msec
			 *
			 * The actual speed in speed can then be used
			 * to estimate how far the motor will travel in that
			 * time as:
			 *
			 * ramp_distance = (( speed * ramp_time ) / (1000)) pulses
			 *
			 * Now it's a simple matter to figure out if we're within
			 * distance pulses of the desired endpoint, and then
			 * we can fill in the ramp_down values. The trick is that we
			 * must constantly update the estimate of the ramp_down
			 * start and endpoints, so it's best to do that before the
			 * state handlers!
			 */

			if (POSITION_ABSOLUTE == ev3_tm->position_mode)
				ev3_tm->ramp.position_sp = ev3_tm->active_params.position_sp;
			else
				ev3_tm->ramp.position_sp = ev3_tm->ramp.position_sp + ev3_tm->active_params.position_sp;

			/* TODO - These get recalculated in SETUP_RAMP_REGULATION - but it's OK */

			ev3_tm->ramp.direction = ((ev3_tm->ramp.position_sp >= (ev3_tm->tacho + ev3_tm->irq_tacho)) ? 1 : -1);

			ev3_tm->ramp.up.start = 0;

			/*
			 * The ramp transition point calculations depend on whether
			 * regulation is on or not
			 */
			if (TM_SPEED_REGULATION_OFF == ev3_tm->active_params.speed_regulation) {
				ev3_tm->ramp.up.full    = ((abs(ev3_tm->active_params.duty_cycle_sp) * ev3_tm->active_params.ramp_up_sp  ) / 100);
				ev3_tm->ramp.down.full  = ((abs(ev3_tm->active_params.duty_cycle_sp) * ev3_tm->active_params.ramp_down_sp) / 100);
			} else if (TM_SPEED_REGULATION_ON == ev3_tm->active_params.speed_regulation) {
				ev3_tm->ramp.up.full    = ((abs(ev3_tm->active_params.speed_sp) * ev3_tm->active_params.ramp_up_sp  )
					/ ev3_tm->info->max_speed);
				ev3_tm->ramp.down.full  = ((abs(ev3_tm->active_params.speed_sp) * ev3_tm->active_params.ramp_down_sp)
					/ ev3_tm->info->max_speed);
			}

			/*
			 * Now set the ramp.up and ramp.down start and end fields based on the values
			 * we just calculated for full in the previous step. We'll check for overlaps later
			 */

			ev3_tm->ramp.up.end     = ev3_tm->ramp.up.start + ev3_tm->ramp.up.full;

//			if (TM_REGULATION_OFF == ev3_tm->regulation_mode) {
//				ev3_tm->ramp.up.end = ev3_tm->ramp.up.start + ((abs(ev3_tm->duty_cycle_sp) * ev3_tm->ramp_up_sp) / 100);
//			} else if (TM_REGULATION_ON == ev3_tm->regulation_mode) {
//				ev3_tm->ramp.up.end = ev3_tm->ramp.up.start + ((abs(ev3_tm->speed_sp) * ev3_tm->ramp_up_sp)
//					/ ev3_tm->info->max_tacho_count_per_sec);
//			}

			/* TODO - Can this get handled in RAMP_CONST? */

			ev3_tm->ramp.down.end   =  60*60*1000;
			ev3_tm->ramp.down.start =  60*60*1000;

			ev3_tm->state = STATE_SETUP_RAMP_REGULATION;
			reprocess = true;
			break;

		case STATE_SETUP_RAMP_REGULATION:
			ev3_tm->ramp.count    = 0;

			ev3_tm->state = STATE_RAMP_UP;
			reprocess = true;
			break;

		/*
		 * The LIMITED_XXX functions have to handle the three phases (any of
		 * which are optional) of a motor move operation. It is assumed that
		 * when the run mode was set, the ramp factors were calculated.
		 *
		 * The LIMITED_XXX functions need to handle the following combinations:
		 *
		 * REGULATED_TIME    - Speed is regulated, ramping is time based
		 * REGULATED_TACHO   - Speed is regulated, ramping is tacho based
		 * UNREGULATED_TIME  - Speed is not regulated, ramping is time based
		 * UNREGULATED_TACHO - Speed is not regulated, ramping is tacho based
		 *
		 * When ramping, the code needs to figure out which combination is in
		 * use, and that's handled by a couple of booleans in the motor struct.
		 *
		 * Regardless of the direction of the ramp (up or down), the first part
		 * of the sequence is ramping up, the tail end of the sequence is
		 * ramping down.
		 */

		case STATE_RAMP_UP:
			/*
			 * Figure out if we're done ramping up - if yes set state to RAMP_CONST
			 * and allow states to get reprocessed
			 */
			if (IS_POS_CMD(ev3_tm->run_command)) {
				adjust_ramp_for_position(ev3_tm);
			}

			if (ev3_tm->ramp.count >= ev3_tm->ramp.up.end) {
				ev3_tm->state = STATE_RAMP_CONST;
				reprocess = true;
			}

			/* Figure out how far along we are in the ramp operation */

			ev3_tm->ramp.percent = calculate_ramp_progress( ev3_tm->ramp.count, ev3_tm->ramp.up.full);

			update_motor_speed_or_power(ev3_tm, ev3_tm->ramp.percent);
			break;

		case STATE_RAMP_CONST:
			/*
			 * Figure out if we're done with the const section - if yes set state to RAMP_DOWN
			 * and allow states to get reprocessed.
			 *
			 *  Just push out the end point if we're in TM_RUN_FOREVER mode
			 */

			if (ev3_tm->run_command == TM_COMMAND_RUN_FOREVER) {
				ev3_tm->ramp.down.start = ev3_tm->ramp.count;
				ev3_tm->ramp.down.end   = ev3_tm->ramp.count + ((abs(ev3_tm->active_params.duty_cycle_sp) * ev3_tm->active_params.ramp_down_sp) / 100);
			}

			/*
			 * In TM_RUN_POSITION mode, estimate where the end point would
			 * be, and ramp down if we're past it.
			 */

			else if (IS_POS_CMD(ev3_tm->run_command)) {
				adjust_ramp_for_position(ev3_tm);

				if (ev3_tm->ramp.count >= ev3_tm->ramp.down.start) {
					ev3_tm->state = STATE_POSITION_RAMP_DOWN;
					reprocess = true;
				}
			}

			/*
			 * This has to be here or else changing the speed_sp
			 * or the duty_cycle_sp when the motor is running won't work...
			 */

			update_motor_speed_or_power(ev3_tm, ev3_tm->ramp.percent);

			break;

		case STATE_POSITION_RAMP_DOWN:
			/* TODO - Maybe incorporate this into the adjust_ramp_for_position() function */

			if (ev3_tm->ramp.direction > 0) {

				if (ev3_tm->ramp.position_sp <= (ev3_tm->tacho + ev3_tm->irq_tacho /*+ (ev3_tm->power/4)*/)) {
					ev3_tm->ramp.down.end = ev3_tm->ramp.count;
				} else if (ev3_tm->ramp.down.end <= ev3_tm->ramp.count) {
					/* TODO - Increase ramp endpoint to nudge the ramp setpoint higher */
					ev3_tm->ramp.down.end = ev3_tm->ramp.count + 100;
				}

			} else {

				if (ev3_tm->ramp.position_sp >= (ev3_tm->tacho + ev3_tm->irq_tacho /*+ (ev3_tm->power/4)*/)) {
					ev3_tm->ramp.down.end = ev3_tm->ramp.count;
				} else if (ev3_tm->ramp.down.end <= ev3_tm->ramp.count) {
					/* TODO - Increase ramp endpoint to nudge the ramp setpoint higher */
					ev3_tm->ramp.down.end = ev3_tm->ramp.count + 100;
				}
			}

			ev3_tm->ramp.down.start = ev3_tm->ramp.down.end - ev3_tm->active_params.ramp_down_sp;

			/*
			 * NOTE: Intentional fallthrough to the STATE_RAMP_DOWN case
			 *
			 * The STATE_POSTION_RAMP_DOWN is busy recalculating the
			 * end point based on the current motor speed, so we can use
			 * the code in STATE_RAMP_DOWN to stop for us!
			 */

		case STATE_RAMP_DOWN:
			/*
			 * Figure out if we're done ramping down - if yes then
			 * decide whether to brake, coast, or leave the motor
			 * unchanged, and allow states to get reprocessed
			 */

			if (ev3_tm->ramp.count >= ev3_tm->ramp.down.end) {
				ev3_tm->state = STATE_STOP;
				reprocess = true;

			}
			/* Figure out how far along we are in the ramp operation */

			ev3_tm->ramp.percent = calculate_ramp_progress( (ev3_tm->ramp.down.end - ev3_tm->ramp.count), ev3_tm->ramp.down.full );

			update_motor_speed_or_power(ev3_tm, ev3_tm->ramp.percent);
			break;

		case STATE_STOP:
			/*
			 * Add in the irq_tacho for the current move so that we can use
			 * the value of irq_tacho in the HOLD mode - the current, real
			 * tacho reading is ALWAYS tacho + irq_tacho!
			 */

			if (IS_POS_CMD(ev3_tm->run_command)) {
				ev3_tm->irq_tacho  = (ev3_tm->tacho + ev3_tm->irq_tacho) - ev3_tm->ramp.position_sp;
				ev3_tm->tacho      = ev3_tm->ramp.position_sp;
			} else {
				ev3_tm->tacho      = ev3_tm->tacho + ev3_tm->irq_tacho;
				ev3_tm->irq_tacho  = 0;
			}

			ev3_tm->speed_reg_sp = 0;
			legoev3_motor_set_power(ev3_tm, 0);

			/*
			 * Reset the PID terms here to avoid having these terms influence the motor
			 * operation at the beginning of the next sequence. The most common issue is
			 * having some residual integral value briefly turn the motor on hard if
			 * we're ramping up slowly
			 */

			ev3_tm->pid.P = 0;
			ev3_tm->pid.I = 0;
			ev3_tm->pid.D = 0;

			reprocess     = true;
			ev3_tm->state = STATE_IDLE;
			break;

		case STATE_IDLE:
			ev3_tm->run = 0;
			schedule_work(&ev3_tm->notify_state_change_work);
			break;
		default:
			/* Intentionally left empty */
			break;
		}
	}

	if (ev3_tm->run && (TM_SPEED_REGULATION_ON == ev3_tm->active_params.speed_regulation))
		regulate_speed(ev3_tm);

no_run:
	/*
	 * Note, we get here even if we're running - so we need to check
	 * explicitly. These are some special cases to handle changes in the
	 * brake_mode when the motor is not running!
	 */

	if (!ev3_tm->run) {
		if (TM_STOP_COMMAND_COAST == ev3_tm->active_params.stop_command)
			motor_ops->set_command(context, DC_MOTOR_COMMAND_COAST);
		else if (TM_STOP_COMMAND_BRAKE == ev3_tm->active_params.stop_command)
			motor_ops->set_command(context, DC_MOTOR_COMMAND_BRAKE);
		else if (TM_STOP_COMMAND_HOLD == ev3_tm->active_params.stop_command)
			regulate_position(ev3_tm);
	}

	return HRTIMER_RESTART;
}

static void legoev3_motor_notify_state_change_work(struct work_struct *work)
{
	struct legoev3_motor_data *ev3_tm =
		container_of(work, struct legoev3_motor_data, notify_state_change_work);

	tacho_motor_notify_state_change(&ev3_tm->tm);
}

static int legoev3_motor_get_position(struct tacho_motor_device *tm, long *position)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(tm, struct legoev3_motor_data, tm);

	*position = ev3_tm->tacho + ev3_tm->irq_tacho;

	return 0;
}

static int legoev3_motor_set_position(struct tacho_motor_device *tm, long position)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(tm, struct legoev3_motor_data, tm);

	ev3_tm->irq_tacho	 = 0;
	ev3_tm->tacho		 = position;
	ev3_tm->ramp.position_sp = position;

	return 0;
}

static int legoev3_motor_get_count_per_rot(struct tacho_motor_device *tm)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(tm, struct legoev3_motor_data, tm);

	return ev3_tm->info->count_per_rot;
}

static int legoev3_motor_get_duty_cycle(struct tacho_motor_device *tm,
					int *duty_cycle)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(tm, struct legoev3_motor_data, tm);

	*duty_cycle = ev3_tm->duty_cycle;

	return 0;
}

static int legoev3_motor_get_state(struct tacho_motor_device *tm)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(tm, struct legoev3_motor_data, tm);

	unsigned state = 0;
	if (ev3_tm->state < STATE_STOP) {
		state |= BIT(TM_STATE_RUNNING);
		if (ev3_tm->state > STATE_RUN_FOREVER)
			state |= BIT(TM_STATE_RAMPING);
	}
	/* TODO: implement stall detection */

	return state;
}

static int legoev3_motor_get_speed(struct tacho_motor_device *tm, int *speed)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(tm, struct legoev3_motor_data, tm);

	*speed = ev3_tm->speed;

	return 0;
}

static unsigned legoev3_motor_get_speed_regulations(struct tacho_motor_device *tm)
{
	return BIT(TM_SPEED_REGULATION_OFF) | BIT(TM_SPEED_REGULATION_ON);
}

static unsigned legoev3_motor_get_stop_commands(struct tacho_motor_device *tm)
{
	return BIT(TM_STOP_COMMAND_COAST) | BIT(TM_STOP_COMMAND_BRAKE) |
		BIT(TM_STOP_COMMAND_HOLD);
}

static int legoev3_motor_get_speed_Kp(struct tacho_motor_device *tm)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(tm, struct legoev3_motor_data, tm);

	return ev3_tm->pid.speed_Kp;
}

static int legoev3_motor_set_speed_Kp(struct tacho_motor_device *tm, int k)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(tm, struct legoev3_motor_data, tm);

	ev3_tm->pid.speed_Kp = k;

	return 0;
}

static int legoev3_motor_get_speed_Ki(struct tacho_motor_device *tm)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(tm, struct legoev3_motor_data, tm);

	return ev3_tm->pid.speed_Ki;
}

static int legoev3_motor_set_speed_Ki(struct tacho_motor_device *tm, int k)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(tm, struct legoev3_motor_data, tm);

	ev3_tm->pid.speed_Ki = k;

	return 0;
}

static int legoev3_motor_get_speed_Kd(struct tacho_motor_device *tm)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(tm, struct legoev3_motor_data, tm);

	return ev3_tm->pid.speed_Kd;
}

static int legoev3_motor_set_speed_Kd(struct tacho_motor_device *tm, int k)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(tm, struct legoev3_motor_data, tm);

	ev3_tm->pid.speed_Kd = k;

	return 0;
}

static unsigned legoev3_motor_get_commands (struct tacho_motor_device *tm)
{
	return BIT(TM_COMMAND_RUN_FOREVER) | BIT (TM_COMMAND_RUN_TO_ABS_POS)
		| BIT(TM_COMMAND_RUN_TO_REL_POS)
		| BIT(TM_COMMAND_STOP) | BIT(TM_COMMAND_RESET);
}

static int legoev3_motor_send_command(struct tacho_motor_device *tm,
				      enum tacho_motor_command command)
{
	struct legoev3_motor_data *ev3_tm =
			container_of(tm, struct legoev3_motor_data, tm);

	ev3_tm->active_params = ev3_tm->tm.params;
	ev3_tm->run_command = command;

	if (command == TM_COMMAND_RESET) {
		legoev3_motor_reset(ev3_tm);
		return 0;
	}

	/*
	 * If the motor is currently running and we're asked to stop
	 * it, then figure out how we're going to stop it - maybe we
	 * need to ramp it down first!
	 */

	if (IS_RUN_CMD(command) && ev3_tm->state != STATE_IDLE) {
		ev3_tm->ramp.down.start = ev3_tm->ramp.count;
		ev3_tm->ramp.down.end   = ev3_tm->active_params.ramp_down_sp;

		if (TM_COMMAND_RUN_FOREVER == command)
			ev3_tm->state = STATE_RAMP_DOWN;
		else if (IS_POS_CMD(command))
			ev3_tm->state = STATE_STOP;
	}

	/*
	 * If the motor is currently idle and we're asked to run
	 * it, then figure out how we're going to get things started
	 */

	else if (IS_RUN_CMD(command) && ev3_tm->state == STATE_IDLE) {
		if (TM_COMMAND_RUN_FOREVER == command)
			ev3_tm->state = STATE_RUN_FOREVER;
		else if (IS_POS_CMD(command))
			ev3_tm->state = STATE_SETUP_RAMP_POSITION;
	}

	/* Otherwise, put the motor in STOP state - it will eventually stop */

	else if (TM_COMMAND_STOP == command) {
		ev3_tm->state = STATE_STOP;
	}

	if (TM_COMMAND_RUN_TO_ABS_POS)
		ev3_tm->position_mode = POSITION_ABSOLUTE;
	if (TM_COMMAND_RUN_TO_REL_POS)
		ev3_tm->position_mode = POSITION_RELATIVE;

	/*
	 * what's going on here - why is run always set to 1?
	 *
	 * the answer is that we check for run == 0 as the first condition
	 * at the top of this function. if it's set, then the next motor state
	 * is stop_motor, but it won't be evaluated if run == 0
	 *
	 * so we always force the state machine to run once, and count on the
	 * state machine to dtrt (do the right thing). This avoids setting motor
	 * power in weird places
	 */

	ev3_tm->run = 1;

	return 0;
}

static const struct tacho_motor_ops legoev3_motor_ops = {
	.get_position		= legoev3_motor_get_position,
	.set_position		= legoev3_motor_set_position,

	.get_state		= legoev3_motor_get_state,
	.get_count_per_rot	= legoev3_motor_get_count_per_rot,
	.get_duty_cycle		= legoev3_motor_get_duty_cycle,
	.get_speed		= legoev3_motor_get_speed,

	.get_commands		= legoev3_motor_get_commands,
	.send_command		= legoev3_motor_send_command,

	.get_speed_regulations	= legoev3_motor_get_speed_regulations,
	.get_stop_commands	= legoev3_motor_get_stop_commands,

	.get_speed_Kp		= legoev3_motor_get_speed_Kp,
	.set_speed_Kp		= legoev3_motor_set_speed_Kp,
	.get_speed_Ki		= legoev3_motor_get_speed_Ki,
	.set_speed_Ki		= legoev3_motor_set_speed_Ki,
	.get_speed_Kd		= legoev3_motor_get_speed_Kd,
	.set_speed_Kd		= legoev3_motor_set_speed_Kd,
};


static int legoev3_motor_probe(struct lego_device *ldev)
{
	struct legoev3_motor_data *ev3_tm;
	struct ev3_motor_platform_data *pdata = ldev->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;
	if (WARN_ON(!ldev->port->motor_ops))
		return -EINVAL;
	if (WARN_ON(!ldev->entry_id))
		return -EINVAL;

	ev3_tm = kzalloc(sizeof(struct legoev3_motor_data), GFP_KERNEL);
	if (!ev3_tm)
		return -ENOMEM;

	ev3_tm->ldev = ldev;
	ev3_tm->info = &legoev3_motor_defs[ldev->entry_id->driver_data];

	ev3_tm->tm.driver_name = ldev->entry_id->name;
	ev3_tm->tm.port_name = ldev->port->port_name;
	ev3_tm->tm.ops = &legoev3_motor_ops;
	ev3_tm->tm.supports_encoder_polarity = true;
	ev3_tm->tm.supports_ramping = true;

	err = register_tacho_motor(&ev3_tm->tm, &ldev->dev);
	if (err)
		goto err_register_tacho_motor;

	dev_set_drvdata(&ldev->dev, ev3_tm);

	/* Here's where we set up the port pins on a per-port basis */
	if (request_irq(gpio_to_irq(pdata->tacho_int_gpio), tacho_motor_isr, 0,
				    dev_name(&ldev->port->dev), ev3_tm))
		goto err_dev_request_irq;

	irq_set_irq_type(gpio_to_irq(pdata->tacho_int_gpio),
			 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);

	hrtimer_init(&ev3_tm->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ev3_tm->timer.function = legoev3_motor_timer_callback;

	INIT_WORK(&ev3_tm->notify_state_change_work,
		  legoev3_motor_notify_state_change_work);

	hrtimer_start(&ev3_tm->timer, ktime_set(0, TACHO_MOTOR_POLL_NS),
		      HRTIMER_MODE_REL);

	legoev3_motor_reset(ev3_tm);

	/*
	 * PID values are not included in the reset.
	 */
	ev3_tm->pid.speed_Kp  = ev3_tm->info->speed_pid_k.p;
	ev3_tm->pid.speed_Ki  = ev3_tm->info->speed_pid_k.i;
	ev3_tm->pid.speed_Kd  = ev3_tm->info->speed_pid_k.d;

	return 0;

err_dev_request_irq:
	dev_set_drvdata(&ldev->dev, NULL);
	unregister_tacho_motor(&ev3_tm->tm);
err_register_tacho_motor:
	kfree(ev3_tm);

	return err;
}

static int legoev3_motor_remove(struct lego_device *ldev)
{
	struct ev3_motor_platform_data *pdata = ldev->dev.platform_data;
	struct legoev3_motor_data *ev3_tm = dev_get_drvdata(&ldev->dev);

	hrtimer_cancel(&ev3_tm->timer);
	cancel_work_sync(&ev3_tm->notify_state_change_work);
	free_irq(gpio_to_irq(pdata->tacho_int_gpio), ev3_tm);
	dev_set_drvdata(&ldev->dev, NULL);
	unregister_tacho_motor(&ev3_tm->tm);
	kfree(ev3_tm);
	return 0;
}

struct lego_device_id legoev3_motor_driver_id_table[] = {
	LEGO_DEVICE_ID(LEGO_EV3_LARGE_MOTOR),
	LEGO_DEVICE_ID(LEGO_EV3_MEDIUM_MOTOR),
	LEGO_DEVICE_ID(FIRGELLI_L12_EV3),
};

/* TODO: Make this a bus driver attribute instead so it is present for all drivers on the lego bus */
static ssize_t driver_names_show(struct device_driver *drv, char *buf)
{
	int i;
	int size = 0;

	for (i = 0; i < NUM_LEGOEV3_MOTOR_ID; i++) {
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

