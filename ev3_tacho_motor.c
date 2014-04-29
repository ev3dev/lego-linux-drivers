/*
 * EV3 Tacho Motor device driver for LEGO Mindstorms EV3
 *
 * Copyright (C) 2013-2014 Ralph Hempel <rhempel@hempeldesigngroup.com>
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
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/pwm.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_output_port.h>
#include <linux/legoev3/tacho_motor_class.h>

#include <mach/time.h>

#include <asm/bug.h>

#define TACHO_MOTOR_POLL_NS	(  2000000)	                /* 2 msec */

#define   TACHO_SAMPLES           128

#define   MAX_PWM_CNT                   (10000)
#define   MAX_SPEED                     (100)
#define   MAX_POWER                     (100)
#define   MAX_SYNC_MOTORS               (2)

enum {
	MOTOR_TYPE_0 = 0,
	MOTOR_TYPE_1,
	MOTOR_TYPE_2,
	MOTOR_TYPE_3,
	MOTOR_TYPE_4,
	MOTOR_TYPE_5,
	MOTOR_TYPE_6,
	MOTOR_TYPE_TACHO,
	MOTOR_TYPE_MINITACHO,
	MOTOR_TYPE_NEWTACHO,
	MOTOR_TYPE_10,
	MOTOR_TYPE_11,
	MOTOR_TYPE_12,
	MOTOR_TYPE_13,
	MOTOR_TYPE_14,
	MOTOR_TYPE_15,
	NO_OF_MOTOR_TYPES,
};

enum {
	SAMPLES_PER_SPEED_BELOW_40 = 0,
	SAMPLES_PER_SPEED_ABOVE_40 = 1,
	SAMPLES_PER_SPEED_ABOVE_60 = 2,
	SAMPLES_PER_SPEED_ABOVE_80 = 3,
	NO_OF_SAMPLE_STEPS = 4
};

enum {
	UNKNOWN, FORWARD, REVERSE, BRAKE, COAST,
};

struct ev3_tacho_motor_data {
	struct tacho_motor_device tm;

	struct legoev3_port_device *out_port;
	struct legoev3_port_device *motor_port;

	struct hrtimer timer;

	unsigned tacho_samples[TACHO_SAMPLES];
	unsigned tacho_samples_head;

	bool got_new_sample;

	unsigned samples_per_speed;
	unsigned dir_chg_samples;

	unsigned counts_per_pulse;
	unsigned pulses_per_second;

#warning "The class mutex interlock is not implemented - should be up at device level to allow busy indication"

	bool class_mutex;
	bool irq_mutex;

	struct {
		struct {
			int start;
			int end;
		} up;

		struct {
			int start;
			int end;
		} down;

		int setpoint;
		int setpoint_sign;
		int position_setpoint;
		int offset;
		int count; /* This must be set to either tacho or time increment! */
	} ramp;

	struct {
		int P;
		int I;
		int D;
		int prev_speed_error;
		int prev_position_error;
	} pid;

	int speed_reg_setpoint;
	int run_direction;
	int set_direction;

	int run;

	int motor_type;

	int tacho;
	int irq_tacho; /* tacho and irq_tacho combine to make position - change name to pulse? */

	int speed;
	int power;
	int state;

	long speed_setpoint;
	long time_setpoint;
	long position_setpoint;

	long run_mode;
	long regulation_mode;
	long brake_mode;
	long hold_mode;
	long position_mode;
	long polarity_mode;

	long ramp_up;
	long ramp_down;
};

static unsigned SamplesPerSpeed[NO_OF_MOTOR_TYPES][NO_OF_SAMPLE_STEPS] = { { 2,
        2, 2, 2 }, /* Motor Type  0             */
{ 2, 2, 2, 2 }, /* Motor Type  1             */
{ 2, 2, 2, 2 }, /* Motor Type  2             */
{ 2, 2, 2, 2 }, /* Motor Type  3             */
{ 2, 2, 2, 2 }, /* Motor Type  4             */
{ 2, 2, 2, 2 }, /* Motor Type  5             */
{ 2, 2, 2, 2 }, /* Motor Type  6             */
{ 4, 16, 32, 64 }, /* Motor Type  7 - TACHO     */
{ 2, 4, 8, 16 }, /* Motor Type  8 - MINITACHO */
{ 2, 2, 2, 2 }, /* Motor Type  9 - NEWTACHO  */
{ 2, 2, 2, 2 }, /* Motor Type 10             */
{ 2, 2, 2, 2 }, /* Motor Type 11             */
{ 2, 2, 2, 2 }, /* Motor Type 12             */
{ 2, 2, 2, 2 }, /* Motor Type 13             */
{ 2, 2, 2, 2 }, /* Motor Type 14             */
{ 2, 2, 2, 2 }, /* Motor Type 15             */
};

static unsigned CountsPerPulse[NO_OF_MOTOR_TYPES] = { 0, /* Motor Type  0             */
0, /* Motor Type  1             */
0, /* Motor Type  2             */
0, /* Motor Type  3             */
0, /* Motor Type  4             */
0, /* Motor Type  5             */
0, /* Motor Type  6             */
3300000, /* Motor Type  7 - TACHO     */
2062500, /* Motor Type  8 - MINITACHO */
0, /* Motor Type  9 - NEWTACHO  */
0, /* Motor Type 10             */
0, /* Motor Type 11             */
0, /* Motor Type 12             */
0, /* Motor Type 13             */
0, /* Motor Type 14             */
0, /* Motor Type 15             */
};

static void set_samples_per_speed(struct ev3_tacho_motor_data *ev3_tm,
                                  int speed)
{
	if (speed > 80) {
		ev3_tm->samples_per_speed =
		        SamplesPerSpeed[ev3_tm->motor_type][SAMPLES_PER_SPEED_ABOVE_80];
	} else if (speed > 60) {
		ev3_tm->samples_per_speed =
		        SamplesPerSpeed[ev3_tm->motor_type][SAMPLES_PER_SPEED_ABOVE_60];
	} else if (speed > 40) {
		ev3_tm->samples_per_speed =
		        SamplesPerSpeed[ev3_tm->motor_type][SAMPLES_PER_SPEED_ABOVE_40];
	} else {
		ev3_tm->samples_per_speed =
		        SamplesPerSpeed[ev3_tm->motor_type][SAMPLES_PER_SPEED_BELOW_40];
	}
}

static irqreturn_t tacho_motor_isr(int irq, void *id)
{
	struct ev3_tacho_motor_data *ev3_tm = id;
	struct ev3_motor_platform_data *pdata =
	        ev3_tm->motor_port->dev.platform_data;

	bool int_state = gpio_get_value(pdata->tacho_int_gpio);
	bool dir_state = !gpio_get_value(pdata->tacho_dir_gpio);

	unsigned long timer = legoev3_hires_timer_read();

	unsigned next_sample;

	int next_direction;

	/* Grab the next incremental sample timestamp */

	next_sample = (ev3_tm->tacho_samples_head + 1) % TACHO_SAMPLES;

	ev3_tm->tacho_samples[next_sample] = timer;
	ev3_tm->tacho_samples_head = next_sample;

	/* If the speed is high enough, just update the tacho counter based on direction */

	if ((35 < ev3_tm->speed) || (-35 > ev3_tm->speed)) {

		if (ev3_tm->dir_chg_samples < (TACHO_SAMPLES - 1))
			ev3_tm->dir_chg_samples++;

	} else {

		/* Update the tacho count and motor direction for low speed, taking
		 * advantage of the fact that if state and dir match, then the motor
		 * is turning FORWARD!
		 */

		if (int_state == dir_state)
			next_direction = FORWARD;
		else
			next_direction = REVERSE;

		/* If the saved and next direction states match, then update the dir_chg_sample count */

		if (ev3_tm->run_direction == next_direction) {
			if (ev3_tm->dir_chg_samples < (TACHO_SAMPLES - 1))
				ev3_tm->dir_chg_samples++;
		} else {
			ev3_tm->dir_chg_samples = 0;
		}

		ev3_tm->run_direction = next_direction;
	}

	ev3_tm->irq_mutex = true;

	if (FORWARD == ev3_tm->run_direction)
		ev3_tm->irq_tacho++;
	else
		ev3_tm->irq_tacho--;

	ev3_tm->got_new_sample = true;

	ev3_tm->irq_mutex = false;

	return IRQ_HANDLED;
}

static void ev3_tacho_motor_forward(struct ev3_tacho_motor_data *ev3_tm)
{
	struct ev3_motor_platform_data *pdata =
	        ev3_tm->motor_port->dev.platform_data;

	if (FORWARD != ev3_tm->set_direction) {
		gpio_direction_output(pdata->motor_dir0_gpio, 1);
		gpio_direction_input(pdata->motor_dir1_gpio);
		ev3_tm->set_direction = FORWARD;
	}
}
static void ev3_tacho_motor_reverse(struct ev3_tacho_motor_data *ev3_tm)
{
	struct ev3_motor_platform_data *pdata =
	        ev3_tm->motor_port->dev.platform_data;

	if (REVERSE != ev3_tm->set_direction) {
		gpio_direction_input(pdata->motor_dir0_gpio);
		gpio_direction_output(pdata->motor_dir1_gpio, 1);
		ev3_tm->set_direction = REVERSE;
	}
}
static void ev3_tacho_motor_brake(struct ev3_tacho_motor_data *ev3_tm)
{
	struct ev3_motor_platform_data *pdata =
	        ev3_tm->motor_port->dev.platform_data;

#warning "The LEGO code sets the regulation power to an opposite level to stop the motor hard"

	if (BRAKE != ev3_tm->set_direction) {
		gpio_direction_output(pdata->motor_dir0_gpio, 1);
		gpio_direction_output(pdata->motor_dir1_gpio, 1);
		ev3_tm->set_direction = BRAKE;
	}
}
static void ev3_tacho_motor_coast(struct ev3_tacho_motor_data *ev3_tm)
{
	struct ev3_motor_platform_data *pdata =
	        ev3_tm->motor_port->dev.platform_data;

	if (COAST != ev3_tm->set_direction) {
		gpio_direction_output(pdata->motor_dir0_gpio, 0);
		gpio_direction_output(pdata->motor_dir1_gpio, 0);
		ev3_tm->set_direction = COAST;
	}
}

static void ev3_tacho_motor_set_power(struct ev3_tacho_motor_data *ev3_tm,
                                      int power)
{
	int err;
	struct ev3_motor_platform_data *pdata =
	        ev3_tm->motor_port->dev.platform_data;

	/* Bail out early if there's no change in the power setting */

	if (ev3_tm->power == power)
		goto no_change_power;

	/* Make sure power is within a reasonable range */

	if (power > MAX_POWER) {
		power = MAX_POWER;
	} else if (power < -MAX_POWER) {
		power = -MAX_POWER;
	}

	if (0 < power) {
		ev3_tacho_motor_forward(ev3_tm);
	} else if (0 > power) {
		ev3_tacho_motor_reverse(ev3_tm);
	} else {
		if (BRAKE_ON == ev3_tm->brake_mode)
			ev3_tacho_motor_brake(ev3_tm);
		else
			ev3_tacho_motor_coast(ev3_tm);
	}

	/* The power sets the duty cycle - 100% power == 100% duty cycle */
	err = pwm_config(pdata->pwm, pdata->pwm->period * abs(power) / 100,
			 pdata->pwm->period);

	if (err) {
		dev_err(&ev3_tm->motor_port->dev,
		        "%s: Failed to set pwm duty percent! (%d)\n", __func__,
		        err);
	}

	no_change_power:

	/* Note, we get here all the time, we always do the assignment since
	 * it does not hurt anything, and it's required because you can't goto
	 * the very end of a void function - who knew?
	 */

	ev3_tm->power = power;
}

static void ev3_tacho_motor_reset(struct ev3_tacho_motor_data *ev3_tm)
{
	struct ev3_motor_platform_data *pdata =
	        ev3_tm->motor_port->dev.platform_data;

	/* This is the same as initializing a motor - we will set everything
	 * to default values, as if it had just been plugged in
	 */

	memset(ev3_tm->tacho_samples, 0, sizeof(unsigned) * TACHO_SAMPLES);

	ev3_tm->tacho_samples_head = 0;
	ev3_tm->got_new_sample = false;
	ev3_tm->samples_per_speed =
	        SamplesPerSpeed[MOTOR_TYPE_TACHO][SAMPLES_PER_SPEED_BELOW_40];
	ev3_tm->dir_chg_samples = 0;
	ev3_tm->counts_per_pulse = CountsPerPulse[MOTOR_TYPE_TACHO];
	ev3_tm->pulses_per_second = 0;
	ev3_tm->class_mutex = false;
	ev3_tm->irq_mutex = false;
	ev3_tm->ramp.up.start = 0;
	ev3_tm->ramp.up.end = 0;
	ev3_tm->ramp.down.start = 0;
	ev3_tm->ramp.down.end = 0;
	ev3_tm->ramp.setpoint = 0;
	ev3_tm->ramp.setpoint_sign = 0;
	ev3_tm->ramp.position_setpoint = 0;
	ev3_tm->ramp.offset = 0;
	ev3_tm->ramp.count = 0;
	ev3_tm->pid.P = 0;
	ev3_tm->pid.I = 0;
	ev3_tm->pid.D = 0;
	ev3_tm->pid.prev_speed_error = 0;
	ev3_tm->pid.prev_position_error = 0;
	ev3_tm->speed_reg_setpoint = 0;
	ev3_tm->run_direction = UNKNOWN;
	ev3_tm->set_direction = UNKNOWN;
	ev3_tm->run = 0;

	if (pdata->motor_type == MOTOR_MINITACHO)
		ev3_tm->motor_type = MOTOR_TYPE_MINITACHO;
	else if (pdata->motor_type == MOTOR_TACHO)
		ev3_tm->motor_type = MOTOR_TYPE_TACHO;
	else
		ev3_tm->motor_type = MOTOR_TYPE_TACHO;

	ev3_tm->tacho = 0;
	ev3_tm->irq_tacho = 0;
	ev3_tm->speed = 0;
	ev3_tm->power = 0;
	ev3_tm->state = STATE_IDLE;
	ev3_tm->speed_setpoint = 0;
	ev3_tm->time_setpoint = 0;
	ev3_tm->position_setpoint = 0;
	ev3_tm->run_mode = RUN_FOREVER;
	ev3_tm->regulation_mode = REGULATION_OFF;
	ev3_tm->brake_mode = BRAKE_OFF;
	ev3_tm->hold_mode = HOLD_OFF;
	ev3_tm->position_mode = POSITION_ABSOLUTE;
	ev3_tm->polarity_mode = POLARITY_POSITIVE;
	ev3_tm->ramp_up = 0;
	ev3_tm->ramp_down = 0;
}
;

/*
 *! \brief    calculate_speed
 *
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
 *          resulting in minimum timer value og: 1,25mS / (1/(33MHz)) = 41250 (approximately)
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

static bool calculate_speed(struct ev3_tacho_motor_data *ev3_tm)
{
	unsigned DiffIdx;
	unsigned Diff;

	long speed;

	bool speed_updated = false;

#warning "Don't run this if we're updating the ev3_tm in the isr!"

	/* Determine the approximate speed of the motor using the difference
	 * in time between this tacho pulse and the previous pulse.
	 *
	 * The old code had a conditional that forced the difference to be at
	 * least 1 by checking for the special case of a zero difference, which
	 * would be almost impossible to acheive in practice.
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

	DiffIdx = ev3_tm->tacho_samples_head;

#warning "This should really be a boolean value that gets set at the ISR level"
#warning "Can/Should we change this to not set_samples_per_speed evry time we're called?"

	if (ev3_tm->dir_chg_samples >= 1) {

		Diff = ev3_tm->tacho_samples[DiffIdx]
		        - ev3_tm->tacho_samples[(DiffIdx + TACHO_SAMPLES - 1)
		                % TACHO_SAMPLES];

		Diff |= 1;

		set_samples_per_speed(ev3_tm, ev3_tm->counts_per_pulse / Diff);
	}

	/* Now get a better estimate of the motor speed by using the total
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

	if (ev3_tm->got_new_sample
	        && (ev3_tm->dir_chg_samples >= ev3_tm->samples_per_speed)) {

		Diff = ev3_tm->tacho_samples[DiffIdx]
		        - ev3_tm->tacho_samples[(DiffIdx + TACHO_SAMPLES
		                - ev3_tm->samples_per_speed) % TACHO_SAMPLES];

		Diff |= 1;

		speed = (ev3_tm->counts_per_pulse * ev3_tm->samples_per_speed)
		        / Diff;

		ev3_tm->pulses_per_second = (33000000
		        * ev3_tm->samples_per_speed) / Diff;

		/* And do some cleanup to limit the max and get direction right */

		if (speed > MAX_SPEED)
			speed = MAX_SPEED;

		if (ev3_tm->run_direction == REVERSE)
			speed = -speed;

		speed_updated = true;

		ev3_tm->got_new_sample = false;

	} else if (ev3_tm->counts_per_pulse
	        < (legoev3_hires_timer_read() - ev3_tm->tacho_samples[DiffIdx])) {

		ev3_tm->dir_chg_samples = 0;

		speed = 0;

#warning "This is where we can put in a calculation for a stalled motor!"

		speed_updated = true;
	}

	if (speed_updated)
		ev3_tm->speed = speed;

	return (speed_updated);
}

static void regulate_speed(struct ev3_tacho_motor_data *ev3_tm)
{
	int power;
	int speed_error;

	/* Make sure speed_reg_setpoint is within a reasonable range */

	if (ev3_tm->speed_reg_setpoint > MAX_SPEED) {
		ev3_tm->speed_reg_setpoint = MAX_SPEED;
	} else if (ev3_tm->speed_reg_setpoint < -MAX_SPEED) {
		ev3_tm->speed_reg_setpoint = -MAX_SPEED;
	}

	speed_error = ev3_tm->speed_reg_setpoint - ev3_tm->speed;

#warning "Implement an attribute set for PID constants that adjusts based on speed"

	if (MOTOR_TYPE_MINITACHO == ev3_tm->motor_type) {
		ev3_tm->pid.P = speed_error * 4;
		ev3_tm->pid.I = ((ev3_tm->pid.I * 9) / 10) + (speed_error / 3);
		ev3_tm->pid.D = (((speed_error - ev3_tm->pid.prev_speed_error)
		        * 4) / 2) * 40;

	} else if (MOTOR_TYPE_TACHO == ev3_tm->motor_type) {
		ev3_tm->pid.P = speed_error * 2;
		ev3_tm->pid.I = ((ev3_tm->pid.I * 9) / 10) + (speed_error / 4);
		ev3_tm->pid.D = (((speed_error - ev3_tm->pid.prev_speed_error)
		        * 4) / 2) * 40;

	} else {
		/* This space intentionally left blank! */
	}

	ev3_tm->pid.prev_speed_error = speed_error;

	power = (ev3_tm->power)
	        + ((ev3_tm->pid.P + ev3_tm->pid.I + ev3_tm->pid.D) / 100);

	ev3_tacho_motor_set_power(ev3_tm, power);
}

/* This function changes either the actual power setting for the motor, or the speed
 * regulation setpoint, depending on whether the regulation_mode is on or off.
 *
 * Note that it is assumed by this function and all of its callers that
 * ev3_tacho_motor_set_power() checks whether there's an actual change
 * as well as limiting the range of input values.
 *
 * Similarly, the regulation function must verify the range of ev3_tm->speed_reg_setpoint
 * to avoid unreasonable values.
 *
 * By pushing the checks further down the line, we simplify the higher levels
 * of code!
 */

static void update_motor_speed_or_power(struct ev3_tacho_motor_data *ev3_tm,
                                        int setpoint)
{
	if (REGULATION_OFF == ev3_tm->regulation_mode) {
		ev3_tacho_motor_set_power(ev3_tm, setpoint);
	} else {
		ev3_tm->speed_reg_setpoint = setpoint;
	}
}

static void regulate_position(struct ev3_tacho_motor_data *ev3_tm)
{
	int power;
	int position_error;

	/* Make sure that the irq_tacho value has been set to a value that represents the
	 * current error from the desired position so we can drive the motor towards
	 * the desired position hold point.
	 */

	position_error = 0 - ev3_tm->irq_tacho;

	if (MOTOR_TYPE_MINITACHO == ev3_tm->motor_type) {
		ev3_tm->pid.P = position_error * 400;
		ev3_tm->pid.I = ((ev3_tm->pid.I * 99) / 100)
		        + (position_error / 1);
		ev3_tm->pid.D = (((position_error
		        - ev3_tm->pid.prev_position_error) * 4) / 2) * 2;

	} else if (MOTOR_TYPE_TACHO == ev3_tm->motor_type) {
		ev3_tm->pid.P = position_error * 400;
		ev3_tm->pid.I = ((ev3_tm->pid.I * 99) / 100)
		        + (position_error / 1);
		ev3_tm->pid.D = (((position_error
		        - ev3_tm->pid.prev_position_error) * 4) / 2) * 2;

	} else {
		/* This space intentionally left blank! */
	}

	ev3_tm->pid.prev_position_error = position_error;

	power = ((ev3_tm->pid.P + ev3_tm->pid.I + ev3_tm->pid.D) / 100);

	ev3_tacho_motor_set_power(ev3_tm, power);
}

static void adjust_ramp_for_position(struct ev3_tacho_motor_data *ev3_tm)
{
#warning "Power might not be the right thing to use here, but it's higher than speed for better margin"
#warning "We only do the calcs on start point if ew are NOT in ramp_down mode."

	long ramp_time = ((abs(ev3_tm->power) * ev3_tm->ramp_down) / 100);
	long ramp_distance = ((ev3_tm->pulses_per_second * ramp_time) / 2000);

	/* If we're turning in the forward direction.... */

	if (ev3_tm->ramp.setpoint_sign > 0) {

		if ((ev3_tm->ramp.position_setpoint - ramp_distance)
		        <= (ev3_tm->tacho + ev3_tm->irq_tacho)) {
			ev3_tm->ramp.up.end = ev3_tm->ramp.count;
			ev3_tm->ramp.down.start = ev3_tm->ramp.count;
			ev3_tm->ramp.down.end = ev3_tm->ramp.down.start
			        + (ramp_time / 2);
		}

	} else {

		if ((ev3_tm->ramp.position_setpoint + ramp_distance)
		        >= (ev3_tm->tacho + ev3_tm->irq_tacho)) {
			ev3_tm->ramp.up.end = ev3_tm->ramp.count;
			ev3_tm->ramp.down.start = ev3_tm->ramp.count;
			ev3_tm->ramp.down.end = ev3_tm->ramp.down.start
			        + (ramp_time / 2);
		}
	}
}

static enum hrtimer_restart ev3_tacho_motor_timer_callback(
        struct hrtimer *timer)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(timer,
	        struct ev3_tacho_motor_data, timer);

	int speed;
	int ramp_time;

	int setpoint;

	bool reprocess = true;

	hrtimer_forward_now(timer, ktime_set(0, TACHO_MOTOR_POLL_NS));

	/* Here's where the business end of things starts - update the tacho data that's
	 *   shared with the world
	 */

// 	/* Early exit from function if someone is reading the struct! */
// 
// 	if (ev3_tm->mutex )
// 		return HRTIMER_RESTART;
	/* Continue with the actual calculations */

	speed = calculate_speed(ev3_tm);

	if (!ev3_tm->run)
		goto no_run;

	/* Update the ramp counter if we're in any of the ramp modes.
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

	default:
		break;
	}

	while (reprocess) {

		/* Some cases (such as RAMP_XXX) may change the state of the
		 * handler and require reprocessing. If so, they must set the
		 * reprocess flag to force an extra evaluation
		 */

		reprocess = 0;

		switch (ev3_tm->state) {

		case STATE_RUN_FOREVER:

			update_motor_speed_or_power(ev3_tm,
			        ev3_tm->speed_setpoint);
			break;

		case STATE_SETUP_RAMP_TIME:
			ev3_tm->ramp.up.start = 0;
			ev3_tm->ramp.up.end = ev3_tm->ramp.up.start
			        + ((abs(ev3_tm->speed_setpoint)
			                * ev3_tm->ramp_up) / 100);

			ev3_tm->ramp.down.end = ev3_tm->time_setpoint;
			ev3_tm->ramp.down.start = ev3_tm->ramp.down.end
			        - ((abs(ev3_tm->speed_setpoint)
			                * ev3_tm->ramp_down) / 100);

			ev3_tm->ramp.setpoint_sign =
			        (ev3_tm->speed_setpoint > 0) ? 1 : -1;

			ev3_tm->ramp.setpoint = ev3_tm->speed_setpoint;

			/* Now figure out if ramp.up.end is past ramp.down.start
			 * and adjust if needed using the intersection of the
			 * ramp up line and ramp down line.
			 *
			 * Basic high-school algebra and knowing ramp.up.end must
			 * equal ramp.down.start gives us:
			 */

			if (ev3_tm->ramp.up.end > ev3_tm->ramp.down.start) {
				ev3_tm->ramp.up.end =
				        ((ev3_tm->time_setpoint
				                * ev3_tm->ramp_up)
				                / (ev3_tm->ramp_up
				                        + ev3_tm->ramp_down));
				ev3_tm->ramp.down.start = ev3_tm->ramp.up.end;
				ev3_tm->ramp.setpoint =
				        ev3_tm->ramp.setpoint_sign
				                * ((ev3_tm->ramp.up.end * 100)
				                        / ev3_tm->ramp_up);
			}

#warning "Also add checks and state machine change for 0 ramp up/down times!"

			ev3_tm->state = STATE_SETUP_RAMP_REGULATION;
			reprocess = true;
			break;

		case STATE_SETUP_RAMP_POSITION:

			/* The position setups are a bit "interesting". We'll want
			 * use the same time based ramping mechanism, but we also
			 * need to take into account position.
			 *
			 * Since the ramp is a linear increase in velocity up to
			 * a setpoint, the position is the "area under the curve"
			 * which happens to be a triangle. The distance covered in
			 * the inital ramp up is 1/2(V*T) where V is measured in
			 * ticks per second and T is measured in milliseconds)
			 *
			 * It's easiest if we simply allow the speed to ramp up
			 * normally up to the speed setpoint and continuously 
			 * estimate the ramp down start and end points based on
			 * the current speed. We have a nice attribute called
			 * pulses_per_second and that value is calculated every time
			 * the speed is actually updated, about 500 times a
			 * second.
			 *
			 * Given the current speed setpoint and the ramp_down
			 * attribute, and assuming a linear ramp down from the
			 * speed setpoint, we can estimate the time it will take
			 * to ramp down as:
			 *
			 * ramp_time = ((speed_setpoint * ramp_down) / 100) msec
			 *
			 * The actual speed in pulses_per_sec can then be used
			 * to estimate how far the motor will travel in that 
			 * time as:
			 *
			 * ramp_distance = (( pulses_per_sec * ramp_time ) / (2 * 1000)) pulses
			 *
			 * Now it's a simple matter to figure out if we're within
			 * distance pulses of the desired endpoint, and then
			 * we can fill in the ramp_down values. The trick is that we
			 * must constantly update the estimate of the ramp_down
			 * start and endpoints, so it's best to do that before the
			 * state handlers!
			 */

			if (POSITION_ABSOLUTE == ev3_tm->position_mode)
				ev3_tm->ramp.position_setpoint =
				        ev3_tm->position_setpoint;
			else
				ev3_tm->ramp.position_setpoint =
				        ev3_tm->ramp.position_setpoint
				                + ev3_tm->position_setpoint;

#warning "These get recalculated in SETUP_RAMP_REGULATION - but it's OK"

			ev3_tm->ramp.setpoint_sign =
			        ((ev3_tm->ramp.position_setpoint
			                >= (ev3_tm->tacho + ev3_tm->irq_tacho)) ?
			                1 : -1);
			ev3_tm->ramp.setpoint = ev3_tm->ramp.setpoint_sign
			        * abs(ev3_tm->speed_setpoint);

			ev3_tm->ramp.up.start = 0;
			ev3_tm->ramp.up.end = ev3_tm->ramp.up.start
			        + ((abs(ev3_tm->speed_setpoint)
			                * ev3_tm->ramp_up) / 100);

			/* Set ramp_down start and end to a ridiculously large number - an hour of milliseconds */

			ev3_tm->ramp.down.end = 3600000;
			ev3_tm->ramp.down.start = 3600000;

			ev3_tm->state = STATE_SETUP_RAMP_REGULATION;
			reprocess = true;
			break;

		case STATE_SETUP_RAMP_REGULATION:
			if (REGULATION_OFF == ev3_tm->regulation_mode) {
				ev3_tm->ramp.offset = 0;
				ev3_tm->ramp.setpoint_sign =
				        (ev3_tm->ramp.setpoint > 0) ? 1 : -1;
				ev3_tm->ramp.setpoint =
				        ev3_tm->ramp.setpoint_sign
				                * abs(ev3_tm->speed_setpoint);
			} else {
#warning "Set regulation offset???"
//				ev3_tm->ramp.offset        = 8;
				ev3_tm->ramp.offset = 0;
				ev3_tm->ramp.setpoint_sign =
				        (ev3_tm->ramp.setpoint > 0) ? 1 : -1;
				ev3_tm->ramp.setpoint =
				        ev3_tm->ramp.setpoint_sign
				                * (abs(ev3_tm->speed_setpoint)
				                        - ev3_tm->ramp.offset);
			}

			ev3_tm->ramp.count = 0;

			ev3_tm->state = STATE_RAMP_UP;
			reprocess = true;
			break;

			/* The LIMITED_XXX functions have to handle the three phases (any of
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
			/* Figure out if we're done ramping up - if yes set state to RAMP_CONST
			 * and allow states to get reprocessed
			 */
			if (ev3_tm->run_mode == RUN_POSITION) {
				adjust_ramp_for_position(ev3_tm);
			}

			if (ev3_tm->ramp.up.end <= ev3_tm->ramp.count) {
				ev3_tm->state = STATE_RAMP_CONST;
				reprocess = true;
//				printk( "Switching to STATE_RAMP_CONST @ %d\n", ev3_tm->ramp.count );
			} else {

				/* Figure out how far along we are in the ramp operation */

				setpoint = ev3_tm->ramp.setpoint_sign
				        * ((ev3_tm->ramp.count * 100)
				                / ev3_tm->ramp_up);

				update_motor_speed_or_power(ev3_tm, setpoint);
			}
			break;

		case STATE_RAMP_CONST:
			/* Figure out if we're done with the const section - if yes set state to RAMP_DOWN
			 * and allow states to get reprocessed
			 */
			if (ev3_tm->run_mode == RUN_POSITION) {
				adjust_ramp_for_position(ev3_tm);
			}

			if (ev3_tm->ramp.down.start <= ev3_tm->ramp.count) {
				if (RUN_TIME == ev3_tm->run_mode) {
					ev3_tm->state = STATE_RAMP_DOWN;
//				printk( "Switching to STATE_RAMP_DOWN @ time %d\n", ev3_tm->ramp.count );
				} else if (RUN_POSITION == ev3_tm->run_mode) {
					ev3_tm->state =
					        STATE_POSITION_RAMP_DOWN;
//				printk( "Switching to STATE_POSITION_RAMP_DOWN @ time/tacho %d/%d\n", ev3_tm->ramp.count, ev3_tm->irq_tacho );
				}

				reprocess = true;
			} else {
				update_motor_speed_or_power(ev3_tm,
				        ev3_tm->ramp.setpoint);
			}
			break;

		case STATE_POSITION_RAMP_DOWN:
			ramp_time = ((abs(ev3_tm->power) * ev3_tm->ramp_down)
			        / 100);

			if (ev3_tm->ramp.setpoint_sign > 0) {

				if (ev3_tm->ramp.position_setpoint
				        <= (ev3_tm->tacho + ev3_tm->irq_tacho
				                + (ev3_tm->power / 4))) {
					ev3_tm->ramp.down.end =
					        ev3_tm->ramp.count;
				} else if (ev3_tm->ramp.down.end
				        <= ev3_tm->ramp.count) {
#warning "Increase ramp endpoint to nudge the ramp setpoint higher"
					ev3_tm->ramp.down.end =
					        ev3_tm->ramp.count + 100;
				}

			} else {

				if (ev3_tm->ramp.position_setpoint
				        >= (ev3_tm->tacho + ev3_tm->irq_tacho
				                + (ev3_tm->power / 4))) {
					ev3_tm->ramp.down.end =
					        ev3_tm->ramp.count;
				} else if (ev3_tm->ramp.down.end
				        <= ev3_tm->ramp.count) {
#warning "Increase ramp endpoint to nudge the ramp setpoint higher"
					ev3_tm->ramp.down.end =
					        ev3_tm->ramp.count + 100;
				}
			}

			/* NOTE: Intentional fallthrough to the STATE_RAMP_DOWN case
			 *
			 * The STATE_POSTION_RAMP_DOWN is busy recalculating the
			 * end point based on the current motor speed, so we can use
			 * the code in STATE_RAMP_DOWN to stop for us!
			 */

		case STATE_RAMP_DOWN:
			/* Figure out if we're done ramping down - if yes then 
			 * decide whether to brake, coast, or leave the motor
			 * unchanged, and allow states to get reprocessed
			 */

			if (ev3_tm->ramp.down.end <= ev3_tm->ramp.count) {
				ev3_tm->state = STATE_STOP;
				reprocess = true;
//				printk( "Switching to STOP @ time %d\n", ev3_tm->ramp.count );
			} else {
				/* Figure out how far along we are in the ramp operation */

				setpoint = ev3_tm->ramp.setpoint_sign
				        * ((ev3_tm->ramp.down.end
				                - ev3_tm->ramp.count) * 100)
				        / ev3_tm->ramp_down;

				update_motor_speed_or_power(ev3_tm, setpoint);
			}
			break;

		case STATE_STOP:
		{
			/* Add in the irq_tacho for the current move so that we can use
			 * the value of irq_tacho in the HOLD mode - the current, real
			 * tacho reading is ALWAYS tacho + irq_tacho!
			 */
			while (ev3_tm->irq_mutex) {
				printk("Waiting for IRQ update!\n");
			};

//			printk( "STOP: ramp.setpoint %d tacho %d irq_tacho %d\n", ev3_tm->ramp.position_setpoint, ev3_tm->tacho, ev3_tm->irq_tacho);
			if (ev3_tm->run_mode == RUN_POSITION) {
				ev3_tm->irq_tacho = (ev3_tm->tacho
				        + ev3_tm->irq_tacho)
				        - ev3_tm->ramp.position_setpoint;
				ev3_tm->tacho = ev3_tm->ramp.position_setpoint;
			} else {
				ev3_tm->tacho = ev3_tm->tacho
				        + ev3_tm->irq_tacho;
				ev3_tm->irq_tacho = 0;
			}
//			printk( "ADJU: ramp.setpoint %d tacho %d irq_tacho %d\n", ev3_tm->ramp.position_setpoint, ev3_tm->tacho, ev3_tm->irq_tacho);
//			}

			ev3_tm->speed_reg_setpoint = 0;
			ev3_tacho_motor_set_power(ev3_tm, 0);

			reprocess = true;
			ev3_tm->state = STATE_IDLE;
		}
			break;

		case STATE_IDLE:
		{
//			if (abs(ev3_tm->speed) <= 3) {
			ev3_tm->run = 0;
		}
			break;
		default:
		{ /* Intentionally left empty */
//				printk( "UNHANDLED MOTOR STATE %d\n", ev3_tm->state );
		}
			break;
		}
	}

	if (ev3_tm->run && (REGULATION_ON == ev3_tm->regulation_mode))
		regulate_speed(ev3_tm);

	no_run:
	/* Note, we get here even if we're running - so we need to check
	 * explicitly. These are some special cases to handle changes in the
	 * hold_mode and brake_mode when the motor is not running!
	 */

	if (!ev3_tm->run) {
		if (HOLD_ON == ev3_tm->hold_mode)
			regulate_position(ev3_tm);

		else if (BRAKE_ON == ev3_tm->brake_mode)
			ev3_tacho_motor_brake(ev3_tm);

		else if (BRAKE_OFF == ev3_tm->brake_mode)
			ev3_tacho_motor_coast(ev3_tm);
	}

	return HRTIMER_RESTART;
}

/* -------------------------------------------------------------------------- */

static int ev3_tacho_motor_get_type(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	if (ev3_tm->motor_type == MOTOR_TYPE_MINITACHO)
		return TACHO_TYPE_MINITACHO;
	else if (ev3_tm->motor_type == MOTOR_TYPE_TACHO)
		return TACHO_TYPE_TACHO;
	else
		return TACHO_TYPE_TACHO;
}

static void ev3_tacho_motor_set_type(struct tacho_motor_device *tm, long type)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	if (type == TACHO_TYPE_TACHO)
		ev3_tm->motor_type = MOTOR_TACHO;
	else if (type == TACHO_TYPE_MINITACHO)
		ev3_tm->motor_type = MOTOR_MINITACHO;
}

static int ev3_tacho_motor_get_position(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->tacho + ev3_tm->irq_tacho;
}

static void ev3_tacho_motor_set_position(struct tacho_motor_device *tm,
                                         long position)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	ev3_tm->irq_tacho = 0;
	ev3_tm->tacho = position;
	ev3_tm->ramp.position_setpoint = position;
}

static int ev3_tacho_motor_get_speed(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->speed;
}

static int ev3_tacho_motor_get_power(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->power;
}

static int ev3_tacho_motor_get_state(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->state;
}

static int ev3_tacho_motor_get_pulses_per_second(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->pulses_per_second;
}

static int ev3_tacho_motor_get_speed_setpoint(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->speed_setpoint;
}

static void ev3_tacho_motor_set_speed_setpoint(struct tacho_motor_device *tm,
                                               long speed_setpoint)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	ev3_tm->speed_setpoint = speed_setpoint;
}

static int ev3_tacho_motor_get_time_setpoint(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->time_setpoint;
}

static void ev3_tacho_motor_set_time_setpoint(struct tacho_motor_device *tm,
                                              long time_setpoint)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	ev3_tm->time_setpoint = time_setpoint;
}

static int ev3_tacho_motor_get_position_setpoint(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->position_setpoint;
}

static void ev3_tacho_motor_set_position_setpoint(struct tacho_motor_device *tm,
                                                  long position_setpoint)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	ev3_tm->position_setpoint = position_setpoint;
}

static int ev3_tacho_motor_get_regulation_mode(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->regulation_mode;
}

static void ev3_tacho_motor_set_regulation_mode(struct tacho_motor_device *tm,
                                                long regulation_mode)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	ev3_tm->regulation_mode = regulation_mode;
}

static int ev3_tacho_motor_get_position_mode(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->position_mode;
}

static void ev3_tacho_motor_set_position_mode(struct tacho_motor_device *tm,
                                              long position_mode)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	ev3_tm->position_mode = position_mode;
}

static int ev3_tacho_motor_get_brake_mode(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->brake_mode;
}

static void ev3_tacho_motor_set_brake_mode(struct tacho_motor_device *tm,
                                           long brake_mode)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	ev3_tm->brake_mode = brake_mode;
}

static int ev3_tacho_motor_get_hold_mode(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->hold_mode;
}

static void ev3_tacho_motor_set_hold_mode(struct tacho_motor_device *tm,
                                          long hold_mode)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	ev3_tm->hold_mode = hold_mode;
}

static int ev3_tacho_motor_get_polarity_mode(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->polarity_mode;
}

static void ev3_tacho_motor_set_polarity_mode(struct tacho_motor_device *tm,
                                              long polarity_mode)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	ev3_tm->polarity_mode = polarity_mode;
}

static int ev3_tacho_motor_get_ramp_up(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->ramp_up;
}

static void ev3_tacho_motor_set_ramp_up(struct tacho_motor_device *tm,
                                        long ramp_up)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	ev3_tm->ramp_up = ramp_up;
}

static int ev3_tacho_motor_get_ramp_down(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->ramp_down;
}

static void ev3_tacho_motor_set_ramp_down(struct tacho_motor_device *tm,
                                          long ramp_down)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	ev3_tm->ramp_down = ramp_down;
}

/* -------------------------------------------------------------------------- */

static int ev3_tacho_motor_get_run_mode(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->run_mode;
}

static void ev3_tacho_motor_set_run_mode(struct tacho_motor_device *tm,
                                         long run_mode)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	ev3_tm->run_mode = run_mode;
}

static int ev3_tacho_motor_get_run(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	return ev3_tm->run;
}

static void ev3_tacho_motor_set_run(struct tacho_motor_device *tm, long run)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	if (0 == run)
		ev3_tm->state = STATE_STOP;

	else if ((RUN_FOREVER == ev3_tm->run_mode))
		ev3_tm->state = STATE_RUN_FOREVER;

	else if ((RUN_TIME == ev3_tm->run_mode))
		ev3_tm->state = STATE_SETUP_RAMP_TIME;

	else if ((RUN_POSITION == ev3_tm->run_mode))
		ev3_tm->state = STATE_SETUP_RAMP_POSITION;

	else
		ev3_tm->state = STATE_IDLE;

	/* what's going on here - why is run always set to 1? 
	 *
	 * the answer is that we check for run == 0 as the first condition
	 * at the top of this function. if it's set, then the next motor state
	 * is stop_motor, but it won't be evaluated if run == 0
	 *
	 * so we always force the state machine to run once, and count on the
	 * state machine to dtrt (do the right thing). This avoids setting motor
	 * power in wierd places
	 */

	ev3_tm->run = 1;
}

static void ev3_tacho_motor_set_reset(struct tacho_motor_device *tm, long reset)
{
	struct ev3_tacho_motor_data *ev3_tm = container_of(tm,
	        struct ev3_tacho_motor_data, tm);

	ev3_tacho_motor_reset(ev3_tm);
}

static const struct function_pointers fp = { .get_type =
        ev3_tacho_motor_get_type, .set_type = ev3_tacho_motor_set_type,

.get_position = ev3_tacho_motor_get_position, .set_position =
        ev3_tacho_motor_set_position,

.get_speed = ev3_tacho_motor_get_speed, .get_power = ev3_tacho_motor_get_power,
        .get_state = ev3_tacho_motor_get_state, .get_pulses_per_second =
                ev3_tacho_motor_get_pulses_per_second,

        .get_speed_setpoint = ev3_tacho_motor_get_speed_setpoint,
        .set_speed_setpoint = ev3_tacho_motor_set_speed_setpoint,

        .get_time_setpoint = ev3_tacho_motor_get_time_setpoint,
        .set_time_setpoint = ev3_tacho_motor_set_time_setpoint,

        .get_position_setpoint = ev3_tacho_motor_get_position_setpoint,
        .set_position_setpoint = ev3_tacho_motor_set_position_setpoint,

        .get_run_mode = ev3_tacho_motor_get_run_mode, .set_run_mode =
                ev3_tacho_motor_set_run_mode,

        .get_regulation_mode = ev3_tacho_motor_get_regulation_mode,
        .set_regulation_mode = ev3_tacho_motor_set_regulation_mode,

        .get_brake_mode = ev3_tacho_motor_get_brake_mode, .set_brake_mode =
                ev3_tacho_motor_set_brake_mode,

        .get_hold_mode = ev3_tacho_motor_get_hold_mode, .set_hold_mode =
                ev3_tacho_motor_set_hold_mode,

        .get_position_mode = ev3_tacho_motor_get_position_mode,
        .set_position_mode = ev3_tacho_motor_set_position_mode,

        .get_polarity_mode = ev3_tacho_motor_get_polarity_mode,
        .set_polarity_mode = ev3_tacho_motor_set_polarity_mode,

        .get_ramp_up = ev3_tacho_motor_get_ramp_up, .set_ramp_up =
                ev3_tacho_motor_set_ramp_up,

        .get_ramp_down = ev3_tacho_motor_get_ramp_down, .set_ramp_down =
                ev3_tacho_motor_set_ramp_down,

        .get_run = ev3_tacho_motor_get_run, .set_run = ev3_tacho_motor_set_run,

        .set_reset = ev3_tacho_motor_set_reset, };

static int ev3_tacho_motor_probe(struct legoev3_port_device *motor)
{
	struct ev3_tacho_motor_data *ev3_tm;
	struct ev3_motor_platform_data *pdata = motor->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;

	ev3_tm = kzalloc(sizeof(struct ev3_tacho_motor_data), GFP_KERNEL);

	if (!ev3_tm)
		return -ENOMEM;

	ev3_tm->out_port = pdata->out_port;
	ev3_tm->motor_port = motor;

	ev3_tm->tm.fp = &fp;

	ev3_tacho_motor_reset(ev3_tm);

	err = register_tacho_motor(&ev3_tm->tm, &motor->dev);
	if (err)
		goto register_tacho_motor_fail;

	err = dev_set_drvdata(&motor->dev, ev3_tm);
	if (err)
		goto dev_set_drvdata_fail;

	dev_info(&motor->dev,
	        "Tacho Motor connected to port %s gpio %d irq %d\n",
	        dev_name(&ev3_tm->out_port->dev), pdata->tacho_int_gpio,
	        gpio_to_irq(pdata->tacho_int_gpio));

	// Here's where we set up the port pins on a per-port basis

	if (request_irq(gpio_to_irq(pdata->tacho_int_gpio), tacho_motor_isr, 0,
	        dev_name(&ev3_tm->out_port->dev), ev3_tm))
		goto dev_request_irq_fail;

	irq_set_irq_type(gpio_to_irq(pdata->tacho_int_gpio),
	IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);

	/* Set up the output port status processing timer */

	hrtimer_init(&ev3_tm->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ev3_tm->timer.function = ev3_tacho_motor_timer_callback;
	hrtimer_start(&ev3_tm->timer, ktime_set(0, TACHO_MOTOR_POLL_NS),
	        HRTIMER_MODE_REL);

	return 0;

	dev_request_irq_fail: dev_set_drvdata(&motor->dev, NULL);

	dev_set_drvdata_fail: unregister_tacho_motor(&ev3_tm->tm);

	register_tacho_motor_fail: kfree(ev3_tm);

	return err;
}

static int ev3_tacho_motor_remove(struct legoev3_port_device *motor)
{
	struct ev3_motor_platform_data *pdata = motor->dev.platform_data;
	struct ev3_tacho_motor_data *ev3_tm = dev_get_drvdata(&motor->dev);

	hrtimer_cancel(&ev3_tm->timer);

	dev_info(&motor->dev,
	        "Unregistering interrupt from gpio %d irq %d on port %s\n",
	        pdata->tacho_int_gpio, gpio_to_irq(pdata->tacho_int_gpio),
	        dev_name(&ev3_tm->out_port->dev));

	free_irq(gpio_to_irq(pdata->tacho_int_gpio), ev3_tm);

	dev_info(&motor->dev, "Tacho motor removed from port %s\n",
	        dev_name(&ev3_tm->out_port->dev));
	dev_set_drvdata(&motor->dev, NULL);
	unregister_tacho_motor(&ev3_tm->tm);
	kfree(ev3_tm);
	return 0;
}

struct legoev3_port_driver ev3_tacho_motor_driver = {
	.probe = ev3_tacho_motor_probe,
	.remove = ev3_tacho_motor_remove,
	.driver = {
		.name = "ev3-tacho-motor",
		.owner = THIS_MODULE,
	},
};
EXPORT_SYMBOL_GPL(ev3_tacho_motor_driver);
legoev3_port_driver(ev3_tacho_motor_driver);

MODULE_DESCRIPTION("EV3 tacho motor driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("Ralph Hempel <rhempel@hempeldesigngroup.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ev3-tacho-motor");

