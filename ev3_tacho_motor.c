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
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_output_port.h>
#include <linux/legoev3/tacho_motor_class.h>

#include <asm/bug.h>

// #define PIN6_NEAR_GND		250		/* 0.25V */

#define   TACHO_SAMPLES           128

#define   MAX_PWM_CNT                   (10000)
#define   MAX_SPEED                     (100)
#define   SPEED_PWMCNT_REL              (100) //(MAX_PWM_CNT/MAX_SPEED)
#define   RAMP_FACTOR                   (1000)
#define   MAX_SYNC_MOTORS               (2)

#define   COUNTS_PER_PULSE_LM           12800L
#define   COUNTS_PER_PULSE_MM           8100L

enum
{
  SAMPLES_BELOW_SPEED_25  =  0,
  SAMPLES_SPEED_25_50     =  1,
  SAMPLES_SPEED_50_75     =  2,
  SAMPLES_ABOVE_SPEED_75  =  3,
  NO_OF_SAMPLE_STEPS      =  4
};


/*! \brief    Number of average samples for each motor
 *
 *  - Medium motor = 1, 2,  4,  8
 *  - Large motor  = 2, 8, 16, 32
 *
 *  Medium motor has not the jitter on the tacho when as large motor because
 *  it has one gear wheel less that the large motor.
 *
 *  Medium motor reaction time is much faster than large motor due to smaller motor
 *  and smaller gearbox
 */
static    unsigned    SamplesMediumMotor[NO_OF_SAMPLE_STEPS] = {2, 4,  8,  16};
static    unsigned    SamplesLargeMotor[NO_OF_SAMPLE_STEPS]  = {4, 16, 32, 64};

struct ev3_tacho_motor_data {
	struct tacho_motor_device tm;

	struct legoev3_port_device *out_port;
	struct legoev3_port_device *motor_port;


        unsigned tacho_samples[TACHO_SAMPLES];
        unsigned tacho_samples_head;
        unsigned tacho_samples_tail;

        unsigned long *TIMER64P3;

	unsigned *samples_per_speed;
	unsigned counts_per_pulse;
	unsigned dir_chg_samples;

        int tacho;
        int speed;
        int direction;
};

// static    unsigned    AVG_TACHO_COUNTS[NO_OF_OUTPUT_PORTS]   = {2,2,2,2};
// static    unsigned    AVG_COUNTS[NO_OF_OUTPUT_PORTS]         = {(2 * COUNTS_PER_PULSE_LM),(2 * COUNTS_PER_PULSE_LM),(2 * COUNTS_PER_PULSE_LM),(2 * COUNTS_PER_PULSE_LM)};

enum
{
  FORWARD,
  BACKWARD,
  BRAKE,
  COAST,
};

/* TIMER64 register configuration */
enum
{
  REVID       = 0,
  EMUMGT      = 1,
  GPINTGPEN   = 2,
  GPDATGPDIR  = 3,
  TIM12       = 4,
  TIM34       = 5,
  PRD12       = 6,
  PRD34       = 7,
  TCR         = 8,
  TGCR        = 9,
  WDTCR       = 10,
  NOTUSED1    = 11,
  NOTUSED2    = 12,
  REL12       = 13,
  REL34       = 14,
  CAP12       = 15,
  CAP34       = 16,
  NOTUSED3    = 17,
  NOTUSED4    = 18,
  NOTUSED5    = 19,
  NOTUSED6    = 20,
  NOTUSED7    = 21,
  NOTUSED8    = 22,
  INTCTLSTAT  = 23,
  CMP0        = 24,
  CMP1        = 25,
  CMP2        = 26,
  CMP3        = 27,
  CMP4        = 28,
  CMP5        = 39,
  CMP6        = 30,
  CMP7        = 31,
};


static irqreturn_t tacho_motor_isr(int irq, void *id)
{
	struct ev3_tacho_motor_data *ev3_tm = id;
	struct ev3_motor_platform_data *pdata = ev3_tm->motor_port->dev.platform_data; 

	bool int_state = gpio_get_value(pdata->tacho_int_gpio);
	bool dir_state = gpio_get_value(pdata->tacho_dir_gpio);
	unsigned long timer = (((unsigned long *)(ev3_tm->TIMER64P3))[TIM34]) >> 8;

	unsigned next_sample;

	int  next_direction;

	/* Grab the next incremental sample timestamp */

	next_sample = (ev3_tm->tacho_samples_head + 1) % TACHO_SAMPLES;

	ev3_tm->tacho_samples[next_sample] = timer;
	ev3_tm->tacho_samples_head = next_sample;

	/* If the speed is high enough, just update the tacho counter based on direction */

	if ((35 < ev3_tm->speed) || (-35 > ev3_tm->speed)) {

		if (ev3_tm->dir_chg_samples < ev3_tm->samples_per_speed[SAMPLES_ABOVE_SPEED_75])
			ev3_tm->dir_chg_samples++;

	} else {

	/* Update the tacho count and motor direction for low speed, taking
	/  advantage of the fact that if state and dir match, then the motor
	/  is turning FORWARD!
	*/
		if (int_state == dir_state)
			next_direction = FORWARD;
		else
			next_direction = BACKWARD;

		/* If the saved and next direction states match, then update the dir_chg_sample count */

		if (ev3_tm->direction == next_direction) {
			if (ev3_tm->dir_chg_samples < ev3_tm->samples_per_speed[SAMPLES_ABOVE_SPEED_75])
				ev3_tm->dir_chg_samples++;
		} else {
			ev3_tm->dir_chg_samples = 0;
		}

		ev3_tm->direction = next_direction;
	}

	if (FORWARD == ev3_tm->direction)
		ev3_tm->tacho++;
	else			
		ev3_tm->tacho--;

//        pr_warning("Got an interrupt on gpio %d on port %s State %d %d Timer %lx tacho %d!\n", irq
//							, dev_name(&ev3_tm->out_port->dev)
//							, int_state
//						        , dir_state
//                                                        , timer
//							, ev3_tm->irq_tacho );
	return IRQ_HANDLED;
}
	
// static bool ev3_touch_sensor_pressed(struct touch_sensor_device *ts)
// {
// 	struct ev3_touch_sensor_data *ev3_ts =
// 			container_of(ts, struct ev3_touch_sensor_data, ts);
// 
// 	return ev3_input_port_get_pin6_mv(ev3_ts->in_port) > PIN6_NEAR_GND;
// }

static unsigned ev3_tacho_motor_get_tacho(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->tacho;
}

static unsigned ev3_tacho_motor_get_direction(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->direction;
}

static unsigned ev3_tacho_motor_get_speed(struct tacho_motor_device *tm)
{
	struct ev3_tacho_motor_data *ev3_tm =
			container_of(tm, struct ev3_tacho_motor_data, tm);

	return ev3_tm->speed;
}

static int __devinit ev3_tacho_motor_probe(struct legoev3_port_device *motor)
{
	struct ev3_tacho_motor_data *ev3_tm;
	struct ev3_motor_platform_data *pdata = motor->dev.platform_data;
	int err;

	if (WARN_ON(!pdata))
		return -EINVAL;

	ev3_tm = kzalloc(sizeof(struct ev3_tacho_motor_data), GFP_KERNEL);

	if (!ev3_tm)
		return -ENOMEM;

	ev3_tm->out_port   = pdata->out_port;
	ev3_tm->motor_port = motor;

        /* Set up motor specific initialization constants */
        /* FIXME: These need to be motor specific later! */

	ev3_tm->tm.get_tacho     = ev3_tacho_motor_get_tacho;
	ev3_tm->tm.get_direction = ev3_tacho_motor_get_direction;
	ev3_tm->tm.get_speed     = ev3_tacho_motor_get_speed;

	ev3_tm->samples_per_speed = SamplesLargeMotor;
	ev3_tm->counts_per_pulse  = COUNTS_PER_PULSE_LM;

        /* FIXME: These need to be motor specific later! */

	err = register_tacho_motor(&ev3_tm->tm, &motor->dev);
	if (err)
		goto register_tacho_motor_fail;

	err = dev_set_drvdata(&motor->dev, ev3_tm);
	if (err)
		goto dev_set_drvdata_fail;

	dev_info(&motor->dev, "Tacho Motor connected to port %s gpio %d irq %d\n",
		dev_name(&ev3_tm->out_port->dev),
                pdata->tacho_int_gpio,
                gpio_to_irq(pdata->tacho_int_gpio));

        // Here's where we set up the port pins on a per-port basis
        
        if(request_irq(gpio_to_irq(pdata->tacho_int_gpio), tacho_motor_isr, 0, dev_name(&ev3_tm->out_port->dev), ev3_tm ))
		goto dev_request_irq_fail;

        irq_set_irq_type(gpio_to_irq(pdata->tacho_int_gpio), IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);

        ev3_tm->TIMER64P3 = legoev3_port_remap_TIMER64P3();

	return 0;

dev_request_irq_fail:
	dev_set_drvdata(&motor->dev, NULL);

dev_set_drvdata_fail:
	unregister_tacho_motor(&ev3_tm->tm);

register_tacho_motor_fail:
	kfree(ev3_tm);

	return err;
}
// int ev3_output_port_request_irq(struct legoev3_port_device *motor, unsigned int type, irq_handler_t handler, void *id)
// {
// 	struct ev3_output_port_data *port =
// 			container_of(motor, struct ev3_output_port_data, motor);
// 
//         pr_warning("GPIO_PIN1       for port %d is: %d\n", port->id, port->gpio[GPIO_PIN1].gpio);
//         pr_warning("GPIO_PIN2       for port %d is: %d\n", port->id, port->gpio[GPIO_PIN2].gpio);
//         pr_warning("GPIO_PIN5       for port %d is: %d\n", port->id, port->gpio[GPIO_PIN5].gpio);
//         pr_warning("GPIO_PIN5_TACHO for port %d is: %d\n", port->id, port->gpio[GPIO_PIN5_TACHO].gpio);
//         pr_warning("GPIO_PIN6       for port %d is: %d\n", port->id, port->gpio[GPIO_PIN6].gpio);
// 
// 
// //        request_irq(port->gpio[GPIO_PIN5_TACHO].gpio, handler, 0, dev_name(&port->pdev->dev), id );
// //        irq_set_irq_type(port->gpio[GPIO_PIN5_TACHO].gpio, type);
// 
//         return 0;
// }
// EXPORT_SYMBOL_GPL(ev3_output_port_request_irq);
// 
// void ev3_output_port_free_irq(struct legoev3_port_device *out_port)
// {
// //	struct ev3_output_port_data *port = dev_get_drvdata(&out_port->dev);
// 
// //        free_irq(port->gpio[GPIO_PIN5_TACHO].gpio, NULL );
// }
// EXPORT_SYMBOL_GPL(ev3_output_port_free_irq);
// 
// 
// 
// int ev3_output_port_get_pin56_levels(struct legoev3_port_device *out_port, unsigned *pin5, unsigned *pin6)
// {
// 	struct ev3_output_port_data *port = dev_get_drvdata(&out_port->dev);
// 
//  	*pin5 = gpio_get_value(port->gpio[GPIO_PIN5_TACHO].gpio);
//  	*pin6 = gpio_get_value(port->gpio[GPIO_PIN6].gpio);
// 
// 	return 0;
// }
// EXPORT_SYMBOL_GPL(ev3_output_port_get_pin56_levels);
// 
// 
static int __devexit ev3_tacho_motor_remove(struct legoev3_port_device *motor)
{
	struct ev3_motor_platform_data *pdata = motor->dev.platform_data;
	struct ev3_tacho_motor_data *ev3_tm = dev_get_drvdata(&motor->dev);

	dev_info(&motor->dev, "Unregistering interrupt from gpio %d irq %d on port %s\n",
		pdata->tacho_int_gpio,
		gpio_to_irq(pdata->tacho_int_gpio),
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
	.probe	= ev3_tacho_motor_probe,
	.remove	= __devexit_p(ev3_tacho_motor_remove),
	.driver = {
		.name	= "ev3-tacho-motor",
		.owner	= THIS_MODULE,
	},
};
EXPORT_SYMBOL_GPL(ev3_tacho_motor_driver);
legoev3_port_driver(ev3_tacho_motor_driver);

MODULE_DESCRIPTION("EV3 tacho motor driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("Ralph Hempel <rhempel@hempeldesigngroup.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ev3-tacho-motor");

