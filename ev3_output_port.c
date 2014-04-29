/*
 * Output port driver for LEGO Mindstorms EV3
 *
 * Copyright (C) 2013-2014 Ralph Hempel <rhempel@hempeldesigngroup.com>
 *
 * Based on input port driver code that is:
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

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/pwm.h>
#include <linux/legoev3/legoev3_analog.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_output_port.h>

#include <mach/mux.h>
 
#define OUTPUT_PORT_POLL_NS	( 10000000)	                /* 10 msec */
#define SETTLE_CNT		( 20000000/OUTPUT_PORT_POLL_NS)	/* 20 msec */
#define ADD_CNT			(350000000/OUTPUT_PORT_POLL_NS) /* 350 msec */
#define REMOVE_CNT		(100000000/OUTPUT_PORT_POLL_NS) /* 100 msec */

#define   ADC_REF                       5000  //  [mV]  Maximum voltage that the A/D can read

#define   PIN5_IIC_HIGH                 3700  //  [mV]  values in between these limits means that
#define   PIN5_IIC_LOW                  2800  //  [mV]    an old IIC sensor or color sensor is connected

#define   PIN5_MINITACHO_HIGH1          2000  //  [mV]  values in between these limits means that
#define   PIN5_MINITACHO_LOW1           1600  //  [mV]    a mini tacho motor is pulling high when pin5 is pulling low

#define   PIN5_BALANCE_HIGH             2600  //  [mV]  values in between these limits means that
#define   PIN5_BALANCE_LOW              2400  //  [mV]    connection 5 is floating

#define   PIN5_LIGHT_HIGH                850  //  [mV]  values in between these limits means that
#define   PIN5_LIGHT_LOW                 650  //  [mV]    an old light sensor is connected

#define   PIN5_MINITACHO_HIGH2           450  //  [mV]  values in between these limits means that
#define   PIN5_MINITACHO_LOW2            250  //  [mV]    a mini tacho motor is pulling low when pin5 floats

#define   PIN5_NEAR_GND                  100  //  [mV]  lower  values mean that connection 5 is shorted to ground

enum gpio_index {
	GPIO_PIN1,
	GPIO_PIN2,
	GPIO_PIN5,
	GPIO_PIN5_INT,
	GPIO_PIN6_DIR,
	NUM_GPIO
};

enum connection_state {
	CON_STATE_INIT,				/* Wait for motor to unregister, then
							Set port to "float" state */
	CON_STATE_INIT_SETTLE,			/* Wait for port to settle */
	CON_STATE_NO_DEV,			/* No device present, wait until something
							interesting happens on one or more
							of the pins and a steady state is
 							reached */
 	CON_STATE_PIN6_SETTLE,			/* Pin6 to settle after changing state  */
 	CON_STATE_CONNECTED,			/* We are ready to figure out what's connected */
 	CON_STATE_PIN5_SETTLE,			/* Pin5 to settle after changing state  */
        CON_STATE_DEVICE_CONNECTED,		/* We detected the connection of a device */
        CON_STATE_WAITING_FOR_DISCONNECT,	/* We are waiting for disconnect */
	NUM_CON_STATE
};

enum pin_state_flag {
	PIN_STATE_FLAG_PIN2_LOW,
	PIN_STATE_FLAG_PIN5_LOADED,
	PIN_STATE_FLAG_PIN5_LOW,
	PIN_STATE_FLAG_PIN6_HIGH,
	NUM_PIN_STATE_FLAG
};

static struct attribute *legoev3_output_port_device_type_attrs[] = {
	NULL
};

struct attribute_group legoev3_output_port_device_type_attr_grp = {
	.attrs	= legoev3_output_port_device_type_attrs,
};

const struct attribute_group *ev3_motor_device_type_attr_groups[] = {
	&legoev3_port_device_type_attr_grp,
	&legoev3_output_port_device_type_attr_grp,
	NULL
};

struct device_type ev3_motor_device_types[] = {
	[MOTOR_NEWTACHO] = {
		.name	= "ev3-tacho-motor",
		.groups	= ev3_motor_device_type_attr_groups,
	},
	[MOTOR_MINITACHO] = {
		.name	= "ev3-tacho-motor",
		.groups	= ev3_motor_device_type_attr_groups,
	},
	[MOTOR_TACHO] = {
		.name	= "ev3-tacho-motor",
		.groups	= ev3_motor_device_type_attr_groups,
	},
};

/**
 * struct ev3_output_port_data - Driver data for an output port on the EV3 brick
 * @id: Unique identifier for the port.
 * @pdev: Pointer to the legoev3_port_device that is bound to this instance.
 * @analog: pointer to the legoev3-analog device for accessing data from the
 *	analog/digital converter.
 * @pwm: Pointer to the pwm device that is bound to this port
 * @gpio: Array of gpio pins used by this input port.
 * @work: Worker for registering and unregistering sensors when they are
 *	connected and disconnected.
 * @timer: Polling timer to monitor the port connect/disconnect.
 * @timer_loop_cnt: Used to measure time in the polling loop.
 * @con_state: The current state of the port.
 * @pin_state_flags: Used in the polling loop to track certain changes in the
 *	state of the port's pins.
 * @pin5_float_mv: Used in the polling loop to track pin 5 voltage.
 * @pin5_low_mv: Used in the polling loop to track pin 5 voltage.
 * @motor_type: The type of motor currently connected.
 * @motor: Pointer to the motor device that is connected to the output port.
 */
struct ev3_output_port_data {
	enum ev3_output_port_id id;
	struct legoev3_port_device *pdev;
	struct legoev3_analog_device *analog;
	struct pwm_device *pwm;
 	struct gpio gpio[NUM_GPIO];
 	struct work_struct work;
 	struct hrtimer timer;
 	unsigned timer_loop_cnt;
 	enum connection_state con_state;
 	unsigned pin_state_flags:NUM_PIN_STATE_FLAG;
 	unsigned pin5_float_mv;
 	unsigned pin5_low_mv;
 	enum motor_type motor_type;
 	struct legoev3_port_device *motor;
};

void ev3_output_port_float(struct ev3_output_port_data *port)
{
	gpio_direction_output(port->gpio[GPIO_PIN1    ].gpio, 0);
	gpio_direction_output(port->gpio[GPIO_PIN2    ].gpio, 0);
	gpio_direction_input( port->gpio[GPIO_PIN5    ].gpio   );
	gpio_direction_input( port->gpio[GPIO_PIN5_INT].gpio   );
	gpio_direction_input( port->gpio[GPIO_PIN6_DIR].gpio   );
}

void ev3_output_port_register_motor(struct work_struct *work)
{
	struct ev3_output_port_data *port =
			container_of(work, struct ev3_output_port_data, work);
	struct legoev3_port_device *motor;
	struct ev3_motor_platform_data pdata;

	if (port->motor_type == MOTOR_NONE
	    || port->motor_type == MOTOR_ERR
	    || port->motor_type >= NUM_MOTOR)
	{
		dev_err(&port->pdev->dev, "Trying to register an invalid motor on %s.\n",
			dev_name(&port->pdev->dev));
		return;
	}

        /* Fill in the motor platform data struct */

 	pdata.out_port  = port->pdev;

 	pdata.motor_dir0_gpio = port->gpio[GPIO_PIN1].gpio;
 	pdata.motor_dir1_gpio = port->gpio[GPIO_PIN2].gpio;
 	pdata.tacho_int_gpio  = port->gpio[GPIO_PIN5_INT].gpio;
 	pdata.tacho_dir_gpio  = port->gpio[GPIO_PIN6_DIR].gpio;
 	pdata.pwm             = port->pwm;
	pdata.motor_type      = port->motor_type;

 	motor = legoev3_port_device_register("motor", -1,
 				&ev3_motor_device_types[port->motor_type], -1,
 				&pdata, sizeof(struct ev3_motor_platform_data),
 				&port->pdev->dev);
 	if (IS_ERR(motor)) {
 		dev_err(&port->pdev->dev, "Could not register motor on port %s.\n",
 			dev_name(&port->pdev->dev));
		return;
 	}
 
	port->motor = motor;

	return;
}

void ev3_output_port_unregister_motor(struct work_struct *work)
{
	struct ev3_output_port_data *port =
			container_of(work, struct ev3_output_port_data, work);

 	legoev3_port_device_unregister(port->motor);

	port->motor = NULL;
}

static enum hrtimer_restart ev3_output_port_timer_callback(struct hrtimer *timer)
{
 	struct ev3_output_port_data *port =
 			container_of(timer, struct ev3_output_port_data, timer);
 	unsigned new_pin_state_flags = 0;
 	unsigned new_pin5_mv = 0;
 
 	hrtimer_forward_now(timer, ktime_set(0, OUTPUT_PORT_POLL_NS));
 	port->timer_loop_cnt++;

	switch(port->con_state) {
	case CON_STATE_INIT:
		if (!port->motor) {
			ev3_output_port_float(port);
			port->timer_loop_cnt = 0;
			port->motor_type = MOTOR_NONE;
			port->con_state = CON_STATE_INIT_SETTLE;
		}
		break;
	case CON_STATE_INIT_SETTLE:
		if (port->timer_loop_cnt >= SETTLE_CNT) {
			port->timer_loop_cnt = 0;
			port->con_state = CON_STATE_NO_DEV;
		}
		break;
	case CON_STATE_NO_DEV:
		new_pin5_mv = legoev3_analog_out_pin5_value(port->analog, port->id);

		if (gpio_get_value(port->gpio[GPIO_PIN6_DIR].gpio))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN6_HIGH);
		if ((new_pin5_mv < PIN5_BALANCE_LOW) || (new_pin5_mv > PIN5_BALANCE_HIGH))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN5_LOADED);

 		if (new_pin_state_flags != port->pin_state_flags) {
 			port->pin_state_flags = new_pin_state_flags;
 			port->timer_loop_cnt = 0;
 		}

		if (port->pin_state_flags && (port->timer_loop_cnt >= ADD_CNT)) {
			port->pin5_float_mv = new_pin5_mv;
 			port->timer_loop_cnt = 0;
			gpio_direction_output(port->gpio[GPIO_PIN6_DIR].gpio, 0);
			port->con_state = CON_STATE_PIN6_SETTLE;
		}
		break;

	case CON_STATE_PIN6_SETTLE:
		new_pin5_mv = legoev3_analog_out_pin5_value(port->analog, port->id);

		if (port->timer_loop_cnt >= SETTLE_CNT) {
			port->pin5_low_mv = new_pin5_mv;
 			port->timer_loop_cnt = 0;
			gpio_direction_input(port->gpio[GPIO_PIN6_DIR].gpio);
			port->con_state = CON_STATE_CONNECTED;
			}
		break;

 	case CON_STATE_CONNECTED:

                // Make a temporary variable that we can use to determine the relative
                // difference between pin5_float_mv and pin5_low_mv

		new_pin5_mv = ADC_REF + port->pin5_float_mv - port->pin5_low_mv;

                if ((new_pin5_mv > (ADC_REF - 50)) && (new_pin5_mv < (ADC_REF + 50))) {
			// The pin5 values are the same, let's see what we have!

			if ((port->pin5_float_mv >= PIN5_BALANCE_LOW) && (port->pin5_float_mv <= PIN5_BALANCE_HIGH) && (port->pin_state_flags & (0x01 << PIN_STATE_FLAG_PIN6_HIGH))) {
				// NXT TOUCH SENSOR, NXT SOUND SENSOR or NEW UART SENSOR
				port->motor_type = MOTOR_ERR;
				port->con_state = CON_STATE_WAITING_FOR_DISCONNECT;

			} else if (port->pin5_float_mv < PIN5_NEAR_GND) {
				// NEW DUMB SENSOR
				port->motor_type = MOTOR_ERR;
				port->con_state = CON_STATE_WAITING_FOR_DISCONNECT;

			} else if ((port->pin5_float_mv >= PIN5_LIGHT_LOW) && (port->pin5_float_mv  <= PIN5_LIGHT_HIGH)) {
				// NXT LIGHT SENSOR
				port->motor_type = MOTOR_ERR;
				port->con_state = CON_STATE_WAITING_FOR_DISCONNECT;

			} else if ((port->pin5_float_mv >= PIN5_IIC_LOW) && (port->pin5_float_mv <= PIN5_IIC_HIGH)) {
				// NXT IIC SENSOR
				port->motor_type = MOTOR_ERR;
				port->con_state = CON_STATE_WAITING_FOR_DISCONNECT;

			} else if (port->pin5_float_mv < PIN5_BALANCE_LOW) {

				if (port->pin5_float_mv > PIN5_MINITACHO_HIGH2) {
					port->motor_type = MOTOR_NEWTACHO;

				} else if (port->pin5_float_mv > PIN5_MINITACHO_LOW2) {
					port->motor_type = MOTOR_MINITACHO;

				} else {
					port->motor_type = MOTOR_TACHO;

				}

				port->con_state = CON_STATE_DEVICE_CONNECTED;

			} else {
				gpio_direction_output(port->gpio[GPIO_PIN5].gpio, 1);
				port->timer_loop_cnt = 0;
				port->con_state        =  CON_STATE_PIN5_SETTLE;
			}

		// Value5Float is NOT equal to Value5Low
		} else if ((port->pin5_float_mv > PIN5_NEAR_GND) && (port->pin5_float_mv < PIN5_BALANCE_LOW)) {
				// NEW ACTUATOR
				port->motor_type = MOTOR_ERR;
				port->con_state = CON_STATE_WAITING_FOR_DISCONNECT;
		} else {
				port->motor_type = MOTOR_ERR;
				port->con_state = CON_STATE_WAITING_FOR_DISCONNECT;
		}
                break;

	case CON_STATE_PIN5_SETTLE:
                // Update connection type, may need to force pin5 low to determine motor type
		if (port->timer_loop_cnt >= SETTLE_CNT) {
			port->pin5_low_mv = legoev3_analog_out_pin5_value(port->analog, port->id);
 			port->timer_loop_cnt = 0;
			gpio_direction_output(port->gpio[GPIO_PIN5].gpio, 0);

			if (port->pin5_low_mv  < PIN5_MINITACHO_LOW1) {
				port->motor_type = MOTOR_ERR;

			} else if (port->pin5_low_mv  < PIN5_MINITACHO_HIGH1) {
				port->motor_type = MOTOR_MINITACHO;

			} else {
				port->motor_type = MOTOR_TACHO;
			}

		        port->con_state = CON_STATE_DEVICE_CONNECTED;
		}

		break;

 	case CON_STATE_DEVICE_CONNECTED:

		port->timer_loop_cnt = 0;
		if (port->motor_type != MOTOR_ERR) {
			PREPARE_WORK(&port->work, ev3_output_port_register_motor);
			schedule_work(&port->work);
                }
		port->con_state = CON_STATE_WAITING_FOR_DISCONNECT;

		break;

 	case CON_STATE_WAITING_FOR_DISCONNECT:

		new_pin5_mv = legoev3_analog_out_pin5_value(port->analog, port->id);

		if (gpio_get_value(port->gpio[GPIO_PIN6_DIR].gpio))
 			port->timer_loop_cnt = 0;

		if ((new_pin5_mv < PIN5_BALANCE_LOW) || (new_pin5_mv > PIN5_BALANCE_HIGH))
 			port->timer_loop_cnt = 0;

		if ((port->timer_loop_cnt >= REMOVE_CNT) && !work_busy(&port->work) && (port->motor)) {
			PREPARE_WORK(&port->work, ev3_output_port_unregister_motor);
			schedule_work(&port->work);

			port->con_state = CON_STATE_INIT;
                }
                break;

	default:
		port->con_state = CON_STATE_INIT;
		break;
	}

	return HRTIMER_RESTART;
}

static int ev3_output_port_probe(struct legoev3_port_device *pdev)
{
 	struct ev3_output_port_data *port;
 	struct ev3_output_port_platform_data *pdata = pdev->dev.platform_data;
        struct pwm_device *pwm;
	int err;

	if (WARN(!pdata, "Platform data is required."))
		return -EINVAL;

	port = kzalloc(sizeof(struct ev3_output_port_data), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	port->id = pdata->id;
	port->pdev = pdev;
 	port->analog = get_legoev3_analog();
	if (IS_ERR(port->analog)) {
		dev_err(&pdev->dev, "Could not get legoev3-analog device.\n");
		err = PTR_ERR(port->analog);
		goto request_legoev3_analog_fail;
	}

	port->gpio[GPIO_PIN1].gpio	= pdata->pin1_gpio;
	port->gpio[GPIO_PIN1].flags	= GPIOF_IN;
	port->gpio[GPIO_PIN1].label	= "pin1";

	port->gpio[GPIO_PIN2].gpio	= pdata->pin2_gpio;
	port->gpio[GPIO_PIN2].flags	= GPIOF_IN;
	port->gpio[GPIO_PIN2].label	= "pin2";

	port->gpio[GPIO_PIN5].gpio	= pdata->pin5_gpio;
	port->gpio[GPIO_PIN5].flags	= GPIOF_IN;
	port->gpio[GPIO_PIN5].label	= "pin5";

	port->gpio[GPIO_PIN5_INT].gpio	= pdata->pin5_int_gpio;
	port->gpio[GPIO_PIN5_INT].flags	= GPIOF_IN;
	port->gpio[GPIO_PIN5_INT].label	= "pin5_tacho";

	port->gpio[GPIO_PIN6_DIR].gpio	= pdata->pin6_dir_gpio;
	port->gpio[GPIO_PIN6_DIR].flags	= GPIOF_IN;
	port->gpio[GPIO_PIN6_DIR].label	= "pin6";

	err = gpio_request_array(port->gpio, ARRAY_SIZE(port->gpio));
	if (err) {
		dev_err(&pdev->dev, "Requesting GPIOs failed.\n");
		goto gpio_request_array_fail;
 	}

 	err = dev_set_drvdata(&pdev->dev, port);
	if (err)
		goto dev_set_drvdata_fail;

	INIT_WORK(&port->work, NULL);
 
	port->con_state = CON_STATE_INIT;

        /* Set up the connect/disconnect poll timer */

	hrtimer_init(&port->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	port->timer.function = ev3_output_port_timer_callback;
	hrtimer_start(&port->timer, ktime_set(0, OUTPUT_PORT_POLL_NS),
		      HRTIMER_MODE_REL);

	/* Now get the PWM driver registered for this port */

 	pwm = pwm_get(&port->pdev->dev, NULL);
   	if (IS_ERR(pwm)) {
   		dev_err(&pdev->dev, "%s: Could not request pwm device '%s'! (%ld)\n",
   			__func__, pdata->pwm_dev_name, PTR_ERR(pwm));
   		err = PTR_ERR(pwm);
   		goto err_pwm_get;
   	}

	/* Separate platform and generic PWM setup code here */

	if (strncmp(pdata->pwm_dev_name, "ehrpwm", 6) == 0) {
   		dev_err(&pdev->dev, "%s: Setting up PWM '%s'! (%lx)\n",
   			__func__, pdata->pwm_dev_name, PTR_ERR(pwm));

		/* Add any custom setup needed by ehrpwm here */

	 	err = pwm_set_polarity(pwm, 1);
 		if (err) {
 			dev_err(&pdev->dev, "%s: Failed to set pwm polarity! (%d)\n",
 				__func__, err);
	 		goto err_pwm_set_polarity;
	 	}

	} else if (strncmp(pdata->pwm_dev_name, "ecap", 4) == 0) {
   		dev_err(&pdev->dev, "%s: Setting up PWM '%s'! (%lx)\n",
   			__func__, pdata->pwm_dev_name, PTR_ERR(pwm));

		/* Add any custom setup needed by ecap here */

	 	err = pwm_set_polarity(pwm, 0);
 		if (err) {
 			dev_err(&pdev->dev, "%s: Failed to set pwm polarity! (%d)\n",
 				__func__, err);
			goto err_pwm_set_polarity;
		}

	}

	err = pwm_config(pwm, 0, NSEC_PER_SEC / 10000);
 	if (err) {
 		dev_err(&pdev->dev, "%s: Failed to set pwm duty percent and frequency! (%d)\n",
 			__func__, err);
 		goto err_pwm_config;
	}

 	err = pwm_enable(pwm);
 	if (err) {
 		dev_err(&pdev->dev, "%s: Failed to start pwm! (%d)\n",
 			__func__, err);
 		goto err_pwm_start;
 	}

	port->pwm = pwm;

	return 0;

err_pwm_start:
err_pwm_config:
err_pwm_set_polarity:
	pwm_put(pwm);
   err_pwm_get:
	hrtimer_cancel(&port->timer);
   dev_set_drvdata_fail:
 	gpio_free_array(port->gpio, ARRAY_SIZE(port->gpio));
   gpio_request_array_fail:
	put_legoev3_analog(port->analog);
   request_legoev3_analog_fail:
	kfree(port);

	return err;
}

static int ev3_output_port_remove(struct legoev3_port_device *pdev)
{
	struct ev3_output_port_data *port = dev_get_drvdata(&pdev->dev);

 	pwm_disable(port->pwm);
 	pwm_put(port->pwm);

	hrtimer_cancel(&port->timer);
	cancel_work_sync(&port->work);
	if (port->motor)
		legoev3_port_device_unregister(port->motor);
	ev3_output_port_float(port);
	gpio_free_array(port->gpio, ARRAY_SIZE(port->gpio));
	put_legoev3_analog(port->analog);
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(port);

	return 0;
}

static void ev3_output_port_shutdown(struct legoev3_port_device *pdev)
{
 	struct ev3_output_port_data *port = dev_get_drvdata(&pdev->dev);

 	pwm_disable(port->pwm);

 	hrtimer_cancel(&port->timer);
 	cancel_work_sync(&port->work);
}

struct legoev3_port_driver ev3_output_port_driver = {
  	.probe		= ev3_output_port_probe,
	.remove		= ev3_output_port_remove,
	.shutdown	= ev3_output_port_shutdown,
	.driver = {
		.name	= "ev3-output-port",
		.owner	= THIS_MODULE,
	},
};
EXPORT_SYMBOL_GPL(ev3_output_port_driver);
legoev3_port_driver(ev3_output_port_driver);

MODULE_DESCRIPTION("Output port driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("Ralph Hempel <rhempel@hempeldesigngroup.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:ev3-output-port");
