/*
 * Output port driver for LEGO Mindstorms EV3
 *
 * Copyright (C) 2013-2014 Ralph Hempel <rhempel@hempeldesigngroup.com>
 *
 * Based on code that is:
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

// #include <linux/i2c-legoev3.h>
#include <linux/legoev3/legoev3_analog.h>
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_output_port.h>

#include <mach/mux.h>
 
#define OUTPUT_PORT_POLL_NS	( 10000000)	                /* 10 msec */
#define SETTLE_CNT		( 20000000/OUTPUT_PORT_POLL_NS)	/* 20 msec */
#define ADD_CNT			(350000000/OUTPUT_PORT_POLL_NS) /* 350 msec */
#define REMOVE_CNT		(100000000/OUTPUT_PORT_POLL_NS) /* 100 msec */

// 
// #define PIN1_NEAR_5V		4800		/* 4.80V */
// #define PIN1_NEAR_PIN2		3100		/* 3.1V */
// #define PIN1_TOUCH_HIGH		950 		/* 0.95V */
// #define PIN1_TOUCH_LOW		850		/* 0.85V */
// #define PIN1_TOUCH_VAR		10		/* 0.01V */
// #define PIN1_NEAR_GND		100		/* 0.1V */
// #define PIN6_NEAR_GND		150		/* 0.15V */
// 
// #define PIN1_ID_01		206
// #define PIN1_ID_02		417	/* EV3 touch sensor */
// #define PIN1_ID_03		575
// #define PIN1_ID_04		833
// #define PIN1_ID_05		1063
// #define PIN1_ID_06		1241
// #define PIN1_ID_07		1403
// #define PIN1_ID_08		1599
// #define PIN1_ID_09		1795
// #define PIN1_ID_10		2024
// #define PIN1_ID_11		2204
// #define PIN1_ID_12		2382
// #define PIN1_ID_13		2619
// #define PIN1_ID_14		2826	/* 3rd party */
// #define PIN1_ID_VAR		50	/* IDs can be +/- 50mV */
// 
// /* resistor ids for EV3 dumb sensor devices */
// enum ev3_in_dev_id {
// 	EV3_IN_DEV_ID_01,
// 	EV3_IN_DEV_ID_02,
// 	EV3_IN_DEV_ID_03,
// 	EV3_IN_DEV_ID_04,
// 	EV3_IN_DEV_ID_05,
// 	EV3_IN_DEV_ID_06,
// 	EV3_IN_DEV_ID_07,
// 	EV3_IN_DEV_ID_08,
// 	EV3_IN_DEV_ID_09,
// 	EV3_IN_DEV_ID_10,
// 	EV3_IN_DEV_ID_11,
// 	EV3_IN_DEV_ID_12,
// 	EV3_IN_DEV_ID_13,
// 	EV3_IN_DEV_ID_14,
// 	NUM_EV3_IN_DEV_ID,
// 	EV3_IN_DEV_ID_ERR = -1
// };
// 
// static int ev3_in_dev_id_max[NUM_EV3_IN_DEV_ID] = {
// 	[EV3_IN_DEV_ID_01] = PIN1_ID_01 + PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_02] = PIN1_ID_02 + PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_03] = PIN1_ID_03 + PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_04] = PIN1_ID_04 + PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_05] = PIN1_ID_05 + PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_06] = PIN1_ID_06 + PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_07] = PIN1_ID_07 + PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_08] = PIN1_ID_08 + PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_09] = PIN1_ID_09 + PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_10] = PIN1_ID_11 + PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_11] = PIN1_ID_11 + PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_12] = PIN1_ID_12 + PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_13] = PIN1_ID_13 + PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_14] = PIN1_ID_14 + PIN1_ID_VAR,
// };
// 
// static int ev3_in_dev_id_min[NUM_EV3_IN_DEV_ID] = {
// 	[EV3_IN_DEV_ID_01] = PIN1_ID_01 - PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_02] = PIN1_ID_02 - PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_03] = PIN1_ID_03 - PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_04] = PIN1_ID_04 - PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_05] = PIN1_ID_05 - PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_06] = PIN1_ID_06 - PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_07] = PIN1_ID_07 - PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_08] = PIN1_ID_08 - PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_09] = PIN1_ID_09 - PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_10] = PIN1_ID_11 - PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_11] = PIN1_ID_11 - PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_12] = PIN1_ID_12 - PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_13] = PIN1_ID_13 - PIN1_ID_VAR,
// 	[EV3_IN_DEV_ID_14] = PIN1_ID_14 - PIN1_ID_VAR,
// };
// 
// /**
//  * to_ev3_in_dev_id - converts id resistor mV value to an enum
//  * @mv: The value to convert.
//  */
// enum ev3_in_dev_id to_ev3_in_dev_id(int mv)
// {
// 	enum ev3_in_dev_id id = NUM_EV3_IN_DEV_ID;
// 
// 	while (id--) {
// 		if (mv >= ev3_in_dev_id_min[id] && mv <= ev3_in_dev_id_max[id])
// 			return id;
// 	}
// 
// 	return EV3_IN_DEV_ID_ERR;
// }

enum gpio_index {
	GPIO_PIN1,
	GPIO_PIN2,
	GPIO_PIN5,
	GPIO_PIN6,
	NUM_GPIO
};

// enum pin5_mux_mode {
// 	PIN5_MUX_MODE_I2C,
// 	PIN5_MUX_MODE_UART,
// 	NUM_PIN5_MUX_MODE
// };
// 
enum connection_state {
	CON_STATE_INIT,			/* Wait for motor to unregister, then
						Set port to "float" state */
// 	CON_STATE_INIT_SETTLE,		/* Wait for port to settle */
// 	CON_STATE_NO_DEV,		/* No device present, wait until something
// 						interesting happens on one or more
// 						of the pins and a steady state is
// 						reached */
// 	CON_STATE_TEST_NXT_TOUCH,	/* We might have a NXT touch sensor that
// 						is pressed. We need to watch
// 						that pin 1 voltage doesn't change
// 						to be sure */
// 	CON_STATE_HAVE_NXT,		/* Wait for pin 2 to float */
// 	CON_STATE_HAVE_EV3,		/* Wait for pin 1 to float */
// 	CON_STATE_HAVE_I2C,		/* Wait for pin 6 to float */
// 	CON_STATE_HAVE_PIN5_ERR,	/* Wait for pin 5 to float */
	NUM_CON_STATE
};

// enum pin_state_flag {
// 	PIN_STATE_FLAG_PIN2_LOW,
// 	PIN_STATE_FLAG_PIN1_LOADED,
// 	PIN_STATE_FLAG_PIN5_LOW,
// 	PIN_STATE_FLAG_PIN6_HIGH,
// 	NUM_PIN_STATE_FLAG
// };
// 
// enum sensor_type {
// 	SENSOR_NONE,
// 	SENSOR_NXT_TOUCH,
// 	SENSOR_NXT_LIGHT,
// 	SENSOR_NXT_COLOR,
// 	SENSOR_NXT_DUMB,
// 	SENSOR_NXT_I2C,
// 	SENSOR_EV3_ID_01,
// 	SENSOR_EV3_ID_02,
// 	SENSOR_EV3_ID_03,
// 	SENSOR_EV3_ID_04,
// 	SENSOR_EV3_ID_05,
// 	SENSOR_EV3_ID_06,
// 	SENSOR_EV3_ID_07,
// 	SENSOR_EV3_ID_08,
// 	SENSOR_EV3_ID_09,
// 	SENSOR_EV3_ID_10,
// 	SENSOR_EV3_ID_11,
// 	SENSOR_EV3_ID_12,
// 	SENSOR_EV3_ID_13,
// 	SENSOR_EV3_ID_14,
// 	SENSOR_EV3_UART,
// 	SENSOR_ERR,
// 	NUM_SENSOR
// };
//





DEVICE_ATTR(foo, S_IRUGO, NULL, NULL);

static struct attribute *legoev3_output_port_device_type_attrs[] = {
	&dev_attr_foo.attr,
	NULL
};

struct attribute_group legoev3_output_port_device_type_attr_grp = {
	.attrs	= legoev3_output_port_device_type_attrs,
};

// EXPORT_SYMBOL_GPL(legoev3_output_port_device_type_attr_grp);

const struct attribute_group *ev3_output_port_device_type_attr_groups[] = {
	&legoev3_output_port_device_type_attr_grp,
	NULL
};

struct device_type ev3_output_port_device_type = {
	.name	= "ev3-output-port",
	.groups	= ev3_output_port_device_type_attr_groups,
};

 

// struct device_type ev3_sensor_device_types[] = {
// 	[SENSOR_NXT_TOUCH] = {
// 		.name	= "nxt-touch-sensor",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_NXT_LIGHT] = {
// 		.name	= "nxt-light-sensor",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_NXT_COLOR] = {
// 		.name	= "nxt-color-sensor",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_NXT_DUMB] = {
// 		.name	= "nxt-generic-analog-sensor",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_NXT_I2C] = {
// 		.name	= "nxt-i2c-sensor",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_ID_01] = {
// 		.name	= "ev3-sensor-01",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_ID_02] = {
// 		.name	= "ev3-touch-sensor",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_ID_03] = {
// 		.name	= "ev3-sensor-03",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_ID_04] = {
// 		.name	= "ev3-sensor-04",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_ID_05] = {
// 		.name	= "ev3-sensor-05",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_ID_06] = {
// 		.name	= "ev3-sensor-06",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_ID_07] = {
// 		.name	= "ev3-sensor-07",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_ID_08] = {
// 		.name	= "ev3-sensor-08",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_ID_09] = {
// 		.name	= "ev3-sensor-09",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_ID_10] = {
// 		.name	= "ev3-sensor-10",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_ID_11] = {
// 		.name	= "ev3-sensor-11",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_ID_12] = {
// 		.name	= "ev3-sensor-12",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_ID_13] = {
// 		.name	= "ev3-sensor-13",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_ID_14] = {
// 		.name	= "ev3-sensor-14",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// 	[SENSOR_EV3_UART] = {
// 		.name	= "ev3-uart-sensor",
// 		.groups	= ev3_sensor_device_type_attr_groups,
// 	},
// };

/**
 * struct ev3_input_port_data - Driver data for an input port on the EV3 brick
 * @id: Unique identifier for the port.
 * @pdev: Pointer to the legoev3_port_device that is bound to this instance.
 * @analog: pointer to the legoev3-analog device for accessing data from the
 *	analog/digital converter.
 * @gpio: Array of gpio pins used by this input port.
 * @pin5_mux: Pin mux info for the i2c clock and uart Tx pins. These two
 *	functions share a physical pin on the microprocessor so we have to
 *	change the pin mux each time we change which one we are using.
 * @i2c_data: Platform data for i2c-gpio platform device.
 * @i2c_pdev_info: Platform device information for creating a new i2c-gpio
 *	device each time we connect an i2c sensor.
 * @i2c_pdev: I2C platform device.
 * @work: Worker for registering and unregistering sensors when they are
 *	connected and disconnected.
 * @timer: Polling timer to monitor the port.
 * @timer_loop_cnt: Used to measure time in the polling loop.
 * @con_state: The current state of the port.
 * @pin_state_flags: Used in the polling loop to track certain changes in the
 *	state of the port's pins.
 * @pin1_mv: Used in the polling loop to track changes in pin 1 voltage.
 * @sensor_type: The type of sensor currently connected.
 * @sensor: Pointer to the sensor device that is connected to the input port.
 */
struct ev3_output_port_data {
	enum ev3_output_port_id id;
	struct legoev3_port_device *pdev;
	struct legoev3_analog_device *analog;
 	struct gpio gpio[NUM_GPIO];

// 	unsigned pin5_mux[NUM_PIN5_MUX_MODE];
// 	struct i2c_legoev3_platform_data i2c_data;
// 	struct platform_device_info i2c_pdev_info;
// 	struct platform_device *i2c_pdev;
 	struct work_struct work;
 	struct hrtimer timer;
 	unsigned timer_loop_cnt;
 	enum connection_state con_state;
// 	unsigned pin_state_flags:NUM_PIN_STATE_FLAG;
// 	unsigned pin1_mv;
// 	enum sensor_type sensor_type;
// 	struct legoev3_port_device *sensor;
};
// 
// int ev3_input_port_get_pin1_mv(struct legoev3_port_device *in_port)
// {
// 	struct ev3_input_port_data *port = dev_get_drvdata(&in_port->dev);
// 
// 	return legoev3_analog_in_pin1_value(port->analog, port->id);
// }
// EXPORT_SYMBOL_GPL(ev3_input_port_get_pin1_mv);
// 
// int ev3_input_port_get_pin6_mv(struct legoev3_port_device *in_port)
// {
// 	struct ev3_input_port_data *port = dev_get_drvdata(&in_port->dev);
// 
// 	return legoev3_analog_in_pin6_value(port->analog, port->id);
// }
// EXPORT_SYMBOL_GPL(ev3_input_port_get_pin6_mv);
// 
// void ev3_input_port_set_pin1_out(struct legoev3_port_device *in_port,
// 					int value)
// {
// 	struct ev3_input_port_data *port = dev_get_drvdata(&in_port->dev);
// 
// 	gpio_set_value(port->gpio[GPIO_PIN1].gpio, value);
// }
// EXPORT_SYMBOL_GPL(ev3_input_port_set_pin1_out);
// 
// int ev3_input_port_register_i2c(struct legoev3_port_device *in_port,
// 				struct device *parent)
// {
// 	struct ev3_input_port_data *port = dev_get_drvdata(&in_port->dev);
// 	struct platform_device *pdev;
// 	int err;
// 
// 	gpio_set_value(port->gpio[GPIO_BUF_ENA].gpio, 0); /* active low */
// 	err = davinci_cfg_reg(port->pin5_mux[PIN5_MUX_MODE_I2C]);
// 	if (err) {
// 		dev_err(&in_port->dev, "Pin 5 mux failed for i2c device.\n");
// 		goto davinci_cfg_reg_fail;
// 	}
// 	port->i2c_pdev_info.parent = parent;
// 	pdev = platform_device_register_full(&port->i2c_pdev_info);
// 	if (IS_ERR(pdev)) {
// 		dev_err(&in_port->dev, "Could not register i2c device.\n");
// 		err = PTR_ERR(pdev);
// 		goto platform_device_register_fail;
// 	}
// 	port->i2c_pdev = pdev;
// 
// 	return 0;
// 
// platform_device_register_fail:
// davinci_cfg_reg_fail:
// 	gpio_set_value(port->gpio[GPIO_BUF_ENA].gpio, 1); /* active low */
// 
// 	return err;
// }
// EXPORT_SYMBOL_GPL(ev3_input_port_register_i2c);
// 
// void ev3_input_port_unregister_i2c(struct legoev3_port_device *in_port)
// {
// 	struct ev3_input_port_data *port = dev_get_drvdata(&in_port->dev);
// 
// 	platform_device_unregister(port->i2c_pdev);
// 	port->i2c_pdev = NULL;
// 	gpio_set_value(port->gpio[GPIO_BUF_ENA].gpio, 1); /* active low */
// }
// EXPORT_SYMBOL_GPL(ev3_input_port_unregister_i2c);
// 
// int ev3_input_port_enable_uart(struct legoev3_port_device *in_port)
// {
// 	struct ev3_input_port_data *port = dev_get_drvdata(&in_port->dev);
// 	int err;
// 
// 	err = davinci_cfg_reg(port->pin5_mux[PIN5_MUX_MODE_UART]);
// 	if (err) {
// 		dev_err(&in_port->dev, "Pin 5 mux failed for uart device.\n");
// 		return err;
// 	}
// 	gpio_set_value(port->gpio[GPIO_BUF_ENA].gpio, 0); /* active low */
// 
// 	return 0;
// }
// EXPORT_SYMBOL_GPL(ev3_input_port_enable_uart);
// 
// void ev3_input_port_disable_uart(struct legoev3_port_device *in_port)
// {
// 	struct ev3_input_port_data *port = dev_get_drvdata(&in_port->dev);
// 
// 	gpio_set_value(port->gpio[GPIO_BUF_ENA].gpio, 1); /* active low */
// }
// EXPORT_SYMBOL_GPL(ev3_input_port_disable_uart);
// 
// void ev3_input_port_float(struct ev3_input_port_data *port)
// {
// 	gpio_direction_output(port->gpio[GPIO_PIN1].gpio, 0);
// 	gpio_direction_input(port->gpio[GPIO_PIN2].gpio);
// 	gpio_direction_input(port->gpio[GPIO_PIN5].gpio);
// 	gpio_direction_input(port->gpio[GPIO_PIN6].gpio);
// 	gpio_direction_output(port->gpio[GPIO_BUF_ENA].gpio, 1); /* active low */
// }
// 
// void ev3_input_port_register_sensor(struct work_struct *work)
// {
// 	struct ev3_input_port_data *port =
// 			container_of(work, struct ev3_input_port_data, work);
// 	struct legoev3_port_device *sensor;
// 	struct ev3_sensor_platform_data pdata;
// 
// 	if (port->sensor_type == SENSOR_NONE
// 	    || port->sensor_type == SENSOR_ERR
// 	    || port->sensor_type >= NUM_SENSOR)
// 	{
// 		dev_err(&port->pdev->dev, "Trying to register an invalid sensor on %s.\n",
// 			dev_name(&port->pdev->dev));
// 		return;
// 	}
// 
// 	/* Give the sensor time to boot */
// 	if (port->sensor_type == SENSOR_NXT_I2C)
// 		msleep(1000);
// 
// 	pdata.in_port = port->pdev;
// 	sensor = legoev3_port_device_register("sensor", -1,
// 				&ev3_sensor_device_types[port->sensor_type],
// 				&pdata, sizeof(struct ev3_sensor_platform_data),
// 				&port->pdev->dev);
// 	if (IS_ERR(sensor)) {
// 		dev_err(&port->pdev->dev, "Could not register sensor on port %s.\n",
// 			dev_name(&port->pdev->dev));
// 		return;
// 	}
// 
// 	port->sensor = sensor;
// 
// 	return;
// }
// 
// void ev3_input_port_unregister_sensor(struct work_struct *work)
// {
// 	struct ev3_input_port_data *port =
// 			container_of(work, struct ev3_input_port_data, work);
// 
// 	legoev3_port_device_unregister(port->sensor);
// 	port->sensor = NULL;
// }

static enum hrtimer_restart ev3_output_port_timer_callback(struct hrtimer *timer)
{
 	struct ev3_output_port_data *port =
 			container_of(timer, struct ev3_output_port_data, timer);
// 	unsigned new_pin_state_flags = 0;
// 	unsigned new_pin1_mv = 0;
// 
 	hrtimer_forward_now(timer, ktime_set(0, OUTPUT_PORT_POLL_NS));
 	port->timer_loop_cnt++;

        if( 25*port->id == (port->timer_loop_cnt % 100) ) {
        	pr_warning("output port timer for %s is at %d\n", port->pdev->name, port->timer_loop_cnt );
	}
// 
// 	switch(port->con_state) {
// 	case CON_STATE_INIT:
// 		if (!port->sensor) {
// 			ev3_input_port_float(port);
// 			port->timer_loop_cnt = 0;
// 			port->sensor_type = SENSOR_NONE;
// 			port->con_state = CON_STATE_INIT_SETTLE;
// 		}
// 		break;
// 	case CON_STATE_INIT_SETTLE:
// 		if (port->timer_loop_cnt >= SETTLE_CNT) {
// 			port->timer_loop_cnt = 0;
// 			port->con_state = CON_STATE_NO_DEV;
// 		}
// 		break;
// 	case CON_STATE_NO_DEV:
// 		new_pin1_mv = legoev3_analog_in_pin1_value(port->analog, port->id);
// 		if (!gpio_get_value(port->gpio[GPIO_PIN2].gpio))
// 			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN2_LOW);
// 		if (new_pin1_mv < PIN1_NEAR_5V)
// 			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN1_LOADED);
// 		if (!gpio_get_value(port->gpio[GPIO_PIN5].gpio))
// 			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN5_LOW);
// 		if (gpio_get_value(port->gpio[GPIO_PIN6].gpio))
// 			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN6_HIGH);
// 		if (new_pin_state_flags != port->pin_state_flags)
// 			port->timer_loop_cnt = 0;
// 		else if (new_pin_state_flags && port->timer_loop_cnt >= ADD_CNT
// 			 && !work_busy(&port->work))
// 		{
// 			if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN2_LOW)) {
// 				port->con_state = CON_STATE_HAVE_NXT;
// 				if (~new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN5_LOW)
// 				    && new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN6_HIGH)) {
// 					if (new_pin1_mv < PIN1_NEAR_GND)
// 						port->sensor_type = SENSOR_NXT_COLOR;
// 					else
// 						port->sensor_type = SENSOR_NXT_I2C;
// 				} else if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN5_LOW)) {
// 					if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN6_HIGH))
// 						port->sensor_type = SENSOR_NXT_DUMB;
// 					else
// 						port->sensor_type = SENSOR_NXT_LIGHT;
// 				} else if (new_pin1_mv < PIN1_NEAR_GND)
// 					port->sensor_type = SENSOR_NXT_COLOR;
// 				else if (new_pin1_mv > PIN1_NEAR_5V)
// 					port->sensor_type = SENSOR_NXT_TOUCH;
// 				else if (new_pin1_mv > PIN1_TOUCH_LOW
// 					 && new_pin1_mv < PIN1_TOUCH_HIGH) {
// 					port->con_state = CON_STATE_TEST_NXT_TOUCH;
// 					port->timer_loop_cnt = 0;
// 					port->pin1_mv = new_pin1_mv;
// 				} else
// 					port->sensor_type = SENSOR_NXT_DUMB;
// 			} else if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN1_LOADED)) {
// 				port->con_state = CON_STATE_HAVE_EV3;
// 				if (new_pin1_mv > PIN1_NEAR_PIN2)
// 					port->sensor_type = SENSOR_ERR;
// 				else if (new_pin1_mv < PIN1_NEAR_GND)
// 					port->sensor_type = SENSOR_EV3_UART;
// 				else {
// 					switch(to_ev3_in_dev_id(new_pin1_mv)) {
// 					case EV3_IN_DEV_ID_01:
// 						port->sensor_type = SENSOR_EV3_ID_01;
// 						break;
// 					case EV3_IN_DEV_ID_02:
// 						port->sensor_type = SENSOR_EV3_ID_02;
// 						break;
// 					case EV3_IN_DEV_ID_03:
// 						port->sensor_type = SENSOR_EV3_ID_03;
// 						break;
// 					case EV3_IN_DEV_ID_04:
// 						port->sensor_type = SENSOR_EV3_ID_04;
// 						break;
// 					case EV3_IN_DEV_ID_05:
// 						port->sensor_type = SENSOR_EV3_ID_05;
// 						break;
// 					case EV3_IN_DEV_ID_06:
// 						port->sensor_type = SENSOR_EV3_ID_06;
// 						break;
// 					case EV3_IN_DEV_ID_07:
// 						port->sensor_type = SENSOR_EV3_ID_07;
// 						break;
// 					case EV3_IN_DEV_ID_08:
// 						port->sensor_type = SENSOR_EV3_ID_08;
// 						break;
// 					case EV3_IN_DEV_ID_09:
// 						port->sensor_type = SENSOR_EV3_ID_09;
// 						break;
// 					case EV3_IN_DEV_ID_10:
// 						port->sensor_type = SENSOR_EV3_ID_10;
// 						break;
// 					case EV3_IN_DEV_ID_11:
// 						port->sensor_type = SENSOR_EV3_ID_11;
// 						break;
// 					case EV3_IN_DEV_ID_12:
// 						port->sensor_type = SENSOR_EV3_ID_12;
// 						break;
// 					case EV3_IN_DEV_ID_13:
// 						port->sensor_type = SENSOR_EV3_ID_13;
// 						break;
// 					case EV3_IN_DEV_ID_14:
// 						port->sensor_type = SENSOR_EV3_ID_14;
// 						break;
// 					default:
// 						port->sensor_type = SENSOR_ERR;
// 						break;
// 					}
// 				}
// 			} else if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN6_HIGH)) {
// 				port->con_state = CON_STATE_HAVE_I2C;
// 				port->sensor_type = SENSOR_NXT_I2C;
// 			} else {
// 				port->con_state = CON_STATE_HAVE_PIN5_ERR;
// 				port->sensor_type = SENSOR_ERR;
// 			}
// 			port->timer_loop_cnt = 0;
// 			if (port->sensor_type != SENSOR_ERR) {
// 				PREPARE_WORK(&port->work, ev3_input_port_register_sensor);
// 				schedule_work(&port->work);
// 			}
// 		}
// 		port->pin_state_flags = new_pin_state_flags;
// 		break;
// 	case CON_STATE_TEST_NXT_TOUCH:
// 		if (port->timer_loop_cnt >= SETTLE_CNT) {
// 			port->con_state = CON_STATE_HAVE_NXT;
// 			new_pin1_mv = legoev3_analog_in_pin1_value(port->analog, port->id);
// 			if (new_pin1_mv > (port->pin1_mv - PIN1_TOUCH_VAR) &&
// 			    new_pin1_mv < (port->pin1_mv + PIN1_TOUCH_VAR))
// 				port->sensor_type = SENSOR_NXT_TOUCH;
// 			else
// 				port->sensor_type = SENSOR_NXT_DUMB;
// 		}
// 		break;
// 	case CON_STATE_HAVE_NXT:
// 		if (!gpio_get_value(port->gpio[GPIO_PIN2].gpio))
// 			port->timer_loop_cnt = 0;
// 		break;
// 	case CON_STATE_HAVE_EV3:
// 		new_pin1_mv = legoev3_analog_in_pin1_value(port->analog, port->id);
// 		if (new_pin1_mv < PIN1_NEAR_5V)
// 			port->timer_loop_cnt = 0;
// 		break;
// 	case CON_STATE_HAVE_I2C:
// 		if (gpio_get_value(port->gpio[GPIO_PIN6].gpio))
// 			port->timer_loop_cnt = 0;
// 		break;
// 	case CON_STATE_HAVE_PIN5_ERR:
// 		if (!gpio_get_value(port->gpio[GPIO_PIN5].gpio))
// 			port->timer_loop_cnt = 0;
// 		break;
// 	default:
// 		port->con_state = CON_STATE_INIT;
// 		break;
// 	}
// 	if (port->sensor_type
// 	    && port->timer_loop_cnt >= REMOVE_CNT && !work_busy(&port->work))
// 	{
// 		if (port->sensor) {
// 			PREPARE_WORK(&port->work, ev3_input_port_unregister_sensor);
// 			schedule_work(&port->work);
// 		}
// 		port->con_state = CON_STATE_INIT;
// 	}
// 
	return HRTIMER_RESTART;
}

static int __devinit ev3_output_port_probe(struct legoev3_port_device *pdev)
{
 	struct ev3_output_port_data *port;
 	struct ev3_output_port_platform_data *pdata = pdev->dev.platform_data;
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

	port->gpio[GPIO_PIN6].gpio	= pdata->pin6_gpio;
	port->gpio[GPIO_PIN6].flags	= GPIOF_IN;
	port->gpio[GPIO_PIN6].label	= "pin6";

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
	hrtimer_init(&port->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	port->timer.function = ev3_output_port_timer_callback;
	hrtimer_start(&port->timer, ktime_set(0, OUTPUT_PORT_POLL_NS),
		      HRTIMER_MODE_REL);

	return 0;
 
   dev_set_drvdata_fail:
 	gpio_free_array(port->gpio, ARRAY_SIZE(port->gpio));
   gpio_request_array_fail:
	put_legoev3_analog(port->analog);
   request_legoev3_analog_fail:
	kfree(port);

	return err;
}

static int __devexit ev3_output_port_remove(struct legoev3_port_device *pdev)
{
	struct ev3_output_port_data *port = dev_get_drvdata(&pdev->dev);

	hrtimer_cancel(&port->timer);
	cancel_work_sync(&port->work);
//	if (port->sensor)
//		legoev3_port_device_unregister(port->sensor);
//	ev3_output_port_float(port);
	gpio_free_array(port->gpio, ARRAY_SIZE(port->gpio));
	put_legoev3_analog(port->analog);
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(port);

	return 0;
}

static void ev3_output_port_shutdown(struct legoev3_port_device *pdev)
{
// 	struct ev3_input_port_data *port = dev_get_drvdata(&pdev->dev);
// 
// 	hrtimer_cancel(&port->timer);
// 	cancel_work_sync(&port->work);
}

struct legoev3_port_driver ev3_output_port_driver = {
  	.probe		= ev3_output_port_probe,
	.remove		= __devexit_p(ev3_output_port_remove),
//	.shutdown	= ev3_output_port_shutdown,
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
