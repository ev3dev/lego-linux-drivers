/*
 * Input port driver for LEGO Mindstorms EV3
 *
 * Copyright (C) 2013 David Lechner <david@lechnology.com>
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
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/legoev3/legoev3_analog.h>
#include <linux/legoev3/legoev3_ports.h>

#define INPUT_PORT_POLL_NS	10000000	/* 10 msec */
#define SETTLE_CNT		2		/* 20 msec */
#define ADD_CNT			35		/* 350 msec */
#define REMOVE_CNT		10		/* 100 msec */

#define PIN1_NEAR_5V		4800		/* 4.80V */
#define PIN1_NEAR_PIN2		3100		/* 3.1V */
#define PIN1_TOUCH_HIGH		950 		/* 0.95V */
#define PIN1_TOUCH_LOW		850		/* 0.85V */
#define PIN1_TOUCH_VAR		10		/* 0.01V */
#define PIN1_NEAR_GND		100		/* 0.1V */
#define PIN6_NEAR_GND		150		/* 0.15V */


enum gpio_index {
	GPIO_PIN1,
	GPIO_PIN2,
	GPIO_PIN5,
	GPIO_PIN6,
	GPIO_BUF_ENA,
	GPIO_I2C_CLK,
	NUM_GPIO
};

enum pin5_mux_mode {
	PIN5_MUX_MODE_I2C,
	PIN5_MUX_MODE_UART,
	NUM_PIN5_MUX_MODE
};

enum connection_state {
	CON_STATE_INIT,			/* Set ports to "float" state */
	CON_STATE_INIT_SETTLE,		/* Wait for port to settle */
	CON_STATE_NO_DEV,		/* No device present, wait until something
						interesting happens on one or more
						of the pins and a steady state is
						reached */
	CON_STATE_TEST_NXT_TOUCH,	/* We might have a NXT touch sensor that
						is pressed. We need to watch
						that pin 1 voltage doesn't change
						to be sure */
	CON_STATE_HAVE_NXT,		/* Wait for pin 2 to float */
	CON_STATE_HAVE_EV3,		/* Wait for pin 1 to float */
	CON_STATE_HAVE_I2C,		/* Wait for pin 6 to float */
	CON_STATE_HAVE_PIN5_ERR,	/* Wait for pin 5 to float */
	NUM_CON_STATE
};

enum pin_state_flag {
	PIN_STATE_FLAG_PIN2_LOW,
	PIN_STATE_FLAG_PIN1_LOADED,
	PIN_STATE_FLAG_PIN5_LOW,
	PIN_STATE_FLAG_PIN6_HIGH,
	NUM_PIN_STATE_FLAG
};

enum sensor_type {
	SENSOR_NONE,
	SENSOR_NXT_TOUCH,
	SENSOR_NXT_LIGHT,
	SENSOR_NXT_COLOR,
	SENSOR_NXT_DUMB,
	SENSOR_NXT_I2C,
	SENSOR_EV3_DUMB,
	SENSOR_EV3_UART,
	SENSOR_ERR,
	NUM_SENSOR
};

const struct attribute_group *legoev3_common_sensor_device_type_attr_groups[] = {
	&legoev3_device_type_attr_grp,
	NULL
};

struct device_type legoev3_sensor_device_types[] = {
	[SENSOR_NXT_TOUCH] = {
		.name	= "legoev3-nxt-touch-sensor",
		.groups	= legoev3_common_sensor_device_type_attr_groups,
	},
	[SENSOR_NXT_LIGHT] = {
		.name	= "legoev3-nxt-light-sensor",
		.groups	= legoev3_common_sensor_device_type_attr_groups,
	},
	[SENSOR_NXT_COLOR] = {
		.name	= "legoev3-nxt-color-sensor",
		.groups	= legoev3_common_sensor_device_type_attr_groups,
	},
	[SENSOR_NXT_DUMB] = {
		.name	= "legoev3-nxt-dumb-sensor",
		.groups	= legoev3_common_sensor_device_type_attr_groups,
	},
	[SENSOR_NXT_I2C] = {
		.name	= "legoev3-nxt-i2c-sensor",
		.groups	= legoev3_common_sensor_device_type_attr_groups,
	},
	[SENSOR_EV3_DUMB] = {
		.name	= "legoev3-ev3-dumb-sensor",
		.groups	= legoev3_common_sensor_device_type_attr_groups,
	},
	[SENSOR_EV3_UART] = {
		.name	= "legoev3-ev3-uart-sensor",
		.groups	= legoev3_common_sensor_device_type_attr_groups,
	},
};

struct legoev3_input_port_device {
	struct device dev;
};

/**
 * struct legoev3_input_port_controller - An input port on the EV3 brick
 * @id: Unique identifier for the port.
 * @dev: Pointer to the device object that this is bound to.
 * @analog: pointer to the legoev3-analog device for accessing data from the
 *	analog/digital converter.
 * @gpio: Array of gpio pins used by this input port.
 * @pin5_mux: Pin mux info for the i2c clock and uart Tx pins. These two
 *	functions share a physical pin on the microprocessor so we have to
 *	change the pin mux each time we change which one we are using.
 * @i2c_data: Platform data for i2c-gpio platform device.
 * @i2c_device: Platform device used when we have an i2c sensor.
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
struct legoev3_input_port_controller {
	enum legoev3_input_port_id id;
	struct device *dev;
	struct legoev3_analog_device *analog;
	struct gpio gpio[NUM_GPIO];
	unsigned pin5_mux[NUM_PIN5_MUX_MODE];
	struct i2c_gpio_platform_data i2c_data;
	struct platform_device i2c_device;
	struct work_struct work;
	struct hrtimer timer;
	unsigned timer_loop_cnt;
	enum connection_state con_state;
	unsigned pin_state_flags:NUM_PIN_STATE_FLAG;
	unsigned pin1_mv;
	enum sensor_type sensor_type;
	struct legoev3_input_port_device *sensor;
};

void legoev3_input_port_float(struct legoev3_input_port_controller *ipc)
{
	/* TODO: ensure i2c and uart devices are unregisted */
	gpio_direction_output(ipc->gpio[GPIO_PIN1].gpio, 0);
	gpio_direction_input(ipc->gpio[GPIO_PIN2].gpio);
	gpio_direction_input(ipc->gpio[GPIO_PIN5].gpio);
	gpio_direction_input(ipc->gpio[GPIO_PIN6].gpio);
	gpio_direction_output(ipc->gpio[GPIO_BUF_ENA].gpio, 1); /* active low */
}

static void legoev3_input_port_device_release (struct device *dev)
{
	struct legoev3_input_port_device *ipd =
		container_of(dev, struct legoev3_input_port_device, dev);
printk("%s: device released\n", dev_name(dev));
	kfree(ipd);
}

void legoev3_input_port_register_sensor(struct work_struct *work)
{
	struct legoev3_input_port_controller *ipc =
		container_of(work, struct legoev3_input_port_controller, work);
	struct legoev3_input_port_device *ipd;
	int err;

	ipd = kzalloc(sizeof(struct legoev3_input_port_device), GFP_KERNEL);
	if (!ipd) {
		dev_err(ipc->dev, "Could not set name of sensor attached to %s.\n",
			dev_name(ipc->dev));
		return;
	}

	ipd->dev.parent = ipc->dev;
	if (dev_set_name(&ipd->dev, "sensor")) {
		dev_err(ipc->dev, "Could not set name of sensor attached to %s.\n",
			dev_name(ipc->dev));
		goto set_dev_name_fail;
	}
	if (ipc->sensor_type == SENSOR_NONE || ipc->sensor_type == SENSOR_ERR
	    || ipc->sensor_type >= NUM_SENSOR)
	{
		dev_err(ipc->dev, "Trying to register an invalid sensor on %s.\n",
			dev_name(ipc->dev));
		goto check_sensor_type_fail;
	}
	ipd->dev.type = &legoev3_sensor_device_types[ipc->sensor_type];
	ipd->dev.bus = &legoev3_bus_type;
	ipd->dev.release = legoev3_input_port_device_release;
	err = device_register(&ipd->dev);
	if (err) {
		dev_err(ipc->dev, "Could not set name of sensor attached to %s.\n",
			dev_name(ipc->dev));
		goto device_register_fail;
	}

	ipc->sensor = ipd;
	return;

device_register_fail:
	put_device(&ipd->dev);
	return;
check_sensor_type_fail:
set_dev_name_fail:
	kfree(ipd);
}

void legoev3_input_port_unregister_sensor(struct work_struct *work)
{
	struct legoev3_input_port_controller *ipc =
		container_of(work, struct legoev3_input_port_controller, work);

	device_unregister(&ipc->sensor->dev);
	ipc->sensor = NULL;
}

static enum hrtimer_restart legoev3_input_port_timer_callback(struct hrtimer *timer)
{
	struct legoev3_input_port_controller *ipc =
		container_of(timer, struct legoev3_input_port_controller, timer);
	unsigned new_pin_state_flags = 0;
	unsigned new_pin1_mv = 0;

	hrtimer_forward_now(timer, ktime_set(0, INPUT_PORT_POLL_NS));
	ipc->timer_loop_cnt++;

	switch(ipc->con_state) {
	case CON_STATE_INIT:
		legoev3_input_port_float(ipc);printk("%s: init\n", dev_name(ipc->dev));
		ipc->timer_loop_cnt = 0;
		ipc->sensor_type = SENSOR_NONE;
		ipc->con_state = CON_STATE_INIT_SETTLE;
		break;
	case CON_STATE_INIT_SETTLE:
		if (ipc->timer_loop_cnt >= SETTLE_CNT) {
			ipc->timer_loop_cnt = 0;
			ipc->con_state = CON_STATE_NO_DEV;
		}
		break;
	case CON_STATE_NO_DEV:
		new_pin1_mv = legoev3_analog_in_pin1_value(ipc->analog, ipc->id);
		if (!gpio_get_value(ipc->gpio[GPIO_PIN2].gpio))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN2_LOW);
		if (new_pin1_mv < PIN1_NEAR_5V)
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN1_LOADED);
		if (!gpio_get_value(ipc->gpio[GPIO_PIN5].gpio))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN5_LOW);
		if (gpio_get_value(ipc->gpio[GPIO_PIN6].gpio))
			new_pin_state_flags |= BIT(PIN_STATE_FLAG_PIN6_HIGH);
		if (new_pin_state_flags != ipc->pin_state_flags)
			ipc->timer_loop_cnt = 0;
		else if (new_pin_state_flags && ipc->timer_loop_cnt >= ADD_CNT
			 && !work_pending(&ipc->work))
		{
			if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN2_LOW)) {
				ipc->con_state = CON_STATE_HAVE_NXT;
				if (~new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN5_LOW)
				    && new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN6_HIGH)) {
					if (new_pin1_mv < PIN1_NEAR_GND)
						ipc->sensor_type = SENSOR_NXT_COLOR;
					else
						ipc->sensor_type = SENSOR_NXT_I2C;
				} else if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN5_LOW)) {
					if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN6_HIGH))
						ipc->sensor_type = SENSOR_NXT_DUMB;
					else
						ipc->sensor_type = SENSOR_NXT_LIGHT;
				} else if (new_pin1_mv < PIN1_NEAR_GND)
					ipc->sensor_type = SENSOR_NXT_COLOR;
				else if (new_pin1_mv > PIN1_NEAR_5V)
					ipc->sensor_type = SENSOR_NXT_TOUCH;
				else if (new_pin1_mv > PIN1_TOUCH_LOW
					 && new_pin1_mv < PIN1_TOUCH_HIGH) {
					ipc->con_state = CON_STATE_TEST_NXT_TOUCH;
					ipc->timer_loop_cnt = 0;
					ipc->pin1_mv = new_pin1_mv;
				} else
					ipc->sensor_type = SENSOR_NXT_DUMB;
			} else if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN1_LOADED)) {
				ipc->con_state = CON_STATE_HAVE_EV3;
				if (new_pin1_mv > PIN1_NEAR_PIN2)
					ipc->sensor_type = SENSOR_ERR;
				else if (new_pin1_mv < PIN1_NEAR_GND)
					ipc->sensor_type = SENSOR_EV3_DUMB;
				else
					ipc->sensor_type = SENSOR_EV3_UART;
			} else if (new_pin_state_flags & BIT(PIN_STATE_FLAG_PIN6_HIGH)) {
				ipc->con_state = CON_STATE_HAVE_I2C;
				ipc->sensor_type = SENSOR_NXT_I2C;
			} else {
				ipc->con_state = CON_STATE_HAVE_PIN5_ERR;
				ipc->sensor_type = SENSOR_ERR;
			}
			ipc->timer_loop_cnt = 0;
			if (ipc->sensor_type != SENSOR_ERR) {
				PREPARE_WORK(&ipc->work, legoev3_input_port_register_sensor);
				schedule_work(&ipc->work);
			}
		}
		ipc->pin_state_flags = new_pin_state_flags;
		break;
	case CON_STATE_TEST_NXT_TOUCH:
		if (ipc->timer_loop_cnt >= SETTLE_CNT) {
			ipc->con_state = CON_STATE_HAVE_NXT;
			new_pin1_mv = legoev3_analog_in_pin1_value(ipc->analog, ipc->id);
			if (new_pin1_mv > (ipc->pin1_mv - PIN1_TOUCH_VAR) &&
			    new_pin1_mv < (ipc->pin1_mv + PIN1_TOUCH_VAR))
				ipc->sensor_type = SENSOR_NXT_TOUCH;
			else
				ipc->sensor_type = SENSOR_NXT_DUMB;
		}
		break;
	case CON_STATE_HAVE_NXT:
		if (!gpio_get_value(ipc->gpio[GPIO_PIN2].gpio))
			ipc->timer_loop_cnt = 0;
		break;
	case CON_STATE_HAVE_EV3:
		new_pin1_mv = legoev3_analog_in_pin1_value(ipc->analog, ipc->id);
		if (new_pin1_mv < PIN1_NEAR_5V)
			ipc->timer_loop_cnt = 0;
		break;
	case CON_STATE_HAVE_I2C:
		if (gpio_get_value(ipc->gpio[GPIO_PIN6].gpio))
			ipc->timer_loop_cnt = 0;
		break;
	case CON_STATE_HAVE_PIN5_ERR:
		if (!gpio_get_value(ipc->gpio[GPIO_PIN5].gpio))
			ipc->timer_loop_cnt = 0;
		break;
	default:
		ipc->con_state = CON_STATE_INIT;
		break;
	}
	if (ipc->sensor_type && ipc->timer_loop_cnt >= REMOVE_CNT && !work_pending(&ipc->work)) {
		if (ipc->sensor) {
			PREPARE_WORK(&ipc->work, legoev3_input_port_unregister_sensor);
			schedule_work(&ipc->work);
		}
		ipc->con_state = CON_STATE_INIT;
	}

	return HRTIMER_RESTART;
}

static int __devinit legoev3_input_port_probe(struct device *dev)
{
	struct legoev3_input_port_controller *ipc;
	struct legoev3_input_port_platform_data *pdata = dev->platform_data;
	int err;

	/* TODO: make a kernel option to disable port 1 when using serial port */
	/* or find a way to auto-detect */
	if (pdata->id == LEGOEV3_PORT_IN1)
		return -EINVAL;

printk("probing input port %s\n", dev_name(dev));
	ipc = kzalloc(sizeof(struct legoev3_input_port_controller), GFP_KERNEL);
	if (!ipc)
		return -ENOMEM;

	ipc->id = pdata->id;
	ipc->dev = dev;
	ipc->analog = request_legoev3_analog();
	if (IS_ERR(ipc->analog)) {
		dev_err(dev, "Could not get legoev3-analog device.\n");
		err = PTR_ERR(ipc->analog);
		goto request_legoev3_analog_fail;
	}

	ipc->gpio[GPIO_PIN1].gpio	= pdata->pin1_gpio;
	ipc->gpio[GPIO_PIN1].flags	= GPIOF_OUT_INIT_LOW;
	ipc->gpio[GPIO_PIN2].gpio	= pdata->pin2_gpio;
	ipc->gpio[GPIO_PIN2].flags	= GPIOF_IN;
	ipc->gpio[GPIO_PIN5].gpio	= pdata->pin5_gpio;
	ipc->gpio[GPIO_PIN5].flags	= GPIOF_IN;
	ipc->gpio[GPIO_PIN6].gpio	= pdata->pin6_gpio;
	ipc->gpio[GPIO_PIN6].flags	= GPIOF_IN;
	ipc->gpio[GPIO_BUF_ENA].gpio	= pdata->buf_ena_gpio;
	ipc->gpio[GPIO_BUF_ENA].flags	= GPIOF_OUT_INIT_HIGH;
	ipc->gpio[GPIO_I2C_CLK].gpio	= pdata->i2c_clk_gpio;
	ipc->gpio[GPIO_I2C_CLK].flags	= GPIOF_IN;

	err = gpio_request_array(ipc->gpio, ARRAY_SIZE(ipc->gpio));
	if (err) {
		dev_err(dev, "Requesting GPIOs failed.\n");
		goto gpio_request_array_fail;
	}

	ipc->pin5_mux[PIN5_MUX_MODE_I2C] = pdata->i2c_pin_mux;
	ipc->pin5_mux[PIN5_MUX_MODE_UART] = pdata->uart_pin_mux;

	ipc->i2c_data.sda_pin	= pdata->pin6_gpio;
	ipc->i2c_data.scl_pin	= pdata->i2c_clk_gpio;
	ipc->i2c_data.udelay	= 52;	/* ~9.6kHz */
	ipc->i2c_device.name			= "legoev3-input-port-i2c";
	ipc->i2c_device.id			= pdata->id;
	ipc->i2c_device.dev.parent		= dev;
	ipc->i2c_device.dev.platform_data	= &ipc->i2c_data;

	INIT_WORK(&ipc->work, NULL);

	ipc->con_state = CON_STATE_INIT;
	hrtimer_init(&ipc->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ipc->timer.function = legoev3_input_port_timer_callback;
	hrtimer_start(&ipc->timer, ktime_set(0, INPUT_PORT_POLL_NS),
		      HRTIMER_MODE_REL);

	dev_set_drvdata(dev, ipc);

	return 0;

gpio_request_array_fail:
request_legoev3_analog_fail:
	kfree(ipc);

	return err;
}

static int __devexit legoev3_input_port_remove(struct device *dev)
{
	struct legoev3_input_port_controller *ipc = dev_get_drvdata(dev);
printk("removing device %s\n", dev_name(dev));
	hrtimer_cancel(&ipc->timer);
	cancel_work_sync(&ipc->work);
	legoev3_input_port_float(ipc); /* this unregisters i2c and uart if needed */
	gpio_free_array(ipc->gpio, ARRAY_SIZE(ipc->gpio));
	dev_set_drvdata(dev, NULL);
	kfree(ipc);

	return 0;
}

static void legoev3_input_port_shutdown(struct device *dev)
{
	struct legoev3_input_port_controller *ipc = dev_get_drvdata(dev);

	hrtimer_cancel(&ipc->timer);
	cancel_work_sync(&ipc->work);
}

struct legoev3_port_driver legoev3_input_port_driver = {
	.driver = {
		.name	= "legoev3-input-port",
		.owner	= THIS_MODULE,
		.probe	= legoev3_input_port_probe,
		.remove	= __devexit_p(legoev3_input_port_remove),
		.shutdown	=legoev3_input_port_shutdown,
	},
};
EXPORT_SYMBOL_GPL(legoev3_input_port_driver);
legoev3_port_driver(legoev3_input_port_driver);

MODULE_DESCRIPTION("Input port driver for LEGO Mindstorms EV3");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("legoev3:legoev3-input-port");
