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
#include <linux/legoev3/legoev3_ports.h>
#include <linux/legoev3/ev3_output_port.h>
 #include <linux/legoev3/tacho_motor_class.h>

#include <asm/bug.h>

// #define PIN6_NEAR_GND		250		/* 0.25V */

struct ev3_tacho_motor_data {
	struct tacho_motor_device tm;
	struct legoev3_port_device *out_port;

	unsigned dir_state;
	unsigned int_state;

//        irqreturn_t (*isr)(struct tacho_motor_device *)
};

// static void tacho_motor_isr(struct tacho_motor_device *tm)
// {
// extern int ev3_output_port_get_pin56_levels(struct legoev3_port_device *out_port, int *, int *)
//static irqreturn_t bfin_rotary_isr(int irq, void *dev_id)
//  62 {
//  63         struct platform_device *pdev = dev_id;
//  64         struct bfin_rot *rotary = platform_get_drvdata(pdev);
//  65         int delta;
//  66 
//  67         switch (bfin_read_CNT_STATUS()) {
//  68 
//  69         case ICII:
//  70                 break;
//  71 
//  72         case UCII:
//  73         case DCII:
//  74                 delta = bfin_read_CNT_COUNTER();
//  75                 if (delta)
//  76                         report_rotary_event(rotary, delta);
//  77                 break;
//  78 
//  79         case CZMII:
//  80                 report_key_event(rotary->input, rotary->button_key);
//  81                 break;
//  82 
//  83         default:
//  84                 break;
//  85         }
//  86 
//  87         bfin_write_CNT_COMMAND(W1LCNT_ZERO);    /* Clear COUNTER */
//  88         bfin_write_CNT_STATUS(-1);      /* Clear STATUS */
//  89 
//  90         return IRQ_HANDLED;
//  91 }
	
// }


// static bool ev3_touch_sensor_pressed(struct touch_sensor_device *ts)
// {
// 	struct ev3_touch_sensor_data *ev3_ts =
// 			container_of(ts, struct ev3_touch_sensor_data, ts);
// 
// 	return ev3_input_port_get_pin6_mv(ev3_ts->in_port) > PIN6_NEAR_GND;
// }

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
//
//	ev3_ts->ts.pressed = ev3_touch_sensor_pressed;
	ev3_tm->out_port = pdata->out_port;

	err = register_tacho_motor(&ev3_tm->tm, &motor->dev);
	if (err)
		goto register_tacho_motor_fail;

	err = dev_set_drvdata(&motor->dev, ev3_tm);
	if (err)
		goto dev_set_drvdata_fail;

	dev_info(&motor->dev, "Tacho Motor connected to port %s\n",
		dev_name(&ev3_tm->out_port->dev));

        // Here's where we set up the port pins on a per-port basis
        //

//         err = ev3_output_port_register_irq_pin5( ev3_tm->out_port );
//         err = request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags,
// 153             const char *name, void *dev);
// 154 
	return 0;

dev_set_drvdata_fail:
	unregister_tacho_motor(&ev3_tm->tm);

register_tacho_motor_fail:
	kfree(ev3_tm);

	return err;
}

static int __devexit ev3_tacho_motor_remove(struct legoev3_port_device *motor)
{
	struct ev3_tacho_motor_data *ev3_tm = dev_get_drvdata(&motor->dev);

	dev_info(&motor->dev, "Tacho motor removed from port %s\n",
		 dev_name(&ev3_tm->out_port->dev));
	unregister_tacho_motor(&ev3_tm->tm);
	dev_set_drvdata(&motor->dev, NULL);
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

