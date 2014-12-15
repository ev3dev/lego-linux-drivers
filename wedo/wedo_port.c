/*
 * Port functions for LEGO WeDo
 *
 * Copyright (C) 2014 Ralph Hempel <rhemple@hempeldesigngroup.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/leds.h>
#include <linux/device.h>
#include <linux/slab.h>

#include "wedo_hub.h"
#include "wedo_port.h"
#include "wedo_sensor.h"
#include "wedo_motor.h"

/*
 * -----------------------------------------------------------------------------
 * This file provides components for interfacing with the ports on the
 * LEGO WeDo USB brick.
 *
 * Each port has its own device node.
 * -----------------------------------------------------------------------------
 */

struct wedo_id_info {
	unsigned char max;
	unsigned char *name;
};

/* The max fields in this table must be in ascending order for the
 * state calculation to work
 */

const struct wedo_id_info wedo_id_infos[] = {
	[WEDO_TYPE_SHORTLO]	= {   9 , "shortlo"	},
	[WEDO_TYPE_BEND]	= {  27 , "bend"	},
	[WEDO_TYPE_TILT]	= {  47 , "tilt"	},
	[WEDO_TYPE_FUTURE]	= {  67 , "future"	},
	[WEDO_TYPE_RAW]		= {  87 , "raw"		},
	[WEDO_TYPE_TOUCH]	= { 109 , "touch"	},
	[WEDO_TYPE_SOUND]	= { 131 , "sound"	},
	[WEDO_TYPE_TEMP]	= { 152 , "temp"	},
	[WEDO_TYPE_LIGHT]	= { 169 , "light"	},
	[WEDO_TYPE_MOTION]	= { 190 , "motion"	},
	[WEDO_TYPE_LIGHTBRICK]	= { 211 , "lightbrick"	},
	[WEDO_TYPE_22]		= { 224 , "22"		},
	[WEDO_TYPE_OPEN]	= { 233 , "open"	},
	[WEDO_TYPE_MOTOR]	= { 246 , "motor"	},
	[WEDO_TYPE_SHORTHI] 	= { 255 , "shorthi"	},
};

/*
 * These functions handle registering lego-sensor devices on WeDo ports
 * as well as the mode callbacks
 */

static int wedo_sensor_set_mode(void *context, u8 mode)
{
	struct wedo_sensor_data *wsd = context;

	if (mode >= wsd->info.num_modes)
		return -EINVAL;

	return 0;
}

static int register_wedo_sensor (struct wedo_port_device *wpd, enum wedo_sensor_types type)
{
	struct wedo_sensor_data *wsd = dev_get_drvdata(&wpd->dev);
	int err;

	if (wsd)
		return -EINVAL;

	wsd = kzalloc(sizeof(struct wedo_sensor_data), GFP_KERNEL);
	if (!wsd)
		return -ENOMEM;

	wsd->wpd = wpd;

	memcpy (&wsd->info, &wedo_sensor_defs[type], sizeof(struct wedo_sensor_info));

	strncpy (wsd->sensor.name, wsd->info.name, LEGO_SENSOR_NAME_SIZE);
	strncpy (wsd->sensor.port_name, wpd->port_name, LEGO_SENSOR_NAME_SIZE);

	dev_info(&wpd->dev, "name %s port_name %s\n", wsd->sensor.name, wsd->sensor.port_name );

	wsd->sensor.num_modes	= wsd->info.num_modes;
	wsd->sensor.mode_info	= wsd->info.mode_info;
	wsd->sensor.set_mode	= wedo_sensor_set_mode;

	wsd->sensor.context		= wsd;

	err = register_lego_sensor(&wsd->sensor, &wpd->dev);
	if (err)
		goto err_register_lego_sensor;

	dev_set_drvdata(&wpd->dev, wsd);

	wedo_sensor_set_mode(wsd, 0);

	return 0;

err_register_lego_sensor:
	kfree(wsd);

	return err;
}

static void unregister_wedo_sensor (struct wedo_port_device *wpd)
{
	struct wedo_sensor_data *wsd = dev_get_drvdata(&wpd->dev);

	if (!wsd)
		return;

	unregister_lego_sensor(&wsd->sensor);
	dev_set_drvdata(&wpd->dev, NULL);
	kfree(wsd);
}

/*
 * These functions handle registering dc_motor devices on WeDo ports
 */

static int register_wedo_motor (struct wedo_port_device *wpd)
{
	struct wedo_motor_data *wmd = dev_get_drvdata(&wpd->dev);
	int err;

	if (wmd)
		return -EINVAL;

	wmd = kzalloc(sizeof(struct wedo_motor_data), GFP_KERNEL);
	if (!wmd)
		return -ENOMEM;

	wmd->wpd = wpd;

	strncpy(wmd->md.name, "wedo-motor", DC_MOTOR_NAME_SIZE);
	strncpy(wmd->md.port_name, wpd->port_name, DC_MOTOR_NAME_SIZE);

	wmd->md.ops = &wedo_motor_ops;
	wmd->md.context = wmd;

	err = register_dc_motor(&wmd->md, &wpd->dev);
	if (err)
		goto err_register_dc_motor;

	dev_set_drvdata(&wpd->dev, wmd);

	return 0;

err_register_dc_motor:
	kfree(wmd);

	return err;
}

static void unregister_wedo_motor (struct wedo_port_device *wpd)
{
	struct wedo_motor_data *wmd = dev_get_drvdata(&wpd->dev);

	if (!wmd)
		return;

	unregister_dc_motor(&wmd->md);
	dev_set_drvdata(&wpd->dev, NULL);
	kfree(wmd);
}

/*
 * These functions handle registering led devices on WeDo ports
 */

struct wedo_lightbrick_data {
	struct wedo_port_device *wpd;
	struct led_classdev cdev;
};

static void wedo_lightbrick_set (struct led_classdev *cdev, enum led_brightness b)
{
	struct wedo_lightbrick_data *wld = container_of(cdev, struct wedo_lightbrick_data, cdev);
	struct wedo_port_device *wpd = wld->wpd;
	struct wedo_hub_device *whd = to_wedo_hub_device (wpd->dev.parent);

	wpd->duty_cycle = b;

	whd->event_callback (whd);
}

static enum led_brightness wedo_lightbrick_get (struct led_classdev *cdev)
{
	struct wedo_lightbrick_data *wld = container_of(cdev, struct wedo_lightbrick_data, cdev);
	struct wedo_port_device *wpd = wld->wpd;

	return wpd->duty_cycle;
}

static int register_wedo_lightbrick (struct wedo_port_device *wpd)
{
	struct wedo_lightbrick_data *wld = dev_get_drvdata(&wpd->dev);
	int err;

	if (wld)
		return -EINVAL;

	wld = kzalloc(sizeof(struct wedo_lightbrick_data), GFP_KERNEL);
	if (!wld)
		return -ENOMEM;

	wld->wpd = wpd;

	wld->cdev.name = wpd->port_name;
	wld->cdev.max_brightness = 100;
	wld->cdev.brightness_set = wedo_lightbrick_set;
	wld->cdev.brightness_get = wedo_lightbrick_get;
	wld->cdev.default_trigger = NULL;

	err = led_classdev_register (&wpd->dev, &wld->cdev);
	if (err)
		goto err_register_lightbrick;

	dev_info(&wpd->dev, "Bound wedo lightbrick as led %s\n",
				dev_name (wld->cdev.dev));

	dev_set_drvdata (&wpd->dev, wld);

	return 0;

err_register_lightbrick:
	kfree(wld);

	return err;
}

static void unregister_wedo_lightbrick (struct wedo_port_device *wpd)
{
	struct wedo_lightbrick_data *wld = dev_get_drvdata(&wpd->dev);

	if (!wld)
		return;

	led_classdev_unregister (&wld->cdev);

	dev_set_drvdata(&wpd->dev, NULL);
	kfree(wld);
}

/*
 * These functions handle registering devices on WeDo ports.
 *
 * There are only two generic types if devices that we handle:
 *
 * Input device ids get registered as lego-sensor class devices
 * Output device ids get registered as dc-motor or leds class devices
 *
 * Currently we only have the tilt and motion sensors for testing
 */

static int register_wedo_device (struct wedo_port_device *wpd, enum wedo_type_id id)
{
	int err = 0;

	wpd->type_id = id;

	switch( wpd->type_id ) {

	case WEDO_TYPE_TILT:
		err = register_wedo_sensor (wpd, WEDO_TILT_SENSOR);
		break;
	case WEDO_TYPE_MOTION:
		err = register_wedo_sensor (wpd, WEDO_MOTION_SENSOR);
		break;
	case WEDO_TYPE_MOTOR:
		err = register_wedo_motor (wpd);
		break;
	case WEDO_TYPE_LIGHTBRICK:
		err = register_wedo_lightbrick (wpd);
		break;
	default:
		break;
	}

	return err;
}

static void unregister_wedo_device (struct wedo_port_device *wpd)
{
	switch( wpd->type_id ) {

	case WEDO_TYPE_TILT:
	case WEDO_TYPE_MOTION:
		unregister_wedo_sensor( wpd );
		break;
	case WEDO_TYPE_MOTOR:
		unregister_wedo_motor (wpd);
		break;
	case WEDO_TYPE_LIGHTBRICK:
		unregister_wedo_lightbrick (wpd);
		break;
	default:
		break;
	}
}

/*
 * Finally, we're at the public driver functions that register the WeDo
 * port devices for each hub.
 */

static void wedo_port_release(struct device *dev)
{
}

struct wedo_port_device *register_wedo_port(unsigned port_num, struct wedo_hub_device *whd)
{
	int err;
	struct wedo_port_device *wpd;

	if (WEDO_PORT_MAX <= port_num)
		return ERR_PTR(-EINVAL);

	/* allocate memory for our new port_device, and initialize it */
	wpd = kzalloc(sizeof(struct wedo_port_device), GFP_KERNEL);
	if (!wpd)
		return ERR_PTR(-ENOMEM);

	wpd->dev.release = wedo_port_release;
	wpd->dev.parent = &whd->dev;

	dev_set_name(&wpd->dev, "port%d", port_num);

	snprintf( wpd->port_name, WEDO_PORT_NAME_SIZE, "%s:wedo%d"
				,whd->port_name, port_num );

	err = device_register(&wpd->dev);

	if (err) {
		dev_err(&wpd->dev, "Failed to register device.\n");
		goto err_wedo_port_register;
	}

	return wpd;

err_wedo_port_register:
	kfree(wpd);

	return ERR_PTR(err);
}

void unregister_wedo_port(struct wedo_port_device *wpd)
{
	if (!wpd)
		return;

	unregister_wedo_device( wpd );
	device_unregister(&wpd->dev);
	kfree(wpd);
}

/* Here's where we update the status of the devices connected to the
 * LEGO WeDo hub - this function is called when the wedo driver has
 * received a complete packet
 *
 * NOTE: only process ID changes if the output value is 0x00 or 0x80
 */
#define WEDO_PORT_TYPE_DEBOUNCE	32

void wedo_port_update_status(struct wedo_port_device *wpd)
{
	int err = 0;
	enum wedo_type_id id;

	struct wedo_sensor_data *wsd = NULL;
	struct wedo_motor_data	*wmd = NULL;

	switch( wpd->type_id ) {

	case WEDO_TYPE_TILT:
	case WEDO_TYPE_MOTION:
		wsd = dev_get_drvdata(&wpd->dev);
		if (wsd && wsd->info.wedo_mode_info[wsd->sensor.mode].analog_cb)
			wsd->info.wedo_mode_info[wsd->sensor.mode].analog_cb (wsd);
		break;
	case WEDO_TYPE_MOTOR:
		wmd = dev_get_drvdata(&wpd->dev);
		break;
	default:
		break;
	}

	for (id=0; id<WEDO_TYPE_MAX; ++id )
		if ( wpd->id <= wedo_id_infos[id].max )
			break;

	if (id != wpd->temp_type_id) {
		wpd->type_debounce = 0;
		wpd->temp_type_id = id;
	}
	else if (WEDO_PORT_TYPE_DEBOUNCE > wpd->type_debounce ) {
		wpd->type_debounce++;
	}
	else if (WEDO_PORT_TYPE_DEBOUNCE == wpd->type_debounce ) {

		if (id != wpd->type_id) {
			unregister_wedo_device( wpd );

			err = register_wedo_device( wpd, id );

			if (err)
				dev_err(&wpd->dev, "Error %d registering device type_id %d to '%s'\n", err, id, dev_name(&wpd->dev));
		}
		wpd->type_debounce++;
	}
}
