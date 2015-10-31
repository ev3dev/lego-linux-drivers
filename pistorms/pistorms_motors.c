/*
 * mindsensors.com PiStorms motor device driver
 *
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

#include "pistorms.h"

/* most of the implementation is shared with the NXTMMX */
#define PISTORMS
#include "../sensors/ms_nxtmmx.c"

int pistorms_motors_register(struct pistorms_data *data)
{
	struct ms_nxtmmx_data *mmx;
	int i, err;

	mmx = kzalloc(sizeof(struct ms_nxtmmx_data) * 2, GFP_KERNEL);
	if (!mmx)
		return -ENOMEM;

	data->motors_data = mmx;

	for (i = 0; i < 2; i++) {
		mmx[i].tm.driver_name = "pistorms";
		snprintf(mmx[i].port_name, LEGO_PORT_NAME_SIZE, "%sM%d",
			 data->name, i + 1);
		mmx[i].tm.port_name = mmx[i].port_name;
		mmx[i].tm.ops = &ms_nxtmmx_tacho_motor_ops;
		mmx[i].tm.context = &mmx[i];
		mmx[i].i2c_client = data->client;
		mmx[i].index = i;
		mmx[i].command_flags = CMD_FLAGS_DEFAULT_VALUE;
	}

	err = register_tacho_motor(&mmx[0].tm, &data->client->dev);
	if (err)
		goto err_register_tacho_motor0;
	err = register_tacho_motor(&mmx[1].tm, &data->client->dev);
	if (err)
		goto err_register_tacho_motor1;

	return 0;

err_register_tacho_motor1:
	unregister_tacho_motor(&mmx[0].tm);
err_register_tacho_motor0:
	data->motors_data = NULL;
	kfree(mmx);

	return err;
}

void pistorms_motors_unregister(struct pistorms_data *data)
{
	struct ms_nxtmmx_data *mmx = data->motors_data;

	unregister_tacho_motor(&mmx[1].tm);
	unregister_tacho_motor(&mmx[0].tm);
	data->motors_data = NULL;
	kfree(mmx);
}
