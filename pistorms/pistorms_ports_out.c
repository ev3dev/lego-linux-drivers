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
#define PISTORMS_NXTMMX
#include "../sensors/ms_nxtmmx.c"
#undef PISTORMS_NXTMMX

int pistorms_out_ports_register(struct pistorms_data *data)
{
	struct ms_nxtmmx_data *mmx;
	int i, err;

	mmx = kzalloc(sizeof(struct ms_nxtmmx_data) * 2, GFP_KERNEL);
	if (!mmx)
		return -ENOMEM;

	data->out_port_data = mmx;

	for (i = 0; i < 2; i++) {
		snprintf(mmx[i].port_name, LEGO_PORT_NAME_SIZE, "%sM%d",
			 data->name, i + 1);
		mmx[i].i2c_client = data->client;
		mmx[i].index = i;
	}
	err = ms_nxtmmx_register_out_port(&mmx[0]);
	if (err)
		goto err_register_out_port0;
	err = ms_nxtmmx_register_out_port(&mmx[1]);
	if (err)
		goto err_register_out_port1;

	return 0;

err_register_out_port1:
	ms_nxtmmx_unregister_out_port(&mmx[0]);
err_register_out_port0:
	data->out_port_data = NULL;
	kfree(mmx);

	return err;
}

void pistorms_out_ports_unregister(struct pistorms_data *data)
{
	struct ms_nxtmmx_data *mmx = data->out_port_data;

	ms_nxtmmx_unregister_out_port(&mmx[1]);
	ms_nxtmmx_unregister_out_port(&mmx[0]);
	data->out_port_data = NULL;
	kfree(mmx);
}
