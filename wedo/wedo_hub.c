/*
 * USB Driver for LEGO WeDo
 *
 * Copyright (C) 2014 Ralph Hempel <rhemple@hempeldesigngroup.com>
 * Copyright (C) 2014 David Lechner <david@lechnology.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/bitops.h>

#include <lego.h>
#include <lego_port_class.h>
#include <lego_sensor_class.h>

#include "wedo.h"

/*
 * Version Information
 */
#define DRIVER_VERSION "v2.0"
#define DRIVER_AUTHOR "Ralph Hempel <rhempel@hempeldesigngroup.com>"
#define DRIVER_DESC "LEGO WeDo USB Hub Driver"
#define DRIVER_LICENSE "GPL"

#define WEDO_STATUS_DEBOUNCE	8

/* table of devices that work with this driver */
static const struct usb_device_id wedo_table [] = {
	{ USB_DEVICE(0x0694, 0x0003) },
	{ },
};

MODULE_DEVICE_TABLE (usb, wedo_table);

#define WEDO_HUB_CTL_BIT_ECHO		BIT(0)
#define WEDO_HUB_CTL_BIT_RESET		BIT(3)
#define WEDO_HUB_CTL_BIT_SHUTDOWN	BIT(4)
#define WEDO_HUB_CTL_BIT_LOW_POWER	BIT(5)
#define WEDO_HUB_CTL_BIT_HIGH_POWER	BIT(6)
#define WEDO_HUB_CTL_BIT_ERROR		BIT(7)

enum wedo_hub_mode {
	WEDO_HUB_MODE_HUB,
	NUM_WEDO_HUB_MODES
};

enum wedo_hub_command {
	WEDO_HUB_CMD_OUTPUT_OFF,
	WEDO_HUB_CMD_OUTPUT_ON,
	WEDO_HUB_CMD_CLEAR_ERROR,
	NUM_WEDO_HUB_CMDS
};


struct wedo_hub_sensor_info {
	struct lego_sensor_mode_info mode_info[NUM_WEDO_HUB_MODES];
	struct lego_sensor_cmd_info cmd_info[NUM_WEDO_HUB_CMDS];
};

static const struct wedo_hub_sensor_info wedo_hub_sensor_defs[] = {
	[0] = {
		/**
		 * @vendor_name: LEGO
		 * @vendor_part_name: WeDo USB Hub
		 * @vendor_part_number: 9581
		 * @vendor_website: https://education.lego.com/en-us/products/wedo-usb-hub/9581
		 * @name: wedo-hub
		 * @num_modes: 1
		 * @num_commands: 3
		 * @module: wedo
		 */
		.mode_info = {
			[WEDO_HUB_MODE_HUB] = {
				/**
				 * .. [#wedo-hub-mode0-value0] Status Bits:
				 *
				 *    =====  =============
				 *     Bit    Description
				 *    =====  =============
				 *     0      Echo
				 *     1
				 *     2
				 *     3
				 *     4
				 *     5
				 *     6      High Power
				 *     7      Outputs Off
				 *    =====  =============
				 *
				 * @description: Hub status
				 * @value0: Status bits
				 * @value0_footnote: [#wedo-hub-mode0-value0]_
				 * @value1: Voltage (millivolts)
				 */
				.name		= "HUB",
				.data_sets	= 2,
				.data_type	= LEGO_SENSOR_DATA_U16,
			},
		},
		.cmd_info = {
			[WEDO_HUB_CMD_OUTPUT_OFF] = {
				/**
				 * @description: Turns off the outputs of the ports.
				 */
				.name	= "OUT-OFF",
			},
			[WEDO_HUB_CMD_OUTPUT_ON] = {
				/**
				 * @description: Turns on the outputs of the ports.
				 */
				.name	= "OUT-ON",
			},
			[WEDO_HUB_CMD_CLEAR_ERROR] = {
				/**
				 * @description: Clears error.
				 */
				.name	= "CLEAR-ERR",
			},
		},
	},
};

/*
 * struct usb_wedo: Structure to hold all of our device specific stuff
 * @usb_device: The USB device for this device
 * @interface: The USB interface for this device
 * @wedo_hub: The LEGO sensor device that represents the WeDo hub itself
 * @wedo_hub_info: Sensor info used by wedo_hub
 * @wedo_ports: The LEGO port devices for the 2 ports on the WeDo hub
 * @in_dma:
 * @in_buf: The read data buffer
 * @in_urb: the urb to read data with
 * @cr: The USB Control Request
 * @ctl_dma:
 * @ctl_buf: The control data buffer
 * @ctl_urb: the urb to write data with
 * @io_lock: lock for I/O operations
 * @status_debounce: Status debounce count after output change
 * @output_bits: Control bits sent to hub during output change
 * @update_output: Output module requested an output change
 * @output_pending: Control URB has been submitted
 * @io_halt:  IO to the WeDo hub must stop
 * */
struct usb_wedo {
	struct usb_device	*usb_device;
	struct usb_interface	*usb_interface;
	char 			address[LEGO_NAME_SIZE + 1];
	struct lego_sensor_device wedo_hub;
	struct wedo_hub_sensor_info wedo_hub_info;
	struct wedo_port_data	*wedo_ports[WEDO_PORT_MAX];
	dma_addr_t		in_dma;
	unsigned char		*in_buf;
	struct urb		*in_urb;
	struct usb_ctrlrequest	cr;
	dma_addr_t		ctl_dma;
	unsigned char		*ctl_buf;
	struct urb		*ctl_urb;
	spinlock_t		io_lock;
	unsigned		status_debounce;
	u8			output_bits;
	bool			update_output;
	bool			output_pending;
	bool			io_halt;
};

void wedo_hub_request_output_update(struct usb_interface *interface)
{
	struct usb_wedo *wedo = usb_get_intfdata(interface);
	unsigned long flags;

	spin_lock_irqsave (&wedo->io_lock, flags);
	if (!wedo->io_halt)
		wedo->update_output = true;
	spin_unlock_irqrestore (&wedo->io_lock, flags);
}

static int wedo_hub_set_mode(void *context, u8 mode)
{
	/* We only have one mode, so nothing to do */
	return 0;
}

static int wedo_hub_send_command(void *context, u8 command)
{
	struct usb_wedo *wedo = context;

	switch (command) {
	case WEDO_HUB_CMD_OUTPUT_OFF:
		wedo->output_bits |= WEDO_HUB_CTL_BIT_SHUTDOWN;
		break;
	case WEDO_HUB_CMD_OUTPUT_ON:
		wedo->output_bits &= ~WEDO_HUB_CTL_BIT_SHUTDOWN;
		break;
	case WEDO_HUB_CMD_CLEAR_ERROR:
		wedo->output_bits |= WEDO_HUB_CTL_BIT_ERROR;
		break;
	default:
		return -EINVAL;
	}

	wedo_hub_request_output_update(wedo->usb_interface);

	return 0;
}

static void wedo_in_callback(struct urb *urb)
{
	struct usb_wedo *wedo = urb->context;
	int status = urb->status;
	struct lego_sensor_device *hub = &wedo->wedo_hub;
	struct wedo_port_data *wpd1 = wedo->wedo_ports[WEDO_PORT_1];
	struct wedo_port_data *wpd2 = wedo->wedo_ports[WEDO_PORT_2];
	struct lego_sensor_mode_info *mode_info = &hub->mode_info[hub->mode];
	u16 *hub_raw_data = (u16 *)mode_info->raw_data;
	ktime_t *hub_last_changed = &mode_info->last_changed_time;
	unsigned long flags;

	if (status) {
		dev_dbg(&wedo->usb_device->dev, "%s: nonzero status received: %d\n",
			__func__, status);
		goto err_in_urb;
	}

	/*
	 * No need to lock access to the input URB results - the wedo_hub_class
	 * is the only reader, and each byte of data is atomic.
	 */
	if (urb->actual_length == 8) {
		u16 new_data[2];

		if (wedo->in_buf[0] & WEDO_HUB_CTL_BIT_ECHO)
			wedo->output_bits &= ~WEDO_HUB_CTL_BIT_ECHO;
		else
			wedo->output_bits |= WEDO_HUB_CTL_BIT_ECHO;

		new_data[0] = wedo->in_buf[0];
		/* multiplying by 49 scales the raw value to millivolts */
		new_data[1] = wedo->in_buf[1] * 49;

		if (memcmp(hub_raw_data, new_data, 4) != 0) {
			*hub_last_changed = ktime_get();
			memcpy(hub_raw_data, new_data, 4);
		}

		wpd1->input	= wedo->in_buf[2];
		wpd1->id	= wedo->in_buf[3];
		/* WEDO_HUB_CTL_BIT_ERROR indicates that outputs are turned off */
		if (!wpd1->output || (wedo->in_buf[0] & WEDO_HUB_CTL_BIT_ERROR))
			wedo_port_update_status(wpd1);
		wpd2->input	= wedo->in_buf[4];
		wpd2->id	= wedo->in_buf[5];
		/* WEDO_HUB_CTL_BIT_ERROR indicates that outputs are turned off */
		if (!wpd2->output || (wedo->in_buf[0] & WEDO_HUB_CTL_BIT_ERROR))
			wedo_port_update_status(wpd2);
	}

	if (wedo->status_debounce < WEDO_STATUS_DEBOUNCE) {
		wedo->status_debounce++;
	} else if (wedo->status_debounce == WEDO_STATUS_DEBOUNCE) {

		spin_lock_irqsave (&wedo->io_lock, flags);
		/*
		 * Some of the hub output control bits need to be reset
		 * after an output attempt
		 */
		wedo->output_bits &= ~WEDO_HUB_CTL_BIT_ERROR;
		wedo->output_bits &= ~WEDO_HUB_CTL_BIT_RESET;

		wedo->status_debounce++;

		spin_unlock_irqrestore (&wedo->io_lock, flags);
	}

	if (wedo->update_output && !wedo->output_pending) {

		spin_lock_irqsave(&wedo->io_lock, flags);

		wedo->ctl_buf[0] = wedo->output_bits;
		wedo->ctl_buf[1] = wpd1->output;
		wedo->ctl_buf[2] = wpd2->output;
		wedo->ctl_buf[3] = 0x00;
		wedo->ctl_buf[4] = 0x00;
		wedo->ctl_buf[5] = 0x00;
		wedo->ctl_buf[6] = 0x00;
		wedo->ctl_buf[7] = 0x00;

		wedo->update_output = false;
		wedo->output_pending = true;

		spin_unlock_irqrestore(&wedo->io_lock, flags);

		usb_submit_urb(wedo->ctl_urb, GFP_ATOMIC);
	}

err_in_urb:
	usb_submit_urb(urb, GFP_ATOMIC);
}


static void wedo_ctrl_callback(struct urb *urb)
{
	struct usb_wedo *wedo = urb->context;
	int status = urb->status;

	if (status) {
		dev_dbg(&wedo->usb_device->dev,
			"%s: nonzero ctl status received: %d\n",
			__func__, status);
		goto err_ctrl_urb;
	}

	wedo->status_debounce = 0;
	wedo->output_pending = false;

	return;

err_ctrl_urb:
	usb_submit_urb(urb, GFP_ATOMIC);
	return;
}

static int wedo_probe(struct usb_interface *interface,
		      const struct usb_device_id *id)
{
	struct usb_wedo *wedo;
	struct usb_endpoint_descriptor *endpoint;
	int ret = -ENOMEM;

	/* allocate memory for our device state and initialize it */
	wedo = kzalloc(sizeof(*wedo), GFP_KERNEL);
	if (!wedo) {
		dev_err(&interface->dev, "Cannot allocate a usb_wedo device\n");
		goto err_alloc_usb_wedo;
	}

	wedo->usb_device = usb_get_dev(interface_to_usbdev(interface));
	wedo->usb_interface = interface;

	wedo->cr.bRequestType = USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT;
	wedo->cr.bRequest = HID_REQ_SET_REPORT;
	wedo->cr.wValue = cpu_to_le16(0x0200);
	wedo->cr.wIndex = cpu_to_le16(0x00);
	wedo->cr.wLength = cpu_to_le16(0x08);

	wedo->ctl_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!wedo->ctl_urb)
		goto err_alloc_ctl_urb;

	wedo->ctl_buf = usb_alloc_coherent(wedo->usb_device, 8, GFP_KERNEL,
					   &wedo->ctl_dma);
	if (!wedo->ctl_buf)
		goto err_alloc_ctl_buf;

	usb_fill_control_urb(wedo->ctl_urb, wedo->usb_device,
			     usb_sndctrlpipe(wedo->usb_device, 0),
			     (void *)&wedo->cr, wedo->ctl_buf, 8,
			     wedo_ctrl_callback, wedo);

	wedo->ctl_urb->transfer_dma = wedo->ctl_dma;
	wedo->ctl_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* We send data to the LEGO WEDO on Endpoint 0 (ctl) and receive data
	 * from endpoint 1 (in) - there's no sense looping through the endpoints
	 * but it makes sense to check that the first endpoint is an interrupt
	 * type, and that it's an input
	 *
	 * The control endpoint is available on every USB device, the first
	 * enumerated endpoint on the interface (array index 0) is endpoint 1
	 */

	endpoint = &interface->cur_altsetting->endpoint[0].desc;

	if (!usb_endpoint_xfer_int(endpoint) && !usb_endpoint_dir_in(endpoint))
		goto err_no_int_in_endpoint;

	wedo->in_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!wedo->in_urb)
		goto err_alloc_in_urb;

	wedo->in_buf = usb_alloc_coherent(wedo->usb_device, 8, GFP_KERNEL,
					  &wedo->in_dma);
	if (!wedo->in_buf)
		goto err_alloc_in_buf;

	usb_fill_int_urb(wedo->in_urb, wedo->usb_device,
			 usb_rcvintpipe(wedo->usb_device,
					endpoint->bEndpointAddress),
			 wedo->in_buf, 8, wedo_in_callback, wedo, 32 );

	wedo->in_urb->transfer_dma = wedo->in_dma;
	wedo->in_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, wedo);

	/*
	 * Make sure that we have have enough power for motors. If we are not
	 * high-power, then we can still use sensors.
	 */
	if (wedo->usb_device->bus_mA >= 500)
		wedo->output_bits = WEDO_HUB_CTL_BIT_HIGH_POWER;

	spin_lock_init(&wedo->io_lock);

	memcpy(&wedo->wedo_hub_info, wedo_hub_sensor_defs,
	       sizeof (struct wedo_hub_sensor_info));
	wedo->wedo_hub.name = "wedo-hub";
	snprintf(wedo->address, LEGO_NAME_SIZE, "usb%s",
		 dev_name(&interface->dev));
	wedo->wedo_hub.address = wedo->address;
	wedo->wedo_hub.num_modes = NUM_WEDO_HUB_MODES;
	wedo->wedo_hub.num_view_modes = NUM_WEDO_HUB_MODES;
	wedo->wedo_hub.mode_info = wedo->wedo_hub_info.mode_info;
	wedo->wedo_hub.num_commands = NUM_WEDO_HUB_CMDS;
	wedo->wedo_hub.cmd_info = wedo->wedo_hub_info.cmd_info;
	wedo->wedo_hub.set_mode = wedo_hub_set_mode;
	wedo->wedo_hub.send_command = wedo_hub_send_command;
	wedo->wedo_hub.context = wedo;

	ret = register_lego_sensor(&wedo->wedo_hub, &interface->dev);
	if (ret) {
		dev_err(&interface->dev,
			"Failed to register the WeDo hub sensor interface. (%d)\n",
			ret);
		goto err_register_wedo_hub;
	}

	wedo->wedo_ports[WEDO_PORT_1] = register_wedo_port(interface, WEDO_PORT_1);
	if (IS_ERR(wedo->wedo_ports[WEDO_PORT_1])) {
		ret = PTR_ERR(wedo->wedo_ports[WEDO_PORT_1]);
		dev_err(&interface->dev,
			"Failed to register WeDo port 1. (%d)\n", ret);
		goto err_register_wedo_port_1;
	}
	wedo->wedo_ports[WEDO_PORT_2] = register_wedo_port(interface, WEDO_PORT_2);
	if (IS_ERR(wedo->wedo_ports[WEDO_PORT_2])) {
		ret = PTR_ERR(wedo->wedo_ports[WEDO_PORT_2]);
		dev_err(&interface->dev,
			"Failed to register WeDo port 2. (%d)\n", ret);
		goto err_register_wedo_port_2;
	}

	ret = usb_submit_urb(wedo->in_urb, GFP_ATOMIC);

	return 0;

err_register_wedo_port_2:
	unregister_wedo_port(wedo->wedo_ports[WEDO_PORT_1]);
err_register_wedo_port_1:
	unregister_lego_sensor(&wedo->wedo_hub);
err_register_wedo_hub:
	usb_free_coherent (wedo->usb_device, 8, wedo->in_buf, wedo->in_dma);
err_alloc_in_buf:
	usb_free_urb (wedo->in_urb);
err_alloc_in_urb:
err_no_int_in_endpoint:
	usb_free_coherent (wedo->usb_device, 8, wedo->ctl_buf, wedo->ctl_dma);
err_alloc_ctl_buf:
	usb_free_urb (wedo->ctl_urb);
err_alloc_ctl_urb:
	usb_put_dev (wedo->usb_device);
	kfree (wedo);
err_alloc_usb_wedo:
	return ret;
}

static void wedo_disconnect(struct usb_interface *interface)
{
	struct usb_wedo *wedo;
	unsigned long flags;

	wedo = usb_get_intfdata(interface);

	spin_lock_irqsave (&wedo->io_lock, flags);
	wedo->io_halt = true;
	spin_unlock_irqrestore (&wedo->io_lock, flags);

	do {
	} while (wedo->update_output || wedo->output_pending);

	unregister_wedo_port(wedo->wedo_ports[WEDO_PORT_2]);
	unregister_wedo_port(wedo->wedo_ports[WEDO_PORT_1]);
	unregister_lego_sensor(&wedo->wedo_hub);

	usb_set_intfdata(interface, NULL);

	wedo->usb_interface = NULL;

	usb_free_coherent (wedo->usb_device, 8, wedo->in_buf, wedo->in_dma);
	usb_free_urb (wedo->in_urb);
	usb_free_coherent (wedo->usb_device, 8, wedo->ctl_buf, wedo->ctl_dma);
	usb_free_urb (wedo->ctl_urb);
	usb_put_dev (wedo->usb_device);
	kfree (wedo);
}

static void wedo_draw_down(struct usb_wedo *dev)
{
}

static int wedo_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_wedo *dev = usb_get_intfdata(intf);

	if (!dev)
		return 0;
	wedo_draw_down(dev);
	return 0;
}

static int wedo_resume(struct usb_interface *intf)
{
	return 0;
}

static struct usb_driver wedo_driver = {
	.name =		"wedo",
	.probe =	wedo_probe,
	.disconnect =	wedo_disconnect,
	.suspend =	wedo_suspend,
	.resume =	wedo_resume,
	.id_table =	wedo_table,
	.supports_autosuspend = 1,
};

static int __init usb_wedo_init(void)
{
	return usb_register(&wedo_driver);
}

static void __exit usb_wedo_exit(void)
{
	usb_deregister (&wedo_driver);
}

module_init(usb_wedo_init);
module_exit(usb_wedo_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE(DRIVER_LICENSE);
