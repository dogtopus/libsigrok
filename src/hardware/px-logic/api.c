/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2026 dogtopus <dogtopus@users.noreply.github.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include "protocol.h"

#define PXLOGIC_VID 0x16c0
#define PXLOGIC_PID 0x05dc

#define PXLOGIC_OLD_VID 0x1a86
#define PXLOGIC_OLD_PID 0x5237

#define USB_INTERFACE_MAIN 0

static const char *variant_names[] = {
	[VARIANT_32] = "Logic 32",
	[VARIANT_16_PRO] = "Logic 16 Pro",
	[VARIANT_16_PLUS] = "Logic 16 Plus",
	[VARIANT_16_BASE] = "Logic 16 Base",
};

static int variant_logic_channels[] = {
	[VARIANT_32] = 32,
	[VARIANT_16_PRO] = 16,
	[VARIANT_16_PLUS] = 16,
	[VARIANT_16_BASE] = 16,
};

static struct sr_dev_driver px_logic_driver_info;

static inline gboolean check_vid_pid(struct libusb_device_descriptor *des)
{
	return (des->idVendor != PXLOGIC_VID ||
		des->idProduct != PXLOGIC_PID) &&
	       (des->idVendor != PXLOGIC_OLD_VID ||
		des->idProduct != PXLOGIC_OLD_PID);
}

static gboolean in_conn_devices(GSList *conn_devices, libusb_device *dev)
{
	GSList *l;
	struct sr_usb_dev_inst *usb;
	usb = NULL;
	for (l = conn_devices; l; l = l->next) {
		usb = l->data;
		if (usb->bus == libusb_get_bus_number(dev) &&
		    usb->address == libusb_get_device_address(dev))
			return TRUE;
	}
	/* This device matched none of the ones that matched the conn
	   specification. */
	return FALSE;
}

static gboolean check_descriptor(libusb_device *dev)
{
	struct libusb_device_descriptor des;
	struct libusb_device_handle *hdl;
	gboolean ret;
	unsigned char strdesc[64];

	hdl = NULL;
	ret = FALSE;
	while (!ret) {
		/* Assume strings won't match, unless proven wrong. */
		libusb_get_device_descriptor(dev, &des);

		if (libusb_open(dev, &hdl) != 0)
			break;

		if (libusb_get_string_descriptor_ascii(hdl, des.iManufacturer,
						       strdesc,
						       sizeof(strdesc)) < 0)
			break;
		if (strcmp((const char *)strdesc, "PX"))
			break;

		/* If we made it here, the strings must match. */
		ret = TRUE;
	}
	if (hdl)
		libusb_close(hdl);

	return ret;
}

/**
 * Detect the device variant and populate channels, etc.
 * 
 * @param sdi Half-initialized device instance.
 * @param dev libusb device context.
 * @retval SR_OK Device variant was determined and capability set.
 * @retval SR_ERR Caller should abort further instance creation and ignore this device.
 */
static int detect_device_variant(struct sr_dev_inst *sdi, libusb_device *dev)
{
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc;
	struct sr_channel *ch;
	struct sr_channel_group *cg;
	int result_call, result;
	int i, ch_offset;
	enum device_variant variant;
	gboolean claimed;
	char name[8];

	usb = sdi->conn;
	devc = sdi->priv;

	/* Make a temporary connection to send register read request. */
	result_call = libusb_open(dev, &usb->devhdl);

	if (result_call != LIBUSB_SUCCESS) {
		sr_err("%s: Failed to open device for detection: %s.", __func__,
		       libusb_error_name(result_call));
		return SR_ERR;
	}

	result = SR_ERR;
	claimed = FALSE;

	result_call = libusb_claim_interface(usb->devhdl, USB_INTERFACE_MAIN);
	if (result_call == LIBUSB_SUCCESS) {
		claimed = TRUE;
		variant = px_logic_get_variant(sdi);

		/* Change the variant. */
		devc->variant = variant;
		if (variant == VARIANT_UNKNOWN) {
			// result = SR_OK;
			goto done;
		}
		if (sdi->model != NULL) {
			g_free(sdi->model);
		}
		sdi->model = g_strdup(variant_names[variant]);

		ch_offset = 0;

		/* Add logic channels based on variant. */
		cg = sr_channel_group_new(sdi, "Logic", NULL);
		devc->cg_logic = cg;

		for (i = 0; i < variant_logic_channels[variant]; i++) {
			g_snprintf(name, sizeof(name) - 1, "CH%d", i);
			name[sizeof(name) - 1] = '\0';
			ch = sr_channel_new(sdi, ch_offset, SR_CHANNEL_LOGIC,
					    TRUE, name);
			cg->channels = g_slist_append(cg->channels, ch);
			ch_offset++;
		}

		/* Add PWM channels. */
		cg = sr_channel_group_new(sdi, "PWM", NULL);
		devc->cg_pwm = cg;

		ch = sr_channel_new(sdi, ch_offset, SR_CHANNEL_ANALOG, FALSE,
				    "PWM0");
		cg->channels = g_slist_append(cg->channels, ch);
		ch_offset++;

		/* Change device state to INACTIVE to mark that it's ready
		   for initialization. Device with unknown variant won't have
		   this transition. */
		sdi->status = SR_ST_INACTIVE;

		result = SR_OK;
	} else {
		sr_err("%s: Failed to claim the main interface: %s.", __func__,
		       libusb_error_name(result_call));
	}

done:
	if (claimed) {
		libusb_release_interface(usb->devhdl, USB_INTERFACE_MAIN);
	}
	libusb_close(usb->devhdl);
	usb->devhdl = NULL;

	return result;
}

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc;
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct sr_config *src;
	GSList *l, *devices, *conn_devices;
	struct libusb_device_descriptor des;
	libusb_device **devlist;
	unsigned int i;
	const char *conn;
	char connection_id[64];
	int res;

	devices = NULL;
	drvc = di->context;
	devc = NULL;
	drvc->instances = NULL;

	/* Check for manually set connection strings in device config. */
	conn = NULL;
	for (l = options; l; l = l->next) {
		src = l->data;
		switch (src->key) {
		case SR_CONF_CONN:
			conn = g_variant_get_string(src->data, NULL);
			break;
		}
	}
	if (conn) {
		conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
	} else {
		conn_devices = NULL;
	}

	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);

	for (i = 0; devlist[i]; i++) {
		if (conn && !in_conn_devices(conn_devices, devlist[i])) {
			continue;
		}

		libusb_get_device_descriptor(devlist[i], &des);

		if (usb_get_port_path(devlist[i], connection_id,
				      sizeof(connection_id)) < 0) {
			continue;
		}

		if (check_vid_pid(&des)) {
			continue;
		}

		if (!check_descriptor(devlist[i])) {
			continue;
		}

		/* We now have pretty high confidence that this is a device
		   we can talk to. Try to create a device instance and get more
		   info from the device */
		sdi = g_malloc0(sizeof(struct sr_dev_inst));
		sdi->status = SR_ST_INITIALIZING;
		sdi->vendor = g_strdup("PX");
		sdi->model = g_strdup("Logic");
		sdi->connection_id = g_strdup(connection_id);
		sdi->inst_type = SR_INST_USB;
		sdi->conn = sr_usb_dev_inst_new(
			libusb_get_bus_number(devlist[i]),
			libusb_get_device_address(devlist[i]), NULL);
		devc = g_malloc0(sizeof(struct dev_context));
		sdi->priv = devc;

		res = detect_device_variant(sdi, devlist[i]);
		if (res != SR_OK) {
			g_free(devc);
			g_free(sdi);
			continue;
		}
		devices = g_slist_append(devices, sdi);
	}

	libusb_free_device_list(devlist, 1);
	g_slist_free_full(conn_devices, (GDestroyNotify)sr_usb_dev_inst_free);

	return std_scan_complete(di, devices);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	(void)sdi;

	/* TODO: get handle from sdi->conn and open it. */

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	(void)sdi;

	/* TODO: get handle from sdi->conn and close it. */

	return SR_OK;
}

static int config_get(uint32_t key, GVariant **data,
		      const struct sr_dev_inst *sdi,
		      const struct sr_channel_group *cg)
{
	int ret;

	(void)sdi;
	(void)data;
	(void)cg;

	ret = SR_OK;
	switch (key) {
	/* TODO */
	default:
		return SR_ERR_NA;
	}

	return ret;
}

static int config_set(uint32_t key, GVariant *data,
		      const struct sr_dev_inst *sdi,
		      const struct sr_channel_group *cg)
{
	int ret;

	(void)sdi;
	(void)data;
	(void)cg;

	ret = SR_OK;
	switch (key) {
	/* TODO */
	default:
		ret = SR_ERR_NA;
	}

	return ret;
}

static int config_list(uint32_t key, GVariant **data,
		       const struct sr_dev_inst *sdi,
		       const struct sr_channel_group *cg)
{
	int ret;

	(void)sdi;
	(void)data;
	(void)cg;

	ret = SR_OK;
	switch (key) {
	/* TODO */
	default:
		return SR_ERR_NA;
	}

	return ret;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
	/* TODO: configure hardware, reset acquisition state, set up
	 * callbacks and send header packet. */

	(void)sdi;

	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	/* TODO: stop acquisition. */

	(void)sdi;

	return SR_OK;
}

static struct sr_dev_driver px_logic_driver_info = {
	.name = "px-logic",
	.longname = "PX Logic",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = std_dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(px_logic_driver_info);
