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

#ifndef LIBSIGROK_HARDWARE_PX_LOGIC_PROTOCOL_H
#define LIBSIGROK_HARDWARE_PX_LOGIC_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "px-logic"

enum device_variant {
	VARIANT_UNKNOWN = -1,
	VARIANT_32,
	VARIANT_16_PRO,
	VARIANT_16_PLUS,
	VARIANT_16_BASE,
	VARIANT_MAX,
};

struct dev_context {
	enum device_variant variant;
	struct sr_channel_group *cg_logic;
	struct sr_channel_group *cg_pwm;
	struct sr_channel_group *cg_ext_trig;
};

SR_PRIV enum device_variant px_logic_get_variant(const struct sr_dev_inst *sdi);
SR_PRIV int px_logic_upload_fpga_firmware(const struct sr_dev_inst *sdi);
SR_PRIV int px_logic_receive_data(int fd, int revents, void *cb_data);

#endif
