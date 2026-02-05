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

#define VREF_MIN 0.1
#define VREF_MAX 6.0
#define VREF_STEP 0.1
#define VREF_DEFAULT 2.0

enum device_variant {
	VARIANT_UNKNOWN = -1,
	VARIANT_32,
	VARIANT_16_PRO,
	VARIANT_16_PLUS,
	VARIANT_16_BASE,
	VARIANT_MAX,
};

enum clk_config {
	CLK_1GHZ,
	CLK_500MHZ,
	CLK_250MHZ,
	CLK_125MHZ,
	CLK_800MHZ,
	CLK_400MHZ,
	CLK_200MHZ,
	CLK_100MHZ,
	CLK_NUM_SUPPORTED,
};

struct dev_config {
	uint16_t vid;
	uint16_t pid;

	enum device_variant variant;
};

struct pwm_config {
	gboolean enabled;
	double freq;
	double duty;
};

struct dev_context {
	struct dev_config config;

	gboolean streaming;
	gboolean filter;
	double voltage_threshold;
	uint64_t samplerate;
	uint64_t limit_samples;

	struct pwm_config pwm[1];

	struct sr_channel_group *cg_logic;
	struct sr_channel_group *cg_pwm;
	struct sr_channel_group *cg_ext_trig;
};

SR_PRIV enum device_variant px_logic_get_variant(const struct sr_dev_inst *sdi);
SR_PRIV int px_logic_dev_open(const struct sr_dev_inst *sdi);
SR_PRIV int px_logic_fpga_ensure_init(const struct sr_dev_inst *sdi);
SR_PRIV int px_logic_receive_config(const struct sr_dev_inst *sdi);
SR_PRIV int px_logic_receive_data(int fd, int revents, void *cb_data);

#endif
