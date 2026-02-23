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

enum cap_state {
	CAP_STATE_INIT,
	CAP_STATE_WAIT_TRIGGER,
	CAP_STATE_SAMPLE_XFER,
	CAP_STATE_HALT,
	CAP_STATE_CLEANUP,
};

struct dev_config {
	uint16_t vid;
	uint16_t pid;
	enum libusb_speed speed;

	enum device_variant variant;
	uint64_t buffer_depth;
	/** Number of total channels the device supports (16 or 32). */
	uint8_t channels;
	/** Sample width in bytes (2 or 4). */
	uint8_t sample_width;
};

struct pwm_config {
	gboolean enabled;
	double freq;
	double duty;
};

struct channel_config {
	uint32_t n;
	uint32_t mask;
};

struct trigger_config {
	uint32_t point;
	uint32_t high_mask;
	uint32_t low_mask;
	uint32_t rising_mask;
	uint32_t falling_mask;
};

struct capture_state {
	enum cap_state state;
	uint32_t trigger_point_real;

	//struct libusb_transfer *wft_xfer;
	struct libusb_transfer **data_xfers;
	uint8_t n_active_data_xfers;
	uint64_t bytes_received;

	struct sr_datafeed_logic logic;
	struct sr_datafeed_packet packet;
	uint8_t *xpose_buffer;
	size_t xpose_buffer_size;
	//gboolean wft_done;
};

struct dev_context {
	/* Hardware configuration. */
	struct dev_config config;

	/* Capture properties. */
	gboolean streaming;
	gboolean filter;
	double voltage_threshold;
	uint64_t samplerate;
	uint64_t limit_samples;
	uint64_t capture_ratio;

	struct trigger_config trigger;
	struct pwm_config pwm[1];

	/* Values derived from properties. */
	uint32_t frame_size;
	struct channel_config channels;

	struct sr_channel_group *cg_logic;
	struct sr_channel_group *cg_pwm;
	struct sr_channel_group *cg_ext_trig;

	struct capture_state cap;
};

SR_PRIV enum device_variant px_logic_get_variant(const struct sr_dev_inst *sdi);
SR_PRIV int px_logic_dev_open(const struct sr_dev_inst *sdi);
SR_PRIV int px_logic_fpga_ensure_init(const struct sr_dev_inst *sdi);
SR_PRIV int px_logic_receive_config(const struct sr_dev_inst *sdi);
SR_PRIV int px_logic_send_config(const struct sr_dev_inst *sdi);
SR_PRIV int px_logic_acquisition_start(const struct sr_dev_inst *sdi);
SR_PRIV int px_logic_acquisition_stop(const struct sr_dev_inst *sdi);
SR_PRIV int px_logic_receive_data(int fd, int revents, void *cb_data);

#endif
