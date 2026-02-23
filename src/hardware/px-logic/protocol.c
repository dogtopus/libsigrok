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
#include <strings.h>
#include "protocol.h"

#define FRAME_SIZE_MS 10

#define POLL_TIMEOUT FRAME_SIZE_MS
#define REQ_TIMEOUT 1000
#define FPGA_DELAY_UNIT_US 10000
#define FPGA_SANITY_MAX_RECHECK 200

#define NUM_SIMUL_XFERS 32

#define FPGA_VCCIO (3.334)
#define FPGA_F_PWM_VREF SR_MHZ(120)
#define FPGA_F_PWM SR_MHZ(125)
#define FPGA_PWM_VREF_PERIOD SR_KHZ(10)
#define FPGA_INPUT_VDIV (1.0 / 2.0)

/* 4MiB */
#define MAX_FRAME_SIZE_SS (4 * 1024 * 1024)
/* 4.8Mbit */
#define MAX_FRAME_SIZE_HS (4800000 / 8)
#define MAX_TRIG_PERCENT 90

#define STRIPE_SIZE_BYTES sizeof(uint64_t)
#define STRIPE_SIZE_BITS STRIPE_SIZE_BYTES * 8

#define REQ_PACKET_LEN 0x08
#define REQ_KEY_READ 0xfefe0001
#define REQ_KEY_WRITE 0xfefe0000
#define REQ_WRITE_ACK 0xfefefefe

#define EP_REG 0x01
#define EP_FIFO_SAMPLE 0x02
#define EP_FIFO_FWRAM 0x03

#define EP0_CMD_GET_TRIGGER_STATUS 0xb0

#define REG_MODE 0x0000
#define REG_PWM_VREF_CMP_PERIOD 0x0004
#define REG_PWM_VREF_CMP_DUTY 0x0008
#define REG_CHANNEL_EN 0x0010
#define REG_CLK_CONF 0x0014
#define REG_CLK_DIV 0x0018
#define REG_SAMPLE_FRAME_SIZE 0x001c
#define REG_STOP 0x0020
#define REG_TRIG_LOW 0x0024
#define REG_TRIG_HIGH 0x0028
#define REG_TRIG_RISING 0x002c
#define REG_TRIG_FALLING 0x0030
#define REG_TRIG_EXT_MODE 0x003c
#define REG_PWM0_CONF 0x0040
#define REG_PWM0_CMP_PERIOD 0x0044
#define REG_PWM0_CMP_DUTY 0x0048
#define REG_PWM1_CONF 0x004c
#define REG_PWM1_CMP_PERIOD 0x0050
#define REG_PWM1_CMP_DUTY 0x0054
#define REG_TRIG_OUT_EN 0x0058

#define REG_XFER_FRAME_SIZE 0x2008
#define REG_FWRAM_READ_START 0x200c
#define REG_FWRAM_READ_END 0x2010
#define REG_FWRAM_READ_PAGE 0x2014
#define REG_FWRAM_WRITE_START 0x2018
#define REG_FWRAM_WRITE_END 0x201c
#define REG_FWRAM_WRITE_PAGE 0x2020
#define REG_NUM_SAMPLES_LO 0x2024
#define REG_NUM_SAMPLES_HI 0x2028
#define REG_BLOCK_START 0x202c
#define REG_MCU_FW_VERSION 0x2034
#define REG_ENABLED_NUM_CH 0x204c
#define REG_TRIG_POINT 0x2050
#define REG_TRIG_POINT_REAL 0x2054
#define REG_DEV_VARIANT 0x2058

#define FWRAM_MCU_PROG_FLASH 0
#define FWRAM_FPGA_CFGRAM 4

#define MODE_MASK_INIT (1 << 0)
#define MODE_MASK_STREAMING (1 << 1)
#define MODE_MASK_INIT2 (1 << 2)
#define MODE_MASK_FILTER_EN (1 << 3)
#define MODE_MASK_UNK_4 (1 << 4)

#define PWM_CONF_MASK_EN (1 << 0)

#define ALIGN_4K(x) ((x / 4096 + 1) * 4096)

#define MCU_FW_NAME "SCI_LOGIC.bin"
#define FPGA_STAGE1_NAME "hspi_ddr_RST.bin"
#define FPGA_STAGE2_NAME "hspi_ddr.bin"

struct trigger_status {
	uint64_t sample_offset;
	uint32_t activated;
	uint32_t pos_real;
};

static const uint64_t clk_conf_table[CLK_NUM_SUPPORTED] = {
	[CLK_1GHZ] = SR_GHZ(1),	    [CLK_500MHZ] = SR_MHZ(500),
	[CLK_250MHZ] = SR_MHZ(250), [CLK_125MHZ] = SR_MHZ(125),
	[CLK_800MHZ] = SR_MHZ(800), [CLK_400MHZ] = SR_MHZ(400),
	[CLK_200MHZ] = SR_MHZ(200), [CLK_100MHZ] = SR_MHZ(100),
};

static int ep0_get_trigger_status(libusb_device_handle *devhdl,
				  struct trigger_status *status)
{
	int ret;

	ret = libusb_control_transfer(
		devhdl, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
		EP0_CMD_GET_TRIGGER_STATUS, 0x0000, 0x0000,
		(unsigned char *)status, sizeof(struct trigger_status),
		POLL_TIMEOUT / 2);

	if (ret < 0) {
		sr_err("Unable to get trigger status: %s.",
		       libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int convert_trigger(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_trigger *trigger;
	struct sr_trigger_stage *stage;
	struct sr_trigger_match *match;
	const GSList *l, *m;
	uint32_t mask;
	uint32_t trigger_point, max_trigger_percent;
	uint64_t depth_per_channel;

	devc = sdi->priv;

	trigger = sr_session_trigger_get(sdi->session);

	if (trigger == NULL) {
		memset(&devc->trigger, 0, sizeof(devc->trigger));
		return SR_OK;
	}

	if (devc->channels.n == 0)
		depth_per_channel = 0;
	else
		depth_per_channel =
			(devc->config.buffer_depth / devc->channels.n) &
			0xfffffc00;

	max_trigger_percent = devc->streaming ? 10 : MAX_TRIG_PERCENT;

	trigger_point = MAX(STRIPE_SIZE_BYTES,
			    devc->capture_ratio / 100 * devc->limit_samples);
	trigger_point = MIN(depth_per_channel * max_trigger_percent / 100,
			    trigger_point);

	devc->trigger.point = trigger_point;

	for (l = trigger->stages; l; l = l->next) {
		stage = l->data;
		for (m = stage->matches; m; m = m->next) {
			match = m->data;
			if (!match->channel->enabled ||
			    match->channel->type != SR_CHANNEL_LOGIC)
				continue;

			mask = 1 << (match->channel->index);

			switch (match->match) {
			case SR_TRIGGER_ONE:
				devc->trigger.high_mask |= mask;
				break;
			case SR_TRIGGER_ZERO:
				devc->trigger.low_mask |= mask;
				break;
			case SR_TRIGGER_RISING:
				devc->trigger.rising_mask |= mask;
				break;
			case SR_TRIGGER_FALLING:
				devc->trigger.falling_mask |= mask;
				break;
			case SR_TRIGGER_EDGE:
				devc->trigger.rising_mask |= mask;
				devc->trigger.falling_mask |= mask;
				break;
			}
		}
	}

	return SR_OK;
}

static inline void write_bit_le(uint8_t *dest, uint8_t bitpos, int bit)
{
	const size_t offset = bitpos / 8;
	const size_t mask = 1 << (bitpos % 8);

	if (bit)
		dest[offset] |= mask;
	else
		dest[offset] &= ~mask;
}

/**
 * Transpose the DSLogic-style samples (striped channels) to sigrok-style
 * (bitfield samples padded to bytes).
 *
 * Converts the samples from format of
 *
 * `aaa...bbb...ccc...ddd...`
 *
 * to
 *
 * `abcd...abcd...abcd...`
 * 
 * @param src Samples in striped channels format.
 *
 * @param length Total length of the samples in bytes. Must be aligned to 64
 *               samples.
 *
 * @param dst_ptr Samples in bitfield format.
 *
 * @param channel_count Number of active channels. Must be consistent with
 *                      channel_mask.
 *
 * @param channel_mask Active channels. Must be consistent with channel_count.
 *
 * @param sample_width_bytes Sample width in bytes (2 or 4 depending on
 *                           variant).
 */
static size_t cap_transpose_samples(const uint8_t *src, size_t length,
				    uint8_t *dst_ptr, size_t channel_count,
				    uint32_t channel_mask,
				    size_t sample_width_bytes)
{
	const uint8_t *const end_ptr = src + length;
	const size_t stripes_step = channel_count * STRIPE_SIZE_BYTES;
	const size_t sample_width_bits = sample_width_bytes * 8;

	const uint8_t *src_ptr, *stripe_ptr;
	uint8_t *out_ptr;
	uint8_t s_pos, channel;
	uint64_t stripe;
	size_t out_samples;

	out_ptr = dst_ptr;
	out_samples = 0;

	/* Process one 64-sample group of all channels at a time. */
	for (src_ptr = src; src_ptr < end_ptr; src_ptr += stripes_step) {
		stripe_ptr = src_ptr;
		/* TODO: use ctz + bit clear here may be better. */
		for (channel = 0; channel < sample_width_bits; channel++) {
			/* This stripe does not belong to this bit in the
			   output sample. */
			if (!(channel_mask & (1 << channel))) {
				continue;
			}

			/* Device endian. */
			stripe = RL64(stripe_ptr);

			/* Write the stripe as a column on the output sample
			 * matrix. */
			for (s_pos = 0; s_pos < STRIPE_SIZE_BITS; s_pos++) {
				write_bit_le(
					&out_ptr[sample_width_bytes * s_pos],
					channel, !!(stripe & (1 << s_pos)));
			}

			stripe_ptr += STRIPE_SIZE_BYTES;
		}

		/* 64 converted samples, each 2 or 4 bytes in size. */
		out_ptr += STRIPE_SIZE_BITS * sample_width_bytes;
		out_samples += STRIPE_SIZE_BITS;
	}

	sr_spew("Transposed %zu samples", out_samples);
	return out_samples;
}

/**
 * Do a roundtrip transaction over the control register access endpoint.
 * 
 * @param[in] sdi Device context.
 * @param[in] tx_buf Transmit buffer.
 * @param[in] tx_len Amount of bytes to transmit.
 * @param[out] rx_buf Receive buffer.
 * @param[in] rx_len Amount of bytes to receive.
 *
 * @retval SR_OK Success.
 * @retval SR_ERR_IO Failed to transmit or receive data.
 * @retval SR_ERR_TIMEOUT Timeout.
 *
 * The transmit buffer can overlap with the receive buffer. This allows one to
 * reuse the same buffer as `tx_buf` and `rx_buf`.
 */
static int reg_ep_trx(const struct sr_dev_inst *sdi, uint8_t *tx_buf,
		      uint16_t tx_len, uint8_t *rx_buf, uint16_t rx_len)
{
	struct sr_usb_dev_inst *usb;
	int xfer_result, xfer_count;

	usb = sdi->conn;

	xfer_result = libusb_bulk_transfer(usb->devhdl,
					   LIBUSB_ENDPOINT_OUT | EP_REG, tx_buf,
					   tx_len, &xfer_count, REQ_TIMEOUT);

	if (xfer_result != LIBUSB_SUCCESS)
		sr_err("Failed to transmit data to EP_REG: %s.",
		       libusb_error_name(xfer_result));

	if (xfer_result == LIBUSB_ERROR_TIMEOUT) {
		return SR_ERR_TIMEOUT;
	} else if (xfer_result != LIBUSB_SUCCESS) {
		return SR_ERR_IO;
	}

	if (xfer_count != tx_len) {
		sr_err("Incomplete data transmitted to EP_REG: expecting %dB, actually sent %dB.",
		       tx_len, xfer_count);
		return SR_ERR_IO;
	}

	xfer_result = libusb_bulk_transfer(usb->devhdl,
					   LIBUSB_ENDPOINT_IN | EP_REG, rx_buf,
					   rx_len, &xfer_count, REQ_TIMEOUT);

	if (xfer_result != LIBUSB_SUCCESS)
		sr_err("Failed to receive data from EP_REG: %s.",
		       libusb_error_name(xfer_result));

	if (xfer_result == LIBUSB_ERROR_TIMEOUT)
		return SR_ERR_TIMEOUT;
	else if (xfer_result != LIBUSB_SUCCESS)
		return SR_ERR_IO;

	if (xfer_count != rx_len) {
		sr_err("Incomplete data received from EP_REG: expecting %dB, actually got %dB.",
		       tx_len, xfer_count);
		return SR_ERR_IO;
	}

	return SR_OK;
}

/**
 * Transmit data to the firmware RAM FIFO endpoint.
 * 
 * @param[in] sdi Device context.
 * @param[in] tx_buf Transmit buffer.
 * @param[in] tx_len Amount of bytes to transmit. If 0, clears the STALL
 * condition on the endpoint and immediately return.
 *
 * @retval SR_OK Success.
 * @retval SR_ERR_IO Failed to transmit or receive data.
 * @retval SR_ERR_TIMEOUT Timeout.
 */
static int fwram_ep_tx(const struct sr_dev_inst *sdi, uint8_t *tx_buf,
		       uint32_t tx_len)
{
	struct sr_usb_dev_inst *usb;
	int xfer_result, xfer_count;

	usb = sdi->conn;

	if (tx_len == 0) {
		libusb_clear_halt(usb->devhdl,
				  LIBUSB_ENDPOINT_OUT | EP_FIFO_FWRAM);
		return SR_OK;
	}

	xfer_result = libusb_bulk_transfer(usb->devhdl,
					   LIBUSB_ENDPOINT_OUT | EP_FIFO_FWRAM,
					   tx_buf, tx_len, &xfer_count,
					   REQ_TIMEOUT);

	if (xfer_result != LIBUSB_SUCCESS)
		sr_err("Failed to transmit data to EP_FIFO_FWRAM: %s.",
		       libusb_error_name(xfer_result));

	if (xfer_result == LIBUSB_ERROR_TIMEOUT)
		return SR_ERR_TIMEOUT;
	else if (xfer_result != LIBUSB_SUCCESS)
		return SR_ERR_IO;

	if ((uint32_t)(xfer_count & 0x7fffffff) != tx_len) {
		sr_err("Incomplete data transmitted to EP_FIFO_FWRAM: expecting %dB, actually sent %dB.",
		       tx_len, xfer_count);
		return SR_ERR_IO;
	}

	return SR_OK;
}

/**
 * Receive data from the firmware RAM FIFO endpoint.
 * 
 * @param[in] sdi Device context.
 * @param[out] rx_buf Receive buffer.
 * @param[in] rx_len Amount of bytes to receive. If 0, clears the STALL
 * condition on the endpoint and immediately return.
 *
 * @retval SR_OK Success.
 * @retval SR_ERR_IO Failed to transmit or receive data.
 * @retval SR_ERR_TIMEOUT Timeout.
 */
static int fwram_ep_rx(const struct sr_dev_inst *sdi, uint8_t *rx_buf,
		       uint32_t rx_len)
{
	struct sr_usb_dev_inst *usb;
	int xfer_result, xfer_count;

	usb = sdi->conn;

	if (rx_len == 0) {
		libusb_clear_halt(usb->devhdl,
				  LIBUSB_ENDPOINT_IN | EP_FIFO_FWRAM);
		return SR_OK;
	}

	xfer_result = libusb_bulk_transfer(usb->devhdl,
					   LIBUSB_ENDPOINT_IN | EP_FIFO_FWRAM,
					   rx_buf, rx_len, &xfer_count,
					   REQ_TIMEOUT);

	if (xfer_result != LIBUSB_SUCCESS)
		sr_err("Failed to receive data from EP_FIFO_FWRAM: %s.",
		       libusb_error_name(xfer_result));

	if (xfer_result == LIBUSB_ERROR_TIMEOUT)
		return SR_ERR_TIMEOUT;
	else if (xfer_result != LIBUSB_SUCCESS)
		return SR_ERR_IO;

	if ((uint32_t)(xfer_count & 0x7fffffff) != rx_len) {
		sr_err("Incomplete data received from EP_FIFO_FWRAM: expecting %dB, actually sent %dB.",
		       rx_len, xfer_count);
		return SR_ERR_IO;
	}

	return SR_OK;
}

/**
 * Read control register.
 * 
 * @param[in] sdi Device context.
 * @param[in] address Address.
 * @param[out] value Value.
 *
 * @retval SR_OK Success.
 * @retval SR_ERR_IO reg_ep_trx() fails.
 */
static int read_reg(const struct sr_dev_inst *sdi, uint32_t address,
		    uint32_t *value)
{
	uint8_t trx[16];
	uint32_t val;
	int res;

	WL32(trx, REQ_KEY_READ);
	WL32(&trx[4], REQ_PACKET_LEN);
	WL32(&trx[8], address);
	WL32(&trx[12], 0);

	res = reg_ep_trx(sdi, trx, sizeof(trx), trx, sizeof(trx));
	if (res != SR_OK) {
		sr_err("Failed to read register at 0x%x.", address);
		return res;
	}

	val = RL32(&trx[12]);
	sr_spew("reg 0x%04x => 0x%08x", address, val);
	*value = val;

	return SR_OK;
}

/** Call read_reg() and bubble up the result code if there's an error. */
#define TRY_READ_REG(sdi, address, value)            \
	{                                            \
		int res;                             \
		res = read_reg(sdi, address, value); \
		if (res != SR_OK)                    \
			return res;                  \
	}

/**
 * Write control register.
 * 
 * @param[in] sdi Device context.
 * @param[in] address Address.
 * @param[in] value Value.
 *
 * @retval SR_OK Success.
 * @retval SR_ERR_IO reg_ep_trx() fails.
 * @retval SR_ERR_DATA Device returns invalid response.
 */
static int write_reg(const struct sr_dev_inst *sdi, uint32_t address,
		     uint32_t value)
{
	uint8_t trx[16];
	int res;

	sr_spew("reg 0x%04x <= 0x%08x", address, value);

	WL32(trx, REQ_KEY_WRITE);
	WL32(&trx[4], REQ_PACKET_LEN);
	WL32(&trx[8], address);
	WL32(&trx[12], value);

	res = reg_ep_trx(sdi, trx, sizeof(trx), trx, sizeof(trx));
	if (res != SR_OK) {
		sr_err("Failed to write 0x%08x to register at 0x%x.", value,
		       address);
		return res;
	}

	/* Check response code. */
	if (RL32(&trx[12]) != REQ_WRITE_ACK) {
		sr_err("Device did not acknowledge write 0x%08x to register at"
		       " 0x%x.",
		       value, address);
		return SR_ERR_DATA;
	}

	return SR_OK;
}

/** Call write_reg() and bubble up the result code if there's an error. */
#define TRY_WRITE_REG(sdi, address, value)            \
	{                                             \
		int res;                              \
		res = write_reg(sdi, address, value); \
		if (res != SR_OK)                     \
			return res;                   \
	}

static int upload_bitstream_to_fpga(const struct sr_dev_inst *sdi,
				    const char *bitstream_name)
{
	struct sr_resource bitstream;
	struct drv_context *drvc;
	int res;
	uint8_t *fw;
	gsize fw_size_page_aligned;

	drvc = sdi->driver->context;

	sr_info("Uploading FPGA bitstream file '%s'", bitstream_name);

	res = sr_resource_open(drvc->sr_ctx, &bitstream, SR_RESOURCE_FIRMWARE,
			       bitstream_name);
	if (res != SR_OK) {
		return res;
	}

	fw_size_page_aligned = ALIGN_4K(bitstream.size);

	TRY_WRITE_REG(sdi, REG_FWRAM_WRITE_START, 0)
	TRY_WRITE_REG(sdi, REG_FWRAM_WRITE_END, fw_size_page_aligned)
	TRY_WRITE_REG(sdi, REG_FWRAM_WRITE_PAGE, FWRAM_FPGA_CFGRAM)

	fw = g_try_malloc0(fw_size_page_aligned);
	if (fw == NULL)
		return SR_ERR_MALLOC;

	res = sr_resource_read(drvc->sr_ctx, &bitstream, fw, bitstream.size);
	if (res <= 0) {
		sr_err("Failed to read firmware file.");
		sr_resource_close(drvc->sr_ctx, &bitstream);
		g_free(fw);
		return res;
	}

	sr_resource_close(drvc->sr_ctx, &bitstream);

	fwram_ep_tx(sdi, NULL, 0);
	res = fwram_ep_tx(sdi, fw, fw_size_page_aligned);

	g_free(fw);
	fw = NULL;

	if (res != SR_OK)
		return res;

	return SR_OK;
}

static gboolean fpga_reg_sanity_check(const struct sr_dev_inst *sdi)
{
	int res;
	uint32_t reg;

	reg = 0xdeadbeef;

	res = read_reg(sdi, REG_CLK_CONF, &reg);
	if (res != SR_OK)
		return FALSE;

	if (reg >= CLK_NUM_SUPPORTED)
		return FALSE;

	res = read_reg(sdi, REG_CLK_DIV, &reg);
	if (res != SR_OK)
		return FALSE;

	/* We only support down to 1MHz so this never goes above 99. */
	if (reg > 99)
		return FALSE;

	res = read_reg(sdi, REG_STOP, &reg);
	if (res != SR_OK)
		return FALSE;

	/* STOP should be 0 on normal operation unless the configuration has
	 * been interrupted (for now). */
	if (reg != 0x00000000)
		return FALSE;

	return TRUE;
}

static int set_vref(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	uint32_t period, duty;

	devc = sdi->priv;

	period = FPGA_F_PWM_VREF / FPGA_PWM_VREF_PERIOD;
	duty = ((devc->voltage_threshold * FPGA_INPUT_VDIV) / FPGA_VCCIO) *
	       period;

	TRY_WRITE_REG(sdi, REG_PWM_VREF_CMP_PERIOD, period);
	TRY_WRITE_REG(sdi, REG_PWM_VREF_CMP_DUTY, duty);

	return SR_OK;
}

static struct channel_config
compute_channel_config(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_channel *channel;
	GSList *l;
	struct channel_config res;

	devc = sdi->priv;
	res.n = 0;
	res.mask = 0;

	for (l = devc->cg_logic->channels; l; l = l->next) {
		channel = l->data;
		if (channel->enabled) {
			res.n++;
			res.mask |= 1 << (channel->index);
		}
	}

	return res;
}

static uint32_t compute_frame_size(enum libusb_speed speed, uint64_t samplerate,
				   uint32_t nchannels)
{
	uint32_t max, typical, final;

	max = speed == LIBUSB_SPEED_SUPER ? MAX_FRAME_SIZE_SS :
					    MAX_FRAME_SIZE_HS;

	typical = (samplerate * nchannels) / 8 / (1000 / FRAME_SIZE_MS);

	final = MIN(max, typical) / 4096 / nchannels * 4096 * nchannels;
	sr_spew("Computed frame size is %u bytes", final);

	return final;
}

static int config_sampler_clock(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	uint32_t clk_conf, clk_div;
	gboolean found;

	devc = sdi->priv;

	if (devc->samplerate < SR_MHZ(100)) {
		clk_conf = CLK_100MHZ;
		clk_div = SR_MHZ(100) / devc->samplerate - 1;
		if (SR_MHZ(100) / (clk_div + 1) != devc->samplerate) {
			sr_err("Cannot determine divider value from samplerate"
			       "%" PRIu64 ".",
			       devc->samplerate);
			return SR_ERR_ARG;
		}
	} else {
		clk_div = 0;
		found = FALSE;
		for (clk_conf = 0; clk_conf < ARRAY_SIZE(clk_conf_table);
		     clk_conf++) {
			if (clk_conf_table[clk_conf] == devc->samplerate) {
				found = TRUE;
				break;
			}
		}
		if (!found) {
			sr_err("Cannot determine clock config from samplerate"
			       "%" PRIu64 ".",
			       devc->samplerate);
			return SR_ERR_ARG;
		}
	}

	TRY_WRITE_REG(sdi, REG_CLK_CONF, clk_conf);
	TRY_WRITE_REG(sdi, REG_CLK_DIV, clk_div);

	return SR_OK;
}

static int cap_data_init(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	devc = sdi->priv;

	devc->cap.xpose_buffer_size = devc->frame_size / devc->channels.n * 8 *
				      devc->config.sample_width;
	sr_spew("%s: Allocating %zu bytes for transpose buffer", __func__,
		devc->cap.xpose_buffer_size);
	devc->cap.xpose_buffer = g_try_malloc0(devc->cap.xpose_buffer_size);
	if (devc->cap.xpose_buffer == NULL) {
		return SR_ERR_MALLOC;
	}

	return SR_OK;
}

static void cap_data_fini(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	devc = sdi->priv;

	g_free(devc->cap.xpose_buffer);
	devc->cap.xpose_buffer = NULL;
	devc->cap.xpose_buffer_size = 0;
}

static void cap_data_send(const struct sr_dev_inst *sdi, const uint8_t *data,
			  size_t length)
{
	struct dev_context *devc;
	size_t samples;
	devc = sdi->priv;

	samples = cap_transpose_samples(data, length, devc->cap.xpose_buffer,
					devc->channels.n, devc->channels.mask,
					devc->config.sample_width);

	const struct sr_datafeed_logic logic = {
		.length = samples * devc->config.sample_width,
		.unitsize = devc->config.sample_width,
		.data = devc->cap.xpose_buffer
	};

	const struct sr_datafeed_packet packet = { .type = SR_DF_LOGIC,
						   .payload = &logic };

	sr_session_send(sdi, &packet);
}

/**
 * Shortcut to resubmit an individual transfer. Must be called at a control
 * flow tail.
 */
static void cap_sample_xfer_resubmit(struct libusb_transfer *xfer,
				     struct dev_context *devc)
{
	int res;

	res = libusb_submit_transfer(xfer);
	if (res != LIBUSB_SUCCESS) {
		sr_err("Failed to resubmit transfer: %s. Aborting "
		       "session.",
		       libusb_error_name(res));
		devc->cap.state = CAP_STATE_HALT;
	}
}

static void LIBUSB_CALL cap_sample_xfer_event(struct libusb_transfer *xfer)
{
	const struct sr_dev_inst *sdi;
	struct dev_context *devc;
	uint64_t bytes_received;

	sdi = xfer->user_data;
	devc = sdi->priv;

	switch (xfer->status) {
	case LIBUSB_TRANSFER_CANCELLED:
		sr_spew("Transfer cancelled");
		devc->cap.n_active_data_xfers--;
		break;
	case LIBUSB_TRANSFER_NO_DEVICE:
	case LIBUSB_TRANSFER_OVERFLOW:
	case LIBUSB_TRANSFER_ERROR:
	case LIBUSB_TRANSFER_STALL:
		sr_err("Unrecoverable status %d. Aborting session.",
		       xfer->status);
		devc->cap.n_active_data_xfers--;
		devc->cap.state = CAP_STATE_HALT;
		break;
	case LIBUSB_TRANSFER_TIMED_OUT:
	case LIBUSB_TRANSFER_COMPLETED:
		sr_spew("got %d bytes from sample FIFO", xfer->actual_length);

		if (devc->cap.state != CAP_STATE_SAMPLE_XFER) {
			/* Ignore data and wait for cancellation if not
			 * receiving samples. */
			devc->cap.n_active_data_xfers--;
			break;
		}

		if (xfer->actual_length == 0) {
			/* Timeout or we got 0 bytes. Make a note about this. */
			if (devc->cap.timeout_counter >= 10) {
				sr_err("Device stopped responding. Aborting "
				       "session.");
				devc->cap.state = CAP_STATE_HALT;
				break;
			}
			cap_sample_xfer_resubmit(xfer, devc);
			devc->cap.timeout_counter++;
			break;
		}

		devc->cap.timeout_counter = 0;

		bytes_received = devc->cap.bytes_received + xfer->actual_length;

		cap_data_send(sdi, xfer->buffer, xfer->actual_length);

		devc->cap.bytes_received = bytes_received;

		if (devc->cap.bytes_received / devc->channels.n * 8 >=
		    devc->limit_samples) {
			sr_info("Got enough samples. Transferring capture "
				"state to HALT.");
			devc->cap.n_active_data_xfers--;
			devc->cap.state = CAP_STATE_HALT;
			break;
		}

		cap_sample_xfer_resubmit(xfer, devc);
		break;
	}
}

// static int cap_trigger_submit(const struct sr_dev_inst *sdi);

// static void LIBUSB_CALL cap_trigger_event_handler(struct libusb_transfer *xfer)
// {
// 	const struct sr_dev_inst *sdi;
// 	struct dev_context *devc;
// 	struct trigger_status *tr;
// 	int res;

// 	sdi = xfer->user_data;
// 	devc = sdi->priv;

// 	switch (xfer->status) {
// 	case LIBUSB_TRANSFER_COMPLETED:
// 		tr = (struct trigger_status *)xfer->buffer;
// 		if (tr->pos_real == 0) {
// 			res = cap_trigger_submit(sdi);
// 			if (res != SR_OK) {
// 				devc->cap.wft_done = TRUE;
// 				devc->cap.state = CAP_STATE_HALT;
// 			}
// 		}

// 		break;
// 	case LIBUSB_TRANSFER_CANCELLED:
// 		devc->cap.wft_done = TRUE;
// 		break;
// 	default:
// 		sr_err("Error waiting for trigger. Aborting session.");
// 		devc->cap.wft_done = TRUE;
// 		devc->cap.state = CAP_STATE_HALT;
// 		break;
// 	}
// }

/**
 * Deletes the sample transfer pool only. Should terminate all transfers
 * before calling this.
 */
static void cap_sample_xfer_fini(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	int i;

	devc = sdi->priv;

	if (devc->cap.data_xfers == NULL) {
		devc->cap.state = CAP_STATE_INIT;
		return;
	}

	for (i = 0; i < NUM_SIMUL_XFERS; i++) {
		g_free(devc->cap.data_xfers[i]->buffer);
		libusb_free_transfer(devc->cap.data_xfers[i]);
		devc->cap.data_xfers[i] = NULL;
	}

	g_free(devc->cap.data_xfers);
	devc->cap.data_xfers = NULL;
	devc->cap.state = CAP_STATE_INIT;
}

/**
 * Allocates the sample transfer pool.
 */
static int cap_sample_xfer_init(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	unsigned char *xfer_buf;
	int i;

	devc = sdi->priv;
	usb = sdi->conn;

	devc->cap.data_xfers = g_try_malloc0(sizeof(struct libusb_transfer *) *
					     NUM_SIMUL_XFERS);
	if (devc->cap.data_xfers == NULL) {
		return SR_ERR_MALLOC;
	}

	for (i = 0; i < NUM_SIMUL_XFERS; i++) {
		xfer_buf = g_try_malloc0(devc->frame_size);
		if (xfer_buf == NULL) {
			cap_sample_xfer_fini(sdi);
			return SR_ERR_MALLOC;
		}
		devc->cap.data_xfers[i] = libusb_alloc_transfer(0);
		if (devc->cap.data_xfers[i] == NULL) {
			g_free(xfer_buf);
			cap_sample_xfer_fini(sdi);
			return SR_ERR_MALLOC;
		}
		libusb_fill_bulk_transfer(devc->cap.data_xfers[i], usb->devhdl,
					  LIBUSB_ENDPOINT_IN | EP_FIFO_SAMPLE,
					  xfer_buf, devc->frame_size,
					  &cap_sample_xfer_event, (void *)sdi,
					  FRAME_SIZE_MS * 1.5);
	}

	return SR_OK;
}

/**
 * Fires all the transfers the sample transfer pool.
 */
static int cap_sample_xfer_begin(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	int i, ret;

	devc = sdi->priv;
	devc->cap.timeout_counter = 0;

	for (i = 0; i < NUM_SIMUL_XFERS; i++) {
		ret = libusb_submit_transfer(devc->cap.data_xfers[i]);
		if (ret != LIBUSB_SUCCESS) {
			return SR_ERR;
		}
		devc->cap.n_active_data_xfers++;
	}

	return SR_OK;
}

/**
 * Signal all the transfers to terminate themselves.
 */
static void cap_sample_xfer_end(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	uint32_t i;

	devc = sdi->priv;

	if (devc->cap.data_xfers == NULL)
		return;

	for (i = 0; i < NUM_SIMUL_XFERS; i++) {
		struct libusb_transfer *xfer = devc->cap.data_xfers[i];
		if (xfer != NULL) {
			libusb_cancel_transfer(xfer);
		}
	}
}

// static int cap_trigger_submit(const struct sr_dev_inst *sdi)
// {
// 	struct dev_context *devc;
// 	struct sr_usb_dev_inst *usb;
// 	unsigned char *ctrl_buf;
// 	int res;

// 	devc = sdi->priv;
// 	usb = sdi->conn;

// 	sr_spew("Waiting for trigger...");
// 	devc->cap.wft_done = FALSE;

// 	if (devc->cap.wft_xfer == NULL) {
// 		devc->cap.wft_xfer = libusb_alloc_transfer(0);
// 		if (devc->cap.wft_xfer == NULL) {
// 			return SR_ERR_MALLOC;
// 		}
// 		ctrl_buf = g_malloc0(sizeof(struct trigger_status));
// 	} else
// 		ctrl_buf = devc->cap.wft_xfer->buffer;

// 	libusb_fill_control_setup(
// 		ctrl_buf, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
// 		EP0_CMD_GET_TRIGGER_STATUS, 0x0000, 0x0000,
// 		sizeof(struct trigger_status));
// 	libusb_fill_control_transfer(devc->cap.wft_xfer, usb->devhdl, ctrl_buf,
// 				     &cap_trigger_event_handler, (void *)sdi,
// 				     POLL_TIMEOUT);

// 	res = libusb_submit_transfer(devc->cap.wft_xfer);
// 	if (res != LIBUSB_SUCCESS) {
// 		sr_err("Failed to submit trigger control transfer: %s",
// 		       libusb_error_name(res));
// 		return SR_ERR_IO;
// 	}

// 	devc->cap.state = CAP_STATE_WAIT_TRIGGER;
// 	return SR_OK;
// }

// static void cap_trigger_cleanup(const struct sr_dev_inst *sdi)
// {
// 	struct dev_context *devc;

// 	devc = sdi->priv;

// 	if (devc->cap.wft_xfer == NULL)
// 		return;

// 	g_free(devc->cap.wft_xfer->buffer);
// 	libusb_free_transfer(devc->cap.wft_xfer);
// 	devc->cap.wft_xfer = NULL;
// }

static int cap_top_event_handler(int fd, int revents, void *cb_data)
{
	const struct sr_dev_inst *sdi;
	struct drv_context *drvc;
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc;
	struct trigger_status status;
	struct timeval tv;

	(void)fd;
	(void)revents;

	/* TODO: Terminate on sanity check failures? */
	sdi = cb_data;
	if (!sdi)
		return TRUE;

	drvc = sdi->driver->context;
	devc = sdi->priv;
	usb = sdi->conn;
	if (!devc || !drvc || !usb)
		return TRUE;

	/* libusb event handler. */
	memset(&tv, 0, sizeof(tv));
	libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx, &tv,
					       NULL);

	switch (devc->cap.state) {
	case CAP_STATE_WAIT_TRIGGER:
		/* Continue to wait for trigger condition if trigger hasn't
		   been fired yet. */
		ep0_get_trigger_status(usb->devhdl, &status);
		sr_spew("sample tick %" PRIu64, status.sample_offset);
		if (status.pos_real != 0) {
			sr_info("triggered after acquiring 0x%" PRIx64
				" samples, point at 0x%" PRIx32 ", int trigger"
				" status 0x%08" PRIx32 ". Transferring capture"
				" state to SAMPLE_XFER",
				status.sample_offset, status.pos_real,
				status.activated);
			devc->cap.state = CAP_STATE_SAMPLE_XFER;
			devc->cap.trigger_point_real = status.pos_real;
			std_session_send_df_header(sdi);
			cap_sample_xfer_begin(sdi);
		}
		break;
	case CAP_STATE_HALT:
		cap_sample_xfer_end(sdi);
		sr_info("%s: Transferring capture state to CLEANUP.", __func__);
		devc->cap.state = CAP_STATE_CLEANUP;
		break;
	case CAP_STATE_CLEANUP:
		/* Make sure all the transfers stop before proceeding. */
		if (devc->cap.n_active_data_xfers == 0) {
			cap_sample_xfer_fini(sdi);
			/* Get rid of the trigger transfer object as well. */
			//cap_trigger_cleanup(sdi);
			cap_data_fini(sdi);
			/* Safe to terminate the event loop. */
			std_session_send_df_end(sdi);
			usb_source_remove(sdi->session, sdi->session->ctx);
			devc->cap.state = CAP_STATE_INIT;
		}
		break;
	default:
		/* Do nothing. */
		break;
	}

	return TRUE;
}

SR_PRIV enum device_variant px_logic_get_variant(const struct sr_dev_inst *sdi)
{
	uint32_t variant;
	int result;

	result = read_reg(sdi, REG_DEV_VARIANT, &variant);
	if (result != SR_OK)
		return VARIANT_UNKNOWN;

	if (variant >= VARIANT_MAX) {
		sr_warn("%s, Unknown device variant %d.", __func__, variant);
		return VARIANT_UNKNOWN;
	}

	return variant;
}

SR_PRIV int px_logic_fpga_ensure_init(const struct sr_dev_inst *sdi)
{
	int res;
	int fail_count;

	if (fpga_reg_sanity_check(sdi)) {
		sr_info("FPGA register config is sane. Skip FPGA initialization.");
		return SR_OK;
	}

	sr_info("Initializing FPGA...");

	res = upload_bitstream_to_fpga(sdi, FPGA_STAGE1_NAME);
	if (res != SR_OK)
		return res;

	res = upload_bitstream_to_fpga(sdi, FPGA_STAGE2_NAME);
	if (res != SR_OK)
		return res;

	res = SR_ERR_TIMEOUT;
	/* Wait until FPGA is fully configured. */
	for (fail_count = 0; fail_count < FPGA_SANITY_MAX_RECHECK;
	     fail_count++) {
		if (fpga_reg_sanity_check(sdi)) {
			res = SR_OK;
			break;
		}
		g_usleep(FPGA_DELAY_UNIT_US);
	}

	if (res == SR_ERR_TIMEOUT)
		sr_err("FPGA initialization never completes.");

	return res;
}

SR_PRIV int px_logic_dev_open(const struct sr_dev_inst *sdi)
{
	struct sr_dev_driver *di;

	di = sdi->driver;

	libusb_device **devlist;
	struct sr_usb_dev_inst *usb;
	struct libusb_device_descriptor des;
	struct dev_context *devc;
	struct drv_context *drvc;
	int ret = SR_ERR, i, device_count;
	uint32_t fw_version;
	char connection_id[64];

	drvc = di->context;
	devc = sdi->priv;
	usb = sdi->conn;
	fw_version = 0;

	device_count =
		libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (device_count < 0) {
		sr_err("Failed to get device list: %s.",
		       libusb_error_name(device_count));
		return SR_ERR;
	}

	for (i = 0; i < device_count; i++) {
		libusb_get_device_descriptor(devlist[i], &des);

		if (des.idVendor != devc->config.vid ||
		    des.idProduct != devc->config.pid)
			continue;

		if ((sdi->status == SR_ST_INITIALIZING) ||
		    (sdi->status == SR_ST_INACTIVE)) {
			/* Check device by its physical USB bus/port address. */
			if (usb_get_port_path(devlist[i], connection_id,
					      sizeof(connection_id)) < 0)
				continue;

			if (strcmp(sdi->connection_id, connection_id))
				/* This is not the one. */
				continue;
		}

		if (!(ret = libusb_open(devlist[i], &usb->devhdl))) {
			if (usb->address == 0xff)
				/*
				 * First time we touch this device after FW
				 * upload, so we don't know the address yet.
				 */
				usb->address =
					libusb_get_device_address(devlist[i]);
		} else {
			sr_err("Failed to open device: %s.",
			       libusb_error_name(ret));
			ret = SR_ERR;
			break;
		}

		/* Check version */
		ret = read_reg(sdi, REG_MCU_FW_VERSION, &fw_version);
		if (ret != SR_OK)
			break;

		sr_info("MCU Version: 0x%08x", fw_version);

		ret = SR_OK;

		break;
	}

	libusb_free_device_list(devlist, 1);

	return ret;
}

SR_PRIV int px_logic_receive_config(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	int res;
	uint32_t pwm_vref_period, pwm_vref_duty, clk_conf, clk_div, mode,
		num_samples_lo, num_samples_hi;
	double pwm_vref_freq, io_vref;
	uint64_t samplerate;

	devc = sdi->priv;

	pwm_vref_period = 0;
	pwm_vref_duty = 0;
	clk_conf = 0;
	clk_div = 0;
	num_samples_lo = 0;
	num_samples_hi = 0;
	mode = 0;

	TRY_READ_REG(sdi, REG_PWM_VREF_CMP_PERIOD, &pwm_vref_period);
	TRY_READ_REG(sdi, REG_PWM_VREF_CMP_PERIOD, &pwm_vref_duty);
	TRY_READ_REG(sdi, REG_CLK_CONF, &clk_conf);
	TRY_READ_REG(sdi, REG_CLK_DIV, &clk_div);
	TRY_READ_REG(sdi, REG_NUM_SAMPLES_LO, &num_samples_lo);
	TRY_READ_REG(sdi, REG_NUM_SAMPLES_HI, &num_samples_hi);
	TRY_READ_REG(sdi, REG_MODE, &mode);

	pwm_vref_freq = (double)FPGA_F_PWM_VREF / pwm_vref_period;
	io_vref = (double)pwm_vref_duty / pwm_vref_freq * FPGA_VCCIO /
		  FPGA_INPUT_VDIV;

	sr_info("VREF from device is %fV, switching freq is %fHz", io_vref,
		pwm_vref_freq);

	if (io_vref < VREF_MIN || io_vref > VREF_MAX) {
		sr_info("VREF exceeds our threshold, reset to 2V.");
		io_vref = VREF_DEFAULT;
	}
	devc->voltage_threshold = io_vref;

	sr_info("Clock configuration on device: CLK_CONF = 0x%08x, "
		"CLK_DIV = 0x%08x",
		clk_conf, clk_div);
	if (clk_div != 0 && clk_conf != CLK_100MHZ) {
		sr_info("Custom sampling clock configuration is not supported "
			"yet. Falling back to a safe default.");
		samplerate = SR_MHZ(125);
	} else {
		switch (clk_conf) {
		case CLK_1GHZ:
			samplerate = SR_GHZ(1);
			break;
		case CLK_800MHZ:
			samplerate = SR_MHZ(800);
			break;
		case CLK_500MHZ:
			samplerate = SR_MHZ(500);
			break;
		case CLK_400MHZ:
			samplerate = SR_MHZ(400);
			break;
		case CLK_250MHZ:
			samplerate = SR_MHZ(250);
			break;
		case CLK_200MHZ:
			samplerate = SR_MHZ(200);
			break;
		case CLK_125MHZ:
			samplerate = SR_MHZ(125);
			break;
		case CLK_100MHZ:
		default:
			switch (clk_div) {
			case 99:
				samplerate = SR_MHZ(1);
				break;
			case 49:
				samplerate = SR_MHZ(2);
				break;
			case 24:
				samplerate = SR_MHZ(4);
				break;
			case 19:
				samplerate = SR_MHZ(5);
				break;
			case 9:
				samplerate = SR_MHZ(10);
				break;
			case 4:
				samplerate = SR_MHZ(20);
				break;
			case 3:
				samplerate = SR_MHZ(25);
				break;
			case 1:
				samplerate = SR_MHZ(50);
				break;
			case 0:
				samplerate = SR_MHZ(100);
				break;
			default:
				sr_info("Irregular divider is not supported"
					"yet. Falling back to a safe default.");
				samplerate = SR_MHZ(125);
				break;
			}
		}
	}
	devc->samplerate = samplerate;

	devc->limit_samples = ((uint64_t)num_samples_lo) |
			      ((uint64_t)num_samples_hi << 32);

	sr_info("Mode configuration on device: 0x%08x", mode);
	devc->streaming = mode & MODE_MASK_STREAMING;
	devc->filter = mode & MODE_MASK_FILTER_EN;

	return SR_OK;
}

SR_PRIV int px_logic_send_config(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	int ret;
	uint32_t mode_reg;

	devc = sdi->priv;

	devc->channels = compute_channel_config(sdi);
	devc->frame_size = compute_frame_size(
		devc->config.speed, devc->samplerate, devc->channels.n);

	/* Disable PWM channels as we are not supporting them for now. */
	TRY_WRITE_REG(sdi, REG_PWM0_CONF, 0);
	TRY_WRITE_REG(sdi, REG_PWM0_CMP_PERIOD, 0);
	TRY_WRITE_REG(sdi, REG_PWM0_CMP_DUTY, 0);

	TRY_WRITE_REG(sdi, REG_PWM1_CONF, 0);
	TRY_WRITE_REG(sdi, REG_PWM1_CMP_PERIOD, 0);
	TRY_WRITE_REG(sdi, REG_PWM1_CMP_DUTY, 0);

	/* Clear the BLOCK_START register. */
	TRY_WRITE_REG(sdi, REG_BLOCK_START, 0);

	/* Set input reference voltage. */
	ret = set_vref(sdi);
	if (ret != SR_OK) {
		return ret;
	}

	TRY_WRITE_REG(sdi, REG_CHANNEL_EN, 0);

	/* Set mode register */
	mode_reg = MODE_MASK_INIT | MODE_MASK_INIT2 |
		   (devc->streaming ? MODE_MASK_STREAMING : 0);

	TRY_WRITE_REG(sdi, REG_MODE, mode_reg);
	TRY_WRITE_REG(sdi, REG_MODE, mode_reg | MODE_MASK_UNK_4);
	TRY_WRITE_REG(sdi, REG_MODE, mode_reg);

	TRY_WRITE_REG(sdi, REG_STOP, 0xffffffff);

	TRY_WRITE_REG(sdi, REG_SAMPLE_FRAME_SIZE, devc->frame_size);
	TRY_WRITE_REG(sdi, REG_XFER_FRAME_SIZE, devc->frame_size);

	TRY_WRITE_REG(sdi, REG_NUM_SAMPLES_LO,
		      devc->limit_samples & 0xffffffff);
	TRY_WRITE_REG(sdi, REG_NUM_SAMPLES_HI, devc->limit_samples >> 32);

	TRY_WRITE_REG(sdi, REG_TRIG_EXT_MODE, 0);
	TRY_WRITE_REG(sdi, REG_TRIG_OUT_EN, 0);

	ret = config_sampler_clock(sdi);
	if (ret != SR_OK) {
		return ret;
	}

	TRY_WRITE_REG(sdi, REG_ENABLED_NUM_CH, devc->channels.n);
	TRY_WRITE_REG(sdi, REG_BLOCK_START, 0);
	ret = convert_trigger(sdi);
	if (ret != SR_OK) {
		return ret;
	}

	TRY_WRITE_REG(sdi, REG_TRIG_POINT, devc->trigger.point);
	TRY_WRITE_REG(sdi, REG_TRIG_LOW, devc->trigger.low_mask);
	TRY_WRITE_REG(sdi, REG_TRIG_HIGH, devc->trigger.high_mask);
	TRY_WRITE_REG(sdi, REG_TRIG_RISING, devc->trigger.rising_mask);
	TRY_WRITE_REG(sdi, REG_TRIG_FALLING, devc->trigger.falling_mask);

	TRY_WRITE_REG(sdi, REG_CHANNEL_EN, devc->channels.mask);

	mode_reg &= ~(MODE_MASK_INIT | MODE_MASK_INIT2);

	TRY_WRITE_REG(sdi, REG_MODE,
		      mode_reg | (devc->filter ? MODE_MASK_FILTER_EN : 0));

	return SR_OK;
}

SR_PRIV int px_logic_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	int res;

	devc = sdi->priv;

	res = px_logic_send_config(sdi);
	if (res != SR_OK) {
		return res;
	}

	res = cap_sample_xfer_init(sdi);
	if (res != SR_OK) {
		return res;
	}

	res = cap_data_init(sdi);
	if (res != SR_OK) {
		cap_sample_xfer_fini(sdi);
		return res;
	}

	res = write_reg(sdi, REG_STOP, 0);
	if (res != SR_OK) {
		cap_data_fini(sdi);
		cap_sample_xfer_fini(sdi);
		return res;
	}

	usb_source_add(sdi->session, sdi->session->ctx, POLL_TIMEOUT,
		       &cap_top_event_handler, (void *)sdi);

	sr_info("%s: Transferring capture state to WAIT_TRIGGER.", __func__);
	devc->cap.state = CAP_STATE_WAIT_TRIGGER;

	return SR_OK;
}

SR_PRIV int px_logic_acquisition_stop(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	sr_info("%s: Transferring capture state to HALT.", __func__);

	devc = sdi->priv;
	devc->cap.state = CAP_STATE_HALT;

	return SR_OK;
}

SR_PRIV int px_logic_receive_data(int fd, int revents, void *cb_data)
{
	const struct sr_dev_inst *sdi;
	struct dev_context *devc;

	(void)fd;

	sdi = cb_data;
	if (!sdi)
		return TRUE;

	devc = sdi->priv;
	if (!devc)
		return TRUE;

	if (revents == G_IO_IN) {
		/* TODO */
	}

	return TRUE;
}
