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

#define FRAME_SIZE_MS 10

#define POLL_TIMEOUT FRAME_SIZE_MS
#define REQ_TIMEOUT 1000
#define FPGA_DELAY_UNIT_US 10000
#define FPGA_SANITY_MAX_RECHECK 200

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

static void deinterleave_buffer(const uint8_t *src, size_t length,
				uint32_t *dst_ptr, size_t channel_count,
				uint32_t channel_mask)
{
	uint32_t sample;

	for (const uint64_t *src_ptr = (uint64_t *)src;
	     src_ptr < (uint64_t *)(src + length); src_ptr += channel_count) {
		for (int bit = 0; bit != 64; bit++) {
			const uint64_t *word_ptr = src_ptr;
			sample = 0;
			for (unsigned int channel = 0; channel != 32;
			     channel++) {
				const uint32_t m = channel_mask >> channel;
				if (!m)
					break;
				if ((m & 1) &&
				    ((*word_ptr++ >> bit) & UINT64_C(1)))
					sample |= 1 << channel;
			}
			*dst_ptr++ = sample;
		}
	}
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

	*value = RL32(&trx[12]);
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
	uint32_t max, typical;

	max = speed == LIBUSB_SPEED_SUPER ? MAX_FRAME_SIZE_SS :
					    MAX_FRAME_SIZE_HS;

	typical = (samplerate * nchannels) / 8 / (1000 / FRAME_SIZE_MS);

	return MIN(max, typical) / 4096 / nchannels * 4096 * nchannels;
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

static void LIBUSB_CALL samples_fetched(struct libusb_transfer *xfer)
{
	const struct sr_dev_inst *sdi;
	sdi = xfer->user_data;

	/* TODO */
}

static int fetch_samples(const struct sr_dev_inst *sdi)
{
	/* TODO */

	std_session_send_df_header(sdi);

	return SR_OK;
}

static int stop_fetch_samples(const struct sr_dev_inst *sdi) {
	/* TODO cancel USB transfer. */
	std_session_send_df_end(sdi);
}

static int handle_events(int fd, int revents, void *cb_data)
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

	memset(&tv, 0, sizeof(tv));
	libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx, &tv,
					       NULL);

	/* Continue to wait for trigger condition if trigger hasn't been fired
	   yet. */
	if (!devc->triggered) {
		ep0_get_trigger_status(usb->devhdl, &status);
		if (status.pos_real != 0) {
			sr_info("triggered after acquiring 0x%" PRIx64
				"samples, point at 0x%" PRIx32 ", int trigger"
				"status 0x%08" PRIx32,
				status.sample_offset, status.pos_real,
				status.activated);
			devc->triggered = TRUE;
			devc->trigger_point_real = status.pos_real;
			fetch_samples(sdi);
		}
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
	//TRY_WRITE_REG(sdi, REG_BLOCK_START, 0);
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

	usb_source_add(sdi->session, sdi->session->ctx, POLL_TIMEOUT,
		       handle_events, (void *)sdi);

	TRY_WRITE_REG(sdi, REG_STOP, 0);

	/* TODO error check */
}

SR_PRIV int px_logic_acquisition_stop(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	int res;

	devc = sdi->priv;

	/* TODO error check */
	stop_fetch_samples(sdi);
	usb_source_remove(sdi->session, sdi->session->ctx);

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
