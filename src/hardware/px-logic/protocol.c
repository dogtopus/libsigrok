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

#define REQ_TIMEOUT 1000
#define FPGA_DELAY_UNIT_US 10000
#define FPGA_SANITY_MAX_RECHECK 200

#define REQ_PACKET_LEN 0x08
#define REQ_KEY_READ 0xfefe0001
#define REQ_KEY_WRITE 0xfefe0000
#define REQ_WRITE_ACK 0xfefefefe

#define EP_IN 0x80
#define EP_OUT 0x00

#define EP_REG 0x01
#define EP_FIFO_SAMPLE 0x02
#define EP_FIFO_FWRAM 0x03

#define REG_CLK_CONF 0x0014
#define REG_CLK_DIV 0x0018
#define REG_STOP 0x0020

#define REG_FWRAM_READ_START 0x200c
#define REG_FWRAM_READ_END 0x2010
#define REG_FWRAM_READ_PAGE 0x2014
#define REG_FWRAM_WRITE_START 0x2018
#define REG_FWRAM_WRITE_END 0x201c
#define REG_FWRAM_WRITE_PAGE 0x2020
#define REG_DEV_VARIANT 0x2058

#define FWRAM_MCU_PROG_FLASH 0
#define FWRAM_FPGA_CFGRAM 4

#define ALIGN_4K(x) ((x / 4096 + 1) * 4096)

#define MCU_FW_NAME "SCI_LOGIC.bin"
#define FPGA_STAGE1_NAME "hspi_ddr_RST.bin"
#define FPGA_STAGE2_NAME "hspi_ddr.bin"

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

	xfer_result = libusb_bulk_transfer(usb->devhdl, EP_OUT | EP_REG, tx_buf,
					   tx_len, &xfer_count, REQ_TIMEOUT);
	if (xfer_result != LIBUSB_SUCCESS) {
		sr_err("Failed to transmit data to EP_REG: %s.",
		       libusb_error_name(xfer_result));
	}
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

	xfer_result = libusb_bulk_transfer(usb->devhdl, EP_IN | EP_REG, rx_buf,
					   rx_len, &xfer_count, REQ_TIMEOUT);
	if (xfer_result != LIBUSB_SUCCESS) {
		sr_err("Failed to receive data from EP_REG: %s.",
		       libusb_error_name(xfer_result));
	}
	if (xfer_result == LIBUSB_ERROR_TIMEOUT) {
		return SR_ERR_TIMEOUT;
	} else if (xfer_result != LIBUSB_SUCCESS) {
		return SR_ERR_IO;
	}
	if (xfer_count != tx_len) {
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
		libusb_clear_halt(usb->devhdl, EP_OUT | EP_FIFO_FWRAM);
		return SR_OK;
	}

	xfer_result = libusb_bulk_transfer(usb->devhdl, EP_OUT | EP_FIFO_FWRAM,
					   tx_buf, tx_len, &xfer_count,
					   REQ_TIMEOUT);
	if (xfer_result != LIBUSB_SUCCESS) {
		sr_err("Failed to transmit data to EP_FIFO_FWRAM: %s.",
		       libusb_error_name(xfer_result));
		return SR_ERR_IO;
	}
	if (xfer_result == LIBUSB_ERROR_TIMEOUT) {
		return SR_ERR_TIMEOUT;
	} else if (xfer_result != LIBUSB_SUCCESS) {
		return SR_ERR_IO;
	}
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
		libusb_clear_halt(usb->devhdl, EP_IN | EP_FIFO_FWRAM);
		return SR_OK;
	}

	xfer_result = libusb_bulk_transfer(usb->devhdl, EP_IN | EP_FIFO_FWRAM,
					   rx_buf, rx_len, &xfer_count,
					   REQ_TIMEOUT);
	if (xfer_result != LIBUSB_SUCCESS) {
		sr_err("Failed to receive data from EP_FIFO_FWRAM: %s.",
		       libusb_error_name(xfer_result));
		return SR_ERR_IO;
	}
	if (xfer_result == LIBUSB_ERROR_TIMEOUT) {
		return SR_ERR_TIMEOUT;
	} else if (xfer_result != LIBUSB_SUCCESS) {
		return SR_ERR_IO;
	}
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
		return res;
	}

	*value = RL32(&trx[12]);
	return SR_OK;
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
		return res;
	}

	/* Check response code. */
	if (RL32(&trx[12]) != REQ_WRITE_ACK) {
		sr_err("%s: Device did not properly acknowledge write request.",
		       __func__);
		return SR_ERR_DATA;
	}

	return SR_OK;
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

	res = write_reg(sdi, REG_FWRAM_WRITE_START, 0);
	if (res != SR_OK) {
		return res;
	}
	res = write_reg(sdi, REG_FWRAM_WRITE_END, fw_size_page_aligned);
	if (res != SR_OK) {
		return res;
	}
	res = write_reg(sdi, REG_FWRAM_WRITE_PAGE, FWRAM_FPGA_CFGRAM);
	if (res != SR_OK) {
		return res;
	}

	fw = g_try_malloc0(fw_size_page_aligned);
	if (fw == NULL) {
		return SR_ERR_MALLOC;
	}

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

	if (res != SR_OK) {
		return res;
	}

	return SR_OK;
}

static gboolean fpga_reg_sanity_check(const struct sr_dev_inst *sdi) {
	int res;
	uint32_t reg;

	reg = 0xdeadbeef;

	res = read_reg(sdi, REG_CLK_CONF, &reg);
	if (res != SR_OK) {
		return FALSE;
	}
	if (reg >= CLK_NUM_SUPPORTED) {
		return FALSE;
	}

	res = read_reg(sdi, REG_CLK_DIV, &reg);
	if (res != SR_OK) {
		return FALSE;
	}
	/* We only support down to 1MHz so this never goes above 99. */
	if (reg > 99) {
		return FALSE;
	}

	res = read_reg(sdi, REG_STOP, &reg);
	if (res != SR_OK) {
		return FALSE;
	}
	/* STOP should be 0 on normal operation unless the configuration has
	 * been interrupted (for now). */
	if (reg != 0x00000000) {
		return FALSE;
	}

	return TRUE;
}

SR_PRIV enum device_variant px_logic_get_variant(const struct sr_dev_inst *sdi)
{
	uint32_t variant;
	int result;

	result = read_reg(sdi, REG_DEV_VARIANT, &variant);
	if (result != SR_OK) {
		return VARIANT_UNKNOWN;
	}

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
	if (res != SR_OK) {
		return res;
	}

	res = upload_bitstream_to_fpga(sdi, FPGA_STAGE2_NAME);
	if (res != SR_OK) {
		return res;
	}

	res = SR_ERR_TIMEOUT;
	/* Wait until FPGA is fully configured. */
	for (fail_count = 0; fail_count < FPGA_SANITY_MAX_RECHECK; fail_count++) {
		if (fpga_reg_sanity_check(sdi)) {
			res = SR_OK;
			break;
		}
		g_usleep(FPGA_DELAY_UNIT_US);
	}

	if (res == SR_ERR_TIMEOUT) {
		sr_err("FPGA initialization never completes.");
	}
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
		ret = read_reg(sdi, 0x2034, &fw_version);
		if (ret != SR_OK) {
			break;
		}

		sr_info("MCU Version: %#010x", fw_version);

		ret = SR_OK;

		break;
	}

	libusb_free_device_list(devlist, 1);

	return ret;
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
