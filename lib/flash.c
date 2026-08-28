/*
 * Microsemi Switchtec(tm) PCIe Management Library
 * Copyright (c) 2026, Microsemi Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/**
 * @defgroup SPI-Flash
 * @brief Library to send read/write/erase MRPC commands to the SPI flash. 
 * 
 * Warning: These commands have direct access to low level firmware SPI 
 * opertions, they can cause device firmware booting issues and can possibly 
 * brick or corrupt the device if used incorrectly. Use only if you understand 
 * what you are doing. (See inc/switchtec/flash.h for more information).
 */

#include "switchtec_priv.h"
#include "switchtec/switchtec.h"
#include "switchtec/flash.h"
#include "switchtec/mrpc.h"
#include "switchtec/endian.h"

#include <string.h>
#include <errno.h>

struct spi_flash_erase_in {
	uint8_t  sub_cmd;
	uint8_t  reserved[3];
	uint32_t spi_offset;
};

struct spi_flash_erase_out {
	uint32_t status;
	uint32_t flm_rc;
	uint32_t erased_start;
	uint32_t erased_end;
};

struct spi_flash_get_erase_size_in {
	uint8_t  sub_cmd;
	uint8_t  reserved[3];
	uint32_t spi_offset;
};

struct spi_flash_get_erase_size_out {
	uint32_t status;
	uint32_t erase_size;
};

struct spi_flash_read_in {
	uint8_t  sub_cmd;
	uint8_t  reserved[3];
	uint32_t spi_offset;
	uint32_t read_length;
};

struct spi_flash_write_in_hdr {
	uint8_t  sub_cmd;
	uint8_t  reserved[3];
	uint32_t spi_offset;
	uint32_t data_length;
};

struct spi_flash_write_out {
	uint32_t status;
	uint32_t flm_rc;
};

/* Compile-time guard against wire-layout drift relative to the firmware
 * structs.
 */
_Static_assert(sizeof(struct spi_flash_erase_in) == 8,
	      "spi_flash_erase_in must match firmware's 8-byte wire size");
_Static_assert(sizeof(struct spi_flash_erase_out) == 16,
	      "spi_flash_erase_out must match firmware's 16-byte wire size");
_Static_assert(sizeof(struct spi_flash_get_erase_size_in) == 8,
	      "spi_flash_get_erase_size_in must match firmware's 8-byte wire size");
_Static_assert(sizeof(struct spi_flash_get_erase_size_out) == 8,
	      "spi_flash_get_erase_size_out must match firmware's 8-byte wire size");
_Static_assert(sizeof(struct spi_flash_read_in) == 12,
	      "spi_flash_read_in must match firmware's 12-byte wire size");
_Static_assert(sizeof(struct spi_flash_write_in_hdr) == 12,
	      "spi_flash_write_in_hdr must match firmware's 12-byte wire size");
_Static_assert(sizeof(struct spi_flash_write_out) == 8,
	      "spi_flash_write_out must match firmware's 8-byte wire size");

int switchtec_spi_flash_erase_sector(struct switchtec_dev *dev,
				      uint32_t offset,
				      struct switchtec_spi_flash_erase_result *res)
{
	int ret;
	struct spi_flash_erase_in in = {
		.sub_cmd = SWITCHTEC_SPI_FLASH_ERASE_SECTOR,
		.spi_offset = htole32(offset),
	};
	struct spi_flash_erase_out out;

	if (offset % SWITCHTEC_SPI_FLASH_SECTOR_SIZE) {
		errno = EINVAL;
		return -1;
	}

	ret = switchtec_cmd(dev, MRPC_SPI_FLASH_OP, &in, sizeof(in),
			    &out, sizeof(out));
	if (ret < 0)
		return ret;

	if (res) {
		res->status       = le32toh(out.status);
		res->flm_rc       = le32toh(out.flm_rc);
		res->erased_start = le32toh(out.erased_start);
		res->erased_end   = le32toh(out.erased_end);
	}

	return ret;
}

int switchtec_spi_flash_get_erase_size(struct switchtec_dev *dev,
					uint32_t offset,
					uint32_t *erase_size)
{
	int ret;
	struct spi_flash_get_erase_size_in in = {
		.sub_cmd = SWITCHTEC_SPI_FLASH_GET_ERASE_SIZE,
		.spi_offset = htole32(offset),
	};
	struct spi_flash_get_erase_size_out out;

	ret = switchtec_cmd(dev, MRPC_SPI_FLASH_OP, &in, sizeof(in),
			    &out, sizeof(out));
	if (ret < 0)
		return ret;

	if (erase_size)
		*erase_size = le32toh(out.erase_size);

	return ret;
}

int switchtec_spi_flash_read(struct switchtec_dev *dev, uint32_t offset,
			      size_t len, void *buf)
{
	int ret;
	struct spi_flash_read_in in;
	unsigned char *cbuf = buf;
	size_t total_read = 0;

	while (len) {
		size_t chunk_len = len;
		if (chunk_len > SWITCHTEC_SPI_FLASH_READ_MAX)
			chunk_len = SWITCHTEC_SPI_FLASH_READ_MAX;

		in.sub_cmd = SWITCHTEC_SPI_FLASH_READ;
		in.reserved[0] = in.reserved[1] = in.reserved[2] = 0;
		in.spi_offset = htole32(offset);
		in.read_length = htole32((uint32_t)chunk_len);

		ret = switchtec_cmd(dev, MRPC_SPI_FLASH_OP, &in, sizeof(in),
				    cbuf, chunk_len);
		if (ret)
			return -1;

		offset += chunk_len;
		len -= chunk_len;
		total_read += chunk_len;
		cbuf += chunk_len;
	}

	return (int)total_read;
}

int switchtec_spi_flash_write(struct switchtec_dev *dev, uint32_t offset,
			       size_t len, const void *buf)
{
	int ret;
	const unsigned char *cbuf = buf;
	size_t total_wrote = 0;
	unsigned char wbuf[sizeof(struct spi_flash_write_in_hdr) +
			    SWITCHTEC_SPI_FLASH_WRITE_MAX];
	struct spi_flash_write_in_hdr *hdr =
		(struct spi_flash_write_in_hdr *)wbuf;
	struct spi_flash_write_out out;

	while (len) {
		size_t chunk_len = len;
		if (chunk_len > SWITCHTEC_SPI_FLASH_WRITE_MAX)
			chunk_len = SWITCHTEC_SPI_FLASH_WRITE_MAX;

		hdr->sub_cmd = SWITCHTEC_SPI_FLASH_WRITE;
		hdr->reserved[0] = hdr->reserved[1] = hdr->reserved[2] = 0;
		hdr->spi_offset = htole32(offset);
		hdr->data_length = htole32((uint32_t)chunk_len);
		memcpy(wbuf + sizeof(*hdr), cbuf, chunk_len);

		ret = switchtec_cmd(dev, MRPC_SPI_FLASH_OP, wbuf,
				    sizeof(*hdr) + chunk_len,
				    &out, sizeof(out));
		if (ret)
			return -1;

		offset += chunk_len;
		len -= chunk_len;
		total_wrote += chunk_len;
		cbuf += chunk_len;
	}

	return (int)total_wrote;
}
