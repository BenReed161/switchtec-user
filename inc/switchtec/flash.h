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
 * @file
 * @brief Raw SPI flash access functions.
 */

/**
 * @defgroup SPI-Flash
 * @brief Functions to access the SPI Flash
 *
 * Generic, low-level SPI flash primitives: erase one 64 KiB sector, read
 * raw bytes, write raw bytes, and query the erase-sector size covering a
 * given offset. Not bound to any particular use case (e.g. 
 * downloading/restoring a full image). These can be composed for whatever 
 * purpose is needed.
 *
 * All four sub-commands are synchronous: each is a single MRPC call that
 * blocks until the firmware has completed the operation and returns the
 * result directly.
 *
 * Read and Write have different per-MRPC-call ceilings. Read is capped
 * at SWITCHTEC_SPI_FLASH_READ_MAX bytes (the response uses the no-echo
 * output layout so the full 1024-byte payload is available), and Write
 * is capped at SWITCHTEC_SPI_FLASH_WRITE_MAX bytes (12-byte input header
 * is co-resident with the payload in the same 1024-byte buffer, leaving
 * 1012 for data. switchtec_spi_flash_read()/_write() chunk transparently
 * for callers that pass a larger length.
 */

#ifndef LIBSWITCHTEC_FLASH_H
#define LIBSWITCHTEC_FLASH_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct switchtec_dev;

/**
 * @brief MRPC_SPI_FLASH_OP sub-commands.
 */
enum switchtec_spi_flash_sub_cmd {
	SWITCHTEC_SPI_FLASH_READ            = 0,
	SWITCHTEC_SPI_FLASH_WRITE           = 1,
	SWITCHTEC_SPI_FLASH_ERASE_SECTOR    = 2,
	SWITCHTEC_SPI_FLASH_GET_ERASE_SIZE  = 3,
};

/** Flat, unconditional flash size assumed by the firmware (non-ECC). */
#define SWITCHTEC_SPI_FLASH_SIZE          0x1000000UL   /* 16 MiB */
/** Erase-sector alignment/size (64 KiB, non-ECC). */
#define SWITCHTEC_SPI_FLASH_SECTOR_SIZE   0x10000UL
/** Per-MRPC-call ceiling for Read payloads (firmware-enforced): the full
 *  1024-byte MRPC output buffer is available. */
#define SWITCHTEC_SPI_FLASH_READ_MAX      1024U
/** Per-MRPC-call ceiling for Write payloads (firmware-enforced): the
 *  12-byte input header is co-resident with the payload in the same
 *  1024-byte MRPC buffer, so only 1012 bytes remain for data. */
#define SWITCHTEC_SPI_FLASH_WRITE_MAX     1012U

/**
 * @brief Result of a synchronous Erase Sector call.
 */
struct switchtec_spi_flash_erase_result {
	/** 0 on success, or the raw MRPC_ERR_SPI_FLASH_* rejection code
	 *  (full 32-bit little-endian value). */
	uint32_t status;
	/** Raw flm_erase()/flm_init() return code (0 unless a failure). */
	uint32_t flm_rc;
	/** Sector-aligned start offset erased. */
	uint32_t erased_start;
	/** Sector-aligned end offset erased. */
	uint32_t erased_end;
};

/**
 * @brief Erase one 64 KiB sector.
 * @param[in]  dev	Switchtec device handle
 * @param[in]  offset	Sector-beginning byte offset (must be a multiple
 *			of SWITCHTEC_SPI_FLASH_SECTOR_SIZE)
 * @param[out] res	Filled in with the result (status/flm_rc/erased
 *			range) on both success and validation-rejection --
 *			may be NULL if the caller only cares about the
 *			return value.
 * @return 0 on success, -1 on transport error, or the raw positive
 *	MRPC_ERR_SPI_FLASH_* code on rejection/failure.
 */
int switchtec_spi_flash_erase_sector(struct switchtec_dev *dev,
				      uint32_t offset,
				      struct switchtec_spi_flash_erase_result *res);

/**
 * @brief Query the erase-sector size covering a given offset.
 * @param[in]  dev		Switchtec device handle
 * @param[in]  offset		Byte offset whose covering sector size is
 *				queried
 * @param[out] erase_size	Filled in with the size in bytes on success
 * @return 0 on success, -1 on transport error, or the raw positive
 *	MRPC_ERR_SPI_FLASH_* code on failure.
 */
int switchtec_spi_flash_get_erase_size(struct switchtec_dev *dev,
					uint32_t offset,
					uint32_t *erase_size);

/**
 * @brief Read raw bytes from the SPI flash.
 * @param[in]  dev	Switchtec device handle
 * @param[in]  offset	Byte offset to read from
 * @param[in]  len	Number of bytes to read
 * @param[out] buf	Destination buffer, at least len bytes
 * @return Number of bytes read (== len) on success, -1 on error.
 */
int switchtec_spi_flash_read(struct switchtec_dev *dev, uint32_t offset,
			      size_t len, void *buf);

/**
 * @brief Write raw bytes to the SPI flash.
 * @param[in]  dev	Switchtec device handle
 * @param[in]  offset	Byte offset to write to
 * @param[in]  len	Number of bytes to write
 * @param[in]  buf	Source buffer, at least len bytes
 * @return Number of bytes written (== len) on success, -1 on error.
 *
 * Write never erases and the destination must already be erased (see
 * switchtec_spi_flash_erase_sector()). This matches the firmware's own
 * spi write primitive.
 */
int switchtec_spi_flash_write(struct switchtec_dev *dev, uint32_t offset,
			       size_t len, const void *buf);

 /** Returned by switchtec_spi_flash_download() when verify is true and the
 *  post-write read-back did not match the source data. Deliberately
 *  outside the MRPC_ERR_SPI_FLASH_* range (all 0x64000+, always positive)
 *  so callers can tell a verify mismatch apart from a firmware-rejected
 *  sub-command with a simple sign check it is a host-side-only
 *  comparison failure, not something the device itself returned. */
#define SWITCHTEC_SPI_FLASH_VERIFY_MISMATCH  (-2)

/**
 * @brief Download (erase + write, optionally verify) a raw image from a
 *        file into the SPI flash at a given offset
 *
 * Three phases, each independently progress-reported:
 *   1. Erase every sector the [offset, offset + len) range covers. The
 *      sector stride comes from switchtec_spi_flash_get_erase_size() at
 *      the range's start offset. Skipped entirely if erase is false
 *   2. Write the file's len bytes starting at offset, chunked at
 *      SWITCHTEC_SPI_FLASH_WRITE_MAX per MRPC call.
 *   3. If verify is true, read the [offset, offset + len) range back and 
 *      byte-compare it against the same file, rewound and re-read a chunk 
 *      at a time.
 *
 * @param[in] dev     Switchtec device handle
 * @param[in] fimg    Open, readable, seekable file
 * @param[in] offset  Destination byte offset. Does not need to be
 *                    sector-aligned.
 * @param[in] len     Number of bytes to write
 * @param[in] erase   If true, erase every sector the write range covers first. 
 * 		      If false, the caller asserts the destination is already 
 * 		      erased and phase 1 is skipped entirely.
 * @param[in] verify  If true, run phase 3 (read-back + compare) after the
 *                    write completes.
 * @param[in] erase_progress_callback   Optional; called after each sector
 *                                     erase with (sectors erased so far,
 *                                     total sectors to erase). Never
 *                                     called if erase is false.
 * @param[in] write_progress_callback   Optional; called after each write
 *                                     chunk with (bytes written so far,
 *                                     len).
 * @param[in] verify_progress_callback  Optional; called after each verify
 *                                     chunk with (bytes verified so far,
 *                                     len). Never called if verify is
 *                                     false.
 * In the case of write failure, some chunks may have already been written
 * to the destination; no rollback is attempted. Callers requiring
 * atomicity must retry the whole download and re-erase first.
 *
 * @return 0 on success.
 */
int switchtec_spi_flash_download(struct switchtec_dev *dev, FILE *fimg,
				  uint32_t offset, size_t len,
				  bool erase, bool verify,
				  void (*erase_progress_callback)(int cur, int tot),
				  void (*write_progress_callback)(int cur, int tot),
				  void (*verify_progress_callback)(int cur, int tot));
#ifdef __cplusplus
}
#endif

#endif
