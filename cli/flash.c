/*
 * Microsemi Switchtec(tm) PCIe Management Command Line Interface
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

#include "commands.h"
#include "argconfig.h"
#include "common.h"

#include <switchtec/switchtec.h>
#include <switchtec/flash.h>
#include <switchtec/errors.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define MRPC_ERR_SPI_FLASH_INVALID_SUBCMD 0x64023

#define CMD_DESC_SPI_FLASH_ERASE \
	"erase one 64 KiB SPI flash sector (destructive)"

static int spi_flash_erase(int argc, char **argv)
{
	int ret;
	struct switchtec_spi_flash_erase_result res;

	static struct {
		struct switchtec_dev *dev;
		unsigned long offset;
		int assume_yes;
	} cfg = {};
	const struct argconfig_options opts[] = {
		DEVICE_OPTION,
		{"offset", .cfg_type = CFG_LONG_SUFFIX,
		 .value_addr = &cfg.offset,
		 .argument_type = required_positional,
		 .help = "sector-beginning byte offset to erase "
			 "(must be a multiple of 0x10000)"},
		{"yes", 'y', "", CFG_NONE, &cfg.assume_yes, no_argument,
		 "assume yes when prompted"},
		{NULL}};

	argconfig_parse(argc, argv, CMD_DESC_SPI_FLASH_ERASE, opts,
			&cfg, sizeof(cfg));

	if (!switchtec_is_gen6(cfg.dev)) {
		fprintf(stderr, "The 'spi-flash' command set is not supported for this generation.\n");
		return 1;
	}
	if (cfg.offset >= SWITCHTEC_SPI_FLASH_SIZE) {
		fprintf(stderr,
			"offset 0x%lx is outside the 0x%lx-byte SPI flash\n",
			cfg.offset,
			(unsigned long)SWITCHTEC_SPI_FLASH_SIZE);
		return 1;
	}

	if (cfg.offset % SWITCHTEC_SPI_FLASH_SECTOR_SIZE) {
		fprintf(stderr,
			"offset 0x%lx is not a multiple of the 64 KiB "
			"sector size (0x10000)\n", cfg.offset);
		return 1;
	}

	printf("This will erase the 64 KiB sector at offset 0x%08lx "
	       "(0x%08lx-0x%08lx) on %s.\n",
	       cfg.offset, cfg.offset,
	       cfg.offset + SWITCHTEC_SPI_FLASH_SECTOR_SIZE - 1,
	       switchtec_name(cfg.dev));
	printf("Note: real-hardware erase timing for a single 64 KiB "
	       "sector varies from about 6ms to over 120ms depending on "
	       "whether the sector already held real data. This command "
	       "blocks until the erase completes.\n");

	ret = ask_if_sure(cfg.assume_yes);
	if (ret)
		return ret;

	ret = switchtec_spi_flash_erase_sector(cfg.dev, cfg.offset, &res);
	if (ret) {
		if (ret == MRPC_ERR_SPI_FLASH_INVALID_SUBCMD)
			fprintf(stderr,
				"Error: SPI flash Erase Sector is not "
				"available on this firmware build (rejected "
				"as reserved/unimplemented).\n");
		switchtec_perror("spi-flash-erase");
		return 1;
	}

	printf("Erased 0x%08x-0x%08x (flm_rc=0x%x).\n",
	       res.erased_start, res.erased_end, res.flm_rc);
	return 0;
}

#define CMD_DESC_SPI_FLASH_READ "read raw bytes from the SPI flash"

static int spi_flash_read(int argc, char **argv)
{
	int ret;
	void *buf;

	static struct {
		struct switchtec_dev *dev;
		unsigned long offset;
		size_t length;
		int out_fd;
	} cfg = {
		.out_fd = -1,
	};
	const struct argconfig_options opts[] = {
		DEVICE_OPTION,
		{"offset", .cfg_type = CFG_LONG_SUFFIX,
		 .value_addr = &cfg.offset,
		 .argument_type = required_positional,
		 .help = "byte offset to read from"},
		{"length", .cfg_type = CFG_SIZE_SUFFIX,
		 .value_addr = &cfg.length,
		 .argument_type = required_positional,
		 .help = "number of bytes to read"},
		{"filename", .cfg_type = CFG_FD_WR,
		 .value_addr = &cfg.out_fd,
		 .argument_type = optional_positional,
		 .help = "output file (default: stdout)"},
		{NULL}};

	argconfig_parse(argc, argv, CMD_DESC_SPI_FLASH_READ, opts,
			&cfg, sizeof(cfg));

	if (!switchtec_is_gen6(cfg.dev)) {
		fprintf(stderr, "The 'spi-flash' command set is not supported for this generation.\n");
		return 1;
	}
	if (!cfg.length) {
		fprintf(stderr, "length must be non-zero\n");
		return 1;
	}

	if (cfg.offset >= SWITCHTEC_SPI_FLASH_SIZE) {
		fprintf(stderr,
			"offset 0x%lx is outside the 0x%lx-byte SPI flash\n",
			cfg.offset,
			(unsigned long)SWITCHTEC_SPI_FLASH_SIZE);
		return 1;
	}
	if ((unsigned long)cfg.length >
	    (unsigned long)(SWITCHTEC_SPI_FLASH_SIZE - cfg.offset)) {
		fprintf(stderr,
			"range [0x%lx, 0x%lx) exceeds the 0x%lx-byte SPI flash\n",
			cfg.offset,
			cfg.offset + (unsigned long)cfg.length,
			(unsigned long)SWITCHTEC_SPI_FLASH_SIZE);
		return 1;
	}

	buf = malloc(cfg.length);
	if (!buf) {
		perror("malloc");
		return 1;
	}

	ret = switchtec_spi_flash_read(cfg.dev, cfg.offset, cfg.length, buf);
	if (ret < 0) {
		switchtec_perror("spi-flash-read");
		free(buf);
		return 1;
	}

	if (cfg.out_fd == -1)
		cfg.out_fd = STDOUT_FILENO;

	if (write(cfg.out_fd, buf, cfg.length) != (ssize_t)cfg.length) {
		perror("write");
		free(buf);
		return 1;
	}

	free(buf);
	return 0;
}

#define CMD_DESC_SPI_FLASH_WRITE \
	"write raw bytes to the SPI flash (does not erase -- the " \
	"destination must already be erased, or pass --erase)"

static int spi_flash_write(int argc, char **argv)
{
	int ret;
	long len;
	void *buf;

	static struct {
		struct switchtec_dev *dev;
		unsigned long offset;
		int in_fd;
		int erase_first;
		int assume_yes;
	} cfg = {
		.in_fd = -1,
	};
	const struct argconfig_options opts[] = {
		DEVICE_OPTION,
		{"offset", .cfg_type = CFG_LONG_SUFFIX,
		 .value_addr = &cfg.offset,
		 .argument_type = required_positional,
		 .help = "byte offset to write to"},
		{"filename", .cfg_type = CFG_FD_RD,
		 .value_addr = &cfg.in_fd,
		 .argument_type = required_positional,
		 .help = "input file"},
		{"erase", 'e', "", CFG_NONE, &cfg.erase_first, no_argument,
		 "erase every 64 KiB sector spanned by the write range "
		 "first (destructive -- prompts for confirmation unless "
		 "--yes is also given)"},
		{"yes", 'y', "", CFG_NONE, &cfg.assume_yes, no_argument,
		 "assume yes when prompted (only relevant with --erase)"},
		{NULL}};

	argconfig_parse(argc, argv, CMD_DESC_SPI_FLASH_WRITE, opts,
			&cfg, sizeof(cfg));

	if (!switchtec_is_gen6(cfg.dev)) {
		fprintf(stderr, "The 'spi-flash' command set is not supported for this generation.\n");
		return 1;
	}
	if (cfg.offset >= SWITCHTEC_SPI_FLASH_SIZE) {
		fprintf(stderr,
			"offset 0x%lx is outside the 0x%lx-byte SPI flash\n",
			cfg.offset,
			(unsigned long)SWITCHTEC_SPI_FLASH_SIZE);
		return 1;
	}

	len = lseek(cfg.in_fd, 0, SEEK_END);
	if (len < 0) {
		perror("lseek");
		return 1;
	}
	lseek(cfg.in_fd, 0, SEEK_SET);

	if (!len) {
		fprintf(stderr, "input file is empty\n");
		return 1;
	}

	if ((unsigned long)len >
	    (unsigned long)(SWITCHTEC_SPI_FLASH_SIZE - cfg.offset)) {
		fprintf(stderr,
			"range [0x%lx, 0x%lx) exceeds the 0x%lx-byte SPI flash\n",
			cfg.offset,
			cfg.offset + (unsigned long)len,
			(unsigned long)SWITCHTEC_SPI_FLASH_SIZE);
		return 1;
	}

	buf = malloc(len);
	if (!buf) {
		perror("malloc");
		return 1;
	}

	if (read(cfg.in_fd, buf, len) != len) {
		perror("read");
		free(buf);
		return 1;
	}

	if (cfg.erase_first) {
		unsigned long sector_start =
			cfg.offset - (cfg.offset % SWITCHTEC_SPI_FLASH_SECTOR_SIZE);
		unsigned long sector_end =
			cfg.offset + len - 1;
		sector_end -= sector_end % SWITCHTEC_SPI_FLASH_SECTOR_SIZE;
		unsigned long s;

		printf("This will erase every 64 KiB sector from "
		       "0x%08lx to 0x%08lx on %s before writing.\n",
		       sector_start,
		       sector_end + SWITCHTEC_SPI_FLASH_SECTOR_SIZE - 1,
		       switchtec_name(cfg.dev));

		ret = ask_if_sure(cfg.assume_yes);
		if (ret) {
			free(buf);
			return ret;
		}

		for (s = sector_start;
		     s <= sector_end && s < SWITCHTEC_SPI_FLASH_SIZE;
		     s += SWITCHTEC_SPI_FLASH_SECTOR_SIZE) {
			ret = switchtec_spi_flash_erase_sector(cfg.dev, s,
								NULL);
			if (ret) {
				if (ret == MRPC_ERR_SPI_FLASH_INVALID_SUBCMD)
					fprintf(stderr,
						"Error: SPI flash Erase Sector "
						"is not available on this "
						"firmware build (rejected as "
						"reserved/unimplemented); "
						"aborting before write.\n");
				switchtec_perror("spi-flash-write (erase)");
				free(buf);
				return 1;
			}
		}
	}

	ret = switchtec_spi_flash_write(cfg.dev, cfg.offset, len, buf);
	free(buf);
	if (ret < 0) {
		/* switchtec_spi_flash_write() collapses to -1 on any
		 * failure; the raw MRPC_ERR_SPI_FLASH_* code (needed to
		 * detect INVALID_SUBCMD specifically) only survives in
		 * errno, set by the transport backend (see lib/flash.c).
		 */
		if (ERRNO_MRPC(errno) == MRPC_ERR_SPI_FLASH_INVALID_SUBCMD)
			fprintf(stderr,
				"Error: SPI flash Write is not available on "
				"this firmware build (rejected as "
				"reserved/unimplemented).\n");
		switchtec_perror("spi-flash-write");
		return 1;
	}

	printf("Wrote %ld bytes to 0x%08lx-0x%08lx.\n", len, cfg.offset,
	       cfg.offset + len - 1);
	return 0;
}

#define CMD_DESC_SPI_FLASH_GET_ERASE_SIZE \
	"query the erase-sector size covering a given SPI flash offset"

static int spi_flash_get_erase_size(int argc, char **argv)
{
	int ret;
	uint32_t erase_size;

	static struct {
		struct switchtec_dev *dev;
		unsigned long offset;
	} cfg = {};
	const struct argconfig_options opts[] = {
		DEVICE_OPTION,
		{"offset", .cfg_type = CFG_LONG_SUFFIX,
		 .value_addr = &cfg.offset,
		 .argument_type = required_positional,
		 .help = "byte offset whose covering sector size to query"},
		{NULL}};

	argconfig_parse(argc, argv, CMD_DESC_SPI_FLASH_GET_ERASE_SIZE, opts,
			&cfg, sizeof(cfg));

	if (!switchtec_is_gen6(cfg.dev)) {
		fprintf(stderr, "The 'spi-flash' command set is not supported for this generation.\n");
		return 1;
	}
	if (cfg.offset >= SWITCHTEC_SPI_FLASH_SIZE) {
		fprintf(stderr,
			"offset 0x%lx is outside the 0x%lx-byte SPI flash\n",
			cfg.offset,
			(unsigned long)SWITCHTEC_SPI_FLASH_SIZE);
		return 1;
	}

	ret = switchtec_spi_flash_get_erase_size(cfg.dev, cfg.offset,
						 &erase_size);
	if (ret) {
		switchtec_perror("spi-flash-get-erase-size");
		return 1;
	}

	printf("Erase-sector size at 0x%08lx: 0x%x (%u bytes)\n",
	       cfg.offset, erase_size, erase_size);
	return 0;
}

static const struct cmd commands[] = {
	{"erase", spi_flash_erase, CMD_DESC_SPI_FLASH_ERASE},
	{"read", spi_flash_read, CMD_DESC_SPI_FLASH_READ},
	{"write", spi_flash_write, CMD_DESC_SPI_FLASH_WRITE},
	{"get_erase_size", spi_flash_get_erase_size,
	 CMD_DESC_SPI_FLASH_GET_ERASE_SIZE},
	{}
};

static struct subcommand subcmd = {
	.name = "spi_flash",
	.cmds = commands,
	.desc = "Generic SPI flash erase/read/write (MRPC_SPI_FLASH_OP, dangerous)",
	.long_desc = "These commands issue raw erase/read/write/get-erase-size "
	      "operations directly against the SPI flash device with no "
	      "partition or region-awareness and no secure-state gate. "
	      "A misdirected erase or write can corrupt or brick the device. "
	      "Use with extreme caution. (Gen 6 devices only)",
};

REGISTER_SUBCMD(subcmd);
