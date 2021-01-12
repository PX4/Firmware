/************************************************************************************
 * Driver for 24xxxx-style I2C EEPROMs.
 *
 * Adapted from:
 *
 * drivers/mtd/at24xx.c
 * Driver for I2C-based at24cxx EEPROM(at24c32,at24c64,at24c128,at24c256)
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *   History: 0.1 2011-08-20 initial version
 *
 *   2011-11-1 Added support for larger MTD block sizes: Hal Glenn <hglenn@2g-eng.com>
 *
 * Derived from drivers/mtd/m25px.c
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_mtd.h>
#include <px4_platform_common/time.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mtd/mtd.h>

#include <perf/perf_counter.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/*
 * Configuration is as for the AT24 driver, but the CONFIG_MTD_AT24XX define should be
 * omitted in order to avoid building the AT24XX driver as well.
 */

/* As a minimum, the size of the AT24 part and its 7-bit I2C address are required. */

#ifndef CONFIG_AT24XX_SIZE
/* XXX this is a well vetted special case,
 * do not issue a warning any more
 * #  warning "Assuming AT24 size 64"
 */
#  define CONFIG_AT24XX_SIZE 64
#endif
#ifndef CONFIG_AT24XX_ADDR
/* XXX this is a well vetted special case,
 * do not issue a warning any more
 * #  warning "Assuming AT24 address of 0x50"
 */
#  define CONFIG_AT24XX_ADDR 0x50
#endif

/* Get the part configuration based on the size configuration */

#if CONFIG_AT24XX_SIZE == 2       /* AT24C02: 2Kbits = 256; 16 * 16 =  256 */
#  define AT24XX_NPAGES     16
#  define AT24XX_PAGESIZE   16
#  define AT24XX_ADDRSIZE   1
#elif CONFIG_AT24XX_SIZE == 4     /* AT24C04: 4Kbits = 512B; 32 * 16 = 512 */
#  define AT24XX_NPAGES     32
#  define AT24XX_PAGESIZE   16
#  define AT24XX_ADDRSIZE   1
#elif CONFIG_AT24XX_SIZE == 8     /* AT24C08: 8Kbits = 1KiB; 64 * 16 = 1024 */
#  define AT24XX_NPAGES     64
#  define AT24XX_PAGESIZE   16
#  define AT24XX_ADDRSIZE   1
#elif CONFIG_AT24XX_SIZE == 16    /* AT24C16: 16Kbits = 2KiB; 128 * 16 = 2048 */
#  define AT24XX_NPAGES     128
#  define AT24XX_PAGESIZE   16
#  define AT24XX_ADDRSIZE   1
#elif CONFIG_AT24XX_SIZE == 32
#  define AT24XX_NPAGES     128
#  define AT24XX_PAGESIZE   32
#elif CONFIG_AT24XX_SIZE == 48
#  define AT24XX_NPAGES     192
#  define AT24XX_PAGESIZE   32
#elif CONFIG_AT24XX_SIZE == 64
#  define AT24XX_NPAGES     256
#  define AT24XX_PAGESIZE   32
#elif CONFIG_AT24XX_SIZE == 128
#  define AT24XX_NPAGES     256
#  define AT24XX_PAGESIZE   64
#elif CONFIG_AT24XX_SIZE == 256
#  define AT24XX_NPAGES     512
#  define AT24XX_PAGESIZE   64
#endif

/* For applications where a file system is used on the AT24, the tiny page sizes
 * will result in very inefficient FLASH usage.  In such cases, it is better if
 * blocks are comprised of "clusters" of pages so that the file system block
 * size is, say, 256 or 512 bytes.  In any event, the block size *must* be an
 * even multiple of the pages.
 */

#ifndef CONFIG_AT24XX_MTD_BLOCKSIZE
/* XXX this is a well vetted special case,
 * do not issue a warning any more
 * #  warning "Assuming driver block size is the same as the FLASH page size"
 */
#  define CONFIG_AT24XX_MTD_BLOCKSIZE AT24XX_PAGESIZE
#endif

/* The AT24 does not respond on the bus during write cycles, so we depend on a long
 * timeout to detect problems.  The max program time is typically ~5ms.
 */
#ifndef CONFIG_AT24XX_WRITE_TIMEOUT_MS
#  define CONFIG_AT24XX_WRITE_TIMEOUT_MS  20
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct at24c_dev_s.
 */

struct at24c_dev_s {
	struct mtd_dev_s      mtd;      /* MTD interface */
	FAR struct i2c_master_s *dev;   /* Saved I2C interface instance */
	uint8_t               addr;     /* I2C address */
	uint16_t              pagesize; /* 32, 63 */
	uint16_t              npages;   /* 128, 256, 512, 1024 */

	perf_counter_t        perf_transfers;
	perf_counter_t        perf_resets_retries;
	perf_counter_t        perf_errors;
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* MTD driver methods */

static int at24c_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t at24c_bread(FAR struct mtd_dev_s *dev, off_t startblock,
			   size_t nblocks, FAR uint8_t *buf);
static ssize_t at24c_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
			    size_t nblocks, FAR const uint8_t *buf);
static int at24c_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

void at24c_test(void);
int at24c_nuke(void);

/************************************************************************************
 * Private Data
 ************************************************************************************/

/* At present, only a single AT24 part is supported.  In this case, a statically
 * allocated state structure may be used.
 */

static struct at24c_dev_s g_at24c;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static int at24c_eraseall(FAR struct at24c_dev_s *priv)
{
	int startblock = 0;
	uint8_t buf[AT24XX_PAGESIZE + 2];

	struct i2c_msg_s msgv[1] = {
		{
			.frequency = 400000,
			.addr = priv->addr,
			.flags = 0,
			.buffer = &buf[0],
			.length = sizeof(buf),
		}
	};

	memset(&buf[2], 0xff, priv->pagesize);

	BOARD_EEPROM_WP_CTRL(false);

	for (startblock = 0; startblock < priv->npages; startblock++) {
		uint16_t offset = startblock * priv->pagesize;
		buf[1] = offset & 0xff;
		buf[0] = (offset >> 8) & 0xff;

		while (I2C_TRANSFER(priv->dev, &msgv[0], 1) < 0) {
			fwarn("erase stall\n");
			px4_usleep(10000);
		}
	}

	BOARD_EEPROM_WP_CTRL(true);

	return OK;
}

/************************************************************************************
 * Name: at24c_erase
 ************************************************************************************/

static int at24c_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
	/* EEprom need not erase */

	return (int)nblocks;
}

/************************************************************************************
 * Name: at24c_test
 ************************************************************************************/

void at24c_test(void)
{
	uint8_t buf[CONFIG_AT24XX_MTD_BLOCKSIZE];
	unsigned count = 0;
	unsigned errors = 0;

	for (count = 0; count < 10000; count++) {
		ssize_t result = at24c_bread(&g_at24c.mtd, 0, 1, buf);

		if (result == ERROR) {
			if (errors++ > 2) {
				syslog(LOG_INFO, "too many errors\n");
				return;
			}

		} else if (result != 1) {
			syslog(LOG_INFO, "unexpected %u\n", result);
		}

		if ((count % 100) == 0) {
			syslog(LOG_INFO, "test %u errors %u\n", count, errors);
		}
	}
}

/************************************************************************************
 * Name: at24c_bread
 ************************************************************************************/

static ssize_t at24c_bread(FAR struct mtd_dev_s *dev, off_t startblock,
			   size_t nblocks, FAR uint8_t *buffer)
{
	FAR struct at24c_dev_s *priv = (FAR struct at24c_dev_s *)dev;
	size_t blocksleft;
	uint8_t addr[2];
	int ret;

	struct i2c_msg_s msgv[2] = {
		{
			.frequency = 400000,
			.addr = priv->addr,
			.flags = 0,
			.buffer = &addr[0],
			.length = sizeof(addr),
		},
		{
			.frequency = 400000,
			.addr = priv->addr,
			.flags = I2C_M_READ,
			.buffer = 0,
			.length = priv->pagesize,
		}
	};

#if CONFIG_AT24XX_MTD_BLOCKSIZE > AT24XX_PAGESIZE
	startblock *= (CONFIG_AT24XX_MTD_BLOCKSIZE / AT24XX_PAGESIZE);
	nblocks    *= (CONFIG_AT24XX_MTD_BLOCKSIZE / AT24XX_PAGESIZE);
#endif
	blocksleft  = nblocks;

	finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

	if (startblock >= priv->npages) {
		return 0;
	}

	if (startblock + nblocks > priv->npages) {
		nblocks = priv->npages - startblock;
	}

	while (blocksleft-- > 0) {
		uint16_t offset = startblock * priv->pagesize;
		unsigned tries = CONFIG_AT24XX_WRITE_TIMEOUT_MS;

		addr[1] = offset & 0xff;
		addr[0] = (offset >> 8) & 0xff;
		msgv[1].buffer = buffer;

		for (;;) {

			perf_begin(priv->perf_transfers);
			ret = I2C_TRANSFER(priv->dev, &msgv[0], 2);
			perf_end(priv->perf_transfers);

			if (ret >= 0) {
				break;
			}

			finfo("read stall");
			px4_usleep(1000);

			/* We should normally only be here on the first read after
			 * a write.
			 *
			 * XXX maybe do special first-read handling with optional
			 * bus reset as well?
			 */
			perf_count(priv->perf_resets_retries);

			if (--tries == 0) {
				perf_count(priv->perf_errors);
				return ERROR;
			}
		}

		startblock++;
		buffer += priv->pagesize;
	}

#if CONFIG_AT24XX_MTD_BLOCKSIZE > AT24XX_PAGESIZE
	return nblocks / (CONFIG_AT24XX_MTD_BLOCKSIZE / AT24XX_PAGESIZE);
#else
	return nblocks;
#endif
}

/************************************************************************************
 * Name: at24c_bwrite
 *
 * Operates on MTD block's and translates to FLASH pages
 *
 ************************************************************************************/

static ssize_t at24c_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
			    FAR const uint8_t *buffer)
{
	FAR struct at24c_dev_s *priv = (FAR struct at24c_dev_s *)dev;
	size_t blocksleft;
	uint8_t buf[AT24XX_PAGESIZE + 2];
	int ret;

	struct i2c_msg_s msgv[1] = {
		{
			.frequency = 400000,
			.addr = priv->addr,
			.flags = 0,
			.buffer = &buf[0],
			.length = sizeof(buf),
		}
	};

#if CONFIG_AT24XX_MTD_BLOCKSIZE > AT24XX_PAGESIZE
	startblock *= (CONFIG_AT24XX_MTD_BLOCKSIZE / AT24XX_PAGESIZE);
	nblocks    *= (CONFIG_AT24XX_MTD_BLOCKSIZE / AT24XX_PAGESIZE);
#endif
	blocksleft  = nblocks;

	if (startblock >= priv->npages) {
		return 0;
	}

	if (startblock + nblocks > priv->npages) {
		nblocks = priv->npages - startblock;
	}

	finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

	BOARD_EEPROM_WP_CTRL(false);

	while (blocksleft-- > 0) {
		uint16_t offset = startblock * priv->pagesize;
		unsigned tries = CONFIG_AT24XX_WRITE_TIMEOUT_MS;

		buf[1] = offset & 0xff;
		buf[0] = (offset >> 8) & 0xff;
		memcpy(&buf[2], buffer, priv->pagesize);

		for (;;) {

			perf_begin(priv->perf_transfers);
			ret = I2C_TRANSFER(priv->dev, &msgv[0], 1);
			perf_end(priv->perf_transfers);

			if (ret >= 0) {
				break;
			}

			finfo("write stall");
			px4_usleep(1000);

			/* We expect to see a number of retries per write cycle as we
			 * poll for write completion.
			 */
			if (--tries == 0) {
				perf_count(priv->perf_errors);
				BOARD_EEPROM_WP_CTRL(true);
				return ERROR;
			}
		}

		startblock++;
		buffer += priv->pagesize;
	}

	BOARD_EEPROM_WP_CTRL(true);

#if CONFIG_AT24XX_MTD_BLOCKSIZE > AT24XX_PAGESIZE
	return nblocks / (CONFIG_AT24XX_MTD_BLOCKSIZE / AT24XX_PAGESIZE);
#else
	return nblocks;
#endif
}

/************************************************************************************
 * Name: at24c_ioctl
 ************************************************************************************/

static int at24c_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
	FAR struct at24c_dev_s *priv = (FAR struct at24c_dev_s *)dev;
	int ret = -EINVAL; /* Assume good command with bad parameters */

	finfo("cmd: %d \n", cmd);

	switch (cmd) {
	case MTDIOC_GEOMETRY: {
			FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)((uintptr_t)arg);

			if (geo) {
				/* Populate the geometry structure with information need to know
				 * the capacity and how to access the device.
				 *
				 * NOTE: that the device is treated as though it where just an array
				 * of fixed size blocks.  That is most likely not true, but the client
				 * will expect the device logic to do whatever is necessary to make it
				 * appear so.
				 *
				 * blocksize:
				 *   May be user defined. The block size for the at24XX devices may be
				 *   larger than the page size in order to better support file systems.
				 *   The read and write functions translate BLOCKS to pages for the
				 *   small flash devices
				 * erasesize:
				 *   It has to be at least as big as the blocksize, bigger serves no
				 *   purpose.
				 * neraseblocks
				 *   Note that the device size is in kilobits and must be scaled by
				 *   1024 / 8
				 */

#if CONFIG_AT24XX_MTD_BLOCKSIZE > AT24XX_PAGESIZE
				geo->blocksize    = CONFIG_AT24XX_MTD_BLOCKSIZE;
				geo->erasesize    = CONFIG_AT24XX_MTD_BLOCKSIZE;
				geo->neraseblocks = (CONFIG_AT24XX_SIZE * 1024 / 8) / CONFIG_AT24XX_MTD_BLOCKSIZE;
#else
				geo->blocksize    = priv->pagesize;
				geo->erasesize    = priv->pagesize;
				geo->neraseblocks = priv->npages;
#endif
				ret               = OK;

				finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
				      geo->blocksize, geo->erasesize, geo->neraseblocks);
			}
		}
		break;

	case MTDIOC_BULKERASE:
		ret = at24c_eraseall(priv);
		break;

	case MTDIOC_XIPBASE:
	default:
		ret = -ENOTTY; /* Bad command */
		break;
	}

	return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: at24c_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/
FAR struct mtd_dev_s *px4_at24c_initialize(FAR struct i2c_master_s *dev,
		uint8_t address)

{
	FAR struct at24c_dev_s *priv;

	finfo("dev: %p\n", dev);

	/* Allocate a state structure (we allocate the structure instead of using
	 * a fixed, static allocation so that we can handle multiple FLASH devices.
	 * The current implementation would handle only one FLASH part per I2C
	 * device (only because of the SPIDEV_FLASH definition) and so would have
	 * to be extended to handle multiple FLASH parts on the same I2C bus.
	 */

	priv = &g_at24c;

	if (priv) {
		/* Initialize the allocated structure */
		priv->addr       = address;
		priv->pagesize   = AT24XX_PAGESIZE;
		priv->npages     = AT24XX_NPAGES;

		priv->mtd.erase  = at24c_erase;
		priv->mtd.bread  = at24c_bread;
		priv->mtd.bwrite = at24c_bwrite;
		priv->mtd.ioctl  = at24c_ioctl;
		priv->dev        = dev;

		priv->perf_transfers = perf_alloc(PC_ELAPSED, "eeprom_trans");
		priv->perf_resets_retries = perf_alloc(PC_COUNT, "eeprom_rst");
		priv->perf_errors = perf_alloc(PC_COUNT, "eeprom_errs");
	}

	/* attempt to read to validate device is present */
	unsigned char buf[5];
	uint8_t addrbuf[2] = {0, 0};

	struct i2c_msg_s msgv[2] = {
		{
			.frequency = 400000,
			.addr = priv->addr,
			.flags = 0,
			.buffer = &addrbuf[0],
			.length = sizeof(addrbuf),
		},
		{
			.frequency = 400000,
			.addr = priv->addr,
			.flags = I2C_M_READ,
			.buffer = &buf[0],
			.length = sizeof(buf),
		}
	};

	BOARD_EEPROM_WP_CTRL(true);

	perf_begin(priv->perf_transfers);
	int ret = I2C_TRANSFER(priv->dev, &msgv[0], 2);
	perf_end(priv->perf_transfers);

	if (ret < 0) {
		return NULL;
	}

	/* Return the implementation-specific state structure as the MTD device */

	finfo("Return %p\n", priv);
	return (FAR struct mtd_dev_s *)priv;
}

/*
 * XXX: debug hackery
 */
int at24c_nuke(void)
{
	return at24c_eraseall(&g_at24c);
}
