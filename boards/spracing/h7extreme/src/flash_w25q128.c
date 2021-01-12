/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 ****************************************************************************/

/**
 * @file flash_w25q128.c
 *
 * Board-specific external flash W25Q128 functions.
 */


#include "board_config.h"
#include "qspi.h"
#include "arm_internal.h"


/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#define N25Q128_SECTOR_SIZE         (4*1024)
#define N25Q128_SECTOR_SHIFT        (12)
#define N25Q128_SECTOR_COUNT        (4096)
#define N25Q128_PAGE_SIZE           (256)
#define N25Q128_PAGE_SHIFT          (8)

#define W25Q_DUMMY_CYCLES_FAST_READ_QUAD	6
#define W25Q_INSTR_FAST_READ_QUAD			0xEB
#define W25Q_ADDRESS_SIZE					3 // 3 bytes -> 24 bits

#define N25QXXX_READ_STATUS        0x05  /* Read status register:                   *
                                          *   0x05 | SR                             */
#define N25QXXX_PAGE_PROGRAM       0x02  /* Page Program:                           *
                                          *   0x02 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | data                       */
#define N25QXXX_WRITE_ENABLE       0x06  /* Write enable:                           *
                                          *   0x06                                  */
#define N25QXXX_WRITE_DISABLE      0x04  /* Write disable command code:             *
                                          *   0x04                                  */
#define N25QXXX_SUBSECTOR_ERASE    0x20  /* Sub-sector Erase (4 kB)                 *
                                          *   0x20 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS)                              */


/* N25QXXX Registers ****************************************************************/
/* Status register bit definitions                                                  */

#define STATUS_BUSY_MASK           (1 << 0) /* Bit 0: Device ready/busy status      */
#  define STATUS_READY             (0 << 0) /*   0 = Not Busy                       */
#  define STATUS_BUSY              (1 << 0) /*   1 = Busy                           */
#define STATUS_WEL_MASK            (1 << 1) /* Bit 1: Write enable latch status     */
#  define STATUS_WEL_DISABLED      (0 << 1) /*   0 = Not Write Enabled              */
#  define STATUS_WEL_ENABLED       (1 << 1) /*   1 = Write Enabled                  */
#define STATUS_BP_SHIFT            (2)      /* Bits 2-4: Block protect bits         */
#define STATUS_BP_MASK             (7 << STATUS_BP_SHIFT)
#  define STATUS_BP_NONE           (0 << STATUS_BP_SHIFT)
#  define STATUS_BP_ALL            (7 << STATUS_BP_SHIFT)
#define STATUS_TB_MASK             (1 << 5) /* Bit 5: Top / Bottom Protect          */
#  define STATUS_TB_TOP            (0 << 5) /*   0 = BP2-BP0 protect Top down       */
#  define STATUS_TB_BOTTOM         (1 << 5) /*   1 = BP2-BP0 protect Bottom up      */
#define STATUS_BP3_MASK            (1 << 5) /* Bit 6: BP3                           */
#define STATUS_SRP0_MASK           (1 << 7) /* Bit 7: Status register protect 0     */
#  define STATUS_SRP0_UNLOCKED     (0 << 7) /*   0 = WP# no effect / PS Lock Down   */
#  define STATUS_SRP0_LOCKED       (1 << 7) /*   1 = WP# protect / OTP Lock Down    */

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s must
 * appear at the beginning of the definition so that you can freely cast between
 * pointers to struct mtd_dev_s and struct n25qxxx_dev_s.
 */

struct n25qxxx_dev_s {
	//struct mtd_dev_s       mtd;         /* MTD interface */
	FAR struct qspi_dev_s *qspi;        /* Saved QuadSPI interface instance */
	uint16_t               nsectors;    /* Number of erase sectors */
	uint8_t                sectorshift; /* Log2 of sector size */
	uint8_t                pageshift;   /* Log2 of page size */
	FAR uint8_t           *cmdbuf;      /* Allocated command buffer */
	FAR uint8_t           *readbuf;     /* Allocated status read buffer */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct qspi_dev_s *ptr_qspi_dev;
struct qspi_meminfo_s qspi_meminfo = {
	.flags   = QSPIMEM_QUADIO,
	.addrlen = W25Q_ADDRESS_SIZE,
	.dummies = W25Q_DUMMY_CYCLES_FAST_READ_QUAD,
	.cmd     = W25Q_INSTR_FAST_READ_QUAD
};

struct n25qxxx_dev_s n25qxxx_dev;
uint8_t cmdbuf[4] = {0u};
uint8_t readbuf[1] = {0u};

/************************************************************************************
 * Private Functions
 ************************************************************************************/
__ramfunc__ int n25qxxx_command(FAR struct qspi_dev_s *qspi, uint8_t cmd);
__ramfunc__ uint8_t n25qxxx_read_status(FAR struct n25qxxx_dev_s *priv);
__ramfunc__ int n25qxxx_command_read(FAR struct qspi_dev_s *qspi, uint8_t cmd,
				     FAR void *buffer, size_t buflen);
__ramfunc__ void n25qxxx_write_enable(FAR struct n25qxxx_dev_s *priv);
__ramfunc__ void n25qxxx_write_disable(FAR struct n25qxxx_dev_s *priv);

__ramfunc__ int n25qxxx_write_page(struct n25qxxx_dev_s *priv, FAR const uint8_t *buffer,
				   off_t address, size_t buflen);

__ramfunc__ int n25qxxx_write_one_page(struct n25qxxx_dev_s *priv, struct qspi_meminfo_s *meminfo);

__ramfunc__ int n25qxxx_erase_sector(struct n25qxxx_dev_s *priv, off_t sector);

__ramfunc__ bool n25qxxx_isprotected(FAR struct n25qxxx_dev_s *priv, uint8_t status,
				     off_t address);

__ramfunc__  int n25qxxx_command_address(FAR struct qspi_dev_s *qspi, uint8_t cmd,
		off_t addr, uint8_t addrlen);

/************************************************************************************
 * Public Functions
 ************************************************************************************/

void flash_w25q128_init(void)
{
	int qspi_interface_number = 0;
	ptr_qspi_dev = stm32h7_qspi_initialize(qspi_interface_number);
	n25qxxx_dev.qspi = ptr_qspi_dev;
	n25qxxx_dev.cmdbuf = cmdbuf;
	n25qxxx_dev.readbuf = readbuf;
	n25qxxx_dev.sectorshift = N25Q128_SECTOR_SHIFT;
	n25qxxx_dev.pageshift   = N25Q128_PAGE_SHIFT;
	n25qxxx_dev.nsectors    = N25Q128_SECTOR_COUNT;
}

__ramfunc__ ssize_t up_progmem_ext_getpage(size_t addr)
{
	ssize_t page_address = (addr - STM32_FMC_BANK4) / N25Q128_SECTOR_COUNT;

	return page_address;
}

__ramfunc__ ssize_t up_progmem_ext_eraseblock(size_t block)
{
	ssize_t size = N25Q128_SECTOR_COUNT;

	irqstate_t irqstate = px4_enter_critical_section();
	stm32h7_qspi_exit_memorymapped(ptr_qspi_dev);

	n25qxxx_erase_sector(&n25qxxx_dev, block);

	stm32h7_qspi_enter_memorymapped(ptr_qspi_dev, &qspi_meminfo, 0);
	px4_leave_critical_section(irqstate);
	return size;
}

__ramfunc__ ssize_t up_progmem_ext_write(size_t addr, FAR const void *buf, size_t count)
{
	ssize_t ret_val = 0;

	irqstate_t irqstate = px4_enter_critical_section();
	stm32h7_qspi_exit_memorymapped(ptr_qspi_dev);

	addr &=  0xFFFFFF;
	n25qxxx_write_page(&n25qxxx_dev, buf, (off_t)addr, count);

	stm32h7_qspi_enter_memorymapped(ptr_qspi_dev, &qspi_meminfo, 0);
	px4_leave_critical_section(irqstate);

	return ret_val;
}

/************************************************************************************
 * Name: n25qxxx_command
 ************************************************************************************/

__ramfunc__ int n25qxxx_command(FAR struct qspi_dev_s *qspi, uint8_t cmd)
{
	struct qspi_cmdinfo_s cmdinfo;

	finfo("CMD: %02x\n", cmd);

	cmdinfo.flags   = 0;
	cmdinfo.addrlen = 0;
	cmdinfo.cmd     = cmd;
	cmdinfo.buflen  = 0;
	cmdinfo.addr    = 0;
	cmdinfo.buffer  = NULL;

	int rv;
	rv = qspi_command(qspi, &cmdinfo);
	return rv;
}

/************************************************************************************
 * Name: n25qxxx_read_status
 ************************************************************************************/

__ramfunc__ uint8_t n25qxxx_read_status(FAR struct n25qxxx_dev_s *priv)
{
	DEBUGVERIFY(n25qxxx_command_read(priv->qspi, N25QXXX_READ_STATUS,
					 (FAR void *)&priv->readbuf[0], 1));
	return priv->readbuf[0];
}

/************************************************************************************
 * Name: n25qxxx_command_read
 ************************************************************************************/

__ramfunc__ int n25qxxx_command_read(FAR struct qspi_dev_s *qspi, uint8_t cmd,
				     FAR void *buffer, size_t buflen)
{
	struct qspi_cmdinfo_s cmdinfo;

	finfo("CMD: %02x buflen: %lu\n", cmd, (unsigned long)buflen);

	cmdinfo.flags   = QSPICMD_READDATA;
	cmdinfo.addrlen = 0;
	cmdinfo.cmd     = cmd;
	cmdinfo.buflen  = buflen;
	cmdinfo.addr    = 0;
	cmdinfo.buffer  = buffer;

	int rv;
	rv = qspi_command(qspi, &cmdinfo);
	return rv;
}


/************************************************************************************
 * Name:  n25qxxx_write_enable
 ************************************************************************************/

__ramfunc__ void n25qxxx_write_enable(FAR struct n25qxxx_dev_s *priv)
{
	uint8_t status;

	do {
		n25qxxx_command(priv->qspi, N25QXXX_WRITE_ENABLE);
		status = n25qxxx_read_status(priv);
	} while ((status & STATUS_WEL_MASK) != STATUS_WEL_ENABLED);
}

/************************************************************************************
 * Name:  n25qxxx_write_disable
 ************************************************************************************/

__ramfunc__ void n25qxxx_write_disable(FAR struct n25qxxx_dev_s *priv)
{
	uint8_t status;

	do {
		n25qxxx_command(priv->qspi, N25QXXX_WRITE_DISABLE);
		status = n25qxxx_read_status(priv);
	} while ((status & STATUS_WEL_MASK) != STATUS_WEL_DISABLED);
}

/************************************************************************************
 * Name:  n25qxxx_write_page
 ************************************************************************************/

__ramfunc__ int n25qxxx_write_page(struct n25qxxx_dev_s *priv, FAR const uint8_t *buffer,
				   off_t address, size_t buflen)
{
	struct qspi_meminfo_s meminfo;
	unsigned int pagesize;
	unsigned int npages;
	unsigned int firstpagesize = 0;
	int ret = OK;
	unsigned int i;

	finfo("address: %08lx buflen: %u\n", (unsigned long)address, (unsigned)buflen);

	pagesize = (1 << priv->pageshift);

	/* Set up non-varying parts of transfer description */

	meminfo.flags   = QSPIMEM_WRITE;
	meminfo.cmd     = N25QXXX_PAGE_PROGRAM;
	meminfo.addrlen = 3;
	meminfo.dummies = 0;
	meminfo.buffer = (void *)buffer;

	if (0 != (address % pagesize)) {
		firstpagesize = pagesize - (address % pagesize);
	}

	if (buflen <= firstpagesize) {
		meminfo.addr   = address;
		meminfo.buflen  = buflen;
		ret = n25qxxx_write_one_page(priv, &meminfo);

	} else {

		if (firstpagesize > 0) {
			meminfo.addr   = address;
			meminfo.buflen  = firstpagesize;
			ret = n25qxxx_write_one_page(priv, &meminfo);

			buffer  += firstpagesize;
			address += firstpagesize;
			buflen -= firstpagesize;
		}

		npages   = (buflen >> priv->pageshift);

		meminfo.buflen  = pagesize;

		/* Then write each page */

		for (i = 0; (i < npages) && (ret == OK); i++) {
			/* Set up varying parts of the transfer description */

			meminfo.addr   = address;
			meminfo.buffer = (void *)buffer;

			ret = n25qxxx_write_one_page(priv, &meminfo);

			/* Update for the next time through the loop */

			buffer  += pagesize;
			address += pagesize;
			buflen  -= pagesize;
		}

		if ((ret == OK) && (buflen > 0)) {
			meminfo.addr   = address;
			meminfo.buffer = (void *)buffer;
			meminfo.buflen = buflen;

			ret = n25qxxx_write_one_page(priv, &meminfo);
		}
	}

	return ret;
}

__ramfunc__ int n25qxxx_write_one_page(struct n25qxxx_dev_s *priv, struct qspi_meminfo_s *meminfo)
{
	int ret;

	n25qxxx_write_enable(priv);
	ret = qspi_memory(priv->qspi, meminfo);
	n25qxxx_write_disable(priv);

	if (ret < 0) {
		ferr("ERROR: QSPI_MEMORY failed writing address=%06x\n",
		     meminfo->addr);
	}

	return ret;
}

/************************************************************************************
 * Name:  n25qxxx_erase_sector
 ************************************************************************************/

__ramfunc__ int n25qxxx_erase_sector(struct n25qxxx_dev_s *priv, off_t sector)
{
	off_t address;
	uint8_t status;

	finfo("sector: %08lx\n", (unsigned long)sector);

	/* Check that the flash is ready and unprotected */

	status = n25qxxx_read_status(priv);

	if ((status & STATUS_BUSY_MASK) != STATUS_READY) {
		ferr("ERROR: Flash busy: %02x", status);
		return -EBUSY;
	}

	/* Get the address associated with the sector */

	address = (off_t)sector << priv->sectorshift;

	if ((status & (STATUS_BP3_MASK | STATUS_BP_MASK)) != 0 &&
	    n25qxxx_isprotected(priv, status, address)) {
		ferr("ERROR: Flash protected: %02x", status);
		return -EACCES;
	}

	/* Send the sector erase command */

	n25qxxx_write_enable(priv);
	n25qxxx_command_address(priv->qspi, N25QXXX_SUBSECTOR_ERASE, address, 3);

	/* Wait for erasure to finish */

	while ((n25qxxx_read_status(priv) & STATUS_BUSY_MASK) != 0);

	return OK;
}

/************************************************************************************
 * Name: n25qxxx_isprotected
 ************************************************************************************/

__ramfunc__ bool n25qxxx_isprotected(FAR struct n25qxxx_dev_s *priv, uint8_t status,
				     off_t address)
{
	off_t protstart;
	off_t protend;
	off_t protsize;
	unsigned int bp;

	/* The BP field is spread across non-contiguous bits */

	bp = (status & STATUS_BP_MASK) >> STATUS_BP_SHIFT;

	if (status & STATUS_BP3_MASK) {
		bp |= 8;
	}

	/* the BP field is essentially the power-of-two of the number of 64k sectors,
	* saturated to the device size.
	*/

	if (0 == bp) {
		return false;
	}

	protsize = 0x00010000;
	protsize <<= (protsize << (bp - 1));
	protend = (1 << priv->sectorshift) * priv->nsectors;

	if (protsize > protend) {
		protsize = protend;
	}

	/* The final protection range then depends on if the protection region is
	* configured top-down or bottom up  (assuming CMP=0).
	*/

	if ((status & STATUS_TB_MASK) != 0) {
		protstart = 0x00000000;
		protend   = protstart + protsize;

	} else {
		protstart = protend - protsize;
		/* protend already computed above */
	}

	return (address >= protstart && address < protend);
}

/************************************************************************************
 * Name: n25qxxx_command_address
 ************************************************************************************/

__ramfunc__  int n25qxxx_command_address(FAR struct qspi_dev_s *qspi, uint8_t cmd,
		off_t addr, uint8_t addrlen)
{
	struct qspi_cmdinfo_s cmdinfo;

	finfo("CMD: %02x Address: %04lx addrlen=%d\n", cmd, (unsigned long)addr, addrlen);

	cmdinfo.flags   = QSPICMD_ADDRESS;
	cmdinfo.addrlen = addrlen;
	cmdinfo.cmd     = cmd;
	cmdinfo.buflen  = 0;
	cmdinfo.addr    = addr;
	cmdinfo.buffer  = NULL;

	int rv;
	rv = qspi_command(qspi, &cmdinfo);
	return rv;
}
