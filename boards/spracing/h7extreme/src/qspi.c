/****************************************************************************
 * arch/arm/src/stm32h7/stm32_qspi.c
 *
 *   Copyright (C) 2016-2017, 2019 Gregory Nutt. All rights reserved.
 *   Author: dev@ziggurat29.com
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/cache.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/qspi.h>

#include "arm_internal.h"
#include "arm_arch.h"
#include "barriers.h"

#include "stm32_gpio.h"
#include "stm32_dma.h"
#include "stm32_rcc.h"
#include "hardware/stm32_qspi.h"
#include "qspi.h"

#ifdef CONFIG_STM32H7_QUADSPI
#define QSPI_BOOT_IN_MEMORY_MAPPED_MOD

/****************************************************************************
 * This defines exist in NuttX from 8.2.2020.
 * It can be deleted once when NuttX in submodule will support it.
 ****************************************************************************/

#define QSPICMD_IDUAL         (1 << 3)  /* Bit 3: Instruction on two lines */
#define QSPICMD_IQUAD         (1 << 4)  /* Bit 4: Instruction on four lines */

#define QSPICMD_ISIDUAL(f)    (((f) & QSPICMD_IDUAL) != 0)
#define QSPICMD_ISIQUAD(f)    (((f) & QSPICMD_IQUAD) != 0)

#define QSPIMEM_IDUAL         (1 << 7)  /* Bit 7: Instruction on two lines */
#define QSPIMEM_IQUAD         (1 << 0)  /* Bit 0: Instruction on four lines */

#define QSPIMEM_ISIDUAL(f)    (((f) & QSPIMEM_IDUAL) != 0)
#define QSPIMEM_ISIQUAD(f)    (((f) & QSPIMEM_IQUAD) != 0)

/* Non-atomic, but more effective modification of registers */

# define modreg8(v,m,a)       putreg8((getreg8(a) & ~(m)) | ((v) & (m)), a)
# define modreg16(v,m,a)      putreg16((getreg16(a) & ~(m)) | ((v) & (m)), a)
# define modreg32(v,m,a)      putreg32((getreg32(a) & ~(m)) | ((v) & (m)), a)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* QSPI memory synchronization */

#define MEMORY_SYNC()     do { ARM_DSB(); ARM_ISB(); } while (0)

/* Ensure that the DMA buffers are word-aligned. */

#define ALIGN_SHIFT       2
#define ALIGN_MASK        3
#define ALIGN_UP(n)       (((n)+ALIGN_MASK) & ~ALIGN_MASK)
#define IS_ALIGNED(n)     (((uint32_t)(n) & ALIGN_MASK) == 0)

/* Debug ********************************************************************/

/* Check if QSPI debug is enabled */

#ifndef CONFIG_DEBUG_DMA
#  undef CONFIG_STM32H7_QSPI_DMADEBUG
#endif

#define DMA_INITIAL      0
#define DMA_AFTER_SETUP  1
#define DMA_AFTER_START  2
#define DMA_CALLBACK     3
#define DMA_TIMEOUT      3
#define DMA_END_TRANSFER 4
#define DMA_NSAMPLES     5

/* Can't have both interrupt-driven QSPI and DMA QSPI */

#if defined(CONFIG_STM32H7_QSPI_INTERRUPTS) && defined(CONFIG_STM32H7_QSPI_DMA)
#  error "Cannot enable both interrupt mode and DMA mode for QSPI"
#endif

/* Sanity check that board.h defines requisite QSPI pinmap options for */

#if (!defined(GPIO_QSPI_CS) || !defined(GPIO_QSPI_IO0) || !defined(GPIO_QSPI_IO1) || \
    !defined(GPIO_QSPI_IO2) || !defined(GPIO_QSPI_IO3) || !defined(GPIO_QSPI_SCK))
#  error you must define QSPI pinmapping options for GPIO_QSPI_CS GPIO_QSPI_IO0 \
GPIO_QSPI_IO1 GPIO_QSPI_IO2 GPIO_QSPI_IO3 GPIO_QSPI_SCK in your board.h
#endif

#ifdef CONFIG_STM32H7_QSPI_DMA

#  ifdef DMAMAP_QUADSPI

/* QSPI DMA Channel/Stream selection.  There
 * are multiple DMA stream options that must be dis-ambiguated in the board.h
 * file.
 */

#    define DMACHAN_QUADSPI           DMAMAP_QUADSPI
#  endif

#  if defined(CONFIG_STM32H7_QSPI_DMAPRIORITY_LOW)
#    define QSPI_DMA_PRIO  DMA_SCR_PRILO
#  elif defined(CONFIG_STM32H7_QSPI_DMAPRIORITY_MEDIUM)
#    define QSPI_DMA_PRIO  DMA_SCR_PRIMED
#  elif defined(CONFIG_STM32H7_QSPI_DMAPRIORITY_HIGH)
#    define QSPI_DMA_PRIO  DMA_SCR_PRIHI
#  elif defined(CONFIG_STM32H7_QSPI_DMAPRIORITY_VERYHIGH)
#    define QSPI_DMA_PRIO  DMA_SCR_PRIVERYHI
#  else
#    define QSPI_DMA_PRIO  DMA_SCR_PRIMED
#  endif

#endif /* CONFIG_STM32H7_QSPI_DMA */

#ifndef STM32_SYSCLK_FREQUENCY
#  error your board.h needs to define the value of STM32_SYSCLK_FREQUENCY
#endif

#if !defined(CONFIG_STM32H7_QSPI_FLASH_SIZE) || 0 == CONFIG_STM32H7_QSPI_FLASH_SIZE
#  error you must specify a positive flash size via CONFIG_STM32H7_QSPI_FLASH_SIZE
#endif

/* DMA timeout.  The value is not critical; we just don't want the system to
 * hang in the event that a DMA does not finish.
 */

#define DMA_TIMEOUT_MS    (800)
#define DMA_TIMEOUT_TICKS MSEC2TICK(DMA_TIMEOUT_MS)

/* Clocking *****************************************************************/

/* The board.h file may choose a different clock source for QUADSPI
 * peripherial by defining the BOARD_QSPI_CLK macro to one of the
 * RCC_D1CCIPR_QSPISEL_XXX values (XXX = HCLK, PLL1, PLL2, PER).
 * QUADSPI clock defaults to HCLK.
 */

#ifndef BOARD_QSPI_CLK
/* Clock QUADSPI from HCLK by default */

#  define BOARD_QSPI_CLK        RCC_D1CCIPR_QSPISEL_HCLK
#endif

/* The QSPI bit rate clock is generated by dividing the peripheral clock by
 * a value between 1 and 255.
 *
 * Find out the frequency of the QSPI clock.
 */

#if BOARD_QSPI_CLK == RCC_D1CCIPR_QSPISEL_HCLK
#  define QSPI_CLK_FREQUENCY    STM32_HCLK_FREQUENCY
#elif BOARD_QSPI_CLK == RCC_D1CCIPR_QSPISEL_PLL1
#  define QSPI_CLK_FREQUENCY    STM32_PLL1Q_FREQUENCY
#elif BOARD_QSPI_CLK == RCC_D1CCIPR_QSPISEL_PLL2
#  define QSPI_CLK_FREQUENCY    STM32_PLL2R_FREQUENCY
#elif BOARD_QSPI_CLK == RCC_D1CCIPR_QSPISEL_PER
#  define QSPI_CLK_FREQUENCY    STM32_PER_FREQUENCY
#else
#  error "BOARD_QSPI_CLK has unknown value!"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The state of the QSPI controller.
 *
 * NOTE: the STM32H7 supports only a single QSPI peripheral.  Logic here is
 * designed to support multiple QSPI peripherals.
 */

struct stm32h7_qspidev_s {
	struct qspi_dev_s qspi; /* Externally visible part of the QSPI interface */
	uint32_t base; /* QSPI controller register base address */
	uint32_t frequency; /* Requested clock frequency */
	uint32_t actual; /* Actual clock frequency */
	uint8_t mode; /* Mode 0,3 */
	uint8_t nbits; /* Width of word in bits (8 to 32) */
	uint8_t intf; /* QSPI controller number (0) */
	bool initialized; /* TRUE: Controller has been initialized */
	sem_t exclsem; /* Assures mutually exclusive access to QSPI */
	bool memmap; /* TRUE: Controller is in memory mapped mode */

#ifdef CONFIG_STM32H7_QSPI_INTERRUPTS
	xcpt_t handler; /* Interrupt handler */
	uint8_t irq; /* Interrupt number */
	sem_t op_sem; /* Block until complete */
	struct qspi_xctnspec_s *xctn; /* context of transaction in progress */
#endif

#ifdef CONFIG_STM32H7_QSPI_DMA
	bool candma; /* DMA is supported */
	sem_t dmawait; /* Used to wait for DMA completion */
	int result; /* DMA result */
	DMA_HANDLE dmach; /* QSPI DMA handle */
	WDOG_ID dmadog; /* Watchdog that handles DMA timeouts */
#endif

	/* Debug stuff */

#ifdef CONFIG_STM32H7_QSPI_DMADEBUG
	struct stm32h7_dmaregs_s dmaregs[DMA_NSAMPLES];
#endif

#ifdef CONFIG_STM32H7_QSPI_REGDEBUG
	bool wrlast; /* Last was a write */
	uint32_t addresslast; /* Last address */
	uint32_t valuelast; /* Last value */
	int ntimes; /* Number of times */
#endif
};

/* The QSPI transaction specification
 *
 * This is mostly the values of the CCR and DLR, AR, ABR, broken out into a C
 * structure  since these fields need to be considered at various phases of
 * thee transaction processing activity.
 */

struct qspi_xctnspec_s {
	uint8_t instrmode; /* 'instruction mode'; 0=none, 1=single, 2=dual, 3=quad */
	uint8_t instr; /* the (8-bit) Instruction (if any) */

	uint8_t addrmode; /* 'address mode'; 0=none, 1=single, 2=dual, 3=quad */
	uint8_t addrsize; /* address size (n - 1); 0, 1, 2, 3 */
	uint32_t addr; /* the address (if any) (1 to 4 bytes as per addrsize) */

	uint8_t altbytesmode; /* 'alt bytes mode'; 0=none, 1=single, 2=dual, 3=quad */
	uint8_t altbytessize; /* 'alt bytes' size (n - 1); 0, 1, 2, 3 */
	uint32_t altbytes; /* the 'alt bytes' (if any) */

	uint8_t dummycycles; /* number of Dummy Cycles; 0 - 32 */

	uint8_t datamode; /* 'data mode'; 0=none, 1=single, 2=dual, 3=quad */
	uint32_t datasize; /* number of data bytes (0xffffffff == undefined) */
	FAR void *buffer; /* Data buffer */

	uint8_t isddr; /* true if 'double data rate' */
	uint8_t issioo; /* true if 'send instruction only once' mode */

#ifdef CONFIG_STM32H7_QSPI_INTERRUPTS
	uint8_t function; /* functional mode; to distinguish a read or write */
	int8_t disposition; /* how it all turned out */
	uint32_t idxnow; /* index into databuffer of current byte in transfer */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

#ifdef CONFIG_STM32H7_QSPI_REGDEBUG
bool qspi_checkreg(struct stm32h7_qspidev_s *priv, bool wr,
		   uint32_t value, uint32_t address);
#else
# define        qspi_checkreg(priv,wr,value,address) (false)
#endif

inline uint32_t qspi_getreg(struct stm32h7_qspidev_s *priv, unsigned int offset);
static inline void qspi_putreg(struct stm32h7_qspidev_s *priv, uint32_t value,
			       unsigned int offset);

#ifdef CONFIG_DEBUG_SPI_INFO
QUADSPI_RAMFUNC void qspi_dumpregs(struct stm32h7_qspidev_s *priv,
				   const char *msg);
#else
# define        qspi_dumpregs(priv,msg)
#endif

#if defined(CONFIG_DEBUG_SPI_INFO) && defined(CONFIG_DEBUG_GPIO)
QUADSPI_RAMFUNC void qspi_dumpgpioconfig(const char *msg);
#else
# define        qspi_dumpgpioconfig(msg)
#endif

/* Interrupts */

#ifdef CONFIG_STM32H7_QSPI_INTERRUPTS
static int qspi0_interrupt(int irq, void *context, FAR void *arg);

#endif

/* DMA support */

#ifdef CONFIG_STM32H7_QSPI_DMA

#  ifdef CONFIG_STM32H7_QSPI_DMADEBUG
#    define qspi_dma_sample(s,i) stm32h7_dmasample((s)->dmach, &(s)->dmaregs[i])
QUADSPI_RAMFUNC void qspi_dma_sampleinit(struct stm32h7_qspidev_s *priv);
QUADSPI_RAMFUNC void qspi_dma_sampledone(struct stm32h7_qspidev_s *priv);
#  else
#    define qspi_dma_sample(s,i)
#    define qspi_dma_sampleinit(s)
#    define qspi_dma_sampledone(s)
#  endif

#  ifndef CONFIG_STM32H7_QSPI_DMATHRESHOLD
#    define CONFIG_STM32H7_QSPI_DMATHRESHOLD 4
#  endif

#endif

/* Initialization */

#if !defined(QSPI_BOOT_IN_MEMORY_MAPPED_MOD)
QUADSPI_RAMFUNC int qspi_hw_initialize(struct stm32h7_qspidev_s *priv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* QSPI0 driver operations */

static const struct qspi_ops_s g_qspi0ops = { .lock = qspi_lock, .setfrequency =
				qspi_setfrequency, .setmode = qspi_setmode, .setbits = qspi_setbits, .command =
						qspi_command, .memory = qspi_memory, .alloc = qspi_alloc, .free = qspi_free,
};

/* This is the overall state of the QSPI0 controller */

static struct stm32h7_qspidev_s g_qspi0dev = { .qspi = { .ops = &g_qspi0ops, },
	       .base = STM32_QUADSPI_BASE,
#ifdef CONFIG_STM32H7_QSPI_INTERRUPTS
	       handler = qspi0_interrupt,
	       .irq = STM32_IRQ_QUADSPI,
#endif
	       .intf = 0,
#ifdef CONFIG_STM32H7_QSPI_DMA
	       .candma = true,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qspi_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   value   - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   false: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_QSPI_REGDEBUG
QUADSPI_RAMFUNC bool qspi_checkreg(struct stm32h7_qspidev_s *priv, bool wr,
				   uint32_t value, uint32_t address)
{
	if (wr == priv->wrlast && /* Same kind of access? */
	    value == priv->valuelast && /* Same value? */
	    address == priv->addresslast) { /* Same address? */
		/* Yes, then just keep a count of the number of times we did this. */

		priv->ntimes++;
		return false;

	} else {
		/* Did we do the previous operation more than once? */

		if (priv->ntimes > 0) {
			/* Yes... show how many times we did it */

			spiinfo("...[Repeats %d times]...\n", priv->ntimes);
		}

		/* Save information about the new access */

		priv->wrlast = wr;
		priv->valuelast = value;
		priv->addresslast = address;
		priv->ntimes = 0;
	}

	/* Return true if this is the first time that we have done this operation */

	return true;
}
#endif

/****************************************************************************
 * Name: qspi_getreg
 *
 * Description:
 *  Read an QSPI register
 *
 ****************************************************************************/

inline uint32_t qspi_getreg(struct stm32h7_qspidev_s *priv, unsigned int offset)
{
	uint32_t address = priv->base + offset;
	uint32_t value = getreg32(address);

#ifdef CONFIG_STM32H7_QSPI_REGDEBUG

	if (qspi_checkreg(priv, false, value, address)) {
		spiinfo("%08x->%08x\n", address, value);
	}

#endif

	return value;
}

/****************************************************************************
 * Name: qspi_putreg
 *
 * Description:
 *  Write a value to an QSPI register
 *
 ****************************************************************************/

static inline void qspi_putreg(struct stm32h7_qspidev_s *priv, uint32_t value,
			       unsigned int offset)
{
	uint32_t address = priv->base + offset;

#ifdef CONFIG_STM32H7_QSPI_REGDEBUG

	if (qspi_checkreg(priv, true, value, address)) {
		spiinfo("%08x<-%08x\n", address, value);
	}

#endif

	putreg32(value, address);
}

/****************************************************************************
 * Name: qspi_dumpregs
 *
 * Description:
 *   Dump the contents of all QSPI registers
 *
 * Input Parameters:
 *   priv - The QSPI controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SPI_INFO
QUADSPI_RAMFUNC void qspi_dumpregs(struct stm32h7_qspidev_s *priv, const char *msg)
{
	uint32_t regval;
	spiinfo("%s:\n", msg);

#if 0
	/* this extra verbose output may be helpful in some cases; you'll need
	 * to make sure your syslog is large enough to accommodate extra output.
	 */

	regval = getreg32(priv->base + STM32_QUADSPI_CR_OFFSET); /* Control Register */
	spiinfo("CR:%08x\n", regval);
	spiinfo("  EN:%1d ABORT:%1d DMAEN:%1d TCEN:%1d SSHIFT:%1d\n"
		"  FTHRES: %d\n"
		"  TEIE:%1d TCIE:%1d FTIE:%1d SMIE:%1d TOIE:%1d APMS:%1d PMM:%1d\n"
		"  PRESCALER: %d\n",
		(regval & QSPI_CR_EN) ? 1 : 0,
		(regval & QSPI_CR_ABORT) ? 1 : 0,
		(regval & QSPI_CR_DMAEN) ? 1 : 0,
		(regval & QSPI_CR_TCEN) ? 1 : 0,
		(regval & QSPI_CR_SSHIFT) ? 1 : 0,
		(regval & QSPI_CR_FTHRES_MASK) >> QSPI_CR_FTHRES_SHIFT,
		(regval & QSPI_CR_TEIE) ? 1 : 0,
		(regval & QSPI_CR_TCIE) ? 1 : 0,
		(regval & QSPI_CR_FTIE) ? 1 : 0,
		(regval & QSPI_CR_SMIE) ? 1 : 0,
		(regval & QSPI_CR_TOIE) ? 1 : 0,
		(regval & QSPI_CR_APMS) ? 1 : 0,
		(regval & QSPI_CR_PMM) ? 1 : 0,
		(regval & QSPI_CR_PRESCALER_MASK) >> QSPI_CR_PRESCALER_SHIFT);

	regval = getreg32(priv->base + STM32_QUADSPI_DCR_OFFSET); /* Device Configuration Register */
	spiinfo("DCR:%08x\n", regval);
	spiinfo("  CKMODE:%1d CSHT:%d FSIZE:%d\n",
		(regval & QSPI_DCR_CKMODE) ? 1 : 0,
		(regval & QSPI_DCR_CSHT_MASK) >> QSPI_DCR_CSHT_SHIFT,
		(regval & QSPI_DCR_FSIZE_MASK) >> QSPI_DCR_FSIZE_SHIFT);

	regval = getreg32(priv->base + STM32_QUADSPI_CCR_OFFSET); /* Communication Configuration Register */
	spiinfo("CCR:%08x\n", regval);
	spiinfo("   INST:%02x IMODE:%d ADMODE:%d ADSIZE:%d ABMODE:%d\n"
		"   ABSIZE:%d DCYC:%d DMODE:%d FMODE:%d\n"
		"   SIOO:%1d DDRM:%1d\n",
		(regval & QSPI_CCR_INSTRUCTION_MASK) >> QSPI_CCR_INSTRUCTION_SHIFT,
		(regval & QSPI_CCR_IMODE_MASK) >> QSPI_CCR_IMODE_SHIFT,
		(regval & QSPI_CCR_ADMODE_MASK) >> QSPI_CCR_ADMODE_SHIFT,
		(regval & QSPI_CCR_ADSIZE_MASK) >> QSPI_CCR_ABSIZE_SHIFT,
		(regval & QSPI_CCR_ABMODE_MASK) >> QSPI_CCR_ABMODE_SHIFT,
		(regval & QSPI_CCR_ABSIZE_MASK) >> QSPI_CCR_ABSIZE_SHIFT,
		(regval & QSPI_CCR_DCYC_MASK) >> QSPI_CCR_DCYC_SHIFT,
		(regval & QSPI_CCR_DMODE_MASK) >> QSPI_CCR_DMODE_SHIFT,
		(regval & QSPI_CCR_FMODE_MASK) >> QSPI_CCR_FMODE_SHIFT,
		(regval & QSPI_CCR_SIOO) ? 1 : 0,
		(regval & QSPI_CCR_DDRM) ? 1 : 0);

	regval = getreg32(priv->base + STM32_QUADSPI_SR_OFFSET); /* Status Register */
	spiinfo("SR:%08x\n", regval);
	spiinfo("  TEF:%1d TCF:%1d FTF:%1d SMF:%1d TOF:%1d BUSY:%1d FLEVEL:%d\n",
		(regval & QSPI_SR_TEF) ? 1 : 0,
		(regval & QSPI_SR_TCF) ? 1 : 0,
		(regval & QSPI_SR_FTF) ? 1 : 0,
		(regval & QSPI_SR_SMF) ? 1 : 0,
		(regval & QSPI_SR_TOF) ? 1 : 0,
		(regval & QSPI_SR_BUSY) ? 1 : 0,
		(regval & QSPI_SR_FLEVEL_MASK) >> QSPI_SR_FLEVEL_SHIFT);

#else
	spiinfo("    CR:%08x   DCR:%08x   CCR:%08x    SR:%08x\n",
		getreg32(priv->base + STM32_QUADSPI_CR_OFFSET), /* Control Register */
		getreg32(priv->base + STM32_QUADSPI_DCR_OFFSET), /* Device Configuration Register */
		getreg32(priv->base + STM32_QUADSPI_CCR_OFFSET), /* Communication Configuration Register */
		getreg32(priv->base + STM32_QUADSPI_SR_OFFSET)); /* Status Register */
	spiinfo("   DLR:%08x   ABR:%08x PSMKR:%08x PSMAR:%08x\n",
		getreg32(priv->base + STM32_QUADSPI_DLR_OFFSET), /* Data Length Register */
		getreg32(priv->base + STM32_QUADSPI_ABR_OFFSET), /* Alternate Bytes Register */
		getreg32(priv->base + STM32_QUADSPI_PSMKR_OFFSET), /* Polling Status mask Register */
		getreg32(priv->base + STM32_QUADSPI_PSMAR_OFFSET)); /* Polling Status match Register */
	spiinfo("   PIR:%08x  LPTR:%08x\n",
		getreg32(priv->base + STM32_QUADSPI_PIR_OFFSET), /* Polling Interval Register */
		getreg32(priv->base + STM32_QUADSPI_LPTR_OFFSET)); /* Low-Power Timeout Register */
	(void)regval;
#endif
}
#endif

#if defined(CONFIG_DEBUG_SPI_INFO) && defined(CONFIG_DEBUG_GPIO)
QUADSPI_RAMFUNC void qspi_dumpgpioconfig(const char *msg)
{
	uint32_t regval;
	spiinfo("%s:\n", msg);

	/* Port B */

	regval = getreg32(STM32_GPIOB_MODER);
	spiinfo("B_MODER:%08x\n", regval);

	regval = getreg32(STM32_GPIOB_OTYPER);
	spiinfo("B_OTYPER:%08x\n", regval);

	regval = getreg32(STM32_GPIOB_OSPEED);
	spiinfo("B_OSPEED:%08x\n", regval);

	regval = getreg32(STM32_GPIOB_PUPDR);
	spiinfo("B_PUPDR:%08x\n", regval);

	regval = getreg32(STM32_GPIOB_AFRL);
	spiinfo("B_AFRL:%08x\n", regval);

	regval = getreg32(STM32_GPIOB_AFRH);
	spiinfo("B_AFRH:%08x\n", regval);

	/* Port D */

	regval = getreg32(STM32_GPIOD_MODER);
	spiinfo("D_MODER:%08x\n", regval);

	regval = getreg32(STM32_GPIOD_OTYPER);
	spiinfo("D_OTYPER:%08x\n", regval);

	regval = getreg32(STM32_GPIOD_OSPEED);
	spiinfo("D_OSPEED:%08x\n", regval);

	regval = getreg32(STM32_GPIOD_PUPDR);
	spiinfo("D_PUPDR:%08x\n", regval);

	regval = getreg32(STM32_GPIOD_AFRL);
	spiinfo("D_AFRL:%08x\n", regval);

	regval = getreg32(STM32_GPIOD_AFRH);
	spiinfo("D_AFRH:%08x\n", regval);

	/* Port E */

	regval = getreg32(STM32_GPIOE_MODER);
	spiinfo("E_MODER:%08x\n", regval);

	regval = getreg32(STM32_GPIOE_OTYPER);
	spiinfo("E_OTYPER:%08x\n", regval);

	regval = getreg32(STM32_GPIOE_OSPEED);
	spiinfo("E_OSPEED:%08x\n", regval);

	regval = getreg32(STM32_GPIOE_PUPDR);
	spiinfo("E_PUPDR:%08x\n", regval);

	regval = getreg32(STM32_GPIOE_AFRL);
	spiinfo("E_AFRL:%08x\n", regval);

	regval = getreg32(STM32_GPIOE_AFRH);
	spiinfo("E_AFRH:%08x\n", regval);
}
#endif

#ifdef CONFIG_STM32H7_QSPI_DMADEBUG
/****************************************************************************
 * Name: qspi_dma_sampleinit
 *
 * Description:
 *   Initialize sampling of DMA registers
 *
 * Input Parameters:
 *   priv - QSPI driver instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

QUADSPI_RAMFUNC void qspi_dma_sampleinit(struct stm32h7_qspidev_s *priv)
{
	/* Put contents of register samples into a known state */

	memset(priv->dmaregs, 0xff,
	       DMA_NSAMPLES * sizeof(struct stm32h7_dmaregs_s));

	/* Then get the initial samples */

	stm32h7_dmasample(priv->dmach, &priv->dmaregs[DMA_INITIAL]);
}

/****************************************************************************
 * Name: qspi_dma_sampledone
 *
 * Description:
 *   Dump sampled DMA registers
 *
 * Input Parameters:
 *   priv - QSPI driver instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

QUADSPI_RAMFUNC void qspi_dma_sampledone(struct stm32h7_qspidev_s *priv)
{
	/* Sample the final registers */

	stm32h7_dmasample(priv->dmach, &priv->dmaregs[DMA_END_TRANSFER]);

	/* Then dump the sampled DMA registers */

	/* Initial register values */

	stm32h7_dmadump(priv->dmach, &priv->dmaregs[DMA_INITIAL],
			"Initial Registers");

	/* Register values after DMA setup */

	stm32h7_dmadump(priv->dmach, &priv->dmaregs[DMA_AFTER_SETUP],
			"After DMA Setup");

	/* Register values after DMA start */

	stm32h7_dmadump(priv->dmach, &priv->dmaregs[DMA_AFTER_START],
			"After DMA Start");

	/* Register values at the time of the TX and RX DMA callbacks
	 * -OR- DMA timeout.
	 *
	 * If the DMA timed out, then there will not be any RX DMA
	 * callback samples.  There is probably no TX DMA callback
	 * samples either, but we don't know for sure.
	 */

	if (priv->result == -ETIMEDOUT) {
		stm32h7_dmadump(priv->dmach, &priv->dmaregs[DMA_TIMEOUT],
				"At DMA timeout");

	} else {
		stm32h7_dmadump(priv->dmach, &priv->dmaregs[DMA_CALLBACK],
				"At DMA callback");
	}

	stm32h7_dmadump(priv->dmach, &priv->dmaregs[DMA_END_TRANSFER],
			"At End-of-Transfer");
}
#endif

/****************************************************************************
 * Name: qspi_setupxctnfromcmd
 *
 * Description:
 *   Setup our transaction descriptor from a command info structure
 *
 * Input Parameters:
 *   xctn  - the transaction descriptor we setup
 *   cmdinfo  - the command info (originating from the MTD device)
 *
 * Returned Value:
 *   OK, or -errno if invalid
 *
 ****************************************************************************/

QUADSPI_RAMFUNC int qspi_setupxctnfromcmd(struct qspi_xctnspec_s *xctn,
		const struct qspi_cmdinfo_s *cmdinfo)
{
	DEBUGASSERT(xctn != NULL && cmdinfo != NULL);

#ifdef CONFIG_DEBUG_SPI_INFO
	spiinfo("Transfer:\n");
	spiinfo("  flags: %02x\n", cmdinfo->flags);
	spiinfo("  cmd: %04x\n", cmdinfo->cmd);

	if (QSPICMD_ISADDRESS(cmdinfo->flags)) {
		spiinfo("  address/length: %08lx/%d\n",
			(unsigned long)cmdinfo->addr, cmdinfo->addrlen);
	}

	if (QSPICMD_ISDATA(cmdinfo->flags)) {
		spiinfo("  %s Data:\n",
			QSPICMD_ISWRITE(cmdinfo->flags) ? "Write" : "Read");
		spiinfo("    buffer/length: %p/%d\n",
			cmdinfo->buffer, cmdinfo->buflen);
	}

#endif

	DEBUGASSERT(cmdinfo->cmd < 256);

	/* Specify the instruction as per command info */

	/* XXX III instruction mode, single dual quad option bits */

	if (QSPICMD_ISIQUAD(cmdinfo->flags)) {
		xctn->instrmode = CCR_IMODE_QUAD;

	} else if (QSPICMD_ISIDUAL(cmdinfo->flags)) {
		xctn->instrmode = CCR_IMODE_DUAL;

	} else {
		xctn->instrmode = CCR_IMODE_SINGLE;
	}

	xctn->instr = cmdinfo->cmd;

	/* XXX III option bits for 'send instruction only once' */

	xctn->issioo = 0;

	/* XXX III options for alt bytes, dummy cycles */

	xctn->altbytesmode = CCR_ABMODE_NONE;
	xctn->altbytessize = CCR_ABSIZE_8;
	xctn->altbytes = 0;
	xctn->dummycycles = 0;

	/* Specify the address size as needed */

	if (QSPICMD_ISADDRESS(cmdinfo->flags)) {
		/* XXX III address mode mode, single, dual, quad option bits */

		xctn->addrmode = CCR_ADMODE_SINGLE;

		if (cmdinfo->addrlen == 1) {
			xctn->addrsize = CCR_ADSIZE_8;

		} else if (cmdinfo->addrlen == 2) {
			xctn->addrsize = CCR_ADSIZE_16;

		} else if (cmdinfo->addrlen == 3) {
			xctn->addrsize = CCR_ADSIZE_24;

		} else if (cmdinfo->addrlen == 4) {
			xctn->addrsize = CCR_ADSIZE_32;

		} else {
			return -EINVAL;
		}

		xctn->addr = cmdinfo->addr;

	} else {
		xctn->addrmode = CCR_ADMODE_NONE;
		xctn->addrsize = 0;
		xctn->addr = cmdinfo->addr;
	}

	/* Specify the data as needed */

	xctn->buffer = cmdinfo->buffer;

	if (QSPICMD_ISDATA(cmdinfo->flags)) {
		/* XXX III data mode mode, single, dual, quad option bits */

		xctn->datamode = CCR_DMODE_SINGLE;
		xctn->datasize = cmdinfo->buflen;

		/* XXX III double data rate option bits */

		xctn->isddr = 0;

	} else {
		xctn->datamode = CCR_DMODE_NONE;
		xctn->datasize = 0;
		xctn->isddr = 0;
	}

#if defined(CONFIG_STM32H7_QSPI_INTERRUPTS)
	xctn->function = QSPICMD_ISWRITE(cmdinfo->flags) ? CCR_FMODE_INDWR :
			 CCR_FMODE_INDRD;
	xctn->disposition = - EIO;
	xctn->idxnow = 0;
#endif

	return OK;
}

/****************************************************************************
 * Name: qspi_setupxctnfrommem
 *
 * Description:
 *   Setup our transaction descriptor from a memory info structure
 *
 * Input Parameters:
 *   xctn  - the transaction descriptor we setup
 *   meminfo  - the memory info (originating from the MTD device)
 *
 * Returned Value:
 *   OK, or -errno if invalid
 *
 ****************************************************************************/

QUADSPI_RAMFUNC int qspi_setupxctnfrommem(struct qspi_xctnspec_s *xctn,
		const struct qspi_meminfo_s *meminfo)
{
	DEBUGASSERT(xctn != NULL && meminfo != NULL);

#ifdef CONFIG_DEBUG_SPI_INFO
	spiinfo("Transfer:\n");
	spiinfo("  flags: %02x\n", meminfo->flags);
	spiinfo("  cmd: %04x\n", meminfo->cmd);
	spiinfo("  address/length: %08lx/%d\n",
		(unsigned long)meminfo->addr, meminfo->addrlen);
	spiinfo("  %s Data:\n", QSPIMEM_ISWRITE(meminfo->flags) ?
		"Write" : "Read");
	spiinfo("    buffer/length: %p/%d\n", meminfo->buffer, meminfo->buflen);
#endif

	DEBUGASSERT(meminfo->cmd < 256);

	/* Specify the instruction as per command info */

	/* XXX III instruction mode, single dual quad option bits */

	if (QSPIMEM_ISIQUAD(meminfo->flags)) {
		xctn->instrmode = CCR_IMODE_QUAD;

	} else if (QSPIMEM_ISIDUAL(meminfo->flags)) {
		xctn->instrmode = CCR_IMODE_DUAL;

	} else {
		xctn->instrmode = CCR_IMODE_SINGLE;
	}

	xctn->instr = meminfo->cmd;

	/* XXX III option bits for 'send instruction only once' */

	xctn->issioo = 0;

	/* XXX III options for alt bytes */

	xctn->altbytesmode = CCR_ABMODE_NONE;
	xctn->altbytessize = CCR_ABSIZE_8;
	xctn->altbytes = 0;

	xctn->dummycycles = meminfo->dummies;

	/* Specify the address size as needed */

	/* XXX III there should be a separate flags for single/dual/quad for each
	 * of i,a,d
	 */

	if (QSPIMEM_ISDUALIO(meminfo->flags)) {
		xctn->addrmode = CCR_ADMODE_DUAL;

	} else if (QSPIMEM_ISQUADIO(meminfo->flags)) {
		xctn->addrmode = CCR_ADMODE_QUAD;

	} else {
		xctn->addrmode = CCR_ADMODE_SINGLE;
	}

	if (meminfo->addrlen == 1) {
		xctn->addrsize = CCR_ADSIZE_8;

	} else if (meminfo->addrlen == 2) {
		xctn->addrsize = CCR_ADSIZE_16;

	} else if (meminfo->addrlen == 3) {
		xctn->addrsize = CCR_ADSIZE_24;

	} else if (meminfo->addrlen == 4) {
		xctn->addrsize = CCR_ADSIZE_32;

	} else {
		return -EINVAL;
	}

	xctn->addr = meminfo->addr;

	/* Specify the data as needed */

	xctn->buffer = meminfo->buffer;

	/* XXX III there should be a separate flags for single/dual/quad for each
	 * of i,a,d
	 */

	if (QSPIMEM_ISDUALIO(meminfo->flags)) {
		xctn->datamode = CCR_DMODE_DUAL;

	} else if (QSPIMEM_ISQUADIO(meminfo->flags)) {
		xctn->datamode = CCR_DMODE_QUAD;

	} else {
		xctn->datamode = CCR_DMODE_SINGLE;
	}

	xctn->datasize = meminfo->buflen;

	/* XXX III double data rate option bits */

	xctn->isddr = 0;

#if defined(CONFIG_STM32H7_QSPI_INTERRUPTS)
	xctn->function = QSPIMEM_ISWRITE(meminfo->flags) ? CCR_FMODE_INDWR :
			 CCR_FMODE_INDRD;
	xctn->disposition = - EIO;
	xctn->idxnow = 0;
#endif

	return OK;
}

/****************************************************************************
 * Name: qspi_waitstatusflags
 *
 * Description:
 *   Spin wait for specified status flags to be set as desired
 *
 * Input Parameters:
 *   priv  - The QSPI controller to dump
 *   mask  - bits to check, can be multiple
 *   polarity - true wait if any set, false to wait if all reset
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

QUADSPI_RAMFUNC static void qspi_waitstatusflags(struct stm32h7_qspidev_s *priv,
		uint32_t mask, int polarity)
{
	uint32_t regval;

	if (polarity) {
		while (!((regval = qspi_getreg(priv, STM32_QUADSPI_SR_OFFSET)) & mask))
			;

	} else {
		while (((regval = qspi_getreg(priv, STM32_QUADSPI_SR_OFFSET)) & mask))
			;
	}
}

/****************************************************************************
 * Name: qspi_abort
 *
 * Description:
 *   Abort any transaction in progress
 *
 * Input Parameters:
 *   priv  - The QSPI controller to dump
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

QUADSPI_RAMFUNC static void qspi_abort(struct stm32h7_qspidev_s *priv)
{
	uint32_t regval;

	regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
	regval |= QSPI_CR_ABORT;
	qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);
}

/****************************************************************************
 * Name: qspi_ccrconfig
 *
 * Description:
 *   Do common Communications Configuration Register setup
 *
 * Input Parameters:
 *   priv  - The QSPI controller to dump
 *   xctn  - the transaction descriptor; CCR setup
 *   fctn  - 'functional mode'; 0=indwrite, 1=indread, 2=autopoll, 3=memmmap
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

QUADSPI_RAMFUNC static void qspi_ccrconfig(struct stm32h7_qspidev_s *priv,
		struct qspi_xctnspec_s *xctn, uint8_t fctn)
{
	uint32_t regval;

	/* If we have data, and it's not memory mapped, write the length */

	if (CCR_DMODE_NONE != xctn->datamode && CCR_FMODE_MEMMAP != fctn) {
		qspi_putreg(priv, xctn->datasize - 1, STM32_QUADSPI_DLR_OFFSET);
	}

	/* If we have alternate bytes, stick them in now */

	if (CCR_ABMODE_NONE != xctn->altbytesmode) {
		qspi_putreg(priv, xctn->altbytes, STM32_QUADSPI_ABR_OFFSET);
	}

	/* Build the CCR value and set it */

	regval = QSPI_CCR_INST(xctn->instr) | QSPI_CCR_IMODE(xctn->instrmode)
		 | QSPI_CCR_ADMODE(xctn->addrmode) | QSPI_CCR_ADSIZE(xctn->addrsize)
		 | QSPI_CCR_ABMODE(xctn->altbytesmode) | QSPI_CCR_ABSIZE(xctn->altbytessize)
		 | QSPI_CCR_DCYC(xctn->dummycycles) | QSPI_CCR_DMODE(xctn->datamode)
		 | QSPI_CCR_FMODE(fctn) | (xctn->isddr ? QSPI_CCR_SIOO : 0)
		 | (xctn->issioo ? QSPI_CCR_DDRM : 0);
	qspi_putreg(priv, regval, STM32_QUADSPI_CCR_OFFSET);

	/* If we have and need and address, set that now, too */

	if (CCR_ADMODE_NONE != xctn->addrmode && CCR_FMODE_MEMMAP != fctn) {
		qspi_putreg(priv, xctn->addr, STM32_QUADSPI_AR_OFFSET);
	}
}

#if defined(CONFIG_STM32H7_QSPI_INTERRUPTS)
/****************************************************************************
 * Name: qspi0_interrupt
 *
 * Description:
 *   Interrupt handler; we handle all QSPI cases -- reads, writes,
 *   automatic status polling, etc.
 *
 * Input Parameters:
 *   irq  -
 *   context  -
 *
 * Returned Value:
 *   OK means we handled it
 *
 ****************************************************************************/

static int qspi0_interrupt(int irq, void *context, FAR void *arg)
{
	uint32_t status;
	uint32_t cr;
	uint32_t regval;

	/* Let's find out what is going on */

	status = qspi_getreg(&g_qspi0dev, STM32_QUADSPI_SR_OFFSET);
	cr = qspi_getreg(&g_qspi0dev, STM32_QUADSPI_CR_OFFSET);

	/* Is it 'FIFO Threshold'? */

	if ((status & QSPI_SR_FTF) && (cr & QSPI_CR_FTIE)) {
		volatile uint32_t *datareg =
			(volatile uint32_t *)(g_qspi0dev.base + STM32_QUADSPI_DR_OFFSET);

		if (g_qspi0dev.xctn->function == CCR_FMODE_INDWR) {
			/* Write data until we have no more or have no place to put it */

			while (((regval = qspi_getreg(
						  &g_qspi0dev, STM32_QUADSPI_SR_OFFSET)) & QSPI_SR_FTF) != 0) {
				if (g_qspi0dev.xctn->idxnow < g_qspi0dev.xctn->datasize) {
					*(volatile uint8_t *)datareg =
						((uint8_t *)g_qspi0dev.xctn->buffer)
						[g_qspi0dev.xctn->idxnow];
					++g_qspi0dev.xctn->idxnow;

				} else {
					/* Fresh out of data to write */

					break;
				}
			}

		} else if (g_qspi0dev.xctn->function == CCR_FMODE_INDRD) {
			/* Read data until we have no more or have no place to put it */

			while (((regval = qspi_getreg(
						  &g_qspi0dev, STM32_QUADSPI_SR_OFFSET)) & QSPI_SR_FTF) != 0) {
				if (g_qspi0dev.xctn->idxnow < g_qspi0dev.xctn->datasize) {
					((uint8_t *)g_qspi0dev.xctn->buffer)
					[g_qspi0dev.xctn->idxnow] = *(volatile uint8_t *)datareg;
					++g_qspi0dev.xctn->idxnow;

				} else {
					/* no room at the inn */

					break;
				}
			}
		}
	}

	/* Is it 'Transfer Complete'? */

	if ((status & QSPI_SR_TCF) && (cr & QSPI_CR_TCIE)) {
		/* Acknowledge interrupt */

		qspi_putreg(&g_qspi0dev, QSPI_FCR_CTCF, STM32_QUADSPI_FCR_OFFSET);

		/* Disable the QSPI FIFO Threshold, Transfer Error and Transfer
		 * complete Interrupts
		 */

		regval = qspi_getreg(&g_qspi0dev, STM32_QUADSPI_CR_OFFSET);
		regval &= ~(QSPI_CR_TEIE | QSPI_CR_TCIE | QSPI_CR_FTIE);
		qspi_putreg(&g_qspi0dev, regval, STM32_QUADSPI_CR_OFFSET);

		/* Do the last bit of read if needed */

		if (g_qspi0dev.xctn->function == CCR_FMODE_INDRD) {
			volatile uint32_t *datareg =
				(volatile uint32_t *)(g_qspi0dev.base + STM32_QUADSPI_DR_OFFSET);

			/* Read any remaining data */

			while (((regval = qspi_getreg(
						  &g_qspi0dev, STM32_QUADSPI_SR_OFFSET)) &
				QSPI_SR_FLEVEL_MASK) != 0) {
				if (g_qspi0dev.xctn->idxnow < g_qspi0dev.xctn->datasize) {
					((uint8_t *)g_qspi0dev.xctn->buffer)
					[g_qspi0dev.xctn->idxnow] = *(volatile uint8_t *)datareg;
					++g_qspi0dev.xctn->idxnow;

				} else {
					/* No room at the inn */

					break;
				}
			}
		}

		/* Use 'abort' to ditch any stray fifo contents and clear BUSY flag */

		qspi_abort(&g_qspi0dev);

		/* Set success status */

		g_qspi0dev.xctn->disposition = OK;

		/* Signal complete */

		nxsem_post(&g_qspi0dev.op_sem);
	}

	/* Is it 'Status Match'? */

	if ((status & QSPI_SR_SMF) && (cr & QSPI_CR_SMIE)) {
		/* Acknowledge interrupt */

		qspi_putreg(&g_qspi0dev, QSPI_FCR_CSMF, STM32_QUADSPI_FCR_OFFSET);

		/* If 'automatic poll mode stop' is activated, we're done */

		if (cr & QSPI_CR_APMS) {
			/* Disable the QSPI Transfer Error and Status Match Interrupts */

			regval = qspi_getreg(&g_qspi0dev, STM32_QUADSPI_CR_OFFSET);
			regval &= ~(QSPI_CR_TEIE | QSPI_CR_SMIE);
			qspi_putreg(&g_qspi0dev, regval, STM32_QUADSPI_CR_OFFSET);

			/* Set success status */

			g_qspi0dev.xctn->disposition = OK;

			/* Signal complete */

			nxsem_post(&g_qspi0dev.op_sem);

		} else {
			/* XXX if it's NOT auto stop; something needs to happen here;
			 * a callback?
			 */
		}
	}

	/* Is it' Transfer Error'? :( */

	if ((status & QSPI_SR_TEF) && (cr & QSPI_CR_TEIE)) {
		/* Acknowledge interrupt */

		qspi_putreg(&g_qspi0dev, QSPI_FCR_CTEF, STM32_QUADSPI_FCR_OFFSET);

		/* Disable all the QSPI Interrupts */

		regval = qspi_getreg(&g_qspi0dev, STM32_QUADSPI_CR_OFFSET);
		regval &= ~(QSPI_CR_TEIE | QSPI_CR_TCIE | QSPI_CR_FTIE |
			    QSPI_CR_SMIE | QSPI_CR_TOIE);
		qspi_putreg(&g_qspi0dev, regval, STM32_QUADSPI_CR_OFFSET);

		/* Set error status; 'transfer error' means that, in 'indirect mode',
		 * an invalid address is attempted to be accessed.  'Invalid' is
		 * presumably relative to the FSIZE field in CCR; the manual is not
		 * explicit, but what else could it be?
		 */

		g_qspi0dev.xctn->disposition = - EIO;

		/* Signal complete */

		nxsem_post(&g_qspi0dev.op_sem);
	}

	/* Is it 'Timeout'? */

	if ((status & QSPI_SR_TOF) && (cr & QSPI_CR_TOIE)) {
		/* Acknowledge interrupt */

		qspi_putreg(&g_qspi0dev, QSPI_FCR_CTOF, STM32_QUADSPI_FCR_OFFSET);

		/* XXX this interrupt simply means that, in 'memory mapped mode',
		 * the QSPI memory has not been accessed for a while, and the
		 * IP block was configured to automatically de-assert CS after
		 * a timeout.  And now we're being informed that has happened.
		 *
		 * But who cares?  If someone does, perhaps a user callback is
		 * appropriate, or some signal?  Either way, realize the xctn
		 * member is /not/ valid, so you can't set the disposition
		 * field.  Also, note signaling completion has no meaning here
		 * because in memory mapped mode no one holds the semaphore.
		 */
	}

	return OK;
}

#elif defined(CONFIG_STM32H7_QSPI_DMA)
/****************************************************************************
 * Name: qspi_dma_timeout
 *
 * Description:
 *   The watchdog timeout setup when a has expired without completion of a
 *   DMA.
 *
 * Input Parameters:
 *   argc   - The number of arguments (should be 1)
 *   arg    - The argument (state structure reference cast to uint32_t)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

QUADSPI_RAMFUNC void qspi_dma_timeout(int argc, uint32_t arg, ...)
{
	struct stm32h7_qspidev_s *priv = (struct stm32h7_qspidev_s *)arg;
	DEBUGASSERT(priv != NULL);

	/* Sample DMA registers at the time of the timeout */

	qspi_dma_sample(priv, DMA_CALLBACK);

	/* Report timeout result, perhaps overwriting any failure reports from
	 * the TX callback.
	 */

	priv->result = -ETIMEDOUT;

	/* Then wake up the waiting thread */

	nxsem_post(&priv->dmawait);
}

/****************************************************************************
 * Name: qspi_dma_callback
 *
 * Description:
 *   This callback function is invoked at the completion of the QSPI DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   isr - source of the DMA interrupt
 *   arg - A pointer to the chip select structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

QUADSPI_RAMFUNC void qspi_dma_callback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
	struct stm32h7_qspidev_s *priv = (struct stm32h7_qspidev_s *)arg;
	DEBUGASSERT(priv != NULL);

	/* Cancel the watchdog timeout */

	wd_cancel(priv->dmadog);

	/* Sample DMA registers at the time of the callback */

	qspi_dma_sample(priv, DMA_CALLBACK);

	/* Report the result of the transfer only if the callback has not already
	 * reported an error.
	 */

	if (priv->result == -EBUSY) {
		/* Save the result of the transfer if no error was previously
		 * reported
		 */

		if (isr & DMA_STREAM_TCIF_BIT) {
			priv->result = OK;

		} else if (isr & DMA_STREAM_TEIF_BIT) {
			priv->result = -EIO;

		} else {
			priv->result = OK;
		}
	}

	/* Then wake up the waiting thread */

	nxsem_post(&priv->dmawait);
}

/****************************************************************************
 * Name: qspi_regaddr
 *
 * Description:
 *   Return the address of an QSPI register
 *
 ****************************************************************************/

static inline uintptr_t qspi_regaddr(struct stm32h7_qspidev_s *priv,
				     unsigned int offset)
{
	return priv->base + offset;
}

/****************************************************************************
 * Name: qspi_memory_dma
 *
 * Description:
 *   Perform one QSPI memory transfer using DMA
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   meminfo - Describes the memory transfer to be performed.
 *   xctn    - Describes the transaction context.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

QUADSPI_RAMFUNC int qspi_memory_dma(struct stm32h7_qspidev_s *priv,
				    struct qspi_meminfo_s *meminfo,
				    struct qspi_xctnspec_s *xctn)
{
	uint32_t dmaflags;
	uint32_t regval;
	int ret;

	/* Initialize register sampling */

	qspi_dma_sampleinit(priv);

	/* Determine DMA flags and setup the DMA */

	if (QSPIMEM_ISWRITE(meminfo->flags)) {
		/* Setup the DMA (memory-to-peripheral) */

		dmaflags = (QSPI_DMA_PRIO | DMA_SCR_MSIZE_8BITS |
			    DMA_SCR_PSIZE_8BITS | DMA_SCR_MINC | DMA_SCR_DIR_M2P);

		up_clean_dcache((uintptr_t)meminfo->buffer,
				(uintptr_t)meminfo->buffer + meminfo->buflen);

	} else {
		/* Setup the DMA (peripheral-to-memory) */

		dmaflags = (QSPI_DMA_PRIO | DMA_SCR_MSIZE_8BITS |
			    DMA_SCR_PSIZE_8BITS | DMA_SCR_MINC | DMA_SCR_DIR_P2M);
	}

	stm32_dmasetup(priv->dmach, qspi_regaddr(priv, STM32_QUADSPI_DR_OFFSET),
		       (uint32_t)meminfo->buffer, meminfo->buflen, dmaflags);

	qspi_dma_sample(priv, DMA_AFTER_SETUP);

	/* Enable the memory transfer */

	regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
	regval |= QSPI_CR_DMAEN;
	qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);

	/* Set up the Communications Configuration Register as per command info */

	qspi_ccrconfig(priv, xctn,
		       QSPIMEM_ISWRITE(meminfo->flags) ? CCR_FMODE_INDWR :
		       CCR_FMODE_INDRD);

	/* Start the DMA */

	priv->result = -EBUSY;
	stm32_dmastart(priv->dmach, qspi_dma_callback, priv, false);

	qspi_dma_sample(priv, DMA_AFTER_START);

	/* Wait for DMA completion.  This is done in a loop because there may be
	 * false alarm semaphore counts that cause nxsem_wait() not fail to wait
	 * or to wake-up prematurely (for example due to the receipt of a signal).
	 * We know that the DMA has completed when the result is anything other
	 * that -EBUSY.
	 */

	do {
		/* Start (or re-start) the watchdog timeout */

		ret = wd_start(priv->dmadog, DMA_TIMEOUT_TICKS,
			       qspi_dma_timeout, 1, (uint32_t)priv);

		if (ret < 0) {
			spierr("ERROR: wd_start failed: %d\n", ret);
		}

		/* Wait for the DMA complete */

		ret = nxsem_wait(&priv->dmawait);

		if (QSPIMEM_ISREAD(meminfo->flags)) {
			up_invalidate_dcache((uintptr_t)meminfo->buffer,
					     (uintptr_t)meminfo->buffer + meminfo->buflen);
		}

		/* Cancel the watchdog timeout */

		wd_cancel(priv->dmadog);

		/* Check if we were awakened by an error of some kind */

		if (ret < 0) {
			/* EINTR is not a failure.  That simply means that the wait
			 * was awakened by a signal.
			 */

			if (ret != -EINTR) {
				DEBUGPANIC();
				regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
				regval &= ~QSPI_CR_DMAEN;
				qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);
				return ret;
			}
		}

		/* Note that we might be awakened before the wait is over due to
		 * residual counts on the semaphore.  So, to handle, that case,
		 * we loop until something changes the DMA result to any value other
		 * than -EBUSY.
		 */
	} while (priv->result == -EBUSY);

	/* Wait for Transfer complete, and not busy */

	qspi_waitstatusflags(priv, QSPI_SR_TCF, 1);
	qspi_waitstatusflags(priv, QSPI_SR_BUSY, 0);
	MEMORY_SYNC();

	/* Dump the sampled DMA registers */

	qspi_dma_sampledone(priv);

	/* Make sure that the DMA is stopped (it will be stopped automatically
	 * on normal transfers, but not necessarily when the transfer terminates
	 * on an error condition).
	 */

	stm32_dmastop(priv->dmach);

	regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
	regval &= ~QSPI_CR_DMAEN;
	qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);

	/* Complain if the DMA fails */

	if (priv->result) {
		spierr("ERROR: DMA failed with result: %d\n", priv->result);
	}

	return priv->result;
}
#endif

#if !defined(CONFIG_STM32H7_QSPI_INTERRUPTS)
/****************************************************************************
 * Name: qspi_receive_blocking
 *
 * Description:
 *   Do common data receive in a blocking (status polling) way
 *
 * Input Parameters:
 *   priv  - The QSPI controller to dump
 *   xctn  - the transaction descriptor
 *
 * Returned Value:
 *   OK, or -errno on error
 *
 ****************************************************************************/

QUADSPI_RAMFUNC int qspi_receive_blocking(struct stm32h7_qspidev_s *priv,
		struct qspi_xctnspec_s *xctn)
{
	int ret = OK;
	volatile uint32_t *datareg = (volatile uint32_t *)(priv->base
				     + STM32_QUADSPI_DR_OFFSET);
	uint8_t *dest = (uint8_t *) xctn->buffer;
	uint32_t addrval;
	uint32_t regval;

	addrval = qspi_getreg(priv, STM32_QUADSPI_AR_OFFSET);

	if (dest != NULL) {
		/* Counter of remaining data */

		uint32_t remaining = xctn->datasize;

		/* Ensure CCR register specifies indirect read */

		regval = qspi_getreg(priv, STM32_QUADSPI_CCR_OFFSET);
		regval &= ~QSPI_CCR_FMODE_MASK;
		regval |= QSPI_CCR_FMODE(CCR_FMODE_INDRD);
		qspi_putreg(priv, regval, STM32_QUADSPI_CCR_OFFSET);

		/* Start the transfer by re-writing the address in AR register */

		qspi_putreg(priv, addrval, STM32_QUADSPI_AR_OFFSET);

		/* Transfer loop */

		while (remaining > 0) {
			/* Wait for Fifo Threshold, or Transfer Complete, to read data */

			qspi_waitstatusflags(priv, QSPI_SR_FTF | QSPI_SR_TCF, 1);

			*dest = *(volatile uint8_t *) datareg;
			dest++;
			remaining--;
		}

		if (ret == OK) {
			/* Wait for transfer complete, then clear it */

			qspi_waitstatusflags(priv, QSPI_SR_TCF, 1);
			qspi_putreg(priv, QSPI_FCR_CTCF, STM32_QUADSPI_FCR_OFFSET);

			/* Use Abort to clear the busy flag, and ditch any extra bytes in
			 * fifo
			 */

			qspi_abort(priv);
		}

	} else {
		ret = -EINVAL;
	}

	return ret;
}

/****************************************************************************
 * Name: qspi_transmit_blocking
 *
 * Description:
 *   Do common data transmit in a blocking (status polling) way
 *
 * Input Parameters:
 *   priv  - The QSPI controller to dump
 *   xctn  - the transaction descriptor
 *
 * Returned Value:
 *   OK, or -errno on error
 *
 ****************************************************************************/

QUADSPI_RAMFUNC int qspi_transmit_blocking(struct stm32h7_qspidev_s *priv,
		struct qspi_xctnspec_s *xctn)
{
	int ret = OK;
	volatile uint32_t *datareg = (volatile uint32_t *)(priv->base
				     + STM32_QUADSPI_DR_OFFSET);
	uint8_t *src = (uint8_t *) xctn->buffer;

	if (src != NULL) {
		/* Counter of remaining data */

		uint32_t remaining = xctn->datasize;

		/* Transfer loop */

		while (remaining > 0) {
			/* Wait for Fifo Threshold to write data */

			qspi_waitstatusflags(priv, QSPI_SR_FTF, 1);

			*(volatile uint8_t *) datareg = *src++;
			remaining--;
		}

		if (ret == OK) {
			/* Wait for transfer complete, then clear it */

			qspi_waitstatusflags(priv, QSPI_SR_TCF, 1);
			qspi_putreg(priv, QSPI_FCR_CTCF, STM32_QUADSPI_FCR_OFFSET);

			/* Use Abort to clear the Busy flag */

			qspi_abort(priv);
		}

	} else {
		ret = -EINVAL;
	}

	return ret;
}

#endif

/****************************************************************************
 * Name: qspi_lock
 *
 * Description:
 *   On QSPI buses where there are multiple devices, it will be necessary to
 *   lock QSPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the QSPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the QSPI is properly
 *   configured for the device.  If the QSPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock QSPI bus, false: unlock QSPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

QUADSPI_RAMFUNC int qspi_lock(struct qspi_dev_s *dev, bool lock)
{
#if 0 //semaphore can't work as ramfunc
	struct stm32h7_qspidev_s *priv = (struct stm32h7_qspidev_s *)dev;
	int ret;

	spiinfo("lock=%d\n", lock);

	if (lock) {
		/* Take the semaphore (perhaps waiting) */

		do {
			ret = nxsem_wait(&priv->exclsem);

			/* The only case that an error should occur here is if the wait
			 * was awakened by a signal.
			 */

			DEBUGASSERT(ret == OK || ret == -EINTR);
		} while (ret == -EINTR);

	} else {
		ret = nxsem_post(&priv->exclsem);
	}

	return ret;
#endif
	return 0;
}

/****************************************************************************
 * Name: qspi_setfrequency
 *
 * Description:
 *   Set the QSPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The QSPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

QUADSPI_RAMFUNC uint32_t qspi_setfrequency(struct qspi_dev_s *dev,
		uint32_t frequency)
{
	struct stm32h7_qspidev_s *priv = (struct stm32h7_qspidev_s *) dev;
	uint32_t actual;
	uint32_t prescaler;
	uint32_t regval;

	if (priv->memmap) {
		/* XXX we have no better return here, but the caller will find out
		 * in their subsequent calls.
		 */

		return 0;
	}

	spiinfo("frequency=%d\n", frequency);
	DEBUGASSERT(priv);

	/* Wait till BUSY flag reset */

	qspi_abort(priv);
	qspi_waitstatusflags(priv, QSPI_SR_BUSY, 0);

	/* Check if the requested frequency is the same as the frequency
	 * selection
	 */

	if (priv->frequency == frequency) {
		/* We are already at this frequency.  Return the actual. */

		return priv->actual;
	}

	/* Configure QSPI to a frequency as close as possible to the requested
	 * frequency.
	 *
	 *   QSCK frequency = QSPI_CLK_FREQUENCY / prescaler, or
	 *     prescaler = QSPI_CLK_FREQUENCY / frequency
	 *
	 * Where prescaler can have the range 1 to 256 and the
	 * STM32_QUADSPI_CR_OFFSET register field holds prescaler - 1.
	 * NOTE that a "ceiling" type of calculation is performed.
	 * 'frequency' is treated as a not-to-exceed value.
	 */

	prescaler = (frequency + QSPI_CLK_FREQUENCY - 1) / frequency;

	/* Make sure that the divider is within range */

	if (prescaler < 1) {
		prescaler = 1;

	} else if (prescaler > 256) {
		prescaler = 256;
	}

	/* Save the new prescaler value (minus one) */

	regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
	regval &= ~(QSPI_CR_PRESCALER_MASK);
	regval |= (prescaler - 1) << QSPI_CR_PRESCALER_SHIFT;
	qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);

	/* Calculate the new actual frequency */

	actual = QSPI_CLK_FREQUENCY / prescaler;
	spiinfo("prescaler=%d actual=%d\n", prescaler, actual);

	/* Save the frequency setting */

	priv->frequency = frequency;
	priv->actual = actual;

	spiinfo("Frequency %d->%d\n", frequency, actual);
	return actual;
}

/****************************************************************************
 * Name: qspi_setmode
 *
 * Description:
 *   Set the QSPI mode. Optional.  See enum qspi_mode_e for mode definitions.
 *   NOTE:  the STM32H7 QSPI supports only modes 0 and 3.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The QSPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

QUADSPI_RAMFUNC void qspi_setmode(struct qspi_dev_s *dev, enum qspi_mode_e mode)
{
	struct stm32h7_qspidev_s *priv = (struct stm32h7_qspidev_s *) dev;
	uint32_t regval;

	if (priv->memmap) {
		/* XXX we have no better return here, but the caller will find out
		 * in their subsequent calls.
		 */

		return;
	}

	spiinfo("mode=%d\n", mode);

	/* Has the mode changed? */

	if (mode != priv->mode) {
		/* Yes... Set the mode appropriately:
		 *
		 * QSPI  CPOL CPHA
		 * MODE
		 *  0    0    0
		 *  1    0    1
		 *  2    1    0
		 *  3    1    1
		 */

		regval = qspi_getreg(priv, STM32_QUADSPI_DCR_OFFSET);
		regval &= ~(QSPI_DCR_CKMODE);

		switch (mode) {
		case QSPIDEV_MODE0: /* CPOL=0; CPHA=0 */
			break;

		case QSPIDEV_MODE3: /* CPOL=1; CPHA=1 */
			regval |= (QSPI_DCR_CKMODE);
			break;

		case QSPIDEV_MODE1: /* CPOL=0; CPHA=1 */
		case QSPIDEV_MODE2: /* CPOL=1; CPHA=0 */
			spiinfo("unsupported mode=%d\n", mode);

		default:
			DEBUGASSERT(FALSE);
			return;
		}

		qspi_putreg(priv, regval, STM32_QUADSPI_DCR_OFFSET);
		spiinfo("DCR=%08x\n", regval);

		/* Save the mode so that subsequent re-configurations will be faster */

		priv->mode = mode;
	}
}

/****************************************************************************
 * Name: qspi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *   NOTE:  the STM32H7 QSPI only supports 8 bits, so this does nothing.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

QUADSPI_RAMFUNC void qspi_setbits(struct qspi_dev_s *dev, int nbits)
{
	/* Not meaningful for the STM32H7x6 */

	if (8 != nbits) {
		spiinfo("unsupported nbits=%d\n", nbits);
		DEBUGASSERT(FALSE);
	}
}

/****************************************************************************
 * Name: qspi_command
 *
 * Description:
 *   Perform one QSPI data transfer
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   cmdinfo - Describes the command transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

QUADSPI_RAMFUNC int qspi_command(struct qspi_dev_s *dev,
				 struct qspi_cmdinfo_s *cmdinfo)
{
	struct stm32h7_qspidev_s *priv = (struct stm32h7_qspidev_s *) dev;
	struct qspi_xctnspec_s xctn;
	int ret;

	/* Reject commands issued while in memory mapped mode, which will
	 * automatically cancel the memory mapping.  You must exit the
	 * memory mapped mode first.
	 */

	if (priv->memmap) {
		return -EBUSY;
	}

	/* Set up the transaction descriptor as per command info */

	ret = qspi_setupxctnfromcmd(&xctn, cmdinfo);

	if (OK != ret) {
		return ret;
	}

	/* Prepare for transaction */

	/* Wait 'till non-busy */

	qspi_abort(priv);
	qspi_waitstatusflags(priv, QSPI_SR_BUSY, 0);

	/* Clear flags */

	qspi_putreg(priv,
		    QSPI_FCR_CTEF | QSPI_FCR_CTCF | QSPI_FCR_CSMF | QSPI_FCR_CTOF,
		    STM32_QUADSPI_FCR_OFFSET);

#ifdef CONFIG_STM32H7_QSPI_INTERRUPTS
	/* interrupt mode will need access to the transaction context */

	priv->xctn = &xctn;

	if (QSPICMD_ISDATA(cmdinfo->flags)) {
		DEBUGASSERT(cmdinfo->buffer != NULL && cmdinfo->buflen > 0);
		DEBUGASSERT(IS_ALIGNED(cmdinfo->buffer));

		if (QSPICMD_ISWRITE(cmdinfo->flags)) {
			uint32_t regval;

			/* Set up the Communications Configuration Register as per command
			 * info
			 */

			qspi_ccrconfig(priv, &xctn, CCR_FMODE_INDWR);

			/* Enable 'Transfer Error' 'FIFO Threshhold' and
			 * 'Transfer Complete' interrupts.
			 */

			regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
			regval |= (QSPI_CR_TEIE | QSPI_CR_FTIE | QSPI_CR_TCIE);
			qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);

		} else {
			uint32_t regval;
			uint32_t addrval;

			addrval = qspi_getreg(priv, STM32_QUADSPI_AR_OFFSET);

			/* Set up the Communications Configuration Register as per command
			 * info
			 */

			qspi_ccrconfig(priv, &xctn, CCR_FMODE_INDRD);

			/* Start the transfer by re-writing the address in AR register */

			qspi_putreg(priv, addrval, STM32_QUADSPI_AR_OFFSET);

			/* Enable 'Transfer Error' 'FIFO Threshhold' and
			 * 'Transfer Complete' interrupts
			 */

			regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
			regval |= (QSPI_CR_TEIE | QSPI_CR_FTIE | QSPI_CR_TCIE);
			qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);
		}

	} else {
		uint32_t regval;

		/* We have no data phase, the command will execute as soon as we emit
		 * the CCR
		 */

		/* Enable 'Transfer Error' and 'Transfer Complete' interrupts */

		regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
		regval |= (QSPI_CR_TEIE | QSPI_CR_TCIE);
		qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);

		/* Set up the Communications Configuration Register as per command
		 * info
		 */

		qspi_ccrconfig(priv, &xctn, CCR_FMODE_INDRD);
	}

	/* Wait for the interrupt routine to finish it's magic */

	nxsem_wait(&priv->op_sem);
	MEMORY_SYNC();

	/* Convey the result */

	ret = xctn.disposition;

	/* because command transfers are so small, we're not going to use
	 * DMA for them, only interrupts or polling
	 */

#else
	/* Polling mode */

	/* Set up the Communications Configuration Register as per command info */

	qspi_ccrconfig(priv, &xctn, CCR_FMODE_INDWR);

	/* That may be it, unless there is also data to transfer */

	if (QSPICMD_ISDATA(cmdinfo->flags)) {
		DEBUGASSERT(cmdinfo->buffer != NULL && cmdinfo->buflen > 0);
		DEBUGASSERT(IS_ALIGNED(cmdinfo->buffer));

		if (QSPICMD_ISWRITE(cmdinfo->flags)) {
			ret = qspi_transmit_blocking(priv, &xctn);

		} else {
			ret = qspi_receive_blocking(priv, &xctn);
		}

		MEMORY_SYNC()
		;

	} else {
		ret = OK;
	}

	/* Wait for Transfer complete, and not busy
	 if (QSPICMD_ISDATA(cmdinfo->flags))
	 {
	 qspi_waitstatusflags(priv, QSPI_SR_TCF, 1);
	 }
	 */
	qspi_waitstatusflags(priv, QSPI_SR_BUSY, 0);

#endif

	return ret;
}

/****************************************************************************
 * Name: qspi_memory
 *
 * Description:
 *   Perform one QSPI memory transfer
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   meminfo - Describes the memory transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

QUADSPI_RAMFUNC int qspi_memory(struct qspi_dev_s *dev,
				struct qspi_meminfo_s *meminfo)
{
	struct stm32h7_qspidev_s *priv = (struct stm32h7_qspidev_s *) dev;
	struct qspi_xctnspec_s xctn;
	int ret;

	/* Reject commands issued while in memory mapped mode, which will
	 * automatically cancel the memory mapping.  You must exit the
	 * memory mapped mode first.
	 */

	if (priv->memmap) {
		return -EBUSY;
	}

	/* Set up the transaction descriptor as per command info */

	ret = qspi_setupxctnfrommem(&xctn, meminfo);

	if (OK != ret) {
		return ret;
	}

	/* Prepare for transaction */

	/* Wait 'till non-busy */

	qspi_abort(priv);
	qspi_waitstatusflags(priv, QSPI_SR_BUSY, 0);

	/* Clear flags */

	qspi_putreg(priv,
		    QSPI_FCR_CTEF | QSPI_FCR_CTCF | QSPI_FCR_CSMF | QSPI_FCR_CTOF,
		    STM32_QUADSPI_FCR_OFFSET);

#ifdef CONFIG_STM32H7_QSPI_INTERRUPTS
	/* interrupt mode will need access to the transaction context */

	priv->xctn = &xctn;

	DEBUGASSERT(meminfo->buffer != NULL && meminfo->buflen > 0);
	DEBUGASSERT(IS_ALIGNED(meminfo->buffer));

	if (QSPIMEM_ISWRITE(meminfo->flags)) {
		uint32_t regval;

		/* Set up the Communications Configuration Register as per command
		 * info
		 */

		qspi_ccrconfig(priv, &xctn, CCR_FMODE_INDWR);

		/* Enable 'Transfer Error' 'FIFO Threshhold' and 'Transfer Complete'
		 * interrupts
		 */

		regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
		regval |= (QSPI_CR_TEIE | QSPI_CR_FTIE | QSPI_CR_TCIE);
		qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);

	} else {
		uint32_t regval;
		uint32_t addrval;

		addrval = qspi_getreg(priv, STM32_QUADSPI_AR_OFFSET);

		/* Set up the Communications Configuration Register as per command
		 * info
		 */

		qspi_ccrconfig(priv, &xctn, CCR_FMODE_INDRD);

		/* Start the transfer by re-writing the address in AR register */

		qspi_putreg(priv, addrval, STM32_QUADSPI_AR_OFFSET);

		/* Enable 'Transfer Error' 'FIFO Threshhold' and 'Transfer Complete'
		 * interrupts
		 */

		regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
		regval |= (QSPI_CR_TEIE | QSPI_CR_FTIE | QSPI_CR_TCIE);
		qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);
	}

	/* Wait for the interrupt routine to finish it's magic */

	nxsem_wait(&priv->op_sem);
	MEMORY_SYNC();

	/* convey the result */

	ret = xctn.disposition;

#elif defined(CONFIG_STM32H7_QSPI_DMA)
	/* Can we perform DMA?  Should we perform DMA? */

	if (priv->candma &&
	    meminfo->buflen > CONFIG_STM32H7_QSPI_DMATHRESHOLD &&
	    IS_ALIGNED((uintptr_t)meminfo->buffer) &&
	    IS_ALIGNED(meminfo->buflen)) {
		ret = qspi_memory_dma(priv, meminfo, &xctn);

	} else {
		/* polling mode */

		/* Set up the Communications Configuration Register as per command
		 * info
		 */

		qspi_ccrconfig(priv, &xctn,
			       QSPIMEM_ISWRITE(meminfo->flags) ? CCR_FMODE_INDWR :
			       CCR_FMODE_INDRD);

		/* Transfer data */

		DEBUGASSERT(meminfo->buffer != NULL && meminfo->buflen > 0);
		DEBUGASSERT(IS_ALIGNED(meminfo->buffer));

		if (QSPIMEM_ISWRITE(meminfo->flags)) {
			ret = qspi_transmit_blocking(priv, &xctn);

		} else {
			ret = qspi_receive_blocking(priv, &xctn);
		}

		/* Wait for Transfer complete, and not busy */

		qspi_waitstatusflags(priv, QSPI_SR_TCF, 1);
		qspi_waitstatusflags(priv, QSPI_SR_BUSY, 0);

		MEMORY_SYNC();
	}

#else
	/* polling mode */

	/* Set up the Communications Configuration Register as per command info */

	qspi_ccrconfig(priv, &xctn,
		       QSPIMEM_ISWRITE(meminfo->flags) ? CCR_FMODE_INDWR :
		       CCR_FMODE_INDRD);

	/* Transfer data */

	DEBUGASSERT(meminfo->buffer != NULL && meminfo->buflen > 0);
	DEBUGASSERT(IS_ALIGNED(meminfo->buffer));

	if (QSPIMEM_ISWRITE(meminfo->flags)) {
		ret = qspi_transmit_blocking(priv, &xctn);

	} else {
		ret = qspi_receive_blocking(priv, &xctn);
	}

	/* Wait for Transfer complete, and not busy */

//qspi_waitstatusflags(priv, QSPI_SR_TCF, 1);
	qspi_waitstatusflags(priv, QSPI_SR_BUSY, 0);

	MEMORY_SYNC()
	;

#endif

	return ret;
}

/****************************************************************************
 * Name: qspi_alloc
 *
 * Description:
 *   Allocate a buffer suitable for DMA data transfer
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buflen - Buffer length to allocate in bytes
 *
 * Returned Value:
 *   Address of the allocated memory on success; NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

QUADSPI_RAMFUNC FAR void *qspi_alloc(FAR struct qspi_dev_s *dev, size_t buflen)
{
	/* Here we exploit the carnal knowledge the kmm_malloc() will return memory
	 * aligned to 64-bit addresses.  The buffer length must be large enough to
	 * hold the rested buflen in units a 32-bits.
	 */

	return kmm_malloc(ALIGN_UP(buflen));
}

/****************************************************************************
 * Name: QSPI_FREE
 *
 * Description:
 *   Free memory returned by QSPI_ALLOC
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - Buffer previously allocated via QSPI_ALLOC
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

QUADSPI_RAMFUNC void qspi_free(FAR struct qspi_dev_s *dev, FAR void *buffer)
{
	if (buffer) {
		kmm_free(buffer);
	}
}

/****************************************************************************
 * Name: qspi_hw_initialize
 *
 * Description:
 *   Initialize the QSPI peripheral from hardware reset.
 *
 * Input Parameters:
 *   priv - Device state structure.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

#if !defined(QSPI_BOOT_IN_MEMORY_MAPPED_MOD)
int qspi_hw_initialize(struct stm32h7_qspidev_s *priv)
{
	uint32_t regval;

	/* Disable the QSPI; abort anything happening, disable, wait for not busy */

	qspi_abort(priv);

	regval = 0;
	regval &= ~(QSPI_CR_EN);
	qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);

	/* Wait till BUSY flag reset */

	qspi_waitstatusflags(priv, QSPI_SR_BUSY, 0);

	/* Disable all interrupt sources for starters */

	regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
	regval &= ~(QSPI_CR_TEIE | QSPI_CR_TCIE | QSPI_CR_FTIE | QSPI_CR_SMIE |
		    QSPI_CR_TOIE | QSPI_CR_FSEL | QSPI_CR_DFM);

#if defined(CONFIG_STM32H7_QSPI_MODE_BANK2)
	regval |= QSPI_CR_FSEL;
#endif

#if defined(CONFIG_STM32H7_QSPI_MODE_DUAL)
	regval |= QSPI_CR_DFM;
#endif

	/* Configure QSPI FIFO Threshold */

	regval &= ~(QSPI_CR_FTHRES_MASK);
	regval |= ((CONFIG_STM32H7_QSPI_FIFO_THESHOLD - 1) <<
		   QSPI_CR_FTHRES_SHIFT);
	qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);

	/* Wait till BUSY flag reset */

	qspi_waitstatusflags(priv, QSPI_SR_BUSY, 0);

	/* Configure QSPI Clock Prescaler and Sample Shift */

	regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
	regval &= ~(QSPI_CR_PRESCALER_MASK | QSPI_CR_SSHIFT);
	regval |= (0x01 << QSPI_CR_PRESCALER_SHIFT);
	regval |= (0x00);
	qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);

	/* Configure QSPI Flash Size, CS High Time and Clock Mode */

	regval = qspi_getreg(priv, STM32_QUADSPI_DCR_OFFSET);
	regval &= ~(QSPI_DCR_CKMODE | QSPI_DCR_CSHT_MASK | QSPI_DCR_FSIZE_MASK);
	regval |= (0x00);
	regval |= ((CONFIG_STM32H7_QSPI_CSHT - 1) << QSPI_DCR_CSHT_SHIFT);

	if (0 != CONFIG_STM32H7_QSPI_FLASH_SIZE) {
		unsigned int nsize = CONFIG_STM32H7_QSPI_FLASH_SIZE;
		int nlog2size = 31;

		while ((nsize & 0x80000000) == 0) {
			--nlog2size;
			nsize <<= 1;
		}

		regval |= ((nlog2size - 1) << QSPI_DCR_FSIZE_SHIFT);
	}

	qspi_putreg(priv, regval, STM32_QUADSPI_DCR_OFFSET);

	/* Enable QSPI */

	regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
	regval |= QSPI_CR_EN;
	qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);

	/* Wait till BUSY flag reset */

	qspi_waitstatusflags(priv, QSPI_SR_BUSY, 0);

	qspi_dumpregs(priv, "After initialization");
	qspi_dumpgpioconfig("GPIO");

	return OK;
}
#endif //!defined(QSPI_BOOT_IN_MEMORY_MAPPED_MOD)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32h7_qspi_initialize
 *
 * Description:
 *   Initialize the selected QSPI port in master mode
 *
 * Input Parameters:
 *   intf - Interface number(must be zero)
 *
 * Returned Value:
 *   Valid QSPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct qspi_dev_s *stm32h7_qspi_initialize(int intf)
{
	struct stm32h7_qspidev_s *priv;

#if  !defined(QSPI_BOOT_IN_MEMORY_MAPPED_MOD)
	uint32_t regval;
	int ret;
#endif

	/* The STM32H7 has only a single QSPI port */

	spiinfo("intf: %d\n", intf);
	DEBUGASSERT(intf == 0);

	/* Select the QSPI interface */

	if (intf == 0) {
		/* If this function is called multiple times, the following operations
		 * will be performed multiple times.
		 */

		/* Select QSPI0 */

		priv = &g_qspi0dev;

#if !defined(QSPI_BOOT_IN_MEMORY_MAPPED_MOD)
		/* Select QSPI clock source */

		modreg32(BOARD_QSPI_CLK, RCC_D1CCIPR_QSPISEL_MASK, STM32_RCC_D1CCIPR);

		/* Enable clocking to the QSPI peripheral */

		regval = getreg32(STM32_RCC_AHB3ENR);
		regval |= RCC_AHB3ENR_QSPIEN;
		putreg32(regval, STM32_RCC_AHB3ENR);

		/* Reset the QSPI peripheral */

		regval = getreg32(STM32_RCC_AHB3RSTR);
		regval |= RCC_AHB3RSTR_QSPIRST;
		putreg32(regval, STM32_RCC_AHB3RSTR);
		regval &= ~RCC_AHB3RSTR_QSPIRST;
		putreg32(regval, STM32_RCC_AHB3RSTR);

		/* Configure multiplexed pins as connected on the board. */

		stm32_configgpio(GPIO_QSPI_CS);
		stm32_configgpio(GPIO_QSPI_IO0);
		stm32_configgpio(GPIO_QSPI_IO1);
		stm32_configgpio(GPIO_QSPI_IO2);
		stm32_configgpio(GPIO_QSPI_IO3);
		stm32_configgpio(GPIO_QSPI_SCK);

#endif  //!defined(QSPI_BOOT_IN_MEMORY_MAPPED_MOD)

	} else {
		spierr("ERROR: QSPI%d not supported\n", intf);
		return NULL;
	}

	/* Has the QSPI hardware been initialized? */

	if (!priv->initialized) {
		/* Now perform one time initialization.
		 *
		 * Initialize the QSPI semaphore that enforces mutually exclusive
		 * access to the QSPI registers.
		 */

		nxsem_init(&priv->exclsem, 0, 1);

#ifdef CONFIG_STM32H7_QSPI_DMA
		/* Pre-allocate DMA channels. */

		if (priv->candma) {
			priv->dmach = stm32_dmachannel(DMACHAN_QUADSPI);

			if (!priv->dmach) {
				spierr("ERROR: Failed to allocate the DMA channel\n");
				priv->candma = false;
			}
		}

		/* Initialize the QSPI semaphore that is used to wake up the waiting
		 * thread when the DMA transfer completes.  This semaphore is used for
		 * signaling and, hence, should not have priority inheritance enabled.
		 */

		nxsem_init(&priv->dmawait, 0, 0);
		nxsem_setprotocol(&priv->dmawait, SEM_PRIO_NONE);

		/* Create a watchdog time to catch DMA timeouts */

		priv->dmadog = wd_create();

		if (priv->dmadog == NULL) {
			spierr("ERROR: Failed to create wdog\n");
			goto errout_with_dmahandles;
		}

#endif

#ifdef CONFIG_STM32H7_QSPI_INTERRUPTS
		/* Attach the interrupt handler */

		ret = irq_attach(priv->irq, priv->handler, NULL);

		if (ret < 0) {
			spierr("ERROR: Failed to attach irq %d\n", priv->irq);
			goto errout_with_dmadog;
		}

		/* Initialize the semaphore that blocks until the operation completes.
		 * This semaphore is used for signaling and, hence, should not have
		 * priority inheritance enabled.
		 */

		nxsem_init(&priv->op_sem, 0, 0);
		nxsem_setprotocol(&priv->op_sem, SEM_PRIO_NONE);
#endif

		/* Perform hardware initialization.  Puts the QSPI into an active
		 * state.
		 */

#if !defined(QSPI_BOOT_IN_MEMORY_MAPPED_MOD)
		ret = qspi_hw_initialize(priv);

		if (ret < 0) {
			spierr("ERROR: Failed to initialize QSPI hardware\n");
			goto errout_with_irq;
		}

#endif

		/* Enable interrupts at the NVIC */

		priv->initialized = true;
		priv->memmap = false;
#ifdef CONFIG_STM32H7_QSPI_INTERRUPTS
		up_enable_irq(priv->irq);
#endif
	}

	return &priv->qspi;

#if !defined(QSPI_BOOT_IN_MEMORY_MAPPED_MOD)
errout_with_irq:
#endif
#ifdef CONFIG_STM32H7_QSPI_INTERRUPTS
	irq_detach(priv->irq);

errout_with_dmadog:
#endif
#ifdef CONFIG_STM32H7_QSPI_DMA
	wd_delete(priv->dmadog);

errout_with_dmahandles:
	nxsem_destroy(&priv->dmawait);

	if (priv->dmach) {
		stm32_dmafree(priv->dmach);
		priv->dmach = NULL;
	}

#endif

	nxsem_destroy(&priv->exclsem);
	return NULL;
}

/****************************************************************************
 * Name: stm32h7_qspi_enter_memorymapped
 *
 * Description:
 *   Put the QSPI device into memory mapped mode
 *
 * Input Parameters:
 *   dev - QSPI device
 *   meminfo - parameters like for a memory transfer used for reading
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

QUADSPI_RAMFUNC void stm32h7_qspi_enter_memorymapped(struct qspi_dev_s *dev,
		const struct qspi_meminfo_s *meminfo, uint32_t lpto)
{
	struct stm32h7_qspidev_s *priv = (struct stm32h7_qspidev_s *) dev;
	uint32_t regval;
	struct qspi_xctnspec_s xctn;

	/* lock during this mode change */

	qspi_lock(dev, true);

	if (priv->memmap) {
		qspi_lock(dev, false);
		return;
	}

	/* Abort anything in-progress */

	qspi_abort(priv);

	/* Wait till BUSY flag reset */

	qspi_waitstatusflags(priv, QSPI_SR_BUSY, 0);

	/* if we want the 'low-power timeout counter' */

	if (lpto > 0) {
		/* Set the Low Power Timeout value (automatically de-assert
		 * CS if memory is not accessed for a while)
		 */

		qspi_putreg(priv, lpto, STM32_QUADSPI_LPTR_OFFSET);

		/* Clear Timeout interrupt */

		qspi_putreg(&g_qspi0dev, QSPI_FCR_CTOF, STM32_QUADSPI_FCR_OFFSET);

#ifdef CONFIG_STM32H7_QSPI_INTERRUPTS
		/* Enable Timeout interrupt */

		regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
		regval |= (QSPI_CR_TCEN | QSPI_CR_TOIE);
		qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);
#endif

	} else {
		regval = qspi_getreg(priv, STM32_QUADSPI_CR_OFFSET);
		regval &= ~QSPI_CR_TCEN;
		qspi_putreg(priv, regval, STM32_QUADSPI_CR_OFFSET);
	}

	/* create a transaction object */

	qspi_setupxctnfrommem(&xctn, meminfo);

#ifdef CONFIG_STM32H7_QSPI_INTERRUPTS
	priv->xctn = NULL;
#endif

	/* set it into the ccr */

	qspi_ccrconfig(priv, &xctn, CCR_FMODE_MEMMAP);
	priv->memmap = true;

	/* we should be in memory mapped mode now */

	qspi_dumpregs(priv, "After memory mapped:");

	/* finished this mode change */

	qspi_lock(dev, false);
}

/****************************************************************************
 * Name: stm32h7_qspi_exit_memorymapped
 *
 * Description:
 *   Take the QSPI device out of memory mapped mode
 *
 * Input Parameters:
 *   dev - QSPI device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

QUADSPI_RAMFUNC void stm32h7_qspi_exit_memorymapped(struct qspi_dev_s *dev)
{
	struct stm32h7_qspidev_s *priv = (struct stm32h7_qspidev_s *) dev;

	qspi_lock(dev, true);

	/* A simple abort is sufficient */

	qspi_abort(priv);
	priv->memmap = false;

	struct qspi_xctnspec_s xctn = { 0 };
	xctn.datamode = 3; /* 3 = quad */
	qspi_ccrconfig(priv, &xctn, CCR_FMODE_INDWR);

	qspi_lock(dev, false);
}

#endif /* CONFIG_STM32H7_QSPI */
