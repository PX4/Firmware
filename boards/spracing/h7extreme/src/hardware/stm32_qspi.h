/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_qspi.h
 *
 *   Copyright (C) 2016, 2019 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7_QSPI_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7_QSPI_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>
#include <arch/stm32h7/chip.h>

#include "chip.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* General Characteristics **************************************************************/

#define STM32H7_QSPI_MINBITS          8         /* Minimum word width */
#define STM32H7_QSPI_MAXBITS          32        /* Maximum word width */

/* QSPI register offsets ****************************************************************/

#define STM32_QUADSPI_CR_OFFSET       0x0000    /* Control Register */
#define STM32_QUADSPI_DCR_OFFSET      0x0004    /* Device Configuration Register */
#define STM32_QUADSPI_SR_OFFSET       0x0008    /* Status Register */
#define STM32_QUADSPI_FCR_OFFSET      0x000c    /* Flag Clear Register */
#define STM32_QUADSPI_DLR_OFFSET      0x0010    /* Data Length Register */
#define STM32_QUADSPI_CCR_OFFSET      0x0014    /* Communication Configuration Register */
#define STM32_QUADSPI_AR_OFFSET       0x0018    /* Address Register */
#define STM32_QUADSPI_ABR_OFFSET      0x001c    /* Alternate Bytes Register */
#define STM32_QUADSPI_DR_OFFSET       0x0020    /* Data Register */
#define STM32_QUADSPI_PSMKR_OFFSET    0x0024    /* Polling Status mask Register */
#define STM32_QUADSPI_PSMAR_OFFSET    0x0028    /* Polling Status match Register */
#define STM32_QUADSPI_PIR_OFFSET      0x002c    /* Polling Interval Register */
#define STM32_QUADSPI_LPTR_OFFSET     0x0030    /* Low-Power Timeout Register */

/* QSPI register addresses **************************************************************/

#define STM32_QUADSPI_CR       (STM32_QUADSPI_BASE+STM32_QUADSPI_CR_OFFSET)    /* Control Register */
#define STM32_QUADSPI_DCR      (STM32_QUADSPI_BASE+STM32_QUADSPI_DCR_OFFSET)   /* Device Configuration Register */
#define STM32_QUADSPI_SR       (STM32_QUADSPI_BASE+STM32_QUADSPI_SR_OFFSET)    /* Status Register */
#define STM32_QUADSPI_FCR      (STM32_QUADSPI_BASE+STM32_QUADSPI_FCR_OFFSET)   /* Flag Clear Register */
#define STM32_QUADSPI_DLR      (STM32_QUADSPI_BASE+STM32_QUADSPI_DLR_OFFSET)   /* Data Length Register */
#define STM32_QUADSPI_CCR      (STM32_QUADSPI_BASE+STM32_QUADSPI_CCR_OFFSET)   /* Communication Configuration Register */
#define STM32_QUADSPI_AR       (STM32_QUADSPI_BASE+STM32_QUADSPI_AR_OFFSET)    /* Address Register */
#define STM32_QUADSPI_ABR      (STM32_QUADSPI_BASE+STM32_QUADSPI_ABR_OFFSET)   /* Alternate Bytes Register */
#define STM32_QUADSPI_DR       (STM32_QUADSPI_BASE+STM32_QUADSPI_DR_OFFSET)    /* Data Register */
#define STM32_QUADSPI_PSMKR    (STM32_QUADSPI_BASE+STM32_QUADSPI_PSMKR_OFFSET) /* Polling Status mask Register */
#define STM32_QUADSPI_PSMAR    (STM32_QUADSPI_BASE+STM32_QUADSPI_PSMAR_OFFSET) /* Polling Status match Register */
#define STM32_QUADSPI_PIR      (STM32_QUADSPI_BASE+STM32_QUADSPI_PIR_OFFSET)   /* Polling Interval Register */
#define STM32_QUADSPI_LPTR     (STM32_QUADSPI_BASE+STM32_QUADSPI_LPTR_OFFSET)  /* Low-Power Timeout Register */

/* QSPI register bit definitions ********************************************************/

/* Control Register */

#define QSPI_CR_EN                 (1 << 0)   /* Bit 0:  QSPI Enable */
#define QSPI_CR_ABORT              (1 << 1)   /* Bit 1:  Abort request */
#define QSPI_CR_TCEN               (1 << 3)   /* Bit 3:  Timeout counter enable */
#define QSPI_CR_SSHIFT             (1 << 4)   /* Bit 4:  Sample shift */
#define QSPI_CR_DFM                (1 << 6)   /* Bit 6:  DFM: Dual-flash mode */
#define QSPI_CR_FSEL               (1 << 7)   /* Bit 7:  FSEL: Flash memory selection */
#define QSPI_CR_FTHRES_SHIFT       (8)        /* Bits 8-11: FIFO threshold level */
#define QSPI_CR_FTHRES_MASK        (0x0f << QSPI_CR_FTHRES_SHIFT)
#define QSPI_CR_TEIE               (1 << 16)  /* Bit 16:  Transfer error interrupt enable */
#define QSPI_CR_TCIE               (1 << 17)  /* Bit 17:  Transfer complete interrupt enable */
#define QSPI_CR_FTIE               (1 << 18)  /* Bit 18:  FIFO threshold interrupt enable */
#define QSPI_CR_SMIE               (1 << 19)  /* Bit 19:  Status match interrupt enable */
#define QSPI_CR_TOIE               (1 << 20)  /* Bit 20:  TimeOut interrupt enable */
#define QSPI_CR_APMS               (1 << 22)  /* Bit 22:  Automatic poll mode stop */
#define QSPI_CR_PMM                (1 << 23)  /* Bit 23:  Polling match mode */
#define QSPI_CR_PRESCALER_SHIFT    (24)       /* Bits 24-31: Clock prescaler */
#define QSPI_CR_PRESCALER_MASK     (0xff << QSPI_CR_PRESCALER_SHIFT)

/* Device Configuration Register */

#define QSPI_DCR_CKMODE            (1 << 0)   /* Bit 0:  Mode 0 / mode 3 */
#define QSPI_DCR_CSHT_SHIFT        (8)        /* Bits 8-10: Chip select high time */
#define QSPI_DCR_CSHT_MASK         (0x7 << QSPI_DCR_CSHT_SHIFT)
#define QSPI_DCR_FSIZE_SHIFT       (16)       /* Bits 16-20: Flash memory size */
#define QSPI_DCR_FSIZE_MASK        (0x1f << QSPI_DCR_FSIZE_SHIFT)

/* Status Register */

#define QSPI_SR_TEF                (1 << 0)   /* Bit 0:  Transfer error flag */
#define QSPI_SR_TCF                (1 << 1)   /* Bit 1:  Transfer complete flag */
#define QSPI_SR_FTF                (1 << 2)   /* Bit 2:  FIFO threshold flag */
#define QSPI_SR_SMF                (1 << 3)   /* Bit 3:  Status match flag */
#define QSPI_SR_TOF                (1 << 4)   /* Bit 4:  Timeout flag */
#define QSPI_SR_BUSY               (1 << 5)   /* Bit 5:  Busy */
#define QSPI_SR_FLEVEL_SHIFT       (8)        /* Bits 8-12: FIFO threshold level */
#define QSPI_SR_FLEVEL_MASK        (0x1f << QSPI_SR_FLEVEL_SHIFT)

/* Flag Clear Register */

#define QSPI_FCR_CTEF              (1 << 0)   /* Bit 0:  Clear Transfer error flag */
#define QSPI_FCR_CTCF              (1 << 1)   /* Bit 1:  Clear Transfer complete flag */
#define QSPI_FCR_CSMF              (1 << 3)   /* Bit 3:  Clear Status match flag */
#define QSPI_FCR_CTOF              (1 << 4)   /* Bit 4:  Clear Timeout flag */

/* Data Length Register */

/* Communication Configuration Register */

#define CCR_IMODE_NONE      0   /* No instruction */
#define CCR_IMODE_SINGLE    1   /* Instruction on a single line */
#define CCR_IMODE_DUAL      2   /* Instruction on two lines */
#define CCR_IMODE_QUAD      3   /* Instruction on four lines */

#define CCR_ADMODE_NONE     0   /* No address */
#define CCR_ADMODE_SINGLE   1   /* Address on a single line */
#define CCR_ADMODE_DUAL     2   /* Address on two lines */
#define CCR_ADMODE_QUAD     3   /* Address on four lines */

#define CCR_ADSIZE_8        0   /* 8-bit address */
#define CCR_ADSIZE_16       1   /* 16-bit address */
#define CCR_ADSIZE_24       2   /* 24-bit address */
#define CCR_ADSIZE_32       3   /* 32-bit address */

#define CCR_ABMODE_NONE     0   /* No alternate bytes */
#define CCR_ABMODE_SINGLE   1   /* Alternate bytes on a single line */
#define CCR_ABMODE_DUAL     2   /* Alternate bytes on two lines */
#define CCR_ABMODE_QUAD     3   /* Alternate bytes on four lines */

#define CCR_ABSIZE_8        0   /* 8-bit alternate byte */
#define CCR_ABSIZE_16       1   /* 16-bit alternate bytes */
#define CCR_ABSIZE_24       2   /* 24-bit alternate bytes */
#define CCR_ABSIZE_32       3   /* 32-bit alternate bytes */

#define CCR_DMODE_NONE      0   /* No data */
#define CCR_DMODE_SINGLE    1   /* Data on a single line */
#define CCR_DMODE_DUAL      2   /* Data on two lines */
#define CCR_DMODE_QUAD      3   /* Data on four lines */

#define CCR_FMODE_INDWR     0   /* Indirect write mode */
#define CCR_FMODE_INDRD     1   /* Indirect read mode */
#define CCR_FMODE_AUTOPOLL  2   /* Automatic polling mode */
#define CCR_FMODE_MEMMAP    3   /* Memory-mapped mode */

#define QSPI_CCR_INSTRUCTION_SHIFT (0)        /* Bits 0-7: Instruction */
#define QSPI_CCR_INSTRUCTION_MASK  (0xff << QSPI_CCR_INSTRUCTION_SHIFT)
#  define QSPI_CCR_INST(n)         ((uint32_t)(n) << QSPI_CCR_INSTRUCTION_SHIFT)
#define QSPI_CCR_IMODE_SHIFT       (8)        /* Bits 8-9: Instruction mode */
#define QSPI_CCR_IMODE_MASK        (0x3 << QSPI_CCR_IMODE_SHIFT)
#  define QSPI_CCR_IMODE(n)        ((uint32_t)(n) << QSPI_CCR_IMODE_SHIFT)
#define QSPI_CCR_ADMODE_SHIFT      (10)        /* Bits 10-11: Address mode */
#define QSPI_CCR_ADMODE_MASK       (0x3 << QSPI_CCR_ADMODE_SHIFT)
#  define QSPI_CCR_ADMODE(n)       ((uint32_t)(n) << QSPI_CCR_ADMODE_SHIFT)
#define QSPI_CCR_ADSIZE_SHIFT      (12)        /* Bits 12-13: Address size */
#define QSPI_CCR_ADSIZE_MASK       (0x3 << QSPI_CCR_ADSIZE_SHIFT)
#  define QSPI_CCR_ADSIZE(n)       ((uint32_t)(n) << QSPI_CCR_ADSIZE_SHIFT)
#define QSPI_CCR_ABMODE_SHIFT      (14)        /* Bits 14-15: Alternate bytes mode */
#define QSPI_CCR_ABMODE_MASK       (0x3 << QSPI_CCR_ABMODE_SHIFT)
#  define QSPI_CCR_ABMODE(n)       ((uint32_t)(n) << QSPI_CCR_ABMODE_SHIFT)
#define QSPI_CCR_ABSIZE_SHIFT      (16)        /* Bits 16-17: Alternate bytes size */
#define QSPI_CCR_ABSIZE_MASK       (0x3 << QSPI_CCR_ABSIZE_SHIFT)
#  define QSPI_CCR_ABSIZE(n)       ((uint32_t)(n) << QSPI_CCR_ABSIZE_SHIFT)
#define QSPI_CCR_DCYC_SHIFT        (18)        /* Bits 18-23: Number of dummy cycles */
#define QSPI_CCR_DCYC_MASK         (0x1f << QSPI_CCR_DCYC_SHIFT)
#  define QSPI_CCR_DCYC(n)         ((uint32_t)(n) << QSPI_CCR_DCYC_SHIFT)
#define QSPI_CCR_DMODE_SHIFT       (24)        /* Bits 24-25: Data mode */
#define QSPI_CCR_DMODE_MASK        (0x3 << QSPI_CCR_DMODE_SHIFT)
#  define QSPI_CCR_DMODE(n)        ((uint32_t)(n) << QSPI_CCR_DMODE_SHIFT)
#define QSPI_CCR_FMODE_SHIFT       (26)        /* Bits 26-27: Functional mode */
#define QSPI_CCR_FMODE_MASK        (0x3 << QSPI_CCR_FMODE_SHIFT)
#  define QSPI_CCR_FMODE(n)        ((uint32_t)(n) << QSPI_CCR_FMODE_SHIFT)
#define QSPI_CCR_SIOO              (1 << 28)   /* Bit 28:  Send instruction only once mode */
#define QSPI_CCR_FRCM              (1 << 29)   /* Bit 28:  Enters Free running clock mode */
#define QSPI_CCR_DDRM              (1 << 31)   /* Bit 31:  Double data rate mode */

/* Address Register */

/* Alternate Bytes Register */

/* Data Register */

/* Polling Status mask Register */

/* Polling Status match Register */

/* Polling Interval Register */

#define QSPI_PIR_INTERVAL_SHIFT    (0)        /* Bits 0-15: Polling interval */
#define QSPI_PIR_INTERVAL_MASK     (0xFFff << QSPI_PIR_INTERVAL_SHIFT)

/* Low-Power Timeout Register */

#define QSPI_LPTR_TIMEOUT_SHIFT    (0)        /* Bits 0-15: Timeout period */
#define QSPI_LPTR_TIMEOUT_MASK     (0xFFff << QSPI_PIR_INTERVAL_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H4_QSPI_H */
