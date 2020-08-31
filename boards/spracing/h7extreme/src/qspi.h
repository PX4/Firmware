/****************************************************************************
 * arch/arm/src/stm32h7/stm32_qspi.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32H7_QSPI_H
#define __ARCH_ARM_SRC_STM32_STM32H7_QSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/qspi.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"

#ifdef CONFIG_STM32H7_QUADSPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#define CONFIG_STM32H7_QUADSPI_USE_RAMFUNC

#ifdef CONFIG_STM32H7_QUADSPI_USE_RAMFUNC
#define QUADSPI_RAMFUNC __ramfunc__
#else
#define QUADSPI_RAMFUNC
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* QSPI methods */

QUADSPI_RAMFUNC int qspi_lock(struct qspi_dev_s *dev, bool lock);
QUADSPI_RAMFUNC uint32_t qspi_setfrequency(struct qspi_dev_s *dev,
		uint32_t frequency);
QUADSPI_RAMFUNC void     qspi_setmode(struct qspi_dev_s *dev, enum qspi_mode_e mode);
QUADSPI_RAMFUNC void     qspi_setbits(struct qspi_dev_s *dev, int nbits);
QUADSPI_RAMFUNC int      qspi_command(struct qspi_dev_s *dev,
				      struct qspi_cmdinfo_s *cmdinfo);
QUADSPI_RAMFUNC int      qspi_memory(struct qspi_dev_s *dev,
				     struct qspi_meminfo_s *meminfo);
QUADSPI_RAMFUNC FAR void *qspi_alloc(FAR struct qspi_dev_s *dev, size_t buflen);
QUADSPI_RAMFUNC void     qspi_free(FAR struct qspi_dev_s *dev, FAR void *buffer);

/****************************************************************************
 * Name: stm32l4_qspi_initialize
 *
 * Description:
 *   Initialize the selected QSPI port in master mode
 *
 * Input Parameters:
 *   intf - Interface number(must be zero)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct qspi_dev_s;
FAR struct qspi_dev_s *stm32h7_qspi_initialize(int intf);

/****************************************************************************
 * Name: stm32l4_qspi_enter_memorymapped
 *
 * Description:
 *   Put the QSPI device into memory mapped mode
 *
 * Input Parameters:
 *   dev - QSPI device
 *   meminfo - parameters like for a memory transfer used for reading
 *   lpto - number of cycles to wait to automatically de-assert CS
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

QUADSPI_RAMFUNC void stm32h7_qspi_enter_memorymapped(struct qspi_dev_s *dev,
		const struct qspi_meminfo_s *meminfo,
		uint32_t lpto);

/****************************************************************************
 * Name: stm32l4_qspi_exit_memorymapped
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

QUADSPI_RAMFUNC void stm32h7_qspi_exit_memorymapped(struct qspi_dev_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_STM32H7_QSPI */
#endif /* __ARCH_ARM_SRC_STM32_STM32H7_QSPI_H */
