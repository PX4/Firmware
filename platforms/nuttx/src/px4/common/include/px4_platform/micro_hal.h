/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
#pragma once


__BEGIN_DECLS

/* For historical reasons (NuttX STM32 numbering) PX4 bus numbering is 1 based
 * All PX4 code, including, board code is written to assuming 1 based numbering.
 * The following macros are used to allow the board config to define the bus
 * numbers in terms of the NuttX driver numbering. 1,2,3 for one based numbering
 * schemes or 0,1,2 for zero based schemes.
 */

#define PX4_BUS_NUMBER_TO_PX4(x)        ((x)+PX4_BUS_OFFSET)  /* Use to define Zero based to match Nuttx Driver but provide 1 based to PX4 */
#define PX4_BUS_NUMBER_FROM_PX4(x)      ((x)-PX4_BUS_OFFSET)  /* Use to map PX4 1 based to NuttX driver 0 based */

#define px4_enter_critical_section()       enter_critical_section()
#define px4_leave_critical_section(flags)  leave_critical_section(flags)

#define px4_udelay(usec) up_udelay(usec)
#define px4_mdelay(msec) up_mdelay(msec)

#include <arch/board/board.h>

__END_DECLS

