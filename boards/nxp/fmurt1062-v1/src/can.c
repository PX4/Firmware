/****************************************************************************
 *
 *   Copyright (C) 2016, 2018 PX4 Development Team. All rights reserved.
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
 * @file can.c
 *
 * Board-specific CAN functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_platform_common/px4_config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include <chip.h>
#include "up_arch.h"

#include "board_config.h"

#ifdef CONFIG_CAN

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#if defined(CONFIG_IMXRT_FLEXCAN1) && defined(CONFIG_IMXRT_FLEXCAN2) \
  && defined(CONFIG_IMXRT_FLEXCAN3)
#  warning "CAN1 and CAN2 and CAN2 are enabled.  Assuming only CAN1."
#endif

#ifdef CONFIG_IMXRT_FLEXCAN1
#  define CAN_PORT 1
#else
#  define CAN_PORT 2
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/
int can_devinit(void);

/************************************************************************************
 * Name: can_devinit
 *
 * Description:
 *   All architectures must provide the following interface to work with
 *   examples/can.
 *
 ************************************************************************************/

int can_devinit(void)
{
	static bool initialized = false;
	struct can_dev_s *can;
	int ret;

	/* Check if we have already initialized */

	if (!initialized) {

		/* Call imxrt_caninitialize() to get an instance of the CAN interface */

		can = imxrt_can_initialize(CAN_PORT);

		if (can == NULL) {
			canerr("ERROR:  Failed to get CAN interface\n");
			return -ENODEV;
		}

		/* Register the CAN driver at "/dev/can0" */

		ret = can_register("/dev/can0", can);

		if (ret < 0) {
			canerr("ERROR: can_register failed: %d\n", ret);
			return ret;
		}

		/* Now we are initialized */

		initialized = true;
	}

	return OK;
}

#endif
