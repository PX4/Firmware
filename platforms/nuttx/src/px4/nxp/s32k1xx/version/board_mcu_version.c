/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
 *   Author: @author Peter van der Perk <peter.vanderperk@nxp.com>
 *                   David Sidrane <david_s5@nscdg.com>
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
 * @file board_mcu_version.c
 * Implementation of S32K1xx based SoC version API
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

#include "arm_arch.h"
#include "hardware/s32k1xx_sim.h"

#define CHIP_TAG     "S32KXXX"
#define CHIP_TAG_LEN sizeof(CHIP_TAG)-1

int board_mcu_version(char *rev, const char **revstr, const char **errata)
{
	uint32_t sim_sdid = getreg32(S32K1XX_SIM_SDID);
	static char chip[sizeof(CHIP_TAG)] = CHIP_TAG;

	chip[CHIP_TAG_LEN - 3] = '0' + ((sim_sdid & SIM_SDID_GENERATION_MASK) >> SIM_SDID_GENERATION_SHIFT);
	chip[CHIP_TAG_LEN - 2] = '0' + ((sim_sdid & SIM_SDID_SUBSERIES_MASK) >> SIM_SDID_SUBSERIES_SHIFT);
	chip[CHIP_TAG_LEN - 1] = '0' + ((sim_sdid & SIM_SDID_DERIVATE_MASK) >> SIM_SDID_DERIVATE_SHIFT);
	*revstr = chip;
	*rev = '0' + ((sim_sdid & SIM_SDID_REVID_MASK) >> SIM_SDID_REVID_SHIFT);

	if (errata) {
		*errata = NULL;
	}

	return 0;
}
