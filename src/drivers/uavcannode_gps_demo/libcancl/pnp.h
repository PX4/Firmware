/****************************************************************************
 * nxp_bms/BMS_v1/inc/UAVCAN/pnp.h
 *
 * BSD 3-Clause License
 *
 * Copyright 2020 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ** ###################################################################
 **     Filename    : pnp.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date   		: 2020-10-29
 **     Abstract    :
 **        pnp module.
 **
 ** ###################################################################*/
/*!
 ** @file pnp.h
 **
 ** @version 01.00
 **
 ** @brief
 **        pnp module. this module implements the UAVCAN plug and play protocol
 **
 */

#ifndef UAVCAN_PNP_H_
#define UAVCAN_PNP_H_

#define NUNAVUT_ASSERT
#include <canard.h>

#include "time.h"


uint32_t initPNPAllocatee(CanardInstance *ins, uint8_t *unique_id);

int32_t PNPAllocRequest(CanardInstance *ins);

int32_t PNPProcess(CanardInstance *ins, CanardTransfer *transfer);

CanardNodeID PNPGetNodeID();

const CanardPortID PNPGetPortID();



#endif //UAVCAN_PNP_H_
