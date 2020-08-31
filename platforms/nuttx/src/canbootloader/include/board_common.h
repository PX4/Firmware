/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
 *                 Author: David Sidrane <david_s5@nscdg.com>
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
 * @file board_common.h
 *
 * Provide the the common board interfaces
 */

#pragma once

/* Defined the types used for board UUID and MFG UID
 *
 * A type suitable for holding the byte format of the UUID
 *
 * The original PX4 stm32 (legacy) based implementation **displayed** the
 * UUID as: ABCD EFGH IJKL
 * Where:
 *       A was bit 31 and D was bit 0
 *       E was bit 63 and H was bit 32
 *       I was bit 95 and L was bit 64
 *
 * Since the string was used by some manufactures to identify the units
 * it must be preserved.
 *
 * For new targets moving forward we will use
 *      IJKL EFGH ABCD
 */

/* A type suitable for defining the 8 bit format of the MFG UID
 * This is always returned as MSD @ index 0 -LSD @ index PX4_CPU_MFGUID_BYTE_LENGTH-1
 */
typedef uint8_t mfguid_t[PX4_CPU_MFGUID_BYTE_LENGTH];

/************************************************************************************
 * Name: board_get_mfguid
 *
 * Description:
 *   All boards either provide a way to retrieve a manafactuers Uniqe ID or
 *   define BOARD_OVERRIDE_MFGUID.
 *    The MFGUID is returned as an array of bytes in
 *    MSD @ index 0 - LSD @ index PX4_CPU_MFGUID_BYTE_LENGTH-1
 *
 ************************************************************************************/

int board_get_mfguid(mfguid_t mfgid);
