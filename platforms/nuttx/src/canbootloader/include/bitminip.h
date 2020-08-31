/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/*
 * @file board.h
 *
 * bootloader board interface
 * This file contains the common interfaces that all boards
 * have to supply
 */

#pragma once

#define BIT(pos)                                (1ull<<(pos))
#define BIT_MASK(length)                        (BIT(length)-1)

#define BITFEILD_MASK(lsb_pos, length)          ( BIT_MASK(length) << (lsb_pos))
#define BITFEILD_ISOLATE(x, lsb_pos, length)    ((x) & (BITFEILD_MASK((lsb_pos), (length))))
#define BITFEILD_EXCLUDE(x, lsb_pos, length)    ((x) & ~(BITFEILD_MASK((lsb_pos), (length))))
#define BITFEILD_GET(y, lsb_pos, length)        (((y)>>(lsb_pos)) & BIT_MASK(length))
#define BITFEILD_SET(y, x, lsb_pos, length)     ( y= ((y) & ~BF_MASK(lsb_pos, length)) | BITFEILD_ISOLATE(x, lsb_pos, length))
