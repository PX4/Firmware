/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file uuv_pos_control_params.c
 *
 * Parameters defined by the position control task for unmanned underwater vehicles (UUVs)
 *
 * This is a modification of the fixed wing/ground rover params and it is designed for ground rovers.
 * It has been developed starting from the fw  module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing/rover app are reported in those files.
 *
 * @author Tim Hansen <t.hansen@jacobs-university.de>
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */
/**
 * Gain of P controller X
 *
 * @group UUV Position Control
 */
PARAM_DEFINE_FLOAT(UUV_GAIN_X_P, 1.0f);
/**
 * Gain of P controller Y
 *
 * @group UUV Position Control
 */
PARAM_DEFINE_FLOAT(UUV_GAIN_Y_P, 1.0f);
/**
 * Gain of P controller Z
 *
 * @group UUV Position Control
 */
PARAM_DEFINE_FLOAT(UUV_GAIN_Z_P, 1.0f);

/**
 * Gain of D controller X
 *
 * @group UUV Position Control
 */
PARAM_DEFINE_FLOAT(UUV_GAIN_X_D, 0.2f);
/**
 * Gain of D controller Y
 *
 * @group UUV Position Control
 */
PARAM_DEFINE_FLOAT(UUV_GAIN_Y_D, 0.2f);
/**
 * Gain of D controller Z
 *
 * @group UUV Position Control
 */
PARAM_DEFINE_FLOAT(UUV_GAIN_Z_D, 0.2f);

/**
 * Stabilization mode(1) or Position Control(0)
 *
 * @value 0 Position Control
 * @value 1 Stabilization Mode
 * @group UUV Position Control
 */
PARAM_DEFINE_INT32(UUV_STAB_MODE, 1);
