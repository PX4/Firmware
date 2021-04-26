/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
* IMU gyro FFT enable.
*
* @boolean
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_INT32(IMU_GYRO_FFT_EN, 0);

/**
* IMU gyro FFT minimum frequency.
*
* @min 1
* @max 1000
* @unit Hz
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_FLOAT(IMU_GYRO_FFT_MIN, 32.f);

/**
* IMU gyro FFT maximum frequency.
*
* @min 1
* @max 1000
* @unit Hz
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_FLOAT(IMU_GYRO_FFT_MAX, 192.f);

/**
* IMU gyro FFT length.
*
* @value 256 256
* @value 1024 1024
* @value 4096 4096
* @unit Hz
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_INT32(IMU_GYRO_FFT_LEN, 1024);
