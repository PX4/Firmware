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
* Notch filter frequency for gyro
*
* The center frequency for the 2nd order notch filter on the primary gyro.
* This filter can be enabled to avoid feedback amplification of structural resonances at a specific frequency.
* This only affects the signal sent to the controllers, not the estimators.
* Applies to both angular velocity and angular acceleration sent to the controllers.
* See "IMU_GYRO_NF_BW" to set the bandwidth of the filter.
*
* A value of 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_FLOAT(IMU_GYRO_NF_FREQ, 0.0f);

/**
* Notch filter bandwidth for gyro
*
* The frequency width of the stop band for the 2nd order notch filter on the primary gyro.
* See "IMU_GYRO_NF_FREQ" to activate the filter and to set the notch frequency.
* Applies to both angular velocity and angular acceleration sent to the controllers.
*
* @min 0
* @max 100
* @unit Hz
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_FLOAT(IMU_GYRO_NF_BW, 20.0f);

/**
* Low pass filter cutoff frequency for gyro
*
* The cutoff frequency for the 2nd order butterworth filter on the primary gyro.
* This only affects the angular velocity sent to the controllers, not the estimators.
* It applies also to the angular acceleration (D-Term filter), see IMU_DGYRO_CUTOFF.
*
* A value of 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_FLOAT(IMU_GYRO_CUTOFF, 30.0f);

/**
* Gyro control data maximum publication rate (inner loop rate)
*
* The maximum rate the gyro control data (vehicle_angular_velocity) will be
* allowed to publish at. This is the loop rate for the rate controller and outputs.
*
* Note: sensor data is always read and filtered at the full raw rate (eg commonly 8 kHz) regardless of this setting.
*
* @min 100
* @max 2000
* @value 100 100 Hz
* @value 250 250 Hz
* @value 400 400 Hz
* @value 800 800 Hz
* @value 1000 1000 Hz
* @value 2000 2000 Hz
* @unit Hz
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_INT32(IMU_GYRO_RATEMAX, 400);

/**
* Cutoff frequency for angular acceleration (D-Term filter)
*
* The cutoff frequency for the 2nd order butterworth filter used on
* the time derivative of the measured angular velocity, also known as
* the D-term filter in the rate controller. The D-term uses the derivative of
* the rate and thus is the most susceptible to noise. Therefore, using
* a D-term filter allows to increase IMU_GYRO_CUTOFF, which
* leads to reduced control latency and permits to increase the P gains.
*
* A value of 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_FLOAT(IMU_DGYRO_CUTOFF, 20.0f);

/**
* IMU gyro dynamic notch filtering
*
* Enable bank of dynamically updating notch filters.
* Requires ESC RPM feedback or onboard FFT (IMU_GYRO_FFT_EN).
* @group Sensors
* @min 0
* @max 3
* @bit 0 ESC RPM
* @bit 1 FFT
*/
PARAM_DEFINE_INT32(IMU_GYRO_DYN_NF, 0);
