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
 * PCF8583 rotorfreq (i2c) pool interval
 *
 * Determines how often the sensor is read out.
 *
 * @reboot_required true
 * @group Sensors
 * @unit us
 */
PARAM_DEFINE_INT32(PCF8583_POOL, 1000000);

/**
 * PCF8583 rotorfreq (i2c) i2c address
 *
 * @reboot_required true
 * @group Sensors
 * @value 80 Address 0x50 (80)
 * @value 81 Address 0x51 (81)
 */
PARAM_DEFINE_INT32(PCF8583_ADDR, 80);

/**
 * PCF8583 rotorfreq (i2c) pulse reset value
 *
 * Internal device counter is reset to 0 when overun this value,
 * counter is able to store upto 6 digits
 * reset of counter takes some time - measurement with reset has worse accurancy.
 * 0 means reset counter after every measurement.
 *
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_INT32(PCF8583_RESET, 500000);

/**
 * PCF8583 rotorfreq (i2c) pulse count
 *
 * Nmumber of signals per rotation of actuator
 *
 * @reboot_required true
 * @group Sensors
 * @min 1
 */
PARAM_DEFINE_INT32(PCF8583_MAGNET, 2);
