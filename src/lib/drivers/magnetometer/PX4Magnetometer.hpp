/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <lib/conversion/rotation.h>
#include <uORB/uORB.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_mag.h>

class PX4Magnetometer : public cdev::CDev
{

public:
	PX4Magnetometer(uint32_t device_id, uint8_t priority = ORB_PRIO_DEFAULT, enum Rotation rotation = ROTATION_NONE);
	~PX4Magnetometer() override;

	int	ioctl(cdev::file_t *filp, int cmd, unsigned long arg) override;

	void set_device_type(uint8_t devtype);
	void set_error_count(uint64_t error_count) { _sensor_mag_pub.get().error_count = error_count; }
	void increase_error_count() { _sensor_mag_pub.get().error_count++; }
	void set_scale(float scale) { _sensor_mag_pub.get().scaling = scale; }
	void set_temperature(float temperature) { _sensor_mag_pub.get().temperature = temperature; }
	void set_external(bool external) { _sensor_mag_pub.get().is_external = external; }
	void set_sensitivity(float x, float y, float z) { _sensitivity = matrix::Vector3f{x, y, z}; }

	void update(hrt_abstime timestamp_sample, float x, float y, float z);

	void print_status();

private:

	uORB::PublicationMultiData<sensor_mag_s>	_sensor_mag_pub;

	const enum Rotation	_rotation;

	matrix::Vector3f	_calibration_scale{1.0f, 1.0f, 1.0f};
	matrix::Vector3f	_calibration_offset{0.0f, 0.0f, 0.0f};

	matrix::Vector3f	_sensitivity{1.0f, 1.0f, 1.0f};

	int			_class_device_instance{-1};

};
