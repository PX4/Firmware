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

#pragma once

#include <lib/conversion/rotation.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_correction.h>

namespace calibration
{
class Gyroscope
{
public:
	static constexpr int MAX_SENSOR_COUNT = 4;

	static constexpr uint8_t DEFAULT_PRIORITY = 50;
	static constexpr uint8_t DEFAULT_EXTERNAL_PRIORITY = 25;

	static constexpr const char *SensorString() { return "GYRO"; }

	Gyroscope();
	explicit Gyroscope(uint32_t device_id, bool external = false);

	~Gyroscope() = default;

	void PrintStatus();

	void set_calibration_index(uint8_t calibration_index) { _calibration_index = calibration_index; }
	void set_device_id(uint32_t device_id, bool external = false);
	void set_external(bool external = true);
	bool set_offset(const matrix::Vector3f &offset);
	void set_rotation(Rotation rotation);

	uint8_t calibration_count() const { return _calibration_count; }
	uint32_t device_id() const { return _device_id; }
	bool enabled() const { return (_priority > 0); }
	bool external() const { return _external; }
	const matrix::Vector3f &offset() const { return _offset; }
	const int32_t &priority() const { return _priority; }
	const matrix::Dcmf &rotation() const { return _rotation; }
	const Rotation &rotation_enum() const { return _rotation_enum; }
	const matrix::Vector3f &thermal_offset() const { return _thermal_offset; }

	// apply offsets and scale
	// rotate corrected measurements from sensor to body frame
	inline matrix::Vector3f Correct(const matrix::Vector3f &data) const
	{
		return _rotation * matrix::Vector3f{data - _thermal_offset - _offset};
	}

	inline matrix::Vector3f Uncorrect(const matrix::Vector3f &corrected_data) const
	{
		return (_rotation.I() * corrected_data) + _thermal_offset + _offset;
	}

	bool ParametersSave();
	void ParametersUpdate();

	void Reset();

	void SensorCorrectionsUpdate(bool force = false);

private:
	uORB::Subscription _sensor_correction_sub{ORB_ID(sensor_correction)};

	Rotation _rotation_enum{ROTATION_NONE};

	matrix::Dcmf _rotation;
	matrix::Vector3f _offset;
	matrix::Vector3f _thermal_offset;

	int8_t _calibration_index{-1};
	uint32_t _device_id{0};
	int32_t _priority{-1};

	bool _external{false};

	uint8_t _calibration_count{0};
};
} // namespace calibration
