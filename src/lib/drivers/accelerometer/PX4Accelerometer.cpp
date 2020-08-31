/****************************************************************************
 *
 *   Copyright (c) 2018-2020 PX4 Development Team. All rights reserved.
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


#include "PX4Accelerometer.hpp"

#include <lib/drivers/device/Device.hpp>

using namespace time_literals;
using matrix::Vector3f;

static constexpr int32_t sum(const int16_t samples[16], uint8_t len)
{
	int32_t sum = 0;

	for (int n = 0; n < len; n++) {
		sum += samples[n];
	}

	return sum;
}

static constexpr uint8_t clipping(const int16_t samples[16], int16_t clip_limit, uint8_t len)
{
	unsigned clip_count = 0;

	for (int n = 0; n < len; n++) {
		if (abs(samples[n]) >= clip_limit) {
			clip_count++;
		}
	}

	return clip_count;
}

PX4Accelerometer::PX4Accelerometer(uint32_t device_id, enum Rotation rotation) :
	ModuleParams(nullptr),
	_sensor_pub{ORB_ID(sensor_accel)},
	_sensor_fifo_pub{ORB_ID(sensor_accel_fifo)},
	_device_id{device_id},
	_rotation{rotation}
{
	// advertise immediately to keep instance numbering in sync
	_sensor_pub.advertise();

	updateParams();
}

PX4Accelerometer::~PX4Accelerometer()
{
	_sensor_pub.unadvertise();
	_sensor_fifo_pub.unadvertise();
}

void PX4Accelerometer::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back
	_device_id = device_id.devid;
}

void PX4Accelerometer::update(const hrt_abstime &timestamp_sample, float x, float y, float z)
{
	// clipping
	uint8_t clip_count[3];
	clip_count[0] = (fabsf(x) >= _clip_limit);
	clip_count[1] = (fabsf(y) >= _clip_limit);
	clip_count[2] = (fabsf(z) >= _clip_limit);

	// publish
	Publish(timestamp_sample, x, y, z, clip_count);
}

void PX4Accelerometer::updateFIFO(sensor_accel_fifo_s &sample)
{
	// publish fifo
	sample.device_id = _device_id;
	sample.scale = _scale;
	sample.rotation = _rotation;

	sample.timestamp = hrt_absolute_time();
	_sensor_fifo_pub.publish(sample);

	{
		// trapezoidal integration (equally spaced, scaled by dt later)
		const uint8_t N = sample.samples;
		const Vector3f integral{
			(0.5f * (_last_sample[0] + sample.x[N - 1]) + sum(sample.x, N - 1)),
			(0.5f * (_last_sample[1] + sample.y[N - 1]) + sum(sample.y, N - 1)),
			(0.5f * (_last_sample[2] + sample.z[N - 1]) + sum(sample.z, N - 1)),
		};

		_last_sample[0] = sample.x[N - 1];
		_last_sample[1] = sample.y[N - 1];
		_last_sample[2] = sample.z[N - 1];

		// clipping
		uint8_t clip_count[3] {
			clipping(sample.x, _clip_limit, N),
			clipping(sample.y, _clip_limit, N),
			clipping(sample.z, _clip_limit, N),
		};

		const float x = integral(0) / (float)N;
		const float y = integral(1) / (float)N;
		const float z = integral(2) / (float)N;

		// publish
		Publish(sample.timestamp_sample, x, y, z, clip_count);
	}
}

void PX4Accelerometer::Publish(const hrt_abstime &timestamp_sample, float x, float y, float z, uint8_t clip_count[3])
{
	// Apply rotation (before scaling)
	rotate_3f(_rotation, x, y, z);

	float clipping_x = clip_count[0];
	float clipping_y = clip_count[1];
	float clipping_z = clip_count[2];
	rotate_3f(_rotation, clipping_x, clipping_y, clipping_z);

	sensor_accel_s report;

	report.timestamp_sample = timestamp_sample;
	report.device_id = _device_id;
	report.temperature = _temperature;
	report.error_count = _error_count;
	report.x = x * _scale;
	report.y = y * _scale;
	report.z = z * _scale;
	report.clip_counter[0] = fabsf(roundf(clipping_x));
	report.clip_counter[1] = fabsf(roundf(clipping_y));
	report.clip_counter[2] = fabsf(roundf(clipping_z));
	report.timestamp = hrt_absolute_time();

	_sensor_pub.publish(report);
}

void PX4Accelerometer::UpdateClipLimit()
{
	// 99.9% of potential max
	_clip_limit = fmaxf((_range / _scale) * 0.999f, INT16_MAX);
}
