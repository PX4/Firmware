/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <lib/conversion/rotation.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/distance_sensor.h>

class PX4Rangefinder
{

public:
	PX4Rangefinder(const uint32_t device_id,
		       const uint8_t device_orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	~PX4Rangefinder();

	void set_device_type(uint8_t device_type);
	//void set_error_count(uint64_t error_count) { _distance_sensor_pub.get().error_count = error_count; }

	void set_device_id(const uint8_t device_id) { _distance_sensor_pub.get().id = device_id; };

	void set_fov(const float fov) { set_hfov(fov); set_vfov(fov); }
	void set_hfov(const float fov) { _distance_sensor_pub.get().h_fov = fov; }
	void set_vfov(const float fov) { _distance_sensor_pub.get().v_fov = fov; }

	void set_max_distance(const float distance) { _distance_sensor_pub.get().max_distance = distance; }
	void set_min_distance(const float distance) { _distance_sensor_pub.get().min_distance = distance; }

	void set_orientation(const uint8_t device_orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	void update(const hrt_abstime &timestamp_sample, const float distance, const int8_t quality = -1);

private:

	uORB::PublicationMultiData<distance_sensor_s> _distance_sensor_pub;

};
