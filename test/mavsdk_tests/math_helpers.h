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

#include <array>

template<typename T>
std::array<T, 3> get_local_mission_item(const Mission::MissionItem &item, const CoordinateTransformation &ct)
{
	using GlobalCoordinate = mavsdk::geometry::CoordinateTransformation::GlobalCoordinate;
	GlobalCoordinate global;
	global.latitude_deg = item.latitude_deg;
	global.longitude_deg = item.longitude_deg;
	auto local = ct.local_from_global(global);
	return {static_cast<T>(local.north_m), static_cast<T>(local.east_m), -item.relative_altitude_m};
}

template<typename T>
T sq(T x)
{
	return x * x;
}

template<typename T>
T norm(const std::array<T, 3> &vec)
{
	return std::sqrt(sq(vec[0]) + sq(vec[1]) + sq(vec[2]));
}

template<typename T>
T dot(const std::array<T, 3> &vec1, const std::array<T, 3> &vec2)
{
	return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
}


template<typename T>
std::array<T, 3> diff(const std::array<T, 3> &vec1, const std::array<T, 3> &vec2)
{
	return {vec1[0] - vec2[0], vec1[1] - vec2[1], vec1[2] - vec2[2]};
}

template<typename T>
std::array<T, 3> normalized(const std::array<T, 3> &vec)
{
	T n = norm(vec);

	if (n > 1e-6f) {
		return {vec[0] / n, vec[1] / n, vec[2] / n};

	} else {
		return {0, 0, 0};
	}
}

template<typename T>
T point_to_line_distance(const std::array<T, 3> &point, const std::array<T, 3> &line_start,
			 const std::array<T, 3> &line_end)
{
	std::array<T, 3> norm_dir = normalized(diff(line_end, line_start));
	T t = dot(norm_dir, diff(point, line_start));

	// closest_on_line = line_start + t * norm_dir;
	std::array<T, 3> closest_on_line { line_start[0] + t *norm_dir[0], line_start[1] + t *norm_dir[1], line_start[2] + t *norm_dir[2]};

	return norm(diff(closest_on_line, point));
}
