/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file fake_geofence.h
 * Provides a fake geofence interface to use for testing
 *
 * @author Roman Bapst
 * @author Julian Kent
 */
#pragma once

#include"../geofence.h"
#include <lib/mathlib/mathlib.h>

class FakeGeofence : public Geofence
{
public:
	FakeGeofence() :
		Geofence(nullptr)
	{};

	virtual ~FakeGeofence() {};

	bool isInsidePolygonOrCircle(double lat, double lon, float altitude) override
	{
		switch (_probe_function_behavior) {
		case ProbeFunction::ALL_POINTS_OUTSIDE: {
				return _allPointsOutside(lat, lon, altitude);
			}

		case ProbeFunction::LEFT_INSIDE_RIGHT_OUTSIDE: {
				return _left_inside_right_outside(lat, lon, altitude);
			}

		case ProbeFunction::RIGHT_INSIDE_LEFT_OUTSIDE: {
				return _right_inside_left_outside(lat, lon, altitude);
			}

		case ProbeFunction::GF_BOUNDARY_20M_AHEAD: {
				return _gf_boundary_is_20m_north(lat, lon, altitude);
			}

		default:
			return _allPointsOutside(lat, lon, altitude);
		}
	}

	enum class ProbeFunction {
		ALL_POINTS_OUTSIDE = 0,
		LEFT_INSIDE_RIGHT_OUTSIDE,
		RIGHT_INSIDE_LEFT_OUTSIDE,
		GF_BOUNDARY_20M_AHEAD
	};

	void setProbeFunctionBehavior(ProbeFunction func) {_probe_function_behavior = func;}


private:

	ProbeFunction _probe_function_behavior = ProbeFunction::ALL_POINTS_OUTSIDE;

	bool _flag_on_left = true;
	bool _flag_on_right = false;

	bool _allPointsOutside(double lat, double lon, float alt)
	{
		return false;
	}

	bool _left_inside_right_outside(double lat, double lon, float alt)
	{
		if (_flag_on_left) {
			_flag_on_left = false;
			return true;

		} else {
			return false;
		}
	}

	bool _right_inside_left_outside(double lat, double lon, float alt)
	{
		if (_flag_on_right) {
			_flag_on_right = false;
			return true;

		} else {
			_flag_on_right = true;
			return false;
		}
	}

	bool _gf_boundary_is_20m_north(double lat, double lon, float alt)
	{
		struct map_projection_reference_s ref = {};
		matrix::Vector2<double> home_global(42.1, 8.2);
		map_projection_init(&ref, home_global(0), home_global(1));

		float x, y;
		map_projection_project(&ref, lat, lon, &x, &y);
		matrix::Vector2f waypoint_local(x, y);

		if (waypoint_local(0) >= 20.0f) {
			return false;
		}

		return true;
	}
};
