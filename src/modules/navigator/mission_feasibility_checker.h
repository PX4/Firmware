/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file mission_feasibility_checker.h
 * Provides checks if mission is feasible given the navigation capabilities
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Sander Smeets <sander@droneslab.com>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#pragma once

#include <dataman/dataman.h>
#include <uORB/topics/mission.h>

class Geofence;
class Navigator;

class MissionFeasibilityChecker
{
private:
	Navigator *_navigator{nullptr};

	/* Checks for all airframes */
	bool checkGeofence(const mission_s &mission, float home_alt, bool home_valid);

	bool checkHomePositionAltitude(const mission_s &mission, float home_alt, bool home_alt_valid, bool throw_error);

	bool checkMissionItemValidity(const mission_s &mission);

	bool checkDistanceToFirstWaypoint(const mission_s &mission, float max_distance);
	bool checkDistancesBetweenWaypoints(const mission_s &mission, float max_distance);

	bool checkTakeoff(const mission_s &mission, float home_alt);

	/* Checks specific to fixedwing airframes */
	bool checkFixedwing(const mission_s &mission, float home_alt, bool land_start_req);
	bool checkFixedWingLanding(const mission_s &mission, bool land_start_req);

	/* Checks specific to rotarywing airframes */
	bool checkRotarywing(const mission_s &mission, float home_alt);

	/* Checks specific to VTOL airframes */
	bool checkVTOL(const mission_s &mission, float home_alt, bool land_start_req);
	bool checkVTOLLanding(const mission_s &mission, bool land_start_req);

public:
	MissionFeasibilityChecker(Navigator *navigator) : _navigator(navigator) {}
	~MissionFeasibilityChecker() = default;

	MissionFeasibilityChecker(const MissionFeasibilityChecker &) = delete;
	MissionFeasibilityChecker &operator=(const MissionFeasibilityChecker &) = delete;

	/*
	 * Returns true if mission is feasible and false otherwise
	 */
	bool checkMissionFeasible(const mission_s &mission,
				  float max_distance_to_1st_waypoint, float max_distance_between_waypoints,
				  bool land_start_req);

};
