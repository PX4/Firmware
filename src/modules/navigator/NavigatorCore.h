/***************************************************************************
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
 */

#include <px4_platform_common/module_params.h>

#include <lib/mathlib/mathlib.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>


class NavigatorCore : public ModuleParams
{

public:
	NavigatorCore();

	~NavigatorCore();

	typedef matrix::Vector3<float> Vector3f;
	typedef matrix::Vector2<double> Vector2d;

	bool getLanded() { return _landed_state.landed; }
	bool getMaybeLanded() { return _landed_state.maybe_landed; }
	float getLandDetectedAltMaxMeter() { return _landed_state.alt_max; }
	double getLatRad() { return _global_pos.lat; }
	double getLonRad() { return _global_pos.lon;}
	float getAltitudeAMSLMeters() { return _global_pos.alt; }
	float getTrueHeadingRad() { return _local_pos.heading; }
	double getHomeLatRad() { return _home.lat; }
	double getHomeLonRad() { return _home.lon; }
	float getHomeAltAMSLMeter() { return _home.alt; }
	float getHomeTrueHeadingRad() { return _home.yaw; }
	home_position_s &getHomePosition() { return _home; }

	bool isNotArmed() { return _status.arming_state != vehicle_status_s::ARMING_STATE_ARMED; }
	uint8_t getArmingState() { return _status.arming_state; }


	float getPosNorthMeter() { return _local_pos.x; }
	float getPosEastMeter() { return _local_pos.y; }
	float getPosDownMeter() { return _local_pos.z; }

	float getVelNorthMPS() { return _local_pos.vx; }
	float getVelEastMPS() { return _local_pos.vy; }
	float getVelDownMPS() { return _local_pos.vz; }

	float getLoiterRadiusMeter() { return _param_nav_loiter_rad.get(); }

	bool getIsInTransitionMode() { return _status.in_transition_mode; }

	position_setpoint_triplet_s getCurrentTriplet() { return _triplet; }

	bool isHomeValid() { return (_home.timestamp > 0 && _home.valid_alt && _home.valid_hpos && _home.valid_lpos); }
	bool isHomeAltValid() { return (_home.timestamp > 0 && _home.valid_alt);  }

	bool isRotaryWing() { return _status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING; }
	bool isFixedWing() { return _status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING; }
	bool isRover() { return _status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER; }
	bool isVTOL() { return _status.is_vtol; }
	uint8_t getVehicleType() { return _status.vehicle_type; }

	float getDefaultAltAcceptanceRadiusMeter();
	float getAltAcceptanceRadMeter();
	float getFixedWingLandingAltAcceptanceRadius() { return _param_nav_fw_altl_rad.get(); }
	float getDefaultHorAcceptanceRadiusMeter() { return _param_nav_acc_rad.get(); }
	float getHorAcceptanceRadiusMeter();
	float getMulticopterAltAcceptanceRadiusMeter() { return _param_nav_mc_alt_rad.get(); }
	float getFixedWingAltAcceptanceRadiusMeter() { return _param_nav_fw_alt_rad.get(); }

	float getAcceptanceRadiusMeter();
	float getAltAcceptanceRadiusMeter();

	float getRelativeTakeoffMinAltitudeMeter() { return _param_mis_takeoff_alt.get(); }
	float getRelativeLoiterMinAltitudeMeter() { return _param_mis_ltrmin_alt.get(); }
	bool isTakeoffRequired() { return _param_mis_takeoff_req.get(); }
	float getWaypointHeadingTimeoutSeconds() { return _param_mis_yaw_tmt.get(); }
	float getWaypointHeadingAcceptanceRad() { return _param_mis_yaw_err.get(); }


	bool forceVTOL();

	void updateLocalPosition(const vehicle_local_position_s &local_pos) { _local_pos = local_pos; };
	void updateVehicleStatus(const vehicle_status_s &status) { _status = status; }
	void updateGlobalPosition(const vehicle_global_position_s &global_pos) { _global_pos = global_pos; }
	void updateLandedState(const vehicle_land_detected_s &landed_state) { _landed_state = landed_state; }
	void updateHomePosition(const home_position_s &home) { _home = home; }
	void updatePositionControllerStatus(const position_controller_status_s &pos_ctrl_status) { _pos_ctrl_status = pos_ctrl_status; }

private:

	vehicle_local_position_s _local_pos{};
	vehicle_status_s _status{};
	vehicle_global_position_s _global_pos{};
	vehicle_land_detected_s _landed_state{};
	home_position_s _home{};
	position_controller_status_s _pos_ctrl_status{};

	position_setpoint_triplet_s _triplet{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_LOITER_RAD>) _param_nav_loiter_rad,	/**< loiter radius for fixedwing */
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_nav_acc_rad,	/**< acceptance for takeoff */
		(ParamFloat<px4::params::NAV_FW_ALT_RAD>)
		_param_nav_fw_alt_rad,	/**< acceptance radius for fixedwing altitude */
		(ParamFloat<px4::params::NAV_FW_ALTL_RAD>)
		_param_nav_fw_altl_rad,	/**< acceptance radius for fixedwing altitude before landing*/
		(ParamFloat<px4::params::NAV_MC_ALT_RAD>)
		_param_nav_mc_alt_rad,	/**< acceptance radius for multicopter altitude */
		(ParamInt<px4::params::NAV_FORCE_VT>) _param_nav_force_vt,	/**< acceptance radius for multicopter altitude */
		(ParamInt<px4::params::NAV_TRAFF_AVOID>) _param_nav_traff_avoid,	/**< avoiding other aircraft is enabled */
		(ParamFloat<px4::params::NAV_TRAFF_A_RADU>) _param_nav_traff_a_radu,	/**< avoidance Distance Unmanned*/
		(ParamFloat<px4::params::NAV_TRAFF_A_RADM>) _param_nav_traff_a_radm,	/**< avoidance Distance Manned*/

		// non-navigator parameters
		// Mission (MIS_*)
		(ParamFloat<px4::params::MIS_LTRMIN_ALT>) _param_mis_ltrmin_alt,
		(ParamFloat<px4::params::MIS_TAKEOFF_ALT>) _param_mis_takeoff_alt,
		(ParamBool<px4::params::MIS_TAKEOFF_REQ>) _param_mis_takeoff_req,
		(ParamFloat<px4::params::MIS_YAW_TMT>) _param_mis_yaw_tmt,
		(ParamFloat<px4::params::MIS_YAW_ERR>) _param_mis_yaw_err
	)
};
