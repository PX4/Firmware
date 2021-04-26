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
 * @file ActuatorEffectivenessMultirotor.hpp
 *
 * Actuator effectiveness computed from rotors position and orientation
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include "ActuatorEffectiveness.hpp"

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

class ActuatorEffectivenessMultirotor: public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessMultirotor();
	virtual ~ActuatorEffectivenessMultirotor() = default;

	static constexpr int NUM_ROTORS_MAX = 8;

	typedef struct {
		float position_x;
		float position_y;
		float position_z;
		float axis_x;
		float axis_y;
		float axis_z;
		float thrust_coef;
		float moment_ratio;
	} RotorGeometry;

	typedef struct {
		RotorGeometry rotors[NUM_ROTORS_MAX];
	} MultirotorGeometry;

	static int computeEffectivenessMatrix(const MultirotorGeometry &geometry,
					      matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness);

	bool getEffectivenessMatrix(matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix) override;

	int numActuators() const override { return _num_actuators; }
private:
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	bool _updated{true};
	int _num_actuators{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_MC_R0_PX>) _param_ca_mc_r0_px,
		(ParamFloat<px4::params::CA_MC_R0_PY>) _param_ca_mc_r0_py,
		(ParamFloat<px4::params::CA_MC_R0_PZ>) _param_ca_mc_r0_pz,
		(ParamFloat<px4::params::CA_MC_R0_AX>) _param_ca_mc_r0_ax,
		(ParamFloat<px4::params::CA_MC_R0_AY>) _param_ca_mc_r0_ay,
		(ParamFloat<px4::params::CA_MC_R0_AZ>) _param_ca_mc_r0_az,
		(ParamFloat<px4::params::CA_MC_R0_CT>) _param_ca_mc_r0_ct,
		(ParamFloat<px4::params::CA_MC_R0_KM>) _param_ca_mc_r0_km,

		(ParamFloat<px4::params::CA_MC_R1_PX>) _param_ca_mc_r1_px,
		(ParamFloat<px4::params::CA_MC_R1_PY>) _param_ca_mc_r1_py,
		(ParamFloat<px4::params::CA_MC_R1_PZ>) _param_ca_mc_r1_pz,
		(ParamFloat<px4::params::CA_MC_R1_AX>) _param_ca_mc_r1_ax,
		(ParamFloat<px4::params::CA_MC_R1_AY>) _param_ca_mc_r1_ay,
		(ParamFloat<px4::params::CA_MC_R1_AZ>) _param_ca_mc_r1_az,
		(ParamFloat<px4::params::CA_MC_R1_CT>) _param_ca_mc_r1_ct,
		(ParamFloat<px4::params::CA_MC_R1_KM>) _param_ca_mc_r1_km,

		(ParamFloat<px4::params::CA_MC_R2_PX>) _param_ca_mc_r2_px,
		(ParamFloat<px4::params::CA_MC_R2_PY>) _param_ca_mc_r2_py,
		(ParamFloat<px4::params::CA_MC_R2_PZ>) _param_ca_mc_r2_pz,
		(ParamFloat<px4::params::CA_MC_R2_AX>) _param_ca_mc_r2_ax,
		(ParamFloat<px4::params::CA_MC_R2_AY>) _param_ca_mc_r2_ay,
		(ParamFloat<px4::params::CA_MC_R2_AZ>) _param_ca_mc_r2_az,
		(ParamFloat<px4::params::CA_MC_R2_CT>) _param_ca_mc_r2_ct,
		(ParamFloat<px4::params::CA_MC_R2_KM>) _param_ca_mc_r2_km,

		(ParamFloat<px4::params::CA_MC_R3_PX>) _param_ca_mc_r3_px,
		(ParamFloat<px4::params::CA_MC_R3_PY>) _param_ca_mc_r3_py,
		(ParamFloat<px4::params::CA_MC_R3_PZ>) _param_ca_mc_r3_pz,
		(ParamFloat<px4::params::CA_MC_R3_AX>) _param_ca_mc_r3_ax,
		(ParamFloat<px4::params::CA_MC_R3_AY>) _param_ca_mc_r3_ay,
		(ParamFloat<px4::params::CA_MC_R3_AZ>) _param_ca_mc_r3_az,
		(ParamFloat<px4::params::CA_MC_R3_CT>) _param_ca_mc_r3_ct,
		(ParamFloat<px4::params::CA_MC_R3_KM>) _param_ca_mc_r3_km,

		(ParamFloat<px4::params::CA_MC_R4_PX>) _param_ca_mc_r4_px,
		(ParamFloat<px4::params::CA_MC_R4_PY>) _param_ca_mc_r4_py,
		(ParamFloat<px4::params::CA_MC_R4_PZ>) _param_ca_mc_r4_pz,
		(ParamFloat<px4::params::CA_MC_R4_AX>) _param_ca_mc_r4_ax,
		(ParamFloat<px4::params::CA_MC_R4_AY>) _param_ca_mc_r4_ay,
		(ParamFloat<px4::params::CA_MC_R4_AZ>) _param_ca_mc_r4_az,
		(ParamFloat<px4::params::CA_MC_R4_CT>) _param_ca_mc_r4_ct,
		(ParamFloat<px4::params::CA_MC_R4_KM>) _param_ca_mc_r4_km,

		(ParamFloat<px4::params::CA_MC_R5_PX>) _param_ca_mc_r5_px,
		(ParamFloat<px4::params::CA_MC_R5_PY>) _param_ca_mc_r5_py,
		(ParamFloat<px4::params::CA_MC_R5_PZ>) _param_ca_mc_r5_pz,
		(ParamFloat<px4::params::CA_MC_R5_AX>) _param_ca_mc_r5_ax,
		(ParamFloat<px4::params::CA_MC_R5_AY>) _param_ca_mc_r5_ay,
		(ParamFloat<px4::params::CA_MC_R5_AZ>) _param_ca_mc_r5_az,
		(ParamFloat<px4::params::CA_MC_R5_CT>) _param_ca_mc_r5_ct,
		(ParamFloat<px4::params::CA_MC_R5_KM>) _param_ca_mc_r5_km,

		(ParamFloat<px4::params::CA_MC_R6_PX>) _param_ca_mc_r6_px,
		(ParamFloat<px4::params::CA_MC_R6_PY>) _param_ca_mc_r6_py,
		(ParamFloat<px4::params::CA_MC_R6_PZ>) _param_ca_mc_r6_pz,
		(ParamFloat<px4::params::CA_MC_R6_AX>) _param_ca_mc_r6_ax,
		(ParamFloat<px4::params::CA_MC_R6_AY>) _param_ca_mc_r6_ay,
		(ParamFloat<px4::params::CA_MC_R6_AZ>) _param_ca_mc_r6_az,
		(ParamFloat<px4::params::CA_MC_R6_CT>) _param_ca_mc_r6_ct,
		(ParamFloat<px4::params::CA_MC_R6_KM>) _param_ca_mc_r6_km,

		(ParamFloat<px4::params::CA_MC_R7_PX>) _param_ca_mc_r7_px,
		(ParamFloat<px4::params::CA_MC_R7_PY>) _param_ca_mc_r7_py,
		(ParamFloat<px4::params::CA_MC_R7_PZ>) _param_ca_mc_r7_pz,
		(ParamFloat<px4::params::CA_MC_R7_AX>) _param_ca_mc_r7_ax,
		(ParamFloat<px4::params::CA_MC_R7_AY>) _param_ca_mc_r7_ay,
		(ParamFloat<px4::params::CA_MC_R7_AZ>) _param_ca_mc_r7_az,
		(ParamFloat<px4::params::CA_MC_R7_CT>) _param_ca_mc_r7_ct,
		(ParamFloat<px4::params::CA_MC_R7_KM>) _param_ca_mc_r7_km
	)
};
