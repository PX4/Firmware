/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file CatapultLaunchMethod.h
 * Catpult Launch detection
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#ifndef CATAPULTLAUNCHMETHOD_H_
#define CATAPULTLAUNCHMETHOD_H_

#include "LaunchMethod.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

namespace launchdetection
{

class CatapultLaunchMethod : public LaunchMethod, public ModuleParams
{
public:
	CatapultLaunchMethod(ModuleParams *parent);
	~CatapultLaunchMethod() override = default;

	void update(const float dt, float accel_x) override;
	LaunchDetectionResult getLaunchDetected() const override;
	void reset() override;
	float getPitchMax(float pitchMaxDefault) override;

private:
	float _integrator{0.0f};
	float _motorDelayCounter{0.0f};

	LaunchDetectionResult state{LAUNCHDETECTION_RES_NONE};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::LAUN_CAT_A>) _param_laun_cat_a,
		(ParamFloat<px4::params::LAUN_CAT_T>) _param_laun_cat_t,
		(ParamFloat<px4::params::LAUN_CAT_MDEL>) _param_laun_cat_mdel,
		(ParamFloat<px4::params::LAUN_CAT_PMAX>) _param_laun_cat_pmax /**< Upper pitch limit before throttle is turned on.
						       Can be used to make sure that the AC does not climb
						       too much while attached to a bungee */
	)

};

#endif /* CATAPULTLAUNCHMETHOD_H_ */

} // namespace launchdetection
