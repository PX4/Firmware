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
 * @file launchDetection.cpp
 * Auto Detection for different launch methods (e.g. catapult)
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "CatapultLaunchMethod.h"
#include "LaunchDetector.h"

#include <px4_platform_common/log.h>

namespace launchdetection
{

LaunchDetector::LaunchDetector(ModuleParams *parent) :
	ModuleParams(parent)
{
	/* init all detectors */
	_launchMethods[0] = new CatapultLaunchMethod(this);
}

LaunchDetector::~LaunchDetector()
{
	delete _launchMethods[0];
}

void LaunchDetector::reset()
{
	/* Reset all detectors */
	for (const auto launchMethod : _launchMethods) {
		launchMethod->reset();
	}

	/* Reset active launchdetector */
	_activeLaunchDetectionMethodIndex = -1;
}

void LaunchDetector::update(const float dt, float accel_x)
{
	if (launchDetectionEnabled()) {
		for (const auto launchMethod : _launchMethods) {
			launchMethod->update(dt, accel_x);
		}
	}
}

LaunchDetectionResult LaunchDetector::getLaunchDetected()
{
	if (launchDetectionEnabled()) {
		if (_activeLaunchDetectionMethodIndex < 0) {
			/* None of the active _launchmethods has detected a launch, check all _launchmethods */
			for (unsigned i = 0; i < (sizeof(_launchMethods) / sizeof(_launchMethods[0])); i++) {
				if (_launchMethods[i]->getLaunchDetected() != LAUNCHDETECTION_RES_NONE) {
					PX4_INFO("selecting launchmethod %d", i);
					_activeLaunchDetectionMethodIndex = i; // from now on only check this method
					return _launchMethods[i]->getLaunchDetected();
				}
			}

		} else {
			return _launchMethods[_activeLaunchDetectionMethodIndex]->getLaunchDetected();
		}
	}

	return LAUNCHDETECTION_RES_NONE;
}

float LaunchDetector::getPitchMax(float pitchMaxDefault)
{
	if (!launchDetectionEnabled()) {
		return pitchMaxDefault;
	}

	/* if a lauchdetectionmethod is active or only one exists return the pitch limit from this method,
	 * otherwise use the default limit */
	if (_activeLaunchDetectionMethodIndex < 0) {
		if (sizeof(_launchMethods) / sizeof(LaunchMethod *) > 1) {
			return pitchMaxDefault;

		} else {
			return _launchMethods[0]->getPitchMax(pitchMaxDefault);
		}

	} else {
		return _launchMethods[_activeLaunchDetectionMethodIndex]->getPitchMax(pitchMaxDefault);
	}
}

} // namespace launchdetection
