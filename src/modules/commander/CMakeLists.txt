############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

add_subdirectory(failure_detector)
add_subdirectory(Arming)

px4_add_module(
	MODULE modules__commander
	MAIN commander
	COMPILE_FLAGS
	SRCS
		accelerometer_calibration.cpp
		airspeed_calibration.cpp
		calibration_routines.cpp
		Commander.cpp
		commander_helper.cpp
		esc_calibration.cpp
		factory_calibration_storage.cpp
		gyro_calibration.cpp
		level_calibration.cpp
		lm_fit.cpp
		mag_calibration.cpp
		ManualControl.cpp
		rc_calibration.cpp
		state_machine_helper.cpp
		worker_thread.cpp
	DEPENDS
		circuit_breaker
		failure_detector
		git_ecl
		ecl_geo
		hysteresis
		PreFlightCheck
		ArmAuthorization
		HealthFlags
		sensor_calibration
	)

if(PX4_TESTING)
	add_subdirectory(commander_tests)
endif()

px4_add_unit_gtest(SRC mag_calibration_test.cpp LINKLIBS modules__commander)
