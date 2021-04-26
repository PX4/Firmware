############################################################################
#
#   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

px4_add_git_submodule(TARGET git_ecl PATH "ecl")
px4_add_git_submodule(TARGET git_matrix PATH "matrix")

add_subdirectory(airspeed)
add_subdirectory(airspeed_validator)
add_subdirectory(avoidance)
add_subdirectory(battery)
add_subdirectory(bezier)
add_subdirectory(cdev)
add_subdirectory(circuit_breaker)
add_subdirectory(collision_prevention)
add_subdirectory(controllib)
add_subdirectory(component_information)
add_subdirectory(conversion)
add_subdirectory(drivers)
add_subdirectory(ecl)
add_subdirectory(hysteresis)
add_subdirectory(l1)
add_subdirectory(landing_slope)
add_subdirectory(led)
add_subdirectory(magnetometer_bias_estimator)
add_subdirectory(mathlib)
add_subdirectory(mixer)
add_subdirectory(mixer_module)
add_subdirectory(motion_planning)
add_subdirectory(output_limit)
add_subdirectory(perf)
add_subdirectory(pid)
add_subdirectory(pwm)
add_subdirectory(rc)
add_subdirectory(sensor_calibration)
add_subdirectory(slew_rate)
add_subdirectory(systemlib)
add_subdirectory(tecs)
add_subdirectory(terrain_estimation)
add_subdirectory(tunes)
add_subdirectory(version)
add_subdirectory(weather_vane)
