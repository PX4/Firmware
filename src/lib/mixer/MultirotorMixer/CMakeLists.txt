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

set(geometry_files
	dodeca_bottom_cox.toml
	dodeca_top_cox.toml
	hex_cox.toml
	hex_plus.toml
	hex_t.toml
	hex_x.toml
	octa_cox.toml
	octa_cox_wide.toml
	octa_plus.toml
	octa_x.toml
	quad_deadcat.toml
	quad_h.toml
	quad_plus.toml
	quad_s250aq.toml
	quad_vtail.toml
	quad_wide.toml
	quad_x_cw.toml
	quad_x.toml
	quad_x_pusher.toml
	quad_y.toml
	tri_y.toml
	twin_engine.toml
)

set(geometries_list)
foreach(geom_file ${geometry_files})
	list(APPEND geometries_list ${CMAKE_CURRENT_SOURCE_DIR}/geometries/${geom_file})
endforeach()

set(MIXER_TOOLS ${CMAKE_CURRENT_SOURCE_DIR}/geometries/tools)

# generate mixers and normalize
add_custom_command(
	OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/mixer_multirotor.generated.h
	COMMAND ${PYTHON_EXECUTABLE} ${MIXER_TOOLS}/px_generate_mixers.py -f ${geometries_list} -o mixer_multirotor.generated.h
	DEPENDS ${MIXER_TOOLS}/px_generate_mixers.py ${geometries_list}
	)
add_custom_command(
	OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/mixer_multirotor_normalized.generated.h
	COMMAND ${PYTHON_EXECUTABLE} ${MIXER_TOOLS}/px_generate_mixers.py --normalize -f ${geometries_list} -o mixer_multirotor_normalized.generated.h
	DEPENDS ${MIXER_TOOLS}/px_generate_mixers.py ${geometries_list}
	)
add_custom_target(mixer_gen DEPENDS mixer_multirotor.generated.h ${CMAKE_CURRENT_BINARY_DIR}/mixer_multirotor_normalized.generated.h)

# 6DOF mixers
add_custom_command(
	OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/mixer_multirotor_6dof.generated.h
	COMMAND ${PYTHON_EXECUTABLE} ${MIXER_TOOLS}/px_generate_mixers.py --sixdof -f ${geometries_list} -o mixer_multirotor_6dof.generated.h
	DEPENDS ${MIXER_TOOLS}/px_generate_mixers.py ${geometries_list}
	)
add_custom_target(mixer_gen_6dof DEPENDS mixer_multirotor_6dof.generated.h)

add_library(MultirotorMixer
	MultirotorMixer.cpp
	MultirotorMixer.hpp

	${CMAKE_CURRENT_BINARY_DIR}/mixer_multirotor.generated.h
	${CMAKE_CURRENT_BINARY_DIR}/mixer_multirotor_normalized.generated.h
	${CMAKE_CURRENT_BINARY_DIR}/mixer_multirotor_6dof.generated.h
)
target_link_libraries(MultirotorMixer PRIVATE MixerBase)
target_include_directories(MultirotorMixer PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
add_dependencies(MultirotorMixer mixer_gen mixer_gen_6dof prebuild_targets)

if(BUILD_TESTING)

	add_executable(test_mixer_multirotor
		test_mixer_multirotor.cpp
		MultirotorMixer.cpp
	)
	target_compile_definitions(test_mixer_multirotor PRIVATE MIXER_MULTIROTOR_USE_MOCK_GEOMETRY)
	target_compile_options(test_mixer_multirotor PRIVATE -Wno-unused-result)
	target_link_libraries(test_mixer_multirotor PRIVATE MixerBase)

	add_test(NAME mixer_multirotor
		COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/mixer_multirotor.py --test --mixer-multirotor-binary $<TARGET_FILE:test_mixer_multirotor>
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	)

endif()
