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

px4_add_git_submodule(TARGET git_uavcan PATH "libuavcan")

set(UAVCAN_USE_CPP03 ON CACHE BOOL "uavcan cpp03")

if(CONFIG_ARCH_CHIP)
	if (${CONFIG_ARCH_CHIP} MATCHES "kinetis")
		set(UAVCAN_PLATFORM "kinetis")
		set(UAVCAN_TIMER 1)
	elseif (${CONFIG_ARCH_CHIP} MATCHES "stm32")
		set(UAVCAN_PLATFORM "stm32")
		set(UAVCAN_TIMER 5) # The default timer the 5
	endif()
endif()

if(NOT DEFINED UAVCAN_PLATFORM)
	message(FATAL_ERROR "UAVCAN_PLATFORM not set")
endif()

if(NOT config_uavcan_num_ifaces)
	message(FATAL_ERROR "config_uavcan_num_ifaces not set")
endif()

string(TOUPPER "${PX4_PLATFORM}" OS_UPPER)
string(TOUPPER "${UAVCAN_PLATFORM}" UAVCAN_PLATFORM_UPPER)
add_definitions(
		-DUAVCAN_${UAVCAN_PLATFORM_UPPER}_${OS_UPPER}=1
		-DUAVCAN_${UAVCAN_PLATFORM_UPPER}_NUM_IFACES=${config_uavcan_num_ifaces}
		-DUAVCAN_${UAVCAN_PLATFORM_UPPER}_TIMER_NUMBER=${UAVCAN_TIMER}
		-DUAVCAN_CPP_VERSION=UAVCAN_CPP03
		-DUAVCAN_DRIVER=uavcan_${UAVCAN_PLATFORM}
		-DUAVCAN_IMPLEMENT_PLACEMENT_NEW=1
		-DUAVCAN_MEM_POOL_BLOCK_SIZE=48
		-DUAVCAN_NO_ASSERTIONS
		-DUAVCAN_PLATFORM=${UAVCAN_PLATFORM}
		)

add_compile_options(-Wno-cast-align) # TODO: fix and enable
add_subdirectory(libuavcan EXCLUDE_FROM_ALL)
add_dependencies(uavcan prebuild_targets)


# generated DSDL
set(DSDLC_INPUTS "${CMAKE_CURRENT_SOURCE_DIR}/dsdl/com" "${CMAKE_CURRENT_SOURCE_DIR}/libuavcan/dsdl/uavcan")
set(DSDLC_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/include/dsdlc_generated")

set(DSDLC_INPUT_FILES)
foreach(DSDLC_INPUT ${DSDLC_INPUTS})
	file(GLOB_RECURSE DSDLC_NEW_INPUT_FILES ${CMAKE_CURRENT_SOURCE_DIR} "${DSDLC_INPUT}/*.uavcan")
	list(APPEND DSDLC_INPUT_FILES ${DSDLC_NEW_INPUT_FILES})
endforeach(DSDLC_INPUT)
add_custom_command(OUTPUT px4_uavcan_dsdlc_run.stamp
                   COMMAND ${PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/libuavcan/libuavcan/dsdl_compiler/libuavcan_dsdlc ${DSDLC_INPUTS} -O${DSDLC_OUTPUT}
                   COMMAND ${CMAKE_COMMAND} -E touch px4_uavcan_dsdlc_run.stamp
                   DEPENDS ${DSDLC_INPUT_FILES}
		   COMMENT "PX4 UAVCAN dsdl compiler"
		   )
add_custom_target(px4_uavcan_dsdlc DEPENDS px4_uavcan_dsdlc_run.stamp)

px4_add_module(
	MODULE modules__uavcan
	MAIN uavcan
	INCLUDES
		${DSDLC_OUTPUT}
		${PX4_SOURCE_DIR}/mavlink/include/mavlink
		libuavcan/libuavcan/include
		libuavcan/libuavcan/include/dsdlc_generated
		libuavcan/libuavcan_drivers/posix/include
		libuavcan/libuavcan_drivers/${UAVCAN_PLATFORM}/driver/include
	SRCS
		# Main
		uavcan_main.cpp
		uavcan_servers.cpp
		uavcan_params.c

		# Actuators
		actuators/esc.cpp
		actuators/hardpoint.cpp
		actuators/servo.cpp

		# Sensors
		sensors/sensor_bridge.cpp
		sensors/gnss.cpp
		sensors/mag.cpp
		sensors/baro.cpp
		sensors/flow.cpp
		sensors/battery.cpp

	DEPENDS
		px4_uavcan_dsdlc

		mixer
		version

		git_uavcan

		# within libuavcan
		uavcan
		uavcan_${UAVCAN_PLATFORM}_driver
	)
