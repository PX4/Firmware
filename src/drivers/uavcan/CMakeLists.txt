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

set(LIBUAVCAN_DIR ${PX4_SOURCE_DIR}/src/drivers/uavcan/libuavcan)
set(LIBUAVCAN_DIR_DRIVERS ${PX4_SOURCE_DIR}/src/drivers/uavcan/uavcan_drivers)

px4_add_git_submodule(TARGET git_uavcan PATH ${LIBUAVCAN_DIR})

set(UAVCAN_USE_CPP03 ON CACHE BOOL "uavcan cpp03")
set(UAVCAN_PLATFORM "generic")

if(CONFIG_ARCH_CHIP)
	if(${CONFIG_ARCH_CHIP} MATCHES "kinetis")
		set(UAVCAN_DRIVER "kinetis")
		set(UAVCAN_TIMER 1)
	elseif(${CONFIG_ARCH_CHIP} MATCHES "stm32h7")
		set(UAVCAN_DRIVER "stm32h7")
		set(UAVCAN_TIMER 5) # The default timer is TIM5
		if (DEFINED config_uavcan_timer_override)
			set (UAVCAN_TIMER  ${config_uavcan_timer_override})
		endif()
	elseif(${CONFIG_ARCH_CHIP} MATCHES "stm32")
		set(UAVCAN_DRIVER "stm32")
		set(UAVCAN_TIMER 5) # The default timer is TIM5
	endif()
endif()

if(NOT DEFINED UAVCAN_DRIVER)
	message(FATAL_ERROR "UAVCAN_DRIVER not set")
endif()

if(NOT config_uavcan_num_ifaces)
	message(FATAL_ERROR "config_uavcan_num_ifaces not set")
endif()

string(TOUPPER "${PX4_PLATFORM}" OS_UPPER)
string(TOUPPER "${UAVCAN_DRIVER}" UAVCAN_DRIVER_UPPER)
add_definitions(
	-DUAVCAN_${UAVCAN_DRIVER_UPPER}_${OS_UPPER}=1
	-DUAVCAN_${UAVCAN_DRIVER_UPPER}_NUM_IFACES=${config_uavcan_num_ifaces}
	-DUAVCAN_${UAVCAN_DRIVER_UPPER}_TIMER_NUMBER=${UAVCAN_TIMER}
	-DUAVCAN_CPP_VERSION=UAVCAN_CPP03
	-DUAVCAN_DRIVER=uavcan_${UAVCAN_DRIVER}
	-DUAVCAN_IMPLEMENT_PLACEMENT_NEW=1
	-DUAVCAN_MEM_POOL_BLOCK_SIZE=48
	-DUAVCAN_NO_ASSERTIONS
	-DUAVCAN_PLATFORM=${UAVCAN_PLATFORM}
)

add_compile_options(
	-Wno-cast-align # TODO: fix and enable
	-Wno-deprecated-copy # TODO: fix
	-Wno-address-of-packed-member
)
add_subdirectory(${LIBUAVCAN_DIR} libuavcan EXCLUDE_FROM_ALL)
add_dependencies(uavcan prebuild_targets)

# driver
add_subdirectory(${LIBUAVCAN_DIR_DRIVERS}/${UAVCAN_DRIVER}/driver libuavcan_drivers EXCLUDE_FROM_ALL)
target_include_directories(uavcan_${UAVCAN_DRIVER}_driver PUBLIC
	${LIBUAVCAN_DIR}/libuavcan/include
	${LIBUAVCAN_DIR}/libuavcan/include/dsdlc_generated
)


# generated DSDL
set(DSDLC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dsdl")
set(DSDLC_INPUTS
	"${DSDLC_DIR}/com"
	"${DSDLC_DIR}/ardupilot"
	"${LIBUAVCAN_DIR}/dsdl/uavcan"
	"${DSDLC_DIR}/cuav"
)
set(DSDLC_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/include/dsdlc_generated")

set(DSDLC_INPUT_FILES)
foreach(DSDLC_INPUT ${DSDLC_INPUTS})
	file(GLOB_RECURSE DSDLC_NEW_INPUT_FILES ${CMAKE_CURRENT_SOURCE_DIR} "${DSDLC_INPUT}/*.uavcan")
	list(APPEND DSDLC_INPUT_FILES ${DSDLC_NEW_INPUT_FILES})
endforeach(DSDLC_INPUT)
add_custom_command(OUTPUT px4_uavcan_dsdlc_run.stamp
	COMMAND ${PYTHON_EXECUTABLE} ${LIBUAVCAN_DIR}/libuavcan/dsdl_compiler/libuavcan_dsdlc ${DSDLC_INPUTS} -O${DSDLC_OUTPUT}
	COMMAND ${CMAKE_COMMAND} -E touch px4_uavcan_dsdlc_run.stamp
	DEPENDS ${DSDLC_INPUT_FILES}
	COMMENT "PX4 UAVCAN dsdl compiler"
)
add_custom_target(px4_uavcan_dsdlc DEPENDS px4_uavcan_dsdlc_run.stamp)

px4_add_module(
	MODULE drivers__uavcan
	MAIN uavcan
	STACK_MAIN 4096
	INCLUDES
		${DSDLC_OUTPUT}
		${LIBUAVCAN_DIR}/libuavcan/include
		${LIBUAVCAN_DIR}/libuavcan/include/dsdlc_generated
		${LIBUAVCAN_DIR}/libuavcan_drivers/posix/include
		${LIBUAVCAN_DIR_DRIVERS}/${UAVCAN_DRIVER}/driver/include
	SRCS
		beep.cpp
		beep.hpp
		rgbled.cpp
		rgbled.hpp
		safety_state.cpp
		safety_state.hpp

		# Main
		uavcan_main.cpp
		uavcan_servers.cpp

		# Actuators
		actuators/esc.cpp
		actuators/hardpoint.cpp

		# Sensors
		sensors/sensor_bridge.cpp
		sensors/differential_pressure.cpp
		sensors/baro.cpp
		sensors/battery.cpp
		sensors/airspeed.cpp
		sensors/flow.cpp
		sensors/gnss.cpp
		sensors/mag.cpp
		sensors/rangefinder.cpp
		sensors/accel.cpp
		sensors/gyro.cpp
		sensors/cbat.cpp

	DEPENDS
		px4_uavcan_dsdlc

		led
		mixer
		mixer_module
		output_limit
		version

		git_uavcan
		uavcan_${UAVCAN_DRIVER}_driver

		# within libuavcan
		uavcan
	)
