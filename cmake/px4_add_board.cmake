############################################################################
#
# Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#=============================================================================
#
#	px4_add_board
#
#	This function creates a PX4 board.
#
#	Usage:
#		px4_add_board(
#			PLATFORM <string>
#			VENDOR <string>
#			MODEL <string>
#			[ LABEL <string> ]
#			[ TOOLCHAIN <string> ]
#			[ ARCHITECTURE <string> ]
#			[ ROMFSROOT <string> ]
#			[ BUILD_BOOTLOADER ]
#			[ IO <string> ]
#			[ UAVCAN_INTERFACES <string> ]
#			[ UAVCAN_PERIPHERALS <list> ]
#			[ DRIVERS <list> ]
#			[ MODULES <list> ]
#			[ SYSTEMCMDS <list> ]
#			[ EXAMPLES <list> ]
#			[ SERIAL_PORTS <list> ]
#			[ CONSTRAINED_FLASH ]
#			[ CONSTRAINED_MEMORY ]
#			[ EXTERNAL_METADATA ]
#			[ TESTING ]
#			[ LINKER_PREFIX <string> ]
#			[ ETHERNET ]
#			[ CRYPTO <string> ]
#			[ KEYSTORE <string> ]
#			)
#
#	Input:
#		PLATFORM		: PX4 platform name (posix, nuttx, qurt)
#		VENDOR			: name of board vendor/manufacturer/brand/etc
#		MODEL			: name of board model
#		LABEL			: optional label, set to default if not specified
#		TOOLCHAIN		: cmake toolchain
#		ARCHITECTURE		: name of the CPU CMake is building for (used by the toolchain)
#		ROMFSROOT		: relative path to the ROMFS root directory
#		BUILD_BOOTLOADER	: flag to enable building and including the bootloader config
#		IO			: name of IO board to be built and included in the ROMFS (requires a valid ROMFSROOT)
#		UAVCAN_INTERFACES	: number of interfaces for UAVCAN
#		UAVCAN_PERIPHERALS      : list of UAVCAN peripheral firmware to build and embed
#		DRIVERS			: list of drivers to build for this board (relative to src/drivers)
#		MODULES			: list of modules to build for this board (relative to src/modules)
#		SYSTEMCMDS		: list of system commands to build for this board (relative to src/systemcmds)
#		EXAMPLES		: list of example modules to build for this board (relative to src/examples)
#		SERIAL_PORTS		: mapping of user configurable serial ports and param facing name
#		CONSTRAINED_FLASH	: flag to enable constrained flash options (eg limit init script status text)
#		CONSTRAINED_MEMORY	: flag to enable constrained memory options (eg limit maximum number of uORB publications)
#		EXTERNAL_METADATA	: flag to exclude metadata to reduce flash
#		TESTING			: flag to enable automatic inclusion of PX4 testing modules
#		LINKER_PREFIX	: optional to prefix on the Linker script.
#		ETHERNET		: flag to indicate that ethernet is enabled
#		CRYPTO			: Crypto implementation selection
#		KEYSTORE		: Keystore implememntation selection
#
#
#	Example:
#		px4_add_board(
#			PLATFORM nuttx
#			VENDOR px4
#			MODEL fmu-v5
#			TOOLCHAIN arm-none-eabi
#			ARCHITECTURE cortex-m7
#			ROMFSROOT px4fmu_common
#			IO px4_io-v2_default
#			SERIAL_PORTS
#				GPS1:/dev/ttyS0
#				TEL1:/dev/ttyS1
#				TEL2:/dev/ttyS2
#				TEL4:/dev/ttyS3
#			DRIVERS
#				barometer/ms5611
#				gps
#				imu/bosch/bmi055
#				imu/invensense/mpu6000
#				magnetometer/isentek/ist8310
#				pwm_out
#				px4io
#				rgbled
#			MODULES
#				commander
#				ekf2
#				land_detector
#				mavlink
#				mc_att_control
#				mc_pos_control
#				navigator
#				sensors
#			MODULES
#				mixer
#				mtd
#				param
#				perf
#				pwm
#				reboot
#				shutdown
#				top
#				topic_listener
#				tune_control
#			)
#
function(px4_add_board)

	px4_parse_function_args(
		NAME px4_add_board
		ONE_VALUE
			PLATFORM
			VENDOR
			MODEL
			LABEL
			TOOLCHAIN
			ARCHITECTURE
			NUTTX_CONFIG
			ROMFSROOT
			IO
			UAVCAN_INTERFACES
			UAVCAN_TIMER_OVERRIDE
			LINKER_PREFIX
			CRYPTO
			KEYSTORE
		MULTI_VALUE
			DRIVERS
			MODULES
			SYSTEMCMDS
			EXAMPLES
			SERIAL_PORTS
			UAVCAN_PERIPHERALS
		OPTIONS
			BUILD_BOOTLOADER
			CONSTRAINED_FLASH
			CONSTRAINED_MEMORY
			EXTERNAL_METADATA
			TESTING
			ETHERNET
		REQUIRED
			PLATFORM
			VENDOR
			MODEL
		ARGN ${ARGN})

	set(PX4_BOARD_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE STRING "PX4 board directory" FORCE)
	include_directories(${PX4_BOARD_DIR}/src)

	set(PX4_BOARD ${VENDOR}_${MODEL} CACHE STRING "PX4 board" FORCE)

	# board name is uppercase with no underscores when used as a define
	string(TOUPPER ${PX4_BOARD} PX4_BOARD_NAME)
	string(REPLACE "-" "_" PX4_BOARD_NAME ${PX4_BOARD_NAME})
	set(PX4_BOARD_NAME ${PX4_BOARD_NAME} CACHE STRING "PX4 board define" FORCE)

	set(PX4_BOARD_VENDOR ${VENDOR} CACHE STRING "PX4 board vendor" FORCE)
	set(PX4_BOARD_MODEL ${MODEL} CACHE STRING "PX4 board model" FORCE)

	if(LABEL)
		set(PX4_BOARD_LABEL ${LABEL} CACHE STRING "PX4 board label" FORCE)
	else()
		set(PX4_BOARD_LABEL "default" CACHE STRING "PX4 board label" FORCE)
	endif()

	set(PX4_CONFIG "${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_${PX4_BOARD_LABEL}" CACHE STRING "PX4 config" FORCE)

	# set OS, and append specific platform module path
	set(PX4_PLATFORM ${PLATFORM} CACHE STRING "PX4 board OS" FORCE)
	list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake)

	# platform-specific include path
	include_directories(${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/src/px4/common/include)

	if(ARCHITECTURE)
		set(CMAKE_SYSTEM_PROCESSOR ${ARCHITECTURE} CACHE INTERNAL "system processor" FORCE)
	endif()

	if(TOOLCHAIN)
		set(CMAKE_TOOLCHAIN_FILE Toolchain-${TOOLCHAIN} CACHE INTERNAL "toolchain file" FORCE)
	endif()

	set(romfs_extra_files)
	set(config_romfs_extra_dependencies)
	# additional embedded metadata
	if (NOT CONSTRAINED_FLASH AND NOT EXTERNAL_METADATA AND NOT ${PX4_BOARD_LABEL} STREQUAL "test")
		list(APPEND romfs_extra_files
			${PX4_BINARY_DIR}/parameters.json.xz
			${PX4_BINARY_DIR}/events/all_events.json.xz)
		list(APPEND romfs_extra_dependencies
			parameters_xml
			events_json)
	endif()
	list(APPEND romfs_extra_files ${PX4_BINARY_DIR}/component_general.json.xz)
	list(APPEND romfs_extra_dependencies component_general_json)
	set(config_romfs_extra_files ${romfs_extra_files} CACHE INTERNAL "extra ROMFS files" FORCE)
	set(config_romfs_extra_dependencies ${romfs_extra_dependencies} CACHE INTERNAL "extra ROMFS deps" FORCE)

	if(SERIAL_PORTS)
		set(board_serial_ports ${SERIAL_PORTS} PARENT_SCOPE)
	endif()

	# ROMFS
	if(ROMFSROOT)
		set(config_romfs_root ${ROMFSROOT} CACHE INTERNAL "ROMFS root" FORCE)

		if(BUILD_BOOTLOADER)
			set(config_build_bootloader "1" CACHE INTERNAL "build bootloader" FORCE)
		endif()

		# IO board (placed in ROMFS)
		if(IO)
			set(config_io_board ${IO} CACHE INTERNAL "IO" FORCE)
		endif()

		if(UAVCAN_PERIPHERALS)
			set(config_uavcan_peripheral_firmware ${UAVCAN_PERIPHERALS} CACHE INTERNAL "UAVCAN peripheral firmware" FORCE)
		endif()
	endif()

	if(UAVCAN_INTERFACES)
		set(config_uavcan_num_ifaces ${UAVCAN_INTERFACES} CACHE INTERNAL "UAVCAN interfaces" FORCE)
	endif()

	if(UAVCAN_TIMER_OVERRIDE)
		set(config_uavcan_timer_override ${UAVCAN_TIMER_OVERRIDE} CACHE INTERNAL "UAVCAN TIMER OVERRIDE" FORCE)
	endif()

	# OPTIONS

	if(CONSTRAINED_FLASH)
		set(px4_constrained_flash_build "1" CACHE INTERNAL "constrained flash build" FORCE)
		add_definitions(-DCONSTRAINED_FLASH)
	endif()

	if(CONSTRAINED_MEMORY)
		set(px4_constrained_memory_build "1" CACHE INTERNAL "constrained memory build" FORCE)
		add_definitions(-DCONSTRAINED_MEMORY)
	endif()

	if(TESTING)
		set(PX4_TESTING "1" CACHE INTERNAL "testing enabled" FORCE)
	endif()

	if(ETHERNET)
		set(PX4_ETHERNET "1" CACHE INTERNAL "ethernet enabled" FORCE)
	endif()

	if(CRYPTO)
		set(PX4_CRYPTO ${CRYPTO} CACHE STRING "PX4 crypto implementation" FORCE)
	endif()

	if(KEYSTORE)
		set(PX4_KEYSTORE ${KEYSTORE} CACHE STRING "PX4 keystore implementation" FORCE)
	endif()

	if(LINKER_PREFIX)
		set(PX4_BOARD_LINKER_PREFIX ${LINKER_PREFIX} CACHE STRING "PX4 board linker prefix" FORCE)
	else()
		set(PX4_BOARD_LINKER_PREFIX "" CACHE STRING "PX4 board linker prefix" FORCE)
	endif()

	include(px4_impl_os)
	px4_os_prebuild_targets(OUT prebuild_targets BOARD ${PX4_BOARD})


	###########################################################################
	# Modules (includes drivers, examples, modules, systemcmds)
	set(config_module_list)

	if(DRIVERS)
		foreach(driver ${DRIVERS})
			list(APPEND config_module_list drivers/${driver})
		endforeach()
	endif()

	if(MODULES)
		foreach(module ${MODULES})
			list(APPEND config_module_list modules/${module})
		endforeach()
	endif()

	if(SYSTEMCMDS)
		foreach(systemcmd ${SYSTEMCMDS})
			list(APPEND config_module_list systemcmds/${systemcmd})
		endforeach()
	endif()

	if(EXAMPLES)
		foreach(example ${EXAMPLES})
			list(APPEND config_module_list examples/${example})
		endforeach()
	endif()

	# add board config directory src to build modules
	file(RELATIVE_PATH board_support_src_rel ${PX4_SOURCE_DIR}/src ${PX4_BOARD_DIR})
	list(APPEND config_module_list ${board_support_src_rel}/src)

	set(config_module_list ${config_module_list} PARENT_SCOPE)

endfunction()
