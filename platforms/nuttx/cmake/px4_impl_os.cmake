############################################################################
#
# Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
#	Defined functions in this file
#
#	Required OS Interface Functions
#
#		* px4_os_add_flags
# 		* px4_os_determine_build_chip
#		* px4_os_prebuild_targets
#

#=============================================================================
#
#	px4_os_add_flags
#
#	Set the nuttx build flags.
#
function(px4_os_add_flags)

	include_directories(BEFORE SYSTEM
		${PX4_BINARY_DIR}/NuttX/nuttx/include
		${PX4_BINARY_DIR}/NuttX/nuttx/include/cxx
		${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/include/cxx	# custom new
	)

	include_directories(
		${PX4_BINARY_DIR}/NuttX/nuttx/arch/${CONFIG_ARCH}/src/${CONFIG_ARCH_FAMILY}
		${PX4_BINARY_DIR}/NuttX/nuttx/arch/${CONFIG_ARCH}/src/chip
		${PX4_BINARY_DIR}/NuttX/nuttx/arch/${CONFIG_ARCH}/src/common

		${PX4_BINARY_DIR}/NuttX/apps/include
	)

	# prevent using the toolchain's std c++ library
	add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-nostdinc++>)

	add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-sized-deallocation>)

	add_definitions(
		-D__PX4_NUTTX

		-D_SYS_CDEFS_H_ # skip toolchain's <sys/cdefs.h>
		-D_SYS_REENT_H_	# skip toolchain's <sys/reent.h>
		)

	if("${CONFIG_ARMV7M_STACKCHECK}" STREQUAL "y")
		message(STATUS "NuttX Stack Checking (CONFIG_ARMV7M_STACKCHECK) enabled")
		add_compile_options(
			-ffixed-r10
			-finstrument-functions
			# instrumenting PX4 Matrix and Param methods is too burdensome
			-finstrument-functions-exclude-file-list=matrix/Matrix.hpp,px4_platform_common/param.h
		)
	endif()

endfunction()

#=============================================================================
#
#	px4_os_determine_build_chip
#
#	Sets PX4_CHIP and PX4_CHIP_MANUFACTURER.
#
#	Usage:
#		px4_os_determine_build_chip()
#
function(px4_os_determine_build_chip)

	# determine chip and chip manufacturer based on NuttX config
	if (CONFIG_STM32_STM32F10XX)
		set(CHIP_MANUFACTURER "stm")
		set(CHIP "stm32f1")
	elseif(CONFIG_STM32_STM32F30XX)
		set(CHIP_MANUFACTURER "stm")
		set(CHIP "stm32f3")
	elseif(CONFIG_STM32_STM32F4XXX)
		set(CHIP_MANUFACTURER "stm")
		set(CHIP "stm32f4")
	elseif(CONFIG_ARCH_CHIP_STM32F7)
		set(CHIP_MANUFACTURER "stm")
		set(CHIP "stm32f7")
	elseif(CONFIG_ARCH_CHIP_STM32H7)
		set(CHIP_MANUFACTURER "stm")
		set(CHIP "stm32h7")
	elseif(CONFIG_ARCH_CHIP_MK66FN2M0VMD18)
		set(CHIP_MANUFACTURER "nxp")
		set(CHIP "k66")
	elseif(CONFIG_ARCH_CHIP_MIMXRT1062DVL6A)
		set(CHIP_MANUFACTURER "nxp")
		set(CHIP "rt106x")
	elseif(CONFIG_ARCH_CHIP_S32K146)
		set(CHIP_MANUFACTURER "nxp")
		set(CHIP "s32k14x")
	else()
		message(FATAL_ERROR "Could not determine chip architecture from NuttX config. You may have to add it.")
	endif()

	set(PX4_CHIP ${CHIP} CACHE STRING "PX4 Chip" FORCE)
	set(PX4_CHIP_MANUFACTURER ${CHIP_MANUFACTURER} CACHE STRING "PX4 Chip Manufacturer" FORCE)
endfunction()

#=============================================================================
#
#	px4_os_prebuild_targets
#
#	This function generates os dependent targets
#
#	Usage:
#		px4_os_prebuild_targets(
#			OUT <out-list_of_targets>
#			BOARD <in-string>
#			)
#
#	Input:
#		BOARD		: board
#
#	Output:
#		OUT	: the target list
#
#	Example:
#		px4_os_prebuild_targets(OUT target_list BOARD px4_fmu-v2)
#
function(px4_os_prebuild_targets)
	px4_parse_function_args(
			NAME px4_os_prebuild_targets
			ONE_VALUE OUT BOARD
			REQUIRED OUT
			ARGN ${ARGN})

	if(EXISTS ${PX4_BOARD_DIR}/nuttx-config/${PX4_BOARD_LABEL})
		set(NUTTX_CONFIG "${PX4_BOARD_LABEL}" CACHE INTERNAL "NuttX config" FORCE)
	else()
		if(NOT NUTTX_CONFIG)
			set(NUTTX_CONFIG "nsh" CACHE INTERNAL "NuttX config" FORCE)
		else()
			set(NUTTX_CONFIG ${NUTTX_CONFIG} CACHE INTERNAL "NuttX config" FORCE)
		endif()
	endif()

	add_library(prebuild_targets INTERFACE)
	target_link_libraries(prebuild_targets INTERFACE nuttx_xx nuttx_c nuttx_fs nuttx_mm nuttx_sched m gcc)
	add_dependencies(prebuild_targets DEPENDS nuttx_build uorb_headers)

endfunction()
