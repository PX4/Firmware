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
# 	OS Specific Functions
#
#		* px4_posix_generate_builtin_commands
#
# 	Required OS Interface Functions
#
# 		* px4_os_add_flags
# 		* px4_os_determine_build_chip
#		* px4_os_prebuild_targets
#

#=============================================================================
#
#	px4_posix_generate_builtin_commands
#
#	This function generates the builtin_commands.c src for posix
#
#	Usage:
#		px4_posix_generate_builtin_commands(
#			MODULE_LIST <in-list>
#			OUT <file>)
#
#	Input:
#		MODULE_LIST	: list of modules
#
#	Output:
#		OUT	: stem of generated apps.cpp/apps.h ("apps")
#
#	Example:
#		px4_posix_generate_builtin_commands(
#			OUT <generated-src> MODULE_LIST px4_simple_app)
#
function(px4_posix_generate_builtin_commands)
	px4_parse_function_args(
		NAME px4_posix_generate_builtin_commands
		ONE_VALUE OUT
		MULTI_VALUE MODULE_LIST
		REQUIRED MODULE_LIST OUT
		ARGN ${ARGN})

	set(builtin_apps_string)
	set(builtin_apps_decl_string)
	set(command_count 0)
	foreach(module ${MODULE_LIST})
		# default
		set(MAIN_DEFAULT MAIN-NOTFOUND)
		set(STACK_DEFAULT 1024)
		set(PRIORITY_DEFAULT SCHED_PRIORITY_DEFAULT)
		foreach(property MAIN STACK PRIORITY)
			get_target_property(${property} ${module} ${property})
			if(NOT ${property})
				set(${property} ${${property}_DEFAULT})
			endif()
		endforeach()
		if (MAIN)
			set(builtin_apps_string
				"${builtin_apps_string}\tapps[\"${MAIN}\"] = ${MAIN}_main;\n")
			set(builtin_apps_decl_string
				"${builtin_apps_decl_string}int ${MAIN}_main(int argc, char *argv[]);\n")
			math(EXPR command_count "${command_count}+1")
		endif()
	endforeach()
	configure_file(${PX4_SOURCE_DIR}/platforms/common/apps.cpp.in ${OUT}.cpp)
	configure_file(${PX4_SOURCE_DIR}/platforms/common/apps.h.in ${OUT}.h)
endfunction()


#=============================================================================
#
#	px4_posix_generate_alias
#
#	This function generates the px4-alias.sh script containing the command
#	aliases for all modules and commands.
#
#	Usage:
#		px4_posix_generate_alias(
#			MODULE_LIST <in-list>
#			OUT <file>
#			PREFIX <prefix>)
#
#	Input:
#		MODULE_LIST	: list of modules
#		PREFIX	: command prefix (e.g. "px4-")
#
#	Output:
#		OUT	: px4-alias.sh file path
#
#	Example:
#		px4_posix_generate_alias(
#			OUT <generated-src> MODULE_LIST px4_simple_app PREFIX px4-)
#
function(px4_posix_generate_alias)
	px4_parse_function_args(
		NAME px4_posix_generate_alias
		ONE_VALUE OUT PREFIX
		MULTI_VALUE MODULE_LIST
		REQUIRED OUT PREFIX MODULE_LIST
		ARGN ${ARGN})

	set(alias_string)
	foreach(module ${MODULE_LIST})
		foreach(property MAIN STACK PRIORITY)
			get_target_property(${property} ${module} ${property})
			if(NOT ${property})
				set(${property} ${${property}_DEFAULT})
			endif()
		endforeach()
		if (MAIN)
			set(alias_string
				"${alias_string}alias ${MAIN}='${PREFIX}${MAIN} --instance $px4_instance'\n"
			)
		endif()
	endforeach()
	configure_file(${PX4_SOURCE_DIR}/platforms/posix/src/px4/common/px4-alias.sh_in ${OUT})
endfunction()


#=============================================================================
#
#	px4_posix_generate_symlinks
#
#	This function generates symlinks for all modules/commands.
#
#	Usage:
#		px4_posix_generate_symlinks(
#			TARGET <target>
#			MODULE_LIST <in-list>
#			PREFIX <prefix>)
#
#	Input:
#		MODULE_LIST	: list of modules
#		PREFIX	: command prefix (e.g. "px4-")
#		TARGET	: cmake target for which the symlinks should be created
#
#	Example:
#		px4_posix_generate_symlinks(
#			TARGET px4 MODULE_LIST px4_simple_app PREFIX px4-)
#
function(px4_posix_generate_symlinks)
	px4_parse_function_args(
		NAME px4_posix_generate_symlinks
		ONE_VALUE TARGET PREFIX
		MULTI_VALUE MODULE_LIST
		REQUIRED TARGET PREFIX MODULE_LIST
		ARGN ${ARGN})

	foreach(module ${MODULE_LIST})

		foreach(property MAIN STACK PRIORITY)
			get_target_property(${property} ${module} ${property})
			if(NOT ${property})
				set(${property} ${${property}_DEFAULT})
			endif()
		endforeach()

		if (MAIN)
			set(ln_name "${PREFIX}${MAIN}")
			add_custom_command(TARGET ${TARGET}
				POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E create_symlink ${TARGET} ${ln_name}
				WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
			)
		endif()
	endforeach()
endfunction()


#=============================================================================
#
#	px4_os_add_flags
#
#	Set the posix build flags.
#
#	Usage:
#		px4_os_add_flags()
#
function(px4_os_add_flags)

	add_definitions(
		-D__PX4_POSIX
		-Dnoreturn_function=__attribute__\(\(noreturn\)\)
		)

	include_directories(platforms/posix/include)

	if ("${PX4_BOARD}" MATCHES "sitl")

		if(UNIX AND APPLE)
			add_definitions(-D__PX4_DARWIN)

			if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
				message(FATAL_ERROR "PX4 Firmware requires XCode 8 or newer on Mac OS. Version installed on this system: ${CMAKE_CXX_COMPILER_VERSION}")
			endif()

			execute_process(COMMAND uname -v OUTPUT_VARIABLE DARWIN_VERSION)
			string(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})
			# message(STATUS "PX4 Darwin Version: ${DARWIN_VERSION}")
			if (DARWIN_VERSION LESS 16)
				add_definitions(
					-DCLOCK_MONOTONIC=1
					-DCLOCK_REALTIME=0
					-D__PX4_APPLE_LEGACY
					)
			endif()

		elseif(CYGWIN)
			add_definitions(
				-D__PX4_CYGWIN
				-D_GNU_SOURCE
				-D__USE_LINUX_IOCTL_DEFS
				-U__CUSTOM_FILE_IO__
				)
		else()
			add_definitions(-D__PX4_LINUX)
		endif()

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

	# always use generic chip and chip manufacturer
	set(PX4_CHIP "generic" CACHE STRING "PX4 Chip" FORCE)
	set(PX4_CHIP_MANUFACTURER "generic" CACHE STRING "PX4 Chip Manufacturer" FORCE)

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

	add_library(prebuild_targets INTERFACE)
	add_dependencies(prebuild_targets DEPENDS uorb_headers)

endfunction()
