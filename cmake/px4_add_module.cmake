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
#	px4_add_module
#
#	This function builds a static library from a module description.
#
#	Usage:
#		px4_add_module(MODULE <string>
#			MAIN <string>
#			[ STACK_MAIN <string> ]
#			[ STACK_MAX <string> ]
#			[ COMPILE_FLAGS <list> ]
#			[ INCLUDES <list> ]
#			[ DEPENDS <string> ]
#			[ SRCS <list> ]
#			[ MODULE_CONFIG <list> ]
#			[ EXTERNAL ]
#			[ DYNAMIC ]
#			)
#
#	Input:
#		MODULE			: unique name of module
#		MAIN			: entry point
#		STACK			: deprecated use stack main instead
#		STACK_MAIN		: size of stack for main function
#		STACK_MAX		: maximum stack size of any frame
#		COMPILE_FLAGS		: compile flags
#		LINK_FLAGS		: link flags
#		SRCS			: source files
#		MODULE_CONFIG		: yaml config file(s)
#		INCLUDES		: include directories
#		DEPENDS			: targets which this module depends on
#		EXTERNAL		: flag to indicate that this module is out-of-tree
#		DYNAMIC			: don't compile into the px4 binary, but build a separate dynamically loadable module (posix)
#		UNITY_BUILD		: merge all source files and build this module as a single compilation unit
#
#	Output:
#		Static library with name matching MODULE.
#		(Or a shared library when DYNAMIC is specified.)
#
#	Example:
#		px4_add_module(MODULE test
#			SRCS
#				file.cpp
#			STACK_MAIN 1024
#			DEPENDS
#				git_nuttx
#			)
#
function(px4_add_module)

	px4_parse_function_args(
		NAME px4_add_module
		ONE_VALUE MODULE MAIN STACK_MAIN STACK_MAX PRIORITY
		MULTI_VALUE COMPILE_FLAGS LINK_FLAGS SRCS INCLUDES DEPENDS MODULE_CONFIG
		OPTIONS EXTERNAL DYNAMIC UNITY_BUILD
		REQUIRED MODULE MAIN
		ARGN ${ARGN})

	if(UNITY_BUILD AND (${PX4_PLATFORM} STREQUAL "nuttx"))
		# build standalone test library to catch compilation errors and provide sane output
		add_library(${MODULE}_original STATIC EXCLUDE_FROM_ALL ${SRCS})
		if(DEPENDS)
			add_dependencies(${MODULE}_original ${DEPENDS})
		endif()

		if(INCLUDES)
			target_include_directories(${MODULE}_original PRIVATE ${INCLUDES})
		endif()
		target_compile_definitions(${MODULE}_original PRIVATE PX4_MAIN=${MAIN}_app_main)
		target_compile_definitions(${MODULE}_original PRIVATE MODULE_NAME="${MAIN}_original")

		# unity build
		add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${MODULE}_unity.cpp
			COMMAND cat ${SRCS} > ${CMAKE_CURRENT_BINARY_DIR}/${MODULE}_unity.cpp
			DEPENDS ${MODULE}_original ${DEPENDS} ${SRCS}
			COMMENT "${MODULE} merging source"
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			)
		set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/${MODULE}_unity.cpp PROPERTIES GENERATED true)

		add_library(${MODULE} STATIC EXCLUDE_FROM_ALL ${CMAKE_CURRENT_BINARY_DIR}/${MODULE}_unity.cpp)
		target_include_directories(${MODULE} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

		if(COMPILE_FLAGS)
			target_compile_options(${MODULE}_original PRIVATE ${COMPILE_FLAGS})
		endif()

		if(DEPENDS)
			# using target_link_libraries for dependencies provides linking
			#  as well as interface include and libraries
			foreach(dep ${DEPENDS})
				get_target_property(dep_type ${dep} TYPE)
				if (${dep_type} STREQUAL "STATIC_LIBRARY")
					target_link_libraries(${MODULE}_original PRIVATE ${dep})
				else()
					add_dependencies(${MODULE}_original ${dep})
				endif()
			endforeach()
		endif()

	elseif(DYNAMIC AND MAIN AND (${PX4_PLATFORM} STREQUAL "posix"))
		add_library(${MODULE} SHARED ${SRCS})
		target_compile_definitions(${MODULE} PRIVATE ${MAIN}_main=px4_module_main)
		set_target_properties(${MODULE} PROPERTIES
			PREFIX ""
			SUFFIX ".px4mod"
			)
		target_link_libraries(${MODULE} PRIVATE px4)
		if(APPLE)
			# Postpone resolving symbols until loading time, which is the default on most systems, but not Mac.
			set_target_properties(${MODULE} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
		endif()

	else()
		add_library(${MODULE} STATIC EXCLUDE_FROM_ALL ${SRCS})
	endif()

	# all modules can potentially use parameters and uORB
	add_dependencies(${MODULE} uorb_headers)

	if(NOT DYNAMIC)
		target_link_libraries(${MODULE} PRIVATE prebuild_targets parameters_interface px4_layer px4_platform systemlib)
		set_property(GLOBAL APPEND PROPERTY PX4_MODULE_LIBRARIES ${MODULE})
		set_property(GLOBAL APPEND PROPERTY PX4_MODULE_PATHS ${CMAKE_CURRENT_SOURCE_DIR})
	endif()

	# set defaults if not set
	set(MAIN_DEFAULT MAIN-NOTFOUND)
	set(STACK_MAIN_DEFAULT 2048)
	set(PRIORITY_DEFAULT SCHED_PRIORITY_DEFAULT)

	foreach(property MAIN STACK_MAIN PRIORITY)
		if(NOT ${property})
			set(${property} ${${property}_DEFAULT})
		endif()
		set_target_properties(${MODULE} PROPERTIES ${property} ${${property}})
	endforeach()

	# default stack max to stack main
	if(NOT STACK_MAX)
		set(STACK_MAX ${STACK_MAIN})
	endif()
	set_target_properties(${MODULE} PROPERTIES STACK_MAX ${STACK_MAX})

	if(${PX4_PLATFORM} STREQUAL "qurt")
		set_property(TARGET ${MODULE} PROPERTY POSITION_INDEPENDENT_CODE TRUE)
	elseif(${PX4_PLATFORM} STREQUAL "nuttx")
		target_compile_options(${MODULE} PRIVATE -Wframe-larger-than=${STACK_MAX})
	endif()

	# MAIN
	if(MAIN)
		target_compile_definitions(${MODULE} PRIVATE PX4_MAIN=${MAIN}_app_main)
		target_compile_definitions(${MODULE} PRIVATE MODULE_NAME="${MAIN}")
	else()
		message(FATAL_ERROR "MAIN required")
	endif()

	if(COMPILE_FLAGS)
		target_compile_options(${MODULE} PRIVATE ${COMPILE_FLAGS})
	endif()

	if(INCLUDES)
		target_include_directories(${MODULE} PRIVATE ${INCLUDES})
	endif()

	if(DEPENDS)
		# using target_link_libraries for dependencies provides linking
		#  as well as interface include and libraries
		foreach(dep ${DEPENDS})
			get_target_property(dep_type ${dep} TYPE)
			if (${dep_type} STREQUAL "STATIC_LIBRARY")
				target_link_libraries(${MODULE} PRIVATE ${dep})
			else()
				add_dependencies(${MODULE} ${dep})
			endif()
		endforeach()
	endif()

	foreach (prop LINK_FLAGS STACK_MAIN MAIN PRIORITY)
		if (${prop})
			set_target_properties(${MODULE} PROPERTIES ${prop} ${${prop}})
		endif()
	endforeach()

	if(MODULE_CONFIG)
		foreach(module_config ${MODULE_CONFIG})
			set_property(GLOBAL APPEND PROPERTY PX4_MODULE_CONFIG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${module_config})
		endforeach()
	endif()
endfunction()
