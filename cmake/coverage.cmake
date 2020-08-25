############################################################################
#
# Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

find_program(LCOV_PATH lcov)
find_program(GENHTML_PATH genhtml)

message(STATUS "Building for code coverage")

# add code coverage build type
if (("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang") OR ("${CMAKE_CXX_COMPILER_ID}" MATCHES "AppleClang"))
	set(CMAKE_C_FLAGS_COVERAGE "--coverage -ftest-coverage -fdiagnostics-absolute-paths -O0 -fprofile-arcs -fno-inline-functions"
		CACHE STRING "Flags used by the C compiler during coverage builds" FORCE)

	set(CMAKE_CXX_FLAGS_COVERAGE "--coverage -ftest-coverage -fdiagnostics-absolute-paths -O0-fprofile-arcs -fno-inline-functions -fno-elide-constructors"
		CACHE STRING "Flags used by the C++ compiler during coverage builds" FORCE)

	set(CMAKE_EXE_LINKER_FLAGS_COVERAGE "-ftest-coverage -fdiagnostics-absolute-paths"
        CACHE STRING "Flags used for linking binaries during coverage builds" FORCE)

else()
	# Add  -fprofile-abs-path for GCC v8/9 later on
	set(CMAKE_C_FLAGS_COVERAGE "--coverage -ftest-coverage -fprofile-arcs -O0 -fno-default-inline -fno-inline"
		CACHE STRING "Flags used by the C compiler during coverage builds" FORCE)

	# Add  -fprofile-abs-path for GCC v8/9 later on
	set(CMAKE_CXX_FLAGS_COVERAGE "--coverage -ftest-coverage -fprofile-arcs -O0 -fno-default-inline -fno-inline -fno-elide-constructors"
		CACHE STRING "Flags used by the C++ compiler during coverage builds" FORCE)

	set(CMAKE_EXE_LINKER_FLAGS_COVERAGE "--coverage -ftest-coverage -lgcov"
        CACHE STRING "Flags used for linking binaries during coverage builds" FORCE)
endif()

mark_as_advanced(CMAKE_CXX_FLAGS_COVERAGE CMAKE_C_FLAGS_COVERAGE CMAKE_EXE_LINKER_FLAGS_COVERAGE)

# Param _targetname     The name of new the custom make target
# Param _testrunner     The name of the target which runs the tests.
#
# Param _outputname     lcov output is generated as _outputname.info
#                       HTML report is generated in _outputname/index.html
# Optional fourth parameter is passed as arguments to _testrunner
#   Pass them in list form, e.g.: "-j;2" for -j 2
FUNCTION(SETUP_TARGET_FOR_COVERAGE _targetname _testrunner _outputname)

	set(options NONE)
	set(oneValueArgs NAME)
	set(multiValueArgs EXECUTABLE EXECUTABLE_ARGS DEPENDENCIES)
	cmake_parse_arguments(Coverage "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

	if (NOT LCOV_PATH)
		message(FATAL_ERROR "lcov required")
	endif()

	if (NOT GENHTML_PATH)
		message(FATAL_ERROR "genhtml required")
	endif()

	set(coverage_info "${CMAKE_BINARY_DIR}/${_outputname}.info")
	set(coverage_cleaned "${coverage_info}.cleaned")

	separate_arguments(test_command UNIX_COMMAND "${_testrunner}")

	# Setup target
	add_custom_command(OUTPUT ${coverage_info}
		# Cleanup lcov
		#COMMAND ${LCOV_PATH} --quiet --directory . --zerocounters

		# Run tests
		COMMAND ${test_command} ${ARGV3}

		# Capturing lcov counters and generating report
		COMMAND ${LCOV_PATH} --quiet --base-directory ${CMAKE_BINARY_DIR} --directory ${CMAKE_SOURCE_DIR} --capture --output-file ${coverage_info}

		DEPENDS px4
		USES_TERMINAL
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
		COMMENT "Running code coverage and generating report."
	)
	add_custom_target(${_targetname} DEPENDS ${coverage_info})

	add_custom_command(OUTPUT ${coverage_cleaned}
		COMMAND ${LCOV_PATH} --quiet
			--remove ${coverage_info} 'tests/*' '/usr/*' 'src/examples*'
			--output-file ${coverage_cleaned}
		DEPENDS ${coverage_info}
	)

	add_custom_target(${_targetname}_genhtml
		COMMAND ${GENHTML_PATH} --quiet -o coverage-html ${coverage_cleaned}
		DEPENDS ${coverage_cleaned}
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	)

endfunction() # SETUP_TARGET_FOR_COVERAGE
