############################################################################
#
#   Copyright (c) 2017 - 2020 PX4 Development Team. All rights reserved.
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

add_compile_options(${MAX_CUSTOM_OPT_LEVEL})

add_subdirectory(tinybson)

if (NOT PARAM_DEFAULT_OVERRIDES)
	set(PARAM_DEFAULT_OVERRIDES "{}")
endif()

# get full path for each module
get_property(module_list GLOBAL PROPERTY PX4_MODULE_PATHS)
get_property(module_config_files GLOBAL PROPERTY PX4_MODULE_CONFIG_FILES)

if(DISABLE_PARAMS_MODULE_SCOPING)
	# search all directories with .c files (potentially containing parameters)
	file(GLOB_RECURSE c_files
		${PX4_SOURCE_DIR}/src/*.c
		${external_module_paths}
		)
	foreach(file_path ${c_files})
		get_filename_component(dir_path ${file_path} PATH)
		list(APPEND module_list "${dir_path}")
	endforeach()

	# search for all module configs as well
	file(GLOB_RECURSE yaml_files
		${PX4_SOURCE_DIR}/src/*.yaml
		)
	foreach(file_path ${yaml_files})
		list(APPEND module_config_files "${file_path}")
	endforeach()

	list(REMOVE_DUPLICATES module_config_files)
else()
	list(APPEND module_list ${external_module_paths})
endif()

list(REMOVE_DUPLICATES module_list)

set(generated_params_dir ${PX4_BINARY_DIR}/generated_params)
set(generated_serial_params_file ${generated_params_dir}/serial_params.c)
file(GLOB jinja_templates ${PX4_SOURCE_DIR}/Tools/serial/*.jinja)
if (px4_constrained_flash_build)
	set(added_arguments --constrained-flash)
endif()
if(PX4_ETHERNET)
	set(added_arguments ${added_arguments} --ethernet)
endif()
add_custom_command(OUTPUT ${generated_serial_params_file}
	COMMAND ${CMAKE_COMMAND} -E make_directory ${generated_params_dir}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/serial/generate_config.py
		--params-file ${generated_serial_params_file}
		--serial-ports ${board_serial_ports} ${added_arguments}
		--config-files ${module_config_files} #--verbose
	DEPENDS
		${module_config_files}
		${jinja_templates}
		${PX4_SOURCE_DIR}/Tools/serial/generate_config.py
	COMMENT "Generating serial_params.c"
)

set(parameters_xml ${PX4_BINARY_DIR}/parameters.xml)
set(parameters_json ${PX4_BINARY_DIR}/parameters.json)
file(GLOB_RECURSE param_src_files ${PX4_SOURCE_DIR}/src/*params.c ${PX4_SOURCE_DIR}/src/*parameters.c)
add_custom_command(OUTPUT ${parameters_xml} ${parameters_json} ${parameters_json}.xz
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/px_process_params.py
		--src-path ${module_list} ${generated_params_dir}
		--xml ${parameters_xml}
		--json ${parameters_json}
		--compress
		--inject-xml ${CMAKE_CURRENT_SOURCE_DIR}/parameters_injected.xml
		--overrides ${PARAM_DEFAULT_OVERRIDES}
		--board ${PX4_BOARD}
		#--verbose
	DEPENDS
		${param_src_files}
		${generated_serial_params_file}
		parameters_injected.xml
		px4params/srcparser.py
		px4params/srcscanner.py
		px4params/jsonout.py
		px4params/xmlout.py
		px_process_params.py
		parameters_injected.xml
	COMMENT "Generating parameters.xml"
)
add_custom_target(parameters_xml DEPENDS ${parameters_xml})

# generate px4_parameters.hpp
add_custom_command(OUTPUT px4_parameters.hpp
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/px_generate_params.py
		--xml ${parameters_xml} --dest ${CMAKE_CURRENT_BINARY_DIR}
	DEPENDS
		${PX4_BINARY_DIR}/parameters.xml
		px_generate_params.py
		templates/px4_parameters.hpp.jinja
	)

set(SRCS)
if ("${CONFIG_SHMEM}" STREQUAL "1")
	message(STATUS "parameters shared memory enabled")
	add_definitions(-DCONFIG_SHMEM=1)
	list(APPEND SRCS parameters_shmem.cpp)
else()
	list(APPEND SRCS parameters.cpp)
	if(BUILD_TESTING)
		list(APPEND SRCS param_translation_unit_tests.cpp)
	else()
		list(APPEND SRCS param_translation.cpp)
	endif()
endif()

if(${PX4_PLATFORM} STREQUAL "nuttx")
	add_subdirectory(flashparams)
endif()

# TODO: find a better way to do this
if (NOT "${PX4_BOARD}" MATCHES "px4_io")
	add_library(parameters
		${SRCS}
		px4_parameters.hpp
	)

	target_link_libraries(parameters PRIVATE perf tinybson px4_layer)
	target_compile_definitions(parameters PRIVATE -DMODULE_NAME="parameters")
	target_compile_options(parameters
		PRIVATE
			-Wno-cast-align		# TODO: fix and enable
			-Wno-sign-compare	# TODO: fix and enable
	)
else()
	add_library(parameters ${PX4_SOURCE_DIR}/platforms/common/empty.c)
endif()
add_dependencies(parameters prebuild_targets)

if(${PX4_PLATFORM} STREQUAL "nuttx")
	target_link_libraries(parameters PRIVATE flashparams tinybson)
endif()

px4_add_functional_gtest(SRC ParameterTest.cpp LINKLIBS parameters)
