############################################################################
#
#   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

# types:
# 0: COMP_METADATA_TYPE_GENERAL
# 1: COMP_METADATA_TYPE_PARAMETER
# 2: COMP_METADATA_TYPE_COMMANDS
# 3: COMP_METADATA_TYPE_PERIPHERALS
set(comp_metadata_types)
set(comp_metadata_board "${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_${PX4_BOARD_LABEL}")
set(s3_url "https://px4-travis.s3.amazonaws.com")
set(comp_metadata_param_uri_board "${s3_url}/Firmware/{version}/${comp_metadata_board}/parameters.json.xz")
list(FIND config_romfs_extra_dependencies "parameters_xml" index)
if (${index} EQUAL -1)
	set(comp_metadata_param_uri ${comp_metadata_param_uri_board})
	# use generic URL as fallback
	set(comp_metadata_param_uri_fallback "${s3_url}/Firmware/{version}/_general/parameters.json.xz")
else()
	set(comp_metadata_param_uri "mftp://etc/extras/parameters.json.xz")
	set(comp_metadata_param_uri_fallback ${comp_metadata_param_uri_board})
endif()
# arguments: type, metadata file, uri, fallback uri, optional translation uri
list(APPEND comp_metadata_types "--type" "1,${PX4_BINARY_DIR}/parameters.json.xz,${comp_metadata_param_uri},${comp_metadata_param_uri_fallback},")

set(component_general_json ${PX4_BINARY_DIR}/component_general.json)
add_custom_command(OUTPUT ${component_general_json} ${component_general_json}.xz
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_component_general.py
		${component_general_json}
		--compress
		${comp_metadata_types}
		--version-file ${PX4_BINARY_DIR}/src/lib/version/build_git_version.h
	DEPENDS
		generate_component_general.py
		${PX4_BINARY_DIR}/parameters.json.xz
		parameters_xml
		ver_gen
	COMMENT "Generating component_general.json"
)
add_custom_target(component_general_json DEPENDS ${component_general_json})


set(component_information_header ${CMAKE_CURRENT_BINARY_DIR}/checksums.h)
add_custom_command(OUTPUT ${component_information_header}
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_crc.py
		${component_general_json}
		--output ${component_information_header}
	DEPENDS
		generate_crc.py
		${component_general_json}
	COMMENT "Generating component_information/checksums.h"
)
add_custom_target(component_information_header DEPENDS ${component_information_header})


