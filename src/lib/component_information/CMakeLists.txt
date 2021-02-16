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

set(component_version_extra_args)
# We disable parameter support if the metadata is not in the ROMFS. This can be
# removed once it is stored on a server
list(FIND config_romfs_extra_dependencies "parameters_xml" index)
if (${index} EQUAL -1)
	set(component_version_extra_args "--no-parameters")
endif()

set(component_version_json ${PX4_BINARY_DIR}/component_version.json)
add_custom_command(OUTPUT ${component_version_json} ${component_version_json}.xz
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_component_version.py
		${component_version_json}
		--compress
		${component_version_extra_args}
	DEPENDS
		generate_component_version.py
	COMMENT "Generating component_version.json"
)
add_custom_target(component_version_json DEPENDS ${component_version_json})


set(component_information_header ${CMAKE_CURRENT_BINARY_DIR}/hashes.h)
add_custom_command(OUTPUT ${component_information_header}
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_hashes.py
		${PX4_BINARY_DIR}/params.json.xz
		${component_version_json}
		--output ${component_information_header}
	DEPENDS
		generate_hashes.py
		parameters_xml
		${PX4_BINARY_DIR}/params.json.xz
		${component_version_json}
	COMMENT "Generating component_information/hashes.h"
)
add_custom_target(component_information_header DEPENDS ${component_information_header})


