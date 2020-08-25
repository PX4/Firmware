############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

# find PX4 config
#  look for in tree board config that matches CONFIG input
if(NOT PX4_CONFIG_FILE)

	file(GLOB_RECURSE board_configs
		RELATIVE "${PX4_SOURCE_DIR}/boards"
		"boards/*.cmake"
		)

	set(PX4_CONFIGS ${board_configs} CACHE STRING "PX4 board configs" FORCE)

	foreach(filename ${board_configs})
		# parse input CONFIG into components to match with existing in tree configs
		#  the platform prefix (eg nuttx_) is historical, and removed if present
		string(REPLACE ".cmake" "" filename_stripped ${filename})
		string(REPLACE "/" ";" config ${filename_stripped})
		list(LENGTH config config_len)

		if(${config_len} EQUAL 3)
			list(GET config 0 vendor)
			list(GET config 1 model)
			list(GET config 2 label)

			set(board "${vendor}${model}")

			# <VENDOR>_<MODEL>_<LABEL> (eg px4_fmu-v2_default)
			# <VENDOR>_<MODEL>_default (eg px4_fmu-v2) # allow skipping label if "default"
			if ((${CONFIG} MATCHES "${vendor}_${model}_${label}") OR # match full vendor, model, label
			    ((${label} STREQUAL "default") AND (${CONFIG} STREQUAL "${vendor}_${model}")) # default label can be omitted
			)
				set(PX4_CONFIG_FILE "${PX4_SOURCE_DIR}/boards/${filename}" CACHE FILEPATH "path to PX4 CONFIG file" FORCE)
				break()
			endif()

			# <BOARD>_<LABEL> (eg px4_fmu-v2_default)
			# <BOARD>_default (eg px4_fmu-v2) # allow skipping label if "default"
			if ((${CONFIG} MATCHES "${board}_${label}") OR # match full board, label
			    ((${label} STREQUAL "default") AND (${CONFIG} STREQUAL "${board}")) # default label can be omitted
			)
				set(PX4_CONFIG_FILE "${PX4_SOURCE_DIR}/boards/${filename}" CACHE FILEPATH "path to PX4 CONFIG file" FORCE)
				break()
			endif()
		endif()
	endforeach()
endif()

if(NOT PX4_CONFIG_FILE)
	message(FATAL_ERROR "PX4 config file not set, try one of ${PX4_CONFIGS}")
endif()

message(STATUS "PX4 config file: ${PX4_CONFIG_FILE}")
