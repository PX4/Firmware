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

if("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS_EQUAL 7)
		message(FATAL_ERROR "GCC 7 or older no longer supported. https://docs.px4.io/master/en/dev_setup/dev_env.html")
	endif()
endif()

if(NOT PX4_BOARD)
	message(FATAL_ERROR "PX4_BOARD must be set (eg px4_fmu-v2)")
endif()

if(NOT PX4_BINARY_DIR)
	message(FATAL_ERROR "PX4_BINARY_DIR must be set")
endif()

if(NOT PX4_BOARD_DIR)
	message(FATAL_ERROR "PX4_BOARD_DIR must be set")
endif()

set(NUTTX_CONFIG_DIR ${PX4_BOARD_DIR}/nuttx-config CACHE FILEPATH "PX4 NuttX config" FORCE)

# NuttX defconfig
#  cmake should trigger reconfigure if defconfig changes
set(NUTTX_DEFCONFIG ${NUTTX_CONFIG_DIR}/${NUTTX_CONFIG}/defconfig CACHE FILEPATH "path to defconfig" FORCE)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${NUTTX_DEFCONFIG})

set(NUTTX_SRC_DIR ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX)
set(NUTTX_DIR ${PX4_BINARY_DIR}/NuttX/nuttx CACHE FILEPATH "NuttX directory" FORCE)
set(NUTTX_APPS_DIR ${PX4_BINARY_DIR}/NuttX/apps CACHE FILEPATH "NuttX apps directory" FORCE)

px4_add_git_submodule(TARGET git_nuttx PATH "${NUTTX_SRC_DIR}/nuttx")
px4_add_git_submodule(TARGET git_nuttx_apps PATH "${NUTTX_SRC_DIR}/apps")

if(CMAKE_HOST_APPLE OR CMAKE_HOST_WIN32)
	# copy with rsync and create file dependencies
	set(NUTTX_COPY_CMD "rsync")
	set(NUTTX_COPY_CMD_OPTS)
	list(APPEND NUTTX_COPY_CMD_OPTS
		-rp
		--inplace
	)
else()
	# copy with hard links
	# archive, recursive, force, link (hardlinks)
	set(NUTTX_COPY_CMD "cp")
	set(NUTTX_COPY_CMD_OPTS "-aRfl")
endif()

execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/NuttX)

###############################################################################
# NuttX: copy to build directory
###############################################################################
if((NOT EXISTS ${PX4_BINARY_DIR}/NuttX/nuttx_copy.stamp) OR (NOT EXISTS ${PX4_BINARY_DIR}/NuttX/nuttx/Kconfig))
	execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/NuttX/nuttx)
	file(RELATIVE_PATH CP_SRC ${CMAKE_SOURCE_DIR} ${NUTTX_SRC_DIR}/nuttx)
	file(RELATIVE_PATH CP_DST ${CMAKE_SOURCE_DIR} ${PX4_BINARY_DIR}/NuttX)
	execute_process(COMMAND ${NUTTX_COPY_CMD} ${NUTTX_COPY_CMD_OPTS} ${CP_SRC} ${CP_DST} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

	# replace NuttX .git with actual git repo location (absolute path)
	execute_process(
		COMMAND git rev-parse --absolute-git-dir
		OUTPUT_VARIABLE nuttx_git_dir
		WORKING_DIRECTORY ${NUTTX_SRC_DIR}/nuttx
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	execute_process(COMMAND ${CMAKE_COMMAND} -E remove ${PX4_BINARY_DIR}/NuttX/nuttx/.git)
	file(WRITE ${PX4_BINARY_DIR}/NuttX/nuttx/.git "gitdir: ${nuttx_git_dir}")

	execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${PX4_BINARY_DIR}/NuttX/nuttx_copy.stamp)
endif()

###############################################################################
# NuttX apps: copy to build directory
###############################################################################
if((NOT EXISTS ${PX4_BINARY_DIR}/NuttX/apps_copy.stamp) OR (NOT EXISTS ${PX4_BINARY_DIR}/NuttX/apps/Kconfig))
	execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/NuttX/apps)
	file(RELATIVE_PATH CP_SRC ${CMAKE_SOURCE_DIR} ${NUTTX_SRC_DIR}/apps)
	file(RELATIVE_PATH CP_DST ${CMAKE_SOURCE_DIR} ${PX4_BINARY_DIR}/NuttX)
	execute_process(COMMAND ${NUTTX_COPY_CMD} ${NUTTX_COPY_CMD_OPTS} ${CP_SRC} ${CP_DST} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
	execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${PX4_BINARY_DIR}/NuttX/apps_copy.stamp)
endif()

###############################################################################
# board nuttx-config
###############################################################################

# If the board provides a Kconfig Use it or create an empty one
if((NOT EXISTS ${PX4_BINARY_DIR}/NuttX/nuttx_config_kconfig.stamp) OR (NOT EXISTS ${NUTTX_DIR}/boards/dummy/Kconfig))
	if(EXISTS ${NUTTX_CONFIG_DIR}/Kconfig)
		execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_CONFIG_DIR}/Kconfig ${NUTTX_DIR}/boards/dummy/Kconfig)
	else()
		execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${NUTTX_DIR}/boards/dummy/Kconfig)
	endif()

	execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${PX4_BINARY_DIR}/NuttX/nuttx_config_kconfig.stamp)
endif()

if((NOT EXISTS ${PX4_BINARY_DIR}/NuttX/nuttx_copy_config_dir.stamp) OR (NOT EXISTS ${PX4_BINARY_DIR}/NuttX/nuttx-config/drivers/Kconfig))
	# copy board's nuttx-config to NuttX/nuttx-config
	file(RELATIVE_PATH CP_SRC ${CMAKE_SOURCE_DIR} ${PX4_BOARD_DIR}/nuttx-config)
	file(RELATIVE_PATH CP_DST ${CMAKE_SOURCE_DIR} ${PX4_BINARY_DIR}/NuttX)
	execute_process(COMMAND ${NUTTX_COPY_CMD} ${NUTTX_COPY_CMD_OPTS} ${CP_SRC} ${CP_DST} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

	execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/NuttX/nuttx-config/drivers)
	execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${PX4_BINARY_DIR}/NuttX/nuttx-config/drivers/Kconfig)
	execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/NuttX/nuttx-config/src)
	execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SRC_DIR}/nsh_romfsimg.h ${PX4_BINARY_DIR}/NuttX/nuttx-config/include/nsh_romfsimg.h)

	execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${PX4_BINARY_DIR}/NuttX/nuttx_copy_config_dir.stamp)
endif()

# make olddefconfig (inflate defconfig to full .config)
if((NOT EXISTS ${PX4_BINARY_DIR}/NuttX/nuttx_olddefconfig.stamp) OR (NOT EXISTS ${PX4_BINARY_DIR}/NuttX/nuttx/.config))
	execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_SRC_DIR}/Make.defs.in ${NUTTX_DIR}/Make.defs) # Create a temporary Toplevel Make.defs for the oldconfig step
	execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config)
	execute_process(
		COMMAND ${NUTTX_SRC_DIR}/tools/px4_nuttx_make_olddefconfig.sh
		WORKING_DIRECTORY ${NUTTX_DIR}
		OUTPUT_FILE nuttx_olddefconfig.log
		RESULT_VARIABLE ret
	)
	execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${PX4_BINARY_DIR}/NuttX/nuttx_olddefconfig.stamp)
	# remove temporary top level Make.defs
	execute_process(COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/Make.defs)
endif()

###############################################################################
# NuttX cmake defconfig
###############################################################################

# parse nuttx config options for cmake
file(STRINGS ${NUTTX_DIR}/.config ConfigContents)
foreach(NameAndValue ${ConfigContents})
	# Strip leading spaces
	string(REGEX REPLACE "^[ ]+" "" NameAndValue ${NameAndValue})

	# Find variable name
	string(REGEX MATCH "^CONFIG[^=]+" Name ${NameAndValue})

	if(Name)
		# Find the value
		string(REPLACE "${Name}=" "" Value ${NameAndValue})

		if(Value)
			# remove extra quotes
			string(REPLACE "\"" "" Value ${Value})

			# Set the variable
			#message(STATUS "${Name} ${Value}")
			set(${Name} ${Value} CACHE INTERNAL "NUTTX DEFCONFIG: ${Name}" FORCE)
		endif()
	endif()
endforeach()
