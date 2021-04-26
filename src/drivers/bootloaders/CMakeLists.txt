############################################################################
#
#   Copyright (c) 2017, 2021 PX4 Development Team. All rights reserved.
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

add_library(drivers_bootloaders
	boot_app_shared.c
	boot_alt_app_shared.c
	)
add_dependencies(drivers_bootloaders prebuild_targets)

include_directories(include)
target_include_directories(drivers_bootloaders INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(drivers_bootloaders PRIVATE systemlib)

# generate bootloader_app_shared_t
if(NOT "${PX4_BOARD_LABEL}" MATCHES  "canbootloader")
	set(HW_MAJOR ${uavcanblid_hw_version_major})
	set(HW_MINOR ${uavcanblid_hw_version_minor})
	set(SW_MAJOR ${uavcanblid_sw_version_major})
	set(SW_MINOR ${uavcanblid_sw_version_minor})

	math(EXPR HWBOARD_ID "(${HW_MAJOR} << 8) + ${HW_MINOR}")

	execute_process(
		COMMAND git rev-list HEAD --max-count=1 --abbrev=8 --abbrev-commit
		OUTPUT_VARIABLE uavcanbl_git_desc
		OUTPUT_STRIP_TRAILING_WHITESPACE
		WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	)

	if("${uavcanbl_git_desc}" STREQUAL "")
		set(uavcanbl_git_desc ffffffff)
	endif()
	set(uavcan_bl_image_name ${HWBOARD_ID}-${SW_MAJOR}.${SW_MINOR}.${uavcanbl_git_desc}.uavcan.bin)
	set(uavcan_bl_image_name "${uavcan_bl_image_name}" CACHE STRING "UAVCAN BL Image name" FORCE)
	message(STATUS "Generating UAVCAN Bootable as ${uavcan_bl_image_name}")
	add_custom_command(
		OUTPUT
			${PX4_BINARY_DIR}/${uavcan_bl_image_name}
			${PX4_BINARY_DIR}/deploy/${HWBOARD_ID}.bin
		COMMAND
			${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/make_can_boot_descriptor.py -v --use-git-hash ${PX4_BOARD}.bin ${uavcan_bl_image_name}
		COMMAND
			 COMMAND ${CMAKE_COMMAND} -E make_directory deploy
		COMMAND
			${CMAKE_COMMAND} -E copy ${uavcan_bl_image_name} deploy/${HWBOARD_ID}.bin
		DEPENDS
			${PX4_BINARY_DIR}/${PX4_BOARD}.bin
			${CMAKE_CURRENT_SOURCE_DIR}/make_can_boot_descriptor.py
		WORKING_DIRECTORY
			${PX4_BINARY_DIR}
	)
	add_custom_target(build_uavcan_bl ALL DEPENDS ${PX4_BINARY_DIR}/${uavcan_bl_image_name})

endif()
