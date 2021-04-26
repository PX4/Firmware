############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
px4_add_module(
	MODULE systemcmds__bl_update
	MAIN bl_update
	STACK_MAIN 4096
	COMPILE_FLAGS
		-Wno-cast-align # TODO: fix and enable
	SRCS
		bl_update.c
)

if(config_build_bootloader)
	# include the bootloader binary in ROMFS
	set(bootloader_config "${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_bootloader")

	if(NOT EXISTS ${PX4_BOARD_DIR}/nuttx-config/bootloader)
		message(FATAL_ERROR "${bootloader_config} does not exist")
	endif()

	message(STATUS "Building and including bootloader ${bootloader_config}")

	include(ExternalProject)
	ExternalProject_Add(bootloader_firmware
		SOURCE_DIR ${CMAKE_SOURCE_DIR}
		DOWNLOAD_COMMAND ""
		UPDATE_COMMAND ""
		CMAKE_ARGS -DCONFIG=${bootloader_config}
		INSTALL_COMMAND ""
		USES_TERMINAL_BUILD true
		DEPENDS git_nuttx git_nuttx_apps
		BUILD_ALWAYS 1
	)

	ExternalProject_Get_Property(bootloader_firmware BINARY_DIR)

	set(board_bootloader_firmware "${BINARY_DIR}/${bootloader_config}.elf" CACHE FILEPATH "bootloader original binary path")
	set(bootloader_bin "${PX4_BINARY_DIR}/romfs_extras/bootloader.bin" CACHE FILEPATH "bootloader binary path")

	file(RELATIVE_PATH bootloader_exe_relative ${CMAKE_CURRENT_BINARY_DIR} ${board_bootloader_firmware})
	file(RELATIVE_PATH bootloader_bin_relative ${CMAKE_CURRENT_BINARY_DIR} ${bootloader_bin})

	add_custom_command(OUTPUT ${bootloader_bin}
		COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/romfs_extras
		COMMAND ${CMAKE_OBJCOPY} -O binary ${bootloader_exe_relative} ${bootloader_bin_relative}
		DEPENDS bootloader_firmware
		COMMENT "Copying ${bootloader_config} to ROMFS extras"
		)
	add_custom_target(copy_bootloader_bin DEPENDS ${bootloader_bin})

elseif(EXISTS "${PX4_BOARD_DIR}/bootloader/")

	if(EXISTS ${PX4_BOARD_DIR}/bootloader/${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_bootloader.bin)
		set(board_bootloader_firmware ${PX4_BOARD_DIR}/bootloader/${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_bootloader.bin CACHE FILEPATH "bootloader original binary path")
		set(bootloader_bin "${PX4_BINARY_DIR}/romfs_extras/bootloader.bin" CACHE FILEPATH "bootloader binary path")

		add_custom_command(OUTPUT ${bootloader_bin}
			COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/romfs_extras
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${board_bootloader_firmware} ${PX4_BINARY_DIR}/romfs_extras/bootloader.bin
			DEPENDS ${board_bootloader_firmware}
			COMMENT "Copying ${board_bootloader_firmware} to ROMFS extras"
		)
		add_custom_target(copy_bootloader_bin DEPENDS ${bootloader_bin})

	else()
		file(GLOB bootloader_bin_files ${PX4_BOARD_DIR}/bootloader/*.bin)
		foreach(bin_file ${bootloader_bin_files})
			message(FATAL_ERROR "incorrect bootloader filename ${bin_file} should be ${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_bootloader.bin")
		endforeach()
	endif()
endif()
