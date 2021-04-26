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
	MODULE drivers__px4io
	MAIN px4io
	COMPILE_FLAGS
		-Wno-cast-align # TODO: fix and enable
	SRCS
		px4io.cpp
		px4io_serial.cpp
		px4io_uploader.cpp
	DEPENDS
		arch_px4io_serial
		circuit_breaker
		mixer
	)

# include the px4io binary in ROMFS
message(STATUS "Building and including ${config_io_board}")

include(ExternalProject)
ExternalProject_Add(px4io_firmware
	SOURCE_DIR ${CMAKE_SOURCE_DIR}
	DOWNLOAD_COMMAND ""
	UPDATE_COMMAND ""
	CMAKE_ARGS -DCONFIG=${config_io_board}
	INSTALL_COMMAND ""
	USES_TERMINAL_BUILD true
	DEPENDS git_nuttx git_nuttx_apps
	BUILD_ALWAYS 1
	BUILD_BYPRODUCTS "${PX4_BINARY_DIR}/external/Build/px4io_firmware/${config_io_board}.elf"
)

ExternalProject_Get_Property(px4io_firmware BINARY_DIR)

set(fw_io_exe "${BINARY_DIR}/${config_io_board}.elf")
set(fw_io_bin "${PX4_BINARY_DIR}/romfs_extras/${config_io_board}.bin" CACHE FILEPATH "px4io binary path")

file(RELATIVE_PATH fw_io_exe_relative ${CMAKE_CURRENT_BINARY_DIR} ${fw_io_exe})
file(RELATIVE_PATH fw_io_bin_relative ${CMAKE_CURRENT_BINARY_DIR} ${fw_io_bin})

add_custom_command(OUTPUT ${fw_io_bin}
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/romfs_extras
	COMMAND ${CMAKE_OBJCOPY} -O binary ${fw_io_exe_relative} ${fw_io_bin_relative}
	DEPENDS ${fw_io_exe} px4io_firmware
	COMMENT "Copying ${config_io_board} to ROMFS extras"
	)
add_custom_target(copy_px4io_bin DEPENDS ${fw_io_bin})
