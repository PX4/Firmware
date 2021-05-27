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

# skip for px4_layer support on an IO board
if(NOT PX4_BOARD MATCHES "io-v2")

	set(SRCS
		board_crashdump.c
		board_dma_alloc.c
		board_fat_dma_alloc.c
		console_buffer.cpp
		cpuload.cpp
		gpio.c
		print_load.cpp
		tasks.cpp
		px4_nuttx_impl.cpp
		px4_init.cpp
		px4_manifest.cpp
		px4_mtd.cpp
		px4_24xxxx_mtd.c
	)

	set(LIBS
		arch_board_reset
		arch_board_critmon
		arch_hrt
		arch_version
		nuttx_xx
		nuttx_sched
	)

if (NOT DEFINED CONFIG_BUILD_FLAT AND "${PX4_PLATFORM}" MATCHES "nuttx")

	list(APPEND LIBS
		nuttx_kc
		nuttx_karch
		nuttx_kmm
	)

	set(SRCS_USER
		tasks.cpp
		console_buffer_usr.cpp
		px4_nuttx_impl.cpp
		usr_mcu_version.cpp
		usr_hrt.cpp
		${PX4_SOURCE_DIR}/platforms/posix/src/px4/common/print_load.cpp
		${PX4_SOURCE_DIR}/platforms/posix/src/px4/common/cpuload.cpp
		usr_reset.cpp
	)

	set(LIBS_USER
		m
		nuttx_c
		nuttx_xx
		nuttx_mm
	)

	add_library(px4_layer
		${SRCS_USER}
	)

	target_link_libraries(px4_layer
		PRIVATE
			${LIBS_USER}
	)

	add_library(px4_kernel_layer
		${SRCS}
	)

	target_link_libraries(px4_kernel_layer
		PRIVATE
			${LIBS}
	)

	target_compile_options(px4_kernel_layer PRIVATE -D__KERNEL__)

	add_library(px4_board_ctrl
		board_ctrl.c
		kernel_modules.c
	)

	add_dependencies(px4_board_ctrl nuttx_context nuttx_kernel_builtin_list_target)
	add_dependencies(px4_kernel_layer prebuild_targets)
else()
	list(APPEND LIBS
		nuttx_c
		nuttx_arch
		nuttx_mm
	)

	add_library(px4_layer
		${SRCS}
	)

	target_link_libraries(px4_layer
		PRIVATE
			${LIBS}
	)
endif()

else()

	add_library(px4_layer ${PX4_SOURCE_DIR}/platforms/common/empty.c)
endif()

add_dependencies(px4_layer prebuild_targets)

add_subdirectory(gpio)
add_subdirectory(srgbled)
