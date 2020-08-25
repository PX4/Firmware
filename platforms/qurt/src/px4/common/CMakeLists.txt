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

list(APPEND CMAKE_MODULE_PATH "${PX4_SOURCE_DIR}/boards/atlflight/cmake_hexagon")
include(hexagon_sdk)

include_directories(${HEXAGON_8074_INCLUDES})

set(QURT_LAYER_SRCS
	px4_qurt_impl.cpp
	tasks.cpp
	lib_crc32.c
	drv_hrt.cpp
	qurt_stubs.c
	main.cpp
	shmem_qurt.cpp
	px4_init.cpp
	)

if ("${QURT_ENABLE_STUBS}" STREQUAL "1")
	list(APPEND QURT_LAYER_SRCS
		stubs/posix.c
		stubs/qurt.c
		)
endif()

add_library(px4_layer
	${QURT_LAYER_SRCS}
	${CONFIG_SRC}
)
target_link_libraries(px4_layer PRIVATE work_queue)
add_dependencies(px4_layer uorb_msgs) # dataman requires mission
