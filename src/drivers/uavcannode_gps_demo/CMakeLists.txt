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

set(LIBCANARD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libcanard)
set(DSDL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/public_regulated_data_types)

px4_add_git_submodule(TARGET git_libcanard PATH ${LIBCANARD_DIR})
px4_add_git_submodule(TARGET git_public_regulated_data_types PATH ${DSDL_DIR})

find_program(NNVG_PATH nnvg)
if(NNVG_PATH)
	execute_process(COMMAND ${NNVG_PATH} --outdir ${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated --target-language c -I ${DSDL_DIR}/uavcan ${DSDL_DIR}/reg)
	execute_process(COMMAND ${NNVG_PATH} --outdir ${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated --target-language c ${DSDL_DIR}/uavcan)

else()
	message(FATAL_ERROR "UAVCAN Nunavut nnvg not found")
endif()

add_definitions(
	-DCONFIG_EXAMPLES_LIBCANARDV1_DEV="can0"
	-DCONFIG_EXAMPLES_LIBCANARDV1_NODE_MEM_POOL_SIZE=8192
	-DCONFIG_EXAMPLES_LIBCANARDV1_DAEMON_STACK_SIZE=5000
	-DCONFIG_EXAMPLES_LIBCANARDV1_DAEMON_PRIORITY=100
	-DCANARD_DSDL_CONFIG_LITTLE_ENDIAN=1
)

px4_add_module(
	MODULE drivers__uavcannode-gps-demo
	MAIN uavcannode_gps_demo
	COMPILE_FLAGS
		-Wno-error
	    -DUINT32_C\(x\)=__UINT32_C\(x\)
	INCLUDES
		${LIBCANARD_DIR}/libcanard/
		${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated
	SRCS
		canard_main.c
		socketcan.c
		socketcan.h
		uorb_converter.c
		uorb_converter.h
		o1heap.c
		o1heap.h
		libcancl/pnp.c
		libcancl/registerinterface.c
		${LIBCANARD_DIR}/libcanard/canard.c
		${LIBCANARD_DIR}/libcanard/canard.h
	DEPENDS
		git_libcanard
		git_public_regulated_data_types
		drivers_bootloaders
		version
	)
