############################################################################
#
#   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

set(CMSIS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5)
set(CMSIS_DSP ${CMSIS_ROOT}/CMSIS/DSP)

if(${PX4_PLATFORM} MATCHES "NuttX")
	add_compile_options(-DARM_MATH_DSP)
endif()

add_compile_options($<$<COMPILE_LANGUAGE:C>:-Wno-nested-externs>)

px4_add_module(
	MODULE modules__gyro_fft
	MAIN gyro_fft
	STACK_MAIN
		4096
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
		-DARM_ALL_FFT_TABLES
		-DARM_MATH_LOOPUNROLL
	INCLUDES
		${CMSIS_ROOT}/CMSIS/Core/Include
		${CMSIS_DSP}/Include
	SRCS
		GyroFFT.cpp
		GyroFFT.hpp

		${CMSIS_ROOT}/CMSIS/Core/Include/cmsis_compiler.h
		${CMSIS_ROOT}/CMSIS/Core/Include/cmsis_gcc.h
		${CMSIS_DSP}/Include/arm_common_tables.h
		${CMSIS_DSP}/Include/arm_const_structs.h
		${CMSIS_DSP}/Include/arm_math.h
		${CMSIS_DSP}/Source/BasicMathFunctions/arm_mult_q15.c
		${CMSIS_DSP}/Source/CommonTables/arm_common_tables.c
		${CMSIS_DSP}/Source/CommonTables/arm_const_structs.c
		${CMSIS_DSP}/Source/SupportFunctions/arm_float_to_q15.c
		${CMSIS_DSP}/Source/TransformFunctions/arm_bitreversal2.c
		${CMSIS_DSP}/Source/TransformFunctions/arm_cfft_q15.c
		${CMSIS_DSP}/Source/TransformFunctions/arm_cfft_radix4_q15.c
		${CMSIS_DSP}/Source/TransformFunctions/arm_rfft_init_q15.c
		${CMSIS_DSP}/Source/TransformFunctions/arm_rfft_q15.c
	DEPENDS
		px4_work_queue
)
