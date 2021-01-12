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


px4_add_git_submodule(TARGET git_cmsis_v5 PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5")

# Needed to include the configBoot module
# Define the path to CMSIS-DSP (ROOT is defined on command line when using cmake)
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5)
set(DSP ${ROOT}/CMSIS/DSP)

set(GCC ON CACHE BOOL "")
set(PLATFORM "FVP" CACHE STRING "")
set(ARM_CPU ${ARCHITECTURE} CACHE STRING "Set ARM CPU. Default : cortex-m4")

set(FASTMATH ON CACHE BOOL "")
set(TRANSFORM ON CACHE BOOL "")
#set(CONFIGTABLE ON CACHE BOOL "")
#set(ALLFAST ON CACHE BOOL "")
#set(ALLFFT ON CACHE BOOL "")

add_compile_options(
	#-Wno-strict-prototypes
	-Wno-error

	-DARM_FFT_ALLOW_TABLES

	-DARM_ALL_FFT_TABLES
	-DARM_TABLE_TWIDDLECOEF_Q15_128
	-DARM_TABLE_BITREVIDX_FXT_128
	-DARM_TABLE_REALCOEF_Q15

	-DARMCM4_FP
	-DARM_MATH_LOOPUNROLL
	-DCORTEXM
)

add_compile_options($<$<COMPILE_LANGUAGE:C>:-Wno-nested-externs>)

px4_add_module(
	MODULE modules__gyro_fft
	MAIN gyro_fft
	STACK_MAIN
		4096
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	INCLUDES
		${ROOT}/CMSIS/Core/Include
		${DSP}/Include
	SRCS
		GyroFFT.cpp
		GyroFFT.hpp

		${DSP}/Source/BasicMathFunctions/arm_shift_q15.c

		${DSP}/Source/CommonTables/arm_common_tables.c
		${DSP}/Source/CommonTables/arm_const_structs.c

		${DSP}/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.c

		${DSP}/Source/TransformFunctions/arm_bitreversal2.c

		#${DSP}/Source/TransformFunctions/arm_cfft_init_q15.c
		${DSP}/Source/TransformFunctions/arm_cfft_q15.c
		${DSP}/Source/TransformFunctions/arm_cfft_radix4_q15.c

		${DSP}/Source/TransformFunctions/arm_rfft_init_q15.c
		${DSP}/Source/TransformFunctions/arm_rfft_q15.c
		#${DSP}/Source/TransformFunctions/arm_rfft_radix4_q15.c

		${DSP}/Source/SupportFunctions/arm_float_to_q15.c
		${DSP}/Source/BasicMathFunctions/arm_mult_q15.c

	DEPENDS
		git_cmsis_v5
		px4_work_queue
)
