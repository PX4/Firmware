############################################################################
#
# Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

if (CMAKE_BUILD_TYPE STREQUAL AddressSanitizer)
	message(STATUS "AddressSanitizer enabled")

	# environment variables
	#  ASAN_OPTIONS=check_initialization_order=1,detect_stack_use_after_return=1
	add_compile_options(
		-O1
		-g3

		-fsanitize=address

		-fno-omit-frame-pointer # Leave frame pointers. Allows the fast unwinder to function properly.
		-fno-common # Do not treat global variable in C as common variables (allows ASan to instrument them)
		-fno-optimize-sibling-calls # disable inlining and and tail call elimination for perfect stack traces
	)

	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address" CACHE INTERNAL "" FORCE)
	set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=address" CACHE INTERNAL "" FORCE)
	set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fsanitize=address" CACHE INTERNAL "" FORCE)

	function(sanitizer_fail_test_on_error test_name)
		set_tests_properties(${test_name} PROPERTIES FAIL_REGULAR_EXPRESSION "ERROR: AddressSanitizer")
		set_tests_properties(${test_name} PROPERTIES FAIL_REGULAR_EXPRESSION "ERROR: LeakSanitizer")
	endfunction(sanitizer_fail_test_on_error)

elseif (CMAKE_BUILD_TYPE STREQUAL MemorySanitizer)
	if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
		message(WARNING "MemorySanitizer might not be available with gcc")
	else()
		message(STATUS "MemorySanitizer enabled")
	endif()

	add_compile_options(
		-O1
		-g3

		-fsanitize=memory
		-fsanitize-memory-track-origins

		-fno-omit-frame-pointer # Leave frame pointers. Allows the fast unwinder to function properly.
		-fno-common # Do not treat global variable in C as common variables (allows ASan to instrument them)
		-fno-optimize-sibling-calls # disable inlining and and tail call elimination for perfect stack traces
	)

	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=memory" CACHE INTERNAL "" FORCE)
	set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=memory" CACHE INTERNAL "" FORCE)
	set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fsanitize=memory" CACHE INTERNAL "" FORCE)

	function(sanitizer_fail_test_on_error test_name)
		set_tests_properties(${test_name} PROPERTIES FAIL_REGULAR_EXPRESSION "WARNING: MemorySanitizer")
	endfunction(sanitizer_fail_test_on_error)

elseif (CMAKE_BUILD_TYPE STREQUAL ThreadSanitizer)
	message(STATUS "ThreadSanitizer enabled")

	add_compile_options(
		-g3
		-fsanitize=thread
	)

	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=thread" CACHE INTERNAL "" FORCE)
	set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=thread" CACHE INTERNAL "" FORCE)
	set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fsanitize=thread" CACHE INTERNAL "" FORCE)

	function(sanitizer_fail_test_on_error test_name)
		set_tests_properties(${test_name} PROPERTIES FAIL_REGULAR_EXPRESSION "WARNING: ThreadSanitizer")
	endfunction(sanitizer_fail_test_on_error)

elseif (CMAKE_BUILD_TYPE STREQUAL UndefinedBehaviorSanitizer)
	message(STATUS "UndefinedBehaviorSanitizer enabled")

	add_compile_options(
		-g3
		-fsanitize=alignment
		-fsanitize=bool
		-fsanitize=bounds
		-fsanitize=enum
		-fsanitize=float-cast-overflow
		-fsanitize=float-divide-by-zero
		-fsanitize=integer-divide-by-zero
		-fsanitize=nonnull-attribute
		-fsanitize=null
		-fsanitize=object-size
		-fsanitize=return
		-fsanitize=returns-nonnull-attribute
		-fsanitize=shift
		-fsanitize=signed-integer-overflow
		-fsanitize=unreachable
		-fsanitize=vla-bound
		-fno-sanitize-recover=bounds,null
	)

	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=undefined" CACHE INTERNAL "" FORCE)
	set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=undefined" CACHE INTERNAL "" FORCE)
	set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fsanitize=undefined" CACHE INTERNAL "" FORCE)

	function(sanitizer_fail_test_on_error test_name)
		set_tests_properties(${test_name} PROPERTIES FAIL_REGULAR_EXPRESSION "runtime error:")
	endfunction(sanitizer_fail_test_on_error)

elseif (CMAKE_BUILD_TYPE STREQUAL FuzzTesting)
	message(STATUS "FuzzTesting enabled")

	add_compile_options(
		-g3
		-fsanitize=fuzzer,address,undefined
		-DFUZZTESTING
	)

	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=fuzzer,address,undefined" CACHE INTERNAL "" FORCE)
	set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=fuzzer,address,undefined" CACHE INTERNAL "" FORCE)
	set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fsanitize=fuzzer,address,undefined" CACHE INTERNAL "" FORCE)

	function(sanitizer_fail_test_on_error test_name)
		# Not sure what to do here
	endfunction(sanitizer_fail_test_on_error)
else()

	function(sanitizer_fail_test_on_error test_name)
		# default: don't do anything
	endfunction(sanitizer_fail_test_on_error)

endif()
