############################################################################
#
#   Copyright (c) 2021 Technology Innoavation Institute. All rights reserved.
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

px4_add_library(monocypher
	monocypher/src/monocypher.c
	monocypher/src/optional/monocypher-ed25519.c
)

target_include_directories(monocypher
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/monocypher/src
)

# There is a one shadow warning in monocypher 3.1.2, ignore it
target_compile_options(monocypher PRIVATE -Wno-shadow)



file(GLOB TOMMATH_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "libtommath/*.c")
px4_add_library(libtommath
  ${TOMMATH_SRC}
)


file(GLOB_RECURSE PK_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "libtomcrypt/src/pk/*.c")
file(GLOB_RECURSE MATH_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "libtomcrypt/src/math/*.c")

px4_add_library(libtomcrypt
libtomcrypt_wrappers.c
	${PK_SRC}
	${MATH_SRC}
	libtomcrypt/src/hashes/sha2/sha256.c
	libtomcrypt/src/hashes/helper/hash_memory.c
	libtomcrypt/src/prngs/sprng.c
	libtomcrypt/src/misc/crypt/crypt_ltc_mp_descriptor.c
	libtomcrypt/src/misc/crypt/crypt_hash_is_valid.c
	libtomcrypt/src/misc/crypt/crypt_prng_is_valid.c
	libtomcrypt/src/misc/zeromem.c
)

target_include_directories(libtomcrypt
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/libtommath
        )

target_include_directories(libtomcrypt
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/libtomcrypt/src/headers
)

# libtomcrypt defines:
# ARGTYPE=3: No argument checking
add_definitions(-DARGTYPE=3 -DLTC_EASY -DLTC_NO_TEST -DLTM_DESC)

# link to libtommath and os provided random library
target_link_libraries(libtomcrypt
	PRIVATE
		libtommath
		px4_random
)

# Fix for erroneous warning on some compilers:
# "der_encode_asn1_identifier.c:39:18: error: comparison is always false due to limited range of data type"
target_compile_options(libtomcrypt PRIVATE -Wno-type-limits)
