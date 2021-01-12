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

set(SRCS_PLATFORM)
if (${PX4_PLATFORM} STREQUAL "nuttx")
	if ("${CONFIG_I2C}" STREQUAL "y")
		list(APPEND SRCS_PLATFORM nuttx/I2C.cpp)
	endif()

	if ("${CONFIG_SPI}" STREQUAL "y")
		list(APPEND SRCS_PLATFORM nuttx/SPI.cpp)
	endif()
elseif((${PX4_PLATFORM} MATCHES "qurt"))
	list(APPEND SRCS_PLATFORM
		qurt/I2C.cpp
		qurt/SPI.cpp
	)
elseif(UNIX AND NOT APPLE AND NOT (${PX4_PLATFORM} MATCHES "qurt")) #TODO: add linux PX4 platform type
	# Linux I2Cdev and SPIdev
	list(APPEND SRCS_PLATFORM
		posix/I2C.cpp
		posix/SPI.cpp
	)
endif()

px4_add_library(drivers__device
	CDev.cpp
	${SRCS_PLATFORM}
	)

if (${PX4_PLATFORM} STREQUAL "nuttx")
	target_link_libraries(drivers__device PRIVATE nuttx_arch)
endif()

target_link_libraries(drivers__device PRIVATE cdev)
