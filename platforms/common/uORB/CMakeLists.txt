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

# this includes the generated topics directory
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(SRCS)

set(SRCS_COMMON
	ORBSet.hpp
	Publication.hpp
	PublicationMulti.hpp
	Subscription.cpp
	Subscription.hpp
	SubscriptionCallback.hpp
	SubscriptionInterval.hpp
	SubscriptionMultiArray.hpp
	uORB.cpp
	uORB.h
	uORBCommon.hpp
	uORBCommunicator.hpp
	uORBManager.hpp
	uORBUtils.cpp
	uORBUtils.hpp
	uORBDeviceMaster.hpp
	uORBDeviceNode.hpp
	)

set(SRCS_KERNEL
	uORBDeviceMaster.cpp
	uORBDeviceNode.cpp
	uORBManager.cpp
	)

set(SRCS_USER
	uORBManagerUsr.cpp
	)

if (NOT DEFINED CONFIG_BUILD_FLAT AND "${PX4_PLATFORM}" MATCHES "nuttx")
	# This is the kernel side library in nuttx kernel/protected build
	px4_add_library(uORB_kernel
			${SRCS_COMMON}
			${SRCS_KERNEL}
		)

	# Sources for the user side library in nuttx kernel/protected build
	list(APPEND SRCS
		${SRCS_COMMON}
		${SRCS_USER}
		)
	target_compile_options(uORB_kernel PRIVATE ${MAX_CUSTOM_OPT_LEVEL} -D__KERNEL__)

else()
	# Sources for all other targets (flat build, posix...)
	list(APPEND SRCS
		${SRCS_COMMON}
		${SRCS_KERNEL}
		)
endif()

px4_add_library(uORB
		${SRCS}
	)

target_compile_options(uORB PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_link_libraries(uORB PRIVATE cdev)

if(PX4_TESTING)
	add_subdirectory(uORB_tests)
endif()
