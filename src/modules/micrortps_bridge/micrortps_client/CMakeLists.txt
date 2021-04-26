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

set(msg_out_path ${CMAKE_CURRENT_BINARY_DIR})
get_filename_component(micrortps_bridge_path ${msg_out_path} PATH)

if (NOT "${config_rtps_send_topics}" STREQUAL "" OR NOT "${config_rtps_receive_topics}" STREQUAL "")

	set(send_topic_files)
	foreach(topic ${config_rtps_send_topics})
		list(APPEND send_topic_files ${PX4_SOURCE_DIR}/msg/${topic}.msg)
	endforeach()

	set(receive_topic_files)
	foreach(topic ${config_rtps_receive_topics})
		list(APPEND receive_topic_files ${PX4_SOURCE_DIR}/msg/${topic}.msg)
	endforeach()

	if (NOT "${config_rtps_send_alias_topics}" STREQUAL "")
		set(config_rtps_send_topics "${config_rtps_send_topics};${config_rtps_send_alias_topics}")
	endif()

	if (NOT "${config_rtps_receive_alias_topics}" STREQUAL "")
		set(config_rtps_receive_topics "${config_rtps_receive_topics};${config_rtps_receive_alias_topics}")
	endif()

	foreach(topic ${config_rtps_send_topics})
		list(APPEND topic_bridge_files_out ${micrortps_bridge_path}/micrortps_agent/${topic}_Publisher.cpp)
		list(APPEND topic_bridge_files_out ${micrortps_bridge_path}/micrortps_agent/${topic}_Publisher.h)
	endforeach()

	foreach(topic ${config_rtps_receive_topics})
		list(APPEND topic_bridge_files_out ${micrortps_bridge_path}/micrortps_agent/${topic}_Subscriber.cpp)
		list(APPEND topic_bridge_files_out ${micrortps_bridge_path}/micrortps_agent/${topic}_Subscriber.h)
	endforeach()

	list(APPEND topic_bridge_files_out
		${micrortps_bridge_path}/micrortps_client/microRTPS_client.cpp
		${micrortps_bridge_path}/micrortps_client/microRTPS_transport.cpp
		${micrortps_bridge_path}/micrortps_client/microRTPS_transport.h
		)

	add_custom_command(OUTPUT ${topic_bridge_files_out}
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/msg/tools/generate_microRTPS_bridge.py
			--fastrtpsgen-dir $ENV{FASTRTPSGEN_DIR}
			--generate-idl
			--mkdir-build
			--generate-cmakelists
			--topic-msg-dir ${PX4_SOURCE_DIR}/msg
			--uorb-templates-dir templates/uorb_microcdr
			--urtps-templates-dir templates/urtps
			--agent-outdir ${micrortps_bridge_path}/micrortps_agent/src
			--client-outdir ${micrortps_bridge_path}/micrortps_client
			--idl-dir ${micrortps_bridge_path}/micrortps_agent/idl
			>micrortps_bridge.log 2>&1 || cat micrortps_bridge.log
		DEPENDS ${send_topic_files} ${receive_topic_files}
		COMMENT "Generating RTPS topic bridge"
	)
	add_custom_target(topic_bridge_files DEPENDS ${topic_bridge_files_out})

	px4_add_module(
		MODULE modules__micrortps_bridge__micrortps_client
		MAIN micrortps_client
		STACK_MAIN 4096
		INCLUDES
			${CMAKE_CURRENT_SOURCE_DIR}
			${micrortps_bridge_path}/micrortps_client
		SRCS
			microRTPS_client_main.cpp
			${micrortps_bridge_path}/micrortps_client/microRTPS_client.cpp
			${micrortps_bridge_path}/micrortps_client/microRTPS_transport.cpp
		MODULE_CONFIG
			module.yaml
		DEPENDS
			topic_bridge_files
	)
	target_link_libraries(modules__micrortps_bridge__micrortps_client PRIVATE uorb_msgs_microcdr)
endif()
