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

#=============================================================================
# RTPS and micro-cdr
#

find_program(FASTRTPSGEN fastrtpsgen PATHS $ENV{FASTRTPSGEN_DIR})
if(NOT FASTRTPSGEN)
	message(FATAL_ERROR "Unable to find fastrtpsgen")
else()
	execute_process(
		COMMAND $ENV{FASTRTPSGEN_DIR}fastrtpsgen -version
		OUTPUT_VARIABLE FASTRTPSGEN_VERSION
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_QUIET
	)
	message(STATUS "${FASTRTPSGEN_VERSION}")
endif()

if (EXISTS "${PX4_SOURCE_DIR}/msg/tools/uorb_rtps_message_ids.yaml")
	set(config_rtps_send_topics)
	execute_process(
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/msg/tools/uorb_rtps_classifier.py -sa
		OUTPUT_VARIABLE config_rtps_send_topics
	)
	set(config_rtps_send_alias_topics "")
	string(FIND ${config_rtps_send_topics} "alias" found_send_alias)
	if (NOT ${found_send_alias} EQUAL "-1")
		STRING(REGEX REPLACE ".*alias " "" config_rtps_send_alias_topics "${config_rtps_send_topics}")
		STRING(REPLACE ", " ";" config_rtps_send_alias_topics "${config_rtps_send_alias_topics}")
		STRING(REPLACE "\n" "" config_rtps_send_alias_topics "${config_rtps_send_alias_topics}")
		STRING(REGEX REPLACE " alias.*" "" config_rtps_send_topics "${config_rtps_send_topics}")
	endif()
	STRING(REGEX REPLACE ", " ";" config_rtps_send_topics "${config_rtps_send_topics}")
	STRING(REGEX REPLACE "\n" "" config_rtps_send_topics "${config_rtps_send_topics}")

	set(config_rtps_receive_topics)
	execute_process(
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/msg/tools/uorb_rtps_classifier.py -ra
		OUTPUT_VARIABLE config_rtps_receive_topics
	)
	set(config_rtps_receive_alias_topics "")
	string(FIND ${config_rtps_receive_topics} "alias" found_receive_alias)
	if (NOT ${found_receive_alias} EQUAL "-1")
		STRING(REGEX REPLACE ".*alias " "" config_rtps_receive_alias_topics "${config_rtps_receive_topics}")
		STRING(REPLACE ", " ";" config_rtps_receive_alias_topics "${config_rtps_receive_alias_topics}")
		STRING(REPLACE "\n" "" config_rtps_receive_alias_topics "${config_rtps_receive_alias_topics}")
		STRING(REGEX REPLACE " alias.*" "" config_rtps_receive_topics "${config_rtps_receive_topics}")
	endif()
	STRING(REPLACE ", " ";" config_rtps_receive_topics "${config_rtps_receive_topics}")
	STRING(REPLACE "\n" "" config_rtps_receive_topics "${config_rtps_receive_topics}")
endif()

if (FASTRTPSGEN AND (config_rtps_send_topics OR config_rtps_receive_topics))
	option(GENERATE_RTPS_BRIDGE "enable RTPS and microCDR" ON)
endif()

if (GENERATE_RTPS_BRIDGE)
	add_subdirectory(micrortps_client)

	###############################################################################
	# micro-cdr serialization
	###############################################################################
	include(px4_git)
	px4_add_git_submodule(TARGET git_micro_cdr PATH micro-CDR)

	set(UCDR_SUPERBUILD CACHE BOOL "Disable micro-CDR superbuild compilation.")
	add_subdirectory(micro-CDR)

	set(msg_out_path_microcdr ${PX4_BINARY_DIR}/uORB_microcdr/topics)
	set(msg_source_out_path_microcdr ${CMAKE_CURRENT_BINARY_DIR}/topics_microcdr_sources)

	set(uorb_headers_microcdr)
	set(uorb_sources_microcdr)

	# send topic files
	STRING(REGEX REPLACE ";" ", " send_list "${config_rtps_send_topics};${config_rtps_send_alias_topics}")
	message(STATUS "RTPS send: ${send_list}")
	set(send_topic_files)
	foreach(topic ${config_rtps_send_topics})
		list(APPEND send_topic_files ${PX4_SOURCE_DIR}/msg/${topic}.msg)
		list(APPEND uorb_headers_microcdr ${msg_out_path_microcdr}/${topic}.h)
		list(APPEND uorb_sources_microcdr ${msg_source_out_path_microcdr}/${topic}.cpp)
	endforeach()

	# receive topic files
	STRING(REGEX REPLACE ";" ", " rcv_list "${config_rtps_receive_topics};${config_rtps_receive_alias_topics}")
	message(STATUS "RTPS receive: ${rcv_list}")
	set(receive_topic_files)
	foreach(topic ${config_rtps_receive_topics})
		list(APPEND receive_topic_files ${PX4_SOURCE_DIR}/msg/${topic}.msg)
		list(APPEND uorb_headers_microcdr ${msg_out_path_microcdr}/${topic}.h)
		list(APPEND uorb_sources_microcdr ${msg_source_out_path_microcdr}/${topic}.cpp)
	endforeach()

	list(REMOVE_DUPLICATES uorb_headers_microcdr)
	list(REMOVE_DUPLICATES uorb_sources_microcdr)

	# Generate uORB serialization headers
	add_custom_command(OUTPUT ${uorb_headers_microcdr}
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/msg/tools/px_generate_uorb_topic_files.py
			--headers
			-f ${send_topic_files} ${receive_topic_files}
			-i ${PX4_SOURCE_DIR}/msg/
			-o ${msg_out_path_microcdr}
			-e ${PX4_SOURCE_DIR}/msg/templates/uorb_microcdr
			-t ${CMAKE_CURRENT_BINARY_DIR}/tmp/headers_microcdr
			-q
		DEPENDS
			${receive_topic_files}
			${send_topic_files}
			${PX4_SOURCE_DIR}/msg/tools/px_generate_uorb_topic_files.py
		COMMENT "Generating uORB microcdr topic headers"
		VERBATIM
	)
	add_custom_target(uorb_headers_microcdr_gen DEPENDS ${uorb_headers_microcdr})

	# Generate uORB serialization sources
	add_custom_command(OUTPUT ${uorb_sources_microcdr}
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/msg/tools/px_generate_uorb_topic_files.py
			--sources
			-f ${send_topic_files} ${receive_topic_files}
			-i ${PX4_SOURCE_DIR}/msg/
			-o ${msg_source_out_path_microcdr}
			-e ${PX4_SOURCE_DIR}/msg/templates/uorb_microcdr
			-t ${CMAKE_CURRENT_BINARY_DIR}/tmp/sources_microcdr
			-q
		DEPENDS
			${receive_topic_files}
			${send_topic_files}
			${PX4_SOURCE_DIR}/msg/tools/px_generate_uorb_topic_files.py
		COMMENT "Generating uORB microcdr topic sources"
		VERBATIM
	)
	px4_add_library(uorb_msgs_microcdr ${uorb_sources_microcdr})
	add_dependencies(uorb_msgs_microcdr
		uorb_headers_microcdr_gen
		git_micro_cdr
		microcdr
	)
	add_dependencies(microcdr prebuild_targets)

	# microCDR
	target_include_directories(uorb_msgs_microcdr
		PUBLIC
			${CMAKE_CURRENT_SOURCE_DIR}/micro-CDR/include
			${CMAKE_CURRENT_BINARY_DIR}/micro-CDR/include
			${CMAKE_CURRENT_BINARY_DIR}/micro-CDR/include/microcdr
	)

	target_link_libraries(uorb_msgs_microcdr PRIVATE microcdr)
endif()
