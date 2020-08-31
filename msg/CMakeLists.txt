############################################################################
#
#   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

# Support IN_LIST if() operator
cmake_policy(SET CMP0057 NEW)

set(msg_files
	actuator_armed.msg
	actuator_controls.msg
	actuator_outputs.msg
	adc_report.msg
	airspeed.msg
	airspeed_validated.msg
	battery_status.msg
	camera_capture.msg
	camera_trigger.msg
	cellular_status.msg
	collision_constraints.msg
	collision_report.msg
	commander_state.msg
	cpuload.msg
	debug_array.msg
	debug_key_value.msg
	debug_value.msg
	debug_vect.msg
	differential_pressure.msg
	distance_sensor.msg
	ekf2_timestamps.msg
	ekf_gps_drift.msg
	ekf_gps_position.msg
	esc_report.msg
	esc_status.msg
	estimator_innovations.msg
	estimator_sensor_bias.msg
	estimator_status.msg
	follow_target.msg
	geofence_result.msg
	gps_dump.msg
	gps_inject_data.msg
	home_position.msg
	hover_thrust_estimate.msg
	input_rc.msg
	iridiumsbd_status.msg
	irlock_report.msg
	landing_gear.msg
	landing_target_innovations.msg
	landing_target_pose.msg
	led_control.msg
	log_message.msg
	logger_status.msg
	manual_control_setpoint.msg
	mavlink_log.msg
	mission.msg
	mission_result.msg
	mount_orientation.msg
	multirotor_motor_limits.msg
	obstacle_distance.msg
	offboard_control_mode.msg
	onboard_computer_status.msg
	optical_flow.msg
	orb_test.msg
	orb_test_large.msg
	orb_test_medium.msg
	orbit_status.msg
	parameter_update.msg
	ping.msg
	position_controller_landing_status.msg
	position_controller_status.msg
	position_setpoint.msg
	position_setpoint_triplet.msg
	power_button_state.msg
	power_monitor.msg
	pwm_input.msg
	px4io_status.msg
	qshell_req.msg
	qshell_retval.msg
	radio_status.msg
	rate_ctrl_status.msg
	rc_channels.msg
	rc_parameter_map.msg
	rpm.msg
	safety.msg
	satellite_info.msg
	sensor_accel.msg
	sensor_accel_fifo.msg
	sensor_baro.msg
	sensor_combined.msg
	sensor_correction.msg
	sensor_gyro.msg
	sensor_gyro_fifo.msg
	sensor_mag.msg
	sensor_preflight_imu.msg
	sensor_preflight_mag.msg
	sensor_selection.msg
	subsystem_info.msg
	system_power.msg
	task_stack_info.msg
	tecs_status.msg
	telemetry_heartbeat.msg
	telemetry_status.msg
	test_motor.msg
	timesync.msg
	timesync_status.msg
	trajectory_bezier.msg
	trajectory_waypoint.msg
	transponder_report.msg
	tune_control.msg
	uavcan_parameter_request.msg
	uavcan_parameter_value.msg
	ulog_stream.msg
	ulog_stream_ack.msg
	vehicle_acceleration.msg
	vehicle_air_data.msg
	vehicle_angular_acceleration.msg
	vehicle_angular_velocity.msg
	vehicle_attitude.msg
	vehicle_attitude_setpoint.msg
	vehicle_command.msg
	vehicle_command_ack.msg
	vehicle_constraints.msg
	vehicle_control_mode.msg
	vehicle_global_position.msg
	vehicle_gps_position.msg
	vehicle_imu.msg
	vehicle_imu_status.msg
	vehicle_land_detected.msg
	vehicle_local_position.msg
	vehicle_local_position_setpoint.msg
	vehicle_magnetometer.msg
	vehicle_odometry.msg
	vehicle_rates_setpoint.msg
	vehicle_roi.msg
	vehicle_status.msg
	vehicle_status_flags.msg
	vehicle_trajectory_bezier.msg
	vehicle_trajectory_waypoint.msg
	vtol_vehicle_status.msg
	wheel_encoders.msg
	wind_estimate.msg
	yaw_estimator_status.msg
)

set(deprecated_msgs
	ekf2_innovations.msg	# 2019-11-22, Updated estimator interface and logging; replaced by 'estimator_innovations'.
	)

foreach(msg IN LISTS deprecated_msgs)
	if(msg IN_LIST msg_files)
		get_filename_component(msg_we ${msg} NAME_WE)
		list(APPEND invalid_msgs ${msg_we})
	endif()
endforeach()
if(invalid_msgs)
	list(LENGTH invalid_msgs invalid_msgs_size)
	if(${invalid_msgs_size} GREATER 1)
		foreach(msg IN LISTS invalid_msgs)
			string(CONCAT invalid_msgs_cs ${invalid_msgs_cs} "'${msg}', ")
		endforeach()
		STRING(REGEX REPLACE ", +$" "" invalid_msgs_cs ${invalid_msgs_cs})
		message(FATAL_ERROR "${invalid_msgs_cs} are listed as deprecated. Please use different names for the messages.")
	else()
		message(FATAL_ERROR "'${invalid_msgs}' is listed as deprecated. Please use a different name for the message.")
	endif()
endif()

if(NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	# Check that the msg directory and the CMakeLists.txt file exists
	if(EXISTS ${EXTERNAL_MODULES_LOCATION}/msg/CMakeLists.txt)
		add_subdirectory(${EXTERNAL_MODULES_LOCATION}/msg external_msg)

		# Add each of the external message files to the global msg_files list
		foreach(external_msg_file ${config_msg_list_external})
			list(APPEND msg_files ${EXTERNAL_MODULES_LOCATION}/msg/${external_msg_file})
		endforeach()
	endif()
endif()

# headers
set(msg_out_path ${PX4_BINARY_DIR}/uORB/topics)
set(msg_source_out_path	${CMAKE_CURRENT_BINARY_DIR}/topics_sources)

set(uorb_headers ${msg_out_path}/uORBTopics.hpp)
set(uorb_sources ${msg_source_out_path}/uORBTopics.cpp)
foreach(msg_file ${msg_files})
	get_filename_component(msg ${msg_file} NAME_WE)
	list(APPEND uorb_headers ${msg_out_path}/${msg}.h)
	list(APPEND uorb_sources ${msg_source_out_path}/${msg}.cpp)
endforeach()

if (px4_constrained_flash_build)
	set(added_arguments --constrained-flash)
endif()

# Generate uORB headers
add_custom_command(OUTPUT ${uorb_headers}
	COMMAND ${PYTHON_EXECUTABLE} tools/px_generate_uorb_topic_files.py
		--headers
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR}
		-o ${msg_out_path}
		-e templates/uorb
		-t ${CMAKE_CURRENT_BINARY_DIR}/tmp/headers
		-q
		${added_arguments}
	DEPENDS
		${msg_files}
		templates/uorb/msg.h.em
		templates/uorb/uORBTopics.hpp.em
		tools/px_generate_uorb_topic_files.py
		tools/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic headers"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)
add_custom_target(uorb_headers DEPENDS ${uorb_headers})

# Generate uORB sources
add_custom_command(OUTPUT ${uorb_sources}
	COMMAND ${PYTHON_EXECUTABLE} tools/px_generate_uorb_topic_files.py
		--sources
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR}
		-o ${msg_source_out_path}
		-e templates/uorb
		-t ${CMAKE_CURRENT_BINARY_DIR}/tmp/sources
		-q
		${added_arguments}
	DEPENDS
		${msg_files}
		templates/uorb/msg.cpp.em
		templates/uorb/uORBTopics.cpp.em
		tools/px_generate_uorb_topic_files.py
		tools/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic sources"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)

add_library(uorb_msgs ${uorb_sources})
add_dependencies(uorb_msgs prebuild_targets uorb_headers)
