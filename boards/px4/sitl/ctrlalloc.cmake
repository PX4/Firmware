
px4_add_board(
	PLATFORM posix
	VENDOR px4
	MODEL sitl
	LABEL default
	TESTING
	DRIVERS
		#barometer # all available barometer drivers
		#batt_smbus
		camera_capture
		camera_trigger
		#differential_pressure # all available differential pressure drivers
		#distance_sensor # all available distance sensor drivers
		gps
		#imu # all available imu drivers
		#magnetometer # all available magnetometer drivers
		pwm_out_sim
		rpm/rpm_simulator
		#telemetry # all available telemetry drivers
		tone_alarm
		#uavcan
	MODULES
		airspeed_selector
		angular_velocity_controller
		attitude_estimator_q
		camera_feedback
		commander
		control_allocator
		dataman
		ekf2
		events
		fw_att_control
		fw_pos_control_l1
		land_detector
		landing_target_estimator
		#load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_hover_thrust_estimator
		mc_pos_control
		mc_rate_control
		navigator
		rc_update
		replay
		rover_pos_control
		sensors
		#sih
		simulator
		temperature_compensation
		vmount
		vtol_att_control
		uuv_att_control

	SYSTEMCMDS
		#dumpfile
		dyn
		esc_calib
		led_control
		mixer
		motor_ramp
		motor_test
		#mtd
		#nshterm
		param
		perf
		pwm
		sd_bench
		shutdown
		tests # tests and test runner
		#top
		topic_listener
		tune_control
		ver
		work_queue
	EXAMPLES
		dyn_hello # dynamically loading modules example
		fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		hello
		#hwtest # Hardware test
		#matlab_csv_serial
		px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		rover_steering_control # Rover example app
		uuv_example_app
		work_item
	)

set(config_sitl_viewer jmavsim CACHE STRING "viewer for sitl")
set_property(CACHE config_sitl_viewer PROPERTY STRINGS "jmavsim;none")

set(config_sitl_debugger disable CACHE STRING "debugger for sitl")
set_property(CACHE config_sitl_debugger PROPERTY STRINGS "disable;gdb;lldb")

# If the environment variable 'replay' is defined, we are building with replay
# support. In this case, we enable the orb publisher rules.
set(REPLAY_FILE "$ENV{replay}")
if(REPLAY_FILE)
	message(STATUS "Building with uorb publisher rules support")
	add_definitions(-DORB_USE_PUBLISHER_RULES)

	message(STATUS "Building without lockstep for replay")
	set(ENABLE_LOCKSTEP_SCHEDULER no)
else()
	set(ENABLE_LOCKSTEP_SCHEDULER yes)
endif()
