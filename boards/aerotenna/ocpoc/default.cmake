add_definitions(
	-D__PX4_LINUX
)

px4_add_board(
	VENDOR aerotenna
	MODEL ocpoc
	LABEL default
	PLATFORM posix
	ARCHITECTURE cortex-a9
	ROMFSROOT px4fmu_common
	TOOLCHAIN arm-linux-gnueabihf
	TESTING
	SERIAL_PORTS
		GPS1:/dev/ttyS3  # GPS/Compass #1           (OcPoC Port 6)
		GPS2:/dev/ttyS7  # GPS/Compass #2           (OcPoC Port 7)
		GPS3:/dev/ttyS1  # GPS/Compass #3           (OcPoC Port 9)
		TEL1:/dev/ttyPS1 # Radio Telemetry          (OcPoC Port 4)
		TEL2:/dev/ttyS6  # uLanding Radar Altimeter (OcPoC Port 8)
		TEL3:/dev/ttyS2  #                          (OcPoC Port 2)
		TEL4:/dev/ttyS0  # uSharp-Patch             (OcPoC Port 5)
	DRIVERS
		#barometer # all available barometer drivers
		barometer/ms5611
		batt_smbus
		camera_capture
		camera_trigger
		differential_pressure # all available differential pressure drivers
		distance_sensor # all available distance sensor drivers
		gps
		imu/invensense/mpu9250
		lights/rgbled
		linux_pwm_out
		#magnetometer # all available magnetometer drivers
		magnetometer/hmc5883
		pwm_out_sim
		rc_input
		#telemetry # all available telemetry drivers
	MODULES
		airspeed_selector
		attitude_estimator_q
		battery_status
		camera_feedback
		commander
		dataman
		ekf2
		events
		flight_mode_manager
		fw_att_control
		fw_pos_control_l1
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_hover_thrust_estimator
		mc_pos_control
		mc_rate_control
		#micrortps_bridge
		navigator
		rc_update
		rover_pos_control
		sensors
		sih
		#simulator
		temperature_compensation
		vmount
		vtol_att_control
	SYSTEMCMDS
		esc_calib
		led_control
		mixer
		motor_ramp
		param
		perf
		pwm
		sd_bench
		system_time
		shutdown
		tests # tests and test runner
		#top
		topic_listener
		tune_control
		ver
		work_queue
	EXAMPLES
		fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		hello
		#hwtest # Hardware test
		#matlab_csv_serial
		px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		rover_steering_control # Rover example app
	)
