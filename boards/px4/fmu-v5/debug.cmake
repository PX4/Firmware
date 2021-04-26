
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v5
	LABEL debug
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common
	IO px4_io-v2_default
	TESTING
	#UAVCAN_INTERFACES 2
	CONSTRAINED_FLASH
	SERIAL_PORTS
		GPS1:/dev/ttyS0
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS3
	DRIVERS
		#adc/ads1115
		adc/board_adc
		#barometer # all available barometer drivers
		barometer/ms5611
		#batt_smbus
		camera_capture
		camera_trigger
		#differential_pressure # all available differential pressure drivers
		distance_sensor # all available distance sensor drivers
		#dshot
		gps
		#heater
		#imu # all available imu drivers
		#imu/bosch/bmi055
		#imu/invensense/icm20602
		imu/invensense/icm20689
		#imu/invensense/icm20948 # required for ak09916 mag
		#irlock
		lights # all available light drivers
		lights/rgbled_pwm
		magnetometer # all available magnetometer drivers
		optical_flow # all available optical flow drivers
		#osd
		#pca9685
		#pca9685_pwm_out
		#power_monitor/ina226
		#protocol_splitter
		#pwm_input
		pwm_out_sim
		pwm_out
		px4io
		rc_input
		#roboclaw
		#rpm
		safety_button
		telemetry # all available telemetry drivers
		test_ppm
		tone_alarm
		#uavcan
	MODULES
		airspeed_selector
		#attitude_estimator_q
		battery_status
		#camera_feedback
		commander
		dataman
		ekf2
		#esc_battery
		events
		flight_mode_manager
		fw_att_control
		fw_pos_control_l1
		gyro_calibration
		#gyro_fft
		land_detector
		#landing_target_estimator
		load_mon
		#local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_hover_thrust_estimator
		mc_pos_control
		mc_rate_control
		#micrortps_bridge
		navigator
		rc_update
		#rover_pos_control
		sensors
		#sih
		#temperature_compensation
		#uuv_att_control
		#uuv_pos_control
		#vmount
		vtol_att_control
	SYSTEMCMDS
		bl_update
		dmesg
		dumpfile
		esc_calib
		gpio
		hardfault_log
		i2cdetect
		led_control
		mft
		mixer
		motor_ramp
		motor_test
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		sd_bench
		#serial_test
		system_time
		tests # tests and test runner
		top
		topic_listener
		tune_control
		uorb
		usb_connected
		ver
		work_queue
	EXAMPLES
		fake_gps
		#fake_gyro
		#fake_magnetometer
		#fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		#hello
		#hwtest # Hardware test
		#matlab_csv_serial
		#px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		#px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		#rover_steering_control # Rover example app
		#uuv_example_app
		#work_item
	)
