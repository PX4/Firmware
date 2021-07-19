
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v2
	LABEL test
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	ROMFSROOT px4fmu_test
	IO px4_io-v2_default
	TESTING
	#UAVCAN_INTERFACES 2
	CONSTRAINED_FLASH
	  NO_HELP
	SERIAL_PORTS
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS6
	DRIVERS
		adc/board_adc
		#barometer # all available barometer drivers
		barometer/ms5611
		#batt_smbus
		#camera_capture
		#camera_trigger
		#differential_pressure # all available differential pressure drivers
		differential_pressure/ms4525
		#distance_sensor # all available distance sensor drivers
		distance_sensor/ll40ls
		distance_sensor/lightware_laser_serial
		#dshot
		gps
		#heater
		#imu # all available imu drivers
		#imu/analog_devices/adis16448
		#imu/adis16477
		#imu/adis16497
		imu/l3gd20
		imu/lsm303d
		imu/invensense/mpu6000
		#imu/invensense/mpu9250
		#irlock
		lights/rgbled
		#magnetometer # all available magnetometer drivers
		magnetometer/hmc5883
		#optical_flow # all available optical flow drivers
		#optical_flow/px4flow
		#osd
		#pca9685
		#protocol_splitter
		#pwm_input
		#pwm_out_sim
		pwm_out
		px4io
		#roboclaw
		#telemetry # all available telemetry drivers
		#test_ppm
		tone_alarm
		#uavcan
	MODULES
		#airspeed_selector
		#attitude_estimator_q
		battery_status
		#camera_feedback
		commander
		dataman
		#ekf2
		#events
		flight_mode_manager
		#fw_att_control
		#fw_pos_control_l1
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
		navigator
		rc_update
		#rover_pos_control
		sensors
		#sih
		#temperature_compensation
		#vmount
		#vtol_att_control
	SYSTEMCMDS
		#bl_update
		#dmesg
		#dumpfile
		#esc_calib
		hardfault_log
		#i2cdetect
		#led_control
		mft
		microbench
		mixer
		#motor_ramp
		#motor_test
		mtd
		#nshterm
		param
		perf
		pwm
		reboot
		#reflect
		#sd_bench
		#shutdown
		tests # tests and test runner
		top
		#topic_listener
		tune_control
		#uorb
		#usb_connected
		ver
		#work_queue
	EXAMPLES
		#fake_gps
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
