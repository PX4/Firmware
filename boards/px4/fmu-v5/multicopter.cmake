
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v5
	LABEL multicopter
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common
	IO px4_io-v2_default
	UAVCAN_INTERFACES 2
	SERIAL_PORTS
		GPS1:/dev/ttyS0
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS3
	DRIVERS
		adc/board_adc
		barometer # all available barometer drivers
		smart_battery # all available smart battery drivers
		camera_capture
		camera_trigger
		distance_sensor # all available distance sensor drivers
		dshot
		gps
		#heater
		#imu # all available imu drivers
		imu/analog_devices/adis16448
		imu/bosch/bmi055
		imu/invensense/icm20602
		imu/invensense/icm20689
		imu/invensense/icm20948 # required for ak09916 mag
		irlock
		lights # all available light drivers
		lights/rgbled_pwm
		magnetometer # all available magnetometer drivers
		optical_flow # all available optical flow drivers
		osd
		pca9685
		pca9685_pwm_out
		power_monitor/ina226
		#protocol_splitter
		pwm_input
		pwm_out_sim
		pwm_out
		px4io
		rc_input
		roboclaw
		rpm
		safety_button
		telemetry # all available telemetry drivers
		tone_alarm
		uavcan
	MODULES
		attitude_estimator_q
		battery_status
		camera_feedback
		commander
		dataman
		ekf2
		esc_battery
		events
		flight_mode_manager
		gyro_calibration
		gyro_fft
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
		navigator
		rc_update
		sensors
		sih
		temperature_compensation
		vmount
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
		serial_test
		system_time
		top
		topic_listener
		tune_control
		uorb
		usb_connected
		ver
		work_queue
	)
