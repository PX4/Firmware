
px4_add_board(
	PLATFORM nuttx
	VENDOR bitcraze
	MODEL crazyflie
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common
	CONSTRAINED_FLASH
	DRIVERS
		barometer/lps25h
		distance_sensor/vl53l0x
		gps
		imu/mpu9250
		optical_flow/pmw3901
		pwm_out
	MODULES
		attitude_estimator_q
		#camera_feedback
		commander
		dataman
		ekf2
		events
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
		#temperature_compensation
	SYSTEMCMDS
		bl_update
		dmesg
		dumpfile
		esc_calib
		hardfault_log
		i2cdetect
		led_control
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
		top
		topic_listener
		tune_control
		usb_connected
		ver
		work_queue
	)
