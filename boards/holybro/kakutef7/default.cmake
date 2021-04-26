
px4_add_board(
	PLATFORM nuttx
	VENDOR holybro
	MODEL kakutef7
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	EXTERNAL_METADATA
	ROMFSROOT px4fmu_common
	SERIAL_PORTS
		TEL1:/dev/ttyS0 # UART1
		TEL2:/dev/ttyS1 # UART2
#		TEL3:/dev/ttyS2 # UART3 (currently NuttX console)
		GPS1:/dev/ttyS3 # UART4
		RC:/dev/ttyS4 # UART6
		# /dev/ttyS5: UART7 (ESC telemetry)
	DRIVERS
		adc/board_adc
		barometer/bmp280
		dshot
		gps
		imu/invensense/icm20689
		imu/invensense/mpu6000
		#magnetometer
		magnetometer/isentek/ist8310
		#optical_flow/px4flow
		osd
		#pwm_out_sim
		pwm_out
		rc_input
		#telemetry
		telemetry/frsky_telemetry
		tone_alarm
	MODULES
		attitude_estimator_q
		battery_status
		commander
		dataman
		#ekf2
		events
		flight_mode_manager
		gyro_calibration
		#gyro_fft
		land_detector
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
		sensors
		#temperature_compensation
	SYSTEMCMDS
		bl_update
		dmesg
		#dumpfile
		#esc_calib
		hardfault_log
		#i2cdetect
		#led_control
		mixer
		#motor_ramp
		#motor_test
		#nshterm
		param
		perf
		pwm
		reboot
		#reflect
		#sd_bench
		top
		#topic_listener
		#tune_control
		#uorb
		#usb_connected
		ver
		work_queue
	)
