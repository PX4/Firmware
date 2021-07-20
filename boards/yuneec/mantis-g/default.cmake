
px4_add_board(
	PLATFORM nuttx
	VENDOR yuneec
	MODEL mantis-g
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common
	SERIAL_PORTS
		#GPS1:/dev/ttyS0
		#TEL1:/dev/ttyS1
		#TEL2:/dev/ttyS2
		#TEL4:/dev/ttyS3
	DRIVERS
		#adc # TODO: not sure what happened to this
		barometer/mpc2520
		batt_smbus
		camera_capture
		camera_trigger
		distance_sensor # all available distance sensor drivers
		gps
		#heater
		#imu/mpu6000 # TODO: required?
		lights/rgbled
		# lights/rgbled_ncp5623c
		lights/rgbled_pwm
		# magnetometer # all available magnetometer drivers
		# magnetometer/ist8310 # TODO: add
		# optical_flow # all available optical flow drivers
		#pca9685
		power_monitor/ina226
		#protocol_splitter
		pwm_input
		pwm_out_sim
		rc_input
		tap_esc # TODO: update
		#telemetry # all available telemetry drivers
		tone_alarm
	MODULES
		battery_status
		camera_feedback
		commander
		dataman
		ekf2
		events
		land_detector
		load_mon
		logger
		mavlink
		mc_att_control
		mc_pos_control
		mc_rate_control
		navigator
		rc_update
		sensors
		sih
		vmount
	SYSTEMCMDS
		bl_update
		dmesg
		dumpfile
		esc_calib
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
