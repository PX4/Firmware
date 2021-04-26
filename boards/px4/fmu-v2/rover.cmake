
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v2
	LABEL rover
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	ROMFSROOT px4fmu_common
	IO px4_io-v2_default
	CONSTRAINED_FLASH

	SERIAL_PORTS
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS6

	DRIVERS
		adc/board_adc
		barometer/ms5611
		batt_smbus
		camera_capture
		camera_trigger
		distance_sensor # all available distance sensor drivers
		gps
		imu/l3gd20
		imu/lsm303d
		imu/invensense/mpu6000
		#imu/invensense/mpu9250
		lights/rgbled
		magnetometer/hmc5883
		optical_flow/px4flow
		pwm_out
		px4io
		tone_alarm

	MODULES
		camera_feedback
		commander
		dataman
		ekf2
		events
		rover_pos_control
		land_detector
		load_mon
		logger
		mavlink
		navigator
		battery_status
		rc_update
		sensors
		temperature_compensation
		#vmount

	SYSTEMCMDS
		bl_update
		#dumpfile
		#esc_calib
		hardfault_log
		i2cdetect
		#led_control
		mft
		mixer
		#motor_ramp
		#motor_test
		mtd
		#nshterm
		param
		perf
		pwm
		reboot
		#sd_bench
		top
		#topic_listener
		tune_control
		uorb
		usb_connected
		ver
		work_queue
	)
