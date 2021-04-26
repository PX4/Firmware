
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v5
	LABEL rover
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
		batt_smbus
		camera_capture
		camera_trigger
		distance_sensor # all available distance sensor drivers
		gps
		imu/analog_devices/adis16448
		imu/bosch/bmi055
		imu/invensense/icm20602
		imu/invensense/icm20689
		imu/invensense/icm20948 # required for ak09916 mag
		lights/rgbled
		lights/rgbled_ncp5623c
		lights/rgbled_pwm
		magnetometer # all available magnetometer drivers
		optical_flow # all available optical flow drivers
		pca9685
		pwm_input
		pwm_out_sim
		pwm_out
		px4io
		rc_input
		roboclaw
		safety_button
		telemetry # all available telemetry drivers
		tone_alarm
		uavcan
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
		navigator
		rc_update
		rover_pos_control
		sensors
		temperature_compensation
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
		system_time
		top
		topic_listener
		tune_control
		uorb
		usb_connected
		ver
		work_queue
	)
