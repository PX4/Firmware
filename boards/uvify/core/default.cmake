
px4_add_board(
	PLATFORM nuttx
	VENDOR uvify
	MODEL core
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common
	UAVCAN_INTERFACES 1
	SERIAL_PORTS
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL3:/dev/ttyS6
	DRIVERS
		adc
		barometer/ms5611
		batt_smbus
		camera_capture
		camera_trigger
		distance_sensor # all available distance sensor drivers
		dshot
		gps
		imu/invensense/icm20602
		imu/invensense/icm20608g
		imu/invensense/mpu9250
		irlock
		lights/rgbled_ncp5623c
		magnetometer/bmm150
		magnetometer/lis3mdl
		magnetometer/ist8310
		optical_flow # all available optical flow drivers
		pca9685
		pwm_input
		pwm_out_sim
		pwm_out
		rc_input
		tone_alarm
		uavcan
	MODULES
		attitude_estimator_q
		battery_status
		camera_feedback
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
		sih
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
