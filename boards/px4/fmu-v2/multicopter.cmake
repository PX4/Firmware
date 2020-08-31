
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v2
	LABEL multicopter
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common
	IO px4_io-v2_default
	#UAVCAN_INTERFACES 2
	CONSTRAINED_FLASH
	SERIAL_PORTS
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS6
	DRIVERS
		adc
		barometer/ms5611
		#batt_smbus
		camera_capture
		camera_trigger
		distance_sensor # all available distance sensor drivers
		gps
		imu/l3gd20
		imu/lsm303d
		imu/mpu6000
		#imu/invensense/mpu9250
		irlock
		lights/rgbled
		magnetometer/hmc5883
		#optical_flow/px4flow
		pwm_out
		px4io
		tone_alarm
	MODULES
		#attitude_estimator_q
		battery_status
		camera_feedback
		commander
		dataman
		ekf2
		#events
		land_detector
		landing_target_estimator
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
		#sih
		#temperature_compensation
		vmount
	SYSTEMCMDS
		#bl_update
		#dumpfile
		#esc_calib
		hardfault_log
		#i2cdetect
		#led_control
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
		#usb_connected
		ver
		#work_queue
	)
