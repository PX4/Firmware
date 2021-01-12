
px4_add_board(
	PLATFORM nuttx
	VENDOR nxp
	MODEL fmurt1062-v1
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common
	LINKER_PREFIX ocram
#	UAVCAN_INTERFACES 2
	SERIAL_PORTS
		GPS1:/dev/ttyS1
		TEL1:/dev/ttyS2
		TEL2:/dev/ttyS3
		GPS2:/dev/ttyS4

	DRIVERS
		adc/board_adc
		barometer # all available barometer drivers
		batt_smbus
		camera_capture
		camera_trigger
		distance_sensor # all available distance sensor drivers
#		dshot not ported
		gps
		#imu/adis16448
		#imu/adis16477
		#imu/adis16497
		#imu # all available imu drivers
		imu/bosch/bmi055
		imu/invensense/icm20602
		imu/invensense/icm20689
		#imu/mpu6000 # legacy icm20602/icm20689 driver
		irlock
		lights/blinkm
		lights/rgbled
		lights/rgbled_ncp5623c
		lights/rgbled_pwm
		magnetometer # all available magnetometer drivers
		optical_flow # all available optical flow drivers
#		pwm_input - not ptorable
		pwm_out_sim
		pwm_out
		rc_input
		roboclaw
		safety_button
		tap_esc
		telemetry # all available telemetry drivers
		tone_alarm
#		uavcan
	MODULES
		attitude_estimator_q
		battery_status
		camera_feedback
		commander
		dataman
		ekf2
		events
		flight_mode_manager
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
#		bl_update
		dmesg
		dumpfile
		esc_calib
		#hardfault_log # Needs bbsrm
		i2cdetect
		led_control
		mixer
		motor_ramp
		motor_test
		mft
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
		usb_connected
		ver
		work_queue
		serial_test
	EXAMPLES
##		fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
#		hello
#		hwtest # Hardware test
		#matlab_csv_serial
#		px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
#		px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
#		rover_steering_control # Rover example app
#		uuv_example_app
	)
