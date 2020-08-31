
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v2
	LABEL lpe
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common
	BOOTLOADER ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/extras/px4fmuv3_bl.bin
	IO px4_io-v2_default
	#TESTING
	CONSTRAINED_FLASH
	#UAVCAN_INTERFACES 2

	SERIAL_PORTS
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS6

	DRIVERS
		adc
		#barometer # all available barometer drivers
		barometer/ms5611
		#batt_smbus
		camera_capture
		camera_trigger
		#differential_pressure # all available differential pressure drivers
		#differential_pressure/ms4525
		distance_sensor # all available distance sensor drivers
		gps
		#heater
		#imu/adis16448
		#imu # all available imu drivers
		imu/l3gd20
		imu/lsm303d
		imu/invensense/mpu6000
		#iridiumsbd
		irlock
		#lights/blinkm
		lights/rgbled
		#magnetometer # all available magnetometer drivers
		magnetometer/hmc5883
		#mkblctrl
		#optical_flow # all available optical flow drivers
		optical_flow/px4flow
		#pca9685
		#protocol_splitter
		#pwm_input
		pwm_out_sim
		pwm_out
		px4io
		#tap_esc
		#telemetry # all available telemetry drivers
		#test_ppm
		tone_alarm
		#uavcan

	MODULES
		attitude_estimator_q
		camera_feedback
		commander
		dataman
		#ekf2
		events
		#fw_att_control
		#fw_pos_control_l1
		#rover_pos_control
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_rate_control
		mc_pos_control
		navigator
		battery_status
		rc_update
		sensors
		temperature_compensation
		vmount
		#vtol_att_control
		#airspeed_selector

	SYSTEMCMDS
		bl_update
		#dumpfile
		#esc_calib
		hardfault_log
		i2cdetect
		#led_control
		mixer
		#motor_ramp
		motor_test
		mtd
		#nshterm
		param
		perf
		pwm
		reboot
		#sd_bench
		#tests # tests and test runner
		top
		#topic_listener
		tune_control
		ver
		work_queue

	EXAMPLES
		#fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		#hello
		#hwtest # Hardware test
		#px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		#px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		#rover_steering_control # Rover example app
	)
