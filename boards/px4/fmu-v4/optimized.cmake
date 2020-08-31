
# set Release for -O3
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
add_compile_options(-Wno-error=array-bounds)

px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v4
	LABEL optimized
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common
	TESTING
	UAVCAN_INTERFACES 1
	SERIAL_PORTS
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
	DRIVERS
		adc
		barometer # all available barometer drivers
		batt_smbus
		camera_capture
		camera_trigger
		differential_pressure # all available differential pressure drivers
		distance_sensor # all available distance sensor drivers
		dshot
		gps
		#heater
		#imu # all available imu drivers
		#imu/adis16448
		#imu/adis16477
		#imu/adis16497
		imu/invensense/icm20602
		imu/invensense/icm20608g
		#imu/invensense/icm40609d
		#imu/invensense/mpu6500
		imu/invensense/mpu9250
		irlock
		lights/blinkm
		lights/rgbled
		lights/rgbled_ncp5623c
		magnetometer # all available magnetometer drivers
		optical_flow # all available optical flow drivers
		#pwm_input
		pwm_out_sim
		pwm_out
		rc_input
		#roboclaw
		safety_button
		#tap_esc
		#telemetry # all available telemetry drivers
		test_ppm
		tone_alarm
		#uavcan
	MODULES
		airspeed_selector
		#attitude_estimator_q
		battery_status
		camera_feedback
		commander
		dataman
		ekf2
		events
		fw_att_control
		fw_pos_control_l1
		land_detector
		#landing_target_estimator
		load_mon
		#local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_hover_thrust_estimator
		mc_pos_control
		mc_rate_control
		#micrortps_bridge
		navigator
		rc_update
		#rover_pos_control
		sensors
		#sih
		temperature_compensation
		#uuv_att_control
		#vmount
		vtol_att_control
	SYSTEMCMDS
		bl_update
		#dmesg
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
		tests # tests and test runner
		top
		topic_listener
		tune_control
		usb_connected
		ver
		work_queue
	)
