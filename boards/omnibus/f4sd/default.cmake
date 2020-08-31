
px4_add_board(
	PLATFORM nuttx
	VENDOR omnibus
	MODEL f4sd
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common
	CONSTRAINED_FLASH
	SERIAL_PORTS
		TEL2:/dev/ttyS1
		URT6:/dev/ttyS2
	DRIVERS
		adc
		#barometer # all available barometer drivers
		barometer/bmp280
		#batt_smbus
		#camera_trigger
		#differential_pressure # all available differential pressure drivers
		#distance_sensor # all available distance sensor drivers
		dshot
		gps
		imu/invensense/icm20602
		imu/invensense/mpu6000
		#irlock
		#lights/blinkm
		lights/rgbled
		#magnetometer # all available magnetometer drivers
		magnetometer/hmc5883
		#mkblctrl
		optical_flow/px4flow
		osd
		#pca9685
		#pwm_input
		#pwm_out_sim
		pwm_out
		rc_input
		#tap_esc
		#telemetry # all available telemetry drivers
		telemetry/frsky_telemetry
		#test_ppm
	MODULES
		#airspeed_selector
		attitude_estimator_q
		battery_status
		#camera_feedback
		commander
		dataman
		ekf2
		events
		#fw_att_control
		#fw_pos_control_l1
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
		navigator
		rc_update
		#rover_pos_control
		sensors
		#sih
		#temperature_compensation
		#vmount
		#vtol_att_control
	SYSTEMCMDS
		#bl_update
		dmesg
		dumpfile
		esc_calib
		hardfault_log
		i2cdetect
		led_control
		mixer
		#motor_ramp
		motor_test
		#mtd
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		sd_bench
		#tests # tests and test runner
		top
		#topic_listener
		tune_control
		usb_connected
		ver
		work_queue
	EXAMPLES
		#fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		#hello
		#hwtest # Hardware test
		#matlab_csv_serial
		#px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		#px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		#rover_steering_control # Rover example app
		#uuv_example_app
		#work_item
	)
