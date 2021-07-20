# TODO: Apply to UAVCAN v1 node
# UAVCAN boot loadable Module ID
# set(uavcanblid_sw_version_major 0)
# set(uavcanblid_sw_version_minor 1)
# add_definitions(
# 	-DAPP_VERSION_MAJOR=${uavcanblid_sw_version_major}
# 	-DAPP_VERSION_MINOR=${uavcanblid_sw_version_minor}
# )

# set(uavcanblid_hw_version_major 1)
# set(uavcanblid_hw_version_minor 0)
# set(uavcanblid_name "\"org.px4.fmu-v4_cannodev1\"")

# add_definitions(
# 	-DHW_UAVCAN_NAME=${uavcanblid_name}
# 	-DHW_VERSION_MAJOR=${uavcanblid_hw_version_major}
# 	-DHW_VERSION_MINOR=${uavcanblid_hw_version_minor}
# )

px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v4
	LABEL v1node
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	NUTTX_CONFIG uavcanv1
	ROMFSROOT cannodev1
	TESTING
	UAVCAN_INTERFACES 1
	SERIAL_PORTS
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		WIFI:/dev/ttyS0

	DRIVERS
		adc/board_adc
		adc/ads1115
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
		imu/analog_devices/adis16448
		imu/adis16477
		imu/adis16497
		imu/invensense/icm20602
		imu/invensense/icm20608g
		imu/invensense/icm40609d
		imu/invensense/mpu6500
		imu/invensense/mpu9250
		irlock
		lights/rgbled
		lights/rgbled_ncp5623c
		magnetometer # all available magnetometer drivers
		optical_flow # all available optical flow drivers
		#osd
		pca9685
		pca9685_pwm_out
		#protocol_splitter
		pwm_input
		#pwm_out_sim
		pwm_out
		#rc_input
		#roboclaw
		safety_button
		telemetry # all available telemetry drivers
		#tone_alarm
		uavcan_v1
	MODULES
		# airspeed_selector
		attitude_estimator_q
		battery_status
		camera_feedback
		commander
		dataman
		ekf2
		esc_battery
		events
		# flight_mode_manager
		# fw_att_control
		# fw_pos_control_l1
		# land_detector
		# landing_target_estimator
		load_mon
		# local_position_estimator
		logger
		mavlink
		# mc_att_control
		# mc_hover_thrust_estimator
		mc_pos_control
		# mc_rate_control
		#micrortps_bridge
		navigator
		rc_update
		# rover_pos_control
		sensors
		sih
		temperature_compensation
		# uuv_att_control
		# uuv_pos_control
		vmount
		# vtol_att_control
	SYSTEMCMDS
		bl_update
		#dmesg
		gpio
		hardfault_log
		i2cdetect
		led_control
		mft
		mixer
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		system_time
		tests # tests and test runner
		top
		topic_listener
		tune_control
		uorb
		usb_connected
		ver
		work_queue
	)
