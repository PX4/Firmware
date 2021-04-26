include (${CMAKE_CURRENT_LIST_DIR}/uavcan_board_identity)

px4_add_board(
	PLATFORM nuttx
	VENDOR nxp
	MODEL ucans32k146
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	ROMFSROOT cannode
	UAVCAN_INTERFACES 2
	SERIAL_PORTS
		GPS1:/dev/ttyS1
	DRIVERS
		#adc/board_adc
		#barometer # all available barometer drivers
		bootloaders
		#differential_pressure # all available differential pressure drivers
		#distance_sensor # all available distance sensor drivers
		#dshot
		gps
		#imu # all available imu drivers
		#lights
		lights/rgbled_pwm
		#magnetometer # all available magnetometer drivers
		#optical_flow # all available optical flow drivers
		pwm_out
		#safety_button
		#tone_alarm
		#uavcannode # TODO: CAN driver needed
		#uavcan_v1
		uavcannode_gps_demo
	MODULES
		#ekf2
		#load_mon
		#sensors
		#temperature_compensation
	SYSTEMCMDS
		#bl_update
		#dmesg
		#dumpfile
		#esc_calib
		#hardfault_log
		i2cdetect
		led_control
		mixer
		mtd
		mft
		#motor_ramp
		#motor_test
		#nshterm
		param
		#perf
		pwm
		reboot
		#reflect
		#sd_bench
		system_time
		top
		topic_listener
		#tune_control
		ver
		work_queue
)
