

# UAVCAN boot loadable Module ID
set(uavcanblid_sw_version_major 0)
set(uavcanblid_sw_version_minor 1)
add_definitions(
	-DAPP_VERSION_MAJOR=${uavcanblid_sw_version_major}
	-DAPP_VERSION_MINOR=${uavcanblid_sw_version_minor}
)

set(uavcanblid_hw_version_major 1)
set(uavcanblid_hw_version_minor 0)
set(uavcanblid_name "\"org.nxp.rddrone-uavcan146\"")

add_definitions(
	-DHW_UAVCAN_NAME=${uavcanblid_name}
	-DHW_VERSION_MAJOR=${uavcanblid_hw_version_major}
	-DHW_VERSION_MINOR=${uavcanblid_hw_version_minor}
)

px4_add_board(
	PLATFORM nuttx
	VENDOR nxp
	MODEL rddrone-uavcan146
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT cannode
	UAVCAN_INTERFACES 2
	DRIVERS
		#adc
		#barometer # all available barometer drivers
		#bootloaders
		#differential_pressure # all available differential pressure drivers
		#distance_sensor # all available distance sensor drivers
		#dshot
		gps
		#imu # all available imu drivers
		#lights
		#magnetometer # all available magnetometer drivers
		#optical_flow # all available optical flow drivers
		#pwm_out
		#safety_button
		#tone_alarm
		#uavcannode # TODO: CAN driver needed
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
		#led_control
		#mixer
		#motor_ramp
		#motor_test
		#nshterm
		#param
		#perf
		#pwm
		reboot
		#reflect
		#sd_bench
		top
		#topic_listener
		#tune_control
		ver
		#work_queue
)
