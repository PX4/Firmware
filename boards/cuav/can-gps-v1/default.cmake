include (${CMAKE_CURRENT_LIST_DIR}/uavcan_board_identity)

add_definitions(-DUSE_S_RGB_LED_DMA)

px4_add_board(
	PLATFORM nuttx
	VENDOR cuav
	MODEL can-gps-v1
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	ROMFSROOT cannode
	UAVCAN_INTERFACES 1
	DRIVERS
		barometer/ms5611
		bootloaders
		gps
		lights/neopixel
		magnetometer/rm3100
		safety_button
		tone_alarm
		uavcannode
	MODULES
		#load_mon
	SYSTEMCMDS
		#i2cdetect
		#led_control
		param
		#perf
		#reboot
		#top
		#topic_listener
		#tune_control
		#uorb
		#ver
		#work_queue
)
