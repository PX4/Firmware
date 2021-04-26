include (${CMAKE_CURRENT_LIST_DIR}/uavcan_board_identity)

px4_add_board(
	PLATFORM nuttx
	VENDOR ark
	MODEL can-flow
	LABEL debug
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	ROMFSROOT cannode
	UAVCAN_INTERFACES 1
	DRIVERS
		bootloaders
		imu/bosch/bmi088
		optical_flow/paw3902
		uavcannode
	MODULES
		#ekf2
		load_mon
		#sensors
	SYSTEMCMDS
		mft
		mtd
		param
		perf
		reboot
		system_time
		top
		topic_listener
		ver
		work_queue
)
