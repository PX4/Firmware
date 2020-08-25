
px4_add_board(
	PLATFORM posix
	VENDOR px4
	MODEL sitl
	LABEL replay
	MODULES
		ekf2
		logger
		replay
	SYSTEMCMDS
		param
		perf
		reboot
		shutdown
		topic_listener
		ver
		work_queue
	)

message(STATUS "Building with uorb publisher rules support")
add_definitions(-DORB_USE_PUBLISHER_RULES)

message(STATUS "Building without lockstep for replay")
set(ENABLE_LOCKSTEP_SCHEDULER no)
