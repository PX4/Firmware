
px4_add_board(
	PLATFORM posix
	VENDOR px4
	MODEL sitl
	ROMFSROOT px4fmu_common
	LABEL replay
	MODULES
		ekf2
		logger
		replay
	SYSTEMCMDS
		param
		perf
		shutdown
		topic_listener
		ver
		work_queue
	)

message(STATUS "Building with uorb publisher rules support")
add_definitions(-DORB_USE_PUBLISHER_RULES)

set(ENABLE_LOCKSTEP_SCHEDULER yes)
