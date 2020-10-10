px4_add_module(
	MODULE modules__internal_resistance
	MAIN internal_resistance
	COMPILE_FLAGS
	SRCS
		internal_resistance.cpp
	DEPENDS
		modules__uORB
		px4_work_queue
	)
