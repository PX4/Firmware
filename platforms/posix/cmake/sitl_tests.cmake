#=============================================================================
# tests
#

# TODO: find a way to keep this in sync with tests_main
set(tests
	atomic_bitset
	bezier
	bitset
	bson
	commander
	controllib
	conv
	dataman
	file2
	float
	hrt
	int
	IntrusiveQueue
	IntrusiveSortedList
	List
	mathlib
	matrix
	microbench_atomic
	microbench_hrt
	microbench_math
	microbench_matrix
	microbench_uorb
	mixer
	param
	parameters
	perf
	rc
	search_min
	servo
	#sf0x
	sleep
	uorb
	versioning
	)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	list(REMOVE_ITEM tests
		mixer
		uorb
	)
endif()

if (CMAKE_SYSTEM_NAME STREQUAL "CYGWIN")
	list(REMOVE_ITEM tests
		uorb
	)
endif()

foreach(test_name ${tests})
	set(test_name_prefix sitl-${test_name})
	configure_file(${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/test_template.in ${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/test_${test_name}_generated)

	add_test(NAME ${test_name_prefix}
		COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh
			$<TARGET_FILE:px4>
			none
			none
			test_${test_name}_generated
			none
			${PX4_SOURCE_DIR}
			${PX4_BINARY_DIR}
		WORKING_DIRECTORY ${SITL_WORKING_DIR})

	set_tests_properties(${test_name_prefix} PROPERTIES FAIL_REGULAR_EXPRESSION "${test_name} FAILED")
	set_tests_properties(${test_name_prefix} PROPERTIES PASS_REGULAR_EXPRESSION "${test_name} PASSED")

	sanitizer_fail_test_on_error(${test_name_prefix})
endforeach()


# Mavlink test requires mavlink running
add_test(NAME mavlink
	COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh
		$<TARGET_FILE:px4>
		none
		none
		test_mavlink
		none
		${PX4_SOURCE_DIR}
		${PX4_BINARY_DIR}
	WORKING_DIRECTORY ${SITL_WORKING_DIR})

set_tests_properties(mavlink PROPERTIES FAIL_REGULAR_EXPRESSION "mavlink FAILED")
set_tests_properties(mavlink PROPERTIES PASS_REGULAR_EXPRESSION "mavlink PASSED")
sanitizer_fail_test_on_error(mavlink)

# A mystery why this fails on Cygwin currently.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "CYGWIN")
	# Shutdown test
	add_test(NAME shutdown
		COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh
			$<TARGET_FILE:px4>
			none
			none
			test_shutdown
			none
			${PX4_SOURCE_DIR}
			${PX4_BINARY_DIR}
		WORKING_DIRECTORY ${SITL_WORKING_DIR})

	#set_tests_properties(shutdown PROPERTIES FAIL_REGULAR_EXPRESSION "shutdown FAILED")
	set_tests_properties(shutdown PROPERTIES PASS_REGULAR_EXPRESSION "Exiting NOW.")
	sanitizer_fail_test_on_error(shutdown)
endif()

# Dynamic module loading test
add_test(NAME dyn
	COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh
		$<TARGET_FILE:px4>
		none
		none
		test_dyn_hello
		none
		${PX4_SOURCE_DIR}
		${PX4_BINARY_DIR}
		$<TARGET_FILE:examples__dyn_hello>
	WORKING_DIRECTORY ${SITL_WORKING_DIR})
set_tests_properties(dyn PROPERTIES PASS_REGULAR_EXPRESSION "1: PASSED")
sanitizer_fail_test_on_error(dyn)

# run arbitrary commands
set(test_cmds
	hrt_test
	cdev_test
	wqueue_test
	)

foreach(cmd_name ${test_cmds})
	configure_file(${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/cmd_template.in ${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/cmd_${cmd_name}_generated)

	add_test(NAME posix_${cmd_name}
		COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh
			$<TARGET_FILE:px4>
			none
			none
			cmd_${cmd_name}_generated
			none
			${PX4_SOURCE_DIR}
			${PX4_BINARY_DIR}
		WORKING_DIRECTORY ${SITL_WORKING_DIR})

	sanitizer_fail_test_on_error(posix_${cmd_name})
	set_tests_properties(posix_${cmd_name} PROPERTIES PASS_REGULAR_EXPRESSION "Exiting NOW.")
endforeach()

if(CMAKE_BUILD_TYPE STREQUAL Coverage)
	setup_target_for_coverage(test_coverage "${CMAKE_CTEST_COMMAND} --output-on-failure -T Test" tests)
	setup_target_for_coverage(generate_coverage "${CMAKE_COMMAND} -E echo" generic)

	# TODO:
	#setup_target_for_coverage(mavsdk_coverage "${PX4_SOURCE_DIR}/test/mavsdk_tests/mavsdk_test_runner.py --speed-factor 20 --iterations 1 --fail-early" mavsdk)
endif()
