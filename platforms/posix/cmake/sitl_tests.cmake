#=============================================================================
# tests
#

# tests command arguments
set(tests
	atomic_bitset
	bezier
	bitset
	bson
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
	search_min
	servo
	sleep
	versioning
)

foreach(test_name ${tests})
	set(test_name_prefix sitl-${test_name})
	configure_file(${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/test_template.in ${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/test_${test_name}_generated)

	add_test(NAME ${test_name_prefix}
		COMMAND $<TARGET_FILE:px4>
			-s ${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/test_${test_name}_generated
			-t ${PX4_SOURCE_DIR}/test_data
			${PX4_SOURCE_DIR}/ROMFS/px4fmu_common
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
	)

	set_tests_properties(${test_name_prefix} PROPERTIES FAIL_REGULAR_EXPRESSION "${test_name} FAILED")
	set_tests_properties(${test_name_prefix} PROPERTIES PASS_REGULAR_EXPRESSION "${test_name} PASSED")

	sanitizer_fail_test_on_error(${test_name_prefix})
endforeach()


# standalone tests
set(cmd_tests
	commander_tests
	controllib_test
	lightware_laser_test
	rc_tests
	uorb_tests
)

foreach(test_name ${cmd_tests})
	set(test_name_prefix sitl-${test_name})
	configure_file(${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/test_cmd_template.in ${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/test_${test_name}_generated)

	add_test(NAME ${test_name_prefix}
		COMMAND $<TARGET_FILE:px4>
			-s ${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/test_${test_name}_generated
			-t ${PX4_SOURCE_DIR}/test_data
			${PX4_SOURCE_DIR}/ROMFS/px4fmu_common
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
	)

	set_tests_properties(${test_name_prefix} PROPERTIES FAIL_REGULAR_EXPRESSION "FAIL")
	set_tests_properties(${test_name_prefix} PROPERTIES PASS_REGULAR_EXPRESSION "PASS")

	sanitizer_fail_test_on_error(${test_name_prefix})
endforeach()



# Mavlink test requires mavlink running
add_test(NAME sitl-mavlink
	COMMAND $<TARGET_FILE:px4>
		-s ${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/test_mavlink
		-t ${PX4_SOURCE_DIR}/test_data
		${PX4_SOURCE_DIR}/ROMFS/px4fmu_common
	WORKING_DIRECTORY ${SITL_WORKING_DIR}
)

set_tests_properties(sitl-mavlink PROPERTIES FAIL_REGULAR_EXPRESSION "FAIL")
set_tests_properties(sitl-mavlink PROPERTIES PASS_REGULAR_EXPRESSION "ALL TESTS PASSED")
sanitizer_fail_test_on_error(sitl-mavlink)



# # Shutdown test
# add_test(NAME sitl-shutdown
# 	COMMAND $<TARGET_FILE:px4>
# 		-s ${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/test_shutdown
# 		-t ${PX4_SOURCE_DIR}/test_data
# 		${PX4_SOURCE_DIR}/ROMFS/px4fmu_common
# 	WORKING_DIRECTORY ${SITL_WORKING_DIR}
# )

# #set_tests_properties(shutdown PROPERTIES FAIL_REGULAR_EXPRESSION "shutdown FAILED")
# set_tests_properties(sitl-shutdown PROPERTIES PASS_REGULAR_EXPRESSION "Exiting NOW.")
# sanitizer_fail_test_on_error(sitl-shutdown)



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
		COMMAND $<TARGET_FILE:px4>
			${PX4_SOURCE_DIR}/ROMFS/px4fmu_common
			-s ${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/cmd_${cmd_name}_generated
			-t ${PX4_SOURCE_DIR}/test_data
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
	)

	sanitizer_fail_test_on_error(posix_${cmd_name})
	set_tests_properties(posix_${cmd_name} PROPERTIES PASS_REGULAR_EXPRESSION "Exiting NOW.")
endforeach()


if(CMAKE_BUILD_TYPE STREQUAL Coverage)
	setup_target_for_coverage(test_coverage "${CMAKE_CTEST_COMMAND} --output-on-failure -T Test" tests)
	setup_target_for_coverage(generate_coverage "${CMAKE_COMMAND} -E echo" generic)

	# TODO:
	#setup_target_for_coverage(mavsdk_coverage "${PX4_SOURCE_DIR}/test/mavsdk_tests/mavsdk_test_runner.py --speed-factor 20 --iterations 1 --fail-early" mavsdk)
endif()
