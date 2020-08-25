set(SITL_WORKING_DIR ${PX4_BINARY_DIR}/tmp)
file(MAKE_DIRECTORY ${SITL_WORKING_DIR})
file(MAKE_DIRECTORY ${SITL_WORKING_DIR}/rootfs)

# add a symlink to the logs dir to make it easier to find them
add_custom_command(OUTPUT ${PX4_BINARY_DIR}/logs
		COMMAND ${CMAKE_COMMAND} -E create_symlink ${SITL_WORKING_DIR}/rootfs/log logs
		WORKING_DIRECTORY ${PX4_BINARY_DIR})
add_custom_target(logs_symlink DEPENDS ${PX4_BINARY_DIR}/logs)

add_dependencies(px4 logs_symlink)

add_custom_target(run_config
		COMMAND Tools/sitl_run.sh
			$<TARGET_FILE:px4>
			${config_sitl_debugger}
			${config_sitl_viewer}
			${config_sitl_model}
			${PX4_SOURCE_DIR}
			${PX4_BINARY_DIR}
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
		USES_TERMINAL
		DEPENDS px4 logs_symlink
		)

px4_add_git_submodule(TARGET git_gazebo PATH "${PX4_SOURCE_DIR}/Tools/sitl_gazebo")
px4_add_git_submodule(TARGET git_jmavsim PATH "${PX4_SOURCE_DIR}/Tools/jMAVSim")

# Add support for external project building
include(ExternalProject)

# project to build sitl_gazebo if necessary
ExternalProject_Add(sitl_gazebo
	SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/sitl_gazebo
	CMAKE_ARGS
		-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
		-DSEND_VISION_ESTIMATION_DATA=ON
	BINARY_DIR ${PX4_BINARY_DIR}/build_gazebo
	INSTALL_COMMAND ""
	DEPENDS
		git_gazebo
	USES_TERMINAL_CONFIGURE true
	USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	BUILD_ALWAYS 1
)

ExternalProject_Add(mavsdk_tests
	SOURCE_DIR ${PX4_SOURCE_DIR}/test/mavsdk_tests
	CMAKE_ARGS
		-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
	BINARY_DIR ${PX4_BINARY_DIR}/mavsdk_tests
	INSTALL_COMMAND ""
	USES_TERMINAL_CONFIGURE true
	USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	BUILD_ALWAYS 1
)

# create targets for each viewer/model/debugger combination
set(viewers none jmavsim gazebo)
set(debuggers none ide gdb lldb ddd valgrind callgrind)
set(models none shell
	if750a iris iris_opt_flow iris_opt_flow_mockup iris_vision iris_rplidar iris_irlock iris_obs_avoid iris_rtps solo typhoon_h480
	plane plane_cam plane_catapult
	standard_vtol tailsitter tiltrotor
	rover boat
	uuv_hippocampus)
set(all_posix_vmd_make_targets)
foreach(viewer ${viewers})
	foreach(debugger ${debuggers})
		foreach(model ${models})
			if (debugger STREQUAL "none")
				if (model STREQUAL "none")
					set(_targ_name "${viewer}")
				else()
					set(_targ_name "${viewer}_${model}")
				endif()
			else()
				if (model STREQUAL "none")
					set(_targ_name "${viewer}___${debugger}")
				else()
					set(_targ_name "${viewer}_${model}_${debugger}")
				endif()
			endif()

			add_custom_target(${_targ_name}
					COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh
						$<TARGET_FILE:px4>
						${debugger}
						${viewer}
						${model}
						${PX4_SOURCE_DIR}
						${PX4_BINARY_DIR}
					WORKING_DIRECTORY ${SITL_WORKING_DIR}
					USES_TERMINAL
					DEPENDS
						logs_symlink
					)
			list(APPEND all_posix_vmd_make_targets ${_targ_name})
			if (viewer STREQUAL "gazebo")
				add_dependencies(${_targ_name} px4 sitl_gazebo)
			elseif(viewer STREQUAL "jmavsim")
				add_dependencies(${_targ_name} px4 git_jmavsim)
			endif()
		endforeach()
	endforeach()
endforeach()

string(REPLACE ";" "," posix_vmd_make_target_list "${all_posix_vmd_make_targets}")

add_custom_target(list_vmd_make_targets
	COMMAND sh -c "printf \"${posix_vmd_make_target_list}\\n\""
	COMMENT "List of acceptable '${PX4_BOARD}' <viewer_model_debugger> targets:"
	VERBATIM
	)

# vscode launch.json
if(${PX4_BOARD_LABEL} MATCHES "replay")
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/launch_replay.json.in ${PX4_SOURCE_DIR}/.vscode/launch.json COPYONLY)
else()
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/launch_sim.json.in ${PX4_SOURCE_DIR}/.vscode/launch.json COPYONLY)
endif()
