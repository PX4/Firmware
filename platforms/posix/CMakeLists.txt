
include_directories(${CMAKE_CURRENT_BINARY_DIR})

get_property(module_libraries GLOBAL PROPERTY PX4_MODULE_LIBRARIES)

# When building with catkin, do not change CMAKE_RUNTIME_OUTPUT_DIR from
# CMAKE_CURRENT_BINARY_DIR ''./platforms/posix' to './bin'
if (NOT CATKIN_DEVEL_PREFIX)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

set(PX4_SHELL_COMMAND_PREFIX "px4-")

add_definitions("-DPX4_SHELL_COMMAND_PREFIX=\"${PX4_SHELL_COMMAND_PREFIX}\"")

px4_posix_generate_builtin_commands(
	OUT apps
	MODULE_LIST ${module_libraries})

px4_posix_generate_alias(
	OUT ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/px4-alias.sh
	MODULE_LIST ${module_libraries}
	PREFIX ${PX4_SHELL_COMMAND_PREFIX}
)

if (("${PX4_BOARD}" MATCHES "atlflight_eagle") OR ("${PX4_BOARD}" MATCHES "atlflight_excelsior"))
	include(fastrpc)
	include(linux_app)

	FASTRPC_STUB_GEN(../qurt/px4muorb.idl)

	LINUX_APP(
		APP_NAME px4
		IDL_NAME px4muorb
		APPS_DEST "/home/linaro"
		SOURCES
			px4muorb_stub.c
			src/px4/common/main.cpp
			apps.cpp
		LINK_LIBS
			-Wl,--start-group
				px4_layer
				parameters
				${module_libraries}
				${FASTRPC_ARM_LIBS}
				pthread m rt
			-Wl,--end-group
		)

else()
	add_executable(px4
		src/px4/common/main.cpp
		apps.cpp
		)

	target_link_libraries(px4
		PRIVATE
			${module_libraries}
			pthread m

			# horrible circular dependencies that need to be teased apart
			px4_layer px4_platform
			work_queue
			parameters
	)

	if(NOT APPLE)
		target_link_libraries(px4 PRIVATE rt)
	endif()

	target_link_libraries(px4 PRIVATE modules__uORB)

	#=============================================================================
	# install
	#

	# TODO: extend to snapdragon

	# px4 dirs
	install(
		DIRECTORY
			${PROJECT_SOURCE_DIR}/posix-configs
			${PROJECT_SOURCE_DIR}/ROMFS
			${PROJECT_SOURCE_DIR}/test
			${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
		DESTINATION
			${PROJECT_NAME}
		USE_SOURCE_PERMISSIONS
		)

endif()

# Module Symlinks
px4_posix_generate_symlinks(
	MODULE_LIST ${module_libraries}
	PREFIX ${PX4_SHELL_COMMAND_PREFIX}
	TARGET px4
)


# board defined upload helper
if(EXISTS "${PX4_BOARD_DIR}/cmake/upload.cmake")
	include(${PX4_BOARD_DIR}/cmake/upload.cmake)
endif()


if("${PX4_BOARD}" MATCHES "beaglebone_blue")
	target_link_libraries(px4 PRIVATE robotics_cape)

elseif("${PX4_BOARD}" MATCHES "emlid_navio2")
	target_link_libraries(px4 PRIVATE atomic)

elseif("${PX4_BOARD}" MATCHES "sitl")

	include(sitl_target)
	if(BUILD_TESTING)
		include(sitl_tests)
	endif()

	# install

	# px4 dirs
	install(
		DIRECTORY
			${PROJECT_SOURCE_DIR}/integrationtests
			${PROJECT_SOURCE_DIR}/launch
			${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
		DESTINATION
			${PROJECT_NAME}
		USE_SOURCE_PERMISSIONS
		)

	# px4 files
	install(
		FILES
			${PROJECT_SOURCE_DIR}/CMakeLists.txt
			${PROJECT_SOURCE_DIR}/package.xml
		DESTINATION
			${PROJECT_NAME}
		)

	# px4 Tools dirs
	install(
		DIRECTORY
			${PROJECT_SOURCE_DIR}/Tools/ecl_ekf
		DESTINATION
			${PROJECT_NAME}/Tools
		USE_SOURCE_PERMISSIONS
		)

	# px4 Tools files
	install(
		PROGRAMS
			${PROJECT_SOURCE_DIR}/Tools/setup_gazebo.bash
			${PROJECT_SOURCE_DIR}/Tools/upload_log.py
		DESTINATION
			${PROJECT_NAME}/Tools
		)

	# sitl_gazebo built plugins
	install(
		DIRECTORY
			${PROJECT_SOURCE_DIR}/build/px4_sitl_default/build_gazebo
		DESTINATION
			${PROJECT_NAME}/build/px4_sitl_default
		FILES_MATCHING
			PATTERN "CMakeFiles" EXCLUDE
			PATTERN "*.so"
		)

	# sitl_gazebo dirs
	install(
		DIRECTORY
			${PROJECT_SOURCE_DIR}/Tools/sitl_gazebo/models
			${PROJECT_SOURCE_DIR}/Tools/sitl_gazebo/worlds
		DESTINATION
			${PROJECT_NAME}/Tools/sitl_gazebo
		)

	# sitl_gazebo files
	install(
		FILES
			${PROJECT_SOURCE_DIR}/Tools/sitl_gazebo/CMakeLists.txt
			${PROJECT_SOURCE_DIR}/Tools/sitl_gazebo/package.xml
		DESTINATION
			${PROJECT_NAME}/Tools/sitl_gazebo
		)

endif()
