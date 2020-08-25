############################################################################
#
#   Copyright (c) 2018 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

message(STATUS "ROMFS: ${config_romfs_root}")

set(romfs_src_dir ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root})
set(romfs_gen_root_dir ${PX4_BINARY_DIR}/genromfs)

set_property(GLOBAL PROPERTY PX4_ROMFS_FILES)
set_property(GLOBAL PROPERTY PX4_ROMFS_CMAKE_FILES)

#=============================================================================
#
#	px4_add_romfs_files
#
#	This function builds a list of files to be included in the ROMFS.
#
#	Usage:
#		px4_add_romfs_files(<list of files in current directory to be added to the project>)
#
#	Output:
#		list config_romfs_files_list populated with a list of ROMFS src files including their full file path.
#
#	Example:
#		px4_add_module(
#			<ROMFS_file_1>
#			<ROMFS_file_2>
#			<ROMFS_file_3>
#			)
#
function(px4_add_romfs_files)
	set_property(GLOBAL APPEND PROPERTY PX4_ROMFS_CMAKE_FILES ${CMAKE_CURRENT_LIST_FILE})

	foreach(arg IN LISTS ARGN)
		if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/${arg})
			message(FATAL_ERROR "${CMAKE_CURRENT_LIST_DIR}/${arg} doesn't exist")
		endif()
		set_property(GLOBAL APPEND PROPERTY PX4_ROMFS_FILES ${CMAKE_CURRENT_LIST_DIR}/${arg})
	endforeach()
endfunction()

# get list of all ROMFS files
add_subdirectory(${romfs_src_dir})

# directory setup
# copy all romfs files, process airframes
get_property(romfs_cmake_files GLOBAL PROPERTY PX4_ROMFS_CMAKE_FILES)
get_property(romfs_copy_files GLOBAL PROPERTY PX4_ROMFS_FILES)
get_property(module_config_files GLOBAL PROPERTY PX4_MODULE_CONFIG_FILES)
file(GLOB jinja_templates ${PX4_SOURCE_DIR}/Tools/serial/*.jinja)
if (px4_constrained_flash_build)
	set(added_arguments --constrained-flash)
endif()
add_custom_command(
	OUTPUT
		${romfs_gen_root_dir}/init.d/rcS
		${romfs_gen_root_dir}/init.d/rc.serial
		${romfs_gen_root_dir}/init.d/rc.autostart
		${romfs_gen_root_dir}/init.d/rc.autostart.post
		romfs_copy.stamp
	COMMAND ${CMAKE_COMMAND} -E remove_directory ${romfs_gen_root_dir}
	# TODO: we should only copy the files in ${romfs_copy_files}
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${romfs_src_dir} ${romfs_gen_root_dir}
	COMMAND ${CMAKE_COMMAND} -E remove_directory ${romfs_gen_root_dir}/init.d-posix
	COMMAND ${CMAKE_COMMAND} -E remove_directory ${romfs_gen_root_dir}/mixers-sitl
	COMMAND ${CMAKE_COMMAND} -E remove ${romfs_gen_root_dir}/mixers/CMakeLists.txt
	COMMAND ${CMAKE_COMMAND} -E remove ${romfs_gen_root_dir}/init.d/CMakeLists.txt
	COMMAND ${CMAKE_COMMAND} -E remove ${romfs_gen_root_dir}/init.d/airframes/CMakeLists.txt
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
		--airframes-path ${romfs_gen_root_dir}/init.d
		--start-script ${romfs_gen_root_dir}/init.d/rc.autostart
		--board ${PX4_BOARD}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/serial/generate_config.py
		--rc-dir ${romfs_gen_root_dir}/init.d
		--serial-ports ${board_serial_ports} ${added_arguments}
		--config-files ${module_config_files} #--verbose
	COMMAND ${CMAKE_COMMAND} -E touch romfs_copy.stamp
	DEPENDS
		${jinja_templates}
		${module_config_files}
		${romfs_cmake_files}
		${romfs_copy_files}
		${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
		${PX4_SOURCE_DIR}/Tools/px4airframes/markdownout.py
		${PX4_SOURCE_DIR}/Tools/px4airframes/rcout.py
		${PX4_SOURCE_DIR}/Tools/px4airframes/srcparser.py
		${PX4_SOURCE_DIR}/Tools/px4airframes/srcscanner.py
		${PX4_SOURCE_DIR}/Tools/px4airframes/xmlout.py
		${PX4_SOURCE_DIR}/Tools/serial/generate_config.py
	COMMENT "ROMFS: copying, generating airframes"
	)

# copy extras into ROMFS
set(extras_dependencies)

if(config_bl_file)
	file(MAKE_DIRECTORY ${PX4_BINARY_DIR}/romfs_extras)
	configure_file(${config_bl_file} ${PX4_BINARY_DIR}/romfs_extras COPYONLY)

	list(APPEND extras_dependencies
		${config_bl_file}
		)
endif()

# copy px4io binary if configured
if(config_io_board)
	list(APPEND extras_dependencies
		copy_px4io_bin
		${fw_io_bin}
	)
endif()

if(config_build_bootloader)
	list(APPEND extras_dependencies
		copy_bootloader_bin
		${bootloader_bin}
	)
endif()

set(OPTIONAL_BOARD_RC)
list(APPEND OPTIONAL_BOARD_RC
	rc.board_defaults
	rc.board_sensors
	rc.board_extras
	rc.board_mavlink
)

foreach(board_rc_file ${OPTIONAL_BOARD_RC})

	if(EXISTS "${PX4_BOARD_DIR}/init/${board_rc_file}")
		message(STATUS "ROMFS: Adding ${board_rc_file}")

		add_custom_command(
			OUTPUT
				${romfs_gen_root_dir}/init.d/${board_rc_file}
				${board_rc_file}.stamp
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PX4_BOARD_DIR}/init/${board_rc_file} ${romfs_gen_root_dir}/init.d/${board_rc_file}
			COMMAND ${CMAKE_COMMAND} -E touch ${board_rc_file}.stamp
			DEPENDS
				${PX4_BOARD_DIR}/init/${board_rc_file}
				romfs_copy.stamp
			COMMENT "ROMFS: copying ${board_rc_file}"
		)

		list(APPEND extras_dependencies
			${board_rc_file}.stamp
			)
	endif()

endforeach()

add_custom_command(OUTPUT romfs_extras.stamp
	COMMAND ${CMAKE_COMMAND} -E make_directory ${romfs_gen_root_dir}/extras/
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/romfs_extras/
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${PX4_BINARY_DIR}/romfs_extras/ ${romfs_gen_root_dir}/extras/
	COMMAND ${CMAKE_COMMAND} -E touch romfs_extras.stamp
	DEPENDS
		romfs_copy.stamp
		${extras_dependencies}
	COMMENT "ROMFS: copying extras"
	)

add_custom_command(
	OUTPUT romfs_pruned.stamp
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_romfs_pruner.py --folder ${romfs_gen_root_dir} --board ${PX4_BOARD}
	COMMAND ${CMAKE_COMMAND} -E touch romfs_pruned.stamp
	DEPENDS
		romfs_copy.stamp
		romfs_extras.stamp
		${PX4_SOURCE_DIR}/Tools/px_romfs_pruner.py
	COMMENT "ROMFS: pruning"
	)

if("${CONFIG_FS_CROMFS}" STREQUAL "y")
	add_custom_command(
		OUTPUT ${CMAKE_BINARY_DIR}/gencromfs
		COMMAND make --no-print-directory --silent -f Makefile.host gencromfs
		COMMAND ${CMAKE_COMMAND} -E copy_if_different gencromfs ${CMAKE_BINARY_DIR}/gencromfs
		DEPENDS ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/nuttx/tools/gencromfs.c
		WORKING_DIRECTORY ${NUTTX_DIR}/tools
	)

	# create nsh_romfsimg.c
	add_custom_command(OUTPUT nsh_romfsimg.c
		COMMAND ${CMAKE_COMMAND} -E remove -f nsh_romfsimg.c
		COMMAND ${CMAKE_BINARY_DIR}/gencromfs ${romfs_gen_root_dir} nsh_romfsimg.c
		DEPENDS
			${CMAKE_BINARY_DIR}/gencromfs
			romfs_pruned.stamp
		COMMENT "ROMFS: generating image"
	)

else()
	# create romfs.img
	find_program(GENROMFS genromfs)
	if(NOT GENROMFS)
		message(FATAL_ERROR "genromfs not found")
	endif()

	add_custom_command(
		OUTPUT
			romfs.img
			romfs.txt
		COMMAND ${CMAKE_COMMAND} -E remove -f romfs.img romfs.txt
		COMMAND ${GENROMFS} -f romfs.img -d ${romfs_gen_root_dir} -V "NSHInitVol" -v > romfs.txt 2>&1
		DEPENDS romfs_pruned.stamp
		COMMENT "ROMFS: generating image"
	)


	# create nsh_romfsimg.c
	find_program(XXD xxd)
	if(NOT XXD)
		message(FATAL_ERROR "xxd not found")
	endif()

	find_program(SED sed)
	if(NOT SED)
		message(FATAL_ERROR "sed not found")
	endif()

	add_custom_command(OUTPUT nsh_romfsimg.c
		COMMAND ${CMAKE_COMMAND} -E remove -f nsh_romfsimg.c
		COMMAND ${XXD} -i romfs.img nsh_romfsimg.c
		COMMAND ${SED} 's/unsigned/const unsigned/g' nsh_romfsimg.c > nsh_romfsimg.c.tmp && ${CMAKE_COMMAND} -E rename nsh_romfsimg.c.tmp nsh_romfsimg.c
		DEPENDS romfs.img
	)
endif()


add_library(romfs STATIC nsh_romfsimg.c)
add_dependencies(romfs prebuild_targets)
set_target_properties(romfs PROPERTIES LINKER_LANGUAGE C)


# shellcheck
find_program(SHELLCHECK_PATH shellcheck)

if(SHELLCHECK_PATH)

	# TODO: fix SC2086, SC2166
	add_custom_target(shellcheck
		COMMAND ${SHELLCHECK_PATH} --shell=sh
			--exclude=SC2121 # SC2121: To assign a variable, use just 'var=value'
			--exclude=SC2086 # SC2086: Double quote to prevent globbing and word splitting.
			--exclude=SC2166 # SC2166: Prefer [ p ] || [ q ] as [ p -o q ] is not well defined.
			--exclude=SC2154 # SC2154: <var> is referenced but not assigned (NuttX uses different asssignment)
			--exclude=SC2164 # SC2164: Use 'cd ... || exit' or 'cd ... || return' in case cd fails.
			--exclude=SC2181 # SC2181: Check exit code directly with e.g. 'if mycmd;', not indirectly with $?.
			`find ${romfs_gen_root_dir}/init.d -type f`
		DEPENDS ${romfs_gen_root_dir}/init.d/rc.autostart
		WORKING_DIRECTORY ${romfs_gen_root_dir}
		USES_TERMINAL
	)
endif()
