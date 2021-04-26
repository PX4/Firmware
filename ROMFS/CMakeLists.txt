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

set(romfs_src_dir ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root})
set(romfs_gen_root_dir ${PX4_BINARY_DIR}/etc)

file(RELATIVE_PATH romfs_path_relative ${PX4_SOURCE_DIR} ${romfs_src_dir})
message(STATUS "ROMFS: ${romfs_path_relative}")

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
if(PX4_ETHERNET)
	set(added_arguments ${added_arguments} --ethernet)
endif()
# create list of relative romfs file names
set(romfs_copy_files_relative)
foreach(romfs_file IN LISTS romfs_copy_files)
	string(REPLACE "${romfs_src_dir}/" "" romfs_file_rel ${romfs_file})
	list(APPEND romfs_copy_files_relative ${romfs_file_rel})
endforeach()
# copy the ROMFS files by creating a tar and extracting it to the build
# directory (which preserves the directory structure)
file(MAKE_DIRECTORY ${romfs_gen_root_dir})
set(romfs_tar_file ${PX4_BINARY_DIR}/romfs_files.tar)
add_custom_command(
	OUTPUT ${romfs_tar_file}
	COMMAND ${CMAKE_COMMAND} -E tar cf ${romfs_tar_file} ${romfs_copy_files_relative}
	WORKING_DIRECTORY ${romfs_src_dir}
	DEPENDS
		${romfs_copy_files}
		${jinja_templates}
		${module_config_files}
		${romfs_cmake_files}
		${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
		${PX4_SOURCE_DIR}/Tools/px4airframes/markdownout.py
		${PX4_SOURCE_DIR}/Tools/px4airframes/rcout.py
		${PX4_SOURCE_DIR}/Tools/px4airframes/srcparser.py
		${PX4_SOURCE_DIR}/Tools/px4airframes/srcscanner.py
		${PX4_SOURCE_DIR}/Tools/px4airframes/xmlout.py
		${PX4_SOURCE_DIR}/Tools/serial/generate_config.py
	)
set(romfs_extract_stamp ${CMAKE_CURRENT_BINARY_DIR}/romfs_extract.stamp)
add_custom_command(
	OUTPUT ${romfs_extract_stamp}
	COMMAND ${CMAKE_COMMAND} -E tar xf ${romfs_tar_file}
	COMMAND ${CMAKE_COMMAND} -E touch ${romfs_extract_stamp}
	WORKING_DIRECTORY ${romfs_gen_root_dir}
	DEPENDS ${romfs_tar_file}
	)

add_custom_command(
	OUTPUT
		${romfs_gen_root_dir}/init.d/rc.serial
		${romfs_gen_root_dir}/init.d/rc.autostart
		${romfs_gen_root_dir}/init.d/rc.autostart.post
		romfs_copy.stamp
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
		${romfs_extract_stamp}
	COMMENT "ROMFS: copying, generating airframes"
	)

# copy extras into ROMFS
set(extras_dependencies)

# copy px4io binary if configured
if(config_io_board)
	list(APPEND extras_dependencies
		copy_px4io_bin
		${fw_io_bin}
	)
	file(RELATIVE_PATH iofw_file_relative ${PX4_SOURCE_DIR} ${fw_io_bin})
	message(STATUS "ROMFS:  Adding ${iofw_file_relative} -> /etc/extras/${config_io_board}.bin")
endif()


# board bootloader (built or included)
if(TARGET copy_bootloader_bin)

	if(board_bootloader_firmware)
		file(RELATIVE_PATH bl_file_relative ${PX4_SOURCE_DIR} ${board_bootloader_firmware})
		message(STATUS "ROMFS:  Adding ${bl_file_relative} -> /etc/extras/bootloader.bin")
	else()
		file(RELATIVE_PATH bl_file_relative ${PX4_SOURCE_DIR} ${bootloader_bin})
		message(STATUS "ROMFS:  Adding ${bl_file_relative} -> /etc/extras/bootloader.bin")
	endif()

	list(APPEND extras_dependencies
		copy_bootloader_bin
		${bootloader_bin}
	)
endif()


# optional board architecture defaults
set(board_arch_rc_file "rc.board_arch_defaults")
if(EXISTS "${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/init/${CONFIG_ARCH_CHIP}/${board_arch_rc_file}")
	file(RELATIVE_PATH rc_file_relative ${PX4_SOURCE_DIR} ${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/init/${CONFIG_ARCH_CHIP}/${board_arch_rc_file})
	message(STATUS "ROMFS:  Adding ${rc_file_relative} -> /etc/init.d/${board_arch_rc_file}")

	add_custom_command(
		OUTPUT
			${romfs_gen_root_dir}/init.d/${board_arch_rc_file}
			${board_arch_rc_file}.stamp
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/init/${CONFIG_ARCH_CHIP}/${board_arch_rc_file} ${romfs_gen_root_dir}/init.d/${board_arch_rc_file}
		COMMAND ${CMAKE_COMMAND} -E touch ${board_arch_rc_file}.stamp
		DEPENDS
			${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/init/${CONFIG_ARCH_CHIP}/${board_arch_rc_file}
			romfs_copy.stamp
		COMMENT "ROMFS: copying ${board_arch_rc_file}"
	)

	list(APPEND extras_dependencies
		${board_arch_rc_file}.stamp
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
		file(RELATIVE_PATH rc_file_relative ${PX4_SOURCE_DIR} ${PX4_BOARD_DIR}/init/${board_rc_file})
		message(STATUS "ROMFS:  Adding ${rc_file_relative} -> /etc/init.d/${board_rc_file}")

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


if(config_uavcan_peripheral_firmware)

	include(ExternalProject)

	foreach(uavcan_peripheral_config ${config_uavcan_peripheral_firmware})
		# include the px4io binary in ROMFS
		message(STATUS "ROMFS:  Adding UAVCAN peripheral ${uavcan_peripheral_config} -> /etc/uavcan/fw/")
		ExternalProject_Add(build_${uavcan_peripheral_config}
			SOURCE_DIR ${CMAKE_SOURCE_DIR}
			DOWNLOAD_COMMAND ""
			UPDATE_COMMAND ""
			CMAKE_ARGS -DCONFIG=${uavcan_peripheral_config}
			INSTALL_COMMAND ""
			USES_TERMINAL_BUILD true
			DEPENDS git_nuttx git_nuttx_apps
			BUILD_ALWAYS 1
		)

		ExternalProject_Get_Property(build_${uavcan_peripheral_config} BINARY_DIR)

		add_custom_command(
			OUTPUT
				${romfs_gen_root_dir}/uavcan/fw/_${uavcan_board_id}.bin
				${uavcan_peripheral_config}.stamp
			COMMAND ${CMAKE_COMMAND} -E make_directory ${romfs_gen_root_dir}/uavcan/fw/
			COMMAND ${CMAKE_COMMAND} -E copy ${BINARY_DIR}/deploy/*.bin ${romfs_gen_root_dir}/uavcan/fw/
			COMMAND ls -lsa ${romfs_gen_root_dir}/uavcan/fw/
			COMMAND ${CMAKE_COMMAND} -E touch ${uavcan_peripheral_config}.stamp
			DEPENDS
				build_${uavcan_peripheral_config}
			COMMENT "ROMFS: copying ${uavcan_peripheral_config}"
		)

		list(APPEND extras_dependencies
			${uavcan_peripheral_config}.stamp
		)
	endforeach()
endif()

list(APPEND extras_dependencies
	${config_romfs_extra_dependencies}
	)

if (config_romfs_extra_files)
	set(extras_copy_cmd COMMAND ${CMAKE_COMMAND} -E copy_if_different ${config_romfs_extra_files} ${romfs_gen_root_dir}/extras/)
else()
	set(extras_copy_cmd "")
endif()
add_custom_command(OUTPUT romfs_extras.stamp
	COMMAND ${CMAKE_COMMAND} -E make_directory ${romfs_gen_root_dir}/extras/
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/romfs_extras/
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${PX4_BINARY_DIR}/romfs_extras/ ${romfs_gen_root_dir}/extras/
	${extras_copy_cmd}
	COMMAND ${CMAKE_COMMAND} -E touch romfs_extras.stamp
	DEPENDS
		romfs_copy.stamp
		${config_romfs_extra_files}
		${extras_dependencies}
	COMMENT "ROMFS: copying extras"
	)

add_custom_target(romfs_gen_files_target
	DEPENDS
		${romfs_extract_stamp}
		${romfs_gen_root_dir}/init.d/rc.serial
		romfs_extras.stamp
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

	add_library(romfs STATIC nsh_romfsimg.c)
	add_dependencies(romfs prebuild_targets)
	set_target_properties(romfs PROPERTIES LINKER_LANGUAGE C)

elseif("${CONFIG_FS_ROMFS}" STREQUAL "y")
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

	add_library(romfs STATIC nsh_romfsimg.c)
	add_dependencies(romfs prebuild_targets)
	set_target_properties(romfs PROPERTIES LINKER_LANGUAGE C)
endif()



# shellcheck
find_program(SHELLCHECK_PATH shellcheck)

if(SHELLCHECK_PATH)

	# TODO: fix SC2086, SC2166
	add_custom_target(shellcheck
		COMMAND ${SHELLCHECK_PATH} --shell=sh
			--exclude=SC1090 # SC1090: use of source (.) - Can't follow non-constant source. Use a directive to specify location.
			--exclude=SC1091 # SC1091: use of source (.) - Not following: xxxx openBinaryFile: does not exist (No such file or directory)
			--exclude=SC2121 # SC2121: To assign a variable, use just 'var=value'
			--exclude=SC2086 # SC2086: Double quote to prevent globbing and word splitting.
			--exclude=SC2166 # SC2166: Prefer [ p ] || [ q ] as [ p -o q ] is not well defined.
			--exclude=SC2154 # SC2154: <var> is referenced but not assigned (NuttX uses different asssignment)
			--exclude=SC2164 # SC2164: Use 'cd ... || exit' or 'cd ... || return' in case cd fails.
			--exclude=SC2169 # SC2169: In dash, 'source' in place of '.' is not supported. (we alias it)
			--exclude=SC2039 # SC2039: In POSIX sh, 'source' in place of '.' is undefined. (we alias it)
			--exclude=SC2181 # SC2181: Check exit code directly with e.g. 'if mycmd;', not indirectly with $?.
			`find ${romfs_gen_root_dir}/init.d -type f`
		DEPENDS ${romfs_gen_root_dir}/init.d/rc.autostart
		WORKING_DIRECTORY ${romfs_gen_root_dir}
		USES_TERMINAL
	)
endif()
