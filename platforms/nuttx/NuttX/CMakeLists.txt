############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

if (CMAKE_HOST_APPLE OR CMAKE_HOST_WIN32)
	# copy with rsync and create file dependencies
	set(NUTTX_COPY_CMD "rsync")
	set(NUTTX_COPY_CMD_OPTS)
	list(APPEND NUTTX_COPY_CMD_OPTS
		-rp
		--inplace
	)
else()
	# copy with hard links
	# archive, recursive, force, link (hardlinks)
	set(NUTTX_COPY_CMD "cp")
	set(NUTTX_COPY_CMD_OPTS "-aRfl")
endif()

set(NUTTX_CONFIG_DIR ${PX4_BOARD_DIR}/nuttx-config)

###############################################################################
# NuttX: copy to build directory
###############################################################################
file(RELATIVE_PATH CP_SRC ${CMAKE_SOURCE_DIR} ${NUTTX_SRC_DIR}/nuttx)
file(RELATIVE_PATH CP_DST ${CMAKE_SOURCE_DIR} ${PX4_BINARY_DIR}/NuttX)

execute_process(
	COMMAND git rev-parse --absolute-git-dir
	OUTPUT_VARIABLE nuttx_git_dir
	WORKING_DIRECTORY ${NUTTX_SRC_DIR}/nuttx
	OUTPUT_STRIP_TRAILING_WHITESPACE
)

# setup custom command to copy changes later
if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.11.0")
	file(GLOB_RECURSE copy_nuttx_files LIST_DIRECTORIES false CONFIGURE_DEPENDS ${NUTTX_SRC_DIR}/nuttx/ Make.defs Kconfig *.c *.h *.cxx *.hxx)
else()
	file(GLOB_RECURSE copy_nuttx_files LIST_DIRECTORIES false ${NUTTX_SRC_DIR}/nuttx/ Make.defs Kconfig *.c *.h *.cxx *.hxx)
endif()
list(REMOVE_ITEM copy_nuttx_files ${NUTTX_SRC_DIR}/nuttx/.git)

add_custom_command(
	OUTPUT ${PX4_BINARY_DIR}/NuttX/nuttx_copy.stamp
	COMMAND ${NUTTX_COPY_CMD} ${NUTTX_COPY_CMD_OPTS} ${CP_SRC} ${CP_DST}
	COMMAND ${CMAKE_COMMAND} -E remove -f ${PX4_BINARY_DIR}/NuttX/nuttx/.git
	COMMAND echo "gitdir: ${nuttx_git_dir}" > ${PX4_BINARY_DIR}/NuttX/nuttx/.git
	COMMAND ${CMAKE_COMMAND} -E touch ${PX4_BINARY_DIR}/NuttX/nuttx_copy.stamp
	DEPENDS
		git_nuttx
		${copy_nuttx_files}
	COMMENT "Copying NuttX/nuttx to ${CP_DST}"
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

###############################################################################
# NuttX apps: copy to build directory
###############################################################################
file(RELATIVE_PATH CP_SRC ${CMAKE_SOURCE_DIR} ${NUTTX_SRC_DIR}/apps)
file(RELATIVE_PATH CP_DST ${CMAKE_SOURCE_DIR} ${PX4_BINARY_DIR}/NuttX)

# setup custom command to copy changes later
if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.11.0")
	file(GLOB_RECURSE copy_apps_files LIST_DIRECTORIES false CONFIGURE_DEPENDS ${NUTTX_SRC_DIR}/apps/ Make.defs Kconfig *.c *.h *.cxx *.hxx)
else()
	file(GLOB_RECURSE copy_apps_files LIST_DIRECTORIES false ${NUTTX_SRC_DIR}/apps/ Make.defs Kconfig *.c *.h *.cxx *.hxx)
endif()
add_custom_command(
	OUTPUT ${PX4_BINARY_DIR}/NuttX/apps_copy.stamp
	COMMAND ${NUTTX_COPY_CMD} ${NUTTX_COPY_CMD_OPTS} ${CP_SRC} ${CP_DST}
	COMMAND ${CMAKE_COMMAND} -E touch ${PX4_BINARY_DIR}/NuttX/apps_copy.stamp
	DEPENDS
		git_nuttx_apps
		${copy_apps_files}
	COMMENT "Copying NuttX/apps to ${CP_DST}"
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
set(APPS_DIR ${CMAKE_CURRENT_BINARY_DIR}/apps)


# For any dependencies of commands on files we need to create a target.
# Otherwise, if "Unix Makefiles" are used as the generator the commands are run in
# parallel on the different files which often can lead to races or redundancies
# in our build.
# A nice write-up can be found here:
# https://samthursfield.wordpress.com/2015/11/21/cmake-dependencies-between-targets-and-files-and-custom-commands/#custom-commands-and-parallel-make
add_custom_target(nuttx_copy_and_apps_target
	DEPENDS
		${PX4_BINARY_DIR}/NuttX/nuttx_copy.stamp
		${PX4_BINARY_DIR}/NuttX/apps_copy.stamp
)

# If the board provides a Kconfig Use it or create an empty one
if(EXISTS ${NUTTX_CONFIG_DIR}/Kconfig)
	add_custom_command(
		OUTPUT ${PX4_BINARY_DIR}/NuttX/nuttx_config_kconfig.stamp
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_CONFIG_DIR}/Kconfig ${NUTTX_DIR}/boards/dummy/Kconfig
		COMMAND ${CMAKE_COMMAND} -E touch ${PX4_BINARY_DIR}/NuttX/nuttx_config_kconfig.stamp
		DEPENDS
			nuttx_copy_and_apps_target ${PX4_BINARY_DIR}/NuttX/nuttx_copy.stamp ${PX4_BINARY_DIR}/NuttX/apps_copy.stamp
		)
else()
	add_custom_command(
		OUTPUT ${PX4_BINARY_DIR}/NuttX/nuttx_config_kconfig.stamp
		COMMAND ${CMAKE_COMMAND} -E touch ${NUTTX_DIR}/boards/dummy/Kconfig
		COMMAND ${CMAKE_COMMAND} -E touch ${PX4_BINARY_DIR}/NuttX/nuttx_config_kconfig.stamp
		DEPENDS
			nuttx_copy_and_apps_target ${PX4_BINARY_DIR}/NuttX/nuttx_copy.stamp ${PX4_BINARY_DIR}/NuttX/apps_copy.stamp
		)
endif()

add_custom_target(nuttx_config_kconfig_target DEPENDS ${PX4_BINARY_DIR}/NuttX/nuttx_config_kconfig.stamp)

###############################################################################
# NuttX configure
###############################################################################

# copy NuttX config directory
file(RELATIVE_PATH CP_SRC ${NUTTX_DIR} ${PX4_BOARD_DIR}/nuttx-config)
file(RELATIVE_PATH CP_DST ${NUTTX_DIR} ${PX4_BINARY_DIR}/NuttX)
add_custom_command(
	OUTPUT ${PX4_BINARY_DIR}/NuttX/nuttx_copy_config_dir.stamp
	COMMAND ${NUTTX_COPY_CMD} ${NUTTX_COPY_CMD_OPTS} ${CP_SRC} ${CP_DST}
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/NuttX/nuttx-config/drivers
	COMMAND ${CMAKE_COMMAND} -E touch ${PX4_BINARY_DIR}/NuttX/nuttx-config/drivers/Kconfig
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/NuttX/nuttx-config/src
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SRC_DIR}/nsh_romfsimg.h ${PX4_BINARY_DIR}/NuttX/nuttx-config/include/nsh_romfsimg.h
	COMMAND ${CMAKE_COMMAND} -E touch ${PX4_BINARY_DIR}/NuttX/nuttx_copy_config_dir.stamp
	DEPENDS
		${NUTTX_CONFIG_DIR}/include/board.h
		${NUTTX_CONFIG_DIR}/scripts/script.ld
		${NUTTX_SRC_DIR}/nsh_romfsimg.h
		nuttx_config_kconfig_target ${PX4_BINARY_DIR}/NuttX/nuttx_config_kconfig.stamp
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Copying NuttX config ${NUTTX_CONFIG}"
)
add_custom_target(nuttx_copy_config_dir_target DEPENDS ${PX4_BINARY_DIR}/NuttX/nuttx_copy_config_dir.stamp)

if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 9.3)
	set(CMAKE_C_COMP_DEP_FLAGS -Wno-stringop-truncation)
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Make.defs.in ${NUTTX_DIR}/Make.defs)

# copy compressed PX4 defconfig into nuttx and inflate
add_custom_command(
	OUTPUT
		${NUTTX_DIR}/.config
		${PX4_BINARY_DIR}/NuttX/nuttx_olddefconfig.stamp
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config
	COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/px4_nuttx_make_olddefconfig.sh > nuttx_olddefconfig.log
	COMMAND ${CMAKE_COMMAND} -E touch ${PX4_BINARY_DIR}/NuttX/nuttx_olddefconfig.stamp
	DEPENDS
		${NUTTX_DIR}/Make.defs
		${NUTTX_DEFCONFIG}
		nuttx_copy_config_dir_target ${PX4_BINARY_DIR}/NuttX/nuttx_copy_config_dir.stamp
		${CMAKE_CURRENT_SOURCE_DIR}/tools/px4_nuttx_make_olddefconfig.sh
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Copying NuttX compressed config ${NUTTX_CONFIG} and inflating (make olddefconfig)"
)
add_custom_target(nuttx_config_target DEPENDS ${NUTTX_DIR}/.config ${PX4_BINARY_DIR}/NuttX/nuttx_olddefconfig.stamp)

###############################################################################
# NuttX build
###############################################################################

# verbose build settings (V=1 or VERBOSE=1)
option(PX4_NUTTX_VERBOSE "PX4 NuttX verbose build" off)

if(($ENV{V} MATCHES "1") OR ($ENV{VERBOSE} MATCHES "1"))
	message(STATUS "NuttX verbose build enabled")
	set(PX4_NUTTX_VERBOSE on)
endif()

if(PX4_NUTTX_VERBOSE)
	set(nuttx_build_options)
	set(nuttx_build_uses_terminal "USES_TERMINAL")
else()
	set(nuttx_build_options "--quiet")
	set(nuttx_build_uses_terminal)
endif()

# context
add_custom_command(
	OUTPUT
		${NUTTX_DIR}/include/nuttx/config.h
		${NUTTX_DIR}/include/nuttx/version.h
	COMMAND
		make ${nuttx_build_options} --no-print-directory CONFIG_ARCH_BOARD_CUSTOM=y pass1dep > nuttx_context.log
	DEPENDS
		${NUTTX_DIR}/Make.defs
		nuttx_config_target ${NUTTX_DIR}/.config ${PX4_BINARY_DIR}/NuttX/nuttx_olddefconfig.stamp
	WORKING_DIRECTORY ${NUTTX_DIR}
	${nuttx_build_uses_terminal}
)
add_custom_target(nuttx_context DEPENDS ${NUTTX_DIR}/include/nuttx/config.h ${NUTTX_DIR}/include/nuttx/version.h)

# library of NuttX libraries
add_library(nuttx_build INTERFACE)

add_custom_target(px4_config_file_target DEPENDS ${PX4_CONFIG_FILE})

# builtins
set(nuttx_builtin_list)
if(CONFIG_NSH_LIBRARY)
	# force builtins regeneration and apps rebuild if nuttx or px4 configuration have changed
	add_custom_command(OUTPUT ${PX4_BINARY_DIR}/NuttX/builtins_clean.stamp
		COMMAND find ${APPS_DIR}/builtin/registry -name px4_\*.bdat -delete
		COMMAND find ${APPS_DIR}/builtin/registry -name px4_\*.pdat -delete
		COMMAND rm -f ${APPS_DIR}/builtin/builtin_list.h
		COMMAND ${CMAKE_COMMAND} -E touch ${PX4_BINARY_DIR}/NuttX/builtins_clean.stamp
		DEPENDS
			nuttx_context ${NUTTX_DIR}/include/nuttx/config.h ${NUTTX_DIR}/include/nuttx/version.h
			px4_config_file_target ${PX4_CONFIG_FILE}
	)

	add_custom_target(builtins_clean_target DEPENDS ${PX4_BINARY_DIR}/NuttX/builtins_clean.stamp)

	foreach(module ${module_libraries})
		get_target_property(MAIN ${module} MAIN)
		get_target_property(STACK_MAIN ${module} STACK_MAIN)
		get_target_property(PRIORITY ${module} PRIORITY)

		if(MAIN)
			add_custom_command(OUTPUT ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.bdat
				COMMAND echo "{ \"${MAIN}\", ${PRIORITY}, ${STACK_MAIN}, ${MAIN}_main }," > ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.bdat
				COMMAND ${CMAKE_COMMAND} -E touch ${APPS_DIR}/builtin/registry/.updated
				DEPENDS
					builtins_clean_target ${PX4_BINARY_DIR}/NuttX/builtins_clean.stamp
					nuttx_context ${NUTTX_DIR}/include/nuttx/config.h ${NUTTX_DIR}/include/nuttx/version.h
				VERBATIM
				)
			list(APPEND nuttx_builtin_list ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.bdat)

			add_custom_command(OUTPUT ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.pdat
				COMMAND echo "int ${MAIN}_main(int argc, char *argv[]);" > ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.pdat
				COMMAND ${CMAKE_COMMAND} -E touch ${APPS_DIR}/builtin/registry/.updated
				DEPENDS
					builtins_clean_target ${PX4_BINARY_DIR}/NuttX/builtins_clean.stamp
					nuttx_context ${NUTTX_DIR}/include/nuttx/config.h ${NUTTX_DIR}/include/nuttx/version.h
				VERBATIM
				)
			list(APPEND nuttx_builtin_list ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.pdat)

		endif()
	endforeach()
endif()

add_custom_target(nuttx_builtin_list_target DEPENDS ${nuttx_builtin_list})

# APPS

# libapps.a
add_custom_command(OUTPUT ${APPS_DIR}/libapps.a
	COMMAND find ${APPS_DIR} -name \*.o -delete
	COMMAND make ${nuttx_build_options} --no-print-directory TOPDIR="${NUTTX_DIR}" > nuttx_apps.log
	DEPENDS
		nuttx_builtin_list_target ${nuttx_builtin_list}
		nuttx_context ${NUTTX_DIR}/include/nuttx/config.h ${NUTTX_DIR}/include/nuttx/version.h
	WORKING_DIRECTORY ${APPS_DIR}
	${nuttx_build_uses_terminal}
)
add_custom_target(nuttx_apps_build DEPENDS ${APPS_DIR}/libapps.a)
add_library(nuttx_apps STATIC IMPORTED GLOBAL)
set_property(TARGET nuttx_apps PROPERTY IMPORTED_LOCATION ${APPS_DIR}/libapps.a)
add_dependencies(nuttx_build nuttx_apps_build)
target_link_libraries(nuttx_build INTERFACE nuttx_apps)

# helper for all targets
function(add_nuttx_dir nuttx_lib nuttx_lib_dir kernel extra)
	file(GLOB_RECURSE nuttx_lib_files
		LIST_DIRECTORIES false
		${CMAKE_CURRENT_SOURCE_DIR}/nuttx/${nuttx_lib_dir}/*)

	add_custom_command(OUTPUT ${NUTTX_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a
		COMMAND find ${nuttx_lib_dir} -type f -name *.o -delete
		COMMAND make -C ${nuttx_lib_dir} ${nuttx_build_options} --no-print-directory all TOPDIR=${NUTTX_DIR} KERNEL=${kernel} EXTRAFLAGS=${extra}
		DEPENDS
			${nuttx_lib_files}
			nuttx_context ${NUTTX_DIR}/include/nuttx/config.h ${NUTTX_DIR}/include/nuttx/version.h
		WORKING_DIRECTORY ${NUTTX_DIR}
		${nuttx_build_uses_terminal}
	)
	add_custom_target(nuttx_${nuttx_lib}_build DEPENDS ${NUTTX_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a)
	add_library(nuttx_${nuttx_lib} STATIC IMPORTED GLOBAL)
	set_property(TARGET nuttx_${nuttx_lib} PROPERTY IMPORTED_LOCATION ${NUTTX_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a)
	add_dependencies(nuttx_build nuttx_${nuttx_lib}_build)
	target_link_libraries(nuttx_build INTERFACE nuttx_${nuttx_lib})
endfunction()

# add_nuttx_dir(NAME DIRECTORY KERNEL EXTRA)
add_nuttx_dir(arch arch/${CONFIG_ARCH}/src y -D__KERNEL__)
add_nuttx_dir(binfmt binfmt y -D__KERNEL__)
add_nuttx_dir(boards boards y -D__KERNEL__)
add_nuttx_dir(drivers drivers y -D__KERNEL__)
add_nuttx_dir(fs fs y -D__KERNEL__)
add_nuttx_dir(sched sched y -D__KERNEL__)
add_nuttx_dir(c libs/libc n "")
add_nuttx_dir(xx libs/libxx n "")
add_nuttx_dir(mm mm n "")

if(CONFIG_NET)
	add_nuttx_dir(net net y -D__KERNEL__)
endif()

###############################################################################
# NuttX oldconfig
add_custom_target(oldconfig_nuttx
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y oldconfig
	DEPENDS nuttx_config_target ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make oldconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX oldconfig + savedefconfig back to PX4
add_custom_target(oldconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/.config
	DEPENDS oldconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make oldconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

###############################################################################
# NuttX olddefconfig
add_custom_target(olddefconfig_nuttx
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y olddefconfig
	DEPENDS nuttx_config_target ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make olddefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
	)

# NuttX olddefconfig + savedefconfig back to PX4
add_custom_target(olddefconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/.config
	DEPENDS olddefconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make olddefconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

###############################################################################
# NuttX menuconfig
add_custom_target(menuconfig_nuttx
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y menuconfig
	DEPENDS nuttx_config_target ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make menuconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
	)

# NuttX menuconfig + savedefconfig back to PX4
add_custom_target(menuconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/.config
	DEPENDS menuconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make nuttx_menuconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

###############################################################################
# NuttX qconfig
add_custom_target(qconfig_nuttx
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y qconfig
	DEPENDS nuttx_config_target ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make qconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
	)

# NuttX qconfig + savedefconfig back to PX4
add_custom_target(qconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/.config
	DEPENDS qconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make qconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)
