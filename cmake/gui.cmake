set(BOARD_DEFCONFIG ${PX4_BOARD_DIR}/${PX4_BOARD_LABEL}-boardconfig CACHE FILEPATH "path to defconfig" FORCE)
set(BOARD_CONFIG ${PX4_BINARY_DIR}/boardconfig CACHE FILEPATH "path to config" FORCE)

find_program(MENUCONFIG_PATH menuconfig)
find_program(GUICONFIG_PATH guiconfig)
find_program(DEFCONFIG_PATH defconfig)
find_program(SAVEDEFCONFIG_PATH savedefconfig)
if(NOT MENUCONFIG_PATH OR NOT GUICONFIG_PATH OR NOT DEFCONFIG_PATH OR NOT SAVEDEFCONFIG_PATH)
    message(STATUS "kconfiglib is not installed\n"
                        "please install using \"pip3 install kconfiglib\"\n")
endif()

set(GUI_ENV_SETTINGS
	GUI_DEFCONFIG=${BOARD_DEFCONFIG}
	GUI_KCONFIG=${PX4_SOURCE_DIR}/Kconfig
	# Set environment variables so that Kconfig can prune Kconfig source
	# files for other architectures
	CONFIG=${CONFIG}
	PLATFORM=${PX4_PLATFORM}
	VENDOR=${PX4_BOARD_VENDOR}
	MODEL=${PX4_BOARD_MODEL}
	LABEL=${PX4_BOARD_LABEL}
	TOOLCHAIN=${CMAKE_TOOLCHAIN_FILE}
	ARCHITECTURE=${CMAKE_SYSTEM_PROCESSOR}
	ROMFSROOT=${config_romfs_root}
)

# FIXME ${PYTHON_EXECUTABLE} doesn't work here

add_custom_target(gui
	${CMAKE_COMMAND} -E env
	${GUI_ENV_SETTINGS}
	python3 Tools/gui.py
	COMMENT "Starting GUI ${PX4_BINARY_DIR}"
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	USES_TERMINAL
	COMMAND_EXPAND_LISTS
)
