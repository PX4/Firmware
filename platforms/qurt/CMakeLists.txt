
px4_add_git_submodule(TARGET git_dspal PATH "${PX4_SOURCE_DIR}/platforms/qurt/dspal")

list(APPEND CMAKE_MODULE_PATH "${PX4_SOURCE_DIR}/platforms/qurt/dspal/cmake_hexagon")
include(toolchain/Toolchain-qurt)
include(fastrpc)
include(qurt_lib)
include(qurt_flags)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

get_property(module_libraries GLOBAL PROPERTY PX4_MODULE_LIBRARIES)

px4_qurt_generate_builtin_commands(
	OUT ${PX4_BINARY_DIR}/apps
	MODULE_LIST ${module_libraries}
)

FASTRPC_STUB_GEN(px4muorb.idl)

add_definitions(-D__QAIC_SKEL_EXPORT=__EXPORT)

# Enable build without HexagonSDK to check link dependencies
if ("${QURT_ENABLE_STUBS}" STREQUAL "1")

	add_definitions(-D QURT_EXE_BUILD=1)

	include_directories(
                ${CMAKE_CURRENT_BINARY_DIR}
                ${FASTRPC_DSP_INCLUDES}
                )

	add_executable(px4
		${PX4_BINARY_DIR}/apps.cpp
		${PX4_BINARY_DIR}/platforms/qurt/px4muorb_skel.c
	)

	target_link_libraries(px4 PRIVATE ${module_libraries})

else()
	# Generate the DSP lib and stubs but not the apps side executable
	# The Apps side executable is generated via the posix_eagle_xxxx target
	QURT_LIB(LIB_NAME px4
		IDL_NAME px4muorb
		SOURCES
			${PX4_BINARY_DIR}/apps.cpp
		LINK_LIBS
			modules__muorb__adsp
			${module_libraries}
			m
		)

endif()

# board defined upload helper
if(EXISTS "${PX4_BOARD_DIR}/cmake/upload.cmake")
	include(${PX4_BOARD_DIR}/cmake/upload.cmake)
endif()
