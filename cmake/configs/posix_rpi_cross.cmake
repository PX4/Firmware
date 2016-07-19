include(configs/posix_rpi_common)

if("$ENV{RPI_USE_CLANG}" STREQUAL "1")
	set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-clang-gnueabihf-raspbian.cmake)
else()
	set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf-raspbian.cmake)
endif()


set(CMAKE_PROGRAM_PATH
	"${RPI_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf-raspbian/bin"
	${CMAKE_PROGRAM_PATH}
)
