
px4_add_board(
	PLATFORM nuttx
	VENDOR nxp
	MODEL fmuk66-v3
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	ROMFSROOT px4fmu_common
	TESTING
	SERIAL_PORTS
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS4
		TEL2:/dev/ttyS1
	)
