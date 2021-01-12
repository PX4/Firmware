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

# NuttX CDCACM vendor and product strings
set(vendorstr_underscore)
set(productstr_underscore)
string(REPLACE " " "_" vendorstr_underscore ${CONFIG_CDCACM_VENDORSTR})
string(REPLACE " " "_" productstr_underscore ${CONFIG_CDCACM_PRODUCTSTR})

set(serial_ports)
if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")

	set(px4_usb_path "${vendorstr_underscore}_${productstr_underscore}")

	list(APPEND serial_ports
		# NuttX vendor + product string
		/dev/serial/by-id/*-${px4_usb_path}*

		# Bootloader
		/dev/serial/by-id/*PX4_BL* # typical bootloader USB device string
		/dev/serial/by-id/*BL_FMU*

		# TODO: handle these per board
		/dev/serial/by-id/usb-The_Autopilot*
		/dev/serial/by-id/usb-Bitcraze*
		/dev/serial/by-id/pci-Bitcraze*
		/dev/serial/by-id/usb-Gumstix*
		/dev/serial/by-id/usb-Hex_ProfiCNC*
		/dev/serial/by-id/usb-UVify*
		/dev/serial/by-id/usb-ArduPilot*
		)

elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Darwin")
	list(APPEND serial_ports
		/dev/tty.usbmodemPX*,/dev/tty.usbmodem*
		)
elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "CYGWIN")
	list(APPEND serial_ports
		/dev/ttyS*
		)
elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
	foreach(port RANGE 32 0)
		list(APPEND serial_ports
			"COM${port}")
	endforeach()
endif()

string(REPLACE ";" "," serial_ports "${serial_ports}")

add_custom_target(upload
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_uploader.py --port ${serial_ports} ${fw_package}
	DEPENDS ${fw_package}
	COMMENT "uploading px4"
	VERBATIM
	USES_TERMINAL
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	)

add_custom_target(force-upload
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_uploader.py --force --port ${serial_ports} ${fw_package}
	DEPENDS ${fw_package}
	COMMENT "uploading px4 with --force"
	VERBATIM
	USES_TERMINAL
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	)
