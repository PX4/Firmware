############################################################################
#
#   Copyright (c) 2016, 2018 PX4 Development Team. All rights reserved.
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
if("${PX4_BOARD_LABEL}" STREQUAL  "canbootloader")

	add_library(drivers_board
		can_boot.c
		led.cpp
		clockconfig.c
		periphclocks.c
		timer_config.cpp
	)

	target_link_libraries(drivers_board
		PRIVATE
			nuttx_arch
			nuttx_drivers
			canbootloader
			arch_io_pins
			arch_led_pwm
	)
	target_include_directories(drivers_board PRIVATE ${PX4_SOURCE_DIR}/platforms/nuttx/src/canbootloader)

else()

	add_library(drivers_board
		autoleds.c
		boot.c
		bringup.c
		buttons.c
		clockconfig.c
		i2c.cpp
		init.c
		mtd.cpp
		periphclocks.c
		spi.c
		timer_config.cpp
		userleds.c
	)

	target_link_libraries(drivers_board
		PRIVATE
			nuttx_arch # sdio
			nuttx_drivers # sdio
			drivers__led # drv_led_start
			px4_layer
			arch_io_pins
		)
endif()
