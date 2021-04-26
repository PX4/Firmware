/****************************************************************************
 *
 *   Copyright (c) 2015-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifdef __PX4_NUTTX

#include "gpio.h"

#include <cstring>
#include <px4_arch/io_timer.h>

CameraInterfaceGPIO::CameraInterfaceGPIO()
{
	_p_polarity = param_find("TRIG_POLARITY");

	// polarity of the trigger (0 = active low, 1 = active high )
	int32_t polarity = 0;
	param_get(_p_polarity, &polarity);
	_trigger_invert = (polarity == 0);

	get_pins();
	setup();
}

void CameraInterfaceGPIO::setup()
{
	for (unsigned i = 0, t = 0; i < arraySize(_pins); i++) {
		// Pin range is from 0 to num_gpios - 1
		if (_pins[i] >= 0 && t < (int)arraySize(_triggers)) {
			uint32_t gpio = io_timer_channel_get_gpio_output(_pins[i]);
			_triggers[t++] = gpio;
			px4_arch_configgpio(gpio);
			px4_arch_gpiowrite(gpio, false ^ _trigger_invert);
		}
	}
}

void CameraInterfaceGPIO::trigger(bool trigger_on_true)
{
	bool trigger_state = trigger_on_true ^ _trigger_invert;

	for (unsigned i = 0; i < arraySize(_triggers); i++) {
		if (_triggers[i] != 0) {
			px4_arch_gpiowrite(_triggers[i], trigger_state);
		}
	}
}

void CameraInterfaceGPIO::info()
{
	PX4_INFO_RAW("GPIO trigger mode, pins enabled: ");

	for (unsigned i = 0; i < arraySize(_pins); ++i) {
		PX4_INFO_RAW("[%d]", _pins[i]);
	}

	PX4_INFO_RAW(", polarity : %s\n", _trigger_invert ? "ACTIVE_LOW" : "ACTIVE_HIGH");
}

#endif /* ifdef __PX4_NUTTX */
