#ifdef __PX4_NUTTX

#include "gpio.h"
#include <cstring>
#include <px4_arch/io_timer.h>

CameraInterfaceGPIO::CameraInterfaceGPIO():
	CameraInterface(),
	_trigger_invert(false),
	_triggers{}
{
	_p_polarity = param_find("TRIG_POLARITY");

	// polarity of the trigger (0 = active low, 1 = active high )
	int32_t polarity;
	param_get(_p_polarity, &polarity);
	_trigger_invert = (polarity == 0);

	get_pins();
	setup();
}

void CameraInterfaceGPIO::setup()
{
	for (unsigned i = 0, t = 0; i < arraySize(_pins); i++) {

		// Pin range is from 0 to num_gpios - 1
		if (_pins[i] >= 0 && t < (int)arraySize(_triggers) && _pins[i] < num_gpios) {
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

		if (_triggers[i] == 0) { break; }

		px4_arch_gpiowrite(_triggers[i], trigger_state);
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
