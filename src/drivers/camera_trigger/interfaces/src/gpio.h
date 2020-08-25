/**
 * @file relay.h
 *
 * Interface with cameras via FMU auxiliary pins.
 *
 */
#pragma once

#ifdef __PX4_NUTTX

#include <board_config.h>

#include "camera_interface.h"

class CameraInterfaceGPIO : public CameraInterface
{
public:
	CameraInterfaceGPIO();
	virtual ~CameraInterfaceGPIO() = default;

	void trigger(bool trigger_on_true);

	void info();

private:
	static const int num_gpios = DIRECT_PWM_OUTPUT_CHANNELS > 6 ? 6 : DIRECT_PWM_OUTPUT_CHANNELS;

	void setup();

	param_t _p_polarity;

	bool _trigger_invert;

	uint32_t _triggers[num_gpios];
};

#endif /* ifdef __PX4_NUTTX */
