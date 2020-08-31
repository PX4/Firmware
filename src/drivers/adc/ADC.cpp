/****************************************************************************
 *
 *   Copyright (C) 2012-2020 PX4 Development Team. All rights reserved.
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

#include <uORB/Subscription.hpp>

#include "ADC.hpp"

ADC::ADC(uint32_t base_address, uint32_t channels) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample")),
	_base_address(base_address)
{
	/* always enable the temperature sensor */
	channels |= px4_arch_adc_temp_sensor_mask();

	/* allocate the sample array */
	for (unsigned i = 0; i < ADC_TOTAL_CHANNELS; i++) {
		if (channels & (1 << i)) {
			_channel_count++;
		}
	}

	if (_channel_count > PX4_MAX_ADC_CHANNELS) {
		PX4_ERR("PX4_MAX_ADC_CHANNELS is too small (%d, %d)", (unsigned)PX4_MAX_ADC_CHANNELS, _channel_count);
	}

	_samples = new px4_adc_msg_t[_channel_count];

	/* prefill the channel numbers in the sample array */
	if (_samples != nullptr) {
		unsigned index = 0;

		for (unsigned i = 0; i < ADC_TOTAL_CHANNELS; i++) {
			if (channels & (1 << i)) {
				_samples[index].am_channel = i;
				_samples[index].am_data = 0;
				index++;
			}
		}
	}
}

ADC::~ADC()
{
	ScheduleClear();

	if (_samples != nullptr) {
		delete _samples;
	}

	perf_free(_sample_perf);
	px4_arch_adc_uninit(_base_address);
}

int ADC::init()
{
	int ret_init = px4_arch_adc_init(_base_address);

	if (ret_init < 0) {
		PX4_ERR("arch adc init failed");
		return ret_init;
	}

	// schedule regular updates
	ScheduleOnInterval(kINTERVAL, kINTERVAL);

	return PX4_OK;
}

void ADC::Run()
{
	hrt_abstime now = hrt_absolute_time();

	/* scan the channel set and sample each */
	for (unsigned i = 0; i < _channel_count; i++) {
		_samples[i].am_data = sample(_samples[i].am_channel);
	}

	update_adc_report(now);
	update_system_power(now);
}

void ADC::update_adc_report(hrt_abstime now)
{
	adc_report_s adc = {};
	adc.timestamp = now;
	adc.device_id = BUILTIN_ADC_DEVID;

	unsigned max_num = _channel_count;

	if (max_num > (sizeof(adc.channel_id) / sizeof(adc.channel_id[0]))) {
		max_num = (sizeof(adc.channel_id) / sizeof(adc.channel_id[0]));
	}

	unsigned i;

	for (i = 0; i < max_num; i++) {
		adc.channel_id[i] = _samples[i].am_channel;
		adc.raw_data[i] = _samples[i].am_data;
	}

	for (; i < PX4_MAX_ADC_CHANNELS; ++i) {	// set unused channel id to -1
		adc.channel_id[i] = -1;
	}

	adc.v_ref = px4_arch_adc_reference_v();
	adc.resolution = px4_arch_adc_dn_fullcount();

	_to_adc_report.publish(adc);
}

void ADC::update_system_power(hrt_abstime now)
{
#if defined (BOARD_ADC_USB_CONNECTED)
	system_power_s system_power {};

	/* Assume HW provides only ADC_SCALED_V5_SENSE */
	int cnt = 1;
	/* HW provides both ADC_SCALED_V5_SENSE and ADC_SCALED_V3V3_SENSORS_SENSE */
#  if defined(ADC_SCALED_V5_SENSE) && defined(ADC_SCALED_V3V3_SENSORS_SENSE)
	cnt++;
#  endif

	for (unsigned i = 0; i < _channel_count; i++) {
#  if defined(ADC_SCALED_V5_SENSE)

		if (_samples[i].am_channel == ADC_SCALED_V5_SENSE) {
			// it is 2:1 scaled
			system_power.voltage5v_v = _samples[i].am_data * (ADC_V5_V_FULL_SCALE / px4_arch_adc_dn_fullcount());
			cnt--;

		} else
#  endif
#  if defined(ADC_SCALED_V3V3_SENSORS_SENSE)
		{
			if (_samples[i].am_channel == ADC_SCALED_V3V3_SENSORS_SENSE) {
				// it is 2:1 scaled
				system_power.voltage3v3_v = _samples[i].am_data * (ADC_3V3_SCALE * (3.3f / px4_arch_adc_dn_fullcount()));
				system_power.v3v3_valid = 1;
				cnt--;
			}
		}

#  endif

		if (cnt == 0) {
			break;
		}
	}

	/* Note once the board_config.h provides BOARD_ADC_USB_CONNECTED,
	 * It must provide the true logic GPIO BOARD_ADC_xxxx macros.
	 */
	// these are not ADC related, but it is convenient to
	// publish these to the same topic

	system_power.usb_connected = BOARD_ADC_USB_CONNECTED;
	/* If provided used the Valid signal from HW*/
#if defined(BOARD_ADC_USB_VALID)
	system_power.usb_valid = BOARD_ADC_USB_VALID;
#else
	/* If not provided then use connected */
	system_power.usb_valid = system_power.usb_connected;
#endif

#if defined(BOARD_BRICK_VALID_LIST)
	/* The valid signals (HW dependent) are associated with each brick */
	bool  valid_chan[BOARD_NUMBER_BRICKS] = BOARD_BRICK_VALID_LIST;
	system_power.brick_valid = 0;

	for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {
		system_power.brick_valid |=  valid_chan[b] ? 1 << b : 0;
	}

#endif

#if defined(BOARD_ADC_SERVO_VALID)
	system_power.servo_valid = BOARD_ADC_SERVO_VALID;
#endif

#if defined(BOARD_ADC_PERIPH_5V_OC)
	// OC pins are active low
	system_power.periph_5v_oc = BOARD_ADC_PERIPH_5V_OC;
#endif

#if defined(BOARD_ADC_HIPOWER_5V_OC)
	system_power.hipower_5v_oc = BOARD_ADC_HIPOWER_5V_OC;
#endif

	system_power.timestamp = hrt_absolute_time();
	_to_system_power.publish(system_power);

#endif // BOARD_ADC_USB_CONNECTED
}

uint32_t ADC::sample(unsigned channel)
{
	perf_begin(_sample_perf);
	uint32_t result = px4_arch_adc_sample(_base_address, channel);

	if (result == UINT32_MAX) {
		PX4_ERR("sample timeout");
	}

	perf_end(_sample_perf);
	return result;
}

int ADC::test()
{
	uORB::Subscription	adc_sub_test{ORB_ID(adc_report)};
	adc_report_s adc;

	px4_usleep(20000);	// sleep 20ms and wait for adc report

	if (adc_sub_test.update(&adc)) {
		PX4_INFO_RAW("DeviceID: %d\n", adc.device_id);
		PX4_INFO_RAW("Resolution: %d\n", adc.resolution);
		PX4_INFO_RAW("Voltage Reference: %f\n", (double)adc.v_ref);

		for (unsigned l = 0; l < 20; ++l) {
			for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
				if (adc.channel_id[i] >= 0) {
					PX4_INFO_RAW("% 2d:% 6d", adc.channel_id[i], adc.raw_data[i]);
				}
			}

			PX4_INFO_RAW("\n");
			px4_usleep(500000);

			if (!adc_sub_test.update(&adc)) {
				PX4_INFO_RAW("\t ADC test failed.\n");
			}
		}

		PX4_INFO_RAW("\t ADC test successful.\n");

		return 0;

	} else {
		return 1;
	}
}

int ADC::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "test")) {
		if (is_running()) {
			return _object.load()->test();
		}

		return PX4_ERROR;
	}

	return print_usage("unknown command");
}

int ADC::task_spawn(int argc, char *argv[])
{
	ADC *instance = new ADC(SYSTEM_ADC_BASE, ADC_CHANNELS);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ADC::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
ADC driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("adc", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("test");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int adc_main(int argc, char *argv[])
{
	return ADC::main(argc, argv);
}
