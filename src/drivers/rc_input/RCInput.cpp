/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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

#include "RCInput.hpp"

#include "crsf_telemetry.h"
#include <uORB/topics/vehicle_command_ack.h>

#include <termios.h>

using namespace time_literals;

constexpr char const *RCInput::RC_SCAN_STRING[];

RCInput::RCInput(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device)),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")),
	_publish_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": publish interval"))
{
	// rc input, published to ORB
	_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;

	// initialize it as RC lost
	_rc_in.rc_lost = true;

	// initialize raw_rc values and count
	for (unsigned i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; i++) {
		_raw_rc_values[i] = UINT16_MAX;
	}

	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}
}

RCInput::~RCInput()
{
#if defined(SPEKTRUM_POWER_PASSIVE)
	// Disable power controls for Spektrum receiver
	SPEKTRUM_POWER_PASSIVE();
#endif
	dsm_deinit();

	delete _crsf_telemetry;
	delete _ghst_telemetry;

	perf_free(_cycle_perf);
	perf_free(_publish_interval_perf);
}

int
RCInput::init()
{
#ifdef RF_RADIO_POWER_CONTROL
	// power radio on
	RF_RADIO_POWER_CONTROL(true);
#endif // RF_RADIO_POWER_CONTROL

	// dsm_init sets some file static variables and returns a file descriptor
	// it also powers on the radio if needed
	_rcs_fd = dsm_init(_device);

	if (_rcs_fd < 0) {
		return -errno;
	}

	if (board_rc_swap_rxtx(_device)) {
#if defined(TIOCSSWAP)
		ioctl(_rcs_fd, TIOCSSWAP, SER_SWAP_ENABLED);
#endif // TIOCSSWAP
	}

	// assume SBUS input and immediately switch it to
	// so that if Single wire mode on TX there will be only
	// a short contention
	sbus_config(_rcs_fd, board_rc_singlewire(_device));

#ifdef GPIO_PPM_IN
	// disable CPPM input by mapping it away from the timer capture input
	px4_arch_unconfiggpio(GPIO_PPM_IN);
#endif // GPIO_PPM_IN

	return 0;
}

int
RCInput::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device = nullptr;
#if defined(RC_SERIAL_PORT)
	device = RC_SERIAL_PORT;
#endif // RC_SERIAL_PORT

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return -1;
	}

	if (device == nullptr) {
		PX4_ERR("valid device required");
		return PX4_ERROR;
	}

	RCInput *instance = new RCInput(device);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->ScheduleOnInterval(_current_update_interval);

	return PX4_OK;
}

void
RCInput::fill_rc_in(uint16_t raw_rc_count_local,
		    uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
		    hrt_abstime now, bool frame_drop, bool failsafe,
		    unsigned frame_drops, int rssi = -1)
{
	// fill rc_in struct for publishing
	_rc_in.channel_count = raw_rc_count_local;

	if (_rc_in.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		_rc_in.channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	unsigned valid_chans = 0;

	for (unsigned i = 0; i < _rc_in.channel_count; i++) {
		_rc_in.values[i] = raw_rc_values_local[i];

		if (raw_rc_values_local[i] != UINT16_MAX) {
			valid_chans++;
		}

		// once filled, reset values back to default
		_raw_rc_values[i] = UINT16_MAX;
	}

	_rc_in.timestamp = now;
	_rc_in.timestamp_last_signal = _rc_in.timestamp;
	_rc_in.rc_ppm_frame_length = 0;

	/* fake rssi if no value was provided */
	if (rssi == -1) {
		if ((_param_rc_rssi_pwm_chan.get() > 0) && (_param_rc_rssi_pwm_chan.get() < _rc_in.channel_count)) {
			const int32_t rssi_pwm_chan = _param_rc_rssi_pwm_chan.get();
			const int32_t rssi_pwm_min = _param_rc_rssi_pwm_min.get();
			const int32_t rssi_pwm_max = _param_rc_rssi_pwm_max.get();

			// get RSSI from input channel
			int rc_rssi = ((_rc_in.values[rssi_pwm_chan - 1] - rssi_pwm_min) * 100) / (rssi_pwm_max - rssi_pwm_min);
			_rc_in.rssi = math::constrain(rc_rssi, 0, 100);

		} else if (_analog_rc_rssi_stable) {
			// set RSSI if analog RSSI input is present
			float rssi_analog = ((_analog_rc_rssi_volt - 0.2f) / 3.0f) * 100.0f;

			if (rssi_analog > 100.0f) {
				rssi_analog = 100.0f;
			}

			if (rssi_analog < 0.0f) {
				rssi_analog = 0.0f;
			}

			_rc_in.rssi = rssi_analog;

		} else {
			_rc_in.rssi = 255;
		}

	} else {
		_rc_in.rssi = rssi;
	}

	if (valid_chans == 0) {
		_rc_in.rssi = 0;
	}

	_rc_in.rc_failsafe = failsafe;
	_rc_in.rc_lost = (valid_chans == 0);
	_rc_in.rc_lost_frame_count = frame_drops;
	_rc_in.rc_total_frame_count = 0;
}

void RCInput::set_rc_scan_state(RC_SCAN newState)
{
	PX4_DEBUG("RCscan: %s failed, trying %s", RCInput::RC_SCAN_STRING[_rc_scan_state], RCInput::RC_SCAN_STRING[newState]);
	_rc_scan_begin = 0;
	_rc_scan_state = newState;
	_rc_scan_locked = false;

	_report_lock = true;
}

void RCInput::rc_io_invert(bool invert)
{
	// First check if the board provides a board-specific inversion method (e.g. via GPIO),
	// and if not use an IOCTL
	if (!board_rc_invert_input(_device, invert)) {
#if defined(TIOCSINVERT)

		if (invert) {
			ioctl(_rcs_fd, TIOCSINVERT, SER_INVERT_ENABLED_RX | SER_INVERT_ENABLED_TX);

		} else {
			ioctl(_rcs_fd, TIOCSINVERT, 0);
		}

#endif // TIOCSINVERT
	}
}

void RCInput::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (!_initialized) {
		if (init() == PX4_OK) {
			_initialized = true;

		} else {
			PX4_ERR("init failed");
			exit_and_cleanup();
		}

	} else {

		perf_begin(_cycle_perf);

		// Check if parameters have changed
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s param_update;
			_parameter_update_sub.copy(&param_update);

			updateParams();
		}

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			}
		}

		const hrt_abstime cycle_timestamp = hrt_absolute_time();


		/* vehicle command */
		vehicle_command_s vcmd;

		if (_vehicle_cmd_sub.update(&vcmd)) {
			// Check for a pairing command
			if (vcmd.command == vehicle_command_s::VEHICLE_CMD_START_RX_PAIR) {

				uint8_t cmd_ret = vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
#if defined(SPEKTRUM_POWER)

				if (!_rc_scan_locked && !_armed) {
					if ((int)vcmd.param1 == 0) {
						// DSM binding command
						int dsm_bind_mode = (int)vcmd.param2;

						int dsm_bind_pulses = 0;

						if (dsm_bind_mode == 0) {
							dsm_bind_pulses = DSM2_BIND_PULSES;

						} else if (dsm_bind_mode == 1) {
							dsm_bind_pulses = DSMX_BIND_PULSES;

						} else {
							dsm_bind_pulses = DSMX8_BIND_PULSES;
						}

						bind_spektrum(dsm_bind_pulses);

						cmd_ret = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
					}

				} else {
					cmd_ret = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}

#endif // SPEKTRUM_POWER

				// publish acknowledgement
				vehicle_command_ack_s command_ack{};
				command_ack.command = vcmd.command;
				command_ack.result = cmd_ret;
				command_ack.target_system = vcmd.source_system;
				command_ack.target_component = vcmd.source_component;
				command_ack.timestamp = hrt_absolute_time();
				uORB::Publication<vehicle_command_ack_s> vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
				vehicle_command_ack_pub.publish(command_ack);
			}
		}


#if defined(ADC_RC_RSSI_CHANNEL)

		// update ADC sampling
		if (_adc_report_sub.updated()) {
			adc_report_s adc;

			if (_adc_report_sub.copy(&adc)) {
				for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
					if (adc.channel_id[i] == ADC_RC_RSSI_CHANNEL) {
						float adc_volt = adc.raw_data[i] *
								 adc.v_ref /
								 adc.resolution;

						if (_analog_rc_rssi_volt < 0.0f) {
							_analog_rc_rssi_volt = adc_volt;
						}

						_analog_rc_rssi_volt = _analog_rc_rssi_volt * 0.995f + adc_volt * 0.005f;

						/* only allow this to be used if we see a high RSSI once */
						if (_analog_rc_rssi_volt > 2.5f) {
							_analog_rc_rssi_stable = true;
						}
					}
				}
			}
		}

#endif // ADC_RC_RSSI_CHANNEL

		bool rc_updated = false;

		// This block scans for a supported serial RC input and locks onto the first one found
		// Scan for 300 msec, then switch protocol
		constexpr hrt_abstime rc_scan_max = 300_ms;

		unsigned frame_drops = 0;

		// TODO: needs work (poll _rcs_fd)
		// int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), 100);
		// then update priority to SCHED_PRIORITY_FAST_DRIVER
		// read all available data from the serial RC input UART

		// read all available data from the serial RC input UART
		int newBytes = ::read(_rcs_fd, &_rcs_buf[0], RC_MAX_BUFFER_SIZE);

		if (newBytes > 0) {
			_bytes_rx += newBytes;
		}

		switch (_rc_scan_state) {
		case RC_SCAN_SBUS:
			if (_rc_scan_begin == 0) {
				_rc_scan_begin = cycle_timestamp;
				// Configure serial port for SBUS
				sbus_config(_rcs_fd, board_rc_singlewire(_device));
				rc_io_invert(true);

				// flush serial buffer and any existing buffered data
				tcflush(_rcs_fd, TCIOFLUSH);
				memset(_rcs_buf, 0, sizeof(_rcs_buf));

			} else if (_rc_scan_locked
				   || cycle_timestamp - _rc_scan_begin < rc_scan_max) {

				// parse new data
				if (newBytes > 0) {
					bool sbus_failsafe = false;
					bool sbus_frame_drop = false;

					rc_updated = sbus_parse(cycle_timestamp, &_rcs_buf[0], newBytes, &_raw_rc_values[0], &_raw_rc_count, &sbus_failsafe,
								&sbus_frame_drop, &frame_drops, input_rc_s::RC_INPUT_MAX_CHANNELS);

					if (rc_updated) {
						// we have a new SBUS frame. Publish it.
						_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS;
						fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp,
							   sbus_frame_drop, sbus_failsafe, frame_drops);
						_rc_scan_locked = true;
					}
				}

			} else {
				// Scan the next protocol
				rc_io_invert(false);
				set_rc_scan_state(RC_SCAN_DSM);
			}

			break;

		case RC_SCAN_DSM:
			if (_rc_scan_begin == 0) {
				_rc_scan_begin = cycle_timestamp;
				// Configure serial port for DSM
				dsm_config(_rcs_fd);

				// flush serial buffer and any existing buffered data
				tcflush(_rcs_fd, TCIOFLUSH);
				memset(_rcs_buf, 0, sizeof(_rcs_buf));

			} else if (_rc_scan_locked
				   || cycle_timestamp - _rc_scan_begin < rc_scan_max) {

				if (newBytes > 0) {
					int8_t dsm_rssi = 0;
					bool dsm_11_bit = false;

					// parse new data
					rc_updated = dsm_parse(cycle_timestamp, &_rcs_buf[0], newBytes, &_raw_rc_values[0], &_raw_rc_count,
							       &dsm_11_bit, &frame_drops, &dsm_rssi, input_rc_s::RC_INPUT_MAX_CHANNELS);

					if (rc_updated) {
						// we have a new DSM frame. Publish it.
						_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_DSM;
						fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp,
							   false, false, frame_drops, dsm_rssi);
						_rc_scan_locked = true;
					}
				}

			} else {
				// Scan the next protocol
				set_rc_scan_state(RC_SCAN_ST24);
			}

			break;

		case RC_SCAN_ST24:
			if (_rc_scan_begin == 0) {
				_rc_scan_begin = cycle_timestamp;
				// Configure serial port for DSM
				dsm_config(_rcs_fd);

				// flush serial buffer and any existing buffered data
				tcflush(_rcs_fd, TCIOFLUSH);
				memset(_rcs_buf, 0, sizeof(_rcs_buf));

			} else if (_rc_scan_locked
				   || cycle_timestamp - _rc_scan_begin < rc_scan_max) {

				if (newBytes > 0) {
					// parse new data
					uint8_t st24_rssi, lost_count;

					rc_updated = false;

					for (unsigned i = 0; i < (unsigned)newBytes; i++) {
						/* set updated flag if one complete packet was parsed */
						st24_rssi = RC_INPUT_RSSI_MAX;
						rc_updated = (OK == st24_decode(_rcs_buf[i], &st24_rssi, &lost_count,
										&_raw_rc_count, _raw_rc_values, input_rc_s::RC_INPUT_MAX_CHANNELS));
					}

					// The st24 will keep outputting RC channels and RSSI even if RC has been lost.
					// The only way to detect RC loss is therefore to look at the lost_count.

					if (rc_updated) {
						if (lost_count == 0) {
							// we have a new ST24 frame. Publish it.
							_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_ST24;
							fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp,
								   false, false, frame_drops, st24_rssi);
							_rc_scan_locked = true;

						} else {
							// if the lost count > 0 means that there is an RC loss
							_rc_in.rc_lost = true;
						}
					}
				}

			} else {
				// Scan the next protocol
				set_rc_scan_state(RC_SCAN_SUMD);
			}

			break;

		case RC_SCAN_SUMD:
			if (_rc_scan_begin == 0) {
				_rc_scan_begin = cycle_timestamp;
				// Configure serial port for DSM
				dsm_config(_rcs_fd);

				// flush serial buffer and any existing buffered data
				tcflush(_rcs_fd, TCIOFLUSH);
				memset(_rcs_buf, 0, sizeof(_rcs_buf));

			} else if (_rc_scan_locked
				   || cycle_timestamp - _rc_scan_begin < rc_scan_max) {

				if (newBytes > 0) {
					// parse new data
					uint8_t sumd_rssi, rx_count;
					bool sumd_failsafe;

					rc_updated = false;

					for (unsigned i = 0; i < (unsigned)newBytes; i++) {
						/* set updated flag if one complete packet was parsed */
						sumd_rssi = RC_INPUT_RSSI_MAX;
						rc_updated = (OK == sumd_decode(_rcs_buf[i], &sumd_rssi, &rx_count,
										&_raw_rc_count, _raw_rc_values, input_rc_s::RC_INPUT_MAX_CHANNELS, &sumd_failsafe));
					}

					if (rc_updated) {
						// we have a new SUMD frame. Publish it.
						_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SUMD;
						fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp,
							   false, sumd_failsafe, frame_drops, sumd_rssi);
						_rc_scan_locked = true;
					}
				}

			} else {
				// Scan the next protocol
				set_rc_scan_state(RC_SCAN_PPM);
			}

			break;

		case RC_SCAN_PPM:
			// skip PPM if it's not supported
#ifdef HRT_PPM_CHANNEL
			if (_rc_scan_begin == 0) {
				_rc_scan_begin = cycle_timestamp;
				// Configure timer input pin for CPPM
				px4_arch_configgpio(GPIO_PPM_IN);

			} else if (_rc_scan_locked || cycle_timestamp - _rc_scan_begin < rc_scan_max) {

				// see if we have new PPM input data
				if ((ppm_last_valid_decode != _rc_in.timestamp_last_signal) && ppm_decoded_channels > 3) {
					// we have a new PPM frame. Publish it.
					rc_updated = true;
					_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;
					fill_rc_in(ppm_decoded_channels, ppm_buffer, cycle_timestamp, false, false, 0);
					_rc_scan_locked = true;
					_rc_in.rc_ppm_frame_length = ppm_frame_length;
					_rc_in.timestamp_last_signal = ppm_last_valid_decode;
				}

			} else {
				// disable CPPM input by mapping it away from the timer capture input
				px4_arch_unconfiggpio(GPIO_PPM_IN);
				// Scan the next protocol
				set_rc_scan_state(RC_SCAN_CRSF);
			}

#else   // skip PPM if it's not supported
			set_rc_scan_state(RC_SCAN_CRSF);

#endif  // HRT_PPM_CHANNEL

			break;

		case RC_SCAN_CRSF:
			if (_rc_scan_begin == 0) {
				_rc_scan_begin = cycle_timestamp;
				// Configure serial port for CRSF
				crsf_config(_rcs_fd);

				// flush serial buffer and any existing buffered data
				tcflush(_rcs_fd, TCIOFLUSH);
				memset(_rcs_buf, 0, sizeof(_rcs_buf));

			} else if (_rc_scan_locked
				   || cycle_timestamp - _rc_scan_begin < rc_scan_max) {

				// parse new data
				if (newBytes > 0) {
					rc_updated = crsf_parse(cycle_timestamp, &_rcs_buf[0], newBytes, &_raw_rc_values[0], &_raw_rc_count,
								input_rc_s::RC_INPUT_MAX_CHANNELS);

					if (rc_updated) {
						// we have a new CRSF frame. Publish it.
						_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_CRSF;
						fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp, false, false, 0);

						// on Pixhawk (-related) boards we cannot write to the RC UART
						// another option is to use a different UART port
#ifdef BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

						if (!_rc_scan_locked && !_crsf_telemetry) {
							_crsf_telemetry = new CRSFTelemetry(_rcs_fd);
						}

#endif /* BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT */

						_rc_scan_locked = true;

						if (_crsf_telemetry) {
							_crsf_telemetry->update(cycle_timestamp);
						}
					}
				}

			} else {
				// Scan the next protocol
				set_rc_scan_state(RC_SCAN_GHST);
			}

			break;

		case RC_SCAN_GHST:
			if (_rc_scan_begin == 0) {
				_rc_scan_begin = cycle_timestamp;
				// Configure serial port for GHST
				ghst_config(_rcs_fd);

				// flush serial buffer and any existing buffered data
				tcflush(_rcs_fd, TCIOFLUSH);
				memset(_rcs_buf, 0, sizeof(_rcs_buf));

			} else if (_rc_scan_locked
				   || cycle_timestamp - _rc_scan_begin < rc_scan_max) {

				// parse new data
				if (newBytes > 0) {
					int8_t ghst_rssi = -1;
					rc_updated = ghst_parse(cycle_timestamp, &_rcs_buf[0], newBytes, &_raw_rc_values[0], &ghst_rssi,
								&_raw_rc_count, input_rc_s::RC_INPUT_MAX_CHANNELS);

					if (rc_updated) {
						// we have a new GHST frame. Publish it.
						_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_GHST;
						fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp, false, false, 0, ghst_rssi);

						// ghst telemetry works on fmu-v5
						// on other Pixhawk (-related) boards we cannot write to the RC UART
						// another option is to use a different UART port
#ifdef BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

						if (!_rc_scan_locked && !_ghst_telemetry) {
							_ghst_telemetry = new GHSTTelemetry(_rcs_fd);
						}

#endif /* BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT */

						_rc_scan_locked = true;

						if (_ghst_telemetry) {
							_ghst_telemetry->update(cycle_timestamp);
						}
					}
				}

			} else {
				// Scan the next protocol
				set_rc_scan_state(RC_SCAN_SBUS);
			}

			break;
		}

		perf_end(_cycle_perf);

		if (rc_updated) {
			perf_count(_publish_interval_perf);

			_to_input_rc.publish(_rc_in);

		} else if (!rc_updated && !_armed && (hrt_elapsed_time(&_rc_in.timestamp_last_signal) > 1_s)) {
			_rc_scan_locked = false;
		}

		if (_report_lock && _rc_scan_locked) {
			_report_lock = false;
			PX4_INFO("RC scan: %s RC input locked", RC_SCAN_STRING[_rc_scan_state]);
		}
	}
}

#if defined(SPEKTRUM_POWER)
bool RCInput::bind_spektrum(int arg) const
{
	int ret = PX4_ERROR;

	/* specify 11ms DSMX. RX will automatically fall back to 22ms or DSM2 if necessary */

	/* only allow DSM2, DSM-X and DSM-X with more than 7 channels */
	PX4_INFO("DSM_BIND_START: DSM%s RX", (arg == 0) ? "2" : ((arg == 1) ? "-X" : "-X8"));

	if (arg == DSM2_BIND_PULSES ||
	    arg == DSMX_BIND_PULSES ||
	    arg == DSMX8_BIND_PULSES) {

		dsm_bind(DSM_CMD_BIND_POWER_DOWN, 0);

		dsm_bind(DSM_CMD_BIND_SET_RX_OUT, 0);
		usleep(500000);

		dsm_bind(DSM_CMD_BIND_POWER_UP, 0);
		usleep(72000);

		irqstate_t flags = px4_enter_critical_section();
		dsm_bind(DSM_CMD_BIND_SEND_PULSES, arg);
		px4_leave_critical_section(flags);

		usleep(50000);

		dsm_bind(DSM_CMD_BIND_REINIT_UART, 0);

		ret = OK;

	} else {
		PX4_ERR("DSM bind failed");
		ret = -EINVAL;
	}

	return (ret == PX4_OK);
}
#endif /* SPEKTRUM_POWER */

int RCInput::custom_command(int argc, char *argv[])
{
#if defined(SPEKTRUM_POWER)
	const char *verb = argv[0];

	if (!strcmp(verb, "bind")) {
		uORB::Publication<vehicle_command_s> vehicle_command_pub{ORB_ID(vehicle_command)};
		vehicle_command_s vcmd{};
		vcmd.command = vehicle_command_s::VEHICLE_CMD_START_RX_PAIR;
		vcmd.timestamp = hrt_absolute_time();
		vehicle_command_pub.publish(vcmd);
		return 0;
	}

#endif /* SPEKTRUM_POWER */

	/* start the FMU if not running */
	if (!is_running()) {
		int ret = RCInput::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int RCInput::print_status()
{
	PX4_INFO("Max update rate: %u Hz", 1000000 / _current_update_interval);

	if (_device[0] != '\0') {
		PX4_INFO("UART device: %s", _device);
		PX4_INFO("UART RX bytes: %"  PRIu32, _bytes_rx);
	}

	PX4_INFO("RC state: %s: %s", _rc_scan_locked ? "found" : "searching for signal", RC_SCAN_STRING[_rc_scan_state]);

	if (_rc_scan_locked) {
		switch (_rc_scan_state) {
		case RC_SCAN_CRSF:
			PX4_INFO("CRSF Telemetry: %s", _crsf_telemetry ? "yes" : "no");
			break;

		case RC_SCAN_GHST:
			PX4_INFO("GHST Telemetry: %s", _ghst_telemetry ? "yes" : "no");
			break;

		case RC_SCAN_SBUS:
			PX4_INFO("SBUS frame drops: %u", sbus_dropped_frames());
			break;


		case RC_SCAN_DSM:
			// DSM status output
#if defined(SPEKTRUM_POWER)
#endif
			break;

		case RC_SCAN_PPM:
			// PPM status output
			break;

		case RC_SCAN_SUMD:
			// SUMD status output
			break;

		case RC_SCAN_ST24:
			// SUMD status output
			break;
		}
	}

#if ADC_RC_RSSI_CHANNEL

	if (_analog_rc_rssi_stable) {
		PX4_INFO("vrssi: %dmV", (int)(_analog_rc_rssi_volt * 1000.0f));
	}

#endif

	perf_print_counter(_cycle_perf);
	perf_print_counter(_publish_interval_perf);

	if (hrt_elapsed_time(&_rc_in.timestamp) < 1_s) {
		print_message(_rc_in);
	}

	return 0;
}

int
RCInput::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module does the RC input parsing and auto-selecting the method. Supported methods are:
- PPM
- SBUS
- DSM
- SUMD
- ST24
- TBS Crossfire (CRSF)

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rc_input", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "RC device", true);

#if defined(SPEKTRUM_POWER)
	PRINT_MODULE_USAGE_COMMAND_DESCR("bind", "Send a DSM bind command (module must be running)");
#endif /* SPEKTRUM_POWER */

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rc_input_main(int argc, char *argv[])
{
	return RCInput::main(argc, argv);
}
