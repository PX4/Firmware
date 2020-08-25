/****************************************************************************
 *
 *   Copyright (C) 2012-2016 PX4 Development Team. All rights reserved.
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

/**
 * @file blinkm.cpp
 *
 * Driver for the BlinkM LED controller connected via I2C.
 *
 * Connect the BlinkM to I2C3 and put the following line to the rc startup-script:
 * blinkm start
 *
 * To start the system monitor put in the next line after the blinkm start:
 * blinkm systemmonitor
 *
 *
 * Description:
 * After startup, the Application checked how many lipo cells are connected to the System.
 * The recognized number off cells, will be blinked 5 times in purple color.
 * 2 Cells = 2 blinks
 * ...
 * 6 Cells = 6 blinks
 * Now the Application will show the actual selected Flightmode, GPS-Fix and Battery Warnings and Alerts.
 *
 * System disarmed and safe:
 * The BlinkM should light solid cyan.
 *
 * System safety off but not armed:
 * The BlinkM should light flashing orange
 *
 * System armed:
 * One message is made of 4 Blinks and a pause in the same length as the 4 blinks.
 *
 * X-X-X-X-_-_-_-_-_-_-
 * -------------------------
 * G G G M
 * P P P O
 * S S S D
 *       E
 *
 * (X = on, _=off)
 *
 * The first 3 blinks indicates the status of the GPS-Signal (red):
 * 0-4 satellites = X-X-X-X-X-_-_-_-_-_-
 *   5 satellites = X-X-_-X-X-_-_-_-_-_-
 *   6 satellites = X-_-_-X-X-_-_-_-_-_-
 * >=7 satellites = _-_-_-X-X-_-_-_-_-_-
 * If no GPS is found the first 3 blinks are white
 *
 * The fourth Blink indicates the Flightmode:
 * MANUAL     : amber
 * STABILIZED : yellow
 * HOLD		  : blue
 * AUTO       : green
 *
 * Battery Warning (low Battery Level):
 * Continuously blinking in yellow X-X-X-X-X-X-X-X-X-X
 *
 * Battery Alert (critical Battery Level)
 * Continuously blinking in red X-X-X-X-X-X-X-X-X-X
 *
 * General Error (no uOrb Data)
 * Continuously blinking in white X-X-X-X-X-X-X-X-X-X
 *
 */

#include <ctype.h>
#include <string.h>
#include <strings.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_blinkm.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>

static const int LED_ONTIME = 120;
static const int LED_OFFTIME = 120;
static const int LED_BLINK = 1;
static const int LED_NOBLINK = 0;

class BlinkM : public device::I2C, public I2CSPIDriver<BlinkM>
{
public:
	BlinkM(I2CSPIBusOption bus_option, const int bus, int bus_frequency, const int address);
	virtual ~BlinkM() = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void custom_method(const BusCLIArguments &cli) override;

	virtual int		init();
	virtual int		probe();
	virtual int		setMode(int mode);
	virtual int		ioctl(device::file_t *filp, int cmd, unsigned long arg);

	static const char	*const script_names[];

	void			RunImpl();
private:
	enum ScriptID {
		USER		= 0,
		RGB,
		WHITE_FLASH,
		RED_FLASH,
		GREEN_FLASH,
		BLUE_FLASH,
		CYAN_FLASH,
		MAGENTA_FLASH,
		YELLOW_FLASH,
		BLACK,
		HUE_CYCLE,
		MOOD_LIGHT,
		VIRTUAL_CANDLE,
		WATER_REFLECTIONS,
		OLD_NEON,
		THE_SEASONS,
		THUNDERSTORM,
		STOP_LIGHT,
		MORSE_CODE
	};

	enum ledColors {
		LED_OFF,
		LED_RED,
		LED_ORANGE,
		LED_YELLOW,
		LED_PURPLE,
		LED_GREEN,
		LED_BLUE,
		LED_CYAN,
		LED_WHITE,
		LED_AMBER
	};

	int led_color_1;
	int led_color_2;
	int led_color_3;
	int led_color_4;
	int led_color_5;
	int led_color_6;
	int led_color_7;
	int led_color_8;
	int led_blink;

	bool systemstate_run;

	uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription safety_sub{ORB_ID(safety)};

	int num_of_cells;
	int detected_cells_runcount;
	int t_led_color[8];
	int t_led_blink;
	int led_thread_runcount;
	int led_interval;

	bool detected_cells_blinked;
	bool led_thread_ready;

	int num_of_used_sats;

	void 			setLEDColor(int ledcolor);

	int			set_rgb(uint8_t r, uint8_t g, uint8_t b);

	int			fade_rgb(uint8_t r, uint8_t g, uint8_t b);
	int			fade_hsb(uint8_t h, uint8_t s, uint8_t b);

	int			fade_rgb_random(uint8_t r, uint8_t g, uint8_t b);
	int			fade_hsb_random(uint8_t h, uint8_t s, uint8_t b);

	int			set_fade_speed(uint8_t s);

	int			play_script(uint8_t script_id);
	int			play_script(const char *script_name);
	int			stop_script();

	int			write_script_line(uint8_t line, uint8_t ticks, uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3);
	int			read_script_line(uint8_t line, uint8_t &ticks, uint8_t cmd[4]);
	int			set_script(uint8_t length, uint8_t repeats);

	int			get_rgb(uint8_t &r, uint8_t &g, uint8_t &b);

	int			get_firmware_version(uint8_t version[2]);
};

/* list of script names, must match script ID numbers */
const char *const BlinkM::script_names[] = {
	"USER",
	"RGB",
	"WHITE_FLASH",
	"RED_FLASH",
	"GREEN_FLASH",
	"BLUE_FLASH",
	"CYAN_FLASH",
	"MAGENTA_FLASH",
	"YELLOW_FLASH",
	"BLACK",
	"HUE_CYCLE",
	"MOOD_LIGHT",
	"VIRTUAL_CANDLE",
	"WATER_REFLECTIONS",
	"OLD_NEON",
	"THE_SEASONS",
	"THUNDERSTORM",
	"STOP_LIGHT",
	"MORSE_CODE",
	nullptr
};


extern "C" __EXPORT int blinkm_main(int argc, char *argv[]);

BlinkM::BlinkM(I2CSPIBusOption bus_option, const int bus, int bus_frequency, const int address) :
	I2C("blinkm", BLINKM0_DEVICE_PATH, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	led_color_1(LED_OFF),
	led_color_2(LED_OFF),
	led_color_3(LED_OFF),
	led_color_4(LED_OFF),
	led_color_5(LED_OFF),
	led_color_6(LED_OFF),
	led_color_7(LED_OFF),
	led_color_8(LED_OFF),
	led_blink(LED_NOBLINK),
	systemstate_run(false),
	num_of_cells(0),
	detected_cells_runcount(0),
	t_led_color{0},
	t_led_blink(0),
	led_thread_runcount(0),
	led_interval(1000),
	detected_cells_blinked(false),
	led_thread_ready(true),
	num_of_used_sats(0)
{
}

int
BlinkM::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		warnx("I2C init failed");
		return ret;
	}

	stop_script();
	set_rgb(0, 0, 0);

	return OK;
}

int
BlinkM::setMode(int mode)
{
	if (mode == 1) {
		if (systemstate_run == false) {
			stop_script();
			set_rgb(0, 0, 0);
			systemstate_run = true;
			ScheduleNow();
		}

	} else {
		systemstate_run = false;
	}

	return OK;
}

int
BlinkM::probe()
{
	uint8_t version[2];
	int ret;

	ret = get_firmware_version(version);

	if (ret == OK) {
		DEVICE_DEBUG("found BlinkM firmware version %c%c", version[1], version[0]);
	}

	return ret;
}

int
BlinkM::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = ENOTTY;

	switch (cmd) {
	case BLINKM_PLAY_SCRIPT_NAMED:
		if (arg == 0) {
			ret = EINVAL;
			break;
		}

		ret = play_script((const char *)arg);
		break;

	case BLINKM_PLAY_SCRIPT:
		ret = play_script(arg);
		break;

	case BLINKM_SET_USER_SCRIPT: {
			if (arg == 0) {
				ret = EINVAL;
				break;
			}

			unsigned lines = 0;
			const uint8_t *script = (const uint8_t *)arg;

			while ((lines < 50) && (script[1] != 0)) {
				ret = write_script_line(lines, script[0], script[1], script[2], script[3], script[4]);

				if (ret != OK) {
					break;
				}

				script += 5;
			}

			if (ret == OK) {
				ret = set_script(lines, 0);
			}

			break;
		}

	default:
		break;
	}

	return ret;
}


void
BlinkM::RunImpl()
{
	if (led_thread_ready == true) {
		if (!detected_cells_blinked) {
			if (num_of_cells > 0) {
				t_led_color[0] = LED_PURPLE;
			}

			if (num_of_cells > 1) {
				t_led_color[1] = LED_PURPLE;
			}

			if (num_of_cells > 2) {
				t_led_color[2] = LED_PURPLE;
			}

			if (num_of_cells > 3) {
				t_led_color[3] = LED_PURPLE;
			}

			if (num_of_cells > 4) {
				t_led_color[4] = LED_PURPLE;
			}

			if (num_of_cells > 5) {
				t_led_color[5] = LED_PURPLE;
			}

			t_led_color[6] = LED_OFF;
			t_led_color[7] = LED_OFF;
			t_led_blink = LED_BLINK;

		} else {
			t_led_color[0] = led_color_1;
			t_led_color[1] = led_color_2;
			t_led_color[2] = led_color_3;
			t_led_color[3] = led_color_4;
			t_led_color[4] = led_color_5;
			t_led_color[5] = led_color_6;
			t_led_color[6] = led_color_7;
			t_led_color[7] = led_color_8;
			t_led_blink = led_blink;
		}

		led_thread_ready = false;
	}

	if (led_thread_runcount & 1) {
		if (t_led_blink) {
			setLEDColor(LED_OFF);
		}

		led_interval = LED_OFFTIME;

	} else {
		setLEDColor(t_led_color[(led_thread_runcount / 2) % 8]);
		//led_interval = (led_thread_runcount & 1) : LED_ONTIME;
		led_interval = LED_ONTIME;
	}

	if (led_thread_runcount == 15) {
		int no_data_vehicle_status = 0;
		int no_data_battery_status = 0;
		int no_data_vehicle_control_mode = 0;
		int no_data_actuator_armed = 0;
		int no_data_vehicle_gps_position = 0;


		vehicle_status_s vehicle_status_raw{};
		bool new_data_vehicle_status = vehicle_status_sub.updated();

		if (new_data_vehicle_status) {
			vehicle_status_sub.copy(&vehicle_status_raw);
			no_data_vehicle_status = 0;

		} else {
			no_data_vehicle_status++;

			if (no_data_vehicle_status >= 3) {
				no_data_vehicle_status = 3;
			}
		}


		battery_status_s battery_status{};
		bool new_data_battery_status = battery_status_sub.updated();

		if (new_data_battery_status) {
			battery_status_sub.copy(&battery_status);

		} else {
			no_data_battery_status++;

			if (no_data_battery_status >= 3) {
				no_data_battery_status = 3;
			}
		}


		vehicle_control_mode_s vehicle_control_mode{};
		bool new_data_vehicle_control_mode = vehicle_control_mode_sub.updated();

		if (new_data_vehicle_control_mode) {
			vehicle_control_mode_sub.copy(&vehicle_control_mode);
			no_data_vehicle_control_mode = 0;

		} else {
			no_data_vehicle_control_mode++;

			if (no_data_vehicle_control_mode >= 3) {
				no_data_vehicle_control_mode = 3;
			}
		}


		actuator_armed_s actuator_armed{};
		bool new_data_actuator_armed = actuator_armed_sub.updated();

		if (new_data_actuator_armed) {
			actuator_armed_sub.copy(&actuator_armed);
			no_data_actuator_armed = 0;

		} else {
			no_data_actuator_armed++;

			if (no_data_actuator_armed >= 3) {
				no_data_actuator_armed = 3;
			}
		}


		vehicle_gps_position_s vehicle_gps_position_raw{};
		bool new_data_vehicle_gps_position = vehicle_gps_position_sub.updated();

		if (new_data_vehicle_gps_position) {
			vehicle_gps_position_sub.copy(&vehicle_gps_position_raw);
			no_data_vehicle_gps_position = 0;

		} else {
			no_data_vehicle_gps_position++;

			if (no_data_vehicle_gps_position >= 3) {
				no_data_vehicle_gps_position = 3;
			}
		}


		/* update safety topic */
		safety_s safety{};
		bool new_data_safety = safety_sub.updated();

		if (new_data_safety) {
			safety_sub.copy(&safety);
		}

		/* get number of used satellites in navigation */
		num_of_used_sats = vehicle_gps_position_raw.satellites_used;

		if (new_data_battery_status || no_data_battery_status < 3) {
			if (num_of_cells == 0) {

				num_of_cells = battery_status.cell_count;

			} else {
				if (battery_status.warning == battery_status_s::BATTERY_WARNING_CRITICAL) {
					/* LED Pattern for battery critical alerting */
					led_color_1 = LED_RED;
					led_color_2 = LED_RED;
					led_color_3 = LED_RED;
					led_color_4 = LED_RED;
					led_color_5 = LED_RED;
					led_color_6 = LED_RED;
					led_color_7 = LED_RED;
					led_color_8 = LED_RED;
					led_blink = LED_BLINK;

				} else if (vehicle_status_raw.rc_signal_lost) {
					/* LED Pattern for FAILSAFE */
					led_color_1 = LED_BLUE;
					led_color_2 = LED_BLUE;
					led_color_3 = LED_BLUE;
					led_color_4 = LED_BLUE;
					led_color_5 = LED_BLUE;
					led_color_6 = LED_BLUE;
					led_color_7 = LED_BLUE;
					led_color_8 = LED_BLUE;
					led_blink = LED_BLINK;

				} else if (battery_status.warning == battery_status_s::BATTERY_WARNING_LOW) {
					/* LED Pattern for battery low warning */
					led_color_1 = LED_YELLOW;
					led_color_2 = LED_YELLOW;
					led_color_3 = LED_YELLOW;
					led_color_4 = LED_YELLOW;
					led_color_5 = LED_YELLOW;
					led_color_6 = LED_YELLOW;
					led_color_7 = LED_YELLOW;
					led_color_8 = LED_YELLOW;
					led_blink = LED_BLINK;

				} else {
					/* no battery warnings here */

					if (actuator_armed.armed == false) {
						/* system not armed */
						if (safety.safety_off) {
							led_color_1 = LED_ORANGE;
							led_color_2 = LED_ORANGE;
							led_color_3 = LED_ORANGE;
							led_color_4 = LED_ORANGE;
							led_color_5 = LED_ORANGE;
							led_color_6 = LED_ORANGE;
							led_color_7 = LED_ORANGE;
							led_color_8 = LED_ORANGE;
							led_blink = LED_BLINK;

						} else {
							led_color_1 = LED_CYAN;
							led_color_2 = LED_CYAN;
							led_color_3 = LED_CYAN;
							led_color_4 = LED_CYAN;
							led_color_5 = LED_CYAN;
							led_color_6 = LED_CYAN;
							led_color_7 = LED_CYAN;
							led_color_8 = LED_CYAN;
							led_blink = LED_NOBLINK;
						}

					} else {
						/* armed system - initial led pattern */
						led_color_1 = LED_RED;
						led_color_2 = LED_RED;
						led_color_3 = LED_RED;
						led_color_4 = LED_OFF;
						led_color_5 = LED_OFF;
						led_color_6 = LED_OFF;
						led_color_7 = LED_OFF;
						led_color_8 = LED_OFF;
						led_blink = LED_BLINK;

						if (new_data_vehicle_control_mode || no_data_vehicle_control_mode < 3) {

							/* indicate main control state */
							if (vehicle_control_mode.flag_control_auto_enabled) {
								led_color_4 = LED_BLUE;

							} else if (vehicle_control_mode.flag_control_position_enabled) {
								led_color_4 = LED_GREEN;

							} else if (vehicle_control_mode.flag_control_altitude_enabled) {
								led_color_4 = LED_YELLOW;

							} else if (vehicle_control_mode.flag_control_manual_enabled) {
								led_color_4 = LED_WHITE;

							} else {
								led_color_4 = LED_OFF;
							}

							led_color_5 = led_color_4;
						}

						if (new_data_vehicle_gps_position || no_data_vehicle_gps_position < 3) {
							/* handling used satus */
							if (num_of_used_sats >= 7) {
								led_color_1 = LED_OFF;
								led_color_2 = LED_OFF;
								led_color_3 = LED_OFF;

							} else if (num_of_used_sats == 6) {
								led_color_2 = LED_OFF;
								led_color_3 = LED_OFF;

							} else if (num_of_used_sats == 5) {
								led_color_3 = LED_OFF;
							}

						} else {
							/* no vehicle_gps_position data */
							led_color_1 = LED_WHITE;
							led_color_2 = LED_WHITE;
							led_color_3 = LED_WHITE;

						}

					}
				}
			}

		} else {
			/* LED Pattern for general Error - no vehicle_status can retrieved */
			led_color_1 = LED_WHITE;
			led_color_2 = LED_WHITE;
			led_color_3 = LED_WHITE;
			led_color_4 = LED_WHITE;
			led_color_5 = LED_WHITE;
			led_color_6 = LED_WHITE;
			led_color_7 = LED_WHITE;
			led_color_8 = LED_WHITE;
			led_blink = LED_BLINK;

		}

		/*
		printf( "<blinkm> Volt:%8.4f\tArmed:%4u\tMode:%4u\tCells:%4u\tNDVS:%4u\tNDSAT:%4u\tSats:%4u\tFix:%4u\tVisible:%4u\n",
		vehicle_status_raw.voltage_battery,
		vehicle_status_raw.flag_system_armed,
		vehicle_status_raw.flight_mode,
		num_of_cells,
		no_data_vehicle_status,
		no_data_vehicle_gps_position,
		num_of_used_sats,
		vehicle_gps_position_raw.fix_type,
		vehicle_gps_position_raw.satellites_visible);
		*/

		led_thread_runcount = 0;
		led_thread_ready = true;
		led_interval = LED_OFFTIME;

		if (detected_cells_runcount < 4) {
			detected_cells_runcount++;

		} else {
			detected_cells_blinked = true;
		}

	} else {
		led_thread_runcount++;
	}

	if (systemstate_run == true) {
		/* re-queue ourselves to run again later */
		ScheduleDelayed(led_interval);

	} else {
		stop_script();
		set_rgb(0, 0, 0);
	}
}

void BlinkM::setLEDColor(int ledcolor)
{
	switch (ledcolor) {
	case LED_OFF:	// off
		set_rgb(0, 0, 0);
		break;

	case LED_RED:	// red
		set_rgb(255, 0, 0);
		break;

	case LED_ORANGE:	// orange
		set_rgb(255, 150, 0);
		break;

	case LED_YELLOW:	// yellow
		set_rgb(200, 200, 0);
		break;

	case LED_PURPLE:	// purple
		set_rgb(255, 0, 255);
		break;

	case LED_GREEN:	// green
		set_rgb(0, 255, 0);
		break;

	case LED_BLUE:	// blue
		set_rgb(0, 0, 255);
		break;

	case LED_CYAN:	// cyan
		set_rgb(0, 128, 128);
		break;

	case LED_WHITE:	// white
		set_rgb(255, 255, 255);
		break;

	case LED_AMBER:	// amber
		set_rgb(255, 65, 0);
		break;
	}
}

int
BlinkM::set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
	const uint8_t msg[4] = { 'n', r, g, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::fade_rgb(uint8_t r, uint8_t g, uint8_t b)
{
	const uint8_t msg[4] = { 'c', r, g, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::fade_hsb(uint8_t h, uint8_t s, uint8_t b)
{
	const uint8_t msg[4] = { 'h', h, s, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::fade_rgb_random(uint8_t r, uint8_t g, uint8_t b)
{
	const uint8_t msg[4] = { 'C', r, g, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::fade_hsb_random(uint8_t h, uint8_t s, uint8_t b)
{
	const uint8_t msg[4] = { 'H', h, s, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::set_fade_speed(uint8_t s)
{
	const uint8_t msg[2] = { 'f', s };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::play_script(uint8_t script_id)
{
	const uint8_t msg[4] = { 'p', script_id, 0, 0 };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::play_script(const char *script_name)
{
	/* handle HTML colour encoding */
	if (isxdigit(script_name[0]) && (strlen(script_name) == 6)) {
		char code[3];
		uint8_t r, g, b;

		code[2] = '\0';

		code[0] = script_name[1];
		code[1] = script_name[2];
		r = strtol(code, 0, 16);
		code[0] = script_name[3];
		code[1] = script_name[4];
		g = strtol(code, 0, 16);
		code[0] = script_name[5];
		code[1] = script_name[6];
		b = strtol(code, 0, 16);

		stop_script();
		return set_rgb(r, g, b);
	}

	for (unsigned i = 0; script_names[i] != nullptr; i++)
		if (!strcasecmp(script_name, script_names[i])) {
			return play_script(i);
		}

	return -1;
}

int
BlinkM::stop_script()
{
	const uint8_t msg[1] = { 'o' };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::write_script_line(uint8_t line, uint8_t ticks, uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3)
{
	const uint8_t msg[8] = { 'W', 0, line, ticks, cmd, arg1, arg2, arg3 };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::read_script_line(uint8_t line, uint8_t &ticks, uint8_t cmd[4])
{
	const uint8_t msg[3] = { 'R', 0, line };
	uint8_t result[5];

	int ret = transfer(msg, sizeof(msg), result, sizeof(result));

	if (ret == OK) {
		ticks = result[0];
		cmd[0] = result[1];
		cmd[1] = result[2];
		cmd[2] = result[3];
		cmd[3] = result[4];
	}

	return ret;
}

int
BlinkM::set_script(uint8_t len, uint8_t repeats)
{
	const uint8_t msg[4] = { 'L', 0, len, repeats };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::get_rgb(uint8_t &r, uint8_t &g, uint8_t &b)
{
	const uint8_t msg = 'g';
	uint8_t result[3];

	int ret = transfer(&msg, sizeof(msg), result, sizeof(result));

	if (ret == OK) {
		r = result[0];
		g = result[1];
		b = result[2];
	}

	return ret;
}

int
BlinkM::get_firmware_version(uint8_t version[2])
{
	const uint8_t msg = 'Z';

	return transfer(&msg, sizeof(msg), version, 2);
}

void
BlinkM::print_usage()
{
	PRINT_MODULE_USAGE_NAME("blinkm", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(9);
	PRINT_MODULE_USAGE_COMMAND("systemstate");
	PRINT_MODULE_USAGE_COMMAND("ledoff");
	PRINT_MODULE_USAGE_COMMAND("list");
	PRINT_MODULE_USAGE_COMMAND("script");
	PRINT_MODULE_USAGE_PARAM_STRING('n', nullptr, "<file>", "Script file name", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *BlinkM::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	BlinkM *instance = new BlinkM(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.i2c_address);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}

void
BlinkM::custom_method(const BusCLIArguments &cli)
{
	setMode(cli.custom1);
}

extern "C" __EXPORT int blinkm_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = BlinkM;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = 9;
	const char *script = nullptr;

	while ((ch = cli.getopt(argc, argv, "n:")) != EOF) {
		switch (ch) {
		case 'n':
			script = cli.optarg();
			break;
		}
	}

	const char *verb = cli.optarg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_LED_DEVTYPE_BLINKM);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "systemstate")) {
		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "ledoff")) {
		cli.custom1 = 0;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "list")) {
		for (unsigned i = 0; BlinkM::script_names[i] != nullptr; i++) {
			PX4_INFO("    %s", BlinkM::script_names[i]);
		}

		PX4_INFO("    <html color number>");
		return 0;
	}

	if (!strcmp(verb, "script") && script) {
		/* things that require access to the device */
		int fd = px4_open(BLINKM0_DEVICE_PATH, 0);

		if (fd < 0) {
			PX4_ERR("can't open BlinkM device");
			return 1;
		}

		px4_ioctl(fd, BLINKM_PLAY_SCRIPT_NAMED, (unsigned long)script);

		px4_close(fd);
		return 0;
	}

	ThisDriver::print_usage();
	return -1;
}
