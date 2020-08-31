/****************************************************************************
 *
 *   Copyright (C) 2012-2015 PX4 Development Team. All rights reserved.
 *   Author: Marco Bauer <marco@wtns.de>
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
 * @file mkblctrl.cpp
 *
 * Driver/configurator for the Mikrokopter BL-Ctrl.
 *
 * @author Marco Bauer <marco@wtns.de>
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <drivers/device/i2c.h>
#include <parameters/param.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>

#include <board_config.h>

#include <drivers/device/device.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_tone_alarm.h>

#include <systemlib/err.h>
#include <lib/mixer/MixerGroup.hpp>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/tune_control.h>

#include <systemlib/err.h>

#define I2C_BUS_SPEED					100000
#define UPDATE_RATE						200
#define MAX_MOTORS 						8
#define BLCTRL_BASE_ADDR 				0x29
#define BLCTRL_OLD 						0
#define BLCTRL_NEW 						1
#define BLCTRL_MIN_VALUE				-0.920F
#define MOTOR_STATE_PRESENT_MASK		0x80
#define MOTOR_STATE_ERROR_MASK			0x7F
#define MOTOR_SPINUP_COUNTER			30
#define MOTOR_LOCATE_DELAY				10000000
#define ESC_UORB_PUBLISH_DELAY			500000

#define CONTROL_INPUT_DROP_LIMIT_MS		20
#define RC_MIN_VALUE					1010
#define RC_MAX_VALUE					2100


struct MotorData_t {
	unsigned int Version;                        // the version of the BL (0 = old)
	unsigned int SetPoint;                       // written by attitude controller
	unsigned int SetPointLowerBits;      // for higher Resolution of new BLs
	float SetPoint_PX4; 			     // Values from PX4
	unsigned int State;                          // 7 bit for I2C error counter, highest bit indicates if motor is present
	unsigned int ReadMode;                       // select data to read
	unsigned short RawPwmValue;							// length of PWM pulse
	// the following bytes must be exactly in that order!
	unsigned int Current;                        // in 0.1 A steps, read back from BL
	unsigned int MaxPWM;                         // read back from BL is less than 255 if BL is in current limit
	unsigned int Temperature;            // old BL-Ctrl will return a 255 here, the new version the temp. in
	unsigned int RoundCount;
};




class MK : public device::I2C
{
public:
	enum MappingMode {
		MAPPING_MK = 0,
		MAPPING_PX4,
	};

	enum FrameType {
		FRAME_PLUS = 0,
		FRAME_X,
	};

	MK(int bus, const char *_device_path);
	~MK();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);
	virtual int	init(unsigned motors);
	virtual ssize_t	write(file *filp, const char *buffer, size_t len);

	int		set_motor_count(unsigned count);
	int		set_motor_test(bool motortest);
	int		set_overrideSecurityChecks(bool overrideSecurityChecks);
	void		set_px4mode(int px4mode);
	void		set_frametype(int frametype);
	unsigned int		mk_check_for_blctrl(unsigned int count, bool showOutput, bool initI2C);
	void		set_rc_min_value(unsigned value);
	void		set_rc_max_value(unsigned value);

private:
	static const unsigned	_max_actuators = MAX_MOTORS;
	static const bool		showDebug = false;

	int 					_update_rate;
	int						_task;
	int						_t_actuators;
	int						_t_actuator_armed;
	unsigned int			_motor;
	int    					_px4mode;
	int    					_frametype;
	char					_device[20];
	orb_advert_t			_t_outputs;
	orb_advert_t			_t_esc_status;
	orb_advert_t			_tune_control_sub;
	unsigned int			_num_outputs;
	bool					_primary_pwm_device;
	bool     				_motortest;
	bool 					_overrideSecurityChecks;
	volatile bool			_task_should_exit;
	bool					_armed;
	unsigned long			debugCounter;
	MixerGroup				*_mixers;
	bool					_indicate_esc;
	unsigned				_rc_min_value;
	unsigned				_rc_max_value;
	param_t					_param_indicate_esc;
	actuator_controls_s 	_controls;
	MotorData_t 			Motor[MAX_MOTORS];

	static int				task_main_trampoline(int argc, char *argv[]);
	int					task_main();

	static int				control_callback(uintptr_t handle,
			uint8_t control_group,
			uint8_t control_index,
			float &input);

	int						pwm_ioctl(file *filp, int cmd, unsigned long arg);
	int						mk_servo_arm(bool status);
	int 					mk_servo_set(unsigned int chan, short val);
	int 					mk_servo_test(unsigned int chan);
	int 					mk_servo_locate();
	short					scaling(float val, float inMin, float inMax, float outMin, float outMax);
	void					play_beep(int count);

};




const int blctrlAddr_quad_plus[] = { 2, 2, -2, -2, 0, 0, 0, 0 };	// Addresstranslator for Quad + configuration
const int blctrlAddr_hexa_plus[] = { 0, 2, 2, -2, 1, -3, 0, 0 };	// Addresstranslator for Hexa + configuration
const int blctrlAddr_octo_plus[] = { 0, 3, -1, 0, 3, 0, 0, -5 };	// Addresstranslator for Octo + configuration

const int blctrlAddr_quad_x[] = { 2, 2, -2, -2, 0, 0, 0, 0 };		// Addresstranslator for Quad X configuration
const int blctrlAddr_hexa_x[] = { 2, 4, -2, 0, -3, -1, 0, 0 };		// Addresstranslator for Hexa X configuration
const int blctrlAddr_octo_x[] = { 1, 4, 0, 1, -4, 1, 1, -4 };		// Addresstranslator for Octo X configuration

const int blctrlAddr_px4[]  = { 0, 0, 0, 0, 0, 0, 0, 0};			// Native PX4 order - nothing to translate

int addrTranslator[] = {0, 0, 0, 0, 0, 0, 0, 0};					// work copy

namespace
{

MK	*g_mk;

} // namespace

MK::MK(int bus, const char *_device_path) :
	I2C(0, "mkblctrl", bus, 0, I2C_BUS_SPEED),
	_update_rate(UPDATE_RATE),
	_task(-1),
	_t_actuators(-1),
	_t_actuator_armed(-1),
	_motor(-1),
	_px4mode(MAPPING_MK),
	_frametype(FRAME_PLUS),
	_t_outputs(0),
	_t_esc_status(0),
	_num_outputs(0),
	_primary_pwm_device(false),
	_motortest(false),
	_overrideSecurityChecks(false),
	_task_should_exit(false),
	_armed(false),
	_mixers(nullptr),
	_indicate_esc(false),
	_rc_min_value(RC_MIN_VALUE),
	_rc_max_value(RC_MAX_VALUE)
{
	strncpy(_device, _device_path, sizeof(_device));
	/* enforce null termination */
	_device[sizeof(_device) - 1] = '\0';
}

MK::~MK()
{
	if (_task != -1) {
		/* tell the task we want it to go away */
		_task_should_exit = true;

		unsigned i = 10;

		do {
			/* wait 50ms - it should wake every 100ms or so worst-case */
			usleep(50000);

			/* if we have given up, kill it */
			if (--i == 0) {
				task_delete(_task);
				break;
			}

		} while (_task != -1);
	}

	/* clean up the alternate device node */
	if (_primary_pwm_device) {
		unregister_driver(_device);
	}

	g_mk = nullptr;
}

int
MK::init(unsigned motors)
{
	_param_indicate_esc	= param_find("MKBLCTRL_TEST");

	_num_outputs = motors;
	debugCounter = 0;
	int ret;
	ASSERT(_task == -1);

	ret = I2C::init();

	if (ret != OK) {
		warnx("I2C init failed");
		return ret;
	}

	usleep(500000);

	if (sizeof(_device) > 0) {
		ret = register_driver(_device, &fops, 0666, (void *)this);

		if (ret == OK) {
			DEVICE_LOG("creating alternate output device");
			_primary_pwm_device = true;
		}

	}

	/* start the IO interface task */
	_task = px4_task_spawn_cmd("mkblctrl",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_MAX - 20,
				   1500,
				   (main_t)&MK::task_main_trampoline,
				   nullptr);


	if (_task < 0) {
		DEVICE_DEBUG("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

int
MK::task_main_trampoline(int argc, char *argv[])
{
	return g_mk->task_main();
}

void
MK::set_px4mode(int px4mode)
{
	_px4mode = px4mode;
}

void
MK::set_frametype(int frametype)
{
	_frametype = frametype;
}

void
MK::set_rc_min_value(unsigned value)
{
	_rc_min_value = value;
	PX4_INFO("rc_min = %i", _rc_min_value);
}

void
MK::set_rc_max_value(unsigned value)
{
	_rc_max_value = value;
	PX4_INFO("rc_max = %i", _rc_max_value);
}

int
MK::set_motor_count(unsigned count)
{
	if (count > 0) {

		_num_outputs = count;

		if (_px4mode == MAPPING_MK) {
			if (_frametype == FRAME_PLUS) {
				PX4_INFO("Mikrokopter ESC addressing. Frame: +");

			} else if (_frametype == FRAME_X) {
				PX4_INFO("Mikrokopter ESC addressing. Frame: X");
			}

			if (_num_outputs == 4) {
				if (_frametype == FRAME_PLUS) {
					memcpy(&addrTranslator, &blctrlAddr_quad_plus, sizeof(blctrlAddr_quad_plus));

				} else if (_frametype == FRAME_X) {
					memcpy(&addrTranslator, &blctrlAddr_quad_x, sizeof(blctrlAddr_quad_x));
				}

			} else if (_num_outputs == 6) {
				if (_frametype == FRAME_PLUS) {
					memcpy(&addrTranslator, &blctrlAddr_hexa_plus, sizeof(blctrlAddr_hexa_plus));

				} else if (_frametype == FRAME_X) {
					memcpy(&addrTranslator, &blctrlAddr_hexa_x, sizeof(blctrlAddr_hexa_x));
				}

			} else if (_num_outputs == 8) {
				if (_frametype == FRAME_PLUS) {
					memcpy(&addrTranslator, &blctrlAddr_octo_plus, sizeof(blctrlAddr_octo_plus));

				} else if (_frametype == FRAME_X) {
					memcpy(&addrTranslator, &blctrlAddr_octo_x, sizeof(blctrlAddr_octo_x));
				}
			}

		} else {
			PX4_INFO("PX4 ESC addressing.");
			memcpy(&addrTranslator, &blctrlAddr_px4, sizeof(blctrlAddr_px4));
		}

		if (_num_outputs == 4) {
			PX4_INFO("4 ESCs = Quadrocopter");

		} else 	if (_num_outputs == 6) {
			PX4_INFO("6 ESCs = Hexacopter");

		} else 	if (_num_outputs == 8) {
			PX4_INFO("8 ESCs = Octocopter");
		}

		return OK;

	} else {
		return -1;
	}

}

int
MK::set_motor_test(bool motortest)
{
	_motortest = motortest;
	return OK;
}

int
MK::set_overrideSecurityChecks(bool overrideSecurityChecks)
{
	_overrideSecurityChecks = overrideSecurityChecks;
	return OK;
}

short
MK::scaling(float val, float inMin, float inMax, float outMin, float outMax)
{
	short retVal = 0;

	retVal = (val - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;

	if (retVal < outMin) {
		retVal = outMin;

	} else if (retVal > outMax) {
		retVal = outMax;
	}

	return retVal;
}

void
MK::play_beep(int count)
{
	tune_control_s tune = {};
	tune.tune_id = static_cast<int>(TuneID::SINGLE_BEEP);

	for (int i = 0; i < count; i++) {
		orb_publish(ORB_ID(tune_control), _tune_control_sub, &tune);
		usleep(300000);
	}

}

int
MK::task_main()
{
	int32_t param_mkblctrl_test = 0;
	/*
	 * Subscribe to the appropriate PWM output topic based on whether we are the
	 * primary PWM output or not.
	 */
	_t_actuators = orb_subscribe(ORB_ID(actuator_controls_0));
	orb_set_interval(_t_actuators, int(1000 / _update_rate));	/* set the topic update rate (200Hz)*/

	/*
	 * Subscribe to actuator_armed topic.
	 */
	_t_actuator_armed = orb_subscribe(ORB_ID(actuator_armed));
	orb_set_interval(_t_actuator_armed, 200);		/* 5Hz update rate */

	/*
	 * advertise the mixed control outputs.
	 */
	actuator_outputs_s outputs;
	memset(&outputs, 0, sizeof(outputs));
	int dummy;
	_t_outputs = orb_advertise_multi(ORB_ID(actuator_outputs), &outputs, &dummy);

	/*
	 * advertise the blctrl status.
	 */
	esc_status_s esc;
	memset(&esc, 0, sizeof(esc));
	_t_esc_status = orb_advertise(ORB_ID(esc_status), &esc);

	/*
	 * advertise the tune_control.
	 */
	tune_control_s tune = {};
	_tune_control_sub = orb_advertise_queue(ORB_ID(tune_control), &tune, tune_control_s::ORB_QUEUE_LENGTH);

	pollfd fds[2];
	fds[0].fd = _t_actuators;
	fds[0].events = POLLIN;
	fds[1].fd = _t_actuator_armed;
	fds[1].events = POLLIN;

	up_pwm_servo_set_rate(_update_rate);	/* unnecessary ? */

	DEVICE_LOG("starting");

	/* loop until killed */
	while (!_task_should_exit) {

		param_get(_param_indicate_esc, &param_mkblctrl_test);

		if (param_mkblctrl_test > 0) {
			_indicate_esc = true;

		} else {
			_indicate_esc = false;
		}

		/* waiting for data */
		int ret = ::poll(&fds[0], 2, CONTROL_INPUT_DROP_LIMIT_MS);

		/* this would be bad... */
		if (ret < 0) {
			DEVICE_LOG("poll error %d", errno);
			usleep(1000000);
			continue;
		}

		/* do we have a control update? */
		if (fds[0].revents & POLLIN) {

			bool changed = false;
			orb_check(_t_actuators, &changed);

			if (changed) {

				/* get controls - must always do this to avoid spinning */
				orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, _t_actuators, &_controls);

				/* can we mix? */
				if (_mixers != nullptr) {

					/* do mixing */
					outputs.noutputs = _mixers->mix(&outputs.output[0], _num_outputs);
					outputs.timestamp = hrt_absolute_time();

					/* iterate actuators */
					for (unsigned int i = 0; i < _num_outputs; i++) {
						/* last resort: catch NaN, INF and out-of-band errors */
						if (i < outputs.noutputs &&
						    PX4_ISFINITE(outputs.output[i]) &&
						    outputs.output[i] >= -1.0f &&
						    outputs.output[i] <= 1.0f) {
							/* scale for PWM output 900 - 2100us */
							/* nothing to do here */
						} else {
							/*
							 * Value is NaN, INF or out of band - set to the minimum value.
							 * This will be clearly visible on the servo status and will limit the risk of accidentally
							 * spinning motors. It would be deadly in flight.
							 */
							if (outputs.output[i] < -1.0f) {
								outputs.output[i] = -1.0f;

							} else if (outputs.output[i] > 1.0f) {
								outputs.output[i] = 1.0f;

							} else {
								outputs.output[i] = -1.0f;
							}
						}

						if (!_overrideSecurityChecks) {
							/* don't go under BLCTRL_MIN_VALUE */
							if (outputs.output[i] < BLCTRL_MIN_VALUE) {
								outputs.output[i] = BLCTRL_MIN_VALUE;
							}
						}

						/* output to BLCtrl's */
						if (_motortest != true && _indicate_esc != true) {
							Motor[i].SetPoint_PX4 = outputs.output[i];
							mk_servo_set(i, scaling(outputs.output[i], -1.0f, 1.0f, 0,
										2047));	// scale the output to 0 - 2047 and sent to output routine
						}
					}
				}
			}
		}

		/* how about an arming update? */
		if (fds[1].revents & POLLIN) {
			actuator_armed_s aa;

			/* get new value */
			orb_copy(ORB_ID(actuator_armed), _t_actuator_armed, &aa);

			/* update PWM servo armed status if armed and not locked down */
			mk_servo_arm(aa.armed && !aa.lockdown);
		}

		/*
		 * Only update esc topic every half second.
		 */

		if (hrt_absolute_time() - esc.timestamp > ESC_UORB_PUBLISH_DELAY) {
			esc.counter++;
			esc.timestamp = hrt_absolute_time();
			esc.esc_count = (uint8_t) _num_outputs;
			esc.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_I2C;
			esc.esc_online_flags = (1 << esc.esc_count) - 1;
			esc.esc_armed_flags = (1 << esc.esc_count) - 1;

			for (unsigned int i = 0; i < _num_outputs; i++) {
				esc.esc[i].esc_address = (uint8_t) BLCTRL_BASE_ADDR + i;
				esc.esc[i].esc_voltage = 0.0F;
				esc.esc[i].esc_current = static_cast<float>(Motor[i].Current) * 0.1F;
				esc.esc[i].esc_rpm = (uint16_t) 0;

				esc.esc[i].esc_temperature = static_cast<uint8_t>(Motor[i].Temperature);
				esc.esc[i].esc_state = (uint8_t) Motor[i].State;
				esc.esc[i].esc_errorcount = (uint16_t) 0;

				// if motortest is requested - do it... (deprecated in future)
				if (_motortest == true) {
					mk_servo_test(i);
				}

				// if esc locate is requested
				if (_indicate_esc == true) {
					mk_servo_locate();
				}
			}

			orb_publish(ORB_ID(esc_status), _t_esc_status, &esc);

		}

	}

	::close(_t_actuators);
	::close(_t_actuator_armed);


	/* make sure servos are off */
	up_pwm_servo_deinit();

	DEVICE_LOG("stopping");

	/* note - someone else is responsible for restoring the GPIO config */

	/* tell the dtor that we are exiting */
	_task = -1;
	return 0;
}


int
MK::mk_servo_arm(bool status)
{
	_armed = status;
	return 0;
}


unsigned int
MK::mk_check_for_blctrl(unsigned int count, bool showOutput, bool initI2C)
{
	if (initI2C) {
		I2C::init();
	}

	_retries = 50;
	uint8_t foundMotorCount = 0;

	for (unsigned i = 0; i < MAX_MOTORS; i++) {
		Motor[i].Version = 0;
		Motor[i].SetPoint = 0;
		Motor[i].SetPointLowerBits = 0;
		Motor[i].State = 0;
		Motor[i].ReadMode = 0;
		Motor[i].RawPwmValue = 0;
		Motor[i].Current = 0;
		Motor[i].MaxPWM = 0;
		Motor[i].Temperature = 0;
		Motor[i].RoundCount = 0;
	}

	uint8_t msg = 0;
	uint8_t result[3];

	for (unsigned i = 0; i < count; i++) {
		result[0] = 0;
		result[1] = 0;
		result[2] = 0;

		set_device_address(BLCTRL_BASE_ADDR + i);

		if (OK == transfer(&msg, 1, &result[0], 3)) {
			Motor[i].Current = result[0];
			Motor[i].MaxPWM = result[1];
			Motor[i].Temperature = result[2];
			Motor[i].State |= MOTOR_STATE_PRESENT_MASK; // set present bit;
			foundMotorCount++;

			if ((Motor[i].MaxPWM & 252) == 248) {
				Motor[i].Version = BLCTRL_NEW;

			} else {
				Motor[i].Version = BLCTRL_OLD;
			}
		}
	}

	if (showOutput) {
		PX4_INFO("MotorsFound: %i", foundMotorCount);

		for (unsigned i = 0; i < foundMotorCount; i++) {
			PX4_INFO("blctrl[%i] : found=%i\tversion=%i\tcurrent=%i\tmaxpwm=%i\ttemperature=%i", i,
				 Motor[i].State, Motor[i].Version, Motor[i].Current, Motor[i].MaxPWM, Motor[i].Temperature);
		}


		if (!_overrideSecurityChecks) {
			if (foundMotorCount != 4 && foundMotorCount != 6 && foundMotorCount != 8) {
				_task_should_exit = true;
			}
		}
	}

	return foundMotorCount;
}

int
MK::mk_servo_set(unsigned int chan, short val)
{
	short tmpVal = 0;
	_retries = 0;
	uint8_t result[3] = { 0, 0, 0 };
	uint8_t msg[2] = { 0, 0 };
	uint8_t bytesToSendBL2 = 2;

	tmpVal = val;

	if (tmpVal > 2047) {
		tmpVal = 2047;

	} else if (tmpVal < 0) {
		tmpVal = 0;
	}

	Motor[chan].SetPoint = (uint8_t)(tmpVal >> 3) & 0xff;
	Motor[chan].SetPointLowerBits = ((uint8_t)tmpVal % 8) & 0x07;

	if (_armed == false) {
		Motor[chan].SetPoint = 0;
		Motor[chan].SetPointLowerBits = 0;
	}

	//if(Motor[chan].State & MOTOR_STATE_PRESENT_MASK) {
	set_device_address(BLCTRL_BASE_ADDR + (chan + addrTranslator[chan]));

	if (Motor[chan].Version == BLCTRL_OLD) {
		/*
		*	Old BL-Ctrl 8Bit served. Version < 2.0
		*/
		msg[0] = Motor[chan].SetPoint;

		if (Motor[chan].RoundCount >= 16) {
			// on each 16th cyle we read out the status messages from the blctrl
			if (OK == transfer(&msg[0], 1, &result[0], 2)) {
				Motor[chan].Current = result[0];
				Motor[chan].MaxPWM = result[1];
				Motor[chan].Temperature = 255;

			} else {
				if ((Motor[chan].State & MOTOR_STATE_ERROR_MASK) < MOTOR_STATE_ERROR_MASK) { Motor[chan].State++; }	// error
			}

			Motor[chan].RoundCount = 0;

		} else {
			if (OK != transfer(&msg[0], 1, nullptr, 0)) {
				if ((Motor[chan].State & MOTOR_STATE_ERROR_MASK) < MOTOR_STATE_ERROR_MASK) { Motor[chan].State++; }	// error
			}
		}

	} else {
		/*
		*	New BL-Ctrl 11Bit served. Version >= 2.0
		*/
		msg[0] = Motor[chan].SetPoint;
		msg[1] = Motor[chan].SetPointLowerBits;

		if (Motor[chan].SetPointLowerBits == 0) {
			bytesToSendBL2 = 1;	// if setpoint lower bits are zero, we send only the higher bits - this saves time
		}

		if (Motor[chan].RoundCount >= 16) {
			// on each 16th cyle we read out the status messages from the blctrl
			if (OK == transfer(&msg[0], bytesToSendBL2, &result[0], 3)) {
				Motor[chan].Current = result[0];
				Motor[chan].MaxPWM = result[1];
				Motor[chan].Temperature = result[2];

			} else {
				if ((Motor[chan].State & MOTOR_STATE_ERROR_MASK) < MOTOR_STATE_ERROR_MASK) { Motor[chan].State++; }	// error
			}

			Motor[chan].RoundCount = 0;

		} else {
			if (OK != transfer(&msg[0], bytesToSendBL2, nullptr, 0)) {
				if ((Motor[chan].State & MOTOR_STATE_ERROR_MASK) < MOTOR_STATE_ERROR_MASK) { Motor[chan].State++; }	// error
			}
		}

	}

	Motor[chan].RoundCount++;
	//}

	if (showDebug == true) {
		debugCounter++;

		if (debugCounter == 2000) {
			debugCounter = 0;

			for (unsigned int i = 0; i < _num_outputs; i++) {
				if (Motor[i].State & MOTOR_STATE_PRESENT_MASK) {
					PX4_INFO("#%i:\tVer: %i\tVal: %i\tCurr: %i\tMaxPWM: %i\tTemp: %i\tState: %i", i, Motor[i].Version,
						 Motor[i].SetPoint, Motor[i].Current, Motor[i].MaxPWM, Motor[i].Temperature, Motor[i].State);
				}
			}

			PX4_INFO("\n");
		}
	}

	return 0;
}


int
MK::mk_servo_test(unsigned int chan)
{
	int ret = 0;
	float tmpVal = 0;
	float val = -1;
	_retries = 0;
	uint8_t msg[2] = { 0, 0 };

	if (debugCounter >= MOTOR_SPINUP_COUNTER) {
		debugCounter = 0;
		_motor++;

		if (_motor < _num_outputs) {
			PX4_INFO("Motortest - #%i:\tspinup", _motor);
		}

		if (_motor >= _num_outputs) {
			_motor = -1;
			_motortest = false;
			PX4_INFO("Motortest finished...");
		}
	}

	debugCounter++;

	if (_motor == chan) {
		val = BLCTRL_MIN_VALUE;

	} else {
		val = -1;
	}

	tmpVal = (511 + (511 * val));

	if (tmpVal > 1024) {
		tmpVal = 1024;
	}

	Motor[chan].SetPoint = (uint8_t)(tmpVal / 4);

	if (_motor != chan) {
		Motor[chan].SetPoint = 0;
		Motor[chan].SetPointLowerBits = 0;
	}

	if (Motor[chan].Version == BLCTRL_OLD) {
		msg[0] = Motor[chan].SetPoint;

	} else {
		msg[0] = Motor[chan].SetPoint;
		msg[1] = Motor[chan].SetPointLowerBits;
	}

	set_device_address(BLCTRL_BASE_ADDR + (chan + addrTranslator[chan]));

	if (Motor[chan].Version == BLCTRL_OLD) {
		ret = transfer(&msg[0], 1, nullptr, 0);

	} else {
		ret = transfer(&msg[0], 2, nullptr, 0);
	}

	return ret;
}


int
MK::mk_servo_locate()
{
	int ret = 0;
	static unsigned int chan = 0;
	static uint64_t last_timestamp = 0;
	_retries = 0;
	uint8_t msg[2] = { 0, 0 };


	if (hrt_absolute_time() - last_timestamp > MOTOR_LOCATE_DELAY) {
		last_timestamp = hrt_absolute_time();

		set_device_address(BLCTRL_BASE_ADDR + (chan + addrTranslator[chan]));
		chan++;

		if (chan <= _num_outputs) {
			PX4_INFO("ESC Locate - #%i:\tgreen", chan);
			play_beep(chan);
		}

		if (chan > _num_outputs) {
			chan = 0;
		}
	}

	// do i2c transfer to selected esc
	ret = transfer(&msg[0], 1, nullptr, 0);

	return ret;
}


int
MK::control_callback(uintptr_t handle,
		     uint8_t control_group,
		     uint8_t control_index,
		     float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls->control[control_index];
	return 0;
}

int
MK::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret;

	ret = pwm_ioctl(filp, cmd, arg);

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int
MK::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		mk_servo_arm(true);
		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_CLEAR_ARM_OK:
		// these are no-ops, as no safety switch
		break;

	case PWM_SERVO_DISARM:
		mk_servo_arm(false);
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		ret = OK;
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		*(uint32_t *)arg = _update_rate;
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		ret = OK;
		break;


	case PWM_SERVO_SET(0) ... PWM_SERVO_SET(_max_actuators - 1):
		if (arg < 2150) {
			Motor[cmd - PWM_SERVO_SET(0)].RawPwmValue = (unsigned short)arg;
			mk_servo_set(cmd - PWM_SERVO_SET(0), scaling(arg, _rc_min_value, _rc_max_value, 0, 2047));

		} else {
			ret = -EINVAL;
		}

		break;

	case PWM_SERVO_GET(0) ... PWM_SERVO_GET(_max_actuators - 1):
		/* copy the current output value from the channel */
		*(servo_position_t *)arg = Motor[cmd - PWM_SERVO_GET(0)].RawPwmValue;

		break;

	case PWM_SERVO_GET_RATEGROUP(0):
	case PWM_SERVO_GET_RATEGROUP(1):
	case PWM_SERVO_GET_RATEGROUP(2):
	case PWM_SERVO_GET_RATEGROUP(3):
		//*(uint32_t *)arg = up_pwm_servo_get_rate_group(cmd - PWM_SERVO_GET_RATEGROUP(0));
		break;

	case PWM_SERVO_GET_COUNT:
		*(unsigned *)arg = _num_outputs;
		break;

	case MIXERIOCRESET:
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
		}

		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);

			if (_mixers == nullptr) {
				_mixers = new MixerGroup();
			}

			if (_mixers == nullptr) {
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(control_callback, (uintptr_t)&_controls, buf, buflen);

				if (ret != 0) {
					DEVICE_DEBUG("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					ret = -EINVAL;
				}
			}

			break;
		}

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			set_rc_min_value((unsigned)pwm->values[0]);
			ret = OK;
			break;
		}

	case PWM_SERVO_GET_MIN_PWM:
		ret = OK;
		break;

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			set_rc_max_value((unsigned)pwm->values[0]);
			ret = OK;
			break;
		}

	case PWM_SERVO_GET_MAX_PWM:
		ret = OK;
		break;


	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

/*
  this implements PWM output via a write() method, for compatibility
  with px4io
 */
ssize_t
MK::write(file *filp, const char *buffer, size_t len)
{
	unsigned count = len / 2;
	uint16_t values[8];

	if (count > _num_outputs) {
		// we only have 8 I2C outputs in the driver
		count = _num_outputs;
	}

	// allow for misaligned values
	memcpy(values, buffer, count * 2);

	for (uint8_t i = 0; i < count; i++) {
		Motor[i].RawPwmValue = (unsigned short)values[i];
		mk_servo_set(i, scaling(values[i], _rc_min_value, _rc_max_value, 0, 2047));
	}

	return count * 2;
}


namespace
{

enum MappingMode {
	MAPPING_MK = 0,
	MAPPING_PX4,
};

enum FrameType {
	FRAME_PLUS = 0,
	FRAME_X,
};


int
mk_new_mode(int motorcount, bool motortest, int px4mode, int frametype, bool overrideSecurityChecks, unsigned rcmin,
	    unsigned rcmax)
{
	int shouldStop = 0;

	/* set rc min pulse value */
	g_mk->set_rc_min_value(rcmin);

	/* set rc max pulse value */
	g_mk->set_rc_max_value(rcmax);

	/* native PX4 addressing) */
	g_mk->set_px4mode(px4mode);

	/* set frametype (geometry) */
	g_mk->set_frametype(frametype);

	/* motortest if enabled */
	g_mk->set_motor_test(motortest);

	/* ovveride security checks if enabled */
	g_mk->set_overrideSecurityChecks(overrideSecurityChecks);

	/* count used motors */
	do {
		if (g_mk->mk_check_for_blctrl(8, false, false) != 0) {
			shouldStop = 4;

		} else {
			shouldStop++;
		}

		sleep(1);
	} while (shouldStop < 3);

	g_mk->set_motor_count(g_mk->mk_check_for_blctrl(8, true, false));

	return OK;
}

int
mk_start(unsigned motors, const char *device_path)
{
	int ret;

	// try i2c3 first
	g_mk = new MK(3, device_path);

	if (!g_mk) {
		return -ENOMEM;
	}

	if (OK == g_mk->init(motors)) {
		warnx("[mkblctrl] scanning i2c3...\n");
		ret = g_mk->mk_check_for_blctrl(8, false, false);

		if (ret > 0) {
			return OK;
		}
	}

	delete g_mk;
	g_mk = nullptr;

	// fallback to bus 1
	g_mk = new MK(1, device_path);

	if (!g_mk) {
		return -ENOMEM;
	}

	if (OK == g_mk->init(motors)) {
		warnx("[mkblctrl] scanning i2c1...\n");
		ret = g_mk->mk_check_for_blctrl(8, false, false);

		if (ret > 0) {
			return OK;
		}
	}

	delete g_mk;
	g_mk = nullptr;

	return -ENXIO;
}


} // namespace

extern "C" __EXPORT int mkblctrl_main(int argc, char *argv[]);

int
mkblctrl_main(int argc, char *argv[])
{
	int motorcount = 8;
	int px4mode = MAPPING_PX4;
	int frametype = FRAME_PLUS;	// + plus is default
	bool motortest = false;
	bool overrideSecurityChecks = false;
	bool showHelp = false;
	bool newMode = true;
	const char *devicepath = "";
	unsigned rc_min_value = RC_MIN_VALUE;
	unsigned rc_max_value = RC_MAX_VALUE;
	char *ep;

	/*
	 * optional parameters
	 */
	for (int i = 1; i < argc; i++) {

		/* look for the optional frame parameter */
		if (strcmp(argv[i], "-mkmode") == 0 || strcmp(argv[i], "--mkmode") == 0) {
			if (argc > i + 1) {
				if (strcmp(argv[i + 1], "+") == 0 || strcmp(argv[i + 1], "x") == 0 || strcmp(argv[i + 1], "X") == 0) {
					px4mode = MAPPING_MK;
					//newMode = true;

					if (strcmp(argv[i + 1], "+") == 0) {
						frametype = FRAME_PLUS;

					} else {
						frametype = FRAME_X;
					}

				} else {
					errx(1, "only + or x for frametype supported !");
				}

			} else {
				errx(1, "missing argument for mkmode (-mkmode)");
				return 1;
			}
		}

		/* look for the optional test parameter */
		if (strcmp(argv[i], "-t") == 0) {
			motortest = true;
			//newMode = true;
		}

		/* look for the optional -h --help parameter */
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			showHelp = true;
		}

		/* look for the optional --override-security-checks  parameter */
		if (strcmp(argv[i], "--override-security-checks") == 0) {
			overrideSecurityChecks = true;
			//newMode = true;
		}

		/* look for the optional device parameter */
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				devicepath = argv[i + 1];
				//newMode = true;

			} else {
				errx(1, "missing the devicename (-d)");
				return 1;
			}
		}

		/* look for the optional -rc_min parameter */
		if (strcmp(argv[i], "-rc_min") == 0) {
			if (argc > i + 1) {
				rc_min_value = strtoul(argv[i + 1], &ep, 0);

				if (*ep != '\0') {
					errx(1, "bad pwm val (-rc_min)");
					return 1;
				}

			} else {
				errx(1, "missing value (-rc_min)");
				return 1;
			}
		}

		/* look for the optional -rc_max parameter */
		if (strcmp(argv[i], "-rc_max") == 0) {
			if (argc > i + 1) {
				rc_max_value = strtoul(argv[i + 1], &ep, 0);

				if (*ep != '\0') {
					errx(1, "bad pwm val (-rc_max)");
					return 1;
				}

			} else {
				errx(1, "missing value (-rc_max)");
				return 1;
			}
		}


	}

	if (showHelp) {
		PX4_INFO("mkblctrl: help:");
		PX4_INFO("  [-mkmode {+/x}] [-b i2c_bus_number] [-d devicename] [--override-security-checks] [-h / --help]");
		PX4_INFO("\t -mkmode {+/x} \t\t Type of frame, if Mikrokopter motor order is used.");
		PX4_INFO("\t -d {devicepath & name}\t\t Create alternate pwm device.");
		PX4_INFO("\t --override-security-checks \t\t Disable all security checks (arming and number of ESCs). Used to test single Motors etc. (DANGER !!!)");
		PX4_INFO("\t -rcmin {pwn-value}\t\t Set RC_MIN Value.");
		PX4_INFO("\t -rcmax {pwn-value}\t\t Set RC_MAX Value.");
		PX4_INFO("\n");
		PX4_INFO("Motortest:");
		PX4_INFO("First you have to start mkblctrl, the you can enter Motortest Mode with:");
		PX4_INFO("mkblctrl -t");
		PX4_INFO("This will spin up once every motor in order of motoraddress. (DANGER !!!)");
		exit(1);
	}


	if (!motortest) {
		if (g_mk == nullptr) {
			if (mk_start(motorcount, devicepath) != OK) {
				errx(1, "failed to start the MK-BLCtrl driver");
			}

			/* parameter set ? */
			if (newMode) {
				/* switch parameter */
				return mk_new_mode(motorcount, motortest, px4mode, frametype, overrideSecurityChecks, rc_min_value, rc_max_value);
			}

			exit(0);

		} else {
			errx(1, "MK-BLCtrl driver already running");
		}

	} else {
		if (g_mk == nullptr) {
			errx(1, "MK-BLCtrl driver not running. You have to start it first.");

		} else {
			g_mk->set_motor_test(motortest);
			exit(0);

		}
	}
}
