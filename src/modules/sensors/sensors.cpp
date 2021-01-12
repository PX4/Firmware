/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file sensors.cpp
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <drivers/drv_adc.h>
#include <drivers/drv_airspeed.h>
#include <drivers/drv_hrt.h>
#include <lib/airspeed/airspeed.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensors_status_imu.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_imu.h>

#include "voted_sensors_update.h"
#include "vehicle_acceleration/VehicleAcceleration.hpp"
#include "vehicle_angular_velocity/VehicleAngularVelocity.hpp"
#include "vehicle_air_data/VehicleAirData.hpp"
#include "vehicle_gps_position/VehicleGPSPosition.hpp"
#include "vehicle_imu/VehicleIMU.hpp"
#include "vehicle_magnetometer/VehicleMagnetometer.hpp"

using namespace sensors;
using namespace time_literals;

/**
 * HACK - true temperature is much less than indicated temperature in baro,
 * subtract 5 degrees in an attempt to account for the electrical upheating of the PCB
 */
#define PCB_TEMP_ESTIMATE_DEG		5.0f
class Sensors : public ModuleBase<Sensors>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	explicit Sensors(bool hil_enabled);
	~Sensors() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	const bool	_hil_enabled;			/**< if true, HIL is active */
	bool		_armed{false};				/**< arming status of the vehicle */

	hrt_abstime     _last_config_update{0};
	hrt_abstime     _sensor_combined_prev_timestamp{0};

	sensor_combined_s _sensor_combined{};

	uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub[MAX_SENSOR_COUNT] {
		{this, ORB_ID(vehicle_imu), 0},
		{this, ORB_ID(vehicle_imu), 1},
		{this, ORB_ID(vehicle_imu), 2},
		{this, ORB_ID(vehicle_imu), 3}
	};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _diff_pres_sub{ORB_ID(differential_pressure)};
	uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};

	uORB::Publication<airspeed_s>             _airspeed_pub{ORB_ID(airspeed)};
	uORB::Publication<sensor_combined_s>      _sensor_pub{ORB_ID(sensor_combined)};

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	DataValidator	_airspeed_validator;		/**< data validator to monitor airspeed */

#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL

	hrt_abstime	_last_adc{0};			/**< last time we took input from the ADC */

	uORB::Subscription	_adc_report_sub{ORB_ID(adc_report)};		/**< adc_report sub */
	differential_pressure_s	_diff_pres {};
	uORB::PublicationMulti<differential_pressure_s>	_diff_pres_pub{ORB_ID(differential_pressure)};		/**< differential_pressure */
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */


	struct Parameters {
		float diff_pres_offset_pa;
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
		float diff_pres_analog_scale;
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

		int32_t air_cmodel;
		float air_tube_length;
		float air_tube_diameter_mm;
	} _parameters{}; /**< local copies of interesting parameters */

	struct ParameterHandles {
		param_t diff_pres_offset_pa;
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
		param_t diff_pres_analog_scale;
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

		param_t air_cmodel;
		param_t air_tube_length;
		param_t air_tube_diameter_mm;
	} _parameter_handles{};		/**< handles for interesting parameters */

	VotedSensorsUpdate _voted_sensors_update;

	VehicleAcceleration	_vehicle_acceleration;
	VehicleAngularVelocity	_vehicle_angular_velocity;
	VehicleAirData          *_vehicle_air_data{nullptr};
	VehicleMagnetometer     *_vehicle_magnetometer{nullptr};
	VehicleGPSPosition	*_vehicle_gps_position{nullptr};

	VehicleIMU      *_vehicle_imu_list[MAX_SENSOR_COUNT] {};

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Poll the differential pressure sensor for updated data.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		diff_pres_poll();

	/**
	 * Check for changes in parameters.
	 */
	void 		parameter_update_poll(bool forced = false);

	/**
	 * Poll the ADC and update readings to suit.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		adc_poll();

	void		InitializeVehicleAirData();
	void		InitializeVehicleGPSPosition();
	void		InitializeVehicleIMU();
	void		InitializeVehicleMagnetometer();

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::SYS_HAS_BARO>) _param_sys_has_baro,
		(ParamBool<px4::params::SYS_HAS_MAG>) _param_sys_has_mag,
		(ParamBool<px4::params::SENS_IMU_MODE>) _param_sens_imu_mode
	)
};

Sensors::Sensors(bool hil_enabled) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_hil_enabled(hil_enabled),
	_loop_perf(perf_alloc(PC_ELAPSED, "sensors")),
	_voted_sensors_update(hil_enabled, _vehicle_imu_sub)
{
	/* Differential pressure offset */
	_parameter_handles.diff_pres_offset_pa = param_find("SENS_DPRES_OFF");
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
	_parameter_handles.diff_pres_analog_scale = param_find("SENS_DPRES_ANSC");
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

	_parameter_handles.air_cmodel = param_find("CAL_AIR_CMODEL");
	_parameter_handles.air_tube_length = param_find("CAL_AIR_TUBELEN");
	_parameter_handles.air_tube_diameter_mm = param_find("CAL_AIR_TUBED_MM");

	param_find("SYS_FAC_CAL_MODE");

	// Parameters controlling the on-board sensor thermal calibrator
	param_find("SYS_CAL_TDEL");
	param_find("SYS_CAL_TMAX");
	param_find("SYS_CAL_TMIN");

	_airspeed_validator.set_timeout(300000);
	_airspeed_validator.set_equal_value_threshold(100);

	_vehicle_acceleration.Start();
	_vehicle_angular_velocity.Start();
}

Sensors::~Sensors()
{
	// clear all registered callbacks
	for (auto &sub : _vehicle_imu_sub) {
		sub.unregisterCallback();
	}

	_vehicle_acceleration.Stop();
	_vehicle_angular_velocity.Stop();

	if (_vehicle_air_data) {
		_vehicle_air_data->Stop();
		delete _vehicle_air_data;
	}

	if (_vehicle_gps_position) {
		_vehicle_gps_position->Stop();
		delete _vehicle_gps_position;
	}

	if (_vehicle_magnetometer) {
		_vehicle_magnetometer->Stop();
		delete _vehicle_magnetometer;
	}

	for (auto &vehicle_imu : _vehicle_imu_list) {
		if (vehicle_imu) {
			vehicle_imu->Stop();
			delete vehicle_imu;
		}
	}

	perf_free(_loop_perf);
}

bool Sensors::init()
{
	_vehicle_imu_sub[0].registerCallback();
	ScheduleNow();
	return true;
}

int Sensors::parameters_update()
{
	if (_armed) {
		return 0;
	}

	/* Airspeed offset */
	param_get(_parameter_handles.diff_pres_offset_pa, &(_parameters.diff_pres_offset_pa));
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
	param_get(_parameter_handles.diff_pres_analog_scale, &(_parameters.diff_pres_analog_scale));
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

	param_get(_parameter_handles.air_cmodel, &_parameters.air_cmodel);
	param_get(_parameter_handles.air_tube_length, &_parameters.air_tube_length);
	param_get(_parameter_handles.air_tube_diameter_mm, &_parameters.air_tube_diameter_mm);

	_voted_sensors_update.parametersUpdate();

	return PX4_OK;
}

void Sensors::diff_pres_poll()
{
	differential_pressure_s diff_pres{};

	if (_diff_pres_sub.update(&diff_pres)) {

		vehicle_air_data_s air_data{};
		_vehicle_air_data_sub.copy(&air_data);

		float air_temperature_celsius = NAN;

		// assume anything outside of a (generous) operating range of -40C to 125C is invalid
		if (PX4_ISFINITE(diff_pres.temperature) && (diff_pres.temperature >= -40.f) && (diff_pres.temperature <= 125.f)) {

			air_temperature_celsius = diff_pres.temperature;

		} else {
			// differential pressure temperature invalid, check barometer
			if ((air_data.timestamp != 0) && PX4_ISFINITE(air_data.baro_temp_celcius)
			    && (air_data.baro_temp_celcius >= -40.f) && (air_data.baro_temp_celcius <= 125.f)) {

				// TODO: review PCB_TEMP_ESTIMATE_DEG, ignore for external baro
				air_temperature_celsius = air_data.baro_temp_celcius - PCB_TEMP_ESTIMATE_DEG;
			}
		}

		airspeed_s airspeed{};
		airspeed.timestamp = diff_pres.timestamp;

		/* push data into validator */
		float airspeed_input[3] = { diff_pres.differential_pressure_raw_pa, diff_pres.temperature, 0.0f };

		_airspeed_validator.put(airspeed.timestamp, airspeed_input, diff_pres.error_count, 100); // TODO: real priority?

		airspeed.confidence = _airspeed_validator.confidence(hrt_absolute_time());

		enum AIRSPEED_SENSOR_MODEL smodel;

		switch ((diff_pres.device_id >> 16) & 0xFF) {
		case DRV_DIFF_PRESS_DEVTYPE_SDP31:

		/* fallthrough */
		case DRV_DIFF_PRESS_DEVTYPE_SDP32:

		/* fallthrough */
		case DRV_DIFF_PRESS_DEVTYPE_SDP33:
			/* fallthrough */
			smodel = AIRSPEED_SENSOR_MODEL_SDP3X;
			break;

		default:
			smodel = AIRSPEED_SENSOR_MODEL_MEMBRANE;
			break;
		}

		/* don't risk to feed negative airspeed into the system */
		airspeed.indicated_airspeed_m_s = calc_IAS_corrected((enum AIRSPEED_COMPENSATION_MODEL)
						  _parameters.air_cmodel,
						  smodel, _parameters.air_tube_length, _parameters.air_tube_diameter_mm,
						  diff_pres.differential_pressure_filtered_pa, air_data.baro_pressure_pa,
						  air_temperature_celsius);

		airspeed.true_airspeed_m_s = calc_TAS_from_CAS(airspeed.indicated_airspeed_m_s, air_data.baro_pressure_pa,
					     air_temperature_celsius); // assume that CAS = IAS as we don't have an CAS-scale here

		airspeed.air_temperature_celsius = air_temperature_celsius;

		if (PX4_ISFINITE(airspeed.indicated_airspeed_m_s) && PX4_ISFINITE(airspeed.true_airspeed_m_s)) {
			_airspeed_pub.publish(airspeed);
		}
	}
}

void
Sensors::parameter_update_poll(bool forced)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || forced) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		parameters_update();
		updateParams();

		/* update airspeed scale */
		int fd = px4_open(AIRSPEED0_DEVICE_PATH, 0);

		/* this sensor is optional, abort without error */
		if (fd >= 0) {
			struct airspeed_scale airscale = {
				_parameters.diff_pres_offset_pa,
				1.0f,
			};

			if (OK != px4_ioctl(fd, AIRSPEEDIOCSSCALE, (long unsigned int)&airscale)) {
				warn("WARNING: failed to set scale / offsets for airspeed sensor");
			}

			px4_close(fd);
		}
	}
}

void Sensors::adc_poll()
{
	/* only read if not in HIL mode */
	if (_hil_enabled) {
		return;
	}

#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL

	if (_parameters.diff_pres_analog_scale > 0.0f) {

		hrt_abstime t = hrt_absolute_time();

		/* rate limit to 100 Hz */
		if (t - _last_adc >= 10000) {
			adc_report_s adc;

			if (_adc_report_sub.update(&adc)) {
				/* Read add channels we got */
				for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; i++) {
					if (adc.channel_id[i] == -1) {
						continue;	// skip non-exist channels
					}

					if (ADC_AIRSPEED_VOLTAGE_CHANNEL == adc.channel_id[i]) {

						/* calculate airspeed, raw is the difference from */
						const float voltage = (float)(adc.raw_data[i]) * adc.v_ref / adc.resolution * ADC_DP_V_DIV;

						/**
						 * The voltage divider pulls the signal down, only act on
						 * a valid voltage from a connected sensor. Also assume a non-
						 * zero offset from the sensor if its connected.
						 *
						 * Notice: This won't work on devices which have PGA controlled
						 * vref. Those devices require no divider at all.
						 */
						if (voltage > 0.4f) {
							const float diff_pres_pa_raw = voltage * _parameters.diff_pres_analog_scale - _parameters.diff_pres_offset_pa;

							_diff_pres.timestamp = t;
							_diff_pres.differential_pressure_raw_pa = diff_pres_pa_raw;
							_diff_pres.differential_pressure_filtered_pa = (_diff_pres.differential_pressure_filtered_pa * 0.9f) +
									(diff_pres_pa_raw * 0.1f);
							_diff_pres.temperature = -1000.0f;

							_diff_pres_pub.publish(_diff_pres);
						}
					}
				}
			}

			_last_adc = t;
		}
	}

#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */
}

void Sensors::InitializeVehicleAirData()
{
	if (_param_sys_has_baro.get()) {
		if (_vehicle_air_data == nullptr) {
			if (orb_exists(ORB_ID(sensor_baro), 0) == PX4_OK) {
				_vehicle_air_data = new VehicleAirData();

				if (_vehicle_air_data) {
					_vehicle_air_data->Start();
				}
			}
		}
	}
}

void Sensors::InitializeVehicleGPSPosition()
{
	if (_vehicle_gps_position == nullptr) {
		if (orb_exists(ORB_ID(sensor_gps), 0) == PX4_OK) {
			_vehicle_gps_position = new VehicleGPSPosition();

			if (_vehicle_gps_position) {
				_vehicle_gps_position->Start();
			}
		}
	}
}

void Sensors::InitializeVehicleIMU()
{
	// create a VehicleIMU instance for each accel/gyro pair
	for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (_vehicle_imu_list[i] == nullptr) {

			uORB::Subscription accel_sub{ORB_ID(sensor_accel), i};
			sensor_accel_s accel{};
			accel_sub.copy(&accel);

			uORB::Subscription gyro_sub{ORB_ID(sensor_gyro), i};
			sensor_gyro_s gyro{};
			gyro_sub.copy(&gyro);

			if (accel.device_id > 0 && gyro.device_id > 0) {
				// if the sensors module is responsible for voting (SENS_IMU_MODE 1) then run every VehicleIMU in the same WQ
				//   otherwise each VehicleIMU runs in a corresponding INSx WQ
				const bool multi_mode = (_param_sens_imu_mode.get() == 0);
				const px4::wq_config_t &wq_config = multi_mode ? px4::ins_instance_to_wq(i) : px4::wq_configurations::INS0;

				VehicleIMU *imu = new VehicleIMU(i, i, i, wq_config);

				if (imu != nullptr) {
					// Start VehicleIMU instance and store
					if (imu->Start()) {
						_vehicle_imu_list[i] = imu;

					} else {
						delete imu;
					}
				}

			} else {
				// abort on first failure, try again later
				return;
			}
		}
	}
}

void Sensors::InitializeVehicleMagnetometer()
{
	if (_param_sys_has_mag.get()) {
		if (_vehicle_magnetometer == nullptr) {
			if (orb_exists(ORB_ID(sensor_mag), 0) == PX4_OK) {
				_vehicle_magnetometer = new VehicleMagnetometer();

				if (_vehicle_magnetometer) {
					_vehicle_magnetometer->Start();
				}
			}
		}
	}
}

void Sensors::Run()
{
	if (should_exit()) {
		// clear all registered callbacks
		for (auto &sub : _vehicle_imu_sub) {
			sub.unregisterCallback();
		}

		exit_and_cleanup();
		return;
	}

	// run once
	if (_last_config_update == 0) {
		InitializeVehicleAirData();
		InitializeVehicleIMU();
		InitializeVehicleGPSPosition();
		InitializeVehicleMagnetometer();
		_voted_sensors_update.init(_sensor_combined);
		parameter_update_poll(true);
	}

	perf_begin(_loop_perf);

	// backup schedule as a watchdog timeout
	ScheduleDelayed(10_ms);

	// check vehicle status for changes to publication state
	if (_vcontrol_mode_sub.updated()) {
		vehicle_control_mode_s vcontrol_mode{};

		if (_vcontrol_mode_sub.copy(&vcontrol_mode)) {
			_armed = vcontrol_mode.flag_armed;
		}
	}

	_voted_sensors_update.sensorsPoll(_sensor_combined);

	// check analog airspeed
	adc_poll();

	diff_pres_poll();

	if (_sensor_combined.timestamp != _sensor_combined_prev_timestamp) {

		_voted_sensors_update.setRelativeTimestamps(_sensor_combined);
		_sensor_pub.publish(_sensor_combined);
		_sensor_combined_prev_timestamp = _sensor_combined.timestamp;
	}

	// keep adding sensors as long as we are not armed,
	// when not adding sensors poll for param updates
	if (!_armed && hrt_elapsed_time(&_last_config_update) > 500_ms) {
		_voted_sensors_update.initializeSensors();
		InitializeVehicleAirData();
		InitializeVehicleIMU();
		InitializeVehicleGPSPosition();
		InitializeVehicleMagnetometer();
		_last_config_update = hrt_absolute_time();

	} else {
		// check parameters for updates
		parameter_update_poll();
	}

	perf_end(_loop_perf);
}

int Sensors::task_spawn(int argc, char *argv[])
{
	bool hil_enabled = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "h", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'h':
			hil_enabled = true;
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
		return PX4_ERROR;
	}

	Sensors *instance = new Sensors(hil_enabled);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
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

int Sensors::print_status()
{
	_voted_sensors_update.printStatus();

	if (_vehicle_magnetometer) {
		PX4_INFO_RAW("\n");
		_vehicle_magnetometer->PrintStatus();
	}

	if (_vehicle_air_data) {
		PX4_INFO_RAW("\n");
		_vehicle_air_data->PrintStatus();
	}

	PX4_INFO_RAW("\n");
	PX4_INFO("Airspeed status:");
	_airspeed_validator.print();

	PX4_INFO_RAW("\n");
	_vehicle_acceleration.PrintStatus();

	PX4_INFO_RAW("\n");
	_vehicle_angular_velocity.PrintStatus();

	if (_vehicle_gps_position) {
		PX4_INFO_RAW("\n");
		_vehicle_gps_position->PrintStatus();
	}

	PX4_INFO_RAW("\n");

	for (auto &i : _vehicle_imu_list) {
		if (i != nullptr) {
			PX4_INFO_RAW("\n");
			i->PrintStatus();
		}
	}

	return 0;
}

int Sensors::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Sensors::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The sensors module is central to the whole system. It takes low-level output from drivers, turns
it into a more usable form, and publishes it for the rest of the system.

The provided functionality includes:
- Read the output from the sensor drivers (`sensor_gyro`, etc.).
  If there are multiple of the same type, do voting and failover handling.
  Then apply the board rotation and temperature calibration (if enabled). And finally publish the data; one of the
  topics is `sensor_combined`, used by many parts of the system.
- Make sure the sensor drivers get the updated calibration parameters (scale & offset) when the parameters change or
  on startup. The sensor drivers use the ioctl interface for parameter updates. For this to work properly, the
  sensor drivers must already be running when `sensors` is started.
- Do sensor consistency checks and publish the `sensors_status_imu` topic.

### Implementation
It runs in its own thread and polls on the currently selected gyro topic.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensors", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('h', "Start in HIL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sensors_main(int argc, char *argv[])
{
	return Sensors::main(argc, argv);
}
