/****************************************************************************
 *
 *   Copyright (c) 2015-2020 PX4 Development Team. All rights reserved.
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

#include "UavcanNode.hpp"

#include "boot_app_shared.h"

#include <lib/ecl/geo/geo.h>
#include <lib/version/version.h>

using namespace time_literals;

/**
 * @file uavcan_main.cpp
 *
 * Implements basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 *         David Sidrane <david_s5@nscdg.com>
 */

/*
 * This is the AppImageDescriptor used
 * by the make_can_boot_descriptor.py tool to set
 * the application image's descriptor so that the
 * uavcan bootloader has the ability to validate the
 * image crc, size etc of this application
*/

boot_app_shared_section app_descriptor_t AppDescriptor = {
	.signature = {APP_DESCRIPTOR_SIGNATURE},
	.image_crc = 0,
	.image_size = 0,
	.vcs_commit = 0,
	.major_version = APP_VERSION_MAJOR,
	.minor_version = APP_VERSION_MINOR,
	.reserved = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff }
};

UavcanNode *UavcanNode::_instance;

UavcanNode::UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
	_node(can_driver, system_clock, _pool_allocator),
	_time_sync_slave(_node),
	_fw_update_listner(_node),
	_ahrs_magnetic_field_strength2_publisher(_node),
	_gnss_fix2_publisher(_node),
	_power_battery_info_publisher(_node),
	_air_data_static_pressure_publisher(_node),
	_air_data_static_temperature_publisher(_node),
	_raw_air_data_publisher(_node),
	_range_sensor_measurement(_node),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")),
	_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")),
	_reset_timer(_node)
{
	int res = pthread_mutex_init(&_node_mutex, nullptr);

	if (res < 0) {
		std::abort();
	}
}

UavcanNode::~UavcanNode()
{
	if (_instance) {
		/* tell the task we want it to go away */
		_task_should_exit.store(true);
		ScheduleNow();

		unsigned i = 10;

		do {
			/* wait 5ms - it should wake every 10ms or so worst-case */
			usleep(5000);

			if (--i == 0) {
				break;
			}

		} while (_instance);
	}

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

int UavcanNode::getHardwareVersion(uavcan::protocol::HardwareVersion &hwver)
{
	int rv = -1;

	if (UavcanNode::instance()) {

		hwver.major = HW_VERSION_MAJOR;
		hwver.minor = HW_VERSION_MINOR;

		mfguid_t mfgid = {};
		board_get_mfguid(mfgid);
		uavcan::copy(mfgid, mfgid + sizeof(mfgid), hwver.unique_id.begin());
		rv = 0;
	}

	return rv;
}

int UavcanNode::start(uavcan::NodeID node_id, uint32_t bitrate)
{
	if (_instance != nullptr) {
		PX4_WARN("Already started");
		return -1;
	}

	/*
	 * CAN driver init
	 * Note that we instantiate and initialize CanInitHelper only once, because the STM32's bxCAN driver
	 * shipped with libuavcan does not support deinitialization.
	 */
	static CanInitHelper *can = nullptr;

	if (can == nullptr) {

		can = new CanInitHelper();

		if (can == nullptr) {                    // We don't have exceptions so bad_alloc cannot be thrown
			PX4_ERR("Out of memory");
			return -1;
		}

		const int can_init_res = can->init(bitrate);

		if (can_init_res < 0) {
			PX4_ERR("CAN driver init failed %i", can_init_res);
			return can_init_res;
		}
	}

	// Node init
	_instance = new UavcanNode(can->driver, UAVCAN_DRIVER::SystemClock::instance());

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}

	const int node_init_res = _instance->init(node_id, can->driver.updateEvent());

	if (node_init_res < 0) {
		delete _instance;
		_instance = nullptr;
		PX4_ERR("Node init failed %i", node_init_res);
		return node_init_res;
	}

	// Keep the bit rate for reboots on BenginFirmware updates
	_instance->active_bitrate = bitrate;

	_instance->ScheduleOnInterval(ScheduleIntervalMs * 1000);

	return PX4_OK;
}

void UavcanNode::fill_node_info()
{
	// software version
	uavcan::protocol::SoftwareVersion swver;

	// Extracting the first 8 hex digits of the git hash and converting them to int
	char fw_git_short[9] = {};
	std::memmove(fw_git_short, px4_firmware_version_string(), 8);
	char *end = nullptr;
	swver.vcs_commit = std::strtol(fw_git_short, &end, 16);
	swver.optional_field_flags |= swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
	swver.major = AppDescriptor.major_version;
	swver.minor = AppDescriptor.minor_version;
	swver.image_crc = AppDescriptor.image_crc;

	_node.setSoftwareVersion(swver);

	/* hardware version */
	uavcan::protocol::HardwareVersion hwver;
	getHardwareVersion(hwver);
	_node.setHardwareVersion(hwver);
}

void UavcanNode::busevent_signal_trampoline()
{
	if (_instance) {
		// trigger the work queue (Note, this is called from IRQ context)
		_instance->ScheduleNow();
	}
}

static void cb_reboot(const uavcan::TimerEvent &)
{
	board_reset(0);
}

void UavcanNode::cb_beginfirmware_update(const uavcan::ReceivedDataStructure<UavcanNode::BeginFirmwareUpdate::Request>
		&req, uavcan::ServiceResponseDataStructure<UavcanNode::BeginFirmwareUpdate::Response> &rsp)
{
	static bool inprogress = false;

	rsp.error = rsp.ERROR_UNKNOWN;

	if (req.image_file_remote_path.path.size()) {
		rsp.error = rsp.ERROR_IN_PROGRESS;

		if (!inprogress) {
			inprogress = true;
			bootloader_app_shared_t shared;
			shared.bus_speed = active_bitrate;
			shared.node_id = _node.getNodeID().get();
			bootloader_app_shared_write(&shared, App);
			//rgb_led(255, 128, 0, 5);
			_reset_timer.setCallback(cb_reboot);
			_reset_timer.startOneShotWithDelay(uavcan::MonotonicDuration::fromMSec(1000));
			rsp.error = rsp.ERROR_OK;
		}
	}
}

int UavcanNode::init(uavcan::NodeID node_id, UAVCAN_DRIVER::BusEvent &bus_events)
{
	_node.setName(HW_UAVCAN_NAME);
	_node.setNodeID(node_id);

	fill_node_info();

	const int srv_start_res = _fw_update_listner.start(BeginFirmwareUpdateCallBack(this,
				  &UavcanNode::cb_beginfirmware_update));

	if (srv_start_res < 0) {
		return PX4_ERROR;
	}

	bus_events.registerSignalCallback(UavcanNode::busevent_signal_trampoline);

	return _node.start();
}

// Restart handler
class RestartRequestHandler: public uavcan::IRestartRequestHandler
{
	bool handleRestartRequest(uavcan::NodeID request_source) override
	{
		PX4_INFO("UAVCAN: Restarting by request from %i\n", int(request_source.get()));
		usleep(20 * 1000 * 1000);
		board_reset(0);
		return true; // Will never be executed BTW
	}
} restart_request_handler;

void UavcanNode::Run()
{
	pthread_mutex_lock(&_node_mutex);

	if (!_initialized) {

		get_node().setRestartRequestHandler(&restart_request_handler);

		// Set up the time synchronization
		const int slave_init_res = _time_sync_slave.start();

		if (slave_init_res < 0) {
			PX4_ERR("Failed to start time_sync_slave");
			_task_should_exit.store(true);
		}

		_node.setModeOperational();

		_diff_pressure_sub.registerCallback();

		for (auto &dist : _distance_sensor_sub) {
			dist.registerCallback();
		}

		_sensor_baro_sub.registerCallback();
		_sensor_mag_sub.registerCallback();
		_vehicle_gps_position_sub.registerCallback();

		_initialized = true;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
	}

	const int spin_res = _node.spin(uavcan::MonotonicTime());

	if (spin_res < 0) {
		PX4_ERR("node spin error %i", spin_res);
	}

	// battery_status -> uavcan::equipment::power::BatteryInfo
	if (_battery_status_sub.updated()) {
		battery_status_s battery;

		if (_battery_status_sub.copy(&battery)) {
			uavcan::equipment::power::BatteryInfo battery_info{};
			battery_info.voltage = battery.voltage_v;
			battery_info.current = fabs(battery.current_a);
			battery_info.temperature = battery.temperature - CONSTANTS_ABSOLUTE_NULL_CELSIUS; // convert from C to K
			battery_info.full_charge_capacity_wh = battery.capacity;
			battery_info.remaining_capacity_wh = battery.remaining * battery.capacity;
			battery_info.state_of_charge_pct = battery.remaining * 100;
			battery_info.state_of_charge_pct_stdev = battery.max_error;
			battery_info.model_instance_id = 0; // TODO: what goes here?
			battery_info.model_name = "ARK BMS Rev 0.2";
			battery_info.battery_id = battery.serial_number;
			battery_info.hours_to_full_charge = 0; // TODO: Read BQ40Z80_TIME_TO_FULL
			battery_info.state_of_health_pct = battery.state_of_health;

			if (battery.current_a > 0.0f) {
				battery_info.status_flags = uavcan::equipment::power::BatteryInfo::STATUS_FLAG_CHARGING;

			} else {
				battery_info.status_flags = uavcan::equipment::power::BatteryInfo::STATUS_FLAG_IN_USE;
			}

			_power_battery_info_publisher.broadcast(battery_info);
		}
	}

	// differential_pressure -> uavcan::equipment::air_data::RawAirData
	if (_diff_pressure_sub.updated()) {
		differential_pressure_s diff_press;

		if (_diff_pressure_sub.copy(&diff_press)) {

			uavcan::equipment::air_data::RawAirData raw_air_data{};

			// raw_air_data.static_pressure =
			raw_air_data.differential_pressure = diff_press.differential_pressure_raw_pa;
			// raw_air_data.static_pressure_sensor_temperature =
			raw_air_data.differential_pressure_sensor_temperature = diff_press.temperature - CONSTANTS_ABSOLUTE_NULL_CELSIUS;
			raw_air_data.static_air_temperature = diff_press.temperature - CONSTANTS_ABSOLUTE_NULL_CELSIUS;
			// raw_air_data.pitot_temperature
			// raw_air_data.covariance
			_raw_air_data_publisher.broadcast(raw_air_data);
		}
	}

	// distance_sensor[] -> uavcan::equipment::range_sensor::Measurement
	for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		distance_sensor_s dist;

		if (_distance_sensor_sub[i].update(&dist)) {
			uavcan::equipment::range_sensor::Measurement range_sensor{};

			range_sensor.sensor_id = i;
			range_sensor.range = dist.current_distance;
			range_sensor.field_of_view = dist.h_fov;

			// sensor type
			switch (dist.type) {
			case distance_sensor_s::MAV_DISTANCE_SENSOR_LASER:
				range_sensor.sensor_type = uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_LIDAR;
				break;

			case distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND:
				range_sensor.sensor_type = uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_SONAR;
				break;

			case distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR:
				range_sensor.sensor_type = uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_RADAR;
				break;

			case distance_sensor_s::MAV_DISTANCE_SENSOR_INFRARED:
			default:
				range_sensor.sensor_type = uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_UNDEFINED;
				break;
			}

			// reading_type
			if (dist.current_distance >= dist.max_distance) {
				range_sensor.reading_type = uavcan::equipment::range_sensor::Measurement::READING_TYPE_TOO_FAR;

			} else if (dist.current_distance <= dist.min_distance) {
				range_sensor.reading_type = uavcan::equipment::range_sensor::Measurement::READING_TYPE_TOO_CLOSE;

			} else if (dist.signal_quality != 0) {
				range_sensor.reading_type = uavcan::equipment::range_sensor::Measurement::READING_TYPE_VALID_RANGE;

			} else {
				range_sensor.reading_type = uavcan::equipment::range_sensor::Measurement::READING_TYPE_UNDEFINED;
			}

			_range_sensor_measurement.broadcast(range_sensor);
		}
	}

	// sensor_baro -> uavcan::equipment::air_data::StaticTemperature
	if (_sensor_baro_sub.updated()) {
		sensor_baro_s baro;

		if (_sensor_baro_sub.copy(&baro)) {
			uavcan::equipment::air_data::StaticPressure static_pressure{};
			static_pressure.static_pressure = baro.pressure * 100; // millibar -> pascals
			_air_data_static_pressure_publisher.broadcast(static_pressure);

			if (hrt_elapsed_time(&_last_static_temperature_publish) > 1_s) {
				uavcan::equipment::air_data::StaticTemperature static_temperature{};
				static_temperature.static_temperature = baro.temperature + CONSTANTS_ABSOLUTE_NULL_CELSIUS;
				_air_data_static_temperature_publisher.broadcast(static_temperature);
				_last_static_temperature_publish = hrt_absolute_time();
			}
		}
	}

	// sensor_mag -> uavcan::equipment::ahrs::MagneticFieldStrength2
	if (_sensor_mag_sub.updated()) {
		sensor_mag_s mag;

		if (_sensor_mag_sub.copy(&mag)) {
			uavcan::equipment::ahrs::MagneticFieldStrength2 magnetic_field{};
			magnetic_field.sensor_id = mag.device_id;
			magnetic_field.magnetic_field_ga[0] = mag.x;
			magnetic_field.magnetic_field_ga[1] = mag.y;
			magnetic_field.magnetic_field_ga[2] = mag.z;
			_ahrs_magnetic_field_strength2_publisher.broadcast(magnetic_field);
		}
	}

	// vehicle_gps_position -> uavcan::equipment::gnss::Fix2
	if (_vehicle_gps_position_sub.updated()) {
		vehicle_gps_position_s gps;

		if (_vehicle_gps_position_sub.copy(&gps)) {
			uavcan::equipment::gnss::Fix2 fix2{};

			fix2.gnss_time_standard = fix2.GNSS_TIME_STANDARD_UTC;
			fix2.gnss_timestamp.usec = gps.time_utc_usec;
			fix2.latitude_deg_1e8 = (int64_t)gps.lat * 10;
			fix2.longitude_deg_1e8 = (int64_t)gps.lon * 10;
			fix2.height_msl_mm = gps.alt;
			fix2.height_ellipsoid_mm = gps.alt_ellipsoid;
			fix2.status = gps.fix_type;
			fix2.ned_velocity[0] = gps.vel_n_m_s;
			fix2.ned_velocity[1] = gps.vel_e_m_s;
			fix2.ned_velocity[2] = gps.vel_d_m_s;
			fix2.pdop = gps.hdop > gps.vdop ? gps.hdop :
				    gps.vdop; // Use pdop for both hdop and vdop since uavcan v0 spec does not support them
			fix2.sats_used = gps.satellites_used;

			// Diagonal matrix
			// position variances -- Xx, Yy, Zz
			fix2.covariance.push_back(gps.eph);
			fix2.covariance.push_back(gps.eph);
			fix2.covariance.push_back(gps.eph);
			// velocity variance -- Vxx, Vyy, Vzz
			fix2.covariance.push_back(gps.s_variance_m_s);
			fix2.covariance.push_back(gps.s_variance_m_s);
			fix2.covariance.push_back(gps.s_variance_m_s);

			_gnss_fix2_publisher.broadcast(fix2);
		}
	}

	perf_end(_cycle_perf);

	pthread_mutex_unlock(&_node_mutex);

	if (_task_should_exit.load()) {
		ScheduleClear();
		_instance = nullptr;
	}
}

void UavcanNode::print_info()
{
	pthread_mutex_lock(&_node_mutex);

	// Memory status
	printf("Pool allocator status:\n");
	printf("\tCapacity hard/soft: %u/%u blocks\n",
	       _pool_allocator.getBlockCapacityHardLimit(), _pool_allocator.getBlockCapacity());
	printf("\tReserved:  %u blocks\n", _pool_allocator.getNumReservedBlocks());
	printf("\tAllocated: %u blocks\n", _pool_allocator.getNumAllocatedBlocks());

	printf("\n");

	// UAVCAN node perfcounters
	printf("UAVCAN node status:\n");
	printf("\tInternal failures: %llu\n", _node.getInternalFailureCount());
	printf("\tTransfer errors:   %llu\n", _node.getDispatcher().getTransferPerfCounter().getErrorCount());
	printf("\tRX transfers:      %llu\n", _node.getDispatcher().getTransferPerfCounter().getRxTransferCount());
	printf("\tTX transfers:      %llu\n", _node.getDispatcher().getTransferPerfCounter().getTxTransferCount());

	printf("\n");

	// CAN driver status
	for (unsigned i = 0; i < _node.getDispatcher().getCanIOManager().getCanDriver().getNumIfaces(); i++) {
		printf("CAN%u status:\n", unsigned(i + 1));

		auto iface = _node.getDispatcher().getCanIOManager().getCanDriver().getIface(i);
		printf("\tHW errors: %llu\n", iface->getErrorCount());

		auto iface_perf_cnt = _node.getDispatcher().getCanIOManager().getIfacePerfCounters(i);
		printf("\tIO errors: %llu\n", iface_perf_cnt.errors);
		printf("\tRX frames: %llu\n", iface_perf_cnt.frames_rx);
		printf("\tTX frames: %llu\n", iface_perf_cnt.frames_tx);
	}

	printf("\n");

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	pthread_mutex_unlock(&_node_mutex);
}

void UavcanNode::shrink()
{
	(void)pthread_mutex_lock(&_node_mutex);
	_pool_allocator.shrink();
	(void)pthread_mutex_unlock(&_node_mutex);
}

static void print_usage()
{
	PX4_INFO("usage: \n"
		 "\tuavcannode {start|status|stop|arm|disarm}");
}

extern "C" int uavcannode_start(int argc, char *argv[])
{
	//board_app_initialize(nullptr);

	// CAN bitrate
	int32_t bitrate = 0;

	// Node ID
	int32_t node_id = 0;

	// Did the bootloader auto baud and get a node ID Allocated
	bootloader_app_shared_t shared;
	int valid = bootloader_app_shared_read(&shared, BootLoader);

	if (valid == 0) {

		bitrate = shared.bus_speed;
		node_id = shared.node_id;

		// Invalidate to prevent deja vu
		bootloader_app_shared_invalidate();

	} else {
		// Node ID
		(void)param_get(param_find("CANNODE_NODE_ID"), &node_id);
		(void)param_get(param_find("CANNODE_BITRATE"), &bitrate);
	}

	if (node_id < 0 || node_id > uavcan::NodeID::Max || !uavcan::NodeID(node_id).isUnicast()) {
		PX4_ERR("Invalid Node ID %i", node_id);
		return 1;
	}

	// Start
	PX4_INFO("Node ID %u, bitrate %u", node_id, bitrate);
	int rv = UavcanNode::start(node_id, bitrate);

	return rv;
}

extern "C" __EXPORT int uavcannode_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return 1;
	}

	if (!std::strcmp(argv[1], "start")) {
		if (UavcanNode::instance()) {
			PX4_ERR("already started");
			return 1;
		}

		return uavcannode_start(argc, argv);
	}

	/* commands below require the app to be started */
	UavcanNode *const inst = UavcanNode::instance();

	if (!inst) {
		PX4_ERR("application not running");
		return 1;
	}

	if (!std::strcmp(argv[1], "status") || !std::strcmp(argv[1], "info")) {
		inst->print_info();
		return 0;
	}

	if (!std::strcmp(argv[1], "stop")) {
		delete inst;
		return 0;
	}

	print_usage();
	return 1;
}
