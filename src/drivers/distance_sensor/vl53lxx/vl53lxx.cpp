/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file vl53lxx.cpp
 * @author Daniele Pettenuzzo
 *
 * Driver for the vl53lxx ToF Sensor from ST Microelectronics connected via I2C.
 */

#include <stdlib.h>
#include <string.h>

#include <containers/Array.hpp>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <uORB/topics/distance_sensor.h>

/* Configuration Constants */
#define VL53LXX_BASEADDR                                0x29
#define VL53LXX_DEVICE_PATH                             "/dev/vl53lxx"

/* VL53LXX Registers addresses */
#define VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW_REG            0x89
#define MSRC_CONFIG_CONTROL_REG                         0x60
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG 0x44
#define SYSTEM_SEQUENCE_CONFIG_REG                      0x01
#define DYNAMIC_SPAD_REF_EN_START_OFFSET_REG            0x4F
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD_REG         0x4E
#define GLOBAL_CONFIG_REF_EN_START_SELECT_REG           0xB6
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG            0xB0
#define SYSTEM_INTERRUPT_CONFIG_GPIO_REG                0x0A
#define SYSTEM_SEQUENCE_CONFIG_REG                      0x01
#define SYSRANGE_START_REG                              0x00
#define RESULT_INTERRUPT_STATUS_REG                     0x13
#define SYSTEM_INTERRUPT_CLEAR_REG                      0x0B
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG            0xB0
#define GPIO_HV_MUX_ACTIVE_HIGH_REG                     0x84
#define SYSTEM_INTERRUPT_CLEAR_REG                      0x0B
#define RESULT_RANGE_STATUS_REG                         0x14
#define VL53LXX_RA_IDENTIFICATION_MODEL_ID              0xC0
#define VL53LXX_IDENTIFICATION_MODEL_ID                 0xEEAA

#define VL53LXX_US                                      1000    // 1ms
#define VL53LXX_SAMPLE_RATE                             50000   // 50ms

#define VL53LXX_MAX_RANGING_DISTANCE                    2.0f
#define VL53LXX_MIN_RANGING_DISTANCE                    0.0f

#define VL53LXX_BUS_CLOCK                               400000 // 400kHz bus clock speed

class VL53LXX : public device::I2C, public I2CSPIDriver<VL53LXX>
{
public:
	VL53LXX(I2CSPIBusOption bus_option, const int bus, const uint8_t rotation, int bus_frequency,
		int address = VL53LXX_BASEADDR);

	virtual ~VL53LXX();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual int init() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_status() override;

	virtual ssize_t read(device::file_t *file_pointer, char *buffer, size_t buflen);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 */
	void start();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void RunImpl();

protected:
	virtual int probe() override;

private:

	/**
	 * Collects the most recent sensor measurement data from the i2c bus.
	 */
	int collect();

	/**
	 * Sends an i2c measure command to the sensor.
	 */
	int measure();

	int readRegister(const uint8_t reg_address, uint8_t &value);
	int readRegisterMulti(const uint8_t reg_address, uint8_t *value, const uint8_t length);

	int writeRegister(const uint8_t reg_address, const uint8_t value);
	int writeRegisterMulti(const uint8_t reg_address, const uint8_t *value, const uint8_t length);

	int sensorInit();
	int sensorTuning();
	int singleRefCalibration(const uint8_t byte);
	int spadCalculations();

	bool _collect_phase{false};
	bool _measurement_started{false};
	bool _new_measurement{true};

	int _class_instance{-1};
	int _measure_interval{VL53LXX_SAMPLE_RATE};
	int _orb_class_instance{-1};

	uint8_t _rotation{0};
	uint8_t _stop_variable{0};

	orb_advert_t _distance_sensor_topic{nullptr};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "vl53lxx_com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "vl53lxx_read")};

	ringbuffer::RingBuffer *_reports{nullptr};
};


VL53LXX::VL53LXX(I2CSPIBusOption bus_option, const int bus, const uint8_t rotation, int bus_frequency, int address) :
	I2C("VL53LXX", VL53LXX_DEVICE_PATH, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_rotation(rotation)
{
	// Allow 3 retries as the device typically misses the first measure attempts.
	I2C::_retries = 3;
}

VL53LXX::~VL53LXX()
{
	// Free any existing reports.
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	// Unadvertise uORB topics.
	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
VL53LXX::collect()
{
	// Read from the sensor.
	uint8_t val[2] = {};
	perf_begin(_sample_perf);

	_collect_phase = false;

	int ret = transfer(nullptr, 0, &val[0], 2);

	if (ret != PX4_OK) {
		DEVICE_LOG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t distance_mm = (val[0] << 8) | val[1];
	float distance_m = distance_mm / 1000.f;

	struct distance_sensor_s report;
	report.timestamp        = hrt_absolute_time();
	report.current_distance = distance_m;
	report.id               = 0;	// TODO: set propoer ID
	report.max_distance     = VL53LXX_MAX_RANGING_DISTANCE;
	report.min_distance     = VL53LXX_MIN_RANGING_DISTANCE;
	report.orientation      = _rotation;
	report.signal_quality   = -1;
	report.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.variance         = 0.0f;

	// Publish the report data if we have a valid topic.
	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		orb_publish_auto(ORB_ID(distance_sensor), &_distance_sensor_topic, &report,
				 &_orb_class_instance, ORB_PRIO_DEFAULT);
	}

	_reports->force(&report);

	// Notify anyone waiting for data.
	poll_notify(POLLIN);
	perf_end(_sample_perf);

	return PX4_OK;
}

int
VL53LXX::init()
{
	set_device_address(VL53LXX_BASEADDR);

	// Initialize I2C (and probe) first.
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	if (_reports == nullptr) {
		return PX4_ERROR;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	// Get a publish handle on the obstacle distance topic.
	distance_sensor_s report {};
	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &report,
				 &_orb_class_instance, ORB_PRIO_DEFAULT);

	if (_distance_sensor_topic == nullptr) {
		PX4_ERR("failed to create distance_sensor object");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
VL53LXX::measure()
{
	uint8_t wait_for_measurement = 0;
	uint8_t system_start = 0;

	// Send the command to begin a measurement.
	const uint8_t cmd = RESULT_RANGE_STATUS_REG + 10;

	if (_new_measurement) {

		_new_measurement = false;

		writeRegister(0x80, 0x01);
		writeRegister(0xFF, 0x01);
		writeRegister(0x00, 0x00);
		writeRegister(0x91, _stop_variable);
		writeRegister(0x00, 0x01);
		writeRegister(0xFF, 0x00);
		writeRegister(0x80, 0x00);

		writeRegister(SYSRANGE_START_REG, 0x01);

		readRegister(SYSRANGE_START_REG, system_start);

		if ((system_start & 0x01) == 1) {
			ScheduleDelayed(VL53LXX_US);
			return PX4_OK;

		} else {
			_measurement_started = true;
		}

	}

	if (!_collect_phase && !_measurement_started) {

		readRegister(SYSRANGE_START_REG, system_start);

		if ((system_start & 0x01) == 1) {
			ScheduleDelayed(VL53LXX_US);
			return PX4_OK;

		} else {
			_measurement_started = true;
		}
	}

	readRegister(RESULT_INTERRUPT_STATUS_REG, wait_for_measurement);

	if ((wait_for_measurement & 0x07) == 0) {
		ScheduleDelayed(VL53LXX_US); // reschedule every 1 ms until measurement is ready
		return PX4_OK;
	}

	_collect_phase = true;

	int ret = transfer(&cmd, sizeof(cmd), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		DEVICE_LOG("i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
}

void
VL53LXX::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
	PX4_INFO("measure interval:  %u msec", _measure_interval / 1000);
	_reports->print_info("report queue");
}

int
VL53LXX::probe()
{
	if (sensorInit() == PX4_OK) {
		return PX4_OK;
	}

	// Device not found on any address.
	return -EIO;
}

ssize_t
VL53LXX::read(device::file_t *file_pointer, char *buffer, size_t buffer_length)
{
	size_t buffer_size = 0;
	unsigned int count = buffer_length / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *read_buffer = reinterpret_cast<distance_sensor_s *>(buffer);

	// Buffer must be large enough.
	if (count < 1) {
		return -ENOSPC;
	}

	// If automatic measurement is enabled.
	if (_measure_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(read_buffer)) {
				buffer_size += sizeof(*read_buffer);
				read_buffer++;
			}
		}

		// If there was no data, warn the caller.
		if (buffer_size == 0) {
			return -EAGAIN;
		}
	}

	// Flush the report ring buffer.
	_reports->flush();

	// Trigger a measurement.
	if (measure() != PX4_OK) {
		return -EIO;
	}

	// Wait for the measurement to complete.
	usleep(_measure_interval);

	// Run the collection phase.
	if (collect() != PX4_OK) {
		return -EIO;
	}

	// State machine will have generated a report, copy it out.
	if (_reports->get(read_buffer)) {
		buffer_size = sizeof(*read_buffer);
	}

	return buffer_size;
}

int
VL53LXX::readRegister(const uint8_t reg_address, uint8_t &value)
{
	// Write register address to the sensor.
	int ret = transfer(&reg_address, sizeof(reg_address), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	// Read from the sensor.
	ret = transfer(nullptr, 0, &value, 1);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}


int
VL53LXX::readRegisterMulti(const uint8_t reg_address, uint8_t *value, const uint8_t length)
{
	// Write register address to the sensor.
	int ret = transfer(&reg_address, 1, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	// Read from the sensor.
	ret = transfer(nullptr, 0, &value[0], length);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

void
VL53LXX::RunImpl()
{
	measure();

	if (_collect_phase) {

		_collect_phase = false;
		_new_measurement = true;

		collect();

		ScheduleDelayed(_measure_interval);
	}
}

int
VL53LXX::spadCalculations()
{
	uint8_t val = 0;
	uint8_t spad_count = 0;
	uint8_t ref_spad_map[6] = {};

	bool spad_type_is_aperture = false;

	writeRegister(0x80, 0x01);
	writeRegister(0xFF, 0x01);
	writeRegister(0x00, 0x00);
	writeRegister(0xFF, 0x06);

	readRegister(0x83, val);
	writeRegister(0x83, val | 0x04);

	writeRegister(0xFF, 0x07);
	writeRegister(0x81, 0x01);
	writeRegister(0x80, 0x01);
	writeRegister(0x94, 0x6b);
	writeRegister(0x83, 0x00);

	readRegister(0x83, val);

	while (val == 0x00) {
		readRegister(0x83, val);
	}

	writeRegister(0x83, 0x01);
	readRegister(0x92, val);

	spad_count = val & 0x7f;
	spad_type_is_aperture = (val >> 7) & 0x01;

	writeRegister(0x81, 0x00);
	writeRegister(0xFF, 0x06);

	readRegister(0x83, val);
	writeRegister(0x83, val  & ~0x04);

	writeRegister(0xFF, 0x01);
	writeRegister(0x00, 0x01);
	writeRegister(0xFF, 0x00);
	writeRegister(0x80, 0x00);

	readRegisterMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG, &ref_spad_map[0], 6);

	writeRegister(0xFF, 0x01);
	writeRegister(DYNAMIC_SPAD_REF_EN_START_OFFSET_REG, 0x00);
	writeRegister(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD_REG, 0x2C);
	writeRegister(0xFF, 0x00);
	writeRegister(GLOBAL_CONFIG_REF_EN_START_SELECT_REG, 0xB4);

	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
	uint8_t spads_enabled = 0;

	for (uint8_t i = 0; i < 48; i++) {
		if (i < first_spad_to_enable || spads_enabled == spad_count) {
			ref_spad_map[i / 8] &= ~(1 << (i % 8));

		} else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
			spads_enabled++;
		}
	}

	writeRegisterMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG, &ref_spad_map[0], 6);

	sensorTuning();

	writeRegister(SYSTEM_INTERRUPT_CONFIG_GPIO_REG, 4);		// 4: GPIO interrupt on new data.
	readRegister(GPIO_HV_MUX_ACTIVE_HIGH_REG, val);

	writeRegister(GPIO_HV_MUX_ACTIVE_HIGH_REG, val & ~0x10);	// Active low.
	writeRegister(SYSTEM_INTERRUPT_CLEAR_REG, 0x01);
	writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0xE8);
	writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0x01);

	singleRefCalibration(0x40);

	writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0x02);

	singleRefCalibration(0x00);

	writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0xE8);		// Restore config.

	return OK;
}

int
VL53LXX::sensorInit()
{
	uint8_t val = 0;

	// I2C at 2.8V on sensor side of level shifter
	int ret = PX4_OK;
	ret |= readRegister(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW_REG, val);

	if (ret != PX4_OK) {
		return PX4_ERROR;
	}

	ret |= writeRegister(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW_REG, val | 0x01);

	// set I2C to standard mode
	ret |= writeRegister(0x88, 0x00);
	ret |= writeRegister(0x80, 0x01);
	ret |= writeRegister(0xFF, 0x01);
	ret |= writeRegister(0x00, 0x00);
	ret |= readRegister(0x91, val);
	ret |= writeRegister(0x00, 0x01);
	ret |= writeRegister(0xFF, 0x00);
	ret |= writeRegister(0x80, 0x00);

	if (ret != PX4_OK) {
		return PX4_ERROR;
	}

	_stop_variable = val;

	// Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	readRegister(MSRC_CONFIG_CONTROL_REG, val);
	writeRegister(MSRC_CONFIG_CONTROL_REG, val | 0x12);

	// Set signal rate limit to 0.1
	float rate_limit = 0.1 * 65536;
	uint8_t rate_limit_split[2] = {};

	rate_limit_split[0] = (((uint16_t)rate_limit) >> 8);
	rate_limit_split[1] = (uint16_t)rate_limit;

	writeRegisterMulti(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG, &rate_limit_split[0], 2);
	writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0xFF);

	spadCalculations();

	return PX4_OK;
}

int
VL53LXX::sensorTuning()
{
	// Magic register settings taken from the ST Micro API.
	writeRegister(0xFF, 0x01);
	writeRegister(0x00, 0x00);
	writeRegister(0xFF, 0x00);
	writeRegister(0x09, 0x00);
	writeRegister(0x10, 0x00);
	writeRegister(0x11, 0x00);
	writeRegister(0x24, 0x01);
	writeRegister(0x25, 0xFF);
	writeRegister(0x75, 0x00);
	writeRegister(0xFF, 0x01);
	writeRegister(0x4E, 0x2C);
	writeRegister(0x48, 0x00);
	writeRegister(0x30, 0x20);
	writeRegister(0xFF, 0x00);
	writeRegister(0x30, 0x09);
	writeRegister(0x54, 0x00);
	writeRegister(0x31, 0x04);
	writeRegister(0x32, 0x03);
	writeRegister(0x40, 0x83);
	writeRegister(0x46, 0x25);
	writeRegister(0x60, 0x00);
	writeRegister(0x27, 0x00);
	writeRegister(0x50, 0x06);
	writeRegister(0x51, 0x00);
	writeRegister(0x52, 0x96);
	writeRegister(0x56, 0x08);
	writeRegister(0x57, 0x30);
	writeRegister(0x61, 0x00);
	writeRegister(0x62, 0x00);
	writeRegister(0x64, 0x00);
	writeRegister(0x65, 0x00);
	writeRegister(0x66, 0xA0);
	writeRegister(0xFF, 0x01);
	writeRegister(0x22, 0x32);
	writeRegister(0x47, 0x14);
	writeRegister(0x49, 0xFF);
	writeRegister(0x4A, 0x00);
	writeRegister(0xFF, 0x00);
	writeRegister(0x7A, 0x0A);
	writeRegister(0x7B, 0x00);
	writeRegister(0x78, 0x21);
	writeRegister(0xFF, 0x01);
	writeRegister(0x23, 0x34);
	writeRegister(0x42, 0x00);
	writeRegister(0x44, 0xFF);
	writeRegister(0x45, 0x26);
	writeRegister(0x46, 0x05);
	writeRegister(0x40, 0x40);
	writeRegister(0x0E, 0x06);
	writeRegister(0x20, 0x1A);
	writeRegister(0x43, 0x40);
	writeRegister(0xFF, 0x00);
	writeRegister(0x34, 0x03);
	writeRegister(0x35, 0x44);
	writeRegister(0xFF, 0x01);
	writeRegister(0x31, 0x04);
	writeRegister(0x4B, 0x09);
	writeRegister(0x4C, 0x05);
	writeRegister(0x4D, 0x04);
	writeRegister(0xFF, 0x00);
	writeRegister(0x44, 0x00);
	writeRegister(0x45, 0x20);
	writeRegister(0x47, 0x08);
	writeRegister(0x48, 0x28);
	writeRegister(0x67, 0x00);
	writeRegister(0x70, 0x04);
	writeRegister(0x71, 0x01);
	writeRegister(0x72, 0xFE);
	writeRegister(0x76, 0x00);
	writeRegister(0x77, 0x00);
	writeRegister(0xFF, 0x01);
	writeRegister(0x0D, 0x01);
	writeRegister(0xFF, 0x00);
	writeRegister(0x80, 0x01);
	writeRegister(0x01, 0xF8);
	writeRegister(0xFF, 0x01);
	writeRegister(0x8E, 0x01);
	writeRegister(0x00, 0x01);
	writeRegister(0xFF, 0x00);
	writeRegister(0x80, 0x00);

	return PX4_OK;
}

int
VL53LXX::singleRefCalibration(const uint8_t byte)
{
	uint8_t val = 0;

	writeRegister(SYSRANGE_START_REG, byte | 0x01);         // VL53L0X_REG_SYSRANGE_MODE_START_STOP
	readRegister(RESULT_INTERRUPT_STATUS_REG, val);

	while ((val & 0x07) == 0) {
		readRegister(RESULT_INTERRUPT_STATUS_REG, val);
	}

	writeRegister(SYSTEM_INTERRUPT_CLEAR_REG, 0x01);
	writeRegister(SYSRANGE_START_REG, 0x00);

	return PX4_OK;
}

void
VL53LXX::start()
{
	// Flush the report ring buffer.
	_reports->flush();

	// Schedule the first cycle.
	ScheduleNow();
}

int
VL53LXX::writeRegister(const uint8_t reg_address, const uint8_t value)
{
	uint8_t cmd[2] = {};
	cmd[0] = reg_address;
	cmd[1] = value;

	int ret = transfer(&cmd[0], 2, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}


int
VL53LXX::writeRegisterMulti(const uint8_t reg_address, const uint8_t *value,
			    const uint8_t length) /* bytes are send in order as they are in the array */
{
	if (length > 6 || length < 1) {
		DEVICE_LOG("VL53LXX::writeRegisterMulti length out of range");
		return PX4_ERROR;
	}

	/* be careful: for uint16_t to send first higher byte */
	uint8_t cmd[7] = {};
	cmd[0] = reg_address;

	memcpy(&cmd[1], &value[0], length);

	int ret = transfer(&cmd[0], length + 1, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

void
VL53LXX::print_usage()
{
	PRINT_MODULE_USAGE_NAME("vl53lxx", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 1, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *VL53LXX::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				       int runtime_instance)
{
	VL53LXX *instance = new VL53LXX(iterator.configuredBusOption(), iterator.bus(), cli.orientation, cli.bus_frequency);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	instance->start();
	return instance;
}


/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int vl53lxx_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = VL53LXX;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	while ((ch = cli.getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.orientation = atoi(cli.optarg());
			break;
		}
	}

	const char *verb = cli.optarg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_VL53LXX);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
