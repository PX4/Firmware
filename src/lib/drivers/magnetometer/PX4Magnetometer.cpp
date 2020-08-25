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


#include "PX4Magnetometer.hpp"

#include <lib/drivers/device/Device.hpp>

PX4Magnetometer::PX4Magnetometer(uint32_t device_id, uint8_t priority, enum Rotation rotation) :
	CDev(nullptr),
	_sensor_mag_pub{ORB_ID(sensor_mag), priority},
	_rotation{rotation}
{
	_class_device_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	_sensor_mag_pub.get().device_id = device_id;
	_sensor_mag_pub.get().scaling = 1.0f;
}

PX4Magnetometer::~PX4Magnetometer()
{
	if (_class_device_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _class_device_instance);
	}
}

int PX4Magnetometer::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case MAGIOCSSCALE: {
			// Copy offsets and scale factors in
			mag_calibration_s cal{};
			memcpy(&cal, (mag_calibration_s *) arg, sizeof(cal));

			_calibration_offset = matrix::Vector3f{cal.x_offset, cal.y_offset, cal.z_offset};
			_calibration_scale = matrix::Vector3f{cal.x_scale, cal.y_scale, cal.z_scale};
		}

		return PX4_OK;

	case MAGIOCGSCALE: {
			// copy out scale factors
			mag_calibration_s cal{};
			cal.x_offset = _calibration_offset(0);
			cal.y_offset = _calibration_offset(1);
			cal.z_offset = _calibration_offset(2);
			cal.x_scale = _calibration_scale(0);
			cal.y_scale = _calibration_scale(1);
			cal.z_scale = _calibration_scale(2);
			memcpy((mag_calibration_s *)arg, &cal, sizeof(cal));
		}

		return 0;

	case DEVIOCGDEVICEID:
		return _sensor_mag_pub.get().device_id;

	default:
		return -ENOTTY;
	}
}

void PX4Magnetometer::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _sensor_mag_pub.get().device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back to report
	_sensor_mag_pub.get().device_id = device_id.devid;
}

void PX4Magnetometer::update(hrt_abstime timestamp_sample, float x, float y, float z)
{
	sensor_mag_s &report = _sensor_mag_pub.get();
	report.timestamp = timestamp_sample;

	// Apply rotation (before scaling)
	rotate_3f(_rotation, x, y, z);

	const matrix::Vector3f raw_f{x, y, z};

	// Apply range scale and the calibrating offset/scale
	const matrix::Vector3f val_calibrated{(((raw_f.emult(_sensitivity) * report.scaling) - _calibration_offset).emult(_calibration_scale))};

	// Raw values (ADC units 0 - 65535)
	report.x_raw = x;
	report.y_raw = y;
	report.z_raw = z;

	report.x = val_calibrated(0);
	report.y = val_calibrated(1);
	report.z = val_calibrated(2);

	_sensor_mag_pub.update();
}

void PX4Magnetometer::print_status()
{
	PX4_INFO(MAG_BASE_DEVICE_PATH " device instance: %d", _class_device_instance);

	PX4_INFO("calibration scale: %.5f %.5f %.5f", (double)_calibration_scale(0), (double)_calibration_scale(1),
		 (double)_calibration_scale(2));
	PX4_INFO("calibration offset: %.5f %.5f %.5f", (double)_calibration_offset(0), (double)_calibration_offset(1),
		 (double)_calibration_offset(2));
}
