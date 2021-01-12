/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include <lib/parameters/param.h>
#include <px4_platform_common/log.h>

#include "factory_calibration_storage.h"


static const char *CALIBRATION_STORAGE = "/fs/mtd_caldata";

static bool filter_calibration_params(param_t handle)
{
	const char *name = param_name(handle);
	// filter all non-calibration params
	return strncmp(name, "CAL_", 4) == 0 || strncmp(name, "TC_", 3) == 0;
}

FactoryCalibrationStorage::FactoryCalibrationStorage()
{
	int32_t param = 0;
	param_get(param_find("SYS_FAC_CAL_MODE"), &param);
	_enabled = param == 1;
}

int FactoryCalibrationStorage::open()
{
	if (_fd >= 0) {
		cleanup();
	}

	if (!_enabled) {
		return 0;
	}

	_fd = ::open(CALIBRATION_STORAGE, O_RDWR);

	if (_fd == -1) {
		return -errno;
	}

	PX4_INFO("Storing parameters to factory storage %s", CALIBRATION_STORAGE);
	param_control_autosave(false);
	return 0;
}

int FactoryCalibrationStorage::store()
{
	if (!_enabled) {
		return 0;
	}

	int ret = param_export(_fd, false, filter_calibration_params);

	if (ret != 0) {
		PX4_ERR("param export failed (%i)", ret);
	}

	return ret;
}

void FactoryCalibrationStorage::cleanup()
{
	if (_enabled) {
		param_control_autosave(true);
	}

	if (_fd >= 0) {
		close(_fd);
		_fd = -1;
	}
}

