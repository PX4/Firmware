/****************************************************************************
 *
 *   Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file spi.cpp
 *
 * Base class for devices connected via SPI.
 *
 * @todo Work out if caching the mode/frequency would save any time.
 *
 * @todo A separate bus/device abstraction would allow for mixed interrupt-mode
 * and non-interrupt-mode clients to arbitrate for the bus.  As things stand,
 * a bus shared between clients of both kinds is vulnerable to races between
 * the two, where an interrupt-mode client will ignore the lock held by the
 * non-interrupt-mode client.
 */

#include "SPI.hpp"

#include <px4_platform_common/px4_config.h>
#include <nuttx/arch.h>

#ifndef CONFIG_SPI_EXCHANGE
# error This driver requires CONFIG_SPI_EXCHANGE
#endif

namespace device
{

SPI::SPI(const char *name, const char *devname, int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency) :
	CDev(name, devname),
	_device(device),
	_mode(mode),
	_frequency(frequency),
	_dev(nullptr)
{
	// fill in _device_id fields for a SPI device
	_device_id.devid_s.bus_type = DeviceBusType_SPI;
	_device_id.devid_s.bus = bus;
	// Use the 2. LSB byte as SPI address. This is currently 0, but will allow to extend
	// for multiple instances of the same device on a bus, should that ever be required.
	_device_id.devid_s.address = (uint8_t)(device >> 8);
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;

	if (!px4_spi_bus_requires_locking(bus)) {
		_locking_mode = LOCK_NONE;
	}
}

SPI::~SPI()
{
	// XXX no way to let go of the bus...
}

int
SPI::init()
{
	/* attach to the spi bus */
	if (_dev == nullptr) {
		int bus = get_device_bus();

		if (!board_has_bus(BOARD_SPI_BUS, bus)) {
			return -ENOENT;
		}

		_dev = px4_spibus_initialize(bus);
	}

	if (_dev == nullptr) {
		DEVICE_DEBUG("failed to init SPI");
		return -ENOENT;
	}

	/* deselect device to ensure high to low transition of pin select */
	SPI_SELECT(_dev, _device, false);

	/* call the probe function to check whether the device is present */
	int ret = probe();

	if (ret != OK) {
		DEVICE_DEBUG("probe failed");
		return ret;
	}

	/* do base class init, which will create the device node, etc. */
	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("cdev init failed");
		return ret;
	}

	/* tell the workd where we are */
	DEVICE_LOG("on SPI bus %d at %d (%u KHz)", get_device_bus(), PX4_SPI_DEV_ID(_device), _frequency / 1000);

	return PX4_OK;
}

int
SPI::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	int result;

	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	LockMode mode = up_interrupt_context() ? LOCK_NONE : _locking_mode;

	/* lock the bus as required */
	switch (mode) {
	default:
	case LOCK_PREEMPTION: {
			irqstate_t state = px4_enter_critical_section();
			result = _transfer(send, recv, len);
			px4_leave_critical_section(state);
		}
		break;

	case LOCK_THREADS:
		SPI_LOCK(_dev, true);
		result = _transfer(send, recv, len);
		SPI_LOCK(_dev, false);
		break;

	case LOCK_NONE:
		result = _transfer(send, recv, len);
		break;
	}

	return result;
}

int
SPI::_transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	SPI_SETFREQUENCY(_dev, _frequency);
	SPI_SETMODE(_dev, _mode);
	SPI_SETBITS(_dev, 8);
	SPI_SELECT(_dev, _device, true);

	/* do the transfer */
	SPI_EXCHANGE(_dev, send, recv, len);

	/* and clean up */
	SPI_SELECT(_dev, _device, false);

	return PX4_OK;
}

int
SPI::transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	int result;

	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	LockMode mode = up_interrupt_context() ? LOCK_NONE : _locking_mode;

	/* lock the bus as required */
	switch (mode) {
	default:
	case LOCK_PREEMPTION: {
			irqstate_t state = px4_enter_critical_section();
			result = _transferhword(send, recv, len);
			px4_leave_critical_section(state);
		}
		break;

	case LOCK_THREADS:
		SPI_LOCK(_dev, true);
		result = _transferhword(send, recv, len);
		SPI_LOCK(_dev, false);
		break;

	case LOCK_NONE:
		result = _transferhword(send, recv, len);
		break;
	}

	return result;
}

int
SPI::_transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	SPI_SETFREQUENCY(_dev, _frequency);
	SPI_SETMODE(_dev, _mode);
	SPI_SETBITS(_dev, 16);			/* 16 bit transfer */
	SPI_SELECT(_dev, _device, true);

	/* do the transfer */
	SPI_EXCHANGE(_dev, send, recv, len);

	/* and clean up */
	SPI_SELECT(_dev, _device, false);

	return PX4_OK;
}

} // namespace device
