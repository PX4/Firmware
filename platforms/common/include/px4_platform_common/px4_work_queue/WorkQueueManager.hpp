/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <stdint.h>

namespace px4
{

class WorkQueue; // forward declaration

struct wq_config_t {
	const char *name;
	uint16_t stacksize;
	int8_t relative_priority; // relative to max
};

namespace wq_configurations
{
static constexpr wq_config_t rate_ctrl{"wq:rate_ctrl", 1664, 0}; // PX4 inner loop highest priority

static constexpr wq_config_t SPI0{"wq:SPI0", 2336, -1};
static constexpr wq_config_t SPI1{"wq:SPI1", 2336, -2};
static constexpr wq_config_t SPI2{"wq:SPI2", 2336, -3};
static constexpr wq_config_t SPI3{"wq:SPI3", 2336, -4};
static constexpr wq_config_t SPI4{"wq:SPI4", 2336, -5};
static constexpr wq_config_t SPI5{"wq:SPI5", 2336, -6};
static constexpr wq_config_t SPI6{"wq:SPI6", 2336, -7};

static constexpr wq_config_t I2C0{"wq:I2C0", 1472, -8};
static constexpr wq_config_t I2C1{"wq:I2C1", 1472, -9};
static constexpr wq_config_t I2C2{"wq:I2C2", 1472, -10};
static constexpr wq_config_t I2C3{"wq:I2C3", 1472, -11};
static constexpr wq_config_t I2C4{"wq:I2C4", 1472, -12};

// PX4 att/pos controllers, highest priority after sensors.
static constexpr wq_config_t attitude_ctrl{"wq:attitude_ctrl", 1672, -13};
static constexpr wq_config_t nav_and_controllers{"wq:nav_and_controllers", 7200, -14};

static constexpr wq_config_t hp_default{"wq:hp_default", 1900, -15};

static constexpr wq_config_t uavcan{"wq:uavcan", 3000, -16};

static constexpr wq_config_t UART0{"wq:UART0", 1400, -17};
static constexpr wq_config_t UART1{"wq:UART1", 1400, -18};
static constexpr wq_config_t UART2{"wq:UART2", 1400, -19};
static constexpr wq_config_t UART3{"wq:UART3", 1400, -20};
static constexpr wq_config_t UART4{"wq:UART4", 1400, -21};
static constexpr wq_config_t UART5{"wq:UART5", 1400, -22};
static constexpr wq_config_t UART6{"wq:UART6", 1400, -23};
static constexpr wq_config_t UART7{"wq:UART7", 1400, -24};
static constexpr wq_config_t UART8{"wq:UART8", 1400, -25};
static constexpr wq_config_t UART_UNKNOWN{"wq:UART_UNKNOWN", 1400, -26};

static constexpr wq_config_t lp_default{"wq:lp_default", 1700, -50};

static constexpr wq_config_t test1{"wq:test1", 2000, 0};
static constexpr wq_config_t test2{"wq:test2", 2000, 0};

} // namespace wq_configurations

/**
 * Start the work queue manager task.
 */
int WorkQueueManagerStart();

/**
 * Stop the work queue manager task.
 */
int WorkQueueManagerStop();

/**
 * Work queue manager status.
 */
int WorkQueueManagerStatus();

/**
 * Create (or find) a work queue with a particular configuration.
 *
 * @param new_wq		The work queue configuration (see WorkQueueManager.hpp).
 * @return		A pointer to the WorkQueue, or nullptr on failure.
 */
WorkQueue *WorkQueueFindOrCreate(const wq_config_t &new_wq);

/**
 * Map a PX4 driver device id to a work queue (by sensor bus).
 *
 * @param device_id		The PX4 driver's device id.
 * @return		A work queue configuration.
 */
const wq_config_t &device_bus_to_wq(uint32_t device_id);

/**
 * Map a serial device path (eg /dev/ttyS1) to a work queue.
 *
 * @param device_id		The device path.
 * @return		A work queue configuration.
 */
const wq_config_t &serial_port_to_wq(const char *serial);


} // namespace px4
