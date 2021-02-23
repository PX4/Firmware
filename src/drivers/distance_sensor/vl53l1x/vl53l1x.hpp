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

/**
 * @file VL53L1X.hpp
 *
 * Driver for the ST VL53L1X ToF Sensor connected via I2C.
 */

#pragma once

#include <px4_log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>

/* ST */
#define SOFT_RESET                                          0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS                    0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND        0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS      0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS  0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS  0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM                  0x001E
#define MM_CONFIG__INNER_OFFSET_MM                          0x0020
#define MM_CONFIG__OUTER_OFFSET_MM                          0x0022
#define GPIO_HV_MUX__CTRL                                   0x0030
#define GPIO__TIO_HV_STATUS                                 0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO                       0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP                     0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI                   0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A                        0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B                        0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI                   0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO                   0x0062
#define RANGE_CONFIG__SIGMA_THRESH                          0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS         0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH                      0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD              0x006C
#define SYSTEM__THRESH_HIGH                                 0x0072
#define SYSTEM__THRESH_LOW                                  0x0074
#define SD_CONFIG__WOI_SD0                                  0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0                        0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD                    0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE       0x0080
#define SYSTEM__SEQUENCE_CONFIG                             0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD               0x0082
#define SYSTEM__INTERRUPT_CLEAR                             0x0086
#define SYSTEM__MODE_START                                  0x0087
#define VL53L1_RESULT__RANGE_STATUS                         0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0       0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD                  0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0               0x0096
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0  0x0098
#define VL53L1_RESULT__OSC_CALIBRATE_VAL                                    0x00DE
#define VL53L1_FIRMWARE__SYSTEM_STATUS                                      0x00E5
#define VL53L1_IDENTIFICATION__MODEL_ID                                     0x010F
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD                             0x013E

/* Configuration Constants */
#define VL53L1X_BASEADDR                                0x29

class VL53L1X : public device::I2C, public I2CSPIDriver<VL53L1X>
{
public:
	VL53L1X(const I2CSPIDriverConfig &config);

	~VL53L1X() override;

	static void print_usage();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_status() override;

	virtual int init() override;

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void RunImpl();

	// Distance mode member variable
	uint16_t distance_mode{VL53L1X_SHORT_RANGE};

private:
	int probe() override;

	/**
	 * Collects the most recent sensor measurement data from the i2c bus.
	 */
	int collect();

	/* ST */
	int8_t VL53L1_RdByte(uint16_t index, uint8_t *data);
	int8_t VL53L1_RdWord(uint16_t index, uint16_t *data);
	int8_t VL53L1_WrByte(uint16_t index, uint8_t data);
	int8_t VL53L1_WrWord(uint16_t index, uint16_t data);
	int8_t VL53L1_WrDWord(uint16_t index, uint32_t data);
	// __more
	int8_t VL53L1X_SensorInit();
	int8_t VL53L1X_StartRanging();
	int8_t VL53L1X_CheckForDataReady(uint8_t *isDataReady);
	int8_t VL53L1X_GetInterruptPolarity(uint8_t *pInterruptPolarity);
	int8_t VL53L1X_ClearInterrupt();
	int8_t VL53L1X_StopRanging();
	int8_t VL53L1X_GetRangeStatus(uint8_t *rangeStatus);
	int8_t VL53L1X_GetDistance(uint16_t *distance);
	// Remove if config store
	int8_t VL53L1X_ConfigBig(uint16_t DM, uint16_t TimingBudgetInMs);
	int8_t VL53L1X_SetInterMeasurementInMs(uint32_t InterMeasurementInMs);
	int8_t VL53L1X_SetROICenter(uint8_t data);
	int8_t VL53L1X_SetROI(uint16_t x, uint16_t y);
	PX4Rangefinder    _px4_rangefinder;

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};
