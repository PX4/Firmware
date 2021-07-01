/*************************************************************************//**
 * @file	argus_hal_test.c
 * @brief	Tests for the AFBR-S50 API hardware abstraction layer.
 *
 * @copyright
 *
 * Copyright (c) 2021, Broadcom, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/


/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "argus_hal_test.h"

#include "platform/argus_print.h"
#include "platform/argus_s2pi.h"
#include "platform/argus_timer.h"
#include "platform/argus_nvm.h"
#include "platform/argus_irq.h"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! An error log message via print(); */
#define error_log(fmt, ...) print("ERROR: " fmt "\n", ##__VA_ARGS__)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static status_t TimerPlausibilityTest(void);
static status_t TimerWraparoundTest(void);
static status_t SpiConnectionTest(s2pi_slave_t slave);
static status_t SpiInterruptTest(s2pi_slave_t slave);
static status_t GpioModeTest(s2pi_slave_t slave);
static status_t TimerTest(s2pi_slave_t slave);

static status_t CheckTimerCounterValues(uint32_t hct, uint32_t lct);
static status_t SPITransferSync(s2pi_slave_t slave, uint8_t *data, uint8_t size);
static status_t ConfigureDevice(s2pi_slave_t slave, int8_t rcoTrim);
static status_t TriggerMeasurement(s2pi_slave_t slave, uint16_t samples);
static status_t AwaitDataReady(s2pi_slave_t slave, uint32_t timeout_ms);
static status_t ReadEEPROM(s2pi_slave_t slave, uint8_t *eeprom);
static status_t ReadRcoTrim(s2pi_slave_t slave, int8_t *RcoTrim);
static status_t RunMeasurement(s2pi_slave_t slave, uint16_t samples);

static void DataReadyCallback(void *param);

extern uint32_t EEPROM_ReadChipId(uint8_t const *eeprom);
extern argus_module_version_t EEPROM_ReadModule(uint8_t const *eeprom);
extern status_t EEPROM_Read(s2pi_slave_t slave, uint8_t address, uint8_t *data);
extern uint8_t hamming_decode(uint8_t const *code, uint8_t *data);

/******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t Argus_VerifyHALImplementation(s2pi_slave_t spi_slave)
{
	status_t status = STATUS_OK;

	PX4_INFO_RAW("########################################################\n");
	PX4_INFO_RAW("#   Running HAL Verification Test - " HAL_TEST_VERSION "\n");
	PX4_INFO_RAW("########################################################\n\n");

	PX4_INFO_RAW("1 > Timer Plausibility Test\n");
	status = TimerPlausibilityTest();

	if (status != STATUS_OK) { goto summary; }

	PX4_INFO_RAW("1 > PASS\n\n");

	PX4_INFO_RAW("2 > Timer Wraparound Test\n");
	status = TimerWraparoundTest();

	if (status != STATUS_OK) { goto summary; }

	PX4_INFO_RAW("2 > PASS\n\n");

	PX4_INFO_RAW("3 > SPI Connection Test\n");
	status = SpiConnectionTest(spi_slave);

	if (status != STATUS_OK) { goto summary; }

	PX4_INFO_RAW("3 > PASS\n\n");

	PX4_INFO_RAW("4 > SPI Interrupt Test\n");
	status = SpiInterruptTest(spi_slave);

	if (status != STATUS_OK) { goto summary; }

	PX4_INFO_RAW("4 > PASS\n\n");

	PX4_INFO_RAW("5 > GPIO Mode Test\n");
	status = GpioModeTest(spi_slave);

	if (status != STATUS_OK) { goto summary; }

	PX4_INFO_RAW("5 > PASS\n\n");

	PX4_INFO_RAW("6 > Timer Test\n");
	status = TimerTest(spi_slave);

	if (status != STATUS_OK) { goto summary; }

	PX4_INFO_RAW("6 > PASS\n\n");

summary:
	PX4_INFO_RAW("########################################################\n");

	if (status != STATUS_OK) {
		PX4_INFO_RAW("#   FAIL: HAL Verification Test finished with error %d!\n", status);

	} else {
		PX4_INFO_RAW("#   PASS: HAL Verification Test finished successfully!\n");
	}

	PX4_INFO_RAW("########################################################\n\n");
	return status;
}

/*!***************************************************************************
 * @brief	Checks the validity of timer counter values.
 *
 * @details	This verifies that the counter values returned from the
 * 			#Timer_GetCounterValue function are valid. This means, the low
 * 			counter value \p lct is within 0 and 999999 μs.
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			- #STATUS_OK on success.
 * 			- #ERROR_FAIL on failure (check the error log for more information).
 *****************************************************************************/
static status_t CheckTimerCounterValues(uint32_t hct, uint32_t lct)
{
	if (lct > 999999) {
		PX4_INFO_RAW("Timer plausibility check:\n"
			     "The parameter \"lct\" of Timer_GetCounterValue() must always "
			     "be within 0 and 999999.\n"
			     "Current Values: hct = %d, lct = %d", hct, lct);
		return ERROR_FAIL;
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief	Plausibility Test for Timer HAL Implementation.
 *
 * @details	Rudimentary tests the lifetime counter (LTC) implementation.
 * 			This verifies that the LTC is running by checking if the returned
 * 			values of two consecutive calls to the #Timer_GetCounterValue
 * 			function are ascending. An artificial delay using the NOP operation
 * 			is induced such that the timer is not read to fast.
 *
 * @warning If using an ultra-fast processor with a rather low timer granularity,
 * 			the test may fail! In this case, it could help to increase the delay
 * 			by increasing the for-loop exit criteria.
 *
 * @warning	This test does not test yet verify if the timing is correct at all!
 * 			This it done in later test...
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			- #STATUS_OK on success.
 * 			- #ERROR_FAIL on failure (check the error log for more information).
 *****************************************************************************/
static status_t TimerPlausibilityTest(void)
{
	uint32_t hct0 = 0;
	uint32_t lct0 = 0;
	uint32_t hct1 = 0;
	uint32_t lct1 = 0;

	/* Get some start values */
	Timer_GetCounterValue(&hct0, &lct0);

	/* Check max value is not exceeded for LCT timer (us) */
	status_t status = CheckTimerCounterValues(hct0, lct0);

	if (status < STATUS_OK) { return status; }

	/* Adding a delay. Depending on MCU speed, this takes any time.
	 * However, the Timer should be able to solve this on any MCU. */
	for (volatile uint32_t i = 0; i < 100000; ++i) { __asm("nop"); }

	/* Get new timer value and verify some time has elapsed. */
	Timer_GetCounterValue(&hct1, &lct1);

	/* Check max value is not exceeded for LCT timer (us) */
	status = CheckTimerCounterValues(hct1, lct1);

	if (status < STATUS_OK) { return status; }

	/* Either the hct value must have been increased or the lct value if the hct
	 * value is still the same. */
	if (!((hct1 > hct0) || ((hct1 == hct0) && (lct1 > lct0)))) {
		PX4_INFO_RAW("Timer plausibility check: the elapsed time could not be "
			     "measured with the Timer_GetCounterValue() function; no time "
			     "has elapsed!\n"
			     "The delay was induced by the following code:\n"
			     "for (volatile uint32_t i = 0; i < 100000; ++i) __asm(\"nop\");\n"
			     "Current Values: hct0 = %d, lct0 = %d, hct1 = %d, lct1 = %d",
			     hct0, lct0, hct1, lct1);
		return ERROR_FAIL;
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief	Wraparound Test for the Timer HAL Implementation.
 *
 * @details The LTC values must wrap from 999999 μs to 0 μs and increase the
 *          seconds counter accordingly. This test verifies the correct wrapping
 *          by consecutively calling the #Timer_GetCounterValue function until
 *          at least 2 wraparound events have been occurred.
 *
 * @note	This test requires the timer to basically run and return ascending
 * 			values. Also, if the timer is too slow, this may take very long!
 * 			Usually, the test takes 2 seconds, since 2 wraparound events are
 * 			verified.
 *
 * @warning	This test does not test yet verify if the timing is correct at all!
 * 			This it done in later test...
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			- #STATUS_OK on success.
 * 			- #ERROR_FAIL on failure (check the error log for more information).
 *****************************************************************************/
static status_t TimerWraparoundTest(void)
{
	/* Test parameter configuration: *****************************************/
	const int8_t n = 2;		// The number of wraparounds to test.
	/*************************************************************************/

	uint32_t hct0 = 0;
	uint32_t lct0 = 0;
	uint32_t hct1 = 0;
	uint32_t lct1 = 0;

	/* Get some start values. */
	Timer_GetCounterValue(&hct0, &lct0);

	/* Check max value is not exceeded for LCT timer (us) */
	status_t status = CheckTimerCounterValues(hct0, lct0);

	if (status < STATUS_OK) { return status; }

	/* Set end after 2 seconds, i.e. 2 wrap around events. */
	uint32_t hct2 = hct0 + n;
	uint32_t lct2 = lct0;

	px4_usleep(20000);

	/* Periodically read timer values. From previous tests we
	 * already know the timer value is increasing. */
	while (hct0 < hct2 || lct0 < lct2) {
		/* add counter a , which is increasing by +1, 1000000 or 1000,
		 * different MCU different times get stuck for hard code value */
		Timer_GetCounterValue(&hct1, &lct1);

		/* Check max value is not exceeded for LCT timer (us) */
		status = CheckTimerCounterValues(hct0, lct0);

		if (status < STATUS_OK) { return status; }

		/* Testing if calls to Timer_GetCounterValue are equal or increasing.
		 * Also testing if wraparound is correctly handled.
		 * Assumption here is that two sequential calls to the get functions are
		 * only a few µs appart! I.e. if hct wraps, the new lct must be smaller
		 * than previous one. */
		if (!(((hct1 == hct0 + 1) && (lct1 < lct0))
		      || ((hct1 == hct0) && (lct1 >= lct0)))) {
			PX4_INFO_RAW("Timer plausibility check: the wraparound of \"lct\" or "
				     "\"hct\" parameters of the Timer_GetCounterValue() "
				     "function was not handled correctly!\n"
				     "Current Values: hct0 = %d, lct0 = %d, hct1 = %d, lct1 = %d",
				     hct0, lct0, hct1, lct1);
			return ERROR_FAIL;
		}

		hct0 = hct1;
		lct0 = lct1;

		px4_usleep(20000);
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief	Helper function for transfer data to SPI in blocking mode.
 *
 * @details Calls the #S2PI_TransferFrame function and waits until the transfer
 * 			has been finished by checking the #S2PI_GetStatus return code to
 * 			become #STATUS_IDLE (or #STATUS_OK).
 *
 * @warning	The test utilizes already the timer HAL in order to implement a
 * 			rudimentary timeout. However, at this time, only some basic
 * 			plausibility checks are performed on the timer HAL. I.e. if there
 * 			is an issue in the time HAL, e.g. too fast or too slow time
 * 			counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 * 			one also needs to verify the timer HAL, especially the
 * 			#Timer_GetCounterValue function.
 *
 * @param	slave The S2PI slave parameter passed to the S2PI HAL functions.
 * @param	data The data array to be transfered.
 * @param	size The size of the data array to be transfered.
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			- #STATUS_OK on success.
 * 			- #ERROR_TIMEOUT if the operation did not finished within a specified
 * 			  time (check also timer HAL implementation).
 * 			- The S2PI layer error code if #S2PI_TransferFrame or #S2PI_GetStatus
 * 			  return any negative status.
 *****************************************************************************/
static status_t SPITransferSync(s2pi_slave_t slave, uint8_t *data, uint8_t size)
{
	/* Test parameter configuration: *****************************************/
	const uint32_t timeout_ms = 100;	// The transfer timeout in ms.
	/*************************************************************************/

	status_t status = S2PI_TransferFrame(slave, data, data, size, 0, 0);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("SPI transfer failed! The call to S2PI_TransferFrame "
			     "yielded error code: %d", status);
		return status;
	}

	/* Wait until the transfer is finished using a timeout.
	 * Note: this already utilizes the timer HAL. So we might
	 * need to test the timer before the SPI connection test. */
	ltc_t start;
	Time_GetNow(&start);

	do {
		status = S2PI_GetStatus();

		if (status < STATUS_OK) {
			PX4_INFO_RAW("SPI transfer failed! The call to S2PI_GetStatus "
				     "yielded error code: %d", status);
			S2PI_Abort();
			return status;
		}

		if (Time_CheckTimeoutMSec(&start, timeout_ms)) {
			PX4_INFO_RAW("SPI transfer failed! The operation did not finished "
				     "within %d ms. This may also be caused by an invalid "
				     "timer implementation!", timeout_ms);
			return ERROR_TIMEOUT;
		}
	} while (status == STATUS_BUSY);

	return status;
}

/*!***************************************************************************
 * @brief	SPI Connection Test for S2PI HAL Implementation.
 *
 * @details This test verifies the basic functionality of the SPI interface.
 * 			The test utilizes the devices laser pattern register, which can
 * 			be freely programmed by any 128-bit pattern. Thus, it writes a byte
 * 			sequence and reads back the written values on the consecutive SPI
 * 			access.
 *
 * @warning	The test utilizes already the timer HAL in order to implement a
 * 			rudimentary timeout. However, at this time, only some basic
 * 			plausibility checks are performed on the timer HAL. I.e. if there
 * 			is an issue in the time HAL, e.g. too fast or too slow time
 * 			counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 * 			one also needs to verify the timer HAL, especially the
 * 			#Timer_GetCounterValue function.
 *
 * @param	slave The S2PI slave parameter passed to the S2PI HAL functions.
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			- #STATUS_OK on success.
 * 			- #ERROR_TIMEOUT if the operation did not finished within a specified
 * 			  time (check also timer HAL implementation).
 * 			- #ERROR_FAIL if the device access failed and the read data did not
 * 			  match the expected values.
 * 			- The S2PI layer error code if #S2PI_TransferFrame or #S2PI_GetStatus
 * 			  return any negative status.
 *****************************************************************************/
static status_t SpiConnectionTest(s2pi_slave_t slave)
{
	status_t status = STATUS_OK;
	uint8_t data[17U] = { 0 };

	/* Transfer a pattern to the register */
	data[0] = 0x04; // Laser Pattern Register Address

	for (uint8_t i = 1; i < 17U; ++i) { data[i] = i; }

	status = SPITransferSync(slave, data, 17U);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("SPI connection test failed!");
		return status;
	}

	/* Clear the laser pattern and read back previous values. */
	data[0] = 0x04; // Laser Pattern Register Address

	for (uint8_t i = 1; i < 17U; ++i) { data[i] = 0; }

	status = SPITransferSync(slave, data, 17U);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("SPI connection test failed!");
		return status;
	}

	/* Verify the read pattern. */
	for (uint8_t i = 1; i < 17U; ++i) {
		if (data[i] != i) {
			PX4_INFO_RAW("SPI connection test failed!\n"
				     "Verification of read data is invalid!\n"
				     "read_data[%d] = %d, but expected was %d",
				     i, data[i], i);
			return ERROR_FAIL;
		}
	}

	return STATUS_OK;
}


/*!***************************************************************************
 * @brief	The data ready callback invoked by the API.
 *
 * @details The callback is invoked by the API when the device GPIO IRQ is
 * 			pending after a measurement has been executed and data is ready to
 * 			be read from the device.
 *
 * @param	param The abstract pointer to the boolean value that determines if
 *                the callback is invoked.
 *****************************************************************************/
static void DataReadyCallback(void *param)
{
	IRQ_LOCK();
	*((bool *) param) = true;
	IRQ_UNLOCK();
}

/*!***************************************************************************
 * @brief	Configures the device with a bare minimum setup to run the tests.
 *
 * @details	This function applies a number of configuration values to the
 * 			device, such that a pseudo measurement w/o laser output can be
 * 			performed.
 *
 * 			A \p rcoTrim parameter can be passed to adjust the actual clock
 * 			setup.
 *
 * @warning	The test utilizes already the timer HAL in order to implement a
 * 			rudimentary timeout. However, at this time, only some basic
 * 			plausibility checks are performed on the timer HAL. I.e. if there
 * 			is an issue in the time HAL, e.g. too fast or too slow time
 * 			counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 * 			one also needs to verify the timer HAL, especially the
 * 			#Timer_GetCounterValue function.
 *
 * @param	slave The S2PI slave parameter passed to the S2PI HAL functions.
 * @param	rcoTrim The RCO Trimming value added to the nominal RCO register
 * 					value. Pass 0 if no fine tuning is required.
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			- #STATUS_OK on success.
 * 			- #ERROR_TIMEOUT if the SPI operation did not finished within a
 * 			  specified time (check also timer HAL implementation).
 * 			- The S2PI layer error code if #S2PI_TransferFrame or #S2PI_GetStatus
 * 			  return any negative status.
 *****************************************************************************/
static status_t ConfigureDevice(s2pi_slave_t slave, int8_t rcoTrim)
{
	/* Setup Device and Trigger Measurement. */
	uint16_t v = 0x0010U | (((34 + rcoTrim) & 0x3F) << 6);
	uint8_t d1[] = { 0x14, v >> 8, v & 0xFF, 0x21 };
	status_t status = SPITransferSync(slave, d1, sizeof(d1));

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Device configuration failed!");
		return status;
	}

	uint8_t d2[] = { 0x16, 0x7F, 0xFF, 0x7F, 0xE9 };
	status = SPITransferSync(slave, d2, sizeof(d2));

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Device configuration failed!");
		return status;
	}

	uint8_t d3[] = { 0x18, 0x00, 0x00, 0x03 };
	status = SPITransferSync(slave, d3, sizeof(d3));

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Device configuration failed!");
		return status;
	}

	uint8_t d4[] = { 0x10, 0x12 };
	status = SPITransferSync(slave, d4, sizeof(d4));

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Device configuration failed!");
		return status;
	}

	uint8_t d5[] = { 0x12, 0x00, 0x2B };
	status = SPITransferSync(slave, d5, sizeof(d5));

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Device configuration failed!");
		return status;
	}

	uint8_t d6[] = { 0x08, 0x04, 0x84, 0x10 };
	status = SPITransferSync(slave, d6, sizeof(d6));

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Device configuration failed!");
		return status;
	}

	uint8_t d7[] = { 0x0A, 0xFE, 0x51, 0x0F, 0x05 };
	status = SPITransferSync(slave, d7, sizeof(d7));

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Device configuration failed!");
		return status;
	}

	uint8_t d8[] = { 0x0C, 0x00, 0x00, 0x00 };
	status = SPITransferSync(slave, d8, sizeof(d8));

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Device configuration failed!");
		return status;
	}

	uint8_t d9[] = { 0x1E, 0x00, 0x00, 0x00 };
	status = SPITransferSync(slave, d9, sizeof(d9));

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Device configuration failed!");
		return status;
	}

	uint8_t d10[] = { 0x20, 0x01, 0xFF, 0xFF };
	status = SPITransferSync(slave, d10, sizeof(d10));

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Device configuration failed!");
		return status;
	}

	uint8_t d11[] = { 0x22, 0xFF, 0xFF, 0x04 };
	status = SPITransferSync(slave, d11, sizeof(d11));

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Device configuration failed!");
		return status;
	}

	return status;
}

/*!***************************************************************************
 * @brief	Triggers a measurement on the device with specified sample count.
 *
 * @details The function triggers a measurement cycle on the device. A
 * 			\p sample count can be specified to setup individual number of
 * 			digital averaging.
 *
 * @warning	The test utilizes already the timer HAL in order to implement a
 * 			rudimentary timeout. However, at this time, only some basic
 * 			plausibility checks are performed on the timer HAL. I.e. if there
 * 			is an issue in the time HAL, e.g. too fast or too slow time
 * 			counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 * 			one also needs to verify the timer HAL, especially the
 * 			#Timer_GetCounterValue function.
 *
 * @param	slave The S2PI slave parameter passed to the S2PI HAL functions.
 * @param	samples The specified number of averaging samples for the measurement.
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			- #STATUS_OK on success.
 * 			- #ERROR_TIMEOUT if the operation did not finished within a specified
 * 			  time (check also timer HAL implementation).
 * 			- The S2PI layer error code if #S2PI_TransferFrame or #S2PI_GetStatus
 * 			  return any negative status.
 *****************************************************************************/
static status_t TriggerMeasurement(s2pi_slave_t slave, uint16_t samples)
{
	// samples is zero based, i.e. writing 0 yields 1 sample
	samples = samples > 0 ? samples - 1 : samples;
	uint16_t v = 0x8000U | ((samples & 0x03FFU) << 5U);
	uint8_t d[] = { 0x1C, v >> 8, v & 0xFFU };
	status_t status = SPITransferSync(slave, d, sizeof(d));

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Trigger measurement failed!");
		return status;
	}

	return status;
}

/*!***************************************************************************
 * @brief	Waits for the data ready interrupt to be pending.
 *
 * @details The function polls the current interrupt pending state of the data
 * 			ready interrupt from the device, i.e. reads the IRQ GPIO pin until
 * 			it is pulled to low by the device.
 *
 *
 * @warning	The test utilizes already the timer HAL in order to implement a
 * 			rudimentary timeout. However, at this time, only some basic
 * 			plausibility checks are performed on the timer HAL. I.e. if there
 * 			is an issue in the time HAL, e.g. too fast or too slow time
 * 			counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 * 			one also needs to verify the timer HAL, especially the
 * 			#Timer_GetCounterValue function.
 *
 * @param	slave The S2PI slave parameter passed to the S2PI HAL functions.
 * @param	timeout_ms The timeout to cancel waiting for the IRQ.
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			- #STATUS_OK on success.
 * 			- #ERROR_TIMEOUT if either the SPI operation did not finished
 * 			  or the IRQ was not detected within a specified time (check also
 * 			  timer HAL implementation).
 * 			- The S2PI layer error code if #S2PI_TransferFrame, #S2PI_GetStatus
 * 			  or #S2PI_SetIrqCallback return any negative status.
 *****************************************************************************/
static status_t AwaitDataReady(s2pi_slave_t slave, uint32_t timeout_ms)
{
	ltc_t start;
	Time_GetNow(&start);

	while (S2PI_ReadIrqPin(slave)) {
		if (Time_CheckTimeoutMSec(&start, timeout_ms)) {
			PX4_INFO_RAW("SPI interrupt test failed! The S2PI_ReadIrqPin did not "
				     "determine an pending interrupt within %d ms.", timeout_ms);
			return ERROR_TIMEOUT;
		}
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief	SPI Interrupt Test for S2PI HAL Implementation.
 *
 * @details	This test verifies the correct implementation of the device
 * 			integration finished interrupt callback. Therefore it configures
 * 			the device with a minimal setup to run a pseudo measurement that
 * 			does not emit any laser light.
 *
 * 			Note that this test does verify the GPIO interrupt that occurs
 * 			whenever the device has finished the integration/measurement and
 * 			new data is waiting to be read from the device. This does not test
 * 			the interrupt that is triggered when the SPI transfer has finished.
 *
 * 			The data ready interrupt implies two S2PI layer functions that
 * 			are tested in this test: The #S2PI_SetIrqCallback function installs
 * 			a callback function that is invoked whenever the IRQ occurs.
 * 			The IRQ can be delayed due to higher priority task, e.g. from the
 * 			user code. It is essential for the laser safety timeout algorithm
 * 			to determine the device ready signal as fast as possible, another
 * 			method is implemented to read if the IRQ is pending but the
 * 			callback has not been reset yet. This is what the #S2PI_ReadIrqPin
 * 			function is for.
 *
 *
 * @warning The test assumes the device is in a fresh power on state and no
 * 			additional reset is required. If the test fail, one may want to
 * 			power cycle the device and try again.
 *
 * @warning	The test utilizes already the timer HAL in order to implement a
 * 			rudimentary timeout. However, at this time, only some basic
 * 			plausibility checks are performed on the timer HAL. I.e. if there
 * 			is an issue in the time HAL, e.g. too fast or too slow time
 * 			counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 * 			one also needs to verify the timer HAL, especially the
 * 			#Timer_GetCounterValue function.
 *
 * @param	slave The S2PI slave parameter passed to the S2PI HAL functions.
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			- #STATUS_OK on success.
 * 			- #ERROR_TIMEOUT if either the SPI operation did not finished
 * 			  or the IRQ was not detected within a specified time (check also
 * 			  timer HAL implementation).
 * 			- #ERROR_FAIL if the IRQ pin readout failed and the no or invalid
 * 			  interrupt was detected.
 * 			- The S2PI layer error code if #S2PI_TransferFrame, #S2PI_GetStatus
 * 			  or #S2PI_SetIrqCallback return any negative status.
 *****************************************************************************/
static status_t SpiInterruptTest(s2pi_slave_t slave)
{
	/* Test parameter configuration: *****************************************/
	const uint32_t timeout_ms = 300; // timeout for measurement, might be increased..
	/*************************************************************************/

	/* Install IRQ callback. */
	volatile bool isDataReady = false;
	status_t status = S2PI_SetIrqCallback(slave, DataReadyCallback, (void *)&isDataReady);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("SPI interrupt test failed! The call to S2PI_SetIrqCallback "
			     "yielded error code: %d", status);
		return status;
	}

	/* Check if IRQ is not yet pending. */
	if (S2PI_ReadIrqPin(slave) == 0) {
		PX4_INFO_RAW("SPI interrupt test failed! The S2PI_ReadIrqPin did "
			     "return 0 but no interrupt is pending since no "
			     "measurements are executed yet!");
		return ERROR_FAIL;
	};

	/* Setup Device. */
	status = ConfigureDevice(slave, 0);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("SPI interrupt test failed!");
		return status;
	}

	/* Trigger Measurement. */
	status = TriggerMeasurement(slave, 0);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("SPI interrupt test failed!");
		return status;
	}

	ltc_t start;
	Time_GetNow(&start);

	/* Wait for Interrupt using the S2PI_ReadIrqPin method. */
	status = AwaitDataReady(slave, timeout_ms);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("SPI interrupt test failed!");
		return status;
	}

	/* Wait for Interrupt using the callback method. */
	while (!isDataReady) {
		if (Time_CheckTimeoutMSec(&start, timeout_ms)) {
			PX4_INFO_RAW("SPI interrupt test failed! The IRQ callback was not "
				     "invoked within %d ms.", timeout_ms);
			return ERROR_TIMEOUT;
		}
	}

	/* Remove callback. */
	status = S2PI_SetIrqCallback(slave, 0, 0);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("SPI interrupt test failed! The call to S2PI_SetIrqCallback "
			     "with null pointers yielded error code: %d", status);
		return status;
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief	Reads the EEPROM bytewise and applies Hamming weight.
 * @details The EEPROM bytes are consecutevly read from the device via GPIO mode.
 * 			The #EEPROM_Read function is an internal API function that enables
 * 			the GPIO mode from the S2PI module and reads the data via a software
 * 			bit-banging protocol. Finally it disables the GPIO mode and returns
 * 			to SPI mode.
 *
 * 			The calls to S2PI HAL module is as follows:
 * 			1. S2PI_CaptureGpioControl
 * 			2. multiple calls to S2PI_WriteGpioPin and S2PI_ReadGpioPin
 * 			3. S2PI_ReleaseGpioControl
 *
 * @param	slave The S2PI slave parameter passed to the S2PI HAL functions.
 * @param	eeprom The 16 byte array to be filled with EEPROM data.
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			- #STATUS_OK on success.
 * 			- #STATUS_ARGUS_EEPROM_BIT_ERROR if the Hamming weight fails.
 * 			  interrupt was detected.
 * 			- The S2PI layer error code if #S2PI_CaptureGpioControl,
 * 			  #S2PI_ReleaseGpioControl, #S2PI_WriteGpioPin or
 * 			  #S2PI_ReadGpioPin return any negative status.
 *****************************************************************************/
static status_t ReadEEPROM(s2pi_slave_t slave, uint8_t *eeprom)
{
	/* Enable EEPROM: */
	uint8_t d1[] = { 0x12, 0x00, 0x4B };
	status_t status = SPITransferSync(slave, d1, sizeof(d1));

	if (status < STATUS_OK) {
		PX4_INFO_RAW("EEPROM readout failed (enable EEPROM), "
			     "error code: %d", status);
		return status;
	}

	uint8_t data[16] = { 0 };

	/* Readout Data */
	for (uint8_t address = 0; address < 16; address++) {
		status = EEPROM_Read(slave, address, &data[address]);

		if (status != STATUS_OK) {
			PX4_INFO_RAW("EEPROM readout failed @ address 0x%02x, "
				     "error code: %d!", address, status);
			return status;
		}
	}

	/* Disable EEPROM: */
	uint8_t d2[] = { 0x12, 0x00, 0x2B };
	status = SPITransferSync(slave, d2, sizeof(d2));

	if (status < STATUS_OK) {
		PX4_INFO_RAW("EEPROM readout failed (enable EEPROM), "
			     "error code: %d", status);
		return status;
	}

	/* Apply Hamming Code */
	uint8_t err = hamming_decode(data, eeprom);

	if (err != 0) {
		PX4_INFO_RAW("EEPROM readout failed! Failed to decoding "
			     "Hamming weight (error: %d)!", err);
		return STATUS_ARGUS_EEPROM_BIT_ERROR;
	}

	/* Add remaining bit to the end. */
	eeprom[15] = data[15] & 0x80U;

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief	GPIO Mode Test for S2PI HAL Implementation.
 *
 * @details This test verifies the GPIO mode of the S2PI HAL module. This is
 * 			done by leveraging the EEPROM readout sequence that accesses the
 * 			devices EEPROM via a software protocol that depends on the GPIO
 * 			mode.
 *
 * 			This the requires several steps, most of them are already verified
 * 			in previous tests:
 * 			- Basic device configuration and enable EEPROM.
 *  		- Read EERPOM via GPIO mode and apply Hamming weight
 *  		- Repeat several times (to eliminate random readout issues).
 *  		- Decode the EEPROM (using EEPROM_Decode in argus_cal_eeprom.c)
 *  		- Check if Module Number and Chip ID is not 0
 *
 * @param	slave The S2PI slave parameter passed to the S2PI HAL functions.
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			- #STATUS_OK on success.
 * 			- #ERROR_FAIL if the GPIO test fails.
 * 			- #STATUS_ARGUS_EEPROM_BIT_ERROR if the Hamming weight fails.
 * 			- The S2PI layer error code if #S2PI_CaptureGpioControl,
 * 			  #S2PI_ReleaseGpioControl, #S2PI_WriteGpioPin or
 * 			  #S2PI_ReadGpioPin return any negative status.
 *****************************************************************************/
static status_t GpioModeTest(s2pi_slave_t slave)
{
	/* Read EEPROM 3 times and verify. */
	uint8_t eeprom1[16] = { 0 };
	uint8_t eeprom2[16] = { 0 };
	uint8_t eeprom3[16] = { 0 };

	status_t status = ReadEEPROM(slave, eeprom1);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("GPIO mode test failed (1st attempt)!");
		return status;
	}

	status = ReadEEPROM(slave, eeprom2);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("GPIO mode test failed (2nd attempt)!");
		return status;
	}

	status = ReadEEPROM(slave, eeprom3);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("GPIO mode test failed (3rd attempt)!");
		return status;
	}

	/* Verify EEPROM data. */
	if ((memcmp(eeprom1, eeprom2, 16) != 0) ||
	    (memcmp(eeprom1, eeprom3, 16) != 0)) {
		PX4_INFO_RAW("GPIO Mode test failed (data comparison)!\n"
			     "The data from 3 distinct EEPROM readout does not match!");
		return ERROR_FAIL;
	}

	/* Check EEPROM data for reasonable chip and module number (i.e. not 0) */
	uint32_t chipID = EEPROM_ReadChipId(eeprom1);
	argus_module_version_t module = EEPROM_ReadModule(eeprom1);

	if (chipID == 0 || module == 0) {
		PX4_INFO_RAW("GPIO Mode test failed (data verification)!\n"
			     "Invalid EEPROM data: Module = %d; Chip ID = %d!", module, chipID);
		return ERROR_FAIL;
	}

	PX4_INFO_RAW("EEPROM Readout succeeded!\n");
	PX4_INFO_RAW("- Module: %d\n", module);
	PX4_INFO_RAW("- Device ID: %d\n", chipID);

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief	Reads the RCO_TRIM value from the devices EEPROM.
 *
 * @details The function reads the devices EEPROM via GPIO mode and extracts
 * 			the RCO_TRIM value from the EEPROM map.
 *
 * @param	slave The S2PI slave parameter passed to the S2PI HAL functions.
 * @param	rcotrim The read RCO_TRIM value will be returned via this pointer.
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			- #STATUS_OK on success.
 * 			- #STATUS_ARGUS_EEPROM_BIT_ERROR if the Hamming weight fails.
 * 			- #ERROR_ARGUS_UNKNOWN_MODULE if the EEPROM module number is invalid.
 * 			- The S2PI layer error code if #S2PI_CaptureGpioControl,
 * 			  #S2PI_ReleaseGpioControl, #S2PI_WriteGpioPin or
 * 			  #S2PI_ReadGpioPin return any negative status.
 *****************************************************************************/
static status_t ReadRcoTrim(s2pi_slave_t slave, int8_t *rcotrim)
{
	/* Read EEPROM */
	uint8_t eeprom[16] = { 0 };
	status_t status = ReadEEPROM(slave, eeprom);

	if (status != STATUS_OK) { return status; }

	argus_module_version_t module = EEPROM_ReadModule(eeprom);

	switch (module) {
	case AFBR_S50MV85G_V1:
	case AFBR_S50MV85G_V2:
	case AFBR_S50MV85G_V3:
	case AFBR_S50LV85D_V1:
	case AFBR_S50MV68B_V1:
	case AFBR_S50MV85I_V1:
	case AFBR_S50SV85K_V1:

		/* Read RCO Trim Value from EEPROM Map 1/2/3: */
		*rcotrim = ((int8_t) eeprom[0]) >> 3;
		break;

	case MODULE_NONE: /* Uncalibrated module; use all 0 data. */
	default:

		PX4_INFO_RAW("EEPROM Readout failed! Unknown module number: %d", module);
		return ERROR_ARGUS_UNKNOWN_MODULE;
	}

	return status;
}

/*!***************************************************************************
 * @brief	Triggers a measurement on the device and waits for the data ready
 * 			interrupt.
 *
 * @details The function triggers a measurement cycle on the device and waits
 * 			until the measurement has been finished. A \p sample count can be
 * 			specified to setup individual number of	digital averaging.
 *
 * @warning	The test utilizes already the timer HAL in order to implement a
 * 			rudimentary timeout. However, at this time, only some basic
 * 			plausibility checks are performed on the timer HAL. I.e. if there
 * 			is an issue in the time HAL, e.g. too fast or too slow time
 * 			counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 * 			one also needs to verify the timer HAL, especially the
 * 			#Timer_GetCounterValue function.
 *
 * @param	slave The S2PI slave parameter passed to the S2PI HAL functions.
 * @param	samples The specified number of averaging samples for the measurement.
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			- #STATUS_OK on success.
 * 			- #ERROR_TIMEOUT if either the SPI operation did not finished
 * 			  or the IRQ was not detected within a specified time (check also
 * 			  timer HAL implementation).
 * 			- The S2PI layer error code if #S2PI_TransferFrame, #S2PI_GetStatus
 * 			  or #S2PI_SetIrqCallback return any negative status.
 *****************************************************************************/
static status_t RunMeasurement(s2pi_slave_t slave, uint16_t samples)
{
	status_t status = TriggerMeasurement(slave, samples);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Speed test failed!\n"
			     "Call to TransferFrame returned code: %d",
			     status);
		return status;
	}

	/* Wait until the transfer is finished using a timeout. */
	status = AwaitDataReady(slave, 300);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Speed test failed!\n"
			     "SPI Read IRQ pin didn't raised, timeout activated at 200ms, error code: %d",
			     status);
		return status;
	}

	return status;
}

/*!***************************************************************************
 * @brief	Test for Timer HAL Implementation by comparing timings to the device.
 *
 * @details	The test verifies the timer HAL implementation by comparing the
 * 			timings	to the AFBR-S50 device as a reference.
 * 			Therefore several measurement are executed on the device, each with
 * 			a different averaging sample count. The elapsed time increases
 * 			linearly with the number of averaging samples. In order to remove
 * 			the time for software/setup, a linear regression fit is applied to
 * 			the measurement results and only the slope is considered for the
 * 			result. A delta of 102.4 microseconds per sample is expected.
 * 			If the measured delta per sample is within an specified error range,
 * 			the timer implementation is considered correct.
 *
 * @param	slave The S2PI slave parameter passed to the S2PI HAL functions.
 *
 * @return 	Returns the \link #status_t status\endlink:
 * 			- #STATUS_OK on success.
 * 			- #ERROR_FAIL if the timer test fails.
 * 			- #STATUS_ARGUS_EEPROM_BIT_ERROR if the EEPROM Hamming weight fails.
 * 			- #ERROR_ARGUS_UNKNOWN_MODULE if the EEPROM module number is invalid.
 * 			- #ERROR_TIMEOUT if either the SPI operation did not finished
 * 			  or the IRQ was not detected within a specified time (check also
 * 			  timer HAL implementation).
 * 			- The S2PI layer error code if #S2PI_TransferFrame, #S2PI_GetStatus,
 * 			  #S2PI_SetIrqCallback, #S2PI_CaptureGpioControl,
 * 			  #S2PI_ReleaseGpioControl, #S2PI_WriteGpioPin or #S2PI_ReadGpioPin
 * 			  return any negative status.
 *****************************************************************************/
static status_t TimerTest(s2pi_slave_t slave)
{
	/* Test parameter configuration: *****************************************/
	const int8_t n = 10;				// The number of measurements.
	const uint32_t ds = 100; 			// The step size in averaging samples.
	const float exp_slope = 102.4; 		// Expected slope is 102.4 μs / phase / sample
	const float rel_slope_error = 3e-2; // Relative slope tolerance is 3%.
	/*************************************************************************/

	/* Read RCOTrim value from EEPROM*/
	int8_t RcoTrim = 0;
	status_t status = ReadRcoTrim(slave, &RcoTrim);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Timer test failed!\n"
			     "EEPROM Read test returned code: %d", status);
		return status;
	}

	PX4_INFO_RAW("RCOTrim = %d\n", RcoTrim);

	/* Configure the device with calibrated RCO to 24MHz. */
	status = ConfigureDevice(slave, RcoTrim);

	if (status < STATUS_OK) {
		PX4_INFO_RAW("Timer test failed!\n"
			     "Configuration test returned code: %d", status);
		return status;
	}


	/* Run multiple measurements and calculate a linear regression.
	 * Note: this uses float types for simplicity. */
	float xsum = 0;
	float ysum = 0;
	float x2sum = 0;
	float xysum = 0;

	PX4_INFO_RAW("+-------+---------+------------+\n");
	PX4_INFO_RAW("| count | samples | elapsed us |\n");
	PX4_INFO_RAW("+-------+---------+------------+\n");

	for (uint8_t i = 1; i <= n; ++i) {
		ltc_t start;
		Time_GetNow(&start);

		int samples = ds * i;
		status = RunMeasurement(slave, samples);

		if (status < STATUS_OK) {
			PX4_INFO_RAW("Timer test failed!\n"
				     "Run measurement returned code: %d",
				     status);
			return status;
		}

		uint32_t elapsed_usec = Time_GetElapsedUSec(&start);

		xsum += (float) samples;
		ysum += (float) elapsed_usec;
		x2sum += (float) samples * samples;
		xysum += (float) samples * elapsed_usec;

		PX4_INFO_RAW("| %5d | %7d | %10d |\n", i, samples, elapsed_usec);
	}

	PX4_INFO_RAW("+-------+---------+------------+\n");


	const float slope = (n * xysum - xsum * ysum) / (n * x2sum - xsum * xsum);
	const float intercept = (ysum * x2sum - xsum * xysum) / (n * x2sum - xsum * xsum);
	PX4_INFO_RAW("Linear Regression: y(x) = %dE-7 sec * x + %dE-7 sec\n",
		     (int)(10 * slope), (int)(10 * intercept));

	/* Check the error of the slope. */
	const float max_slope = exp_slope * (1.f + rel_slope_error);
	const float min_slope = exp_slope * (1.f - rel_slope_error);

	if (slope > max_slope || slope < min_slope) {
		PX4_INFO_RAW("Time test failed!\n"
			     "The measured time slope does not match the expected value! "
			     "(actual: %dE-7, expected: %dE-7, min: %dE-7, max: %dE-7)\n",
			     (int)(10 * slope), (int)(10 * exp_slope),
			     (int)(10 * min_slope), (int)(10 * max_slope));
		return ERROR_FAIL;
	}

	return STATUS_OK;
}
