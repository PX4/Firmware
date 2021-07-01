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

#ifndef ARGUS_HAL_TEST_H
#define ARGUS_HAL_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

__BEGIN_DECLS

/*!***************************************************************************
 * @addtogroup 	argustest
 * @{
 *****************************************************************************/

#include "argus.h"

/*!***************************************************************************
 * @brief	Version number of the HAL Self Test.
 *
 * @details Changes:
 *
 *  		- v1.0:
 *  			- Initial release.
 *  			.
 *  		- v1.1:
 *  			- Added additional print output.
 *  			- Increased tolerance for timer test to 3%.
 *  			- Fixed callback issue by disabling it after IRQ test.
 *				.
 *****************************************************************************/
#define HAL_TEST_VERSION "v1.1"

/*!***************************************************************************
 * @brief	Executes a series of tests in order to verify the HAL implementation.
 *
 * @details A series of automated tests are executed in order to verify the
 * 			implementation of the HAL required by the API.
 *
 * 			The following tests are executed:
 *
 * 			1) Timer Plausibility Test:
 *
 * 			Rudimentary tests of the lifetime counter (LTC) implementation.
 * 			This verifies that the LTC is running by checking if the returned
 * 			values of two consecutive calls to the #Timer_GetCounterValue
 * 			function are ascending. An artificial delay using the NOP operation
 * 			is induced such that the timer is not read to fast.
 *
 * 			2) Timer Wraparound Test:
 *
 *			The LTC values must wrap from 999999 μs to 0 μs and increase the
 *          seconds counter accordingly. This test verifies the correct wrapping
 *          by consecutively calling the #Timer_GetCounterValue function until
 *          at least 2 wraparound events have been occurred.
 *
 * 			3) SPI Connection Test:
 *
 * 			This test verifies the basic functionality of the SPI interface.
 * 			The test utilizes the devices laser pattern register, which can
 * 			be freely programmed by any 128-bit pattern. Thus, it writes a byte
 * 			sequence and reads back the written values on the consecutive SPI
 * 			access.
 *
 * 			4) SPI Interrupt Test:
 *
 * 			This test verifies the correct implementation of the device
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
 * 			5) GPIO Mode Test:
 *
 *          This test verifies the GPIO mode of the S2PI HAL module. This is
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
 * 			6) Timer Test:
 *
 *         	The test verifies the timer HAL implementation by comparing the
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
 *          -------------------------------------------------------------------
 *
 * 			Each test will write an error description via the print (i.e. UART)
 * 			function that shows what went wrong. Also an corresponding status is
 * 			returned in case no print functionality is available.
 *
 *
 * @param	spi_slave The SPI hardware slave, i.e. the specified CS and IRQ
 * 						lines. This is actually just a number that is passed
 * 						to the SPI interface to distinct for multiple SPI slave
 * 						devices. Note that the slave must be not equal to 0,
 * 						since is reserved for error handling.
 *
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_VerifyHALImplementation(s2pi_slave_t spi_slave);

__END_DECLS

#ifdef __cplusplus
}
#endif

/*! @} */
#endif /* ARGUS_CAL_API_H */
