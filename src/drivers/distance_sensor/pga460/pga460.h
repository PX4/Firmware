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
 * @file pga460.h
 * @author Jacob Dahl <jacob.dahl@tealdrones.com>
 *
 * Driver for the TI PGA460 Ultrasonic Signal Processor and Transducer Driver
 */

#ifndef _PGA460_H
#define _PGA460_H

#include <cstring>
#include <termios.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/distance_sensor.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/tasks.h>


#define PGA460_DEFAULT_PORT "/dev/ttyS6"
#define MAX_DETECTABLE_DISTANCE          3.0f
#define MIN_DETECTABLE_DISTANCE          0.05f
#define MAX_DETECTABLE_TEMPERATURE     100.0f
#define MIN_DETECTABLE_TEMPERATURE     -20.0f
#define MODE_SET_THRESH                  0.6f
#define MODE_SET_HYST                    0.0f
#define MAX_SAMPLE_DEVIATION             0.15f
#define NUM_SAMPLES_CONSISTENT           5
// #define POLL_RATE_US 50000ULL
#define POLL_RATE_US 0ULL

#define MODE_SHORT_RANGE P1BL
#define MODE_LONG_RANGE  P2BL

#define SYNCBYTE        0x55

//      Define UART commands by name

// Single Address
#define P1BL            0x00    // Burst and Listen (Preset 1)
#define P2BL            0x01    // Burst and Listen (Preset 2)
#define P1LO            0x02    // Listen Only (Preset 1)
#define P2LO            0x03    // Listen Only (Preset 2)
#define TNLM            0x04    // Temperature and Noise-level measurement
#define UMR             0x05    // Ultrasonic Measurement Result
#define TNLR            0x06    // Temperature and noise level result
#define TEDD            0x07    // Transducer echo data dump
#define SD              0x08    // System diagnostics
#define SRR             0x09    // Register read
#define SRW             0x0A    // Register write
#define EEBR            0x0B    // EEPROM bulk read
#define EEBW            0x0C    // EEPROM bulk write
#define TVGBR           0x0D    // Time-varying-gain bulk read
#define TVGBW           0x0E    // Time-varying-gain bulk write
#define THRBR           0x0F    // Threshold bulk read
#define THRBW           0x10    // Threshold bulk write

// Broadcast -- device will execute command irrespecive of UART address field
#define BC_P1BL         0x11    // Burst and listen (Preset 1)
#define BC_P2BL         0x12    // Burst and listen (Preset 2)
#define BC_P1LO         0x13    // Listen only (Preset 1)
#define BC_P2LO         0x14    // Listen only (Preset 2)
#define BC_TNLM         0x15    // Temperature and noise-level measurement
#define BC_SRW          0x16    // Register write
#define BC_EEBW         0x17    // EEPROM bulk write
#define BC_TVGBW        0x18    // Time varying-gain bulk write
#define BC_THRBW        0x19    // Threshold bulk write

// Addresses and Settings
#define EE_CNTRL_ADDR   0x40
#define EE_UNLOCK_ST1   0x68
#define EE_UNLOCK_ST2   0x69

// EEPROM -- non-volatile
#define USER_DATA1      0xAA    //reg addr      0x0
#define USER_DATA2      0x0     //reg addr      0x1
#define USER_DATA3      0x0     //reg addr      0x2
#define USER_DATA4      0x0     //reg addr      0x3
#define USER_DATA5      0x0     //reg addr      0x4
#define USER_DATA6      0x0     //reg addr      0x5
#define USER_DATA7      0x0     //reg addr      0x6
#define USER_DATA8      0x0     //reg addr      0x7
#define USER_DATA9      0x0     //reg addr      0x8
#define USER_DATA10     0x0     //reg addr      0x9
#define USER_DATA11     0x0     //reg addr      0x0A
#define USER_DATA12     0x0     //reg addr      0x0B
#define USER_DATA13     0x0     //reg addr      0x0C
#define USER_DATA14     0x0     //reg addr      0x0D
#define USER_DATA15     0x0     //reg addr      0x0E
#define USER_DATA16     0x0     //reg addr      0x0F
#define USER_DATA17     0x0     //reg addr      0x10
#define USER_DATA18     0x0     //reg addr      0x11
#define USER_DATA19     0x0     //reg addr      0x12
#define USER_DATA20     0x0     //reg addr      0x13
#define TVGAIN0         0x9D    //reg addr      0x14
#define TVGAIN1         0xBD    //reg addr      0x15
#define TVGAIN2         0xEF    //reg addr      0x16
#define TVGAIN3         0x31    //reg addr      0x17
#define TVGAIN4         0x48    //reg addr      0x18
#define TVGAIN5         0x67    //reg addr      0x19
#define TVGAIN6         0xAC    //reg addr      0x1A
#define INIT_GAIN       0x40    //reg addr      0x1B
#define FREQUENCY   (uint8_t)(5*(_resonant_frequency - 30.0f))       //reg addr      0x1C
#define DEADTIME        0xF0    //reg addr      0x1D
#define PULSE_P1        0x0C    //reg addr      0x1E
#define PULSE_P2        0x1F    //reg addr      0x1F
#define CURR_LIM_P1     0x7F    //reg addr      0x20
#define CURR_LIM_P2     0x7F    //reg addr      0x21
#define REC_LENGTH      0x44    //reg addr      0x22
#define FREQ_DIAG       0x1B    //reg addr      0x23
#define SAT_FDIAG_TH    0x2C    //reg addr      0x24
#define FVOLT_DEC       0x7C    //reg addr      0x25
#define DECPL_TEMP      0xDF    //reg addr      0x26
#define DSP_SCALE       0x0     //reg addr      0x27
#define TEMP_TRIM       0x0     //reg addr      0x28
#define P1_GAIN_CTRL    0x0     //reg addr      0x29
#define P2_GAIN_CTRL    0x8     //reg addr      0x2A
#define EE_CRC          0x29    //reg addr      0x2B

// Register-based -- volatile
#define EE_CNTRL        0x0     //reg addr      0x40

#define BPF_A2_MSB      0x85    //reg addr      0x41
#define BPF_A2_LSB      0xEA    //reg addr      0x42
#define BPF_A3_MSB      0xF9    //reg addr      0x43
#define BPF_A3_LSB      0xA5    //reg addr      0x44
#define BPF_B1_MSB      0x3     //reg addr      0x45
#define BPF_B1_LSB      0x2D    //reg addr      0x46
#define LPF_A2_MSB      0x7E    //reg addr      0x47
#define LPF_A2_LSB      0x67    //reg addr      0x48
#define LPF_B1_MSB      0x0     //reg addr      0x49
#define LPF_B1_LSB      0xCD    //reg addr      0x4A

#define TEST_MUX        0x0     //reg addr      0x4B
#define DEV_STAT0       0x80    //reg addr      0x4C
#define DEV_STAT1       0x0     //reg addr      0x4D

// Register-based -- volatile
#define P1_THR_0        0x54    //reg addr      0x5F
#define P1_THR_1        0x5C    //reg addr      0x60
#define P1_THR_2        0xBD    //reg addr      0x61
#define P1_THR_3        0xE0    //reg addr      0x62
#define P1_THR_4        0x6     //reg addr      0x63
#define P1_THR_5        0xCF    //reg addr      0x64
#define P1_THR_6        0xEE    //reg addr      0x65
#define P1_THR_7        0x8E    //reg addr      0x66
#define P1_THR_8        0x84    //reg addr      0x67
#define P1_THR_9        0xB6    //reg addr      0x68
#define P1_THR_10       0x5A    //reg addr      0x69
#define P1_THR_11       0xFF    //reg addr      0x6A
#define P1_THR_12       0xFF    //reg addr      0x6B
#define P1_THR_13       0xFF    //reg addr      0x6C
#define P1_THR_14       0xFF    //reg addr      0x6D
#define P1_THR_15       0x0     //reg addr      0x6E
#define P2_THR_0        0xBA    //reg addr      0x6F
#define P2_THR_1        0xCF    //reg addr      0x70
#define P2_THR_2        0xFF    //reg addr      0x71
#define P2_THR_3        0xF5    //reg addr      0x72
#define P2_THR_4        0x1A    //reg addr      0x73
#define P2_THR_5        0x5F    //reg addr      0x74
#define P2_THR_6        0xFA    //reg addr      0x75
#define P2_THR_7        0xD6    //reg addr      0x76
#define P2_THR_8        0xB6    //reg addr      0x77
#define P2_THR_9        0x35    //reg addr      0x78
#define P2_THR_10       0xDF    //reg addr      0x79
#define P2_THR_11       0xFF    //reg addr      0x7A
#define P2_THR_12       0xFF    //reg addr      0x7B
#define P2_THR_13       0xFF    //reg addr      0x7C
#define P2_THR_14       0xFF    //reg addr      0x7D
#define P2_THR_15       0x0     //reg addr      0x7E
#define THR_CRC         0x1D    //reg addr      0x7F

class PGA460 : public ModuleBase<PGA460>
{
public:

	PGA460(const char *port = PGA460_DEFAULT_PORT);

	virtual ~PGA460();

	/**
	 * @see ModuleBase::custom_command().
	 * @brief main Main entry point to the module that should be
	 *        called directly from the module's main method.
	 * @param argc The input argument count.
	 * @param argv Pointer to the input argument array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase::instantiate().
	 * @brief Instantiates the pga460 object.
	 * @param argc The input argument count.
	 * @param argv Pointer to the input argument array.
	 */
	static PGA460 *instantiate(int argc, char *argv[]);

	/**
	 * @see ModuleBase::print_usage().
	 * @brief Prints the module usage to the nuttshell console.
	 * @param reason The requested reason for printing to console.
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @see ModuleBase::task_spawn().
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @brief Closes the serial port.
	 * @return Returns 0 if success or ERRNO.
	 */
	int close_serial();

	/**
	 * @brief Opens the serial port.
	 * @return Returns true if the open was successful or ERRNO.
	 */
	int open_serial();

	/**
	 * @brief Reports the diagnostic data from device status registers 1 and 2 if there is anything to report.
	 */
	void print_device_status();
	/**
	 * @brief Reports the diagnostic data the diagnostic byte (first byte from slave).
	 * @param diagnostic_byte The diagnostic byte that contains the bitflags.
	 */
	void print_diagnostics(const uint8_t diagnostic_byte);

	/**
	 * @brief Reads the threshold registers.
	 * @return Returns true if the threshold registers are set to default
	 */
	int read_threshold_registers();

	/**
	 * @see ModuleBase::run().
	 */
	void run() override;

	/**
	 * @brief Reads the EEPROM and checks to see if it matches the default parameters.
	 * @note This method is only called once at boot.
	 * @return Returns PX4_OK if the EEPROM has default values, PX4_ERROR otherwise.
	 */
	int read_eeprom();

	/**
	 * @brief Writes the user defined paramaters to device EEPROM.
	 * @return Returns true if the EEPROM was successfully written to.
	 */
	int write_eeprom();

	/**
	 * @brief Reads a register.
	 * @param reg The register to read from.
	 * @return Returns the value of the register at the specified address.
	 */
	uint8_t read_register(const uint8_t reg);

	/**
	 * @brief Writes a value to a register.
	 * @param reg The register address to write to.
	 * @param val The value to write.
	 * @return Returns true for success or false for fail.
	 */
	int write_register(const uint8_t reg, const uint8_t val);

private:

	/**
	* @brief Calculates the checksum of the transmitted commmand + data.
	* @param data Pointer to the data a checksum will be calculated for.
	* @param size The size of the data (bytes) the checksum will be calculated for.
	* @return Returns the single byte checksum.
	*/
	uint8_t calc_checksum(uint8_t *data, const uint8_t size);

	/**
	 * @brief Calculates the distance from the measurement time of flight (time_of_flight) and current temperature.
	 * @param time_of_flight The reported time of flight in ms from the device.
	 * @return Returns the distance measurement in meters.
	 */
	float calculate_object_distance(uint16_t time_of_flight);

	/**
	 * @brief Collects the data in the serial port rx buffer, does math, and publishes the value to uORB
	 * @return Returns the measurment results in format: (u16)time_of_flight, (u8)width, (u8)amplitude
	 */
	uint32_t collect_results();

	/**
	 * @brief Send the program command to the EEPROM to start the flash process.
	 */
	int flash_eeprom();

	/**
	 * @brief Writes program defined threshold defaults to the register map and checks/writes the EEPROM.
	 * @return Returns PX4_OK upon success or PX4_ERROR on fail.
	 */
	int initialize_device_settings();

	/**
	 * @brief Writes the user defined paramaters to device register map.
	 * @return Returns true if the thresholds were successfully written.
	 */
	int initialize_thresholds();

	/**
	 * @brief Measurement is read from UART RX buffer and published to the uORB distance sensor topic.
	 */
	int request_results();

	/**
	 * @brief Checks the measurement from last report and sets the range distance mode (long range , short range).
	 * @return Returns the either P1Bl or P1B2. Preset 1 (P1BL) is short range mode and preset two (P2BL) is long range mode.
	 */
	uint8_t set_range_mode();

	/**
	 * @brief Commands the device to perform an ultrasonic measurement.
	 */
	int take_measurement(const uint8_t mode);

	/*
	 * @brief Gets a temperature measurement in degrees C.
	 * @return Returns the temperature measurement in degrees C.
	 */
	float get_temperature();

	/**
	 * @brief Send the unlock command to the EEPROM to enable reading and writing -- not needed w/ bulk write
	 */
	int unlock_eeprom();

	/**
	 * @brief Commands the device to publish the measurement results to uORB.
	 * @param dist The calculated distance to the object.
	 */
	void uORB_publish_results(const float dist);

	/** @orb_advert_t orb_advert_t uORB advertisement topic. */
	orb_advert_t _distance_sensor_topic{nullptr};

	/** @param _fd Returns the file descriptor from open(). */
	int _fd{-1};

	/** @param _port Stores the port name. */
	char _port[20] {};

	/** @param _previous_report_distance The previous reported sensor distance. */
	float _previous_report_distance{0};

	/** @param _previous_valid_report_distance The previous valid reported sensor distance. */
	float _previous_valid_report_distance{0};

	/** @param _resonant_frequency The sensor resonant (transmit) frequency. */
	float _resonant_frequency{41.0f};

	/** @param _mode_long_range Flag for long range mode. If false, sensor is in short range mode. */
	uint8_t _ranging_mode{MODE_SHORT_RANGE};

	/** @param _start_loop The starting value for the loop time of the main loop. */
	uint64_t _start_loop{0};

	device::Device::DeviceId _device_id;
};

#endif
