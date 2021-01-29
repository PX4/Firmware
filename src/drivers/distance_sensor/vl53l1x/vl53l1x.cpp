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
/**********
 * Based on the driver from ST: Ultra lite driver. UM2510; en.STSW-IMG009
***********/

#include "vl53l1x.hpp"

#define VL53L1X_SAMPLE_RATE                                20  // ms, default
#define VL53L1X_INTER_MEAS_MS				   22  //ms
/* ST */
const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
	0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
	0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
	0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
	0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
	0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
	0x00, /* 0x32 : not user-modifiable */
	0x02, /* 0x33 : not user-modifiable */
	0x08, /* 0x34 : not user-modifiable */
	0x00, /* 0x35 : not user-modifiable */
	0x08, /* 0x36 : not user-modifiable */
	0x10, /* 0x37 : not user-modifiable */
	0x01, /* 0x38 : not user-modifiable */
	0x01, /* 0x39 : not user-modifiable */
	0x00, /* 0x3a : not user-modifiable */
	0x00, /* 0x3b : not user-modifiable */
	0x00, /* 0x3c : not user-modifiable */
	0x00, /* 0x3d : not user-modifiable */
	0xff, /* 0x3e : not user-modifiable */
	0x00, /* 0x3f : not user-modifiable */
	0x0F, /* 0x40 : not user-modifiable */
	0x00, /* 0x41 : not user-modifiable */
	0x00, /* 0x42 : not user-modifiable */
	0x00, /* 0x43 : not user-modifiable */
	0x00, /* 0x44 : not user-modifiable */
	0x00, /* 0x45 : not user-modifiable */
	0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
	0x0b, /* 0x47 : not user-modifiable */
	0x00, /* 0x48 : not user-modifiable */
	0x00, /* 0x49 : not user-modifiable */
	0x02, /* 0x4a : not user-modifiable */
	0x0a, /* 0x4b : not user-modifiable */
	0x21, /* 0x4c : not user-modifiable */
	0x00, /* 0x4d : not user-modifiable */
	0x00, /* 0x4e : not user-modifiable */
	0x05, /* 0x4f : not user-modifiable */
	0x00, /* 0x50 : not user-modifiable */
	0x00, /* 0x51 : not user-modifiable */
	0x00, /* 0x52 : not user-modifiable */
	0x00, /* 0x53 : not user-modifiable */
	0xc8, /* 0x54 : not user-modifiable */
	0x00, /* 0x55 : not user-modifiable */
	0x00, /* 0x56 : not user-modifiable */
	0x38, /* 0x57 : not user-modifiable */
	0xff, /* 0x58 : not user-modifiable */
	0x01, /* 0x59 : not user-modifiable */
	0x00, /* 0x5a : not user-modifiable */
	0x08, /* 0x5b : not user-modifiable */
	0x00, /* 0x5c : not user-modifiable */
	0x00, /* 0x5d : not user-modifiable */
	0x01, /* 0x5e : not user-modifiable */
	0xcc, /* 0x5f : not user-modifiable */
	0x0f, /* 0x60 : not user-modifiable */
	0x01, /* 0x61 : not user-modifiable */
	0xf1, /* 0x62 : not user-modifiable */
	0x0d, /* 0x63 : not user-modifiable */
	0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
	0x68, /* 0x65 : Sigma threshold LSB */
	0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
	0x80, /* 0x67 : Min count Rate LSB */
	0x08, /* 0x68 : not user-modifiable */
	0xb8, /* 0x69 : not user-modifiable */
	0x00, /* 0x6a : not user-modifiable */
	0x00, /* 0x6b : not user-modifiable */
	0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
	0x00, /* 0x6d : Intermeasurement period */
	0x0f, /* 0x6e : Intermeasurement period */
	0x89, /* 0x6f : Intermeasurement period LSB */
	0x00, /* 0x70 : not user-modifiable */
	0x00, /* 0x71 : not user-modifiable */
	0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
	0x00, /* 0x73 : distance threshold high LSB */
	0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
	0x00, /* 0x75 : distance threshold low LSB */
	0x00, /* 0x76 : not user-modifiable */
	0x01, /* 0x77 : not user-modifiable */
	0x0f, /* 0x78 : not user-modifiable */
	0x0d, /* 0x79 : not user-modifiable */
	0x0e, /* 0x7a : not user-modifiable */
	0x0e, /* 0x7b : not user-modifiable */
	0x00, /* 0x7c : not user-modifiable */
	0x00, /* 0x7d : not user-modifiable */
	0x02, /* 0x7e : not user-modifiable */
	0xc7, /* 0x7f : ROI center, use SetROI() */
	0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
	0x9B, /* 0x81 : not user-modifiable */
	0x00, /* 0x82 : not user-modifiable */
	0x00, /* 0x83 : not user-modifiable */
	0x00, /* 0x84 : not user-modifiable */
	0x01, /* 0x85 : not user-modifiable */
	0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
	0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};

static const uint8_t status_rtn[24] = { 255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
					255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
					255, 255, 11, 12
				      };

/* end ST */

<<<<<<< HEAD
VL53L1X::VL53L1X(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_rangefinder(get_device_id(), config.rotation)
{
=======
VL53L1X::VL53L1X(I2CSPIBusOption bus_option, const int bus, const uint8_t rotation, int bus_frequency, int address) :
	I2C(DRV_DIST_DEVTYPE_VL53L1X, MODULE_NAME, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_rangefinder(get_device_id(), rotation)
{
>>>>>>> cleaned up merge conflicts w/ upstream remote master, added region of interest center setting and size
	//Set distance mode (1 for ~2m ranging, 2 for ~4m ranging
	distance_mode = 1;

	// VL53L1X typical range 0-4 meters with 27 degree field of view
	_px4_rangefinder.set_min_distance(0.f);

	if (distance_mode == 1) {
	    _px4_rangefinder.set_max_distance(2.f);
	}
        else {
	    _px4_rangefinder.set_max_distance(4.f);
	}

	_px4_rangefinder.set_fov(math::radians(27.f));

	// Allow 3 retries as the device typically misses the first measure attempts.
	I2C::_retries = 3;

	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_VL53L1X);
}

VL53L1X::~VL53L1X()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int VL53L1X::collect()
{
	uint8_t ret = 0;
	uint8_t rangeStatus = 0;
	uint16_t distance_mm = 0;

	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	ret = VL53L1X_GetRangeStatus(&rangeStatus);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	ret = VL53L1X_GetDistance(&distance_mm);
	ret |= VL53L1X_ClearInterrupt();

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	perf_end(_sample_perf);

	float distance_m = distance_mm / 1000.f;

	_px4_rangefinder.update(timestamp_sample, distance_m);

	return PX4_OK;
}

void VL53L1X::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
}

int VL53L1X::probe()
{
	uint8_t ret, data;
	ret = VL53L1_RdByte(1, &data);

	if ((ret != PX4_OK) | (data != _device_id.devid_s.address)) {
		return -EIO;
	}

	return PX4_OK;
}

void VL53L1X::RunImpl()
{
	uint8_t dataReady = 0;

	uint8_t roiCenter[] = {239, 215, 191, 167, 151};
        static uint8_t zone;

	VL53L1X_CheckForDataReady(&dataReady);

	if (dataReady == 1) {
                collect();
	}

	ScheduleDelayed(VL53L1X_SAMPLE_RATE);

	//zone modulus & change center
	zone = zone% 5;
	VL53L1X_SetROICenter(roiCenter[zone]);

        //reset inf. counter check
	if(zone >= sizeof(roiCenter)){
	        zone = 0;
	}
        //increment
	zone++;
}

int VL53L1X::init()
{
	int ret = PX4_OK;
        uint8_t x, y;
	ret = device::I2C::init();

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return PX4_ERROR;
	}
	//Spad width (x) & height (y)
        x = 4;
	y = 16;

	ret |= VL53L1X_SensorInit();
	ret |= VL53L1X_ConfigBig(distance_mode, VL53L1X_SAMPLE_RATE);
	ret |= VL53L1X_SetROI(x, y);
	ret |= VL53L1X_SetInterMeasurementInMs(VL53L1X_INTER_MEAS_MS);
	ret |= VL53L1X_StartRanging();

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	PX4_DEBUG("vl53l1x init success");
	ScheduleNow();
	return PX4_OK;
}

void VL53L1X::print_usage()
{
	PRINT_MODULE_USAGE_NAME("vl53l1x", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x29);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 1, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

/* ST */
int8_t VL53L1X::VL53L1_RdByte(uint16_t index, uint8_t *data)
{
	int8_t ret;
	uint8_t data_local[2];

	// Convert
	data_local[0] = (index >> 8) & 0xff;
	data_local[1] = index & 0xff;

	// Write register address to the sensor.
	ret = transfer(&data_local[0], sizeof(data_local), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	// Read from the sensor.
	ret = transfer(nullptr, 0, data, 1);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

int8_t VL53L1X::VL53L1_RdWord(uint16_t index, uint16_t *data)
{
	int8_t ret;
	uint8_t data_local[2], data_out[2];

	// Convert
	data_local[0] = (index >> 8) & 0xff;
	data_local[1] = index & 0xff;

	// Write register address to the sensor.
	ret = transfer(&data_local[0], sizeof(data_local), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	// Read from the sensor.
	ret = transfer(nullptr, 0, data_out, 2);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	*data = ((uint16_t)data_out[0] << 8) + (uint16_t)data_out[1];

	return PX4_OK;
}

int8_t VL53L1X::VL53L1_WrByte(uint16_t index, uint8_t data)
{
	int8_t ret;
	uint8_t data_local[3];

	// Convert
	data_local[0] = (index >> 8) & 0xff;
	data_local[1] = index & 0xff;
	data_local[2] = data;

	// Write register address to the sensor.
	ret = transfer(&data_local[0], sizeof(data_local), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

int8_t VL53L1X::VL53L1_WrWord(uint16_t index, uint16_t data)
{
	int8_t ret;
	uint8_t data_local[4];

	// Convert
	data_local[0] = (index >> 8) & 0xff;
	data_local[1] = index & 0xff;
	data_local[2] = (data >> 8) & 0xff;
	data_local[3] = data & 0xff;

	// Write register address to the sensor.
	ret = transfer(&data_local[0], sizeof(data_local), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

int8_t VL53L1X::VL53L1_WrDWord(uint16_t index, uint32_t data)
{
	int8_t ret;
	uint8_t data_local[6];

	// Convert
	data_local[0] = (index >> 8) & 0xff;
	data_local[1] = index & 0xff;
	data_local[2] = (data >> 24) & 0xff;
	data_local[3] = (data >> 16) & 0xff;
	data_local[4] = (data >> 8) & 0xff;
	data_local[5] = data & 0xff;

	// Write register address to the sensor.
	ret = transfer(&data_local[0], sizeof(data_local), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

// __more
int8_t VL53L1X::VL53L1X_SensorInit()
{
	int8_t status = 0;
	uint8_t tmp = 0;
	uint16_t Addr = 0x0000;

	for (Addr = 0x2D; Addr <= 0x87; Addr++) {
		status |= VL53L1_WrByte(Addr, VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D]);
	}

	status |= VL53L1X_StartRanging();

	while (tmp == 0) {
		status = VL53L1X_CheckForDataReady(&tmp);
	}

	status = VL53L1X_ClearInterrupt();
	status = VL53L1X_StopRanging();
	status = VL53L1_WrByte(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
	status = VL53L1_WrByte(0x0B, 0); /* start VHV from the previous temperature */
	return status;
}

int8_t VL53L1X::VL53L1X_StartRanging()
{
	int8_t status = 0;

	status = VL53L1_WrByte(SYSTEM__MODE_START, 0x40);    /* Enable VL53L1X */
	return status;
}

/**
 * @brief This function checks if the new ranging data is available by polling the dedicated register.
 * @param : isDataReady==0 -> not ready; isDataReady==1 -> ready
 */
int8_t VL53L1X::VL53L1X_CheckForDataReady(uint8_t *isDataReady)
{
	uint8_t Temp;
	uint8_t IntPol;
	int8_t status = 0;

	status = VL53L1X_GetInterruptPolarity(&IntPol);
	status = VL53L1_RdByte(GPIO__TIO_HV_STATUS, &Temp);

	/* Read in the register to check if a new value is available */
	if (status == 0) {
		if ((Temp & 1) == IntPol) {
			*isDataReady = 1;

		} else {
			*isDataReady = 0;
		}
	}

	return status;
}

int8_t VL53L1X::VL53L1X_GetInterruptPolarity(uint8_t *pInterruptPolarity)
{
	uint8_t Temp;
	int8_t status = 0;

	status = VL53L1_RdByte(GPIO_HV_MUX__CTRL, &Temp);
	Temp = Temp & 0x10;
	*pInterruptPolarity = !(Temp >> 4);
	return status;
}

int8_t VL53L1X::VL53L1X_ClearInterrupt()
{
	int8_t status = 0;

	status = VL53L1_WrByte(SYSTEM__INTERRUPT_CLEAR, 0x01);
	return status;
}

int8_t VL53L1X::VL53L1X_StopRanging()
{
	int8_t status = 0;

	status = VL53L1_WrByte(SYSTEM__MODE_START, 0x00);    /* Disable VL53L1X */
	return status;
}

int8_t VL53L1X::VL53L1X_SetROI(uint16_t x, uint16_t y)
{
	int8_t status = 0;

	status = VL53L1_WrByte(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,  (y - 1) << 4 | (x - 1));    /* set ROI size x and y */

	return status;
}

int8_t VL53L1X::VL53L1X_SetROICenter(uint8_t zone)
{
	int8_t status = 0;

	status = VL53L1_WrByte(VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD, zone);    /* Set ROI spad center */
	return status;
}

/**
 * @brief This function programs the Intermeasurement period in ms\n
 * Intermeasurement period must be >/= timing budget.
 * This condition is not checked by the API,
 * the customer has the duty to check the condition. Default = 100 ms
 */
int8_t VL53L1X::VL53L1X_SetInterMeasurementInMs(uint32_t InterMeasMs)
{
	uint16_t ClockPLL = 0;
	int8_t status = 0;

	status = VL53L1_RdWord(VL53L1_RESULT__OSC_CALIBRATE_VAL, &ClockPLL);
	ClockPLL = ClockPLL & 0x3FF;
	VL53L1_WrDWord(VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD,
		       (uint32_t)(ClockPLL * InterMeasMs * 1.075));
	return status;
}

/**
 * @brief This function returns the ranging status error \n
 * (0:no error, 1:sigma failed, 2:signal failed, ..., 7:wrap-around)
 */
int8_t VL53L1X::VL53L1X_GetRangeStatus(uint8_t *rangeStatus)
{
	int8_t status = 0;
	uint8_t RgSt;

	*rangeStatus = 255;
	status = VL53L1_RdByte(VL53L1_RESULT__RANGE_STATUS, &RgSt);
	RgSt = RgSt & 0x1F;

	if (RgSt < 24) {
		*rangeStatus = status_rtn[RgSt];
	}

	return status;
}

/**
 * @brief This function returns the distance measured by the sensor in mm
 */
int8_t VL53L1X::VL53L1X_GetDistance(uint16_t *distance)
{
	int8_t status = 0;
	uint16_t tmp = 0;

	status = (VL53L1_RdWord(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp));
	*distance = tmp;
	return status;
}

/**
 *  Config
 * @brief This function programs the distance mode (1=short, 2=long(default)).
 * Short mode max distance is limited to 1.3 m but better ambient immunity.\n
 * Long mode can range up to 4 m in the dark with 200 ms timing budget.
 */
int8_t VL53L1X::VL53L1X_ConfigBig(uint16_t DM, uint16_t TimingBudgetInMs)
{
	int8_t status = 0;

	switch (DM) {
	case 1:
		status |= VL53L1_WrByte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
		status |= VL53L1_WrByte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
		status |= VL53L1_WrByte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
		status |= VL53L1_WrByte(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
		status |= VL53L1_WrWord(SD_CONFIG__WOI_SD0, 0x0705);
		status |= VL53L1_WrWord(SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
		break;

	case 2:
		status |= VL53L1_WrByte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
		status |= VL53L1_WrByte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
		status |= VL53L1_WrByte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
		status |= VL53L1_WrByte(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
		status |= VL53L1_WrWord(SD_CONFIG__WOI_SD0, 0x0F0D);
		status |= VL53L1_WrWord(SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
		break;

	default:
		status = 1;
		break;
	}

	// ----

	if (DM == 0) {
		return 1;

	} else if (DM == 1) {  /* Short DistanceMode */
		switch (TimingBudgetInMs) {
		case 15: /* only available in short distance mode */
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01D);
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027);
			break;

		case 20:
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051);
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
			break;

		case 33:
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00D6);
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
			break;

		case 50:
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x1AE);
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01E8);
			break;

		case 100:
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02E1);
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388);
			break;

		case 200:
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03E1);
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496);
			break;

		default:
			status = 1;
			break;
		}

	} else {
		switch (TimingBudgetInMs) {
		case 20:
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001E);
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022);
			break;

		case 33:
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060);
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
			break;

		case 50:
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00AD);
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00C6);
			break;

		case 100:
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01CC);
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01EA);
			break;

		case 200:
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02D9);
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02F8);
			break;

		case 500:
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048F);
			VL53L1_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04A4);
			break;

		default:
			status = 1;
			break;
		}
	}

	return status;
}

/* end ST */

extern "C" __EXPORT int vl53l1x_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = VL53L1X;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.rotation = (Rotation)distance_sensor_s::ROTATION_DOWNWARD_FACING;
	cli.i2c_address = VL53L1X_BASEADDR;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_VL53L1X);

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
