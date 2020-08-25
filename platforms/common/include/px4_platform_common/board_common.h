/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
 *                 Author: David Sidrane <david_s5@nscdg.com>
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
 * @file board_common.h
 *
 * Provide the the common board interfaces
 */

#pragma once

/************************************************************************************
 * Included Files
 ************************************************************************************/
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/************************************************************************************
 * Definitions
 ************************************************************************************/

/* SPI bus defining tools
 *
 * For new boards we use a board_config.h to define all the SPI functionality
 *  A board provides SPI bus definitions and a set of buses that should be
 *  enumerated as well as chip selects that will be iterateable
 *
 * We will use these in macros place of the uint32_t enumeration to select a
 * specific SPI device on given SPI1 bus.
 *
 * These macros will define BUS:DEV For clarity and indexing
 *
 * The board config then defines:
 * 1) PX4_SPI_BUS_xxx Ids -the buses as a 1 based PX4_SPI_BUS_xxx as n+1 aping to SPI n
 *       where n is {1-highest SPI supported by SoC}
 * 2) PX4_SPIDEV_yyyy handles - PX4_SPIDEV_xxxxx handles using the macros below.
 * 3) PX4_xxxx_BUS_CS_GPIO - a set of chip selects that are indexed by the handles. and of set to 0 are
 *       ignored.
 * 4) PX4_xxxx_BUS_FIRST_CS and PX4_xxxxx_BUS_LAST_CS as  PX4_SPIDEV_lll for the first CS and
 *       PX4_SPIDEV_hhhh as the last CS
 *
 * Example:
 *
 * The PX4_SPI_BUS_xxx
 * #define PX4_SPI_BUS_SENSORS  1
 * #define PX4_SPI_BUS_MEMORY   2
 *
 * the PX4_SPIDEV_yyyy
 * #define PX4_SPIDEV_ICM_20689      PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,0)
 * #define PX4_SPIDEV_ICM_20602      PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,1)
 * #define PX4_SPIDEV_BMI055_GYRO    PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,2)
 *
 * The PX4_xxxx_BUS_CS_GPIO
 * #define PX4_SENSOR_BUS_CS_GPIO    {GPIO_SPI_CS_ICM20689, GPIO_SPI_CS_ICM20602, GPIO_SPI_CS_BMI055_GYR,...
 *
 * The PX4_xxxx_BUS_FIRST_CS and PX4_xxxxx_BUS_LAST_CS
 * #define PX4_SENSORS_BUS_FIRST_CS  PX4_SPIDEV_ICM_20689
 * #define PX4_SENSORS_BUS_LAST_CS   PX4_SPIDEV_BMI055_ACCEL
 *
 *
 */
#define PX4_SPIDEV_ID(type, index)  ((((type) & 0xffff) << 16) | ((index) & 0xffff))

#define PX4_SPI_DEVICE_ID         (1 << 12)
#define PX4_MK_SPI_SEL(b,d)       PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, ((((b) & 0xff) << 8) | ((d) & 0xff)))
#define PX4_SPI_BUS_ID(devid)     (((devid) >> 8) & 0xff)
#define PX4_SPI_DEV_ID(devid)     ((devid) & 0xff)
#define PX4_CHECK_ID(devid)       ((devid) & PX4_SPI_DEVICE_ID)

/* I2C PX4 clock configuration
 *
 * A board may override BOARD_I2C_BUS_CLOCK_INIT simply by defining the #defines.
 */

#if defined(BOARD_I2C_BUS_CLOCK_INIT)
#  define PX4_I2C_BUS_CLOCK_INIT BOARD_I2C_BUS_CLOCK_INIT
#else
#  if (PX4_NUMBER_I2C_BUSES) == 1
#    define PX4_I2C_BUS_CLOCK_INIT {100000}
#  elif (PX4_NUMBER_I2C_BUSES) == 2
#    define PX4_I2C_BUS_CLOCK_INIT {100000, 100000}
#  elif (PX4_NUMBER_I2C_BUSES) == 3
#    define PX4_I2C_BUS_CLOCK_INIT {100000, 100000, 100000}
#  elif (PX4_NUMBER_I2C_BUSES) == 4
#    define PX4_I2C_BUS_CLOCK_INIT {100000, 100000, 100000, 100000}
#  else
#    error PX4_NUMBER_I2C_BUSES not supported
#  endif
#endif

#ifdef BOARD_SPI_BUS_MAX_BUS_ITEMS
#define SPI_BUS_MAX_BUS_ITEMS BOARD_SPI_BUS_MAX_BUS_ITEMS
#else
#define SPI_BUS_MAX_BUS_ITEMS 6
#endif

#ifndef BOARD_NUM_SPI_CFG_HW_VERSIONS
#define BOARD_NUM_SPI_CFG_HW_VERSIONS 1
#endif

/* ADC defining tools
 * We want to normalize the V5 Sensing to V = (adc_dn) * ADC_V5_V_FULL_SCALE/(2 ^ ADC_BITS) * ADC_V5_SCALE)
 */

/* Provide overrideable defaults ADC Full scale ranges and Divider ratios
 * If the board has a different ratio or full scale range for any voltage sensing
 * the board_congig.h file should define the constants that differ from these
 * defaults
 */
#if !defined(ADC_V5_V_FULL_SCALE)
#define ADC_V5_V_FULL_SCALE             (6.6f)  // 5 volt Rail full scale voltage
#endif
#if !defined(ADC_V5_SCALE)
#define ADC_V5_SCALE                    (2.0f) // The scale factor defined by HW's resistive divider (Rt+Rb)/ Rb
#endif

#if !defined(ADC_3V3_V_FULL_SCALE)
#define ADC_3V3_V_FULL_SCALE             (3.6f)  // 3.3V volt Rail full scale voltage
#endif
#if !defined(ADC_3V3_SCALE)
#define ADC_3V3_SCALE                    (2.0f) // The scale factor defined by HW's resistive divider (Rt+Rb)/ Rb
#endif

#ifndef BOARD_ADC_POS_REF_V
#define BOARD_ADC_POS_REF_V              (3.3f) // Default reference voltage for every channels
#endif

#ifndef BOARD_ADC_POS_REF_V_FOR_CURRENT_CHAN
#define BOARD_ADC_POS_REF_V_FOR_CURRENT_CHAN (3.3f) // Reference voltage for reading out the current channel
#endif

#ifndef BOARD_ADC_POS_REF_V_FOR_VOLTAGE_CHAN
#define BOARD_ADC_POS_REF_V_FOR_VOLTAGE_CHAN (3.3f) // Reference voltage for reading out the voltage channel
#endif

/* Provide define for Bricks and Battery */

/* Define the default maximum voltage resulting from the bias on ADC termination */

#if !defined(BOARD_ADC_OPEN_CIRCUIT_V)
#  define BOARD_ADC_OPEN_CIRCUIT_V  (1.5f)
#endif

/* Define the default Under voltage Window on the LTC4417 as set by resistors on the
 * board. Default is that of the FMUv2 at 3.7V
 */

#if !defined(BOARD_VALID_UV)
#  define BOARD_VALID_UV  (3.7f)
#endif

/* Legacy default */

#if !defined(BOARD_NUMBER_BRICKS)
#  define BOARD_NUMBER_BRICKS 1
#  if !defined(BOARD_ADC_BRICK_VALID)
#    define BOARD_ADC_BRICK_VALID (1)
#  endif
#endif

#if BOARD_NUMBER_BRICKS == 0
/* allow SITL to disable all bricks */
#elif BOARD_NUMBER_BRICKS == 1
#  define BOARD_BATT_V_LIST       {ADC_BATTERY_VOLTAGE_CHANNEL}
#  define BOARD_BATT_I_LIST       {ADC_BATTERY_CURRENT_CHANNEL}
#  define BOARD_BRICK_VALID_LIST  {BOARD_ADC_BRICK_VALID}
#elif BOARD_NUMBER_BRICKS == 2
#  if  !defined(BOARD_NUMBER_DIGITAL_BRICKS)
#    define BOARD_BATT_V_LIST       {ADC_BATTERY1_VOLTAGE_CHANNEL, ADC_BATTERY2_VOLTAGE_CHANNEL}
#    define BOARD_BATT_I_LIST       {ADC_BATTERY1_CURRENT_CHANNEL, ADC_BATTERY2_CURRENT_CHANNEL}
#  endif
#define BOARD_BRICK_VALID_LIST  {BOARD_ADC_BRICK1_VALID, BOARD_ADC_BRICK2_VALID}
#elif BOARD_NUMBER_BRICKS == 3
#  define BOARD_BATT_V_LIST       {ADC_BATTERY1_VOLTAGE_CHANNEL, ADC_BATTERY2_VOLTAGE_CHANNEL, ADC_BATTERY3_VOLTAGE_CHANNEL}
#  define BOARD_BATT_I_LIST       {ADC_BATTERY1_CURRENT_CHANNEL, ADC_BATTERY2_CURRENT_CHANNEL, ADC_BATTERY3_CURRENT_CHANNEL}
#  define BOARD_BRICK_VALID_LIST  {BOARD_ADC_BRICK1_VALID, BOARD_ADC_BRICK2_VALID, BOARD_ADC_BRICK3_VALID}
#elif BOARD_NUMBER_BRICKS == 4
#  define BOARD_BATT_V_LIST       {ADC_BATTERY1_VOLTAGE_CHANNEL, ADC_BATTERY2_VOLTAGE_CHANNEL, ADC_BATTERY3_VOLTAGE_CHANNEL, ADC_BATTERY4_VOLTAGE_CHANNEL}
#  define BOARD_BATT_I_LIST       {ADC_BATTERY1_CURRENT_CHANNEL, ADC_BATTERY2_CURRENT_CHANNEL, ADC_BATTERY3_CURRENT_CHANNEL, ADC_BATTERY4_CURRENT_CHANNEL}
#  define BOARD_BRICK_VALID_LIST  {BOARD_ADC_BRICK1_VALID, BOARD_ADC_BRICK2_VALID, BOARD_ADC_BRICK3_VALID, BOARD_ADC_BRICK4_VALID}
#else
#  error Unsuported BOARD_NUMBER_BRICKS number.
#endif

/* Choose the source for ADC_SCALED_V5_SENSE */
#if defined(ADC_5V_RAIL_SENSE)
#define ADC_SCALED_V5_SENSE ADC_5V_RAIL_SENSE
#else
#  if defined(ADC_SCALED_V5_CHANNEL)
#    define ADC_SCALED_V5_SENSE ADC_SCALED_V5_CHANNEL
#  endif
#endif

/* Define the source for ADC_SCALED_V3V3_SENSORS_SENSE */

#if defined(ADC_SCALED_VDD_3V3_SENSORS_CHANNEL)
#  define ADC_SCALED_V3V3_SENSORS_SENSE ADC_SCALED_VDD_3V3_SENSORS_CHANNEL
#endif

/* Define an overridable default of 0.0f V for batery v div
 * This is done to ensure the missing default trips a low
 * voltage lockdown
 */
#if !defined(BOARD_BATTERY1_V_DIV)
#define BOARD_BATTERY1_V_DIV 0.0f
#endif

#if !defined(BOARD_BATTERY2_V_DIV)
#define BOARD_BATTERY2_V_DIV 0.0f
#endif

/* Define an overridable default of 0.0f for A per V
 * This is done to ensure the default leads to an
 * unrealistic current value
 */
#if !defined(BOARD_BATTERY1_A_PER_V)
#define BOARD_BATTERY1_A_PER_V 0.0f
#endif

#if !defined(BOARD_BATTERY2_A_PER_V)
#define BOARD_BATTERY2_A_PER_V 0.0f
#endif

/* Conditional use of PX4 PIO is Used to determine if the board
 * has a PX4IO processor.
 * We then publish the logical BOARD_USES_PX4IO
 */
#if defined(BOARD_USES_PX4IO_VERSION)
#  define BOARD_USES_PX4IO	1
#  if defined(BOARD_HAS_STATIC_MANIFEST) && BOARD_HAS_STATIC_MANIFEST == 1
#     define PX4_MFT_HW_SUPPORTED_PX4_MFT_PX4IO 1
#  endif
/*  Allow a board_config to override the PX4IO FW search paths */
#  if defined(BOARD_PX4IO_FW_SEARCH_PATHS)
#    define PX4IO_FW_SEARCH_PATHS BOARD_PX4IO_FW_SEARCH_PATHS
#  else
/*  Use PX4IO FW search paths defaults based on version */
#    if BOARD_USES_PX4IO_VERSION == 2
#      define PX4IO_FW_SEARCH_PATHS {"/etc/extras/px4_io-v2_default.bin","/fs/microsd/px4_io-v2_default.bin", "/fs/microsd/px4io2.bin", nullptr }
#    endif
#  endif
#endif

/* Provide an overridable default nop
 * for BOARD_EEPROM_WP_CTRL
 */
#if !defined(BOARD_EEPROM_WP_CTRL)
#  define BOARD_EEPROM_WP_CTRL(on_true)
#endif

/*
 * Defined when a board has capture and uses channels.
 */
#if defined(DIRECT_INPUT_TIMER_CHANNELS) && DIRECT_INPUT_TIMER_CHANNELS > 0
#define BOARD_HAS_CAPTURE 1
#endif

/*
 * Defined when a supports version and type API.
 */
#if defined(BOARD_HAS_SIMPLE_HW_VERSIONING)
#  define BOARD_HAS_VERSIONING 1
#  define HW_VER_SIMPLE(s)	     0x90000+(s)

#  define HW_VER_FMUV2           HW_VER_SIMPLE(HW_VER_FMUV2_STATE)
#  define HW_VER_FMUV3           HW_VER_SIMPLE(HW_VER_FMUV3_STATE)
#  define HW_VER_FMUV2MINI       HW_VER_SIMPLE(HW_VER_FMUV2MINI_STATE)
#  define HW_VER_FMUV2X          HW_VER_SIMPLE(HW_VER_FMUV2X_STATE)
#endif

#if defined(BOARD_HAS_HW_VERSIONING)
#  define BOARD_HAS_VERSIONING 1
#endif

/* Default LED logical to color mapping */

#if defined(BOARD_OVERLOAD_LED)
#  define BOARD_OVERLOAD_LED_TOGGLE() led_toggle(BOARD_OVERLOAD_LED)
#  define BOARD_OVERLOAD_LED_OFF()    led_off(BOARD_OVERLOAD_LED)
#else
#  define BOARD_OVERLOAD_LED_TOGGLE()
#  define BOARD_OVERLOAD_LED_OFF()
#endif

#if defined(BOARD_HAS_CONTROL_STATUS_LEDS)

#  if defined(BOARD_ARMED_LED)
#    define BOARD_ARMED_LED_TOGGLE() led_toggle(BOARD_ARMED_LED)
#    define BOARD_ARMED_LED_OFF()    led_off(BOARD_ARMED_LED)
#    define BOARD_ARMED_LED_ON()     led_on(BOARD_ARMED_LED)
#  else
#    define BOARD_ARMED_LED_TOGGLE()
#    define BOARD_ARMED_LED_OFF()
#    define BOARD_ARMED_LED_ON()
#  endif

#  if defined(BOARD_ARMED_STATE_LED)
#    define BOARD_ARMED_STATE_LED_TOGGLE() led_toggle(BOARD_ARMED_STATE_LED)
#    define BOARD_ARMED_STATE_LED_OFF()    led_off(BOARD_ARMED_STATE_LED)
#    define BOARD_ARMED_STATE_LED_ON()     led_on(BOARD_ARMED_STATE_LED)
#  else
#    define BOARD_ARMED_STATE_LED_TOGGLE()
#    define BOARD_ARMED_STATE_LED_OFF()
#    define BOARD_ARMED_STATE_LED_ON()
#  endif
#endif //

/* Provide an overridable default nop
 * for BOARD_INDICATE_ARMED_STATE
 */

#if !defined(BOARD_INDICATE_ARMED_STATE)
#  define BOARD_INDICATE_ARMED_STATE(on_armed)
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* board reset control */

typedef enum board_reset_e {
	board_reset_normal           = 0,  /* Perform a normal reset */
	board_reset_extended         = 1,  /* Perform an extend reset as defined by board */
	board_reset_power_off        = 2,  /* Reset to the boot loader, signaling a power off */
	board_reset_enter_bootloader = 3   /* Perform a reset to the boot loader */
} board_reset_e;

/* board power button state notification */

typedef enum board_power_button_state_notification_e {
	PWR_BUTTON_IDEL,                       /* Button went up without meeting shutdown button down time */
	PWR_BUTTON_DOWN,                       /* Button went Down */
	PWR_BUTTON_UP,                         /* Button went Up */
	PWR_BUTTON_REQUEST_SHUT_DOWN,          /* Button went up after meeting shutdown button down time */

	PWR_BUTTON_RESPONSE_SHUT_DOWN_PENDING, /* Response from call back board code does nothing the
                                            * expectation is that board_shutdown will be called.
                                            */
	PWR_BUTTON_RESPONSE_SHUT_DOWN_NOW,     /* Response from call back board code does shutdown now. */
} board_power_button_state_notification_e;

/* board call back signature  */

typedef int (*power_button_state_notification_t)(board_power_button_state_notification_e request);

/*  PX4_SOC_ARCH_ID is monotonic ordinal number assigned by PX4 to a chip
 *  architecture. The 2 bytes are used to create a globally unique ID when
 *  prepended to a padded Soc ID.
 */


typedef enum PX4_SOC_ARCH_ID_t {

	PX4_SOC_ARCH_ID_UNUSED         =  0x0000,

	PX4_SOC_ARCH_ID_STM32F4        =  0x0001,
	PX4_SOC_ARCH_ID_STM32F7        =  0x0002,
	PX4_SOC_ARCH_ID_KINETISK66     =  0x0003,
	PX4_SOC_ARCH_ID_SAMV7          =  0x0004,
	PX4_SOC_ARCH_ID_NXPIMXRT1062   =  0x0005,

	PX4_SOC_ARCH_ID_STM32H7        =  0x0006,

	PX4_SOC_ARCH_ID_NXPS32K146     =  0x0007,

	PX4_SOC_ARCH_ID_EAGLE          =  0x1001,
	PX4_SOC_ARCH_ID_QURT           =  0x1002,
	PX4_SOC_ARCH_ID_OCPOC          =  0x1003,
	PX4_SOC_ARCH_ID_RPI            =  0x1004,
	PX4_SOC_ARCH_ID_SIM            =  0x1005,
	PX4_SOC_ARCH_ID_SITL           =  0x1006,
	PX4_SOC_ARCH_ID_BEBOP          =  0x1007,
	PX4_SOC_ARCH_ID_BBBLUE         =  0x1008,

} PX4_SOC_ARCH_ID_t;


/* UUID
 *
 * Define the types used for board UUID, MFG UID and PX4 GUID
 *
 * A type suitable for holding the byte format of the UUID
 *
 * The original PX4 stm32 (legacy) based implementation **displayed** the
 * UUID as: ABCD EFGH IJKL
 * Where:
 *       A was bit 31 and D was bit 0
 *       E was bit 63 and H was bit 32
 *       I was bit 95 and L was bit 64
 *
 * Since the string was used by some manufactures to identify the units
 * it must be preserved. DEPRECATED - This will be removed in PX4 Release
 * 1.7.0.
 *
 * For new targets moving forward we will use an 18 byte globally unique
 * PX4 GUID in the form:
 *
 *           <ARCH MSD><ARCH LSD>[MNOP] IJKL EFGH ABCD
 *
 *  Where <ARCH MSD><ARCH LSD> are a monotonic ordinal number assigned by
 *  PX4 to a chip architecture (PX4_SOC_ARCH_ID). The 2 bytes are used to
 *  create a globally unique ID when prepended to a padded CPU UUID.
 *
 *  In the case where the MFG UUID is shorter than 16 bytes it will be
 *  padded with 0's starting at offset [2] until
 *  PX4_GUID_BYTE_LENGTH-PX4_CPU_UUID_BYTE_LENGTH -1
 *
 *  I.E. For the STM32
 *  offset:0         1     2  3  4  5  6             -            17
 *    <ARCH MSD><ARCH LSD>[0][0][0][0]<MSD CPU UUID>...<LSD CPU UUID>
 *
 *  I.E. For the Kinetis
 *  offset:0         1         2         -           17
 *    <ARCH MSD><ARCH LSD><MSD CPU UUID>...<LSD CPU UUID>
 */

/* Define the PX4 Globally unique ID (GUID) length and format size */
#define PX4_GUID_BYTE_LENGTH              18
#define PX4_GUID_FORMAT_SIZE              ((2*PX4_GUID_BYTE_LENGTH)+1)

/* DEPRICATED as of 1.7.0 A type suitable for defining the 8 bit format of the CPU UUID */
typedef uint8_t uuid_byte_t[PX4_CPU_UUID_BYTE_LENGTH];

/* DEPRICATED as of 1.7.0  A type suitable for defining the 32bit format of the CPU UUID */
typedef uint32_t uuid_uint32_t[PX4_CPU_UUID_WORD32_LENGTH];

/* A type suitable for defining the 8 bit format of the MFG UID
 * This is always returned as MSD @ index 0 -LSD @ index PX4_CPU_MFGUID_BYTE_LENGTH-1
 */
typedef uint8_t mfguid_t[PX4_CPU_MFGUID_BYTE_LENGTH];

/* A type suitable for defining the 8 bit format of the px4 globally unique
 * PX4 GUID. This is always returned as MSD @ index 0 -LSD @ index
 * PX4_CPU_GUID_BYTE_LENGTH-1
 */
typedef uint8_t px4_guid_t[PX4_GUID_BYTE_LENGTH];

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/
__BEGIN_DECLS


/************************************************************************************
 * Name: board_rc_singlewire
 *
 * Description:
 *   A board may define RC_SERIAL_SINGLEWIRE, so that RC_SERIAL_PORT is configured
 *   as singlewire UART.
 *
 * Input Parameters:
 *   device: serial device, e.g. "/dev/ttyS0"
 *
 * Returned Value:
 *   true singlewire should be enabled.
 *   false if not.
 *
 ************************************************************************************/

#if defined(RC_SERIAL_SINGLEWIRE)
static inline bool board_rc_singlewire(const char *device) { return strcmp(device, RC_SERIAL_PORT) == 0; }
#else
static inline bool board_rc_singlewire(const char *device) { return false; }
#endif

/************************************************************************************
 * Name: board_rc_swap_rxtx
 *
 * Description:
 *   A board may define RC_SERIAL_SWAP_RXTX, so that RC_SERIAL_PORT is configured
 *   as UART with RX/TX swapped.
 *
 * Input Parameters:
 *   device: serial device, e.g. "/dev/ttyS0"
 *
 * Returned Value:
 *   true RX/RX should be swapped.
 *   false if not.
 *
 ************************************************************************************/

#if defined(RC_SERIAL_SWAP_RXTX)
static inline bool board_rc_swap_rxtx(const char *device) { return strcmp(device, RC_SERIAL_PORT) == 0; }
#else
static inline bool board_rc_swap_rxtx(const char *device) { return false; }
#endif

/************************************************************************************
 * Name: board_rc_invert_input
 *
 * Description:
 *   All boards may optionally define RC_INVERT_INPUT(bool invert) that is
 *   used to invert the RC_SERIAL_PORT RC port (e.g. to toggle an external XOR via
 *   GPIO).
 *
 * Input Parameters:
 *   invert_on - A positive logic value, that when true (on) will set the HW in
 *               inverted NRZ mode where a MARK will be 0 and SPACE will be a 1.
 *
 * Returned Value:
 *   true the UART inversion got set.
 *
 ************************************************************************************/

#ifdef RC_INVERT_INPUT
static inline bool board_rc_invert_input(const char *device, bool invert)
{
	if (strcmp(device, RC_SERIAL_PORT) == 0) { RC_INVERT_INPUT(invert); return true; }

	return false;
}
#else
static inline bool board_rc_invert_input(const char *device, bool invert) { return false; }
#endif

/* Provide an interface for reading the connected state of VBUS */

/************************************************************************************
 * Name: board_read_VBUS_state
 *
 * Description:
 *   All boards must provide a way to read the state of VBUS, this my be simple
 *   digital input on a GPIO. Or something more complicated like a Analong input
 *   or reading a bit from a USB controller register.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if connected.
 *
 ************************************************************************************/

#if defined(GPIO_OTGFS_VBUS)
#  define board_read_VBUS_state() (px4_arch_gpioread(GPIO_OTGFS_VBUS) ? 0 : 1)
#else
int board_read_VBUS_state(void);
#endif

/************************************************************************************
 * Name: board_on_reset
 *
 * Description:
 * Optionally provided function called on entry to board_system_reset
 * It should perform any house keeping prior to the rest.
 * For example setting PWM outputs to the off state to avoid
 * triggering a motor spin.
 *
 * As a workaround for the bug seen on some ESC, the code will delay
 * rebooting the flight controller to insure that the 3.2 ms pulse
 * that occurs from the delay from reset to GPIO init due to memory
 * initialization is pushed out from the last PWM by > 6 Ms.
 *
 * Input Parameters:
 *  status - 1 Resetting to boot loader
 *           0 Just resetting CPU
 *           -1 used internally by board init to to initialize
 *            PWM IO pins with delay.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#if defined(BOARD_HAS_NO_RESET) || !defined(BOARD_HAS_ON_RESET)
#  define board_on_reset(status)
#else
__EXPORT void board_on_reset(int status);
#endif

/************************************************************************************
 * Name: board_reset
 *
 * Description:
 *   All boards my optionally provide this API to reset the board
 *
 * Input Parameters:
 *  status - 1 Resetting to boot loader
 *           0 Just resetting CPU
 *
 * Returned Value:
 *   If function is supported by board it will not return.
 *   If not supported it is a noop.
 *
 ************************************************************************************/
#if defined(BOARD_HAS_NO_RESET)
#  define board_system_reset(status)
#else
__EXPORT void board_system_reset(int status) noreturn_function;
#endif

/************************************************************************************
 * Name: board_set_bootload_mode
 *
 * Description:
 *   All boards my optionally provide this API to enter configure the entry to
 *   boot loader mode on the next system reset.
 *
 * Input Parameters:
 *   mode -  is an board_reset_e that controls the type of reset.
 *           board_reset_normal  Perform a normal reset
 *           board_reset_extended Perform an extend reset as defined by board
 *           board_reset_power_off Reset to the boot loader, signaling a power off
 *           board_reset_enter_bootloader  Perform a reset to the boot loader
 *
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated EINVAL value is returned if an
 *             invalid mode is requested.
 *
 ************************************************************************************/

#if defined(BOARD_HAS_NO_BOOTLOADER)
#  define board_set_bootload_mode(mode)
#else
__EXPORT int board_set_bootload_mode(board_reset_e mode);
#endif

/************************************************************************************
 * Name: board_query_manifest
 *
 * Description:
 *   Optional returns manifest item.
 *
 * Input Parameters:
 *   manifest_id - the ID for the manifest item to retrieve
 *
 * Returned Value:
 *   0 - item is not in manifest => assume legacy operations
 *   pointer to a manifest item
 *
 ************************************************************************************/

typedef enum {
	PX4_MFT_PX4IO = 0,
	PX4_MFT_USB   = 1,
	PX4_MFT_CAN2  = 2,
	PX4_MFT_CAN3  = 3,
} px4_hw_mft_item_id_t;

typedef enum {
	px4_hw_con_unknown  = 0,
	px4_hw_con_onboard  = 1,
	px4_hw_con_connector = 3,
} px4_hw_connection_t;


typedef struct {
	unsigned int present:    1;   /* 1 if this board have this item */
	unsigned int mandatory:  1;   /* 1 if this item has to be present and working */
	unsigned int connection: 2;   /* See px4_hw_connection_t */
} px4_hw_mft_item_t;

typedef const px4_hw_mft_item_t  *px4_hw_mft_item;
#define px4_hw_mft_uninitialized (px4_hw_mft_item) -1
#define px4_hw_mft_unsupported   (px4_hw_mft_item) 0

#if defined(BOARD_HAS_VERSIONING)
__EXPORT px4_hw_mft_item board_query_manifest(px4_hw_mft_item_id_t id);

#  define PX4_MFT_HW_SUPPORTED(ID)           (board_query_manifest((ID))->present)
#  define PX4_MFT_HW_REQUIRED(ID)            (board_query_manifest((ID))->mandatory)
#  define PX4_MFT_HW_IS_ONBOARD(ID)          (board_query_manifest((ID))->connection == px4_hw_con_onboard)
#  define PX4_MFT_HW_IS_OFFBOARD(ID)         (board_query_manifest((ID))->connection == px4_hw_con_connector)
#  define PX4_MFT_HW_IS_CONNECTION_KNOWN(ID) (board_query_manifest((ID))->connection != px4_hw_con_unknown)
#elif defined(BOARD_HAS_STATIC_MANIFEST) && BOARD_HAS_STATIC_MANIFEST == 1
/* Board has a static configuration and will supply what it has */
#  define PX4_MFT_HW_SUPPORTED(ID)           PX4_MFT_HW_SUPPORTED_##ID
#  define PX4_MFT_HW_REQUIRED(ID)            PX4_MFT_HW_REQUIRED_##ID
#  define PX4_MFT_HW_IS_ONBOARD(ID)          PX4_MFT_HW_IS_ONBOARD_##ID
#  define PX4_MFT_HW_IS_OFFBOARD(ID)         PX4_MFT_HW_IS_OFFBOARD_##ID
#  define PX4_MFT_HW_IS_CONNECTION_KNOWN(ID) PX4_MFT_HW_IS_CONNECTION_KNOWN_##ID
#  define board_query_manifest(_na)          px4_hw_mft_unsupported
#else
/* Default are Not Supported */
#  define PX4_MFT_HW_SUPPORTED(ID)           (0)
#  define PX4_MFT_HW_REQUIRED(ID)            (0)
#  define PX4_MFT_HW_IS_ONBOARD(ID)          (0)
#  define PX4_MFT_HW_IS_OFFBOARD(ID)         (0)
#  define PX4_MFT_HW_IS_CONNECTION_KNOWN(ID) (0)
#  define board_query_manifest(_na)          px4_hw_mft_unsupported
#endif

/************************************************************************************
 * Name: board_get_can_interfaces
 *
 * Description:
 *   Optional returns a bit mask of the enabled can interfaces, that are
 *   dependent on the on board CAN configuration.
 *
 *   In UAVCAN the number of interfaces is a compile time setting. On some HW
 *   using the same binary, all the CAN interfaces are not present.
 *
 *   The default is now 3 CAN interfaces and all active, the the build will set
 *   the actual max number of interfaces.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A bit mask of the can interfaces enabled for this board.
 *   i.e CAN1 and CAN2   0x3
 *       CAN0 and CAN1   0x3
 *
 ************************************************************************************/

#if defined(UAVCAN_NUM_IFACES_RUNTIME)
__EXPORT uint16_t board_get_can_interfaces(void);
#else
inline uint16_t board_get_can_interfaces(void) { return 0x7; }
#endif

/************************************************************************************
 * Name: board_get_hw_type_name
 *
 * Description:
 *   Optional returns a 0 terminated string defining the HW type.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   a 0 terminated string defining the HW type. This may be a 0 length string ""
 *
 ************************************************************************************/

#if defined(BOARD_HAS_VERSIONING)
__EXPORT const char *board_get_hw_type_name(void);
#else
#define board_get_hw_type_name() ""
#endif

/************************************************************************************
 * Name: board_get_hw_version
 *
 * Description:
 *   Optional returns a integer HW version
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer value of this boards hardware version.
 *   A value of -1 is the default for boards not supporting the BOARD_HAS_VERSIONING API.
 *   A value of 0 is the default for boards supporting the API but not having version.
 *
 ************************************************************************************/

#if defined(BOARD_HAS_VERSIONING)
__EXPORT int board_get_hw_version(void);
#else
#define board_get_hw_version() (-1)
#endif

/************************************************************************************
 * Name: board_get_hw_revision
 *
 * Description:
 *   Optional returns a integer HW revision
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer value of this boards hardware revision.
 *   A value of -1 is the default for boards not supporting the BOARD_HAS_VERSIONING API.
 *   A value of 0 is the default for boards supporting the API but not having revision.
 *
 ************************************************************************************/

#if defined(BOARD_HAS_VERSIONING)
__EXPORT int board_get_hw_revision(void);
#else
#define board_get_hw_revision() (-1)
#endif

/************************************************************************************
 * Name: board_get_uuid DEPRICATED use board_get_px4_guid
 *
 * Description:
 *   All boards either provide a way to read a uuid of PX4_CPU_UUID_BYTE_LENGTH
 *   from PX4_CPU_UUID_ADDRESS in the SoC's address space OR define
 *   BOARD_OVERRIDE_UUID as an array of bytes that is PX4_CPU_UUID_BYTE_LENGTH
 *
 * Input Parameters:
 *   uuid_bytes - uuid_byte_t and array of bytes PX4_CPU_UUID_BYTE_LENGTH in length.
 *
 * Returned Value:
 *   The uuid_bytes array is populated with the CPU uuid in the legacy format for
 *   STM32.
 *
 ************************************************************************************/

__EXPORT void board_get_uuid(uuid_byte_t uuid_bytes); // DEPRICATED use board_get_px4_guid

/************************************************************************************
 * Name: board_get_uuid32 DEPRICATED use board_get_px4_guid
 *
 * Description:
 *   All boards either provide a way to read a uuid of PX4_CPU_UUID_WORD32_LENGTH
 *   from PX4_CPU_UUID_ADDRESS in the Soc's address space OR define
 *   BOARD_OVERRIDE_UUID as an array of bytes that is PX4_CPU_UUID_BYTE_LENGTH
 *   On Legacy (stm32) targets the uuid_words format is the result of coping
 *   returning the 32bit words from low memory to high memory. On new targets the result
 *   will be an array of words with the MSW at index 0 and the LSW: at index
 *   PX4_CPU_UUID_WORD32_LENGTH-1.
 *
 * Input Parameters:
 *   uuid_words - a uuid_uint32_t and array of 32 bit words PX4_CPU_UUID_WORD32_
 *   LENGTH in length.
 *
 * Returned Value:
 *   The uuid_words array is populated with the CPU uuid.
 *
 ************************************************************************************/
__EXPORT void board_get_uuid32(uuid_uint32_t uuid_words); // DEPRICATED use board_get_px4_guid

/************************************************************************************
 * Name: board_get_uuid32_formated DEPRICATED use board_get_px4_guid_formated
 *
 * Description:
 *   All boards either provide a way to retrieve a uuid and format it
 *   or define BOARD_OVERRIDE_UUID
 *   This function is used to populate a buffer with the UUID to be a printed
 *   with the optional separator
 *
 * Input Parameters:
 *   format_buffer - A pointer to a bufferer of at least PX4_CPU_UUID_WORD32_FORMAT_SIZE
 *                   that will contain a 0 terminated string formated as described
 *                   the format string and optional separator.
 *   size          - The size of the buffer (should be atleaset PX4_CPU_UUID_WORD32_FORMAT_SIZE)
 *   format        - The fort mat specifier for the hex digit see CPU_UUID_FORMAT
 *   separator     - Optional pointer to a 0 terminated string or NULL:
 *                   With separator = ":"
 *                               31-00:63-32:95-64
 *                               32383336:412038:33355110
 *                   With separator = " "
 *                               31-00:63-32:95-64
 *                               32383336 412038 33355110
 *                   With separator = NULL
 *                               31-00:63-32:95-64
 *                               3238333641203833355110
 *
 * Returned Value:
 *   The format buffer is populated with a 0 terminated string formated as described.
 *   Zero (OK) is returned on success;
 *
 ************************************************************************************/
__EXPORT int board_get_uuid32_formated(char *format_buffer, int size,
				       const char *format,
				       const char *seperator); // DEPRICATED use board_get_px4_guid_formated

/************************************************************************************
 * Name: board_get_mfguid
 *
 * Description:
 *   All boards either provide a way to retrieve a manufactures Unique ID or
 *   define BOARD_OVERRIDE_UUID.
 *    The MFGUID is returned as an array of bytes in
 *    MSD @ index 0 - LSD @ index PX4_CPU_MFGUID_BYTE_LENGTH-1
 *
 * Input Parameters:
 *   mfgid - mfguid_t and array of bytes PX4_CPU_MFGUID_BYTE_LENGTH in length.
 *
 * Returned Value:
 *   The mfguid_t array is populated with the CPU uuid with the MSD @ index 0
 *   and the LSD @ index PX4_CPU_MFGUID_BYTE_LENGTH-1.
 *
 ************************************************************************************/

int board_get_mfguid(mfguid_t mfgid);

/************************************************************************************
 * Name: board_get_mfguid_formated DEPRICATED use board_get_px4_guid_formated
 *
 * Description:
 *   All boards either provide a way to retrieve a formatted string of the
 *   manufactures unique ID or define BOARD_OVERRIDE_MFGUID
 *
 * Input Parameters:
 *   format_buffer - A pointer to a bufferer of at least PX4_CPU_MFGUID_FORMAT_SIZE
 *                   that will contain a 0 terminated string formated as 0 prefixed
 *                   lowercase hex. 2 charaters per digit of the mfguid_t.
 *
 * Returned Value:
 *   format_buffer is populated with a 0 terminated string of hex digits. The
 *   return value is the number of printable in the string.
 *   Usually PX4_CPU_MFGUID_FORMAT_SIZE-1
 *
 ************************************************************************************/

int board_get_mfguid_formated(char *format_buffer, int size); // DEPRICATED use board_get_px4_guid_formated

/************************************************************************************
 * Name: board_get_px4_guid
 *
 * Description:
 *   All boards either provide a way to retrieve a PX4 Globally unique ID or
 *   define BOARD_OVERRIDE_UUID.
 *
 *   The form of the GUID is as follows:
 *  offset:0         1         2         -           17
 *    <ARCH MSD><ARCH LSD><MSD CPU UUID>...<LSD CPU UUID>
 *
 *  Where <ARCH MSD><ARCH LSD> are a monotonic ordinal number assigned by
 *  PX4 to a chip architecture (PX4_SOC_ARCH_ID). The 2 bytes are used to
 *  create a globally unique ID when prepended to a padded CPU ID.
 *
 *  In the case where the CPU's UUID is shorter than 16 bytes it will be
 *  padded with 0's starting at offset [2] until
 *  PX4_CPU_MFGUID_BYTE_LENGTH-PX4_CPU_UUID_BYTE_LENGTH -1
 *  I.E. For the STM32
 *  offset:0         1     2  3  4  5  6             -            17
 *    <ARCH MSD><ARCH LSD>[0][0][0][0]<MSD CPU UUID>...<LSD CPU UUID>
 *
 *  I.E. For as CPU with a 16 byte UUID
 *  offset:0         1         2         -           17
 *    <ARCH MSD><ARCH LSD><MSD CPU UUID>...<LSD CPU UUID>
 *
 * Input Parameters:
 *   guid - a px4_guid_t which is byte array of PX4_GUID_BYTE_LENGTH length.
 *
 * Returned Value:
 *   guid is populated as  <ARCH MSD><ARCH LSD><MSD CPU UUID>...<LSD CPU UUID>
 *   the return value is PX4_GUID_BYTE_LENGTH
 *
 ************************************************************************************/

int board_get_px4_guid(px4_guid_t guid);

/************************************************************************************
 * Name: board_get_mfguid_formated
 *
 * Description:
 *   All boards either provide a way to retrieve a formatted string of the
 *   manufactures Unique ID or define BOARD_OVERRIDE_PX4_GUID
 *
 * Input Parameters:
 * format_buffer - A buffer to receive the 0 terminated formated px4
 *                 guid string.
 * size          - Size of the buffer provided. Normally this would
 *                 be PX4_GUID_FORMAT_SIZE.
 *                 If the size is less than PX4_GUID_FORMAT_SIZE the string
 *                 returned will be truncated from the MSD end and even
 *                 in length.
 *
 * Returned Value:
 *   The number of printable characters. This value will be even and one less the
 *   the size passed in.
 *
 ************************************************************************************/

int board_get_px4_guid_formated(char *format_buffer, int size);

/************************************************************************************
 * Name: board_mcu_version
 *
 * Description:
 *   All boards either provide a way to retrieve the cpu revision
 *   Or define BOARD_OVERRIDE_CPU_VERSION
 *
 * Input Parameters:
 * rev    - The silicon revision character
 * revstr - The full chip name string
 * errata  -The eratta if any.
 *
 * Returned Value:
 *           The silicon revision / version number as integer
 *           or -1 on error and rev, revstr and errata will
 *           not be set
 */
#if defined(BOARD_OVERRIDE_CPU_VERSION)
#define board_mcu_version(rev, revstr, errata) BOARD_OVERRIDE_CPU_VERSION
#else
__EXPORT int board_mcu_version(char *rev, const char **revstr, const char **errata);
#endif // !defined(BOARD_OVERRIDE_CPU_VERSION)

#if defined(BOARD_HAS_POWER_CONTROL)
/************************************************************************************
 * Name: board_register_power_state_notification_cb
 *
 * Description:
 *   boards may provide a function to register a power button state notification
 *   call back.
 *
 *   N.B. this call back may be called off an interrupt. Do not attempt to block
 *   or run any long threads.
 *
 * Input Parameters:
 *   cb     - A pointer to a power button state notification function.
 *
 * Returned Value:
 *   Zero (OK) is returned on success;
 */

int board_register_power_state_notification_cb(power_button_state_notification_t cb);

/************************************************************************************
 * Name: board_shutdown
 *
 * Description:
 *   boards may provide a function to power off the board.
 *
 * Input Parameters:
 *   None.
 * Returned Value:
 *    - If supported the function will not return.
 *      OK, or -EINVAL if unsupported.
 */
int board_shutdown(void);

#else
static inline int board_register_power_state_notification_cb(power_button_state_notification_t cb) { return 0; }
static inline int board_shutdown(void) { return -EINVAL; }
#endif

/************************************************************************************
 * Name: board_has_bus
 *
 ************************************************************************************/

enum board_bus_types {
	BOARD_INVALID_BUS = 0,
	BOARD_SPI_BUS = 1,
	BOARD_I2C_BUS = 2
};

#if defined(BOARD_HAS_BUS_MANIFEST)

__EXPORT bool board_has_bus(enum board_bus_types type, uint32_t bus);

#else
#  define board_has_bus(t, b) true
#endif /* BOARD_HAS_BUS_MANIFEST */


/************************************************************************************
 * Name: board_spi_reset
 *
 * Description:
 *   Reset SPI buses and devices
 *
 * Input Parameters:
 *  ms                   - delay in msbetween powering off the devices and re-enabling power.
 *
 *  bus_mask             - bitmask to select buses - use 0xffff to select all.
 */
__EXPORT void board_spi_reset(int ms, int bus_mask);

/************************************************************************************
 * Name: board_control_spi_sensors_power_configgpio
 *
 * Description:
 *   Initialize GPIO pins for all SPI bus power enable pins
 */
__EXPORT void board_control_spi_sensors_power_configgpio(void);

/************************************************************************************
 * Name: board_control_spi_sensors_power
 *
 * Description:
 *   Control the power of SPI buses
 *
 * Input Parameters:
 *  enable_power         - true to enable power, false to disable
 *
 *  bus_mask             - bitmask to select buses - use 0xffff to select all.
 */
__EXPORT void board_control_spi_sensors_power(bool enable_power, int bus_mask);

/************************************************************************************
 * Name: board_hardfault_init
 *
 * Description:
 *   boards may provide a to determine if a hard fault occurred
 *   call back.
 *
 * Input Parameters:
 *   display_to_console  - one less then the number of boots with an unsaved hard fault.
 *                         can can occur with displaying the hard fault data to the screen.
 *                         INT_MAX - Never display.
 *                         n-1 - n boots with out a save.
 *
 *  allow_prompt         - if false will not stop on boot, even if a hardfault has happened
 *                         and there are characters waiting on STDIN.
 *
 * Returned Value:
 *   Zero (OK) is returned on success: No hardfaults
 *    >0       - There is a hardfault logged.
 *   -EIO      - there is a Problem with the bbsram
 *   -ENOSPC   - There have been no boots that reset the hard fault count in the last
 *               32000 resets.
 */
int board_hardfault_init(int display_to_console, bool allow_prompt);

__END_DECLS
