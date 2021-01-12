/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * NXP fmuk66-e internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/
#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

/* these headers are not C++ safe */
#include <kinetis.h>
#include <hardware/kinetis_pinmux.h>
#include <arch/board/board.h>

__END_DECLS

/* FMUK66 GPIOs ***********************************************************************************/
/* LEDs */
/* An RGB LED is connected through GPIO as shown below:
 * TBD (no makring on schematic)
 *   LED    K66
 *   ------ -------------------------------------------------------
 *   RED    FB_CS0_b/ UART2_CTS_b / ADC0_SE5b / SPI0_SCK / FTM3_CH1/ PTD1
 *   GREEN  FTM2_FLT0/ CMP0_IN3/ FB_AD6 / I2S0_RX_BCLK/ FTM3_CH5/ ADC1_SE5b/ PTC9
 *   BLUE   CMP0_IN2/ FB_AD7 / I2S0_MCLK/ FTM3_CH4/ ADC1_SE4b/ PTC8
 */

#define GPIO_LED_R             (GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE | PIN_PORTD | PIN1)
#define GPIO_LED_G             (GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE | PIN_PORTC | PIN9)
#define GPIO_LED_B             (GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE | PIN_PORTC | PIN8)


#define GPIO_LED_1             (GPIO_HIGHDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTD | PIN13)
#define GPIO_LED_2             (GPIO_HIGHDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTD | PIN14)

#define BOARD_HAS_CONTROL_STATUS_LEDS 1 // Use D9 and D10
#define BOARD_OVERLOAD_LED     LED_AMBER
#define BOARD_ARMED_STATE_LED  LED_GREEN

#define GPIO_NFC_IO            (GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE  | PIN_PORTA | PIN26)
#define GPIO_SENSOR_P_EN       (GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE  | PIN_PORTB | PIN8)


/* UART tty Mapping
 * Device   tty        alt           Connector Name
 * ------- ---------- -------------- --------- -------------------------
 * LPUART0 /dev/tty0  /dev/console    J16      DCD-Mini
 * UART0   /dev/tty1      ---         J7       SERIAL 2 / TELEMETRY 2 / IRDA
 * UART1   /dev/tty2      ---         J15      SERIAL4/FrSky, RC_IN
 * UART2   /dev/tty3      ---         J3       GPS connector
 * UART4   /dev/tty4      ---         J10      SERIAL 1 / TELEMETRY 1
 */

/* High-resolution timer */
#define HRT_TIMER              1  /* TPM1 timer for the HRT */
#define HRT_TIMER_CHANNEL      0  /* Use capture/compare channel 0 */

/* PPM IN
 */

#define HRT_PPM_CHANNEL        1  /* Use TPM1 capture/compare channel 1 */
#define GPIO_PPM_IN            PIN_TPM1_CH1_1    /* PTC3 USART1 RX and PTA9 and PIN_TPM1_CH1 AKA FrSky_IN_RC_IN */


/*
 *
 * NXP fmuk66-e has separate RC_IN
 *
 * GPIO PPM_IN on PTA9 PIN_TPM1_CH1 and PCT3 USART1 RX
 * SPEKTRUM_RX (it's TX or RX in Bind) on PCT3 USART1 RX
 * Inversion is possible the UART
 * The FMU can drive GPIO PPM_IN as an output
 */

/* Spektrum controls ******************************************************/

/* Power is a p-Channel FET */

#define GPIO_SPEKTRUM_P_EN          (GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE | PIN_PORTA | PIN7)

/* For binding the Spektrum 3-pin interfaces is used with it TX (output)
 * as an input Therefore we drive are UARTx_RX (normaly an input) as an
 * output
 */

#define GPIO_PPM_IN_AS_OUT          (GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE | PIN_PORTC | PIN3)

#define SPEKTRUM_RX_AS_GPIO_OUTPUT() px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
#define SPEKTRUM_RX_AS_UART()        px4_arch_configgpio(PIN_UART1_RX)
#define SPEKTRUM_OUT(_one_true)      px4_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))

/* RC input */

#define RC_SERIAL_PORT          "/dev/ttyS2"      /* UART1 */
#define GPIO_RSSI_IN            PIN_ADC1_SE13

/* Ethernet Control
 *
 * Uninitialized to Reset Disabled and Inhibited
 * All pins driven low to not back feed when power is off
 */

#define nGPIO_ETHERNET_P_EN     (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE  | PIN_PORTE | PIN10)
#define GPIO_ENET_RST           (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTA | PIN28)
#define GPIO_ENET_EN            (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTA | PIN29)
#define GPIO_ENET_INH           (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTA | PIN8)

/* CAN Control
 * Control pin S allows two operating modes to be selected:
 * high-speed mode (Low) or silent mode (high)
 */

#define GPIO_CAN0_STB           (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO  | PIN_PORTC | PIN19)
#define GPIO_CAN1_STB           (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO  | PIN_PORTC | PIN18)


/* Safety Switch
 * TBD
 */
#define GPIO_LED_SAFETY         (GPIO_HIGHDRIVE | GPIO_OUTPUT_ZERO  | PIN_PORTC | PIN0)
#define GPIO_BTN_SAFETY         (GPIO_PULLUP | PIN_PORTE | PIN28)

/* NXP FMUK66-E GPIOs ****************************************************************/

/*	SDHC
 *
 * A micro Secure Digital (SD) card slot is available on the board connected to
 * the SD Host Controller (SDHC) signals of the MCU. This slot will accept micro
 * format SD memory cards. The SD card detect pin (PTE6) is an open switch that
 * shorts with VDD when card is inserted.
 *
 *   ------------ ------------- --------
 *    SD Card Slot Board Signal  K66 Pin
 *    ------------ ------------- --------
 *    DAT0         SDHC0_D0      PTE1
 *    DAT1         SDHC0_D1      PTE0
 *    DAT2         SDHC0_D2      PTE5
 *    CD/DAT3      SDHC0_D3      PTE4
 *    CMD          SDHC0_CMD     PTE3
 *    CLK          SDHC0_DCLK    PTE2
 *    SWITCH       D_CARD_DETECT PTD10
 *                 CAED_P_EN     PTD6
 *    ------------ ------------- --------
 *
 * There is no Write Protect pin available to the K66
 */
#define nVDD_3V3_SD_CARD_EN                 (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE | PIN_PORTA | PIN25)


/* SE050 Secure Element  */
#define SE050_ENA                           (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTA | PIN24)
#define SE050_RST_N                         (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTD | PIN11)

//#define GPIO_SD_CARDDETECT (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTD | PIN10)

/* SPI device reset signals
 * In Active state
 */

/* Sensor interrupts */

#define GPIO_nSPI1_DRDY1_BMI1088_ACCEL_INT1 (GPIO_PULLUP | PIN_INT_FALLING | PIN_PORTD | PIN12)
#define GPIO_nSPI1_DRDY2_BMI1088_GYRO_INT2  (GPIO_PULLUP | PIN_INT_FALLING | PIN_PORTE | PIN7)
#define GPIO_nSPI1_DRDY3_ICM42688_INT1      (GPIO_PULLUP | PIN_INT_FALLING | PIN_PORTE | PIN9)

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4
 * Firmware in the adc driver. ADC1 has 32 channels, with some a/b selection overlap
 * in the AD4-AD7 range on the same ADC.
 *
 * Only ADC1 is used
 *         Bits 31:0 are ADC1 channels 31:0
 */

#define ADC1_CH(c)    (((c) & 0x1f))	/* Define ADC number Channel number */
#define ADC1_GPIO(n)  PIN_ADC1_SE##n

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC_USB_VBUS_VALID          ADC1_CH(0)      /* USB_VBUS_VALID   29    -    ADC1_DP0  */
#define ADC_BATTERY_VOLTAGE_CHANNEL ADC1_CH(10)     /* BAT_VSENS        85   PTB4  ADC1_SE10 */
#define ADC_BATTERY_CURRENT_CHANNEL ADC1_CH(11)     /* BAT_ISENS        86   PTB5  ADC1_SE11 */
#define ADC_5V_RAIL_SENSE           ADC1_CH(12)     /* 5V_VSENS         87   PTB6  ADC1_SE12 */
#define ADC_RSSI_IN                 ADC1_CH(13)     /* RSSI_IN          88   PTB7  ADC1_SE13 */
#define ADC_AD1                     ADC1_CH(16)     /* AD1              35    -    ADC1_SE16 */
#define ADC_AD2                     ADC1_CH(18)     /* AD2              37    -    ADC1_SE18 */
#define ADC_AD3                     ADC1_CH(23)     /* AD3              39    -    ADC1_SE23 */

/* Mask use to initialize the ADC driver */

#define ADC_CHANNELS ((1 << ADC_USB_VBUS_VALID) | \
		      (1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
		      (1 << ADC_BATTERY_CURRENT_CHANNEL) | \
		      (1 << ADC_5V_RAIL_SENSE) | \
		      (1 << ADC_RSSI_IN) | \
		      (1 << ADC_AD1) | \
		      (1 << ADC_AD2) | \
		      (1 << ADC_AD3))


/* GPIO that require Configuration */

#define PX4_ADC_GPIO  \
	/* PTB4  ADC1_SE10 */  ADC1_GPIO(10),  \
	/* PTB5  ADC1_SE11 */  ADC1_GPIO(11),  \
	/* PTB6  ADC1_SE12 */  ADC1_GPIO(12),  \
	/* PTB7  ADC1_SE13 */  ADC1_GPIO(13)



#define BOARD_BATTERY1_V_DIV   (10.177939394f)
#define BOARD_BATTERY1_A_PER_V (15.391030303f)


/* User GPIOs
 *
 */

/* Timer I/O PWM and capture
 *
 * 14 PWM outputs are configured.
 * 14 Timer inputs are configured.
 *
 * Pins:
 *      Defined in board.h
 */
// todo:Design this!

#define DIRECT_PWM_OUTPUT_CHANNELS  8
#define DIRECT_INPUT_TIMER_CHANNELS 8

#define GPIO_ULTRASOUND_TRIGGER  /* PTD0 */  (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTD | PIN0)
#define GPIO_ULTRASOUND_ECHO     /* PTA10 */ (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTA | PIN10)

/* Power supply control and monitoring GPIOs */
// None

#define GPIO_PERIPH_3V3_EN  0


/* Tone alarm output PTA11 - TMP 2_CH1 is On +P12-4, -P12-5
 * It is driving a NPN
 */
#define TONE_ALARM_TIMER     2   /* timer */
#define TONE_ALARM_CHANNEL   1   /* channel  */
#define GPIO_TONE_ALARM_IDLE (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTA | PIN11)
#define GPIO_TONE_ALARM      (PIN_TPM2_CH1_1)

/* USB
 *
 *  VBUS detection is on 29  ADC_DPM0 and PTE8
 */
#define GPIO_USB_VBUS_VALID         /* PTE8 */ (GPIO_PULLUP | PIN_PORTE | PIN8)

/* PWM input driver. Use FMU PWM14 pin
 * todo:desing this
 */
#define PWMIN_TIMER		0
#define PWMIN_TIMER_CHANNEL	2
#define GPIO_PWM_IN		GPIO_FTM0_CH2IN

/* Define True logic Power Control in arch agnostic form */

#define VDD_ETH_EN(on_true)                px4_arch_gpiowrite(nGPIO_ETHERNET_P_EN, !(on_true))
// Do not have #define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_nVDD_5V_PERIPH_EN, !(on_true))
// Do not have #define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_nVDD_5V_HIPOWER_EN, !(on_true))
#define VDD_3V3_SENSORS_EN(on_true)        px4_arch_gpiowrite(GPIO_SENSOR_P_EN, (on_true))
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_SPEKTRUM_P_EN, !(on_true))
#define READ_VDD_3V3_SPEKTRUM_POWER_EN()   px4_arch_gpioread(GPIO_SPEKTRUM_P_EN)
// Do not have #define VDD_5V_RC_EN(on_true)              px4_arch_gpiowrite(GPIO_VDD_5V_RC_EN, (on_true))
// Do not have #define VDD_5V_WIFI_EN(on_true)            px4_arch_gpiowrite(GPIO_VDD_5V_WIFI_EN, (on_true))
#define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(nVDD_3V3_SD_CARD_EN, !(on_true))

/* Map to control term used in RC lib */
#define SPEKTRUM_POWER(on_true) VDD_3V3_SPEKTRUM_POWER_EN((on_true))


/*
 * By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */

#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_USB_VBUS_VALID))
#define BOARD_ADC_BRICK_VALID   (1)
#define BOARD_ADC_SERVO_VALID   (1)
#define BOARD_ADC_PERIPH_5V_OC  (0)
#define BOARD_ADC_HIPOWER_5V_OC (0)

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_HAS_LED_PWM              1

#define LED_TIM3_CH1OUT   /* PTD1  RGB_R */ PIN_FTM3_CH1_1
#define LED_TIM3_CH5OUT   /* PTC9  RGB_G */ PIN_FTM3_CH5_1
#define LED_TIM3_CH4OUT   /* PTC8  RGB_B */ PIN_FTM3_CH4_1

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST {  \
		GPIO_LED_R,           \
		GPIO_LED_G,           \
		GPIO_LED_B,           \
		GPIO_SENSOR_P_EN,     \
		nVDD_3V3_SD_CARD_EN,  \
		nGPIO_ETHERNET_P_EN,  \
		GPIO_SPEKTRUM_P_EN,   \
		SE050_ENA,            \
		SE050_RST_N,          \
		PX4_ADC_GPIO,         \
		GPIO_USB_VBUS_VALID,  \
		GPIO_ENET_RST,        \
		GPIO_ENET_EN,         \
		GPIO_ENET_INH,        \
		PIN_CAN0_RX,          \
		PIN_CAN0_TX,          \
		PIN_CAN1_RX,          \
		PIN_CAN1_TX,          \
		GPIO_CAN0_STB,        \
		GPIO_CAN1_STB,        \
		GPIO_BTN_SAFETY,      \
		GPIO_TONE_ALARM_IDLE, \
		GPIO_NFC_IO,          \
		GPIO_LED_1,           \
		GPIO_LED_2            \
	}

/* Automounter */

#define HAVE_MMCSD      1
#define HAVE_AUTOMOUNTER 1
#if !defined(CONFIG_FS_AUTOMOUNTER) || !defined(HAVE_MMCSD)
#  undef HAVE_AUTOMOUNTER
#  undef CONFIG_FMUK66_SDHC_AUTOMOUNT
#endif

#ifndef CONFIG_FMUK66_SDHC_AUTOMOUNT
#  undef HAVE_AUTOMOUNTER
#endif

/* Automounter defaults */

#ifdef HAVE_AUTOMOUNTER

#  ifndef CONFIG_FMUK66_SDHC_AUTOMOUNT_FSTYPE
#    define CONFIG_FMUK66_SDHC_AUTOMOUNT_FSTYPE "vfat"
#  endif

#  ifndef CONFIG_FMUK66_SDHC_AUTOMOUNT_BLKDEV
#    define CONFIG_FMUK66_SDHC_AUTOMOUNT_BLKDEV "/dev/mmcds0"
#  endif

#  ifndef CONFIG_FMUK66_SDHC_AUTOMOUNT_MOUNTPOINT
#    define CONFIG_FMUK66_SDHC_AUTOMOUNT_MOUNTPOINT "/mnt/sdcard"
#  endif

#  ifndef CONFIG_FMUK66_SDHC_AUTOMOUNT_DDELAY
#    define CONFIG_FMUK66_SDHC_AUTOMOUNT_DDELAY 1000
#  endif

#  ifndef CONFIG_FMUK66_SDHC_AUTOMOUNT_UDELAY
#    define CONFIG_FMUK66_SDHC_AUTOMOUNT_UDELAY 2000
#  endif
#endif /* HAVE_AUTOMOUNTER */

#define BOARD_HAS_NOISY_FXOS8700_MAG 1 // Disable internal MAG

#define BOARD_NUM_IO_TIMERS 3

/************************************************************************************
 * Public data
 ************************************************************************************/

__BEGIN_DECLS

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: fmuk66_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP FMUK66-E board.
 *
 ************************************************************************************/

void fmuk66_spidev_initialize(void);

/************************************************************************************
 * Name: fmuk66_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI Buses.
 *
 ************************************************************************************/

int  fmuk66_spi_bus_initialize(void);

/****************************************************************************************************
 * Name: board_spi_reset board_peripheral_reset
 *
 * Description:
 *   Called to reset SPI and the perferal bus
 *
 ****************************************************************************************************/
void board_peripheral_reset(int ms);

/************************************************************************************
 * Name: fmuk66_bringup
 *
 * Description:
 *   Bring up board features
 *
 ************************************************************************************/

#if defined(CONFIG_LIB_BOARDCTL) || defined(CONFIG_BOARD_INITIALIZE)
int fmuk66_bringup(void);
#endif

/****************************************************************************
 * Name: fmuk66_sdhc_initialize
 *
 * Description:
 *   Inititialize the SDHC SD card slot
 *
 ****************************************************************************/

int fmuk66_sdhc_initialize(void);

/************************************************************************************
 * Name: fmuk66_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the SDHC slot
 *
 ************************************************************************************/

#ifdef HAVE_AUTOMOUNTER
bool fmuk66_cardinserted(void);
#else
#  define fmuk66_cardinserted() (false)
#endif

/************************************************************************************
 * Name: fmuk66_writeprotected
 *
 * Description:
 *   Check if the card in the MMC/SD slot is write protected
 *
 ************************************************************************************/

#ifdef HAVE_AUTOMOUNTER
bool fmuk66_writeprotected(void);
#else
#  define fmuk66_writeprotected() (false)
#endif

/************************************************************************************
 * Name:  fmuk66_automount_initialize
 *
 * Description:
 *   Configure auto-mounter for the configured SDHC slot
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ************************************************************************************/

#ifdef HAVE_AUTOMOUNTER
void fmuk66_automount_initialize(void);
#endif

/************************************************************************************
 * Name:  fmuk66_automount_event
 *
 * Description:
 *   The SDHC card detection logic has detected an insertion or removal event.  It
 *   has already scheduled the MMC/SD block driver operations.  Now we need to
 *   schedule the auto-mount event which will occur with a substantial delay to make
 *   sure that everything has settle down.
 *
 * Input Parameters:
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ************************************************************************************/

#ifdef HAVE_AUTOMOUNTER
void fmuk66_automount_event(bool inserted);
#endif

/************************************************************************************
 * Name: fmuk66_timer_initialize
 *
 * Description:
 *   Called to configure the FTM to provide 1 Mhz
 *
 ************************************************************************************/

void fmuk66_timer_initialize(void);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
