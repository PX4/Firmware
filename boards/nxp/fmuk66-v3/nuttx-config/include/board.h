/************************************************************************************
 * configs/nxp/fmuk66-v3/include/board.h
 *
 *   Copyright (C) 2016-2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *   		  Jordan MacIntyre
 *   		  David Sidrane <david_s5@nscdg.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ************************************************************************************/

#ifndef __CONFIG_NXP_FMUK66_V3_INCLUDE_BOARD_H
#define __CONFIG_NXP_FMUK66_V3_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#include <arch/chip/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/*
 *  The FMUK66-v3 is populated with a MK66FN2M0VLQ18 has 2 MiB of FLASH and
 *  256 KiB of SRAM.
 */
/* Clocking *************************************************************************/
/* The NXP FMUK66-V3 uses a 16MHz external powered Oscillator.  The Kinetis MCU startup
 * from an internal digitally-controlled oscillator (DCO). Nuttx will enable the main
 * external oscillator EXTAL0. The external oscillator can range from
 * 32.768 KHz up to 50 MHz. The default external source for the MCG oscillator inputs
 * EXTAL.
 *
 * Y1 a High-frequency, Oscillator
 */
#define BOARD_EXTAL_LP		 1
#define BOARD_EXTAL_FREQ     16000000       /* 16MHz Oscillator Y1 */
#define BOARD_XTAL32_FREQ    32768          /* 32KHz RTC Oscillator */

#define BOARD_OSC_CR         OSC_CR_ERCLKEN /* Enable the OSCERCLK */
#define BOARD_OSC_DIV        OSC_DIV_ERPS_DIV1 /* No OSCERCLK Divide */

/* FLL Configuration.
 *  BOARD_EXTAL_FREQ / BOARD_FRDIV has to be in the range 31.25 kHz to 39.0625
 *  16 Mhz / MCG_C1_FRDIV_DIV512 = 31.25 kHz * 640 the default for MCG_C4
 *  FLL is 20Mhz
 */
#define BOARD_FRDIV          MCG_C1_FRDIV_DIV512

/* PLL Configuration.  Either the external clock or crystal frequency is used to
 * select the PRDIV value. Only reference clock frequencies are supported that will
 * produce a KINETIS_MCG_PLL_REF_MIN=8MHz >= PLLIN <=KINETIS_MCG_PLL_REF_MAX=16Mhz
 * reference clock to the PLL.
 *
 *   PLL Input frequency:   PLLIN  = REFCLK / PRDIV = 16 MHz  / 2 = 8Mhz MHz
 *   PLL Output frequency:  PLLOUT = PLLIN  * VDIV  = 8 MHz  * 42 = 336 MHz
 *   MCG Frequency:         PLLOUT = 168 Mhz = 336 MHz / KINETIS_MCG_PLL_INTERNAL_DIVBY=2
 *
 * PRDIV register value is the divider minus KINETIS_MCG_C5_PRDIV_BASE.
 * VDIV  register value is offset by KINETIS_MCG_C6_VDIV_BASE.
 */

#define BOARD_PRDIV          2              /* PLL External Reference Divider */
#define BOARD_VDIV           42             /* PLL VCO Divider (frequency multiplier) */

/* Define additional MCG_C2 Setting */

#define BOARD_MCG_C2_FCFTRIM 0              /* Do not enable FCFTRIM */
#define BOARD_MCG_C2_LOCRE0  MCG_C2_LOCRE0  /* Enable reset on loss of clock */

#define BOARD_PLLIN_FREQ     (BOARD_EXTAL_FREQ / BOARD_PRDIV)
#define BOARD_PLLOUT_FREQ    (BOARD_PLLIN_FREQ * BOARD_VDIV)
#define BOARD_MCG_FREQ       (BOARD_PLLOUT_FREQ/KINETIS_MCG_PLL_INTERNAL_DIVBY)

/* SIM CLKDIV1 dividers */

#define BOARD_OUTDIV1        1              /* Core        = MCG,    168 MHz */
#define BOARD_OUTDIV2        3              /* Bus         = MCG / 3, 56 MHz */
#define BOARD_OUTDIV3        3              /* FlexBus     = MCG / 3, 56 MHz */
#define BOARD_OUTDIV4        6              /* Flash clock = MCG / 6, 28 MHz */

#define BOARD_CORECLK_FREQ  (BOARD_MCG_FREQ / BOARD_OUTDIV1)
#define BOARD_BUS_FREQ      (BOARD_MCG_FREQ / BOARD_OUTDIV2)
#define BOARD_FLEXBUS_FREQ  (BOARD_MCG_FREQ / BOARD_OUTDIV3)
#define BOARD_FLASHCLK_FREQ (BOARD_MCG_FREQ / BOARD_OUTDIV4)

/* Use BOARD_MCG_FREQ as the output SIM_SOPT2 MUX selected by
 * SIM_SOPT2[PLLFLLSEL]
 */

#define BOARD_SOPT2_PLLFLLSEL   SIM_SOPT2_PLLFLLSEL_MCGPLLCLK
#define BOARD_SOPT2_FREQ        BOARD_MCG_FREQ

/* Divider output clock = Divider input clock × [ (USBFRAC+1) / (USBDIV+1) ]
 *     SIM_CLKDIV2_FREQ = BOARD_SOPT2_FREQ × [ (USBFRAC+1) / (USBDIV+1) ]
 *                48Mhz = 168Mhz X [(1 + 1) / (6 + 1)]
 *                48Mhz = 168Mhz / (6 + 1) * (1 + 1)
 */

#define BOARD_SIM_CLKDIV2_USBFRAC     2
#define BOARD_SIM_CLKDIV2_USBDIV      7
#define BOARD_SIM_CLKDIV2_FREQ        (BOARD_SOPT2_FREQ / BOARD_SIM_CLKDIV2_USBDIV * BOARD_SIM_CLKDIV2_USBFRAC)
#define BOARD_USB_CLKSRC               SIM_SOPT2_USBSRC
#define BOARD_USB_FREQ                 BOARD_SIM_CLKDIV2_FREQ


/* Divider output clock = Divider input clock * ((PLLFLLFRAC+1)/(PLLFLLDIV+1))
 *  SIM_CLKDIV3_FREQ = BOARD_SOPT2_FREQ × [ (PLLFLLFRAC+1) / (PLLFLLDIV+1)]
 *            84 Mhz = 168 Mhz X [(0 + 1) / (1 + 1)]
 *            84 Mhz = 168 Mhz / (1 + 1) * (0 + 1)
 */

#define BOARD_SIM_CLKDIV3_PLLFLLFRAC  1
#define BOARD_SIM_CLKDIV3_PLLFLLDIV   2
#define BOARD_SIM_CLKDIV3_FREQ        (BOARD_SOPT2_FREQ / BOARD_SIM_CLKDIV3_PLLFLLDIV * BOARD_SIM_CLKDIV3_PLLFLLFRAC)

#define BOARD_LPUART0_CLKSRC           SIM_SOPT2_LPUARTSRC_MCGCLK
#define BOARD_LPUART0_FREQ             BOARD_SIM_CLKDIV3_FREQ

#define BOARD_TPM_CLKSRC               SIM_SOPT2_TPMSRC_OCSERCLK
#define BOARD_TPM_FREQ                 BOARD_EXTAL_FREQ

/* SDHC clocking ********************************************************************/

/* SDCLK configurations corresponding to various modes of operation.   Formula is:
 *
 *   SDCLK  frequency = (base clock) / (prescaler * divisor)
 *
 * The SDHC module is always configure configured so that the core clock is the base
 * clock.  Possible values for prescaler and divisor are:
 *
 *   SDCLKFS: {2, 4, 8, 16, 32, 63, 128, 256}
 *   DVS:     {1..16}
 */

/* SDHC pull-up resistors **********************************************************/

/* There are external pull-ups on the NXP fmuk66-v3
 * So enable we do not them.
 */

#undef BOARD_SDHC_ENABLE_PULLUPS

/* Identification mode:  Optimal 400KHz, Actual 168Mhz / (32 * 14) = 375 KHz */

#define BOARD_SDHC_IDMODE_PRESCALER    SDHC_SYSCTL_SDCLKFS_DIV32
#define BOARD_SDHC_IDMODE_DIVISOR      SDHC_SYSCTL_DVS_DIV(14)

/* MMC normal mode: Optimal 20MHz, Actual 168Mhz / (2 * 5) = 16.8 MHz */

#define BOARD_SDHC_MMCMODE_PRESCALER   SDHC_SYSCTL_SDCLKFS_DIV2
#define BOARD_SDHC_MMCMODE_DIVISOR     SDHC_SYSCTL_DVS_DIV(5)

/* SD normal mode (1-bit): Optimal 20MHz, Actual 168Mhz / (2 * 5) = 16.8 MHz */

#define BOARD_SDHC_SD1MODE_PRESCALER   SDHC_SYSCTL_SDCLKFS_DIV2
#define BOARD_SDHC_SD1MODE_DIVISOR     SDHC_SYSCTL_DVS_DIV(5)

/* SD normal mode (4-bit): Optimal 25MHz, Actual Actual 168Mhz / (2 * 4) = 21 MHz (with DMA)
 * SD normal mode (4-bit): Optimal 20MHz, Actual 168Mhz / (2 * 5) = 16.8 MHz (no DMA)
 */

#ifdef CONFIG_KINETIS_SDHC_DMA
#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV2
#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(4)
#else
#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV2
#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(5)
#endif


/* LED definitions ******************************************************************/
/* The NXP FMUK66-V3 has a separate Red, Green and Blue LEDs driven by the K66 as
 * follows:
 *
 *   LED    K66
 *   ------ -------------------------------------------------------
 *   RED    FB_CS0_b/ UART2_CTS_b / ADC0_SE5b / SPI0_SCK / FTM3_CH1/ PTD1
 *   GREEN  FTM2_FLT0/ CMP0_IN3/ FB_AD6 / I2S0_RX_BCLK/ FTM3_CH5/ ADC1_SE5b/ PTC9
 *   BLUE   CMP0_IN2/ FB_AD7 / I2S0_MCLK/ FTM3_CH4/ ADC1_SE4b/ PTC8
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_R       0
#define BOARD_LED_G       1
#define BOARD_LED_B       2
#define BOARD_NLEDS       3

/* LED bits for use with board_userled_all() */

#define BOARD_LED_R_BIT   (1 << BOARD_LED_R)
#define BOARD_LED_G_BIT   (1 << BOARD_LED_G)
#define BOARD_LED_B_BIT   (1 << BOARD_LED_B)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the NXP FMUK66-V3.  The following definitions describe how NuttX controls
 * the LEDs:
 *
 *   SYMBOL                Meaning                      LED state
 *                                                      RED   GREEN  BLUE
 *   -------------------  ----------------------------  ----------------- */
#define LED_STARTED       1 /* NuttX has been started    OFF   OFF    OFF */
#define LED_HEAPALLOCATE  2 /* Heap has been allocated   OFF   OFF    ON  */
#define LED_IRQSENABLED   0 /* Interrupts enabled        OFF   OFF    ON  */
#define LED_STACKCREATED  3 /* Idle stack created        OFF   ON     OFF */
#define LED_INIRQ         0 /* In an interrupt          (no change)       */
#define LED_SIGNAL        0 /* In a signal handler      (no change)       */
#define LED_ASSERTION     0 /* An assertion failed      (no change)       */
#define LED_PANIC         4 /* The system has crashed    FLASH OFF    OFF */
#undef  LED_IDLE            /* K66 is in sleep mode     (Not used)        */

/* Alternative pin resolution *******************************************************/
/* If there are alternative configurations for various pins in the
 * kinetis_k66pinmux.h header file, those alternative pins will be labeled with a
 * suffix like _1, _2, etc.  The logic in this file must select the correct pin
 * configuration for the board by defining a pin configuration (with no suffix) that
 * maps to the correct alternative.
 */

/* CAN
 * Signal Conn      Port Pin   Name
 * ------- -------- ----- ----- --------
 * CAN0TX  P8-2(H)  PTB18  97   CAN0_TX
 * CAN0RX  P8-3(L)  PTB19  98   CAN0_RX
 * CAN1TX  P19-2(H) PTC16  123  CAN1_TX
 * CAN1RX  P19-3(L) PTC17  124  CAN1_RX
 *
 */
#define PIN_CAN0_RX       PIN_CAN0_RX_2
#define PIN_CAN0_TX       PIN_CAN0_TX_2
#define PIN_CAN1_RX       PIN_CAN1_RX_2
#define PIN_CAN1_TX       PIN_CAN1_TX_2

/* 12C
 *
 */

/* I2C0
 *
 * This device can be pinned out to be either or

 * Bit   Pin Device   Signal Usage                 Conn
 * ----- --- -------  --------------------------- ------
 * PTB2   83 I2C0_SCL U_ECH Ultrasonic            P13-3
 * PTB3   84 I2C0_SDA U_TRI Ultrasonic            P13-2
 * ----- --- -------  --------------------------- ------
 *
 * Bit   Pin Device   Signal Usage                 Conn
 * ----- --- -------  --------------------------- ------
 * PTE24  45 I2C0_SCL IIC_SCL NFC Connector, IIC  P2-2
 * PTE25  46 I2C0_SDA IIC_SDA NFC Connector, IIC  P2-3
 * ----- --- -------  --------------------------- ------
 */

#define PIN_I2C0_SCL     PIN_I2C0_SCL_4   /* PTE24  IIC_SCL */
#define PIN_I2C0_SDA     PIN_I2C0_SDA_4   /* PTE25  IIC_SDA */

/* I2C1
 *
 * Bit   Pin Device   Signal         Usage         Conn
 * ----- --- -------  -------------- ------------- ------
 * PTC10 115 I2C1_SCL P_SCL, GPS_SCL Pressure, GPS P3-4
 * PTC11 116 I2C1_SDA P_SDA, GPS_SDA Pressure, GPS P3-5
 * ----- --- -------  -------------- ------------- ------
 */

#define PIN_I2C1_SCL     PIN_I2C1_SCL_1   /* PTC10 GPS / Pressure Sensor*/
#define PIN_I2C1_SDA     PIN_I2C1_SDA_1   /* PTC11 GPS / Pressure Sensor */


/* SPI
 *
 */

/* SPI0 FRAM */

#define PIN_SPI0_PCS0    PIN_SPI0_PCS2_1  /* PTC2 SPI_CS  FRAM_CS   */
#define PIN_SPI0_SCK     PIN_SPI0_SCK_2   /* PTC5 SPI_CLK FRAM_SCK  */
#define PIN_SPI0_OUT     PIN_SPI0_SOUT_2  /* PTC6 SPI_OUT FRAM_MOSI */
#define PIN_SPI0_SIN     PIN_SPI0_SIN_2   /* PTC7 SPI_IN  FRAM_MISO */

/* SPI1
 * FXOS8700CQ Accelerometer
 * FXAS21002CQ Gyroscope
 */

#define PIN_SPI1_PCS0    PIN_SPI1_PCS0_1  /* PTB10 A_CS   */
#define PIN_SPI1_PCS1    PIN_SPI1_PCS1_1  /* PTB9  GM_CS  */
#define PIN_SPI1_SCK     PIN_SPI1_SCK_1   /* PTB11 A_SCLK */
#define PIN_SPI1_OUT     PIN_SPI1_SOUT_1  /* PTB16 A_MOSI */
#define PIN_SPI1_SIN     PIN_SPI1_SIN_1   /* PTB17 A_MISO */

/* SPI2
 * Bit   Pin Device   Signal     Conn
 * ----- --- -------  --------- ------
 * PTB20 99  SPI2_PCS0 SPI2_CS  P18-5
 * PTB21 100 SPI2_SCK  SPI2_CLK P18-2
 * PTB22 101 SPI2_SOUT SPI2_OUT P18-3
 * PTB23 102 SPI2_SIN SPI2_IN   P18-4
 *
 */

#define PIN_SPI2_PCS0    PIN_SPI2_PCS0_1  /* PTB20 SPI2_CS  */
#define PIN_SPI2_SCK     PIN_SPI2_SCK_1   /* PTB21 SPI2_CLK */
#define PIN_SPI2_OUT     PIN_SPI2_SOUT_1  /* PTB22 SPI2_OUT */
#define PIN_SPI2_SIN     PIN_SPI2_SIN_1   /* PTB23 SPI2_IN  */

/* UART
 *
 * NuttX Will use LPUART0 as the Console
 *
 * LPUAR0
 *  P16 Pin     Name        K66   Name
 *  -------- ------------ ------ ---------
 *      2    UART_TX       PTD9 LPUART0_TX
 *      3    UART_RX       PTD8 LPUART0_RX
 *  -------- ----- ------ ---------
 */

#define PIN_LPUART0_RX      (PIN_LPUART0_RX_3 | GPIO_PULLUP)
#define PIN_LPUART0_TX      PIN_LPUART0_TX_3

/* UART0
 *
 *  P7   Pin     Name        K66 Name
 *  -------- ------------ ------- ---------
 *      2    IR_Transmitter PTA2  UART0_TX
 *      4    IR_Receiver    PTA1  UART0_RX
 *  -------- ------------ ------- ---------
 */

#define PIN_UART0_RX      PIN_UART0_RX_1
#define PIN_UART0_TX      PIN_UART0_TX_1

/* UART1
 *
 *     Pin        Name           K66   Name
 *  ------------- -------------- ----- ---------
 *  P14-3,P15-2   FrSky_IN_RC_IN PTC3  UART1_RX
 *  P14-2         FrSky_OUT      PTC4  UART1_TX
 *  ------------- ------------ ----- ---------
 */

#define PIN_UART1_RX      PIN_UART1_RX_1
#define PIN_UART1_TX      PIN_UART1_TX_1

/* UART2
 * No Alternative pins for UART2
 *
 *  P7   Pin     Name        K66 Name
 *  -------- ------------ ------- ---------
 *      2    GPS_TX       PTD3  UART2_TX
 *      3    GPS_RX       PTD2  UART2_RX
 *  -------- ------------ ------- ---------
 */

/* UART4
 *
 *  P10  Pin   Name        K66 Name
 *  -------- ------------ ------- ---------
 *      2    UART4_TX     PTC15  UART4_TX
 *      3    UART4_RX     PTC14  UART4_RX
 *      4    UART4_CTS    PTC13  UART4_CTS
 *      5    UART4_RTS    PTE27  UART4_RTS
 *  -------- ------------ ------- ---------
 */

#define PIN_UART4_RX      PIN_UART4_RX_1
#define PIN_UART4_TX      PIN_UART4_TX_1
#define PIN_UART4_RTS     PIN_UART4_RTS_2
#define PIN_UART4_CTS     PIN_UART4_CTS_1

/*
 * Ethernet TJA1100 OPEN Alliance BroadR-Reach PHY for Automotive Ethernet
 * -----------------------------------------------------------------------
 *
 *  -------------- ----------------- -----------------------------------------
 * TJA1100        Board Signal(s)   K66F Pin
 * Pin Signal                       Function               pinmux Name
 * --- ---------- ----------------- ------------------------------------------
 * 1   MDC        RMII0_MDC         PTB1/RMII0_MDC         PIN_RMII0_MDC
 * 2   INT        RMII0_INT_B,      PTA27                  PTA27
 * 3   nRST       ENET_RST          PTA28                  PTA28
 * 4   VDDA(1V8)  Cap to GND         ---                    ---
 * 5   XO         25Mhz OSC          ---                    ---
 * 6   XI         25Mhz OSC          ---                    ---
 * 7   VDDA(3V3)  E_3V3              ---                    ---
 * 8   LED (LED_ENABLE = 1)  WAKE (LED_ENABLE = 0)          ---
 * 8   VBAT       E_3V3              ---                    ---
 * 10  nINH       ENET_INH          PTA8                   PTA8
 * 11  VDDA(TX)   E_3V3              ---                    ---
 * 12  TRX_P      ENET_P             ---                    ---
 * 13  TRX_M      ENET_P             ---                    ---
 * 14  VDDD(3V3)  E_3V3              ---                    ---
 * 15  GND         ---               ---                    ---
 * 16  VDDD(1V8)  Cap to GND         ---                    ---
 * 17  RXER       RMII0_RXER        PTA5/RMII0_RXER        PIN_RMII0_RXER
 * 18  CRS_DIV    RMII_CRSDV        PTA14/RMII0_CRS_DV     PIN_RMII0_CRS_DV
 * 19  TXEN       RMII0_TXEN        PTA15/RMII0_TXEN       PIN_RMII0_TXEN
 * 20  GND         ---               ---                    ---
 * 21  CONFIG1    ENET_CON1    THIS PIN IS A NO CONNECT
 * 22  CONFIG0    ENET_CONFIG0      PTA24                  PTA24
 * 23  RXD1       RMII0_RXD_1       PTA12/RMII0_RXD1       PIN_RMII0_RXD1
 * 24  RXD0       RMII0_RXD_0       PTA13/RMII0_RXD0       PIN_RMII0_RXD0
 * 25  REF_CLK    E_REF_CLK         PTE26/ENET_1588_CLKIN  PIN_ENET_1588_CLKIN
 * 26  GND2        ---               ---                    ---
 * 27  VDD(IO)    E_3V3              ---                    ---
 * 28  TXC         ---               ---                    ---
 * 29  TXEN       RMII_TXEN         PTA15/RMII0_TXEN       PIN_RMII0_TXEN
 * 30  TXD3
 * 31  TXD2
 * 32  TXD1       RMII0_TXD_1       PTA17/RMII0_TXD1       PIN_RMII0_TXD1
 * 33  TXD0       RMII0_TXD_0       PTA16/RMII0_TXD0       PIN_RMII0_TXD0
 * 34  TXER
 * 35  EN         ENET_EN           PTA29                  PTA29
 * 36  MDIO       RMII0_MDIO        PTB0/RMII0_MDIO        PIN_RMII0_MDIO
 * --- ---------- ----------------- ------------------------------------
 *
 */

#define PIN_RMII0_MDIO    PIN_RMII0_MDIO_1
#define PIN_RMII0_MDC     PIN_RMII0_MDC_1
#define PIN_ENET_PHY_RST  (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTA | PIN28)
#define PIN_ENET_PHY_EN   (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTA | PIN29)

/* Board provides GPIO or other Hardware for signaling to timing analyzer */

#if defined(CONFIG_BOARD_USE_PROBES)
# define PROBE_N(n) (1<<((n)-1))
# define PROBE_1  (GPIO_LOWDRIVE | GPIO_OUTPUT_ZER0  | PIN_PORTC | PIN1)
# define PROBE_2  (GPIO_LOWDRIVE | GPIO_OUTPUT_ZER0  | PIN_PORTA | PIN6)
# define PROBE_3  (GPIO_LOWDRIVE | GPIO_OUTPUT_ZER0  | PIN_PORTD | PIN4)
# define PROBE_4  (GPIO_LOWDRIVE | GPIO_OUTPUT_ZER0  | PIN_PORTD | PIN5)
# define PROBE_5  (GPIO_LOWDRIVE | GPIO_OUTPUT_ZER0  | PIN_PORTE | PIN11)
# define PROBE_6  (GPIO_LOWDRIVE | GPIO_OUTPUT_ZER0  | PIN_PORTE | PIN12)

# define PROBE_INIT(mask) \
	do { \
		if ((mask)& PROBE_N(1)) { kinetis_pinconfig(PROBE_1); } \
		if ((mask)& PROBE_N(2)) { kinetis_pinconfig(PROBE_2); } \
		if ((mask)& PROBE_N(3)) { kinetis_pinconfig(PROBE_3); } \
		if ((mask)& PROBE_N(4)) { kinetis_pinconfig(PROBE_4); } \
		if ((mask)& PROBE_N(5)) { kinetis_pinconfig(PROBE_5); } \
		if ((mask)& PROBE_N(6)) { kinetis_pinconfig(PROBE_6); } \
	} while(0)

# define PROBE(n,s)  do {kinetis_gpiowrite(PROBE_##n,(s));}while(0)
# define PROBE_MARK(n) PROBE(n,false);PROBE(n,true)
#else
# define PROBE_INIT(mask)
# define PROBE(n,s)
# define PROBE_MARK(n)
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: kinetis_boardinitialize
 *
 * Description:
 *   All kinetis architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void kinetis_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_NXP_FMUK66_V3_INCLUDE_BOARD_H */
