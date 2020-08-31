/*******************************************************************************
  *         The following FDCAN register definitions were taken from:
  * @file    stm32h743xx.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-August-2017
  * @brief   CMSIS STM32H743xx Device Peripheral Access Layer Header File.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* ------ Adapted for UAVCAN v0.9 by: Jacob Crabill <jacob@flyvoly.com> ------ */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>

#define FDCAN1_IT0_IRQn 19 /*!< FDCAN1 Interrupt line 0 */
#define FDCAN2_IT0_IRQn 20 /*!< FDCAN2 Interrupt line 0 */
#define FDCAN1_IT1_IRQn 21 /*!< FDCAN1 Interrupt line 1 */
#define FDCAN2_IT1_IRQn 22 /*!< FDCAN2 Interrupt line 1 */

typedef struct {
	uint32_t CREL;         /*!< FDCAN Core Release register,                                     Address offset: 0x000 */
	uint32_t ENDN;         /*!< FDCAN Endian register,                                           Address offset: 0x004 */
	uint32_t RESERVED1;    /*!< Reserved,                                                                        0x008 */
	uint32_t DBTP;         /*!< FDCAN Data Bit Timing & Prescaler register,                      Address offset: 0x00C */
	uint32_t TEST;         /*!< FDCAN Test register,                                             Address offset: 0x010 */
	uint32_t RWD;          /*!< FDCAN RAM Watchdog register,                                     Address offset: 0x014 */
	uint32_t CCCR;         /*!< FDCAN CC Control register,                                       Address offset: 0x018 */
	uint32_t NBTP;         /*!< FDCAN Nominal Bit Timing & Prescaler register,                   Address offset: 0x01C */
	uint32_t TSCC;         /*!< FDCAN Timestamp Counter Configuration register,                  Address offset: 0x020 */
	uint32_t TSCV;         /*!< FDCAN Timestamp Counter Value register,                          Address offset: 0x024 */
	uint32_t TOCC;         /*!< FDCAN Timeout Counter Configuration register,                    Address offset: 0x028 */
	uint32_t TOCV;         /*!< FDCAN Timeout Counter Value register,                            Address offset: 0x02C */
	uint32_t RESERVED2[4]; /*!< Reserved,                                                                0x030 - 0x03C */
	uint32_t ECR;          /*!< FDCAN Error Counter register,                                    Address offset: 0x040 */
	uint32_t PSR;          /*!< FDCAN Protocol Status register,                                  Address offset: 0x044 */
	uint32_t TDCR;         /*!< FDCAN Transmitter Delay Compensation register,                   Address offset: 0x048 */
	uint32_t RESERVED3;    /*!< Reserved,                                                                        0x04C */
	uint32_t IR;           /*!< FDCAN Interrupt register,                                        Address offset: 0x050 */
	uint32_t IE;           /*!< FDCAN Interrupt Enable register,                                 Address offset: 0x054 */
	uint32_t ILS;          /*!< FDCAN Interrupt Line Select register,                            Address offset: 0x058 */
	uint32_t ILE;          /*!< FDCAN Interrupt Line Enable register,                            Address offset: 0x05C */
	uint32_t RESERVED4[8]; /*!< Reserved,                                                                0x060 - 0x07C */
	uint32_t GFC;          /*!< FDCAN Global Filter Configuration register,                      Address offset: 0x080 */
	uint32_t SIDFC;        /*!< FDCAN Standard ID Filter Configuration register,                 Address offset: 0x084 */
	uint32_t XIDFC;        /*!< FDCAN Extended ID Filter Configuration register,                 Address offset: 0x088 */
	uint32_t RESERVED5;    /*!< Reserved,                                                                        0x08C */
	uint32_t XIDAM;        /*!< FDCAN Extended ID AND Mask register,                             Address offset: 0x090 */
	uint32_t HPMS;         /*!< FDCAN High Priority Message Status register,                     Address offset: 0x094 */
	uint32_t NDAT1;        /*!< FDCAN New Data 1 register,                                       Address offset: 0x098 */
	uint32_t NDAT2;        /*!< FDCAN New Data 2 register,                                       Address offset: 0x09C */
	uint32_t RXF0C;        /*!< FDCAN Rx FIFO 0 Configuration register,                          Address offset: 0x0A0 */
	uint32_t RXF0S;        /*!< FDCAN Rx FIFO 0 Status register,                                 Address offset: 0x0A4 */
	uint32_t RXF0A;        /*!< FDCAN Rx FIFO 0 Acknowledge register,                            Address offset: 0x0A8 */
	uint32_t RXBC;         /*!< FDCAN Rx Buffer Configuration register,                          Address offset: 0x0AC */
	uint32_t RXF1C;        /*!< FDCAN Rx FIFO 1 Configuration register,                          Address offset: 0x0B0 */
	uint32_t RXF1S;        /*!< FDCAN Rx FIFO 1 Status register,                                 Address offset: 0x0B4 */
	uint32_t RXF1A;        /*!< FDCAN Rx FIFO 1 Acknowledge register,                            Address offset: 0x0B8 */
	uint32_t RXESC;        /*!< FDCAN Rx Buffer/FIFO Element Size Configuration register,        Address offset: 0x0BC */
	uint32_t TXBC;         /*!< FDCAN Tx Buffer Configuration register,                          Address offset: 0x0C0 */
	uint32_t TXFQS;        /*!< FDCAN Tx FIFO/Queue Status register,                             Address offset: 0x0C4 */
	uint32_t TXESC;        /*!< FDCAN Tx Buffer Element Size Configuration register,             Address offset: 0x0C8 */
	uint32_t TXBRP;        /*!< FDCAN Tx Buffer Request Pending register,                        Address offset: 0x0CC */
	uint32_t TXBAR;        /*!< FDCAN Tx Buffer Add Request register,                            Address offset: 0x0D0 */
	uint32_t TXBCR;        /*!< FDCAN Tx Buffer Cancellation Request register,                   Address offset: 0x0D4 */
	uint32_t TXBTO;        /*!< FDCAN Tx Buffer Transmission Occurred register,                  Address offset: 0x0D8 */
	uint32_t TXBCF;        /*!< FDCAN Tx Buffer Cancellation Finished register,                  Address offset: 0x0DC */
	uint32_t TXBTIE;       /*!< FDCAN Tx Buffer Transmission Interrupt Enable register,          Address offset: 0x0E0 */
	uint32_t TXBCIE;       /*!< FDCAN Tx Buffer Cancellation Finished Interrupt Enable register, Address offset: 0x0E4 */
	uint32_t RESERVED6[2]; /*!< Reserved,                                                                0x0E8 - 0x0EC */
	uint32_t TXEFC;        /*!< FDCAN Tx Event FIFO Configuration register,                      Address offset: 0x0F0 */
	uint32_t TXEFS;        /*!< FDCAN Tx Event FIFO Status register,                             Address offset: 0x0F4 */
	uint32_t TXEFA;        /*!< FDCAN Tx Event FIFO Acknowledge register,                        Address offset: 0x0F8 */
	uint32_t RESERVED7;    /*!< Reserved,                                                                        0x0FC */
} FDCAN_GlobalTypeDef;

/**
  * @brief TTFD Controller Area Network
  */

typedef struct {
	uint32_t TTTMC;          /*!< TT Trigger Memory Configuration register,    Address offset: 0x100 */
	uint32_t TTRMC;          /*!< TT Reference Message Configuration register, Address offset: 0x104 */
	uint32_t TTOCF;          /*!< TT Operation Configuration register,         Address offset: 0x108 */
	uint32_t TTMLM;          /*!< TT Matrix Limits register,                   Address offset: 0x10C */
	uint32_t TURCF;          /*!< TUR Configuration register,                  Address offset: 0x110 */
	uint32_t TTOCN;          /*!< TT Operation Control register,               Address offset: 0x114 */
	uint32_t TTGTP;          /*!< TT Global Time Preset register,              Address offset: 0x118 */
	uint32_t TTTMK;          /*!< TT Time Mark register,                       Address offset: 0x11C */
	uint32_t TTIR;           /*!< TT Interrupt register,                       Address offset: 0x120 */
	uint32_t TTIE;           /*!< TT Interrupt Enable register,                Address offset: 0x124 */
	uint32_t TTILS;          /*!< TT Interrupt Line Select register,           Address offset: 0x128 */
	uint32_t TTOST;          /*!< TT Operation Status register,                Address offset: 0x12C */
	uint32_t TURNA;          /*!< TT TUR Numerator Actual register,            Address offset: 0x130 */
	uint32_t TTLGT;          /*!< TT Local and Global Time register,           Address offset: 0x134 */
	uint32_t TTCTC;          /*!< TT Cycle Time and Count register,            Address offset: 0x138 */
	uint32_t TTCPT;          /*!< TT Capture Time register,                    Address offset: 0x13C */
	uint32_t TTCSM;          /*!< TT Cycle Sync Mark register,                 Address offset: 0x140 */
	uint32_t RESERVED1[111]; /*!< Reserved,                                            0x144 - 0x2FC */
	uint32_t TTTS;           /*!< TT Trigger Select register,                  Address offset: 0x300 */
} TTCAN_TypeDef;

/**
  * @brief FD Controller Area Network
  */

typedef struct {
	uint32_t CREL;  /*!< Clock Calibration Unit Core Release register, Address offset: 0x00 */
	uint32_t CCFG;  /*!< Calibration Configuration register,           Address offset: 0x04 */
	uint32_t CSTAT; /*!< Calibration Status register,                  Address offset: 0x08 */
	uint32_t CWD;   /*!< Calibration Watchdog register,                Address offset: 0x0C */
	uint32_t IR;    /*!< CCU Interrupt register,                       Address offset: 0x10 */
	uint32_t IE;    /*!< CCU Interrupt Enable register,                Address offset: 0x14 */
} FDCAN_ClockCalibrationUnit_TypeDef;


/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Base address of : AHB/ABP Peripherals                                                   */
#define D2_APB1PERIPH_BASE    PERIPH_BASE

#define FDCAN1_BASE           (D2_APB1PERIPH_BASE + 0xA000)
#define FDCAN2_BASE           (D2_APB1PERIPH_BASE + 0xA400)
#define FDCAN_CCU_BASE        (D2_APB1PERIPH_BASE + 0xA800)
#define SRAMCAN_BASE          (D2_APB1PERIPH_BASE + 0xAC00)

/******************************************************************************/
/*                                                                            */
/*                 Flexible Datarate Controller Area Network                  */
/*                                                                            */
/******************************************************************************/
/*!<FDCAN control and status registers */
/*****************  Bit definition for FDCAN_CREL register  *******************/
#define FDCAN_CREL_DAY_Pos        (0U)
#define FDCAN_CREL_DAY_Msk        (0xFFU << FDCAN_CREL_DAY_Pos)                /*!< 0x000000FF */
#define FDCAN_CREL_DAY            FDCAN_CREL_DAY_Msk                           /*!<Timestamp Day                           */
#define FDCAN_CREL_MON_Pos        (8U)
#define FDCAN_CREL_MON_Msk        (0xFFU << FDCAN_CREL_MON_Pos)                /*!< 0x0000FF00 */
#define FDCAN_CREL_MON            FDCAN_CREL_MON_Msk                           /*!<Timestamp Month                         */
#define FDCAN_CREL_YEAR_Pos       (16U)
#define FDCAN_CREL_YEAR_Msk       (0xFU << FDCAN_CREL_YEAR_Pos)                /*!< 0x000F0000 */
#define FDCAN_CREL_YEAR           FDCAN_CREL_YEAR_Msk                          /*!<Timestamp Year                          */
#define FDCAN_CREL_SUBSTEP_Pos    (20U)
#define FDCAN_CREL_SUBSTEP_Msk    (0xFU << FDCAN_CREL_SUBSTEP_Pos)             /*!< 0x00F00000 */
#define FDCAN_CREL_SUBSTEP        FDCAN_CREL_SUBSTEP_Msk                       /*!<Sub-step of Core release                */
#define FDCAN_CREL_STEP_Pos       (24U)
#define FDCAN_CREL_STEP_Msk       (0xFU << FDCAN_CREL_STEP_Pos)                /*!< 0x0F000000 */
#define FDCAN_CREL_STEP           FDCAN_CREL_STEP_Msk                          /*!<Step of Core release                    */
#define FDCAN_CREL_REL_Pos        (28U)
#define FDCAN_CREL_REL_Msk        (0xFU << FDCAN_CREL_REL_Pos)                 /*!< 0xF0000000 */
#define FDCAN_CREL_REL            FDCAN_CREL_REL_Msk                           /*!<Core release                            */

/*****************  Bit definition for FDCAN_ENDN register  *******************/
#define FDCAN_ENDN_ETV_Pos        (0U)
#define FDCAN_ENDN_ETV_Msk        (0xFFFFFFFFU << FDCAN_ENDN_ETV_Pos)          /*!< 0xFFFFFFFF */
#define FDCAN_ENDN_ETV            FDCAN_ENDN_ETV_Msk                           /*!<Endiannes Test Value                    */

/*****************  Bit definition for FDCAN_DBTP register  *******************/
#define FDCAN_DBTP_DSJW_Pos       (0U)
#define FDCAN_DBTP_DSJW_Msk       (0xFU << FDCAN_DBTP_DSJW_Pos)                /*!< 0x0000000F */
#define FDCAN_DBTP_DSJW           FDCAN_DBTP_DSJW_Msk                          /*!<Synchronization Jump Width              */
#define FDCAN_DBTP_DTSEG2_Pos     (4U)
#define FDCAN_DBTP_DTSEG2_Msk     (0xFU << FDCAN_DBTP_DTSEG2_Pos)              /*!< 0x000000F0 */
#define FDCAN_DBTP_DTSEG2         FDCAN_DBTP_DTSEG2_Msk                        /*!<Data time segment after sample point    */
#define FDCAN_DBTP_DTSEG1_Pos     (8U)
#define FDCAN_DBTP_DTSEG1_Msk     (0xFU << FDCAN_DBTP_DTSEG1_Pos)              /*!< 0x00000F00 */
#define FDCAN_DBTP_DTSEG1         FDCAN_DBTP_DTSEG1_Msk                        /*!<Data time segment before sample point   */
#define FDCAN_DBTP_DBRP_Pos       (16U)
#define FDCAN_DBTP_DBRP_Msk       (0x1FU << FDCAN_DBTP_DBRP_Pos)               /*!< 0x001F0000 */
#define FDCAN_DBTP_DBRP           FDCAN_DBTP_DBRP_Msk                          /*!<Data BIt Rate Prescaler                 */
#define FDCAN_DBTP_TDC_Pos        (23U)
#define FDCAN_DBTP_TDC_Msk        (0x1U << FDCAN_DBTP_TDC_Pos)                 /*!< 0x00800000 */
#define FDCAN_DBTP_TDC            FDCAN_DBTP_TDC_Msk                           /*!<Transceiver Delay Compensation          */

/*****************  Bit definition for FDCAN_TEST register  *******************/
#define FDCAN_TEST_LBCK_Pos       (4U)
#define FDCAN_TEST_LBCK_Msk       (0x1U << FDCAN_TEST_LBCK_Pos)                /*!< 0x00000010 */
#define FDCAN_TEST_LBCK           FDCAN_TEST_LBCK_Msk                          /*!<Loop Back mode                           */
#define FDCAN_TEST_TX_Pos         (5U)
#define FDCAN_TEST_TX_Msk         (0x3U << FDCAN_TEST_TX_Pos)                  /*!< 0x00000060 */
#define FDCAN_TEST_TX             FDCAN_TEST_TX_Msk                            /*!<Control of Transmit Pin                  */
#define FDCAN_TEST_RX_Pos         (7U)
#define FDCAN_TEST_RX_Msk         (0x1U << FDCAN_TEST_RX_Pos)                  /*!< 0x00000080 */
#define FDCAN_TEST_RX             FDCAN_TEST_RX_Msk                            /*!<Receive Pin                              */

/*****************  Bit definition for FDCAN_RWD register  ********************/
#define FDCAN_RWD_WDC_Pos         (0U)
#define FDCAN_RWD_WDC_Msk         (0xFU << FDCAN_RWD_WDC_Pos)                  /*!< 0x0000000F */
#define FDCAN_RWD_WDC             FDCAN_RWD_WDC_Msk                            /*!<Watchdog configuration                   */
#define FDCAN_RWD_WDV_Pos         (4U)
#define FDCAN_RWD_WDV_Msk         (0xFU << FDCAN_RWD_WDV_Pos)                  /*!< 0x000000F0 */
#define FDCAN_RWD_WDV             FDCAN_RWD_WDV_Msk                            /*!<Watchdog value                           */

/*****************  Bit definition for FDCAN_CCCR register  ********************/
#define FDCAN_CCCR_INIT_Pos       (0U)
#define FDCAN_CCCR_INIT_Msk       (0x1U << FDCAN_CCCR_INIT_Pos)                /*!< 0x00000001 */
#define FDCAN_CCCR_INIT           FDCAN_CCCR_INIT_Msk                          /*!<Initialization                           */
#define FDCAN_CCCR_CCE_Pos        (1U)
#define FDCAN_CCCR_CCE_Msk        (0x1U << FDCAN_CCCR_CCE_Pos)                 /*!< 0x00000002 */
#define FDCAN_CCCR_CCE            FDCAN_CCCR_CCE_Msk                           /*!<Configuration Change Enable              */
#define FDCAN_CCCR_ASM_Pos        (2U)
#define FDCAN_CCCR_ASM_Msk        (0x1U << FDCAN_CCCR_ASM_Pos)                 /*!< 0x00000004 */
#define FDCAN_CCCR_ASM            FDCAN_CCCR_ASM_Msk                           /*!<ASM Restricted Operation Mode            */
#define FDCAN_CCCR_CSA_Pos        (3U)
#define FDCAN_CCCR_CSA_Msk        (0x1U << FDCAN_CCCR_CSA_Pos)                 /*!< 0x00000008 */
#define FDCAN_CCCR_CSA            FDCAN_CCCR_CSA_Msk                           /*!<Clock Stop Acknowledge                   */
#define FDCAN_CCCR_CSR_Pos        (4U)
#define FDCAN_CCCR_CSR_Msk        (0x1U << FDCAN_CCCR_CSR_Pos)                 /*!< 0x00000010 */
#define FDCAN_CCCR_CSR            FDCAN_CCCR_CSR_Msk                           /*!<Clock Stop Request                       */
#define FDCAN_CCCR_MON_Pos        (5U)
#define FDCAN_CCCR_MON_Msk        (0x1U << FDCAN_CCCR_MON_Pos)                 /*!< 0x00000020 */
#define FDCAN_CCCR_MON            FDCAN_CCCR_MON_Msk                           /*!<Bus Monitoring Mode                      */
#define FDCAN_CCCR_DAR_Pos        (6U)
#define FDCAN_CCCR_DAR_Msk        (0x1U << FDCAN_CCCR_DAR_Pos)                 /*!< 0x00000040 */
#define FDCAN_CCCR_DAR            FDCAN_CCCR_DAR_Msk                           /*!<Disable Automatic Retransmission         */
#define FDCAN_CCCR_TEST_Pos       (7U)
#define FDCAN_CCCR_TEST_Msk       (0x1U << FDCAN_CCCR_TEST_Pos)                /*!< 0x00000080 */
#define FDCAN_CCCR_TEST           FDCAN_CCCR_TEST_Msk                          /*!<Test Mode Enable                         */
#define FDCAN_CCCR_FDOE_Pos       (8U)
#define FDCAN_CCCR_FDOE_Msk       (0x1U << FDCAN_CCCR_FDOE_Pos)                /*!< 0x00000100 */
#define FDCAN_CCCR_FDOE           FDCAN_CCCR_FDOE_Msk                          /*!<FD Operation Enable                      */
#define FDCAN_CCCR_BRSE_Pos       (9U)
#define FDCAN_CCCR_BRSE_Msk       (0x1U << FDCAN_CCCR_BRSE_Pos)                /*!< 0x00000200 */
#define FDCAN_CCCR_BRSE           FDCAN_CCCR_BRSE_Msk                          /*!<FDCAN Bit Rate Switching                 */
#define FDCAN_CCCR_PXHD_Pos       (12U)
#define FDCAN_CCCR_PXHD_Msk       (0x1U << FDCAN_CCCR_PXHD_Pos)                /*!< 0x00001000 */
#define FDCAN_CCCR_PXHD           FDCAN_CCCR_PXHD_Msk                          /*!<Protocol Exception Handling Disable      */
#define FDCAN_CCCR_EFBI_Pos       (13U)
#define FDCAN_CCCR_EFBI_Msk       (0x1U << FDCAN_CCCR_EFBI_Pos)                /*!< 0x00002000 */
#define FDCAN_CCCR_EFBI           FDCAN_CCCR_EFBI_Msk                          /*!<Edge Filtering during Bus Integration    */
#define FDCAN_CCCR_TXP_Pos        (14U)
#define FDCAN_CCCR_TXP_Msk        (0x1U << FDCAN_CCCR_TXP_Pos)                 /*!< 0x00004000 */
#define FDCAN_CCCR_TXP            FDCAN_CCCR_TXP_Msk                           /*!<Two CAN bit times Pause                  */
#define FDCAN_CCCR_NISO_Pos       (15U)
#define FDCAN_CCCR_NISO_Msk       (0x1U << FDCAN_CCCR_NISO_Pos)                /*!< 0x00008000 */
#define FDCAN_CCCR_NISO           FDCAN_CCCR_NISO_Msk                          /*!<Non ISO Operation                        */

/*****************  Bit definition for FDCAN_NBTP register  ********************/
#define FDCAN_NBTP_TSEG2_Pos      (0U)
#define FDCAN_NBTP_TSEG2_Msk      (0x7FU << FDCAN_NBTP_TSEG2_Pos)              /*!< 0x0000007F */
#define FDCAN_NBTP_TSEG2          FDCAN_NBTP_TSEG2_Msk                         /*!<Nominal Time segment after sample point  */
#define FDCAN_NBTP_NTSEG1_Pos     (8U)
#define FDCAN_NBTP_NTSEG1_Msk     (0xFFU << FDCAN_NBTP_NTSEG1_Pos)             /*!< 0x0000FF00 */
#define FDCAN_NBTP_NTSEG1         FDCAN_NBTP_NTSEG1_Msk                        /*!<Nominal Time segment before sample point */
#define FDCAN_NBTP_NBRP_Pos       (16U)
#define FDCAN_NBTP_NBRP_Msk       (0x1FFU << FDCAN_NBTP_NBRP_Pos)              /*!< 0x01FF0000 */
#define FDCAN_NBTP_NBRP           FDCAN_NBTP_NBRP_Msk                          /*!<Bit Rate Prescaler                       */
#define FDCAN_NBTP_NSJW_Pos       (25U)
#define FDCAN_NBTP_NSJW_Msk       (0x7FU << FDCAN_NBTP_NSJW_Pos)               /*!< 0xFE000000 */
#define FDCAN_NBTP_NSJW           FDCAN_NBTP_NSJW_Msk                          /*!<Nominal (Re)Synchronization Jump Width   */

/*****************  Bit definition for FDCAN_TSCC register  ********************/
#define FDCAN_TSCC_TSS_Pos        (0U)
#define FDCAN_TSCC_TSS_Msk        (0x3U << FDCAN_TSCC_TSS_Pos)                 /*!< 0x00000003 */
#define FDCAN_TSCC_TSS            FDCAN_TSCC_TSS_Msk                           /*!<Timestamp Select                         */
#define FDCAN_TSCC_TCP_Pos        (16U)
#define FDCAN_TSCC_TCP_Msk        (0xFU << FDCAN_TSCC_TCP_Pos)                 /*!< 0x000F0000 */
#define FDCAN_TSCC_TCP            FDCAN_TSCC_TCP_Msk                           /*!<Timestamp Counter Prescaler              */

/*****************  Bit definition for FDCAN_TSCV register  ********************/
#define FDCAN_TSCV_TSC_Pos        (0U)
#define FDCAN_TSCV_TSC_Msk        (0xFFFFU << FDCAN_TSCV_TSC_Pos)              /*!< 0x0000FFFF */
#define FDCAN_TSCV_TSC            FDCAN_TSCV_TSC_Msk                           /*!<Timestamp Counter                        */

/*****************  Bit definition for FDCAN_TOCC register  ********************/
#define FDCAN_TOCC_ETOC_Pos       (0U)
#define FDCAN_TOCC_ETOC_Msk       (0x1U << FDCAN_TOCC_ETOC_Pos)                /*!< 0x00000001 */
#define FDCAN_TOCC_ETOC           FDCAN_TOCC_ETOC_Msk                          /*!<Enable Timeout Counter                   */
#define FDCAN_TOCC_TOS_Pos        (1U)
#define FDCAN_TOCC_TOS_Msk        (0x3U << FDCAN_TOCC_TOS_Pos)                 /*!< 0x00000006 */
#define FDCAN_TOCC_TOS            FDCAN_TOCC_TOS_Msk                           /*!<Timeout Select                           */
#define FDCAN_TOCC_TOP_Pos        (16U)
#define FDCAN_TOCC_TOP_Msk        (0xFFFFU << FDCAN_TOCC_TOP_Pos)              /*!< 0xFFFF0000 */
#define FDCAN_TOCC_TOP            FDCAN_TOCC_TOP_Msk                           /*!<Timeout Period                           */

/*****************  Bit definition for FDCAN_TOCV register  ********************/
#define FDCAN_TOCV_TOC_Pos        (0U)
#define FDCAN_TOCV_TOC_Msk        (0xFFFFU << FDCAN_TOCV_TOC_Pos)              /*!< 0x0000FFFF */
#define FDCAN_TOCV_TOC            FDCAN_TOCV_TOC_Msk                           /*!<Timeout Counter                          */

/*****************  Bit definition for FDCAN_ECR register  *********************/
#define FDCAN_ECR_TEC_Pos         (0U)
#define FDCAN_ECR_TEC_Msk         (0xFU << FDCAN_ECR_TEC_Pos)                  /*!< 0x0000000F */
#define FDCAN_ECR_TEC             FDCAN_ECR_TEC_Msk                            /*!<Transmit Error Counter                   */
#define FDCAN_ECR_TREC_Pos        (8U)
#define FDCAN_ECR_TREC_Msk        (0x7FU << FDCAN_ECR_TREC_Pos)                /*!< 0x00007F00 */
#define FDCAN_ECR_TREC            FDCAN_ECR_TREC_Msk                           /*!<Receive Error Counter                    */
#define FDCAN_ECR_RP_Pos          (15U)
#define FDCAN_ECR_RP_Msk          (0x1U << FDCAN_ECR_RP_Pos)                   /*!< 0x00008000 */
#define FDCAN_ECR_RP              FDCAN_ECR_RP_Msk                             /*!<Receive Error Passive                    */
#define FDCAN_ECR_CEL_Pos         (16U)
#define FDCAN_ECR_CEL_Msk         (0xFFU << FDCAN_ECR_CEL_Pos)                 /*!< 0x00FF0000 */
#define FDCAN_ECR_CEL             FDCAN_ECR_CEL_Msk                            /*!<CAN Error Logging                        */

/*****************  Bit definition for FDCAN_PSR register  *********************/
#define FDCAN_PSR_LEC_Pos         (0U)
#define FDCAN_PSR_LEC_Msk         (0x7U << FDCAN_PSR_LEC_Pos)                  /*!< 0x00000007 */
#define FDCAN_PSR_LEC             FDCAN_PSR_LEC_Msk                            /*!<Last Error Code                          */
#define FDCAN_PSR_ACT_Pos         (3U)
#define FDCAN_PSR_ACT_Msk         (0x3U << FDCAN_PSR_ACT_Pos)                  /*!< 0x00000018 */
#define FDCAN_PSR_ACT             FDCAN_PSR_ACT_Msk                            /*!<Activity                                 */
#define FDCAN_PSR_EP_Pos          (5U)
#define FDCAN_PSR_EP_Msk          (0x1U << FDCAN_PSR_EP_Pos)                   /*!< 0x00000020 */
#define FDCAN_PSR_EP              FDCAN_PSR_EP_Msk                             /*!<Error Passive                            */
#define FDCAN_PSR_EW_Pos          (6U)
#define FDCAN_PSR_EW_Msk          (0x1U << FDCAN_PSR_EW_Pos)                   /*!< 0x00000040 */
#define FDCAN_PSR_EW              FDCAN_PSR_EW_Msk                             /*!<Warning Status                           */
#define FDCAN_PSR_BO_Pos          (7U)
#define FDCAN_PSR_BO_Msk          (0x1U << FDCAN_PSR_BO_Pos)                   /*!< 0x00000080 */
#define FDCAN_PSR_BO              FDCAN_PSR_BO_Msk                             /*!<Bus_Off Status                           */
#define FDCAN_PSR_DLEC_Pos        (8U)
#define FDCAN_PSR_DLEC_Msk        (0x7U << FDCAN_PSR_DLEC_Pos)                 /*!< 0x00000700 */
#define FDCAN_PSR_DLEC            FDCAN_PSR_DLEC_Msk                           /*!<Data Last Error Code                     */
#define FDCAN_PSR_RESI_Pos        (11U)
#define FDCAN_PSR_RESI_Msk        (0x1U << FDCAN_PSR_RESI_Pos)                 /*!< 0x00000800 */
#define FDCAN_PSR_RESI            FDCAN_PSR_RESI_Msk                           /*!<ESI flag of last received FDCAN Message  */
#define FDCAN_PSR_RBRS_Pos        (12U)
#define FDCAN_PSR_RBRS_Msk        (0x1U << FDCAN_PSR_RBRS_Pos)                 /*!< 0x00001000 */
#define FDCAN_PSR_RBRS            FDCAN_PSR_RBRS_Msk                           /*!<BRS flag of last received FDCAN Message  */
#define FDCAN_PSR_REDL_Pos        (13U)
#define FDCAN_PSR_REDL_Msk        (0x1U << FDCAN_PSR_REDL_Pos)                 /*!< 0x00002000 */
#define FDCAN_PSR_REDL            FDCAN_PSR_REDL_Msk                           /*!<Received FDCAN Message                   */
#define FDCAN_PSR_PXE_Pos         (14U)
#define FDCAN_PSR_PXE_Msk         (0x1U << FDCAN_PSR_PXE_Pos)                  /*!< 0x00004000 */
#define FDCAN_PSR_PXE             FDCAN_PSR_PXE_Msk                            /*!<Protocol Exception Event                 */
#define FDCAN_PSR_TDCV_Pos        (16U)
#define FDCAN_PSR_TDCV_Msk        (0x7FU << FDCAN_PSR_TDCV_Pos)                /*!< 0x007F0000 */
#define FDCAN_PSR_TDCV            FDCAN_PSR_TDCV_Msk                           /*!<Transmitter Delay Compensation Value     */

/*****************  Bit definition for FDCAN_TDCR register  ********************/
#define FDCAN_TDCR_TDCF_Pos       (0U)
#define FDCAN_TDCR_TDCF_Msk       (0x7FU << FDCAN_TDCR_TDCF_Pos)               /*!< 0x0000007F */
#define FDCAN_TDCR_TDCF           FDCAN_TDCR_TDCF_Msk                          /*!<Transmitter Delay Compensation Filter    */
#define FDCAN_TDCR_TDCO_Pos       (8U)
#define FDCAN_TDCR_TDCO_Msk       (0x7FU << FDCAN_TDCR_TDCO_Pos)               /*!< 0x00007F00 */
#define FDCAN_TDCR_TDCO           FDCAN_TDCR_TDCO_Msk                          /*!<Transmitter Delay Compensation Offset    */

/*****************  Bit definition for FDCAN_IR register  **********************/
#define FDCAN_IR_RF0N_Pos         (0U)
#define FDCAN_IR_RF0N_Msk         (0x1U << FDCAN_IR_RF0N_Pos)                  /*!< 0x00000001 */
#define FDCAN_IR_RF0N             FDCAN_IR_RF0N_Msk                            /*!<Rx FIFO 0 New Message                    */
#define FDCAN_IR_RF0W_Pos         (1U)
#define FDCAN_IR_RF0W_Msk         (0x1U << FDCAN_IR_RF0W_Pos)                  /*!< 0x00000002 */
#define FDCAN_IR_RF0W             FDCAN_IR_RF0W_Msk                            /*!<Rx FIFO 0 Watermark Reached              */
#define FDCAN_IR_RF0F_Pos         (2U)
#define FDCAN_IR_RF0F_Msk         (0x1U << FDCAN_IR_RF0F_Pos)                  /*!< 0x00000004 */
#define FDCAN_IR_RF0F             FDCAN_IR_RF0F_Msk                            /*!<Rx FIFO 0 Full                           */
#define FDCAN_IR_RF0L_Pos         (3U)
#define FDCAN_IR_RF0L_Msk         (0x1U << FDCAN_IR_RF0L_Pos)                  /*!< 0x00000008 */
#define FDCAN_IR_RF0L             FDCAN_IR_RF0L_Msk                            /*!<Rx FIFO 0 Message Lost                   */
#define FDCAN_IR_RF1N_Pos         (4U)
#define FDCAN_IR_RF1N_Msk         (0x1U << FDCAN_IR_RF1N_Pos)                  /*!< 0x00000010 */
#define FDCAN_IR_RF1N             FDCAN_IR_RF1N_Msk                            /*!<Rx FIFO 1 New Message                    */
#define FDCAN_IR_RF1W_Pos         (5U)
#define FDCAN_IR_RF1W_Msk         (0x1U << FDCAN_IR_RF1W_Pos)                  /*!< 0x00000020 */
#define FDCAN_IR_RF1W             FDCAN_IR_RF1W_Msk                            /*!<Rx FIFO 1 Watermark Reached              */
#define FDCAN_IR_RF1F_Pos         (6U)
#define FDCAN_IR_RF1F_Msk         (0x1U << FDCAN_IR_RF1F_Pos)                  /*!< 0x00000040 */
#define FDCAN_IR_RF1F             FDCAN_IR_RF1F_Msk                            /*!<Rx FIFO 1 Full                           */
#define FDCAN_IR_RF1L_Pos         (7U)
#define FDCAN_IR_RF1L_Msk         (0x1U << FDCAN_IR_RF1L_Pos)                  /*!< 0x00000080 */
#define FDCAN_IR_RF1L             FDCAN_IR_RF1L_Msk                            /*!<Rx FIFO 1 Message Lost                   */
#define FDCAN_IR_HPM_Pos          (8U)
#define FDCAN_IR_HPM_Msk          (0x1U << FDCAN_IR_HPM_Pos)                   /*!< 0x00000100 */
#define FDCAN_IR_HPM              FDCAN_IR_HPM_Msk                             /*!<High Priority Message                    */
#define FDCAN_IR_TC_Pos           (9U)
#define FDCAN_IR_TC_Msk           (0x1U << FDCAN_IR_TC_Pos)                    /*!< 0x00000200 */
#define FDCAN_IR_TC               FDCAN_IR_TC_Msk                              /*!<Transmission Completed                   */
#define FDCAN_IR_TCF_Pos          (10U)
#define FDCAN_IR_TCF_Msk          (0x1U << FDCAN_IR_TCF_Pos)                   /*!< 0x00000400 */
#define FDCAN_IR_TCF              FDCAN_IR_TCF_Msk                             /*!<Transmission Cancellation Finished       */
#define FDCAN_IR_TFE_Pos          (11U)
#define FDCAN_IR_TFE_Msk          (0x1U << FDCAN_IR_TFE_Pos)                   /*!< 0x00000800 */
#define FDCAN_IR_TFE              FDCAN_IR_TFE_Msk                             /*!<Tx FIFO Empty                            */
#define FDCAN_IR_TEFN_Pos         (12U)
#define FDCAN_IR_TEFN_Msk         (0x1U << FDCAN_IR_TEFN_Pos)                  /*!< 0x00001000 */
#define FDCAN_IR_TEFN             FDCAN_IR_TEFN_Msk                            /*!<Tx Event FIFO New Entry                  */
#define FDCAN_IR_TEFW_Pos         (13U)
#define FDCAN_IR_TEFW_Msk         (0x1U << FDCAN_IR_TEFW_Pos)                  /*!< 0x00002000 */
#define FDCAN_IR_TEFW             FDCAN_IR_TEFW_Msk                            /*!<Tx Event FIFO Watermark Reached          */
#define FDCAN_IR_TEFF_Pos         (14U)
#define FDCAN_IR_TEFF_Msk         (0x1U << FDCAN_IR_TEFF_Pos)                  /*!< 0x00004000 */
#define FDCAN_IR_TEFF             FDCAN_IR_TEFF_Msk                            /*!<Tx Event FIFO Full                       */
#define FDCAN_IR_TEFL_Pos         (15U)
#define FDCAN_IR_TEFL_Msk         (0x1U << FDCAN_IR_TEFL_Pos)                  /*!< 0x00008000 */
#define FDCAN_IR_TEFL             FDCAN_IR_TEFL_Msk                            /*!<Tx Event FIFO Element Lost               */
#define FDCAN_IR_TSW_Pos          (16U)
#define FDCAN_IR_TSW_Msk          (0x1U << FDCAN_IR_TSW_Pos)                   /*!< 0x00010000 */
#define FDCAN_IR_TSW              FDCAN_IR_TSW_Msk                             /*!<Timestamp Wraparound                     */
#define FDCAN_IR_MRAF_Pos         (17U)
#define FDCAN_IR_MRAF_Msk         (0x1U << FDCAN_IR_MRAF_Pos)                  /*!< 0x00020000 */
#define FDCAN_IR_MRAF             FDCAN_IR_MRAF_Msk                            /*!<Message RAM Access Failure               */
#define FDCAN_IR_TOO_Pos          (18U)
#define FDCAN_IR_TOO_Msk          (0x1U << FDCAN_IR_TOO_Pos)                   /*!< 0x00040000 */
#define FDCAN_IR_TOO              FDCAN_IR_TOO_Msk                             /*!<Timeout Occurred                         */
#define FDCAN_IR_DRX_Pos          (19U)
#define FDCAN_IR_DRX_Msk          (0x1U << FDCAN_IR_DRX_Pos)                   /*!< 0x00080000 */
#define FDCAN_IR_DRX              FDCAN_IR_DRX_Msk                             /*!<Message stored to Dedicated Rx Buffer    */
#define FDCAN_IR_ELO_Pos          (22U)
#define FDCAN_IR_ELO_Msk          (0x1U << FDCAN_IR_ELO_Pos)                   /*!< 0x00400000 */
#define FDCAN_IR_ELO              FDCAN_IR_ELO_Msk                             /*!<Error Logging Overflow                   */
#define FDCAN_IR_EP_Pos           (23U)
#define FDCAN_IR_EP_Msk           (0x1U << FDCAN_IR_EP_Pos)                    /*!< 0x00800000 */
#define FDCAN_IR_EP               FDCAN_IR_EP_Msk                              /*!<Error Passive                            */
#define FDCAN_IR_EW_Pos           (24U)
#define FDCAN_IR_EW_Msk           (0x1U << FDCAN_IR_EW_Pos)                    /*!< 0x01000000 */
#define FDCAN_IR_EW               FDCAN_IR_EW_Msk                              /*!<Warning Status                           */
#define FDCAN_IR_BO_Pos           (25U)
#define FDCAN_IR_BO_Msk           (0x1U << FDCAN_IR_BO_Pos)                    /*!< 0x02000000 */
#define FDCAN_IR_BO               FDCAN_IR_BO_Msk                              /*!<Bus_Off Status                           */
#define FDCAN_IR_WDI_Pos          (26U)
#define FDCAN_IR_WDI_Msk          (0x1U << FDCAN_IR_WDI_Pos)                   /*!< 0x04000000 */
#define FDCAN_IR_WDI              FDCAN_IR_WDI_Msk                             /*!<Watchdog Interrupt                       */
#define FDCAN_IR_PEA_Pos          (27U)
#define FDCAN_IR_PEA_Msk          (0x1U << FDCAN_IR_PEA_Pos)                   /*!< 0x08000000 */
#define FDCAN_IR_PEA              FDCAN_IR_PEA_Msk                             /*!<Protocol Error in Arbitration Phase      */
#define FDCAN_IR_PED_Pos          (28U)
#define FDCAN_IR_PED_Msk          (0x1U << FDCAN_IR_PED_Pos)                   /*!< 0x10000000 */
#define FDCAN_IR_PED              FDCAN_IR_PED_Msk                             /*!<Protocol Error in Data Phase             */
#define FDCAN_IR_ARA_Pos          (29U)
#define FDCAN_IR_ARA_Msk          (0x1U << FDCAN_IR_ARA_Pos)                   /*!< 0x20000000 */
#define FDCAN_IR_ARA              FDCAN_IR_ARA_Msk                             /*!<Access to Reserved Address               */

/*****************  Bit definition for FDCAN_IE register  **********************/
#define FDCAN_IE_RF0NE_Pos        (0U)
#define FDCAN_IE_RF0NE_Msk        (0x1U << FDCAN_IE_RF0NE_Pos)                 /*!< 0x00000001 */
#define FDCAN_IE_RF0NE            FDCAN_IE_RF0NE_Msk                           /*!<Rx FIFO 0 New Message Enable                 */
#define FDCAN_IE_RF0WE_Pos        (1U)
#define FDCAN_IE_RF0WE_Msk        (0x1U << FDCAN_IE_RF0WE_Pos)                 /*!< 0x00000002 */
#define FDCAN_IE_RF0WE            FDCAN_IE_RF0WE_Msk                           /*!<Rx FIFO 0 Watermark Reached Enable           */
#define FDCAN_IE_RF0FE_Pos        (2U)
#define FDCAN_IE_RF0FE_Msk        (0x1U << FDCAN_IE_RF0FE_Pos)                 /*!< 0x00000004 */
#define FDCAN_IE_RF0FE            FDCAN_IE_RF0FE_Msk                           /*!<Rx FIFO 0 Full Enable                        */
#define FDCAN_IE_RF0LE_Pos        (3U)
#define FDCAN_IE_RF0LE_Msk        (0x1U << FDCAN_IE_RF0LE_Pos)                 /*!< 0x00000008 */
#define FDCAN_IE_RF0LE            FDCAN_IE_RF0LE_Msk                           /*!<Rx FIFO 0 Message Lost Enable                */
#define FDCAN_IE_RF1NE_Pos        (4U)
#define FDCAN_IE_RF1NE_Msk        (0x1U << FDCAN_IE_RF1NE_Pos)                 /*!< 0x00000010 */
#define FDCAN_IE_RF1NE            FDCAN_IE_RF1NE_Msk                           /*!<Rx FIFO 1 New Message Enable                 */
#define FDCAN_IE_RF1WE_Pos        (5U)
#define FDCAN_IE_RF1WE_Msk        (0x1U << FDCAN_IE_RF1WE_Pos)                 /*!< 0x00000020 */
#define FDCAN_IE_RF1WE            FDCAN_IE_RF1WE_Msk                           /*!<Rx FIFO 1 Watermark Reached Enable           */
#define FDCAN_IE_RF1FE_Pos        (6U)
#define FDCAN_IE_RF1FE_Msk        (0x1U << FDCAN_IE_RF1FE_Pos)                 /*!< 0x00000040 */
#define FDCAN_IE_RF1FE            FDCAN_IE_RF1FE_Msk                           /*!<Rx FIFO 1 Full Enable                        */
#define FDCAN_IE_RF1LE_Pos        (7U)
#define FDCAN_IE_RF1LE_Msk        (0x1U << FDCAN_IE_RF1LE_Pos)                 /*!< 0x00000080 */
#define FDCAN_IE_RF1LE            FDCAN_IE_RF1LE_Msk                           /*!<Rx FIFO 1 Message Lost Enable                */
#define FDCAN_IE_HPME_Pos         (8U)
#define FDCAN_IE_HPME_Msk         (0x1U << FDCAN_IE_HPME_Pos)                  /*!< 0x00000100 */
#define FDCAN_IE_HPME             FDCAN_IE_HPME_Msk                            /*!<High Priority Message Enable                 */
#define FDCAN_IE_TCE_Pos          (9U)
#define FDCAN_IE_TCE_Msk          (0x1U << FDCAN_IE_TCE_Pos)                   /*!< 0x00000200 */
#define FDCAN_IE_TCE              FDCAN_IE_TCE_Msk                             /*!<Transmission Completed Enable                */
#define FDCAN_IE_TCFE_Pos         (10U)
#define FDCAN_IE_TCFE_Msk         (0x1U << FDCAN_IE_TCFE_Pos)                  /*!< 0x00000400 */
#define FDCAN_IE_TCFE             FDCAN_IE_TCFE_Msk                            /*!<Transmission Cancellation Finished Enable    */
#define FDCAN_IE_TFEE_Pos         (11U)
#define FDCAN_IE_TFEE_Msk         (0x1U << FDCAN_IE_TFEE_Pos)                  /*!< 0x00000800 */
#define FDCAN_IE_TFEE             FDCAN_IE_TFEE_Msk                            /*!<Tx FIFO Empty Enable                         */
#define FDCAN_IE_TEFNE_Pos        (12U)
#define FDCAN_IE_TEFNE_Msk        (0x1U << FDCAN_IE_TEFNE_Pos)                 /*!< 0x00001000 */
#define FDCAN_IE_TEFNE            FDCAN_IE_TEFNE_Msk                           /*!<Tx Event FIFO New Entry Enable               */
#define FDCAN_IE_TEFWE_Pos        (13U)
#define FDCAN_IE_TEFWE_Msk        (0x1U << FDCAN_IE_TEFWE_Pos)                 /*!< 0x00002000 */
#define FDCAN_IE_TEFWE            FDCAN_IE_TEFWE_Msk                           /*!<Tx Event FIFO Watermark Reached Enable       */
#define FDCAN_IE_TEFFE_Pos        (14U)
#define FDCAN_IE_TEFFE_Msk        (0x1U << FDCAN_IE_TEFFE_Pos)                 /*!< 0x00004000 */
#define FDCAN_IE_TEFFE            FDCAN_IE_TEFFE_Msk                           /*!<Tx Event FIFO Full Enable                    */
#define FDCAN_IE_TEFLE_Pos        (15U)
#define FDCAN_IE_TEFLE_Msk        (0x1U << FDCAN_IE_TEFLE_Pos)                 /*!< 0x00008000 */
#define FDCAN_IE_TEFLE            FDCAN_IE_TEFLE_Msk                           /*!<Tx Event FIFO Element Lost Enable            */
#define FDCAN_IE_TSWE_Pos         (16U)
#define FDCAN_IE_TSWE_Msk         (0x1U << FDCAN_IE_TSWE_Pos)                  /*!< 0x00010000 */
#define FDCAN_IE_TSWE             FDCAN_IE_TSWE_Msk                            /*!<Timestamp Wraparound Enable                  */
#define FDCAN_IE_MRAFE_Pos        (17U)
#define FDCAN_IE_MRAFE_Msk        (0x1U << FDCAN_IE_MRAFE_Pos)                 /*!< 0x00020000 */
#define FDCAN_IE_MRAFE            FDCAN_IE_MRAFE_Msk                           /*!<Message RAM Access Failure Enable            */
#define FDCAN_IE_TOOE_Pos         (18U)
#define FDCAN_IE_TOOE_Msk         (0x1U << FDCAN_IE_TOOE_Pos)                  /*!< 0x00040000 */
#define FDCAN_IE_TOOE             FDCAN_IE_TOOE_Msk                            /*!<Timeout Occurred Enable                      */
#define FDCAN_IE_DRXE_Pos         (19U)
#define FDCAN_IE_DRXE_Msk         (0x1U << FDCAN_IE_DRXE_Pos)                  /*!< 0x00080000 */
#define FDCAN_IE_DRXE             FDCAN_IE_DRXE_Msk                            /*!<Message stored to Dedicated Rx Buffer Enable */
#define FDCAN_IE_BECE_Pos         (20U)
#define FDCAN_IE_BECE_Msk         (0x1U << FDCAN_IE_BECE_Pos)                  /*!< 0x00100000 */
#define FDCAN_IE_BECE             FDCAN_IE_BECE_Msk                            /*!<Bit Error Corrected Interrupt Enable         */
#define FDCAN_IE_BEUE_Pos         (21U)
#define FDCAN_IE_BEUE_Msk         (0x1U << FDCAN_IE_BEUE_Pos)                  /*!< 0x00200000 */
#define FDCAN_IE_BEUE             FDCAN_IE_BEUE_Msk                            /*!<Bit Error Uncorrected Interrupt Enable       */
#define FDCAN_IE_ELOE_Pos         (22U)
#define FDCAN_IE_ELOE_Msk         (0x1U << FDCAN_IE_ELOE_Pos)                  /*!< 0x00400000 */
#define FDCAN_IE_ELOE             FDCAN_IE_ELOE_Msk                            /*!<Error Logging Overflow Enable                */
#define FDCAN_IE_EPE_Pos          (23U)
#define FDCAN_IE_EPE_Msk          (0x1U << FDCAN_IE_EPE_Pos)                   /*!< 0x00800000 */
#define FDCAN_IE_EPE              FDCAN_IE_EPE_Msk                             /*!<Error Passive Enable                         */
#define FDCAN_IE_EWE_Pos          (24U)
#define FDCAN_IE_EWE_Msk          (0x1U << FDCAN_IE_EWE_Pos)                   /*!< 0x01000000 */
#define FDCAN_IE_EWE              FDCAN_IE_EWE_Msk                             /*!<Warning Status Enable                        */
#define FDCAN_IE_BOE_Pos          (25U)
#define FDCAN_IE_BOE_Msk          (0x1U << FDCAN_IE_BOE_Pos)                   /*!< 0x02000000 */
#define FDCAN_IE_BOE              FDCAN_IE_BOE_Msk                             /*!<Bus_Off Status Enable                        */
#define FDCAN_IE_WDIE_Pos         (26U)
#define FDCAN_IE_WDIE_Msk         (0x1U << FDCAN_IE_WDIE_Pos)                  /*!< 0x04000000 */
#define FDCAN_IE_WDIE             FDCAN_IE_WDIE_Msk                            /*!<Watchdog Interrupt Enable                    */
#define FDCAN_IE_PEAE_Pos         (27U)
#define FDCAN_IE_PEAE_Msk         (0x1U << FDCAN_IE_PEAE_Pos)                  /*!< 0x08000000 */
#define FDCAN_IE_PEAE             FDCAN_IE_PEAE_Msk                            /*!<Protocol Error in Arbitration Phase Enable   */
#define FDCAN_IE_PEDE_Pos         (28U)
#define FDCAN_IE_PEDE_Msk         (0x1U << FDCAN_IE_PEDE_Pos)                  /*!< 0x10000000 */
#define FDCAN_IE_PEDE             FDCAN_IE_PEDE_Msk                            /*!<Protocol Error in Data Phase Enable          */
#define FDCAN_IE_ARAE_Pos         (29U)
#define FDCAN_IE_ARAE_Msk         (0x1U << FDCAN_IE_ARAE_Pos)                  /*!< 0x20000000 */
#define FDCAN_IE_ARAE             FDCAN_IE_ARAE_Msk                            /*!<Access to Reserved Address Enable            */

/*****************  Bit definition for FDCAN_ILS register  **********************/
#define FDCAN_ILS_RF0NL_Pos       (0U)
#define FDCAN_ILS_RF0NL_Msk       (0x1U << FDCAN_ILS_RF0NL_Pos)                /*!< 0x00000001 */
#define FDCAN_ILS_RF0NL           FDCAN_ILS_RF0NL_Msk                          /*!<Rx FIFO 0 New Message Line                  */
#define FDCAN_ILS_RF0WL_Pos       (1U)
#define FDCAN_ILS_RF0WL_Msk       (0x1U << FDCAN_ILS_RF0WL_Pos)                /*!< 0x00000002 */
#define FDCAN_ILS_RF0WL           FDCAN_ILS_RF0WL_Msk                          /*!<Rx FIFO 0 Watermark Reached Line            */
#define FDCAN_ILS_RF0FL_Pos       (2U)
#define FDCAN_ILS_RF0FL_Msk       (0x1U << FDCAN_ILS_RF0FL_Pos)                /*!< 0x00000004 */
#define FDCAN_ILS_RF0FL           FDCAN_ILS_RF0FL_Msk                          /*!<Rx FIFO 0 Full Line                         */
#define FDCAN_ILS_RF0LL_Pos       (3U)
#define FDCAN_ILS_RF0LL_Msk       (0x1U << FDCAN_ILS_RF0LL_Pos)                /*!< 0x00000008 */
#define FDCAN_ILS_RF0LL           FDCAN_ILS_RF0LL_Msk                          /*!<Rx FIFO 0 Message Lost Line                 */
#define FDCAN_ILS_RF1NL_Pos       (4U)
#define FDCAN_ILS_RF1NL_Msk       (0x1U << FDCAN_ILS_RF1NL_Pos)                /*!< 0x00000010 */
#define FDCAN_ILS_RF1NL           FDCAN_ILS_RF1NL_Msk                          /*!<Rx FIFO 1 New Message Line                  */
#define FDCAN_ILS_RF1WL_Pos       (5U)
#define FDCAN_ILS_RF1WL_Msk       (0x1U << FDCAN_ILS_RF1WL_Pos)                /*!< 0x00000020 */
#define FDCAN_ILS_RF1WL           FDCAN_ILS_RF1WL_Msk                          /*!<Rx FIFO 1 Watermark Reached Line            */
#define FDCAN_ILS_RF1FL_Pos       (6U)
#define FDCAN_ILS_RF1FL_Msk       (0x1U << FDCAN_ILS_RF1FL_Pos)                /*!< 0x00000040 */
#define FDCAN_ILS_RF1FL           FDCAN_ILS_RF1FL_Msk                          /*!<Rx FIFO 1 Full Line                         */
#define FDCAN_ILS_RF1LL_Pos       (7U)
#define FDCAN_ILS_RF1LL_Msk       (0x1U << FDCAN_ILS_RF1LL_Pos)                /*!< 0x00000080 */
#define FDCAN_ILS_RF1LL           FDCAN_ILS_RF1LL_Msk                          /*!<Rx FIFO 1 Message Lost Line                 */
#define FDCAN_ILS_HPML_Pos        (8U)
#define FDCAN_ILS_HPML_Msk        (0x1U << FDCAN_ILS_HPML_Pos)                 /*!< 0x00000100 */
#define FDCAN_ILS_HPML            FDCAN_ILS_HPML_Msk                           /*!<High Priority Message Line                  */
#define FDCAN_ILS_TCL_Pos         (9U)
#define FDCAN_ILS_TCL_Msk         (0x1U << FDCAN_ILS_TCL_Pos)                  /*!< 0x00000200 */
#define FDCAN_ILS_TCL             FDCAN_ILS_TCL_Msk                            /*!<Transmission Completed Line                 */
#define FDCAN_ILS_TCFL_Pos        (10U)
#define FDCAN_ILS_TCFL_Msk        (0x1U << FDCAN_ILS_TCFL_Pos)                 /*!< 0x00000400 */
#define FDCAN_ILS_TCFL            FDCAN_ILS_TCFL_Msk                           /*!<Transmission Cancellation Finished Line     */
#define FDCAN_ILS_TFEL_Pos        (11U)
#define FDCAN_ILS_TFEL_Msk        (0x1U << FDCAN_ILS_TFEL_Pos)                 /*!< 0x00000800 */
#define FDCAN_ILS_TFEL            FDCAN_ILS_TFEL_Msk                           /*!<Tx FIFO Empty Line                          */
#define FDCAN_ILS_TEFNL_Pos       (12U)
#define FDCAN_ILS_TEFNL_Msk       (0x1U << FDCAN_ILS_TEFNL_Pos)                /*!< 0x00001000 */
#define FDCAN_ILS_TEFNL           FDCAN_ILS_TEFNL_Msk                          /*!<Tx Event FIFO New Entry Line                */
#define FDCAN_ILS_TEFWL_Pos       (13U)
#define FDCAN_ILS_TEFWL_Msk       (0x1U << FDCAN_ILS_TEFWL_Pos)                /*!< 0x00002000 */
#define FDCAN_ILS_TEFWL           FDCAN_ILS_TEFWL_Msk                          /*!<Tx Event FIFO Watermark Reached Line        */
#define FDCAN_ILS_TEFFL_Pos       (14U)
#define FDCAN_ILS_TEFFL_Msk       (0x1U << FDCAN_ILS_TEFFL_Pos)                /*!< 0x00004000 */
#define FDCAN_ILS_TEFFL           FDCAN_ILS_TEFFL_Msk                          /*!<Tx Event FIFO Full Line                     */
#define FDCAN_ILS_TEFLL_Pos       (15U)
#define FDCAN_ILS_TEFLL_Msk       (0x1U << FDCAN_ILS_TEFLL_Pos)                /*!< 0x00008000 */
#define FDCAN_ILS_TEFLL           FDCAN_ILS_TEFLL_Msk                          /*!<Tx Event FIFO Element Lost Line             */
#define FDCAN_ILS_TSWL_Pos        (16U)
#define FDCAN_ILS_TSWL_Msk        (0x1U << FDCAN_ILS_TSWL_Pos)                 /*!< 0x00010000 */
#define FDCAN_ILS_TSWL            FDCAN_ILS_TSWL_Msk                           /*!<Timestamp Wraparound Line                   */
#define FDCAN_ILS_MRAFE_Pos       (17U)
#define FDCAN_ILS_MRAFE_Msk       (0x1U << FDCAN_ILS_MRAFE_Pos)                /*!< 0x00020000 */
#define FDCAN_ILS_MRAFE           FDCAN_ILS_MRAFE_Msk                          /*!<Message RAM Access Failure Line             */
#define FDCAN_ILS_TOOE_Pos        (18U)
#define FDCAN_ILS_TOOE_Msk        (0x1U << FDCAN_ILS_TOOE_Pos)                 /*!< 0x00040000 */
#define FDCAN_ILS_TOOE            FDCAN_ILS_TOOE_Msk                           /*!<Timeout Occurred Line                       */
#define FDCAN_ILS_DRXE_Pos        (19U)
#define FDCAN_ILS_DRXE_Msk        (0x1U << FDCAN_ILS_DRXE_Pos)                 /*!< 0x00080000 */
#define FDCAN_ILS_DRXE            FDCAN_ILS_DRXE_Msk                           /*!<Message stored to Dedicated Rx Buffer Line  */
#define FDCAN_ILS_BECE_Pos        (20U)
#define FDCAN_ILS_BECE_Msk        (0x1U << FDCAN_ILS_BECE_Pos)                 /*!< 0x00100000 */
#define FDCAN_ILS_BECE            FDCAN_ILS_BECE_Msk                           /*!<Bit Error Corrected Interrupt Line          */
#define FDCAN_ILS_BEUE_Pos        (21U)
#define FDCAN_ILS_BEUE_Msk        (0x1U << FDCAN_ILS_BEUE_Pos)                 /*!< 0x00200000 */
#define FDCAN_ILS_BEUE            FDCAN_ILS_BEUE_Msk                           /*!<Bit Error Uncorrected Interrupt Line        */
#define FDCAN_ILS_ELOE_Pos        (22U)
#define FDCAN_ILS_ELOE_Msk        (0x1U << FDCAN_ILS_ELOE_Pos)                 /*!< 0x00400000 */
#define FDCAN_ILS_ELOE            FDCAN_ILS_ELOE_Msk                           /*!<Error Logging Overflow Line                 */
#define FDCAN_ILS_EPE_Pos         (23U)
#define FDCAN_ILS_EPE_Msk         (0x1U << FDCAN_ILS_EPE_Pos)                  /*!< 0x00800000 */
#define FDCAN_ILS_EPE             FDCAN_ILS_EPE_Msk                            /*!<Error Passive Line                          */
#define FDCAN_ILS_EWE_Pos         (24U)
#define FDCAN_ILS_EWE_Msk         (0x1U << FDCAN_ILS_EWE_Pos)                  /*!< 0x01000000 */
#define FDCAN_ILS_EWE             FDCAN_ILS_EWE_Msk                            /*!<Warning Status Line                         */
#define FDCAN_ILS_BOE_Pos         (25U)
#define FDCAN_ILS_BOE_Msk         (0x1U << FDCAN_ILS_BOE_Pos)                  /*!< 0x02000000 */
#define FDCAN_ILS_BOE             FDCAN_ILS_BOE_Msk                            /*!<Bus_Off Status Line                         */
#define FDCAN_ILS_WDIE_Pos        (26U)
#define FDCAN_ILS_WDIE_Msk        (0x1U << FDCAN_ILS_WDIE_Pos)                 /*!< 0x04000000 */
#define FDCAN_ILS_WDIE            FDCAN_ILS_WDIE_Msk                           /*!<Watchdog Interrupt Line                     */
#define FDCAN_ILS_PEAE_Pos        (27U)
#define FDCAN_ILS_PEAE_Msk        (0x1U << FDCAN_ILS_PEAE_Pos)                 /*!< 0x08000000 */
#define FDCAN_ILS_PEAE            FDCAN_ILS_PEAE_Msk                           /*!<Protocol Error in Arbitration Phase Line    */
#define FDCAN_ILS_PEDE_Pos        (28U)
#define FDCAN_ILS_PEDE_Msk        (0x1U << FDCAN_ILS_PEDE_Pos)                 /*!< 0x10000000 */
#define FDCAN_ILS_PEDE            FDCAN_ILS_PEDE_Msk                           /*!<Protocol Error in Data Phase Line           */
#define FDCAN_ILS_ARAE_Pos        (29U)
#define FDCAN_ILS_ARAE_Msk        (0x1U << FDCAN_ILS_ARAE_Pos)                 /*!< 0x20000000 */
#define FDCAN_ILS_ARAE            FDCAN_ILS_ARAE_Msk                           /*!<Access to Reserved Address Line             */

/*****************  Bit definition for FDCAN_ILE register  **********************/
#define FDCAN_ILE_EINT0_Pos       (0U)
#define FDCAN_ILE_EINT0_Msk       (0x1U << FDCAN_ILE_EINT0_Pos)                /*!< 0x00000001 */
#define FDCAN_ILE_EINT0           FDCAN_ILE_EINT0_Msk                          /*!<Enable Interrupt Line 0                   */
#define FDCAN_ILE_EINT1_Pos       (1U)
#define FDCAN_ILE_EINT1_Msk       (0x1U << FDCAN_ILE_EINT1_Pos)                /*!< 0x00000002 */
#define FDCAN_ILE_EINT1           FDCAN_ILE_EINT1_Msk                          /*!<Enable Interrupt Line 1                   */

/*****************  Bit definition for FDCAN_GFC register  **********************/
#define FDCAN_GFC_RRFE_Pos        (0U)
#define FDCAN_GFC_RRFE_Msk        (0x1U << FDCAN_GFC_RRFE_Pos)                 /*!< 0x00000001 */
#define FDCAN_GFC_RRFE            FDCAN_GFC_RRFE_Msk                           /*!<Reject Remote Frames Extended             */
#define FDCAN_GFC_RRFS_Pos        (1U)
#define FDCAN_GFC_RRFS_Msk        (0x1U << FDCAN_GFC_RRFS_Pos)                 /*!< 0x00000002 */
#define FDCAN_GFC_RRFS            FDCAN_GFC_RRFS_Msk                           /*!<Reject Remote Frames Standard             */
#define FDCAN_GFC_ANFE_Pos        (2U)
#define FDCAN_GFC_ANFE_Msk        (0x3U << FDCAN_GFC_ANFE_Pos)                 /*!< 0x0000000C */
#define FDCAN_GFC_ANFE            FDCAN_GFC_ANFE_Msk                           /*!<Accept Non-matching Frames Extended       */
#define FDCAN_GFC_ANFS_Pos        (4U)
#define FDCAN_GFC_ANFS_Msk        (0x3U << FDCAN_GFC_ANFS_Pos)                 /*!< 0x00000030 */
#define FDCAN_GFC_ANFS            FDCAN_GFC_ANFS_Msk                           /*!<Accept Non-matching Frames Standard       */

/*****************  Bit definition for FDCAN_SIDFC register  ********************/
#define FDCAN_SIDFC_FLSSA_Pos     (2U)
#define FDCAN_SIDFC_FLSSA_Msk     (0x3FFFU << FDCAN_SIDFC_FLSSA_Pos)           /*!< 0x0000FFFC */
#define FDCAN_SIDFC_FLSSA         FDCAN_SIDFC_FLSSA_Msk                        /*!<Filter List Standard Start Address        */
#define FDCAN_SIDFC_LSS_Pos       (16U)
#define FDCAN_SIDFC_LSS_Msk       (0xFFU << FDCAN_SIDFC_LSS_Pos)               /*!< 0x00FF0000 */
#define FDCAN_SIDFC_LSS           FDCAN_SIDFC_LSS_Msk                          /*!<List Size Standard                        */

/*****************  Bit definition for FDCAN_XIDFC register  ********************/
#define FDCAN_XIDFC_FLESA_Pos     (2U)
#define FDCAN_XIDFC_FLESA_Msk     (0x3FFFU << FDCAN_XIDFC_FLESA_Pos)           /*!< 0x0000FFFC */
#define FDCAN_XIDFC_FLESA         FDCAN_XIDFC_FLESA_Msk                        /*!<Filter List Standard Start Address        */
#define FDCAN_XIDFC_LSE_Pos       (16U)
#define FDCAN_XIDFC_LSE_Msk       (0xFFU << FDCAN_XIDFC_LSE_Pos)               /*!< 0x00FF0000 */
#define FDCAN_XIDFC_LSE           FDCAN_XIDFC_LSE_Msk                          /*!<List Size Extended                        */

/*****************  Bit definition for FDCAN_XIDAM register  ********************/
#define FDCAN_XIDAM_EIDM_Pos      (0U)
#define FDCAN_XIDAM_EIDM_Msk      (0x1FFFFFFFU << FDCAN_XIDAM_EIDM_Pos)        /*!< 0x1FFFFFFF */
#define FDCAN_XIDAM_EIDM          FDCAN_XIDAM_EIDM_Msk                         /*!<Extended ID Mask                          */

/*****************  Bit definition for FDCAN_HPMS register  *********************/
#define FDCAN_HPMS_BIDX_Pos       (0U)
#define FDCAN_HPMS_BIDX_Msk       (0x3FU << FDCAN_HPMS_BIDX_Pos)               /*!< 0x0000003F */
#define FDCAN_HPMS_BIDX           FDCAN_HPMS_BIDX_Msk                          /*!<Buffer Index                              */
#define FDCAN_HPMS_MSI_Pos        (6U)
#define FDCAN_HPMS_MSI_Msk        (0x3U << FDCAN_HPMS_MSI_Pos)                 /*!< 0x000000C0 */
#define FDCAN_HPMS_MSI            FDCAN_HPMS_MSI_Msk                           /*!<Message Storage Indicator                 */
#define FDCAN_HPMS_FIDX_Pos       (8U)
#define FDCAN_HPMS_FIDX_Msk       (0x7FU << FDCAN_HPMS_FIDX_Pos)               /*!< 0x00007F00 */
#define FDCAN_HPMS_FIDX           FDCAN_HPMS_FIDX_Msk                          /*!<Filter Index                              */
#define FDCAN_HPMS_FLST_Pos       (15U)
#define FDCAN_HPMS_FLST_Msk       (0x1U << FDCAN_HPMS_FLST_Pos)                /*!< 0x00008000 */
#define FDCAN_HPMS_FLST           FDCAN_HPMS_FLST_Msk                          /*!<Filter List                               */

/*****************  Bit definition for FDCAN_NDAT1 register  ********************/
#define FDCAN_NDAT1_ND0_Pos       (0U)
#define FDCAN_NDAT1_ND0_Msk       (0x1U << FDCAN_NDAT1_ND0_Pos)                /*!< 0x00000001 */
#define FDCAN_NDAT1_ND0           FDCAN_NDAT1_ND0_Msk                          /*!<New Data flag of Rx Buffer 0              */
#define FDCAN_NDAT1_ND1_Pos       (1U)
#define FDCAN_NDAT1_ND1_Msk       (0x1U << FDCAN_NDAT1_ND1_Pos)                /*!< 0x00000002 */
#define FDCAN_NDAT1_ND1           FDCAN_NDAT1_ND1_Msk                          /*!<New Data flag of Rx Buffer 1              */
#define FDCAN_NDAT1_ND2_Pos       (2U)
#define FDCAN_NDAT1_ND2_Msk       (0x1U << FDCAN_NDAT1_ND2_Pos)                /*!< 0x00000004 */
#define FDCAN_NDAT1_ND2           FDCAN_NDAT1_ND2_Msk                          /*!<New Data flag of Rx Buffer 2              */
#define FDCAN_NDAT1_ND3_Pos       (3U)
#define FDCAN_NDAT1_ND3_Msk       (0x1U << FDCAN_NDAT1_ND3_Pos)                /*!< 0x00000008 */
#define FDCAN_NDAT1_ND3           FDCAN_NDAT1_ND3_Msk                          /*!<New Data flag of Rx Buffer 3              */
#define FDCAN_NDAT1_ND4_Pos       (4U)
#define FDCAN_NDAT1_ND4_Msk       (0x1U << FDCAN_NDAT1_ND4_Pos)                /*!< 0x00000010 */
#define FDCAN_NDAT1_ND4           FDCAN_NDAT1_ND4_Msk                          /*!<New Data flag of Rx Buffer 4              */
#define FDCAN_NDAT1_ND5_Pos       (5U)
#define FDCAN_NDAT1_ND5_Msk       (0x1U << FDCAN_NDAT1_ND5_Pos)                /*!< 0x00000020 */
#define FDCAN_NDAT1_ND5           FDCAN_NDAT1_ND5_Msk                          /*!<New Data flag of Rx Buffer 5              */
#define FDCAN_NDAT1_ND6_Pos       (6U)
#define FDCAN_NDAT1_ND6_Msk       (0x1U << FDCAN_NDAT1_ND6_Pos)                /*!< 0x00000040 */
#define FDCAN_NDAT1_ND6           FDCAN_NDAT1_ND6_Msk                          /*!<New Data flag of Rx Buffer 6              */
#define FDCAN_NDAT1_ND7_Pos       (7U)
#define FDCAN_NDAT1_ND7_Msk       (0x1U << FDCAN_NDAT1_ND7_Pos)                /*!< 0x00000080 */
#define FDCAN_NDAT1_ND7           FDCAN_NDAT1_ND7_Msk                          /*!<New Data flag of Rx Buffer 7              */
#define FDCAN_NDAT1_ND8_Pos       (8U)
#define FDCAN_NDAT1_ND8_Msk       (0x1U << FDCAN_NDAT1_ND8_Pos)                /*!< 0x00000100 */
#define FDCAN_NDAT1_ND8           FDCAN_NDAT1_ND8_Msk                          /*!<New Data flag of Rx Buffer 8              */
#define FDCAN_NDAT1_ND9_Pos       (9U)
#define FDCAN_NDAT1_ND9_Msk       (0x1U << FDCAN_NDAT1_ND9_Pos)                /*!< 0x00000200 */
#define FDCAN_NDAT1_ND9           FDCAN_NDAT1_ND9_Msk                          /*!<New Data flag of Rx Buffer 9              */
#define FDCAN_NDAT1_ND10_Pos      (10U)
#define FDCAN_NDAT1_ND10_Msk      (0x1U << FDCAN_NDAT1_ND10_Pos)               /*!< 0x00000400 */
#define FDCAN_NDAT1_ND10          FDCAN_NDAT1_ND10_Msk                         /*!<New Data flag of Rx Buffer 10             */
#define FDCAN_NDAT1_ND11_Pos      (11U)
#define FDCAN_NDAT1_ND11_Msk      (0x1U << FDCAN_NDAT1_ND11_Pos)               /*!< 0x00000800 */
#define FDCAN_NDAT1_ND11          FDCAN_NDAT1_ND11_Msk                         /*!<New Data flag of Rx Buffer 11             */
#define FDCAN_NDAT1_ND12_Pos      (12U)
#define FDCAN_NDAT1_ND12_Msk      (0x1U << FDCAN_NDAT1_ND12_Pos)               /*!< 0x00001000 */
#define FDCAN_NDAT1_ND12          FDCAN_NDAT1_ND12_Msk                         /*!<New Data flag of Rx Buffer 12             */
#define FDCAN_NDAT1_ND13_Pos      (13U)
#define FDCAN_NDAT1_ND13_Msk      (0x1U << FDCAN_NDAT1_ND13_Pos)               /*!< 0x00002000 */
#define FDCAN_NDAT1_ND13          FDCAN_NDAT1_ND13_Msk                         /*!<New Data flag of Rx Buffer 13             */
#define FDCAN_NDAT1_ND14_Pos      (14U)
#define FDCAN_NDAT1_ND14_Msk      (0x1U << FDCAN_NDAT1_ND14_Pos)               /*!< 0x00004000 */
#define FDCAN_NDAT1_ND14          FDCAN_NDAT1_ND14_Msk                         /*!<New Data flag of Rx Buffer 14             */
#define FDCAN_NDAT1_ND15_Pos      (15U)
#define FDCAN_NDAT1_ND15_Msk      (0x1U << FDCAN_NDAT1_ND15_Pos)               /*!< 0x00008000 */
#define FDCAN_NDAT1_ND15          FDCAN_NDAT1_ND15_Msk                         /*!<New Data flag of Rx Buffer 15             */
#define FDCAN_NDAT1_ND16_Pos      (16U)
#define FDCAN_NDAT1_ND16_Msk      (0x1U << FDCAN_NDAT1_ND16_Pos)               /*!< 0x00010000 */
#define FDCAN_NDAT1_ND16          FDCAN_NDAT1_ND16_Msk                         /*!<New Data flag of Rx Buffer 16             */
#define FDCAN_NDAT1_ND17_Pos      (17U)
#define FDCAN_NDAT1_ND17_Msk      (0x1U << FDCAN_NDAT1_ND17_Pos)               /*!< 0x00020000 */
#define FDCAN_NDAT1_ND17          FDCAN_NDAT1_ND17_Msk                         /*!<New Data flag of Rx Buffer 17             */
#define FDCAN_NDAT1_ND18_Pos      (18U)
#define FDCAN_NDAT1_ND18_Msk      (0x1U << FDCAN_NDAT1_ND18_Pos)               /*!< 0x00040000 */
#define FDCAN_NDAT1_ND18          FDCAN_NDAT1_ND18_Msk                         /*!<New Data flag of Rx Buffer 18             */
#define FDCAN_NDAT1_ND19_Pos      (19U)
#define FDCAN_NDAT1_ND19_Msk      (0x1U << FDCAN_NDAT1_ND19_Pos)               /*!< 0x00080000 */
#define FDCAN_NDAT1_ND19          FDCAN_NDAT1_ND19_Msk                         /*!<New Data flag of Rx Buffer 19             */
#define FDCAN_NDAT1_ND20_Pos      (20U)
#define FDCAN_NDAT1_ND20_Msk      (0x1U << FDCAN_NDAT1_ND20_Pos)               /*!< 0x00100000 */
#define FDCAN_NDAT1_ND20          FDCAN_NDAT1_ND20_Msk                         /*!<New Data flag of Rx Buffer 20             */
#define FDCAN_NDAT1_ND21_Pos      (21U)
#define FDCAN_NDAT1_ND21_Msk      (0x1U << FDCAN_NDAT1_ND21_Pos)               /*!< 0x00200000 */
#define FDCAN_NDAT1_ND21          FDCAN_NDAT1_ND21_Msk                         /*!<New Data flag of Rx Buffer 21             */
#define FDCAN_NDAT1_ND22_Pos      (22U)
#define FDCAN_NDAT1_ND22_Msk      (0x1U << FDCAN_NDAT1_ND22_Pos)               /*!< 0x00400000 */
#define FDCAN_NDAT1_ND22          FDCAN_NDAT1_ND22_Msk                         /*!<New Data flag of Rx Buffer 22             */
#define FDCAN_NDAT1_ND23_Pos      (23U)
#define FDCAN_NDAT1_ND23_Msk      (0x1U << FDCAN_NDAT1_ND23_Pos)               /*!< 0x00800000 */
#define FDCAN_NDAT1_ND23          FDCAN_NDAT1_ND23_Msk                         /*!<New Data flag of Rx Buffer 23             */
#define FDCAN_NDAT1_ND24_Pos      (24U)
#define FDCAN_NDAT1_ND24_Msk      (0x1U << FDCAN_NDAT1_ND24_Pos)               /*!< 0x01000000 */
#define FDCAN_NDAT1_ND24          FDCAN_NDAT1_ND24_Msk                         /*!<New Data flag of Rx Buffer 24             */
#define FDCAN_NDAT1_ND25_Pos      (25U)
#define FDCAN_NDAT1_ND25_Msk      (0x1U << FDCAN_NDAT1_ND25_Pos)               /*!< 0x02000000 */
#define FDCAN_NDAT1_ND25          FDCAN_NDAT1_ND25_Msk                         /*!<New Data flag of Rx Buffer 25             */
#define FDCAN_NDAT1_ND26_Pos      (26U)
#define FDCAN_NDAT1_ND26_Msk      (0x1U << FDCAN_NDAT1_ND26_Pos)               /*!< 0x04000000 */
#define FDCAN_NDAT1_ND26          FDCAN_NDAT1_ND26_Msk                         /*!<New Data flag of Rx Buffer 26             */
#define FDCAN_NDAT1_ND27_Pos      (27U)
#define FDCAN_NDAT1_ND27_Msk      (0x1U << FDCAN_NDAT1_ND27_Pos)               /*!< 0x08000000 */
#define FDCAN_NDAT1_ND27          FDCAN_NDAT1_ND27_Msk                         /*!<New Data flag of Rx Buffer 27             */
#define FDCAN_NDAT1_ND28_Pos      (28U)
#define FDCAN_NDAT1_ND28_Msk      (0x1U << FDCAN_NDAT1_ND28_Pos)               /*!< 0x10000000 */
#define FDCAN_NDAT1_ND28          FDCAN_NDAT1_ND28_Msk                         /*!<New Data flag of Rx Buffer 28             */
#define FDCAN_NDAT1_ND29_Pos      (29U)
#define FDCAN_NDAT1_ND29_Msk      (0x1U << FDCAN_NDAT1_ND29_Pos)               /*!< 0x20000000 */
#define FDCAN_NDAT1_ND29          FDCAN_NDAT1_ND29_Msk                         /*!<New Data flag of Rx Buffer 29             */
#define FDCAN_NDAT1_ND30_Pos      (30U)
#define FDCAN_NDAT1_ND30_Msk      (0x1U << FDCAN_NDAT1_ND30_Pos)               /*!< 0x40000000 */
#define FDCAN_NDAT1_ND30          FDCAN_NDAT1_ND30_Msk                         /*!<New Data flag of Rx Buffer 30             */
#define FDCAN_NDAT1_ND31_Pos      (31U)
#define FDCAN_NDAT1_ND31_Msk      (0x1U << FDCAN_NDAT1_ND31_Pos)               /*!< 0x80000000 */
#define FDCAN_NDAT1_ND31          FDCAN_NDAT1_ND31_Msk                         /*!<New Data flag of Rx Buffer 31             */

/*****************  Bit definition for FDCAN_NDAT2 register  ********************/
#define FDCAN_NDAT2_ND32_Pos      (0U)
#define FDCAN_NDAT2_ND32_Msk      (0x1U << FDCAN_NDAT2_ND32_Pos)               /*!< 0x00000001 */
#define FDCAN_NDAT2_ND32          FDCAN_NDAT2_ND32_Msk                         /*!<New Data flag of Rx Buffer 32             */
#define FDCAN_NDAT2_ND33_Pos      (1U)
#define FDCAN_NDAT2_ND33_Msk      (0x1U << FDCAN_NDAT2_ND33_Pos)               /*!< 0x00000002 */
#define FDCAN_NDAT2_ND33          FDCAN_NDAT2_ND33_Msk                         /*!<New Data flag of Rx Buffer 33             */
#define FDCAN_NDAT2_ND34_Pos      (2U)
#define FDCAN_NDAT2_ND34_Msk      (0x1U << FDCAN_NDAT2_ND34_Pos)               /*!< 0x00000004 */
#define FDCAN_NDAT2_ND34          FDCAN_NDAT2_ND34_Msk                         /*!<New Data flag of Rx Buffer 34             */
#define FDCAN_NDAT2_ND35_Pos      (3U)
#define FDCAN_NDAT2_ND35_Msk      (0x1U << FDCAN_NDAT2_ND35_Pos)               /*!< 0x00000008 */
#define FDCAN_NDAT2_ND35          FDCAN_NDAT2_ND35_Msk                         /*!<New Data flag of Rx Buffer 35             */
#define FDCAN_NDAT2_ND36_Pos      (4U)
#define FDCAN_NDAT2_ND36_Msk      (0x1U << FDCAN_NDAT2_ND36_Pos)               /*!< 0x00000010 */
#define FDCAN_NDAT2_ND36          FDCAN_NDAT2_ND36_Msk                         /*!<New Data flag of Rx Buffer 36             */
#define FDCAN_NDAT2_ND37_Pos      (5U)
#define FDCAN_NDAT2_ND37_Msk      (0x1U << FDCAN_NDAT2_ND37_Pos)               /*!< 0x00000020 */
#define FDCAN_NDAT2_ND37          FDCAN_NDAT2_ND37_Msk                         /*!<New Data flag of Rx Buffer 37             */
#define FDCAN_NDAT2_ND38_Pos      (6U)
#define FDCAN_NDAT2_ND38_Msk      (0x1U << FDCAN_NDAT2_ND38_Pos)               /*!< 0x00000040 */
#define FDCAN_NDAT2_ND38          FDCAN_NDAT2_ND38_Msk                         /*!<New Data flag of Rx Buffer 38             */
#define FDCAN_NDAT2_ND39_Pos      (7U)
#define FDCAN_NDAT2_ND39_Msk      (0x1U << FDCAN_NDAT2_ND39_Pos)               /*!< 0x00000080 */
#define FDCAN_NDAT2_ND39          FDCAN_NDAT2_ND39_Msk                         /*!<New Data flag of Rx Buffer 39             */
#define FDCAN_NDAT2_ND40_Pos      (8U)
#define FDCAN_NDAT2_ND40_Msk      (0x1U << FDCAN_NDAT2_ND40_Pos)               /*!< 0x00000100 */
#define FDCAN_NDAT2_ND40          FDCAN_NDAT2_ND40_Msk                         /*!<New Data flag of Rx Buffer 40             */
#define FDCAN_NDAT2_ND41_Pos      (9U)
#define FDCAN_NDAT2_ND41_Msk      (0x1U << FDCAN_NDAT2_ND41_Pos)               /*!< 0x00000200 */
#define FDCAN_NDAT2_ND41          FDCAN_NDAT2_ND41_Msk                         /*!<New Data flag of Rx Buffer 41             */
#define FDCAN_NDAT2_ND42_Pos      (10U)
#define FDCAN_NDAT2_ND42_Msk      (0x1U << FDCAN_NDAT2_ND42_Pos)               /*!< 0x00000400 */
#define FDCAN_NDAT2_ND42          FDCAN_NDAT2_ND42_Msk                         /*!<New Data flag of Rx Buffer 42             */
#define FDCAN_NDAT2_ND43_Pos      (11U)
#define FDCAN_NDAT2_ND43_Msk      (0x1U << FDCAN_NDAT2_ND43_Pos)               /*!< 0x00000800 */
#define FDCAN_NDAT2_ND43          FDCAN_NDAT2_ND43_Msk                         /*!<New Data flag of Rx Buffer 43             */
#define FDCAN_NDAT2_ND44_Pos      (12U)
#define FDCAN_NDAT2_ND44_Msk      (0x1U << FDCAN_NDAT2_ND44_Pos)               /*!< 0x00001000 */
#define FDCAN_NDAT2_ND44          FDCAN_NDAT2_ND44_Msk                         /*!<New Data flag of Rx Buffer 44             */
#define FDCAN_NDAT2_ND45_Pos      (13U)
#define FDCAN_NDAT2_ND45_Msk      (0x1U << FDCAN_NDAT2_ND45_Pos)               /*!< 0x00002000 */
#define FDCAN_NDAT2_ND45          FDCAN_NDAT2_ND45_Msk                         /*!<New Data flag of Rx Buffer 45             */
#define FDCAN_NDAT2_ND46_Pos      (14U)
#define FDCAN_NDAT2_ND46_Msk      (0x1U << FDCAN_NDAT2_ND46_Pos)               /*!< 0x00004000 */
#define FDCAN_NDAT2_ND46          FDCAN_NDAT2_ND46_Msk                         /*!<New Data flag of Rx Buffer 46             */
#define FDCAN_NDAT2_ND47_Pos      (15U)
#define FDCAN_NDAT2_ND47_Msk      (0x1U << FDCAN_NDAT2_ND47_Pos)               /*!< 0x00008000 */
#define FDCAN_NDAT2_ND47          FDCAN_NDAT2_ND47_Msk                         /*!<New Data flag of Rx Buffer 47             */
#define FDCAN_NDAT2_ND48_Pos      (16U)
#define FDCAN_NDAT2_ND48_Msk      (0x1U << FDCAN_NDAT2_ND48_Pos)               /*!< 0x00010000 */
#define FDCAN_NDAT2_ND48          FDCAN_NDAT2_ND48_Msk                         /*!<New Data flag of Rx Buffer 48             */
#define FDCAN_NDAT2_ND49_Pos      (17U)
#define FDCAN_NDAT2_ND49_Msk      (0x1U << FDCAN_NDAT2_ND49_Pos)               /*!< 0x00020000 */
#define FDCAN_NDAT2_ND49          FDCAN_NDAT2_ND49_Msk                         /*!<New Data flag of Rx Buffer 49             */
#define FDCAN_NDAT2_ND50_Pos      (18U)
#define FDCAN_NDAT2_ND50_Msk      (0x1U << FDCAN_NDAT2_ND50_Pos)               /*!< 0x00040000 */
#define FDCAN_NDAT2_ND50          FDCAN_NDAT2_ND50_Msk                         /*!<New Data flag of Rx Buffer 50             */
#define FDCAN_NDAT2_ND51_Pos      (19U)
#define FDCAN_NDAT2_ND51_Msk      (0x1U << FDCAN_NDAT2_ND51_Pos)               /*!< 0x00080000 */
#define FDCAN_NDAT2_ND51          FDCAN_NDAT2_ND51_Msk                         /*!<New Data flag of Rx Buffer 51             */
#define FDCAN_NDAT2_ND52_Pos      (20U)
#define FDCAN_NDAT2_ND52_Msk      (0x1U << FDCAN_NDAT2_ND52_Pos)               /*!< 0x00100000 */
#define FDCAN_NDAT2_ND52          FDCAN_NDAT2_ND52_Msk                         /*!<New Data flag of Rx Buffer 52             */
#define FDCAN_NDAT2_ND53_Pos      (21U)
#define FDCAN_NDAT2_ND53_Msk      (0x1U << FDCAN_NDAT2_ND53_Pos)               /*!< 0x00200000 */
#define FDCAN_NDAT2_ND53          FDCAN_NDAT2_ND53_Msk                         /*!<New Data flag of Rx Buffer 53             */
#define FDCAN_NDAT2_ND54_Pos      (22U)
#define FDCAN_NDAT2_ND54_Msk      (0x1U << FDCAN_NDAT2_ND54_Pos)               /*!< 0x00400000 */
#define FDCAN_NDAT2_ND54          FDCAN_NDAT2_ND54_Msk                         /*!<New Data flag of Rx Buffer 54             */
#define FDCAN_NDAT2_ND55_Pos      (23U)
#define FDCAN_NDAT2_ND55_Msk      (0x1U << FDCAN_NDAT2_ND55_Pos)               /*!< 0x00800000 */
#define FDCAN_NDAT2_ND55          FDCAN_NDAT2_ND55_Msk                         /*!<New Data flag of Rx Buffer 55             */
#define FDCAN_NDAT2_ND56_Pos      (24U)
#define FDCAN_NDAT2_ND56_Msk      (0x1U << FDCAN_NDAT2_ND56_Pos)               /*!< 0x01000000 */
#define FDCAN_NDAT2_ND56          FDCAN_NDAT2_ND56_Msk                         /*!<New Data flag of Rx Buffer 56             */
#define FDCAN_NDAT2_ND57_Pos      (25U)
#define FDCAN_NDAT2_ND57_Msk      (0x1U << FDCAN_NDAT2_ND57_Pos)               /*!< 0x02000000 */
#define FDCAN_NDAT2_ND57          FDCAN_NDAT2_ND57_Msk                         /*!<New Data flag of Rx Buffer 57             */
#define FDCAN_NDAT2_ND58_Pos      (26U)
#define FDCAN_NDAT2_ND58_Msk      (0x1U << FDCAN_NDAT2_ND58_Pos)               /*!< 0x04000000 */
#define FDCAN_NDAT2_ND58          FDCAN_NDAT2_ND58_Msk                         /*!<New Data flag of Rx Buffer 58             */
#define FDCAN_NDAT2_ND59_Pos      (27U)
#define FDCAN_NDAT2_ND59_Msk      (0x1U << FDCAN_NDAT2_ND59_Pos)               /*!< 0x08000000 */
#define FDCAN_NDAT2_ND59          FDCAN_NDAT2_ND59_Msk                         /*!<New Data flag of Rx Buffer 59             */
#define FDCAN_NDAT2_ND60_Pos      (28U)
#define FDCAN_NDAT2_ND60_Msk      (0x1U << FDCAN_NDAT2_ND60_Pos)               /*!< 0x10000000 */
#define FDCAN_NDAT2_ND60          FDCAN_NDAT2_ND60_Msk                         /*!<New Data flag of Rx Buffer 60             */
#define FDCAN_NDAT2_ND61_Pos      (29U)
#define FDCAN_NDAT2_ND61_Msk      (0x1U << FDCAN_NDAT2_ND61_Pos)               /*!< 0x20000000 */
#define FDCAN_NDAT2_ND61          FDCAN_NDAT2_ND61_Msk                         /*!<New Data flag of Rx Buffer 61             */
#define FDCAN_NDAT2_ND62_Pos      (30U)
#define FDCAN_NDAT2_ND62_Msk      (0x1U << FDCAN_NDAT2_ND62_Pos)               /*!< 0x40000000 */
#define FDCAN_NDAT2_ND62          FDCAN_NDAT2_ND62_Msk                         /*!<New Data flag of Rx Buffer 62             */
#define FDCAN_NDAT2_ND63_Pos      (31U)
#define FDCAN_NDAT2_ND63_Msk      (0x1U << FDCAN_NDAT2_ND63_Pos)               /*!< 0x80000000 */
#define FDCAN_NDAT2_ND63          FDCAN_NDAT2_ND63_Msk                         /*!<New Data flag of Rx Buffer 63             */

/*****************  Bit definition for FDCAN_RXF0C register  ********************/
#define FDCAN_RXF0C_F0SA_Pos      (2U)
#define FDCAN_RXF0C_F0SA_Msk      (0x3FFFU << FDCAN_RXF0C_F0SA_Pos)            /*!< 0x0000FFFC */
#define FDCAN_RXF0C_F0SA          FDCAN_RXF0C_F0SA_Msk                         /*!<Rx FIFO 0 Start Address                   */
#define FDCAN_RXF0C_F0S_Pos       (16U)
#define FDCAN_RXF0C_F0S_Msk       (0x7FU << FDCAN_RXF0C_F0S_Pos)               /*!< 0x007F0000 */
#define FDCAN_RXF0C_F0S           FDCAN_RXF0C_F0S_Msk                          /*!<Number of Rx FIFO 0 elements              */
#define FDCAN_RXF0C_F0WM_Pos      (24U)
#define FDCAN_RXF0C_F0WM_Msk      (0x7FU << FDCAN_RXF0C_F0WM_Pos)              /*!< 0x7F000000 */
#define FDCAN_RXF0C_F0WM          FDCAN_RXF0C_F0WM_Msk                         /*!<FIFO 0 Watermark                          */
#define FDCAN_RXF0C_F0OM_Pos      (31U)
#define FDCAN_RXF0C_F0OM_Msk      (0x1U << FDCAN_RXF0C_F0OM_Pos)               /*!< 0x80000000 */
#define FDCAN_RXF0C_F0OM          FDCAN_RXF0C_F0OM_Msk                         /*!<FIFO 0 Operation Mode                     */

/*****************  Bit definition for FDCAN_RXF0S register  ********************/
#define FDCAN_RXF0S_F0FL_Pos      (0U)
#define FDCAN_RXF0S_F0FL_Msk      (0x7FU << FDCAN_RXF0S_F0FL_Pos)              /*!< 0x0000007F */
#define FDCAN_RXF0S_F0FL          FDCAN_RXF0S_F0FL_Msk                         /*!<Rx FIFO 0 Fill Level                      */
#define FDCAN_RXF0S_F0GI_Pos      (8U)
#define FDCAN_RXF0S_F0GI_Msk      (0x3FU << FDCAN_RXF0S_F0GI_Pos)              /*!< 0x00003F00 */
#define FDCAN_RXF0S_F0GI          FDCAN_RXF0S_F0GI_Msk                         /*!<Rx FIFO 0 Get Index                       */
#define FDCAN_RXF0S_F0PI_Pos      (16U)
#define FDCAN_RXF0S_F0PI_Msk      (0x3FU << FDCAN_RXF0S_F0PI_Pos)              /*!< 0x003F0000 */
#define FDCAN_RXF0S_F0PI          FDCAN_RXF0S_F0PI_Msk                         /*!<Rx FIFO 0 Put Index                       */
#define FDCAN_RXF0S_F0F_Pos       (24U)
#define FDCAN_RXF0S_F0F_Msk       (0x1U << FDCAN_RXF0S_F0F_Pos)                /*!< 0x01000000 */
#define FDCAN_RXF0S_F0F           FDCAN_RXF0S_F0F_Msk                          /*!<Rx FIFO 0 Full                            */
#define FDCAN_RXF0S_RF0L_Pos      (25U)
#define FDCAN_RXF0S_RF0L_Msk      (0x1U << FDCAN_RXF0S_RF0L_Pos)               /*!< 0x02000000 */
#define FDCAN_RXF0S_RF0L          FDCAN_RXF0S_RF0L_Msk                         /*!<Rx FIFO 0 Message Lost                    */

/*****************  Bit definition for FDCAN_RXF0A register  ********************/
#define FDCAN_RXF0A_F0AI_Pos      (0U)
#define FDCAN_RXF0A_F0AI_Msk      (0x3FU << FDCAN_RXF0A_F0AI_Pos)              /*!< 0x0000003F */
#define FDCAN_RXF0A_F0AI          FDCAN_RXF0A_F0AI_Msk                         /*!<Rx FIFO 0 Acknowledge Index               */

/*****************  Bit definition for FDCAN_RXBC register  ********************/
#define FDCAN_RXBC_RBSA_Pos       (2U)
#define FDCAN_RXBC_RBSA_Msk       (0x3FU << FDCAN_RXBC_RBSA_Pos)               /*!< 0x000000FC */
#define FDCAN_RXBC_RBSA           FDCAN_RXBC_RBSA_Msk                          /*!<Rx Buffer Start Address                   */

/*****************  Bit definition for FDCAN_RXF1C register  ********************/
#define FDCAN_RXF1C_F1SA_Pos      (2U)
#define FDCAN_RXF1C_F1SA_Msk      (0x3FU << FDCAN_RXF1C_F1SA_Pos)              /*!< 0x000000FC */
#define FDCAN_RXF1C_F1SA          FDCAN_RXF1C_F1SA_Msk                         /*!<Rx FIFO 1 Start Address                   */
#define FDCAN_RXF1C_F1S_Pos       (16U)
#define FDCAN_RXF1C_F1S_Msk       (0x7FU << FDCAN_RXF1C_F1S_Pos)               /*!< 0x007F0000 */
#define FDCAN_RXF1C_F1S           FDCAN_RXF1C_F1S_Msk                          /*!<Number of Rx FIFO 1 elements              */
#define FDCAN_RXF1C_F1WM_Pos      (24U)
#define FDCAN_RXF1C_F1WM_Msk      (0x7FU << FDCAN_RXF1C_F1WM_Pos)              /*!< 0x7F000000 */
#define FDCAN_RXF1C_F1WM          FDCAN_RXF1C_F1WM_Msk                         /*!<Rx FIFO 1 Watermark                       */
#define FDCAN_RXF1C_F1OM_Pos      (31U)
#define FDCAN_RXF1C_F1OM_Msk      (0x1U << FDCAN_RXF1C_F1OM_Pos)               /*!< 0x80000000 */
#define FDCAN_RXF1C_F1OM          FDCAN_RXF1C_F1OM_Msk                         /*!<FIFO 1 Operation Mode                     */

/*****************  Bit definition for FDCAN_RXF1S register  ********************/
#define FDCAN_RXF1S_F1FL_Pos      (0U)
#define FDCAN_RXF1S_F1FL_Msk      (0x7FU << FDCAN_RXF1S_F1FL_Pos)              /*!< 0x0000007F */
#define FDCAN_RXF1S_F1FL          FDCAN_RXF1S_F1FL_Msk                         /*!<Rx FIFO 1 Fill Level                      */
#define FDCAN_RXF1S_F1GI_Pos      (8U)
#define FDCAN_RXF1S_F1GI_Msk      (0x3FU << FDCAN_RXF1S_F1GI_Pos)              /*!< 0x00003F00 */
#define FDCAN_RXF1S_F1GI          FDCAN_RXF1S_F1GI_Msk                         /*!<Rx FIFO 1 Get Index                       */
#define FDCAN_RXF1S_F1PI_Pos      (16U)
#define FDCAN_RXF1S_F1PI_Msk      (0x3FU << FDCAN_RXF1S_F1PI_Pos)              /*!< 0x003F0000 */
#define FDCAN_RXF1S_F1PI          FDCAN_RXF1S_F1PI_Msk                         /*!<Rx FIFO 1 Put Index                       */
#define FDCAN_RXF1S_F1F_Pos       (24U)
#define FDCAN_RXF1S_F1F_Msk       (0x1U << FDCAN_RXF1S_F1F_Pos)                /*!< 0x01000000 */
#define FDCAN_RXF1S_F1F           FDCAN_RXF1S_F1F_Msk                          /*!<Rx FIFO 1 Full                            */
#define FDCAN_RXF1S_RF1L_Pos      (25U)
#define FDCAN_RXF1S_RF1L_Msk      (0x1U << FDCAN_RXF1S_RF1L_Pos)               /*!< 0x02000000 */
#define FDCAN_RXF1S_RF1L          FDCAN_RXF1S_RF1L_Msk                         /*!<Rx FIFO 1 Message Lost                    */

/*****************  Bit definition for FDCAN_RXF1A register  ********************/
#define FDCAN_RXF1A_F1AI_Pos      (0U)
#define FDCAN_RXF1A_F1AI_Msk      (0x3FU << FDCAN_RXF1A_F1AI_Pos)              /*!< 0x0000003F */
#define FDCAN_RXF1A_F1AI          FDCAN_RXF1A_F1AI_Msk                         /*!<Rx FIFO 1 Acknowledge Index               */

/*****************  Bit definition for FDCAN_RXESC register  ********************/
#define FDCAN_RXESC_F0DS_Pos      (0U)
#define FDCAN_RXESC_F0DS_Msk      (0x7U << FDCAN_RXESC_F0DS_Pos)               /*!< 0x00000007 */
#define FDCAN_RXESC_F0DS          FDCAN_RXESC_F0DS_Msk                         /*!<Rx FIFO 1 Data Field Size                 */
#define FDCAN_RXESC_F1DS_Pos      (4U)
#define FDCAN_RXESC_F1DS_Msk      (0x7U << FDCAN_RXESC_F1DS_Pos)               /*!< 0x00000070 */
#define FDCAN_RXESC_F1DS          FDCAN_RXESC_F1DS_Msk                         /*!<Rx FIFO 0 Data Field Size                 */
#define FDCAN_RXESC_RBDS_Pos      (8U)
#define FDCAN_RXESC_RBDS_Msk      (0x7U << FDCAN_RXESC_RBDS_Pos)               /*!< 0x00000700 */
#define FDCAN_RXESC_RBDS          FDCAN_RXESC_RBDS_Msk                         /*!<Rx Buffer Data Field Size                 */

/*****************  Bit definition for FDCAN_TXBC register  *********************/
#define FDCAN_TXBC_TBSA_Pos       (2U)
#define FDCAN_TXBC_TBSA_Msk       (0x3FU << FDCAN_TXBC_TBSA_Pos)               /*!< 0x000000FC */
#define FDCAN_TXBC_TBSA           FDCAN_TXBC_TBSA_Msk                          /*!<Tx Buffers Start Address                  */
#define FDCAN_TXBC_NDTB_Pos       (16U)
#define FDCAN_TXBC_NDTB_Msk       (0x3FU << FDCAN_TXBC_NDTB_Pos)               /*!< 0x003F0000 */
#define FDCAN_TXBC_NDTB           FDCAN_TXBC_NDTB_Msk                          /*!<Number of Dedicated Transmit Buffers      */
#define FDCAN_TXBC_TFQS_Pos       (24U)
#define FDCAN_TXBC_TFQS_Msk       (0x3FU << FDCAN_TXBC_TFQS_Pos)               /*!< 0x3F000000 */
#define FDCAN_TXBC_TFQS           FDCAN_TXBC_TFQS_Msk                          /*!<Transmit FIFO/Queue Size                  */
#define FDCAN_TXBC_TFQM_Pos       (30U)
#define FDCAN_TXBC_TFQM_Msk       (0x1U << FDCAN_TXBC_TFQM_Pos)                /*!< 0x40000000 */
#define FDCAN_TXBC_TFQM           FDCAN_TXBC_TFQM_Msk                          /*!<Tx FIFO/Queue Mode                        */

/*****************  Bit definition for FDCAN_TXFQS register  *********************/
#define FDCAN_TXFQS_TFFL_Pos      (0U)
#define FDCAN_TXFQS_TFFL_Msk      (0x3FU << FDCAN_TXFQS_TFFL_Pos)              /*!< 0x0000003F */
#define FDCAN_TXFQS_TFFL          FDCAN_TXFQS_TFFL_Msk                         /*!<Tx FIFO Free Level                        */
#define FDCAN_TXFQS_TFGI_Pos      (8U)
#define FDCAN_TXFQS_TFGI_Msk      (0x1FU << FDCAN_TXFQS_TFGI_Pos)              /*!< 0x00001F00 */
#define FDCAN_TXFQS_TFGI          FDCAN_TXFQS_TFGI_Msk                         /*!<Tx FIFO Get Index                         */
#define FDCAN_TXFQS_TFQPI_Pos     (16U)
#define FDCAN_TXFQS_TFQPI_Msk     (0x1FU << FDCAN_TXFQS_TFQPI_Pos)             /*!< 0x001F0000 */
#define FDCAN_TXFQS_TFQPI         FDCAN_TXFQS_TFQPI_Msk                        /*!<Tx FIFO/Queue Put Index                   */
#define FDCAN_TXFQS_TFQF_Pos      (21U)
#define FDCAN_TXFQS_TFQF_Msk      (0x1U << FDCAN_TXFQS_TFQF_Pos)               /*!< 0x00200000 */
#define FDCAN_TXFQS_TFQF          FDCAN_TXFQS_TFQF_Msk                         /*!<Tx FIFO/Queue Full                        */

/*****************  Bit definition for FDCAN_TXESC register  *********************/
#define FDCAN_TXESC_TBDS_Pos      (0U)
#define FDCAN_TXESC_TBDS_Msk      (0x7U << FDCAN_TXESC_TBDS_Pos)               /*!< 0x00000007 */
#define FDCAN_TXESC_TBDS          FDCAN_TXESC_TBDS_Msk                         /*!<Tx Buffer Data Field Size                 */

/*****************  Bit definition for FDCAN_TXBRP register  *********************/
#define FDCAN_TXBRP_TRP_Pos       (0U)
#define FDCAN_TXBRP_TRP_Msk       (0xFFFFFFFFU << FDCAN_TXBRP_TRP_Pos)         /*!< 0xFFFFFFFF */
#define FDCAN_TXBRP_TRP           FDCAN_TXBRP_TRP_Msk                          /*!<Transmission Request Pending              */

/*****************  Bit definition for FDCAN_TXBAR register  *********************/
#define FDCAN_TXBAR_AR_Pos        (0U)
#define FDCAN_TXBAR_AR_Msk        (0xFFFFFFFFU << FDCAN_TXBAR_AR_Pos)          /*!< 0xFFFFFFFF */
#define FDCAN_TXBAR_AR            FDCAN_TXBAR_AR_Msk                           /*!<Add Request                               */

/*****************  Bit definition for FDCAN_TXBCR register  *********************/
#define FDCAN_TXBCR_CR_Pos        (0U)
#define FDCAN_TXBCR_CR_Msk        (0xFFFFFFFFU << FDCAN_TXBCR_CR_Pos)          /*!< 0xFFFFFFFF */
#define FDCAN_TXBCR_CR            FDCAN_TXBCR_CR_Msk                           /*!<Cancellation Request                      */

/*****************  Bit definition for FDCAN_TXBTO register  *********************/
#define FDCAN_TXBTO_TO_Pos        (0U)
#define FDCAN_TXBTO_TO_Msk        (0xFFFFFFFFU << FDCAN_TXBTO_TO_Pos)          /*!< 0xFFFFFFFF */
#define FDCAN_TXBTO_TO            FDCAN_TXBTO_TO_Msk                           /*!<Transmission Occurred                     */

/*****************  Bit definition for FDCAN_TXBCF register  *********************/
#define FDCAN_TXBCF_CF_Pos        (0U)
#define FDCAN_TXBCF_CF_Msk        (0xFFFFFFFFU << FDCAN_TXBCF_CF_Pos)          /*!< 0xFFFFFFFF */
#define FDCAN_TXBCF_CF            FDCAN_TXBCF_CF_Msk                           /*!<Cancellation Finished                     */

/*****************  Bit definition for FDCAN_TXBTIE register  ********************/
#define FDCAN_TXBTIE_TIE_Pos      (0U)
#define FDCAN_TXBTIE_TIE_Msk      (0xFFFFFFFFU << FDCAN_TXBTIE_TIE_Pos)        /*!< 0xFFFFFFFF */
#define FDCAN_TXBTIE_TIE          FDCAN_TXBTIE_TIE_Msk                         /*!<Transmission Interrupt Enable             */

/*****************  Bit definition for FDCAN_ TXBCIE register  *******************/
#define FDCAN_TXBCIE_CF_Pos       (0U)
#define FDCAN_TXBCIE_CF_Msk       (0xFFFFFFFFU << FDCAN_TXBCIE_CF_Pos)         /*!< 0xFFFFFFFF */
#define FDCAN_TXBCIE_CF           FDCAN_TXBCIE_CF_Msk                          /*!<Cancellation Finished Interrupt Enable    */

/*****************  Bit definition for FDCAN_TXEFC register  *********************/
#define FDCAN_TXEFC_EFSA_Pos      (2U)
#define FDCAN_TXEFC_EFSA_Msk      (0x3FU << FDCAN_TXEFC_EFSA_Pos)              /*!< 0x000000FC */
#define FDCAN_TXEFC_EFSA          FDCAN_TXEFC_EFSA_Msk                         /*!<Event FIFO Start Address                  */
#define FDCAN_TXEFC_EFS_Pos       (8U)
#define FDCAN_TXEFC_EFS_Msk       (0x3FU << FDCAN_TXEFC_EFS_Pos)               /*!< 0x00003F00 */
#define FDCAN_TXEFC_EFS           FDCAN_TXEFC_EFS_Msk                          /*!<Event FIFO Size                           */
#define FDCAN_TXEFC_EFWM_Pos      (24U)
#define FDCAN_TXEFC_EFWM_Msk      (0x3FU << FDCAN_TXEFC_EFWM_Pos)              /*!< 0x3F000000 */
#define FDCAN_TXEFC_EFWM          FDCAN_TXEFC_EFWM_Msk                         /*!<Event FIFO Watermark                      */

/*****************  Bit definition for FDCAN_TXEFS register  *********************/
#define FDCAN_TXEFS_EFFL_Pos      (0U)
#define FDCAN_TXEFS_EFFL_Msk      (0x3FU << FDCAN_TXEFS_EFFL_Pos)              /*!< 0x0000003F */
#define FDCAN_TXEFS_EFFL          FDCAN_TXEFS_EFFL_Msk                         /*!<Event FIFO Fill Level                     */
#define FDCAN_TXEFS_EFGI_Pos      (8U)
#define FDCAN_TXEFS_EFGI_Msk      (0x1FU << FDCAN_TXEFS_EFGI_Pos)              /*!< 0x00001F00 */
#define FDCAN_TXEFS_EFGI          FDCAN_TXEFS_EFGI_Msk                         /*!<Event FIFO Get Index                      */
#define FDCAN_TXEFS_EFPI_Pos      (16U)
#define FDCAN_TXEFS_EFPI_Msk      (0x1FU << FDCAN_TXEFS_EFPI_Pos)              /*!< 0x001F0000 */
#define FDCAN_TXEFS_EFPI          FDCAN_TXEFS_EFPI_Msk                         /*!<Event FIFO Put Index                      */
#define FDCAN_TXEFS_EFF_Pos       (24U)
#define FDCAN_TXEFS_EFF_Msk       (0x1U << FDCAN_TXEFS_EFF_Pos)                /*!< 0x01000000 */
#define FDCAN_TXEFS_EFF           FDCAN_TXEFS_EFF_Msk                          /*!<Event FIFO Full                           */
#define FDCAN_TXEFS_TEFL_Pos      (25U)
#define FDCAN_TXEFS_TEFL_Msk      (0x1U << FDCAN_TXEFS_TEFL_Pos)               /*!< 0x02000000 */
#define FDCAN_TXEFS_TEFL          FDCAN_TXEFS_TEFL_Msk                         /*!<Tx Event FIFO Element Lost                */

/*****************  Bit definition for FDCAN_TXEFA register  *********************/
#define FDCAN_TXEFA_EFAI_Pos      (0U)
#define FDCAN_TXEFA_EFAI_Msk      (0x1FU << FDCAN_TXEFA_EFAI_Pos)              /*!< 0x0000001F */
#define FDCAN_TXEFA_EFAI          FDCAN_TXEFA_EFAI_Msk                         /*!<Event FIFO Acknowledge Index              */

/*****************  Bit definition for FDCAN_TTTMC register  *********************/
#define FDCAN_TTTMC_TMSA_Pos      (2U)
#define FDCAN_TTTMC_TMSA_Msk      (0x3FFFU << FDCAN_TTTMC_TMSA_Pos)            /*!< 0x0000FFFC */
#define FDCAN_TTTMC_TMSA          FDCAN_TTTMC_TMSA_Msk                         /*!<Trigger Memory Start Address              */
#define FDCAN_TTTMC_TME_Pos       (16U)
#define FDCAN_TTTMC_TME_Msk       (0x7FU << FDCAN_TTTMC_TME_Pos)               /*!< 0x007F0000 */
#define FDCAN_TTTMC_TME           FDCAN_TTTMC_TME_Msk                          /*!<Trigger Memory Elements                   */

/*****************  Bit definition for FDCAN_TTRMC register  *********************/
#define FDCAN_TTRMC_RID_Pos       (0U)
#define FDCAN_TTRMC_RID_Msk       (0x1FFFFFFFU << FDCAN_TTRMC_RID_Pos)         /*!< 0x1FFFFFFF */
#define FDCAN_TTRMC_RID           FDCAN_TTRMC_RID_Msk                          /*!<Reference Identifier                      */
#define FDCAN_TTRMC_XTD_Pos       (30U)
#define FDCAN_TTRMC_XTD_Msk       (0x1U << FDCAN_TTRMC_XTD_Pos)                /*!< 0x40000000 */
#define FDCAN_TTRMC_XTD           FDCAN_TTRMC_XTD_Msk                          /*!< Extended Identifier                      */
#define FDCAN_TTRMC_RMPS_Pos      (31U)
#define FDCAN_TTRMC_RMPS_Msk      (0x1U << FDCAN_TTRMC_RMPS_Pos)               /*!< 0x80000000 */
#define FDCAN_TTRMC_RMPS          FDCAN_TTRMC_RMPS_Msk                         /*!<Reference Message Payload Select          */

/*****************  Bit definition for FDCAN_TTOCF register  *********************/
#define FDCAN_TTOCF_OM_Pos        (0U)
#define FDCAN_TTOCF_OM_Msk        (0x3U << FDCAN_TTOCF_OM_Pos)                 /*!< 0x00000003 */
#define FDCAN_TTOCF_OM            FDCAN_TTOCF_OM_Msk                           /*!<Operation Mode                            */
#define FDCAN_TTOCF_GEN_Pos       (3U)
#define FDCAN_TTOCF_GEN_Msk       (0x1U << FDCAN_TTOCF_GEN_Pos)                /*!< 0x00000008 */
#define FDCAN_TTOCF_GEN           FDCAN_TTOCF_GEN_Msk                          /*!<Gap Enable                                */
#define FDCAN_TTOCF_TM_Pos        (4U)
#define FDCAN_TTOCF_TM_Msk        (0x1U << FDCAN_TTOCF_TM_Pos)                 /*!< 0x00000010 */
#define FDCAN_TTOCF_TM            FDCAN_TTOCF_TM_Msk                           /*!<Time Master                               */
#define FDCAN_TTOCF_LDSDL_Pos     (5U)
#define FDCAN_TTOCF_LDSDL_Msk     (0x7U << FDCAN_TTOCF_LDSDL_Pos)              /*!< 0x000000E0 */
#define FDCAN_TTOCF_LDSDL         FDCAN_TTOCF_LDSDL_Msk                        /*!<LD of Synchronization Deviation Limit     */
#define FDCAN_TTOCF_IRTO_Pos      (8U)
#define FDCAN_TTOCF_IRTO_Msk      (0x7FU << FDCAN_TTOCF_IRTO_Pos)              /*!< 0x00007F00 */
#define FDCAN_TTOCF_IRTO          FDCAN_TTOCF_IRTO_Msk                         /*!<Initial Reference Trigger Offset          */
#define FDCAN_TTOCF_EECS_Pos      (15U)
#define FDCAN_TTOCF_EECS_Msk      (0x1U << FDCAN_TTOCF_EECS_Pos)               /*!< 0x00008000 */
#define FDCAN_TTOCF_EECS          FDCAN_TTOCF_EECS_Msk                         /*!<Enable External Clock Synchronization     */
#define FDCAN_TTOCF_AWL_Pos       (16U)
#define FDCAN_TTOCF_AWL_Msk       (0xFFU << FDCAN_TTOCF_AWL_Pos)               /*!< 0x00FF0000 */
#define FDCAN_TTOCF_AWL           FDCAN_TTOCF_AWL_Msk                          /*!<Application Watchdog Limit                */
#define FDCAN_TTOCF_EGTF_Pos      (24U)
#define FDCAN_TTOCF_EGTF_Msk      (0x1U << FDCAN_TTOCF_EGTF_Pos)               /*!< 0x01000000 */
#define FDCAN_TTOCF_EGTF          FDCAN_TTOCF_EGTF_Msk                         /*!<Enable Global Time Filtering              */
#define FDCAN_TTOCF_ECC_Pos       (25U)
#define FDCAN_TTOCF_ECC_Msk       (0x1U << FDCAN_TTOCF_ECC_Pos)                /*!< 0x02000000 */
#define FDCAN_TTOCF_ECC           FDCAN_TTOCF_ECC_Msk                          /*!<Enable Clock Calibration                  */
#define FDCAN_TTOCF_EVTP_Pos      (26U)
#define FDCAN_TTOCF_EVTP_Msk      (0x1U << FDCAN_TTOCF_EVTP_Pos)               /*!< 0x04000000 */
#define FDCAN_TTOCF_EVTP          FDCAN_TTOCF_EVTP_Msk                         /*!<Event Trigger Polarity                    */

/*****************  Bit definition for FDCAN_TTMLM register  *********************/
#define FDCAN_TTMLM_CCM_Pos       (0U)
#define FDCAN_TTMLM_CCM_Msk       (0x3FU << FDCAN_TTMLM_CCM_Pos)               /*!< 0x0000003F */
#define FDCAN_TTMLM_CCM           FDCAN_TTMLM_CCM_Msk                          /*!<Cycle Count Max                           */
#define FDCAN_TTMLM_CSS_Pos       (6U)
#define FDCAN_TTMLM_CSS_Msk       (0x3U << FDCAN_TTMLM_CSS_Pos)                /*!< 0x000000C0 */
#define FDCAN_TTMLM_CSS           FDCAN_TTMLM_CSS_Msk                          /*!<Cycle Start Synchronization               */
#define FDCAN_TTMLM_TXEW_Pos      (8U)
#define FDCAN_TTMLM_TXEW_Msk      (0xFU << FDCAN_TTMLM_TXEW_Pos)               /*!< 0x00000F00 */
#define FDCAN_TTMLM_TXEW          FDCAN_TTMLM_TXEW_Msk                         /*!<Tx Enable Window                          */
#define FDCAN_TTMLM_ENTT_Pos      (16U)
#define FDCAN_TTMLM_ENTT_Msk      (0xFFFU << FDCAN_TTMLM_ENTT_Pos)             /*!< 0x0FFF0000 */
#define FDCAN_TTMLM_ENTT          FDCAN_TTMLM_ENTT_Msk                         /*!<Expected Number of Tx Triggers            */

/*****************  Bit definition for FDCAN_TURCF register  *********************/
#define FDCAN_TURCF_NCL_Pos       (0U)
#define FDCAN_TURCF_NCL_Msk       (0xFFFFU << FDCAN_TURCF_NCL_Pos)             /*!< 0x0000FFFF */
#define FDCAN_TURCF_NCL           FDCAN_TURCF_NCL_Msk                          /*!<Numerator Configuration Low               */
#define FDCAN_TURCF_DC_Pos        (16U)
#define FDCAN_TURCF_DC_Msk        (0x3FFFU << FDCAN_TURCF_DC_Pos)              /*!< 0x3FFF0000 */
#define FDCAN_TURCF_DC            FDCAN_TURCF_DC_Msk                           /*!<Denominator Configuration                 */
#define FDCAN_TURCF_ELT_Pos       (31U)
#define FDCAN_TURCF_ELT_Msk       (0x1U << FDCAN_TURCF_ELT_Pos)                /*!< 0x80000000 */
#define FDCAN_TURCF_ELT           FDCAN_TURCF_ELT_Msk                          /*!<Enable Local Time                         */

/*****************  Bit definition for FDCAN_TTOCN register  ********************/
#define FDCAN_TTOCN_SGT_Pos       (0U)
#define FDCAN_TTOCN_SGT_Msk       (0x1U << FDCAN_TTOCN_SGT_Pos)                /*!< 0x00000001 */
#define FDCAN_TTOCN_SGT           FDCAN_TTOCN_SGT_Msk                          /*!<Set Global time                           */
#define FDCAN_TTOCN_ECS_Pos       (1U)
#define FDCAN_TTOCN_ECS_Msk       (0x1U << FDCAN_TTOCN_ECS_Pos)                /*!< 0x00000002 */
#define FDCAN_TTOCN_ECS           FDCAN_TTOCN_ECS_Msk                          /*!<External Clock Synchronization            */
#define FDCAN_TTOCN_SWP_Pos       (2U)
#define FDCAN_TTOCN_SWP_Msk       (0x1U << FDCAN_TTOCN_SWP_Pos)                /*!< 0x00000004 */
#define FDCAN_TTOCN_SWP           FDCAN_TTOCN_SWP_Msk                          /*!<Stop Watch Polarity                       */
#define FDCAN_TTOCN_SWS_Pos       (3U)
#define FDCAN_TTOCN_SWS_Msk       (0x3U << FDCAN_TTOCN_SWS_Pos)                /*!< 0x00000018 */
#define FDCAN_TTOCN_SWS           FDCAN_TTOCN_SWS_Msk                          /*!<Stop Watch Source                         */
#define FDCAN_TTOCN_RTIE_Pos      (5U)
#define FDCAN_TTOCN_RTIE_Msk      (0x1U << FDCAN_TTOCN_RTIE_Pos)               /*!< 0x00000020 */
#define FDCAN_TTOCN_RTIE          FDCAN_TTOCN_RTIE_Msk                         /*!<Register Time Mark Interrupt Pulse Enable */
#define FDCAN_TTOCN_TMC_Pos       (6U)
#define FDCAN_TTOCN_TMC_Msk       (0x3U << FDCAN_TTOCN_TMC_Pos)                /*!< 0x000000C0 */
#define FDCAN_TTOCN_TMC           FDCAN_TTOCN_TMC_Msk                          /*!<Register Time Mark Compare                */
#define FDCAN_TTOCN_TTIE_Pos      (8U)
#define FDCAN_TTOCN_TTIE_Msk      (0x1U << FDCAN_TTOCN_TTIE_Pos)               /*!< 0x00000100 */
#define FDCAN_TTOCN_TTIE          FDCAN_TTOCN_TTIE_Msk                         /*!<Trigger Time Mark Interrupt Pulse Enable  */
#define FDCAN_TTOCN_GCS_Pos       (9U)
#define FDCAN_TTOCN_GCS_Msk       (0x1U << FDCAN_TTOCN_GCS_Pos)                /*!< 0x00000200 */
#define FDCAN_TTOCN_GCS           FDCAN_TTOCN_GCS_Msk                          /*!<Gap Control Select                        */
#define FDCAN_TTOCN_FGP_Pos       (10U)
#define FDCAN_TTOCN_FGP_Msk       (0x1U << FDCAN_TTOCN_FGP_Pos)                /*!< 0x00000400 */
#define FDCAN_TTOCN_FGP           FDCAN_TTOCN_FGP_Msk                          /*!<Finish Gap                                */
#define FDCAN_TTOCN_TMG_Pos       (11U)
#define FDCAN_TTOCN_TMG_Msk       (0x1U << FDCAN_TTOCN_TMG_Pos)                /*!< 0x00000800 */
#define FDCAN_TTOCN_TMG           FDCAN_TTOCN_TMG_Msk                          /*!<Time Mark Gap                             */
#define FDCAN_TTOCN_NIG_Pos       (12U)
#define FDCAN_TTOCN_NIG_Msk       (0x1U << FDCAN_TTOCN_NIG_Pos)                /*!< 0x00001000 */
#define FDCAN_TTOCN_NIG           FDCAN_TTOCN_NIG_Msk                          /*!<Next is Gap                               */
#define FDCAN_TTOCN_ESCN_Pos      (13U)
#define FDCAN_TTOCN_ESCN_Msk      (0x1U << FDCAN_TTOCN_ESCN_Pos)               /*!< 0x00002000 */
#define FDCAN_TTOCN_ESCN          FDCAN_TTOCN_ESCN_Msk                         /*!<External Synchronization Control          */
#define FDCAN_TTOCN_LCKC_Pos      (15U)
#define FDCAN_TTOCN_LCKC_Msk      (0x1U << FDCAN_TTOCN_LCKC_Pos)               /*!< 0x00008000 */
#define FDCAN_TTOCN_LCKC          FDCAN_TTOCN_LCKC_Msk                         /*!<TT Operation Control Register Locked      */

/*****************  Bit definition for FDCAN_TTGTP register  ********************/
#define FDCAN_TTGTP_TP_Pos        (0U)
#define FDCAN_TTGTP_TP_Msk        (0xFFFFU << FDCAN_TTGTP_TP_Pos)              /*!< 0x0000FFFF */
#define FDCAN_TTGTP_TP            FDCAN_TTGTP_TP_Msk                           /*!<Time Preset                               */
#define FDCAN_TTGTP_CTP_Pos       (16U)
#define FDCAN_TTGTP_CTP_Msk       (0xFFFFU << FDCAN_TTGTP_CTP_Pos)             /*!< 0xFFFF0000 */
#define FDCAN_TTGTP_CTP           FDCAN_TTGTP_CTP_Msk                          /*!<Cycle Time Target Phase                   */

/*****************  Bit definition for FDCAN_TTTMK register  ********************/
#define FDCAN_TTTMK_TM_Pos        (0U)
#define FDCAN_TTTMK_TM_Msk        (0xFFFFU << FDCAN_TTTMK_TM_Pos)              /*!< 0x0000FFFF */
#define FDCAN_TTTMK_TM            FDCAN_TTTMK_TM_Msk                           /*!<Time Mark                                 */
#define FDCAN_TTTMK_TICC_Pos      (16U)
#define FDCAN_TTTMK_TICC_Msk      (0x7FU << FDCAN_TTTMK_TICC_Pos)              /*!< 0x007F0000 */
#define FDCAN_TTTMK_TICC          FDCAN_TTTMK_TICC_Msk                         /*!<Time Mark Cycle Code                      */
#define FDCAN_TTTMK_LCKM_Pos      (31U)
#define FDCAN_TTTMK_LCKM_Msk      (0x1U << FDCAN_TTTMK_LCKM_Pos)               /*!< 0x80000000 */
#define FDCAN_TTTMK_LCKM          FDCAN_TTTMK_LCKM_Msk                         /*!<TT Time Mark Register Locked              */

/*****************  Bit definition for FDCAN_TTIR register  ********************/
#define FDCAN_TTIR_SBC_Pos        (0U)
#define FDCAN_TTIR_SBC_Msk        (0x1U << FDCAN_TTIR_SBC_Pos)                 /*!< 0x00000001 */
#define FDCAN_TTIR_SBC            FDCAN_TTIR_SBC_Msk                           /*!<Start of Basic Cycle                      */
#define FDCAN_TTIR_SMC_Pos        (1U)
#define FDCAN_TTIR_SMC_Msk        (0x1U << FDCAN_TTIR_SMC_Pos)                 /*!< 0x00000002 */
#define FDCAN_TTIR_SMC            FDCAN_TTIR_SMC_Msk                           /*!<Start of Matrix Cycle                     */
#define FDCAN_TTIR_CSM_Pos        (2U)
#define FDCAN_TTIR_CSM_Msk        (0x1U << FDCAN_TTIR_CSM_Pos)                 /*!< 0x00000004 */
#define FDCAN_TTIR_CSM            FDCAN_TTIR_CSM_Msk                           /*!<Change of Synchronization Mode            */
#define FDCAN_TTIR_SOG_Pos        (3U)
#define FDCAN_TTIR_SOG_Msk        (0x1U << FDCAN_TTIR_SOG_Pos)                 /*!< 0x00000008 */
#define FDCAN_TTIR_SOG            FDCAN_TTIR_SOG_Msk                           /*!<Start of Gap                              */
#define FDCAN_TTIR_RTMI_Pos       (4U)
#define FDCAN_TTIR_RTMI_Msk       (0x1U << FDCAN_TTIR_RTMI_Pos)                /*!< 0x00000010 */
#define FDCAN_TTIR_RTMI           FDCAN_TTIR_RTMI_Msk                          /*!<Register Time Mark Interrupt              */
#define FDCAN_TTIR_TTMI_Pos       (5U)
#define FDCAN_TTIR_TTMI_Msk       (0x1U << FDCAN_TTIR_TTMI_Pos)                /*!< 0x00000020 */
#define FDCAN_TTIR_TTMI           FDCAN_TTIR_TTMI_Msk                          /*!<Trigger Time Mark Event Internal          */
#define FDCAN_TTIR_SWE_Pos        (6U)
#define FDCAN_TTIR_SWE_Msk        (0x1U << FDCAN_TTIR_SWE_Pos)                 /*!< 0x00000040 */
#define FDCAN_TTIR_SWE            FDCAN_TTIR_SWE_Msk                           /*!<Stop Watch Event                          */
#define FDCAN_TTIR_GTW_Pos        (7U)
#define FDCAN_TTIR_GTW_Msk        (0x1U << FDCAN_TTIR_GTW_Pos)                 /*!< 0x00000080 */
#define FDCAN_TTIR_GTW            FDCAN_TTIR_GTW_Msk                           /*!<Global Time Wrap                          */
#define FDCAN_TTIR_GTD_Pos        (8U)
#define FDCAN_TTIR_GTD_Msk        (0x1U << FDCAN_TTIR_GTD_Pos)                 /*!< 0x00000100 */
#define FDCAN_TTIR_GTD            FDCAN_TTIR_GTD_Msk                           /*!<Global Time Discontinuity                 */
#define FDCAN_TTIR_GTE_Pos        (9U)
#define FDCAN_TTIR_GTE_Msk        (0x1U << FDCAN_TTIR_GTE_Pos)                 /*!< 0x00000200 */
#define FDCAN_TTIR_GTE            FDCAN_TTIR_GTE_Msk                           /*!<Global Time Error                         */
#define FDCAN_TTIR_TXU_Pos        (10U)
#define FDCAN_TTIR_TXU_Msk        (0x1U << FDCAN_TTIR_TXU_Pos)                 /*!< 0x00000400 */
#define FDCAN_TTIR_TXU            FDCAN_TTIR_TXU_Msk                           /*!<Tx Count Underflow                        */
#define FDCAN_TTIR_TXO_Pos        (11U)
#define FDCAN_TTIR_TXO_Msk        (0x1U << FDCAN_TTIR_TXO_Pos)                 /*!< 0x00000800 */
#define FDCAN_TTIR_TXO            FDCAN_TTIR_TXO_Msk                           /*!<Tx Count Overflow                         */
#define FDCAN_TTIR_SE1_Pos        (12U)
#define FDCAN_TTIR_SE1_Msk        (0x1U << FDCAN_TTIR_SE1_Pos)                 /*!< 0x00001000 */
#define FDCAN_TTIR_SE1            FDCAN_TTIR_SE1_Msk                           /*!<Scheduling Error 1                        */
#define FDCAN_TTIR_SE2_Pos        (13U)
#define FDCAN_TTIR_SE2_Msk        (0x1U << FDCAN_TTIR_SE2_Pos)                 /*!< 0x00002000 */
#define FDCAN_TTIR_SE2            FDCAN_TTIR_SE2_Msk                           /*!<Scheduling Error 2                        */
#define FDCAN_TTIR_ELC_Pos        (14U)
#define FDCAN_TTIR_ELC_Msk        (0x1U << FDCAN_TTIR_ELC_Pos)                 /*!< 0x00004000 */
#define FDCAN_TTIR_ELC            FDCAN_TTIR_ELC_Msk                           /*!<Error Level Changed                       */
#define FDCAN_TTIR_IWT_Pos        (15U)
#define FDCAN_TTIR_IWT_Msk        (0x1U << FDCAN_TTIR_IWT_Pos)                 /*!< 0x00008000 */
#define FDCAN_TTIR_IWT            FDCAN_TTIR_IWT_Msk                           /*!<Initialization Watch Trigger              */
#define FDCAN_TTIR_WT_Pos         (16U)
#define FDCAN_TTIR_WT_Msk         (0x1U << FDCAN_TTIR_WT_Pos)                  /*!< 0x00010000 */
#define FDCAN_TTIR_WT             FDCAN_TTIR_WT_Msk                            /*!<Watch Trigger                             */
#define FDCAN_TTIR_AW_Pos         (17U)
#define FDCAN_TTIR_AW_Msk         (0x1U << FDCAN_TTIR_AW_Pos)                  /*!< 0x00020000 */
#define FDCAN_TTIR_AW             FDCAN_TTIR_AW_Msk                            /*!<Application Watchdog                      */
#define FDCAN_TTIR_CER_Pos        (18U)
#define FDCAN_TTIR_CER_Msk        (0x1U << FDCAN_TTIR_CER_Pos)                 /*!< 0x00040000 */
#define FDCAN_TTIR_CER            FDCAN_TTIR_CER_Msk                           /*!<Configuration Error                       */

/*****************  Bit definition for FDCAN_TTIE register  ********************/
#define FDCAN_TTIE_SBCE_Pos       (0U)
#define FDCAN_TTIE_SBCE_Msk       (0x1U << FDCAN_TTIE_SBCE_Pos)                /*!< 0x00000001 */
#define FDCAN_TTIE_SBCE           FDCAN_TTIE_SBCE_Msk                          /*!<Start of Basic Cycle Interrupt Enable             */
#define FDCAN_TTIE_SMCE_Pos       (1U)
#define FDCAN_TTIE_SMCE_Msk       (0x1U << FDCAN_TTIE_SMCE_Pos)                /*!< 0x00000002 */
#define FDCAN_TTIE_SMCE           FDCAN_TTIE_SMCE_Msk                          /*!<Start of Matrix Cycle Interrupt Enable            */
#define FDCAN_TTIE_CSME_Pos       (2U)
#define FDCAN_TTIE_CSME_Msk       (0x1U << FDCAN_TTIE_CSME_Pos)                /*!< 0x00000004 */
#define FDCAN_TTIE_CSME           FDCAN_TTIE_CSME_Msk                          /*!<Change of Synchronization Mode Interrupt Enable   */
#define FDCAN_TTIE_SOGE_Pos       (3U)
#define FDCAN_TTIE_SOGE_Msk       (0x1U << FDCAN_TTIE_SOGE_Pos)                /*!< 0x00000008 */
#define FDCAN_TTIE_SOGE           FDCAN_TTIE_SOGE_Msk                          /*!<Start of Gap Interrupt Enable                     */
#define FDCAN_TTIE_RTMIE_Pos      (4U)
#define FDCAN_TTIE_RTMIE_Msk      (0x1U << FDCAN_TTIE_RTMIE_Pos)               /*!< 0x00000010 */
#define FDCAN_TTIE_RTMIE          FDCAN_TTIE_RTMIE_Msk                         /*!<Register Time Mark Interrupt Interrupt Enable     */
#define FDCAN_TTIE_TTMIE_Pos      (5U)
#define FDCAN_TTIE_TTMIE_Msk      (0x1U << FDCAN_TTIE_TTMIE_Pos)               /*!< 0x00000020 */
#define FDCAN_TTIE_TTMIE          FDCAN_TTIE_TTMIE_Msk                         /*!<Trigger Time Mark Event Internal Interrupt Enable */
#define FDCAN_TTIE_SWEE_Pos       (6U)
#define FDCAN_TTIE_SWEE_Msk       (0x1U << FDCAN_TTIE_SWEE_Pos)                /*!< 0x00000040 */
#define FDCAN_TTIE_SWEE           FDCAN_TTIE_SWEE_Msk                          /*!<Stop Watch Event Interrupt Enable                 */
#define FDCAN_TTIE_GTWE_Pos       (7U)
#define FDCAN_TTIE_GTWE_Msk       (0x1U << FDCAN_TTIE_GTWE_Pos)                /*!< 0x00000080 */
#define FDCAN_TTIE_GTWE           FDCAN_TTIE_GTWE_Msk                          /*!<Global Time Wrap Interrupt Enable                 */
#define FDCAN_TTIE_GTDE_Pos       (8U)
#define FDCAN_TTIE_GTDE_Msk       (0x1U << FDCAN_TTIE_GTDE_Pos)                /*!< 0x00000100 */
#define FDCAN_TTIE_GTDE           FDCAN_TTIE_GTDE_Msk                          /*!<Global Time Discontinuity Interrupt Enable        */
#define FDCAN_TTIE_GTEE_Pos       (9U)
#define FDCAN_TTIE_GTEE_Msk       (0x1U << FDCAN_TTIE_GTEE_Pos)                /*!< 0x00000200 */
#define FDCAN_TTIE_GTEE           FDCAN_TTIE_GTEE_Msk                          /*!<Global Time Error Interrupt Enable                */
#define FDCAN_TTIE_TXUE_Pos       (10U)
#define FDCAN_TTIE_TXUE_Msk       (0x1U << FDCAN_TTIE_TXUE_Pos)                /*!< 0x00000400 */
#define FDCAN_TTIE_TXUE           FDCAN_TTIE_TXUE_Msk                          /*!<Tx Count Underflow Interrupt Enable               */
#define FDCAN_TTIE_TXOE_Pos       (11U)
#define FDCAN_TTIE_TXOE_Msk       (0x1U << FDCAN_TTIE_TXOE_Pos)                /*!< 0x00000800 */
#define FDCAN_TTIE_TXOE           FDCAN_TTIE_TXOE_Msk                          /*!<Tx Count Overflow Interrupt Enable                */
#define FDCAN_TTIE_SE1E_Pos       (12U)
#define FDCAN_TTIE_SE1E_Msk       (0x1U << FDCAN_TTIE_SE1E_Pos)                /*!< 0x00001000 */
#define FDCAN_TTIE_SE1E           FDCAN_TTIE_SE1E_Msk                          /*!<Scheduling Error 1 Interrupt Enable               */
#define FDCAN_TTIE_SE2E_Pos       (13U)
#define FDCAN_TTIE_SE2E_Msk       (0x1U << FDCAN_TTIE_SE2E_Pos)                /*!< 0x00002000 */
#define FDCAN_TTIE_SE2E           FDCAN_TTIE_SE2E_Msk                          /*!<Scheduling Error 2 Interrupt Enable               */
#define FDCAN_TTIE_ELCE_Pos       (14U)
#define FDCAN_TTIE_ELCE_Msk       (0x1U << FDCAN_TTIE_ELCE_Pos)                /*!< 0x00004000 */
#define FDCAN_TTIE_ELCE           FDCAN_TTIE_ELCE_Msk                          /*!<Error Level Changed Interrupt Enable              */
#define FDCAN_TTIE_IWTE_Pos       (15U)
#define FDCAN_TTIE_IWTE_Msk       (0x1U << FDCAN_TTIE_IWTE_Pos)                /*!< 0x00008000 */
#define FDCAN_TTIE_IWTE           FDCAN_TTIE_IWTE_Msk                          /*!<Initialization Watch Trigger Interrupt Enable     */
#define FDCAN_TTIE_WTE_Pos        (16U)
#define FDCAN_TTIE_WTE_Msk        (0x1U << FDCAN_TTIE_WTE_Pos)                 /*!< 0x00010000 */
#define FDCAN_TTIE_WTE            FDCAN_TTIE_WTE_Msk                           /*!<Watch Trigger Interrupt Enable                    */
#define FDCAN_TTIE_AWE_Pos        (17U)
#define FDCAN_TTIE_AWE_Msk        (0x1U << FDCAN_TTIE_AWE_Pos)                 /*!< 0x00020000 */
#define FDCAN_TTIE_AWE            FDCAN_TTIE_AWE_Msk                           /*!<Application Watchdog Interrupt Enable             */
#define FDCAN_TTIE_CERE_Pos       (18U)
#define FDCAN_TTIE_CERE_Msk       (0x1U << FDCAN_TTIE_CERE_Pos)                /*!< 0x00040000 */
#define FDCAN_TTIE_CERE           FDCAN_TTIE_CERE_Msk                          /*!<Configuration Error Interrupt Enable              */

/*****************  Bit definition for FDCAN_TTILS register  ********************/
#define FDCAN_TTILS_SBCS_Pos      (0U)
#define FDCAN_TTILS_SBCS_Msk      (0x1U << FDCAN_TTILS_SBCS_Pos)               /*!< 0x00000001 */
#define FDCAN_TTILS_SBCS          FDCAN_TTILS_SBCS_Msk                         /*!<Start of Basic Cycle Interrupt Line               */
#define FDCAN_TTILS_SMCS_Pos      (1U)
#define FDCAN_TTILS_SMCS_Msk      (0x1U << FDCAN_TTILS_SMCS_Pos)               /*!< 0x00000002 */
#define FDCAN_TTILS_SMCS          FDCAN_TTILS_SMCS_Msk                         /*!<Start of Matrix Cycle Interrupt Line              */
#define FDCAN_TTILS_CSMS_Pos      (2U)
#define FDCAN_TTILS_CSMS_Msk      (0x1U << FDCAN_TTILS_CSMS_Pos)               /*!< 0x00000004 */
#define FDCAN_TTILS_CSMS          FDCAN_TTILS_CSMS_Msk                         /*!<Change of Synchronization Mode Interrupt Line     */
#define FDCAN_TTILS_SOGS_Pos      (3U)
#define FDCAN_TTILS_SOGS_Msk      (0x1U << FDCAN_TTILS_SOGS_Pos)               /*!< 0x00000008 */
#define FDCAN_TTILS_SOGS          FDCAN_TTILS_SOGS_Msk                         /*!<Start of Gap Interrupt Line                       */
#define FDCAN_TTILS_RTMIS_Pos     (4U)
#define FDCAN_TTILS_RTMIS_Msk     (0x1U << FDCAN_TTILS_RTMIS_Pos)              /*!< 0x00000010 */
#define FDCAN_TTILS_RTMIS         FDCAN_TTILS_RTMIS_Msk                        /*!<Register Time Mark Interrupt Interrupt Line       */
#define FDCAN_TTILS_TTMIS_Pos     (5U)
#define FDCAN_TTILS_TTMIS_Msk     (0x1U << FDCAN_TTILS_TTMIS_Pos)              /*!< 0x00000020 */
#define FDCAN_TTILS_TTMIS         FDCAN_TTILS_TTMIS_Msk                        /*!<Trigger Time Mark Event Internal Interrupt Line   */
#define FDCAN_TTILS_SWES_Pos      (6U)
#define FDCAN_TTILS_SWES_Msk      (0x1U << FDCAN_TTILS_SWES_Pos)               /*!< 0x00000040 */
#define FDCAN_TTILS_SWES          FDCAN_TTILS_SWES_Msk                         /*!<Stop Watch Event Interrupt Line                   */
#define FDCAN_TTILS_GTWS_Pos      (7U)
#define FDCAN_TTILS_GTWS_Msk      (0x1U << FDCAN_TTILS_GTWS_Pos)               /*!< 0x00000080 */
#define FDCAN_TTILS_GTWS          FDCAN_TTILS_GTWS_Msk                         /*!<Global Time Wrap Interrupt Line                   */
#define FDCAN_TTILS_GTDS_Pos      (8U)
#define FDCAN_TTILS_GTDS_Msk      (0x1U << FDCAN_TTILS_GTDS_Pos)               /*!< 0x00000100 */
#define FDCAN_TTILS_GTDS          FDCAN_TTILS_GTDS_Msk                         /*!<Global Time Discontinuity Interrupt Line          */
#define FDCAN_TTILS_GTES_Pos      (9U)
#define FDCAN_TTILS_GTES_Msk      (0x1U << FDCAN_TTILS_GTES_Pos)               /*!< 0x00000200 */
#define FDCAN_TTILS_GTES          FDCAN_TTILS_GTES_Msk                         /*!<Global Time Error Interrupt Line                  */
#define FDCAN_TTILS_TXUS_Pos      (10U)
#define FDCAN_TTILS_TXUS_Msk      (0x1U << FDCAN_TTILS_TXUS_Pos)               /*!< 0x00000400 */
#define FDCAN_TTILS_TXUS          FDCAN_TTILS_TXUS_Msk                         /*!<Tx Count Underflow Interrupt Line                 */
#define FDCAN_TTILS_TXOS_Pos      (11U)
#define FDCAN_TTILS_TXOS_Msk      (0x1U << FDCAN_TTILS_TXOS_Pos)               /*!< 0x00000800 */
#define FDCAN_TTILS_TXOS          FDCAN_TTILS_TXOS_Msk                         /*!<Tx Count Overflow Interrupt Line                  */
#define FDCAN_TTILS_SE1S_Pos      (12U)
#define FDCAN_TTILS_SE1S_Msk      (0x1U << FDCAN_TTILS_SE1S_Pos)               /*!< 0x00001000 */
#define FDCAN_TTILS_SE1S          FDCAN_TTILS_SE1S_Msk                         /*!<Scheduling Error 1 Interrupt Line                 */
#define FDCAN_TTILS_SE2S_Pos      (13U)
#define FDCAN_TTILS_SE2S_Msk      (0x1U << FDCAN_TTILS_SE2S_Pos)               /*!< 0x00002000 */
#define FDCAN_TTILS_SE2S          FDCAN_TTILS_SE2S_Msk                         /*!<Scheduling Error 2 Interrupt Line                 */
#define FDCAN_TTILS_ELCS_Pos      (14U)
#define FDCAN_TTILS_ELCS_Msk      (0x1U << FDCAN_TTILS_ELCS_Pos)               /*!< 0x00004000 */
#define FDCAN_TTILS_ELCS          FDCAN_TTILS_ELCS_Msk                         /*!<Error Level Changed Interrupt Line                */
#define FDCAN_TTILS_IWTS_Pos      (15U)
#define FDCAN_TTILS_IWTS_Msk      (0x1U << FDCAN_TTILS_IWTS_Pos)               /*!< 0x00008000 */
#define FDCAN_TTILS_IWTS          FDCAN_TTILS_IWTS_Msk                         /*!<Initialization Watch Trigger Interrupt Line       */
#define FDCAN_TTILS_WTS_Pos       (16U)
#define FDCAN_TTILS_WTS_Msk       (0x1U << FDCAN_TTILS_WTS_Pos)                /*!< 0x00010000 */
#define FDCAN_TTILS_WTS           FDCAN_TTILS_WTS_Msk                          /*!<Watch Trigger Interrupt Line                      */
#define FDCAN_TTILS_AWS_Pos       (17U)
#define FDCAN_TTILS_AWS_Msk       (0x1U << FDCAN_TTILS_AWS_Pos)                /*!< 0x00020000 */
#define FDCAN_TTILS_AWS           FDCAN_TTILS_AWS_Msk                          /*!<Application Watchdog Interrupt Line               */
#define FDCAN_TTILS_CERS_Pos      (18U)
#define FDCAN_TTILS_CERS_Msk      (0x1U << FDCAN_TTILS_CERS_Pos)               /*!< 0x00040000 */
#define FDCAN_TTILS_CERS          FDCAN_TTILS_CERS_Msk                         /*!<Configuration Error Interrupt Line                */

/*****************  Bit definition for FDCAN_TTOST register  ********************/
#define FDCAN_TTOST_EL_Pos        (0U)
#define FDCAN_TTOST_EL_Msk        (0x3U << FDCAN_TTOST_EL_Pos)                 /*!< 0x00000003 */
#define FDCAN_TTOST_EL            FDCAN_TTOST_EL_Msk                           /*!<Error Level                              */
#define FDCAN_TTOST_MS_Pos        (2U)
#define FDCAN_TTOST_MS_Msk        (0x3U << FDCAN_TTOST_MS_Pos)                 /*!< 0x0000000C */
#define FDCAN_TTOST_MS            FDCAN_TTOST_MS_Msk                           /*!<Master State                             */
#define FDCAN_TTOST_SYS_Pos       (4U)
#define FDCAN_TTOST_SYS_Msk       (0x3U << FDCAN_TTOST_SYS_Pos)                /*!< 0x00000030 */
#define FDCAN_TTOST_SYS           FDCAN_TTOST_SYS_Msk                          /*!<Synchronization State                    */
#define FDCAN_TTOST_QGTP_Pos      (6U)
#define FDCAN_TTOST_QGTP_Msk      (0x1U << FDCAN_TTOST_QGTP_Pos)               /*!< 0x00000040 */
#define FDCAN_TTOST_QGTP          FDCAN_TTOST_QGTP_Msk                         /*!<Quality of Global Time Phase             */
#define FDCAN_TTOST_QCS_Pos       (7U)
#define FDCAN_TTOST_QCS_Msk       (0x1U << FDCAN_TTOST_QCS_Pos)                /*!< 0x00000080 */
#define FDCAN_TTOST_QCS           FDCAN_TTOST_QCS_Msk                          /*!<Quality of Clock Speed                   */
#define FDCAN_TTOST_RTO_Pos       (8U)
#define FDCAN_TTOST_RTO_Msk       (0xFFU << FDCAN_TTOST_RTO_Pos)               /*!< 0x0000FF00 */
#define FDCAN_TTOST_RTO           FDCAN_TTOST_RTO_Msk                          /*!<Reference Trigger Offset                 */
#define FDCAN_TTOST_WGTD_Pos      (22U)
#define FDCAN_TTOST_WGTD_Msk      (0x1U << FDCAN_TTOST_WGTD_Pos)               /*!< 0x00400000 */
#define FDCAN_TTOST_WGTD          FDCAN_TTOST_WGTD_Msk                         /*!<Wait for Global Time Discontinuity       */
#define FDCAN_TTOST_GFI_Pos       (23U)
#define FDCAN_TTOST_GFI_Msk       (0x1U << FDCAN_TTOST_GFI_Pos)                /*!< 0x00800000 */
#define FDCAN_TTOST_GFI           FDCAN_TTOST_GFI_Msk                          /*!<Gap Finished Indicator                   */
#define FDCAN_TTOST_TMP_Pos       (24U)
#define FDCAN_TTOST_TMP_Msk       (0x7U << FDCAN_TTOST_TMP_Pos)                /*!< 0x07000000 */
#define FDCAN_TTOST_TMP           FDCAN_TTOST_TMP_Msk                          /*!<Time Master Priority                     */
#define FDCAN_TTOST_GSI_Pos       (27U)
#define FDCAN_TTOST_GSI_Msk       (0x1U << FDCAN_TTOST_GSI_Pos)                /*!< 0x08000000 */
#define FDCAN_TTOST_GSI           FDCAN_TTOST_GSI_Msk                          /*!<Gap Started Indicator                    */
#define FDCAN_TTOST_WFE_Pos       (28U)
#define FDCAN_TTOST_WFE_Msk       (0x1U << FDCAN_TTOST_WFE_Pos)                /*!< 0x10000000 */
#define FDCAN_TTOST_WFE           FDCAN_TTOST_WFE_Msk                          /*!<Wait for Event                           */
#define FDCAN_TTOST_AWE_Pos       (29U)
#define FDCAN_TTOST_AWE_Msk       (0x1U << FDCAN_TTOST_AWE_Pos)                /*!< 0x20000000 */
#define FDCAN_TTOST_AWE           FDCAN_TTOST_AWE_Msk                          /*!<Application Watchdog Event               */
#define FDCAN_TTOST_WECS_Pos      (30U)
#define FDCAN_TTOST_WECS_Msk      (0x1U << FDCAN_TTOST_WECS_Pos)               /*!< 0x40000000 */
#define FDCAN_TTOST_WECS          FDCAN_TTOST_WECS_Msk                         /*!<Wait for External Clock Synchronization  */
#define FDCAN_TTOST_SPL_Pos       (31U)
#define FDCAN_TTOST_SPL_Msk       (0x1U << FDCAN_TTOST_SPL_Pos)                /*!< 0x80000000 */
#define FDCAN_TTOST_SPL           FDCAN_TTOST_SPL_Msk                          /*!<Schedule Phase Lock                      */

/*****************  Bit definition for FDCAN_TURNA register  ********************/
#define FDCAN_TURNA_NAV_Pos       (0U)
#define FDCAN_TURNA_NAV_Msk       (0x3FFFFU << FDCAN_TURNA_NAV_Pos)            /*!< 0x0003FFFF */
#define FDCAN_TURNA_NAV           FDCAN_TURNA_NAV_Msk                          /*!<Numerator Actual Value                   */

/*****************  Bit definition for FDCAN_TTLGT register  ********************/
#define FDCAN_TTLGT_LT_Pos        (0U)
#define FDCAN_TTLGT_LT_Msk        (0xFFFFU << FDCAN_TTLGT_LT_Pos)              /*!< 0x0000FFFF */
#define FDCAN_TTLGT_LT            FDCAN_TTLGT_LT_Msk                           /*!<Local Time                               */
#define FDCAN_TTLGT_GT_Pos        (16U)
#define FDCAN_TTLGT_GT_Msk        (0xFFFFU << FDCAN_TTLGT_GT_Pos)              /*!< 0xFFFF0000 */
#define FDCAN_TTLGT_GT            FDCAN_TTLGT_GT_Msk                           /*!<Global Time                              */

/*****************  Bit definition for FDCAN_TTCTC register  ********************/
#define FDCAN_TTCTC_CT_Pos        (0U)
#define FDCAN_TTCTC_CT_Msk        (0xFFFFU << FDCAN_TTCTC_CT_Pos)              /*!< 0x0000FFFF */
#define FDCAN_TTCTC_CT            FDCAN_TTCTC_CT_Msk                           /*!<Cycle Time                               */
#define FDCAN_TTCTC_CC_Pos        (16U)
#define FDCAN_TTCTC_CC_Msk        (0x3FU << FDCAN_TTCTC_CC_Pos)                /*!< 0x003F0000 */
#define FDCAN_TTCTC_CC            FDCAN_TTCTC_CC_Msk                           /*!<Cycle Count                              */

/*****************  Bit definition for FDCAN_TTCPT register  ********************/
#define FDCAN_TTCPT_CCV_Pos       (0U)
#define FDCAN_TTCPT_CCV_Msk       (0x3FU << FDCAN_TTCPT_CCV_Pos)               /*!< 0x0000003F */
#define FDCAN_TTCPT_CCV           FDCAN_TTCPT_CCV_Msk                          /*!<Cycle Count Value                        */
#define FDCAN_TTCPT_SWV_Pos       (16U)
#define FDCAN_TTCPT_SWV_Msk       (0xFFFFU << FDCAN_TTCPT_SWV_Pos)             /*!< 0xFFFF0000 */
#define FDCAN_TTCPT_SWV           FDCAN_TTCPT_SWV_Msk                          /*!<Stop Watch Value                         */

/*****************  Bit definition for FDCAN_TTCSM register  ********************/
#define FDCAN_TTCSM_CSM_Pos       (0U)
#define FDCAN_TTCSM_CSM_Msk       (0xFFFFU << FDCAN_TTCSM_CSM_Pos)             /*!< 0x0000FFFF */
#define FDCAN_TTCSM_CSM           FDCAN_TTCSM_CSM_Msk                          /*!<Cycle Sync Mark                          */

/*****************  Bit definition for FDCAN_TTTS register  *********************/
#define FDCAN_TTTS_SWTSEL_Pos     (0U)
#define FDCAN_TTTS_SWTSEL_Msk     (0x3U << FDCAN_TTTS_SWTSEL_Pos)              /*!< 0x00000003 */
#define FDCAN_TTTS_SWTSEL         FDCAN_TTTS_SWTSEL_Msk                        /*!<Stop watch trigger input selection       */
#define FDCAN_TTTS_EVTSEL_Pos     (4U)
#define FDCAN_TTTS_EVTSEL_Msk     (0x3U << FDCAN_TTTS_EVTSEL_Pos)              /*!< 0x00000030 */
#define FDCAN_TTTS_EVTSEL         FDCAN_TTTS_EVTSEL_Msk                        /*!<Event trigger input selection            */

/********************************************************************************/
/*                                                                              */
/*                      FDCANCCU (Clock Calibration unit)                       */
/*                                                                              */
/********************************************************************************/

/*****************  Bit definition for FDCANCCU_CREL register  ******************/
#define FDCANCCU_CREL_DAY_Pos        (0U)
#define FDCANCCU_CREL_DAY_Msk        (0xFFU << FDCANCCU_CREL_DAY_Pos)          /*!< 0x000000FF */
#define FDCANCCU_CREL_DAY            FDCANCCU_CREL_DAY_Msk                     /*!<Timestamp Day                           */
#define FDCANCCU_CREL_MON_Pos        (8U)
#define FDCANCCU_CREL_MON_Msk        (0xFFU << FDCANCCU_CREL_MON_Pos)          /*!< 0x0000FF00 */
#define FDCANCCU_CREL_MON            FDCANCCU_CREL_MON_Msk                     /*!<Timestamp Month                         */
#define FDCANCCU_CREL_YEAR_Pos       (16U)
#define FDCANCCU_CREL_YEAR_Msk       (0xFU << FDCANCCU_CREL_YEAR_Pos)          /*!< 0x000F0000 */
#define FDCANCCU_CREL_YEAR           FDCANCCU_CREL_YEAR_Msk                    /*!<Timestamp Year                          */
#define FDCANCCU_CREL_SUBSTEP_Pos    (20U)
#define FDCANCCU_CREL_SUBSTEP_Msk    (0xFU << FDCANCCU_CREL_SUBSTEP_Pos)       /*!< 0x00F00000 */
#define FDCANCCU_CREL_SUBSTEP        FDCANCCU_CREL_SUBSTEP_Msk                 /*!<Sub-step of Core release                */
#define FDCANCCU_CREL_STEP_Pos       (24U)
#define FDCANCCU_CREL_STEP_Msk       (0xFU << FDCANCCU_CREL_STEP_Pos)          /*!< 0x0F000000 */
#define FDCANCCU_CREL_STEP           FDCANCCU_CREL_STEP_Msk                    /*!<Step of Core release                    */
#define FDCANCCU_CREL_REL_Pos        (28U)
#define FDCANCCU_CREL_REL_Msk        (0xFU << FDCANCCU_CREL_REL_Pos)           /*!< 0xF0000000 */
#define FDCANCCU_CREL_REL            FDCANCCU_CREL_REL_Msk                     /*!<Core release                            */

/*****************  Bit definition for FDCANCCU_CCFG register  ******************/
#define FDCANCCU_CCFG_TQBT_Pos       (0U)
#define FDCANCCU_CCFG_TQBT_Msk       (0x1FU << FDCANCCU_CCFG_TQBT_Pos)         /*!< 0x0000001F */
#define FDCANCCU_CCFG_TQBT           FDCANCCU_CCFG_TQBT_Msk                    /*!<Time Quanta per Bit Time                */
#define FDCANCCU_CCFG_BCC_Pos        (6U)
#define FDCANCCU_CCFG_BCC_Msk        (0x1U << FDCANCCU_CCFG_BCC_Pos)           /*!< 0x00000040 */
#define FDCANCCU_CCFG_BCC            FDCANCCU_CCFG_BCC_Msk                     /*!<Bypass Clock Calibration                */
#define FDCANCCU_CCFG_CFL_Pos        (7U)
#define FDCANCCU_CCFG_CFL_Msk        (0x1U << FDCANCCU_CCFG_CFL_Pos)           /*!< 0x00000080 */
#define FDCANCCU_CCFG_CFL            FDCANCCU_CCFG_CFL_Msk                     /*!<Calibration Field Length                */
#define FDCANCCU_CCFG_OCPM_Pos       (8U)
#define FDCANCCU_CCFG_OCPM_Msk       (0xFFU << FDCANCCU_CCFG_OCPM_Pos)         /*!< 0x0000FF00 */
#define FDCANCCU_CCFG_OCPM           FDCANCCU_CCFG_OCPM_Msk                    /*!<Oscillator Clock Periods Minimum        */
#define FDCANCCU_CCFG_CDIV_Pos       (16U)
#define FDCANCCU_CCFG_CDIV_Msk       (0xFU << FDCANCCU_CCFG_CDIV_Pos)          /*!< 0x000F0000 */
#define FDCANCCU_CCFG_CDIV           FDCANCCU_CCFG_CDIV_Msk                    /*!<Clock Divider                           */
#define FDCANCCU_CCFG_SWR_Pos        (31U)
#define FDCANCCU_CCFG_SWR_Msk        (0x1U << FDCANCCU_CCFG_SWR_Pos)           /*!< 0x80000000 */
#define FDCANCCU_CCFG_SWR            FDCANCCU_CCFG_SWR_Msk                     /*!<Software Reset                          */

/*****************  Bit definition for FDCANCCU_CSTAT register  *****************/
#define FDCANCCU_CSTAT_OCPC_Pos      (0U)
#define FDCANCCU_CSTAT_OCPC_Msk      (0x3FFFFU << FDCANCCU_CSTAT_OCPC_Pos)     /*!< 0x0003FFFF */
#define FDCANCCU_CSTAT_OCPC          FDCANCCU_CSTAT_OCPC_Msk                   /*!<Oscillator Clock Period Counter        */
#define FDCANCCU_CSTAT_TQC_Pos       (18U)
#define FDCANCCU_CSTAT_TQC_Msk       (0x7FFU << FDCANCCU_CSTAT_TQC_Pos)        /*!< 0x1FFC0000 */
#define FDCANCCU_CSTAT_TQC           FDCANCCU_CSTAT_TQC_Msk                    /*!<Time Quanta Counter                    */
#define FDCANCCU_CSTAT_CALS_Pos      (30U)
#define FDCANCCU_CSTAT_CALS_Msk      (0x3U << FDCANCCU_CSTAT_CALS_Pos)         /*!< 0xC0000000 */
#define FDCANCCU_CSTAT_CALS          FDCANCCU_CSTAT_CALS_Msk                   /*!<Calibration State                      */

/******************  Bit definition for FDCANCCU_CWD register  ******************/
#define FDCANCCU_CWD_WDC_Pos         (0U)
#define FDCANCCU_CWD_WDC_Msk         (0xFFFFU << FDCANCCU_CWD_WDC_Pos)         /*!< 0x0000FFFF */
#define FDCANCCU_CWD_WDC             FDCANCCU_CWD_WDC_Msk                      /*!<Watchdog Configuration                 */
#define FDCANCCU_CWD_WDV_Pos         (16U)
#define FDCANCCU_CWD_WDV_Msk         (0xFFFFU << FDCANCCU_CWD_WDV_Pos)         /*!< 0xFFFF0000 */
#define FDCANCCU_CWD_WDV             FDCANCCU_CWD_WDV_Msk                      /*!<Watchdog Value                         */

/******************  Bit definition for FDCANCCU_IR register  *******************/
#define FDCANCCU_IR_CWE_Pos          (0U)
#define FDCANCCU_IR_CWE_Msk          (0x1U << FDCANCCU_IR_CWE_Pos)             /*!< 0x00000001 */
#define FDCANCCU_IR_CWE              FDCANCCU_IR_CWE_Msk                       /*!<Calibration Watchdog Event             */
#define FDCANCCU_IR_CSC_Pos          (1U)
#define FDCANCCU_IR_CSC_Msk          (0x1U << FDCANCCU_IR_CSC_Pos)             /*!< 0x00000002 */
#define FDCANCCU_IR_CSC              FDCANCCU_IR_CSC_Msk                       /*!<Calibration State Changed              */

/******************  Bit definition for FDCANCCU_IE register  *******************/
#define FDCANCCU_IE_CWEE_Pos         (0U)
#define FDCANCCU_IE_CWEE_Msk         (0x1U << FDCANCCU_IE_CWEE_Pos)            /*!< 0x00000001 */
#define FDCANCCU_IE_CWEE             FDCANCCU_IE_CWEE_Msk                      /*!<Calibration Watchdog Event Enable      */
#define FDCANCCU_IE_CSCE_Pos         (1U)
#define FDCANCCU_IE_CSCE_Msk         (0x1U << FDCANCCU_IE_CSCE_Pos)            /*!< 0x00000002 */
#define FDCANCCU_IE_CSCE             FDCANCCU_IE_CSCE_Msk                      /*!<Calibration State Changed Enable       */


/// TODO: I put the important bits elsewhere, but should it all go here?

//#define RCC_D2CCIP1R_FDCANSEL_Pos              (28U)
//#define RCC_D2CCIP1R_FDCANSEL_Msk              (0x3U << RCC_D2CCIP1R_FDCANSEL_Pos) /*!< 0x30000000 */
//#define RCC_D2CCIP1R_FDCANSEL                  RCC_D2CCIP1R_FDCANSEL_Msk
//#define RCC_D2CCIP1R_FDCANSEL_0                (0x1U << RCC_D2CCIP1R_FDCANSEL_Pos) /*!< 0x10000000 */
//#define RCC_D2CCIP1R_FDCANSEL_1                (0x2U << RCC_D2CCIP1R_FDCANSEL_Pos) /*!< 0x20000000 */
//
//#define RCC_APB1HENR_FDCANEN_Pos               (8U)
//#define RCC_APB1HENR_FDCANEN_Msk               (0x1U << RCC_APB1HENR_FDCANEN_Pos) /*!< 0x00000100 */
//#define RCC_APB1HENR_FDCANEN                   RCC_APB1HENR_FDCANEN_Msk
//
//
//#define RCC_APB1HRSTR_FDCANRST_Pos             (8U)
//#define RCC_APB1HRSTR_FDCANRST_Msk             (0x1U << RCC_APB1HRSTR_FDCANRST_Pos) /*!< 0x00000100 */
//#define RCC_APB1HRSTR_FDCANRST                 RCC_APB1HRSTR_FDCANRST_Msk
//
///********************  Bit definition for RCC_APB1HLPENR register  ******************/
//#define RCC_APB1HLPENR_FDCANLPEN_Pos           (8U)
//#define RCC_APB1HLPENR_FDCANLPEN_Msk           (0x1U << RCC_APB1HLPENR_FDCANLPEN_Pos) /*!< 0x00000100 */
//#define RCC_APB1HLPENR_FDCANLPEN               RCC_APB1HLPENR_FDCANLPEN_Msk
//
//
//
///********************  Bit definition for APB1HFZ1 register  ************/
//#define DBGMCU_APB1HFZ1_DBG_FDCAN_Pos     (8U)
//#define DBGMCU_APB1HFZ1_DBG_FDCAN_Msk     (0x1U << DBGMCU_APB1HFZ1_DBG_FDCAN_Pos) /*!< 0x00000100 */
//#define DBGMCU_APB1HFZ1_DBG_FDCAN         DBGMCU_APB1HFZ1_DBG_FDCAN_Msk

#ifdef __cplusplus
}
#endif /* __cplusplus */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
