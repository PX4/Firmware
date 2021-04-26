/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *       Author: David Sidrane <david.sidrane@nscdg.com>
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

enum { HWMaxMB = 16 };

typedef union MBcsType {
	volatile uint32_t w;
	struct {
		volatile uint32_t time_stamp : 16;
		volatile uint32_t dlc : 4;
		volatile uint32_t rtr : 1;
		volatile uint32_t ide : 1;
		volatile uint32_t srr : 1;
		volatile uint32_t res : 1;
		volatile uint32_t code : 4;
		volatile uint32_t res2 : 4;
	};
} MBcsType;

typedef union FIFOcsType {
	volatile uint32_t cs;
	struct {
		volatile uint32_t time_stamp : 16;
		volatile uint32_t dlc : 4;
		volatile uint32_t rtr : 1;
		volatile uint32_t ide : 1;
		volatile uint32_t srr : 1;
		volatile uint32_t res : 9;
	};
} FIFOcsType;

typedef union IDType {
	volatile uint32_t w;
	struct {
		volatile uint32_t ext : 29;
		volatile uint32_t resex : 3;
	};
	struct {
		volatile uint32_t res : 18;
		volatile uint32_t std : 11;
		volatile uint32_t resstd : 3;
	};
} IDType;

typedef union FilterType {
	volatile uint32_t w;
	struct {
		volatile uint32_t res : 1; // Bit 0 - Reserved
		volatile uint32_t ext : 29; // Bits 1 - 29 EID
	};
	struct {
		volatile uint32_t ress : 19; // Bits 0, 1-18 Reserved
		volatile uint32_t std : 11; // StD ID
	};
	struct {
		volatile uint32_t resc : 30; // Bits 0 - 29  Reserved
		volatile uint32_t ide : 1; // Bit 30 - EID
		volatile uint32_t rtr : 1; // Bit 31 Remote
	};
} FilterType;

typedef union DataType {
	struct {
		volatile uint32_t l;
		volatile uint32_t h;
	};
	struct {
		volatile uint32_t b3 : 8;
		volatile uint32_t b2 : 8;
		volatile uint32_t b1 : 8;
		volatile uint32_t b0 : 8;
		volatile uint32_t b7 : 8;
		volatile uint32_t b6 : 8;
		volatile uint32_t b5 : 8;
		volatile uint32_t b4 : 8;
	};
} DataType;


typedef struct MessageBufferType {
	MBcsType CS;
	IDType ID;
	DataType data;
} MessageBufferType;

enum mb_code_tx {
	TxMbInactive = 0x8,     /*!< MB is not active.*/
	TxMbAbort = 0x9,        /*!< MB is aborted.*/
	TxMbDataOrRemote = 0xC, /*!< MB is a TX Data Frame(when MB RTR = 0) or */
	/*!< MB is a TX Remote Request Frame (when MB RTR = 1).*/
	TxMbTanswer = 0xE,      /*!< MB is a TX Response Request Frame from */
	/*!  an incoming Remote Request Frame.*/
	TxMbNotUsed = 0xF,      /*!< Not used.*/
};

typedef struct RxFiFoType {
	FIFOcsType CS;
	IDType ID;
	DataType data;
} RxFiFoType;

enum mb_code_rx {
	kRxMbInactive = 0x0, /*!< MB is not active.*/
	kRxMbFull = 0x2,     /*!< MB is full.*/
	kRxMbEmpty = 0x4,    /*!< MB is active and empty.*/
	kRxMbOverrun = 0x6,  /*!< MB is overwritten into a full buffer.*/
	kRxMbBusy = 0x8,     /*!< FlexCAN is updating the contents of the MB.*/
	/*!  The CPU must not access the MB.*/
	kRxMbRanswer = 0xA,  /*!< A frame was configured to recognize a Remote Request Frame */
	/*!  and transmit a Response Frame in return.*/
	kRxMbNotUsed = 0xF,  /*!< Not used.*/
};

typedef struct CanType {
	volatile uint32_t MCR;                  /*!< Module Configuration Register,       Address offset: 0x0000        */
	volatile uint32_t CTRL1;                /*!< Control 1 Register,                  Address offset: 0x0004        */
	volatile uint32_t TIMER;                /*!< Free Running Timer,                  Address offset: 0x0008        */
	uint32_t Reserved0;                     /*!< Reserved                             Address offset: 0x000c        */
	volatile uint32_t RXMGMASK;             /*!< Rx Mailboxes Global Mask Register,   Address offset: 0x0010        */
	volatile uint32_t RX14MASK;             /*!< Rx 14 Mask Register,                 Address offset: 0x0014        */
	volatile uint32_t RX15MASK;             /*!< Rx 15 Mask Register,                 Address offset: 0x0018        */
	volatile uint32_t ECR;                  /*!< Error Counter,                       Address offset: 0x001c        */
	volatile uint32_t ESR1;                 /*!< Error and Status 1 Register,         Address offset: 0x0020        */
	uint32_t Reserved1;                     /*!< Reserved                             Address offset: 0x0024        */
	volatile uint32_t IMASK1;               /*!< Interrupt Masks 1 Register,          Address offset: 0x0028        */
	uint32_t Reserved2;                     /*!< Reserved                             Address offset: 0x002C        */
	volatile uint32_t IFLAG1;               /*!< Interrupt Flags 1 Register,          Address offset: 0x0030        */
	volatile uint32_t CTRL2;                /*!< Control 2 Register,                  Address offset: 0x0034        */
	volatile uint32_t ESR2;                 /*!< Error and Status 2 Register,         Address offset: 0x0038        */
	uint32_t Reserved3;                     /*!< Reserved                             Address offset: 0x003c        */
	uint32_t Reserved4;                     /*!< Reserved                             Address offset: 0x0040        */
	volatile uint32_t CRCR;                 /*!< CRC Register,                        Address offset: 0x0044        */
	volatile uint32_t RXFGMASK;             /*!< Rx FIFO Global Mask Register,        Address offset: 0x0048        */
	volatile uint32_t RXFIR;                /*!< Rx FIFO Information Register,        Address offset: 0x004c        */
	uint32_t RESERVED5[12];                 /*!< Reserved                             Address offset: 0x0050        */
	union {
		RxFiFoType fifo;
		MessageBufferType mb;
	} MB[HWMaxMB];
	uint32_t RESERVED6[448];                /*!< Reserved                             Address offset: 0x0180        */
	volatile FilterType RXIMR[HWMaxMB];         /*!< R0 Individual Mask Registers,        Address offset: 0x0880
                                                          */
} CanType;

/* Layout of Fifo, filters and Message buffers  */

enum { FiFo = 0 };
/* 0                       */
/* 1                       */
/* 2                       */
/* 3         Fifo          */
/* 4                       */
/* 5                       */
enum { FirstFilter = 6 };
/* 6                       */
/* 7                       */
/* 8         Filters       */
/* 9                       */
enum { NumHWFilters = 16 };
enum { NumMBinFiFoAndFilters = 10 };
/* 10                      */
/* 11                      */
/* 12                      */
/* 13 Tx Message Buffers   */
/* 14                      */
/* 15                      */
/*-- ----------------------*/

enum { TXMBMask = (0b111111 << NumMBinFiFoAndFilters)  };


#define CAN_IFLAG1_0    CAN_IFLAG1(0)             /* Bit 0: Buffer MB0 Interrupt */
#define CAN_IFLAG1_1    CAN_IFLAG1(1)             /* Bit 1: Buffer MB1 Interrupt */
#define CAN_IFLAG1_2    CAN_IFLAG1(2)             /* Bit 2: Buffer MB2 Interrupt */
#define CAN_IFLAG1_3    CAN_IFLAG1(3)             /* Bit 3: Buffer MB3 Interrupt */
#define CAN_IFLAG1_4    CAN_IFLAG1(4)             /* Bit 4: Buffer MB4 Interrupt */
#define CAN_IFLAG1_5    CAN_IFLAG1(5)             /* Bit 5: Buffer MB5 Interrupt */
#define CAN_FIFO_NE     CAN_IFLAG1_5              /* Bit 5: Fifo almost Not empty Interrupt */
#define CAN_IFLAG1_6    CAN_IFLAG1(6)             /* Bit 6: Buffer MB6 Interrupt */
#define CAN_FIFO_WARN   CAN_IFLAG1_6              /* Bit 6: Fifo almost full Interrupt */
#define CAN_IFLAG1_7    CAN_IFLAG1(7)             /* Bit 7: Buffer MB7 Interrupt */
#define CAN_FIFO_OV     CAN_IFLAG1_7              /* Bit 7: Fifo Overflowed Interrupt */
