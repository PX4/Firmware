/****************************************************************************
 * config/imxrt1060-evk/src/imxrt_flexspi_nor_flash.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
 *            David Sidrane <david_s5@nscdg.com>
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
 ****************************************************************************/

#ifndef __CONFIGS_IMXRT1060_EVK_SRC_IMXRT_FLEXSPI_NOR_FLASH_H
#define __CONFIGS_IMXRT1060_EVK_SRC_IMXRT_FLEXSPI_NOR_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FLEXSPI memory config block related defintions */

#define FLEXSPI_CFG_BLK_TAG         (0x42464346ul)
#define FLEXSPI_CFG_BLK_VERSION     (0x56010400ul)
#define FLEXSPI_CFG_BLK_SIZE        (512)

/* FLEXSPI Feature related definitions */

#define FLEXSPI_FEATURE_HAS_PARALLEL_MODE 1

/* Lookup table related defintions */

#define CMD_INDEX_READ              0
#define CMD_INDEX_READSTATUS        1
#define CMD_INDEX_WRITEENABLE       2
#define CMD_INDEX_WRITE             4

#define CMD_LUT_SEQ_IDX_READ        0
#define CMD_LUT_SEQ_IDX_READSTATUS  1
#define CMD_LUT_SEQ_IDX_WRITEENABLE 3
#define CMD_LUT_SEQ_IDX_WRITE       9

#define CMD_SDR                     0x01
#define CMD_DDR                     0x21
#define RADDR_SDR                   0x02
#define RADDR_DDR                   0x22
#define CADDR_SDR                   0x03
#define CADDR_DDR                   0x23
#define MODE1_SDR                   0x04
#define MODE1_DDR                   0x24
#define MODE2_SDR                   0x05
#define MODE2_DDR                   0x25
#define MODE4_SDR                   0x06
#define MODE4_DDR                   0x26
#define MODE8_SDR                   0x07
#define MODE8_DDR                   0x27
#define WRITE_SDR                   0x08
#define WRITE_DDR                   0x28
#define READ_SDR                    0x09
#define READ_DDR                    0x29
#define LEARN_SDR                   0x0a
#define LEARN_DDR                   0x2a
#define DATSZ_SDR                   0x0b
#define DATSZ_DDR                   0x2b
#define DUMMY_SDR                   0x0c
#define DUMMY_DDR                   0x2c
#define DUMMY_RWDS_SDR              0x0d
#define DUMMY_RWDS_DDR              0x2d
#define JMP_ON_CS                   0x1f
#define STOP                        0

#define FLEXSPI_1PAD                0
#define FLEXSPI_2PAD                1
#define FLEXSPI_4PAD                2
#define FLEXSPI_8PAD                3

#define FLEXSPI_LUT_OPERAND0_MASK   (0xffu)
#define FLEXSPI_LUT_OPERAND0_SHIFT  (0U)
#define FLEXSPI_LUT_OPERAND0(x)     (((uint32_t) \
				      (((uint32_t)(x)) << FLEXSPI_LUT_OPERAND0_SHIFT)) & \
				     FLEXSPI_LUT_OPERAND0_MASK)
#define FLEXSPI_LUT_NUM_PADS0_MASK  (0x300u)
#define FLEXSPI_LUT_NUM_PADS0_SHIFT (8u)
#define FLEXSPI_LUT_NUM_PADS0(x)    (((uint32_t) \
				      (((uint32_t)(x)) << FLEXSPI_LUT_NUM_PADS0_SHIFT)) & \
				     FLEXSPI_LUT_NUM_PADS0_MASK)
#define FLEXSPI_LUT_OPCODE0_MASK    (0xfc00u)
#define FLEXSPI_LUT_OPCODE0_SHIFT   (10u)
#define FLEXSPI_LUT_OPCODE0(x)      (((uint32_t) \
				      (((uint32_t)(x)) << FLEXSPI_LUT_OPCODE0_SHIFT)) & \
				     FLEXSPI_LUT_OPCODE0_MASK)
#define FLEXSPI_LUT_OPERAND1_MASK   (0xff0000u)
#define FLEXSPI_LUT_OPERAND1_SHIFT  (16U)
#define FLEXSPI_LUT_OPERAND1(x)     (((uint32_t) \
				      (((uint32_t)(x)) << FLEXSPI_LUT_OPERAND1_SHIFT)) & \
				     FLEXSPI_LUT_OPERAND1_MASK)
#define FLEXSPI_LUT_NUM_PADS1_MASK  (0x3000000u)
#define FLEXSPI_LUT_NUM_PADS1_SHIFT (24u)
#define FLEXSPI_LUT_NUM_PADS1(x)    (((uint32_t) \
				      (((uint32_t)(x)) << FLEXSPI_LUT_NUM_PADS1_SHIFT)) & \
				     FLEXSPI_LUT_NUM_PADS1_MASK)
#define FLEXSPI_LUT_OPCODE1_MASK    (0xfc000000u)
#define FLEXSPI_LUT_OPCODE1_SHIFT   (26u)
#define FLEXSPI_LUT_OPCODE1(x)      (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_OPCODE1_SHIFT)) & \
				     FLEXSPI_LUT_OPCODE1_MASK)

#define FLEXSPI_LUT_SEQ(cmd0, pad0, op0, cmd1, pad1, op1)  \
	(FLEXSPI_LUT_OPERAND0(op0) | FLEXSPI_LUT_NUM_PADS0(pad0) | \
	 FLEXSPI_LUT_OPCODE0(cmd0) | FLEXSPI_LUT_OPERAND1(op1) | \
	 FLEXSPI_LUT_NUM_PADS1(pad1) | FLEXSPI_LUT_OPCODE1(cmd1))

/*  */

#define NOR_CMD_INDEX_READ          CMD_INDEX_READ
#define NOR_CMD_INDEX_READSTATUS    CMD_INDEX_READSTATUS
#define NOR_CMD_INDEX_WRITEENABLE   CMD_INDEX_WRITEENABLE
#define NOR_CMD_INDEX_ERASESECTOR   3
#define NOR_CMD_INDEX_PAGEPROGRAM   CMD_INDEX_WRITE
#define NOR_CMD_INDEX_CHIPERASE     5
#define NOR_CMD_INDEX_DUMMY         6
#define NOR_CMD_INDEX_ERASEBLOCK    7

/*  READ LUT sequence id in lookupTable stored in config block */

#define NOR_CMD_LUT_SEQ_IDX_READ    CMD_LUT_SEQ_IDX_READ

/* Read Status LUT sequence id in lookupTable stored in config block */

#define NOR_CMD_LUT_SEQ_IDX_READSTATUS CMD_LUT_SEQ_IDX_READSTATUS

/* 2  Read status DPI/QPI/OPI sequence id in lookupTable stored in config block */

#define NOR_CMD_LUT_SEQ_IDX_READSTATUS_XPI 2

/* 3  Write Enable sequence id in lookupTable stored in config block */

#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE CMD_LUT_SEQ_IDX_WRITEENABLE

/* 4  Write Enable DPI/QPI/OPI sequence id in lookupTable stored in config block */

#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE_XPI 4

/* 5  Erase Sector sequence id in lookupTable stored in config block */

#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR 5

/* 8 Erase Block sequence id in lookupTable stored in config block */

#define NOR_CMD_LUT_SEQ_IDX_ERASEBLOCK 8

/* 9  Program sequence id in lookupTable stored in config block */

#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM CMD_LUT_SEQ_IDX_WRITE

/* 11 Chip Erase sequence in lookupTable id stored in config block */

#define NOR_CMD_LUT_SEQ_IDX_CHIPERASE 11

/* 13 Read SFDP sequence in lookupTable id stored in config block */

#define NOR_CMD_LUT_SEQ_IDX_READ_SFDP 13

/* 14 Restore 0-4-4/0-8-8 mode sequence id in lookupTable stored in config block */

#define NOR_CMD_LUT_SEQ_IDX_RESTORE_NOCMD 14

/* 15 Exit 0-4-4/0-8-8 mode sequence id in lookupTable stored in config blobk */

#define NOR_CMD_LUT_SEQ_IDX_EXIT_NOCMD 15

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Definitions for FlexSPI Serial Clock Frequency */

enum flexspi_serial_clkfreq_e {
	FLEXSPI_SERIAL_CLKFREQ_30MHz  = 1,
	FLEXSPI_SERIAL_CLKFREQ_50MHz  = 2,
	FLEXSPI_SERIAL_CLKFREQ_60MHz  = 3,
	FLEXSPI_SERIAL_CLKFREQ_75MHz  = 4,
	FLEXSPI_SERIAL_CLKFREQ_80MHz  = 5,
	FLEXSPI_SERIAL_CLKFREQ_100MHz = 6,
	FLEXSPI_SERIAL_CLKFREQ_133MHz = 7,
	FLEXSPI_SERIAL_CLKFREQ_166MHz = 8,
	FLEXSPI_SERIAL_CLKFREQ_200MHz = 9,
};

/* FlexSPI clock configuration type*/

enum flexspi_serial_clockmode_e {
	FLEXSPI_CLKMODE_SDR,
	FLEXSPI_CLKMODE_DDR,
};

/* FlexSPI Read Sample Clock Source definition */

enum flash_read_sample_clk_e {
	FLASH_READ_SAMPLE_CLK_LOOPBACK_INTERNELLY = 0,
	FLASH_READ_SAMPLE_CLK_LOOPBACK_FROM_DQSPAD = 1,
	FLASH_READ_SAMPLE_CLK_LOOPBACK_FROM_SCKPAD = 2,
	FLASH_READ_SAMPLE_CLK_EXTERNALINPUT_FROM_DQSPAD = 3,
};

/*  Misc feature bit definitions */

enum flash_misc_feature_e {
	FLEXSPIMISC_OFFSET_DIFFCLKEN = 0,                /* Bit for Differential clock enable */
	FLEXSPIMISC_OFFSET_CK2EN = 1,                    /* Bit for CK2 enable */
	FLEXSPIMISC_OFFSET_PARALLELEN = 2,               /* Bit for Parallel mode enable */
	FLEXSPIMISC_OFFSET_WORD_ADDRESSABLE_EN = 3,      /* Bit for Word Addressable enable */
	FLEXSPIMISC_OFFSET_SAFECONFIG_FREQ_EN = 4,       /* Bit for Safe Configuration Frequency enable */
	FLEXSPIMISC_OFFSET_PAD_SETTING_OVERRIDE_EN = 5,  /* Bit for Pad setting override enable */
	FLEXSPIMISC_OFFSET_DDR_MODE_EN = 6,              /* Bit for DDR clock confiuration indication. */
};

/* Flash Type Definition */

enum flash_flash_type_e {
	FLEXSPI_DEVICE_TYPE_SERIAL_NOR = 1,              /* Flash devices are Serial NOR */
	FLEXSPI_DEVICE_TYPE_SERIAL_NAND = 2,             /* Flash devices are Serial NAND */
	FLEXSPI_DEVICE_TYPE_SERIAL_RAM = 3,              /* Flash devices are Serial RAM/HyperFLASH */
	FLEXSPI_DEVICE_TYPE_MCP_NOR_NAND = 0x12,         /* Flash device is MCP device, A1 is Serial NOR, A2 is Serial NAND */
	FLEXSPI_DEVICE_TYPE_MCP_NOR_RAM = 0x13,          /* Flash deivce is MCP device, A1 is Serial NOR, A2 is Serial RAMs */
};

/* Flash Pad Definitions */

enum flash_flash_pad_e {
	SERIAL_FLASH_1PAD  = 1,
	SERIAL_FLASH_2PADS = 2,
	SERIAL_FLASH_4PADS = 4,
	SERIAL_FLASH_8PADS = 8,
};

/* Flash Configuration Command Type */

enum flash_config_cmd_e {
	DEVICE_CONFIG_CMD_TYPE_GENERIC,     /* Generic command, for example: configure dummy cycles, drive strength, etc */
	DEVICE_CONFIG_CMD_TYPE_QUADENABLE,  /* Quad Enable command */
	DEVICE_CONFIG_CMD_TYPE_SPI2XPI,     /* Switch from SPI to DPI/QPI/OPI mode */
	DEVICE_CONFIG_CMD_TYPE_XPI2SPI,     /* Switch from DPI/QPI/OPI to SPI mode */
	DEVICE_CONFIG_CMD_TYPE_SPI2NO_CMD,  /* Switch to 0-4-4/0-8-8 mode */
	DEVICE_CONFIG_CMD_TYPE_RESET,       /* Reset device command */
};

/* FlexSPI LUT Sequence structure */

struct flexspi_lut_seq_s {
	uint8_t seq_num;                    /* Sequence Number, valid number: 1-16 */
	uint8_t seq_id;                     /* Sequence Index, valid number: 0-15 */
	uint16_t reserved;
};

/* FlexSPI Memory Configuration Block */

struct flexspi_mem_config_s {
	uint32_t tag;
	uint32_t version;
	uint32_t reserved0;
	uint8_t read_sample_clksrc;
	uint8_t cs_hold_time;
	uint8_t cs_setup_time;
	uint8_t column_address_width;       /* [0x00f-0x00f] Column Address with, for
                                       * HyperBus protocol, it is fixed to 3, For
                                       * Serial NAND, need to refer to datasheet */
	uint8_t device_mode_cfg_enable;
	uint8_t device_mode_type;
	uint16_t wait_time_cfg_commands;
	struct flexspi_lut_seq_s device_mode_seq;
	uint32_t device_mode_arg;
	uint8_t config_cmd_enable;
	uint8_t config_mode_type[3];
	struct flexspi_lut_seq_s config_cmd_seqs[3];
	uint32_t reserved1;
	uint32_t config_cmd_args[3];
	uint32_t reserved2;
	uint32_t controller_misc_option;
	uint8_t device_type;
	uint8_t sflash_pad_type;
	uint8_t serial_clk_freq;
	uint8_t lut_custom_seq_enable;
	uint32_t reserved3[2];
	uint32_t sflash_a1size;
	uint32_t sflash_a2size;
	uint32_t sflash_b1size;
	uint32_t sflash_b2size;
	uint32_t cspad_setting_override;
	uint32_t sclkpad_setting_override;
	uint32_t datapad_setting_override;
	uint32_t dqspad_setting_override;
	uint32_t timeout_in_ms;
	uint32_t command_interval;
	uint16_t data_valid_time[2];
	uint16_t busy_offset;
	uint16_t busybit_polarity;
	uint32_t lookup_table[64];
	struct flexspi_lut_seq_s lut_customseq[12];
	uint32_t reserved4[4];
};

/* Serial NOR configuration block */

struct flexspi_nor_config_s {
	struct flexspi_mem_config_s mem_config; /* Common memory configuration info via FlexSPI */
	uint32_t page_size;                  /* Page size of Serial NOR */
	uint32_t sector_size;                /* Sector size of Serial NOR */
	uint8_t ipcmd_serial_clkfreq;        /* Clock frequency for IP command */
	uint8_t is_uniform_blocksize;        /* Sector/Block size is the same */
	uint8_t reserved0[2];                /* Reserved for future use */
	uint8_t serial_nor_type;             /* Serial NOR Flash type: 0/1/2/3 */
	uint8_t need_exit_nocmdmode;         /* Need to exit NoCmd mode before other IP command */
	uint8_t halfclk_for_nonreadcmd;      /* Half the Serial Clock for non-read command: true/false */
	uint8_t need_restore_nocmdmode;      /* Need to Restore NoCmd mode after IP commmand execution */
	uint32_t blocksize;                  /* Block size */
	uint32_t reserve2[11];               /* Reserved for future use */
};

#endif /* __CONFIGS_IMXRT1060_EVK_SRC_IMXRT_FLEXSPI_NOR_FLASH_H */
