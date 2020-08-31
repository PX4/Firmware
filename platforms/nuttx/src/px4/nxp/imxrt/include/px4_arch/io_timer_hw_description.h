/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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


#include <px4_arch/io_timer.h>
#include <px4_arch/hw_description.h>
#include <px4_platform_common/constexpr_util.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform/io_timer_init.h>

#include <hardware/imxrt_tmr.h>
#include <hardware/imxrt_flexpwm.h>
#include <imxrt_gpio.h>
#include <imxrt_iomuxc.h>
#include <hardware/imxrt_pinmux.h>

#include <board_config.h>
#ifndef CONFIG_ARCH_CHIP_MIMXRT1062DVL6A
# error "This code has only been validated with IMXRT1062. Make sure it is correct before using it on another board."
#endif


static inline constexpr timer_io_channels_t initIOTimerChannel(const io_timers_t io_timers_conf[MAX_IO_TIMERS],
		PWM::FlexPWMConfig pwm_config, IOMUX::Pad pad)
{
	timer_io_channels_t ret{};

	PWM::FlexPWM pwm{};

	// FlexPWM Muxing Options
	switch (pwm_config.module) {
	case PWM::PWM1_PWM_A:
		pwm = PWM::FlexPWM1;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_EMC_23) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_23_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN23;

			} else if (pad == IOMUX::Pad::GPIO_SD_B0_00) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_SD_B0_00_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN12;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_25) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_25_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN25;

			} else if (pad == IOMUX::Pad::GPIO_SD_B0_02) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_SD_B0_02_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN14;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_27) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_27_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN27;

			} else if (pad == IOMUX::Pad::GPIO_SD_B0_04) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_SD_B0_04_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN16;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_EMC_38) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_38_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN24;

			} else if (pad == IOMUX::Pad::GPIO_SD_B1_00) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT2 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_SD_B1_00_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN0;

			} else if (pad == IOMUX::Pad::GPIO_AD_B0_10) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_B0_10_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN10;

			} else if (pad == IOMUX::Pad::GPIO_EMC_12) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_12_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN12;

			} else if (pad == IOMUX::Pad::GPIO_B1_00) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT6 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_B1_00_INDEX);
				ret.gpio_portpin = GPIO_PORT2 | GPIO_PIN16;
			}

			break;
		}

		break;

	case PWM::PWM1_PWM_B:
		pwm = PWM::FlexPWM1;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_EMC_24) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_24_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN24;

			} else if (pad == IOMUX::Pad::GPIO_SD_B0_01) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_SD_B0_01_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN13;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_26) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_26_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN26;

			} else if (pad == IOMUX::Pad::GPIO_SD_B0_03) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_SD_B0_03_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN15;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_28) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_28_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN28;

			} else if (pad == IOMUX::Pad::GPIO_SD_B0_05) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_SD_B0_05_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN17;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_EMC_39) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_39_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN25;

			} else if (pad == IOMUX::Pad::GPIO_SD_B1_01) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT2 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_SD_B1_01_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN1;

			} else if (pad == IOMUX::Pad::GPIO_AD_B0_11) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_B0_11_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN11;

			} else if (pad == IOMUX::Pad::GPIO_EMC_13) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_13_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN13;

			} else if (pad == IOMUX::Pad::GPIO_B1_01) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT6 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_B1_01_INDEX);
				ret.gpio_portpin = GPIO_PORT2 | GPIO_PIN17;
			}

			break;
		}

		break;

	case PWM::PWM1_PWM_X:
		pwm = PWM::FlexPWM1;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_AD_B0_02) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_B0_02_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN2;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_AD_B0_03) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_B0_03_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN3;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_AD_B0_12) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_B0_12_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN12;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_AD_B0_13) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_B0_13_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN13;
			}

			break;
		}

		break;


	case PWM::PWM2_PWM_A:
		pwm = PWM::FlexPWM2;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_B0_06) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT2 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_B0_06_INDEX);
				ret.gpio_portpin = GPIO_PORT2 | GPIO_PIN6;

			} else if (pad == IOMUX::Pad::GPIO_EMC_06) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_06_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN6;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_08) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_08_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN8;

			} else if (pad == IOMUX::Pad::GPIO_B0_08) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT2 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_B0_08_INDEX);
				ret.gpio_portpin = GPIO_PORT2 | GPIO_PIN8;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_10) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_10_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN10;

			} else if (pad == IOMUX::Pad::GPIO_B0_10) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT2 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_B0_10_INDEX);
				ret.gpio_portpin = GPIO_PORT2 | GPIO_PIN10;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_AD_B0_09) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_B0_09_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN9;

			} else if (pad == IOMUX::Pad::GPIO_SD_B1_02) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT2 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_SD_B1_02_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN2;

			} else if (pad == IOMUX::Pad::GPIO_EMC_19) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_19_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN19;

			} else if (pad == IOMUX::Pad::GPIO_B1_02) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT6 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_B1_02_INDEX);
				ret.gpio_portpin = GPIO_PORT2 | GPIO_PIN18;

			} else if (pad == IOMUX::Pad::GPIO_AD_B0_00) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT0 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_B0_00_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN0;
			}

			break;
		}

		break;

	case PWM::PWM2_PWM_B:
		pwm = PWM::FlexPWM2;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_B0_07) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT2 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_B0_07_INDEX);
				ret.gpio_portpin = GPIO_PORT2 | GPIO_PIN6;

			} else if (pad == IOMUX::Pad::GPIO_EMC_07) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_07_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN6;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_09) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_09_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN8;

			} else if (pad == IOMUX::Pad::GPIO_B0_09) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT2 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_B0_09_INDEX);
				ret.gpio_portpin = GPIO_PORT2 | GPIO_PIN8;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_11) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_11_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN10;

			} else if (pad == IOMUX::Pad::GPIO_B0_11) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT2 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_B0_11_INDEX);
				ret.gpio_portpin = GPIO_PORT2 | GPIO_PIN10;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_AD_B0_01) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT0 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_B0_01_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN1;

			} else if (pad == IOMUX::Pad::GPIO_SD_B1_03) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT2 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_SD_B1_03_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN3;

			} else if (pad == IOMUX::Pad::GPIO_EMC_20) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_20_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN20;

			} else if (pad == IOMUX::Pad::GPIO_B1_03) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT6 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_B1_03_INDEX);
				ret.gpio_portpin = GPIO_PORT2 | GPIO_PIN19;
			}

			break;
		}

		break;


	case PWM::PWM3_PWM_A:
		pwm = PWM::FlexPWM3;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_EMC_29) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_29_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN29;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_31) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_31_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN31;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_33) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_33_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN19;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_EMC_21) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_21_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN21;
			}

			break;
		}

		break;

	case PWM::PWM3_PWM_B:
		pwm = PWM::FlexPWM3;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_EMC_30) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_30_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN30;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_32) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_32_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN18;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_34) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_34_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN20;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_EMC_22) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_22_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN22;
			}

			break;
		}

		break;


	case PWM::PWM4_PWM_A:
		pwm = PWM::FlexPWM4;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_AD_B1_08) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_B1_08_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN24;

			} else if (pad == IOMUX::Pad::GPIO_EMC_00) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_00_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN0;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_02) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_02_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN2;

			} else if (pad == IOMUX::Pad::GPIO_AD_B1_09) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_B1_09_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN25;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_04) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_04_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN4;

			} else if (pad == IOMUX::Pad::GPIO_B1_14) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_B1_14_INDEX);
				ret.gpio_portpin = GPIO_PORT2 | GPIO_PIN30;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_EMC_17) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_17_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN17;

			} else if (pad == IOMUX::Pad::GPIO_B1_15) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_B1_15_INDEX);
				ret.gpio_portpin = GPIO_PORT2 | GPIO_PIN31;
			}

			break;
		}

		break;

	case PWM::PWM4_PWM_B:
		pwm = PWM::FlexPWM4;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_EMC_01) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_01_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN1;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_03) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_03_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN3;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_05) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_05_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN5;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_EMC_18) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_18_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN18;
			}

			break;
		}

		break;
	}

	constexpr_assert(ret.gpio_out != 0, "Invalid PWM/Pad config");
	ret.gpio_out |= IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST;

	switch (pwm_config.module) {
	case PWM::PWM1_PWM_A:
	case PWM::PWM2_PWM_A:
	case PWM::PWM3_PWM_A:
	case PWM::PWM4_PWM_A:
		ret.val_offset = PWMA_VAL;
		break;

	case PWM::PWM1_PWM_B:
	case PWM::PWM2_PWM_B:
	case PWM::PWM3_PWM_B:
	case PWM::PWM4_PWM_B:
		ret.val_offset = PWMB_VAL;
		break;

	default:
		constexpr_assert(false, "not implemented");
	}

	switch (pwm_config.submodule) {
	case PWM::Submodule0:
		ret.sub_module = SM0;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM0);
		break;

	case PWM::Submodule1:
		ret.sub_module = SM1;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM1);
		break;

	case PWM::Submodule2:
		ret.sub_module = SM2;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM2);
		break;

	case PWM::Submodule3:
		ret.sub_module = SM3;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM3);
		break;
	}

	ret.gpio_in = 0; // TODO (not used yet)

	// find timer index
	ret.timer_index = 0xff;
	const uint32_t timer_base = getFlexPWMBaseRegister(pwm);

	for (int i = 0; i < MAX_IO_TIMERS; ++i) {
		if (io_timers_conf[i].base == timer_base) {
			ret.timer_index = i;
			break;
		}
	}

	constexpr_assert(ret.timer_index != 0xff, "Timer not found");

	return ret;
}

static inline constexpr io_timers_t initIOPWM(PWM::FlexPWM pwm)
{
	io_timers_t ret{};

	ret.base = getFlexPWMBaseRegister(pwm);

	return ret;
}
