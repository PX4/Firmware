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

#pragma once

#include "defines.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/perf/perf_counter.h>

class TCBP001TA : public px4::ScheduledWorkItem
{
public:
	TCBP001TA(tcbp001ta::ITCBP001TA *interface);
	virtual ~TCBP001TA();

	int			init();
	void			print_info();

	uint32_t                tmp_osr_scale_coeff;
	uint32_t                prs_osr_scale_coeff;

	uint32_t                TCBP001TA_get_scaling_coef(uint8_t osr);

private:
	void			Run() override;
	void			Start();
	void			Stop();

	int			measure(); //start measure
	int			collect(); //get results and publish

	PX4Barometer		_px4_baro;

	tcbp001ta::ITCBP001TA	*_interface;

	// set config, recommended settings
	//static constexpr uint8_t	_curr_ctrl{TCBP001TA_CTRL_P16 | TCBP001TA_CTRL_T2};
	static constexpr uint32_t	_measure_interval{TCBP001TA_MT_INIT + TCBP001TA_MT *(16 - 1 + 2 - 1)};

	bool			_collect_phase{false};

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	tcbp001ta::calibration_s	*_cal{nullptr}; //stored calibration constants
	tcbp001ta::fcalibration_s	_fcal{}; //pre processed calibration constants
};
