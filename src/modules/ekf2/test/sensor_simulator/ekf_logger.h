/****************************************************************************
 *
 *   Copyright (c) 2019 ECL Development Team. All rights reserved.
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
 * Class to write EKF state to file
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */
#ifndef EKF_EKF_LOGGER_H
#define EKF_EKF_LOGGER_H

#include "EKF/ekf.h"
#include "EKF/estimator_interface.h"
#include "ekf_wrapper.h"
#include <fstream>
#include <iostream>

class EkfLogger
{
public:
	EkfLogger(std::shared_ptr<Ekf> ekf);
	~EkfLogger() = default;
	void setFilePath(std::string file_path);

	void enableStateLogging() { _state_logging_enabled = true; };
	void disableStateLogging() { _state_logging_enabled = false; };
	void enableVarianceLogging() { _variance_logging_enabled = true; };
	void disableVarianceLogging() { _variance_logging_enabled = false; };

	void writeStateToFile();

private:
	std::shared_ptr<Ekf> _ekf;
	EkfWrapper _ekf_wrapper;

	std::string _file_path;
	std::ofstream _file;

	bool _file_opened {false};

	bool _state_logging_enabled {true};
	bool _variance_logging_enabled {true};

	void writeState();

};
#endif // !EKF_EKF_LOGGER_H
