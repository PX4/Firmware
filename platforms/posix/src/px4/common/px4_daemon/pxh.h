/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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
 * @file pxh.h
 *
 * The POSIX PX4 implementation features a simple shell to start modules
 * or use system commands.
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 * @author Roman Bapst <bapstroman@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */
#pragma once

#include <vector>
#include <string>
#include <termios.h>

#include <platforms/posix/apps.h>
#include "history.h"

namespace px4_daemon
{


class Pxh
{
public:
	Pxh();
	~Pxh();

	/**
	 * Process and run one command line.
	 *
	 * @param silently_fail: don't make a fuss on failure
	 * @return 0 if successful. */
	static int process_line(const std::string &line, bool silently_fail);

	/**
	 * Run the pxh shell. This will only return if stop() is called.
	 */
	void run_pxh();

	/**
	 * Can be called to stop pxh.
	 */
	static void stop();

private:
	void _print_prompt();
	void _move_cursor(int position);
	void _clear_line();

	void _setup_term();
	static void _restore_term();

	bool _should_exit{false};
	History _history;
	struct termios _orig_term {};

	static apps_map_type _apps;
	static Pxh *_instance;
};



} // namespace px4_daemon

