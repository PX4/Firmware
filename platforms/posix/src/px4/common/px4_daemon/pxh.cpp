/****************************************************************************
 *
 *   Copyright (C) 2015-2016 PX4 Development Team. All rights reserved.
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
 * @file pxh.cpp
 *
 * This is a simple PX4 shell implementation used to start modules.
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 * @author Roman Bapst <bapstroman@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */

#include <string>
#include <sstream>
#include <vector>
#include <stdio.h>

#include "pxh.h"

namespace px4_daemon
{

Pxh *Pxh::_instance = nullptr;

apps_map_type Pxh::_apps = {};


Pxh::Pxh()
{
	_history.try_to_add("commander takeoff"); // for convenience
	_history.reset_to_end();
	_instance = this;
}

Pxh::~Pxh()
{
	_instance = nullptr;
}

int Pxh::process_line(const std::string &line, bool silently_fail)
{
	if (line.empty()) {
		return 0;
	}

	if (_apps.empty()) {
		init_app_map(_apps);
	}

	std::stringstream line_stream(line);
	std::string word;
	std::vector<std::string> words;

	// First arg should be the command.
	while (line_stream >> word) {
		words.push_back(word);
	}

	if (words.empty()) {
		return 0;
	}

	const std::string &command(words.front());

	if (_apps.find(command) != _apps.end()) {

		// Note that argv[argc] always needs to be a nullptr.
		// Therefore add one more entry.
		const char *arg[words.size() + 1];

		for (unsigned i = 0; i < words.size(); ++i) {
			arg[i] = (char *)words[i].c_str();
		}

		// Explicitly set this nullptr.
		arg[words.size()] = nullptr;

		int retval = _apps[command](words.size(), (char **)arg);

		if (retval) {
			if (!silently_fail) {
				printf("Command '%s' failed, returned %d.\n", command.c_str(), retval);
			}
		}

		return retval;

	} else if (command == "help") {
		list_builtins(_apps);
		return 0;

	} else if (command.length() == 0 || command[0] == '#') {
		// Do nothing
		return 0;

	} else if (!silently_fail) {
		//std::cout << "Invalid command: " << command << "\ntype 'help' for a list of commands" << endl;
		printf("Invalid command: %s\ntype 'help' for a list of commands\n", command.c_str());
		return -1;

	} else {
		return -1;
	}
}


void Pxh::run_pxh()
{
	_should_exit = false;

	_setup_term();

	std::string mystr;
	int cursor_position = 0; // position of the cursor from right to left
	// (0: all the way to the right, mystr.length: all the way to the left)

	_print_prompt();

	while (!_should_exit) {

		int c = getchar();
		std::string add_string; // string to add at current cursor position
		bool update_prompt = true;

		switch (c) {
		case EOF:
			_should_exit = true;
			break;

		case 127:	// backslash
			if ((int)mystr.length() - cursor_position > 0) {
				mystr.erase(mystr.length() - cursor_position - 1, 1);
			}

			break;

		case '\n':	// user hit enter
			_history.try_to_add(mystr);
			_history.reset_to_end();

			printf("\n");
			process_line(mystr, false);
			// reset string and cursor position
			mystr = "";
			cursor_position = 0;

			update_prompt = false;
			_print_prompt();
			break;

		case '\033': {	// arrow keys
				c = getchar();	// skip first one, does not have the info
				c = getchar();

				if (c == 'A') { // arrow up
					_history.try_to_save_current_line(mystr);
					_history.get_previous(mystr);
					cursor_position = 0; // move cursor to end of line

				} else if (c == 'B') { // arrow down
					_history.get_next(mystr);
					cursor_position = 0; // move cursor to end of line

				} else if (c == 'C') { // arrow right
					if (cursor_position > 0) {
						cursor_position--;
					}

				} else if (c == 'D') { // arrow left
					if (cursor_position < (int)mystr.length()) {
						cursor_position++;
					}

				} else if (c == 'H') { // Home (go to the beginning of the command)
					cursor_position = mystr.length();

				} else if (c == '1') { // Home (go to the beginning of the command, Editing key)
					(void)getchar(); // swallow '~'
					cursor_position = mystr.length();

				} else if (c == 'F') { // End (go to the end of the command)
					cursor_position = 0;

				} else if (c == '4') { // End (go to the end of the command, Editing key)
					(void)getchar(); // swallow '~'
					cursor_position = 0;
				}

				break;
			}

		default:	// any other input
			if (c > 3) {
				add_string += (char)c;

			} else {
				update_prompt = false;
			}

			break;
		}


		if (update_prompt) {
			// reprint prompt with mystr
			mystr.insert(mystr.length() - cursor_position, add_string);
			_clear_line();
			_print_prompt();
			printf("%s", mystr.c_str());

			// Move the cursor to its position
			if (cursor_position > 0) {
				_move_cursor(cursor_position);
			}
		}

	}
}

void Pxh::stop()
{
	_restore_term();

	if (_instance) {
		_instance->_should_exit = true;
	}
}

void Pxh::_setup_term()
{
	// Make sure we restore terminal at exit.
	tcgetattr(0, &_orig_term);
	atexit(Pxh::_restore_term);

	// change input mode so that we can manage shell
	struct termios term;
	tcgetattr(0, &term);
	term.c_lflag &= ~ICANON;
	term.c_lflag &= ~ECHO;
	tcsetattr(0, TCSANOW, &term);
	setbuf(stdin, nullptr);
}

void Pxh::_restore_term()
{
	if (_instance) {
		tcsetattr(0, TCSANOW, &_instance->_orig_term);
	}
}

void Pxh::_print_prompt()
{
	fflush(stdout);
	printf("pxh> ");
	fflush(stdout);
}

void Pxh::_clear_line()
{
	printf("%c[2K%c", (char)27, (char)13);
}
void Pxh::_move_cursor(int position)
{
	printf("\033[%dD", position);
}

} // namespace px4_daemon
