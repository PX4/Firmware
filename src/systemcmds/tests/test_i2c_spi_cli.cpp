/****************************************************************************
 *
 *  Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <unit_test.h>
#include <px4_platform_common/i2c_spi_buses.h>

class CLIArgsHelper
{
public:
	CLIArgsHelper(const char *args[], int num_argc)
	{
		// get args into the right format: the first string is the module name, the last nullptr
		int len = 0;
		strcpy(_buf + len, "module");
		len += strlen("module") + 1;

		for (int i = 0; i < num_argc; ++i) {
			strcpy(_buf + len, args[i]);
			argv[i + 1] = _buf + len;
			len += strlen(args[i]) + 1;
		}

		argc = num_argc + 1;
		argv[argc] = nullptr;
	}

	int argc;
	char *argv[32];
private:
	char _buf[1024];
};

class I2CSPICLITest : public UnitTest
{
public:
	virtual bool run_tests();

	bool test_basic();
	bool test_invalid();
	bool test_custom();
};

bool I2CSPICLITest::run_tests()
{
	ut_run_test(test_basic);
	ut_run_test(test_invalid);
	ut_run_test(test_custom);

	return (_tests_failed == 0);
}

bool I2CSPICLITest::test_basic()
{

	{
		BusCLIArguments cli{true, true};
		cli.default_i2c_frequency = 1234;
		cli.default_spi_frequency = 12345;
		const char *argv[] = { "start", "-I" };
		CLIArgsHelper cli_args(argv, 2);
		const char *verb = cli.parseDefaultArguments(cli_args.argc, cli_args.argv);
		ut_assert_true(verb != nullptr);
		ut_assert_true(strcmp(verb, "start") == 0);
		ut_assert_true(cli.bus_option == I2CSPIBusOption::I2CInternal);
		ut_assert_true(cli.bus_frequency == cli.default_i2c_frequency);
	}

	{
		BusCLIArguments cli{true, true};
		cli.default_i2c_frequency = 1234;
		cli.default_spi_frequency = 12345;
		const char *argv[] = { "start", "-s", "-f", "10"};
		CLIArgsHelper cli_args(argv, 4);
		const char *verb = cli.parseDefaultArguments(cli_args.argc, cli_args.argv);
		ut_assert_true(verb != nullptr);
		ut_assert_true(strcmp(verb, "start") == 0);
		ut_assert_true(cli.bus_option == I2CSPIBusOption::SPIInternal);
		ut_assert_true(cli.bus_frequency ==  10000);
	}

	{
		BusCLIArguments cli{true, true};
		cli.default_i2c_frequency = 11111;
		cli.default_spi_frequency = 22222;
		const char *argv[] = { "-S", "-b", "3", "stop"};
		CLIArgsHelper cli_args(argv, 4);
		const char *verb = cli.parseDefaultArguments(cli_args.argc, cli_args.argv);
		ut_assert_true(verb != nullptr);
		ut_assert_true(strcmp(verb, "stop") == 0);
		ut_assert_true(cli.bus_option == I2CSPIBusOption::SPIExternal);
		ut_assert_true(cli.requested_bus == 3);
	}

	{
		BusCLIArguments cli{true, true};
		cli.default_i2c_frequency = 11111;
		cli.default_spi_frequency = 22222;
		cli.i2c_address = 0xab;
		const char *argv[] = { "start", "-X", "-a", "0x14"};
		CLIArgsHelper cli_args(argv, 4);
		const char *verb = cli.parseDefaultArguments(cli_args.argc, cli_args.argv);
		ut_assert_true(verb != nullptr);
		ut_assert_true(strcmp(verb, "start") == 0);
		ut_assert_true(cli.bus_option == I2CSPIBusOption::I2CExternal);
		ut_assert_true(cli.i2c_address == 0x14);
	}

	return true;
}

bool I2CSPICLITest::test_invalid()
{
	{
		// SPI disabled, but SPI option provided
		BusCLIArguments cli{true, false};
		cli.default_i2c_frequency = 11111;
		const char *argv[] = { "start", "-S"};
		CLIArgsHelper cli_args(argv, 2);
		const char *verb = cli.parseDefaultArguments(cli_args.argc, cli_args.argv);
		ut_assert_true(verb == nullptr);
	}

	{
		// Unknown argument
		BusCLIArguments cli{true, true};
		cli.default_i2c_frequency = 11111;
		cli.default_spi_frequency = 22222;
		const char *argv[] = { "start", "-I", "-x", "3"};
		CLIArgsHelper cli_args(argv, 3);
		const char *verb = cli.parseDefaultArguments(cli_args.argc, cli_args.argv);
		ut_assert_true(verb == nullptr);
	}

	{
		// default bus frequency not set
		BusCLIArguments cli{true, true};
		const char *argv[] = { "start", "-I"};
		CLIArgsHelper cli_args(argv, 2);
		const char *verb = cli.parseDefaultArguments(cli_args.argc, cli_args.argv);
		ut_assert_true(verb == nullptr);
	}

	{
		// Another unknown argument
		BusCLIArguments cli{true, true};
		cli.default_i2c_frequency = 11111;
		cli.default_spi_frequency = 22222;
		const char *argv[] = { "-x", "start" };
		CLIArgsHelper cli_args(argv, 2);
		const char *verb = cli.parseDefaultArguments(cli_args.argc, cli_args.argv);
		ut_assert_true(verb == nullptr);
	}
	return true;
}

bool I2CSPICLITest::test_custom()
{
	{
		// custom argument
		BusCLIArguments cli{true, true};
		cli.default_i2c_frequency = 11111;
		cli.default_spi_frequency = 22222;
		const char *argv[] = { "start", "-T", "432", "-a", "12"};
		CLIArgsHelper cli_args(argv, 5);
		int ch;
		int T = 0;
		int a = 0;

		while ((ch = cli.getopt(cli_args.argc, cli_args.argv, "T:a:")) != EOF) {
			switch (ch) {
			case 'T': T = atoi(cli.optarg());
				break;

			case 'a': a = atoi(cli.optarg());
				break;
			}
		}

		const char *verb = cli.optarg();
		ut_assert_true(verb != nullptr);
		ut_assert_true(strcmp(verb, "start") == 0);
		ut_assert_true(T == 432);
		ut_assert_true(a == 12);
	}

	{
		// duplicate argument
		BusCLIArguments cli{true, true};
		cli.default_i2c_frequency = 11111;
		cli.default_spi_frequency = 22222;
		const char *argv[] = { "start"};
		CLIArgsHelper cli_args(argv, 1);
		int ch;

		while ((ch = cli.getopt(cli_args.argc, cli_args.argv, "I:")) != EOF) {
			switch (ch) {
			case 'I': ut_assert_true(false); // must not get here, because 'I' is already used
				break;
			}
		}

		const char *verb = cli.optarg();
		ut_assert_true(verb == nullptr);
	}
	return true;
}

ut_declare_test_c(test_i2c_spi_cli, I2CSPICLITest)
