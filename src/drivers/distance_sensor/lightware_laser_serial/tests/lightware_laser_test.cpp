#include <unit_test.h>

#include "../parser.h"

#include <systemlib/err.h>

#include <cstdio>
#include <cstring>
#include <unistd.h>

extern "C" __EXPORT int lightware_laser_test_main(int argc, char *argv[]);

class LightwareLaserTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool runTest();
};

bool LightwareLaserTest::run_tests()
{
	ut_run_test(runTest);

	return (_tests_failed == 0);
}

bool LightwareLaserTest::runTest()
{
	const char _LINE_MAX = 20;
	//char _linebuf[_LINE_MAX];
	//_linebuf[0] = '\0';

	const char *lines[] = {"0.01\r\n",
			       "0.02\r\n",
			       "0.03\r\n",
			       "0.04\r\n",
			       "0",
			       ".",
			       "0",
			       "5",
			       "\r",
			       "\n",
			       "0",
			       "3\r",
			       "\n",
			       "\r\n",
			       "0.06",
			       "\r\n"
			      };

	enum LW_PARSE_STATE state = LW_PARSE_STATE0_UNSYNC;
	float dist_m;
	char _parserbuf[_LINE_MAX];
	unsigned _parsebuf_index = 0;

	for (unsigned l = 0; l < sizeof(lines) / sizeof(lines[0]); l++) {
		//printf("\n%s", _linebuf);

		int parse_ret = -1;

		for (int i = 0; i < (ssize_t)strlen(lines[l]); i++) {
			parse_ret = lightware_parser(lines[l][i], _parserbuf, &_parsebuf_index, &state, &dist_m);

			if (parse_ret == 0) {
				if (l == 0) {
					ut_test(dist_m - 0.010000f < 0.001f);

				} else if (l == 1) {
					ut_test(dist_m - 0.020000f < 0.001f);

				} else if (l == 2) {
					ut_test(dist_m - 0.030000f < 0.001f);

				} else if (l == 3) {
					ut_test(dist_m - 0.040000f < 0.001f);
				}

				//printf("\nparsed: %f %s\n", dist_m, (parse_ret == 0) ? "OK" : "");
			}
		}

		//printf("%s", lines[l]);

	}

	return true;
}

ut_declare_test_c(lightware_laser_test_main, LightwareLaserTest)
