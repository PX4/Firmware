/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * testing binary that runs the multirotor mixer through test cases given
 * via file or stdin and compares the mixer output against expected values.
 */

#include "MultirotorMixer.hpp"

#include <cstdio>
#include <math.h>

static const unsigned output_max = 16;
static float actuator_controls[output_max] {};

static int	mixer_callback(uintptr_t handle,
			       uint8_t control_group,
			       uint8_t control_index,
			       float &control);

static int
mixer_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &control)
{
	control = actuator_controls[control_index];
	return 0;
}

int main(int argc, char *argv[])
{
	FILE *file_in = stdin;

	if (argc > 1) {
		file_in = fopen(argv[1], "r");
	}

	unsigned rotor_count = 0;
	MultirotorMixer::Rotor rotors[output_max];
	float actuator_outputs[output_max];

	// read airmode
	int airmode;
	fscanf(file_in, "%i", &airmode);

	// read the motor count & control allocation matrix
	fscanf(file_in, "%i", &rotor_count);

	if (rotor_count > output_max) {
		return -1;
	}

	for (unsigned i = 0; i < rotor_count; ++i) {
		fscanf(file_in, "%f %f %f %f", &rotors[i].roll_scale, &rotors[i].pitch_scale,
		       &rotors[i].yaw_scale, &rotors[i].thrust_scale);
	}

	MultirotorMixer mixer(mixer_callback, 0, rotors, rotor_count);
	mixer.set_airmode((Mixer::Airmode)airmode);

	int test_counter = 0;
	int num_failed = 0;

	while (!feof(file_in)) {

		// read actuator controls
		unsigned count = 0;

		while (count < 4 && fscanf(file_in, "%f", &actuator_controls[count]) == 1) {
			++count;
		}

		if (count < 4) {
			break;
		}

		// do the mixing
		if (mixer.mix(actuator_outputs, output_max) != rotor_count) {
			return -1;
		}

		// Account for MultirotorMixer outputing [-1,1] and python script [0,1]
		for (unsigned i = 0; i < rotor_count; i++) {
			actuator_outputs[i] = (actuator_outputs[i] + 1.f) * .5f;
		}

		// read expected outputs
		count = 0;
		float expected_output[output_max];
		bool failed = false;

		while (count < rotor_count && fscanf(file_in, "%f", &expected_output[count]) == 1) {
			if (fabsf(expected_output[count] - actuator_outputs[count]) > 0.00001f) {
				failed = true;
			}

			++count;
		}

		if (count < rotor_count) {
			break;
		}

		if (failed) {
			printf("test %i failed:\n", test_counter + 1);
			printf("control input  : %.3f %.3f %.3f %.3f\n", (double)actuator_controls[0], (double)actuator_controls[1],
			       (double)actuator_controls[2], (double)actuator_controls[3]);
			printf("mixer output   : ");

			for (unsigned i = 0; i < rotor_count; ++i) {
				printf("%.3f ", (double)actuator_outputs[i]);
			}

			printf("\n");
			printf("expected output: ");

			for (unsigned i = 0; i < rotor_count; ++i) {
				printf("%.3f ", (double)expected_output[i]);
			}

			printf("\n");
			printf("diff           : ");

			for (unsigned i = 0; i < rotor_count; ++i) {
				printf("%.3f ", (double)(expected_output[i] - actuator_outputs[i]));
			}

			printf("\n");
			++num_failed;
		}

		++test_counter;
	}

	printf("tested %i cases: %i success, %i failed\n", test_counter,
	       test_counter - num_failed, num_failed);


	if (file_in != stdin) {
		fclose(file_in);
	}

	return num_failed > 0 ? -1 : 0;
}
