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
 * Test code for the Velocity Smoothing library
 * Build and run using: make && ./test_velocity_smoothing
 */

#include "VelocitySmoothing.hpp"
#include <cstdio>
#include <matrix/matrix/math.hpp>

int main(int argc, char *argv[])
{
	VelocitySmoothing trajectory[3];

	float a0[3] = {0.f, 0.f, 0.f};
	float v0[3] = {0.f, 0.f, 0.f};
	float x0[3] = {0.f, 0.f, 0.f};

	float j_max = 55.2f;
	float a_max = 6.f;
	float v_max = 6.f;

	for (int i = 0; i < 3; i++) {
		trajectory[i].setMaxJerk(j_max);
		trajectory[i].setMaxAccel(a_max);
		trajectory[i].setMaxVel(v_max);
		trajectory[i].setCurrentAcceleration(a0[i]);
		trajectory[i].setCurrentVelocity(v0[i]);
	}

	const float dt = 0.01f;

	float velocity_setpoint[3] = {1.f, 0.f, -1.f};

	for (int i = 0; i < 3; i++) {
		trajectory[i].updateDurations(velocity_setpoint[i]);
	}

	float t123 = trajectory[0].getTotalTime();
	int nb_steps = ceil(t123 / dt);
	printf("Nb steps = %d\n", nb_steps);

	for (int i = 0; i < nb_steps; i++) {
		for (int i = 0; i < 3; i++) {
			trajectory[i].updateTraj(dt);
		}

		for (int i = 0; i < 3; i++) {
			trajectory[i].updateDurations(velocity_setpoint[i]);
		}

		VelocitySmoothing::timeSynchronization(trajectory, 2);

		for (int i = 0; i < 1; i++) {
			printf("Traj[%d]\n", i);
			printf("jerk = %.3f\taccel = %.3f\tvel = %.3f\tpos = %.3f\n",
			       trajectory[i].getCurrentJerk(),
			       trajectory[i].getCurrentAcceleration(),
			       trajectory[i].getCurrentVelocity(),
			       trajectory[i].getCurrentPosition());
			printf("T1 = %.3f\tT2 = %.3f\tT3 = %.3f\n", trajectory[i].getT1(), trajectory[i].getT2(), trajectory[i].getT3());
			printf("\n");
		}
	}

	return 0;
}
