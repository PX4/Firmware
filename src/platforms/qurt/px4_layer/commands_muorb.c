/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file commands_muorb_test.c
 * Commands to run for the "qurt_muorb_test" config
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

const char *get_commands()
{
	static const char *commands =
		"uorb start\n"
		"muorb_test start\n";

	/*
	                  "hil mode_pwm\n"
	                  "mixer load /dev/pwm_output0 /startup/quad_x.main.mix\n";
	*/
	/*
	                  "param show\n"
	                  "param set CAL_GYRO_ID 2293760\n"
	                  "param set CAL_ACC0_ID 1310720\n"
	                  "param set CAL_ACC1_ID 1376256\n"
	                  "param set CAL_MAG0_ID 196608\n"
	                  "gyrosim start\n"
	                  "accelsim start\n"
	                  "rgbled start\n"
	                  "tone_alarm start\n"
	                  "simulator start -s\n"
	                  "commander start\n"
	                  "sensors start\n"
	                  "ekf_att_pos_estimator start\n"
	                  "mc_pos_control start\n"
	                  "mc_att_control start\n"
	                  "param set MAV_TYPE 2\n"
	                  "param set RC1_MAX 2015\n"
	                  "param set RC1_MIN 996\n"
	                  "param set RC_TRIM 1502\n"
	*/

	return commands;
	/*====================================== Working set
	======================================*/

	//"muorb_test start\n"
	//"gyrosim start\n"
	//"adcsim start\n"

}
