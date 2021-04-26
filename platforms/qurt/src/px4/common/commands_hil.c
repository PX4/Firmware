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

const char *get_commands(void);
const char *get_commands()
{

	static const char *commands =
		"param set CAL_GYRO0_ID 2293760\n"
		"param set CAL_ACC0_ID 1310720\n"
		"param set CAL_ACC1_ID 1376256\n"
		"param set CAL_MAG0_ID 196608\n"
//		"rgbled start\n"
//		"tone_alarm start\n"
		"rc_update start\n"
		"commander start --hil\n"
		"sensors start\n"
		"ekf2 start\n"
		"mc_hover_thrust_estimator start\n"
		"flight_mode_manager start\n"
		"mc_pos_control start\n"
		"mc_att_control start\n"
		"mc_rate_control start\n"
		"sleep 1\n"
		"pwm_out_sim start\n"
		"param set RC1_MAX 2015\n"
		"param set RC1_MIN 996\n"
		"param set RC1_TRIM 1502\n"
		"param set RC1_REV -1\n"
		"param set RC2_MAX 2016 \n"
		"param set RC2_MIN 995\n"
		"param set RC2_TRIM 1500\n"
		"param set RC3_MAX 2003\n"
		"param set RC3_MIN 992\n"
		"param set RC3_TRIM 992\n"
		"param set RC4_MAX 2011\n"
		"param set RC4_MIN 997\n"
		"param set RC4_TRIM 1504\n"
		"param set RC4_REV -1\n"
		"param set RC6_MAX 2016\n"
		"param set RC6_MIN 992\n"
		"param set RC6_TRIM 1504\n"
		"param set RC_CHAN_CNT 8\n"
		"param set RC_MAP_MODE_SW 5\n"
		"param set RC_MAP_POSCTL_SW 7\n"
		"param set RC_MAP_RETURN_SW 8\n"
		"param set MC_YAW_P 1.5\n"
		"param set MC_PITCH_P 3.0\n"
		"param set MC_ROLL_P 3.0\n"
		"param set MC_YAWRATE_P 0.2\n"
		"param set MC_PITCHRATE_P 0.03\n"
		"param set MC_ROLLRATE_P 0.03\n"
		"param set ATT_W_ACC 0.0002\n"
		"param set ATT_W_MAG 0.002\n"
		"param set ATT_W_GYRO_BIAS 0.05\n"
		"sleep 1\n"


		"param set MAV_TYPE 2\n"
		"mixer load /dev/pwm_output0 /startup/quad_x.main.mix\n"
		"list_files\n"
		"list_tasks\n"
		"sleep 10\n"
		"list_tasks\n"
		"sleep 10\n"

		;

	return commands;

}
