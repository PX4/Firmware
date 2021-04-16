/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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
 * @file rc_params.c
 *
 * Parameters defined for RC.
 *
 */

/**
 * RC channel 1 minimum
 *
 * Minimum value for RC channel 1
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_MIN, 1000.0f);

/**
 * RC channel 1 trim
 *
 * Mid point value (same as min for throttle)
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_TRIM, 1500.0f);

/**
 * RC channel 1 maximum
 *
 * Maximum value for RC channel 1
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_MAX, 2000.0f);

/**
 * RC channel 1 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_REV, 1.0f);

/**
 * RC channel 1 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_DZ, 10.0f);

/**
 * RC channel 2 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_MIN, 1000.0f);

/**
 * RC channel 2 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_TRIM, 1500.0f);

/**
 * RC channel 2 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_MAX, 2000.0f);

/**
 * RC channel 2 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_REV, 1.0f);

/**
 * RC channel 2 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_DZ, 10.0f);

/**
 * RC channel 3 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_MIN, 1000);

/**
 * RC channel 3 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_TRIM, 1500);

/**
 * RC channel 3 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_MAX, 2000);

/**
 * RC channel 3 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_REV, 1.0f);

/**
 * RC channel 3 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_DZ, 10.0f);

/**
 * RC channel 4 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_MIN, 1000);

/**
 * RC channel 4 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_TRIM, 1500);

/**
 * RC channel 4 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_MAX, 2000);

/**
 * RC channel 4 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_REV, 1.0f);

/**
 * RC channel 4 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_DZ, 10.0f);

/**
 * RC channel 5 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_MIN, 1000);

/**
 * RC channel 5 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_TRIM, 1500);

/**
 * RC channel 5 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_MAX, 2000);

/**
 * RC channel 5 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_REV, 1.0f);

/**
 * RC channel 5 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_DZ,  10.0f);

/**
 * RC channel 6 minimum
 *
 * Minimum value for this channel.
 *
 * @unit us
 * @min 800.0
 * @max 1500.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_MIN, 1000);

/**
 * RC channel 6 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_TRIM, 1500);

/**
 * RC channel 6 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_MAX, 2000);

/**
 * RC channel 6 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_REV, 1.0f);

/**
 * RC channel 6 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_DZ, 10.0f);

/**
 * RC channel 7 minimum
 *
 * Minimum value for this channel.
 *
 * @unit us
 * @min 800.0
 * @max 1500.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_MIN, 1000);

/**
 * RC channel 7 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_TRIM, 1500);

/**
 * RC channel 7 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_MAX, 2000);

/**
 * RC channel 7 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_REV, 1.0f);

/**
 * RC channel 7 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_DZ, 10.0f);

/**
 * RC channel 8 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_MIN, 1000);

/**
 * RC channel 8 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_TRIM, 1500);

/**
 * RC channel 8 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_MAX, 2000);

/**
 * RC channel 8 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_REV, 1.0f);

/**
 * RC channel 8 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_DZ, 10.0f);

/**
 * RC channel 9 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_MIN, 1000);

/**
 * RC channel 9 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_TRIM, 1500);

/**
 * RC channel 9 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_MAX, 2000);

/**
 * RC channel 9 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_REV, 1.0f);

/**
 * RC channel 9 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_DZ, 0.0f);

/**
 * RC channel 10 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_MIN, 1000);

/**
 * RC channel 10 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_TRIM, 1500);

/**
 * RC channel 10 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_MAX, 2000);

/**
 * RC channel 10 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_REV, 1.0f);

/**
 * RC channel 10 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_DZ, 0.0f);

/**
 * RC channel 11 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_MIN, 1000);

/**
 * RC channel 11 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_TRIM, 1500);

/**
 * RC channel 11 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_MAX, 2000);

/**
 * RC channel 11 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_REV, 1.0f);

/**
 * RC channel 11 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_DZ, 0.0f);

/**
 * RC channel 12 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_MIN, 1000);

/**
 * RC channel 12 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_TRIM, 1500);

/**
 * RC channel 12 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_MAX, 2000);

/**
 * RC channel 12 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_REV, 1.0f);

/**
 * RC channel 12 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_DZ, 0.0f);

/**
 * RC channel 13 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_MIN, 1000);

/**
 * RC channel 13 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_TRIM, 1500);

/**
 * RC channel 13 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_MAX, 2000);

/**
 * RC channel 13 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_REV, 1.0f);

/**
 * RC channel 13 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_DZ, 0.0f);

/**
 * RC channel 14 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_MIN, 1000);

/**
 * RC channel 14 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_TRIM, 1500);

/**
 * RC channel 14 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_MAX, 2000);

/**
 * RC channel 14 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_REV, 1.0f);

/**
 * RC channel 14 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_DZ, 0.0f);

/**
 * RC channel 15 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_MIN, 1000);

/**
 * RC channel 15 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_TRIM, 1500);

/**
 * RC channel 15 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_MAX, 2000);

/**
 * RC channel 15 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_REV, 1.0f);

/**
 * RC channel 15 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_DZ, 0.0f);

/**
 * RC channel 16 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_MIN, 1000);

/**
 * RC channel 16 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_TRIM, 1500);

/**
 * RC channel 16 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_MAX, 2000);

/**
 * RC channel 16 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_REV, 1.0f);

/**
 * RC channel 16 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_DZ, 0.0f);

/**
 * RC channel 17 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_MIN, 1000);

/**
 * RC channel 17 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_TRIM, 1500);

/**
 * RC channel 17 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_MAX, 2000);

/**
 * RC channel 17 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_REV, 1.0f);

/**
 * RC channel 17 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_DZ, 0.0f);

/**
 * RC channel 18 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_MIN, 1000);

/**
 * RC channel 18 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_TRIM, 1500);

/**
 * RC channel 18 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_MAX, 2000);

/**
 * RC channel 18 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_REV, 1.0f);

/**
 * RC channel 18 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_DZ, 0.0f);

/**
 * RC channel count
 *
 * This parameter is used by Ground Station software to save the number
 * of channels which were used during RC calibration. It is only meant
 * for ground station use.
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_CHAN_CNT, 0);

/**
 * Roll control channel mapping.
 *
 * The channel index (starting from 1 for channel 1) indicates
 * which channel should be used for reading roll inputs from.
 * A value of zero indicates the switch is not assigned.
 *
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_ROLL, 0);

/**
 * Pitch control channel mapping.
 *
 * The channel index (starting from 1 for channel 1) indicates
 * which channel should be used for reading pitch inputs from.
 * A value of zero indicates the switch is not assigned.
 *
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_PITCH, 0);

/**
 * Failsafe channel mapping.
 *
 * Configures which channel is used by the receiver to indicate the signal was lost.
 * Futaba receivers do report that way.
 * If 0, whichever channel is mapped to throttle is used
 * otherwise the value indicates the specific RC channel to use
 *
 * Use RC_FAILS_THR to set the threshold indicating lost signal. By default it's below
 * the expected range and hence diabled.
 *
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_FAILSAFE, 0);

/**
 * Throttle control channel mapping.
 *
 * The channel index (starting from 1 for channel 1) indicates
 * which channel should be used for reading throttle inputs from.
 * A value of zero indicates the switch is not assigned.
 *
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_THROTTLE, 0);

/**
 * Yaw control channel mapping.
 *
 * The channel index (starting from 1 for channel 1) indicates
 * which channel should be used for reading yaw inputs from.
 * A value of zero indicates the switch is not assigned.
 *
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_YAW, 0);

/**
 * Single channel flight mode selection
 *
 * If this parameter is non-zero, flight modes are only selected
 * by this channel and are assigned to six slots.
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_FLTMODE, 0);

/**
 * Return switch channel
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_RETURN_SW, 0);

/**
 * Loiter switch channel
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_LOITER_SW, 0);

/**
 * Offboard switch channel
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_OFFB_SW, 0);

/**
 * Emergency Kill switch channel
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_KILL_SW, 0);

/**
 * Arm switch channel.
 *
 * Use it to arm/disarm via switch instead of default throttle stick. If this is
 * assigned, arming and disarming via stick is disabled.
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_ARM_SW, 0);

/**
 * Flaps channel
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_FLAPS, 0);

/**
 * VTOL transition switch channel mapping
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_TRANS_SW, 0);

/**
 * Landing gear switch channel
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_GEAR_SW, 0);

/**
 * AUX1 Passthrough RC channel
 *
 * Default function: Camera pitch
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_AUX1, 0);

/**
 * AUX2 Passthrough RC channel
 *
 * Default function: Camera roll
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_AUX2, 0);

/**
 * AUX3 Passthrough RC channel
 *
 * Default function: Camera azimuth / yaw
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_AUX3, 0);

/**
 * AUX4 Passthrough RC channel
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_AUX4, 0);

/**
 * AUX5 Passthrough RC channel
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_AUX5, 0);

/**
 * AUX6 Passthrough RC channel
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_AUX6, 0);
/**
 * PARAM1 tuning channel
 *
 * Can be used for parameter tuning with the RC. This one is further referenced as the 1st parameter channel.
 * Set to 0 to deactivate *
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_PARAM1, 0);

/**
 * PARAM2 tuning channel
 *
 * Can be used for parameter tuning with the RC. This one is further referenced as the 2nd parameter channel.
 * Set to 0 to deactivate *
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_PARAM2, 0);

/**
 * PARAM3 tuning channel
 *
 * Can be used for parameter tuning with the RC. This one is further referenced as the 3th parameter channel.
 * Set to 0 to deactivate *
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_PARAM3, 0);

/**
 * Failsafe channel PWM threshold.
 *
 * Set to a value slightly above the PWM value assumed by throttle in a failsafe event,
 * but ensure it is below the PWM value assumed by throttle during normal operation.
 *
 * Use RC_MAP_FAILSAFE to specify which channel is used to check.
 * Note: The default value of 0 is below the epxed range and hence disables the feature.
 *
 * @min 0
 * @max 2200
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_FAILS_THR, 0);

/**
 * Threshold for selecting return to launch mode
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 */
PARAM_DEFINE_FLOAT(RC_RETURN_TH, 0.75f);

/**
 * Threshold for selecting loiter mode
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 */
PARAM_DEFINE_FLOAT(RC_LOITER_TH, 0.75f);

/**
 * Threshold for selecting offboard mode
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 */
PARAM_DEFINE_FLOAT(RC_OFFB_TH, 0.75f);

/**
 * Threshold for the kill switch
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 */
PARAM_DEFINE_FLOAT(RC_KILLSWITCH_TH, 0.75f);

/**
 * Threshold for the arm switch
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 */
PARAM_DEFINE_FLOAT(RC_ARMSWITCH_TH, 0.75f);

/**
 * Threshold for the VTOL transition switch
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 */
PARAM_DEFINE_FLOAT(RC_TRANS_TH, 0.75f);

/**
 * Threshold for the landing gear switch
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 */
PARAM_DEFINE_FLOAT(RC_GEAR_TH, 0.75f);

/**
 * PWM input channel that provides RSSI.
 *
 * 0: do not read RSSI from input channel
 * 1-18: read RSSI from specified input channel
 *
 * Specify the range for RSSI input with RC_RSSI_PWM_MIN and RC_RSSI_PWM_MAX parameters.
 *
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group Radio Calibration
 *
 */
PARAM_DEFINE_INT32(RC_RSSI_PWM_CHAN, 0);

/**
 * Min input value for RSSI reading.
 *
 * Only used if RC_RSSI_PWM_CHAN > 0
 *
 * @min 0
 * @max 2000
 * @group Radio Calibration
 *
 */
PARAM_DEFINE_INT32(RC_RSSI_PWM_MIN, 1000);

/**
 * Max input value for RSSI reading.
 *
 * Only used if RC_RSSI_PWM_CHAN > 0
 *
 * @min 0
 * @max 2000
 * @group Radio Calibration
 *
 */
PARAM_DEFINE_INT32(RC_RSSI_PWM_MAX, 2000);
