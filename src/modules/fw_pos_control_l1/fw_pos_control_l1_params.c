/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file fw_pos_control_l1_params.c
 *
 * Parameters defined by the L1 position control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

/*
 * Controller parameters, accessible via MAVLink
 */

/**
 * L1 period
 *
 * This is the L1 distance and defines the tracking
 * point ahead of the aircraft its following.
 * A value of 18-25 meters works for most aircraft. Shorten
 * slowly during tuning until response is sharp without oscillation.
 *
 * @unit m
 * @min 12.0
 * @max 50.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_L1_PERIOD, 20.0f);

/**
 * L1 damping
 *
 * Damping factor for L1 control.
 *
 * @min 0.6
 * @max 0.9
 * @decimal 2
 * @increment 0.05
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_L1_DAMPING, 0.75f);

/**
 * L1 controller roll slew rate limit.
 *
 * The maxium change in roll angle setpoint per second.
 *
 * @unit deg/s
 * @min 0
 * @increment 1
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_L1_R_SLEW_MAX, 90.0f);

/**
 * Cruise throttle
 *
 * This is the throttle setting required to achieve the desired cruise speed. Most airframes have a value of 0.5-0.7.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_THR_CRUISE, 0.6f);

/**
 * Scale throttle by pressure change
 *
 * Automatically adjust throttle to account for decreased air density at higher altitudes.
 * Start with a scale factor of 1.0 and adjust for different propulsion systems.
 *
 * When flying without airspeed sensor this will help to keep a constant performance over large altitude ranges.
 *
 * The default value of 0 will disable scaling.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 0.1
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_THR_ALT_SCL, 0.0f);

/**
 * Throttle max slew rate
 *
 * Maximum slew rate for the commanded throttle
 *
 * @min 0.0
 * @max 1.0
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_THR_SLEW_MAX, 0.0f);

/**
 * Negative pitch limit
 *
 * The minimum negative pitch the controller will output.
 *
 * @unit deg
 * @min -60.0
 * @max 0.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_P_LIM_MIN, -45.0f);

/**
 * Positive pitch limit
 *
 * The maximum positive pitch the controller will output.
 *
 * @unit deg
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_P_LIM_MAX, 45.0f);

/**
 * Controller roll limit
 *
 * The maximum roll the controller will output.
 *
 * @unit deg
 * @min 35.0
 * @max 65.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_R_LIM, 50.0f);

/**
 * Throttle limit max
 *
 * This is the maximum throttle % that can be used by the controller.
 * For overpowered aircraft, this should be reduced to a value that
 * provides sufficient thrust to climb at the maximum pitch angle PTCH_MAX.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_THR_MAX, 1.0f);

/**
 * Throttle limit min
 *
 * This is the minimum throttle % that can be used by the controller.
 * For electric aircraft this will normally be set to zero, but can be set
 * to a small non-zero value if a folding prop is fitted to prevent the
 * prop from folding and unfolding repeatedly in-flight or to provide
 * some aerodynamic drag from a turning prop to improve the descent rate.
 *
 * For aircraft with internal combustion engine this parameter should be set
 * for desired idle rpm.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_THR_MIN, 0.0f);

/**
 * Idle throttle
 *
 * This is the minimum throttle while on the ground
 *
 * For aircraft with internal combustion engine this parameter should be set
 * above desired idle rpm.
 *
 * @unit norm
 * @min 0.0
 * @max 0.4
 * @decimal 2
 * @increment 0.01
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_THR_IDLE, 0.15f);

/**
 * Throttle limit during landing below throttle limit altitude
 *
 * During the flare of the autonomous landing process, this value will be set
 * as throttle limit when the aircraft altitude is below FW_LND_TLALT.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_THR_LND_MAX, 1.0f);

/**
 * Climbout Altitude difference
 *
 * If the altitude error exceeds this parameter, the system will climb out
 * with maximum throttle and minimum airspeed until it is closer than this
 * distance to the desired altitude. Mostly used for takeoff waypoints / modes.
 * Set to 0 to disable climbout mode (not recommended).
 *
 * @unit m
 * @min 0.0
 * @max 150.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_CLMBOUT_DIFF, 10.0f);

/**
 * Landing slope angle
 *
 * @unit deg
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_ANG, 5.0f);

/**
 * Minimum pitch during takeoff.
 *
 * @unit deg
 * @min -5.0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_TKO_PITCH_MIN, 10.0f);

/**
 *
 *
 * @unit m
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_HVIRT, 10.0f);

/**
 * Landing flare altitude (relative to landing altitude)
 *
 * @unit m
 * @min 0.0
 * @max 25.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_FLALT, 3.0f);

/**
 * Landing throttle limit altitude (relative landing altitude)
 *
 * Default of -1.0 lets the system default to applying throttle
 * limiting at 2/3 of the flare altitude.
 *
 * @unit m
 * @min -1.0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_TLALT, -1.0f);

/**
 * Landing heading hold horizontal distance.
 *
 * Set to 0 to disable heading hold.
 *
 * @unit m
 * @min 0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_HHDIST, 15.0f);

/**
 * Use terrain estimate during landing.
 *
 * This is turned off by default and a waypoint or return altitude is normally used
 * (or sea level for an arbitrary land position).
 *
 * @boolean
 * @group FW L1 Control
 */
PARAM_DEFINE_INT32(FW_LND_USETER, 0);

/**
 * Early landing configuration deployment
 *
 * When disabled, the landing configuration (flaps, landing airspeed, etc.) is only activated
 * on the final approach to landing. When enabled, it is already activated when entering the
 * final loiter-down (loiter-to-alt) waypoint before the landing approach. This shifts the (often large)
 * altitude and airspeed errors caused by the configuration change away from the ground such that
 * these are not so critical. It also gives the controller enough time to adapt to the new
 * configuration such that the landing approach starts with a cleaner initial state.
 *
 * @boolean
 *
 * @group FW L1 Control
 */
PARAM_DEFINE_INT32(FW_LND_EARLYCFG, 0);

/**
 * Flare, minimum pitch
 *
 * Minimum pitch during flare, a positive sign means nose up
 * Applied once FW_LND_FLALT is reached
 *
 * @unit deg
 * @min 0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_PMIN, 2.5f);

/**
 * Flare, maximum pitch
 *
 * Maximum pitch during flare, a positive sign means nose up
 * Applied once FW_LND_FLALT is reached
 *
 * @unit deg
 * @min 0
 * @max 45.0
 * @decimal 1
 * @increment 0.5
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_PMAX, 15.0f);

/**
 * Min. airspeed scaling factor for landing
 *
 * Multiplying this factor with the minimum airspeed of the plane
 * gives the target airspeed the landing approach.
 * FW_AIRSPD_MIN * FW_LND_AIRSPD_SC
 *
 * @unit norm
 * @min 1.0
 * @max 1.5
 * @decimal 2
 * @increment 0.01
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_AIRSPD_SC, 1.3f);

/**
 * Altitude time constant factor for landing
 *
 * Set this parameter to less than 1.0 to make TECS react faster to altitude errors during
 * landing than during normal flight (i.e. giving efficiency and low motor wear at
 * high altitudes but control accuracy during landing). During landing, the TECS
 * altitude time constant (FW_T_ALT_TC) is multiplied by this value.
 *
 * @unit
 * @min 0.2
 * @max 1.0
 * @increment 0.1
 * @group FW L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_THRTC_SC, 1.0f);



/*
 * TECS parameters
 *
 */


/**
 * Minimum Airspeed (CAS)
 *
 * If the CAS (calibrated airspeed) falls below this value, the TECS controller will try to
 * increase airspeed more aggressively.
 *
 * @unit m/s
 * @min 0.5
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_MIN, 10.0f);

/**
 * Maximum Airspeed (CAS)
 *
 * If the CAS (calibrated airspeed) is above this value, the TECS controller will try to decrease
 * airspeed more aggressively.
 *
 * @unit m/s
 * @min 0.5
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_MAX, 20.0f);

/**
 * Cruise Airspeed (CAS)
 *
 * The trim CAS (calibrated airspeed) of the vehicle. If an airspeed controller is active,
 * this is the default airspeed setpoint that the controller will try to achieve if
 * no other airspeed setpoint sources are present (e.g. through non-centered RC sticks).
 *
 * @unit m/s
 * @min 0.5
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_TRIM, 15.0f);

/**
 * Maximum climb rate
 *
 * This is the best climb rate that the aircraft can achieve with
 * the throttle set to THR_MAX and the airspeed set to the
 * default value. For electric aircraft make sure this number can be
 * achieved towards the end of flight when the battery voltage has reduced.
 * The setting of this parameter can be checked by commanding a positive
 * altitude change of 100m in loiter, RTL or guided mode. If the throttle
 * required to climb is close to THR_MAX and the aircraft is maintaining
 * airspeed, then this parameter is set correctly. If the airspeed starts
 * to reduce, then the parameter is set to high, and if the throttle
 * demand required to climb and maintain speed is noticeably less than
 * FW_THR_MAX, then either FW_T_CLMB_MAX should be increased or
 * FW_THR_MAX reduced.
 *
 * @unit m/s
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_CLMB_MAX, 5.0f);

/**
 * Minimum descent rate
 *
 * This is the sink rate of the aircraft with the throttle
 * set to THR_MIN and flown at the same airspeed as used
 * to measure FW_T_CLMB_MAX.
 *
 * @unit m/s
 * @min 1.0
 * @max 5.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SINK_MIN, 2.0f);

/**
 * Maximum descent rate
 *
 * This sets the maximum descent rate that the controller will use.
 * If this value is too large, the aircraft can over-speed on descent.
 * This should be set to a value that can be achieved without
 * exceeding the lower pitch angle limit and without over-speeding
 * the aircraft.
 *
 * @unit m/s
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SINK_MAX, 5.0f);

/**
 * Throttle damping factor
 *
 * This is the damping gain for the throttle demand loop.
 * Increase to add damping to correct for oscillations in speed and height.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_THR_DAMP, 0.1f);

/**
 * Integrator gain throttle
 *
 * This is the integrator gain on the throttle part of the control loop.
 * Increasing this gain increases the speed at which speed
 * and height offsets are trimmed out, but reduces damping and
 * increases overshoot. Set this value to zero to completely
 * disable all integrator action.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_I_GAIN_THR, 0.3f);

/**
 * Integrator gain pitch
 *
 * This is the integrator gain on the pitch part of the control loop.
 * Increasing this gain increases the speed at which speed
 * and height offsets are trimmed out, but reduces damping and
 * increases overshoot. Set this value to zero to completely
 * disable all integrator action.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_I_GAIN_PIT, 0.1f);

/**
 * Maximum vertical acceleration
 *
 * This is the maximum vertical acceleration (in m/s/s)
 * either up or down that the controller will use to correct speed
 * or height errors. The default value of 7 m/s/s (equivalent to +- 0.7 g)
 * allows for reasonably aggressive pitch changes if required to recover
 * from under-speed conditions.
 *
 * @unit m/s^2
 * @min 1.0
 * @max 10.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_VERT_ACC, 7.0f);

/**
 * Complementary filter "omega" parameter for speed
 *
 * This is the cross-over frequency (in radians/second) of the complementary
 * filter used to fuse longitudinal acceleration and airspeed to obtain an
 * improved airspeed estimate. Increasing this frequency weights the solution
 * more towards use of the airspeed sensor, whilst reducing it weights the
 * solution more towards use of the accelerometer data.
 *
 * @unit rad/s
 * @min 1.0
 * @max 10.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SPD_OMEGA, 2.0f);

/**
 * Roll -> Throttle feedforward
 *
 * Increasing this gain turn increases the amount of throttle that will
 * be used to compensate for the additional drag created by turning.
 * Ideally this should be set to  approximately 10 x the extra sink rate
 * in m/s created by a 45 degree bank turn. Increase this gain if
 * the aircraft initially loses energy in turns and reduce if the
 * aircraft initially gains energy in turns. Efficient high aspect-ratio
 * aircraft (eg powered sailplanes) can use a lower value, whereas
 * inefficient low aspect-ratio models (eg delta wings) can use a higher value.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_RLL2THR, 15.0f);

/**
 * Speed <--> Altitude priority
 *
 * This parameter adjusts the amount of weighting that the pitch control
 * applies to speed vs height errors. Setting it to 0.0 will cause the
 * pitch control to control height and ignore speed errors. This will
 * normally improve height accuracy but give larger airspeed errors.
 * Setting it to 2.0 will cause the pitch control loop to control speed
 * and ignore height errors. This will normally reduce airspeed errors,
 * but give larger height errors. The default value of 1.0 allows the pitch
 * control to simultaneously control height and speed.
 * Note to Glider Pilots - set this parameter to 2.0 (The glider will
 * adjust its pitch angle to maintain airspeed, ignoring changes in height).
 *
 * @min 0.0
 * @max 2.0
 * @decimal 1
 * @increment 1.0
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SPDWEIGHT, 1.0f);

/**
 * Pitch damping factor
 *
 * This is the damping gain for the pitch demand loop. Increase to add
 * damping to correct for oscillations in height. The default value of 0.0
 * will work well provided the pitch to servo controller has been tuned
 * properly.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_PTCH_DAMP, 0.1f);

/**
 * Altitude error time constant.
 *
 * @min 2.0
 * @decimal 2
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_ALT_TC, 5.0f);

/**
 * Height rate feed forward
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_HRATE_FF, 0.3f);

/**
 * True airspeed error time constant.
 *
 * @min 2.0
 * @decimal 2
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_TAS_TC, 5.0f);

/**
 * Minimum groundspeed
 *
 * The controller will increase the commanded airspeed to maintain
 * this minimum groundspeed to the next waypoint.
 *
 * @unit m/s
 * @min 0.0
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_GND_SPD_MIN, 5.0f);

/**
 * RC stick mapping fixed-wing.
 *
 * Set RC/joystick configuration for fixed-wing position and altitude controlled flight.
 *
 * @min 0
 * @max 1
 * @value 0 Normal stick configuration (airspeed on throttle stick, altitude on pitch stick)
 * @value 1 Alternative stick configuration (altitude on throttle stick, airspeed on pitch stick)
 * @group FW L1 Control
 */
PARAM_DEFINE_INT32(FW_POSCTL_INV_ST, 0);

/**
 * Specific total energy rate first order filter time constant.
 *
 * This filter is applied to the specific total energy rate used for throttle damping.
 *
 * @min 0.0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_STE_R_TC, 0.4f);


/**
 * True airspeed rate first order filter time constant.
 *
 * This filter is applied to the true airspeed rate.
 *
 * @min 0.0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_TAS_R_TC, 0.2f);


/**
 * Specific total energy balance rate feedforward gain.
 *
 *
 * @min 0.5
 * @max 3
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SEB_R_FF, 1.0f);
