/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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
 * @file sih_params.c
 * Parameters for quadcopter X simulator in hardware.
 *
 * @author Romain Chiappinelli <romain.chiap@gmail.com>
 * February 2019
 */

/**
 * Vehicle mass
 *
 * This value can be measured by weighting the quad on a scale.
 *
 * @unit kg
 * @min 0.0
 * @decimal 2
 * @increment 0.1
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_MASS, 1.0f);

/**
 * Vehicle inertia about X axis
 *
 * The intertia is a 3 by 3 symmetric matrix.
 * It represents the difficulty of the vehicle to modify its angular rate.
 *
 * @unit kg m^2
 * @min 0.0
 * @decimal 3
 * @increment 0.005
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_IXX, 0.025f);

/**
 * Vehicle inertia about Y axis
 *
 * The intertia is a 3 by 3 symmetric matrix.
 * It represents the difficulty of the vehicle to modify its angular rate.
 *
 * @unit kg m^2
 * @min 0.0
 * @decimal 3
 * @increment 0.005
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_IYY, 0.025f);

/**
 * Vehicle inertia about Z axis
 *
 * The intertia is a 3 by 3 symmetric matrix.
 * It represents the difficulty of the vehicle to modify its angular rate.
 *
 * @unit kg m^2
 * @min 0.0
 * @decimal 3
 * @increment 0.005
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_IZZ, 0.030f);

/**
 * Vehicle cross term inertia xy
 *
 * The intertia is a 3 by 3 symmetric matrix.
 * This value can be set to 0 for a quad symmetric about its center of mass.
 *
 * @unit kg m^2
 * @decimal 3
 * @increment 0.005
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_IXY, 0.0f);

/**
 * Vehicle cross term inertia xz
 *
 * The intertia is a 3 by 3 symmetric matrix.
 * This value can be set to 0 for a quad symmetric about its center of mass.
 *
 * @unit kg m^2
 * @decimal 3
 * @increment 0.005
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_IXZ, 0.0f);

/**
 * Vehicle cross term inertia yz
 *
 * The intertia is a 3 by 3 symmetric matrix.
 * This value can be set to 0 for a quad symmetric about its center of mass.
 *
 * @unit kg m^2
 * @decimal 3
 * @increment 0.005
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_IYZ, 0.0f);

/**
 * Max propeller thrust force
 *
 * This is the maximum force delivered by one propeller
 * when the motor is running at full speed.
 *
 * This value is usually about 5 times the mass of the quadrotor.
 *
 * @unit N
 * @min 0.0
 * @decimal 2
 * @increment 0.5
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_T_MAX, 5.0f);

/**
 * Max propeller torque
 *
 * This is the maximum torque delivered by one propeller
 * when the motor is running at full speed.
 *
 * This value is usually about few percent of the maximum thrust force.
 *
 * @unit Nm
 * @min 0.0
 * @decimal 3
 * @increment 0.05
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_Q_MAX, 0.1f);

/**
 * Roll arm length
 *
 * This is the arm length generating the rolling moment
 *
 * This value can be measured with a ruler.
 * This corresponds to half the distance between the left and right motors.
 *
 * @unit m
 * @min 0.0
 * @decimal 2
 * @increment 0.05
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_L_ROLL, 0.2f);

/**
 * Pitch arm length
 *
 * This is the arm length generating the pitching moment
 *
 * This value can be measured with a ruler.
 * This corresponds to half the distance between the front and rear motors.
 *
 * @unit m
 * @min 0.0
 * @decimal 2
 * @increment 0.05
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_L_PITCH, 0.2f);

/**
 * First order drag coefficient
 *
 * Physical coefficient representing the friction with air particules.
 * The greater this value, the slower the quad will move.
 *
 * Drag force function of velocity: D=-KDV*V.
 * The maximum freefall velocity can be computed as V=10*MASS/KDV [m/s]
 *
 * @unit N/(m/s)
 * @min 0.0
 * @decimal 2
 * @increment 0.05
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_KDV, 1.0f);

/**
 * First order angular damper coefficient
 *
 * Physical coefficient representing the friction with air particules during rotations.
 * The greater this value, the slower the quad will rotate.
 *
 * Aerodynamic moment function of body rate: Ma=-KDW*W_B.
 * This value can be set to 0 if unknown.
 *
 * @unit Nm/(rad/s)
 * @min 0.0
 * @decimal 3
 * @increment 0.005
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_KDW, 0.025f);

/**
 * Initial geodetic latitude
 *
 * This value represents the North-South location on Earth where the simulation begins.
 * A value of 45 deg should be written 450000000.
 *
 * LAT0, LON0, H0, MU_X, MU_Y, and MU_Z should ideally be consistent among each others
 * to represent a physical ground location on Earth.
 *
 * @unit deg*1e7
 * @min -850000000
 * @max  850000000
 * @group Simulation In Hardware
 */
PARAM_DEFINE_INT32(SIH_LOC_LAT0, 454671160);

/**
 * Initial geodetic longitude
 *
 * This value represents the East-West location on Earth where the simulation begins.
 * A value of 45 deg should be written 450000000.
 *
 * LAT0, LON0, H0, MU_X, MU_Y, and MU_Z should ideally be consistent among each others
 * to represent a physical ground location on Earth.
 *
 * @unit deg*1e7
 * @min -1800000000
 * @max  1800000000
 * @group Simulation In Hardware
 */
PARAM_DEFINE_INT32(SIH_LOC_LON0, -737578370);

/**
 * Initial AMSL ground altitude
 *
 * This value represents the Above Mean Sea Level (AMSL) altitude where the simulation begins.
 *
 * If using FlightGear as a visual animation,
 * this value can be tweaked such that the vehicle lies on the ground at takeoff.
 *
 * LAT0, LON0, H0, MU_X, MU_Y, and MU_Z should ideally be consistent among each others
 * to represent a physical ground location on Earth.
 *
 *
 * @unit m
 * @min -420.0
 * @max 8848.0
 * @decimal 2
 * @increment 0.01
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_LOC_H0, 32.34f);

/**
 * North magnetic field at the initial location
 *
 * This value represents the North magnetic field at the initial location.
 *
 * A magnetic field calculator can be found on the NOAA website
 * Note, the values need to be converted from nano Tesla to Gauss
 *
 * LAT0, LON0, H0, MU_X, MU_Y, and MU_Z should ideally be consistent among each others
 * to represent a physical ground location on Earth.
 *
 * @unit gauss
 * @min -1.0
 * @max  1.0
 * @decimal 2
 * @increment 0.001
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_LOC_MU_X,  0.179f);

/**
 * East magnetic field at the initial location
 *
 * This value represents the East magnetic field at the initial location.
 *
 * A magnetic field calculator can be found on the NOAA website
 * Note, the values need to be converted from nano Tesla to Gauss
 *
 * LAT0, LON0, H0, MU_X, MU_Y, and MU_Z should ideally be consistent among each others
 * to represent a physical ground location on Earth.
 *
 * @unit gauss
 * @min -1.0
 * @max  1.0
 * @decimal 2
 * @increment 0.001
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_LOC_MU_Y, -0.045f);

/**
 * Down magnetic field at the initial location
 *
 * This value represents the Down magnetic field at the initial location.
 *
 * A magnetic field calculator can be found on the NOAA website
 * Note, the values need to be converted from nano Tesla to Gauss
 *
 * LAT0, LON0, H0, MU_X, MU_Y, and MU_Z should ideally be consistent among each others
 * to represent a physical ground location on Earth.
 *
 * @unit gauss
 * @min -1.0
 * @max  1.0
 * @decimal 2
 * @increment 0.001
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_LOC_MU_Z,  0.504f);

/**
 * Number of GPS satellites used
 *
 * @min 0
 * @max  50
 * @group Simulation In Hardware
 */
PARAM_DEFINE_INT32(SIH_GPS_USED, 10);

/**
 * Barometer offset in meters
 *
 * Absolute value superior to 10000 will disable barometer
 *
 * @unit m
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_BARO_OFFSET,  0.0f);

/**
 * magnetometer X offset in Gauss
 *
 * Absolute value superior to 10000 will disable magnetometer
 *
 * @unit gauss
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_MAG_OFFSET_X,  0.0f);

/**
 * magnetometer Y offset in Gauss
 *
 * Absolute value superior to 10000 will disable magnetometer
 *
 * @unit gauss
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_MAG_OFFSET_Y,  0.0f);
/**
 * magnetometer Z offset in Gauss
 *
 * Absolute value superior to 10000 will disable magnetometer
 *
 * @unit gauss
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_MAG_OFFSET_Z,  0.0f);

/**
 * distance sensor minimun range
 *
 * @unit m
 * @min 0.0
 * @max 10.0
 * @decimal 4
 * @increment 0.01
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_DISTSNSR_MIN, 0.0f);

/**
 * distance sensor maximun range
 *
 * @unit m
 * @min 0.0
 * @max 1000.0
 * @decimal 4
 * @increment 0.01
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_DISTSNSR_MAX, 100.0f);

/**
 * if >= 0 the distance sensor measures will be overrided by this value
 *
 * Absolute value superior to 10000 will disable distance sensor
 *
 * @unit m
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_DISTSNSR_OVR, -1.0f);

/**
 * thruster time constant tau
 *
 * the time taken for the thruster to step from 0 to 100% should be about 4 times tau
 *
 * @unit s
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_T_TAU, 0.05f);
