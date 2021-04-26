#include <parameters/param.h>

// 16 is max name length

/**
 * Optical flow z offset from center
 *
 * @group Local Position Estimator
 * @unit m
 * @min -1
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_FLW_OFF_Z, 0.0f);

/**
 * Optical flow scale
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.1
 * @max 10.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_FLW_SCALE, 1.3f);

/**
 * Optical flow rotation (roll/pitch) noise gain
 *
 * @group Local Position Estimator
 * @unit m/s/rad
 * @min 0.1
 * @max 10.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_FLW_R, 7.0f);

/**
 * Optical flow angular velocity noise gain
 *
 * @group Local Position Estimator
 * @unit m/rad
 * @min 0.0
 * @max 10.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_FLW_RR, 7.0f);

/**
 * Optical flow minimum quality threshold
 *
 * @group Local Position Estimator
 * @min 0
 * @max 255
 * @decimal 0
 */
PARAM_DEFINE_INT32(LPE_FLW_QMIN, 150);

/**
 * Sonar z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_SNR_Z, 0.05f);

/**
 * Sonar z offset from center of vehicle +down
 *
 * @group Local Position Estimator
 * @unit m
 * @min -1
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_SNR_OFF_Z, 0.00f);

/**
 * Lidar z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_LDR_Z, 0.03f);

/**
 * Lidar z offset from center of vehicle +down
 *
 * @group Local Position Estimator
 * @unit m
 * @min -1
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_LDR_OFF_Z, 0.00f);

/**
 * Accelerometer xy noise density
 *
 * Data sheet noise density = 150ug/sqrt(Hz) = 0.0015 m/s^2/sqrt(Hz)
 *
 * Larger than data sheet to account for tilt error.
 *
 * @group Local Position Estimator
 * @unit m/s^2/sqrt(Hz)
 * @min 0.00001
 * @max 2
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(LPE_ACC_XY, 0.012f);

/**
 * Accelerometer z noise density
 *
 * Data sheet noise density = 150ug/sqrt(Hz) = 0.0015 m/s^2/sqrt(Hz)
 *
 * @group Local Position Estimator
 * @unit m/s^2/sqrt(Hz)
 * @min 0.00001
 * @max 2
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(LPE_ACC_Z, 0.02f);

/**
 * Barometric presssure altitude z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 100
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(LPE_BAR_Z, 3.0f);

/**
 * GPS delay compensaton
 *
 * @group Local Position Estimator
 * @unit s
 * @min 0
 * @max 0.4
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_DELAY, 0.29f);


/**
 * Minimum GPS xy standard deviation, uses reported EPH if greater.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 5
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_XY, 1.0f);

/**
 * Minimum GPS z standard deviation, uses reported EPV if greater.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 200
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_Z, 3.0f);

/**
 * GPS xy velocity standard deviation.
 *
 * EPV used if greater than this value.
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 2
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_GPS_VXY, 0.25f);

/**
 * GPS z velocity standard deviation.
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 2
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_GPS_VZ, 0.25f);

/**
 * Max EPH allowed for GPS initialization
 *
 * @group Local Position Estimator
 * @unit m
 * @min 1.0
 * @max 5.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_EPH_MAX, 3.0f);

/**
 * Max EPV allowed for GPS initialization
 *
 * @group Local Position Estimator
 * @unit m
 * @min 1.0
 * @max 5.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_EPV_MAX, 5.0f);

/**
 * Vision delay compensaton.
 *
 * Set to zero to enable automatic compensation from measurement timestamps
 *
 * @group Local Position Estimator
 * @unit s
 * @min 0
 * @max 0.1
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(LPE_VIS_DELAY, 0.1f);

/**
 * Vision xy standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_VIS_XY, 0.1f);

/**
 * Vision z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 100
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_VIS_Z, 0.5f);

/**
 * Vicon position standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.0001
 * @max 1
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(LPE_VIC_P, 0.001f);

/**
 * Position propagation noise density
 *
 * Increase to trust measurements more.
 * Decrease to trust model more.
 *
 * @group Local Position Estimator
 * @unit m/s/sqrt(Hz)
 * @min 0
 * @max 1
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(LPE_PN_P, 0.1f);

/**
 * Velocity propagation noise density
 *
 * Increase to trust measurements more.
 * Decrease to trust model more.
 *
 * @group Local Position Estimator
 * @unit m/s^2/sqrt(Hz)
 * @min 0
 * @max 1
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(LPE_PN_V, 0.1f);

/**
 * Accel bias propagation noise density
 *
 * @group Local Position Estimator
 * @unit m/s^3/sqrt(Hz)
 * @min 0
 * @max 1
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(LPE_PN_B, 1e-3f);

/**
 * Terrain random walk noise density, hilly/outdoor (0.1), flat/Indoor (0.001)
 *
 * @group Local Position Estimator
 * @unit m/s/sqrt(Hz)
 * @min 0
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_PN_T, 0.001f);

/**
 * Terrain maximum percent grade, hilly/outdoor (100 = 45 deg), flat/Indoor (0 = 0 deg)
 *
 * Used to calculate increased terrain random walk nosie due to movement.
 *
 * @group Local Position Estimator
 * @unit %
 * @min 0
 * @max 100
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_T_MAX_GRADE, 1.0f);

/**
 * Flow gyro high pass filter cut off frequency
 *
 * @group Local Position Estimator
 * @unit Hz
 * @min 0
 * @max 2
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_FGYRO_HP, 0.001f);

/**
 * Enable publishing of a fake global position (e.g for AUTO missions using Optical Flow)
 *
 * By initializing the estimator to the LPE_LAT/LON parameters when global information is unavailable
 *
 * @group Local Position Estimator
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(LPE_FAKE_ORIGIN, 0);

/**
 * Local origin latitude for nav w/o GPS
 *
 * @group Local Position Estimator
 * @unit deg
 * @min -90
 * @max 90
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(LPE_LAT, 47.397742f);

/**
 * Local origin longitude for nav w/o GPS
 *
 * @group Local Position Estimator
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(LPE_LON, 8.545594);

/**
 * Cut frequency for state publication
 *
 * @group Local Position Estimator
 * @unit Hz
 * @min 5
 * @max 1000
 * @decimal 0
 */
PARAM_DEFINE_FLOAT(LPE_X_LP, 5.0f);

/**
 * Required velocity xy standard deviation to publish position
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 1.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_VXY_PUB, 0.3f);

/**
 * Required z standard deviation to publish altitude/ terrain
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.3
 * @max 5.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(LPE_Z_PUB, 1.0f);

/**
 * Land detector z standard deviation
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.001
 * @max 10.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_LAND_Z, 0.03f);

/**
 * Land detector xy velocity standard deviation
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 10.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(LPE_LAND_VXY, 0.05f);

/**
 * Minimum landing target standard covariance, uses reported covariance if greater.
 *
 * @group Local Position Estimator
 * @unit m^2
 * @min 0.0
 * @max 10
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(LPE_LT_COV, 0.0001f);

/**
 * Integer bitmask controlling data fusion
 *
 * Set bits in the following positions to enable:
 * 0 : Set to true to fuse GPS data if available, also requires GPS for altitude init
 * 1 : Set to true to fuse optical flow data if available
 * 2 : Set to true to fuse vision position
 * 3 : Set to true to enable landing target
 * 4 : Set to true to fuse land detector
 * 5 : Set to true to publish AGL as local position down component
 * 6 : Set to true to enable flow gyro compensation
 * 7 : Set to true to enable baro fusion
 *
 * default (145 - GPS, baro, land detector)
 *
 * @group Local Position Estimator
 * @min 0
 * @max 255
 * @bit 0  fuse GPS, requires GPS for alt. init
 * @bit 1  fuse optical flow
 * @bit 2  fuse vision position
 * @bit 3  fuse landing target
 * @bit 4  fuse land detector
 * @bit 5  pub agl as lpos down
 * @bit 6  flow gyro compensation
 * @bit 7  fuse baro
 */
PARAM_DEFINE_INT32(LPE_FUSION, 145);
