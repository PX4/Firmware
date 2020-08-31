
/**
 * Airspeed Selector: Wind estimator wind process noise
 *
 * Wind process noise of the internal wind estimator(s) of the airspeed selector.
 *
 * @min 0
 * @max 1
 * @unit m/s^2
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_W_P_NOISE, 0.1f);

/**
 * Airspeed Selector: Wind estimator true airspeed scale process noise
 *
 * Airspeed scale process noise of the internal wind estimator(s) of the airspeed selector.
 *
 * @min 0
 * @max 0.1
 * @unit Hz
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_SC_P_NOISE, 0.0001);

/**
 * Airspeed Selector: Wind estimator true airspeed measurement noise
 *
 * True airspeed measurement noise of the internal wind estimator(s) of the airspeed selector.
 *
 * @min 0
 * @max 4
 * @unit m/s
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_TAS_NOISE, 1.4);

/**
 * Airspeed Selector: Wind estimator sideslip measurement noise
 *
 * Sideslip measurement noise of the internal wind estimator(s) of the airspeed selector.
 *
 * @min 0
 * @max 1
 * @unit rad
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_BETA_NOISE, 0.3);

/**
 * Airspeed Selector: Gate size for true airspeed fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @min 1
 * @max 5
 * @unit SD
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_TAS_GATE, 3);

/**
 * Airspeed Selector: Gate size for sideslip angle fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @min 1
 * @max 5
 * @unit SD
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_BETA_GATE, 1);

/**
 * Automatic airspeed scale estimation on
 *
 * Turns the automatic airspeed scale (scale from IAS to CAS/EAS) on or off. It is recommended to fly level
 * altitude while performing the estimation. Set to 1 to start estimation (best when already flying).
 * Set to 0 to end scale estimation. The estimated scale is then saved using the ASPD_SCALE parameter.
 *
 * @boolean
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_SCALE_EST, 0);

/**
 * Airspeed scale (scale from IAS to CAS/EAS)
 *
 * Scale can either be entered manually, or estimated in-flight by setting ASPD_SCALE_EST to 1.
 *
 * @min 0.5
 * @max 1.5
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_SCALE, 1.0f);

/**
 * Index or primary airspeed measurement source
 *
 * @value -1 Disabled
 * @value 0 Groundspeed minus windspeed
 * @value 1 First airspeed sensor
 * @value 2 Second airspeed sensor
 * @value 3 Third airspeed sensor
 *
 * @reboot_required true
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_PRIMARY, 1);


/**
 * Enable checks on airspeed sensors
 *
 * If set to true then the data comming from the airspeed sensors is checked for validity. Only applied if ASPD_PRIMARY > 0.
 *
 * @reboot_required true
 * @boolean
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_DO_CHECKS, 0);

/**
 * Enable fallback to secondary airspeed measurement.
 *
 * If ASPD_DO_CHECKS is set to true, then airspeed estimation can fallback from what specified in ASPD_PRIMARY to secondary source (other airspeed sensors, groundspeed minus windspeed).

 * @value 0 To other airspeed sensor (if one valid), else disable airspeed
 * @value 1 To other airspeed sensor (if one valid), else to ground-windspeed
 * @boolean
 * @reboot_required true
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ASPD_FALLBACK, 0);

/**
 * Airspeed failsafe consistency threshold (Experimental)
 *
 * This specifies the minimum airspeed test ratio required to trigger a failsafe. Larger values make the check less sensitive,
 * smaller values make it more sensitive. Start with a value of 1.0 when tuning. When tas_test_ratio is > 1.0 it indicates the
 * inconsistency between predicted and measured airspeed is large enough to cause the navigation EKF to reject airspeed measurements.
 * The time required to detect a fault when the threshold is exceeded depends on the size of the exceedance and is controlled by the ASPD_FS_INTEG parameter.
*
 * @min 0.5
 * @max 3.0
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_FS_INNOV, 1.0f);

/**
 * Airspeed failsafe consistency delay (Experimental)
 *
 * This sets the time integral of airspeed test ratio exceedance above ASPD_FS_INNOV required to trigger a failsafe.
 * For example if ASPD_FS_INNOV is 1 and estimator_status.tas_test_ratio is 2.0, then the exceedance is 1.0 and the integral will
 * rise at a rate of 1.0/second. A negative value disables the check. Larger positive values make the check less sensitive, smaller positive values
 * make it more sensitive.
 *
 * @unit s
 * @max 30.0
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ASPD_FS_INTEG, -1.0f);

/**
 * Airspeed failsafe stop delay (Experimental)
 *
 * Delay before stopping use of airspeed sensor if checks indicate sensor is bad.
 *
 * @unit s
 * @group Airspeed Validator
 * @min 1
 * @max 10
 */
PARAM_DEFINE_INT32(ASPD_FS_T1, 3);

/**
 * Airspeed failsafe start delay (Experimental)
 *
 * Delay before switching back to using airspeed sensor if checks indicate sensor is good.
 *
 * @unit s
 * @group Airspeed Validator
 * @min 10
 * @max 1000
 */
PARAM_DEFINE_INT32(ASPD_FS_T2, 100);

/**
 * Airspeed fault detection stall airspeed. (Experimental)
 *
 * This is the minimum indicated airspeed at which the wing can produce 1g of lift. It is used by the airspeed sensor fault detection and failsafe calculation to detect a significant airspeed low measurement error condition and should be set based on flight test for reliable operation.
 *
 * @group Airspeed Validator
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(ASPD_STALL, 10.0f);
