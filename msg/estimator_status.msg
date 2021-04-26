uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32[3] vibe			# IMU vibration metrics in the following array locations
# 0 : Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
# 1 : Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
# 2 : Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)

float32[3] output_tracking_error # return a vector containing the output predictor angular, velocity and position tracking error magnitudes (rad), (m/s), (m)

uint16 gps_check_fail_flags     # Bitmask to indicate status of GPS checks - see definition below
# bits are true when corresponding test has failed
uint8 GPS_CHECK_FAIL_GPS_FIX = 0		# 0 : insufficient fix type (no 3D solution)
uint8 GPS_CHECK_FAIL_MIN_SAT_COUNT = 1		# 1 : minimum required sat count fail
uint8 GPS_CHECK_FAIL_MAX_PDOP = 2		# 2 : maximum allowed PDOP fail
uint8 GPS_CHECK_FAIL_MAX_HORZ_ERR = 3		# 3 : maximum allowed horizontal position error fail
uint8 GPS_CHECK_FAIL_MAX_VERT_ERR = 4		# 4 : maximum allowed vertical position error fail
uint8 GPS_CHECK_FAIL_MAX_SPD_ERR = 5		# 5 : maximum allowed speed error fail
uint8 GPS_CHECK_FAIL_MAX_HORZ_DRIFT = 6		# 6 : maximum allowed horizontal position drift fail - requires stationary vehicle
uint8 GPS_CHECK_FAIL_MAX_VERT_DRIFT = 7		# 7 : maximum allowed vertical position drift fail - requires stationary vehicle
uint8 GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR = 8	# 8 : maximum allowed horizontal speed fail - requires stationary vehicle
uint8 GPS_CHECK_FAIL_MAX_VERT_SPD_ERR = 9	# 9 : maximum allowed vertical velocity discrepancy fail

uint32 control_mode_flags	# Bitmask to indicate EKF logic state
uint8 CS_TILT_ALIGN = 0		# 0 - true if the filter tilt alignment is complete
uint8 CS_YAW_ALIGN = 1		# 1 - true if the filter yaw alignment is complete
uint8 CS_GPS = 2		# 2 - true if GPS measurements are being fused
uint8 CS_OPT_FLOW = 3		# 3 - true if optical flow measurements are being fused
uint8 CS_MAG_HDG = 4		# 4 - true if a simple magnetic yaw heading is being fused
uint8 CS_MAG_3D = 5		# 5 - true if 3-axis magnetometer measurement are being fused
uint8 CS_MAG_DEC = 6		# 6 - true if synthetic magnetic declination measurements are being fused
uint8 CS_IN_AIR = 7		# 7 - true when thought to be airborne
uint8 CS_WIND = 8		# 8 - true when wind velocity is being estimated
uint8 CS_BARO_HGT = 9		# 9 - true when baro height is being fused as a primary height reference
uint8 CS_RNG_HGT = 10		# 10 - true when range finder height is being fused as a primary height reference
uint8 CS_GPS_HGT = 11		# 11 - true when GPS height is being fused as a primary height reference
uint8 CS_EV_POS = 12		# 12 - true when local position data from external vision is being fused
uint8 CS_EV_YAW = 13		# 13 - true when yaw data from external vision measurements is being fused
uint8 CS_EV_HGT = 14		# 14 - true when height data from external vision measurements is being fused
uint8 CS_BETA = 15		# 15 - true when synthetic sideslip measurements are being fused
uint8 CS_MAG_FIELD = 16		# 16 - true when only the magnetic field states are updated by the magnetometer
uint8 CS_FIXED_WING = 17	# 17 - true when thought to be operating as a fixed wing vehicle with constrained sideslip
uint8 CS_MAG_FAULT = 18		# 18 - true when the magnetomer has been declared faulty and is no longer being used
uint8 CS_ASPD = 19		# 19 - true when airspeed measurements are being fused
uint8 CS_GND_EFFECT = 20	# 20 - true when when protection from ground effect induced static pressure rise is active
uint8 CS_RNG_STUCK = 21		# 21 - true when a stuck range finder sensor has been detected
uint8 CS_GPS_YAW = 22		# 22 - true when yaw (not ground course) data from a GPS receiver is being fused
uint8 CS_MAG_ALIGNED = 23	# 23 - true when the in-flight mag field alignment has been completed

uint32 filter_fault_flags	# Bitmask to indicate EKF internal faults
# 0 - true if the fusion of the magnetometer X-axis has encountered a numerical error
# 1 - true if the fusion of the magnetometer Y-axis has encountered a numerical error
# 2 - true if the fusion of the magnetometer Z-axis has encountered a numerical error
# 3 - true if the fusion of the magnetic heading has encountered a numerical error
# 4 - true if the fusion of the magnetic declination has encountered a numerical error
# 5 - true if fusion of the airspeed has encountered a numerical error
# 6 - true if fusion of the synthetic sideslip constraint has encountered a numerical error
# 7 - true if fusion of the optical flow X axis has encountered a numerical error
# 8 - true if fusion of the optical flow Y axis has encountered a numerical error
# 9 - true if fusion of the North velocity has encountered a numerical error
# 10 - true if fusion of the East velocity has encountered a numerical error
# 11 - true if fusion of the Down velocity has encountered a numerical error
# 12 - true if fusion of the North position has encountered a numerical error
# 13 - true if fusion of the East position has encountered a numerical error
# 14 - true if fusion of the Down position has encountered a numerical error
# 15 - true if bad delta velocity bias estimates have been detected
# 16 - true if bad vertical accelerometer data has been detected
# 17 - true if delta velocity data contains clipping (asymmetric railing)

float32 pos_horiz_accuracy # 1-Sigma estimated horizontal position accuracy relative to the estimators origin (m)
float32 pos_vert_accuracy # 1-Sigma estimated vertical position accuracy relative to the estimators origin (m)
uint16 innovation_check_flags # Bitmask to indicate pass/fail status of innovation consistency checks
# 0 - true if velocity observations have been rejected
# 1 - true if horizontal position observations have been rejected
# 2 - true if true if vertical position observations have been rejected
# 3 - true if the X magnetometer observation has been rejected
# 4 - true if the Y magnetometer observation has been rejected
# 5 - true if the Z magnetometer observation has been rejected
# 6 - true if the yaw observation has been rejected
# 7 - true if the airspeed observation has been rejected
# 8 - true if the synthetic sideslip observation has been rejected
# 9 - true if the height above ground observation has been rejected
# 10 - true if the X optical flow observation has been rejected
# 11 - true if the Y optical flow observation has been rejected

float32 mag_test_ratio # ratio of the largest magnetometer innovation component to the innovation test limit
float32 vel_test_ratio # ratio of the largest velocity innovation component to the innovation test limit
float32 pos_test_ratio # ratio of the largest horizontal position innovation component to the innovation test limit
float32 hgt_test_ratio # ratio of the vertical position innovation to the innovation test limit
float32 tas_test_ratio # ratio of the true airspeed innovation to the innovation test limit
float32 hagl_test_ratio # ratio of the height above ground innovation to the innovation test limit
float32 beta_test_ratio # ratio of the synthetic sideslip innovation to the innovation test limit

uint16 solution_status_flags # Bitmask indicating which filter kinematic state outputs are valid for flight control use.
# 0 - True if the attitude estimate is good
# 1 - True if the horizontal velocity estimate is good
# 2 - True if the vertical velocity estimate is good
# 3 - True if the horizontal position (relative) estimate is good
# 4 - True if the horizontal position (absolute) estimate is good
# 5 - True if the vertical position (absolute) estimate is good
# 6 - True if the vertical position (above ground) estimate is good
# 7 - True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
# 8 - True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
# 9 - True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
# 10 - True if the EKF has detected a GPS glitch
# 11 - True if the EKF has detected bad accelerometer data

uint8 reset_count_vel_ne # number of horizontal position reset events (allow to wrap if count exceeds 255)
uint8 reset_count_vel_d  # number of vertical velocity reset events (allow to wrap if count exceeds 255)
uint8 reset_count_pos_ne # number of horizontal position reset events (allow to wrap if count exceeds 255)
uint8 reset_count_pod_d  # number of vertical position reset events (allow to wrap if count exceeds 255)
uint8 reset_count_quat   # number of quaternion reset events (allow to wrap if count exceeds 255)

float32 time_slip # cumulative amount of time in seconds that the EKF inertial calculation has slipped relative to system time

bool pre_flt_fail_innov_heading
bool pre_flt_fail_innov_vel_horiz
bool pre_flt_fail_innov_vel_vert
bool pre_flt_fail_innov_height
bool pre_flt_fail_mag_field_disturbed

uint32 accel_device_id
uint32 gyro_device_id
uint32 baro_device_id
uint32 mag_device_id

# legacy local position estimator (LPE) flags
uint8 health_flags		# Bitmask to indicate sensor health states (vel, pos, hgt)
uint8 timeout_flags		# Bitmask to indicate timeout flags (vel, pos, hgt)
