# Vehicle odometry data. Fits ROS REP 147 for aerial vehicles
uint64 timestamp		# time since system start (microseconds)

uint64 timestamp_sample

# Covariance matrix index constants
uint8 COVARIANCE_MATRIX_X_VARIANCE=0
uint8 COVARIANCE_MATRIX_Y_VARIANCE=6
uint8 COVARIANCE_MATRIX_Z_VARIANCE=11
uint8 COVARIANCE_MATRIX_ROLL_VARIANCE=15
uint8 COVARIANCE_MATRIX_PITCH_VARIANCE=18
uint8 COVARIANCE_MATRIX_YAW_VARIANCE=20
uint8 COVARIANCE_MATRIX_VX_VARIANCE=0
uint8 COVARIANCE_MATRIX_VY_VARIANCE=6
uint8 COVARIANCE_MATRIX_VZ_VARIANCE=11
uint8 COVARIANCE_MATRIX_ROLLRATE_VARIANCE=15
uint8 COVARIANCE_MATRIX_PITCHRATE_VARIANCE=18
uint8 COVARIANCE_MATRIX_YAWRATE_VARIANCE=20

# Position and linear velocity frame of reference constants
uint8 LOCAL_FRAME_NED=0         # NED earth-fixed frame
uint8 LOCAL_FRAME_FRD=1         # FRD earth-fixed frame, arbitrary heading reference
uint8 LOCAL_FRAME_OTHER=2       # Not aligned with the std frames of reference
uint8 BODY_FRAME_FRD=3          # FRD body-fixed frame

# Position and linear velocity local frame of reference
uint8 local_frame

# Position in meters. Frame of reference defined by local_frame. NaN if invalid/unknown
float32 x			# North position
float32 y			# East position
float32 z			# Down position

# Orientation quaternion. First value NaN if invalid/unknown
float32[4] q			# Quaternion rotation from FRD body frame to refernce frame
float32[4] q_offset		# Quaternion rotation from odometry reference frame to navigation frame

# Row-major representation of 6x6 pose cross-covariance matrix URT.
# NED earth-fixed frame.
# Order: x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
# If position covariance invalid/unknown, first cell is NaN
# If orientation covariance invalid/unknown, 16th cell is NaN
float32[21] pose_covariance

# Reference frame of the velocity data
uint8 velocity_frame

# Velocity in meters/sec. Frame of reference defined by velocity_frame variable. NaN if invalid/unknown
float32 vx 			# North velocity
float32 vy			# East velocity
float32 vz			# Down velocity

# Angular rate in body-fixed frame (rad/s). NaN if invalid/unknown
float32 rollspeed		# Angular velocity about X body axis
float32 pitchspeed		# Angular velocity about Y body axis
float32 yawspeed		# Angular velocity about Z body axis

# Row-major representation of 6x6 velocity cross-covariance matrix URT.
# Linear velocity in NED earth-fixed frame. Angular velocity in body-fixed frame.
# Order: vx, vy, vz, rotation rate about X axis, rotation rate about Y axis, rotation rate about Z axis
# If linear velocity covariance invalid/unknown, first cell is NaN
# If angular velocity covariance invalid/unknown, 16th cell is NaN
float32[21] velocity_covariance

# TOPICS vehicle_odometry vehicle_mocap_odometry vehicle_visual_odometry
# TOPICS estimator_odometry estimator_visual_odometry_aligned
