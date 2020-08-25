uint64 timestamp		# time since system start (microseconds)

float32 roll_body		# body angle in NED frame (can be NaN for FW)
float32 pitch_body		# body angle in NED frame (can be NaN for FW)
float32 yaw_body		# body angle in NED frame (can be NaN for FW)

float32 yaw_sp_move_rate	# rad/s (commanded by user)

# For quaternion-based attitude control
float32[4] q_d			# Desired quaternion for quaternion control

# For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.
# For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.
float32[3] thrust_body		# Normalized thrust command in body NED frame [-1,1]

bool roll_reset_integral	# Reset roll integral part (navigation logic change)
bool pitch_reset_integral	# Reset pitch integral part (navigation logic change)
bool yaw_reset_integral	# Reset yaw integral part (navigation logic change)

bool fw_control_yaw		# control heading with rudder (used for auto takeoff on runway)

uint8 apply_flaps       	# flap config specifier
uint8 FLAPS_OFF = 0     	# no flaps
uint8 FLAPS_LAND = 1    	# landing config flaps
uint8 FLAPS_TAKEOFF = 2 	# take-off config flaps

# TOPICS vehicle_attitude_setpoint mc_virtual_attitude_setpoint fw_virtual_attitude_setpoint
